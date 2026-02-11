"""sumo3Dviz: A three-dimensional traffic visualisation [2026]
Authors: Kevin Riehl <kriehl@ethz.ch>, Julius Schlapbach <jschlapbach@ethz.ch>
Organisation: ETH Zürich, Institute for Transport Planning and Systems (IVT)
"""

import warnings
from tqdm import tqdm
import numpy as np
import pandas as pd
import pandera.pandas as pa
from typing import cast, Union
from pandera.typing import DataFrame, Series
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import Optional, Tuple


class TrajectoryDFSchema(pa.DataFrameModel):
    time: Series[float]
    veh_id: Series[str]
    pos_x: Series[float]
    pos_y: Series[float]
    angle: Series[float]


class NumericTrajectoryDFSchema(pa.DataFrameModel):
    pos_x: Series[float]
    pos_y: Series[float]
    angle: Series[float]


class SmoothenedTrajectoryDFSchema(pa.DataFrameModel):
    time: Series[float]
    pos_x: Series[float]
    pos_y: Series[float]
    computed_angle_deg: Series[float]
    angle: Series[float]


class TrajectoryTools:
    """Tools for processing and smoothing vehicle trajectories from SUMO simulations."""

    @pa.check_types
    def get_vehicle_trajectory_raw(
        self,
        df_simulation_log_cars: DataFrame[TrajectoryDFSchema],
        ego_identifier: str,
    ) -> DataFrame[TrajectoryDFSchema]:
        """Extract the raw trajectory for a specific vehicle from simulation data.

        Args:
            df_simulation_log_cars (DataFrame[TrajectoryDFSchema]): Complete simulation
                log containing trajectories for all vehicles with columns: time, veh_id,
                pos_x, pos_y, angle.
            ego_identifier (str): Vehicle ID to extract trajectory for.

        Returns:
            DataFrame with the vehicle's trajectory
        """
        # extract the trajectory for the ego vehicle
        df_trajectory = TrajectoryDFSchema.validate(
            cast(
                pd.DataFrame,
                df_simulation_log_cars[
                    df_simulation_log_cars["veh_id"] == ego_identifier
                ],
            )
        )

        return df_trajectory

    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to be within [0, 360] degrees.

        Args:
            angle (float): Angle in degrees to normalize.

        Returns:
            float: Normalized angle in degrees within the range [0, 360].
        """
        return (angle + 360) % 360

    @pa.check_types
    def interpolate_trajectory(
        self, df_trajectory: DataFrame[TrajectoryDFSchema], video_fps: float
    ) -> Union[DataFrame[SmoothenedTrajectoryDFSchema], None]:
        """Interpolate and smooth a vehicle trajectory to match video frame rate.

        Resamples the trajectory to the specified FPS, applies linear interpolation
        for missing values, smooths position and angle data using rolling means,
        and computes movement direction based on position changes.

        Args:
            df_trajectory (DataFrame[TrajectoryDFSchema]): Raw vehicle trajectory
                with columns: time, veh_id, pos_x, pos_y, angle.
            video_fps (float): Target video frames per second for resampling.

        Returns:
            DataFrame[SmoothenedTrajectoryDFSchema] | None: Smoothed trajectory with
                columns: time, pos_x, pos_y, computed_angle_deg, angle. Returns None
                if the trajectory has less than 2 data points.
        """
        # set the time column as index for interpolation
        df_trajectory = df_trajectory.set_index("time")
        t_min = df_trajectory.index.min()
        t_max = df_trajectory.index.max()

        # unwrap angles to handle 0/360 wraparound before interpolation
        angle_unwrapped = np.rad2deg(np.unwrap(np.deg2rad(df_trajectory["angle"])))
        df_trajectory["angle"] = angle_unwrapped

        # resample the trajectory to the desired video fps and interpolate missing values linearly
        desired_timestamps = np.arange(t_min, t_max, 1 / video_fps)
        df_resampled = df_trajectory.reindex(
            df_trajectory.index.union(pd.Index(desired_timestamps))
        ).sort_index()

        # linearly interpolate the values in numerical columns at the desired timestamps
        df_interpolated = (
            df_resampled[["pos_x", "pos_y", "angle"]]
            .interpolate(method="linear")
            .loc[desired_timestamps]
        )

        # apply smoothing to the numerical columns using a rolling mean
        df_numeric = NumericTrajectoryDFSchema.validate(
            cast(pd.DataFrame, df_interpolated[["pos_x", "pos_y", "angle"]])
        )
        window_size = 21
        df_smoothed = df_numeric.rolling(
            window=window_size, center=True, min_periods=1
        ).mean()

        # extract the times from the index as explicit columns
        df_smoothed["time"] = df_smoothed.index

        # compute the linear speed based on the position changes
        df_smoothed["dx"] = df_smoothed["pos_x"].diff()
        df_smoothed["dy"] = df_smoothed["pos_y"].diff()
        df_smoothed["step_distance"] = np.sqrt(
            df_smoothed["dx"] ** 2 + df_smoothed["dy"] ** 2
        )

        # drop the first line, since it contains NaN values due to the diff() operation
        try:
            df_smoothed = df_smoothed.drop(df_smoothed.index[0])
        except Exception:
            # warnings.warn(
            #     "Trajectory too short to interpolate (less than 2 data points). Skipping trajectory."
            # )
            return None

        # compute the movement direction (rad and deg) based on the position changes
        df_smoothed["computed_angle_rad"] = np.arctan2(
            df_smoothed["dy"], df_smoothed["dx"]
        )
        df_smoothed["computed_angle_deg"] = np.degrees(
            df_smoothed["computed_angle_rad"]
        )
        df_smoothed["computed_angle_deg"] = 90 - df_smoothed["computed_angle_deg"]

        # if some values could not be computed, fill them with the original angle values
        df_smoothed["computed_angle_deg"] = df_smoothed["computed_angle_deg"].fillna(
            df_smoothed["angle"]
        )

        # reset the computed angles with the original values where the vehicle is not moving enough
        # -> the ratio between dx and dy becomes unstable from the numerical simulation
        min_step_distance = 0.03
        valid_mask = df_smoothed["step_distance"] > min_step_distance
        first_valid_idx = valid_mask.idxmax() if valid_mask.any() else None

        if first_valid_idx is not None:
            mask = (df_smoothed.index >= first_valid_idx) & (~valid_mask)
            df_smoothed["computed_angle_deg"] = np.where(
                mask,  # (df_smoothed.index == 0) | (df_smoothed.index == 1) | (df_smoothed['step_distance'] > min_step_distance),
                np.nan,
                df_smoothed["computed_angle_deg"],
            )
            df_smoothed["computed_angle_deg"] = df_smoothed[
                "computed_angle_deg"
            ].ffill()

        # smoothen the computed angles again to reduce jitter with a window size of 41
        window_size_angle = 41
        df_smoothed["computed_angle_deg"] = (
            df_smoothed["computed_angle_deg"]
            .rolling(window=window_size_angle, center=True, min_periods=1)
            .mean()
        )

        # select the relevant columns and validate the final DataFrame schema
        df_smoothed = SmoothenedTrajectoryDFSchema.validate(
            cast(
                pd.DataFrame,
                df_smoothed[["time", "pos_x", "pos_y", "computed_angle_deg", "angle"]],
            )
        )
        df_smoothed["time"] = df_smoothed["time"].round(3)

        return df_smoothed

    def generate_camera_cinematic_trajectory(
        self,
        camera_position_trajectory: dict,
        simtime_start: float,
        simtime_end: float,
        video_fps: float,
    ):
        """
        Generate a smooth camera trajectory from keyframes.

        Args:
            camera_position_trajectory (dict): keyframes as {time: {"pos_x":..,"ori_h":..,...}}
            simtime_start (float): start time
            simtime_end (float): end time
            video_fps (float): frames per second

        Returns:
            pd.DataFrame: smoothed camera trajectory with columns ["time", "pos_x", "pos_y", "pos_z", "ori_h", "ori_p", "ori_r"]
        """

        # 1. Prepare the time array
        dt = 1.0 / video_fps
        times = np.arange(simtime_start, simtime_end + dt, dt)

        # 2. Extract keyframe times and sort
        keyframe_times = np.array(sorted(camera_position_trajectory.keys()))

        # 3. Prepare arrays for interpolation
        keys = ["pos_x", "pos_y", "pos_z", "ori_h", "ori_p", "ori_r"]
        interp_data = {}

        for k in keys:
            # Extract keyframe values
            values = np.array(
                [camera_position_trajectory[t][k] for t in keyframe_times], dtype=float
            )

            if k.startswith("ori_"):  # handle angular wrapping
                # Convert to radians and unwrap
                values_rad = np.deg2rad(values)
                values_rad = np.unwrap(
                    values_rad
                )  # unwrap ensures smooth interpolation
                # Interpolate
                interp_values_rad = np.interp(times, keyframe_times, values_rad)
                # Convert back to degrees
                interp_data[k] = np.rad2deg(interp_values_rad) % 360
            else:
                # Linear interpolation with out-of-bounds clamping
                interp_values = np.interp(
                    times,
                    keyframe_times,
                    values,
                    left=values[0],  # stable before first keyframe
                    right=values[-1],  # stable after last keyframe
                )
                interp_data[k] = interp_values

        # 4. Apply centered moving average smoothing (window=41)
        window_size = 41

        def moving_average(arr, window):
            return np.convolve(arr, np.ones(window) / window, mode="same")

        smoothed_data = {k: moving_average(interp_data[k], window_size) for k in keys}

        # 5. Build the DataFrame
        df_camera_trajectory_smoothed = pd.DataFrame({"time": times, **smoothed_data})

        return df_camera_trajectory_smoothed

    @pa.check_types
    def load_smoothened_trajectories(
        self,
        ego_identifier: str,
        df_simulation_log_cars: DataFrame[TrajectoryDFSchema],
        video_fps: float,
    ) -> list[DataFrame[SmoothenedTrajectoryDFSchema]]:

        grouped_trajectories = df_simulation_log_cars.groupby("veh_id")
        # materialize groups to a list so we can iterate multiple times
        groups = [(veh_id, group) for veh_id, group in grouped_trajectories]
        num_vehicles = len(groups)

        def process_one(item: Tuple[str, pd.DataFrame]) -> Optional[DataFrame]:
            veh_id, df_trajectory = item

            if veh_id == ego_identifier:
                return None
            if len(df_trajectory) == 0:
                return None

            df_trajectory_valid = TrajectoryDFSchema.validate(
                cast(pd.DataFrame, df_trajectory)
            )

            df_smoothed = self.interpolate_trajectory(
                df_trajectory=df_trajectory_valid,
                video_fps=video_fps,
            )
            if df_smoothed is None:
                return None

            df_smoothed["angle"] = [
                self.normalize_angle(angle=angle) for angle in df_smoothed["angle"]
            ]
            df_smoothed["computed_angle_deg"] = [
                self.normalize_angle(angle=angle)
                for angle in df_smoothed["computed_angle_deg"]
            ]

            return df_smoothed

        smoothened_trajectory_data: list[DataFrame[SmoothenedTrajectoryDFSchema]] = []

        with ThreadPoolExecutor(max_workers=5) as executor:
            futures = [executor.submit(process_one, g) for g in groups]

            for f in tqdm(
                as_completed(futures),
                total=num_vehicles,
                desc="Smoothening trajectories (parallel)",
            ):
                res = f.result()
                if res is not None:
                    smoothened_trajectory_data.append(res)

        # validate the correct data structure of the smoothened trajectories
        for df in smoothened_trajectory_data:
            SmoothenedTrajectoryDFSchema.validate(cast(pd.DataFrame, df))

        return smoothened_trajectory_data

    def get_closest_vehicles(
        self,
        smoothened_trajectory_data: list,
        current_pos: list,
        current_time: float,
        max_vehicles: int = 200,
    ) -> list:
        """Find the closest vehicles to a given position at a specific time.

        Searches through smoothed trajectory data to find vehicles present at the
        specified time, computes their distances from the current position, and
        returns the closest ones sorted by distance.

        Args:
            smoothened_trajectory_data (list): List of smoothed trajectory DataFrames
                for all vehicles.
            current_pos (list): Current position as [x, y] coordinates.
            current_time (float): Current simulation time to query vehicle positions.
            max_vehicles (int): Maximum number of closest vehicles to return.
                Defaults to 200. If set to -1, all vehicles are returned.

        Returns:
            list: List of vehicle data for the closest vehicles, where each entry
                is [pos_x, pos_y, angle, veh_id], sorted by distance from current_pos.
        """
        # collect vehicles and their positions at current_time
        vehicle_data = []
        for i, df in enumerate(smoothened_trajectory_data):
            # find rows at current_time with tolerance
            mask = np.isclose(df["time"], current_time, atol=0.01)
            if mask.any():
                row = df[mask].iloc[0]
                # use 'computed_angle_deg' if present, otherwise use 'angle'
                angle = row.get("computed_angle_deg", row.get("angle", np.nan))
                vehicle_data.append(
                    {
                        "veh_id": i,  # vehicle index in trajectory data
                        "pos_x": row["pos_x"],
                        "pos_y": row["pos_y"],
                        "angle": angle,
                    }
                )

        # if no vehicles at current_time, return empty list
        if not vehicle_data:
            return []

        # compute distances
        positions = np.array([[v["pos_x"], v["pos_y"]] for v in vehicle_data])
        current_pos_array = np.array(current_pos)
        distances = np.linalg.norm(positions - current_pos_array, axis=1)

        # add distances to vehicle info
        for i, v in enumerate(vehicle_data):
            v["distance"] = distances[i]

        # sort by distance and select top max_vehicles
        sorted_vehicles = sorted(vehicle_data, key=lambda x: x["distance"])

        # return list of [pos_x, pos_y, angle, veh_id] for closest vehicles
        if max_vehicles == -1:
            selected = sorted_vehicles
        else:
            selected = sorted_vehicles[:max_vehicles]
        return [[v["pos_x"], v["pos_y"], v["angle"], v["veh_id"]] for v in selected]
