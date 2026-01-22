import pandera.pandas as pa
import xml.etree.ElementTree as ET
from typing import cast, Tuple, Union
from pandera.typing import DataFrame

from .trajectory_tools import (
    TrajectoryTools,
    TrajectoryDFSchema,
    SmoothenedTrajectoryDFSchema,
)


class LoaderTools:
    # TODO: add docstring

    @pa.check_types
    def load_trajectory(
        self,
        trajectory_file: str,
        ego_identifier: str,
        simtime_start: float,
        simtime_end: float,
        video_fps: float = 25,
        show_other_vehicles: bool = True,
    ) -> Tuple[
        DataFrame[SmoothenedTrajectoryDFSchema],
        Union[list[DataFrame[SmoothenedTrajectoryDFSchema]], None],
        int,
        int,
    ]:
        """
        # TODO: add docstring
        """
        # load the trajectory data from the SUMO simulation log
        tree = ET.parse(trajectory_file)
        root = tree.getroot()
        rows = []

        for entry in root.findall("timestep"):
            # extract the current timestamp
            if "time" not in entry.attrib:
                raise ValueError("SUMO log timestamp entry missing 'time' attribute.")

            t = float(cast(str, entry.get("time")))

            # extract the vehicle position and orientation for all vehicles at this timestamp
            for veh in entry.findall("vehicle"):
                if (
                    "id" not in veh.attrib
                    or "x" not in veh.attrib
                    or "y" not in veh.attrib
                    or "angle" not in veh.attrib
                ):
                    raise ValueError(
                        "SUMO log vehicle entry missing one of the required attributes: 'id', 'x', 'y', 'angle'."
                    )

                rows.append(
                    {
                        "time": float(t),
                        "veh_id": veh.get("id"),
                        "pos_x": float(cast(str, veh.get("x"))),
                        "pos_y": float(cast(str, veh.get("y"))),
                        "angle": float(cast(str, veh.get("angle"))),
                    }
                )

        # wrap the data into a pandas DataFrame for easier processing
        df_simulation_log_cars = DataFrame[TrajectoryDFSchema](
            rows, columns=["time", "veh_id", "pos_x", "pos_y", "angle"]
        )

        # load raw trajectory of ego vehicle in preparation for processing
        trajectory_tools = TrajectoryTools()
        (
            df_ego_trajectory,
            first_timestamp,
            last_timestamp,
        ) = trajectory_tools.get_vehicle_trajectory_raw(
            df_simulation_log_cars, ego_identifier
        )

        # normalize the angles to be within [-180, 180] degrees
        df_ego_trajectory["angle"] = [
            trajectory_tools.normalize_angle(angle)
            for angle in df_ego_trajectory["angle"]
        ]

        # smoothen the ego vehicle trajectory for better visualization
        df_ego_smoothed = trajectory_tools.interpolate_trajectory(
            df_ego_trajectory, video_fps
        )

        # if the trajectory smoothing failed, abort the loading process
        if df_ego_smoothed is None:
            raise ValueError(
                "Could not smoothen the ego vehicle trajectory, aborting process."
            )

        # reset the time to the current index
        df_ego_smoothed = df_ego_smoothed.reset_index()
        df_ego_smoothed = df_ego_smoothed.drop(columns=["time"], errors="ignore")
        df_ego_smoothed = df_ego_smoothed.rename(columns={"index": "time"})

        # obtain the index values for the selected start and end times of the video
        video_current_point = int(
            (df_ego_smoothed["time"] - int(simtime_start)).abs().idxmin()
        )
        video_final_point = int(
            (df_ego_smoothed["time"] - int(simtime_end)).abs().idxmin()
        )

        # compute the smoothened vehicle trajectories for all other vehicles in the scene
        if show_other_vehicles:
            smoothened_trajectory_data = trajectory_tools.load_smoothened_trajectories(
                ego_identifier=ego_identifier,
                df_simulation_log_cars=df_simulation_log_cars,
                first_timestamp=first_timestamp,
                last_timestamp=last_timestamp,
                video_fps=video_fps,
            )
        else:
            smoothened_trajectory_data = None

        # return the ego vehicle's smoothened trajectory along with all other vehicles' trajectories
        return (
            df_ego_smoothed,
            smoothened_trajectory_data,
            video_current_point,
            video_final_point,
        )
