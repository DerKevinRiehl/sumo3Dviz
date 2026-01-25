import os
import warnings
import sumolib
import pandas as pd
import numpy as np
import pandera.pandas as pa
import xml.etree.ElementTree as ET
from panda3d.core import Filename, NodePath
from direct.showbase.ShowBase import ShowBase
from typing import cast, Tuple, Union
from pandera.typing import DataFrame, Series

from .trajectory_tools import (
    TrajectoryTools,
    TrajectoryDFSchema,
    SmoothenedTrajectoryDFSchema,
)


class TrafficLightBasicDFSchema(pa.DataFrameModel):
    time: Series[float]
    state: Series[str]


class TrafficLightDFSchema(pa.DataFrameModel):
    time: Series[float]
    state: Series[str]
    timer: Series[float]


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
        video_start_idx = int(
            (df_ego_smoothed["time"] - int(simtime_start)).abs().idxmin()
        )
        video_end_idx = int((df_ego_smoothed["time"] - int(simtime_end)).abs().idxmin())

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
            video_start_idx,
            video_end_idx,
        )

    @pa.check_types
    def load_traffic_light_signals(
        self,
        traffic_signal_states_file: Union[str, None],
        start_time: float,
        end_time: float,
        traffic_light_id: str,
        video_fps: float,
    ) -> Union[DataFrame[TrafficLightDFSchema], None]:
        """
        # TODO: add docstring
        """

        # check if the file exists, return early otherwise
        if traffic_signal_states_file is None:
            return None

        if not os.path.exists(traffic_signal_states_file):
            warnings.warn(
                f"Traffic signal states file {traffic_signal_states_file} does not exist."
            )
            return None

        # load the traffic signal states from the SUMO simulation log
        print("Loading traffic light signals...")
        tree = ET.parse(traffic_signal_states_file)
        root = tree.getroot()

        # extract the traffic light state changes for the specified traffic light
        records = []
        for ts in root.findall("tlsState"):
            if ts.get("id") == traffic_light_id:
                # extract the current timestamp
                if (
                    "time" not in ts.attrib
                    or ts.get("time") is None
                    or ts.get("state") is None
                ):
                    raise ValueError(
                        "SUMO log entry missing 'time' or 'state' attribute."
                    )

                time = float(cast(str, ts.get("time")))
                state = ts.get("state")
                records.append({"time": time, "state": state})

        # store the time series data in a pandas DataFrame
        df_signals_log = pd.DataFrame(records, columns=["time", "state"])
        TrafficLightBasicDFSchema.validate(df_signals_log)

        # convert the traffic light state time series to specified video fps rate
        df_signals_log = self._convert_to_fps_rate(
            df=cast(DataFrame[TrafficLightBasicDFSchema], df_signals_log),
            video_fps=video_fps,
        )

        # shorten the data series to the specified start and end times
        df_signals_log = df_signals_log[df_signals_log["time"] <= end_time]
        df_signals_log = df_signals_log[df_signals_log["time"] >= start_time]

        # add yellow phases between red-green transitions and map the state to descriptive strings
        df_signals_log = self._add_yellow_transition(df_signals_log=df_signals_log)
        df_signals_log["state"] = df_signals_log["state"].replace(
            {"G": "green", "y": "yellow", "y2": "yellow2", "r": "red"}
        )

        # compute countdowns between current time and next green phase
        df_signals_log = self._add_countdown_timer(df_signals_log=df_signals_log)

        # validate that the dataframe is consistent with the expected schema
        df_signals_log = TrafficLightDFSchema.validate(df_signals_log)

        print("Traffic light signals loaded ✓")
        return df_signals_log

    def load_tree_positions(
        self, xml_file: Union[str, None]
    ) -> Union[list[list[float]], None]:
        """
        # TODO: add docstring
        """

        # check if the file exists, return early otherwise
        if xml_file is None:
            return None

        if not os.path.exists(xml_file):
            warnings.warn(f"Tree positions file {xml_file} does not exist.")
            return None

        # load the tree positions from the SUMO map file
        print("Loading tree positions...")
        tree_pois = sumolib.xml.parse(xml_file, "poi")
        tree_positions: list[list[float]] = []
        for poi in tree_pois:
            x = float(poi.x)
            y = float(poi.y)
            tree_positions.append([x, y])

        print("Tree positions loaded ✓")
        return tree_positions

    def load_fence_lines(
        self, xml_file: Union[str, None]
    ) -> Union[list[list[list[float]]], None]:
        """
        # TODO: add docstring
        """

        # check if the file exists, return early otherwise
        if xml_file is None:
            return None

        if not os.path.exists(xml_file):
            warnings.warn(f"Fence lines file {xml_file} does not exist.")
            return None

        # load the fence lines from the SUMO map file
        print("Loading fence lines...")
        polys = sumolib.xml.parse(xml_file, "poly")
        poly_lines: list[list[list[float]]] = []
        for poly in polys:
            # poly.shape is a string like "x1,y1 x2,y2 ..."
            points: list[list[float]] = []
            for point in poly.shape.split():
                x, y = map(float, point.split(","))
                points.append([x, y])

            poly_lines.append(points)

        print("Fence lines loaded ✓")
        return poly_lines

    def load_shop_positions(
        self, xml_file: Union[str, None]
    ) -> Union[list[list[float]], None]:
        """
        # TODO: add docstring
        """

        # check if the file exists, return early otherwise
        if xml_file is None:
            return None

        if not os.path.exists(xml_file):
            warnings.warn(f"Shop positions file {xml_file} does not exist.")
            return None

        # load the shop positions from the SUMO map file
        print("Loading shop positions...")
        shop_pois = sumolib.xml.parse(xml_file, "poi")
        shop_positions: list[list[float]] = []
        for poi in shop_pois:
            x = float(poi.x)
            y = float(poi.y)
            shop_positions.append([x, y])

        print("Shop positions loaded ✓")
        return shop_positions

    def load_car_models(self, context: ShowBase, low_poly_cars_file: str):
        """
        # TODO: add docstring
        """
        if not os.path.exists(low_poly_cars_file):
            raise ValueError(f"Car models file {low_poly_cars_file} does not exist.")

        if context.loader is None:
            raise ValueError("Panda3D context loader is not initialized.")

        print("Loading car models...")
        car_collection: NodePath = context.loader.loadModel(low_poly_cars_file)
        car_models = [car_collection.find("**/" + str(n)) for n in range(1, 10 + 1)]
        print("Car models loaded ✓")
        return car_models

    def load_ego_car_model(
        self,
        context: ShowBase,
        car_file: str,
    ):
        """
        # TODO: add docstring
        """
        if not os.path.exists(car_file):
            raise ValueError(f"Ego car model file {car_file} does not exist.")

        if context.loader is None:
            raise ValueError("Panda3D context loader is not initialized.")

        print("Loading ego car model...")
        ego_car: NodePath = context.loader.loadModel(car_file)
        ego_car.reparentTo(context.render)
        print("Ego car model loaded ✓")
        return ego_car

    @pa.check_types
    def _convert_to_fps_rate(
        self, df: DataFrame[TrafficLightBasicDFSchema], video_fps: float
    ) -> DataFrame[TrafficLightBasicDFSchema]:
        """
        # TODO: add docstring
        """

        # type validation (as state cannot be validated correctly with pandera)
        df["time"] = df["time"].astype(float)

        # set the time as index
        df = df.set_index("time")

        # create new time index at desired fps rate
        start_time = df.index.min()
        end_time = df.index.max()
        new_time_index = np.arange(start_time, end_time + 1 / video_fps, 1 / video_fps)

        # reindex and forward-fill
        df_fps = df.reindex(new_time_index, method="ffill")
        df_fps = df_fps.reset_index().rename(columns={"index": "time"})
        return df_fps

    @pa.check_types
    def _add_yellow_transition(
        self, df_signals_log: DataFrame[TrafficLightBasicDFSchema]
    ) -> DataFrame[TrafficLightBasicDFSchema]:
        """
        # TODO: add docstring
        """

        # find the indices where the state changes from red to green or vice versa
        df = df_signals_log.copy()
        red_to_green_idx = df.index[
            (df["state"].shift(1) == "r") & (df["state"] == "G")
        ]
        green_to_red_idx = df.index[
            (df["state"].shift(1) == "G") & (df["state"] == "r")
        ]

        # add 3 second yellow state within red-green transitions
        for idx in red_to_green_idx:
            transition_time = df.loc[idx, "time"]
            mask_before = (
                (df["state"] == "r")
                & (df["time"] >= transition_time - 3)
                & (df["time"] < transition_time)
            )
            df.loc[mask_before, "state"] = "y"

        # add 3 second yellow2 state within green-red transitions
        for idx in green_to_red_idx:
            transition_time = df.loc[idx, "time"]
            mask_before = (
                (df["state"] == "G")
                & (df["time"] >= transition_time - 3)
                & (df["time"] < transition_time)
            )
            df.loc[mask_before, "state"] = "y2"

        return df

    @pa.check_types
    def _add_countdown_timer(
        self, df_signals_log: DataFrame[TrafficLightBasicDFSchema]
    ) -> DataFrame[TrafficLightDFSchema]:
        """
        # TODO: add docstring
        """

        # for each row, find the next green index (or np.nan if none)
        df = df_signals_log.copy()
        next_green = np.full(len(df), np.nan)
        last_green = None

        # traverse backwards to fill the next green time for each row
        for i in reversed(range(len(df))):
            if df.iloc[i]["state"] == "green":
                last_green = df.iloc[i]["time"]
            next_green[i] = last_green if last_green is not None else np.nan

        # calculate countdown: time until next green, or 0 if already green
        df["timer"] = np.where(
            (df["state"] == "green") | (df["state"] == "yellow2"),
            0,
            next_green - df["time"],
        )

        # validate that the dataframe is consistent with the expected schema
        df = TrafficLightDFSchema.validate(df)

        return df
