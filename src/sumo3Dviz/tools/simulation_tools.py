"""sumo3Dviz: A three-dimensional traffic visualisation [2026]
Authors: Kevin Riehl <kriehl@ethz.ch>, Julius Schlapbach <juliussc@ethz.ch>
Organisation: ETH Zürich, Institute for Transport Planning and Systems (IVT)
"""

import cv2
import math
import numpy as np
import pandas as pd
from typing import cast, Optional, Dict, Any, Union, Literal
from direct.showbase.ShowBase import ShowBase
from panda3d.core import NodePath, Camera, GraphicsOutput

from .trajectory_tools import TrajectoryTools


class SimulationManager:
    """
    Manages the simulation loop for rendering SUMO traffic simulations.
    Encapsulates all state variables and update logic for the Panda3D task manager.
    """

    def __init__(
        self,
        configuration: Dict,
        context: ShowBase,
        trajectory_data: Dict,
        car_instances: Dict,
        rendering_tools: Any,
        mode: Literal["lagrangian", "eulerian", "cinematic"],
        video_writer: Optional[cv2.VideoWriter] = None,
        show_other_vehicles_simple: bool = False,
        signal_instances: Optional[list[dict]] = None,
        camera_position: Optional[dict] = None,
        cinematic_camera_trajectory: Union[pd.DataFrame, None] = None,
    ):
        """
        Initialize the SimulationManager with all required parameters.

        Args:
            context: The Panda3D ShowBase context
            configuration: Dict of configuration
            trajectory_data: Trajectory Data
            car_instances: Dict
            rendering_tools: Instance of RenderingTools for traffic light updates
            mode: The visualization mode ('lagrangian', 'eulerian', or 'cinematic')
            video_writer: OpenCV VideoWriter (optional)
            show_other_vehicles_simple: Whether to render other vehicles as box or car 3D object (optional)
            signal_instances: List of TrafficLightSchema objects (optional)
            camera_position: Eulerian camera position (optional)
            cinematic_camera_trajectory: Cinematic camera trajectory (optional)
        """
        self.context = context
        self.configuration = configuration
        self.trajectory_data = trajectory_data
        self.car_instances = car_instances
        self.signal_instances = signal_instances
        self.video_writer = video_writer
        self.trajectory_tools = TrajectoryTools()
        self.rendering_tools = rendering_tools

        # trajectory data details
        self.trajectory_points = self.trajectory_data["ego_trajectory"][
            ["veh_id", "pos_x", "pos_y", "computed_angle_deg", "time"]
        ].values

        # extract traffic light signal state data for all traffic lights
        self.traffic_light_ids = list(self.trajectory_data["signal_states"].keys())
        self.signal_points = {}
        for traffic_light_id in self.traffic_light_ids:
            self.signal_points[traffic_light_id] = self.trajectory_data[
                "signal_states"
            ][traffic_light_id][["time", "state", "timer"]].values

        # state variables
        self.current_point = self.trajectory_data["video_start_idx"]
        self.screenshot_counter = 0
        self.others_car_instances: Dict[int, NodePath] = {}

        # other settings
        self.show_other_vehicles_simple = show_other_vehicles_simple

        # mode and position specific
        self.mode = mode.upper()
        self.camera_position = camera_position
        self.cinematic_camera_trajectory = cinematic_camera_trajectory
        print("[sumo3Dviz mode]", self.mode)

    def _update_camera(self, x, y, angle, current_time):
        if self.mode == "LAGRANGIAN":
            cast(Camera, self.context.camera).setPos(
                x, y, 2.12
            )  # adjusted camera height for car z position in Lagrangian mode
            cast(Camera, self.context.camera).setHpr(-angle, 0, 0)
        elif self.mode == "EULERIAN":
            if self.camera_position is None:
                raise ValueError("Camera position data is required for Eulerian mode")

            cast(Camera, self.context.camera).setPos(
                self.camera_position["pos_x"],
                self.camera_position["pos_y"],
                self.camera_position["pos_z"],
            )
            cast(Camera, self.context.camera).setHpr(
                self.camera_position["ori_h"],
                self.camera_position["ori_p"],
                self.camera_position["ori_r"],
            )
        elif self.mode == "CINEMATIC":
            if self.cinematic_camera_trajectory is None:
                raise ValueError(
                    "Cinematic camera trajectory data is required for cinematic mode"
                )

            closest_idx = (
                (self.cinematic_camera_trajectory["time"] - current_time).abs().idxmin()
            )
            row = self.cinematic_camera_trajectory.iloc[closest_idx]
            cast(Camera, self.context.camera).setPos(
                row["pos_x"], row["pos_y"], row["pos_z"]
            )
            # normalize angles to [0, 360) range for Panda3D
            cast(Camera, self.context.camera).setHpr(
                row["ori_h"] % 360, row["ori_p"] % 360, row["ori_r"] % 360
            )

    def _get_signal_state(self, traffic_light_id: str):
        if self.signal_points is not None:
            if traffic_light_id not in self.signal_points:
                raise ValueError(
                    f"Traffic light ID {traffic_light_id} not found in signal points"
                )

            _, signal, timer = self.signal_points[traffic_light_id][self.current_point]
        else:
            signal, timer = None, 0
        return signal, timer

    def _update_ego_car_position(self, x, y, angle, turn_vehicle=False):
        distance = 1.6  # how far in front you want the car to be
        car_x = x + distance * math.cos(math.radians(90 - angle))
        car_y = y + distance * math.sin(math.radians(90 - angle))
        car_z = (
            0.12  # slightly above road surface to avoid z-fighting (road is at z=0.01)
        )
        self.car_instances["ego_car"].setPos(car_x, car_y, car_z)
        self.car_instances["ego_car"].setHpr(
            (180 - angle) % 360 if turn_vehicle is False else -angle, 90, 0
        )

    def _update_traffic_lights(self):
        if self.signal_instances is None:
            return

        for signal in self.signal_instances:
            signal_state, timer = self._get_signal_state(traffic_light_id=signal["id"])
            if (
                self.configuration["visualization"]["show_signals"]
                and signal is not None
            ):
                if self.signal_instances is None:
                    raise ValueError(
                        "Signal instances are required for traffic light visualization"
                    )

                self.rendering_tools.update_traffic_light(
                    signal_state=signal_state,
                    # prefer per-signal override stored in the signal instance
                    num_heads=signal.get(
                        "num_heads", self.configuration["signals"].get("num_heads", 3)
                    ),
                    countdown_timer=signal.get(
                        "countdown_timer",
                        self.configuration["signals"].get("countdown_timer", False),
                    ),
                    timer=timer,
                    box_node1=signal["box_node1"],
                    box_node2=signal["box_node2"],
                    box_node3=signal["box_node3"],
                    text_node=signal["text_node"],
                )

    def _update_car_positions(self, x, y, current_time):
        # set other cars position
        if (
            self.configuration["visualization"]["show_other_vehicles"]
            and self.trajectory_data["trajectory_data"] is not None
        ):
            # position all cars on the road
            current_pos = [x, y]
            neighborhood_vehicles = []
            if self.mode == "EULERIAN" or self.mode == "CINEMATIC":
                neighborhood_vehicles = self.trajectory_tools.get_closest_vehicles(
                    self.trajectory_data["trajectory_data"],
                    current_pos,
                    current_time,
                    max_vehicles=-1,
                )
            elif self.mode == "LAGRANGIAN":
                neighborhood_vehicles = self.trajectory_tools.get_closest_vehicles(
                    self.trajectory_data["trajectory_data"],
                    current_pos,
                    current_time,
                )

            # create new instances
            current_vehicle_ids = []
            for vehicle in neighborhood_vehicles:
                vehicle_id = vehicle[-1]
                current_vehicle_ids.append(vehicle_id)
                if vehicle_id not in self.others_car_instances:
                    if self.show_other_vehicles_simple:
                        # Create simple box-based car
                        new_vehicle_instance = (
                            self.rendering_tools.generate_simple_car_model(self.context)
                        )
                        new_vehicle_instance.reparentTo(self.context.render)
                    else:
                        # Original detailed models
                        available_choices = [i for i in range(1, 11) if i != 2]
                        selected_model = np.random.choice(available_choices)
                        new_vehicle_instance = self.car_instances["car_models"][
                            selected_model - 1
                        ].copyTo(self.context.render)
                        new_vehicle_instance.setScale(5.0)
                    self.others_car_instances[vehicle_id] = new_vehicle_instance

            # delete unused instances
            ids_to_delete = []
            for vehicle in self.others_car_instances:
                if vehicle not in current_vehicle_ids:
                    ids_to_delete.append(vehicle)
            for ids in ids_to_delete:
                self.others_car_instances[ids].removeNode()  # remove from app
                del self.others_car_instances[ids]

            # move instances
            for vehicle in neighborhood_vehicles:
                car_instance = self.others_car_instances[vehicle[-1]]
                car_instance.setPos(
                    vehicle[0], vehicle[1], 0.05
                )  # slightly above road surface (road is at z=0.01)
                car_instance.setHpr(180 - vehicle[2], 90, 0)

    def _record_video_frame(self):
        if self.video_writer is not None:
            self.context.graphicsEngine.renderFrame()
            tex = cast(GraphicsOutput, self.context.win).getScreenshot()
            data = tex.getRamImageAs("BGRA")
            img_array = np.frombuffer(data, np.uint8)
            img_array = img_array.reshape(
                (
                    int(self.configuration["rendering"]["video_height_px"]),
                    int(self.configuration["rendering"]["video_width_px"]),
                    4,
                )
            )
            img_array = cv2.rotate(img_array, cv2.ROTATE_180)
            img_array = cv2.flip(img_array, 1)
            img = img_array[:, :, :3]
            self.video_writer.write(img)
            self.screenshot_counter += 1

    def _log_to_console(self, current_time):
        print(
            "t={:.2f}".format(current_time),
            "\t, Frame (",
            self.current_point,
            "/",
            self.trajectory_data["video_end_idx"],
            ")",
        )

    def update_world(self, task):
        """
        Update function called by Panda3D task manager each frame.
        Updates camera, ego vehicle, other vehicles, traffic lights, and records video.

        Args:
            task: Panda3D task object

        Returns:
            task.again to continue or task.done to stop
        """
        if self.current_point < len(self.trajectory_points):
            _, x, y, angle, current_time = self.trajectory_points[self.current_point]

            # update world scene
            self._update_camera(x, y, angle, current_time)
            self._update_ego_car_position(
                x, y, angle, turn_vehicle=(self.mode == "LAGRANGIAN")
            )
            self._update_traffic_lights()
            self._update_car_positions(x, y, current_time)
            self.current_point += 2

            # logging to console
            self._log_to_console(current_time)

            # record video frame
            self._record_video_frame()

            # exit criterion
            if self.current_point > self.trajectory_data["video_end_idx"]:
                if self.video_writer is not None:
                    self.video_writer.release()
                self.context.userExit()
                return task.done  # stop task when trajectory is complete
            return task.again  # run again after interval
        else:
            if self.video_writer is not None:
                self.video_writer.release()
            self.context.userExit()
            return task.done  # stop task when trajectory is complete
