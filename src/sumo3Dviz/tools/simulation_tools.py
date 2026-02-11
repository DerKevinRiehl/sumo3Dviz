"""sumo3Dviz: A three-dimensional traffic visualisation [2026]
Authors: Kevin Riehl <kriehl@ethz.ch>, Julius Schlapbach <jschlapbach@ethz.ch>
Organisation: ETH Zürich, Institute for Transport Planning and Systems (IVT)
"""

import numpy as np
import pandas as pd
import math
import cv2
import sys
from typing import cast, Optional, Dict, Any
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
        video_writer: Optional[cv2.VideoWriter] = None,
        show_other_vehicles_simple: bool = False,
        signal_instances: Dict = None,
        camera_position: Optional[dict] = None,
        cinematic_camera_trajectory: pd.DataFrame = None,
    ):
        """
        Initialize the SimulationManager with all required parameters.

        Args:
            context: The Panda3D ShowBase context
            configuration: Dict of configuration
            trajectory_data: Trajectory Data
            car_instances: Dict
            rendering_tools: Instance of RenderingTools for traffic light updates
            video_writer: OpenCV VideoWriter (optional)
            show_other_vehicles_simple: Whether to render other vehicles as box or car 3D object (optional)
            signal_instances: Dict Traffic light nodes (optional)
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
        self.signal_points = (
            self.trajectory_data["signal_states"][["time", "state", "timer"]].values
            if self.trajectory_data["signal_states"] is not None
            else None
        )
        # mode and position specific
        self.camera_position = camera_position
        self.cinematic_camera_trajectory = cinematic_camera_trajectory
        self.mode = "LAGRANGIAN"
        if self.camera_position is not None:
            self.mode = "EULERIAN"
        if self.cinematic_camera_trajectory is not None:
            self.mode = "CINEMATIC"
        # state variables
        self.current_point = self.trajectory_data["video_start_idx"]
        self.screenshot_counter = 0
        self.others_car_instances: Dict[int, NodePath] = {}
        # other settings
        self.show_other_vehicles_simple = show_other_vehicles_simple

    def _update_camera(self, x, y, angle, current_time):
        if self.mode == "LAGRANGIAN":
            cast(Camera, self.context.camera).setPos(
                x, y, self.configuration["visualization"]["viewer_height"]
            )
            cast(Camera, self.context.camera).setHpr(-angle, 0, 0)
        elif self.mode == "EULERIAN":
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
            closest_idx = (
                (self.cinematic_camera_trajectory["time"] - current_time).abs().idxmin()
            )
            row = self.cinematic_camera_trajectory.iloc[closest_idx]
            cast(Camera, self.context.camera).setPos(
                row["pos_x"], row["pos_y"], row["pos_z"]
            )
            cast(Camera, self.context.camera).setHpr(
                row["ori_h"], row["ori_p"], row["ori_r"]
            )

    def _get_signal_state(self):
        if self.signal_points is not None:
            _, signal, timer = self.signal_points[self.current_point]
        else:
            signal, timer = None, 0
        return signal, timer

    def _update_ego_car_position(self, x, y, angle):
        distance = 1.6  # how far in front you want the car to be
        car_x = x + distance * math.cos(math.radians(90 - angle))
        car_y = y + distance * math.sin(math.radians(90 - angle))
        car_z = -0.5
        self.car_instances["ego_car"].setPos(car_x, car_y, car_z)
        self.car_instances["ego_car"].setHpr(-angle, 90, 0)

    def _update_traffic_lights(self):
        signal, timer = self._get_signal_state()
        if self.configuration["visualization"]["show_signals"] and signal is not None:
            self.rendering_tools.update_traffic_light(
                signal=signal,
                design=self.configuration["signals"]["signal_design"],
                timer=timer,
                box_node1=self.signal_instances["box_node1"],
                box_node2=self.signal_instances["box_node2"],
                box_node3=self.signal_instances["box_node3"],
                text_node=self.signal_instances["text_node"],
            )

    def _update_car_positions(self, x, y, current_time):
        # set other cars position
        if (
            self.configuration["visualization"]["show_other_vehicles"]
            and self.trajectory_data["trajectory_data"] is not None
        ):
            # position all cars on the road
            current_pos = [x, y]
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
                car_instance.setPos(vehicle[0], vehicle[1], 0)
                car_instance.setHpr(180 - vehicle[2], 90, 0)

    def _record_video_frame(self):
        if self.video_writer is not None:
            self.context.graphicsEngine.renderFrame()
            tex = cast(GraphicsOutput, self.context.win).getScreenshot()
            data = tex.getRamImageAs("BGRA")
            img_array = np.frombuffer(data, np.uint8)
            img_array = img_array.reshape(
                (
                    int(self.configuration["rendering"]["video_width_px"]),
                    int(self.configuration["rendering"]["video_height_px"]),
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
            self._update_ego_car_position(x, y, angle)
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
                sys.exit(0)
                return task.done  # stop task when trajectory is complete
            return task.again  # run again after interval
        else:
            if self.video_writer is not None:
                self.video_writer.release()
            sys.exit(0)
            return task.done  # stop task when trajectory is complete
