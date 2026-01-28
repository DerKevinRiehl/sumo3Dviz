import numpy as np
import math
import cv2
import sys
from typing import cast, Optional, Dict, Any
from direct.showbase.ShowBase import ShowBase
from panda3d.core import NodePath, Camera, GraphicsOutput


class SimulationManager:
    """
    Manages the simulation loop for rendering SUMO traffic simulations.
    Encapsulates all state variables and update logic for the Panda3D task manager.
    """

    def __init__(
        self,
        context: ShowBase,
        trajectory_points: np.ndarray,
        signal_points: Optional[np.ndarray],
        video_start_idx: int,
        video_end_idx: int,
        car_models: list,
        ego_car: NodePath,
        smoothened_trajectory_data: Optional[list],
        trajectory_tools: Any,
        rendering_tools: Any,
        viewer_height: float,
        show_other_vehicles: bool,
        ramp_metering: bool,
        design: str,
        video_width_px: float,
        video_height_px: float,
        video_writer: Optional[cv2.VideoWriter] = None,
        box_node1: Optional[NodePath] = None,
        box_node2: Optional[NodePath] = None,
        box_node3: Optional[NodePath] = None,
        text_node: Optional[Any] = None,
    ):
        """
        Initialize the SimulationManager with all required parameters.

        Args:
            context: The Panda3D ShowBase context
            trajectory_points: Array of [veh_id, pos_x, pos_y, angle, time] for ego vehicle
            signal_points: Array of [time, state, timer] for traffic signals (optional)
            video_start_idx: Starting index in trajectory_points
            video_end_idx: Ending index in trajectory_points
            car_models: List of loaded car models for other vehicles
            ego_car: The ego vehicle NodePath
            smoothened_trajectory_data: List of smoothed trajectories for other vehicles
            trajectory_tools: Instance of TrajectoryTools for vehicle positioning
            rendering_tools: Instance of RenderingTools for traffic light updates
            viewer_height: Camera height above ground
            show_other_vehicles: Whether to render other vehicles
            ramp_metering: Whether ramp metering is enabled
            design: Traffic light design type ("simple", "three_headed", "countdown_timer")
            video_width_px: Width of the output video in pixels
            video_height_px: Height of the output video in pixels
            video_writer: OpenCV VideoWriter (optional)
            box_node1, box_node2, box_node3: Traffic light nodes (optional)
            text_node: Traffic light timer text node (optional)
        """
        self.context = context
        self.trajectory_points = trajectory_points
        self.signal_points = signal_points
        self.video_start_idx = video_start_idx
        self.video_end_idx = video_end_idx
        self.car_models = car_models
        self.ego_car = ego_car
        self.smoothened_trajectory_data = smoothened_trajectory_data
        self.trajectory_tools = trajectory_tools
        self.rendering_tools = rendering_tools
        self.viewer_height = viewer_height
        self.show_other_vehicles = show_other_vehicles
        self.ramp_metering = ramp_metering
        self.design = design
        self.video_width_px = video_width_px
        self.video_height_px = video_height_px
        self.video_writer = video_writer
        self.box_node1 = box_node1
        self.box_node2 = box_node2
        self.box_node3 = box_node3
        self.text_node = text_node

        # state variables
        self.current_point = video_start_idx
        self.screenshot_counter = 0
        self.others_car_instances: Dict[int, NodePath] = {}

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

            # get signal state if available
            if self.signal_points is not None:
                _, signal, timer = self.signal_points[self.current_point]
            else:
                signal, timer = None, 0

            # set camera position and heading
            cast(Camera, self.context.camera).setPos(x, y, self.viewer_height)
            cast(Camera, self.context.camera).setHpr(-angle, 0, 0)

            # set ego car position
            distance = 1.6  # how far in front you want the car to be
            car_x = x + distance * math.cos(math.radians(90 - angle))
            car_y = y + distance * math.sin(math.radians(90 - angle))
            car_z = -0.5
            self.ego_car.setPos(car_x, car_y, car_z)
            self.ego_car.setHpr(-angle, 90, 0)

            # set traffic light
            if self.ramp_metering and signal is not None:
                self.rendering_tools.update_traffic_light(
                    signal=signal,
                    design=self.design,
                    timer=timer,
                    box_node1=self.box_node1,
                    box_node2=self.box_node2,
                    box_node3=self.box_node3,
                    text_node=self.text_node,
                )

            print(current_time, signal, timer, self.current_point, self.video_end_idx)

            # set other cars position
            if self.show_other_vehicles and self.smoothened_trajectory_data is not None:
                # position all cars on the road
                current_pos = [x, y]
                neighborhood_vehicles = self.trajectory_tools.get_closest_vehicles(
                    self.smoothened_trajectory_data, current_pos, current_time
                )

                # create new instances
                current_vehicle_ids = []
                for vehicle in neighborhood_vehicles:
                    vehicle_id = vehicle[-1]
                    current_vehicle_ids.append(vehicle_id)
                    if vehicle_id not in self.others_car_instances:
                        available_choices = [i for i in range(1, 11) if i != 2]
                        selected_model = np.random.choice(available_choices)
                        new_vehicle_instance = self.car_models[
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

            self.current_point += 1
            self.current_point += 1  # double render speed

            # record video frame
            if self.video_writer is not None:
                self.context.graphicsEngine.renderFrame()
                tex = cast(GraphicsOutput, self.context.win).getScreenshot()
                data = tex.getRamImageAs("BGRA")
                img_array = np.frombuffer(data, np.uint8)
                img_array = img_array.reshape(
                    (
                        int(self.video_height_px),
                        int(self.video_width_px),
                        4,
                    )
                )
                img_array = cv2.rotate(img_array, cv2.ROTATE_180)
                img_array = cv2.flip(img_array, 1)
                img = img_array[:, :, :3]
                self.video_writer.write(img)
                self.screenshot_counter += 1

            if self.current_point > self.video_end_idx:
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
