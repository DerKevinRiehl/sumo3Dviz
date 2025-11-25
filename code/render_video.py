"""
sumo3Dviz - A three dimensional visualization of traffic simulations with SUMO
==============================================================================================
This script renders a video from a given SUMO Simulation for an ego's perspective.
==============================================================================================
Organization: Institute for Transport Planning and Systems (IVT), ETH Zürich
Authors: Kevin Riehl <kriehl@ethz.ch>, Julius Schlappbach <julius.schlapbach@ivt.baug.ethz.ch>
Submitted to: SUMO User Conference 2026
"""

# #############################################################################
# # IMPORTS
# #############################################################################
import numpy as np  
import math
import cv2
import sys

from direct.showbase.ShowBase import ShowBase
from panda3d.core import Filename, loadPrcFileData

from tools_rendering import create_light, create_sky, create_floor, create_trees, create_building_shops, create_building_homes, create_building_blocks
from tools_rendering import create_road_network, draw_highway_fences, draw_traffic_light, draw_white_signal_line, update_traffic_light
from tools_interactive import addCameraControlKeyboard
from tools_trajectory import get_closest_vehicles
from tools_loader import load_car_models, load_tree_positions, load_fence_lines, load_shop_positions, load_trajectory, load_traffic_light_signals


# #############################################################################
# # PARAMETERS
# #############################################################################

video_parameters = {
    "output_file": "output_video.avi",
    "fps_rate": 25,
    "frame_width_px": 1140,
    "frame_heigth_px": 900,
}

trajectory_parameters = {
    "input_file": "../examples/barcelona_simulation/vehicle_pos_log_collection/xx_vehicle_pos_log_file_60_ALINEA.zip",
    "ego_individual": "sample_flow_E2_A3.32",
    "render_from_simtime": 3119.178096,
    "render_to_simtime": 3379.616172,
}

visualization_parameter = {
    "lane_width": 3.2,
    "viewer_height": 1.5,
    "record_video": True,
    "show_others": True
}

DESIGN = 0 # DESIGN TYPES: 0 = SIMPLE, 1 = 3-head, 2 = COUNTDOWN_TIMER, 3 = SIMPLE 
RAMP_METERING = False
SIMULATION = "NO_CONTROL" # SIMULATION TYPES: "NO_CONTROL", "45_ALINEA", "60_ALINEA"


# #############################################################################
# # METHODS
# #############################################################################

def update_scene_world(task):
    global VIDEO_CURRENT_POINT
    # global screenshot_counter
    if VIDEO_CURRENT_POINT < len(trajectory_points):
        veh_id, x, y, angle, current_time = trajectory_points[VIDEO_CURRENT_POINT]
        _, signal, timer = signal_points[VIDEO_CURRENT_POINT]
        # Set camera position and heading
        context.camera.setPos(x, y, visualization_parameter["viewer_height"])  # Z=2 is your previous camera height
        context.camera.setHpr(-angle, 0, 0)  # Adjust pitch and roll as needed
        # Set ego car
        distance = 1.6  # How far in front you want the car to be
        car_x = x + distance * math.cos(math.radians(90-angle))
        car_y = y + distance * math.sin(math.radians(90-angle))
        car_z = -0.5
        ego_car.setPos(car_x, car_y, car_z)
        ego_car.setHpr(-angle, 90, 0)
        # Set traffic light
        if RAMP_METERING:
            update_traffic_light(signal, DESIGN, timer, box_node1, box_node2, box_node3, text_node)
        print(current_time, signal, timer, VIDEO_CURRENT_POINT, VIDEO_FINAL_POINT)
        # Set Other Cars position
        if visualization_parameter["show_others"]:
            # position all cars on the road
                # determine list of 100 closest cars
            current_pos=[x,y]
            neighborhood_vehicles = get_closest_vehicles(smoothened_trajectory_data, current_pos, current_time)  
            # create new instance
            current_vehicle_ids = []
            for vehicle in neighborhood_vehicles:
                vehicle_id = vehicle[-1]
                current_vehicle_ids.append(vehicle_id)
                if vehicle_id not in others_car_instances:
                    available_choices = [i for i in range(1, 11) if i != 2]
                    selected_model = np.random.choice(available_choices)
                    new_vehicle_instance = car_models[selected_model-1].copyTo(context.render)
                    new_vehicle_instance.setScale(5.0)
                    others_car_instances[vehicle_id] = new_vehicle_instance
            # delete unused instances
            ids_to_delete = []
            for vehicle in others_car_instances:
                if vehicle not in current_vehicle_ids:
                    ids_to_delete.append(vehicle)
            for ids in ids_to_delete:
                others_car_instances[ids].removeNode() # remove from app
                del others_car_instances[ids]
            # move instances
            for vehicle in neighborhood_vehicles:
                car_instance = others_car_instances[vehicle[-1]]
                car_instance.setPos(vehicle[0], vehicle[1], 0)
                car_instance.setHpr(180-vehicle[2], 90, 0)            
        VIDEO_CURRENT_POINT += 1
        VIDEO_CURRENT_POINT += 1 # double render speed
        if visualization_parameter["record_video"]:
            # Screen Shot
            filename = "screenshots/"+video_parameters["output_file"]+".png"
            context.win.saveScreenshot(Filename(filename))
            # Write the frame to the video
            img = cv2.imread(filename)
            video_writer.write(img)
        if VIDEO_CURRENT_POINT > VIDEO_FINAL_POINT:
            video_writer.release()
            sys.exit(0)
            return task.done  # Stop task when trajectory is complete
        return task.again  # Run again after interval
    else:
        video_writer.release()
        sys.exit(0)
        return task.done  # Stop task when trajectory is complete


# #############################################################################
# # LOAD DATA
# #############################################################################      
# load trajectory
df_ego_smoothed, smoothened_trajectory_data, VIDEO_CURRENT_POINT, VIDEO_FINAL_POINT = load_trajectory(trajectory_parameters["input_file"], trajectory_parameters, video_parameters, visualization_parameter["show_others"])
# load traffic light signal
df_simulation_log_light = load_traffic_light_signals("../examples/barcelona_simulation/vehicle_pos_log_collection/xx_vehicle_pos_log_file_"+SIMULATION+"_lights.csv", df_ego_smoothed)
# convert list of tuples for easier access
trajectory_points = df_ego_smoothed[["veh_id", 'pos_x', 'pos_y', 'computed_angle_deg', 'time']].values
signal_points = df_simulation_log_light[["time", "state", "timer"]].values
# load object positions
tree_positions = load_tree_positions("../examples/barcelona_simulation/viz_object_positions/trees.add.xml")
fence_lines = load_fence_lines("../examples/barcelona_simulation/viz_object_positions/fences.add.xml")
shop_positions = load_shop_positions("../examples/barcelona_simulation/viz_object_positions/buildings_shops.add.xml")
homes_positions = load_shop_positions("../examples/barcelona_simulation/viz_object_positions/buildings_homes.add.xml")
block_positions = load_shop_positions("../examples/barcelona_simulation/viz_object_positions/buildings_blocks.add.xml")
traffic_light_positions = {
    "ramp_1": {
        "pos_x": 20217.08-0.0,
        "pos_y": 18261.92+0.0,
        "stop_line_a": 20224.71,
        "stop_line_b": 18264.87,
        "stop_line_c": 20226.77,
        "stop_line_d": 18261.15
    },
    "ramp_2": {
        "pos_x": 19315.13-0.0,
        "pos_y": 17822.42+4.5,
        "stop_line_a": 20224.71,
        "stop_line_b": 18264.87,
        "stop_line_c": 20226.77,
        "stop_line_d": 18261.15
    }
}


# #############################################################################
# # MAIN
# #############################################################################

######## CREATE VIDEO WRITER
if visualization_parameter["record_video"]:
    video_writer = cv2.VideoWriter(video_parameters["output_file"],
                                   cv2.VideoWriter_fourcc(*'MJPG'),
                                   video_parameters["fps_rate"],
                                   (video_parameters["frame_width_px"], video_parameters["frame_heigth_px"]), True)


######## CREATE 3D CONTEXT OBJECT
loadPrcFileData('', 'win-size '+str(video_parameters["frame_width_px"])+' '+str(video_parameters["frame_heigth_px"]))
loadPrcFileData('', 'framebuffer-multisample 1')
loadPrcFileData('', 'multisamples 8')
# loadPrcFileData('', 'load-file-type p3assimp')
context = ShowBase()
# Settings
addCameraControlKeyboard(context)

######## CREATE ELEMENTS
# Draw World
    # Light source (otherwise all will be dark)
create_light(context)
    # SkyBox
create_sky(context)
    # GrassFloor
create_floor(context)
    # roads / sumo network
create_road_network(context, '../examples/barcelona_simulation/Network_2Sided.net.xml')
    # trees
tree_instances = create_trees(context, tree_positions)
    # highways fences
draw_highway_fences(context, fence_lines)
    # buildings
create_building_shops(context, shop_positions)
create_building_homes(context, homes_positions)
create_building_blocks(context, block_positions)
    # other cars collection
car_models = load_car_models(context)
others_car_instances = {}
    # ego car
ego_car = context.loader.loadModel('../data/3d_models/cars/Car.glb')
ego_car.reparentTo(context.render)
ego_car.setPos(visualization_parameter["lane_width"]/2, 25, 0)
ego_car.setHpr(180, 90, 0)
    # traffic light
if RAMP_METERING:
    box_node1, box_node2, box_node3, text_node = draw_traffic_light(context, DESIGN, traffic_light_positions["ramp_1"]["pos_x"], traffic_light_positions["ramp_1"]["pos_y"], 0)
    draw_white_signal_line(context, traffic_light_positions["ramp_1"]["stop_line_a"], traffic_light_positions["ramp_1"]["stop_line_b"], traffic_light_positions["ramp_1"]["stop_line_c"], traffic_light_positions["ramp_1"]["stop_line_d"])

######## POSITION CAMERA (INITIALLY)
context.camera.setPos(df_ego_smoothed["pos_x"].iloc[VIDEO_CURRENT_POINT], df_ego_smoothed["pos_y"].iloc[VIDEO_CURRENT_POINT], visualization_parameter["viewer_height"])
context.camera.setHpr(120, 0, 0)

######## RUN SCHEDULE / GO THROUGH TRAJECTORY
context.taskMgr.doMethodLater(0.0, update_scene_world, 'update_scene_world')
context.run()

######## CLOSE VIDEO WRITER
if visualization_parameter["record_video"]:
    video_writer.release()