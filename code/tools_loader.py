"""
sumo3Dviz - A three dimensional visualization of traffic simulations with SUMO
==============================================================================================
Organization: Institute for Transport Planning and Systems (IVT), ETH Zürich
Authors: Kevin Riehl <kriehl@ethz.ch>, Julius Schlappbach <julius.schlapbach@ivt.baug.ethz.ch>
Submitted to: SUMO User Conference 2026
"""

# #############################################################################
# # IMPORTS
# #############################################################################
import numpy as np
import pandas as pd
import sumolib

from tools_trajectory import getVehicleTrajectoryRaw, normalize_angle, interpolateTrajectory, load_smoothened_trajectories


# #############################################################################
# # LOADING METHODS
# #############################################################################

def load_car_models(context, path_car_models="../data/3d_models/cars/Low Poly Cars.glb"):
    car_collection = context.loader.loadModel(path_car_models)
    car_models = [car_collection.find("**/"+str(n)) for n in range(1,10+1)]
    return car_models

def load_tree_positions(xml_file):
    tree_pois = sumolib.xml.parse(xml_file, 'poi')
    tree_positions = []
    for poi in tree_pois:
        x = float(poi.x)
        y = float(poi.y)
        tree_positions.append([x, y])
    return tree_positions
    
def load_fence_lines(xml_file):
    polys = sumolib.xml.parse(xml_file, 'poly')
    poly_lines = []
    for poly in polys:
        # poly.shape is a string like "x1,y1 x2,y2 ..."
        points = []
        for point in poly.shape.split():
            x, y = map(float, point.split(','))
            points.append([x, y])
        poly_lines.append(points)
    return poly_lines

def load_shop_positions(xml_file):
    tree_pois = sumolib.xml.parse(xml_file, 'poi')
    tree_positions = []
    for poi in tree_pois:
        x = float(poi.x)
        y = float(poi.y)
        tree_positions.append([x, y])
    return tree_positions

def load_trajectory(path_trajectory_file, trajectory_parameter, video_parameters, SHOW_OTHERS):
    # Determine trajectory
    df_simulation_log_cars = pd.read_csv(path_trajectory_file)
    # df_simulation_log_cars = pd.read_csv("../examples/barcelona_simulation/vehicle_pos_log_collection/xx_vehicle_pos_log_file_"+SIMULATION+".csv")
    df_ego_trajectory, simulation_from_time, simulation_to_time = getVehicleTrajectoryRaw(df_simulation_log_cars, trajectory_parameter["ego_individual"])
    df_ego_trajectory["angle"] = [normalize_angle(angle) for angle in df_ego_trajectory["angle"]]
    df_ego_smoothed = interpolateTrajectory(df_ego_trajectory, video_parameters["fps_rate"])
    df_ego_smoothed["veh_id"] = trajectory_parameter["ego_individual"]
    del df_ego_smoothed["time"]
    df_ego_smoothed = df_ego_smoothed.reset_index()
    # Determine Video cat
    VIDEO_CURRENT_POINT = (df_ego_smoothed['time'] - int(trajectory_parameter["render_from_simtime"])).abs().idxmin()
    VIDEO_FINAL_POINT = (df_ego_smoothed['time'] - int(trajectory_parameter["render_to_simtime"])).abs().idxmin()
    # Get smoothed trajectory of all vehicles
    if SHOW_OTHERS:
        smoothened_trajectory_data = load_smoothened_trajectories(video_parameters["fps_rate"], df_simulation_log_cars, trajectory_parameter["ego_individual"], simulation_from_time, simulation_to_time)
    else:
        smoothened_trajectory_data = None
    return df_ego_smoothed, smoothened_trajectory_data, VIDEO_CURRENT_POINT, VIDEO_FINAL_POINT

def load_traffic_light_signals(path_signal_file, df_ego_smoothed):
    df_simulation_log_light = pd.read_csv(path_signal_file)
    df_simulation_log_light = _convert_to_25Hz(df_simulation_log_light)
    df_simulation_log_light = df_simulation_log_light[df_simulation_log_light["time"]<=df_ego_smoothed["time"].iloc[-1]+0.001]
    df_simulation_log_light = df_simulation_log_light[df_simulation_log_light["time"]>=df_ego_smoothed["time"].iloc[0]-0.001]
    df_simulation_log_light = _add_yellow_transition(df_simulation_log_light)
    df_simulation_log_light['state'] = df_simulation_log_light['state'].replace({'G': 'green', 'y': 'yellow', 'y2': 'yellow2', 'r': 'red'})
    df_simulation_log_light = _add_countdown_timer(df_simulation_log_light)
    return df_simulation_log_light

def _add_yellow_transition(df_simulation_log_light):
    df = df_simulation_log_light.copy()
    # Find indices where the state changes from 'r' to 'G'
    red_to_green_idx = df.index[(df['state'].shift(1) == 'r') & (df['state'] == 'G')]
    green_to_red_idx = df.index[(df['state'].shift(1) == 'G') & (df['state'] == 'r')]
    for idx in red_to_green_idx:
        transition_time = df.loc[idx, 'time']
        # 3 seconds BEFORE transition from 'r' to 'G' : set to 'y'
        mask_before = (
            (df['state'] == 'r') &
            (df['time'] >= transition_time - 3) &
            (df['time'] < transition_time)
        )
        df.loc[mask_before, 'state'] = 'y'
    for idx in green_to_red_idx:
        transition_time = df.loc[idx, 'time']
        # 3 seconds BEFORE transition from 'G' to 'r' : set to 'y2'
        mask_before = (
            (df['state'] == 'G') &
            (df['time'] >= transition_time - 3) &
            (df['time'] < transition_time)
        )
        df.loc[mask_before, 'state'] = 'y2'
    return df

def _add_countdown_timer(df):
    df = df.copy()
    # For each row, find the next green index (or np.nan if none)
    next_green = np.full(len(df), np.nan)
    last_green = None
    # Traverse backwards to fill the next green time for each row
    for i in reversed(range(len(df))):
        if df.iloc[i]['state'] == 'green':
            last_green = df.iloc[i]['time']
        next_green[i] = last_green if last_green is not None else np.nan
    # Calculate countdown: time until next green, or 0 if already green
    df['timer'] = np.where(
        (df['state'] == 'green') | (df["state"] == "yellow2"),
        0,
        next_green - df['time']
    )
    return df

def _convert_to_25Hz(df_simulation_log_light):
    # Step 1: Ensure time is float
    df_simulation_log_light['time'] = df_simulation_log_light['time'].astype(float)
    # Step 2: Set time as index
    df_simulation_log_light = df_simulation_log_light.set_index('time')
    # Step 3: Create new time index at 25 Hz
    start_time = df_simulation_log_light.index.min()
    end_time = df_simulation_log_light.index.max()
    new_time_index = np.arange(start_time, end_time + 1/25, 1/25)
    # Step 4: Reindex and forward-fill
    df_25hz = df_simulation_log_light.reindex(new_time_index, method='ffill')
    df_25hz = df_25hz.reset_index().rename(columns={'index': 'time'})
    return df_25hz