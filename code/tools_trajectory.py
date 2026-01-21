"""
sumo3Dviz - A three dimensional visualization of traffic simulations with SUMO
==============================================================================================
Organization: Institute for Transport Planning and Systems (IVT), ETH Zürich
Authors: Kevin Riehl <kriehl@ethz.ch>, Julius Schlapbach <juliussc@ethz.ch>
Submitted to: SUMO User Conference 2026
"""

# #############################################################################
# # IMPORTS
# #############################################################################
import numpy as np  


# #############################################################################
# # LOADING METHODS
# #############################################################################

def getVehicleTrajectoryRaw(df_simulation_log_cars, track_individual):
    df_trajectory = df_simulation_log_cars[df_simulation_log_cars["veh_id"]==track_individual]
    simulation_from_time = df_trajectory["time"].iloc[0]
    simulation_to_time = df_trajectory["time"].iloc[-1]
    return df_trajectory, simulation_from_time, simulation_to_time

def normalize_angle(angle):
    # Brings angle to [-180, 180)
    return ((angle + 180) % 360) - 180

def interpolateTrajectory(df_trajectory, VIDEO_FPS):
    df_trajectory = df_trajectory.set_index('time')
    t_min = df_trajectory.index.min()
    t_max = df_trajectory.index.max()
    new_times = np.arange(t_min, t_max, 1/VIDEO_FPS)
    df_resampled = df_trajectory.reindex(df_trajectory.index.union(new_times))
    df_resampled = df_resampled.sort_index()
    df_interpolated = df_resampled.interpolate(method='linear')
    df_interpolated_25hz = df_interpolated.loc[new_times]
    # Smooth only numeric columns
    numeric_cols = ['pos_x', 'pos_y', 'angle']
    df_numeric = df_interpolated_25hz[numeric_cols]
    window_size = 21
    df_smoothed = df_numeric.rolling(window=window_size, center=True, min_periods=1).mean()
    # Add back the non-numeric column if needed
    df_smoothed['veh_id'] = df_interpolated_25hz['veh_id']
    # Add the time column
    df_smoothed['time'] = df_smoothed.index
    # Calculate the differences (dx, dy)
    df_smoothed['dx'] = df_smoothed['pos_x'].diff()
    df_smoothed['dy'] = df_smoothed['pos_y'].diff()
    # df_smoothed['dx'] = df_smoothed['dx'].fillna(0)
    # df_smoothed['dy'] = df_smoothed['dy'].fillna(0)
    df_smoothed['distance'] = np.sqrt(df_smoothed['dx']**2 + df_smoothed['dy']**2)
    # df_smoothed['distance'] = df_smoothed['distance'].fillna(0)
    if len(df_smoothed)>0:
        df_smoothed = df_smoothed.drop(df_smoothed.index[0])
    # Compute angle in radians using atan2 (dy, dx)
    df_smoothed['computed_angle_rad'] = np.arctan2(df_smoothed['dy'], df_smoothed['dx'])
    # Convert to degrees (optional, standard for many applications)
    df_smoothed['computed_angle_deg'] = np.degrees(df_smoothed['computed_angle_rad'])
    df_smoothed['computed_angle_deg'] = 90 - df_smoothed['computed_angle_deg']
    # fill NA
    # if 'angle' in df_smoothed.columns:
    df_smoothed['computed_angle_deg'] = df_smoothed['computed_angle_deg'].fillna(df_smoothed['angle'])
    df_smoothed["computed_angle_deg"] = [normalize_angle(angle) for angle in df_smoothed["computed_angle_deg"]]
    # now delete angles where speed to small (then angle based on dx dy not reliable anymore) and replace with previous angle
    MIN_DIST = 0.03
    valid_mask = df_smoothed['distance'] > MIN_DIST
    first_valid_idx = valid_mask.idxmax() if valid_mask.any() else None
    
    if first_valid_idx is not None:
        mask = (df_smoothed.index >= first_valid_idx) & (~valid_mask)
        df_smoothed['computed_angle_deg'] = np.where(
            mask, #(df_smoothed.index == 0) | (df_smoothed.index == 1) | (df_smoothed['distance'] > MIN_DIST),
            np.nan,
            df_smoothed['computed_angle_deg'],
        )
        df_smoothed['computed_angle_deg'] = df_smoothed['computed_angle_deg'].ffill()
    # Smooth computed_angle_deg with window_size 41
    window_size_angle = 41
    df_smoothed['computed_angle_deg'] = df_smoothed['computed_angle_deg'].rolling(window=window_size_angle, center=True, min_periods=1).mean()    
    df_smoothed = df_smoothed[["time", "pos_x", "pos_y", "computed_angle_deg", "angle"]]
    df_smoothed['time'] = df_smoothed['time'].round(3)
    return df_smoothed

def load_smoothened_trajectories(VIDEO_FPS, df_simulation_log_cars, ego_individual, simulation_from_time, simulation_to_time):
    vehicles = df_simulation_log_cars["veh_id"].unique().tolist()
    grouped = df_simulation_log_cars.groupby('veh_id')
    smoothened_trajectory_data = []
    ctr = 0
    for veh_id, df_trajectory in grouped:
        if veh_id != ego_individual:
            ctr += 1        
            if len(df_trajectory)>0:
                # if (df_trajectory["time"].iloc[0]>=simulation_from_time and df_trajectory["time"].iloc[0]<=simulation_to_time) or (df_trajectory["time"].iloc[-1]>=simulation_from_time and df_trajectory["time"].iloc[-1]<=simulation_to_time):
                df_trajectory["angle"] = [normalize_angle(angle) for angle in df_trajectory["angle"]]
                df_smoothed = interpolateTrajectory(df_trajectory, VIDEO_FPS)
                smoothened_trajectory_data.append(df_smoothed)
                print("Smoothening Trajectories...", ctr, len(vehicles))
    return smoothened_trajectory_data

def get_closest_vehicles(smoothened_trajectory_data, current_pos, current_time):
    # Collect vehicles and their positions at current_time
    vehicle_data = []
    for i, df in enumerate(smoothened_trajectory_data):
        # mask = df['time'] == current_time
        mask = np.isclose(df['time'], current_time, atol=0.01)
        if mask.any():
            row = df[mask].iloc[0]
            # Use 'angle' if present, otherwise use 'computed_angle_deg' or similar
            # angle = row.get('angle', row.get('computed_angle_deg', np.nan))
            angle = row.get('computed_angle_deg', row.get('angle', np.nan))
            vehicle_data.append({
                'veh_id': i,  # Optional, if you want to keep track
                'pos_x': row['pos_x'],
                'pos_y': row['pos_y'],
                'angle': angle
            })
    # If no vehicles at current_time, return empty list
    if not vehicle_data:
        return []
    # Compute distances
    positions = np.array([[v['pos_x'], v['pos_y']] for v in vehicle_data])
    current_pos = np.array(current_pos)
    distances = np.linalg.norm(positions - current_pos, axis=1)
    # Add distances to vehicle info
    for i, v in enumerate(vehicle_data):
        v['distance'] = distances[i]
    # Sort by distance and select top 100
    sorted_vehicles = sorted(vehicle_data, key=lambda x: x['distance'])
    # Return list of [pos_x, pos_y, angle] for up to 100 closest vehicles
    return [[v['pos_x'], v['pos_y'], v['angle'], v["veh_id"]] for v in sorted_vehicles[:200]]
