# #############################################################################
# # IMPORTS
# #############################################################################
import numpy as np  
import sumolib
import pandas as pd
import math
import cv2
import sys

from direct.showbase.ShowBase import ShowBase
from panda3d.core import Geom, GeomNode, GeomVertexData, GeomVertexFormat, GeomVertexWriter, GeomTriangles
from panda3d.core import CardMaker, LColor, Filename, TextureStage, loadPrcFileData
from panda3d.core import LineSegs, FontPool, TextNode, Texture
from panda3d.core import DirectionalLight, AmbientLight, Vec4




# #############################################################################
# # METHODS
# #############################################################################

def createSkyBox(app):
    # generate an inverted sphere model
    sky_sphere = app.loader.loadModel("models/smiley") 
    sky_sphere.reparentTo(app.render)
    sky_sphere.setScale(INFINITY) 
    sky_sphere.setTwoSided(True) 
    # Load the texture
    sky_texture = Texture()
    sky_texture.read(Filename("../data/images/puresky.jpg"))
    sky_sphere.setTexture(sky_texture, 1)
    # Set rendering properties
    sky_sphere.setBin('background', 0)
    sky_sphere.setDepthWrite(False)
    sky_sphere.setLightOff() 

def createGrassFloor(app):
    # Grass Floor
    cm_ground = CardMaker("ground")
    cm_ground.setFrame(-INFINITY, INFINITY, -INFINITY, INFINITY)
    ground = app.render.attachNewNode(cm_ground.generate())
    ground.setPos(0, 0, -0.05)
    ground.setHpr(25, -90, 0)
    # Load the texture
    ground_tex = app.loader.loadTexture("../data/images/grass.jpg")
    ground_tex.setWrapU(Texture.WM_repeat)
    ground_tex.setWrapV(Texture.WM_repeat)
    ground.setTexture(ground_tex)
    ground.setTexScale(TextureStage.getDefault(), 40000, 40000)

def addCameraControlKeyboard(app):
    # Move camera left/right/forward/backward
    def move_left(): 
        app.camera.setX(app.camera, -1)
    def move_right(): 
        app.camera.setX(app.camera, 1)
    def move_forward(): 
        app.camera.setY(app.camera, 1)
    def move_backward(): 
        app.camera.setY(app.camera, -1)
    # Move camera up/down (w/s)
    def move_up():
        app.camera.setZ(app.camera, 1)
    def move_down():
        app.camera.setZ(app.camera, -1)
    # Look up/down (q/a) -- rotate pitch
    def look_up():
        app.camera.setP(app.camera.getP() + 5)
    def look_down():
        app.camera.setP(app.camera.getP() - 5)
    # Look left/right (e/d) -- rotate heading
    def look_left():
        app.camera.setH(app.camera.getH() + 5)
    def look_right():
        app.camera.setH(app.camera.getH() - 5)
    app.accept("arrow_left", move_left)
    app.accept("arrow_right", move_right)
    app.accept("arrow_up", move_forward)
    app.accept("arrow_down", move_backward)
    app.accept("w", move_up)
    app.accept("s", move_down)
    app.accept("q", look_up)
    app.accept("a", look_down)
    app.accept("e", look_left)
    app.accept("d", look_right)
    app.disableMouse()

def create_light(app):
    # Ambient light (fill light)
    alight = AmbientLight('alight')
    alight.setColor(Vec4(1.0, 1.0, 1.0, 1))
    alnp = app.render.attachNewNode(alight)
    app.render.setLight(alnp)
    # Directional light (for highlights and shading)
    dlight = DirectionalLight('dlight')
    dlight.setColor(Vec4(1, 1, 1, 1))
    dlnp = app.render.attachNewNode(dlight)
    dlnp.setPos(0, -100, 200)
    dlnp.setHpr(-30, -45, 0)  # pointing downward and forward
    app.render.setLight(dlnp)
     
def draw_concrete(app, lane_shape, color, z=0.01):
    for i in range(0, len(lane_shape)-1):
        pA = np.asarray(lane_shape[i])
        pB = np.asarray(lane_shape[i+1])
        # length
        lane_length = np.linalg.norm(pA-pB)
        # angle
        dx = pB[0] - pA[0]
        dy = pB[1] - pA[1]
        angle_deg = np.degrees(np.arctan2(dy, dx))-90
        # center
        center_x = (pA[0])
        center_y = (pA[1])
        # make road
        cm_road = CardMaker("card")
        cm_road.setFrame(-LANE_WIDTH/2, LANE_WIDTH/2, 0, lane_length)
        road = app.render.attachNewNode(cm_road.generate())
        road.setPos(center_x, center_y, z)
        road.setHpr(angle_deg, -90, 0)
        road.setColor(LColor(*color))    

def draw_white_seperator_line_left(app, lane_shape, color, z=0.03):
    for i in range(0, len(lane_shape)-1):
        pA = np.asarray(lane_shape[i])
        pB = np.asarray(lane_shape[i+1])
        # length
        lane_length = np.linalg.norm(pA-pB)
        # angle
        dx = pB[0] - pA[0]
        dy = pB[1] - pA[1]
        angle_deg = np.degrees(np.arctan2(dy, dx))-90
        # center
        center_x = (pA[0])
        center_y = (pA[1])
        # make road
        cm_road = CardMaker("card")
        cm_road.setFrame(-LANE_WIDTH/2+SEP_LINE_WIDTH-SEP_LINE_WIDTH/2, -LANE_WIDTH/2+SEP_LINE_WIDTH+SEP_LINE_WIDTH/2, 0, lane_length)
        road = app.render.attachNewNode(cm_road.generate())
        road.setPos(center_x, center_y, z)
        road.setHpr(angle_deg, -90, 0)
        road.setColor(LColor(*color))  
        
def draw_white_seperator_line_right(app, lane_shape, color, z=0.03):
    for i in range(0, len(lane_shape)-1):
        pA = np.asarray(lane_shape[i])
        pB = np.asarray(lane_shape[i+1])
        # length
        lane_length = np.linalg.norm(pA-pB)
        # angle
        dx = pB[0] - pA[0]
        dy = pB[1] - pA[1]
        angle_deg = np.degrees(np.arctan2(dy, dx))-90
        # center
        center_x = (pA[0])
        center_y = (pA[1])
        # make road
        cm_road = CardMaker("card")
        cm_road.setFrame(+LANE_WIDTH/2-SEP_LINE_WIDTH-SEP_LINE_WIDTH/2, +LANE_WIDTH/2-SEP_LINE_WIDTH+SEP_LINE_WIDTH/2, 0, lane_length)
        road = app.render.attachNewNode(cm_road.generate())
        road.setPos(center_x, center_y, z)
        road.setHpr(angle_deg, -90, 0)
        road.setColor(LColor(*color))  
       
def draw_white_separator_line_right_dashed(app, lane_shape, color, z=0.03, dash_length=1.0, gap_length=1.0):
    for i in range(len(lane_shape) - 1):
        pA = np.asarray(lane_shape[i])
        pB = np.asarray(lane_shape[i + 1])
        segment_vector = pB - pA
        segment_length = np.linalg.norm(segment_vector)
        if segment_length == 0:
            continue
        direction = segment_vector / segment_length
        num_dashes = int(segment_length // (dash_length + gap_length))
        angle_deg = np.degrees(np.arctan2(direction[1], direction[0])) - 90
        for j in range(num_dashes):
            start_offset = (dash_length + gap_length) * j
            dash_center = pA + direction * (start_offset + dash_length / 2)
            # Offset to the right side of the lane
            normal = np.array([-direction[1], direction[0]])  # 90-degree rotation
            offset = normal * (LANE_WIDTH / 2 - SEP_LINE_WIDTH / 2)
            final_pos = dash_center + offset
            # Create dashed card
            cm_dash = CardMaker("dash")
            cm_dash.setFrame(-SEP_LINE_WIDTH / 2, SEP_LINE_WIDTH / 2, 0, dash_length)
            dash = app.render.attachNewNode(cm_dash.generate())
            dash.setPos(final_pos[0], final_pos[1], z)
            dash.setHpr(angle_deg, -90, 0)
            dash.setColor(LColor(*color))
            
def draw_road_edge_lane(app, lane_shape, lane_id):
    draw_concrete(app, lane_shape, CONCRETE_COLOR)
    if lane_id=="b":
        draw_white_seperator_line_left(app, lane_shape, SEPERATOR_COLOR)
        draw_white_seperator_line_right(app, lane_shape, SEPERATOR_COLOR)
    if lane_id=="a":
        draw_white_seperator_line_right(app, lane_shape, SEPERATOR_COLOR)
        draw_white_separator_line_right_dashed(app, lane_shape, SEPERATOR_COLOR, z=0.03, dash_length=1.0, gap_length=1.0)
    elif lane_id=="e":
        draw_white_seperator_line_left(app, lane_shape, SEPERATOR_COLOR)
    elif lane_id=="i":
        draw_white_separator_line_right_dashed(app, lane_shape, SEPERATOR_COLOR, z=0.03, dash_length=1.0, gap_length=1.0)     

def draw_polygon_fan(app, shape_points, z=0.01):
    if len(shape_points) < 3:
        return
    gformat = GeomVertexFormat.getV3cp()
    vdata = GeomVertexData("fan", gformat, Geom.UHDynamic)
    vertex = GeomVertexWriter(vdata, "vertex")
    color_writer = GeomVertexWriter(vdata, "color")
    # Write center vertex
    xs, ys = zip(*shape_points)
    cx = sum(xs) / len(xs)
    cy = sum(ys) / len(ys)
    vertex.addData3(cx, cy, z)
    color_writer.addData4f(*CONCRETE_COLOR)
    center_index = 0
    # Write perimeter vertices
    for x, y in shape_points:
        vertex.addData3(x, y, z)
        color_writer.addData4f(*CONCRETE_COLOR)
    # Create triangle fan
    tris = GeomTriangles(Geom.UHDynamic)
    for i in range(1, len(shape_points)):
        tris.addVertices(center_index, i, i+1)
    tris.addVertices(center_index, len(shape_points), 1)  # Close the loop
    geom = Geom(vdata)
    geom.addPrimitive(tris)
    node = GeomNode("polygon_fan")
    node.addGeom(geom)
    nodepath = app.render.attachNewNode(node)
    nodepath.setTwoSided(True)

def draw_sumo_network(app, network_file):
    # Draw Roads
    net = sumolib.net.readNet(network_file)
    for edge in net.getEdges():
        for lane in edge.getLanes():
            lane_shape = lane.getShape()
            lane_id = lane.getID().split("_")[-1]
            if edge.getLaneNumber()==1:
                lane_l = "b"
            elif lane_id=="0":
                lane_l = "a"
            elif (lane_id)==str(edge.getLaneNumber()-1):
                lane_l = "e"
            else:
                lane_l = "i"
            draw_road_edge_lane(app, lane_shape, lane_l)
    # Draw Junctions
    for junction in net.getNodes():
        junction_shape = junction.getShape()
        if junction_shape:
            draw_polygon_fan(app, junction_shape)
    
def getVehicleTrajectoryRaw(df_simulation_log_cars, track_individual):
    df_trajectory = df_simulation_log_cars[df_simulation_log_cars["veh_id"]==track_individual]
    simulation_from_time = df_trajectory["time"].iloc[0]
    simulation_to_time = df_trajectory["time"].iloc[-1]
    return df_trajectory, simulation_from_time, simulation_to_time

def normalize_angle(angle):
    # Brings angle to [-180, 180)
    return ((angle + 180) % 360) - 180

def interpolateTrajectory(df_trajectory):
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

def load_smoothened_trajectories(df_simulation_log_cars, ego_individual, simulation_from_time, simulation_to_time):
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
                df_smoothed = interpolateTrajectory(df_trajectory)
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

def update_scene_world(task):
    global VIDEO_CURRENT_POINT
    # global screenshot_counter
    if VIDEO_CURRENT_POINT < len(trajectory_points):
        veh_id, x, y, angle, current_time = trajectory_points[VIDEO_CURRENT_POINT]
        _, signal, timer = signal_points[VIDEO_CURRENT_POINT]
        # Set camera position and heading
        app.camera.setPos(x, y, VIEWER_HEIGHT)  # Z=2 is your previous camera height
        app.camera.setHpr(-angle, 0, 0)  # Adjust pitch and roll as needed
        # Set ego car
        distance = 1.6  # How far in front you want the car to be
        car_x = x + distance * math.cos(math.radians(90-angle))
        car_y = y + distance * math.sin(math.radians(90-angle))
        car_z = -0.5
        ego_car.setPos(car_x, car_y, car_z)
        ego_car.setHpr(-angle, 90, 0)
        # Set traffic light
        if RAMP_METERING:
            update_traffic_light(signal, timer, box_node1, box_node2, box_node3, text_node)
        print(current_time, signal, timer, VIDEO_CURRENT_POINT, VIDEO_FINAL_POINT)
        # Set Other Cars position
        if SHOW_OTHERS:
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
                    new_vehicle_instance = car_models[selected_model-1].copyTo(app.render)
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
        if RECORD_VIDEO:
            # Screen Shot
            filename = "screenshots/"+VIDEO_FILE+".png"
            app.win.saveScreenshot(Filename(filename))
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

def load_tree_positions(xml_file):
    tree_pois = sumolib.xml.parse(xml_file, 'poi')
    tree_positions = []
    for poi in tree_pois:
        x = float(poi.x)
        y = float(poi.y)
        tree_positions.append([x, y])
    return tree_positions

def spawn_trees(app, tree_positions):
    # Load tree models
    tree1_model = app.loader.loadModel('../data/3d_models/trees/MapleTree.obj')
    tree2_model = app.loader.loadModel('../data/3d_models/trees/Hazelnut.obj')
    tree_scale_1 = 0.2
    tree_scale_2 = 0.5
    tree_size_variability = 1
    tree_color_variability = 0.1
    # generate trees
    tree_instances = []
    for position in tree_positions:
        x = position[0]
        y = position[1]
        # select a tree randomly
        if np.random.random()<0.5:
            tree_instance = tree1_model.copyTo(app.render)
            tree_scale = tree_scale_1
        else:
            tree_instance = tree2_model.copyTo(app.render)
            tree_scale = tree_scale_2
        # select a color randomly
        tree_instance.setColor(0,0.1+(1+np.random.random())*tree_color_variability,0,1)
        # select a scale randomly
        tree_instance.setScale(tree_scale*(1+np.random.random())*tree_size_variability)
        # place tree
        tree_instance.setPos(x, y, 0)
        tree_instance.setHpr(np.random.random()*360, 90, 0)
        tree_instances.append(tree_instance)
    return tree_instances
    
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

def draw_vertical_barrier(app, lines, color, z_start=0.03, z_end=1.0, spacing=2.0):
    POLE_THICKNESS = 10
    VERT_LINE_DIST = 0.3
    for line in lines:
        all_posts = []  # Collect all post positions
        for i in range(0, len(line)-1):
            pA = np.asarray(line[i])
            pB = np.asarray(line[i+1])
            dx = pB[0] - pA[0]
            dy = pB[1] - pA[1]
            lane_length = np.linalg.norm(pA-pB)
            num_posts = int(lane_length / spacing)
            for j in range(num_posts + 1):
                x = pA[0] + (dx * j / num_posts)
                y = pA[1] + (dy * j / num_posts)
                all_posts.append((x, y, z_end))  # Collect top of each post
                # Draw vertical line
                lines = LineSegs()
                lines.setColor(LColor(*color))
                lines.setThickness(POLE_THICKNESS)
                lines.moveTo(x, y, z_start)
                lines.drawTo(x, y, z_end)
                app.render.attachNewNode(lines.create())
        # Draw two connecting lines through the tops of the posts
        if all_posts:
            lines = LineSegs()
            lines.setColor(LColor(*color))
            lines.setThickness(POLE_THICKNESS)
            for i in range(len(all_posts)-1):
                lines.moveTo(all_posts[i][0], all_posts[i][1], z_end)
                lines.drawTo(all_posts[i+1][0], all_posts[i+1][1], z_end)
            app.render.attachNewNode(lines.create())
        if all_posts:
            lines = LineSegs()
            lines.setColor(LColor(*color))
            lines.setThickness(POLE_THICKNESS)
            for i in range(len(all_posts)-1):
                lines.moveTo(all_posts[i][0], all_posts[i][1], z_end-VERT_LINE_DIST)
                lines.drawTo(all_posts[i+1][0], all_posts[i+1][1], z_end-VERT_LINE_DIST)
            app.render.attachNewNode(lines.create())

def load_shop_positions(xml_file):
    tree_pois = sumolib.xml.parse(xml_file, 'poi')
    tree_positions = []
    for poi in tree_pois:
        x = float(poi.x)
        y = float(poi.y)
        tree_positions.append([x, y])
    return tree_positions

def spawn_shops(app, shop_positions):
    # Load shop model
    building1 = app.loader.loadModel('../data/3d_models/buildings/10065_Corner Grocery Store_V2_L3.obj')
    # Get the original bounding box
    min_point, max_point = building1.getTightBounds()
    if min_point is None or max_point is None:
        print("Could not determine model bounds.")
        return [], None
    # Calculate original width and depth
    original_width = max_point[0] - min_point[0]  # X axis
    original_depth = max_point[1] - min_point[1]  # Y axis
    # Calculate scale so the width becomes 8 units
    target_width = 8.0
    scale_factor = target_width / original_width
    # Calculate scaled depth
    scaled_depth = original_depth * scale_factor
    # Generate shops
    shop_instances = []
    for position in shop_positions:
        x, y = position
        shop_instance = building1.copyTo(app.render)
        shop_instance.setScale(scale_factor)
        shop_instance.setPos(x, y, 0)
        shop_instance.setHpr(0, 0, 0)
        shop_instances.append(shop_instance)
    return shop_instances, scaled_depth

def spawn_homes(app, home_positions):
    building3 = app.loader.loadModel('../data/3d_models/buildings/10084_Small Home_V3_Iteration0.obj')
    # generate trees
    tree_instances = []
    for position in home_positions:
        x = position[0]
        y = position[1]
        # select a tree randomly
        tree_instance = building3.copyTo(app.render)
        # select a scale randomly
        tree_instance.setScale(0.01)
        r = np.abs(0.05*np.random.random())
        tree_instance.setColor(1-r,1-r,1,1)
        # place tree
        tree_instance.setPos(x, y, 0)
        tree_instance.setHpr(0, 0, 0)
        tree_instances.append(tree_instance)
        
def spawn_blocks(app, home_positions):
    building3 = app.loader.loadModel('../data/3d_models/buildings/Residential Buildings 002.obj')
    # generate trees
    tree_instances = []
    for position in home_positions:
        x = position[0]
        y = position[1]
        # select a tree randomly
        tree_instance = building3.copyTo(app.render)
        # select a scale randomly
        # tree_instance.setScale(0.01)
        # r = np.abs(0.1*np.random.random())
        tree_instance.setColor(0.3+0.7*np.random.random(), 
                               0.3+0.7*np.random.random(), 
                               0.3+0.7*np.random.random(),
                               1)
        # place tree
        tree_instance.setPos(x, y, 0)
        tree_instance.setHpr(90, 90, 0)
        tree_instances.append(tree_instance)
        
def load_car_models(app):
    car_collection = app.loader.loadModel('../data/3d_models/cars/Low Poly Cars.glb')
    car_models = [car_collection.find("**/"+str(n)) for n in range(1,10+1)]
    return car_models

def make_box(width, height, depth, lbl, color=(0, 0, 0, 1)):
    frm = GeomVertexFormat.getV3n3c4()
    vdata = GeomVertexData(lbl, frm, Geom.UHStatic)
    vertex = GeomVertexWriter(vdata, 'vertex')
    normal = GeomVertexWriter(vdata, 'normal')
    color_writer = GeomVertexWriter(vdata, 'color')
    # Define the 8 corners of the box
    w2, h2, d2 = width/2, height/2, depth/2
    vertices = [
        (-w2, -h2, -d2), (w2, -h2, -d2), (w2, h2, -d2), (-w2, h2, -d2),  # Front face corners
        (-w2, -h2, d2), (w2, -h2, d2), (w2, h2, d2), (-w2, h2, d2)        # Back face corners
    ]
    # Add vertices and normals (simplified here)
    for pos in vertices:
        vertex.addData3f(pos[0], pos[1], pos[2])
        normal.addData3f(0, 0, 1)  # All normals same for simplicity (not correct, but fast)
    for _ in range(8):
        color_writer.addData4f(*color)
    # Indices for 6 faces (each face has 2 triangles)
    tris = GeomTriangles(Geom.UHStatic)
    # Front face
    tris.addVertices(0, 1, 2); tris.addVertices(0, 2, 3)
    # Back face
    tris.addVertices(4, 7, 6); tris.addVertices(4, 6, 5)
    # Right face
    tris.addVertices(1, 5, 6); tris.addVertices(1, 6, 2)
    # Left face
    tris.addVertices(0, 3, 7); tris.addVertices(0, 7, 4)
    # Top face
    tris.addVertices(3, 2, 6); tris.addVertices(3, 6, 7)
    # Bottom face
    tris.addVertices(0, 4, 5); tris.addVertices(0, 5, 1)
    geom = Geom(vdata)
    geom.addPrimitive(tris)
    node = GeomNode(lbl)
    node.addGeom(geom)
    return node

def make_billboarded_rectangle(app, x, y, z, width, height, color=(0, 0, 0, 1)):
    cm = CardMaker("billboarded_rect")
    cm.setFrame(-width/2, width/2, -height/2, height/2)
    rect = app.render.attachNewNode(cm.generate())
    rect.setPos(x, y, z)
    rect.setColor(*color)
    rect.setBillboardPointEye()  # Always faces camera

def make_billboarded_text(app, x, y, z, text, color=(1, 1, 1, 1)):
    # Create a TextNode
    text_node = TextNode('billboarded_text')
    text_node.setText(text)
    text_node.setTextColor(*color)
    consolas_font = FontPool.load_font("consola.ttf")
    text_node.setFont(consolas_font)
    # Attach to a node
    node = app.render.attachNewNode(text_node)
    node.setPos(x, y, z)
    node.setBillboardPointEye()  # Always faces camera
    node.setScale(0.6)  # Adjust scale as needed
    return node, text_node

def draw_traffic_light(app, x, y, z=0, pole_height=2.0, signal ="yellow", timer=5):
    if DESIGN==0 or DESIGN==3: # ## RAMP METERING SIMPLE
        THREE_HEAD = False
        COUNTDOWN_TIMER = False 
    elif DESIGN==1: # ############# THREE HEADED
        THREE_HEAD = True
        COUNTDOWN_TIMER = False 
    elif DESIGN==2: # ############# COUNTDONW_TIMER
        THREE_HEAD = False
        COUNTDOWN_TIMER = True
    # 1. Draw a vertical line as a pole
    pole_thickness = 10
    lines = LineSegs()
    lines.setColor(LColor(0.1, 0.1, 0.1, 1))  # Dark gray for pole
    lines.setThickness(pole_thickness)
    lines.moveTo(x, y, z)
    lines.drawTo(x, y, z + pole_height)
    app.render.attachNewNode(lines.create())
    # 2. Draw a black box (housing) for the traffic light
    box_width = 0.5
    box_height = 0.5
    box_depth = 1.5
    box_z = z + pole_height  # Place box on top of pole
    box_node = app.render.attachNewNode(
        make_box(box_width, box_height, box_depth, "black_box")
    )
    box_node.setPos(x, y, box_z + box_depth/2)
    # 3. Draw three colored circles (lights) inside the box
    full_color = 1.0
    dark_color = 0.2
    if signal=="green":
        color_green = full_color
        color_red = dark_color
        color_yellow = dark_color
    elif signal=="red":
        color_green = dark_color
        color_red = full_color
        color_yellow = dark_color
    elif signal=="yellow":
        if THREE_HEAD:
            color_green = dark_color
            color_red = full_color
            color_yellow = full_color
        else:
            color_green = dark_color
            color_red = full_color
            color_yellow = dark_color
    elif signal=="yellow2":
        if THREE_HEAD:
            color_green = dark_color
            color_red = dark_color
            color_yellow = full_color
        else:
            color_green = dark_color
            color_red = full_color
            color_yellow = dark_color
    color_green = full_color
    color_red = full_color
    color_yellow = full_color
    if THREE_HEAD:
        box_node1 = app.render.attachNewNode(make_box(0.3, 0.3, 0.3, "red_box", color=(color_red,0,0,1)))
        box_node1.setPos(x, y, z + pole_height + box_depth/2 + 0.05 + 0.3 + 0.2)
        box_node2 = app.render.attachNewNode(make_box(0.3, 0.3, 0.3, "yellow_box", color=(color_yellow,color_yellow,0,1)))
        box_node2.setPos(x, y, z + pole_height + box_depth/2 + 0.05)
        box_node3 = app.render.attachNewNode(make_box(0.3, 0.3, 0.3, "green_box", color=(0,color_green,0,1)))
        box_node3.setPos(x, y, z + pole_height + box_depth/2 + 0.05 - 0.3 - 0.2)
    else:
        box_node1 = app.render.attachNewNode(make_box(0.3, 0.3, 0.3, "red_box", color=(color_red,0,0,1)))
        box_node1.setPos(x, y, z + pole_height + box_depth/2 + 0.05 + 0.25)
        box_node2 = app.render.attachNewNode(make_box(0.3, 0.3, 0.3, "green_box", color=(0,color_green,0,1)))
        box_node2.setPos(x, y, z + pole_height + box_depth/2 + 0.05 - 0.25)
        box_node3 = None
    if COUNTDOWN_TIMER and timer!=0:
        # Draw black rectangle
        make_billboarded_rectangle(
            app, x + box_width/2 + 1.4, y, z + pole_height + box_depth/2, 0.8, 0.8
        )
        # Draw white text (timer)
        _, text_node = make_billboarded_text(
            app, x + box_width/2 + 2.2, y, z + pole_height + box_depth/2 - 0.15, "{:02d}".format(timer)
        )
    else:
        text_node = None
    return box_node1, box_node2, box_node3, text_node
    
def update_traffic_light(signal, timer, box_node1, box_node2, box_node3, text_node):
    # Determine Design
    if DESIGN==0 or DESIGN==3: # ## RAMP METERING SIMPLE
        THREE_HEAD = False
        COUNTDOWN_TIMER = False 
    elif DESIGN==1: # ############# THREE HEADED
        THREE_HEAD = True
        COUNTDOWN_TIMER = False 
    elif DESIGN==2: # ############# COUNTDONW_TIMER
        THREE_HEAD = False
        COUNTDOWN_TIMER = True
    # Set Traffic Light
    full_color = 1.0
    dark_color = 0.2
    if signal=="green":
        color_green = full_color
        color_red = dark_color
        color_yellow = dark_color
    elif signal=="red":
        color_green = dark_color
        color_red = full_color
        color_yellow = dark_color
    elif signal=="yellow":
        if THREE_HEAD:
            color_green = dark_color
            color_red = full_color
            color_yellow = full_color
        else:
            color_green = dark_color
            color_red = full_color
            color_yellow = dark_color
    elif signal=="yellow2":
        if THREE_HEAD:
            color_green = dark_color
            color_red = dark_color
            color_yellow = full_color
        else:
            color_green = dark_color
            color_red = full_color
            color_yellow = dark_color
    if THREE_HEAD:
        box_node1.setColor(color_red,0,0,1)
        box_node2.setColor(color_yellow,color_yellow,0,1)
        box_node3.setColor(0,color_green,0,1)
    else:
        box_node1.setColor(color_red,0,0,1)
        box_node2.setColor(0,color_green,0,1)
    if COUNTDOWN_TIMER:
        if timer!=0:
            text = "{:02d}".format(int(timer+1))
        else:
            text = ""
        text_node.setText(text)
        
def convert_to_25Hz(df_simulation_log_light):
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

def add_yellow_transition(df_simulation_log_light):
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

def add_countdown_timer(df):
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

def draw_white_signal_line(app, p1, p2, p3, p4, color, z=0.03):
    # Convert to numpy arrays
    pA = np.asarray([p1, p2])
    pB = np.asarray([p3, p4])
    # Compute lane length
    lane_length = np.linalg.norm(pA - pB)
    # Compute angle in degrees
    dx = pB[0] - pA[0]
    dy = pB[1] - pA[1]
    angle_deg = np.degrees(np.arctan2(dy, dx)) - 90
    # Compute center
    center_x = (pA[0] + pB[0]) / 2
    center_y = (pA[1] + pB[1]) / 2
    # Make road (card)
    cm_road = CardMaker("card")
    cm_road.setFrame(-SEP_LINE_WIDTH/2, SEP_LINE_WIDTH/2, -lane_length/2, lane_length/2)
    road = app.render.attachNewNode(cm_road.generate())
    road.setPos(center_x, center_y, z)
    road.setHpr(angle_deg, -90, 0)
    road.setColor(LColor(*color))
    
    



# #############################################################################
# # PARAMETERS
# #############################################################################
INFINITY = 50000
LANE_WIDTH = 3.2
SEP_LINE_WIDTH = 0.2
CONCRETE_COLOR = (0.2, 0.2, 0.2, 1)
SEPERATOR_COLOR = (1.0, 1.0, 1.0, 1)
FENCE_COLOR = (0.3, 0.3, 0.3, 1)
VIEWER_HEIGHT = 1.5
SHOW_OTHERS = True
VIDEO_CURRENT_POINT = 8025 #5000
VIDEO_FPS = 25
VIDEO_WIDTH = 1440
VIDEO_HEIGHT = 900
VIDEO_FILE = "output_video.avi"
RECORD_VIDEO = True

# DESIGN TYPES: 0 = SIMPLE, 1 = 3-head, 2 = COUNTDOWN_TIMER, 3 = SIMPLE 

# TRAFFIC LIGHT POSITIONS
    # ENTRANCE RAMP 1
TRAFFIC_LIGHT_POS_X = 20217.08-0.0
TRAFFIC_LIGHT_POS_Y = 18261.92+0.0
TRAFFIC_LIGHT_STOP_LINE_A = 20224.71
TRAFFIC_LIGHT_STOP_LINE_B = 18264.87
TRAFFIC_LIGHT_STOP_LINE_C = 20226.77
TRAFFIC_LIGHT_STOP_LINE_D = 18261.15
    # ENTRANCE RAMP 2
# TRAFFIC_LIGHT_POS_X = 19315.13-0.0
# TRAFFIC_LIGHT_POS_Y = 17822.42+4.5

# SIMULATION TYPES "NO_CONTROL", "45_ALINEA", "60_ALINEA"



###############################################################################
###############################################################################
# ######### DESIGN 0 ##########################################################
###############################################################################
###############################################################################

# VID 1
VIDEO_FILE = "output_video_1.avi"
DESIGN = 0
RAMP_METERING = False
SIMULATION = "NO_CONTROL"
ego_individual = "sample_flow_E2_A3.32"
RENDER_FROM = 3194.089
RENDER_TO = 3324.308
RENDER_FROM = 3119.178096
RENDER_TO = 3379.616172



# #############################################################################
# # LOAD DATA
# #############################################################################           

# Determine trajectory
df_simulation_log_cars = pd.read_csv("../examples/barcelona_simulation/vehicle_pos_log_collection/xx_vehicle_pos_log_file_60_ALINEA.zip")
# df_simulation_log_cars = pd.read_csv("../examples/barcelona_simulation/vehicle_pos_log_collection/xx_vehicle_pos_log_file_"+SIMULATION+".csv")
df_ego_trajectory, simulation_from_time, simulation_to_time = getVehicleTrajectoryRaw(df_simulation_log_cars, ego_individual)
df_ego_trajectory["angle"] = [normalize_angle(angle) for angle in df_ego_trajectory["angle"]]
df_ego_smoothed = interpolateTrajectory(df_ego_trajectory)
df_ego_smoothed["veh_id"] = ego_individual
del df_ego_smoothed["time"]
df_ego_smoothed = df_ego_smoothed.reset_index()

# Determine Video cat
VIDEO_CURRENT_POINT = (df_ego_smoothed['time'] - int(RENDER_FROM)).abs().idxmin()
VIDEO_FINAL_POINT = (df_ego_smoothed['time'] - int(RENDER_TO)).abs().idxmin()

# Determine Trafficlight
df_simulation_log_light = pd.read_csv("../examples/barcelona_simulation/vehicle_pos_log_collection/xx_vehicle_pos_log_file_"+SIMULATION+"_lights.csv")
df_simulation_log_light = convert_to_25Hz(df_simulation_log_light)
df_simulation_log_light = df_simulation_log_light[df_simulation_log_light["time"]<=df_ego_smoothed["time"].iloc[-1]+0.001]
df_simulation_log_light = df_simulation_log_light[df_simulation_log_light["time"]>=df_ego_smoothed["time"].iloc[0]-0.001]
df_simulation_log_light = add_yellow_transition(df_simulation_log_light)
df_simulation_log_light['state'] = df_simulation_log_light['state'].replace({'G': 'green', 'y': 'yellow', 'y2': 'yellow2', 'r': 'red'})
df_simulation_log_light = add_countdown_timer(df_simulation_log_light)

# Convert to list of tuples for easier access
trajectory_points = df_ego_smoothed[["veh_id", 'pos_x', 'pos_y', 'computed_angle_deg', 'time']].values
signal_points = df_simulation_log_light[["time", "state", "timer"]].values
            
# Get smoothed trajectory of all vehicles
if SHOW_OTHERS:
    smoothened_trajectory_data = load_smoothened_trajectories(df_simulation_log_cars, ego_individual, simulation_from_time, simulation_to_time)


# RENDER_FROM



# sys.exit(0)


# points_all_x = []
# points_all_y = []

# for df in smoothened_trajectory_data:
#     df2 = df[df["time"]>RENDER_FROM]
#     if len(df2)>0:
#         df2 = df2.iloc[0]
#         points_all_x.append(df2["pos_x"])
#         points_all_y.append(df2["pos_y"])
    
# ego_pos = []
# df2 = df_ego_smoothed[df_ego_smoothed["time"]>RENDER_FROM]
# df2 = df2.iloc[0]

# dfX = df_simulation_log_cars[df_simulation_log_cars["time"]==3119]
    
# import matplotlib.pyplot as plt
# plt.scatter(points_all_x, points_all_y, color="gray")
# plt.scatter(df2["pos_x"], df2["pos_y"], color="cyan")
# plt.scatter(dfX["pos_x"], dfX["pos_y"], color="blue")


# sys.exit(0)


# import matplotlib.pyplot as plt

# idx = 1000
# plt.subplot(3,1,1)
# plt.title("posx")
# plt.plot(smoothened_trajectory_data[idx]["pos_x"], smoothened_trajectory_data[idx]["pos_y"])
# # plt.subplot(3,1,2)
# # plt.title("posy")
# # plt.plot(smoothened_trajectory_data[idx]["pos_y"])
# plt.subplot(3,1,3)
# plt.title("angle")
# plt.plot(smoothened_trajectory_data[idx]["angle"])
# plt.plot(smoothened_trajectory_data[idx]["computed_angle_deg"])

# import sys
# sys.exit(0)

# Load additional elements
tree_positions = load_tree_positions("../examples/barcelona_simulation/viz_object_positions/trees.add.xml")
fence_lines = load_fence_lines("../examples/barcelona_simulation/viz_object_positions/fences.add.xml")
shop_positions = load_shop_positions("../examples/barcelona_simulation/viz_object_positions/buildings_shops.add.xml")
homes_positions = load_shop_positions("../examples/barcelona_simulation/viz_object_positions/buildings_homes.add.xml")
block_positions = load_shop_positions("../examples/barcelona_simulation/viz_object_positions/buildings_blocks.add.xml")




# #############################################################################
# # MAIN
# #############################################################################

# Create Video Writer
if RECORD_VIDEO:
    video_writer = cv2.VideoWriter(VIDEO_FILE,
                                   cv2.VideoWriter_fourcc(*'MJPG'),
                                   VIDEO_FPS,
                                   (VIDEO_WIDTH, VIDEO_HEIGHT), True)

# Create APP
loadPrcFileData('', 'win-size '+str(VIDEO_WIDTH)+' '+str(VIDEO_HEIGHT))
loadPrcFileData('', 'framebuffer-multisample 1')
loadPrcFileData('', 'multisamples 8')
# loadPrcFileData('', 'load-file-type p3assimp')
app = ShowBase()

# Settings
addCameraControlKeyboard(app)

# LIGHT
create_light(app)

# Draw World
    # SkyBox
createSkyBox(app)
    # GrassFloor
createGrassFloor(app)
    # roads / sumo network
draw_sumo_network(app, '../examples/barcelona_simulation/Network_2Sided.net.xml')
    # trees
tree_instances = spawn_trees(app, tree_positions)
    # highways fences
draw_vertical_barrier(app, fence_lines, FENCE_COLOR, z_start=0.03, z_end=1.0, spacing=2.0)
    # buildings
spawn_shops(app, shop_positions)
spawn_homes(app, homes_positions)
spawn_blocks(app, block_positions)
    # other cars collection
car_models = load_car_models(app)
others_car_instances = {}
    # ego car
ego_car = app.loader.loadModel('../data/3d_models/cars/Car.glb')
ego_car.reparentTo(app.render)
ego_car.setPos(LANE_WIDTH/2, 25, 0)
ego_car.setHpr(180, 90, 0)
    # traffic light
if RAMP_METERING:
    box_node1, box_node2, box_node3, text_node = draw_traffic_light(app, TRAFFIC_LIGHT_POS_X, TRAFFIC_LIGHT_POS_Y, 0)
    draw_white_signal_line(app, TRAFFIC_LIGHT_STOP_LINE_A, TRAFFIC_LIGHT_STOP_LINE_B, TRAFFIC_LIGHT_STOP_LINE_C, TRAFFIC_LIGHT_STOP_LINE_D, color=SEPERATOR_COLOR, z=0.03)

# 20224.71,18264.87
# 20226.77,18261.15

# INITIAL CAMERA POSITION
app.camera.setPos(df_ego_smoothed["pos_x"].iloc[VIDEO_CURRENT_POINT], df_ego_smoothed["pos_y"].iloc[VIDEO_CURRENT_POINT], VIEWER_HEIGHT)
app.camera.setHpr(120, 0, 0)

# # Schedule the task to run every 1 second
app.taskMgr.doMethodLater(0.0, update_scene_world, 'update_scene_world')
app.run()

if RECORD_VIDEO:
    video_writer.release()