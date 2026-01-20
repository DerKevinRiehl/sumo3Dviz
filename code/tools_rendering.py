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
import sumolib
import os
from pathlib import Path

# Get current working directory (should be repository root)
REPO_ROOT = Path(os.getcwd())

from panda3d.core import Geom, GeomNode, GeomVertexData, GeomVertexFormat, GeomVertexWriter, GeomTriangles
from panda3d.core import CardMaker, LColor, Filename, TextureStage
from panda3d.core import LineSegs, FontPool, TextNode, Texture
from panda3d.core import DirectionalLight, AmbientLight, Vec4


# #############################################################################
# # PARAMETERS
# #############################################################################
LANE_WIDTH = 3.2
SEP_LINE_WIDTH = 0.2
CONCRETE_COLOR = (0.2, 0.2, 0.2, 1)
SEPERATOR_COLOR = (1.0, 1.0, 1.0, 1)
FENCE_COLOR = (0.3, 0.3, 0.3, 1)


# #############################################################################
# # RENDER METHODS
# #############################################################################

def create_light(context):
    # Ambient light (fill light)
    alight = AmbientLight('alight')
    alight.setColor(Vec4(1.0, 1.0, 1.0, 1))
    alnp = context.render.attachNewNode(alight)
    context.render.setLight(alnp)
    # Directional light (for highlights and shading)
    dlight = DirectionalLight('dlight')
    dlight.setColor(Vec4(1, 1, 1, 1))
    dlnp = context.render.attachNewNode(dlight)
    dlnp.setPos(0, -100, 200)
    dlnp.setHpr(-30, -45, 0)  # pointing downward and forward
    context.render.setLight(dlnp)
    
def create_sky(context, path_sky_texture=str(REPO_ROOT / "data/images/puresky.jpg"), INFINITY = 50000):
    # generate an inverted sphere model
    sky_sphere = context.loader.loadModel("models/smiley") 
    sky_sphere.reparentTo(context.render)
    sky_sphere.setScale(INFINITY) 
    sky_sphere.setTwoSided(True) 
    # Load the texture
    sky_texture = Texture()
    sky_texture.read(Filename(path_sky_texture))
    sky_sphere.setTexture(sky_texture, 1)
    # Set rendering properties
    sky_sphere.setBin('background', 0)
    sky_sphere.setDepthWrite(False)
    sky_sphere.setLightOff() 

def create_floor(context, path_floor_texture=str(REPO_ROOT / "data/images/grass.jpg"), INFINITY = 50000):
    # Grass Floor
    cm_ground = CardMaker("ground")
    cm_ground.setFrame(-INFINITY, INFINITY, -INFINITY, INFINITY)
    ground = context.render.attachNewNode(cm_ground.generate())
    ground.setPos(0, 0, -0.05)
    ground.setHpr(25, -90, 0)
    # Load the texture
    ground_tex = context.loader.loadTexture(path_floor_texture)
    ground_tex.setWrapU(Texture.WM_repeat)
    ground_tex.setWrapV(Texture.WM_repeat)
    ground.setTexture(ground_tex)
    ground.setTexScale(TextureStage.getDefault(), 40000, 40000)
    
def create_trees(app, tree_positions):
    # Load tree models
    tree1_model = app.loader.loadModel(str(REPO_ROOT / 'data/3d_models/trees/MapleTree.obj'))
    tree2_model = app.loader.loadModel(str(REPO_ROOT / 'data/3d_models/trees/Hazelnut.obj'))
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

def create_building_shops(app, shop_positions):
    # Load shop model
    building1 = app.loader.loadModel(str(REPO_ROOT / 'data/3d_models/buildings/10065_Corner Grocery Store_V2_L3.obj'))
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

def create_building_homes(app, home_positions):
    building3 = app.loader.loadModel(str(REPO_ROOT / 'data/3d_models/buildings/10084_Small Home_V3_Iteration0.obj'))
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
        
def create_building_blocks(app, home_positions):
    building3 = app.loader.loadModel(str(REPO_ROOT / 'data/3d_models/buildings/Residential Buildings 002.obj'))
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
        
        
def create_road_network(context, sumo_network_file):
    # Draw Roads
    net = sumolib.net.readNet(sumo_network_file)

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
            _draw_road_edge_lane(context, lane_shape, lane_l)
    # Draw Junctions
    for junction in net.getNodes():
        junction_shape = junction.getShape()
        if junction_shape:
            _draw_polygon_fan(context, junction_shape)
            
def _draw_road_edge_lane(context, lane_shape, lane_id):
    _draw_concrete(context, lane_shape, CONCRETE_COLOR)
    if lane_id=="b":
        _draw_white_seperator_line_left(context, lane_shape, SEPERATOR_COLOR)
        _draw_white_seperator_line_right(context, lane_shape, SEPERATOR_COLOR)
    if lane_id=="a":
        _draw_white_seperator_line_right(context, lane_shape, SEPERATOR_COLOR)
        _draw_white_separator_line_right_dashed(context, lane_shape, SEPERATOR_COLOR, z=0.03, dash_length=1.0, gap_length=1.0)
    elif lane_id=="e":
        _draw_white_seperator_line_left(context, lane_shape, SEPERATOR_COLOR)
    elif lane_id=="i":
        _draw_white_separator_line_right_dashed(context, lane_shape, SEPERATOR_COLOR, z=0.03, dash_length=1.0, gap_length=1.0)     

def _draw_concrete(context, lane_shape, color, z=0.01):
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
        road = context.render.attachNewNode(cm_road.generate())
        road.setPos(center_x, center_y, z)
        road.setHpr(angle_deg, -90, 0)
        road.setColor(LColor(*color))    

def _draw_white_seperator_line_left(context, lane_shape, color, z=0.03):
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
        road = context.render.attachNewNode(cm_road.generate())
        road.setPos(center_x, center_y, z)
        road.setHpr(angle_deg, -90, 0)
        road.setColor(LColor(*color))  
        
def _draw_white_seperator_line_right(context, lane_shape, color, z=0.03):
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
        road = context.render.attachNewNode(cm_road.generate())
        road.setPos(center_x, center_y, z)
        road.setHpr(angle_deg, -90, 0)
        road.setColor(LColor(*color))  
       
def _draw_white_separator_line_right_dashed(context, lane_shape, color, z=0.03, dash_length=1.0, gap_length=1.0):
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
            dash = context.render.attachNewNode(cm_dash.generate())
            dash.setPos(final_pos[0], final_pos[1], z)
            dash.setHpr(angle_deg, -90, 0)
            dash.setColor(LColor(*color))

def _draw_polygon_fan(context, shape_points, z=0.01):
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
    nodepath = context.render.attachNewNode(node)
    nodepath.setTwoSided(True)

def draw_highway_fences(context, lines, color=FENCE_COLOR, z_start=0.03, z_end=1.0, spacing=2.0):
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
                context.render.attachNewNode(lines.create())
        # Draw two connecting lines through the tops of the posts
        if all_posts:
            lines = LineSegs()
            lines.setColor(LColor(*color))
            lines.setThickness(POLE_THICKNESS)
            for i in range(len(all_posts)-1):
                lines.moveTo(all_posts[i][0], all_posts[i][1], z_end)
                lines.drawTo(all_posts[i+1][0], all_posts[i+1][1], z_end)
            context.render.attachNewNode(lines.create())
        if all_posts:
            lines = LineSegs()
            lines.setColor(LColor(*color))
            lines.setThickness(POLE_THICKNESS)
            for i in range(len(all_posts)-1):
                lines.moveTo(all_posts[i][0], all_posts[i][1], z_end-VERT_LINE_DIST)
                lines.drawTo(all_posts[i+1][0], all_posts[i+1][1], z_end-VERT_LINE_DIST)
            context.render.attachNewNode(lines.create())
            

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

def draw_traffic_light(context, DESIGN, x, y, z=0, pole_height=2.0, signal ="yellow", timer=5):
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
    context.render.attachNewNode(lines.create())
    # 2. Draw a black box (housing) for the traffic light
    box_width = 0.5
    box_height = 0.5
    box_depth = 1.5
    box_z = z + pole_height  # Place box on top of pole
    box_node = context.render.attachNewNode(
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
        box_node1 = context.render.attachNewNode(make_box(0.3, 0.3, 0.3, "red_box", color=(color_red,0,0,1)))
        box_node1.setPos(x, y, z + pole_height + box_depth/2 + 0.05 + 0.3 + 0.2)
        box_node2 = context.render.attachNewNode(make_box(0.3, 0.3, 0.3, "yellow_box", color=(color_yellow,color_yellow,0,1)))
        box_node2.setPos(x, y, z + pole_height + box_depth/2 + 0.05)
        box_node3 = context.render.attachNewNode(make_box(0.3, 0.3, 0.3, "green_box", color=(0,color_green,0,1)))
        box_node3.setPos(x, y, z + pole_height + box_depth/2 + 0.05 - 0.3 - 0.2)
    else:
        box_node1 = context.render.attachNewNode(make_box(0.3, 0.3, 0.3, "red_box", color=(color_red,0,0,1)))
        box_node1.setPos(x, y, z + pole_height + box_depth/2 + 0.05 + 0.25)
        box_node2 = context.render.attachNewNode(make_box(0.3, 0.3, 0.3, "green_box", color=(0,color_green,0,1)))
        box_node2.setPos(x, y, z + pole_height + box_depth/2 + 0.05 - 0.25)
        box_node3 = None
    if COUNTDOWN_TIMER and timer!=0:
        # Draw black rectangle
        make_billboarded_rectangle(
            context, x + box_width/2 + 1.4, y, z + pole_height + box_depth/2, 0.8, 0.8
        )
        # Draw white text (timer)
        _, text_node = make_billboarded_text(
            context, x + box_width/2 + 2.2, y, z + pole_height + box_depth/2 - 0.15, "{:02d}".format(timer)
        )
    else:
        text_node = None
    return box_node1, box_node2, box_node3, text_node
    
def update_traffic_light(signal, DESIGN, timer, box_node1, box_node2, box_node3, text_node):
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
    