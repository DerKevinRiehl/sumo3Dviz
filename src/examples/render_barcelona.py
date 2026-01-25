# This script renders a 3D visualization of a SUMO simulation scenario in Barcelona
# using the available classes directly and configuring them through corresponding
# properties. For an illustration on how to use this library through the CLI,
# please refer to the render_barcelona_cli.py script and the corresponding
# configuration file / command line arguments described there.
import os
import cv2
import platform
from typing import cast
from direct.showbase.ShowBase import ShowBase
from panda3d.core import (
    Filename,
    Camera,
    loadPrcFileData,
    AntialiasAttrib,
    FrameBufferProperties,
    get_model_path,
)

from src.tools.loader_tools import LoaderTools
from src.tools.interaction_tools import InteractionTools
from src.tools.rendering_tools import RenderingTools
from src.tools.trajectory_tools import TrajectoryTools
from src.tools.simulation_tools import SimulationManager

if __name__ == "__main__":
    # ! CONFIGURATION PARAMETERS (passed directly to classes / functions)
    # ? alternative config-based approach in `render_barcelona_cli.py`
    # region
    # global parameters
    headless = False

    # trajectory parameters
    trajectory_file = os.path.join(
        os.path.dirname(__file__),
        "../../examples/barcelona_simulation/simulation_logs/vehicle_positions.xml",
    )
    ego_identifier = "flow_0_car_aggr1_route_E3_AEnd_lane0.0"
    sumo_network_file = os.path.join(
        os.path.dirname(__file__),
        "../../examples/barcelona_simulation/Network.net.xml",
    )
    lane_width = 3.2
    sep_line_width = 0.2
    simtime_start = 39.0
    simtime_end = 369.0

    # video parameters
    record_video = True
    video_fps = 25
    video_width_px = 1140
    video_height_px = 900
    output_file = os.path.join(
        os.path.dirname(__file__), "../../results/barcelona_simulation.avi"
    )

    # traffic signals
    ramp_metering = False
    design = 0  # DESIGN TYPES: 0 = SIMPLE, 1 = 3-head, 2 = COUNTDOWN_TIMER, 3 = SIMPLE
    traffic_light_positions = {
        "ramp_1": {
            "pos_x": 20217.08 - 0.0,
            "pos_y": 18261.92 + 0.0,
            "stop_line_a": 20224.71,
            "stop_line_b": 18264.87,
            "stop_line_c": 20226.77,
            "stop_line_d": 18261.15,
        },
        "ramp_2": {
            "pos_x": 19315.13 - 0.0,
            "pos_y": 17822.42 + 4.5,
            "stop_line_a": 20224.71,
            "stop_line_b": 18264.87,
            "stop_line_c": 20226.77,
            "stop_line_d": 18261.15,
        },
    }

    traffic_light_id = "JE3"
    tree_positions_file = os.path.join(
        os.path.dirname(__file__),
        "../../examples/barcelona_simulation/viz_object_positions/trees.add.xml",
    )

    # visualization parameters
    show_other_vehicles = True
    viewer_height = 1.5
    traffic_signal_states_file = os.path.join(
        os.path.dirname(__file__),
        "../../examples/barcelona_simulation/simulation_logs/signal_states.xml",
    )
    fence_lines_file = os.path.join(
        os.path.dirname(__file__),
        "../../examples/barcelona_simulation/viz_object_positions/fences.add.xml",
    )
    shops_positions_file = os.path.join(
        os.path.dirname(__file__),
        "../../examples/barcelona_simulation/viz_object_positions/buildings_shops.add.xml",
    )
    homes_positions_file = os.path.join(
        os.path.dirname(__file__),
        "../../examples/barcelona_simulation/viz_object_positions/buildings_homes.add.xml",
    )
    blocks_positions_file = os.path.join(
        os.path.dirname(__file__),
        "../../examples/barcelona_simulation/viz_object_positions/buildings_blocks.add.xml",
    )

    if platform.system() == "Windows":
        get_model_path().append_directory(Filename("data"))
        low_poly_cars_file = "3d_models/cars/Low Poly Cars.glb"
        car_file = "3d_models/cars/Car.glb"
        sky_texture_file = "images/texture_sky_blue.jpg"
        floor_texture_file = "images/texture_ground_grass.jpg"
        store_model_file = "3d_models/buildings/10065_Corner Grocery Store_V2_L3.obj"
        home_model_file = "3d_models/buildings/10084_Small Home_V3_Iteration0.obj"
        block_model_file = "3d_models/buildings/Residential Buildings 002.obj"
        tree_model_file_1 = "3d_models/trees/MapleTree.obj"
        tree_model_file_2 = "3d_models/trees/Hazelnut.obj"
    else:
        low_poly_cars_file = os.path.join(
            os.path.dirname(__file__), "../../data/3d_models/cars/Low Poly Cars.glb"
        )
        car_file = os.path.join(
            os.path.dirname(__file__), "../../data/3d_models/cars/Car.glb"
        )
        sky_texture_file = os.path.join(
            os.path.dirname(__file__), "../../data/images/texture_sky_daycloud1.jpg"
        )
        floor_texture_file = os.path.join(
            os.path.dirname(__file__),
            "../../data/images/texture_ground_grass.jpg",
        )
        store_model_file = os.path.join(
            os.path.dirname(__file__),
            "../../data/3d_models/buildings/10065_Corner Grocery Store_V2_L3.obj",
        )
        home_model_file = os.path.join(
            os.path.dirname(__file__),
            "../../data/3d_models/buildings/10084_Small Home_V3_Iteration0.obj",
        )
        block_model_file = os.path.join(
            os.path.dirname(__file__),
            "../../data/3d_models/buildings/Residential Buildings 002.obj",
        )
        tree_model_file_1 = os.path.join(
            os.path.dirname(__file__), "../../data/3d_models/trees/MapleTree.obj"
        )
        tree_model_file_2 = os.path.join(
            os.path.dirname(__file__), "../../data/3d_models/trees/Hazelnut.obj"
        )

    tree_scale_1 = 0.2
    tree_scale_2 = 0.5
    tree_size_variability = 1
    tree_color_variability = 0.1
    # endregion

    # ! LOADER CALLS
    # region
    loader = LoaderTools()
    interaction_tools = InteractionTools()
    rendering_tools = RenderingTools()
    trajectory_tools = TrajectoryTools()

    # load the trajectories from the SUMO log and apply trajectory smoothing to them
    (
        df_ego_smoothed,
        smoothened_trajectory_data,
        video_start_idx,
        video_end_idx,
    ) = loader.load_trajectory(
        trajectory_file=trajectory_file,
        ego_identifier=ego_identifier,
        simtime_start=simtime_start,
        simtime_end=simtime_end,
        video_fps=video_fps,
        show_other_vehicles=show_other_vehicles,
    )

    # add veh_id column to df_ego_smoothed for compatibility
    df_ego_smoothed["veh_id"] = ego_identifier

    # load traffic light signal
    df_traffic_light = loader.load_traffic_light_signals(
        traffic_signal_states_file=traffic_signal_states_file,
        start_time=df_ego_smoothed["time"].iloc[0] - 0.001,
        end_time=df_ego_smoothed["time"].iloc[-1] + 0.001,
        traffic_light_id=traffic_light_id,
        video_fps=video_fps,
    )

    # load the tree positions
    tree_positions = loader.load_tree_positions(xml_file=tree_positions_file)

    # load the fence lines
    fence_lines = loader.load_fence_lines(xml_file=fence_lines_file)

    # load shop, home, block positions
    # (using the same function as for shops for simplicity)
    shop_positions = loader.load_shop_positions(xml_file=shops_positions_file)
    homes_positions = loader.load_shop_positions(xml_file=homes_positions_file)
    block_positions = loader.load_shop_positions(xml_file=blocks_positions_file)
    # endregion

    # ! RENDERING CALLS
    # region
    # if the video should be stored, initialize the video writer
    if record_video:
        video_writer = cv2.VideoWriter(
            filename=output_file,
            fourcc=cv2.VideoWriter.fourcc(*"MJPG"),
            fps=video_fps,
            frameSize=(video_width_px, video_height_px),
            isColor=True,
        )
    else:
        video_writer = None

    # create 3D rendering context
    if headless:
        loadPrcFileData("", "window-type offscreen")
    loadPrcFileData(
        "",
        "win-size " + str(video_width_px) + " " + str(video_height_px),
    )
    loadPrcFileData("", "framebuffer-multisample 1")
    context = ShowBase()
    context.render.setAntialias(AntialiasAttrib.MAuto)
    fbprobs = FrameBufferProperties()
    fbprobs.setMultisamples(8)

    # add camera control
    interaction_tools.addCameraControlKeyboard(context)

    # add the road network
    rendering_tools.create_road_network(
        context=context,
        sumo_network_file=sumo_network_file,
        lane_width=lane_width,
        sep_line_width=sep_line_width,
    )  # roads / sumo network

    # add scenery elements
    rendering_tools.create_light(
        context=context
    )  # light source (otherwise all will be dark)
    rendering_tools.create_sky(
        context=context, sky_texture_file=sky_texture_file
    )  # skybox / skydome
    rendering_tools.create_floor(
        context=context, path_floor_texture=floor_texture_file
    )  # grass floor
    rendering_tools.create_trees(
        context=context,
        tree_positions=tree_positions,
        tree_model_file_1=tree_model_file_1,
        tree_model_file_2=tree_model_file_2,
        tree_scale_1=tree_scale_1,
        tree_scale_2=tree_scale_2,
        tree_size_variability=tree_size_variability,
        tree_color_variability=tree_color_variability,
    )  # trees
    rendering_tools.create_highway_fences(
        context=context, fence_lines=fence_lines
    )  # highways fences
    rendering_tools.create_building_shops(
        context=context,
        shop_positions=shop_positions,
        store_model_file=store_model_file,
    )  # shops
    rendering_tools.create_building_homes(
        context=context,
        homes_positions=homes_positions,
        home_model_file=home_model_file,
    )  # homes
    rendering_tools.create_building_blocks(
        context=context,
        block_positions=block_positions,
        block_model_file=block_model_file,
    )  # blocks

    # load cars and ego vehicle car
    car_models = loader.load_car_models(
        context=context, low_poly_cars_file=low_poly_cars_file
    )
    ego_car = loader.load_ego_car_model(context=context, car_file=car_file)
    ego_car.setPos(lane_width / 2, 25, 0)
    ego_car.setHpr(180, 90, 0)

    # traffic light and ramp metering
    if ramp_metering:
        box_node1, box_node2, box_node3, text_node = (
            rendering_tools.create_traffic_light(
                context=context,
                design=design,
                x=traffic_light_positions["ramp_1"]["pos_x"],
                y=traffic_light_positions["ramp_1"]["pos_y"],
                z=0,
            )
        )

        rendering_tools.create_white_signal_line(
            context=context,
            p1=traffic_light_positions["ramp_1"]["stop_line_a"],
            p2=traffic_light_positions["ramp_1"]["stop_line_b"],
            p3=traffic_light_positions["ramp_1"]["stop_line_c"],
            p4=traffic_light_positions["ramp_1"]["stop_line_d"],
            sep_line_width=sep_line_width,
        )
    else:
        box_node1, box_node2, box_node3, text_node = None, None, None, None

    # set the initial camera position
    cast(Camera, context.camera).setPos(
        df_ego_smoothed["pos_x"].iloc[video_start_idx],
        df_ego_smoothed["pos_y"].iloc[video_start_idx],
        viewer_height,
    )
    cast(Camera, context.camera).setHpr(120, 0, 0)
    # endregion

    # ! RUN SIMULATION / RENDERING LOOP
    # region
    # convert list of tuples for easier access
    trajectory_points = df_ego_smoothed[
        ["veh_id", "pos_x", "pos_y", "computed_angle_deg", "time"]
    ].values
    signal_points = (
        df_traffic_light[["time", "state", "timer"]].values
        if df_traffic_light is not None
        else None
    )

    # create the simulation manager
    simulation_manager = SimulationManager(
        context=context,
        trajectory_points=trajectory_points,
        signal_points=signal_points,
        video_start_idx=video_start_idx,
        video_end_idx=video_end_idx,
        car_models=car_models,
        ego_car=ego_car,
        smoothened_trajectory_data=smoothened_trajectory_data,
        trajectory_tools=trajectory_tools,
        rendering_tools=rendering_tools,
        viewer_height=viewer_height,
        show_other_vehicles=show_other_vehicles,
        ramp_metering=ramp_metering,
        design=design,
        video_width_px=video_width_px,
        video_height_px=video_height_px,
        video_writer=video_writer,
        box_node1=box_node1,
        box_node2=box_node2,
        box_node3=box_node3,
        text_node=text_node,
    )

    # assign the update function to the task manager
    context.taskMgr.doMethodLater(0.0, simulation_manager.update_world, "update_world")
    context.run()

    # once completed, release the video writer
    if record_video and video_writer is not None:
        video_writer.release()
    # endregion
