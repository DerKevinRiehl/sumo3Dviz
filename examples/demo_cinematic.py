"""sumo3Dviz: A three-dimensional traffic visualisation [2026]
Authors: Kevin Riehl <kriehl@ethz.ch>, Julius Schlapbach <jschlapbach@ethz.ch>
Organisation: ETH Zürich, Institute for Transport Planning and Systems (IVT)
"""

# This script renders a 3D visualization of a SUMO simulation scenario in Barcelona
# using the available classes directly and configuring them through corresponding
# properties. For an illustration on how to use this library through the CLI,
# please refer to the render_barcelona_cli.py script and the corresponding
# configuration file / command line arguments described there.
import os
import cv2
from typing import cast
from direct.showbase.ShowBase import ShowBase
from panda3d.core import (
    Camera,
    loadPrcFileData,
    AntialiasAttrib,
    FrameBufferProperties,
)

from sumo3Dviz import (
    LoaderTools,
    InteractionTools,
    RenderingTools,
    TrajectoryTools,
    SimulationManager,
)


if __name__ == "__main__":
    # ! CONFIGURATION PARAMETERS (passed directly to classes / functions)
    # ? alternative config-based approach in `render_barcelona_cli.py`
    # region
    # global parameters
    headless = False

    # trajectory parameters
    trajectory_file = os.path.join(
        os.path.dirname(__file__),
        "barcelona_simulation/simulation_logs/vehicle_positions.xml",
    )
    ego_identifier = "flow_0_car_aggr1_route_E3_AEnd_lane0.0"
    sumo_network_file = os.path.join(
        os.path.dirname(__file__),
        "barcelona_simulation/Network.net.xml",
    )
    lane_width = 3.2
    sep_line_width = 0.2
    simtime_start = 139.0
    simtime_end = 369.0

    # video parameters
    record_video = True
    video_fps = 25
    video_width_px = 1140
    video_height_px = 900
    output_file = os.path.join(
        os.path.dirname(__file__), "barcelona_simulation_cinematic.avi"
    )
    camera_position_trajectory = {
        139.0: {
            "pos_x": 19517.38,
            "pos_y": 17889.49,
            "pos_z": 37.38,
            "ori_h": 50.0,
            "ori_p": -10.0,
            "ori_r": 0.0,
        },
        160.0: {
            "pos_x": 19493.39,
            "pos_y": 17880.76,
            "pos_z": 26.78,
            "ori_h": 110.0,
            "ori_p": -10.0,
            "ori_r": 0.0,
        },
        190.0: {
            "pos_x": 19229.93,
            "pos_y": 17758.21,
            "pos_z": 10.49,
            "ori_h": 100.0,
            "ori_p": 0.0,
            "ori_r": 0.0,
        },
        250.0: {
            "pos_x": 19042.54,
            "pos_y": 17722.10,
            "pos_z": 83.70,
            "ori_h": 100.0,
            "ori_p": -20.0,
            "ori_r": 0.0,
        },
        350.0: {
            "pos_x": 19042.54,
            "pos_y": 17722.10,
            "pos_z": 43.70,
            "ori_h": 100.0,
            "ori_p": -15.0,
            "ori_r": 0.0,
        },
        369.0: {
            "pos_x": 19100.90,
            "pos_y": 17895.91,
            "pos_z": 3.30,
            "ori_h": 0.0,
            "ori_p": 0.0,
            "ori_r": 0.0,
        },
    }
    show_other_vehicles_simple = True

    # traffic signals
    ramp_metering = True
    design = "three_headed"  # DESIGN TYPES: simple, three_headed, countdown_timer
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
        "barcelona_simulation/viz_object_positions/trees.add.xml",
    )

    # visualization parameters
    show_other_vehicles = True
    viewer_height = 1.5
    traffic_signal_states_file = os.path.join(
        os.path.dirname(__file__),
        "barcelona_simulation/simulation_logs/signal_states.xml",
    )
    fence_lines_file = os.path.join(
        os.path.dirname(__file__),
        "barcelona_simulation/viz_object_positions/fences.add.xml",
    )
    shops_positions_file = os.path.join(
        os.path.dirname(__file__),
        "barcelona_simulation/viz_object_positions/buildings_shops.add.xml",
    )
    homes_positions_file = os.path.join(
        os.path.dirname(__file__),
        "barcelona_simulation/viz_object_positions/buildings_homes.add.xml",
    )
    blocks_positions_file = os.path.join(
        os.path.dirname(__file__),
        "barcelona_simulation/viz_object_positions/buildings_blocks.add.xml",
    )

    # tree scaling and variability parameters
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

    # smoothen cinematic camera trajectory
    df_camera_trajectory_smoothed = (
        trajectory_tools.generate_camera_cinematic_trajectory(
            camera_position_trajectory, simtime_start, simtime_end, video_fps
        )
    )

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
    # light source (otherwise all will be dark)
    rendering_tools.create_light(context=context)

    # skybox / skydome
    # choose from one of the pre-defined options (sky_texture parameter)
    # or, optionally, a custom sky texture file can be provided
    # -> in this case, the correct handling of file paths
    #    for Panda3D on Windows needs to be ensured
    rendering_tools.create_sky(context=context, sky_texture="sky_cloudy")

    # surrounding ground
    # choose from one of the pre-defined options (ground_texture parameter)
    # or, optionally, a custom ground texture file can be provided
    # -> in this case, the correct handling of file paths
    #    for Panda3D on Windows needs to be ensured
    rendering_tools.create_ground(context=context, ground_texture="ground_grass")

    # trees
    # optionally, a custom tree object model file can be provided
    # -> in this case, the correct handling of file paths
    #    for Panda3D on Windows needs to be ensured
    rendering_tools.create_trees(
        context=context,
        tree_positions=tree_positions,
        tree_scale_1=tree_scale_1,
        tree_scale_2=tree_scale_2,
        tree_size_variability=tree_size_variability,
        tree_color_variability=tree_color_variability,
    )

    # highway fences
    rendering_tools.create_highway_fences(context=context, fence_lines=fence_lines)

    # buildings: shops, homes, blocks
    # optionally, a custom building model file can be provided
    # -> in this case, the correct handling of file paths
    #    for Panda3D on Windows needs to be ensured
    rendering_tools.create_building_shops(
        context=context,
        shop_positions=shop_positions,
    )
    rendering_tools.create_building_homes(
        context=context,
        homes_positions=homes_positions,
    )
    rendering_tools.create_building_blocks(
        context=context,
        block_positions=block_positions,
    )

    # load cars and ego vehicle car
    car_models = loader.load_car_models(context=context)
    if not show_other_vehicles_simple:
        ego_car = loader.load_ego_car_model(context=context)
        ego_car.setPos(lane_width / 2, 25, 0)
    else:
        ego_car = rendering_tools.generate_simple_car_model(context)
        ego_car.reparentTo(context.render)
        ego_car.setPos(lane_width / 2, 25, 0.2)
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
        cinematic_camera_trajectory=df_camera_trajectory_smoothed,
        show_other_vehicles_simple=show_other_vehicles_simple,
    )

    # assign the update function to the task manager
    context.taskMgr.doMethodLater(0.0, simulation_manager.update_world, "update_world")
    context.run()

    # once completed, release the video writer
    if record_video and video_writer is not None:
        video_writer.release()
    # endregion
