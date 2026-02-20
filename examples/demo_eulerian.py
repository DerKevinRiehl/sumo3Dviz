"""sumo3Dviz: A three-dimensional traffic visualisation (Eulerian demo)
Authors: Kevin Riehl <kriehl@ethz.ch>, Julius Schlapbach <juliussc@ethz.ch>
Organisation: ETH Zürich, Institute for Transport Planning and Systems (IVT)

This example demonstrates how to render a short Eulerian-mode video
with a fixed camera position. It uses the same configuration structure
as the CLI pipeline.

Usage (from repo root):
    python examples/demo_eulerian.py
"""

# This script renders a 3D visualization of a SUMO simulation in the "Eulerian" mode
# (camera is fixed at a specific location).
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
    RenderingTools,
    SimulationManager,
    validate_configuration,
)


# Configuration dictionary
# - Mirrors the minimal fields expected by the demo helpers and
#   the `SimulationManager`. Grouping follows: `rendering`,
#   `visualization`, `paths`, `signals`, and `modes`.
# - If you prefer, replace this dictionary by loading a yaml file using
#   the CLI helper `load_configuration(...)` and then validate it with
#   `validate_configuration(...)`.
configuration = {
    "rendering": {
        "video_width_px": 1140,
        "video_height_px": 900,
        "record_video": True,
        "video_fps": 25,
        "headless": False,
    },
    "visualization": {
        "lane_width": 3.2,
        "sep_line_width": 0.2,
        "tree_scale_1": 0.2,
        "tree_scale_2": 0.5,
        "tree_size_variability": 1,
        "tree_color_variability": 0.1,
        "sky_texture": "sky_cloudy",
        "ground_texture": "ground_grass",
        "show_other_vehicles": True,
        "viewer_height": 1.5,
        "show_signals": True,
    },
    "paths": {
        "trajectory_file": os.path.join(
            os.path.dirname(__file__),
            "barcelona_simulation/simulation_logs/vehicle_positions.xml",
        ),
        "sumo_network_file": os.path.join(
            os.path.dirname(__file__),
            "barcelona_simulation/Network.net.xml",
        ),
        "traffic_signal_states_file": os.path.join(
            os.path.dirname(__file__),
            "barcelona_simulation/simulation_logs/signal_states.xml",
        ),
        "fence_lines_file": os.path.join(
            os.path.dirname(__file__),
            "barcelona_simulation/viz_object_positions/fences.add.xml",
        ),
        "shops_positions_file": os.path.join(
            os.path.dirname(__file__),
            "barcelona_simulation/viz_object_positions/buildings_shops.add.xml",
        ),
        "homes_positions_file": os.path.join(
            os.path.dirname(__file__),
            "barcelona_simulation/viz_object_positions/buildings_homes.add.xml",
        ),
        "blocks_positions_file": os.path.join(
            os.path.dirname(__file__),
            "barcelona_simulation/viz_object_positions/buildings_blocks.add.xml",
        ),
        "tree_positions_file": os.path.join(
            os.path.dirname(__file__),
            "barcelona_simulation/viz_object_positions/trees.add.xml",
        ),
    },
    "signals": {
        "signal_design": "three_headed",  # DESIGN TYPES: simple, three_headed, countdown_timer
        "traffic_light_positions": {
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
        },
        "traffic_light_id": "JE3",
    },
    "modes": {
        "eulerian": {
            "simtime_start": 39.0,
            "simtime_end": 369.0,
            "ego_identifier": "flow_0_car_aggr1_route_E3_AEnd_lane0.0",
            "camera_position": {
                "pos_x": 19357.74,
                "pos_y": 17808.70,
                "pos_z": 11.95,
                "ori_h": 90.0,
                "ori_p": -10.0,
                "ori_r": 0.0,
            },
        },
    },
}

# Specification of the output file
output_file = os.path.join(
    os.path.dirname(__file__), "barcelona_simulation_eulerian.avi"
)

if __name__ == "__main__":
    # ! STEP 1: Validate the configuration file
    # region
    # alternatively, you may also load a yaml configuration file (as for the CLI-version)
    # using the helper function `load_configuration` before validation
    validate_configuration(configuration, mode="eulerian")
    # endregion

    # ! STEP 2: Load the data (trajectories, positions of static objects, etc.) using the helper functions
    # region
    loader = LoaderTools()
    print(50 * "#")
    print("  ==>  Loading Trajectories")
    print(50 * "#")
    (
        ego_trajectory,
        trajectory_data,
        video_start_idx,
        video_end_idx,
    ) = loader.load_trajectory(
        trajectory_file=configuration["paths"]["trajectory_file"],
        ego_identifier=configuration["modes"]["eulerian"]["ego_identifier"],
        simtime_start=configuration["modes"]["eulerian"]["simtime_start"],
        simtime_end=configuration["modes"]["eulerian"]["simtime_end"],
        video_fps=configuration["rendering"]["video_fps"],
        show_other_vehicles=configuration["visualization"]["show_other_vehicles"],
    )
    ego_trajectory["veh_id"] = configuration["modes"]["eulerian"]["ego_identifier"]

    # load traffic light signal
    signal_states = loader.load_traffic_light_signals(
        traffic_signal_states_file=configuration["paths"]["traffic_signal_states_file"],
        start_time=ego_trajectory["time"].iloc[0] - 0.001,
        end_time=ego_trajectory["time"].iloc[-1] + 0.001,
        traffic_light_id=configuration["signals"]["traffic_light_id"],
        video_fps=configuration["rendering"]["video_fps"],
    )
    print(50 * "#")
    trajectories = {
        "ego_trajectory": ego_trajectory,
        "trajectory_data": trajectory_data,
        "video_start_idx": video_start_idx,
        "video_end_idx": video_end_idx,
        "signal_states": signal_states,
    }

    print(50 * "#")
    print("  ==>  Loading Resources")
    print(50 * "#")
    # load the tree positions from the provided XML file (used to place
    # decorative tree models in the scene)
    tree_positions = loader.load_tree_positions(
        xml_file=configuration["paths"]["tree_positions_file"]
    )
    # load the fence line definitions (highway barriers / guard rails)
    fence_lines = loader.load_fence_lines(
        xml_file=configuration["paths"]["fence_lines_file"]
    )
    # load shop positions (building placements for commercial structures)
    shop_positions = loader.load_shop_positions(
        xml_file=configuration["paths"]["shops_positions_file"]
    )
    # load home positions (residential building placements)
    home_positions = loader.load_shop_positions(
        xml_file=configuration["paths"]["homes_positions_file"]
    )
    # load block positions (larger building blocks / apartment clusters)
    block_positions = loader.load_shop_positions(
        xml_file=configuration["paths"]["blocks_positions_file"]
    )
    print(50 * "#")
    positions = {
        "tree": tree_positions,
        "fence": fence_lines,
        "shop": shop_positions,
        "home": home_positions,
        "block": block_positions,
    }
    # endregion

    # ! STEP 3: Set up Panda3D and the video writer (if video recording is enabled in the configuration)
    # region
    if configuration["rendering"]["record_video"]:
        video_writer = cv2.VideoWriter(
            filename=output_file,
            fourcc=cv2.VideoWriter.fourcc(*"MJPG"),
            fps=configuration["rendering"]["video_fps"],
            frameSize=(
                configuration["rendering"]["video_width_px"],
                configuration["rendering"]["video_height_px"],
            ),
            isColor=True,
        )
    else:
        video_writer = None

    if configuration["rendering"]["headless"]:
        loadPrcFileData("", "window-type offscreen")

    loadPrcFileData(
        "",
        "win-size "
        + str(configuration["rendering"]["video_width_px"])
        + " "
        + str(configuration["rendering"]["video_height_px"]),
    )
    loadPrcFileData("", "framebuffer-multisample 1")
    context = ShowBase()
    context.render.setAntialias(AntialiasAttrib.MAuto)
    fbprobs = FrameBufferProperties()
    fbprobs.setMultisamples(8)
    # endregion

    # ! STEP 4: Create the world scene (road, lane markings, trees, sky, etc.) using the rendering tools and the loaded data
    # region
    print(50 * "#")
    print("  ==>  Rendering World")
    print(50 * "#")
    rendering_tools = RenderingTools()

    # set initial camera position to ego vehicle start pose
    cast(Camera, context.camera).setPos(
        trajectories["ego_trajectory"]["pos_x"].iloc[trajectories["video_start_idx"]],
        trajectories["ego_trajectory"]["pos_y"].iloc[trajectories["video_start_idx"]],
        configuration["visualization"]["viewer_height"],
    )
    cast(Camera, context.camera).setHpr(120, 0, 0)
    # create a light source to ensure scene objects are visible
    rendering_tools.create_light(context=context)

    # create skybox / skydome and ground plane with configured textures
    rendering_tools.create_sky(
        context=context,
        sky_texture=configuration["visualization"]["sky_texture"],
    )
    rendering_tools.create_ground(
        context=context,
        ground_texture=configuration["visualization"]["ground_texture"],
    )

    # create the road network from the SUMO network file (lanes and markings)
    rendering_tools.create_road_network(
        context=context,
        sumo_network_file=configuration["paths"]["sumo_network_file"],
        lane_width=configuration["visualization"]["lane_width"],
        sep_line_width=configuration["visualization"]["sep_line_width"],
    )

    # instantiate tree models at the loaded tree positions
    rendering_tools.create_trees(
        context=context,
        tree_positions=positions["tree"],
        tree_scale_1=configuration["visualization"]["tree_scale_1"],
        tree_scale_2=configuration["visualization"]["tree_scale_2"],
        tree_size_variability=configuration["visualization"]["tree_size_variability"],
        tree_color_variability=configuration["visualization"]["tree_color_variability"],
    )

    # create highway fences / guard rails using the fence line definitions
    rendering_tools.create_highway_fences(
        context=context,
        fence_lines=positions["fence"],
    )

    # create building models (shops, homes, blocks) at their positions
    rendering_tools.create_building_shops(
        context=context,
        shop_positions=positions["shop"],
    )
    rendering_tools.create_building_homes(
        context=context,
        homes_positions=positions["home"],
    )
    rendering_tools.create_building_blocks(
        context=context,
        block_positions=positions["block"],
    )

    # load car models and place the ego vehicle model in the scene
    car_models = loader.load_car_models(context=context)
    ego_car = loader.load_ego_car_model(context=context)
    ego_car.setPos(configuration["visualization"]["lane_width"] / 2, 25, 0)
    ego_car.setHpr(180, 90, 0)

    # create traffic light models and auxiliary stop lines when enabled
    if configuration["visualization"]["show_signals"]:
        box_node1, box_node2, box_node3, text_node = (
            rendering_tools.create_traffic_light(
                context=context,
                design=configuration["signals"]["signal_design"],
                x=configuration["signals"]["traffic_light_positions"]["ramp_1"][
                    "pos_x"
                ],
                y=configuration["signals"]["traffic_light_positions"]["ramp_1"][
                    "pos_y"
                ],
                z=0,
            )
        )
        rendering_tools.create_white_signal_line(
            context=context,
            p1=configuration["signals"]["traffic_light_positions"]["ramp_1"][
                "stop_line_a"
            ],
            p2=configuration["signals"]["traffic_light_positions"]["ramp_1"][
                "stop_line_b"
            ],
            p3=configuration["signals"]["traffic_light_positions"]["ramp_1"][
                "stop_line_c"
            ],
            p4=configuration["signals"]["traffic_light_positions"]["ramp_1"][
                "stop_line_d"
            ],
            sep_line_width=configuration["visualization"]["sep_line_width"],
        )
    else:
        box_node1, box_node2, box_node3, text_node = None, None, None, None
    print(50 * "#")
    cars = {"ego_car": ego_car, "car_models": car_models}
    signals = {
        "box_node1": box_node1,
        "box_node2": box_node2,
        "box_node3": box_node3,
        "text_node": text_node,
    }
    # endregion

    # ! STEP 5: Launch the simulation in Eulerian mode.
    # The `SimulationManager` encapsulates the update loop: at each frame it
    # advances the ego and other vehicle poses, updates traffic light
    # visual states, and writes frames to `video_writer` when recording is
    # enabled. We hand the manager the pre-built scene and trajectory data
    # and let it drive the Panda3D task manager.
    # region
    simulation_manager = SimulationManager(
        context=context,
        configuration=configuration,
        trajectory_data=trajectories,
        car_instances=cars,
        rendering_tools=rendering_tools,
        video_writer=video_writer,
        signal_instances=signals,
        camera_position=configuration["modes"]["eulerian"]["camera_position"],
        show_other_vehicles_simple=True,
    )
    context.taskMgr.doMethodLater(0.0, simulation_manager.update_world, "update_world")
    context.run()

    # once completed, release the video writer
    if configuration["rendering"]["record_video"] and video_writer is not None:
        video_writer.release()
    # endregion
