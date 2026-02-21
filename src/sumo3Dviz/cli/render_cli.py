"""sumo3Dviz: A three-dimensional traffic visualisation [2026]
Authors: Kevin Riehl <kriehl@ethz.ch>, Julius Schlapbach <juliussc@ethz.ch>
Organisation: ETH Zürich, Institute for Transport Planning and Systems (IVT)
"""

# This script renders a 3D visualization of a SUMO simulation scenario using
# the configuration parameters in the corresponding configuration script.
#
# Example commands (using the entry-point script):
# sumo3Dviz --config examples/config_barcelona.yaml --mode lagrangian --output results/output_lagrangian.avi
# sumo3Dviz --config examples/config_barcelona.yaml --mode eulerian --output results/output_eulerian.avi
# sumo3Dviz --config examples/config_barcelona.yaml --mode cinematic --output results/output_cinematic.avi
# sumo3Dviz --config examples/config_barcelona.yaml --mode interactive
# sumo3Dviz --config examples/config_ci.yaml --mode interactive --headless --max-frames 50  # CI mode

import cv2
import argparse
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
from sumo3Dviz.cli.configuration_helpers import (
    load_configuration,
    validate_configuration,
)


def main():
    # ! STEP 1: Get configuration parameters from specified file and validate
    # region
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        type=str,
        required=True,
        help="Path to the configuration file containing all necessary parameters to run the video generation pipeline.",
    )
    parser.add_argument(
        "--output",
        type=str,
        required=False,
        default=None,
        help="Path to the output video file (.avi) where the rendered visualization will be saved. Required if --mode is not 'interactive'.",
    )
    parser.add_argument(
        "--mode",
        type=str,
        choices=["lagrangian", "eulerian", "cinematic", "interactive"],
        default="lagrangian",
        help="The mode in which the visualization should be rendered. Options are: 'lagrangian', 'eulerian', 'cinematic', 'interactive'. Default is 'lagrangian'.",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="If set, the rendering will be done in headless mode without opening a window.",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        required=False,
        default=None,
        help="Maximum number of frames to render before exiting. Useful for CI testing in interactive mode. If not specified, runs until user closes the window.",
    )
    args = parser.parse_args()
    config_path = args.config
    mode = args.mode
    headless = args.headless
    max_frames = args.max_frames

    # enforce conditional requirement: --output is required for non-interactive modes
    if mode != "interactive" and not args.output:
        parser.error("--output is required when --mode is not 'interactive'.")

    # load the configuration from the specified YAML file
    configuration = load_configuration(config_path)

    # validate the configuration against the schema for the specified mode
    validate_configuration(configuration, mode=mode)
    # endregion

    # ! STEP 2: Load the data (trajectories, positions of static objects, etc.) using the helper functions
    # region
    loader = LoaderTools()

    # Initialize variables for type checker
    trajectories = None
    cars = None
    signals = None

    # Load trajectories and traffic signals (only for non-interactive modes)
    if mode != "interactive":
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
            ego_identifier=configuration["modes"][mode]["ego_identifier"],
            simtime_start=configuration["modes"][mode]["simtime_start"],
            simtime_end=configuration["modes"][mode]["simtime_end"],
            video_fps=configuration["rendering"]["video_fps"],
            show_other_vehicles=configuration["visualization"]["show_other_vehicles"],
        )
        ego_trajectory["veh_id"] = configuration["modes"][mode]["ego_identifier"]

        # load traffic light signal
        signal_states = loader.load_traffic_light_signals(
            traffic_signal_states_file=configuration["paths"][
                "traffic_signal_states_file"
            ],
            start_time=ego_trajectory["time"].iloc[0] - 0.001,
            end_time=ego_trajectory["time"].iloc[-1] + 0.001,
            traffic_light_ids=[
                signal["id"]
                for signal in configuration["signals"]["traffic_light_positions"]
            ],
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
    # Create video writer (only for non-interactive modes)
    if mode != "interactive" and configuration["rendering"].get("record_video", False):
        video_writer = cv2.VideoWriter(
            filename=args.output,
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

    # Setup headless mode if specified via CLI or config
    if headless or configuration["rendering"].get("headless", False):
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

    # set initial camera position based on mode
    if mode == "interactive":
        # set initial camera position from config
        cast(Camera, context.camera).setPos(
            *configuration["modes"]["interactive"]["initial_camera_position"][
                "position"
            ]
        )
        cast(Camera, context.camera).setHpr(
            *configuration["modes"]["interactive"]["initial_camera_position"][
                "orientation"
            ]
        )
    else:
        # set initial camera position to ego vehicle start pose
        # (for lagrangian, eulerian, and cinematic modes)
        assert (
            trajectories is not None
        ), "Trajectories must be loaded for non-interactive modes"
        cast(Camera, context.camera).setPos(
            trajectories["ego_trajectory"]["pos_x"].iloc[
                trajectories["video_start_idx"]
            ],
            trajectories["ego_trajectory"]["pos_y"].iloc[
                trajectories["video_start_idx"]
            ],
            configuration["visualization"]["viewer_height"],
        )
        cast(Camera, context.camera).setHpr(120, 0, 0)

    # setup interactive mode controls and HUD if in interactive mode
    if mode == "interactive":
        # head up display (HUD) to inform user in interactive mode
        rendering_tools.render_hud(context=context)

        # setup interactive camera controls
        interaction_tools = InteractionTools()
        interaction_tools.add_camera_control_interactive_mode(context)

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
        tree_scale_1=configuration["visualization"].get("tree_scale_1", 0.2),
        tree_scale_2=configuration["visualization"].get("tree_scale_2", 0.5),
        tree_size_variability=configuration["visualization"].get(
            "tree_size_variability", 1.0
        ),
        tree_color_variability=configuration["visualization"].get(
            "tree_color_variability", 0.1
        ),
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

    # load car models and place the ego vehicle model in the scene (only for non-interactive modes)
    if mode != "interactive":
        car_models = loader.load_car_models(context=context)
        ego_car = loader.load_ego_car_model(context=context)
        ego_car.setPos(configuration["visualization"]["lane_width"] / 2, 25, 0)
        ego_car.setHpr(180 if mode == "lagrangian" else 0, 90, 0)

        # create traffic light models and auxiliary stop lines when enabled
        signals = []
        if configuration["visualization"]["show_signals"]:
            for signal in configuration["signals"]["traffic_light_positions"]:
                print(f"Creating traffic light for signal ID: {signal['id']}")
                box_node1, box_node2, box_node3, text_node = (
                    rendering_tools.create_traffic_light(
                        context=context,
                        design=configuration["signals"]["signal_design"],
                        x=signal["pos_x"],
                        y=signal["pos_y"],
                        z=0,
                    )
                )
                rendering_tools.create_white_signal_line(
                    context=context,
                    p1=signal["stop_line_a"],
                    p2=signal["stop_line_b"],
                    p3=signal["stop_line_c"],
                    p4=signal["stop_line_d"],
                    sep_line_width=configuration["visualization"]["sep_line_width"],
                )

                signals.append(
                    {
                        "id": signal["id"],
                        "box_node1": box_node1,
                        "box_node2": box_node2,
                        "box_node3": box_node3,
                        "text_node": text_node,
                    }
                )
        print(50 * "#")
        cars = {"ego_car": ego_car, "car_models": car_models}
    # endregion

    # ! STEP 5: Launch the simulation in the specified mode
    # region
    if mode == "interactive":
        # Interactive mode: just run the context with keyboard controls
        print(50 * "#")
        print("  ==>  Launch Interactive Mode")
        print(50 * "#")

        # If max_frames is set (CI mode), add a task to limit the number of frames
        if max_frames is not None:
            frame_count = [0]  # Use list to allow modification in nested function

            def frame_counter_task(task):
                """Task to count frames and exit after max_frames."""
                frame_count[0] += 1
                if frame_count[0] >= max_frames:
                    print(f"\n  ==>  Reached max frames ({max_frames}), exiting...")
                    context.userExit()
                    return task.done
                return task.cont

            # Add the frame counter task
            context.taskMgr.add(frame_counter_task, "frame_counter")
            print(f"  ==>  CI Mode: Will exit after {max_frames} frames")

        context.run()
    elif mode == "lagrangian":
        # Lagrangian mode: camera follows the ego vehicle
        # The `SimulationManager` encapsulates the update loop: at each frame it
        # advances the ego and other vehicle poses, updates traffic light
        # visual states, and writes frames to `video_writer` when recording is
        # enabled. We hand the manager the pre-built scene and trajectory data
        # and let it drive the Panda3D task manager.
        assert (
            trajectories is not None and cars is not None and signals is not None
        ), "Data must be loaded for lagrangian mode"
        simulation_manager = SimulationManager(
            context=context,
            configuration=configuration,
            trajectory_data=trajectories,
            car_instances=cars,
            rendering_tools=rendering_tools,
            mode="lagrangian",
            video_writer=video_writer,
            signal_instances=signals,
        )
        context.taskMgr.doMethodLater(
            0.0, simulation_manager.update_world, "update_world"
        )
        context.run()

        # once completed, release the video writer
        if (
            configuration["rendering"].get("record_video", False)
            and video_writer is not None
        ):
            video_writer.release()
    elif mode == "eulerian":
        # Eulerian mode: camera is fixed at a specific position
        # The `SimulationManager` encapsulates the update loop: at each frame it
        # advances the ego and other vehicle poses, updates traffic light
        # visual states, and writes frames to `video_writer` when recording is
        # enabled. We hand the manager the pre-built scene and trajectory data
        # and let it drive the Panda3D task manager.
        assert (
            trajectories is not None and cars is not None and signals is not None
        ), "Data must be loaded for eulerian mode"
        simulation_manager = SimulationManager(
            context=context,
            configuration=configuration,
            trajectory_data=trajectories,
            car_instances=cars,
            rendering_tools=rendering_tools,
            mode="eulerian",
            video_writer=video_writer,
            signal_instances=signals,
            camera_position=configuration["modes"]["eulerian"]["camera_position"],
            show_other_vehicles_simple=True,
        )
        context.taskMgr.doMethodLater(
            0.0, simulation_manager.update_world, "update_world"
        )
        context.run()

        # once completed, release the video writer
        if (
            configuration["rendering"].get("record_video", False)
            and video_writer is not None
        ):
            video_writer.release()
    elif mode == "cinematic":
        # Cinematic mode: camera follows a smoothened trajectory through keyframes
        # The `SimulationManager` encapsulates the update loop: at each frame it
        # advances the ego and other vehicle poses, updates traffic light
        # visual states, and writes frames to `video_writer` when recording is
        # enabled. We hand the manager the pre-built scene and trajectory data
        # and let it drive the Panda3D task manager.
        assert (
            trajectories is not None and cars is not None and signals is not None
        ), "Data must be loaded for cinematic mode"
        # prepare cinematic camera trajectory
        trajectory_tools = TrajectoryTools()
        smoothened_camera_trajectory = (
            trajectory_tools.generate_camera_cinematic_trajectory(
                camera_position_trajectory=configuration["modes"]["cinematic"][
                    "camera_position_trajectory"
                ],
                simtime_start=configuration["modes"]["cinematic"]["simtime_start"],
                simtime_end=configuration["modes"]["cinematic"]["simtime_end"],
                video_fps=configuration["rendering"]["video_fps"],
            )
        )

        simulation_manager = SimulationManager(
            context=context,
            configuration=configuration,
            trajectory_data=trajectories,
            car_instances=cars,
            rendering_tools=rendering_tools,
            mode="cinematic",
            video_writer=video_writer,
            signal_instances=signals,
            cinematic_camera_trajectory=smoothened_camera_trajectory,
            show_other_vehicles_simple=True,
        )

        # Set camera to correct initial position before rendering starts
        first_camera_frame = smoothened_camera_trajectory.iloc[0]
        cast(Camera, context.camera).setPos(
            first_camera_frame["pos_x"],
            first_camera_frame["pos_y"],
            first_camera_frame["pos_z"],
        )
        # Normalize angles to [0, 360) range for Panda3D
        cast(Camera, context.camera).setHpr(
            first_camera_frame["ori_h"] % 360,
            first_camera_frame["ori_p"] % 360,
            first_camera_frame["ori_r"] % 360,
        )

        context.taskMgr.doMethodLater(
            0.0, simulation_manager.update_world, "update_world"
        )
        context.run()

        # once completed, release the video writer
        if (
            configuration["rendering"].get("record_video", False)
            and video_writer is not None
        ):
            video_writer.release()
    # endregion


if __name__ == "__main__":
    main()
