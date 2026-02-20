"""sumo3Dviz: A three-dimensional traffic visualisation [2026]
Authors: Kevin Riehl <kriehl@ethz.ch>, Julius Schlapbach <juliussc@ethz.ch>
Organisation: ETH Zürich, Institute for Transport Planning and Systems (IVT)
"""

# This script renders a 3D visualization of a SUMO simulation scenario using
# the configuration parameters in the corresponding configuration script.
#
# Example command:
# python src/sumo3Dviz/cli/render_cli.py --config examples/config_barcelona.yaml

import os
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
    # ! Get configuration parameters from specified file
    # region
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        type=str,
        required=True,
        help="Path to the configuration file containing all necessary parameters to run the video generation pipeline.",
    )
    parser.add_argument(
        "--mode",
        type=str,
        choices=["lagrangian", "cinematic", "eulerian", "cinematic"],
        default="lagrangian",
        help="The mode in which the visualization should be rendered. Options are: 'lagrangian', 'cinematic', 'eulerian', 'cinematic'. Default is 'lagrangian'.",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="If set, the rendering will be done in headless mode without opening a window.",
    )
    args = parser.parse_args()
    config_path = args.config
    headless = args.headless

    # load the configuration from the specified YAML file
    config = load_configuration(config_path)

    # validate the configuration against the schema
    validate_configuration(config)
    # endregion

    # ! LOADER FUNCTIONS
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
        trajectory_file=os.path.join(os.getcwd(), config["paths"]["trajectory_file"]),
        ego_identifier=config["paths"]["ego_identifier"],
        simtime_start=config["paths"]["simtime_start"],
        simtime_end=config["paths"]["simtime_end"],
        video_fps=config["video_parameters"]["video_fps"],
        show_other_vehicles=config["visualization"]["show_other_vehicles"],
    )

    # add veh_id column to df_ego_smoothed for compatibility
    df_ego_smoothed["veh_id"] = config["paths"]["ego_identifier"]

    # load traffic light signal
    df_traffic_light = loader.load_traffic_light_signals(
        traffic_signal_states_file=config["traffic_signals"][
            "traffic_signal_states_file"
        ],
        start_time=df_ego_smoothed["time"].iloc[0] - 0.001,
        end_time=df_ego_smoothed["time"].iloc[-1] + 0.001,
        traffic_light_id=config["traffic_signals"]["traffic_light_id"],
        video_fps=config["video_parameters"]["video_fps"],
    )

    # load the tree positions
    tree_positions = loader.load_tree_positions(
        xml_file=config["visualization"]["tree_positions_file"]
    )

    # load the fence lines
    fence_lines = loader.load_fence_lines(
        xml_file=config["visualization"]["fence_lines_file"]
    )

    # load shop, home, block positions
    # (using the same function as for shops for simplicity)
    shop_positions = loader.load_shop_positions(
        xml_file=config["visualization"]["shops_positions_file"]
    )
    homes_positions = loader.load_shop_positions(
        xml_file=config["visualization"]["homes_positions_file"]
    )
    block_positions = loader.load_shop_positions(
        xml_file=config["visualization"]["blocks_positions_file"]
    )
    # endregion

    # ! RENDERING CALLS
    # region
    # if the video should be stored, initialize the video writer
    if config["video_parameters"]["record_video"]:
        video_writer = cv2.VideoWriter(
            filename=config["video_parameters"]["output_file"],
            fourcc=cv2.VideoWriter.fourcc(*"MJPG"),
            fps=config["video_parameters"]["video_fps"],
            frameSize=(
                config["video_parameters"]["video_width_px"],
                config["video_parameters"]["video_height_px"],
            ),
            isColor=True,
        )
    else:
        video_writer = None

    # create 3D rendering context
    if headless:
        loadPrcFileData("", "window-type offscreen")
    loadPrcFileData(
        "",
        "win-size "
        + str(config["video_parameters"]["video_width_px"])
        + " "
        + str(config["video_parameters"]["video_height_px"]),
    )
    loadPrcFileData("", "framebuffer-multisample 1")
    context = ShowBase()
    context.render.setAntialias(AntialiasAttrib.MAuto)
    fbprobs = FrameBufferProperties()
    fbprobs.setMultisamples(8)

    # add camera control
    interaction_tools.add_camera_control_interactive_mode(context)

    # add the road network
    rendering_tools.create_road_network(
        context=context,
        sumo_network_file=os.path.join(
            os.getcwd(), config["paths"]["sumo_network_file"]
        ),
        lane_width=config["paths"]["lane_width"],
        sep_line_width=config["paths"]["sep_line_width"],
    )  # roads / sumo network

    # add scenery elements
    # light source (otherwise all will be dark)
    rendering_tools.create_light(context=context)

    # skybox / skydome
    rendering_tools.create_sky(
        context=context, sky_texture=config["visualization"]["sky_texture"]
    )

    # grass floor
    rendering_tools.create_ground(
        context=context, ground_texture=config["visualization"]["ground_texture"]
    )

    # trees
    rendering_tools.create_trees(
        context=context,
        tree_positions=tree_positions,
    )

    # highway fences
    rendering_tools.create_highway_fences(context=context, fence_lines=fence_lines)

    # buildings: shops, homes, blocks
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
    ego_car = loader.load_ego_car_model(context=context)
    ego_car.setPos(config["paths"]["lane_width"] / 2, 25, 0)
    ego_car.setHpr(180, 90, 0)

    # traffic light and ramp metering
    if config["traffic_signals"]["ramp_metering"]:
        box_node1, box_node2, box_node3, text_node = (
            rendering_tools.create_traffic_light(
                context=context,
                design=config["traffic_signals"]["design"],
                x=config["traffic_signals"]["ramp"]["pos_x"],
                y=config["traffic_signals"]["ramp"]["pos_y"],
                z=0,
            )
        )

        rendering_tools.create_white_signal_line(
            context=context,
            p1=config["traffic_signals"]["ramp"]["stop_line_a"],
            p2=config["traffic_signals"]["ramp"]["stop_line_b"],
            p3=config["traffic_signals"]["ramp"]["stop_line_c"],
            p4=config["traffic_signals"]["ramp"]["stop_line_d"],
            sep_line_width=config["paths"]["sep_line_width"],
        )
    else:
        box_node1, box_node2, box_node3, text_node = None, None, None, None

    # set the initial camera position
    cast(Camera, context.camera).setPos(
        df_ego_smoothed["pos_x"].iloc[video_start_idx],
        df_ego_smoothed["pos_y"].iloc[video_start_idx],
        config["visualization"]["viewer_height"],
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
        viewer_height=config["visualization"]["viewer_height"],
        show_other_vehicles=config["visualization"]["show_other_vehicles"],
        ramp_metering=config["traffic_signals"]["ramp_metering"],
        design=config["traffic_signals"]["design"],
        video_width_px=config["video_parameters"]["video_width_px"],
        video_height_px=config["video_parameters"]["video_height_px"],
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
    if config["video_parameters"]["record_video"] and video_writer is not None:
        video_writer.release()
    # endregion


if __name__ == "__main__":
    main()
