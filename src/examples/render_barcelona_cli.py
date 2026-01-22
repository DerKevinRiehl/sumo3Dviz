# This script renders a 3D visualization of a SUMO simulation scenario in Barcelona
# using the configuration parameters in the corresponding configuration script.
# In order for the script to work correctly, please specify the configuration script
# path and name in the command line arguments.
#
# Command:
# python src/examples/render_barcelona_cli.py --config src/examples/config_barcelona.yaml

import os
import cv2
import argparse
import yaml
from typing import cast
from direct.showbase.ShowBase import ShowBase
from panda3d.core import loadPrcFileData, AntialiasAttrib, FrameBufferProperties, Camera

from src.tools.loader_tools import LoaderTools
from src.tools.interaction_tools import InteractionTools
from src.tools.rendering_tools import RenderingTools


if __name__ == "__main__":
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
        "--headless",
        action="store_true",
        help="If set, the rendering will be done in headless mode without opening a window.",
    )
    args = parser.parse_args()
    config_path = args.config
    headless = args.headless

    if not config_path:
        raise ValueError(
            "Configuration file path must be specified using --config argument."
        )

    try:
        # use the full loader to get the yaml file content as a python dictionary
        with open(config_path) as file:
            config = yaml.load(file, Loader=yaml.FullLoader)
            assert type(config) is dict

    except:
        print(
            f"Could not load the configuration file, please make sure it exists at: {config_path}"
        )
        exit(1)
    # endregion

    # ! LOADER FUNCTIONS
    # region
    loader = LoaderTools()
    interaction_tools = InteractionTools()
    rendering_tools = RenderingTools()

    # load the trajectories from the SUMO log and apply trajectory smoothing to them
    (
        df_ego_smoothed,
        smoothened_trajectory_data,
        video_start_idx,
        video_end_idx,
    ) = loader.load_trajectory(
        trajectory_file=os.path.join(
            os.getcwd(), config["trajectory_parameters"]["trajectory_file"]
        ),
        ego_identifier=config["trajectory_parameters"]["ego_identifier"],
        simtime_start=config["trajectory_parameters"]["simtime_start"],
        simtime_end=config["trajectory_parameters"]["simtime_end"],
        video_fps=config["video_parameters"]["video_fps"],
        show_other_vehicles=config["visualization_parameters"]["show_other_vehicles"],
    )

    # load traffic light signal
    df_simulation_log_light = loader.load_traffic_light_signals(
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
        xml_file=config["visualization_parameters"]["tree_positions_file"]
    )

    # load the fence lines
    fence_lines = loader.load_fence_lines(
        xml_file=config["visualization_parameters"]["fence_lines_file"]
    )

    # load shop, home, block positions
    # (using the same function as for shops for simplicity)
    shop_positions = loader.load_shop_positions(
        xml_file=config["visualization_parameters"]["shops_positions_file"]
    )
    homes_positions = loader.load_shop_positions(
        xml_file=config["visualization_parameters"]["homes_positions_file"]
    )
    block_positions = loader.load_shop_positions(
        xml_file=config["visualization_parameters"]["blocks_positions_file"]
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
                config["video_parameters"]["video_heigth_px"],
            ),
            isColor=True,
        )

    # create 3D rendering context
    if headless:
        loadPrcFileData("", "window-type offscreen")
    loadPrcFileData(
        "",
        "win-size "
        + str(config["video_parameters"]["video_width_px"])
        + " "
        + str(config["video_parameters"]["video_heigth_px"]),
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
        sumo_network_file=os.path.join(
            os.getcwd(), config["trajectory_parameters"]["sumo_network_file"]
        ),
        lane_width=config["trajectory_parameters"]["lane_width"],
        sep_line_width=config["trajectory_parameters"]["sep_line_width"],
    )  # roads / sumo network

    # add scenery elements
    rendering_tools.create_light(
        context=context
    )  # light source (otherwise all will be dark)
    rendering_tools.create_sky(
        context=context,
        sky_texture_file=os.path.join(
            os.getcwd(), config["visualization_parameters"]["sky_texture_file"]
        ),
    )  # skybox / skydome
    rendering_tools.create_floor(
        context=context,
        path_floor_texture=os.path.join(
            os.getcwd(), config["visualization_parameters"]["floor_texture_file"]
        ),
    )  # grass floor
    rendering_tools.create_trees(
        context=context,
        tree_positions=tree_positions,
        tree_model_file_1=os.path.join(
            os.getcwd(), config["visualization_parameters"]["tree_model_file_1"]
        ),
        tree_model_file_2=os.path.join(
            os.getcwd(), config["visualization_parameters"]["tree_model_file_2"]
        ),
        tree_scale_1=config["visualization_parameters"]["tree_scale_1"],
        tree_scale_2=config["visualization_parameters"]["tree_scale_2"],
        tree_size_variability=config["visualization_parameters"][
            "tree_size_variability"
        ],
        tree_color_variability=config["visualization_parameters"][
            "tree_color_variability"
        ],
    )  # trees
    rendering_tools.create_highway_fences(
        context=context, fence_lines=fence_lines
    )  # highways fences
    rendering_tools.create_building_shops(
        context=context,
        shop_positions=shop_positions,
        store_model_file=os.path.join(
            os.getcwd(), config["visualization_parameters"]["store_model_file"]
        ),
    )  # shops
    rendering_tools.create_building_homes(
        context=context,
        homes_positions=homes_positions,
        home_model_file=os.path.join(
            os.getcwd(), config["visualization_parameters"]["home_model_file"]
        ),
    )  # homes
    rendering_tools.create_building_blocks(
        context=context,
        block_positions=block_positions,
        block_model_file=os.path.join(
            os.getcwd(), config["visualization_parameters"]["block_model_file"]
        ),
    )  # blocks

    # load cars and ego vehicle car
    car_models = loader.load_car_models(
        context=context,
        low_poly_cars_file=os.path.join(
            os.getcwd(), config["visualization_parameters"]["low_poly_cars_file"]
        ),
    )
    ego_car = loader.load_ego_car_model(
        context=context,
        car_file=os.path.join(
            os.getcwd(), config["visualization_parameters"]["car_file"]
        ),
    )
    ego_car.setPos(config["trajectory_parameters"]["lane_width"] / 2, 25, 0)
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
            sep_line_width=config["trajectory_parameters"]["sep_line_width"],
        )

    # set the initial camera position
    cast(Camera, context.camera).setPos(
        df_ego_smoothed["pos_x"].iloc[video_start_idx],
        df_ego_smoothed["pos_y"].iloc[video_start_idx],
        config["visualization_parameters"]["viewer_height"],
    )
    cast(Camera, context.camera).setHpr(120, 0, 0)
    # endregion
