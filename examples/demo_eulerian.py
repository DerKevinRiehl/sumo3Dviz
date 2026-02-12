"""sumo3Dviz: A three-dimensional traffic visualisation [2026]
Authors: Kevin Riehl <kriehl@ethz.ch>, Julius Schlapbach <jschlapbach@ethz.ch>
Organisation: ETH Zürich, Institute for Transport Planning and Systems (IVT)
"""

# This script renders a 3D visualization of a SUMO simulation in the "Eulerian mode"
# showing the static perspective from a fixed camera position.
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
)


################ METHODS
def _setup_panda_3d(configuration):
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
    return context, fbprobs


def _setup_video_writer(configuration):
    if configuration["rendering"]["record_video"]:
        video_writer = cv2.VideoWriter(
            filename=configuration["paths"]["output_file"],
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
    return video_writer


def _load_data(configuration):
    loader = LoaderTools()
    trajectories = _load_trajectories(loader, configuration)
    positions = _load_positions(loader, configuration)
    return trajectories, positions, loader


def _load_trajectories(loader, configuration):
    print("###############################")
    print("  ==>  Loading Trajectories")
    print("###############################")
    (
        ego_trajectory,
        trajectory_data,
        video_start_idx,
        video_end_idx,
    ) = loader.load_trajectory(
        trajectory_file=configuration["paths"]["trajectory_file"],
        ego_identifier=configuration["eulerian"]["ego_identifier"],
        simtime_start=configuration["eulerian"]["simtime_start"],
        simtime_end=configuration["eulerian"]["simtime_end"],
        video_fps=configuration["rendering"]["video_fps"],
        show_other_vehicles=configuration["visualization"]["show_other_vehicles"],
    )
    ego_trajectory["veh_id"] = configuration["eulerian"]["ego_identifier"]
    # load traffic light signal
    signal_states = loader.load_traffic_light_signals(
        traffic_signal_states_file=configuration["paths"]["traffic_signal_states_file"],
        start_time=ego_trajectory["time"].iloc[0] - 0.001,
        end_time=ego_trajectory["time"].iloc[-1] + 0.001,
        traffic_light_id=configuration["signals"]["traffic_light_id"],
        video_fps=configuration["rendering"]["video_fps"],
    )
    print("###############################")
    return {
        "ego_trajectory": ego_trajectory,
        "trajectory_data": trajectory_data,
        "video_start_idx": video_start_idx,
        "video_end_idx": video_end_idx,
        "signal_states": signal_states,
    }


def _load_positions(loader, configuration):
    print("###############################")
    print("  ==>  Loading Resources")
    print("###############################")
    tree_positions = loader.load_tree_positions(
        xml_file=configuration["paths"]["tree_positions_file"]
    )
    fence_lines = loader.load_fence_lines(
        xml_file=configuration["paths"]["fence_lines_file"]
    )
    shop_positions = loader.load_shop_positions(
        xml_file=configuration["paths"]["shops_positions_file"]
    )
    home_positions = loader.load_shop_positions(
        xml_file=configuration["paths"]["homes_positions_file"]
    )
    block_positions = loader.load_shop_positions(
        xml_file=configuration["paths"]["blocks_positions_file"]
    )
    print("###############################\n")
    return {
        "tree": tree_positions,
        "fence": fence_lines,
        "shop": shop_positions,
        "home": home_positions,
        "block": block_positions,
    }


def _render_world_scene(context, loader, configuration, positions, trajectories):
    print("###############################")
    print("  ==>  Rendering World")
    print("###############################")
    rendering_tools = RenderingTools()
    # camera
    cast(Camera, context.camera).setPos(
        trajectories["ego_trajectory"]["pos_x"].iloc[trajectories["video_start_idx"]],
        trajectories["ego_trajectory"]["pos_y"].iloc[trajectories["video_start_idx"]],
        configuration["visualization"]["viewer_height"],
    )
    cast(Camera, context.camera).setHpr(120, 0, 0)
    # light
    rendering_tools.create_light(context=context)
    # sky and ground
    rendering_tools.create_sky(
        context=context, sky_texture=configuration["visualization"]["sky_texture"]
    )
    rendering_tools.create_ground(
        context=context, ground_texture=configuration["visualization"]["ground_texture"]
    )
    # road network
    rendering_tools.create_road_network(
        context=context,
        sumo_network_file=configuration["paths"]["sumo_network_file"],
        lane_width=configuration["visualization"]["lane_width"],
        sep_line_width=configuration["visualization"]["sep_line_width"],
    )
    # trees
    rendering_tools.create_trees(
        context=context,
        tree_positions=positions["tree"],
        tree_scale_1=configuration["visualization"]["tree_scale_1"],
        tree_scale_2=configuration["visualization"]["tree_scale_2"],
        tree_size_variability=configuration["visualization"]["tree_size_variability"],
        tree_color_variability=configuration["visualization"]["tree_color_variability"],
    )
    # fences
    rendering_tools.create_highway_fences(
        context=context,
        fence_lines=positions["fence"],
    )
    # place buildings
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
    # cars
    car_models = loader.load_car_models(context=context)
    ego_car = loader.load_ego_car_model(context=context)
    ego_car.setPos(configuration["visualization"]["lane_width"] / 2, 25, 0)
    ego_car.setHpr(180, 90, 0)
    # signals
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
    print("###############################\n")
    cars = {"ego_car": ego_car, "car_models": car_models}
    signals = {
        "box_node1": box_node1,
        "box_node2": box_node2,
        "box_node3": box_node3,
        "text_node": text_node,
    }
    return cars, signals, rendering_tools


def _launch_eulerian_mode(
    context,
    configuration,
    trajectory_data,
    car_instances,
    rendering_tools,
    video_writer,
    signal_instances,
):
    simulation_manager = SimulationManager(
        context=context,
        configuration=configuration,
        trajectory_data=trajectory_data,
        car_instances=car_instances,
        rendering_tools=rendering_tools,
        video_writer=video_writer,
        signal_instances=signal_instances,
        camera_position=configuration["eulerian"]["camera_position"],
        show_other_vehicles_simple=True,
    )
    context.taskMgr.doMethodLater(0.0, simulation_manager.update_world, "update_world")
    context.run()


def _terminate_eulerian_mode(configuration, video_writer):
    # once completed, release the video writer
    if configuration["rendering"]["record_video"] and video_writer is not None:
        video_writer.release()


################ CONFIGURATION
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
        "output_file": os.path.join(
            os.path.dirname(__file__), "barcelona_simulation_eulerian.avi"
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
}

################ MAIN
if __name__ == "__main__":
    trajectory_data, positions, loader = _load_data(configuration)
    video_writer = _setup_video_writer(configuration)
    context, fbprobs = _setup_panda_3d(configuration)
    car_instances, signal_instances, rendering_tools = _render_world_scene(
        context, loader, configuration, positions, trajectory_data
    )
    _launch_eulerian_mode(
        context,
        configuration,
        trajectory_data,
        car_instances,
        rendering_tools,
        video_writer,
        signal_instances,
    )
    _terminate_eulerian_mode(configuration, video_writer)
