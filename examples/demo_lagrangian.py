"""sumo3Dviz: A three-dimensional traffic visualisation [2026]
Authors: Kevin Riehl <kriehl@ethz.ch>, Julius Schlapbach <jschlapbach@ethz.ch>
Organisation: ETH Zürich, Institute for Transport Planning and Systems (IVT)
"""

# This script renders a 3D visualization of a SUMO simulation in the "Lagrangian mode"
# following the perspective of a car.
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
    TrajectoryTools,
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
        ego_identifier=configuration["lagrangian"]["ego_identifier"],
        simtime_start=configuration["lagrangian"]["simtime_start"],
        simtime_end=configuration["lagrangian"]["simtime_end"],
        video_fps=configuration["rendering"]["video_fps"],
        show_other_vehicles=configuration["visualization"]["show_other_vehicles"],
    )
    ego_trajectory["veh_id"] = configuration["lagrangian"]["ego_identifier"]
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
    rendering_tools.create_sky(context=context, sky_texture="sky_cloudy")
    rendering_tools.create_ground(context=context, ground_texture="ground_grass")
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
            os.path.dirname(__file__), "barcelona_simulation_lagrangian.avi"
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
    "lagrangian": {
        "simtime_start": 39.0,
        "simtime_end": 369.0,
        "ego_identifier": "flow_0_car_aggr1_route_E3_AEnd_lane0.0",
    },
}

################ MAIN
if __name__ == "__main__":
    trajectories, positions, loader = _load_data(configuration)
    video_writer = _setup_video_writer(configuration)
    context, fbprobs = _setup_panda_3d(configuration)
    cars, signals, rendering_tools = _render_world_scene(
        context, loader, configuration, positions, trajectories
    )

    # create the simulation manager
    trajectory_tools = TrajectoryTools()
    trajectory_points = trajectories["ego_trajectory"][
        ["veh_id", "pos_x", "pos_y", "computed_angle_deg", "time"]
    ].values
    signal_points = (
        trajectories["signal_states"][["time", "state", "timer"]].values
        if trajectories["signal_states"] is not None
        else None
    )
    simulation_manager = SimulationManager(
        context=context,
        trajectory_points=trajectory_points,
        signal_points=signal_points,
        video_start_idx=trajectories["video_start_idx"],
        video_end_idx=trajectories["video_end_idx"],
        car_models=cars["car_models"],
        ego_car=cars["ego_car"],
        smoothened_trajectory_data=trajectories["trajectory_data"],
        trajectory_tools=trajectory_tools,
        rendering_tools=rendering_tools,
        viewer_height=configuration["visualization"]["viewer_height"],
        show_other_vehicles=configuration["visualization"]["show_other_vehicles"],
        ramp_metering=configuration["visualization"]["show_signals"],
        design=configuration["signals"]["signal_design"],
        video_width_px=configuration["rendering"]["video_width_px"],
        video_height_px=configuration["rendering"]["video_height_px"],
        video_writer=video_writer,
        box_node1=signals["box_node1"],
        box_node2=signals["box_node2"],
        box_node3=signals["box_node3"],
        text_node=signals["text_node"],
    )

    # assign the update function to the task manager
    context.taskMgr.doMethodLater(0.0, simulation_manager.update_world, "update_world")
    context.run()

    # once completed, release the video writer
    if configuration["rendering"]["record_video"] and video_writer is not None:
        video_writer.release()
