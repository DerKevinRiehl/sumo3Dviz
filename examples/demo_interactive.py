"""sumo3Dviz: A three-dimensional traffic visualisation [2026]
Authors: Kevin Riehl <kriehl@ethz.ch>, Julius Schlapbach <jschlapbach@ethz.ch>
Organisation: ETH Zürich, Institute for Transport Planning and Systems (IVT)
"""

# This script renders a 3D visualization of a SUMO simulation in the "Interactive mode"
# enabling the user to navigate the scene with the keyboard.
# This mode can be useful to determine specific camera points that can be used for cinematic mode.

import os
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
)


################ METHODS
def _setup_panda_3d():
    loadPrcFileData(
        "",
        f'win-size {configuration["rendering"]["video_width_px"]} {configuration["rendering"]["video_height_px"]}',
    )
    loadPrcFileData("", "framebuffer-multisample 1")
    context = ShowBase()
    context.render.setAntialias(AntialiasAttrib.MAuto)
    fbprobs = FrameBufferProperties()
    fbprobs.setMultisamples(8)
    return context, fbprobs


def _setup_interactive_mode(context):
    interaction_tools = InteractionTools()
    interaction_tools.add_camera_control_interactive_mode(context)


def _load_positions(configuration):
    print("###############################")
    print("  ==>  Loading Resources")
    print("###############################")
    loader = LoaderTools()
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


def _render_world_scene(context, configuration, positions):
    print("###############################")
    print("  ==>  Rendering World")
    print("###############################")
    rendering_tools = RenderingTools()
    # camera
    cast(Camera, context.camera).setPos(
        *configuration["rendering"]["initial_camera_position"]["position"]
    )
    cast(Camera, context.camera).setHpr(
        *configuration["rendering"]["initial_camera_position"]["orientation"]
    )
    # head up display (HUD) to inform user in interactive mode
    rendering_tools.render_hud(context=context)
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
    print("###############################\n")


def _launch_interactive_mode(context):
    print("###############################")
    print("  ==>  Launch Interactive Mode")
    print("###############################")
    context.run()


################ CONFIGURATION
configuration = {
    "rendering": {
        "video_width_px": 1140,
        "video_height_px": 900,
        "initial_camera_position": {
            "position": (20000, 18000, 50),
            "orientation": (120, -20, 0),
        },
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
    },
    "paths": {
        "sumo_network_file": os.path.join(
            os.path.dirname(__file__),
            "barcelona_simulation/Network.net.xml",
        ),
        "tree_positions_file": os.path.join(
            os.path.dirname(__file__),
            "barcelona_simulation/viz_object_positions/trees.add.xml",
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
    },
}

################ MAIN
if __name__ == "__main__":
    context, fbprobs = _setup_panda_3d()
    _setup_interactive_mode(context)
    positions = _load_positions(configuration)
    _render_world_scene(context, configuration, positions)
    _launch_interactive_mode(context)
