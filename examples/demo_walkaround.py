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


if __name__ == "__main__":
    # --- CONFIG ---
    headless = False
    video_width_px = 1140
    video_height_px = 900

    sumo_network_file = os.path.join(
        os.path.dirname(__file__),
        "barcelona_simulation/Network.net.xml",
    )
    lane_width = 3.2
    sep_line_width = 0.2

    tree_positions_file = os.path.join(
        os.path.dirname(__file__),
        "barcelona_simulation/viz_object_positions/trees.add.xml",
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

    tree_scale_1 = 0.2
    tree_scale_2 = 0.5
    tree_size_variability = 1
    tree_color_variability = 0.1

    # --- LOADERS ---
    loader = LoaderTools()
    interaction_tools = InteractionTools()
    rendering_tools = RenderingTools()

    # pre-load positions etc. (no trajectories, no signals)
    tree_positions = loader.load_tree_positions(xml_file=tree_positions_file)
    fence_lines = loader.load_fence_lines(xml_file=fence_lines_file)
    shop_positions = loader.load_shop_positions(xml_file=shops_positions_file)
    homes_positions = loader.load_shop_positions(xml_file=homes_positions_file)
    block_positions = loader.load_shop_positions(xml_file=blocks_positions_file)

    # --- RENDERING CONTEXT ---
    if headless:
        loadPrcFileData("", "window-type offscreen")
    loadPrcFileData(
        "",
        f"win-size {video_width_px} {video_height_px}",
    )
    loadPrcFileData("", "framebuffer-multisample 1")

    context = ShowBase()
    context.render.setAntialias(AntialiasAttrib.MAuto)
    fbprobs = FrameBufferProperties()
    fbprobs.setMultisamples(8)

    # camera keyboard controls (e.g. arrow keys)
    interaction_tools.addCameraControlKeyboard(context)

    # --- STATIC SCENE SETUP ---

    # road network
    rendering_tools.create_road_network(
        context=context,
        sumo_network_file=sumo_network_file,
        lane_width=lane_width,
        sep_line_width=sep_line_width,
    )

    # light
    rendering_tools.create_light(context=context)

    # sky and ground
    rendering_tools.create_sky(context=context, sky_texture="sky_cloudy")
    rendering_tools.create_ground(context=context, ground_texture="ground_grass")

    # trees
    rendering_tools.create_trees(
        context=context,
        tree_positions=tree_positions,
        tree_scale_1=tree_scale_1,
        tree_scale_2=tree_scale_2,
        tree_size_variability=tree_size_variability,
        tree_color_variability=tree_color_variability,
    )

    # fences
    rendering_tools.create_highway_fences(
        context=context,
        fence_lines=fence_lines,
    )

    # buildings
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

    # place ego car purely as a static object
    car_models = loader.load_car_models(context=context)
    ego_car = loader.load_ego_car_model(context=context)
    ego_car.setPos(lane_width / 2, 25, 0)
    ego_car.setHpr(180, 90, 0)

    # initial camera position (pick some reasonable coordinates in your network)
    cast(Camera, context.camera).setPos(20000, 18000, 50)
    cast(Camera, context.camera).setHpr(120, -20, 0)

    # --- MAIN LOOP ---
    # no SimulationManager, no task, no video_writer
    context.run()
