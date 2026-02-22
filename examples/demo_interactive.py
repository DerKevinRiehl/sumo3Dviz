"""sumo3Dviz: A three-dimensional traffic visualisation (Interactive demo)
Authors: Kevin Riehl <kriehl@ethz.ch>, Julius Schlapbach <juliussc@ethz.ch>
Organisation: ETH Zürich, Institute for Transport Planning and Systems (IVT)

This example demonstrates how to render a short Interactive-mode video
allowing for a manual exploration of the scenario and collect camera positions
for a cinematic mode configuration. It uses the same configuration structure
as the CLI pipeline.

Usage (from repo root):
    python examples/demo_interactive.py
"""

# This script renders a 3D visualization of a SUMO simulation in the "Interactive" mode
# (camera follows the ego vehicle).
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
    RenderingTools,
    InteractionTools,
    validate_configuration,
)


# Configuration dictionary
# - Mirrors the minimal fields expected by the demo helpers and
#   the `SimulationManager`. Grouping follows: `rendering`,
#   `visualization`, `paths`, `signals`, and `modes`.
# - If you prefer, replace this dictionary by loading a yaml file using
#   the CLI helper `load_configuration(...)` and then validate it with
#   `validate_configuration(...)`.
mode_configuration = {
    "rendering": {
        "video_width_px": 1140,
        "video_height_px": 900,
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
    "modes": {
        "interactive": {
            "initial_camera_position": {
                "position": [20000, 18000, 50],
                "orientation": [120, -20, 0],
            },
        },
    },
}


def main(config_override=None, max_frames=None):
    """Main function to run the Interactive demo. CI-only parameters: config_override, max_frames."""
    # CI-only: Apply configuration overrides (not needed for normal usage)
    configuration = mode_configuration.copy()
    if config_override:
        for key, value in config_override.items():
            if (
                key in configuration
                and isinstance(configuration[key], dict)
                and isinstance(value, dict)
            ):
                configuration[key].update(value)
            else:
                configuration[key] = value

    # ! STEP 1: Validate the configuration file
    # region
    # alternatively, you may also load a yaml configuration file (as for the CLI-version)
    # using the helper function `load_configuration` before validation
    validate_configuration(configuration, mode="interactive")
    # endregion

    # ! STEP 2: Load the data (trajectories, positions of static objects, etc.) using the helper functions
    # region
    loader = LoaderTools()
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

    # ! STEP 3: Set up Panda3D
    # region
    if configuration["rendering"].get("headless", False):
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
        *configuration["modes"]["interactive"]["initial_camera_position"]["position"]
    )
    cast(Camera, context.camera).setHpr(
        *configuration["modes"]["interactive"]["initial_camera_position"]["orientation"]
    )

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

    # in interactive mode, ground uses default z_offset=-0.5 which provides
    # sufficient separation from road (z=0.01) to avoid z-fighting artifacts.
    rendering_tools.create_ground(
        context=context,
        ground_texture=configuration["visualization"]["ground_texture"],
        z_offset=-0.5,
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
    # endregion

    # ! STEP 5: Launch the simulation in Interactive mode.
    # The camera will follow the ego vehicle, allowing for manual exploration of the scene.
    # region
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

        # add the frame counter task
        context.taskMgr.add(frame_counter_task, "frame_counter")
        print(f"  ==>  CI Mode: Will exit after {max_frames} frames")

    context.run()
    # endregion


if __name__ == "__main__":
    main()
