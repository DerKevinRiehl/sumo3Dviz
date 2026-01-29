import os
import sumolib
import warnings
import platform
import numpy as np
from typing import Tuple, Union
from direct.showbase.ShowBase import ShowBase
from panda3d.core import (
    Geom,
    GeomNode,
    GeomVertexData,
    GeomVertexFormat,
    GeomVertexWriter,
    GeomTriangles,
    NodePath,
    CardMaker,
    LColor,
    Filename,
    TextureStage,
    LineSegs,
    FontPool,
    TextNode,
    Texture,
    DirectionalLight,
    AmbientLight,
    Vec4,
    get_model_path,
)


class RenderingTools:
    """Provides tools for rendering 3D scene elements for SUMO traffic visualization."""

    def __init__(self):
        """Initialize the RenderingTools and adjust model path on Windows."""
        if platform.system() == "Windows":
            windows_path = os.path.normpath(
                os.path.join(os.path.dirname(__file__), "..", "data")
            )
            windows_path = Filename.fromOsSpecific(windows_path)
            get_model_path().append_directory(windows_path)

    def create_light(self, context: ShowBase):
        """Create and configure lighting sources for the 3D scene.

        Sets up ambient light for general illumination and a directional light
        positioned to provide highlights and shading effects.

        Args:
            context (ShowBase): The Panda3D ShowBase context to attach lights to.
        """

        # ambient light (fill light)
        print("Rendering light sources...")
        alight = AmbientLight("alight")
        alight.setColor(Vec4(1.0, 1.0, 1.0, 1))
        alnp = context.render.attachNewNode(alight)
        context.render.setLight(alnp)

        # directional light (for highlights and shading)
        dlight = DirectionalLight("dlight")
        dlight.setColor(Vec4(1, 1, 1, 1))
        dlnp = context.render.attachNewNode(dlight)
        dlnp.setPos(0, -100, 200)
        dlnp.setHpr(-30, -45, 0)  # pointing downward and forward
        context.render.setLight(dlnp)
        print("Light sources rendered ✓")

    def create_sky(
        self,
        context: ShowBase,
        sky_texture: Union[str, None] = None,
        sky_texture_file: Union[str, None] = None,
        horizon_distance: float = 50000,
    ):
        """Create a textured sky sphere as the scene background.

        Loads a spherical model, applies a sky texture, and configures it to render
        as the background without depth writing.

        Args:
            context (ShowBase): The Panda3D ShowBase context.
            sky_texture (Union[str, None]): Textures that can be loaded directly
                from an available selection. Available textures are: sky_blue,
                sky_cloudy, sky_overcast, sky_dawn, sky_night_stars, sky_night_clear,
                sky_night_forest, sky_halloween.
            sky_texture_file (Union[str, None]): Optional path to the sky texture
                image file. If None is provided, sky texture string is checked and if
                this is not provided either, the default texture is loaded. If a
                texture file is provided by the user, they are responsible for ensuring
                it is a seamless equirectangular projection and that the filepath
                format is valid and compatible with Panda3D (potential model path
                adjustments may be necessary).
            horizon_distance (float): Radius of the sky sphere. Defaults to 50000.

        Raises:
            ValueError: If context.loader is not initialized or if an unknown
                sky texture is specified.
        """

        if context.loader is None:
            raise ValueError("Context loader is not initialized.")

        if sky_texture_file is not None:
            pass  # use provided texture file
        elif sky_texture is not None:
            # if a predefined texture is selected, load it
            if sky_texture == "sky_blue":
                sky_texture_file = "images/texture_sky_blue.jpg"
            elif sky_texture == "sky_cloudy":
                sky_texture_file = "images/texture_sky_daycloud1.jpg"
            elif sky_texture == "sky_overcast":
                sky_texture_file = "images/texture_sky_daycloud2.png"
            elif sky_texture == "sky_dawn":
                sky_texture_file = "images/texture_sky_daycloud3.png"
            elif sky_texture == "sky_night_stars":
                sky_texture_file = "images/texture_sky_night1.png"
            elif sky_texture == "sky_night_clear":
                sky_texture_file = "images/texture_sky_night2.png"
            elif sky_texture == "sky_night_forest":
                sky_texture_file = "images/texture_sky_night3.png"
            elif sky_texture == "sky_night_desert":
                sky_texture_file = "images/texture_sky_night4.png"
            elif sky_texture == "sky_halloween":
                sky_texture_file = "images/texture_sky_halloween.jpg"
            else:
                raise ValueError(f"Unknown sky texture: {sky_texture}")

            if platform.system() != "Windows":
                sky_texture_file = os.path.join(
                    os.path.dirname(__file__), f"../data/{sky_texture_file}"
                )
        else:
            # load default sky texture (if neither string nor file provided)
            if platform.system() == "Windows":
                sky_texture_file = "images/texture_sky_daycloud1.jpg"
            else:
                sky_texture_file = os.path.join(
                    os.path.dirname(__file__),
                    "../data/images/texture_sky_daycloud1.jpg",
                )

        # generate an inverted sphere model
        print("Rendering sky...")
        sky_sphere: NodePath = context.loader.loadModel("models/smiley")
        sky_sphere.reparentTo(context.render)
        sky_sphere.setScale(horizon_distance)
        sky_sphere.setTwoSided(True)

        # load the texture
        sky_texture = Texture()
        sky_texture = context.loader.loadTexture(sky_texture_file)
        sky_sphere.setTexture(sky_texture, 1)

        # set rendering properties
        sky_sphere.setBin("background", 0)
        sky_sphere.setDepthWrite(False)
        sky_sphere.setLightOff()
        print("Sky rendered ✓")

    def create_ground(
        self,
        context: ShowBase,
        ground_texture: Union[str, None] = None,
        ground_texture_file: Union[str, None] = None,
        INFINITY: float = 50000,
    ):
        """Create a textured ground plane for the scene.

        Generates a large card with a repeating texture to serve as the ground surface.

        Args:
            context (ShowBase): The Panda3D ShowBase context.
            ground_texture (Union[str, None]): Textures that can be loaded directly
                from an available selection. Available textures are: ground_grass,
                ground_stone, ground_sand, ground_chess, ground_chesslarge,
                ground_halloween.
            ground_texture_file (Union[str, None]): Optional path to the floor texture
                image file. If None is provided, ground texture string is checked and if
                this is not provided either, the default texture is loaded. If a
                texture file is provided by the user, they are responsible for ensuring
                the filepath format is valid and compatible with Panda3D (potential
                model path adjustments may be necessary).
            INFINITY (float): Size of the ground plane in each direction.
                Defaults to 50000.

        Raises:
            ValueError: If context.loader is not initialized or if an unknown
                ground texture is specified.
        """

        if context.loader is None:
            raise ValueError("Context loader is not initialized.")

        if ground_texture_file is not None:
            pass  # use provided texture string
        elif ground_texture is not None:
            # if a predefined texture is selected, load it
            if ground_texture == "ground_grass":
                ground_texture_file = "images/texture_ground_grass.jpg"
            elif ground_texture == "ground_stone":
                ground_texture_file = "images/texture_ground_stone.png"
            elif ground_texture == "ground_sand":
                ground_texture_file = "images/texture_ground_sand.png"
            elif ground_texture == "ground_chess":
                ground_texture_file = "images/texture_ground_chess.png"
            elif ground_texture == "ground_chesslarge":
                ground_texture_file = "images/texture_ground_chesslarge.png"
            elif ground_texture == "ground_halloween":
                ground_texture_file = "images/texture_ground_halloween.png"
            else:
                raise ValueError(f"Unknown ground texture: {ground_texture}")

            if platform.system() != "Windows":
                ground_texture_file = os.path.join(
                    os.path.dirname(__file__), f"../data/{ground_texture_file}"
                )
        else:
            # load default ground texture (if neither string nor file provided)
            if platform.system() == "Windows":
                ground_texture_file = "images/texture_ground_grass.jpg"
            else:
                ground_texture_file = os.path.join(
                    os.path.dirname(__file__),
                    "../data/images/texture_ground_grass.jpg",
                )

        # set ground parameters
        print("Rendering ground floor...")
        cm_ground = CardMaker("ground")
        cm_ground.setFrame(-INFINITY, INFINITY, -INFINITY, INFINITY)
        ground = context.render.attachNewNode(cm_ground.generate())
        ground.setPos(0, 0, -0.05)
        ground.setHpr(25, -90, 0)

        # load the texture
        ground_tex = context.loader.loadTexture(ground_texture_file)
        ground_tex.setWrapU(Texture.WM_repeat)
        ground_tex.setWrapV(Texture.WM_repeat)
        ground.setTexture(ground_tex)
        ground.setTexScale(TextureStage.getDefault(), 40000, 40000)
        print("Ground floor rendered ✓")

    def create_trees(
        self,
        context: ShowBase,
        tree_positions: Union[list[list[float]], None],
        tree_model_file_1: Union[str, None] = None,
        tree_model_file_2: Union[str, None] = None,
        tree_scale_1=0.2,
        tree_scale_2=0.5,
        tree_size_variability=1,
        tree_color_variability=0.1,
    ):
        """Create tree instances at specified positions with randomized appearance.

        Loads two tree models and randomly places them at given positions with
        variations in scale, color, and rotation for natural appearance.

        Args:
            context (ShowBase): The Panda3D ShowBase context.
            tree_positions (list[list[float]] | None): List of [x, y] coordinates
                for tree placement. If None, no trees are created.
            tree_model_file_1 (Union[str, None]): Optional path to the first tree
                model file. If None, the default model is used. If a model file is provided
                by the user, they are responsible for ensuring the filepath format is
                valid and compatible with Panda3D (potential model path adjustments
                may be necessary).
            tree_model_file_2 (Union[str, None]): Optional path to the second tree
                model file. If None, the default model is used. If a model file is provided
                by the user, they are responsible for ensuring the filepath format is
                valid and compatible with Panda3D (potential model path adjustments
                may be necessary).
            tree_scale_1 (float): Base scale for the first tree model. Defaults to 0.2.
            tree_scale_2 (float): Base scale for the second tree model. Defaults to 0.5.
            tree_size_variability (float): Scale variability multiplier. Defaults to 1.
            tree_color_variability (float): Color variation amount. Defaults to 0.1.

        Returns:
            list: List of tree NodePath instances created.

        Raises:
            ValueError: If context.loader is not initialized.
        """

        if context.loader is None:
            raise ValueError("Context loader is not initialized.")

        if tree_positions is None:
            warnings.warn("No tree positions provided. Skipping tree creation.")
            return []

        if tree_model_file_1 is None:
            if platform.system() == "Windows":
                tree_model_file_1 = "3d_models/trees/MapleTree.obj"
            else:
                tree_model_file_1 = os.path.join(
                    os.path.dirname(__file__),
                    "../data/3d_models/trees/MapleTree.obj",
                )

        if tree_model_file_2 is None:
            if platform.system() == "Windows":
                tree_model_file_2 = "3d_models/trees/Hazelnut.obj"
            else:
                tree_model_file_2 = os.path.join(
                    os.path.dirname(__file__),
                    "../data/3d_models/trees/Hazelnut.obj",
                )

        # load tree models
        print("Rendering trees...")
        tree1_model: NodePath = context.loader.loadModel(tree_model_file_1)
        tree2_model: NodePath = context.loader.loadModel(tree_model_file_2)

        # generate trees
        tree_instances = []
        for position in tree_positions:
            x = position[0]
            y = position[1]

            # select a tree randomly
            if np.random.random() < 0.5:
                tree_instance = tree1_model.copyTo(context.render)
                tree_scale = tree_scale_1
            else:
                tree_instance = tree2_model.copyTo(context.render)
                tree_scale = tree_scale_2

            # select a color randomly
            tree_instance.setColor(
                0, 0.1 + (1 + np.random.random()) * tree_color_variability, 0, 1
            )

            # select a scale randomly
            tree_instance.setScale(
                tree_scale * (1 + np.random.random()) * tree_size_variability
            )

            # place tree
            tree_instance.setPos(x, y, 0)
            tree_instance.setHpr(np.random.random() * 360, 90, 0)
            tree_instances.append(tree_instance)

        print("Trees rendered ✓")
        return tree_instances

    def create_highway_fences(
        self,
        context: ShowBase,
        fence_lines: Union[list[list[list[float]]], None],
        color: Tuple[float, float, float, float] = (0.3, 0.3, 0.3, 1),
        pole_thickness=10,
        vertical_line_dist=0.3,
        z_start=0.03,
        z_end=1.0,
        spacing=2.0,
    ):
        """Create highway fence structures along specified polylines.

        Generates vertical posts at regular intervals along fence lines and connects
        them with horizontal rails at two heights.

        Args:
            context (ShowBase): The Panda3D ShowBase context.
            fence_lines (list[list[list[float]]] | None): List of polylines, where
                each polyline is a list of [x, y] coordinates. If None, no fences
                are created.
            color (Tuple[float, float, float, float]): RGBA color for fence lines.
                Defaults to (0.3, 0.3, 0.3, 1).
            pole_thickness (float): Thickness of fence lines in pixels. Defaults to 10.
            vertical_line_dist (float): Vertical distance between the two horizontal
                rails. Defaults to 0.3.
            z_start (float): Starting height for vertical posts. Defaults to 0.03.
            z_end (float): Ending height for vertical posts. Defaults to 1.0.
            spacing (float): Distance between adjacent fence posts. Defaults to 2.0.
        """

        if fence_lines is None:
            warnings.warn("No fence lines provided. Skipping fence creation.")
            return

        print("Rendering highway fences...")
        for line in fence_lines:
            all_posts = []  # collect all post positions
            for i in range(0, len(line) - 1):
                pA = np.asarray(line[i])
                pB = np.asarray(line[i + 1])
                dx = pB[0] - pA[0]
                dy = pB[1] - pA[1]
                lane_length = np.linalg.norm(pA - pB)
                num_posts = int(lane_length / spacing)

                for j in range(num_posts + 1):
                    x = pA[0] + (dx * j / num_posts)
                    y = pA[1] + (dy * j / num_posts)
                    all_posts.append((x, y, z_end))  # collect top of each post

                    # draw vertical line
                    lines = LineSegs()
                    lines.setColor(LColor(*color))
                    lines.setThickness(pole_thickness)
                    lines.moveTo(x, y, z_start)
                    lines.drawTo(x, y, z_end)
                    context.render.attachNewNode(lines.create())

            # draw two connecting lines through the tops of the posts
            if all_posts:
                lines = LineSegs()
                lines.setColor(LColor(*color))
                lines.setThickness(pole_thickness)
                for i in range(len(all_posts) - 1):
                    lines.moveTo(all_posts[i][0], all_posts[i][1], z_end)
                    lines.drawTo(all_posts[i + 1][0], all_posts[i + 1][1], z_end)
                context.render.attachNewNode(lines.create())
            if all_posts:
                lines = LineSegs()
                lines.setColor(LColor(*color))
                lines.setThickness(pole_thickness)
                for i in range(len(all_posts) - 1):
                    lines.moveTo(
                        all_posts[i][0], all_posts[i][1], z_end - vertical_line_dist
                    )
                    lines.drawTo(
                        all_posts[i + 1][0],
                        all_posts[i + 1][1],
                        z_end - vertical_line_dist,
                    )
                context.render.attachNewNode(lines.create())

        print("Highway fences rendered ✓")

    def create_building_shops(
        self,
        context: ShowBase,
        shop_positions: Union[list[list[float]], None],
        store_model_file: Union[str, None] = None,
    ):
        """Create shop building instances at specified positions.

        Loads a shop model, scales it to a target width of 8 units, and places
        instances at given positions.

        Args:
            context (ShowBase): The Panda3D ShowBase context.
            shop_positions (list[list[float]] | None): List of [x, y] coordinates
                for shop placement. If None, no shops are created.
            store_model_file (str | None): Optional path to the 3D shop
                model file. If None is provided, the default model is used. If a
                model file is provided by the user, they are responsible for ensuring
                the filepath format is valid and compatible with Panda3D (potential
                model path adjustments may be necessary).

        Returns:
            tuple: A tuple containing:
                - list: List of shop NodePath instances created
                - float | None: Scaled depth of the shop model

        Raises:
            ValueError: If context.loader is not initialized.
        """

        if context.loader is None:
            raise ValueError("Context loader is not initialized.")

        if shop_positions is None:
            warnings.warn("No shop positions provided. Skipping shop creation.")
            return [], None

        if store_model_file is None:
            if platform.system() == "Windows":
                store_model_file = (
                    "3d_models/buildings/10065_Corner Grocery Store_V2_L3.obj"
                )
            else:
                store_model_file = os.path.join(
                    os.path.dirname(__file__),
                    "../data/3d_models/buildings/10065_Corner Grocery Store_V2_L3.obj",
                )

        # load shop model
        print("Rendering shops...")
        building: NodePath = context.loader.loadModel(store_model_file)

        # get the original bounding box
        min_point, max_point = building.getTightBounds()
        if min_point is None or max_point is None:
            warnings.warn("Could not determine model bounds.")
            return [], None

        # calculate original width and depth
        original_width = max_point[0] - min_point[0]  # x-axis
        original_depth = max_point[1] - min_point[1]  # y-axis

        # calculate scale so the width becomes 8 units
        target_width = 8.0
        scale_factor = target_width / original_width

        # calculate scaled depth
        scaled_depth = original_depth * scale_factor

        # generate shops
        shop_instances = []
        for position in shop_positions:
            shop_instance = building.copyTo(context.render)
            shop_instance.setScale(scale_factor)
            shop_instance.setPos(position[0], position[1], 0)
            shop_instance.setHpr(0, 0, 0)
            shop_instances.append(shop_instance)

        print("Shops rendered ✓")
        return shop_instances, scaled_depth

    def create_building_homes(
        self,
        context: ShowBase,
        homes_positions: Union[list[list[float]], None],
        home_model_file: Union[str, None] = None,
    ):
        """Create residential building instances at specified positions.

        Loads a home model, scales it to 0.01, and places instances at given
        positions with slight random color variations.

        Args:
            context (ShowBase): The Panda3D ShowBase context.
            homes_positions (list[list[float]] | None): List of [x, y] coordinates
                for home placement. If None, no homes are created.
            home_model_file (str | None): Optional path to the 3D home
                model file. If None is provided, the default model is used. If a
                model file is provided by the user, they are responsible for ensuring
                the filepath format is valid and compatible with Panda3D (potential
                model path adjustments may be necessary).

        Returns:
            tuple: A tuple containing:
                - list: List of home NodePath instances created
                - None: Placeholder for API consistency

        Raises:
            ValueError: If context.loader is not initialized.
        """

        if context.loader is None:
            raise ValueError("Context loader is not initialized.")

        if homes_positions is None:
            warnings.warn("No home positions provided. Skipping home creation.")
            return [], None

        if home_model_file is None:
            if platform.system() == "Windows":
                home_model_file = (
                    "3d_models/buildings/10084_Small Home_V3_Iteration0.obj"
                )
            else:
                home_model_file = os.path.join(
                    os.path.dirname(__file__),
                    "../data/3d_models/buildings/10084_Small Home_V3_Iteration0.obj",
                )

        # load home model
        print("Rendering homes...")
        building: NodePath = context.loader.loadModel(home_model_file)

        # generate homes
        home_instances = []
        for position in homes_positions:
            # select a scale randomly
            home_instance = building.copyTo(context.render)
            home_instance.setScale(0.01)
            r = np.abs(0.05 * np.random.random())
            home_instance.setColor(1 - r, 1 - r, 1, 1)

            # place building
            home_instance.setPos(position[0], position[1], 0)
            home_instance.setHpr(0, 0, 0)
            home_instances.append(home_instance)

        print("Homes rendered ✓")

    def create_building_blocks(
        self,
        context: ShowBase,
        block_positions: Union[list[list[float]], None],
        block_model_file: Union[str, None] = None,
    ):
        """Create large building block instances at specified positions.

        Loads a building block model and places instances at given positions with
        randomized colors for variety.

        Args:
            context (ShowBase): The Panda3D ShowBase context.
            block_positions (list[list[float]] | None): List of [x, y] coordinates
                for block placement. If None, no blocks are created.
            block_model_file (str | None): Optional path to the 3D building block
                model file. If None is provided, the default model is used. If a
                model file is provided by the user, they are responsible for ensuring
                the filepath format is valid and compatible with Panda3D (potential
                model path adjustments may be necessary).

        Returns:
            tuple: A tuple containing:
                - list: List of building block NodePath instances created
                - None: Placeholder for API consistency

        Raises:
            ValueError: If context.loader is not initialized.
        """

        if context.loader is None:
            raise ValueError("Context loader is not initialized.")

        if block_positions is None:
            warnings.warn("No block positions provided. Skipping block creation.")
            return [], None

        if block_model_file is None:
            if platform.system() == "Windows":
                block_model_file = "3d_models/buildings/Residential Buildings 002.obj"
            else:
                block_model_file = os.path.join(
                    os.path.dirname(__file__),
                    "../data/3d_models/buildings/Residential Buildings 002.obj",
                )

        # load block model
        print("Rendering building blocks...")
        building: NodePath = context.loader.loadModel(block_model_file)

        # generate blocks
        block_instances = []
        for position in block_positions:
            block_instance = building.copyTo(context.render)
            block_instance.setColor(
                0.3 + 0.7 * np.random.random(),
                0.3 + 0.7 * np.random.random(),
                0.3 + 0.7 * np.random.random(),
                1,
            )
            block_instance.setPos(position[0], position[1], 0)
            block_instance.setHpr(90, 90, 0)
            block_instances.append(block_instance)

        print("Building blocks rendered ✓")

    def create_traffic_light(
        self,
        context: ShowBase,
        design: str,
        x: float,
        y: float,
        z: float = 0,
        pole_height: float = 2.0,
        signal: str = "yellow",
        timer: float = 5,
    ):
        """Create a traffic light at a specified position with initial state.

        Generates a traffic light structure with a pole, housing box, and light
        indicators. Supports different designs including simple ramp metering,
        three-headed traffic lights, and countdown timer displays.

        Args:
            context (ShowBase): The Panda3D ShowBase context.
            design (str): Traffic light design type:
                - "simple": Simple ramp metering (red/green only)
                - "three_headed": Three-headed traffic light (red/yellow/green)
                - "countdown_timer": Ramp metering with countdown timer
            x (float): X coordinate for traffic light position.
            y (float): Y coordinate for traffic light position.
            z (float): Z coordinate (ground level). Defaults to 0.
            pole_height (float): Height of the traffic light pole. Defaults to 2.0.
            signal (str): Initial signal state ('green', 'red', 'yellow', 'yellow2').
                Defaults to 'yellow'.
            timer (float): Initial countdown timer value in seconds. Defaults to 5.

        Returns:
            tuple: A tuple containing:
                - box_node1: First light box NodePath (red light)
                - box_node2: Second light box NodePath (yellow or green light)
                - box_node3: Third light box NodePath (green light, None if not 3-headed)
                - text_node: TextNode for countdown timer (None if no timer)
        """

        print("Drawing traffic light...")
        if design == "simple":  # RAMP METERING SIMPLE
            THREE_HEAD = False
            COUNTDOWN_TIMER = False
        elif design == "three_headed":  # THREE HEADED
            THREE_HEAD = True
            COUNTDOWN_TIMER = False
        elif design == "countdown_timer":  # COUNTDOWN TIMER
            THREE_HEAD = False
            COUNTDOWN_TIMER = True
        else:
            THREE_HEAD = False
            COUNTDOWN_TIMER = False

        # 1. Draw a vertical line as a pole
        pole_thickness = 10
        lines = LineSegs()
        lines.setColor(LColor(0.1, 0.1, 0.1, 1))  # Dark gray for pole
        lines.setThickness(pole_thickness)
        lines.moveTo(x, y, z)
        lines.drawTo(x, y, z + pole_height)
        context.render.attachNewNode(lines.create())

        # 2. Draw a black box (housing) for the traffic light
        box_width = 0.5
        box_height = 0.5
        box_depth = 1.5
        box_z = z + pole_height  # place box on top of pole
        box_node = context.render.attachNewNode(
            self._make_box(
                box_width, box_height, box_depth, "black_box", color=(0, 0, 0, 0)
            )
        )
        box_node.setPos(x, y, box_z + box_depth / 2)

        # 3. Draw three colored circles (lights) inside the box
        full_color = 1.0
        dark_color = 0.2
        if signal == "green":
            color_green = full_color
            color_red = dark_color
            color_yellow = dark_color
        elif signal == "red":
            color_green = dark_color
            color_red = full_color
            color_yellow = dark_color
        elif signal == "yellow":
            if THREE_HEAD:
                color_green = dark_color
                color_red = full_color
                color_yellow = full_color
            else:
                color_green = dark_color
                color_red = full_color
                color_yellow = dark_color
        elif signal == "yellow2":
            if THREE_HEAD:
                color_green = dark_color
                color_red = dark_color
                color_yellow = full_color
            else:
                color_green = dark_color
                color_red = full_color
                color_yellow = dark_color
        color_green = full_color
        color_red = full_color
        color_yellow = full_color

        if THREE_HEAD:
            box_node1 = context.render.attachNewNode(
                self._make_box(0.3, 0.3, 0.3, "red_box", color=(color_red, 0, 0, 1))
            )
            box_node1.setPos(x, y, z + pole_height + box_depth / 2 + 0.05 + 0.3 + 0.2)
            box_node2 = context.render.attachNewNode(
                self._make_box(
                    0.3,
                    0.3,
                    0.3,
                    "yellow_box",
                    color=(color_yellow, color_yellow, 0, 1),
                )
            )
            box_node2.setPos(x, y, z + pole_height + box_depth / 2 + 0.05)
            box_node3 = context.render.attachNewNode(
                self._make_box(0.3, 0.3, 0.3, "green_box", color=(0, color_green, 0, 1))
            )
            box_node3.setPos(x, y, z + pole_height + box_depth / 2 + 0.05 - 0.3 - 0.2)
        else:
            box_node1 = context.render.attachNewNode(
                self._make_box(0.3, 0.3, 0.3, "red_box", color=(color_red, 0, 0, 1))
            )
            box_node1.setPos(x, y, z + pole_height + box_depth / 2 + 0.05 + 0.25)
            box_node2 = context.render.attachNewNode(
                self._make_box(0.3, 0.3, 0.3, "green_box", color=(0, color_green, 0, 1))
            )
            box_node2.setPos(x, y, z + pole_height + box_depth / 2 + 0.05 - 0.25)
            box_node3 = None

        if COUNTDOWN_TIMER and timer != 0:
            # draw black rectangle
            self._make_billboarded_rectangle(
                context,
                x + box_width / 2 + 1.4,
                y,
                z + pole_height + box_depth / 2,
                0.8,
                0.8,
            )
            # Draw white text (timer)
            _, text_node = self._make_billboarded_text(
                context,
                x + box_width / 2 + 2.2,
                y,
                z + pole_height + box_depth / 2 - 0.15,
                "{:02d}".format(timer),
            )
        else:
            text_node = None

        print("Traffic light drawn ✓")
        return box_node1, box_node2, box_node3, text_node

    def _make_box(
        self,
        width: float,
        height: float,
        depth: float,
        lbl: str,
        color: Tuple[float, float, float, float],
    ):
        """Create a 3D box geometry node with specified dimensions and color.

        Args:
            width (float): Width of the box (x-axis).
            height (float): Height of the box (y-axis).
            depth (float): Depth of the box (z-axis).
            lbl (str): Label for the geometry node.
            color (Tuple[float, float, float, float]): RGBA color values.

        Returns:
            GeomNode: A geometry node representing the box.
        """
        frm = GeomVertexFormat.getV3n3c4()
        vdata = GeomVertexData(lbl, frm, Geom.UHStatic)
        vertex = GeomVertexWriter(vdata, "vertex")
        normal = GeomVertexWriter(vdata, "normal")
        color_writer = GeomVertexWriter(vdata, "color")

        # define the 8 corners of the box
        w2, h2, d2 = width / 2, height / 2, depth / 2
        vertices = [
            (-w2, -h2, -d2),
            (w2, -h2, -d2),
            (w2, h2, -d2),
            (-w2, h2, -d2),  # Front face corners
            (-w2, -h2, d2),
            (w2, -h2, d2),
            (w2, h2, d2),
            (-w2, h2, d2),  # Back face corners
        ]

        # add vertices and normals (simplified here)
        for pos in vertices:
            vertex.addData3f(pos[0], pos[1], pos[2])
            normal.addData3f(
                0, 0, 1
            )  # all normals same for simplicity (not correct, but fast)

        for _ in range(8):
            color_writer.addData4f(*color)

        # indices for 6 faces (each face has 2 triangles)
        tris = GeomTriangles(Geom.UHStatic)

        # front face
        tris.addVertices(0, 1, 2)
        tris.addVertices(0, 2, 3)

        # back face
        tris.addVertices(4, 7, 6)
        tris.addVertices(4, 6, 5)

        # right face
        tris.addVertices(1, 5, 6)
        tris.addVertices(1, 6, 2)

        # left face
        tris.addVertices(0, 3, 7)
        tris.addVertices(0, 7, 4)

        # top face
        tris.addVertices(3, 2, 6)
        tris.addVertices(3, 6, 7)

        # bottom face
        tris.addVertices(0, 4, 5)
        tris.addVertices(0, 5, 1)

        geom = Geom(vdata)
        geom.addPrimitive(tris)
        node = GeomNode(lbl)
        node.addGeom(geom)
        return node

    def _make_billboarded_rectangle(
        self,
        context: ShowBase,
        x: float,
        y: float,
        z: float,
        width: float,
        height: float,
        color: Tuple[float, float, float, float] = (0, 0, 0, 1),
    ):
        """Create a rectangular card that always faces the camera.

        Args:
            context (ShowBase): The Panda3D ShowBase context.
            x (float): X coordinate for rectangle center.
            y (float): Y coordinate for rectangle center.
            z (float): Z coordinate for rectangle center.
            width (float): Width of the rectangle.
            height (float): Height of the rectangle.
            color (Tuple[float, float, float, float]): RGBA color values.
                Defaults to (0, 0, 0, 1).
        """
        cm = CardMaker("billboarded_rect")
        cm.setFrame(-width / 2, width / 2, -height / 2, height / 2)
        rect = context.render.attachNewNode(cm.generate())
        rect.setPos(x, y, z)
        rect.setColor(*color)
        rect.setBillboardPointEye()  # always faces camera

    def _make_billboarded_text(
        self,
        context: ShowBase,
        x: float,
        y: float,
        z: float,
        text: str,
        color: Tuple[float, float, float, float] = (1, 1, 1, 1),
    ):
        """Create text that always faces the camera.

        Args:
            context (ShowBase): The Panda3D ShowBase context.
            x (float): X coordinate for text position.
            y (float): Y coordinate for text position.
            z (float): Z coordinate for text position.
            text (str): Text content to display.
            color (Tuple[float, float, float, float]): RGBA color values for text.
                Defaults to (1, 1, 1, 1).

        Returns:
            tuple: A tuple containing:
                - node: NodePath of the text
                - text_node: TextNode instance for updating text content
        """
        # create a TextNode
        text_node = TextNode("billboarded_text")
        text_node.setText(text)
        text_node.setTextColor(*color)
        consolas_font = FontPool.load_font("consola.ttf")
        text_node.setFont(consolas_font)

        # attach to a node
        node = context.render.attachNewNode(text_node)
        node.setPos(x, y, z)
        node.setBillboardPointEye()  # Always faces camera
        node.setScale(0.6)  # Adjust scale as needed
        return node, text_node

    def create_white_signal_line(
        self,
        context: ShowBase,
        p1: float,
        p2: float,
        p3: float,
        p4: float,
        sep_line_width: float,
        color: Tuple[float, float, float, float] = (1, 1, 1, 1),
        z=0.03,
    ):
        """Create a white signal line (stop line) between two points.

        Args:
            context (ShowBase): The Panda3D ShowBase context.
            p1 (float): X coordinate of first point.
            p2 (float): Y coordinate of first point.
            p3 (float): X coordinate of second point.
            p4 (float): Y coordinate of second point.
            sep_line_width (float): Width of the signal line.
            color (Tuple[float, float, float, float]): RGBA color values.
                Defaults to (1, 1, 1, 1).
            z (float): Z coordinate (height) of the line. Defaults to 0.03.
        """
        # convert to numpy arrays
        print("Drawing white signal line...")
        pA = np.asarray([p1, p2])
        pB = np.asarray([p3, p4])

        # compute lane length
        lane_length = np.linalg.norm(pA - pB)

        # compute angle in degrees
        dx = pB[0] - pA[0]
        dy = pB[1] - pA[1]
        angle_deg = np.degrees(np.arctan2(dy, dx)) - 90

        # compute center
        center_x = (pA[0] + pB[0]) / 2
        center_y = (pA[1] + pB[1]) / 2

        # create road (card)
        cm_road = CardMaker("card")
        cm_road.setFrame(
            -sep_line_width / 2, sep_line_width / 2, -lane_length / 2, lane_length / 2
        )
        road = context.render.attachNewNode(cm_road.generate())
        road.setPos(center_x, center_y, z)
        road.setHpr(angle_deg, -90, 0)
        road.setColor(LColor(*color))
        print("White signal line drawn ✓")

    def update_traffic_light(
        self,
        signal: str,
        design: str,
        timer: float,
        box_node1: NodePath,
        box_node2: NodePath,
        box_node3: Union[NodePath, None],
        text_node: Union[TextNode, None],
    ):
        """
        Update traffic light colors and timer display based on signal state.

        Args:
            signal: Traffic light state ('green', 'red', 'yellow', 'yellow2')
            design: Traffic light design type ("simple", "three_headed", "countdown_timer")
            timer: Countdown timer value
            box_node1: First light box node (red or red in simple)
            box_node2: Second light box node (yellow or green in simple)
            box_node3: Third light box node (green in 3-head, None otherwise)
            text_node: Text node for countdown timer (None if no timer)
        """
        # determine Design
        if design == "simple":  # RAMP METERING SIMPLE
            THREE_HEAD = False
            COUNTDOWN_TIMER = False
        elif design == "three_headed":  # THREE HEADED
            THREE_HEAD = True
            COUNTDOWN_TIMER = False
        elif design == "countdown_timer":  # COUNTDOWN TIMER
            THREE_HEAD = False
            COUNTDOWN_TIMER = True
        else:
            THREE_HEAD = False
            COUNTDOWN_TIMER = False

        # set Traffic Light colors
        full_color = 1.0
        dark_color = 0.2

        if signal == "green":
            color_green = full_color
            color_red = dark_color
            color_yellow = dark_color
        elif signal == "red":
            color_green = dark_color
            color_red = full_color
            color_yellow = dark_color
        elif signal == "yellow":
            if THREE_HEAD:
                color_green = dark_color
                color_red = full_color
                color_yellow = full_color
            else:
                color_green = dark_color
                color_red = full_color
                color_yellow = dark_color
        elif signal == "yellow2":
            if THREE_HEAD:
                color_green = dark_color
                color_red = dark_color
                color_yellow = full_color
            else:
                color_green = dark_color
                color_red = full_color
                color_yellow = dark_color
        else:
            color_green = dark_color
            color_red = dark_color
            color_yellow = dark_color

        # update light colors
        if THREE_HEAD:
            box_node1.setColor(color_red, 0, 0, 1)
            box_node2.setColor(color_yellow, color_yellow, 0, 1)
            if box_node3 is not None:
                box_node3.setColor(0, color_green, 0, 1)
        else:
            box_node1.setColor(color_red, 0, 0, 1)
            box_node2.setColor(0, color_green, 0, 1)

        # update countdown timer text
        if COUNTDOWN_TIMER and text_node is not None:
            if timer != 0:
                text = "{:02d}".format(int(timer + 1))
            else:
                text = ""
            text_node.setText(text)

    def create_road_network(
        self,
        context: ShowBase,
        sumo_network_file: str,
        lane_width: float,
        sep_line_width: float,
        concrete_color: Tuple[float, float, float, float] = (0.2, 0.2, 0.2, 1),
        separator_color: Tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1),
    ):
        """Create a 3D representation of a SUMO road network.

        Loads a SUMO network file and renders all edges, lanes, and junctions
        with appropriate lane markings.

        Args:
            context (ShowBase): The Panda3D ShowBase context.
            sumo_network_file (str): Path to the SUMO network (.net.xml) file.
            lane_width (float): Width of each lane.
            sep_line_width (float): Width of lane separator lines.
            concrete_color (Tuple[float, float, float, float]): RGBA color for
                road surface. Defaults to (0.2, 0.2, 0.2, 1).
            separator_color (Tuple[float, float, float, float]): RGBA color for
                lane markings. Defaults to (1.0, 1.0, 1.0, 1).
        """
        # load SUMO road network
        print("Rendering road network...")
        net = sumolib.net.readNet(sumo_network_file)

        # draw road edges and lanes
        for edge in net.getEdges():
            for lane in edge.getLanes():
                lane_shape = lane.getShape()
                lane_id = lane.getID().split("_")[-1]
                if edge.getLaneNumber() == 1:
                    lane_l = "b"
                elif lane_id == "0":
                    lane_l = "a"
                elif lane_id == str(edge.getLaneNumber() - 1):
                    lane_l = "e"
                else:
                    lane_l = "i"
                self._create_road_edge_lane(
                    context=context,
                    lane_shape=lane_shape,
                    lane_width=lane_width,
                    sep_line_width=sep_line_width,
                    lane_id=lane_l,
                    concrete_color=concrete_color,
                    separator_color=separator_color,
                )

        # draw junctions
        for junction in net.getNodes():
            junction_shape = junction.getShape()
            if junction_shape:
                self._create_polygon_fan(
                    context=context,
                    shape_points=junction_shape,
                    concrete_color=concrete_color,
                )

        print("Road network rendered ✓")

    def _create_road_edge_lane(
        self,
        context: ShowBase,
        lane_shape,
        lane_width: float,
        sep_line_width: float,
        lane_id: str,
        concrete_color: Tuple[float, float, float, float],
        separator_color: Tuple[float, float, float, float],
    ):
        """Create a single lane with appropriate road markings.

        Args:
            context (ShowBase): The Panda3D ShowBase context.
            lane_shape: List of (x, y) coordinate tuples defining the lane centerline.
            lane_width (float): Width of the lane.
            sep_line_width (float): Width of lane separator lines.
            lane_id (str): Lane identifier determining marking type:
                - 'b': Both side markings (single lane edge)
                - 'a': Rightmost lane (solid right, dashed left)
                - 'e': Leftmost lane (solid left)
                - 'i': Interior lane (dashed right)
            concrete_color (Tuple[float, float, float, float]): RGBA color for
                road surface.
            separator_color (Tuple[float, float, float, float]): RGBA color for
                lane markings.
        """
        self._create_concrete(
            context=context,
            lane_shape=lane_shape,
            lane_width=lane_width,
            color=concrete_color,
        )
        if lane_id == "b":
            self._create_white_seperator_line_left(
                context=context,
                lane_shape=lane_shape,
                lane_width=lane_width,
                sep_line_width=sep_line_width,
                color=separator_color,
            )
            self._create_white_seperator_line_right(
                context=context,
                lane_shape=lane_shape,
                lane_width=lane_width,
                sep_line_width=sep_line_width,
                color=separator_color,
            )
        if lane_id == "a":
            self._create_white_seperator_line_right(
                context=context,
                lane_shape=lane_shape,
                lane_width=lane_width,
                sep_line_width=sep_line_width,
                color=separator_color,
            )
            self._create_white_separator_line_right_dashed(
                context=context,
                lane_shape=lane_shape,
                lane_width=lane_width,
                sep_line_width=sep_line_width,
                color=separator_color,
                z=0.03,
                dash_length=1.0,
                gap_length=1.0,
            )
        elif lane_id == "e":
            self._create_white_seperator_line_left(
                context=context,
                lane_shape=lane_shape,
                lane_width=lane_width,
                sep_line_width=sep_line_width,
                color=separator_color,
            )
        elif lane_id == "i":
            self._create_white_separator_line_right_dashed(
                context=context,
                lane_shape=lane_shape,
                lane_width=lane_width,
                sep_line_width=sep_line_width,
                color=separator_color,
                z=0.03,
                dash_length=1.0,
                gap_length=1.0,
            )

    def _create_concrete(
        self,
        context: ShowBase,
        lane_shape,
        lane_width: float,
        color: Tuple[float, float, float, float],
        z=0.01,
    ):
        """Create road surface geometry along a lane centerline.

        Args:
            context (ShowBase): The Panda3D ShowBase context.
            lane_shape: List of (x, y) coordinate tuples defining the lane centerline.
            lane_width (float): Width of the lane.
            color (Tuple[float, float, float, float]): RGBA color for road surface.
            z (float): Z coordinate (height) of the road surface. Defaults to 0.01.
        """

        for i in range(0, len(lane_shape) - 1):
            pA = np.asarray(lane_shape[i])
            pB = np.asarray(lane_shape[i + 1])

            # length
            lane_length = np.linalg.norm(pA - pB)

            # angle
            dx = pB[0] - pA[0]
            dy = pB[1] - pA[1]
            angle_deg = np.degrees(np.arctan2(dy, dx)) - 90

            # center
            center_x = pA[0]
            center_y = pA[1]

            # make road
            cm_road = CardMaker("card")
            cm_road.setFrame(-lane_width / 2, lane_width / 2, 0, lane_length)
            road = context.render.attachNewNode(cm_road.generate())
            road.setPos(center_x, center_y, z)
            road.setHpr(angle_deg, -90, 0)
            road.setColor(LColor(*color))

    def _create_white_seperator_line_left(
        self,
        context: ShowBase,
        lane_shape,
        lane_width: float,
        sep_line_width: float,
        color: Tuple[float, float, float, float],
        z=0.03,
    ):
        """Create a solid separator line on the left side of a lane.

        Args:
            context (ShowBase): The Panda3D ShowBase context.
            lane_shape: List of (x, y) coordinate tuples defining the lane centerline.
            lane_width (float): Width of the lane.
            sep_line_width (float): Width of the separator line.
            color (Tuple[float, float, float, float]): RGBA color for the line.
            z (float): Z coordinate (height) of the line. Defaults to 0.03.
        """

        for i in range(0, len(lane_shape) - 1):
            pA = np.asarray(lane_shape[i])
            pB = np.asarray(lane_shape[i + 1])

            # length
            lane_length = np.linalg.norm(pA - pB)

            # angle
            dx = pB[0] - pA[0]
            dy = pB[1] - pA[1]
            angle_deg = np.degrees(np.arctan2(dy, dx)) - 90

            # center
            center_x = pA[0]
            center_y = pA[1]

            # make road
            cm_road = CardMaker("card")
            cm_road.setFrame(
                -lane_width / 2 + sep_line_width - sep_line_width / 2,
                -lane_width / 2 + sep_line_width + sep_line_width / 2,
                0,
                lane_length,
            )
            road = context.render.attachNewNode(cm_road.generate())
            road.setPos(center_x, center_y, z)
            road.setHpr(angle_deg, -90, 0)
            road.setColor(LColor(*color))

    def _create_white_seperator_line_right(
        self,
        context: ShowBase,
        lane_shape,
        lane_width: float,
        sep_line_width: float,
        color: Tuple[float, float, float, float],
        z=0.03,
    ):
        """Create a solid separator line on the right side of a lane.

        Args:
            context (ShowBase): The Panda3D ShowBase context.
            lane_shape: List of (x, y) coordinate tuples defining the lane centerline.
            lane_width (float): Width of the lane.
            sep_line_width (float): Width of the separator line.
            color (Tuple[float, float, float, float]): RGBA color for the line.
            z (float): Z coordinate (height) of the line. Defaults to 0.03.
        """

        for i in range(0, len(lane_shape) - 1):
            pA = np.asarray(lane_shape[i])
            pB = np.asarray(lane_shape[i + 1])

            # length
            lane_length = np.linalg.norm(pA - pB)

            # angle
            dx = pB[0] - pA[0]
            dy = pB[1] - pA[1]
            angle_deg = np.degrees(np.arctan2(dy, dx)) - 90

            # center
            center_x = pA[0]
            center_y = pA[1]

            # make road
            cm_road = CardMaker("card")
            cm_road.setFrame(
                +lane_width / 2 - sep_line_width - sep_line_width / 2,
                +lane_width / 2 - sep_line_width + sep_line_width / 2,
                0,
                lane_length,
            )
            road = context.render.attachNewNode(cm_road.generate())
            road.setPos(center_x, center_y, z)
            road.setHpr(angle_deg, -90, 0)
            road.setColor(LColor(*color))

    # TODO - figure out why this function causes issues with rendering speed
    def _create_white_separator_line_right_dashed(
        self,
        context: ShowBase,
        lane_shape,
        lane_width: float,
        sep_line_width: float,
        color: Tuple[float, float, float, float],
        z=0.03,
        dash_length=1.0,
        gap_length=1.0,
    ):
        """Create a dashed separator line on the right side of a lane.

        Currently not implemented due to rendering performance issues.

        Args:
            context (ShowBase): The Panda3D ShowBase context.
            lane_shape: List of (x, y) coordinate tuples defining the lane centerline.
            lane_width (float): Width of the lane.
            sep_line_width (float): Width of the separator line.
            color (Tuple[float, float, float, float]): RGBA color for the line.
            z (float): Z coordinate (height) of the line. Defaults to 0.03.
            dash_length (float): Length of each dash. Defaults to 1.0.
            gap_length (float): Length of gap between dashes. Defaults to 1.0.
        """

        pass
        # for i in range(len(lane_shape) - 1):
        #     pA = np.asarray(lane_shape[i])
        #     pB = np.asarray(lane_shape[i + 1])
        #     segment_vector = pB - pA
        #     segment_length = np.linalg.norm(segment_vector)
        #     if segment_length == 0:
        #         continue
        #     direction = segment_vector / segment_length
        #     num_dashes = int(segment_length // (dash_length + gap_length))
        #     angle_deg = np.degrees(np.arctan2(direction[1], direction[0])) - 90
        #     for j in range(num_dashes):
        #         start_offset = (dash_length + gap_length) * j
        #         dash_center = pA + direction * (start_offset + dash_length / 2)
        #         # Offset to the right side of the lane
        #         normal = np.array([-direction[1], direction[0]])  # 90-degree rotation
        #         offset = normal * (lane_width / 2 - sep_line_width / 2)
        #         final_pos = dash_center + offset
        #         # Create dashed card
        #         cm_dash = CardMaker("dash")
        #         cm_dash.setFrame(-sep_line_width / 2, sep_line_width / 2, 0, dash_length)
        #         dash = context.render.attachNewNode(cm_dash.generate())
        #         dash.setPos(final_pos[0], final_pos[1], z)
        #         dash.setHpr(angle_deg, -90, 0)
        #         dash.setColor(LColor(*color))

    def _create_polygon_fan(
        self,
        context: ShowBase,
        shape_points,
        concrete_color: Tuple[float, float, float, float],
        z=0.01,
    ):
        """Create a filled polygon using a triangle fan geometry.

        Used for rendering junction areas in the road network.

        Args:
            context (ShowBase): The Panda3D ShowBase context.
            shape_points: List of (x, y) coordinate tuples defining the polygon vertices.
            concrete_color (Tuple[float, float, float, float]): RGBA color for
                the polygon.
            z (float): Z coordinate (height) of the polygon. Defaults to 0.01.
        """

        if len(shape_points) < 3:
            return

        gformat = GeomVertexFormat.getV3cp()
        vdata = GeomVertexData("fan", gformat, Geom.UHDynamic)
        vertex = GeomVertexWriter(vdata, "vertex")
        color_writer = GeomVertexWriter(vdata, "color")

        # write center vertex
        xs, ys = zip(*shape_points)
        cx = sum(xs) / len(xs)
        cy = sum(ys) / len(ys)
        vertex.addData3(cx, cy, z)
        color_writer.addData4f(*concrete_color)
        center_index = 0

        # write perimeter vertices
        for x, y in shape_points:
            vertex.addData3(x, y, z)
            color_writer.addData4f(*concrete_color)

        # create triangle fan
        tris = GeomTriangles(Geom.UHDynamic)
        for i in range(1, len(shape_points)):
            tris.addVertices(center_index, i, i + 1)
        tris.addVertices(center_index, len(shape_points), 1)  # Close the loop
        geom = Geom(vdata)
        geom.addPrimitive(tris)
        node = GeomNode("polygon_fan")
        node.addGeom(geom)
        nodepath = context.render.attachNewNode(node)
        nodepath.setTwoSided(True)
