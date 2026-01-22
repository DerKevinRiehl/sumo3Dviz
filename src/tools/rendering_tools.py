import numpy as np
import sumolib
import warnings
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
)
from panda3d.core import CardMaker, LColor, Filename, TextureStage
from panda3d.core import LineSegs, FontPool, TextNode, Texture
from panda3d.core import DirectionalLight, AmbientLight, Vec4


class RenderingTools:
    # TODO: add docstring

    def create_light(self, context: ShowBase):
        """
        # TODO: add docstring
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
        sky_texture_file: str,
        horizon_distance=50000,
    ):
        """
        # TODO: add docstring
        """
        if context.loader is None:
            raise ValueError("Context loader is not initialized.")

        # generate an inverted sphere model
        print("Rendering sky...")
        sky_sphere: NodePath = context.loader.loadModel("models/smiley")
        sky_sphere.reparentTo(context.render)
        sky_sphere.setScale(horizon_distance)
        sky_sphere.setTwoSided(True)

        # load the texture
        sky_texture = Texture()
        sky_texture.read(
            Filename(Filename.fromOsSpecific(sky_texture_file).get_fullpath())
        )
        sky_sphere.setTexture(sky_texture, 1)

        # set rendering properties
        sky_sphere.setBin("background", 0)
        sky_sphere.setDepthWrite(False)
        sky_sphere.setLightOff()
        print("Sky rendered ✓")

    def create_floor(self, context: ShowBase, path_floor_texture: str, INFINITY=50000):
        """
        # TODO: add docstring
        """
        if context.loader is None:
            raise ValueError("Context loader is not initialized.")

        # set ground parameters
        print("Rendering ground floor...")
        cm_ground = CardMaker("ground")
        cm_ground.setFrame(-INFINITY, INFINITY, -INFINITY, INFINITY)
        ground = context.render.attachNewNode(cm_ground.generate())
        ground.setPos(0, 0, -0.05)
        ground.setHpr(25, -90, 0)

        # load the texture
        ground_tex = context.loader.loadTexture(
            Filename.fromOsSpecific(path_floor_texture).get_fullpath()
        )
        ground_tex.setWrapU(Texture.WM_repeat)
        ground_tex.setWrapV(Texture.WM_repeat)
        ground.setTexture(ground_tex)
        ground.setTexScale(TextureStage.getDefault(), 40000, 40000)
        print("Ground floor rendered ✓")

    def create_trees(
        self,
        context: ShowBase,
        tree_positions: Union[list[list[float]], None],
        tree_model_file_1: str,
        tree_model_file_2: str,
        tree_scale_1=0.2,
        tree_scale_2=0.5,
        tree_size_variability=1,
        tree_color_variability=0.1,
    ):
        """
        # TODO: add docstring
        """

        if context.loader is None:
            raise ValueError("Context loader is not initialized.")

        if tree_positions is None:
            warnings.warn("No tree positions provided. Skipping tree creation.")
            return []

        # load tree models
        print("Rendering trees...")
        tree1_model: NodePath = context.loader.loadModel(
            Filename.fromOsSpecific(tree_model_file_1).get_fullpath()
        )
        tree2_model: NodePath = context.loader.loadModel(
            Filename.fromOsSpecific(tree_model_file_2).get_fullpath()
        )

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
        """
        # TODO: add docstring
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
        store_model_file: str,
    ):
        """
        # TODO: add docstring
        """

        if context.loader is None:
            raise ValueError("Context loader is not initialized.")

        if shop_positions is None:
            warnings.warn("No shop positions provided. Skipping shop creation.")
            return [], None

        # load shop model
        print("Rendering shops...")
        building: NodePath = context.loader.loadModel(
            Filename.fromOsSpecific(store_model_file).get_fullpath()
        )

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
        home_model_file: str,
    ):
        """
        # TODO: add docstring
        """

        if context.loader is None:
            raise ValueError("Context loader is not initialized.")

        if homes_positions is None:
            warnings.warn("No home positions provided. Skipping home creation.")
            return [], None

        # load home model
        print("Rendering homes...")
        building: NodePath = context.loader.loadModel(
            Filename.fromOsSpecific(home_model_file)
        )

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
        block_model_file: str,
    ):
        """
        # TODO: add docstring
        """

        if context.loader is None:
            raise ValueError("Context loader is not initialized.")

        if block_positions is None:
            warnings.warn("No block positions provided. Skipping block creation.")
            return [], None

        # load block model
        print("Rendering building blocks...")
        building: NodePath = context.loader.loadModel(
            Filename.fromOsSpecific(block_model_file)
        )

        # generate blocks
        block_instances = []
        for position in block_positions:
            tree_instance = building.copyTo(context.render)
            tree_instance.setColor(
                0.3 + 0.7 * np.random.random(),
                0.3 + 0.7 * np.random.random(),
                0.3 + 0.7 * np.random.random(),
                1,
            )
            tree_instance.setPos(position[0], position[1], 0)
            tree_instance.setHpr(90, 90, 0)
            block_instances.append(tree_instance)

        print("Building blocks rendered ✓")

    def create_road_network(
        self,
        context: ShowBase,
        sumo_network_file: str,
        lane_width: float,
        sep_line_width: float,
        concrete_color: Tuple[float, float, float, float] = (0.2, 0.2, 0.2, 1),
        separator_color: Tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1),
    ):
        """
        # TODO: add docstring
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
                self._draw_road_edge_lane(
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
                self._draw_polygon_fan(
                    context=context,
                    shape_points=junction_shape,
                    concrete_color=concrete_color,
                )

        print("Road network rendered ✓")

    def _draw_road_edge_lane(
        self,
        context: ShowBase,
        lane_shape,
        lane_width: float,
        sep_line_width: float,
        lane_id: str,
        concrete_color: Tuple[float, float, float, float],
        separator_color: Tuple[float, float, float, float],
    ):
        """
        # TODO: add docstring
        """
        self._draw_concrete(
            context=context,
            lane_shape=lane_shape,
            lane_width=lane_width,
            color=concrete_color,
        )
        if lane_id == "b":
            self._draw_white_seperator_line_left(
                context=context,
                lane_shape=lane_shape,
                lane_width=lane_width,
                sep_line_width=sep_line_width,
                color=separator_color,
            )
            self._draw_white_seperator_line_right(
                context=context,
                lane_shape=lane_shape,
                lane_width=lane_width,
                sep_line_width=sep_line_width,
                color=separator_color,
            )
        if lane_id == "a":
            self._draw_white_seperator_line_right(
                context=context,
                lane_shape=lane_shape,
                lane_width=lane_width,
                sep_line_width=sep_line_width,
                color=separator_color,
            )
            self._draw_white_separator_line_right_dashed(
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
            self._draw_white_seperator_line_left(
                context=context,
                lane_shape=lane_shape,
                lane_width=lane_width,
                sep_line_width=sep_line_width,
                color=separator_color,
            )
        elif lane_id == "i":
            self._draw_white_separator_line_right_dashed(
                context=context,
                lane_shape=lane_shape,
                lane_width=lane_width,
                sep_line_width=sep_line_width,
                color=separator_color,
                z=0.03,
                dash_length=1.0,
                gap_length=1.0,
            )

    def _draw_concrete(
        self,
        context: ShowBase,
        lane_shape,
        lane_width: float,
        color: Tuple[float, float, float, float],
        z=0.01,
    ):
        """
        # TODO: add docstring
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

    def _draw_white_seperator_line_left(
        self,
        context: ShowBase,
        lane_shape,
        lane_width: float,
        sep_line_width: float,
        color: Tuple[float, float, float, float],
        z=0.03,
    ):
        """
        # TODO: add docstring
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

    def _draw_white_seperator_line_right(
        self,
        context: ShowBase,
        lane_shape,
        lane_width: float,
        sep_line_width: float,
        color: Tuple[float, float, float, float],
        z=0.03,
    ):
        """
        # TODO: add docstring
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
    def _draw_white_separator_line_right_dashed(
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
        """
        # TODO: add docstring
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

    def _draw_polygon_fan(
        self,
        context: ShowBase,
        shape_points,
        concrete_color: Tuple[float, float, float, float],
        z=0.01,
    ):
        """
        # TODO: add docstring
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
