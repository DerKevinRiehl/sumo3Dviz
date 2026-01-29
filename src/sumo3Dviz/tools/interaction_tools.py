from typing import cast
from direct.showbase.ShowBase import ShowBase
from panda3d.core import Camera


class InteractionTools:
    """Provides tools for handling user interaction with the 3D visualization."""

    def addCameraControlKeyboard(self, context: ShowBase):
        """Configure keyboard controls for camera movement and rotation.

        Sets up arrow keys for camera position (left/right/forward/backward),
        W/S keys for vertical movement (up/down), Q/A keys for pitch rotation
        (look up/down), and E/D keys for heading rotation (look left/right).

        Args:
            context (ShowBase): The Panda3D ShowBase context to attach keyboard
                controls to.
        """

        # move camera left/right/forward/backward
        def move_left():
            cast(Camera, context.camera).setX(context.camera, -1)

        def move_right():
            cast(Camera, context.camera).setX(context.camera, 1)

        def move_forward():
            cast(Camera, context.camera).setY(context.camera, 1)

        def move_backward():
            cast(Camera, context.camera).setY(context.camera, -1)

        # move camera up/down (w/s)
        def move_up():
            cast(Camera, context.camera).setZ(context.camera, 1)

        def move_down():
            cast(Camera, context.camera).setZ(context.camera, -1)

        # look up/down (q/a) -- rotate pitch
        def look_up():
            cast(Camera, context.camera).setP(cast(Camera, context.camera).getP() + 5)

        def look_down():
            cast(Camera, context.camera).setP(cast(Camera, context.camera).getP() - 5)

        # look left/right (e/d) -- rotate heading
        def look_left():
            cast(Camera, context.camera).setH(cast(Camera, context.camera).getH() + 5)

        def look_right():
            cast(Camera, context.camera).setH(cast(Camera, context.camera).getH() - 5)

        context.accept("arrow_left", move_left)
        context.accept("arrow_right", move_right)
        context.accept("arrow_up", move_forward)
        context.accept("arrow_down", move_backward)
        context.accept("w", move_up)
        context.accept("s", move_down)
        context.accept("q", look_up)
        context.accept("a", look_down)
        context.accept("e", look_left)
        context.accept("d", look_right)
        context.disableMouse()
