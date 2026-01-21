"""
sumo3Dviz - A three dimensional visualization of traffic simulations with SUMO
==============================================================================================
Organization: Institute for Transport Planning and Systems (IVT), ETH Zürich
Authors: Kevin Riehl <kriehl@ethz.ch>, Julius Schlapbach <juliussc@ethz.ch>
Submitted to: SUMO User Conference 2026
"""

# #############################################################################
# # METHODS
# #############################################################################

def addCameraControlKeyboard(context):
    # Move camera left/right/forward/backward
    def move_left(): 
        context.camera.setX(context.camera, -1)
    def move_right(): 
        context.camera.setX(context.camera, 1)
    def move_forward(): 
        context.camera.setY(context.camera, 1)
    def move_backward(): 
        context.camera.setY(context.camera, -1)
    # Move camera up/down (w/s)
    def move_up():
        context.camera.setZ(context.camera, 1)
    def move_down():
        context.camera.setZ(context.camera, -1)
    # Look up/down (q/a) -- rotate pitch
    def look_up():
        context.camera.setP(context.camera.getP() + 5)
    def look_down():
        context.camera.setP(context.camera.getP() - 5)
    # Look left/right (e/d) -- rotate heading
    def look_left():
        context.camera.setH(context.camera.getH() + 5)
    def look_right():
        context.camera.setH(context.camera.getH() - 5)
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
