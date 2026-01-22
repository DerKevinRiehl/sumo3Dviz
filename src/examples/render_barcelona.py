# This script renders a 3D visualization of a SUMO simulation scenario in Barcelona
# using the available classes directly and configuring them through corresponding
# properties. For an illustration on how to use this library through the CLI,
# please refer to the render_barcelona_cli.py script and the corresponding
# configuration file / command line arguments described there.
import os

from src.tools.loader_tools import LoaderTools

if __name__ == "__main__":
    # ! CONFIGURATION PARAMETERS (passed directly to classes / functions)
    # ? alternative config-based approach in `render_barcelona_cli.py`
    # region
    # trajectory parameters
    trajectory_file = os.path.join(
        os.path.dirname(__file__),
        "../../examples/barcelona_simulation/simulation_logs/vehicle_positions.xml",
    )
    ego_identifier = "flow_0_car_aggr1_route_E3_AEnd_lane0.0"
    simtime_start = 39.0
    simtime_end = 369.0

    # video parameters
    video_fps = 25

    # visualization parameters
    show_other_vehicles = True
    # endregion

    # ! LOADER FUNCTIONS
    # region
    loader = LoaderTools()

    # load the trajectories from the SUMO log and apply trajectory smoothing to them
    (
        df_ego_smoothed,
        smoothened_trajectory_data,
        VIDEO_CURRENT_POINT,
        VIDEO_FINAL_POINT,
    ) = loader.load_trajectory(
        trajectory_file=trajectory_file,
        ego_identifier=ego_identifier,
        simtime_start=simtime_start,
        simtime_end=simtime_end,
        video_fps=video_fps,
        show_other_vehicles=show_other_vehicles,
    )
    # endregion

    # TODO: MAKE SURE TO MAKE ALL THE SAME CHANGES ALSO IN THE CLI SCRIPT
