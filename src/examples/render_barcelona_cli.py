# This script renders a 3D visualization of a SUMO simulation scenario in Barcelona
# using the configuration parameters in the corresponding configuration script.
# In order for the script to work correctly, please specify the configuration script
# path and name in the command line arguments.
#
# Command:
# python src/examples/render_barcelona_cli.py --config src/examples/config_barcelona.yaml

import os
import argparse
import yaml

from src.tools.loader_tools import LoaderTools

if __name__ == "__main__":
    # ! Get configuration parameters from specified file
    # region
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        type=str,
        required=True,
        help="Path to the configuration file containing all necessary parameters to run the video generation pipeline.",
    )
    args = parser.parse_args()
    config_path = args.config

    if not config_path:
        raise ValueError(
            "Configuration file path must be specified using --config argument."
        )

    try:
        # use the full loader to get the yaml file content as a python dictionary
        with open(config_path) as file:
            config = yaml.load(file, Loader=yaml.FullLoader)
            assert type(config) is dict

    except:
        print(
            f"Could not load the configuration file, please make sure it exists at: {config_path}"
        )
        exit(1)
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
        trajectory_file=os.path.join(
            os.getcwd(), config["trajectory_parameters"]["trajectory_file"]
        ),
        ego_identifier=config["trajectory_parameters"]["ego_identifier"],
        simtime_start=config["trajectory_parameters"]["simtime_start"],
        simtime_end=config["trajectory_parameters"]["simtime_end"],
        video_fps=config["video_parameters"]["video_fps"],
        show_other_vehicles=config["visualization_parameters"]["show_other_vehicles"],
    )
    # endregion

    pass
