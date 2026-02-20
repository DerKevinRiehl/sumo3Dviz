import os
import yaml
import json
from jsonschema import validate, ValidationError


def load_configuration(config_path: str) -> dict:
    """
    Load the configuration file from the specified path and return it as a dictionary.

    Args:
        config_path: Path to the YAML configuration file

    Returns:
        Configuration as a dictionary
    """
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

    return config


def validate_configuration(config: dict, mode: str) -> None:
    """
    Validate the configuration dictionary against the schema.

    Args:
        config: Configuration dictionary to validate
        mode: Mode string ('interactive', 'eulerian', 'lagrangian', 'cinematic')
              for mode-specific validation

    Raises:
        ValueError: If validation fails
        SystemExit: If critical validation errors occur
    """
    # temporarily add mode to config for schema validation
    config_with_mode = config.copy()
    config_with_mode["_validation_mode"] = mode

    # validate the configuration against the schema
    _schema_path = os.path.join(os.path.dirname(__file__), "config_schema.json")
    with open(_schema_path, "r") as f:
        schema = json.load(f)

    try:
        validate(instance=config_with_mode, schema=schema)
        print("✓ Configuration file validated successfully")
    except ValidationError as e:
        error_msg = f"Configuration validation error: {e.message}"
        error_path = " -> ".join(str(p) for p in e.path)
        if error_path:
            error_msg += f"\nFailed at: {error_path}"
        raise ValueError(error_msg)

    # validate texture selections against available options
    available_sky_textures = [
        "sky_blue",
        "sky_cloudy",
        "sky_overcast",
        "sky_dawn",
        "sky_night_stars",
        "sky_night_clear",
        "sky_night_forest",
        "sky_night_desert",
        "sky_halloween",
    ]
    available_ground_textures = [
        "ground_grass",
        "ground_stone",
        "ground_sand",
        "ground_chess",
        "ground_chesslarge",
        "ground_halloween",
    ]

    sky_texture = config["visualization"].get("sky_texture", "sky_cloudy")
    ground_texture = config["visualization"].get("ground_texture", "ground_grass")

    if sky_texture not in available_sky_textures:
        raise ValueError(
            f"Sky texture '{sky_texture}' is not available. Please choose from: {available_sky_textures}"
        )

    if ground_texture not in available_ground_textures:
        raise ValueError(
            f"Ground texture '{ground_texture}' is not available. Please choose from: {available_ground_textures}"
        )
