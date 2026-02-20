"""
CI test script for running all demo scenarios.

This script imports and runs all four demo modules (lagrangian, eulerian,
cinematic, interactive) with CI-optimized configurations:
- Headless rendering (no display required)
- Short simulation times (3-5 seconds)
- Low resolution and FPS for speed
- Frame limit for interactive mode

After each demo runs, the script verifies that the output file was created
and has a valid size.

Usage:
    python .github/scripts/test_demos.py
"""

import os
import sys
import tempfile
from pathlib import Path
from typing import Union

# Add the src directory to Python path so we can import sumo3Dviz
repo_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(repo_root / "src"))
sys.path.insert(0, str(repo_root / "examples"))

# Import demo modules
try:
    from examples import (
        demo_lagrangian,
        demo_eulerian,
        demo_cinematic,
        demo_interactive,
    )
except ImportError as e:
    print(f"❌ Failed to import demo modules: {e}")
    print("Make sure sumo3Dviz is installed: pip install -e .")
    sys.exit(1)


# CI-optimized configuration overrides
CI_CONFIG_OVERRIDE = {
    "rendering": {
        "headless": True,
        "record_video": True,
        "video_width_px": 640,
        "video_height_px": 480,
        "video_fps": 10,
    },
}

# Mode-specific overrides for short simulation times
LAGRANGIAN_OVERRIDE = {
    **CI_CONFIG_OVERRIDE,
    "modes": {
        "lagrangian": {
            "simtime_start": 40.0,
            "simtime_end": 45.0,  # 5 seconds
        }
    },
}

EULERIAN_OVERRIDE = {
    **CI_CONFIG_OVERRIDE,
    "modes": {
        "eulerian": {
            "simtime_start": 40.0,
            "simtime_end": 45.0,  # 5 seconds
        }
    },
}

CINEMATIC_OVERRIDE = {
    **CI_CONFIG_OVERRIDE,
    "modes": {
        "cinematic": {
            "simtime_start": 142.0,
            "simtime_end": 145.0,  # 3 seconds
        }
    },
}

INTERACTIVE_OVERRIDE = CI_CONFIG_OVERRIDE.copy()
INTERACTIVE_MAX_FRAMES = 50  # At 10 FPS, this is 5 seconds


def verify_output(output_path: str, min_size_bytes: int = 10 * 1024) -> bool:
    """
    Verify that the output file exists and has a valid size.

    Args:
        output_path: Path to the output video file
        min_size_bytes: Minimum expected file size (default 10KB)

    Returns:
        True if file exists and size is valid, False otherwise
    """
    if not os.path.exists(output_path):
        print(f"  ❌ Output file not found: {output_path}")
        return False

    file_size = os.path.getsize(output_path)
    if file_size < min_size_bytes:
        print(
            f"  ❌ Output file too small: {file_size} bytes (expected >= {min_size_bytes})"
        )
        return False

    print(f"  ✅ Output file valid: {file_size:,} bytes")
    return True


def run_demo(
    demo_name: str,
    demo_module,
    config_override: dict,
    output_path: Union[str, None] = None,
    max_frames: Union[int, None] = None,
) -> bool:
    """
    Run a single demo with the given configuration.

    Args:
        demo_name: Name of the demo for logging
        demo_module: The demo module to run
        config_override: Configuration overrides
        output_path: Path where output video should be saved
        max_frames: Maximum frames to render (for interactive mode only)

    Returns:
        True if demo ran successfully and output is valid, False otherwise
    """
    print(f"\n{'=' * 70}")
    print(f"Running {demo_name} demo...")
    print(f"{'=' * 70}")

    try:
        if max_frames is not None:
            # Interactive mode with frame limit
            demo_module.main(config_override=config_override, max_frames=max_frames)
        else:
            # Other modes with output file override
            demo_module.main(
                config_override=config_override, output_file_override=output_path
            )

        # For interactive mode, we don't generate an output file
        if max_frames is not None:
            print(f"  ✅ {demo_name} demo completed successfully (interactive mode)")
            return True

        # Verify output file
        if config_override is None:
            print(
                f"  ✅ {demo_name} demo completed successfully, no configuration override provided = interactive mode"
            )
            return True
        elif output_path is not None and verify_output(output_path):
            print(f"  ✅ {demo_name} demo completed successfully")
            return True
        else:
            print(f"  ❌ {demo_name} demo failed output verification")
            return False

    except Exception as e:
        print(f"  ❌ {demo_name} demo failed with error:")
        print(f"     {type(e).__name__}: {e}")
        import traceback

        traceback.print_exc()
        return False


def main():
    """Run all demo tests."""
    print("\n" + "=" * 70)
    print("CI Demo Tests")
    print("=" * 70)
    print(f"Repository root: {repo_root}")

    # Create temporary directory for output files
    with tempfile.TemporaryDirectory() as tmpdir:
        print(f"Output directory: {tmpdir}")

        results = {}

        # Test Lagrangian mode
        lagrangian_output = os.path.join(tmpdir, "ci_test_lagrangian.avi")
        results["Lagrangian"] = run_demo(
            "Lagrangian", demo_lagrangian, LAGRANGIAN_OVERRIDE, lagrangian_output
        )

        # Test Eulerian mode
        eulerian_output = os.path.join(tmpdir, "ci_test_eulerian.avi")
        results["Eulerian"] = run_demo(
            "Eulerian", demo_eulerian, EULERIAN_OVERRIDE, eulerian_output
        )

        # Test Cinematic mode
        cinematic_output = os.path.join(tmpdir, "ci_test_cinematic.avi")
        results["Cinematic"] = run_demo(
            "Cinematic", demo_cinematic, CINEMATIC_OVERRIDE, cinematic_output
        )

        # Test Interactive mode (with frame limit)
        results["Interactive"] = run_demo(
            "Interactive",
            demo_interactive,
            INTERACTIVE_OVERRIDE,
            None,  # No output file for interactive mode
            max_frames=INTERACTIVE_MAX_FRAMES,
        )

    # Print summary
    print("\n" + "=" * 70)
    print("Test Summary")
    print("=" * 70)

    for demo_name, success in results.items():
        status = "✅ PASSED" if success else "❌ FAILED"
        print(f"  {demo_name:15} {status}")

    # Exit with appropriate code
    all_passed = all(results.values())
    if all_passed:
        print("\n✅ All demos passed!")
        sys.exit(0)
    else:
        failed_count = sum(1 for success in results.values() if not success)
        print(f"\n❌ {failed_count} demo(s) failed")
        sys.exit(1)


if __name__ == "__main__":
    main()
