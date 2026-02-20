"""
CI test script for Eulerian demo scenario.

Usage:
    python .github/scripts/test_eulerian.py
"""

import os
import sys
import tempfile
from pathlib import Path

# Add the src directory to Python path so we can import sumo3Dviz
repo_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(repo_root / "src"))
sys.path.insert(0, str(repo_root / "examples"))

# Import demo module
try:
    import demo_eulerian  # type: ignore
except ImportError as e:
    print(f"❌ Failed to import demo_eulerian: {e}")
    print("Make sure sumo3Dviz is installed: pip install -e .")
    sys.exit(1)


# CI-optimized configuration override
EULERIAN_OVERRIDE = {
    "rendering": {
        "headless": True,
        "record_video": True,
        "video_width_px": 640,
        "video_height_px": 480,
        "video_fps": 10,
    },
    "modes": {
        "eulerian": {
            "simtime_start": 40.0,
            "simtime_end": 45.0,  # 5 seconds
            "ego_identifier": "flow_0_car_aggr1_route_E3_AEnd_lane0.0",
            "camera_position": {
                "pos_x": 19357.74,
                "pos_y": 17808.70,
                "pos_z": 11.95,
                "ori_h": 90.0,
                "ori_p": -10.0,
                "ori_r": 0.0,
            },
        }
    },
}


def verify_output(output_path: str, min_size_bytes: int = 10 * 1024) -> bool:
    """Verify that the output file exists and has a valid size."""
    if not os.path.exists(output_path):
        print(f"❌ Output file not found: {output_path}")
        return False

    file_size = os.path.getsize(output_path)
    if file_size < min_size_bytes:
        print(
            f"❌ Output file too small: {file_size} bytes (expected >= {min_size_bytes})"
        )
        return False

    print(f"✅ Output file valid: {file_size:,} bytes")
    return True


def main():
    """Run Eulerian demo test."""
    print("\n" + "=" * 70)
    print("Testing Eulerian Demo")
    print("=" * 70)

    # Create temporary directory for output file
    with tempfile.TemporaryDirectory() as tmpdir:
        output_path = os.path.join(tmpdir, "ci_test_eulerian.avi")

        try:
            demo_eulerian.main(
                config_override=EULERIAN_OVERRIDE, output_file_override=output_path
            )

            if verify_output(output_path):
                print("✅ Eulerian demo PASSED")
                sys.exit(0)
            else:
                print("❌ Eulerian demo FAILED (output verification)")
                sys.exit(1)

        except Exception as e:
            print(f"❌ Eulerian demo FAILED with error:")
            print(f"   {type(e).__name__}: {e}")
            import traceback

            traceback.print_exc()
            sys.exit(1)


if __name__ == "__main__":
    main()
