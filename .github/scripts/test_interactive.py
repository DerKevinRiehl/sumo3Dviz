"""
CI test script for Interactive demo scenario.

Usage:
    python .github/scripts/test_interactive.py
"""

import sys
from pathlib import Path

# Add the src directory to Python path so we can import sumo3Dviz
repo_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(repo_root / "src"))
sys.path.insert(0, str(repo_root / "examples"))

# Import demo module
try:
    import demo_interactive  # type: ignore
except ImportError as e:
    print(f"❌ Failed to import demo_interactive: {e}")
    print("Make sure sumo3Dviz is installed: pip install -e .")
    sys.exit(1)


# CI-optimized configuration override
INTERACTIVE_OVERRIDE = {
    "rendering": {
        "headless": True,
        "record_video": True,
        "video_width_px": 640,
        "video_height_px": 480,
        "video_fps": 10,
    },
}

INTERACTIVE_MAX_FRAMES = 50  # At 10 FPS, this is 5 seconds


def main():
    """Run Interactive demo test."""
    print("\n" + "=" * 70)
    print("Testing Interactive Demo")
    print("=" * 70)

    try:
        demo_interactive.main(
            config_override=INTERACTIVE_OVERRIDE, max_frames=INTERACTIVE_MAX_FRAMES
        )

        print("✅ Interactive demo PASSED")
        sys.exit(0)

    except Exception as e:
        print(f"❌ Interactive demo FAILED with error:")
        print(f"   {type(e).__name__}: {e}")
        import traceback

        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
