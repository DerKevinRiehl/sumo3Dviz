"""
CI test script for testing CLI commands.

This script tests all sumo3Dviz CLI commands using the CI configuration file:
- lagrangian mode
- eulerian mode
- cinematic mode
- interactive mode (with --max-frames)

Each command is run with a timeout to prevent hanging, and the output files
are verified to ensure they were created successfully.

Usage:
    python .github/scripts/test_cli.py
"""

import os
import sys
import subprocess
import tempfile
from pathlib import Path
from typing import Union

# Repository root
repo_root = Path(__file__).parent.parent.parent
config_ci_path = repo_root / "examples" / "config_ci.yaml"

# Check that CI config exists
if not config_ci_path.exists():
    print(f"❌ CI configuration file not found: {config_ci_path}")
    sys.exit(1)


def run_cli_command(command: list, timeout_seconds: int = 300) -> tuple:
    """
    Run a CLI command with timeout.

    Args:
        command: Command and arguments as a list
        timeout_seconds: Maximum time to wait for command completion

    Returns:
        Tuple of (success: bool, stdout: str, stderr: str, returncode: int)
    """
    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=timeout_seconds,
            cwd=repo_root,
        )

        success = result.returncode == 0
        return success, result.stdout, result.stderr, result.returncode

    except subprocess.TimeoutExpired:
        print(f"  ❌ Command timed out after {timeout_seconds} seconds")
        return False, "", f"Timeout after {timeout_seconds}s", -1
    except Exception as e:
        print(f"  ❌ Command failed with exception: {e}")
        return False, "", str(e), -1


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


def test_cli_mode(
    mode: str, output_path: Union[str, None], extra_args: Union[list, None] = None
) -> bool:
    """
    Test a single CLI mode.

    Args:
        mode: The rendering mode (lagrangian, eulerian, cinematic, interactive)
        output_path: Path where output video should be saved
        extra_args: Additional CLI arguments

    Returns:
        True if test passed, False otherwise
    """
    print(f"\n{'=' * 70}")
    print(f"Testing CLI: {mode} mode")
    print(f"{'=' * 70}")

    # Build command
    command = [
        "sumo3Dviz",
        "--config",
        str(config_ci_path),
        "--mode",
        mode,
        "--headless",
    ]

    # Add output argument for non-interactive modes
    if mode != "interactive":
        if output_path is None:
            print(f"  ❌ Output path must be provided for {mode} mode")
            return False

        command.extend(["--output", output_path])

    # Add any extra arguments
    if extra_args:
        command.extend(extra_args)

    print(f"Command: {' '.join(command)}")

    # Run command
    success, stdout, stderr, returncode = run_cli_command(command)

    if not success:
        print(f"  ❌ Command failed with return code {returncode}")
        if stderr:
            print(f"  STDERR:\n{stderr}")
        if stdout:
            print(f"  STDOUT:\n{stdout}")
        return False

    print(f"  ✅ Command executed successfully")

    # For interactive mode with max-frames, we don't generate output
    if mode == "interactive" and "--max-frames" in command:
        return True

    # Verify output file for other modes
    if mode != "interactive":
        if output_path is None:
            print(f"  ❌ Output path must be provided for {mode} mode")
            return False

        if verify_output(output_path):
            print(f"  ✅ {mode} mode test PASSED")
            return True
        else:
            print(f"  ❌ {mode} mode test FAILED (output verification)")
            return False

    return True


def main():
    """Run all CLI tests."""
    print("\n" + "=" * 70)
    print("CI CLI Tests")
    print("=" * 70)
    print(f"Repository root: {repo_root}")
    print(f"CI config: {config_ci_path}")

    # Check if sumo3Dviz CLI is available
    try:
        result = subprocess.run(
            ["sumo3Dviz", "--help"], capture_output=True, text=True, timeout=10
        )
        if result.returncode != 0:
            print("❌ sumo3Dviz CLI not available")
            print("   Make sure it's installed: pip install -e .")
            sys.exit(1)
    except (subprocess.TimeoutExpired, FileNotFoundError):
        print("❌ sumo3Dviz CLI not available")
        print("   Make sure it's installed: pip install -e .")
        sys.exit(1)

    print("✅ sumo3Dviz CLI is available")

    # Create temporary directory for output files
    with tempfile.TemporaryDirectory() as tmpdir:
        print(f"Output directory: {tmpdir}")

        results = {}

        # Test Lagrangian mode
        lagrangian_output = os.path.join(tmpdir, "cli_test_lagrangian.avi")
        results["Lagrangian"] = test_cli_mode("lagrangian", lagrangian_output)

        # Test Eulerian mode
        eulerian_output = os.path.join(tmpdir, "cli_test_eulerian.avi")
        results["Eulerian"] = test_cli_mode("eulerian", eulerian_output)

        # Test Cinematic mode
        cinematic_output = os.path.join(tmpdir, "cli_test_cinematic.avi")
        results["Cinematic"] = test_cli_mode("cinematic", cinematic_output)

        # Test Interactive mode with max-frames
        print(f"\n{'=' * 70}")
        print("Testing CLI: interactive mode with --max-frames")
        print(f"{'=' * 70}")
        results["Interactive"] = test_cli_mode(
            "interactive", None, ["--max-frames", "50"]
        )

    # Print summary
    print("\n" + "=" * 70)
    print("Test Summary")
    print("=" * 70)

    for mode_name, success in results.items():
        status = "✅ PASSED" if success else "❌ FAILED"
        print(f"  {mode_name:15} {status}")

    # Exit with appropriate code
    all_passed = all(results.values())
    if all_passed:
        print("\n✅ All CLI tests passed!")
        sys.exit(0)
    else:
        failed_count = sum(1 for success in results.values() if not success)
        print(f"\n❌ {failed_count} CLI test(s) failed")
        sys.exit(1)


if __name__ == "__main__":
    main()
