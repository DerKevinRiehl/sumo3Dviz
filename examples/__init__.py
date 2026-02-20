"""
sumo3Dviz Examples Package

This package contains example scripts demonstrating different visualization modes:
- demo_lagrangian: Camera follows an ego vehicle
- demo_eulerian: Fixed camera position
- demo_cinematic: Camera follows a predefined trajectory
- demo_interactive: Free camera movement with keyboard controls

Each demo module exports a main() function that can be called programmatically
with optional configuration overrides for testing purposes.
"""

# Export main functions from demo modules for programmatic access
from .demo_lagrangian import main as run_lagrangian_demo
from .demo_eulerian import main as run_eulerian_demo
from .demo_cinematic import main as run_cinematic_demo
from .demo_interactive import main as run_interactive_demo

__all__ = [
    "run_lagrangian_demo",
    "run_eulerian_demo",
    "run_cinematic_demo",
    "run_interactive_demo",
]
