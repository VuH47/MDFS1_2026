"""
robot package
Unified high-level drive interface for the 4WD robot.

Currently (Phase 1): thin wrapper around tb6612 (front) + l298n (rear)
using the "same command per side" rule defined in DESIGN.md.

Later phases will add:
- set_velocity(v, omega) kinematics
- PID speed loop on front
- odometry feedback
"""

from . import drive  # noqa: F401
