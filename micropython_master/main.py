"""
main.py - Post-Calibration Startup (safe REPL entry)

After odometry calibration is complete (m_per_count persisted in /calibration.json),
this is the normal boot entry point.

Behavior:
- Prints system status + the active m_per_count (loaded from calibration.json if present)
- Does NOT drive motors automatically (safety).
- Drops to REPL so you can manually run tests, the wifi_agent, or your own loops.

Recommended usage:
    # On device (via WebREPL or mpremote)
    import main          # shows status
    import tools.wifi_agent   # for wireless drive + telemetry UI

For a quick odometry verification drive (no auto-start on every power-on):
    import test_integrated_loop as t
    t.run_straight(20, 3)   # safe low speed, short time
"""

import time
import sys
import config
from odometry.odometry import Odometry

print("=" * 65)
print("  4WD ROBOT - POST CALIBRATION STARTUP")
print("  (calibration.json is now loaded automatically by Odometry)")
print("=" * 65)

# Instantiate to trigger the load print + show current scale
odo = Odometry()
print(f"Active m_per_count : {odo.m_per_count:.6e} m/count")
print(f"Wheel diameter     : {config.WHEEL_DIAMETER_MM} mm")
print(f"Track width (TODO) : {config.TRACK_WIDTH_MM} mm  <-- measure accurately for good turns")
print(f"Control period     : {config.CONTROL_LOOP_PERIOD_MS} ms")
print()

# Quick sanity: if calibration.json was loaded the value will be the measured one (~1.49e-3)
# instead of the pure geometric fallback (~1.31e-4).

print("Motors are SAFE - no automatic motion on boot.")
print("To start wireless control + telemetry UI:")
print("    import tools.wifi_agent")
print()
print("For a short verification drive (measure vs odometry):")
print("    import test_integrated_loop as t")
print("    t.run_straight(speed=15, duration_s=4)   # very safe")
print("=" * 65)
print("Ready. Dropping to REPL...")