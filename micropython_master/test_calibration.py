"""
test_calibration.py

Simple, focused script for odometry calibration.

What it does:
- Drives the robot straight forward at constant speed.
- Accumulates raw encoder counts from both sides.
- At the end, prints total counts and estimated distance (using current m_per_count).
- Gives you the exact formula to calculate the correct m_per_count.

Usage (recommended):

    mpremote run test_calibration.py

    # Or with custom duration and speed:
    mpremote run test_calibration.py 10 25     # 10 seconds at 25% speed

After running:
1. Measure the real straight-line distance the robot traveled on the floor.
2. Use the printed numbers to compute the new m_per_count.

Example output you will see:
    Total counts Left :  2840
    Total counts Right:  2815
    ...
    Suggested command:
        odo.m_per_count = 1.00 / 2827.5
"""

import sys
import time
from utils.timing import RateTimer
from drivers.quad_encoder import QuadEncoder
from robot import drive
import config
from odometry.odometry import Odometry


def run(duration_s: float = 8.0, speed: float = 30.0, period_ms: int = 20):
    print("=" * 60)
    print(" ODOMETRY CALIBRATION RUN")
    print(f" Speed: {speed}%   |   Duration: {duration_s}s")
    print(" Drive as straight as possible. Measure real distance afterwards.")
    print("=" * 60)

    el = QuadEncoder(config.ENCODER_LEFT_A, config.ENCODER_LEFT_B,
                     invert=config.ENCODER_LEFT_INVERT, name="left")
    er = QuadEncoder(config.ENCODER_RIGHT_A, config.ENCODER_RIGHT_B,
                     invert=config.ENCODER_RIGHT_INVERT, name="right")

    odo = Odometry()                    # used only to read current m_per_count
    timer = RateTimer(period_ms)

    total_left_counts = 0
    total_right_counts = 0

    start_time = time.ticks_ms()
    end_time = start_time + int(duration_s * 1000)

    drive.set_side_speeds(speed, speed)

    try:
        while time.ticks_diff(end_time, time.ticks_ms()) > 0:
            timer.wait()
            total_left_counts  += el.get_delta()
            total_right_counts += er.get_delta()

        drive.brake_all()

        avg_counts = (total_left_counts + total_right_counts) / 2.0
        est_dist_left  = total_left_counts  * odo.m_per_count
        est_dist_right = total_right_counts * odo.m_per_count

        print("\n" + "=" * 60)
        print(" CALIBRATION RESULTS")
        print("=" * 60)
        print(f"Total counts Left : {total_left_counts:7d}")
        print(f"Total counts Right: {total_right_counts:7d}")
        print(f"Average counts    : {avg_counts:8.1f}")
        print()
        print(f"Current m_per_count : {odo.m_per_count:.6f}")
        print(f"Estimated dist Left : {est_dist_left:.4f} m")
        print(f"Estimated dist Right: {est_dist_right:.4f} m")
        print()
        print(">>> NEXT STEP <<<")
        print("1. Measure the REAL straight distance the robot traveled (in meters).")
        print("2. Calculate the correct scale with this formula:")
        print()
        print(f"   new_m_per_count = REAL_DISTANCE / {avg_counts:.1f}")
        print()
        print("3. Update odometry/odometry.py with the new value:")
        print(f"   self.m_per_count = {odo.m_per_count:.6f}   # <-- replace this")
        print()
        print("Example (if you measured 1.05 m):")
        print(f"   self.m_per_count = 1.05 / {avg_counts:.1f}")
        print("=" * 60)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        drive.brake_all()
    finally:
        el.deinit()
        er.deinit()
        drive.deinit_all()
        print("\nCalibration run finished. Motors stopped.")


if __name__ == "__main__":
    duration = 8.0
    speed = 30.0

    if len(sys.argv) >= 2:
        try:
            duration = float(sys.argv[1])
        except ValueError:
            pass

    if len(sys.argv) >= 3:
        try:
            speed = float(sys.argv[2])
        except ValueError:
            pass

    run(duration_s=duration, speed=speed)