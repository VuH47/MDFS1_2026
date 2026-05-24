"""
test_integrated_loop.py
Full stack 20 ms control loop test — now that front + rear are wired and tested.

This combines:
- RateTimer (20 ms, excellent jitter proven in your tests)
- robot.drive (front TB6612 + rear L298N with same-command-per-side)
- Encoders (with correct inverts from config)
- Odometry (wheel-based for now)

Use this to:
- Verify the complete robot moves as expected
- Collect data for m_per_count calibration
- See live pose (x, y, theta) and velocities

Run examples:

    import test_integrated_loop as t
    t.run_straight(30, 8)     # 30% speed for 8 seconds straight
    t.run_pivot(25, 5)        # pivot test
    t.run_calibration_drive() # helper for 1m straight calibration
"""

import time
import math
from utils.timing import RateTimer
from drivers.quad_encoder import QuadEncoder
from odometry.odometry import Odometry
from robot import drive
import config


def _create_encoders():
    return (
        QuadEncoder(config.ENCODER_LEFT_A, config.ENCODER_LEFT_B,
                    invert=config.ENCODER_LEFT_INVERT, name="left"),
        QuadEncoder(config.ENCODER_RIGHT_A, config.ENCODER_RIGHT_B,
                    invert=config.ENCODER_RIGHT_INVERT, name="right"),
    )


def run_straight(speed: float = 30.0, duration_s: float = 8.0, period_ms: int = 20):
    """
    Drive straight while printing odometry.
    Good for calibration and basic verification.
    """
    print("=" * 65)
    print(f" INTEGRATED 20ms LOOP  |  STRAIGHT {speed}%  |  {duration_s}s")
    print(" Full chassis (front + rear) + odometry")
    print("=" * 65)

    el, er = _create_encoders()
    odo = Odometry()
    timer = RateTimer(period_ms)

    start = time.ticks_ms()
    odo.reset()

    try:
        drive.set_side_speeds(speed, speed)

        while time.ticks_diff(time.ticks_ms(), start) < int(duration_s * 1000):
            dt = timer.wait() / 1000.0

            dl = el.get_delta()
            dr = er.get_delta()
            odo.update_from_encoders(dl, dr, dt)

            if timer.iteration % 10 == 0:   # every ~200 ms
                x, y, th = odo.get_pose()
                v, w = odo.get_velocities()
                rear_l, rear_r = drive.get_rear_directions()
                print(f"[{timer.iteration:4d}] x={x:6.3f} y={y:6.3f} θ={math.degrees(th):6.1f}°  "
                      f"v={v:5.3f}  rear=({rear_l},{rear_r})")

        drive.brake_all()

        x, y, th = odo.get_pose()
        print("\n=== FINAL POSE ===")
        print(f"x = {x:.4f} m")
        print(f"y = {y:.4f} m")
        print(f"θ = {math.degrees(th):.2f} °")
        print(f"Total left distance:  {odo.last_v * duration_s:.3f} m (approx)")

    except KeyboardInterrupt:
        print("\nInterrupted!")
        drive.brake_all()
    finally:
        el.deinit()
        er.deinit()
        drive.deinit_all()
        print("Integrated loop stopped cleanly.")


def run_pivot(speed: float = 25.0, duration_s: float = 6.0):
    """Pivot in place (good test for odometry turning)."""
    print("=" * 65)
    print(f" INTEGRATED 20ms LOOP  |  PIVOT  |  {speed}%  |  {duration_s}s")
    print("=" * 65)

    el, er = _create_encoders()
    odo = Odometry()
    timer = RateTimer(20)

    start = time.ticks_ms()
    odo.reset()

    try:
        # Left backward, right forward → pivot
        drive.set_side_speeds(-speed, speed)

        while time.ticks_diff(time.ticks_ms(), start) < int(duration_s * 1000):
            dt = timer.wait() / 1000.0

            dl = el.get_delta()
            dr = er.get_delta()
            odo.update_from_encoders(dl, dr, dt)

            if timer.iteration % 15 == 0:
                x, y, th = odo.get_pose()
                rear_l, rear_r = drive.get_rear_directions()
                print(f"[{timer.iteration:4d}] θ={math.degrees(th):6.1f}°   "
                      f"rear=({rear_l},{rear_r})")

        drive.brake_all()
        _, _, th = odo.get_pose()
        print(f"\nFinal heading change: {math.degrees(th):.1f} °")

    except KeyboardInterrupt:
        drive.brake_all()
    finally:
        el.deinit()
        er.deinit()
        drive.deinit_all()


def run_calibration_drive(speed: float = 30.0, target_distance_m: float = 1.0):
    """
    Helper to drive roughly 1 meter straight for odometry calibration.
    After the run, use the printed total counts to calculate m_per_count.
    """
    print("=== ODOMETRY CALIBRATION RUN ===")
    print(f"Drive the robot as straight as possible for ~{target_distance_m}m")
    print("Then measure the actual distance traveled on the ground.\n")

    # Run a straight segment
    run_straight(speed, duration_s=12.0)   # will stop early if you interrupt

    print("\nAfter the run, measure the real distance the robot traveled.")
    print("Then we can compute the correct m_per_count for your robot.")


if __name__ == "__main__":
    # Default action when running the file directly
    run_straight(30, 6)