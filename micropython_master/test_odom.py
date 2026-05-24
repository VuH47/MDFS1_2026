"""
test_odom.py
Basic odometry bring-up using the 20 ms RateTimer + encoders + new Odometry class.

Run after Phase 1/2 fixes:

    >>> import test_odom
    >>> test_odom.run(duration_s=8, speed=35)

Drive the robot ~1 meter straight on the floor, note the final (x,y,theta),
then use that to calibrate `m_per_count` in odometry/odometry.py or config.

Gyro fusion is now available via sensors.mpu6500 (see test_imu.py for full example).
"""

import time
import math
from utils.timing import RateTimer
from drivers.quad_encoder import QuadEncoder
from odometry.odometry import Odometry
from robot import drive
import config


def run(duration_s: float = 8.0, speed: float = 35.0, period_ms: int = 20):
    print("=" * 60)
    print(f" ODOMETRY TEST  |  {duration_s}s @ {speed}%  |  20 ms loop")
    print(" Watch x/y/theta integrate. Drive 1 m straight for calibration.")
    print("=" * 60)

    el = QuadEncoder(config.ENCODER_LEFT_A, config.ENCODER_LEFT_B,
                     invert=config.ENCODER_LEFT_INVERT, name="left")
    er = QuadEncoder(config.ENCODER_RIGHT_A, config.ENCODER_RIGHT_B,
                     invert=config.ENCODER_RIGHT_INVERT, name="right")

    odo = Odometry()
    timer = RateTimer(period_ms=period_ms)

    start = time.ticks_ms()
    total_left = 0.0
    total_right = 0.0

    try:
        drive.set_side_speeds(speed, speed)

        while time.ticks_diff(time.ticks_ms(), start) < int(duration_s * 1000):
            dt = timer.wait() / 1000.0          # seconds

            dl_counts = el.get_delta()
            dr_counts = er.get_delta()

            dl = dl_counts * odo.m_per_count
            dr = dr_counts * odo.m_per_count

            odo.update(dl, dr, dt)              # pure wheel; pass gyro_z=mpu.gyro_z for fusion

            total_left += dl
            total_right += dr

            if timer.iteration % 25 == 0:       # ~every 0.5 s
                x, y, th = odo.get_pose()
                v, w = odo.get_velocities()
                rear_l, rear_r = drive.get_rear_directions()
                print(f"[{timer.iteration:4d}] x={x:6.3f} y={y:6.3f} θ={th:5.2f} rad  "
                      f"v={v:5.3f}  rear=({rear_l},{rear_r})  totalL={total_left:6.3f}m")

        drive.brake_all()
        x, y, th = odo.get_pose()
        print("\n=== FINAL POSE ===")
        print(f"x = {x:.4f} m")
        print(f"y = {y:.4f} m")
        print(f"θ = {th:.4f} rad  ({math.degrees(th):.1f}°)")
        print(f"Total left distance:  {total_left:.4f} m")
        print(f"Total right distance: {total_right:.4f} m")
        print("\nCalibration tip: if you drove exactly 1.00 m straight,")
        print("new m_per_count ≈ (1.0) / ((totalL + totalR)/2 / current_m_per_count)")

    except KeyboardInterrupt:
        drive.brake_all()
        print("\nInterrupted.")
    finally:
        el.deinit()
        er.deinit()
        drive.deinit_all()
        print("Odom test clean shutdown.")


if __name__ == "__main__":
    run()