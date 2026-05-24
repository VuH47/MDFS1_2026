"""
test_imu.py

MPU6500 IMU bring-up and gyro fusion demo for the 4WD robot.

Run this after wiring the I2C bus (SDA=17, SCL=5) with 4.7k pull-ups.

Recommended sequence:
    1. mpremote cp sensors/mpu6500.py :sensors/mpu6500.py
    2. mpremote cp sensors/__init__.py :sensors/__init__.py
    3. mpremote cp test_imu.py :test_imu.py
    4. On device (REPL or WebREPL):

        import test_imu
        test_imu.run_calibration_demo()     # prints bias, lets you twist the robot by hand
        # or
        test_imu.run_with_odometry_fusion(duration_s=10)

The second function shows the exact pattern you will use inside your 20 ms control loop
together with encoders + RateTimer + odometry.

Hardware notes:
- External pull-ups on SDA/SCL are required (the ESP32 pins don't have reliable ones).
- Keep the robot completely still during the bias calibration step.
"""

import time
import math
from machine import I2C, Pin
import config
from utils.timing import RateTimer
from drivers.quad_encoder import QuadEncoder
from odometry.odometry import Odometry
from robot import drive
from sensors import MPU6500


def _get_i2c():
    """Create the shared I2C bus (can be reused later for VL53L0X)."""
    return I2C(
        0,
        sda=Pin(config.I2C_SDA),
        scl=Pin(config.I2C_SCL),
        freq=config.I2C_FREQ,
    )


def run_calibration_demo(count: int = 200):
    """
    Minimal bring-up + gyro bias calibration.

    Robot must be sitting completely still on the floor (no vibration).
    Prints whoami, temperature, and the computed bias (should be very small, e.g. < 0.02 rad/s).
    After calibration you can gently rotate the chassis by hand and watch gz.
    """
    print("=" * 65)
    print(" MPU6500 IMU CALIBRATION + LIVE GYRO DEMO")
    print(" Keep the robot perfectly still during the bias capture phase!")
    print("=" * 65)

    i2c = _get_i2c()
    mpu = MPU6500(i2c)

    print(f"WHO_AM_I   : 0x{mpu.whoami:02x}  (expected 0x70 / 0x71 / 0x90)")
    print(f"Temperature: {mpu.temperature:.2f} °C")
    print()

    print(f"Starting bias calibration ({count} samples, ~{count*5/1000:.1f}s)...")
    t0 = time.ticks_ms()
    bias = mpu.calibrate(count=count, delay_ms=5)
    dt = time.ticks_diff(time.ticks_ms(), t0) / 1000.0

    print(f"Bias calibration complete in {dt:.2f}s")
    print(f"  gyro_offset = ({bias[0]:+.6f}, {bias[1]:+.6f}, {bias[2]:+.6f}) rad/s")
    print()
    print("Now rotate the robot gently by hand (in place). You should see gz change.")
    print("Press Ctrl-C to stop live printing.\n")

    print("     gz (rad/s)     gz (deg/s)     temp °C")
    print("-" * 45)

    try:
        while True:
            gz = mpu.gyro_z
            temp = mpu.temperature
            print(f"  {gz:+8.4f}     {math.degrees(gz):+8.2f}      {temp:6.2f}")
            time.sleep_ms(100)
    except KeyboardInterrupt:
        print("\nLive demo stopped.")


def run_with_odometry_fusion(duration_s: float = 10.0, speed: float = 25.0, period_ms: int = 20):
    """
    Full 20 ms loop demonstration with wheel odometry + gyro fusion.

    This is the exact pattern you will use in production code.
    Drive straight or in gentle curves and watch how the fused theta behaves
    compared to pure wheel odometry.
    """
    print("=" * 65)
    print(f" 20 ms LOOP + GYRO FUSION  |  {duration_s}s @ {speed}%")
    print(" Using MPU6500 yaw rate in the complementary filter")
    print("=" * 65)

    i2c = _get_i2c()
    mpu = MPU6500(i2c)

    # Quick bias calibration (robot should already be still)
    print("Quick gyro bias capture (keep still for ~1 s)...")
    bias = mpu.calibrate(count=150, delay_ms=5)
    print(f"  bias_z = {bias[2]:+.5f} rad/s\n")

    el = QuadEncoder(config.ENCODER_LEFT_A, config.ENCODER_LEFT_B,
                     invert=config.ENCODER_LEFT_INVERT, name="left")
    er = QuadEncoder(config.ENCODER_RIGHT_A, config.ENCODER_RIGHT_B,
                     invert=config.ENCODER_RIGHT_INVERT, name="right")

    odo = Odometry()          # will pick up m_per_count from /calibration.json automatically
    timer = RateTimer(period_ms)

    start = time.ticks_ms()
    end = start + int(duration_s * 1000)

    drive.set_side_speeds(speed, speed)

    print("time   x       y      θ(°)   gz(rad/s)   wheel-only-θ(°)")
    print("-" * 70)

    try:
        while time.ticks_diff(end, time.ticks_ms()) > 0:
            dt = timer.wait() / 1000.0

            # Encoder deltas → meters
            dl = el.get_delta() * odo.m_per_count
            dr = er.get_delta() * odo.m_per_count

            # Fresh gyro reading every tick
            gz = mpu.gyro_z

            # === THE IMPORTANT LINE ===
            # Pass gyro_z into the odometry update for complementary fusion
            odo.update(dl, dr, dt, gyro_z=gz)

            if timer.iteration % 10 == 0:   # ~every 200 ms
                x, y, th = odo.get_pose()
                # For comparison: what pure wheel odometry would have given
                wheel_only_dtheta = (dr - dl) / odo.track_width
                wheel_only_th = (odo.theta - gz * dt) + wheel_only_dtheta * dt   # rough reconstruction

                print(f"{time.ticks_ms()/1000:5.1f}  "
                      f"{x:6.3f}  {y:6.3f}  "
                      f"{math.degrees(th):6.1f}   "
                      f"{gz:+7.4f}     "
                      f"{math.degrees(odo.theta):6.1f}")   # fused theta is already in odo.theta

        drive.brake_all()
        x, y, th = odo.get_pose()
        print("\n=== FINAL FUSED POSE ===")
        print(f"x = {x:.4f} m")
        print(f"y = {y:.4f} m")
        print(f"θ = {th:.4f} rad  ({math.degrees(th):.1f}°)")

    except KeyboardInterrupt:
        drive.brake_all()
        print("\nInterrupted by user.")
    finally:
        el.deinit()
        er.deinit()
        drive.deinit_all()
        print("IMU + odometry fusion test finished.")


def run_tof_demo(duration_s: float = 15.0):
    """
    Quick VL53L0X bring-up test (continuous mode).
    Point the sensor at different surfaces / distances and watch the readings.
    """
    print("=" * 65)
    print(" VL53L0X Time-of-Flight Demo (continuous ranging)")
    print(" Same I2C bus as MPU6500")
    print("=" * 65)

    from machine import I2C, Pin
    import config
    from sensors import VL53L0X

    i2c = I2C(0, sda=Pin(config.I2C_SDA), scl=Pin(config.I2C_SCL),
              freq=config.I2C_FREQ)
    tof = VL53L0X(i2c)
    tof.start_continuous()

    print("Range (mm)  |  Note: -1 = no valid reading yet")
    print("-" * 40)

    start = time.ticks_ms()
    try:
        while time.ticks_diff(time.ticks_ms(), start) < int(duration_s * 1000):
            dist = tof.range_mm
            print(f"{dist:6d} mm")
            time.sleep_ms(150)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        tof.stop_continuous()


if __name__ == "__main__":
    # Default action when run directly: the safe calibration + hand-twist demo
    run_calibration_demo(count=180)
