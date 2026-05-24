"""
odometry/odometry.py
2D differential drive odometry with optional gyro fusion.

Direct port of the validated logic from algorithm/matlab/updateOdometry.m
and the project algorithm docs (complementary filter α≈0.98).

Usage in 20 ms loop (with optional MPU6500 gyro fusion):

    from odometry.odometry import Odometry
    import config
    from machine import I2C, Pin
    from drivers.quad_encoder import QuadEncoder
    from utils.timing import RateTimer
    from sensors import MPU6500

    i2c = I2C(0, sda=Pin(config.I2C_SDA), scl=Pin(config.I2C_SCL), freq=config.I2C_FREQ)
    mpu = MPU6500(i2c)
    mpu.calibrate(count=150)          # robot must be still; stores bias internally

    odo = Odometry()
    el = QuadEncoder(...)   # with correct invert from config
    er = QuadEncoder(...)

    timer = RateTimer(20)
    while True:
        dt = timer.wait() / 1000.0
        dl = el.get_delta() * odo.m_per_count
        dr = er.get_delta() * odo.m_per_count

        gz = mpu.gyro_z                 # fresh yaw rate every tick (rad/s)
        odo.update(dl, dr, dt, gyro_z=gz)

        x, y, th = odo.get_pose()
"""

import json
import math
import config


class Odometry:
    """
    Simple wheel odometry + complementary fusion hook.

    All distances in meters, angles in radians.
    """

    def __init__(self):
        # Mechanical parameters from the single source of truth (config)
        self.wheel_radius = config.WHEEL_DIAMETER_MM / 2000.0      # meters
        self.track_width = config.TRACK_WIDTH_MM / 1000.0          # meters

        # Default (fallback) scale - overridden if /calibration.json exists.
        # Empirical calibration (1600 mm drive @ ~10% in debug log) gave 1.4939e-3 m/count.
        # The json is written by the wifi_agent calibration UI and loaded below.
        self.m_per_count = (2 * math.pi * self.wheel_radius) / 1560.0   # wheel-geom fallback only

        # Apply persisted calibration (from tools/wifi_agent.py "Compute & save")
        # so every script (tests, main, etc.) automatically uses the measured value.
        try:
            with open("/calibration.json") as _f:
                _saved = json.load(_f)
            if "m_per_count" in _saved:
                self.m_per_count = _saved["m_per_count"]
                print("[Odometry] Loaded m_per_count={:.6f} from /calibration.json".format(self.m_per_count))
        except Exception:
            pass  # no file or first boot - use default

        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Fusion weight (from MATLAB prototype)
        self.gyro_weight = 0.98
        self.wheel_weight = 1.0 - self.gyro_weight

        # For velocity estimation
        self.last_v = 0.0
        self.last_w = 0.0

    def update(self, d_left: float, d_right: float, dt: float, gyro_z: float = 0.0):
        """
        Integrate one timestep.

        :param d_left:   distance traveled by left wheel this dt (meters, signed)
        :param d_right:  distance traveled by right wheel this dt (meters, signed)
        :param dt:       actual time step (seconds)
        :param gyro_z:   yaw rate from IMU (rad/s), positive = CCW. 0 = use wheel only.
        """
        d_center = (d_left + d_right) / 2.0
        d_theta_wheel = (d_right - d_left) / self.track_width

        # Complementary fusion (same as MATLAB)
        d_theta = (self.gyro_weight * gyro_z * dt) + (self.wheel_weight * d_theta_wheel)

        # Update pose (simple Euler integration)
        self.theta += d_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        # Store velocities for higher controllers
        self.last_v = d_center / dt if dt > 0 else 0.0
        self.last_w = d_theta / dt if dt > 0 else 0.0

    def update_from_encoders(self, delta_left_counts: int, delta_right_counts: int, dt: float, gyro_z: float = 0.0):
        """
        Convenience method — pass raw deltas from QuadEncoder.get_delta().
        Converts counts to meters using self.m_per_count.
        """
        dl = delta_left_counts * self.m_per_count
        dr = delta_right_counts * self.m_per_count
        self.update(dl, dr, dt, gyro_z)

    def get_pose(self):
        """Returns (x, y, theta) in meters and radians."""
        return self.x, self.y, self.theta

    def get_velocities(self):
        """Returns (v, omega) in m/s and rad/s from last update."""
        return self.last_v, self.last_w

    def reset(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.last_v = 0.0
        self.last_w = 0.0

    def set_scale(self, meters_per_count: float):
        """Update after 1 m calibration drive."""
        self.m_per_count = meters_per_count

    def __repr__(self):
        return f"Odometry(x={self.x:.3f}, y={self.y:.3f}, θ={math.degrees(self.theta):.1f}°)"