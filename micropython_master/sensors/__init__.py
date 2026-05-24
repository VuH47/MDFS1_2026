"""
sensors package

Currently provides:
    - MPU6500 : 6-axis IMU (gyro + accel + temp) with bias calibration

Planned:
    - VL53L0X   : Time-of-Flight distance sensor (for obstacle / safety)

Usage:
    from machine import I2C, Pin
    import config
    from sensors import MPU6500

    i2c = I2C(0, sda=Pin(config.I2C_SDA), scl=Pin(config.I2C_SCL), freq=config.I2C_FREQ)
    mpu = MPU6500(i2c)
"""

from .mpu6500 import MPU6500
from .vl53l0x import VL53L0X

__all__ = ["MPU6500", "VL53L0X"]
