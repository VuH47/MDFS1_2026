"""
sensors/mpu6500.py

MicroPython I2C driver for the MPU6500 6-axis IMU (gyro + accel + temp).

Adapted from the excellent tuupola/micropython-mpu9250 library
(https://github.com/tuupola/micropython-mpu9250) with project-specific
tweaks for style, imports, and odometry fusion convenience.

Primary use in this robot:
    - Yaw rate (gyro_z) for complementary fusion in odometry/odometry.py
    - Optional stationary gyro bias calibration (strongly recommended)

Hardware:
    - I2C on pins defined in config (SDA=17, SCL=5 @ 400 kHz)
    - Default address 0x68 (AD0 low). Pull AD0 high for 0x69 if needed.

Typical bring-up (from REPL or test_imu.py):
    from machine import I2C, Pin
    import config
    from sensors.mpu6500 import MPU6500

    i2c = I2C(0, sda=Pin(config.I2C_SDA), scl=Pin(config.I2C_SCL),
              freq=config.I2C_FREQ)
    mpu = MPU6500(i2c)                    # auto-detects + resets device

    # One-time bias calibration (robot must be perfectly still, ~1-2 seconds)
    bias = mpu.calibrate(count=200, delay=5)   # returns (ox, oy, oz)
    print("Gyro bias (rad/s):", bias)

    # In your 20 ms control loop:
    gz = mpu.gyro_z          # or mpu.gyro[2]
    odo.update(dl, dr, dt, gyro_z=gz)

See:
    - test_imu.py for full example + live printing
    - odometry/odometry.py for the fusion math (matches MATLAB updateOdometry.m)
"""

__version__ = "0.4.0-project"

# pylint: disable=import-error
import struct
import time
from machine import I2C
from micropython import const
# pylint: enable=import-error

# Register map (MPU6500 / MPU9250 compatible)
_GYRO_CONFIG   = const(0x1b)
_ACCEL_CONFIG  = const(0x1c)
_ACCEL_CONFIG2 = const(0x1d)
_ACCEL_XOUT_H  = const(0x3b)
_ACCEL_XOUT_L  = const(0x3c)
_ACCEL_YOUT_H  = const(0x3d)
_ACCEL_YOUT_L  = const(0x3e)
_ACCEL_ZOUT_H  = const(0x3f)
_ACCEL_ZOUT_L  = const(0x40)
_TEMP_OUT_H    = const(0x41)
_TEMP_OUT_L    = const(0x42)
_GYRO_XOUT_H   = const(0x43)
_GYRO_XOUT_L   = const(0x44)
_GYRO_YOUT_H   = const(0x45)
_GYRO_YOUT_L   = const(0x46)
_GYRO_ZOUT_H   = const(0x47)
_GYRO_ZOUT_L   = const(0x48)
_WHO_AM_I      = const(0x75)
_PWR_MGMT_1    = const(0x6b)

# Accelerometer full-scale options
ACCEL_FS_SEL_2G  = const(0b00000000)
ACCEL_FS_SEL_4G  = const(0b00001000)
ACCEL_FS_SEL_8G  = const(0b00010000)
ACCEL_FS_SEL_16G = const(0b00011000)

_ACCEL_SO_2G  = 16384.0
_ACCEL_SO_4G  = 8192.0
_ACCEL_SO_8G  = 4096.0
_ACCEL_SO_16G = 2048.0

# Gyro full-scale options (250 DPS recommended for robot yaw)
GYRO_FS_SEL_250DPS  = const(0b00000000)
GYRO_FS_SEL_500DPS  = const(0b00001000)
GYRO_FS_SEL_1000DPS = const(0b00010000)
GYRO_FS_SEL_2000DPS = const(0b00011000)

_GYRO_SO_250DPS  = 131.0
_GYRO_SO_500DPS  = 62.5
_GYRO_SO_1000DPS = 32.8
_GYRO_SO_2000DPS = 16.4

_TEMP_SO     = 333.87
_TEMP_OFFSET = 21.0

# Scale factors for property returns
SF_G     = 1.0
SF_M_S2  = 9.80665          # 1 g in m/s²
SF_DEG_S = 1.0
SF_RAD_S = 0.017453292519943  # deg/s → rad/s


class MPU6500:
    """
    Interface to MPU6500 6-axis motion tracking device (gyro + accel + temp).

    The driver performs a device reset on construction and leaves the sensor
    in a ready state with the chosen full-scale ranges.

    Args:
        i2c: machine.I2C instance (400 kHz recommended)
        address: 0x68 (default) or 0x69
        accel_fs: one of ACCEL_FS_SEL_* (default 2 g)
        gyro_fs: one of GYRO_FS_SEL_* (default 250 DPS — best for robot heading)
        accel_sf: SF_M_S2 (m/s²) or SF_G
        gyro_sf: SF_RAD_S (rad/s, recommended) or SF_DEG_S
        gyro_offset: (ox, oy, oz) initial bias in gyro_sf units (rad/s or deg/s)
    """

    def __init__(
        self,
        i2c,
        address=0x68,
        accel_fs=ACCEL_FS_SEL_2G,
        gyro_fs=GYRO_FS_SEL_250DPS,
        accel_sf=SF_M_S2,
        gyro_sf=SF_RAD_S,
        gyro_offset=(0.0, 0.0, 0.0),
    ):
        self.i2c = i2c
        self.address = address

        # 0x70 = standalone MPU6500, 0x71 = MPU6250 SIP, 0x90 = MPU6700
        if self.whoami not in (0x70, 0x71, 0x90):
            raise RuntimeError(
                "MPU6500 not found at I2C address 0x{:02x} (whoami=0x{:02x})".format(
                    address, self.whoami
                )
            )

        # Reset device, wait for internal registers to settle, then wake
        self._register_char(_PWR_MGMT_1, 0x80)
        time.sleep_ms(100)
        self._register_char(_PWR_MGMT_1, 0x00)
        time.sleep_ms(100)

        self._accel_so = self._accel_fs(accel_fs)
        self._gyro_so = self._gyro_fs(gyro_fs)
        self._accel_sf = accel_sf
        self._gyro_sf = gyro_sf
        self._gyro_offset = gyro_offset

    # ------------------------------------------------------------------
    # Public sensor properties (odometry mainly uses gyro_z)
    # ------------------------------------------------------------------

    @property
    def acceleration(self):
        """
        3-tuple (ax, ay, az) in m/s² (or g if accel_sf=SF_G).
        """
        so = self._accel_so
        sf = self._accel_sf
        xyz = self._register_three_shorts(_ACCEL_XOUT_H)
        return tuple(v / so * sf for v in xyz)

    @property
    def gyro(self):
        """
        3-tuple (gx, gy, gz) in rad/s (or deg/s) with bias correction applied.
        """
        so = self._gyro_so
        sf = self._gyro_sf
        ox, oy, oz = self._gyro_offset

        xyz = self._register_three_shorts(_GYRO_XOUT_H)
        xyz = [v / so * sf for v in xyz]

        xyz[0] -= ox
        xyz[1] -= oy
        xyz[2] -= oz
        return tuple(xyz)

    @property
    def gyro_z(self):
        """
        Yaw rate (Z axis) in rad/s (bias corrected).

        This is the primary value used for odometry gyro fusion.
        """
        return self.gyro[2]

    @property
    def temperature(self):
        """Die temperature in °C."""
        temp = self._register_short(_TEMP_OUT_H)
        return ((temp - _TEMP_OFFSET) / _TEMP_SO) + _TEMP_OFFSET

    @property
    def whoami(self):
        """Contents of the WHO_AM_I register (for debug / detection)."""
        return self._register_char(_WHO_AM_I)

    # ------------------------------------------------------------------
    # Bias calibration (call with robot completely stationary)
    # ------------------------------------------------------------------

    def calibrate(self, count=256, delay_ms=5):
        """
        Compute and store gyro bias by averaging 'count' samples.

        Robot **must be perfectly still** during this process (1–3 seconds typical).

        Returns:
            (bias_x, bias_y, bias_z) tuple in the current gyro_sf units (rad/s recommended)
        """
        ox = oy = oz = 0.0
        self._gyro_offset = (0.0, 0.0, 0.0)
        n = float(count)

        for _ in range(count):
            time.sleep_ms(delay_ms)
            gx, gy, gz = self.gyro
            ox += gx
            oy += gy
            oz += gz

        self._gyro_offset = (ox / n, oy / n, oz / n)
        return self._gyro_offset

    # ------------------------------------------------------------------
    # Low-level register helpers (private)
    # ------------------------------------------------------------------

    def _register_short(self, register, value=None, buf=bytearray(2)):
        if value is None:
            self.i2c.readfrom_mem_into(self.address, register, buf)
            return struct.unpack(">h", buf)[0]

        struct.pack_into(">h", buf, 0, value)
        return self.i2c.writeto_mem(self.address, register, buf)

    def _register_three_shorts(self, register, buf=bytearray(6)):
        self.i2c.readfrom_mem_into(self.address, register, buf)
        return struct.unpack(">hhh", buf)

    def _register_char(self, register, value=None, buf=bytearray(1)):
        if value is None:
            self.i2c.readfrom_mem_into(self.address, register, buf)
            return buf[0]

        struct.pack_into("<b", buf, 0, value)
        return self.i2c.writeto_mem(self.address, register, buf)

    def _accel_fs(self, value):
        self._register_char(_ACCEL_CONFIG, value)
        if value == ACCEL_FS_SEL_2G:
            return _ACCEL_SO_2G
        elif value == ACCEL_FS_SEL_4G:
            return _ACCEL_SO_4G
        elif value == ACCEL_FS_SEL_8G:
            return _ACCEL_SO_8G
        elif value == ACCEL_FS_SEL_16G:
            return _ACCEL_SO_16G
        return _ACCEL_SO_2G

    def _gyro_fs(self, value):
        self._register_char(_GYRO_CONFIG, value)
        if value == GYRO_FS_SEL_250DPS:
            return _GYRO_SO_250DPS
        elif value == GYRO_FS_SEL_500DPS:
            return _GYRO_SO_500DPS
        elif value == GYRO_FS_SEL_1000DPS:
            return _GYRO_SO_1000DPS
        elif value == GYRO_FS_SEL_2000DPS:
            return _GYRO_SO_2000DPS
        return _GYRO_SO_250DPS

    # Context manager support
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        pass
