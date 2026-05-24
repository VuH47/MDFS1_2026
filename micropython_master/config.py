"""
config.py
Central configuration for the MicroPython version of the robot (micropython_master).

Single source of truth for all hardware pins and tuning constants.
Update the values here when you change wiring.

Board: ESP32-WROOM (both master and slave)
Communication: ESP-NOW

------------------------------------------------------------------------------
GPIO RULES for THIS dev board
(verified against ESP-WROOM-32 datasheet v2.4 and the actual headers exposed
on the current board: GPIO 36 and 39 are NOT broken out.)

  Available header GPIOs : 0, 2, 4, 5, 12-19, 21, 22, 23, 25-27, 32-35
  NOT exposed on board   : 36 (SENSOR_VP), 39 (SENSOR_VN)
  Never use (flash)      : 6, 7, 8, 9, 10, 11
  Never use (REPL UART)  : 1 (TX0), 3 (RX0)
  Strapping (avoid)      : 0, 2, 12, 15
                            - 0  : boot mode select (pull-up default)
                            - 2  : must NOT be pulled high during flash
                            - 12 : VDD_SDIO voltage select - driving HIGH at
                                   boot can BRICK the boot. Avoid as output.
                            - 15 : boot-log silence (pull-up default)
  Strapping (OK for I2C) : 5  (SDIO timing strap, safe as I2C SCL post-boot)
  Input-only, no pulls   : 34, 35   (external 4.7k-10k pull-ups REQUIRED)
------------------------------------------------------------------------------

Allocation summary:
  Used  (16): 4, 5, 13, 14, 17, 18, 19, 22, 23, 25, 26, 27, 32, 33, 34, 35
  Spare ( 2): 16, 21   (free for status LED, bumper switch, e-stop, etc.)
  Reserved : 0, 1, 2, 3, 6-11, 12, 15, 36, 39
"""

# =============================================================================
# MECHANICAL CONSTANTS
# =============================================================================
WHEEL_DIAMETER_MM = 65.0
TRACK_WIDTH_MM = 185.0          # TODO: measure accurately

# Temporary scaling (mm/s -> PWM duty). Tune during testing.
MM_PER_S_TO_PWM = 1.8

# Control loop timing (in milliseconds)
CONTROL_LOOP_PERIOD_MS = 20


# =============================================================================
# FRONT MOTORS - TB6612 4-Channel Board (Encoder Motors)
# =============================================================================
# Each front motor uses 5 ESP32 pins: PWM, IN1, IN2, EncA, EncB.

# --- Front Left (TB6612 channel 1) ---
MOTOR_LEFT_PWM = 25
MOTOR_LEFT_IN1 = 26
MOTOR_LEFT_IN2 = 27

ENCODER_LEFT_A = 34   # INPUT-ONLY - external pull-up REQUIRED
ENCODER_LEFT_B = 35   # INPUT-ONLY - external pull-up REQUIRED

# --- Front Right (TB6612 channel 4) ---
MOTOR_RIGHT_PWM = 33
MOTOR_RIGHT_IN1 = 32
MOTOR_RIGHT_IN2 = 4

ENCODER_RIGHT_A = 18  # Internal pull-up available
ENCODER_RIGHT_B = 19  # Internal pull-up available

# Per-side direction invert (software fix for wiring polarity)
# Set to True if commanding positive speed makes the wheel/encoder go the wrong way
# relative to the left side convention (positive = "forward" for odometry).
# Determined from Phase 1 hardware test (see debug log.txt, reverse section lines ~54-66):
#   Left encoder correctly went negative on negative command.
#   Right encoder went positive (or stalled) on negative command → right motor polarity inverted.
MOTOR_LEFT_INVERT = True
MOTOR_RIGHT_INVERT = False

#current MOTOR_LEFT_INVERT = True
#MOTOR_RIGHT_INVERT = False

# Encoder inversion (rarely needed if motor polarity is corrected above; positive counts should mean
# "forward" travel for odometry). Provided for symmetry and completeness.
ENCODER_LEFT_INVERT = False
ENCODER_RIGHT_INVERT = False


# =============================================================================
# REAR MOTORS - L298N Dual H-Bridge (Plain DC Motors, DIRECTION-ONLY)
# =============================================================================
# Design decision: rear motors are run at fixed speed, NOT PWM-controlled.
# Tie L298N ENA and ENB HIGH in hardware (jumper to +5V/+3V3).
# The ESP32 only drives IN1/IN2 to set direction:
#   IN1=H, IN2=L  -> forward
#   IN1=L, IN2=H  -> reverse
#   IN1=L, IN2=L  -> stop (coast)

MOTOR_REAR_LEFT_IN1 = 13
MOTOR_REAR_LEFT_IN2 = 14

MOTOR_REAR_RIGHT_IN1 = 22
MOTOR_REAR_RIGHT_IN2 = 23


# =============================================================================
# I2C SENSORS (MPU6500 IMU + VL53L0X ToF)
# =============================================================================
# SCL on GPIO 5 is the SDIO timing strapping pin, but it is safe to drive as
# I2C SCL after boot. I2C buses always require external pull-ups (4.7k-10k)
# so the strapping default is consistent with normal I2C wiring.
I2C_SDA = 17
I2C_SCL = 5
I2C_FREQ = 400000

# Device addresses
MPU6500_ADDR = 0x68
VL53L0X_ADDR = 0x29

# Usage example (see sensors/mpu6500.py, vl53l0x.py and test_imu.py):
#   from machine import I2C, Pin
#   from sensors import MPU6500, VL53L0X
#   i2c = I2C(0, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=I2C_FREQ)
#   mpu = MPU6500(i2c); mpu.calibrate()
#   tof = VL53L0X(i2c); tof.start_continuous()
#   gz = mpu.gyro_z
#   dist = tof.range_mm


# =============================================================================
# ESP-NOW COMMUNICATION
# =============================================================================
ESP_NOW_CHANNEL = 1

# MAC address of the peer board (the other ESP32-WROOM).
# Example: b'\x24\x6f\x28\x12\x34\x56'
PEER_MAC = None   # TODO: fill in after reading the peer's MAC


# =============================================================================
# MISC / DEBUG
# =============================================================================
# Set to True during development for more verbose prints.
DEBUG = True

# True  -> WiFi/BLE left enabled (teaching/GUI mode).
# False -> radios disabled in playback for stability.
ENABLE_RADIO_IN_PLAYBACK = False


# =============================================================================
# WIFI AGENT (tools/wifi_agent.py)
# =============================================================================
# Shared secret required by the wireless calibration GUI. Anyone on the same
# WiFi network would otherwise be able to drive the robot. Change this string
# to something only you know; the browser UI will ask for it once.
WIFI_AGENT_TOKEN = "robot1234"
WIFI_AGENT_PORT = 80
