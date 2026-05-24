"""
slave_config.py
Configuration for the ESP32-WROOM **slave** board.

Responsibilities on this board:
- Drive 5 hobby servos (gripper, camera pan/tilt, auxiliary mechanisms, etc.)
- Read 4 push-button / bumper inputs (2 front, 2 rear) for safety and user commands
- Respond to ESP-NOW commands from the master board

Pin map chosen to avoid strapping pins, input-only pins for outputs, and leave
headroom for future expansion (I2C, extra UART, etc.).

All pins are valid on the ESP32-WROOM-32 module used in the project.
"""

from micropython import const

# =============================================================================
# BOARD IDENTITY
# =============================================================================
BOARD_ROLE = "SLAVE"
BOARD_NAME = "robot-slave-periph"

# =============================================================================
# ESP-NOW PEER (MASTER MAC ADDRESS)
# =============================================================================
# Fill this with the actual MAC of the master board (printed at boot on master).
# Example: b'\x24\x6f\x28\x12\x34\x56'
MASTER_MAC = None   # <-- MUST BE SET AFTER FIRST BOOT OF MASTER

# ESP-NOW channel (must match master)
ESP_NOW_CHANNEL = 1

# =============================================================================
# SERVO OUTPUTS (5 channels)
# =============================================================================
# All pins support PWM/LEDC. Order is arbitrary — map to your mechanical needs.
SERVO_PINS = [25, 26, 27, 32, 33]          # GPIO for servos 0..4

# Default servo calibration (µs). Adjust per servo if needed.
SERVO_MIN_US = 500
SERVO_MAX_US = 2500
SERVO_FREQ   = 50

# Logical names (for logging / commands)
SERVO_NAMES = ["gripper", "gripper_rotate", "camera_pan", "camera_tilt", "aux"]

# =============================================================================
# PUSH BUTTONS / BUMPERS (4 inputs, active-low with pull-ups)
# =============================================================================
# 2 front + 2 rear. Use internal pull-ups. Wire buttons between pin and GND.
BUTTON_PINS = {
    "front_left":  13,
    "front_right": 14,
    "rear_left":   18,
    "rear_right":  19,
}

# Debounce time in ms
BUTTON_DEBOUNCE_MS = 30

# =============================================================================
# STATUS / DEBUG
# =============================================================================
DEBUG = True
STATUS_LED_PIN = 2          # Built-in blue LED on many WROOM boards (optional)

# =============================================================================
# I2C (optional future use on slave, e.g. extra sensors)
# =============================================================================
I2C_SDA = 17
I2C_SCL = 5
I2C_FREQ = 400000

# =============================================================================
# PACKET TYPES (must match master side)
# =============================================================================
PKT_SERVO_CMD   = const(0x10)   # Master → Slave : set 5 servo angles
PKT_BUTTON_STAT = const(0x20)   # Slave → Master : button bitmask + timestamp
PKT_HEARTBEAT   = const(0x30)   # Either direction

# Button bitmask (sent in PKT_BUTTON_STAT)
BTN_FRONT_LEFT  = const(1 << 0)
BTN_FRONT_RIGHT = const(1 << 1)
BTN_REAR_LEFT   = const(1 << 2)
BTN_REAR_RIGHT  = const(1 << 3)