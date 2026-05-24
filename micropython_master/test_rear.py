"""
test_rear.py  -  VERY SIMPLE REAR MOTOR TEST (Diagnostic Mode)

Goal: Help you quickly see which physical wheel is moving and in which direction.

Current problem you reported:
- Rear Left and Rear Right are wired swapped.
- One side rotates in the wrong direction.

This script is intentionally kept extremely simple.

=== How to use from your computer ===

mpremote run test_rear.py code_left_forward
mpremote run test_rear.py code_left_reverse
mpremote run test_rear.py code_right_forward
mpremote run test_rear.py code_right_reverse

Each command runs for 6 seconds with very clear messages telling you what to look at.

From REPL you can also do:
    import test_rear
    test_rear.code_left_forward()
"""

import time
import sys
from drivers.l298n import create_rear_left, create_rear_right
import config

# "code_left"  = motor connected to the pins you defined as LEFT in config.py (GPIO 13+14)
# "code_right" = motor connected to the pins you defined as RIGHT in config.py (GPIO 22+23)
code_left  = create_rear_left()
code_right = create_rear_right()

DURATION = 6.0


def _stop():
    code_left.stop()
    code_right.stop()


def code_left_forward():
    print("\n" + "=" * 55)
    print("CODE LEFT  →  pins 13 + 14  (L298N left channel)")
    print("Watch the PHYSICAL wheel on the LEFT side of the robot.")
    print("Is it going FORWARD or BACKWARD?")
    print("=" * 55 + "\n")
    code_right.stop()
    code_left.set_direction(True)
    time.sleep(DURATION)
    _stop()


def code_left_reverse():
    print("\n" + "=" * 55)
    print("CODE LEFT  →  pins 13 + 14   (REVERSE)")
    print("Watch the PHYSICAL LEFT wheel. Forward or backward?")
    print("=" * 55 + "\n")
    code_right.stop()
    code_left.set_direction(False)
    time.sleep(DURATION)
    _stop()


def code_right_forward():
    print("\n" + "=" * 55)
    print("CODE RIGHT →  pins 22 + 23  (L298N right channel)")
    print("Watch the PHYSICAL wheel on the RIGHT side of the robot.")
    print("Is it going FORWARD or BACKWARD?")
    print("=" * 55 + "\n")
    code_left.stop()
    code_right.set_direction(True)
    time.sleep(DURATION)
    _stop()


def code_right_reverse():
    print("\n" + "=" * 55)
    print("CODE RIGHT →  pins 22 + 23   (REVERSE)")
    print("Watch the PHYSICAL RIGHT wheel. Forward or backward?")
    print("=" * 55 + "\n")
    code_left.stop()
    code_right.set_direction(False)
    time.sleep(DURATION)
    _stop()


def stop():
    _stop()
    print("Stopped.")


if __name__ == "__main__":
    if len(sys.argv) <= 1:
        print("Please give a command, for example:")
        print("  mpremote run test_rear.py code_left_forward")
        print("  mpremote run test_rear.py code_right_reverse")
        sys.exit(0)

    cmd = sys.argv[1].lower()

    if cmd in ("code_left_forward", "left_forward", "lf"):
        code_left_forward()
    elif cmd in ("code_left_reverse", "left_reverse", "lr"):
        code_left_reverse()
    elif cmd in ("code_right_forward", "right_forward", "rf"):
        code_right_forward()
    elif cmd in ("code_right_reverse", "right_reverse", "rr"):
        code_right_reverse()
    elif cmd in ("stop", "stop_all"):
        stop()
    else:
        print(f"Unknown command: {cmd}")
        print("Valid: code_left_forward, code_left_reverse, code_right_forward, code_right_reverse")