"""
test_encoder.py

Quick test script to verify that the quadrature encoders are working.

Upload this file with mpremote / ampy / rshell, then run from the REPL:
    >>> import test_encoder

Spin the wheels by hand and watch the printed deltas and positions.
Ctrl-C in the REPL to stop cleanly.
"""

import time
from drivers.quad_encoder import QuadEncoder
import config

print("Starting encoder test...")
print("Left  encoder pins: A=GPIO{}  B=GPIO{}".format(
    config.ENCODER_LEFT_A, config.ENCODER_LEFT_B))
print("Right encoder pins: A=GPIO{}  B=GPIO{}".format(
    config.ENCODER_RIGHT_A, config.ENCODER_RIGHT_B))
print("Reminder: Left encoders (34/35) REQUIRE external 4.7-10k pull-ups to 3V3.\n"
      "Right encoders (18/19) use internal pull-ups - no external needed.\n")

enc_left = QuadEncoder(
    pin_a=config.ENCODER_LEFT_A,
    pin_b=config.ENCODER_LEFT_B,
    invert=config.ENCODER_LEFT_INVERT,
    name="left",
)

enc_right = QuadEncoder(
    pin_a=config.ENCODER_RIGHT_A,
    pin_b=config.ENCODER_RIGHT_B,
    invert=config.ENCODER_RIGHT_INVERT,
    name="right",
)

print("Encoders initialized. Spin the wheels by hand.\n")

try:
    while True:
        delta_l = enc_left.get_delta()
        delta_r = enc_right.get_delta()

        pos_l = enc_left.get_position()
        pos_r = enc_right.get_position()

        print("Left:  delta={:5d}  pos={:7d}   |   "
              "Right: delta={:5d}  pos={:7d}".format(
                  delta_l, pos_l, delta_r, pos_r))

        time.sleep(0.2)

except KeyboardInterrupt:
    print("\nTest stopped by user.")
finally:
    enc_left.deinit()
    enc_right.deinit()
    print("Encoder IRQs detached. Safe to re-import.")
