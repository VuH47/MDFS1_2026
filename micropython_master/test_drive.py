"""
test_drive.py
Interactive open-loop drive test for the full 4-motor robot (Phase 1).

After uploading, run from REPL:
    >>> import test_drive
    >>> test_drive.run()

Or call individual helpers:
    >>> test_drive.forward(50, 1.5)
    >>> test_drive.turn_in_place(40, 2.0)
    etc.

Always spins up the encoders too so you can watch deltas live while the
robot is moving (or wheels lifted for safety). Confirms "same command per side"
and encoder sign agreement.

Safety: all movements are time-bounded; KeyboardInterrupt or any exception
triggers full brake + deinit.
"""

import time
from drivers.quad_encoder import QuadEncoder
from robot import drive
import config


def _init_encoders():
    """Return (left_enc, right_enc) – caller must deinit."""
    el = QuadEncoder(config.ENCODER_LEFT_A, config.ENCODER_LEFT_B,
                     invert=config.ENCODER_LEFT_INVERT, name="left")
    er = QuadEncoder(config.ENCODER_RIGHT_A, config.ENCODER_RIGHT_B,
                     invert=config.ENCODER_RIGHT_INVERT, name="right")
    return el, er


def _print_status(el, er, label=""):
    dl = el.get_delta()
    dr = er.get_delta()
    pl = el.get_position()
    pr = er.get_position()
    print(f"{label:12s} L: d={dl:5d} p={pl:7d}  |  R: d={dr:5d} p={pr:7d}")


def forward(speed=50, seconds=2.0, print_every=0.2):
    """Drive straight forward (both sides same sign)."""
    el, er = _init_encoders()
    try:
        drive.set_side_speeds(speed, speed)
        print(f"FORWARD {speed}% for {seconds}s ...")
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < int(seconds * 1000):
            _print_status(el, er, "forward")
            time.sleep(print_every)
    finally:
        drive.brake_all()
        el.deinit()
        er.deinit()
        print("forward done + braked")


def reverse(speed=50, seconds=2.0, print_every=0.2):
    """Drive straight reverse."""
    el, er = _init_encoders()
    try:
        drive.set_side_speeds(-speed, -speed)
        print(f"REVERSE {speed}% for {seconds}s ...")
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < int(seconds * 1000):
            _print_status(el, er, "reverse")
            time.sleep(print_every)
    finally:
        drive.brake_all()
        el.deinit()
        er.deinit()
        print("reverse done + braked")


def turn_in_place(speed=40, seconds=2.0, print_every=0.2):
    """Pivot turn: left side backward, right side forward (or vice-versa)."""
    el, er = _init_encoders()
    try:
        drive.set_side_speeds(-speed, speed)   # left back, right fwd → CCW if viewed from above
        print(f"TURN_IN_PLACE (L-,R+) {speed}% for {seconds}s ...")
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < int(seconds * 1000):
            _print_status(el, er, "turn")
            time.sleep(print_every)
    finally:
        drive.brake_all()
        el.deinit()
        er.deinit()
        print("turn done + braked")


def ramp_test(max_speed=70, step=10, hold=0.8):
    """Ramp speed up and down on both sides together (sanity + current check)."""
    el, er = _init_encoders()
    try:
        print("RAMP TEST (both sides together)")
        for s in list(range(0, max_speed + 1, step)) + list(range(max_speed - step, -1, -step)):
            drive.set_side_speeds(s, s)
            _print_status(el, er, f"ramp {s:3d}%")
            time.sleep(hold)
        drive.brake_all()
        print("ramp complete")
    finally:
        drive.brake_all()
        el.deinit()
        er.deinit()


def stop():
    drive.brake_all()
    print("All motors braked.")


def run():
    """
    Full interactive demo – call this from REPL for a guided bring-up sequence.
    Hit Ctrl-C at any time to abort safely.
    """
    print("=" * 60)
    print(" 4WD ROBOT OPEN-LOOP DRIVE TEST (Phase 1)")
    print(" Pins from config – same command per side")
    print(" Lift wheels or use a safe test area!")
    print("=" * 60)

    try:
        print("\n[1/5] Forward 50%  2s")
        forward(50, 2.0)

        print("\n[2/5] Reverse 40%  1.5s")
        reverse(40, 1.5)

        print("\n[3/5] Turn in place 40%  2s")
        turn_in_place(40, 2.0)

        print("\n[4/5] Ramp 0→70→0")
        ramp_test(70, 10, 0.6)

        print("\n[5/5] Final brake")
        stop()

        print("\nAll tests finished. Check encoder deltas matched commanded directions.")
        print("If a side runs opposite to encoder, set invert=True in that QuadEncoder.")
        print("If rear does not follow front direction, verify L298N wiring.")

    except KeyboardInterrupt:
        print("\nInterrupted by user – emergency brake!")
        drive.brake_all()
    except Exception as e:
        print("ERROR:", e)
        drive.brake_all()
        raise
    finally:
        drive.deinit_all()
        print("Motors de-initialized. Safe to re-import.")


if __name__ == "__main__":
    run()