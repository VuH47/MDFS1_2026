"""
test_loop.py
Demonstrates a real 20 ms fixed-rate control loop using the new RateTimer.

It drives the robot at constant speed while printing:
- loop iteration, actual dt, jitter
- live encoder deltas (speed proxy)
- last drive commands

Run from REPL after the Phase 1 invert fixes:
    >>> import test_loop
    >>> test_loop.run(duration_s=10, speed=35)

This is the skeleton for the future main control loop (odometry, path following, etc.).
"""

import time
from utils.timing import RateTimer
from drivers.quad_encoder import QuadEncoder
from robot import drive
import config


def run(duration_s: float = 10.0, speed: float = 40.0, period_ms: int = 20):
    print("=" * 60)
    print(f" 20 ms CONTROL LOOP DEMO  |  speed={speed}%  |  {duration_s}s")
    print(" Uses RateTimer + drive + encoders (inverts from config)")
    print("=" * 60)

    # Encoders (respect config inverts)
    el = QuadEncoder(config.ENCODER_LEFT_A, config.ENCODER_LEFT_B,
                     invert=config.ENCODER_LEFT_INVERT, name="left")
    er = QuadEncoder(config.ENCODER_RIGHT_A, config.ENCODER_RIGHT_B,
                     invert=config.ENCODER_RIGHT_INVERT, name="right")

    timer = RateTimer(period_ms=period_ms)
    start = time.ticks_ms()
    max_jitter = 0
    sum_jitter = 0
    count = 0

    try:
        drive.set_side_speeds(speed, speed)

        while time.ticks_diff(time.ticks_ms(), start) < int(duration_s * 1000):
            dt = timer.wait()
            dl = el.get_delta()
            dr = er.get_delta()

            sum_jitter += timer.jitter
            max_jitter = max(max_jitter, timer.jitter)
            count += 1

            if count % 10 == 0:   # print ~every 200 ms
                cmd_l, cmd_r = drive.get_last_commands()
                rear_l, rear_r = drive.get_rear_directions()
                print(f"[{timer.iteration:4d}] dt={dt:3d} j={timer.jitter:2d}  "
                      f"Ld={dl:4d} Rd={dr:4d}  cmds=({cmd_l:5.1f},{cmd_r:5.1f})  rear=({rear_l},{rear_r})")

        drive.brake_all()
        print("\nLoop finished. Statistics:")
        print(f"  Iterations : {count}")
        print(f"  Avg jitter : {sum_jitter / max(1, count):.2f} ms")
        print(f"  Max jitter : {max_jitter} ms")
        print(f"  Target     : {period_ms} ms")

    except KeyboardInterrupt:
        print("\nInterrupted — braking...")
        drive.brake_all()
    finally:
        el.deinit()
        er.deinit()
        drive.deinit_all()
        print("Clean shutdown complete.")


if __name__ == "__main__":
    run()