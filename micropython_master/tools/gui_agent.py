"""
tools/gui_agent.py
==================
Lightweight serial-line agent that runs ON the ESP32 and exposes the robot
to the PC-side GUI (tools/gui/robot_gui.py).

START FROM REPL:
    >>> import tools.gui_agent

It blocks the REPL while running. Ctrl-C in the GUI (or on the serial line)
stops it, releases the encoders, and stops the motors.

PROTOCOL (plain text, one message per line, LF terminated)
----------------------------------------------------------
PC -> ESP32
    CMD <left%> <right%>     set side speeds, e.g. "CMD -50 50"
    STOP                     coast_all()
    BRAKE                    brake_all()
    PING                     responds with "PONG"

ESP32 -> PC
    READY                    sent once at startup
    ENC <dL> <dR> <posL> <posR>     periodic, default 50 ms
    ACK <left%> <right%>     after a CMD
    PONG                     reply to PING
    LOG <free text>          warnings / errors
    BYE                      sent on clean shutdown

Why a separate agent (and not the raw REPL)?
- Predictable line-oriented protocol the PC can parse safely.
- No collision with main.py's banner prints.
- ~20 Hz encoder updates without round-trip overhead per sample.
"""

import sys
import time

import config
from drivers.quad_encoder import QuadEncoder
from robot import drive


# How often to push an ENC line to the PC, in milliseconds.
ENC_PERIOD_MS = 50

# Safety: if no command is received for this many ms, stop the motors.
COMMAND_TIMEOUT_MS = 1000


def _emit(line):
    """Single point of output. Always one line, always flushed."""
    print(line)


def _parse_cmd(parts):
    """
    Parse a CMD line tokens (already split by whitespace, including 'CMD').
    Returns (left, right) clamped to [-100, 100], or raises ValueError.
    """
    if len(parts) != 3:
        raise ValueError("CMD expects 2 args")
    left = float(parts[1])
    right = float(parts[2])
    if left < -100:
        left = -100.0
    elif left > 100:
        left = 100.0
    if right < -100:
        right = -100.0
    elif right > 100:
        right = 100.0
    return left, right


def run():
    """Main loop. Returns when stdin closes or Ctrl-C is received."""
    enc_left = QuadEncoder(
        pin_a=config.ENCODER_LEFT_A,
        pin_b=config.ENCODER_LEFT_B,
        invert=False,
        name="left",
    )
    enc_right = QuadEncoder(
        pin_a=config.ENCODER_RIGHT_A,
        pin_b=config.ENCODER_RIGHT_B,
        invert=False,
        name="right",
    )

    _emit("READY")
    _emit("LOG gui_agent started, ENC_PERIOD_MS=%d" % ENC_PERIOD_MS)

    last_emit_ms = time.ticks_ms()
    last_cmd_ms = time.ticks_ms()
    last_left = 0.0
    last_right = 0.0

    try:
        while True:
            # Non-blocking-ish stdin read: poll a single line if available.
            # MicroPython on ESP32 does not give us select() on sys.stdin,
            # so we use a short blocking readline() and rely on the GUI
            # always sending lines promptly. To stay responsive we cap
            # readline by checking time around it.
            line = sys.stdin.readline()
            now = time.ticks_ms()

            if line:
                line = line.strip()
                if line:
                    parts = line.split()
                    head = parts[0].upper()

                    if head == "CMD":
                        try:
                            left, right = _parse_cmd(parts)
                        except Exception as e:
                            _emit("LOG bad CMD: %s" % e)
                        else:
                            drive.set_side_speeds(left, right)
                            last_left, last_right = left, right
                            last_cmd_ms = now
                            _emit("ACK %.1f %.1f" % (left, right))

                    elif head == "STOP":
                        drive.coast_all()
                        last_left = last_right = 0.0
                        last_cmd_ms = now
                        _emit("ACK 0.0 0.0")

                    elif head == "BRAKE":
                        drive.brake_all()
                        last_left = last_right = 0.0
                        last_cmd_ms = now
                        _emit("ACK 0.0 0.0")

                    elif head == "PING":
                        _emit("PONG")

                    else:
                        _emit("LOG unknown cmd: %s" % head)

            # Watchdog: stop motors if PC went silent.
            if (last_left != 0.0 or last_right != 0.0) and \
               time.ticks_diff(now, last_cmd_ms) > COMMAND_TIMEOUT_MS:
                drive.coast_all()
                last_left = last_right = 0.0
                _emit("LOG command timeout - motors coasted")

            # Periodic encoder push.
            if time.ticks_diff(now, last_emit_ms) >= ENC_PERIOD_MS:
                last_emit_ms = now
                dL = enc_left.get_delta()
                dR = enc_right.get_delta()
                pL = enc_left.get_position()
                pR = enc_right.get_position()
                _emit("ENC %d %d %d %d" % (dL, dR, pL, pR))

    except KeyboardInterrupt:
        _emit("LOG KeyboardInterrupt, shutting down")
    except Exception as e:
        _emit("LOG fatal: %r" % e)
    finally:
        try:
            drive.coast_all()
        except Exception:
            pass
        try:
            enc_left.deinit()
            enc_right.deinit()
        except Exception:
            pass
        _emit("BYE")


run()
