"""
tools/wifi_agent.py
===================
Wireless control + calibration agent for the 4WD robot.

Phone or laptop -> WiFi -> ESP32. No USB cable required.

START FROM REPL (or WebREPL):
    >>> import tools.wifi_agent

The robot's IP is printed at boot by boot.py. Open that IP in any modern
browser; you will see a single-page UI for tank-drive, live telemetry,
odometry pose, and one-button odometry calibration.

DESIGN
------
- Pure asyncio. No threads (MicroPython doesn't have them anyway).
- Two HTTP endpoints:
    GET  /                  -> serves the embedded HTML/JS UI
    GET  /events            -> Server-Sent Events (telemetry stream, ~10 Hz)
    POST /cmd               -> JSON command (drive, stop, brake, calibrate, save)
- Auth: shared secret in config.WIFI_AGENT_TOKEN, supplied by the browser as
  the "X-Auth" header on every /cmd and as ?token=... on /events.
- Watchdog: motors coast if no drive command arrives for 1 s.
- All hardware ownership stays inside this one file (encoders + drive
  singletons), exactly the way tools/gui_agent.py does it.

CALIBRATION RUN
---------------
The UI exposes a small calibration card:
  1. You click "Start run" with a chosen speed + duration.
  2. The robot drives in a straight line, accumulating encoder counts.
  3. When it stops, the UI shows you the total counts.
  4. You measure the real distance with a tape measure.
  5. You enter the measured distance (mm) and click "Save".
  6. The robot writes /calibration.json with the new m_per_count.
     (Wiring odometry.py to read it is a separate, optional change.)

This module intentionally does NOT modify odometry.py itself.
"""

import gc
import json
import sys
import time

try:
    import uasyncio as asyncio
except ImportError:
    import asyncio  # CPython fallback for syntax-checking only

import config
from drivers.quad_encoder import QuadEncoder
from robot import drive
from odometry.odometry import Odometry

# Sensors are optional — agent still works for drive + odometry if hardware is missing
try:
    from machine import I2C, Pin
    from sensors import MPU6500, VL53L0X
    _SENSORS_AVAILABLE = True
except Exception:
    _SENSORS_AVAILABLE = False
    MPU6500 = None
    VL53L0X = None
    I2C = None
    Pin = None


# =============================================================================
# Configuration knobs
# =============================================================================

TELEMETRY_PERIOD_MS = 100      # 10 Hz SSE updates
COMMAND_TIMEOUT_MS = 1000      # coast if no drive command for this long
CALIBRATION_FILE = "/calibration.json"


# =============================================================================
# Global state (single instance, owned by this module)
# =============================================================================

class AgentState:
    def __init__(self):
        self.enc_left = None
        self.enc_right = None
        self.odo = None
        self.mpu = None          # MPU6500 (optional)
        self.tof = None          # VL53L0X (optional)
        self.gz = 0.0
        self.range_mm = -1
        self.last_cmd_left = 0.0
        self.last_cmd_right = 0.0
        self.last_cmd_ms = time.ticks_ms()
        self.running = False

        # Latest telemetry snapshot (filled by control loop, read by SSE).
        self.snap = {
            "t": 0,
            "L_d": 0, "R_d": 0,
            "L_pos": 0, "R_pos": 0,
            "cmd_L": 0.0, "cmd_R": 0.0,
            "x": 0.0, "y": 0.0, "th_deg": 0.0,
            "v": 0.0, "w": 0.0,
            "gz": 0.0, "gz_deg": 0.0,     # IMU yaw rate (from MPU6500)
            "range_mm": -1,              # VL53L0X distance (-1 = no reading)
            "cal": None,         # populated only during a calibration run
        }

        # Calibration session (None when idle).
        self.cal = None


STATE = AgentState()


# =============================================================================
# Hardware lifecycle
# =============================================================================

def _hw_init():
    if STATE.enc_left is None:
        STATE.enc_left = QuadEncoder(
            config.ENCODER_LEFT_A, config.ENCODER_LEFT_B,
            invert=config.ENCODER_LEFT_INVERT, name="left",
        )
        STATE.enc_right = QuadEncoder(
            config.ENCODER_RIGHT_A, config.ENCODER_RIGHT_B,
            invert=config.ENCODER_RIGHT_INVERT, name="right",
        )
        STATE.odo = Odometry()

    # Initialize I2C sensors on a shared bus (graceful degradation if hardware missing)
    if _SENSORS_AVAILABLE and (STATE.mpu is None or STATE.tof is None):
        try:
            i2c = I2C(0, sda=Pin(config.I2C_SDA), scl=Pin(config.I2C_SCL),
                      freq=config.I2C_FREQ)

            if STATE.mpu is None:
                STATE.mpu = MPU6500(i2c)
                try:
                    STATE.mpu.calibrate(count=80, delay_ms=3)
                    print("[wifi_agent] MPU6500 ready, bias captured")
                except Exception as e:
                    print("[wifi_agent] MPU6500 calibrate warning:", e)

            if STATE.tof is None:
                STATE.tof = VL53L0X(i2c)
                STATE.tof.start_continuous()
                print("[wifi_agent] VL53L0X ready")

        except Exception as e:
            print("[wifi_agent] Sensor init issue (continuing):", e)
            if STATE.mpu is None:
                STATE.mpu = None
            if STATE.tof is None:
                STATE.tof = None


def _hw_deinit():
    try:
        drive.coast_all()
    except Exception:
        pass
    try:
        if STATE.enc_left is not None:
            STATE.enc_left.deinit()
        if STATE.enc_right is not None:
            STATE.enc_right.deinit()
    except Exception:
        pass


# =============================================================================
# Control loop  (50 Hz: drives encoders + odometry; SSE samples this at 10 Hz)
# =============================================================================

async def control_loop():
    period_ms = 20
    last_t = time.ticks_ms()

    while STATE.running:
        await asyncio.sleep_ms(period_ms)
        now = time.ticks_ms()
        dt_ms = time.ticks_diff(now, last_t)
        last_t = now
        dt_s = dt_ms / 1000.0

        dL = STATE.enc_left.get_delta()
        dR = STATE.enc_right.get_delta()
        pL = STATE.enc_left.get_position()
        pR = STATE.enc_right.get_position()

        # IMU yaw rate (if available)
        gz = 0.0
        if STATE.mpu is not None:
            try:
                gz = STATE.mpu.gyro_z
            except Exception:
                gz = 0.0
        STATE.gz = gz

        # VL53L0X ranging (non-blocking best effort)
        rng = -1
        if STATE.tof is not None:
            try:
                rng = STATE.tof.range_mm
            except Exception:
                rng = -1
        STATE.range_mm = rng

        # Odometry with gyro fusion when IMU is present
        STATE.odo.update_from_encoders(dL, dR, dt_s, gyro_z=gz)
        x, y, th = STATE.odo.get_pose()
        v, w = STATE.odo.get_velocities()

        STATE.snap["t"] = now
        STATE.snap["L_d"] = dL
        STATE.snap["R_d"] = dR
        STATE.snap["L_pos"] = pL
        STATE.snap["R_pos"] = pR
        STATE.snap["cmd_L"] = STATE.last_cmd_left
        STATE.snap["cmd_R"] = STATE.last_cmd_right
        STATE.snap["x"] = x
        STATE.snap["y"] = y
        STATE.snap["th_deg"] = th * 57.29578
        STATE.snap["v"] = v
        STATE.snap["w"] = w
        STATE.snap["gz"] = gz
        STATE.snap["gz_deg"] = gz * 57.29578
        STATE.snap["range_mm"] = rng

        # Calibration accumulation
        if STATE.cal is not None and STATE.cal["active"]:
            STATE.cal["counts_L"] += dL
            STATE.cal["counts_R"] += dR
            STATE.cal["elapsed_ms"] = time.ticks_diff(now, STATE.cal["t_start"])
            if STATE.cal["elapsed_ms"] >= STATE.cal["duration_ms"]:
                STATE.cal["active"] = False
                drive.brake_all()
                STATE.last_cmd_left = 0.0
                STATE.last_cmd_right = 0.0
                STATE.cal["finished"] = True
            STATE.snap["cal"] = {
                "active": STATE.cal["active"],
                "finished": STATE.cal.get("finished", False),
                "counts_L": STATE.cal["counts_L"],
                "counts_R": STATE.cal["counts_R"],
                "elapsed_ms": STATE.cal["elapsed_ms"],
                "duration_ms": STATE.cal["duration_ms"],
                "speed": STATE.cal["speed"],
            }
        else:
            STATE.snap["cal"] = None

        # Watchdog: stop motors if no command in 1 s
        if (STATE.last_cmd_left != 0.0 or STATE.last_cmd_right != 0.0):
            if time.ticks_diff(now, STATE.last_cmd_ms) > COMMAND_TIMEOUT_MS:
                # Don't stomp on an active calibration run
                if STATE.cal is None or not STATE.cal["active"]:
                    drive.coast_all()
                    STATE.last_cmd_left = 0.0
                    STATE.last_cmd_right = 0.0


# =============================================================================
# HTTP helpers
# =============================================================================

def _parse_request(raw_head):
    """
    Parse minimal HTTP head. Returns (method, path, query, headers).
    raw_head is a bytes object holding everything up to "\r\n\r\n".
    """
    text = raw_head.decode("utf-8", "replace")
    lines = text.split("\r\n")
    if not lines:
        return None, None, {}, {}
    parts = lines[0].split(" ")
    if len(parts) < 2:
        return None, None, {}, {}
    method, target = parts[0], parts[1]
    path, _, query_str = target.partition("?")
    query = {}
    if query_str:
        for kv in query_str.split("&"):
            k, _, v = kv.partition("=")
            query[k] = v
    headers = {}
    for line in lines[1:]:
        if ":" in line:
            k, _, v = line.partition(":")
            headers[k.strip().lower()] = v.strip()
    return method, path, query, headers


def _check_token(headers, query):
    expected = getattr(config, "WIFI_AGENT_TOKEN", "")
    if not expected:
        return True
    return (headers.get("x-auth") == expected) or (query.get("token") == expected)


async def _send(writer, status_line, headers, body=b""):
    out = b"HTTP/1.0 " + status_line + b"\r\n"
    for k, v in headers.items():
        out += k.encode() + b": " + v.encode() + b"\r\n"
    out += b"Content-Length: " + str(len(body)).encode() + b"\r\n"
    out += b"\r\n"
    if body:
        out += body
    writer.write(out)
    await writer.drain()


async def _send_json(writer, obj, status=b"200 OK"):
    body = json.dumps(obj).encode()
    await _send(writer, status, {"Content-Type": "application/json",
                                 "Cache-Control": "no-store"}, body)


# =============================================================================
# Command handler
# =============================================================================

def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def _handle_cmd(payload):
    """
    Returns (status_ok, response_dict).
    """
    op = payload.get("op", "")

    if op == "drive":
        left = _clamp(float(payload.get("left", 0.0)), -100.0, 100.0)
        right = _clamp(float(payload.get("right", 0.0)), -100.0, 100.0)
        drive.set_side_speeds(left, right)
        STATE.last_cmd_left = left
        STATE.last_cmd_right = right
        STATE.last_cmd_ms = time.ticks_ms()
        return True, {"ok": True, "L": left, "R": right}

    if op == "stop":
        drive.coast_all()
        STATE.last_cmd_left = 0.0
        STATE.last_cmd_right = 0.0
        STATE.last_cmd_ms = time.ticks_ms()
        return True, {"ok": True}

    if op == "brake":
        drive.brake_all()
        STATE.last_cmd_left = 0.0
        STATE.last_cmd_right = 0.0
        STATE.last_cmd_ms = time.ticks_ms()
        return True, {"ok": True}

    if op == "reset_odom":
        STATE.odo.reset()
        return True, {"ok": True}

    if op == "cal_start":
        if STATE.cal is not None and STATE.cal["active"]:
            return False, {"ok": False, "err": "calibration already running"}
        speed = _clamp(float(payload.get("speed", 30.0)), 10.0, 80.0)
        duration_ms = int(_clamp(float(payload.get("duration_s", 8.0)), 1.0, 30.0) * 1000)
        STATE.odo.reset()
        STATE.enc_left.get_delta()    # flush
        STATE.enc_right.get_delta()
        STATE.cal = {
            "active": True,
            "finished": False,
            "speed": speed,
            "duration_ms": duration_ms,
            "elapsed_ms": 0,
            "counts_L": 0,
            "counts_R": 0,
            "t_start": time.ticks_ms(),
        }
        drive.set_side_speeds(speed, speed)
        STATE.last_cmd_left = speed
        STATE.last_cmd_right = speed
        STATE.last_cmd_ms = time.ticks_ms()
        return True, {"ok": True, "duration_ms": duration_ms, "speed": speed}

    if op == "cal_abort":
        drive.brake_all()
        STATE.last_cmd_left = 0.0
        STATE.last_cmd_right = 0.0
        if STATE.cal is not None:
            STATE.cal["active"] = False
            STATE.cal["finished"] = True
        return True, {"ok": True}

    if op == "cal_save":
        measured_mm = float(payload.get("measured_mm", 0.0))
        if measured_mm <= 0:
            return False, {"ok": False, "err": "measured_mm must be > 0"}
        if STATE.cal is None or STATE.cal["counts_L"] == 0 or STATE.cal["counts_R"] == 0:
            return False, {"ok": False, "err": "no calibration counts available"}
        avg_counts = (STATE.cal["counts_L"] + STATE.cal["counts_R"]) / 2.0
        new_mpc = (measured_mm / 1000.0) / avg_counts
        data = {
            "m_per_count": new_mpc,
            "measured_mm": measured_mm,
            "counts_L": STATE.cal["counts_L"],
            "counts_R": STATE.cal["counts_R"],
            "avg_counts": avg_counts,
            "speed": STATE.cal["speed"],
            "duration_ms": STATE.cal["duration_ms"],
            "saved_at_ms": time.ticks_ms(),
        }
        try:
            with open(CALIBRATION_FILE, "w") as f:
                json.dump(data, f)
        except Exception as e:
            return False, {"ok": False, "err": "write failed: %r" % e}
        STATE.odo.set_scale(new_mpc)
        return True, {"ok": True, "m_per_count": new_mpc, "path": CALIBRATION_FILE}

    if op == "info":
        try:
            with open(CALIBRATION_FILE) as f:
                cal_file = json.load(f)
        except Exception:
            cal_file = None
        return True, {
            "ok": True,
            "m_per_count": STATE.odo.m_per_count,
            "wheel_diameter_mm": config.WHEEL_DIAMETER_MM,
            "track_width_mm": config.TRACK_WIDTH_MM,
            "saved_calibration": cal_file,
        }

    return False, {"ok": False, "err": "unknown op: %s" % op}


# =============================================================================
# HTTP request dispatcher
# =============================================================================

async def _handle_client(reader, writer):
    try:
        # Read headers up to blank line.
        head = b""
        while b"\r\n\r\n" not in head:
            chunk = await reader.read(256)
            if not chunk:
                return
            head += chunk
            if len(head) > 4096:
                await _send(writer, b"431 Request Header Fields Too Large", {}, b"")
                return

        sep = head.index(b"\r\n\r\n")
        body = head[sep + 4:]
        head = head[:sep + 2]

        method, path, query, headers = _parse_request(head)
        if method is None:
            await _send(writer, b"400 Bad Request", {}, b"")
            return

        # --- routing ---
        if method == "GET" and path == "/":
            await _send(writer, b"200 OK",
                        {"Content-Type": "text/html; charset=utf-8"},
                        INDEX_HTML.encode())
            return

        if method == "GET" and path == "/events":
            if not _check_token(headers, query):
                await _send(writer, b"401 Unauthorized", {}, b"unauthorized")
                return
            await _stream_sse(writer)
            return

        if method == "POST" and path == "/cmd":
            if not _check_token(headers, query):
                await _send_json(writer, {"ok": False, "err": "unauthorized"},
                                 status=b"401 Unauthorized")
                return
            # Read remaining body if Content-Length says so.
            try:
                cl = int(headers.get("content-length", "0"))
            except ValueError:
                cl = 0
            while len(body) < cl:
                more = await reader.read(cl - len(body))
                if not more:
                    break
                body += more
            try:
                payload = json.loads(body.decode("utf-8")) if body else {}
            except Exception:
                await _send_json(writer, {"ok": False, "err": "bad json"},
                                 status=b"400 Bad Request")
                return
            ok, resp = _handle_cmd(payload)
            await _send_json(writer, resp,
                             status=(b"200 OK" if ok else b"400 Bad Request"))
            return

        await _send(writer, b"404 Not Found", {}, b"not found")

    except Exception as e:
        try:
            sys.print_exception(e)
        except Exception:
            pass
        try:
            await _send(writer, b"500 Internal Server Error", {}, b"server error")
        except Exception:
            pass
    finally:
        try:
            await writer.aclose() if hasattr(writer, "aclose") else writer.close()
        except Exception:
            pass


async def _stream_sse(writer):
    writer.write(
        b"HTTP/1.0 200 OK\r\n"
        b"Content-Type: text/event-stream\r\n"
        b"Cache-Control: no-store\r\n"
        b"Connection: close\r\n"
        b"\r\n"
    )
    await writer.drain()

    try:
        while STATE.running:
            payload = json.dumps(STATE.snap).encode()
            writer.write(b"data: " + payload + b"\n\n")
            await writer.drain()
            await asyncio.sleep_ms(TELEMETRY_PERIOD_MS)
    except Exception:
        return


# =============================================================================
# Embedded UI  (served from "/")
# =============================================================================
# Kept as a module-level constant so the bytes live in flash, not RAM.

INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>Robot Control</title>
<style>
:root {
  --bg:#0e1116; --card:#181c24; --ink:#e8ecf2; --mute:#8b95a7;
  --acc:#4cc9f0; --ok:#5cd6a0; --warn:#f0b455; --err:#f06b6b;
  --rad:14px;
}
*{box-sizing:border-box}
html,body{margin:0;padding:0;background:var(--bg);color:var(--ink);
  font:14px/1.4 system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
header{padding:10px 14px;border-bottom:1px solid #232936;display:flex;
  align-items:center;gap:10px;flex-wrap:wrap}
header h1{font-size:16px;margin:0;font-weight:600}
.dot{width:10px;height:10px;border-radius:50%;background:var(--mute)}
.dot.ok{background:var(--ok)}.dot.err{background:var(--err)}
main{padding:10px;display:grid;grid-template-columns:1fr;gap:10px;max-width:760px;margin:0 auto}
.card{background:var(--card);border:1px solid #232936;border-radius:var(--rad);
  padding:12px}
.card h2{margin:0 0 8px 0;font-size:13px;color:var(--mute);
  text-transform:uppercase;letter-spacing:.08em;font-weight:600}
.row{display:flex;gap:8px;flex-wrap:wrap}
.kv{display:grid;grid-template-columns:auto 1fr;gap:4px 14px;font-variant-numeric:tabular-nums}
.kv b{color:var(--acc);font-weight:600}
button{font:inherit;border:0;border-radius:10px;padding:14px 12px;
  background:#222a3a;color:var(--ink);cursor:pointer;min-width:64px}
button:active{transform:translateY(1px)}
button.primary{background:var(--acc);color:#0a141c;font-weight:600}
button.danger{background:var(--err);color:#fff;font-weight:600}
button.warn{background:var(--warn);color:#241500;font-weight:600}
.pad{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;
  max-width:300px;margin:0 auto}
.pad .sp{visibility:hidden}
.pad button{height:60px;font-size:16px;font-weight:700}
.bar{height:6px;background:#222a36;border-radius:3px;overflow:hidden;margin-top:6px}
.bar > div{height:100%;background:var(--acc);transition:width 80ms linear}
.bar.rev > div{background:var(--warn)}
canvas{width:100%;height:90px;background:#0a0d12;border-radius:8px;
  display:block;border:1px solid #1d2330}
input[type=number],input[type=password]{background:#0e1218;border:1px solid #2a3142;
  border-radius:8px;color:var(--ink);padding:8px 10px;font:inherit;width:120px}
label{color:var(--mute);font-size:12px}
.gridcols{display:grid;grid-template-columns:1fr 1fr;gap:10px}
@media(max-width:520px){.gridcols{grid-template-columns:1fr}}
.smallnote{color:var(--mute);font-size:12px;margin-top:6px}
.tag{display:inline-block;padding:2px 8px;border-radius:999px;
  background:#1f2532;color:var(--mute);font-size:12px}
.tag.ok{background:#13332b;color:var(--ok)}
.tag.err{background:#3a1c1c;color:var(--err)}
.tag.run{background:#33291a;color:var(--warn)}
</style>
</head>
<body>
<header>
  <div class="dot" id="dot"></div>
  <h1>Robot Control</h1>
  <span class="tag" id="link">disconnected</span>
  <span style="flex:1"></span>
  <button id="auth" title="Set token">token</button>
</header>

<main>

<section class="card">
  <h2>Drive</h2>
  <div class="pad">
    <div class="sp"></div>
    <button id="fwd">▲</button>
    <div class="sp"></div>
    <button id="left">◄</button>
    <button id="stop" class="warn">STOP</button>
    <button id="right">►</button>
    <div class="sp"></div>
    <button id="rev">▼</button>
    <div class="sp"></div>
  </div>
  <div class="row" style="justify-content:center;margin-top:10px">
    <label>Speed <input type="number" id="speed" value="40" min="10" max="100" step="5"></label>
    <button id="estop" class="danger">E-STOP (brake)</button>
  </div>
</section>

<section class="card">
  <h2>Telemetry</h2>
  <div class="gridcols">
    <div>
      <div class="kv">
        <span>Left  Δ</span><b id="dL">0</b>
        <span>Right Δ</span><b id="dR">0</b>
        <span>Left  pos</span><b id="pL">0</b>
        <span>Right pos</span><b id="pR">0</b>
        <span>Cmd L</span><b id="cL">0.0</b>
        <span>Cmd R</span><b id="cR">0.0</b>
      </div>
    </div>
    <div>
      <div class="kv">
        <span>x</span><b id="x">0.000 m</b>
        <span>y</span><b id="y">0.000 m</b>
        <span>θ</span><b id="th">0.0°</b>
        <span>v</span><b id="v">0.000 m/s</b>
        <span>ω</span><b id="w">0.000 rad/s</b>
      </div>
      <div style="margin-top:6px"><button id="resetodom">reset pose</button></div>
    </div>
    <div>
      <div class="kv" style="font-size:0.92em">
        <span>gz</span><b id="gz">0.00 rad/s</b>
        <span>Yaw rate</span><b id="gzdeg">0.0 °/s</b>
        <span>Range</span><b id="rng">-- mm</b>
      </div>
      <div class="smallnote" style="margin-top:2px">MPU6500 + VL53L0X (optional)</div>
    </div>
  </div>
  <div style="margin-top:8px">
    <div>Left  Δ <span id="dLpct" class="tag">0</span></div>
    <div class="bar" id="bL"><div></div></div>
    <div style="margin-top:6px">Right Δ <span id="dRpct" class="tag">0</span></div>
    <div class="bar" id="bR"><div></div></div>
  </div>
  <canvas id="plot" width="700" height="90"></canvas>
  <div class="smallnote">green = left Δ, blue = right Δ. last ~10 s.</div>
</section>

<section class="card">
  <h2>Odometry calibration</h2>
  <div class="row">
    <label>Speed % <input type="number" id="calSpeed" value="30" min="10" max="80" step="5"></label>
    <label>Duration s <input type="number" id="calDuration" value="8" min="1" max="30" step="1"></label>
    <button id="calStart" class="primary">Start run</button>
    <button id="calAbort" class="warn">Abort</button>
  </div>
  <div class="smallnote" id="calStatus">idle</div>

  <div class="row" style="margin-top:10px">
    <label>Measured distance (mm)
      <input type="number" id="measured" placeholder="e.g. 1000" min="1" step="1">
    </label>
    <button id="calSave" class="primary">Compute & save</button>
  </div>
  <div class="smallnote" id="calSaveStatus"></div>
  <div class="kv" style="margin-top:8px">
    <span>current m_per_count</span><b id="mpc">-</b>
    <span>saved m_per_count</span><b id="mpcSaved">-</b>
  </div>
</section>

</main>

<script>
const $ = id => document.getElementById(id);
let token = localStorage.getItem("robotToken") || "robot1234";
let es = null;
let cmdInFlight = false;
let heldDir = null;       // "fwd","rev","left","right" or null
let resendTimer = null;
let plot = $("plot");
let plotCtx = plot.getContext("2d");
let history = [];         // [{L,R}, ...]

function setLink(state) {
  const dot = $("dot"), link = $("link");
  dot.className = "dot " + (state==="ok"?"ok":(state==="err"?"err":""));
  link.textContent = state==="ok"?"connected":(state==="err"?"error":"connecting...");
  link.className = "tag " + (state==="ok"?"ok":(state==="err"?"err":""));
}

function api(payload) {
  return fetch("/cmd", {
    method:"POST",
    headers:{"Content-Type":"application/json","X-Auth":token},
    body: JSON.stringify(payload),
  }).then(r => r.json()).catch(e => ({ok:false, err:String(e)}));
}

function startStream() {
  if (es) try { es.close(); } catch(e){}
  setLink("...");
  es = new EventSource("/events?token=" + encodeURIComponent(token));
  es.onopen = () => setLink("ok");
  es.onerror = () => setLink("err");
  es.onmessage = ev => {
    let s;
    try { s = JSON.parse(ev.data); } catch(e){ return; }
    $("dL").textContent = s.L_d;
    $("dR").textContent = s.R_d;
    $("pL").textContent = s.L_pos;
    $("pR").textContent = s.R_pos;
    $("cL").textContent = s.cmd_L.toFixed(1);
    $("cR").textContent = s.cmd_R.toFixed(1);
    $("x").textContent  = s.x.toFixed(3) + " m";
    $("y").textContent  = s.y.toFixed(3) + " m";
    $("th").textContent = s.th_deg.toFixed(1) + "°";
    $("v").textContent  = s.v.toFixed(3) + " m/s";
    $("w").textContent  = s.w.toFixed(3) + " rad/s";

    // IMU + ToF (new)
    if (typeof s.gz !== "undefined") {
        $("gz").textContent = s.gz.toFixed(3) + " rad/s";
        $("gzdeg").textContent = s.gz_deg.toFixed(1) + " °/s";
    }
    if (typeof s.range_mm !== "undefined") {
        $("rng").textContent = (s.range_mm >= 0) ? (s.range_mm + " mm") : "--";
    }

    // Bars (scale to ±60 counts which is roughly full speed in 100ms tick)
    const scale = 60;
    function setBar(el, dotEl, val) {
      const pct = Math.min(100, Math.abs(val)/scale*100);
      el.firstElementChild.style.width = pct + "%";
      el.classList.toggle("rev", val < 0);
      dotEl.textContent = val;
    }
    setBar($("bL"), $("dLpct"), s.L_d);
    setBar($("bR"), $("dRpct"), s.R_d);

    history.push({L:s.L_d, R:s.R_d});
    if (history.length > 100) history.shift();
    drawPlot();

    // Calibration update
    if (s.cal) {
      const c = s.cal;
      const pctTime = Math.min(100, (c.elapsed_ms/c.duration_ms)*100).toFixed(0);
      if (c.active) {
        $("calStatus").innerHTML =
          `<span class="tag run">RUNNING</span> ${pctTime}%  -  L=${c.counts_L}  R=${c.counts_R}  speed=${c.speed}%`;
      } else if (c.finished) {
        $("calStatus").innerHTML =
          `<span class="tag ok">DONE</span> L=${c.counts_L}  R=${c.counts_R}  avg=${((c.counts_L+c.counts_R)/2).toFixed(0)}`;
      }
    } else {
      // leave the last status alone
    }
  };
}

function drawPlot() {
  const ctx = plotCtx;
  const W = plot.width, H = plot.height;
  ctx.clearRect(0,0,W,H);
  ctx.strokeStyle = "#1d2330"; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(0,H/2); ctx.lineTo(W,H/2); ctx.stroke();
  if (history.length < 2) return;
  const peak = Math.max(20, ...history.map(p => Math.max(Math.abs(p.L), Math.abs(p.R))));
  const stepX = W / (history.length - 1);
  function line(getter, color) {
    ctx.strokeStyle = color; ctx.lineWidth = 1.5;
    ctx.beginPath();
    for (let i=0;i<history.length;i++) {
      const v = getter(history[i]);
      const x = i * stepX;
      const y = H/2 - (v/peak) * (H/2 - 4);
      if (i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
    }
    ctx.stroke();
  }
  line(p => p.L, "#5cd6a0");
  line(p => p.R, "#4cc9f0");
}

// ---------- driving ----------
function speedVal() { return parseFloat($("speed").value) || 0; }

function applyHeld() {
  const s = speedVal();
  let L=0, R=0;
  if (heldDir === "fwd")   { L= s; R= s; }
  else if (heldDir === "rev")   { L=-s; R=-s; }
  else if (heldDir === "left")  { L=-s; R= s; }
  else if (heldDir === "right") { L= s; R=-s; }
  api({op:"drive", left:L, right:R});
}

function hold(dir) {
  heldDir = dir;
  applyHeld();
  if (resendTimer) clearInterval(resendTimer);
  resendTimer = setInterval(applyHeld, 250);
}

function release() {
  heldDir = null;
  if (resendTimer) { clearInterval(resendTimer); resendTimer = null; }
  api({op:"stop"});
}

function bindHold(id, dir) {
  const b = $(id);
  const start = e => { e.preventDefault(); hold(dir); };
  const end   = e => { e.preventDefault(); release(); };
  b.addEventListener("mousedown", start);
  b.addEventListener("touchstart", start, {passive:false});
  b.addEventListener("mouseup", end);
  b.addEventListener("mouseleave", e => { if (heldDir === dir) release(); });
  b.addEventListener("touchend", end);
  b.addEventListener("touchcancel", end);
}
bindHold("fwd","fwd"); bindHold("rev","rev");
bindHold("left","left"); bindHold("right","right");

$("stop").onclick = () => release();
$("estop").onclick = () => { release(); api({op:"brake"}); };
$("resetodom").onclick = () => api({op:"reset_odom"});

// Keyboard for laptop users
window.addEventListener("keydown", e => {
  if (e.repeat) return;
  if (e.key === "w") hold("fwd");
  else if (e.key === "s") hold("rev");
  else if (e.key === "a") hold("left");
  else if (e.key === "d") hold("right");
  else if (e.key === " ") release();
  else if (e.key.toLowerCase() === "x") { release(); api({op:"brake"}); }
});
window.addEventListener("keyup", e => {
  if (["w","a","s","d"].includes(e.key)) release();
});

// ---------- calibration ----------
$("calStart").onclick = async () => {
  const speed = parseFloat($("calSpeed").value);
  const duration_s = parseFloat($("calDuration").value);
  const r = await api({op:"cal_start", speed, duration_s});
  if (!r.ok) $("calStatus").innerHTML = `<span class="tag err">${r.err||"failed"}</span>`;
};
$("calAbort").onclick = () => api({op:"cal_abort"});

$("calSave").onclick = async () => {
  const measured_mm = parseFloat($("measured").value);
  if (!(measured_mm > 0)) {
    $("calSaveStatus").innerHTML = `<span class="tag err">enter measured mm</span>`;
    return;
  }
  const r = await api({op:"cal_save", measured_mm});
  if (r.ok) {
    $("calSaveStatus").innerHTML =
      `<span class="tag ok">saved</span>  m_per_count=${r.m_per_count.toExponential(4)}  -> ${r.path}`;
    refreshInfo();
  } else {
    $("calSaveStatus").innerHTML = `<span class="tag err">${r.err}</span>`;
  }
};

async function refreshInfo() {
  const r = await api({op:"info"});
  if (r.ok) {
    $("mpc").textContent = r.m_per_count.toExponential(4);
    if (r.saved_calibration) {
      $("mpcSaved").textContent =
        r.saved_calibration.m_per_count.toExponential(4) +
        "  (measured " + r.saved_calibration.measured_mm + " mm)";
    } else {
      $("mpcSaved").textContent = "(none yet)";
    }
  }
}

// ---------- token UI ----------
$("auth").onclick = () => {
  const t = prompt("Robot auth token:", token);
  if (t !== null) {
    token = t;
    localStorage.setItem("robotToken", token);
    startStream();
  }
};

// boot
startStream();
refreshInfo();
</script>
</body>
</html>"""


# =============================================================================
# Entry point
# =============================================================================

def _get_ip():
    try:
        import network
        wlan = network.WLAN(network.STA_IF)
        if wlan.isconnected():
            return wlan.ifconfig()[0]
    except Exception:
        pass
    return None


async def _main():
    _hw_init()
    STATE.running = True

    port = getattr(config, "WIFI_AGENT_PORT", 80)
    ip = _get_ip() or "0.0.0.0"

    print("=" * 60)
    print(" WIFI AGENT")
    print(" Open in browser: http://{}:{}/".format(ip, port))
    print(" Token: {}".format(getattr(config, "WIFI_AGENT_TOKEN", "(none)")))
    print(" Endpoints: GET /  GET /events  POST /cmd  (+ live gz + range)")
    print("=" * 60)

    # Apply persisted calibration if present (read-only, no edits to odometry.py)
    try:
        with open(CALIBRATION_FILE) as f:
            saved = json.load(f)
        if "m_per_count" in saved:
            STATE.odo.set_scale(saved["m_per_count"])
            print(" Loaded saved m_per_count from {}".format(CALIBRATION_FILE))
    except Exception:
        pass

    gc.collect()

    server = await asyncio.start_server(_handle_client, "0.0.0.0", port)
    loop_task = asyncio.create_task(control_loop())

    try:
        # Keep running until cancelled.
        while STATE.running:
            await asyncio.sleep_ms(500)
    finally:
        STATE.running = False
        try:
            loop_task.cancel()
        except Exception:
            pass
        try:
            server.close()
            await server.wait_closed()
        except Exception:
            pass
        _hw_deinit()
        print("[wifi_agent] stopped.")


def run():
    """Public entry point. Blocks until Ctrl-C."""
    try:
        asyncio.run(_main())
    except KeyboardInterrupt:
        print("[wifi_agent] KeyboardInterrupt")
        _hw_deinit()


# Auto-run when imported (so `import tools.wifi_agent` is enough).
run()
