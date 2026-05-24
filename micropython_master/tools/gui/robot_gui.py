"""
tools/gui/robot_gui.py
======================
PC-side Textual TUI for driving the robot and watching the encoders.

It speaks the simple line protocol implemented by tools/gui_agent.py on
the ESP32. See tools/gui/README.md for the protocol summary and launch
instructions.

Quick start:
    pip install -r requirements.txt
    python robot_gui.py --port COM5

Keys (when focus is anywhere in the app):
    W / S  -> forward / reverse  (both sides)
    A / D  -> spin left / spin right
    Space  -> soft stop (coast)
    X      -> emergency brake
    +/-    -> nudge max speed up/down (10%)
    Q      -> quit (also coasts the robot)
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from queue import Queue, Empty
from typing import Deque, Optional

try:
    import serial
except ImportError:
    print("Missing dependency: pyserial. Run:  pip install -r requirements.txt")
    sys.exit(1)

try:
    from textual.app import App, ComposeResult
    from textual.binding import Binding
    from textual.containers import Container, Horizontal, Vertical
    from textual.reactive import reactive
    from textual.widgets import (
        Button,
        Footer,
        Header,
        Label,
        RichLog,
        Static,
    )
except ImportError:
    print("Missing dependency: textual. Run:  pip install -r requirements.txt")
    sys.exit(1)


# =============================================================================
# Serial transport
# =============================================================================

@dataclass
class RobotState:
    """Snapshot of robot telemetry, updated by the serial reader thread."""
    connected: bool = False
    ready: bool = False
    last_left_cmd: float = 0.0
    last_right_cmd: float = 0.0
    delta_left: int = 0
    delta_right: int = 0
    pos_left: int = 0
    pos_right: int = 0
    enc_history_left: Deque[int] = field(
        default_factory=lambda: deque(maxlen=80)
    )
    enc_history_right: Deque[int] = field(
        default_factory=lambda: deque(maxlen=80)
    )
    last_msg_time: float = 0.0


class SerialLink:
    """
    Owns the pyserial port. A background thread continuously reads
    lines and pushes them into rx_queue. Writes are direct (small,
    rare, GIL-protected).
    """

    def __init__(self, port: str, baud: int = 115200):
        self.port = port
        self.baud = baud
        self.ser: Optional[serial.Serial] = None
        self.rx_queue: "Queue[str]" = Queue()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def open(self) -> None:
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        self._stop.clear()
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def _reader(self) -> None:
        assert self.ser is not None
        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(128)
            except Exception as e:
                self.rx_queue.put(f"LOG serial read error: {e!r}")
                break
            if not chunk:
                continue
            buf.extend(chunk)
            while b"\n" in buf:
                line, _, rest = buf.partition(b"\n")
                buf = bytearray(rest)
                try:
                    text = line.decode("utf-8", errors="replace").strip()
                except Exception:
                    text = ""
                if text:
                    self.rx_queue.put(text)

    def send(self, line: str) -> None:
        if self.ser is None:
            return
        if not line.endswith("\n"):
            line += "\n"
        try:
            self.ser.write(line.encode("utf-8"))
        except Exception as e:
            self.rx_queue.put(f"LOG serial write error: {e!r}")


# =============================================================================
# Widgets
# =============================================================================

class StatusPanel(Static):
    """Top status line: port, connection, last command."""

    def __init__(self, port: str, baud: int):
        super().__init__()
        self._port = port
        self._baud = baud

    def render_status(self, state: RobotState, max_speed: float) -> str:
        link = "[green]CONNECTED[/green]" if state.connected else "[red]disconnected[/red]"
        ready = "[green]READY[/green]" if state.ready else "[yellow]waiting...[/yellow]"
        return (
            f" Port: [b]{self._port}[/b]@{self._baud}   "
            f"Link: {link}   Agent: {ready}   "
            f"Max speed: [b]{max_speed:.0f}%[/b]   "
            f"Last cmd: L=[b]{state.last_left_cmd:+6.1f}[/b]  "
            f"R=[b]{state.last_right_cmd:+6.1f}[/b]"
        )


class TelemetryPanel(Static):
    """Live encoder readouts + tiny sparkline plot."""

    BARS = " ▁▂▃▄▅▆▇█"

    def render_telemetry(self, state: RobotState) -> str:
        def sparkline(hist: Deque[int]) -> str:
            if not hist:
                return ""
            peak = max(1, max(abs(v) for v in hist))
            out = []
            for v in hist:
                idx = int(abs(v) / peak * (len(self.BARS) - 1))
                ch = self.BARS[idx]
                if v < 0:
                    out.append(f"[red]{ch}[/red]")
                else:
                    out.append(f"[green]{ch}[/green]")
            return "".join(out)

        spark_l = sparkline(state.enc_history_left)
        spark_r = sparkline(state.enc_history_right)

        return (
            "[b]ENCODERS[/b]\n\n"
            f"  Left   delta=[b]{state.delta_left:+5d}[/b]   "
            f"pos=[b]{state.pos_left:+8d}[/b]\n"
            f"  {spark_l}\n\n"
            f"  Right  delta=[b]{state.delta_right:+5d}[/b]   "
            f"pos=[b]{state.pos_right:+8d}[/b]\n"
            f"  {spark_r}\n"
        )


class HelpPanel(Static):
    HELP_TEXT = (
        "[b]CONTROLS[/b]\n\n"
        "  [b]W[/b]  forward         [b]S[/b]  reverse\n"
        "  [b]A[/b]  spin left       [b]D[/b]  spin right\n"
        "  [b]Space[/b]  coast       [b]X[/b]  brake (E-STOP)\n"
        "  [b]+[/b]  max speed +10%  [b]-[/b]  max speed -10%\n"
        "  [b]Q[/b]  quit (coasts)\n\n"
        "Commands are re-sent every 100 ms while a key\n"
        "is held down. The ESP32 also has a 1 s watchdog\n"
        "and will coast automatically if the GUI stops.\n"
    )

    def render(self):  # type: ignore[override]
        return self.HELP_TEXT


# =============================================================================
# App
# =============================================================================

class RobotGUI(App):
    CSS = """
    Screen {
        layout: vertical;
    }
    #status {
        height: 1;
        background: $boost;
        padding: 0 1;
    }
    #body {
        layout: horizontal;
        height: 1fr;
    }
    #left_col, #right_col {
        width: 1fr;
        border: round $accent;
        padding: 1 2;
    }
    #log {
        height: 12;
        border: round $secondary;
    }
    Button.estop {
        background: red;
        color: white;
    }
    """

    BINDINGS = [
        Binding("w", "drive_forward", "Forward", show=False),
        Binding("s", "drive_reverse", "Reverse", show=False),
        Binding("a", "drive_left", "Spin L", show=False),
        Binding("d", "drive_right", "Spin R", show=False),
        Binding("space", "soft_stop", "Stop", show=True),
        Binding("x", "emergency", "E-STOP", show=True),
        Binding("plus", "speed_up", "Speed +", show=True),
        Binding("equals_sign", "speed_up", "Speed +", show=False),
        Binding("minus", "speed_down", "Speed -", show=True),
        Binding("q", "quit_app", "Quit", show=True),
    ]

    max_speed: reactive[float] = reactive(60.0)

    def __init__(self, port: str, baud: int = 115200):
        super().__init__()
        self.state = RobotState()
        self.link = SerialLink(port, baud)
        self._status = StatusPanel(port, baud)
        self._telemetry = TelemetryPanel()
        self._help = HelpPanel()
        self._log = RichLog(id="log", highlight=True, markup=True, wrap=True)
        self._held: dict[str, float] = {}  # key -> last-sent timestamp

    # ---------- composition ----------
    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        yield Static(id="status")
        with Container(id="body"):
            with Vertical(id="left_col"):
                yield Label("[b]Telemetry[/b]")
                yield self._telemetry
            with Vertical(id="right_col"):
                yield Label("[b]Help[/b]")
                yield self._help
                with Horizontal():
                    yield Button("E-STOP (X)", id="estop", classes="estop")
                    yield Button("STOP (Space)", id="stop")
        yield self._log
        yield Footer()

    # ---------- lifecycle ----------
    async def on_mount(self) -> None:
        self.title = "micropython_master  -  Drive GUI"
        try:
            self.link.open()
            self.state.connected = True
            self._log.write(f"[green]Opened {self.link.port} @ {self.link.baud}[/green]")
        except Exception as e:
            self._log.write(f"[red]Failed to open serial: {e!r}[/red]")
        self.set_interval(0.02, self._tick)        # 50 Hz UI + RX drain
        self.set_interval(0.1, self._resend_held)  # 10 Hz key repeat
        self.set_interval(0.25, self._redraw)      # 4 Hz panel redraw

    async def on_unmount(self) -> None:
        try:
            self.link.send("STOP")
        except Exception:
            pass
        self.link.close()

    # ---------- timers ----------
    def _tick(self) -> None:
        drained = 0
        while drained < 64:
            try:
                line = self.link.rx_queue.get_nowait()
            except Empty:
                break
            drained += 1
            self._handle_line(line)

    def _resend_held(self) -> None:
        if not self._held:
            return
        # Combine held keys into a single CMD.
        left = right = 0.0
        ms = self.max_speed
        if "w" in self._held:
            left += ms; right += ms
        if "s" in self._held:
            left -= ms; right -= ms
        if "a" in self._held:
            left -= ms; right += ms
        if "d" in self._held:
            left += ms; right -= ms
        # Clamp.
        left = max(-100.0, min(100.0, left))
        right = max(-100.0, min(100.0, right))
        self._send_cmd(left, right)

    def _redraw(self) -> None:
        self.query_one("#status", Static).update(
            self._status.render_status(self.state, float(self.max_speed))
        )
        self._telemetry.update(self._telemetry.render_telemetry(self.state))

    # ---------- protocol parsing ----------
    def _handle_line(self, line: str) -> None:
        parts = line.split()
        if not parts:
            return
        head = parts[0]

        if head == "READY":
            self.state.ready = True
            self._log.write("[green]Agent READY[/green]")
        elif head == "BYE":
            self.state.ready = False
            self._log.write("[yellow]Agent said BYE[/yellow]")
        elif head == "PONG":
            self._log.write("PONG")
        elif head == "ACK" and len(parts) == 3:
            try:
                self.state.last_left_cmd = float(parts[1])
                self.state.last_right_cmd = float(parts[2])
            except ValueError:
                pass
        elif head == "ENC" and len(parts) == 5:
            try:
                dL, dR, pL, pR = (int(x) for x in parts[1:5])
            except ValueError:
                return
            self.state.delta_left = dL
            self.state.delta_right = dR
            self.state.pos_left = pL
            self.state.pos_right = pR
            self.state.enc_history_left.append(dL)
            self.state.enc_history_right.append(dR)
        elif head == "LOG":
            self._log.write(line[4:])
        else:
            self._log.write(f"[dim]{line}[/dim]")

        self.state.last_msg_time = time.time()

    # ---------- commands ----------
    def _send_cmd(self, left: float, right: float) -> None:
        self.link.send(f"CMD {left:.1f} {right:.1f}")

    # ---------- actions ----------
    def action_drive_forward(self) -> None:
        self._held = {"w": time.time()}
        self._resend_held()

    def action_drive_reverse(self) -> None:
        self._held = {"s": time.time()}
        self._resend_held()

    def action_drive_left(self) -> None:
        self._held = {"a": time.time()}
        self._resend_held()

    def action_drive_right(self) -> None:
        self._held = {"d": time.time()}
        self._resend_held()

    def action_soft_stop(self) -> None:
        self._held.clear()
        self.link.send("STOP")

    def action_emergency(self) -> None:
        self._held.clear()
        self.link.send("BRAKE")
        self._log.write("[red][b]E-STOP[/b][/red]")

    def action_speed_up(self) -> None:
        self.max_speed = float(min(100.0, self.max_speed + 10.0))

    def action_speed_down(self) -> None:
        self.max_speed = float(max(10.0, self.max_speed - 10.0))

    def action_quit_app(self) -> None:
        self.link.send("STOP")
        self.exit()

    # ---------- button presses ----------
    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "estop":
            self.action_emergency()
        elif event.button.id == "stop":
            self.action_soft_stop()


# =============================================================================
# main
# =============================================================================

def main() -> None:
    parser = argparse.ArgumentParser(description="micropython_master drive GUI")
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM5 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    RobotGUI(port=args.port, baud=args.baud).run()


if __name__ == "__main__":
    main()
