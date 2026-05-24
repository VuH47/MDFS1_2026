"""
robot/drive.py
Unified drive layer – "same command per side" for the 4WD robot.

Left side  = Front-Left (TB6612 PWM + direction)  + Rear-Left (L298N direction only)
Right side = Front-Right (TB6612 PWM + direction) + Rear-Right (L298N direction only)

IMPORTANT (as of latest hardware update):
- Rear L298N is now physically connected (both sides).
- ENA/ENB are hard-tied high → rear only follows sign of the side command.
- Motor polarity inversions are handled via config.MOTOR_LEFT_INVERT / MOTOR_RIGHT_INVERT
  (set after Phase 1 test results in debug log.txt).

This is the single place higher layers should call to move the robot.

Current state: open-loop only (Phase 1/2). Speed % on front, direction on rear.
Kinematics + closed loop coming in Phase 3+.

All pins and invert flags come from config.py (single source of truth).
"""

from drivers.tb6612 import create_left as _tb_left, create_right as _tb_right
from drivers.l298n  import create_rear_left as _l298_left, create_rear_right as _l298_right


# Module-level singletons (created on first import – cheap and REPL friendly)
_left_front = None
_right_front = None
_left_rear = None
_right_rear = None


def _ensure_motors():
    global _left_front, _right_front, _left_rear, _right_rear
    if _left_front is None:
        _left_front = _tb_left()
        _right_front = _tb_right()
        _left_rear = _l298_left()
        _right_rear = _l298_right()


def set_side_speeds(left: float, right: float, *, brake_on_zero: bool = False):
    """
    Core primitive for the entire robot.

    :param left:  -100..+100  (percent for front-left TB6612)
    :param right: -100..+100  (percent for front-right TB6612)
    :param brake_on_zero: if True, use brake() instead of coast() when speed==0
    """
    _ensure_motors()

    # Front (PWM + direction)
    _left_front.set(left)
    _right_front.set(right)

    # Rear (direction only – follows sign of the command for that side)
    _left_rear.set_direction(left)
    _right_rear.set_direction(right)

    # Optional stronger stop
    if brake_on_zero:
        if left == 0:
            _left_front.brake()
        if right == 0:
            _right_front.brake()


def brake_all():
    """Emergency / strong stop – short-brakes the front motors, coasts rear."""
    _ensure_motors()
    _left_front.brake()
    _right_front.brake()
    _left_rear.stop()
    _right_rear.stop()


def coast_all():
    """Freewheel stop."""
    _ensure_motors()
    _left_front.coast()
    _right_front.coast()
    _left_rear.stop()
    _right_rear.stop()


def stop():
    """Alias for the safest default stop (coast)."""
    coast_all()


def get_last_commands():
    """Debug helper: returns (left_speed, right_speed) last sent to fronts."""
    _ensure_motors()
    return (_left_front.get_last_speed(), _right_front.get_last_speed())


def get_rear_directions():
    """Debug helper (especially useful now rear L298N is connected).
    Returns (left_dir, right_dir) where +1 = forward, -1 = reverse, 0 = stop.
    """
    _ensure_motors()
    return (_left_rear.get_last_dir(), _right_rear.get_last_dir())


def deinit_all():
    """Clean shutdown for REPL / reload."""
    _ensure_motors()
    _left_front.deinit()
    _right_front.deinit()
    _left_rear.deinit()
    _right_rear.deinit()


# Future (Phase 3+)
# def set_velocity(v_mm_s: float, omega_rad_s: float): ...
# def set_wheel_speeds(wL, wR): ...   # after odometry scale factors exist