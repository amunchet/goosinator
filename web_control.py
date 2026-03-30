#!/usr/bin/env python3

import threading
import time
from dataclasses import dataclass

import cv2
from flask import Flask, Response, jsonify, render_template, request
from gpiozero import LED
from picamera2 import Picamera2

from PCA9685 import PCA9685


# Hardware/channel conventions
R_CHANNEL = 4  # rotation (left/right)
Y_CHANNEL = 0  # vertical (up/down)
PWM_FREQ_HZ = 50

# Known safe-ish limits from limits.txt
R_MIN = 889
R_MAX = 1149
Y_MIN = 917
Y_MAX = 1117

# Backlash compensation (intentional): direction-specific deltas
# INC = target > current, DEC = target < current
R_BACKLASH_DELTA_INC = 10
R_BACKLASH_DELTA_DEC = 14
Y_BACKLASH_DELTA_INC = 10
Y_BACKLASH_DELTA_DEC = 14
R_APPROACH_FROM_HIGH = True
Y_APPROACH_FROM_HIGH = True
BACKLASH_SETTLE_SEC = 0.02
AXIS_MOVE_DELAY_SEC = 0.08

# Deadband compensation (pulse units)
# Small commands below deadband are accumulated until movement is meaningful.
R_DEADBAND = 8
Y_DEADBAND = 10

# Direction tuning
# Jog buttons were intentionally inverted to match user expectation.
INVERT_JOG_R = True
INVERT_JOG_Y = True
# Click mapping uses calibration directly; keep non-inverted unless camera is mirrored.
INVERT_CLICK_R = False
INVERT_CLICK_Y = False


@dataclass
class Calibration:
    # Image-space edges mapped to servo pulses
    # Left image edge -> r_left; Right image edge -> r_right
    # Top image edge -> y_top; Bottom image edge -> y_bottom
    r_left: int = R_MAX
    r_right: int = R_MIN
    y_top: int = Y_MAX
    y_bottom: int = Y_MIN
    
    # Click-based calibration points (x/y click position, servo position)
    left_point: tuple[float, int] | None = None  # (x_click, r_servo)
    right_point: tuple[float, int] | None = None
    top_point: tuple[float, int] | None = None  # (y_click, y_servo)
    bottom_point: tuple[float, int] | None = None
    
    def extrapolate_edges(self) -> bool:
        """Calculate image edges from calibration points. Returns True if successful."""
        # Need left and right points for R axis
        if self.left_point and self.right_point:
            x1, r1 = self.left_point
            x2, r2 = self.right_point
            if abs(x2 - x1) > 0.1:  # Require reasonable separation
                slope = (r2 - r1) / (x2 - x1)
                self.r_left = round(r1 - slope * x1)  # Extrapolate to x=0
                self.r_right = round(r2 + slope * (1.0 - x2))  # Extrapolate to x=1
        
        # Need top and bottom points for Y axis
        if self.top_point and self.bottom_point:
            y1, y_servo1 = self.top_point
            y2, y_servo2 = self.bottom_point
            if abs(y2 - y1) > 0.1:  # Require reasonable separation
                slope = (y_servo2 - y_servo1) / (y2 - y1)
                self.y_top = round(y_servo1 - slope * y1)  # Extrapolate to y=0
                self.y_bottom = round(y_servo2 + slope * (1.0 - y2))  # Extrapolate to y=1
        
        # Return True if we have complete calibration
        return (self.left_point is not None and self.right_point is not None and
                self.top_point is not None and self.bottom_point is not None)


@dataclass
class GridCalibrationPoint:
    # Expected normalized position
    expected_x: float
    expected_y: float
    # Actual clicked normalized position
    actual_x: float
    actual_y: float
    # Servo positions used
    servo_r: int
    servo_y: int


class GridCalibration:
    def __init__(self, grid_rows: int = 5, grid_cols: int = 5):
        self.grid_rows = grid_rows
        self.grid_cols = grid_cols
        self.points: list[GridCalibrationPoint] = []
        self.active = False
        self.current_index = 0
        self.pending_actual: tuple[float, float] | None = None

    def reset(self):
        self.points = []
        self.active = False
        self.current_index = 0
        self.pending_actual = None

    def total_points(self) -> int:
        return self.grid_rows * self.grid_cols

    def set_pending_click(self, actual_x: float, actual_y: float):
        self.pending_actual = (
            max(0.0, min(1.0, float(actual_x))),
            max(0.0, min(1.0, float(actual_y))),
        )

    def save_current_point(self, servo_r: int, servo_y: int) -> bool:
        """Save pending click for current target point. Returns False if no pending click."""
        if self.pending_actual is None:
            return False

        expected_x, expected_y = self.get_expected_position(self.current_index)
        actual_x, actual_y = self.pending_actual
        self.add_point(expected_x, expected_y, actual_x, actual_y, servo_r, servo_y)

        self.current_index += 1
        self.pending_actual = None

        if self.current_index >= self.total_points():
            self.active = False
        return True

    def get_expected_position(self, index: int) -> tuple[float, float]:
        """Get expected normalized x,y for grid point index"""
        if index >= self.grid_rows * self.grid_cols:
            return (0.5, 0.5)
        row = index // self.grid_cols
        col = index % self.grid_cols
        x = col / (self.grid_cols - 1) if self.grid_cols > 1 else 0.5
        y = row / (self.grid_rows - 1) if self.grid_rows > 1 else 0.5
        return (x, y)

    def add_point(self, expected_x: float, expected_y: float, actual_x: float, actual_y: float, servo_r: int, servo_y: int):
        self.points.append(GridCalibrationPoint(expected_x, expected_y, actual_x, actual_y, servo_r, servo_y))

    def apply_correction(self, click_x: float, click_y: float) -> tuple[float, float]:
        """Apply bilinear interpolation to correct click position based on error map"""
        if len(self.points) < 4:
            return (click_x, click_y)

        # Find 4 nearest points for bilinear interpolation
        # Simple approach: use inverse distance weighting
        total_weight = 0.0
        correction_x = 0.0
        correction_y = 0.0

        for pt in self.points:
            dist = ((pt.expected_x - click_x)**2 + (pt.expected_y - click_y)**2)**0.5
            if dist < 0.001:  # Very close, use directly
                return (pt.actual_x, pt.actual_y)
            weight = 1.0 / (dist + 0.01)  # Add small epsilon to avoid division by zero
            total_weight += weight
            correction_x += weight * (pt.actual_x - pt.expected_x)
            correction_y += weight * (pt.actual_y - pt.expected_y)

        if total_weight > 0:
            correction_x /= total_weight
            correction_y /= total_weight
            return (click_x + correction_x, click_y + correction_y)

        return (click_x, click_y)


app = Flask(__name__)
lock = threading.Lock()

# Camera setup
picam2 = Picamera2()
config = picam2.create_video_configuration(main={"size": (1280, 720)})
picam2.configure(config)
picam2.start()

# Motion + laser setup
pwm = PCA9685(0x40, debug=False)
pwm.setPWMFreq(PWM_FREQ_HZ)
laser = LED(25)

state = {
    "current_r": int((R_MIN + R_MAX) / 2),
    "current_y": int((Y_MIN + Y_MAX) / 2),
    "laser_on": False,
    "step": 5,
    "edge_calib_mode": None,  # 'left', 'right', 'top', 'bottom', or None
}
calibration = Calibration()
grid_calibration = GridCalibration(grid_rows=5, grid_cols=5)

# Accumulate sub-deadband commands per axis
deadband_residual = {"r": 0.0, "y": 0.0}


def clamp(value: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, value))


def _apply_backlash(channel: int, target: int, delta: int, approach_from_high: bool) -> None:
    """Always approach target from same direction to reduce backlash error."""
    preload = target + delta if approach_from_high else target - delta
    if preload != target:
        pwm.setServoPulse(channel, int(preload))
        time.sleep(BACKLASH_SETTLE_SEC)
    pwm.setServoPulse(channel, int(target))


def _deadband_adjust_target(axis: str, current: int, requested: int) -> tuple[int, bool]:
    """
    Convert requested absolute target into an executable target accounting for deadband.
    Returns (target_to_execute, moved).
    """
    requested = int(requested)
    delta = requested - current
    if delta == 0:
        return current, False

    db = R_DEADBAND if axis == "r" else Y_DEADBAND

    # If request is already large enough, execute directly and clear accumulated residue.
    if abs(delta) >= db:
        deadband_residual[axis] = 0.0
        return requested, True

    # For small requests, accumulate until enough to overcome deadband.
    deadband_residual[axis] += delta
    if abs(deadband_residual[axis]) < db:
        return current, False

    # Execute an effective move in the accumulated direction.
    step = int(round(deadband_residual[axis]))
    deadband_residual[axis] = 0.0
    return current + step, True


def move_r(target: int, clamp_limits: bool = True) -> None:
    current = state["current_r"]
    requested = int(target)
    target, moved = _deadband_adjust_target("r", current, requested)
    if not moved:
        return
    if clamp_limits:
        target = clamp(target, R_MIN, R_MAX)
    delta = R_BACKLASH_DELTA_INC if target > current else R_BACKLASH_DELTA_DEC
    _apply_backlash(
        channel=R_CHANNEL,
        target=target,
        delta=delta,
        approach_from_high=R_APPROACH_FROM_HIGH,
    )
    state["current_r"] = target


def move_y(target: int, clamp_limits: bool = True) -> None:
    current = state["current_y"]
    requested = int(target)
    target, moved = _deadband_adjust_target("y", current, requested)
    if not moved:
        return
    if clamp_limits:
        target = clamp(target, Y_MIN, Y_MAX)
    delta = Y_BACKLASH_DELTA_INC if target > current else Y_BACKLASH_DELTA_DEC
    _apply_backlash(
        channel=Y_CHANNEL,
        target=target,
        delta=delta,
        approach_from_high=Y_APPROACH_FROM_HIGH,
    )
    state["current_y"] = target


def move_to(r_target: int, y_target: int, clamp_limits: bool = True) -> None:
    with lock:
        move_r(r_target, clamp_limits=clamp_limits)
        move_y(y_target, clamp_limits=clamp_limits)


def normalized_to_servo(x_norm: float, y_norm: float) -> tuple[int, int]:
    x_norm = max(0.0, min(1.0, x_norm))
    y_norm = max(0.0, min(1.0, y_norm))

    if INVERT_CLICK_R:
        x_norm = 1.0 - x_norm
    if INVERT_CLICK_Y:
        y_norm = 1.0 - y_norm

    r = round(calibration.r_left + (calibration.r_right - calibration.r_left) * x_norm)
    y = round(calibration.y_top + (calibration.y_bottom - calibration.y_top) * y_norm)
    return r, y


# Initialize to center
move_to(state["current_r"], state["current_y"])


@app.route("/")
def index():
    return render_template("index.html")


def generate_frames():
    while True:
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        ok, buffer = cv2.imencode(".jpg", frame)
        if not ok:
            continue
        jpg_bytes = buffer.tobytes()
        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg_bytes + b"\r\n"


@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/api/state")
def api_state():
    grid_state = None
    if grid_calibration.active:
        exp_x, exp_y = grid_calibration.get_expected_position(grid_calibration.current_index)
        grid_state = {
            "active": True,
            "current_index": grid_calibration.current_index,
            "total_points": grid_calibration.total_points(),
            "expected_x": exp_x,
            "expected_y": exp_y,
            "pending_click": {
                "x": grid_calibration.pending_actual[0],
                "y": grid_calibration.pending_actual[1],
            } if grid_calibration.pending_actual else None,
        }
    else:
        grid_state = {
            "active": False,
            "points_collected": len(grid_calibration.points),
            "total_points": grid_calibration.total_points(),
        }

    return jsonify(
        {
            **state,
            "calibration": {
                "r_left": calibration.r_left,
                "r_right": calibration.r_right,
                "y_top": calibration.y_top,
                "y_bottom": calibration.y_bottom,
                "left_point_set": calibration.left_point is not None,
                "right_point_set": calibration.right_point is not None,
                "top_point_set": calibration.top_point is not None,
                "bottom_point_set": calibration.bottom_point is not None,
            },
            "grid_calibration": grid_state,
        }
    )


@app.route("/api/step", methods=["POST"])
def api_step():
    payload = request.get_json(silent=True) or {}
    step = int(payload.get("step", state["step"]))
    state["step"] = clamp(step, 1, 150)
    return jsonify({"ok": True, "step": state["step"]})


@app.route("/api/jog", methods=["POST"])
def api_jog():
    payload = request.get_json(silent=True) or {}
    axis = str(payload.get("axis", "")).lower()
    direction = int(payload.get("dir", 0))
    delta = state["step"] * (1 if direction >= 0 else -1)

    with lock:
        if axis == "r":
            if INVERT_JOG_R:
                delta = -delta
            move_r(state["current_r"] + delta, clamp_limits=False)
        elif axis == "y":
            if INVERT_JOG_Y:
                delta = -delta
            move_y(state["current_y"] + delta, clamp_limits=False)
        else:
            return jsonify({"ok": False, "error": "axis must be 'r' or 'y'"}), 400

    return jsonify({"ok": True})


@app.route("/api/set_edge", methods=["POST"])
def api_set_edge():
    """Enter edge calibration mode - next click on image will set this edge point"""
    payload = request.get_json(silent=True) or {}
    edge = str(payload.get("edge", "")).lower()

    if edge not in ["left", "right", "top", "bottom"]:
        return jsonify({"ok": False, "error": "edge must be left/right/top/bottom"}), 400

    state["edge_calib_mode"] = edge
    return jsonify({"ok": True, "mode": edge, "message": f"Click on image near {edge} edge"})


@app.route("/api/click", methods=["POST"])
def api_click():
    payload = request.get_json(silent=True) or {}
    try:
        x = float(payload.get("x", 0.5))
        y = float(payload.get("y", 0.5))
    except (TypeError, ValueError):
        return jsonify({"ok": False, "error": "x and y must be numbers"}), 400

    # Handle edge calibration clicks
    if state["edge_calib_mode"]:
        mode = state["edge_calib_mode"]
        with lock:
            if mode == "left":
                calibration.left_point = (x, state["current_r"])
            elif mode == "right":
                calibration.right_point = (x, state["current_r"])
            elif mode == "top":
                calibration.top_point = (y, state["current_y"])
            elif mode == "bottom":
                calibration.bottom_point = (y, state["current_y"])
            
            # Try to extrapolate edges
            calibration.extrapolate_edges()
            state["edge_calib_mode"] = None
        
        return jsonify({
            "ok": True,
            "edge_calibration": True,
            "mode": mode,
            "extrapolated": True,
        })

    # Normal click with correction applied
    corrected_x, corrected_y = grid_calibration.apply_correction(x, y)
    r_target, y_target = normalized_to_servo(corrected_x, corrected_y)

    # Brownout-safe behavior: move one axis at a time (sequential), not simultaneously.
    with lock:
        move_r(r_target)
        time.sleep(AXIS_MOVE_DELAY_SEC)
        move_y(y_target)

    return jsonify(
        {
            "ok": True,
            "moved_axis": "r_then_y",
            "r": state["current_r"],
            "y": state["current_y"],
            "corrected": (corrected_x != x or corrected_y != y),
        }
    )


@app.route("/api/toggle_laser", methods=["POST"])
def api_toggle_laser():
    with lock:
        state["laser_on"] = not state["laser_on"]
        if state["laser_on"]:
            laser.on()
        else:
            laser.off()
    return jsonify({"ok": True, "laser_on": state["laser_on"]})


@app.route("/api/grid_calibration/start", methods=["POST"])
def api_grid_calibration_start():
    payload = request.get_json(silent=True) or {}
    rows = int(payload.get("rows", 5))
    cols = int(payload.get("cols", 5))

    with lock:
        grid_calibration.grid_rows = clamp(rows, 3, 10)
        grid_calibration.grid_cols = clamp(cols, 3, 10)
        grid_calibration.reset()
        grid_calibration.active = True
        grid_calibration.current_index = 0
        grid_calibration.pending_actual = None

        # Turn on laser for manual calibration
        state["laser_on"] = True
        laser.on()

        first_x, first_y = grid_calibration.get_expected_position(0)

    return jsonify({
        "ok": True,
        "grid_rows": grid_calibration.grid_rows,
        "grid_cols": grid_calibration.grid_cols,
        "total_points": grid_calibration.total_points(),
        "current_index": grid_calibration.current_index,
        "expected_x": first_x,
        "expected_y": first_y,
    })


@app.route("/api/grid_calibration/cancel", methods=["POST"])
def api_grid_calibration_cancel():
    with lock:
        grid_calibration.active = False
        state["laser_on"] = False
        laser.off()
    return jsonify({"ok": True})


@app.route("/api/grid_calibration/skip", methods=["POST"])
def api_grid_calibration_skip():
    """Skip current calibration point when laser is not visible"""
    if not grid_calibration.active:
        return jsonify({"ok": False, "error": "No active calibration"}), 400

    with lock:
        grid_calibration.pending_actual = None
        grid_calibration.current_index += 1

        # Check if calibration complete
        if grid_calibration.current_index >= grid_calibration.total_points():
            grid_calibration.active = False
            state["laser_on"] = False
            laser.off()
            return jsonify({"ok": True, "calibration_complete": True, "points": len(grid_calibration.points)})

        # Move to next grid position
        next_x, next_y = grid_calibration.get_expected_position(grid_calibration.current_index)

    return jsonify({
        "ok": True,
        "skipped": True,
        "current_index": grid_calibration.current_index,
        "total_points": grid_calibration.total_points(),
        "expected_x": next_x,
        "expected_y": next_y,
    })


@app.route("/api/grid_calibration/pending_click", methods=["POST"])
def api_grid_calibration_pending_click():
    """Record clicked laser location for current grid target (does not save yet)."""
    if not grid_calibration.active:
        return jsonify({"ok": False, "error": "No active calibration"}), 400

    payload = request.get_json(silent=True) or {}
    try:
        x = float(payload.get("x"))
        y = float(payload.get("y"))
    except (TypeError, ValueError):
        return jsonify({"ok": False, "error": "x and y must be numbers"}), 400

    with lock:
        grid_calibration.set_pending_click(x, y)

    return jsonify(
        {
            "ok": True,
            "pending_click": {
                "x": grid_calibration.pending_actual[0],
                "y": grid_calibration.pending_actual[1],
            },
        }
    )


@app.route("/api/grid_calibration/save_point", methods=["POST"])
def api_grid_calibration_save_point():
    """Save current target point using last pending click and current servo position."""
    if not grid_calibration.active:
        return jsonify({"ok": False, "error": "No active calibration"}), 400

    with lock:
        saved = grid_calibration.save_current_point(state["current_r"], state["current_y"])
        if not saved:
            return jsonify({"ok": False, "error": "Click on image first"}), 400

        if not grid_calibration.active:
            state["laser_on"] = False
            laser.off()
            return jsonify({"ok": True, "calibration_complete": True, "points": len(grid_calibration.points)})

        next_x, next_y = grid_calibration.get_expected_position(grid_calibration.current_index)
        return jsonify(
            {
                "ok": True,
                "saved": True,
                "current_index": grid_calibration.current_index,
                "total_points": grid_calibration.total_points(),
                "expected_x": next_x,
                "expected_y": next_y,
                "points": len(grid_calibration.points),
            }
        )


@app.route("/api/grid_calibration/clear", methods=["POST"])
def api_grid_calibration_clear():
    with lock:
        grid_calibration.points = []
    return jsonify({"ok": True, "points_cleared": True})


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
