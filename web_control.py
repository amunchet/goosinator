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
R_MIN = 840
R_MAX = 1099
Y_MIN = 375
Y_MAX = 700

# Backlash compensation (intentional): move high first, then down to target
Y_BACKLASH_OVERSHOOT = 750
AXIS_MOVE_DELAY_SEC = 0.08

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

    def reset(self):
        self.points = []
        self.active = False
        self.current_index = 0

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
}
calibration = Calibration()
grid_calibration = GridCalibration(grid_rows=5, grid_cols=5)


def clamp(value: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, value))


def move_r(target: int, clamp_limits: bool = True) -> None:
    target = int(target)
    if clamp_limits:
        target = clamp(target, R_MIN, R_MAX)
    pwm.setServoPulse(R_CHANNEL, target)
    state["current_r"] = target


def move_y(target: int, clamp_limits: bool = True) -> None:
    target = int(target)
    if clamp_limits:
        target = clamp(target, Y_MIN, Y_MAX)
    pwm.setServoPulse(Y_CHANNEL, Y_BACKLASH_OVERSHOOT)
    pwm.setServoPulse(Y_CHANNEL, target)
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
            "total_points": grid_calibration.grid_rows * grid_calibration.grid_cols,
            "expected_x": exp_x,
            "expected_y": exp_y,
        }
    else:
        grid_state = {
            "active": False,
            "points_collected": len(grid_calibration.points),
        }

    return jsonify(
        {
            **state,
            "calibration": {
                "r_left": calibration.r_left,
                "r_right": calibration.r_right,
                "y_top": calibration.y_top,
                "y_bottom": calibration.y_bottom,
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
    payload = request.get_json(silent=True) or {}
    edge = str(payload.get("edge", "")).lower()

    with lock:
        if edge == "left":
            calibration.r_left = state["current_r"]
        elif edge == "right":
            calibration.r_right = state["current_r"]
        elif edge == "top":
            calibration.y_top = state["current_y"]
        elif edge == "bottom":
            calibration.y_bottom = state["current_y"]
        else:
            return jsonify({"ok": False, "error": "edge must be left/right/top/bottom"}), 400

    return jsonify({"ok": True})


@app.route("/api/click", methods=["POST"])
def api_click():
    payload = request.get_json(silent=True) or {}
    try:
        x = float(payload.get("x", 0.5))
        y = float(payload.get("y", 0.5))
    except (TypeError, ValueError):
        return jsonify({"ok": False, "error": "x and y must be numbers"}), 400

    # Handle grid calibration clicks
    if grid_calibration.active:
        expected_x, expected_y = grid_calibration.get_expected_position(grid_calibration.current_index)
        grid_calibration.add_point(
            expected_x, expected_y,
            x, y,
            state["current_r"], state["current_y"]
        )
        grid_calibration.current_index += 1

        # Check if calibration complete
        if grid_calibration.current_index >= grid_calibration.grid_rows * grid_calibration.grid_cols:
            grid_calibration.active = False
            return jsonify({"ok": True, "calibration_complete": True, "points": len(grid_calibration.points)})

        # Move to next grid position
        next_x, next_y = grid_calibration.get_expected_position(grid_calibration.current_index)
        r_next, y_next = normalized_to_servo(next_x, next_y)
        with lock:
            move_r(r_next)
            time.sleep(AXIS_MOVE_DELAY_SEC)
            move_y(y_next)

        return jsonify({
            "ok": True,
            "calibration_active": True,
            "current_index": grid_calibration.current_index,
            "expected_x": next_x,
            "expected_y": next_y,
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

        # Turn on laser and move to first position
        state["laser_on"] = True
        laser.on()

        first_x, first_y = grid_calibration.get_expected_position(0)
        r_first, y_first = normalized_to_servo(first_x, first_y)
        move_r(r_first)
        time.sleep(AXIS_MOVE_DELAY_SEC)
        move_y(y_first)

    return jsonify({
        "ok": True,
        "grid_rows": grid_calibration.grid_rows,
        "grid_cols": grid_calibration.grid_cols,
        "total_points": grid_calibration.grid_rows * grid_calibration.grid_cols,
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


@app.route("/api/grid_calibration/clear", methods=["POST"])
def api_grid_calibration_clear():
    with lock:
        grid_calibration.points = []
    return jsonify({"ok": True, "points_cleared": True})


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
