#!/usr/bin/env python3

import threading
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
Y_MAX = 545

# Backlash compensation (intentional): move high first, then down to target
Y_BACKLASH_OVERSHOOT = 750

# UI/Camera direction tuning (set True when physical motion is reversed)
INVERT_R = True
INVERT_Y = True


@dataclass
class Calibration:
    # Image-space edges mapped to servo pulses
    # Left image edge -> r_left; Right image edge -> r_right
    # Top image edge -> y_top; Bottom image edge -> y_bottom
    r_left: int = R_MAX
    r_right: int = R_MIN
    y_top: int = Y_MAX
    y_bottom: int = Y_MIN


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

    if INVERT_R:
        x_norm = 1.0 - x_norm
    if INVERT_Y:
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
    return jsonify(
        {
            **state,
            "calibration": {
                "r_left": calibration.r_left,
                "r_right": calibration.r_right,
                "y_top": calibration.y_top,
                "y_bottom": calibration.y_bottom,
            },
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
            if INVERT_R:
                delta = -delta
            move_r(state["current_r"] + delta, clamp_limits=False)
        elif axis == "y":
            if INVERT_Y:
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

    r_target, y_target = normalized_to_servo(x, y)
    move_to(r_target, y_target)
    return jsonify({"ok": True, "r": state["current_r"], "y": state["current_y"]})


@app.route("/api/toggle_laser", methods=["POST"])
def api_toggle_laser():
    with lock:
        state["laser_on"] = not state["laser_on"]
        if state["laser_on"]:
            laser.on()
        else:
            laser.off()
    return jsonify({"ok": True, "laser_on": state["laser_on"]})


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
