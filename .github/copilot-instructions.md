# Copilot instructions for goosinator

## Project purpose and architecture
- This is a Raspberry Pi hardware-control project for a laser pointer + 2-axis servo rig + camera stream.
- Core modules are intentionally simple scripts, not a package:
  - [PCA9685.py](PCA9685.py): I2C PWM driver wrapper + movement workflows (`cli()`, `calibration()`).
  - [laser_on.py](laser_on.py): toggles laser GPIO pin (`LED(25)`) briefly.
  - [stream_camera.py](stream_camera.py): Flask MJPEG server using Picamera2 + OpenCV.
  - [limits.txt](limits.txt): documented servo limits/backlash notes (currently manual, not enforced in code).

## Hardware and channel conventions (critical)
- PCA9685 channel mapping in current code:
  - `channel 4` = rotation axis (`R`)
  - `channel 0` = vertical axis (`Y`)
- Servo pulses are expressed as microseconds via `setServoPulse()`; PWM frequency is fixed to `50Hz`.
- Current known movement bounds from [limits.txt](limits.txt):
  - `R`: 840 (right-most) to 1099 (left-most)
  - `Y`: 375 (lowest) to 545 (highest)
- Y-axis backlash compensation pattern is real and intentional: move to a larger value first (e.g., `750`) then back down to target, as seen in `calibration()`.

## Runtime workflows
- Install system dependencies with [install_packages.sh](install_packages.sh). These apt packages are required for camera and GPIO integration.
- Python deps are in [requirements.txt](requirements.txt), but camera stack (`python3-picamera2`, `python3-libcamera`) comes from apt.
- Important environment note from [README.md](README.md): camera interaction may need system Python on Pi (not a venv).

## Typical commands
- Servo calibration loop (default entrypoint in [PCA9685.py](PCA9685.py)): run the script directly.
- Laser pulse test: run [laser_on.py](laser_on.py).
- Camera stream server: run [stream_camera.py](stream_camera.py), then open `/` and `/video_feed` (default port `5000`).

## Editing guidance for AI agents
- Preserve direct hardware behavior unless explicitly asked; this repo controls physical devices.
- If adding motion code, keep axis/channel constants explicit and centralized.
- If implementing limits, use [limits.txt](limits.txt) as source-of-truth and apply clamping before `setServoPulse()` calls.
- Avoid introducing heavy architecture (framework rewrites, package restructuring) unless requested; current design favors quick field testing.
- Keep imports Pi-compatible (`gpiozero`, `smbus`/`smbus2`, `picamera2`) and avoid desktop-only assumptions.

## Gaps to be aware of
- No automated tests or lint pipeline are currently defined.
- `laser_status` exists in [PCA9685.py](PCA9685.py) but is not functionally used.
- There is no safety interlock layer; movement and laser actions execute immediately.
