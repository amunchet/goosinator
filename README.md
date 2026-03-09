# Goosinator
Code to handle:
- Servo motion
    * Current in PCA*.py
- Camera read in
     
- Laser on/off

## Web control UI (point-and-click)
- Run `python3 web_control.py` on the Pi.
- Open `http://<pi-ip>:5000`.
- Features:
  - Live camera stream.
  - Click image to move servos to that point (after edge calibration).
  - Laser toggle button (on/off hold).
  - Calibration jog buttons with configurable step size.
  - Set `Left/Right/Top/Bottom` edges from current servo position.

## Required apt installs
- libcap-dev

```
sudo apt update
sudo apt install -y \
  libcamera-apps \
  python3-libcamera \
  python3-picamera2
```

Seems as though you need to run python outside of VENV for interaction with special Pi5 packages
