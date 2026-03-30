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

# NOTES
- Make sure to reset `limits.txt` LIMITS any time the laser changes!
I changed from pointing directly out of the assembly to at more of a right angle.  This messed up the limits (but helped the laser be able to point down).

## Camera calibration
- Need to do the Aruco Board camera thing

## Servo backlash and deadband
- This hardware definitely seems to have deadbands (small commands do nothing, then it jumps).
- There is some backlash correction, but it's static for now.
