# Goosinator
Code to handle:
- Servo motion
    * Current in PCA*.py
- Camera read in
     
- Laser on/off

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
