#!/bin/bash

echo "Installing packages..."

sudo apt update
sudo apt install -y \
  libcap-dev \
  libcamera-apps \
  python3-libcamera \
  python3-picamera2 \
  python3-opencv
