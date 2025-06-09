#!/bin/bash
set -e

echo "Updating package list..."
sudo apt update

echo "Installing Python 3 and pip..."
sudo apt install -y python3 python3-pip python3-venv

echo "Installing GStreamer core and plugins..."
sudo apt install -y \
  python3-gi \
  gir1.2-gst-plugins-base-1.0 \
  gir1.2-gstreamer-1.0 \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  gstreamer1.0-x \
  libgirepository1.0-dev \
  fonts-dejavu-core \
  x11-utils xinit

echo "Testing GStreamer availability..."
gst-launch-1.0 --version

echo "All dependencies installed!"
echo "You can now run your ground station with: python3 your_script.py"
