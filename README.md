# PAR — Patient Assistance Robot

**Final Year Project** — Politeknik Sultan Salahuddin Abdul Aziz Shah

An autonomous mobile IV drip stand robot that follows a patient using AprilTag-based visual tracking, with a live web dashboard for monitoring and control. Built on ROS Noetic, Raspberry Pi 4B, and Arduino-driven pan/tilt camera hardware.

## Features

- **Autonomous patient following** — AprilTag detection determines follow commands:
  - Tag 0 → STOP (idle, no active follow)
  - Tag 1 → RUN (actively follows Tag 3 at a safe distance)
  - Tag 2 → PAUSE/PRIVACY (camera feed blurs on the dashboard, e.g. patient privacy moments; resumes on Tag 1)
- **Live web dashboard** (Flask + Socket.IO)
  - Real-time MJPEG camera stream with AprilTag overlay (follows actual tag orientation/corners)
  - System monitor: CPU, RAM, temperature, IP
  - Patient info panel (name, bed — persisted across restarts)
  - Emergency alert system: visual overlay, beep sequence, and voice announcement (pre-rendered via espeak-ng)
  - Pan/tilt servo calibration panel with manual slider, step control, 9-position presets, and a freestyle auto-scan mode
- **Vein detection module** — YCrCb skin masking, CLAHE, blackhat morphology, and contour filtering, tuned for Raspberry Pi CPU constraints
- **Pan/tilt camera control** — smoothed (EMA) servo motion via PCA9685/Arduino

## Hardware

- Raspberry Pi 4B
- USB webcam (HD, used for both patient-following and vein detection — no depth camera)
- Arduino + PCA9685 servo driver (pan/tilt camera mount)
- AprilTag markers (tags 0, 1, 2, 3 — role defined above)

## Software Stack

- ROS Noetic (Ubuntu 20.04)
- Python 3 (rospy, OpenCV)
- Flask + Flask-SocketIO (web dashboard backend)
- apriltag_ros (tag detection)
- usb_cam (camera driver)
- espeak-ng (offline emergency voice synthesis)

## Repository Structure

## Setup

```bash
# Clone into your catkin workspace
cd ~/catkin_ws/src
git clone https://github.com/panz18/PAR-PATIENT_ASSISTANCE_ROBOT.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# Install Python dependencies for the dashboard
pip3 install flask flask-socketio python-socketio eventlet

# Install espeak-ng for emergency voice alerts
sudo apt install -y espeak-ng
```

## Usage

```bash
roslaunch par_bringup par_full.launch
```

Dashboard will be available at `http://<raspberry-pi-ip>:5000`.

## Author

**Panz** — Politeknik Sultan Salahuddin Abdul Aziz Shah

## License

See [LICENSE](LICENSE) file.
