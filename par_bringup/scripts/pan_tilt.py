#!/usr/bin/env python3
"""
PAR Pan-Tilt Camera Controller
Subscribe: /par/pan_tilt (std_msgs/String JSON)
           /tag_detections (auto follow AprilTag)
"""
import rospy
import json
import time
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetectionArray

try:
    from adafruit_servokit import ServoKit
    kit = ServoKit(channels=16)
    SERVO_OK = True
except Exception as e:
    rospy.logwarn(f"ServoKit error: {e}")
    SERVO_OK = False

# === PARAMETER ===
PAN_CH   = 0      # Channel servo pan (kiri-kanan)
TILT_CH  = 1      # Channel servo tilt (atas-bawah)

PAN_MIN  = 0
PAN_MAX  = 180
PAN_MID  = 90

TILT_MIN = 45     # Jangan terlalu bawah
TILT_MAX = 135    # Jangan terlalu atas
TILT_MID = 90

# Pan-tilt semasa
current_pan  = PAN_MID
current_tilt = TILT_MID

# Mode: 'manual' atau 'auto' (follow AprilTag)
current_mode = 'manual'

# Camera resolution untuk auto follow
CAM_W = 640
CAM_H = 480

def set_servo(channel, angle):
    angle = max(0, min(180, int(angle)))
    if SERVO_OK:
        try:
            kit.servo[channel].angle = angle
        except Exception as e:
            rospy.logwarn(f"Servo error: {e}")

def set_pan(angle):
    global current_pan
    angle = max(PAN_MIN, min(PAN_MAX, angle))
    current_pan = angle
    set_servo(PAN_CH, angle)

def set_tilt(angle):
    global current_tilt
    angle = max(TILT_MIN, min(TILT_MAX, angle))
    current_tilt = angle
    set_servo(TILT_CH, angle)

def command_cb(msg):
    """Terima command dari web dashboard"""
    global current_mode
    try:
        data = json.loads(msg.data)
        cmd  = data.get('cmd', '')

        if cmd == 'set_mode':
            current_mode = data.get('mode', 'manual')
            rospy.loginfo(f"Pan-tilt mode: {current_mode}")

        elif cmd == 'move':
            if current_mode == 'manual':
                pan  = data.get('pan',  current_pan)
                tilt = data.get('tilt', current_tilt)
                set_pan(pan)
                set_tilt(tilt)

        elif cmd == 'center':
            set_pan(PAN_MID)
            set_tilt(TILT_MID)
            rospy.loginfo("Pan-tilt centered")

        elif cmd == 'pan_left':
            set_pan(current_pan - data.get('step', 10))

        elif cmd == 'pan_right':
            set_pan(current_pan + data.get('step', 10))

        elif cmd == 'tilt_up':
            set_tilt(current_tilt + data.get('step', 10))

        elif cmd == 'tilt_down':
            set_tilt(current_tilt - data.get('step', 10))

    except Exception as e:
        rospy.logerr(f"Command error: {e}")

def tag_cb(msg):
    """Auto follow AprilTag dalam frame camera"""
    global current_mode
    if current_mode != 'auto':
        return
    if not msg.detections:
        return

    # Ambil tag yang paling dekat (z terkecil)
    best = min(msg.detections,
               key=lambda d: d.pose.pose.pose.position.z)

    x = best.pose.pose.pose.position.x
    y = best.pose.pose.pose.position.y
    z = best.pose.pose.pose.position.z

    if z <= 0:
        return

    # Convert position ke pixel
    fx = 554.256
    fy = 554.256
    cx = 320.0
    cy = 240.0

    px = (x / z) * fx + cx  # pixel x dalam frame
    py = (y / z) * fy + cy  # pixel y dalam frame

    # Error dari tengah frame
    err_x = px - CAM_W / 2   # positif = tag kat kanan
    err_y = py - CAM_H / 2   # positif = tag kat bawah

    # Dead zone
    if abs(err_x) > 30:
        # Pan: tag kat kanan → servo pan ke kanan (+)
        step_pan = err_x * 0.05
        set_pan(current_pan + step_pan)

    if abs(err_y) > 30:
        # Tilt: tag kat bawah → servo tilt ke bawah (-)
        step_tilt = err_y * 0.03
        set_tilt(current_tilt - step_tilt)

if __name__ == '__main__':
    rospy.init_node('pan_tilt_controller', anonymous=True)

    rospy.Subscriber('/par/pan_tilt', String, command_cb)
    rospy.Subscriber('/tag_detections',
                     AprilTagDetectionArray, tag_cb)

    # Center on startup
    set_pan(PAN_MID)
    set_tilt(TILT_MID)
    rospy.loginfo(f"Pan-Tilt ready | Servo OK: {SERVO_OK}")
    rospy.loginfo("Pan=CH0, Tilt=CH1 | Mode: manual")

    rospy.spin()
