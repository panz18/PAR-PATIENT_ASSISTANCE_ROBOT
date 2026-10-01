#!/usr/bin/env python3
"""
PAR Robot Controller -- v3 (demo mode)
Follow MANA-MANA AprilTag yang detect (tak kira ID).
Jarak stop: <= STOP_DIST -> berhenti (terlalu dekat)
Jarak lebih dari tu -> robot bergerak follow tag tersebut.
"""
import rospy
import json
from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import String

# === PARAMETER ===
STOP_DIST       = 0.07  # 7cm -- jarak stop (meter)
DEAD_ZONE       = 0.08  # Centroid dead zone (meter) -- kurang dari ni = lurus
MAX_LINEAR      = 0.25  # Kelajuan maju maksimum (m/s)
MAX_ANGULAR     = 0.5   # Kelajuan pusing maksimum (rad/s)
MISSING_TIMEOUT = 3.0   # Saat sebelum robot stop bila tag hilang


class PARController:
    def __init__(self):
        rospy.init_node('par_controller', anonymous=True)

        self.cmd_pub    = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.status_pub = rospy.Publisher('/par/status', String, queue_size=1)

        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_cb)

        self.last_seen = rospy.Time.now()
        self.state     = 'STOP'

        rospy.Timer(rospy.Duration(0.5), self.safety_check)

        rospy.loginfo("PAR Controller (demo) ready -- follow mana-mana tag")
        self.publish_status("STOP", "Tiada tag dikesan")

    def tag_cb(self, msg):
        detections = msg.detections
        if not detections:
            return

        # Cari tag PALING DEKAT (z terkecil), tak kira ID
        target = min(
            (d for d in detections if d.pose.pose.pose.position.z > 0),
            key=lambda d: d.pose.pose.pose.position.z,
            default=None)

        if target is None:
            return

        self.last_seen = rospy.Time.now()

        x = target.pose.pose.pose.position.x
        z = target.pose.pose.pose.position.z
        tag_id = target.id[0]

        if z <= STOP_DIST:
            self.stop_robot()
            if self.state != 'STOP':
                self.state = 'STOP'
                self.publish_status('STOP', f'Tag {tag_id} terlalu dekat ({z*100:.1f}cm)')
        else:
            self.move_robot(x, z)
            if self.state != 'FOLLOW':
                self.state = 'FOLLOW'
            self.publish_status('FOLLOW', f'Follow Tag {tag_id} ({z*100:.1f}cm)')

    def move_robot(self, x, z):
        twist = Twist()

        dist_error = z - STOP_DIST
        linear = max(0.0, min(MAX_LINEAR, dist_error * 0.3))

        if abs(x) > DEAD_ZONE:
            angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, -x * 0.8))
        else:
            angular = 0.0

        twist.linear.x  = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

        rospy.logdebug(f"Follow x:{x:.2f} z:{z:.2f} lin:{linear:.2f} ang:{angular:.2f}")

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def safety_check(self, event):
        elapsed = (rospy.Time.now() - self.last_seen).to_sec()
        if elapsed > MISSING_TIMEOUT and self.state != 'STOP':
            self.stop_robot()
            self.state = 'STOP'
            self.publish_status('STOP', 'Tiada tag dikesan')

    def publish_status(self, state, message):
        self.status_pub.publish(json.dumps({'state': state, 'message': message}))

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        PARController().run()
    except rospy.ROSInterruptException:
        pass
