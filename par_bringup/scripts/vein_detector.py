#!/usr/bin/env python3
"""
vein_detector.py -- versi ringan, AUTO-ONLY
- Buang mode manual (tak dipakai lagi)
- Proses pada resolusi lebih kecil (320x240) lalu scale naik semula
- Throttle ke ~8fps proses (bukan setiap frame kamera) utk jimat CPU
- CLAHE tileGridSize dikecilkan sikit (8->6) -- kesan visual minimum
"""
import rospy, cv2, json, time
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

PROC_W, PROC_H = 320, 240   # saiz proses dalaman -- lebih kecil = lebih ringan
OUT_W, OUT_H   = 640, 480   # saiz output ke dashboard (kekal sama)
MIN_INTERVAL   = 1.0 / 8.0  # throttle proses ke ~8fps

class VeinDetector:
    def __init__(self):
        rospy.init_node('vein_detector', anonymous=True)
        self.enabled = False
        self.channel = 'green'
        self.last_proc = 0.0
        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(6, 6))
        self.pub = rospy.Publisher('/par/vein_image/compressed', CompressedImage, queue_size=1)
        rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, self.camera_cb)
        rospy.Subscriber('/par/vein_settings', String, self.settings_cb)

    def settings_cb(self, msg):
        try:
            s = json.loads(msg.data)
            self.enabled = s.get('enabled', False)
            self.channel = s.get('channel', 'green')
        except Exception:
            pass

    def camera_cb(self, msg):
        if not self.enabled:
            return
        now = time.time()
        if now - self.last_proc < MIN_INTERVAL:
            return
        self.last_proc = now
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                return
            frame = cv2.resize(frame, (PROC_W, PROC_H), interpolation=cv2.INTER_AREA)
            result = self.process_auto(frame)
            result = cv2.resize(result, (OUT_W, OUT_H), interpolation=cv2.INTER_LINEAR)
            ok, buf = cv2.imencode('.jpg', result, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if not ok:
                return
            out_msg = CompressedImage()
            out_msg.header.stamp = rospy.Time.now()
            out_msg.format = "jpeg"
            out_msg.data = buf.tobytes()
            self.pub.publish(out_msg)
        except Exception:
            pass

    def get_channel(self, frame):
        if self.channel == 'green':
            return frame[:, :, 1]
        elif self.channel == 'red':
            return frame[:, :, 2]
        elif self.channel == 'blue':
            return frame[:, :, 0]
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def process_auto(self, frame):
        gray = self.get_channel(frame)
        cl = self.clahe.apply(gray)
        blurred = cv2.GaussianBlur(cl, (5, 5), 0)
        inverted = cv2.bitwise_not(blurred)
        thresh = cv2.adaptiveThreshold(
            inverted, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY, 15, -2)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        cleaned = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel, iterations=1)
        contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        vein_contours = [c for c in contours if cv2.arcLength(c, False) > 20 and cv2.contourArea(c) > 40]

        output = cv2.cvtColor(cl, cv2.COLOR_GRAY2BGR)
        overlay = np.full_like(output, (0, 20, 10))
        output = cv2.addWeighted(output, 0.7, overlay, 0.3, 0)

        for cnt in vein_contours:
            approx = cv2.approxPolyDP(cnt, 0.005 * cv2.arcLength(cnt, True), False)
            cv2.drawContours(output, [approx], -1, (0, 220, 120), 1, cv2.LINE_AA)

        cv2.putText(output, "VEIN DETECTION - AUTO", (8, 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 150), 1, cv2.LINE_AA)
        return output

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    VeinDetector().run()
