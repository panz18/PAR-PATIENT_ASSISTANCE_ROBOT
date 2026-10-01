#!/usr/bin/env python3
import rospy, threading, time, os, json, signal, random, subprocess, re
from flask import Flask, render_template, Response, request, jsonify
from flask_socketio import SocketIO
from sensor_msgs.msg import CompressedImage, CameraInfo
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import String, Bool

signal.signal(signal.SIGTERM, signal.SIG_DFL)

BASE = os.path.expanduser('~/par_dashboard')
PATIENT_FILE = os.path.join(BASE, 'patient_info.json')
VITALS_LOG_FILE = os.path.join(BASE, 'vitals_log.json')
STATIC_DIR   = os.path.join(BASE, 'static')
EMG_AUDIO    = os.path.join(STATIC_DIR, 'emergency.wav')
os.makedirs(STATIC_DIR, exist_ok=True)

app = Flask(__name__,
            template_folder=os.path.join(BASE, 'templates'),
            static_folder=STATIC_DIR)
socketio = SocketIO(app, cors_allowed_origins="*",
                     async_mode='threading',
                     logger=False, engineio_logger=False)

latest_frame, latest_vein_frame = None, None
lock, vein_lock = threading.Lock(), threading.Lock()
frame_event, vein_frame_event = threading.Event(), threading.Event()

cpu_prev     = [0, 0]
cam_info     = {'fx':554.256,'fy':554.256,'cx':320.5,'cy':240.5,
                'width':640,'height':480}
current_mode = 'patient'
vein_pub_ref, pan_tilt_pub_ref, test_cmd_pub_ref = [None], [None], [None]
vein_proc_ref = [None]

pt_state   = {'pan':90,'tilt':135,'mode':'manual'}
pt_float   = {'pan':90.0,'tilt':135.0}
pt_current = {'pan':90,'tilt':135}
pt_target  = {'pan':90,'tilt':135}

latest_vitals   = {'hr': 0, 'spo2': 0}
last_completed  = None
latest_robot_status = {'state': 'STOP', 'message': 'Menunggu arahan GO...'}

SMART_WAYPOINTS = [
    (90,135),(0,135),(180,135),
    (90,90), (0,90), (180,90),
    (90,45), (0,45), (180,45)]

def load_patient():
    try:
        with open(PATIENT_FILE) as f: return json.load(f)
    except: return {'name':'Unnamed Patient','bed':'-'}

def save_patient(info):
    try:
        with open(PATIENT_FILE, 'w') as f: json.dump(info, f)
    except: pass

patient_info = load_patient()

def load_vitals_log():
    try:
        with open(VITALS_LOG_FILE) as f: return json.load(f)
    except: return []

def save_vitals_log(log):
    try:
        with open(VITALS_LOG_FILE, 'w') as f: json.dump(log[-200:], f)
    except: pass

vitals_log = load_vitals_log()

def to_speech_friendly(text):
    if not text or text == '-':
        return ''
    text = re.sub(r'([A-Za-z])(\d)', r'\1 \2', text)
    text = re.sub(r'(\d)([A-Za-z])', r'\1 \2', text)
    text = text.replace('-', ' ')
    return text

def generate_emergency_audio():
    name = to_speech_friendly(patient_info.get('name', 'Unnamed Patient')) or 'Unnamed Patient'
    bed  = to_speech_friendly(patient_info.get('bed', '-'))
    bed_text = f", bed {bed}," if bed else ","
    msg = f"Emergency alert. Patient {name}{bed_text} requires immediate assistance. Please respond now."
    try:
        subprocess.run(
            ['espeak-ng', '-v', 'en+f3', '-s', '150', msg, '-w', EMG_AUDIO],
            check=True, timeout=15)
        print(f"[TTS] emergency.wav regenerated for: {name}")
    except Exception as e:
        print("[TTS] generation failed:", e)

generate_emergency_audio()

def get_cpu():
    try:
        with open('/proc/stat') as f: ln = f.readline().split()
        idle, total = int(ln[4]), sum(int(x) for x in ln[1:])
        if cpu_prev[1] == 0:
            cpu_prev[0], cpu_prev[1] = idle, total
            return 0.0
        di, dt = idle-cpu_prev[0], total-cpu_prev[1]
        cpu_prev[0], cpu_prev[1] = idle, total
        return round((1-di/dt)*100, 1) if dt else 0.0
    except: return 0.0

def get_ram():
    try:
        info = {}
        with open('/proc/meminfo') as f:
            for line in f:
                k, v = line.split(':')
                info[k.strip()] = int(v.split()[0])
        used = info['MemTotal'] - info['MemAvailable']
        return round(used/info['MemTotal']*100, 1)
    except: return 0.0

def get_temp():
    try:
        return round(int(open(
            "/sys/class/thermal/thermal_zone0/temp").read())/1000.0, 1)
    except: return 0.0

def get_ip():
    try:
        for iface in ['wlan0', 'eth0']:
            r = subprocess.check_output(
                f"ip addr show {iface} 2>/dev/null | grep 'inet ' | "
                f"awk '{{print $2}}' | cut -d/ -f1",
                shell=True).decode().strip()
            if r: return r
        return "N/A"
    except: return "N/A"

def camera_cb(msg):
    global latest_frame
    with lock: latest_frame = bytes(msg.data)
    frame_event.set()

def vein_camera_cb(msg):
    global latest_vein_frame
    with vein_lock: latest_vein_frame = bytes(msg.data)
    vein_frame_event.set()

def camera_info_cb(msg):
    if msg.K[0] > 0:
        cam_info.update({'fx':msg.K[0],'fy':msg.K[4],
                          'cx':msg.K[2],'cy':msg.K[5],
                          'width':msg.width,'height':msg.height})

def quat_to_rot(qx, qy, qz, qw):
    return [
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)]
    ]

def project(X, Y, Z, fx, fy, cx, cy):
    if Z <= 0: return None
    return [round((X/Z)*fx+cx, 1), round((Y/Z)*fy+cy, 1)]

def tag_cb(msg):
    tags = []
    fx, fy, cx_, cy_ = cam_info['fx'], cam_info['fy'], cam_info['cx'], cam_info['cy']
    for det in msg.detections:
        pos = det.pose.pose.pose.position
        ori = det.pose.pose.pose.orientation
        x, y, z = pos.x, pos.y, pos.z
        if z <= 0: continue

        center = project(x, y, z, fx, fy, cx_, cy_)
        if not center: continue
        px, py = center

        s = (det.size[0] if det.size else 0.05) / 2.0
        R = quat_to_rot(ori.x, ori.y, ori.z, ori.w)
        local = [(-s,-s,0), (s,-s,0), (s,s,0), (-s,s,0)]
        corners = []
        for (lx, ly, lz) in local:
            X = R[0][0]*lx + R[0][1]*ly + R[0][2]*lz + x
            Y = R[1][0]*lx + R[1][1]*ly + R[1][2]*lz + y
            Z = R[2][0]*lx + R[2][1]*ly + R[2][2]*lz + z
            pt = project(X, Y, Z, fx, fy, cx_, cy_)
            if pt: corners.append(pt)

        dist_cm = round(z*100, 1)
        if abs(x) < 0.05: direction, arrow = "STRAIGHT", "up"
        elif x < 0:       direction, arrow = "LEFT", "left"
        else:             direction, arrow = "RIGHT", "right"
        dist_status = ("TOO CLOSE" if dist_cm < 30
                       else "FAR" if dist_cm > 150 else "SAFE")

        tags.append({'id':det.id[0], 'px':px, 'py':py,
                     'corners': corners if len(corners) == 4 else None,
                     'x':round(x,3),'y':round(y,3),'z':round(z,3),
                     'dist_cm':dist_cm,'dist_status':dist_status,
                     'direction':direction,'arrow':arrow})
    socketio.emit('tags', {'detections':tags,
                            'cam_w':cam_info['width'],
                            'cam_h':cam_info['height']})

def emergency_cb(msg):
    if msg.data:
        socketio.emit('emergency_alert', {
            'active':True, 'patient':patient_info['name'],
            'bed':patient_info['bed'],
            'message':'Emergency button pressed!'})
    else:
        socketio.emit('emergency_alert', {'active':False})

def vitals_cb(msg):
    global latest_vitals, last_completed
    try:
        d = json.loads(msg.data)
    except (ValueError, TypeError):
        return
    hr        = d.get('hr', 0)
    spo2      = d.get('spo2', 0)
    phase     = d.get('phase', 'idle')
    remaining = d.get('remaining', 0)
    latest_vitals = {'hr': hr, 'spo2': spo2}
    if phase == 'done':
        last_completed = {'hr': hr, 'spo2': spo2}
    alert = bool(hr and (hr > 100 or hr < 50)) or bool(spo2 and spo2 < 95)
    socketio.emit('vitals', {
        'hr': hr,
        'spo2': spo2,
        'wave': d.get('wave', 0),
        'source': d.get('source', 'sim'),
        'alert': alert,
        'phase': phase,
        'remaining': remaining,
        'reason': d.get('reason', '')
    })

def robot_status_cb(msg):
    global latest_robot_status
    try:
        d = json.loads(msg.data)
    except (ValueError, TypeError):
        return
    latest_robot_status = {'state': d.get('state', 'STOP'),
                            'message': d.get('message', '')}
    socketio.emit('robot_status', latest_robot_status)

def generate_cam():
    while True:
        frame_event.wait(timeout=0.5)
        frame_event.clear()
        with lock: frame = latest_frame
        if frame:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                   + frame + b'\r\n')

def generate_vein():
    while True:
        vein_frame_event.wait(timeout=0.5)
        vein_frame_event.clear()
        with vein_lock: frame = latest_vein_frame
        if frame:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                   + frame + b'\r\n')

def broadcast():
    while True:
        try:
            socketio.emit('stats', {
                'temp':get_temp(), 'cpu':get_cpu(),
                'ram':get_ram(), 'ip':get_ip(),
                'time':time.strftime('%H:%M:%S'),
                'mode':current_mode})
            socketio.emit('pt_state', pt_state)
        except: pass
        time.sleep(3.0)

def pt_smoother():
    rate = rospy.Rate(30)
    freestyle_pause = 0
    ALPHA = 0.04

    while not rospy.is_shutdown():
        if pt_state.get('mode') == 'freestyle':
            if (abs(pt_float['pan']-pt_target['pan']) < 0.5 and
                    abs(pt_float['tilt']-pt_target['tilt']) < 0.5):
                if freestyle_pause > 0:
                    freestyle_pause -= 1
                else:
                    wp = random.choice(SMART_WAYPOINTS)
                    pt_target['pan'], pt_target['tilt'] = wp
                    pt_state['pan'], pt_state['tilt'] = wp
                    try: socketio.emit('pt_state', pt_state)
                    except: pass
                    freestyle_pause = 90

        changed = False
        for axis in ['pan', 'tilt']:
            diff = pt_target[axis] - pt_float[axis]
            if abs(diff) > 0.1:
                pt_float[axis] += diff * ALPHA
                changed = True

        if changed and pan_tilt_pub_ref[0]:
            cp, ct = int(round(pt_float['pan'])), int(round(pt_float['tilt']))
            if cp != pt_current['pan'] or ct != pt_current['tilt']:
                pt_current['pan'], pt_current['tilt'] = cp, ct
                pan_tilt_pub_ref[0].publish(json.dumps(
                    {'cmd':'move','pan':cp,'tilt':ct}))
        try: rate.sleep()
        except: break

def start_vein_process():
    if vein_proc_ref[0] is None or vein_proc_ref[0].poll() is not None:
        try:
            vein_proc_ref[0] = subprocess.Popen(
                ['rosrun', 'par_bringup', 'vein_detector.py',
                 '__name:=vein_detector'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            rospy.loginfo("[dashboard] vein_detector.py started (on-demand)")
        except Exception as e:
            rospy.logerr(f"[dashboard] failed to start vein_detector: {e}")

def stop_vein_process():
    p = vein_proc_ref[0]
    if p and p.poll() is None:
        try:
            p.terminate()
            p.wait(timeout=3)
        except Exception:
            try: p.kill()
            except Exception: pass
        rospy.loginfo("[dashboard] vein_detector.py stopped (on-demand)")
    vein_proc_ref[0] = None

@app.route('/')
def index(): return render_template('index.html')

@app.route('/stream')
def stream():
    return Response(generate_cam(),
                     mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/vein_stream')
def vein_stream():
    return Response(generate_vein(),
                     mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/stats')
def api_stats():
    return jsonify({'temp':get_temp(),'cpu':get_cpu(),
                     'ram':get_ram(),'ip':get_ip(),
                     'time':time.strftime('%H:%M:%S')})

@app.route('/api/pan_tilt', methods=['POST'])
def pan_tilt_cmd():
    data = request.json or {}
    cmd = data.get('cmd')
    if cmd == 'move':
        pt_state['mode'] = 'manual'
        pt_target['pan']  = int(data.get('pan', pt_target['pan']))
        pt_target['tilt'] = int(data.get('tilt', pt_target['tilt']))
        pt_state['pan'], pt_state['tilt'] = pt_target['pan'], pt_target['tilt']
    elif cmd == 'center':
        pt_state['mode'] = 'manual'
        pt_target['pan'], pt_target['tilt'] = 90, 135
        pt_state['pan'], pt_state['tilt'] = 90, 135
    elif cmd == 'set_mode':
        pt_state['mode'] = data.get('mode', 'manual')
        if pt_state['mode'] == 'manual':
            pt_target['pan']  = pt_current['pan']
            pt_target['tilt'] = pt_current['tilt']
    socketio.emit('pt_state', pt_state)
    return jsonify({'status':'ok', 'state':pt_state})

@app.route('/api/set_patient', methods=['POST'])
def set_patient():
    data = request.json or {}
    patient_info['name'] = (data.get('name') or 'Unnamed Patient').strip()
    patient_info['bed']  = (data.get('bed') or '-').strip()
    save_patient(patient_info)
    generate_emergency_audio()
    socketio.emit('patient_updated', patient_info)
    return jsonify({'status':'ok', 'patient':patient_info})

@app.route('/api/robot_status')
def get_robot_status():
    return jsonify(latest_robot_status)

@app.route('/api/get_patient')
def get_patient(): return jsonify(patient_info)

@app.route('/api/vitals_log', methods=['GET'])
def get_vitals_log():
    return jsonify(vitals_log)

@app.route('/api/vitals_log', methods=['POST'])
def record_vitals_log():
    global last_completed
    if not last_completed:
        return jsonify({'status': 'error',
                         'message': 'Belum ada test selesai. Tekan Start Test dahulu.'}), 400
    entry = {
        'time': time.strftime('%Y-%m-%d %H:%M:%S'),
        'hr': last_completed.get('hr', 0),
        'spo2': last_completed.get('spo2', 0),
        'patient': patient_info.get('name', '-'),
        'bed': patient_info.get('bed', '-')
    }
    vitals_log.append(entry)
    save_vitals_log(vitals_log)
    last_completed = None
    return jsonify({'status': 'ok', 'entry': entry, 'log': vitals_log[-10:]})

@app.route('/api/start_vitals_test', methods=['POST'])
def start_vitals_test():
    if test_cmd_pub_ref[0]:
        test_cmd_pub_ref[0].publish('start')
    return jsonify({'status': 'ok'})

@app.route('/api/set_mode', methods=['POST'])
def set_mode():
    global current_mode
    data = request.json or {}
    new_mode = data.get('mode', 'patient')

    if new_mode == 'vein' and current_mode != 'vein':
        start_vein_process()
        time.sleep(1.0)
    elif new_mode != 'vein' and current_mode == 'vein':
        stop_vein_process()

    current_mode = new_mode
    if vein_pub_ref[0]:
        vein_pub_ref[0].publish(json.dumps({
            'enabled': current_mode == 'vein',
            'mode': data.get('vein_mode', 'auto')}))
    return jsonify({'status':'ok', 'mode':current_mode})

@app.route('/api/vein_settings', methods=['POST'])
def vein_settings():
    data = request.json or {}
    if current_mode == 'vein' and vein_pub_ref[0]:
        data['enabled'] = True
        vein_pub_ref[0].publish(json.dumps(data))
    return jsonify({'status':'ok'})

if __name__ == '__main__':
    threading.Thread(target=lambda: socketio.run(
        app, host='0.0.0.0', port=5000, log_output=False),
        daemon=True).start()
    threading.Thread(target=broadcast, daemon=True).start()

    rospy.init_node('par_dashboard', anonymous=True, disable_signals=True)
    rospy.on_shutdown(stop_vein_process)
    threading.Thread(target=pt_smoother, daemon=True).start()

    rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, camera_cb)
    rospy.Subscriber('/par/vein_image/compressed', CompressedImage, vein_camera_cb)
    rospy.Subscriber('/usb_cam/camera_info', CameraInfo, camera_info_cb)
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_cb)
    rospy.Subscriber('/emergency_button', Bool, emergency_cb)
    rospy.Subscriber('/par/vitals', String, vitals_cb)
    rospy.Subscriber('/par/status', String, robot_status_cb)
    vein_pub_ref[0]     = rospy.Publisher('/par/vein_settings', String, queue_size=1)
    pan_tilt_pub_ref[0] = rospy.Publisher('/par/pan_tilt', String, queue_size=1)
    test_cmd_pub_ref[0] = rospy.Publisher('/par/vitals_cmd', String, queue_size=1)

    time.sleep(1.0)
    if pan_tilt_pub_ref[0]:
        pan_tilt_pub_ref[0].publish(json.dumps({'cmd':'move','pan':90,'tilt':135}))

    print(f"\n{'='*44}\n  PAR Dashboard  ->  http://{get_ip()}:5000\n{'='*44}\n")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try: rate.sleep()
        except: break
