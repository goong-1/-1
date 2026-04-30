import os
from flask import Flask, render_template, Response
from flask_socketio import SocketIO
from goong_vision import VisionSystem
from robot_manager import RobotManager
import cv2
import time
import threading
import math
from datetime import datetime


# --- 서버 경로 및 Flask 설정 ---
base_dir = os.path.dirname(os.path.abspath(__file__))
app = Flask(__name__, 
            template_folder=os.path.join(base_dir, 'templates'),
            static_folder=os.path.join(base_dir, 'static'))
socketio = SocketIO(app, cors_allowed_origins="*")

# --- 시스템 초기화 ---
vision = VisionSystem(os.path.join(base_dir, "calibration.npz"))

# TODO: 웹캠 화면 기준 매트 네 꼭짓점 픽셀 좌표로 수정하기
# 순서: left_top, right_top, right_bottom, left_bottom
vision.set_homography_from_corners([
    [100, 80],    # left_top
    [860, 80],    # right_top
    [860, 460],   # right_bottom
    [100, 460],   # left_bottom
])

robot_mgr = RobotManager()

# --- 전역 상태 변수 ---
current_vision_mode = 'TRACK'
last_cmd_time = 0
is_robot1_manual = False
was_stopped = False
#USE_MAP_CONTROL = True   # 새 image-map-robot 좌표 기반 제어 사용
#USE_OLD_AUTO = False     # 기존 픽셀/거리 기반 자동추종 끄기

robot_calibrations = {0: 1.18, 1: 1.05}

# --- 카메라 설정 ---
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

last_error_angle = 0
back_turn_dir = 0

last_robot_poses = {
    0: {'heading': 0},
    1: {'heading': 0}
}

def get_follow_target_point(leader_pose, follow_dist=0.20):
    x0, y0 = leader_pose['center']
    theta0 = leader_pose['heading']

    tx = x0 - follow_dist * math.cos(theta0)
    ty = y0 - follow_dist * math.sin(theta0)

    return tx, ty

def map_to_robot_frame(target_point, robot_pose):
    """
    map 좌표계의 target_point를 robot_pose 기준 로봇 좌표계로 변환한다.

    target_point: (tx, ty), meter
    robot_pose:
        {
            'center': (x, y),
            'heading': theta_rad
        }

    return:
        local_x: 로봇 기준 앞/뒤 방향
        local_y: 로봇 기준 좌/우 방향
    """
    tx, ty = target_point
    x, y = robot_pose['center']
    theta = robot_pose['heading']

    dx = tx - x
    dy = ty - y

    local_x = math.cos(theta) * dx + math.sin(theta) * dy
    local_y = -math.sin(theta) * dx + math.cos(theta) * dy

    return local_x, local_y

def calculate_map_based_control(local_x, local_y):
    """
    robot1 좌표계 기준 target point 위치(local_x, local_y)를 PWM 명령으로 변환한다.

    local_x > 0: 목표가 로봇 앞쪽
    local_y > 0: 목표가 로봇 왼쪽
    local_y < 0: 목표가 로봇 오른쪽
    """

    dist = math.sqrt(local_x**2 + local_y**2)
    angle_error = math.degrees(math.atan2(local_y, local_x))

    # 공통 설정값은 if/else 밖에서 먼저 정의해야 함
    stop_dist = 0.15
    max_pwm = 140

    # 목표점에 충분히 가까우면 정지
    if dist < stop_dist:
        return dist, angle_error, 0, 0, "목표점 도달"

    # 목표가 로봇 뒤쪽에 있으면 전진하지 말고 제자리 회전
    if local_x < 0:
        turn_pwm = 80

        if local_y > 0:
            # 목표가 로봇 기준 왼쪽 뒤쪽
            pwm_l = -turn_pwm
            pwm_r = turn_pwm
        else:
            # 목표가 로봇 기준 오른쪽 뒤쪽
            pwm_l = turn_pwm
            pwm_r = -turn_pwm

        move_desc = "후방 목표: 제자리 회전"

    else:
        move_desc = "전진 추종"

        # 거리가 멀수록 기본 속도 증가
        if dist > 0.40:
            base_pwm = 110
        elif dist > 0.25:
            base_pwm = 95
        else:
            base_pwm = 85

        # 각도 오차에 따라 좌우 차이 생성
        turn = int(angle_error * 2.0)

        # 너무 작은 조향은 효과가 없으므로 최소 조향 보정
        if 3 < abs(angle_error) < 15:
            turn = 20 if angle_error > 0 else -20

        pwm_l = base_pwm - turn
        pwm_r = base_pwm + turn

    # 최종 PWM 제한
    pwm_l = max(min(pwm_l, max_pwm), -max_pwm)
    pwm_r = max(min(pwm_r, max_pwm), -max_pwm)

    return dist, angle_error, int(pwm_l), int(pwm_r), move_desc
    # PWM 제한
    # limit = 140
    # min_pwm = 80

    # pwm_l = v - w
    # pwm_r = v + w

    # pwm_l = max(min(pwm_l, limit), -limit)
    # pwm_r = max(min(pwm_r, limit), -limit)

    # # deadzone 보정
    # if 0 < pwm_l < min_pwm:
    #     pwm_l = min_pwm
    # elif -min_pwm < pwm_l < 0:
    #     pwm_l = -min_pwm

    # if 0 < pwm_r < min_pwm:
    #     pwm_r = min_pwm
    # elif -min_pwm < pwm_r < 0:
    #     pwm_r = -min_pwm

    # return dist, angle_error, int(pwm_l), int(pwm_r), move_desc

def send_map_based_command(pwm_l, pwm_r, dist, angle_error, move_desc):
    def sign(n):
        return f"+{n}" if n >= 0 else str(n)

    cmd = f"a{sign(pwm_l)},d{sign(pwm_r)}\n"

    print(
        f"[MAP_CTRL] {move_desc} | "
        f"cmd={cmd.strip()} | "
        f"dist={dist:.3f}m | angle={angle_error:.1f}°"
    )

    robot_mgr.send_command(1, cmd)

    socketio.emit('log', {
        'type': 'MAP_CTRL',
        'msg': f"{move_desc} | {cmd.strip()} | dist={dist:.2f}m angle={angle_error:.1f}°",
        'status': 'info'
    })

# ──────────────────────────────────────────────
# [1] 거리 / 각도 / v / w / PWM 계산
# ──────────────────────────────────────────────
def _calculate_control(target_robot, follower_robot):
    global last_error_angle

    r_pos  = follower_robot['center']
    r_head = follower_robot['heading']
    t_pos  = target_robot['center']

    dist_px = math.sqrt((t_pos[0]-r_pos[0])**2 + (t_pos[1]-r_pos[1])**2)
    dist_cm = dist_px * vision.pixel_to_cm
    target_rad = math.atan2(t_pos[1]-r_pos[1], t_pos[0]-r_pos[0])
    error_angle = (math.degrees(target_rad) - r_head + 180) % 360 - 180

    stop_threshold = 35.0
    if dist_cm <= stop_threshold:
        last_error_angle = 0
        return dist_cm, error_angle, None, None, None, None

    dist_error = dist_cm - stop_threshold
    
    # [수정] 각도가 많이 틀어져 있으면(45도 이상) 전진 속도를 줄여서 각도부터 잡게 함
    # 인위적인 제한이 아니라 dist_error에 곱해지는 계수를 유동적으로 조절
    v_weight = 2.2 if abs(error_angle) < 45 else 1.0
    v = dist_error * v_weight

    # --- 조향 w 계산 (원본 로직 유지) ---
    deadzone = 5.0
    
    # [수정] 멀리 있을 때 더 강력하게 꺾도록 base_w_gain을 거리에 비례하게 설정
    # 원래 0.8이었던 값을 거리에 따라 최대 1.5배까지 확장
    dist_factor = max(1.0, dist_cm / 100.0) 
    base_w_gain = 0.8 * dist_factor 
    
    d_gain = 0.3

    if abs(error_angle) <= deadzone:
        w = 0
        last_error_angle = 0
    else:
        if abs(error_angle) > 140:
            # 140도 이상일 때 회전력을 120으로 상향 (더 확실하게 돌게 함)
            w = -120 if error_angle > 0 else 120 
        else:
            adjusted_error = error_angle - (deadzone if error_angle > 0 else -deadzone)
            
            # 기존 dynamic_gain 로직 유지하되 상한선을 늘어난 base_w_gain에 맞춤
            dynamic_gain = base_w_gain * (abs(adjusted_error) / 90.0)
            dynamic_gain = max(0.4, min(dynamic_gain, base_w_gain))
            
            p_term = adjusted_error * dynamic_gain
            diff_angle = (error_angle - last_error_angle + 180) % 360 - 180
            d_term = diff_angle * d_gain
            
            w = (p_term + d_term)
        
        last_error_angle = error_angle

    # [수정] max_w 제한을 풀어서 시원하게 돌게 함 (limit까지 허용)
    max_w = 120 
    w = max(min(w, max_w), -max_w)

    # --- PWM 계산 (원본 로직 유지) ---
    min_pwm, limit = 80, 140
    pwm_l_raw = v + w
    pwm_r_raw = v - w

    pwm_l = max(min(pwm_l_raw, limit), -limit)
    pwm_r = max(min(pwm_r_raw, limit), -limit)

    if 0 < pwm_l < min_pwm: pwm_l = min_pwm
    elif -min_pwm < pwm_l < 0: pwm_l = -min_pwm
    if 0 < pwm_r < min_pwm: pwm_r = min_pwm
    elif -min_pwm < pwm_r < 0: pwm_r = -min_pwm

    current_cali = robot_calibrations.get(1, 1.0)
    pwm_l_final = int(pwm_l)
    pwm_r_final = max(min(int(pwm_r * current_cali), limit), -limit)

    move_dir = "직진" if abs(w) < 1 else ("좌회전" if w < 0 else "우회전")
    print(f"[교정] 각도: {error_angle:.1f}° | 명령: L {pwm_l_final}, R {pwm_r_final} | Dist: {dist_cm:.1f}")

    return dist_cm, error_angle, pwm_l_final, pwm_r_final, move_dir, current_cali

# ──────────────────────────────────────────────
# [2] 명령 전송 + 로깅
# ──────────────────────────────────────────────
def _send_auto_command(pwm_l_final, pwm_r_final, dist_cm, error_angle, move_dir, cali):
    """계산된 PWM으로 명령 문자열을 만들어 로봇1에 전송하고 로그를 emit한다."""
    global was_stopped

    def sign(n): return f"+{n}" if n >= 0 else str(n)
    final_auto_cmd = f"a{sign(pwm_l_final)},d{sign(pwm_r_final)}\n"

    print(f"[자동] {move_dir} | 계수: {cali:.2f} | 명령: {final_auto_cmd.strip()} | 거리: {dist_cm:.1f}cm | 각도: {error_angle:.1f}°")

    robot_mgr.send_command(1, final_auto_cmd)
    was_stopped = False

    socketio.emit('log', {
        'type': 'AUTO',
        'msg': f"[자동] {move_dir} | 명령: {final_auto_cmd.strip()} | 거리: {dist_cm:.1f}cm | 각도: {error_angle:.1f}°",
        'status': 'info'
    })


# ──────────────────────────────────────────────
# [3] TRACK 로직 통합 (마커 → 제어 → 전송)
# ──────────────────────────────────────────────
def _run_track_logic(markers):
    """마커 딕셔너리를 받아 자동 추적 제어 전 과정을 실행한다."""
    global was_stopped

    #----------------------------whizz------------------
    # [좌우 구성용] 마커 존재 여부 체크
    # 로봇 0 (타겟): 10(좌), 11(우) / 로봇 1 (추적기): 20(좌), 21(우)
    m_ids = markers.keys()
    check_list = {10: "로봇0 좌", 11: "로봇0 우", 20: "로봇1 좌", 21: "로봇1 우"}
    missing = [name for id, name in check_list.items() if id not in m_ids]

    if missing:
        # 빨간색 텍스트로 어떤 마커가 나갔는지 출력
        print(f"\033[91m[마커 분실] {', '.join(missing)} 인식 불가!\033[0m")
    #----------------------------whizz------------------
              
              
    target_robot   = vision.get_robot_pose(markers, 10, 11)  # 로봇 0 (타겟)
    follower_robot = vision.get_robot_pose(markers, 20, 21)  # 로봇 1 (추적기)
    
    target_map = vision.get_robot_pose_map(markers, 10, 11)
    follower_map = vision.get_robot_pose_map(markers, 20, 21)

    if target_map['detected'] and follower_map['detected']:
        print(
            f"[MAP] R0=({target_map['center'][0]:.3f}, {target_map['center'][1]:.3f}) "
            f"θ0={target_map['heading_deg']:.1f}° | "
            f"R1=({follower_map['center'][0]:.3f}, {follower_map['center'][1]:.3f}) "
            f"θ1={follower_map['heading_deg']:.1f}°"
        )
        # ✅ 로봇끼리 너무 가까우면 target 계산/제어 전에 바로 정지
        r0x, r0y = target_map['center']
        r1x, r1y = follower_map['center']
        
        robot_gap = math.sqrt((r0x - r1x)**2 + (r0y - r1y)**2)
        
        if robot_gap < 0.30:
            print(f"[SAFETY] 로봇 간 거리 너무 가까움: {robot_gap:.3f}m → 정지")
            robot_mgr.send_command(1, "a+0,d+0\n")
            return
        # ✅ 안전거리 이상일 때만 0번 로봇 뒤쪽 target point 계산
        target_point = get_follow_target_point(target_map, follow_dist=0.40)

        print(f"[TARGET] follow point=({target_point[0]:.3f}, {target_point[1]:.3f})")
        
        dx_map = target_point[0] - follower_map['center'][0]
        dy_map = target_point[1] - follower_map['center'][1]
        
        print(f"[MAP_DIFF] dx={dx_map:.3f}m, dy={dy_map:.3f}m")

        
        local_x, local_y = map_to_robot_frame(target_point, follower_map)

        dist = math.sqrt(local_x**2 + local_y**2)
        angle = math.degrees(math.atan2(local_y, local_x))

        print(
            f"[ROBOT1_FRAME] local_x={local_x:.3f}m, "
            f"local_y={local_y:.3f}m, "
            f"dist={dist:.3f}m, "
            f"angle={angle:.1f}°"
        )
        ctrl_dist, ctrl_angle, pwm_l, pwm_r, move_desc = calculate_map_based_control(local_x, local_y)

        print(
            f"[MAP_CTRL_TEST] {move_desc} | "
            f"L={pwm_l}, R={pwm_r}, "
            f"dist={ctrl_dist:.3f}m, angle={ctrl_angle:.1f}°"
        )
        
        if not is_robot1_manual:
            send_map_based_command(pwm_l, pwm_r, ctrl_dist, ctrl_angle, move_desc)
        
    
        
    if target_robot.get('mode') == 'DUAL':
        last_robot_poses[0]['heading'] = target_robot['heading']
    elif target_robot['detected']:
        target_robot['heading'] = last_robot_poses[0]['heading']

    if follower_robot.get('mode') == 'DUAL':
        last_robot_poses[1]['heading'] = follower_robot['heading']
    elif follower_robot['detected']:
        follower_robot['heading'] = last_robot_poses[1]['heading']

    
    # if target_robot['detected'] and follower_robot['detected']:
    #     if not is_robot1_manual:
    #         dist_cm, error_angle, pwm_l, pwm_r, move_dir, cali = _calculate_control(target_robot, follower_robot)
    #         socketio.emit('vision_data', {"dist": round(dist_cm, 1), "angle": round(error_angle, 1)})

    #         if pwm_l is None:
    #             # 목표 도달 → 정지
    #             if not was_stopped:
    #                 robot_mgr.send_command(1, "a+0,d+0\n")
    #                 was_stopped = True
    #                 print(f"[디버그] 정지: 목표 도달 | 거리: {dist_cm:.1f}cm | 각도: {error_angle:.1f}°")
    #                 socketio.emit('log', {'type': 'SYSTEM', 'msg': '목표 도달: 정지', 'status': 'success'})
    #         else:
    #             _send_auto_command(pwm_l, pwm_r, dist_cm, error_angle, move_dir, cali)

    else:
        # 마커 미인식 → 정지
        if not is_robot1_manual and not was_stopped:
            robot_mgr.send_command(1, "a+0,d+0\n")
            was_stopped = True
            print("[디버그] 정지: 인식 불가")


# ──────────────────────────────────────────────
# Flask 라우트 / SocketIO 이벤트
# ──────────────────────────────────────────────
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@socketio.on('change_vision_mode')
def handle_mode_change(data):
    global current_vision_mode
    current_vision_mode = data.get('mode')
    if current_vision_mode != 'TRACK':
        robot_mgr.send_command(0, 's')
        robot_mgr.send_command(1, 's')

@socketio.on('update_calibration')
def handle_calibration(data):
    global robot_calibrations
    try:
        robot_calibrations[int(data.get('id'))] = float(data.get('factor', 1.0))
    except: pass

@socketio.on('drive_control')
def handle_drive(data):
    cmd = str(data.get('command', ''))
    robot_id = int(data.get('id', 0))

    print(f"[웹수동] robot {robot_id} cmd: {cmd.strip()}")

    ok = robot_mgr.send_command(robot_id, cmd)

    socketio.emit('log', {
        'type': 'MANUAL',
        'msg': f"Robot {robot_id} CMD: {cmd.strip()} | sent={ok}",
        'status': 'success' if ok else 'danger'
    })

@socketio.on('emergency_control_robot1')
def handle_emergency_robot1(data):
    global is_robot1_manual
    key_cmd = data.get('command')

    if key_cmd == 'stop':
        is_robot1_manual = False
    else:
        is_robot1_manual = True

    base_pwm = data.get('pwm')
    base_pwm = int(base_pwm) if base_pwm is not None else 85
    cali     = robot_calibrations.get(1, 1.0)
    half_pwm = int(base_pwm / 2)

    pwm_l, pwm_r = 0, 0
    move_desc    = ""

    if key_cmd == 'up':
        pwm_l, pwm_r = base_pwm, base_pwm
        move_desc = "전진"

    elif key_cmd == 'down':
        pwm_l, pwm_r = -base_pwm, -base_pwm
        move_desc = "후진"

    elif key_cmd == 'left':
        pwm_l, pwm_r = half_pwm, base_pwm
        move_desc = "전진 좌회전"

    elif key_cmd == 'right':
        pwm_l, pwm_r = base_pwm, half_pwm
        move_desc = "전진 우회전"

    elif key_cmd == 'backLeft':
        pwm_l, pwm_r = -half_pwm, -base_pwm
        move_desc = "후진 좌회전"

    elif key_cmd == 'backRight':
        pwm_l, pwm_r = -base_pwm, -half_pwm
        move_desc = "후진 우회전"

    elif key_cmd == 'stop':
        pwm_l, pwm_r = 0, 0
        move_desc = "정지"

    pwm_l_final = int(pwm_l)
    pwm_r_final = int(pwm_r * cali)

    def sign(n): return f"+{n}" if n >= 0 else str(n)
    final_cmd = f"a{sign(pwm_l_final)},d{sign(pwm_r_final)}\n"

    if key_cmd != 'stop':
        print(f"[수동] 로봇1 {move_desc} | 계수: {cali:.2f} | 최종명령: {final_cmd.strip()}")
    else:
        print(f"[수동] 로봇1 {move_desc}")

    robot_mgr.send_command(1, final_cmd)
    socketio.emit('log', {
        'type': 'EMERGENCY',
        'msg': f"Robot 1 Override: {final_cmd.strip()} (Cali: {cali})",
        'status': 'danger'
    })


# ──────────────────────────────────────────────
# 영상 스트리밍 (프레임 루프)
# ──────────────────────────────────────────────
def gen_frames():
    global last_cmd_time

    while True:
        success, frame = cap.read()
        if not success: break

        frame, markers = vision.process_frame(frame)

        if current_vision_mode == 'TRACK':
            now = time.time()
            if now - last_cmd_time > 0.05:
                _run_track_logic(markers)
                last_cmd_time = now

        ret, buffer = cv2.imencode('.jpg', frame)
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')


def _input_listener():
    """엔터 입력 시 터미널에 타임스탬프 마킹 출력"""
    while True:
        input()
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        print("\033[91m\033[1m")
        print("=" * 60)
        print(f"★★★  이상 동작 마킹 [{ts}]  ★★★")
        print("=" * 60)
        print("\033[0m")


if __name__ == '__main__':
    threading.Thread(target=robot_mgr.start_server, args=(socketio,), daemon=True).start()
    threading.Thread(target=_input_listener, daemon=True).start()
    socketio.run(app, host='0.0.0.0', port=5001, debug=False)
