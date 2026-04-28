from flask import Flask, render_template, jsonify, request, Response
from flask_socketio import SocketIO
import pymysql
import threading
import socket 
import cv2
import time

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# --- [전역 상태 변수] ---
PC_IP = "0.0.0.0"       # 모든 인터페이스에서 접속 허용
PC_PORT = 8080         # 로봇(ESP32)이 찾아올 포트
esp_sock = None        # 로봇과 연결된 소켓 객체
auto_stop_enabled = True

# MariaDB 접속 정보
DB_HOST = '192.168.0.136'
DB_USER = 'vboxmaster'
DB_PASSWORD = '1234'
DB_NAME = 'tp_and_joy'

def get_db_connection():
    return pymysql.connect(host=DB_HOST, user=DB_USER, password=DB_PASSWORD, database=DB_NAME, charset='utf8mb4', cursorclass=pymysql.cursors.DictCursor)

# --- [1] TCP 서버 시작 (로봇의 접속을 기다림) ---
def start_robot_server():
    global esp_sock
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.bind((PC_IP, PC_PORT))
    server_sock.listen(5)
    print(f"🚀 관제 본부 가동: {PC_PORT}번 포트에서 로봇의 접속을 기다립니다...")

    while True:
        conn, addr = server_sock.accept()
        print(f"🤖 로봇 접속 성공! 주소: {addr}")
        esp_sock = conn
        # 로봇으로부터 데이터(엔코더 등) 수신 시작
        threading.Thread(target=receive_robot_data, args=(conn,), daemon=True).start()

def receive_robot_data(conn):
    global esp_sock
    buffer = ""
    while True:
        try:
            data = conn.recv(1024).decode('utf-8')
            if not data: break
            buffer += data
            if '\n' in buffer:
                lines = buffer.split('\n')
                for line in lines[:-1]:
                    val = line.strip()
                    if val.startswith("E:"): # 엔코더 데이터인 경우
                        print(f"\r📊 [엔코더 수신] {val}    ", end="", flush=True)
                        save_to_db(val)
                buffer = lines[-1]
        except: break
    print("\n❌ 로봇 연결 끊김")
    esp_sock = None

def save_to_db(msg):
    try:
        conn = get_db_connection()
        with conn.cursor() as cursor:
            cursor.execute('INSERT INTO encoder_logs (encoder_data) VALUES (%s)', (msg,))
        conn.commit()
        conn.close()
    except Exception as e:
        print(f"DB 저장 에러: {e}")

# --- [2] 웹캠 AI 로직 (장애물 감지 시 M:0,0 송신) ---
def gen_frames():
    global esp_sock, auto_stop_enabled
    camera = cv2.VideoCapture(0)
    while True:
        success, frame = camera.read()
        if not success: break
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red = cv2.inRange(hsv, (0, 150, 50), (10, 255, 255))
        upper_red = cv2.inRange(hsv, (170, 150, 50), (180, 255, 255))
        mask = lower_red + upper_red
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            if cv2.contourArea(cnt) > 5000:
                if auto_stop_enabled and esp_sock:
                    try:
                        esp_sock.sendall(b'M:0,0\n') # 🌟 프로토콜에 맞게 수정
                        print("AI 장애물 감지: M:0,0 전송")
                    except: pass
                # 화면 표시 로직 생략(기존과 동일)
        
        ret, buffer = cv2.imencode('.jpg', frame)
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

# --- [3] 웹 제어 API (M:L,R 프로토콜 번역기) ---
@app.route('/update_position', methods=['POST'])
def update_position():
    global esp_sock
    data = request.get_json()
    direction = str(data.get('direction', 'stop')).lower()
    speed = int(data.get('speed', 150))
    
    if not esp_sock: return jsonify({"status": "error", "message": "로봇 연결 없음"}), 500

    # 🌟 8방향 및 정지 명령 조립
    half = speed // 2
    cmd_map = {
        'w': f"M:{speed},{speed}\n",
        's': f"M:{-speed},{-speed}\n",
        'a': f"M:{-speed},{speed}\n",
        'd': f"M:{speed},{-speed}\n",
        'wa': f"M:{half},{speed}\n",
        'wd': f"M:{speed},{half}\n",
        'sa': f"M:{-half},{-speed}\n",
        'sd': f"M:{-speed},{-half}\n",
        'x': "M:0,0\n", 'stop': "M:0,0\n"
    }
    
    cmd = cmd_map.get(direction, "M:0,0\n")
    try:
        esp_sock.sendall(cmd.encode('utf-8'))
        return jsonify({"status": "success", "sent": cmd.strip()})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/')
def index(): return render_template('index.html')

@app.route('/video_feed')
def video_feed(): return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # 로봇의 접속을 기다리는 서버 스레드 시작
    threading.Thread(target=start_robot_server, daemon=True).start()
    socketio.run(app, host='0.0.0.0', port=5001, debug=True, use_reloader=False)