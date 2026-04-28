from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO
import pymysql  # 🌟 sqlite3 대신 pymysql 사용!
import random
import threading
import serial
import time
import socket 
import cv2  

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*" , async_mode='threading') # 비동기방식(쓰레딩) 

# --- [전역 상태 변수] ---
is_arduino_online = False
ESP32_IP = "192.168.0.48" 
ESP32_PORT = 8080
esp_sock = None
auto_stop_enabled = True
# 🌟 MariaDB(리눅스) 접속 정보 셋팅
DB_HOST = '192.168.0.136'
DB_USER = 'vboxmaster'
DB_PASSWORD = '1234'
DB_NAME = 'tp_and_joy'

# DB 연결 헬퍼 함수
def get_db_connection():
    return pymysql.connect(
        host=DB_HOST,
        user=DB_USER,
        password=DB_PASSWORD,
        database=DB_NAME,
        charset='utf8mb4',
        cursorclass=pymysql.cursors.DictCursor # 데이터를 딕셔너리 형태로 깔끔하게 가져옴
    )

# --- [1] 리눅스 MariaDB 테이블 초기화 ---
def init_db():
    try:
        conn = get_db_connection()
        cursor = conn.cursor()
        
        # 🌟 MariaDB 문법에 맞게 AUTO_INCREMENT로 변경됨
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS telemetry (
                id INT AUTO_INCREMENT PRIMARY KEY,
                temp FLOAT,
                speed FLOAT,
                timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
            )
        ''')
        
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS encoder_logs (
                id INT AUTO_INCREMENT PRIMARY KEY,
                encoder_data VARCHAR(255),
                timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
            )
        ''')
        
        # 🌟 테스트용 데이터 강제 삽입 (MariaDB는 ? 대신 %s 사용)
        cursor.execute('INSERT INTO encoder_logs (encoder_data) VALUES (%s)', ('[SYSTEM] 리눅스 MariaDB 접속 성공!',))
        
        cursor.execute('SELECT COUNT(*) AS cnt FROM telemetry')
        if cursor.fetchone()['cnt'] == 0:
            cursor.execute('INSERT INTO telemetry (temp, speed) VALUES (%s, %s)', (25.0, 0.0))
            
        conn.commit()
        conn.close()
        print(f"📂 리눅스 MariaDB({DB_HOST}) 테이블 연결 및 초기화 완료!")
    except Exception as e:
        print(f"❌ MariaDB 연결 실패 (리눅스 DB가 켜져있는지 확인하세요): {e}")

# 서버가 켜질 때 무조건 DB 세팅 실행!
init_db()

# --- [2] ESP32 로봇 연결 및 수신 (MariaDB 저장) ---
def connect_esp32():
    global esp_sock
    try:
        esp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        esp_sock.settimeout(5) 
        esp_sock.connect((ESP32_IP, ESP32_PORT))
        print(f"✅ SUCCESS: 로봇(ESP32: {ESP32_IP}) TCP 연결 성공!")
        threading.Thread(target=receive_esp32_data, daemon=True).start()
    except Exception as e:
        print(f"⚠️ 로봇 연결 실패: {e}")

def receive_esp32_data():
    global esp_sock
    buffer = ""
    while True:
        try:
            data = esp_sock.recv(1024).decode('utf-8')
            if not data: break
            buffer += data
            if '\n' in buffer:
                lines = buffer.split('\n')
                for line in lines[:-1]:
                    encoder_val = line.strip()
                    print(f"\r📊 [로봇 수신] {encoder_val}    ", end="", flush=True)
                    
                    # 🌟 수신된 데이터를 리눅스 MariaDB로 쏩니다!
                    try:
                        conn = get_db_connection()
                        cursor = conn.cursor()
                        cursor.execute('INSERT INTO encoder_logs (encoder_data) VALUES (%s)', (encoder_val,))
                        conn.commit()
                        conn.close()
                    except Exception as db_err:
                        print(f"\n❌ DB 저장 에러: {db_err}")
                        
                buffer = lines[-1]
        except Exception as e:
            print(f"\n❌ 수신 오류: {e}")
            break
    esp_sock = None

# --- [3] 로컬 아두이노 시리얼 통신 ---
def read_serial():
    global ser, is_arduino_online
    try:
        ser = serial.Serial('com9', 9600, timeout=1)
        print("✅ SUCCESS: Arduino Connected")
    except Exception as e:
        ser = None
        print(f"⚠️ Arduino Connection Failed: {e}")

    last_received_time = time.time()
    offline_sent = False 

    while True:
        try:
            if ser and ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    parts = line.split(',')
                    if len(parts) == 3:
                        temp, joyX, joyY = float(parts[0]), int(parts[1]), int(parts[2])
                        
                        is_arduino_online = True
                        socketio.emit('serial_data', {
                            'temp': temp, 'joyX': joyX, 'joyY': joyY, 'status': 'Online'
                        }, namespace='/') 
                        last_received_time = time.time()
                        offline_sent = False 
        except Exception as e:
            ser = None

        if (time.time() - last_received_time > 3.0) and not offline_sent:
            is_arduino_online = False 
            socketio.emit('serial_data', {'status': 'Offline'})
            offline_sent = True 
        time.sleep(0.1)
        
# --- [추가: 웹캠 영상 처리 및 AI 로직] ---
def gen_frames():
    global esp_sock, auto_stop_enabled
    camera = cv2.VideoCapture(0)  # 0번 웹캠 사용
    
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            # 🔴 빨간색 인식 로직 시작
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # 빨간색 범위 (주황~빨강 사이 조정 필요)
            lower_red = cv2.inRange(hsv, (0, 150, 50), (10, 255, 255))
            upper_red = cv2.inRange(hsv, (170, 150, 50), (180, 255, 255))
            mask = lower_red + upper_red

            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 5000: # 5000 이상의 면적이면 근접 장애물로 판단
                    # [웹으로 알림]
                    socketio.emit('alert_message', {'msg': '🚨 장애물 감지: 자동 정지 실행!'}, namespace='/')
                
                    # [로봇에게 정지 명령] 
                    # 사용자님의 기존 cmd_map에 '3'이 stop이었으니 3을 보냅니다.
                    if esp_sock:
                        try:
                            esp_sock.sendall(b'3\n') 
                            print("AI가 장애물을 보고 로봇을 멈췄습니다!")
                        except Exception as e:
                            print(f"전송 에러: {e}")
                    # 1. 화면에 표시
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    cv2.putText(frame, "!!! DANGER !!!", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

                    # 2. 로봇 정지 명령 (연결되어 있다면)
                    if auto_stop_enabled and esp_sock:
                        try:
                            esp_sock.sendall(b'3\n') # 기존 cmd_map의 stop('3')
                            # 🌟 MariaDB에 자동 정지 로그 기록
                            conn = get_db_connection()
                            cursor = conn.cursor()
                            cursor.execute('INSERT INTO encoder_logs (encoder_data) VALUES (%s)', ('[AI] 장애물 감지: 긴급 정지 실행',))
                            conn.commit()
                            conn.close()
                        except: pass

            # 영상을 웹 브라우저가 인식할 수 있는 포맷으로 변환
            ret, buffer = cv2.imencode('.jpg', frame)
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            
# --- [4] 웹 API ---
@app.route('/update_position', methods=['POST'])
def update_position():
    global esp_sock
    data = request.get_json()
    if not data: return jsonify({"status": "error"}), 400

    direction = data.get('direction')

    if esp_sock:
        try:
            cmd_map = {
                'forward': '1', 'backward': '2', 'stop': '3', 
                'left': '4', 'right': '5',
                '1': '1', '2': '2', '3': '3', '4': '4', '5': '5'
            }
            physical_cmd = cmd_map.get(str(direction).lower(), '3') 
            
            esp_sock.sendall((physical_cmd + '\n').encode('utf-8'))
            print(f"\n📤 전송 명령: {physical_cmd}")
            return jsonify({"status": "success"})
        except Exception as e:
            return jsonify({"status": "error", "message": str(e)}), 500
    return jsonify({"status": "error", "message": "연결 없음"}), 500

@app.route('/api/telemetry')
def get_telemetry():
    global is_arduino_online
    
    if not is_arduino_online:
        return jsonify({
            "temp": "--", "battery": "--", "cpu": "--", "memory": "--", "status": "Offline"
        })

    try:
        conn = get_db_connection()
        cursor = conn.cursor()
        cursor.execute('SELECT temp FROM telemetry ORDER BY timestamp DESC LIMIT 1')
        row = cursor.fetchone()
        conn.close()

        return jsonify({
            "temp": row['temp'] if row else "--",
            "battery": random.randint(95, 96),
            "cpu": random.randint(10, 15),
            "memory": random.randint(30, 33),
            "status": "Online"
        })
    except:
        return jsonify({"temp": "--", "status": "DB Error"})

# 🌟🌟 [리눅스 DB 강제 확인용 API] 🌟🌟
@app.route('/check_db')
def check_db():
    try:
        conn = get_db_connection()
        cursor = conn.cursor()
        
        # 리눅스 MariaDB에서 최신 로그 10개를 꺼내옵니다.
        cursor.execute('SELECT * FROM encoder_logs ORDER BY id DESC LIMIT 10')
        logs = cursor.fetchall()
        conn.close()
        
        return jsonify({
            "목적지": f"리눅스 MariaDB ({DB_HOST})",
            "현재_데이터": logs
        })
    except Exception as e:
        return jsonify({"오류발생": str(e)})

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    connect_esp32()
    threading.Thread(target=read_serial, daemon=True).start()
    socketio.run(app, host='0.0.0.0', port=5001, debug=True, use_reloader=False)