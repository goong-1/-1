from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO
import sqlite3
import random
import threading
import serial
import time
import socket 
import os # 경로 처리를 위해 추가

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*" , async_mode='threading')

# --- [경로 설정] ---
base_dir = os.path.abspath(os.path.dirname(__file__))
db_path = os.path.join(base_dir, 'robot_data.db')

# --- [전역 상태 변수] ---
is_arduino_online = False
ESP32_IP = "192.168.0.60" 
ESP32_PORT = 8080
esp_sock = None

# --- [1] 로컬 DB 초기화 ---
def init_db():
    conn = sqlite3.connect(db_path) # 절대 경로 사용
    cursor = conn.cursor()
    
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS telemetry (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            temp REAL,
            speed REAL,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
        )
    ''')
    
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS encoder_logs (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            encoder_data TEXT,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
        )
    ''')
    
    cursor.execute('SELECT COUNT(*) FROM telemetry')
    if cursor.fetchone()[0] == 0:
        cursor.execute('INSERT INTO telemetry (temp, speed) VALUES (?, ?)', (25.0, 0.0))
        
    conn.commit()
    conn.close()
    print(f"📂 DB 초기화 완료: {db_path}")
    
init_db() # 여기서만 호출

# --- [2] ESP32 로봇 연결 및 수신 ---
def connect_esp32():
    global esp_sock
    try:
        esp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        esp_sock.settimeout(5) # 연결 타임아웃 추가
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
                    
                    with sqlite3.connect(db_path) as conn:
                        cursor = conn.cursor()
                        cursor.execute('INSERT INTO encoder_logs (encoder_data) VALUES (?)', (encoder_val,))
                        conn.commit()
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
                        
                        # 방향 판별 로직은 그대로 유지
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

# --- [4] 웹 API ---
@app.route('/update_position', methods=['POST'])
def update_position():
    global esp_sock
    data = request.get_json()
    if not data: return jsonify({"status": "error"}), 400

    direction = data.get('direction')

    if esp_sock:
        try:
            # 🌟 [수정] 4와 5를 추가하여 좌우 회전이 가능하게 함
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

@app.route('/')
def index():
    return render_template('index.html')

if __name__ == '__main__':
    init_db() # 여기서만 호출
    connect_esp32()
    threading.Thread(target=read_serial, daemon=True).start()
    socketio.run(app, host='0.0.0.0', port=5001, debug=True, use_reloader=False)