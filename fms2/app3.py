from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO
import sqlite3
import random
import threading
import serial
import time

app = Flask(__name__)
app.config['SQLALCHEMY_DATABASE_URI'] = 'mysql+pymysql://vboxmaster:1234@192.168.0.136/tp_and_joy'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
socketio = SocketIO(app, cors_allowed_origins="*" , async_mode='threading') # 명시적으로 스레딩 모드 지정, 신궁 추가!

# 전역 상태 변수 추가
is_arduino_online = False

# --- [1] DB 초기화 (변동 없음) ---
def init_db():
    conn = sqlite3.connect('robot_data.db')
    cursor = conn.cursor()
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS telemetry (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            temp REAL,
            speed REAL,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
        )
    ''')
    cursor.execute('SELECT COUNT(*) FROM telemetry')
    if cursor.fetchone()[0] == 0:
        cursor.execute('INSERT INTO telemetry (temp, speed) VALUES (?, ?)', (25.0, 0.0))
        conn.commit()
    conn.close()

init_db()

# --- [2] 아두이노 시리얼 통신 (상태 업데이트 통합) ---
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
                        
                       
                        # 방향 판별 - 신궁 추가!
                        direction = "CENTER"
                        if joyY < 212:   direction = "UP"
                        elif joyY > 812: direction = "DOWN"
                        elif joyX < 212: direction = "LEFT"
                        elif joyX > 812: direction = "RIGHT"
                        # 신궁 추가!
                        last_received_time = time.time()
                        offline_sent = False

                        # CENTER면 전송 안 함 - 신궁 추가!
                        if direction == "CENTER":
                            continue

                        with sqlite3.connect('robot_data.db') as conn:
                            cursor = conn.cursor()
                            cursor.execute('INSERT INTO telemetry (temp, speed) VALUES (?, ?)', (temp, 0.0))
                            conn.commit()

                        # 온라인 상태 업데이트
                        is_arduino_online = True
                        socketio.emit('serial_data', {
                            'temp': temp, 'joyX': joyX, 'joyY': joyY, 'status': 'Online'
                        }, namespace='/') # 네임스페이스 명시적으로 지정, 신궁 추가!

                        print(f"📥 Received: {temp}, {joyX}, {joyY}")
                        last_received_time = time.time()
                        offline_sent = False 
                        continue 

        except Exception as e:
            print(f"❌ Read Error: {e}")
            ser = None

        # 리셋/연결 끊김 감지 (3초 기준)
        if (time.time() - last_received_time > 3.0) and not offline_sent:
            print("⚠️ [TIMEOUT]: Arduino is resetting... Sending '--'")
            is_arduino_online = False # 오프라인 상태로 변경
            socketio.emit('serial_data', {
                'temp': '--', 'joyX': 512, 'joyY': 512, 'status': 'Offline'
            })
            offline_sent = True 

        time.sleep(0.1)

threading.Thread(target=read_serial, daemon=True).start()

# --- [3] 웹 API (상태에 따라 -- 반환하도록 수정) ---
@app.route('/api/telemetry')
def get_telemetry():
    global is_arduino_online
    
    # 아두이노가 오프라인이면 DB 조회 없이 즉시 -- 반환
    if not is_arduino_online:
        return jsonify({
            "temp": "--",
            "battery": "--",
            "cpu": "--",
            "memory": "--",
            "status": "Offline"
        })

    # 온라인일 때만 DB 조회
    conn = sqlite3.connect('robot_data.db')
    cursor = conn.cursor()
    cursor.execute('SELECT temp FROM telemetry ORDER BY timestamp DESC LIMIT 1')
    row = cursor.fetchone()
    conn.close()

    return jsonify({
        "temp": row[0] if row else "--",
        "battery": random.randint(95, 96),
        "cpu": random.randint(10, 15),
        "memory": random.randint(30, 33),
        "status": "Online"
    })

@app.route('/')
def index():
    return render_template('index.html')

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5001, debug=True, use_reloader=False)