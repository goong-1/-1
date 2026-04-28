import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool
import cv2
import numpy as np
import threading
import time
from flask import Flask, render_template, Response
from flask_socketio import SocketIO

# --- [1] Flask 및 ROS 2 하이브리드 노드 정의 ---
class RobotMasterNode(Node):
    def __init__(self):
        super().__init__('robot_master_node')
        
        # ROS 2 퍼블리셔 및 클라이언트 설정
        self.coord_pub = self.create_publisher(Point, 'red_dot_coord', 10)
        self.motor_client = self.create_client(SetBool, 'motor_control')
        
        self.is_motor_on = False
        self.frame = None  # 최신 프레임 저장용
        self.get_logger().info('🚀 ROS 2 마스터 노드 및 비전 시스템 준비 완료!')

    def send_motor_command(self, turn_on):
        """상태가 변할 때만 모터 제어 명령 전송 (중복 호출 방지)"""
        if self.is_motor_on != turn_on:
            req = SetBool.Request()
            req.data = turn_on
            self.motor_client.call_async(req)
            self.is_motor_on = turn_on
            action = "가동" if turn_on else "정지"
            self.get_logger().info(f"🤖 모터 {action} 명령 전송")

# --- [2] Flask 서버 설정 ---
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")
robot_node = None # 글로벌 노드 객체

def gen_frames():
    global robot_node
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    safe_zone = 100

    while True:
        success, frame = cap.read()
        if not success: break

        height, width, _ = frame.shape
        center_x, center_y = width // 2, height // 2

        # 🔵 세이프 존 박스 그리기
        cv2.rectangle(frame, (center_x - safe_zone, center_y - safe_zone),
                      (center_x + safe_zone, center_y + safe_zone), (255, 0, 0), 2)

        # 🔴 빨간색 인식 (HSV)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255])) + \
               cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)

            if M["m00"] > 0 and radius > 15:
                cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                
                # 타겟 표시
                cv2.circle(frame, (cx, cy), int(radius), (0, 255, 255), 2)
                
                # ROS 2 좌표 퍼블리시
                msg = Point(x=float(cx), y=float(cy), z=0.0)
                robot_node.coord_pub.publish(msg)

                # 🚨 세이프 존 이탈 체크 및 제어
                is_out = (cx < center_x - safe_zone or cx > center_x + safe_zone or
                          cy < center_y - safe_zone or cy > center_y + safe_zone)
                
                if is_out:
                    cv2.putText(frame, "OUT -> MOTOR ON!", (10, 30), 1, 2, (0, 0, 255), 2)
                    robot_node.send_motor_command(True)
                    socketio.emit('alert', {'msg': '🚨 타겟 이탈! 모터 가동'}, namespace='/')
                else:
                    cv2.putText(frame, "IN -> MOTOR OFF", (10, 30), 1, 2, (0, 255, 0), 2)
                    robot_node.send_motor_command(False)
        else:
            robot_node.send_motor_command(False)

        # 영상 전송
        ret, buffer = cv2.imencode('.jpg', frame)
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# --- [3] 메인 실행부 ---
def ros_thread():
    global robot_node
    rclpy.init()
    robot_node = RobotMasterNode()
    rclpy.spin(robot_node)
    robot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # ROS 2 노드를 백그라운드 스레드에서 실행
    threading.Thread(target=ros_thread, daemon=True).start()
    
    # Flask 웹 서버 실행
    socketio.run(app, host='0.0.0.0', port=5001, use_reloader=False)