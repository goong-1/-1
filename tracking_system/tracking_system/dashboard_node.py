import os
import threading
import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Bool

from flask import Flask, render_template, Response
from flask_socketio import SocketIO
from ament_index_python.packages import get_package_share_directory


class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 5001)

        self.host = self.get_parameter('host').value
        self.port = int(self.get_parameter('port').value)

        # ROS2 motor command publishers
        self.pub_robot0 = self.create_publisher(String, '/robot0/motor_cmd', 10)
        self.pub_robot1 = self.create_publisher(String, '/robot1/motor_cmd', 10)
        
        self.latest_jpeg = None
        self.image_lock = threading.Lock()

        self.sub_debug_image = self.create_subscription(
            CompressedImage,
            '/debug/annotated_image/compressed',
            self.debug_image_callback,
            10
        )

        # 자동추종 명령 로그 표시용
        self.sub_robot1_cmd = self.create_subscription(
            String,
            '/robot1/motor_cmd',
            self.robot1_cmd_callback,
            10
        )
        
        self.pub_robot1_manual_override = self.create_publisher(
            Bool,
            '/robot1/manual_override',
            10
        )

        base_dir = get_package_share_directory('tracking_system')

        self.app = Flask(
            __name__,
            template_folder=os.path.join(base_dir, 'templates'),
            static_folder=os.path.join(base_dir, 'static')
        )

        self.socketio = SocketIO(
            self.app,
            cors_allowed_origins='*',
            async_mode='threading',
            manage_session=False
        )

        self.robot_calibrations = {
            0: 1.18,
            1: 1.05,
        }

        self.register_routes()
        self.register_socket_events()
        
    def debug_image_callback(self, msg):
        with self.image_lock:
            self.latest_jpeg = bytes(msg.data)
            
    def register_routes(self):
        @self.app.route('/')
        def index():
            return render_template('index.html')

        @self.app.route('/video_feed')
        def video_feed():
            """
            기존 zone3_map.html이 url_for('video_feed')를 사용하므로
            라우트는 일단 유지한다.

            단, 지금 버전에서는 aruco_tf_node가 카메라를 잡고 있으므로
            dashboard_node가 카메라를 또 열면 충돌할 수 있다.
            그래서 임시 빈 스트림을 둔다.
            나중에 ROS2 image topic 구독 방식으로 교체하면 된다.
            """
            return Response(
                self.empty_stream(),
                mimetype='multipart/x-mixed-replace; boundary=frame'
            )

    def empty_stream(self):
        while rclpy.ok():
            with self.image_lock:
                frame = self.latest_jpeg

            if frame is None:
                img = np.zeros((360, 640, 3), dtype=np.uint8)
                cv2.putText(
                    img,
                    'Waiting for camera image...',
                    (120, 180),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (180, 180, 180),
                    2,
                    cv2.LINE_AA
                )
                ret, buffer = cv2.imencode('.jpg', img)
                if not ret:
                    time.sleep(0.1)
                    continue
                frame = buffer.tobytes()

            yield (
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' +
                frame +
                b'\r\n'
            )

            time.sleep(0.03)

    def register_socket_events(self):
        @self.socketio.on('connect')
        def on_connect():
            self.get_logger().info('Dashboard client connected')

            self.socketio.emit('log', {
                'type': 'SYSTEM',
                'msg': 'Dashboard connected',
                'status': 'success'
            })

        @self.socketio.on('drive_control')
        def handle_drive(data):
            """
            기존 goong.js의 window.moveRobot()이 보내는 이벤트.
            기존: robot_mgr.send_command(robot_id, cmd)
            변경: /robot0/motor_cmd 또는 /robot1/motor_cmd publish
            """
            print("[DEBUG drive_control received]", data, flush=True)
            cmd = str(data.get('command', 'a+0,d+0')).strip()
            robot_id = int(data.get('id', 0))

            msg = String()
            msg.data = cmd

            if robot_id == 0:
                self.pub_robot0.publish(msg)
            elif robot_id == 1:
                self.pub_robot1.publish(msg)

            self.get_logger().info(f'[WEB] robot{robot_id} cmd: {cmd}')

            if not self.is_stop_cmd(cmd):
                self.socketio.emit('log', {
                    'type': 'MANUAL',
                    'msg': f'Robot {robot_id} CMD: {cmd}',
                    'status': 'success'
                })

        @self.socketio.on('emergency_control_robot1')
        def handle_emergency_robot1(data):
            """
            기존 goong.js의 방향키 제어용.
            robot1 수동 override 명령을 /robot1/motor_cmd로 publish.
            """
            key_cmd = data.get('command', 'stop')
            base_pwm = int(data.get('pwm') or 85)
            
            override_msg = Bool()
            override_msg.data = key_cmd != 'stop'
            self.pub_robot1_manual_override.publish(override_msg)
            cali = self.robot_calibrations.get(1, 1.0)
            half_pwm = int(base_pwm / 2)

            pwm_l, pwm_r = 0, 0
            move_desc = '정지'

            if key_cmd == 'up':
                pwm_l, pwm_r = base_pwm, base_pwm
                move_desc = '전진'

            elif key_cmd == 'down':
                pwm_l, pwm_r = -base_pwm, -base_pwm
                move_desc = '후진'

            elif key_cmd == 'left':
                pwm_l, pwm_r = half_pwm, base_pwm
                move_desc = '전진 좌회전'

            elif key_cmd == 'right':
                pwm_l, pwm_r = base_pwm, half_pwm
                move_desc = '전진 우회전'

            elif key_cmd == 'backLeft':
                pwm_l, pwm_r = -half_pwm, -base_pwm
                move_desc = '후진 좌회전'

            elif key_cmd == 'backRight':
                pwm_l, pwm_r = -base_pwm, -half_pwm
                move_desc = '후진 우회전'

            elif key_cmd == 'stop':
                pwm_l, pwm_r = 0, 0
                move_desc = '정지'

            pwm_l_final = int(pwm_l)
            pwm_r_final = int(pwm_r * cali)

            cmd = self.make_motor_cmd(pwm_l_final, pwm_r_final)

            msg = String()
            msg.data = cmd
            self.pub_robot1.publish(msg)

            self.get_logger().info(f'[WEB-EMERGENCY] robot1 {move_desc}: {cmd}')

            self.socketio.emit('log', {
                'type': 'EMERGENCY',
                'msg': f'Robot 1 Override: {cmd}',
                'status': 'danger'
            })

        @self.socketio.on('update_calibration')
        def handle_calibration(data):
            try:
                robot_id = int(data.get('id'))
                factor = float(data.get('factor', 1.0))
                self.robot_calibrations[robot_id] = factor

                self.socketio.emit('log', {
                    'type': 'CALIB',
                    'msg': f'Robot {robot_id} calibration={factor}',
                    'status': 'success'
                })

            except Exception as e:
                self.get_logger().warn(f'calibration update failed: {e}')

        @self.socketio.on('change_vision_mode')
        def handle_mode_change(data):
            mode = data.get('mode', 'TRACK')

            self.get_logger().info(f'[WEB] vision mode={mode}')

            self.socketio.emit('log', {
                'type': 'MODE',
                'msg': f'Vision mode changed: {mode}',
                'status': 'info'
            })

        @self.socketio.on('emergency_stop')
        def handle_emergency_stop():
            stop = String()
            stop.data = 'a+0,d+0'

            self.pub_robot0.publish(stop)
            self.pub_robot1.publish(stop)
            
            override_msg = Bool()
            override_msg.data = False
            self.pub_robot1_manual_override.publish(override_msg)
            

            self.socketio.emit('log', {
                'type': 'STOP',
                'msg': 'Emergency stop',
                'status': 'danger'
            })
    def is_stop_cmd(self, cmd):
        normalized = cmd.replace(" ", "").replace("\n", "").replace("\r", "")
        return normalized in ["a+0,d+0", "a0,d0"]        
            
    def make_motor_cmd(self, pwm_l, pwm_r):
        def sign(n):
            return f'+{n}' if n >= 0 else str(n)

        return f'a{sign(pwm_l)},d{sign(pwm_r)}'

    def robot1_cmd_callback(self, msg):
        
        """follower_controller_node가 발행한 자동추종 명령을 대시보드 로그에 표시."""
        if self.is_stop_cmd(msg.data):
            return
            
        self.socketio.emit('log', {
            'type': 'AUTO',
            'msg': f'Robot 1 AUTO CMD: {msg.data}',
            'status': 'info'
        })

    def run_flask(self):
        self.get_logger().info(
            f'Dashboard running at http://{self.host}:{self.port}'
        )

        self.socketio.run(
            self.app,
            host=self.host,
            port=self.port,
            debug=False,
            allow_unsafe_werkzeug=True
        )


def main(args=None):
    rclpy.init(args=args)

    node = DashboardNode()

    flask_thread = threading.Thread(
        target=node.run_flask,
        daemon=True
    )
    flask_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    stop = String()
    stop.data = 'a+0,d+0'
    node.pub_robot0.publish(stop)
    node.pub_robot1.publish(stop)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()