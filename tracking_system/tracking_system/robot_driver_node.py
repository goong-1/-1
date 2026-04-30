import socket
import threading
import re
import time
import platform
import subprocess


import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class RobotConnectionManager:
    def __init__(self, node, host, port, robot_macs):
        self.node = node
        self.host = host
        self.port = port
        self.robot_macs = robot_macs
        self.robots = {}
        self.lock = threading.Lock()

    def get_mac_address(self, ip):
        try:
            if platform.system() == 'Windows':
                cmd = ['arp', '-a', ip]
            else:
                cmd = ['arp', '-n', ip]

            output = subprocess.check_output(cmd).decode('utf-8', errors='ignore')
            mac_pattern = r'([0-9a-fA-F]{2}[:-]){5}([0-9a-fA-F]{2})'
            match = re.search(mac_pattern, output)

            if match:
                return match.group(0).lower().replace('-', ':')

            return None

        except Exception as e:
            self.node.get_logger().warn(f'MAC 조회 실패 {ip}: {e}')
            return None
        
    def read_robot_id(self, client):
        """
        ESP32가 접속 직후 보내는 ID 메시지를 읽는다.
        예: ID:0, ID:1
        """
        client.settimeout(3.0)

        try:
            data = client.recv(1024).decode(errors='ignore')
            self.node.get_logger().info(f'초기 수신 데이터: {data.strip()}')
            
            lines = data.replace('\r', '').split('\n')

            for line in lines:
                line = line.strip()

                if line.startswith('ID:'):
                    value = line.split(':', 1)[1].strip()
                    robot_id = int(value)

                    if robot_id in [0, 1]:
                        return robot_id

            return None

        except Exception as e:
            self.node.get_logger().warn(f'로봇 ID 수신 실패: {e}')
            return None

        finally:
            client.settimeout(None)

    def start_server(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.host, self.port))
        s.listen(5)

        self.node.get_logger().info(f'로봇 TCP 서버 대기 중: {self.host}:{self.port}')

        while rclpy.ok():
            try:
                client, addr = s.accept()
            except Exception:
                continue

            ip = addr[0]

            robot_id = self.read_robot_id(client)

            if robot_id is None:
                self.node.get_logger().warn(
                    f'로봇 ID 확인 실패, 접속 차단: IP={ip}'
                )
                client.close()
                continue

            with self.lock:
                # 같은 ID가 이미 연결되어 있으면 기존 연결 닫고 새 연결로 교체
                if robot_id in self.robots:
                    self.node.get_logger().warn(
                        f'로봇 {robot_id} 기존 연결 교체'
                    )
                    try:
                        self.robots[robot_id].close()
                    except Exception:
                        pass

                self.robots[robot_id] = client

            self.node.get_logger().info(
                f'로봇 {robot_id} 연결 성공, IP={ip}'
            )

            threading.Thread(
                target=self.handle_robot,
                args=(client, robot_id),
                daemon=True
            ).start()

    def handle_robot(self, client, robot_id):
        client.setblocking(False)

        try:
            while rclpy.ok():
                try:
                    data = client.recv(1024)
                    if not data:
                        break
                except BlockingIOError:
                    time.sleep(0.01)
                except Exception:
                    break
        finally:
            with self.lock:
                if robot_id in self.robots:
                    del self.robots[robot_id]

            client.close()
            self.node.get_logger().warn(f'로봇 {robot_id} 연결 종료')

    def send_command(self, robot_id, cmd):
        with self.lock:
            if robot_id not in self.robots:
                self.node.get_logger().warn(f'로봇 {robot_id} 미연결: {cmd}')
                return False

            try:
                self.robots[robot_id].sendall((cmd + '\n').encode())
                self.node.get_logger().info(f'로봇 {robot_id} 전송: {cmd}')
                return True
            except Exception as e:
                self.node.get_logger().error(f'로봇 {robot_id} 전송 실패: {e}')
                return False


class RobotDriverNode(Node):
    def __init__(self):
        super().__init__('robot_driver_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('host', '0.0.0.0'),
                ('port', 10000),
                ('robot0_mac', 'd0:ef:76:47:d3:f4'),
                ('robot1_mac', 'cc:7b:5c:27:d3:c0'),
            ]
        )

        host = self.get_parameter('host').value
        port = int(self.get_parameter('port').value)

        robot0_mac = self.get_parameter('robot0_mac').value.lower()
        robot1_mac = self.get_parameter('robot1_mac').value.lower()

        robot_macs = {
            robot0_mac: 0,
            robot1_mac: 1,
        }

        self.manager = RobotConnectionManager(self, host, port, robot_macs)

        threading.Thread(
            target=self.manager.start_server,
            daemon=True
        ).start()

        self.sub0 = self.create_subscription(
            String,
            '/robot0/motor_cmd',
            self.robot0_callback,
            10
        )

        self.sub1 = self.create_subscription(
            String,
            '/robot1/motor_cmd',
            self.robot1_callback,
            10
        )

    def robot0_callback(self, msg):
        self.manager.send_command(0, msg.data)

    def robot1_callback(self, msg):
        self.manager.send_command(1, msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = RobotDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.manager.send_command(0, 'a+0,d+0')
    node.manager.send_command(1, 'a+0,d+0')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()