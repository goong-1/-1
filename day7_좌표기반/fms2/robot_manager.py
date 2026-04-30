import socket
import threading
import re
import time
import os
import platform
import subprocess

class RobotManager:
    def __init__(self, host='0.0.0.0', port=10000):
        self.host = host
        self.port = port
        self.robots = {}  # { robot_id: socket_client }
        self.lock = threading.Lock()
        
        # --- [MAC 어드레스 설정 영역] ---
        # 로봇의 실제 MAC 주소를 소문자로 입력하세요.
        # 예: "3c:61:05:xx:xx:xx"
        self.ROBOT_MACS = {
            "d0:ef:76:47:d3:f4": 0,   # 리더 로봇 (수동 조종용)
            "cc:7b:5c:27:d3:c0": 1  # 추적 로봇 (자동 추종용) 
        }

    def get_mac_address(self, ip):
        """IP 주소를 기반으로 해당 기기의 MAC 주소를 ARP 테이블에서 조회"""
        try:
            if platform.system() == "Windows":
                # Windows: arp -a [IP]
                cmd = ["arp", "-a", ip]
            else:
                # macOS/Linux: arp -n [IP]
                cmd = ["arp", "-n", ip]
                
            output = subprocess.check_output(cmd).decode('utf-8', errors='ignore')
            
            # MAC 주소 정규표현식 (aa:bb:cc... 또는 aa-bb-cc...)
            mac_pattern = r"([0-9a-fA-F]{2}[:-]){5}([0-9a-fA-F]{2})"
            match = re.search(mac_pattern, output)
            if match:
                return match.group(0).lower().replace("-", ":")
            return None
        except Exception as e:
            print(f"⚠️ MAC 조회 실패 ({ip}): {e}")
            return None

    def start_server(self, socketio):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.host, self.port))
        s.listen(5)
        print(f"🤖 MAC 기반 로봇 제어 서버 대기 중 (Port: {self.port})")

        while True:
            client, addr = s.accept()
            ip_address = addr[0]
            
            # 1. 접속한 IP로 MAC 주소 조회
            mac = self.get_mac_address(ip_address)
            
            # 2. MAC 주소를 기반으로 ID 할당
            if mac in self.ROBOT_MACS:
                robot_id = self.ROBOT_MACS[mac]
                with self.lock:
                    self.robots[robot_id] = client
                print(f"✅ 로봇 {robot_id} 연결 성공 (MAC: {mac}, IP: {ip_address})")
            else:
                # 등록되지 않은 MAC일 경우 에러 메시지 출력 후 연결 종료
#                print(f"⚠️ 미등록 기기 접속 차단 - IP: {ip_address}, MAC: {mac}")
#                print(f"💡 이 MAC 주소를 ROBOT_MACS 설정에 추가하세요.")
                client.close()
                continue
            
            threading.Thread(target=self._handle_robot, args=(client, robot_id, socketio), daemon=True).start()

    def _handle_robot(self, client, robot_id, socketio):
        client.setblocking(False)
        buffer = ""
        try:
            while True:
                try:
                    data = client.recv(1024).decode(errors='ignore')
                    if not data: break
                    buffer += data
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        match = re.search(r"L:(\d+)\s+R:(\d+)", line)
                        if match:
                            socketio.emit('encoder_data', {'id': robot_id, 'l': match.group(1), 'r': match.group(2)})
                except BlockingIOError:
                    time.sleep(0.01)
                    continue
                except: break
        finally:
            with self.lock:
                if robot_id in self.robots: del self.robots[robot_id]
            client.close()
            print(f"❌ 로봇 {robot_id} 접속 종료")

    def send_command(self, robot_id, cmd):
        with self.lock:
            if robot_id in self.robots:
                try:
                    self.robots[robot_id].sendall(str(cmd).encode())
                except: pass