import rclpy  # ROS 2 파이썬 클라이언트 라이브러리
from rclpy.node import Node  # 노드 클래스
import cv2  # OpenCV 라이브러리
from cv_bridge import CvBridge  # ROS 이미지와 OpenCV 이미지 변환 도구
from sensor_msgs.msg import Image  # ROS 이미지 메시지 타입

class CameraNode(Node):
    def __init__(self):
        # 노드 이름을 'webcam_node'로 초기화
        super().__init__('webcam_node')
        # 큐 사이즈를 조금 넉넉히 10으로 늘려줍니다.
        # self.publisher_ = self.create_publisher(Image, 'webcam_image', 10)
        # self.timer = self.create_timer(0.033, self.timer_callback)
        # 'webcam_image'라는 토픽으로 Image 메시지를 보낼 퍼블리셔 생성 (큐 사이즈 1)
        self.publisher_ = self.create_publisher(Image, 'webcam_image', 1)
       
        # 0.033초마다(약 30 FPS) timer_callback 함수를 실행하는 타이머 생성
        self.timer = self.create_timer(0.033, self.timer_callback)
       
        # 기본 웹캠(0번) 연결
        # self.cap = cv2.VideoCapture(0)
        
        # 1. V4L2 백엔드를 명시하여 카메라 열기
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

        # 2. MJPG 압축 포맷 설정 (WSL2 병목 현상 해결의 핵심!)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

        # 3. 해상도 명시 (아까 확인하신 640x480으로 고정)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
       
        # 이미지 변환을 위한 CvBridge 객체 생성
        self.bridge = CvBridge()

    def timer_callback(self):
        # 웹캠으로부터 한 프레임을 읽어옴
        ret, frame = self.cap.read()
       
        if ret:
            # 읽어온 프레임을 640x480 사이즈로 조절
            frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_NEAREST)
           
            # OpenCV 이미지(numpy 배열)를 ROS 이미지 메시지로 변환
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
           
            # 변환된 메시지를 토픽으로 발행(Publish)
            self.publisher_.publish(msg)
           
            # (옵션) 주석을 풀면 노드 실행 시 화면을 띄워줍니다.
            # cv2.imshow('Webcam', frame)
           
            # 'q' 키를 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()
        else:
            self.get_logger().error('Failed to capture image')

def main(args=None):
    rclpy.init(args=args)  # ROS 2 초기화
    node = CameraNode()    # 노드 객체 생성
   
    try:
        rclpy.spin(node)   # 노드가 종료될 때까지 계속 실행
    except KeyboardInterrupt:
        pass               # Ctrl+C 입력 시 안전하게 종료
    finally:
        # 종료 시 자원 해제
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()