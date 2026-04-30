import math
import cv2
import cv2.aruco as aruco
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import CompressedImage

def yaw_to_quaternion(yaw):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return 0.0, 0.0, qz, qw


class ArucoTfNode(Node):
    def __init__(self):
        super().__init__('aruco_tf_node')

        self.br = TransformBroadcaster(self)
        
        self.image_pub = self.create_publisher(
            CompressedImage,
            '/debug/annotated_image/compressed',
            10
        )
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera_index', 0),
                ('frame_width', 960),
                ('frame_height', 540),
                ('map_width_m', 1.77),
                ('map_height_m', 1.11),
                ('image_corners.left_top', [100, 80]),
                ('image_corners.right_top', [860, 80]),
                ('image_corners.right_bottom', [860, 460]),
                ('image_corners.left_bottom', [100, 460]),
                ('aruco.robot0_left', 10),
                ('aruco.robot0_right', 11),
                ('aruco.robot1_left', 20),
                ('aruco.robot1_right', 21),
                ('frames.map', 'map'),
                ('frames.robot0', 'robot0/base_link'),
                ('frames.robot1', 'robot1/base_link'),
            ]
        )

        self.camera_index = self.get_parameter('camera_index').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value

        self.map_width_m = float(self.get_parameter('map_width_m').value)
        self.map_height_m = float(self.get_parameter('map_height_m').value)

        self.robot0_left = int(self.get_parameter('aruco.robot0_left').value)
        self.robot0_right = int(self.get_parameter('aruco.robot0_right').value)
        self.robot1_left = int(self.get_parameter('aruco.robot1_left').value)
        self.robot1_right = int(self.get_parameter('aruco.robot1_right').value)

        self.map_frame = self.get_parameter('frames.map').value
        self.robot0_frame = self.get_parameter('frames.robot0').value
        self.robot1_frame = self.get_parameter('frames.robot1').value

        image_corners = [
            self.get_parameter('image_corners.left_top').value,
            self.get_parameter('image_corners.right_top').value,
            self.get_parameter('image_corners.right_bottom').value,
            self.get_parameter('image_corners.left_bottom').value,
        ]

        self.H_img_to_map = self.make_homography(image_corners)

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
        self.parameters = aruco.DetectorParameters_create()
        self.parameters.minMarkerPerimeterRate = 0.02
        self.parameters.adaptiveThreshConstant = 5
        self.parameters.minMarkerDistanceRate = 0.02
        self.parameters.adaptiveThreshWinSizeStep = 4
        self.parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        if not self.cap.isOpened():
            self.get_logger().error('카메라 열기 실패')
        else:
            self.get_logger().info('카메라 열림')

        self.timer = self.create_timer(0.033, self.timer_callback)

    def make_homography(self, image_corners):
        img_pts = np.array(image_corners, dtype=np.float32)

        map_pts = np.array([
            [0.0, self.map_height_m],
            [self.map_width_m, self.map_height_m],
            [self.map_width_m, 0.0],
            [0.0, 0.0],
        ], dtype=np.float32)

        H, _ = cv2.findHomography(img_pts, map_pts)

        self.get_logger().info('Homography 설정 완료')
        self.get_logger().info(str(H))

        return H

    def image_to_map(self, point):
        u, v = point
        src = np.array([[[u, v]]], dtype=np.float32)
        dst = cv2.perspectiveTransform(src, self.H_img_to_map)

        x = float(dst[0][0][0])
        y = float(dst[0][0][1])

        return x, y

    def detect_markers(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.parameters
        )

        markers = {}

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

            for i, marker_id in enumerate(ids.flatten()):
                c = corners[i][0]
                cx = int(np.mean(c[:, 0]))
                cy = int(np.mean(c[:, 1]))

                markers[int(marker_id)] = {
                    'center': (cx, cy)
                }

        return markers, frame

    def get_robot_pose_map(self, markers, left_id, right_id):
        if left_id not in markers or right_id not in markers:
            return None

        l_img = markers[left_id]['center']
        r_img = markers[right_id]['center']

        l_map = self.image_to_map(l_img)
        r_map = self.image_to_map(r_img)

        x = (l_map[0] + r_map[0]) / 2.0
        y = (l_map[1] + r_map[1]) / 2.0

        dx = r_map[0] - l_map[0]
        dy = r_map[1] - l_map[1]

        side_theta = math.atan2(dy, dx)

        # 네 실습에서 heading 반전 문제 해결했던 기준
        theta = side_theta + math.pi / 2.0
        theta = (theta + math.pi) % (2.0 * math.pi) - math.pi

        return x, y, theta

    def publish_tf(self, parent, child, x, y, yaw):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.br.sendTransform(t)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('프레임 읽기 실패')
            return

        markers, frame = self.detect_markers(frame)

        robot0_pose = self.get_robot_pose_map(
            markers,
            self.robot0_left,
            self.robot0_right
        )

        robot1_pose = self.get_robot_pose_map(
            markers,
            self.robot1_left,
            self.robot1_right
        )

        if robot0_pose is not None:
            x, y, theta = robot0_pose
            self.publish_tf(self.map_frame, self.robot0_frame, x, y, theta)
            self.get_logger().info(
                f'R0 x={x:.3f}, y={y:.3f}, yaw={math.degrees(theta):.1f}°'
            )

        if robot1_pose is not None:
            x, y, theta = robot1_pose
            self.publish_tf(self.map_frame, self.robot1_frame, x, y, theta)
            self.get_logger().info(
                f'R1 x={x:.3f}, y={y:.3f}, yaw={math.degrees(theta):.1f}°'
            )
            
        self.publish_debug_image(frame)
        
        # cv2.imshow('aruco_tf_node', frame)
        # cv2.waitKey(1)

    def destroy_node(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        #cv2.destroyAllWindows()
        super().destroy_node()
    
    def publish_debug_image(self, frame):
        if frame is None:
            self.get_logger().warn('debug image frame is None')
            return

        if not hasattr(frame, 'size') or frame.size == 0:
            self.get_logger().warn('debug image frame is empty')
            return

        frame = np.ascontiguousarray(frame)

        if frame.dtype != np.uint8:
            frame = frame.astype(np.uint8)

        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            self.get_logger().warn('cv2.imencode failed')
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'overhead_camera'
        msg.format = 'jpeg'
        msg.data = buffer.tobytes()

        self.image_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTfNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()