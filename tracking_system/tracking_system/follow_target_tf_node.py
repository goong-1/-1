import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


def yaw_to_quaternion(yaw):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return 0.0, 0.0, qz, qw


class FollowTargetTfNode(Node):
    def __init__(self):
        super().__init__('follow_target_tf_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('follow_dist', 0.40),
                ('robot0_frame', 'robot0/base_link'),
                ('target_frame', 'follow_target'),
            ]
        )

        self.follow_dist = float(self.get_parameter('follow_dist').value)
        self.robot0_frame = self.get_parameter('robot0_frame').value
        self.target_frame = self.get_parameter('target_frame').value

        self.br = StaticTransformBroadcaster(self)
        self.publish_static_target()

    def publish_static_target(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.robot0_frame
        t.child_frame_id = self.target_frame

        # robot0/base_link 기준 뒤쪽
        t.transform.translation.x = -self.follow_dist
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(0.0)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.br.sendTransform(t)

        self.get_logger().info(
            f'{self.robot0_frame} -> {self.target_frame} published, '
            f'x={-self.follow_dist:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = FollowTargetTfNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()