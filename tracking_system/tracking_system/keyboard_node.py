import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class KeyboardNode(Node):
    def __init__(self):
        super().__init__('keyboard_node')

        self.declare_parameter('robot0_topic', '/robot0/motor_cmd')
        self.declare_parameter('base_pwm', 85)

        topic = self.get_parameter('robot0_topic').value
        self.base_pwm = int(self.get_parameter('base_pwm').value)

        self.pub = self.create_publisher(String, topic, 10)

        self.get_logger().info('WASD 조종 시작. Space: 정지, q: 종료')

        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self.timer = self.create_timer(0.05, self.timer_callback)

    def make_cmd(self, key):
        p = self.base_pwm
        half = int(p / 2)

        if key == 'w':
            return f'a+{p},d+{p}'
        if key == 's':
            return f'a-{p},d-{p}'
        if key == 'a':
            return f'a+{half},d+{p}'
        if key == 'd':
            return f'a+{p},d+{half}'
        if key == ' ':
            return 'a+0,d+0'

        return None

    def timer_callback(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)

            if key == 'q':
                self.publish_cmd('a+0,d+0')
                self.get_logger().info('종료')
                rclpy.shutdown()
                return

            cmd = self.make_cmd(key)
            if cmd is not None:
                self.publish_cmd(cmd)

    def publish_cmd(self, cmd):
        msg = String()
        msg.data = cmd
        self.pub.publish(msg)
        self.get_logger().info(f'robot0 cmd: {cmd}')

    def destroy_node(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.publish_cmd('a+0,d+0')
    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()