import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import tf2_ros
from tf2_ros import TransformException
from std_msgs.msg import String, Bool

class FollowerControllerNode(Node):
    def __init__(self):
        super().__init__('follower_controller_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot1_frame', 'robot1/base_link'),
                ('target_frame', 'follow_target'),
                ('map_frame', 'map'),
                ('stop_dist', 0.15),
                ('safety_gap', 0.30),
                ('base_pwm_near', 85),
                ('base_pwm_mid', 95),
                ('base_pwm_far', 110),
                ('max_pwm', 140),
                ('turn_gain', 2.0),
                ('min_turn_pwm', 20),
                ('rear_turn_pwm', 80),
            ]
        )

        
        self.robot1_frame = self.get_parameter('robot1_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.map_frame = self.get_parameter('map_frame').value

        self.stop_dist = float(self.get_parameter('stop_dist').value)
        self.safety_gap = float(self.get_parameter('safety_gap').value)

        self.base_pwm_near = int(self.get_parameter('base_pwm_near').value)
        self.base_pwm_mid = int(self.get_parameter('base_pwm_mid').value)
        self.base_pwm_far = int(self.get_parameter('base_pwm_far').value)

        self.max_pwm = int(self.get_parameter('max_pwm').value)
        self.turn_gain = float(self.get_parameter('turn_gain').value)
        self.min_turn_pwm = int(self.get_parameter('min_turn_pwm').value)
        self.rear_turn_pwm = int(self.get_parameter('rear_turn_pwm').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(String, '/robot1/motor_cmd', 10)

        self.manual_override = False

        self.sub_manual_override = self.create_subscription(
            Bool,
            '/robot1/manual_override',
            self.manual_override_callback,
            10
        )
        
        self.timer = self.create_timer(0.1, self.timer_callback)

    def manual_override_callback(self, msg):
        self.manual_override = msg.data

        if self.manual_override:
            self.get_logger().warn('[MANUAL OVERRIDE] robot1 자동추종 일시정지')
        else:
            self.get_logger().info('[MANUAL OVERRIDE] robot1 자동추종 복귀')
            
    
    def make_cmd(self, pwm_l, pwm_r):
        def sign(n):
            return f'+{n}' if n >= 0 else str(n)

        return f'a{sign(int(pwm_l))},d{sign(int(pwm_r))}'

    def calculate_control(self, local_x, local_y):
        dist = math.sqrt(local_x**2 + local_y**2)
        angle_error = math.degrees(math.atan2(local_y, local_x))

        if dist < self.stop_dist:
            return dist, angle_error, 0, 0, '목표점 도달'

        if local_x < 0:
            if local_y > 0:
                pwm_l = -self.rear_turn_pwm
                pwm_r = self.rear_turn_pwm
            else:
                pwm_l = self.rear_turn_pwm
                pwm_r = -self.rear_turn_pwm

            return dist, angle_error, pwm_l, pwm_r, '후방 목표: 제자리 회전'

        if dist > 0.40:
            base_pwm = self.base_pwm_far
        elif dist > 0.25:
            base_pwm = self.base_pwm_mid
        else:
            base_pwm = self.base_pwm_near

        turn = int(angle_error * self.turn_gain)

        if 3 < abs(angle_error) < 15:
            turn = self.min_turn_pwm if angle_error > 0 else -self.min_turn_pwm

        pwm_l = base_pwm - turn
        pwm_r = base_pwm + turn

        pwm_l = max(min(pwm_l, self.max_pwm), -self.max_pwm)
        pwm_r = max(min(pwm_r, self.max_pwm), -self.max_pwm)

        return dist, angle_error, pwm_l, pwm_r, '전진 추종'

    def safety_gap_ok(self):
        try:
            t_r0 = self.tf_buffer.lookup_transform(
                self.map_frame,
                'robot0/base_link',
                rclpy.time.Time()
            )

            t_r1 = self.tf_buffer.lookup_transform(
                self.map_frame,
                'robot1/base_link',
                rclpy.time.Time()
            )

            x0 = t_r0.transform.translation.x
            y0 = t_r0.transform.translation.y
            x1 = t_r1.transform.translation.x
            y1 = t_r1.transform.translation.y

            gap = math.sqrt((x0 - x1)**2 + (y0 - y1)**2)

            if gap < self.safety_gap:
                self.get_logger().warn(
                    f'[SAFETY] robot gap={gap:.3f}m < {self.safety_gap:.3f}m'
                )
                return False

            return True

        except TransformException as e:
            self.get_logger().warn(f'safety TF 조회 실패: {e}')
            return False

    def publish_stop(self, reason='정지'):
        msg = String()
        msg.data = 'a+0,d+0'
        self.pub.publish(msg)
        self.get_logger().info(f'[STOP] {reason}')

    def timer_callback(self):
        
        if self.manual_override:
            return
        
        if not self.safety_gap_ok():
            self.publish_stop('안전거리 부족 또는 TF 없음')
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.robot1_frame,
                self.target_frame,
                rclpy.time.Time()
            )
        except TransformException as e:
            self.get_logger().warn(f'TF 조회 실패: {e}')
            self.publish_stop('target TF 없음')
            return

        local_x = tf.transform.translation.x
        local_y = tf.transform.translation.y

        dist, angle, pwm_l, pwm_r, desc = self.calculate_control(local_x, local_y)
        cmd = self.make_cmd(pwm_l, pwm_r)

        msg = String()
        msg.data = cmd
        self.pub.publish(msg)

        self.get_logger().info(
            f'[CTRL] {desc} | local_x={local_x:.3f}, local_y={local_y:.3f}, '
            f'dist={dist:.3f}, angle={angle:.1f}°, cmd={cmd}'
        )
    

def main(args=None):
    rclpy.init(args=args)
    node = FollowerControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.publish_stop('종료')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()