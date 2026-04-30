import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class PureFollower(Node):
    def __init__(self):
        super().__init__('pure_follower')
        self.cmd_pub = self.create_publisher(Twist, '/model/robot_1/cmd_vel', 10)
        self.create_subscription(Odometry, '/model/robot_0/odometry', self.leader_callback, 10)
        self.create_subscription(Odometry, '/model/robot_1/odometry', self.follower_callback, 10)
        
        self.leader_pose = None
        self.follower_pose = None
        self.follower_yaw = 0.0
        self.last_error_angle = 0.0
        
        self.spawn_leader_x = 0.5  
        self.spawn_leader_y = 0.0
        self.spawn_follower_x = -0.5 
        self.spawn_follower_y = 0.0
        
        # 신궁님 코드의 전역 변수 이식
        self.last_error_angle = 0.0

    def leader_callback(self, msg):
        self.leader_pose = msg.pose.pose.position

    def follower_callback(self, msg):
        self.follower_pose = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.follower_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.control_loop()

    def control_loop(self):
        if self.leader_pose is None or self.follower_pose is None:
            return
       
        true_leader_x = self.leader_pose.x + self.spawn_leader_x
        true_leader_y = self.leader_pose.y + self.spawn_leader_y

        true_follower_x = self.follower_pose.x + self.spawn_follower_x
        true_follower_y = self.follower_pose.y + self.spawn_follower_y
        
        # 1. 위치 기반 거리 및 각도 오차 계산 (app.py 로직)
        dx = true_leader_x - true_follower_x
        dy = true_leader_y - true_follower_y
        dist_cm = math.sqrt(dx**2 + dy**2) * 100.0 # m -> cm 변환

        # self.get_logger().info(f"리더위치: ({true_leader_x:.2f}, {true_leader_y:.2f}) | 팔로워위치: ({true_follower_x:.2f}, {true_follower_y:.2f}) | 계산된거리: {dist_cm:.1f}cm")

        target_rad = math.atan2(dy, dx)
        follower_heading_deg = math.degrees(self.follower_yaw)
        target_heading_deg = math.degrees(target_rad)

        # 각도 오차 계산 (app.py 동일): 양수면 타겟이 좌측에 있음
        error_angle = (target_heading_deg - follower_heading_deg + 180) % 360 - 180

        twist = Twist()
        
        # app.py의 정지 기준거리 30cm
        stop_threshold = 40.0 

        if dist_cm > stop_threshold:
            # --- [app.py의 V (선속도) 및 W (조향) 계산 로직 시작] ---
            dist_error = dist_cm - stop_threshold
            v = dist_error * 3.0
            # v = 75.0 + (dist_error * 2.2)
            v = max(min(v, 150.0), -150.0)
            
            deadzone = 10.0
            base_w_gain = 0.8
            d_gain = 0.3
            w = 0.0

            if abs(error_angle) <= deadzone:
                w = 0.0
                self.last_error_angle = 0.0
            else:
                # [A] 배면 회전 고정
                if abs(error_angle) > 140:
                    w = 100.0 if error_angle > 0 else -100.0
                # [B] 정밀 PD 제어
                else:
                    adjusted_error = error_angle - (deadzone if error_angle > 0 else -deadzone)
                    dynamic_gain = base_w_gain * (abs(adjusted_error) / 90.0)
                    dynamic_gain = max(0.4, min(dynamic_gain, base_w_gain))
                    
                    p_term = adjusted_error * dynamic_gain
                    d_term = (error_angle - self.last_error_angle) * d_gain
                    w = p_term + d_term 
                
                self.last_error_angle = error_angle

            max_w = 100.0 if abs(v) < 80 else abs(v) * 0.9
            w = max(min(w, max_w), -max_w)

            # --- [시뮬레이션 전용: PWM을 m/s 및 rad/s로 변환] ---
            # 좌/우 모터 PWM 계산 (app.py 동일)
            limit = 150.0
            min_pwm = 75.0
            pwm_l_raw = v - w 
            pwm_r_raw = v + w

            pwm_l = max(min(pwm_l_raw, limit), -limit)
            pwm_r = max(min(pwm_r_raw, limit), -limit)

            # 사구간(Deadzone) 처리: 75 미만은 정지 (0)
            #if 0 < pwm_l < min_pwm: pwm_l = 0
            #elif -min_pwm < pwm_l < 0: pwm_l = 0
            #if 0 < pwm_r < min_pwm: pwm_r = 0
            #elif -min_pwm < pwm_r < 0: pwm_r = 0

            # 1. 선속도(linear.x) 변환: 평균 PWM을 m/s로
            # (PWM 80일 때 0.2 m/s가 되도록 스케일링: 0.2 / 80 = 0.0025)
            average_pwm = (pwm_l + pwm_r) / 2.0
            twist.linear.x = average_pwm * 0.0025
            
            # 2. 각속도(angular.z) 변환: w값을 회전속도(rad/s)로
            # w가 양수면 좌회전(+) (비율 상수 0.015 적용)
            twist.angular.z = w * 0.015

            # 디버깅 출력
            move_dir = "직진" if abs(w) < 1 else ("좌회전" if w > 0 else "우회전")
            self.get_logger().info(f"[자동] {move_dir} | 거리: {dist_cm:.1f}cm | 각도오차: {error_angle:.1f} | L:{int(pwm_l)} R:{int(pwm_r)}")

        else:
            # 30cm 이내면 정지
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("목표 도달 (30cm 이내) -> 정지")

        self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    node = PureFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()