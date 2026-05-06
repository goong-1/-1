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

        self.leader_pose   = None
        self.follower_pose = None
        self.follower_yaw  = 0.0
        self.last_error_angle = 0.0

        self.spawn_leader_x   =  0.5
        self.spawn_leader_y   =  0.0
        self.spawn_follower_x = -0.5
        self.spawn_follower_y =  0.0

        # ── path_queue 파라미터 ─────────────────────────────────────
        self.path_queue            = []
        self.last_saved_pos        = None
        self.min_save_dist_m       = 0.02   # 5cm마다 경로점 저장
        self.target_reach_dist_cm  = 4.0  # 경로점 도달 판정 거리
        self.stop_threshold_cm     = 24.0  # 안전 정지 거리
        self.jump_threshold_m      = 0.80  # 80cm 초과 시 점프
        self.jump_queue_count      = 10    # 큐 10개 초과 시 점프
        # ────────────────────────────────────────────────────────────

        # ── 후진 탈출 파라미터 ───────────────────────────────────────
        self.stuck_check_dist_cm = 0.3   # 이 이하로 줄면 stuck 의심
        self.stuck_timeout       = 3.0   # 초
        self.reverse_duration    = 0.6   # 초
        self.reverse_speed       = -0.15 # m/s

        self.last_dist_to_target = None
        self.stuck_start_time    = None
        self.reversing           = False
        self.reverse_start_time  = None
        # ────────────────────────────────────────────────────────────

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

    # ── 오도메트리 콜백 ─────────────────────────────────────────────
    def leader_callback(self, msg):
        self.leader_pose = msg.pose.pose.position

    def follower_callback(self, msg):
        self.follower_pose = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.follower_yaw = math.atan2(siny_cosp, cosy_cosp)

    # ── 유틸 ────────────────────────────────────────────────────────
    def dist_m(self, p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def publish_stop(self, reason='정지'):
        twist = Twist()
        self.cmd_pub.publish(twist)
        self.get_logger().info(f'[STOP] {reason}')

    def save_path_if_needed(self, pos):
        """Robot0 경로점을 min_save_dist_m 간격으로 큐에 저장"""
        if self.last_saved_pos is None:
            self.path_queue.append(pos)
            self.last_saved_pos = pos
            return
        if self.dist_m(pos, self.last_saved_pos) >= self.min_save_dist_m:
            self.path_queue.append(pos)
            self.last_saved_pos = pos

    def map_to_local(self, target, robot_pos, robot_yaw):
        """맵 좌표의 target을 Robot1 로컬 좌표로 변환"""
        dx = target[0] - robot_pos[0]
        dy = target[1] - robot_pos[1]
        local_x = math.cos(-robot_yaw) * dx - math.sin(-robot_yaw) * dy
        local_y = math.sin(-robot_yaw) * dx + math.cos(-robot_yaw) * dy
        return local_x, local_y

    # ── 메인 제어 루프 (10Hz 타이머) ────────────────────────────────
    def timer_callback(self):
        if self.leader_pose is None or self.follower_pose is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9

        # 절대 좌표 계산
        leader_pos   = (self.leader_pose.x   + self.spawn_leader_x,
                        self.leader_pose.y   + self.spawn_leader_y)
        follower_pos = (self.follower_pose.x + self.spawn_follower_x,
                        self.follower_pose.y + self.spawn_follower_y)

        realtime_dist_m = self.dist_m(leader_pos, follower_pos)

        # ── [1] 후진 중 처리 ────────────────────────────────────────
        if self.reversing:
            elapsed = now - self.reverse_start_time
            if elapsed < self.reverse_duration:
                twist = Twist()
                twist.linear.x = self.reverse_speed
                self.cmd_pub.publish(twist)
                self.get_logger().info(f'[후진 탈출] {elapsed:.1f}s / {self.reverse_duration}s')
                return
            else:
                self.reversing           = False
                self.reverse_start_time  = None
                self.stuck_start_time    = None
                self.last_dist_to_target = None
                self.get_logger().info('[후진 완료] 추종 재개')

        # ── [2] Robot0 경로 저장 ────────────────────────────────────
        self.save_path_if_needed(leader_pos)

        # ── [3] Jump 로직 ────────────────────────────────────────────
        if (realtime_dist_m >= self.jump_threshold_m or
                len(self.path_queue) >= self.jump_queue_count):
            reason = (f'{realtime_dist_m*100:.1f}cm 초과'
                      if realtime_dist_m >= self.jump_threshold_m
                      else f'큐 {len(self.path_queue)}개 초과')
            self.path_queue.clear()
            self.path_queue.append(leader_pos)
            self.last_saved_pos = leader_pos
            self.get_logger().warn(f'[JUMP] {reason} → 현재 위치로 점프')

        # ── [4] 안전 정지 ────────────────────────────────────────────
        if realtime_dist_m * 100 <= self.stop_threshold_cm:
            self.publish_stop(f'인접 대기 {realtime_dist_m*100:.1f}cm')
            self.stuck_start_time    = None
            self.last_dist_to_target = None
            return

        # ── [5] 경로 큐 없으면 정지 ─────────────────────────────────
        if not self.path_queue:
            self.publish_stop('path_queue 비어있음')
            return

        # ── [6] 현재 목표점 ──────────────────────────────────────────
        target = self.path_queue[0]
        local_x, local_y = self.map_to_local(target, follower_pos, self.follower_yaw)
        dist_to_target_cm = math.sqrt(local_x**2 + local_y**2) * 100.0

        # 목표점 도달 → 큐에서 제거
        if dist_to_target_cm <= self.target_reach_dist_cm:
            self.path_queue.pop(0)
            self.last_dist_to_target = None
            self.stuck_start_time    = None
            if not self.path_queue:
                self.publish_stop('경로점 도달, 다음 목표 없음')
                return
            target = self.path_queue[0]
            local_x, local_y = self.map_to_local(target, follower_pos, self.follower_yaw)
            dist_to_target_cm = math.sqrt(local_x**2 + local_y**2) * 100.0

        # ── [7] Stuck 감지 ───────────────────────────────────────────
        if self.last_dist_to_target is not None:
            dist_change = self.last_dist_to_target - dist_to_target_cm
            if dist_change < self.stuck_check_dist_cm:
                if self.stuck_start_time is None:
                    self.stuck_start_time = now
                    self.get_logger().warn('[STUCK 감지] 거리가 줄지 않음')
                elif now - self.stuck_start_time >= self.stuck_timeout:
                    self.reversing          = True
                    self.reverse_start_time = now
                    self.stuck_start_time   = None
                    self.path_queue.clear()
                    self.path_queue.append(leader_pos)
                    self.last_saved_pos = leader_pos
                    self.get_logger().warn('[STUCK 확정] 후진 탈출 + 큐 초기화!')
                    twist = Twist()
                    twist.linear.x = self.reverse_speed
                    self.cmd_pub.publish(twist)
                    self.last_dist_to_target = None
                    return
            else:
                self.stuck_start_time = None

        self.last_dist_to_target = dist_to_target_cm

        # ── [8] PD 제어 ──────────────────────────────────────────────
        angle_error = math.degrees(math.atan2(local_y, local_x))

        # 목표가 뒤쪽 → 제자리 회전
        if local_x < 0:
            twist = Twist()
            twist.angular.z = 0.8 if local_y > 0 else -0.8
            self.cmd_pub.publish(twist)
            self.get_logger().info(f'[후면 회전] angle={angle_error:.1f}°')
            return

        # 거리 비례 선속도
        dist_error = dist_to_target_cm - self.target_reach_dist_cm
        v = dist_error * 2.0
        v = max(min(v, 100.0), 0.0)

        # 조향 PD
        deadzone    = 2.0
        base_w_gain = 1.8
        d_gain      = 0.10

        if abs(angle_error) <= deadzone:
            w = 0.0
        else:
            adjusted = angle_error - (deadzone if angle_error > 0 else -deadzone)

            p_term = adjusted * base_w_gain
            d_term = (angle_error - self.last_error_angle) * d_gain
            w = p_term + d_term

        self.last_error_angle = angle_error

        # 각도가 클수록 전진 속도를 줄여서 크게 도는 현상 방지
        if abs(angle_error) > 45:
            v *= 0.02
        elif abs(angle_error) > 25:
            v *= 0.12
        elif abs(angle_error) > 12:
            v *= 0.35

        # 각속도 제한
        max_w = 160.0
        w = max(min(w, max_w), -max_w)

        # Twist 명령 생성
        twist = Twist()
        twist.linear.x  = v * 0.0014
        twist.angular.z = w * 0.065

        self.cmd_pub.publish(twist)

        move_dir = '직진' if abs(w) < 1 else ('좌회전' if w > 0 else '우회전')
        self.get_logger().info(
            f'[{move_dir}] 목표까지:{dist_to_target_cm:.1f}cm | '
            f'각도:{angle_error:.1f}° | v:{v:.1f} | w:{w:.1f} | 큐:{len(self.path_queue)}'
        )

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