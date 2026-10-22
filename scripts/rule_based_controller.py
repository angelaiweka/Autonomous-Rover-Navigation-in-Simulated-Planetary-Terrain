import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
GOAL_X = 9.0
GOAL_Y = 0.0
ATTRACTIVE_GAIN = 1.0
REPULSIVE_GAIN = 2.0
REPULSIVE_RADIUS = 0.6
TANGENTIAL_GAIN = 1.2
MAX_LINEAR_SPEED = 0.2
MAX_ANGULAR_SPEED = 1.0
STALL_CHECK_WINDOW = 30
STALL_DISTANCE_THRESHOLD = 0.05
MAX_WALL_FOLLOW_STEPS = 300
class RuleBasedController(Node):
    def __init__(self):
        super().__init__('rule_based_controller')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.ranges = []
        self.angle_min = 0.0
        self.angle_increment = 0.0
        self.mode = 'seek'
        self.position_history = []
        self.follow_start_distance = None
        self.wall_follow_steps = 0
        self.follow_direction = 1.0
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
    def scan_callback(self, msg):
        self.ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
    def control_loop(self):
        dx = GOAL_X - self.x
        dy = GOAL_Y - self.y
        distance_to_goal = math.sqrt(dx * dx + dy * dy)
        if distance_to_goal < 0.2:
            self.cmd_pub.publish(TwistStamped())
            return
        self.position_history.append((self.x, self.y))
        if len(self.position_history) > STALL_CHECK_WINDOW:
            self.position_history.pop(0)
        if self.mode == 'seek' and len(self.position_history) == STALL_CHECK_WINDOW:
            oldest_x, oldest_y = self.position_history[0]
            moved = math.sqrt((self.x - oldest_x) ** 2 + (self.y - oldest_y) ** 2)
            if moved < STALL_DISTANCE_THRESHOLD:
                self.mode = 'wall_follow'
                self.follow_start_distance = distance_to_goal
                self.wall_follow_steps = 0
                goal_angle = math.atan2(dy, dx)
                plus_diff = abs(math.atan2(math.sin(goal_angle - (self.yaw + math.pi / 2)), math.cos(goal_angle - (self.yaw + math.pi / 2))))
                minus_diff = abs(math.atan2(math.sin(goal_angle - (self.yaw - math.pi / 2)), math.cos(goal_angle - (self.yaw - math.pi / 2))))
                self.follow_direction = 1.0 if plus_diff < minus_diff else -1.0
        attractive_x = ATTRACTIVE_GAIN * dx
        attractive_y = ATTRACTIVE_GAIN * dy
        repulsive_x = 0.0
        repulsive_y = 0.0
        tangent_sign = self.follow_direction if self.mode == 'wall_follow' else 1.0
        for i, r in enumerate(self.ranges):
            if r < REPULSIVE_RADIUS and r > 0.01:
                angle = self.angle_min + i * self.angle_increment + self.yaw
                strength = REPULSIVE_GAIN * (1.0 / r - 1.0 / REPULSIVE_RADIUS) * (1.0 / (r * r))
                repulsive_x -= strength * math.cos(angle)
                repulsive_y -= strength * math.sin(angle)
                tangent_angle = angle + tangent_sign * (math.pi / 2)
                repulsive_x += TANGENTIAL_GAIN * strength * math.cos(tangent_angle)
                repulsive_y += TANGENTIAL_GAIN * strength * math.sin(tangent_angle)
        if self.mode == 'wall_follow':
            self.wall_follow_steps += 1
            has_wall = repulsive_x != 0.0 or repulsive_y != 0.0
            made_progress = distance_to_goal <= self.follow_start_distance
            if (not has_wall and made_progress) or self.wall_follow_steps > MAX_WALL_FOLLOW_STEPS:
                self.mode = 'seek'
                self.position_history = []
                total_x = attractive_x + repulsive_x
                total_y = attractive_y + repulsive_y
            else:
                total_x = repulsive_x
                total_y = repulsive_y
        else:
            total_x = attractive_x + repulsive_x
            total_y = attractive_y + repulsive_y
        target_angle = math.atan2(total_y, total_x)
        angle_error = target_angle - self.yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = min(MAX_LINEAR_SPEED, math.sqrt(total_x**2 + total_y**2) * 0.3)
        cmd.twist.angular.z = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, angle_error * 2.0))
        self.cmd_pub.publish(cmd)
        self.get_logger().info(f'mode={self.mode} pos=({self.x:.2f},{self.y:.2f}) dist={distance_to_goal:.2f} steps={self.wall_follow_steps}')
def main(args=None):
    rclpy.init(args=args)
    node = RuleBasedController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
