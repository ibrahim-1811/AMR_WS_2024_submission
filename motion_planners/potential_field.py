import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np

class PotentialFieldPathPlanning(Node):
    def __init__(self):
        super().__init__('potential_node')

        # Create subscriptions
        self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Create publishers
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)

        # Constants for potential field method
        self.k_a = 1.0  # Attractive force constant
        self.k_r = 0.9  # Repulsive force constant
        self.rho_0 = 1.0 # Threshold distance for repulsive force

        self.position_x = None
        self.position_y = None
        self.orientation_z = None
        self.orientation_w = None
        self.max_linear_velocity = 1.0 # Adjust based on your robot's capabilities
        self.max_angular_velocity = 1.0  # Adjust based on your robot's capabilities

        # Initialize goal pose
        self.goal_pose = PoseStamped()

        # Initialize obstacles
        self.obstacles = []

    def laser_callback(self, msg):
        total_velocity = Twist()

        obstacles = []

        for i, value in enumerate(msg.ranges):
            if value < self.rho_0:
                angle = msg.angle_min + i * msg.angle_increment
                obstacle = {
                    'x0': value * np.cos(angle),
                    'y0': value * np.sin(angle)
                }
                
                obstacles.append(obstacle)

        self.obstacles = obstacles

        repulsive_velocity = self.repulsive_vel(obstacles)
        attractive_velocity = self.attractive_vel()

        total_velocity.linear.x = repulsive_velocity.linear.x + attractive_velocity.linear.x
        total_velocity.linear.y = repulsive_velocity.linear.y #+ attractive_velocity.linear.y
        total_velocity.angular.z = repulsive_velocity.angular.z + attractive_velocity.angular.z

        total_velocity.linear.x = min(total_velocity.linear.x, self.max_linear_velocity)
        total_velocity.linear.y = min(total_velocity.linear.y, self.max_linear_velocity)
        total_velocity.angular.z = min(total_velocity.angular.z, self.max_angular_velocity)

        self.vel_publisher.publish(total_velocity)

    def repulsive_vel(self, obstacles):
        repulsive_velocity = Twist()
        q = np.array([0, 0])

        for obstacle in obstacles:
            q0 = np.array([obstacle['x0'], obstacle['y0']])
            distance = np.linalg.norm(q - q0)
            normalized_vector = (q - q0) / distance
            self.get_logger().warn("Robot is in a repulsive field.")

            repulsive_velocity.linear.x += (self.k_r * ((1 / distance) - (1 / self.rho_0)) * (1 / distance**2) * normalized_vector[0])
            repulsive_velocity.linear.y += (self.k_r * ((1 / distance) - (1 / self.rho_0)) * (1 / distance**2) * normalized_vector[1])

        return repulsive_velocity

    def attractive_vel(self):
        attractive_velocity = Twist()

        if self.position_x is None or self.position_y is None:
            self.get_logger().warn("Position values are None. Skipping attractive velocity calculation.")
            return attractive_velocity

        q = np.array([self.position_x, self.position_y])
        q_goal = np.array([self.goal_pose.pose.position.x, self.goal_pose.pose.position.y])

        delta_x = q_goal[0] - q[0]
        delta_y = q_goal[1] - q[1]

        desired_heading = np.arctan2(delta_y, delta_x)
        current_heading = np.arctan2(2.0 * (self.orientation_w * self.orientation_z),
                                    1.0 - 2.0 * (self.orientation_z ** 2))

        angle_difference = desired_heading - current_heading
        angle_difference = np.arctan2(np.sin(angle_difference), np.cos(angle_difference))
        self.get_logger().warn("Robot is in a attarctive field.")
        attractive_velocity.linear.x = -self.k_a * (q[0] - q_goal[0]) / np.linalg.norm(q - q_goal)
        attractive_velocity.linear.y = -self.k_a * (q[1] - q_goal[1]) / np.linalg.norm(q - q_goal)
        attractive_velocity.angular.z = self.k_a * angle_difference

        # Scale the attractive velocity when repulsive fields are not detected
        if not self.in_repulsive_field(q):
            
            scale_factor = 1.2  # Adjust the scale factor based on your preferences
            attractive_velocity.linear.x *= scale_factor
            attractive_velocity.linear.y *= scale_factor
            attractive_velocity.angular.z *= scale_factor

        # Publish the attractive velocity
        self.vel_publisher.publish(attractive_velocity)

        if np.linalg.norm(q - q_goal) <= 0.1:
            attractive_velocity.linear.x = 0.0
            attractive_velocity.linear.y = 0.0
            attractive_velocity.angular.z = 0.0
            self.get_logger().info(f"goal is reached")
            self.vel_publisher.publish(attractive_velocity)
            rclpy.shutdown()


        return attractive_velocity

    def in_repulsive_field(self, q):
        repulsive_threshold = 1.2
        return any(np.linalg.norm(q - np.array([obstacle['x0'], obstacle['y0']])) < repulsive_threshold for obstacle in self.obstacles)

    def odom_callback(self, msg):
        current_pose = msg
        self.position_x = current_pose.pose.pose.position.x
        self.position_y = current_pose.pose.pose.position.y
        self.orientation_z = current_pose.pose.pose.orientation.z
        self.orientation_w = current_pose.pose.pose.orientation.w
        # self.get_logger().info(f"Position: ({self.position_x}, {self.position_y})")


    def send_goal(self, goal_x, goal_y):
        self.goal_pose.header.frame_id = 'base_link'
        self.goal_pose.pose.position.x = goal_x
        self.goal_pose.pose.position.y = goal_y
        self.goal_pose.pose.orientation.z = 1.0
        self.goal_pose.pose.orientation.w = 1.0
        self.goal_publisher.publish(self.goal_pose)
        self.get_logger().info(f"Goal sent: x={goal_x}, y={goal_y}")

def main(args=None):
    rclpy.init(args=args)
    potential_field_planner = PotentialFieldPathPlanning()

    goal_x, goal_y = 10.0, 0.0
    potential_field_planner.send_goal(goal_x, goal_y)

    rclpy.spin(potential_field_planner, executor=rclpy.executors.MultiThreadedExecutor(num_threads=4))
    potential_field_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()