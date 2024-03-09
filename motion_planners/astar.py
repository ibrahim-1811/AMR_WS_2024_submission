import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSLivelinessPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped, Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.msg import Path
import tf2_ros
import numpy as np
from math import isinf, sin, cos, atan2, floor, ceil
import random
import pandas as pd
import cv2
import copy


class globalplanner:
    def __init__(self, init_robot_pos, goal_pos, map_msg):

        self.init_robot_pos = init_robot_pos
        self.goal_pos = goal_pos
        self.i_state = None
        self.g_state = None
        self.map = map_msg
        self.grids = None
        self.no_of_points = 5
        self.marker_points = []

        


    def neighbours(self, position):
        print('position', position)
        x_cell = floor((position[1]- self.map.info.origin.position.y)/self.map.info.resolution)
        y_cell = floor((position[0]- self.map.info.origin.position.x) / self.map.info.resolution)
        print(x_cell, y_cell)
        return [x_cell, y_cell]

    def mapcell_coord(self, state):
        x = (state[1]*self.map.info.resolution) + self.map.info.origin.position.x
        y = (state[0] * self.map.info.resolution) + self.map.info.origin.position.y
        return (x,y)
    

    def look_for_path(self):
        print('start state')
        self.i_state = self.neighbours(self.init_robot_pos)
        print('end state')
        self.g_state = self.neighbours(self.goal_pos)
        grids = np.array(self.map.data).reshape((self.map.info.height, self.map.info.width))
        path = self.astar()
        path = path[::self.no_of_points]
        print('********PATH**************')
        print(path)
        for cell in path:
            self.marker_points.append(self.mapcell_coord(cell))
        return self.marker_points
    


    def heuristic(self, state):
        return abs(state[0]-self.g_state[0]) + abs(state[1]-self.g_state[1])



    def get_possible_moves(self, state):
        possible_moves = []
        #up
        if self.grids[state[0]][state[1]+1] <10:
            possible_moves.append([state[0],state[1]+1])
        #down
        if self.grids[state[0]][state[1]-1] <10:
            possible_moves.append([state[0],state[1]-1])
        # right
        if self.grids[state[0]+1][state[1]] <10:
            possible_moves.append([state[0]+1,state[1]])
        #left
        if self.grids[state[0]-1][state[1]] <10:
            possible_moves.append([state[0]-1,state[1]])
        #up right
        if self.grids[state[0]+1][state[1]+1] <10:
            possible_moves.append([state[0]+1,state[1]+1])
        # up left
        if self.grids[state[0]-1][state[1]+1] < 10:
            possible_moves.append([state[0]-1,state[1]+1])
        # down right
        if self.grids[state[0] + 1][state[1] - 1] < 10:
            possible_moves.append([state[0] + 1, state[1] - 1])
        # down left
        if self.grids[state[0] - 1][state[1] - 1] < 10:
            possible_moves.append([state[0] - 1, state[1] - 1])
        return possible_moves


    def astar(self):
        open_nodes = []
        open_nodes_f = []
        open_nodes_state = []
        explored_states = []
        g = 0
        path = []
        # start with the initial puzzle config
        init_node = {'state': self.i_state, 'g': 0, 'parent': None}
        open_nodes_f.append(self.heuristic(self.i_state))
        open_nodes.append(init_node)
        open_nodes_state.append(init_node['state'])
        while True:
            # exit if there are no more nodes to explore
            if len(open_nodes) <= 0:
                break
            # get the node with least f value
            current_node = open_nodes.pop(0)
            current_f = open_nodes_f.pop(0)
            # exit if goal reached
            if self.g_state == current_node['state']:
                break
            g = current_node['g'] + 1
            explored_states.append(current_node['state'])
            # get next possible states for robot to move
            possible_states = self.get_possible_moves(current_node['state'])
            possible_states = [state for state in possible_states if state not in explored_states]
            possible_states = [state for state in possible_states if state not in open_nodes_state]
            h_list = [self.heuristic(state) for state in possible_states]
            # f(n) = h(n)+ g(n)
            f_list = [h + g for h in h_list]
            # add all the next possible puzzle configs to open list
            for f, state in zip(f_list, possible_states):
                open_nodes.append({'state': state, 'g': g, 'parent': current_node})
                open_nodes_f.append(f)
                open_nodes_state.append(state)
            # sort based on f value
            open_nodes = pd.Series(data=open_nodes, index=open_nodes_f).sort_index().tolist()
            open_nodes_f = sorted(open_nodes_f)
        # backtrack the path
        path.append(current_node['state'])
        while True:
            parent = current_node['parent']
            if parent is None:
                break
            path.append(parent['state'])
            current_node = parent
        return path
    

class PotentialFieldPathPlanning(Node):
    def __init__(self):
        super().__init__('potential_node')

        # Create subscriptions
        # self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        # self.create_subscription(Odometry, 'odom', self.odom_callback, 10)


        qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_profile)
        self.amcl_pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, qos_profile)
        self.goal_pose_subscriber = self.create_subscription(PoseStamped, '/goal_pose',
                                                             self.goal_pose_callback, 10)
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_publisher = self.create_publisher(Path, '/astar', 10)

        # Create publishers
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

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

    def map_callback(self, msg):
        if self.map_msg is None:
            self.map_msg = msg
            print("*****************got map")
            self.destroy_subscription(self.map_subscriber)

    def amcl_pose_callback(self, msg):
        if self.tf_buffer.can_transform("map", "odom", rclpy.time.Time().to_msg()):
            if self.init_x is None:
                print('got amcl position')
            self.init_x = msg.pose.pose.position.x
            self.init_y = msg.pose.pose.position.y
            odom_to_map_tf = self.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time().to_msg())
            init_pose_wrt_map = do_transform_pose(msg.pose.pose, odom_to_map_tf)
            self.init_x_wrt_map = init_pose_wrt_map.position.x
            self.init_y_wrt_map = init_pose_wrt_map.position.y


