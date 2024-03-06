import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path
import numpy as np
import heapq
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class AStarPlanner:
    def __init__(self):
        self.node = rclpy.create_node('a_star_planner')
        self.map_sub = self.node.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        )
        self.goal_pub = self.node.create_publisher(
            PoseStamped,
            '/goal',
            10
        )
        self.path_pub = self.node.create_publisher(
            Path,
            '/path',
            10
        )
        self.map_data = None

    def map_callback(self, msg):
        self.map_data = msg
        print("got the map yeah!!")

    def heuristic(self, a, b):
        return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

    # def get_map(self):
    #     client = self.node.create_client(GetMap, '/static_map')
    #     while not client.wait_for_service(timeout_sec=1.0):
    #         self.node.get_logger().info('Waiting for the map service...')
    #     request = GetMap.Request()
    #     future = client.call_async(request)
    #     rclpy.spin_until_future_complete(self.node, future)
    #     if future.result() is not None:
    #         return future.result().map

    def a_star_search(self, start, goal):
        if not self.map_data:
            return None

        map_info = self.map_data.info
        map_data = np.array(self.map_data.data).reshape((map_info.height, map_info.width))

        queue = [(0, start)]
        visited = set()
        came_from = {}

        while queue:
            current_cost, current_pos = heapq.heappop(queue)

            if current_pos == goal:
                path = []
                while current_pos in came_from:
                    path.append(current_pos)
                    current_pos = came_from[current_pos]
                path.append(start)
                path.reverse()
                return path

            if current_pos in visited:
                continue

            visited.add(current_pos)

            for neighbor in self.get_neighbors(current_pos):
                if neighbor in visited:
                    continue

                new_cost = current_cost + 1
                heapq.heappush(queue, (new_cost + self.heuristic(neighbor, goal), neighbor))
                came_from[neighbor] = current_pos

        return None

    def get_neighbors(self, pos):
        x, y = pos
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                new_x, new_y = x + dx, y + dy
                if 0 <= new_x < self.map_data.info.width and 0 <= new_y < self.map_data.info.height and \
                        self.map_data.data[new_y * self.map_data.info.width + new_x] == 0:
                    neighbors.append((new_x, new_y))
        return neighbors

    def plan_path(self, start, goal):
        path = self.a_star_search(start, goal)

        if path:
            self.publish_path(path)
            self.publish_goal(path[-1])
            print("path is", path)

    def publish_goal(self, goal):
        pose = PoseStamped()
        pose.pose.position.x = goal[0]
        pose.pose.position.y = goal[1]
        pose.header.frame_id = 'map'
        self.goal_pub.publish(pose)

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for pos in path:
            pose = PoseStamped()
            pose.pose.position.x = pos[0]
            pose.pose.position.y = pos[1]
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)
        print("path",path_msg)

def main():
    rclpy.init()
    a_star_planner = AStarPlanner()

    start = (0, 0)
    goal = (5, 5)

    while rclpy.ok():
        a_star_planner.plan_path(start, goal)
        rclpy.spin_once(a_star_planner.node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
