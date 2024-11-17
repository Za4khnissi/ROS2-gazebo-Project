#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import math
import random
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class RandomWalker(Node):
    """
    Optimized Random Walker Node with Nav2 and SLAM integration.
    Incorporates frontier-based exploration and obstacle avoidance.
    """
    def __init__(self):
        super().__init__('random_walker')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('min_x', -10.0),
                ('max_x', 10.0),
                ('min_y', -10.0),
                ('max_y', 10.0),
                ('map_frame', 'map'),
                ('pose_topic', 'amcl_pose'),
                ('nav_action_server', '/navigate_to_pose'),
                ('min_distance_from_current', 0.5),
                ('goal_tolerance', 0.5),
                ('progress_timeout', 30.0),
                ('return_to_init', True),
            ]
        )

        # Get parameters
        self.min_x = self.get_parameter('min_x').value
        self.max_x = self.get_parameter('max_x').value
        self.min_y = self.get_parameter('min_y').value
        self.max_y = self.get_parameter('max_y').value
        self.map_frame = self.get_parameter('map_frame').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.nav_action_server = self.get_parameter('nav_action_server').value
        self.min_distance_from_current = self.get_parameter('min_distance_from_current').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.progress_timeout = self.get_parameter('progress_timeout').value
        self.return_to_init = self.get_parameter('return_to_init').value

        # Initialize variables
        self.current_pose = None
        self.initial_pose = None
        self.map_data = None
        self.goal_blacklist = []
        self._navigation_active = False
        self.last_progress_time = self.get_clock().now()
        self.callback_group = ReentrantCallbackGroup()

        # Action client for navigation
        self.nav_client = ActionClient(
            self, NavigateToPose, self.nav_action_server, callback_group=self.callback_group
        )

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.pose_topic, self.pose_callback, 10
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10
        )
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10
        )

        # Wait for Nav2 action server
        self.get_logger().info(f'Waiting for {self.nav_action_server}...')
        self.nav_client.wait_for_server()

        self.get_logger().info('Random walker initialized and ready')

    def pose_callback(self, msg):
        """Callback for AMCL pose."""
        self.current_pose = msg.pose.pose
        if self.return_to_init and self.initial_pose is None:
            self.initial_pose = self.current_pose

    def map_callback(self, msg):
        """Callback for map data."""
        self.map_data = msg

    def lidar_callback(self, msg):
        """Callback for LiDAR data."""
        if any(distance < 0.5 for distance in msg.ranges):
            self.get_logger().info("Obstacle detected! Replanning...")
            self.plan_new_goal()

    def is_valid_goal(self, goal_x, goal_y):
        """Validate goal by ensuring it's far enough from the current position."""
        if self.current_pose is None:
            return False

        dx = goal_x - self.current_pose.position.x
        dy = goal_y - self.current_pose.position.y
        distance = math.sqrt(dx**2 + dy**2)

        return distance >= self.min_distance_from_current

    def find_frontiers(self):
        """Identify frontiers from the map."""
        if self.map_data is None:
            return []

        data = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width)
        )
        unexplored = (data == -1).astype(np.uint8)
        frontiers = []

        for x in range(1, data.shape[0] - 1):
            for y in range(1, data.shape[1] - 1):
                if unexplored[x, y] == 1 and np.any(data[x - 1:x + 2, y - 1:y + 2] == 0):
                    frontiers.append((x, y))

        return frontiers

    def select_closest_frontier(self, frontiers):
        """Select the closest frontier to explore."""
        if self.current_pose is None or not frontiers:
            return None

        current_x = int(
            (self.current_pose.position.x - self.map_data.info.origin.position.x)
            / self.map_data.info.resolution
        )
        current_y = int(
            (self.current_pose.position.y - self.map_data.info.origin.position.y)
            / self.map_data.info.resolution
        )

        closest_frontier = min(
            frontiers,
            key=lambda f: math.sqrt((f[0] - current_x) ** 2 + (f[1] - current_y) ** 2),
        )
        return closest_frontier

    def plan_new_goal(self):
        """Generate a new goal for exploration."""
        if self.map_data is None:
            self.get_logger().warning('No map data available')
            return

        frontiers = self.find_frontiers()
        closest_frontier = self.select_closest_frontier(frontiers)

        if closest_frontier is None:
            self.get_logger().info('No valid frontier found. Exploration complete.')
            return

        goal_x = (
            closest_frontier[0] * self.map_data.info.resolution
            + self.map_data.info.origin.position.x
        )
        goal_y = (
            closest_frontier[1] * self.map_data.info.resolution
            + self.map_data.info.origin.position.y
        )

        if self.is_valid_goal(goal_x, goal_y):
            self.send_goal(goal_x, goal_y)
        else:
            self.get_logger().warning('Generated goal is invalid.')

    def send_goal(self, x, y):
        """Send a navigation goal to Nav2."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Sending goal: x={x:.2f}, y={y:.2f}')
        self.nav_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle response from Nav2."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warning('Goal was rejected.')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        goal_handle.get_result_async().add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """Handle result of navigation."""
        result = future.result().result
        if result:
            self.get_logger().info('Goal reached successfully.')
            self.last_progress_time = self.get_clock().now()
            self.plan_new_goal()
        else:
            self.get_logger().warning('Failed to reach goal, replanning...')
            self.plan_new_goal()


def main(args=None):
    rclpy.init(args=args)
    node = RandomWalker()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Random Walker...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
