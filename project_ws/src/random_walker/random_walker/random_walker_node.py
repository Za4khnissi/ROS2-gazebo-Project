#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan
import random
import math
import time


class RandomWalker(Node):
    """
    A ROS2 node that implements random walking behavior using Nav2.
    The robot navigates to random positions while avoiding obstacles
    and can return to its initial pose at the end of the mission.
    """

    def __init__(self):
        super().__init__('random_walker')

        # Create a timer for periodic goal planning
        self.timer = self.create_timer(5.0, self.make_plan)

        # Create callback group for thread-safe subscriptions and clients
        self.callback_group = ReentrantCallbackGroup()

        # Subscribe to the LiDAR topic for obstacle detection
        self.obstacle_detected = False
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/limo_105_3/scan',
            self.lidar_callback,
            10
        )

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('goal_delay', 5.0),  # Timeout for navigation
                ('min_x', -5.0),
                ('max_x', 5.0),
                ('min_y', -5.0),
                ('max_y', 5.0),
                ('map_frame', 'map'),
                ('pose_topic', 'amcl_pose'),
                ('nav_action_server', '/navigate_to_pose'),
                ('min_distance_from_current', 1.0),
                ('max_attempts', 10),
                ('return_to_init', False),
                ('progress_timeout', 30.0),
            ]
        )

        # Fetch parameters
        self.map_frame = self.get_parameter('map_frame').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.nav_action_server = self.get_parameter('nav_action_server').value
        self.return_to_init = self.get_parameter('return_to_init').value
        self.progress_timeout = self.get_parameter('progress_timeout').value
        self.min_x = self.get_parameter('min_x').value
        self.max_x = self.get_parameter('max_x').value
        self.min_y = self.get_parameter('min_y').value
        self.max_y = self.get_parameter('max_y').value
        self.min_distance_from_current = self.get_parameter('min_distance_from_current').value
        self.max_attempts = self.get_parameter('max_attempts').value

        # Initialize variables
        self.initial_pose = None
        self.prev_goal = Point()
        self.last_progress = self.get_clock().now()
        self.goal_blacklist = []
        self.is_active = False
        self.current_pose = None
        self._navigation_active = False
        self._goal_handle = None

        # Action client for Nav2 navigation
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.nav_action_server,
            callback_group=self.callback_group
        )

        # Subscribe to pose updates
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.pose_topic,
            self.pose_callback,
            10
        )

        # Wait for navigation action server
        self.get_logger().info(f'Waiting for {self.nav_action_server} action server...')
        if not self.nav_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('Navigation action server not available')
            return

        # Wait for initial pose
        self.get_logger().info('Waiting for initial pose...')
        while self.current_pose is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=1.0)

        # Store initial pose if required
        if self.return_to_init:
            self.store_initial_pose()

        self.get_logger().info('Random walker initialized and ready')

    def lidar_callback(self, msg):
        """Check for nearby obstacles using LiDAR data."""
        min_distance = min([distance for distance in msg.ranges if distance > 0.01], default=10.0)
        self.obstacle_detected = min_distance < 0.5
        if self.obstacle_detected:
            self.get_logger().warn(f"Obstacle detected at {min_distance:.2f} meters! Pausing.")

    def pose_callback(self, msg):
        """Callback for AMCL pose updates."""
        self.current_pose = msg.pose.pose

    def store_initial_pose(self):
        """Store the robot's initial pose."""
        if self.current_pose is not None:
            self.initial_pose = self.current_pose
            self.get_logger().info('Stored initial pose')
        else:
            self.get_logger().error('Failed to store initial pose: No pose available')
            self.return_to_init = False

    def make_plan(self):
        """Generate and send a new navigation goal."""
        if self.obstacle_detected:
            self.get_logger().warn("Obstacle detected, not sending new goal.")
            return

        goal_position = self.generate_random_goal()
        if goal_position is None:
            self.get_logger().warn('Failed to generate valid goal')
            return

        self.send_goal(goal_position)

    def generate_random_goal(self):
        """Generate a random valid navigation goal."""
        for _ in range(self.max_attempts):
            x = random.uniform(self.min_x, self.max_x)
            y = random.uniform(self.min_y, self.max_y)
            if self.is_valid_goal(x, y):
                return (x, y)

        self.get_logger().warn('Failed to find a valid random goal.')
        return None

    def is_valid_goal(self, x, y):
        """Validate the goal's distance from the current position."""
        if self.current_pose is None:
            return False

        dx = x - self.current_pose.position.x
        dy = y - self.current_pose.position.y
        distance = math.sqrt(dx * dx + dy * dy)
        return distance >= self.min_distance_from_current

    def send_goal(self, position):
        """Send a navigation goal to Nav2."""
        x, y = position
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(random.uniform(-math.pi, math.pi) / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(random.uniform(-math.pi, math.pi) / 2.0)

        self.get_logger().info(f'Sending goal to x={x:.2f}, y={y:.2f}')
        self._navigation_active = True
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle response from Nav2 after sending a goal."""
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by Nav2')
            self._navigation_active = False
        else:
            self.get_logger().info('Goal accepted by Nav2')

    def feedback_callback(self, feedback_msg):
        """Handle feedback during navigation."""
        pass

    def stop(self):
        """Stop the random walker."""
        self.is_active = False
        self.get_logger().info('Random walker stopped')

    def return_to_initial_pose(self):
        """Return the robot to its initial pose."""
        if self.initial_pose is None:
            self.get_logger().warn('No initial pose stored. Cannot return to base.')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose = self.initial_pose

        self.get_logger().info('Returning to initial pose...')
        self.nav_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    random_walker = RandomWalker()
    executor = MultiThreadedExecutor()
    executor.add_node(random_walker)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        random_walker.return_to_initial_pose()
        random_walker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
