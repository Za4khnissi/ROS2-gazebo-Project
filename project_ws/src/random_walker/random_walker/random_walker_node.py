#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point, Transform
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from std_msgs.msg import Bool
import random
import math
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class RandomWalker(Node):
    def __init__(self):
        super().__init__('random_walker')
        
        # Create callback group
        self.callback_group = ReentrantCallbackGroup()
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('goal_delay', 5.0),
                ('min_x', -5.0),
                ('max_x', 5.0),
                ('min_y', -5.0),
                ('max_y', 5.0),
                ('map_frame', 'map'),
                ('robot_frame', 'base_link'),
                ('nav_action_server', '/navigate_to_pose'),
                ('goal_tolerance', 0.5),
                ('min_distance_from_current', 1.0),
                ('max_attempts', 10),
                ('return_to_init', False),
                ('progress_timeout', 30.0)
            ]
        )

        # Get parameters
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.nav_action_server = self.get_parameter('nav_action_server').value
        self.return_to_init = self.get_parameter('return_to_init').value
        self.progress_timeout = self.get_parameter('progress_timeout').value
        
        # Initialize variables
        self.initial_pose = None
        self.prev_goal = Point()
        self.prev_distance = 0.0
        self.last_progress = self.get_clock().now()
        self.goal_blacklist = []
        self.is_active = True
        self._current_timer = None
        
        # Set up tf2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create action client
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.nav_action_server,
            callback_group=self.callback_group)
        
        # Create control subscription
        self.control_sub = self.create_subscription(
            Bool,
            'random_walker/control',
            self.control_callback,
            10
        )
        
        # Wait for navigation action server
        self.get_logger().info(f'Waiting for {self.nav_action_server} action server...')
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Navigation action server not available')
            return

        # Store initial pose if needed
        if self.return_to_init:
            self.store_initial_pose()

        # Start the first plan
        self.make_plan()
        
        self.get_logger().info('Random walker initialized and ready')

    def store_initial_pose(self):
        """Store the initial pose of the robot."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time().to_msg(),
                Duration(seconds=1.0))
            
            self.initial_pose = transform.transform
            self.get_logger().info('Stored initial pose')
        except Exception as e:
            self.get_logger().error(f'Failed to store initial pose: {str(e)}')
            self.return_to_init = False

    def control_callback(self, msg):
        """Handle control messages to start/stop random walking."""
        if msg.data and not self.is_active:
            self.resume()
        elif not msg.data and self.is_active:
            self.stop()

    def get_robot_pose(self):
        """Get the current robot pose in the map frame."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time().to_msg(),
                Duration(seconds=1.0))
            return transform.transform
        except Exception as e:
            self.get_logger().error(f'Failed to get robot pose: {str(e)}')
            return None

    def is_valid_goal(self, x, y):
        """Check if the goal position is valid."""
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return False
        
        dx = x - robot_pose.translation.x
        dy = y - robot_pose.translation.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        min_distance = self.get_parameter('min_distance_from_current').value
        return distance >= min_distance

    def is_goal_blacklisted(self, point, tolerance=0.5):
        """Check if a goal position is blacklisted."""
        for blacklisted_point in self.goal_blacklist:
            dx = point.x - blacklisted_point.x
            dy = point.y - blacklisted_point.y
            if math.sqrt(dx*dx + dy*dy) < tolerance:
                return True
        return False

    def generate_random_goal(self):
        """Generate a random goal position."""
        max_attempts = self.get_parameter('max_attempts').value
        attempts = 0
        
        while attempts < max_attempts:
            x = random.uniform(
                self.get_parameter('min_x').value,
                self.get_parameter('max_x').value
            )
            y = random.uniform(
                self.get_parameter('min_y').value,
                self.get_parameter('max_y').value
            )
            
            point = Point()
            point.x = x
            point.y = y
            
            if self.is_valid_goal(x, y) and not self.is_goal_blacklisted(point):
                return (x, y)
            
            attempts += 1
        
        self.get_logger().warn('Failed to find valid goal position')
        return None

    def make_plan(self):
        """Generate and send a new navigation goal."""
        if not self.is_active:
            return

        goal_position = self.generate_random_goal()
        if goal_position is None:
            self.get_logger().warn('Failed to generate valid goal')
            # Try again after delay
            self._current_timer = self.create_timer(
                self.get_parameter('goal_delay').value,
                self.make_plan
            )
            return

        # Check if we're making progress
        same_goal = (abs(self.prev_goal.x - goal_position[0]) < 0.1 and
                    abs(self.prev_goal.y - goal_position[1]) < 0.1)

        if not same_goal:
            self.last_progress = self.get_clock().now()
            self.send_goal(goal_position)
            self.prev_goal.x = goal_position[0]
            self.prev_goal.y = goal_position[1]
        elif (self.get_clock().now() - self.last_progress).nanoseconds / 1e9 > self.progress_timeout:
            self.get_logger().warn('No progress made, blacklisting current goal')
            point = Point()
            point.x = goal_position[0]
            point.y = goal_position[1]
            self.goal_blacklist.append(point)
            self.make_plan()

    def send_goal(self, position):
        """Send a goal to the navigation stack."""
        x, y = position
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        yaw = random.uniform(-math.pi, math.pi)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.get_logger().info(
            f'Sending goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}'
        )
        
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle the goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            # Try again after delay
            self._current_timer = self.create_timer(
                self.get_parameter('goal_delay').value,
                self.make_plan
            )
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle the goal result."""
        status = future.result().status
        if status == 4:  # Succeeded
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().info(f'Goal failed with status: {status}')
            # Add current goal to blacklist if it failed
            if hasattr(self, 'prev_goal'):
                self.goal_blacklist.append(self.prev_goal)

        # Schedule next goal after delay
        if self._current_timer is not None:
            self.destroy_timer(self._current_timer)
        
        self._current_timer = self.create_timer(
            self.get_parameter('goal_delay').value,
            self.make_plan
        )

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        pass

    def stop(self):
        """Stop the random walker."""
        self.is_active = False
        self.get_logger().info('Random walker stopped')
        
        # Cancel current timer if it exists
        if self._current_timer is not None:
            self.destroy_timer(self._current_timer)
            self._current_timer = None
        
        # Cancel current goal
        self.nav_client.cancel_all_goals()
        
        if self.return_to_init and self.initial_pose is not None:
            self.return_to_initial_pose()

    def resume(self):
        """Resume the random walker."""
        self.is_active = True
        self.get_logger().info('Random walker resumed')
        self.goal_blacklist.clear()
        self.make_plan()

    def return_to_initial_pose(self):
        """Return to the initial pose."""
        if self.initial_pose is None:
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position = self.initial_pose.translation
        goal_msg.pose.pose.orientation = self.initial_pose.rotation
        
        self.get_logger().info('Returning to initial pose')
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
        random_walker.stop()
        executor.shutdown()
        random_walker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()