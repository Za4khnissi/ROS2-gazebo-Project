#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point, Transform, PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from std_msgs.msg import Bool
import random
import math
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class RandomWalker(Node):
    """
    A ROS2 node that implements random walking behavior using Nav2.
    The robot will navigate to random positions while avoiding blacklisted areas.
    """
    def __init__(self):
        super().__init__('random_walker')
        
        # Create callback group
        self.callback_group = ReentrantCallbackGroup()
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('goal_delay', 5.0),  # Used as timeout for navigation
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
                ('progress_timeout', 30.0)
            ]
        )

        # Get parameters
        self.map_frame = self.get_parameter('map_frame').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.nav_action_server = self.get_parameter('nav_action_server').value
        self.return_to_init = self.get_parameter('return_to_init').value
        self.progress_timeout = self.get_parameter('progress_timeout').value
        self.goal_delay = self.get_parameter('goal_delay').value
        self.min_x = self.get_parameter('min_x').value
        self.max_x = self.get_parameter('max_x').value
        self.min_y = self.get_parameter('min_y').value
        self.max_y = self.get_parameter('max_y').value
        self.min_distance_from_current = self.get_parameter('min_distance_from_current').value
        self.max_attempts = self.get_parameter('max_attempts').value
        
        # Initialize variables
        self.initial_pose = None
        self.prev_goal = Point()
        self.prev_distance = 0.0
        self.last_progress = self.get_clock().now()
        self.goal_blacklist = []
        self.is_active = True
        self._navigation_active = False
        self._timeout_timer = None
        self.current_pose = None
        self._goal_handle = None

        self.current_direction = None  # Current movement direction (angle)
        self.direction_weight = 0.7    # How much to favor current direction (0-1)
        self.direction_variance = math.pi/4  # Maximum angle deviation (45 degrees)
        self.consecutive_failures = 0  # Track failures to know when to change direction
        self.max_failures_before_direction_change = 3  # When to significantly change direction
        
        # Create action client
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.nav_action_server,
            callback_group=self.callback_group)
        
        # Create subscriptions
        self.control_sub = self.create_subscription(
            Bool,
            'random_walker/control',
            self.control_callback,
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.pose_topic,
            self.pose_callback,
            10
        )
        
        # Wait for navigation action server
        self.get_logger().info(f'Waiting for {self.nav_action_server} action server...')
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Navigation action server not available')
            return

        # Wait for initial pose
        self.get_logger().info('Waiting for initial pose...')
        while self.current_pose is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=1.0)

        # Store initial pose if needed
        if self.return_to_init:
            self.store_initial_pose()

        # Start the first plan
        self.make_plan()
        
        self.get_logger().info('Random walker initialized and ready')

    def pose_callback(self, msg):
        """Callback for the AMCL pose updates."""
        self.current_pose = msg.pose.pose

    def store_initial_pose(self):
        """Store the initial pose of the robot for potential return."""
        if self.current_pose is not None:
            self.initial_pose = self.current_pose
            self.get_logger().info('Stored initial pose')
        else:
            self.get_logger().error('Failed to store initial pose: No pose available')
            self.return_to_init = False

    def start_timeout_timer(self):
        """Start or restart the timeout timer for navigation goals."""
        if self._timeout_timer is not None:
            self.destroy_timer(self._timeout_timer)
        
        self._timeout_timer = self.create_timer(
            self.goal_delay,
            self.timeout_callback
        )

    def timeout_callback(self):
        """Handle navigation timeout by cancelling current goal and sending a new one."""
        if self.is_active and self._navigation_active:
            self.get_logger().info('Timeout reached, sending new goal')
            # Instead of cancel_all_goals, we'll just mark as inactive and plan new goal
            self._navigation_active = False
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
            self.make_plan()

    def control_callback(self, msg):
        """Handle control messages to start/stop random walking."""
        if msg.data and not self.is_active:
            self.resume()
        elif not msg.data and self.is_active:
            self.stop()

    def get_robot_pose(self):
        """Get the current robot pose."""
        return self.current_pose

    def is_valid_goal(self, x, y, cached_robot_pose=None):
        """Check if the goal position is valid based on distance from current position."""
        robot_pose = self.get_robot_pose() #cached_robot_pose if cached_robot_pose is not None else self.get_robot_pose()
        if robot_pose is None:
            return False, None
        
        dx = x - robot_pose.position.x
        dy = y - robot_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        return distance >= self.min_distance_from_current, robot_pose

    def is_goal_blacklisted(self, point, tolerance=0.5):
        """Check if a goal position is in the blacklist."""
        for blacklisted_point in self.goal_blacklist:
            dx = point.x - blacklisted_point.x
            dy = point.y - blacklisted_point.y
            if math.sqrt(dx*dx + dy*dy) < tolerance:
                return True
        return False
    
    def update_direction(self, success=True):
        """Update movement direction based on success/failure."""
        if success:
            # If successful, maintain current direction
            self.consecutive_failures = 0
        else:
            self.consecutive_failures += 1
            if self.consecutive_failures >= self.max_failures_before_direction_change:
                # Change direction significantly after multiple failures
                self.current_direction = random.uniform(-math.pi, math.pi)
                self.consecutive_failures = 0

    def generate_random_goal(self):
        """Generate a goal position using momentum-based direction."""
        max_attempts = self.max_attempts
        attempts = 0
        cached_robot_pose = None
        
        # Initialize direction if none exists
        if self.current_direction is None:
            self.current_direction = random.uniform(-math.pi, math.pi)
        
        while attempts < max_attempts:
            # Generate angle with bias towards current direction
            if random.random() < self.direction_weight:
                # Use current direction with some variance
                angle = self.current_direction + random.uniform(
                    -self.direction_variance, 
                    self.direction_variance
                )
            else:
                # Sometimes choose completely random direction
                angle = random.uniform(-math.pi, math.pi)
            
            # Generate distance
            distance = random.uniform(
                self.min_distance_from_current,
                min(5.0, self.max_x - self.min_x)  # Use reasonable maximum distance
            )
            
            # Calculate position
            robot_pose = self.get_robot_pose()
            if robot_pose is None:
                return None
                
            x = robot_pose.position.x + distance * math.cos(angle)
            y = robot_pose.position.y + distance * math.sin(angle)
            
            # Ensure within bounds
            x = max(self.min_x, min(self.max_x, x))
            y = max(self.min_y, min(self.max_y, y))
            
            point = Point()
            point.x = x
            point.y = y
            
            isValidGoal, robot_pose = self.is_valid_goal(x, y, cached_robot_pose)
            
            if isValidGoal and not self.is_goal_blacklisted(point):
                # Update direction based on chosen point
                self.current_direction = math.atan2(
                    y - robot_pose.position.y,
                    x - robot_pose.position.x
                )
                return (x, y)
            elif robot_pose is not None:
                cached_robot_pose = robot_pose
            
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
        """Send a navigation goal to Nav2."""
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
        
        self._navigation_active = True
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        self.start_timeout_timer()

    def goal_response_callback(self, future):
        """Handle the goal response from Nav2."""
        goal_handle = future.result()
        self._goal_handle = goal_handle  # Store the goal handle
        
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self._navigation_active = False
            if self._timeout_timer is not None:
                self.destroy_timer(self._timeout_timer)
            self.make_plan()
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle the navigation result and initiate the next goal."""
        status = future.result().status
        self._navigation_active = False
        
        if status == 4:  # Succeeded
            self.get_logger().info('Goal succeeded!')
            self.update_direction(success=True)
        else:
            self.get_logger().info(f'Goal failed with status: {status}')
            self.update_direction(success=False)
            if hasattr(self, 'prev_goal'):
                self.goal_blacklist.append(self.prev_goal)

        if self._timeout_timer is not None:
            self.destroy_timer(self._timeout_timer)
            self._timeout_timer = None

        self.make_plan()

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        pass

    def stop(self):
        """Stop the random walker and clean up."""
        self.is_active = False
        self._navigation_active = False
        self.get_logger().info('Random walker stopped')
        
        if self._timeout_timer is not None:
            self.destroy_timer(self._timeout_timer)
            self._timeout_timer = None
        
        # Cancel current goal if it exists
        if hasattr(self, '_goal_handle') and self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        
        if self.return_to_init and self.initial_pose is not None:
            self.return_to_initial_pose()

    def resume(self):
        """Resume random walking."""
        self.is_active = True
        self.get_logger().info('Random walker resumed')
        self.goal_blacklist.clear()
        self.make_plan()

    def return_to_initial_pose(self):
        """Return to the initial pose if it was stored."""
        if self.initial_pose is None:
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose = self.initial_pose
        
        self.get_logger().info('Returning to initial pose')
        self.nav_client.send_goal_async(goal_msg)

def main(args=None):
    """Main function."""
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