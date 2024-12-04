#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point, Transform, PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from std_msgs.msg import Bool, Int8
import random
import math
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist


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
                ('progress_timeout', 30.0),
                ('global_costmap_topic', '/limo_105_3/global_costmap/costmap'),
                ('local_costmap_topic', '/limo_105_3/local_costmap/costmap'),
                ('costmap_threshold', 80)  # Threshold for considering a point occupied
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


        self.global_costmap_topic = self.get_parameter('global_costmap_topic').value
        self.local_costmap_topic = self.get_parameter('local_costmap_topic').value
        self.costmap_threshold = self.get_parameter('costmap_threshold').value
        
        # Initialize costmap data
        self.global_costmap = None
        self.local_costmap = None

        self.global_costmap_sub = self.create_subscription(
        OccupancyGrid,
        self.global_costmap_topic,
        self.global_costmap_callback,
        1
        )
    
        self.local_costmap_sub = self.create_subscription(
            OccupancyGrid,
            self.local_costmap_topic,
            self.local_costmap_callback,
            1
        )
        
        # Initialize variables
        self.initial_pose = None
        self.prev_goal = Point()
        self.prev_distance = 0.0
        self.last_progress = self.get_clock().now()
        self.goal_blacklist = []
        self.is_active = False # Changed to match explore lite default behavior
        self._navigation_active = False
        self._timeout_timer = None
        self.current_pose = None
        self._goal_handle = None

        self.current_direction = None  # Current movement direction (angle)
        self.direction_weight = 1    # How much to favor current direction (0-1)
        self.direction_variance = math.pi/6  # Maximum angle deviation (45 degrees)
        self.consecutive_failures = 0  # Track failures to know when to change direction
        self.max_failures_before_direction_change = 3  # When to significantly change direction
        
        # Create action client
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.nav_action_server,
            callback_group=self.callback_group)
        
    
        # Create subscriptions
        self.resume_sub = self.create_subscription(
            Int8,
            'explore/resume',
            self.resume_callback,
            10,
            
        )
        
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.pose_topic,
            self.pose_callback,
            10
        )

        namespace = self.get_namespace()
        self.vel_pub = self.create_publisher(
            Twist,
            f'{namespace}/cmd_vel',
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

        self.get_logger().info('Waiting for costmap data...')
        while (self.global_costmap is None or self.local_costmap is None) and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=1.0)

        # Store initial pose if needed
        if self.return_to_init:
            self.store_initial_pose()

        self.get_logger().info('Random walker initialized and ready')
        #self.resume()

    def pose_callback(self, msg):
        """Callback for the AMCL pose updates."""
        self.current_pose = msg.pose.pose


    def global_costmap_callback(self, msg):
        """Callback for global costmap updates."""
        self.global_costmap = msg

    def local_costmap_callback(self, msg):
        """Callback for local costmap updates."""
        self.local_costmap = msg

    def is_point_valid_in_costmap(self, x, y, costmap):
        """Check if a point is valid in the given costmap."""
        if costmap is None:
            return True  # If no costmap data, assume valid
            
        # Convert world coordinates to costmap coordinates
        mx = int((x - costmap.info.origin.position.x) / costmap.info.resolution)
        my = int((y - costmap.info.origin.position.y) / costmap.info.resolution)
        
        # Check if point is within costmap bounds
        if (mx < 0 or mx >= costmap.info.width or 
            my < 0 or my >= costmap.info.height):
            return False
        
        # Get cost value
        index = my * costmap.info.width + mx
        cost = costmap.data[index]
        
        # Return True if cost is below threshold
        return cost < self.costmap_threshold

    def resume_callback(self, msg):
        """Handle resume/stop messages."""
        if msg.data == 1:  # Resume exploration
            self.resume()
        elif msg.data == 2:  # Stop exploration
            self.stop(force_return=False)
        elif msg.data == 3:  # Stop exploration and force return
            self.stop(force_return=True)
        else:
            self.get_logger().warn('Invalid command received on explore/resume topic. Valid values are: 1 (resume), 2 (stop), 3 (stop and return)')




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

    def get_robot_pose(self):
        """Get the current robot pose."""
        return self.current_pose

    def is_valid_goal(self, x, y, cached_robot_pose=None):
        """Check if the goal position is valid based on distance and costmaps."""
        # First check distance
        robot_pose = cached_robot_pose if cached_robot_pose is not None else self.get_robot_pose()
        if robot_pose is None:
            return False, None
        
        dx = x - robot_pose.position.x
        dy = y - robot_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < self.min_distance_from_current:
            return False, robot_pose
        
        # Check global costmap
        if not self.is_point_valid_in_costmap(x, y, self.global_costmap):
            return False, robot_pose
        
        # Check local costmap
        if not self.is_point_valid_in_costmap(x, y, self.local_costmap):
            return False, robot_pose
        
        return True, robot_pose

    def update_direction(self, success=True):
        """Update movement direction based on success/failure."""
        if success:
            # If successful, gradually rotate direction to explore new areas
            self.consecutive_failures = 0
            if self.current_direction is not None:
                # Rotate direction by a small amount for exploration
                self.current_direction += math.pi/4  # 45-degree rotation
                self.current_direction = math.atan2(math.sin(self.current_direction), 
                                                math.cos(self.current_direction))
        else:
            self.consecutive_failures += 1
            if self.consecutive_failures >= self.max_failures_before_direction_change:
                # After multiple failures, choose a completely new direction
                self.current_direction = random.uniform(-math.pi, math.pi)
                self.consecutive_failures = 0

    def generate_random_goal(self):
        """Generate a goal position using improved exploration strategy."""
        max_attempts = self.max_attempts
        attempts = 0
        best_goal = None
        best_score = float('-inf')
        
        # Get current robot pose
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return None
        
        # Initialize or update exploration parameters
        if self.current_direction is None:
            self.current_direction = random.uniform(-math.pi, math.pi)
        
        # Calculate minimum distance based on consecutive failures
        min_distance = self.min_distance_from_current * (1 + self.consecutive_failures * 0.5)
        max_distance = min(8.0, self.max_x - self.min_x)  # Increased max distance for better exploration
        
        while attempts < max_attempts:
            # Generate goals with increasing distance and varying angles
            angle_variance = math.pi/2  # 90 degrees variance
            base_angle = self.current_direction
            
            # Use spiral pattern for exploration
            spiral_factor = attempts / max_attempts
            distance = min_distance + (max_distance - min_distance) * spiral_factor
            angle = base_angle + spiral_factor * 2 * math.pi + random.uniform(-angle_variance, angle_variance)
            
            x = robot_pose.position.x + distance * math.cos(angle)
            y = robot_pose.position.y + distance * math.sin(angle)
            
            # Ensure within bounds
            x = max(self.min_x, min(self.max_x, x))
            y = max(self.min_y, min(self.max_y, y))
            
            point = Point()
            point.x = x
            point.y = y
            
            # Check validity
            isValidGoal, _ = self.is_valid_goal(x, y)
            
            if isValidGoal and not self.is_goal_blacklisted(point):
                # Score the candidate
                score = self._evaluate_goal(x, y, distance, angle)
                
                if score > best_score:
                    best_score = score
                    best_goal = (x, y)
                    
                # Accept first good goal
                if score > 0.7:
                    break
            
            attempts += 1
        
        # If no good goal found, try emergency recovery
        if best_goal is None:
            return self._generate_emergency_goal(robot_pose)
        
        return best_goal

    def _evaluate_goal(self, x, y, distance, angle):
        """Evaluate the quality of a potential goal with emphasis on exploration."""
        score = 1.0
        
        # Strongly prefer goals that are far from previous goals
        for old_goal in self.goal_blacklist[-10:]:  # Consider last 10 goals
            dx = x - old_goal.x
            dy = y - old_goal.y
            dist_to_old = math.sqrt(dx*dx + dy*dy)
            if dist_to_old < 3.0:  # Increased minimum distance from old goals
                score *= 0.3  # Stronger penalty for being close to previous goals
        
        # Prefer goals that are farther away (encourages exploration)
        distance_score = min(1.0, distance / 5.0)  # Normalize distance up to 5 meters
        score *= (0.5 + 0.5 * distance_score)
        
        # Check clearance in costmap
        clearance_score = self._evaluate_clearance(x, y)
        score *= clearance_score
        
        return score

    def _generate_emergency_goal(self, robot_pose):
        """Generate an emergency goal when stuck."""
        # Try to move in a random direction but close by
        for _ in range(8):  # Try 8 different directions
            angle = random.uniform(-math.pi, math.pi)
            distance = random.uniform(self.min_distance_from_current, 2.0)
            
            x = robot_pose.position.x + distance * math.cos(angle)
            y = robot_pose.position.y + distance * math.sin(angle)
            
            # Ensure within bounds
            x = max(self.min_x, min(self.max_x, x))
            y = max(self.min_y, min(self.max_y, y))
            
            isValidGoal, _ = self.is_valid_goal(x, y)
            if isValidGoal:
                return (x, y)
        
        return None

    def _evaluate_clearance(self, x, y, radius=0.8):  # Increased radius for better clearance
        """Evaluate the clearance around a potential goal."""
        if self.global_costmap is None:
            return 1.0
        
        resolution = self.global_costmap.info.resolution
        cells_to_check = int(radius / resolution)
        
        total_cells = 0
        free_cells = 0
        
        for dx in range(-cells_to_check, cells_to_check + 1):
            for dy in range(-cells_to_check, cells_to_check + 1):
                check_x = x + dx * resolution
                check_y = y + dy * resolution
                if self.is_point_valid_in_costmap(check_x, check_y, self.global_costmap):
                    free_cells += 1
                total_cells += 1
        
        return free_cells / total_cells if total_cells > 0 else 0.0

    def is_goal_blacklisted(self, point, tolerance=1.0):  # Increased tolerance
        """Check if a goal position is in the blacklist with larger tolerance."""
        for blacklisted_point in self.goal_blacklist:
            dx = point.x - blacklisted_point.x
            dy = point.y - blacklisted_point.y
            if math.sqrt(dx*dx + dy*dy) < tolerance:
                return True
        return False

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
    
    def send_stop_command(self):
        """Send an immediate stop command to the robot."""
        stop_msg = Twist()
        # All velocities set to 0
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0
        
        # Publish stop command multiple times to ensure it's received
        for _ in range(5):
            self.vel_pub.publish(stop_msg)
            rclpy.spin_once(self, timeout_sec=0.01)
    
    def stop(self, force_return=False):
        """Stop the random walker and clean up."""
        self.is_active = False
        self._navigation_active = False
        self.get_logger().info('Random walker stopped')
        
        # Send immediate stop command
        if force_return==False:
            self.send_stop_command()
        
        if self._timeout_timer is not None:
            self.destroy_timer(self._timeout_timer)
            self._timeout_timer = None
        
        # Cancel current goal if it exists
        if hasattr(self, '_goal_handle') and self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        
        if force_return and self.initial_pose is not None:
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

        # Get initial position before sending goal
        initial_robot_pose = self.get_robot_pose()
        if initial_robot_pose is None:
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose = self.initial_pose
        
        self.get_logger().info('Returning to initial pose')
        self.nav_client.send_goal_async(goal_msg)

        # Wait 2 seconds
        rclpy.spin_once(self, timeout_sec=2.0)  # Spins for 2 seconds

        # Get new position after waiting
        new_robot_pose = self.get_robot_pose()

        # If position hasn't changed, retry
        if (abs(initial_robot_pose.position.x - new_robot_pose.position.x) < 0.01 and
            abs(initial_robot_pose.position.y - new_robot_pose.position.y) < 0.01):
            self.get_logger().warn('Robot hasn\'t moved, retrying to return to initial pose')
            self.return_to_initial_pose()

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