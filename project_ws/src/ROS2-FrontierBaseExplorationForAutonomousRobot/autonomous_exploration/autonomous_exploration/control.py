import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import heapq, math, random, time
from rclpy.qos import qos_profile_sensor_data

# Paramètres globaux
lookahead_distance = 0.5
speed = 0.2
expansion_size = 3
target_error = 0.05
robot_r = 0.2
pathGlobal = None

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    yaw_z = math.atan2(t0, t1)
    return yaw_z

def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def astar(array, start, goal):
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data[::-1]
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0] and 0 <= neighbor[1] < array.shape[1]:
                if array[neighbor[0]][neighbor[1]] == 1:
                    continue
            else:
                continue
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return False

def detect_frontiers(data):
    """Détecte les frontières entre zones explorées et inconnues."""
    frontier_points = np.argwhere(
        (data == 0) & (np.roll(data == -1, 1, axis=0) | np.roll(data == -1, -1, axis=0) |
                       np.roll(data == -1, 1, axis=1) | np.roll(data == -1, -1, axis=1))
    )
    return frontier_points


class AutonomousExploration(Node):
    def __init__(self):
        super().__init__('autonomous_exploration')
        self.map_sub = self.create_subscription(OccupancyGrid, '/limo_105_3/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/limo_105_3/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/limo_105_3/scan', self.scan_callback, qos_profile_sensor_data)
        self.cmd_vel_pub = self.create_publisher(Twist, '/limo_105_3/cmd_vel', 10)

        self.map_data = None
        self.odom_data = None
        self.scan_data = None
        self.path = None
        self.current_goal = None
        print("[INFO] Autonomous exploration initialized.")
        self.timer = self.create_timer(0.1, self.explore)

    def explore(self):
        if not all([self.map_data, self.odom_data, self.scan_data]):
            return  # Attendre que toutes les données soient disponibles

        if self.path is None or len(self.path) == 0:
            # Générer un nouveau chemin vers une frontière
            print("[DEBUG] Generating new path...")
            self.generate_path()
        else:
            # Suivre le chemin existant
            v, w = self.follow_path()
            self.publish_velocity(v, w)

    def generate_path(self):
        row = int((self.x - self.originX) / self.resolution)
        column = int((self.y - self.originY) / self.resolution)
        frontiers = detect_frontiers(self.data)

        if len(frontiers) > 0:
            closest_frontier = min(frontiers, key=lambda p: heuristic((row, column), (p[0], p[1])))
            path = astar(self.data, (row, column), tuple(closest_frontier))
            if path:
                self.path = [(p[1] * self.resolution + self.originX, p[0] * self.resolution + self.originY) for p in path]
                print(f"[INFO] Path generated: {self.path}")
            else:
                print("[WARN] No valid path found!")
        else:
            print("[INFO] No more frontiers to explore.")

    def follow_path(self):
        if self.path:
            target = self.path.pop(0)
            target_angle = math.atan2(target[1] - self.y, target[0] - self.x)
            angle_diff = target_angle - self.yaw

            if abs(angle_diff) > 0.1:
                return 0.0, angle_diff  # Tourner vers la cible
            else:
                return speed, 0.0  # Avancer vers la cible
        return 0.0, 0.0

    def publish_velocity(self, linear, angular):
        twist = Twist()
        twist.linear.x = max(min(linear, 0.5), -0.5)  # Limite linéaire
        twist.angular.z = max(min(angular, 1.0), -1.0)  # Limite angulaire
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"Publishing velocity: linear={twist.linear.x}, angular={twist.angular.z}")


    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info(f"Received map data: size={msg.info.width}x{msg.info.height}, resolution={msg.info.resolution}")
        self.resolution = msg.info.resolution
        self.originX = msg.info.origin.position.x
        self.originY = msg.info.origin.position.y
        self.width = msg.info.width
        self.height = msg.info.height
        self.data = np.array(msg.data).reshape((self.height, self.width))

    def odom_callback(self, msg):
        self.odom_data = msg
        self.get_logger().info(f"Received odometry: position=({msg.pose.pose.position.x}, {msg.pose.pose.position.y})")
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

    def scan_callback(self, msg):
        self.scan_data = msg
        self.get_logger().info(f"Received scan data: {len(msg.ranges)} ranges")

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousExploration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
