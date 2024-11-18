import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import heapq, math, random
from rclpy.qos import qos_profile_sensor_data

# Paramètres globaux
LOOKAHEAD_DISTANCE = 0.5
SPEED = 0.5
EXPANSION_SIZE = 3
TARGET_ERROR = 0.05
ROBOT_RADIUS = 0.2

# Utilitaires pour les calculs
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
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]
        
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0] and 0 <= neighbor[1] < array.shape[1]:
                if array[neighbor[0]][neighbor[1]] == 1:  # Obstacle
                    continue
            else:
                continue
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, float('inf')):
                continue
            if tentative_g_score < gscore.get(neighbor, float('inf')) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return None

def detect_frontiers(data):
    """Détecte les frontières entre zones explorées et inconnues."""
    frontier_points = np.argwhere(
        (data == 0) & (
            np.roll(data == -1, 1, axis=0) |
            np.roll(data == -1, -1, axis=0) |
            np.roll(data == -1, 1, axis=1) |
            np.roll(data == -1, -1, axis=1)
        )
    )
    return frontier_points

# Classe principale
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
        self.path = []
        self.timer = self.create_timer(0.1, self.explore)

        print("[INFO] Autonomous exploration initialized.")

    def explore(self):
        if not all([self.map_data, self.odom_data, self.scan_data]):
            return  # Attendre que toutes les données soient disponibles

        if self.path:
            self.follow_path()
        else:
            self.generate_path()

    def generate_path(self):
        row = int((self.x - self.originX) / self.resolution)
        column = int((self.y - self.originY) / self.resolution)
        frontiers = detect_frontiers(self.data)
        
        if not frontiers.size:
            print("[INFO] No more frontiers to explore.")
            self.publish_velocity(0.0, 0.0)
            return
        
        closest_frontier = min(frontiers, key=lambda p: heuristic((row, column), tuple(p)))
        path = astar(self.data, (row, column), tuple(closest_frontier))
        
        if path:
            self.path = [(p[1] * self.resolution + self.originX, p[0] * self.resolution + self.originY) for p in path]
            print(f"[INFO] Path generated: {self.path}")
        else:
            print("[WARN] No valid path found!")
            self.path = []

    def follow_path(self):
        if not self.path:
            return

        target = self.path[0]
        target_angle = math.atan2(target[1] - self.y, target[0] - self.x)
        angle_diff = target_angle - self.yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        # Logs pour diagnostic
        print(f"[DEBUG] Target: {target}, Current Position: ({self.x}, {self.y}), Yaw: {self.yaw}")
        print(f"[DEBUG] Target Angle: {target_angle}, Angle Difference: {angle_diff}")

        # Vérification des obstacles proches
        if self.scan_data and min(self.scan_data.ranges) < 0.5:  # Limite de 0.5 m pour les obstacles
            print("[WARN] Obstacle detected! Stopping motion.")
            self.publish_velocity(0.0, 0.0)  # Arrêter le robot
            return

        # Rotation pour alignement
        if abs(angle_diff) > 0.3:  # Tolérance angulaire augmentée
            angular_speed = max(min(angle_diff * 2, 1.5), -1.5)  # Rotation plus rapide
            self.publish_velocity(0.05, angular_speed)  # Réduire la vitesse linéaire pendant la rotation

        # Translation directe si alignement acceptable
        elif heuristic((self.x, self.y), target) > TARGET_ERROR:
            print(f"[INFO] Moving towards target: {target}")
            self.publish_velocity(SPEED, 0.0)

        # Arrêt à la cible
        else:
            print(f"[INFO] Reached target: {target}")
            self.path.pop(0)



    def publish_velocity(self, linear, angular):
        twist = Twist()
        twist.linear.x = max(min(linear, 0.5), -0.5)
        twist.angular.z = max(min(angular, 1.0), -1.0)
        self.cmd_vel_pub.publish(twist)
        print(f"[DEBUG] Publishing velocity: linear={twist.linear.x}, angular={twist.angular.z}")


    def map_callback(self, msg):
        self.map_data = msg
        self.resolution = msg.info.resolution
        self.originX = msg.info.origin.position.x
        self.originY = msg.info.origin.position.y
        self.width = msg.info.width
        self.height = msg.info.height
        self.data = np.array(msg.data).reshape((self.height, self.width))

    def odom_callback(self, msg):
        self.odom_data = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        )
        print(f"[DEBUG] Odometry updated: x={self.x}, y={self.y}, yaw={self.yaw}")

    def scan_callback(self, msg):
        self.scan_data = msg
        min_distance = min(msg.ranges)
        print(f"[DEBUG] Closest obstacle distance: {min_distance}")

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousExploration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
