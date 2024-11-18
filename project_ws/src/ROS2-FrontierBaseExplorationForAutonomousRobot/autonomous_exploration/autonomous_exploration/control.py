import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int8
import numpy as np
import heapq, math, random, threading, time
from rclpy.qos import qos_profile_sensor_data

# Paramètres globaux
lookahead_distance = 0.5 #0.24 initially
speed = 0.2  #0.05 initially
expansion_size = 3
target_error = 0.05  
robot_r = 0.2

pathGlobal = 0


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


def localControl(scan):
    """Évite les obstacles locaux."""
    for i in range(60):
        if scan[i] < robot_r:
            return 0.0, -math.pi / 4
    for i in range(300, 360):
        if scan[i] < robot_r:
            return 0.0, math.pi / 4
    return None, None


class NavigationControl(Node):
    def __init__(self):
        super().__init__('exploration_node')

        self.map_sub = self.create_subscription(OccupancyGrid, '/limo_105_3/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/limo_105_3/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/limo_105_3/scan', self.scan_callback, qos_profile_sensor_data)
        self.resume_sub = self.create_subscription(Int8, '/limo_105_3/explore/resume', self.resume_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/limo_105_3/cmd_vel', 10)

        # Variables d'état
        self.kesif = True
        self.map_data = None
        self.odom_data = None
        self.scan_data = None
        self.path = None
        self.timer = None
        print("[INFO] Exploration node initialized.")
        threading.Thread(target=self.explore).start()

    def resume_callback(self, msg):
        """Callback pour activer ou désactiver l'exploration."""
        if msg.data == 1:
            print("[INFO] Exploration resumed.")
            self.kesif = True
        elif msg.data == 0:
            print("[INFO] Exploration paused.")
            self.kesif = False

    def explore(self):
        """Boucle principale de l'exploration."""
        twist = Twist()
        while True:
            if not all([self.map_data, self.odom_data, self.scan_data]):
                time.sleep(0.1)
                continue

            if self.kesif:
                print("[INFO] Exploration active.")
                if self.path is None:
                    print("[DEBUG] Generating new path.")
                    self.generate_path()
                if self.path and self.is_goal_reached():
                    print("[INFO] Goal reached.")
                    self.kesif = False
                else:
                    v, w = localControl(self.scan_data.ranges)
                    if v is None:
                        v, w = self.pure_pursuit()
                    twist.linear.x = v
                    twist.angular.z = w
                    self.cmd_vel_pub.publish(twist)
                    time.sleep(0.1)

    def generate_path(self):
        """Génère un chemin vers un nouvel objectif."""
        pass

    def is_goal_reached(self):
        """Vérifie si le robot a atteint son objectif."""
        pass

    def map_callback(self, msg):
        self.map_data = msg
        print("[DEBUG] Map data updated.")

    def odom_callback(self, msg):
        self.odom_data = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        print(f"[DEBUG] Odom updated: x={self.x}, y={self.y}, yaw={self.yaw}")

    def scan_callback(self, msg):
        self.scan_data = msg
        print("[DEBUG] LaserScan updated.")

    def pure_pursuit(self):
        """Implémentation de l'algorithme Pure Pursuit."""
        return 0.2, 0.0   #0.1, 0.0


def main(args=None):
    rclpy.init(args=args)
    node = NavigationControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
