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
speed = 0.5  #0.05 initially
expansion_size = 3
target_error = 0.05  
robot_r = 0.3

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


def localControlV2(scan):
    """Évite les obstacles locaux et ajuste la vitesse."""
    # Vérification des zones critiques à l'avant
    front_ranges = scan[0:30] + scan[330:360]
    ## for debug front_ranges = [r for r in (scan[0:30] + scan[330:360]) if r > 0.05]
    min_front = min(front_ranges)

    print(f"[DEBUG] LaserScan min_front = {min_front}")

    # Si un obstacle est détecté dans une zone critique
    if min_front < robot_r + 0.1:
        print("[DEBUG] Obstacle très proche, stop et tourne.")
        return 0.0, max(min(math.pi / 2, 1.5), -1.5)  # Tourne rapidement

    # Si un obstacle est détecté à une distance modérée, ralentir
    if min_front < robot_r + 0.2:
        adjusted_speed = max(speed * (min_front / (robot_r + 0.2)), 0.1)
        print(f"[DEBUG] Obstacle modéré, adjusted_speed = {adjusted_speed}")
        return adjusted_speed, 0.0  # Ajuste la vitesse en fonction de la distance

    # Aucun obstacle détecté, avancer à pleine vitesse
    print(f"[DEBUG] Aucun obstacle, avance à pleine vitesse = {speed}")
    return speed, 0.0
def preprocess_scan(scan):
    """Prétraite les données de LaserScan pour éviter les valeurs aberrantes."""
    return [max(min(r, 10.0), 0.05) for r in scan]

def localControl(scan):
    """Évite les obstacles locaux et ajuste la vitesse en fonction des données LiDAR."""
    # Filtrer les données invalides
    front_ranges = [r for r in scan[0:30] + scan[330:360] if 0.1 < r < 12.0]
    left_ranges = [r for r in scan[60:120] if 0.1 < r < 12.0]
    right_ranges = [r for r in scan[240:300] if 0.1 < r < 12.0]

    # Vérifier si aucune donnée valide n'est détectée
    if not front_ranges and not left_ranges and not right_ranges:
        print("[DEBUG] Aucune donnée valide du LiDAR. Avance par défaut.")
        return speed, 0.0  # Avancer par défaut

    # Minima des distances détectées
    min_front = min(front_ranges) if front_ranges else float('inf')
    min_left = min(left_ranges) if left_ranges else float('inf')
    min_right = min(right_ranges) if right_ranges else float('inf')

    # Cas : Obstacle très proche à l'avant
    if min_front < robot_r + 0.1:
        print("[DEBUG] Obstacle très proche devant, arrêt et rotation.")
        return 0.0, 1.0  # Tourner à gauche

    # Cas : Obstacle détecté à gauche ou à droite
    if min_left < robot_r + 0.3:
        print("[DEBUG] Obstacle détecté à gauche, ajustement vers la droite.")
        return speed * 0.5, -1.0
    if min_right < robot_r + 0.3:
        print("[DEBUG] Obstacle détecté à droite, ajustement vers la gauche.")
        return speed * 0.5, 1.0

    # Aucun obstacle détecté, avancer
    print("[DEBUG] Aucun obstacle détecté, avance à pleine vitesse.")
    return speed, 0.0




class NavigationControl(Node):
    def __init__(self):
        super().__init__('exploration_node')

        self.map_sub = self.create_subscription(OccupancyGrid, '/limo_105_1/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/limo_105_1/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/limo_105_1/scan', self.scan_callback, qos_profile_sensor_data)
        self.resume_sub = self.create_subscription(Int8, '/limo_105_1/explore/resume', self.resume_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/limo_105_1/cmd_vel', 10)

        # Variables d'état
        self.kesif = True
        self.map_data = None
        self.odom_data = None
        self.scan_data = None
        self.path = None
        self.timer = None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

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
    
    def detect_frontiers(self, data):
        """Détecte les frontières entre zones explorées et inconnues."""
        frontiers = []
        for i in range(1, data.shape[0] - 1):
            for j in range(1, data.shape[1] - 1):
                if data[i, j] == 0:  # Zone libre
                    neighbors = [data[i-1, j], data[i+1, j], data[i, j-1], data[i, j+1]]
                    if -1 in neighbors:  # Zone inconnue adjacente
                        frontiers.append((i, j))
        return frontiers    


    def explore(self):
        """Boucle principale de l'exploration."""
        twist = Twist()
        last_position = [self.x, self.y]
        stuck_count = 0

        while True:
            if not all([self.map_data, self.odom_data, self.scan_data]):
                time.sleep(0.05)  
                continue

            if self.kesif:
                print("[INFO] Exploration active.")

                if self.path is None or len(self.path) == 0:
                    print("[DEBUG] Generating new path.")
                    self.generate_path()

                if self.path and self.is_goal_reached():
                    print("[INFO] Goal reached.")
                    self.kesif = False
                else:
                    if abs(self.x - last_position[0]) < 0.01 and abs(self.y - last_position[1]) < 0.01:
                        stuck_count += 1
                    else:
                        stuck_count = 0

                    last_position = [self.x, self.y]

                    if stuck_count > 10:  
                        print("[WARN] Robot stuck, reculer et tourner.")
                        twist.linear.x = -0.1  # Reculer légèrement
                        twist.angular.z = 1.0  # Tourner
                        self.cmd_vel_pub.publish(twist)
                        stuck_count = 0  
                        time.sleep(1.0)  
                        continue


                    
                    v, w = localControl(self.scan_data.ranges)
                    if v is None:  # Si aucun obstacle n'est détecté localement
                        v, w = self.pure_pursuit()

                    twist.linear.x = v
                    twist.angular.z = w
                    self.cmd_vel_pub.publish(twist)

                    
                    time.sleep(0.05)




    def generate_path(self):
        if not self.map_data:
            return

        data = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))
        row = int((self.x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        col = int((self.y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

        # Détecter les frontières
        frontiers = self.detect_frontiers(data)
        if len(frontiers) == 0:
            print("[INFO] No more frontiers to explore.")
            self.path = None
            return

        closest_frontier = min(frontiers, key=lambda f: heuristic((row, col), f))

        goal = tuple(closest_frontier)
        start = (row, col)
        path = astar(data, start, goal)

        if path:
            self.path = [(p[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x,
                        p[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y) for p in path]
            print("[INFO] New path generated.")
        else:
            print("[WARN] No valid path found.")
            self.path = None


    def is_goal_reached(self):
        if self.path and len(self.path) == 0:
            return True
        return False


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
        if not self.path or len(self.path) == 0:
            return 0.0, 0.0  # Pas de chemin, arrêt

        target = self.path[0]
        target_angle = math.atan2(target[1] - self.y, target[0] - self.x)
        angle_diff = target_angle - self.yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  

        if abs(angle_diff) > 0.5:  
            return 0.0, max(min(angle_diff * 1.5, 1.0), -1.0) 
        elif heuristic((self.x, self.y), target) > target_error:  
            return speed, max(min(angle_diff * 0.5, 0.5), -0.5)  
        else:  
            self.path.pop(0)
            return 0.0, 0.0




def main(args=None):
    rclpy.init(args=args)
    node = NavigationControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
