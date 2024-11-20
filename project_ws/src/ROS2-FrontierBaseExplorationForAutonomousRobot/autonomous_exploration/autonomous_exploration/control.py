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
speed = 0.1  #0.05 initially
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


def localControl(scan):
    """Évite les obstacles locaux et ajuste la vitesse."""
    filtered_ranges = [r if r > 0.1 else float('inf') for r in (scan[0:30] + scan[330:360])]

   
    min_front = min(filtered_ranges)  

    print(f"[DEBUG] LaserScan min_front après filtration = {min_front}")

   
    if min_front < robot_r + 0.1:
        print("[DEBUG] Obstacle très proche, stop et tourne.")
        return 0.0, 1.0  # Stoppe et tourne sur place

   
    if min_front < robot_r + 0.5: 
        adjusted_speed = max(speed * (min_front / (robot_r + 0.5)), 0.1)
        print(f"[DEBUG] Obstacle modéré, adjusted_speed = {adjusted_speed}")
        return adjusted_speed, 0.0  

    # Aucun obstacle détecté, avancer à pleine vitesse
    print(f"[DEBUG] Aucun obstacle, avance à pleine vitesse = {speed}")
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
    """Boucle principale d'exploration."""
    twist = Twist()
    last_position = [self.x, self.y]
    stuck_count = 0

    while True:
        if not all([self.map_data, self.odom_data, self.scan_data]):
            time.sleep(0.05)  
            continue

        if self.kesif:
            print("[INFO] Exploration active.")

            # Planification globale si nécessaire
            if self.path is None or len(self.path) == 0:
                print("[DEBUG] Génération d'un nouveau chemin avec la carte.")
                self.generate_path()

            if self.path and not self.is_goal_reached():
                v, w = self.pure_pursuit()
                print(f"[DEBUG] Suivi de chemin avec pure_pursuit: v={v}, w={w}")
            else:
                # Fallback vers LIDAR
                print("[DEBUG] Aucun chemin, bascule sur le contrôle local.")
                v, w = localControl(self.scan_data.ranges)

            # Publier les commandes
            twist.linear.x = v
            twist.angular.z = w
            self.cmd_vel_pub.publish(twist)

            time.sleep(0.05)





    def generate_path(self):
        if not self.map_data:
            print("[WARN] Aucune donnée de carte disponible pour la planification.")
            return

        data = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))
        row = int((self.x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        col = int((self.y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

        frontiers = self.detect_frontiers(data)
        if len(frontiers) == 0:
            print("[INFO] Aucune frontière détectée. L'exploration est terminée.")
            self.path = None
            return

        
        closest_frontier = min(frontiers, key=lambda f: heuristic((row, col), f))

       
        start = (row, col)
        goal = tuple(closest_frontier)
        path = astar(data, start, goal)

        if path:
            self.path = [(p[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x,
                        p[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y) for p in path]
            print(f"[INFO] Nouveau chemin généré vers {goal}.")
        else:
            print("[WARN] Aucun chemin valide trouvé.")
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
