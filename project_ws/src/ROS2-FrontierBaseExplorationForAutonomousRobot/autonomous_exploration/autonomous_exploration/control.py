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
lookahead_distance = 0.5
speed = 0.2
expansion_size = 3
target_error = 0.05
robot_r = 0.2
pathGlobal = 0

# Fonctions utilitaires
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

def costmap(data, width, height, resolution):
    data = np.array(data).reshape(height, width)
    wall = np.where(data == 100)
    for i in range(-expansion_size, expansion_size + 1):
        for j in range(-expansion_size, expansion_size + 1):
            if i == 0 and j == 0:
                continue
            x = wall[0] + i
            y = wall[1] + j
            x = np.clip(x, 0, height - 1)
            y = np.clip(y, 0, width - 1)
            data[x, y] = 100
    return data

def frontier_exploration(data, width, height, resolution, column, row, originX, originY):
    global pathGlobal
    data = costmap(data, width, height, resolution)
    data[row][column] = 0
    data[data > 5] = 1

    # Ajout de logs pour visualiser un extrait de la carte traitée
    print(f"[DEBUG] Processed map sample: {data[:10, :10]}")

    frontier_points = np.argwhere((data == 0) & (np.pad(data != 0, 1, constant_values=1)[1:-1, 1:-1] == 1))
    if len(frontier_points) > 0:
        print(f"[DEBUG] Found {len(frontier_points)} frontier points.")
        closest_frontier = min(frontier_points, key=lambda p: heuristic((row, column), (p[0], p[1])))
        path = astar(data, (row, column), tuple(closest_frontier))
        pathGlobal = [(p[1] * resolution + originX, p[0] * resolution + originY) for p in path]
    else:
        print("[DEBUG] No frontier points found.")
        pathGlobal = -1



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
        self.cmd_vel_pub = self.create_publisher(Twist, '/limo_105_3/cmd_vel', 10)
        self.map_data = None
        self.odom_data = None
        self.scan_data = None
        self.path = None
        print("[INFO] Exploration node initialized.")
        threading.Thread(target=self.explore).start()

    def explore(self):
        twist = Twist()
        while True:
            if not all([self.map_data, self.odom_data, self.scan_data]):
                time.sleep(0.1)
                continue

            if self.path is None or self.path == -1:
                print("[DEBUG] Generating new path.")
                self.generate_path()
            else:
                v, w = localControl(self.scan_data.ranges)
                if v is None:
                    v, w = self.pure_pursuit()
                twist.linear.x = v
                twist.angular.z = w
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.1)

    def generate_path(self):
        if self.map_data is None or self.odom_data is None:
            self.get_logger().error("Map or odometry data is missing!")
            return

        row = int((self.x - self.originX) / self.resolution)
        column = int((self.y - self.originY) / self.resolution)

        if not (0 <= row < self.height and 0 <= column < self.width):
            self.get_logger().error(f"Invalid robot position on the map: row={row}, column={column}")
            return

        self.get_logger().info(f"Generating path from position ({row}, {column})...")
        frontier_exploration(self.data, self.width, self.height, self.resolution, column, row, self.originX, self.originY)
        self.path = pathGlobal

        if self.path == -1:
            self.get_logger().warn("No valid path found!")
        else:
            self.get_logger().info(f"Path generated: {self.path}")



    def pure_pursuit(self):
        return speed, 0.0

    def map_callback(self, msg):
        self.map_data = msg
        self.resolution = msg.info.resolution
        self.originX = msg.info.origin.position.x
        self.originY = msg.info.origin.position.y
        self.width = msg.info.width
        self.height = msg.info.height
        self.data = np.array(msg.data).reshape((self.height, self.width))
        self.get_logger().info(f"Map received: {self.data.shape}, sample: {self.data[:5, :5]}")


    def odom_callback(self, msg):
        self.odom_data = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

    def scan_callback(self, msg):
        self.scan_data = msg

def main(args=None):
    rclpy.init(args=args)
    node = NavigationControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
