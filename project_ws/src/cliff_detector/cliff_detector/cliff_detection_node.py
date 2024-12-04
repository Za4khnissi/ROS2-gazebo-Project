import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import sensor_msgs_py.point_cloud2 as pc2

class CliffDetectionNode(Node):
    def __init__(self):
        super().__init__('CliffDetectionNode')
        
        # Paramètre pour le seuil de différence d'altitude
        self.declare_parameter('elevation_diff_threshold', 0.001)
        self.elevation_diff_threshold = self.get_parameter('elevation_diff_threshold').get_parameter_value().double_value

        # Initialisation des abonnements et publications
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.point_cloud_sub = self.create_subscription(PointCloud2, '/camera/depth/points', self.point_cloud_callback, 10)

        # Variables pour stocker les altitudes minimales
        self.previous_min_z = None
        self.negative_elevation_detected = False

        # Timer pour commander le robot régulièrement
        self.timer = self.create_timer(0.1, self.command_robot)

    def point_cloud_callback(self, msg):
        # Lire le seuil à partir du paramètre
        self.elevation_diff_threshold = self.get_parameter('elevation_diff_threshold').get_parameter_value().double_value

        # Extraire le point avec la valeur z (altitude) minimale
        min_z = None
        for point in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            z = point[2]
            if min_z is None or z < min_z:
                min_z = z

        # Si aucune valeur valide n'est trouvée, ignorer
        if min_z is None:
            self.get_logger().warning("Aucun point valide trouvé dans le nuage de points.")
            return

        # Comparer avec l'altitude minimale précédente
        if self.previous_min_z is not None:
            elevation_diff = min_z - self.previous_min_z
            if elevation_diff < -self.elevation_diff_threshold:
                self.negative_elevation_detected = True
                self.get_logger().info(f"Changement d'altitude détecté : {elevation_diff:.4f}.")
            else:
                self.negative_elevation_detected = False
                self.get_logger().info(f"Altitude stable ou en augmentation : {elevation_diff:.4f}.")
        
        # Mettre à jour l'altitude minimale précédente
        self.previous_min_z = min_z

    def command_robot(self):
        # Créer le message de commande de mouvement
        cmd = Twist()
        if self.negative_elevation_detected:
            self.get_logger().info("Vide détecté, Robot s'arrete.")
            cmd.linear.x = 0.0  # Reculer
        else:
            self.get_logger().info("Aucun vide détecté, le robot avance.")
            cmd.linear.x = 0.1  # Avancer

        # Publier la commande
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = CliffDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
