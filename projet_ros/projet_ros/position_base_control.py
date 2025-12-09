import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys

class ServoMover(Node):
    def __init__(self):
        super().__init__('servo_mover_node')

        # 1. Configuration du Publisher
        # Le topic standard pour MoveIt Servo est souvent 'delta_twist_cmds'
        # Vérifie si ton servo_node attend ce topic (c'est le défaut).
        self.publisher_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        
        # 2. Timer pour la boucle de publication
        # Servo a besoin d'un flux continu de commandes (ex: 10Hz ou plus)
        self.timer_period = 0.1  # 100ms (10Hz)
        self.timer = self.create_timer(self.timer_period, self.publish_command)
        
        self.get_logger().info("Nœud ServoMover démarré. Le robot devrait bouger en X...")

    def publish_command(self):
        msg = TwistStamped()

        # 3. Remplissage du Header
        # Important : Le timestamp doit être actuel pour que Servo accepte la commande
        msg.header.stamp = self.get_clock().now().to_msg()
        # Le frame de référence. Pour un UR3, c'est souvent 'base_link' ou 'base_link_inertia'
        msg.header.frame_id = 'base_link' 

        # 4. Définition du mouvement (Vitesse Cartésienne)
        # Ici, on demande une petite vitesse linéaire sur l'axe X (en m/s)
        msg.twist.linear.x = 0.0  # 10 cm/s
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.1
        
        # Pas de rotation
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        # Publication
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    servo_mover = ServoMover()

    try:
        rclpy.spin(servo_mover)
    except KeyboardInterrupt:
        pass
    finally:
        # En quittant, on envoie une commande vide pour stopper le robot proprement
        stop_msg = TwistStamped()
        stop_msg.header.stamp = servo_mover.get_clock().now().to_msg()
        stop_msg.header.frame_id = 'base_link'
        servo_mover.publisher_.publish(stop_msg)
        
        servo_mover.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()