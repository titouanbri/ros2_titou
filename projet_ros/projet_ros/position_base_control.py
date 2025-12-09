import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class SimpleServoMover(Node):
    def __init__(self):
        super().__init__('simple_servo_mover')

        # 1. Création du Publisher
        # Le topic standard pour MoveIt Servo est souvent 'servo_node/delta_twist_cmds'
        # Assurez-vous que ce topic correspond à votre configuration launch file
        self.publisher_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

        # Timer pour publier à 50Hz (20ms)
        # Il est important de publier régulièrement pour que Servo ne stoppe pas le robot
        self.timer = self.create_timer(0.02, self.publish_command)
        
        # Compteur pour arrêter le mouvement après un certain temps (sécurité pour ce test)
        self.count = 0

        self.get_logger().info("Nœud démarré. Attention, le robot va bouger !")

    def publish_command(self):
        msg = TwistStamped()

        # 2. Configuration du Header (CRUCIAL)
        # MoveIt Servo rejette les messages si le timestamp est trop vieux
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Le repère de référence. Pour un UR3e, c'est souvent 'base_link' ou 'base'
        msg.header.frame_id = 'base_link'

        # 3. Définition du Mouvement
        # On bouge seulement si le compteur est < 200 (environ 4 secondes à 50Hz)
        if self.count < 200:
            # Avancer de 5 cm/s (0.05 m/s) sur l'axe X
            msg.twist.linear.x = 0.05 
            
            # Pas de rotation
            msg.twist.angular.x = 0.0
            msg.twist.angular.y = 0.0
            msg.twist.angular.z = 0.0
            
            if self.count % 50 == 0:
                self.get_logger().info(f"Envoi de commande... (vitesse linéaire X: {msg.twist.linear.x})")
        else:
            # Après 4 secondes, on envoie des zéros pour arrêter proprement
            msg.twist.linear.x = 0.0
            if self.count == 200:
                self.get_logger().info("Arrêt du mouvement.")

        # 4. Publication
        self.publisher_.publish(msg)
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimpleServoMover()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Envoi d'une dernière commande d'arrêt par sécurité
        stop_msg = TwistStamped()
        stop_msg.header.stamp = node.get_clock().now().to_msg()
        stop_msg.header.frame_id = 'base_link'
        node.publisher_.publish(stop_msg)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()