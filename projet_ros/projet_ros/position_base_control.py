#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Point, TransformStamped
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_point
import numpy as np
import math

class PBVSNode(Node):
    def __init__(self):
        super().__init__('ur3e_pbvs_node')

        # --- Configuration ---
        self.camera_frame = 'camera_optical_frame' # Nom typique, à vérifier sur ton UR3e
        self.base_frame = 'base_link'
        self.target_distance = 0.30 # Distance souhaitée par rapport au point M (en mètres)
        self.kp_linear = 1.0        # Gain proportionnel linéaire
        self.kp_angular = 2.0       # Gain proportionnel angulaire

        # --- Subscribers & Publishers ---
        # Le point M du cours, publié en temps réel (coordonnées 3D)
        self.target_sub = self.create_subscription(
            Point, 
            '/target', 
            self.target_callback, 
            10
        )
        
        # Commande de vitesse pour MoveIt Servo
        self.servo_pub = self.create_publisher(
            TwistStamped, 
            '/servo_node/delta_twist_cmds', 
            10
        )

        # --- TF Buffer (Pour la géométrie spatiale) ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Variable pour stocker la dernière position de M connue
        self.latest_target_M = None
        
        # Timer de contrôle (ex: 50Hz)
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info("Nœud PBVS démarré. En attente de /target...")

    def target_callback(self, msg):
        """Réception du point M (X, Y, Z) défini dans le cours[cite: 21, 93]."""
        self.latest_target_M = msg

    def control_loop(self):
        if self.latest_target_M is None:
            return

        try:
            # 1. Obtenir la transformation Robot -> Caméra
            # Nous devons exprimer M dans le repère de la caméra pour calculer l'erreur
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.base_frame, # On suppose que /target est publié dans base_link (ou world)
                rclpy.time.Time()
            )

            # 2. Transformer le point M dans le repère caméra (Camera-centered reference frame)
            # Selon le cours, M a pour coordonnées (X, Y, Z) dans ce repère.
            p_target = Point()
            p_target.x = self.latest_target_M.x
            p_target.y = self.latest_target_M.y
            p_target.z = self.latest_target_M.z
            
            # Astuce pour tf2_geometry_msgs : il faut un PointStamped pour la transfo
            from geometry_msgs.msg import PointStamped
            ps_target = PointStamped()
            ps_target.header.frame_id = self.base_frame
            ps_target.header.stamp = self.get_clock().now().to_msg()
            ps_target.point = p_target

            # M_cam = Coordonnées de M vues par la caméra
            m_cam_stamped = do_transform_point(ps_target, transform)
            M_cam = m_cam_stamped.point

            # 3. Calcul de l'erreur (Loi de commande PBVS)
            # Le cours indique que l'axe Z est l'axe principal.
            # Pour regarder l'objet, M doit être à (0, 0, Z) dans le repère caméra.
            
            # Erreur de position (pour s'approcher/s'éloigner)
            error_z = M_cam.z - self.target_distance 
            
            # Erreur d'orientation (pour centrer l'objet)
            # Nous voulons que X et Y soient 0.
            # Cela correspond aux équations de projection u = f*X/Z, v = f*Y/Z.
            # Si X=0 et Y=0, alors u=pu et v=pv (le centre de l'image)[cite: 127, 129].
            
            # Commande : Twist (vitesse linéaire + angulaire)
            cmd = TwistStamped()
            cmd.header.frame_id = self.camera_frame
            cmd.header.stamp = self.get_clock().now().to_msg()

            # Vitesse Linéaire : On avance selon Z pour atteindre la distance cible
            # On peut aussi ajuster X/Y en translation (optionnel, ici on le fait en rotation)
            cmd.twist.linear.z = 1.0 * error_z 
            cmd.twist.linear.x = 0.5 * M_cam.x # Aide au centrage en translation (optionnel)
            cmd.twist.linear.y = 0.5 * M_cam.y

            # Vitesse Angulaire : C'est le cœur du "Look-at"
            # Si M est à gauche (X positif ou négatif selon convention), on tourne autour de Y.
            # Si M est en haut/bas, on tourne autour de X.
            # Approximation des petites rotations : w_x ~ -Y/Z, w_y ~ X/Z
            
            if M_cam.z > 0.1: # Sécurité pour éviter la division par zéro
                cmd.twist.angular.x = -self.kp_angular * (M_cam.y / M_cam.z)
                cmd.twist.angular.y =  self.kp_angular * (M_cam.x / M_cam.z)
                cmd.twist.angular.z = 0.0 # On ne gère pas le roulis (roll)

            # Saturation de sécurité (très important avec un vrai robot)
            max_lin = 0.2
            max_rot = 0.5
            cmd.twist.linear.x = np.clip(cmd.twist.linear.x, -max_lin, max_lin)
            cmd.twist.linear.y = np.clip(cmd.twist.linear.y, -max_lin, max_lin)
            cmd.twist.linear.z = np.clip(cmd.twist.linear.z, -max_lin, max_lin)
            cmd.twist.angular.x = np.clip(cmd.twist.angular.x, -max_rot, max_rot)
            cmd.twist.angular.y = np.clip(cmd.twist.angular.y, -max_rot, max_rot)

            # Publication
            self.servo_pub.publish(cmd)

        except Exception as e:
            self.get_logger().warn(f"Erreur TF ou calcul: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PBVSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()