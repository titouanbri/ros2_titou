#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time 
import numpy as np

# Import MediaPipe
import mediapipe as mp

class HandDetNode(Node):
    def __init__(self):
        super().__init__('hand_det_node')

        self.get_logger().info("Initialisation MediaPipe Hands...")

        # --- Configuration MediaPipe ---
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

        # Initialisation du détecteur
        # max_num_hands=2 : détecte jusqu'à 2 mains
        # min_detection_confidence=0.5 : seuil de confiance
        self.hands = self.mp_hands.Hands(
            model_complexity=0,  # 0=Lite (rapide), 1=Full (précis)
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        
        # Ros config
        self.br = CvBridge()

        # Subscriber
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        
        # Publisher 
        self.publisher_ = self.create_publisher(Image, '/hand_det/result', 10)

    def image_callback(self, msg):
        start_total = time.time()
        
        # 1. Conversion ROS Image -> OpenCV
        try:
            cv_image = self.br.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Error CvBridge: {e}")
            return

        # 2. Inférence (MediaPipe requiert du RGB)
        start_inference = time.time()
        
        # Conversion BGR (OpenCV) vers RGB (MediaPipe)
        image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        
        # Pour améliorer les perfs, on marque l'image comme non-modifiable
        image_rgb.flags.writeable = False
        results = self.hands.process(image_rgb)
        image_rgb.flags.writeable = True
        
        end_inference = time.time()

        # 3. Visualisation & Récupération de position
        start_visualization = time.time()
        
        # On redessine sur l'image originale (BGR)
        res_image = cv_image.copy()

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Dessiner le squelette de la main
                self.mp_drawing.draw_landmarks(
                    res_image,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style())

                # --- EXEMPLE: Récupérer la position du poignet (Wrist) ---
                # Les coordonnées sont normalisées [0.0, 1.0]
                h, w, _ = res_image.shape
                wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
                
                cx, cy = int(wrist.x * w), int(wrist.y * h)
                
                # Afficher un cercle sur le poignet et les coordonnées
                cv2.circle(res_image, (cx, cy), 10, (0, 255, 0), -1)
                cv2.putText(res_image, f"Wrist: {cx},{cy}", (cx+10, cy), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        end_visualization = time.time()

        # 4. Conversion OpenCV -> ROS Image and Publish
        out_msg = self.br.cv2_to_imgmsg(res_image, "bgr8")
        self.publisher_.publish(out_msg)

        end_total = time.time()

        # Stats
        total_time = end_total - start_total
        fps = 1.0 / total_time if total_time > 0 else 0.0
        
        # Log moins verbeux (optionnel)
        # self.get_logger().info(f"FPS: {fps:.2f} | Hands detected: {len(results.multi_hand_landmarks) if results.multi_hand_landmarks else 0}")

def main(args=None):
    rclpy.init(args=args)

    try:
        node = HandDetNode()
        print("Node MediaPipe Hands started.")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()