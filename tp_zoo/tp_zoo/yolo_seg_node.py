#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time 
import numpy as np

from ultralytics import YOLO

class YoloSegNode(Node):
    def __init__(self):
        super().__init__('yolo_seg_node')

        self.get_logger().info("Initialisation YOLOv8...")
        
        self.model = YOLO("yolov8n-seg.pt")  
        
        # Ros config
        self.br = CvBridge()

        # Subscriber
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        
        # Publisher 
        self.publisher_ = self.create_publisher(Image, '/yolo/result', 10)

    def image_callback(self, msg):
        start_total = time.time()
        
        # Conversion ROS Image -> OpenCV
        try:
            cv_image = self.br.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Error CvBridge: {e}")
            return

        # Inférence 
        start_inference = time.time()
        
        results = self.model.predict(source=cv_image, verbose=False, classes=[0], conf=0.5)
        
        end_inference = time.time()

        # Visualisation
        start_visualization = time.time()
        
        res_image = results[0].plot() 
        
        end_visualization = time.time()

        # Conversion OpenCV -> ROS Image and Publish
        out_msg = self.br.cv2_to_imgmsg(res_image, "bgr8")
        self.publisher_.publish(out_msg)

        end_total = time.time()

        # Calculation times
        total_time = end_total - start_total
        inference_time = end_inference - start_inference
        visualization_time = end_visualization - start_visualization
        fps = 1.0 / total_time if total_time > 0 else 0.0

        self.get_logger().info(f"Total: {total_time:.3f}s | Infer: {inference_time:.3f}s | Visu: {visualization_time:.3f}s | FPS: {fps:.2f}")

def main(args=None):
    rclpy.init(args=args)

    try:
        node = YoloSegNode()
        print("Node YOLOv8 started. Waiting for images...")
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