#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time 

# Imports Detectron2
from detectron2.utils.logger import setup_logger
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog

class Detectron2Node(Node):
    def __init__(self):
        super().__init__('detectron2_node')

        self.get_logger().info("Initialisation ...")
        setup_logger()
        
        self.cfg = get_cfg()
        #Mask R-CNN standard
        config_file = "COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"
        self.cfg.merge_from_file(model_zoo.get_config_file(config_file))
        self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5 
        self.cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url(config_file)
        self.cfg.MODEL.DEVICE = "cpu" # Force CPU (i don't have external GPU)

        self.predictor = DefaultPredictor(self.cfg)
        self.get_logger().info("Successfully loaded model.")

        # Metadata for visualization (e.g., class names)
        self.metadata = MetadataCatalog.get(self.cfg.DATASETS.TRAIN[0])

        # Ros config
        self.br = CvBridge()

        # Subscriber
        self.subscription = self.create_subscription(Image,'/camera/camera/color/image_raw',self.image_callback,10)
        
        # Publisher 
        self.publisher_ = self.create_publisher(Image, '/detectron2/result', 10)

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
        outputs = self.predictor(cv_image)
        end_inference = time.time()

        # Visualisation
        #BGR -> RGB for Visualizer
        start_visualization = time.time()
        im_rgb = cv_image[:, :, ::-1]
        v = Visualizer(im_rgb, self.metadata, scale=1.0)

        
        # Draw predictions
        out = v.draw_instance_predictions(outputs["instances"].to("cpu"))
        
        # Retrieve the result image (RGB -> BGR)
        res_image = out.get_image()[:, :, ::-1]
        end_visualization = time.time()

        # Conversion OpenCV -> ROS Image and Publish
        out_msg = self.br.cv2_to_imgmsg(res_image, "bgr8")
        self.publisher_.publish(out_msg)

        end_total = time.time()

        #calculate times
        total_time = end_total - start_total
        inference_time = end_inference - start_inference
        visualization_time = end_visualization - start_visualization
        fps=1.0 / total_time if total_time > 0 else 0.0

        self.get_logger().info(f"Total time: {total_time:.3f}s | Inference time: {inference_time:.3f}s | Visualization time: {visualization_time:.3f}s | FPS: {fps:.2f}")

def main(args=None):
    rclpy.init(args=args)

    try:
        node = Detectron2Node()
        print("Node Detectron2 started. waiting for the image...")
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