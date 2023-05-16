# ROS2 Libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# Jetson Libraries
import jetson_inference
import jetson_utils

# Math Libraries
import argparse
import sys
import numpy as np
from cv_bridge import CvBridge

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('Detectnet')
        # Subscribe to Image
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10)
        # Object Detection Overlay Image Publisher
        self.publisher_ = self.create_publisher(
            Image, 
            'Detectnet/image_overlay', 
            10)

        self.bridge = CvBridge()

        # load the object detection network
        self.net = jetson_inference.detectNet("ssd-mobilenet-v2", threshold=0.4)

    def listener_callback(self, msg):
    	 # Convert img_msg to cv2 image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Convert cv2 image to cuda image
        cuda_image = jetson_utils.cudaFromNumpy(cv_image)

        # detect objects in the image (with overlay)
        detections = self.net.Detect(cuda_image, overlay="box,labels,conf")

        # print the detections
        self.get_logger().info("detected {:d} objects in image".format(len(detections)))

        for detection in detections:
            self.get_logger().info(str(detection))

        # render the image
        output_image = jetson_utils.cudaToNumpy(cuda_image)
        output_image_msg = self.bridge.cv2_to_imgmsg(np.array(output_image, dtype=np.uint8), encoding='bgr8')

        # Publish the image
        self.publisher_.publish(output_image_msg)

        # print out performance info
        self.net.PrintProfilerTimes()


def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    rclpy.spin(image_processor)
    # Destroy the node explicitly
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

