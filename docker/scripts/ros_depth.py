# ROS2 Libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

# Jetson Libraries
import jetson_inference
import jetson_utils
import cv2

# Math Libraries
import numpy as np
import struct
from math import sqrt, sin, radians



class DepthEstimationNode(Node):
    def __init__(self):
        super().__init__('DepthNet')
        self.net = jetson_inference.depthNet()
        # Subscribe to Image
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.process_image,
            10)
        # Depth Point Cloud Publisher
        self.publisher = self.create_publisher(
            PointCloud2,
            'DepthNet/depth_point_cloud',
            5,
        )
        # Depth Image Publisher
        self.image_publisher = self.create_publisher(
            Image,
            'DepthNet/depth_image',
            5
        )
        self.bridge = CvBridge()
        
        # Initialize dimensions and Image Transformation Parameters
        self.image_width_px = None
        self.image_height_px = None
        self.cx = None
        self.cx = None
        self.u = None
        self.v = None


    def process_image(self, img_msg):
    
        # Convert img_msg to cv2 image
        img_cv = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
       
        # Convert cv2 image to cuda image
        img = jetson_utils.cudaFromNumpy(img_cv)
       
        # Process Image
        self.net.Process(img)
        jetson_utils.cudaDeviceSynchronize()
        
        depth_numpy = jetson_utils.cudaToNumpy(self.net.GetDepthField())      
        
        # Convert depth_numpy to a colormap for visualization
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_numpy, alpha=80), cv2.COLORMAP_RAINBOW)
        
	    # Convert To Image
        depth_image_msg = self.bridge.cv2_to_imgmsg(depth_colormap, 'bgr8')

        # Publish the depth image
        self.image_publisher.publish(depth_image_msg)
        
        # Convert To 2D array 
        depth_2d = depth_numpy.reshape(depth_numpy.shape[0], -1)

        # Generate point cloud from depth_numpy
        point_cloud_msg = self.create_pointcloud(depth_2d)

        # Publish the generated point cloud
        self.publisher.publish(point_cloud_msg)

    def create_pointcloud(self, depth_numpy):
        
        # Initialize dimensions
        if self.image_width_px is None or self.image_height_px is None:
            self.image_width_px = depth_numpy.shape[1]
            self.image_height_px = depth_numpy.shape[0]
            self.initialize_image_properties()

        # Create header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'odom'

        # Create fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            # Assuming depth image is grayscale, so intensity is the same value as depth
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        # Create point cloud
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = self.image_height_px
        cloud.width = self.image_width_px
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = 16  # Four 32-bit floats for x, y, z, and intensity
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
       
        
        # Convert depth image to point cloud
        
        scaling_factor = 1  # Adjust this value according to your need
        z = depth_numpy * scaling_factor  # Apply scaling factor 
        x = np.flip((self.u - self.cx) * z / self.fx)
        y = (self.v - self.cy) * z / self.fy
        intensity = z  # Assuming depth image is grayscale, so intensity is the same value as depth
        
        # 3D Rotation
        xr = z
        yr = x
        zr = -y + 1
        
        # Stack x, y, z, intensity into a 3D array
        pointcloud = np.stack((xr, yr, zr, intensity), axis=2)
        
        # Reshape and convert point cloud data to bytes
        cloud.data = pointcloud.astype(np.float32).tobytes()
        
        return cloud

    def initialize_image_properties(self):
            
            # Logitech C922 Camera Specifications
            diagonal_FOV = radians(78.0)  # Convert to radians
            f_mm = 3.67  # Focal length in mm

            # Calculate sensor size
            sensor_size_mm = f_mm * sqrt(2) / sin(diagonal_FOV)

            # Calculate aspect ratio
            aspect_ratio = self.image_width_px / self.image_height_px

            # Calculate sensor width and height
            sensor_width_mm = sensor_size_mm * aspect_ratio / sqrt(1 + aspect_ratio**2)
            sensor_height_mm = sensor_size_mm / sqrt(1 + aspect_ratio**2)

            # Calculate fx, fy in pixel units
            self.fx = (self.image_width_px * f_mm) / sensor_width_mm
            self.fy = (self.image_height_px * f_mm) / sensor_height_mm

            # Principal point (usually at the image center)
            self.cx, self.cy = self.image_width_px / 2, self.image_height_px / 2

            # Create range vectors for u and v
            u = np.arange(self.image_width_px)
            v = np.arange(self.image_height_px)
            self.u, self.v = np.meshgrid(u, v)

def main(args=None):
    rclpy.init(args=args)
    depth_estimation_node = DepthEstimationNode()
    rclpy.spin(depth_estimation_node)
    depth_estimation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
