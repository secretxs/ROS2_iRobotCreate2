import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from create_msgs.msg import Bumper
import numpy as np
import struct

class BumperVizNode(Node):

    def __init__(self):
        super().__init__('bumper_viz_node')
        self.publisher = self.create_publisher(PointCloud2, '/bumper_viz', 10)
        self.subscription = self.create_subscription(Bumper, '/bumper', self.bumper_callback, 10)

    def bumper_callback(self, msg: Bumper):
        # Process the bumper state message and create a PointCloud2 message
        pointcloud = self.bumper_to_pointcloud(msg)

        # Publish the PointCloud2 message
        self.publisher.publish(pointcloud)

    def bumper_to_pointcloud(self, bumper_msg: Bumper) -> PointCloud2:
        # Create an empty point cloud message
        pointcloud = PointCloud2()
        pointcloud.header = bumper_msg.header
        pointcloud.height = 1
        pointcloud.width = 6
        pointcloud.is_bigendian = False
        pointcloud.point_step = 12
        pointcloud.row_step = pointcloud.width * pointcloud.point_step
        pointcloud.is_dense = True

        # Define the point fields
        pointcloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        # Create a numpy array to store the points
        points = np.full((pointcloud.width, 3), np.nan, dtype=np.float32)
    
        # Process the bumper light sensor values
        light_sensors = [
            bumper_msg.light_signal_left,
            bumper_msg.light_signal_front_left,
            bumper_msg.light_signal_center_left,
            bumper_msg.light_signal_center_right,
            bumper_msg.light_signal_front_right,
            bumper_msg.light_signal_right

        ]
        # Define the angles of the light sensors in degrees
        angles_deg = np.array([
            35,
            20,
            10,
            -10,
            -20,
            -35
        ])

        # Convert angles to radians
        angles = np.deg2rad(angles_deg)

        # Update the points array based on the light sensor values
        for i, sensor_value in enumerate(light_sensors):
            if sensor_value > 10:
                # Light sensors range 0-4000 but even when data is around 0-700 so
                if sensor_value > 700:
                    distance = 1
                else:
                    distance = 10 / sensor_value

                # Apply an offset to the distance
                distance += 0.2 # You can change this value based on your requirements
                
                # Compute the x and y coordinates of the point based on the distance and angle
                points[i, 0] = distance * np.cos(angles[i])
                points[i, 1] = distance * np.sin(angles[i])

                # Set the constant z offset
                z_offset = 0.07
                points[i, 2] = z_offset

        # Convert the points array to a byte array and set it as the data of the point cloud
        pointcloud.data = points.tobytes()

        return pointcloud

def main(args=None):
    try:
        rclpy.init(args=args)
        bumper_viz_node = BumperVizNode()
        rclpy.spin(bumper_viz_node)
    except KeyboardInterrupt:
        print("Interrupted by keyboard, closing point cloud.")
    finally:
        if 'bumper_viz_node' in locals():
            bumper_viz_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

