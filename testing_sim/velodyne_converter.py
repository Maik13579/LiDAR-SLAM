#Velodyne in husky sim has no time field, this script adds it

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class PointCloudModifier(Node):
    def __init__(self):
        super().__init__('velodyne_converter')
        
        # Subscriber to PointCloud2
        self.subscription = self.create_subscription(
            PointCloud2,
            '/husky/sensors/lidar3d_0/points',
            self.pointcloud_callback,
            10
        )
        
        # Publisher for modified PointCloud2
        self.publisher = self.create_publisher(PointCloud2, '/husky/sensors/lidar3d_0/points_with_time', 10)
    
    def pointcloud_callback(self, msg):
        # Calculate the number of points
        num_points = msg.width * msg.height

        # Original point step and data
        original_point_step = msg.point_step
        original_data = np.frombuffer(msg.data, dtype=np.uint8)

        # Create the new fields list with the additional 'time' field
        new_fields = list(msg.fields)
        new_fields.append(PointField(
            name='time',
            offset=msg.point_step,  # New field added at the end
            datatype=PointField.FLOAT32,
            count=1
        ))

        # New point step includes the new field (4 bytes for FLOAT32)
        new_point_step = msg.point_step + 4

        # Initialize new data buffer
        new_data = np.zeros(num_points * new_point_step, dtype=np.uint8)

        # Copy existing data and add zeros for the 'time' field
        for i in range(num_points):
            # Calculate positions in the old and new data arrays
            start_src = i * original_point_step
            end_src = start_src + original_point_step

            start_dst = i * new_point_step
            end_dst = start_dst + original_point_step

            # Copy original data to new data buffer
            new_data[start_dst:end_dst] = original_data[start_src:end_src]
            # The 'time' field remains zero (already initialized to zero)

        # Create new PointCloud2 message
        modified_msg = PointCloud2()
        modified_msg.header = msg.header
        modified_msg.height = msg.height
        modified_msg.width = msg.width
        modified_msg.fields = new_fields
        modified_msg.is_bigendian = msg.is_bigendian
        modified_msg.point_step = new_point_step
        modified_msg.row_step = new_point_step * msg.width
        modified_msg.is_dense = msg.is_dense
        modified_msg.data = new_data.tobytes()

        # Publish the modified PointCloud2
        self.publisher.publish(modified_msg)





def main(args=None):
    rclpy.init(args=args)
    pointcloud_modifier = PointCloudModifier()
    rclpy.spin(pointcloud_modifier)
    pointcloud_modifier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
