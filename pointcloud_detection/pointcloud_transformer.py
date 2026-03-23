import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
import struct
import tf2_ros

class PointCloudTransformer(Node):
    def __init__(self):
        super().__init__('pointcloud_transformer')

        self.declare_parameter('sub_topic_name', None)
        self.declare_parameter('pub_topic_name', None)
        
        # Either pass a target frame or set the values manually
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('roll_deg', 0.0)
        self.declare_parameter('pitch_deg', 0.0)
        self.declare_parameter('yaw_deg', 0.0)
        self.declare_parameter('x_offset', 0.0)
        self.declare_parameter('y_offset', 0.0)
        self.declare_parameter('z_offset', 0.0)

        self.declare_parameter('subsampling_ratio', 1)

        sub_topic_name = self.get_parameter('sub_topic_name').get_parameter_value().string_value
        pub_topic_name = self.get_parameter('pub_topic_name').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        roll = self.get_parameter('roll_deg').get_parameter_value().double_value / 180.0 * np.pi
        pitch = self.get_parameter('pitch_deg').get_parameter_value().double_value / 180.0 * np.pi
        yaw = self.get_parameter('yaw_deg').get_parameter_value().double_value / 180.0 * np.pi
        x_offset = self.get_parameter('x_offset').get_parameter_value().double_value
        y_offset = self.get_parameter('y_offset').get_parameter_value().double_value
        z_offset = self.get_parameter('z_offset').get_parameter_value().double_value
        self.subsampling_ratio = self.get_parameter('subsampling_ratio').get_parameter_value().integer_value

        if sub_topic_name is None or sub_topic_name == "":
            self.get_logger().error("sub_topic_name is None")
            exit()

        # If no pub_topic_name given, then append sub_topic_name with "_transformed"
        if pub_topic_name is None or pub_topic_name == "":
            self.get_logger().info(f"No pub_topic_name provided. Using sub_topic_name: {sub_topic_name}")
            sub_topic_name_split = sub_topic_name.split("/")
            sub_topic_name_split[-1] =  f'{sub_topic_name_split[-1]}_transformed'
            pub_topic_name = '/'.join(sub_topic_name_split)
            

        self.subscription = self.create_subscription(
            PointCloud2,
            sub_topic_name,
            self.pointcloud_callback,
            1)
        self.publisher = self.create_publisher(PointCloud2, pub_topic_name, 10)
        
        self.get_logger().info(f"Subscribing to topic: {sub_topic_name}")
        self.get_logger().info(f"Publishing to topic: {pub_topic_name}")

        # Define transformations
        self.rotation_matrix = None
        self.translation_matrix = None
        if self.target_frame is not None and self.target_frame != "":
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.transform = None
        else:
            self.get_logger().info(f"roll: {roll}, pitch: {pitch} yaw: {yaw}")
            self.get_logger().info(f"x: {x_offset}, y: {y_offset}, z: {z_offset}")
            self.rotation_matrix = self.euler_to_rotation_matrix(roll, pitch, yaw)
            self.translation_matrix = np.array([x_offset, y_offset, z_offset])
            self.transformation_matrix_optical_to_ros2 = self.create_transformation_matrix(
                self.rotation_matrix, self.translation_matrix)
            self.get_logger().info(f"T: \n{self.transformation_matrix_optical_to_ros2}")
            self.target_frame = None

        self.get_logger().info(f"Sub-sampling ratio: {self.subsampling_ratio}")

        self.get_logger().info('PointCloudTransformer is initialized.')

    def pointcloud_callback(self, msg):
        self.get_logger().debug('Received PointCloud2 message.')
        
        try:
            if self.target_frame is None:
                # Convert the point cloud using the vectorized function
                points = self.pointcloud2_to_xyz_array(msg)
                if points.size == 0:
                    self.get_logger().warn('No points found in the PointCloud2 message.')
                    return
            
                # Apply subsampling using slicing (e.g., take every N-th point)
                subsampled_points = points[::self.subsampling_ratio]
            
                # Convert to homogeneous coordinates: add a column of ones
                ones = np.ones((subsampled_points.shape[0], 1), dtype=np.float32)
                homogeneous_points = np.hstack((subsampled_points, ones))  # shape: (N, 4)
            
                # Apply the transformation matrix (4x4) to all points at once
                transformed_homogeneous = (self.transformation_matrix_optical_to_ros2 @ homogeneous_points.T).T
            
                # Drop the homogeneous coordinate to get back to 3D points
                transformed_points = transformed_homogeneous[:, :3]
            
                # Update the header: you might want to use a fixed frame_id if using a static transform
                header = msg.header
                header.frame_id = self.target_frame
            
                # Create and publish the transformed PointCloud2 message
                transformed_cloud_msg = pc2.create_cloud_xyz32(header, transformed_points.tolist())

            else:
                # Get the transformation from source_frame to target_frame
                if self.transform is None:
                    self.transform = self.tf_buffer.lookup_transform(self.target_frame, msg.header.frame_id, rclpy.time.Time())
                    # Extract translation and rotation from the transform
                    self.translation_matrix = np.array([self.transform.transform.translation.x, 
                                            self.transform.transform.translation.y, 
                                            self.transform.transform.translation.z])
                    rotation = self.transform.transform.rotation
                    q = [rotation.x, rotation.y, rotation.z, rotation.w]
                    self.rotation_matrix = self.quaternion_to_rotation_matrix(q)

                # Extract points from PointCloud2 message
                points = self.pointcloud2_to_xyz_array(msg)
                # Sub-sample the points
                if self.subsampling_ratio > 1:
                    points = points[::self.subsampling_ratio]
                # Apply the transformation to the points
                transformed_points = (self.rotation_matrix @ points.T).T + self.translation_matrix
                
                # NOTE: publishing only ground points
                # transformed_points = transformed_points[(-0.2 < transformed_points[:,2]) & (transformed_points[:,2] < -0.1)]
                # Create and publish the transformed PointCloud2 message
                transformed_cloud_msg = self.xyz_array_to_pointcloud2(transformed_points, frame_id=self.target_frame, stamp=msg.header.stamp)

            self.publisher.publish(transformed_cloud_msg)
        except tf2_ros.LookupException as e:
            self.get_logger().warn(f'Could not transform point cloud: {e}')

    def euler_to_rotation_matrix(self, roll, pitch, yaw):
        """Convert Euler angles in radians to a rotation matrix."""
        Rx = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])

        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])

        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])

        R = Rz @ Ry @ Rx
        return R

    def quaternion_to_rotation_matrix(self, q):
        # Convert quaternion (x, y, z, w) to a 3x3 rotation matrix
        x, y, z, w = q
        return np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x**2 + y**2)]
        ])

    def pointcloud2_to_xyz_array(self, cloud_msg):
        """
        Convert a PointCloud2 message to a numpy array of shape (num_points, 3) using vectorized operations.
        Assumes that the x, y, z fields are stored as float32 and appear consecutively at the beginning of each point.
        """
        point_step = cloud_msg.point_step  # Total bytes per point
        num_points = len(cloud_msg.data) // point_step
        # Convert the raw data buffer into an array of float32
        data = np.frombuffer(cloud_msg.data, dtype=np.float32)
        # Calculate how many float32 numbers make up each point
        floats_per_point = point_step // 4
        # Reshape the data to have one row per point
        points_all = data.reshape(num_points, floats_per_point)
        # Extract only the first three values corresponding to x, y, z
        points = points_all[:, :3]
        return points


    def _get_struct_fmt(self, cloud_msg):
        # Determine the struct format for unpacking PointCloud2 data
        field_names = {field.name: (field.offset, field.datatype) for field in cloud_msg.fields}
        return 'fff'  # Assuming the order is x, y, z and each is a float32

    def xyz_array_to_pointcloud2(self, xyz_array, frame_id, stamp):
        # Convert a numpy array of points to a PointCloud2 message
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        point_step = 12  # 3 float32 values
        data = np.array(xyz_array, dtype=np.float32).tobytes()

        point_cloud_msg = PointCloud2(
            header=header,
            height=1,
            width=xyz_array.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=point_step,
            row_step=point_step * xyz_array.shape[0],
            data=data
        )

        return point_cloud_msg

    def create_transformation_matrix(self, rotation_matrix, translation):
        """Create a homogeneous transformation matrix from rotation matrix and translation."""
        T = np.eye(4)
        T[0:3, 0:3] = rotation_matrix
        T[0:3, 3] = translation
        return T

    def transform_point(self, point, transformation_matrix):
        """Transform a point using the given transformation matrix."""
        homogeneous_point = np.append(point, 1)
        transformed_homogeneous_point = transformation_matrix @ homogeneous_point
        return transformed_homogeneous_point[:3]

def main(args=None):
    rclpy.init(args=args)
    pointcloud_transformer = PointCloudTransformer()
    rclpy.spin(pointcloud_transformer)
    pointcloud_transformer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
