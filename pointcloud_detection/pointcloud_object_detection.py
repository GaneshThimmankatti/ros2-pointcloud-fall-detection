import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2
import numpy as np

class PointcloudObjectDetection(Node):
    def __init__(self):
        super().__init__('pointcloud_object_detection')

        # Declare parameters with default values
        self.declare_parameter('fov_angle_deg', 35.0)
        self.declare_parameter('ground_distance_min', 2.0)
        self.declare_parameter('ground_distance_max', 3.0)
        self.declare_parameter('ground_plane_range', 0.1)
        self.declare_parameter('grid_angle_deg', 10.0)
        self.declare_parameter('angle_increment', 0.25 / 180 * np.pi)  # radians
        self.declare_parameter('cell_threshold', 100.0)
        self.declare_parameter('max_object_height', 1.8)
        self.declare_parameter('height_over_ground', 0.20)
        self.declare_parameter('sub_topic_name', "/camera/camera/depth/color/points_transformed")
        self.declare_parameter('pub_topic_name', '/camera/camera/depth/color/scan')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('range_max',6.0)

        # Retrieve parameters
        self.fov_angle_deg = self.get_parameter('fov_angle_deg').get_parameter_value().double_value
        self.ground_distance_min = self.get_parameter('ground_distance_min').get_parameter_value().double_value
        self.ground_distance_max = self.get_parameter('ground_distance_max').get_parameter_value().double_value
        self.ground_plane_range = self.get_parameter('ground_plane_range').get_parameter_value().double_value
        self.grid_angle_deg = self.get_parameter('grid_angle_deg').get_parameter_value().double_value
        self.angle_increment = self.get_parameter('angle_increment').get_parameter_value().double_value
        self.cell_threshold = self.get_parameter('cell_threshold').get_parameter_value().double_value
        self.max_object_height = self.get_parameter('max_object_height').get_parameter_value().double_value
        self.height_over_ground = self.get_parameter('height_over_ground').get_parameter_value().double_value
        sub_topic_name = self.get_parameter('sub_topic_name').get_parameter_value().string_value
        pub_topic_name = self.get_parameter('pub_topic_name').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.range_max = self.get_parameter('range_max').get_parameter_value().double_value

        # TODO: add flags to enable/disable detections via parameters

        if sub_topic_name is None or sub_topic_name == "":
            self.get_logger().error("sub_topic_name is None")
            exit()

        # If no pub_topic_name given, then append sub_topic_name with "_scan"
        if pub_topic_name is None or pub_topic_name == "":
            self.get_logger().info(f"No pub_topic_name provided. Using sub_topic_name: {sub_topic_name}_scan")
            sub_topic_name_split = sub_topic_name.split("/")
            sub_topic_name_split[-1] =  f'{sub_topic_name_split[-1]}_scan'
            pub_topic_name = '/'.join(sub_topic_name_split)
        
        
        self.angle_min = -(self.fov_angle_deg/180*np.pi) / 2
        self.angle_max = (self.fov_angle_deg/180*np.pi) / 2
        self.radius = self.ground_distance_max - self.ground_distance_min
        self.grid_num = int(self.radius / 0.25)
        self.radial_distances = np.geomspace(self.ground_distance_min, self.ground_distance_max, self.grid_num)
        self.num_angular_sectors = int(self.fov_angle_deg / self.grid_angle_deg)
        self.num_radial_sectors = len(self.radial_distances)-1

        self.enable_object_detection = True
        self.enable_ground_detection = False
        # Once detected a gap in the ground, should we continue looking at ground further away or break and return closest value?
        self.break_on_detection = False

        self.point_cloud_subscriber = self.create_subscription(
            PointCloud2,
            sub_topic_name,
            self.point_cloud_callback,
            10
        )
        self.laser_scan_publisher = self.create_publisher(LaserScan, pub_topic_name, 10)

        self.get_logger().info(f"Subscribing to topic: {sub_topic_name}")
        self.get_logger().info(f"Publishing to topic: {pub_topic_name}")

        self.get_logger().info(f"Object detection: {self.enable_object_detection}")
        self.get_logger().info(f"Ground detection: {self.enable_ground_detection}")
        self.get_logger().info("PointcloudObjectDetection initialized")

    def point_cloud_callback(self, msg):
        # Convert PointCloud2 to numpy array
        pc_data = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(list(pc_data))
        if points.size == 0:
            self.get_logger().info("No points received")
            return

        x = points['x']
        y = points['y']
        # NOTE: adjusting height over ground as the transformation is at the base_link and not on the ground
        z = points['z'] + self.height_over_ground
        
        # Initialize the scan ranges
        ranges = np.ones(int((self.angle_max-self.angle_min)/self.angle_increment+1)) * np.nan

        # Find objects and update the ranges
        if self.enable_object_detection:
            # Objects are anything above ground until maximum object height & objects below ground
            object_mask = (np.abs(z) > self.ground_plane_range) & (z < self.max_object_height)
            object_x = x[object_mask]
            object_y = y[object_mask]
            object_distances = np.sqrt(object_x**2 + object_y**2)
            object_angles = np.arctan2(object_y, object_x)
            indices = ((object_angles - self.angle_min) / self.angle_increment).astype(int)
            indices_valid = np.where((0 <= indices) & (indices < len(ranges)) & (object_distances < self.range_max))[0]
            
            indices = indices[indices_valid]
            distances = object_distances[indices_valid]

            # Step 1: Handle duplicates by selecting the smallest rs for each index
            unique_indexes, unique_inverse, unique_counts = np.unique(indices, return_inverse=True, return_counts=True)
            # Create an array to store the smallest distances for each unique index
            min_rs_per_index = np.full_like(unique_indexes, np.inf, dtype=float)
            # Update min_rs_per_index to keep the smallest rs for each unique index
            np.minimum.at(min_rs_per_index, unique_inverse, distances)

            # Step 2: Update the ranges array using min_rs_per_index
            ranges[unique_indexes] = np.minimum(ranges[unique_indexes], min_rs_per_index)
            # Handle NaN values by replacing them with the corresponding min_rs_per_index values
            nan_mask = np.isnan(ranges[unique_indexes])
            ranges[unique_indexes[nan_mask]] = min_rs_per_index[nan_mask]

        # Now work on the ground points usingb grid cell counting
        if self.enable_ground_detection:
            ground_mask = np.abs(z) <= self.ground_plane_range
            ground_x = x[ground_mask]
            ground_y = y[ground_mask]
            ground_distances = np.sqrt(ground_x**2 + ground_y**2)
            ground_angles_deg = np.degrees(np.arctan2(ground_y, ground_x))
            # Count points in each grid cell and highlight first cell below the threshold
            boundary_as = []
            boundary_rs = []
            grid_counts = np.zeros((self.num_radial_sectors, self.num_angular_sectors))
            for j in range(self.num_angular_sectors):
                for i in range(len(self.radial_distances) - 1):
                    start_angle = (-self.fov_angle_deg) / 2 + j * self.grid_angle_deg
                    end_angle = start_angle + self.grid_angle_deg
                    r_min = self.radial_distances[i]
                    r_max = self.radial_distances[i + 1]

                    # Count points in the current grid cell
                    count = np.sum(
                        (ground_angles_deg >= start_angle) & (ground_angles_deg < end_angle) &
                        (ground_distances >= r_min) & (ground_distances < r_max) 
                    )
                    grid_counts[i, j] = count

                    if grid_counts[i, j] < self.cell_threshold:
                        # Find points between the two angular lines
                        angle_rad_start = np.radians(start_angle)
                        angle_rad_end = np.radians(end_angle)
                        angles_in_range = np.linspace(angle_rad_start, angle_rad_end, int((angle_rad_end - angle_rad_start) / self.angle_increment))
                        boundary_rs.append((np.ones(len(angles_in_range)) * r_min).tolist())
                        boundary_as.append(angles_in_range.tolist())

                        # Break to move to the next angular grid line
                        if self.break_on_detection:
                            break
                            
            # Flatten boundary angles and distances
            boundary_as_flat = [
                x 
                for xs in boundary_as 
                for x in xs
                ]
            boundary_rs_flat = [
                x 
                for xs in boundary_rs 
                for x in xs]
            # Sort the values
            sort_index = np.argsort(boundary_as_flat)
            boundary_as_flat = np.array(boundary_as_flat)[sort_index]
            boundary_rs_flat = np.array(boundary_rs_flat)[sort_index]
            # Convert angle to index and update the ranges
            indexes = np.array((boundary_as_flat-self.angle_min)/self.angle_increment, dtype=np.int32)
            ranges[indexes] = boundary_rs_flat


        # Create LaserScan message
        scan_msg = LaserScan()
        scan_msg.header = msg.header
        scan_msg.header.frame_id = self.target_frame
        scan_msg.angle_min = -(self.fov_angle_deg / 180 * np.pi) / 2
        scan_msg.angle_max = (self.fov_angle_deg / 180 * np.pi) / 2
        scan_msg.angle_increment = self.angle_increment
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.0
        scan_msg.range_max = self.range_max
        scan_msg.ranges = ranges.tolist()

        self.laser_scan_publisher.publish(scan_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PointcloudObjectDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
