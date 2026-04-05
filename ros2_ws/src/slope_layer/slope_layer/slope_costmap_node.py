#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class SlopeCostmapNode(Node):
    def __init__(self):
        super().__init__("slope_costmap_node")
        
        # Parameters
        self.declare_parameter("pointcloud_topic", "/front_rgbd/depth/points")
        self.declare_parameter("global_frame", "odom")
        self.declare_parameter("max_slope_deg", 15.0)
        self.declare_parameter("lethal_slope_deg", 25.0)
        
        self.pointcloud_topic = self.get_parameter("pointcloud_topic").value
        self.global_frame = self.get_parameter("global_frame").value
        self.max_slope = math.radians(self.get_parameter("max_slope_deg").value)
        self.lethal_slope = math.radians(self.get_parameter("lethal_slope_deg").value)
        
        self.resolution = 0.05
        self.width = 120
        self.height = 120

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.pub = self.create_publisher(OccupancyGrid, "/slope_costmap", 10)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.sub = self.create_subscription(
            PointCloud2, self.pointcloud_topic, self.pointcloud_callback, qos_profile)       
        
        self.get_logger().info("Slope Detector Online - Monitoring Terrain Saftey")

    def pointcloud_callback(self, msg: PointCloud2):
        try:
            # Get transform 
            t = self.tf_buffer.lookup_transform(
                self.global_frame, msg.header.frame_id, rclpy.time.Time(nanoseconds=0))
        except Exception:
            return

        # 1. Setup Grids
        # z_grid stores the minimum Z height per cell
        z_grid = np.full((self.height, self.width), np.nan, dtype=np.float32)
        grid_data = np.zeros((self.height, self.width), dtype=np.int8)

        origin_x = -(self.width * self.resolution) / 2.0
        origin_y = -(self.height * self.resolution) / 2.0

        # 2. Extract and Transform Points
        # 2. Extract and Transform Points
        tx, ty, tz = t.transform.translation.x, t.transform.translation.y, t.transform.translation.z
        
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        
        for p in points:
            # FIX: Skip infinite values from Gazebo "void"
            if not np.isfinite(p[0]) or not np.isfinite(p[1]) or not np.isfinite(p[2]):
                continue

            wx, wy, wz = p[0] + tx, p[1] + ty, p[2] + tz
            
            # Use floor to safely handle float-to-int conversion
            gx = int(math.floor((wx - origin_x) / self.resolution))
            gy = int(math.floor((wy - origin_y) / self.resolution))

            if 0 <= gx < self.width and 0 <= gy < self.height:
                if np.isnan(z_grid[gy, gx]) or wz < z_grid[gy, gx]:
                    z_grid[gy, gx] = wz


        # 3. Vectorized Gradient Calculation (Much faster than nested loops)
        # Using a 3x3 Sobel-like filter to find steepness
        if not np.all(np.isnan(z_grid)):
            # Fill NaNs with local mean to avoid "holes" in slope math
            mask = np.isnan(z_grid)
            z_filled = np.where(mask, 0, z_grid) 
            
            dzdx, dzdy = np.gradient(z_filled, self.resolution)
            slope_map = np.arctan(np.sqrt(dzdx**2 + dzdy**2))

            # Apply thresholds
            grid_data[slope_map >= self.lethal_slope] = 100
            grid_data[(slope_map >= self.max_slope) & (slope_map < self.lethal_slope)] = 50

        # 4. Publish
        costmap = OccupancyGrid()
        costmap.header.stamp = self.get_clock().now().to_msg()
        costmap.header.frame_id = self.global_frame
        costmap.info.resolution = self.resolution
        costmap.info.width = self.width
        costmap.info.height = self.height
        costmap.info.origin.position.x = float(origin_x)
        costmap.info.origin.position.y = float(origin_y)
        costmap.info.origin.orientation.w = 1.0
        costmap.data = grid_data.flatten().tolist()

        self.pub.publish(costmap)

def main(args=None):
    rclpy.init(args=args)
    node = SlopeCostmapNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()