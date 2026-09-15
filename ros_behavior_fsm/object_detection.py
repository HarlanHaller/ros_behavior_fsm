import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Pose, Quaternion, PoseArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan, PointCloud2
from typing import Literal, Optional, List, Tuple

from sensor_msgs_py import point_cloud2

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__("object_detection_node")
        self.scan_data_sub = self.create_subscription(LaserScan, 'stable_scan', self.on_scan_data, 10)
        self.obj_pub = self.create_publisher(PoseArray, 'detected_clusters', 10)
        self.dbg_point_pub = self.create_publisher(PointCloud2, 'laser_points', 10)
        self.min_range = 0.2
        self.max_range = 2
        self.obj_detect_dist_threshold = 0.3 

    def on_scan_data(self, data: LaserScan):
        # first, convert from the polar coords to cartesian coords in the lidar frame

        # List of x, y pairs
        points: List[Tuple[float, float, float]] = []
        # OPT: do this nice and vectorized with numpy
        for theta, range in enumerate(data.ranges):
            if range < max(self.min_range, data.range_min) or range > min(self.max_range, data.range_max):
                continue
            points.append((range * np.cos(theta*np.pi/180), range * np.sin(theta*np.pi/180), 0))
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "base_link"
        self.dbg_point_pub.publish(point_cloud2.create_cloud_xyz32(header, points))
        
        
        
    

def main(args=None):
    """Initializes a node, runs it, and cleans up after termination.
    Input: args(list) -- list of arguments to pass into rclpy. Default None.
    """
    rclpy.init(args=args)      # Initialize communication with ROS
    node = ObjectDetectionNode()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()