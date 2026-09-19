import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Pose, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan, PointCloud2
from typing import Literal, Optional, List, Tuple
from sensor_msgs_py import point_cloud2
try:
    from sklearn.cluster import AgglomerativeClustering
except ImportError:
    print("Object detection requires scikit-learn. Run `python3 -m pip install --break-system-packages scikit-learn`.")
    raise SystemExit

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__("object_detection_node")
        self.scan_data_sub = self.create_subscription(LaserScan, 'stable_scan', self.on_scan_data, 10)
        self.obj_pub = self.create_publisher(PointCloud2, 'detected_clusters', 10)
        self.dbg_point_pub = self.create_publisher(PointCloud2, 'laser_points', 10)
        # TODO: make these ROS params
        self.min_range = 0.2
        self.max_range = 3
        self.cluster_dist_threshold = 0.2 # m, points farther apart than this will be considered separate clusters
        self.min_cluster_size = 5 # cluster with fewer points wont be considered an object

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

        if len(points) <= 1:
            # too few points - no need to detect clusters
            object_positions = []
        else:
            # Find clusters of points in space
            points = np.array(points) # convert to numpy array for convenience

            clusterer = AgglomerativeClustering(n_clusters=None, distance_threshold=self.cluster_dist_threshold, linkage='single') # set up clustering algorithm
            cluster_labels = clusterer.fit_predict(points)

            object_positions: List[Tuple[float, float]] = [] # list of locations of detected objects

            for cluster_label in np.unique(cluster_labels):
                points_in_cluster = points[cluster_labels == cluster_label]
                if len(points_in_cluster) < self.min_cluster_size:
                    continue
                # it's debatable whether the representative point for the cluster should be the closest point to the neato or the centroid of all points
                # this one is the closest point to the neato
                representative_point = points_in_cluster[np.argmin(np.linalg.norm(points_in_cluster, axis=1))]
                print(f"Object detected! {len(points_in_cluster)} points, rep point: {representative_point} ({np.linalg.norm(representative_point)}m away)")
                object_positions.append(tuple(representative_point))

        # finally, convert the list of object positions into a PointCloud2 for publication
        # OPT: stay in numpy arrays the entire time
        self.obj_pub.publish(point_cloud2.create_cloud_xyz32(header, object_positions))

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