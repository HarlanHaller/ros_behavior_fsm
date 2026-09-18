""" This script explores publishing ROS messages in ROS using Python """
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Pose, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from typing import Literal, Optional, Tuple
from sensor_msgs_py import point_cloud2


class AttackNode(Node):
    """This node will handle the patrol/wall-following behavior."""
    def __init__(self):
        super().__init__('attack_node')
        # Create a timer that fires ten times per second
        timer_period = 0.1
        # self.vel_timer = self.create_timer(timer_period, self.compute_and_send_vel)
        # self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_publisher = self.create_publisher(String, 'current_state', 10)
        self.state_subscriber = self.create_subscription(String, 'current_state', self.update_state, 10)
        # self.detected_objects_subscriber = self.create_subscription(PointCloud2, 'detected_clusters', self.on_detected_objects, 10)
        self.active = True
        # self.current_velocity: Tuple[float, float] = (0.0, 0.0) # forward vel (m/s), angular vel (rad/s)
        
    def update_state(self, msg: String):
        self.active = (msg.data == "attack")
        print(f"Node: 'attack' -- recived state: {msg.data}, active?: {self.active}")
        
        
def main(args=None):
    """Initializes a node, runs it, and cleans up after termination.
    Input: args(list) -- list of arguments to pass into rclpy. Default None.
    """
    rclpy.init(args=args)      # Initialize communication with ROS
    node = AttackNode()   # Create our Node
    try:
        rclpy.spin(node)           # Run the Node until ready to shutdown
    finally:
        # print("exiting cleanly") # Testing if this would code would run
        rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()

        
    