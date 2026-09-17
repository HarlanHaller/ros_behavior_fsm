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


class PatrolNode(Node):
    """This node will handle the patrol/wall-following behavior."""
    def __init__(self):
        super().__init__('patrol_node')
        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.vel_timer = self.create_timer(timer_period, self.compute_and_send_vel)
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_publisher = self.create_publisher(String, 'current_state', 10)
        self.state_subscriber = self.create_subscription(String, 'current_state', self.update_state, 10)
        self.detected_objects_subscriber = self.create_subscription(PointCloud2, 'detected_clusters', self.on_detected_objects, 10)
        self.active = True
        self.current_velocity: Tuple[float, float] = (0, 0.0) # forward vel (m/s), angular vel (rad/s)
        self.declare_parameter('wall_follow_distance', 0.5) # m
        self.declare_parameter('patrol_speed', 0.15) # m/s
        self.declare_parameter('linear_weight', 0.4) # rad/s/m, how much we should turn to correct distance issues
        self.declare_parameter('angular_weight', 0.2) # rad/s/rad, how much we should turn to correct angle issues

    def update_state(self, msg: String):
        self.active = (msg.data == "patrol")
        print(f"Node: 'patrol' -- recived state: {msg.data}, active?: {self.active}")
        
    def on_detected_objects(self, msg: PointCloud2):
        """Callback function for object detection update. 
        Decide what we're protecting, then figure out how to drive to do that."""
        if not self.active:
            # don't do any math if we aren't the active state
            return
        # extract object positions from the message
        object_positions = point_cloud2.read_points_list(msg)
        # figure out which object is our protected object
        protected_object = None
        if len(object_positions) == 1:
            # if we only see one object, that's what we're protecting
            protected_object = object_positions[0]
        elif len(object_positions) > 1:
            # if we see more than one object, we protect the one closest to 90deg=pi/2 rad
            object_thetas = np.array([np.arctan2(obj.y, obj.x) for obj in object_positions])
            protected_object = object_positions[np.argmin(np.abs(object_thetas-np.pi/2))]
            # if there's a suspicious object (object to our outside, so object_theta<pi), then go to the suspicious state
            if any(object_thetas < np.pi):
                print(f"GRRRRR SUSPICIOUS OBJECT DETECTED (from {object_positions} with angles {object_thetas})")
                self.state_publisher.publish(String(data="suspicious"))
                self.active = False
                return
        else:
            # we detected no objects
            print("WARNING: Patrol had no objects to follow")
            return # we can't make any updates to our velocity, let's just hope something appears next time (:

        # wall-following algorithm
        # our goal is to keep the object wall_follow_distance away and at 270 degrees
        # if it's to far away or too far backward, we should turn to the left a bit
        # if it's too close or too far forward, we should turn to the right a bit
        linear_error = np.linalg.norm(protected_object) - self.get_parameter('wall_follow_distance')
        linear_term = self.get_parameter('linear_weight') * linear_error
        angular_error = (np.arctan2(protected_object.y, protected_object.x) - np.pi/2)
        angular_term = self.get_parameter('angular_weight') * angular_error
        self.current_velocity[1] = linear_term + angular_term
        # we'll proceed at the same forward velocity no mater what
        self.current_velocity[0] = self.get_parameter('patrol_speed')
        print(f"Using vel={self.current_velocity} to correct linear error of {linear_error:.3f}m and angular error of {angular_error:.3f}rad")

    def compute_and_send_vel(self):
        if self.active:
            twist_msg = Twist(linear=Vector3(x=self.current_velocity[0],y=0.0,z=0.0), angular=Vector3(x=0.0,y=0.0,z=self.current_velocity[1]))
            self.vel_publisher.publish(twist_msg)

def main(args=None):
    """Initializes a node, runs it, and cleans up after termination.
    Input: args(list) -- list of arguments to pass into rclpy. Default None.
    """
    rclpy.init(args=args)      # Initialize communication with ROS
    node = PatrolNode()   # Create our Node
    try:
        rclpy.spin(node)           # Run the Node until ready to shutdown
    finally:
        # print("exiting cleanly") # Testing if this would code would run
        rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()
