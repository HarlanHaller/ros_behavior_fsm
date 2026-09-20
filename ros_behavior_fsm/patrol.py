""" This script explores publishing ROS messages in ROS using Python """
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Pose, Quaternion, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from typing import Literal, Optional, Tuple
from sensor_msgs_py import point_cloud2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs


class PatrolNode(Node):
    """This node will handle the patrol/wall-following behavior."""
    def __init__(self):
        super().__init__('patrol_node')
        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.vel_timer = self.create_timer(timer_period, self.compute_and_send_vel)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_publisher = self.create_publisher(String, 'current_state', 10)
        self.state_subscriber = self.create_subscription(String, 'current_state', self.update_state, 10)
        self.detected_objects_subscriber = self.create_subscription(PointCloud2, 'detected_clusters', self.on_detected_objects, 10)
        self.dead_zone_subscriber = self.create_subscription(PointStamped, 'dead_zone', self.update_dead_zone, 10)
        self.active = True
        self.dead_zone: PointStamped = None
        self.current_velocity: Tuple[float, float] = (0.0, 0.0) # forward vel (m/s), angular vel (rad/s)
        self.declare_parameter('dead_zone_radius', 0.25)
        self.declare_parameter('wall_follow_distance', 0.3) # m
        self.declare_parameter('patrol_speed', 0.15) # m/s
        self.declare_parameter('linear_weight', 0.6) # rad/s/m, how much we should turn to correct distance issues
        self.declare_parameter('angular_weight', 0.6) # rad/s/rad, how much we should turn to correct angle issues

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
        object_positions = point_cloud2.read_points_numpy(msg).transpose()
        print(object_positions)
        # figure out which object is our protected object
        protected_object = None
        if np.shape(object_positions)[1] == 1:
            # if we only see one object, that's what we're protecting
            protected_object = object_positions[:, 0]
        elif np.shape(object_positions)[1] > 1:
            # if we see more than one object, we protect the one closest to 90deg=pi/2 rad
            
            if self.dead_zone is not None:
                self.dead_zone.header.stamp = rclpy.time.Time()
                dead_zone_base_link = self.tf_buffer.transform(self.dead_zone, 'base_link', )
                # print(f'dead zone in base link at x: {dead_zone_base_link.point.x}, y: {dead_zone_base_link.point.y}')
                tmp = np.array([dead_zone_base_link.point.x, dead_zone_base_link.point.y, 0])
                dead_zone_np = tmp[:, np.newaxis]
                print(f'dist to dead zone: {np.linalg.norm(object_positions-dead_zone_np, axis=0)}')
                object_positions = object_positions[:, (np.linalg.norm(object_positions-dead_zone_np, axis=0))>self.get_parameter('dead_zone_radius').value]
                # print(f'filtered: {object_positions}')
                if np.shape(object_positions)[1] < 1:
                    # we filtered out all our object positions
                    print("WARNING: filtered out all positions")
                    return               
            
            # object_thetas = np.array([np.arctan2(obj.y, obj.x) for obj in object_positions])
            object_thetas = np.arctan2(object_positions[1, :], object_positions[0, :])
            # print(f'thetas{object_thetas}')
            
            protected_object =  object_positions[:, np.argmin(np.abs(object_thetas-np.pi/2))]
            # print(protected_object)
            # if there's a suspicious object (object to our outside, so object_theta<pi), then go to the suspicious state
            # print(object_thetas, flush=True)
            if any(object_thetas < 0 ):
                # print(f"GRRRRR SUSPICIOUS OBJECT DETECTED (from {object_positions} with angles {object_thetas})")
                print(f"GRRRRR SUSPICIOUS OBJECT DETECTED")
                self.state_publisher.publish(String(data="suspicious"))
                self.active = False
                return
        else:
            # we detected no objects
            print("WARNING: Patrol had no objects to follow")
            return # we can't make any updates to our velocity, let's just hope something appears next time (:

        # print(protected_object)

        # wall-following algorithm
        # our goal is to keep the object wall_follow_distance away and at 270 degrees
        # if it's to far away or too far backward, we should turn to the left a bit
        # if it's too close or too far forward, we should turn to the right a bit
        linear_error = np.linalg.norm(protected_object) - self.get_parameter('wall_follow_distance').value
        linear_term = self.get_parameter('linear_weight').value * linear_error
        angular_error = (np.arctan2(protected_object[1], protected_object[0]) - np.pi/2)
        angular_term = self.get_parameter('angular_weight').value * angular_error
        # we'll proceed at the same forward velocity no mater what
        angular_velocity = float(linear_term + angular_term)
        self.current_velocity = (self.get_parameter('patrol_speed').value, angular_velocity)
        # print(f"Using angvel={angular_velocity:.3f} to correct linear error of {linear_error:.3f}m and angular error of {angular_error:.3f}rad")

    def compute_and_send_vel(self):
        if self.active:
            #print(f"sending velocity command for {self.current_velocity}")
            twist_msg = Twist(linear=Vector3(x=self.current_velocity[0],y=0.0,z=0.0), angular=Vector3(x=0.0,y=0.0,z=self.current_velocity[1]))
            self.vel_publisher.publish(twist_msg)
            
    def update_dead_zone(self, msg:PointStamped):
        # print(f'recived dead zone x: {msg.point.x}, y: {msg.point.y}')
        self.dead_zone = msg

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
