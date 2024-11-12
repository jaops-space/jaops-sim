#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# from tf_transformations import euler_from_quaternion

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


class RectangularTrajectoryNode(Node):
    def __init__(self):
        super().__init__('rectangular_trajectory_node')

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for odometry
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Parameters for movement
        self.target_distance = 5.0  # meters
        self.target_angle = math.radians(90)  # 90 degrees
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.5  # rad/s
        self.state = 'MOVE_FORWARD'

        # Initial position and orientation
        self.start_x = None
        self.start_y = None
        self.start_yaw = None

    def odom_callback(self, msg):
        # Get the current position and orientation
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        current_z = msg.pose.pose.position.z
        orientation_q = msg.pose.pose.orientation
        _, _, current_yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])

        if self.state == 'MOVE_FORWARD':
            # Store the starting position for distance measurement
            if self.start_x is None and self.start_y is None:
                self.start_x = current_x
                self.start_y = current_y

            # Calculate the distance from the starting point
            distance_travelled = math.sqrt((current_x - self.start_x)**2 + (current_y - self.start_y)**2)
            
            if distance_travelled < self.target_distance:
                self.publish_velocity(self.linear_speed, 0.0)
            else:
                # Switch to the next state to rotate
                self.state = 'ROTATE'
                self.start_yaw = current_yaw  # Set initial yaw for rotation

        elif self.state == 'ROTATE':
            # Calculate the rotation needed
            if self.start_yaw is None:
                self.start_yaw = current_yaw

            yaw_difference = (current_yaw - self.start_yaw + math.pi) % (2 * math.pi) - math.pi

            if abs(yaw_difference) < self.target_angle:
                self.publish_velocity(0.0, self.angular_speed)
            else:
                # Complete the rotation and reset for next move
                self.state = 'MOVE_FORWARD'
                self.start_x = None
                self.start_y = None
                self.start_yaw = None

    def publish_velocity(self, linear, angular):
        """Helper function to publish a Twist message with specified linear and angular velocities."""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = RectangularTrajectoryNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
