#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def euler_from_quaternion(x, y, z, w):
    """Convert a quaternion into euler angles (roll, pitch, yaw)."""
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
 
    return roll_x, pitch_y, yaw_z  # in radians


class RectangularTrajectoryNode(Node):
    def __init__(self):
        super().__init__('rectangular_trajectory_node')

        # Parameters
        self.declare_parameter('side_length', 5.0)
        self.declare_parameter('rounds', 1)
        
        self.side_length = self.get_parameter('side_length').value
        self.rounds = self.get_parameter('rounds').value

        # Publisher and subscriber
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Movement parameters
        self.linear_speed = 1.0  # m/s
        self.angular_speed = 0.5  # rad/s
        self.target_angle = math.radians(90)  # 90 degrees in radians
        self.state = 'MOVE_FORWARD'
        
        # Initialize position, orientation, and round tracking
        self.start_x = None
        self.start_y = None
        self.start_yaw = None
        self.sides_completed = 0
        self.rounds_completed = 0

    def odom_callback(self, msg):
        if self.state == 'COMPLETED':
            self.publish_velocity(0.0, 0.0)
            return

        # Get current position and orientation
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, current_yaw = euler_from_quaternion(
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )

        if self.state == 'MOVE_FORWARD':
            # Set start position for this side
            if self.start_x is None and self.start_y is None:
                self.start_x = current_x
                self.start_y = current_y

            # Check distance traveled
            distance_traveled = math.sqrt((current_x - self.start_x)**2 + (current_y - self.start_y)**2)
            if distance_traveled < self.side_length:
                self.publish_velocity(self.linear_speed, 0.0)
            else:
                # Transition to rotate state
                self.state = 'ROTATE'
                self.start_yaw = current_yaw

        elif self.state == 'ROTATE':
            if self.start_yaw is None:
                self.start_yaw = current_yaw

            yaw_difference = (current_yaw - self.start_yaw + math.pi) % (2 * math.pi) - math.pi
            if abs(yaw_difference) < self.target_angle:
                self.publish_velocity(0.0, self.angular_speed)
            else:
                # Completed rotation, reset start positions and update state
                self.sides_completed += 1
                if self.sides_completed >= 4:
                    self.rounds_completed += 1
                    self.sides_completed = 0
                    # Switch to COMPLETED state after finishing all rounds
                    if self.rounds_completed >= self.rounds:
                        self.state = 'COMPLETED'
                        self.get_logger().info("Trajectory completed.")
                        return

                # Reset and prepare for next side
                self.state = 'MOVE_FORWARD'
                self.start_x = None
                self.start_y = None
                self.start_yaw = None

    def publish_velocity(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"{twist.linear.x} {twist.angular.z}")


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
