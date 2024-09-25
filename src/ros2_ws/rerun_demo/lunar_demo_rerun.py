#!/usr/bin/env python3
"""
Simple example of a ROS node that republishes some common types to Rerun.

The solution here is mostly a toy example to show how ROS concepts can be
mapped to Rerun. Fore more information on future improved ROS support,
see the tracking issue: <https://github.com/rerun-io/rerun/issues/1537>.

NOTE: Unlike many of the other examples, this example requires a system installation of ROS
in addition to the packages from requirements.txt.
"""

from __future__ import annotations

import argparse
import sys

import numpy as np
import rerun as rr  # pip install rerun-sdk

try:
    import cv_bridge
    import laser_geometry
    import rclpy
    import rerun_urdf
    import trimesh
    from image_geometry import PinholeCameraModel
    from nav_msgs.msg import Odometry
    from numpy.lib.recfunctions import structured_to_unstructured
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile
    from rclpy.time import Duration, Time
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField, Imu, LaserScan, JointState
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import String
    from tf2_ros import TransformException
    from tf2_ros.buffer import Buffer
    from tf2_ros.transform_listener import TransformListener

except ImportError:
    print(
        """
Could not import the required ROS2 packages.

Make sure you have installed ROS2 (https://docs.ros.org/en/humble/index.html)
and sourced /opt/ros/humble/setup.bash

See: README.md for more details.
"""
    )
    sys.exit(1)


class PragyaanSubscriber(Node):  # type: ignore[misc]
    def __init__(self) -> None:
        super().__init__("pragyaan_rover")

        # Used for subscribing to latching topics
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        # Allow concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Subscribe to TF topics
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Define a mapping for transforms
        self.path_to_frame = {
            "world": "world",
            "world/points": "depth_cam",
            "world/robot": "base_link",
            "world/robot/depth_camera": "depth_cam",
            "world/robot/depth_camera/points": "depth_cam",
            # "world/robot/left_camera": "NavCam_left",
            # "world/robot/left_camera/points": "camera_depth_frame",
            # "world/robot/right_camera": "NavCam_right",
            # "world/robot/right_camera/points": "camera_depth_frame",
        }

        # Assorted helpers for data conversions
        self.model = PinholeCameraModel()
        self.cv_bridge = cv_bridge.CvBridge()
        self.laser_proj = laser_geometry.laser_geometry.LaserProjection()

        # Log a bounding box as a visual placeholder for the map
        # TODO(jleibs): Log the real map once [#1531](https://github.com/rerun-io/rerun/issues/1531) is merged
        rr.log(
            "world/box",
            rr.Boxes3D(half_sizes=[3, 3, 1], centers=[0, 0, 1], colors=[255, 255, 255, 255]),
            static=True,
        )

        # Subscriptions
        self.info_sub = self.create_subscription(
            CameraInfo,
            "/depth_cam/camera_info",
            self.cam_info_callback,
            10,
            callback_group=self.callback_group,
        )

        self.img_sub = self.create_subscription(
            Image,
            "/depth_cam/rgb",
            self.rgb_image_callback,
            10,
            callback_group=self.callback_group,
        )

        self.depth_img_sub = self.create_subscription(
            Image,
            "/depth_cam/depth",
            self.depth_image_callback,
            10,
            callback_group=self.callback_group,
        )

        self.points_sub = self.create_subscription(
            PointCloud2,
            "/depth_cam/depth_pcl",
            self.points_callback,
            10,
            callback_group=self.callback_group,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "odom",
            self.odom_callback,
            10,
            callback_group=self.callback_group,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "odom",
            self.odom_callback,
            10,
            callback_group=self.callback_group,
        )

        self.imu_sub = self.create_subscription(
            Imu,
            "imu",
            self.imu_callback,
            10,
            callback_group=self.callback_group,
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            "joint_states",
            self.joint_states_callback,
            10,
            callback_group=self.callback_group,
        )


    def log_tf_as_transform3d(self, path: str, time: Time) -> None:
        """
        Helper to look up a transform with tf and log using `log_transform3d`.

        Note: we do the lookup on the client side instead of re-logging the raw transforms until
        Rerun has support for Derived Transforms [#1533](https://github.com/rerun-io/rerun/issues/1533)
        """
        # Get the parent path
        parent_path = path.rsplit("/", 1)[0]

        # Find the corresponding frames from the mapping
        child_frame = self.path_to_frame[path]
        parent_frame = self.path_to_frame[parent_path]

        # Do the TF lookup to get transform from child (source) -> parent (target)
        try:
            tf = self.tf_buffer.lookup_transform(parent_frame, child_frame, time, timeout=Duration(nanoseconds=100000000))
            t = tf.transform.translation
            q = tf.transform.rotation
            rr.log(path, rr.Transform3D(translation=[t.x, t.y, t.z], rotation=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w])))
        except TransformException as ex:
            print(f"Failed to get transform: {ex}")

    def cam_info_callback(self, info: CameraInfo) -> None:
        """Log a `CameraInfo` with `log_pinhole`."""
        time = Time.from_msg(info.header.stamp)
        rr.set_time_nanos("ros_time", time.nanoseconds)

        self.model.fromCameraInfo(info)

        rr.log(
            "world/robot/depth_camera/depth_simg",
            rr.Pinhole(
                resolution=[self.model.width, self.model.height],
                image_from_camera=self.model.intrinsicMatrix(),
            ),
        )

    def rgb_image_callback(self, img: Image) -> None:
        """Log an `Image` with `log_image` using `cv_bridge`."""
        time = Time.from_msg(img.header.stamp)
        rr.set_time_nanos("ros_time", time.nanoseconds)

        rr.log("world/robot/depth_camera/rgb_img", rr.Image(self.cv_bridge.imgmsg_to_cv2(img)))
        self.log_tf_as_transform3d("world/robot/depth_camera", time)

    def depth_image_callback(self, img: Image) -> None:
        """Log an `Image` with `log_image` using `cv_bridge`."""
        time = Time.from_msg(img.header.stamp)
        rr.set_time_nanos("ros_time", time.nanoseconds)

        rr.log("world/robot/depth_camera/depth_img", rr.Image(self.cv_bridge.imgmsg_to_cv2(img)))
        self.log_tf_as_transform3d("world/robot/depth_camera", time)

    def points_callback(self, points: PointCloud2) -> None:
        """Log a `PointCloud2` with `log_points`."""
        time = Time.from_msg(points.header.stamp)
        rr.set_time_nanos("ros_time", time.nanoseconds)

        pts = point_cloud2.read_points(points, field_names=["x", "y", "z"], skip_nans=True)

        # Comment or remove the color fields if they're not available
        # colors = point_cloud2.read_points(points, field_names=["r", "g", "b"], skip_nans=True)

        # colors = point_cloud2.read_points(points, field_names=["r", "g", "b"], skip_nans=True)

        pts = structured_to_unstructured(pts)
        # colors = colors = structured_to_unstructured(colors)

        # Log only the points if colors are not available
        rr.log("world/robot/depth_camera/points", rr.Points3D(pts, colors=[255, 0, 0, 255]))
        self.log_tf_as_transform3d("world/robot/depth_camera/points", time)


    def odom_callback(self, odom: Odometry) -> None:
        """Update transforms when odom is updated."""
        time = Time.from_msg(odom.header.stamp)
        rr.set_time_nanos("ros_time", time.nanoseconds)

        # Capture time-series data for the linear and angular velocities
        rr.log("odometry/x", rr.Scalar(odom.pose.pose.position.x))
        rr.log("odometry/y", rr.Scalar(odom.pose.pose.position.y))
        rr.log("odometry/z", rr.Scalar(odom.pose.pose.position.z))

        # Update the robot pose itself via TF
        self.log_tf_as_transform3d("world/robot", time)


    def imu_callback(self, imu: Imu) -> None:
        """Log IMU data """
        time = Time.from_msg(imu.header.stamp)
        rr.set_time_nanos("ros_time", time.nanoseconds)

        # Capture time-series data for the linear and angular velocities
        rr.log("imu/orientation/x", rr.Scalar(imu.orientation.x))
        rr.log("imu/orientation/y", rr.Scalar(imu.orientation.y))
        rr.log("imu/orientation/z", rr.Scalar(imu.orientation.z))

    def joint_states_callback(self, jointstates: JointState) -> None:
        """ Log Joint states data"""
        time = Time.from_msg(jointstates.header.stamp)
        rr.set_time_nanos("ros_time", time.nanoseconds)

        # Capture time-series data for the linear and angular velocities
        rr.log("joint_states/position/wheel_fl", rr.Scalar(jointstates.position[2]))
        rr.log("joint_states/position/wheel_fr", rr.Scalar(jointstates.position[4]))
        rr.log("joint_states/position/wheel_bl", rr.Scalar(jointstates.position[6]))
        rr.log("joint_states/position/wheel_br", rr.Scalar(jointstates.position[9]))
        rr.log("joint_states/position/wheel_ml", rr.Scalar(jointstates.position[7]))
        rr.log("joint_states/position/wheel_mr", rr.Scalar(jointstates.position[8]))


def main() -> None:
    parser = argparse.ArgumentParser(description="Simple example of a ROS node that republishes to Rerun.")
    rr.script_add_args(parser)
    args, unknownargs = parser.parse_known_args()
    rr.script_setup(args, "rerun_example_ros_node")

    # Any remaining args go to rclpy
    rclpy.init(args=unknownargs)

    pragyaan_subscriber = PragyaanSubscriber()

    # Use the MultiThreadedExecutor so that calls to `lookup_transform` don't block the other threads
    rclpy.spin(pragyaan_subscriber, executor=rclpy.executors.MultiThreadedExecutor())

    pragyaan_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
