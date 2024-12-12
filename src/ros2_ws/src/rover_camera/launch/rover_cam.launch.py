from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time for nodes'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Encoding converting node since foxglove depth camera features are available only for 16UC1
    depth_cam_encoding_node = Node(
        package='rover_camera',
        executable='depth_encoding.py',
        name='depth_encoding',
        parameters=[{'use_sim_time': use_sim_time}]
    )


    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,

        depth_cam_encoding_node,

    ])
