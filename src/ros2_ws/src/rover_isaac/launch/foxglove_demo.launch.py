from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the launch arguments
    send_buffer_limit_arg = DeclareLaunchArgument(
        'send_buffer_limit',
        default_value='1000000000',
        description='Buffer limit for sending data in foxglove bridge'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time for nodes'
    )

    # Get launch configurations
    send_buffer_limit = LaunchConfiguration('send_buffer_limit')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Node for foxglove_bridge
    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'send_buffer_limit': send_buffer_limit,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # Encoding converting node since foxglove depth camera features are available only for 16UC1
    depth_cam_encoding_node = Node(
        package='rover_camera',
        executable='depth_encoding.py',
        name='depth_encoding',
        parameters=[{'use_sim_time': use_sim_time}]
    )


    return LaunchDescription([
        # Launch arguments
        send_buffer_limit_arg,
        use_sim_time_arg,

        foxglove_node,
        depth_cam_encoding_node,

    ])
