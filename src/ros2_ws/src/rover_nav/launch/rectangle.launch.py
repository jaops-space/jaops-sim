# rectangular_trajectory.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Declare launch arguments for side_length and rounds
    side_length_arg = DeclareLaunchArgument(
        'side_length',
        default_value='5.0',
        description='Length of each side of the rectangle in meters'
    )

    rounds_arg = DeclareLaunchArgument(
        'rounds',
        default_value='1',
        description='Number of rounds to complete the rectangular trajectory'
    )

    side_length = LaunchConfiguration('side_length')
    rounds = LaunchConfiguration('rounds')
    
    # Node definition for RectangularTrajectoryNode
    rect_node = Node(
        package='rover_nav',  # Replace with your package name
        executable='rectangular_traj.py',
        name='rectangular_trajectory_node',
        output='screen',
        parameters=[
            {'side_length': side_length},
            {'rounds': rounds}
        ]
    )

    return LaunchDescription([
        side_length_arg,
        rounds_arg,

        rect_node,
    ])
