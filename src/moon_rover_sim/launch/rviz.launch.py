import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directory
    pkg_share = get_package_share_directory('moon_rover_sim')

    # RViz config file
    rviz_config = os.path.join(pkg_share, 'rviz', 'rover.rviz')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # RViz2 node
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        rviz2
    ])
