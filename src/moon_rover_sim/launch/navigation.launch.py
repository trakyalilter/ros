import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_share = get_package_share_directory('moon_rover_sim')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    # File paths
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'nav2.rviz')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # SLAM Toolbox Node
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': use_sim_time}]
    )

    # Nav2 Bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params,
            'autostart': 'true'
        }.items()
    )

    # RViz2 Node
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
        slam_toolbox,
        nav2_bringup,
        rviz2
    ])
