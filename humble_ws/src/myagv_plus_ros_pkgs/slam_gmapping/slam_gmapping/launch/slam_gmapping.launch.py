import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch.actions
import launch_ros.actions


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    rviz_config_dir = os.path.join(
        get_package_share_directory('slam_gmapping'),
        'rviz',
        'gmapping.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Isaac Sim) clock if true'),

        launch_ros.actions.Node(
            package='slam_gmapping', executable='slam_gmapping', output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),

        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),

    ])
