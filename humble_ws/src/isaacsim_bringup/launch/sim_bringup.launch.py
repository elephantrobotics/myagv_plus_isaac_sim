from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    desc_share = get_package_share_directory("myagv_plus_description")
    rviz_config = os.path.join(
        os.path.abspath(os.path.join(desc_share, "..", "..", "..", "..")),
        "src", "myagv_plus_description", "rviz", "myagv_plus_display.rviz")
    if not os.path.isfile(rviz_config):
        rviz_config = os.path.join(desc_share, "rviz", "myagv_plus_display.rviz")

    use_rviz = LaunchConfiguration("use_rviz")
    namespace = LaunchConfiguration("namespace")

    declared_arguments = [
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("namespace", default_value=""),
    ]

    bringup = GroupAction([
        SetParameter("use_sim_time", True),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("myagv_plus_bringup"), "launch", "myagv_plus_bringup.launch.py"])),
            launch_arguments={
                "namespace": namespace,
                "use_sim": "true",
                "sim_backend": "isaac",
                "enable_esp32": "false",
                "enable_csi_camera": "false",
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])

    return LaunchDescription(declared_arguments + [bringup])
