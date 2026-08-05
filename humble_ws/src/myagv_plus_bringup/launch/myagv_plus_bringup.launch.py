from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    enable_esp32 = LaunchConfiguration("enable_esp32")
    enable_csi_camera = LaunchConfiguration("enable_csi_camera")
    use_sim = LaunchConfiguration("use_sim")
    sim_backend = LaunchConfiguration("sim_backend")

    # Launch arguments
    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the robot"
    )

    declare_enable_esp32 = DeclareLaunchArgument(
        "enable_esp32",
        default_value="true",
        description="Enable ESP32 driver node"
    )

    declare_esp32_debug_mode = DeclareLaunchArgument(
        "debug",
        default_value="false",
        description="Enable debug mode for ESP32 driver (logs sent and received frames)"
    )

    declare_enable_csi_camera = DeclareLaunchArgument(
        "enable_csi_camera",
        default_value="false",
        description="Enable CSI camera node"
    )

    declare_use_sim = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="Run against a simulator instead of the real hardware"
    )

    declare_sim_backend = DeclareLaunchArgument(
        "sim_backend",
        default_value="gazebo",
        description="Which simulator when use_sim is true: gazebo or isaac"
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("myagv_plus_controller"),
                "launch",
                "controller.launch.py"
            ])
        ),
        launch_arguments={
            "namespace": namespace,
            "use_sim": use_sim,
            "sim_backend": sim_backend,
        }.items()
    )

    esp32_node = Node(
        package="myagv_plus_esp32_driver",
        executable="esp32_node",
        name="esp32_node",
        output="screen",
        parameters=[{
            "debug_mode": LaunchConfiguration("debug")
        }],
        condition=IfCondition(enable_esp32),
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ydlidar_ros2_driver"),
                "launch",
                "ydlidar_launch.py"
            ])
        ),
        condition=UnlessCondition(use_sim),
    )

    csi_camera_node = Node(
        package="myagv_plus_camera",
        executable="csi_camera",
        name="csi_camera",
        output="screen",
        condition=IfCondition(enable_csi_camera),
    )

    return LaunchDescription([
        declare_namespace,
        declare_enable_esp32,
        declare_enable_csi_camera,
        declare_esp32_debug_mode,
        declare_use_sim,
        declare_sim_backend,

        PushRosNamespace(namespace),

        controller_launch,
        esp32_node,
        lidar_launch,
        csi_camera_node,
    ])