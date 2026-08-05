from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler,IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition,UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Initialize Arguments
    use_sim = LaunchConfiguration("use_sim")
    sim_backend = LaunchConfiguration("sim_backend")
    namespace = LaunchConfiguration("namespace")

    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Whether to run in simulation (Gazebo)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_backend",
            default_value="gazebo",
            description="Which simulator when use_sim is true: gazebo or isaac",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace for the robot"
        )
    )

    use_gazebo = PythonExpression(["'", use_sim, "' == 'true' and '", sim_backend, "' == 'gazebo'"])
    need_control_node = PythonExpression(["not (", use_gazebo, ")"])

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("myagv_plus_controller"),
            "config",
            "controllers.yaml",
        ]
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("myagv_plus_description"),
                "urdf",
                "myagv_plus.urdf.xacro"
            ]),
            " ",
            "use_sim:=", use_sim,
            " ",
            "sim_backend:=", sim_backend,
            " ",
            "controller_config:=", robot_controllers,
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    def launch_gz_sim(context, *args, **kwargs):
        if not IfCondition(use_gazebo).evaluate(context):
            return []
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ])
            )
        )]

    gz_sim = OpaqueFunction(function=launch_gz_sim)

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "robot_description"),
            #("mecanum_drive_controller/reference_unstamped", "/cmd_vel"),
            ("mecanum_drive_controller/reference", "/cmd_vel"),
            ("mecanum_drive_controller/odometry", "/odom"),
            ("mecanum_drive_controller/tf_odometry", "/tf"),
        ],
        condition=IfCondition(need_control_node),
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description,
                    {"publish_frequency": 200.0}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager",
                   PathJoinSubstitution(["/", namespace, "controller_manager"]),
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_drive_controller",
                   "--controller-manager",
                   PathJoinSubstitution(["/", namespace, "controller_manager"]),
        ],
    )


    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    return LaunchDescription(
        [
            *declared_arguments,
            PushRosNamespace(namespace),
            control_node,
            gz_sim,
            robot_state_pub_node,
            joint_state_broadcaster_spawner,
            delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        ]
    )