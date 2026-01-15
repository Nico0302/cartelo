from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from launch.conditions import UnlessCondition, IfCondition


def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix of the joint names, useful for multi-robot setup.",
        ),
        DeclareLaunchArgument(
            "teleoperation_package",
            description='Package with the teleoperation configuration in "config" folder.',
        ),
        DeclareLaunchArgument(
            "teleoperation_file",
            default_value="cartesian_teleoperation.yaml",
            description="YAML file with the teleoperation configuration.",
        ),
        DeclareLaunchArgument(
            "use_gripper",
            default_value="true",
            description="Load a gripper node to control the robot gripper.",
        ),
        DeclareLaunchArgument(
            "use_twist",
            default_value="false",
            description="Utilize twist teleoperation instead of pose inputs (e.g. for spacenav/twist).",
        ),
    ]

    # Configuration variables
    teleoperation_package = LaunchConfiguration("teleoperation_package")
    teleoperation_file = LaunchConfiguration("teleoperation_file")
    use_gripper = LaunchConfiguration("use_gripper")
    use_twist = LaunchConfiguration("use_twist")

    teleoperation_config = ParameterFile(
        PathJoinSubstitution(
            [
                FindPackageShare(teleoperation_package),
                "config",
                teleoperation_file,
            ]
        ),
        allow_substs=True,
    )

    composed_node = ComposableNodeContainer(
        name="teleoperation_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="both",
    )

    pose_teleoperation = LoadComposableNodes(
        target_container='teleoperation_container',
        condition=UnlessCondition(use_twist),
        composable_node_descriptions=[
            ComposableNode(
                package="cartelo",
                plugin="cartelo::PoseTeleoperation",
                name="pose_teleoperation",
                parameters=[
                    teleoperation_config
                ],
            )
        ]
    )

    twist_teleoperation = LoadComposableNodes(
        target_container='teleoperation_container',
        condition=IfCondition(use_twist),
        composable_node_descriptions=[
            ComposableNode(
                package="cartelo",
                plugin="cartelo::TwistTeleoperation",
                name="twist_teleoperation",
                parameters=[
                    teleoperation_config
                ],
            )
        ]
    )

    gripper_teleoperation = LoadComposableNodes(
        target_container='teleoperation_container',
        condition=IfCondition(use_gripper),
        composable_node_descriptions=[
            ComposableNode(
                package="cartelo",
                plugin="cartelo::GripperTeleoperation",
                name="gripper_teleoperation",
                parameters=[
                    teleoperation_config
                ],
            )
        ]
    )

    return LaunchDescription(declared_arguments + [composed_node, pose_teleoperation, twist_teleoperation, gripper_teleoperation])
