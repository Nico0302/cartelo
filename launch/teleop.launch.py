from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix of the joint names, useful for multi-robot setup.",
        ),
        DeclareLaunchArgument(
            "use_vive",
            default_value="false",
            description="Start libsruvive node",
        ),
        DeclareLaunchArgument(
            "controller_frame_id",
            default_value="right_hand",
            description="The frame ID for the controller",
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
    ]

    # Configuration variables
    controller_frame_id = LaunchConfiguration("controller_frame_id")
    use_vive = LaunchConfiguration("use_vive")
    teleoperation_package = LaunchConfiguration("teleoperation_package")
    teleoperation_file = LaunchConfiguration("teleoperation_file")

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

    pose_teleoperation = ComposableNode(
        package="cartelo",
        plugin="cartelo::PoseTeleoperation",
        name="pose_teleoperation",
        parameters=[
            teleoperation_config,
            {
                "controller_frame_id": controller_frame_id,
            },
        ],
    )

    gripper_teleoperation = ComposableNode(
        package="cartelo",
        plugin="cartelo::GripperTeleoperation",
        name="gripper_teleoperation",
        parameters=[
            teleoperation_config,
            {
                "controller_frame_id": controller_frame_id,
            },
        ],
    )

    vive_node = ComposableNode(
        package="libsurvive_ros2",
        plugin="libsurvive_ros2::Component",
        name="libsurvive_ros2_component",
        parameters=[
            {"driver_args": "--v 100 --force-calibrate"},
            {"tracking_frame": "virtual_world"},
            {"imu_topic": "imu"},
            {"joy_topic": "joy"},
            {"cfg_topic": "cfg"},
            {"lighthouse_rate": 4.0},
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
        condition=IfCondition(use_vive),
    )

    composed_node = ComposableNodeContainer(
        name="teleoperation_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            pose_teleoperation,
            gripper_teleoperation,
            vive_node,
        ],
        output="log",
    )

    return LaunchDescription(declared_arguments + [composed_node])
