from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetRemap


def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix of the joint names, useful for multi-robot setup.",
        ),
        DeclareLaunchArgument(
            "usb_port",
            default_value="/dev/ttyACM0",
            description="USB port for the robot.",
        ),
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Use Gazebo sim",
        ),
        DeclareLaunchArgument(
            "use_vive",
            default_value="false",
            description="Use Vive system",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use mock system",
        ),
        DeclareLaunchArgument(
            "controller_frame_id",
            default_value="right_hand",
            description="The frame ID for the controller",
        ),
    ]

    # Configuration variables
    prefix = LaunchConfiguration("prefix")
    usb_port = LaunchConfiguration("usb_port")
    use_sim = LaunchConfiguration("use_sim")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    controller_frame_id = LaunchConfiguration("controller_frame_id")
    use_vive = LaunchConfiguration("use_vive")

    so101_bringup_launch = GroupAction(
        actions=[
            SetRemap(
                src="/cartesian_motion_controller/target_frame", dst="/target_pose"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("so101_description"),
                            "launch",
                            "bringup.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "prefix": prefix,
                    "usb_port": usb_port,
                    "use_sim": use_sim,
                    "use_fake_hardware": use_fake_hardware,
                    "controllers_file": "cartesian_controllers.yaml",
                    "arm_controller": "cartesian_motion_controller",
                }.items(),
            ),
        ]
    )

    teleop_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("cartelo"),
                            "launch",
                            "teleop.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "controller_frame_id": controller_frame_id,
                    "use_vive": use_vive,
                    "teleoperation_package": "so101_description",
                    "teleoperation_file": "cartesian_teleoperation.yaml",
                }.items(),
            ),
        ]
    )

    nodes = [
        so101_bringup_launch,
        teleop_launch,
    ]

    return LaunchDescription(declared_arguments + nodes)
