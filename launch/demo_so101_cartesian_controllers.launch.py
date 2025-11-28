from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetRemap, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
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
            "use_fake_hardware",
            default_value="false",
            description="Use mock system",
        ),
    ]

    # Configuration variables
    prefix = LaunchConfiguration("prefix")
    usb_port = LaunchConfiguration("usb_port")
    use_sim = LaunchConfiguration("use_sim")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    vive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("cartelo"), "launch", "demo_vive.launch.py"]
            )
        )
    )

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
                    "controllers_file": "ros2_cartesian_controllers.yaml",
                    "arm_controller": "cartesian_motion_controller",
                }.items(),
            ),
        ]
    )

    pose_teleoperation = ComposableNode(
        package="cartelo",
        plugin="cartelo::PoseTeleoperation",
        name="pose_teleoperation",
        parameters=[
            {
                "end_effector_frame_id": "gripper_frame_link",
                "bounds.z_min": 0.01,
                "bounds.z_max": 0.4,
                "bounds.x_min": -0.2,
                "bounds.x_max": 0.4,
                "bounds.y_min": -0.4,
                "bounds.y_max": 0.4,
                "use_relative_pose": False,
                "update_rate": 100,
                "home.joint_controller_name": "joint_trajectory_controller",
                "cartesian_controller_name": "cartesian_motion_controller",
                "home.joint_names": [
                    "elbow_flex",
                    "shoulder_lift",
                    "shoulder_pan",
                    "wrist_flex",
                    "wrist_roll",
                ],
                "home.joint_positions": [1.578, -1.7453, 0.0, 1.2, 0.0],
                "joystick.home_button": 3,
            }
        ],
    )

    gripper_teleoperation = ComposableNode(
        package="cartelo",
        plugin="cartelo::GripperTeleoperation",
        name="gripper_teleoperation",
        parameters=[
            {
                "joint_name": "gripper",
                "max_target_value": 1.745329252,
                "min_target_value": -0.1745329252,
                "gripper_command_topic": "/gripper_controller/gripper_cmd",
                "joystick.toggle_button": 0,
                "joystick.gripper_axis": 1,
            }
        ],
    )

    vive_node = ComposableNode(
        package="libsurvive_ros2",
        plugin="libsurvive_ros2::Component",
        name="libsurvive_ros2_component",
        parameters=[
            {"driver_args": f"--v 100 --force-calibrate"},
            {"tracking_frame": "teleop_world"},
            {"imu_topic": "imu"},
            {"joy_topic": "joy"},
            {"cfg_topic": "cfg"},
            {"lighthouse_rate": 4.0},
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
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

    nodes = [
        so101_bringup_launch,
        composed_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
