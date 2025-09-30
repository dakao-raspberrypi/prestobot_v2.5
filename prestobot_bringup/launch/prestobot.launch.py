from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 1. Declare the launch arguments
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="true",
        description="Set to 'false' when launching on the real robot.",
    )

    auto_arg = DeclareLaunchArgument(
        "auto",
        default_value="true",
        description="Set to 'true' to launch the navigation stack.",
    )

    output_topic_arg = DeclareLaunchArgument(
        "output_topic",
        default_value="/diff_drive_controller/cmd_vel",
        description="The output topic for the cmd_vel relay."
    )

    # Arguments for initial robot pose in simulation
    x_pose_arg = DeclareLaunchArgument(
        "x_pose",
        default_value="-33.0",
        description="Initial x position of the robot in the simulation."
    )
    y_pose_arg = DeclareLaunchArgument(
        "y_pose",
        default_value="-16.0",
        description="Initial y position of the robot in the simulation."
    )
    z_pose_arg = DeclareLaunchArgument(
        "z_pose",
        default_value="0.01",
        description="Initial z position of the robot in the simulation."
    )


    # 2. Define Actions to Include/Run

    robot_description_include = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("prestobot_description"),
                    "launch",
                    "robot_description.launch.xml",
                ]
            )
        ),
        launch_arguments={"is_sim": LaunchConfiguration("is_sim")}.items(),
    )

    ros2_control_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("prestobot_controller"),
                    "launch",
                    "ros2_control.launch.py",
                ]
            )
        ),
        launch_arguments={"is_sim": LaunchConfiguration("is_sim")}.items(),
    )

    gazebo_include = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("prestobot_simulation"),
                    "launch",
                    "gazebo.launch.xml",
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration("is_sim")),
        launch_arguments={
            "x_pose": LaunchConfiguration("x_pose"),
            "y_pose": LaunchConfiguration("y_pose"),
            "z_pose": LaunchConfiguration("z_pose"),
        }.items(),
    )

    navigation_include = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("prestobot_navigation"),
                    "launch",
                    "navigation.launch.xml",
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration("auto")),
        launch_arguments={
            "output_topic": LaunchConfiguration("output_topic"),
            "use_sim_time": LaunchConfiguration("is_sim")
        }.items(),
    )

    sllidar_c1_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("sllidar_ros2"),
                    "launch",
                    "sllidar_c1_launch.py",
                ]
            )
        ),
        condition=UnlessCondition(LaunchConfiguration("is_sim")),
    )

    # Add the MPU6050 driver node, conditioned on NOT being in simulation
    mpu6050_driver_node = Node(
        package="your_mpu6050_package",
        executable="mpu6050_driver_script.py",
        name="mpu6050_driver",
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("is_sim")),
    )

    # 3. Return the LaunchDescription, collecting all arguments and actions
    return LaunchDescription(
        [
            # Arguments
            is_sim_arg,
            auto_arg,
            output_topic_arg,
            x_pose_arg,
            y_pose_arg,
            z_pose_arg,

            # Nodes and Includes
            robot_description_include,
            ros2_control_include,
            gazebo_include,
            navigation_include,
            sllidar_c1_include,
            mpu6050_driver_node, # <-- Added the new node to the list
        ]
    )