from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = True
    map_name_arg = DeclareLaunchArgument(
        "map_name", default_value="tb3", description="Name of the map"
    )
    headless_arg = DeclareLaunchArgument(
        "headless", default_value="false", description="Headless mode"
    )
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_base"), "config", "ekf.yaml"]
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_description"), "launch", "description.launch.py"]
    )

    included_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("my_rmf"), "launch", "my_sim.launch.xml"]
            )
        ),
        launch_arguments={
            "map_name": LaunchConfiguration("map_name"),
            "headless": LaunchConfiguration("headless"),
        }.items(),
    )

    spawn_robot_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="urdf_spawner",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "linorobot2",
            "-x",
            # "10",
            "30",
            "-y",
            # "-8",
            "-22",
        ],
    )

    lino_timeout_node = Node(
        package="linorobot2_gazebo",
        executable="command_timeout.py",
        name="command_timeout",
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}, ekf_config_path],
        remappings=[("odometry/filtered", "odom")],
    )

    lino_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_path),
        launch_arguments={
            "use_sim_time": str(use_sim_time),
            "publish_joints": "false",
        }.items(),
    )

    return LaunchDescription(
        [
            map_name_arg,
            headless_arg,
            included_launch,
            lino_launch,
            spawn_robot_node,
            lino_timeout_node,
            ekf_node,
        ]
    )
