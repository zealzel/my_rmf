import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_name_arg = DeclareLaunchArgument(
        "map_name", default_value="tb3", description="Name of the map"
    )
    headless_arg = DeclareLaunchArgument(
        "headless", default_value="false", description="Headless mode"
    )
    # x,y = 10,-8,0.05 (tb3)
    # x,y = 30,-22,0.05 (factory)
    # x_arg = DeclareLaunchArgument("x", default_value="30", description="x position")
    # y_arg = DeclareLaunchArgument("y", default_value="-22", description="y position")
    x_arg = DeclareLaunchArgument("x", default_value="10", description="x position")
    y_arg = DeclareLaunchArgument("y", default_value="-8", description="y position")
    z_arg = DeclareLaunchArgument("z", default_value="0.05", description="z position")

    robot_package_name = "articubot_one"

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(robot_package_name),
                    "launch",
                    "rsp.launch.py",
                )
            ]
        ),
        launch_arguments={"use_sim_time": "true", "use_ros2_control": "true"}.items(),
    )

    twist_mux_params = os.path.join(
        get_package_share_directory(robot_package_name), "config", "twist_mux.yaml"
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {"use_sim_time": True}],
        remappings=[("/cmd_vel_out", "/diff_cont/cmd_vel_unstamped")],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "my_bot",
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            "-z",
            LaunchConfiguration("z"),
        ],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
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

    return LaunchDescription(
        [
            map_name_arg,
            headless_arg,
            x_arg,
            y_arg,
            z_arg,
            included_launch,
            rsp,
            twist_mux,
            spawn_entity,
            diff_drive_spawner,
            joint_broad_spawner,
        ]
    )
