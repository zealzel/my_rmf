from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression 
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = True
    map_name_arg = DeclareLaunchArgument(
        "map_name", default_value="tb3", description="Name of the map"
    )
    headless_arg = DeclareLaunchArgument(
        "headless", default_value="false", description="Headless mode"
    )

    map_defaultpos = {
        "tb3": [10, -8, 0.05],
        "fit_office": [7, -17, 2],
        "factory_bk": [30, -22, 0.05],
    }
    # x_arg = DeclareLaunchArgument("x", default_value="0", description="x position")
    # y_arg = DeclareLaunchArgument("y", default_value="0", description="y position")
    # z_arg = DeclareLaunchArgument("z", default_value="0", description="z position")
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

    spawn_robot_node_tb3 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="urdf_spawner",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "linorobot2",
            "-x", str(map_defaultpos["tb3"][0]),
            "-y", str(map_defaultpos["tb3"][1]),
            "-z", str(map_defaultpos["tb3"][2]),
        ],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('map_name'), "' == 'tb3'"]))
    )

    spawn_robot_node_fit_office = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="urdf_spawner",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "linorobot2",
            "-x", str(map_defaultpos["fit_office"][0]),
            "-y", str(map_defaultpos["fit_office"][1]),
            "-z", str(map_defaultpos["fit_office"][2]),
        ],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('map_name'), "' == 'fit_office'"]))
    )

    spawn_robot_node_factory_bk = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="urdf_spawner",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "linorobot2",
            "-x", str(map_defaultpos["factory_bk"][0]),
            "-y", str(map_defaultpos["factory_bk"][1]),
            "-z", str(map_defaultpos["factory_bk"][2]),
        ],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('map_name'), "' == 'factory_bk'"]))
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
            # x_arg,
            # y_arg,
            # z_arg,
            included_launch,
            lino_launch,
            spawn_robot_node_tb3,
            spawn_robot_node_fit_office,
            spawn_robot_node_factory_bk,
            lino_timeout_node,
            ekf_node,
        ]
    )
