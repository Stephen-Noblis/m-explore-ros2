#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    setup_path = LaunchConfiguration("setup_path")
    use_rviz = LaunchConfiguration("use_rviz")
    use_nav2 = LaunchConfiguration("use_nav2")

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("clearpath_nav2_demos"),
                "launch",
                "nav2.launch.py",
            ])
        ),
        condition=IfCondition(use_nav2),
        launch_arguments={
            "setup_path": setup_path,
            "namespace": namespace,
        }.items(),
    )

    costmap_republisher = Node(
        package="jackal_costmap_viz",
        executable="costmap_republisher",
        name="costmap_republisher",
        output="screen",
        parameters=[
            {
                "input_topic": ["/", namespace, "/local_costmap/costmap"],
                "output_topic": ["/", namespace, "/local_costmap/occupancy_grid"],
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=[
            "--ros-args",
            "-r", ["/tf:=/", namespace, "/tf"],
            "-r", ["/tf_static:=/", namespace, "/tf_static"],
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="j100_0001",
            description="Robot namespace",
        ),

        DeclareLaunchArgument(
            "setup_path",
            default_value="/ws/src/m-explore-ros2/clearpath/jackal/",
            description="Clearpath setup path",
        ),

        DeclareLaunchArgument(
            "use_nav2",
            default_value="true",
            description="Launch Clearpath Nav2",
        ),

        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz",
        ),

        # Match your running setup.
        SetEnvironmentVariable(
            name="ROS_DOMAIN_ID",
            value="0",
        ),

        # Helpful for RViz in Docker / NoVNC / software-rendered environments.
        SetEnvironmentVariable(
            name="LIBGL_ALWAYS_SOFTWARE",
            value="1",
        ),
        SetEnvironmentVariable(
            name="QT_X11_NO_MITSHM",
            value="1",
        ),

        nav2_launch,
        costmap_republisher,
        rviz,
    ])