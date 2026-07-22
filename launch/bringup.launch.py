from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition


def generate_launch_description():
    # Package paths
    description_pkg_path = get_package_share_directory("mr3_mini_description")
    driver_pkg_path = get_package_share_directory("mr3_mini_driver")

    # Robot URDF
    urdf_file = os.path.join(description_pkg_path, "urdf", "mr3_mini.urdf")

    with open(urdf_file, "r", encoding="utf-8") as f:
        robot_desc = f.read()

    # Default Rviz view configuration
    rviz_def_config = os.path.join(description_pkg_path, "rviz", "mr3_mini_view.rviz")

    # Plot juggler configuration
    plotjuggler_config = os.path.join(driver_pkg_path, "config", "plotjuggler_layout.xml")

    # Launch configurations
    show_plotter = LaunchConfiguration("show_plotter")

    return LaunchDescription(
        [
            # Controls launching plotter
            DeclareLaunchArgument("show_plotter", default_value="false"),
            
            # MR-3 Mini Driver
            Node(
                package="mr3_mini_driver",
                executable="mr3_mini_driver",
                name="mr3_mini_driver",
                output="screen",
            ),
            
            # Publishes the robot state (required for Rviz)
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_desc}],
            ),
            
            # Plotjuggler with pre-configured view
            Node(
                package="plotjuggler",
                executable="plotjuggler",
                name="plotjuggler",
                output="screen",
                arguments=["-l", plotjuggler_config],
                condition=IfCondition(show_plotter),
            ),
            
            # Rviz with pre-configured view
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_def_config],
            ),
        ]
    )
