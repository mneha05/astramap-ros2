from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("astra_slam"), "config", "slam.yaml"
    )
    use_viewer = LaunchConfiguration("viewer")
    return LaunchDescription([
        DeclareLaunchArgument(
            "viewer", default_value="true",
            description="Start the OpenGL 3.3 visualization window"
        ),
        Node(
            package="astra_slam",
            executable="sensor_sim_node",
            name="sensor_sim",
            output="screen",
        ),
        Node(
            package="astra_slam",
            executable="fusion_slam_node",
            name="fusion_slam",
            parameters=[config],
            output="screen",
        ),
        Node(
            package="astra_slam",
            executable="opengl_viewer_node",
            name="opengl_viewer",
            condition=IfCondition(use_viewer),
            output="screen",
        ),
    ])
