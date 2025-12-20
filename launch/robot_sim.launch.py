import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    
    package_name = "diff_robot"

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(package_name),
                    "description",
                    "robot.urdf.xacro"
                ]
            )
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    return LaunchDescription([
        robot_description
    ])