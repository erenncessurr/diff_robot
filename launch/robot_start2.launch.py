import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'diff_robot'

    # Xacro dosya yolu - isimlendirmeye dikkat (agv_control_bot.xacro mu robot.urdf.xacro mu?)
    xacro_file = os.path.join(get_package_share_directory(package_name), 'description', 'robot.urdf.xacro')
    
    # Hatanın çözümü: Command içinde doğrudan string listesi kullanmak
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {"robot_description": robot_description_content}

    # Controller Manager (Hardware Interface'i yükleyen ana node)
    robot_controllers = os.path.join(get_package_share_directory(package_name), 'config', 'diff_drive.yaml')
    
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Spawnerlar (Controller Manager'dan sonra başlamalıdır)
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--param-file',
            robot_controllers
            ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--param-file',
            robot_controllers
        ],
    )

    return LaunchDescription([
        robot_state_pub_node,
        controller_manager, 
        diff_drive_spawner,
        joint_state_broadcaster_spawner
    ])