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

    package_name='diff_robot' 

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(package_name),
                    "description",
                    "robot.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    

    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.world'
        )    
    
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': [' -r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
             )

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', 
                        executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'robot',
                                   '-z', '0.1'],
                        output='screen')

    robot_controllers = PathJoinSubstitution(
         [
         FindPackageShare(package_name),
         'config',
         'diff_drive.yaml',
         ]
     )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--param-file',
            robot_controllers,
            ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--param-file',
            robot_controllers,
            ],
    )
    
    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_params.yaml')

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    # Launch them all!
    return LaunchDescription([
        world_arg,
        gazebo,
        ros_gz_bridge,
        robot_state_pub_node,
        spawn_entity,
        diff_drive_spawner,
        joint_state_broadcaster_spawner

    ])