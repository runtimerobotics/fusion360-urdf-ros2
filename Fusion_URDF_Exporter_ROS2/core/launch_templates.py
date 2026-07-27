display_launch = """from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('%s')
    xacro_path = PathJoinSubstitution([pkg_share, 'urdf', '%s.xacro'])
    rviz_cfg = PathJoinSubstitution([pkg_share, 'config', 'display.rviz'])
    prefix = LaunchConfiguration('prefix')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path, ' ', 'prefix:=', prefix]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix applied to links and joints',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen',
        ),
    ])
"""


gazebo_launch = """import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share_dir = get_package_share_directory('%s')
    xacro_file = os.path.join(share_dir, 'urdf', '%s.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    spawn_z = LaunchConfiguration('spawn_z')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}],
        output='screen',
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py',
            ])
        ]),
        launch_arguments={'pause': 'true'}.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py',
            ])
        ])
    )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', '%s',
            '-topic', 'robot_description',
            '-z', spawn_z,
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'spawn_z',
            default_value='0.0',
            description='Initial robot spawn height in Gazebo Classic',
        ),
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
    ])
"""


gazebo_sim_launch = """import os
from os.path import join

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_robot = get_package_share_directory('%s')
    spawn_z = LaunchConfiguration('spawn_z')

    robot_description_file = os.path.join(pkg_robot, 'urdf', '%s.xacro')
    ros_gz_bridge_config = os.path.join(
        pkg_robot,
        'config',
        'ros_gz_bridge_gazebo.yaml',
    )

    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items(),
    )

    spawn_robot = TimerAction(
        period=5.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', '/robot_description',
                '-name', '%s',
                '-allow_renaming', 'false',
                '-z', spawn_z,
                '-x', '0.0',
                '-y', '0.0',
                '-Y', '0.0',
            ],
            output='screen',
        )],
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': ros_gz_bridge_config}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'spawn_z',
            default_value='0.0',
            description='Initial robot spawn height in Gazebo Sim',
        ),
        gazebo,
        spawn_robot,
        ros_gz_bridge,
        robot_state_publisher,
    ])
"""

def get_display_launch_text(package_name, robot_name):
    return display_launch % (package_name, robot_name)


def get_gazebo_launch_text(package_name, robot_name):
    return gazebo_launch % (package_name, robot_name, robot_name)


def get_gazebo_sim_launch_text(package_name, robot_name):
    return gazebo_sim_launch % (package_name, robot_name, robot_name)
