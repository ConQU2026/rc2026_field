from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os.path


def generate_launch_description():
    gazebo_ros_pkg = FindPackageShare('gazebo_ros')
    rc2026_field_pkg = FindPackageShare('rc2026_field')
    gz_launch_path = PathJoinSubstitution([gazebo_ros_pkg, 'launch', 'gazebo.launch.py'])
    
    controller_config = PathJoinSubstitution([rc2026_field_pkg, 'config', 'controller.yaml'])

    world_path = PathJoinSubstitution([rc2026_field_pkg, 'resource', 'homework.world'])
    rviz_config_path = PathJoinSubstitution([rc2026_field_pkg, 'rviz', 'field.rviz'])
    xacro_file = PathJoinSubstitution([rc2026_field_pkg, 'urdf', 'R2.xacro'])
    robot_description = Command(['xacro ', xacro_file])


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
     
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'R2',
            '-topic', 'robot_description',
            '-x', '1.0',
            '-y', '1.0',
            '-z', '0.0',
        ],
        output='screen'
    )

    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config_path],
    #     parameters=[{'use_sim_time': True}]
    # )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['-0.32015', '0.30715', '-0.151', '0', '0', '0', 'body', 'base_link'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Values for Target
    target_xacro_path = PathJoinSubstitution([rc2026_field_pkg, 'urdf', 'target.xacro'])
    
    # Process Xacro
    target_description = Command(['xacro ', target_xacro_path])

    # Target State Publisher (to handle robot_description parameter)
    target_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='target_state_publisher',
        namespace='target',
        output='screen',
        parameters=[{
            'robot_description': target_description,
            'use_sim_time': True
        }]
    )

    target_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'target',
            '-topic', '/target/robot_description',
            '-x', '-2.0',
            '-y', '-2.0',
            '-z', '0.5',
            '-robot_namespace', 'target'
        ],
        output='screen'
    )

    target_controller_node = Node(
        package='rc2026_field',
        executable='target_controller',
        name='target_controller',
        namespace='target',
        output='screen',
        parameters=[
            controller_config,
            {'use_sim_time': True}
        ]
    )


    ld = LaunchDescription()

    ld.add_action(AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=PathJoinSubstitution([rc2026_field_pkg, 'resource'])
    ))


    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'world': world_path,
            'extra_gazebo_args': '--verbose',
        }.items(),
    ))


    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(urdf_spawn_node)
    # ld.add_action(rviz_node)
    ld.add_action(static_tf)
    ld.add_action(target_state_publisher_node)
    ld.add_action(target_spawn_node)
    ld.add_action(target_controller_node)

    return ld