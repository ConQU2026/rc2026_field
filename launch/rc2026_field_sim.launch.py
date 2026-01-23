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
    
    world_path = PathJoinSubstitution([rc2026_field_pkg, 'worlds', 'robocon2026_random.world'])
    rviz_config_path = PathJoinSubstitution([rc2026_field_pkg, 'rviz', 'field.rviz'])

    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config_path],
    #     parameters=[{'use_sim_time': True}]
    #)
    ld = LaunchDescription()

    ld.add_action(AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=PathJoinSubstitution([rc2026_field_pkg, 'models'])
    ))


    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'world': world_path,
            'extra_gazebo_args': '--verbose',
        }.items(),
    ))
    
    return ld