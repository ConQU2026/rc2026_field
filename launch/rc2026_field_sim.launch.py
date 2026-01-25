import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_rc2026_field = get_package_share_directory('rc2026_field')
    config_path = os.path.join(pkg_rc2026_field, 'config', 'kfs_config.yaml')
    
    world_path = os.path.join(pkg_rc2026_field, 'worlds', 'robocon2026_with_kfs.world')
    set_model_path = AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_rc2026_field, 'models')
    )
    
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'world': world_path,
        }.items(),
    )


    return LaunchDescription([
        set_model_path,
        gazebo
    ])
