import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_rc2026_field = get_package_share_directory('rc2026_field')
    # 声明配置路径参数
    kfs_config_path = os.path.join(pkg_rc2026_field, 'config', 'kfs_config.yaml')

    world_path = os.path.join(pkg_rc2026_field, 'worlds', 'robocon2026.world')
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

    kfs_manager = Node(
        package='rc2026_field',
        executable='kfs_manager',
        name='kfs_manager',
        output='screen',
        parameters=[{'use_sim_time': True, 'config_path': kfs_config_path}]
    )

    field_gui = Node(
        package='rc2026_field',
        executable='field_gui',
        name='field_gui',
        output='screen',
        parameters=[{'use_sim_time': True, 'config_path': kfs_config_path}]
    )

    return LaunchDescription([
        ExtractConfigPath,
        set_model_path,
        gazebo,
        kfs_manager,
        field_gui
    ])
