import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # For simulation, use_sim_time should be true
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # Get package directories
    pranesh_description_dir = get_package_share_directory('pranesh_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            pranesh_description_dir,
            'map',
            'map3.yaml'))
            
    config_file_name = 'nav2_params.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            pranesh_description_dir,
            'config',
            config_file_name))
            
    nav2_launch_file_dir = os.path.join(nav2_bringup_dir, 'launch')
    
    rviz_config_dir = os.path.join(
        pranesh_description_dir,
        'rviz',
        'urdf.rviz')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),
            
        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),
            
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',  # Changed to true for simulation
            description='Use simulation (Gazebo) clock if true'),
            
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to launch RViz'),
        
        # Map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': map_dir,
                'tf_buffer_duration': 30.0
            }]
        ),
        
        # Lifecycle manager for map server
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server']
            }]
        ),
        
        # Nav2 bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir,
                'autostart': 'true'
            }.items(),
        ),
        
        # For simulation only: Add a fixed delay to buffer transforms
        ExecuteProcess(
            cmd=['sleep', '3.0'],
            name='delay_for_transform_sync'
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{
                'use_sim_time': use_sim_time,
                'tf_buffer_duration': 30.0
            }],
            condition=IfCondition(use_rviz),
            output='screen'),
        
       Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),
        
        # TF buffer monitor
        Node(
            package='tf2_ros',
            executable='tf2_monitor',
            name='tf2_monitor',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Optional: Transform echo for debugging
        Node(
            package='tf2_ros',
            executable='tf2_echo',
            name='tf2_echo',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['map', 'base_link'],
            output='screen'
        )
    ])
