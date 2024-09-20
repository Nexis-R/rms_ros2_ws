from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the package's share directory
    rms_ros2_client_demo_dir = get_package_share_directory('rms_ros2_client_demo')
    rms_ros2_client_dir = get_package_share_directory('rms_ros2_client')

    return LaunchDescription([
        # Declare use_sim_time argument
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'
        ),

        # Launch RQt with a perspective file
        ExecuteProcess(
            cmd=[
                'rqt', '--perspective-file', 
                os.path.join(rms_ros2_client_demo_dir, 'config', 'rms_gauge.perspective')
            ],
            output='screen'
        ),
        
        # Include another launch file using the dynamically located package path
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                rms_ros2_client_dir, 'launch', 'rms_ros2_client_eqpt_updater.launch.py'
            ))
        ),
    ])
