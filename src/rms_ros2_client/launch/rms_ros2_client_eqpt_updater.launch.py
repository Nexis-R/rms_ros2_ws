import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # パッケージディレクトリのパスを取得
    rms_ros2_client_dir = get_package_share_directory('rms_ros2_client')

    image_saver_node = Node(
        package='rms_ros2_client',
        executable='image_saver_node',
        name='image_saver_node',
        remappings=[
            ('/image_in', '/recognition_result')
        ],
        parameters=[
            # パッケージ内のディレクトリパスを使用
            {'save_directory': os.path.join(rms_ros2_client_dir, 'snaps')},
            {'file_name': 'result.jpg'}
        ]
    )

    rms_ros_client_eqpt_updater = Node(
        package='rms_ros2_client',
        executable='rms_ros2_client_eqpt_updater',
        name='rms_ros2_client_eqpt_updater',
        remappings=[
            ('/qrcode_info', '/qrcode_info'),
            ('/recognition_value', '/recognition_value'),
            ('/update_result', '/update_result')
        ],
        parameters=[
            # パッケージ内のディレクトリパスを使用
            {'ip': '52.193.111.81'},
            {'robot_id': 16},
            # {'mac_id': ''},
            {'image_path': os.path.join(rms_ros2_client_dir, 'snaps/result.jpg')}
        ]
    )

    return LaunchDescription([
        image_saver_node,
        rms_ros_client_eqpt_updater
    ])