#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from ament_index_python.packages import get_package_share_directory

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')
        
        # トピック名を指定
        #self.image_topic = '/image_in/compressed'
        self.image_topic = '/image_in'
        
        # 保存先ディレクトリをROSパラメータから取得、デフォルトはrms_ros2_clientパッケージのshareディレクトリ
        try:
            default_save_directory = os.path.join(get_package_share_directory('rms_ros2_client'), 'snaps')
        except Exception as e:
            self.get_logger().error(f"Error getting package path: {str(e)}")
            default_save_directory = './snaps'
        
        self.save_directory = self.declare_parameter('save_directory', default_save_directory).get_parameter_value().string_value
        self.file_name = self.declare_parameter('file_name', 'cam1.jpg').get_parameter_value().string_value
        
        self.get_logger().info(f"Image file path: {self.save_directory}")
        
        # 保存先ディレクトリが存在しない場合は作成
        if not os.path.exists(self.save_directory):
            os.makedirs(self.save_directory)
        
        # カメライメージを受け取るためのサブスクライバを設定
        self.subscription = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        
        # CVBridgeのインスタンスを作成
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # ImageメッセージをOpenCV形式に変換
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            
            # 保存するファイル名を生成
            timestamp = self.get_clock().now().to_msg().sec
            filename = self.file_name.format(timestamp=timestamp)
            
            # 画像を保存
            filepath = os.path.join(self.save_directory, filename)
            cv2.imwrite(filepath, cv_image)
            self.get_logger().info(f"Image saved: {filename}")
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        image_saver_node = ImageSaverNode()
        rclpy.spin(image_saver_node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
