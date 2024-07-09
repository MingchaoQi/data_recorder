import os
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from std_msgs.msg import String
from lbr_fri_idl.msg import LBRJointPositionCommand
from datetime import datetime

import rosbag2_py

class SimpleBagRecorder(Node):
    def __init__(self):
        super().__init__('topic_recorder')
        self.writer = rosbag2_py.SequentialWriter()

        base_dir = '/root/ros2_ws/ros2_driver_layer/training_data/human_teaching'

        # 确保路径存在，不存在则创建
        os.makedirs(base_dir, exist_ok=True)

        # 创建基于当前时间戳的唯一目录
        unique_dir = datetime.now().strftime('%Y%m%d_%H%M%S')
        save_path = os.path.join(base_dir, unique_dir)

        storage_options = rosbag2_py._storage.StorageOptions(
            uri=save_path,
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        topic_info = rosbag2_py._storage.TopicMetadata(
            name='topic_recorder',
            type='lbr_fri_idl/msg/LBRJointPositionCommand',
            serialization_format='cdr')
        self.writer.create_topic(topic_info)

        self.subscription = self.create_subscription(
            LBRJointPositionCommand,
            '/lbr/command/joint_position',
            self.topic_callback,
            10)
        self.subscription

    def topic_callback(self, msg):
        self.writer.write(
            'topic_recorder',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)


def main(args=None):
    rclpy.init(args=args)
    sbr = SimpleBagRecorder()
    rclpy.spin(sbr)
    rclpy.shutdown()


if __name__ == '__main__':
    main()