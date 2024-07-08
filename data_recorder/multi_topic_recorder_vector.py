import os
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from lbr_fri_idl.msg import LBRJointPositionCommand, LBRState
from sensor_msgs.msg import JointState
from tutorial_interfaces.msg import Array3, Cloud
import rosbag2_py
from datetime import datetime


class SimpleBagRecorder(Node):
    def __init__(self):
        super().__init__('multi_topic_recorder_image')
        self.writer = rosbag2_py.SequentialWriter()

        base_dir = '/root/ros2_ws/ros2_driver_layer/training_data/multi_topic_recorder_vector'

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

        self.topic_types = {
            '/lbr/command/joint_position': 'lbr_fri_idl/msg/LBRJointPositionCommand',
            '/lbr/state': 'lbr_fri_idl/msg/LBRState',
            '/lbr/joint_states': 'sensor_msgs/msg/JointState',
            '/force_r': 'tutorial_interfaces/msg/Array3',
            '/position_r': 'tutorial_interfaces/msg/Cloud'
        }

        self._subscriptions = []

        for topic_name, topic_type in self.topic_types.items():
            topic_info = rosbag2_py._storage.TopicMetadata(
                name=topic_name,
                type=topic_type,
                serialization_format='cdr')
            self.writer.create_topic(topic_info)

            msg_type = eval(topic_type.split('/')[2])

            self._subscriptions.append(self.create_subscription(
                msg_type,
                topic_name,
                self.topic_callback(topic_name),
                10
            ))

    def topic_callback(self, topic_name):
        def callback(msg):
            self.writer.write(
                topic_name,
                serialize_message(msg),
                self.get_clock().now().nanoseconds)

        return callback


def main(args=None):
    rclpy.init(args=args)
    sbr = SimpleBagRecorder()
    rclpy.spin(sbr)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
