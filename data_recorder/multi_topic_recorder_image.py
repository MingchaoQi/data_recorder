import os
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from lbr_fri_idl.msg import LBRJointPositionCommand, LBRState
from sensor_msgs.msg import JointState, Image
import rosbag2_py
from datetime import datetime
import cv2
import cv_bridge


class SimpleBagRecorder(Node):
    def __init__(self):
        super().__init__('multi_topic_recorder_image')
        self.writer = rosbag2_py.SequentialWriter()
        self.bridge = cv_bridge.CvBridge()

        base_dir = '/root/ros2_ws/ros2_driver_layer/training_data/multi_topic_recorder_image'

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
            '/zed/zed_node/left/image_rect_color': 'sensor_msgs/msg/Image',
            '/zed/zed_node/depth/depth_registered': 'sensor_msgs/msg/Image',
        }

        self._subscriptions = []
        self.msg_buffers = {topic: None for topic in self.topic_types.keys()}
        self.msg_sizes = {topic: (0, 0) for topic in self.topic_types.keys()}

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
                self.msg_buffer_callback(topic_name),
                10
            ))

        self.timer = self.create_timer(0.1, self.write_messages)  # 10Hz 定时器

    def msg_buffer_callback(self, topic_name):
        def callback(msg):
            if topic_name in self.msg_buffers:
                # 调整图像大小为640x360像素
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                resized_image = cv2.resize(cv_image, (640, 360))
                msg = self.bridge.cv2_to_imgmsg(resized_image, encoding='passthrough')

                # 更新消息缓冲区和图像大小信息
                self.msg_buffers[topic_name] = msg
                self.msg_sizes[topic_name] = (resized_image.shape[1], resized_image.shape[0])

        return callback

    def write_messages(self):
        current_time = self.get_clock().now().nanoseconds
        for topic_name, msg in self.msg_buffers.items():
            if msg is not None:
                self.writer.write(
                    topic_name,
                    serialize_message(msg),
                    current_time)
                self.get_logger().info(f'Topic: {topic_name}, Size: {self.msg_sizes[topic_name]} bytes')


def main(args=None):
    rclpy.init(args=args)
    sbr = SimpleBagRecorder()
    rclpy.spin(sbr)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
