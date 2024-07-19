import os
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from lbr_fri_idl.msg import LBRJointPositionCommand, LBRState
from sensor_msgs.msg import JointState, Image
from tutorial_interfaces.msg import Array3, Cloud
import rosbag2_py
from datetime import datetime
import cv2
import cv_bridge

def open_writer(writer, abs_dir_path):
    storage_options = rosbag2_py._storage.StorageOptions(
        uri=abs_dir_path,
        storage_id='sqlite3')
    converter_options = rosbag2_py._storage.ConverterOptions('', '')
    writer.open(storage_options, converter_options)

class SimpleBagRecorder(Node):
    def __init__(self):
        super().__init__('dataset_recorder')
        self.image_writer = rosbag2_py.SequentialWriter()
        self.vec_writer = rosbag2_py.SequentialWriter()
        self.bridge = cv_bridge.CvBridge()

        current_workspace_path=os.getcwd()
        base_dir = os.path.join(current_workspace_path, 'training_data')
        image_dir = os.path.join(base_dir, 'multi_topic_recorder_image')
        vec_dir = os.path.join(base_dir, 'multi_topic_recorder_vector')
        os.makedirs(image_dir, exist_ok=True)
        os.makedirs(vec_dir, exist_ok=True)

        # 创建基于当前时间戳的唯一目录
        unique_dir = datetime.now().strftime('%Y%m%d_%H%M%S')
        image_save_path = os.path.join(image_dir, unique_dir)
        vec_save_path = os.path.join(vec_dir, unique_dir)

        open_writer(self.image_writer, image_save_path)
        open_writer(self.vec_writer, vec_save_path)

        self.topic_types = {
            '/zed/zed_node/left/image_rect_color': 'sensor_msgs/msg/Image',
            '/zed/zed_node/depth/depth_registered': 'sensor_msgs/msg/Image',

            '/lbr/command/joint_position': 'lbr_fri_idl/msg/LBRJointPositionCommand',
            '/lbr/state': 'lbr_fri_idl/msg/LBRState',
            '/lbr/joint_states': 'sensor_msgs/msg/JointState',
            '/force_r': 'tutorial_interfaces/msg/Array3',
            '/position_r': 'tutorial_interfaces/msg/Cloud'
        }

        self._subscriptions = []
        self.msg_latest = {topic: None for topic in self.topic_types.keys()}

        for topic_name, topic_type in self.topic_types.items():
            topic_info = rosbag2_py._storage.TopicMetadata(
                name=topic_name,
                type=topic_type,
                serialization_format='cdr')
            if topic_name in ['/zed/zed_node/left/image_rect_color', '/zed/zed_node/depth/depth_registered']:
                self.image_writer.create_topic(topic_info)
            else:
                self.vec_writer.create_topic(topic_info)

            msg_type = eval(topic_type.split('/')[2])

            self._subscriptions.append(self.create_subscription(
                msg_type,
                topic_name,
                self.msg_buffer_callback(topic_name),
                10
            ))

        self.timer = self.create_timer(0.1, self.write_messages)  # 10Hz 定时器
        # TODO: image 15Hz，在0.1s时获取的可能有延迟，但joint等没有延迟，二者可能不匹配。可能取相机频率为关键帧会更好。

    def msg_buffer_callback(self, topic_name):
        def img_callback(msg):
            # 调整图像大小为640x360像素
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            resized_image = cv2.resize(cv_image, (640, 360))
            msg = self.bridge.cv2_to_imgmsg(resized_image, encoding='passthrough', header=msg.header)
            self.get_logger().info(str(msg.header))
            self.msg_latest[topic_name] = msg

        def vec_callback(msg):
            self.msg_latest[topic_name] = msg

        if topic_name in ['/zed/zed_node/left/image_rect_color', '/zed/zed_node/depth/depth_registered']:
            return img_callback
        else:
            return vec_callback

    def write_messages(self):
        if None not in self.msg_latest.values():
            current_time = self.get_clock().now().nanoseconds
            for topic_name, msg in self.msg_latest.items():
                if topic_name in ['/zed/zed_node/left/image_rect_color', '/zed/zed_node/depth/depth_registered']:
                    self.image_writer.write(
                        topic_name,
                        serialize_message(msg),
                        current_time)
                else:
                    self.vec_writer.write(topic_name, serialize_message(msg), current_time)
        else:
            self.get_logger().warn("有些消息仍然为空!暂时不启动数据集录制。")
            for topic_name, msg in self.msg_latest.items():
            	if msg is None:
            		print(topic_name, "is none")


def main(args=None):
    rclpy.init(args=args)
    sbr = SimpleBagRecorder()
    rclpy.spin(sbr)
    rclpy.shutdown()


if __name__ == '__main__':
    main()