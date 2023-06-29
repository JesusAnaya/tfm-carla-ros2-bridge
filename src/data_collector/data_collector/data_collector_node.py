import rclpy
import rosbag2_py
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image
from custom_interfaces.msg import VehicleControl
from std_msgs.msg import Bool
from datetime import datetime

# Create a custom QoS profile
qos = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class ImageSaverService(Node):
    def __init__(self):
        super().__init__("image_saver_service")
        self.recording = False
        self.writer = None

        self.subscription = self.create_subscription(
            Bool, "/ros2/service/recording", self.recording_callback, qos_profile=qos
        )

        self.subscription_center = self.create_subscription(
            Image,
            "/carla/vehicle/camera_center",
            self.image_callback_center,
            qos_profile=qos,
        )

        self.subscription_left = self.create_subscription(
            Image,
            "/carla/vehicle/camera_left",
            self.image_callback_left,
            qos_profile=qos,
        )

        self.subscription_right = self.create_subscription(
            Image,
            "/carla/vehicle/camera_right",
            self.image_callback_right,
            qos_profile=qos,
        )

        self.subscription_vehicle_control = self.create_subscription(
            VehicleControl,
            "/carla/vehicle/control",
            self.vehicle_control_callback,
            qos_profile=qos,
        )

    def recording_callback(self, msg):
        if msg.data == self.recording:
            return

        self.recording = msg.data
        if self.recording:
            if self.writer is None:
                now = datetime.now()
                filename = f"/data/data_{now.strftime('%Y%m%d_%H%M%S')}.mcap"
                self.writer = rosbag2_py.SequentialWriter()
                self.writer.open(
                    rosbag2_py.StorageOptions(uri=filename, storage_id="mcap"),
                    rosbag2_py.ConverterOptions(
                        input_serialization_format="cdr",
                        output_serialization_format="cdr",
                    ),
                )
                for topic in [
                    "/carla/vehicle/camera_center",
                    "/carla/vehicle/camera_left",
                    "/carla/vehicle/camera_right",
                    "/carla/vehicle/control",
                ]:
                    type = (
                        "sensor_msgs/msg/Image"
                        if "camera" in topic
                        else "custom_interfaces/msg/VehicleControl"
                    )
                    self.writer.create_topic(
                        rosbag2_py.TopicMetadata(
                            name=topic, type=type, serialization_format="cdr"
                        )
                    )
        else:
            if self.writer is not None:
                del self.writer
                self.writer = None

    def image_callback_center(self, msg):
        if self.recording and self.writer is not None:
            self.writer.write(
                "/carla/vehicle/camera_center",
                serialize_message(msg),
                msg.header.stamp.nanosec,
            )

    def image_callback_left(self, msg):
        if self.recording and self.writer is not None:
            self.writer.write(
                "/carla/vehicle/camera_left",
                serialize_message(msg),
                msg.header.stamp.nanosec,
            )

    def image_callback_right(self, msg):
        if self.recording and self.writer is not None:
            self.writer.write(
                "/carla/vehicle/camera_right",
                serialize_message(msg),
                msg.header.stamp.nanosec,
            )

    def vehicle_control_callback(self, msg):
        if self.recording and self.writer is not None:
            self.writer.write(
                "/carla/vehicle/control",
                serialize_message(msg),
                self.get_clock().now().nanoseconds,
            )


def main(args=None):
    rclpy.init(args=args)
    image_saver_service = ImageSaverService()
    rclpy.spin(image_saver_service)
    image_saver_service.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
