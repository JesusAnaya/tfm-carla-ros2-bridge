from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from carla_ros_interfaces.msg import VehicleControl, VehicleInfo
from .carla_connector import CarlaConnector
from cv_bridge import CvBridge
from typing import Any
import rclpy
import carla
import signal
import numpy as np
import cv2


# Create a QoS profile with "best effort" reliability
qos_profile = QoSProfile(
    depth=24,  # Adjust this as necessary
    reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
)
cv_bridge = CvBridge()


def process_image(carla_image: carla.Image, publisher: Any) -> None:
    # Convert the CARLA image from BGRA to RGB format
    image_bgra = np.array(carla_image.raw_data).reshape(
        (carla_image.height, carla_image.width, 4)
    )
    image_rgb = image_bgra[:, :, :3][:, :, ::-1]

    # resize image with opencv
    image_rgb = cv2.resize(image_rgb, (320, 160), interpolation=cv2.INTER_CUBIC)

    # Convert the OpenCV image to a ROS Image message
    image_msg = cv_bridge.cv2_to_imgmsg(image_rgb, encoding="rgb8")

    publisher.publish(image_msg)


class CarlaROSNode(Node):
    def __init__(self):
        super().__init__("carla_module_node")
        # Parameters
        self.camera_right_publisher: Any = None
        self.camera_left_publisher: Any = None
        self.camera_center_publisher: Any = None
        self.vehicle_control_sub: Any = None
        self.ego_vehicle_info_publisher: Any = None

        self.declare_parameter("host", "localhost")
        self.declare_parameter("town", "Town01_Opt")
        host: str = self.get_parameter("host").value
        town: str = self.get_parameter("town").value

        self.carla_connector = CarlaConnector(host, town)

        # Publishers
        self.set_publishers()

        # Subscribers
        self.set_subscribers()

        # Shutdown callback
        signal.signal(signal.SIGINT, self.shutdown_callback)

        # Connect to the Carla simulator
        self.get_logger().info("Connecting to Carla...")
        self.carla_connector.connect_to_carla()

        # Set camera callbacks
        self.get_logger().info("Setting camera callbacks...")
        self.carla_connector.set_camera_center_callback(self.camera_center_callback)
        self.carla_connector.set_camera_left_callback(self.camera_left_callback)
        self.carla_connector.set_camera_right_callback(self.camera_right_callback)

        # desired simulation rate in seconds (e.g., 0.05 s for 20 Hz)
        self.timer_period: float = 0.05
        self.timer: Any = self.create_timer(
            self.timer_period, self.timer_callback
        )
        self.get_logger().info("Node ROS Bridge to Carla started...")

    def set_publishers(self):
        self.camera_center_publisher = self.create_publisher(
            Image, "carla_bridge/ego/camera_center", qos_profile
        )

        self.camera_left_publisher = self.create_publisher(
            Image, "carla_bridge/ego/camera_left", qos_profile
        )

        self.camera_right_publisher = self.create_publisher(
            Image, "carla_bridge/ego/camera_right", qos_profile
        )

        self.ego_vehicle_info_publisher = self.create_publisher(
            VehicleInfo, "carla_bridge/ego/vehicle_info", 10
        )

    def set_subscribers(self):
        self.vehicle_control_sub = self.create_subscription(
            VehicleControl, "/carla_bridge/ego/control", self.vehicle_control_callback, 10
        )

    def timer_callback(self) -> None:
        self.carla_connector.timer_callback()
        info = self.carla_connector.get_ego_vehicle_info()

        # Publish vehicle info
        vehicle_info: VehicleInfo = VehicleInfo()
        vehicle_info.velocity = float(info["velocity"])
        self.ego_vehicle_info_publisher.publish(vehicle_info)

    def camera_center_callback(self, image: carla.Image) -> None:
        process_image(image, self.camera_center_publisher)

    def camera_left_callback(self, image: carla.Image) -> None:
        process_image(image, self.camera_left_publisher)

    def camera_right_callback(self, image: carla.Image) -> None:
        process_image(image, self.camera_right_publisher)

    def vehicle_control_callback(self, msg: VehicleControl) -> None:
        # Apply the control commands to the ego vehicle
        # self.get_logger().info(f'Applying control: {msg}')

        control = carla.VehicleControl(
            throttle=msg.throttle, steer=msg.steer, brake=msg.brake, reverse=msg.reverse
        )
        self.carla_connector.set_vehicle_control(control)

    def shutdown_callback(self) -> None:
        self.get_logger().info("Cleaning up Carla connector...")
        self.carla_connector.cleanup()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CarlaROSNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
