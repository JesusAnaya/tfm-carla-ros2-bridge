import rclpy
import cv2
import carla
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from custom_interfaces.msg import VehicleControl
from cv_bridge import CvBridge


class CarlaROSNode(Node):
    def __init__(self):
        super().__init__('carla_module_node')
        self.world = None
        self.ego_vehicle = None
        self.camera = None
        self.client = None

        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter('host', 'localhost')
        self.host = self.get_parameter('host').value

        self.declare_parameter('town', 'Town01_Opt')
        self.town = self.get_parameter('town').value

        # Create a QoS profile with "best effort" reliability
        qos_profile = QoSProfile(
            depth=24,  # Adjust this as necessary
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
        )

        # Publishers
        self.image_pub = self.create_publisher(Image, 'carla/vehicle/camera', qos_profile)

        # Subscribers
        self.vehicle_control_sub = self.create_subscription(
            VehicleControl,
            '/carla/vehicle/control',
            self.vehicle_control_callback,
            10
        )

        # Connect to the Carla simulator
        self.connect_to_carla()

    def connect_to_carla(self) -> None:
        # Connect to the Carla simulator
        self.client = carla.Client(self.host, 2000)
        self.client.set_timeout(10.0)

        # Load the town
        self.world = self.client.load_world(self.town)

        # Spawn the Tesla ego vehicle
        blueprint_library = self.world.get_blueprint_library()
        tesla_blueprint = blueprint_library.find('vehicle.tesla.model3')
        spawn_points = self.world.get_map().get_spawn_points()
        self.ego_vehicle = self.world.spawn_actor(tesla_blueprint, spawn_points[0])

        # Attach an RGB camera to the Tesla
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '800')
        camera_bp.set_attribute('image_size_y', '600')
        camera_bp.set_attribute('fov', '110')

        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        self.camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=self.ego_vehicle)

        # Register the image callback
        self.camera.listen(lambda image: self.process_image(image))

    def process_image(self, carla_image: carla.Image) -> None:
        # Convert the CARLA image from BGRA to RGB format
        bgra_image = np.array(carla_image.raw_data).reshape((carla_image.height, carla_image.width, 4))
        rgb_image = bgra_image[:, :, :3][:, :, ::-1]

        # Convert the OpenCV image to a ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding='rgb8')

        self.image_pub.publish(image_msg)

    def vehicle_control_callback(self, msg: VehicleControl) -> None:
        # Apply the control commands to the ego vehicle
        # self.get_logger().info(f'Applying control: {msg}')

        control = carla.VehicleControl(
            throttle=msg.throttle,
            steer=msg.steer,
            brake=msg.brake,
            reverse=msg.reverse
        )
        self.ego_vehicle.apply_control(control)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CarlaROSNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
