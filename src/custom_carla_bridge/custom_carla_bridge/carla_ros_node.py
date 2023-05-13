import rclpy
import cv2
import carla
import numpy as np
import random
import signal
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.executors import SingleThreadedExecutor
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

        self.cv_bridge = CvBridge()

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

        # Shutdown callback
        signal.signal(signal.SIGINT, self.shutdown_callback)

        # Connect to the Carla simulator
        self.connect_to_carla()

        self.timer_period = 0.05  # desired simulation rate in seconds (e.g., 0.05 s for 20 Hz)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def connect_to_carla(self) -> None:
        # Connect to the Carla simulator
        self.client = carla.Client(self.host, 2000)
        self.client.set_timeout(10.0)

        # Load the town
        self.world = self.client.load_world(self.town)
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        self.world.apply_settings(settings)
        self.world.tick()

        # Get the current weather parameters
        weather = self.world.get_weather()

        # Set the desired parameters
        weather.cloudiness = 0.0
        weather.precipitation = 0.0
        weather.fog_density = 0.0
        weather.sun_altitude_angle = 10.0

        # Apply the new weather parameters
        self.world.set_weather(weather)

        # Load the Tesla model
        blueprint_library = self.world.get_blueprint_library()
        tesla_blueprint = blueprint_library.find('vehicle.tesla.model3')

        # Spawn the Tesla at a random spawn point
        spawn_point = random.choice(self.world.get_map().get_spawn_points())
        self.ego_vehicle = self.world.try_spawn_actor(tesla_blueprint, spawn_point)
        self.world.tick()

        # Attach an RGB camera to the Tesla
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '800')
        camera_bp.set_attribute('image_size_y', '600')
        camera_bp.set_attribute('fov', '110')

        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        self.camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=self.ego_vehicle)

        # Register the image callback
        self.camera.listen(lambda image: self.process_image(image))

    def timer_callback(self):
        self.world.tick()

    def process_image(self, carla_image: carla.Image) -> None:
        # Convert the CARLA image from BGRA to RGB format
        bgra_image = np.array(carla_image.raw_data).reshape((carla_image.height, carla_image.width, 4))
        rgb_image = bgra_image[:, :, :3][:, :, ::-1]

        # Convert the OpenCV image to a ROS Image message
        image_msg = self.cv_bridge.cv2_to_imgmsg(rgb_image, encoding='rgb8')

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

    def shutdown_callback(self) -> None:
        # Cleanup the Carla ROS node
        self.camera.destroy()
        self.ego_vehicle.destroy()

        settings = self.world.get_settings()
        settings.synchronous_mode = False
        self.world.apply_settings(settings)


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


if __name__ == '__main__':
    main()
