import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from custom_interfaces.msg import VehicleControl
from cv_bridge import CvBridge, CvBridgeError
from .model import NvidiaModel
from .filters import AverageFilter
import torch
import torchvision.transforms as transforms
import cv2

# Create a custom QoS profile
qos = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)


def get_device():
    if torch.cuda.is_available():
        if torch.cuda.device_count() > 1:
            return "cuda:1"
        else:
            return "cuda:0"
    else:
        return "cpu"


def crop_down(image, top=230, bottom=25):
    return image[top:-bottom or None, :]


def clip(steering_angle):
    return max(min(steering_angle, 1.0), -1.0)


class VehicleInferenceNode(Node):
    def __init__(self):
        super().__init__("vehicle_inference_node_node")

        # Action inference
        self.inference_on = False

        self.device = get_device()

        self.filter = AverageFilter(max_size=5)

        # Load your pretrained PyTorch model here
        self.model = NvidiaModel()
        self.model.load_state_dict(torch.load("/models/model.pt", map_location=torch.device(self.device)))
        self.model.to(self.device)  # Move the model to the device
        self.model.eval()  # Set the model to evaluation mode

        # Prepare data transformations (adjust as needed)
        self.transforms = transforms.Compose(
            [
                transforms.ToTensor(),
                transforms.Resize((66, 200)),
                transforms.Normalize(
                    mean=[0.485, 0.456, 0.406],
                    std=[0.229, 0.224, 0.225]
                ),
            ]
        )

        # Placeholder for the current vehicle control state
        self.current_vehicle_control = VehicleControl()  # Placeholder for the current vehicle control state

        self.control_subscription = self.create_subscription(
            VehicleControl, '/carla/vehicle/control', self.control_callback, 10
        )

        # CV Bridge to convert ROS Image to OpenCV image
        self.bridge = CvBridge()

        self.control_publisher = self.create_publisher(
            VehicleControl, "/carla/vehicle/control", 10
        )

        self.camera_center_subscription = self.create_subscription(
            Image, "/carla/vehicle/camera_center", self.image_callback, qos_profile=qos
        )

        self.turn_on_inference_subscription = self.create_subscription(
            Bool, "/ros2/services/turn_on_inference", self.turn_on_inference_callback, 10
        )

    def control_callback(self, msg):
        # Store the current state of the vehicle control
        self.current_vehicle_control = msg

    def turn_on_inference_callback(self, msg):
        if msg.data == self.inference_on:
            return

        self.inference_on = msg.data

    def image_callback(self, msg):
        if not self.inference_on:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().warn(f"Failed to convert image: {str(e)}")
            return

        # Crop the image
        cropped_image = crop_down(cv_image)

        # Convert BGR to RGB
        rgb_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2RGB)

        # Perform inference
        frame = self.transforms(rgb_image).to(self.device)
        batch_t = torch.unsqueeze(frame, 0)

        with torch.no_grad():
            output = self.model(batch_t)

        # Assume output is a single value tensor representing the steering angle
        steer_angle = output.item()

        # Add the steering angle to the filter
        self.filter.add(clip(steer_angle))

        # Use the current state of the vehicle control and update only the steer value
        self.current_vehicle_control.steer = self.filter.get_average()

        #self.get_logger().info(f'Applying control: {self.current_vehicle_control.steer}')

        # Publish the control command
        self.control_publisher.publish(self.current_vehicle_control)


def main(args=None):
    rclpy.init(args=args)

    vehicle_inference_node = VehicleInferenceNode()

    rclpy.spin(vehicle_inference_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vehicle_inference_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
