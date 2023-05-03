import os
import csv
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from tfm_carla_data_collector.srv import SetDataCollection


class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')
        self.bridge = CvBridge()
        self.collect_data = False
        self.image_sub = self.create_subscription(
            Image,
            '/carla/ego_vehicle/camera/rgb/front/image',
            self.image_callback,
            10)
        self.steering_sub = self.create_subscription(
            Float64,
            '/carla/ego_vehicle/vehicle_control_cmd/steer',
            self.steering_callback,
            10)
        self.control_sub = self.create_subscription(
            Bool,
            '/data_collector/control',
            self.control_callback,
            10)

        self.srv = self.create_service(SetDataCollection, 'set_data_collection', self.set_data_collection_callback)
        self.image_counter = 0
        self.steering_data = []
        self.output_dir = 'output'

        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
        self.get_logger().info('DataCollector initialized.')

    def set_data_collection_callback(self, request, response):
        self.collect_data = request.collect_data
        response.success = True
        response.message = f"Data collection {'enabled' if self.collect_data else 'disabled'}"
        self.get_logger().info(response.message)
        return response

    def image_callback(self, msg):
        if not self.collect_data:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            image_filename = f"{self.output_dir}/image_{self.image_counter:05d}.png"
            cv2.imwrite(image_filename, cv_image)
            self.get_logger().info(f'Saved image: {image_filename}')
            self.image_counter += 1
        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")

    def steering_callback(self, msg):
        if not self.collect_data:
            return

        steering_value = msg.data
        self.steering_data.append(steering_value)
        self.get_logger().info(f'Received steering value: {steering_value}')

    def control_callback(self, msg):
        self.collect_data = msg.data
        self.get_logger().info(f"Data collection {'enabled' if self.collect_data else 'disabled'}")

    def save_steering_data(self):
        csv_filename = f"{self.output_dir}/steering_data.csv"
        with open(csv_filename, mode='w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            for steering_value in self.steering_data:
                csv_writer.writerow([steering_value])
        self.get_logger().info(f'Saved steering data: {csv_filename}')


def main(args=None):
    rclpy.init(args=args)
    data_collector = DataCollector()
    try:
        rclpy.spin(data_collector)
    except KeyboardInterrupt:
        data_collector.save_steering_data()
        data_collector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
