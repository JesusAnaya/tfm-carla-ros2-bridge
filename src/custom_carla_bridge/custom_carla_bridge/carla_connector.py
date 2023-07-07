import carla
import random
import math
from typing import Union, Any, Callable


class CarlaConnector(object):
    def __init__(self, host: str, town: str) -> None:
        self.host = host
        self.town = town
        self.world: Union[carla.World, None] = None
        self.ego_vehicle: Union[carla.Vehicle, None] = None
        self.camera_center: Union[carla.Sensor, None] = None
        self.camera_left: Union[carla.Sensor, None] = None
        self.camera_right: Union[carla.Sensor, None] = None
        self.client: Union[carla.Client, None] = None
        self.camera_deviation: float = 0.4

    def connect_to_carla(self) -> None:
        # Connect to the Carla simulator
        self.client = carla.Client(self.host, 2000)
        self.client.set_timeout(200.0)

        # Load the town
        self.init_world()

        # Get the blueprint library
        blueprint_library = self.world.get_blueprint_library()

        # Set the weather
        self.init_weather()

        # init ego vehicle
        self.init_ego_vehicle(blueprint_library)

        self.init_cameras(blueprint_library)

    def init_world(self) -> None:
        # Load the town
        self.world = self.client.load_world(self.town)
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        self.world.apply_settings(settings)
        self.world.tick()

    def init_weather(self) -> None:
        # Get the current weather parameters
        weather = self.world.get_weather()

        # Set the desired parameters
        weather.cloudiness = 70.0
        weather.precipitation = 0.0
        weather.precipitation_deposits = 30.0
        weather.fog_density = 0.0
        weather.sun_altitude_angle = 30.0

        # Apply the new weather parameters
        self.world.set_weather(weather)

    def init_ego_vehicle(self, blueprint_library: Any) -> None:
        # Load the Tesla model
        tesla_blueprint = blueprint_library.find("vehicle.tesla.model3")

        # Spawn the Tesla at a random spawn point
        spawn_point = random.choice(self.world.get_map().get_spawn_points())
        self.ego_vehicle = self.world.try_spawn_actor(tesla_blueprint, spawn_point)
        self.world.tick()

    def init_cameras(self, blueprint_library: Any) -> None:
        # Attach an RGB camera to the Tesla
        camera_bp = blueprint_library.find("sensor.camera.rgb")
        camera_bp.set_attribute("image_size_x", "960")
        camera_bp.set_attribute("image_size_y", "480")
        camera_bp.set_attribute("fov", "90")

        # Get Vehicle Physics Control
        # physics_control = self.ego_vehicle.get_physics_control()
        # Get the maximum steer angle of the front wheels
        # max_steer_angle = physics_control.wheels[0].max_steer_angle

        # Set cameras to the center, left, and right of the vehicle
        self.camera_center = self.set_camera(
            camera_bp, carla.Transform(carla.Location(x=2.5, z=1.2))
        )
        self.camera_right = self.set_camera(
            camera_bp,
            carla.Transform(carla.Location(x=2.5, y=0.5, z=1.2))  # y=-0.5 for right side
        )
        self.camera_left = self.set_camera(
            camera_bp,
            carla.Transform(carla.Location(x=2.5, y=-0.5, z=1.2))  # y=0.5 for left side
        )

    def set_camera(
        self, camera_bp: carla.Sensor, transform: carla.Transform
    ) -> carla.Sensor:
        return self.world.spawn_actor(camera_bp, transform, attach_to=self.ego_vehicle)

    def set_camera_center_callback(self, callback: Callable) -> None:
        # Register the image callback
        self.camera_center.listen(callback)

    def set_camera_left_callback(self, callback: Callable) -> None:
        # Register the image callback
        self.camera_left.listen(callback)

    def set_camera_right_callback(self, callback: Callable) -> None:
        # Register the image callback
        self.camera_right.listen(callback)

    def timer_callback(self) -> None:
        self.world.tick()

    def set_vehicle_control(self, control: carla.VehicleControl) -> None:
        self.ego_vehicle.apply_control(control)

    def get_ego_vehicle_info(self) -> dict:
        velocity_s = self.ego_vehicle.get_velocity()
        velocity = 3.6 * math.sqrt(velocity_s.x ** 2 + velocity_s.y ** 2 + velocity_s.z ** 2)
        return {
            "velocity": velocity,
        }

    def cleanup(self) -> None:
        # Cleanup the Carla ROS node
        if self.camera_center is not None:
            self.camera_center.destroy()

        if self.camera_left is not None:
            self.camera_left.destroy()

        if self.camera_right is not None:
            self.camera_right.destroy()

        if self.ego_vehicle is not None:
            self.ego_vehicle.destroy()

        settings = self.world.get_settings()
        settings.synchronous_mode = False
        self.world.apply_settings(settings)
