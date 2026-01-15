import math
from typing import Optional

from sensor_msgs.msg import CameraInfo


class CameraBase:
    def __init__(self, camera_info: Optional[CameraInfo] = None):
        self.camera_info = camera_info or CameraInfo()
        self.horizontal_fov = 1.2290609
        self.focal_length = 3.67
        self._default_width = 640
        self._default_height = 480

    @property
    def resolution_x(self) -> int:
        width = int(self.camera_info.width) if self.camera_info else 0
        return width if width > 0 else self._default_width

    @property
    def resolution_y(self) -> int:
        height = int(self.camera_info.height) if self.camera_info else 0
        return height if height > 0 else self._default_height

    @property
    def vertical_fov(self) -> float:
        return 2.0 * math.atan(math.tan(self.horizontal_fov * 0.5) * (self.resolution_y / self.resolution_x))

    @property
    def image_sensor_height(self) -> float:
        return math.tan(self.vertical_fov / 2.0) * 2.0 * self.focal_length

    @property
    def image_sensor_width(self) -> float:
        return math.tan(self.horizontal_fov / 2.0) * 2.0 * self.focal_length

    @property
    def pixel_height(self) -> float:
        return self.image_sensor_height / self.resolution_y

    @property
    def pixel_width(self) -> float:
        return self.image_sensor_width / self.resolution_x

    def image_to_world_frame(self, pixel_x: int, pixel_y: int) -> tuple:
        return (
            (self.resolution_x / 2.0 - (pixel_x + 0.5)) * self.pixel_width,
            (self.resolution_y / 2.0 - (pixel_y + 0.5)) * self.pixel_height,
        )

    def world_to_image_frame(self, pos_x: float, pos_y: float) -> tuple:
        return (
            (self.resolution_x / 2.0 + pos_x / self.pixel_width) - 0.5,
            (self.resolution_y / 2.0 + pos_y / self.pixel_height) - 0.5,
        )
