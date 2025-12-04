from pathlib import Path
from typing import Optional, List
from ament_index_python.packages import get_package_share_directory
from pydantic import BaseModel, field_validator, model_validator

USB_CAM_DIR = get_package_share_directory('usb_cam')


class CameraConfig(BaseModel):
    name: str = 'camera'  # Changed from 'camera1' to 'camera'
    param_path: Path = Path(USB_CAM_DIR, 'config', 'params_1.yaml')
    remappings: Optional[List] = None
    namespace: Optional[str] = None

    @field_validator('param_path')
    @classmethod
    def validate_param_path(cls, value):
        if value and not value.exists():
            raise FileNotFoundError(f'Could not find parameter file: {value}')
        return value

    @model_validator(mode='after')
    def validate_root(self):
        name = self.name
        remappings = self.remappings
        if name and not remappings:
            # Set remappings to match /camera/image_raw format
            remappings = [
                ('image_raw', f'{name}/image_raw'),
                ('image_raw/compressed', f'{name}/image_raw/compressed'),
                ('image_raw/compressedDepth', f'{name}/image_raw/compressedDepth'),
                ('image_raw/theora', f'{name}/image_raw/theora'),
                ('camera_info', f'{name}/camera_info'),
            ]
        self.remappings = remappings
        return self