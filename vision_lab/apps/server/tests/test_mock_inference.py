import numpy as np

from app.inference.mock_models import mock_detections, mock_mask


def test_mock_detection_returns_box() -> None:
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    detections = mock_detections(image, ["ball"])
    assert detections
    assert detections[0]["label"] == "ball"


def test_mock_mask_matches_image_size() -> None:
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    mask = mock_mask(image)
    assert mask.shape == (240, 320)
