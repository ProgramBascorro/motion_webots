from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from app.adapters.image_input import ImageInputAdapter


def test_image_input_adapter_accepts_mp4(tmp_path: Path) -> None:
    video_path = tmp_path / "sample.mp4"
    writer = cv2.VideoWriter(str(video_path), cv2.VideoWriter_fourcc(*"mp4v"), 5.0, (64, 48))
    assert writer.isOpened()
    for intensity in (40, 120):
        frame = np.full((48, 64, 3), intensity, dtype=np.uint8)
        writer.write(frame)
    writer.release()

    adapter = ImageInputAdapter(str(video_path))
    frames = list(adapter.frames())
    adapter.close()

    assert len(frames) == 2
    assert frames[0].metadata["media_kind"] == "video"
    assert frames[0].width == 64
    assert frames[0].height == 48
