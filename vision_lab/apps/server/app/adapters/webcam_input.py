from __future__ import annotations

import cv2

from app.adapters.base import BaseAdapter
from app.engine.packets import FramePacket
from app.utils.ids import make_id
from app.utils.time import now_ms


class WebcamInputAdapter(BaseAdapter):
    def __init__(self, device_index: int, width: int, height: int) -> None:
        self.capture = cv2.VideoCapture(device_index)
        if not self.capture.isOpened():
            raise ValueError(f"Failed to open webcam device {device_index}")
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def frames(self):
        while True:
            ok, frame = self.capture.read()
            if not ok:
                break
            yield FramePacket(
                frame_id=make_id("frame"),
                timestamp_ms=now_ms(),
                image=frame,
                width=int(frame.shape[1]),
                height=int(frame.shape[0]),
                metadata={},
            )

    def close(self) -> None:
        self.capture.release()
