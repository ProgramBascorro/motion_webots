from __future__ import annotations

import cv2

from app.adapters.base import BaseAdapter
from app.engine.packets import FramePacket
from app.utils.ids import make_id
from app.utils.time import now_ms


class VideoInputAdapter(BaseAdapter):
    def __init__(self, path: str, loop: bool = False) -> None:
        self.path = path
        self.loop = loop
        self.capture = cv2.VideoCapture(path)
        if not self.capture.isOpened():
            raise ValueError(f"Failed to open video file: {path}")

    def frames(self):
        while True:
            ok, frame = self.capture.read()
            if not ok:
                if self.loop:
                    self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                break
            yield FramePacket(
                frame_id=make_id("frame"),
                timestamp_ms=now_ms(),
                image=frame,
                width=int(frame.shape[1]),
                height=int(frame.shape[0]),
                metadata={"source": self.path},
            )

    def close(self) -> None:
        self.capture.release()
