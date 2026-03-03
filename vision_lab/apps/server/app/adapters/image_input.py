from __future__ import annotations

from pathlib import Path

import cv2

from app.adapters.base import BaseAdapter
from app.engine.packets import FramePacket
from app.utils.ids import make_id
from app.utils.time import now_ms


class ImageInputAdapter(BaseAdapter):
    VIDEO_SUFFIXES = {".mp4", ".avi", ".mov", ".mkv", ".webm", ".m4v"}

    def __init__(self, path: str) -> None:
        self.path = path
        self.capture: cv2.VideoCapture | None = None

    def frames(self):
        suffix = Path(self.path).suffix.lower()
        if suffix in self.VIDEO_SUFFIXES:
            yield from self._video_frames()
            return

        image = cv2.imread(self.path)
        if image is not None:
            yield FramePacket(
                frame_id=make_id("frame"),
                timestamp_ms=now_ms(),
                image=image,
                width=int(image.shape[1]),
                height=int(image.shape[0]),
                metadata={"source": self.path, "media_kind": "image"},
            )
            return

        # Fall back to OpenCV video capture so existing image-detection graphs can also consume mp4 uploads.
        yield from self._video_frames()

    def _video_frames(self):
        self.capture = cv2.VideoCapture(self.path)
        if not self.capture.isOpened():
            raise ValueError(f"Failed to read image or video file: {self.path}")

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
                metadata={"source": self.path, "media_kind": "video"},
            )

    def close(self) -> None:
        if self.capture is not None:
            self.capture.release()
            self.capture = None
