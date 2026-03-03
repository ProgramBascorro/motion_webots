from __future__ import annotations

from pathlib import Path

import cv2

from app.adapters.base import BaseAdapter
from app.engine.packets import FramePacket
from app.utils.ids import make_id
from app.utils.time import now_ms


class FolderReplayInputAdapter(BaseAdapter):
    def __init__(self, paths: list[str]) -> None:
        self.paths = [Path(path) for path in paths]

    def frames(self):
        for path in sorted(self.paths, key=lambda item: item.name):
            image = cv2.imread(str(path))
            if image is None:
                continue
            yield FramePacket(
                frame_id=make_id("frame"),
                timestamp_ms=now_ms(),
                image=image,
                width=int(image.shape[1]),
                height=int(image.shape[0]),
                metadata={"source": str(path)},
            )
