from __future__ import annotations

from collections.abc import Iterator

from app.engine.packets import FramePacket


class BaseAdapter:
    def frames(self) -> Iterator[FramePacket]:
        raise NotImplementedError

    def close(self) -> None:
        return None
