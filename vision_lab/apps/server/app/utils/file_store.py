from __future__ import annotations

import shutil
from pathlib import Path

from fastapi import UploadFile

from app.config import UPLOAD_ROOT
from app.core.models import UploadedAssetRef
from app.utils.ids import make_id


class FileStore:
    def __init__(self, root: Path | None = None) -> None:
        self.root = root or UPLOAD_ROOT
        self.root.mkdir(parents=True, exist_ok=True)

    async def save_upload(self, upload: UploadFile) -> UploadedAssetRef:
        file_id = make_id("file")
        suffix = Path(upload.filename or "").suffix
        path = self.root / f"{file_id}{suffix}"
        with path.open("wb") as output:
            shutil.copyfileobj(upload.file, output)
        return UploadedAssetRef(
            file_id=file_id,
            filename=upload.filename or path.name,
            media_type=upload.content_type or "application/octet-stream",
            path=str(path),
            size_bytes=path.stat().st_size,
        )

    def resolve(self, file_id: str) -> Path | None:
        for path in self.root.glob(f"{file_id}*"):
            return path
        return None
