from __future__ import annotations

from fastapi import APIRouter, File, Request, UploadFile

from app.core.models import UploadResponse

router = APIRouter(prefix="/api", tags=["uploads"])


@router.post("/upload", response_model=UploadResponse)
async def upload_files(request: Request, files: list[UploadFile] = File(...)) -> UploadResponse:
    file_store = request.app.state.file_store
    saved = []
    for upload in files:
        saved.append(await file_store.save_upload(upload))
    return UploadResponse(files=saved)
