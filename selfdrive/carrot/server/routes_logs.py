"""
로그 탭(대시캠 / 화면녹화) 백엔드 라우트.

콤마 기기(C3X)에 저장된 openpilot 주행 세그먼트(rlog/qlog/qcamera)와
화면 녹화 파일을 목록으로 보여주고, 브라우저(핸드폰 등)에서 다운로드할 수
있도록 하는 최소 구현입니다. c3-atune 브랜치의
selfdrive/carrot/server/features/dashcam, features/screenrecord 를
참고해 c3-ms 백엔드 구조(단일 모듈 + app_factory 라우트 등록)에 맞게
새로 작성했습니다. 업로드(FTP)/썸네일(ffmpeg) 기능은 포함하지 않았습니다
(요청 범위: 다운로드).
"""

import asyncio
import hashlib
import io
import mimetypes
import os
import threading
import time
import zipfile
from datetime import datetime
from typing import Any

from aiohttp import web

# ---------------------------------------------------------------------------
# 경로 설정 (openpilot/carrot 표준 경로. c3-atune 의 config.py 값과 동일)
# ---------------------------------------------------------------------------
DASHCAM_ROOT = "/data/media/0/realdata"

SCREEN_RECORDING_DIRS = (
  "/data/media/0/videos",
  "/data/media/0/screenrecord",
  "/data/media/0/screen_recordings",
  "/data/media/0/screenrecords",
  "/data/media/0/ScreenRecords",
  "/data/media/0/Movies",
  "/sdcard/Movies",
)
SCREEN_RECORDING_EXTS = (".mp4", ".mkv", ".avi", ".mov", ".ts", ".hevc")

DASHCAM_ARTIFACT_NAMES = {
  "qcamera": ("qcamera.ts", "qcamera.mp4"),
  "rlog": ("rlog.zst", "rlog.bz2", "rlog"),
  "qlog": ("qlog.zst", "qlog.bz2", "qlog"),
}

ROUTE_CACHE_TTL = 3.0
DASHCAM_ROUTE_LIMIT_DEFAULT = 40
DASHCAM_ROUTE_LIMIT_MAX = 200

_route_cache_lock = threading.Lock()
_route_cache = {"time": 0.0, "routes": []}

_video_cache_lock = threading.Lock()
_video_cache = {"time": 0.0, "videos": []}


# ---------------------------------------------------------------------------
# 경로/포맷 헬퍼
# ---------------------------------------------------------------------------
def safe_segment(segment: str) -> str:
  segment = (segment or "").strip()
  if not segment or "/" in segment or "\\" in segment or segment in {".", ".."}:
    raise web.HTTPBadRequest(text="bad segment")
  parts = segment.split("--")
  if len(parts) < 2 or not parts[-1].isdigit():
    raise web.HTTPBadRequest(text="bad segment")
  return segment


def segment_index(segment: str) -> int:
  try:
    return int(segment.split("--")[-1])
  except Exception:
    return 0


def route_name(segment: str) -> str:
  try:
    return "--".join(str(segment or "").split("--")[:-1])
  except Exception:
    return str(segment or "")


def file_size_label(size: int) -> str:
  try:
    n = float(size)
  except Exception:
    return "-"
  if n < 1024:
    return f"{int(n)} B"
  if n < 1024 * 1024:
    return f"{n / 1024:.1f} KB"
  if n < 1024 * 1024 * 1024:
    return f"{n / (1024 * 1024):.1f} MB"
  return f"{n / (1024 * 1024 * 1024):.1f} GB"


def date_label(epoch_seconds: int) -> str:
  """실제 파일 mtime 기준 절대 날짜/시간 라벨 (예: 2026-08-01 16:40)."""
  if not epoch_seconds:
    return "-"
  try:
    return datetime.fromtimestamp(epoch_seconds).strftime("%Y-%m-%d %H:%M")
  except Exception:
    return "-"


def compact_datetime(epoch_seconds: int) -> str:
  """다운로드 파일명에 붙일 압축 날짜/시간 (예: 20260801_1640)."""
  if not epoch_seconds:
    return "unknown-date"
  try:
    return datetime.fromtimestamp(epoch_seconds).strftime("%Y%m%d_%H%M%S")
  except Exception:
    return "unknown-date"


def relative_time(epoch_seconds: int) -> str:
  if epoch_seconds <= 0:
    return "-"
  delta = max(0, int(time.time()) - int(epoch_seconds))
  if delta < 60:
    return "방금 전"
  if delta < 3600:
    return f"{delta // 60}분 전"
  if delta < 86400:
    return f"{delta // 3600}시간 전"
  return f"{delta // 86400}일 전"


def segment_dir(segment: str) -> str:
  segment = safe_segment(segment)
  root = os.path.abspath(DASHCAM_ROOT)
  path = os.path.abspath(os.path.join(root, segment))
  if not path.startswith(root + os.sep):
    raise web.HTTPBadRequest(text="bad segment path")
  if not os.path.isdir(path):
    raise web.HTTPNotFound(text="segment not found")
  return path


# ---------------------------------------------------------------------------
# 대시캠(주행 로그/영상) 카탈로그
# ---------------------------------------------------------------------------
def has_source_video(segment_dir_path: str) -> bool:
  for name in ("qcamera.mp4", "qcamera.ts"):
    path = os.path.join(segment_dir_path, name)
    if os.path.isfile(path) and os.path.getsize(path) > 0:
      return True
  return False


def segment_file_summary(segment_dir_path: str) -> list[dict[str, Any]]:
  out: list[dict[str, Any]] = []
  for name in ("qcamera.mp4", "qcamera.ts", "rlog.zst", "rlog.bz2", "rlog", "qlog.zst", "qlog.bz2", "qlog"):
    path = os.path.join(segment_dir_path, name)
    if os.path.isfile(path):
      try:
        size = os.path.getsize(path)
      except OSError:
        size = 0
      out.append({"name": name, "size": size, "sizeLabel": file_size_label(size)})
  return out


def build_routes() -> list[dict[str, Any]]:
  if not os.path.isdir(DASHCAM_ROOT):
    return []

  route_segments: dict[str, list[str]] = {}
  route_modified: dict[str, int] = {}
  segment_modified: dict[str, int] = {}
  with os.scandir(DASHCAM_ROOT) as it:
    for entry in it:
      try:
        if not entry.is_dir(follow_symlinks=False) or "--" not in entry.name:
          continue
        if not has_source_video(entry.path):
          continue
        parts = entry.name.split("--")
        if len(parts) < 2 or not parts[-1].isdigit():
          continue
        route = "--".join(parts[:-1])
        route_segments.setdefault(route, []).append(entry.name)
        modified = int(entry.stat(follow_symlinks=False).st_mtime)
        segment_modified[entry.name] = modified
        if modified > route_modified.get(route, 0):
          route_modified[route] = modified
      except Exception:
        continue

  routes: list[dict[str, Any]] = []
  for route, segments in route_segments.items():
    sorted_segments = sorted(segments, key=lambda s: (segment_index(s), s))
    latest = route_modified.get(route, 0)
    segment_details = [
      {
        "segment": s,
        "segmentIndex": segment_index(s),
        "modifiedEpoch": segment_modified.get(s, 0),
        "dateLabel": date_label(segment_modified.get(s, 0)),
      }
      for s in sorted_segments
    ]
    routes.append({
      "route": route,
      "title": route.lstrip("0") or route,
      "dateLabel": date_label(latest),
      "segmentFolders": sorted_segments,
      "segments": segment_details,
      "segmentCount": len(sorted_segments),
      "latestModifiedEpoch": latest,
      "latestModifiedLabel": relative_time(latest),
    })
  routes.sort(key=lambda r: (r.get("route", ""), r.get("latestModifiedEpoch", 0)), reverse=True)
  return routes


def cached_dashcam_routes() -> list[dict]:
  now = time.monotonic()
  with _route_cache_lock:
    if now - float(_route_cache.get("time") or 0.0) < ROUTE_CACHE_TTL:
      return list(_route_cache.get("routes") or [])
  routes = build_routes()
  with _route_cache_lock:
    _route_cache["time"] = time.monotonic()
    _route_cache["routes"] = routes
  return list(routes)


def find_dashcam_route(routes: list[dict], route: str) -> dict | None:
  if not route or "/" in route or "\\" in route or route in (".", ".."):
    return None
  for entry in routes:
    if entry.get("route") == route:
      return entry
  return None


def bounded_query_int(request: web.Request, name: str, default: int, maximum: int) -> int:
  try:
    value = int(request.query.get(name, str(default)) or default)
  except (TypeError, ValueError):
    value = default
  return max(0 if name == "offset" else 1, min(maximum, value))


# ---------------------------------------------------------------------------
# 대시캠 API
# ---------------------------------------------------------------------------
async def api_dashcam_routes(request: web.Request) -> web.Response:
  try:
    offset = bounded_query_int(request, "offset", 0, 1000000)
    limit = bounded_query_int(request, "limit", DASHCAM_ROUTE_LIMIT_DEFAULT, DASHCAM_ROUTE_LIMIT_MAX)
    routes = await asyncio.to_thread(cached_dashcam_routes)
    total = len(routes)
    end = min(offset + limit, total)
    return web.json_response({
      "ok": True,
      "routes": routes[offset:end],
      "root": DASHCAM_ROOT,
      "offset": offset,
      "limit": limit,
      "total": total,
      "nextOffset": end if end < total else None,
      "hasMore": end < total,
    })
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


async def api_dashcam_segments(request: web.Request) -> web.Response:
  try:
    route = request.match_info.get("route", "")
    routes = await asyncio.to_thread(cached_dashcam_routes)
    entry = find_dashcam_route(routes, route)
    if not entry:
      return web.json_response({"ok": False, "error": "route not found"}, status=404)

    segments = list(entry.get("segmentFolders") or [])
    summaries = []
    for segment in segments:
      seg_dir = os.path.join(DASHCAM_ROOT, segment)
      files = await asyncio.to_thread(segment_file_summary, seg_dir)
      total_size = sum(int(item.get("size") or 0) for item in files)
      try:
        modified = int(await asyncio.to_thread(os.path.getmtime, seg_dir))
      except OSError:
        modified = 0
      summaries.append({
        "segment": segment,
        "segmentIndex": segment_index(segment),
        "files": files,
        "totalSize": total_size,
        "totalSizeLabel": file_size_label(total_size),
        "modifiedEpoch": modified,
        "dateLabel": date_label(modified),
      })
    return web.json_response({"ok": True, "route": route, "segments": summaries})
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


async def api_dashcam_download(request: web.Request) -> web.StreamResponse:
  segment = request.match_info.get("segment", "")
  kind = (request.match_info.get("kind", "") or "").strip()
  segment_path = await asyncio.to_thread(segment_dir, segment)
  for name in DASHCAM_ARTIFACT_NAMES.get(kind, ()):
    path = os.path.join(segment_path, name)
    if os.path.isfile(path):
      mime = mimetypes.guess_type(path)[0] or "application/octet-stream"
      try:
        mtime = int(await asyncio.to_thread(os.path.getmtime, path))
      except OSError:
        mtime = 0
      date_prefix = compact_datetime(mtime)
      ext = name.split(".", 1)[1] if "." in name else name
      download_name = f"{date_prefix}_{segment}_{kind}.{ext}" if ext else f"{date_prefix}_{segment}_{name}"
      return web.FileResponse(
        path,
        headers={
          "Content-Type": mime,
          "Content-Disposition": f'attachment; filename="{download_name}"',
        },
      )
  raise web.HTTPNotFound(text="artifact not found")


async def api_dashcam_download_zip(request: web.Request) -> web.StreamResponse:
  """선택한 세그먼트들(영상+로그)을 하나의 zip으로 묶어 다운로드."""
  try:
    body = await request.json()
  except Exception:
    body = {}
  segments = body.get("segments")
  if not isinstance(segments, list) or not segments:
    return web.json_response({"ok": False, "error": "missing segments"}, status=400)
  segments = [safe_segment(str(s)) for s in segments][:200]  # 안전 상한

  def build_zip() -> bytes:
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_STORED) as zf:
      for segment in segments:
        try:
          seg_path = segment_dir(segment)
        except web.HTTPException:
          continue
        try:
          seg_mtime = int(os.path.getmtime(seg_path))
        except OSError:
          seg_mtime = 0
        date_prefix = compact_datetime(seg_mtime)
        for name in ("qcamera.mp4", "qcamera.ts", "rlog.zst", "rlog.bz2", "rlog", "qlog.zst", "qlog.bz2", "qlog"):
          path = os.path.join(seg_path, name)
          if os.path.isfile(path):
            zf.write(path, arcname=f"{date_prefix}_{segment}/{name}")
    return buf.getvalue()

  data = await asyncio.to_thread(build_zip)
  filename = f"dashcam_{int(time.time())}.zip"
  return web.Response(
    body=data,
    headers={
      "Content-Type": "application/zip",
      "Content-Disposition": f'attachment; filename="{filename}"',
    },
  )


# ---------------------------------------------------------------------------
# 화면 녹화(screenrecord) 카탈로그 + API
# ---------------------------------------------------------------------------
def file_id(path: str) -> str:
  return hashlib.sha1(os.path.abspath(path).encode("utf-8", errors="ignore")).hexdigest()[:24]


def build_screen_videos() -> list[dict[str, Any]]:
  videos: list[dict[str, Any]] = []
  seen: set[str] = set()
  for folder in SCREEN_RECORDING_DIRS:
    if not os.path.isdir(folder):
      continue
    try:
      with os.scandir(folder) as it:
        for entry in it:
          try:
            name = entry.name
            if not entry.is_file(follow_symlinks=False):
              continue
            if not name.lower().endswith(SCREEN_RECORDING_EXTS):
              continue
            stat = entry.stat(follow_symlinks=False)
            if stat.st_size <= 0:
              continue
            path = os.path.abspath(entry.path)
            real = os.path.realpath(path)
            if real in seen:
              continue
            seen.add(real)
            modified = int(stat.st_mtime)
            videos.append({
              "id": file_id(path),
              "name": name,
              "size": int(stat.st_size),
              "sizeLabel": file_size_label(int(stat.st_size)),
              "modifiedEpoch": modified,
              "modifiedLabel": date_label(modified),
              "relativeModifiedLabel": relative_time(modified),
            })
          except Exception:
            continue
    except Exception:
      continue
  videos.sort(key=lambda item: (item.get("modifiedEpoch", 0), item.get("name", "")), reverse=True)
  return videos


def cached_screen_videos() -> list[dict]:
  now = time.monotonic()
  with _video_cache_lock:
    if now - float(_video_cache.get("time") or 0.0) < ROUTE_CACHE_TTL:
      return list(_video_cache.get("videos") or [])
  videos = build_screen_videos()
  with _video_cache_lock:
    _video_cache["time"] = time.monotonic()
    _video_cache["videos"] = videos
  return list(videos)


def find_screen_video(file_id_in: str) -> str:
  file_id_in = (file_id_in or "").strip()
  if not file_id_in or "/" in file_id_in or "\\" in file_id_in or len(file_id_in) > 64:
    raise web.HTTPBadRequest(text="bad file id")
  for item in build_screen_videos():
    for folder in SCREEN_RECORDING_DIRS:
      path = os.path.abspath(os.path.join(folder, item.get("name", "")))
      if file_id(path) == file_id_in and os.path.isfile(path):
        return path
  raise web.HTTPNotFound(text="screen recording not found")


async def api_screenrecord_videos(request: web.Request) -> web.Response:
  try:
    videos = await asyncio.to_thread(cached_screen_videos)
    return web.json_response({"ok": True, "videos": videos, "total": len(videos)})
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


async def api_screenrecord_download(request: web.Request) -> web.StreamResponse:
  file_id_in = request.match_info.get("file_id", "")
  path = await asyncio.to_thread(find_screen_video, file_id_in)
  filename = os.path.basename(path)
  try:
    mtime = int(await asyncio.to_thread(os.path.getmtime, path))
  except OSError:
    mtime = 0
  date_prefix = compact_datetime(mtime)
  safe_filename = "".join(ch if 32 <= ord(ch) < 127 and ch not in {'"', "\\"} else "_" for ch in filename)
  download_name = f"{date_prefix}_{safe_filename}" if safe_filename else f"{date_prefix}_screenrecord"
  mime = mimetypes.guess_type(path)[0] or "application/octet-stream"
  return web.FileResponse(
    path,
    headers={
      "Content-Type": mime,
      "Content-Disposition": f'attachment; filename="{download_name}"',
    },
  )
