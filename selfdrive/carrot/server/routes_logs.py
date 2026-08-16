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
import json
import mimetypes
import os
import threading
import time
import urllib.parse
import urllib.request
import uuid
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

GDRIVE_STATE = {
  "connected": False,
  "access_token": None,
  "token_type": "Bearer",
  "scope": "",
  "expires_at": 0,
  "client_id": "",
  "client_secret": "",
  "device_code": "",
  "user_code": "",
  "verification_uri": "",
  "interval": 5,
  "status": "disconnected",
  "last_error": "",
  "folder_id": "",
}


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


# ---------------------------------------------------------------------------
# Google Drive (Device Authorization Grant, minimal upload support)
# ---------------------------------------------------------------------------

def _gdrive_status_payload() -> dict[str, Any]:
  return {
    "ok": True,
    "connected": bool(GDRIVE_STATE["access_token"]),
    "status": GDRIVE_STATE["status"],
    "user_code": GDRIVE_STATE["user_code"],
    "verification_uri": GDRIVE_STATE["verification_uri"],
    "device_code": GDRIVE_STATE["device_code"],
    "folder_id": GDRIVE_STATE["folder_id"],
    "last_error": GDRIVE_STATE["last_error"],
  }


async def api_gdrive_status(request: web.Request) -> web.Response:
  return web.json_response(_gdrive_status_payload())


async def api_gdrive_device(request: web.Request) -> web.Response:
  try:
    payload = await request.json()
  except Exception:
    payload = {}

  client_id = str(payload.get("client_id") or "").strip()
  client_secret = str(payload.get("client_secret") or "").strip()
  if not client_id:
    return web.json_response({"ok": False, "error": "missing client_id"}, status=400)

  GDRIVE_STATE["client_id"] = client_id
  GDRIVE_STATE["client_secret"] = client_secret
  GDRIVE_STATE["last_error"] = ""

  data = urllib.parse.urlencode({
    "client_id": client_id,
    "scope": "https://www.googleapis.com/auth/drive.file",
  }).encode("utf-8")
  req = urllib.request.Request(
    "https://oauth2.googleapis.com/device/code",
    data=data,
    headers={"Content-Type": "application/x-www-form-urlencoded"},
    method="POST",
  )
  try:
    def _do():
      with urllib.request.urlopen(req, timeout=30) as resp:
        return json.loads(resp.read().decode("utf-8"))
    body = await asyncio.to_thread(_do)
  except Exception as e:
    GDRIVE_STATE["status"] = "error"
    GDRIVE_STATE["last_error"] = str(e)
    return web.json_response({"ok": False, "error": str(e)}, status=500)

  GDRIVE_STATE["device_code"] = str(body.get("device_code") or "")
  GDRIVE_STATE["user_code"] = str(body.get("user_code") or "")
  GDRIVE_STATE["verification_uri"] = str(body.get("verification_uri") or "https://www.google.com/device")
  GDRIVE_STATE["interval"] = int(body.get("interval") or 5)
  GDRIVE_STATE["status"] = "pending"
  return web.json_response({
    "ok": True,
    "device_code": GDRIVE_STATE["device_code"],
    "user_code": GDRIVE_STATE["user_code"],
    "verification_uri": GDRIVE_STATE["verification_uri"],
    "interval": GDRIVE_STATE["interval"],
    "status": GDRIVE_STATE["status"],
  })


async def api_gdrive_token(request: web.Request) -> web.Response:
  """Device flow 토큰 폴링. 프론트가 5초 간격으로 이 엔드포인트를 반복
  호출해야 사용자의 Google 승인 여부를 알 수 있다 (Google 쪽에서 알아서
  push 해주지 않음 — device authorization grant는 클라이언트가 계속
  물어봐야 하는 방식)."""
  try:
    payload = await request.json()
  except Exception:
    payload = {}

  client_id = str(payload.get("client_id") or GDRIVE_STATE["client_id"] or "").strip()
  client_secret = str(payload.get("client_secret") or GDRIVE_STATE["client_secret"] or "").strip()
  device_code = str(payload.get("device_code") or GDRIVE_STATE["device_code"] or "").strip()
  if not client_id or not device_code:
    return web.json_response({"ok": False, "error": "missing client_id or device_code"}, status=400)

  data = urllib.parse.urlencode({
    "client_id": client_id,
    "client_secret": client_secret,
    "device_code": device_code,
    "grant_type": "urn:ietf:params:oauth:grant-type:device_code",
  }).encode("utf-8")
  req = urllib.request.Request(
    "https://oauth2.googleapis.com/token",
    data=data,
    headers={"Content-Type": "application/x-www-form-urlencoded"},
    method="POST",
  )
  try:
    def _do():
      with urllib.request.urlopen(req, timeout=30) as resp:
        return json.loads(resp.read().decode("utf-8"))
    body = await asyncio.to_thread(_do)
  except urllib.error.HTTPError as e:
    try:
      err = json.loads(e.read().decode("utf-8"))
    except Exception:
      err = {"error": str(e)}
    err_code = err.get("error") if isinstance(err, dict) else None
    if err_code in ("authorization_pending", "slow_down"):
      # 사용자가 아직 Google 화면에서 승인하지 않은 "정상" 상태.
      # 이걸 에러로 취급하면 폴링 첫 시도에서 바로 status="error"가 되어
      # 실제로 승인해도 절대 연결되지 않는다 (보고된 증상의 2차 원인).
      return web.json_response({
        "ok": True,
        "connected": False,
        "pending": True,
        "status": GDRIVE_STATE["status"],
      })
    GDRIVE_STATE["status"] = "error"
    GDRIVE_STATE["last_error"] = str(err)
    return web.json_response({"ok": False, "error": err}, status=400)
  except Exception as e:
    GDRIVE_STATE["status"] = "error"
    GDRIVE_STATE["last_error"] = str(e)
    return web.json_response({"ok": False, "error": str(e)}, status=500)

  access_token = str(body.get("access_token") or "")
  refresh_token = str(body.get("refresh_token") or "")
  expires_in = int(body.get("expires_in") or 3600)
  if access_token:
    GDRIVE_STATE["access_token"] = access_token
    GDRIVE_STATE["token_type"] = str(body.get("token_type") or "Bearer")
    GDRIVE_STATE["scope"] = str(body.get("scope") or "")
    GDRIVE_STATE["expires_at"] = int(time.time()) + expires_in
    GDRIVE_STATE["status"] = "connected"
    GDRIVE_STATE["last_error"] = ""
    if refresh_token:
      GDRIVE_STATE["refresh_token"] = refresh_token
  return web.json_response({
    "ok": bool(access_token),
    "connected": bool(access_token),
    "pending": not access_token,
    "status": GDRIVE_STATE["status"],
    "message": "Google Drive 연결 완료" if access_token else "권한을 아직 승인하지 않았습니다",
    "token": body,
  })



def _resolve_drive_payload(body: dict[str, Any]) -> tuple[str, str, str] | None:
  kind = str(body.get("kind") or "").strip()
  if kind == "dashcam":
    segment = safe_segment(str(body.get("segment") or ""))
    path = os.path.join(DASHCAM_ROOT, segment)
    candidates = ["qcamera.mp4", "qcamera.ts", "rlog.zst", "rlog.bz2", "rlog", "qlog.zst", "qlog.bz2", "qlog"]
    for name in candidates:
      full = os.path.join(path, name)
      if os.path.isfile(full):
        return full, os.path.basename(full), segment
    return None
  if kind == "screenrecord":
    file_id_in = str(body.get("file_id") or "").strip()
    if not file_id_in:
      return None
    for video in build_screen_videos():
      path = os.path.abspath(os.path.join(next((d for d in SCREEN_RECORDING_DIRS if os.path.isdir(d)), SCREEN_RECORDING_DIRS[0]), video.get("name", "")))
      if file_id(path) == file_id_in and os.path.isfile(path):
        return path, os.path.basename(path), file_id_in
    return None
  return None


def _upload_file_to_drive(file_path: str, filename: str, access_token: str, folder_id: str = "") -> dict[str, Any]:
  if not os.path.isfile(file_path):
    raise FileNotFoundError(file_path)

  metadata = {"name": filename}
  if folder_id:
    metadata["parents"] = [folder_id]

  boundary = uuid.uuid4().hex
  file_bytes = open(file_path, "rb").read()
  mime_type = mimetypes.guess_type(filename)[0] or "application/octet-stream"
  body = (
    f"--{boundary}\r\n"
    f"Content-Type: application/json; charset=UTF-8\r\n\r\n"
    f"{json.dumps(metadata)}\r\n"
    f"--{boundary}\r\n"
    f"Content-Type: {mime_type}\r\n"
    f"Content-Transfer-Encoding: binary\r\n\r\n"
  ).encode("utf-8") + file_bytes + f"\r\n--{boundary}--\r\n".encode("utf-8")

  req = urllib.request.Request(
    "https://www.googleapis.com/upload/drive/v3/files?uploadType=multipart",
    data=body,
    headers={
      "Authorization": f"Bearer {access_token}",
      "Content-Type": f"multipart/related; boundary={boundary}",
    },
    method="POST",
  )
  try:
    with urllib.request.urlopen(req, timeout=120) as resp:
      response_body = resp.read().decode("utf-8", errors="replace")
      parsed = json.loads(response_body) if response_body else {}
      return {"ok": True, "file_id": parsed.get("id"), "name": parsed.get("name") or filename, "webViewLink": parsed.get("webViewLink"), "drive": parsed}
  except Exception as e:
    raise RuntimeError(str(e))


async def api_gdrive_upload(request: web.Request) -> web.Response:
  try:
    payload = await request.json()
  except Exception:
    payload = {}

  token = str(GDRIVE_STATE.get("access_token") or "").strip()
  if not token:
    return web.json_response({"ok": False, "error": "Google Drive not connected"}, status=401)

  resolved = _resolve_drive_payload(payload)
  if not resolved:
    return web.json_response({"ok": False, "error": "no upload target found"}, status=400)

  file_path, filename, _ = resolved
  try:
    result = await asyncio.to_thread(_upload_file_to_drive, file_path, filename, token, GDRIVE_STATE.get("folder_id", ""))
    return web.json_response({"ok": True, "result": result, "name": filename})
  except Exception as e:
    GDRIVE_STATE["status"] = "error"
    GDRIVE_STATE["last_error"] = str(e)
    return web.json_response({"ok": False, "error": str(e)}, status=500)
