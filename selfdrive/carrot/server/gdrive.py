"""
Google Drive 연동 — 로그 탭 "Drive 업로드" 기능.

c3-ms-web 브랜치에서 검증된 방식을 c3-ms-dev의 기존 엔드포인트
계약(/api/gdrive/status, /api/gdrive/device, /api/gdrive/token,
/api/gdrive/upload + 프론트 web/js/logs.js)에 맞춰 이식한 버전입니다.

핵심 설계:
  1. OAuth2 "Device Authorization Grant" — 콤마 기기 자체 브라우저 없이도
     폰/PC에서 코드를 입력해 인증할 수 있음. refresh_token은 Params에
     영구 저장하여 carrotweb 재시작/기기 재부팅 후에도 재인증이 필요 없음.
  2. 업로드는 uploadType=resumable을 청크(8MB) 단위로 PUT.
     - uploadType=multipart는 요청 본문 5MB 제한이 있어 대시캠 영상처럼
       큰 파일에서 "Malformed multipart body." 400을 유발함 (실제 관찰됨).
     - 파일을 한 번에 메모리로 읽지 않고(open().read()) 청크 단위로만
       읽어 올려 대용량 파일 전송 중 OOM을 방지함.
  3. 업로드는 시간이 걸릴 수 있어 job 방식으로 비동기 처리 — 요청은
     즉시 job_id를 반환하고, 프론트는 /api/gdrive/job?id=...로 폴링해
     진행률(%)을 받는다 (core.py의 tools job과 동일한 패턴).

필요 사전 준비 (Google Cloud Console에서 1회 설정):
  1. https://console.cloud.google.com/ 에서 프로젝트 생성
  2. "API 및 서비스 > 라이브러리"에서 Google Drive API 활성화
  3. "API 및 서비스 > 사용자 인증 정보 > OAuth 클라이언트 ID 만들기"
     -> 애플리케이션 유형: "TV 및 제한된 입력이 있는 기기" (필수!)
  4. 발급된 클라이언트 ID / 클라이언트 보안 비밀번호를 로그탭 설정에 입력

권한 범위는 drive.file(앱이 만든 파일만 접근)로 최소화했다.
"""

import os
import time
import uuid
from typing import Any

import aiohttp
from aiohttp import web

try:
  from openpilot.common.params import Params as _Params
  HAS_PARAMS = True
except Exception:
  _Params = None
  HAS_PARAMS = False

DEVICE_CODE_URL = "https://oauth2.googleapis.com/device/code"
TOKEN_URL = "https://oauth2.googleapis.com/token"
DRIVE_FILES_URL = "https://www.googleapis.com/drive/v3/files"
DRIVE_UPLOAD_URL = "https://www.googleapis.com/upload/drive/v3/files"
DRIVE_SCOPE = "https://www.googleapis.com/auth/drive.file"
DRIVE_FOLDER_NAME = "CarrotWeb Logs"

PARAM_CLIENT_ID = "CarrotGDriveClientId"
PARAM_CLIENT_SECRET = "CarrotGDriveClientSecret"
PARAM_REFRESH_TOKEN = "CarrotGDriveRefreshToken"

# device code flow 진행 상태 (짧게 유지되는 값이라 Params 대신 메모리에 보관)
_pending_flow: dict[str, Any] = {}
# access token 캐시 (만료 전까지 재사용, refresh_token으로부터 재발급)
_access_token_cache: dict[str, Any] = {"token": None, "expires_at": 0}
# 마지막 에러 (상태 배지에 표시용)
_last_error: dict[str, str] = {"message": ""}


# ---------------------------------------------------------------------------
# Params 기반 자격증명/연결 상태
# ---------------------------------------------------------------------------
def _params():
  if not HAS_PARAMS:
    raise RuntimeError("openpilot Params 모듈을 사용할 수 없습니다")
  return _Params()


def _param_str(key: str) -> str:
  try:
    v = _params().get(key)
  except Exception:
    return ""
  if v is None:
    return ""
  if isinstance(v, bytes):
    return v.decode("utf-8", errors="ignore")
  return str(v)


def get_client_credentials() -> tuple[str, str]:
  return _param_str(PARAM_CLIENT_ID), _param_str(PARAM_CLIENT_SECRET)


def is_connected() -> bool:
  return bool(_param_str(PARAM_REFRESH_TOKEN))


def disconnect() -> None:
  try:
    _params().put(PARAM_REFRESH_TOKEN, "")
  except Exception:
    pass
  _access_token_cache["token"] = None
  _access_token_cache["expires_at"] = 0


# ---------------------------------------------------------------------------
# 공통 응답 파싱 헬퍼
# ---------------------------------------------------------------------------
async def _read_json_safe(resp: aiohttp.ClientResponse) -> dict[str, Any]:
  """Google이 오류 응답을 text/plain 등으로 줄 때가 있어(특히 요청 자체가
  형식 오류일 때 GFE 단계에서 반환) content_type 체크 없이 우선 JSON
  파싱을 시도하고, 그래도 실패하면 원문 텍스트를 담은 에러로 변환한다."""
  try:
    return await resp.json(content_type=None)
  except Exception:
    text = (await resp.text())[:500]
    raise RuntimeError(f"HTTP {resp.status}: {text or '(empty body)'}") from None


# ---------------------------------------------------------------------------
# OAuth2 Device Authorization Grant
# ---------------------------------------------------------------------------
async def api_gdrive_status(request: web.Request) -> web.Response:
  client_id, client_secret = get_client_credentials()
  connected = is_connected()
  pending = bool(_pending_flow.get("device_code")) and not connected
  status = "connected" if connected else ("pending" if pending else ("error" if _last_error["message"] else "disconnected"))
  return web.json_response({
    "ok": True,
    "connected": connected,
    "status": status,
    "hasCredentials": bool(client_id and client_secret),
    "user_code": _pending_flow.get("user_code", ""),
    "verification_uri": _pending_flow.get("verification_uri", ""),
    "last_error": _last_error["message"],
  })


async def api_gdrive_device(request: web.Request) -> web.Response:
  """device code 플로우 시작. client_id/secret을 Params에 저장해둔다."""
  try:
    body = await request.json()
  except Exception:
    body = {}
  client_id = str(body.get("client_id") or "").strip()
  client_secret = str(body.get("client_secret") or "").strip()
  if not client_id:
    return web.json_response({"ok": False, "error": "missing client_id"}, status=400)

  try:
    _params().put(PARAM_CLIENT_ID, client_id)
    _params().put(PARAM_CLIENT_SECRET, client_secret)
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)

  try:
    async with aiohttp.ClientSession() as session:
      async with session.post(DEVICE_CODE_URL, data={
        "client_id": client_id,
        "scope": DRIVE_SCOPE,
      }) as resp:
        data = await _read_json_safe(resp)
        if resp.status != 200:
          _last_error["message"] = data.get("error_description", str(data))
          return web.json_response({"ok": False, "error": _last_error["message"]}, status=400)
  except Exception as e:
    _last_error["message"] = str(e)
    return web.json_response({"ok": False, "error": str(e)}, status=500)

  _pending_flow.clear()
  _pending_flow.update({
    "device_code": data.get("device_code"),
    "user_code": data.get("user_code", ""),
    "verification_uri": data.get("verification_url") or data.get("verification_uri") or "https://www.google.com/device",
    "interval": int(data.get("interval") or 5),
    "expires_at": time.monotonic() + float(data.get("expires_in", 1800)),
  })
  _last_error["message"] = ""

  return web.json_response({
    "ok": True,
    "device_code": _pending_flow["device_code"],
    "user_code": _pending_flow["user_code"],
    "verification_uri": _pending_flow["verification_uri"],
    "interval": _pending_flow["interval"],
    "status": "pending",
  })


async def api_gdrive_token(request: web.Request) -> web.Response:
  """폰/PC에서 사용자가 코드 입력을 완료했는지 폴링으로 확인.
  프론트가 이 엔드포인트를 5초 간격으로 반복 호출해야 승인 여부를 알 수
  있다 (Google이 먼저 알려주지 않음 — device flow는 클라이언트가 계속
  물어봐야 하는 방식)."""
  try:
    body = await request.json()
  except Exception:
    body = {}

  saved_client_id, saved_client_secret = get_client_credentials()
  client_id = str(body.get("client_id") or saved_client_id or "").strip()
  client_secret = str(body.get("client_secret") or saved_client_secret or "").strip()
  device_code = str(body.get("device_code") or _pending_flow.get("device_code") or "").strip()
  if not client_id or not device_code:
    return web.json_response({"ok": False, "error": "missing client_id or device_code"}, status=400)

  if _pending_flow.get("expires_at") and time.monotonic() > _pending_flow["expires_at"]:
    _pending_flow.clear()
    _last_error["message"] = "인증 코드가 만료되었습니다. 다시 시도해주세요."
    return web.json_response({"ok": False, "error": _last_error["message"], "expired": True}, status=400)

  try:
    async with aiohttp.ClientSession() as session:
      async with session.post(TOKEN_URL, data={
        "client_id": client_id,
        "client_secret": client_secret,
        "device_code": device_code,
        "grant_type": "urn:ietf:params:oauth:grant-type:device_code",
      }) as resp:
        data = await _read_json_safe(resp)
  except Exception as e:
    _last_error["message"] = str(e)
    return web.json_response({"ok": False, "error": str(e)}, status=500)

  error = data.get("error")
  if error in ("authorization_pending", "slow_down"):
    # 사용자가 아직 Google 화면에서 승인하지 않은 "정상" 대기 상태.
    # 에러로 취급하면 폴링 첫 시도에서 바로 실패 처리되어 버린다.
    return web.json_response({"ok": True, "connected": False, "pending": True, "status": "pending"})
  if error:
    _pending_flow.clear()
    _last_error["message"] = data.get("error_description", error)
    return web.json_response({"ok": False, "error": _last_error["message"]}, status=400)

  access_token = str(data.get("access_token") or "")
  refresh_token = str(data.get("refresh_token") or "")
  expires_in = int(data.get("expires_in") or 3600)
  if not access_token:
    return web.json_response({"ok": False, "error": "access_token 없음"}, status=500)

  if refresh_token:
    try:
      _params().put(PARAM_REFRESH_TOKEN, refresh_token)
    except Exception as e:
      _last_error["message"] = str(e)
      return web.json_response({"ok": False, "error": str(e)}, status=500)

  _access_token_cache["token"] = access_token
  _access_token_cache["expires_at"] = time.monotonic() + float(expires_in) - 60
  _pending_flow.clear()
  _last_error["message"] = ""

  return web.json_response({
    "ok": True,
    "connected": True,
    "pending": False,
    "status": "connected",
    "message": "Google Drive 연결 완료",
  })


async def api_gdrive_disconnect(request: web.Request) -> web.Response:
  disconnect()
  return web.json_response({"ok": True, "connected": False})


# ---------------------------------------------------------------------------
# access token 관리 (refresh_token으로 자동 재발급)
# ---------------------------------------------------------------------------
async def _get_access_token(session: aiohttp.ClientSession) -> str:
  now = time.monotonic()
  if _access_token_cache["token"] and now < _access_token_cache["expires_at"]:
    return _access_token_cache["token"]

  client_id, client_secret = get_client_credentials()
  refresh_token = _param_str(PARAM_REFRESH_TOKEN)
  if not (client_id and refresh_token):
    raise RuntimeError("Google Drive가 연결되어 있지 않습니다")

  async with session.post(TOKEN_URL, data={
    "client_id": client_id,
    "client_secret": client_secret,
    "refresh_token": refresh_token,
    "grant_type": "refresh_token",
  }) as resp:
    data = await _read_json_safe(resp)
    if resp.status != 200:
      raise RuntimeError(data.get("error_description", str(data)))

  token = data["access_token"]
  _access_token_cache["token"] = token
  _access_token_cache["expires_at"] = now + float(data.get("expires_in", 3600)) - 60
  return token


async def _ensure_folder(session: aiohttp.ClientSession, token: str) -> str:
  headers = {"Authorization": f"Bearer {token}"}
  query = f"name='{DRIVE_FOLDER_NAME}' and mimeType='application/vnd.google-apps.folder' and trashed=false"
  async with session.get(DRIVE_FILES_URL, headers=headers, params={"q": query, "fields": "files(id,name)"}) as resp:
    data = await _read_json_safe(resp)
  files = data.get("files") or []
  if files:
    return files[0]["id"]

  async with session.post(
    DRIVE_FILES_URL,
    headers={**headers, "Content-Type": "application/json"},
    json={"name": DRIVE_FOLDER_NAME, "mimeType": "application/vnd.google-apps.folder"},
  ) as resp:
    data = await _read_json_safe(resp)
    if resp.status not in (200, 201):
      raise RuntimeError(data.get("error", {}).get("message", str(data)))
    return data["id"]


# ---------------------------------------------------------------------------
# 업로드 진행률 조회용 job 저장소 (core.py의 _tool_jobs 패턴과 동일한 방식)
# ---------------------------------------------------------------------------
UPLOAD_JOB_KEEP_COUNT = 12
_upload_jobs: dict[str, dict[str, Any]] = {}


def create_job() -> dict[str, Any]:
  job_id = uuid.uuid4().hex[:12]
  job: dict[str, Any] = {
    "id": job_id,
    "status": "running",  # running | done | failed
    "message": "준비 중...",
    "sent": 0,
    "total": 0,
    "percent": 0,
    "error": None,
    "result": None,
    "created_at": time.time(),
    "updated_at": time.time(),
  }
  _upload_jobs[job_id] = job
  _prune_jobs()
  return job


def get_job(job_id: str) -> dict[str, Any] | None:
  return _upload_jobs.get(job_id)


def job_snapshot(job: dict[str, Any]) -> dict[str, Any]:
  return {
    "ok": True,
    "id": job["id"],
    "status": job["status"],
    "done": job["status"] in ("done", "failed"),
    "message": job.get("message") or "",
    "sent": job.get("sent"),
    "total": job.get("total"),
    "percent": job.get("percent"),
    "error": job.get("error"),
    "result": job.get("result"),
  }


def _touch_job(job: dict[str, Any]) -> None:
  job["updated_at"] = time.time()


def set_job_message(job: dict[str, Any] | None, message: str) -> None:
  if not job:
    return
  job["message"] = message
  _touch_job(job)


def _set_job_progress(job: dict[str, Any] | None, sent: int, total: int, message: str | None = None) -> None:
  if not job:
    return
  job["sent"] = sent
  job["total"] = total
  job["percent"] = int(max(0, min(100, round(sent / total * 100)))) if total else 0
  if message is not None:
    job["message"] = message
  _touch_job(job)


def finish_job(job: dict[str, Any] | None, *, ok: bool, result: dict[str, Any] | None = None, error: str | None = None) -> None:
  if not job:
    return
  job["status"] = "done" if ok else "failed"
  job["result"] = result
  job["error"] = error
  if ok:
    job["percent"] = 100
    job["message"] = "완료"
  _touch_job(job)
  _prune_jobs()


def _prune_jobs() -> None:
  finished = [j for j in _upload_jobs.values() if j.get("status") in ("done", "failed")]
  if len(finished) <= UPLOAD_JOB_KEEP_COUNT:
    return
  finished.sort(key=lambda j: float(j.get("updated_at") or 0), reverse=True)
  for old in finished[UPLOAD_JOB_KEEP_COUNT:]:
    _upload_jobs.pop(old["id"], None)


async def api_gdrive_job(request: web.Request) -> web.Response:
  job_id = (request.query.get("id") or "").strip()
  if not job_id:
    return web.json_response({"ok": False, "error": "missing job id"}, status=400)
  job = get_job(job_id)
  if not job:
    return web.json_response({"ok": False, "error": "job not found"}, status=404)
  return web.json_response(job_snapshot(job))


# ---------------------------------------------------------------------------
# 업로드 (resumable, 디스크에서 청크 단위로 직접 읽어 전송 — 메모리에
# 파일 전체를 올리지 않음)
# ---------------------------------------------------------------------------
# 느린 모바일 회선에서 대용량 영상 업로드가 끝까지 갈 수 있도록 넉넉한
# 타임아웃을 둔다. 기본 aiohttp 타임아웃(5분)은 큰 전송에서 서버 쪽
# 예외가 str(e)=="" 인 asyncio.TimeoutError로 끝나 프론트에 의미 없는
# 메시지만 남기는 원인이 될 수 있다.
_UPLOAD_TIMEOUT = aiohttp.ClientTimeout(total=1800, sock_connect=30, sock_read=300)

# Google Drive의 uploadType=multipart는 요청 본문 5MB 제한이 있다. 대시캠
# 영상 파일은 보통 이보다 커서 multipart로 보내면 GFE가 파싱 단계에서
# 거부하고 "Malformed multipart body." 같은 text/plain 400을 돌려준다
# (실제 관찰된 에러). 대신 uploadType=resumable을 8MB 청크로 PUT한다:
# 1) 크기 제한이 없고, 2) 청크마다 진행률(job)을 갱신해 %를 보여줄 수 있다.
UPLOAD_CHUNK_SIZE = 8 * 1024 * 1024  # 8MB


async def upload_file_resumable(file_path: str, filename: str, job: dict[str, Any] | None = None) -> dict[str, Any]:
  """디스크에 있는 파일(zip이든 원본 영상이든)을 Drive 전용 폴더에
  resumable 업로드로 전송. 파일 전체를 메모리에 올리지 않고 8MB씩
  읽어서 보낸다.
  """
  if not os.path.isfile(file_path):
    raise FileNotFoundError(file_path)
  size = os.path.getsize(file_path)
  set_job_message(job, "Google Drive 연결 확인 중...")
  try:
    async with aiohttp.ClientSession(timeout=_UPLOAD_TIMEOUT) as session:
      token = await _get_access_token(session)
      folder_id = await _ensure_folder(session, token)

      set_job_message(job, "업로드 세션 여는 중...")
      metadata = {"name": filename, "parents": [folder_id]}
      async with session.post(
        DRIVE_UPLOAD_URL,
        params={"uploadType": "resumable", "fields": "id,name,webViewLink,size"},
        headers={
          "Authorization": f"Bearer {token}",
          "Content-Type": "application/json; charset=UTF-8",
          "X-Upload-Content-Length": str(size),
        },
        json=metadata,
      ) as resp:
        if resp.status not in (200, 201):
          data = await _read_json_safe(resp)
          raise RuntimeError(data.get("error", {}).get("message", str(data)))
        session_uri = resp.headers.get("Location")
        if not session_uri:
          raise RuntimeError("업로드 세션 URI를 받지 못했습니다")

      _set_job_progress(job, 0, size, "업로드 중...")
      sent = 0
      with open(file_path, "rb") as fh:
        while sent < size:
          chunk = fh.read(UPLOAD_CHUNK_SIZE)
          if not chunk:
            break
          chunk_len = len(chunk)
          end = sent + chunk_len - 1
          async with session.put(
            session_uri,
            data=chunk,
            headers={
              "Content-Length": str(chunk_len),
              "Content-Range": f"bytes {sent}-{end}/{size}",
            },
          ) as resp:
            if resp.status in (200, 201):
              result = await _read_json_safe(resp)
              sent += chunk_len
              _set_job_progress(job, sent, size, "업로드 완료 처리 중...")
              return result
            if resp.status == 308:
              sent += chunk_len
              _set_job_progress(job, sent, size)
              continue
            text = (await resp.text())[:400]
            raise RuntimeError(f"업로드 청크 실패(HTTP {resp.status}): {text or '(empty body)'}")
      raise RuntimeError("업로드가 완료되지 않았습니다(응답 없음)")
  except (TimeoutError, aiohttp.ClientError) as e:
    msg = str(e) or type(e).__name__
    raise RuntimeError(f"Drive 업로드 실패(네트워크/타임아웃): {msg}") from None
