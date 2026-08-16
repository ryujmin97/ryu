"use strict";

// Logs page — Dashcam(주행 로그) + Screenrecord(화면 녹화) 탭.
// c3-ms의 기존 헬퍼(showAppToast, appConfirm, escapeHtml, getUIText 등)를
// 그대로 사용하며, c3-atune의 js/pages/logs/* 는 아키텍처가 달라 이식하지
// 않고 c3-ms 스타일로 새로 작성했습니다. 목적: 세그먼트(영상/rlog/qlog)와
// 화면 녹화 파일을 목록에서 선택해 다운로드.

const logsState = {
  dashcamLoaded: false,
  dashcamRoutes: [],
  dashcamExpanded: new Set(),
  dashcamSelected: new Set(), // segment 이름들
  screenrecordLoaded: false,
  screenrecordVideos: [],
  screenrecordSelected: new Set(), // file id 들
  bound: false,
  activeTab: "dashcam",
  gdrive: {
    connected: false,
    status: "disconnected",
    userCode: "",
    verificationUri: "",
    lastError: "",
    polling: null,
    clientId: "",
    clientSecret: "",
    deviceCode: "",
  },
};

function logsSegmentIndex(segment) {
  const parts = String(segment || "").split("--");
  const n = Number.parseInt(parts[parts.length - 1] || "0", 10);
  return Number.isFinite(n) ? n : 0;
}

function logsSetStatus(message, tone = "") {
  const el = document.getElementById("logsStatus");
  if (!el) return;
  el.textContent = message || "";
  el.hidden = !message;
  el.classList.toggle("is-error", tone === "error");
}

function logsSwitchTab(tab) {
  logsState.activeTab = tab;
  const tabDashcam = document.getElementById("logsTabDashcam");
  const tabScreenrecord = document.getElementById("logsTabScreenrecord");
  const panelDashcam = document.getElementById("logsPanelDashcam");
  const panelScreenrecord = document.getElementById("logsPanelScreenrecord");
  const isDashcam = tab === "dashcam";

  if (tabDashcam) {
    tabDashcam.classList.toggle("active", isDashcam);
    tabDashcam.setAttribute("aria-selected", isDashcam ? "true" : "false");
  }
  if (tabScreenrecord) {
    tabScreenrecord.classList.toggle("active", !isDashcam);
    tabScreenrecord.setAttribute("aria-selected", !isDashcam ? "true" : "false");
  }
  if (panelDashcam) panelDashcam.hidden = !isDashcam;
  if (panelScreenrecord) panelScreenrecord.hidden = isDashcam;

  if (isDashcam && !logsState.dashcamLoaded) loadDashcamRoutes();
  if (!isDashcam && !logsState.screenrecordLoaded) loadScreenrecordVideos();
}

/* ---------------------------------------------------------------------- */
/* 대시캠 (주행 세그먼트: qcamera 영상 + rlog/qlog)                        */
/* ---------------------------------------------------------------------- */

async function loadDashcamRoutes() {
  const meta = document.getElementById("dashcamMeta");
  if (meta) meta.textContent = "loading...";
  try {
    const r = await fetch("/api/dashcam/routes?limit=100");
    const j = await r.json();
    if (!j.ok) {
      if (meta) meta.textContent = "Failed: " + (j.error || "unknown");
      return;
    }
    logsState.dashcamRoutes = j.routes || [];
    logsState.dashcamLoaded = true;
    if (meta) {
      meta.textContent = `${j.total ?? logsState.dashcamRoutes.length}개 라우트`;
    }
    renderDashcamRoutes();
  } catch (e) {
    if (meta) meta.textContent = "Failed: " + (e?.message || e);
  }
}

function dashcamSelectedCountFor(route) {
  const entry = logsState.dashcamRoutes.find((r) => r.route === route);
  if (!entry) return 0;
  return (entry.segmentFolders || []).filter((s) => logsState.dashcamSelected.has(s)).length;
}

function renderDashcamRoutes() {
  const host = document.getElementById("dashcamRoutes");
  if (!host) return;
  if (!logsState.dashcamRoutes.length) {
    host.innerHTML = `<div class="muted logs-empty">주행 로그가 없습니다.</div>`;
    return;
  }

  host.innerHTML = logsState.dashcamRoutes.map((route) => {
    const expanded = logsState.dashcamExpanded.has(route.route);
    const segments = route.segmentFolders || [];
    const segmentDetails = route.segments || [];
    const selectedCount = dashcamSelectedCountFor(route.route);
    const title = escapeHtml((route.title || route.route || "").replace(/^0+(?=\d{3})/, ""));

    const segmentsHtml = expanded
      ? segmentDetails.map((seg) => {
          const segment = seg.segment;
          const checked = logsState.dashcamSelected.has(segment) ? "checked" : "";
          return `
            <div class="dashcam-segment" data-segment="${escapeHtml(segment)}">
              <label class="dashcam-segment__check">
                <input type="checkbox" class="logs-checkbox" data-segment="${escapeHtml(segment)}" ${checked} />
                <span>SEG ${seg.segmentIndex}</span>
              </label>
              <div class="dashcam-segment__name">
                <div>${escapeHtml(segment)}</div>
                <div class="muted dashcam-segment__date">${escapeHtml(seg.dateLabel || "")}</div>
              </div>
              <div class="dashcam-segment__actions">
                <a class="smallBtn" href="/api/dashcam/download/${encodeURIComponent(segment)}/qcamera" title="영상 다운로드">영상</a>
                <a class="smallBtn" href="/api/dashcam/download/${encodeURIComponent(segment)}/rlog" title="rlog 다운로드">rlog</a>
                <a class="smallBtn" href="/api/dashcam/download/${encodeURIComponent(segment)}/qlog" title="qlog 다운로드">qlog</a>
              </div>
            </div>`;
        }).join("")
      : "";

    return `
      <div class="dashcam-route" data-route="${escapeHtml(route.route)}">
        <button type="button" class="dashcam-route__head" data-toggle-route="${escapeHtml(route.route)}">
          <div class="dashcam-route__title">${title}</div>
          <div class="dashcam-route__meta muted">
            <span>세그먼트 ${route.segmentCount ?? segments.length}개</span>
            <span>${escapeHtml(route.dateLabel || "")}</span>
            <span>${escapeHtml(route.latestModifiedLabel || "")}</span>
            ${selectedCount ? `<span class="dashcam-route__selected">선택 ${selectedCount}개</span>` : ""}
          </div>
        </button>
        ${expanded ? `<div class="dashcam-route__body">${segmentsHtml}</div>` : ""}
      </div>`;
  }).join("");
}

function toggleDashcamRoute(route) {
  if (logsState.dashcamExpanded.has(route)) logsState.dashcamExpanded.delete(route);
  else logsState.dashcamExpanded.add(route);
  renderDashcamRoutes();
}

function toggleDashcamSegment(segment, checked) {
  if (checked) logsState.dashcamSelected.add(segment);
  else logsState.dashcamSelected.delete(segment);
}

function dashcamSelectAll() {
  const allSelected = logsState.dashcamRoutes.every((route) =>
    (route.segmentFolders || []).every((s) => logsState.dashcamSelected.has(s))
  );
  if (allSelected) {
    logsState.dashcamSelected.clear();
  } else {
    logsState.dashcamRoutes.forEach((route) => {
      (route.segmentFolders || []).forEach((s) => logsState.dashcamSelected.add(s));
    });
  }
  renderDashcamRoutes();
}

async function dashcamDownloadSelected() {
  const segments = Array.from(logsState.dashcamSelected);
  if (!segments.length) {
    showAppToast(getUIText("logs_select_empty", "선택된 항목이 없습니다"), { tone: "error" });
    return;
  }
  const ok = await appConfirm(
    `${segments.length}개 세그먼트를 zip으로 다운로드할까요?`,
    { title: getUIText("logs", "Logs") },
  );
  if (!ok) return;

  logsSetStatus("다운로드 준비 중...");
  try {
    const r = await fetch("/api/dashcam/download_zip", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ segments }),
    });
    if (!r.ok) throw new Error(`HTTP ${r.status}`);
    const blob = await r.blob();
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `dashcam_${Date.now()}.zip`;
    document.body.appendChild(a);
    a.click();
    a.remove();
    setTimeout(() => URL.revokeObjectURL(url), 4000);
    logsSetStatus("");
    showAppToast("다운로드를 시작했습니다");
  } catch (e) {
    logsSetStatus("다운로드 실패: " + (e?.message || e), "error");
  }
}

/* ---------------------------------------------------------------------- */
/* 화면 녹화 (Screenrecord)                                                */
/* ---------------------------------------------------------------------- */

async function loadScreenrecordVideos() {
  const meta = document.getElementById("screenrecordMeta");
  if (meta) meta.textContent = "loading...";
  try {
    const r = await fetch("/api/screenrecord/videos");
    const j = await r.json();
    if (!j.ok) {
      if (meta) meta.textContent = "Failed: " + (j.error || "unknown");
      return;
    }
    logsState.screenrecordVideos = j.videos || [];
    logsState.screenrecordLoaded = true;
    if (meta) meta.textContent = `${j.total ?? logsState.screenrecordVideos.length}개 파일`;
    renderScreenrecordList();
  } catch (e) {
    if (meta) meta.textContent = "Failed: " + (e?.message || e);
  }
}

function renderScreenrecordList() {
  const host = document.getElementById("screenrecordList");
  if (!host) return;
  if (!logsState.screenrecordVideos.length) {
    host.innerHTML = `<div class="muted logs-empty">화면 녹화 파일이 없습니다.</div>`;
    return;
  }
  host.innerHTML = logsState.screenrecordVideos.map((video) => {
    const checked = logsState.screenrecordSelected.has(video.id) ? "checked" : "";
    return `
      <div class="screenrecord-item" data-id="${escapeHtml(video.id)}">
        <label class="screenrecord-item__check">
          <input type="checkbox" class="logs-checkbox" data-screenrecord-id="${escapeHtml(video.id)}" ${checked} />
        </label>
        <div class="screenrecord-item__info">
          <div class="screenrecord-item__name">${escapeHtml(video.name)}</div>
          <div class="muted">${escapeHtml(video.sizeLabel || "")} · ${escapeHtml(video.modifiedLabel || "")} (${escapeHtml(video.relativeModifiedLabel || "")})</div>
        </div>
        <a class="smallBtn" href="/api/screenrecord/download/${encodeURIComponent(video.id)}">다운로드</a>
      </div>`;
  }).join("");
}

function toggleScreenrecordItem(id, checked) {
  if (checked) logsState.screenrecordSelected.add(id);
  else logsState.screenrecordSelected.delete(id);
}

function screenrecordSelectAll() {
  const allSelected = logsState.screenrecordVideos.every((v) => logsState.screenrecordSelected.has(v.id));
  if (allSelected) {
    logsState.screenrecordSelected.clear();
  } else {
    logsState.screenrecordVideos.forEach((v) => logsState.screenrecordSelected.add(v.id));
  }
  renderScreenrecordList();
}

async function screenrecordDownloadSelected() {
  const ids = Array.from(logsState.screenrecordSelected);
  if (!ids.length) {
    showAppToast(getUIText("logs_select_empty", "선택된 항목이 없습니다"), { tone: "error" });
    return;
  }
  const ok = await appConfirm(
    `${ids.length}개 파일을 다운로드할까요?`,
    { title: getUIText("logs", "Logs") },
  );
  if (!ok) return;

  // 화면 녹화는 개별 파일 다운로드 엔드포인트만 있으므로 순차적으로 트리거.
  for (const id of ids) {
    const a = document.createElement("a");
    a.href = `/api/screenrecord/download/${encodeURIComponent(id)}`;
    a.download = "";
    document.body.appendChild(a);
    a.click();
    a.remove();
    await new Promise((resolve) => setTimeout(resolve, 400));
  }
  showAppToast("다운로드를 시작했습니다");
}

/* ---------------------------------------------------------------------- */
/* 초기화 / 이벤트 바인딩                                                   */
/* ---------------------------------------------------------------------- */

function getGdriveStatusHTML() {
  if (logsState.gdrive.connected) {
    return "Google Drive: 연결됨";
  } else if (logsState.gdrive.status === "pending") {
    const href = logsState.gdrive.verificationUri || "#";
    const code = logsState.gdrive.userCode || "code";
    return `<a href="${href}" target="_blank" style="color: inherit; text-decoration: underline; cursor: pointer;">Google Drive</a>: 인증 대기 (${escapeHtml(code)}) <button id="btnCopyGdriveCode" class="smallBtn" type="button" title="코드 복사">복사</button>`;
  } else if (logsState.gdrive.status === "error") {
    const msg = logsState.gdrive.lastError ? `: ${escapeHtml(String(logsState.gdrive.lastError))}` : "";
    return `Google Drive: 인증 실패${msg}`;
  } else {
    return "Google Drive: 연결 안됨";
  }
}

function bindGdriveCopyButton() {
  const btn = document.getElementById("btnCopyGdriveCode");
  if (!btn) return;
  btn.onclick = async () => {
    const code = logsState.gdrive.userCode || "";
    if (!code) return;
    try {
      // 최신 방식: navigator.clipboard
      if (navigator.clipboard && navigator.clipboard.writeText) {
        await navigator.clipboard.writeText(code);
        showAppToast("코드가 복사되었습니다", { tone: "success" });
        return;
      }
      
      // 폴백: execCommand 방식
      const textarea = document.createElement("textarea");
      textarea.value = code;
      textarea.style.position = "fixed";
      textarea.style.left = "-9999px";
      document.body.appendChild(textarea);
      textarea.select();
      const success = document.execCommand("copy");
      document.body.removeChild(textarea);
      
      if (success) {
        showAppToast("코드가 복사되었습니다", { tone: "success" });
      } else {
        showAppToast("복사 실패: 지원되지 않는 환경", { tone: "error" });
      }
    } catch (e) {
      showAppToast("복사 실패: " + (e?.message || ""), { tone: "error" });
    }
  };
}

async function loadGdriveStatus() {
  try {
    const r = await fetch("/api/gdrive/status");
    const j = await r.json();
    if (!j || !j.ok) return;
    logsState.gdrive.connected = !!j.connected;
    logsState.gdrive.status = j.status || "disconnected";
    logsState.gdrive.userCode = j.user_code || "";
    logsState.gdrive.verificationUri = j.verification_uri || "";
    logsState.gdrive.lastError = j.last_error || "";
    const el = document.getElementById("logsGdriveStatus");
    if (el) {
      el.innerHTML = getGdriveStatusHTML();
      bindGdriveCopyButton();
    }
    // 페이지를 새로고침했는데 서버가 여전히 "인증 대기" 상태를 들고 있으면
    // (예: 폰에서 코드는 입력했지만 아직 브라우저를 안 닫음) 폴링을
    // 재개한다. device_code/client_id는 서버(GDRIVE_STATE)에 남아있는
    // 값을 그대로 쓰도록 비워서 보낸다 (api_gdrive_token이 폴백 처리).
    if (!logsState.gdrive.connected && logsState.gdrive.status === "pending" && !logsState.gdrive.polling) {
      startGdriveTokenPolling();
    }
  } catch (e) {
    // ignore
  }
}

function startGdriveTokenPolling() {
  // Google device authorization grant는 승인 여부를 서버가 능동적으로
  // "알려주지" 않는다. 클라이언트가 /api/gdrive/token(grant_type=
  // device_code)을 반복 호출해서 물어봐야 한다. 예전 코드는 여기서
  // /api/gdrive/status(로컬 상태 읽기)만 반복 호출했는데, 그 로컬 상태는
  // 이 token 폴링이 실제로 실행돼야만 바뀌므로 절대 "연결됨"으로 전환되지
  // 않는 버그가 있었다 — 방금 Google 쪽에서 승인해도 carrotweb은 절대
  // 알 수 없는 구조였다.
  if (logsState.gdrive.polling) clearInterval(logsState.gdrive.polling);
  logsState.gdrive.polling = setInterval(async () => {
    let sj;
    try {
      const sr = await fetch("/api/gdrive/token", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          client_id: logsState.gdrive.clientId,
          client_secret: logsState.gdrive.clientSecret,
          device_code: logsState.gdrive.deviceCode,
        }),
      });
      sj = await sr.json();
    } catch (e) {
      return; // 일시적 네트워크 오류는 다음 폴링에서 재시도
    }
    if (sj && sj.connected) {
      logsState.gdrive.connected = true;
      logsState.gdrive.status = "connected";
      showAppToast("Google Drive 연결 완료", { tone: "success" });
      if (logsState.gdrive.polling) clearInterval(logsState.gdrive.polling);
      logsState.gdrive.polling = null;
    } else if (sj && sj.pending) {
      // 아직 사용자가 Google 화면에서 승인하지 않음 — 정상, 계속 폴링
      logsState.gdrive.status = "pending";
    } else if (sj && !sj.ok) {
      logsState.gdrive.status = "error";
      logsState.gdrive.lastError = (sj.error && (sj.error.error_description || sj.error.error)) || sj.error || "error";
      showAppToast(`Drive 인증 실패: ${logsState.gdrive.lastError}`, { tone: "error" });
      if (logsState.gdrive.polling) clearInterval(logsState.gdrive.polling);
      logsState.gdrive.polling = null;
    }
    const s = document.getElementById("logsGdriveStatus");
    if (s) {
      s.innerHTML = getGdriveStatusHTML();
      bindGdriveCopyButton();
    }
  }, 5000);
}

async function triggerGdriveAuth() {
  const clientId = document.getElementById("gdriveClientId")?.value?.trim();
  const clientSecret = document.getElementById("gdriveClientSecret")?.value?.trim();
  if (!clientId) {
    showAppToast("Client ID를 입력하세요", { tone: "error" });
    return;
  }
  try {
    const r = await fetch("/api/gdrive/device", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ client_id: clientId, client_secret: clientSecret }),
    });
    const j = await r.json();
    if (!j || !j.ok) {
      showAppToast(j?.error || "인증 요청 실패", { tone: "error" });
      return;
    }
    logsState.gdrive.clientId = clientId;
    logsState.gdrive.clientSecret = clientSecret;
    logsState.gdrive.deviceCode = j.device_code || "";
    logsState.gdrive.userCode = j.user_code || "";
    logsState.gdrive.verificationUri = j.verification_uri || "";
    logsState.gdrive.status = "pending";
    const el = document.getElementById("logsGdriveStatus");
    if (el) {
      el.innerHTML = getGdriveStatusHTML();
      bindGdriveCopyButton();
    }
    startGdriveTokenPolling();
    showAppToast("브라우저에서 인증 코드를 입력해 주세요", { tone: "success" });
  } catch (e) {
    showAppToast(e?.message || "인증 실패", { tone: "error" });
  }
}

async function pollGdriveUploadJob(jobId, itemLabel) {
  // core.py의 tools job 폴링과 같은 패턴: 완료될 때까지 짧은 간격으로 조회.
  while (true) {
    const r = await fetch(`/api/gdrive/job?id=${encodeURIComponent(jobId)}`);
    const snap = await r.json();
    if (!snap || !snap.ok) throw new Error((snap && snap.error) || "job not found");
    if (!snap.done) {
      const pct = (snap.total && typeof snap.percent === "number") ? ` ${snap.percent}%` : "";
      logsSetStatus(`${itemLabel} 업로드 중... ${snap.message || ""}${pct}`.trim());
      await new Promise((resolve) => setTimeout(resolve, 500));
      continue;
    }
    return snap;
  }
}

async function uploadSelectedFiles(kind) {
  if (!logsState.gdrive.connected) {
    showAppToast("Google Drive 연결이 필요합니다", { tone: "error" });
    return;
  }
  const payloads = kind === "dashcam" ? Array.from(logsState.dashcamSelected) : Array.from(logsState.screenrecordSelected);
  if (!payloads.length) {
    showAppToast("업로드할 파일을 선택하세요", { tone: "error" });
    return;
  }

  for (const item of payloads) {
    const body = kind === "dashcam" ? { kind: "dashcam", segment: item } : { kind: "screenrecord", file_id: item };
    logsSetStatus(`${item} 업로드 준비 중...`);
    try {
      const r = await fetch("/api/gdrive/upload", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      });
      const j = await r.json();
      if (!j || !j.ok) throw new Error(j?.error || "업로드 실패");
      const snap = await pollGdriveUploadJob(j.job_id, item);
      if (snap.status !== "done") throw new Error(snap.error || "업로드 실패");
      showAppToast(`${item} 업로드 완료`, { tone: "success" });
    } catch (e) {
      showAppToast(`${item} 업로드 실패: ${e?.message || e}`, { tone: "error" });
      break;
    }
  }
  logsSetStatus("");
}

function bindLogsEvents() {
  if (logsState.bound) return;
  logsState.bound = true;

  const tabDashcam = document.getElementById("logsTabDashcam");
  const tabScreenrecord = document.getElementById("logsTabScreenrecord");
  if (tabDashcam) tabDashcam.onclick = () => logsSwitchTab("dashcam");
  if (tabScreenrecord) tabScreenrecord.onclick = () => logsSwitchTab("screenrecord");

  const btnDashcamSelectAll = document.getElementById("btnDashcamSelectAll");
  if (btnDashcamSelectAll) btnDashcamSelectAll.onclick = () => dashcamSelectAll();

  const btnDashcamDownloadSelected = document.getElementById("btnDashcamDownloadSelected");
  if (btnDashcamDownloadSelected) btnDashcamDownloadSelected.onclick = () => dashcamDownloadSelected();

  const btnDashcamUploadSelected = document.getElementById("btnDashcamUploadSelected");
  if (btnDashcamUploadSelected) btnDashcamUploadSelected.onclick = () => uploadSelectedFiles("dashcam");

  const btnScreenrecordSelectAll = document.getElementById("btnScreenrecordSelectAll");
  if (btnScreenrecordSelectAll) btnScreenrecordSelectAll.onclick = () => screenrecordSelectAll();

  const btnScreenrecordDownloadSelected = document.getElementById("btnScreenrecordDownloadSelected");
  if (btnScreenrecordDownloadSelected) btnScreenrecordDownloadSelected.onclick = () => screenrecordDownloadSelected();

  const btnScreenrecordUploadSelected = document.getElementById("btnScreenrecordUploadSelected");
  if (btnScreenrecordUploadSelected) btnScreenrecordUploadSelected.onclick = () => uploadSelectedFiles("screenrecord");

  const btnGdriveAuth = document.getElementById("btnGdriveAuth");
  if (btnGdriveAuth) btnGdriveAuth.onclick = triggerGdriveAuth;

  const dashcamRoutesHost = document.getElementById("dashcamRoutes");
  if (dashcamRoutesHost) {
    dashcamRoutesHost.addEventListener("click", (ev) => {
      const toggleBtn = ev.target.closest("[data-toggle-route]");
      if (toggleBtn) {
        toggleDashcamRoute(toggleBtn.getAttribute("data-toggle-route"));
      }
    });
    dashcamRoutesHost.addEventListener("change", (ev) => {
      const checkbox = ev.target.closest("input[data-segment]");
      if (checkbox) {
        toggleDashcamSegment(checkbox.getAttribute("data-segment"), checkbox.checked);
        renderDashcamRoutes();
      }
    });
  }

  const screenrecordListHost = document.getElementById("screenrecordList");
  if (screenrecordListHost) {
    screenrecordListHost.addEventListener("change", (ev) => {
      const checkbox = ev.target.closest("input[data-screenrecord-id]");
      if (checkbox) {
        toggleScreenrecordItem(checkbox.getAttribute("data-screenrecord-id"), checkbox.checked);
      }
    });
  }
}

function initLogsPage() {
  bindLogsEvents();
  loadGdriveStatus();
  logsSwitchTab(logsState.activeTab || "dashcam");
}
