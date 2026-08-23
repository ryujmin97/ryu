#!/usr/bin/env python3
import math
import numpy as np
from collections import deque
from typing import Any
import heapq
import copy

import capnp
from cereal import messaging, log, car
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.common.simple_kalman import KF1D


# Default lead acceleration decay set to 50% at 1s
_LEAD_ACCEL_TAU = 1.5

# radar tracks
SPEED, ACCEL = 0, 1     # Kalman filter states enum

# stationary qualification parameters
V_EGO_STATIONARY = 4.   # no stationary object flag below this speed

RADAR_TO_CENTER = 2.7   # (deprecated) RADAR is ~ 2.7m ahead from center of car
RADAR_TO_CAMERA = 1.52  # RADAR is ~ 1.5m ahead from center of mesh frame

# --- LeadBlend safety tuning ---
# EnableRadarTracks < 3 cars have a working single-point SCC radar for the
# in-lane lead (it's the primary source most of the time -- ~74-82% of tracked
# time in real drive logs), but no multi-track/corner radar for adjacent lanes.
# When the SCC radar briefly loses lock (close range, blind spot, transitions)
# leadOne temporarily falls back to the vision model, and that fallback window
# is where track switches and brief misses actually happen (~80-90% of
# lost-lead events despite being <30% of tracked time). This layer debounces
# that noise WITHOUT ever delaying a genuinely dangerous change.
LEAD_BLEND_TTC_DANGER    = 2.5   # s   : TTC below this => treat as dangerous, apply immediately
LEAD_BLEND_DANGER_HOLD   = 0.3   # s   : once flagged dangerous, keep bypassing smoothing this long
LEAD_BLEND_SAFE_DIST_TIME = 0.35 # s   : time constant to blend dRel/vRel toward a safe-direction switch
LEAD_LOST_GRACE_TIME     = 0.6   # s   : hold last known lead through a brief vision miss (debounce)
CUTOUT_DPATH_THRESH      = 2.0   # m   : |dPath| beyond this = lead has clearly left our path (cut-out)
CUTOUT_VREL_GATE         = -0.5  # m/s : only treat a miss as a cut-out if lead wasn't strongly closing
# 2026-08-16 실주행 로그(총 67분) 분석 결과 추가된 게이트:
LEAD_BLEND_CLOSER_JUMP_DIST = 8.0  # m : 새 dRel이 이전보다 이만큼 더 가까우면, vRel이 잠잠해 보여도
                                    #     위험으로 간주하고 즉시 반영. SCC가 근접구간/사각지대에서
                                    #     순간적으로 락을 놓치면서 직전의 오래된(먼 거리) 값을 잠깐
                                    #     들고 있다가, 비전으로 넘어가는 순간 정확한 근거리 값으로
                                    #     튀는 패턴 대응 (route1 seg13 t=794s 실측)
LEAD_BLEND_BIG_JUMP_DIST    = 15.0 # m : 이보다 큰 '안전 방향' 점프는 노이즈가 아니라 다른 물체로
                                    #     대상이 바뀐 것으로 보고, 블렌딩하지 않고 즉시 스냅
                                    #     (블렌딩 시 실제로 없는 가짜 상대속도가 생겨 MPC를 오도할
                                    #     수 있음. 정체 구간 비전 트랙 흔들림에서 실측: route1
                                    #     t=1388~1390s / route2 t=825~827s)

# --- SCC 단일점 폴백 안전 게이트 (37차) ---
# get_lead()에서 비전 매칭 실패/저확신(prob<.6) 시 track_scc(단일점 SCC
# 레이더, trackId=0)를 차로내 위치 검증 없이 그대로 채택하던 문제 대응.
# [NEEDS_VALIDATION] 실차 로그 4건(옆차선 3건 dPath 미실측 -- 당시는 yRel
# -5.5~-10.5m로 게이트 자체가 불필요할 만큼 명백; 저속 도심 커브 1건은
# yRel -1.4~-1.5m로 단순 yRel 게이트로는 못 거를 가능성 있었음)을 근거로
# dPath(차선 중심 대비, 곡률/차선폭 보정 포함) 기준으로 설계 -- 이 상수
# 자체의 실차 재검증은 아직 없음. CUTOUT_DPATH_THRESH(2.0)와 동일 철학:
# "이 값을 넘으면 이미 차로를 벗어난 것으로 간주"를 폴백 채택 시점에도
# 선제 적용.
SCC_FALLBACK_DPATH_GATE  = 2.0  # m : track_scc.dPath가 이보다 크면(차로 밖으로
                                 #     판단) 폴백으로 채택하지 않음


def laplacian_pdf(x: float, mu: float, b: float):
  diff = abs(x - mu) / max(b, 1e-4)
  return 0.0 if diff > 50.0 else math.exp(-diff)

def clamp(x: float, lo: float, hi: float) -> float:
  return float(np.clip(x, lo, hi))


class Track:
  def __init__(self, identifier: int):
    self.identifier = identifier
    self.cnt = 0
    self.aLeadTau = FirstOrderFilter(_LEAD_ACCEL_TAU, 0.45, DT_MDL)

    self.is_stopped_car_count = 0
    self.selected_count = 0
    self.cut_in_count = 0
    self.measured = False
    self.score = 0.0
    self.in_lane_prob = 0.0
    self.in_lane_prob_future = 0.0

    self.dPath = 0.0

    # ---- noise filter state (new) ----
    self._vLead_last = 0.0
    self._vLead_filt = 0.0
    self._vLead_filt_init = False

  def update(self, md, pt, ready, radar_reaction_factor, radar_lat_factor):
    self.dRel = pt.dRel
    self.yRel = pt.yRel
    self.vRel = pt.vRel

    self.vLead = self.vLeadK = pt.vLead
    self.aLead = self.aLeadK = pt.aLead
    self.jLead = pt.jLead
    self.yvLead = pt.yvRel

    self.measured = pt.measured
    if not self.measured:
      self.cnt = 0
      # optional: also reset filter init when track is not measured
      self._vLead_filt_init = False

    self.yRel_future = self.yRel + self.yvLead * radar_lat_factor
    self.dRel_future = self.dRel + self.vLead * radar_lat_factor
    if ready:
      self.d_path(md)

    a_lead_threshold = 0.5 * radar_reaction_factor
    if abs(self.aLead) < a_lead_threshold and abs(self.jLead) < 0.5:
      self.aLeadTau.x = _LEAD_ACCEL_TAU * radar_reaction_factor
    else:
      self.aLeadTau.update(0.0)

    self.cnt += 1

  def d_path(self, md):
    lane_xs = md.laneLines[1].x
    left_ys = md.laneLines[1].y
    right_ys = md.laneLines[2].y

    def d_path_interp(dRel, yRel):
      left_lane_y = np.interp(dRel, lane_xs, left_ys)
      right_lane_y = np.interp(dRel, lane_xs, right_ys)
      center_y = (left_lane_y + right_lane_y) / 2.0
      lane_half_width = max(0.1, abs(right_lane_y - left_lane_y) / 2.0)
      dist_from_center = yRel + center_y
      in_lane_prob = max(0.0, 1.0 - (abs(dist_from_center) / lane_half_width))
      return dist_from_center, in_lane_prob

    self.dPath, self.in_lane_prob = d_path_interp(self.dRel, self.yRel)
    self.dPath_future, self.in_lane_prob_future = d_path_interp(self.dRel_future, self.yRel_future)

  # ---- noise suppression only when cnt>=2 ----
  def vlead_for_matching(self, dv_max: float = 4.0, alpha: float = 0.35) -> float:
    """
    Returns vLead to be used in matching score.
    - If cnt < 2: raw vLead (no filtering)
    - If cnt >= 2: clamp spike + IIR smooth
    """
    v = float(self.vLead)

    if self.cnt < 2:
      return v

    if not self._vLead_filt_init:
      self._vLead_last = v
      self._vLead_filt = v
      self._vLead_filt_init = True
      return v

    v_last = self._vLead_last
    self._vLead_last = v

    v_clamped = clamp(v, v_last - dv_max, v_last + dv_max)
    self._vLead_filt = alpha * v_clamped + (1.0 - alpha) * self._vLead_filt
    return float(self._vLead_filt)

  def get_RadarState(self, model_prob: float = 0.0, vision_y_rel=0.0):
    return {
      "dRel": float(self.dRel),
      "yRel": float(self.yRel) if self.yRel != 0.0 else vision_y_rel,
      "dPath": float(self.dPath),
      "vRel": float(self.vRel),
      "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK),
      "aLead": float(self.aLead),
      "aLeadK": float(self.aLeadK),
      "aLeadTau": float(self.aLeadTau.x),
      "jLead": float(self.jLead),
      "vLat": float(self.yvLead),
      "status": True,
      "fcw": self.is_potential_fcw(model_prob),
      "modelProb": model_prob,
      "radar": True,
      "radarTrackId": self.identifier,
      "score": self.score,
    }

  def potential_low_speed_lead(self, v_ego: float):
    return abs(self.yRel) < 1.0 and (v_ego < V_EGO_STATIONARY) and (0.75 < self.dRel < 25)

  def is_potential_fcw(self, model_prob: float):
    return model_prob > .9

  def __str__(self):
    return f"x: {self.dRel:4.1f}  y: {self.yRel:4.1f}  v: {self.vRel:4.1f}  a: {self.aLeadK:4.1f}"


def match_vision_to_track(v_ego: float, lead: capnp._DynamicStructReader, tracks: dict[int, Track]):
  if not tracks:
    return None

  offset_vision_dist = float(lead.x[0] - RADAR_TO_CAMERA)

  # distance gates
  max_vision_dist  = max(offset_vision_dist * 1.25, 5.0)
  min_vision_dist  = max(offset_vision_dist * 0.80, 1.0)
  max_vision_dist2 = max(offset_vision_dist * 1.45, 5.0)
  min_vision_dist2 = 1.5

  # velocity tolerance (same intent)
  vel_tol = float(max(lead.v[0] * np.interp(lead.prob, [0.8, 0.98], [0.3, 0.5]), 5.0))
  # hard guardrail for moving-bias (prevents absurd match)
  vel_guard = max(vel_tol * 3.0, 20.0)

  def dist_sane(t: Track, wide: bool = False) -> bool:
    if wide:
      return (min_vision_dist2 < t.dRel < max_vision_dist2)
    return (min_vision_dist < t.dRel < max_vision_dist)

  def y_sane(t: Track, wide: bool = False) -> bool:
    lim = 4.0 if wide else 2.0
    return abs(t.yRel + float(lead.y[0])) < lim

  def vel_sane(t: Track) -> bool:
    """
    Keep your philosophy:
      - if it's moving, likely "the car we should read"
    but add guardrail and (optionally) in-lane preference.
    """
    v_vis = float(lead.v[0])
    v_trk = float(t.vLead)
    dv = abs(v_trk - v_vis)

    # normal strict check
    if dv < vel_tol:
      return True

    # moving-bias: allow more mismatch for moving objects,
    # but only within a reasonable guardrail.
    moving = (v_trk > 3.0)
    if not moving:
      return False

    if dv > vel_guard:
      return False

    # If in-lane probability exists (it does in your Track), use it as safety.
    # When it's clearly not in our lane, don't use moving-bias.
    # (This line is intentionally mild; you can tune 0.2~0.5)
    if hasattr(t, "dPath") and (t.in_lane_prob < 0.25):
      return False

    return True

  def score_pair(t: Track):
    """
    score1: normal yStd
    score2: wide yStd for cut-in
    NOTE: uses t.vlead_for_matching() only for scoring (cnt>=2 only).
    """
    pd = laplacian_pdf(float(t.dRel), offset_vision_dist, float(lead.xStd[0]))
    py = laplacian_pdf(float(t.yRel), -float(lead.y[0]), float(lead.yStd[0]))
    py2 = laplacian_pdf(float(t.yRel), -float(lead.y[0]), float(lead.yStd[0]) * 2.0)

    v_use = float(t.vlead_for_matching())  # noise suppression only if cnt>=2
    pv = laplacian_pdf(v_use, float(lead.v[0]), float(lead.vStd[0]))

    s1 = pd * py * pv
    s2 = pd * py2 * pv
    return s1, s2

  # ---- pick best candidates (FIX: true 1st/2nd) ----
  first_track, second_track, extra_track = None, None, None
  first_score, second_score, extra_score = -1e18, -1e18, -1e18

  for t in tracks.values():
    s1, s2 = score_pair(t)
    t.score = s1

    if s1 > first_score:
      second_track, second_score = first_track, first_score
      first_track, first_score = t, s1
    elif s1 > second_score:
      second_track, second_score = t, s1

    if s2 > extra_score:
      extra_track, extra_score = t, s2

  # score floor
  if first_track is None or first_score < 1e-4:
    return None

  # ---- selection policy (same logic, cleaner & safer) ----
  best_track = None

  # A) normal match
  if dist_sane(first_track) and vel_sane(first_track):
    if y_sane(first_track):
      if lead.prob > 0.5:
        best_track = first_track
      elif lead.prob > 0.4 and first_track.selected_count > 0:
        best_track = first_track
    elif lead.prob > 0.6:
      best_track = first_track

  # B) stopped-car-like (only if not chosen yet)
  if best_track is None and dist_sane(first_track) and y_sane(first_track, wide=True):
    if (second_track is not None and second_score > 1e-5 and
        dist_sane(second_track) and y_sane(second_track) and vel_sane(second_track)):
      best_track = second_track
    elif first_track.selected_count > 0:
      best_track = first_track
    else:
      first_track.is_stopped_car_count += 2
      if first_track.is_stopped_car_count > int(1.0 / DT_MDL):
        best_track = first_track

  # C) cut-in wide matching (only if not chosen yet)
  if best_track is None and offset_vision_dist < 90.0 and lead.prob > 0.65:
    # wide-y winner first (cut-in)
    if (extra_track is not None and extra_score > first_score and
        dist_sane(extra_track, wide=True) and vel_sane(extra_track) and y_sane(extra_track, wide=True)):
      best_track = extra_track

    # then allow first/second with wide gates
    elif dist_sane(first_track, wide=True) and vel_sane(first_track) and y_sane(first_track, wide=True):
      best_track = first_track

    elif (second_track is not None and second_score > 1e-4 and
          dist_sane(second_track, wide=True) and vel_sane(second_track) and y_sane(second_track, wide=True)):
      best_track = second_track

  # ---- update counters ----
  for t in tracks.values():
    if t is best_track and best_track is not None:
      t.selected_count += 1
    else:
      t.selected_count = 0
      t.is_stopped_car_count = max(0, t.is_stopped_car_count - 1)

  return best_track


def get_RadarState_from_vision(md, lead_msg: capnp._DynamicStructReader, v_ego: float, model_v_ego: float):
  lead_v_rel_pred = lead_msg.v[0] - model_v_ego
  dRel = float(lead_msg.x[0] - RADAR_TO_CAMERA)
  yRel = float(-lead_msg.y[0])
  dPath = yRel + np.interp(dRel, md.position.x, md.position.y)
  return {
    "dRel": float(dRel),
    "yRel": yRel,
    "dPath" : float(dPath),
    "vRel": float(lead_v_rel_pred),
    "vLead": float(v_ego + lead_v_rel_pred),
    "vLeadK": float(v_ego + lead_v_rel_pred),
    "aLead": float(lead_msg.a[0]),
    "aLeadK": float(lead_msg.a[0]),
    "aLeadTau": 0.3,
    "jLead": 0.0,
    "vLat" : 0.0,
    "fcw": False,
    "modelProb": float(lead_msg.prob),
    "status": True,
    "radar": False,
    "radarTrackId": -1,
  }

# --- VisionTrack measured-derivative gate tuning (58차 1번) ---
# VisionTrack.update()의 vRel 추정은 두 경로 중 하나를 씀:
#   (a) 단일 프레임 모델 예측(lead_v_rel_pred) -- 노이즈에 약하고 원거리에서
#       접근율을 과소평가하는 경향(userMemories/FINDINGS 다수 세션에서 확인된
#       "카메라 인식 시 미감속"의 root cause)
#   (b) 실측 dRel 미분값(v_rel = (dRel-dRel_last)/dt, alpha 저역통과) -- 레이더가
#       쓰는 것과 동일한 방식, 훨씬 정확하지만 dRel 자체가 튀면 노이즈에 노출됨
# 기존 게이트(prob<0.97 또는 cnt<20이면 (a)만 사용)가 문제였음: 실제 주행
# 로그에서 원거리 vision lead의 prob는 0.5~0.8대가 흔하고 0.97을 넘는 경우가
# 드물어 (b) 경로가 사실상 죽어있었음(56차/57차 qcamera 대조로 반복 확인된
# "vision 미감속" 패턴의 근본원인). 게이트를 완화해 (b)를 훨씬 자주 타게 하고,
# model_weight 보간 구간도 낮춘 게이트값에 맞춰 재설계 -- prob가 게이트값
# 근처일 땐 아직 모델쪽 비중을 높게(안전측) 유지하고 prob==1.0에 가까워질수록
# 실측값 비중을 거의 100%로 올리는 점진적 전환은 유지한다(급전환 방지).
VISION_TRACK_PROB_GATE = 0.70   # 기존 0.97 -- 실측 원거리 prob 분포에 맞춤
VISION_TRACK_CNT_GATE  = 10     # 기존 20(=1.0s) -- 0.5s(20Hz 기준)로 단축

# 58차 3번 (A): 정식 등록 문턱(prob>.5)을 못 넘어도, prob가 "애매한 구간"
# (TENTATIVE_PROB_GATE~정식문턱)에서 같은 위치(dRel)로 여러 프레임 연속
# 잡히면(=노이즈성 순간 오탐지가 아니라 진짜 물체) 조기 등록을 허용한다.
# "정지차량_미인식" 실사례(8초간 화면에 명백히 보이는데 prob<0.5라 트랙
# 자체가 안 생겼던 문제)의 근본 대응. dRel 튐 감시로 다른 물체로의 오인
# 승격은 차단.
VISION_TRACK_TENTATIVE_PROB_GATE = 0.35    # 이 이상이면 tentative 카운트 시작
VISION_TRACK_TENTATIVE_CNT_GATE  = 10      # 0.5s(20Hz) 연속 유지 시 정식 등록으로 승격
VISION_TRACK_TENTATIVE_DREL_JITTER = 8.0   # tentative 추적 중 dRel이 이 이상 튀면 다른 물체로 판단, 리셋

# 58차 3번 (B): prob<VISION_TRACK_PROB_GATE라 아직 모델예측(lead_v_rel_pred)만
# 쓰는 구간에서도, dRel 실측 이력이 최소 SAFETY_MIN_CNT프레임 쌓였으면 그
# 실측 기반 속도가 모델예측보다 "더 위험"(접근이 더 빠름=vLead가 더 작음)할
# 때만 min()으로 안전측 보정한다. 모델이 정상(과소평가 아님)일 땐 절대 개입
# 안 함 -- 58차1번에서 이미 검증된 "min() 안전클램프만, 완화 방향 없음" 원칙
# 그대로 재사용.
VISION_TRACK_SAFETY_MIN_CNT = 2

class VisionTrack:
  def __init__(self, radar_ts):
    self.radar_ts = radar_ts
    self.dRel = 0.0
    self.vRel = 0.0
    self.yRel = 0.0
    self.vLead = 0.0
    self.aLead = 0.0
    self.vLeadK = 0.0
    self.aLeadK = 0.0
    self.aLeadTau = _LEAD_ACCEL_TAU
    self.prob = 0.0
    self.status = False

    self.dRel_last = 0.0
    self.vLead_last = 0.0
    self.alpha = 0.02
    self.alpha_a = 0.02

    self.vLat = 0.0

    self.v_ego = 0.0
    self.cnt = 0

    self.dPath = 0.0

    # 58차 3번 (A): tentative(예비) 등록 추적용
    self.tentative_cnt = 0
    self.tentative_dRel_last = 0.0

  def get_lead(self, md):
    #aLeadK = 0.0 if self.mixRadarInfo in [3] else clip(self.aLeadK, self.aLead - 1.0, self.aLead + 1.0)
    return {
      "dRel": self.dRel,
      "yRel": self.yRel,
      "dPath": float(self.dPath),  # needed by LeadBlend cut-out detection (was disabled)
      "vRel": self.vRel,
      "vLead": self.vLead,
      "vLeadK": self.vLeadK,    ## TODO: 아직 vLeadK는 엉망인듯...
      "aLead": self.aLead,
      "aLeadK": self.aLeadK,
      "aLeadTau": self.aLeadTau,
      "jLead": 0.0,
      "vLat": 0.0,
      "fcw": False,
      "modelProb": self.prob,
      "status": self.status,
      "radar": False,
      "radarTrackId": -1,
      #"aLead": self.aLead,
      #"vLat": self.vLat,
    }

  def reset(self):
    self.status = False
    self.aLeadTau = _LEAD_ACCEL_TAU

    self.vRel = 0.0
    self.vLead = self.vLeadK = self.v_ego
    self.aLead = self.aLeadK = 0.0
    self.vLat = 0.0

  def update(self, lead_msg, model_v_ego, v_ego, md):

    lead_v_rel_pred = lead_msg.v[0] - model_v_ego
    self.prob = lead_msg.prob
    self.v_ego = v_ego

    # 58차 3번 (A): 정식 등록 문턱(prob>.5) 못 넘는 "애매한" prob에서도,
    # 같은 위치의 물체가 여러 프레임 연속 잡히면 tentative_cnt를 쌓아
    # 문턱 전에 조기 등록을 허용한다. 다른 물체로 튀면(jitter 초과)
    # tentative_cnt를 리셋해 오인 승격을 막는다.
    dRel_candidate = float(lead_msg.x[0]) - RADAR_TO_CAMERA
    if VISION_TRACK_TENTATIVE_PROB_GATE <= self.prob <= 0.5:
      if self.tentative_cnt > 0 and abs(dRel_candidate - self.tentative_dRel_last) > VISION_TRACK_TENTATIVE_DREL_JITTER:
        self.tentative_cnt = 0
      self.tentative_cnt += 1
      self.tentative_dRel_last = dRel_candidate
    elif self.prob < VISION_TRACK_TENTATIVE_PROB_GATE:
      self.tentative_cnt = 0

    register_ok = (self.prob > .5) or (self.tentative_cnt >= VISION_TRACK_TENTATIVE_CNT_GATE)

    if register_ok:
      dRel = dRel_candidate
      if abs(self.dRel - dRel) > 5.0:
        self.cnt = 0
      self.dRel = dRel

      self.yRel = float(-lead_msg.y[0])
      dPath = self.yRel + np.interp(self.dRel, md.position.x, md.position.y)
      a_lead_vision = lead_msg.a[0]
      # 58차 1번: 게이트를 0.97/20 -> VISION_TRACK_PROB_GATE/VISION_TRACK_CNT_GATE로 완화.
      # (레이더측정시 cnt는 0, 레이더사라지고 cnt<GATE인 동안엔 비젼데이터 그대로 사용)
      if self.cnt < VISION_TRACK_CNT_GATE or self.prob < VISION_TRACK_PROB_GATE:
        self.vRel = lead_v_rel_pred
        self.vLead = float(v_ego + lead_v_rel_pred)
        self.aLead = a_lead_vision
        self.vLat = 0.0
        # 58차 3번 (B): 아직 모델예측만 쓰는 구간이라도, dRel 실측 이력이
        # 충분히 쌓였으면(SAFETY_MIN_CNT+) 실측 기반 속도가 모델예측보다
        # 더 위험(접근이 더 빠름)할 때만 min()으로 안전측 보정한다.
        # 모델이 실제로 맞을 땐(과소평가 아닐 땐) 전혀 개입하지 않음 --
        # 58차1번의 v_lead 안전클램프와 동일 원칙, 완화 방향 없음.
        if self.cnt >= VISION_TRACK_SAFETY_MIN_CNT and self.dRel_last > 0.0 and self.radar_ts > 0:
          v_rel_measured = (self.dRel - self.dRel_last) / self.radar_ts
          vLead_measured = float(v_ego + v_rel_measured)
          if vLead_measured < self.vLead:
            self.vLead = vLead_measured
            self.vRel = v_rel_measured
      else:
        v_rel = (self.dRel - self.dRel_last) / self.radar_ts
        v_rel = self.vRel * (1. - self.alpha) + v_rel * self.alpha

        #self.vRel = lead_v_rel_pred if self.mixRadarInfo == 3 else (lead_v_rel_pred + self.vRel) / 2
        # prob==GATE 근처(막 진입)일 땐 아직 모델 비중 높게(0.5, 안전측), prob->1.0으로
        # 갈수록 실측 dRel미분(v_rel)에 거의 전적으로 의존(0.0)하도록 점진 전환.
        model_weight = np.interp(self.prob, [VISION_TRACK_PROB_GATE, 1.0], [0.5, 0.0])  # prob가 높으면 v_rel(dRel미분값)에 가중치를 줌.
        self.vRel = float(lead_v_rel_pred * model_weight + v_rel * (1. - model_weight))
        #self.vRel = (lead_v_rel_pred + v_rel) / 2
        self.vLead = float(v_ego + self.vRel)

        a_lead = (self.vLead - self.vLead_last) / self.radar_ts * 0.2 #0.5 -> 0.2 vel 미분적용을 줄임.
        self.aLead = self.aLead * (1. - self.alpha_a) + a_lead * self.alpha_a
        if abs(a_lead_vision) > abs(self.aLead): # or self.mixRadarInfo == 3:
          self.aLead = a_lead_vision

        vLat_alpha = 0.002
        self.vLat = self.vLat * (1. - vLat_alpha) + (dPath - self.dPath) / self.radar_ts * vLat_alpha

      self.dPath = dPath

      self.vLeadK= self.vLead
      self.aLeadK = self.aLead

      self.status = True
      self.cnt += 1
    else:
      self.reset()
      self.cnt = 0
      self.dPath = self.yRel + np.interp(v_ego ** 2 / (2 * 2.5), md.position.x, md.position.y)

    self.dRel_last = self.dRel
    self.vLead_last = self.vLead

    # Learn if constant acceleration
    #aLeadTauValue = self.aLeadTauPos if self.aLead > self.aLeadTauThreshold else self.aLeadTauNeg
    if abs(self.aLead) < 0.3: #self.aLeadTauThreshold:
      self.aLeadTau = 0.2 #aLeadTauValue
    else:
      #self.aLeadTau = min(self.aLeadTau * 0.9, aLeadTauValue)
      self.aLeadTau *= 0.9

class LeadBlend:
  """
  Debounces leadOne track-switch/miss noise, but never at the cost of reaction
  time on a genuinely dangerous change.

  - Lost-lead debounce: a lead that disappears for < LEAD_LOST_GRACE_TIME is
    held (extrapolated) instead of instantly reported as gone, so a single
    missed vision frame doesn't cause a follow-distance jerk.
  - Cut-out bypass: if the last known lead was clearly leaving our path
    (|dPath| > CUTOUT_DPATH_THRESH) and wasn't strongly closing, that's a real
    cut-out, not a vision blip -- skip the grace hold and report it gone now.
  - Asymmetric blend: a track switch that makes things safer (opening
    distance / slower closing) is smoothed in over LEAD_BLEND_SAFE_DIST_TIME.
    A track switch that makes things worse (closing distance, worsening
    relative speed, TTC < LEAD_BLEND_TTC_DANGER, or a jump revealing a
    meaningfully closer lead regardless of vRel) is passed through immediately.
  - Jumps bigger than LEAD_BLEND_BIG_JUMP_DIST in the safe direction are
    treated as a track identity change, not measurement noise, and are
    snapped immediately instead of blended (blending a large gap over a fixed
    time window fabricates an implied relative speed that isn't real).
  """
  def __init__(self):
    self.prev: dict | None = None
    self.miss_cnt = 0
    self.danger_hold_cnt = 0

  @staticmethod
  def _ttc(dRel: float, vRel: float) -> float:
    if vRel >= -0.1:
      return 1e3
    return max(dRel, 0.0) / max(-vRel, 0.1)

  def _is_dangerous(self, raw: dict) -> bool:
    ttc = self._ttc(raw['dRel'], raw['vRel'])
    closing = raw['vRel'] < -0.1
    worsening = (self.prev is not None and self.prev.get('status') and
                 raw['vRel'] < self.prev.get('vRel', 0.0) - 0.3)
    # 순간 vRel만으로는 못 잡는 케이스: 트랙이 바뀌면서 실제로는 훨씬 가까운
    # 리드가 드러나는 경우 (예: SCC가 근접구간/사각지대에서 순간적으로 락을
    # 놓치면서 오래된 원거리 값을 들고 있다가 비전으로 넘어가는 순간 정확한
    # 근거리 값이 드러남). vRel 부호와 무관하게 위험으로 취급한다.
    closer_jump = (self.prev is not None and self.prev.get('status') and
                   (self.prev.get('dRel', 0.0) - raw['dRel']) > LEAD_BLEND_CLOSER_JUMP_DIST)
    return closer_jump or (ttc < LEAD_BLEND_TTC_DANGER and (closing or worsening))

  def _is_cutout(self) -> bool:
    if self.prev is None or not self.prev.get('status'):
      return False
    dPath = abs(self.prev.get('dPath', 0.0))
    vRel = self.prev.get('vRel', 0.0)
    return dPath > CUTOUT_DPATH_THRESH and vRel > CUTOUT_VREL_GATE

  def update(self, raw: dict, dt: float) -> dict:
    if not raw.get('status'):
      if self._is_cutout():
        self.prev, self.miss_cnt, self.danger_hold_cnt = None, 0, 0
        return raw  # clear cut-out: report lost immediately, skip grace hold

      if self.prev is not None and self.prev.get('status'):
        self.miss_cnt += 1
        if self.miss_cnt * dt < LEAD_LOST_GRACE_TIME:
          held = dict(self.prev)
          held['dRel'] = max(0.0, held.get('dRel', 0.0) + held.get('vRel', 0.0) * dt)
          self.prev = held
          return held

      self.prev, self.miss_cnt, self.danger_hold_cnt = None, 0, 0
      return raw

    self.miss_cnt = 0

    if self.prev is None or not self.prev.get('status'):
      self.prev = dict(raw)
      return raw

    dangerous = self._is_dangerous(raw)
    if dangerous:
      self.danger_hold_cnt = int(LEAD_BLEND_DANGER_HOLD / max(dt, 1e-3))

    if dangerous or self.danger_hold_cnt > 0:
      self.danger_hold_cnt = max(0, self.danger_hold_cnt - 1)
      self.prev = dict(raw)
      return raw

    # 큰 폭(>LEAD_BLEND_BIG_JUMP_DIST)의 '안전 방향' 점프는 측정 노이즈가 아니라
    # 다른 물체로 대상이 바뀐 것으로 보고 블렌딩 없이 즉시 반영한다. 고정된
    # 시간(LEAD_BLEND_SAFE_DIST_TIME)에 큰 거리 차를 나눠 블렌딩하면, 실제로는
    # 없는 상대속도가 그 구간 동안 인위적으로 생겨 MPC 입력을 왜곡할 수 있다.
    if abs(raw['dRel'] - self.prev.get('dRel', raw['dRel'])) > LEAD_BLEND_BIG_JUMP_DIST:
      self.prev = dict(raw)
      return raw

    alpha = float(np.clip(dt / LEAD_BLEND_SAFE_DIST_TIME, 0.0, 1.0))
    blended = dict(raw)
    for k in ('dRel', 'vRel', 'vLead', 'aLead', 'aLeadK'):
      if k in raw and k in self.prev:
        blended[k] = self.prev[k] + (raw[k] - self.prev[k]) * alpha
    self.prev = dict(blended)
    return blended


class RadarD:
  def __init__(self, delay: float = 0.0):
    self.current_time = 0.0

    self.tracks: dict[int, Track] = {}

    self.v_ego = 0.0
    print("###RadarD.. : delay = ", delay, int(round(delay / DT_MDL))+1)
    self.v_ego_hist = deque([0.0], maxlen=int(round(delay / DT_MDL))+1)
    self.last_v_ego_frame = -1

    self.radar_state: capnp._DynamicStructBuilder | None = None
    self.radar_state_valid = False

    self.ready = False

    self.vision_tracks = [VisionTrack(DT_MDL), VisionTrack(DT_MDL)]
    self.lead_blend = LeadBlend()

    self.params = Params()
    self.enable_radar_tracks = self.params.get_int("EnableRadarTracks")
    self.enable_corner_radar = self.params.get_int("EnableCornerRadar")
    self.radar_lat_factor = 0.0

    self.radar_detected = False

    self._corner_lat_hist = {
      "L": deque(maxlen=10),
      "R": deque(maxlen=10),
    }
    self._corner_state = {"L": 0, "R": 0}  # -1,0,+1


  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    self.ready = sm.seen['modelV2']
    self.current_time = 1e-9*max(sm.logMonoTime.values())

    self.enable_radar_tracks = self.params.get_int("EnableRadarTracks")
    self.enable_corner_radar = self.params.get_int("EnableCornerRadar")
    self.radar_lat_factor = self.params.get_float("RadarLatFactor") * 0.01
    self.radar_reaction_factor = self.params.get_float("RadarReactionFactor") * 0.01
    self.detect_cut_in = self.radar_lat_factor > 0

    leads_v3 = sm['modelV2'].leadsV3
    if sm.recv_frame['carState'] != self.last_v_ego_frame:
      self.v_ego = sm['carState'].vEgo
      self.v_ego_hist.append(self.v_ego)
      self.last_v_ego_frame = sm.recv_frame['carState']

    valid_ids = set()
    for pt in rr.points:
      track_id = pt.trackId
      valid_ids.add(track_id)      

      if track_id not in self.tracks:
        self.tracks[track_id] = Track(track_id)

      self.tracks[track_id].update(sm['modelV2'], pt, self.ready, self.radar_reaction_factor, self.radar_lat_factor)

    for tid in list(self.tracks.keys()):
      if tid not in valid_ids:
        self.tracks.pop(tid)

    # *** publish radarState ***
    self.radar_state_valid = sm.all_checks()
    self.radar_state = log.RadarState.new_message()

    model_updated = False if self.radar_state.mdMonoTime == sm.logMonoTime['modelV2'] else True

    self.radar_state.mdMonoTime = sm.logMonoTime['modelV2']
    self.radar_state.radarErrors = rr.errors
    self.radar_state.carStateMonoTime = sm.logMonoTime['carState']

    if len(sm['modelV2'].velocity.x):
      model_v_ego = sm['modelV2'].velocity.x[0]
    else:
      model_v_ego = self.v_ego

    if len(leads_v3) > 1:

      md = sm['modelV2']
      if model_updated:
        if self.radar_detected:
          self.vision_tracks[0].cnt = 0
          self.vision_tracks[1].cnt = 0
        self.vision_tracks[0].update(leads_v3[0], model_v_ego, self.v_ego, md)
        self.vision_tracks[1].update(leads_v3[1], model_v_ego, self.v_ego, md)

      alive_tracks = {tid: trk for tid, trk in self.tracks.items() if trk.cnt > 2 }
      lead_one_raw, self.radar_detected, lead_one_scc_fallback = self.get_lead(sm['carState'], md, alive_tracks, 0, leads_v3[0], model_v_ego, low_speed_override=False)
      if lead_one_raw.get('radar') and not lead_one_scc_fallback:
        # 빨간박스: 비전과 교차검증된 레이더 트랙(또는 다중레이더) 락온 상태.
        # 이미 안정적인 실측값이므로 블렌딩 지연 없이 그대로 사용.
        # prev는 계속 갱신해둬서, 이후 파란박스(비전)로 전환되는 순간 블렌딩이 오래된 값부터
        # 시작하지 않도록 함.
        self.radar_state.leadOne = lead_one_raw
        self.lead_blend.prev = dict(lead_one_raw)
        self.lead_blend.miss_cnt = 0
        self.lead_blend.danger_hold_cnt = 0
      else:
        # 파란박스(비전) 또는 sccFallback(37차: 단일점 SCC 폴백, 비전 교차검증
        # 없이 채택된 트랙 -- 옆차선/경로이탈 물체 오탐 위험). 두 경우 모두
        # LeadBlend로 디바운싱(cutout/danger-passthrough 그대로 적용).
        # 위험한 변화(TTC 급락/closer_jump)는 danger-passthrough 경로로
        # 즉시 반영되므로 반응속도 저하는 없음 -- 완만한 케이스만 스무딩됨.
        self.radar_state.leadOne = self.lead_blend.update(lead_one_raw, DT_MDL)
      self.radar_state.leadTwo, _, _ = self.get_lead(sm['carState'], md, alive_tracks, 1, leads_v3[1], model_v_ego, low_speed_override=False)

      self.lane_line_available = md.laneLineProbs[1] > 0.5 and md.laneLineProbs[2] > 0.5
      self.compute_leads(self.v_ego, alive_tracks, md)
      if self.leadTwo is not None:
        self.radar_state.leadTwo = self.leadTwo
      if self.enable_radar_tracks >= 3:
        self._pick_lead_one_from_state()

  def publish(self, pm: messaging.PubMaster):
    assert self.radar_state is not None

    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = self.radar_state_valid
    radar_msg.radarState = self.radar_state
    pm.send("radarState", radar_msg)

  def get_lead(self, CS, md, tracks: dict[int, Track], index: int, lead_msg: capnp._DynamicStructReader,
               model_v_ego: float, low_speed_override: bool = True) -> dict[str, Any]:

    v_ego = self.v_ego
    ready = self.ready

    ## backup SCC radar(0, 1 trackid)
    if self.enable_radar_tracks <= 0:
      track_scc = tracks.get(0)
    else:
      track_scc = tracks.pop(0, None)

    # Determine leads, this is where the essential logic happens
    if len(tracks) > 0 and ready and lead_msg.prob > .4:
      track = match_vision_to_track(v_ego, lead_msg, tracks)
    else:
      track = None

    used_scc_fallback = False
    if (track is None or lead_msg.prob < .6) and track_scc is not None and track_scc.cnt > 2:
      #if self.enable_radar_tracks in [-1, 2] or model_v_ego < 5 or track_scc.vLead < 5.0:
      if self.enable_radar_tracks == -1 or (self.enable_radar_tracks >= 2 and track_scc.vLead < 5.0):
        # 37차: track_scc는 비전 대응 없이 무조건 채택되는 단일점 폴백이라
        # 옆차선/주행경로 밖 정지물체를 걸러낼 안전장치가 없었음. dPath로
        # 차로내 위치를 선제 검증 -- 게이트를 넘으면(명백히 차로 밖) 폴백
        # 자체를 채택하지 않는다. 이 검증은 track이 이미 있었는지 여부와
        # 무관하게 항상 적용(있었으면 그 기존 track을 유지, 없었으면
        # vision-only 경로로 자연스럽게 넘어감) -- track이 이미 있다고
        # 게이트를 건너뛰면 저확신(prob<.6) 케이스에서 검증을 우회하는
        # 구멍이 그대로 남는다.
        if abs(track_scc.dPath) < SCC_FALLBACK_DPATH_GATE:
          track = track_scc
          used_scc_fallback = True

    lead_dict = {'status': False}
    radar = False
    if track is not None:
      lead_dict = track.get_RadarState(lead_msg.prob, self.vision_tracks[0].yRel)
      radar = True
    elif (track is None) and ready and (lead_msg.prob > .5):
        lead_dict = self.vision_tracks[index].get_lead(md)

    if self.enable_corner_radar > 1:
      lead_dict = self.corner_radar(CS, lead_dict)

    if low_speed_override:
      low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
      if len(low_speed_tracks) > 0:
        closest_track = min(low_speed_tracks, key=lambda c: c.dRel)

        # Only choose new track if it is actually closer than the previous one
        if (not lead_dict['status']) or (closest_track.dRel < lead_dict['dRel']):
          #lead_dict = closest_track.get_RadarState(lead_msg.prob, self.vision_tracks[0].yRel, self.vision_tracks[0].vLat)
          lead_dict = closest_track.get_RadarState(lead_msg.prob, self.vision_tracks[0].yRel)

    return lead_dict, radar, used_scc_fallback

  def compute_leads(self, v_ego, tracks, md):
    lead_msg = md.leadsV3[0] if (md is not None and len(md.position.x) == 33) else None
    self.leadCutIn = {'status': False}
    if lead_msg is None:
      # reset
      self.radar_state.leadsLeft = []
      self.radar_state.leadsCenter = []
      self.radar_state.leadsRight = []
      self.radar_state.leadLeft = {'status': False}
      self.radar_state.leadRight = {'status': False}
      return
    
    left_list, right_list, center_list, cutin_list = [], [], [], []
    for c in tracks.values():
      y_rel_neg = - c.yRel
      # center
      if c.in_lane_prob > 0.3:
        if c.cnt > 3:
          ld = c.get_RadarState(lead_msg.prob, float(-lead_msg.y[0]))
          ld['modelProb'] = 0.01
          center_list.append(ld)

      # left/right
      elif y_rel_neg < 0: #left_lane_y:
        ld = c.get_RadarState(0, 0)
        if self.lane_line_available and c.in_lane_prob_future > 0.1 and c.cnt > int(2.0/DT_MDL):
          if c.cut_in_count > int(0.1/DT_MDL):
            ld['modelProb'] = 0.03
            cutin_list.append(ld)
          c.cut_in_count += 2
        left_list.append(ld)
      else:
        ld = c.get_RadarState(0, 0)
        if self.lane_line_available and c.in_lane_prob_future > 0.1 and c.cnt > int(2.0/DT_MDL):
          if c.cut_in_count > int(0.1/DT_MDL):
            ld['modelProb'] = 0.03
            cutin_list.append(ld)
          c.cut_in_count += 2
        right_list.append(ld)

      c.cut_in_count = max(c.cut_in_count - 1, 0)

    self.radar_state.leadsLeft   = left_list
    self.radar_state.leadsRight  = right_list
    self.radar_state.leadsCenter = center_list
    self.radar_state.leadsCutIn = cutin_list
    self.leadCutIn = min(
      (ld for ld in cutin_list if 3 < ld['dRel'] < 50 and ld['vLead'] > 4),
      key=lambda d: d['dRel'],
      default={'status': False}
    )

    self.radar_state.leadLeft  = min(
        (ld for ld in left_list if ld['dRel'] > 5 and abs(ld['dPath']) < 3.5),
        key=lambda d: d['dRel'],
        default={'status': False}
    )
    self.radar_state.leadRight = min(
        (ld for ld in right_list if ld['dRel'] > 5 and abs(ld['dPath']) < 3.5),
        key=lambda d: d['dRel'],
        default={'status': False}
    )
   
    self.leadTwo = None
    if self.lane_line_available:
      self.leadCenter = min(
          (ld for ld in center_list if ld['vLead'] > 5 and ld['radar'] and ld['dRel'] > 3.5),
          key=lambda d: d['dRel'],
          default=None
      )
      if self.radar_state.leadOne.status and self.radar_state.leadOne.radar:
        self.leadTwo = min(
            (ld for ld in center_list if ld['vLead'] > 5 and ld['radar'] and self.radar_state.leadOne.dRel < ld['dRel'] < 80),
            key=lambda d: d['dRel'],
            default=None
        )
        if self.leadTwo is not None:
          self.leadTwo = copy.deepcopy(self.leadTwo)
          #gap = self.leadTwo['dRel'] - self.radar_state.leadOne.dRel
          #offset = 3.0 + min(gap * 0.2, 10)
          #self.leadTwo['dRel'] = self.radar_state.leadOne.dRel + offset
          self.leadTwo['dRel'] = max(self.radar_state.leadOne.dRel + 3.0, self.leadTwo['dRel'] - 8.0) # lead+1 차를 뒤로 8M후퇴하여, mpc에서  감자하도록함.. 최소 lead보다 3M앞에 위치하도록
    else:
      self.leadCenter = None

    def _ok(ld):
        return (ld.get('vLead', 0) > 2 and
                abs(ld.get('dPath', 0)) < 4.2 and
                ld.get('dRel', 0) > 2)

    def _pick_two_with_gap(cands, min_gap=5.0):
        xs = sorted((ld for ld in cands if _ok(ld)), key=lambda d: d['dRel'])
        if not xs:
            return []
        first = xs[0]
        second = None
        for ld in xs[1:]:
            # 5m 이상 떨어진 후보만 허용 (>= 5.0)
            if (ld['dRel'] - first['dRel']) >= min_gap:
                second = ld
                break
        return [first] if second is None else [first, second]

    self.radar_state.leadsLeft2  = _pick_two_with_gap(left_list,  min_gap=5.0)
    self.radar_state.leadsRight2 = _pick_two_with_gap(right_list, min_gap=5.0)

  def _pick_lead_one_from_state(self):
    chosen = None
    detected = self.radar_detected

    if self.leadCutIn and self.leadCutIn.get("status") and self.detect_cut_in:
      if self.radar_state.leadOne.status:
        if self.leadCutIn["dRel"] < self.radar_state.leadOne.dRel:
          chosen = self.leadCutIn
          chosen["modelProb"] = 0.03
          detected = True
      else:
        chosen = self.leadCutIn
        chosen["modelProb"] = 0.03
        detected = True

    elif self.leadCenter and self.leadCenter["status"]:
      if self.radar_detected:
        if self.radar_state.leadOne.status and self.leadCenter["dRel"] < self.radar_state.leadOne.dRel:
          chosen = self.leadCenter
          chosen["modelProb"] = 0.01
      else:
        chosen = self.leadCenter
        chosen["modelProb"] = 0.02
        detected = True

    if chosen is not None:
        self.radar_state.leadOne = chosen
        self.radar_detected = detected

  def _corner_update_state(self, side: str, cur_lat: float, enter_lat: float = 2.8) -> int:
    # 유효 범위 밖이면 리셋
    if not (0.0 < cur_lat < enter_lat):
      self._corner_lat_hist[side].clear()
      self._corner_state[side] = 0
      return 0

    h = self._corner_lat_hist[side]
    h.append(cur_lat)

    n = len(h)
    if n < 3:
      # 데이터 너무 적으면 이전 상태 유지
      return self._corner_state[side]

    delta = h[-1] - h[0]
    th = 0.02 # 3 * (20 / n)

    if delta < -th:
      self._corner_state[side] = +1   # approaching
    elif delta > th:
      self._corner_state[side] = -1   # leaving
    else:
      self._corner_state[side] = 0    # maintain

    return self._corner_state[side]
 
  def corner_radar(self, CS, lead_dict):
    ENTER_LAT = 2.2
    KEEP_LAT  = 2.0
    EXIT_LAT  = 1.2

    left_lat, right_lat = abs(CS.leftLatDist), abs(CS.rightLatDist)
    left_state  = self._corner_update_state("L", left_lat)
    right_state = self._corner_update_state("R", right_lat)

    # 1) left usable?
    left_ok = False
    if left_state > 0:
      left_ok = left_lat < ENTER_LAT
    elif left_state == 0:
      left_ok = 0 < left_lat < KEEP_LAT
    else:  # leaving
      left_ok = left_lat <= EXIT_LAT

    # 2) right usable?
    right_ok = False
    if right_state > 0:
      right_ok = right_lat < ENTER_LAT
    elif right_state == 0:
      right_ok = 0 < right_lat < KEEP_LAT
    else:
      right_ok = right_lat <= EXIT_LAT

    # 3) 아무도 못 쓰면 skip
    if not left_ok and not right_ok:
      return lead_dict

    # 4) 둘 다 되면 longDist로 선택
    if left_ok and right_ok:
      if CS.leftLongDist <= CS.rightLongDist:
        lat_dist, long_dist = +left_lat, CS.leftLongDist
      else:
        lat_dist, long_dist = -right_lat, CS.rightLongDist
    elif left_ok:
      lat_dist, long_dist = +left_lat, CS.leftLongDist
    else:
      lat_dist, long_dist = -right_lat, CS.rightLongDist
    
    if lead_dict['status']:
      if lead_dict['dRel'] > long_dist:
        lead_dict['dRel'] = long_dist
        lead_dict['yRel'] = lat_dist
        lead_dict['vRel'] = 0.0
        lead_dict['vLead'] = CS.vEgo if CS.vEgo < lead_dict['vLead'] else lead_dict['vLead']
        lead_dict['vLeadK'] = lead_dict['vLead']
        lead_dict['aLead'] = CS.aEgo if CS.aEgo < lead_dict['aLead'] else lead_dict['aLead']
        lead_dict['aLeadK'] = lead_dict['aLead']
        lead_dict['aLeadTau'] = _LEAD_ACCEL_TAU
        lead_dict['jLead'] = 0.0
        lead_dict['vLat'] = 0.0
        lead_dict['modelProb'] = 1.0
        lead_dict['radarTrackId'] = -1
        lead_dict['radar'] = True
    else:
      lead_dict['status'] = True
      lead_dict['dRel'] = long_dist
      lead_dict['yRel'] = lat_dist
      lead_dict['vRel'] = 0.0
      lead_dict['vLead'] = CS.vEgo
      lead_dict['vLeadK'] = CS.vEgo
      lead_dict['aLead'] = CS.aEgo
      lead_dict['aLeadK'] = CS.aEgo
      lead_dict['aLeadTau'] = _LEAD_ACCEL_TAU
      lead_dict['jLead'] = 0.0
      lead_dict['vLat'] = 0.0
      lead_dict['modelProb'] = 1.0
      lead_dict['radarTrackId'] = -1
      lead_dict['radar'] = True

    return lead_dict

# fuses camera and radar data for best lead detection
def main() -> None:
  config_realtime_process(5, Priority.CTRL_LOW)

  # wait for stats about the car to come in from controls
  cloudlog.info("radard is waiting for CarParams")
  CP = messaging.log_from_bytes(Params().get("CarParams", block=True), car.CarParams)
  cloudlog.info("radard got CarParams")

  # *** setup messaging
  #sm = messaging.SubMaster(['modelV2', 'carState', 'liveTracks'], poll='modelV2')
  sm = messaging.SubMaster(['modelV2', 'carState', 'liveTracks'])
  pm = messaging.PubMaster(['radarState'])

  RD = RadarD(CP.radarDelay)

  while 1:
    sm.update()

    if sm.updated['modelV2']:
      RD.update(sm, sm['liveTracks'])
      RD.publish(pm)


if __name__ == "__main__":
  main()
