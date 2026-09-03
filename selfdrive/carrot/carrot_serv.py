import fcntl
import json
import math
import os
import socket
import struct
import subprocess
import threading
import time
import numpy as np
from datetime import datetime

from ftplib import FTP
from cereal import log
import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.common.params import Params
from openpilot.common.filter_simple import MyMovingAverage
from openpilot.system.hardware import PC, TICI
from openpilot.selfdrive.navd.helpers import Coordinate
from opendbc.car.common.conversions import Conversions as CV
from openpilot.common.gps import get_gps_location_service

nav_type_mapping = {
  12: ("turn", "left", 1),
  16: ("turn", "sharp left", 1),
  1000: ("turn", "slight left", 1),
  1001: ("turn", "slight right", 2),
  1002: ("fork", "slight left", 3),
  1003: ("fork", "slight right", 4),
  1006: ("off ramp", "left", 3),
  1007: ("off ramp", "right", 4),
  13: ("turn", "right", 2),
  19: ("turn", "sharp right", 2),
  102: ("off ramp", "slight left", 3),
  105: ("off ramp", "slight left", 3),
  112: ("off ramp", "slight left", 3),
  115: ("off ramp", "slight left", 3),
  101: ("off ramp", "slight right", 4),
  104: ("off ramp", "slight right", 4),
  111: ("off ramp", "slight right", 4),
  114: ("off ramp", "slight right", 4),
  7: ("fork", "left", 3),
  44: ("fork", "left", 3),
  17: ("fork", "left", 3),
  75: ("fork", "left", 3),
  76: ("fork", "left", 3),
  118: ("fork", "left", 3),
  6: ("fork", "right", 4),
  43: ("fork", "right", 4),
  73: ("fork", "right", 4),
  74: ("fork", "right", 4),
  123: ("fork", "right", 4),
  124: ("fork", "right", 4),
  117: ("fork", "right", 4),
  131: ("rotary", "slight right", 5),
  132: ("rotary", "slight right", 5),
  140: ("rotary", "slight left", 5),
  141: ("rotary", "slight left", 5),
  133: ("rotary", "right", 5),
  134: ("rotary", "sharp right", 5),
  135: ("rotary", "sharp right", 5),
  136: ("rotary", "sharp left", 5),
  137: ("rotary", "sharp left", 5),
  138: ("rotary", "sharp left", 5),
  139: ("rotary", "left", 5),
  142: ("rotary", "straight", 5),
  14: ("turn", "uturn", 5),
  201: ("arrive", "straight", 5),
  51: ("notification", "straight", None),
  52: ("notification", "straight", None),
  53: ("notification", "straight", None),
  54: ("notification", "straight", None),
  55: ("notification", "straight", None),
  153: ("", "", 6),  #TG
  154: ("", "", 6),  #TG
  249: ("", "", 6)   #TG
}

import collections
class CarrotServ:
  def __init__(self):
    self.params = Params()
    self.params_memory = Params("/dev/shm/params")

    self.nRoadLimitSpeed = 30
    self.nRoadLimitSpeed_last = 30
    self.nRoadLimitSpeed_counter = 0

    self.active_carrot = 0     ## 1: CarrotMan Active, 2: sdi active , 3: speed decel active, 4: section active, 5: bump active, 6: speed limit active
    self.active_count = 0
    self.active_sdi_count = 0
    self.active_sdi_count_max = 200 # 20 sec

    self.active_kisa_count = 0

    self.nSdiType = -1
    self.nSdiSpeedLimit = 0
    self.nSdiSection = 0
    self.nSdiDist = 0
    self.nSdiBlockType = -1
    self.nSdiBlockSpeed = 0
    self.nSdiBlockDist = 0

    self.nTBTDist = 0
    self.nTBTTurnType = -1
    self.szTBTMainText = ""
    self.szNearDirName = ""
    self.szFarDirName = ""
    self.nTBTNextRoadWidth = 0

    self.nTBTDistNext = 0
    self.nTBTTurnTypeNext = -1
    self.szTBTMainTextNext = ""

    self.nGoPosDist = 0
    self.nGoPosTime = 0
    self.szPosRoadName = ""
    self.nSdiPlusType = -1
    self.nSdiPlusSpeedLimit = 0
    self.nSdiPlusDist = 0
    self.nSdiPlusBlockType = -1
    self.nSdiPlusBlockSpeed = 0
    self.nSdiPlusBlockDist = 0

    self.goalPosX = 0.0
    self.goalPosY = 0.0
    self.szGoalName = ""
    self.vpPosPointLatNavi = 0.0
    self.vpPosPointLonNavi = 0.0
    self.vpPosPointLat = 0.0
    self.vpPosPointLon = 0.0
    self.roadcate = 8

    self.nPosSpeed = 0.0
    self.nPosAngle = 0.0
    self.nPosAnglePhone = 0.0

    self.diff_angle_count = 0
    self.last_calculate_gps_time = 0
    # [166차] CC.orientationNED 델타앵커링 헤딩보정 상태
    self.cc_yaw_at_fix = None
    self._prev_fix_time_for_heading = 0
    # [167차] 163차 게이트 조건 좁히기용 -- _update_gps 최초 호출 전
    # 기본값은 False(안전측, 방향2가 정상 발동 가능한 상태로 시작)
    self.cc_pose_valid = False
    self.last_update_gps_time = 0
    self.last_update_gps_time_phone = 0
    self.last_update_gps_time_navi = 0
    self.bearing_offset = 0.0
    self.bearing_measured = 0.0
    # [162차] 마지막 실제 위치 fix(navi/phone/internal GPS) 이후 estimate_position()이
    # 데드레커닝만으로 외삽해온 경과시간(초). carrot_man.py::carrot_navi_route()가
    # route_speed 램프리미터의 위치불확실성 게이트 판정에 사용(FINDINGS.md 162차).
    self.position_dt_since_fix = 0.0
    self.bearing = 0.0
    self.gps_valid = False
    # [169차 계측] "패킷단절 vs 내용정지" 구분용 -- navi 패킷이 마지막으로
    # "도착"한 시점(last_update_gps_time_navi) 기준 경과시간. 이 값이 계속
    # 3.0 미만으로 유지되는데도 vpPosPointLatNavi가 안 바뀐다면 "내용정지"
    # (패킷은 오지만 값이 그대로), 이 값 자체가 3.0을 넘으면 "패킷단절"
    # (FINDINGS.md 169차 NEEDS_INVESTIGATION 참고).
    self.dt_navi_packet_age = 0.0

    # [194차] carrot_man.py::carrot_navi_route()가 계산한 route apex 진단
    # telemetry를 cereal(msg.carrotMan)로 실제 발행하기 위한 저장 공간.
    # carrot_man.py는 CarrotMan(self._route_apex_idx 등)과 CarrotServ가
    # 별개 객체이므로, carrot_man.py가 계산 직후 이 속성에도 값을 써준다
    # (FINDINGS.md 193차/194차 참고). 미계산/스킵 시 기본값 유지.
    self.route_apex_idx = -1
    self.route_apex_dist = 0.0
    self.route_apex_speed = 0.0
    self.route_out_speed = 300.0

    # [227차] carrot_man.py::carrot_navi_route()의 self.route_active(ACTIVE
    # 추적 상태기계, 223차)를 update_navi()에서도 판별하기 위한 저장 공간.
    # 위 route_apex_* 와 동일한 이유(별개 객체, 계산 직후 이 속성에 값을
    # 써줌)로 존재 -- 226차가 ACTIVE 진입 게이트(route_active=False 유지)에서
    # out_speed=apex_speed(ceiling)를 반환하게 바꾼 뒤, 225차 B의
    # min(v_ego_kph, ...) 클램프가 이 ceiling 분기에도 무차별 적용되어
    # route_speed가 매 프레임 vEgo 그 자체로 고정되고, 그 결과
    # desired_speed=min(route,...)도 vEgo로 고정되어 가속 명령이 전혀
    # 생성되지 않는 회귀가 발견됨(FINDINGS.md 227차, 다중 프레임 시뮬레이션
    # 재현). ACTIVE 추적 분기(route_active=True, target 방향 실제 감속/inert
    # 통과)와 ACTIVE 진입 게이트 ceiling 분기(route_active=False, apex_speed
    # 상수 ceiling)는 route_speed의 의미가 다르므로 클램프도 분기별로
    # 달라야 한다 -- 아래 update_navi()에서 이 값으로 구분.
    self.route_active = False
    # [228차, route_inert v2, FINDINGS.md 228차] route_active=True(ACTIVE
    # 추적) 중에서도 "아직 거리는 남았지만 vEgo가 이미 target 이하"인
    # far-inert 프레임을 구분하기 위한 신규 상태. 위 route_active와 동일한
    # mirroring 패턴(carrot_man.py::carrot_navi_route()가 매 프레임 계산
    # 직후 이 속성에 값을 씀)으로 존재 -- route_active=True인데
    # route_inert=True이면 아래 update_navi()의 vEgo 상한 클램프를
    # 생략해야 정차 원인 해소 후 재가속 경로가 막히는 자기참조적 고착이
    # 재발하지 않는다.
    self.route_inert = False

    # [204차 계측, 203차 옵션1] apex 선택 직전의 candidates 리스트(개수 +
    # 최근접 3개) 관측용 저장 공간. 위 route_apex_* 와 동일한 이유/패턴으로
    # carrot_man.py가 계산 직후 여기에 값을 써준다. 제어 로직에는 사용되지 않음.
    self.route_candidate_count = 0
    self.route_candidate0_idx = -1
    self.route_candidate0_dist = 0.0
    self.route_candidate0_speed = 0.0
    self.route_candidate1_idx = -1
    self.route_candidate1_dist = 0.0
    self.route_candidate1_speed = 0.0
    self.route_candidate2_idx = -1
    self.route_candidate2_dist = 0.0
    self.route_candidate2_speed = 0.0

    self.phone_gps_accuracy = 0.0
    self.gps_accuracy_device = 0.0
    self.phone_latitude = 0.0
    self.phone_longitude = 0.0
    self.phone_gps_frame = 0

    self.totalDistance = 0
    self.xSpdLimit = 0
    self.xSpdDist = 0
    self.xSpdType = -1

    self.xTurnInfo = -1
    self.xDistToTurn = 0
    self.xTurnInfoNext = -1
    self.xDistToTurnNext = 0

    self.navType, self.navModifier = "invalid", ""
    self.navTypeNext, self.navModifierNext = "invalid", ""

    self.carrotIndex = 0
    self.carrotCmdIndex = 0
    self.carrotCmd = ""
    self.carrotArg = ""
    self.carrotCmdIndex_last = 0

    self.traffic_light_q = collections.deque(maxlen=int(2.0/0.1))  # 2 secnods
    self.traffic_light_count = -1
    self.traffic_state = 0

    self.left_spd_sec = 0
    self.left_tbt_sec = 0
    self.left_sec = 100
    self.max_left_sec = 100
    self.carrot_left_sec = 100
    self.sdi_inform = False


    self.atc_paused = False
    self.atc_activate_count = 0
    self.gas_override_speed = 0
    self.gas_pressed_state = False
    self.source_last = "none"

    # 2026-08-20 (devnotes FINDINGS.md "[NEEDS_VALIDATION] src/desiredSpeed
    # 플리커 -- vturn<->road/model/route 전환에서 대규모 재현" 대응, 11차
    # 코드 재검토로 게이팅 기준을 desiredCurvature(현재값) -> model_turn_speed
    # 추세(trend) 기반으로 재설계): "model" 후보(modelTurnSpeed)는
    # desire_helper._make_model_turn_speed()가 모델 예측 "미래" 궤적의
    # 속도를 저역통과 필터링한 값이다 -- 반면 vturn/route는 각자 곡률/거리
    # 기반으로 "이 지점이 커브인지 직선인지"를 명시적으로 판단해서 무제한
    # (250에 가까운 값)으로 되돌린다. 이 판단 지연 때문에 model 후보는
    # 실제로는 이미 직선에 들어섰는데도 필터 지연으로 낮은 값을 잠깐 더
    # 들고 있다가 vturn/route가 이미 250으로 복귀한 뒤에야 뒤늦게 따라
    # 올라온다 -- 그 사이 min() 후보가 프레임마다 왕복하며 src/desiredSpeed
    # 플리커로 나타남 (FINDINGS.md 실측 164건 중 A->B->A 49건, vturn<->model이
    # 가장 우세한 쌍).
    #
    # 1차 대응(2226db7)은 modelV2.action.desiredCurvature(차선 추종용
    # "현재" 곡률)가 일정 시간 미만이면 직선으로 보고 model 후보를
    # 배제했으나, 재검토 결과 이 기준은 "커브를 빠져나온 뒤 model이 뒤늦게
    # 따라오는" 트레일링 케이스뿐 아니라 "아직 커브에 진입하기 전, 직선
    # 구간에서 model이 먼저 앞의 커브를 보고 사전감속을 시도하는" 정상
    # 케이스까지 같이 걸러낼 위험이 있었다 -- desiredCurvature는 lookahead가
    # 없는 "지금 이 순간" 값이라 커브 진입 직전 직선 구간에서도 threshold
    # 미만이 되기 때문 (devnotes FINDINGS.md
    # "[RISK_IDENTIFIED] model_turn_straight_gate ... 진입 전 사전감속 억제
    # 위험" 참고).
    #
    # 재설계: desiredCurvature 대신 model_turn_speed 자기 자신의 추세를
    # 본다. "최근 model_turn_straight_hold_sec 동안 model_turn_speed가
    # (노이즈 허용범위를 넘어) 한 번도 감소하지 않고 계속 높거나 회복
    # (반등) 중"일 때만 -- 즉 이미 최저점을 지나 복귀하는 중인데 vturn/
    # route는 진작 풀린 트레일링 상태로 판단해 배제한다. 반대로 지금
    # 하강 중(=커브 접근하며 사전감속 시도 중)이면 단 한 프레임의 하강만
    # 있어도 카운터가 즉시 리셋되어 배제되지 않는다 -- 진입측 사전감속은
    # 건드리지 않는 비대칭 설계 유지.
    # noise_tol(0.3km/h)은 모델 예측값의 프레임간 미세 흔들림까지 "감소"로
    # 오판해 카운터가 계속 리셋되는 것을 막기 위한 허용폭 -- 실차 검증 후
    # 필요시 조정.
    #
    # [50차 재설계] 위 "직전 프레임 대비 비감소 연속" 방식은 실측(route1
    # 203f99d429 seg8) 결과 완만한 하강 추세 중의 잔떨림(프레임간 noise_tol
    # 0.3km/h를 넘는 상승 튐)에도 카운터가 계속 리셋되어, 정작 커브에 접근
    # 중인 구간(vEgo가 계속 가속하는데 model_turn_speed는 91~108km/h로
    # 안정적으로 낮게 유지되던 11초 구간)에서도 트레일링으로 오판, model
    # 후보가 반복적으로 배제됨을 확인(FINDINGS.md 50차 항목). 이 때문에
    # 실제 감속 트리거가 apex 3초 전까지 늦춰지는 문제가 있었다.
    # 대응: "직전 프레임 대비"가 아니라 "최근 확인된 최저점(min_recent)
    # 대비 recover_margin(3km/h) 이상 지속 회복"으로 트레일링을 판정한다.
    # 완만한 하강 중의 잔떨림은 min_recent를 거의 갱신하지 못해도 margin
    # 안에서 계속 "회복 아님"으로 유지되므로 카운터가 리셋되지 않고, 실제로
    # 커브를 빠져나와 뚜렷하게 반등하는 경우만 hold_sec 동안 margin을
    # 초과 유지해 트레일링으로 확정된다.
    self.model_turn_speed_min_recent = None           # 최근 확인된 model_turn_speed 최저점(트레일링 판단 기준선)
    self.model_turn_recover_margin = 3.0              # km/h, 이 폭 이상 최저점 위로 지속 회복해야 트레일링으로 침
    self.model_turn_straight_hold_sec = 0.6           # 이 시간 이상 연속 회복 유지돼야 model 후보 배제
    self.model_turn_straight_count = 0                # 연속 "회복 유지" 프레임 카운터 (20Hz 루프 기준)

    self.debugText = ""

    # 默认语言，稍后在 update_params 中从 Params 读取覆盖，
    # 规则：main_ko -> 韩语；main_zh-CHS -> 中文；其他 -> 英文
    self.lang = "en"

    self.update_params()

  def update_params(self):
    self.autoNaviSpeedBumpSpeed = float(self.params.get_int("AutoNaviSpeedBumpSpeed"))
    self.autoNaviSpeedBumpTime = float(self.params.get_int("AutoNaviSpeedBumpTime"))
    self.autoNaviSpeedCtrlEnd = float(self.params.get_int("AutoNaviSpeedCtrlEnd"))
    self.autoNaviSpeedCtrlMode = self.params.get_int("AutoNaviSpeedCtrlMode")
    self.autoNaviSpeedSafetyFactor = float(self.params.get_int("AutoNaviSpeedSafetyFactor")) * 0.01
    self.autoNaviSpeedDecelRate = float(self.params.get_int("AutoNaviSpeedDecelRate")) * 0.01
    self.autoNaviCountDownMode = self.params.get_int("AutoNaviCountDownMode")
    self.turnSpeedControlMode= self.params.get_int("TurnSpeedControlMode")
    # [210차] 이 값을 곱하던 유일한 사용처(update_navi() route_speed 계산,
    # 아래 L~1101 부근)를 제거함 -- 현재는 어디에서도 쓰이지 않는 죽은 값이다.
    # UI 슬라이더("경로턴속도반영비율")와 params_keys.h 기본값은 그대로
    # 남겨둠(최소변경 원칙, §27) -- 필요시 향후 재사용/제거는 별도 논의.
    self.mapTurnSpeedFactor= self.params.get_float("MapTurnSpeedFactor") * 0.01

    self.autoTurnControlSpeedTurn = self.params.get_int("AutoTurnControlSpeedTurn")
    self.autoTurnMapChange = self.params.get_int("AutoTurnMapChange")
    self.autoTurnControl = self.params.get_int("AutoTurnControl")
    self.autoTurnControlTurnEnd = self.params.get_int("AutoTurnControlTurnEnd")
    #self.autoNaviSpeedDecelRate = float(self.params.get_int("AutoNaviSpeedDecelRate")) * 0.01
    self.autoCurveSpeedLowerLimit = int(self.params.get("AutoCurveSpeedLowerLimit"))
    self.is_metric = self.params.get_bool("IsMetric")
    self.autoRoadSpeedLimitOffset = self.params.get_int("AutoRoadSpeedLimitOffset")

    # 读取语言设置：优先使用 LanguageSetting，与 UI 保持一致；回退读取可能存在的 "lang"
    try:
      lang_val = self.params.get('LanguageSetting') or self.params.get('lang')
    except Exception:
      lang_val = None
    if isinstance(lang_val, bytes):
      try:
        lang_val = lang_val
      except Exception:
        lang_val = None
    if lang_val == "main_ko":
      self.lang = "ko"
    elif lang_val == "main_zh-CHS":
      self.lang = "zh"
    else:
      self.lang = "en"


  def _update_cmd(self):
    if self.carrotCmdIndex != self.carrotCmdIndex_last:
      self.carrotCmdIndex_last = self.carrotCmdIndex
      command_handlers = {
        "DETECT": self._handle_detect_command,
      }

      handler = command_handlers.get(self.carrotCmd)
      if handler:
        handler(self.carrotArg)

    self.traffic_light_q.append((-1, -1, "none", 0.0))
    self.traffic_light_count -= 1
    if self.traffic_light_count < 0:
      self.traffic_light_count = -1
      self.traffic_state = 0

  def _handle_detect_command(self, xArg):
    elements = [e.strip() for e in xArg.split(',')]
    if len(elements) >= 4:
      try:
        state = elements[0]
        value1 = float(elements[1])
        value2 = float(elements[2])
        value3 = float(elements[3])
        self.traffic_light(value1, value2, state, value3)
        self.traffic_light_count = int(0.5 / 0.1)
      except ValueError:
        pass

  def traffic_light(self, x, y, color, cnf):
    traffic_red = 0
    traffic_green = 0
    traffic_left = 0
    traffic_red_trig = 0
    traffic_green_trig = 0
    traffic_left_trig = 0
    for pdata in self.traffic_light_q:
      px, py, pcolor,pcnf = pdata
      if abs(x - px) < 0.2 and abs(y - py) < 0.2:
        if pcolor in ["Green Light", "Left turn"]:
          if color in ["Red Light", "Yellow Light"]:
            traffic_red_trig += cnf
            traffic_red += cnf
          elif color in ["Green Light", "Left turn"]:
            traffic_green += cnf
        elif pcolor in ["Red Light", "Yellow Light"]:
          if color in ["Green Light"]: #, "Left turn"]:
            traffic_green_trig += cnf
            traffic_green += cnf
          elif color in ["Left turn"]:
            traffic_left_trig += cnf
            traffic_left += cnf
          elif color in ["Red Light", "Yellow Light"]:
            traffic_red += cnf

    #print(self.traffic_light_q)
    if traffic_red_trig > 0:
      self.traffic_state = 1
      #self._add_log("Red light triggered")
      #print("Red light triggered")
    elif traffic_green_trig > 0 and traffic_green > traffic_red:  #주변에 red light의 cnf보다 더 크면 출발... 감지오류로 출발하는경우가 생김.
      self.traffic_state = 2
      #self._add_log("Green light triggered")
      #print("Green light triggered")
    elif traffic_left_trig > 0:
      self.traffic_state = 3
    elif traffic_red > 0:
      self.traffic_state = 1
      #self._add_log("Red light continued")
      #print("Red light continued")
    elif traffic_green > 0:
      self.traffic_state = 2
      #self._add_log("Green light continued")
      #print("Green light continued")
    else:
      self.traffic_state = 0
      #print("TrafficLight none")

    self.traffic_light_q.append((x,y,color,cnf))


  def calculate_current_speed(self, left_dist, safe_speed_kph, safe_time, safe_decel_rate):
    safe_speed = safe_speed_kph / 3.6
    safe_dist = safe_speed * safe_time
    decel_dist = left_dist - safe_dist

    if decel_dist <= 0:
      return safe_speed_kph

    # v_i^2 = v_f^2 + 2ad
    temp = safe_speed**2 + 2 * safe_decel_rate * decel_dist  # 공식에서 감속 적용

    if temp < 0:
      speed_mps = safe_speed
    else:
      speed_mps = math.sqrt(temp)
    return max(safe_speed_kph, min(250, speed_mps * 3.6))

  def _update_tbt(self):
    #xTurnInfo : 1: left turn, 2: right turn, 3: left lane change, 4: right lane change, 5: rotary, 6: tg, 7: arrive or uturn
    turn_type_mapping = {
      12: ("turn", "left", 1),
      16: ("turn", "sharp left", 1),
      13: ("turn", "right", 2),
      19: ("turn", "sharp right", 2),
      102: ("off ramp", "slight left", 3),
      105: ("off ramp", "slight left", 3),
      112: ("off ramp", "slight left", 3),
      115: ("off ramp", "slight left", 3),
      101: ("off ramp", "slight right", 4),
      104: ("off ramp", "slight right", 4),
      111: ("off ramp", "slight right", 4),
      114: ("off ramp", "slight right", 4),
      7: ("fork", "left", 3),
      44: ("fork", "left", 3),
      17: ("fork", "left", 3),
      75: ("fork", "left", 3),
      76: ("fork", "left", 3),
      118: ("fork", "left", 3),
      6: ("fork", "right", 4),
      43: ("fork", "right", 4),
      73: ("fork", "right", 4),
      74: ("fork", "right", 4),
      123: ("fork", "right", 4),
      124: ("fork", "right", 4),
      117: ("fork", "right", 4),
      131: ("rotary", "slight right", 5),
      132: ("rotary", "slight right", 5),
      140: ("rotary", "slight left", 5),
      141: ("rotary", "slight left", 5),
      133: ("rotary", "right", 5),
      134: ("rotary", "sharp right", 5),
      135: ("rotary", "sharp right", 5),
      136: ("rotary", "sharp left", 5),
      137: ("rotary", "sharp left", 5),
      138: ("rotary", "sharp left", 5),
      139: ("rotary", "left", 5),
      142: ("rotary", "straight", 5),
      14: ("turn", "uturn", 7),
      201: ("arrive", "straight", 8),
      51: ("notification", "straight", 0),
      52: ("notification", "straight", 0),
      53: ("notification", "straight", 0),
      54: ("notification", "straight", 0),
      55: ("notification", "straight", 0),
      153: ("", "", 6),  #TG
      154: ("", "", 6),  #TG
      249: ("", "", 6)   #TG
    }

    if self.nTBTTurnType in turn_type_mapping:
      self.navType, self.navModifier, self.xTurnInfo = turn_type_mapping[self.nTBTTurnType]
    else:
      self.navType, self.navModifier, self.xTurnInfo = "invalid", "", -1

    if self.nTBTTurnTypeNext in turn_type_mapping:
      self.navTypeNext, self.navModifierNext, self.xTurnInfoNext = turn_type_mapping[self.nTBTTurnTypeNext]
    else:
      self.navTypeNext, self.navModifierNext, self.xTurnInfoNext = "invalid", "", -1

    if self.nTBTDist > 0 and self.xTurnInfo > 0:
      self.xDistToTurn = self.nTBTDist
    if self.nTBTDistNext > 0 and self.xTurnInfoNext > 0:
      self.xDistToTurnNext = self.nTBTDistNext + self.nTBTDist

  def _get_sdi_descr(self, nSdiType):
    # 多语言映射：ko（韩语，原始），zh（简体中文），en（英文）。
    sdi_ko = {
        0: "신호과속",
        1: "과속 (고정식)",
        2: "구간단속 시작",
        3: "구간단속 끝",
        4: "구간단속중",
        5: "꼬리물기단속카메라",
        6: "신호 단속",
        7: "과속 (이동식)",
        8: "고정식 과속위험 구간(박스형)",
        9: "버스전용차로구간",
        10: "가변 차로 단속",
        11: "갓길 감시 지점",
        12: "끼어들기 금지",
        13: "교통정보 수집지점",
        14: "방범용cctv",
        15: "과적차량 위험구간",
        16: "적재 불량 단속",
        17: "주차단속 지점",
        18: "일방통행도로",
        19: "철길 건널목",
        20: "어린이 보호구역(스쿨존 시작 구간)",
        21: "어린이 보호구역(스쿨존 끝 구간)",
        22: "과속방지턱",
        23: "lpg충전소",
        24: "터널 구간",
        25: "휴게소",
        26: "톨게이트",
        27: "안개주의 지역",
        28: "유해물질 지역",
        29: "사고다발",
        30: "급커브지역",
        31: "급커브구간1",
        32: "급경사구간",
        33: "야생동물 교통사고 잦은 구간",
        34: "우측시야불량지점",
        35: "시야불량지점",
        36: "좌측시야불량지점",
        37: "신호위반다발구간",
        38: "과속운행다발구간",
        39: "교통혼잡지역",
        40: "방향별차로선택지점",
        41: "무단횡단사고다발지점",
        42: "갓길 사고 다발 지점",
        43: "과속 사발 다발 지점",
        44: "졸음 사고 다발 지점",
        45: "사고다발지점",
        46: "보행자 사고다발지점",
        47: "차량도난사고 상습발생지점",
        48: "낙석주의지역",
        49: "결빙주의지역",
        50: "병목지점",
        51: "합류 도로",
        52: "추락주의지역",
        53: "지하차도 구간",
        54: "주택밀집지역(교통진정지역)",
        55: "인터체인지",
        56: "분기점",
        57: "휴게소(lpg충전가능)",
        58: "교량",
        59: "제동장치사고다발지점",
        60: "중앙선침범사고다발지점",
        61: "통행위반사고다발지점",
        62: "목적지 건너편 안내",
        63: "졸음 쉼터 안내",
        64: "노후경유차단속",
        65: "터널내 차로변경단속",
        66: "",
    }

    sdi_en = {
        0: "Signal speed enforcement",
        1: "Speed camera (fixed)",
        2: "Section control start",
        3: "Section control end",
        4: "Under section control",
        5: "Block-the-box camera",
        6: "Signal violation enforcement",
        7: "Speed camera (mobile)",
        8: "Fixed speed camera zone (box)",
        9: "Bus-only lane zone",
        10: "Reversible/variable lane enforcement",
        11: "Shoulder surveillance point",
        12: "No cut-in",
        13: "Traffic data collection point",
        14: "Security CCTV",
        15: "Overloaded vehicle risk zone",
        16: "Improper loading enforcement",
        17: "Parking enforcement point",
        18: "One-way road",
        19: "Railroad crossing",
        20: "School zone start",
        21: "School zone end",
        22: "Speed bump",
        23: "LPG station",
        24: "Tunnel section",
        25: "Rest area",
        26: "Toll gate",
        27: "Fog caution area",
        28: "Hazardous materials area",
        29: "Accident-prone section",
        30: "Sharp curve area",
        31: "Sharp curve section 1",
        32: "Steep slope section",
        33: "Wild animal crossing area",
        34: "Poor visibility (right)",
        35: "Poor visibility",
        36: "Poor visibility (left)",
        37: "Frequent signal violations",
        38: "Frequent speeding",
        39: "Traffic congestion area",
        40: "Lane selection by direction",
        41: "Frequent jaywalking accidents",
        42: "Frequent shoulder accidents",
        43: "Frequent speeding accidents",
        44: "Frequent drowsy driving accidents",
        45: "Accident-prone spot",
        46: "Frequent pedestrian accidents",
        47: "Frequent vehicle theft",
        48: "Falling rock caution area",
        49: "Icy road caution area",
        50: "Bottleneck point",
        51: "Merging road",
        52: "Cliff/Drop caution area",
        53: "Underpass section",
        54: "Residential area (traffic calming)",
        55: "Interchange",
        56: "Junction",
        57: "Rest area (LPG available)",
        58: "Bridge",
        59: "Frequent brake failure accidents",
        60: "Center line invasion accidents",
        61: "Violation-of-passage accidents",
        62: "Destination on opposite side",
        63: "Drowsy rest area",
        64: "Old diesel control",
        65: "Lane change enforcement in tunnel",
        66: "",
    }

    sdi_zh = {
        0: "信号测速/闯灯拍照",
        1: "固定测速摄像头",
        2: "区间测速开始",
        3: "区间测速结束",
        4: "区间测速中",
        5: "路口压线摄像头",
        6: "闯红灯拍照",
        7: "流动测速摄像头",
        8: "测速拍照",
        9: "公交专用车道区间",
        10: "可变/潮汐车道拍照",
        11: "应急车道拍照",
        12: "禁止加塞",
        13: "交通信息采集点",
        14: "治安监控",
        15: "超载车辆风险区",
        16: "装载不当拍照",
        17: "违停拍照点",
        18: "单行道",
        19: "铁路道口",
        20: "学校区域开始",
        21: "学校区域结束",
        22: "减速带",
        23: "LPG加气站",
        24: "隧道区间",
        25: "服务区",
        26: "ETC计费拍照",
        27: "多雾路段",
        28: "危险品区域",
        29: "事故多发路段",
        30: "急弯路段",
        31: "急弯区段1",
        32: "陡坡路段",
        33: "野生动物出没路段",
        34: "右侧视野不良点",
        35: "视野不良点",
        36: "左侧视野不良点",
        37: "闯红灯多发",
        38: "超速多发",
        39: "交通拥堵区域",
        40: "按方向选择车道点",
        41: "行人乱穿马路多发处",
        42: "应急车道事故多发",
        43: "超速事故多发",
        44: "疲劳驾驶事故多发",
        45: "事故多发点",
        46: "行人事故多发点",
        47: "车辆盗窃多发点",
        48: "落石危险路段",
        49: "路面结冰危险",
        50: "瓶颈路段",
        51: "汇入道路",
        52: "坠落危险路段",
        53: "地下车道区间",
        54: "居民区（交通缓和）",
        55: "立交",
        56: "分岔点",
        57: "服务区（可加气）",
        58: "桥梁",
        59: "制动故障事故多发点",
        60: "越线事故多发点",
        61: "违法通行事故多发点",
        62: "目的地在对面",
        63: "瞌睡停车区",
        64: "老旧柴油车管制",
        65: "隧道内变道拍照",
        66: "",
    }

    sdi_map = sdi_en
    if self.lang == "ko":
      sdi_map = sdi_ko
    elif self.lang == "zh":
      sdi_map = sdi_zh

    return sdi_map.get(nSdiType, "")

  def _update_sdi(self):
    #sdiBlockType
    # 1: startOSEPS: 구간단속시작
    # 2: inOSEPS: 구간단속중
    # 3: endOSEPS: 구간단속종료
    # 0:감속안함,1:과속카메라,2:+사고방지턱,3:+이동식카메라
    if self.nSdiType in [0,1,2,3,4,7,8, 75, 76] and self.nSdiSpeedLimit > 0 and self.autoNaviSpeedCtrlMode > 0:
      self.xSpdLimit = self.nSdiSpeedLimit * self.autoNaviSpeedSafetyFactor
      self.xSpdDist = self.nSdiDist
      self.xSpdType = self.nSdiType
      if self.nSdiBlockType in [2,3]:
        self.xSpdDist = self.nSdiBlockDist
        self.xSpdType = 4
      elif self.nSdiType == 7 and self.autoNaviSpeedCtrlMode < 3: #이동식카메라
        self.xSpdLimit = self.xSpdDist = 0
    elif (self.nSdiPlusType == 22 or self.nSdiType == 22) and self.roadcate > 1 and self.autoNaviSpeedCtrlMode >= 2: # speed bump, roadcate:0,1: highway
      self.xSpdLimit = self.autoNaviSpeedBumpSpeed
      self.xSpdDist = self.nSdiPlusDist if self.nSdiPlusType == 22 else self.nSdiDist
      self.xSpdType = 22
    else:
      self.xSpdLimit = 0
      self.xSpdType = -1
      self.xSpdDist = 0

  def _update_gps(self, v_ego, sm, gps_service):
    gps = sm[gps_service]
    #print(f"location = {sm.valid[llk]}, {sm.updated[llk]}, {sm.recv_frame[llk]}, {sm.recv_time[llk]}")
    if not sm.updated['carState'] or not sm.updated['carControl']: # or not sm.updated[llk]:
      return self.nPosAngle
    CS = sm['carState']
    CC = sm['carControl']
    self.gps_valid = sm.updated[gps_service] and gps.hasFix

    now = time.monotonic()
    gps_updated_phone = (now - self.last_update_gps_time_phone) < 3
    # [169차 계측] gps_updated_navi가 참조하는 것과 동일한 경과시간을
    # 별도 보관 -- cereal로 발행해 실차 로그에서 "패킷단절"(이 값이 3.0
    # 초과) 여부를 직접 관측하기 위함(FINDINGS.md 169차).
    self.dt_navi_packet_age = now - self.last_update_gps_time_navi
    gps_updated_navi = self.dt_navi_packet_age < 3

    bearing = self.nPosAngle
    if gps_updated_navi:
      bearing = self.nPosAngle
    elif gps_updated_phone:
      bearing = self.nPosAnglePhone
    elif self.gps_valid:
      bearing = self.nPosAngle = gps.bearingDeg

    self.bearing_offset = 0.0

    # [166차] CC.orientationNED[2] 델타앵커링 헤딩보정 (TODO 해결, 방향1)
    # - 절대값 대입이 아니라 마지막 fix 시점 기준 상대회전(Δyaw)만 반영해
    #   locationd 드리프트에 안전하게 설계 (FINDINGS.md 165/166차 참고).
    # - "새 fix 도착"은 last_calculate_gps_time 변화로 간접 감지.
    heading_correction_deg = 0.0
    ned = list(CC.orientationNED)
    cc_pose_valid = len(ned) > 2
    # [167차] 163차 위치불확실성 게이트(방향2)가 "방향1이 무력화되는
    # 폴백 구간에서만" 발동하도록 좁히기 위해 노출.
    self.cc_pose_valid = cc_pose_valid
    if cc_pose_valid:
      cc_yaw_now = ned[2]
      if self.last_calculate_gps_time != self._prev_fix_time_for_heading:  # 새 fix 도착
        self.cc_yaw_at_fix = cc_yaw_now
      self._prev_fix_time_for_heading = self.last_calculate_gps_time
      if self.cc_yaw_at_fix is not None:
        dyaw = (cc_yaw_now - self.cc_yaw_at_fix + math.pi) % (2 * math.pi) - math.pi
        heading_correction_deg = math.degrees(dyaw)

    #print(f"bearing = {bearing:.1f}, posA=={self.nPosAngle:.1f}, posP=={self.nPosAnglePhone:.1f}, offset={self.bearing_offset:.1f}, {gps_updated_phone}, {gps_updated_navi}")
    gpsDelayTimeAdjust = 0.0
    if gps_updated_navi:
      gpsDelayTimeAdjust = 0 #1.0

    external_gps_update_timedout = not (gps_updated_phone or gps_updated_navi)
    #print(f"gps_valid = {self.gps_valid}, bearing = {bearing:.1f}, pos = {location.positionGeodetic.value[0]:.6f}, {location.positionGeodetic.value[1]:.6f}")
    if self.gps_valid and external_gps_update_timedout:    # 내부GPS가 작동하고 carrotman으로부터 gps신호가 없는경우
      self.vpPosPointLatNavi = gps.latitude
      self.vpPosPointLonNavi = gps.longitude
      self.last_calculate_gps_time = now #sm.recv_time[llk]
    elif gps_updated_navi:  # carrot navi로부터 gps신호가 수신되는 경우..
      if abs(self.bearing_measured - bearing) < 0.1:
          self.diff_angle_count += 1
      else:
          self.diff_angle_count = 0
      self.bearing_measured = bearing

      if self.diff_angle_count > 5: # 조향각도변화가 거의 없을때만 업데이트
        diff_angle = (self.nPosAngle - bearing) % 360
        if diff_angle > 180:
          diff_angle -= 360
        self.bearing_offset = self.bearing_offset * 0.9 + diff_angle * 0.1

    bearing_calculated = (bearing + self.bearing_offset + heading_correction_deg) % 360

    dt = now - self.last_calculate_gps_time
    # [162차] carrot_navi_route()의 route_speed 램프리미터 위치불확실성
    # 게이트가 읽는 값 -- 실제 fix 이후 데드레커닝만으로 흘러온 시간.
    self.position_dt_since_fix = dt
    #print(f"dt = {dt:.1f}, {self.vpPosPointLatNavi}, {self.vpPosPointLonNavi}")
    if dt > 5.0:
      self.vpPosPointLat, self.vpPosPointLon = 0.0, 0.0
    elif dt == 0:
      self.vpPosPointLat, self.vpPosPointLon = self.vpPosPointLatNavi, self.vpPosPointLonNavi
    else:
      self.vpPosPointLat, self.vpPosPointLon = self.estimate_position(float(self.vpPosPointLatNavi), float(self.vpPosPointLonNavi), v_ego, bearing_calculated, dt + gpsDelayTimeAdjust)

    #self.debugText = " {} {:.1f},{:.1f}={:.1f}+{:.1f}".format(self.active_sdi_count, self.nPosAngle, bearing_calculated, bearing, self.bearing_offset)
    #print("nPosAngle = {:.1f},{:.1f} = {:.1f}+{:.1f}".format(self.nPosAngle, bearing_calculated, bearing, self.bearing_offset))

    return float(bearing_calculated)


  def estimate_position(self, lat, lon, speed, angle, dt):
    R = 6371000
    angle_rad = math.radians(angle)
    delta_d = speed * dt
    delta_lat = delta_d * math.cos(angle_rad) / R
    new_lat = lat + math.degrees(delta_lat)
    delta_lon = delta_d * math.sin(angle_rad) / (R * math.cos(math.radians(lat)))
    new_lon = lon + math.degrees(delta_lon)

    return new_lat, new_lon

  def update_auto_turn(self, v_ego_kph, sm, x_turn_info, x_dist_to_turn, check_steer=False):
    turn_speed = self.autoTurnControlSpeedTurn
    fork_speed = self.nRoadLimitSpeed
    stop_speed = 1
    turn_dist_for_speed = self.autoTurnControlTurnEnd * turn_speed / 3.6 # 5
    fork_dist_for_speed = self.autoTurnControlTurnEnd * fork_speed / 3.6 # 5
    stop_dist_for_speed = 5
    start_fork_dist = np.interp(self.nRoadLimitSpeed, [30, 50, 100], [160, 200, 350])
    start_turn_dist = np.interp(self.nTBTNextRoadWidth, [5, 10], [43, 60])
    turn_info_mapping = {
        1: {"type": "turn left", "speed": turn_speed, "dist": turn_dist_for_speed, "start": start_fork_dist},
        2: {"type": "turn right", "speed": turn_speed, "dist": turn_dist_for_speed, "start": start_fork_dist},
        5: {"type": "straight", "speed": turn_speed, "dist": turn_dist_for_speed, "start": start_turn_dist},
        3: {"type": "fork left", "speed": fork_speed, "dist": fork_dist_for_speed, "start": start_fork_dist},
        4: {"type": "fork right", "speed": fork_speed, "dist": fork_dist_for_speed, "start": start_fork_dist},
        6: {"type": "straight", "speed": fork_speed, "dist": fork_dist_for_speed, "start": start_fork_dist},
        7: {"type": "straight", "speed": stop_speed, "dist": stop_dist_for_speed, "start": 1000},
        8: {"type": "straight", "speed": stop_speed, "dist": stop_dist_for_speed, "start": 1000},
    }

    default_mapping = {"type": "none", "speed": 0, "dist": 0, "start": 1000}

    mapping = turn_info_mapping.get(x_turn_info, default_mapping)

    atc_type = mapping["type"]
    atc_speed = mapping["speed"]
    atc_dist = mapping["dist"]
    atc_start_dist = mapping["start"]

    if x_dist_to_turn > atc_start_dist:
      atc_type += " prepare"
      if check_steer:
        self.atc_activate_count = min(0, self.atc_activate_count - 1)
    else:
      if check_steer:
        self.atc_activate_count = max(0, self.atc_activate_count + 1)
      if atc_type in ["turn left", "turn right"] and x_dist_to_turn > start_turn_dist:
        atc_type = "atc left" if atc_type == "turn left" else "atc right"

    if self.autoTurnMapChange > 0 and check_steer:
      #print(f"x_dist_to_turn: {x_dist_to_turn}, atc_start_dist: {atc_start_dist}")
      #print(f"atc_activate_count: {self.atc_activate_count}")
      if self.atc_activate_count == 2:
        self.carrotCmdIndex += 100
        self.carrotCmd = "DISPLAY";
        self.carrotArg = "MAP";
      elif self.atc_activate_count == -50:
        self.carrotCmdIndex += 100
        self.carrotCmd = "DISPLAY";
        self.carrotArg = "ROAD";

    if check_steer:
      if 0 <= x_dist_to_turn < atc_start_dist and atc_type in ["fork left", "fork right"]:
        if not self.atc_paused:
          steering_pressed = sm["carState"].steeringPressed
          steering_torque = sm["carState"].steeringTorque
          if steering_pressed and steering_torque < 0 and atc_type in ["fork left", "atc left"]:
            self.atc_paused = True
          elif steering_pressed and steering_torque > 0 and atc_type in ["fork right", "atc right"]:
            self.atc_paused = True
      else:
        self.atc_paused = False

      if self.atc_paused:
        atc_type += " canceled"

    atc_desired = 250
    if atc_speed > 0 and x_dist_to_turn > 0:
      decel = self.autoNaviSpeedDecelRate
      safe_sec = 2.0
      atc_desired = min(atc_desired, self.calculate_current_speed(x_dist_to_turn - atc_dist, atc_speed, safe_sec, decel))


    return atc_desired, atc_type, atc_speed, atc_dist

  def update_nav_instruction(self, sm):
    if sm.alive['navInstruction'] and sm.valid['navInstruction']:
      msg_nav = sm['navInstruction']

      self.nGoPosDist = int(msg_nav.distanceRemaining)
      self.nGoPosTime = int(msg_nav.timeRemaining)
      if self.active_kisa_count <= 0 and msg_nav.speedLimit > 0:
        self.nRoadLimitSpeed = max(30, round(msg_nav.speedLimit * CV.MS_TO_KPH))
      self.xDistToTurn = int(msg_nav.maneuverDistance)
      self.szTBTMainText = msg_nav.maneuverPrimaryText
      self.xTurnInfo = -1
      for key, value in nav_type_mapping.items():
        if value[0] == msg_nav.maneuverType and value[1] == msg_nav.maneuverModifier:
          self.xTurnInfo = value[2]
          break

      self.debugText = f"{self.nRoadLimitSpeed if self.is_metric else self.nRoadLimitSpeed * CV.KPH_TO_MPH:.0f},{msg_nav.maneuverType},{msg_nav.maneuverModifier} "
      #print(msg_nav)
      #print(f"navInstruction: {self.xTurnInfo}, {self.xDistToTurn}, {self.szTBTMainText}")

  def update_kisa(self, data):
    self.active_kisa_count = 100
    if "kisawazecurrentspd" in data:
      pass
    if "kisawazeroadspdlimit" in data:
      road_limit_speed = data["kisawazeroadspdlimit"]
      if road_limit_speed > 0:
        print(f"kisawazeroadspdlimit: {road_limit_speed} km/h")
        if not self.is_metric:
          road_limit_speed *= CV.MPH_TO_KPH
        self.nRoadLimitSpeed = road_limit_speed
    if "kisawazealert" in data:
      pass
    if "kisawazeendalert" in data:
      pass
    if "kisawazeroadname" in data:
      print(f"kisawazeroadname: {data['kisawazeroadname']}")
      self.szPosRoadName = data["kisawazeroadname"]
    if "kisawazereportid" in data and "kisawazealertdist" in data:
      id_str = data["kisawazereportid"]
      dist_str = data["kisawazealertdist"].lower()
      import re
      match = re.search(r'(\d+)', dist_str)
      distance = int(match.group(1)) if match else 0
      if not self.is_metric:
        distance = int(distance * 0.3048)
      print(f"{id_str}: {distance} m")
      xSpdType = -1
      if 'camera' in id_str:
        xSpdType = 101    # 101: waze speed cam, 100: police
      elif 'police' in id_str:
        xSpdType = 100

      if xSpdType >= 0:
        offset = 5 if self.is_metric else 5 * CV.MPH_TO_KPH
        self.xSpdLimit = self.nRoadLimitSpeed + offset

        self.xSpdDist = distance
        self.xSpdType =xSpdType

  def update_navi(self, remote_ip, sm, pm, vturn_speed, coords, distances, route_speed, gps_service,
                   navi_points_active=False, navd_active=False, dt_route_inactive=0.0, navi_route_source=""):

    self.debugText = ""
    self.update_params()
    if sm.alive['carState'] and sm.alive['selfdriveState']:
      CS = sm['carState']
      v_ego = CS.vEgo
      v_ego_kph = v_ego * 3.6
      distanceTraveled = sm['selfdriveState'].distanceTraveled
      delta_dist = distanceTraveled - self.totalDistance
      self.totalDistance = distanceTraveled
      if CS.speedLimit > 0 and self.active_carrot <= 1:
        self.nRoadLimitSpeed = CS.speedLimit
    else:
      v_ego = v_ego_kph = 0
      delta_dist = 0
      CS = None

    road_speed_limit_changed = True if self.nRoadLimitSpeed != self.nRoadLimitSpeed_last else False
    self.nRoadLimitSpeed_last = self.nRoadLimitSpeed
    #self.bearing = self.nPosAngle #self._update_gps(v_ego, sm)
    self.bearing = self._update_gps(v_ego, sm, gps_service)

    self.xSpdDist = max(self.xSpdDist - delta_dist, -1000)
    self.xDistToTurn = self.xDistToTurn - delta_dist
    self.xDistToTurnNext = self.xDistToTurnNext - delta_dist
    self.active_count = max(self.active_count - 1, 0)
    self.active_sdi_count = max(self.active_sdi_count - 1, 0)
    self.active_kisa_count = max(self.active_kisa_count - 1, 0)
    if self.active_kisa_count > 0:
      self.active_carrot = 2

    elif self.active_count > 0:
      self.active_carrot = 2 if self.active_sdi_count > 0 else 1
    else:
      self.active_carrot = 0

    limit_speed = 200
    if self.autoRoadSpeedLimitOffset >= 0 and self.active_carrot>=2:
      if self.nRoadLimitSpeed >= 30:
        road_speed_limit_offset = self.autoRoadSpeedLimitOffset
        if not self.is_metric:
          road_speed_limit_offset *= CV.KPH_TO_MPH
        limit_speed = self.nRoadLimitSpeed + road_speed_limit_offset

    if self.active_carrot <= 1:
      self.xSpdType = self.navType = self.xTurnInfo = self.xTurnInfoNext = -1
      self.nSdiType = self.nSdiBlockType = self.nSdiPlusBlockType = -1
      self.nTBTTurnType = self.nTBTTurnTypeNext = -1
      self.roadcate = 8
      self.nGoPosDist = 0
    if self.active_carrot <= 1 or self.active_kisa_count > 0:
      self.update_nav_instruction(sm)

    if self.xSpdType < 0 or (self.xSpdType not in [100,101] and self.xSpdDist <= 0) or (self.xSpdType in [100,101] and self.xSpdDist < -250):
      self.xSpdType = -1
      self.xSpdDist = self.xSpdLimit = 0
    if self.xTurnInfo < 0 or self.xDistToTurn < -50:
      if self.xDistToTurn > 0:
        self.xDistToTurn = 0
      self.xTurnInfo = -1
      self.xDistToTurnNext = 0
      self.xTurnInfoNext = -1

    sdi_speed = 250
    hda_active = False
    ### 과속카메라, 사고방지턱
    if (self.xSpdDist > 0 or self.xSpdType in [100, 101]) and self.active_carrot > 0:
      safe_sec = self.autoNaviSpeedBumpTime if self.xSpdType == 22 else self.autoNaviSpeedCtrlEnd
      decel = self.autoNaviSpeedDecelRate
      sdi_speed = min(sdi_speed, self.calculate_current_speed(self.xSpdDist, self.xSpdLimit, safe_sec, decel))
      self.active_carrot = 5 if self.xSpdType == 22 else 3
      if self.xSpdType == 4 or (self.xSpdType in [100, 101] and self.xSpdDist <= 0):
        sdi_speed = self.xSpdLimit
        self.active_carrot = 4
    elif CS is not None and CS.speedLimit > 0 and CS.speedLimitDistance > 0:
      sdi_speed = min(sdi_speed,
                      self.calculate_current_speed(CS.speedLimitDistance,
                                                   CS.speedLimit * self.autoNaviSpeedSafetyFactor,
                                                   self.autoNaviSpeedCtrlEnd,
                                                   self.autoNaviSpeedDecelRate))
      #self.active_carrot = 6
      hda_active = True

    #print(f"sdi_speed: {sdi_speed}, hda_active: {hda_active}, xSpdType: {self.xSpdType}, xSpdDist: {self.xSpdDist}, active_carrot: {self.active_carrot}, v_ego_kph: {v_ego_kph}, nRoadLimitSpeed: {self.nRoadLimitSpeed}")
    ### TBT 속도제어
    atc_desired, self.atcType, self.atcSpeed, self.atcDist = self.update_auto_turn(v_ego*3.6, sm, self.xTurnInfo, self.xDistToTurn, True)
    atc_desired_next, _, _, _ = self.update_auto_turn(v_ego*3.6, sm, self.xTurnInfoNext, self.xDistToTurnNext, False)

    if self.nSdiType  >= 0: # or self.active_carrot > 0:
      pass
      # self.debugText = (
      #   f"Atc:{atc_desired:.1f}," +
      #   f"{self.xTurnInfo}:{self.xDistToTurn:.1f}, " +
      #   f"I({self.nTBTNextRoadWidth},{self.roadcate}) " +
      #   f"Atc2:{atc_desired_next:.1f}," +
      #   f"{self.xTurnInfoNext},{self.xDistToTurnNext:.1f}"
      # )
      #self.debugText = "" #f" {self.nSdiType}/{self.nSdiSpeedLimit}/{self.nSdiDist},BLOCK:{self.nSdiBlockType}/{self.nSdiBlockSpeed}/{self.nSdiBlockDist}, PLUS:{self.nSdiPlusType}/{self.nSdiPlusSpeedLimit}/{self.nSdiPlusDist}"
    #elif self.nGoPosDist > 0 and self.active_carrot > 1:
    #  self.debugText = " 목적지:{:.1f}km/{:.1f}분 남음".format(self.nGoPosDist/1000., self.nGoPosTime / 60)
    else:
      #self.debugText = ""
      pass

    if self.autoTurnControl not in [2, 3]:    # auto turn speed control
      atc_desired = atc_desired_next = 250

    if self.autoTurnControl not in [1,2]:    # auto turn control
      self.atcType = "none"


    speed_n_sources = [
      (atc_desired, "atc"),
      (atc_desired_next, "atc2"),
      (sdi_speed, "hda" if hda_active else "bump" if self.xSpdType == 22 else "section" if self.xSpdType == 4 else "police" if self.xSpdType == 100 else "waze" if self.xSpdType == 101 else "cam"),
      (limit_speed, "road"),
    ]
    if self.turnSpeedControlMode in [1,2]:
      speed_n_sources.append((max(abs(vturn_speed), self.autoCurveSpeedLowerLimit), "vturn"))

    # [210차, 사용자 실차 스크린샷 제보 대응 -- 삭제] 기존엔 여기서
    # route_speed(carrot_man.py::carrot_navi_route()가 205/207차에서
    # "max(v_ego_kph, sharpest_candidate_speed)"로 vEgo 상한을 이미 적용해
    # 반환한 값)에 mapTurnSpeedFactor(사용자 실측 설정값 1.30, PARAMS_REGISTRY.md
    # 201차)를 다시 곱하고 있었다. 곱셈이 vEgo 상한 *이후*에 걸리기 때문에,
    # 205/207차가 만든 "route는 현재속도보다 빠른 속도를 권하지 않는다"는
    # 불변식이 최종 출력(HUD route= 표시값, arbitration 후보값) 단계에서는
    # 성립하지 않았다 -- 210차 실차 스크린샷(vEgo=53, route=62.7,
    # 62.7/1.30=48.2로 vEgo 상한 적용 전 raw 값과 일치)으로 실측 확인.
    # 사용자 판단(210차)으로 이 배율 자체를 완전히 제거 -- MapTurnSpeedFactor는
    # 이 곱셈이 repo 내 유일한 사용처였으므로(라인 304 주석 참고) route
    # 감속에 대해서는 이제 관여하지 않는다. autoCurveSpeedLowerLimit 바닥은
    # 그대로 유지(route가 이 값 밑으로는 내려가지 않는 기존 하한).
    # [223차, design doc §2/§19, STEP3 결론] route_speed가 None이면 route가
    # 이번 프레임 비활성(mode 0/1, RELEASE 2초 hold 중, 직선, 또는 ACTIVE
    # 진입조건 미충족)이라는 뜻 -- carrot_man.py::carrot_navi_route()가
    # 명시적으로 반환한 값이다. 과거(211~221차)엔 이 경우도 150(=
    # ROUTE_MAX_SPEED_KPH) sentinel을 그대로 append해서, "150이 다른 후보
    # 보다 항상 큰가"라는 암묵적 가정에 안전을 의존했다(§19가 우려한 "route
    # source가 이전 값을 붙잡는 현상"의 원인 중 하나). 이제는 None이면 애초에
    # speed_n_sources 후보에 넣지 않아 min() 경쟁에서 구조적으로 제외한다 --
    # 다른 소스가 150을 넘는 비정상 상황에서도 안전하다.
    if route_speed is not None:
      # [224차, "224차 Route 로직 수정 지침" §5, 버그 수정] 224차
      # carrot_man.py::carrot_navi_route() ceiling-fix(§2/§4 -- route는
      # vEgo를 넘지 않는 상한이지 가속 목표가 아님)로 route_speed(=이 시점
      # 이미 ceiling-limited out_speed)는 항상 v_ego_kph 이하로 보장된다.
      # 그런데 구코드는 여기서 autoCurveSpeedLowerLimit(기본 30kph, 사용자
      # 조절범위 30~200) 바닥을 무조건 적용해, route_speed(예: 정지 중
      # ceiling-fix에 의해 0)를 30kph로 다시 밀어올렸다 -- 224차
      # carrot_man.py 수정이 막 없앤 "route가 vEgo보다 높은 값을 출력"
      # 버그를 이 한 줄이 그대로 재현시키는 구조였다(정지 중이 아니어도
      # v_ego_kph < autoCurveSpeedLowerLimit인 저속 구간이면 동일하게 재현
      # 가능). autoCurveSpeedLowerLimit 자체의 원래 목적(곡률 계산 노이즈로
      # 목표속도가 비정상적으로 낮게 나오는 것을 막는 하한)은 유지하되,
      # v_ego_kph를 넘어서까지 끌어올리지는 않도록 상한을 다시 씌운다
      # (최소변경 원칙, §27 -- 이 줄만 수정, 바닥값 자체나 다른 소스는
      # 그대로 둠). 검증: toolkit/sim_route_224_serv_floor_fix.py.
      # [227차] 위 self.route_active 설명 참고 -- ACTIVE 추적 분기(True)만
      # vEgo 상한 클램프를 적용한다. ACTIVE 진입 게이트 ceiling 분기(False,
      # 226차가 out_speed=apex_speed를 반환하는 경우)는 apex_speed 자체가
      # 이미 "이 값을 넘지 말라"는 상한이므로, vEgo로 다시 잘라내면
      # ceiling이 아니라 vEgo 고착(가속 명령 원천 봉쇄)이 되어버린다(224차
      # ceiling-fix가 보장한 "route_speed<=vEgo"는 ACTIVE 분기에서만 성립하는
      # 증명이었음 -- 225차가 이미 한 번 정정한 것과 같은 종류의 일반화
      # 오류). False일 때는 autoCurveSpeedLowerLimit 하한만 유지(§27 최소
      # 변경 -- 하한 목적 자체는 두 분기 모두 동일하게 필요).
      # [228차, route_inert v2, FINDINGS.md 228차] route_active=True이면서도
      # route_inert=True(far-inert -- 아직 거리는 남았지만 vEgo가 이미
      # target 이하)인 프레임에는 vEgo 상한 클램프를 생략한다. 227차는
      # route_active만으로 분기했으나, ACTIVE 추적 중 vEgo가 완전히 0까지
      # 떨어지면 이 클램프가 carrot_man이 무엇을 계산해 넘기든
      # route_speed를 다시 실측 v_ego로 눌러버려, 정차 원인이 해소된 뒤에도
      # route_speed가 0에서 벗어나지 못하는 자기참조적 고착이 발생했다
      # (결함이 carrot_man.py 단독이 아니라 이 클램프까지 두 파일에 걸쳐
      # 있었음). route_inert=False(eff_dist<=0 apex 근접 구간 포함, 224차
      # 의도 보존) 및 route_active=False(ceiling 분기)에서는 기존 동작
      # 그대로 유지.
      if self.route_active and not self.route_inert:
        route_speed = min(v_ego_kph, max(route_speed, self.autoCurveSpeedLowerLimit))
      else:
        route_speed = max(route_speed, self.autoCurveSpeedLowerLimit)
      if self.turnSpeedControlMode in [2, 3, 4]:
        # 81차: mode 2의 -500<xDistToTurn<500(TBT 회전지점 근접) 게이트를 제거.
        # 기존엔 TBT 안내가 없는 일반 도로 굽이길에서 route_speed가 계산은 되고도
        # 후보에서 빠져 vturn 단독으로만 대응하던 사각지대가 있었음(mode 2도 mode
        # 3/4처럼 항상 참가하도록 통일). vturn 참가 조건(위 [1,2] 분기)은 그대로라
        # mode 2에서 vturn+route가 함께 경쟁하는 구조가 됨(mode 3/4는 기존대로
        # vturn 자체가 미참가라 route 단독).
        speed_n_sources.append((route_speed, "route"))
        #speed_n_sources.append((self.calculate_current_speed(dist, speed * self.mapTurnSpeedFactor, 0, 1.2), "route"))

    model_turn_speed = max(sm['modelV2'].meta.modelTurnSpeed, self.autoCurveSpeedLowerLimit)

    # model_turn_speed 자기 자신의 추세로 "트레일링(커브를 이미 빠져나와
    # 복귀 중)" 여부를 판단 (위 __init__ 주석 참고, 11차 도입 / 50차
    # min_recent+margin 방식으로 재설계).
    # "최근 확인된 최저점(min_recent) 대비 recover_margin km/h 이상 지속
    # 회복"일 때만 트레일링으로 확정한다. 완만한 하강 추세 중의 프레임간
    # 잔떨림(상승 튐)은 margin 안이라 min_recent를 갱신 못해도 카운터가
    # 리셋되지 않고, 실제로 커브를 빠져나와 뚜렷하게 반등하는 경우만
    # hold_sec 동안 margin을 초과 유지해 트레일링으로 잡힌다.
    if self.model_turn_speed_min_recent is None:
      self.model_turn_speed_min_recent = model_turn_speed

    if model_turn_speed <= self.model_turn_speed_min_recent + self.model_turn_recover_margin:
      self.model_turn_straight_count = 0
      self.model_turn_speed_min_recent = min(self.model_turn_speed_min_recent, model_turn_speed)
    else:
      self.model_turn_straight_count += 1
    # carrot_man 브로드캐스트 루프 주기와 동일 (Ratekeeper(20) -> 0.05s/frame)
    model_turn_confirmed_trailing = (self.model_turn_straight_count * 0.05) >= self.model_turn_straight_hold_sec
    if model_turn_confirmed_trailing:
      # 트레일링 확정 -- 다음 커브 사이클을 위해 기준선을 현재값으로 리셋
      self.model_turn_speed_min_recent = model_turn_speed

    # [50차 제거] 기존 "abs(vturn_speed) < 120" 게이트는 vturn 원시값이
    # 원거리에서 극도로 불안정(부호까지 요동, -249~249 관측)해 실제로는
    # 커브가 다가오고 있어도 이 조건에 계속 걸려 model 후보 자체가 min()
    # 경쟁에서 배제되는 경우가 실측으로 확인됨(route1 203f99d429 seg8,
    # FINDINGS.md 50차 항목 -- vEgo가 11초간 가속하는데 model은 이미
    # 안정적으로 낮은 값을 유지하고도 배제됨). 트레일링 판정이 위에서
    # 재설계되어 자체적으로 "커브 접근 vs 이탈 후 복귀"를 구분하므로,
    # vturn 절대값과 무관하게 model_turn_speed<200(=유의미한 커브 감지)
    # 여부와 트레일링 여부만으로 참여를 결정한다.
    if model_turn_speed < 200 and not model_turn_confirmed_trailing:
      speed_n_sources.append((model_turn_speed, "model"))

    desired_speed, source = min(speed_n_sources, key=lambda x: x[0])

    if CS is not None:
      if source != self.source_last:
        self.gas_override_speed = 0
        self.gas_pressed_state = CS.gasPressed
      if CS.vEgo < 0.1 or desired_speed > 150 or source in ["cam", "section", "police"] or CS.brakePressed or road_speed_limit_changed:
        self.gas_override_speed = 0
      elif CS.gasPressed and not self.gas_pressed_state:
        self.gas_override_speed = max(v_ego_kph, self.gas_override_speed)
      else:
        self.gas_pressed_state = False
      self.source_last = source

      if desired_speed < self.gas_override_speed:
        source = "gas"
        desired_speed = self.gas_override_speed

      # [223차] route_speed가 None(route 비활성/미개입)일 수 있으므로 포맷 전 방어.
      self.debugText += f"route={route_speed:.1f}" if route_speed is not None else "route=off"

    left_spd_sec = 100
    left_tbt_sec = 100
    if self.autoNaviCountDownMode > 0:
      if self.xSpdType == 22 and self.autoNaviCountDownMode == 1: # speed bump
        pass
      else:
        if self.xSpdDist > 0:
          left_spd_sec = min(self.left_spd_sec, int(max(self.xSpdDist - v_ego, 1) / max(1, v_ego) + 0.5))

      if self.xDistToTurn > 0:
        left_tbt_sec = min(self.left_tbt_sec, int(max(self.xDistToTurn - v_ego, 1) / max(1, v_ego) + 0.5))

    self.left_spd_sec = left_spd_sec
    self.left_tbt_sec = left_tbt_sec

    left_sec = min(left_spd_sec, left_tbt_sec)

    if left_sec > 11:
      self.left_sec = 100
      self.max_left_sec = 100
    else:
      self.sdi_inform = True if source in ["cam", "hda"] else False
      self.max_left_sec = min(11, max(6, int(v_ego_kph/10) + 1))

    if left_sec != self.left_sec:
      if left_sec == self.max_left_sec and self.sdi_inform:
        self.carrot_left_sec = 11
      elif 1 <= left_sec < self.max_left_sec:
        self.carrot_left_sec = left_sec
      elif left_sec == 0 and self.left_sec == 1:
        self.carrot_left_sec = left_sec

      self.left_sec = left_sec


    self._update_cmd()
    msg = messaging.new_message('carrotMan')
    msg.valid = True
    msg.carrotMan.activeCarrot = self.active_carrot
    msg.carrotMan.nRoadLimitSpeed = int(self.nRoadLimitSpeed)
    msg.carrotMan.remote = remote_ip
    msg.carrotMan.xSpdType = int(self.xSpdType)
    msg.carrotMan.xSpdLimit = int(self.xSpdLimit)
    msg.carrotMan.xSpdDist = int(self.xSpdDist)
    msg.carrotMan.xSpdCountDown = int(left_spd_sec)
    msg.carrotMan.xTurnInfo = int(self.xTurnInfo)
    msg.carrotMan.xDistToTurn = int(self.xDistToTurn)
    msg.carrotMan.xTurnCountDown = int(left_tbt_sec)
    msg.carrotMan.atcType = self.atcType
    msg.carrotMan.vTurnSpeed = int(vturn_speed)
    # 154차: 도로명과 디버그(route=) 텍스트를 '\n'으로 구분해서 보냄.
    # UI(carrot.cc)에서 두 줄로 나눠 그리고, route= 뒤 숫자만 강조 표시하기 위함.
    # (기존엔 이어붙여서 한 줄 문자열로 보냈음 -> UI에서 줄바꿈 파싱 불가)
    msg.carrotMan.szPosRoadName = self.szPosRoadName + (("\n" + self.debugText) if self.debugText else "")
    msg.carrotMan.szTBTMainText = self.szTBTMainText
    msg.carrotMan.desiredSpeed = int(desired_speed)
    msg.carrotMan.desiredSource = source
    msg.carrotMan.carrotCmdIndex = int(self.carrotCmdIndex)
    msg.carrotMan.carrotCmd = self.carrotCmd
    msg.carrotMan.carrotArg = self.carrotArg
    msg.carrotMan.trafficState = self.traffic_state

    msg.carrotMan.xPosSpeed = float(v_ego_kph) #float(self.nPosSpeed)
    msg.carrotMan.xPosAngle = float(self.bearing)
    msg.carrotMan.xPosLat = float(self.vpPosPointLat)
    msg.carrotMan.xPosLon = float(self.vpPosPointLon)

    msg.carrotMan.nGoPosDist = self.nGoPosDist
    msg.carrotMan.nGoPosTime = self.nGoPosTime
    msg.carrotMan.szSdiDescr = self._get_sdi_descr(-1 if self.nSdiType == 0 and self.nSdiDist == 0 else self.nSdiType)

    #coords_str = ";".join([f"{x},{y}" for x, y in coords])
    coords_str = ";".join([f"{x:.2f},{y:.2f},{d:.2f}" for (x, y), d in zip(coords, distances, strict=False)])
    msg.carrotMan.naviPaths = coords_str

    msg.carrotMan.leftSec = int(self.carrot_left_sec)
    # [169차 계측] "패킷단절 vs 내용정지" 구분용(FINDINGS.md 169차 참고).
    msg.carrotMan.vpPosPointLatNavi = float(self.vpPosPointLatNavi)
    msg.carrotMan.vpPosPointLonNavi = float(self.vpPosPointLonNavi)
    msg.carrotMan.dtNaviPacketAge = float(self.dt_navi_packet_age)
    msg.carrotMan.positionDtSinceFix = float(self.position_dt_since_fix)
    msg.carrotMan.ccPoseValid = bool(self.cc_pose_valid)
    # [182차 계측] navi_points_active 드롭아웃 원인규명용 (FINDINGS.md 182차)
    msg.carrotMan.naviPointsActive = bool(navi_points_active)
    msg.carrotMan.navdActive = bool(navd_active)
    msg.carrotMan.dtRouteInactive = float(dt_route_inactive)
    msg.carrotMan.routeSource = str(navi_route_source)
    # [194차] route apex 진단 telemetry 실제 발행 (custom.capnp @38~@41).
    # carrot_man.py::carrot_navi_route()가 매 사이클 self.route_apex_* 에
    # 값을 써주므로 여기서는 그대로 msg에 담기만 한다.
    msg.carrotMan.routeApexIdx = int(self.route_apex_idx)
    msg.carrotMan.routeApexDist = float(self.route_apex_dist)
    msg.carrotMan.routeApexSpeed = float(self.route_apex_speed)
    msg.carrotMan.routeOutSpeed = float(self.route_out_speed)
    # [204차 계측, 203차 옵션1] candidate telemetry 실제 발행(custom.capnp @42~@51).
    # carrot_man.py::carrot_navi_route()가 매 사이클 self.route_candidate* 에
    # 값을 써주므로 여기서는 그대로 msg에 담기만 한다(위 apex와 동일 패턴).
    msg.carrotMan.routeCandidateCount = int(self.route_candidate_count)
    msg.carrotMan.routeCandidate0Idx = int(self.route_candidate0_idx)
    msg.carrotMan.routeCandidate0Dist = float(self.route_candidate0_dist)
    msg.carrotMan.routeCandidate0Speed = float(self.route_candidate0_speed)
    msg.carrotMan.routeCandidate1Idx = int(self.route_candidate1_idx)
    msg.carrotMan.routeCandidate1Dist = float(self.route_candidate1_dist)
    msg.carrotMan.routeCandidate1Speed = float(self.route_candidate1_speed)
    msg.carrotMan.routeCandidate2Idx = int(self.route_candidate2_idx)
    msg.carrotMan.routeCandidate2Dist = float(self.route_candidate2_dist)
    msg.carrotMan.routeCandidate2Speed = float(self.route_candidate2_speed)
    pm.send('carrotMan', msg)

    inst = messaging.new_message('navInstructionCarrot')
    if self.active_carrot > 1 and self.active_kisa_count <= 0:
      inst.valid = True

      instruction = inst.navInstructionCarrot
      instruction.distanceRemaining = self.nGoPosDist
      instruction.timeRemaining = self.nGoPosTime
      instruction.speedLimit = self.nRoadLimitSpeed / 3.6 if self.nRoadLimitSpeed > 0 else 0
      instruction.maneuverDistance = float(self.nTBTDist)
      instruction.maneuverSecondaryText = self.szNearDirName
      if self.szFarDirName and len(self.szFarDirName):
        instruction.maneuverSecondaryText += "[{}]".format(self.szFarDirName)
      instruction.maneuverPrimaryText = self.szTBTMainText
      instruction.timeRemainingTypical = self.nGoPosTime

      navType, navModifier, xTurnInfo1 = "invalid", "", -1
      if self.nTBTTurnType in nav_type_mapping:
        navType, navModifier, xTurnInfo1 = nav_type_mapping[self.nTBTTurnType]
      navTypeNext, navModifierNext, xTurnInfoNext = "invalid", "", -1
      if self.nTBTTurnTypeNext in nav_type_mapping:
        navTypeNext, navModifierNext, xTurnInfoNext = nav_type_mapping[self.nTBTTurnTypeNext]

      instruction.maneuverType = navType
      instruction.maneuverModifier = navModifier

      maneuvers = []
      if self.nTBTTurnType >= 0:
        maneuver = {}
        maneuver['distance'] = float(self.xDistToTurn)
        maneuver['type'] = navType
        maneuver['modifier'] = navModifier
        maneuvers.append(maneuver)
        if self.nTBTDistNext >= self.nTBTDist:
          maneuver = {}
          maneuver['distance'] = float(self.nTBTDistNext)
          maneuver['type'] = navTypeNext
          maneuver['modifier'] = navModifierNext
          maneuvers.append(maneuver)

      instruction.allManeuvers = maneuvers
    elif sm.alive['navInstruction'] and sm.valid['navInstruction']:
      inst.navInstructionCarrot = sm['navInstruction']

    pm.send('navInstructionCarrot', inst)

  def _update_system_time(self, epoch_time_remote, timezone_remote):
    epoch_time = int(time.time())
    if epoch_time_remote > 0:
      epoch_time_offset = epoch_time_remote - epoch_time
      print(f"epoch_time_offset = {epoch_time_offset}")
      if abs(epoch_time_offset) > 60:
        os.system(f"sudo timedatectl set-timezone {timezone_remote}")
        formatted_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(epoch_time_remote))
        print(f"Setting system time to: {formatted_time}")
        os.system(f'sudo date -s "{formatted_time}"')

  def set_time(self, epoch_time, timezone):
    import datetime
    new_time = datetime.datetime.utcfromtimestamp(epoch_time)
    localtime_path = "/data/etc/localtime"

    no_timezone = False
    try:
      if os.path.getsize(localtime_path) == 0:
        no_timezone = True
    except (FileNotFoundError, OSError):
      no_timezone = True

    diff = datetime.datetime.utcnow() - new_time
    if abs(diff) < datetime.timedelta(seconds=10) and not no_timezone:
      #print(f"Time diff too small: {diff}")
      return

    print(f"Setting time to {new_time}, diff={diff}")
    zoneinfo_path = f"/usr/share/zoneinfo/{timezone}"
    if os.path.exists(localtime_path) or os.path.islink(localtime_path):
        try:
            subprocess.run(["sudo", "rm", "-f", localtime_path], check=True)
            print(f"Removed existing file or link: {localtime_path}")
        except subprocess.CalledProcessError as e:
            print(f"Error removing {localtime_path}: {e}")
            return
    try:
        subprocess.run(["sudo", "ln", "-s", zoneinfo_path, localtime_path], check=True)
        print(f"Timezone successfully set to: {timezone}")
    except subprocess.CalledProcessError as e:
        print(f"Failed to set timezone to {timezone}: {e}")


    try:
      subprocess.run(f"TZ=UTC date -s '{new_time}'", shell=True, check=True)
      #subprocess.run()
    except subprocess.CalledProcessError:
      print("timed.failed_setting_time")

  def update(self, json):
    def _i(v, default=0):
      return default if v is None else int(v)
    def _f(v, default=0.0):
      return default if v is None else float(v)  
    def _s(v, default=""):
      return default if v is None else str(v)  
    if json is None:
      return
    if "carrotIndex" in json:
      self.carrotIndex = int(json.get("carrotIndex") or self.carrotIndex + 1)

    #print(json)
    if self.carrotIndex % 60 == 0 and "epochTime" in json:
      epoch = json.get("epochTime")
      if epoch is not None:
        # op는 ntp를 사용하기때문에... 필요없는 루틴으로 보임.
        timezone_remote = json.get("timezone", "Asia/Seoul")

        if not PC:
          self.set_time(int(epoch), timezone_remote)

      #self._update_system_time(int(json.get("epochTime")), timezone_remote)

    if "carrotCmd" in json:
      #print(json.get("carrotCmd"), json.get("carrotArg"))
      self.carrotCmdIndex = self.carrotIndex
      self.carrotCmd = json.get("carrotCmd")
      self.carrotArg = json.get("carrotArg")
      print(f"carrotCmd = {self.carrotCmd}, {self.carrotArg}")

    self.active_count = 80
    now = time.monotonic()

    if "goalPosX" in json:
      gx = json.get("goalPosX")
      gy = json.get("goalPosY")
      if gx is not None and gy is not None:
        self.goalPosX = float(json.get("goalPosX", self.goalPosX))
        self.goalPosY = float(json.get("goalPosY", self.goalPosY))
        self.szGoalName = json.get("szGoalName", self.szGoalName)

    if "nRoadLimitSpeed" in json:
      #print(json)
      self.active_sdi_count = self.active_sdi_count_max
      ### roadLimitSpeed
      nRoadLimitSpeed = int(json.get("nRoadLimitSpeed", 20))
      if nRoadLimitSpeed > 0:
        if nRoadLimitSpeed > 200:
          nRoadLimitSpeed = (nRoadLimitSpeed - 20) / 10
        elif nRoadLimitSpeed == 120:
          nRoadLimitSpeed = 115 # 120 -> 115 fix bug
      else:
        nRoadLimitSpeed = 30
      #self.nRoadLimitSpeed = nRoadLimitSpeed
      if self.nRoadLimitSpeed != nRoadLimitSpeed:
        self.nRoadLimitSpeed_counter += 1
        if self.nRoadLimitSpeed_counter > 5:
          self.nRoadLimitSpeed = nRoadLimitSpeed
      else:
        self.nRoadLimitSpeed_counter = 0

      ### SDI
      self.nSdiType = _i(json.get("nSdiType"), -1)
      self.nSdiSpeedLimit = _i(json.get("nSdiSpeedLimit"), 0)
      self.nSdiSection = _i(json.get("nSdiSection"), -1)
      self.nSdiDist = _i(json.get("nSdiDist"), -1)
      self.nSdiBlockType = _i(json.get("nSdiBlockType"), -1)
      self.nSdiBlockSpeed = _i(json.get("nSdiBlockSpeed"), 0)
      self.nSdiBlockDist = _i(json.get("nSdiBlockDist"), 0)

      self.nSdiPlusType = _i(json.get("nSdiPlusType"), -1)
      self.nSdiPlusSpeedLimit = _i(json.get("nSdiPlusSpeedLimit"), 0)
      self.nSdiPlusDist = _i(json.get("nSdiPlusDist"), 0)
      self.nSdiPlusBlockType = _i(json.get("nSdiPlusBlockType"), -1)
      self.nSdiPlusBlockSpeed = _i(json.get("nSdiPlusBlockSpeed"), 0)
      self.nSdiPlusBlockDist = _i(json.get("nSdiPlusBlockDist"), 0)
      self.roadcate = _i(json.get("roadcate"), 0)

      ## GuidePoint
      self.nTBTDist = int(json.get("nTBTDist", 0))
      self.nTBTTurnType = int(json.get("nTBTTurnType", -1))
      self.szTBTMainText = _s(json.get("szTBTMainText"))
      self.szNearDirName = _s(json.get("szNearDirName"))
      self.szFarDirName = _s(json.get("szFarDirName"))

      self.nTBTNextRoadWidth = int(json.get("nTBTNextRoadWidth", 0))
      self.nTBTDistNext = int(json.get("nTBTDistNext", 0))
      self.nTBTTurnTypeNext = int(json.get("nTBTTurnTypeNext", -1))
      self.szTBTMainTextNext = json.get("szTBTMainText", "")

      self.nGoPosDist = int(json.get("nGoPosDist", 0))
      self.nGoPosTime = int(json.get("nGoPosTime", 0))
      self.szPosRoadName = _s(json.get("szPosRoadName"))
      if self.szPosRoadName == "null":
        self.szPosRoadName = ""

      self.vpPosPointLatNavi = float(json.get("vpPosPointLat", 0.0))
      self.vpPosPointLonNavi = float(json.get("vpPosPointLon", 0.0))
      if self.vpPosPointLatNavi != 0.0:
        self.last_update_gps_time_navi = self.last_calculate_gps_time = now
        self.nPosAngle = float(json.get("nPosAngle", self.nPosAngle))

      self.nPosSpeed = float(json.get("nPosSpeed", self.nPosSpeed))
      self._update_tbt()
      self._update_sdi()
      print(
        f"sdi = {self.nSdiType}, {self.nSdiSpeedLimit}, {self.nSdiPlusType}, " +
        f"tbt = {self.nTBTTurnType}, {self.nTBTDist}, " +
        f"next = {self.nTBTTurnTypeNext}, {self.nTBTDistNext}"
      )
      #print(json)
    else:
      #print(json)
      pass

    # 3초간 navi 데이터가 없으면, phone gps로 업데이트
    if "latitude" in json:
      self.nPosAnglePhone = _f(json.get("heading"), self.nPosAngle)
      self.phone_latitude = _f(json.get("latitude"), self.vpPosPointLatNavi)
      self.phone_longitude = _f(json.get("longitude"), self.vpPosPointLonNavi)
      self.phone_gps_accuracy = _f(json.get("accuracy"), 0)
      if self.phone_gps_accuracy < 15.0:
        self.phone_gps_frame += 1
      if (now - self.last_update_gps_time_navi) > 3.0:
        self.vpPosPointLatNavi = self.phone_latitude
        self.vpPosPointLonNavi = self.phone_longitude

        self.nPosAngle = self.nPosAnglePhone
        # self.nPosSpeed = self.ve # TODO speed from v_ego
        self.last_update_gps_time_phone = self.last_calculate_gps_time = now        
        self.nPosSpeed = float(json.get("gps_speed", 0))
        print(f"phone gps: {self.vpPosPointLatNavi}, {self.vpPosPointLonNavi}, {self.phone_gps_accuracy}, {self.nPosSpeed}")


import traceback

def main():
  print("CarrotManager Started")
  #print("Carrot GitBranch = {}, {}".format(Params().get("GitBranch"), Params().get("GitCommitDate")))
  # 延迟导入，避免与 carrot_man 中导入 CarrotServ 的循环依赖
  from openpilot.selfdrive.carrot.carrot_man import CarrotMan
  carrot_man = CarrotMan()

  print(f"CarrotMan {carrot_man}")
  threading.Thread(target=carrot_man.kisa_app_thread).start()
  while True:
    try:
      carrot_man.carrot_man_thread()
    except Exception as e:
      print(f"carrot_man error...: {e}")
      traceback.print_exc()
      time.sleep(10)


if __name__ == "__main__":
  main()
