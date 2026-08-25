#!/usr/bin/env python3
import os
import time
import collections
import numpy as np
from cereal import log
from opendbc.car.interfaces import ACCEL_MIN
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
# WARNING: imports outside of constants will not trigger a rebuild
from openpilot.selfdrive.modeld.constants import index_function
from openpilot.selfdrive.controls.radard import _LEAD_ACCEL_TAU
# from openpilot.selfdrive.carrot.carrot_functions import CarrotPlanner
from openpilot.selfdrive.carrot.carrot_functions import XState

if __name__ == '__main__':  # generating code
  from openpilot.third_party.acados.acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
else:
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code.acados_ocp_solver_pyx import AcadosOcpSolverCython

from casadi import SX, vertcat

MODEL_NAME = 'long'
LONG_MPC_DIR = os.path.dirname(os.path.abspath(__file__))
EXPORT_DIR = os.path.join(LONG_MPC_DIR, "c_generated_code")
JSON_FILE = os.path.join(LONG_MPC_DIR, "acados_ocp_long.json")

SOURCES = ['lead0', 'lead1', 'cruise', 'e2e']

X_DIM = 3
U_DIM = 1
PARAM_DIM = 8
COST_E_DIM = 5
COST_DIM = COST_E_DIM + 1
CONSTR_DIM = 4

X_EGO_OBSTACLE_COST = 5.
X_EGO_COST = 0.
V_EGO_COST = 0.
A_EGO_COST = 0.
J_EGO_COST = 5.0
A_CHANGE_COST = 200.
A_CHANGE_COST_STARTING = 10. #30.
DANGER_ZONE_COST = 100.
CRASH_DISTANCE = .25
LEAD_DANGER_FACTOR = 0.8 # 0.75
LIMIT_COST = 1e6
ACADOS_SOLVER_TYPE = 'SQP_RTI'


# Fewer timestamps don't hurt performance and lead to
# much better convergence of the MPC with low iterations
N = 12
MAX_T = 10.0
T_IDXS_LST = [index_function(idx, max_val=MAX_T, max_idx=N) for idx in range(N+1)]

T_IDXS = np.array(T_IDXS_LST)
FCW_IDXS = T_IDXS < 5.0
T_DIFFS = np.diff(T_IDXS, prepend=[0.])
COMFORT_BRAKE = 2.5
STOP_DISTANCE = 6.0

def get_jerk_factor(personality=log.LongitudinalPersonality.standard):
  if personality==log.LongitudinalPersonality.moreRelaxed:
    return 1.0
  elif personality==log.LongitudinalPersonality.relaxed:
    return 1.0
  elif personality==log.LongitudinalPersonality.standard:
    return 1.0
  elif personality==log.LongitudinalPersonality.aggressive:
    return 0.5
  else:
    raise NotImplementedError("Longitudinal personality not supported")


def get_T_FOLLOW(personality=log.LongitudinalPersonality.standard):
  if personality==log.LongitudinalPersonality.moreRelaxed:
    return 2.0
  elif personality==log.LongitudinalPersonality.relaxed:
    return 1.75
  elif personality==log.LongitudinalPersonality.standard:
    return 1.45
  elif personality==log.LongitudinalPersonality.aggressive:
    return 1.25
  else:
    raise NotImplementedError("Longitudinal personality not supported")

def get_stopped_equivalence_factor(v_lead):
  return (v_lead**2) / (2 * COMFORT_BRAKE)

def get_safe_obstacle_distance(v_ego, t_follow=None, comfort_brake=COMFORT_BRAKE, stop_distance=STOP_DISTANCE):
  if t_follow is None:
    t_follow = get_T_FOLLOW()
  return (v_ego**2) / (2 * comfort_brake) + t_follow * v_ego + stop_distance

def desired_follow_distance(v_ego, v_lead, comfort_brake, stop_distance, t_follow=None):
  if t_follow is None:
    t_follow = get_T_FOLLOW()
  return get_safe_obstacle_distance(v_ego, t_follow, comfort_brake, stop_distance) - get_stopped_equivalence_factor(v_lead)

# --- margin-based lead accel damping ---
# Goal: when there's plenty of following-distance margin, ignore the lead's aLead
# jitter (accel AND decel) and hold a steady speed instead of chasing every wiggle.
# As the margin shrinks toward the safety-distance threshold, damping fades out so
# the full raw aLead is used -- safety response is never delayed.
# NOTE: only aLead is gated here. dRel/vRel (the actual measured kinematic state)
# are left completely untouched, since damping those would corrupt the real
# closing-distance estimate and could delay braking on a genuinely closing lead.
MARGIN_ACCEL_GATE_FULL = 1.5  # dRel/desired_distance ratio at/above which aLead is fully damped (weight=0)
MARGIN_ACCEL_GATE_NONE = 1.0  # dRel/desired_distance ratio at/below which aLead passes through unchanged (weight=1)

# 2026-08-22 실주행 로그 대응 ("앞차_민감" 이슈): 위 dRel/desired_distance 비율만으로는
# 고속도로 구간에서 damping이 사실상 항상 꺼져 있었다. desired_distance가
# t_follow*v_ego 항 때문에 속도의 제곱보다 완만하게(선형에 가깝게) 커지는 반면
# get_safe_obstacle_distance의 v_ego^2/(2*comfort_brake) 항도 동시에 커져서, 실제로는
# 고속 주행 중 안전한 차간거리(TTC 15s 이상)에서도 ratio가 GATE_NONE(1.0) 밑으로
# 내려가 weight=1(무감쇠)로 굳어지는 경우가 흔함 -- 60m/28m/s(TTC~15s 수준) 구간에서
# aLeadK가 -2.7m/s^2까지 흔들리자 aTarget이 그대로 따라가 -2.78m/s^2까지 반응한
# 사례로 확인됨(FINDINGS.md 38차 참고). ratio 게이트는 "거리 여유"만 보고 "실제 위험
# (TTC)"은 안 보기 때문에 못 걸러낸 것.
# 대응: 기존 dRel/desired_distance ratio 게이트에 TTC 기반 게이트를 추가로 곱해(min),
# "거리 여유가 있고 AND TTC도 여유 있음"일 때만 damping이 걸리도록 한다. TTC 임계값은
# 이 파일 아래쪽 LEAD_ACQ_TTC_CAUTION/DANGER(선점 감속 램프에서 이미 쓰는 값)와 같은
# 축을 공유하되, 이 감쇠는 "안전 확인 후 무시"가 목적이므로 CAUTION(6.0s)을 완전
# 무감쇠(weight=1) 경계로, 그보다 넉넉한 12.0s를 완전 감쇠(weight=0) 경계로 잡는다.
LEAD_ACCEL_TTC_GATE_FULL = 12.0  # TTC(s) 이상이면 aLead 완전 감쇠(weight=0) -- NEEDS_VALIDATION
LEAD_ACCEL_TTC_GATE_NONE = 6.0   # TTC(s) 이하이면 aLead 무감쇠(weight=1), LEAD_ACQ_TTC_CAUTION과 동일 값


def margin_accel_weight(dRel, desired_distance):
  if desired_distance <= 1.0:
    return 1.0
  ratio = dRel / desired_distance
  return float(np.clip((MARGIN_ACCEL_GATE_FULL - ratio) / (MARGIN_ACCEL_GATE_FULL - MARGIN_ACCEL_GATE_NONE), 0.0, 1.0))


# 2026-08-22 실주행 로그 대응 ("저속_앞차" 이슈): 위 TTC 게이트 자체는 정상 작동하지만
# 저속 구간에서는 dRel(절대거리)이 작아 동일한 vRel 변화에도 TTC가 훨씬 빠르게(짧은
# 시간 안에) GATE_NONE(6.0s) 밑으로 떨어진다 -- 예: dRel=16m 근방에서 closing이 1->2.2
# m/s로만 늘어도 TTC가 16s대에서 6~7s대로 0.5~0.6초 만에 붕괴함(고속에서는 dRel이
# 커서 같은 vRel 변화가 훨씬 완만한 TTC 변화로 나타남). 이 급격한 weight 상승(0.3->1.0
# 이 0.6초 안에 발생) 때문에, 그동안 감쇠돼 화면에 안 보이던 aLeadK의 누적된 감속값이
# 한꺼번에 풀려나오면서 aEgo jerk가 순간적으로 -4~-5 m/s^3까지 튀는 "급정지 느낌"이
# 발생함(FINDINGS.md 39차 프레임 대조로 확인).
# 대응: weight가 "안전 -> 위험" 방향(damping 풀림, 값 증가)으로 변할 때만 사이클당
# 최대 변화폭을 제한한다. 반대로 "위험 -> 안전"(damping 강화, 값 감소) 방향은 즉시
# 반영 -- 이 방향은 반응을 더 부드럽게 만들 뿐 위험 상황 반응을 지연시키지 않으므로
# 속도 제한이 필요 없다.
LEAD_ACCEL_WEIGHT_RISE_RATE = 1.0  # 1/s : weight가 0->1까지 올라가는 데 최소 1.0초 걸리도록 제한
                                    # (TTC<=LEAD_ACQ_TTC_DANGER 실제위험 시엔 이 제한 자체를
                                    # 우회하므로 진짜 위급 상황 반응은 지연되지 않음)


# 2026-08-22 실주행 로그 대응 ("정지 후 출발 가속 약화" 이슈, FINDINGS.md 45차):
# ttc_accel_weight()의 closing<=0.1(=벌어짐/등속) -> weight=0 분기는 "위험하지 않은
# 잡음성 가감속 무시"가 목적이었으나, 정차->출발 직후는 정확히 이 조건(자차는 아직
# 0 근방, 앞차는 이미 가속 중이라 v_ego<=v_lead)과 겹쳐 앞차의 실측 가속(aLeadK)이
# extrapolate_lead()에서 통째로 사라지고, 그 결과 출발 시 목표가속도가 38차 이전보다
# 보수적으로 산출됨(가속이 매끈하게 안 이어지고 톱니형으로 끊기는 체감).
# 대응: "정차->출발" 구간을 별도 상태로 잡아 이 구간에서만 ttc_accel_weight()(38차)를
# 완전히 우회하고, 38차 patch 이전과 동일하게 margin_accel_weight()(dist_w)만으로
# a_lead damping을 결정한다. 출발이 끝나(LAUNCH_EXIT_V_EGO 이상) 정상 주행 속도로
# 올라가면 즉시 38차/39차 로직으로 복귀 -- 38차가 막으려던 고속 잡음성 가감속
# 과잉반응 방지는 이 구간(저속) 밖이라 영향 없음.
LAUNCH_BYPASS_STOP_V_EGO = 0.3   # m/s : 이하이면 "정차"로 판정(bypass 진입 준비 상태)
LAUNCH_BYPASS_EXIT_V_EGO = 5.0   # m/s : 정차에서 출발한 뒤 이 속도를 넘으면 38차 로직으로 복귀


# 2026-08-23 실주행 대응 ("정체구간 붕끗" 이슈, FINDINGS.md 58차 2번 계속3):
# 저속 구간(정체 등)에서 앞차가 이미 강하게 감속 중인데도 TTC가 아직
# LEAD_ACCEL_TTC_GATE_NONE(6.0s)을 넘지 않아 ttc_accel_weight()가 그 감속
# (aLeadK)을 계속 감쇠하다가, TTC가 뒤늦게 문턱을 넘는 순간 그동안 감쇠돼
# 있던 aLeadK가 1초 이내로 몰려 반영되며 급가속->급감속 반전으로 체감되는
# 패턴을 실측으로 확인함. route a3a55cb808 seg12 t=4420~4423 실측: min
# TTC=4.45s로 danger override(LEAD_ACQ_TTC_DANGER=2.5s) 문턱과는 무관,
# dRel 17~24m대(정체 특유의 초근접 상황도 아님), 앞차는 ego가 여전히
# 가속 중이던 시점부터 이미 실측 감속 근사치 -1.5~-2.0m/s²대를 유지 중이었음
# (vLead 수치미분 기준, 노이즈 있으나 추세 뚜렷).
#
# 대응: "TTC 문턱을 넘을 때까지 기다렸다가 몰아서 반영" 대신, 저속 구간
# (LOW_SPEED_STRONG_DECEL_V_EGO_GATE 이하)에서만 앞차의 실측 감속이 일정
# 크기(LOW_SPEED_STRONG_DECEL_A_LEAD_THRESH)보다 강하면 TTC 위치와 무관하게
# danger override와 동일하게 즉시 무감쇠(rise-rate도 우회)로 반영한다 --
# 감쇠 누적 자체를 없애 "몰아서 터지는" 상황을 애초에 방지하는 방향.
# **핵심 제약(사용자 요구)**: 저속 게이트 밖(v_ego > 게이트값, 일반/고속
# 주행)에서는 이 분기 자체를 안 타므로 patch 이전과 동작이 100% 동일해야
# 한다 -- 회귀 검증 시 v_ego > 게이트값 시나리오는 diff 0을 기준으로 확인.
LOW_SPEED_STRONG_DECEL_V_EGO_GATE = 30.0 / 3.6   # m/s (~30km/h) : 이 속도 이하에서만 적용
LOW_SPEED_STRONG_DECEL_A_LEAD_THRESH = -1.8      # m/s^2 : 앞차 실측 감속이 이보다 강하면(더 음수) 적용


def ttc_accel_weight(dRel, v_ego, v_lead):
  closing = v_ego - v_lead
  if closing <= 0.1:
    # 벌어지고 있거나 등속 -- TTC 축만 보면 위험 요소 없음(완전 감쇠 방향).
    # 최종 weight는 margin_accel_weight와 min()으로 합쳐지므로, 거리 여유가 없는
    # 근접 상황에서는 ratio 게이트가 이미 weight=1을 유지해 안전 반응은 그대로 통과한다.
    return 0.0
  ttc = dRel / closing
  return float(np.clip((LEAD_ACCEL_TTC_GATE_FULL - ttc) / (LEAD_ACCEL_TTC_GATE_FULL - LEAD_ACCEL_TTC_GATE_NONE), 0.0, 1.0))


# --- Lead-acquisition proactive deceleration ramp (2026-08-17 실주행 로그 대응) ---
# 증상: 고속도로에서 앞차가 처음 인식될 때는 감속이 없다가, 인식 소스가
# 바뀌거나(비전<->레이더) 관측치가 뒤늦게 보정되는 순간부터 갑자기 감속이
# 시작되어 급감속처럼 느껴짐.
#
# 원인: leadOne이 막 나타난 시점의 dRel/vLead 추정치가 가장 부정확하다.
# 이건 비전에만 해당하는 얘기가 아니다 -- Genesis DH 단일빔 SCC 레이더도
# 근접구간/사각지대에서 순간적으로 락을 놓쳤다가 다시 잡을 때 낡은 값을
# 들고 있는 경우가 있고, 반대로 레이더가 먼저 애매한 값으로 락온한 뒤
# 비전이 더 정확한 근거리 값을 보여주는 경우도 있다. 어느 쪽이 먼저
# 인식하든, "막 나타난 리드"의 첫 관측치는 신뢰도가 낮다는 점은 동일하다.
#
# 대응: leadOne이 새로 감지되어 LEAD_ACQ_CONFIRM_TIME 이상 연속으로(짧은
# 순간유실은 LEAD_ACQ_LOSS_GRACE_TIME까지 허용) 유지되는 순간부터, "지금 내
# 속도라면 유지해야 할 표준 차간거리"를 가상의 안전마진 하한선으로 두고,
# 이 하한선을 LEAD_ACQ_RAMP_TIME에 걸쳐 서서히(step 없이) 적용한다.
# - 비전이 먼저 인식하든 레이더가 먼저 인식하든 동일하게 적용 (source 무관,
#   radarstate.leadOne.status만 본다).
# - 1회성 노이즈 블립(아주 잠깐 나타났다 사라지는 오검출)은 CONFIRM_TIME을
#   채우기 전에 사라지므로 램프 자체가 시작되지 않는다.
# - 최초 인식 이후 "연속된 락온"이 아니라 소스 전환/순간유실로 인해 status가
#   깜빡이는 경우, LOSS_GRACE_TIME 이내의 유실은 진행 중이던 램프를 리셋하지
#   않고 그대로 이어간다 (다시 나타나면 누적된 진행률에서 계속). GRACE_TIME을
#   넘겨 정말로 사라지면 그때 완전히 리셋된다.
# - 원본 raw 관측치가 이 하한선보다 이미 더 타이트(위험)하면 그대로 raw가
#   이긴다 (min 연산이므로 이 로직은 감속을 절대 완화시키지 않고, 오직
#   "최소한 이만큼은 이미 감속을 시작해야 한다"는 바닥만 깔아준다).
# - 램프가 끝나면(기본 5초) 하한선은 완전히 해제되고 이후로는 실측치가 그대로
#   사용된다 -- 정말로 멀리 있는(위협적이지 않은) 리드를 영구히 좁은
#   목표거리로 묶어두지 않기 위함.
#
# 튜닝 이력 (2026-08-17, 6개 구간 실주행 로그 22세그먼트 재검증):
# RAMP_TIME=3.0s 상태로는 최초 인식 시점에 이미 접근속도(vRel)가 크게
# 마이너스인 "급접근" 리드(예: vRel0 -8~-13m/s)에서 급감속이 ramp 완료
# 이전(1.3~3.0s 부근)이나 ramp 종료 직후(~5s)에 터지는 사례가 다수
# 관측됨 (route2a t=162s: aMin=-2.99@1.3s, route4b t=94.8s: aMin=-5.46@3.0s,
# route4a t=211s: aMin=-2.57@5.0s). 개입 구간을 5.0s로 늘려 완만한 접근
# 케이스의 부드러움은 유지하면서 급접근 케이스에 대한 커버리지를 넓힘.
LEAD_ACQ_RAMP_TIME       = 5.0   # s   : 하한선을 0 -> 100% 로 서서히 적용하는 시간 (경과시간 기준 기본 램프)
LEAD_ACQ_MIN_V_EGO       = 3.0   # m/s : 이 속도 미만에서는 적용하지 않음 (정체/크립 구간 노이즈 방지)
LEAD_ACQ_CONFIRM_TIME    = 0.2   # s   : 이 시간 이상 연속 감지되어야 "진짜 인식"으로 보고 램프 시작 (1회성 블립 무시)
LEAD_ACQ_LOSS_GRACE_TIME = 0.5   # s   : 짧은 순간유실은 이 시간까지 봐주고 진행 중인 램프를 그대로 유지

# 튜닝 이력 (2026-08-17 #2, TTC 기반 가변 강도 추가):
# 경과시간 기준 램프(위 RAMP_TIME)만으로는 최초 인식 시점의 vRel 자체가
# 낙관적으로 저평가된 케이스(비전 단독)에서, 진짜 접근속도가 램프 도중/직후에
# 뒤늦게 드러나는 경우를 완전히 못 잡는다 (예: route2a t=162s는 1.3초 만에
# aMin 발생, 5초 램프라도 그 시점엔 26%만 걸린 상태).
#
# 그래서 "인식 시점의 TTC를 한 번만 스냅샷"하는 대신, 매 프레임 현재
# dRel/vRel로 TTC를 다시 계산해서 "지금 이 순간 진짜 위험한가"를 실시간으로
# 반영한다. radard.py의 LeadBlend가 TTC<2.5s에서 즉시 반응하는 것과 동일한
# 임계값을 사용해 두 로직의 위험 판단 기준을 통일한다.
# - TTC >= LEAD_ACQ_TTC_CAUTION(6s): 위험 요소 없음, 기존 경과시간 램프만 적용
# - TTC <= LEAD_ACQ_TTC_DANGER(2.5s): 경과시간 램프 진행 상황과 무관하게 즉시
#   최대 강도(frac=1.0)로 개입 -- 진짜 접근속도가 막 드러난 순간이라도 그
#   프레임부터 바로 반응
# - 그 사이는 선형 보간
# - frac = max(경과시간 기준 frac, TTC 기준 frac) 이므로 이 로직 역시 절대
#   감속을 완화시키지 않고, 위험 신호가 잡히면 강도를 끌어올리기만 한다.
LEAD_ACQ_TTC_DANGER      = 2.5   # s   : 이하이면 즉시 frac=1.0 (radard.py LeadBlend danger_hold와 동일 임계값)
LEAD_ACQ_TTC_CAUTION     = 6.0   # s   : 이상이면 TTC 기반 개입 없음 (경과시간 램프만 작동)

# --- Vision-only closing-rate cross-check (2026-08-20 실주행 대응) ---
# 증상: 고속도로에서 먼 거리의 서행/정지 차량을 비전이 먼저 인식(파란박스,
# modelProb 0.5대의 약한 확신)한 뒤로도 한참(수 초~10초 가까이) 감속이
# 시작되지 않다가, SCC 레이더가 락온(빨간박스)하는 순간부터 갑자기 감속이
# 시작됨.
#
# 원인: radard.py VisionTrack.update()는 modelProb < 0.97인 동안(즉 먼
# 거리에서 거의 항상) leadOne.vRel을 모델이 예측한 순간 속도차이
# (lead_msg.v[0] - model_v_ego)에서 그대로 가져온다. 이 값은 원거리·저확신
# 구간에서 실제 접근속도보다 낙관적으로(0에 가깝게) 추정되는 경향이 있다
# (VISION_RADAR_CROSSOVER.md 참고, highway 크로스오버 사례 중 갭 7~8초 동안
# 90m 이상 좁혀진 경우 다수 확인). 위 LEAD_ACQ_TTC_* 로직은 이미 매 프레임
# TTC를 재계산하지만, 그 TTC 자체가 이 낙관적인 vRel로 계산되므로 실제
# 위험이 가려진 채로는 절대 임계값을 넘지 못해 개입하지 않는다 -- 레이더가
# 락온해 정확한 vRel로 교체되는 그 프레임에야 비로소 TTC가 뚝 떨어지면서
# 뒤늦게 급하게 반응하는 것처럼 느껴짐.
#
# 대응: radarstate.leadOne.vRel과는 별개로, leadOne.dRel 자체(위치 측정치는
# 비전도 비교적 정확함)를 프레임 간 미분해서 독립적인 접근속도 추정치를
# 저역통과 필터로 누적한다. 레이더가 아직 락온하지 않은 상태(leadOne.radar
# == False)에서만 갱신/사용하며, 최소 VISION_CLOSING_RATE_MIN_TIME 이상
# 연속 추적된 뒤부터만 신뢰해 초기 몇 프레임의 미분 노이즈를 걸러낸다.
# 이렇게 얻은 TTC는 기존 vRel 기반 TTC와 min()으로 합쳐 "둘 중 더 위험한
# 쪽"을 frac_ttc 계산에 사용한다 -- 기존 로직과 동일하게 순수 바닥(floor)
# 역할만 하며 감속을 완화시키는 방향으로는 절대 작동하지 않는다.
VISION_CLOSING_RATE_TAU      = 1.0   # s   : dRel 미분값 저역통과 필터 시정수 (짧을수록 반응 빠르지만 노이즈에 민감)
VISION_CLOSING_RATE_MIN_TIME = 0.5   # s   : 이 시간 이상 연속 추적(비전 단독) 후에만 dRel 미분 TTC를 신뢰
                                      # (모델 주기 DT_MDL=0.05s 기준 10프레임 -- 저역통과 필터가 원값 대비
                                      # 약 39% 정도 수렴한 시점. 1.0s(20프레임, ~63% 수렴)보다 반응은
                                      # 빠르지만 초기 수렴폭이 작으므로 danger 판정이 다소 보수적으로
                                      # 나올 수 있음 -- 실측으로 추가 단축 여지 판단 필요)

# --- 곡선 구간 dRel 스냅 노이즈 대응 (2026-08-21, 25차 실주행 재현) ---
# 증상/원인: 23차 로그 분석에서 곡선(src=vturn) 구간에 도로 가장자리의
# 대형/정차 차량이 리드 후보 셋에 간헐적으로 혼입되며 leadDRel이 한 프레임
# 만에 8m+ 튀었다가(노이즈) 곧 원래 값 근처로 복귀하는 "스냅-복귀" 패턴이
# 확인됨(devnotes toolkit curve_lead_dRel_jump_consistency() 참고, 91.7%가
# 이 패턴). 위 VISION_CLOSING_RATE_TAU 저역통과 필터는 단일 프레임 raw_rate를
# 바로 입력으로 쓰기 때문에, 이런 순간 스냅 하나가 alpha(=dt/TAU≈0.05) 비중
# 만큼이라도 즉시 _vision_dRel_rate를 오염시켜 노이즈성 DANGER급 TTC를
# 유발할 수 있음(23차 routeB seg12 t=815/817, 필터링 후 값 기준 -12~-25m/s
# 관측).
#
# devnotes의 오프라인 분석(curve_lead_dRel_jump_consistency)은 점프 이후
# "미래" 1.5초 구간을 봐서 복귀 여부를 판단하지만, 실시간 제어 코드는 미래를
# 볼 수 없다. 대신 두 단계로 근사한다:
# 1) raw_rate를 물리적으로 타당한 최대치로 클램프 -- 곡선 노이즈 스냅
#    (한 프레임 dt=0.05s에 8m+ 점프 -> raw_rate 160m/s+)은 실제 어떤 동일
#    차로 선행차 시나리오보다도 압도적으로 크므로 클램프만으로도 원천 차단.
# 2) 클램프된 raw_rate를 저역통과 필터에 바로 먹이지 않고, 최근 N프레임의
#    "중앙값"을 먼저 취한 뒤 필터에 넣는다 -- 노이즈 스냅은 한두 프레임짜리
#    "튀었다 복귀"이므로 중앙값 윈도우 안에서 다수결에 밀려 걸러지고, 진짜
#    지속적인 접근(여러 프레임 연속 같은 방향)은 중앙값에도 그대로 반영된다.
#    devnotes의 monotonic_frac/reverted 체크가 노리는 것과 동일한 효과를
#    미래를 보지 않고 얻는 방식.
VISION_CLOSING_RATE_MAX_PLAUSIBLE = 30.0  # m/s : 이보다 빠른(음의) 순간 dRel 변화율은 물리적으로 불가능한
                                           #        노이즈로 간주해 클램프 (동일 차로 선행차 시나리오 기준 넉넉한 상한)
VISION_CLOSING_RATE_MEDIAN_WINDOW = 3     # 프레임 : 최근 N개 클램프된 raw_rate의 중앙값을 필터 입력으로 사용
                                           #          (DT_MDL=0.05s 기준 최대 0.1s 지연 추가, 곡선 노이즈 스냅 억제용)

# --- 방안 E: 참고 closing rate(leadVLead) 기반 상대적 타당성 클램프 ---
# (2026-08-24, 63차 계속9 설계 / 63차 계속10 재생검증+정정으로 확정)
#
# 배경: cut-in(끼어들기) 상황에서 트랙이 기존 리드 -> 새로 끼어든 차량으로
# 전환되는 순간, raw dRel 미분(raw_rate)이 물리적으로 불가능한 크기
# (-100~-339m/s급)로 튀는 사례 확인(seg3/seg14). 위 절대값 클램프
# (VISION_CLOSING_RATE_MAX_PLAUSIBLE=30.0)만으로는 이런 트랙전환성 점프를
# 다 못 거름 -- 30m/s 자체가 이미 상당히 넉넉한 상한이라, 그보다 작지만
# 여전히 비현실적인 값(예: -25m/s)은 통과함.
#
# 방안: raw_rate 클램프 하한에 "모델이 이미 추정한 상대속도"(lead.vLead)
# 기반 참고 closing rate를 추가한다. leadVLead는 raw dRel 프레임간 미분보다
# 훨씬 안정적인 신호(63차 계속9 실측: raw_rate -235~-6m/s vs vEgo-vLead
# 기반 참고치 -0.5~-3.2m/s)이므로, 이 참고치에서 너무 크게 벗어나는(즉
# 참고치보다 훨씬 더 접근중이라고 말하는) raw_rate는 신뢰하지 않는다.
#
# ref_rate = -(v_ego - lead.vLead)          # 모델 기반 참고 closing rate
# plausible_min = ref_rate - VISION_RATE_REF_MARGIN
# raw_rate_clamped = max(raw_rate, -VISION_CLOSING_RATE_MAX_PLAUSIBLE, plausible_min)
#
# 안전 성질(63차 계속10 재확인): leadVLead가 실제 위험(빠른 접근)을 정확히
# 가리키는 경우엔 ref_rate도 함께 크게 음수가 돼 plausible_min도 충분히
# 낮아지므로 raw_rate를 거의 그대로 통과시킴 -- 즉 이 클램프는 leadVLead가
# "안전하다"(접근 아님/느린 접근)고 말할 때만 raw dRel의 과도한 튐을
# 억제하는 구조. leadVLead도 위험을 인지하는 진짜 급접근까지 억제하는
# 방향은 아니며, DANGER override(TTC<=2.5s) 경로는 이 클램프와 완전히
# 무관하게 항상 그대로 유지된다.
#
# 검증 이력: 63차 계속9 로직단위 재생(seg14, 완만한 계단식 포화 -> 램프로
# 개선), 63차 계속10 재생검증(seg3, 진짜 cutin에서 leadVLead가 실제로도
# 안전(opening 직전)했음을 레이더 락온 후 vRel 실측(+3.2~+7m/s)으로 확인
# -- PATCHED_E의 억제가 오탐 억제가 아니라 정탐이었음이 사후 확정됨).
VISION_RATE_REF_MARGIN = 5.0              # m/s : ref_rate 대비 이만큼까지만 raw_rate가 더 낮을(더 접근할) 수 있다고 인정

# --- Vision-only closing-rate 절대값 게이트 (2026-08-21, 25차) ---
# 증상: 25차 실주행 화면녹화 영상 판독 결과, 원거리(dRel 85~120m)에서
# closing rate가 5m/s 안팎으로 뚜렷이 존재하는데도 TTC(=dRel/rate, 이
# 거리에서 15~20s+)가 LEAD_ACQ_TTC_CAUTION(6.0s) 문턱을 한참 못 넘어
# a_target이 거의 변화 없이 유지되는 구간이 다수 관찰됨. 22~24차에서 이미
# 파악한 "TTC 캐션 문턱이 원거리에서 구조적으로 안 넘어간다"는 한계와
# 일치.
#
# 대응: 위에서 정제한 _vision_dRel_rate(클램프+중앙값 필터로 곡선 노이즈
# 억제된 값) 자체가 이미 상황적으로 위험한 접근속도라면, 거리가
# 멀어서 TTC가 아직 문턱을 못 넘었더라도 별도 성분(frac_rate)으로 개입
# 강도를 끌어올린다. frac_time/frac_ttc와 마찬가지로 순수 바닥(floor) 역할만
# 하며 최종적으로 max()로 합쳐지므로 감속을 완화시키는 방향으로는 절대
# 작동하지 않는다.
# 문턱값은 22차에서 설계된 대안 2번 그대로: CAUTION -5.5m/s(약 20km/h
# 상대속도)부터 서서히 개입 시작, DANGER -10.0m/s부터 최대 강도.
VISION_CLOSING_RATE_GATE_CAUTION = -2.2   # m/s : 이 값 이상(접근 느림)이면 frac_rate=0
VISION_CLOSING_RATE_GATE_DANGER  = -5.0   # m/s : 이 값 이하(급접근)면 frac_rate=1.0, 그 사이는 선형보간

# 60차 계속 (2026-08-24): cutin/차선변경 3건(--5/--17/--12)에서 vision-only
# 구간의 다중 프레임 dRel catch-up이 58차1의 v_lead 직접보정
# (measured_v_lead = v_ego + vision_dRel_rate)을 오염시켜 과잉감속을 유발
# 한다는 가설(NEEDS_VALIDATION)이 3건 모두에서 재현됨. 58차1 보정 자체는
# 그대로 두되(정상적인 원거리 지속 접근 상황에서는 유효한 개선), 아래 두
# "구조적 catch-up 발생 상황"에서만 보정을 일시 유예한다:
#  1) 리드 신규 등록 직후(cutin류) -- _lead_acq_timer가 짧을 때 dRel이
#     화면 진입/트랙 매칭 관성으로 급격히 "따라잡히는" 구간
#  2) 자차 차선변경 조작 중(blinker on, 자동/수동 무관) -- 조향에 연동해
#     비전 매칭 대상의 dRel 추정이 흔들리는 구간
# 두 상황 모두 밖(정상 추종)에서는 58차1 보정이 그대로 100% 동일하게
# 작동 -- 완화가 아니라 취약 구간에서의 "패치 이전 로직 복귀"임에 유의.
NEW_LEAD_VLEAD_CORRECTION_SUPPRESS_S = 1.5  # s : 리드 신규등록 후 이 시간
                                             # 동안은 v_lead 직접보정 유예
LANE_CHANGE_VLEAD_CORRECTION_HOLD_S = 1.0   # s : 차선변경(blinker) 종료 후
                                             # 이 시간 더 유예 유지(잔여 흔들림 대비)

# 61차 계속(2026-08-24, 방안 C): cutin 급감속 2건(r1-3/r1-14) 재발견 --
# 위 신규등록 게이트(NEW_LEAD_VLEAD_CORRECTION_SUPPRESS_S)만으로는 못 잡는
# 별개 패턴: 자차로로 측면 진입(cutin)하는 순간, vision dRel의 프레임간
# 미분값이 "종방향으로 급접근중"이라는 착시 신호를 만들어냄(실제로는 옆
# 차로에서 들어오며 dRel이 기하학적으로 뚝 떨어지는 것뿐) -- 레이더가
# 나중에 락온해서야 실제 vRel(오히려 이탈 중)이 드러남. 이미 등록된
# (신규등록 게이트 유예기간을 지난) 리드에서도 발생할 수 있으므로 별도
# 감지가 필요.
# 대응: 최근 DREL_DISCONTINUITY_WINDOW_N 프레임 내 dRel이 DREL_DISCONTINUITY_
# DROP_THRESH 이상 불연속 급락하면 "타겟 전환/측면진입 의심"으로 보고,
# 기존에 이미 검증된 신규등록 suppress 메커니즘을 그대로 재사용한다
# (_lead_acq_timer를 리셋 -- 새 코드경로 추가 없이 위 NEW_LEAD_VLEAD_
# CORRECTION_SUPPRESS_S 유예가 자동으로 이어서 적용됨). TTC danger
# override(실제 위험, TTC<=LEAD_ACQ_TTC_DANGER)는 이 리셋과 무관하게
# process_lead()에서 항상 그대로 작동(안전 백스톱 유지, 위 617번째줄대
# 부근 참고).
DREL_DISCONTINUITY_DROP_THRESH = 15.0   # m : 이 이상 급락하면 급락으로 판정
DREL_DISCONTINUITY_WINDOW_N = 5         # 최근 5프레임(~0.25s @ 20Hz) 내 기준

# 66차/67차(방안G): discontinuity(위 DREL_DISCONTINUITY_* 참고) 직후 아직
# 절대거리가 부족한 상황(danger override는 아님, 예: 세그4-1류)에서 목표거리
# 자체는 그대로 두고 MPC가 그 거리에 "도달하는 속도"(저크비용)만 한시적으로
# 완만하게 만든다. danger override/저속강한감속(process_lead)이나 proactive
# floor(frac_time/frac_ttc/frac_rate) 중 하나라도 위험을 감지하면 즉시 무시.
DISCONTINUITY_JERK_COST_BOOST_S = 1.0   # s : 부스트 유지시간(트리거 후 감쇠)
DISCONTINUITY_JERK_COST_BOOST = 500.0   # 부스트 값(평시 최대 A_CHANGE_COST=200보다 큼)

# 72차(방안 I, 2026-08-25 실차 재현: "정지앞차 레이더락온시 급감속"): 위
# DREL_DISCONTINUITY_*는 "비전 단독 추적 중 dRel(거리) 급락"만 감지한다.
# 그런데 실차 사례(route1 t=690.05)는 거리가 아니라 **레이더가 막 락온되는
# 그 프레임의 vRel(속도)**이 불연속으로 튀는 경우였다 -- 비전이 6초 넘게
# "리드 속도 12~16m/s(자차와 비슷, 안전)"로 낙관적으로 보고하다가, 레이더가
# 락온되는 순간 실제 vRel이 -3.6 -> -10.8m/s로 급변하며 진짜(서행/정차)
# 상태가 드러남. 이 전환 프레임 자체는 기존 코드(위 elif 분기)가 비전 부기
# 리셋만 하고 불연속 판정을 하지 않아 사각지대였음.
# 대응: 레이더가 False->True로 바뀌는 바로 그 프레임에서, 직전 프레임의
# vRel 대비 이번 프레임 vRel이 RADAR_HANDOFF_VREL_JUMP_THRESH 이상 더
# 접근방향(음수)으로 튀면 -- 이미 검증된 DISCONTINUITY_JERK_COST_BOOST를
# 그대로 arm한다(새 부스트 값/메커니즘 추가 아님, 트리거 조건만 확장).
# TTC danger override/proactive floor(frac>0)가 하나라도 걸리면 위
# a_change_cost 적용부(L1087 부근)에서 즉시 무시되므로 진짜 위험 상황에는
# 영향 없음 -- 도달 감속량이 아니라 도달 속도(저크)만 완만화.
RADAR_HANDOFF_VREL_JUMP_THRESH = 3.0    # m/s : 레이더 락온 프레임의 vRel이 직전 프레임보다
                                         #        이 이상 더 나빠지면(접근방향) 불연속으로 판정


def gen_long_model():
  model = AcadosModel()
  model.name = MODEL_NAME

  # set up states & controls
  x_ego = SX.sym('x_ego')
  v_ego = SX.sym('v_ego')
  a_ego = SX.sym('a_ego')
  model.x = vertcat(x_ego, v_ego, a_ego)

  # controls
  j_ego = SX.sym('j_ego')
  model.u = vertcat(j_ego)

  # xdot
  x_ego_dot = SX.sym('x_ego_dot')
  v_ego_dot = SX.sym('v_ego_dot')
  a_ego_dot = SX.sym('a_ego_dot')
  model.xdot = vertcat(x_ego_dot, v_ego_dot, a_ego_dot)

  # live parameters
  a_min = SX.sym('a_min')
  a_max = SX.sym('a_max')
  x_obstacle = SX.sym('x_obstacle')
  prev_a = SX.sym('prev_a')
  lead_t_follow = SX.sym('lead_t_follow')
  lead_danger_factor = SX.sym('lead_danger_factor')
  comfort_brake = SX.sym('comfort_brake')
  stop_distance = SX.sym('stop_distance')
  model.p = vertcat(a_min, a_max, x_obstacle, prev_a, lead_t_follow, lead_danger_factor, comfort_brake, stop_distance)

  # dynamics model
  f_expl = vertcat(v_ego, a_ego, j_ego)
  model.f_impl_expr = model.xdot - f_expl
  model.f_expl_expr = f_expl
  return model


def gen_long_ocp():
  ocp = AcadosOcp()
  ocp.model = gen_long_model()

  Tf = T_IDXS[-1]

  # set dimensions
  ocp.dims.N = N

  # set cost module
  ocp.cost.cost_type = 'NONLINEAR_LS'
  ocp.cost.cost_type_e = 'NONLINEAR_LS'

  QR = np.zeros((COST_DIM, COST_DIM))
  Q = np.zeros((COST_E_DIM, COST_E_DIM))

  ocp.cost.W = QR
  ocp.cost.W_e = Q

  x_ego, v_ego, a_ego = ocp.model.x[0], ocp.model.x[1], ocp.model.x[2]
  j_ego = ocp.model.u[0]

  a_min, a_max = ocp.model.p[0], ocp.model.p[1]
  x_obstacle = ocp.model.p[2]
  prev_a = ocp.model.p[3]
  lead_t_follow = ocp.model.p[4]
  lead_danger_factor = ocp.model.p[5]
  comfort_brake = ocp.model.p[6]
  stop_distance = ocp.model.p[7]

  ocp.cost.yref = np.zeros((COST_DIM, ))
  ocp.cost.yref_e = np.zeros((COST_E_DIM, ))

  desired_dist_comfort = get_safe_obstacle_distance(v_ego, lead_t_follow, comfort_brake, stop_distance)

  # The main cost in normal operation is how close you are to the "desired" distance
  # from an obstacle at every timestep. This obstacle can be a lead car
  # or other object. In e2e mode we can use x_position targets as a cost
  # instead.
  costs = [((x_obstacle - x_ego) - (desired_dist_comfort)) / (v_ego + 10.),
           x_ego,
           v_ego,
           a_ego,
           a_ego - prev_a,
           j_ego]
  ocp.model.cost_y_expr = vertcat(*costs)
  ocp.model.cost_y_expr_e = vertcat(*costs[:-1])

  # Constraints on speed, acceleration and desired distance to
  # the obstacle, which is treated as a slack constraint so it
  # behaves like an asymmetrical cost.
  constraints = vertcat(v_ego,
                        (a_ego - a_min),
                        (a_max - a_ego),
                        ((x_obstacle - x_ego) - lead_danger_factor * (desired_dist_comfort)) / (v_ego + 10.))
  ocp.model.con_h_expr = constraints

  x0 = np.zeros(X_DIM)
  ocp.constraints.x0 = x0
  ocp.parameter_values = np.array([-1.2, 1.2, 0.0, 0.0, lead_t_follow, LEAD_DANGER_FACTOR, comfort_brake, stop_distance])


  # We put all constraint cost weights to 0 and only set them at runtime
  cost_weights = np.zeros(CONSTR_DIM)
  ocp.cost.zl = cost_weights
  ocp.cost.Zl = cost_weights
  ocp.cost.Zu = cost_weights
  ocp.cost.zu = cost_weights

  ocp.constraints.lh = np.zeros(CONSTR_DIM)
  ocp.constraints.uh = 1e4*np.ones(CONSTR_DIM)
  ocp.constraints.idxsh = np.arange(CONSTR_DIM)

  # The HPIPM solver can give decent solutions even when it is stopped early
  # Which is critical for our purpose where compute time is strictly bounded
  # We use HPIPM in the SPEED_ABS mode, which ensures fastest runtime. This
  # does not cause issues since the problem is well bounded.
  ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
  ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
  ocp.solver_options.integrator_type = 'ERK'
  ocp.solver_options.nlp_solver_type = ACADOS_SOLVER_TYPE
  ocp.solver_options.qp_solver_cond_N = 1

  # More iterations take too much time and less lead to inaccurate convergence in
  # some situations. Ideally we would run just 1 iteration to ensure fixed runtime.
  ocp.solver_options.qp_solver_iter_max = 10
  ocp.solver_options.qp_tol = 1e-3

  # set prediction horizon
  ocp.solver_options.tf = Tf
  ocp.solver_options.shooting_nodes = T_IDXS

  ocp.code_export_directory = EXPORT_DIR
  return ocp


class LongitudinalMpc:
  def __init__(self, mode='acc', dt=DT_MDL):
    self.mode = mode
    self.dt = dt
    self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)

    self.a_change_cost = A_CHANGE_COST
    self.j_lead = 0.0

    self.reset()
    self.source = SOURCES[2]

    self.t_follow = 1.0
    self.desired_distance = 0.0
    self.lead_danger_factor = LEAD_DANGER_FACTOR

    # lead-acquisition proactive deceleration ramp state (see LEAD_ACQ_RAMP_TIME above)
    self._lead_present_run_timer = 0.0   # 연속(짧은 유실 포함) 감지 지속시간 -> CONFIRM_TIME과 비교
    self._lead_absent_timer = 0.0        # 미검출 지속시간 -> LOSS_GRACE_TIME과 비교, 넘으면 완전 리셋
    self._lead_acq_ramp_started = False  # CONFIRM_TIME을 채워서 램프가 실제로 시작되었는지
    self._lead_acq_timer = 0.0           # 램프 시작 이후 경과시간 -> frac 계산용

    # vision-only closing-rate cross-check state (see VISION_CLOSING_RATE_* below)
    self._vision_dRel_prev = None        # 직전 프레임 dRel (레이더 미확인 상태에서만 갱신)
    self._vision_dRel_rate = 0.0         # 저역통과 필터링된 dRel 변화율(m/s), 음수=접근중
    self._vision_dRel_rate_window = collections.deque(maxlen=VISION_CLOSING_RATE_MEDIAN_WINDOW)
                                          # 클램프된 raw_rate 최근 N프레임 (중앙값 필터용, 곡선 노이즈 스냅 억제)

    # 61차 계속(방안 C): cutin 불연속 급락 감지용 raw dRel 이력(위
    # DREL_DISCONTINUITY_* 주석 참고) -- rate가 아니라 원본 dRel 값 자체를
    # 담아 윈도우 양끝 차이로 급락 여부를 판정한다.
    self._dRel_raw_history = collections.deque(maxlen=DREL_DISCONTINUITY_WINDOW_N)

    # 60차 계속: 58차1 v_lead 직접보정 유예 상태(차선변경 hold 타이머) --
    # blinker가 꺼진 뒤에도 LANE_CHANGE_VLEAD_CORRECTION_HOLD_S만큼 유예 유지
    self._lane_change_vlead_hold_timer = 0.0

    # lead-accel damping weight rise-rate limit state (see LEAD_ACCEL_WEIGHT_RISE_RATE below)
    self._lead_accel_weight_prev = 1.0   # 직전 사이클 weight -- 초기값 1.0(무감쇠)이 안전측 기본값

    # launch bypass state (see LAUNCH_BYPASS_* above, FINDINGS.md 45차)
    self._launch_bypass_active = False   # True인 동안 ttc_accel_weight()(38차) 완전 우회

    # 66차/67차(방안G): discontinuity 직후 a_change_cost 한시적 부스트 상태
    # (위 DISCONTINUITY_JERK_COST_BOOST_* 주석 참고)
    self._discontinuity_jerk_boost_timer = 0.0  # >0인 동안 부스트 윈도우 내, 매 사이클 감쇠
    self._lead0_danger_active = False           # process_lead(leadOne)의 danger override/저속강한감속 최신 상태

    # 72차(방안 I): 레이더 락온 전환(vision->radar handoff) 프레임의 vRel
    # 불연속 감지용 상태 (위 RADAR_HANDOFF_VREL_JUMP_THRESH 주석 참고)
    self._prev_lead_radar = False        # 직전 프레임의 leadOne.radar -- False->True 엣지 검출용
    self._prev_lead_vRel = None          # 직전 프레임의 leadOne.vRel (status True인 프레임에서만 갱신)


  def reset(self):
    # self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)
    self.solver.reset()
    # self.solver.options_set('print_level', 2)
    self.v_solution = np.zeros(N+1)
    self.a_solution = np.zeros(N+1)
    self.prev_a = np.array(self.a_solution)
    self.j_solution = np.zeros(N)
    self.yref = np.zeros((N+1, COST_DIM))
    for i in range(N):
      self.solver.cost_set(i, "yref", self.yref[i])
    self.solver.cost_set(N, "yref", self.yref[N][:COST_E_DIM])
    self.x_sol = np.zeros((N+1, X_DIM))
    self.u_sol = np.zeros((N,1))
    self.params = np.zeros((N+1, PARAM_DIM))
    for i in range(N+1):
      self.solver.set(i, 'x', np.zeros(X_DIM))
    self.last_cloudlog_t = 0
    self.status = False
    self.crash_cnt = 0.0
    self.solution_status = 0
    # timers
    self.solve_time = 0.0
    self.time_qp_solution = 0.0
    self.time_linearization = 0.0
    self.time_integrator = 0.0
    self.x0 = np.zeros(X_DIM)
    self.set_weights()

  def set_cost_weights(self, cost_weights, constraint_cost_weights):
    W = np.asfortranarray(np.diag(cost_weights))
    for i in range(N):
      # TODO don't hardcode A_CHANGE_COST idx
      # reduce the cost on (a-a_prev) later in the horizon.
      W[4,4] = cost_weights[4] * np.interp(T_IDXS[i], [0.0, 1.0, 2.0], [1.0, 1.0, 0.0])
      self.solver.cost_set(i, 'W', W)
    # Setting the slice without the copy make the array not contiguous,
    # causing issues with the C interface.
    self.solver.cost_set(N, 'W', np.copy(W[:COST_E_DIM, :COST_E_DIM]))

    # Set L2 slack cost on lower bound constraints
    Zl = np.array(constraint_cost_weights)
    for i in range(N):
      self.solver.cost_set(i, 'Zl', Zl)

  def set_weights(self, prev_accel_constraint=True, personality=log.LongitudinalPersonality.standard, jerk_factor=1.0, a_change_cost_starting=A_CHANGE_COST_STARTING):
    #jerk_factor = get_jerk_factor(personality)
    if self.mode == 'acc':
      a_change_cost = self.a_change_cost if prev_accel_constraint else a_change_cost_starting
      cost_weights = [X_EGO_OBSTACLE_COST, X_EGO_COST, V_EGO_COST, A_EGO_COST, jerk_factor * a_change_cost, jerk_factor * J_EGO_COST]
      constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, DANGER_ZONE_COST]
    elif self.mode == 'blended':
      a_change_cost = 40.0 if prev_accel_constraint else 0
      cost_weights = [0., 0.1, 0.2, 5.0, a_change_cost, 1.0]
      constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, DANGER_ZONE_COST]
    else:
      raise NotImplementedError(f'Planner mode {self.mode} not recognized in planner cost set')
    self.set_cost_weights(cost_weights, constraint_cost_weights)

  def set_cur_state(self, v, a):
    v_prev = self.x0[1]
    self.x0[1] = v
    self.x0[2] = a
    if abs(v_prev - v) > 2.:  # probably only helps if v < v_prev
      for i in range(N+1):
        self.solver.set(i, 'x', self.x0)

  @staticmethod
  def extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau, j_lead):
    j_lead_tau = np.interp(j_lead, [-2.0, 0.0, 2.0], [0.2, 2.0, 0.1]) # tau: 2: 2sec, 1: 4sec, 0.5: 10sec
    j_lead_traj = j_lead * np.exp(-j_lead_tau * (T_IDXS**2)/2.)
    a_lead_traj = a_lead * np.exp(-a_lead_tau * (T_IDXS**2)/2.) + j_lead_traj
    v_lead_traj = np.clip(v_lead + np.cumsum(T_DIFFS * a_lead_traj), 0.0, 1e8)
    x_lead_traj = x_lead + np.cumsum(T_DIFFS * v_lead_traj)
    lead_xv = np.column_stack((x_lead_traj, v_lead_traj))
    return lead_xv
  
  def process_lead(self, lead, j_lead, vision_dRel_rate=None, is_lead0=False):
    v_ego = self.x0[1]

    # launch bypass state update (FINDINGS.md 45차) -- 정차->출발 감지는 리드 유무와
    # 무관하게 v_ego만으로 판단하므로 lead 분기 이전에 갱신한다.
    if v_ego < LAUNCH_BYPASS_STOP_V_EGO:
      self._launch_bypass_active = True    # 정차 상태 -- bypass 진입 arm
    elif v_ego >= LAUNCH_BYPASS_EXIT_V_EGO:
      self._launch_bypass_active = False   # 출발 완료(가속 계속) -- 38차 로직 복귀

    if lead is not None and lead.status:
      x_lead = lead.dRel
      v_lead = lead.vLead
      a_lead = lead.aLeadK
      a_lead_tau = lead.aLeadTau

      # 58차 1번: "카메라 인식 감속이 레이더 대비 약함" 개선. vision_dRel_rate는
      # long_mpc가 25/26차부터 독립적으로 계산해온 실측 dRel 미분(레이더가 쓰는
      # 것과 동일한 방식, VISION_CLOSING_RATE_MIN_TIME 이상 연속추적 후에만
      # 신뢰) -- 지금까지는 frac_rate로 MPC obstacle-distance의 하한(floor)만
      # 조이는 데 썼고, MPC가 실제 lead 궤적을 extrapolate하는 v_lead 자체는
      # vision 원본(lead.vLead, 원거리에서 과소평가 경향 확인됨)을 그대로
      # 썼음. 여기서도 반영해 vision-only 상황의 감속 반응을 레이더 락온 시
      # 수준에 가깝게 강화한다. 안전측(더 빠른 접근 쪽)으로만 보정하며 절대
      # 완화하지 않음 -- v_lead를 낮추는(closing을 더 크게 보는) 방향일 때만
      # 적용.
      if (not lead.radar) and vision_dRel_rate is not None:
        measured_v_lead = v_ego + vision_dRel_rate
        if measured_v_lead < v_lead:
          v_lead = measured_v_lead

      # margin-based accel damping: with enough following-distance margin AND
      # enough TTC margin, ignore lead accel/decel jitter and hold steady;
      # response ramps back to full as either margin shrinks toward its
      # respective safety threshold (min() of the two -- distance ratio alone
      # under-damps at highway speed since desired_distance itself grows with
      # v_ego, see LEAD_ACCEL_TTC_GATE_* comment above). self.desired_distance
      # is the previous cycle's value (one 0.05s-old sample) -- negligible staleness.
      dist_w = margin_accel_weight(x_lead, self.desired_distance)
      if self._launch_bypass_active:
        # 정차->출발 구간(FINDINGS.md 45차): 38차 TTC 게이트를 완전히 우회하고,
        # 38차 patch 이전과 동일하게 dist_w만으로 damping을 결정한다. 이 구간은
        # v_ego<=v_lead(=closing<=0.1)가 정상적으로 발생하는 구간이라 ttc_accel_weight()가
        # 앞차의 실측 가속(aLeadK)을 통째로 지워버리는 부작용이 생기기 때문.
        ttc_w = 1.0
      else:
        ttc_w = ttc_accel_weight(x_lead, v_ego, v_lead)
      w = min(dist_w, ttc_w)
      closing = v_ego - v_lead
      ttc_now = x_lead / closing if closing > 0.1 else float('inf')
      # 58차 2번: 저속 구간 한정, 앞차가 이미 강하게 감속 중이면 TTC 문턱 도달
      # 여부와 무관하게 danger override와 동일하게 취급 (위 LOW_SPEED_STRONG_DECEL_*
      # 주석 참고). 이 조건 자체가 v_ego 게이트로 닫혀 있으므로 게이트 밖(고속/
      # 일반 주행)에서는 아래 두 분기 중 하나도 True가 될 수 없어 기존 동작과
      # 동일하다.
      low_speed_strong_lead_decel = (
        v_ego <= LOW_SPEED_STRONG_DECEL_V_EGO_GATE
        and a_lead <= LOW_SPEED_STRONG_DECEL_A_LEAD_THRESH
      )
      lead0_danger_now = ttc_now <= LEAD_ACQ_TTC_DANGER or low_speed_strong_lead_decel
      if is_lead0:
        # 67차(방안G)가 a_change_cost 부스트 게이트에서 참조하는 최신 위험
        # 판정 -- leadOne 호출 시에만 갱신(leadTwo는 부스트와 무관).
        self._lead0_danger_active = lead0_danger_now
      if lead0_danger_now:
        # 실제 위험(TTC<=2.5s, radard LeadBlend danger_hold와 동일 임계값) 또는
        # 저속+앞차 강한감속이면 rise-rate 제한 없이 즉시 무감쇠 -- 안전 반응을
        # 늦추지 않는다. launch bypass 여부와 무관하게 항상 최우선.
        w = 1.0
      elif self._launch_bypass_active:
        # bypass 중엔 39차 rise-rate 제한도 함께 우회 -- 이 제한은 저속에서 TTC가
        # 급붕괴하며 생기는 급정지 느낌 방지가 목적인데, 출발 가속 구간에서는
        # 오히려 매끈한 가속 상승을 지연시키는 부작용만 남는다.
        pass
      elif w > self._lead_accel_weight_prev:
        # rising edge(감쇠가 풀리는 방향)만 사이클당 변화폭 제한 -- 저속에서 TTC가
        # 급격히 붕괴하며 weight가 순간적으로 튀어 aLeadK 누적값이 한꺼번에
        # 풀려나오는 것을 막는다. 내려가는 방향(더 감쇠)은 그대로 즉시 반영.
        w = min(w, self._lead_accel_weight_prev + LEAD_ACCEL_WEIGHT_RISE_RATE * self.dt)
      self._lead_accel_weight_prev = w
      a_lead *= w
    else:
      # Fake a fast lead car, so mpc can keep running in the same mode
      x_lead = 50.0
      v_lead = v_ego + 10.0
      a_lead = 0.0
      a_lead_tau = _LEAD_ACCEL_TAU
      # 리드가 없는 사이클엔 이전 리드와 무관하므로 다음 리드 재획득 시 불필요한
      # rise-rate 제한이 이어지지 않도록 리셋(안전측 기본값 1.0으로).
      self._lead_accel_weight_prev = 1.0
      if is_lead0:
        # 리드가 없으면 위험 신호도 없음 -- 부스트 게이트가 stale True에
        # 걸려 있지 않도록 안전측(무위험)으로 리셋.
        self._lead0_danger_active = False

    # MPC will not converge if immediate crash is expected
    # Clip lead distance to what is still possible to brake for
    min_x_lead = ((v_ego + v_lead)/2) * (v_ego - v_lead) / (-ACCEL_MIN * 2)
    x_lead = np.clip(x_lead, min_x_lead, 1e8)
    v_lead = np.clip(v_lead, 0.0, 1e8)
    a_lead = np.clip(a_lead, -10., 5.)

    if a_lead < -2.0 and j_lead > 0.5:
      a_lead = a_lead + j_lead
      a_lead = min(a_lead, -0.5)
      a_lead_tau = max(a_lead_tau, 1.5)

    lead_xv = self.extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau, j_lead)
    return lead_xv, v_lead

  def set_accel_limits(self, min_a, max_a):
    # TODO this sets a max accel limit, but the minimum limit is only for cruise decel
    # needs refactor
    self.cruise_min_a = min_a
    self.max_a = max_a

  def update(self, carrot, reset_state, radarstate, v_cruise, x, v, a, j, personality=log.LongitudinalPersonality.standard,
             lane_change_blinker_active=False):
    v_ego = self.x0[1]
    a_ego = self.x0[2]
    t_follow = carrot.get_T_FOLLOW(personality, v_ego, a_ego)
    self.status = radarstate.leadOne.status or radarstate.leadTwo.status

    # 66차/67차(방안G): discontinuity 부스트 타이머 감쇠(매 사이클, lane_change
    # hold 타이머와 동일 패턴) -- arm은 아래 discontinuity 트리거 지점에서.
    self._discontinuity_jerk_boost_timer = max(0.0, self._discontinuity_jerk_boost_timer - self.dt)

    # lead-acquisition ramp bookkeeping: source (vision-first or radar-first)
    # doesn't matter here -- only radarstate.leadOne.status is watched, so a
    # radar-first lock ramps exactly the same way a vision-first one does.
    lead_one_status_now = bool(radarstate.leadOne.status)
    if lead_one_status_now:
      self._lead_absent_timer = 0.0
      self._lead_present_run_timer += self.dt
      if not self._lead_acq_ramp_started:
        # not yet confirmed as a real (non-blip) lead -- don't start ramping
        # until it's been continuously present for LEAD_ACQ_CONFIRM_TIME.
        if self._lead_present_run_timer >= LEAD_ACQ_CONFIRM_TIME:
          self._lead_acq_ramp_started = True
          self._lead_acq_timer = 0.0
      else:
        self._lead_acq_timer += self.dt
    else:
      self._lead_absent_timer += self.dt
      if self._lead_absent_timer > LEAD_ACQ_LOSS_GRACE_TIME:
        # genuinely gone (not just a brief source-switch/miss blip) -- reset
        # everything so the next real lead starts a fresh ramp from zero.
        self._lead_present_run_timer = 0.0
        self._lead_acq_ramp_started = False
        self._lead_acq_timer = 0.0
        self._vision_dRel_prev = None
        self._vision_dRel_rate = 0.0
        self._vision_dRel_rate_window.clear()
        self._dRel_raw_history.clear()
      # else: within grace -- freeze state as-is, don't reset. If the lead
      # reappears next cycle, an already-started ramp just keeps going from
      # where it left off instead of restarting.

    # vision-only closing-rate cross-check bookkeeping (see
    # VISION_CLOSING_RATE_* above). Only accumulated while radar hasn't
    # locked on yet -- once radar confirms, its own vRel is already accurate
    # and this cross-check is no longer needed (also avoids the dRel jump at
    # the vision->radar handoff itself being misread as a closing-rate spike),
    # so that case resets immediately (no grace).
    #
    # A brief leadStatus=False blip (model missing a frame, not a real loss)
    # must NOT reset this -- same LEAD_ACQ_LOSS_GRACE_TIME grace as the ramp
    # bookkeeping above, and for the same reason: real drive logs show
    # leadStatus flickering False for 0.15~0.4s multiple times inside what is
    # otherwise one continuous vision-only track (route2 260820 seg5 t~1647,
    # route1 seg9 t~1078 -- see FINDINGS.md 22차). Before this fix, this block
    # zeroed self._vision_dRel_rate on every such blip regardless of the grace
    # timer above, so the low-pass filter kept restarting from 0 and never
    # got the full continuous-tracking duration to converge -- silently
    # undermining the ramp bookkeeping's own "freeze state within grace"
    # intent (lines above). This was NOT caught by the 17차 validation
    # because those 6 events happened to have unbroken leadStatus runs.
    if lead_one_status_now and not radarstate.leadOne.radar:
      dRel_now = float(radarstate.leadOne.dRel)

      # 61차 계속(방안 C): cutin 불연속 급락 감지 -- rate 계산/필터와는
      # 별개로, 원본 dRel 값 자체의 최근 윈도우 양끝 차이를 본다(위
      # DREL_DISCONTINUITY_* 주석 참고). rate/중앙값 필터는 "지속적인
      # 접근"과 "일회성 스냅"을 구분하는 게 목적이라 이 급락 자체를
      # 완만하게 흡수해버릴 수 있어, 원본 값으로 별도 판정한다.
      self._dRel_raw_history.append(dRel_now)
      if (len(self._dRel_raw_history) == self._dRel_raw_history.maxlen and
          (self._dRel_raw_history[-1] - self._dRel_raw_history[0]) < -DREL_DISCONTINUITY_DROP_THRESH):
        # 측면진입(cutin)/타겟 전환 의심 -- 기존에 이미 검증된 신규등록
        # suppress 메커니즘을 재사용(새 코드경로 추가 없음). 이 프레임부터
        # NEW_LEAD_VLEAD_CORRECTION_SUPPRESS_S 동안 v_lead 직접보정/frac_rate
        # 등이 자동으로 유예된다. TTC danger override(process_lead 내
        # ttc_now<=LEAD_ACQ_TTC_DANGER)는 이 리셋과 무관하게 항상 그대로 작동.
        self._lead_acq_timer = 0.0
        # 66차/67차(방안G): 같은 discontinuity 트리거 지점에서 저크비용 부스트도
        # 함께 arm -- 목표거리(v_lead 보정)와는 별개로 도달 속도만 완만하게.
        self._discontinuity_jerk_boost_timer = DISCONTINUITY_JERK_COST_BOOST_S

      if self._vision_dRel_prev is not None:
        raw_rate = (dRel_now - self._vision_dRel_prev) / max(self.dt, 1e-3)
        # 곡선 노이즈 스냅 클램프 (VISION_CLOSING_RATE_MAX_PLAUSIBLE 위 주석 참고) --
        # 접근 방향(음수)만 클램프한다. 멀어지는 방향(양수) 스냅은 급브레이크
        # 유발 리스크가 없으므로 그대로 둔다.
        #
        # 방안 E (VISION_RATE_REF_MARGIN 위 주석 참고, 63차 계속9/10): 절대값
        # 클램프만으론 트랙전환성 점프(cut-in 등)를 다 못 거르므로, leadVLead
        # 기반 참고 closing rate 대비 상대적 타당성 클램프를 추가로 적용한다.
        ref_rate = -(v_ego - float(radarstate.leadOne.vLead))
        plausible_min = ref_rate - VISION_RATE_REF_MARGIN
        raw_rate_clamped = max(raw_rate, -VISION_CLOSING_RATE_MAX_PLAUSIBLE, plausible_min)
        self._vision_dRel_rate_window.append(raw_rate_clamped)
        # 중앙값을 필터 입력으로 사용 -- 한두 프레임짜리 스냅-복귀는 윈도우 내
        # 다수결에 밀려 걸러지고, 지속적인 접근은 중앙값에도 그대로 반영된다.
        rate_for_filter = float(np.median(self._vision_dRel_rate_window))
        alpha = float(np.clip(self.dt / VISION_CLOSING_RATE_TAU, 0.0, 1.0))
        self._vision_dRel_rate = self._vision_dRel_rate * (1. - alpha) + rate_for_filter * alpha
      self._vision_dRel_prev = dRel_now
    elif lead_one_status_now and radarstate.leadOne.radar:
      # radar just confirmed -- reset immediately, no grace (see comment above).
      # 72차(방안 I): 부기 리셋 전에 먼저 락온 전환(False->True) 엣지인지
      # 확인하고, 엣지라면 직전 프레임 vRel 대비 이번 프레임 vRel의 불연속
      # 여부를 판정한다(위 RADAR_HANDOFF_VREL_JUMP_THRESH 주석 참고). 이미
      # 락온이 유지 중인 프레임(엣지 아님)에는 매 사이클 재트리거되지
      # 않도록 self._prev_lead_radar로 엣지만 골라낸다.
      if (not self._prev_lead_radar) and self._prev_lead_vRel is not None:
        vRel_now = float(radarstate.leadOne.vRel)
        if (vRel_now - self._prev_lead_vRel) < -RADAR_HANDOFF_VREL_JUMP_THRESH:
          self._discontinuity_jerk_boost_timer = DISCONTINUITY_JERK_COST_BOOST_S
      self._vision_dRel_prev = None
      self._vision_dRel_rate = 0.0
      self._vision_dRel_rate_window.clear()
      self._dRel_raw_history.clear()
    elif self._lead_absent_timer > LEAD_ACQ_LOSS_GRACE_TIME:
      # lead genuinely gone (grace exceeded) -- reset for a fresh start next time.
      self._vision_dRel_prev = None
      self._vision_dRel_rate = 0.0
      self._vision_dRel_rate_window.clear()
      self._dRel_raw_history.clear()
    # else: brief status blip within grace -- freeze dRel_prev/rate as-is.
    # If the lead reappears next cycle with radar still unlocked, the rate
    # estimate resumes accumulating instead of restarting from zero. Note
    # dRel_prev is left stale across the blip (not extrapolated forward like
    # LeadBlend does for the lost-lead case in radard.py) -- when tracking
    # resumes, one raw_rate sample will be computed across the gap duration
    # instead of one DT_MDL step, which the low-pass filter absorbs the same
    # way it absorbs any other single noisy sample.

    # 72차(방안 I): 다음 프레임의 락온 엣지/vRel 불연속 판정을 위해 이번
    # 프레임 상태를 저장. status가 False인 프레임(리드 완전 유실)에서는
    # 갱신하지 않고 그대로 두어, 짧은 blip 이후 재등장 시에도 마지막으로
    # 유효했던 vRel/radar 상태가 남아있게 한다 -- 리드가 진짜로 사라졌다가
    # 완전히 새로 등록되는 경우는 _lead_acq_timer가 다시 짧아지므로 별도
    # 신규등록 보호(NEW_LEAD_VLEAD_CORRECTION_SUPPRESS_S)가 이미 담당한다.
    if lead_one_status_now:
      self._prev_lead_radar = bool(radarstate.leadOne.radar)
      self._prev_lead_vRel = float(radarstate.leadOne.vRel)

    if radarstate.leadOne.status:
      j_lead = radarstate.leadOne.jLead
      self.j_lead = j_lead * 0.1 + self.j_lead * 0.9
    else:
      self.j_lead = 0.0

    # 58차 1번: process_lead에 넘길 vision_dRel_rate는 leadOne에 대해서만
    # 의미가 있음(_vision_dRel_rate 자체가 leadOne 기준으로만 부기됨, 위
    # "vision-only closing-rate cross-check bookkeeping" 참고) -- leadTwo에는
    # None으로 전달해 보정 없이 기존 동작 유지.
    # 60차 계속: 아래 두 취약 구간에서는 v_lead 직접보정을 유예(패치 이전
    # 로직인 lead.vLead 그대로 사용으로 복귀) -- (1) 리드 신규등록 직후
    # catch-up 구간, (2) 차선변경(blinker) 중 + 종료 후 hold 구간.
    if lane_change_blinker_active:
      self._lane_change_vlead_hold_timer = LANE_CHANGE_VLEAD_CORRECTION_HOLD_S
    else:
      self._lane_change_vlead_hold_timer = max(0.0, self._lane_change_vlead_hold_timer - self.dt)
    vlead_correction_suppressed = (
      self._lead_acq_timer < NEW_LEAD_VLEAD_CORRECTION_SUPPRESS_S or
      lane_change_blinker_active or
      self._lane_change_vlead_hold_timer > 0.0
    )
    vision_rate_for_lead0 = (self._vision_dRel_rate
                              if self._lead_acq_timer >= VISION_CLOSING_RATE_MIN_TIME and not vlead_correction_suppressed
                              else None)
    lead_xv_0, lead_v_0 = self.process_lead(radarstate.leadOne, np.clip(self.j_lead * carrot.j_lead_factor, -1.0, 1.0),
                                             vision_dRel_rate=vision_rate_for_lead0, is_lead0=True)
    lead_xv_1, _ = self.process_lead(radarstate.leadTwo, 0.0)

    mode = self.mode
    comfort_brake = carrot.comfort_brake
    stop_distance = carrot.stop_distance
    
    if mode == 'blended':
      stop_x = 1000.0
    else:
      v_cruise, stop_x, mode = carrot.v_cruise, carrot.stop_dist, carrot.mode
      desired_distance = desired_follow_distance(v_ego, lead_v_0, comfort_brake, stop_distance, t_follow)
      t_follow = carrot.dynamic_t_follow(t_follow, radarstate.leadOne, desired_distance, self.prev_a)

    # t_follow의 증가 방향 레이트리미터(apply_t_follow)는 이 사이클의 최종 t_follow 값에
    # 대해 정확히 한 번만 적용한다. get_T_FOLLOW/dynamic_t_follow 내부에서 각자 호출하면
    # 차선변경 등으로 값이 줄었을 때 그 줄어든 값이 다음 사이클 기준선이 되어 계속
    # 누적으로 더 줄어드는(0으로 수렴하는) 버그가 생긴다.
    t_follow = carrot.apply_t_follow(t_follow)

    # To estimate a safe distance from a moving lead, we calculate how much stopping
    # distance that lead needs as a minimum. We can add that to the current distance
    # and then treat that as a stopped car/obstacle at this new distance.
    lead_0_obstacle = lead_xv_0[:,0] + get_stopped_equivalence_factor(lead_xv_0[:,1])
    lead_1_obstacle = lead_xv_1[:,0] + get_stopped_equivalence_factor(lead_xv_1[:,1])
    
    self.desired_distance = desired_follow_distance(v_ego, lead_v_0, comfort_brake, stop_distance, t_follow)

    # apply the lead-acquisition proactive floor (see LEAD_ACQ_RAMP_TIME /
    # LEAD_ACQ_TTC_DANGER) only in 'acc' mode, only once the acquisition has
    # been confirmed (not a one-off blip).
    # 67차(방안G): 아래 블록 밖(조건 미충족)에서도 항상 참조 가능하도록
    # 기본값을 먼저 정의 -- a_change_cost 부스트 게이트가 이 값을 사용.
    frac = 0.0
    if mode == 'acc' and radarstate.leadOne.status and v_ego >= LEAD_ACQ_MIN_V_EGO and self._lead_acq_ramp_started:
      # elapsed-time component: 0 -> 1 over LEAD_ACQ_RAMP_TIME, then fully
      # released (0) once the window closes -- a genuinely far/safe lead
      # isn't held to the tighter virtual distance forever.
      if self._lead_acq_timer <= LEAD_ACQ_RAMP_TIME:
        frac_time = float(np.clip(self._lead_acq_timer / LEAD_ACQ_RAMP_TIME, 0.0, 1.0))
      else:
        frac_time = 0.0

      # live TTC component: recomputed every frame from the *current*
      # dRel/vRel (not a one-time snapshot at acquisition), so if the true
      # closing rate only reveals itself mid-ramp -- exactly the vision
      # under-estimation failure mode this whole feature exists for -- the
      # response snaps up immediately instead of waiting for frac_time to
      # catch up. Same TTC<2.5s danger threshold as LeadBlend in radard.py.
      lead_v_rel = radarstate.leadOne.vRel
      if lead_v_rel < -0.1:
        ttc_now = radarstate.leadOne.dRel / max(-lead_v_rel, 0.1)
      else:
        ttc_now = 999.0  # not closing (or moving away) -- no urgency

      # vision-only closing-rate cross-check: the model-predicted vRel above
      # can under-report true closing speed for a distant, low-confidence
      # vision lead (see VISION_CLOSING_RATE_* comment). If we've tracked it
      # continuously long enough to trust the dRel-derivative, take whichever
      # of the two says it's more dangerous -- this never relaxes braking,
      # only tightens it when the raw vRel is hiding real risk.
      if (not radarstate.leadOne.radar) and self._lead_acq_timer >= VISION_CLOSING_RATE_MIN_TIME:
        if self._vision_dRel_rate < -0.1:
          ttc_dRel = radarstate.leadOne.dRel / max(-self._vision_dRel_rate, 0.1)
          ttc_now = min(ttc_now, ttc_dRel)

      frac_ttc = float(np.clip((LEAD_ACQ_TTC_CAUTION - ttc_now) / (LEAD_ACQ_TTC_CAUTION - LEAD_ACQ_TTC_DANGER), 0.0, 1.0))

      # vision-only closing-rate absolute gate (see VISION_CLOSING_RATE_GATE_*
      # above): at long range, TTC = dRel / rate never crosses the caution
      # threshold even when the closing rate itself is already dangerous
      # (dRel large keeps TTC large regardless of rate) -- frac_ttc alone
      # structurally can't catch that case. This component looks at the
      # (curve-noise-filtered) rate directly, independent of distance, so a
      # genuinely fast approach still raises the floor even while still far
      # away. Same continuous-tracking gate (VISION_CLOSING_RATE_MIN_TIME) as
      # the TTC cross-check, and only vision-only (no radar lock yet).
      frac_rate = 0.0
      if (not radarstate.leadOne.radar) and self._lead_acq_timer >= VISION_CLOSING_RATE_MIN_TIME:
        frac_rate = float(np.clip(
          (VISION_CLOSING_RATE_GATE_CAUTION - self._vision_dRel_rate) /
          (VISION_CLOSING_RATE_GATE_CAUTION - VISION_CLOSING_RATE_GATE_DANGER), 0.0, 1.0))

      # take the strongest of the three -- this stays a pure floor and never
      # softens braking versus any component alone.
      frac = max(frac_time, frac_ttc, frac_rate)
      if frac > 0.0:
        # virtual reference: an object exactly at the standard current-speed
        # follow distance, assumed to move with ego (matching speed) across the
        # horizon -- same construction as cruise_obstacle below, just anchored
        # to v_ego instead of v_cruise.
        virtual_v_traj = np.full(N + 1, v_ego)
        virtual_obstacle = np.cumsum(T_DIFFS * virtual_v_traj) + get_safe_obstacle_distance(virtual_v_traj, t_follow, comfort_brake, stop_distance)
        # ramp from "no effect" (frac=0, cap == raw) to "full floor" (frac=1, cap == virtual)
        floor_cap = lead_0_obstacle + (virtual_obstacle - lead_0_obstacle) * frac
        lead_0_obstacle = np.minimum(lead_0_obstacle, floor_cap)

    self.params[:,0] = ACCEL_MIN if not reset_state else a_ego
    # negative accel constraint causes problems because negative speed is not allowed
    self.params[:,1] = max(0.0, self.max_a if not reset_state else a_ego)

    # Update in ACC mode or ACC/e2e blend
    if mode == 'acc':
      #self.params[:,5] = LEAD_DANGER_FACTOR
      # Fake an obstacle for cruise, this ensures smooth acceleration to set speed
      # when the leads are no factor.
      v_lower = v_ego + (T_IDXS * self.cruise_min_a * 1.05)
      # TODO does this make sense when max_a is negative?
      v_upper = v_ego + (T_IDXS * self.max_a * 1.05)
      v_cruise_clipped = np.clip(v_cruise * np.ones(N+1),
                                 v_lower,
                                 v_upper)
      cruise_obstacle = np.cumsum(T_DIFFS * v_cruise_clipped) + get_safe_obstacle_distance(v_cruise_clipped, t_follow, comfort_brake, stop_distance)

      adjust_dist = carrot.trafficStopDistanceAdjust if v_ego > 0.1 else -2.0
      if 50 < stop_x + adjust_dist < cruise_obstacle[0]:
        stop_x = cruise_obstacle[0] - adjust_dist
      x2 = stop_x * np.ones(N+1) + adjust_dist

      x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle, cruise_obstacle, x2])
      self.source = SOURCES[np.argmin(x_obstacles[0])]

      if v_cruise == 0 and self.source == 'cruise':
        self.params[:,0] = - carrot.autoNaviSpeedDecelRate
      #elif self.source in ['cruise', 'e2e']:
      #  self.params[:,0] = - COMFORT_BRAKE

      # These are not used in ACC mode
      x[:], v[:], a[:], j[:] = 0.0, 0.0, 0.0, 0.0

      if radarstate.leadOne.status:
        base_a_change_cost = np.interp(abs(self.j_lead), [0.3, 2.0], [A_CHANGE_COST, 20])
      else:
        base_a_change_cost = A_CHANGE_COST

      # 66차/67차(방안G): discontinuity 직후 부스트 윈도우 내 + danger
      # override/저속강한감속 미발동(process_lead) + proactive floor(frac)
      # 미발동 -- 셋 다 무위험일 때만 저크비용을 한시적으로 강화한다. 하나라도
      # 위험을 감지하면 즉시 기존 j_lead 기반 식(base_a_change_cost)으로 복귀.
      if (self._discontinuity_jerk_boost_timer > 0.0
          and not self._lead0_danger_active
          and frac <= 0.0):
        self.a_change_cost = DISCONTINUITY_JERK_COST_BOOST
      else:
        self.a_change_cost = base_a_change_cost

      #safe_distance = lead_0_obstacle[0] - get_safe_obstacle_distance(v_ego, comfort_brake, stop_distance)
      self.lead_danger_factor = LEAD_DANGER_FACTOR #np.interp(safe_distance, [-30.0, 0.0], [0.9, LEAD_DANGER_FACTOR]) # 이걸적용하니, 사고방지턱 감속시 너무 급정거하는것 같음.
      self.params[:,5] = self.lead_danger_factor
      
    elif mode == 'blended':
      self.params[:,5] = 1.0

      x_obstacles = np.column_stack([lead_0_obstacle,
                                     lead_1_obstacle])
      cruise_target = T_IDXS * np.clip(v_cruise, v_ego - 2.0, 1e3) + x[0]
      xforward = ((v[1:] + v[:-1]) / 2) * (T_IDXS[1:] - T_IDXS[:-1])
      x = np.cumsum(np.insert(xforward, 0, x[0]))

      x_and_cruise = np.column_stack([x, cruise_target])
      x = np.min(x_and_cruise, axis=1)

      self.source = 'e2e' if x_and_cruise[1,0] < x_and_cruise[1,1] else 'cruise'

    else:
      raise NotImplementedError(f'Planner mode {self.mode} not recognized in planner update')

    self.yref[:,1] = x
    self.yref[:,2] = v
    self.yref[:,3] = a
    self.yref[:,5] = j
    for i in range(N):
      self.solver.set(i, "yref", self.yref[i])
    self.solver.set(N, "yref", self.yref[N][:COST_E_DIM])

    self.params[:,2] = np.min(x_obstacles, axis=1)
    self.params[:,3] = np.copy(self.prev_a)
    self.params[:,4] = t_follow
    self.params[:,6] = comfort_brake
    self.params[:,7] = stop_distance

    self.t_follow = t_follow

    self.run()
    if (np.any(lead_xv_0[FCW_IDXS,0] - self.x_sol[FCW_IDXS,0] < CRASH_DISTANCE) and
            radarstate.leadOne.modelProb > 0.9):
      self.crash_cnt += 1
    else:
      self.crash_cnt = 0

    # Check if it got within lead comfort range
    # TODO This should be done cleaner
    if self.mode == 'blended':
      if any((lead_0_obstacle - get_safe_obstacle_distance(self.x_sol[:,1], t_follow, comfort_brake, stop_distance))- self.x_sol[:,0] < 0.0):
        self.source = 'lead0'
      if any((lead_1_obstacle - get_safe_obstacle_distance(self.x_sol[:,1], t_follow, comfort_brake, stop_distance))- self.x_sol[:,0] < 0.0) and \
         (lead_1_obstacle[0] - lead_0_obstacle[0]):
        self.source = 'lead1'

  def run(self):
    # t0 = time.monotonic()
    # reset = 0
    for i in range(N+1):
      self.solver.set(i, 'p', self.params[i])
    self.solver.constraints_set(0, "lbx", self.x0)
    self.solver.constraints_set(0, "ubx", self.x0)

    self.solution_status = self.solver.solve()
    self.solve_time = float(self.solver.get_stats('time_tot')[0])
    self.time_qp_solution = float(self.solver.get_stats('time_qp')[0])
    self.time_linearization = float(self.solver.get_stats('time_lin')[0])
    self.time_integrator = float(self.solver.get_stats('time_sim')[0])

    # qp_iter = self.solver.get_stats('statistics')[-1][-1] # SQP_RTI specific
    # print(f"long_mpc timings: tot {self.solve_time:.2e}, qp {self.time_qp_solution:.2e}, lin {self.time_linearization:.2e}, \
    # integrator {self.time_integrator:.2e}, qp_iter {qp_iter}")
    # res = self.solver.get_residuals()
    # print(f"long_mpc residuals: {res[0]:.2e}, {res[1]:.2e}, {res[2]:.2e}, {res[3]:.2e}")
    # self.solver.print_statistics()

    for i in range(N+1):
      self.x_sol[i] = self.solver.get(i, 'x')
    for i in range(N):
      self.u_sol[i] = self.solver.get(i, 'u')

    self.v_solution = self.x_sol[:,1]
    self.a_solution = self.x_sol[:,2]
    self.j_solution = self.u_sol[:,0]

    self.prev_a = np.interp(T_IDXS + self.dt, T_IDXS, self.a_solution)

    t = time.monotonic()
    if self.solution_status != 0:
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning(f"Long mpc reset, solution_status: {self.solution_status}")
      self.reset()
      # reset = 1
    # print(f"long_mpc timings: total internal {self.solve_time:.2e}, external: {(time.monotonic() - t0):.2e} qp {self.time_qp_solution:.2e}, \
    # lin {self.time_linearization:.2e} qp_iter {qp_iter}, reset {reset}")


if __name__ == "__main__":
  ocp = gen_long_ocp()
  AcadosOcpSolver.generate(ocp, json_file=JSON_FILE)
  # AcadosOcpSolver.build(ocp.code_export_directory, with_cython=True)
