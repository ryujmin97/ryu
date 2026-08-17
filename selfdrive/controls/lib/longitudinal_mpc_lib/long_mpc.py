#!/usr/bin/env python3
import os
import time
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


def margin_accel_weight(dRel, desired_distance):
  if desired_distance <= 1.0:
    return 1.0
  ratio = dRel / desired_distance
  return float(np.clip((MARGIN_ACCEL_GATE_FULL - ratio) / (MARGIN_ACCEL_GATE_FULL - MARGIN_ACCEL_GATE_NONE), 0.0, 1.0))


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
LEAD_ACQ_RAMP_TIME       = 5.0   # s   : 하한선을 0 -> 100% 로 서서히 적용하는 시간
LEAD_ACQ_MIN_V_EGO       = 3.0   # m/s : 이 속도 미만에서는 적용하지 않음 (정체/크립 구간 노이즈 방지)
LEAD_ACQ_CONFIRM_TIME    = 0.2   # s   : 이 시간 이상 연속 감지되어야 "진짜 인식"으로 보고 램프 시작 (1회성 블립 무시)
LEAD_ACQ_LOSS_GRACE_TIME = 0.5   # s   : 짧은 순간유실은 이 시간까지 봐주고 진행 중인 램프를 그대로 유지


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
  
  def process_lead(self, lead, j_lead):
    v_ego = self.x0[1]
    if lead is not None and lead.status:
      x_lead = lead.dRel
      v_lead = lead.vLead
      a_lead = lead.aLeadK
      a_lead_tau = lead.aLeadTau

      # margin-based accel damping: with enough following-distance margin, ignore
      # lead accel/decel jitter and hold steady; response ramps back to full as
      # margin shrinks toward the safety threshold. self.desired_distance is the
      # previous cycle's value (one 0.05s-old sample) -- negligible staleness.
      a_lead *= margin_accel_weight(x_lead, self.desired_distance)
    else:
      # Fake a fast lead car, so mpc can keep running in the same mode
      x_lead = 50.0
      v_lead = v_ego + 10.0
      a_lead = 0.0
      a_lead_tau = _LEAD_ACCEL_TAU

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

  def update(self, carrot, reset_state, radarstate, v_cruise, x, v, a, j, personality=log.LongitudinalPersonality.standard):
    v_ego = self.x0[1]
    a_ego = self.x0[2]
    t_follow = carrot.get_T_FOLLOW(personality, v_ego, a_ego)
    self.status = radarstate.leadOne.status or radarstate.leadTwo.status

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
      # else: within grace -- freeze state as-is, don't reset. If the lead
      # reappears next cycle, an already-started ramp just keeps going from
      # where it left off instead of restarting.

    if radarstate.leadOne.status:
      j_lead = radarstate.leadOne.jLead
      self.j_lead = j_lead * 0.1 + self.j_lead * 0.9
    else:
      self.j_lead = 0.0

    lead_xv_0, lead_v_0 = self.process_lead(radarstate.leadOne, np.clip(self.j_lead * carrot.j_lead_factor, -1.0, 1.0))
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

    # To estimate a safe distance from a moving lead, we calculate how much stopping
    # distance that lead needs as a minimum. We can add that to the current distance
    # and then treat that as a stopped car/obstacle at this new distance.
    lead_0_obstacle = lead_xv_0[:,0] + get_stopped_equivalence_factor(lead_xv_0[:,1])
    lead_1_obstacle = lead_xv_1[:,0] + get_stopped_equivalence_factor(lead_xv_1[:,1])
    
    self.desired_distance = desired_follow_distance(v_ego, lead_v_0, comfort_brake, stop_distance, t_follow)

    # apply the lead-acquisition proactive floor (see LEAD_ACQ_RAMP_TIME) only
    # in 'acc' mode, only once the acquisition has been confirmed (not a
    # one-off blip), and only while the ramp window is still open.
    if (mode == 'acc' and radarstate.leadOne.status and v_ego >= LEAD_ACQ_MIN_V_EGO
        and self._lead_acq_ramp_started and self._lead_acq_timer <= LEAD_ACQ_RAMP_TIME):
      frac = float(np.clip(self._lead_acq_timer / LEAD_ACQ_RAMP_TIME, 0.0, 1.0))
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
        self.a_change_cost = np.interp(abs(self.j_lead), [0.3, 2.0], [A_CHANGE_COST, 20])
      else:
        self.a_change_cost = A_CHANGE_COST

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
