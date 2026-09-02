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
import zmq
from datetime import datetime
import traceback
from typing import Any, Dict, List, Optional

from aiohttp import web
import asyncio

from ftplib import FTP
from cereal import log
import urllib.request
import urllib.error
import ssl

import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper, set_core_affinity
from openpilot.common.params import Params
from openpilot.common.filter_simple import MyMovingAverage
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.system.hardware import PC, TICI
from openpilot.selfdrive.navd.helpers import Coordinate
from opendbc.car.common.conversions import Conversions as CV

from openpilot.selfdrive.carrot.carrot_serv import CarrotServ

from openpilot.common.gps import get_gps_location_service

NetworkType = log.DeviceState.NetworkType

################ CarrotNavi
## 국가법령정보센터: 도로설계기준
#V_CURVE_LOOKUP_BP = [0., 1./800., 1./670., 1./560., 1./440., 1./360., 1./265., 1./190., 1./135., 1./85., 1./55., 1./30., 1./15.]
#V_CRUVE_LOOKUP_VALS = [300, 150, 120, 110, 100, 90, 80, 70, 60, 50, 45, 35, 30]
V_CURVE_LOOKUP_BP = [0., 1./800., 1./670., 1./560., 1./440., 1./360., 1./265., 1./190., 1./135., 1./85., 1./55., 1./30., 1./25.]
V_CRUVE_LOOKUP_VALS = [300, 150, 120, 110, 100, 90, 80, 70, 60, 50, 40, 15, 5]
# [132차] carrot_navi_route()가 호출되는 broadcast_version_info()의
# Ratekeeper(20) 주기(초). Hypothesis C(131차) 대응 프레임간 램프
# 리미터에서 accel_limit_kmh와 곱해 "이 주기 동안 물리적으로 허용되는
# 최대 속도 변화"를 계산하는 데 사용.
ROUTE_SPEED_LOOP_DT = 0.05

# [157차, 사용자 제안 재설계] 곡률이 이 값 미만이면 "진짜 노이즈 수준의
# 직선"으로 보고 nRoadLimitSpeed로 플로어(=사실상 무제한)한다. 기존
# 0.02(R≈50m)는 V_CURVE_LOOKUP 테이블상 이미 curvature 0.009~0.018
# 구간이 45~56km/h급 커브로 계산되는데도 그 값을 통째로 버리고
# 도로제한속도로 되돌리는 과도한 임계값이었음(156차가 실측으로 확인 --
# 연속 완만한 굽이길 curvature 0.002~0.013이 전부 이 플로어에 걸려
# route= 표시가 16초+ 고정, 실제로는 vturn이 대신 감속 중이었음).
# 0.001(R≈1000m)로 낮춰 "진짜 직선 GPS 노이즈"만 걸러내고, 그보다 조금이라도
# 뚜렷한 곡률은 테이블 값을 그대로 신뢰한다. 검증: devnotes
# toolkit/sim_route_apex_redesign.py(157차, 직선도로 시나리오 오탐 없음
# 확인, PASS).
ROUTE_CURVE_NEGLIGIBLE_THRESHOLD = 0.001

# [147차] carrot_navi_route()의 곡률 계산 chord(=sample*10m) 보조 샘플.
# 기존 sample=4(40m chord)는 장거리 lookahead 매크로 형상 파악용으로
# 그대로 유지하되, 같은 지점에서 이 값(1=10m, 네이티브 리샘플 해상도)
# 으로도 3점 곡률을 한 번 더 계산해 더 급한(=speed_cap이 더 낮은) 쪽을
# 채택한다. 실측 naviPaths(147차, route 우회전 실주행 로그)로 40m
# chord 단독은 실제 R≈27m급 교차로 커브를 R≈110m급으로 평활화해
# curvature<0.02 임계값 아래로 숨겨 nRoadLimitSpeed(사실상 무제한)
# 클램프가 걸리는 것을 확인. 직선 구간(같은 로그, steer~0 122포인트)
# 에서는 sample=1 재계산 curvature도 임계값(0.02) 미도달로 오탐 없음.
# 89/90차는 이 문제를 raw navi_points 로그 부재로 직접검증 못하고
# desiredCurvature(모델 자신의 이미 평활화된 출력) 적분 재구성이라는
# 순환논리로 "chord 축소 효과 미미"라 오판했었음(devnotes FINDINGS.md
# 147차 참고, 근본 원인은 raw navi_points가 아니라 carrotMan.naviPaths
# 필드가 이미 발행 중이었는데 extract_log.py가 뽑지 않았던 것).
ROUTE_CURVATURE_FINE_SAMPLE = 1

# [196차, 179차후속2 게이트 폐기] 179차후속2가 도입했던 상대적 심각도
# 게이트(ROUTE_APEX_RELATIVE_SEVERITY_RATIO, 아래 carrot_navi_route()
# 참고)를 제거했다. 연속곡선을 1차->2차 순서로 처리하는 설계(사용자
# 원본 설계문서 `곡선_가감속_코딩.txt` 5번)를 위해서는 근접 감속필요
# 지점을 게이트 없이 무조건 우선해야 하기 때문. 이 게이트가 막고
# 있던 근접 미세잡음 오탐(179차 계속, sharpest 대비 최대 9.72km/h
# 덜 안전한 출력 -- route 00000374, t≈753.5~759.3 실측)이 이 변경으로
# 다시 열릴 수 있으므로 실차 검증 필수(196차 실차 검증: 미실시). 게이트
# 도입 배경/실험(유닛테스트 15/15 PASS 등)은 devnotes FINDINGS.md
# 179차/179차후속2 항목에 그대로 보존되어 있다(삭제하지 않음, §24).

# [162차, 방향2 - 보수적 완화] carrot_serv.py::_update_gps()가 계산하는
# position_dt_since_fix(마지막 실제 위치 fix 이후 데드레커닝 경과시간)가
# 이 값을 넘으면 위치추정을 신뢰할 수 없는 상태로 보고, route_speed
# 램프리미터가 "완화(속도 상향)" 방향으로는 움직이지 못하게 동결한다.
# 배경(FINDINGS.md 162차): route aeeed9e4a5 seg3 실측에서 앱/폰 GPS
# 갱신이 11초간 끊기며 estimate_position() 데드레커닝이 옛 헤딩(296.0°
# 고정)으로 계속 직진 외삽 -> 실제 급우회전(steer 최대 -121.9°) 중인데
# curvature 계산이 "직선"으로 오판(위치오차 최대 28m 누적)해 route_speed가
# 300 쪽으로 완화되며 상승. 이 게이트는 근본원인(헤딩 오차) 자체를 고치는
# 것(방향1, livePose 자세데이터 보정)이 아니라, 그 오판이 "제약 해제"
# 방향으로 새는 것만 막는 보수적 완화다 -- 하강(더 낮은 속도로 감속)
# 방향은 그대로 허용하므로 실제 커브를 늦게라도 정상 감지하면 여전히
# 반응한다. 이 구간에서도 vturn(비전)이 이미 안전하게 인계받고 있음이
# 실측으로 확인됨(FINDINGS.md 162차) -- 안전 회귀 위험 없이 route의
# 잘못된 "완화" 오탐만 억제하는 목적.
# 3.0s는 carrot_serv.py의 gps_updated_navi/gps_updated_phone 신선도
# 판정과 동일한 관례값(_update_gps() L713-714)을 그대로 재사용.
# 사전검증: devnotes toolkit/sim_route_position_uncertainty_gate.py(162차).
#
# [167차, 병행조건 좁힘] 166차 헤딩보정(방향1, CC.orientationNED 델타앵커링)이
# 실차검증 전 단계에서 이 게이트(방향2)와 병행하기로 결정됨(사용자 지시).
# 다만 방향1이 정상 동작 중(cc_pose_valid=True)이면 방향1 자체가 이 문제를
# 이미 해결하므로 방향2까지 겹쳐 켜두면 완화(상승) 방향이 불필요하게
# 과도 억제됨(트레이드오프 논의 FINDINGS.md 167차 참고) -- 따라서 실제
# 적용부(아래 사용처)에서는 cc_pose_valid=False(방향1이 무력화되는 폴백,
# 예: 캘리브레이션 미완료)일 때만 이 게이트가 발동하도록 조건을 좁혔다.
ROUTE_POSITION_UNCERTAIN_DT_S = 3.0

# [199차, 설계 A v2->v3] vEgo 기반 동적 감속(설계 A) 재검토 배경.
#
# 198차에서 확인된 사실(devnotes FINDINGS.md/WIP.md 198차 참고, 이하 요약):
# calculate_current_speed()의 raw out_speed는 항상 197차 그대로 유지하고,
# 132/173차 프레임간 램프리미터의 "하강 상한(accel_limit_kmh)"만 그 순간
# 실제로 필요한 감속도(required_decel, vEgo 기반)로 동적 부스트하면(v2),
# "이미 램프가 걸린 채 더 급한 2차 apex로 갱신"되는 뒤늦은 발견 상황에서
# 오버슈트를 줄일 수 있다(149/150차가 시도해 NEGATIVE였던 "out_speed
# 자체를 올린다" 방향과는 반대로, out_speed는 그대로 두고 램프만 따라잡게
# 하는 조합이라 목표를 완화하는 부작용이 구조적으로 불가능함).
#
# 그러나 198차 v2는 "연속되는 굽이길(winding road)에서 apex_dist가 거의
# 항상 정확히 distance_interval(=10m, lookahead 상대배열의 첫 샘플)로
# 구조적으로 고정된다"는 FAIL2를 남겼다 -- candidates[0](위 179/196차,
# "road_limit 미만인 가장 가까운 지점") 선택 방식 자체가 원인이라, 이
# apex_dist 값만으로는 "뒤늦게 발견된 진짜 급커브"와 "정상 주행 중 매
# 프레임 갱신되는 다음 곡률 샘플"을 구분할 수 없다(devnotes toolkit/
# sim_route_vego_required_decel_v2.py 실측: winding road 600m 전체에서
# apex_dist 값의 집합이 {10.0} 하나뿐이었음).
#
# [199차, 부분 해결] apex_dist 대신 **apex_speed의 프레임간 낙차**를
# 구분 기준으로 쓴다. 연속 굽이길은 곡률이 서서히 바뀌므로 apex_speed도
# 프레임당(20Hz) 완만하게만(devnotes 실측 최대 ~2.6km/h, 0.5m 간격
# fine sweep 기준으로도 ~1.4km/h) 변한다. 반면 "뒤늦게 발견된 급커브"는
# 정의상 apex_speed가 한 프레임 만에 큰 폭(수십 km/h)으로 떨어진다.
# 이 낙차가 아래 임계값을 넘는 프레임에서만 "무장(armed)"하여 부스트를
# 켜고, 그 이후에도 apex_speed가 무장 당시 값 근방을 유지하는 동안만
# 부스트를 유지한다(같은 급커브를 계속 추적 중이라는 뜻). 일반 굽이길
# 처럼 apex_speed가 계속 완만하게만 바뀌면 이 게이트가 한 번도 무장되지
# 않아 기존(197차) 동작과 완전히 동일(diff-0)하다.
#
# 안전마진: 실측 최대 완만변화(~2.6km/h)의 약 6배.
# 사전검증: devnotes toolkit/sim_route_vego_required_decel_v3.py(199차)
# -- 156차 winding road 전체 궤적 diff-0(불연속 게이트 한 번도 무장 안
# 됨) + 급커브 인위 주입 시 해당 프레임에서 즉시 무장 감지 + 기존 v2의
# "이미 램프 걸린 상태에서 더 급한 2차 apex" 오버슈트 감소 효과 유지,
# 총 9/9 PASS.
#
# [부분 해결인 이유, 반드시 기록] apex_speed가 "한 프레임에 크게"
# 떨어지는 불연속만 잡는다. 만약 실제 도로에서 급커브가 여러 프레임에
# 걸쳐 이 임계값 미만씩 점진적으로(계단식이 아니라 매 프레임 조금씩)
# 나타나는 경우 -- 누적으로는 크지만 프레임당 낙차는 항상 임계값
# 미만인 경우 -- 이 게이트는 뚫리지 않고 여전히 197차와 동일하게(부스트
# 없이) 동작한다. apex 절대위치를 프레임 간 추적하는 완전한 재설계
# (158/159차가 시도했다 실측 악화로 폐기된 전례가 있는 방향, FINDINGS.md
# 159차/195차/198차 참고) 없이는 원천적으로 닫을 수 없는 구멍이며,
# 이번 v3는 "한 프레임 급락형" 불연속만 부분적으로 해결한다.
#
# **실차 검증: 미실시** -- 시뮬레이션/유닛테스트만 완료된 상태.
ROUTE_APEX_SPEED_DISCONTINUITY_THRESH_KPH = 15.0
# [199차] vEgo 기반 required_decel 부스트가 올라갈 수 있는 상한. 149/150차
# (NEGATIVE) 당시 근사값으로 vturn_decel_rate(1.2)를 재사용했던 것과 달리,
# 이번엔 out_speed 자체를 건드리지 않는 구조(v2/v3 공통 불변식)이므로 더
# 넉넉한 상한을 둬도 149/150차식 "목표 완화" 부작용이 재현되지 않는다.
# devnotes toolkit 시뮬레이션(198/199차)에서 쓰인 값(MAXD=3.0)과 동일하게
# 맞춤 -- 일반적인 편안한 감속(0.7~1.2대)보다는 높지만 비상제동 수준은
# 아닌 보수적 상한.
ROUTE_VEGO_BOOST_MAX_MSS = 3.0
# [202차, 사용자+ChatGPT("지선생") 합의] route apex out_speed 상한을 기존
# sentinel 값(300.0)에서 실주행 상한 150.0km/h로 명시적으로 분리.
# 이 상수는 apex 계산 직후(carrot_navi_route, out_speed = min(out_speed,
# ROUTE_MAX_SPEED_KPH))의 "계산된 값" 클리핑에만 쓰인다 -- route가 아예
# 비활성/미계산 상태임을 나타내는 "제약 없음" sentinel(_route_out_speed
# 초기값 300.0, carrot_serv.route_out_speed 초기값 300.0)은 의미가 다르므로
# 그대로 300.0 유지(150으로 낮추면 route 비활성 구간에서도 마치 150km/h
# 제약이 걸린 것처럼 arbitration에 잘못 참여할 위험). 적용 순서는 이
# 클리핑 이후 carrot_serv.update_navi()에서 MapTurnSpeedFactor(기본
# 1.30)가 곱해지므로(carrot_serv.py `route_speed = max(route_speed *
# self.mapTurnSpeedFactor, ...)`), 최종 arbitration 입력값 상한은
# 150 * mapTurnSpeedFactor(예: 1.30 기준 195km/h)가 될 수 있음 -- 실사용상
# 문제 없다고 판단(어차피 vturn/road_limit 등 다른 후보가 그보다 낮게
# 형성되는 것이 일반적).
ROUTE_MAX_SPEED_KPH = 150.0

# Haversine formula to calculate distance between two GPS coordinates
#haversine_cache = {}
def haversine(lon1, lat1, lon2, lat2):
    #key = (lon1, lat1, lon2, lat2)
    #if key in haversine_cache:
    #    return haversine_cache[key]

    R = 6371000  # Radius of Earth in meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    distance = 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    #haversine_cache[key] = distance
    return distance


# Get the closest point on a segment between two coordinates
def closest_point_on_segment(p1, p2, current_position):
    x1, y1 = p1
    x2, y2 = p2
    px, py = current_position

    dx = x2 - x1
    dy = y2 - y1
    if dx == 0 and dy == 0:
        return p1  # p1 and p2 are the same point

    # Parameter t is the projection factor onto the line segment
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = max(0, min(1, t))  # Clamp t to the segment

    closest_x = x1 + t * dx
    closest_y = y1 + t * dy

    return (closest_x, closest_y)


# Get path after a certain distance from the current position
def get_path_after_distance(start_index, coordinates, current_position, distance_m):
    total_distance = 0
    path_after_distance = []
    closest_index = -1
    closest_point = None
    min_distance = float('inf')

    start_index = max(0, start_index - 2)

    # 가까운 점만 탐색하도록 수정
    for i in range(start_index, len(coordinates) - 1):
        p1 = coordinates[i]
        p2 = coordinates[i + 1]
        candidate_point = closest_point_on_segment(p1, p2, current_position)
        distance = haversine(current_position[0], current_position[1], candidate_point[0], candidate_point[1])

        if distance < min_distance:
            min_distance = distance
            closest_point = candidate_point
            closest_index = i
        elif distance > min_distance and min_distance < 10:
            break

    start_index = closest_index
    # Start from the closest point and calculate the path after the specified distance
    if closest_index != -1:
        path_after_distance.append(closest_point)

        path_after_distance.append(coordinates[closest_index + 1])
        total_distance = haversine(closest_point[0], closest_point[1], coordinates[closest_index + 1][0],
                                   coordinates[closest_index + 1][1])

        # Traverse the path forward from the next point
        for i in range(closest_index + 1, len(coordinates) - 1):
            coord1 = coordinates[i]
            coord2 = coordinates[i + 1]
            segment_distance = haversine(coord1[0], coord1[1], coord2[0], coord2[1])

            if total_distance + segment_distance >= distance_m and segment_distance > 0:
                remaining_distance = distance_m - total_distance
                ratio = remaining_distance / segment_distance
                interpolated_lon = coord1[0] + ratio * (coord2[0] - coord1[0])
                interpolated_lat = coord1[1] + ratio * (coord2[1] - coord1[1])
                path_after_distance.append((interpolated_lon, interpolated_lat))
                break

            total_distance += segment_distance
            path_after_distance.append(coord2)

    return path_after_distance, start_index, closest_point


# [84차] route 커브 lookahead 거리 캡을 300m 고정값 대신 v_ego/accel_limit
# 기반으로 동적 계산(300~600m, 85차에서 500->600 상향 — 120->60km/h
# 풀커버에 accel=0.70 기준 이론상 필요한 ≈595m를 온전히 커버하기 위함).
# "assumed_target_kph"는 실제 커브 목표속도가 아니라(그건 carrot_navi_route()의
# 곡률 계산 이후에야 정해짐 - 이 함수는 그보다 먼저 호출돼야 해서 실제
# 목표속도를 알 수 없음) 캡 크기 산정용 가정값(흔한 조임 커브 수준)일 뿐이다.
# 저속(<=60km/h 부근)에서는 항상 min_m(기존 300m)으로 수렴해 회귀 없음,
# 고속에서만 max_m(600m)까지 확장.
def compute_route_lookahead_distance(v_ego_kph, accel_limit_mss, min_m=300.0, max_m=600.0,
                                      assumed_target_kph=30.0):
  if accel_limit_mss is None or accel_limit_mss <= 0:
    return min_m
  v_ego_ms = max(0.0, v_ego_kph) / 3.6
  v_target_ms = assumed_target_kph / 3.6
  needed_m = max(0.0, (v_ego_ms ** 2 - v_target_ms ** 2) / (2.0 * accel_limit_mss))
  return float(min(max_m, max(min_m, needed_m)))


def calculate_angle(point1, point2):
    delta_lon = point2[0] - point1[0]
    delta_lat = point2[1] - point1[1]
    return math.degrees(math.atan2(delta_lat, delta_lon))

# Convert GPS coordinates to relative x, y coordinates based on a reference point and heading
def gps_to_relative_xy(gps_path, reference_point, heading_deg):
    ref_lon, ref_lat = reference_point
    relative_coordinates = []

    # Convert heading from degrees to radians
    heading_rad = math.radians(heading_deg)

    for lon, lat in gps_path:
        # Convert lat/lon differences to meters (assuming small distances for simple approximation)
        x = (lon - ref_lon) * 40008000 * math.cos(math.radians(ref_lat)) / 360
        y = (lat - ref_lat) * 40008000 / 360

        # Rotate coordinates based on the heading angle to align with the car's direction
        x_rot = x * math.cos(heading_rad) - y * math.sin(heading_rad)
        y_rot = x * math.sin(heading_rad) + y * math.cos(heading_rad)

        relative_coordinates.append((y_rot, x_rot))

    return relative_coordinates


# [99차 발견 -> 100차 패치] carrot_navi_route()가 매 20Hz 사이클마다 Shapely
# LineString(...) 객체를 새로 만들고 그 위에서 .interpolate()를 반복호출하던
# 부분을 numpy 벡터화로 대체. Shapely/GEOS의 interpolate()는 호출마다
# 누적거리를 처음부터 다시 훑는 방식이라 "정점 수 x 호출 횟수"에 비례하는
# 불필요한 재계산이 매 사이클 반복되고 있었음.
# 동작 동일성은 devnotes work/verify_resample_np.py로 검증 완료 -- 랜덤
# 경로 20개 + 급커브/직선/경계조건(2점, 정확히 배수인 길이) + 600m급 긴
# 경로까지 전부 원본(Shapely) 대비 최대오차 1.2e-13m(부동소수점 오차
# 수준) 이내로 100% 일치. 결과 좌표/개수 모두 원본과 동일하므로 이후
# calculate_curvature()/속도산출 로직은 변경 없음.
def resample_10m_np(points_xy, distance_interval=10.0):
    pts = np.asarray(points_xy, dtype=np.float64)
    if len(pts) < 2:
        return [tuple(p) for p in pts]
    seg_vec = np.diff(pts, axis=0)
    seg_len = np.hypot(seg_vec[:, 0], seg_vec[:, 1])
    cum_len = np.concatenate(([0.0], np.cumsum(seg_len)))
    total_len = cum_len[-1]
    if total_len <= 0:
        return [tuple(pts[0])]

    n_samples = int(total_len // distance_interval) + 1
    sample_d = np.arange(n_samples, dtype=np.float64) * distance_interval
    sample_d = sample_d[sample_d <= total_len]

    idx = np.searchsorted(cum_len, sample_d, side="right") - 1
    idx = np.clip(idx, 0, len(seg_len) - 1)

    seg_start_len = cum_len[idx]
    seg_total_len = seg_len[idx]
    with np.errstate(divide="ignore", invalid="ignore"):
        t = np.where(seg_total_len > 0, (sample_d - seg_start_len) / seg_total_len, 0.0)

    p_start = pts[idx]
    p_end = pts[idx + 1]
    out_xy = p_start + (p_end - p_start) * t[:, None]
    return [tuple(p) for p in out_xy]


# Calculate curvature given three points using a faster vector-based method
#curvature_cache = {}
def calculate_curvature(p1, p2, p3):
    #key = (p1, p2, p3)
    #if key in curvature_cache:
    #    return curvature_cache[key]

    v1 = (p2[0] - p1[0], p2[1] - p1[1])
    v2 = (p3[0] - p2[0], p3[1] - p2[1])

    cross_product = v1[0] * v2[1] - v1[1] * v2[0]
    len_v1 = math.sqrt(v1[0] ** 2 + v1[1] ** 2)
    len_v2 = math.sqrt(v2[0] ** 2 + v2[1] ** 2)

    if len_v1 * len_v2 == 0:
        curvature = 0
    else:
        curvature = cross_product / (len_v1 * len_v2 * len_v1)

    #curvature_cache[key] = curvature
    return curvature

class CarrotMan:
  def __init__(self):
    print("************************************************CarrotMan init************************************************")
    self.params = Params()
    self.params_memory = Params("/dev/shm/params")
    self.gps_location_service = get_gps_location_service(self.params)
    self.sm = messaging.SubMaster(['deviceState', 'carState', 'controlsState', 'radarState', 'longitudinalPlan', 'modelV2', 'selfdriveState', 'carControl', 'navRouteNavd', self.gps_location_service, 'navInstruction'])
    self.pm = messaging.PubMaster(['carrotMan', "navRoute", "navInstructionCarrot"])

    self.carrot_serv = CarrotServ()

    self.show_panda_debug = False
    self.broadcast_ip = self.get_broadcast_address()
    self.broadcast_port = 7705
    self.carrot_man_port = 7706
    self.connection = None

    self.ip_address = "0.0.0.0"
    self.remote_addr = None

    self.turn_speed_last = 250
    self.vturn_last_speed = 250.0
    # 2026-08-20 (사용자 실주행 체감 보고 대응): 조여드는 커브 진입 전
    # 사전감속 시간이 부족해 커브 내부에서 급감속(급브레이크)이 발생하는
    # 사례 확인 (devnotes FINDINGS.md "[INVESTIGATING] 조여드는 커브
    # 중간에 vturn 감속 진행 중 운전자 브레이크 개입" 참고 — 260819-7
    # seg6, 곡률이 8.6초에 걸쳐 서서히 증가하는 커브에서 vturn 자체
    # 감속률(1.2 m/s²)은 매끈했지만 aEgo가 -3.41m/s²까지 도달한 직후
    # 운전자가 추가 브레이크 개입).
    #
    # 원인: 아래 vturn_speed()는 모델이 예측한 전방 궤적 중
    # vturn_lookahead_horizon_s 이내(시간 기준)의 지점들만 보고 그중
    # 가장 엄격한(작은) 필요속도를 취한다. 지평선 밖에 있는(아직 안
    # 보이는) 더 급한 정점은 반영되지 않으므로, 정점까지 걸리는 시간이
    # 이 지평선보다 긴 커브(예: 8.6초짜리)에서는 접근 중 정점이 뒤늦게
    # 지평선 안으로 들어오는 순간 필요속도가 갑자기 크게 떨어져 급감속처럼
    # 느껴진다 -- 물리공식(v_i^2=v_f^2+2ad) 자체는 매 프레임 정확해도,
    # "그 순간 보이는 거리"가 짧으면 그만큼 늦게 감속을 시작하는 셈이라
    # 결과적으로는 부족한 사전감속과 동일한 증상이 된다.
    #
    # 대응: 지평선을 4.5s -> 6.5s(1차) -> 8.0s(2차, 사용자 요청)로 확대해
    # 더 먼 지점까지 필요속도 계산에 포함시켜, 급하게 조여드는 커브의
    # 정점을 더 일찍 "본" 상태로 감속을 시작하게 한다. 모델 예측 궤적
    # (ModelConstants.T_IDXS) 자체가 최대 10.0s까지 있으므로 8.0s는 모델
    # 데이터 범위 안에서 안전하게 늘릴 수 있는 값이다.
    # 주의: 근거 사례(260819-7 seg6)의 조임 지속시간이 8.6s라 8.0s로도
    # 극단적으로 긴 조임 구간의 아주 초반부는 완전히 커버하지 못할 수
    # 있음(8.0s < 8.6s) -- 표본 1건 기반 조정이라 다음 세션에서 이 값
    # 적용 후 실주행으로 재검증 필요 (devnotes PARAMS_REGISTRY.md 참고).
    self.vturn_lookahead_horizon_s = 8.0  # 진입 조기감속용 예측 구간(초). T_IDXS가 비선형이라 '초' 기준으로 계산한다.
    # 과속방지턱(calculate_current_speed)과 동일한 v_i^2 = v_f^2 + 2ad 물리공식을 커브에도
    # 적용하기 위한 파라미터. AutoNaviSpeedBumpTime/AutoNaviSpeedDecelRate 기본값과 동일하게
    # 맞춰서 사용자가 이미 익숙한 방지턱 감속 '느낌'과 최대한 비슷하게 시작한다.
    self.vturn_safe_time = 2.0     # 초. 목표속도에 여유있게 미리 도달해 정점까지 유지 (81차: 1.0s는 실제 차량 감속 반응 램프업 시간 대비 부족하다는 체감 보고로 2.0s 상향, c3-ms-curv 실차검증 대상)
    self.vturn_decel_rate = 1.2    # m/s^2. 방지턱 기본 감속률(AutoNaviSpeedDecelRate=120)과 동일
    # 아래는 모델 프레임 노이즈 제거용 저역통과 필터일 뿐, 감속/가속의 '모양'은 위 물리공식이
    # 만든다. 별도의 '진입/탈출 이벤트' 판정이나 지연(hold) 로직은 두지 않는다 - turnSpeed는
    # 매 프레임 전방예측(lookahead) 기반 거리로 재계산되므로, 곡선 구간을 벗어나는 즉시(또는
    # 정점을 지나 남은 구간이 줄면 그 이전부터) 자연스럽게 제약이 풀리고 가속이 시작된다.
    self.vturn_decel_rc = 0.15
    self.vturn_accel_rc = 0.15
    self.curvatureFilter = MyMovingAverage(20)

    # [99차 발견 -> 100차 패치, 101차 순서수정] carrot_navi_route()/
    # carrot_curve_speed_params()가 20Hz 루프(broadcast_version_info)에서
    # 매 사이클 무캐싱으로 읽던 Params 3개(IsOnroad, AutoCurveSpeedFactor,
    # AutoCurveSpeedAggressiveness)를 controlsd.py/radard.py/
    # longitudinal_planner.py(98차)와 동일한 "readParams 카운트다운" 패턴으로
    # 통일 -- 100프레임(20Hz 기준 5s)마다 1회 재조회. 최초 1회는 즉시 읽어
    # 기본값을 채워둔다(readParams=0으로 시작해 첫 호출에서 바로 갱신됨).
    #
    # [101차 수정] 이 블록은 반드시 아래 self.carrot_curve_speed_params()
    # 호출보다 먼저 와야 한다 -- 그 함수가 self._auto_curve_speed_factor /
    # self._auto_curve_speed_aggressiveness를 그대로 참조한다(carrot_curve_
    # speed_params() 정의부 참고). 100차 패치는 이 캐시 초기화 블록을
    # __init__ 맨 끝(self.is_metric 이후)에 둔 채로 놔둬서, 이미 그 위쪽에
    # 있던 carrot_curve_speed_params() 호출이 아직 존재하지 않는 캐시
    # 필드를 참조 -> AttributeError로 carrot_man이 __init__ 도중 즉시
    # 죽는 버그가 발생했다. 이 죽음은 cloudlog가 아직 설정되기 전(또는
    # 그와 매우 가까운 시점)에 일어나 rlog/qlog에 Python traceback이
    # 전혀 남지 않았고, managerState에도 exitCode=1만 반복 기록되어
    # 원인 추적이 어려웠다(devnotes FINDINGS.md "101차" 참고).
    self.readParams = 0
    self._is_onroad_cached = self.params.get_bool("IsOnroad")
    self._auto_curve_speed_factor = self.params.get_int("AutoCurveSpeedFactor") * 0.01
    self._auto_curve_speed_aggressiveness = self.params.get_int("AutoCurveSpeedAggressiveness") * 0.01

    self.carrot_curve_speed_params()

    self.carrot_zmq_thread = threading.Thread(target=self.carrot_cmd_zmq, args=[])
    self.carrot_zmq_thread.daemon = True
    self.carrot_zmq_thread.start()

    self.carrot_panda_debug_thread = threading.Thread(target=self.carrot_panda_debug, args=[])
    self.carrot_panda_debug_thread.daemon = True
    self.carrot_panda_debug_thread.start()

    self.carrot_route_thread = threading.Thread(target=self.carrot_route, args=[])
    self.carrot_route_thread.daemon = True
    self.carrot_route_thread.start()

    self.is_running = True
    threading.Thread(target=self.broadcast_version_info).start()

    self.navi_points = []
    self.navi_points_start_index = 0
    self.navi_points_active = False
    self.navd_active = False
    # [182차 계측] navi_points_active 드롭아웃(FINDINGS.md 182차) 원인규명용.
    # 어느 경로(navd cereal/TCP 7709 raw/TCP 7712 handle_route)가 마지막으로
    # route를 성공 수신했는지, 비활성 상태가 얼마나 지속됐는지를 cereal로
    # 노출한다 -- 이전엔 print()로만 남아 rlog 재분석이 불가능했음.
    self._navi_route_source = ""
    self._navi_active_last_ts = None
    self._dt_route_inactive = 0.0
    # [132차] Hypothesis C(131차) 대응: carrot_navi_route()의 route_lookahead
    # 윈도우 경계로 급커브가 이산적으로 진입하며 out_speed가 단일 20Hz
    # 프레임에 급락(최대 Δ-25kph 실측)하는 현상을 완화하기 위한 프레임간
    # 램프 리미터 상태값. None이면 리미터 미적용(최초 활성화/직후 상태).
    self._route_speed_prev = None
    # [199차, 설계 A v3] apex_speed 불연속 감지 게이트 상태(위
    # ROUTE_APEX_SPEED_DISCONTINUITY_THRESH_KPH 주석 참고). prev는 직전
    # 프레임 apex_speed(불연속 판정 기준), armed는 이번 프레임 vEgo 기반
    # 부스트 무장 여부, armed_speed는 무장 당시의 apex_speed(같은 급커브를
    # 계속 추적 중인지 판정하는 기준값).
    self._route_apex_speed_prev = None
    self._route_apex_boost_armed = False
    self._route_apex_boost_armed_speed = None

    self.active_carrot_last = False

    self._rgdata_ts_lock = threading.Lock()
    self._last_rgdata_timestamp_ms = 0

    self.is_metric = self.params.get_bool("IsMetric")

  def _refresh_cached_params(self):
    # [99차/100차] 20Hz 루프 내 Params I/O 캐싱 -- 98차(controlsd.py 등)와
    # 동일한 카운트다운 패턴. 이 3개 파라미터는 주행 중 실시간으로 바뀔
    # 필요가 없는 설정값(온로드 상태/커브속도 튜닝 계수)이라 5s 지연은
    # 회귀 위험 없음.
    self.readParams -= 1
    if self.readParams <= 0:
      self.readParams = 100
      self._is_onroad_cached = self.params.get_bool("IsOnroad")
      self._auto_curve_speed_factor = self.params.get_int("AutoCurveSpeedFactor") * 0.01
      self._auto_curve_speed_aggressiveness = self.params.get_int("AutoCurveSpeedAggressiveness") * 0.01

  def get_broadcast_address(self):
    if PC:
      iface = b'br0'
    else:
      iface = b'wlan0'
    try:
      with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        ip = fcntl.ioctl(
          s.fileno(),
          0x8919,
          struct.pack('256s', iface)
        )[20:24]
        return socket.inet_ntoa(ip)
    except (OSError, Exception):
      return None

  def get_local_ip(self):
      try:
          # 외부 서버와의 연결을 통해 로컬 IP 확인
          with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
              s.connect(("8.8.8.8", 80))  # Google DNS로 연결 시도
              return s.getsockname()[0]
      except Exception as e:
          return f"Error: {e}"

    
  # 브로드캐스트 메시지 전송
  def broadcast_version_info(self):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    frame = 0
    self.save_toggle_values()

    rk = Ratekeeper(20, print_delay_threshold=None)

    while self.is_running:
      try:
        self.sm.update(0)
        self._refresh_cached_params()  # [99차/100차] IsOnroad/AutoCurveSpeed* 캐싱 갱신 (100프레임=5s마다)
        if self.sm.updated['navRouteNavd']:
          self.send_routes(self.sm['navRouteNavd'].coordinates, True)
        remote_addr = self.remote_addr
        remote_ip = remote_addr[0] if remote_addr is not None else ""
        vturn_speed = self.carrot_curve_speed(self.sm)
        coords, distances, route_speed = self.carrot_navi_route()

        # [182차 계측] navi_points_active 비활성 지속시간 갱신 (FINDINGS.md
        # 182차 -- 이전엔 이 상태전이가 cereal 미발행이라 사후 로그분석 불가).
        now_ts = time.monotonic()
        if self.navi_points_active:
          self._navi_active_last_ts = now_ts
          self._dt_route_inactive = 0.0
        elif self._navi_active_last_ts is not None:
          self._dt_route_inactive = now_ts - self._navi_active_last_ts
        # else: 이번 온로드 세션에서 한 번도 활성화된 적 없음 -- 0.0 유지

        #print("coords=", coords)
        #print("curvatures=", curvatures)
        self.carrot_serv.update_navi(remote_ip, self.sm, self.pm, vturn_speed, coords, distances, route_speed, self.gps_location_service,
                                      self.navi_points_active, self.navd_active, self._dt_route_inactive, self._navi_route_source)

        if frame % 20 == 0 or remote_addr is not None:
          try:
            self.broadcast_ip = self.get_broadcast_address() if remote_addr is None else remote_addr[0]
            if not PC:
              ip_address = socket.gethostbyname(socket.gethostname())
            else:
              ip_address = self.get_local_ip()
            if ip_address != self.ip_address:
              self.ip_address = ip_address
              self.remote_addr = None
            self.params_memory.put_nonblocking("NetworkAddress", self.ip_address)

            msg = self.make_send_message()
            if self.broadcast_ip is not None:
              dat = msg.encode('utf-8')
              sock.sendto(dat, (self.broadcast_ip, self.broadcast_port))
            #for i in range(1, 255):
            #  ip_tuple = socket.inet_aton(self.broadcast_ip)
            #  new_ip = ip_tuple[:-1] + bytes([i])
            #  address = (socket.inet_ntoa(new_ip), self.broadcast_port)
            #  sock.sendto(dat, address)

            if remote_addr is None:
              #print(f"Broadcasting: {self.broadcast_ip}") #:{msg}")
              if not self.navd_active:
                #print("clear path_points: navd_active: ", self.navd_active)
                self.navi_points = []
                self.navi_points_active = False

          except Exception as e:
            if self.connection:
              self.connection.close()
            self.connection = None
            print(f"##### broadcast_error...: {e}")
            traceback.print_exc()

        rk.keep_time()
        frame += 1
      except Exception as e:
        print(f"broadcast_version_info error...: {e}")
        traceback.print_exc()
        time.sleep(1)

  
  def carrot_navi_route(self):

    # [193차] route apex 진단값은 매 호출마다 초기화한다.
    # route 비활성/후보 없음 프레임에서 이전 프레임 값이 남지 않도록 한다.
    self._route_apex_idx = -1
    self._route_apex_dist = 0.0
    self._route_apex_speed = 0.0
    self._route_out_speed = 300.0
    # [194차] cereal로 실제 발행되는 CarrotServ 쪽 저장소도 동일하게
    # 매 호출 초기화 -- 이걸 빼먹으면 route가 비활성화된 프레임에서도
    # rlog에 직전 활성 프레임의 apex 값이 그대로 남아 오분석을 유발한다.
    self.carrot_serv.route_apex_idx = -1
    self.carrot_serv.route_apex_dist = 0.0
    self.carrot_serv.route_apex_speed = 0.0
    self.carrot_serv.route_out_speed = 300.0

    # [99차/100차, 죽은 코드 정리] 여기 있던 `if self.carrot_serv.active_carrot > 1:
    # if False and self.navd_active:` 블록은 항상 거짓이라 실행된 적이 없는
    # 죽은 분기 -- 제거 (동작 변화 없음).
    # [99차/100차] 매 호출(20Hz)마다 새로 읽던 것을 캐시값으로 대체 (100프레임=5s마다 갱신)
    is_onroad = self._is_onroad_cached
    if not is_onroad or not self.navi_points_active or (self.carrot_serv.active_carrot <= 1 and not self.navd_active):
      #print(f"navi_points_active: {self.navi_points_active}, active_carrot: {self.carrot_serv.active_carrot}")
      if self.navi_points_active:
        print("navi_points_active: ", self.navi_points_active, "active_carrot: ", self.carrot_serv.active_carrot, "navd_active: ", self.navd_active)
        #haversine_cache.clear()
        #curvature_cache.clear()
        self.navi_points = []
        self.navi_points_active = False
        if self.active_carrot_last > 1:
          #self.params.remove("NavDestination")
          pass
      self.active_carrot_last = self.carrot_serv.active_carrot
      # [132차] route 비활성화 -- 다음 활성화 시 리미터가 과거 값을 끌고
      # 오지 않도록 리셋(제약 해제 방향은 항상 즉시 반영되어야 안전).
      self._route_speed_prev = None
      # [199차] 불연속 감지 게이트도 함께 리셋 -- route가 다시 활성화될
      # 때 직전(비활성 이전) apex_speed와 비교해 오탐하지 않도록 한다.
      self._route_apex_speed_prev = None
      self._route_apex_boost_armed = False
      self._route_apex_boost_armed_speed = None
      return [],[],300

    current_position = (self.carrot_serv.vpPosPointLon, self.carrot_serv.vpPosPointLat)
    heading_deg = self.carrot_serv.bearing

    distance_interval = 10.0
    out_speed = 300
    # [84차, 85차 500->600 상향] 300m 고정 캡 -> v_ego/accel_limit 기반 동적 캡(300~600m)
    route_lookahead_m = compute_route_lookahead_distance(self.sm['carState'].vEgo * 3.6,
                                                          self.carrot_serv.autoNaviSpeedDecelRate)
    path, self.navi_points_start_index, start_point = get_path_after_distance(self.navi_points_start_index, self.navi_points, current_position, route_lookahead_m)
    relative_coords = []
    if path:
        #relative_coords = gps_to_relative_xy(path, current_position, heading_deg)
        relative_coords = gps_to_relative_xy(path, start_point, heading_deg)
        # [99차/100차] distance_interval(10m) 간격으로 리샘플 -- Shapely
        # LineString.interpolate() 반복호출 대신 numpy 벡터화 함수 사용
        # (수치 동일성 검증: devnotes work/verify_resample_np.py)
        resampled_points = resample_10m_np(relative_coords, distance_interval)
        resampled_distances = [i * distance_interval for i in range(len(resampled_points))]

        curvatures = []
        distances = []
        distance = 10.0
        sample = 4
        if len(resampled_points) >= sample * 2 + 1:
            # Calculate curvatures and speeds based on curvature
            speeds = []
            for i in range(len(resampled_points) - sample * 2):
                distance += distance_interval
                p1, p2, p3 = resampled_points[i], resampled_points[i + sample], resampled_points[i + sample * 2]
                curvature = calculate_curvature(p1, p2, p3)
                speed = np.interp(abs(curvature), V_CURVE_LOOKUP_BP, V_CRUVE_LOOKUP_VALS)
                if abs(curvature) < ROUTE_CURVE_NEGLIGIBLE_THRESHOLD:
                  speed = max(speed, self.carrot_serv.nRoadLimitSpeed)
                curvatures.append(curvature)
                speeds.append(speed)
                distances.append(distance)

            # [147차] 미세(fine) chord 보조 샘플 -- 위 매크로(sample=4,
            # 40m) 루프가 놓치는 좁은 코너(예: 교차로 우회전)를 보정.
            # 같은 리샘플 폴리라인에 ROUTE_CURVATURE_FINE_SAMPLE(기본
            # 10m) 간격으로 3점 곡률을 한 번 더 계산해, 같은 거리
            # 위치에서 더 급한(=speed가 더 낮은) 쪽만 채택한다. 매크로
            # 결과 자체를 대체하지 않으므로 장거리 lookahead 매크로
            # 형상(직선 오탐 방지)은 그대로 유지된다.
            sample_fine = ROUTE_CURVATURE_FINE_SAMPLE
            if sample_fine and sample_fine < sample and len(resampled_points) >= sample_fine * 2 + 1:
                fine_distance = 10.0
                fine_points = []  # (distance, curvature, speed)
                for i in range(len(resampled_points) - sample_fine * 2):
                    fine_distance += distance_interval
                    p1, p2, p3 = resampled_points[i], resampled_points[i + sample_fine], resampled_points[i + sample_fine * 2]
                    f_curvature = calculate_curvature(p1, p2, p3)
                    f_speed = np.interp(abs(f_curvature), V_CURVE_LOOKUP_BP, V_CRUVE_LOOKUP_VALS)
                    if abs(f_curvature) < ROUTE_CURVE_NEGLIGIBLE_THRESHOLD:
                      f_speed = max(f_speed, self.carrot_serv.nRoadLimitSpeed)
                    fine_points.append((fine_distance, f_curvature, f_speed))
                if fine_points:
                    fine_idx = 0
                    for j in range(len(distances)):
                        d = distances[j]
                        # distances[]와 fine_points[]는 둘 다 10m 간격
                        # 같은 시작점이므로, 가장 가까운 fine 포인트를
                        # 앞에서부터 순차 탐색(선형, O(n))으로 찾는다.
                        while (fine_idx + 1 < len(fine_points)
                               and abs(fine_points[fine_idx + 1][0] - d) <= abs(fine_points[fine_idx][0] - d)):
                            fine_idx += 1
                        f_dist, f_curv, f_speed = fine_points[fine_idx]
                        if f_speed < speeds[j]:
                            speeds[j] = f_speed
                            curvatures[j] = f_curv
            #print(f"curvatures= {[round(s, 4) for s in curvatures]}")
            #print(f"speeds= {[round(s, 1) for s in speeds]}")
            # [160차, 사용자 설계 전면 교체 -- 곡선_가감속_코딩.txt +
            # 곡선_개념도.pdf] 157차의 "apex까지 거리로 accel_limit을
            # 동적 부스트(필요감속률> accel_limit이면 vturn_decel_rate까지
            # 상한 부스트)"하던 커스텀 공식을 폐기하고, **과속카메라
            # 감속(carrot_serv.calculate_current_speed)과 완전히 동일한
            # 물리공식을 그대로 재사용**하는 것으로 교체.
            #
            # 배경: "route 감속의 목적은 Vturn(비전) 감속만으로는 부족한
            # 사전감속" -- apex(최대곡률지점) 목표속도를 과속카메라의
            # 제한속도처럼 취급해, 카메라와 동일하게 서서히 감속 ->
            # apex 도달(거리<=0) 시 원복하는 형태로 단순화한다.
            # [196차] 전방 lookahead 전체의 sharpest(전역 최저 목표속도)를
            # 하나의 apex로 선택하지 않는다. 연속곡선을 1차->2차 순서로
            # 처리하기 위해 "가장 먼저 만나는 감속 필요 지점"을 apex로
            # 선택(아래 candidates[0]). 이 함수는 20Hz마다 lookahead
            # 윈도우를 새로 계산하는 무상태(stateless) 구조이므로,
            # "1차 apex를 지나면(=1차 지점이 차량 뒤로 빠져 윈도우에서
            # 사라지면) 다음 프레임에 자동으로 2차 apex가 새로 선택"된다
            # (158~159차가 명시적 히스테리시스 상태를 시도했다가 실측
            # 악화로 폐기한 전례가 있어, 이번에도 상태를 새로 만들지
            # 않고 기존 무상태 원칙을 유지 -- FINDINGS.md 159차 참고).
            #
            # calculate_current_speed(left_dist, safe_speed_kph, safe_time,
            # safe_decel_rate)는 decel_dist(=left_dist-safe_speed*safe_time)
            # 가 0 이하면 함수 내부에서 자동으로 safe_speed_kph를 반환하므로,
            # 기존의 "apex_dist>0/else" 수동 분기와 동적 accel 부스트 분기가
            # 전부 불필요해져 제거했다. v_ego는 카메라 공식과 마찬가지로
            # 아예 쓰지 않는다(거리만으로 결정).
            #
            # 사전검증: devnotes toolkit/sim_route_camera_style_decel.py
            # (160차) -- 156차류 연속 굽이길/직선 회귀없음/147차류 단일
            # 커브/152·153차 근정지/연속 S자커브(2차가 더 급한 경우 +
            # 1차가 더 급해 apex가 전환되는 경우 둘 다) 7/7 PASS, 132차
            # 램프리미터 이론상한(accel_limit_kmh*dt) 내에서 톱니 진동
            # 없이 apex 전환됨을 확인.
            #
            # [179차, apex 선택기준 변경 -- "가장 급한 지점" -> "가장 가까운
            # 지점"] 157/160차부터 유지되던 min(speeds) 전역탐색(=lookahead
            # 윈도우 전체에서 목표속도가 가장 낮은, 즉 가장 급커브인 지점을
            # apex로 선택)은 그 지점이 아무리 멀리 있어도(예: lookahead
            # 끝자락의 급커브) 그 먼 지점 기준으로 조기 감속을 시작하게
            # 만들어, 실제로는 먼저 지나야 할 더 가까운(하지만 상대적으로
            # 완만한) 커브를 무시하는 문제가 있었다. distances[]는 항상
            # 오름차순(가장 가까운 포인트가 index 0)이므로, speeds[k] <
            # 도로제한속도(nRoadLimitSpeed)인 -- 즉 실제 감속이 필요한 --
            # 가장 가까운(작은 distance) 지점을 apex로 선택하도록 변경.
            # 감속이 필요한 지점이 lookahead 윈도우 내에 하나도 없으면
            # (전부 직선) 기존과 동일하게 전역 min(speeds)로 폴백 --
            # 이 경우 모든 speeds가 사실상 도로제한속도이므로 어느 지점을
            # 골라도 out_speed에 미치는 영향은 없다.
            # [196차, 179차후속2 게이트 제거] "가장 가까운 지점"(candidates)
            # 선택 이후 sharpest 대비 상대적 심각도로 재필터링하던 게이트를
            # 제거했다. candidates[0](거리 오름차순 첫 감속 필요 지점)을
            # 게이트 없이 그대로 apex로 선택 -- 이유는 위 상수 정의부 주석
            # 참고. 실차 검증 필수(196차 실차 검증: 미실시).
            road_limit_speed = self.carrot_serv.nRoadLimitSpeed
            candidates = [k for k in range(len(speeds)) if speeds[k] < road_limit_speed]
            if not candidates:
                apex_idx = min(range(len(speeds)), key=lambda k: speeds[k])  # 폴백: 감속필요구간 없음(직선)
            else:
                apex_idx = candidates[0]
            apex_dist = distances[apex_idx]
            apex_speed = speeds[apex_idx]

            out_speed = self.carrot_serv.calculate_current_speed(
                apex_dist,
                apex_speed,
                self.carrot_serv.autoNaviSpeedCtrlEnd,    # safe_time -- 카메라와 동일 파라미터 재사용
                self.carrot_serv.autoNaviSpeedDecelRate,  # safe_decel_rate -- 카메라와 동일 파라미터 재사용
            )
            # [193차] route apex 실차/replay 진단 telemetry.
            # 주행 계산값은 변경하지 않고, 선택된 apex와 최종 route 출력만 보존한다.
            self._route_apex_idx = apex_idx
            self._route_apex_dist = apex_dist
            self._route_apex_speed = apex_speed
            self._route_out_speed = out_speed
            # [194차] 193차는 이 값을 CarrotMan(self) 내부에만 저장해서
            # cereal(msg.carrotMan)까지 전달되지 않았음(FINDINGS.md 193차).
            # CarrotServ가 실제 cereal 발행부를 갖고 있고 self.carrot_serv는
            # CarrotServ 인스턴스이므로, 계산 직후 CarrotServ 쪽 저장소에도
            # 같은 값을 써서 carrot_serv.py의 update_navi()가 msg에 담을 수
            # 있게 한다. 주행 계산 로직(out_speed 등)은 변경하지 않는다.
            self.carrot_serv.route_apex_idx = apex_idx
            self.carrot_serv.route_apex_dist = apex_dist
            self.carrot_serv.route_apex_speed = apex_speed
            self.carrot_serv.route_out_speed = out_speed
            out_speed = min(out_speed, ROUTE_MAX_SPEED_KPH)  # [202차] 300.0 -> 150.0(명시적 상수)
            # accel_limit_kmh 기본값(부스트 없을 때) -- 132차 램프리미터가
            # 그대로 사용.
            base_accel_limit_kmh = self.carrot_serv.autoNaviSpeedDecelRate * 3.6
            accel_limit_kmh = base_accel_limit_kmh

            # [199차, 설계 A v3 -- 위 ROUTE_APEX_SPEED_DISCONTINUITY_THRESH_KPH
            # 주석 참고] raw out_speed(위 out_speed)는 절대 건드리지 않는다
            # (197차와 100% 동일 -- 이 블록 전체는 아래 132차 램프리미터의
            # 하강 상한(accel_limit_kmh)에만 영향을 준다).
            #
            # 1) 불연속 감지: 직전 프레임 대비 apex_speed가 임계값 이상
            #    떨어졌으면 새로 무장, 이미 무장된 상태면 apex_speed가
            #    무장 당시 값 근방을 유지하는 동안 계속 무장 유지.
            if self._route_apex_speed_prev is None:
                # 최초 관측 -- 불연속 여부 판단 불가, 무장하지 않음(197차와
                # diff-0가 기본값). 무장 안 된 다음 프레임부터 정상적으로
                # 델타 비교가 시작된다(0.05s 지연, 무시 가능).
                pass
            else:
                apex_delta_kph = self._route_apex_speed_prev - apex_speed  # 양수=더 급해짐
                if apex_delta_kph > ROUTE_APEX_SPEED_DISCONTINUITY_THRESH_KPH:
                    self._route_apex_boost_armed = True
                    self._route_apex_boost_armed_speed = apex_speed
                elif self._route_apex_boost_armed:
                    if abs(apex_speed - self._route_apex_boost_armed_speed) > ROUTE_APEX_SPEED_DISCONTINUITY_THRESH_KPH:
                        self._route_apex_boost_armed = False
                        self._route_apex_boost_armed_speed = None
            self._route_apex_speed_prev = apex_speed

            # 2) 무장된 프레임에서만 vEgo 기반 필요감속도로 하강 상한을
            #    동적 부스트(target 자체는 건드리지 않으므로 149/150차식
            #    "목표 완화" 부작용이 구조적으로 불가능).
            if self._route_apex_boost_armed and apex_dist > 0:
                v_ego_ms = self.sm['carState'].vEgo
                v_target_ms = apex_speed / 3.6
                if v_ego_ms > v_target_ms:
                    required_decel_mss = (v_ego_ms ** 2 - v_target_ms ** 2) / (2.0 * apex_dist)
                    if required_decel_mss > self.carrot_serv.autoNaviSpeedDecelRate:
                        boosted_mss = min(required_decel_mss, ROUTE_VEGO_BOOST_MAX_MSS)
                        accel_limit_kmh = boosted_mss * 3.6

            # [132차, Hypothesis C(131차) 대응] route_lookahead_m 윈도우
            # 경계로 급커브 지점이 이산적으로 curvature 배열에 "출현"하는
            # 순간, 위 역방향 DP가 그 프레임에 즉시 전체를 재계산해
            # out_speeds[0](=out_speed)까지 낮은 값이 단일 20Hz 프레임에
            # 전파될 수 있음(129차 실측 Δ-24~-25kph 단일프레임 급락,
            # 131차 합성검증 SUCCESS로 메커니즘 확인). 이 자체가 새로운
            # 제약을 추가하는 게 아니라 -- route_lookahead_m이 애초에
            # "이 accel_limit_kmh로 감속하기에 충분한 거리"를 목표로
            # 산정되므로(84차/85차), 경계 스냅이 없었다면 매 프레임 이미
            # 성립했어야 할 불변식(프레임당 변화 <= accel_limit_kmh*dt)을
            # 최종 출력에서 강제로 복원하는 것에 가깝다.
            # 사전검증: devnotes toolkit/sim_route_boundary_ramp_limiter.py
            # (132차) -- curve_R 10~25m/accel 0.70~1.2 조합에서 정상주행
            # 구간 최대 프레임당 낙차가 이론 상한(accel_limit_kmh*dt)
            # 이내로 억제됨을 확인(PASS).
            #
            # [172차/173차, 원인A 수정 -- 대칭 -> 비대칭 램프로 변경]
            # 132차 도입 당시(91차 backward-DP, apex 개념 없음)엔 "회전
            # 종료 즉시 원복" 계단(129/131차)까지 함께 완화하려고 증가
            # (원복) 방향에도 동일 램프를 대칭 적용했었다(원 커밋 주석에
            # 명시). 그러나 157/160차가 아키텍처를 카메라식 apex 거리공식
            # (calculate_current_speed 재사용)으로 전면 교체하면서 "apex
            # 통과 시 즉시 원복"을 설계 의도로 못박았고(160차 커밋
            # 메시지), 실제로 카메라감속(sdi_speed)/회전감속(atc_desired)도
            # 이 램프 없이 즉시 원복한다. 즉 132차의 증가측 램프는 160차
            # 이후로는 더 이상 필요한 완화가 아니라, 오히려 160차가
            # 의도한 즉시 원복을 무력화하는 부작용이 됐다(172차 실측:
            # apex 통과 후 desiredSpeed 30->48이 accel_limit_kmh 그대로
            # 5.5초에 걸쳐 서서히 상승 -- 사용자 제보 "우회전 통과 후
            # route 속도가 서서히 상승"과 정합).
            # 하강(lo) 방향은 129차/131차가 해결하려던 문제(경계 스냅으로
            # 인한 단일프레임 급락)가 아키텍처가 바뀐 지금도 여전히
            # 발생 가능하므로 그대로 유지 -- 감속 스케줄 보호 목적은
            # 살아있다.
            # 사전검증: devnotes toolkit/sim_route_boundary_ramp_limiter.py
            # (173차, RampLimiterState(asymmetric_up=True) 추가) -- 정상
            # 주행 중 하강측 낙차 억제는 그대로 유지, 상승측은 raw
            # out_speed를 즉시 추종(지연 없음)함을 확인.
            if self._route_speed_prev is not None:
              max_step_kmh = accel_limit_kmh * ROUTE_SPEED_LOOP_DT
              lo = self._route_speed_prev - max_step_kmh
              # [173차] 증가(원복) 방향은 기본적으로 무제한 -- 160차 설계
              # 의도(즉시 원복)를 그대로 따른다.
              hi = math.inf
              # [162차] 위치추정이 불확실한 구간(데드레커닝이 실제 GPS/앱
              # 위치갱신 없이 오래 지속)에서는 상승(완화) 쪽 상한을 이전
              # 값으로 고정 -- curvature 오판으로 인한 "가짜 직선" 판정이
              # route_speed를 300 쪽으로 밀어올리지 못하게 막는다. 하강
              # 쪽(lo)은 그대로 둬 실제 감속 필요는 계속 반영한다.
              # [167차] 166차 헤딩보정(방향1)이 CC.orientationNED로 이미
              # 이 상황을 커버하므로, 방향1이 무력화되는 폴백 구간
              # (cc_pose_valid=False -- 캘리브레이션 미완료 등)에서만
              # 이 게이트(방향2)가 안전망으로 발동하도록 조건을 좁힘.
              # 방향1이 정상 동작 중(cc_pose_valid=True)이면 방향2는
              # 물러나 route가 정확해진 curvature를 그대로 따라가게 둔다.
              # [173차] 이 게이트는 132차 대칭램프 시절과 동일하게 그대로
              # 유지 -- 위치불확실 상황에서만 예외적으로 상승측도 다시
              # prev로 고정(안전망), 정상 상황에서만 위 무제한(hi=inf)이
              # 적용된다.
              if (self.carrot_serv.position_dt_since_fix > ROUTE_POSITION_UNCERTAIN_DT_S
                  and not self.carrot_serv.cc_pose_valid):
                hi = self._route_speed_prev
              out_speed = min(max(out_speed, lo), hi)
            self._route_speed_prev = out_speed
    else:
        resampled_points = []
        resampled_distances = []
        curvatures = []
        speeds = []
        distances = []
        # [132차] "제약 없음"(윈도우 내 유효 포인트 부족) 상태로 전환 --
        # 이 방향은 허용속도가 올라가는 안전한 방향(제약 해제)이므로
        # 리미터를 즉시 리셋해 다음 번 실제 제약이 나타날 때 정상적으로
        # 다시 램프가 걸리도록 한다(과거 값에 묶여 완화가 지연되는 역설
        # 방지).
        self._route_speed_prev = None
        # [199차] 불연속 감지 게이트도 동일 이유로 리셋.
        self._route_apex_speed_prev = None
        self._route_apex_boost_armed = False
        self._route_apex_boost_armed_speed = None
        #self.params.remove("NavDestination")

    return resampled_points, resampled_distances, out_speed #speeds, distances


  def make_send_message(self):
    msg = {}
    msg['Carrot2'] = self.params.get("Version")
    isOnroad = self.params.get_bool("IsOnroad")
    msg['IsOnroad'] = isOnroad
    msg['CarrotRouteActive'] = self.navi_points_active
    msg['ip'] = self.ip_address
    msg['port'] = self.carrot_man_port
    self.controls_active = False
    self.xState = 0
    self.trafficState = 0
    v_ego_kph = 0
    log_carrot = ""
    v_cruise_kph = 0
    carcruiseSpeed = 0
    if not isOnroad:
      self.xState = 0
      self.trafficState = 0
    else:
      if self.sm.alive['carState']:
        carState = self.sm['carState']
        v_ego_kph = int(carState.vEgoCluster * 3.6 + 0.5)
        log_carrot = carState.logCarrot
        v_cruise_kph = carState.vCruise
        carcruiseSpeed = carState.cruiseState.speed * 3.6
      if self.sm.alive['selfdriveState']:
        selfdrive = self.sm['selfdriveState']
        self.controls_active = selfdrive.active
      if self.sm.alive['longitudinalPlan']:
        lp = self.sm['longitudinalPlan']
        self.xState = lp.xState
        self.trafficState = lp.trafficState

    msg['log_carrot'] = log_carrot
    msg['v_cruise_kph'] = v_cruise_kph
    msg['carcruiseSpeed'] = carcruiseSpeed
    msg['v_ego_kph'] = v_ego_kph
    msg['tbt_dist'] = self.carrot_serv.xDistToTurn
    msg['sdi_dist'] = self.carrot_serv.xSpdDist
    msg['route_apex_idx'] = getattr(self, '_route_apex_idx', -1)
    msg['route_apex_dist'] = getattr(self, '_route_apex_dist', 0.0)
    msg['route_apex_speed'] = getattr(self, '_route_apex_speed', 0.0)
    msg['route_out_speed'] = getattr(self, '_route_out_speed', 0.0)
    msg['active'] = self.controls_active
    msg['xState'] = self.xState
    msg['trafficState'] = self.trafficState
    return json.dumps(msg)

  def receive_fixed_length_data(self, sock, length):
    buffer = b""
    while len(buffer) < length:
      data = sock.recv(length - len(buffer))
      if not data:
        raise ConnectionError("Connection closed before receiving all data")
      buffer += data
    return buffer


  def carrot_man_thread(self):
    while True:
      try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
          sock.settimeout(10)  # 소켓 타임아웃 설정 (10초)
          sock.bind(('0.0.0.0', self.carrot_man_port))  # UDP 포트 바인딩
          print("#########carrot_man_thread: UDP thread started...")

          while True:
            try:
              #self.remote_addr = None
              # 데이터 수신 (UDP는 recvfrom 사용)
              try:
                data, remote_addr = sock.recvfrom(4096)  # 최대 4096 바이트 수신
                #print(f"Received data from {self.remote_addr}")

                if not data:
                  raise ConnectionError("No data received")

                if self.remote_addr is None:
                  print("Connected to: ", remote_addr)
                self.remote_addr = remote_addr
                try:
                  json_obj = json.loads(data.decode())
                  self.carrot_serv.update(json_obj)
                except Exception as e:
                  print(f"carrot_man_thread: json error...: {e}")
                  print(data)

                # 응답 메시지 생성 및 송신 (UDP는 sendto 사용)
                #try:
                #  msg = self.make_send_message()
                #  sock.sendto(msg.encode('utf-8'), self.remote_addr)
                #except Exception as e:
                #  print(f"carrot_man_thread: send error...: {e}")

              except TimeoutError:
                #print("Waiting for data (timeout)...")
                self.remote_addr = None
                time.sleep(1)

              except Exception as e:
                print(f"carrot_man_thread: error...: {e}")
                self.remote_addr = None
                break

            except Exception as e:
              print(f"carrot_man_thread: recv error...: {e}")
              self.remote_addr = None
              break

          time.sleep(1)
      except Exception as e:
        self.remote_addr = None
        print(f"Network error, retrying...: {e}")
        time.sleep(2)

  def parse_kisa_data(self, data: bytes):
    result = {}

    try:
      decoded = data.decode('utf-8')
    except UnicodeDecodeError:
      print("Decoding error:", data)
      return result

    parts = decoded.split('/')
    for part in parts:
      if ':' in part:
        key, value = part.split(':', 1)
        try:
          result[key] = int(value)
        except ValueError:
          result[key] = value
    return result

  def kisa_app_thread(self):
    while True:
      try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
          sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
          sock.settimeout(10)  # 소켓 타임아웃 설정 (10초)
          sock.bind(('', 12345))  # UDP 포트 바인딩
          print("#########kisa_app_thread: UDP thread started...")

          while True:
            try:
              #self.remote_addr = None
              # 데이터 수신 (UDP는 recvfrom 사용)
              try:
                data, remote_addr = sock.recvfrom(4096)  # 최대 4096 바이트 수신
                #print(f"Received data from {self.remote_addr}")

                if not data:
                  raise ConnectionError("No data received")

                #if self.remote_addr is None:
                #  print("Connected to: ", remote_addr)
                #self.remote_addr = remote_addr
                try:
                  print(data)
                  kisa_data = self.parse_kisa_data(data)
                  self.carrot_serv.update_kisa(kisa_data)
                  #json_obj = json.loads(data.decode())
                  #print(json_obj)
                except Exception as e:
                  traceback.print_exc()
                  print(f"kisa_app_thread: json error...: {e}")
                  print(data)

              except TimeoutError:
                #print("Waiting for data (timeout)...")
                #self.remote_addr = None
                time.sleep(1)

              except Exception as e:
                print(f"kisa_app_thread: error...: {e}")
                #self.remote_addr = None
                break

            except Exception as e:
              print(f"kisa_app_thread: recv error...: {e}")
              #self.remote_addr = None
              break

          time.sleep(1)
      except Exception as e:
        #self.remote_addr = None
        print(f"Network error, retrying...: {e}")
        time.sleep(2)

  def make_tmux_data(self):
    try:
      subprocess.run("rm /data/media/tmux.log; tmux capture-pane -pq -S-1000 > /data/media/tmux.log", shell=True, capture_output=True, text=False)
      subprocess.run("/data/openpilot/selfdrive/apilot.py", shell=True, capture_output=True, text=False)
    except Exception as e:
      print(f"TMUX creation error: {e}")
      return

  def send_tmux(self, ftp_password, tmux_why, send_settings=False):

    ftp_server = "shind0.synology.me"
    ftp_port = 8021
    ftp_username = "carrotpilot"
    ftp = FTP()
    ftp.connect(ftp_server, ftp_port)
    ftp.login(ftp_username, ftp_password)
    car_selected = Params().get("CarName")
    if car_selected is None:
      car_selected = "none"
    else:
      car_selected = car_selected

    git_branch = Params().get("GitBranch")
    try:
      ftp.mkd(git_branch)
    except Exception as e:
      print(f"Directory creation failed: {e}")
    ftp.cwd(git_branch)

    directory = car_selected + " " + Params().get("DongleId")
    current_time = datetime.now().strftime("%Y%m%d-%H%M%S")
    filename = tmux_why + "-" + current_time + "-" + git_branch + ".txt"

    try:
      ftp.mkd(directory)
    except Exception as e:
      print(f"Directory creation failed: {e}")
    ftp.cwd(directory)

    try:
      with open("/data/media/tmux.log", "rb") as file:
        ftp.storbinary(f'STOR {filename}', file)
    except Exception as e:
      print(f"ftp sending error...: {e}")

    if send_settings:
      self.save_toggle_values()
      try:
        #with open("/data/backup_params.json", "rb") as file:
        with open("/data/toggle_values.json", "rb") as file:
          ftp.storbinary(f'STOR toggles-{current_time}.json', file)
      except Exception as e:
        print(f"ftp params sending error...: {e}")

    ftp.quit()

  def carrot_panda_debug(self):
    #time.sleep(2)
    while True:
      if self.show_panda_debug:
        self.show_panda_debug = False
        try:
          subprocess.run("/data/openpilot/selfdrive/debug/debug_console_carrot.py", shell=True)
        except Exception as e:
          print(f"debug_console error: {e}")
          time.sleep(2)
      else:
        time.sleep(1)

  def save_toggle_values(self):
    try:
      import openpilot.selfdrive.frogpilot.fleetmanager.helpers as fleet

      toggle_values = fleet.get_all_toggle_values()
      file_path = os.path.join('/data', 'toggle_values.json')
      with open(file_path, 'w') as file:
        json.dump(toggle_values, file, indent=2)
    except Exception as e:
      print(f"save_toggle_values error: {e}")

  def carrot_cmd_zmq(self):

    context = zmq.Context()
    def setup_socket():
        socket = context.socket(zmq.REP)
        socket.bind("tcp://*:7710")
        poller = zmq.Poller()
        poller.register(socket, zmq.POLLIN)
        return socket, poller

    socket, poller = setup_socket()
    isOnroadCount = 0
    is_tmux_sent = False

    print("#########carrot_cmd_zmq: thread started...")
    while True:
      try:
        socks = dict(poller.poll(100))

        if socket in socks and socks[socket] == zmq.POLLIN:
          message = socket.recv(zmq.NOBLOCK)
          print(f"Received:7710 request: {message}")
          json_obj = json.loads(message.decode())
        else:
          json_obj = None

        if json_obj is None:
          isOnroadCount = isOnroadCount + 1 if self.params.get_bool("IsOnroad") else 0
          if isOnroadCount == 0:
            is_tmux_sent = False
          if isOnroadCount == 1:
            self.show_panda_debug = True

          network_type = self.sm['deviceState'].networkType # if not force_wifi else NetworkType.wifi
          networkConnected = False if network_type == NetworkType.none else True

          if isOnroadCount == 500:
            self.make_tmux_data()
          if isOnroadCount > 500 and not is_tmux_sent and networkConnected:
            self.send_tmux("Ekdrmsvkdlffjt7710", "onroad", send_settings = True)
            is_tmux_sent = True
          carrot_exception = self.params.get("CarrotException")
          if carrot_exception in ["exception", "log", "tmux_send"] and networkConnected:
            self.params.put_bool("CarrotException", "")
            self.make_tmux_data()
            self.send_tmux("Ekdrmsvkdlffjt7710", carrot_exception)
        elif 'echo_cmd' in json_obj:
          try:
            result = subprocess.run(json_obj['echo_cmd'], shell=True, capture_output=True, text=False)
            exitStatus = result.returncode
            try:
              stdout = result.stdout.decode('utf-8')
              stderr = result.stderr.decode('utf-8')
            except UnicodeDecodeError:
              stdout = result.stdout.decode('euc-kr', 'ignore')
              stderr = result.stderr.decode('euc-kr', 'ignore')

            echo = json.dumps({"echo_cmd": json_obj['echo_cmd'], "exitStatus": exitStatus, "result": stdout, "error": stderr})
          except Exception as e:
            echo = json.dumps({"echo_cmd": json_obj['echo_cmd'], "exitStatus": exitStatus, "result": "", "error": f"exception error: {str(e)}"})
          #print(echo)
          socket.send(echo.encode())
        elif 'tmux_send' in json_obj:
          self.make_tmux_data()
          self.send_tmux(json_obj['tmux_send'], "tmux_send")
          echo = json.dumps({"tmux_send": json_obj['tmux_send'], "result": "success"})
          socket.send(echo.encode())
      except Exception as e:
        print(f"carrot_cmd_zmq error: {e}")
        socket.close()
        time.sleep(1)
        socket, poller = setup_socket()

  def recvall(self, sock, n):
    """n바이트를 수신할 때까지 반복적으로 데이터를 받는 함수"""
    data = bytearray()
    while len(data) < n:
      packet = sock.recv(n - len(data))
      if not packet:
        return None
      data.extend(packet)
    return data

  def receive_double(self, sock):
    double_data = self.recvall(sock, 8)  # Double은 8바이트
    return struct.unpack('!d', double_data)[0]

  def receive_float(self, sock):
    float_data = self.recvall(sock, 4)  # Float은 4바이트
    return struct.unpack('!f', float_data)[0]


  def send_routes(self, coords, from_navd=False):
    if from_navd:
      if len(coords) > 0:
        self.navi_points = [(c.longitude, c.latitude) for c in coords]
        self.navi_points_start_index = 0
        self.navi_points_active = True
        print("Received points from navd:", len(self.navi_points))
        self.navd_active = True
        self._navi_route_source = "navd"  # [182차 계측]

        # 경로수신 -> carrotman active되고 약간의 시간지연이 발생함..
        if not from_navd:
          self.carrot_serv.active_count = 80
          self.carrot_serv.active_sdi_count = self.carrot_serv.active_sdi_count_max
          self.carrot_serv.active_carrot = 2

        coords = [{"latitude": c.latitude, "longitude": c.longitude} for c in coords]
        #print("navdNaviPoints=", self.navi_points)
      else:
        print("Received points from navd: 0")
        self.navd_active = False

    msg = messaging.new_message('navRoute', valid=True)
    msg.navRoute.coordinates = coords
    self.pm.send('navRoute', msg)

  def carrot_route(self):
    host = '0.0.0.0'  # 혹은 다른 호스트 주소
    port = 7709  # 포트 번호

    try:
      with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()

        while True:
          print("################# waiting connection from CarrotMan route #####################")
          conn, addr = s.accept()
          with conn:
            print(f"Connected by {addr}")
            #self.clear_route()

            # 전체 데이터 크기 수신
            total_size_bytes = self.recvall(conn, 4)
            if not total_size_bytes:
              print("Connection closed or error occurred")
              continue
            try:
              total_size = struct.unpack('!I', total_size_bytes)[0]
              # 전체 데이터를 한 번에 수신
              all_data = self.recvall(conn, total_size)
              if all_data is None:
                  print("Connection closed or incomplete data received")
                  continue

              self.navi_points = []
              points = []
              for i in range(0, len(all_data), 8):
                x, y = struct.unpack('!ff', all_data[i:i+8])
                self.navi_points.append((x, y))
                coord = Coordinate.from_mapbox_tuple((x, y))
                points.append(coord)
              coords = [c.as_dict() for c in points]
              self.navi_points_start_index = 0
              self.navi_points_active = True
              self._navi_route_source = "tcp_raw"  # [182차 계측] TCP 7709 경로
              print("Received points:", len(self.navi_points))
              #print("Received points:", self.navi_points)

              self.send_routes(coords)
              """
              try:
                module_name = "route_engine"
                class_name = "RouteEngine"
                moduel = importlib.import_module(module_name)
                cls = getattr(moduel, class_name)
                route_engine_instance = cls(name="Loaded at Runtime")

                route_engine_instance.send_route_coords(coords, True)
              except Exception as e:
                print(f"route_engine error: {e}")

              #msg = messaging.new_message('navRoute', valid=True)
              #msg.navRoute.coordinates = coords
              #self.pm.send('navRoute', msg)
              """

              if len(coords):
                dest = coords[-1]
                dest['place_name'] = "External Navi"
                self.params.put("NavDestination", json.dumps(dest))

            except Exception as e:
              print(e)
    except Exception as e:
      print("################# CarrotMan route server error #####################")
      print(e)

  def carrot_curve_speed_params(self):
    # [99차/100차] 매 호출(20Hz)마다 새로 읽던 것을 __init__/_refresh_cached_params()의
    # 캐시값으로 대체 (100프레임=5s마다 갱신, 제어 로직/결과값 동일).
    self.autoCurveSpeedFactor = self._auto_curve_speed_factor
    self.autoCurveSpeedAggressiveness = self._auto_curve_speed_aggressiveness

  def carrot_curve_speed(self, sm):
    self.carrot_curve_speed_params()
    if not sm.alive['carState'] and not sm.alive['modelV2']:
        return 250
    #print(len(sm['modelV2'].orientationRate.z))
    if len(sm['modelV2'].orientationRate.z) == 0:
        return 250

    return self.vturn_speed(sm['carState'], sm)

  def vturn_speed(self, CS, sm):
    """전방 커브에 대해 과속방지턱(calculate_current_speed)과 동일한
    v_i^2 = v_f^2 + 2ad 물리공식으로 '미리 서서히 감속 -> 정점 근처에서 목표속도 유지
    -> 커브를 빠져나오는 즉시 제약 해제' 형태의 주행감을 만든다.

    방지턱과의 차이는, 방지턱은 내비게이션 데이터로부터 고정된 지점까지의 거리를
    받아오는 반면, 커브는 비전모델이 매 프레임 새로 예측하는 전방 궤적(위치/속도/
    회전각속도)에서 '지점별 필요속도'를 직접 계산한다는 점이다. 단일 정점(최대 곡률
    지점)까지의 거리만 보지 않고 예측구간 내 모든 지점에 대해 방지턱과 같은 공식을
    적용한 뒤 그중 가장 엄격한(작은) 값을 취하므로, S자 커브처럼 정점 앞에 더 가까운
    완만한 커브가 끼어 있어도 놓치지 않는다.
    """
    TARGET_LAT_A = 1.6  # m/s^2

    modelData = sm['modelV2']

    orientation_rate = np.array(modelData.orientationRate.z, dtype=np.float64) * self.autoCurveSpeedFactor
    velocity = np.array(modelData.velocity.x, dtype=np.float64)
    position = np.array(modelData.position.x, dtype=np.float64)

    n = min(len(orientation_rate), len(velocity), len(position))
    if n == 0:
      return 250.0
    orientation_rate = orientation_rate[:n]
    velocity = velocity[:n]
    position = position[:n]

    valid = np.isfinite(orientation_rate) & np.isfinite(velocity) & np.isfinite(position)
    orientation_rate = orientation_rate[valid]
    velocity = velocity[valid]
    position = position[valid]
    if len(orientation_rate) == 0:
      return 250.0

    # 진입 조기감속용 예측 구간을 '초' 단위로 산정한다. ModelConstants.T_IDXS는
    # 뒤로 갈수록 간격이 넓어지는 비선형(2차) 배열이므로, 인덱스 개수를 고정하면
    # 실제 예측 시간이 모델 프레임 수에 따라 달라진다.
    n_pts = min(len(orientation_rate), ModelConstants.IDX_N)
    t_idxs = np.array(ModelConstants.T_IDXS[:n_pts])
    within_horizon = np.count_nonzero(t_idxs <= self.vturn_lookahead_horizon_s)
    lookahead_steps = max(5, min(n_pts, within_horizon))

    lookahead_rate = orientation_rate[:lookahead_steps]
    lookahead_vel = velocity[:lookahead_steps]
    lookahead_pos = np.maximum(position[:lookahead_steps], 0.0)  # 후방(음수) 지점은 배제
    lookahead_t = t_idxs[:lookahead_steps]

    # 각 지점에서, 모델이 예측한 그 지점의 주행속도로 지날 때 발생하는 횡가속도를
    # 근거로 그 지점에서 필요한 속도상한(=커브 심할수록 낮음)을 지점별로 계산한다.
    adjusted_target_lat_a = TARGET_LAT_A * self.autoCurveSpeedAggressiveness
    point_lat_acc = np.abs(lookahead_rate) * np.abs(lookahead_vel)
    point_curve = point_lat_acc / np.maximum(lookahead_vel, 0.1) ** 2
    point_target_speed = np.where(
      point_curve > 1e-8,
      np.clip((adjusted_target_lat_a / np.maximum(point_curve, 1e-8)) ** 0.5 * 3.6, 5.0, 250.0),
      250.0,
    )

    # ---- 과속방지턱과 동일한 감속 프로파일을 모든 지점에 벡터화 적용 ----
    # carrot_serv.calculate_current_speed()의 v_i^2 = v_f^2 + 2ad 공식과 동일하며,
    # safe_time만큼 여유를 두고 정점 이전에 이미 목표속도에 도달하도록 한다.
    safe_speed_mps = point_target_speed / 3.6
    safe_dist = safe_speed_mps * self.vturn_safe_time
    decel_dist = np.maximum(lookahead_pos - safe_dist, 0.0)
    required_speed_mps = np.sqrt(np.maximum(safe_speed_mps ** 2 + 2 * self.vturn_decel_rate * decel_dist, 0.0))
    required_speed_kph = np.clip(required_speed_mps * 3.6, 5.0, 250.0)

    # 여러 지점 중 가장 엄격한(=지금 당장 가장 느려야 하는) 지점이 최종 제약이 된다.
    apex_idx = int(np.argmin(required_speed_kph))
    turnSpeed = float(required_speed_kph[apex_idx])

    # ---- [82차] 원복(가속) 측 대칭 버퍼 ----
    # 위 safe_dist는 진입(감속) 측에만 적용되는 비대칭 버퍼다 -- "차량이 실제로
    # 그 감속도까지 도달하는 데 시간이 걸리니 정점보다 safe_time만큼 미리
    # 도달시키자"는 논리(81차)인데, 가속(원복) 반응도 동일하게 지연이 있으므로
    # 같은 논리를 대칭 적용한다. lookahead 배열은 항상 전방(미래) 지점만 담고
    # 있어 "정점을 이미 지난" 지점을 직접 표현할 수 없으므로, 정점을 실제보다
    # vturn_safe_time(=CS.vEgo 기준 거리)만큼 더 일찍 지난 것으로 가정한
    # 두 번째 후보를 계산해 -- 이번 프레임이 이미 회복(가속) 추세일 때만 -- 채택한다.
    # 진입(감속) 경로는 이 블록과 완전히 무관하게 그대로 유지되어 회귀 위험이 없다.
    accel_lead_dist = max(CS.vEgo, 0.0) * self.vturn_safe_time
    decel_dist_recovery = np.maximum(lookahead_pos - safe_dist + accel_lead_dist, 0.0)
    required_speed_mps_recovery = np.sqrt(np.maximum(
      safe_speed_mps ** 2 + 2 * self.vturn_decel_rate * decel_dist_recovery, 0.0))
    required_speed_kph_recovery = np.clip(required_speed_mps_recovery * 3.6, 5.0, 250.0)
    apex_idx_recovery = int(np.argmin(required_speed_kph_recovery))
    turnSpeed_recovery = float(required_speed_kph_recovery[apex_idx_recovery])

    if turnSpeed > self.vturn_last_speed and turnSpeed_recovery > turnSpeed:
      # 상승 추세(가속/원복 중)이고, 대칭 버퍼를 적용하면 더 일찍 풀리는(더 높은)
      # 값이 나올 때만 채택 -- 새 커브가 나타나 다시 조여야 하는 프레임에서는
      # turnSpeed(진입 계산값)가 그대로 낮게 유지되므로 자동으로 무시된다.
      turnSpeed = turnSpeed_recovery
      apex_idx = apex_idx_recovery

    # 방향 판단: 실제로 속도를 제한하는 지점 기준. 전방에 유의미한 커브가 없어
    # 해당 지점도 사실상 무제한(커브 없음)이면 근거리 가중합으로 대체한다.
    if point_curve[apex_idx] > 1e-8:
      curv_direction = np.sign(lookahead_rate[apex_idx])
    else:
      weights = np.clip(1.0 - 0.55 * (lookahead_t / max(self.vturn_lookahead_horizon_s, 0.1)), 0.45, 1.0)
      curv_direction = np.sign(np.sum(lookahead_rate * weights))
    if curv_direction == 0:
      curv_direction = np.sign(orientation_rate[0]) if orientation_rate[0] != 0 else 1.0

    # ---- 저역통과 필터 ----
    # 감속/가속의 '모양'(미리 서서히 감속, 벗어나는 즉시 해제)은 이미 위의 거리기반
    # 물리공식이 만들어내므로, 여기서는 모델 프레임 노이즈로 인한 잔떨림만 제거한다.
    # 별도의 '진입/탈출 이벤트' 판정이나 지연(hold) 로직은 두지 않는다.
    dt = 1.0 / 20.0  # carrot_man 브로드캐스트 루프 주기 (Ratekeeper(20))
    if np.isfinite(self.vturn_last_speed):
      rc = self.vturn_decel_rc if turnSpeed < self.vturn_last_speed else self.vturn_accel_rc
      alpha = dt / (rc + dt)
      turnSpeed = self.vturn_last_speed + (turnSpeed - self.vturn_last_speed) * alpha
    self.vturn_last_speed = float(turnSpeed)

    return turnSpeed * curv_direction

  def carrot_navi_thread(self):
    self.carrot_navi_tcp_server(7712)

  def handle_route(self, arr: list):
    if not arr:
      print("Received route: 0")
      # navd route가 비어오면 비활성 처리
      self.navi_points = []
      self.navi_points_start_index = 0
      self.navi_points_active = False
      self.navd_active = False
      return

    # valid만 필터 (필요 없으면 제거)
    valid_pts = [p for p in arr if isinstance(p, dict) and p.get("valid", True)]
    if not valid_pts:
      print("Received route: 0 valid")
      self.navi_points = []
      self.navi_points_start_index = 0
      self.navi_points_active = False
      self.navd_active = False
      return

    # x=lon, y=lat
    coords = []
    navi_points = []

    for p in valid_pts:
      try:
        lon = float(p.get("x"))
        lat = float(p.get("y"))
      except Exception:
        continue

      navi_points.append((lon, lat))
      coords.append({"latitude": lat, "longitude": lon})

    self.navi_points = navi_points
    self.navi_points_start_index = 0
    self.navi_points_active = True
    self.navd_active = True
    self._navi_route_source = "tcp_navi"  # [182차 계측] TCP 7712 handle_route 경로

    print("Received points:", len(self.navi_points))

    self.send_routes(coords)

    if coords:
      dest = dict(coords[-1])
      dest["place_name"] = "External Navi"
      try:
        self.params.put("NavDestination", json.dumps(dest))
      except Exception as e:
        print("NavDestination put error:", e)

  def handle_traffic_light(self, d: dict):
    print(f"[Traffic] {d}")

    # {'distance': 120, 'greenLightRemainTime': 0, 'leftLightRemainTime': 0, 'location': {'coordString': 'x:127.045286, y:37.477032', 'latitude': 37.47703188722564, 'longitude': 127.04528634430659},
    #       'redLightRemainTime': 15, 'rightLightRemainTime': 0, 'uturnLightRemainTime': 0, 'greenLightOn': False, 'leftLightOn': False, 'redLightOn': True, 'rightLightOn': False, 'uturnLightOn': False}



  def handle_carrot_state(self, d: dict):
    try:
      self.carrot_serv.update(d)
    except Exception as e:
      print("carrot_state update error:", e)

  def handle_unknown(self, obj: Any):
    print("[UNKNOWN]", str(obj)[:200])

  def _get_timestamp_ms(self, obj: Any) -> int:
    if not isinstance(obj, dict):
      return 0
    try:
      return int(obj.get("timestamp_ms", 0))
    except Exception:
      return 0


  def _is_stale_rgdata(self, timestamp_ms: int):
    if timestamp_ms <= 0:
      return False, 0

    with self._rgdata_ts_lock:
      last_ts = self._last_rgdata_timestamp_ms
      if timestamp_ms <= last_ts:
        return True, last_ts

      self._last_rgdata_timestamp_ms = timestamp_ms
      return False, last_ts
  
  def _dispatch_obj(self, obj: Any):
    if obj is None:
      return

    # obj가 str이면 여기서 JSON 파싱
    if isinstance(obj, str):
      s = obj.strip()
      if not s:
        return
      try:
        obj = json.loads(s)
      except Exception:
        # JSON 아니면 unknown 처리
        return self.handle_unknown(s[:200])

    if not isinstance(obj, dict):
      return self.handle_unknown(obj)

    if "vrtx" in obj:
      self.handle_route(obj["vrtx"])

    if "rgdata" in obj:
      timestamp_ms = self._get_timestamp_ms(obj)
      stale, last_ts = self._is_stale_rgdata(timestamp_ms)
      if stale:
        print(f"[STALE DROP] rgdata ts={timestamp_ms} <= last={last_ts}")
      else:
        self.handle_carrot_state(obj["rgdata"])
      
    if "sinf" in obj:
      self.handle_traffic_light(obj["sinf"])

  def carrot_navi_http_thread(self):
    asyncio.run(self.carrot_navi_http_server(7713))

  def carrot_navi_tcp_server(self, port: int = 7712):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", port))
    server.listen(5)
    print("TCP server listening", port)

    while True:
      conn, addr = server.accept()
      self.remote_addr = addr
      print("Connected:", addr)
      conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
      try:
        f = conn.makefile("r", encoding="utf-8", errors="ignore")
        while True:
          try:
            line = f.readline()
          except socket.timeout:
            print("TCP timeout: closing connection", addr)
            break

          if not line:
            break

          s = line.strip()
          if not s:
            continue

          try:
            obj = json.loads(s)
          except Exception:
            obj = s

          try:
            self._dispatch_obj(obj)
          except Exception as e:
            print("dispatch error:", e, "raw:", repr(s[:200]))

      except Exception as e:
        print("TCP error:", e)

      finally:
        try:
          conn.close()
        except Exception:
          pass
        self.remote_addr = None

  async def carrot_http_post(self, request: web.Request):
    tmap_version = request.match_info.get("tmap_version", "")

    try:
      peer = request.transport.get_extra_info("peername")
    except Exception:
      peer = None

    #print(f"[HTTP] request from={peer} version={tmap_version}")

    try:
      obj = await request.json()
      #if isinstance(obj, dict):
      #  print(f"[HTTP] json keys={list(obj.keys())[:10]}")
      #else:
      #  print(f"[HTTP] json type={type(obj).__name__}")
    except Exception as e:
      print(f"[HTTP] json parse error: {e}")
      return web.json_response({
        "ok": False,
        "error": f"invalid json: {e}"
      }, status=400)

    if isinstance(obj, dict):
      obj["_tmap_version"] = tmap_version

    try:
      self._dispatch_obj(obj)
      #print(f"[HTTP] dispatch ok version={tmap_version}")
      #print(obj)
      return web.json_response({
        "ok": True,
        "tmap_version": tmap_version
      })
    except Exception as e:
      print(f"[HTTP] dispatch error: {e}")
      traceback.print_exc()
      return web.json_response({
        "ok": False,
        "error": str(e),
        "tmap_version": tmap_version
      }, status=500)
  
  async def carrot_http_health(self, request: web.Request):
    return web.json_response({
      "ok": True,
      "service": "carrot_navi_http"
    })

  async def carrot_navi_http_server(self, port: int = 7713):
    app = web.Application(client_max_size=1024 * 1024)

    app.router.add_post("/api/navi/{tmap_version}", self.carrot_http_post)
    app.router.add_get("/health", self.carrot_http_health)

    runner = web.AppRunner(app, access_log=None)
    await runner.setup()

    site = web.TCPSite(runner, "0.0.0.0", port)
    await site.start()

    print("HTTP server listening", port)

    while True:
      await asyncio.sleep(3600)

def main():
  try:
    set_core_affinity([0, 1, 2, 3])
  except Exception:
    print("[carrot_man] failed to set core affinity")

  print("CarrotManager Started")
  #print("Carrot GitBranch = {}, {}".format(Params().get("GitBranch"), Params().get("GitCommitDate")))
  carrot_man = CarrotMan()

  print(f"CarrotMan {carrot_man}")
  threading.Thread(target=carrot_man.kisa_app_thread).start()
  threading.Thread(target=carrot_man.carrot_navi_thread).start()
  threading.Thread(target=carrot_man.carrot_navi_http_thread).start()
  
  while True:
    try:
      carrot_man.carrot_man_thread()
    except Exception as e:
      print(f"carrot_man error...: {e}")
      traceback.print_exc()
      time.sleep(10)


if __name__ == "__main__":
  main()
