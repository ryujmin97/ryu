using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

@0xb526ba661d550a59;

# custom.capnp: a home for empty structs reserved for custom forks
# These structs are guaranteed to remain reserved and empty in mainline
# cereal, so use these if you want custom events in your fork.

# DO rename the structs
# DON'T change the identifier (e.g. @0x81c2f05a394cf4af)

# you can rename the struct, but don't change the identifier
struct CarrotMan @0x81c2f05a394cf4af {
	activeCarrot @0 : Int32;
	nRoadLimitSpeed @1 : Int32;
	remote @2 : Text;
	xSpdType @3 : Int32;
	xSpdLimit @4 : Int32;
	xSpdDist @5 : Int32;
	xSpdCountDown @6 : Int32;
	xTurnInfo @7 : Int32;
	xDistToTurn @8 : Int32;
	xTurnCountDown @9 : Int32;
	atcType @10 : Text;
	vTurnSpeed @11 : Int32;
	szPosRoadName @12 : Text;
	szTBTMainText @13 : Text;
	desiredSpeed @14 : Int32;
	desiredSource @15 : Text;
	carrotCmdIndex @16 : Int32;
	carrotCmd @17 : Text;
	carrotArg @18 : Text;
	xPosLat @19 : Float32;
	xPosLon @20 : Float32;
	xPosAngle @21 : Float32;
	xPosSpeed @22 : Float32;
	trafficState @23 : Int32;
	nGoPosDist @24 : Int32;
	nGoPosTime @25 : Int32;
	szSdiDescr @26 : Text;
	naviPaths @27 : Text;
	leftSec @28 : Int32;
	# [169차 계측] "패킷단절 vs 내용정지" 구분용 (FINDINGS.md 169차
	# NEEDS_INVESTIGATION). 기존 vpPosPointLatNavi/LonNavi와
	# last_update_gps_time_navi/last_calculate_gps_time은 cereal
	# 미발행이라 실차 로그에서 이 구분이 불가능했음.
	vpPosPointLatNavi @29 : Float32;   # navi 원본 위도(estimate_position 입력값, 폴백 전)
	vpPosPointLonNavi @30 : Float32;   # navi 원본 경도
	dtNaviPacketAge @31 : Float32;     # now - last_update_gps_time_navi (초). 3.0 초과 시 "패킷단절"
	positionDtSinceFix @32 : Float32;  # 162/163/167차 게이트가 실제로 읽는 값(now - last_calculate_gps_time)
	ccPoseValid @33 : Bool;            # 166/167차 CC.orientationNED 기반 방향1 유효 여부(len(ned)>2)

	# [182차 계측] carrotMan.navi_points_active(route 폴리라인 활성 플래그)가
	# 이전까지 cereal 미발행이라 "route 사전감속이 61초간 없었다"는 현상을
	# 실차 로그(rlog)만으로 재현/원인규명할 수 없었음(FINDINGS.md 182차 —
	# navi_points_active=False 61초 지속으로 carrot_navi_route()가 곡률계산
	# 자체를 스킵, route=390.0 "제약없음" 기본값 노출). 162/163차 게이트
	# (positionDtSinceFix)와는 무관한 별개 실패모드 -- navi_points_active는
	# navi_points_active=True인 상태에서의 위치추정 오차가 아니라, 그 전
	# 단계인 "route 폴리라인 수신 자체"가 끊기는 문제.
	naviPointsActive @34 : Bool;  # carrot_man.navi_points_active 그대로 발행
	navdActive @35 : Bool;        # carrot_man.navd_active 그대로 발행 (navd cereal 경로 활성 여부)
	dtRouteInactive @36 : Float32; # navi_points_active=False 상태 지속시간(초). True면 0.0
	routeSource @37 : Text;        # 마지막으로 route를 성공 수신한 경로: "navd"/"tcp_raw"(7709)/"tcp_navi"(7712 handle_route)/""(아직 없음)

	# [194차] 193차에서 carrot_man.py 내부(_route_apex_idx 등)에만 저장되고
	# cereal에는 발행되지 않던 route apex 진단 telemetry를 실제로 rlog에
	# 싣기 위해 추가. 기존 필드(@0~@37)는 건드리지 않고 뒤에 append.
	routeApexIdx @38 : Int32;      # carrot_navi_route()가 선택한 apex(최대곡률지점) 인덱스. 미계산/스킵 시 -1
	routeApexDist @39 : Float32;   # 현재위치 ~ apex까지 거리(m)
	routeApexSpeed @40 : Float32;  # apex 지점 목표속도(km/h)
	routeOutSpeed @41 : Float32;   # calculate_current_speed()가 계산한 route 최종 출력 속도(km/h, min(...,300.0) 적용 전 값)

	# [204차 계측, 203차 사용자 결정 옵션1] apex_idx/speed 1개만으로는
	# "허위 직선 스파이크(먼 후보로 순간 전환)"와 "정상적인 연속곡선/
	# 커브탈출가속 중 후보 전환"을 구분할 수 없음이 203차 시뮬레이션으로
	# 확인됨(WIP.md 203차 참고). carrot_navi_route()가 apex 선택 직전에
	# 실제로 갖고 있던 후보 리스트(candidates, road_limit_speed 미만인
	# 지점들, 거리 오름차순)의 개수와 최근접 3개를 그대로 노출한다.
	# 제어 로직/값은 전혀 변경하지 않음 -- 순수 관측용 추가 필드.
	routeCandidateCount @42 : Int32;  # candidates 리스트 길이. 0이면 179차 폴백(전역 최소, "직선" 판정) 경로
	routeCandidate0Idx @43 : Int32;   # 최근접 후보(=apex로 선택된 candidates[0])의 speeds[] 인덱스. 없으면 -1
	routeCandidate0Dist @44 : Float32;
	routeCandidate0Speed @45 : Float32;
	routeCandidate1Idx @46 : Int32;   # 두 번째로 가까운 후보. 없으면 -1
	routeCandidate1Dist @47 : Float32;
	routeCandidate1Speed @48 : Float32;
	routeCandidate2Idx @49 : Int32;   # 세 번째로 가까운 후보. 없으면 -1
	routeCandidate2Dist @50 : Float32;
	routeCandidate2Speed @51 : Float32;
}

struct CustomReserved1 @0xaedffd8f31e7b55d {
}

struct CustomReserved2 @0xf35cc4560bbf6ec2 {
}

struct CustomReserved3 @0xda96579883444c35 {
}

struct CustomReserved4 @0x80ae746ee2596b11 {
}

struct CustomReserved5 @0xa5cd762cd951a455 {
}

struct CustomReserved6 @0xf98d843bfd7004a3 {
}

struct CustomReserved7 @0xb86e6369214c01c8 {
}

struct CustomReserved8 @0xf416ec09499d9d19 {
}

struct CustomReserved9 @0xa1680744031fdb2d {
}

struct CustomReserved10 @0xcb9fd56c7057593a {
}

struct CustomReserved11 @0xc2243c65e0340384 {
}

struct CustomReserved12 @0x9ccdc8676701b412 {
}

struct CustomReserved13 @0xcd96dafb67a082d0 {
}

struct CustomReserved14 @0xb057204d7deadf3f {
}

struct CustomReserved15 @0xbd443b539493bc68 {
}

struct CustomReserved16 @0xfc6241ed8877b611 {
}

struct CustomReserved17 @0xa30662f84033036c {
}

struct CustomReserved18 @0xc86a3d38d13eb3ef {
}

struct CustomReserved19 @0xa4f1eb3323f5f582 {
}
