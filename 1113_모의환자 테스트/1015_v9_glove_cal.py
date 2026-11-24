"""
3finger_cali_coupling_UI_v30.py — 손가락별 Auto-tensioning / 양방향 제어

v30 변경:
  - 손가락별 Auto-tensioning과 양방향 결합 체크를 독립적으로 분리.
    Auto: 지도 기반 슬랙 PID + T_BASE. 양방향: KC/BC 결합력.
  - 구동 해제, 탭 전환, 0점 설정, 비상정지 시 적용 게인 보존.
  - 파라미터 일괄 검증/적용, 미적용 입력 및 루프 적용값 표시.
  - 비상정지/해제 상단 고정, 작은 화면은 탭 내용 스크롤.
  - Extension-only return 호출 오류 수정 및 제어 예외 시 출력 정리.

v33 변경 (Extension return 을 힘 기준 full extension 으로 통일):
  - 0점 위치 서보 제거. 서보가 0점 근처에서 스스로 힘을 빼 손가락이 멈춘 것을
    '끝 도달'로 오판 → full extension 전에 다음 단계로 넘어가던 원인.
  - passive(FLEX 풀기 + EXT take-up) → force(EXT 장력 램프) 2단계.
  - 정지 판정은 적용 장력이 목표(T_CAL_E, 상한 RETURN_CMD_MAX_N)의 90% 이상일
    때만 센다. 굽힘(T_CAL_F 당김 중 정지)과 대칭인 정의.
  - FLEX payout 은 EXT 엔코더가 움직이는 동안에만 (정지 시 슬랙 축적 방지).
  - 완료/abort 메시지에 그 시점의 EXT 적용 장력을 표시.

v34 변경 (중간 구간에서 멈추는 문제):
  - FLEX payout 을 속도 게이트 → 예산 방식으로. 굽힘 시작점 + 25° 까지 되풀릴 동안은
    손가락이 멈춰 있어도 계속 풀어 준다 (FLEX 텐던이 붙잡아 중간에 서던 원인 제거).
  - Breakaway: 목표 장력에서 0점 밖에 멈추면 abort 전에 0.3 s 힘 펄스
    (목표+2N, +4N, … 최대 14N, 4회) 로 stick-slip 을 끊는다. 다시 움직이면
    시도 횟수 리셋. 펄스 뒤에는 목표 장력으로 복귀.

v35 변경 (released 0점 기준 판정 제거):
  - 0점은 무부하 스풀 위치라 부하 하(7~8 N) 스풀 위치와 비교 불가 — 케이블 신장과
    직물/라우팅 settling 만으로 +30° 를 넘겨 overshoot abort 가 나던 원인.
  - ±15° 창과 +30° overshoot 대신, 이번 굽힘에서 EXT 가 풀어 준 양(Δe)을 기준으로
    되감은 비율(ret)로 판정: ret<80% 정지 → breakaway, ret≥80% 정지 → full extension,
    굽힘 시작점을 90° 이상 지나면 overtravel abort.
  - 과신전 보호는 힘 상한(RETURN_CMD_MAX_N)과 과속/케이블 한계 estop 이 담당.

v36 변경 (살살 당기기):
  - T_CAL_E 기본 14 → 5 N, UI 상한 10 N. 램프 4 → 1 N/s. passive 단계에 1.5 N 보조,
    최대 8 s. 정지 창 0.4 s/2° → 1.0 s/1.5° (느린 creep 도 움직임으로 인정).
  - 중간 정지 시 펄스 대신 목표 장력 유지 8 s (착용자/치료사 가벼운 보조 허용).
    breakaway 는 기본 OFF (EXT_BREAKAWAY_TRIES=0), 켜도 최대 8 N.
  - overtravel 한계 90 → 150° (gross guard 로만).

v37 변경 (완전히 펴졌는데 overtravel abort):
  - 굽힘 시작점 기준 'past' 는 전부 탄성 take-up(5 N 에서 ~150° ≈ 10 mm)임이 확인됨.
    abort 한계 150 → 400° (텐던 이탈 수준만). past 값은 사이클별로 meta.ext_takeup_deg
    에 기록 (시스템 직렬 컴플라이언스 데이터).

v38 변경 (다 펴졌는데 overtravel):
  - 정지 창 1.0 s/1.5° → 0.5 s/2.5°. 손가락이 펴진 뒤 글러브/앵커 creep(수 dps)이
    움직임으로 잡혀 정지 판정이 영원히 안 나던 원인.
  - take-up 한계(250°) 도달은 abort 가 아니라 '끝 도달' 처리. 메시지와 meta 에 표시.

v39 변경 (마지막 release 에서 'unstable' abort / extension 통증):
  - reference_settle: 한 시점 속도 검사 → 연속 0.45 s 잠잠 대기(최대 6 s). 탄성 recoil
    이 0.45 s 안에 안 끝나 3 사이클을 다 돌고 마지막에 abort 되던 원인.
  - FLEX 풀기 지령을 UI 파라미터 T_PAYOUT(기본 0.3 N-eq) 로 노출.
  - FLEX 풀기 예산 = 40° + ext take-up(past_start). 글러브가 밀린 만큼 flex 텐던도
    더 풀어야 하는데 25° 고정 예산이 먼저 바닥나 flex 가 손가락을 붙잡던 원인.

v40 변경 (실행 모드 extension 시 FLEX 풀기):
  - Auto-tensioning / 양방향 공통: motion=extending (source=flex) 일 때 FLEX 에
    -PO_FLEX (UI, 기본 0.2 N-eq) 풀기 지령. 지도상 기대 flex 위치보다 10° 이상
    앞서면 선형 감쇠, 30° 에서 0 (슬랙 → self-chasing 폭주 방지).

v41 변경 (굽힘 시 EXT 풀기, 대칭):
  - Sweep 굽힘: EXT 풀기 지령을 UI T_PAYOUT_E(기본 0.2) 로. 예산 = 직전 사이클 실측
    |Δe|/|Δf| × FLEX 감긴 양 + 30° (첫 사이클 비율 1.3). 예산 밖이면 0 (지도 오염 방지).
  - 실행: motion=flexing (source=ext) 일 때 EXT 에 -PO_EXT, 굽힘 branch 지도에 묶어
    extension 쪽과 같은 10°/30° 감쇠.

사용: 지도 준비 → 같은 펴진 자세에서 제어 0점 정렬(불러온 지도)
      → 파라미터 전체 적용 → 원하는 Auto/양방향 체크.
장갑1=환자, 장갑2=치료사. 엄지/검지/중지, 각 Flex/Ext 모터.
Sweep: 양쪽 3N 지령 5초 → 임시 0점 → warm-up 1회 → 공식 0점
       → 기록 3회 → 방향별 평균 지도 자동 저장.
장력 표시는 측정값이 아닌 전류 환산 지령값이다.
MOTOR_SPECS의 배선, 극성, 장치 설정은 실제 리그와 일치해야 한다.
"""

import os
import csv
import json
import math
import time
import queue
import threading
import statistics
import traceback
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

try:
    import ctypes
    ctypes.windll.winmm.timeBeginPeriod(1)
except Exception:
    pass

try:
    import nidaqmx
    from nidaqmx.constants import AngleUnits, EncoderType, EncoderZIndexPhase
    DAQ_AVAILABLE = True
except ImportError:
    DAQ_AVAILABLE = False


# ═══════════════════════════════════════════════════════ 설정

# ── 전류 변환 ──
AO_MAX_V, AO_MIN_V = 10.0, -10.0
AMP_AT_10V = 4.12
V_PER_AMP  = AO_MAX_V / AMP_AT_10V
KT      = 0.0156          # Nm/A
SPOOL_R = 0.0038          # m
DEG_TO_MM = math.radians(1.0) * SPOOL_R * 1000.0

# ── 안전 ──
I_MAX   = 4.12            # A; AO ±10 V current scaling; confirm ESCON and motor ratings
T_MAX   = I_MAX * KT / SPOOL_R
SLACK_SAT = 4.5           # N  슬랙 루프 힘 상한
TGT_RATE  = 700.0         # deg/s 목표 이동 속도 상한
SPIKE_VEL      = 7000.0   # deg/s
CABLE_LIMIT_MM = 2000.0
DTH_LIMIT      = 2000.0    # deg  Δθ 이상 판정

# ── 제어 기본값 (UI 실시간 변경) ──
DEFAULT_PARAMS = dict(
    KP1 = 0.0, KI1 = 0.0, KD1 = 0.0,   # 장갑1(환자) 슬랙 PID (KD=0 → PI)
    KP2 = 0.0, KI2 = 0.0, KD2 = 0.0,   # 장갑2(치료사)  KI=0 → PD
    ERR_DB = 3.0,                # deg 슬랙 허용
    KC1 = 0.0,  BC1 = 0.0,       # 장갑1 결합 게인 (환자가 받는 힘)
    KC2 = 0.0,  BC2 = 0.0,       # 장갑2 결합 게인 (치료사가 받는 힘)
    OVER = 0.0,                  # curl 끝범위 허용 (0~0.15, 스트레칭)
    PO_FLEX = 0.2,               # N-eq 실행 중 extension 시 FLEX(source) 풀기 지령 (0 = 끔)
    PO_EXT  = 0.2,               # N-eq 실행 중 flexion  시 EXT(source)  풀기 지령 (0 = 끔)
    T_BASE = 0.0,                # N 실행 중 상시 기본장력 (기본 0: antagonist 방해 최소화)
    T_CAL_F  =3.0, T_CAL_E = 3.0,   # N sweep 당김 장력 (지도 기준 장력, 고정 권장) — 사람 손: Ext 4~6 N
    T_PRE = 0.0,                # N sweep 반대쪽 팽팽 유지 장력
    T_PAYOUT = 0.2,             # N-eq sweep extension 중 FLEX 스풀 풀기 지령
    T_PAYOUT_E = 0.1,           # N-eq sweep flexion   중 EXT  스풀 풀기 지령
)
CONTROL_PARAM_LIMITS = dict(
    KP1=(0, 1), KI1=(0, 2), KD1=(0, 0.02),
    KP2=(0, 1), KI2=(0, 2), KD2=(0, 0.02),
    ERR_DB=(0, 30), T_BASE=(0, 2),
    KC1=(0, 0.5), BC1=(0, 0.05), KC2=(0, 0.5), BC2=(0, 0.05),
    OVER=(0, 0.15), PO_FLEX=(0, 0.6), PO_EXT=(0, 0.6),
)
CAL_PARAM_LIMITS = dict(T_CAL_F=(0.5, 6), T_CAL_E=(0.5, 10), T_PRE=(0, 2),
                        T_PAYOUT=(0, 1), T_PAYOUT_E=(0, 1))
LOG_TUNING_KEYS = tuple(
    k for k in CONTROL_PARAM_LIMITS
    if k not in ("KC1", "BC1", "KC2", "BC2", "OVER", "T_BASE")
)
I_CLAMP      = 3.0
I_DECAY      = 0.90
T_ASSIST_MAX = 15.0

# 모드 프리셋: (KC1, BC1, KC2, BC2)
#   환자 주도 : 환자 투명(KC1=0), 치료사 장갑이 환자를 강하게 추종 → 치료사가 느낌
#   치료사 주도: 환자에게 보조 전달 + 치료사도 환자 저항을 되받음 (비대칭 양방향)
#   구속(Hold) : 치료사가 자세 유지 → 환자 이탈(enslavement 등)을 강하게 저지
PRESETS = {
    "투명":       (0.00, 0.000, 0.00, 0.000),
    "환자 주도":   (0.00, 0.000, 0.15, 0.003),
    "치료사 주도": (0.10, 0.002, 0.06, 0.0015),
    "구속(Hold)":  (0.20, 0.004, 0.10, 0.002),
}

# ── 반전/역할 감지 ──
MOVE_TH  = 8.0           # deg/s
DIR_DIST = 5.0
DIR_CONFIRM_S = 0.10
# v40: 실행 중 extension 시 FLEX(source) 풀기. source 텐던은 센서이므로 '풀기'가 슬랙을
# 만들면 방향 판정과 driven 목표가 스풀 공회전을 따라가는 self-chasing 폭주가 된다.
# 그래서 풀기는 지도에 묶는다: 현재 ext 위치에 지도가 대응시키는 flex 위치보다
# LEAD_FULL 까지 앞서는 건 허용(전력), LEAD_OFF 에서 0 으로 선형 감쇠.
RUN_PAYOUT_LEAD_FULL_DEG = 10.0   # deg  PI 데드밴드(ERR_DB)+추종 지연 정도는 정상
RUN_PAYOUT_LEAD_OFF_DEG  = 30.0   # deg  (≈2 mm) 이보다 앞서면 슬랙으로 보고 풀기 중단
CURL_BLEND_TIME = 0.25           # deg
FRIC_FALLBACK = 2.5
VEL_CAP = 1000.0

# ── Sweep 캘리브레이션 ──
SWEEP_CYCLES   = 3        # map 계산에 사용하는 실제 기록 왕복 수
WARMUP_CYCLES = 1        # 첫 왕복은 settling/warm-up, map 평균 제외
TOTAL_SWEEP_CYCLES = WARMUP_CYCLES + SWEEP_CYCLES
SAMPLE_DEG     = 2.0      # deg  flex 이 이만큼 움직일 때마다 샘플
STALL_VEL      = 18.0     # deg/s 이하이면 정지로 간주
STALL_TIME     = 0.35     # s    정지 유지 시간 → 끝 도달 판정
MIN_TRAVEL     = 25.0     # deg  이만큼은 움직여야 끝 판정 허용 (초기 오판 방지)
PHASE_TIMEOUT  = 15.0     # s    한 방향 최대 시간 (초과 시 중단)
PRETEN_TIME     = 0.50      # s  장력 도달 + 정지 상태를 이 시간 연속 유지
PRETEN_TIMEOUT  = 4.0       # s  pretension 최대 대기 시간
PRETEN_FORCE_OK = 0.85      # 목표 pretension의 85% 이상

# ── Extension return (v33): 힘 기준 full extension ──
# 굽힘과 대칭인 정의: "T_CAL_E 로 당기는데도 더 안 움직이는 위치" = full extension.
# 0점을 향한 위치 서보는 사용하지 않는다(서보가 0점 근처에서 스스로 힘을 빼
# 손가락이 멈춘 것을 '끝 도달'로 오판하던 원인).
# 단계 1 passive : EXT는 take-up(T_PRE)만, FLEX 스풀을 풀어 글러브 탄성으로 펴지게 둔다.
# 단계 2 force   : EXT 장력을 T_CAL_E(상한 RETURN_CMD_MAX_N)까지 램프 → 목표 장력
#                  도달 후에만 정지 판정 창을 센다.
EXT_PASSIVE_MIN_S      = 1.0    # s   passive 최소 유지
EXT_PASSIVE_MAX_S      = 8.0    # s   passive 최대 대기 → force 단계로
EXT_PASSIVE_STALL_S    = 1.0    # s   passive 중 |vel| 작게 유지되면 force 단계로
EXT_PASSIVE_STALL_DPS  = 5.0    # deg/s
EXT_PASSIVE_ASSIST_N   = 1.5    # N   passive 중 EXT 가벼운 보조 (T_PRE 보다 크면 이 값)
EXT_RAMP_NPS           = 1.0    # N/s EXT 장력 램프 속도 — 살살 (5 N 까지 ~4 s)
# 중간 정지(ret < MIN_FRAC)에서 목표 장력을 유지하며 기다리는 시간. 점탄성 creep 과
# 착용자/치료사의 가벼운 보조로 풀리는 경우가 많다. 이 시간 안에 다시 움직이면 계속.
EXT_MIDSTALL_HOLD_S    = 8.0    # s
EXT_FORCE_OK_RATIO     = 0.90   # 적용 장력이 목표의 이 비율 이상일 때만 정지 판정 허용
# v35: released-preload 0점은 무부하 스풀 위치라 부하 하 스풀 위치와 비교할 수 없다
# (케이블 신장 + 라우팅/직물 settling 이 7~8 N 에서 이미 30° ≈ 2 mm). 0점 기준
# 판정(±15° 창, +30° overshoot)은 모두 제거하고, 이번 굽힘에서 EXT 스풀이 풀어 준
# 양(Δe)을 기준으로 삼는다:
#   returned/Δe < MIN_FRAC 에서 정지  → 중간 고착 → breakaway
#   returned/Δe ≥ MIN_FRAC 에서 정지  → full extension
#   returned  > Δe + OVERTRAVEL      → 텐던 이탈/착용 틀어짐 → abort
# 과신전 보호의 실체는 힘 상한(RETURN_CMD_MAX_N)이지 스풀 각도가 아니다.
EXT_RETURN_MIN_FRAC    = 0.80   # 굽힘 때 풀린 양의 이 비율 이상 되감겨야 끝으로 인정
# v37: 굽힘 시작점도 '풀어준(무부하) 상태'라서 부하 하 스풀 위치와 비교하면 탄성
# take-up(케이블 신장 + 글러브 직물 + 슬랙 회수)이 통째로 'past' 로 잡힌다. 실측 5 N
# 에서 +150° ≈ 10 mm, abort 후 장력을 풀면 그대로 되돌아옴 = 전부 탄성. 그래서 이
# 값은 판정이 아니라 **시스템 컴플라이언스 측정치**로 기록하고(meta.ext_takeup_deg),
# abort 는 텐던 이탈 수준(스풀이 끝없이 감김)에만 건다.
# v38: 한계 도달 시 abort 가 아니라 '끝 도달'로 처리한다. 손가락은 이미 펴져 있고
# 더 감기는 건 글러브/앵커 미끄러짐이므로 당기기를 멈추는 게 맞다. 메시지와
# meta.ext_takeup_deg 에 표시되어 하드웨어 점검 신호로 쓴다. 텐던 완전 이탈(스풀
# 공회전)은 CABLE_LIMIT_MM estop 이 잡는다.
EXT_OVERTRAVEL_DEG     = 250.0  # deg (≈17 mm) 이 이상 감기면 당기기 중단 = 끝 처리
EXT_RETURN_SETTLE      = 0.25   # s
# v38: 1.0 s/1.5°(=1.5 dps) 로 느슨하게 했더니 손가락이 다 펴진 뒤 글러브/앵커가
# 천천히 밀리는 creep(수 dps)이 '움직임'으로 잡혀 정지 판정이 영원히 안 났다.
# 손가락이 실제로 펴질 때는 수십 dps 이므로 5 dps 평균을 정지 경계로 둔다.
EXT_STALL_WINDOW       = 0.50   # s   정지 판정 창
EXT_STALL_DELTA_DEG    = 2.5    # deg/window  (= 5 dps 평균 이하면 정지)
EXT_STALL_VEL_TOL_DPS  = 40.0   # deg/s  느슨한 보호값 (1 count/8 ms = 11 dps 양자화 노이즈 고려)
# FLEX payout 예산(v34): return 중 FLEX 스풀을 풀 수 있는 양 =
#   (이번 굽힘에서 FLEX 가 감은 양) + 여유.  굽힘 때 감은 만큼만 되풀면 손가락은
#   시작 자세로 돌아갈 수 있고, 그 이상은 슬랙이다. 손가락이 멈춰 있어도 예산 안에서는
#   계속 풀어 준다 (FLEX 텐던이 손가락을 붙잡아 중간에 서는 것 방지).
EXT_PAYOUT_MARGIN_DEG  = 40.0   # deg (≈2.6 mm) + ext take-up(past_start) 만큼 추가 허용
# Breakaway(v34): 목표 장력에서 0점 밖에 멈추면(라우팅 stick-slip, 중간에 안 펴지는
# 구간) 바로 abort 하지 않고 짧은 힘 펄스로 끊어 준다. 펄스는 시간 제한, 상한 고정.
EXT_BREAKAWAY_STEP_N   = 1.0    # N   펄스 = 목표 + step × 시도횟수
EXT_BREAKAWAY_MAX_N    = 8.0    # N   펄스 절대 상한
EXT_BREAKAWAY_S        = 0.30   # s   펄스 지속
EXT_BREAKAWAY_TRIES    = 0      # 한 정지 구간당 최대 시도 (0 = 펄스 사용 안 함, 사람 손 기본)
EXT_BREAKAWAY_TOTAL    = 12     # return 한 번당 총 펄스 상한

# Full-extension reference preload
ZERO_PRELOAD_N         = 2.0   # N, flex/ext 각각
ZERO_PRELOAD_TIME      = 3.0
ZERO_RELEASE_TIME = 2.0
ZERO_SETTLE_TIME = 0.45   # s, 두 엔코더가 이 시간 동안 연속으로 잠잠해야 reference 확정
ZERO_SETTLE_VEL_DPS = 18.0  # deg/s
ZERO_SETTLE_TIMEOUT = 6.0   # s, 탄성 recoil(수 mm)이 풀리는 시간을 기다린 뒤에도 안 잠잠하면 abort
ZERO_PRELOAD_FORCE_OK  = 0.85  # 목표장력 도달 판정 비율
ZERO_PRELOAD_TIMEOUT   = 15.0   # s
FLEX_RELEASE_RAMP_TIME = 0.40  # s, Ext 4N -> 0N release ramp
# Calibration-only antagonist payout: negative N-equivalent current command,
# NOT a measured cable force and NOT a position/encoder target.
# v41: 굽힘 중 EXT 풀기 예산. EXT 가 풀린 양 ≤ ratio × FLEX 감긴 양 + 여유.
# ratio 는 직전 사이클 굽힘에서 실측한 |Δe|/|Δf| (첫 사이클은 기본값). 풀기가 손가락보다
# 앞서면(슬랙) 굽힘 branch 지도의 ext 좌표가 오염되므로 반드시 묶어야 한다.
CAL_EXT_PAYOUT_RATIO0   = 1.3   # 첫 사이클 기본 |Δe|/|Δf|
CAL_EXT_PAYOUT_MARGIN   = 30.0  # deg
CAL_EXT_FLEX_PAYOUT_N = 0.2  # Extension: FLEX motor gently unwinds
CAL_FLEX_TO_EXT_RELEASE_S = 2.5  # smoothly remove FLEX winding command first

# Extension return 안전 상한 / 시간
RETURN_CMD_MAX_N      = 10.0    # N-eq  EXT 램프 목표 상한 (UI T_CAL_E 가 커도 이 값에서 자름)
RETURN_TIMEOUT        = 120.0   # s
MAP_BINS       = 13       # 지도 점 개수

# ── 루프 ──
CTRL_PERIOD = 0.008
UI_PERIOD_MS = 100
ENCODER_PPR  = 1024
RAMP_UP   = 60.0
RAMP_DOWN = 12.0
MAP_DIR = "tendon_maps"
FLEX_TGT_BIAS = 0.0
MAP_BLEND_TIME = 0.30          # s

# ── 손가락 정의 ──
FINGER_IDS   = ("I", "M", "T")                       # 검지, 중지, 약지
FINGER_NAMES = {"I": "검지", "M": "중지", "T": "엄지"}

# ★★★ 배선된 손가락만 여기에. 내일 검지만 테스트면 이대로. ★★★
ENABLED_FINGERS = {
    "G1": ["I"],          # 예: ["I", "M", "T"]
    "G2": ["I"],
}

# ── 모터 정의 ──
# 검지 4개 = 실측 확정 (2026-08-04/05). 중지·약지 8개 = ★임시값(TODO)★:
# 선배 리그 실배선을 NI MAX 로 확인해 ao/ci/id/sign 수정할 것.
# sign: ESCON 지령 극성(+지령=감김이 되도록), enc_sign: 엔코더 극성(지도
# 기울기로 방향을 읽으므로 통일 불필요, 기본 +1 두고 지도 측정으로 흡수).
MOTOR_SPECS = {
    # 검지 
    "G1_I_FLEX": dict(id=3,  ao="Dev3/ao2", ci="Dev1/ctr2", sign=+1, enc_sign=+1),
    "G1_I_EXT":  dict(id=8, ao="Dev1/ao3", ci="Dev3/ctr3", sign=+1, enc_sign=+1),
    "G2_I_FLEX": dict(id=5, ao="Dev1/ao0", ci="Dev3/ctr0", sign=+1, enc_sign=+1),
    "G2_I_EXT":  dict(id=13, ao="Dev5/ao0", ci="Dev6/ctr0", sign=+1, enc_sign=+1),
    # 중지 
    "G1_M_FLEX": dict(id=4, ao="Dev3/ao3", ci="Dev1/ctr3", sign=+1, enc_sign=+1),
    "G1_M_EXT":  dict(id=1, ao="Dev3/ao0", ci="Dev1/ctr0", sign=+1, enc_sign=+1),
    "G2_M_FLEX": dict(id=6, ao="Dev1/ao1", ci="Dev3/ctr1", sign=+1, enc_sign=+1),
    "G2_M_EXT":  dict(id=12, ao="Dev5/ao1", ci="Dev5/ctr0", sign=-1, enc_sign=-1),
    # 엄지 
    "G1_T_FLEX": dict(id=2, ao="Dev3/ao1", ci="Dev1/ctr1", sign=+1, enc_sign=+1),
    "G1_T_EXT":  dict(id=7, ao="Dev1/ao2", ci="Dev3/ctr2", sign=+1, enc_sign=+1),
    "G2_T_FLEX": dict(id=9, ao="Dev4/ao0", ci="Dev4/ctr0", sign=-1, enc_sign=-1),
    "G2_T_EXT":  dict(id=11, ao="Dev6/ao0", ci="Dev6/ctr1", sign=+1, enc_sign=+1),
}
ENC_OK = {k: True for k in MOTOR_SPECS}


def _active_keys():
    """활성화된 손가락의 모터 키만."""
    keys = []
    for gs, fids in ENABLED_FINGERS.items():
        for fid in fids:
            keys += [f"{gs}_{fid}_FLEX", f"{gs}_{fid}_EXT"]
    return keys


# ═══════════════════════════════════════════════════════ PCHIP

def _pchip_prepare(xs, ys):
    n = len(xs)
    if n < 2:
        return [0.0] * n
    h = [xs[i+1] - xs[i] for i in range(n-1)]
    d = [(ys[i+1] - ys[i]) / h[i] if h[i] != 0 else 0.0 for i in range(n-1)]
    m = [0.0] * n
    m[0], m[-1] = d[0], d[-1]
    for i in range(1, n-1):
        if d[i-1] * d[i] <= 0:
            m[i] = 0.0
        else:
            w1_ = 2*h[i] + h[i-1]
            w2_ = h[i] + 2*h[i-1]
            m[i] = (w1_ + w2_) / (w1_/d[i-1] + w2_/d[i])
    return m


def _pchip_eval(xs, ys, ms, x):
    n = len(xs)
    if n == 0:
        return 0.0
    if n == 1 or x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    lo, hi = 0, n-1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if xs[mid] <= x:
            lo = mid
        else:
            hi = mid
    h = xs[hi] - xs[lo]
    t = (x - xs[lo]) / h
    h00 = (1 + 2*t) * (1-t)**2
    h10 = t * (1-t)**2
    h01 = t*t * (3 - 2*t)
    h11 = t*t * (t - 1)
    return h00*ys[lo] + h10*h*ms[lo] + h01*ys[hi] + h11*h*ms[hi]


class _Curve:
    def __init__(self, xs, ys):
        if len(xs) >= 2 and xs[-1] < xs[0]:
            xs = [-v for v in xs]
            self.flip = True
        else:
            self.flip = False
        pairs = sorted(zip(xs, ys))
        self.xs = [p[0] for p in pairs]
        self.ys = [p[1] for p in pairs]
        self.ms = _pchip_prepare(self.xs, self.ys)

    def __call__(self, x):
        return _pchip_eval(self.xs, self.ys, self.ms,
                           -x if self.flip else x)


# ═══════════════════════════════════════════════════════ 지도

class TendonMap:

    def __init__(self):
        self.name = ""
        self.raw = []                 # legacy/general 중앙선
        self.raw_flexion = []         # 1/2/3회 flexion 평균
        self.raw_extension = []       # 1/2/3회 extension 평균
        self.meta = {}
        self.ready = False
        self._branches = {
            "flexion": None,
            "extension": None,
            "mid": None,
        }
        self._f0 = self._e0 = 0.0
        self._sf = self._se = 0.0

    @staticmethod
    def _make_branch(raw):
        raw = sorted([tuple(p) for p in raw], key=lambda p: p[0])
        if len(raw) < 2:
            return None

        f0, e0 = raw[0][1], raw[0][2]
        fr = [p[1] - f0 for p in raw]
        er = [p[2] - e0 for p in raw]
        sf = fr[-1]
        se = er[-1]

        if abs(sf) < 1.0 and abs(se) < 1.0:
            return None

        return {
            "raw": raw,
            "f0": f0,
            "e0": e0,
            "sf": sf,
            "se": se,
            "e2f": _Curve(er, fr),
            "f2e": _Curve(fr, er),
        }

    @staticmethod
    def _midline(raw_f, raw_e):
        df = {int(p[0]): p for p in raw_f}
        de = {int(p[0]): p for p in raw_e}
        keys = sorted(set(df) | set(de))
        out = []

        for new_i, k in enumerate(keys):
            pf = df.get(k)
            pe = de.get(k)

            if pf is not None and pe is not None:
                fd = 0.5 * (pf[1] + pe[1])
                ed = 0.5 * (pf[2] + pe[2])
            elif pf is not None:
                fd, ed = pf[1], pf[2]
            else:
                fd, ed = pe[1], pe[2]

            out.append((new_i, fd, ed))

        return out

    def _select_branch(self, branch=None):
        if branch in ("flexion", "flexing", +1):
            key = "flexion"
        elif branch in ("extension", "extending", -1):
            key = "extension"
        else:
            key = "mid"

        br = self._branches.get(key)
        if br is None:
            br = self._branches.get("mid")
        if br is None:
            br = self._branches.get("flexion") or self._branches.get("extension")
        return br

    def record(self, level, flex_deg, ext_deg):
        self.raw = [p for p in self.raw if p[0] != level]
        self.raw.append((level, flex_deg, ext_deg))
        self.raw.sort(key=lambda p: p[0])
        self.raw_flexion = list(self.raw)
        self.raw_extension = list(self.raw)
        self._rebuild()

    def set_raw(self, raw, meta=None):
        raw = sorted([tuple(p) for p in raw], key=lambda p: p[0])
        self.raw = list(raw)
        self.raw_flexion = list(raw)
        self.raw_extension = list(raw)
        if meta:
            self.meta = dict(meta)
        self._rebuild()

    def set_branches(self, raw_flexion, raw_extension, meta=None):
        self.raw_flexion = sorted(
            [tuple(p) for p in raw_flexion],
            key=lambda p: p[0]
        )
        self.raw_extension = sorted(
            [tuple(p) for p in raw_extension],
            key=lambda p: p[0]
        )
        self.raw = self._midline(
            self.raw_flexion,
            self.raw_extension
        )
        if meta:
            self.meta = dict(meta)
        self._rebuild()

    def clear(self):
        self.name = ""
        self.raw = []
        self.raw_flexion = []
        self.raw_extension = []
        self.meta = {}
        self.ready = False
        self._branches = {
            "flexion": None,
            "extension": None,
            "mid": None,
        }

    def _rebuild(self):
        if not self.raw_flexion and self.raw:
            self.raw_flexion = list(self.raw)
        if not self.raw_extension and self.raw:
            self.raw_extension = list(self.raw)

        if not self.raw and (self.raw_flexion or self.raw_extension):
            self.raw = self._midline(
                self.raw_flexion,
                self.raw_extension
            )

        self._branches["flexion"] = self._make_branch(self.raw_flexion)
        self._branches["extension"] = self._make_branch(self.raw_extension)
        self._branches["mid"] = self._make_branch(self.raw)

        self.ready = (
            self._branches["flexion"] is not None
            and self._branches["extension"] is not None
        )

        base = self._select_branch("mid")
        if base is not None:
            self._f0 = base["f0"]
            self._e0 = base["e0"]
            self._sf = base["sf"]
            self._se = base["se"]

    def flex_target_from_ext(self, ext_abs, branch=None):
        br = self._select_branch(branch)
        if br is None:
            return 0.0
        return br["e2f"](ext_abs - br["e0"]) + br["f0"]

    def ext_target_from_flex(self, flex_abs, branch=None):
        br = self._select_branch(branch)
        if br is None:
            return 0.0
        return br["f2e"](flex_abs - br["f0"]) + br["e0"]

    def targets_from_curl(self, curl, over=0.0, branch=None):
        br = self._select_branch(branch)
        if br is None:
            return 0.0, 0.0

        c = max(-over, min(1.0 + over, curl))
        tf = br["f0"] + c * br["sf"]
        te = br["f2e"](tf - br["f0"]) + br["e0"]
        return tf, te

    def curl_from_flex(self, flex_abs, branch=None):
        br = self._select_branch(branch)
        if br is None or abs(br["sf"]) <= 1.0:
            return 0.0
        return (flex_abs - br["f0"]) / br["sf"]

    def curl_from_ext(self, ext_abs, branch=None):
        br = self._select_branch(branch)
        if br is None or abs(br["se"]) <= 1.0:
            return 0.0
        return (ext_abs - br["e0"]) / br["se"]

    def wind_dirs(self):
        br = self._select_branch("mid")
        if br is None:
            return 1.0, -1.0
        wf = 1.0 if br["sf"] >= 0 else -1.0
        we = -1.0 if br["se"] >= 0 else 1.0
        return wf, we

    def save(self, name):
        os.makedirs(MAP_DIR, exist_ok=True)
        self.name = name
        path = os.path.join(MAP_DIR, f"{name}.json")

        with open(path, "w", encoding="utf-8") as fp:
            json.dump(
                {
                    "name": name,
                    "map_type": "direction_separated",
                    "raw_flexion": self.raw_flexion,
                    "raw_extension": self.raw_extension,
                    "raw": self.raw,
                    "meta": self.meta,
                },
                fp,
                indent=2,
                ensure_ascii=False
            )
        return path

    def load(self, path):
        with open(path, "r", encoding="utf-8") as fp:
            d = json.load(fp)

        return self.load_data(d, os.path.splitext(os.path.basename(path))[0])

    def load_data(self, d, default_name="map"):
        if "raw_flexion" in d and "raw_extension" in d:
            self.raw_flexion = [
                tuple(p) for p in d["raw_flexion"]
            ]
            self.raw_extension = [
                tuple(p) for p in d["raw_extension"]
            ]
            if "raw" in d:
                self.raw = [tuple(p) for p in d["raw"]]
            else:
                self.raw = self._midline(
                    self.raw_flexion,
                    self.raw_extension
                )

        elif "raw" in d:
            self.raw = [tuple(p) for p in d["raw"]]
            self.raw_flexion = list(self.raw)
            self.raw_extension = list(self.raw)

        elif "levels" in d:
            self.raw = [
                (i, p[1], p[2])
                for i, p in enumerate(d["levels"])
            ]
            self.raw_flexion = list(self.raw)
            self.raw_extension = list(self.raw)

        else:
            raise ValueError("지원하지 않는 tendon map 형식")

        self.raw.sort(key=lambda p: p[0])
        self.raw_flexion.sort(key=lambda p: p[0])
        self.raw_extension.sort(key=lambda p: p[0])

        self.meta = d.get("meta", {})
        self.name = d.get(
            "name",
            default_name
        )
        self._rebuild()
        return self.ready

    def curve_points(self, n=60, branch=None):
        br = self._select_branch(branch)
        if br is None:
            return []

        er = [p[2] - br["e0"] for p in br["raw"]]
        lo, hi = min(er), max(er)
        pts = []

        for i in range(n):
            e = lo + (hi - lo) * i / max(1, n - 1)
            pts.append((e, br["e2f"](e)))

        return pts

    def summary(self):
        if not self.ready:
            return "지도 없음"

        s = (
            f"[{self.name}] 방향분리 map | "
            f"Flexion {len(self.raw_flexion)}점 / "
            f"Extension {len(self.raw_extension)}점 | "
            f"ROM {abs(self._sf)*DEG_TO_MM:.1f}mm"
        )

        if "hyst_mm" in self.meta:
            s += f" | 히스테리시스 {self.meta['hyst_mm']:.2f}mm"

        return s


# ═══════════════════════════════════════════════════════ 하드웨어

class AOBank:

    def __init__(self, specs, keys):
        self.slot, self.vals, self.tasks = {}, {}, {}
        groups = {}
        for key in keys:
            sp = specs[key]
            dev = sp["ao"].split("/")[0]
            groups.setdefault(dev, []).append((key, sp["ao"]))
        for dev, items in groups.items():
            items.sort(key=lambda x: x[1])
            self.vals[dev] = [0.0] * len(items)
            for i, (key, ch) in enumerate(items):
                self.slot[key] = (dev, i)
            if DAQ_AVAILABLE:
                try:
                    t = nidaqmx.Task()
                    for _, ch in items:
                        t.ao_channels.add_ao_voltage_chan(ch, min_val=AO_MIN_V,
                                                          max_val=AO_MAX_V)
                    self.tasks[dev] = t
                except Exception as e:
                    print(f"[오류] {dev} AO 열기 실패: {e}")
                    print(f"       NI MAX 에서 {dev} 연결/채널을 확인하거나")
                    print(f"       MOTOR_SPECS / ENABLED_FINGERS 를 수정하세요")
                    raise
        self.flush()

    def set(self, key, volts):
        dev, i = self.slot[key]
        self.vals[dev][i] = volts

    def flush(self):
        for dev, t in self.tasks.items():
            v = self.vals[dev]
            try:
                t.write(v if len(v) > 1 else v[0])
            except Exception as exc:
                raise RuntimeError(f"DAQ analog output failed on {dev}: {exc}") from exc

    def zero(self):
        for dev in self.vals:
            self.vals[dev] = [0.0] * len(self.vals[dev])
        self.flush()

    def close(self):
        self.zero()
        for t in self.tasks.values():
            try:
                t.close()
            except Exception:
                pass


class MotorChannel:

    def __init__(self, key, bank):
        sp = MOTOR_SPECS[key]
        self.key, self.bank = key, bank
        self.sign, self.enc_sign = sp["sign"], sp["enc_sign"]
        self.enc_ok = ENC_OK.get(key, True)
        self.target_N = 0.0
        self.applied_N = 0.0
        self.deg_raw = self.offset = self.deg = self.vel = 0.0
        self._prev = 0.0
        self.ci = None
        if DAQ_AVAILABLE and self.enc_ok:
            try:
                self.ci = nidaqmx.Task()
                self.ci.ci_channels.add_ci_ang_encoder_chan(
                    sp["ci"], units=AngleUnits.DEGREES,
                    pulses_per_rev=ENCODER_PPR,
                    decoding_type=EncoderType.X_4,
                    zidx_enable=False,
                    zidx_phase=EncoderZIndexPhase.AHIGH_BHIGH)
                self.ci.start()
            except Exception as e:
                print(f"[경고] {key} 엔코더: {e}")
                self.ci = None
                self.enc_ok = False

    def read(self, dt):
        if self.ci:
            try:
                self.deg_raw = float(self.ci.read())
            except Exception:
                pass
        self.deg = (self.deg_raw - self.offset) * self.enc_sign
        if dt > 0:
            self.vel += 0.4 * ((self.deg - self._prev) / dt - self.vel)
        self._prev = self.deg

    def emit(self, dt):
        d = self.target_N - self.applied_N
        lim = (RAMP_UP if d > 0 else RAMP_DOWN) * dt
        self.applied_N = self.target_N if abs(d) <= lim else \
                         self.applied_N + (lim if d > 0 else -lim)
        n = self.applied_N
        sp = abs(self.vel)
        if sp > VEL_CAP:
            n *= max(0.0, 2.0 - sp / VEL_CAP)
        amps = max(-I_MAX, min(I_MAX, n * SPOOL_R / KT))
        self.bank.set(self.key,
                      max(AO_MIN_V, min(AO_MAX_V, self.sign * amps * V_PER_AMP)))

    def kill(self):
        self.target_N = 0.0
        self.applied_N = 0.0
        self.bank.set(self.key, 0.0)

    @property
    def mm(self):
        return self.deg * DEG_TO_MM

    def zero(self):
        self.offset = self.deg_raw
        self.deg = self._prev = self.vel = 0.0

    def close(self):
        if self.ci:
            try:
                self.ci.close()
            except Exception:
                pass


# ═══════════════════════════════════════════════════════ 손가락 (v6 GloveIO)

class FingerUnit:
    """한 손가락 = 굴곡·신전 모터쌍 + 지도 + 역할/슬랙 상태기계."""

    def __init__(self, glove_name, glove_short, fid, bank):
        self.fid = fid
        self.fname = FINGER_NAMES[fid]
        self.glove_short = glove_short
        self.name = f"{glove_name}-{self.fname}"
        self.key = f"{glove_short}_{fid}"
        self.flex = MotorChannel(f"{self.key}_FLEX", bank)
        self.ext = MotorChannel(f"{self.key}_EXT", bank)
        self.map = TendonMap()
        self.curl = self.curl_vel = 0.0
        self.source = None
        self.driven = None
        self.d_flex = self.d_ext = 0.0
        self.tgt_flex = self.tgt_ext = None
        self.pos_i = 0.0
        self._tgt_prev = None
        self.soft = 0.0             # auto-tensioning soft start
        self.coupling_soft = 0.0    # independent coupling soft start
        self.auto_enabled = False
        self.coupling_enabled = False
        self.single = not (self.flex.enc_ok and self.ext.enc_ok)
        self.motion = "idle"
        self.cpl_f = 0.0
        self.cpl_e = 0.0
        self.payout_f = 0.0
        self.payout_e = 0.0
        self.payout_lead = 0.0
        self._dir_cand = None
        self._dir_anchor = 0.0
        self._dir_elapsed = 0.0
        self._curl_blend_t = 0.0
        self._curl_blend_from = 0.0
        self._curl_blend_active = False
        self.curl_raw_flex = self.curl_raw_ext = self.curl_target = 0.0
        self._last_motion = None
        self._pi_on = False
        self._err_prev = None      # D항: 이전 오차 (역할교대/idle 시 리셋 = kick 방지)
        self._derr = 0.0
        self.map_aligned = False
        self._map_blend_t = 0.0
        self._map_blend_start = 0.0
        self._map_blend_active = False
        self._map_anchor_source = None
        self._map_anchor_driven = None

    @property
    def motors(self):
        return (self.flex, self.ext)

    @property
    def enabled(self):
        return self.auto_enabled or self.coupling_enabled

    def reset_auto(self):
        self.pos_i = self.soft = 0.0
        self._pi_on = False
        self._err_prev = self._tgt_prev = None
        self._derr = 0.0
        self.d_flex = self.d_ext = 0.0
        self.tgt_flex = self.tgt_ext = None
        self._map_blend_t = 0.0
        self._map_blend_active = self.driven is not None
        self._map_blend_start = (
            self.flex.deg if self.driven == "flex" else self.ext.deg
        )

    def set_control(self, kind, on):
        """Called with the controller lock held; keep encoder/motion tracking."""
        if kind == "auto":
            if self.auto_enabled != on:
                self.reset_auto()
            self.auto_enabled = on
        elif kind == "coupling":
            if self.coupling_enabled != on:
                self.coupling_soft = 0.0
                self.cpl_f = self.cpl_e = 0.0
            self.coupling_enabled = on
        else:
            raise ValueError(f"Unknown control: {kind}")
        if not self.enabled:
            for motor in self.motors:
                motor.kill()

    def _swap(self, new_source):
        if self.source != new_source:
            self._curl_blend_from = self.curl
            self._curl_blend_t = 0.0
            self._curl_blend_active = True
            self.source = new_source
            self.driven = "ext" if new_source == "flex" else "flex"
            self.pos_i = 0.0
            self._pi_on = False
            self._err_prev = None
            self._derr = 0.0

            self._map_blend_t = 0.0
            self._map_blend_active = True
            self._map_blend_start = (
                self.flex.deg if self.driven == "flex"
                else self.ext.deg
            )
            self._tgt_prev = self._map_blend_start

            self._map_anchor_source = None
            self._map_anchor_driven = None

    def read(self, dt, p):
        self.flex.read(dt)
        self.ext.read(dt)

        if not self.map.ready:
            self.curl = 0.0
            return

        if self.single:
            if self.flex.enc_ok:
                target = self.map.curl_from_flex(self.flex.deg)
                self.source = "flex 단독"
            elif self.ext.enc_ok:
                target = self.map.curl_from_ext(self.ext.deg)
                self.source = "ext 단독"
            else:
                target = self.curl
                self.source = "센서없음"
            prev = self.curl
            self.curl += 0.35 * (target - self.curl)
            if dt > 0:
                self.curl_vel += 0.4 * ((self.curl - prev) / dt - self.curl_vel)
            return

        wf, we = self.map.wind_dirs()
        unwind_f = -wf * self.flex.vel
        unwind_e = -we * self.ext.vel

        if unwind_e > MOVE_TH and unwind_e > unwind_f:
            cand, mon = "flexing", self.ext.deg
        elif unwind_f > MOVE_TH and unwind_f > unwind_e:
            cand, mon = "extending", self.flex.deg
        else:
            cand, mon = "idle", 0.0

        if cand != self.motion:
            if cand != self._dir_cand:
                self._dir_cand = cand
                self._dir_anchor = mon
                self._dir_elapsed = 0.0
            else:
                self._dir_elapsed += dt
                need = DIR_DIST if cand not in ("idle", self._last_motion) else 0.0
                if self._dir_elapsed >= DIR_CONFIRM_S and (cand == "idle" or abs(mon - self._dir_anchor) >= need):
                    self.motion = cand
                    if cand == "flexing":
                        self._swap("ext")
                    elif cand == "extending":
                        self._swap("flex")
                    if cand != "idle":
                        self._last_motion = cand
                    self._dir_cand = None
        else:
            self._dir_cand = None

        self.curl_raw_flex = self.map.curl_from_flex(self.flex.deg, "extension")
        self.curl_raw_ext = self.map.curl_from_ext(self.ext.deg, "flexion")
        if self.source == "flex":
            branch = "extension" if self.motion == "extending" else None
            target = self.map.curl_from_flex(
                self.flex.deg,
                branch=branch
            )
        elif self.source == "ext":
            branch = "flexion" if self.motion == "flexing" else None
            target = self.map.curl_from_ext(
                self.ext.deg,
                branch=branch
            )
        else:
            target = self.curl
        if self._curl_blend_active:
            self._curl_blend_t += dt
            a = min(1.0, self._curl_blend_t/CURL_BLEND_TIME)
            target = (1-a)*self._curl_blend_from + a*target
            if a >= 1: self._curl_blend_active = False
        self.curl_target = target
        prev = self.curl
        self.curl += 0.35 * (target - self.curl)
        self.curl = max(-0.2, min(1.2, self.curl))
        if dt > 0:
            self.curl_vel += 0.4 * ((self.curl - prev) / dt - self.curl_vel)

    # ── 결합 (독립 게인) ──

    def couple(self, partner_curl, partner_vel, kc, bc, over):
        """상대 curl → 내 지도의 목표 → 오차 비례 힘.
        kc, bc  : 이 장갑(=이 힘을 받는 사람) 전용 게인 [N/°, N·s/°]
        over    : 끝범위 허용 폭 (targets_from_curl 참조)"""
        self.cpl_f = self.cpl_e = 0.0
        if self.single or not (
            self.map.ready and self.map_aligned and self.coupling_enabled
        ):
            return 0.0, 0.0
        if kc <= 0.0 and bc <= 0.0:
            return 0.0, 0.0

        wf, we = self.map.wind_dirs()
        tf, te = self.map.targets_from_curl(partner_curl, over)

        e_f = wf * (tf - self.flex.deg)
        e_e = we * (te - self.ext.deg)
        de_f = partner_vel * abs(self.map._sf) - wf * self.flex.vel
        de_e = -partner_vel * abs(self.map._se) - we * self.ext.vel

        f_f = self.coupling_soft * (kc * e_f + bc * de_f)
        f_e = self.coupling_soft * (kc * e_e + bc * de_e)
        if getattr(self, "coupling_mode", "force") == "position_velocity":
            # Experimental symmetric position/velocity outer servo with current output.
            # Choose one pulling tendon based on the relative curl error.
            delta = partner_curl - self.curl
            # Position -> desired velocity -> current-equivalent pulling force.
            # KC acts as an assistance scale (not calibrated physical stiffness).
            # BC adds velocity-error damping in N*s/deg.
            assist = min(1.0, max(0.0, kc / 0.02))
            if abs(delta) < 0.015 or assist == 0.0:
                f_f = f_e = 0.0
            elif delta > 0.0:
                pos_err = max(0.0, e_f)
                v_ref = min(120.0, 1.5 * pos_err)
                v_actual = wf * self.flex.vel
                f_f = self.coupling_soft * assist * max(0.0,
                      0.02 * (v_ref - v_actual) + bc * (v_ref - v_actual))
                f_e = 0.0
            else:
                pos_err = max(0.0, e_e)
                v_ref = min(120.0, 1.5 * pos_err)
                v_actual = we * self.ext.vel
                f_e = self.coupling_soft * assist * max(0.0,
                      0.02 * (v_ref - v_actual) + bc * (v_ref - v_actual))
                f_f = 0.0
        self.cpl_f = max(0.0, min(T_ASSIST_MAX, f_f))
        self.cpl_e = max(0.0, min(T_ASSIST_MAX, f_e))
        return self.cpl_f, self.cpl_e

    def apply(self, cpl_forces, p, dt):
        if not self.enabled or not (self.map.ready and self.map_aligned):
            self.flex.target_N = self.ext.target_N = 0.0
            self.d_flex = self.d_ext = 0.0
            self.tgt_flex = self.tgt_ext = None
            self.pos_i = 0.0
            self.soft = 0.0
            self.coupling_soft = 0.0
            self._tgt_prev = None
            self._err_prev = None
            self._derr = 0.0
            self.cpl_f = self.cpl_e = 0.0
            return

        self.soft = min(1.0, self.soft + 0.8 * dt) if self.auto_enabled else 0.0
        self.coupling_soft = (
            min(1.0, self.coupling_soft + 0.8 * dt)
            if self.coupling_enabled else 0.0
        )

        a_flex, a_ext = cpl_forces if self.coupling_enabled else (0.0, 0.0)
        tb = p["T_BASE"] if self.auto_enabled else 0.0
        self.d_flex = self.d_ext = 0.0
        self.tgt_flex = self.tgt_ext = None
        pos_out = 0.0

        if self.single:
            ff = FRIC_FALLBACK if self.auto_enabled and self.curl_vel > 0.02 else 0.0
            fe = FRIC_FALLBACK if self.auto_enabled and self.curl_vel < -0.02 else 0.0
            self.flex.target_N = min(T_MAX, tb + (ff if self.flex.enc_ok else 0.0) + a_flex)
            self.ext.target_N = min(T_MAX, tb + (fe if not self.ext.enc_ok else 0.0) + a_ext)
            return

        if self.auto_enabled and self.driven is not None and self.motion != "idle":
            wf, we = self.map.wind_dirs()

            if self.driven == "flex":
                map_target = self.map.flex_target_from_ext(
                    self.ext.deg,
                    branch="flexion"
                )
            else:
                map_target = self.map.ext_target_from_flex(
                    self.flex.deg,
                    branch="extension"
                )

            if self._map_blend_active:
                self._map_blend_t += dt
                alpha = min(1.0, self._map_blend_t / max(1e-6, MAP_BLEND_TIME))
                raw_tgt = (
                    (1.0 - alpha) * self._map_blend_start
                    + alpha * map_target
                )
                if alpha >= 1.0:
                    self._map_blend_active = False
            else:
                raw_tgt = map_target

            if self._tgt_prev is None:
                self._tgt_prev = (
                    self.flex.deg if self.driven == "flex"
                    else self.ext.deg
                )

            lim = TGT_RATE * dt
            raw_tgt = max(
                self._tgt_prev - lim,
                min(self._tgt_prev + lim, raw_tgt)
            )
            self._tgt_prev = raw_tgt

            if self.driven == "flex":
                self.tgt_flex = raw_tgt
                err = wf * (raw_tgt - self.flex.deg)
                self.d_flex = err
            else:
                self.tgt_ext = raw_tgt
                err = we * (raw_tgt - self.ext.deg)
                self.d_ext = err

            if self.glove_short == "G1":
                kp, ki, kd = p["KP1"], p["KI1"], p["KD1"]
            else:
                kp, ki, kd = p["KP2"], p["KI2"], p["KD2"]

            # D항: 오차 변화율 (저역 필터, 역할교대/idle 후 첫 루프는 0 → kick 방지)
            if self._err_prev is None:
                self._derr = 0.0
            else:
                raw_d = (err - self._err_prev) / dt
                self._derr += 0.5 * (raw_d - self._derr)
            self._err_prev = err

            if err > p["ERR_DB"]:
                self._pi_on = True
            elif err < p["ERR_DB"] * 0.5:
                self._pi_on = False

            if not self._pi_on:
                self.pos_i *= I_DECAY
                if self.pos_i < 1e-4:
                    self.pos_i = 0.0
            else:
                self.pos_i = max(
                    0.0,
                    min(
                        I_CLAMP,
                        self.pos_i + ki * max(0.0, err) * dt
                    )
                )
                pos_out = max(
                    0.0,
                    min(
                        SLACK_SAT,
                        kp * max(0.0, err)
                        + self.pos_i
                        + kd * self._derr
                    )
                )
            pos_out *= self.soft

        if self.motion == "idle":
            self.pos_i *= I_DECAY
            self._tgt_prev = None
            self._err_prev = None
            self._derr = 0.0

        # v40: extension 중 FLEX(source) 풀기 — 캘리브레이션의 Flex 풀기와 같은 역할.
        # 손가락이 flex 스풀/라우팅 마찰을 끌지 않도록 작은 음(풀림) 지령을 주되,
        # 지도상 기대 flex 위치보다 너무 앞서면(슬랙) 감쇠/중단한다 (self-chasing 방지).
        payout_f = payout_e = 0.0
        self.payout_lead = 0.0
        span = max(1e-6, RUN_PAYOUT_LEAD_OFF_DEG - RUN_PAYOUT_LEAD_FULL_DEG)
        wf, we = self.map.wind_dirs()
        if (self.motion == "extending" and self.source == "flex"
                and p["PO_FLEX"] > 0.0):
            flex_expected = self.map.flex_target_from_ext(
                self.ext.deg, branch="extension"
            )
            # >0 : 지도가 말하는 것보다 flex 가 더 풀려 있음 (슬랙 쪽으로 앞섬)
            slack_lead = -wf * (self.flex.deg - flex_expected)
            self.payout_lead = slack_lead
            scale = max(0.0, min(1.0, (RUN_PAYOUT_LEAD_OFF_DEG - slack_lead) / span))
            payout_f = p["PO_FLEX"] * scale
        elif (self.motion == "flexing" and self.source == "ext"
                and p["PO_EXT"] > 0.0):
            # v41: 대칭 — 굽힘 중 EXT(source) 풀기, 굽힘 branch 지도에 묶음
            ext_expected = self.map.ext_target_from_flex(
                self.flex.deg, branch="flexion"
            )
            slack_lead = -we * (self.ext.deg - ext_expected)
            self.payout_lead = slack_lead
            scale = max(0.0, min(1.0, (RUN_PAYOUT_LEAD_OFF_DEG - slack_lead) / span))
            payout_e = p["PO_EXT"] * scale
        self.payout_f = payout_f
        self.payout_e = payout_e

        self.flex.target_N = min(
            T_MAX,
            tb + (pos_out if self.driven == "flex" else 0.0) + a_flex - payout_f
        )
        self.ext.target_N = min(
            T_MAX,
            tb + (pos_out if self.driven == "ext" else 0.0) + a_ext - payout_e
        )

    def set_manual(self, fn, en):
        self.flex.target_N = max(-T_MAX, min(T_MAX, fn))
        self.ext.target_N = max(-T_MAX, min(T_MAX, en))

    def kill(self):
        # 모터 출력을 0으로 만들고 제어 상태를 완전히 초기화
        self.auto_enabled = self.coupling_enabled = False
        self.coupling_soft = 0.0
        self.pos_i = self.soft = 0.0
        self._tgt_prev = None
        self._err_prev = None
        self._derr = 0.0
        self._pi_on = False

        # E-stop 해제 직후 stale Δθ 때문에 즉시 재진입하는 문제 방지
        self.d_flex = self.d_ext = 0.0
        self.tgt_flex = self.tgt_ext = None
        self.cpl_f = self.cpl_e = 0.0

        self.motion = "idle"
        self._curl_blend_active = False
        self._dir_elapsed = 0.0
        self._dir_cand = None
        self._last_motion = None
        self.source = None if not self.single else self.source
        self.driven = None

        self._map_anchor_source = None
        self._map_anchor_driven = None
        self._map_blend_t = 0.0
        self._map_blend_start = 0.0
        self._map_blend_active = False

        for m in self.motors:
            m.kill()

    def zero(self):
        for m in self.motors:
            m.zero()
        self.curl = self.curl_vel = 0.0
        self._curl_blend_active = False
        self._dir_elapsed = 0.0
        self.pos_i = 0.0
        self.source = None if not self.single else self.source
        self.driven = None
        self._tgt_prev = None
        self._map_blend_t = 0.0
        self._map_blend_start = 0.0
        self._map_blend_active = False

    def close(self):
        for m in self.motors:
            m.close()


# ═══════════════════════════════════════════════════════ 장갑 (손가락 묶음)

class Glove:

    def __init__(self, name, short, bank):
        self.name, self.short = name, short
        self.fingers = {fid: FingerUnit(name, short, fid, bank)
                        for fid in ENABLED_FINGERS.get(short, [])}

    def read(self, dt, p):
        for f in self.fingers.values():
            f.read(dt, p)

    def kill(self):
        for f in self.fingers.values():
            f.kill()

    def zero(self):
        for f in self.fingers.values():
            f.zero()

    def close(self):
        for f in self.fingers.values():
            f.close()

    @property
    def all_motors(self):
        for f in self.fingers.values():
            yield from f.motors


# ═══════════════════════════════════════════════════════ Sweep 캘리브레이션

class SweepCal:
    """한 손가락 3왕복 자동 캘리브레이션 상태기계.
    control 스레드에서 step() 호출. 완료 시 지도 생성+자동 저장."""

    def __init__(self):
        self.finger = None
        self.state = "off"       # off|preten|pull_f|pull_e|done|abort
        self.msg = ""
        self.cycle = 0
        self._t_state = 0.0
        self._t_stall = 0.0
        self._anchor = 0.0
        self._last_samp = None
        self.samples = []
        self.map_start_idx = 0        # (dir, flex_abs, ext_abs)  dir: +1 굽힘 / -1 펴기
        self.ends_flex = []      # 굽힘 끝 flex 위치들
        self.ends_ext = []       # 펴기 끝 flex 위치들
        self.takeup_log = []
        self.result = None       # (rom_mm, hyst_mm, n_pts) 완료 시
        self._release_next = ""
        self._release_initial = (0.,0.)
        self._flex_to_ext_start_N = 0.0
        self._pull_f_start_flex = 0.0
        self._pull_f_start_ext = 0.0
        self._pull_f_end_ext = 0.0
        self._ratio_ef = CAL_EXT_PAYOUT_RATIO0

    def begin(self, finger):
        self.finger = finger
        self.state = "preten"
        self.msg = "초기 Pretension..."
        self.cycle = 0
        self._t_state = 0.0
        self.samples = []
        self.map_start_idx = 0
        self.ends_flex = []
        self.ends_ext = []
        self.takeup_log = []     # 사이클별 (applied_N, past_start_deg) — 탄성 take-up 기록
        self.result = None
        finger.map.clear()

    def abort(self, msg="중단"):
        self.state = "abort"
        self.msg = msg
        if self.finger:
            self.finger.set_manual(0.0, 0.0)

    def _enter(self, state):
        self.state = state
        self._t_state = 0.0
        self._t_stall = 0.0
        f = self.finger

        # 실제 active tendon encoder를 이동량 기준으로 사용
        if state == "pull_e":
            self._anchor = f.ext.deg
        else:
            self._anchor = f.flex.deg
        if state == "pull_f":
            self._pull_f_start_flex = f.flex.deg
            self._pull_f_start_ext = f.ext.deg

        if state == "flex_to_ext_release":
            self._flex_to_ext_start_N = max(0.0, f.flex.applied_N)

        if state == "position_return":
            self._return_ext_start_deg = f.ext.deg
            self._return_phase = "passive"     # passive → force
            self._return_phase_t = 0.0
            self._return_passive_still_t = 0.0
            self._return_obs_t = 0.0
            self._return_ref_e = f.ext.deg
            # 굽힘 때 FLEX 가 감긴 방향(부호). return 중 이 방향의 반대로 되풀린다.
            self._return_wind_sign = (
                1.0 if (f.flex.deg - self._pull_f_start_flex) >= 0.0 else -1.0
            )
            self._bk_active = False
            self._bk_t = 0.0
            self._bk_level = 0.0
            self._bk_tries = 0
            self._bk_total = 0
            self._mid_hold_t = 0.0

        self._last_samp = None

    def _release_reference(self, next_state):
        self._release_next = next_state
        self._release_initial = (self.finger.flex.applied_N, self.finger.ext.applied_N)
        self._enter("reference_release")

    def _sample(self, direction):
        f = self.finger

        # 굽힘은 flex encoder, 펴기는 extension encoder를 sampling 기준으로 사용
        monitor_deg = f.flex.deg if direction == +1 else f.ext.deg

        if self._last_samp is None or \
           abs(monitor_deg - self._last_samp) >= SAMPLE_DEG:
            self.samples.append((direction, f.flex.deg, f.ext.deg))
            self._last_samp = monitor_deg

    def step(self, dt, p):
        if self.state in ("off", "done", "abort") or self.finger is None:
            return

        f = self.finger
        self._t_state += dt

        t_cal_f = p["T_CAL_F"]
        return_user_limit = p["T_CAL_E"]
        return_cmd_limit = min(
            max(0.5, return_user_limit),
            RETURN_CMD_MAX_N
        )

        def reference_preload_ok():
            return (
                f.flex.applied_N >= ZERO_PRELOAD_FORCE_OK * ZERO_PRELOAD_N
                and
                f.ext.applied_N >= ZERO_PRELOAD_FORCE_OK * ZERO_PRELOAD_N
            )

        if self.state == "reference_release":
            a = min(1., self._t_state/ZERO_RELEASE_TIME)
            f.set_manual(self._release_initial[0]*(1-a), self._release_initial[1]*(1-a))
            self.msg = f"Preload release {a*100:.0f}%"
            if a >= 1 and max(abs(f.flex.applied_N),abs(f.ext.applied_N)) < .04:
                self._enter("reference_settle")
            elif self._t_state > ZERO_RELEASE_TIME+4:
                self.abort("Preload release timeout")
            return
        if self.state == "reference_settle":
            f.set_manual(0.,0.)
            # v39: 장력을 풀면 케이블/글러브 탄성 recoil 로 스풀이 한동안 돈다.
            # 한 시점의 속도를 보고 abort 하지 않고, 두 엔코더가 ZERO_SETTLE_TIME 동안
            # 연속으로 잠잠해질 때까지 기다린다 (최대 ZERO_SETTLE_TIMEOUT).
            vmax = max(abs(f.flex.vel), abs(f.ext.vel))
            if vmax <= ZERO_SETTLE_VEL_DPS:
                self._t_stall += dt
            else:
                self._t_stall = 0.0
            self.msg = (
                f"Released reference settling | vel={vmax:.0f}dps "
                f"calm={self._t_stall:.2f}/{ZERO_SETTLE_TIME:.2f}s"
            )
            if self._t_stall >= ZERO_SETTLE_TIME:
                if self._release_next == "official":
                    sf,se = f.flex.deg,f.ext.deg
                    self.samples = [(d,fd-sf,ed-se) for d,fd,ed in self.samples]
                    self.map_start_idx = len(self.samples)
                    f.zero()
                    self._enter("release_to_flex")
                elif self._release_next == "cycle":
                    self.samples.append((-1,f.flex.deg,f.ext.deg))
                    self.ends_ext.append(f.flex.deg)
                    self.cycle += 1
                    if self.cycle >= TOTAL_SWEEP_CYCLES: self._finish(p)
                    else: self._enter("release_to_flex")
                else:
                    f.zero()
                    self._enter("release_to_flex")
            elif self._t_state > ZERO_SETTLE_TIMEOUT:
                self.abort(
                    f"Released reference unstable: vel={vmax:.0f}dps after "
                    f"{ZERO_SETTLE_TIMEOUT:.0f}s"
                )
            return
        # A. INITIAL REFERENCE PRELOAD
        if self.state == "preten":
            f.set_manual(ZERO_PRELOAD_N, ZERO_PRELOAD_N)
            force_ok = reference_preload_ok()

            if force_ok:
                self._t_stall += dt
            else:
                self._t_stall = 0.0

            self.msg = (
                f"초기 기준 Preload | "
                f"F={f.flex.applied_N:.2f}/{ZERO_PRELOAD_N:.1f}N  "
                f"E={f.ext.applied_N:.2f}/{ZERO_PRELOAD_N:.1f}N  "
                f"hold={self._t_stall:.2f}/{ZERO_PRELOAD_TIME:.1f}s"
            )

            if self._t_stall >= ZERO_PRELOAD_TIME:
                self._release_reference("initial")
                self.msg = "Initial preload done; releasing"
                return

            if self._t_state > ZERO_PRELOAD_TIMEOUT:
                self.abort("초기 기준 Preload 시간 초과")
            return

        # B. PRELOAD -> FLEXION RELEASE RAMP
        if self.state == "release_to_flex":
            if FLEX_RELEASE_RAMP_TIME <= 0.0:
                alpha = 1.0
            else:
                alpha = min(
                    1.0,
                    max(0.0, self._t_state / FLEX_RELEASE_RAMP_TIME)
                )

            flex_cmd = (
                alpha * t_cal_f
            )
            ext_cmd = -p["T_PAYOUT_E"] * alpha

            f.set_manual(flex_cmd, ext_cmd)

            self.msg = (
                f"Flexion 전환 ramp | "
                f"F={flex_cmd:.2f}N E={ext_cmd:.2f}N | "
                f"{alpha*100:.0f}%"
            )

            if alpha >= 1.0:
                self._enter("pull_f")
                self.msg = f"Flexion 시작 | F={t_cal_f:.1f}N E=0N"
            return

        # C. WARM-UP 후 공식 reference zero
        if self.state == "repreten":
            f.set_manual(ZERO_PRELOAD_N, ZERO_PRELOAD_N)
            force_ok = reference_preload_ok()

            if force_ok:
                self._t_stall += dt
            else:
                self._t_stall = 0.0

            self.msg = (
                f"Warm-up 후 공식 기준 Preload | "
                f"F={f.flex.applied_N:.2f}/{ZERO_PRELOAD_N:.1f}N  "
                f"E={f.ext.applied_N:.2f}/{ZERO_PRELOAD_N:.1f}N  "
                f"hold={self._t_stall:.2f}/{ZERO_PRELOAD_TIME:.1f}s"
            )

            if self._t_stall >= ZERO_PRELOAD_TIME:
                self._release_reference("official")
                self.msg = "Official preload done; releasing"
                return

            if self._t_state > ZERO_PRELOAD_TIMEOUT:
                self.abort("Warm-up 후 공식 기준 Preload 시간 초과")
            return

        # D. FLEXION = FORCE CONTROL
        if self.state == "pull_f":
            is_warmup = self.cycle < WARMUP_CYCLES
            phase_name = (
                "Warm-up"
                if is_warmup
                else f"{self.cycle - WARMUP_CYCLES + 1}/{SWEEP_CYCLES}"
            )

            # Main FLEX: force command; antagonist EXT: 예산 내 풀기 (v41).
            flex_wound = abs(f.flex.deg - self._pull_f_start_flex)
            ext_unwound = abs(f.ext.deg - self._pull_f_start_ext)
            ext_budget = self._ratio_ef * flex_wound + CAL_EXT_PAYOUT_MARGIN
            ext_payout_on = ext_unwound < ext_budget
            ext_cmd = -p["T_PAYOUT_E"] if ext_payout_on else 0.0
            f.set_manual(t_cal_f, ext_cmd)
            self._sample(+1)

            moved = abs(f.flex.deg - self._anchor)
            pull_ok = f.flex.applied_N >= 0.85 * t_cal_f
            slow = abs(f.flex.vel) < STALL_VEL

            self.msg = (
                f"{phase_name} Flex force | "
                f"F={t_cal_f:.1f}N Ecmd={ext_cmd:+.2f} "
                f"(E풀림 {ext_unwound:.0f}/{ext_budget:.0f}deg) | "
                f"moved={moved:.1f}deg"
            )

            if moved >= MIN_TRAVEL and pull_ok and slow:
                self._t_stall += dt
            else:
                self._t_stall = 0.0

            if self._t_stall >= STALL_TIME:
                self._pull_f_end_ext = f.ext.deg
                # 다음 사이클 EXT 풀기 예산용 실측 비율 (0.5~2.0 로 제한)
                if flex_wound >= MIN_TRAVEL:
                    self._ratio_ef = max(0.5, min(2.0, ext_unwound / flex_wound))
                if not is_warmup:
                    self.ends_flex.append(f.flex.deg)

                self._enter("flex_to_ext_release")
                self.msg = f"{phase_name} Flex 끝 → FLEX current ramp-down"
                return

            if self._t_state > PHASE_TIMEOUT:
                self.abort(f"Flexion timeout: moved={moved:.1f}, Fvel={f.flex.vel:.1f}, Evel={f.ext.vel:.1f}, pull={pull_ok}, slow={slow}")
            return

        # D2. Full flexion: release FLEX winding before EXT position return.
        if self.state == "flex_to_ext_release":
            a = min(1.0, self._t_state / max(1e-6, CAL_FLEX_TO_EXT_RELEASE_S))
            flex_cmd = self._flex_to_ext_start_N * (1.0 - a)
            f.set_manual(flex_cmd, 0.0)
            self.msg = f"Flex release {100*a:.0f}% | Fcmd={flex_cmd:.2f} N-eq"
            # MotorChannel.emit has its own RAMP_DOWN; do not start return
            # while a substantial residual FLEX winding command is applied.
            if a >= 1.0 and abs(f.flex.applied_N) < 0.08:
                self._enter("position_return")
                return
            if self._t_state > CAL_FLEX_TO_EXT_RELEASE_S + 4.0:
                self.abort("Flex release timeout before extension")
            return

        # E. EXTENSION = FORCE-BASED RETURN (v33)
        # 굽힘과 같은 정의: EXT 를 T_CAL_E 로 당기는데도 더 움직이지 않는 위치가
        # full extension. 위치 서보 없음. 정지 판정은 목표 장력 도달 후에만 센다.
        #   passive : EXT take-up 만, FLEX 스풀 풀어 글러브 탄성으로 펴지게 둠
        #   force   : EXT 장력 램프 → 목표 도달 → 0.4 s 창 변위 ≤ 2° → 완료
        if self.state == "position_return":
            self._return_phase_t += dt
            ext_travel = abs(f.ext.deg - self._return_ext_start_deg)

            # 이번 굽힘에서 EXT 스풀이 풀어 준 양(Δe)과 그 중 되감은 양.
            delta_e = self._pull_f_end_ext - self._pull_f_start_ext
            de_abs = max(1.0, abs(delta_e))
            returned = (self._pull_f_end_ext - f.ext.deg) * (1.0 if delta_e >= 0 else -1.0)
            ret_frac = returned / de_abs
            past_start = returned - de_abs      # >0: 굽힘 시작점을 지나 더 감김(신장분)

            # FLEX payout 예산: 굽힘 때 감은 양 + 여유 만큼만 되푼다. 그 안에서는
            # 손가락이 멈춰 있어도 계속 풀어 FLEX 텐던이 저항하지 않게 한다.
            # remaining > 0 : 굽힘 시작점 기준으로 아직 감겨 있는 양.
            # 굽힘 시작점을 여유(25°)만큼 지나 되풀리면 그 뒤는 슬랙이므로 payout 중지.
            remaining = self._return_wind_sign * (f.flex.deg - self._pull_f_start_flex)
            # v39: 글러브/케이블이 ext 쪽에서 past_start 만큼 밀려 들어갔으면 flex 텐던도
            # 같은 손끝에 묶여 있으므로 그만큼 더 풀려야 한다. 예산 = 여유 + ext take-up.
            payout_limit = EXT_PAYOUT_MARGIN_DEG + max(0.0, past_start)
            payout_on = remaining > -payout_limit
            flex_cmd = -p["T_PAYOUT"] if payout_on else 0.0

            # ── 단계 1: passive release ──
            if self._return_phase == "passive":
                ext_cmd = max(p["T_PRE"], EXT_PASSIVE_ASSIST_N)
                f.set_manual(flex_cmd, ext_cmd)
                self._sample(-1)

                if abs(f.ext.vel) < EXT_PASSIVE_STALL_DPS:
                    self._return_passive_still_t += dt
                else:
                    self._return_passive_still_t = 0.0

                self.msg = (
                    f"Extension passive | "
                    f"E={f.ext.deg:+.1f}deg vel={f.ext.vel:+.0f}dps | "
                    f"Fcmd={flex_cmd:+.2f} (wound {remaining:+.0f}deg) "
                    f"Ecmd={ext_cmd:.2f}N | "
                    f"travel={ext_travel:.1f}deg | "
                    f"still={self._return_passive_still_t:.2f}/{EXT_PASSIVE_STALL_S:.2f}s"
                )

                done_passive = (
                    self._return_phase_t >= EXT_PASSIVE_MIN_S
                    and (
                        self._return_passive_still_t >= EXT_PASSIVE_STALL_S
                        or self._return_phase_t >= EXT_PASSIVE_MAX_S
                    )
                )
                if done_passive:
                    self._return_phase = "force"
                    self._return_phase_t = 0.0
                    self._return_obs_t = 0.0
                    self._return_ref_e = f.ext.deg
                    self._t_stall = 0.0
                    self._return_ramp_from = max(p["T_PRE"], f.ext.applied_N)
                return

            # ── 단계 2: force ramp → stall at target force ──
            ext_cmd = min(
                return_cmd_limit,
                self._return_ramp_from + EXT_RAMP_NPS * self._return_phase_t
            )

            # Breakaway 펄스 진행 중이면 목표 대신 펄스 레벨을 낸다.
            if self._bk_active:
                self._bk_t += dt
                ext_cmd = self._bk_level
                if self._bk_t >= EXT_BREAKAWAY_S:
                    self._bk_active = False
                    self._return_obs_t = 0.0
                    self._return_ref_e = f.ext.deg

            f.set_manual(flex_cmd, ext_cmd)
            self._sample(-1)

            # 정지 판정은 목표 장력에 도달한 뒤부터만 유효.
            # "약하게 당겨서 안 움직임"과 "끝에 닿아서 안 움직임"을 여기서 구분한다.
            force_ok = f.ext.applied_N >= EXT_FORCE_OK_RATIO * return_cmd_limit

            if not force_ok:
                self._return_obs_t = 0.0
                self._return_ref_e = f.ext.deg
            else:
                self._return_obs_t += dt

            de_window = abs(f.ext.deg - self._return_ref_e)

            # 창 내 변위가 기준을 넘으면 즉시 창을 현재 위치에서 다시 시작
            if force_ok and de_window > EXT_STALL_DELTA_DEG:
                self._return_obs_t = 0.0
                self._return_ref_e = f.ext.deg
                de_window = 0.0
                self._bk_tries = 0        # 다시 움직였으면 새 정지 구간으로 본다
                self._mid_hold_t = 0.0

            stalled = (
                force_ok
                and not self._bk_active
                and self._return_obs_t >= EXT_STALL_WINDOW
                and abs(f.ext.vel) <= EXT_STALL_VEL_TOL_DPS
            )

            self.msg = (
                f"Extension force | "
                f"E={f.ext.deg:+.1f}deg | "
                f"Ecmd={ext_cmd:.2f}/{return_cmd_limit:.1f}N "
                f"applied={f.ext.applied_N:.2f}N ok={force_ok} | "
                f"Fcmd={flex_cmd:+.2f} (wound {remaining:+.0f}deg) | "
                f"bk={self._bk_tries}/{EXT_BREAKAWAY_TRIES} | "
                f"ret={ret_frac*100:.0f}% past={past_start:+.0f}deg | "
                f"travel={ext_travel:.1f}deg "
                f"dE={de_window:.1f}deg "
                f"({self._return_obs_t:.2f}/{EXT_STALL_WINDOW:.2f}s)"
            )

            # 굽힘 시작점을 신장분 이상으로 한참 지나면 텐던 이탈/착용 틀어짐.
            # take-up 한계: 손가락은 펴졌는데 글러브/앵커가 밀리며 계속 감기는 상황.
            # abort 하지 않고 끝으로 처리한다 (아래 end_reached).
            takeup_limited = past_start > EXT_OVERTRAVEL_DEG

            returned_enough = ret_frac >= EXT_RETURN_MIN_FRAC

            # 목표 장력으로 당기는데 되감은 양이 굽힘 때 풀린 양에 한참 못 미친 채 정지
            # = 중간 고착 (라우팅 stick-slip / FLEX 텐던이 붙잡음) → breakaway.
            if stalled and not returned_enough and not takeup_limited:
                # 1) 먼저 목표 장력을 유지한 채 기다린다 (creep / 가벼운 보조로 풀림).
                self._mid_hold_t += dt
                if self._mid_hold_t < EXT_MIDSTALL_HOLD_S:
                    self.msg = (
                        f"Extension 중간 정지 — 유지 중 ({self._mid_hold_t:.1f}/"
                        f"{EXT_MIDSTALL_HOLD_S:.0f}s @ {f.ext.applied_N:.2f}N) "
                        f"ret={ret_frac*100:.0f}% | 손가락을 살짝 펴 주세요"
                    )
                    return
                # 2) 그래도 안 움직이면 (설정된 경우) breakaway 펄스.
                if (self._bk_tries < EXT_BREAKAWAY_TRIES
                        and self._bk_total < EXT_BREAKAWAY_TOTAL):
                    self._bk_tries += 1
                    self._bk_total += 1
                    self._bk_level = min(
                        EXT_BREAKAWAY_MAX_N,
                        return_cmd_limit + EXT_BREAKAWAY_STEP_N * self._bk_tries
                    )
                    self._bk_active = True
                    self._bk_t = 0.0
                    self._mid_hold_t = 0.0
                    self._return_obs_t = 0.0
                    self._return_ref_e = f.ext.deg
                    self._t_stall = 0.0
                    self.msg = (
                        f"Extension breakaway {self._bk_tries}/{EXT_BREAKAWAY_TRIES} "
                        f"@ {self._bk_level:.1f}N | E={f.ext.deg:+.1f}deg "
                        f"ret={ret_frac*100:.0f}%"
                    )
                    return
                self.abort(
                    f"Extension stalled at {ret_frac*100:.0f}% of flexion travel "
                    f"(need {EXT_RETURN_MIN_FRAC*100:.0f}%): "
                    f"E={f.ext.deg:+.1f}deg at {f.ext.applied_N:.2f}N "
                    f"after {self._bk_tries} breakaway pulses "
                    f"(Fdeg={f.flex.deg:+.1f}, wound {remaining:+.0f}deg)"
                )
                return

            end_reached = (stalled and returned_enough) or takeup_limited
            if end_reached:
                self._t_stall += dt
            else:
                self._t_stall = 0.0

            if takeup_limited:
                self.msg = (
                    f"Extension take-up 한계 ({past_start:+.0f}deg > "
                    f"{EXT_OVERTRAVEL_DEG:.0f}deg @ {f.ext.applied_N:.2f}N) "
                    f"→ 끝 처리. 글러브/손끝 앵커 미끄러짐 점검"
                )

            if self._t_stall >= EXT_RETURN_SETTLE:
                is_warmup = self.cycle < WARMUP_CYCLES
                self._return_obs_t = 0.0

                # 완료: 양쪽 지령 0 → 다음 단계(preload)에서 다시 잡는다.
                f.set_manual(0.0, 0.0)
                self.takeup_log.append(
                    (round(f.ext.applied_N, 2), round(past_start, 1),
                     "takeup_limited" if takeup_limited else "stall")
                )

                if is_warmup:
                    self.cycle += 1
                    self._enter("repreten")
                    self.msg = (
                        f"Warm-up full extension (ret={ret_frac*100:.0f}%, "
                        f"past={past_start:+.0f}deg @ {return_cmd_limit:.1f}N) "
                        f"→ reference preload / official zero"
                    )
                    return

                self._enter("handover_f")
                self.msg = (
                    f"Full extension (ret={ret_frac*100:.0f}%, "
                    f"past={past_start:+.0f}deg @ {return_cmd_limit:.1f}N) "
                    f"→ reference preload / next cycle"
                )
                return

            if self._t_state > RETURN_TIMEOUT:
                self.abort(
                    f"Extension return 시간 초과 | "
                    f"E={f.ext.deg:+.1f}deg applied={f.ext.applied_N:.2f}N | "
                    f"travel={ext_travel:.1f}deg"
                )
            return

        # F. 실제 cycle FULL-EXTENSION 4N/4N ALIGNMENT
        if self.state == "handover_f":
            f.set_manual(ZERO_PRELOAD_N, ZERO_PRELOAD_N)
            force_ok = reference_preload_ok()

            if force_ok:
                self._t_stall += dt
            else:
                self._t_stall = 0.0

            self.msg = (
                f"Full-extension 기준 정렬 | "
                f"F={f.flex.applied_N:.2f}/{ZERO_PRELOAD_N:.1f}N  "
                f"E={f.ext.applied_N:.2f}/{ZERO_PRELOAD_N:.1f}N | "
                f"hold={self._t_stall:.2f}/{ZERO_PRELOAD_TIME:.1f}s | "
                f"Fdeg={f.flex.deg:+.1f} Edeg={f.ext.deg:+.1f}"
            )

            if self._t_stall >= ZERO_PRELOAD_TIME:
                self._release_reference("cycle")
                self.msg = "Cycle preload done; releasing"
                return

            if self._t_state > ZERO_PRELOAD_TIMEOUT:
                self.abort("Full-extension 기준 정렬 시간 초과")
            return

        self.abort(f"알 수 없는 calibration state: {self.state}")

    def _finish(self, p):
        f = self.finger
        f.set_manual(0.0, 0.0)

        try:
            raw_flexion, raw_extension, rom_mm, hyst_mm = (
                self._build_directional_maps()
            )
        except Exception as e:
            self.abort(f"지도 생성 실패: {e}")
            return

        meta = dict(
            rom_mm=round(rom_mm, 2),
            hyst_mm=round(hyst_mm, 2),
            t_cal_f=p["T_CAL_F"],
            t_cal_e=p["T_CAL_E"],
            t_pre=p["T_PRE"],
            t_payout=p["T_PAYOUT"],
            t_payout_e=p["T_PAYOUT_E"],
            cycles=SWEEP_CYCLES,
            warmup_cycles=WARMUP_CYCLES,
            map_type="direction_separated",
            cycle_rebased_full_extension=True,
            ext_takeup_deg=list(self.takeup_log),   # (N, deg) 사이클별 탄성 take-up
            runtime_tracking="directional_absolute_blend",
            zero_reference="released_after_preload",
            date=time.strftime("%Y-%m-%d %H:%M")
        )

        f.map.set_branches(
            raw_flexion,
            raw_extension,
            meta
        )
        f.map_aligned = True

        if f.map.ready:
            name = time.strftime(f"map_{f.key}_%m%d_%H%M")
            if getattr(self, "autosave", True):
                f.map.save(name)

            self.result = (
                rom_mm,
                hyst_mm,
                min(len(raw_flexion), len(raw_extension))
            )

            self.state = "done"
            self.msg = (
                f"완료: ROM {rom_mm:.1f}mm, "
                f"히스테리시스 {hyst_mm:.2f}mm | "
                f"Flexion {len(raw_flexion)}점 / "
                f"Extension {len(raw_extension)}점 "
                f"→ {name}.json 저장"
            )
        else:
            self.abort("방향별 지도 점 부족")

    @staticmethod
    def _split_direction_segments(samples):
        segments = []
        if not samples:
            return segments

        cur_d = samples[0][0]
        cur = []

        for d, fd, ed in samples:
            if d != cur_d and cur:
                segments.append((cur_d, cur))
                cur = []
                cur_d = d

            cur.append((fd, ed))

        if cur:
            segments.append((cur_d, cur))

        return segments

    def _build_directional_maps(self):
        map_samples = self.samples[self.map_start_idx:]
        if not map_samples:
            raise ValueError("Warm-up 제외 후 calibration sample이 없습니다.")

        segments = self._split_direction_segments(map_samples)
        flex_segments = [pts for d, pts in segments if d == +1]
        ext_segments = [pts for d, pts in segments if d == -1]

        if len(flex_segments) < SWEEP_CYCLES:
            raise ValueError(
                f"Flexion 기록 부족 ({len(flex_segments)}/{SWEEP_CYCLES}회)"
            )
        if len(ext_segments) < SWEEP_CYCLES:
            raise ValueError(
                f"Extension 기록 부족 ({len(ext_segments)}/{SWEEP_CYCLES}회)"
            )

        flex_segments = flex_segments[-SWEEP_CYCLES:]
        ext_segments = ext_segments[-SWEEP_CYCLES:]

        def rebase_flexion(seg):
            rf, re = seg[0]
            pts = [(fd-rf, ed-re) for fd, ed in seg]
            if pts:
                pts[0] = (0.0, 0.0)
            return pts

        def rebase_extension(seg):
            rf, re = seg[-1]
            pts = [(fd-rf, ed-re) for fd, ed in seg]
            if pts:
                pts[-1] = (0.0, 0.0)
            return pts

        flex_rebased = [rebase_flexion(seg) for seg in flex_segments]
        ext_rebased = [rebase_extension(seg) for seg in ext_segments]

        full_flex_vals = []
        for seg in flex_rebased:
            if seg:
                full_flex_vals.append(seg[-1][0])
        for seg in ext_rebased:
            if seg:
                full_flex_vals.append(seg[0][0])

        if not full_flex_vals:
            raise ValueError("Full-flex endpoint 계산 실패")

        f_flex_end = statistics.median(full_flex_vals)
        span = abs(f_flex_end)
        if span < MIN_TRAVEL:
            raise ValueError(f"Rebased 이동량 부족 ({span:.0f}°)")

        lo, hi = sorted((0.0, f_flex_end))
        nb = MAP_BINS
        binw = (hi-lo) / (nb-1)

        def segment_to_bins(points):
            bins = [[] for _ in range(nb)]
            for fd, ed in points:
                if not (lo-binw <= fd <= hi+binw):
                    continue
                i = int(round((fd-lo)/binw))
                i = max(0, min(nb-1, i))
                bins[i].append(ed)
            return [
                statistics.fmean(vals) if vals else None
                for vals in bins
            ]

        flex_cycles = [segment_to_bins(seg) for seg in flex_rebased]
        ext_cycles = [segment_to_bins(seg) for seg in ext_rebased]

        def average_cycles(cycle_bins):
            raw = []
            for i in range(nb):
                vals = [cyc[i] for cyc in cycle_bins if cyc[i] is not None]
                if not vals:
                    continue
                fd = lo + i*binw
                ed = statistics.fmean(vals)
                raw.append((i, fd, ed))
            return raw

        raw_f = average_cycles(flex_cycles)
        raw_e = average_cycles(ext_cycles)

        if len(raw_f) < 4:
            raise ValueError(f"Flexion 유효 bin 부족 ({len(raw_f)})")
        if len(raw_e) < 4:
            raise ValueError(f"Extension 유효 bin 부족 ({len(raw_e)})")

        def orient_and_zero(raw):
            if abs(raw[0][1]) > abs(raw[-1][1]):
                raw = list(reversed(raw))
            out = [
                (new_i, fd, ed)
                for new_i, (_, fd, ed) in enumerate(raw)
            ]
            if out:
                out[0] = (0, 0.0, 0.0)
            return out

        raw_f = orient_and_zero(raw_f)
        raw_e = orient_and_zero(raw_e)

        hyst_list = []
        for _, fd_f, ed_f in raw_f:
            nearest = min(raw_e, key=lambda p: abs(p[1]-fd_f))
            if abs(nearest[1]-fd_f) <= max(binw*0.75, 1.0):
                hyst_list.append(abs(ed_f-nearest[2]))

        rom_mm = span * DEG_TO_MM
        hyst_mm = (
            statistics.fmean(hyst_list) * DEG_TO_MM
            if hyst_list else 0.0
        )
        return raw_f, raw_e, rom_mm, hyst_mm


# ═══════════════════════════════════════════════════════ 결합 감시

class Coupling:

    def __init__(self):
        self.err = self.force = self.energy = 0.0

    def monitor(self, pairs, dt):
        self.err = 0.0
        self.force = 0.0
        p_sum = 0.0
        n = 0
        for f1, f2 in pairs:
            self.err += abs(f1.curl - f2.curl)
            self.force = max(self.force, f1.cpl_f, f1.cpl_e, f2.cpl_f, f2.cpl_e)
            n += 1
            for g in (f1, f2):
                if g.map.ready:
                    wf, we = g.map.wind_dirs()
                    p_sum += (g.cpl_f * (wf * g.flex.vel)
                              + g.cpl_e * (we * g.ext.vel)) * DEG_TO_MM / 1000.0
        if n:
            self.err /= n
        self.energy += p_sum * dt

    def reset(self):
        self.energy = 0.0


# ═══════════════════════════════════════════════════════ 제어 스레드

class GloveSweepCal:
    """Run finger sweeps in phases and save one glove bundle.

    엄지와 검지가 동시에 굽혀지면 서로 부딪혀 끝까지 굽혀지지 않으므로
    SWEEP_PHASES 순서대로 진행한다: 엄지 단독 → 검지+중지 동시.
    """
    SWEEP_PHASES = (("T",), ("I", "M"))

    def __init__(self):
        self.children = []        # 현재 phase 에서 진행 중인 SweepCal
        self.completed = []       # 이미 끝난 phase 의 SweepCal
        self.fingers = []
        self.phases = []
        self.phase_idx = 0
        self.state = "off"
        self.msg = ""
        self.cycle = 0

    @property
    def finger(self):
        return self.fingers[0] if self.fingers else None

    @property
    def samples(self):
        return self.children[0].samples if self.children else []

    def begin(self, fingers):
        self.fingers = list(fingers)
        by_fid = {f.fid: f for f in self.fingers}
        self.phases = []
        used = set()
        for group in self.SWEEP_PHASES:
            ph = [by_fid[fid] for fid in group if fid in by_fid]
            used.update(f.fid for f in ph)
            if ph:
                self.phases.append(ph)
        rest = [f for f in self.fingers if f.fid not in used]
        if rest:
            self.phases.append(rest)
        self.children = []
        self.completed = []
        self.state = "running"
        self.cycle = 0
        for f in self.fingers:
            f.map_aligned = False
        self._start_phase(0)

    def _phase_label(self):
        names = "+".join(f.fname for f in self.phases[self.phase_idx])
        return f"[{self.phase_idx + 1}/{len(self.phases)} {names}]"

    def _start_phase(self, idx):
        self.phase_idx = idx
        self.children = []
        for f in self.phases[idx]:
            child = SweepCal()
            child.autosave = False
            child.begin(f)
            self.children.append(child)
        self.msg = f"{self._phase_label()} sweep 시작"

    def abort(self, msg="Stopped"):
        for child in self.children:
            child.abort(msg)
            child.finger.kill()
        self.state = "abort"
        self.msg = msg

    def step(self, dt, params):
        if self.state != "running":
            for f in self.fingers:
                f.set_manual(0., 0.)
            return
        try:
            active = set(c.finger for c in self.children)
            # 현재 phase 가 아닌 손가락은 장력 0 (대기/완료)
            for f in self.fingers:
                if f not in active:
                    f.set_manual(0., 0.)
            for child in self.children:
                if child.state != "done":
                    child.step(dt, params)
                else:
                    child.finger.set_manual(0., 0.)
                if child.state == "abort":
                    self.abort(f"{child.finger.name}: {child.msg}")
                    return
            self.cycle = min(c.cycle for c in self.children)
            self.msg = self._phase_label() + " " + " | ".join(
                f"{c.finger.fid}: {c.msg if c.state != 'done' else 'Done'}" for c in self.children)
            if all(c.state == "done" for c in self.children):
                self.completed.extend(self.children)
                for c in self.children:
                    c.finger.set_manual(0., 0.)
                if self.phase_idx + 1 < len(self.phases):
                    self._start_phase(self.phase_idx + 1)
                    return
                self._save()
                self.state = "done"
                for f in self.fingers:
                    f.set_manual(0., 0.)
        except Exception as exc:
            self.abort(f"Glove calibration failed: {exc}")

    def _save(self):
        glove = self.fingers[0].glove_short
        payload = {"format": "glove_tendon_maps_v1", "glove": glove,
                   "date": time.strftime("%Y-%m-%d %H:%M:%S"), "fingers": {}}
        for f in self.fingers:
            m = f.map
            payload["fingers"][f.fid] = dict(
                name=f.key, map_type="direction_separated",
                raw_flexion=m.raw_flexion, raw_extension=m.raw_extension,
                raw=m.raw, meta=m.meta)
        os.makedirs(MAP_DIR, exist_ok=True)
        path = os.path.join(MAP_DIR, f"{glove}.json")
        temporary = path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as fp:
            json.dump(payload, fp, ensure_ascii=False, indent=2)
        os.replace(temporary, path)
        self.msg = f"{glove}: all fingers complete -> {path}"


class ControlThread(threading.Thread):

    def __init__(self):
        super().__init__(daemon=True)
        self.bank = AOBank(MOTOR_SPECS, _active_keys())
        self.g1 = Glove("장갑1 (환자)", "G1", self.bank)
        self.g2 = Glove("장갑2 (치료사)", "G2", self.bank)
        self.gloves = (self.g1, self.g2)
        self.cpl = Coupling()
        self.sweep = GloveSweepCal()

        self.params = dict(DEFAULT_PARAMS)
        # UI changes and a complete output tick share the same lock.
        self.lock = threading.RLock()
        self.params_version = 0
        self.active_params = dict(self.params)
        self.active_params_version = -1
        self.control_error = ""

        self.mode = "idle"            # idle | map | sweep | run
        self.coupling_mode = "force"
        self.cal_finger = None        # 캘리브레이션 대상 FingerUnit
        self.manual = (0.0, 0.0)
        self.manual_on = False

        self.running = True
        self.estop = False
        self.estop_msg = ""
        self.zero_req = False
        self.run_zero_req = False
        self.run_zero_active = False
        self.run_zero_t = 0.0
        self.run_zero_stable = 0.0
        self.run_zero_phase = "hold"
        self.run_zero_release_t = 0.0
        self.run_zero_msg = ""
        self.hz = 0.0

        self.log_q = queue.Queue()
        self.logging = False
        self.trial_id = "UNLABELED"
        self.event_marker = ""
        self.t0 = time.perf_counter()

    # ── 외부(UI) 명령 ──

    def set_param(self, k, v):
        self.set_params({k: v})

    def set_params(self, values):
        # Validate the whole batch before committing; never partially apply.
        limits = dict(CONTROL_PARAM_LIMITS, **CAL_PARAM_LIMITS)
        checked = {}
        for k, value in values.items():
            if k not in limits:
                raise ValueError(f"알 수 없는 파라미터: {k}")
            v = float(value)
            lo, hi = limits[k]
            if not math.isfinite(v) or not lo <= v <= hi:
                raise ValueError(f"{k}: {lo:g} ~ {hi:g} 범위의 숫자를 입력하세요.")
            checked[k] = v
        with self.lock:
            self.params.update(checked)
            self.params_version += 1

    def param_snapshot(self):
        with self.lock:
            return (dict(self.params), self.params_version,
                    dict(self.active_params), self.active_params_version)

    def set_coupling_mode(self, mode):
        if mode not in ("force", "position_velocity"):
            raise ValueError("Invalid mode")
        with self.lock:
            if any(f.enabled for g in self.gloves for f in g.fingers.values()):
                raise ValueError("Disable all controls before changing mode")
            self.coupling_mode = mode

    def set_preset(self, name):
        kc1, bc1, kc2, bc2 = PRESETS[name]
        self.set_params(dict(KC1=kc1, BC1=bc1, KC2=kc2, BC2=bc2))

    def request_zero(self):
        with self.lock:
            if not self.estop:
                self.zero_req = True

    def request_run_zero(self):
        with self.lock:
            if not self.estop:
                self.run_zero_req = True

    def trigger_estop(self, msg="사용자"):
        with self.lock:
            self.estop = True
            self.estop_msg = msg
            self.stop_all()
            # Also flush when the request comes from the UI, without waiting
            # for another tick. Never resume an interrupted zeroing procedure.
            self.bank.zero()

    def clear_estop(self):
        with self.lock:
            if self.control_error:
                return False
            self.stop_all()
            # Preserve map coordinates and gains; never re-enable motors.
            for g in self.gloves:
                for f in g.fingers.values():
                    for m in f.motors:
                        m.vel = 0.0
                        m._prev = m.deg
            self.estop_msg = ""
            self.cpl.reset()
            self.estop = False
            return True

    def stop_all(self):
        with self.lock:
            for g in self.gloves:
                g.kill()
            self.manual_on = False
            self.zero_req = self.run_zero_req = False
            self.run_zero_active = False
            self.run_zero_msg = ""
            if self.mode == "run_zero":
                self.mode = "run"
            if self.sweep.state not in ("off", "done", "abort"):
                self.sweep.abort("사용자 정지")
            # Stopping output must not overwrite the user's applied gains.
            self.bank.zero()

    def set_control_enabled(self, finger, kind, on):
        with self.lock:
            if on:
                if self.estop or self.control_error:
                    return "비상정지를 해제한 뒤 다시 체크하세요."
                if self.mode != "run" or self.run_zero_req:
                    return "양방향 제어 탭에서 0점 초기화 완료 후 체크하세요."
                if not finger.map.ready:
                    return f"{finger.name}: 먼저 지도를 준비하세요."
                if not finger.map_aligned:
                    return f"{finger.name}: 먼저 '전체 엔코더 0점'을 완료하세요."
                if kind == "coupling":
                    partner_glove = self.g2 if finger.glove_short == "G1" else self.g1
                    partner = partner_glove.fingers.get(finger.fid)
                    if (finger.single or partner is None or partner.single
                            or not partner.map.ready or not partner.map_aligned):
                        return "양쪽 같은 손가락의 지도·0점·엔코더 2개가 모두 필요합니다."
            finger.set_control(kind, on)
            # The next regular tick recomputes the remaining loop's output.
            # Both-off has already killed the motor commands above.
            self.bank.flush()
            return None

    def start_sweep(self, finger):
        with self.lock:
            if self.estop:
                return
            self.stop_all()
            self.cal_finger = finger
            glove = next(g for g in self.gloves if g.short == finger.glove_short)
            fingers = list(glove.fingers.values())
            if not fingers or any(f.single for f in fingers):
                return
            self.sweep.begin(fingers)
            self.mode = "sweep"

    def _apply_run(self, p, dt):
        prs = self.pairs()
        for f1, f2 in prs:
            f1.coupling_mode = f2.coupling_mode = self.coupling_mode
            pair_ready = all(
                f.map.ready and f.map_aligned and not f.single for f in (f1, f2)
            )
            if pair_ready:
                F1 = f1.couple(f2.curl, f2.curl_vel, p["KC1"], p["BC1"], p["OVER"])
                F2 = f2.couple(f1.curl, f1.curl_vel, p["KC2"], p["BC2"], p["OVER"])
            else:
                f1.cpl_f = f1.cpl_e = f2.cpl_f = f2.cpl_e = 0.0
                F1 = F2 = (0.0, 0.0)
            f1.apply(F1, p, dt)
            f2.apply(F2, p, dt)
        paired = {id(f) for pr in prs for f in pr}
        for g in self.gloves:
            for f in g.fingers.values():
                if id(f) not in paired:
                    f.cpl_f = f.cpl_e = 0.0
                    f.apply((0.0, 0.0), p, dt)
        self.cpl.monitor(prs, dt)

    # ── 결합 대상 손가락 쌍 ──

    def pairs(self):
        out = []
        for fid in FINGER_IDS:
            f1 = self.g1.fingers.get(fid)
            f2 = self.g2.fingers.get(fid)
            if f1 and f2:
                out.append((f1, f2))
        return out

    # ── 루프 ──

    def run(self):
        t_last = time.perf_counter()
        try:
            while self.running:
                now = time.perf_counter()
                dt = max(0.0005, min(now - t_last, 0.05))
                t_last = now
                with self.lock:
                    self._tick(now, dt)
                elapsed = time.perf_counter() - now
                if elapsed < CTRL_PERIOD:
                    time.sleep(CTRL_PERIOD - elapsed)
        except Exception as exc:
            self.control_error = f"{type(exc).__name__}: {exc}"
            traceback.print_exc()
            self.trigger_estop(f"제어 오류 — {self.control_error}")
        finally:
            self.running = False
            with self.lock:
                try:
                    self.stop_all()
                finally:
                    for g in self.gloves:
                        g.close()
                    self.bank.close()

    def _tick(self, now, dt):
        # Called only while holding self.lock, including parameter snapshot,
        # state transition, command calculation, output and logging.
        p = dict(self.params)
        self.hz += 0.05 * (1.0 / dt - self.hz)

        if self.zero_req:
            self.zero_req = False
            for g in self.gloves:
                g.zero()
                for f in g.fingers.values():
                    f.map_aligned = False

        if self.run_zero_req:
            self.run_zero_req = False
            for g in self.gloves:
                for f in g.fingers.values():
                    f.kill()
                    f.map_aligned = False
            self.run_zero_active = True
            self.run_zero_t = 0.0
            self.run_zero_stable = 0.0
            self.run_zero_phase = "hold"
            self.run_zero_release_t = 0.0
            self.run_zero_msg = "제어 기준 초기화: Pretension 시작"
            self.mode = "run_zero"

        for g in self.gloves:
            g.read(dt, p)

        # ── 안전 (v6 중복 루프 버그 수정: 평탄한 단일 패스) ──
        if not self.estop:
            for g in self.gloves:
                for f in g.fingers.values():
                    for m in f.motors:
                        if not m.enc_ok:
                            continue
                        if abs(m.vel) > SPIKE_VEL:
                            self.trigger_estop(f"{m.key} 과속")
                        elif self.mode != "run_zero" and abs(m.mm) > CABLE_LIMIT_MM:
                            self.trigger_estop(f"{m.key} 이동한계")
                    if self.mode != "run_zero" and max(abs(f.d_flex), abs(f.d_ext)) > DTH_LIMIT:
                        self.trigger_estop(f"{f.key} Δθ 이상")

        if self.estop:
            for g in self.gloves:
                g.kill()
            if self.sweep.state not in ("off", "done", "abort"):
                self.sweep.abort(f"비상정지: {self.estop_msg}")

        elif self.mode == "map":
            cf = self.cal_finger
            for g in self.gloves:
                for f in g.fingers.values():
                    if f is cf and self.manual_on:
                        f.set_manual(*self.manual)
                    else:
                        f.set_manual(0.0, 0.0)

        elif self.mode == "run_zero":
            self.run_zero_t += dt
            ready = [f for g in self.gloves for f in g.fingers.values() if f.map.ready]
            if self.run_zero_phase == "hold":
                for g in self.gloves:
                    for f in g.fingers.values():
                        v = ZERO_PRELOAD_N if f in ready else 0.
                        f.set_manual(v,v)
                ok = all(f.flex.applied_N >= .85*ZERO_PRELOAD_N and
                         f.ext.applied_N >= .85*ZERO_PRELOAD_N for f in ready)
                self.run_zero_stable = self.run_zero_stable+dt if ok else 0.
                self.run_zero_msg = f"Preload {self.run_zero_stable:.1f}/{ZERO_PRELOAD_TIME}s"
                if self.run_zero_stable >= ZERO_PRELOAD_TIME:
                    self.run_zero_phase = "release"
                    self.run_zero_release_t = 0.
            elif self.run_zero_phase == "release":
                self.run_zero_release_t += dt
                alpha = min(1.,self.run_zero_release_t/ZERO_RELEASE_TIME)
                for g in self.gloves:
                    for f in g.fingers.values():
                        v = ZERO_PRELOAD_N*(1-alpha) if f in ready else 0.
                        f.set_manual(v,v)
                self.run_zero_msg = f"Release {alpha*100:.0f}%"
                if alpha >= 1 and all(max(abs(f.flex.applied_N),abs(f.ext.applied_N)) < .04 for f in ready):
                    self.run_zero_phase = "settle"
                    self.run_zero_release_t = 0.
            else:
                self.run_zero_release_t += dt
                for g in self.gloves:
                    for f in g.fingers.values(): f.set_manual(0.,0.)
                if self.run_zero_release_t >= ZERO_SETTLE_TIME:
                    if any(max(abs(f.flex.vel),abs(f.ext.vel)) > 18 for f in ready):
                        self.run_zero_msg = "Zero FAILED: encoder moving"
                    else:
                        for g in self.gloves:
                            g.zero()
                            for f in g.fingers.values():
                                f.kill()
                                if f.map.ready: f.map_aligned = True
                        self.run_zero_msg = "Released zero complete"
                    self.run_zero_active = False
                    self.mode = "run"
            if self.run_zero_t > ZERO_PRELOAD_TIMEOUT + ZERO_RELEASE_TIME + 4:
                for g in self.gloves: g.kill()
                self.run_zero_active = False
                self.run_zero_msg = "Zero FAILED: timeout"
                self.mode = "run"

        elif self.mode == "sweep":
            cf = self.cal_finger
            self.sweep.step(dt, p)
            for g in self.gloves:
                for f in g.fingers.values():
                    if f not in self.sweep.fingers:
                        f.set_manual(0.0, 0.0)

        elif self.mode == "run":
            self._apply_run(p, dt)

        else:
            for g in self.gloves:
                g.kill()

        for g in self.gloves:
            for m in g.all_motors:
                m.emit(dt)
        self.bank.flush()
        self.active_params = dict(p)
        self.active_params_version = self.params_version

        if self.logging:
            row = [now - self.t0,
                   p["KC1"], p["BC1"], p["KC2"], p["BC2"],
                   p["OVER"], p["T_BASE"]]
            for f1, f2 in self.pairs():
                row += [f1.curl, f2.curl, f1.curl - f2.curl,
                        f1.cpl_f, f1.cpl_e, f2.cpl_f, f2.cpl_e,
                        f1.flex.applied_N, f1.ext.applied_N,
                        f2.flex.applied_N, f2.ext.applied_N,
                        f1.flex.deg, f1.ext.deg, f2.flex.deg, f2.ext.deg,
                        f1.motion, f2.motion]
            row += [self.cpl.energy]
            row += [self.active_params_version] + [p[k] for k in LOG_TUNING_KEYS]
            for g in self.gloves:
                for f in g.fingers.values():
                    row += [int(f.auto_enabled), int(f.coupling_enabled)]
            # Extended diagnostics; all forces below are COMMAND estimates, not sensor measurements.
            row += [time.time(), dt, 1.0/dt if dt > 0 else 0.0,
                    self.mode, self.sweep.state, self.sweep.cycle,
                    int(self.estop), self.estop_msg, self.trial_id,
                    self.event_marker, self.cpl.err, self.cpl.force]
            self.event_marker = ""
            for g in self.gloves:
                for fid in FINGER_IDS:
                    f = g.fingers.get(fid)
                    if f is None:
                        row += [""] * 28
                        continue
                    row += [f.curl, f.curl_vel, f.source or "", f.driven or "",
                            f.motion, int(f.map.ready), int(f.map_aligned),
                            f.tgt_flex if f.tgt_flex is not None else "",
                            f.tgt_ext if f.tgt_ext is not None else "",
                            f.d_flex, f.d_ext, f.pos_i, f.soft, f.coupling_soft,
                            f.cpl_f, f.cpl_e]
                    for m in f.motors:
                        row += [m.deg_raw, m.deg, m.vel, m.target_N,
                                m.applied_N, m.sign * max(AO_MIN_V, min(AO_MAX_V,
                                m.applied_N * SPOOL_R / KT * V_PER_AMP)),]
            row += [self.coupling_mode]
            for g in self.gloves:
                for fid in FINGER_IDS:
                    f = g.fingers.get(fid)
                    row += [f.curl_raw_flex, f.curl_raw_ext, f.curl_target,
                            int(f._curl_blend_active)] if f else ["", "", "", ""]
            self.log_q.put(row)



# ═══════════════════════════════════════════════════════ UI





class StatusBarsView(tk.Canvas):
    """손바닥 그림의 실제 손가락 위치에 굽힘 게이지를 표시한다."""

    def __init__(self, parent, glove, width=470, height=340):
        super().__init__(parent, width=width, height=height,
                         background="#f7fafc", highlightthickness=1,
                         highlightbackground="#dce4eb")
        self.glove = glove
        self.W, self.H = width, height

    @staticmethod
    def _clamp01(v):
        return max(0.0, min(1.0, v))

    def update_view(self):
        self.delete("all")
        width = max(self.winfo_width(), self.W) if self.winfo_width() <= 1 else self.winfo_width()
        height = self.winfo_height() if self.winfo_height() > 1 else self.H
        scale = min(width / 470., height / 340.)
        ox, oy = (width - 470 * scale) / 2, (height - 340 * scale) / 2

        def coords(points):
            return [v for x, y in points for v in (ox+x*scale, oy+y*scale)]

        def line(points, color, thickness):
            self.create_line(*coords(points), fill=color, width=thickness*scale,
                             capstyle=tk.ROUND, joinstyle=tk.ROUND)

        def text(x, y, label, color="#344454", size=10, bold=False):
            self.create_text(*coords([(x,y)]), text=label, fill=color,
                             font=("맑은 고딕", max(8, round(size*scale)), "bold" if bold else "normal"))

        text(235, 18, "손가락 굽힘 정도", size=12, bold=True)
        text(235, 39, "0% = 편 상태     ·     100% = 굽힌 상태", color="#637587", size=9)
        skin, edge = "#f4dfcd", "#bd9b84"
        # Five fingers and palm form a recognizable open-hand silhouette.
        fingers = [([(193,196),(191,91)],30),
                   ([(235,190),(235,70)],32),
                   ([(278,197),(279,91)],29),
                   ([(313,209),(316,130)],25),
                   ([(177,236),(137,192),(112,160)],32)]
        for points, thickness in fingers:
            line(points, edge, thickness+3)
            line(points, skin, thickness)
        palm = [(176,170),(204,163),(281,169),(318,190),
                (325,233),(306,271),(281,291),(281,313),
                (211,313),(207,283),(178,261),(161,231)]
        self.create_polygon(*coords(palm), smooth=True, splinesteps=24,
                            fill=skin, outline=edge, width=1.5*scale)
        line([(202,231),(229,217),(285,226)], "#d8b9a2", 1.2)
        line([(210,253),(246,261),(285,249)], "#d8b9a2", 1.2)
        # Gauges follow the thumb/index/middle finger axes.
        gauges = [("T", "엄지", (152,211),(117,166),(69,184)),
                  ("I", "검지", (191,162),(191,95),(169,69)),
                  ("M", "중지", (235,160),(235,76),(256,57))]
        for fid, label, base, tip, labelpos in gauges:
            f = self.glove.fingers.get(fid)
            ready = bool(f and f.map.ready and f.map_aligned)
            value = self._clamp01(f.curl) if ready else 0.
            color = "#167fa3" if ready else "#9aa7af"
            line([base,tip], "#ffffff", 18)
            line([base,tip], "#d4dde3", 13)
            if ready and value > 0:
                end = (base[0]+(tip[0]-base[0])*value,
                       base[1]+(tip[1]-base[1])*value)
                line([base,end], color, 13)
            text(*labelpos, label, size=10, bold=True)
            bx, by = {"T": (74,211), "I": (175,184), "M": (249,184)}[fid]
            status = f"{value*100:.0f}%" if ready else "미측정" if not f or not f.map.ready else "0점 필요"
            text(bx,by,status,color,size=10,bold=True)
        text(365,108,"약지",color="#8a969f",size=9)
        text(368,128,"새끼",color="#8a969f",size=9)
        text(365,149,"측정 대상 아님",color="#8a969f",size=8)
        text(245,294,"G1 · 환자" if self.glove.short == "G1" else "G2 · 치료사",
             color="#7e6655",size=10,bold=True)
        text(235,329,"캘리브레이션 맵 기준 추정값",color="#637587",size=8)


class CompactMapView(tk.Canvas):
    # 한 손가락 tendon map + 현재점 (run 탭용 compact)

    def __init__(self, parent, finger, width=350, height=125):
        super().__init__(
            parent, width=width, height=height,
            background="white", highlightthickness=1,
            highlightbackground="#d0d0d0"
        )
        self.finger = finger
        self.W = width
        self.H = height

    def update_view(self):
        self.delete("all")
        f = self.finger
        W, H = self.W, self.H
        L, R, T, B = 38, 10, 24, 22

        self.create_text(7, 6, anchor="nw",
                         text=f"{f.fname} | curl {f.curl:.3f}",
                         font=("", 9, "bold"))

        if not f.map.ready:
            self.create_text(W/2, H/2,
                             text="Tendon map 없음\n지도 불러오기 또는 Sweep 필요",
                             justify="center", fill="#888")
            return

        raw_f = f.map.raw_flexion
        raw_e = f.map.raw_extension

        xs_f = [p[2] - f.map._e0 for p in raw_f]
        ys_f = [p[1] - f.map._f0 for p in raw_f]

        xs_e = [p[2] - f.map._e0 for p in raw_e]
        ys_e = [p[1] - f.map._f0 for p in raw_e]

        cx = f.ext.deg - f.map._e0
        cy = f.flex.deg - f.map._f0

        all_x = xs_f + xs_e + [cx]
        all_y = ys_f + ys_e + [cy]
        xmin, xmax = min(all_x), max(all_x)
        ymin, ymax = min(all_y), max(all_y)

        if abs(xmax - xmin) < 1.0:
            xmin -= 1.0
            xmax += 1.0
        if abs(ymax - ymin) < 1.0:
            ymin -= 1.0
            ymax += 1.0

        padx = 0.08 * (xmax - xmin)
        pady = 0.08 * (ymax - ymin)
        xmin -= padx
        xmax += padx
        ymin -= pady
        ymax += pady

        def sx(x):
            return L + (x - xmin) / (xmax - xmin) * (W - L - R)

        def sy(y):
            return H - B - (y - ymin) / (ymax - ymin) * (H - T - B)

        self.create_line(L, T, L, H-B, fill="#b8b8b8")
        self.create_line(L, H-B, W-R, H-B, fill="#b8b8b8")

        pts_f = []
        for x, y in zip(xs_f, ys_f):
            pts_f.extend([sx(x), sy(y)])
        if len(pts_f) >= 4:
            self.create_line(
                *pts_f,
                width=3,
                fill="#1f77b4",
                smooth=True
            )

        pts_e = []
        for x, y in zip(xs_e, ys_e):
            pts_e.extend([sx(x), sy(y)])
        if len(pts_e) >= 4:
            self.create_line(
                *pts_e,
                width=3,
                fill="#ff7f0e",
                smooth=True
            )

        self.create_text(
            L + 4, T + 4,
            anchor="nw",
            text="파랑: Flexion 평균   주황: Extension 평균",
            fill="#555",
            font=("", 7)
        )

        px, py = sx(cx), sy(cy)
        self.create_oval(px-5, py-5, px+5, py+5,
                         fill="#d62728", outline="white", width=1)
        self.create_text(px+7, py-5, anchor="sw", text="현재",
                         fill="#d62728", font=("", 8, "bold"))

        tx = ty = None
        if f.driven == "flex" and f.tgt_flex is not None:
            tx = cx
            ty = f.tgt_flex - f.map._f0
        elif f.driven == "ext" and f.tgt_ext is not None:
            tx = f.tgt_ext - f.map._e0
            ty = cy

        if tx is not None and ty is not None:
            tpx, tpy = sx(tx), sy(ty)
            self.create_oval(
                tpx-5, tpy-5, tpx+5, tpy+5,
                outline="#111111", width=2
            )
            self.create_text(
                tpx+7, tpy+6, anchor="nw",
                text="목표", fill="#111111",
                font=("", 7, "bold")
            )

        self.create_text(W-8, 7, anchor="ne",
                         text=f"F {f.flex.deg:+.1f}° / E {f.ext.deg:+.1f}°",
                         font=("Consolas", 8), fill="#555")



class SweepLiveView(tk.Canvas):
    # 실시간 Sweep 기록
    # x축: extension encoder 절대각도, y축: flex encoder 절대각도
    # 색 궤적과 완료 map 모두 동일한 absolute encoder 좌표계 사용

    COLORS = ["#7f7f7f", "#b0b0b0",
              "#1f77b4", "#ff7f0e", "#2ca02c", "#9467bd",
              "#d62728", "#8c564b"]

    def __init__(self, parent, ctl, width=760, height=430):
        super().__init__(
            parent, width=width, height=height,
            background="white", highlightthickness=1,
            highlightbackground="#d0d0d0"
        )
        self.ctl = ctl
        self.W = width
        self.H = height

    def _segments(self, samples):
        segs = []
        if not samples:
            return segs

        cur_dir = samples[0][0]
        cur = []
        for d, fd, ed in samples:
            if d != cur_dir and cur:
                segs.append((cur_dir, cur))
                cur = []
                cur_dir = d
            cur.append((fd, ed))
        if cur:
            segs.append((cur_dir, cur))
        return segs

    def update_view(self):
        self.delete("all")
        W, H = self.W, self.H

        # 제목/범례를 두 줄로 완전히 분리
        L, R, T, B = 58, 20, 88, 36

        group = self.ctl.sweep
        sw = group.children[0] if group.children else group
        f = sw.finger if sw.finger is not None else self.ctl.cal_finger

        self.create_text(12, 14, anchor="w",
                         text="실시간 Sweep 기록",
                         font=("", 10, "bold"))

        if f is None:
            self.create_text(W/2, H/2,
                             text="선택된 손가락이 없습니다.",
                             fill="#888")
            return

        self.create_text(W-12, 14, anchor="e",
                         text=f"{f.name}   state={sw.state}",
                         fill="#555", font=("Consolas", 9))

        segs = self._segments(sw.samples)

        # 범례 전용 2번째 줄
        legend_y = 40
        legend_x = 78
        gap = 118
        for i, (d, seg) in enumerate(segs):
            color = self.COLORS[i % len(self.COLORS)]
            raw_cycle = i // 2
            motion = "굽힘" if d == +1 else "펴기"
            if raw_cycle < WARMUP_CYCLES:
                label = f"Warm-up {motion}"
            else:
                label = f"{raw_cycle - WARMUP_CYCLES + 1}회 {motion}"

            x0 = legend_x + (i % 6) * gap
            y0 = legend_y + (i // 6) * 20

            self.create_rectangle(x0, y0-5, x0+10, y0+5,
                                  fill=color, outline=color)
            self.create_text(x0+15, y0, anchor="w",
                             text=label, fill="#444",
                             font=("", 8))

        # 실시간 sample: absolute encoder 좌표
        live_pts = [(ed, fd) for _, fd, ed in sw.samples]

        avg_flex_pts = []
        avg_ext_pts = []

        if f.map.ready:
            avg_flex_pts = [
                (ed, fd)
                for _, fd, ed in f.map.raw_flexion
            ]
            avg_ext_pts = [
                (ed, fd)
                for _, fd, ed in f.map.raw_extension
            ]

        cur_x = f.ext.deg
        cur_y = f.flex.deg

        combined = (
            live_pts
            + avg_flex_pts
            + avg_ext_pts
            + [(cur_x, cur_y)]
        )

        if len(combined) <= 1 and not sw.samples:
            self.create_text(
                W/2, H/2,
                text="Sweep를 시작하면\n각 왕복의 encoder 궤적이 실시간으로 표시됩니다.",
                justify="center", fill="#888"
            )
            return

        xs = [p[0] for p in combined]
        ys = [p[1] for p in combined]
        xmin, xmax = min(xs), max(xs)
        ymin, ymax = min(ys), max(ys)

        if abs(xmax-xmin) < 1.0:
            xmin -= 1.0
            xmax += 1.0
        if abs(ymax-ymin) < 1.0:
            ymin -= 1.0
            ymax += 1.0

        padx = 0.08 * (xmax-xmin)
        pady = 0.08 * (ymax-ymin)
        xmin -= padx
        xmax += padx
        ymin -= pady
        ymax += pady

        def sx(x):
            return L + (x-xmin)/(xmax-xmin) * (W-L-R)

        def sy(y):
            return H-B - (y-ymin)/(ymax-ymin) * (H-T-B)

        # axes
        self.create_line(L, T, L, H-B, fill="#b0b0b0")
        self.create_line(L, H-B, W-R, H-B, fill="#b0b0b0")
        self.create_text(W/2, H-11,
                         text="Extension encoder θ [°]",
                         fill="#666", font=("", 8))
        self.create_text(18, (T+H-B)/2,
                         text="Flex\nencoder\nθ [°]",
                         justify="center", fill="#666", font=("", 8))

        # 완료된 directional average tendon maps
        if avg_flex_pts:
            bg = []
            for x, y in avg_flex_pts:
                bg.extend([sx(x), sy(y)])
            if len(bg) >= 4:
                self.create_line(
                    *bg,
                    fill="#1f77b4",
                    width=4,
                    smooth=True
                )

        if avg_ext_pts:
            bg = []
            for x, y in avg_ext_pts:
                bg.extend([sx(x), sy(y)])
            if len(bg) >= 4:
                self.create_line(
                    *bg,
                    fill="#ff7f0e",
                    width=4,
                    smooth=True
                )

        if avg_flex_pts or avg_ext_pts:
            self.create_text(
                W-15, T+8,
                anchor="ne",
                text=(
                    "파랑 = Flexion 1·2·3회 평균   "
                    "주황 = Extension 1·2·3회 평균"
                ),
                fill="#555",
                font=("", 8, "bold")
            )

        # 각 왕복 실시간 궤적
        for i, (d, seg) in enumerate(segs):
            color = self.COLORS[i % len(self.COLORS)]

            pts = []
            for fd, ed in seg:
                pts.extend([sx(ed), sy(fd)])

            if len(pts) >= 4:
                self.create_line(*pts, fill=color, width=2)

            for fd, ed in seg:
                px, py = sx(ed), sy(fd)
                self.create_oval(px-1.5, py-1.5, px+1.5, py+1.5,
                                 fill=color, outline=color)

        # 현재점
        px, py = sx(cur_x), sy(cur_y)
        self.create_oval(px-5, py-5, px+5, py+5,
                         fill="#d62728", outline="white", width=1)
        self.create_text(px+8, py-6, anchor="sw",
                         text="현재", fill="#d62728",
                         font=("", 8, "bold"))

        if sw.msg:
            self.create_text(W-14, H-B-7, anchor="se",
                             text=sw.msg, fill="#666", font=("", 8))


class App:

    def __init__(self, root, ctl: ControlThread):
        self.root = root
        self.ctl = ctl
        root.title("손가락별 Auto-tensioning / 양방향 제어 v32 - validation")
        width = min(1480, max(800, root.winfo_screenwidth() - 80))
        height = min(980, max(540, root.winfo_screenheight() - 80))
        root.geometry(f"{width}x{height}")
        root.minsize(min(1000, width), min(620, height))
        try:
            root.state("zoomed")
        except Exception:
            pass

        root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.log_file = None
        self.log_writer = None
        self.fpanel = {}
        self.map_views = {}
        self.hand_views = {}
        self.pvars = {}
        self.cal_pvars = {}
        self.applied_labels = {}

        self._build()
        self._poll()

    def _finger_list(self):
        items = []
        for g in self.ctl.gloves:
            for fid, f in g.fingers.items():
                items.append((f"{g.short}-{FINGER_NAMES[fid]}", f))
        return items

    def _sel_finger(self):
        for g in self.ctl.gloves:
            if g.short == self.cal_var.get():
                return next(iter(g.fingers.values()), None)
        return None

    def _build(self):
        pad = dict(padx=8, pady=4)
        top = ttk.Frame(self.root)
        top.pack(fill="x", **pad)
        mode = "하드웨어" if DAQ_AVAILABLE else "하드웨어 미연결"
        nf = sum(len(g.fingers) for g in self.ctl.gloves)
        ttk.Label(top, text=f"{mode}   지령 상한 {T_MAX:.1f} N   손가락 {nf}개",
                  foreground="#087b3c").pack(side="left")
        self.lbl_hz = ttk.Label(top, text="-- Hz")
        self.lbl_hz.pack(side="right")

        # Pack fixed controls BEFORE the expanding notebook so even small
        # windows always reserve space for stop and release.
        safety = ttk.Frame(self.root)
        safety.pack(fill="x", padx=8, pady=4)
        self.btn_estop = tk.Button(
            safety, text="■ 비상정지 (Space / Esc)", command=self._emergency_stop,
            bg="#c62828", fg="white", activebackground="#a51f1f",
            activeforeground="white", font=("", 11, "bold"), height=2)
        self.btn_estop.pack(side="left", fill="x", expand=True)
        self.btn_clear_estop = ttk.Button(
            safety, text="비상정지 해제", command=self._clear_estop)
        self.btn_clear_estop.pack(side="left", padx=8, ipady=8)
        ttk.Button(safety, text="전체 구동 해제", command=self._stop_controls
                   ).pack(side="left", padx=4, ipady=8)
        ttk.Button(safety, text="0점 재설정", command=self._zero_all_encoders
                   ).pack(side="left", padx=4, ipady=8)
        self.banner = tk.Label(self.root, text="", font=("", 10, "bold"), height=1)
        self.banner.pack(fill="x", padx=8)

        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill="both", expand=True, padx=8, pady=4)
        self.tab_cal = self._scroll_tab("1. 캘리브레이션")
        self.tab_run = self._scroll_tab("2. Auto-tensioning / 양방향 제어")
        self.notebook.bind("<<NotebookTabChanged>>",
            lambda e: self._tab_changed(self.notebook.index(self.notebook.select())))
        self._build_cal_tab()
        self._build_run_tab()
        self.root.bind("<space>", self._emergency_stop)
        self.root.bind("<Escape>", self._emergency_stop)

    def _scroll_tab(self, title):
        page = ttk.Frame(self.notebook)
        page.rowconfigure(0, weight=1)
        page.columnconfigure(0, weight=1)
        canvas = tk.Canvas(page, highlightthickness=0)
        vertical = ttk.Scrollbar(page, orient="vertical", command=canvas.yview)
        horizontal = ttk.Scrollbar(page, orient="horizontal", command=canvas.xview)
        canvas.configure(yscrollcommand=vertical.set, xscrollcommand=horizontal.set)
        canvas.grid(row=0, column=0, sticky="nsew")
        vertical.grid(row=0, column=1, sticky="ns")
        horizontal.grid(row=1, column=0, sticky="ew")
        content = ttk.Frame(canvas)
        window = canvas.create_window((0, 0), window=content, anchor="nw")

        def resize(event=None):
            canvas.itemconfigure(window, width=max(canvas.winfo_width(),
                                                    content.winfo_reqwidth()))
            canvas.configure(scrollregion=canvas.bbox("all"))

        def wheel(event):
            if not str(event.widget).startswith(str(page)):
                return
            if getattr(event, "num", None) in (4, 5):
                units = -1 if event.num == 4 else 1
            else:
                delta = getattr(event, "delta", 0)
                if not delta:
                    return
                units = -max(1, abs(int(delta / 120))) if delta > 0 else max(1, abs(int(delta / 120)))
            canvas.yview_scroll(units, "units")
            return "break"

        content.bind("<Configure>", resize)
        canvas.bind("<Configure>", resize)
        for event in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
            canvas.bind_all(event, wheel, add="+")
        self.notebook.add(page, text=title)
        return content

    def _build_cal_tab(self):
        p = self.tab_cal
        pad = dict(padx=8, pady=4)

        setup = ttk.LabelFrame(p, text="Sweep 캘리브레이션")
        setup.pack(fill="x", **pad)

        ttk.Label(setup, text="대상 장갑").grid(row=0, column=0, sticky="e", **pad)

        labels = [g.short for g in self.ctl.gloves]
        self.cal_var = tk.StringVar(value=labels[0] if labels else "")
        cb = ttk.Combobox(setup, textvariable=self.cal_var,
                          values=labels, width=13, state="readonly")
        cb.grid(row=0, column=1, sticky="w", **pad)
        cb.bind("<<ComboboxSelected>>", lambda e: self._pick_cal_finger())

        cal_items = [
            ("T_CAL_F", "Flex 장력 [N]"),
            ("T_CAL_E", "Ext 장력 [N]"),
            ("T_PRE", "Take-up 장력 [N]"),
            ("T_PAYOUT", "Flex 풀기 [N]"),
            ("T_PAYOUT_E", "Ext 풀기 [N]"),
        ]
        for j, (k, label) in enumerate(cal_items):
            ttk.Label(setup, text=label).grid(row=0, column=2+j*2, sticky="e", **pad)
            v = tk.StringVar(value=str(DEFAULT_PARAMS[k]))
            ent = ttk.Entry(setup, textvariable=v, width=7, justify="right")
            ent.grid(row=0, column=3+j*2, sticky="w", **pad)
            ent.bind("<Return>", lambda ev, kk=k: self._apply_cal_param(kk))
            self.cal_pvars[k] = v

        ttk.Button(setup, text="장력 적용",
                   command=self._apply_all_cal_params
                   ).grid(row=0, column=12, sticky="ew", **pad)

        ctrl = ttk.Frame(p)
        ctrl.pack(fill="x", **pad)

        self.btn_sweep = tk.Button(
            ctrl, text=f"▶ Sweep 시작 (Warm-up {WARMUP_CYCLES} + 기록 {SWEEP_CYCLES}회)",
            bg="#1565c0", fg="white", font=("", 11, "bold"),
            command=self._start_sweep
        )
        self.btn_sweep.pack(side="left")
        ttk.Button(ctrl, text="Sweep 중단",
                   command=self._stop_sweep
                   ).pack(side="left", padx=6)
        ttk.Button(
            ctrl,
            text="전체 엔코더 0점",
            command=self._zero_all_encoders
        ).pack(side="left", padx=(2, 8))
        self.lbl_sweep = ttk.Label(ctrl, text="대기", foreground="#666")
        self.lbl_sweep.pack(side="left", padx=10)

        live = ttk.LabelFrame(p, text="실시간 엔코더 / 상태")
        live.pack(fill="x", **pad)

        self.lbl_cal_name = ttk.Label(live, text="-", font=("", 10, "bold"))
        self.lbl_cal_name.grid(row=0, column=0, sticky="w", **pad)

        ttk.Label(live, text="Flex encoder").grid(row=1, column=0, sticky="e", **pad)
        self.lbl_cal_flex = ttk.Label(live, text="+0.0°", width=12)
        self.lbl_cal_flex.grid(row=1, column=1, sticky="w", **pad)

        ttk.Label(live, text="Extension encoder").grid(row=1, column=2, sticky="e", **pad)
        self.lbl_cal_ext = ttk.Label(live, text="+0.0°", width=12)
        self.lbl_cal_ext.grid(row=1, column=3, sticky="w", **pad)

        self.lbl_map_sum = ttk.Label(live, text="지도 없음", foreground="#666")
        self.lbl_map_sum.grid(row=2, column=0, columnspan=6, sticky="w", **pad)

        plot_frame = ttk.LabelFrame(p, text="실시간 Sweep plot")
        plot_frame.pack(fill="both", expand=True, **pad)
        self.sweep_view = SweepLiveView(plot_frame, self.ctl, width=860, height=500)
        self.sweep_view.pack(fill="both", expand=True, padx=6, pady=6)

        self._pick_cal_finger()
        ttk.Button(ctrl, text="기록 시작/정지", command=self._toggle_log).pack(side="left", padx=6)

    def _build_run_tab(self):
        p = self.tab_run
        pad = dict(padx=8, pady=4)
        self._build_control_params(p)
        modes = ttk.Frame(p)
        modes.pack(fill="x", padx=8)
        ttk.Label(modes, text="Coupling mode (experimental):").pack(side="left")
        self.coupling_mode_var = tk.StringVar(value="force")
        combo = ttk.Combobox(modes, textvariable=self.coupling_mode_var,
                             values=("force", "position_velocity"),
                             state="readonly", width=20)
        combo.pack(side="left", padx=8)
        combo.bind("<<ComboboxSelected>>", self._change_coupling_mode)
        ttk.Label(p, text="Auto-tensioning: 슬랙 PID + 기본장력  |  양방향: 상대 장갑과의 결합력  |  체크 해제·탭 전환·0점 후에도 적용 게인은 유지됩니다.",
                  foreground="#444").pack(anchor="w", padx=10, pady=4)
        top = ttk.Frame(p)
        top.pack(fill="x", **pad)
        top.columnconfigure(0, weight=1)
        top.columnconfigure(1, weight=1)
        for col, g in enumerate(self.ctl.gloves):
            lf = ttk.LabelFrame(top, text=g.name)
            lf.grid(row=0, column=col, sticky="nsew", padx=4)
            sv = StatusBarsView(lf, g, width=470, height=340)
            sv.pack(fill="x", padx=5, pady=5)
            self.hand_views[g.short] = sv
            rows = ttk.Frame(lf)
            rows.pack(fill="x", padx=5, pady=(0, 6))
            for col_idx, (label, width) in enumerate((
                    ("손가락", 5), ("Auto-tensioning", 14), ("양방향", 6),
                    ("Map", 17), ("curl", 6), ("지령 F/E [N]", 14))):
                ttk.Label(rows, text=label, width=width).grid(row=0, column=col_idx)
            for r, fid in enumerate(("T", "I", "M"), start=1):
                fu = g.fingers.get(fid)
                if fu is None:
                    continue
                w = {}
                ttk.Label(rows, text=FINGER_NAMES[fid], width=5).grid(row=r, column=0)
                for col_idx, kind in enumerate(("auto", "coupling"), start=1):
                    w[kind] = tk.BooleanVar(value=False)
                    ttk.Checkbutton(rows, variable=w[kind],
                        command=lambda fu=fu, w=w, kind=kind: self._enable(fu, w, kind)
                        ).grid(row=r, column=col_idx)
                map_cell = ttk.Frame(rows)
                map_cell.grid(row=r, column=3, sticky="w")
                w["map"] = ttk.Label(map_cell, text="없음", width=5, foreground="#a15d00")
                w["map"].pack(side="left")
                ttk.Button(map_cell, text="지도 불러오기",
                    command=lambda fu=fu: self._load_map_for_finger(fu)
                    ).pack(side="left", padx=2)
                w["curl"] = ttk.Label(rows, text="0.000", width=6, anchor="e")
                w["curl"].grid(row=r, column=4)
                w["force"] = ttk.Label(rows, text="0.00 / 0.00", width=14, anchor="e")
                w["force"].grid(row=r, column=5)
                self.fpanel[fu.key] = w

        self.lbl_cpl = ttk.Label(p, text="")
        self.lbl_cpl.pack(anchor="w", padx=10, pady=3)
        maps = ttk.LabelFrame(p, text="Tendon map / 현재 손가락 위치")
        maps.pack(fill="x", **pad)
        maps.columnconfigure(0, weight=1)
        maps.columnconfigure(1, weight=1)
        for col, g in enumerate(self.ctl.gloves):
            gf = ttk.LabelFrame(maps, text=g.name)
            gf.grid(row=0, column=col, sticky="nsew", padx=4, pady=4)
            for r, fid in enumerate(("T", "I", "M")):
                fu = g.fingers.get(fid)
                if fu is None:
                    continue
                mv = CompactMapView(gf, fu, width=360, height=130)
                mv.grid(row=r, column=0, sticky="ew", padx=4, pady=3)
                self.map_views[fu.key] = mv

    def _build_control_params(self, parent):
        params = ttk.LabelFrame(parent, text="제어 파라미터 — 입력 아래의 '적용' 값은 제어 루프가 사용한 값")
        params.pack(fill="x", padx=8, pady=4)
        for i, k in enumerate(CONTROL_PARAM_LIMITS):
            cell = ttk.Frame(params)
            cell.grid(row=i // 7, column=i % 7, padx=5, pady=3, sticky="w")
            ttk.Label(cell, text=k).grid(row=0, column=0, sticky="w")
            v = tk.StringVar(value=str(self.ctl.params[k]))
            e = ttk.Entry(cell, textvariable=v, width=7, justify="right")
            e.grid(row=0, column=1, padx=(3, 0))
            e.bind("<Return>", lambda ev, kk=k: self._apply_param(kk))
            self.pvars[k] = v
            lab = ttk.Label(cell, text="적용: 대기", foreground="#555", font=("", 8))
            lab.grid(row=1, column=0, columnspan=2, sticky="w")
            self.applied_labels[k] = lab
        btns = ttk.Frame(params)
        n_rows = (len(CONTROL_PARAM_LIMITS) + 6) // 7
        btns.grid(row=n_rows, column=0, columnspan=7, sticky="ew", padx=5, pady=5)
        self.btn_apply_params = ttk.Button(btns, text="파라미터 전체 적용",
                                          command=self._apply_all_params)
        self.btn_apply_params.pack(side="left")
        ttk.Button(btns, text="전체 엔코더 0점", command=self._zero_all_encoders
                   ).pack(side="left", padx=8)
        self.btn_log = ttk.Button(btns, text="기록 시작", command=self._toggle_log)
        self.btn_log.pack(side="left", padx=8)
        ttk.Label(btns, text="Trial ID").pack(side="left", padx=(12, 2))
        self.trial_var = tk.StringVar(value="PILOT_01")
        ttk.Entry(btns, textvariable=self.trial_var, width=18).pack(side="left")
        ttk.Button(btns, text="이벤트 표시", command=self._mark_event).pack(side="left", padx=4)
        self.lbl_log = ttk.Label(btns, text="", foreground="#666")
        self.lbl_log.pack(side="left", padx=6)
        self.lbl_params = ttk.Label(params, text="", foreground="#555")
        self.lbl_params.grid(row=n_rows + 1, column=0, columnspan=7, sticky="w", padx=5, pady=3)
        ttk.Label(params, text="입력 → 전체 적용(또는 해당 칸 Enter). 켜진 상태에서도 적용 가능하며, 적용만으로 체크가 켜지지는 않습니다.",
                  foreground="#555").grid(row=n_rows + 2, column=0, columnspan=7, sticky="w", padx=5, pady=3)

    def _change_coupling_mode(self, event=None):
        try:
            self.ctl.set_coupling_mode(self.coupling_mode_var.get())
        except ValueError as exc:
            self.coupling_mode_var.set(self.ctl.coupling_mode)
            messagebox.showwarning("Coupling mode", str(exc))

    def _zero_all_encoders(self):
        c = self.ctl
        with c.lock:
            if c.estop:
                messagebox.showwarning("0점", "비상정지 해제 후 0점을 설정하세요.")
                return
            if c.run_zero_active or c.run_zero_req:
                return
            in_run = c.mode == "run"
            c.stop_all()
            self._sync_enable_widgets_off()
            if in_run:
                c.request_run_zero()
            else:
                c.request_zero()

    def _tab_changed(self, idx):
        with self.ctl.lock:
            self.ctl.stop_all()
            self.ctl.mode = "map" if idx == 0 else "run"
        self._sync_enable_widgets_off()

    def _pick_cal_finger(self):
        f = self._sel_finger()
        if f is None:
            return
        with self.ctl.lock:
            self.ctl.stop_all()
            self.ctl.mode = "map"
            self.ctl.cal_finger = f
        self._sync_enable_widgets_off()
        self.lbl_cal_name.config(text=f.name)
        self.lbl_map_sum.config(text=f.map.summary())

    def _start_sweep(self):
        f = self._sel_finger()
        if f is None:
            return
        glove = next(g for g in self.ctl.gloves if g.short == f.glove_short)
        if any(unit.single for unit in glove.fingers.values()):
            messagebox.showwarning("Sweep", f"{f.name}: 엔코더 2개가 모두 필요합니다")
            return
        if self.ctl.estop:
            messagebox.showwarning("Sweep", "비상정지 상태입니다")
            return
        if not self._apply_all_cal_params():
            return
        self.ctl.start_sweep(f)
        self._sync_enable_widgets_off()

    def _stop_sweep(self):
        with self.ctl.lock:
            self.ctl.sweep.abort("사용자 중단")
            self.ctl.stop_all()
            self.ctl.mode = "map"
        self._sync_enable_widgets_off()

    def _load_map_for_finger(self, f):
        self._stop_controls()
        path = filedialog.askopenfilename(
            title=f"{f.name} tendon map 불러오기",
            initialdir=MAP_DIR if os.path.isdir(MAP_DIR) else ".",
            filetypes=[("Tendon map JSON", "*.json")])
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as fp:
                payload = json.load(fp)
            if payload.get("format") == "glove_tendon_maps_v1":
                if payload.get("glove") != f.glove_short:
                    raise ValueError("Glove mismatch (G1/G2)")
                glove = next(g for g in self.ctl.gloves if g.short == f.glove_short)
                candidates = {}
                for fid, unit in glove.fingers.items():
                    candidate = TendonMap()
                    if not candidate.load_data(payload["fingers"][fid], unit.key):
                        raise ValueError(f"{fid}: invalid map")
                    candidates[fid] = candidate
                with self.ctl.lock:
                    for fid, candidate in candidates.items():
                        glove.fingers[fid].map = candidate
                        glove.fingers[fid].map_aligned = False
                return
            candidate = TendonMap()
            ok = candidate.load(path)
            if not ok:
                raise ValueError("점이 부족하거나 형식이 맞지 않습니다.")
        except Exception as e:
            messagebox.showerror("지도 불러오기", str(e))
            return
        with self.ctl.lock:
            f.map = candidate
            f.map_aligned = False
        if f is self.ctl.cal_finger:
            self.lbl_map_sum.config(text=f.map.summary())

    def _enable(self, fu, w, kind):
        on = w[kind].get()
        error = self.ctl.set_control_enabled(fu, kind, on)
        if error:
            w[kind].set(fu.auto_enabled if kind == "auto" else fu.coupling_enabled)
            messagebox.showwarning("구동", error)

    def _sync_enable_widgets_off(self):
        for w in self.fpanel.values():
            w["auto"].set(False)
            w["coupling"].set(False)

    def _stop_controls(self):
        self.ctl.stop_all()
        self._sync_enable_widgets_off()

    def _emergency_stop(self, event=None):
        self.ctl.trigger_estop("사용자")
        self._sync_enable_widgets_off()
        return "break"

    def _clear_estop(self):
        if not self.ctl.clear_estop():
            messagebox.showerror("제어 오류", "제어 스레드가 종료되었습니다. 원인 확인 후 프로그램을 다시 시작하세요.\n" + self.ctl.control_error)
        self._sync_enable_widgets_off()

    def _apply_values(self, variables, keys):
        try:
            values = {}
            for k in keys:
                try:
                    values[k] = float(variables[k].get())
                except (ValueError, TypeError):
                    raise ValueError(f"{k}: 숫자를 입력하세요.") from None
            self.ctl.set_params(values)
        except ValueError as exc:
            messagebox.showerror("파라미터 적용 실패", f"{exc}\n이번 적용에서는 어떤 값도 변경하지 않았습니다.")
            return False
        for k, v in values.items():
            variables[k].set(str(v))
        return True

    def _apply_cal_param(self, k):
        return self._apply_values(self.cal_pvars, (k,))

    def _apply_all_cal_params(self):
        return self._apply_values(self.cal_pvars, tuple(self.cal_pvars))

    def _apply_param(self, k):
        return self._apply_values(self.pvars, (k,))

    def _apply_all_params(self):
        return self._apply_values(self.pvars, tuple(self.pvars))

    def _refresh_param_status(self):
        requested, version, active, active_version = self.ctl.param_snapshot()
        dirty = []
        for k, variable in self.pvars.items():
            try:
                val = float(variable.get())
                changed = not math.isfinite(val) or val != requested[k]
            except (ValueError, TypeError):
                changed = True
            if changed:
                dirty.append(k)
            self.applied_labels[k].config(
                text=f"적용: {active[k]}" if active_version >= 0 else "적용: 대기",
                foreground="#b15b00" if changed else "#176b35")
        if self.ctl.control_error:
            text = "제어 오류 — 프로그램 재시작 필요"
            color = "#c62828"
        elif dirty:
            text = "미적용 입력: " + ", ".join(dirty) + " — 전체 적용 또는 해당 칸 Enter"
            color = "#b15b00"
        elif active_version != version:
            text = f"전체 입력 접수 #{version} — 제어 루프 반영 대기"
            color = "#b15b00"
        else:
            text = f"적용 완료 #{version} — 입력값과 제어 루프 값이 일치합니다."
            color = "#176b35"
        self.lbl_params.config(text=text, foreground=color)

    def _log_header(self):
        h = ["t_s", "KC1", "BC1", "KC2", "BC2", "OVER", "T_BASE"]
        for f1, f2 in self.ctl.pairs():
            k = f1.fid
            h += [
                f"{k}_c1", f"{k}_c2", f"{k}_err",
                f"{k}_g1cf", f"{k}_g1ce",
                f"{k}_g2cf", f"{k}_g2ce",
                f"{k}_g1fN", f"{k}_g1eN",
                f"{k}_g2fN", f"{k}_g2eN",
                f"{k}_g1fdeg", f"{k}_g1edeg",
                f"{k}_g2fdeg", f"{k}_g2edeg",
                f"{k}_mot1", f"{k}_mot2"
            ]
        h += ["energy"]
        h += ["params_version"] + list(LOG_TUNING_KEYS)
        for g in self.ctl.gloves:
            for f in g.fingers.values():
                h += [f"{f.key}_auto_on", f"{f.key}_coupling_on"]
        h += ["unix_time_s", "dt_s", "instant_hz", "mode", "sweep_state",
              "sweep_cycle", "estop", "estop_message", "trial_id",
              "event", "mean_curl_error", "max_coupling_cmd_N"]
        for gs in ("G1", "G2"):
            for fid in FINGER_IDS:
                prefix = f"{gs}_{fid}"
                h += [f"{prefix}_{v}" for v in (
                    "curl", "curl_vel", "source", "driven", "motion", "map_ready",
                    "map_aligned", "target_flex_deg", "target_ext_deg",
                    "slack_err_flex_deg", "slack_err_ext_deg", "integral_N",
                    "auto_soft", "coupling_soft", "coupling_flex_cmd_N",
                    "coupling_ext_cmd_N")]
                for direction in ("flex", "ext"):
                    h += [f"{prefix}_{direction}_{v}" for v in (
                        "raw_encoder_deg", "encoder_deg", "encoder_vel_dps",
                        "target_cmd_N", "applied_cmd_N", "estimated_AO_V")]
        h += ["coupling_mode"]
        for gs in ("G1", "G2"):
            for fid in FINGER_IDS:
                h += [f"{gs}_{fid}_{v}" for v in
                      ("curl_raw_flex", "curl_raw_ext", "curl_target", "curl_blending")]
        return h

    def _mark_event(self):
        with self.ctl.lock:
            self.ctl.event_marker = "MANUAL_EVENT"

    def _toggle_log(self):
        if self.log_file:
            with self.ctl.lock:
                self.ctl.logging = False
            self._drain_log()
            self.log_file.close()
            self.log_file = self.log_writer = None
            self.btn_log.config(text="기록 시작")
            self.lbl_log.config(text="")
            return

        with self.ctl.lock:
            self.ctl.trial_id = self.trial_var.get().strip() or "UNLABELED"
            self.ctl.event_marker = "LOG_START"
        name = time.strftime("run_%Y%m%d_%H%M%S.csv")
        self.log_file = open(name, "w", newline="", encoding="utf-8-sig")
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(self._log_header())
        with self.ctl.lock:
            self.ctl.logging = True
        self.btn_log.config(text="기록 정지")
        self.lbl_log.config(text=name, foreground="#087b3c")

    def _drain_log(self):
        try:
            while True:
                row = self.ctl.log_q.get_nowait()
                if self.log_writer:
                    self.log_writer.writerow(
                        [f"{v:.4f}" if isinstance(v, float) else v for v in row]
                    )
        except queue.Empty:
            pass

    def _poll(self):
        c = self.ctl
        self.lbl_hz.config(text=f"{c.hz:.0f} Hz")
        self._refresh_param_status()

        if c.estop:
            self.banner.config(
                text=f"비상정지 — {c.estop_msg}",
                bg="#c62828", fg="white"
            )
            self.btn_clear_estop.state(["!disabled"])
        else:
            if getattr(c, "run_zero_active", False):
                self.banner.config(text=c.run_zero_msg, bg="#fff3cd", fg="#6b5200")
            elif getattr(c, "run_zero_msg", ""):
                self.banner.config(text=c.run_zero_msg, bg="#e8f5e9", fg="#176b35")
            else:
                self.banner.config(text="", bg=self.root.cget("bg"))
            self.btn_clear_estop.state(["disabled"])

        f = c.cal_finger
        if f is not None:
            glove = next(g for g in c.gloves if g.short == f.glove_short)
            units = list(glove.fingers.values())
            self.lbl_cal_name.config(text=f"{glove.short} (plot: {f.name})")
            self.lbl_cal_flex.config(text=" / ".join(f"{u.fid}:{u.flex.deg:+.1f}" for u in units), width=32)
            self.lbl_cal_ext.config(text=" / ".join(f"{u.fid}:{u.ext.deg:+.1f}" for u in units), width=32)
            self.lbl_map_sum.config(text=" | ".join(f"{u.fid}: {u.map.summary()}" for u in units))

        sw = c.sweep
        color = {
            "off": "#666",
            "done": "#087b3c",
            "abort": "#c62828"
        }.get(sw.state, "#1565c0")
        self.lbl_sweep.config(text=sw.msg or "대기", foreground=color)

        self.sweep_view.update_view()

        for hv in self.hand_views.values():
            hv.update_view()

        for g in c.gloves:
            for fu in g.fingers.values():
                w = self.fpanel.get(fu.key)
                if not w:
                    continue
                w["auto"].set(fu.auto_enabled)
                w["coupling"].set(fu.coupling_enabled)
                w["curl"].config(text=f"{fu.curl:.3f}")
                w["force"].config(text=f"{fu.flex.applied_N:.2f} / {fu.ext.applied_N:.2f}")
                w["map"].config(
                    text=("완료" if fu.map_aligned else "0점필요") if fu.map.ready else "없음",
                    foreground="#087b3c" if fu.map.ready else "#a15d00"
                )

        for mv in self.map_views.values():
            mv.update_view()

        self.lbl_cpl.config(
            text=(
                f"평균 결합오차 {c.cpl.err:+.3f}    "
                f"최대 결합력 {c.cpl.force:+.2f} N    "
                f"에너지 {c.cpl.energy:+.4f} J"
            )
        )

        self._drain_log()
        self.root.after(UI_PERIOD_MS, self._poll)

    def on_close(self):
        self.ctl.stop_all()
        self.ctl.running = False
        if self.ctl.is_alive():
            self.ctl.join(timeout=1.0)
        if self.log_file:
            with self.ctl.lock:
                self.ctl.logging = False
            self._drain_log()
            self.log_file.close()
        self.root.destroy()

def main():
    ctl = ControlThread()
    ctl.start()
    root = tk.Tk()
    app = App(root, ctl)
    try:
        root.mainloop()
    finally:
        ctl.running = False
        if ctl.is_alive():
            ctl.join(timeout=1.0)


if __name__ == "__main__":
    main()