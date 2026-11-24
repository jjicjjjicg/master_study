"""
vpatient_run.py — 가상 환자(virtual patient)로 단방향/양방향 느낌 비교

기존 제어 프로그램(1015_v9_glove_cal.py)을 그대로 불러와 실행하되, 환자 장갑(G1) 대신
소프트웨어로 만든 '가상 환자 손가락'을 치료사 장갑(G2)에 결합한다. 제어 파일은 수정하지 않는다.

  양방향 : 가상 환자의 curl → 치료사 장갑에 결합력(KC2, BC2)으로 전달 → 치료사가 느낀다
  단방향 : 치료사 → 환자 방향(KC1, BC1)만 살아 있고, 치료사 장갑에는 결합력이 0
           (auto-tensioning 은 그대로라 장갑은 '투명'하게 따라온다)

가상 환자 손가락 모델 (curl 좌표, 1차 점성 + 스프링들, 암시적 적분이라 게인과 무관하게 안정):
  - 능동 노력   : 시나리오의 목표 curl 을 따라가려는 힘. 굽힘/폄 최대 힘이 따로 있고 포화된다
  - 근긴장      : 안정 자세(rest)로 당기는 스프링. 환자가 힘을 쓸수록 더 뻣뻣해진다
  - 경직        : 손가락마다 등급(0/1/1+/2/3). 속도 의존 신장반사 — 빨리 펼수록 더 굽힌 위치에서
                  굽히는 쪽 반사 힘이 걸린다. 걸린 순간은 정답지(_answer.csv)에 기록된다
  - 물체 접촉   : 물체 두께보다 더 굽히면 단단한 스프링이 막는다 (한쪽 방향 제약)
  - 피로, 떨림  : 프로필에 따라
  ※ 프로필 수치는 느낌을 보기 위한 임의값이다. 임상 데이터에 맞춘 것이 아니다.

사용:
  python vpatient_run.py                          # 같은 폴더의 1015_v9_glove_cal.py 사용
  python vpatient_run.py --control my_control.py  # 다른 제어 파일 지정

절차:
  1) 치료사 장갑(G2)만 착용한다. 환자 장갑(G1)의 제어 체크는 모두 꺼 둔다.
  2) 평소대로 G2 지도 준비 → 0점 초기화 → auto-tensioning 체크.
  3) '가상 환자' 창에서 [가상 환자 사용] 을 켠 뒤, G2 의 양방향(결합) 체크를 켠다.
  4) 프로필·경직 등급·시나리오·방향(양방향/단방향)을 고르고 [시작].
  5) 경직을 느낄 때마다 키보드 1(엄지) / 2(검지) / 3(중지). 정지하면 정답지와 맞춰 감지율이 나온다.
  Space/Esc 비상정지는 메인 창과 이 창 어디서나 동작한다. 비상정지되면 시나리오도 멈춘다.

리듬 포즈 탭:
  노트가 오른쪽에서 다가와 판정선에 닿는 순간 그 포즈(초록 점선)여야 한다. 노트마다 PERFECT/GOOD/MISS,
  타이밍, 손가락별 '환자 기여'(환자가 목표 쪽으로 낸 힘 ÷ 환자+보조 힘)를 _rhythm.csv 에 남긴다.
  보조 방식: 치료사(결합) / 고정 로봇 보조(노트 궤적으로 일정 강성) / 없음. 같은 '순서 번호'면 같은 노트 순서.
  slacking 을 켜면 도움을 받을수록 환자가 스스로 내는 힘이 줄어든다(임의 모델).
"""
import argparse
import csv
import importlib.util
import math
import os
import random
import statistics
import sys
import time
from datetime import datetime

FIDS = ("I", "M", "T")
FNAME = {"I": "검지", "M": "중지", "T": "엄지"}

F2_CAP_DEFAULT = 4.0     # N  치료사 장갑에 전달하는 결합력 상한 (본인 손 통증 기준보다 낮게)
F2_CAP_RANGE = (1.0, 8.0)
F1_CAP_DEFAULT = 5.0     # N  환자 장갑이 환자 손가락에 낼 수 있는 힘 상한 (실제 장갑처럼 무한대가 아니게)
F1_CAP_RANGE = (1.0, 15.0)
BI_RAMP_UP_S = 1.0       # s  단방향 → 양방향 전환 시 힘을 서서히 올린다
BI_RAMP_DOWN_S = 0.5     # s  양방향 → 단방향 전환 시 힘을 서서히 내린다
CURL_RATE_MAX = 2.0      # curl/s  가상 환자 손가락 속도 상한
K_OBJ = 0.25             # N/deg   물체 접촉 강성
GRIP_OK_N = 0.4          # N       이 이상 눌러야 '잡고 있음'으로 본다
SPAN_FALLBACK = 300.0
CURL_CPL_DB = 0.01       # curl  굽힘 차이 결합의 데드밴드
CURL_CPL_MAX = 15.0      # N     계산값 자체의 상한 (실제로는 UI 의 힘 상한이 먼저 걸림)    # deg     지도 기울기를 못 읽을 때의 curl 0→1 스풀 각도


# ───────────────────────────────────────────── 환자 프로필 (임의값)
def _p(**kw):
    base = dict(k_act=0.05,      # N/deg  목표를 따라가려는 능동 강성
                fmax_flex=6.0,   # N      굽힘 최대 능동 힘
                fmax_ext=6.0,    # N      폄 최대 능동 힘
                k_tone=0.002,    # N/deg  근긴장(안정 자세로 당김)
                tone_gain=0.0,   #        힘을 쓸 때 긴장이 커지는 배율 (k_tone × (1 + gain × 노력))
                rest=0.25,       # curl   안정 자세
                b=0.002,         # N·s/deg 손가락 점성 (작게: 경직이 없을 때 끌려오는 저항이 작아야 차이가 보인다)
                catch_vel=9e9,   # curl/s 이보다 빠르게 펴지면 catch
                b_catch=0.0,     # N·s/deg catch 때 더해지는 점성
                tremor=0.0,      # N      떨림 진폭 (5 Hz)
                fatigue_tau=0.0) # s      0 이면 피로 없음
    base.update(kw)
    return base


_HEALTHY = _p()
_WEAK = _p(fmax_flex=1.5, fmax_ext=0.5, k_tone=0.010, rest=0.40)
_FATIGUE = _p(fmax_flex=3.0, fmax_ext=2.0, k_tone=0.006, rest=0.35, fatigue_tau=12.0)
_TREMOR = _p(fmax_flex=3.0, fmax_ext=2.0, k_tone=0.006, rest=0.35, tremor=0.5)

PROFILES = {
    "정상":                  {f: _HEALTHY for f in FIDS},
    "약함(폄 부족)":          {f: _WEAK for f in FIDS},
    "피로(점점 약해짐)":       {f: _FATIGUE for f in FIDS},
    "떨림":                  {f: _TREMOR for f in FIDS},
}

# ───────────────────────────────────────────── 경직 등급 (손가락마다 따로)
# MAS(Modified Ashworth Scale) 등급 이름을 빌린 임의 매핑이다 — 임상 데이터로 보정한 값이 아님.
# 경직 = 속도 의존 신장반사. TSRT 모델(Levin & Feldman)처럼 반사가 켜지는 굽힘 정도(threshold)가
#        펴는 속도에 따라 옮겨 간다:  thr = lam0 + mu × (펴는 속도)
#        굽힘 curl 이 thr 보다 작아지면(=굽힘근이 그 이상 늘어나면) 굽히는 쪽 반사 힘이 생긴다.
#   k_tone/rest/tone_gain : 근긴장(쉬는 자세, 기본 뻣뻣함)
#   lam0 [curl] : 천천히 펼 때 반사가 시작되는 굽힘 정도 (작을수록 끝범위에서만 걸림)
#   mu   [curl per curl/s] : 빨리 펼수록 반사가 얼마나 일찍 걸리나
#   k_sr [N/deg], f_sr [N] : 반사 힘의 기울기와 최대값
SPAS_GRADES = ("0", "1", "1+", "2", "3")
SPAS = {
    "0":  dict(k_tone=0.002, rest=0.25, tone_gain=0.0, lam0=0.0,   mu=0.0,  k_sr=0.0,  f_sr=0.0),
    "1":  dict(k_tone=0.004, rest=0.30, tone_gain=0.3, lam0=-0.15, mu=0.35, k_sr=0.02, f_sr=2.0),
    "1+": dict(k_tone=0.006, rest=0.35, tone_gain=0.6, lam0=-0.05, mu=0.45, k_sr=0.03, f_sr=3.0),
    "2":  dict(k_tone=0.010, rest=0.45, tone_gain=1.0, lam0=0.10,  mu=0.55, k_sr=0.05, f_sr=5.0),
    "3":  dict(k_tone=0.016, rest=0.55, tone_gain=1.4, lam0=0.30,  mu=0.60, k_sr=0.08, f_sr=8.0),
}
SPAS_DEFAULT = {"T": "1", "I": "2", "M": "1+"}
SR_TAU_UP = 0.05     # s   반사 힘이 올라오는 시간 (반사 지연 포함한 근사)
SR_TAU_DOWN = 0.15   # s   반사 힘이 빠지는 시간
SR_ON_N = 0.3        # N   이 이상이면 '경직 발생'(정답지 기록 시작)
SR_OFF_N = 0.1       # N   이 아래로 떨어지면 끝
SR_MERGE_S = 0.3     # s   끝난 뒤 이 안에 다시 걸리면 같은 경직으로 묶는다 (clonus 처럼 끊기는 경우)
CATCH_VEL_MIN = 0.3  # curl/s 이보다 느리게 펴다가 끝범위에서 걸린 것은 'catch' 가 아니라 '끝범위' 저항으로 분류
MARK_KEYS = {"1": "T", "2": "I", "3": "M"}   # 치료사가 '경직 느꼈다' 표시하는 키
MARK_WINDOW = (-0.2, 1.5)   # s  표시 시각 - 경직 시작 이 범위면 '감지'로 인정 (반응 시간)

# ───────────────────────────────────────────── 시나리오
# 단계 = (이름, 지속 s, 목표 curl dict 또는 None(힘 뺌), 물체 두께 curl dict 또는 None)
_OPEN = {"I": 0.05, "M": 0.05, "T": 0.05}
_OBJ = {"I": 0.55, "M": 0.55, "T": 0.45}
_SQUEEZE = {"I": 0.62, "M": 0.62, "T": 0.52}   # 물체보다 조금 더 굽히려 해야 쥐는 힘이 생긴다

SCENARIOS = {
    "물체 잡고 옮기기": [
        ("손 펴기", 3.0, _OPEN, None),
        ("물체로 뻗기 (편 채 유지)", 2.0, _OPEN, None),
        ("잡기", 2.5, _SQUEEZE, _OBJ),
        ("들어 옮기기 (쥔 채 유지)", 5.0, _SQUEEZE, _OBJ),
        ("내려놓고 놓기", 3.0, _OPEN, _OBJ),
        ("휴식 (힘 빼기)", 3.0, None, None),
    ],
    "특정 포즈 취하기": [
        ("손 펴기", 4.0, _OPEN, None),
        ("집게 (엄지+검지)", 5.0, {"I": 0.55, "M": 0.10, "T": 0.50}, None),
        ("손 펴기", 3.0, _OPEN, None),
        ("가리키기 (검지만 폄)", 5.0, {"I": 0.05, "M": 0.80, "T": 0.60}, None),
        ("손 펴기", 3.0, _OPEN, None),
        ("주먹", 5.0, {"I": 0.90, "M": 0.90, "T": 0.70}, None),
        ("세 손가락 모으기", 5.0, {"I": 0.50, "M": 0.50, "T": 0.50}, None),
        ("휴식 (힘 빼기)", 3.0, None, None),
    ],
}
APERTURE_MARGIN = 0.08   # curl  잡으러 갈 때 물체 두께보다 이만큼은 더 펴져 있어야 한다
RELEASE_MARGIN = 0.02    # curl  놓을 때 물체 두께보다 이만큼 더 펴지면 '놓음'
GRASP_LIMIT_DEFAULT = 20.0      # s  물체에 도착한 뒤 벌리기→잡기→옮기기→놓기까지의 제한 시간
GRASP_LIMIT_RANGE = (5.0, 120.0)
OPEN_S, REACH_S, REST_S = 3.0, 2.0, 3.0   # s  시간으로 넘어가는 단계
TRANSPORT_S = 4.0        # s  쥐고 있는 동안에만 흐르는 이동 시간
GRIP_HOLD_S = 0.3        # s  이만큼 계속 쥐고 있어야 '잡음'
SLIP_S = 0.2             # s  이만큼 쥐는 힘이 빠지면 '미끄러짐' (이동 멈춤)

# 화면에 그리는 손 (옆모습). 손가락: (손목 기준 뿌리 x, y, 마디 길이, 마디별 최대 굽힘각[deg], 시작 방향[rad], 굽는 방향)
HAND_GEOM = {
    "I": (100, -30, (64, 42, 30), (62, 78, 50), -0.12, +1),
    "M": (102, -16, (68, 46, 32), (62, 78, 50), -0.06, +1),
    "T": (60, 40, (54, 42), (48, 42), 1.25, -1),
}
MOVE_TIME = 1.5   # s  단계가 바뀔 때 목표가 이 시간에 걸쳐 부드럽게 이동

# ───────────────────────────────────────────── 리듬 포즈 과제
# 정해진 박자마다 손 포즈 노트가 오고, 노트가 판정선에 닿는 순간 그 포즈를 하고 있어야 한다.
# 가상 환자는 노트 시각에 맞춰 포즈를 만들려고 노력하고(흰 점선), 치료사는 필요한 만큼 돕는다.
RHYTHM = "리듬 포즈"
RHYTHM_TEMPI = {"느림 (4 s)": 4.0, "보통 (3 s)": 3.0, "빠름 (2 s)": 2.0}   # 노트 간격
RHYTHM_POSES = {
    "펴기": {"I": 0.05, "M": 0.05, "T": 0.05},
    "주먹": {"I": 0.85, "M": 0.85, "T": 0.65},
    "집게": {"I": 0.55, "M": 0.10, "T": 0.50},
    "가리키기": {"I": 0.05, "M": 0.80, "T": 0.60},
    "세 손가락 모으기": {"I": 0.50, "M": 0.50, "T": 0.50},
}
RHYTHM_LEVELS = {"편 손": 0.08, "반쯤": 0.45, "굽힘": 0.82}   # 켜진 손가락이 하나뿐일 때 쓰는 굽힘 단계
RHYTHM_LEAD_S = 3.0          # s  첫 노트까지 준비 시간
RHYTHM_JUDGE = (-0.15, 0.45) # s  노트 시각 기준 판정 구간 (이 동안의 굽힘 오차 중간값으로 판정)
RHYTHM_PERFECT = 0.10        # curl  모든 손가락 오차가 이하면 PERFECT
RHYTHM_GOOD = 0.20           # curl  이하면 GOOD, 넘으면 MISS
RHYTHM_TIMING_TOL = 0.15     # curl  '포즈에 도착했다'로 보는 오차 (타이밍 계산용)
RHYTHM_MIN_STEP = 0.25       # curl  연속한 두 노트는 적어도 한 손가락이 이만큼 달라야 한다
NOTES_DEFAULT, NOTES_RANGE = 20, (5, 80)
ASSIST_MODES = {"therapist": "치료사 (결합)", "robot": "고정 로봇 보조", "none": "없음 (환자 혼자)"}
K_ROBOT_DEFAULT, K_ROBOT_RANGE = 0.03, (0.005, 0.15)   # N/deg  고정 로봇이 노트 궤적으로 당기는 강성
# slacking: 밖에서 목표 쪽으로 도와주는 힘이 클수록 환자가 스스로 내는 힘(drive)이 줄고, 도움이 없으면 회복한다
SLACK_RECOVER_S = 8.0        # s   도움이 없을 때 drive 가 1 로 돌아오는 시간 상수
SLACK_GAIN = 0.06            # 1/s 도움 SLACK_F_REF 만큼일 때 drive 가 줄어드는 속도
SLACK_F_REF = 2.0            # N
SLACK_MIN = 0.2


def _minjerk(a):
    a = max(0.0, min(1.0, a))
    return a * a * a * (10.0 - 15.0 * a + 6.0 * a * a)


# ───────────────────────────────────────────── 가상 손가락
class VFinger:
    def __init__(self, fid):
        self.fid = fid
        self.prof = _HEALTHY
        self.grade = "0"
        self.slack = False         # slacking 모델 사용
        self.drive = 1.0           # 1 = 스스로 낼 수 있는 힘을 다 냄, 낮을수록 도움에 기대 힘을 덜 씀
        self.reset(0.0)

    def reset(self, curl):
        self.curl = float(curl)
        self.vel = 0.0
        self.fatigue = 1.0        # 1 = 멀쩡, 낮을수록 지침
        self.des = None           # 현재 목표 curl (None = 힘 뺌)
        self.f_act = 0.0          # N  환자 능동 힘 (+굽힘)
        self.f_cpl = 0.0          # N  치료사가 환자에게 주는 힘 (+굽힘)
        self.f_contact = 0.0      # N  물체를 누르는 힘
        self.effort = 0.0         # 0~1 환자가 낼 수 있는 힘 대비 쓰는 정도
        self.f_sr = 0.0           # N  경직(신장반사) 힘 (+굽힘)
        self.sr_thr = None        # curl  지금 펴는 속도에서 반사가 걸리기 시작하는 굽힘 정도
        self.cmd = self.raw = (0.0, 0.0)

    def step(self, dt, t, des, obj, c_t, v_t, kc1, bc1, span, cap1=None):
        """암시적 오일러:  (B + dt·ΣK)·x_new = B·x + dt·(ΣK·x_i + F_const)"""
        pr = self.prof
        x = self.curl * span
        K = KX = Fc = 0.0
        self.des = des

        # 능동 노력 (포화)
        f_act = 0.0
        if des is not None:
            e = (des - self.curl) * span
            drive = self.drive if self.slack else 1.0
            k_act = pr["k_act"] * drive
            fmax = (pr["fmax_flex"] if e > 0 else pr["fmax_ext"]) * self.fatigue * drive
            lin = k_act * e
            if abs(lin) <= fmax:
                K += k_act; KX += k_act * des * span
                f_act = lin
            else:
                f_act = math.copysign(fmax, lin)
                Fc += f_act
            self.effort = min(1.0, abs(f_act) / max(fmax, 1e-6))
        else:
            self.effort = 0.0
        self.f_act = f_act

        # 근긴장 — 힘을 쓸수록 뻣뻣해진다 (프로필과 경직 등급 중 큰 값)
        sp = SPAS[self.grade]
        rest = max(pr["rest"], sp["rest"])
        k_tone = max(pr["k_tone"], sp["k_tone"]) * (
            1.0 + max(pr["tone_gain"], sp["tone_gain"]) * self.effort)
        K += k_tone; KX += k_tone * rest * span

        # 경직 — 속도 의존 신장반사. 빨리 펼수록 더 굽힌 위치에서 걸린다.
        raw_sr = 0.0
        if sp["k_sr"] > 0.0:
            ve = max(0.0, -self.vel)                     # 펴는 속도 [curl/s]
            self.sr_thr = sp["lam0"] + sp["mu"] * ve
            if self.curl < self.sr_thr:
                raw_sr = min(sp["f_sr"], sp["k_sr"] * (self.sr_thr - self.curl) * span)
        else:
            self.sr_thr = None
        tau = SR_TAU_UP if raw_sr > self.f_sr else SR_TAU_DOWN
        self.f_sr += (raw_sr - self.f_sr) * min(1.0, dt / tau)
        Fc += self.f_sr

        if pr["tremor"] > 0.0 and des is not None:
            Fc += pr["tremor"] * math.sin(2.0 * math.pi * 5.0 * t)

        # 치료사 결합 (환자 장갑이 내는 힘). 먼저 스프링으로 풀고, 상한을 넘으면 그 상한의 일정한 힘으로 다시 푼다.
        B0, K0, KX0, Fc0 = pr["b"], K, KX, Fc
        B = B0 + bc1
        if kc1 > 0.0 or bc1 > 0.0:
            K += kc1; KX += kc1 * c_t * span
            Fc += bc1 * v_t * span
        x_new = (B * x + dt * (KX + Fc)) / (B + dt * K)
        self.cpl_sat = False
        if cap1 is not None and (kc1 > 0.0 or bc1 > 0.0):
            v_try = (x_new - x) / dt / span
            f_try = kc1 * (c_t * span - x_new) + bc1 * (v_t - v_try) * span
            if abs(f_try) > cap1:
                self.cpl_sat = True
                B, K, KX = B0, K0, KX0
                Fc = Fc0 + math.copysign(cap1, f_try)
                x_new = (B * x + dt * (KX + Fc)) / (B + dt * K)
        self.f_contact = 0.0
        if obj is not None and x_new > obj * span:
            x_new = (B * x + dt * (KX + K_OBJ * obj * span + Fc)) / (B + dt * (K + K_OBJ))
            x_new = max(x_new, obj * span)
            self.f_contact = K_OBJ * (x_new - obj * span)

        c_new = x_new / span
        lim = CURL_RATE_MAX * dt
        c_new = max(self.curl - lim, min(self.curl + lim, c_new))
        c_new = max(-0.05, min(1.05, c_new))
        v = (c_new - self.curl) / dt if dt > 0 else 0.0
        self.vel += 0.4 * (v - self.vel)
        self.curl = c_new
        self.f_cpl = kc1 * (c_t - c_new) * span + bc1 * (v_t - self.vel) * span
        if cap1 is not None:
            self.f_cpl = max(-cap1, min(cap1, self.f_cpl))

        if self.slack:                     # 도움을 받을수록 스스로 덜 한다
            help_n = 0.0
            if des is not None and abs(des - c_new) > 0.02:
                help_n = max(0.0, math.copysign(1.0, des - c_new) * self.f_cpl)
            self.drive += dt * ((1.0 - self.drive) / SLACK_RECOVER_S - SLACK_GAIN * help_n / SLACK_F_REF)
            self.drive = max(SLACK_MIN, min(1.0, self.drive))

        if pr["fatigue_tau"] > 0.0:
            if self.effort > 0.3:
                self.fatigue -= dt * self.effort / pr["fatigue_tau"]
            else:
                self.fatigue += dt * 0.5 / pr["fatigue_tau"]
            self.fatigue = max(0.25, min(1.0, self.fatigue))


# ───────────────────────────────────────────── 가상 환자 전체
GRASP = "물체 잡고 옮기기"
G_NAMES = {
    "open": "손 펴기",
    "reach": "물체로 뻗기",
    "wait_open": "손 벌리기 — 물체를 감쌀 만큼 펴질 때까지 대기",
    "grasp": "잡기 — 꽉 쥘 때까지 대기",
    "transport": "들어 옮기기 — 쥐고 있는 동안만 이동",
    "release": "내려놓기 — 손을 펴서 놓을 때까지 대기",
    "rest": "휴식 (힘 빼기)",
}
ANSWER_COLS = ["event", "kind", "finger", "grade", "trial", "state", "bidirectional",
               "t_onset_s", "t_end_s", "duration_s", "curl_onset", "ext_vel_onset_cps",
               "thr_curl", "peak_N", "n_bursts", "detected", "t_mark_s", "reaction_s"]
TRIAL_COLS = ["trial", "result", "fail_at", "t_open_s", "t_grasp_s", "t_transport_s",
              "t_total_s", "slips", "limit_s", "bidirectional", "mode_changed", "profile"]


class VirtualPatient:
    def __init__(self):
        self.active = False            # 가상 환자 모드 사용
        self.bidirectional = True      # False = 단방향
        self.bi_gain = 0.0             # 치료사에게 가는 힘의 램프 (0~1)
        self.f2_cap = F2_CAP_DEFAULT
        self.f1_cap = F1_CAP_DEFAULT   # 가상 환자 장갑이 환자에게 낼 수 있는 힘 상한
        self.cpl_law = "curl"          # "curl" = 굽힘 차이·한쪽 줄 / "map" = 기존 줄별 지도 목표
        self.limit_s = GRASP_LIMIT_DEFAULT
        self.fingers = {f: VFinger(f) for f in FIDS}
        self.profile_name = "정상"
        self.scenario_name = GRASP
        self.loop = False
        self.playing = False
        self.t = 0.0                   # 시나리오 경과 시간 (반복해도 계속 증가)
        self.pose_t0 = 0.0             # 포즈 과제: 이번 반복이 시작된 시각
        self.cycle_n = 1
        self.phase_i = 0
        self.phase_name = "-"
        self.phase_left = 0.0
        self._prev_tgt = {f: None for f in FIDS}
        self.log = []
        self.log_on = False
        self.resync = True             # 켜는 순간 가상 손가락을 치료사 curl 에 맞춘다
        self.grip_ok = False
        self.live = FIDS               # 실제로 장갑에 있는 손가락 (ENABLED_FINGERS)
        self.trials = []
        self.grades = dict(SPAS_DEFAULT)
        self.events = []               # 경직 정답지
        self.marks = []                # 치료사가 '느꼈다' 표시한 (t, fid)
        self._open_ev = {f: None for f in FIDS}
        self._last_ev = {f: None for f in FIDS}
        self.kc = (0.0, 0.0, 0.0, 0.0)
        # 리듬 포즈
        self.assist = "therapist"
        self.k_robot = K_ROBOT_DEFAULT
        self.slacking = False
        self.r_interval = 3.0
        self.r_n = NOTES_DEFAULT
        self.r_seed = 1
        self.r_tempo = "보통 (3 s)"
        self.r_plan, self.r_notes = [], []
        self._r_reset()
        self._g_reset()
        self.set_profile(self.profile_name)
        self.set_grades(self.grades)

    def _g_reset(self):
        self.g_state = "idle"
        self.g_t = 0.0
        self.hand_pos = -1.0           # -1 대기 위치, 0 출발 받침, 1 목표 받침
        self.obj_state = "pad"         # pad | held | slip | placed | fallen | stuck
        self.obj_u = 0.0               # 옮긴 정도 0~1
        self.trial_on = False
        self.trial_t = 0.0
        self.trial = None
        self.n_ok = 0
        self.n_try = 0
        self.g_msg = ""
        self.g_msg_ok = True
        self._grip_t = 0.0
        self._nogrip_t = 0.0
        self._rest_from = -1.0
        self._tg_from = {f: None for f in FIDS}
        self._tg_to = None
        self._tg_t = 0.0

    def _r_reset(self):
        self.r_combo = self.r_max_combo = self.r_score = 0
        self.r_last = None             # (결과, 놓친 손가락, 시각)
        self.r_move = min(MOVE_TIME, 0.6 * self.r_interval)

    def assist_mode(self):
        """지금 실제로 쓰이는 보조 방식. 리듬 과제가 아니면 항상 치료사 결합."""
        return self.assist if self.scenario_name == RHYTHM else "therapist"

    # UI 명령 (ctl.lock 을 잡고 호출)
    def set_profile(self, name):
        self.profile_name = name
        for f, vf in self.fingers.items():
            vf.prof = PROFILES[name][f]
            vf.fatigue = 1.0

    def set_grades(self, grades):
        for f, g in grades.items():
            if f in self.fingers and g in SPAS:
                self.grades[f] = g
                self.fingers[f].grade = g

    def start(self):
        self.pose_t0 = 0.0
        self.cycle_n = 1
        self.events, self.marks = [], []
        self._open_ev = {f: None for f in FIDS}
        self._last_ev = {f: None for f in FIDS}
        self.t = 0.0
        self.phase_i = 0
        self._prev_tgt = {f: self.fingers[f].curl for f in FIDS}
        self.log = []
        self.trials = []
        self._g_reset()
        for vf in self.fingers.values():
            vf.slack = self.slacking
            vf.drive = 1.0
        self.r_notes = []
        self._r_reset()
        if self.scenario_name == GRASP:
            self._g_enter("open", _OPEN)
        elif self.scenario_name == RHYTHM:
            self._r_build()
        self.log_on = True
        self.playing = True

    def stop(self):
        for f, ev in self._open_ev.items():          # 정지 시 열려 있던 경직은 지금 시각으로 닫는다
            if ev is not None:
                ev["_t_end"] = self.t
                ev["t_end_s"] = round(self.t, 3)
                ev["duration_s"] = round(self.t - ev["t_onset_s"], 3)
                self._last_ev[f] = ev
        self._open_ev = {f: None for f in FIDS}
        self.playing = False
        self.log_on = False
        self.phase_name = "-"
        self.phase_left = 0.0
        self.trial_on = False
        self.g_state = "idle"

    # ── 물체 잡고 옮기기: 조건이 채워질 때까지 기다리는 상태기계 ──
    def _g_enter(self, state, tgt):
        self.g_state = state
        self.g_t = 0.0
        self._grip_t = self._nogrip_t = 0.0
        self._tg_from = {f: (self.fingers[f].des if self.fingers[f].des is not None
                             else self.fingers[f].curl) for f in FIDS}
        self._tg_to = tgt
        self._tg_t = 0.0
        self.phase_name = G_NAMES[state]

    def _g_finish(self, ok, fail_at=""):
        tr = self.trial
        tr.update(result="success" if ok else "fail", fail_at=fail_at,
                  t_total_s=round(self.trial_t, 3))
        self.trials.append(tr)
        self.n_try += 1
        self.n_ok += int(ok)
        self.trial_on = False
        if ok:
            self.obj_state = "placed"
            self.g_msg, self.g_msg_ok = f"성공!  {self.trial_t:0.1f}초", True
        else:
            self.obj_state = {"transport": "fallen", "release": "stuck"}.get(fail_at, "pad")
            why = {"wait_open": "손을 못 벌림", "grasp": "못 잡음",
                   "transport": "옮기다 시간 초과", "release": "못 놓음"}[fail_at]
            self.g_msg, self.g_msg_ok = f"실패 — {why} (제한 {self.limit_s:g}초)", False
        self._rest_from = self.hand_pos
        self._g_enter("rest", None)

    def _grasp_targets(self, dt):
        fs, live = self.fingers, self.live
        s = self.g_state
        self.g_t += dt
        if self.trial_on:
            self.trial_t += dt
            if self.trial["bidirectional"] != int(self.bidirectional):
                self.trial["mode_changed"] = 1
        grip = bool(live) and all(fs[f].f_contact >= GRIP_OK_N for f in live)

        if s == "open":
            self.hand_pos = -1.0
            if self.g_t >= OPEN_S:
                self._g_enter("reach", _OPEN)
        elif s == "reach":
            self.hand_pos = -1.0 + _minjerk(self.g_t / REACH_S)
            if self.g_t >= REACH_S:
                self.trial_on, self.trial_t = True, 0.0          # 여기서부터 제한 시간을 잰다
                self.trial = dict(trial=len(self.trials) + 1, result="", fail_at="",
                                  t_open_s="", t_grasp_s="", t_transport_s="", t_total_s="",
                                  slips=0, limit_s=self.limit_s,
                                  bidirectional=int(self.bidirectional), mode_changed=0,
                                  profile=self.profile_name)
                self.g_msg = ""
                self._g_enter("wait_open", _OPEN)
        elif s == "wait_open":
            self.hand_pos = 0.0
            if all(fs[f].curl <= _OBJ[f] - APERTURE_MARGIN for f in live):
                self.trial["t_open_s"] = round(self.trial_t, 3)
                self._g_enter("grasp", _SQUEEZE)
        elif s == "grasp":
            self._grip_t = self._grip_t + dt if grip else 0.0
            if self._grip_t >= GRIP_HOLD_S:
                self.trial["t_grasp_s"] = round(self.trial_t, 3)
                self.obj_state = "held"
                self._g_enter("transport", _SQUEEZE)
        elif s == "transport":
            if grip:
                self._nogrip_t = 0.0
                if self.obj_state == "slip":
                    self._grip_t += dt
                    if self._grip_t >= GRIP_HOLD_S:
                        self.obj_state = "held"
                else:
                    self.obj_u = min(1.0, self.obj_u + dt / TRANSPORT_S)
            else:
                self._grip_t = 0.0
                self._nogrip_t += dt
                if self.obj_state == "held" and self._nogrip_t >= SLIP_S:
                    self.obj_state = "slip"
                    self.trial["slips"] += 1
            self.hand_pos = _minjerk(self.obj_u)
            if self.obj_u >= 1.0:
                self.trial["t_transport_s"] = round(self.trial_t, 3)
                self._g_enter("release", _OPEN)
        elif s == "release":
            self.hand_pos = 1.0
            if all(fs[f].f_contact <= 0.0 and fs[f].curl <= _OBJ[f] - RELEASE_MARGIN
                   for f in live):
                self._g_finish(True)
        elif s == "rest":
            self.hand_pos = self._rest_from + (-1.0 - self._rest_from) * _minjerk(self.g_t / REST_S)
            if self.g_t >= REST_S:
                if self.loop:
                    self.obj_state, self.obj_u = "pad", 0.0
                    self._g_enter("open", _OPEN)
                else:
                    self.stop()
                    return {f: None for f in FIDS}, None

        if self.trial_on and self.trial_t > self.limit_s:
            self._g_finish(False, self.g_state)

        s = self.g_state
        self.grip_ok = grip
        self.phase_left = (self.limit_s - self.trial_t) if self.trial_on else 0.0
        obj = _OBJ if s in ("grasp", "transport", "release") else None
        if self._tg_to is None:
            return {f: None for f in FIDS}, obj
        a = _minjerk(self._tg_t / MOVE_TIME)
        self._tg_t += dt
        des = {}
        for f in FIDS:
            p0 = self._tg_from[f]
            des[f] = p0 + a * (self._tg_to[f] - p0)
        return des, obj

    # ── 리듬 포즈 ──
    def _r_build(self):
        """노트 순서를 만든다. 같은 seed 면 같은 순서 → 조건끼리 같은 악보로 비교할 수 있다."""
        live = [f for f in self.live] or list(FIDS)
        lib, seen = {}, set()
        for name, pose in RHYTHM_POSES.items():
            key = tuple(round(pose[f], 2) for f in live)
            if key not in seen:
                seen.add(key)
                lib[name] = dict(pose)
        if len(live) < 2 or len(lib) < 3:              # 손가락이 하나뿐이면 포즈 이름 대신 굽힘 단계로
            lib = {name: {f: v for f in FIDS} for name, v in RHYTHM_LEVELS.items()}
        rng = random.Random(self.r_seed)
        names = list(lib)
        prev = dict(_OPEN)
        self.r_plan = []
        for k in range(self.r_n):
            for _ in range(100):
                nm = rng.choice(names)
                if max(abs(lib[nm][f] - prev[f]) for f in live) >= RHYTHM_MIN_STEP:
                    break
            self.r_plan.append(dict(k=k + 1, pose=nm, tgt=dict(lib[nm]), start=dict(prev),
                                    hit=RHYTHM_LEAD_S + k * self.r_interval,
                                    acc={}, t_ok=None, done=False, res=None))
            prev = lib[nm]

    def _rhythm_targets(self, dt):
        self.t += dt
        t, plan, mv = self.t, self.r_plan, self.r_move
        if not plan or t > plan[-1]["hit"] + RHYTHM_JUDGE[1] + 1.0:
            self.stop()
            return {f: None for f in FIDS}, None
        cur = None
        for nt in plan:
            if t < nt["hit"] - mv:
                break
            cur = nt
        if cur is None:
            des = dict(_OPEN)
            self.phase_name = "준비"
        else:
            a = _minjerk((t - (cur["hit"] - mv)) / mv)
            des = {f: cur["start"][f] + a * (cur["tgt"][f] - cur["start"][f]) for f in FIDS}
            self.phase_name = f"노트 {cur['k']}/{len(plan)} — {cur['pose']}"
        nxt = next((nt for nt in plan if nt["hit"] >= t), None)
        self.phase_left = (nxt["hit"] - t) if nxt else 0.0
        return des, None

    def rhythm_track(self, dt):
        """매 틱, 모든 손가락을 움직인 뒤 호출: 판정 구간의 오차와 환자/보조 기여를 모은다."""
        if self.scenario_name != RHYTHM or not self.log_on:
            return
        t, mv, j0, j1 = self.t, self.r_move, RHYTHM_JUDGE[0], RHYTHM_JUDGE[1]
        for nt in self.r_plan:
            if nt["done"]:
                continue
            if t < nt["hit"] - mv:
                break
            acc = nt["acc"]
            if not acc:                                    # 이 노트로 움직이기 시작한 순간
                for f in self.live:
                    vf = self.fingers[f]
                    d = nt["tgt"][f] - vf.curl
                    acc[f] = dict(s=(0 if abs(d) < 0.05 else (1 if d > 0 else -1)),
                                  err=[], pw=0.0, aw=0.0, drive=vf.drive)
            for f, a in acc.items():
                vf = self.fingers[f]
                if a["s"]:
                    a["pw"] += max(0.0, a["s"] * vf.f_act) * dt     # 환자가 목표 쪽으로 낸 힘 × 시간
                    a["aw"] += max(0.0, a["s"] * vf.f_cpl) * dt     # 치료사/로봇이 목표 쪽으로 낸 힘 × 시간
                if nt["hit"] + j0 <= t <= nt["hit"] + j1:
                    a["err"].append(abs(vf.curl - nt["tgt"][f]))
                a["drive"] = vf.drive
            if nt["t_ok"] is None and acc and all(
                    abs(self.fingers[f].curl - nt["tgt"][f]) <= RHYTHM_TIMING_TOL for f in acc):
                nt["t_ok"] = t
            if t > nt["hit"] + j1:
                self._r_judge(nt)

    def _r_judge(self, nt):
        errs = {f: (statistics.median(a["err"]) if a["err"] else None) for f, a in nt["acc"].items()}
        valid = [e for e in errs.values() if e is not None]
        worst = max(valid) if valid else 1.0
        res = "PERFECT" if worst <= RHYTHM_PERFECT else "GOOD" if worst <= RHYTHM_GOOD else "MISS"
        miss_f = [FNAME[f] for f, e in errs.items() if e is None or e > RHYTHM_GOOD]
        self.r_combo = self.r_combo + 1 if res != "MISS" else 0
        self.r_max_combo = max(self.r_max_combo, self.r_combo)
        self.r_score += {"PERFECT": 100, "GOOD": 50}.get(res, 0)
        row = dict(note=nt["k"], pose=nt["pose"], t_hit_s=round(nt["hit"], 3), result=res,
                   miss_fingers="·".join(miss_f),
                   timing_s=(round(nt["t_ok"] - nt["hit"], 3) if nt["t_ok"] is not None else ""),
                   combo=self.r_combo, assist=self.assist, bidirectional=int(self.bidirectional),
                   slacking=int(self.slacking))
        for f, a in nt["acc"].items():
            tot = a["pw"] + a["aw"]
            row.update({f"{f}_target": nt["tgt"][f],
                        f"{f}_err": (round(errs[f], 3) if errs[f] is not None else ""),
                        f"{f}_patient_Ns": round(a["pw"], 3), f"{f}_assist_Ns": round(a["aw"], 3),
                        f"{f}_contrib": (round(a["pw"] / tot, 3) if tot > 0.05 else ""),
                        f"{f}_drive": round(a["drive"], 3), f"{f}_grade": self.grades[f]})
        nt["res"], nt["done"] = res, True
        self.r_notes.append(row)
        self.r_last = (res, miss_f, self.t, row)

    # 제어 스레드에서 매 틱 호출
    def targets(self, dt):
        """현재 단계의 (목표 dict, 물체 dict). 재생 중이 아니면 (None, None) = 힘 뺌."""
        if not self.playing:
            return {f: None for f in FIDS}, None
        if self.scenario_name == GRASP:
            self.t += dt
            return self._grasp_targets(dt)
        if self.scenario_name == RHYTHM:
            return self._rhythm_targets(dt)
        phases = SCENARIOS[self.scenario_name]
        tp = self.t - self.pose_t0          # 이번 반복 안에서의 시간 (전체 시계 self.t 는 되돌리지 않는다)
        t_acc = 0.0
        for i, (name, dur, tgt, obj) in enumerate(phases):
            if tp < t_acc + dur:
                if i != self.phase_i:
                    prev = phases[self.phase_i][2]
                    self._prev_tgt = {f: (prev[f] if prev else self.fingers[f].curl) for f in FIDS}
                    self.phase_i = i
                self.phase_name = name
                self.phase_left = t_acc + dur - tp
                a = _minjerk((tp - t_acc) / min(MOVE_TIME, dur))
                if tgt is None:
                    des = {f: None for f in FIDS}
                else:
                    des = {}
                    for f in FIDS:
                        p0 = self._prev_tgt[f]
                        p0 = self.fingers[f].curl if p0 is None else p0
                        des[f] = p0 + a * (tgt[f] - p0)
                self.t += dt
                return des, obj
            t_acc += dur
        if self.loop:
            self.pose_t0 = self.t
            self.cycle_n += 1
            self.phase_i = 0
            self._prev_tgt = {f: self.fingers[f].curl for f in FIDS}
            return {f: None for f in FIDS}, None
        self.stop()
        return {f: None for f in FIDS}, None

    # ── 경직 정답지 ──
    def track_spasm(self, f, vf):
        if not self.log_on:
            return
        ev = self._open_ev[f]
        if ev is None and vf.f_sr >= SR_ON_N:
            last = self._last_ev[f]
            if last is not None and self.t - last["_t_end"] < SR_MERGE_S:
                ev = last
                ev["n_bursts"] += 1
            else:
                ev = dict(event=len(self.events) + 1, finger=FNAME[f], fid=f, grade=vf.grade,
                          trial=(self.trial["trial"] if self.trial_on
                                 else "" if self.scenario_name == GRASP else self.cycle_n),
                          kind=("끝범위" if (vf.curl <= SPAS[vf.grade]["lam0"] + 0.02
                                             and max(0.0, -vf.vel) < CATCH_VEL_MIN) else "catch"),
                          state=self.phase_name, bidirectional=int(self.bidirectional),
                          t_onset_s=round(self.t, 3), t_end_s="", duration_s="",
                          curl_onset=round(vf.curl, 3),
                          ext_vel_onset_cps=round(max(0.0, -vf.vel), 3),
                          thr_curl=round(vf.sr_thr if vf.sr_thr is not None else 0.0, 3),
                          peak_N=0.0, n_bursts=1, detected="", t_mark_s="", reaction_s="")
                self.events.append(ev)
            self._open_ev[f] = ev
        if ev is not None:
            ev["peak_N"] = max(ev["peak_N"], round(vf.f_sr, 2))
            if vf.f_sr < SR_OFF_N:
                ev["_t_end"] = self.t
                ev["t_end_s"] = round(self.t, 3)
                ev["duration_s"] = round(self.t - ev["t_onset_s"], 3)
                self._open_ev[f] = None
                self._last_ev[f] = ev

    def add_mark(self, f):
        if self.log_on:
            if any(g == f and self.t - tm < 0.15 for tm, g in self.marks[-3:]):
                return                                   # 같은 키가 두 번 잡힌 것
            self.marks.append((round(self.t, 3), f))
            self._mark_tag = (self._mark_tag + f) if getattr(self, "_mark_tag", "") else f

    def pop_mark_tag(self):
        tag, self._mark_tag = getattr(self, "_mark_tag", ""), ""
        return tag

    def score_marks(self):
        """표시를 경직 이벤트에 맞춘다. 반환: (오경보 목록, 손가락별 요약 dict)."""
        for ev in self.events:
            ev.update(detected=0, t_mark_s="", reaction_s="")
        false_alarms = []
        lo, hi = MARK_WINDOW
        for tm, f in sorted(self.marks):
            cands = [ev for ev in self.events
                     if ev["fid"] == f and ev["detected"] == 0 and lo <= tm - ev["t_onset_s"] <= hi]
            if cands:
                ev = max(cands, key=lambda e: e["t_onset_s"])   # 가장 최근에 시작한 경직
                ev.update(detected=1, t_mark_s=tm, reaction_s=round(tm - ev["t_onset_s"], 3))
            else:
                false_alarms.append((tm, f))
        summary = {}
        for f in FIDS:
            evs = [e for e in self.events if e["fid"] == f]
            fa = sum(1 for _, g in false_alarms if g == f)
            if evs or fa or f in self.live:
                cat = [e for e in evs if e.get("kind") == "catch"]
                summary[f] = dict(n=len(evs), hit=sum(e["detected"] for e in evs), fa=fa,
                                  n_catch=len(cat), hit_catch=sum(e["detected"] for e in cat),
                                  grade=self.grades.get(f, "0"))
        return false_alarms, summary

    def update_gain(self, dt):
        if self.bidirectional:
            self.bi_gain = min(1.0, self.bi_gain + dt / BI_RAMP_UP_S)
        else:
            self.bi_gain = max(0.0, self.bi_gain - dt / BI_RAMP_DOWN_S)

    def save_log(self, folder):
        """시계열 CSV 와 (물체 과제라면) 시도별 요약 CSV 를 저장. (시계열 경로, 요약 경로) 반환."""
        if not self.log:
            return None, None, None
        os.makedirs(folder, exist_ok=True)
        stamp = datetime.now().strftime("vp_%Y%m%d_%H%M%S")
        path = os.path.join(folder, stamp + ".csv")
        hdr = ["t_s", "phase", "bidirectional", "bi_gain", "state", "trial", "trial_t_s", "object",
               "mark", "assist"]
        for f in self.live:
            hdr += [f"{f}_des", f"{f}_patient_curl", f"{f}_therapist_curl",
                    f"{f}_cpl_cmd_flex_N", f"{f}_cpl_cmd_ext_N",
                    f"{f}_cpl_uncapped_flex_N", f"{f}_cpl_uncapped_ext_N",
                    f"{f}_motor_flex_N", f"{f}_motor_ext_N",
                    f"{f}_F_to_patient_N", f"{f}_patient_active_N", f"{f}_contact_N",
                    f"{f}_effort", f"{f}_fatigue",
                    f"{f}_spasm_N", f"{f}_spasm_thr_curl", f"{f}_spasm_on", f"{f}_drive"]
        kc = self.kc
        meta = ["# profile", self.profile_name, "scenario", self.scenario_name,
                "KC1", kc[0], "BC1", kc[1], "KC2", kc[2], "BC2", kc[3],
                "therapist_cap_N", self.f2_cap, "patient_cap_N", self.f1_cap,
                "coupling_law", self.cpl_law, "slacking", int(self.slacking)]
        if self.scenario_name == RHYTHM:
            meta += ["assist", self.assist, "K_robot", self.k_robot, "tempo_s", self.r_interval,
                     "notes", self.r_n, "seed", self.r_seed]
        meta += [f"grade_{f}={self.grades[f]}" for f in self.live]
        with open(path, "w", newline="", encoding="utf-8-sig") as fp:
            w = csv.writer(fp)
            w.writerow(meta)
            w.writerow(hdr)
            w.writerows(self.log)
        tpath = None
        if self.trials:
            tpath = os.path.join(folder, stamp + "_trials.csv")
            with open(tpath, "w", newline="", encoding="utf-8-sig") as fp:
                w = csv.writer(fp)
                w.writerow(TRIAL_COLS)
                for tr in self.trials:
                    w.writerow([tr.get(k, "") for k in TRIAL_COLS])
        apath = os.path.join(folder, stamp + "_answer.csv")
        false_alarms, summary = self.score_marks()
        with open(apath, "w", newline="", encoding="utf-8-sig") as fp:
            w = csv.writer(fp)
            w.writerow(["# 경직 정답지", "profile", self.profile_name, "scenario", self.scenario_name]
                       + [f"grade_{f}={self.grades[f]}" for f in self.live])
            w.writerow(ANSWER_COLS)
            for ev in self.events:
                w.writerow([ev.get(k, "") for k in ANSWER_COLS])
            w.writerow([])
            w.writerow(["# 오경보 (경직이 없는데 표시함)", "t_mark_s", "finger"])
            for tm, f in false_alarms:
                w.writerow(["false_alarm", tm, FNAME[f]])
            w.writerow([])
            w.writerow(["# 요약", "finger", "grade", "spasms", "detected", "false_alarms",
                        "mean_reaction_s", "catch_only", "catch_detected"])
            for f, sm in summary.items():
                rts = [e["reaction_s"] for e in self.events if e["fid"] == f and e["detected"] == 1]
                w.writerow(["summary", FNAME[f], sm["grade"], sm["n"], sm["hit"], sm["fa"],
                            round(sum(rts) / len(rts), 3) if rts else "",
                            sm["n_catch"], sm["hit_catch"]])
        self.last_summary = summary
        self.rpath = None
        if self.r_notes:
            self.rpath = os.path.join(folder, stamp + "_rhythm.csv")
            cols = []
            for row in self.r_notes:
                cols += [k for k in row if k not in cols]
            with open(self.rpath, "w", newline="", encoding="utf-8-sig") as fp:
                w = csv.writer(fp)
                w.writerow(["# 리듬 포즈", "assist", ASSIST_MODES[self.assist], "tempo_s", self.r_interval,
                            "seed", self.r_seed, "slacking", int(self.slacking),
                            "profile", self.profile_name]
                           + [f"grade_{f}={self.grades[f]}" for f in self.live])
                w.writerow(cols)
                for row in self.r_notes:
                    w.writerow([row.get(k, "") for k in cols])
                w.writerow([])
                n = len(self.r_notes)
                cnt = {r: sum(1 for x in self.r_notes if x["result"] == r) for r in ("PERFECT", "GOOD", "MISS")}
                w.writerow(["# 요약", "notes", n, "PERFECT", cnt["PERFECT"], "GOOD", cnt["GOOD"],
                            "MISS", cnt["MISS"], "accuracy_pct",
                            round(100.0 * (cnt["PERFECT"] + 0.5 * cnt["GOOD"]) / max(1, n), 1),
                            "max_combo", self.r_max_combo])
                for f in self.live:
                    cs = [x[f"{f}_contrib"] for x in self.r_notes if x.get(f"{f}_contrib") not in ("", None)]
                    w.writerow(["summary", FNAME[f], "mean_patient_contrib",
                                round(sum(cs) / len(cs), 3) if cs else ""])
        return path, tpath, apath


def curl_couple(f2, vf, kc, bc, span):
    """굽힘 차이 기반 결합 (가상 환자 → 치료사).
    환자와 내 굽힘(curl) 차이에만 비례하고, 한 번에 한쪽 줄만 당긴다:
      환자가 더 굽어 있으면 굽힘 줄, 더 펴져 있으면 폄 줄.
    굽힘이 같으면 힘이 0 이라 양쪽 줄이 동시에 당기는(co-contraction) 일이 없다.
    (기존 couple() 은 줄마다 지도 목표 위치와 비교해서, 지도 히스테리시스만큼 두 줄이 같이 당겨질 수 있다)"""
    return _curl_force(f2, vf.curl, vf.vel, kc, bc, span)


def _curl_force(me, partner_curl, partner_vel, kc, bc, span):
    """me 장갑이 받는 힘 = 상대와의 굽힘 차이 × 게인. 한쪽 줄만 당긴다. (굽힘 줄, 폄 줄) [N]"""
    if not me.coupling_enabled or (kc <= 0.0 and bc <= 0.0):
        return 0.0, 0.0
    dc = partner_curl - me.curl
    dc = math.copysign(max(0.0, abs(dc) - CURL_CPL_DB), dc)      # 작은 차이는 무시 (떨림 방지)
    dv = partner_vel - me.curl_vel
    F = me.coupling_soft * (kc * dc * span + bc * dv * span)
    F = max(-CURL_CPL_MAX, min(CURL_CPL_MAX, F))
    return (F, 0.0) if F > 0.0 else (0.0, -F)


def person_couple(me, partner, kc, bc, over, law):
    """사람끼리(G1↔G2) 결합에서 me 가 받는 힘. law: "curl" 굽힘 차이 / "map_nocc" 기존 − 동시 당김."""
    me.cpl_f = me.cpl_e = 0.0
    if me.single or not (me.map.ready and me.map_aligned and me.coupling_enabled):
        return 0.0, 0.0
    if law == "curl":
        F = _curl_force(me, partner.curl, partner.curl_vel, kc, bc, _span(me))
    else:
        F = me.couple(partner.curl, partner.curl_vel, kc, bc, over)
        cc = min(F)                                   # 두 줄에 공통으로 걸린 만큼을 뺀다
        F = (F[0] - cc, F[1] - cc)
    me.cpl_f, me.cpl_e = F
    return F


def _span(finger):
    sf = getattr(finger.map, "_sf", None)
    try:
        sf = abs(float(sf))
    except (TypeError, ValueError):
        sf = 0.0
    return sf if sf > 1.0 else SPAN_FALLBACK


# ───────────────────────────────────────────── 제어 스레드에 끼워 넣기
def install(mod, vp):
    """ControlThread 의 두 메서드를 감싼다. 가상 환자 모드가 꺼져 있으면 원래 동작 그대로."""
    CT = mod.ControlThread
    orig_apply_run = CT._apply_run
    orig_set_enabled = CT.set_control_enabled

    def _apply_run(self, p, dt):
        if not vp.active:
            vp.bi_gain = 0.0
            vp.resync = True
            # 사람끼리(G1↔G2): 기존 방식이거나 실험용 position_velocity 모드면 원래 코드 그대로
            if vp.cpl_law == "map" or getattr(self, "coupling_mode", "force") != "force":
                return orig_apply_run(self, p, dt)
            prs = self.pairs()
            for f1, f2 in prs:
                f1.coupling_mode = f2.coupling_mode = self.coupling_mode
                pair_ready = all(f.map.ready and f.map_aligned and not f.single for f in (f1, f2))
                if pair_ready:
                    F1 = person_couple(f1, f2, p["KC1"], p["BC1"], p["OVER"], vp.cpl_law)
                    F2 = person_couple(f2, f1, p["KC2"], p["BC2"], p["OVER"], vp.cpl_law)
                else:
                    f1.cpl_f = f1.cpl_e = f2.cpl_f = f2.cpl_e = 0.0
                    F1 = F2 = (0.0, 0.0)
                f1.apply(F1, p, dt)
                f2.apply(F2, p, dt)
            paired = {id(f) for pr in prs for f in pr}
            for gl in self.gloves:
                for f in gl.fingers.values():
                    if id(f) not in paired:
                        f.cpl_f = f.cpl_e = 0.0
                        f.apply((0.0, 0.0), p, dt)
            self.cpl.monitor(prs, dt)
            return

        vp.live = tuple(f for f in FIDS if f in self.g2.fingers)
        vp.kc = (p["KC1"], p["BC1"], p["KC2"], p["BC2"])
        des, obj = vp.targets(dt)
        vp.update_gain(dt)
        row = [round(vp.t, 4), vp.phase_name, int(vp.bidirectional), round(vp.bi_gain, 3),
               vp.g_state, (vp.trial["trial"] if vp.trial_on else ""),
               (round(vp.trial_t, 3) if vp.trial_on else ""), vp.obj_state,
               vp.pop_mark_tag(), vp.assist_mode()]
        mode = vp.assist_mode()
        grip = bool(obj)
        for f2 in (self.g2.fingers[f] for f in vp.live):   # 환자 장갑(G1)이 없어도 동작
            vf = vp.fingers[f2.fid]
            ready = f2.map.ready and f2.map_aligned and not f2.single
            F2 = raw = (0.0, 0.0)
            if ready:
                if vp.resync:
                    vf.reset(f2.curl)
                span = _span(f2)
                g = f2.coupling_soft if f2.coupling_enabled else 0.0   # 결합을 켠 만큼만 연결
                d = des[f2.fid]
                if mode == "robot":                 # 고정 로봇: 노트 궤적으로 일정 강성으로 당김 (치료사와 끊김)
                    kc, bc, ct, vt = (vp.k_robot, 0.0, d, 0.0) if d is not None else (0.0, 0.0, 0.0, 0.0)
                elif mode == "none":                # 환자 혼자
                    kc, bc, ct, vt = 0.0, 0.0, 0.0, 0.0
                else:
                    kc, bc, ct, vt = p["KC1"] * g, p["BC1"] * g, f2.curl, f2.curl_vel
                vf.step(dt, vp.t, d, (obj or {}).get(f2.fid), ct, vt, kc, bc, span, vp.f1_cap)
                f2.coupling_mode = self.coupling_mode
                if vp.bi_gain > 0.0 and mode == "therapist":
                    if vp.cpl_law == "curl":
                        raw = curl_couple(f2, vf, p["KC2"], p["BC2"], span)
                    else:
                        raw = f2.couple(vf.curl, vf.vel, p["KC2"], p["BC2"], p["OVER"])
                        if vp.cpl_law == "map_nocc":       # 두 줄에 공통으로 걸린 만큼(동시 당김)을 뺀다
                            cc = min(raw)
                            raw = (raw[0] - cc, raw[1] - cc)
                    cap = vp.f2_cap
                    F2 = (min(raw[0], cap) * vp.bi_gain, min(raw[1], cap) * vp.bi_gain)
                f2.cpl_f, f2.cpl_e = F2
            else:
                f2.cpl_f = f2.cpl_e = 0.0
            f2.apply(F2, p, dt)
            vf.cmd, vf.raw = F2, raw                       # 화면 표시용: 실제 지령 / 상한 적용 전
            if ready:
                vp.track_spasm(f2.fid, vf)
            if obj and obj.get(f2.fid) is not None and vf.f_contact < GRIP_OK_N:
                grip = False
            row += [None if vf.des is None else round(vf.des, 4), round(vf.curl, 4),
                    round(f2.curl, 4), round(F2[0], 3), round(F2[1], 3),
                    round(raw[0], 3), round(raw[1], 3),
                    round(f2.flex.applied_N, 3), round(f2.ext.applied_N, 3),
                    round(vf.f_cpl, 3), round(vf.f_act, 3), round(vf.f_contact, 3),
                    round(vf.effort, 3), round(vf.fatigue, 3),
                    round(vf.f_sr, 3), (None if vf.sr_thr is None else round(vf.sr_thr, 3)),
                    int(vp._open_ev[f2.fid] is not None), round(vf.drive, 3)]
        vp.resync = False
        vp.rhythm_track(dt)
        if vp.scenario_name != GRASP:
            vp.grip_ok = grip
        for f in self.g1.fingers.values():       # 실제 환자 장갑에는 결합력을 보내지 않는다
            f.cpl_f = f.cpl_e = 0.0
            f.apply((0.0, 0.0), p, dt)
        self.cpl.monitor([], dt)
        if vp.log_on and len(vp.log) < 400000:
            vp.log.append(row)

    def set_control_enabled(self, finger, kind, on):
        # 가상 환자 모드에서 G2 결합을 켤 때는 실제 환자 장갑(G1)의 지도·0점이 필요 없다.
        if on and kind == "coupling" and vp.active and finger.glove_short == "G2":
            with self.lock:
                if self.estop or self.control_error:
                    return "비상정지를 해제한 뒤 다시 체크하세요."
                if self.mode != "run" or self.run_zero_req:
                    return "양방향 제어 탭에서 0점 초기화 완료 후 체크하세요."
                if not finger.map.ready:
                    return f"{finger.name}: 먼저 지도를 준비하세요."
                if not finger.map_aligned:
                    return f"{finger.name}: 먼저 '전체 엔코더 0점'을 완료하세요."
                if finger.single:
                    return f"{finger.name}: 엔코더 2개가 모두 필요합니다."
                vp.resync = True
                finger.set_control(kind, on)
                self.bank.flush()
                return None
        if on and kind == "coupling" and vp.active and finger.glove_short == "G1":
            return "가상 환자 모드에서는 환자 장갑(G1) 결합을 켤 수 없습니다."
        return orig_set_enabled(self, finger, kind, on)

    CT._apply_run = _apply_run
    CT.set_control_enabled = set_control_enabled


# ───────────────────────────────────────────── 조작 창
def build_panel(tk, ttk, root, ctl, vp, log_dir, app=None):
    win = tk.Toplevel(root)
    win.title("가상 환자 — 단방향 / 양방향 비교")
    win.geometry("800x1060")
    win.protocol("WM_DELETE_WINDOW", lambda: None)   # 실수로 닫히지 않게

    v_active = tk.BooleanVar(value=False)
    v_dir = tk.StringVar(value="bi")
    v_prof = tk.StringVar(value=vp.profile_name)
    v_scen = tk.StringVar(value=vp.scenario_name)
    v_loop = tk.BooleanVar(value=False)
    v_show = tk.BooleanVar(value=True)
    v_cap = tk.DoubleVar(value=vp.f2_cap)
    v_cap1 = tk.DoubleVar(value=vp.f1_cap)
    v_limit = tk.DoubleVar(value=vp.limit_s)
    v_answer = tk.BooleanVar(value=True)
    v_grade = {f: tk.StringVar(value=vp.grades[f]) for f in FIDS}
    v_tempo = tk.StringVar(value=vp.r_tempo)
    v_notes = tk.IntVar(value=vp.r_n)
    v_seed = tk.IntVar(value=vp.r_seed)
    v_assist = tk.StringVar(value=vp.assist)
    v_krobot = tk.DoubleVar(value=vp.k_robot)
    v_slack = tk.BooleanVar(value=vp.slacking)
    status = tk.StringVar(value="가상 환자 꺼짐 — 기존 양방향 제어와 동일하게 동작")

    def coupling_on():
        return any(f.coupling_enabled for g in ctl.gloves for f in g.fingers.values())

    def on_active():
        with ctl.lock:
            if coupling_on():
                v_active.set(vp.active)
                status.set("결합(양방향) 체크를 모두 끈 뒤에 전환하세요.")
                return
            vp.active = v_active.get()
            vp.resync = True
            vp.bi_gain = 0.0
            if not vp.active:
                vp.stop()
        status.set("가상 환자 켜짐 — G2 결합 체크를 켜고 [시작]" if v_active.get()
                   else "가상 환자 꺼짐 — 기존 양방향 제어와 동일하게 동작")

    def on_dir():
        with ctl.lock:
            vp.bidirectional = (v_dir.get() == "bi")

    def on_prof(_=None):
        with ctl.lock:
            vp.set_profile(v_prof.get())

    def on_scen(_=None):
        with ctl.lock:
            if vp.playing:
                v_scen.set(vp.scenario_name)
                return
            vp.scenario_name = v_scen.get()

    def on_loop():
        with ctl.lock:
            vp.loop = v_loop.get()

    def _num(var, lo, hi):
        try:
            return max(lo, min(hi, float(var.get())))
        except (tk.TclError, ValueError):
            return None

    def on_cap(_=None):
        c = _num(v_cap, *F2_CAP_RANGE)
        if c is not None:
            with ctl.lock:
                vp.f2_cap = c

    def on_cap1(_=None):
        c = _num(v_cap1, *F1_CAP_RANGE)
        if c is not None:
            with ctl.lock:
                vp.f1_cap = c

    def on_limit(_=None):
        c = _num(v_limit, *GRASP_LIMIT_RANGE)
        if c is not None:
            with ctl.lock:
                vp.limit_s = c                    # 진행 중인 시도에도 바로 적용된다

    def on_start():
        on_cap(); on_cap1(); on_limit()
        with ctl.lock:
            if not vp.active:
                status.set("먼저 [가상 환자 사용] 을 켜세요.")
                return
            if ctl.estop:
                status.set("비상정지 상태입니다.")
                return
            vp.scenario_name = v_scen.get()
            vp.start()
        status.set("재생 중")

    def on_rhythm_start():
        on_cap(); on_cap1()
        try:
            n = int(max(NOTES_RANGE[0], min(NOTES_RANGE[1], int(v_notes.get()))))
            seed = int(v_seed.get())
        except (tk.TclError, ValueError):
            status.set("노트 수와 순서 번호는 정수로 넣어 주세요.")
            return
        k_rob = _num(v_krobot, *K_ROBOT_RANGE)
        with ctl.lock:
            if not vp.active:
                status.set("먼저 [가상 환자 사용] 을 켜세요.")
                return
            if ctl.estop:
                status.set("비상정지 상태입니다.")
                return
            vp.scenario_name = RHYTHM
            vp.live = tuple(f for f in FIDS if f in ctl.g2.fingers)
            vp.r_tempo = v_tempo.get()
            vp.r_interval = RHYTHM_TEMPI[vp.r_tempo]
            vp.r_n, vp.r_seed = n, seed
            vp.assist = v_assist.get()
            vp.k_robot = k_rob if k_rob is not None else K_ROBOT_DEFAULT
            vp.slacking = v_slack.get()
            vp.start()
        status.set(f"리듬 포즈 재생 중 — {ASSIST_MODES[v_assist.get()]}, 순서 #{seed}")

    def on_new_seed():
        v_seed.set(random.randint(1, 9999))

    def on_stop(msg="정지"):
        with ctl.lock:
            was = vp.playing or bool(vp.log)
            vp.stop()
            path, tpath, apath = vp.save_log(log_dir) if was else (None, None, None)
            vp.log = []
            summary = getattr(vp, "last_summary", {}) if path else {}
            rpath = getattr(vp, "rpath", None) if path else None
            rn = list(vp.r_notes) if rpath else []
            r_max = vp.r_max_combo
        if rpath and rn:
            cnt = {r: sum(1 for x in rn if x["result"] == r) for r in ("PERFECT", "GOOD", "MISS")}
            acc = 100.0 * (cnt["PERFECT"] + 0.5 * cnt["GOOD"]) / len(rn)
            msg += (f" — 리듬 {len(rn)}노트: P {cnt['PERFECT']} / G {cnt['GOOD']} / M {cnt['MISS']}"
                    f" · 정확도 {acc:0.0f}% · 최대 콤보 {r_max}")
        if path:
            msg += f" — 저장: {os.path.basename(path)}"
            if tpath:
                msg += " + _trials"
            msg += " + _answer(경직 정답지)" + (" + _rhythm(노트별)" if rpath else "")
            parts = [f"{FNAME[f]}({sm['grade']}) 경직 {sm['n']}(catch {sm['n_catch']}) · 감지 {sm['hit']}"
                     f"(catch {sm['hit_catch']}) · 오경보 {sm['fa']}"
                     for f, sm in summary.items()]
            if parts:
                msg += "\n" + "   ".join(parts)
        status.set(msg)

    pad = dict(padx=8, pady=3)
    top = ttk.Frame(win); top.pack(fill="x", **pad)
    ttk.Checkbutton(top, text="가상 환자 사용 (환자 장갑 G1 대신)", variable=v_active,
                    command=on_active).pack(side="left")
    r = ttk.Frame(win); r.pack(fill="x", **pad)
    ttk.Label(r, text="방향").pack(side="left")
    ttk.Radiobutton(r, text="양방향 (치료사가 느낌)", value="bi", variable=v_dir,
                    command=on_dir).pack(side="left", padx=6)
    ttk.Radiobutton(r, text="단방향 (치료사 힘 0)", value="uni", variable=v_dir,
                    command=on_dir).pack(side="left", padx=6)
    v_law = tk.StringVar(value=vp.cpl_law)

    def on_law():
        with ctl.lock:
            if vp.cpl_law != v_law.get():
                vp.cpl_law = v_law.get()
                for gl in ctl.gloves:                  # 방식이 바뀌면 결합력을 0 에서 다시 천천히 올린다
                    for f in gl.fingers.values():
                        f.coupling_soft = 0.0

    r = ttk.Frame(win); r.pack(fill="x", **pad)
    ttk.Label(r, text="결합 방식 (가상 환자·사람끼리 공통)").pack(side="left")
    ttk.Radiobutton(r, text="굽힘 차이 (한쪽 줄만)", value="curl", variable=v_law,
                    command=on_law).pack(side="left", padx=6)
    ttk.Radiobutton(r, text="기존 방식 (줄별 지도 목표)", value="map", variable=v_law,
                    command=on_law).pack(side="left", padx=6)
    ttk.Radiobutton(r, text="기존 방식 − 동시 당김 제거", value="map_nocc", variable=v_law,
                    command=on_law).pack(side="left", padx=6)
    r = ttk.Frame(win); r.pack(fill="x", **pad)
    ttk.Label(r, text="환자 프로필").pack(side="left")
    cb = ttk.Combobox(r, textvariable=v_prof, values=list(PROFILES), state="readonly", width=38)
    cb.pack(side="left", padx=6); cb.bind("<<ComboboxSelected>>", on_prof)
    r = ttk.Frame(win); r.pack(fill="x", **pad)
    ttk.Label(r, text="경직 등급").pack(side="left")
    grade_boxes = {}

    def on_grade(_=None):
        with ctl.lock:
            vp.set_grades({f: v_grade[f].get() for f in FIDS if v_grade[f].get() in SPAS})

    def sync_grade_boxes():
        shown = v_answer.get()
        for f, box in grade_boxes.items():
            v_grade[f].set(vp.grades[f] if shown else "?")
            box.config(state=("readonly" if shown else "disabled"))

    def on_random():
        live = [f for f in FIDS if f in ctl.g2.fingers]
        for _ in range(100):
            g = {f: random.choice(SPAS_GRADES) for f in FIDS}
            if len(live) < 2 or len({g[f] for f in live}) > 1:   # 손가락이 둘 이상이면 서로 다르게
                break
        with ctl.lock:
            vp.set_grades(g)
        v_answer.set(False)                          # 무작위는 기본으로 숨긴다 (블라인드)
        sync_grade_boxes()
        status.set("경직 등급을 무작위로 배정했습니다 — 정답은 숨김. [정답 보기]로 확인.")

    for f in ("T", "I", "M"):
        ttk.Label(r, text=FNAME[f]).pack(side="left", padx=(8, 2))
        box = ttk.Combobox(r, textvariable=v_grade[f], values=SPAS_GRADES, state="readonly", width=3)
        box.pack(side="left")
        box.bind("<<ComboboxSelected>>", on_grade)
        grade_boxes[f] = box
    ttk.Button(r, text="무작위 배정", command=on_random).pack(side="left", padx=8)
    ttk.Checkbutton(r, text="정답 보기 (경직 표시)", variable=v_answer,
                    command=sync_grade_boxes).pack(side="left")
    r = ttk.Frame(win); r.pack(fill="x", **pad)
    ttk.Label(r, foreground="#666",
              text="경직을 느끼면 키보드  1 = 엄지 · 2 = 검지 · 3 = 중지  를 누르세요 "
                   "(정지 때 정답지와 맞춰 감지율 계산).  Space/Esc = 비상정지").pack(side="left")
    r = ttk.Frame(win); r.pack(fill="x", **pad)
    ttk.Label(r, text=f"치료사에게 가는 힘 상한 [N] ({F2_CAP_RANGE[0]:g}~{F2_CAP_RANGE[1]:g})").pack(side="left")
    sp = ttk.Spinbox(r, from_=F2_CAP_RANGE[0], to=F2_CAP_RANGE[1], increment=0.5,
                     textvariable=v_cap, width=6, command=on_cap)
    sp.pack(side="left", padx=6); sp.bind("<Return>", on_cap); sp.bind("<FocusOut>", on_cap)
    ttk.Label(r, text=f"   환자에게 가는 힘 상한 [N] ({F1_CAP_RANGE[0]:g}~{F1_CAP_RANGE[1]:g})").pack(side="left")
    sp1 = ttk.Spinbox(r, from_=F1_CAP_RANGE[0], to=F1_CAP_RANGE[1], increment=0.5,
                      textvariable=v_cap1, width=6, command=on_cap1)
    sp1.pack(side="left", padx=6); sp1.bind("<Return>", on_cap1); sp1.bind("<FocusOut>", on_cap1)
    ttk.Checkbutton(r, text="환자 상태를 화면에 표시", variable=v_show).pack(side="left", padx=8)

    # ── 과제 탭 ──
    nb = ttk.Notebook(win); nb.pack(fill="x", padx=8, pady=(4, 2))
    t1 = ttk.Frame(nb); nb.add(t1, text="  기본 과제  ")
    t2 = ttk.Frame(nb); nb.add(t2, text="  리듬 포즈  ")

    r = ttk.Frame(t1); r.pack(fill="x", **pad)
    ttk.Label(r, text="시나리오").pack(side="left")
    cb = ttk.Combobox(r, textvariable=v_scen, values=list(SCENARIOS), state="readonly", width=24)
    cb.pack(side="left", padx=6); cb.bind("<<ComboboxSelected>>", on_scen)
    ttk.Checkbutton(r, text="반복", variable=v_loop, command=on_loop).pack(side="left", padx=6)
    ttk.Label(r, text=f"   물체 과제 제한 시간 [s]").pack(side="left")
    sp = ttk.Spinbox(r, from_=GRASP_LIMIT_RANGE[0], to=GRASP_LIMIT_RANGE[1], increment=5,
                     textvariable=v_limit, width=6, command=on_limit)
    sp.pack(side="left", padx=6); sp.bind("<Return>", on_limit); sp.bind("<FocusOut>", on_limit)
    r = ttk.Frame(t1); r.pack(fill="x", **pad)
    ttk.Button(r, text="시작", command=on_start).pack(side="left", ipadx=14, ipady=4)
    ttk.Button(r, text="정지 + 기록 저장", command=on_stop).pack(side="left", padx=8, ipadx=8, ipady=4)

    r = ttk.Frame(t2); r.pack(fill="x", **pad)
    ttk.Label(r, text="노트 간격").pack(side="left")
    ttk.Combobox(r, textvariable=v_tempo, values=list(RHYTHM_TEMPI), state="readonly",
                 width=10).pack(side="left", padx=6)
    ttk.Label(r, text="  노트 수").pack(side="left")
    ttk.Spinbox(r, from_=NOTES_RANGE[0], to=NOTES_RANGE[1], increment=5, textvariable=v_notes,
                width=5).pack(side="left", padx=6)
    ttk.Label(r, text="  순서 번호").pack(side="left")
    ttk.Spinbox(r, from_=1, to=9999, increment=1, textvariable=v_seed, width=6).pack(side="left", padx=6)
    ttk.Button(r, text="새 순서", command=on_new_seed).pack(side="left", padx=4)
    ttk.Label(r, foreground="#666", text="(같은 번호 = 같은 노트 순서)").pack(side="left", padx=4)
    r = ttk.Frame(t2); r.pack(fill="x", **pad)
    ttk.Label(r, text="보조 방식").pack(side="left")
    for key, label in ASSIST_MODES.items():
        ttk.Radiobutton(r, text=label, value=key, variable=v_assist).pack(side="left", padx=6)
    ttk.Label(r, text="  로봇 강성 [N/°]").pack(side="left")
    ttk.Spinbox(r, from_=K_ROBOT_RANGE[0], to=K_ROBOT_RANGE[1], increment=0.005, textvariable=v_krobot,
                width=6).pack(side="left", padx=6)
    r = ttk.Frame(t2); r.pack(fill="x", **pad)
    ttk.Checkbutton(r, text="slacking (도움을 받을수록 환자가 스스로 덜 함)", variable=v_slack).pack(side="left")
    ttk.Label(r, foreground="#666",
              text="   치료사 (결합) 일 때만 단방향/양방향 선택이 의미 있음").pack(side="left")
    r = ttk.Frame(t2); r.pack(fill="x", **pad)
    ttk.Button(r, text="리듬 시작", command=on_rhythm_start).pack(side="left", ipadx=14, ipady=4)
    ttk.Button(r, text="정지 + 기록 저장", command=on_stop).pack(side="left", padx=8, ipadx=8, ipady=4)

    # 이 창에 포커스가 있어도 비상정지가 먹도록 (Space/Esc 는 원래 메인 창에만 묶여 있음)
    def estop(event=None):
        if app is not None:
            return app._emergency_stop(event)
        ctl.trigger_estop("사용자")
        return "break"
    win.bind("<space>", estop)
    win.bind("<Escape>", estop)

    flash = {}

    def on_mark(event):
        w = event.widget
        try:
            if w.winfo_class() in ("Entry", "TEntry", "Spinbox", "TSpinbox", "TCombobox"):
                return None                          # 숫자 입력 중에는 표시로 쓰지 않는다
        except Exception:
            pass
        f = MARK_KEYS.get(event.char)
        if f is None:
            return None
        with ctl.lock:
            vp.add_mark(f)
            ok = vp.log_on
        if ok:
            flash[f] = 8                              # 화면에 잠깐 표시
        return "break"
    for k in MARK_KEYS:
        win.bind(f"<KeyPress-{k}>", on_mark)
        root.bind(f"<KeyPress-{k}>", on_mark, add="+")

    phase = tk.Label(win, text="-", font=("", 14, "bold"), anchor="w")
    phase.pack(fill="x", padx=10, pady=(6, 0))
    CW, CH = 760, 480
    cv = tk.Canvas(win, width=CW, height=CH, bg="#10162A", highlightthickness=0)
    cv.pack(padx=8, pady=4)

    # ── 숫자 표: 굽힘 정도와 지금 나가고 있는 힘 ──
    tbl = ttk.Frame(win); tbl.pack(fill="x", padx=10, pady=(2, 2))
    heads = ["손가락", "굽힘 (환자 / 나)", "내 장갑 결합력 지령\n굽힘 / 폄 [N]",
             "상한 적용 전\n굽힘 / 폄 [N]", "내 장갑 모터 지령\n굽힘 / 폄 [N]", "환자가 받는 힘 [N]",
             "경직 정답\n등급 · 횟수 · 내 표시"]
    for j, h in enumerate(heads):
        ttk.Label(tbl, text=h, foreground="#555", justify="center", anchor="center"
                  ).grid(row=0, column=j, padx=6, sticky="ew")
        tbl.columnconfigure(j, weight=1)
    cells = {}
    for i, f in enumerate(FIDS, start=1):
        ttk.Label(tbl, text=FNAME[f], font=("", 10, "bold")).grid(row=i, column=0, padx=6)
        for j in range(1, len(heads)):
            lab = ttk.Label(tbl, text="-", font=("Consolas", 10), anchor="center")
            lab.grid(row=i, column=j, padx=6, sticky="ew")
            cells[(f, j)] = lab

    ttk.Label(win, textvariable=status, foreground="#444").pack(fill="x", padx=10)
    ttk.Label(win, foreground="#888", wraplength=740, justify="left",
              text="파란 손 = 가상 환자, 주황 선 = 치료사(나), 흰 점선 = 환자가 하려는 자세. "
                   "결합력 지령 = 환자 때문에 내 장갑에 실제로 더해지는 힘(상한·램프 적용 후). "
                   "모터 지령 = auto-tensioning 까지 합쳐 모터로 나가는 값. "
                   "프로필 수치는 느낌 확인용 임의값입니다.").pack(fill="x", padx=10, pady=(2, 6))

    # ── 손 그림 (옆에서 본 모습, 손가락이 오른쪽을 향함) ──
    def fk(ox, oy, curls):
        """curl → 관절 좌표. 화면 y 는 아래가 +, 굽힘은 아래쪽으로 말린다."""
        out = {}
        for fid, (bx, by, segs, gains, a0, sgn) in HAND_GEOM.items():
            c = max(0.0, min(1.0, curls.get(fid, 0.0)))
            x, y, a = ox + bx, oy + by, a0
            pts = [(x, y)]
            for L, g in zip(segs, gains):
                a += sgn * math.radians(g * c)
                x += L * math.cos(a); y += L * math.sin(a)
                pts.append((x, y))
            out[fid] = pts
        return out

    def draw_hand(ox, oy, curls, color, width, dash=None, palm=None, colors=None):
        if palm:
            cv.create_polygon(ox - 10, oy - 34, ox + 96, oy - 40, ox + 104, oy + 8,
                              ox + 70, oy + 52, ox - 10, oy + 44,
                              fill=palm, outline="", smooth=True)
        chains = fk(ox, oy, curls)
        for fid in ("M", "T", "I"):                       # 검지를 맨 위에 그린다
            if fid not in curls:
                continue
            pts = [v for p_ in chains[fid] for v in p_]
            w = width if fid != "M" else max(1, width - 2)
            col = color if (fid != "M" or dash or width < 8) else "#3A4F8C"
            if colors and fid in colors:
                col = colors[fid]
            cv.create_line(*pts, fill=col, width=w, capstyle="round", joinstyle="round",
                           dash=dash)
        return chains

    def draw_pose_icon(cx, cy, sc, curls, col, live_f, dim="#4A5578"):
        """포즈를 손등 쪽에서 본 손 그림으로. 굽힌 손가락은 짧아지고 끝에 접힘 표시(빈 원).
        약지·새끼는 장갑이 없으니 중지를 따라 흐리게, 꺼진 손가락도 흐리게 그린다."""
        pw, ph = 74 * sc, 64 * sc
        cv.create_rectangle(cx - pw / 2, cy - ph / 2 + 10 * sc, cx + pw / 2, cy + ph / 2 + 10 * sc,
                            fill="#33416E", outline="")
        top = cy - ph / 2 + 14 * sc
        w = max(3, int(15 * sc))
        for fid, dx, L in (("I", -27, 64), ("M", -9, 70), ("R", 9, 64), ("P", 27, 52)):
            c = max(0.0, min(1.0, curls.get(fid if fid in ("I", "M") else "M", 0.0)))
            vis = L * sc * (1.0 - 0.82 * c)
            colr = col if fid in live_f else dim
            x = cx + dx * sc
            cv.create_line(x, top, x, top - vis, fill=colr, width=w, capstyle="round")
            if c > 0.35:                                   # 접힌 손끝
                rr = w / 2
                cv.create_oval(x - rr, top - vis - rr, x + rr, top - vis + rr, fill="#1E2A4D",
                               outline=colr, width=max(1, int(3 * sc)))
        c = max(0.0, min(1.0, curls.get("T", 0.0)))       # 엄지: 펴면 바깥 위로, 굽히면 손바닥 쪽으로
        ang = math.radians(-135.0 + 105.0 * c)
        L = (52.0 - 18.0 * c) * sc
        bx, by = cx - pw / 2 + 6 * sc, cy + ph / 2 - 6 * sc
        cv.create_line(bx, by, bx + L * math.cos(ang), by + L * math.sin(ang),
                       fill=(col if "T" in live_f else dim), width=w, capstyle="round")

    OBJ_REL = {}
    def obj_rel():
        """물체 중심(손 기준)과 반지름 — 물체 두께 curl 에서 검지 끝과 엄지 끝 사이."""
        if not OBJ_REL:
            ch = fk(0.0, 0.0, _OBJ)
            (ix, iy), (tx, ty) = ch["I"][-1], ch["T"][-1]
            OBJ_REL["c"] = ((ix + tx) / 2.0 - 4, (iy + ty) / 2.0 - 4)
            OBJ_REL["r"] = max(14.0, math.hypot(ix - tx, iy - ty) / 2.0 - 8.0)
        return OBJ_REL["c"], OBJ_REL["r"]

    FLOOR_Y, PAD_A, PAD_B, PAD_Y = 392, 300, 620, 300
    ui = dict(fall_x=PAD_A, fall_y=0.0, fall_v=0.0, pose_acc=[], pose_last=None, tick=0)

    def fmt_pair(a, b):
        return f"{a:4.1f} / {b:4.1f}"

    def poll():
        with ctl.lock:
            estop = ctl.estop
            st = {}
            for f in FIDS:
                vf = vp.fingers[f]
                st[f] = (vf.des, vf.curl, vf.f_contact, vf.f_cpl, vf.effort,
                         getattr(vf, "cmd", (0.0, 0.0)), getattr(vf, "raw", (0.0, 0.0)))
            spas = {f: (vp.fingers[f].f_sr, SPAS[vp.fingers[f].grade]["f_sr"], vp.grades[f],
                        vp._open_ev[f] is not None,
                        sum(1 for e in vp.events if e["fid"] == f),
                        sum(1 for _, g in vp.marks if g == f)) for f in FIDS}
            th = {}
            for f in FIDS:
                fu = ctl.g2.fingers.get(f)
                th[f] = (fu.curl, fu.cpl_f, fu.cpl_e, fu.coupling_enabled,
                         fu.flex.applied_N, fu.ext.applied_N) if fu else None
            playing, name, left = vp.playing, vp.phase_name, vp.phase_left
            active, gain, grip = vp.active, vp.bi_gain, vp.grip_ok
            live = tuple(f for f in FIDS if f in ctl.g2.fingers)
            kc1, bc1, kc2, bc2 = (float(ctl.params.get(k, 0.0)) for k in ("KC1", "BC1", "KC2", "BC2"))
            scen_name = vp.scenario_name
            scen = SCENARIOS.get(scen_name) or [("-", 1.0, None, None)]
            is_rhythm = scen_name == RHYTHM
            if is_rhythm:
                t_now = vp.t
                rnotes = [(nt["k"], nt["pose"], dict(nt["tgt"]), nt["hit"], nt["res"])
                          for nt in vp.r_plan if -4.0 < nt["hit"] - t_now < 7.0]
                r_cur = next((nt for nt in vp.r_plan
                              if nt["hit"] - vp.r_move <= t_now <= nt["hit"] + RHYTHM_JUDGE[1]), None)
                r_tgt = dict(r_cur["tgt"]) if r_cur else None
                r_last = vp.r_last
                r_stat = (vp.r_score, vp.r_combo, len(vp.r_notes), len(vp.r_plan),
                          sum(1 for x in vp.r_notes if x["result"] == "PERFECT"),
                          sum(1 for x in vp.r_notes if x["result"] == "GOOD"))
                r_mode = vp.assist_mode()
                drives = {f: vp.fingers[f].drive for f in FIDS}
                r_slack = vp.slacking
            phase_i = min(vp.phase_i, len(scen) - 1)
            tgt_final = scen[phase_i][2]
            g_state, hand_pos, obj_state = vp.g_state, vp.hand_pos, vp.obj_state
            trial_on, trial_t, limit_s = vp.trial_on, vp.trial_t, vp.limit_s
            n_ok, n_try, g_msg, g_msg_ok = vp.n_ok, vp.n_try, vp.g_msg, vp.g_msg_ok
            cur = dict(vp.trial) if vp.trial else None
            last_trials = [dict(t) for t in vp.trials[-3:]]
        if estop and playing:
            on_stop("비상정지로 중단")
        is_grasp = scen_name == GRASP
        show = v_show.get() and active
        mode_now = r_mode if is_rhythm else "therapist"
        if not playing:
            phase.config(text="대기")
        elif is_grasp:
            phase.config(text=name + (f"   (남은 시간 {max(0.0, left):0.1f}s)" if trial_on else ""))
        else:
            phase.config(text=f"{name}   ({left:0.1f}s)")

        cv.delete("all")
        cv.create_rectangle(0, FLOOR_Y, CW, CH, fill="#1B2440", outline="")
        (rx, ry), r = obj_rel()
        hx, hy = 300.0, 200.0
        if is_rhythm:
            hx, hy = 280.0, 170.0
        if is_grasp:
            xa, xb = PAD_A - rx, PAD_B - rx
            y0 = PAD_Y - r - ry
            pos = hand_pos if playing else -1.0
            if pos >= 0.0:
                hx = xa + (xb - xa) * pos
                hy = y0 - 70.0 * math.sin(math.pi * min(1.0, pos)) if g_state == "transport" else y0
            else:
                hx, hy = xa + 120.0 * pos, y0
            for px_, lab in ((PAD_A, "출발"), (PAD_B, "목표")):
                cv.create_rectangle(px_ - 46, PAD_Y, px_ + 46, FLOOR_Y, fill="#2B3760", outline="")
                cv.create_text(px_, PAD_Y + 16, text=lab, fill="#9FB0DD", font=("", 11, "bold"))
            cv.create_oval(PAD_B - r - 6, PAD_Y - 2 * r - 6, PAD_B + r + 6, PAD_Y + 6,
                           outline="#7CE0A0", dash=(4, 4))

        chains = None
        if is_rhythm and playing and r_tgt is not None:     # 지금 맞춰야 하는 포즈 (초록 점선)
            draw_hand(hx, hy, {f: r_tgt[f] for f in live}, "#7CE0A0", 3, dash=(6, 4))
        if show:
            answer = v_answer.get()
            colors = None
            if answer:                                   # 경직 힘만큼 빨갛게
                colors = {}
                for f in live:
                    fs, fmax = spas[f][0], spas[f][1]
                    a = 0.0 if fmax <= 0 else max(0.0, min(1.0, fs / (0.6 * fmax)))
                    base = (0x5B, 0x7F, 0xE0) if f != "M" else (0x3A, 0x4F, 0x8C)
                    red = (0xFF, 0x4D, 0x4D)
                    colors[f] = "#%02X%02X%02X" % tuple(int(b + (r_ - b) * a) for b, r_ in zip(base, red))
            chains = draw_hand(hx, hy, {f: st[f][1] for f in live}, "#5B7FE0", 15, palm="#3A4F8C",
                               colors=colors)
            if answer:
                for f in live:
                    tx, ty = chains[f][-1]
                    lab = f"{FNAME[f]} {spas[f][2]}"
                    if spas[f][3]:
                        lab += "  경직!"
                    cv.create_text(tx + 16, ty + (14 if f == "T" else -14), anchor="w",
                                   text=lab, font=("", 10, "bold"),
                                   fill=("#FF6B6B" if spas[f][3] else "#C9D3F0"))
            if live and all(st[f][0] is not None for f in live):
                draw_hand(hx, hy, {f: st[f][0] for f in live}, "#FFFFFF", 2, dash=(3, 4))
            for f in live:                                   # 환자가 힘쓰는 정도 = 손끝 불빛
                tx, ty = chains[f][-1]
                e = st[f][4]
                if e > 0.05:
                    rr = 4 + 7 * e
                    cv.create_oval(tx - rr, ty - rr, tx + rr, ty + rr, outline="#FFD166", width=2)
        if any(v is not None for v in th.values()):
            draw_hand(hx, hy, {f: th[f][0] for f in FIDS if th[f]}, "#FF9F43", 4)

        if is_grasp:
            # 물체 위치
            if obj_state in ("held", "slip"):
                ox, oy = hx + rx, hy + ry
                ui.update(fall_x=ox, fall_y=oy, fall_v=0.0)
            elif obj_state == "fallen":
                ui["fall_v"] += 2.2
                ui["fall_y"] = min(FLOOR_Y - r, ui["fall_y"] + ui["fall_v"])
                ox, oy = ui["fall_x"], ui["fall_y"]
            elif obj_state in ("placed", "stuck"):
                ox, oy = PAD_B, PAD_Y - r
            else:
                ox, oy = PAD_A, PAD_Y - r
            col = {"held": "#FFD166", "slip": "#FF7B7B", "placed": "#7CE0A0",
                   "fallen": "#FF7B7B", "stuck": "#8891AA"}.get(obj_state, "#E8ECF8")
            cv.create_oval(ox - r, oy - r, ox + r, oy + r, fill=col, outline="")
            cv.create_text(CW - 14, 20, anchor="e", fill="#E8ECF8", font=("", 14, "bold"),
                           text=f"성공 {n_ok} / 시도 {n_try}")
            # 제한 시간 막대
            if trial_on:
                frac = max(0.0, min(1.0, 1.0 - trial_t / max(limit_s, 1e-6)))
                cv.create_rectangle(CW - 214, 36, CW - 14, 46, outline="#48557F")
                cv.create_rectangle(CW - 214, 36, CW - 214 + 200 * frac, 46, outline="",
                                    fill=("#7CE0A0" if frac > 0.5 else "#FFD166" if frac > 0.2 else "#FF7B7B"))
                cv.create_text(CW - 14, 60, anchor="e", fill="#9FB0DD",
                               text=f"{trial_t:0.1f} / {limit_s:g} s")
            # 단계별 걸린 시간 (이번 시도 → 없으면 직전 시도)
            tr = cur if trial_on else (last_trials[-1] if last_trials else None)
            if tr:
                def tv(k):
                    return f"{tr[k]:0.1f}s" if tr.get(k) not in ("", None) else "…"
                cv.create_text(CW - 14, 84, anchor="ne", fill="#C9D3F0", justify="right",
                               text=f"시도 {tr['trial']}\n손 벌림 {tv('t_open_s')}\n잡음 {tv('t_grasp_s')}\n"
                                    f"옮김 {tv('t_transport_s')}\n미끄러짐 {tr['slips']}회")
            if g_msg and not trial_on:
                cv.create_text(CW / 2, 60, text=g_msg, font=("", 18, "bold"),
                               fill=("#7CE0A0" if g_msg_ok else "#FF7B7B"))
            elif playing and g_state == "wait_open":
                tight = [FNAME[f] for f in live if st[f][1] > _OBJ[f] - APERTURE_MARGIN]
                cv.create_text(CW / 2, 60, font=("", 14, "bold"), fill="#FFD166",
                               text="더 펴야 함: " + "·".join(tight) if tight else "")
            elif playing and g_state in ("grasp", "transport"):
                txt = ("꽉 잡고 있음" if grip else
                       "미끄러짐 — 다시 쥐어야 이동" if obj_state == "slip" else "쥐는 힘 부족")
                cv.create_text(CW / 2, 60, font=("", 14, "bold"),
                               fill=("#7CE0A0" if grip else "#FFD166"), text=txt)
            elif playing and g_state == "release":
                cv.create_text(CW / 2, 60, font=("", 14, "bold"), fill="#FFD166",
                               text="손을 펴서 놓아야 함")
        elif playing and tgt_final is not None:
            err = sum(abs(st[f][1] - tgt_final[f]) for f in live) / max(1, len(live))
            score = max(0.0, 1.0 - err / 0.4) * 100.0
            if left < 0.15 and ui["pose_last"] != phase_i:      # 단계가 끝날 때의 점수를 기록
                ui["pose_last"] = phase_i
                ui["pose_acc"].append(score)
            cv.create_text(CW - 14, 20, anchor="e", fill="#E8ECF8", font=("", 14, "bold"),
                           text=f"자세 일치 {score:0.0f}%")
            cv.create_rectangle(CW - 214, 36, CW - 14, 46, outline="#48557F")
            cv.create_rectangle(CW - 214, 36, CW - 214 + 2 * score, 46, outline="",
                                fill=("#7CE0A0" if score > 80 else "#FFD166" if score > 50 else "#FF7B7B"))
            if ui["pose_acc"]:
                cv.create_text(CW - 14, 62, anchor="e", fill="#9FB0DD",
                               text=f"평균 {sum(ui['pose_acc']) / len(ui['pose_acc']):0.0f}%"
                                    f"  ({len(ui['pose_acc'])}개 자세)")
        if is_rhythm:
            # 노트 레인: 오른쪽에서 다가와 판정선(세로선)에 닿는 순간 그 포즈여야 한다
            ly0, ly1, jx, pxs = 352, 476, 250, 110.0
            cv.create_rectangle(0, ly0, CW, ly1, fill="#141C34", outline="")
            cv.create_line(jx, ly0 - 6, jx, ly1 + 2, fill="#7CE0A0", width=3)
            cv.create_text(jx, ly0 - 14, text="판정선", fill="#7CE0A0", font=("", 9, "bold"))
            if playing:
                for k, pose, tgt, hit, res in rnotes:
                    x = jx + (hit - t_now) * pxs
                    if x < -60 or x > CW + 60:
                        continue
                    col = {"PERFECT": "#7CE0A0", "GOOD": "#FFD166", "MISS": "#FF7B7B"}.get(res, "#E8ECF8")
                    cv.create_rectangle(x - 56, ly0 + 6, x + 56, ly1 - 6, outline=col, width=2,
                                        fill="#1E2A4D")
                    cv.create_text(x, ly0 + 20, text=pose, fill=col,
                                   font=("", 11 if len(pose) <= 5 else 9, "bold"))
                    draw_pose_icon(x, ly0 + 80, 0.5, tgt, col, live)
                # 다음에 할 포즈 카드 (크게)
                nxt = r_cur or next(((k, pose, tgt, hit, res) for k, pose, tgt, hit, res in rnotes
                                     if hit >= t_now), None)
                if r_cur:
                    nxt = (r_cur["k"], r_cur["pose"], r_cur["tgt"], r_cur["hit"], r_cur["res"])
                if nxt:
                    k, pose, tgt, hit, res = nxt
                    cx0, cy0 = 662, 230
                    cv.create_rectangle(cx0 - 92, cy0 - 120, cx0 + 92, cy0 + 100, outline="#7CE0A0",
                                        width=2, fill="#16213F")
                    dt_hit = hit - t_now
                    cv.create_text(cx0, cy0 - 104, fill="#9FB0DD", font=("", 10),
                                   text=(f"{dt_hit:0.1f}초 후" if dt_hit > 0.05 else "지금!"))
                    cv.create_text(cx0, cy0 - 80, text=pose, fill="#7CE0A0", font=("", 18, "bold"))
                    draw_pose_icon(cx0, cy0 + 30, 1.0, tgt, "#7CE0A0", live)
                    cv.create_text(cx0, cy0 + 86, fill="#C9D3F0", font=("", 9),
                                   text="  ".join(f"{FNAME[f]} {tgt[f] * 100:0.0f}%" for f in live))
                if r_last and t_now - r_last[2] < 0.9:      # 직전 판정
                    res, miss_f, _, row = r_last
                    col = {"PERFECT": "#7CE0A0", "GOOD": "#FFD166"}.get(res, "#FF7B7B")
                    txt = res + ("  (" + "·".join(miss_f) + ")" if miss_f else "")
                    cv.create_text(CW / 2 + 60, 62, text=txt, fill=col, font=("", 22, "bold"))
            score, combo, n_done, n_all, n_p, n_g = r_stat
            acc_pct = 100.0 * (n_p + 0.5 * n_g) / max(1, n_done)
            cv.create_text(CW - 14, 20, anchor="e", fill="#E8ECF8", font=("", 14, "bold"),
                           text=f"점수 {score}   콤보 {combo}")
            cv.create_text(CW - 14, 42, anchor="e", fill="#9FB0DD",
                           text=f"{n_done}/{n_all} 노트 · 정확도 {acc_pct:0.0f}%")
            if r_last:                                      # 직전 노트의 환자 기여
                row = r_last[3]
                parts = []
                for f in live:
                    c = row.get(f"{f}_contrib")
                    if c not in ("", None):
                        parts.append(f"{FNAME[f]} {c * 100:0.0f}%")
                if parts:
                    cv.create_text(CW - 14, 62, anchor="e", fill="#9FB0DD",
                                   text="환자 기여(직전): " + "  ".join(parts))
            if r_slack and show:
                cv.create_text(CW - 14, 82, anchor="e", fill="#9FB0DD",
                               text="환자 의욕: " + "  ".join(f"{FNAME[f]} {drives[f] * 100:0.0f}%" for f in live))
            cv.create_text(CW - 14, 100, anchor="e", fill="#9FB0DD",
                           text="보조: " + ASSIST_MODES[mode_now])
        if not playing:
            ui["pose_last"] = None

        # 내가 '느꼈다' 표시한 손가락 (잠깐)
        for i, f in enumerate([f for f in FIDS if flash.get(f, 0) > 0]):
            flash[f] -= 1
            cv.create_text(CW / 2, CH - 40 - 22 * i, text=f"✋ {FNAME[f]} 경직 표시",
                           fill="#FFD166", font=("", 14, "bold"))

        # 내 장갑이 받는 힘 (막대)
        cv.create_text(14, 20, anchor="w", fill="#9FB0DD", font=("", 11, "bold"),
                       text=("양방향" if gain > 0.5 else "단방향") + f"  (치료사 힘 {gain * 100:0.0f}%)")
        for i, f in enumerate(FIDS):
            if th[f] is None:
                continue
            _, cf, ce, con = th[f][:4]
            x0, ym = 20 + i * 44, 110
            cv.create_text(x0 + 12, 44, text=FNAME[f], fill="#E8ECF8")
            cv.create_rectangle(x0, ym - 50, x0 + 24, ym + 50, outline="#48557F")
            cv.create_line(x0, ym, x0 + 24, ym, fill="#48557F")
            k = 50.0 / F2_CAP_RANGE[1]
            cv.create_rectangle(x0 + 2, ym, x0 + 22, ym - min(50, cf * k), fill="#FF9F43", outline="")
            cv.create_rectangle(x0 + 2, ym, x0 + 22, ym + min(50, ce * k), fill="#54C6EB", outline="")
            cv.create_text(x0 + 12, ym + 62, fill="#9FB0DD",
                           text=("%.1f" % max(cf, ce)) if con else "꺼짐")
        cv.create_text(14, 196, anchor="w", fill="#9FB0DD",
                       text=f"KC1 {kc1:g}  BC1 {bc1:g}  (내가 환자에게)\nKC2 {kc2:g}  BC2 {bc2:g}  (환자가 나에게)")
        why = []
        if not active:
            why.append("[가상 환자 사용]이 꺼져 있음")
        else:
            if mode_now != "therapist":
                why.append(f"보조 = {ASSIST_MODES[mode_now]} → 치료사와 연결 끊김 (내 장갑 결합력 0)")
            elif not any(v and v[3] for v in th.values()):
                why.append("G2 결합(양방향) 체크가 꺼져 있음 → 힘 전달 없음")
            if mode_now == "therapist" and kc2 <= 0.0 and bc2 <= 0.0:
                why.append("KC2 = BC2 = 0 → 양방향이어도 나에게 오는 힘이 0 (프리셋을 고르세요)")
            if mode_now == "therapist" and kc1 <= 0.0 and bc1 <= 0.0:
                why.append("KC1 = BC1 = 0 → 내 움직임이 환자에게 전달되지 않음")
            if mode_now == "therapist" and gain <= 0.0 and v_dir.get() == "bi":
                why.append("양방향 전환 중…")
            if not playing:
                why.append("[시작] 전 — 환자는 힘을 뺀 상태")
        for i, line in enumerate(why):
            cv.create_text(CW / 2, 92 + i * 20, text="⚠ " + line, fill="#FFB36B",
                           font=("", 11, "bold"))

        # 숫자 표는 초당 10번만 갱신 (읽을 수 있게)
        ui["tick"] += 1
        if ui["tick"] % 3 == 0:
            for f in FIDS:
                if th[f] is None:
                    for j in range(1, len(heads)):
                        cells[(f, j)].config(text="(제외)" if j == 1 else "")
                    continue
                tc, _, _, con, mf, me = th[f]
                des, pc, fcon, fcpl, eff, cmd, raw = st[f]
                capped = raw[0] > cmd[0] + 0.05 or raw[1] > cmd[1] + 0.05
                cells[(f, 1)].config(text=(f"{pc * 100:3.0f}% / {tc * 100:3.0f}%" if show
                                           else f"  —  / {tc * 100:3.0f}%"))
                cells[(f, 2)].config(text=fmt_pair(*cmd) if con else "결합 꺼짐")
                cells[(f, 3)].config(text=fmt_pair(*raw) + (" ◀" if capped else ""),
                                     foreground=("#c62828" if capped else ""))
                cells[(f, 4)].config(text=fmt_pair(mf, me))
                gr, n_ev, n_mk = spas[f][2], spas[f][4], spas[f][5]
                if v_answer.get():
                    cells[(f, 6)].config(text=f"{gr} · {n_ev}회 · {n_mk}",
                                         foreground=("#c62828" if spas[f][3] else ""))
                else:
                    cells[(f, 6)].config(text=f"? · ? · {n_mk}", foreground="")
                if not show:
                    cells[(f, 5)].config(text="—")
                else:
                    cells[(f, 5)].config(text=(f"굽힘 {fcpl:4.1f}" if fcpl >= 0.05 else
                                               f"폄 {-fcpl:4.1f}" if fcpl <= -0.05 else "0.0"))
        win.after(33, poll)

    sync_grade_boxes()
    poll()
    return win


def load_control(path):
    spec = importlib.util.spec_from_file_location("glove_control", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["glove_control"] = mod
    spec.loader.exec_module(mod)
    return mod


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--control", default=os.path.join(here, "1015_v9_glove_cal.py"),
                    help="기존 제어 프로그램 파일")
    ap.add_argument("--log-dir", default=os.path.join(here, "vpatient_logs"))
    args = ap.parse_args()
    if not os.path.isfile(args.control):
        raise SystemExit(f"제어 파일을 찾을 수 없습니다: {args.control}")

    mod = load_control(args.control)
    for name in ("ControlThread", "App", "tk", "ttk"):
        if not hasattr(mod, name):
            raise SystemExit(f"제어 파일에 {name} 이(가) 없습니다 — 버전이 다른 파일일 수 있습니다.")
    vp = VirtualPatient()
    install(mod, vp)

    ctl = mod.ControlThread()
    ctl.start()
    root = mod.tk.Tk()
    app = mod.App(root, ctl)
    build_panel(mod.tk, mod.ttk, root, ctl, vp, args.log_dir, app)
    try:
        root.mainloop()
    finally:
        ctl.running = False
        if ctl.is_alive():
            ctl.join(timeout=1.0)


if __name__ == "__main__":
    main()