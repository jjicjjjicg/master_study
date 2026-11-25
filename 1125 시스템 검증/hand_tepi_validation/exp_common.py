"""
exp_common.py — Hand TEPI 시스템 검증 실험 공통 모듈

기존 제어 코드(1015_v9_glove_cal.py)를 수정하지 않고 그대로 불러와서 쓴다.
  - ControlThread 를 상속해 두 군데에만 끼어든다.
      _apply_run : 원래 결합/Auto 계산 뒤, '모터로 움직이는 가상 사람(CurlServo)'과
                   실험용 모니터(불안정 감지 등)를 적용
      _tick      : (sim 이면) 가상 손가락을 한 스텝 진행, 끝나면 실험 로그 한 줄 기록
  - 장력 값은 전부 '전류 지령으로 환산한 모터단 장력'이다. 손끝 힘이 아니다.

공통 사용:
  python e1_stability.py --finger I --params params.json
  python e1_stability.py --sim --no-gui      # 하드웨어 없이 로직만 시험
"""

import os
import sys
import csv
import json
import math
import time
import queue
import random
import argparse
import threading
import importlib.util

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CTRL = os.path.join(HERE, "1015_v9_glove_cal.py")
FINGER_CHOICES = ("I", "M", "T")


# ═══════════════════════════════════════════════════════ 제어 모듈 불러오기

def load_control_module(path=None):
    """파일 이름이 숫자로 시작해도 import 되도록 경로로 직접 불러온다."""
    path = path or os.environ.get("TEPI_CTRL") or DEFAULT_CTRL
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"제어 코드를 찾을 수 없습니다: {path}\n"
            f"--ctrl 로 경로를 주거나 이 폴더에 1015_v9_glove_cal.py 를 두세요.")
    spec = importlib.util.spec_from_file_location("tepi_ctrl", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["tepi_ctrl"] = mod
    spec.loader.exec_module(mod)
    return mod


# ═══════════════════════════════════════════════════════ 기준 궤적

class RefSource:
    """시간 → 목표 curl. 실험 스레드가 func 를 바꾸면 그 시점부터 t=0 으로 다시 센다."""

    def __init__(self):
        self.func = None
        self.t_start = 0.0
        self.value = float("nan")
        self.dvalue = 0.0

    def set(self, func, t_now):
        self.t_start = t_now
        self.func = func

    def clear(self):
        self.func = None
        self.value = float("nan")
        self.dvalue = 0.0

    def eval(self, t_now):
        if self.func is None:
            self.value, self.dvalue = float("nan"), 0.0
            return self.value, self.dvalue
        tr = t_now - self.t_start
        c = self.func(tr)
        h = 1e-3
        self.value = c
        self.dvalue = (self.func(tr + h) - c) / h
        return self.value, self.dvalue


def traj_hold(c):
    return lambda t: c


def traj_ramp(c0, c1, T):
    return lambda t: c0 + (c1 - c0) * min(1.0, max(0.0, t / max(T, 1e-6)))


def traj_sine(mid, amp, freq, lead_in=1.0):
    """mid 에서 시작해 lead_in 초 동안 진폭을 0→amp 로 키운 뒤 일정 진폭 사인."""
    def f(t):
        env = min(1.0, t / lead_in) if lead_in > 0 else 1.0
        return mid + env * amp * math.sin(2 * math.pi * freq * t)
    return f


def traj_triangle(c_lo, c_hi, period):
    def f(t):
        ph = (t % period) / period
        x = 2 * ph if ph < 0.5 else 2 * (1 - ph)
        return c_lo + (c_hi - c_lo) * x
    return f


def traj_pulse(c0, delta, t_wait, t_ramp, t_hold):
    """대기 → ramp → 유지 → ramp 복귀 (지각 실험 자극)."""
    def f(t):
        if t < t_wait:
            return c0
        t -= t_wait
        if t < t_ramp:
            return c0 + delta * t / t_ramp
        t -= t_ramp
        if t < t_hold:
            return c0 + delta
        t -= t_hold
        if t < t_ramp:
            return c0 + delta * (1 - t / t_ramp)
        return c0
    return f


def traj_hold_cycles(c_lo, c_hi, t_up, t_hold, t_down, t_rest):
    """ramp up → hold → ramp down → rest 를 반복 (저항 전달 실험)."""
    period = t_up + t_hold + t_down + t_rest

    def f(t):
        t = t % period
        if t < t_up:
            return c_lo + (c_hi - c_lo) * t / t_up
        t -= t_up
        if t < t_hold:
            return c_hi
        t -= t_hold
        if t < t_down:
            return c_hi - (c_hi - c_lo) * t / t_down
        return c_lo
    return f, period


# ═══════════════════════════════════════════════════════ 모터로 움직이는 가상 사람

class CurlServo:
    """한 손가락을 curl 기준으로 위치 제어 (굽힘 텐던/폄 텐던 중 한쪽만 추가로 당김).

    u = kp·(ref − curl) + kd·(dref − curl_vel)
    u > 0 → FLEX 에 t_base + u,  u < 0 → EXT 에 t_base + |u|
    kp [N/curl], kd [N·s/curl]. 실제 리그에서 반드시 작은 값부터 튜닝할 것.
    """

    def __init__(self, ref: RefSource, kp=20.0, kd=0.8, t_base=0.3, t_max=6.0):
        self.ref = ref
        self.kp, self.kd = kp, kd
        self.t_base, self.t_max = t_base, t_max
        self.cocon = 0.0          # 소리 마스킹용 동시수축 (양쪽에 같은 값 추가)
        self.u = 0.0

    def command(self, f, t_now):
        r, dr = self.ref.eval(t_now)
        if math.isnan(r):
            return 0.0, 0.0
        u = self.kp * (r - f.curl) + self.kd * (dr - f.curl_vel)
        self.u = u
        fn = self.t_base + self.cocon + max(0.0, u)
        en = self.t_base + self.cocon + max(0.0, -u)
        return min(self.t_max, fn), min(self.t_max, en)


# ═══════════════════════════════════════════════════════ 시뮬레이션 (하드웨어 없이 로직 시험)

class SimFinger:
    """curl 좌표의 간단한 2차 모델. 단위는 실제 물리량이 아니라 '그럴듯한' 값."""

    def __init__(self):
        self.c = 0.0
        self.v = 0.0
        self.a = 6.0          # curl/s² per N (순 장력)
        self.b = 4.0          # 감쇠
        self.k = 0.0          # 더미 손가락 스프링 [N-eq per curl]
        self.c_rest = 0.0
        self.fric = 0.3       # 정지 마찰 [N-eq]
        self.mode = "free"    # free | clamp | human
        self.clamp_c = 0.4
        self.human_ref = None  # callable(t) → curl
        self.kh = 60.0
        self.bh = 8.0

    def step(self, Tf, Te, t, dt):
        if self.mode == "clamp":
            self.c, self.v = self.clamp_c, 0.0
            return
        n = 4
        h = dt / n
        for _ in range(n):
            net = (Tf - Te) - self.k * (self.c - self.c_rest)
            if abs(self.v) < 1e-3 and abs(net) < self.fric and self.mode != "human":
                self.v = 0.0
                continue
            acc = self.a * (net - math.copysign(self.fric, self.v if self.v else net)) - self.b * self.v
            if self.mode == "human" and self.human_ref is not None:
                acc += self.kh * (self.human_ref(t) - self.c) - self.bh * self.v
            self.v += acc * h
            self.c += self.v * h
            if self.c < 0.0:
                self.c, self.v = 0.0, max(0.0, self.v)
            if self.c > 1.0:
                self.c, self.v = 1.0, min(0.0, self.v)


class SimPlant:
    SF, SE = 300.0, 280.0      # 합성 지도 스풀 이동량 [deg] (0→full flexion)
    STRETCH = 4.0              # deg/N 직렬 탄성

    def __init__(self, ctl):
        self.fingers = {}
        for g in ctl.gloves:
            for fid in g.fingers:
                self.fingers[(g.short, fid)] = SimFinger()
        self.weights = {}      # motor key → dict(W, Ff, x)  (E0 추 실험)

    def get(self, gs, fid):
        return self.fingers[(gs, fid)]

    def step(self, ctl, dt):
        t = ctl.t_now
        q = 360.0 / (1024 * 4)
        for g in ctl.gloves:
            for fid, f in g.fingers.items():
                s = self.fingers[(g.short, fid)]
                Tf = max(0.0, f.flex.applied_N)
                Te = max(0.0, f.ext.applied_N)
                s.step(Tf, Te, t, dt)
                fd = self.SF * s.c + self.STRETCH * Tf
                ed = self.SE * s.c - self.STRETCH * Te
                for m, d in ((f.flex, fd), (f.ext, ed)):
                    w = self.weights.get(m.key)
                    if w is not None:
                        T = max(0.0, m.applied_N)
                        if T > w["W"] + w["Ff"]:
                            w["x"] += 40.0 * dt
                        elif T < w["W"] - w["Ff"] and w["x"] > 0:
                            w["x"] = max(0.0, w["x"] - 40.0 * dt)
                        d = w["x"]
                    d = round(d / q) * q
                    m.deg_raw = d / (m.enc_sign or 1)


def synthetic_maps(ctl):
    for g in ctl.gloves:
        for f in g.fingers.values():
            raw = [(i, SimPlant.SF * i / 12, SimPlant.SE * i / 12) for i in range(13)]
            f.map.set_branches(raw, raw, {"synthetic": True})
            f.map.name = f"sim_{f.key}"


# ═══════════════════════════════════════════════════════ 로그

LOG_FIELDS_G = ("curl", "curl_vel", "motion", "flex_deg", "ext_deg", "flex_vel",
                "ext_vel", "flex_tgtN", "ext_tgtN", "flex_N", "ext_N",
                "cpl_f", "cpl_e", "auto_on", "cpl_on")


class Recorder:
    """제어 루프 한 틱마다 한 줄. 쓰기는 별도 스레드."""

    def __init__(self, path, fid):
        self.path = path
        self.fid = fid
        self.q = queue.Queue()
        self.fp = open(path, "w", newline="", encoding="utf-8-sig")
        self.w = csv.writer(self.fp)
        head = ["t", "unix_time_s", "dt", "mode", "phase", "trial", "cond",
                "event", "ref", "flag", "estop", "KC1", "BC1", "KC2", "BC2"]
        for gs in ("G1", "G2"):
            head += [f"{gs}_{k}" for k in LOG_FIELDS_G]
        self.w.writerow(head)
        self.on = True
        self._th = threading.Thread(target=self._drain, daemon=True)
        self._th.start()

    def push(self, ctl, now, dt):
        if not self.on:
            return
        p = ctl.params
        lab = ctl.labels
        row = [round(ctl.t_now, 5), round(time.time(), 5), round(dt, 5), ctl.mode,
               lab.get("phase", ""), lab.get("trial", ""), lab.get("cond", ""),
               lab.pop("event", "") if "event" in lab else "",
               ctl.ref.value, int(ctl.flag), int(ctl.estop),
               p["KC1"], p["BC1"], p["KC2"], p["BC2"]]
        for g in (ctl.g1, ctl.g2):
            f = g.fingers.get(self.fid)
            if f is None:
                row += [""] * len(LOG_FIELDS_G)
                continue
            row += [f.curl, f.curl_vel, f.motion, f.flex.deg, f.ext.deg,
                    f.flex.vel, f.ext.vel, f.flex.target_N, f.ext.target_N,
                    f.flex.applied_N, f.ext.applied_N, f.cpl_f, f.cpl_e,
                    int(f.auto_enabled), int(f.coupling_enabled)]
        self.q.put(row)

    def _drain(self):
        while self.on or not self.q.empty():
            try:
                row = self.q.get(timeout=0.2)
            except queue.Empty:
                continue
            self.w.writerow([f"{v:.5g}" if isinstance(v, float) else v for v in row])

    def close(self):
        self.on = False
        self._th.join(timeout=2.0)
        self.fp.close()


# ═══════════════════════════════════════════════════════ 실험용 컨트롤러

def make_controller(mod, fid, sim=False):
    Base = mod.ControlThread

    class ExpController(Base):
        def __init__(self):
            super().__init__()
            self.fid = fid
            self.t_now = 0.0
            self.ref = RefSource()          # 로그/가이드 창에 보이는 목표 curl
            self.drivers = {}               # "G1"/"G2" → CurlServo
            self.monitors = []              # fn(ctl, dt), run 모드에서만
            self.labels = {"phase": "", "trial": "", "cond": "", "msg": ""}
            self.flag = 0                   # 실험별 표시 (예: 불안정 감지)
            self.rec = None
            self.sim = SimPlant(self) if sim else None

        def finger(self, gs):
            g = self.g1 if gs == "G1" else self.g2
            return g.fingers[self.fid]

        def _apply_run(self, p, dt):
            super()._apply_run(p, dt)
            for gs, servo in list(self.drivers.items()):
                f = self.finger(gs)
                f.set_manual(*servo.command(f, self.t_now))
            for m in list(self.monitors):
                m(self, dt)

        def _tick(self, now, dt):
            self.t_now = now - self.t0
            self.ref.eval(self.t_now)
            if self.sim is not None:
                self.sim.step(self, dt)
            super()._tick(now, dt)
            if self.rec is not None:
                self.rec.push(self, now, dt)

    return ExpController()


# ═══════════════════════════════════════════════════════ 세션 준비/정리

def add_common_args(ap, need_finger=True):
    ap.add_argument("--ctrl", default=None, help="제어 코드 경로 (기본: 같은 폴더의 1015_v9_glove_cal.py)")
    ap.add_argument("--map-dir", default="tendon_maps", help="G1.json / G2.json 이 있는 폴더")
    if need_finger:
        ap.add_argument("--finger", default="I", choices=FINGER_CHOICES, help="I=검지 M=중지 T=엄지")
    ap.add_argument("--params", default=None, help="튜닝된 제어 파라미터 JSON (KP1, KI1, T_BASE ...)")
    ap.add_argument("--out", default="results", help="결과 폴더")
    ap.add_argument("--sim", action="store_true", help="하드웨어 없이 가상 손가락으로 실행")
    ap.add_argument("--no-gui", action="store_true", help="가이드 창 없이 콘솔만")
    ap.add_argument("--analyze-only", default=None, help="기존 로그 폴더(또는 csv)만 다시 분석")
    return ap


class Session:
    """한 실험 실행 단위: 컨트롤러, 로그, 결과 폴더."""

    def __init__(self, name, args, need_maps=True):
        self.name = name
        self.args = args
        self.sim = bool(getattr(args, "sim", False))
        self.mod = load_control_module(args.ctrl)
        self.fid = getattr(args, "finger", "I")
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.outdir = os.path.join(args.out, f"{name}_{stamp}")
        os.makedirs(self.outdir, exist_ok=True)
        self.ctl = make_controller(self.mod, self.fid, sim=self.sim)
        self.meta = {"experiment": name, "start": stamp, "finger": self.fid,
                     "sim": self.sim, "args": {k: v for k, v in vars(args).items()}}
        if args.params:
            with open(args.params, "r", encoding="utf-8") as fp:
                pr = json.load(fp)
            pr = {k: v for k, v in pr.items() if not k.startswith("_")}
            self.ctl.set_params(pr)
            self.meta["params_file"] = pr
        if need_maps:
            if self.sim:
                synthetic_maps(self.ctl)
            else:
                load_glove_maps(self.ctl, args.map_dir)
            self.meta["maps"] = map_spans(self.ctl)
        self.log_path = os.path.join(self.outdir, "log.csv")
        self.ctl.rec = Recorder(self.log_path, self.fid)
        self.ctl.start()
        time.sleep(0.3)

    # ── 편의 함수 ──
    @property
    def f1(self):
        return self.ctl.finger("G1")

    @property
    def f2(self):
        return self.ctl.finger("G2")

    def ask(self, msg):
        """사용자 확인. sim 이면 바로 진행."""
        self.ctl.labels["msg"] = msg
        if self.sim:
            print(f"[SIM] {msg}")
            return ""
        return input(f"\n▶ {msg}\n  (준비되면 Enter) ")

    def event(self, name):
        self.ctl.labels["event"] = name

    def end_trial(self):
        """TRIAL_END 이벤트를 남기고 trial 라벨을 비운다 (쉬는 구간이 분석에 섞이지 않게)."""
        self.ctl.labels["trial"] = ""
        self.ctl.labels["event"] = "TRIAL_END"

    def label(self, **kw):
        self.ctl.labels.update(kw)

    def wait(self, seconds, show=None):
        t_end = time.perf_counter() + seconds
        last = -1
        while time.perf_counter() < t_end:
            if self.ctl.estop:
                return False
            rem = int(t_end - time.perf_counter())
            if show and rem != last:
                print(f"\r  {show} {rem:3d}s", end="", flush=True)
                last = rem
            time.sleep(0.02)
        if show:
            print()
        return True

    def run_zero(self, pose_msg="양쪽 손가락을 자연스럽게 편 자세로 두세요 (0점 = full extension)"):
        self.ask(pose_msg)
        ctl = self.ctl
        with ctl.lock:
            ctl.stop_all()
            ctl.mode = "run"
        ctl.request_run_zero()
        t0 = time.perf_counter()
        time.sleep(0.2)
        while ctl.run_zero_active or ctl.run_zero_req:
            if time.perf_counter() - t0 > 40:
                raise RuntimeError("0점 시간 초과")
            time.sleep(0.05)
        if not (self.f1.map_aligned and self.f2.map_aligned):
            raise RuntimeError(f"0점 실패: {ctl.run_zero_msg}")
        print(f"  0점 완료 ({ctl.run_zero_msg})")

    def set_gains(self, **kw):
        self.ctl.set_params(kw)

    def enable(self, gs, kind, on=True):
        f = self.ctl.finger(gs)
        err = self.ctl.set_control_enabled(f, kind, on)
        if err:
            raise RuntimeError(f"{gs} {kind}: {err}")

    def disable_all(self):
        with self.ctl.lock:
            self.ctl.drivers.clear()
            self.ctl.monitors.clear()
            self.ctl.ref.clear()
        self.ctl.stop_all()

    def set_ref(self, func):
        with self.ctl.lock:
            self.ctl.ref.set(func, self.ctl.t_now)

    def drive(self, gs, servo):
        with self.ctl.lock:
            self.ctl.drivers[gs] = servo

    def clear_estop_interactive(self):
        print(f"\n!! 비상정지: {self.ctl.estop_msg}")
        self.ask("원인을 확인하고 안전하면 Enter 로 해제합니다")
        if not self.ctl.clear_estop():
            raise RuntimeError("제어 오류로 해제 불가 — 프로그램 재시작 필요")
        with self.ctl.lock:
            self.ctl.mode = "run"

    def save_meta(self):
        self.meta["hz_last"] = round(self.ctl.hz, 1)
        with open(os.path.join(self.outdir, "meta.json"), "w", encoding="utf-8") as fp:
            json.dump(self.meta, fp, ensure_ascii=False, indent=2, default=str)

    def close(self):
        try:
            self.disable_all()
        finally:
            self.ctl.running = False
            if self.ctl.is_alive():
                self.ctl.join(timeout=2.0)
            if self.ctl.rec:
                self.ctl.rec.close()
            self.save_meta()
        print(f"\n결과 폴더: {self.outdir}")


def load_glove_maps(ctl, map_dir):
    mod = sys.modules["tepi_ctrl"]
    for g in ctl.gloves:
        path = os.path.join(map_dir, f"{g.short}.json")
        if not os.path.isfile(path):
            raise FileNotFoundError(f"{path} 없음 — 먼저 캘리브레이션 sweep 으로 지도를 만드세요")
        with open(path, "r", encoding="utf-8") as fp:
            payload = json.load(fp)
        saved = payload.get("fingers", {})
        for fid, f in g.fingers.items():
            if fid not in saved:
                continue
            m = mod.TendonMap()
            if m.load_data(saved[fid], f.key):
                f.map = m
                f.map_aligned = False


def map_spans(ctl):
    out = {}
    for g in ctl.gloves:
        for fid, f in g.fingers.items():
            if f.map.ready:
                out[f.key] = {"sf_deg": round(f.map._sf, 2), "se_deg": round(f.map._se, 2),
                              "rom_mm": round(abs(f.map._sf) * sys.modules["tepi_ctrl"].DEG_TO_MM, 2)}
    return out


def k_per_curl(meta, gs, fid, kc):
    """결합 게인 kc [N/스풀deg] 를 그 장갑의 지도 기준 [N/curl] 로 환산 (flex, ext)."""
    s = meta["maps"][f"{gs}_{fid}"]
    return kc * abs(s["sf_deg"]), kc * abs(s["se_deg"])


# ═══════════════════════════════════════════════════════ 가이드 창 + 실행 틀

def run_protocol(session, protocol, gui=True):
    """protocol(session) 을 별도 스레드에서 돌리고, 메인 스레드는 가이드 창을 띄운다."""
    err = {}

    def worker():
        try:
            protocol(session)
        except KeyboardInterrupt:
            err["e"] = "사용자 중단"
        except Exception as e:      # noqa: BLE001
            import traceback
            traceback.print_exc()
            err["e"] = repr(e)
        finally:
            session.ctl.labels["done"] = True

    th = threading.Thread(target=worker, daemon=True)
    th.start()
    if gui:
        try:
            _guide_window(session)
        except Exception as e:      # noqa: BLE001
            print(f"(가이드 창 사용 불가: {e}) 콘솔로 진행합니다")
            gui = False
    if not gui:
        try:
            while th.is_alive():
                th.join(timeout=0.2)
        except KeyboardInterrupt:
            print("\n중단 — 모터 출력을 끕니다")
            session.ctl.trigger_estop("사용자 Ctrl+C")
    th.join(timeout=1.0)
    if err:
        print(f"실험이 중간에 멈췄습니다: {err['e']}")
    return not err


def _guide_window(session):
    import tkinter as tk
    ctl = session.ctl
    root = tk.Tk()
    root.title(f"{session.name} — 가이드 (Space/Esc = 비상정지)")
    W, H = 900, 300
    cv = tk.Canvas(root, width=W, height=H, background="white")
    cv.pack(fill="both", expand=True)
    root.bind("<space>", lambda e: ctl.trigger_estop("사용자"))
    root.bind("<Escape>", lambda e: ctl.trigger_estop("사용자"))
    L, R = 80, W - 40

    def x(c):
        c = max(-0.1, min(1.1, c))
        return L + (R - L) * c

    def draw():
        cv.delete("all")
        cv.create_text(W / 2, 22, text=ctl.labels.get("msg", ""), font=("맑은 고딕", 13, "bold"))
        cv.create_text(W / 2, 48, text=f"{ctl.labels.get('phase', '')}  {ctl.labels.get('trial', '')}",
                       fill="#555", font=("맑은 고딕", 10))
        rows = (("목표", ctl.ref.value, "#111"), ("치료사 G2", ctl.finger("G2").curl, "#1565c0"),
                ("환자 G1", ctl.finger("G1").curl, "#c62828"))
        for i, (name, val, col) in enumerate(rows):
            y = 100 + i * 60
            cv.create_text(40, y, text=name, font=("맑은 고딕", 10, "bold"))
            cv.create_rectangle(L, y - 12, R, y + 12, outline="#ccc", fill="#f3f5f7")
            if not math.isnan(val):
                if i == 0:
                    cv.create_line(x(val), y - 22, x(val), y + 22, width=5, fill=col)
                else:
                    cv.create_rectangle(L, y - 12, x(val), y + 12, outline="", fill=col)
                cv.create_text(R + 20, y, text=f"{val:.2f}", font=("Consolas", 9))
        for c in (0, 0.5, 1.0):
            cv.create_text(x(c), H - 20, text=f"{c:.1f}", fill="#777")
        if ctl.estop:
            cv.create_text(W / 2, H - 45, text=f"비상정지: {ctl.estop_msg}", fill="#c62828",
                           font=("맑은 고딕", 12, "bold"))
        if ctl.labels.get("done"):
            root.destroy()
            return
        root.after(40, draw)

    draw()
    root.mainloop()


def instability_monitor(thr=0.03, fc=3.0, hold=0.25):
    """결합오차(G1 curl − G2 curl)의 고주파 성분 RMS 가 thr 을 hold 초 넘으면 결합을 끈다."""
    st = {"e_prev": None, "y": 0.0, "ms": 0.0, "over": 0.0}

    def mon(ctl, dt):
        if not (ctl.finger("G1").coupling_enabled or ctl.finger("G2").coupling_enabled):
            st.update(e_prev=None, y=0.0, ms=0.0, over=0.0)
            return
        e = ctl.finger("G1").curl - ctl.finger("G2").curl
        if st["e_prev"] is None:
            st["e_prev"] = e
        a = 1.0 / (1.0 + 2 * math.pi * fc * dt)
        st["y"] = a * (st["y"] + e - st["e_prev"])
        st["e_prev"] = e
        st["ms"] += (st["y"] ** 2 - st["ms"]) * min(1.0, dt / 0.3)
        st["over"] = st["over"] + dt if math.sqrt(st["ms"]) > thr else 0.0
        if st["over"] >= hold:
            ctl.flag = 1
            ctl.labels["event"] = "UNSTABLE_STOP"
            ctl.stop_all()
    return mon


def parse_floats(s):
    return [float(v) for v in str(s).replace(",", " ").split()]
