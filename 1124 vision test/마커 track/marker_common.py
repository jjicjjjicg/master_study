"""
marker_common.py — 마커 HSV 튜닝 / 3D 트래킹 공통 코드.

- RSCamera   : RealSense 한 대. 백그라운드 스레드에서 color + (color 에 정렬된) depth 수신,
               필요하면 같은 스레드에서 검출(processor)까지 수행.
- HSV json   : 카메라마다 hsv_<serial>.json
               { "<마커이름>": {"h":[lo,hi], "s":[lo,hi], "v":[lo,hi], "n_samples":N, "count":개수}, ... }
               (1123_비전통합 marker_colors.json 과 같은 형식 + count)
- detect     : HSV 마스크 → blob → blob 내부 depth median → 카메라 좌표계 3D (mm)
               카메라 좌표계: X 오른쪽, Y 아래, Z 카메라 앞쪽 (RealSense 기본)
"""

import json
import sys
import os
import threading
import time
from pathlib import Path

os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")
import cv2
import numpy as np
import pyrealsense2 as rs

HERE = Path(__file__).resolve().parent

# 카메라 스레드 3개 + 화면 스레드가 GIL 을 나눠 쓰는데, 기본 전환 간격(5ms)이면 카메라가 GIL 을 기다리느라
# 30 → 15fps 로 떨어짐 (측정). 0.5ms 로 줄이면 3대 모두 30fps 유지.
sys.setswitchinterval(0.0005)

# 마커 기본 구성 (이름, 개수). --markers 로 바꿀 수 있고, 튜너에서 [ ] 키로 개수 조절 가능.
DEFAULT_MARKERS = [("wrist", 1), ("thumb", 3), ("index", 4), ("middle", 4)]

MIN_AREA = 6          # 10mm 마커가 멀리 있으면 몇 십 픽셀밖에 안 됨
MAX_AREA = 4000
KERNEL = np.ones((3, 3), np.uint8)


# --------------------------------------------------------------------------- 장치
def list_devices():
    out = []
    for d in rs.context().query_devices():
        name = d.get_info(rs.camera_info.name)
        if "Platform Camera" in name:
            continue
        out.append((name.replace("Intel RealSense ", ""), d.get_info(rs.camera_info.serial_number)))
    return out


def hsv_path(serial, folder=HERE):
    return Path(folder) / f"hsv_{serial}.json"


def load_hsv(serial, folder=HERE):
    p = hsv_path(serial, folder)
    if not p.is_file():
        return {}
    with open(p, encoding="utf-8") as f:
        return json.load(f)


def save_hsv(serial, data, folder=HERE):
    p = hsv_path(serial, folder)
    tmp = p.with_suffix(".tmp")
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    tmp.replace(p)
    return p


def parse_markers(items):
    """['wrist:1', 'thumb:3'] -> [('wrist',1), ('thumb',3)]"""
    out = []
    for it in items:
        name, _, n = it.partition(":")
        out.append((name, int(n) if n else 1))
    return out


# --------------------------------------------------------------------------- 카메라
def cam_path(serial, folder=HERE):
    return Path(folder) / f"cam_{serial}.json"


class RSCamera:
    """settings_dir 에 cam_<serial>.json 이 있고 auto=false 면 그 노출/게인/화이트밸런스로 고정해서 연다.
    (HSV 튜닝 때와 같은 색으로 보여야 하므로 트래킹 코드도 같은 파일을 자동으로 읽는다)"""

    def __init__(self, serial, W=640, H=480, fps=30, processor=None, settings_dir=HERE):
        self.serial, self.W, self.H, self.fps = serial, W, H, fps
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, fps)
        cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, fps)
        prof = self.pipe.start(cfg)
        dev = prof.get_device()
        self.model = dev.get_info(rs.camera_info.name).replace("Intel RealSense ", "")
        self.depth_scale = dev.first_depth_sensor().get_depth_scale()
        # depth 를 color 에 정렬하므로 3D 복원은 color intrinsics 로 한다
        self.intr = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.align = rs.align(rs.stream.color)
        self.processor = processor
        self.color_sensor = dev.first_color_sensor()
        self.settings_file = cam_path(serial, settings_dir) if settings_dir else None
        self.locking = False           # lock_color() 진행 중
        # 카메라는 프로그램이 끝나도 마지막 수동 노출을 기억함 → 저장된 LOCK 이 없으면 항상 자동으로 시작
        st = {}
        if self.settings_file and self.settings_file.is_file():
            try:
                with open(self.settings_file, encoding="utf-8") as f:
                    st = json.load(f)
            except Exception as e:
                print(f"  {self.serial}: {self.settings_file.name} 읽기 실패: {e}")
        try:
            if st and not st.get("auto", True):
                self.set_manual(st["exposure"], st["gain"], st["white_balance"])
            else:
                self.set_auto()
        except Exception as e:
            print(f"  {self.serial}: 노출 설정 실패: {e}")

        self.lock = threading.Lock()
        self.latest = None     # dict(color, depth, hsv, result, ts_ms, t, seq)
        self.seq = 0
        self.fps_val = 0.0
        self.error = None
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    @property
    def label(self):
        return f"{self.model} [{self.serial}]"

    def _loop(self):
        t0, n = time.time(), 0
        while self.running:
            try:
                fs = self.align.process(self.pipe.wait_for_frames(3000))
            except Exception as e:
                self.error = str(e)
                continue
            cf, df = fs.get_color_frame(), fs.get_depth_frame()
            if not cf or not df:
                continue
            color = np.asanyarray(cf.get_data()).copy()
            depth = np.asanyarray(df.get_data()).copy()          # uint16 raw
            hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
            item = dict(color=color, depth=depth, hsv=hsv, ts_ms=cf.get_timestamp(),
                        frame_no=cf.get_frame_number(), t=time.time(), result=None)
            if self.processor is not None:
                try:
                    item["result"] = self.processor(self, item)
                except Exception as e:
                    self.error = f"processor: {e}"
            with self.lock:
                self.seq += 1
                item["seq"] = self.seq
                self.latest = item
            self.error = None
            n += 1
            if time.time() - t0 >= 1.0:
                self.fps_val, t0, n = n / (time.time() - t0), time.time(), 0

    def get(self):
        with self.lock:
            return self.latest

    def stop(self):
        self.running = False
        self.thread.join(timeout=3)
        try:
            self.pipe.stop()
        except Exception:
            pass

    # ---- 컬러 노출 / 게인 / 화이트밸런스 ----
    def _opt(self, name):
        return float(self.color_sensor.get_option(getattr(rs.option, name)))

    def _set(self, name, val):
        r = self.color_sensor.get_option_range(getattr(rs.option, name))
        step = r.step if r.step > 0 else 1.0                  # white_balance 는 10 단위만 받음
        val = float(np.clip(r.min + round((val - r.min) / step) * step, r.min, r.max))
        self.color_sensor.set_option(getattr(rs.option, name), val)
        return val

    @property
    def is_auto(self):
        return bool(self._opt("enable_auto_exposure")) or bool(self._opt("enable_auto_white_balance"))

    def color_settings(self):
        return dict(auto=self.is_auto, exposure=self._opt("exposure"), gain=self._opt("gain"),
                    white_balance=self._opt("white_balance"))

    def set_manual(self, exposure=None, gain=None, white_balance=None):
        self._set("enable_auto_exposure", 0)
        self._set("enable_auto_white_balance", 0)
        if exposure is not None:
            self._set("exposure", exposure)
        if gain is not None:
            self._set("gain", gain)
        if white_balance is not None:
            self._set("white_balance", white_balance)

    def set_auto(self):
        self._set("enable_auto_exposure", 1)
        self._set("enable_auto_white_balance", 1)

    def save_settings(self):
        if not self.settings_file:
            return None
        with open(self.settings_file, "w", encoding="utf-8") as f:
            json.dump(self.color_settings(), f, ensure_ascii=False, indent=2)
        return self.settings_file

    def _measure(self, settle=6, avg=3, timeout=5.0):
        """설정 바꾼 '뒤에 찍힌' 프레임만 avg 개 평균: (평균 밝기 V, log(R/B))
        변경이 화면에 반영되기까지 4프레임 걸림(측정). 프레임 '개수'로 세면 화면/다른 카메라가 바빠서
        처리가 밀릴 때 옛 프레임을 재게 되므로, 카메라 타임스탬프(system_time, ms)로 settle 프레임 이후만 쓴다."""
        t_ok = time.time() * 1000.0 + settle * 1000.0 / self.fps
        t_end = time.time() + timeout
        vs, rbs, last = [], [], None
        while len(vs) < avg and time.time() < t_end:
            it = self.get()
            if it is None or it["ts_ms"] < t_ok or it["seq"] == last:
                time.sleep(0.005)
                continue
            last = it["seq"]
            c = it["color"][::4, ::4].astype(np.float32)
            vs.append(float(it["hsv"][::4, ::4, 2].mean()))
            rbs.append(float(np.log((c[..., 2].mean() + 1) / (c[..., 0].mean() + 1))))
        if not vs:
            raise RuntimeError("측정할 새 프레임이 안 옴")
        return np.mean(vs), np.mean(rbs)

    def lock_color(self, done=None):
        """지금 자동 노출/화이트밸런스가 만든 화면(밝기, 색감)과 같아지는 수동 값을 찾아 고정.
        (이 PC 에서는 프레임 메타데이터가 안 나와서 자동 값을 직접 읽을 수 없음 → 측정해서 맞춤)
        별도 스레드에서 몇 초 걸린다. done(cam, ok, msg) 콜백."""
        def run():
            self.locking = True
            try:
                self.set_auto()
                time.sleep(1.0)                                   # 자동 값이 자리 잡을 시간
                tv, trb = self._measure(settle=1, avg=8)
                if tv is None:
                    raise RuntimeError("프레임 없음")
                # 1) 화이트밸런스: 자동일 때와 R/B 비율이 같아지는 값 (단조 → 이분 탐색)
                self._set("enable_auto_white_balance", 0)
                lo, hi = 2800.0, 6500.0
                self._set("white_balance", lo)
                f_lo = self._measure()[1] - trb
                self._set("white_balance", hi)
                f_hi = self._measure()[1] - trb
                if f_lo * f_hi > 0:                              # 범위 밖 → 가까운 끝
                    wb = lo if abs(f_lo) < abs(f_hi) else hi
                else:
                    for _ in range(7):
                        mid = (lo + hi) / 2
                        self._set("white_balance", mid)
                        f_mid = self._measure()[1] - trb
                        if f_mid * f_lo > 0:
                            lo, f_lo = mid, f_mid
                        else:
                            hi = mid
                    wb = (lo + hi) / 2
                self._set("white_balance", wb)
                # 2) 노출: 자동일 때와 평균 밝기가 같아지게. 프레임 주기보다 길면 fps 가 떨어지므로 상한.
                #    이 카메라들은 노출이 계단식(19, 39, 78, 156, 312 ...)으로만 바뀜 → 목표보다 살짝 어두운 계단을
                #    고르고 (노출이 짧을수록 손 움직임 번짐↓) 나머지는 게인으로 미세 조정.
                self._set("enable_auto_exposure", 0)
                e_max = 10000.0 / self.fps * 0.95                  # 단위 100us. 이보다 길면 fps 떨어짐
                levels = set()                                    # 실제로 적용되는 노출 계단 (카메라마다 다름)
                for req in np.geomspace(1, e_max, 40):
                    self._set("exposure", req)
                    levels.add(self._opt("exposure"))
                levels = sorted(x for x in levels if x <= e_max + 1)
                # 게인 80 으로 목표 밝기에 닿는 가장 짧은 노출 (짧을수록 번짐↓, 게인 낮을수록 노이즈↓)
                exp = levels[-1]
                self._set("gain", 80)
                for lv in levels:
                    self._set("exposure", lv)
                    if self._measure()[0] >= tv:
                        exp = lv
                        break
                self._set("exposure", exp)
                glo, ghi = 0.0, 128.0                             # 나머지는 게인으로 미세 조정
                for _ in range(7):
                    mid = (glo + ghi) / 2
                    self._set("gain", mid)
                    if self._measure()[0] < tv:
                        glo = mid
                    else:
                        ghi = mid
                self._set("gain", (glo + ghi) / 2)
                v, _ = self._measure()
                if abs(v - tv) > max(12.0, 0.15 * tv):           # 손이 움직였거나 측정이 꼬임 → 자동으로 되돌림
                    raise RuntimeError(f"밝기가 안 맞음 (V {v:.0f} / auto {tv:.0f}) - 손을 가만히 두고 다시 L")
                st = self.color_settings()
                msg = (f"{self.model}: locked exp {st['exposure']:.0f} gain {st['gain']:.0f} "
                       f"wb {st['white_balance']:.0f}  (V {v:.0f} / auto {tv:.0f})")
                ok = True
            except Exception as e:
                ok, msg = False, f"{self.model}: lock failed -> AUTO ({e})"
                try:
                    self.set_auto()                               # 어중간한 수동 값(어두운 화면)으로 남지 않게
                except Exception:
                    pass
            self.locking = False
            if done:
                done(self, ok, msg)

        threading.Thread(target=run, daemon=True).start()

    def deproject(self, u, v, z_mm):
        p = rs.rs2_deproject_pixel_to_point(self.intr, [float(u), float(v)], z_mm / 1000.0)
        return np.array(p) * 1000.0


# --------------------------------------------------------------------------- 색 / 검출
def make_mask(hsv, rng, clean=True):
    hlo, hhi = rng["h"]
    s, v = rng["s"], rng["v"]
    if hlo < 0 or hhi > 179:            # 빨강처럼 0/180 경계를 넘는 색
        a = cv2.inRange(hsv, (int(hlo % 180), s[0], v[0]), (179, s[1], v[1]))
        b = cv2.inRange(hsv, (0, s[0], v[0]), (int(hhi % 180), s[1], v[1]))
        m = cv2.bitwise_or(a, b)
    else:
        m = cv2.inRange(hsv, (int(hlo), s[0], v[0]), (int(hhi), s[1], v[1]))
    if clean:
        # 마커가 작아서 OPEN 은 쓰지 않음(작은 마커가 사라짐). CLOSE 로 구멍만 메움.
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, KERNEL)
    return m


def blob_depth_mm(depth, labels, idx, scale, stats=None):
    """blob 내부 depth median (mm). 작은 마커는 depth 구멍이 잘 나므로 없으면 주변 2px 까지 넓혀 본다."""
    if stats is not None:                                    # bbox 만 잘라서 계산 (빠름)
        x, y, w, h = stats[idx, :4]
        P = 3
        x0, y0 = max(0, x - P), max(0, y - P)
        x1, y1 = min(labels.shape[1], x + w + P), min(labels.shape[0], y + h + P)
        labels, depth = labels[y0:y1, x0:x1], depth[y0:y1, x0:x1]
    sel = (labels == idx).astype(np.uint8)
    inner = cv2.erode(sel, KERNEL)
    if inner.sum() < 4:
        inner = sel
    for region in (inner, cv2.dilate(sel, KERNEL, iterations=2)):
        d = depth[region > 0]
        d = d[d > 0]
        if d.size >= 3:
            return float(np.median(d)) * scale * 1000.0
    return None


def detect_all(cam, item, rng, mask=None, max_depth=None, min_area=MIN_AREA, refine=None, limit=None):
    """한 색의 blob 전부 (큰 순서). max_depth(mm) 가 있으면 그보다 멀거나 depth 를 못 읽은 blob 은 버린다.
    mask 를 주면 그걸 쓴다 (make_masks 로 여러 색을 같이 정리한 마스크).
    refine: 앞에서부터 이 개수만 정밀 depth(median) + 3D 좌표 계산. 나머지는 평균 depth 로 거르기만 하고 xyz=None.
            (노이즈 blob 이 수천 개일 때 전부 median 을 구하면 느림) None = 전부 정밀.
    limit: 이 개수를 찾으면 멈춤 (전체 개수가 필요 없을 때).
    반환: (blobs, mask, labels)   blob = dict(u, v, area, z, xyz or None, lab)"""
    m = mask if mask is not None else make_mask(item["hsv"], rng)
    n, lab, stats, cent = cv2.connectedComponentsWithStats(m)
    areas = stats[:, cv2.CC_STAT_AREA]
    idx = np.nonzero((areas >= min_area) & (areas <= MAX_AREA))[0]
    idx = idx[idx > 0]
    idx = idx[np.argsort(-areas[idx], kind="stable")]
    if idx.size == 0:
        return [], m, lab
    # blob 마다 평균 depth (한 번에): 배경 같은 먼 것은 median 계산 없이 바로 버림
    on = m.ravel() > 0                                 # 마스크 픽셀만 (보통 화면의 일부)
    flat = lab.ravel()[on]
    dep = item["depth"].ravel()[on]
    dsum = np.bincount(flat, weights=dep, minlength=n)
    dcnt = np.bincount(flat, weights=(dep > 0), minlength=n)
    mean_mm = dsum / np.maximum(dcnt, 1) * cam.depth_scale * 1000.0
    has = dcnt >= 3
    if max_depth:
        idx = idx[~has[idx] | (mean_mm[idx] <= max_depth * 1.1)]
    out = []
    for i in idx:
        if limit is not None and len(out) >= limit:
            break
        precise = refine is None or len(out) < refine
        if precise or not has[i]:
            if not precise and max_depth:          # 정밀 계산 대상이 아니고 depth 도 없음 → 버림
                continue
            z = blob_depth_mm(item["depth"], lab, i, cam.depth_scale, stats)
        else:
            z = float(mean_mm[i])
        if max_depth and (z is None or z > max_depth):
            continue
        u, v = cent[i]
        xyz = cam.deproject(u, v, z) if (z is not None and precise) else None
        out.append(dict(u=float(u), v=float(v), area=int(areas[i]), z=z, xyz=xyz, lab=int(i)))
    return out, m, lab


def detect(cam, item, rng, want, min_area=MIN_AREA, mask=None, max_depth=None):
    """한 색의 blob 을 큰 순서로 want 개까지. 각 blob: dict(u,v,area,xyz or None)."""
    out, m, _ = detect_all(cam, item, rng, mask, max_depth, min_area, refine=want, limit=want)
    return out[:want], m


# ---- 여러 색이 HSV 범위에서 겹칠 때: 픽셀마다 '가장 가까운 색' 하나에만 배정 ----
MAHA_GATE = 16.0   # 마할라노비스 거리² 상한 (3차원, 16 ≈ 99.9%). 범위 안이라도 이보다 멀면 버림(배경 제거)


def _color_feat(h, s, v):
    """HSV → (S·cos H, S·sin H, V). hue 가 원형이라 0/180 경계와 채도 낮은 색이 자연스럽게 처리됨."""
    a = np.radians(np.asarray(h, np.float32) * 2)
    s = np.asarray(s, np.float32)
    return np.stack([s * np.cos(a), s * np.sin(a), np.asarray(v, np.float32)], -1)


# 주의: 아래 두 함수는 일부러 np.linalg.inv / @ (BLAS·LAPACK) 를 안 쓴다.
# 카메라 3대를 스레드로 동시에 처리할 때 이 PC 의 numpy BLAS 가 동시 호출에서 segfault 남 (재현됨).
def core_pixels(px):
    """클릭 한 번으로 잡힌 영역에서 '마커 몸통' 픽셀만 남긴다.
    영역 가장자리의 어두운 테두리/그림자/주변 검은색이 섞이면 범위가 넓어져서 검은색·회색까지 잡히므로,
    채도·밝기가 영역 중앙값보다 확실히 낮은 픽셀과 색상(hue)이 많이 벗어난 픽셀을 버린다."""
    px = np.asarray(px)
    if len(px) < 8:
        return px
    h, s, v = px[:, 0].astype(float), px[:, 1].astype(float), px[:, 2].astype(float)
    keep = (s >= 0.75 * np.median(s)) & (v >= 0.65 * np.median(v)) & (s >= S_FLOOR) & (v >= V_FLOOR)
    if keep.sum() < 5:
        return px
    ang = np.radians(h[keep] * 2)
    center = (np.degrees(np.arctan2(np.sin(ang).mean(), np.cos(ang).mean())) % 360) / 2
    dev = np.abs(((h - center + 90) % 180) - 90)
    keep &= dev <= max(3.0, 2.5 * np.median(dev[keep]))
    return px[keep] if keep.sum() >= 5 else px


def _inv3(c):
    """3x3 역행렬 (여인수 공식, 순수 파이썬)"""
    (a, b, c_), (d, e, f), (g, h, i) = [[float(x) for x in row] for row in c]
    A, B, C = e * i - f * h, -(d * i - f * g), d * h - e * g
    det = a * A + b * B + c_ * C
    return [[A / det, -(b * i - c_ * h) / det, (b * f - c_ * e) / det],
            [B / det, (a * i - c_ * g) / det, -(a * f - c_ * d) / det],
            [C / det, -(a * h - b * g) / det, (a * e - b * d) / det]]


def _maha2(d, ic):
    """행마다 d^T · ic · d (원소별 연산만)"""
    d0, d1, d2 = d[:, 0], d[:, 1], d[:, 2]
    return (ic[0][0] * d0 * d0 + ic[1][1] * d1 * d1 + ic[2][2] * d2 * d2
            + (ic[0][1] + ic[1][0]) * d0 * d1 + (ic[0][2] + ic[2][0]) * d0 * d2
            + (ic[1][2] + ic[2][1]) * d1 * d2)


def make_masks(hsv, rngs, clean=True):
    """{이름: 범위} → {이름: 마스크}.
    범위에 mean/cov(샘플 색 분포)가 있으면, 여러 색 범위에 동시에 들어가는 픽셀은 분포가 가장 가까운 색에만 남기고,
    어느 분포와도 gate 이상 멀면 (범위 안이라도) 버린다. mean/cov 가 없는 옛 json 은 범위만 쓴다."""
    raw = {n: make_mask(hsv, r, clean=False) for n, r in rngs.items() if r}
    stat = [n for n in raw if "mean" in rngs[n] and "cov" in rngs[n]]
    if stat:
        union = np.zeros(hsv.shape[:2], np.uint8)
        for n in stat:
            union |= raw[n]
        ys, xs = np.nonzero(union)
        if ys.size:
            px = hsv[ys, xs]
            f = _color_feat(px[:, 0], px[:, 1], px[:, 2])
            D = np.full((len(stat), ys.size), np.inf, np.float32)
            for i, n in enumerate(stat):
                r = rngs[n]
                d = f - np.asarray(r["mean"], np.float32)
                m2 = _maha2(d, _inv3(r["cov"]))
                m2[m2 > float(r.get("gate", MAHA_GATE))] = np.inf
                D[i] = np.where(raw[n][ys, xs] > 0, m2, np.inf)
            best, ok = D.argmin(0), np.isfinite(D.min(0))
            for i, n in enumerate(stat):
                k = ok & (best == i)
                mm = np.zeros_like(raw[n])
                mm[ys[k], xs[k]] = 255
                raw[n] = mm
    if clean:
        raw = {n: cv2.morphologyEx(m, cv2.MORPH_CLOSE, KERNEL) for n, m in raw.items()}
    return raw


S_FLOOR, V_FLOOR = 45, 45   # 이보다 어둡거나 탁한 픽셀은 hue 가 노이즈라서 절대 포함하지 않음 (검은색/회색 차단)


def ranges_from(samples, hue_margin=2, sat_margin=20, val_margin=30, pct=(5, 95)):
    """HSV 샘플 픽셀 -> inRange 범위. hue 는 원형 평균으로 0/180 경계 처리.
    S/V 하한은 S_FLOOR/V_FLOOR 아래로 내려가지 않게 막는다 (어두운 배경 노이즈가 전부 잡히는 것 방지)."""
    if len(samples) < 5:
        return None
    s = np.asarray(samples, float)
    h = s[:, 0]
    ang = np.radians(h * 2)
    center = (np.degrees(np.arctan2(np.sin(ang).mean(), np.cos(ang).mean())) % 360) / 2
    dev = ((h - center + 90) % 180) - 90
    lo, hi = np.percentile(np.column_stack((dev, s[:, 1:])), pct, axis=0)
    # 색 분포 (make_masks 에서 겹치는 색 구분용). 바깥 3% 는 빼고, 너무 납작하지 않게 대각선을 조금 더함
    f = _color_feat(s[:, 0], s[:, 1], s[:, 2]).astype(np.float64)
    keep = np.all((np.column_stack((dev, s[:, 1:])) >= lo) & (np.column_stack((dev, s[:, 1:])) <= hi), 1)
    if keep.sum() >= 5:
        f = f[keep]
    cov = np.cov(f, rowvar=False) + np.eye(3) * 9.0
    return dict(h=[round(float(center + lo[0] - hue_margin), 1), round(float(center + hi[0] + hue_margin), 1)],
                s=[int(max(S_FLOOR, lo[1] - sat_margin)), int(min(255, hi[1] + sat_margin))],
                v=[int(max(V_FLOOR, lo[2] - val_margin)), int(min(255, hi[2] + val_margin))],
                mean=[round(float(x), 2) for x in f.mean(0)],
                cov=[[round(float(x), 2) for x in row] for row in cov],
                n_samples=int(len(samples)))


def range_bgr(rng):
    """표시용 색: HSV 범위 중심색(밝게)."""
    if not rng:
        return (200, 200, 200)
    h = ((rng["h"][0] + rng["h"][1]) / 2) % 180
    px = np.uint8([[[h, 220, 255]]])
    b, g, r = cv2.cvtColor(px, cv2.COLOR_HSV2BGR)[0, 0]
    return int(b), int(g), int(r)


# --------------------------------------------------------------------------- 공통 그리기
FONT = cv2.FONT_HERSHEY_SIMPLEX


def put(img, text, org, scale=0.5, color=(255, 255, 255), thick=1):
    cv2.putText(img, text, org, FONT, scale, (0, 0, 0), thick + 3, cv2.LINE_AA)
    cv2.putText(img, text, org, FONT, scale, color, thick, cv2.LINE_AA)
