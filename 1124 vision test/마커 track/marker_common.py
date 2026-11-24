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
import os
import threading
import time
from pathlib import Path

os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")
import cv2
import numpy as np
import pyrealsense2 as rs

HERE = Path(__file__).resolve().parent

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
class RSCamera:
    def __init__(self, serial, W=640, H=480, fps=30, processor=None):
        self.serial, self.W, self.H = serial, W, H
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


def blob_depth_mm(depth, labels, idx, scale):
    sel = (labels == idx).astype(np.uint8)
    inner = cv2.erode(sel, KERNEL)
    if inner.sum() < 4:
        inner = sel
    d = depth[inner > 0]
    d = d[d > 0]
    if d.size < 3:
        return None
    return float(np.median(d)) * scale * 1000.0


def detect(cam, item, rng, want, min_area=MIN_AREA):
    """한 색의 blob 을 큰 순서로 want 개까지. 각 blob: dict(u,v,area,xyz or None)."""
    m = make_mask(item["hsv"], rng)
    n, lab, stats, cent = cv2.connectedComponentsWithStats(m)
    cand = [(stats[i, cv2.CC_STAT_AREA], i) for i in range(1, n)
            if min_area <= stats[i, cv2.CC_STAT_AREA] <= MAX_AREA]
    cand.sort(key=lambda x: -x[0])
    out = []
    for area, i in cand[:want]:
        u, v = cent[i]
        z = blob_depth_mm(item["depth"], lab, i, cam.depth_scale)
        xyz = cam.deproject(u, v, z) if z is not None else None
        out.append(dict(u=float(u), v=float(v), area=int(area), xyz=xyz))
    return out, m


S_FLOOR, V_FLOOR = 30, 40   # 이보다 어둡거나 탁한 픽셀은 hue 가 노이즈라서 절대 포함하지 않음


def ranges_from(samples, hue_margin=3, sat_margin=35, val_margin=35, pct=(3, 97)):
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
    return dict(h=[round(float(center + lo[0] - hue_margin), 1), round(float(center + hi[0] + hue_margin), 1)],
                s=[int(max(S_FLOOR, lo[1] - sat_margin)), int(min(255, hi[1] + sat_margin))],
                v=[int(max(V_FLOOR, lo[2] - val_margin)), int(min(255, hi[2] + val_margin))],
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
