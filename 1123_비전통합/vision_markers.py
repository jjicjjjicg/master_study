"""
vision_markers.py — 색 마커 기반 손 관절각 (RealSense top 1대 + 사이드 2대)

marker_3d.py 의 검출 / depth 3D / 사슬 정렬 / 관절각 로직을 그대로 가져와
'장갑 구동 코드와 별도 프로세스'로 돌리기 위한 모듈.

카메라 구성 (vision_config.json)
  top      : 위에서 양손(G1 환자, G2 치료사)을 같이 봄 → ROI 로 손을 나눔
  side_G1  : G1 손 오른쪽 사이드 뷰
  side_G2  : G2 손 오른쪽 사이드 뷰

3D 경로
  각 뷰(카메라+ROI)마다 그 카메라의 aligned depth 로 마커 3D 를 만든다(카메라 좌표계).
  관절각은 좌표계와 무관하므로, 카메라 간 외부 캘리브레이션 없이
  '관절각 단계'에서 같은 손의 top / side 결과를 합친다(fuse_angles).

단독 실행 (모터 없이 카메라·ROI·색 확인용)
  python vision_markers.py --list          연결된 RealSense 시리얼 출력
  python vision_markers.py                 미리보기 + 1초마다 관절각 출력
  미리보기 창: M 마스크 보기 | Q 종료(단독 실행일 때만)

장갑 코드에서는 VisionLink 를 쓴다:
  link = VisionLink(); link.start()
  curl, age_s = link.get_curl("G2", "I")
"""
import argparse
import itertools
import json
import math
import queue
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
CONFIG_PATH = HERE / "vision_config.json"
ROM_PATH = HERE / "vision_rom.json"

HANDS = ("G1", "G2")
FINGERS = {
    "thumb":  ["MCP", "IP", "tip"],
    "index":  ["MCP", "PIP", "DIP", "tip"],
    "middle": ["MCP", "PIP", "DIP", "tip"],
}
PART_TO_FID = {"thumb": "T", "index": "I", "middle": "M"}
FID_TO_PART = {v: k for k, v in PART_TO_FID.items()}
DEFAULT_BONES = {"thumb": [34.0, 25.0],
                 "index": [45.0, 28.0, 22.0],
                 "middle": [48.0, 30.0, 23.0]}
ANGLE_NAMES = ["thumb_MCP", "thumb_IP", "index_MCP", "index_PIP", "index_DIP",
               "middle_MCP", "middle_PIP", "middle_DIP"]
FINGER_JOINTS = {f: [f"{f}_{n}" for n in names[:-1]] for f, names in FINGERS.items()}
JOINT_KEYS = ["wrist"] + [f"{f}_{n}" for f, ns in FINGERS.items() for n in ns]
CAMERA_ROLES = ("top", "side_G1", "side_G2")

DEFAULT_CONFIG = {
    # 빈 문자열이면 연결된 카메라를 시리얼 순서대로 top, side_G1, side_G2 에 배정
    "cameras": {"top": "", "side_G1": "", "side_G2": ""},
    # roi = [x0, y0, x1, y1] (픽셀) / null 이면 화면 전체
    "views": [
        {"name": "top_G1", "camera": "top", "hand": "G1", "roi": [0, 0, 320, 480]},
        {"name": "top_G2", "camera": "top", "hand": "G2", "roi": [320, 0, 640, 480]},
        {"name": "side_G1", "camera": "side_G1", "hand": "G1", "roi": None},
        {"name": "side_G2", "camera": "side_G2", "hand": "G2", "roi": None},
    ],
    "colors": {"G1": "marker_colors.json", "G2": "marker_colors.json"},
    "bones": {"G1": "marker_bones.json", "G2": "marker_bones.json"},
    "width": 640, "height": 480, "fps": 30,
    "max_jump_mm": 60.0,        # 프레임 간 이동 상한 (튐 제거)
    "bone_tol": 0.25,           # 뼈 길이 허용 비율
    "smooth": 3,                # 시간 중앙값 창
    "min_blob_area": 12,
    "fuse_max_diff_deg": 25.0,  # 두 뷰 관절각 차이가 이보다 크면 관절 더 많이 잡힌 뷰를 택함
    "reject_reset_frames": 15,  # 연속으로 이만큼 버려지면 그 관절의 직전값을 리셋
    "preview": True,
    "preview_scale": 0.6,
}


# ═══════════════════════════════════════════════════════ 설정 / ROM

def _resolve(path):
    p = Path(path)
    return p if p.is_absolute() else HERE / p


def load_config(path=CONFIG_PATH):
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))
    path = Path(path)
    if path.is_file():
        with open(path, encoding="utf-8") as fp:
            user = json.load(fp)
        for k, v in user.items():
            if isinstance(v, dict) and isinstance(cfg.get(k), dict):
                cfg[k].update(v)
            else:
                cfg[k] = v
    return cfg


def load_colors(cfg):
    out = {}
    for hand in HANDS:
        p = _resolve(cfg["colors"][hand])
        if not p.is_file():
            raise FileNotFoundError(f"색 파일 없음 ({hand}): {p} — marker_tune.py 먼저")
        with open(p, encoding="utf-8") as fp:
            colors = json.load(fp)
        miss = [k for k in ["wrist"] + list(FINGERS) if k not in colors]
        if miss:
            raise ValueError(f"{p.name}: 색 미등록 {miss}")
        out[hand] = colors
    return out


def load_bones(cfg):
    out = {}
    for hand in HANDS:
        p = _resolve(cfg["bones"].get(hand) or "")
        prior = dict(DEFAULT_BONES)
        if p.is_file():
            with open(p, encoding="utf-8") as fp:
                prior.update(json.load(fp))
        out[hand] = prior
    return out


def load_rom(path=ROM_PATH):
    if Path(path).is_file():
        with open(path, encoding="utf-8") as fp:
            return json.load(fp)
    return {}


def save_rom(rom, path=ROM_PATH):
    tmp = str(path) + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fp:
        json.dump(rom, fp, ensure_ascii=False, indent=2)
    Path(tmp).replace(path)


def finger_curl(angles, hand_rom, part):
    """관절각(180=편 상태) → 0(편 손)~1(주먹) curl.
    관절마다 (flex - open) / (fist - open) 를 ROM 폭으로 가중 평균.
    = 보이는 관절만으로 '굽힘 합' 을 정규화한 값. 안 보이는 관절은 빠진다."""
    if not hand_rom:
        return None
    o_all, f_all = hand_rom.get("open", {}), hand_rom.get("fist", {})
    num = den = 0.0
    for j in FINGER_JOINTS[part]:
        a = angles.get(j)
        o, c = o_all.get(j), f_all.get(j)
        if a is None or o is None or c is None or not math.isfinite(a):
            continue
        rng = c - o
        if rng < 10.0:            # ROM 이 너무 작으면 그 관절은 쓰지 않음
            continue
        num += (180.0 - a) - o
        den += rng
    if den <= 0.0:
        return None
    return num / den


# ═══════════════════════════════════════════════════════ 검출 / 3D (marker_3d.py 기반)

def make_mask(cv2, hsv, rng, kernel, roi=None):
    hlo, hhi = rng["h"]
    s, v = rng["s"], rng["v"]
    if hlo < 0 or hhi > 179:
        a = cv2.inRange(hsv, (int(hlo % 180), s[0], v[0]), (179, s[1], v[1]))
        b = cv2.inRange(hsv, (0, s[0], v[0]), (int(hhi % 180), s[1], v[1]))
        m = cv2.bitwise_or(a, b)
    else:
        m = cv2.inRange(hsv, (int(hlo), s[0], v[0]), (int(hhi), s[1], v[1]))
    if roi is not None:
        x0, y0, x1, y1 = roi
        keep = np.zeros_like(m)
        keep[y0:y1, x0:x1] = 255
        m = cv2.bitwise_and(m, keep)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel)


def find_blobs(cv2, hsv, rng, kernel, want, roi, min_area):
    m = make_mask(cv2, hsv, rng, kernel, roi)
    n, lab, stats, cent = cv2.connectedComponentsWithStats(m)
    cand = [(stats[i, cv2.CC_STAT_AREA], i) for i in range(1, n)
            if stats[i, cv2.CC_STAT_AREA] >= min_area]
    cand.sort(key=lambda x: -x[0])
    cand = cand[:want]
    pts = np.array([cent[i] for _, i in cand], dtype=float).reshape(-1, 2)
    ids = [i for _, i in cand]
    return pts, lab, ids, m


def blob_depth(cv2, depth, lab, blob_id, scale, kernel):
    sel = (lab == blob_id).astype(np.uint8)
    eroded = cv2.erode(sel, kernel)
    if eroded.sum() < 4:
        eroded = sel
    ys, xs = np.nonzero(eroded)
    d = depth[ys, xs].astype(float)
    d = d[d > 0]
    if len(d) < 4:
        return None
    return float(np.median(d)) * scale * 1000.0


def backproject(cv2, K, D, px, Z):
    u = cv2.undistortPoints(np.array(px, float).reshape(1, 1, 2), K, D).reshape(2)
    return np.array([u[0] * Z, u[1] * Z, Z])


def order_chain(pts, wrist, prior):
    n = len(pts)
    if n != len(prior) + 1:
        return None
    best, bc = None, np.inf
    for perm in itertools.permutations(range(n)):
        P = pts[list(perm)]
        segs = np.linalg.norm(np.diff(P, axis=0), axis=1)
        c = float(np.sum((segs - np.array(prior)) ** 2))
        if wrist is not None:
            c += 0.3 * float(np.linalg.norm(P[0] - wrist) ** 2)
        if c < bc:
            best, bc = perm, c
    return list(best)


def compute_angles(J):
    def ang3(a, b, c):
        v1, v2 = a - b, c - b
        n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
        if n1 < 1e-9 or n2 < 1e-9:
            return np.nan
        return float(np.degrees(np.arccos(np.clip(np.dot(v1, v2) / (n1 * n2), -1, 1))))

    out = {k: np.nan for k in ANGLE_NAMES}
    g = J.get
    for f, names in FINGERS.items():
        P = [g(f"{f}_{n}") for n in names]
        for i in range(1, len(P) - 1):
            if P[i - 1] is not None and P[i] is not None and P[i + 1] is not None:
                out[f"{f}_{names[i]}"] = ang3(P[i - 1], P[i], P[i + 1])

    w, I, M = g("wrist"), g("index_MCP"), g("middle_MCP")
    if w is None or I is None or M is None:
        return out
    across = M - I
    na = np.linalg.norm(across)
    if na < 1e-9:
        return out
    across = across / na
    along = 0.5 * (I + M) - w
    along = along - np.dot(along, across) * across
    nl = np.linalg.norm(along)
    if nl < 1e-9:
        return out
    along = along / nl
    for f, names in FINGERS.items():
        a, b = g(f"{f}_{names[0]}"), g(f"{f}_{names[1]}")
        if a is None or b is None:
            continue
        seg = b - a
        ns = np.linalg.norm(seg)
        if ns < 1e-9:
            continue
        out[f"{f}_MCP"] = float(180.0 - np.degrees(
            np.arccos(np.clip(np.dot(seg / ns, along), -1, 1))))
    return out


class ViewTracker:
    """뷰(카메라 좌표계) 하나의 튐 억제: 속도 게이트 + 뼈 길이 게이트 + 시간 중앙값."""

    def __init__(self, cfg, prior):
        self.cfg = cfg
        self.last = {}
        self.reject = {}
        self.hist = {k: deque(maxlen=max(1, int(cfg["smooth"]))) for k in JOINT_KEYS}
        self.parent, self.bone_of = {}, {}
        for f, names in FINGERS.items():
            for i, n in enumerate(names):
                key = f"{f}_{n}"
                self.parent[key] = "wrist" if i == 0 else f"{f}_{names[i-1]}"
                self.bone_of[key] = None if i == 0 else prior[f][i - 1]

    def accept(self, key, X, accepted):
        ok = X is not None and np.all(np.isfinite(X))
        if ok:
            prev = self.last.get(key)
            if prev is not None and np.linalg.norm(X - prev) > self.cfg["max_jump_mm"]:
                ok = False
        if ok:
            L = self.bone_of.get(key)
            par = accepted.get(self.parent.get(key, ""))
            if L and par is not None:
                if abs(np.linalg.norm(X - par) - L) > self.cfg["bone_tol"] * L:
                    ok = False
        if ok:
            self.reject[key] = 0
        else:
            self.reject[key] = self.reject.get(key, 0) + 1
            if self.reject[key] >= self.cfg["reject_reset_frames"]:
                # 첫 관측이 틀렸으면 속도 게이트가 계속 막으므로 리셋
                self.last.pop(key, None)
                self.hist[key].clear()
                self.reject[key] = 0
        return ok

    def commit(self, key, X):
        self.last[key] = X
        self.hist[key].append(X)

    def smoothed(self, key):
        h = self.hist[key]
        return np.median(np.array(h), axis=0) if h else None


def _assign_partial(tracker, f, names, pts):
    """마커 일부만 보일 때: 직전 관절 위치와 가까운 순으로 짝짓기."""
    keys = [f"{f}_{n}" for n in names]
    pairs = []
    for i, X in enumerate(pts):
        for k in keys:
            prev = tracker.last.get(k)
            if prev is not None:
                pairs.append((float(np.linalg.norm(X - prev)), i, k))
    pairs.sort()
    used_i, used_k, out = set(), set(), {}
    for d, i, k in pairs:
        if i in used_i or k in used_k or d > tracker.cfg["max_jump_mm"]:
            continue
        used_i.add(i)
        used_k.add(k)
        out[k] = i
    return out


def process_view(cv2, view, hsv, depth, cam, colors, prior, kernel, cfg):
    """뷰 하나: 마커 검출 → depth 3D → 사슬 정렬 → 관절각."""
    tracker = view["tracker"]
    roi = view["roi"]
    J, px_of, mask = {}, {}, None
    min_area = int(cfg["min_blob_area"])

    def blobs3d(part, want):
        nonlocal mask
        pts, lab, ids, m = find_blobs(cv2, hsv, colors[part], kernel, want, roi, min_area)
        mask = m if mask is None else (mask | m)
        out = []
        for p, bid in zip(pts, ids):
            z = blob_depth(cv2, depth, lab, bid, cam["scale"], kernel)
            if z is not None:
                out.append((backproject(cv2, cam["K"], cam["D"], p, z), p))
        return out

    w = blobs3d("wrist", 1)
    if w and tracker.accept("wrist", w[0][0], J):
        J["wrist"] = w[0][0]
        px_of["wrist"] = w[0][1]

    for f, names in FINGERS.items():
        cands = blobs3d(f, len(names))
        if not cands:
            continue
        P = np.array([c[0] for c in cands])
        if len(P) == len(names):
            order = order_chain(P, J.get("wrist"), prior[f])
            assign = {f"{f}_{names[i]}": oi for i, oi in enumerate(order)}
        else:
            assign = _assign_partial(tracker, f, names, P)
        for n in names:                      # 부모 → 자식 순서로 게이트
            key = f"{f}_{n}"
            if key in assign:
                i = assign[key]
                if tracker.accept(key, P[i], J):
                    J[key] = P[i]
                    px_of[key] = cands[i][1]

    for k, X in J.items():
        tracker.commit(k, X)
    smooth = {k: tracker.smoothed(k) for k in J}
    return dict(angles=compute_angles(smooth), n=len(J), px=px_of, mask=mask)


def fuse_angles(results, max_diff):
    """같은 손의 여러 뷰 관절각 합치기. 값이 비슷하면 관절 수로 가중 평균,
    크게 다르면 관절을 더 많이 잡은 뷰 값을 택한다."""
    out = {}
    for name in ANGLE_NAMES:
        vals = [(r["angles"][name], r["n"]) for r in results
                if r is not None and math.isfinite(r["angles"][name])]
        if not vals:
            out[name] = float("nan")
        elif len(vals) == 1:
            out[name] = vals[0][0]
        else:
            vs = [v for v, _ in vals]
            if max(vs) - min(vs) <= max_diff:
                wsum = sum(n for _, n in vals) or 1
                out[name] = sum(v * n for v, n in vals) / wsum
            else:
                out[name] = max(vals, key=lambda x: x[1])[0]
    return out


# ═══════════════════════════════════════════════════════ 카메라 프로세스

def list_cameras():
    import pyrealsense2 as rs
    ctx = rs.context()
    return sorted((d.get_info(rs.camera_info.serial_number), d.get_info(rs.camera_info.name))
                  for d in ctx.query_devices()
                  if "Platform Camera" not in d.get_info(rs.camera_info.name))


def _put_latest(q, item):
    if q is None:
        return
    try:
        q.put_nowait(item)
    except queue.Full:
        try:
            q.get_nowait()
        except queue.Empty:
            pass
        try:
            q.put_nowait(item)
        except queue.Full:
            pass


HAND_BGR = {"G1": (60, 160, 255), "G2": (255, 160, 60)}


def _draw_preview(cv2, cams, views, fused, fps, show_mask, scale):
    tiles = []
    for role, cam in cams.items():
        img = cam.get("last_color")
        if img is None:
            img = np.zeros((480, 640, 3), np.uint8)
        img = img.copy()
        if show_mask:
            m = None
            for v in views:
                if v["camera"] == role and v["result"] is not None and v["result"]["mask"] is not None:
                    m = v["result"]["mask"] if m is None else (m | v["result"]["mask"])
            if m is not None:
                img = cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)
        for v in views:
            if v["camera"] != role:
                continue
            col = HAND_BGR[v["hand"]]
            if v["roi"] is not None:
                x0, y0, x1, y1 = v["roi"]
                cv2.rectangle(img, (x0, y0), (x1 - 1, y1 - 1), col, 2)
            r = v["result"]
            if r is None:
                continue
            for key, (x, y) in r["px"].items():
                cv2.circle(img, (int(x), int(y)), 5, col, 2)
                cv2.putText(img, key.split("_")[-1] if key != "wrist" else "W",
                            (int(x) + 6, int(y) - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1)
            x0 = v["roi"][0] if v["roi"] is not None else 0
            cv2.putText(img, f'{v["name"]} {r["n"]}/12', (x0 + 6, 470),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
        cv2.rectangle(img, (0, 0), (640, 22), (0, 0, 0), -1)
        cv2.putText(img, f'{role}  {cam["serial"]}  {cam["rate"]:.0f} fps',
                    (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        tiles.append(img)
    info = np.zeros((480, 640, 3), np.uint8)
    y = 24
    cv2.putText(info, f"fused  {fps:.0f} Hz   [M] mask", (8, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    for hand in HANDS:
        y += 30
        cv2.putText(info, hand, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, HAND_BGR[hand], 2)
        for part in FINGERS:
            y += 22
            t = " ".join(f"{fused[hand][j]:5.0f}" if math.isfinite(fused[hand][j]) else "  ---"
                         for j in FINGER_JOINTS[part])
            cv2.putText(info, f"{part:7s} {t}", (20, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, HAND_BGR[hand], 1)
    tiles.append(info)
    while len(tiles) < 4:
        tiles.append(np.zeros_like(info))
    grid = np.vstack([np.hstack(tiles[0:2]), np.hstack(tiles[2:4])])
    if scale != 1.0:
        grid = cv2.resize(grid, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    return grid


def run_vision(cfg, out_q=None, stop_evt=None, standalone=False):
    """카메라 루프. out_q 로 최신 결과(dict)를 보낸다. 오류는 {'error': msg} 로 보낸다."""
    try:
        import cv2
        import pyrealsense2 as rs
    except Exception as exc:
        _put_latest(out_q, {"error": f"import 실패: {exc}"})
        return

    cams = {}
    try:
        colors = load_colors(cfg)
        bones = load_bones(cfg)

        # ── 카메라 배정 ──
        connected = [s for s, _ in list_cameras()]
        wanted = {r: (cfg["cameras"].get(r) or "") for r in CAMERA_ROLES}
        free = [s for s in connected if s not in wanted.values()]
        for r in CAMERA_ROLES:
            if not wanted[r] and free:
                wanted[r] = free.pop(0)
                print(f"[vision] {r}: 시리얼 미지정 → {wanted[r]} 자동 배정 (vision_config.json 에 고정 권장)")
        W, H, fps = int(cfg["width"]), int(cfg["height"]), int(cfg["fps"])
        ctx = rs.context()
        for r in CAMERA_ROLES:
            s = wanted[r]
            if not s:
                print(f"[vision] {r}: 카메라 없음 — 이 뷰는 건너뜀")
                continue
            if s not in connected:
                print(f"[vision] {r}: {s} 연결 안 됨 — 건너뜀")
                continue
            pipe = rs.pipeline(ctx)
            c = rs.config()
            c.enable_device(s)
            c.enable_stream(rs.stream.color, W, H, rs.format.bgr8, fps)
            c.enable_stream(rs.stream.depth, W, H, rs.format.z16, fps)
            prof = pipe.start(c)
            intr = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
            cams[r] = dict(serial=s, pipe=pipe, align=rs.align(rs.stream.color), K=K,
                           D=np.asarray(intr.coeffs, float),
                           scale=prof.get_device().first_depth_sensor().get_depth_scale(),
                           last_color=None, t=0.0, rate=0.0, ok=True)
            print(f"[vision] {r}: {s} 시작")
        if not cams:
            raise RuntimeError("사용할 수 있는 RealSense 카메라가 없습니다.")

        views = []
        for v in cfg["views"]:
            if v["camera"] not in cams:
                continue
            roi = v.get("roi")
            if roi is not None:
                x0, y0, x1, y1 = [int(t) for t in roi]
                roi = (max(0, x0), max(0, y0), min(W, x1), min(H, y1))
            views.append(dict(name=v.get("name", f'{v["camera"]}_{v["hand"]}'),
                              camera=v["camera"], hand=v["hand"], roi=roi,
                              tracker=ViewTracker(cfg, bones[v["hand"]]),
                              result=None, t=0.0))
        kernel = np.ones((3, 3), np.uint8)
        win = "vision markers (top / side_G1 / side_G2)"
        if cfg.get("preview", True):
            cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        show_mask = False
        seq, rate, t_prev, t_prev_show, t_print = 0, 0.0, time.time(), 0.0, 0.0
        stale_s = 3.0 / max(1, fps) + 0.2

        while stop_evt is None or not stop_evt.is_set():
            got = False
            for role, cam in cams.items():
                ok, fs = cam["pipe"].try_wait_for_frames(1)
                if not ok:
                    continue
                fs = cam["align"].process(fs)
                cf, df = fs.get_color_frame(), fs.get_depth_frame()
                if not cf or not df:
                    continue
                color = np.asanyarray(cf.get_data()).copy()
                depth = np.asanyarray(df.get_data()).copy()
                now = time.time()
                if cam["t"]:
                    cam["rate"] += 0.1 * (1.0 / max(1e-3, now - cam["t"]) - cam["rate"])
                cam["t"] = now
                cam["last_color"] = color
                hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
                for v in views:
                    if v["camera"] == role:
                        v["result"] = process_view(cv2, v, hsv, depth, cam,
                                                   colors[v["hand"]], bones[v["hand"]],
                                                   kernel, cfg)
                        v["t"] = now
                got = True
            if not got:
                continue

            now = time.time()
            fused, per_view = {}, {}
            for hand in HANDS:
                rs_ = [v["result"] for v in views
                       if v["hand"] == hand and now - v["t"] < stale_s]
                fused[hand] = fuse_angles(rs_, float(cfg["fuse_max_diff_deg"]))
            for v in views:
                r = v["result"]
                per_view[v["name"]] = dict(
                    hand=v["hand"], n=r["n"] if r else 0,
                    fresh=bool(r) and now - v["t"] < stale_s)
            rate += 0.1 * (1.0 / max(1e-3, now - t_prev) - rate)
            t_prev = now
            seq += 1
            _put_latest(out_q, dict(
                t=now, seq=seq, fps=rate,
                cams={r: dict(serial=c["serial"], fps=c["rate"],
                              ok=now - c["t"] < stale_s) for r, c in cams.items()},
                views=per_view,
                hands={h: {"angles": fused[h]} for h in HANDS}))

            if standalone and now - t_print > 1.0:
                t_print = now
                for h in HANDS:
                    print(h, " ".join(f"{k}={fused[h][k]:.0f}" for k in ANGLE_NAMES
                                      if math.isfinite(fused[h][k])))
            if cfg.get("preview", True) and now - t_prev_show > 0.066:
                t_prev_show = now
                cv2.imshow(win, _draw_preview(cv2, cams, views, fused, rate, show_mask,
                                              float(cfg.get("preview_scale", 0.6))))
                k = cv2.waitKey(1) & 0xFF
                if k == ord("m"):
                    show_mask = not show_mask
                if standalone and k in (ord("q"), 27):
                    break
    except Exception as exc:
        import traceback
        traceback.print_exc()
        _put_latest(out_q, {"error": f"{type(exc).__name__}: {exc}"})
    finally:
        for cam in cams.values():
            try:
                cam["pipe"].stop()
            except Exception:
                pass
        try:
            import cv2
            cv2.destroyAllWindows()
        except Exception:
            pass


# ═══════════════════════════════════════════════════════ 장갑 코드 쪽 연결

class VisionLink:
    """비전 프로세스를 띄우고 최신 결과를 손가락 curl 로 바꿔 보관한다.
    제어 스레드는 get_curl() / angles() 만 호출한다(짧은 lock)."""

    def __init__(self, config_path=CONFIG_PATH, rom_path=ROM_PATH):
        self.config_path = Path(config_path)
        self.rom_path = Path(rom_path)
        self.rom = load_rom(self.rom_path)
        self.lock = threading.Lock()
        self.proc = self.q = self.stop_evt = self.thread = None
        self._reading = False
        self.latest = None
        self.curls = {}
        self.error = ""
        self.capture = None
        self.capture_msg = ""

    # ── 프로세스 ──
    @property
    def running(self):
        return self.proc is not None and self.proc.is_alive()

    def start(self):
        if self.running:
            return
        import multiprocessing as mp
        cfg = load_config(self.config_path)
        load_colors(cfg)                  # 색 파일 문제는 여기서 바로 알림
        ctx = mp.get_context("spawn")
        self.q = ctx.Queue(maxsize=2)
        self.stop_evt = ctx.Event()
        self.proc = ctx.Process(target=run_vision, args=(cfg, self.q, self.stop_evt),
                                daemon=True, name="vision")
        self.proc.start()
        self.error = ""
        self._reading = True
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def stop(self):
        self._reading = False
        if self.stop_evt is not None:
            self.stop_evt.set()
        if self.proc is not None:
            self.proc.join(timeout=3.0)
            if self.proc.is_alive():
                self.proc.terminate()
        self.proc = None
        with self.lock:
            self.latest = None
            self.curls = {}

    def _reader(self):
        while self._reading:
            try:
                pkt = self.q.get(timeout=0.2)
            except queue.Empty:
                if self.proc is not None and not self.proc.is_alive() and self._reading:
                    self.error = self.error or "비전 프로세스 종료됨"
                    self._reading = False
                continue
            except (EOFError, OSError):
                break
            if "error" in pkt:
                self.error = pkt["error"]
                continue
            self._ingest(pkt)

    def _ingest(self, pkt):
        curls = {}
        for hand, h in pkt["hands"].items():
            for part, fid in PART_TO_FID.items():
                curls[(hand, fid)] = finger_curl(h["angles"], self.rom.get(hand), part)
        cap = self.capture
        if cap is not None:
            ang = pkt["hands"][cap["hand"]]["angles"]
            for j in ANGLE_NAMES:
                if math.isfinite(ang[j]):
                    cap["samples"][j].append(180.0 - ang[j])
            if time.time() >= cap["t_end"]:
                self._finish_capture()
        with self.lock:
            self.latest = pkt
            self.curls = curls

    # ── 조회 (제어 스레드) ──
    def get_curl(self, hand, fid):
        """(curl or None, age_s)."""
        with self.lock:
            if self.latest is None:
                return None, float("inf")
            return self.curls.get((hand, fid)), time.time() - self.latest["t"]

    def angles(self, hand):
        with self.lock:
            if self.latest is None:
                return [float("nan")] * len(ANGLE_NAMES)
            a = self.latest["hands"][hand]["angles"]
            return [a[k] for k in ANGLE_NAMES]

    def status(self):
        with self.lock:
            pkt = self.latest
        if not self.running:
            return "정지" + (f" — {self.error}" if self.error else "")
        if pkt is None:
            return "카메라 시작 중..." + (f" — {self.error}" if self.error else "")
        age = (time.time() - pkt["t"]) * 1000
        cams = "  ".join(f'{r} {c["fps"]:.0f}fps' + ("" if c["ok"] else "(끊김)")
                         for r, c in pkt["cams"].items())
        views = "  ".join(f'{n}:{v["n"]}' for n, v in pkt["views"].items())
        return f"{pkt['fps']:.0f} Hz  지연 {age:.0f} ms  |  {cams}  |  관절 {views}"

    # ── ROM (편 손 / 주먹) 기록 ──
    def has_rom(self, hand, fid):
        r = self.rom.get(hand, {})
        part = FID_TO_PART[fid]
        return all(j in r.get("open", {}) and j in r.get("fist", {})
                   for j in FINGER_JOINTS[part])

    def start_capture(self, hand, kind, seconds=2.0):
        if not self.running:
            raise RuntimeError("비전을 먼저 시작하세요.")
        self.capture = dict(hand=hand, kind=kind, t_end=time.time() + seconds,
                            samples={j: [] for j in ANGLE_NAMES})
        self.capture_msg = f"{hand} {'편 손' if kind == 'open' else '주먹'} 기록 중 ({seconds:.0f}s)..."

    def _finish_capture(self):
        cap, self.capture = self.capture, None
        got, miss = {}, []
        for j, v in cap["samples"].items():
            if len(v) >= 5:
                got[j] = float(np.median(v))
            else:
                miss.append(j)
        hand_rom = self.rom.setdefault(cap["hand"], {})
        hand_rom.setdefault(cap["kind"], {}).update(got)
        hand_rom["date"] = time.strftime("%Y-%m-%d %H:%M:%S")
        save_rom(self.rom, self.rom_path)
        label = "편 손" if cap["kind"] == "open" else "주먹"
        self.capture_msg = (f"{cap['hand']} {label} 저장: {len(got)}/{len(ANGLE_NAMES)} 관절"
                            + (f" (안 보임: {', '.join(miss)})" if miss else ""))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list", action="store_true", help="연결된 RealSense 시리얼 출력")
    ap.add_argument("--config", type=Path, default=CONFIG_PATH)
    a = ap.parse_args()
    if a.list:
        for s, n in list_cameras():
            print(f"{s}: {n}")
        return
    cfg = load_config(a.config)
    cfg["preview"] = True
    run_vision(cfg, standalone=True)


if __name__ == "__main__":
    main()
