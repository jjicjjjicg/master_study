"""
marker_track_3d.py — 카메라 여러 대에서 동시에 마커를 잡아, 각 카메라의 depth 로 3D 좌표를 구해
실시간 CSV 저장 + 3D 공간에서 움직임을 보여준다.

- 색 범위: 카메라마다 hsv_<serial>.json  (marker_hsv_tune.py 로 먼저 만들기)
- 3D 좌표: 각 카메라 자기 좌표계 (mm).  X 오른쪽, Y 아래, Z 카메라 앞쪽 (RealSense 기본)
  카메라끼리 외부 캘리브레이션은 하지 않으므로 카메라마다 3D 뷰가 따로 있다.
- 화면: 카메라마다 한 줄 = [컬러 + 검출 표시] | [3D 뷰 + 좌표 목록]
  3D 뷰에서는 보기 편하게 위쪽 = -Y 로 그린다 (CSV 에는 원래 카메라 좌표 그대로 저장).

실행:
    conda activate realsense
    python marker_track_3d.py
    python marker_track_3d.py --res 1280x720      # 튜닝한 해상도와 같게
    python marker_track_3d.py --no-save           # CSV 저장 안 함

저장 (data/ 폴더):
    marker3d_<날짜시간>.csv : 프레임마다 검출된 마커 한 줄씩
        wall_time, t_s, camera, serial, frame_no, cam_ts_ms, marker, id, u, v, area, X_mm, Y_mm, Z_mm
        (depth 를 못 읽은 마커는 X/Y/Z 가 빈칸)
    V 키로 화면 녹화 시 marker3d_<날짜시간>.mp4

조작 (3D 뷰 위에서):
    왼쪽 드래그 : 회전          오른쪽 드래그 : 이동          휠 : 확대/축소
    1 정면(카메라 시점)  2 위에서  3 옆에서  0 기본 시점   F 현재 마커 위치로 중심 맞추기
키:
    R : CSV 저장 일시정지/재개     V : 화면 녹화 시작/정지     C : 궤적 지우기
    Q / ESC : 종료
"""

import argparse
import csv
import math
import shutil
import tempfile
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np

from marker_common import (HERE, RSCamera, cv2, detect, list_devices, load_hsv, put, range_bgr)

BAR_H = 30
MATCH_PX = 80          # 프레임 간 같은 마커로 볼 최대 픽셀 이동
ID_TIMEOUT = 0.5       # 이 시간 동안 안 보이면 그 id 해제


# --------------------------------------------------------------------------- id 유지
class IdAssigner:
    """같은 색 마커가 여러 개일 때 프레임 간 번호(id)가 바뀌지 않게 가까운 것끼리 짝짓기."""

    def __init__(self, count):
        self.count = count
        self.prev = {}     # id -> (u, v, t)

    def assign(self, dets, t):
        self.prev = {i: p for i, p in self.prev.items() if t - p[2] < ID_TIMEOUT}
        pairs = sorted((math.hypot(d["u"] - p[0], d["v"] - p[1]), di, i)
                       for di, d in enumerate(dets) for i, p in self.prev.items())
        used_d, used_i, ids = set(), set(), {}
        for dist, di, i in pairs:
            if dist > MATCH_PX or di in used_d or i in used_i:
                continue
            used_d.add(di)
            used_i.add(i)
            ids[di] = i
        free = [i for i in range(self.count) if i not in used_i and i not in self.prev]
        free += [i for i in range(self.count) if i not in used_i and i in self.prev]
        for di in range(len(dets)):
            if di not in ids and free:
                ids[di] = free.pop(0)
        for di, i in ids.items():
            d = dets[di]
            d["id"] = i
            self.prev[i] = (d["u"], d["v"], t)
        return [d for d in dets if "id" in d]


# --------------------------------------------------------------------------- 3D 뷰
class View3D:
    def __init__(self):
        self.reset()

    def reset(self):
        self.yaw, self.pitch, self.dist = math.radians(-35), math.radians(22), 1300.0
        self.target = np.array([0.0, 500.0, 0.0])

    def preset(self, k):
        self.target = np.array([0.0, 500.0, 0.0])
        if k == 1:
            self.yaw, self.pitch, self.dist = 0.0, 0.0, 900.0
        elif k == 2:
            self.yaw, self.pitch, self.dist = 0.0, math.radians(89), 1300.0
        elif k == 3:
            self.yaw, self.pitch, self.dist = math.radians(-90), 0.0, 1300.0

    def basis(self):
        cp, sp, cy, sy = math.cos(self.pitch), math.sin(self.pitch), math.cos(self.yaw), math.sin(self.yaw)
        fwd = np.array([cp * sy, cp * cy, -sp])
        right = np.cross(fwd, [0, 0, 1.0])
        if np.linalg.norm(right) < 1e-6:
            right = np.array([1.0, 0, 0])
        right /= np.linalg.norm(right)
        up = np.cross(right, fwd)
        eye = self.target - fwd * self.dist
        return eye, right, up, fwd

    def project(self, P, W, H):
        """world (N,3) -> (N,2) 픽셀, (N,) 깊이. 뒤에 있는 점은 깊이 <= 0."""
        eye, right, up, fwd = self.basis()
        R = np.stack([right, up, fwd])
        pc = (np.atleast_2d(P) - eye) @ R.T
        f = 0.9 * W
        z = pc[:, 2]
        zs = np.where(z > 1, z, 1)
        uv = np.stack([W / 2 + f * pc[:, 0] / zs, H / 2 - f * pc[:, 1] / zs], axis=1)
        return uv, z


def cam_to_world(xyz):
    """카메라 좌표(X 오른쪽, Y 아래, Z 앞) -> 표시용(x 오른쪽, y 앞, z 위)."""
    xyz = np.asarray(xyz, float)
    return np.stack([xyz[..., 0], xyz[..., 2], -xyz[..., 1]], axis=-1)


def draw_line3d(img, view, a, b, col, th=1):
    H, W = img.shape[:2]
    uv, z = view.project(np.array([a, b]), W, H)
    if z[0] > 1 and z[1] > 1:
        cv2.line(img, tuple(np.int32(uv[0])), tuple(np.int32(uv[1])), col, th, cv2.LINE_AA)


def render_3d(view, W, H, cam_label, markers_now, trails, colors, trail_s, now):
    img = np.full((H, W, 3), 22, np.uint8)
    floor = -250.0
    # 바닥 격자 (카메라 아래 25cm, 100mm 간격)
    for x in range(-600, 601, 100):
        draw_line3d(img, view, (x, 0, floor), (x, 1400, floor), (60, 60, 60) if x else (90, 90, 90))
    for y in range(0, 1401, 100):
        draw_line3d(img, view, (-600, y, floor), (600, y, floor), (60, 60, 60) if y % 500 else (90, 90, 90))
    for y in range(500, 1401, 500):
        uv, z = view.project(np.array([(-600, y, floor)]), W, H)
        if z[0] > 1:
            put(img, f"{y / 10:.0f}cm", tuple(np.int32(uv[0])), 0.4, (120, 120, 120))
    # 카메라 (원점) + 시야 피라미드 + 축
    o = (0, 0, 0)
    for cx, cz in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        draw_line3d(img, view, o, (cx * 60, 100, cz * 45), (200, 200, 200))
    for (a, b) in [((-60, 100, -45), (60, 100, -45)), ((60, 100, -45), (60, 100, 45)),
                   ((60, 100, 45), (-60, 100, 45)), ((-60, 100, 45), (-60, 100, -45))]:
        draw_line3d(img, view, a, b, (200, 200, 200))
    draw_line3d(img, view, o, (150, 0, 0), (60, 60, 255), 2)     # X 오른쪽 (빨강)
    draw_line3d(img, view, o, (0, 150, 0), (255, 160, 60), 2)    # Z 앞 (파랑)
    draw_line3d(img, view, o, (0, 0, 150), (60, 220, 60), 2)     # -Y 위 (초록)

    # 궤적
    for key, tr in trails.items():
        pts = [(t, p) for t, p in tr if now - t <= trail_s]
        if len(pts) < 2:
            continue
        uv, z = view.project(np.array([p for _, p in pts]), W, H)
        col = np.array(colors.get(key[0], (200, 200, 200)), float)
        for i in range(1, len(pts)):
            if z[i - 1] > 1 and z[i] > 1:
                a = 1.0 - (now - pts[i][0]) / trail_s
                c = tuple(int(v) for v in col * (0.25 + 0.75 * a))
                cv2.line(img, tuple(np.int32(uv[i - 1])), tuple(np.int32(uv[i])), c, 1, cv2.LINE_AA)

    # 현재 마커 (먼 것부터)
    pts = [(m, cam_to_world(m["xyz"])) for m in markers_now if m["xyz"] is not None]
    if pts:
        P = np.array([p for _, p in pts])
        uv, z = view.project(P, W, H)
        fl = P.copy()
        fl[:, 2] = floor
        uvf, zf = view.project(fl, W, H)
        for i in np.argsort(-z):
            if z[i] <= 1:
                continue
            m = pts[i][0]
            col = colors.get(m["name"], (200, 200, 200))
            p0 = tuple(np.int32(uv[i]))
            if zf[i] > 1:
                cv2.line(img, p0, tuple(np.int32(uvf[i])), (70, 70, 70), 1)       # 바닥까지 수직선
                cv2.circle(img, tuple(np.int32(uvf[i])), 2, (90, 90, 90), -1)
            r = int(np.clip(0.9 * W * 7 / z[i], 3, 20))
            cv2.circle(img, p0, r, col, -1, cv2.LINE_AA)
            cv2.circle(img, p0, r, (0, 0, 0), 1, cv2.LINE_AA)
            put(img, f"{m['name']}{m['id']}", (p0[0] + r + 3, p0[1] + 4), 0.4, col)

    # 좌표 목록 (카메라 좌표, mm)
    cv2.rectangle(img, (0, 0), (W, 22), (0, 0, 0), -1)
    put(img, f"3D  {cam_label}   (camera frame, mm)", (6, 16), 0.5)
    rows = sorted(markers_now, key=lambda m: (m["name"], m["id"]))
    y = H - 8 - 16 * (len(rows) - 1)
    if rows:
        put(img, "marker      X      Y      Z", (8, y - 18), 0.42, (150, 150, 150))
    for m in rows:
        col = colors.get(m["name"], (200, 200, 200))
        if m["xyz"] is not None:
            X, Y, Z = m["xyz"]
            t = f"{m['name'][:6]:6s}{m['id']} {X:6.0f} {Y:6.0f} {Z:6.0f}"
        else:
            t = f"{m['name'][:6]:6s}{m['id']}   no depth"
        put(img, t, (8, y), 0.42, col)
        y += 16
    return img


# --------------------------------------------------------------------------- 녹화
class VideoRec:
    """한글 경로에서 VideoWriter 가 실패하므로 임시폴더(ASCII)에 쓰고 끝나면 옮김. 실제 시간 속도 유지."""

    def __init__(self, path, fps, size):
        self.final = path
        self.tmp = Path(tempfile.gettempdir()) / path.name
        self.fps = fps
        self.w = cv2.VideoWriter(str(self.tmp), cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
        if not self.w.isOpened():
            raise RuntimeError("VideoWriter 열기 실패")
        self.t0, self.n = time.time(), 0

    def write(self, frame):
        target = int((time.time() - self.t0) * self.fps) + 1
        while self.n < target:
            self.w.write(frame)
            self.n += 1

    def close(self):
        self.w.release()
        shutil.move(str(self.tmp), str(self.final))
        return self.final


# --------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--serial", nargs="*", help="사용할 카메라 시리얼 (생략 시 전부)")
    ap.add_argument("--res", default="640x480", help="HSV 튜닝한 해상도와 같게")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--trail", type=float, default=2.0, help="3D 궤적 길이(초)")
    ap.add_argument("--hsv-dir", default=str(HERE), help="hsv_<serial>.json 이 있는 폴더")
    ap.add_argument("--out-dir", default=str(HERE / "data"))
    ap.add_argument("--no-save", action="store_true", help="CSV 저장 안 함")
    ap.add_argument("--win-h", type=int, default=950, help="처음 창 높이")
    ap.add_argument("--duration", type=float, default=0, help="N초 후 자동 종료(테스트용)")
    ap.add_argument("--record", action="store_true", help="시작하자마자 화면 녹화")
    args = ap.parse_args()
    W, H = map(int, args.res.lower().split("x"))

    devs = list_devices()
    print("연결된 RealSense:", ", ".join(f"{n} ({s})" for n, s in devs) or "없음")
    serials = args.serial or [s for _, s in devs]

    cams, conf = [], {}
    for s in serials:
        hsv = load_hsv(s, args.hsv_dir)
        conf[s] = dict(hsv=hsv, ids={n: IdAssigner(int(r.get("count", 1))) for n, r in hsv.items()},
                       queue=deque(maxlen=600))
        if not hsv:
            print(f"  경고: hsv_{s}.json 없음 → 이 카메라는 영상만 표시 (marker_hsv_tune.py 먼저)")

        def processor(cam, item, c=conf[s]):
            out = []
            for name, rng in c["hsv"].items():
                dets, _ = detect(cam, item, rng, int(rng.get("count", 1)))
                for d in c["ids"][name].assign(dets, item["t"]):
                    d["name"] = name
                    out.append(d)
            c["queue"].append((item["t"], item["frame_no"], item["ts_ms"], out))
            return out

        try:
            cams.append(RSCamera(s, W, H, args.fps, processor=processor))
            print("열림:", cams[-1].label, " 마커:", ", ".join(f"{n}x{r.get('count', 1)}" for n, r in conf[s]["hsv"].items()))
        except Exception as e:
            print(f"열기 실패 {s}: {e}")
    if not cams:
        return

    colors = {}
    for c in conf.values():
        for n, r in c["hsv"].items():
            colors.setdefault(n, range_bgr(r))

    # ---- 저장 ----
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = f"{datetime.now():%Y%m%d_%H%M%S}"
    csv_f = writer = None
    saving = not args.no_save
    if saving:
        csv_path = out_dir / f"marker3d_{stamp}.csv"
        csv_f = open(csv_path, "w", newline="", encoding="utf-8-sig")
        writer = csv.writer(csv_f)
        writer.writerow(["wall_time", "t_s", "camera", "serial", "frame_no", "cam_ts_ms", "marker", "id",
                         "u", "v", "area", "X_mm", "Y_mm", "Z_mm"])
        print("CSV 저장:", csv_path)
    n_rows, t_flush = 0, time.time()
    vrec = None

    view = View3D()
    trails = {c.serial: {} for c in cams}
    current = {c.serial: [] for c in cams}
    drag = dict(btn=None, x=0, y=0)
    row_h = H

    def on_mouse(ev, x, y, flags, *_):
        if ev in (cv2.EVENT_LBUTTONDOWN, cv2.EVENT_RBUTTONDOWN):
            if x >= W and y >= BAR_H:
                drag.update(btn="L" if ev == cv2.EVENT_LBUTTONDOWN else "R", x=x, y=y)
        elif ev in (cv2.EVENT_LBUTTONUP, cv2.EVENT_RBUTTONUP):
            drag["btn"] = None
        elif ev == cv2.EVENT_MOUSEMOVE and drag["btn"]:
            dx, dy = x - drag["x"], y - drag["y"]
            drag.update(x=x, y=y)
            if drag["btn"] == "L":
                view.yaw += dx * 0.01
                view.pitch = float(np.clip(view.pitch + dy * 0.01, -1.55, 1.55))
            else:
                _, right, up, _ = view.basis()
                s = view.dist / (0.9 * W)
                view.target = view.target - right * dx * s + up * dy * s
        elif ev == cv2.EVENT_MOUSEWHEEL:
            view.dist = float(np.clip(view.dist * (0.9 if cv2.getMouseWheelDelta(flags) > 0 else 1.1), 150, 6000))

    win = "marker 3D track"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    total_h = BAR_H + row_h * len(cams)
    if total_h > args.win_h:
        cv2.resizeWindow(win, int(2 * W * args.win_h / total_h), args.win_h)
    cv2.setMouseCallback(win, on_mouse)

    t_start = time.time()
    try:
        while True:
            now = time.time()
            # ---- 카메라 스레드 결과 꺼내기: CSV + 궤적 ----
            for c in cams:
                q = conf[c.serial]["queue"]
                while q:
                    t, fno, ts, dets = q.popleft()
                    current[c.serial] = dets
                    for d in dets:
                        if d["xyz"] is not None:
                            tr = trails[c.serial].setdefault((d["name"], d["id"]), deque(maxlen=300))
                            tr.append((t, cam_to_world(d["xyz"])))
                        if saving:
                            xyz = d["xyz"]
                            writer.writerow([f"{t:.4f}", f"{t - t_start:.4f}", c.model, c.serial, fno, f"{ts:.1f}",
                                             d["name"], d["id"], f"{d['u']:.1f}", f"{d['v']:.1f}", d["area"]]
                                            + ([f"{v:.1f}" for v in xyz] if xyz is not None else ["", "", ""]))
                            n_rows += 1
            if saving and now - t_flush > 1.0:
                csv_f.flush()
                t_flush = now

            # ---- 화면 ----
            rows = []
            for c in cams:
                it = c.get()
                img = it["color"].copy() if it is not None else np.zeros((H, W, 3), np.uint8)
                dets = current[c.serial]
                for d in dets:
                    col = colors.get(d["name"], (200, 200, 200))
                    p = (int(d["u"]), int(d["v"]))
                    cv2.circle(img, p, 8, col, 2)
                    put(img, f"{d['name']}{d['id']}" + ("" if d["xyz"] is not None else " ?"),
                        (p[0] + 9, p[1] - 6), 0.4, col)
                hsv = conf[c.serial]["hsv"]
                cv2.rectangle(img, (0, 0), (W, 22), (0, 0, 0), -1)
                put(img, f"{c.label}  {c.fps_val:4.1f}fps", (6, 16), 0.5)
                if not hsv:
                    put(img, f"no hsv_{c.serial}.json -> run marker_hsv_tune.py", (10, H // 2), 0.6, (0, 0, 255), 2)
                else:
                    y = H - 10 - 18 * (len(hsv) - 1)
                    for n, r in hsv.items():
                        k = sum(1 for d in dets if d["name"] == n)
                        k3 = sum(1 for d in dets if d["name"] == n and d["xyz"] is not None)
                        want = int(r.get("count", 1))
                        col = (80, 220, 80) if k3 == want else ((0, 200, 255) if k else (60, 60, 255))
                        put(img, f"{n:7s} {k3}/{want}" + (f" ({k - k3} no depth)" if k > k3 else ""), (8, y), 0.5, col)
                        y += 18
                v3 = render_3d(view, W, H, c.model, dets, trails[c.serial], colors, args.trail, now)
                row = np.hstack([img, v3])
                cv2.line(row, (W, 0), (W, H), (90, 90, 90), 1)
                cv2.line(row, (0, H - 1), (2 * W, H - 1), (90, 90, 90), 2)
                rows.append(row)

            bar = np.full((BAR_H, 2 * W, 3), 40, np.uint8)
            st = f"CSV {'ON' if saving else 'PAUSED' if writer else 'OFF'} {n_rows} rows" if writer else "CSV OFF"
            put(bar, f"marker 3D track  {len(cams)} cam  |  {st}", (8, 20), 0.5, (170, 170, 170))
            put(bar, "drag:rotate  R-drag:pan  wheel:zoom  1/2/3/0 view  F fit  C clear  R csv  V video  Q quit",
                (2 * W - 700, 20), 0.42, (150, 150, 150))
            ui = np.vstack([bar] + rows)

            if args.record and vrec is None:
                vrec = VideoRec(out_dir / f"marker3d_{stamp}.mp4", args.fps, (ui.shape[1], ui.shape[0]))
                args.record = False
            if vrec is not None:
                vrec.write(ui)
                el = time.time() - vrec.t0
                cv2.circle(ui, (2 * W - 760, 15), 7, (0, 0, 255), -1)
                put(ui, f"REC {int(el // 60):02d}:{int(el % 60):02d}", (2 * W - 748, 21), 0.5, (0, 0, 255), 2)

            cv2.imshow(win, ui)
            k = cv2.waitKey(1) & 0xFF
            ch = chr(k).lower() if k < 128 else ""
            if ch == "q" or k == 27:
                break
            elif ch in ("0", "1", "2", "3"):
                view.reset() if ch == "0" else view.preset(int(ch))
            elif ch == "f":
                P = [cam_to_world(d["xyz"]) for c in cams for d in current[c.serial] if d["xyz"] is not None]
                if P:
                    view.target = np.mean(P, axis=0)
                    view.dist = 600.0
            elif ch == "c":
                for t in trails.values():
                    t.clear()
            elif ch == "r" and writer is not None:
                saving = not saving
                print("CSV", "재개" if saving else "일시정지")
            elif ch == "v":
                if vrec is None:
                    vrec = VideoRec(out_dir / f"marker3d_{datetime.now():%Y%m%d_%H%M%S}.mp4", args.fps,
                                    (ui.shape[1], ui.shape[0]))
                    print("화면 녹화 시작")
                else:
                    print("화면 녹화 저장:", vrec.close())
                    vrec = None
            if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
            if args.duration and now - t_start > args.duration:
                break
    finally:
        if vrec is not None:
            print("화면 녹화 저장:", vrec.close())
        if csv_f is not None:
            csv_f.close()
            print(f"CSV 저장 완료: {csv_path}  ({n_rows} rows)")
        for c in cams:
            c.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
