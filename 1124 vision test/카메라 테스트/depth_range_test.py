"""
RealSense depth 안정 거리 테스트 + UI 화면 녹화 (여러 대 동시 지원).

손을 카메라에 가까이/멀리 움직이면서, ROI(사각형) 안의 depth 가
어느 거리 구간에서 안정적으로 나오는지 실시간으로 표시하고, UI 화면 그대로 mp4 로 녹화한다.
연결된 RealSense 가 여러 대면 카메라마다 한 줄씩 위아래로 쌓아서 동시에 보여준다.

카메라 한 줄 구성
    위   : 컬러 | depth 컬러맵  (둘 다 ROI 박스 표시, depth 는 컬러에 정렬됨)
    아래 : 거리 / 유효픽셀 비율 / 흔들림(std) 수치 + 최근 N초 그래프 + 안정 구간 기록

안정 판정 (기본값, 옵션으로 변경 가능)
    - ROI 유효 픽셀 비율(depth != 0) >= --min-valid (기본 0.90)
    - 최근 0.5초 동안 median depth 의 표준편차 <= max(--std-mm, --std-pct% x 거리)

실행:
    conda activate realsense
    python depth_range_test.py --auto                       # 연결된 RealSense 전부
    python depth_range_test.py --serial 745412070763        # 특정 카메라만
    python depth_range_test.py --serial 213622301272 745412070763

키:
    r          : 녹화 시작/정지  (recordings/ 에 mp4 + 프레임별 수치 csv 저장)
    a          : ROI 자동추적(가장 가까운 물체 = 손) on/off  (모든 카메라)
    마우스 클릭 : 클릭한 카메라의 ROI 위치 지정 (그 카메라만 자동추적 꺼짐)
    + / -      : ROI 크기 조절
    c          : 안정 구간 기록 초기화
    s          : 스냅샷 png
    q / ESC    : 종료
"""

import argparse
import csv
import os
import shutil
import tempfile
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")
import cv2
import numpy as np
import pyrealsense2 as rs

HERE = Path(__file__).parent
FONT = cv2.FONT_HERSHEY_SIMPLEX
GREEN, YELLOW, RED, WHITE, GRAY = (80, 220, 80), (0, 220, 255), (60, 60, 255), (255, 255, 255), (130, 130, 130)
PANEL_H = 230
BAR_H = 28


# --------------------------------------------------------------------------- #
def put(img, text, org, scale=0.6, color=WHITE, thick=1):
    cv2.putText(img, text, org, FONT, scale, (0, 0, 0), thick + 3, cv2.LINE_AA)
    cv2.putText(img, text, org, FONT, scale, color, thick, cv2.LINE_AA)


class Recorder:
    """한글 경로에서 cv2.VideoWriter 가 실패하는 문제 때문에 임시폴더(ASCII)에 쓰고 끝나면 옮김."""

    def __init__(self, out_dir, fps, size):
        out_dir.mkdir(parents=True, exist_ok=True)
        stamp = f"{datetime.now():%Y%m%d_%H%M%S}"
        self.final_mp4 = out_dir / f"depth_test_{stamp}.mp4"
        self.tmp_mp4 = Path(tempfile.gettempdir()) / f"depth_test_{stamp}.mp4"
        self.fps = fps
        self.writer = cv2.VideoWriter(str(self.tmp_mp4), cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
        if not self.writer.isOpened():
            raise RuntimeError("VideoWriter 열기 실패")
        self.csv_path = out_dir / f"depth_test_{stamp}.csv"
        self.csv_f = open(self.csv_path, "w", newline="", encoding="utf-8-sig")
        self.csv = csv.writer(self.csv_f)
        self.csv.writerow(["t_s", "camera", "serial", "roi_x", "roi_y", "median_mm", "valid_ratio",
                           "spatial_std_mm", "temporal_std_mm", "stable"])
        self.t0 = time.time()
        self.n_written = 0

    def write(self, frame, rows):
        # 실제 경과 시간에 맞춰 프레임 수를 맞춤 (렌더링이 느려도 영상 속도가 실제와 같게)
        target = int((time.time() - self.t0) * self.fps) + 1
        while self.n_written < target:
            self.writer.write(frame)
            self.n_written += 1
        t = f"{time.time() - self.t0:.3f}"
        for r in rows:
            self.csv.writerow([t] + r)

    def close(self):
        self.writer.release()
        self.csv_f.close()
        shutil.move(str(self.tmp_mp4), str(self.final_mp4))
        return self.final_mp4, self.csv_path


def draw_graph(w, h, hist, window_s, max_mm, stable_thr_valid):
    """최근 window_s 초의 median depth(선) + 유효비율(아래 막대)."""
    g = np.full((h, w, 3), 25, np.uint8)
    for mm in range(0, int(max_mm) + 1, 250):
        y = int(h - 1 - mm / max_mm * (h - 1))
        cv2.line(g, (0, y), (w, y), (50, 50, 50), 1)
        if mm % 500 == 0:
            put(g, f"{mm / 10:.0f}cm", (4, max(y - 3, 12)), 0.4, GRAY)
    put(g, f"median depth (last {window_s:.0f}s)  green=stable red=unstable", (60, 16), 0.45, GRAY)
    if not hist:
        return g
    t_now = hist[-1][0]
    pts = []
    for t, d, v, st in hist:
        x = int(w - 1 - (t_now - t) / window_s * (w - 1))
        if x < 0:
            continue
        bar = int(v * 20)
        cv2.line(g, (x, h - 1), (x, h - 1 - bar), (90, 90, 90) if v >= stable_thr_valid else (60, 60, 160), 1)
        if d > 0:
            y = int(h - 1 - min(d, max_mm) / max_mm * (h - 1))
            pts.append((x, y, st))
    for (x0, y0, _), (x1, y1, s1) in zip(pts, pts[1:]):
        cv2.line(g, (x0, y0), (x1, y1), GREEN if s1 else RED, 2, cv2.LINE_AA)
    return g


# --------------------------------------------------------------------------- #
class DepthCam:
    """RealSense 한 대: 백그라운드 스레드로 정렬된 프레임 수신 + ROI 통계 + 화면 블록 렌더링."""

    def __init__(self, serial, W, H, fps, auto):
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
        self.align = rs.align(rs.stream.color)

        self.lock = threading.Lock()
        self.latest = None          # (color, depth_mm)
        self.seq = 0
        self.fps_val = 0.0
        self.error = None
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

        # ROI / 통계 상태
        self.cx, self.cy, self.auto = W // 2, H // 2, auto
        self.last_seq = -1
        self.temporal = deque()     # (t, median)
        self.hist = deque()         # (t, median, valid, stable)
        self.stable_min = self.stable_max = None
        self.m = dict(median=0.0, valid=0.0, sstd=0.0, tstd=0.0, thr=0.0, stable=False)

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
            depth = np.asanyarray(df.get_data()).astype(np.float32) * (self.depth_scale * 1000.0)
            with self.lock:
                self.latest = (color, depth)
                self.seq += 1
            self.error = None
            n += 1
            if time.time() - t0 >= 1.0:
                self.fps_val, t0, n = n / (time.time() - t0), time.time(), 0

    def stop(self):
        self.running = False
        self.thread.join(timeout=3)
        try:
            self.pipe.stop()
        except Exception:
            pass

    # ---- ROI 자동추적: 중앙 80% 영역에서 충분히 큰 덩어리 중 가장 가까운 지점 ----
    def _track_nearest(self, depth_mm):
        W, H = self.W, self.H
        small = cv2.resize(depth_mm, (W // 8, H // 8), interpolation=cv2.INTER_NEAREST)
        small = cv2.medianBlur(small.astype(np.uint16), 5).astype(np.float32)
        solid = cv2.erode((small > 0).astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
        small = cv2.erode(small, np.ones((3, 3), np.uint8))
        small[~solid] = 1e9
        mh, mw = small.shape
        m = np.full_like(small, 1e9)
        m[mh // 10: mh * 9 // 10, mw // 10: mw * 9 // 10] = small[mh // 10: mh * 9 // 10, mw // 10: mw * 9 // 10]
        iy, ix = np.unravel_index(np.argmin(m), m.shape)
        if m[iy, ix] < 1e8:
            self.cx = int(0.7 * self.cx + 0.3 * (ix * 8 + 4))
            self.cy = int(0.7 * self.cy + 0.3 * (iy * 8 + 4))

    def update(self, args, roi):
        """새 프레임이 들어왔으면 통계 갱신. 반환: (color, depth_mm) 또는 None."""
        with self.lock:
            latest, seq = self.latest, self.seq
        if latest is None:
            return None
        if seq == self.last_seq:
            return latest
        self.last_seq = seq
        color, depth_mm = latest
        now = time.time()
        if self.auto:
            self._track_nearest(depth_mm)

        half = roi // 2
        self.cx = int(np.clip(self.cx, half, self.W - half - 1))
        self.cy = int(np.clip(self.cy, half, self.H - half - 1))
        patch = depth_mm[self.cy - half: self.cy + half, self.cx - half: self.cx + half]
        valid = patch[patch > 0]
        valid_ratio = valid.size / patch.size
        median = float(np.median(valid)) if valid.size else 0.0
        sstd = float(np.std(valid)) if valid.size else 0.0

        self.temporal.append((now, median))
        while self.temporal and now - self.temporal[0][0] > 0.5:
            self.temporal.popleft()
        tv = [d for _, d in self.temporal if d > 0]
        tstd = float(np.std(tv)) if len(tv) >= 3 else 0.0
        thr = max(args.std_mm, args.std_pct / 100.0 * median)
        stable = valid_ratio >= args.min_valid and median > 0 and len(tv) >= 3 and tstd <= thr
        if stable:
            self.stable_min = median if self.stable_min is None else min(self.stable_min, median)
            self.stable_max = median if self.stable_max is None else max(self.stable_max, median)

        self.hist.append((now, median, valid_ratio, stable))
        while self.hist and now - self.hist[0][0] > args.graph_sec:
            self.hist.popleft()
        self.m = dict(median=median, valid=valid_ratio, sstd=sstd, tstd=tstd, thr=thr, stable=stable)
        return latest

    def csv_row(self):
        m = self.m
        return [self.model, self.serial, self.cx, self.cy, f"{m['median']:.1f}", f"{m['valid']:.3f}",
                f"{m['sstd']:.2f}", f"{m['tstd']:.2f}", int(m["stable"])]

    def render(self, frames, args, roi):
        W, H = self.W, self.H
        PW = 2 * W
        if frames is None:
            block = np.zeros((H + PANEL_H, PW, 3), np.uint8)
            put(block, f"{self.model} [{self.serial}]  waiting... {self.error or ''}", (20, H // 2), 0.8, RED, 2)
            return block
        color, depth_mm = frames
        m = self.m
        median, valid_ratio, stable = m["median"], m["valid"], m["stable"]
        st_col = GREEN if stable else (YELLOW if valid_ratio >= 0.5 else RED)
        half = roi // 2
        cx, cy = self.cx, self.cy

        dvis = np.clip(depth_mm / args.max_mm * 255, 0, 255).astype(np.uint8)
        dvis = cv2.applyColorMap(255 - dvis, cv2.COLORMAP_JET)
        dvis[depth_mm == 0] = 0
        cview = color.copy()
        for img in (cview, dvis):
            cv2.rectangle(img, (cx - half, cy - half), (cx + half, cy + half), st_col, 2)
            cv2.drawMarker(img, (cx, cy), st_col, cv2.MARKER_CROSS, 12, 1)
            label = f"{median / 10:.1f} cm" if median > 0 else "NO DEPTH"
            put(img, label, (min(cx + half + 6, W - 150), max(cy - half + 18, 20)), 0.8, st_col, 2)
        put(cview, f"{self.model} [{self.serial}]  COLOR  {self.fps_val:4.1f}fps", (10, 24), 0.6)
        put(dvis, "DEPTH (aligned, black = no data)", (10, 24), 0.6)
        put(cview, "ROI AUTO" if self.auto else "ROI MANUAL", (10, H - 12), 0.5, GRAY)
        top = np.hstack([cview, dvis])

        panel = np.full((PANEL_H, PW, 3), 15, np.uint8)
        put(panel, self.model, (14, 26), 0.6, GRAY)
        big = f"{median / 10:6.1f} cm" if median > 0 else "  NO DEPTH"
        put(panel, big, (14, 72), 1.6, st_col, 3)
        put(panel, "STABLE" if stable else "UNSTABLE", (14, 106), 0.8, st_col, 2)
        put(panel, f"valid pixels : {valid_ratio * 100:5.1f} %   (>= {args.min_valid * 100:.0f}%)", (14, 134), 0.55,
            WHITE if valid_ratio >= args.min_valid else RED)
        put(panel, f"temporal std : {m['tstd']:5.1f} mm  (<= {m['thr']:.1f})", (14, 158), 0.55,
            WHITE if m["tstd"] <= m["thr"] else RED)
        put(panel, f"spatial std  : {m['sstd']:5.1f} mm   ROI {roi}px", (14, 182), 0.55)
        rng = (f"{self.stable_min / 10:.1f} ~ {self.stable_max / 10:.1f} cm"
               if self.stable_min is not None else "-")
        put(panel, f"stable range seen : {rng}", (14, 212), 0.6, GREEN)
        gx = 430
        panel[8:PANEL_H - 8, gx:PW - 10] = draw_graph(PW - 10 - gx, PANEL_H - 16, list(self.hist),
                                                      args.graph_sec, args.max_mm, args.min_valid)
        block = np.vstack([top, panel])
        cv2.line(block, (0, block.shape[0] - 1), (PW, block.shape[0] - 1), (80, 80, 80), 2)
        return block


# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--serial", nargs="*", default=None, help="사용할 RealSense 시리얼 (생략 시 연결된 전부)")
    ap.add_argument("--res", default="640x480", help="해상도 WxH (640x480, 848x480, 1280x720)")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--roi", type=int, default=40, help="ROI 한 변 픽셀")
    ap.add_argument("--min-valid", type=float, default=0.90, help="안정 판정: 최소 유효픽셀 비율")
    ap.add_argument("--std-mm", type=float, default=3.0, help="안정 판정: 허용 temporal std 하한(mm)")
    ap.add_argument("--std-pct", type=float, default=1.0, help="안정 판정: 허용 temporal std (거리의 %%)")
    ap.add_argument("--graph-sec", type=float, default=10.0, help="그래프 시간 범위(초)")
    ap.add_argument("--max-mm", type=float, default=1500, help="그래프/컬러맵 최대 거리(mm)")
    ap.add_argument("--auto", action="store_true", help="시작부터 ROI 자동추적(가장 가까운 물체)")
    ap.add_argument("--win-h", type=int, default=950, help="처음 창 높이(px). 녹화는 원본 해상도로 저장")
    ap.add_argument("--duration", type=float, default=0, help="N초 후 자동 종료(테스트용)")
    ap.add_argument("--out-dir", default=str(HERE / "recordings"), help="녹화 저장 폴더")
    ap.add_argument("--record", action="store_true", help="시작하자마자 녹화")
    args = ap.parse_args()
    W, H = map(int, args.res.lower().split("x"))

    found = [(d.get_info(rs.camera_info.name), d.get_info(rs.camera_info.serial_number))
             for d in rs.context().query_devices()]
    print("연결된 RealSense:", ", ".join(f"{n} ({s})" for n, s in found) or "없음")
    serials = args.serial if args.serial else [s for _, s in found]
    cams = []
    for s in serials:
        try:
            cams.append(DepthCam(s, W, H, args.fps, args.auto))
            print(f"열림: {cams[-1].model} ({s})  {W}x{H}@{args.fps}")
        except Exception as e:
            print(f"열기 실패: {s}: {e}")
    if not cams:
        print("사용할 수 있는 RealSense 가 없습니다.")
        return

    roi = args.roi
    rec = None
    block_h = H + PANEL_H
    total_h = BAR_H + block_h * len(cams)

    win = "Depth range test"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    if total_h > args.win_h:
        cv2.resizeWindow(win, int(2 * W * args.win_h / total_h), args.win_h)

    def on_mouse(ev, x, y, *_):
        if ev != cv2.EVENT_LBUTTONDOWN or y < BAR_H:
            return
        i, yy = divmod(y - BAR_H, block_h)
        if i < len(cams) and yy < H:
            c = cams[i]
            c.cx, c.cy, c.auto = x % W, yy, False   # 컬러/깊이 어느 쪽을 클릭해도 동일 좌표
    cv2.setMouseCallback(win, on_mouse)

    t_start = time.time()
    try:
        while True:
            blocks = [c.render(c.update(args, roi), args, roi) for c in cams]
            bar = np.full((BAR_H, 2 * W, 3), 40, np.uint8)
            put(bar, f"Depth range test  {len(cams)} camera(s)  {W}x{H}", (8, 19), 0.5, GRAY)
            put(bar, "[r]rec [a]auto [click]ROI [+/-]size [c]clear [q]quit", (2 * W - 440, 19), 0.45, GRAY)
            ui = np.vstack([bar] + blocks)

            if args.record and rec is None:
                rec = Recorder(Path(args.out_dir), args.fps, (ui.shape[1], ui.shape[0]))
                print("녹화 시작")
                args.record = False
            if rec is not None:
                rec.write(ui, [c.csv_row() for c in cams])
                el = time.time() - rec.t0
                cv2.circle(ui, (2 * W - 520, 14), 8, (0, 0, 255), -1)    # REC 표시는 화면에만
                put(ui, f"REC {int(el // 60):02d}:{int(el % 60):02d}", (2 * W - 505, 20), 0.55, (0, 0, 255), 2)

            cv2.imshow(win, ui)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            elif key == ord("r"):
                if rec is None:
                    rec = Recorder(Path(args.out_dir), args.fps, (ui.shape[1], ui.shape[0]))
                    print("녹화 시작")
                else:
                    mp4, csvp = rec.close()
                    rec = None
                    print(f"녹화 저장: {mp4}\n          {csvp}")
            elif key == ord("a"):
                new = not all(c.auto for c in cams)
                for c in cams:
                    c.auto = new
            elif key in (ord("+"), ord("=")):
                roi = min(roi + 10, min(W, H) - 2)
            elif key in (ord("-"), ord("_")):
                roi = max(roi - 10, 10)
            elif key == ord("c"):
                for c in cams:
                    c.stable_min = c.stable_max = None
            elif key == ord("s"):
                d = HERE / "snapshots"
                d.mkdir(exist_ok=True)
                p = d / f"depth_{datetime.now():%Y%m%d_%H%M%S}.png"
                cv2.imencode(".png", ui)[1].tofile(str(p))
                print("저장:", p)
            if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
            if args.duration and time.time() - t_start > args.duration:
                break
    finally:
        if rec is not None:
            mp4, csvp = rec.close()
            print(f"녹화 저장: {mp4}\n          {csvp}")
        for c in cams:
            c.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
