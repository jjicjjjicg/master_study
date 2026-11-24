"""
연결된 모든 카메라 화면을 한 창에 격자로 띄우는 테스트 스크립트.

- Intel RealSense : pyrealsense2 로 시리얼별로 열어서 color (+ 옵션: depth 컬러맵) 표시
- 일반 USB/UVC 웹캠 : OpenCV(DirectShow)로 인덱스 0..N 을 탐색해서 열리는 것 모두 표시

실행 (cv2 + pyrealsense2 가 설치된 env, 예: realsense):
    conda activate realsense
    python view_all_cameras.py
    python view_all_cameras.py --depth          # RealSense depth 도 같이 표시
    python view_all_cameras.py --no-uvc         # RealSense 만
    python view_all_cameras.py --max-index 5    # UVC 탐색 범위

키:
    q / ESC : 종료
    s       : 현재 화면 전체 스냅샷 저장 (snapshots/ 폴더)
    d       : RealSense depth 표시 on/off (--depth 로 켰을 때만 의미 있음)
"""

import argparse
import os
import threading
import time
from datetime import datetime
from pathlib import Path

os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")   # 인덱스 탐색 시 WARN 스팸 숨김
import cv2
import numpy as np

TILE_W, TILE_H = 640, 360


# --------------------------------------------------------------------------- #
# 카메라 소스
# --------------------------------------------------------------------------- #
class CameraSource:
    """백그라운드 스레드에서 프레임을 계속 받아두는 공통 베이스."""

    def __init__(self, name):
        self.name = name
        self.lock = threading.Lock()
        self.frames = {}          # label -> BGR image
        self.fps = 0.0
        self.running = False
        self.error = None
        self.thread = None

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _grab(self):
        """{label: BGR image} 반환. 실패 시 None."""
        raise NotImplementedError

    def _loop(self):
        t_last, n = time.time(), 0
        while self.running:
            try:
                got = self._grab()
            except Exception as e:  # 장치 분리 등
                self.error = str(e)
                time.sleep(0.5)
                continue
            if got is None:
                time.sleep(0.01)
                continue
            with self.lock:
                self.frames = got
            n += 1
            now = time.time()
            if now - t_last >= 1.0:
                self.fps = n / (now - t_last)
                t_last, n = now, 0

    def get(self):
        with self.lock:
            return dict(self.frames), self.fps, self.error

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)
        self._close()

    def _close(self):
        pass


class UVCCamera(CameraSource):
    def __init__(self, index, cap):
        super().__init__(f"UVC #{index}")
        self.cap = cap

    def _grab(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return None
        if frame.ndim == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        return {"color": frame}

    def _close(self):
        self.cap.release()


class RealSenseCamera(CameraSource):
    def __init__(self, rs, serial, dev_name, use_depth, w=640, h=480, fps=30):
        super().__init__(f"{dev_name} [{serial}]")
        self.rs = rs
        self.use_depth = use_depth
        self.show_depth = use_depth
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
        if use_depth:
            cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
        self.pipe.start(cfg)
        self.colorizer = rs.colorizer() if use_depth else None

    def _grab(self):
        fs = self.pipe.wait_for_frames(2000)
        out = {}
        c = fs.get_color_frame()
        if c:
            out["color"] = np.asanyarray(c.get_data()).copy()
        if self.use_depth and self.show_depth:
            d = fs.get_depth_frame()
            if d:
                out["depth"] = np.asanyarray(self.colorizer.colorize(d).get_data()).copy()
        return out or None

    def _close(self):
        try:
            self.pipe.stop()
        except Exception:
            pass


# --------------------------------------------------------------------------- #
# 탐색
# --------------------------------------------------------------------------- #
def find_realsense(use_depth):
    try:
        import pyrealsense2 as rs
    except ImportError:
        print("[RealSense] pyrealsense2 없음 -> 건너뜀")
        return []
    cams = []
    for dev in rs.context().query_devices():
        serial = dev.get_info(rs.camera_info.serial_number)
        name = dev.get_info(rs.camera_info.name)
        try:
            cams.append(RealSenseCamera(rs, serial, name, use_depth))
            print(f"[RealSense] 열림: {name} ({serial})")
        except Exception as e:
            print(f"[RealSense] {name} ({serial}) 열기 실패: {e}")
    return cams


def dshow_names():
    """DirectShow 장치 이름 목록 (pygrabber 가 있으면). 인덱스 순서와 일치."""
    try:
        from pygrabber.dshow_graph import FilterGraph
        return FilterGraph().get_input_devices()
    except Exception:
        return None


def find_uvc(max_index, skip_realsense):
    names = dshow_names()
    cams = []
    for i in range(max_index):
        name = names[i] if names and i < len(names) else None
        if skip_realsense and name and "realsense" in name.lower():
            print(f"[UVC] #{i} '{name}' -> RealSense 라서 건너뜀")
            continue
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap.release()
            continue
        ok, frame = False, None
        for _ in range(5):           # 처음 몇 프레임은 비어 있을 수 있음
            ok, frame = cap.read()
            if ok and frame is not None:
                break
            time.sleep(0.05)
        if not ok:
            cap.release()
            continue
        cam = UVCCamera(i, cap)
        if name:
            cam.name = f"UVC #{i} {name}"
        h, w = frame.shape[:2]
        print(f"[UVC] 열림: {cam.name} ({w}x{h})")
        cams.append(cam)
    return cams


# --------------------------------------------------------------------------- #
# 화면 합성
# --------------------------------------------------------------------------- #
def make_tile(img, title, fps):
    if img is None:
        tile = np.zeros((TILE_H, TILE_W, 3), np.uint8)
        cv2.putText(tile, "NO SIGNAL", (TILE_W // 2 - 90, TILE_H // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
    else:
        h, w = img.shape[:2]
        s = min(TILE_W / w, TILE_H / h)
        r = cv2.resize(img, (int(w * s), int(h * s)))
        tile = np.zeros((TILE_H, TILE_W, 3), np.uint8)
        y0, x0 = (TILE_H - r.shape[0]) // 2, (TILE_W - r.shape[1]) // 2
        tile[y0:y0 + r.shape[0], x0:x0 + r.shape[1]] = r
        title = f"{title}  {w}x{h}"
    cv2.rectangle(tile, (0, 0), (TILE_W, 26), (0, 0, 0), -1)
    cv2.putText(tile, f"{title}  {fps:4.1f}fps", (6, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    return tile


def make_grid(tiles):
    n = len(tiles)
    cols = int(np.ceil(np.sqrt(n)))
    rows = int(np.ceil(n / cols))
    blank = np.zeros((TILE_H, TILE_W, 3), np.uint8)
    tiles = tiles + [blank] * (rows * cols - n)
    return np.vstack([np.hstack(tiles[r * cols:(r + 1) * cols]) for r in range(rows)])


# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--depth", action="store_true", help="RealSense depth 도 표시")
    ap.add_argument("--no-rs", action="store_true", help="RealSense 사용 안 함")
    ap.add_argument("--no-uvc", action="store_true", help="일반 웹캠 탐색 안 함")
    ap.add_argument("--max-index", type=int, default=10, help="UVC 인덱스 탐색 범위")
    ap.add_argument("--duration", type=float, default=0, help="N초 후 자동 종료 (0=무한)")
    args = ap.parse_args()

    cams = []
    if not args.no_rs:
        cams += find_realsense(args.depth)      # RealSense 를 먼저 잡아야 UVC 쪽 중복이 줄어듦
    if not args.no_uvc:
        cams += find_uvc(args.max_index, skip_realsense=not args.no_rs)

    if not cams:
        print("연결된 카메라를 찾지 못했습니다.")
        return
    print(f"\n총 {len(cams)}개 카메라. q/ESC 종료, s 스냅샷, d depth 토글")

    for c in cams:
        c.start()

    win = "All Cameras"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    snap_dir = Path(__file__).parent / "snapshots"
    t0 = time.time()
    try:
        while True:
            tiles = []
            for c in cams:
                frames, fps, err = c.get()
                if not frames:
                    tiles.append(make_tile(None, f"{c.name} {err or ''}", fps))
                    continue
                for label, img in frames.items():
                    t = c.name if label == "color" else f"{c.name} ({label})"
                    tiles.append(make_tile(img, t, fps))
            grid = make_grid(tiles)
            cv2.imshow(win, grid)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                snap_dir.mkdir(exist_ok=True)
                p = snap_dir / f"cams_{datetime.now():%Y%m%d_%H%M%S}.png"
                cv2.imencode(".png", grid)[1].tofile(str(p))   # 한글 경로 대응
                print("저장:", p)
            if key == ord("d"):
                for c in cams:
                    if isinstance(c, RealSenseCamera) and c.use_depth:
                        c.show_depth = not c.show_depth
            if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
            if args.duration and time.time() - t0 > args.duration:
                break
    finally:
        for c in cams:
            c.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
