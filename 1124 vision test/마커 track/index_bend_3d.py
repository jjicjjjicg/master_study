"""
index_bend_3d.py — 검지에 붙인 마커 3개로 손가락 모양을 3D 로 보여주고 굽힘 각도를 실시간 저장.

마커 (hsv_<serial>.json 의 'index', count 3) 3개를
    P (손끝에서 먼 쪽, proximal) — M (가운데) — D (손끝 쪽, distal)
로 정렬해서 3D 에 손가락 뼈대처럼 선으로 잇고,

    굽힘 각도 bend = 180° - (P-M-D 가 M 에서 이루는 각)
        0° = 쭉 편 상태, 커질수록 많이 굽힘

을 계산한다. 각도는 좌표계와 상관없으므로 카메라끼리 캘리브레이션 없이 바로 비교/합칠 수 있다
(fused = 두 카메라 값이 비슷하면 평균, 많이 다르면 마커가 더 크게 보이는 카메라 값).

굽힘 정도(curl, 0~1):  O 키 = 편 손 기록(2초), F 키 = 최대로 굽힌 손 기록(2초)
    curl = (bend - open) / (fist - open)      → index_rom.json 에 저장, 다음 실행 때 자동 로드
    ※ CSV 에 마커 3D 원본 좌표가 다 저장되므로, 나중에 영상/데이터 보고 정의를 바꿔도 다시 계산 가능

실행:
    conda activate realsense
    python index_bend_3d.py --res 1280x720
    python index_bend_3d.py --res 1280x720 --zmin 250 --zmax 800     # 손이 있는 거리만 (배경 오검출 제거)

저장 (data/):
    index_bend_<시간>.csv : 카메라 프레임마다 한 줄
        P/M/D 의 X,Y,Z(mm, 각 카메라 좌표계), 마디 길이, bend(원본), bend_smooth, fused_bend, curl
    V 키: 화면 녹화 index_bend_<시간>.mp4

키:
    O : 편 손 기록 (2초)       F : 굽힌 손 기록 (2초)       P : P/D (손가락 시작/끝) 방향 뒤집기
    V : 화면 녹화              R : CSV 일시정지/재개        C : 궤적/그래프 지우기
    3D 뷰: 왼쪽 드래그 회전, 오른쪽 드래그 이동, 휠 확대, 1/2/3/0 시점, G 손가락으로 중심 맞추기
    Q / ESC : 종료
"""

import argparse
import csv
import json
import math
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np

from marker_common import HERE, RSCamera, cv2, detect, list_devices, load_hsv, put, range_bgr
from marker_track_3d import IdAssigner, VideoRec, View3D, cam_to_world, draw_line3d, render_3d

BAR_H = 30
TILE_W = 640
SIDE_W = 400
GRAPH_H = 210
SEG_MIN, SEG_MAX = 8.0, 90.0       # 마커 사이 거리(mm)가 이 범위를 벗어나면 잘못 잡힌 것으로 보고 버림
FUSE_DIFF = 15.0                   # 두 카메라 bend 차이가 이 이하면 평균
CAM_COLS = [(0, 170, 255), (255, 200, 0), (200, 120, 255), (120, 255, 120)]
ROM_PATH = HERE / "index_rom.json"


def angle_at(a, b, c):
    v1, v2 = a - b, c - b
    n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
    if n1 < 1e-6 or n2 < 1e-6:
        return float("nan")
    return float(np.degrees(np.arccos(np.clip(np.dot(v1, v2) / (n1 * n2), -1, 1))))


def order_finger(dets, prev_prox):
    """3D 가 있는 마커 3개 -> [P, M, D] 순서.
    가운데(M) = 가장 먼 두 점(양 끝)이 아닌 나머지 하나.
    P/D = 직전 프레임 P 위치에 가까운 끝이 P (처음엔 화면 아래쪽 끝을 P 로 가정, P 키로 뒤집기)."""
    X = [np.asarray(d["xyz"], float) for d in dets]
    pairs = [(np.linalg.norm(X[i] - X[j]), i, j) for i, j in ((0, 1), (0, 2), (1, 2))]
    _, i, j = max(pairs)
    k = 3 - i - j
    if prev_prox is not None:
        p = i if np.linalg.norm(X[i] - prev_prox) <= np.linalg.norm(X[j] - prev_prox) else j
    else:
        p = i if dets[i]["v"] >= dets[j]["v"] else j
    d = j if p == i else i
    return [dets[p], dets[k], dets[d]]


def draw_graph(w, h, hist, cams, window_s, now, rom):
    g = np.full((h, w, 3), 20, np.uint8)
    top = 120.0
    for deg in range(0, int(top) + 1, 30):
        y = int(h - 10 - deg / top * (h - 30))
        cv2.line(g, (40, y), (w, y), (50, 50, 50), 1)
        put(g, f"{deg}", (6, y + 4), 0.4, (130, 130, 130))
    for key, col in (("open", (120, 120, 120)), ("fist", (120, 120, 120))):
        if rom.get(key) is not None:
            y = int(h - 10 - min(rom[key], top) / top * (h - 30))
            cv2.line(g, (40, y), (w, y), col, 1, cv2.LINE_4)
            put(g, key, (w - 40, y - 3), 0.4, col)
    put(g, f"bend (deg), last {window_s:.0f}s   thick white = fused", (46, 16), 0.45, (150, 150, 150))
    series = [(ci, CAM_COLS[ci % len(CAM_COLS)], 1) for ci in range(len(cams))] + [("f", (255, 255, 255), 2)]
    for key, col, th in series:
        prev = None
        for t, vals, fused in hist:
            x = int(w - 1 - (now - t) / window_s * (w - 41))
            v = fused if key == "f" else vals.get(key)
            if x < 40 or v is None:
                prev = None
                continue
            y = int(h - 10 - min(max(v, 0), top) / top * (h - 30))
            if prev is not None:
                cv2.line(g, prev, (x, y), col, th, cv2.LINE_AA)
            prev = (x, y)
    return g


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--serial", nargs="*")
    ap.add_argument("--res", default="1280x720")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--marker", default="index", help="hsv json 안의 마커 이름")
    ap.add_argument("--zmin", type=float, default=150, help="이보다 가까운 blob 무시 (mm)")
    ap.add_argument("--zmax", type=float, default=1000, help="이보다 먼 blob 무시 (mm)")
    ap.add_argument("--hsv-dir", default=str(HERE))
    ap.add_argument("--out-dir", default=str(HERE / "data"))
    ap.add_argument("--no-save", action="store_true")
    ap.add_argument("--graph-sec", type=float, default=15.0)
    ap.add_argument("--win-h", type=int, default=980)
    ap.add_argument("--duration", type=float, default=0)
    ap.add_argument("--record", action="store_true")
    args = ap.parse_args()
    W, H = map(int, args.res.lower().split("x"))
    TH = int(TILE_W * H / W)
    sc = TILE_W / W

    devs = list_devices()
    print("연결된 RealSense:", ", ".join(f"{n} ({s})" for n, s in devs) or "없음")
    serials = args.serial or [s for _, s in devs]

    cams, conf = [], {}
    for s in serials:
        rng = load_hsv(s, args.hsv_dir).get(args.marker)
        conf[s] = dict(rng=rng, ids=IdAssigner(3), queue=deque(maxlen=600))
        if not rng:
            print(f"  경고: hsv_{s}.json 에 '{args.marker}' 없음 → 이 카메라는 영상만 (marker_hsv_tune.py 로 먼저 등록)")

        def processor(cam, item, c=conf[s]):
            if not c["rng"]:
                c["queue"].append((item["t"], item["frame_no"], []))
                return []
            dets, _ = detect(cam, item, c["rng"], 8)            # 넉넉히 찾고
            dets = [d for d in dets if d["xyz"] is not None and args.zmin <= d["xyz"][2] <= args.zmax][:3]
            dets = c["ids"].assign(dets, item["t"])            # 거리 범위 안에서 큰 순 3개
            for d in dets:
                d["name"] = args.marker
            c["queue"].append((item["t"], item["frame_no"], dets))
            return dets

        try:
            cams.append(RSCamera(s, W, H, args.fps, processor=processor))
            print("열림:", cams[-1].label)
        except Exception as e:
            print(f"열기 실패 {s}: {e}  (USB 2 로 잡혔으면 USB 3 포트에 다시 꽂기)")
    if not cams:
        return

    rom = json.loads(ROM_PATH.read_text(encoding="utf-8")) if ROM_PATH.is_file() else {}
    if rom:
        print(f"ROM 로드: open {rom.get('open')}  fist {rom.get('fist')}")

    def curl_of(b):
        if b is None or rom.get("open") is None or rom.get("fist") is None or abs(rom["fist"] - rom["open"]) < 5:
            return None
        return (b - rom["open"]) / (rom["fist"] - rom["open"])

    # ---- 저장 ----
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = f"{datetime.now():%Y%m%d_%H%M%S}"
    csv_f = writer = None
    saving = not args.no_save
    if saving:
        csv_path = out_dir / f"index_bend_{stamp}.csv"
        csv_f = open(csv_path, "w", newline="", encoding="utf-8-sig")
        writer = csv.writer(csv_f)
        writer.writerow(["wall_time", "t_s", "camera", "serial", "frame_no", "n_markers",
                         "P_x", "P_y", "P_z", "M_x", "M_y", "M_z", "D_x", "D_y", "D_z",
                         "seg_PM_mm", "seg_MD_mm", "bend_deg", "bend_smooth_deg", "fused_bend_deg", "curl"])
        print("CSV 저장:", csv_path)
    n_rows, t_flush = 0, time.time()
    vrec = None

    st = {c.serial: dict(prox=None, smooth=deque(maxlen=5), res=None, t=0.0, quality=0) for c in cams}
    trails = {c.serial: {} for c in cams}
    current = {c.serial: [] for c in cams}
    hist = deque()
    capture = None
    fused = None
    view = View3D()
    drag = dict(btn=None, x=0, y=0)
    row_w = TILE_W * 2 + SIDE_W

    def on_mouse(ev, x, y, flags, *_):
        if ev in (cv2.EVENT_LBUTTONDOWN, cv2.EVENT_RBUTTONDOWN):
            if TILE_W <= x < 2 * TILE_W and BAR_H <= y < BAR_H + TH * len(cams):
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
                s_ = view.dist / (0.9 * TILE_W)
                view.target = view.target - right * dx * s_ + up * dy * s_
        elif ev == cv2.EVENT_MOUSEWHEEL:
            view.dist = float(np.clip(view.dist * (0.9 if cv2.getMouseWheelDelta(flags) > 0 else 1.1), 100, 6000))

    win = "index bend 3D"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    total_h = BAR_H + TH * len(cams) + GRAPH_H
    if total_h > args.win_h:
        cv2.resizeWindow(win, int(row_w * args.win_h / total_h), args.win_h)
    cv2.setMouseCallback(win, on_mouse)

    t_start = time.time()
    try:
        while True:
            now = time.time()
            # ---- 카메라 결과 처리 ----
            for c in cams:
                s = st[c.serial]
                q = conf[c.serial]["queue"]
                while q:
                    t, fno, dets = q.popleft()
                    current[c.serial] = dets
                    for d in dets:
                        tr = trails[c.serial].setdefault((d["name"], d["id"]), deque(maxlen=200))
                        tr.append((t, cam_to_world(d["xyz"])))
                    res = None
                    if len(dets) == 3:
                        o = order_finger(dets, s["prox"])
                        X = [np.asarray(d["xyz"], float) for d in o]
                        seg = (float(np.linalg.norm(X[1] - X[0])), float(np.linalg.norm(X[2] - X[1])))
                        if all(SEG_MIN <= v <= SEG_MAX for v in seg):
                            s["prox"] = X[0]
                            bend = 180.0 - angle_at(X[0], X[1], X[2])
                            s["smooth"].append(bend)
                            res = dict(order=o, X=X, seg=seg, bend=bend,
                                       bend_s=float(np.median(s["smooth"])),
                                       quality=min(d["area"] for d in o))
                    s["res"], s["t"] = res, t
                    if saving:
                        row = [f"{t:.4f}", f"{t - t_start:.4f}", c.model, c.serial, fno, len(dets)]
                        if res:
                            row += [f"{v:.1f}" for X_ in res["X"] for v in X_]
                            row += [f"{res['seg'][0]:.1f}", f"{res['seg'][1]:.1f}", f"{res['bend']:.2f}",
                                    f"{res['bend_s']:.2f}"]
                        else:
                            row += [""] * 13
                        cu = curl_of(fused)
                        row += ["" if fused is None else f"{fused:.2f}", "" if cu is None else f"{cu:.3f}"]
                        writer.writerow(row)
                        n_rows += 1

            # ---- 두 카메라 합치기 ----
            vals, qual = {}, {}
            for ci, c in enumerate(cams):
                r = st[c.serial]["res"]
                if r is not None and now - st[c.serial]["t"] < 0.25:
                    vals[ci], qual[ci] = r["bend_s"], r["quality"]
            if not vals:
                fused = None
            elif len(vals) == 1 or max(vals.values()) - min(vals.values()) <= FUSE_DIFF:
                fused = float(np.mean(list(vals.values())))
            else:
                fused = vals[max(qual, key=qual.get)]
            hist.append((now, vals, fused))
            while hist and now - hist[0][0] > args.graph_sec:
                hist.popleft()

            if capture is not None:
                if fused is not None:
                    capture["v"].append(fused)
                if now >= capture["t_end"]:
                    if len(capture["v"]) >= 10:
                        rom[capture["kind"]] = round(float(np.median(capture["v"])), 2)
                        rom["date"] = time.strftime("%Y-%m-%d %H:%M:%S")
                        ROM_PATH.write_text(json.dumps(rom, indent=2), encoding="utf-8")
                        print(f"ROM 저장 ({capture['kind']}): {rom[capture['kind']]}°  → {ROM_PATH.name}")
                    else:
                        print("ROM 기록 실패: 손가락 3 마커가 충분히 안 보였음")
                    capture = None

            # ---- 화면 ----
            rows = []
            for ci, c in enumerate(cams):
                ccol = CAM_COLS[ci % len(CAM_COLS)]
                it = c.get()
                img = cv2.resize(it["color"], (TILE_W, TH)) if it is not None else np.zeros((TH, TILE_W, 3), np.uint8)
                dets = current[c.serial]
                r = st[c.serial]["res"]
                mcol = range_bgr(conf[c.serial]["rng"])
                for d in dets:
                    cv2.circle(img, (int(d["u"] * sc), int(d["v"] * sc)), 7, mcol, 2)
                if r is not None:
                    pts = [(int(d["u"] * sc), int(d["v"] * sc)) for d in r["order"]]
                    cv2.polylines(img, [np.array(pts)], False, (255, 255, 255), 2, cv2.LINE_AA)
                    for lab, p in zip("PMD", pts):
                        put(img, lab, (p[0] + 8, p[1] + 5), 0.5, (255, 255, 255), 1)
                    put(img, f"{r['bend_s']:.0f} deg", (pts[1][0] + 12, pts[1][1] - 12), 0.7, ccol, 2)
                cv2.rectangle(img, (0, 0), (TILE_W, 22), (0, 0, 0), -1)
                put(img, f"{c.label}  {c.fps_val:4.1f}fps", (6, 16), 0.5, ccol)
                if not conf[c.serial]["rng"]:
                    put(img, f"no '{args.marker}' in hsv_{c.serial}.json", (10, TH // 2), 0.6, (0, 0, 255), 2)

                v3 = render_3d(view, TILE_W, TH, c.model, dets, trails[c.serial], {args.marker: mcol}, 1.5, now)
                if r is not None:
                    Wp = [cam_to_world(X) for X in r["X"]]
                    draw_line3d(v3, view, Wp[0], Wp[1], (255, 255, 255), 3)
                    draw_line3d(v3, view, Wp[1], Wp[2], (255, 255, 255), 3)
                    for lab, Pw in zip("PMD", Wp):
                        uv, z = view.project(np.array([Pw]), TILE_W, TH)
                        if z[0] > 1:
                            put(v3, lab, (int(uv[0][0]) - 4, int(uv[0][1]) - 12), 0.5, (255, 255, 255))

                side = np.full((TH, SIDE_W, 3), 15, np.uint8)
                put(side, c.model, (14, 30), 0.7, ccol, 2)
                put(side, f"markers: {len(dets)}/3", (14, 58), 0.55,
                    (80, 220, 80) if len(dets) == 3 else (60, 60, 255))
                if r is not None:
                    put(side, f"{r['bend_s']:5.1f} deg", (14, 120), 1.5, ccol, 3)
                    put(side, f"raw {r['bend']:5.1f}", (14, 150), 0.5, (150, 150, 150))
                    put(side, f"P-M {r['seg'][0]:4.0f} mm   M-D {r['seg'][1]:4.0f} mm", (14, 178), 0.5)
                    put(side, f"M at Z = {r['X'][1][2]:.0f} mm", (14, 202), 0.5)
                else:
                    put(side, "  ---", (14, 120), 1.5, (90, 90, 90), 3)
                    put(side, "need 3 markers with depth" if len(dets) < 3 else "marker spacing out of range",
                        (14, 150), 0.5, (60, 60, 255))
                row = np.hstack([img, v3, side])
                cv2.line(row, (TILE_W, 0), (TILE_W, TH), (90, 90, 90), 1)
                cv2.line(row, (2 * TILE_W, 0), (2 * TILE_W, TH), (90, 90, 90), 1)
                cv2.line(row, (0, TH - 1), (row_w, TH - 1), (90, 90, 90), 2)
                rows.append(row)

            # 아래: fused 값 + 그래프
            gp = np.full((GRAPH_H, row_w, 3), 15, np.uint8)
            cu = curl_of(fused)
            put(gp, "FUSED", (14, 30), 0.6, (200, 200, 200))
            put(gp, f"{fused:5.1f} deg" if fused is not None else "  ---", (14, 80), 1.4, (255, 255, 255), 3)
            if cu is not None:
                put(gp, f"curl {cu:4.2f}", (14, 122), 0.9, (80, 220, 80), 2)
                bw = 200
                cv2.rectangle(gp, (14, 135), (14 + bw, 150), (80, 80, 80), 1)
                cv2.rectangle(gp, (14, 135), (14 + int(bw * float(np.clip(cu, 0, 1))), 150), (80, 220, 80), -1)
            else:
                put(gp, "curl: press O (open) / F (fist)", (14, 122), 0.5, (150, 150, 150))
            put(gp, f"open {rom.get('open', '-')}   fist {rom.get('fist', '-')}", (14, 178), 0.5, (150, 150, 150))
            if capture is not None:
                put(gp, f"RECORDING {capture['kind'].upper()}  {capture['t_end'] - now:.1f}s", (14, 200), 0.55,
                    (0, 200, 255), 2)
            gp[0:GRAPH_H, 300:row_w] = draw_graph(row_w - 300, GRAPH_H, hist, cams, args.graph_sec, now, rom)

            bar = np.full((BAR_H, row_w, 3), 40, np.uint8)
            stx = f"CSV {'ON' if saving else 'PAUSED'} {n_rows} rows" if writer else "CSV OFF"
            put(bar, f"index bend 3D  |  {stx}  |  Z {args.zmin:.0f}-{args.zmax:.0f}mm", (8, 20), 0.5, (170, 170, 170))
            put(bar, "O open  F fist  P flip  V video  R csv  C clear  G fit  1/2/3/0 view  Q quit",
                (row_w - 640, 20), 0.45, (150, 150, 150))
            ui = np.vstack([bar] + rows + [gp])

            if args.record and vrec is None:
                vrec = VideoRec(out_dir / f"index_bend_{stamp}.mp4", args.fps, (ui.shape[1], ui.shape[0]))
                args.record = False
            if vrec is not None:
                vrec.write(ui)
                el = time.time() - vrec.t0
                cv2.circle(ui, (row_w - 720, 15), 7, (0, 0, 255), -1)
                put(ui, f"REC {int(el // 60):02d}:{int(el % 60):02d}", (row_w - 708, 21), 0.5, (0, 0, 255), 2)

            cv2.imshow(win, ui)
            k = cv2.waitKey(1) & 0xFF
            ch = chr(k).lower() if k < 128 else ""
            if ch == "q" or k == 27:
                break
            elif ch in ("o", "f") and capture is None:
                capture = dict(kind="open" if ch == "o" else "fist", t_end=now + 2.0, v=[])
                print("편 손 기록 중 (2초)..." if ch == "o" else "굽힌 손 기록 중 (2초)...")
            elif ch == "p":
                for c in cams:
                    r = st[c.serial]["res"]
                    st[c.serial]["prox"] = r["X"][2] if r is not None else None
                print("P/D 방향 뒤집음")
            elif ch in ("0", "1", "2", "3"):
                view.reset() if ch == "0" else view.preset(int(ch))
            elif ch == "g":
                P = [cam_to_world(d["xyz"]) for c in cams for d in current[c.serial]]
                if P:
                    view.target = np.mean(P, axis=0)
                    view.dist = 350.0
            elif ch == "c":
                for t in trails.values():
                    t.clear()
                hist.clear()
            elif ch == "r" and writer is not None:
                saving = not saving
            elif ch == "v":
                if vrec is None:
                    vrec = VideoRec(out_dir / f"index_bend_{datetime.now():%Y%m%d_%H%M%S}.mp4", args.fps,
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
