"""
marker_color_select.py — 마커 후보 6색 중에서 '서로 확실히 다르고, 살색/검은 장갑/배경과 안 겹치는' 3색 고르기.

방법
  1) 카메라 화면(여러 대 동시)에서 샘플 수집
       마커 6색 : 마커를 클릭 (튜너와 같은 방식: 근처 마커에 자동으로 붙음)
       살색/장갑/배경 : 마우스로 드래그해서 사각형 영역 지정
  2) 각 마커 색 샘플로, 트래킹 코드가 실제로 쓰는 것과 같은 함수(ranges_from)로 HSV 범위를 만든다
  3) 그 범위 안에 '다른 클래스' 픽셀이 몇 % 들어오는지(오검출률) 계산 → 6x9 행렬
  4) 6색 중 3색 조합 20개를 모두 비교:
       worst = 조합 안의 어떤 색 범위가, 조합의 다른 마커/살색/장갑/배경을 잡는 비율의 최댓값
       → worst 가 낮은 순, 같으면 색상(H) 간격이 넓은 순. 카메라가 여러 대면 더 나쁜 카메라 값으로 평가.
  5) 결과: 콘솔 표 + 색상-채도 원판 그림 + 보고서 파일 (color_select/ 폴더)

실행:
    conda activate realsense
    python marker_color_select.py --res 1280x720
    python marker_color_select.py --eval-only          # 저장된 샘플로 평가만 다시

팁
  - 실제 실험 조명/거리에서, 각 마커를 여러 각도로 2~3번씩 클릭 (카메라마다)
  - 살색: 손바닥/손가락 피부를 여러 군데 드래그.  장갑: 검은 장갑 여러 군데.  배경: 책상/벽 등 화면에 보이는 것
  - SPACE 로 화면 정지 후 클릭/드래그하면 편함
  - 마커가 작으면: 마커 근처를 '오른쪽 클릭' → 그 주변이 크게 확대된 창이 고정되고 화면도 멈춤
    → 확대창 안에서 마커를 클릭(또는 살색/장갑 드래그) → 원본의 정확한 픽셀로 들어감
    → 다시 오른쪽 클릭 / ESC 로 닫기. 마우스를 올리기만 해도 구석에 작은 돋보기가 뜸.

키:
    오른쪽 클릭 : 확대창 열기/닫기 (확대창 안에서 클릭/드래그)     ESC : 확대창 닫기 (없으면 종료)
    1~6 : 마커 색 선택 (1 빨강 2 초록 3 보라 4 핑크 5 노랑 6 파랑)
    7 살색   8 장갑   9 배경          ← 이 셋은 드래그로 영역 지정
    SPACE 화면 정지/재개   U 마지막 샘플 취소   R 선택 클래스 초기화   M 선택 마커 범위 마스크 보기
    E 평가 (결과 창 + 콘솔 표)      S 샘플/결과 저장      Q 종료
"""

import argparse
import itertools
import json
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from marker_common import (HERE, RSCamera, cv2, list_devices, make_mask, put, ranges_from)

MARKERS = [("red", "빨강"), ("green", "초록"), ("purple", "보라"),
           ("pink", "핑크"), ("yellow", "노랑"), ("blue", "파랑")]
REGIONS = [("skin", "살색"), ("glove", "장갑(검정)"), ("bg", "배경")]
CLASSES = MARKERS + REGIONS
KO = dict(CLASSES)
MARKER_KEYS = [k for k, _ in MARKERS]
REGION_KEYS = [k for k, _ in REGIONS]
# 화면 표시용 색 (BGR)
DRAW = dict(red=(40, 40, 230), green=(60, 200, 60), purple=(200, 70, 150), pink=(180, 120, 255),
            yellow=(40, 220, 240), blue=(220, 120, 30), skin=(140, 180, 230), glove=(110, 110, 110),
            bg=(170, 170, 170))
HOVER_R, HOVER_K = 20, 6     # 마우스 따라다니는 돋보기: ±20px 을 6배
LOCK_R, LOCK_K = 40, 6       # 오른쪽 클릭 고정 확대창: ±40px 을 (최대) 6배 → 이 안에서 클릭/드래그
COUNTS_H = 20 * 9 + 24       # 타일 왼쪽 아래 '클래스별 샘플 수' 목록 높이 (확대창이 가리지 않게)
MAX_PER_CLASS = 20000       # 카메라·클래스당 저장 픽셀 상한 (영역 드래그가 커도 이만큼만 랜덤 추출)
MIN_MARKER_PX = 30
MIN_REGION_PX = 300
BAR_H = 64
DATA_DIR = HERE / "color_select"


# --------------------------------------------------------------------------- 평가 (카메라 없이도 동작)
def in_range(px, rng):
    """make_mask 와 같은 규칙으로 픽셀(N,3) 이 범위 안인지."""
    px = np.asarray(px)
    h, s, v = px[:, 0].astype(int), px[:, 1].astype(int), px[:, 2].astype(int)
    hlo, hhi = rng["h"]
    if hlo < 0 or hhi > 179:
        hok = (h >= int(hlo % 180)) | (h <= int(hhi % 180))
    else:
        hok = (h >= int(hlo)) & (h <= int(hhi))
    return hok & (s >= rng["s"][0]) & (s <= rng["s"][1]) & (v >= rng["v"][0]) & (v <= rng["v"][1])


def hue_center_deg(px):
    ang = np.radians(np.asarray(px)[:, 0].astype(float) * 2)
    return float(np.degrees(np.arctan2(np.sin(ang).mean(), np.cos(ang).mean())) % 360)


def hue_dist(a, b):
    d = abs(a - b) % 360
    return min(d, 360 - d)


def evaluate(samples_by_cam):
    """samples_by_cam: {cam_label: {class: (N,3) uint8 HSV}}
    반환: dict(per_cam, fp, recall, hue, triples, missing)"""
    per_cam = {}
    for cam, smp in samples_by_cam.items():
        res = dict(rng={}, recall={}, fp={}, hue={})
        for c in MARKER_KEYS:
            px = smp.get(c)
            if px is None or len(px) < MIN_MARKER_PX:
                continue
            rng = ranges_from(px)
            if rng is None:
                continue
            res["rng"][c] = rng
            res["recall"][c] = float(in_range(px, rng).mean())
            res["hue"][c] = hue_center_deg(px)
            res["fp"][c] = {}
            for o, _ in CLASSES:
                opx = smp.get(o)
                if o == c or opx is None or len(opx) < (MIN_MARKER_PX if o in MARKER_KEYS else MIN_REGION_PX):
                    continue
                res["fp"][c][o] = float(in_range(opx, rng).mean())
        per_cam[cam] = res

    # 카메라 합치기: 오검출은 최댓값(더 나쁜 카메라), recall 은 최솟값
    avail = [c for c in MARKER_KEYS if any(c in r["rng"] for r in per_cam.values())]
    fp, recall, hue = {}, {}, {}
    for c in avail:
        fp[c] = {}
        for o, _ in CLASSES:
            vals = [r["fp"][c][o] for r in per_cam.values() if c in r["fp"] and o in r["fp"][c]]
            if vals:
                fp[c][o] = max(vals)
        recall[c] = min(r["recall"][c] for r in per_cam.values() if c in r["recall"])
        hs = [r["hue"][c] for r in per_cam.values() if c in r["hue"]]
        ang = np.radians(hs)
        hue[c] = float(np.degrees(np.arctan2(np.sin(ang).mean(), np.cos(ang).mean())) % 360)

    triples = []
    for T in itertools.combinations(avail, 3):
        worst, worst_pair = 0.0, None
        for c in T:
            for o in [x for x in T if x != c] + REGION_KEYS:
                v = fp[c].get(o)
                if v is not None and v >= worst:
                    worst, worst_pair = v, (c, o)
        gap = min(hue_dist(hue[a], hue[b]) for a, b in itertools.combinations(T, 2))
        triples.append(dict(colors=T, worst=worst, worst_pair=worst_pair, hue_gap=gap,
                            recall=min(recall[c] for c in T)))
    # 오검출 0.05% 단위로 같은 등급이면 색상 간격 넓은 쪽, 그다음 recall
    triples.sort(key=lambda t: (round(t["worst"] * 2000), -t["hue_gap"], -t["recall"]))

    missing = {}
    for cam, smp in samples_by_cam.items():
        miss = [c for c in MARKER_KEYS if smp.get(c) is None or len(smp[c]) < MIN_MARKER_PX]
        miss += [r for r in REGION_KEYS if smp.get(r) is None or len(smp[r]) < MIN_REGION_PX]
        if miss:
            missing[cam] = miss
    return dict(per_cam=per_cam, fp=fp, recall=recall, hue=hue, triples=triples, missing=missing)


def report_text(ev):
    L = []
    if ev["missing"]:
        L.append("※ 샘플 부족(평가에서 빠짐): " +
                 "; ".join(f"{cam}: {', '.join(KO[m] for m in ms)}" for cam, ms in ev["missing"].items()))
        L.append("")
    cols = MARKER_KEYS + REGION_KEYS
    L.append("오검출률 [%] — 행: 이 색의 HSV 범위가 / 열: 이 클래스 픽셀을 잡는 비율 (카메라 중 최댓값)")
    L.append("          " + "".join(f"{KO[o]:>7s}" for o in cols) + "   자기검출  색상중심")
    for c in MARKER_KEYS:
        if c not in ev["fp"]:
            L.append(f"{KO[c]:6s}    (샘플 없음)")
            continue
        row = ""
        for o in cols:
            v = ev["fp"][c].get(o)
            row += "      -" if o == c else ("      ?" if v is None else f"{v * 100:7.2f}")
        L.append(f"{KO[c]:6s}  {row}   {ev['recall'][c] * 100:6.1f}%   {ev['hue'][c]:5.0f}°")
    L.append("")
    L.append("3색 조합 순위 (worst = 조합 안에서 가장 나쁜 오검출률, 낮을수록 좋음)")
    for i, t in enumerate(ev["triples"][:8], 1):
        a, b = t["worst_pair"] if t["worst_pair"] else ("-", "-")
        ok = "OK " if t["worst"] < 0.001 and t["recall"] > 0.9 else ("주의" if t["worst"] < 0.01 else "나쁨")
        L.append(f"{i:2d}. {' + '.join(KO[c] for c in t['colors']):14s}  worst {t['worst'] * 100:6.2f}%"
                 f" ({KO.get(a, a)}→{KO.get(b, b)})  색상간격 {t['hue_gap']:5.0f}°  "
                 f"자기검출 {t['recall'] * 100:5.1f}%  [{ok}]")
    return "\n".join(L)


def draw_hs_disk(samples_by_cam, ev, size=600):
    """색상(각도)-채도(반지름) 원판에 클래스별 픽셀 분포. 검은 장갑처럼 어두운 픽셀(V<40)은 회색 점."""
    img = np.full((size, size, 3), 25, np.uint8)
    c0, R = size // 2, size // 2 - 40
    for k in range(180):                         # 바깥 색상 링
        a = np.radians(k * 2)
        col = cv2.cvtColor(np.uint8([[[k, 255, 255]]]), cv2.COLOR_HSV2BGR)[0, 0].tolist()
        p = (int(c0 + (R + 14) * np.cos(a)), int(c0 - (R + 14) * np.sin(a)))
        cv2.circle(img, p, 5, col, -1)
    for rr in (0.25, 0.5, 0.75, 1.0):
        cv2.circle(img, (c0, c0), int(R * rr), (60, 60, 60), 1)
    put(img, "S=255", (c0 + R - 40, c0 - 4), 0.4, (120, 120, 120))
    rng = np.random.default_rng(0)
    for cls, _ in REGIONS + MARKERS:            # 영역 먼저 그리고 마커를 위에
        pts = [s[cls] for s in samples_by_cam.values() if s.get(cls) is not None and len(s[cls])]
        if not pts:
            continue
        px = np.concatenate(pts)
        px = px[rng.choice(len(px), min(len(px), 1500), replace=False)]
        a = np.radians(px[:, 0].astype(float) * 2)
        r = px[:, 1].astype(float) / 255 * R
        x = (c0 + r * np.cos(a)).astype(int)
        y = (c0 - r * np.sin(a)).astype(int)
        col = DRAW[cls]
        dark = px[:, 2] < 40
        for xi, yi, d in zip(x, y, dark):
            img[max(0, yi - 1):yi + 1, max(0, xi - 1):xi + 1] = (70, 70, 70) if d else col
        if cls in ev["hue"]:
            a0 = np.radians(ev["hue"][cls])
            rm = float(np.median(px[:, 1])) / 255 * R
            p = (int(c0 + rm * np.cos(a0)), int(c0 - rm * np.sin(a0)))
            put(img, cls, (p[0] + 6, p[1]), 0.5, col, 1)
        elif cls in REGION_KEYS:
            put(img, cls, (int(np.median(x)) + 6, int(np.median(y))), 0.5, col, 1)
    put(img, "hue = angle, saturation = radius  (gray dots: V<40, too dark)", (10, size - 12), 0.42, (150, 150, 150))
    return img


def result_window(samples_by_cam, ev):
    disk = draw_hs_disk(samples_by_cam, ev)
    panel = np.full((600, 560, 3), 18, np.uint8)
    put(panel, "Best 3-color sets", (14, 30), 0.7, (255, 255, 255), 2)
    y = 66
    for i, t in enumerate(ev["triples"][:8], 1):
        ok = t["worst"] < 0.001 and t["recall"] > 0.9
        col = (80, 220, 80) if ok else ((0, 200, 255) if t["worst"] < 0.01 else (60, 60, 255))
        x = 14
        put(panel, f"{i}.", (x, y), 0.55, col, 1)
        x += 30
        for c in t["colors"]:
            cv2.circle(panel, (x + 6, y - 5), 7, DRAW[c], -1)
            put(panel, c, (x + 16, y), 0.5, (230, 230, 230))
            x += 90
        put(panel, f"worst {t['worst'] * 100:5.2f}%  gap {t['hue_gap']:3.0f}deg", (x, y), 0.45, col)
        if t["worst_pair"]:
            put(panel, f"   worst: {t['worst_pair'][0]} range catches {t['worst_pair'][1]}",
                (44, y + 18), 0.4, (140, 140, 140))
        y += 46
    put(panel, "worst = max false-detection rate inside the set", (14, 560), 0.42, (150, 150, 150))
    put(panel, "(vs other 2 markers, skin, glove, background)", (14, 580), 0.42, (150, 150, 150))
    return np.hstack([disk, panel])


# --------------------------------------------------------------------------- 저장/불러오기
def save_samples(samples, cams_label, out_dir):
    out_dir.mkdir(parents=True, exist_ok=True)
    paths = []
    for serial, smp in samples.items():
        arrs = {c: np.concatenate(v) if v else np.zeros((0, 3), np.uint8) for c, v in smp.items()}
        p = out_dir / f"samples_{serial}.npz"
        np.savez_compressed(p, camera=cams_label.get(serial, serial), **arrs)
        paths.append(p)
    return paths


def load_samples(out_dir):
    res = {}
    for p in sorted(Path(out_dir).glob("samples_*.npz")):
        d = np.load(p)
        label = str(d["camera"]) if "camera" in d else p.stem
        res[label] = {c: d[c] for c, _ in CLASSES if c in d and len(d[c])}
    return res


def save_report(samples_by_cam, ev, out_dir):
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = f"{datetime.now():%Y%m%d_%H%M%S}"
    txt = out_dir / f"report_{stamp}.txt"
    txt.write_text(report_text(ev), encoding="utf-8")
    png = out_dir / f"report_{stamp}.png"
    cv2.imencode(".png", result_window(samples_by_cam, ev))[1].tofile(str(png))
    # 1등 조합의 범위를 바로 쓸 수 있게 카메라별 HSV json 도 같이 (marker_hsv_tune 형식)
    best = ev["triples"][0]["colors"] if ev["triples"] else ()
    js = {cam: {c: r["rng"][c] for c in best if c in r["rng"]} for cam, r in ev["per_cam"].items()}
    (out_dir / f"best_ranges_{stamp}.json").write_text(json.dumps(js, ensure_ascii=False, indent=2), encoding="utf-8")
    return txt, png


# --------------------------------------------------------------------------- 실시간 수집 UI
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--serial", nargs="*")
    ap.add_argument("--res", default="1280x720")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--data-dir", default=str(DATA_DIR))
    ap.add_argument("--eval-only", action="store_true", help="카메라 없이 저장된 샘플로 평가만")
    ap.add_argument("--resume", action="store_true", help="저장된 샘플에 이어서 수집")
    args = ap.parse_args()
    out_dir = Path(args.data_dir)

    if args.eval_only:
        smp = load_samples(out_dir)
        if not smp:
            print("저장된 샘플이 없습니다:", out_dir)
            return
        ev = evaluate(smp)
        print(report_text(ev))
        txt, png = save_report(smp, ev, out_dir)
        print("\n보고서:", txt, "\n그림:", png)
        return

    from marker_hsv_tune import snap_region, region_contours   # 튜너와 같은 클릭 방식

    W, H = map(int, args.res.lower().split("x"))
    devs = list_devices()
    print("연결된 RealSense:", ", ".join(f"{n} ({s})" for n, s in devs) or "없음")
    cams = []
    for s in (args.serial or [sv for _, sv in devs]):
        try:
            cams.append(RSCamera(s, W, H, args.fps))
            print("열림:", cams[-1].label)
        except Exception as e:
            print(f"열기 실패 {s}: {e}")
    if not cams:
        return
    labels = {c.serial: c.label for c in cams}

    samples = {c.serial: {k: [] for k, _ in CLASSES} for c in cams}
    if args.resume:
        for label, d in load_samples(out_dir).items():
            for c in cams:
                if c.label == label:
                    for k, v in d.items():
                        samples[c.serial][k].append(v)
        print("저장된 샘플에 이어서 수집")
    history = []
    st = dict(sel=0, frozen=False, mask=False, mouse=None, drag=None, drag_inz=False, flash=None,
              zoom=None, msg="", msg_t=0.0)
    items = {}
    ev_win = "color select result"

    def say(en, ko=None, col=(255, 255, 255)):
        print(ko or en)
        st["msg"], st["msg_col"], st["msg_t"] = en, col, time.time()

    def add(serial, cls, px):
        px = np.asarray(px, np.uint8).reshape(-1, 3)
        total = sum(len(a) for a in samples[serial][cls])
        if total + len(px) > MAX_PER_CLASS:          # 상한 넘으면 랜덤으로 줄여서 저장
            keep = max(0, MAX_PER_CLASS - total)
            px = px[np.random.default_rng().choice(len(px), keep, replace=False)] if keep else px[:0]
        if len(px):
            samples[serial][cls].append(px)
            history.append((serial, cls))
        return len(px)

    def cur_cls():
        return CLASSES[st["sel"]][0]

    # ---- 고정 확대창 (오른쪽 클릭) ----
    def open_zoom(ci, tx, ty):
        n = 2 * LOCK_R + 1
        # 위 헤더(24px) 아래부터, 왼쪽 아래 샘플 수 목록(COUNTS_H)을 가리지 않는 크기까지만
        K = max(2, min(LOCK_K, (H - 30 - COUNTS_H) // n))
        pw = n * K
        st["zoom"] = dict(
            ci=ci, n=n, K=K,
            zx0=int(np.clip(tx - LOCK_R, 0, W - n)), zy0=int(np.clip(ty - LOCK_R, 0, H - n)),
            px0=10 if tx > W // 2 else W - pw - 10,           # 크롭 위치 반대쪽에 띄움
            py0=30,
            auto_frozen=not st["frozen"])
        st["frozen"] = True                                    # 확대 중에는 화면 정지
        say("ZOOM: click/drag inside the zoom box | right-click or ESC to close",
            "확대창: 확대창 안에서 클릭/드래그 → 원본 정확한 위치로 들어감 (오른쪽 클릭/ESC 닫기)", (0, 200, 255))

    def close_zoom():
        z = st["zoom"]
        if z and z["auto_frozen"]:
            st["frozen"] = False
        st["zoom"] = None
        st["drag"] = None

    def tile_point(x, y, clamp_zoom=False):
        """창 좌표 → (ci, 원본 x, 원본 y, 확대창 안인지). 확대창 안이면 원본 픽셀로 환산."""
        if y < BAR_H or x // W >= len(cams):
            return None
        ci, tx, ty = x // W, x % W, y - BAR_H
        z = st["zoom"]
        if z and z["ci"] == ci:
            pw = z["n"] * z["K"]
            inside = z["px0"] <= tx < z["px0"] + pw and z["py0"] <= ty < z["py0"] + pw
            if inside or clamp_zoom:
                qx = int(np.clip(tx - z["px0"], 0, pw - 1)) // z["K"]
                qy = int(np.clip(ty - z["py0"], 0, pw - 1)) // z["K"]
                return ci, z["zx0"] + qx, z["zy0"] + qy, True
        return ci, tx, ty, False

    def on_mouse(ev, x, y, flags, *_):
        dragging_zoom = st["drag"] is not None and st["drag_inz"]
        tp = tile_point(x, y, clamp_zoom=dragging_zoom)
        if tp is None:
            st["mouse"] = None
            return
        ci, tx, ty, inz = tp
        st["mouse"] = (ci, tx, ty, inz)
        if ev == cv2.EVENT_RBUTTONDOWN:
            if st["zoom"]:
                close_zoom()
            else:
                open_zoom(ci, tx, ty)
            return
        cam = cams[ci]
        it = items.get(cam.serial)
        if it is None:
            return
        cls = cur_cls()
        if cls in REGION_KEYS:                      # 드래그 사각형
            if ev == cv2.EVENT_LBUTTONDOWN:
                st["drag"] = (ci, tx, ty, tx, ty)
                st["drag_inz"] = inz
            elif ev == cv2.EVENT_MOUSEMOVE and st["drag"] and st["drag"][0] == ci:
                st["drag"] = st["drag"][:3] + (tx, ty)
            elif ev == cv2.EVENT_LBUTTONUP and st["drag"]:
                _, x0, y0, x1, y1 = st["drag"]
                st["drag"] = None
                x0, x1 = sorted((x0, x1))
                y0, y1 = sorted((y0, y1))
                if (x1 - x0) * (y1 - y0) < 25:
                    say("drag a bigger box", "더 크게 드래그하세요", (60, 60, 255))
                    return
                n = add(cam.serial, cls, it["hsv"][y0:y1, x0:x1].reshape(-1, 3))
                say(f"OK {cam.model} / {cls}: +{n}px", f"{cam.model} / {KO[cls]}: +{n}px", (80, 220, 80))
        elif ev == cv2.EVENT_LBUTTONDOWN:          # 마커 클릭
            reg, off = snap_region(it["hsv"], tx, ty)
            if reg is None:
                say("no marker near click", "클릭 근처에서 마커를 못 찾음", (60, 60, 255))
                return
            x0, y0 = off
            px = it["hsv"][y0:y0 + reg.shape[0], x0:x0 + reg.shape[1]][reg]
            n = add(cam.serial, cls, px)
            st["flash"] = (ci, region_contours(reg, off), time.time())
            say(f"OK {cam.model} / {cls}: +{n}px", f"{cam.model} / {KO[cls]}: +{n}px", (80, 220, 80))

    win = "marker color select"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    total_w = W * len(cams)
    if total_w > 1800:
        cv2.resizeWindow(win, 1800, int((H + BAR_H) * 1800 / total_w))
    cv2.setMouseCallback(win, on_mouse)
    print(__doc__.split("키:")[1])

    try:
        while True:
            if not st["frozen"]:
                for c in cams:
                    it = c.get()
                    if it is not None:
                        items[c.serial] = it
            cls = cur_cls()
            tiles = []
            for ci, c in enumerate(cams):
                it = items.get(c.serial)
                if it is None:
                    tiles.append(np.zeros((H, W, 3), np.uint8))
                    continue
                img = it["color"].copy()
                if st["mask"] and cls in MARKER_KEYS:
                    px = samples[c.serial][cls]
                    rng = ranges_from(np.concatenate(px)) if px and sum(map(len, px)) >= MIN_MARKER_PX else None
                    m = make_mask(it["hsv"], rng) if rng else np.zeros((H, W), np.uint8)
                    img = cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)
                mo = st["mouse"]
                if mo and mo[0] == ci and cls in MARKER_KEYS and not st["mask"]:
                    reg, off = snap_region(it["hsv"], mo[1], mo[2])
                    if reg is not None:
                        cv2.drawContours(img, region_contours(reg, off), -1, (0, 255, 255), 1)
                fl = st["flash"]
                if fl and fl[0] == ci and time.time() - fl[2] < 1.5:
                    cv2.drawContours(img, fl[1], -1, (0, 255, 0), 2)
                dr = st["drag"]
                if dr and dr[0] == ci:
                    cv2.rectangle(img, (dr[1], dr[2]), (dr[3], dr[4]), DRAW[cls], 1 if st["zoom"] else 2)

                z = st["zoom"]
                if z and z["ci"] == ci:
                    # 고정 확대창: 원본에서 크롭 → 확대해서 붙임. 이 안의 클릭/드래그는 원본 좌표로 환산됨
                    n, K = z["n"], z["K"]
                    big = cv2.resize(img[z["zy0"]:z["zy0"] + n, z["zx0"]:z["zx0"] + n], None,
                                     fx=K, fy=K, interpolation=cv2.INTER_NEAREST)
                    if mo and mo[0] == ci and mo[3]:
                        c_ = ((mo[1] - z["zx0"]) * K + K // 2, (mo[2] - z["zy0"]) * K + K // 2)
                        cv2.drawMarker(big, c_, (255, 255, 255), cv2.MARKER_CROSS, 24, 1)
                    cv2.rectangle(img, (z["zx0"], z["zy0"]), (z["zx0"] + n, z["zy0"] + n), (255, 255, 255), 1)
                    pw = n * K
                    img[z["py0"]:z["py0"] + pw, z["px0"]:z["px0"] + pw] = big
                    cv2.rectangle(img, (z["px0"] - 2, z["py0"] - 2), (z["px0"] + pw + 1, z["py0"] + pw + 1),
                                  (0, 200, 255), 2)
                    put(img, f"ZOOM x{K}: click/drag here | right-click/ESC close",
                        (z["px0"] + 4, z["py0"] + 16), 0.45, (0, 200, 255))     # 확대창 안쪽 위
                elif mo and mo[0] == ci and not st["mask"]:
                    # 마우스 따라다니는 돋보기 (보기 전용, 마우스 반대쪽 모서리)
                    n = 2 * HOVER_R + 1
                    zx0 = int(np.clip(mo[1] - HOVER_R, 0, W - n))
                    zy0 = int(np.clip(mo[2] - HOVER_R, 0, H - n))
                    big = cv2.resize(img[zy0:zy0 + n, zx0:zx0 + n], None, fx=HOVER_K, fy=HOVER_K,
                                     interpolation=cv2.INTER_NEAREST)
                    c_ = ((mo[1] - zx0) * HOVER_K + HOVER_K // 2, (mo[2] - zy0) * HOVER_K + HOVER_K // 2)
                    cv2.drawMarker(big, c_, (255, 255, 255), cv2.MARKER_CROSS, 30, 1)
                    zh = big.shape[0]
                    px0 = 8 if mo[1] > W // 2 else W - zh - 8     # 마우스 반대쪽, 항상 위쪽
                    py0 = 30                                       # (왼쪽 아래 샘플 수 목록 안 가림)
                    img[py0:py0 + zh, px0:px0 + zh] = big
                    cv2.rectangle(img, (px0 - 1, py0 - 1), (px0 + zh, py0 + zh), (255, 255, 255), 1)
                cv2.rectangle(img, (0, 0), (W, 24), (0, 0, 0), -1)
                put(img, f"{c.label}  {c.fps_val:4.1f}fps" + ("   FROZEN" if st["frozen"] else ""), (6, 17), 0.5)
                # 클래스별 샘플 수
                y = H - 12 - 20 * (len(CLASSES) - 1)
                for k, _ in CLASSES:
                    n = sum(len(a) for a in samples[c.serial][k])
                    need = MIN_MARKER_PX if k in MARKER_KEYS else MIN_REGION_PX
                    put(img, f"{k:7s} {n:6d}px", (8, y), 0.45,
                        (80, 220, 80) if n >= need else ((0, 200, 255) if n else (130, 130, 130)))
                    y += 20
                cv2.line(img, (W - 1, 0), (W - 1, H), (90, 90, 90), 1)
                tiles.append(img)

            bar = np.full((BAR_H, total_w, 3), 30, np.uint8)
            x = 8
            for i, (k, _) in enumerate(CLASSES):
                txt = f"{i + 1}:{k}"
                tw = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0][0]
                if i == st["sel"]:
                    cv2.rectangle(bar, (x - 4, 4), (x + tw + 4, 28), DRAW[k], 2)
                put(bar, txt, (x, 22), 0.5, DRAW[k])
                x += tw + 16
            if st["msg"] and time.time() - st["msg_t"] < 4:
                put(bar, st["msg"], (8, 52), 0.5, st.get("msg_col", (255, 255, 255)))
            else:
                hint = ("click the marker" if cls in MARKER_KEYS else "DRAG a box over " + cls)
                put(bar, f"{hint} | RIGHT-CLICK zoom | SPACE freeze | U undo | R reset | M mask | "
                         f"E evaluate | S save | Q quit", (8, 52), 0.45, (170, 170, 170))
            cv2.imshow(win, np.vstack([bar, np.hstack(tiles)]))

            k = cv2.waitKey(1) & 0xFF
            ch = chr(k).lower() if k < 128 else ""
            if ch and ch in "123456789":
                st["sel"] = int(ch) - 1
                print(f"→ {KO[cur_cls()]}" + (" (드래그로 영역 지정)" if cur_cls() in REGION_KEYS else " (마커 클릭)"))
            elif ch == " ":
                st["frozen"] = not st["frozen"]
            elif ch == "m":
                st["mask"] = not st["mask"]
            elif ch == "u" and history:
                s, c_ = history.pop()
                samples[s][c_].pop()
                say(f"undo {c_}", f"취소: {KO[c_]}")
            elif ch == "r":
                for c in cams:
                    samples[c.serial][cls] = []
                history[:] = [h for h in history if h[1] != cls]
                say(f"reset {cls}", f"{KO[cls]} 샘플 초기화")
            elif ch in ("e", "s"):
                by_cam = {labels[s]: {k_: np.concatenate(v) for k_, v in d.items() if v}
                          for s, d in samples.items()}
                ev = evaluate(by_cam)
                print("\n" + report_text(ev) + "\n")
                cv2.imshow(ev_win, result_window(by_cam, ev))
                if ch == "s":
                    paths = save_samples(samples, labels, out_dir)
                    txt, png = save_report(by_cam, ev, out_dir)
                    print("샘플 저장:", *paths, sep="\n  ")
                    print("보고서:", txt, "\n그림:", png)
            elif k == 27 and st["zoom"]:
                close_zoom()
            elif ch == "q" or k == 27:
                break
            if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        for c in cams:
            c.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
