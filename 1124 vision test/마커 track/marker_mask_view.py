"""
marker_mask_view.py — 저장한 HSV 샘플(color_select/samples_<serial>.npz)로 범위를 만들어,
체크한 색만 카메라 화면(여러 대 동시)에 실시간으로 칠해서 보여준다.

  - 1~6 으로 색 켜기/끄기 (1 빨강 2 초록 3 보라 4 핑크 5 노랑 6 파랑)
  - 켠 색이 잡히는 픽셀을 그 색으로 칠함. 두 색 이상이 같은 픽셀을 잡으면 흰색(= 서로 헷갈림)
  - 색마다 잡힌 덩어리 수 / 픽셀 수 표시 → 마커 개수보다 많으면 오검출
  - 범위는 카메라마다 따로 (그 카메라에서 찍은 샘플로), 트래커와 같은 ranges_from 사용

샘플 정리 (C 키, 기본 ON)
  클릭 실수로 다른 색 마커 픽셀이 섞이면 범위가 엄청 넓어진다. 그래서 각 색 샘플에서
  가장 많이 모인 색상(hue)을 찾고 그로부터 ±HUE_KEEP_DEG 밖의 픽셀은 빼고 범위를 만든다.

실행:
    conda activate realsense
    python marker_mask_view.py --res 1280x720

키:
    1~6 색 켜기/끄기   A 전부 켜기/끄기   V 보기 전환(원본+칠하기 / 마스크만)
    C 샘플 정리 ON/OFF   SPACE 화면 정지   S 스냅샷 저장   Q/ESC 종료
"""

import argparse
from datetime import datetime
from pathlib import Path

import numpy as np

from marker_common import HERE, MIN_AREA, RSCamera, cv2, list_devices, make_mask, put, ranges_from
from marker_color_select import DATA_DIR, DRAW, KO, MARKERS

MARKER_KEYS = [k for k, _ in MARKERS]
HUE_KEEP_DEG = 35.0
SIMILAR_DEG, SIMILAR_SAT = 20.0, 60   # 색상 중심 20° 이내 + 채도 중앙값 60 이내면 '비슷한 색' 경고
BAR_H = 64
LIST_H = 20 * len(MARKERS) + 30


def dominant_hue(px, keep_deg=HUE_KEEP_DEG):
    """가장 많이 모인 hue(10° 막대, 원형 스무딩) 주변 ±keep_deg 픽셀만 남김."""
    h = px[:, 0].astype(float) * 2
    hist, _ = np.histogram(h, bins=36, range=(0, 360))
    sm = hist + 0.5 * np.roll(hist, 1) + 0.5 * np.roll(hist, -1)
    peak = (np.argmax(sm) + 0.5) * 10
    d = np.abs((h - peak + 180) % 360 - 180)
    return px[d <= keep_deg]


def load_ranges(data_dir, serial, clean):
    p = Path(data_dir) / f"samples_{serial}.npz"
    if not p.is_file():
        return None, {}
    z = np.load(p)
    rngs, info = {}, {}
    for k in MARKER_KEYS:
        if k not in z.files or len(z[k]) < 30:
            continue
        px = z[k]
        used = dominant_hue(px) if clean else px
        r = ranges_from(used) if len(used) >= 30 else None
        if r:
            rngs[k] = r
            a = np.radians(used[:, 0].astype(float) * 2)
            center = float(np.degrees(np.arctan2(np.sin(a).mean(), np.cos(a).mean())) % 360)
            info[k] = (len(px), len(used), center, float(np.median(used[:, 1])))
    return p, (rngs, info)


def similar_pairs(info, deg=SIMILAR_DEG, sat=SIMILAR_SAT):
    """같은 카메라에서 색상 중심이 deg 이내이고 채도도 비슷한 색 쌍 = 샘플 실수이거나 원래 구분이 안 되는 색."""
    keys = list(info)
    out = []
    for i in range(len(keys)):
        for j in range(i + 1, len(keys)):
            a, b = info[keys[i]], info[keys[j]]
            dh = abs((a[2] - b[2] + 180) % 360 - 180)
            if dh <= deg and abs(a[3] - b[3]) <= sat:
                out.append((keys[i], keys[j], dh))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--serial", nargs="*")
    ap.add_argument("--res", default="1280x720")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--data-dir", default=str(DATA_DIR))
    args = ap.parse_args()
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

    st = dict(on={k: False for k in MARKER_KEYS}, clean=True, view="overlay", frozen=False)
    ranges, warns = {}, {}

    def reload():
        for c in cams:
            p, res = load_ranges(args.data_dir, c.serial, st["clean"])
            ranges[c.serial] = res[0] if res else {}
            warns[c.serial] = similar_pairs(res[1]) if res else []
            if p is None:
                print(f"  {c.label}: 샘플 파일 없음 (samples_{c.serial}.npz) → 이 카메라는 원본만")
                continue
            rngs, info = res
            print(f"  {c.label}  [샘플 정리 {'ON' if st['clean'] else 'OFF'}]")
            for k in MARKER_KEYS:
                if k in rngs:
                    r = rngs[k]
                    n0, n1, center, _ = info[k]
                    drop = f" (정리로 {n0 - n1}px 제외)" if n0 != n1 else ""
                    print(f"    {KO[k]:3s} H {r['h'][0]:6.1f}~{r['h'][1]:6.1f}  S {r['s'][0]:3d}~{r['s'][1]:3d}  "
                          f"V {r['v'][0]:3d}~{r['v'][1]:3d}  중심 {center:3.0f}°  샘플 {n1}px{drop}")
                else:
                    print(f"    {KO[k]:3s} (샘플 부족)")
            for a, b, dh in warns[c.serial]:
                print(f"    ⚠ {KO[a]} / {KO[b]}: 색상 중심이 {dh:.0f}° 차이뿐 — 이 카메라에서 구분 어려움 "
                      f"(또는 클릭 실수로 다른 색 샘플이 들어감 → 그 색 다시 찍기)")

    reload()
    items = {}
    win = "marker mask view"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    total_w = W * len(cams)
    if total_w > 1800:
        cv2.resizeWindow(win, 1800, int((H + BAR_H) * 1800 / total_w))
    print(__doc__.split("키:")[1])

    try:
        while True:
            if not st["frozen"]:
                for c in cams:
                    it = c.get()
                    if it is not None:
                        items[c.serial] = it
            tiles = []
            for c in cams:
                it = items.get(c.serial)
                if it is None:
                    tiles.append(np.zeros((H, W, 3), np.uint8))
                    continue
                rngs = ranges.get(c.serial, {})
                masks, stats = {}, {}
                for k in MARKER_KEYS:
                    if st["on"][k] and k in rngs:
                        m = make_mask(it["hsv"], rngs[k])
                        n, _, cs, _ = cv2.connectedComponentsWithStats(m)
                        stats[k] = (int((cs[1:, cv2.CC_STAT_AREA] >= MIN_AREA).sum()), int((m > 0).sum()))
                        masks[k] = m > 0
                img = it["color"].copy() if st["view"] == "overlay" else np.zeros((H, W, 3), np.uint8)
                if masks:
                    cnt = np.zeros((H, W), np.uint8)
                    for k, m in masks.items():
                        cnt += m
                        if st["view"] == "overlay":
                            img[m] = (0.35 * img[m] + 0.65 * np.array(DRAW[k])).astype(np.uint8)
                        else:
                            img[m] = DRAW[k]
                    img[cnt >= 2] = (255, 255, 255)          # 두 색 이상이 같이 잡은 픽셀 = 충돌
                    # 잡힌 덩어리 테두리 (작은 마커도 보이게)
                    for k, m in masks.items():
                        cs_, _ = cv2.findContours(m.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        cv2.drawContours(img, [q for q in cs_ if cv2.contourArea(q) >= MIN_AREA], -1, DRAW[k], 1)
                cv2.rectangle(img, (0, 0), (W, 24), (0, 0, 0), -1)
                put(img, f"{c.label}  {c.fps_val:4.1f}fps" + ("   FROZEN" if st["frozen"] else ""), (6, 17), 0.5)
                if warns.get(c.serial):
                    put(img, "similar samples: " + ", ".join(f"{a}~{b}" for a, b, _ in warns[c.serial]),
                        (6, 44), 0.5, (60, 60, 255), 1)
                # 왼쪽 아래 목록: 색 / 덩어리 수 / 픽셀 수
                cv2.rectangle(img, (0, H - LIST_H), (300, H), (0, 0, 0), -1)
                y = H - LIST_H + 20
                put(img, "color   blobs   pixels", (8, y), 0.45, (150, 150, 150))
                for k in MARKER_KEYS:
                    y += 20
                    if k not in rngs:
                        put(img, f"{k:7s} (no samples)", (8, y), 0.45, (100, 100, 100))
                    elif k in stats:
                        b, px = stats[k]
                        put(img, f"{k:7s} {b:5d}  {px:8d}", (8, y), 0.45, DRAW[k])
                    else:
                        put(img, f"{k:7s}   off", (8, y), 0.45, (110, 110, 110))
                cv2.line(img, (W - 1, 0), (W - 1, H), (90, 90, 90), 1)
                tiles.append(img)

            bar = np.full((BAR_H, total_w, 3), 30, np.uint8)
            x = 8
            for i, k in enumerate(MARKER_KEYS):
                on = st["on"][k]
                box = (x, 8, x + 16, 24)
                cv2.rectangle(bar, box[:2], box[2:], DRAW[k], -1 if on else 1)
                txt = f"{i + 1}:{k}"
                put(bar, txt, (x + 22, 22), 0.5, DRAW[k] if on else (120, 120, 120))
                x += 22 + cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0][0] + 18
            put(bar, f"clean {'ON' if st['clean'] else 'OFF'}  |  white = 2+ colors catch the same pixel  |  "
                     f"1-6 toggle  A all  V view  C clean  SPACE freeze  S snapshot  Q quit",
                (8, 52), 0.45, (170, 170, 170))
            ui = np.vstack([bar, np.hstack(tiles)])
            cv2.imshow(win, ui)

            k = cv2.waitKey(1) & 0xFF
            ch = chr(k).lower() if k < 128 else ""
            if ch and ch in "123456":
                key = MARKER_KEYS[int(ch) - 1]
                st["on"][key] = not st["on"][key]
            elif ch == "a":
                new = not all(st["on"].values())
                for key in MARKER_KEYS:
                    st["on"][key] = new
            elif ch == "v":
                st["view"] = "mask" if st["view"] == "overlay" else "overlay"
            elif ch == "c":
                st["clean"] = not st["clean"]
                reload()
            elif ch == " ":
                st["frozen"] = not st["frozen"]
            elif ch == "s":
                d = Path(args.data_dir)
                d.mkdir(parents=True, exist_ok=True)
                on = "+".join(key for key in MARKER_KEYS if st["on"][key]) or "none"
                p = d / f"maskview_{on}_{datetime.now():%Y%m%d_%H%M%S}.png"
                cv2.imencode(".png", ui)[1].tofile(str(p))
                print("저장:", p)
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
