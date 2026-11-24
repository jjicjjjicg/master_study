"""
marker_hsv_tune.py — 카메라 여러 대를 동시에 띄워서, 카메라마다 마커 HSV 범위를 잡아 json 으로 저장.

같은 스티커라도 카메라(화이트밸런스/센서)와 보는 각도에 따라 HSV 가 달라지므로
카메라별로 따로 저장한다:  hsv_<serial>.json  (이 폴더)

사용:
    conda activate realsense
    python marker_hsv_tune.py                                   # 연결된 RealSense 전부
    python marker_hsv_tune.py --markers wrist:1 thumb:3 index:4 middle:4
    python marker_hsv_tune.py --res 1280x720                    # 10mm 마커가 작게 보이면

순서:
    1) 숫자키로 마커 이름(색) 선택
    2) 각 카메라 화면에서 그 색 마커를 클릭 → 클릭 지점과 비슷한 색 영역을 샘플로 추가
       (여러 개 / 여러 각도에서 몇 번 클릭할수록 조명 변화에 강해짐)
    3) M 으로 마스크 확인: 그 색 마커만 하얗게 남아야 함. 손/배경이 섞이면 R 로 지우고 다시
    4) S 저장 (카메라마다 hsv_<serial>.json)

키:
    SPACE   : 화면 정지/재개  ← 손이 떨리면 정지시킨 상태에서 클릭
    1~9     : 마커 선택
    [ / ]   : 선택한 마커의 개수 -1 / +1  (트래킹에서 그 색 blob 을 몇 개까지 찾을지)
    클릭     : 클릭한 카메라에 대해 선택 마커 색 샘플 추가
               - 마우스를 올리면 돋보기 + 노란 테두리(지금 클릭하면 샘플될 영역)가 보임
               - 마커에서 15px 정도 빗나가도 가장 가까운 마커에 자동으로 붙음
               - 샘플 성공하면 초록 테두리가 잠깐 뜨고 위 막대에 'OK ... +N px' 표시
    U       : 마지막 클릭 취소
    R       : 선택 마커 샘플 초기화 (모든 카메라)
    M       : 마스크 보기 on/off
    S       : 저장
    Q / ESC : 종료 (저장 안 한 변경이 있으면 한 번 더 눌러야 종료)
"""

import argparse
import time

import numpy as np

from marker_common import (DEFAULT_MARKERS, HERE, MIN_AREA, FONT, RSCamera, cv2, detect, list_devices,
                           load_hsv, make_mask, parse_markers, put, range_bgr, ranges_from,
                           save_hsv)

BAR_H = 64
FLOOD_DIFF = (8, 70, 80)       # 시작 픽셀 기준 H/S/V 허용 차이 (flood fill)
CROP_R = 60                    # 클릭 주변 이 반경 안에서만 영역을 찾음 (마커보다 충분히 크게)
SNAP_R = 15                    # 클릭이 마커에서 이만큼(px) 빗나가도 가장 가까운 마커로 붙여줌
SNAP_MIN_S, SNAP_MIN_V = 60, 50  # '색이 있는' 픽셀 기준 (마커 후보)
ZOOM_R, ZOOM_K = 20, 6         # 돋보기: 마우스 주변 ±20px 을 6배 확대


def snap_region(hsv, x, y):
    """클릭(또는 마우스) 위치 근처의 마커 영역을 찾는다.
    1) 클릭 주변 SNAP_R 안에 있는 '색 있는 덩어리' 중 가장 가까운 것을 고르고
    2) 그 덩어리에서 채도가 가장 높은 픽셀(=마커 중심 쪽)에서 flood fill
    → 손이 떨려 마커 가장자리나 살짝 옆을 찍어도 마커 중심색으로 샘플링된다.
    반환: (region bool 배열 (crop 크기), (x0, y0) crop 시작점) 또는 (None, ...)"""
    Hh, Ww = hsv.shape[:2]
    x, y = int(x), int(y)
    x0, y0 = max(0, x - CROP_R), max(0, y - CROP_R)
    x1, y1 = min(Ww, x + CROP_R + 1), min(Hh, y + CROP_R + 1)
    crop = np.ascontiguousarray(hsv[y0:y1, x0:x1])
    ch, cw = crop.shape[:2]
    cx, cy = x - x0, y - y0

    colored = ((crop[..., 1] >= SNAP_MIN_S) & (crop[..., 2] >= SNAP_MIN_V)).astype(np.uint8)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(colored, connectivity=4)
    yy, xx = np.mgrid[0:ch, 0:cw]
    dist = np.hypot(xx - cx, yy - cy)
    cands = []
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < 6:      # 노이즈 점(몇 픽셀)은 마커 후보에서 제외
            continue
        sel = lab == i
        dmin = float(dist[sel].min())
        if dmin <= SNAP_R:
            s = np.where(sel, crop[..., 1].astype(int), -1)
            sy, sx = np.unravel_index(np.argmax(s), s.shape)
            cands.append((dmin, int(sx), int(sy)))
    cands.sort()
    seeds = [(sx, sy) for _, sx, sy in cands[:4]] + [(cx, cy)]   # 마지막: 클릭 지점 그대로 (연한 색 마커용)

    flags = 4 | cv2.FLOODFILL_FIXED_RANGE | cv2.FLOODFILL_MASK_ONLY | (255 << 8)
    for sx, sy in seeds:
        m = np.zeros((ch + 2, cw + 2), np.uint8)
        cv2.floodFill(crop, m, (sx, sy), 0, FLOOD_DIFF, FLOOD_DIFF, flags)
        reg = m[1:-1, 1:-1] > 0
        k = int(reg.sum())
        touches_edge = reg[0, :].any() or reg[-1, :].any() or reg[:, 0].any() or reg[:, -1].any()
        if k >= 3 and not touches_edge:          # 가장자리까지 퍼지면 마커가 아니라 배경/장갑
            return reg, (x0, y0)
    return None, (x0, y0)


def region_contours(reg, off):
    cs, _ = cv2.findContours(reg.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    return [c + np.array(off) for c in cs]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--serial", nargs="*", help="사용할 카메라 시리얼 (생략 시 전부)")
    ap.add_argument("--markers", nargs="*", help="마커 이름:개수 목록 (예: wrist:1 thumb:3)")
    ap.add_argument("--res", default="640x480")
    ap.add_argument("--hsv-dir", default=str(HERE), help="hsv_<serial>.json 저장 폴더")
    ap.add_argument("--fps", type=int, default=30)
    args = ap.parse_args()
    W, H = map(int, args.res.lower().split("x"))

    devs = list_devices()
    print("연결된 RealSense:", ", ".join(f"{n} ({s})" for n, s in devs) or "없음")
    serials = args.serial or [s for _, s in devs]
    cams = []
    for s in serials:
        try:
            cams.append(RSCamera(s, W, H, args.fps))
            print("열림:", cams[-1].label)
        except Exception as e:
            print(f"열기 실패 {s}: {e}")
    if not cams:
        return

    # ---- 마커 목록: 인자 > 기존 json 에 있는 이름 > 기본값 ----
    old = {c.serial: load_hsv(c.serial, args.hsv_dir) for c in cams}
    if args.markers:
        markers = parse_markers(args.markers)
    else:
        names = []
        for d in old.values():
            for k, v in d.items():
                if k not in [n for n, _ in names]:
                    names.append((k, int(v.get("count", 1))))
        markers = names or list(DEFAULT_MARKERS)
    markers = [list(m) for m in markers][:9]
    for c in cams:
        for k in old[c.serial]:
            if k not in [n for n, _ in markers]:
                print(f"  참고: {c.label} 의 기존 json 에 '{k}' 가 있지만 이번 목록에 없음 → 저장 시 유지")

    samples = {c.serial: {n: [] for n, _ in markers} for c in cams}
    ranges = {c.serial: {n: old[c.serial].get(n) for n, _ in markers} for c in cams}
    history = []           # (serial, name, 추가된 샘플 수)
    clicks = {c.serial: {n: 0 for n, _ in markers} for c in cams}
    state = dict(sel=0, mask=False, dirty=False, quit_armed=False, frozen=False,
                 msg="", msg_col=(255, 255, 255), msg_t=0.0, mouse=None, flash=None)
    items = {}

    def say(msg_en, msg_ko=None, col=(255, 255, 255)):
        print(msg_ko or msg_en)
        state["msg"], state["msg_col"], state["msg_t"] = msg_en, col, time.time()

    def on_mouse(ev, x, y, *_):
        if y < BAR_H or x // W >= len(cams):
            state["mouse"] = None
            return
        ci, tx, ty = x // W, x % W, y - BAR_H
        state["mouse"] = (ci, tx, ty)
        if ev != cv2.EVENT_LBUTTONDOWN:
            return
        cam = cams[ci]
        it = items.get(cam.serial)
        if it is None:
            return
        name = markers[state["sel"]][0]
        reg, off = snap_region(it["hsv"], tx, ty)
        if reg is None:
            say(f"{cam.model}: no marker near click - try again (SPACE to freeze)",
                f"{cam.model}: 클릭 근처에서 마커를 못 찾음 — 다시 클릭 (SPACE 로 화면 정지하면 쉬움)", (60, 60, 255))
            return
        x0, y0 = off
        px = it["hsv"][y0:y0 + reg.shape[0], x0:x0 + reg.shape[1]][reg].tolist()
        samples[cam.serial][name].extend(px)
        ranges[cam.serial][name] = ranges_from(samples[cam.serial][name])
        clicks[cam.serial][name] += 1
        history.append((cam.serial, name, len(px)))
        state["dirty"] = True
        state["flash"] = (ci, region_contours(reg, off), time.time())
        say(f"OK  {cam.model} / {name}: +{len(px)}px  (total {len(samples[cam.serial][name])}px, "
            f"{clicks[cam.serial][name]} clicks)",
            f"{cam.model} / {name}: +{len(px)}px (총 {len(samples[cam.serial][name])}px, {clicks[cam.serial][name]}번째 클릭)",
            (80, 220, 80))

    win = "marker HSV tune"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    total_w = W * len(cams)
    if total_w > 1800:
        cv2.resizeWindow(win, 1800, int((H + BAR_H) * 1800 / total_w))
    cv2.setMouseCallback(win, on_mouse)
    print(__doc__.split("키:")[1])

    try:
        while True:
            if not state["frozen"]:            # SPACE: 화면 정지 → 떨림 없이 클릭
                for c in cams:
                    it = c.get()
                    if it is not None:
                        items[c.serial] = it
            sel_name, sel_count = markers[state["sel"]]

            tiles = []
            for ci, c in enumerate(cams):
                it = items.get(c.serial)
                if it is None:
                    tiles.append(np.zeros((H, W, 3), np.uint8))
                    continue
                rngs = ranges[c.serial]
                if state["mask"]:
                    r = rngs.get(sel_name)
                    m = make_mask(it["hsv"], r) if r else np.zeros((H, W), np.uint8)
                    img = cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)
                else:
                    img = it["color"].copy()
                found_txt = []
                for name, count in markers:
                    r = rngs.get(name)
                    if not r:
                        found_txt.append((name, "--", (120, 120, 120)))
                        continue
                    dets, dmask = detect(c, it, r, count + 2)   # 개수보다 많이 잡히는지도 보이게 +2
                    nb, _, bst, _ = cv2.connectedComponentsWithStats(dmask)
                    n_all = int((bst[1:, cv2.CC_STAT_AREA] >= MIN_AREA).sum())   # 이 색으로 잡히는 덩어리 전체 개수
                    col = range_bgr(r)
                    is_sel = name == sel_name
                    for k, d in enumerate(dets):
                        extra = k >= count
                        cv2.circle(img, (int(d["u"]), int(d["v"])), 9 if is_sel else 6,
                                   (0, 0, 255) if extra else col, 2 if is_sel else 1)
                        if is_sel and not state["mask"]:
                            xyz = d["xyz"]
                            lab = (f"({xyz[0]:.0f},{xyz[1]:.0f},{xyz[2]:.0f})mm" if xyz is not None
                                   else "no depth")
                            put(img, lab, (int(d["u"]) + 10, int(d["v"]) - 8), 0.4, col)
                    n_ok = min(len(dets), count)
                    c_txt = (0, 200, 0) if len(dets) == count else ((0, 0, 255) if len(dets) > count else (0, 200, 255))
                    c_txt = (0, 200, 0) if n_all == count else ((0, 0, 255) if n_all > count else (0, 200, 255))
                    found_txt.append((name, f"{n_ok}/{count}" + (f"  +{n_all - count} false" if n_all > count else ""), c_txt))
                # 방금 클릭으로 샘플링된 영역 (1.5초간 초록 테두리)
                fl = state["flash"]
                if fl and fl[0] == ci and time.time() - fl[2] < 1.5:
                    cv2.drawContours(img, fl[1], -1, (0, 255, 0), 2)
                # 마우스 위치: 지금 클릭하면 샘플링될 영역 미리보기 (노란 테두리)
                mo = state["mouse"]
                if mo and mo[0] == ci:
                    reg, off = snap_region(it["hsv"], mo[1], mo[2])
                    if reg is not None:
                        cv2.drawContours(img, region_contours(reg, off), -1, (0, 255, 255), 1)
                    # 돋보기: 마우스 주변 확대 (마우스 반대쪽 위 모서리에)
                    zx0, zy0 = int(np.clip(mo[1] - ZOOM_R, 0, W - 2 * ZOOM_R - 1)), int(np.clip(mo[2] - ZOOM_R, 0, H - 2 * ZOOM_R - 1))
                    zoom = cv2.resize(img[zy0:zy0 + 2 * ZOOM_R + 1, zx0:zx0 + 2 * ZOOM_R + 1], None,
                                      fx=ZOOM_K, fy=ZOOM_K, interpolation=cv2.INTER_NEAREST)
                    zc = ((mo[1] - zx0) * ZOOM_K + ZOOM_K // 2, (mo[2] - zy0) * ZOOM_K + ZOOM_K // 2)
                    cv2.drawMarker(zoom, zc, (255, 255, 255), cv2.MARKER_CROSS, 30, 1)
                    cv2.circle(zoom, zc, SNAP_R * ZOOM_K, (200, 200, 200), 1)       # 붙잡기 반경
                    zh, zw = zoom.shape[:2]
                    # 마우스와 대각선 반대쪽 모서리에 놓아서 마커를 가리지 않게
                    px0 = 8 if mo[1] > W // 2 else W - zw - 8
                    py0 = 30 if mo[2] > H // 2 else H - zh - 8
                    img[py0:py0 + zh, px0:px0 + zw] = zoom
                    cv2.rectangle(img, (px0 - 1, py0 - 1), (px0 + zw, py0 + zh), (255, 255, 255), 1)
                cv2.rectangle(img, (0, 0), (W, 22), (0, 0, 0), -1)
                put(img, f"{c.label}  {c.fps_val:4.1f}fps  {'MASK: ' + sel_name if state['mask'] else ''}",
                    (6, 16), 0.5)
                if state["frozen"]:
                    put(img, "FROZEN (SPACE)", (W - 170, 16), 0.5, (0, 200, 255), 1)
                y = H - 10 - 20 * len(found_txt)
                put(img, f"{sel_name} samples: {len(samples[c.serial][sel_name])}px / "
                         f"{clicks[c.serial][sel_name]} clicks", (8, y), 0.5, (255, 255, 255))
                y += 20
                for name, t, col in found_txt:
                    put(img, f"{name:7s} {t}", (8, y), 0.5, col)
                    y += 20
                cv2.line(img, (W - 1, 0), (W - 1, H), (90, 90, 90), 1)
                tiles.append(img)

            bar = np.full((BAR_H, total_w, 3), 30, np.uint8)
            x = 8
            for i, (name, count) in enumerate(markers):
                r0 = next((ranges[c.serial][name] for c in cams if ranges[c.serial].get(name)), None)
                col = range_bgr(r0)
                regs = sum(1 for c in cams if ranges[c.serial].get(name))
                txt = f"{i + 1}:{name} x{count} [{regs}/{len(cams)}cam]"
                tw = cv2.getTextSize(txt, FONT, 0.5, 1)[0][0]
                if i == state["sel"]:
                    cv2.rectangle(bar, (x - 4, 4), (x + tw + 4, 28), col, 2)
                put(bar, txt, (x, 22), 0.5, col)
                x += tw + 18
            if state["msg"] and time.time() - state["msg_t"] < 4:
                put(bar, state["msg"], (8, 52), 0.5, state["msg_col"])
            else:
                help_t = ("SPACE freeze | click marker (yellow = will sample) | [ ] count | U undo | "
                          "R reset | M mask | S save | Q quit")
                put(bar, help_t, (8, 52), 0.45, (170, 170, 170))
            if state["dirty"]:
                put(bar, "* unsaved", (total_w - 110, 52), 0.5, (0, 200, 255))
            ui = np.vstack([bar, np.hstack(tiles)])
            cv2.imshow(win, ui)

            k = cv2.waitKey(1) & 0xFF
            if k == 255:
                if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                    break
                continue
            ch = chr(k).lower() if k < 128 else ""
            if ch in "123456789" and ch and int(ch) <= len(markers):
                state["sel"] = int(ch) - 1
                print(f"→ {markers[state['sel']][0]}")
            elif ch == "[":
                markers[state["sel"]][1] = max(1, sel_count - 1)
                state["dirty"] = True
            elif ch == "]":
                markers[state["sel"]][1] = sel_count + 1
                state["dirty"] = True
            elif ch == "m":
                state["mask"] = not state["mask"]
            elif ch == " ":
                state["frozen"] = not state["frozen"]
                say("FROZEN - click markers calmly" if state["frozen"] else "LIVE",
                    "화면 정지 — 천천히 클릭하세요" if state["frozen"] else "실시간 화면", (0, 200, 255))
            elif ch == "u":
                if history:
                    s, name, n = history.pop()
                    del samples[s][name][-n:]
                    clicks[s][name] = max(0, clicks[s][name] - 1)
                    ranges[s][name] = ranges_from(samples[s][name]) if samples[s][name] else None
                    state["dirty"] = True
                    say(f"취소: {s} / {name} -{n}px")
            elif ch == "r":
                for c in cams:
                    samples[c.serial][sel_name] = []
                    clicks[c.serial][sel_name] = 0
                    ranges[c.serial][sel_name] = None
                history[:] = [h for h in history if h[1] != sel_name]
                state["dirty"] = True
                say(f"{sel_name}: 모든 카메라 샘플 초기화")
            elif ch == "s":
                for c in cams:
                    data = dict(old[c.serial])            # 목록에 없는 기존 항목은 유지
                    for name, count in markers:
                        r = ranges[c.serial].get(name)
                        if r:
                            data[name] = dict(r, count=int(count))
                        else:
                            data.pop(name, None)
                    p = save_hsv(c.serial, data, args.hsv_dir)
                    miss = [n for n, _ in markers if not ranges[c.serial].get(n)]
                    print(f"저장: {p}" + (f"   (미등록: {', '.join(miss)})" if miss else ""))
                    old[c.serial] = data
                state["dirty"] = False
            elif ch == "q" or k == 27:
                if state["dirty"] and not state["quit_armed"]:
                    state["quit_armed"] = True
                    say("저장 안 한 변경이 있습니다. 그냥 종료하려면 Q 를 한 번 더 누르세요 (저장은 S).")
                    continue
                break
            if ch != "q":
                state["quit_armed"] = False
            if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        for c in cams:
            c.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
