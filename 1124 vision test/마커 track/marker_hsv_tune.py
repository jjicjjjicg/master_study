"""
marker_hsv_tune.py — 카메라 여러 대를 동시에 띄워서, 카메라마다 마커 HSV 범위를 잡아 json 으로 저장.

같은 스티커라도 카메라(화이트밸런스/센서)와 보는 각도에 따라 HSV 가 달라지므로
카메라별로 따로 저장한다:  hsv_<serial>.json  (이 폴더)

사용:
    conda activate rs_env
    python marker_hsv_tune.py                                   # 연결된 RealSense 전부
    python marker_hsv_tune.py --markers wrist:1 thumb:3 index:4 middle:4
    python marker_hsv_tune.py --res 1280x720                    # 10mm 마커가 작게 보이면
    python marker_hsv_tune.py --max-depth 800                   # 이보다 먼 blob 무시 (mm, 기본 800)

순서:
    0) L 로 노출/화이트밸런스 고정 (자동이면 손을 움직일 때마다 색이 바뀜). 반드시 샘플 클릭 '전에'.
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
    M       : 마스크 보기 on/off   (흰색 = 채택된 마커, 회색 = 색은 맞지만 너무 멀거나 depth 없음)
    L       : 노출/게인/화이트밸런스 고정 <-> 자동 (마우스가 올라간 카메라만, 화면 밖이면 전부)
    , / .   : 노출 줄이기 / 늘리기 (고정 상태에서. 마우스 올린 카메라만, 화면 밖이면 전부)
    - / =   : 선택 마커의 색 거리 허용치(gate) 줄이기 / 늘리기
              줄이면 배경/다른 색이 덜 잡히고, 너무 줄이면 마커도 일부 빠짐
    F       : 전체화면 on/off (창 크기를 바꿔도 카메라 비율 유지, 남는 곳은 검은 여백)
    S       : 저장 (hsv_<serial>.json + 카메라 설정 cam_<serial>.json -> 트래킹 때 같은 설정으로 열림)
    Q / ESC : 종료 (저장 안 한 변경이 있으면 한 번 더 눌러야 종료)
"""

import argparse
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np

from marker_common import (DEFAULT_MARKERS, HERE, MAHA_GATE, FONT, RSCamera, core_pixels, cv2, detect_all, list_devices,
                           load_hsv, make_masks, parse_markers, put, range_bgr, ranges_from,
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
    ap.add_argument("--max-depth", type=float, default=800, help="이보다 먼(mm) 또는 depth 없는 blob 무시. 0 = 끔")
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
    gates = {n: next((float(old[c.serial][n]["gate"]) for c in cams
                      if "gate" in old[c.serial].get(n, {})), MAHA_GATE) for n, _ in markers}
    state = dict(sel=0, mask=False, dirty=False, quit_armed=False, frozen=False,
                 msg="", msg_col=(255, 255, 255), msg_t=0.0, mouse=None, flash=None)
    items = {}

    cam_st = {c.serial: c.color_settings() for c in cams}   # 화면 표시용 캐시 (매 프레임 조회하면 느림)
    for c in cams:
        print(f"  {c.label}: " + ("AUTO 노출" if cam_st[c.serial]["auto"] else "LOCK (cam_*.json 적용)"))

    def on_locked(cam, ok, msg):
        cam_st[cam.serial] = cam.color_settings()
        state["dirty"] = True
        say(msg, None, (80, 220, 80) if ok else (60, 60, 255))

    def target_cams():
        mo = state["mouse"]
        return [cams[mo[0]]] if mo else list(cams)

    def say(msg_en, msg_ko=None, col=(255, 255, 255)):
        print(msg_ko or msg_en)
        state["msg"], state["msg_col"], state["msg_t"] = msg_en, col, time.time()

    def on_mouse(ev, x, y, *_):
        # 화면(레터박스 포함) 좌표 → 원본 UI 좌표
        sc, ox, oy = view
        x, y = int((x - ox) / sc), int((y - oy) / sc)
        if y < BAR_H or y >= BAR_H + H or x < 0 or x // W >= len(cams):
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
        px_all = it["hsv"][y0:y0 + reg.shape[0], x0:x0 + reg.shape[1]][reg]
        px = core_pixels(px_all).tolist()           # 테두리/그림자/검은색 빼고 마커 몸통만
        if cam_st[cam.serial]["auto"]:
            say(f"{cam.model}: AUTO exposure - press L first! (colors drift)",
                f"{cam.model}: 자동 노출 상태 - 먼저 L 로 고정하세요 (색이 계속 바뀜)", (0, 140, 255))
        samples[cam.serial][name].extend(px)
        ranges[cam.serial][name] = ranges_from(samples[cam.serial][name])
        clicks[cam.serial][name] += 1
        history.append((cam.serial, name, len(px)))
        state["dirty"] = True
        state["flash"] = (ci, region_contours(reg, off), time.time())
        say(f"OK  {cam.model} / {name}: +{len(px)}/{len(px_all)}px core  (total {len(samples[cam.serial][name])}px, "
            f"{clicks[cam.serial][name]} clicks)",
            f"{cam.model} / {name}: +{len(px)}px (총 {len(samples[cam.serial][name])}px, {clicks[cam.serial][name]}번째 클릭)",
            (80, 220, 80))

    win = "marker HSV tune"
    # 비율은 직접 레터박스로 맞춘다 (WINDOW_KEEPRATIO 는 Windows 최대화/전체화면에서 늘어나 버림)
    cv2.namedWindow(win, cv2.WINDOW_NORMAL | cv2.WINDOW_FREERATIO)
    view = [1.0, 0, 0]     # (scale, offset_x, offset_y): 원본 UI → 창 화면
    total_w = W * len(cams)
    if total_w > 1800:
        cv2.resizeWindow(win, 1800, int((H + BAR_H) * 1800 / total_w))
    cv2.setMouseCallback(win, on_mouse)
    print(__doc__.split("키:")[1])

    pool = ThreadPoolExecutor(max_workers=len(cams))   # 카메라별 검출을 동시에 (numpy/OpenCV 는 GIL 을 풀어줌)
    try:
        while True:
            if not state["frozen"]:            # SPACE: 화면 정지 → 떨림 없이 클릭
                for c in cams:
                    it = c.get()
                    if it is not None:
                        items[c.serial] = it
            sel_name, sel_count = markers[state["sel"]]

            tiles = []
            def analyze(c):
                """카메라 한 대: 색 마스크(겹치는 색 정리) + 거리 안 blob. 3대를 스레드로 동시에."""
                it = items.get(c.serial)
                if it is None:
                    return None
                rngs = {n: dict(r, gate=gates[n]) for n, r in ranges[c.serial].items() if r}
                masks = make_masks(it["hsv"], rngs)      # 겹치는 색은 가장 가까운 색 하나에만
                cnt = dict(markers)
                return {n: detect_all(c, it, r, mask=masks[n], max_depth=args.max_depth, refine=cnt.get(n, 1) + 2)
                        for n, r in rngs.items()}

            analyses = list(pool.map(analyze, cams))
            for ci, c in enumerate(cams):
                it = items.get(c.serial)
                if it is None:
                    tiles.append(np.zeros((H, W, 3), np.uint8))
                    continue
                rngs = ranges[c.serial]
                img = np.zeros((H, W, 3), np.uint8) if state["mask"] else it["color"].copy()
                found_txt = []
                for name, count in markers:
                    r = rngs.get(name)
                    if not r:
                        found_txt.append((name, "--", (120, 120, 120)))
                        continue
                    all_d, dmask, lab = analyses[ci][name]
                    n_all = len(all_d)                         # 거리 안에서 이 색으로 잡히는 덩어리 전체 개수
                    dets = all_d[:count + 2]                   # 개수보다 많이 잡히는지도 보이게 +2
                    if state["mask"] and name == sel_name:
                        img[dmask > 0] = (70, 70, 70)          # 색은 맞지만 버려진 것 (멀거나 depth 없음/너무 작음)
                        if all_d:
                            keep = np.zeros(int(lab.max()) + 1, bool)
                            keep[[d["lab"] for d in all_d]] = True
                            img[keep[lab]] = (255, 255, 255)
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
                cst = cam_st[c.serial]
                if c.locking:
                    st_txt, st_col = "LOCKING... keep hand in view", (0, 200, 255)
                elif not cst["auto"]:
                    st_txt, st_col = (f"LOCK e{cst['exposure']:.0f} g{cst['gain']:.0f} "
                                      f"wb{cst['white_balance']:.0f}"), (80, 220, 80)
                else:
                    st_txt, st_col = "AUTO exposure (L to lock)", (0, 140, 255)
                put(img, st_txt + f"  V{it['hsv'][::8, ::8, 2].mean():.0f}  gate {gates[sel_name]:.0f}",
                    (6, 40), 0.45, st_col)
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
                          "R reset | M mask | L lock | ,. exp | -= gate | F full | S save | Q quit")
                put(bar, help_t, (8, 52), 0.45, (170, 170, 170))
            if state["dirty"]:
                put(bar, "* unsaved", (total_w - 110, 52), 0.5, (0, 200, 255))
            ui = np.vstack([bar, np.hstack(tiles)])
            uh, uw = ui.shape[:2]
            try:
                _, _, ww, wh = cv2.getWindowImageRect(win)
            except cv2.error:
                ww, wh = uw, uh
            if ww > 0 and wh > 0 and (ww, wh) != (uw, uh):
                sc = min(ww / uw, wh / uh)
                nw, nh = max(1, int(uw * sc)), max(1, int(uh * sc))
                ox, oy = (ww - nw) // 2, (wh - nh) // 2
                canvas = np.zeros((wh, ww, 3), np.uint8)
                canvas[oy:oy + nh, ox:ox + nw] = cv2.resize(
                    ui, (nw, nh), interpolation=cv2.INTER_AREA if sc < 1 else cv2.INTER_LINEAR)
                view[:] = [sc, ox, oy]
                cv2.imshow(win, canvas)
            else:
                view[:] = [1.0, 0, 0]
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
            elif ch == "l":
                tgt = target_cams()
                if any(c.locking for c in tgt):
                    continue
                if all(not cam_st[c.serial]["auto"] for c in tgt):
                    for c in tgt:
                        c.set_auto()
                        cam_st[c.serial] = c.color_settings()
                    state["dirty"] = True
                    say("AUTO exposure", "자동 노출/화이트밸런스로 되돌림", (0, 140, 255))
                else:
                    if any(samples[c.serial][n] for c in tgt for n, _ in markers):
                        say("Locked after sampling - consider R + re-click",
                            "샘플을 찍은 뒤에 고정함 -> 색이 달라졌을 수 있으니 R 로 지우고 다시 클릭 권장", (0, 140, 255))
                    else:
                        say("Locking... keep the hand in view (~5s)",
                            "고정 중... 손을 화면 안에 두고 몇 초 기다리세요", (0, 200, 255))
                    for c in tgt:
                        if cam_st[c.serial]["auto"]:
                            c.lock_color(on_locked)
            elif ch in (",", "."):
                for c in target_cams():
                    st = cam_st[c.serial]
                    if st["auto"] or c.locking:
                        continue
                    c.set_manual(exposure=max(1.0, st["exposure"] * (0.8 if ch == "," else 1.25)))
                    cam_st[c.serial] = c.color_settings()
                    state["dirty"] = True
                    e_max = 10000.0 / args.fps
                    if cam_st[c.serial]["exposure"] > e_max:
                        say(f"exposure > {e_max:.0f}: fps will drop",
                            f"노출이 {e_max:.0f} 보다 길면 fps 가 떨어짐", (0, 140, 255))
            elif ch in ("-", "="):
                gates[sel_name] = float(np.clip(gates[sel_name] * (0.75 if ch == "-" else 1.33), 2, 400))
                state["dirty"] = True
                say(f"{sel_name} gate {gates[sel_name]:.1f}")
            elif ch == "f":
                full = cv2.getWindowProperty(win, cv2.WND_PROP_FULLSCREEN) == cv2.WINDOW_FULLSCREEN
                cv2.setWindowProperty(win, cv2.WND_PROP_FULLSCREEN,
                                      cv2.WINDOW_NORMAL if full else cv2.WINDOW_FULLSCREEN)
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
                            data[name] = dict(r, count=int(count), gate=round(gates[name], 1))
                        else:
                            data.pop(name, None)
                    p = save_hsv(c.serial, data, args.hsv_dir)
                    miss = [n for n, _ in markers if not ranges[c.serial].get(n)]
                    print(f"저장: {p}" + (f"   (미등록: {', '.join(miss)})" if miss else ""))
                    old[c.serial] = data
                    cp = c.save_settings()
                    print(f"저장: {cp}  ({'AUTO' if cam_st[c.serial]['auto'] else 'LOCK'})")
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
        pool.shutdown(wait=False)
        for c in cams:
            c.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
