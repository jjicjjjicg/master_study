"""
marker_tune.py — 색 마커의 HSV 범위를 현장 조명에서 잡는다

조명·카메라 화이트밸런스에 따라 같은 스티커도 HSV 값이 달라지므로,
실제 촬영 환경에서 한 번 잡아 json 으로 저장한다. marker_3d.py 가 이를 읽는다.

사용:
  python marker_tune.py --serial <SN>

  1~4로 부위를 선택하고 검은 배경 위에 올린 마커 하나를 화면 중앙 사각형 안에 둔다.
  A를 누르면 마커 색을 읽어 해당 부위의 HSV 범위를 갱신한다.
  필요한 경우 마커를 클릭해 해당 지점 주변의 색을 추가로 샘플링한다.
  원본 화면에는 검출된 마커의 카메라 기준 3D 좌표를 mm로 표시한다.

  1~4 : 부위 선택 (1 손목, 2 엄지, 3 검지, 4 중지)
  A   : 중앙 사각형에서 현재 부위 마커 색 자동 등록
  R   : 현재 색의 샘플 초기화
  M   : 마스크 보기 ↔ 원본 보기 전환
  S   : 저장하고 종료
  Q   : 저장 없이 종료

팁: 마스크 보기(M)에서 해당 색 마커만 하얗게 남고 나머지는 검게 보여야 한다.
    손이나 배경이 하얗게 섞이면 그 색은 범위가 너무 넓은 것 — R 로 지우고 다시.
"""
import argparse
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent

# 부위별 색 배정 — 색상환에서 서로 30도 이상 떨어뜨려 혼동을 막는다.
# 빨강·주황 계열은 구멍으로 드러난 피부와 겹치므로 쓰지 않는다.
PARTS = [
    ('wrist',  '손목',  'cyan',    (90, 200, 120)),
    ('thumb',  '엄지',  'green',   (60, 200, 120)),
    ('index',  '검지',  'blue',    (120, 200, 120)),
    ('middle', '중지',  'magenta', (165, 200, 120)),
]
SAMPLE_RADIUS = 3          # 클릭 지점 주변 이 반경의 픽셀을 샘플로 쓴다
AUTO_ROI = (160, 120, 480, 360)
AUTO_MIN_AREA = 40
AUTO_MIN_SAT, AUTO_MIN_VAL = 65, 45
INDEX_MIN_SAT = 20         # 연보라 등록 시 흰색에 가까운 픽셀을 제외한다
HUE_MARGIN = 3             # 샘플 색상 범위 양쪽에 더해줄 여유
SAT_MARGIN, VAL_MARGIN = 60, 60


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--serial', help='카메라 시리얼 (생략 시 첫 번째)')
    p.add_argument('--fps', type=int, default=15)
    p.add_argument('--out', type=Path, default=HERE / 'marker_colors.json')
    return p.parse_args()


def ranges_from(samples, automatic=False, min_sat=0):
    """샘플 HSV 픽셀들 → inRange 에 쓸 lower/upper. 색상 경계(0/180)를 넘는 경우 처리."""
    if len(samples) < 5:
        return None
    s = np.array(samples)
    h = s[:, 0].astype(float)
    # 색상이 0/180 경계를 넘나들면 평균이 엉뚱해진다 → 원형 평균으로 중심을 잡는다
    ang = np.radians(h * 2)
    cx, cy = np.cos(ang).mean(), np.sin(ang).mean()
    center = (np.degrees(np.arctan2(cy, cx)) % 360) / 2
    dev = ((h - center + 90) % 180) - 90       # 중심 기준 부호 있는 편차
    percentiles = (5, 95) if automatic else (0, 100)
    lower, upper = np.percentile(np.column_stack((dev, s[:, 1:])),
                                 percentiles, axis=0)
    hlo = center + lower[0] - HUE_MARGIN
    hhi = center + upper[0] + HUE_MARGIN
    slo = max(min_sat, int(lower[1]) - SAT_MARGIN)
    shi = min(255, int(upper[1]) + SAT_MARGIN)
    if slo > upper[1]:
        return None
    vlo = max(AUTO_MIN_VAL if automatic else 0, int(lower[2]) - VAL_MARGIN)
    vhi = min(255, int(upper[2]) + VAL_MARGIN)
    return dict(h=[float(hlo), float(hhi)], s=[slo, shi], v=[vlo, vhi],
                n_samples=len(samples))


def auto_samples(cv2, hsv, min_sat=AUTO_MIN_SAT):
    x0, y0, x1, y1 = AUTO_ROI
    roi = hsv[y0:y1, x0:x1]
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.inRange(roi, (0, min_sat, AUTO_MIN_VAL), (179, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    candidates = [i for i in range(1, count)
                  if stats[i, cv2.CC_STAT_AREA] >= AUTO_MIN_AREA]
    if len(candidates) != 1:
        return []
    component = (labels == candidates[0]).astype(np.uint8)
    interior = cv2.erode(component, kernel, borderType=cv2.BORDER_CONSTANT,
                         borderValue=0)
    return roi[interior != 0].tolist()


def make_mask(cv2, hsv, rng):
    """색상 경계를 넘는 범위도 올바르게 처리하는 마스크."""
    hlo, hhi = rng['h']
    s, v = rng['s'], rng['v']
    lo2, hi2 = (s[0], v[0]), (s[1], v[1])
    if hlo < 0 or hhi > 179:
        a = cv2.inRange(hsv, (int(hlo % 180), lo2[0], lo2[1]), (179, hi2[0], hi2[1]))
        b = cv2.inRange(hsv, (0, lo2[0], lo2[1]), (int(hhi % 180), hi2[0], hi2[1]))
        return cv2.bitwise_or(a, b)
    return cv2.inRange(hsv, (int(hlo), lo2[0], lo2[1]), (int(hhi), hi2[0], hi2[1]))


def main():
    args = parse_args()
    import cv2
    import pyrealsense2 as rs
    from marker_3d import backproject, blob_depth

    ctx = rs.context()
    devs = [d.get_info(rs.camera_info.serial_number) for d in ctx.query_devices()
            if 'Platform Camera' not in d.get_info(rs.camera_info.name)]
    if not devs:
        raise SystemExit('RealSense 카메라를 찾지 못했습니다.')
    serial = args.serial or devs[0]
    pipe = rs.pipeline(ctx)
    cfg = rs.config()
    cfg.enable_device(serial)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, args.fps)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, args.fps)
    profile = pipe.start(cfg)
    align = rs.align(rs.stream.color)
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    camera_matrix = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
    distortion = np.asarray(intr.coeffs)
    kernel = np.ones((3, 3), np.uint8)
    print('카메라:', serial)

    samples = {k: [] for k, _, _, _ in PARTS}
    automatic = set()
    # 기존 파일이 있으면 이어서 다듬을 수 있게 불러온다
    if args.out.is_file():
        try:
            old = json.load(open(args.out, encoding='utf-8'))
            print(f'기존 {args.out.name} 을 불러왔습니다 (덮어쓰려면 R 로 지우세요)')
        except Exception:
            old = {}
    else:
        old = {}

    state = dict(idx=0, show_mask=False, hsv=None)
    measured = {}

    def current_range(key):
        return measured.get(key) or old.get(key)

    def on_mouse(event, x, y, flags, param):
        if (event != cv2.EVENT_LBUTTONDOWN or state['hsv'] is None
                or state['show_mask'] or y <= 56):
            return
        hsv = state['hsv']
        h, w = hsv.shape[:2]
        r = SAMPLE_RADIUS
        patch = hsv[max(0, y - r):min(h, y + r + 1),
                    max(0, x - r):min(w, x + r + 1)].reshape(-1, 3)
        key = PARTS[state['idx']][0]
        samples[key].extend(patch.tolist())
        min_sat = INDEX_MIN_SAT if key == 'index' else (
            AUTO_MIN_SAT if key in automatic else 0)
        measured[key] = ranges_from(samples[key], key in automatic, min_sat)
        print(f"  {PARTS[state['idx']][1]}: 샘플 {len(samples[key])}개")

    cv2.namedWindow('marker tune', cv2.WINDOW_NORMAL)
    cv2.setMouseCallback('marker tune', on_mouse)
    print('\n검은 배경 위 마커 하나를 중앙 사각형에 두고 A를 누르세요.')
    print('1~4 부위 선택, A 자동 등록, M 마스크, R 초기화, S 저장, Q 취소')
    for i, (_, ko, _, _) in enumerate(PARTS, 1):
        print(f'  {i}: {ko}')

    try:
        while True:
            frames = align.process(pipe.wait_for_frames())
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            img = np.asanyarray(color_frame.get_data())
            depth_frame = frames.get_depth_frame()
            depth = np.asanyarray(depth_frame.get_data()) if depth_frame else None
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            state['hsv'] = hsv
            key_name, ko, _, _ = PARTS[state['idx']]
            rng = current_range(key_name)

            if state['show_mask'] and rng:
                m = make_mask(cv2, hsv, rng)
                m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel)
                disp = cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)
            else:
                disp = img.copy()
                if rng:
                    m = make_mask(cv2, hsv, rng)
                    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel)
                    n, labels, stats, cent = cv2.connectedComponentsWithStats(m)
                    for j in range(1, n):
                        if stats[j, cv2.CC_STAT_AREA] < 12:
                            continue
                        x, y = map(int, cent[j])
                        cv2.circle(disp, (x, y), 6, (255, 255, 255), 1)
                        z = blob_depth(cv2, depth, labels, j, depth_scale, kernel) \
                            if depth is not None else None
                        if z is None:
                            label = f'{key_name}: no depth'
                        else:
                            X, Y, Z = backproject(cv2, camera_matrix, distortion, cent[j], z)
                            label = f'{key_name}: ({X:.0f}, {Y:.0f}, {Z:.0f}) mm'
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        width = cv2.getTextSize(label, font, 0.42, 1)[0][0]
                        origin = (min(x + 8, disp.shape[1] - width - 4), max(y - 8, 70))
                        cv2.putText(disp, label, origin, font, 0.42, (0, 0, 0), 3)
                        cv2.putText(disp, label, origin, font, 0.42, (255, 255, 255), 1)
                x0, y0, x1, y1 = AUTO_ROI
                cv2.rectangle(disp, (x0, y0), (x1 - 1, y1 - 1), (0, 255, 255), 1)

            cv2.rectangle(disp, (0, 0), (640, 56), (0, 0, 0), -1)
            status = f'{rng["n_samples"]} pixels' if rng and 'n_samples' in rng else (
                'loaded' if rng else 'unregistered')
            txt = f'[{state["idx"]+1}] {key_name}  {status}'
            cv2.putText(disp, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            sub = '1-4 Part | A Auto | M View | R Reset | S Save | Q Quit'
            cv2.putText(disp, sub, (8, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.imshow('marker tune', disp)

            k = ord(chr(cv2.waitKey(1) & 0xFF).lower())
            if k == ord('q'):
                print('저장하지 않고 종료')
                return
            if k in (ord('1'), ord('2'), ord('3'), ord('4')):
                state['idx'] = k - ord('1')
                print(f'→ {PARTS[state["idx"]][1]}')
            if k == ord('m'):
                state['show_mask'] = not state['show_mask']
            if k == ord('a'):
                min_sat = INDEX_MIN_SAT if key_name == 'index' else AUTO_MIN_SAT
                pixels = auto_samples(cv2, hsv, min_sat)
                if len(pixels) < 5:
                    print(f'{ko}: 색을 읽지 못했습니다. 검은 배경 위 마커 하나를 사각형에 두세요.')
                else:
                    samples[key_name] = pixels
                    automatic.add(key_name)
                    measured[key_name] = ranges_from(pixels, automatic=True, min_sat=min_sat)
                    print(f'{ko}: 자동 등록 완료 ({len(pixels)} 픽셀)')
            if k == ord('r'):
                samples[key_name] = []
                automatic.discard(key_name)
                measured.pop(key_name, None)
                old.pop(key_name, None)
                print(f'{ko} 샘플 초기화')
            if k == ord('s'):
                out = {}
                for kk, kko, _, _ in PARTS:
                    r2 = current_range(kk)
                    if r2:
                        out[kk] = r2
                    else:
                        print(f'⚠ {kko}: 샘플이 없어 저장되지 않습니다')
                if not out:
                    print('등록된 색이 없습니다.')
                    continue
                json.dump(out, open(args.out, 'w', encoding='utf-8'), indent=2)
                print(f'저장: {args.out}  ({len(out)}/4 색)')
                return
    finally:
        pipe.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
