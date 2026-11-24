"""
depth_pose_test.py — RealSense depth + 12-keypoint YOLO Pose로 손가락 굽힘 지표의
안정성(반복성)을 측정하는 테스트 스크립트.

목적 (vision을 엔코더 map drift 보정용 '상대 안정 기준'으로 쓸 수 있는지 판정):
  1) 각 키포인트(M01~M12)의 3D 좌표 (x, y, z) [m] 산출 — depth align
  2) depth 유효율 — 손가락 키포인트에서 depth가 비는(hole) 비율
  3) 반복성 — 같은 자세를 가만히 들고 있을 때 굽힘 지표가 얼마나 떨리나(정적 분산)

굽힘 지표 두 가지를 동시에 계산해 비교:
  (ㄱ) joint_angle_sum : 손가락별 관절각(MCP+PIP+DIP) 합  [deg]
  (ㄴ) tip_dist_norm   : (손끝-손목 거리)/(손 크기)로 정규화한 굽힘  [무차원]

키포인트 매핑 (사용자 확인):
  M01 손목
  M02 엄지 MCP   M03 엄지 IP    M04 엄지 tip
  M05 검지 MCP   M06 검지 PIP   M07 검지 DIP   M08 검지 tip
  M09 중지 MCP   M10 중지 PIP   M11 중지 DIP   M12 중지 tip

조작:
  Q / Esc : 종료
  R       : 반복성 측정 시작 (REPEAT_SECONDS 동안 현재 자세 고정) → 콘솔에 통계
  S       : 현재 프레임 스냅샷 저장
  1       : depth 평균 패치 크기 토글 (단일 픽셀 ↔ 5x5 중앙값)
"""
import argparse
import time
import uuid
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
DEFAULT_MODEL = HERE / 'best.pt'
if not DEFAULT_MODEL.exists():
    DEFAULT_MODEL = HERE.parent / 'outputs/pose/session3_plus_session1_20261027_235755_f16724/training/weights/best.pt'

# ── 키포인트 인덱스 (0-based) ──
WRIST = 0
THUMB = dict(mcp=1, ip=2, tip=3)              # M02, M03, M04
INDEX = dict(mcp=4, pip=5, dip=6, tip=7)      # M05~M08
MIDDLE = dict(mcp=8, pip=9, dip=10, tip=11)   # M09~M12

# 관절각 정의: (관절, [점a, 점b(꼭짓점), 점c])  각 = ∠(a-b-c)
JOINT_ANGLES = {
    'thumb_MCP':  (WRIST,        THUMB['mcp'],  THUMB['ip']),
    'thumb_IP':   (THUMB['mcp'], THUMB['ip'],   THUMB['tip']),
    'index_MCP':  (WRIST,        INDEX['mcp'],  INDEX['pip']),
    'index_PIP':  (INDEX['mcp'], INDEX['pip'],  INDEX['dip']),
    'index_DIP':  (INDEX['pip'], INDEX['dip'],  INDEX['tip']),
    'middle_MCP': (WRIST,        MIDDLE['mcp'], MIDDLE['pip']),
    'middle_PIP': (MIDDLE['mcp'],MIDDLE['pip'], MIDDLE['dip']),
    'middle_DIP': (MIDDLE['pip'],MIDDLE['dip'], MIDDLE['tip']),
}
# 손가락별 지표를 묶는 관절 그룹
FINGER_JOINTS = {
    'thumb':  ['thumb_MCP', 'thumb_IP'],
    'index':  ['index_MCP', 'index_PIP', 'index_DIP'],
    'middle': ['middle_MCP', 'middle_PIP', 'middle_DIP'],
}
FINGER_TIP = {'thumb': THUMB['tip'], 'index': INDEX['tip'], 'middle': MIDDLE['tip']}
FINGER_MCP = {'thumb': THUMB['mcp'], 'index': INDEX['mcp'], 'middle': MIDDLE['mcp']}

REPEAT_SECONDS = 5.0   # 반복성 측정 지속 시간


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--weights', type=Path, default=DEFAULT_MODEL)
    p.add_argument('--serial', help='Camera serial (default: first found)')
    p.add_argument('--device', default='auto', help='auto, cpu, or CUDA index such as 0')
    p.add_argument('--conf', type=float, default=0.25)
    p.add_argument('--kpt-conf', type=float, default=0.5, help='이 미만 신뢰도의 키포인트는 3D 계산에서 제외')
    p.add_argument('--imgsz', type=int, default=640)
    p.add_argument('--fps', type=int, default=15)
    p.add_argument('--output', type=Path, default=HERE / 'depth_test_out')
    return p.parse_args()


def angle_3pt(a, b, c):
    """점 a, c를 b(꼭짓점)에서 본 사잇각 [deg]. 유효하지 않으면 nan."""
    if a is None or b is None or c is None:
        return np.nan
    v1, v2 = a - b, c - b
    n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
    if n1 < 1e-6 or n2 < 1e-6:
        return np.nan
    cosv = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
    return float(np.degrees(np.arccos(cosv)))


def compute_metrics(pts3d):
    """pts3d: (12,3) ndarray, 유효하지 않은 점은 nan.
    반환: joint_angles{관절:deg}, finger_anglesum{손가락:deg}, finger_tipdist{손가락:무차원}"""
    def get(i):
        p = pts3d[i]
        return None if not np.all(np.isfinite(p)) else p

    joint_angles = {}
    for name, (ia, ib, ic) in JOINT_ANGLES.items():
        joint_angles[name] = angle_3pt(get(ia), get(ib), get(ic))

    finger_anglesum = {}
    for finger, joints in FINGER_JOINTS.items():
        vals = [joint_angles[j] for j in joints]
        finger_anglesum[finger] = float(np.sum(vals)) if np.all(np.isfinite(vals)) else np.nan

    # 손 크기 스케일: 손목-중지MCP 거리 (주먹을 쥐어도 거의 안 변하는 기준)
    wrist = get(WRIST)
    mmcp = get(MIDDLE['mcp'])
    hand_scale = np.linalg.norm(wrist - mmcp) if (wrist is not None and mmcp is not None) else np.nan

    finger_tipdist = {}
    for finger in FINGER_JOINTS:
        tip = get(FINGER_TIP[finger])
        if tip is None or wrist is None or not np.isfinite(hand_scale) or hand_scale < 1e-4:
            finger_tipdist[finger] = np.nan
        else:
            # 손끝이 손목에 가까울수록(굽힐수록) 값이 작아짐 → 1에서 빼서 '굽힘이 클수록 큰 값'
            finger_tipdist[finger] = 1.0 - np.linalg.norm(tip - wrist) / hand_scale
    return joint_angles, finger_anglesum, finger_tipdist


def deproject(rs, intr, px, py, depth_m):
    """픽셀 + depth → 카메라 좌표계 3D [m]."""
    return np.array(rs.rs2_deproject_pixel_to_point(intr, [float(px), float(py)], float(depth_m)))


def sample_depth(depth_frame, px, py, w, h, patch):
    """키포인트 픽셀의 depth [m]. patch>1이면 주변 창의 0 아닌 값들의 중앙값(hole 완화)."""
    px, py = int(round(px)), int(round(py))
    if not (0 <= px < w and 0 <= py < h):
        return 0.0
    if patch <= 1:
        return depth_frame.get_distance(px, py)
    half = patch // 2
    vals = []
    for yy in range(max(0, py - half), min(h, py + half + 1)):
        for xx in range(max(0, px - half), min(w, px + half + 1)):
            d = depth_frame.get_distance(xx, yy)
            if d > 0:
                vals.append(d)
    return float(np.median(vals)) if vals else 0.0


def main():
    args = parse_args()
    import cv2
    import pyrealsense2 as rs
    import torch
    from ultralytics import YOLO

    if not args.weights.is_file():
        raise SystemExit(f'Model not found: {args.weights}')
    device = ('0' if torch.cuda.is_available() else 'cpu') if args.device == 'auto' else args.device
    print('Weights:', args.weights.resolve())
    print('Device :', device)
    model = YOLO(str(args.weights))
    shape = model.model.yaml.get('kpt_shape')
    if list(shape or []) != [12, 3]:
        raise SystemExit(f'Expected 12-point pose model, got {shape}')

    # ── 카메라: color + depth, depth를 color에 정렬 ──
    ctx = rs.context()
    devs = [d.get_info(rs.camera_info.serial_number) for d in ctx.query_devices()
            if 'Platform Camera' not in d.get_info(rs.camera_info.name)]
    if not devs:
        raise SystemExit('No RealSense camera found.')
    serial = args.serial or devs[0]
    print('Camera :', serial)

    pipeline = rs.pipeline(ctx)
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, args.fps)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, args.fps)
    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)  # depth를 color 프레임에 정렬
    color_intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

    # depth 후처리(hole 완화) — 가벼운 것만
    spatial = rs.spatial_filter()
    temporal = rs.temporal_filter()

    def infer(img):
        return model.predict(source=img, imgsz=args.imgsz, conf=args.conf,
                             iou=0.7, device=device, verbose=False, save=False)[0]

    patch = 5  # depth 샘플 패치(5x5 중앙값). '1' 키로 토글
    rec = None  # 반복성 측정 상태
    trials = []  # 완료된 측정들 — 착용 간 재현성 계산용

    args.output.mkdir(parents=True, exist_ok=True)
    session_dir = args.output / datetime.now().strftime('session_%Y%m%d_%H%M%S')
    session_dir.mkdir(parents=True, exist_ok=True)
    print(f'저장 위치: {session_dir}')
    cv2.namedWindow('depth pose test', cv2.WINDOW_NORMAL)
    print('R: 측정 시작(5s) | S: 스냅샷 | 1: patch 토글 | Q/Esc: 종료 후 요약')

    try:
        while True:
            frames = align.process(pipeline.wait_for_frames())
            cframe = frames.get_color_frame()
            dframe = frames.get_depth_frame()
            if not cframe or not dframe:
                continue
            dframe = spatial.process(dframe)
            dframe = temporal.process(dframe).as_depth_frame()
            color = np.asanyarray(cframe.get_data())
            h, w = color.shape[:2]

            res = infer(color)
            canvas = color.copy()

            pts3d = np.full((12, 3), np.nan)
            n_valid_kpt = 0
            n_depth_hole = 0

            if res.boxes is not None and len(res.boxes):
                cand = [i for i, c in enumerate(res.boxes.cls.cpu().numpy()) if int(c) == 0]
                if cand:
                    i = max(cand, key=lambda j: float(res.boxes.conf[j]))
                    if res.keypoints is not None:
                        pts2d = res.keypoints.xy[i].cpu().numpy()
                        kconf = res.keypoints.conf
                        kconf = kconf[i].cpu().numpy() if kconf is not None else np.ones(len(pts2d))
                        for k, ((px, py), sc) in enumerate(zip(pts2d, kconf)):
                            if sc < args.kpt_conf or (px == 0 and py == 0):
                                continue
                            n_valid_kpt += 1
                            d = sample_depth(dframe, px, py, w, h, patch)
                            if d <= 0:
                                n_depth_hole += 1
                                cv2.circle(canvas, (int(px), int(py)), 4, (0, 0, 255), -1)  # 빨강 = depth 없음
                                continue
                            pts3d[k] = deproject(rs, color_intr, px, py, d)
                            cv2.circle(canvas, (int(px), int(py)), 4, (0, 255, 255), -1)
                            cv2.putText(canvas, f'M{k+1:02} {d*100:.0f}cm', (int(px)+5, int(py)-5),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)

            joint_angles, anglesum, tipdist = compute_metrics(pts3d)

            # ── 화면 오버레이 ──
            cv2.rectangle(canvas, (0, 0), (w, 92), (0, 0, 0), -1)
            depth_ok = n_valid_kpt - n_depth_hole
            cv2.putText(canvas, f'kpts {n_valid_kpt}/12  depth_ok {depth_ok}/{n_valid_kpt}  patch {patch}',
                       (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y = 36
            for finger, keys in (('thumb', ['thumb_MCP', 'thumb_IP']),
                                 ('index', ['index_MCP', 'index_PIP', 'index_DIP']),
                                 ('middle', ['middle_MCP', 'middle_PIP', 'middle_DIP'])):
                parts = ' '.join(f'{joint_angles[k]:5.0f}' if np.isfinite(joint_angles[k]) else '  ---'
                                 for k in keys)
                t = tipdist[finger]
                tt = f'{t:+.3f}' if np.isfinite(t) else ' ---'
                cv2.putText(canvas, f'{finger:6s} {parts}  tip {tt}', (8, y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 120), 1)
                y += 18
            cv2.putText(canvas, '(각도 = MCP PIP DIP 순, 180=편 상태)', (8, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.38, (160, 160, 160), 1)

            # ── 반복성 측정 ──
            if rec is not None:
                rec['pts3d'].append(pts3d.copy())   # (12,3) 카메라 좌표계 [m]
                rec['joints'].append([joint_angles[j] for j in JOINT_ANGLES])
                rec['anglesum'].append([anglesum[f] for f in ('thumb', 'index', 'middle')])
                rec['tipdist'].append([tipdist[f] for f in ('thumb', 'index', 'middle')])
                rec['holes'].append(n_depth_hole)
                elapsed = time.monotonic() - rec['t0']
                cv2.putText(canvas, f'REC {elapsed:.1f}/{REPEAT_SECONDS:.0f}s  자세 고정',
                           (8, h-12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                if elapsed >= REPEAT_SECONDS:
                    summary = report_repeatability(rec, session_dir, len(trials) + 1)
                    trials.append(summary)
                    rec = None

            cv2.imshow('depth pose test', canvas)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            if key == ord('r') and rec is None:
                rec = dict(t0=time.monotonic(), pts3d=[], joints=[],
                           anglesum=[], tipdist=[], holes=[])
                print(f'\n[REC #{len(trials)+1}] {REPEAT_SECONDS:.0f}초간 자세 고정하세요...')
            if key == ord('1'):
                patch = 1 if patch > 1 else 5
            if key == ord('s'):
                dest = args.output / (datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:6] + '.jpg')
                cv2.imwrite(str(dest), canvas)
                print('snapshot:', dest)
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        if trials:
            report_across_trials(trials)
            print(f'모든 측정 CSV: {session_dir}')


def report_repeatability(rec, session_dir, trial_no):
    """측정 1회 요약: 관절각 8개 + 손가락 지표의 mean/std를 콘솔 출력 + CSV 저장.
    std가 작을수록 drift 보정 기준으로 안정적. 반환값은 착용 간 재현성 계산용 요약."""
    import csv as _csv
    import numpy as np
    fingers = ('thumb', 'index', 'middle')
    joints = list(JOINT_ANGLES.keys())
    ja = np.array(rec['joints'], dtype=float)      # (N,8) 개별 관절각
    ang = np.array(rec['anglesum'], dtype=float)   # (N,3)
    tip = np.array(rec['tipdist'], dtype=float)
    n = len(ang)

    def ms(col):
        v = col[np.isfinite(col)]
        return (np.mean(v), np.std(v), 100 * len(v) / n if n else 0) if len(v) else (np.nan, np.nan, 0)

    print('\n' + '=' * 72)
    print(f'[측정 #{trial_no}]  N={n} frames, {REPEAT_SECONDS:.0f}s 고정 자세')
    print('-' * 72)
    print('■ 개별 관절각 [deg]  (180=편 상태, 작을수록 굽음)')
    print(f'  {"joint":12s} {"mean":>8s} {"std":>7s} {"valid%":>7s}')
    joint_summary = {}
    for j, name in enumerate(joints):
        m, s, p = ms(ja[:, j])
        joint_summary[name] = m
        print(f'  {name:12s} {m:8.2f} {s:7.3f} {p:6.0f}%')
    print('-' * 72)
    print('■ 손가락 지표')
    print(f'  {"finger":8s} | {"anglesum[deg]":>24s} | {"tipdist":>24s}')
    print(f'  {"":8s} | {"mean":>8s} {"std":>7s} {"valid%":>6s} | {"mean":>9s} {"std":>8s} {"valid%":>4s}')
    fsum = {}
    for j, f in enumerate(fingers):
        am, asd, ap = ms(ang[:, j])
        tm, tsd, tp = ms(tip[:, j])
        fsum[f] = (am, tm)
        print(f'  {f:8s} | {am:8.2f} {asd:7.3f} {ap:5.0f}% | {tm:9.4f} {tsd:8.5f} {tp:4.0f}%')
    holes = np.array(rec['holes'], dtype=float)
    print('-' * 72)
    print(f'depth hole 평균 {np.mean(holes):.2f}개/프레임, 최대 {int(np.max(holes))}개')

    # ── CSV 저장: 프레임별 raw 데이터 (3D 좌표 포함) ──
    pts = np.array(rec['pts3d'], dtype=float)   # (N,12,3) 카메라 좌표계 [m]
    kpt_cols = [f'M{k+1:02d}_{ax}' for k in range(12) for ax in ('x', 'y', 'z')]
    path = session_dir / f'trial{trial_no:02d}_raw.csv'
    with open(path, 'w', newline='', encoding='utf-8-sig') as fp:
        wtr = _csv.writer(fp)
        wtr.writerow(['frame'] + kpt_cols + joints
                     + [f'{f}_anglesum' for f in fingers]
                     + [f'{f}_tipdist' for f in fingers] + ['depth_holes'])
        for i in range(n):
            wtr.writerow([i] + list(pts[i].ravel()) + list(ja[i])
                         + list(ang[i]) + list(tip[i]) + [rec['holes'][i]])
    print(f'저장: {path.name}  (3D 좌표 12점 × xyz 포함)')
    print('=' * 72 + '\n')
    return dict(trial=trial_no, joints=joint_summary, fingers=fsum)


def report_across_trials(trials):
    """측정 간(= 자세/착용을 바꾼 경우) 평균값이 얼마나 흩어지는지.
    같은 자세·다른 착용으로 측정했다면 이 흩어짐이 '착용 간 재현성'이다."""
    import numpy as np
    if len(trials) < 2:
        return
    joints = list(JOINT_ANGLES.keys())
    print('\n' + '#' * 72)
    print(f'측정 간 비교  ({len(trials)}회)')
    print('  같은 자세 · 착용만 바꿔 측정했다면 → 아래 spread = 착용 간 재현성')
    print('  자세를 바꿔 측정했다면 → 아래 spread = 그 지표가 자세 변화를 잡아내는 정도')
    print('-' * 72)
    headers = ' '.join('{:>8s}'.format('#%d' % t['trial']) for t in trials)
    print('  {:12s} '.format('joint') + headers + ' {:>8s}'.format('spread'))
    for name in joints:
        vals = [t['joints'].get(name, np.nan) for t in trials]
        v = np.array(vals, dtype=float)
        vv = v[np.isfinite(v)]
        spread = (np.max(vv) - np.min(vv)) if len(vv) >= 2 else np.nan
        print(f'  {name:12s} ' + ' '.join(f'{x:8.2f}' for x in v) + f' {spread:8.2f}')
    print('#' * 72 + '\n')


if __name__ == '__main__':
    main()