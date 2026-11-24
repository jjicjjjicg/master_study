"""
marker_3d.py — 색 마커 기반 손 관절 3D 복원 (3단계 폴백 + 튐 억제)

관절 하나의 3D 좌표를 얻는 경로를 세 단계로 둔다. 가장 정확한 것부터 시도한다.

  1) tri   : 양쪽 뷰에서 모두 보임 → 삼각측량            (~1 mm)
  2) depth : 한쪽에서만 보임 → 그 카메라의 depth 로 3D 를 만들고
             캘리브레이션 R, T 로 cam1 좌표계로 변환      (~2~5 mm)
  3) bone  : 양쪽 다 안 보임 → 부모 관절에서 직전 방향으로
             뼈 길이만큼 뻗어 추정                        (측정값 아님)

CSV 의 <joint>_src 열에 경로가 기록되므로 나중에 구분해 분석할 수 있다.
depth 가 과거에 실패했던 이유(얇은 손가락에서 픽셀 하나가 빗나가 배경 깊이를 읽음)는
마커에서는 적용되지 않는다 — 덩어리 전체의 중앙값을 쓰고, 경계 번짐은 침식으로 제거한다.

튐 억제:
  - 속도 게이트 : 프레임 간 이동이 --max-jump 를 넘으면 그 관측을 버린다
  - 뼈 길이 게이트 : 부모와의 거리가 사전값에서 --bone-tol 이상 벗어나면 버린다
  - 시간 중앙값 : 최근 --smooth 프레임의 중앙값을 출력한다 (3 이면 1프레임 튐 제거)

준비:
  1) marker_tune.py → marker_colors.json
  2) 두 대: stereo_calibrate.py → stereo_calib.npz
     한 대: python marker_3d.py --single [--serials <SN>]
  3) --measure-bones 로 편 손에서 뼈 길이 실측 (사슬 정렬과 게이트의 기준)

조작: R 측정 | M 마스크 | Q 종료
"""
import argparse
import csv as _csv
import itertools
import json
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPEAT_SECONDS = 5.0
MIN_BLOB_AREA = 12

FINGERS = {
    'thumb':  ['MCP', 'IP', 'tip'],
    'index':  ['MCP', 'PIP', 'DIP', 'tip'],
    'middle': ['MCP', 'PIP', 'DIP', 'tip'],
}
DEFAULT_BONES = {'thumb': [34.0, 25.0],
                 'index': [45.0, 28.0, 22.0],
                 'middle': [48.0, 30.0, 23.0]}
ANGLE_NAMES = ['thumb_MCP', 'thumb_IP', 'index_MCP', 'index_PIP', 'index_DIP',
               'middle_MCP', 'middle_PIP', 'middle_DIP']
JOINT_KEYS = ['wrist'] + [f'{f}_{n}' for f, ns in FINGERS.items() for n in ns]


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--single', action='store_true', help='RealSense 한 대의 깊이로 3D 계산')
    p.add_argument('--calib', type=Path, help='두 카메라 사이의 캘리브레이션 파일')
    p.add_argument('--colors', type=Path, default=HERE / 'marker_colors.json')
    p.add_argument('--bones', type=Path, default=HERE / 'marker_bones.json')
    p.add_argument('--serials', nargs='+', help='두 대의 시리얼 또는 단일 모드에서 한 대의 시리얼')
    p.add_argument('--fps', type=int, default=30)
    p.add_argument('--max-reproj', type=float, default=4.0,
                   help='삼각측량 재투영 오차 상한 [px]')
    p.add_argument('--epi-tol', type=float, default=6.0,
                   help='두 뷰 짝짓기의 에피폴라 거리 허용값 [px]')
    p.add_argument('--max-jump', type=float, default=60.0,
                   help='프레임 간 관절 이동 상한 [mm] — 넘으면 튐으로 보고 버림')
    p.add_argument('--bone-tol', type=float, default=0.25,
                   help='부모와의 거리가 뼈 길이에서 이 비율 이상 벗어나면 버림')
    p.add_argument('--smooth', type=int, default=3,
                   help='출력에 쓸 시간 중앙값 창 크기 (1이면 끔)')
    p.add_argument('--no-depth', action='store_true', help='2단계(depth) 사용 안 함')
    p.add_argument('--no-bone-fill', action='store_true', help='3단계(역산) 사용 안 함')
    p.add_argument('--measure-bones', action='store_true')
    p.add_argument('--output', type=Path, default=HERE / 'marker_out')
    args = p.parse_args()
    if args.single:
        if args.no_depth:
            p.error('--single 모드에는 깊이 스트림이 필요합니다.')
        if args.serials and len(args.serials) != 1:
            p.error('--single 모드에서는 시리얼을 하나만 지정하세요.')
    elif args.calib is None or len(args.serials or []) != 2:
        p.error('두 대 모드에는 --calib 및 --serials <SN1> <SN2>가 필요합니다.')
    return args


def load_calib(path):
    if not path.is_file():
        raise SystemExit(f'캘리브레이션 파일 없음: {path}')
    d = np.load(path) if path.suffix == '.npz' else json.load(open(path, encoding='utf-8'))
    g = lambda k: np.asarray(d[k], dtype=float)
    K1, D1, K2, D2 = g('K1'), g('D1'), g('K2'), g('D2')
    R, T = g('R'), g('T').reshape(3, 1)
    P1 = K1 @ np.hstack([np.eye(3), np.zeros((3, 1))])
    P2 = K2 @ np.hstack([R, T])
    # 에피폴라 매칭용 기본행렬 — cam1 의 점 → cam2 의 직선
    Tx = np.array([[0, -T[2, 0], T[1, 0]],
                   [T[2, 0], 0, -T[0, 0]],
                   [-T[1, 0], T[0, 0], 0]])
    F = np.linalg.inv(K2).T @ (Tx @ R) @ np.linalg.inv(K1)
    return dict(K1=K1, D1=D1, K2=K2, D2=D2, R=R, T=T, P1=P1, P2=P2, F=F)


def make_mask(cv2, hsv, rng, kernel):
    hlo, hhi = rng['h']
    s, v = rng['s'], rng['v']
    if hlo < 0 or hhi > 179:
        a = cv2.inRange(hsv, (int(hlo % 180), s[0], v[0]), (179, s[1], v[1]))
        b = cv2.inRange(hsv, (0, s[0], v[0]), (int(hhi % 180), s[1], v[1]))
        m = cv2.bitwise_or(a, b)
    else:
        m = cv2.inRange(hsv, (int(hlo), s[0], v[0]), (int(hhi), s[1], v[1]))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel)


def find_blobs(cv2, hsv, rng, kernel, want):
    """색 범위의 덩어리 중심과 라벨맵. 면적 큰 순 want 개."""
    m = make_mask(cv2, hsv, rng, kernel)
    n, lab, stats, cent = cv2.connectedComponentsWithStats(m)
    cand = [(stats[i, cv2.CC_STAT_AREA], i) for i in range(1, n)
            if stats[i, cv2.CC_STAT_AREA] >= MIN_BLOB_AREA]
    cand.sort(key=lambda x: -x[0])
    cand = cand[:want]
    pts = np.array([cent[i] for _, i in cand], dtype=float).reshape(-1, 2)
    ids = [i for _, i in cand]
    return pts, lab, ids, m


def blob_depth(cv2, depth_frame, lab, blob_id, scale, kernel):
    """덩어리 픽셀들의 depth 중앙값 [mm]. 경계 번짐을 피하려 한 겹 침식한다."""
    sel = (lab == blob_id).astype(np.uint8)
    eroded = cv2.erode(sel, kernel)
    if eroded.sum() < 4:
        eroded = sel                      # 너무 작으면 침식 생략
    ys, xs = np.nonzero(eroded)
    d = depth_frame[ys, xs].astype(float)
    d = d[d > 0]
    if len(d) < 4:
        return None
    return float(np.median(d)) * scale * 1000.0   # m → mm


def backproject(cv2, K, D, px, Z):
    """픽셀 + 거리 → 그 카메라 좌표계의 3D [mm]."""
    u = cv2.undistortPoints(np.array(px, float).reshape(1, 1, 2), K, D).reshape(2)
    return np.array([u[0] * Z, u[1] * Z, Z])


def triangulate(cv2, cal, p1, p2):
    if len(p1) == 0:
        return np.zeros((0, 3)), np.zeros(0)
    u1 = cv2.undistortPoints(p1.reshape(-1, 1, 2), cal['K1'], cal['D1'],
                             P=cal['K1']).reshape(-1, 2)
    u2 = cv2.undistortPoints(p2.reshape(-1, 1, 2), cal['K2'], cal['D2'],
                             P=cal['K2']).reshape(-1, 2)
    X = cv2.triangulatePoints(cal['P1'], cal['P2'], u1.T, u2.T)
    X = (X[:3] / X[3]).T
    r1, _ = cv2.projectPoints(X, np.zeros(3), np.zeros(3), cal['K1'], cal['D1'])
    rv2, _ = cv2.Rodrigues(cal['R'])
    r2, _ = cv2.projectPoints(X, rv2, cal['T'], cal['K2'], cal['D2'])
    e = 0.5 * (np.linalg.norm(r1.reshape(-1, 2) - u1, axis=1)
               + np.linalg.norm(r2.reshape(-1, 2) - u2, axis=1))
    return X, e


def epipolar_match(cal, A, B, tol):
    """에피폴라 거리로 두 뷰의 덩어리를 짝짓는다 (전수 순열보다 훨씬 빠르고 정확).
    반환: [(a_idx, b_idx), ...], 짝 없는 a 와 b 의 인덱스."""
    from scipy.optimize import linear_sum_assignment
    if len(A) == 0 or len(B) == 0:
        return [], list(range(len(A))), list(range(len(B)))
    Ah = np.hstack([A, np.ones((len(A), 1))])
    lines = (cal['F'] @ Ah.T).T                      # cam2 에서의 에피폴라 직선
    nrm = np.linalg.norm(lines[:, :2], axis=1, keepdims=True)
    nrm[nrm < 1e-9] = 1e-9
    lines = lines / nrm
    Bh = np.hstack([B, np.ones((len(B), 1))])
    Dm = np.abs(lines @ Bh.T)                        # (len(A), len(B)) 점-직선 거리
    cost = Dm.copy()
    cost[cost > tol] = 1e6                           # 허용값 밖은 사실상 금지
    ri, ci = linear_sum_assignment(cost)
    pairs = [(int(a), int(b)) for a, b in zip(ri, ci) if Dm[a, b] <= tol]
    ua = [i for i in range(len(A)) if i not in {a for a, _ in pairs}]
    ub = [j for j in range(len(B)) if j not in {b for _, b in pairs}]
    return pairs, ua, ub


def order_chain(pts, wrist, prior):
    """마커들을 MCP→tip 순서로. 뼈 길이가 사전값에 가장 잘 맞는 배열을 고른다.
    주먹을 쥐면 손끝이 MCP보다 손목에 가까워지므로 거리순 정렬로는 뒤집힌다."""
    n = len(pts)
    if n != len(prior) + 1:
        return None
    best, bc = None, np.inf
    for perm in itertools.permutations(range(n)):
        P = pts[list(perm)]
        segs = np.linalg.norm(np.diff(P, axis=0), axis=1)
        c = float(np.sum((segs - np.array(prior)) ** 2))
        if wrist is not None:
            c += 0.3 * float(np.linalg.norm(P[0] - wrist) ** 2)
        if c < bc:
            best, bc = perm, c
    return list(best)


def compute_angles(J):
    def ang3(a, b, c):
        v1, v2 = a - b, c - b
        n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
        if n1 < 1e-9 or n2 < 1e-9:
            return np.nan
        return float(np.degrees(np.arccos(np.clip(np.dot(v1, v2) / (n1 * n2), -1, 1))))

    out = {k: np.nan for k in ANGLE_NAMES}
    g = lambda k: J.get(k)
    for f, names in FINGERS.items():
        P = [g(f'{f}_{n}') for n in names]
        for i in range(1, len(P) - 1):
            if P[i - 1] is not None and P[i] is not None and P[i + 1] is not None:
                out[f'{f}_{names[i]}'] = ang3(P[i - 1], P[i], P[i + 1])

    w, I, M = g('wrist'), g('index_MCP'), g('middle_MCP')
    if w is None or I is None or M is None:
        return out
    across = M - I
    na = np.linalg.norm(across)
    if na < 1e-9:
        return out
    across /= na
    along = 0.5 * (I + M) - w
    along = along - np.dot(along, across) * across
    nl = np.linalg.norm(along)
    if nl < 1e-9:
        return out
    along /= nl
    for f, names in FINGERS.items():
        a, b = g(f'{f}_{names[0]}'), g(f'{f}_{names[1]}')
        if a is None or b is None:
            continue
        seg = b - a
        ns = np.linalg.norm(seg)
        if ns < 1e-9:
            continue
        out[f'{f}_MCP'] = float(180.0 - np.degrees(
            np.arccos(np.clip(np.dot(seg / ns, along), -1, 1))))
    return out


class Tracker:
    """튐 억제: 속도 게이트 + 뼈 길이 게이트 + 시간 중앙값."""

    def __init__(self, args, prior):
        self.args = args
        self.prior = prior
        self.last = {}                       # 관절 → 직전에 채택된 3D
        self.hist = {k: deque(maxlen=max(1, args.smooth)) for k in JOINT_KEYS}
        self.parent = {}
        self.bone_of = {}
        for f, names in FINGERS.items():
            for i, n in enumerate(names):
                key = f'{f}_{n}'
                if i == 0:
                    self.parent[key] = 'wrist'
                    self.bone_of[key] = None     # 손목-MCP 는 길이가 일정치 않아 게이트 제외
                else:
                    self.parent[key] = f'{f}_{names[i-1]}'
                    self.bone_of[key] = prior[f][i - 1]

    def accept(self, key, X, accepted):
        """게이트 통과 여부. accepted 는 이번 프레임에서 이미 확정된 관절들."""
        if X is None or not np.all(np.isfinite(X)):
            return False
        prev = self.last.get(key)
        if prev is not None and np.linalg.norm(X - prev) > self.args.max_jump:
            return False
        L = self.bone_of.get(key)
        if L:
            par = accepted.get(self.parent[key])
            if par is not None:
                d = np.linalg.norm(X - par)
                if abs(d - L) > self.args.bone_tol * L:
                    return False
        return True

    def fill_from_bone(self, key, accepted):
        """양쪽 다 안 보일 때: 부모에서 직전 방향으로 뼈 길이만큼. 추정값."""
        L = self.bone_of.get(key)
        par = accepted.get(self.parent.get(key, ''))
        if not L or par is None:
            return None
        pv, cv_ = self.last.get(self.parent[key]), self.last.get(key)
        if pv is None or cv_ is None:
            return None
        d = cv_ - pv
        n = np.linalg.norm(d)
        if n < 1e-6:
            return None
        return par + d / n * L

    def commit(self, key, X):
        self.last[key] = X
        self.hist[key].append(X)

    def smoothed(self, key):
        h = self.hist[key]
        if not h:
            return None
        return np.median(np.array(h), axis=0)


def report(rec, session, trial_no, prior, timing):
    n = len(rec['angles'])
    A = np.array(rec['angles'], float)
    print('\n' + '=' * 76)
    print(f'[측정 #{trial_no}]  N={n} frames   유효 {np.mean(rec["nmark"]):.1f}/12 관절')
    print('-' * 76)
    print('■ 관절각 [deg] (180=편 상태)')
    print(f'  {"joint":12s} {"mean":>8s} {"std":>7s} {"valid%":>7s}')
    jsum = {}
    for j, nm in enumerate(ANGLE_NAMES):
        v = A[:, j][np.isfinite(A[:, j])]
        m, s = (np.mean(v), np.std(v)) if len(v) else (np.nan, np.nan)
        jsum[nm] = m
        print(f'  {nm:12s} {m:8.2f} {s:7.3f} {100*len(v)/max(n,1):6.0f}%')

    print('-' * 76)
    print('■ 좌표 출처별 비율 (tri=삼각측량, depth=단일뷰, bone=역산추정)')
    S = rec['src']
    for key in JOINT_KEYS:
        c = S[key]
        tot = sum(c.values()) or 1
        print(f'  {key:14s} tri {100*c["tri"]/tot:3.0f}%  '
              f'depth {100*c["depth"]/tot:3.0f}%  bone {100*c["bone"]/tot:3.0f}%  '
              f'none {100*c["none"]/tot:3.0f}%')

    print('-' * 76)
    print('■ 뼈 길이 [mm] — 자세와 무관하게 일정해야 한다')
    B = np.array(rec['bones'], float)
    bi = 0
    for f, names in FINGERS.items():
        for i in range(len(names) - 1):
            v = B[:, bi][np.isfinite(B[:, bi])]
            lbl = f'{f}_{names[i]}-{names[i+1]}'
            if len(v):
                print(f'  {lbl:16s} median {np.median(v):6.1f}  std {np.std(v):5.2f}  '
                      f'사전값 {prior[f][i]:5.1f}')
            bi += 1

    if rec['tri_vs_depth']:
        d = np.array(rec['tri_vs_depth'])
        print('-' * 76)
        print(f'■ 삼각측량 대비 depth 오차: 중앙값 {np.median(d):.2f} mm, '
              f'95% {np.percentile(d,95):.2f} mm  (N={len(d)})')
        print('   → 한쪽 뷰만 보일 때 depth 를 얼마나 믿을지의 근거')

    print('-' * 76)
    print(f'■ 처리 속도: {timing["fps"]:.1f} fps  '
          f'(검출 {timing["detect"]*1000:.1f} ms, 3D {timing["solve"]*1000:.1f} ms, '
          f'카메라 대기 {timing["wait"]*1000:.1f} ms)')

    path = session / f'mk{trial_no:02d}_raw.csv'
    hdr = ['frame']
    for k in JOINT_KEYS:
        hdr += [f'{k}_x', f'{k}_y', f'{k}_z', f'{k}_src']
    hdr += ANGLE_NAMES
    with open(path, 'w', newline='', encoding='utf-8-sig') as fp:
        w = _csv.writer(fp)
        w.writerow(hdr)
        for i in range(n):
            w.writerow([i] + rec['rows'][i] + list(A[i]))
    print(f'저장: {path.name}')
    print('=' * 76 + '\n')
    return dict(trial=trial_no, joints=jsum)


def main():
    args = parse_args()
    import cv2
    import pyrealsense2 as rs

    cal = None if args.single else load_calib(args.calib)
    if not args.colors.is_file():
        raise SystemExit(f'색 파일 없음: {args.colors} — marker_tune.py 먼저')
    colors = json.load(open(args.colors, encoding='utf-8'))
    miss = [k for k in (['wrist'] + list(FINGERS)) if k not in colors]
    if miss:
        raise SystemExit(f'색 미등록: {miss}')
    prior = json.load(open(args.bones, encoding='utf-8')) \
        if args.bones.is_file() else dict(DEFAULT_BONES)
    print('뼈 길이 사전값:', {k: [round(x, 1) for x in v] for k, v in prior.items()})

    ctx = rs.context()
    pipes, aligns, dscales = [], [], []
    serials = args.serials
    if serials is None:
        serials = [d.get_info(rs.camera_info.serial_number) for d in ctx.query_devices()
                   if 'Platform Camera' not in d.get_info(rs.camera_info.name)][:1]
        if not serials:
            raise SystemExit('RealSense 카메라를 찾지 못했습니다.')

    for s in serials:
        pipe = rs.pipeline(ctx)
        cfg = rs.config()
        cfg.enable_device(s)
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, args.fps)
        if not args.no_depth:
            cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, args.fps)
        prof = pipe.start(cfg)
        pipes.append(pipe)
        aligns.append(rs.align(rs.stream.color) if not args.no_depth else None)
        dscales.append(prof.get_device().first_depth_sensor().get_depth_scale()
                       if not args.no_depth else 1.0)
        print('카메라:', s)

    if args.single:
        intr = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
        Ks = [(K, np.asarray(intr.coeffs))]
    else:
        Ks = [(cal['K1'], cal['D1']), (cal['K2'], cal['D2'])]
        Rm, Tv = cal['R'], cal['T'].reshape(3)
    kernel = np.ones((3, 3), np.uint8)
    tracker = Tracker(args, prior)

    args.output.mkdir(parents=True, exist_ok=True)
    session = args.output / datetime.now().strftime('session_%Y%m%d_%H%M%S')
    session.mkdir(parents=True, exist_ok=True)
    print('저장 위치:', session)
    print('R 측정 | M 마스크 | Q 종료')

    cv2.namedWindow('marker 3D', cv2.WINDOW_NORMAL)
    rec, trials, show_mask = None, [], False
    bone_acc = {f: [] for f in FINGERS}
    tm = dict(wait=0.0, detect=0.0, solve=0.0, fps=0.0)
    t_prev = time.monotonic()

    try:
        while True:
            t0 = time.monotonic()
            frames, depths = [], []
            for pipe, al in zip(pipes, aligns):
                fs = pipe.wait_for_frames()
                if al is not None:
                    fs = al.process(fs)
                frames.append(np.asanyarray(fs.get_color_frame().get_data()).copy())
                df = fs.get_depth_frame() if al is not None else None
                depths.append(np.asanyarray(df.get_data()).copy() if df else None)
            t1 = time.monotonic()

            hsvs = [cv2.cvtColor(f, cv2.COLOR_BGR2HSV) for f in frames]
            det = {}
            masks = [np.zeros(frame.shape[:2], np.uint8) for frame in frames]
            for part in ['wrist'] + list(FINGERS):
                want = 1 if part == 'wrist' else len(FINGERS[part])
                per = []
                for i, h in enumerate(hsvs):
                    pts, lab, ids, m = find_blobs(cv2, h, colors[part], kernel, want)
                    masks[i] |= m
                    per.append((pts, lab, ids))
                det[part] = per
            t2 = time.monotonic()

            # ── 관절별 3D: tri → depth → bone ──
            J, SRC, tri_dep = {}, {}, []

            def solve_part(part, names):
                if args.single:
                    points, labels, ids = det[part][0]
                    cands = []
                    if depths[0] is not None:
                        for point, blob_id in zip(points, ids):
                            z = blob_depth(cv2, depths[0], labels, blob_id, dscales[0], kernel)
                            if z is not None:
                                cands.append((backproject(cv2, *Ks[0], point, z), 'depth'))
                    return cands

                (A, labA, idA), (B, labB, idB) = det[part]
                pairs, ua, ub = epipolar_match(cal, A, B, args.epi_tol)
                cands = []        # (3D, src)
                if pairs:
                    X, e = triangulate(cv2, cal,
                                       A[[a for a, _ in pairs]],
                                       B[[b for _, b in pairs]])
                    for k, (a, b) in enumerate(pairs):
                        if e[k] <= args.max_reproj:
                            cands.append((X[k], 'tri'))
                            # 같은 점의 depth 와 비교해 depth 신뢰도를 실측
                            if depths[0] is not None:
                                Z = blob_depth(cv2, depths[0], labA, idA[a],
                                               dscales[0], kernel)
                                if Z:
                                    Xd = backproject(cv2, *Ks[0], A[a], Z)
                                    tri_dep.append(float(np.linalg.norm(Xd - X[k])))
                if not args.no_depth:
                    for a in ua:                      # cam1 에서만 보임
                        if depths[0] is None:
                            break
                        Z = blob_depth(cv2, depths[0], labA, idA[a], dscales[0], kernel)
                        if Z:
                            cands.append((backproject(cv2, *Ks[0], A[a], Z), 'depth'))
                    for b in ub:                      # cam2 에서만 보임 → cam1 좌표계로
                        if depths[1] is None:
                            break
                        Z = blob_depth(cv2, depths[1], labB, idB[b], dscales[1], kernel)
                        if Z:
                            X2 = backproject(cv2, *Ks[1], B[b], Z)
                            cands.append((Rm.T @ (X2 - Tv), 'depth'))
                return cands

            wc = solve_part('wrist', ['wrist'])
            if wc and tracker.accept('wrist', wc[0][0], J):
                J['wrist'] = wc[0][0]
                SRC['wrist'] = wc[0][1]
            elif not args.no_bone_fill and tracker.last.get('wrist') is not None:
                J['wrist'] = tracker.last['wrist']
                SRC['wrist'] = 'bone'

            for f, names in FINGERS.items():
                cands = solve_part(f, names)
                pts = np.array([c[0] for c in cands]) if cands else np.zeros((0, 3))
                srcs = [c[1] for c in cands]
                order = order_chain(pts, J.get('wrist'), prior[f]) \
                    if len(pts) == len(names) else None
                if order is not None:
                    for i, oi in enumerate(order):
                        key = f'{f}_{names[i]}'
                        if tracker.accept(key, pts[oi], J):
                            J[key] = pts[oi]
                            SRC[key] = srcs[oi]
                if args.measure_bones and order is not None:
                    P = pts[order]
                    bone_acc[f].append(np.linalg.norm(np.diff(P, axis=0), axis=1))
                # 3단계: 아직 빈 관절을 부모+뼈길이로 역산
                if not args.no_bone_fill:
                    for n_ in names:
                        key = f'{f}_{n_}'
                        if key in J:
                            continue
                        X = tracker.fill_from_bone(key, J)
                        if X is not None:
                            J[key] = X
                            SRC[key] = 'bone'

            for k in JOINT_KEYS:
                if k in J:
                    tracker.commit(k, J[k])
            out = {k: tracker.smoothed(k) for k in JOINT_KEYS if k in J}
            ang = compute_angles(out)
            t3 = time.monotonic()

            a = 0.15
            tm['wait'] += a * ((t1 - t0) - tm['wait'])
            tm['detect'] += a * ((t2 - t1) - tm['detect'])
            tm['solve'] += a * ((t3 - t2) - tm['solve'])
            now = time.monotonic()
            tm['fps'] += a * (1.0 / max(1e-6, now - t_prev) - tm['fps'])
            t_prev = now

            # ── 표시 ──
            vis = [cv2.cvtColor(masks[i], cv2.COLOR_GRAY2BGR) if show_mask
                   else frame.copy() for i, frame in enumerate(frames)]
            canvas = np.hstack(vis)
            H, W = canvas.shape[:2]
            cv2.rectangle(canvas, (0, 0), (W, 100), (0, 0, 0), -1)
            ntri = sum(1 for v in SRC.values() if v == 'tri')
            ndep = sum(1 for v in SRC.values() if v == 'depth')
            nbon = sum(1 for v in SRC.values() if v == 'bone')
            cv2.putText(canvas, f'관절 {len(J)}/12  (tri {ntri} / depth {ndep} / bone {nbon})'
                        f'   {tm["fps"]:.0f} fps' + ('   [마스크]' if show_mask else ''),
                        (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            y = 44
            for f, names in FINGERS.items():
                keys = [f'{f}_{n}' for n in names[:-1]]
                t = ' '.join(f'{ang[k]:5.0f}' if np.isfinite(ang[k]) else '  ---'
                             for k in keys)
                cv2.putText(canvas, f'{f:7s} {t}', (8, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 120), 1)
                y += 18

            if rec is not None:
                row = []
                for k in JOINT_KEYS:
                    X = out.get(k)
                    row += (list(X) if X is not None else [np.nan] * 3) \
                        + [SRC.get(k, 'none')]
                    rec['src'][k][SRC.get(k, 'none')] += 1
                rec['rows'].append(row)
                rec['angles'].append([ang[k] for k in ANGLE_NAMES])
                bl = []
                for f, names in FINGERS.items():
                    for i in range(len(names) - 1):
                        A_, B_ = out.get(f'{f}_{names[i]}'), out.get(f'{f}_{names[i+1]}')
                        bl.append(float(np.linalg.norm(B_ - A_))
                                  if A_ is not None and B_ is not None else np.nan)
                rec['bones'].append(bl)
                rec['nmark'].append(len(J))
                rec['tri_vs_depth'].extend(tri_dep)
                el = time.monotonic() - rec['t0']
                cv2.putText(canvas, f'REC {el:.1f}/{REPEAT_SECONDS:.0f}s',
                            (8, H - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                if el >= REPEAT_SECONDS:
                    trials.append(report(rec, session, len(trials) + 1, prior, tm))
                    rec = None

            cv2.imshow('marker 3D', canvas)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord('q'), 27):
                break
            if k == ord('m'):
                show_mask = not show_mask
            if k == ord('r') and rec is None:
                rec = dict(t0=time.monotonic(), rows=[], angles=[], bones=[],
                           nmark=[], tri_vs_depth=[],
                           src={kk: dict(tri=0, depth=0, bone=0, none=0)
                                for kk in JOINT_KEYS})
                print(f'\n[REC #{len(trials)+1}] {REPEAT_SECONDS:.0f}초 고정...')
    finally:
        for p in pipes:
            try:
                p.stop()
            except Exception:
                pass
        cv2.destroyAllWindows()
        if args.measure_bones:
            out = {f: np.median(np.array(v), axis=0).round(2).tolist()
                   for f, v in bone_acc.items() if v}
            if out:
                merged = dict(prior)
                merged.update(out)
                json.dump(merged, open(args.bones, 'w', encoding='utf-8'), indent=2)
                print('\n뼈 길이 실측 저장:', args.bones)
                for f, v in out.items():
                    print(f'  {f}: {v}')
        if trials:
            print('CSV:', session)


if __name__ == '__main__':
    main()
