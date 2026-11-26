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

원형 필터 (D 로 켜기/끄기, 기본 ON) - 동그란 덩어리만 센다 (거리에 따라 크기가 달라도 OK)
  1) 원형도: 4π·넓이/둘레² >= --circ(기본 0.65)   (원 = 1, 선/모서리/뭉개진 얼룩은 낮음)
     + 외접원 채움 >= 0.55, 가로세로 비 <= 1.6
  2) 픽셀 지름: --px-range(기본 6~80px) 밖이면 버림 (점 노이즈, 화면 가득한 큰 물체)
  3) 안정성: 연속 --stable(기본 4) 프레임 이상 같은 자리 근처에서 계속 잡혀야 인정 (깜빡이는 노이즈)
     한번 인정된 마커는 (트래킹)
       - 움직여서 번지거나(모션 블러) 살짝 찌그러져도 직전 위치 근처면 계속 인정 (원형도 기준 절반까지)
       - 몇 프레임 놓쳐도 --hold(기본 6) 프레임 동안은 예상 위치에 유지 (점선 원 + "hold")
  4) 개수 제한: 색마다 가장 안정적인(최근에 가장 꾸준히 잡힌) 마커 --expect(기본 1) 개만 센다.
     나머지는 세지 않고 회색 "extra 0.xx"(안정도)로만 표시. 한번 고른 마커는 계속 유지하고,
     다른 덩어리가 훨씬 더 꾸준하면(안정도 +0.3 이상) 그때만 바꿈. --expect 0 = 제한 없음
  5) (선택) 거리: --max-depth mm 를 주면 depth 가 그보다 먼 것 버림. depth 를 못 읽은 건 버리지 않음.
  6) (선택) 실제 지름: --size mm 를 주면 depth 로 계산한 지름이 size*0.5~1.8 밖이면 버림
  버린 덩어리는 회색으로 칠하고, 이유를 작게 표시 (round 0.42 / small / big / new 2/4 / far / size 23mm)
  너무 작아서(지름 7px 미만) 원형도를 믿기 어려운 것은 원형도 대신 채움/비율만 봄

보기 (V 로 전환)
  dim  : 원본을 어둡게 흑백으로 + 잡힌 픽셀을 그 색으로 진하게 칠함 (기본, 제일 잘 보임)
  mask : 검은 바탕에 잡힌 픽셀만
  raw  : 원본 그대로 (동그라미 표시만)
  잡힌 덩어리마다 동그라미 + 번호. 개수가 --expect(기본 1)와 같으면 초록 OK, 많으면 빨강 +N FALSE.

저장 (color_select/maskview/ 아래)
  S : 스냅샷 → 폴더 하나에 화면(screen.png) + 카메라별 원본(raw_*.png)/마스크(mask_*.png, 원본 해상도)
      + counts.csv.  모든 스냅샷의 개수는 maskview/counts_log.csv 에도 누적
  R : 녹화 시작/정지 → 화면 영상(screen.mp4) + 프레임마다 개수(counts.csv)

실행:
    conda activate rs_env
    python marker_mask_view.py --res 1280x720
    python marker_mask_view.py --expect 1          # 색마다 정상 마커 개수 (기본 1)
    python marker_mask_view.py --circ 0.7 --stable 4 --px-range 6 80
    python marker_mask_view.py --max-depth 800                  # 80cm 보다 먼 배경도 버리기
    python marker_mask_view.py --no-xyz                         # 3D 좌표 끄기 (depth 안 받음)

3D 좌표 (기본 ON)
  depth 영상 전체를 color 에 정렬하지 않고(느림), 마커 중심 픽셀 하나만 depth 에서 찾아 3D 로 바꿈 (점당 ~0.1ms).
  좌표는 각 카메라의 color 카메라 기준 mm (x 오른쪽, y 아래, z 앞). 마커 표면 기준 (구 중심은 반지름만큼 더 뒤).

키:
    1~6 색 켜기/끄기   A 전부 켜기/끄기   V 보기 전환(dim / mask / raw)   F 전체화면
    D 구 필터 ON/OFF   C 샘플 정리 ON/OFF   SPACE 화면 정지   S 스냅샷 저장   R 녹화 시작/정지   Q/ESC 종료
"""

import argparse
import csv
import math
import shutil
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path

import numpy as np

from marker_common import FONT, MIN_AREA, RSCamera, blob_depth_mm, cv2, list_devices, make_mask, ranges_from
from marker_color_select import DATA_DIR, DRAW, KO, MARKERS

MARKER_KEYS = [k for k, _ in MARKERS]
HUE_KEEP_DEG = 35.0
SIMILAR_DEG, SIMILAR_SAT = 20.0, 60   # 색상 중심 20° 이내 + 채도 중앙값 60 이내면 '비슷한 색' 경고
SHORT = dict(red="R", green="G", purple="Pu", pink="Pk", yellow="Y", blue="B")
VIEWS = ["dim", "mask", "raw"]
REC_FPS = 15


def put(img, text, org, scale=0.5, color=(255, 255, 255), thick=1):
    """검은 테두리 글자. 테두리를 굵기 대신 위치를 조금씩 옮겨 그림 (굵게 그리면 글자 간격이 달라져 번져 보임)."""
    x, y = org
    d = max(1, int(round(thick * 1.2)))
    for dx, dy in ((-d, -d), (d, d), (-d, d), (d, -d)):
        cv2.putText(img, text, (x + dx, y + dy), FONT, scale, (0, 0, 0), thick, cv2.LINE_AA)
    cv2.putText(img, text, org, FONT, scale, color, thick, cv2.LINE_AA)


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


def grid_layout(n, aw, ah, ar):
    """카메라 n대를 aw x ah 안에 가장 크게: (cols, rows, tile_w, tile_h)."""
    best = None
    for cols in range(1, n + 1):
        rows = math.ceil(n / cols)
        tw = min(aw / cols, ah / rows * ar)
        if best is None or tw > best[2]:
            best = (cols, rows, tw)
    cols, rows, tw = best
    tw = max(16, int(tw))
    return cols, rows, tw, max(9, int(tw / ar))


def shape_metrics(lab_crop, i):
    """덩어리 하나의 (원형도, 외접원 채움, 가로세로 비, 지름 px)."""
    sel = (lab_crop == i).astype(np.uint8)
    cs_, _ = cv2.findContours(sel, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not cs_:
        return 0.0, 0.0, 99.0, 0.0
    c = max(cs_, key=cv2.contourArea)
    area = float(sel.sum())
    per = cv2.arcLength(c, True)
    circ = 4 * math.pi * area / max(per * per, 1e-6)
    (_, _), rad = cv2.minEnclosingCircle(c)
    efill = area / max(math.pi * rad * rad, 1e-6)
    x, y, w, h = cv2.boundingRect(c)
    return circ, efill, max(w, h) / max(1, min(w, h)), 2 * rad


def shape_ok(met, circ_min, aspect_max=1.6, efill_min=0.55):
    circ, efill, aspect, diam = met
    if aspect > aspect_max or efill < efill_min:
        return False
    return diam < 7 or circ >= circ_min        # 아주 작으면 둘레가 계단이라 원형도가 부정확 → 채움/비율만


def match(tracks_, blobs, reach):
    """가까운 쌍부터 1:1 매칭. 반환 [(track, blob)]"""
    pairs = sorted((math.hypot(b["x"] - t["px"], b["y"] - t["py"]), ti, bi)
                   for ti, t in enumerate(tracks_) for bi, b in enumerate(blobs))
    used_t, used_b, out = set(), set(), []
    for d, ti, bi in pairs:
        if d > reach:
            break
        if ti in used_t or bi in used_b:
            continue
        used_t.add(ti)
        used_b.add(bi)
        out.append((tracks_[ti], blobs[bi]))
    return out


def analyze(cam, it, rngs, on, cfg, tracks):
    """새 프레임 하나 분석. 켠 색마다: 인정된 마커(트래킹) / 버린 덩어리 / 마스크.
    tracks[k] = 트랙 목록 (프레임 사이에 유지). 트랙: x,y 위치, vx,vy 속도, hits 연속 검출, miss 놓친 프레임, conf 인정됨."""
    res = {}
    fx = float(cam.intr.fx)
    reach = 40.0 * it["color"].shape[1] / 1280      # 한 프레임 사이 이동 허용 (px)
    pmin, pmax = cfg["px"]
    for k in MARKER_KEYS:
        if not on[k] or k not in rngs:
            continue
        m = make_mask(it["hsv"], rngs[k])
        n, lab, cs, cents = cv2.connectedComponentsWithStats(m)
        areas = cs[:, cv2.CC_STAT_AREA]
        idx = np.nonzero(areas >= MIN_AREA)[0]
        idx = idx[idx > 0]
        keep, rejected, strict, loose = [], [], [], []
        extra = 0
        if cfg["filter"]:
            ext = np.maximum(cs[:, cv2.CC_STAT_WIDTH], cs[:, cv2.CC_STAT_HEIGHT])
            need_z = cfg["max_depth"] > 0 or cfg["size"] > 0
            if need_z and idx.size:
                onp = m.ravel() > 0
                flat, dep = lab.ravel()[onp], it["depth"].ravel()[onp]
                dsum = np.bincount(flat, weights=dep, minlength=n)
                dcnt = np.bincount(flat, weights=(dep > 0), minlength=n)
                zmean = dsum / np.maximum(dcnt, 1) * cam.depth_scale * 1000.0
            for i in idx:
                b = dict(x=float(cents[i][0]), y=float(cents[i][1]), area=int(areas[i]), z=None, d=None,
                         lab=int(i))
                if ext[i] < pmin or ext[i] > pmax * 3:
                    continue                                  # 점 노이즈 / 아주 큰 배경: 표시도 안 함
                if need_z and cfg["max_depth"] > 0 and dcnt[i] >= 3 and zmean[i] > cfg["max_depth"]:
                    continue                                  # 먼 배경: 표시도 안 함
                if ext[i] > pmax:
                    b["why"] = "big"
                    rejected.append(b)
                    continue
                x, y, w, h = cs[i, :4]
                met = shape_metrics(lab[y:y + h, x:x + w], i)
                b["circ"] = met[0]
                if cfg["size"] > 0 and it["depth"] is not None and cam.align is not None:
                    z = blob_depth_mm(it["depth"], lab, i, cam.depth_scale, cs)
                    if z is not None:
                        b["d"] = max(w, h) * z / fx
                        if not (cfg["size"] * 0.5 <= b["d"] <= cfg["size"] * 1.8):
                            b["why"] = f"size {b['d']:.0f}mm"
                            rejected.append(b)
                            continue
                if shape_ok(met, cfg["circ"]):
                    strict.append(b)
                else:
                    b["why"] = f"round {min(met[0], 1):.2f}"
                    # 인정된 마커 근처면 쓸 수 있는 '느슨한' 후보 (모션 블러로 길쭉해진 것 등)
                    if shape_ok(met, cfg["circ"] * 0.5, aspect_max=2.6, efill_min=0.35):
                        loose.append(b)
                    else:
                        rejected.append(b)
            # ---- 트래킹
            tr = tracks.setdefault(k, [])
            for t in tr:                                      # 예상 위치 = 직전 위치 + 속도
                t["px"], t["py"] = t["x"] + t["vx"], t["y"] + t["vy"]
            pairs = match(tr, strict, reach)
            got_t = {id(t) for t, _ in pairs}
            got_b = {id(b) for _, b in pairs}
            conf_left = [t for t in tr if t["conf"] and id(t) not in got_t]
            pairs += match(conf_left, loose, reach)           # 인정된 마커만 느슨한 후보로 이어감
            got_t = {id(t) for t, _ in pairs}
            got_b |= {id(b) for _, b in pairs}
            for t, b in pairs:
                t["vx"] = 0.5 * t["vx"] + 0.5 * (b["x"] - t["x"])
                t["vy"] = 0.5 * t["vy"] + 0.5 * (b["y"] - t["y"])
                t["x"], t["y"], t["hits"], t["miss"], t["b"] = b["x"], b["y"], t["hits"] + 1, 0, b
                t["conf"] = t["conf"] or t["hits"] >= cfg["stable"]
                t["stab"] = 0.9 * t["stab"] + 0.1          # 최근 프레임 중 잡힌 비율 (안정도, 0~1)
                b.pop("why", None)
            for t in tr:
                if id(t) not in got_t:
                    t["stab"] *= 0.9
                    t["miss"] += 1
                    t["x"], t["y"], t["b"] = t["px"], t["py"], None
                    t["vx"] *= 0.5
                    t["vy"] *= 0.5
            tr[:] = [t for t in tr if t["miss"] <= (cfg["hold"] if t["conf"] else 1)]
            for b in strict:
                if id(b) not in got_b:                        # 새로 나타난 후보 → 새 트랙
                    tr.append(dict(x=b["x"], y=b["y"], vx=0.0, vy=0.0, hits=1, miss=0, conf=cfg["stable"] <= 1, b=b,
                                   px=b["x"], py=b["y"], stab=0.1, sel=False))
            # ---- 색마다 가장 안정적인 마커 expect 개만 (나머지는 세지 않음)
            # 이미 고른 마커는 계속 유지 (번갈아 바뀌지 않게). 다른 것이 훨씬 더 안정적이면(+0.3) 그때 바꿈.
            conf = [t for t in tr if t["conf"]]
            conf.sort(key=lambda t: (t["stab"] + (0.3 if t["sel"] else 0.0), t["hits"]), reverse=True)
            for j, t in enumerate(conf):
                t["sel"] = cfg["expect"] <= 0 or j < cfg["expect"]
            extra = 0
            for t in tr:
                b = t["b"]
                if t["conf"] and not t["sel"]:
                    extra += 1
                    if b is not None:
                        b["why"] = f"extra {t['stab']:.2f}"
                        rejected.append(b)
                    continue
                if t["conf"]:
                    if b is not None:
                        b["held"] = 0
                        keep.append(b)
                    else:                                     # 잠깐 놓침 → 예상 위치에 유지
                        keep.append(dict(x=t["x"], y=t["y"], area=0, z=None, d=None, lab=None, held=t["miss"]))
                elif b is not None:
                    b["why"] = f"new {t['hits']}/{cfg['stable']}"
                    rejected.append(b)
            rejected += [b for b in loose if id(b) not in got_b]
            keep.sort(key=lambda b: (b["held"] > 0, -b["area"]))
            acc = np.zeros(m.shape, np.uint8)               # 채택된 덩어리만 (bbox 안에서만 칠함)
            for b in keep:
                if b["lab"] is not None:
                    x, y, w, h = cs[b["lab"], :4]
                    acc[y:y + h, x:x + w][lab[y:y + h, x:x + w] == b["lab"]] = 255
        else:
            keep = [dict(x=float(cents[i][0]), y=float(cents[i][1]), area=int(areas[i]), z=None, d=None,
                         lab=int(i), held=0) for i in idx]
            keep.sort(key=lambda b: -b["area"])
            acc = m
        # ---- 3D: 실제로 보인 마커만, 그 점 하나만 depth 에서 찾음
        if cfg["xyz"] and it.get("depth") is not None:
            for b in keep:
                if b["lab"] is not None:
                    b["xyz"] = cam.xyz_at(b["x"], b["y"], it["depth"])
        res[k] = dict(mask=acc, all=m, blobs=keep, rejected=rejected, extra=extra,
                      pixels=int(sum(b["area"] for b in keep)))
    return res


def colorize(color, res, view, size=None):
    """size=(w,h) 를 주면 그 크기로 줄인 뒤 칠함 (화면용, 빠름). 작은 마커도 사라지지 않게 마스크는 INTER_AREA 후 >0."""
    def fit(m):
        return (cv2.resize(m, size, interpolation=cv2.INTER_AREA) if size else m) > 0

    base = cv2.resize(color, size, interpolation=cv2.INTER_AREA) if size else color
    if view == "raw":
        return base.copy()
    if view == "mask":
        out = np.zeros_like(base)
    else:                                   # dim: 흑백 + 어둡게 → 칠한 색이 확 튐
        g = cv2.convertScaleAbs(cv2.cvtColor(base, cv2.COLOR_BGR2GRAY), alpha=0.4)
        out = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    if res:
        cnt = np.zeros(base.shape[:2], np.uint8)
        acc = {k: fit(r["mask"]) for k, r in res.items()}
        for k, r in res.items():
            if r["mask"] is not r["all"]:
                out[fit(cv2.subtract(r["all"], r["mask"]))] = (75, 75, 75)   # 색은 맞지만 필터에서 버린 픽셀
        for k, a in acc.items():
            out[a] = DRAW[k]
            cnt += a
        out[cnt >= 2] = (255, 255, 255)     # 두 색 이상이 같은 픽셀 = 충돌
    return out


def status(nb, expect):
    if nb == expect:
        return "OK", (60, 220, 60)
    if nb > expect:
        return f"+{nb - expect} FALSE", (60, 60, 255)
    return "MISSING", (0, 170, 255)


def draw_overlay(img, cam, res, rngs, sc, tf, expect, frozen, warn, rec):
    """img(이미 sc 배로 줄인 화면) 위에 동그라미/글자. 글자 크기 tf 는 화면 픽셀 기준."""
    th = 2 if tf >= 0.7 else 1
    lh = int(cv2.getTextSize("Ag", FONT, tf, th)[0][1] * 1.9)
    H, W = img.shape[:2]
    # 덩어리마다 동그라미 + 이름번호 (두 색이 같은 덩어리를 잡으면 글자를 아래로 비켜서)
    placed = []
    for k, r in res.items():
        for b in r.get("rejected", []):                # 버린 후보: 작은 회색 원 + 이유
            x, y = int(b["x"] * sc), int(b["y"] * sc)
            cv2.circle(img, (x, y), max(4, int(lh * 0.35)), (150, 150, 150), 1, cv2.LINE_AA)
            put(img, b["why"], (x + int(lh * 0.45), y - int(lh * 0.3)), tf * 0.6, (160, 160, 160), 1)
        for i, b in enumerate(r["blobs"]):
            cx, cy, area = b["x"], b["y"], b["area"]
            x, y = int(cx * sc), int(cy * sc)
            rad = max(int(lh * 0.7), int(math.sqrt(area / math.pi) * sc) + int(lh * 0.3))
            if b.get("held"):                               # 잠깐 놓쳐서 예상 위치에 유지 중
                for a0_ in range(0, 360, 30):
                    cv2.ellipse(img, (x, y), (rad, rad), 0, a0_, a0_ + 15, DRAW[k], th, cv2.LINE_AA)
            else:
                cv2.circle(img, (x, y), rad, (0, 0, 0), th + 3, cv2.LINE_AA)
                cv2.circle(img, (x, y), rad, DRAW[k], th + 1, cv2.LINE_AA)
            tx, ty = x + rad + 3, y + lh // 3
            while any(abs(tx - px) < lh * 2 and abs(ty - py) < lh * 0.8 for px, py in placed):
                ty += int(lh * 0.85)
            placed.append((tx, ty))
            put(img, f"{SHORT[k]}{i + 1}" + (f" {b['d']:.0f}mm" if b["d"] else "") + (" hold" if b.get("held") else ""),
                (tx, ty), tf * 0.85, DRAW[k], th)
            if b.get("xyz") is not None:
                X, Y, Z = b["xyz"]
                put(img, f"({X:.0f}, {Y:.0f}, {Z:.0f})", (tx, ty + int(lh * 0.75)), tf * 0.6, (230, 230, 230), 1)
    # 위: 카메라 이름
    head = f"{cam.label}  {cam.fps_val:4.1f}fps" + ("  FROZEN" if frozen else "") + ("  REC" if rec else "")
    cv2.rectangle(img, (0, 0), (W, lh + 6), (0, 0, 0), -1)
    put(img, head, (6, lh - 2), tf, (255, 255, 255) if not rec else (80, 80, 255), th)
    if warn:
        put(img, "similar: " + ", ".join(f"{a}~{b}" for a, b, _ in warn), (6, 2 * lh + 2), tf * 0.85,
            (60, 60, 255), th)
    # 아래 왼쪽: 색 / 개수 / 상태 / 픽셀
    rows = [k for k in MARKER_KEYS if k in res] or [None]
    tf = tf * 0.85
    th = 2 if tf >= 0.75 else 1
    lh = int(cv2.getTextSize("Ag", FONT, tf, th)[0][1] * 1.8)
    cw = cv2.getTextSize("purple  9  +9 FALSE  999999px", FONT, tf, th)[0][0] + 16
    y0 = H - lh * (len(rows) + 1) - 8
    roi = img[max(0, y0):H, 0:min(W, cw)]
    roi[:] = (roi * 0.45).astype(np.uint8)
    y = y0 + lh
    put(img, f"color   n   (expect {expect})", (8, y), tf * 0.9, (170, 170, 170), th)
    for k in rows:
        y += lh
        if k is None:
            put(img, "1-6: turn colors on", (8, y), tf, (170, 170, 170), th)
            continue
        nb = len(res[k]["blobs"])
        s, col = status(nb, expect)
        put(img, f"{k:7s}{nb:2d}", (8, y), tf, DRAW[k], th)
        x2 = 8 + cv2.getTextSize(f"{k:7s}{nb:2d}  ", FONT, tf, th)[0][0]
        put(img, s, (x2, y), tf, col, th)
        x3 = x2 + cv2.getTextSize("+9 FALSE  ", FONT, tf, th)[0][0]
        put(img, f"{res[k]['pixels']}px", (x3, y), tf * 0.8, (170, 170, 170), th)


def marker_cols(blobs, n=4):
    """CSV 용: 마커 1~n 번의 픽셀 위치 / hold 여부 / 3D(mm). 없으면 빈 칸 (열 개수 고정)."""
    out = {}
    for j in range(n):
        b = blobs[j] if j < len(blobs) else None
        xyz = b.get("xyz") if b else None
        vals = ("", "", "", "", "", "") if b is None else (
            f"{b['x']:.1f}", f"{b['y']:.1f}", int(bool(b.get("held"))),
            *((f"{q:.1f}" for q in xyz) if xyz is not None else ("", "", "")))
        for f, v in zip(("u", "v", "hold", "x_mm", "y_mm", "z_mm"), vals):
            out[f"m{j + 1}_{f}"] = v
    return out


def count_rows(cams, results, expect):
    out = []
    for c in cams:
        for k, r in results.get(c.serial, {}).items():
            nb = len(r["blobs"])
            out.append(dict(camera=c.label, serial=c.serial, color=k, blobs=nb, expect=expect,
                            status=status(nb, expect)[0], pixels=r["pixels"],
                            rejected=len(r["rejected"]),
                            held=sum(1 for b in r["blobs"] if b.get("held")), extra=r.get("extra", 0),
                            **marker_cols(r["blobs"])))
    return out


def write_csv(path, rows, extra=None, append=False):
    if not rows:
        return
    rows = [dict(extra or {}, **r) for r in rows]
    new = not (append and path.is_file())
    with open(path, "a" if append else "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        if new:
            w.writeheader()
        w.writerows(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--serial", nargs="*")
    ap.add_argument("--res", default="1280x720")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--data-dir", default=str(DATA_DIR))
    ap.add_argument("--expect", type=int, default=1, help="색마다 셀 마커 개수 (가장 안정적인 것부터, 기본 1, 0=제한 없음)")
    ap.add_argument("--circ", type=float, default=0.65, help="원형도 최소값 (원=1, 높일수록 엄격)")
    ap.add_argument("--px-range", type=float, nargs=2, default=[6, 80], metavar=("MIN", "MAX"),
                    help="허용 덩어리 지름(px)")
    ap.add_argument("--stable", type=int, default=4, help="연속 몇 프레임 잡혀야 인정")
    ap.add_argument("--hold", type=int, default=6, help="인정된 마커를 놓쳐도 이 프레임 수만큼 예상 위치에 유지")
    ap.add_argument("--no-xyz", dest="xyz", action="store_false", help="3D 좌표 끄기 (depth 안 받음)")
    ap.add_argument("--max-depth", type=float, default=0, help="(선택) 이보다 먼(mm) 덩어리 무시. 0=끔")
    ap.add_argument("--size", type=float, default=0, help="(선택) 실제 지름 mm 검사. 0=끔")
    args = ap.parse_args()
    W, H = map(int, args.res.lower().split("x"))
    out_root = Path(args.data_dir) / "maskview"

    devs = list_devices()
    print("연결된 RealSense:", ", ".join(f"{n} ({s})" for n, s in devs) or "없음")
    cams = []
    for s in (args.serial or [sv for _, sv in devs]):
        try:
            # 크기(mm)/먼 배경 필터는 정렬된 depth 가 필요(느림). 3D 좌표만이면 점 단위 계산(raw, 빠름)
            need = True if (args.max_depth > 0 or args.size > 0) else ("raw" if args.xyz else False)
            cams.append(RSCamera(s, W, H, args.fps, depth=need))
            print("열림:", cams[-1].label)
        except Exception as e:
            print(f"열기 실패 {s}: {e}")
    if not cams:
        return

    st = dict(filter=True, on={k: False for k in MARKER_KEYS}, clean=True, view="dim", frozen=False, full=False, msg=("", 0))
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

    def say(m):
        print(m)
        st["msg"] = (m, time.time())

    reload()
    items, results, tracks, seen = {}, {}, {c.serial: {} for c in cams}, {}
    rec = None   # dict(dir, writer, size, t0, n, csv)
    win = "marker mask view"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL | cv2.WINDOW_FREERATIO)
    cv2.resizeWindow(win, 1600, 900)
    print(__doc__.split("키:")[1])
    pool = ThreadPoolExecutor(max_workers=len(cams))     # 카메라별 분석/그리기 병렬 (OpenCV 는 GIL 을 놓음)

    try:
        while True:
            if not st["frozen"]:
                for c in cams:
                    it = c.get()
                    if it is not None:
                        items[c.serial] = it
            def analyze_cam(c):
                it = items.get(c.serial)
                if it is None:
                    return
                key = (it.get("seq"), st["filter"], st["clean"], tuple(st["on"].values()))
                old = seen.get(c.serial)
                if old == key:                              # 새 프레임도 설정 변화도 없으면 다시 계산 안 함
                    return
                if old is not None and old[1:] != key[1:]:  # 설정이 바뀌면 트래킹 새로
                    tracks[c.serial].clear()
                cfg = dict(filter=st["filter"], circ=args.circ, px=args.px_range, size=args.size,
                           max_depth=args.max_depth, stable=args.stable, hold=args.hold, xyz=args.xyz,
                           expect=args.expect)
                results[c.serial] = analyze(c, it, ranges.get(c.serial, {}), st["on"], cfg, tracks[c.serial])
                seen[c.serial] = key

            list(pool.map(analyze_cam, cams))

            # ---- 창 크기에 맞춰 배치 (글자는 화면 픽셀 기준이라 전체화면에서도 또렷)
            try:
                _, _, ww, wh = cv2.getWindowImageRect(win)
            except cv2.error:
                ww, wh = 1600, 900
            if ww < 200 or wh < 150:
                ww, wh = 1600, 900
            ui = float(np.clip(min(ww / 1600, wh / 900), 0.6, 1.8))
            bf, bth = 0.62 * ui, (2 if ui >= 1.1 else 1)
            blh = int(cv2.getTextSize("Ag", FONT, bf, bth)[0][1] * 2.0)
            bar_h = blh * 2 + 8
            cols, rows, tw, th_ = grid_layout(len(cams), ww, wh - bar_h, W / H)
            gx = (ww - cols * tw) // 2
            gy = bar_h + (wh - bar_h - rows * th_) // 2
            tf = float(np.clip(th_ / 480 * 0.75, 0.5, 1.0))
            sc = tw / W
            canvas = np.full((wh, ww, 3), 18, np.uint8)
            def render_cam(ic):
                i, c = ic
                x0, y0 = gx + (i % cols) * tw, gy + (i // cols) * th_
                it = items.get(c.serial)
                if it is None:
                    put(canvas, f"{c.label}: no frame", (x0 + 10, y0 + 30), tf)
                    return
                res = results.get(c.serial, {})
                tile = colorize(it["color"], res, st["view"], (tw, th_))
                draw_overlay(tile, c, res, ranges.get(c.serial, {}), sc, tf, args.expect, st["frozen"],
                             warns.get(c.serial), rec is not None)
                cv2.rectangle(tile, (0, 0), (tw - 1, th_ - 1), (90, 90, 90), 1)
                canvas[y0:y0 + th_, x0:x0 + tw] = tile

            list(pool.map(render_cam, enumerate(cams)))

            # ---- 위 막대: 색 켜기 상태 + 키 도움말
            x = 10
            for i, k in enumerate(MARKER_KEYS):
                on = st["on"][k]
                s = blh // 2
                cv2.rectangle(canvas, (x, blh // 3), (x + s, blh // 3 + s), DRAW[k], -1 if on else 2)
                txt = f"{i + 1}:{k}"
                put(canvas, txt, (x + s + 6, blh - 4), bf, DRAW[k] if on else (120, 120, 120), bth)
                x += s + 6 + cv2.getTextSize(txt, FONT, bf, bth)[0][0] + int(22 * ui)
            sph = f">={args.circ:.2f}" if st["filter"] else "OFF"
            put(canvas, f"view {st['view']}  circle {sph}  clean {'ON' if st['clean'] else 'OFF'}"
                        + ("   * REC *" if rec else ""), (x + 10, blh - 4), bf,
                (80, 80, 255) if rec else (200, 200, 200), bth)
            m, t = st["msg"]
            help_txt = (m if m and time.time() - t < 4 else
                        "white = 2+ colors  gray = rejected | 1-6 toggle  A all  V view  D circle  F full  C clean  "
                        "SPACE freeze  S snapshot  R record  Q quit")
            put(canvas, help_txt, (10, 2 * blh), bf * 0.85, (230, 230, 120) if m and time.time() - t < 4
                else (170, 170, 170), bth)
            cv2.imshow(win, canvas)

            # ---- 녹화: 실제 시간에 맞춰 REC_FPS 로 기록 (느리면 프레임 복제)
            if rec:
                fr = canvas if canvas.shape[1::-1] == rec["size"] else cv2.resize(canvas, rec["size"])
                now = time.time()
                while rec["n"] < int((now - rec["t0"]) * REC_FPS) + 1:
                    rec["writer"].write(fr)
                    rec["n"] += 1
                write_csv(rec["csv"], count_rows(cams, results, args.expect),
                          dict(time_s=f"{now - rec['t0']:.3f}"), append=True)

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
                st["view"] = VIEWS[(VIEWS.index(st["view"]) + 1) % len(VIEWS)]
            elif ch == "d":
                st["filter"] = not st["filter"]
                say(f"circle filter {'ON' if st['filter'] else 'OFF'}")
            elif ch == "c":
                st["clean"] = not st["clean"]
                reload()
            elif ch == " ":
                st["frozen"] = not st["frozen"]
            elif ch == "f":
                st["full"] = not st["full"]
                cv2.setWindowProperty(win, cv2.WND_PROP_FULLSCREEN,
                                      cv2.WINDOW_FULLSCREEN if st["full"] else cv2.WINDOW_NORMAL)
            elif ch == "s":
                on = "+".join(key for key in MARKER_KEYS if st["on"][key]) or "none"
                stamp = datetime.now()
                d = out_root / f"snap_{stamp:%Y%m%d_%H%M%S}_{on}"
                d.mkdir(parents=True, exist_ok=True)
                cv2.imencode(".png", canvas)[1].tofile(str(d / "screen.png"))
                for c in cams:
                    it = items.get(c.serial)
                    if it is None:
                        continue
                    res = results.get(c.serial, {})
                    full = colorize(it["color"], res, "dim" if st["view"] == "raw" else st["view"])
                    draw_overlay(full, c, res, ranges.get(c.serial, {}), 1.0, 0.8, args.expect, False,
                                 warns.get(c.serial), False)
                    cv2.imencode(".png", it["color"])[1].tofile(str(d / f"raw_{c.serial}.png"))
                    cv2.imencode(".png", full)[1].tofile(str(d / f"mask_{c.serial}.png"))
                rows_ = count_rows(cams, results, args.expect)
                write_csv(d / "counts.csv", rows_)
                write_csv(out_root / "counts_log.csv", rows_,
                          dict(time=f"{stamp:%Y-%m-%d %H:%M:%S}", snapshot=d.name), append=True)
                for r in rows_:
                    print(f"    {r['camera']}  {r['color']:7s} {r['blobs']}개  {r['status']}")
                say(f"saved {d.name}")
            elif ch == "r":
                if rec is None:
                    d = out_root / f"rec_{datetime.now():%Y%m%d_%H%M%S}"
                    d.mkdir(parents=True, exist_ok=True)
                    size = (canvas.shape[1] // 2 * 2, canvas.shape[0] // 2 * 2)
                    tmpv = Path(tempfile.gettempdir()) / f"maskview_{d.name}.mp4"   # 한글 경로는 VideoWriter 가 못 씀
                    wr = cv2.VideoWriter(str(tmpv), cv2.VideoWriter_fourcc(*"mp4v"), REC_FPS, size)
                    if not wr.isOpened():
                        say("REC failed: cannot open video writer")
                    else:
                        rec = dict(dir=d, tmp=tmpv, writer=wr, size=size, t0=time.time(), n=0, csv=d / "counts.csv")
                        say(f"REC start -> {d.name}")
                else:
                    rec["writer"].release()
                    shutil.move(str(rec["tmp"]), str(rec["dir"] / "screen.mp4"))
                    say(f"REC saved {rec['dir'].name}  ({rec['n'] / REC_FPS:.1f}s)")
                    rec = None
            elif ch == "q" or k == 27:
                break
            if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        pool.shutdown(wait=False)
        if rec:
            rec["writer"].release()
            shutil.move(str(rec["tmp"]), str(rec["dir"] / "screen.mp4"))
            print("녹화 저장:", rec["dir"])
        for c in cams:
            c.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
