"""
E0. 기초 특성화 — 이후 모든 실험의 단위 환산

  tension    : 추를 매달아 '전류 지령 장력 → 실제 장력' 보정식과 모터·스풀 마찰을 구한다
               (장력센서 없이: 올라가기 시작하는 장력 T_up, 내려가기 시작하는 장력 T_down
                → 가운데값 = 추 무게, 반폭 = 마찰)
  friction   : 글러브를 더미 손가락에 끼운 무부하 상태에서 굽힘/폄 기동 마찰(breakaway)
  resolution : 지도 + 엔코더 사양으로 해상도 계산, (선택) 비전 각도와 curl 회귀 → deg/curl

예)
  python e0_characterize.py tension  --glove G1 --finger I --tendon FLEX --mass 0.1 0.2 0.5 --reps 3
  python e0_characterize.py friction --glove G1 --finger I --reps 5
  python e0_characterize.py resolution --glove G1 --finger I
  python e0_characterize.py merge results/e0_tension_*/tension_cal.json -o tension_cal_all.json
  python e0_characterize.py resolution --glove G2 --finger I --log results/e3_xxx --vision vision.csv --vision-col index_pip_mcp_sum
"""

import os
import sys
import json
import time
import argparse

import numpy as np

import exp_common as ec
import analysis_common as ac

G = 9.81


# ── 공통: 수동 지령으로 한 모터 램프 ──

def _manual(ctl, f, which, T):
    with ctl.lock:
        ctl.manual = (T, 0.0) if which == "FLEX" else (0.0, T)
        ctl.manual_on = True


def _motor(f, which):
    return f.flex if which == "FLEX" else f.ext


def ramp_until_move(s, f, which, T0, rate, T_cap, move_deg, direction=None, ref_deg=None):
    """T0 에서 rate [N/s] 로 올리거나(rate>0) 내리면서(rate<0) 엔코더가 move_deg 움직이는 순간의 적용 장력.
    direction 이 주어지면 그 부호 방향 이동만 인정. 반환 (T_at_move, 이동 부호, 이동 시작 기준 deg) 또는 None."""
    ctl = s.ctl
    m = _motor(f, which)
    start = m.deg if ref_deg is None else ref_deg
    T = T0
    t_last = time.perf_counter()
    while True:
        now = time.perf_counter()
        T += rate * (now - t_last)
        t_last = now
        if (rate > 0 and T > T_cap) or (rate < 0 and T < 0.0):
            return None
        _manual(ctl, f, which, max(0.0, T))
        d = m.deg - start
        if direction is None and abs(d) >= move_deg:
            return m.applied_N, (1 if d > 0 else -1), start
        if direction is not None and direction * d >= move_deg:
            return m.applied_N, direction, start
        if ctl.estop:
            return None
        if direction is not None and rate < 0:
            # 내리는 중에는 기준을 '가장 많이 감긴 위치'로 갱신 (탄성 되돌림 오판 방지)
            if -direction * d > 0:
                start = m.deg
        time.sleep(0.01)


def _hold(s, f, which, T, sec):
    _manual(s.ctl, f, which, T)
    time.sleep(sec)


def setup_manual(s, gs):
    f = s.ctl.finger(gs)
    with s.ctl.lock:
        s.ctl.stop_all()
        s.ctl.mode = "map"
        s.ctl.cal_finger = f
        s.ctl.manual = (0.0, 0.0)
        s.ctl.manual_on = True
    return f


# ── tension ──

def cmd_tension(args):
    s = ec.Session("e0_tension", args, need_maps=False)
    key = f"{args.glove}_{args.finger}_{args.tendon}"
    rows = []
    try:
        f = setup_manual(s, args.glove)
        s.ask(f"[{key}] 글러브에서 이 텐던을 빼고, 모터 출구 직후 케이블에 추를 매달 준비를 하세요.\n"
              f"  추는 바닥/받침에 살짝 닿아 쉬는 상태에서 시작합니다. 최대 지령 {args.t_cap:.1f} N.")
        for mass in args.mass:
            W = mass * G
            s.ask(f"{mass*1000:.0f} g 추를 거세요 (W = {W:.2f} N)")
            if s.sim:
                s.ctl.sim.weights[key] = {"W": W, "Ff": 0.35, "x": 0.0}
            for r in range(args.reps):
                s.label(phase="tension", trial=f"{mass}kg_r{r+1}")
                _hold(s, f, args.tendon, 0.0, 1.0)
                up = ramp_until_move(s, f, args.tendon, 0.0, args.rate, min(args.t_cap, W * 2 + 2),
                                     args.move_deg)
                if up is None:
                    print("  올라가지 않음 (상한 도달) — 건너뜀")
                    _hold(s, f, args.tendon, 0.0, 1.0)
                    continue
                T_up, sgn, _ = up
                _hold(s, f, args.tendon, T_up, 0.5)
                dn = ramp_until_move(s, f, args.tendon, T_up, -args.rate, args.t_cap,
                                     args.move_deg / 2, direction=-sgn)
                T_dn = dn[0] if dn else float("nan")
                _hold(s, f, args.tendon, 0.0, 1.5)
                rows.append(dict(mass_kg=mass, W_N=W, rep=r + 1, T_up=T_up, T_down=T_dn))
                print(f"  {mass} kg r{r+1}: T_up={T_up:.2f}  T_down={T_dn:.2f}  (지령 N-eq)")
            if s.sim:
                s.ctl.sim.weights.pop(key, None)
    finally:
        s.close()
    analyze_tension(s.outdir, rows, key)


def analyze_tension(folder, rows, key):
    import pandas as pd
    import matplotlib.pyplot as plt
    if not rows:
        print("데이터 없음")
        return
    df = pd.DataFrame(rows)
    df.to_csv(os.path.join(folder, "tension_points.csv"), index=False)
    g = df.groupby("W_N").agg(T_up=("T_up", "mean"), T_down=("T_down", "mean"), n=("rep", "count")).reset_index()
    g["T_mid"] = (g["T_up"] + g["T_down"]) / 2
    g["friction_N"] = (g["T_up"] - g["T_down"]) / 2
    ok = g.dropna()
    alpha, beta = np.polyfit(ok["W_N"], ok["T_mid"], 1) if len(ok) >= 2 else (1.0, 0.0)
    pred = alpha * ok["W_N"] + beta
    r2 = 1 - np.sum((ok["T_mid"] - pred) ** 2) / max(1e-12, np.sum((ok["T_mid"] - ok["T_mid"].mean()) ** 2))
    fric = float(ok["friction_N"].mean())

    fig, ax = plt.subplots(figsize=(4.8, 3.6))
    ax.errorbar(g["W_N"], g["T_mid"], yerr=g["friction_N"], fmt="o", capsize=4, label="T_mid ± friction")
    xs = np.linspace(0, g["W_N"].max() * 1.1, 20)
    ax.plot(xs, alpha * xs + beta, "-", label=f"fit: {alpha:.3f}·W + {beta:.3f}")
    ax.plot(xs, xs, "k:", label="ideal (cmd = true)")
    ax.set_xlabel("Hanging weight W [N]")
    ax.set_ylabel("Commanded tension [N-eq]")
    ax.set_title(f"E0 current→tension: {key}")
    ax.legend()
    ac.savefig(fig, folder, "tension_cal.png")

    cal = {"motors": {key: {"alpha": float(alpha), "beta": float(beta), "r2": float(r2), "friction_N": fric}},
           "note": "T_cmd = alpha * T_true + beta. 보정 장력 = (T_cmd - beta) / alpha"}
    with open(os.path.join(folder, "tension_cal.json"), "w", encoding="utf-8") as fp:
        json.dump(cal, fp, indent=2)
    ac.write_summary(folder, f"E0 전류→장력 보정 ({key})", [
        ("추별 결과 [N]", g.round(3)),
        ("보정식", f"T_cmd = {alpha:.3f} × T_true + {beta:.3f}  (R² = {r2:.3f})\n\n"
                  f"평균 마찰(반폭) = {fric:.3f} N\n\n"
                  f"여러 모터 결과는 tension_cal.json 의 motors 를 하나로 합쳐 --cal 로 넘기면 됩니다."),
    ], cal)


# ── friction ──
# 주의: 텐던을 당기면 손가락이 안 움직여도 케이블/글러브 탄성 때문에 '당기는 쪽' 엔코더는 계속 돈다
# (5 N 에서 ~150°). 그래서 움직임 판정은 '반대쪽(길항) 엔코더'로 한다. 길항 텐던을 작은 장력(T_hold)으로
# 팽팽하게 유지하면, 그 스풀은 손가락이 실제로 움직일 때만 끌려 나간다.

def breakaway(s, f, which, rate, T_cap, move_deg, T_hold):
    ctl = s.ctl
    anta = f.ext if which == "FLEX" else f.flex
    agon = f.flex if which == "FLEX" else f.ext
    with ctl.lock:
        ctl.manual = (0.0, T_hold) if which == "FLEX" else (T_hold, 0.0)
    time.sleep(1.5)                              # 길항 텐던 take-up 안정
    start = anta.deg
    T = T_hold
    t_last = time.perf_counter()
    while T <= T_cap and not ctl.estop:
        now = time.perf_counter()
        T += rate * (now - t_last)
        t_last = now
        with ctl.lock:
            ctl.manual = (T, T_hold) if which == "FLEX" else (T_hold, T)
        if abs(anta.deg - start) >= move_deg:
            return agon.applied_N - T_hold       # 길항 장력을 뺀 '순' 기동 장력
        time.sleep(0.01)
    return float("nan")


def cmd_friction(args):
    s = ec.Session("e0_friction", args, need_maps=False)
    rows = []
    try:
        f = setup_manual(s, args.glove)
        if s.sim:
            sf = s.ctl.sim.get(args.glove, args.finger)
            sf.mode, sf.k, sf.fric, sf.c_rest = "free", 0.0, 0.6, 0.0
        s.ask(f"[{args.glove}-{args.finger}] 글러브를 더미 손가락(스프링 없음) 또는 힘을 뺀 손에 끼우세요")
        for r in range(args.reps):
            for which in ("FLEX", "EXT"):
                pose = "편 자세" if which == "FLEX" else "반쯤 굽힌 자세 (폄 방향으로 움직일 여유가 있게)"
                s.ask(f"{which} 시험 {r+1}/{args.reps}: 손가락을 {pose}로 두고 손을 떼세요")
                if s.sim:                       # 가상 손이 자세를 잡아 주고 손을 뗀다
                    tgt = 0.0 if which == "FLEX" else 0.5
                    sf.mode, sf.human_ref = "human", (lambda t, c=tgt: c)
                    time.sleep(1.5)
                    sf.mode = "free"
                s.label(phase="friction", trial=f"{which}_r{r+1}")
                T = breakaway(s, f, which, args.rate, args.t_cap, args.move_deg, args.t_hold)
                _manual(s.ctl, f, which, 0.0)
                time.sleep(1.0)
                rows.append(dict(tendon=which, rep=r + 1, T_breakaway=T))
                print(f"  {which} r{r+1}: 순 기동 장력 {T:.2f} N-eq")
    finally:
        s.close()
    import pandas as pd
    df = pd.DataFrame(rows)
    df.to_csv(os.path.join(s.outdir, "friction_points.csv"), index=False)
    g = df.groupby("tendon")["T_breakaway"].agg(["median", "mean", "std", "count"]).reset_index()
    data = {"glove": args.glove, "finger": args.finger, "t_hold": args.t_hold,
            "breakaway_N": {r.tendon: float(r["median"]) for _, r in g.iterrows()}}
    with open(os.path.join(s.outdir, "friction.json"), "w", encoding="utf-8") as fp:
        json.dump(data, fp, indent=2)
    ac.write_summary(s.outdir, f"E0 기동 마찰 ({args.glove}-{args.finger})", [
        ("순 기동 장력 [N-eq] (당긴 장력 − 길항 유지 장력)", g.round(3)),
        ("읽는 법", "이 값보다 작은 결합력은 손가락을 움직이지 못한다 = 치료사가 느낄 수 있는 힘의 바닥. "
                  "굽힘/폄 차이가 라우팅 마찰 비대칭이다."),
    ], data)


# ── resolution ──

def cmd_resolution(args):
    mod = ec.load_control_module(args.ctrl)
    path = os.path.join(args.map_dir, f"{args.glove}.json")
    if args.sim:
        sf, se = ec.SimPlant.SF, ec.SimPlant.SE
    else:
        with open(path, "r", encoding="utf-8") as fp:
            saved = json.load(fp)["fingers"][args.finger]
        m = mod.TendonMap()
        m.load_data(saved)
        sf, se = m._sf, m._se
    cnt_deg = 360.0 / (mod.ENCODER_PPR * 4)
    res = {
        "glove": args.glove, "finger": args.finger,
        "spool_deg_per_count": cnt_deg,
        "cable_mm_per_count": cnt_deg * mod.DEG_TO_MM,
        "flex_span_deg": sf, "ext_span_deg": se,
        "rom_mm_flex": abs(sf) * mod.DEG_TO_MM,
        "curl_per_count_flex": cnt_deg / abs(sf),
        "curl_per_count_ext": cnt_deg / abs(se),
        "control_period_s": mod.CTRL_PERIOD,
    }
    blocks = [("엔코더·지도 기반 해상도", "\n".join(f"- {k}: {v:.5g}" if isinstance(v, float) else f"- {k}: {v}"
                                          for k, v in res.items()))]
    os.makedirs(args.out, exist_ok=True)
    folder = os.path.join(args.out, f"e0_resolution_{args.glove}{args.finger}_{time.strftime('%Y%m%d_%H%M%S')}")
    os.makedirs(folder, exist_ok=True)

    if args.log and args.vision:
        _, logp, _ = ac.resolve_run(args.log)
        df = ac.load_log(logp)
        df = ac.attach_vision(df, args.vision, args.vision_col)
        c = df[f"{args.glove}_curl"].to_numpy(float)
        v = df["vision_deg"].to_numpy(float)
        ok = ~(np.isnan(c) | np.isnan(v))
        if ok.sum() > 20:
            k, b = np.polyfit(c[ok], v[ok], 1)
            pred = k * c[ok] + b
            r2 = 1 - np.sum((v[ok] - pred) ** 2) / np.sum((v[ok] - v[ok].mean()) ** 2)
            resid = float(np.std(v[ok] - pred))
            res.update(deg_per_curl=float(k), vision_offset_deg=float(b), vision_r2=float(r2),
                       vision_resid_sd_deg=resid,
                       joint_deg_per_count=float(k * res["curl_per_count_flex"]))
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(figsize=(4.6, 3.6))
            ax.plot(c[ok][::5], v[ok][::5], ".", ms=2, alpha=0.5, label="samples")
            xs = np.linspace(np.nanmin(c[ok]), np.nanmax(c[ok]), 20)
            ax.plot(xs, k * xs + b, "r-", label=f"{k:.1f} deg/curl, R²={r2:.2f}")
            ax.set_xlabel(f"{args.glove} curl (encoder map)")
            ax.set_ylabel(f"Vision {args.vision_col} [deg]")
            ax.legend()
            ac.savefig(fig, folder, "curl_vs_vision.png")
            blocks.append(("비전 대비 curl", f"- deg_per_curl = {k:.2f} (R² {r2:.3f}, 잔차 SD {resid:.2f}°)\n"
                                           f"- 관절각 해상도 ≈ {res['joint_deg_per_count']:.3f}°/count\n"
                                           f"- E7 의 --deg-per-curl 에 이 값을 넣으세요."))
    with open(os.path.join(folder, "resolution.json"), "w", encoding="utf-8") as fp:
        json.dump(res, fp, indent=2)
    ac.write_summary(folder, f"E0 해상도 ({args.glove}-{args.finger})", blocks, res)


def cmd_merge(args):
    allm = {}
    for p in args.files:
        with open(p, "r", encoding="utf-8") as fp:
            allm.update(json.load(fp).get("motors", {}))
    with open(args.output, "w", encoding="utf-8") as fp:
        json.dump({"motors": allm, "note": "T_cmd = alpha * T_true + beta"}, fp, indent=2)
    print(f"{len(allm)}개 모터 → {args.output}: {', '.join(allm)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    t = sub.add_parser("tension")
    ec.add_common_args(t)
    t.add_argument("--glove", default="G1", choices=("G1", "G2"))
    t.add_argument("--tendon", default="FLEX", choices=("FLEX", "EXT"))
    t.add_argument("--mass", type=float, nargs="+", default=[0.1, 0.2, 0.5], help="kg")
    t.add_argument("--reps", type=int, default=3)
    t.add_argument("--rate", type=float, default=0.3, help="N/s 램프 속도")
    t.add_argument("--move-deg", type=float, default=4.0, help="움직임 판정 스풀 각도")
    t.add_argument("--t-cap", type=float, default=12.0, help="지령 상한 N (IRB 20 N 이하)")
    t.set_defaults(fn=cmd_tension)

    fr = sub.add_parser("friction")
    ec.add_common_args(fr)
    fr.add_argument("--glove", default="G1", choices=("G1", "G2"))
    fr.add_argument("--reps", type=int, default=5)
    fr.add_argument("--rate", type=float, default=0.2)
    fr.add_argument("--move-deg", type=float, default=3.0, help="길항 엔코더 이동 판정 [deg]")
    fr.add_argument("--t-cap", type=float, default=5.0)
    fr.add_argument("--t-hold", type=float, default=0.3, help="길항 텐던 유지 장력 N")
    fr.set_defaults(fn=cmd_friction)

    rs = sub.add_parser("resolution")
    ec.add_common_args(rs)
    rs.add_argument("--glove", default="G1", choices=("G1", "G2"))
    rs.add_argument("--log", default=None, help="curl 이 기록된 실험 로그 (비전 회귀용)")
    rs.add_argument("--vision", default=None, help="비전 CSV (unix_time_s + 각도 열)")
    rs.add_argument("--vision-col", default="angle_deg")
    rs.set_defaults(fn=cmd_resolution)

    mg = sub.add_parser("merge", help="모터별 tension_cal.json 합치기")
    mg.add_argument("files", nargs="+")
    mg.add_argument("-o", "--output", default="tension_cal_all.json")
    mg.set_defaults(fn=cmd_merge)

    args = ap.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
