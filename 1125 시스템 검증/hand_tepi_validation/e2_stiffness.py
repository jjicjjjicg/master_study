"""
E2. 렌더링 강성 — 설정한 K 가 실제로 얼마의 강성으로 구현되는가

셋업: 환자 글러브(G1)를 더미 손가락에 끼우고 고정 블록으로 특정 curl 에 묶는다.
      치료사(G2)가 가이드 창의 목표를 따라 아주 천천히 굽혔다 폈다 한다 (삼각파).
측정: Δcurl = G2 − G1 (+ 선택: 비전 관절각) vs 치료사 쪽 모터단 장력
결과: 기울기 = 유효 강성 [N/curl], 설정값(K × 지도 스팬)과의 비, 히스테리시스 루프 면적, G1 고정 드리프트

예)
  python e2_stiffness.py --finger I --params params.json --K 0.05 0.1 0.2 --cycles 3
  python e2_stiffness.py --analyze-only results/e2_stiffness_xxx --cal tension_cal_all.json
"""

import os
import math
import argparse

import numpy as np
import pandas as pd

import exp_common as ec
import analysis_common as ac


def protocol(s):
    a = s.args
    s.run_zero()
    s.ask(f"환자 글러브(G1)를 더미 손가락에 끼우고 curl ≈ {a.clamp_curl:.1f} 위치에서 고정 블록으로 묶으세요")
    if s.sim:
        p1 = s.ctl.sim.get("G1", s.fid)
        p1.mode, p1.clamp_c = "clamp", a.clamp_curl
    for K in a.K:
        trial = f"K{K:g}"
        s.set_gains(KC1=K, BC1=a.B, KC2=K, BC2=a.B)
        s.label(phase="E2", trial=trial, cond="clamped",
                msg="검은 선(목표)을 천천히 따라가세요 — 치료사 G2")
        period = 1.0 / a.freq
        func = ec.traj_triangle(a.c_lo, a.c_hi, period)
        s.ask(f"[K={K:g}] 치료사가 G2 를 끼고 목표선을 따라 {a.cycles}회 왕복합니다")
        if s.sim:
            p2 = s.ctl.sim.get("G2", s.fid)
            p2.mode = "human"
            t0 = s.ctl.t_now
            p2.human_ref = lambda t, t0=t0: func(t - t0)
        if a.auto:
            s.enable("G2", "auto")
        s.enable("G1", "coupling")
        s.enable("G2", "coupling")
        s.set_ref(func)
        s.event("TRIAL_START")
        ok = s.wait(period * a.cycles, show=trial)
        s.end_trial()
        s.disable_all()
        if not ok:
            s.clear_estop_interactive()
        s.wait(2.0)


def analyze(folder, cal_path=None, vision=None, vision_col="angle_deg", deadband=0.02):
    folder, logp, meta = ac.resolve_run(folder)
    df = ac.load_log(logp)
    fid = meta.get("finger", "I")
    cal = ac.load_tension_cal(cal_path)
    if vision:
        df = ac.attach_vision(df, vision, vision_col)
    rows = []
    import matplotlib.pyplot as plt
    trials = [t for t in dict.fromkeys(df["trial"]) if t]
    fig, axes = plt.subplots(1, max(1, len(trials)), figsize=(3.8 * max(1, len(trials)), 3.4), squeeze=False)
    for ax, trial in zip(axes[0], trials):
        d = df[(df["trial"] == trial) & (df["G2_cpl_on"] == 1)]
        if len(d) < 20:
            continue
        K = float(d["KC2"].iloc[-1])
        dc = (d["G2_curl"] - d["G1_curl"]).to_numpy(float)
        fN, eN = ac.glove_tensions(d, "G2", fid, cal)
        Fnet = eN - fN                       # +: 폄 쪽으로 잡아당김 (치료사가 앞섰을 때 느끼는 저항)
        Fcmd = (d["G2_cpl_e"] - d["G2_cpl_f"]).to_numpy(float)
        m = np.abs(dc) > deadband
        slope_app = np.polyfit(dc[m], Fnet[m], 1)[0] if m.sum() > 10 else np.nan
        slope_cmd = np.polyfit(dc[m], Fcmd[m], 1)[0] if m.sum() > 10 else np.nan
        kf, ke = (ec.k_per_curl(meta, "G2", fid, K) if "maps" in meta else (np.nan, np.nan))
        area = float(abs(np.trapezoid(Fnet, dc))) / max(1, _cycles(d))
        drift = float(d["G1_curl"].max() - d["G1_curl"].min())
        row = dict(K=K, K_set_ext_N_per_curl=ke, K_set_flex_N_per_curl=kf,
                   slope_cmd=slope_cmd, slope_applied=slope_app,
                   ratio_applied_vs_set=slope_app / ke if ke else np.nan,
                   loop_area_N_curl=area, G1_drift_curl=drift,
                   dc_max=float(np.nanmax(np.abs(dc))))
        if "vision_deg" in d:
            vd = d["vision_deg"].to_numpy(float)
            ok = m & ~np.isnan(vd)
            if ok.sum() > 10:
                row["slope_vs_vision_N_per_deg"] = float(np.polyfit(vd[ok] - np.nanmin(vd[ok]), Fnet[ok], 1)[0])
        rows.append(row)
        ax.plot(dc, Fnet, "-", lw=0.8, label="applied (motor-side)")
        ax.plot(dc, Fcmd, ":", lw=0.8, label="coupling command")
        xs = np.linspace(np.nanmin(dc), np.nanmax(dc), 10)
        if ke == ke:
            ax.plot(xs, ke * xs, "k--", lw=1, label=f"set: {ke:.1f} N/curl")
        ax.set_title(f"E2  K={K:g}  slope={slope_app:.1f} N/curl")
        ax.set_xlabel("Δcurl (G2 − G1)")
        ax.set_ylabel("G2 net tension ext−flex [N]")
        ax.legend()
    ac.savefig(fig, folder, "e2_force_vs_dcurl.png")
    res = pd.DataFrame(rows)
    res.to_csv(os.path.join(folder, "e2_results.csv"), index=False)
    ac.write_summary(folder, "E2 렌더링 강성", [
        ("K별 결과", res.round(3)),
        ("읽는 법", "slope_cmd 는 제어기가 의도한 값(지도 기준이라 거의 설정값과 같음). slope_applied 는 램프 제한·"
                  "soft-start·포화 뒤 실제로 낸 모터단 장력 기준. 비전 열이 있으면 실제 관절각 기준 강성도 계산. "
                  "loop_area 는 왕복 1회당 히스테리시스(마찰·방향전환 지연), G1_drift 는 고정했는데도 움직인 양(탄성)."),
    ], {"rows": rows, "cal": bool(cal)})
    return res


def _cycles(d):
    ev = (d["event"] == "TRIAL_START").sum()
    a = (d["G2_curl"] - d["G2_curl"].mean()).to_numpy(float)
    return max(1, int(np.sum((a[:-1] < 0) & (a[1:] >= 0))) or ev)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ec.add_common_args(ap)
    ap.add_argument("--K", type=float, nargs="+", default=[0.05, 0.1, 0.2])
    ap.add_argument("--B", type=float, default=0.0, help="강성만 보려면 0 권장")
    ap.add_argument("--clamp-curl", type=float, default=0.3)
    ap.add_argument("--c-lo", type=float, default=0.1)
    ap.add_argument("--c-hi", type=float, default=0.7)
    ap.add_argument("--freq", type=float, default=0.1, help="삼각파 주파수 [Hz] (천천히)")
    ap.add_argument("--cycles", type=int, default=3)
    ap.add_argument("--auto", action="store_true")
    ap.add_argument("--cal", default=None, help="E0 tension_cal.json (motors 합본)")
    ap.add_argument("--vision", default=None)
    ap.add_argument("--vision-col", default="angle_deg")
    a = ap.parse_args()
    if a.analyze_only:
        analyze(a.analyze_only, a.cal, a.vision, a.vision_col)
        return
    s = ec.Session("e2_stiffness", a)
    try:
        ec.run_protocol(s, protocol, gui=not a.no_gui)
    finally:
        s.close()
    analyze(s.outdir, a.cal, a.vision, a.vision_col)


if __name__ == "__main__":
    main()
