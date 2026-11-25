"""
E4. 투명성 ② 저항 전달 — 환자 저항이 다르면 치료사가 느끼는 힘도 달라지는가 (양방향 vs 단방향)

셋업: 환자 글러브(G1)를 더미 손가락에 끼우고 스프링을 바꿔 가며 (약/중/강) + 고정 블록(완전 경직).
      치료사(G2)는 가이드 창 목표를 따라: 0.3 s 대기 → 굽히기(2 s) → 유지(2 s) → 펴기(2 s) → 쉬기(1 s), 반복.
모드:  bi  = 양방향 (KC1 = KC2 = K)
       uni = 단방향 (KC1 = K, KC2 = 0 → 치료사는 환자 저항을 못 느낌)
지표: 유지 구간에서 치료사 쪽 저항 장력(EXT), 환자 손가락이 따라온 curl, 추종 오차

--loads 는 '이름=값' 목록. 값은 스프링 강성(아무 단위나 일관되게, 고정 블록은 inf).
예)
  python e4_resistance.py --finger I --params params.json --K 0.1 --loads free=0 soft=1 medium=2 stiff=4 clamp=inf --reps 5
"""

import os
import math
import argparse

import numpy as np
import pandas as pd

import exp_common as ec
import analysis_common as ac

T_UP, T_HOLD, T_DOWN, T_REST = 2.0, 2.0, 2.0, 1.0


def parse_loads(items):
    out = []
    for it in items:
        name, val = it.split("=")
        out.append((name, float(val)))
    return out


def protocol(s):
    a = s.args
    s.run_zero()
    func, period = ec.traj_hold_cycles(a.c_lo, a.c_hi, T_UP, T_HOLD, T_DOWN, T_REST)
    for name, val in parse_loads(a.loads):
        s.ask(f"[부하 {name}] 환자 글러브(G1) 더미 손가락에 '{name}' 스프링/고정을 설치하세요"
              + (" (고정 블록 → curl 약 0.1 에서 고정)" if math.isinf(val) else ""))
        if s.sim:
            p1 = s.ctl.sim.get("G1", s.fid)
            if math.isinf(val):
                p1.mode, p1.clamp_c = "clamp", a.c_lo
            else:
                p1.mode, p1.k, p1.c_rest = "free", val, 0.0
        for mode in a.modes:
            trial = f"{name}_{mode}"
            kc2 = a.K if mode == "bi" else 0.0
            bc2 = a.B if mode == "bi" else 0.0
            s.set_gains(KC1=a.K, BC1=a.B, KC2=kc2, BC2=bc2)
            s.label(phase="E4", trial=trial, cond=f"{val:g}",
                    msg=f"{name} / {'양방향' if mode == 'bi' else '단방향'}: 목표선을 따라가세요")
            s.meta.setdefault("loads", {})[name] = val
            if a.auto:
                s.enable("G1", "auto")
                s.enable("G2", "auto")
            s.enable("G1", "coupling")
            if kc2 > 0:
                s.enable("G2", "coupling")
            if s.sim:
                p2 = s.ctl.sim.get("G2", s.fid)
                p2.mode = "human"
                t0 = s.ctl.t_now
                p2.human_ref = lambda t, t0=t0: func(t - t0 + 0.1)
            s.set_ref(func)
            s.event("TRIAL_START")
            ok = s.wait(period * a.reps, show=trial)
            s.end_trial()
            s.disable_all()
            if not ok:
                s.clear_estop_interactive()
            s.wait(2.0)


def analyze(folder, cal_path=None):
    folder, logp, meta = ac.resolve_run(folder)
    df = ac.load_log(logp)
    fid = meta.get("finger", "I")
    cal = ac.load_tension_cal(cal_path)
    a = meta.get("args", {})
    c_lo, c_hi = float(a.get("c_lo", 0.1)), float(a.get("c_hi", 0.7))
    period = T_UP + T_HOLD + T_DOWN + T_REST
    rows = []
    for trial in [t for t in dict.fromkeys(df["trial"]) if t]:
        d = df[df["trial"] == trial]
        st = d.index[d["event"] == "TRIAL_START"]
        if not len(st):
            continue
        d = d.loc[st[0]:]
        t_rel = (d["t"] - d["t"].iloc[0]).to_numpy(float)
        ph = t_rel % period
        hold = (ph > T_UP + 0.3 * T_HOLD) & (ph < T_UP + T_HOLD)     # 유지 구간 뒤쪽 70%
        rep = (t_rel // period).astype(int)
        fN, eN = ac.glove_tensions(d, "G2", fid, cal)
        name, mode = trial.rsplit("_", 1)
        for r in np.unique(rep[hold]):
            m = hold & (rep == r)
            if m.sum() < 5:
                continue
            rows.append(dict(load=name, stiffness=float(d["cond"].iloc[0] or "nan"), mode=mode, rep=int(r) + 1,
                             G2_resist_N=float(np.mean(eN[m])),
                             G2_net_N=float(np.mean(eN[m] - fN[m])),
                             G2_curl=float(d["G2_curl"].to_numpy(float)[m].mean()),
                             G1_curl=float(d["G1_curl"].to_numpy(float)[m].mean())))
    per = pd.DataFrame(rows)
    per.to_csv(os.path.join(folder, "e4_per_rep.csv"), index=False)
    if per.empty:
        print("데이터 없음")
        return per
    per["track_err"] = per["G2_curl"] - per["G1_curl"]
    agg = per.groupby(["load", "stiffness", "mode"], sort=False).agg(
        resist_mean=("G2_resist_N", "mean"), resist_sd=("G2_resist_N", "std"),
        G1_curl=("G1_curl", "mean"), track_err=("track_err", "mean"), n=("rep", "count")).reset_index()

    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(8.6, 3.4))
    loads = list(dict.fromkeys(agg["load"]))
    x = np.arange(len(loads))
    w = 0.38
    for i, (mode, col) in enumerate((("bi", "#1565c0"), ("uni", "#9e9e9e"))):
        r = agg[agg["mode"] == mode].set_index("load").reindex(loads)
        axes[0].bar(x + (i - 0.5) * w, r["resist_mean"], w, yerr=r["resist_sd"], capsize=3, color=col,
                    label="bilateral" if mode == "bi" else "unilateral")
        axes[1].plot(x, r["G1_curl"], "o-", color=col, label=("bilateral" if mode == "bi" else "unilateral"))
    axes[0].set_xticks(x, loads)
    axes[0].set_ylabel("Therapist resisting tension in hold [N]")
    axes[0].set_title("E4 resistance felt by therapist")
    axes[0].legend()
    axes[1].axhline(c_hi, color="k", ls=":", lw=1, label="target curl")
    axes[1].set_xticks(x, loads)
    axes[1].set_ylabel("Patient (G1) curl in hold")
    axes[1].set_title("E4 patient finger reached")
    axes[1].legend()
    ac.savefig(fig, folder, "e4_resistance.png")

    piv = agg.pivot_table(index="load", columns="mode", values="resist_mean", sort=False)
    txt = ""
    if {"bi", "uni"} <= set(piv.columns):
        bi = agg[agg["mode"] == "bi"]
        ok = bi.replace([np.inf, -np.inf], np.nan).dropna(subset=["stiffness"])
        if len(ok) >= 2:
            sl = np.polyfit(ok["stiffness"], ok["resist_mean"], 1)[0]
            txt = f"양방향: 스프링 강성 1 단위당 치료사 저항 증가 ≈ {sl:.3f} N (고정 블록 제외 선형 근사)\n\n"
        txt += "부하별 양방향/단방향 저항 [N]: " + ", ".join(
            f"{k}: {r['bi']:.2f} / {r['uni']:.2f}" for k, r in piv.iterrows())
    ac.write_summary(folder, "E4 저항 전달", [
        ("부하·모드별 평균", agg.round(3)),
        ("요약", txt),
        ("읽는 법", "양방향은 부하가 클수록 치료사 저항이 커져야 하고, 단방향은 부하와 무관하게 거의 평평해야 한다. "
                  "G1_curl 은 환자 손가락이 실제로 따라온 정도 (경직 조건에서 낮아짐)."),
    ], {"per_rep": rows})
    return agg


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ec.add_common_args(ap)
    ap.add_argument("--K", type=float, default=0.1)
    ap.add_argument("--B", type=float, default=0.002)
    ap.add_argument("--loads", nargs="+", default=["free=0", "soft=1", "medium=2", "stiff=4", "clamp=inf"])
    ap.add_argument("--modes", nargs="+", default=["bi", "uni"], choices=["bi", "uni"])
    ap.add_argument("--c-lo", type=float, default=0.1)
    ap.add_argument("--c-hi", type=float, default=0.7)
    ap.add_argument("--reps", type=int, default=5)
    ap.add_argument("--auto", action="store_true")
    ap.add_argument("--cal", default=None)
    a = ap.parse_args()
    if a.analyze_only:
        analyze(a.analyze_only, a.cal)
        return
    s = ec.Session("e4_resistance", a)
    try:
        ec.run_protocol(s, protocol, gui=not a.no_gui)
    finally:
        s.close()
    analyze(s.outdir, a.cal)


if __name__ == "__main__":
    main()
