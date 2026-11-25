"""
E3. 투명성 ① 자유 운동 — 상대가 저항하지 않을 때 따라가는 정확도와 치료사가 느끼는 잔여 저항

셋업: 환자 글러브(G1)를 더미 손가락(스프링 없음, 자유) 또는 힘을 뺀 사람 손에.
      치료사(G2)가 가이드 창의 사인파 목표를 따라 움직인다 (느리게 / 보통 / 빠르게).
조건: coupled  — 양방향 결합 ON (K, B)
      baseline — 결합 OFF (--auto 면 G2 Auto-tensioning 만 ON = '텐던만 따라가는' 상태)
지표: 추종 오차 RMSE (G1 vs G2 curl), 지연(상호상관), 치료사 쪽 잔여 저항
      (굽힐 때는 EXT 장력, 펼 때는 FLEX 장력 = 움직임을 방해하는 쪽 텐던)

예)
  python e3_free_motion.py --finger I --params params.json --K 0.1 --B 0.002 --freqs 0.25 0.5 1.0 --auto
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
    s.ask("환자 글러브(G1)를 더미 손가락(자유) 또는 힘을 뺀 손에 끼우세요")
    if s.sim:
        p1 = s.ctl.sim.get("G1", s.fid)
        p1.mode, p1.k = "free", 0.3
    for cond in a.conds:
        for fr in a.freqs:
            trial = f"{cond}_f{fr:g}"
            func = ec.traj_sine(a.mid, a.amp, fr, lead_in=1.5)
            dur = a.cycles / fr + 1.5
            s.ask(f"[{trial}] 치료사가 목표선을 {fr:g} Hz 로 {a.cycles}회 따라갑니다")
            s.label(phase="E3", trial=trial, cond=cond, msg=f"{cond}: 목표선을 따라가세요 ({fr:g} Hz)")
            if cond == "coupled":
                s.set_gains(KC1=a.K, BC1=a.B, KC2=a.K, BC2=a.B)
            if a.auto:
                s.enable("G2", "auto")
                if cond == "coupled":
                    s.enable("G1", "auto")
            if cond == "coupled":
                s.enable("G1", "coupling")
                s.enable("G2", "coupling")
            if s.sim:
                p2 = s.ctl.sim.get("G2", s.fid)
                p2.mode = "human"
                t0 = s.ctl.t_now
                p2.human_ref = lambda t, t0=t0, func=func: func(t - t0 + 0.08)
            s.set_ref(func)
            s.event("TRIAL_START")
            ok = s.wait(dur, show=trial)
            s.end_trial()
            s.disable_all()
            if not ok:
                s.clear_estop_interactive()
            s.wait(2.0)


def analyze(folder, cal_path=None, vthr=0.05, skip_s=1.5):
    folder, logp, meta = ac.resolve_run(folder)
    df = ac.load_log(logp)
    fid = meta.get("finger", "I")
    cal = ac.load_tension_cal(cal_path)
    rows = []
    for trial in [t for t in dict.fromkeys(df["trial"]) if t]:
        d = df[df["trial"] == trial]
        st = d.index[d["event"] == "TRIAL_START"]
        if len(st):
            d = d.loc[st[0]:]
        d = d[d["t"] >= d["t"].iloc[0] + skip_s]          # lead-in 제외
        if len(d) < 30:
            continue
        cond, fr = trial.split("_f")
        fr = float(fr)
        fs = 200.0
        tu, sg = ac.uniform(d, ["G1_curl", "G2_curl", "ref"], fs)
        fN, eN = ac.glove_tensions(d, "G2", fid, cal)
        v = d["G2_curl_vel"].to_numpy(float)
        resist = np.where(v > vthr, eN, np.where(v < -vthr, fN, np.nan))
        row = dict(cond=cond, freq_hz=fr,
                   G2_amp=float(ac.sine_fit(tu, sg["G2_curl"], fr)[0]),
                   resist_mean_N=float(np.nanmean(resist)),
                   resist_p95_N=float(np.nanpercentile(resist, 95)) if np.isfinite(resist).any() else np.nan,
                   resist_flexing_N=float(np.nanmean(np.where(v > vthr, eN, np.nan))),
                   resist_extending_N=float(np.nanmean(np.where(v < -vthr, fN, np.nan))))
        if cond == "coupled":
            row["rmse_curl"] = ac.rmse(sg["G1_curl"], sg["G2_curl"])
            row["lag_s"] = ac.xcorr_lag(sg["G2_curl"], sg["G1_curl"], fs, 0.6)
            A1 = ac.sine_fit(tu, sg["G1_curl"], fr)[0]
            row["amp_ratio_G1_G2"] = A1 / row["G2_amp"] if row["G2_amp"] > 0 else np.nan
        rows.append(row)
    res = pd.DataFrame(rows)
    res.to_csv(os.path.join(folder, "e3_results.csv"), index=False)

    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(8.4, 3.4))
    for cond, mk in (("coupled", "o-"), ("baseline", "s--")):
        r = res[res["cond"] == cond].sort_values("freq_hz")
        if len(r):
            axes[0].plot(r["freq_hz"], r["resist_mean_N"], mk, label=f"{cond} mean")
            axes[0].plot(r["freq_hz"], r["resist_p95_N"], mk, alpha=0.4, label=f"{cond} p95")
    axes[0].set_xlabel("Movement frequency [Hz]")
    axes[0].set_ylabel("Therapist-side resisting tension [N]")
    axes[0].set_title("E3 residual resistance")
    axes[0].legend()
    r = res[res["cond"] == "coupled"].sort_values("freq_hz")
    if len(r):
        axes[1].plot(r["freq_hz"], r["rmse_curl"], "o-", label="RMSE curl")
        ax2 = axes[1].twinx()
        ax2.plot(r["freq_hz"], r["lag_s"] * 1000, "s--", color="#c62828", label="lag [ms]")
        ax2.set_ylabel("Lag G1 behind G2 [ms]")
        ax2.legend(loc="lower right")
    axes[1].set_xlabel("Movement frequency [Hz]")
    axes[1].set_ylabel("Tracking RMSE [curl]")
    axes[1].set_title("E3 tracking (coupled)")
    axes[1].legend(loc="upper left")
    ac.savefig(fig, folder, "e3_free_motion.png")

    extra = ""
    if len(res[res["cond"] == "baseline"]) and len(r):
        b = res[res["cond"] == "baseline"].set_index("freq_hz")["resist_mean_N"]
        c = r.set_index("freq_hz")["resist_mean_N"]
        add = (c - b).dropna()
        extra = "결합이 추가한 저항(coupled − baseline, 평균) [N]: " + ", ".join(f"{k:g} Hz → {v:.2f}" for k, v in add.items())
    ac.write_summary(folder, "E3 자유 운동 투명성", [
        ("조건·속도별 결과", res.round(3)),
        ("결합이 추가한 저항", extra or "(baseline 없음)"),
        ("읽는 법", "resist = 치료사 움직임을 방해하는 쪽 텐던의 모터단 장력. 굽힘/폄 차이는 라우팅 비대칭. "
                  "lag 가 클수록 환자 손이 늦게 따라옴."),
    ], {"rows": rows})
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ec.add_common_args(ap)
    ap.add_argument("--K", type=float, default=0.1)
    ap.add_argument("--B", type=float, default=0.002)
    ap.add_argument("--freqs", type=float, nargs="+", default=[0.25, 0.5, 1.0])
    ap.add_argument("--conds", nargs="+", default=["coupled", "baseline"], choices=["coupled", "baseline"])
    ap.add_argument("--mid", type=float, default=0.4)
    ap.add_argument("--amp", type=float, default=0.3)
    ap.add_argument("--cycles", type=int, default=8)
    ap.add_argument("--auto", action="store_true", help="Auto-tensioning ON (baseline 정의에 필요)")
    ap.add_argument("--cal", default=None)
    a = ap.parse_args()
    if a.analyze_only:
        analyze(a.analyze_only, a.cal)
        return
    s = ec.Session("e3_free_motion", a)
    try:
        ec.run_protocol(s, protocol, gui=not a.no_gui)
    finally:
        s.close()
    analyze(s.outdir, a.cal)


if __name__ == "__main__":
    main()
