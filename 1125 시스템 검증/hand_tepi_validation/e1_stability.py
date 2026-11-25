"""
E1. 안정성 맵 — K(결합 강성) × B(결합 댐핑) 격자에서 떨림 없이 쓸 수 있는 범위

조건 (기본 3개):
  free    : 두 사람이 양쪽 글러브를 끼고 자유롭게 굽혔다 폈다
  clamp   : 환자 쪽(G1)을 고정 블록에 묶고 치료사(G2)가 밀기  (hard contact)
  release : 치료사가 크게 굽혔다가 갑자기 힘을 빼기          (가장 불안정해지기 쉬움)

안전:
  - 결합오차의 고주파(>3 Hz) RMS 가 임계값을 0.25 s 넘으면 자동으로 결합을 끈다 (flag=1)
  - 같은 B 에서 어떤 K 가 불안정이면 그보다 큰 K 는 건너뛴다
  - K 는 작은 값부터. Space/Esc(가이드 창) 또는 Ctrl+C = 비상정지

예)
  python e1_stability.py --finger I --params params.json --K 0.02 0.05 0.1 0.2 --B 0 0.001 0.003
  python e1_stability.py --analyze-only results/e1_stability_20261125_xxxx
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
    order = []
    for cond in a.conditions:
        for B in a.B:
            for K in sorted(a.K):
                order.append((cond, K, B))
    s.meta["grid"] = {"K": a.K, "B": a.B, "conditions": a.conditions, "duration": a.duration}
    unstable_at = {}                     # (cond, B) → 불안정이 처음 나온 K
    instr = {
        "free": "두 사람 모두 글러브 착용, 치료사가 천천히~보통 속도로 굽혔다 폈다 하세요",
        "clamp": "환자 쪽(G1) 손가락을 고정 블록에 묶고, 치료사가 굽히는 방향으로 밀었다 풀기를 반복하세요",
        "release": "치료사가 크게 굽혀 힘을 받은 상태에서, 신호마다 손가락 힘을 '툭' 빼세요 (3~4회)",
    }
    cur_cond = None
    for cond, K, B in order:
        if K > unstable_at.get((cond, B), math.inf):
            print(f"  [{cond} K{K:g} B{B:g}] 건너뜀 (더 작은 K 에서 불안정)")
            continue
        if cond != cur_cond:
            s.ask(f"[조건 {cond}] {instr[cond]}")
            cur_cond = cond
        trial = f"{cond}_K{K:g}_B{B:g}"
        s.label(phase="E1", trial=trial, cond=cond, msg=f"{cond}: {instr[cond]}")
        s.set_gains(KC1=K, BC1=B, KC2=K, BC2=B)
        s.ctl.flag = 0
        _sim_behaviour(s, cond)
        with s.ctl.lock:
            s.ctl.monitors[:] = [ec.instability_monitor(a.thr)]
        if a.auto:
            s.enable("G1", "auto")
            s.enable("G2", "auto")
        s.enable("G1", "coupling")
        s.enable("G2", "coupling")
        s.event("TRIAL_START")
        print(f"\n[{trial}] {a.duration:.0f}s")
        ok = s.wait(a.duration, show="진행")
        s.end_trial()
        unstable = s.ctl.flag == 1 or not ok
        s.disable_all()
        if not ok:
            s.clear_estop_interactive()
        if unstable:
            print(f"  → 불안정 감지. 같은 조건·B 에서 K>{K:g} 는 건너뜀")
            unstable_at[(cond, B)] = min(K, unstable_at.get((cond, B), math.inf))
        s.wait(a.rest)


def _sim_behaviour(s, cond):
    if not s.sim:
        return
    p1, p2 = s.ctl.sim.get("G1", s.fid), s.ctl.sim.get("G2", s.fid)
    p1.mode, p1.k, p1.c_rest = "free", 0.5, 0.0
    p2.mode = "human"
    t0 = s.ctl.t_now
    if cond == "free":
        p2.human_ref = lambda t: 0.4 + 0.3 * math.sin(2 * math.pi * 0.5 * (t - t0))
    elif cond == "clamp":
        p1.mode, p1.clamp_c = "clamp", 0.2
        p2.human_ref = lambda t: 0.3 + 0.25 * (1 + math.sin(2 * math.pi * 0.3 * (t - t0)))
    else:
        p2.k, p2.c_rest = 3.0, 0.0       # 힘을 빼면 글러브 탄성으로 펴지는 손
        # 2.5 s 주기로 힘 빼기
        def ref(t):
            ph = (t - t0) % 2.5
            p2.kh = 60.0 if ph < 1.8 else 0.0
            return 0.8
        p2.human_ref = ref


# ── 분석 ──

def analyze(folder, thr=0.03):
    folder, logp, meta = ac.resolve_run(folder)
    df = ac.load_log(logp)
    df = df[df["trial"] != ""]
    rows = []
    for trial, d in df.groupby("trial", sort=False):
        on = d[(d["G1_cpl_on"] == 1) | (d["G2_cpl_on"] == 1)]
        if len(on) < 20:
            on = d
        fs = 200.0
        tu, sig = ac.uniform(on, ["G1_curl", "G2_curl"], fs)
        e = sig["G1_curl"] - sig["G2_curl"] if len(tu) else np.array([])
        hp = ac.highpass(e, fs, 3.0) if len(e) else e
        fpk, apk = ac.osc_peak(e, fs, 2.0)
        unstable = bool((d["flag"] == 1).any() or (d["estop"] == 1).any())
        settle = float("nan")
        cond = d["cond"].iloc[0]
        if cond == "release" and len(hp):
            # 힘을 뺀 순간 = 치료사 curl 이 빠르게 펴지기 시작하는 지점 (−1 curl/s 이하, 1 s 간격)
            env = pd.Series(np.abs(hp)).rolling(int(0.2 * fs), min_periods=1).max().to_numpy()
            _, v = ac.uniform(on, ["G2_curl_vel"], fs)
            fall = np.where((v["G2_curl_vel"][1:] < -1.0) & (v["G2_curl_vel"][:-1] >= -1.0))[0]
            st, last = [], -1e9
            for k0 in fall:
                if (k0 - last) / fs < 1.0:
                    continue
                last = k0
                calm = env[k0:] < thr / 2
                hold = int(0.3 * fs)
                for j in range(len(calm) - hold):
                    if calm[j:j + hold].all():
                        st.append(j / fs)
                        break
            settle = float(np.median(st)) if st else float("nan")
        rows.append(dict(trial=trial, cond=cond, K=float(d["KC1"].iloc[-1]), B=float(d["BC1"].iloc[-1]),
                         stable="N" if unstable else "Y", hp_rms=float(np.sqrt(np.mean(hp ** 2))) if len(hp) else np.nan,
                         osc_hz=fpk, osc_amp=apk, settle_s=settle,
                         dur_s=float(d["t"].iloc[-1] - d["t"].iloc[0])))
    res = pd.DataFrame(rows)
    res.to_csv(os.path.join(folder, "e1_trials.csv"), index=False)

    import matplotlib.pyplot as plt
    conds = list(dict.fromkeys(res["cond"]))
    fig, axes = plt.subplots(1, len(conds), figsize=(4.0 * len(conds), 3.4), squeeze=False)
    for ax, c in zip(axes[0], conds):
        r = res[res["cond"] == c]
        for st, col, mk in (("Y", "#2e7d32", "o"), ("N", "#c62828", "x")):
            q = r[r["stable"] == st]
            ax.scatter(q["K"], q["B"], c=col, marker=mk, s=60, label="stable" if st == "Y" else "unstable")
        ax.set_title(f"E1 stability — {c}")
        ax.set_xlabel("K  [N / spool deg]")
        ax.set_ylabel("B  [N·s / spool deg]")
        ax.legend(loc="upper left")
    ac.savefig(fig, folder, "e1_stability_map.png")

    hz = meta.get("hz_last", "?")
    kmax = res[res["stable"] == "Y"].groupby("cond")["K"].max().rename("K_max_stable").reset_index()
    ac.write_summary(folder, "E1 안정성 맵", [
        ("시행별 결과", res[["cond", "K", "B", "stable", "hp_rms", "osc_hz", "settle_s"]].round(4)),
        ("조건별 최대 안정 K", kmax),
        ("메모", f"제어 주기 ≈ {hz} Hz. K 단위는 스풀 각도 기준 N/deg "
               f"(× 지도 스팬 → N/curl, ÷ 0.0663 → N/mm 케이블)."),
    ], {"trials": rows})
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ec.add_common_args(ap)
    ap.add_argument("--K", type=float, nargs="+", default=[0.02, 0.05, 0.1, 0.2])
    ap.add_argument("--B", type=float, nargs="+", default=[0.0, 0.001, 0.003])
    ap.add_argument("--conditions", nargs="+", default=["free", "clamp", "release"],
                    choices=["free", "clamp", "release"])
    ap.add_argument("--duration", type=float, default=10.0)
    ap.add_argument("--rest", type=float, default=2.0)
    ap.add_argument("--thr", type=float, default=0.03, help="불안정 판정 고주파 RMS [curl]")
    ap.add_argument("--auto", action="store_true", help="Auto-tensioning 도 같이 켬 (params 의 PID 사용)")
    a = ap.parse_args()
    if a.analyze_only:
        analyze(a.analyze_only, a.thr)
        return
    s = ec.Session("e1_stability", a)
    try:
        ec.run_protocol(s, protocol, gui=not a.no_gui)
    finally:
        s.close()
    analyze(s.outdir, a.thr)


if __name__ == "__main__":
    main()
