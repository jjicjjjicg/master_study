"""
E7. 지각 역치 — 치료사가 환자 손가락의 작은 움직임을 실제로 느끼는가 (치료사 요구: 2~3°)

셋업: 환자 글러브(G1) = 더미 손가락 + '모터로 움직이는 가상 환자' (기준 curl 유지 → 작은 변위 → 복귀)
      참가자 = 치료사 글러브(G2) 착용, 손 힘을 빼고 편하게. 눈가림 + 이어폰(백색소음).
      결합: KC2 = K (치료사가 받는 힘), KC1 = 0 (G1 은 서보가 움직임)
시행: 변위 {0(가짜), 1, 2, 3, 5, 8}° × 방향(굽힘/폄) 무작위, 각 reps 회.
      자극 = 무작위 대기 → 0.3 s 램프 → 1.0 s 유지 → 0.3 s 복귀
응답: 느꼈다(y)/못 느꼈다(n), 느꼈으면 방향(f=굽힘 / e=폄)
분석: 변위별 감지율, 가짜 시행 오경보율, 방향 정답률, 로지스틱 곡선 → 75% 감지 역치 [°]

관절각 [°] ↔ curl 환산은 E0 resolution(비전 회귀)의 deg_per_curl 을 --deg-per-curl 로 넣는다.
모터 소리로 답을 알 수 있으면 --mask 로 모든 시행(가짜 포함)에서 같은 동시수축을 걸어 소리를 맞춘다.

※ 사람 참가자 실험 — 현재 IRB 승인 범위에 포함되는지 먼저 확인할 것.

예)
  python e7_perception.py --finger I --params params.json --deg-per-curl 85 --K 0.1 --deltas 0 1 2 3 5 8 --reps 8
"""

import os
import csv
import math
import time
import random
import argparse

import numpy as np
import pandas as pd

import exp_common as ec
import analysis_common as ac

T_RAMP, T_HOLD = 0.3, 1.0


def protocol(s):
    a = s.args
    s.run_zero()
    s.ask("G1 은 더미 손가락, 참가자는 G2 를 끼고 손 힘을 뺍니다. 눈가림·이어폰 착용.")
    if s.sim:
        p1, p2 = s.ctl.sim.get("G1", s.fid), s.ctl.sim.get("G2", s.fid)
        p1.mode, p1.k = "free", 0.3
        p2.mode, p2.k, p2.c_rest = "free", 2.0, a.c0
    servo = ec.CurlServo(s.ctl.ref, kp=a.servo_kp, kd=a.servo_kd, t_base=a.servo_tbase, t_max=a.servo_tmax)
    s.set_ref(ec.traj_ramp(s.f1.curl, a.c0, 2.0))
    s.drive("G1", servo)
    peak = {"on": False, "F": 0.0}

    def peak_mon(ctl, dt):                      # 자극 중 치료사가 받은 결합력 최대값
        if peak["on"]:
            f2 = ctl.finger("G2")
            peak["F"] = max(peak["F"], abs(f2.cpl_f - f2.cpl_e))
    with s.ctl.lock:
        s.ctl.monitors.append(peak_mon)
    s.wait(2.5)
    tpath = os.path.join(s.outdir, "trials.csv")
    fp = open(tpath, "w", newline="", encoding="utf-8-sig")
    w = csv.writer(fp)
    w.writerow(["block_K", "trial", "delta_deg", "direction", "delta_curl_cmd", "t_stim", "peak_cpl_G2_N",
                "response", "resp_dir", "rt_s"])
    rng = random.Random(a.seed)
    try:
        for K in a.K:
            s.set_gains(KC1=0.0, BC1=0.0, KC2=K, BC2=a.B)
            s.enable("G2", "coupling")
            trials = [(d, sgn) for d in a.deltas for sgn in ((+1, -1) if d > 0 else (0,)) for _ in
                      range(a.reps if d > 0 else 2 * a.reps)]
            rng.shuffle(trials)
            s.ask(f"[블록 K={K:g}] 총 {len(trials)}회. 시작합니다")
            for i, (deg, sgn) in enumerate(trials, 1):
                dcurl = sgn * deg / a.deg_per_curl
                trial = f"K{K:g}_t{i:03d}"
                s.label(phase="E7", trial=trial, cond=f"{sgn * deg:+g}", msg=f"시행 {i}/{len(trials)} — 집중하세요")
                if not s.sim:
                    input(f"\n  [{i}/{len(trials)}] 참가자 준비되면 Enter ")
                t_wait = rng.uniform(0.8, 1.6)
                with s.ctl.lock:
                    s.ctl.ref.set(ec.traj_pulse(a.c0, dcurl, t_wait, T_RAMP, T_HOLD), s.ctl.t_now)
                s.event("TRIAL_START")
                s.wait(t_wait)
                servo.cocon = a.mask
                peak.update(on=True, F=0.0)
                s.event("STIM_ON")
                t_stim = s.ctl.t_now
                s.wait(2 * T_RAMP + T_HOLD)
                servo.cocon = 0.0
                peak["on"] = False
                s.event("STIM_OFF")
                s.wait(0.4)
                t_ask = time.perf_counter()
                if s.sim:
                    resp, rdir = _sim_response(s, peak["F"], rng)
                else:
                    resp = _ask_choice("  느꼈나요? (y/n) ", "yn")
                    rdir = _ask_choice("  방향? (f=굽힘 / e=폄) ", "fe") if resp == "y" else ""
                rt = time.perf_counter() - t_ask
                s.end_trial()
                w.writerow([K, trial, deg, sgn, dcurl, t_stim, round(peak["F"], 3), resp, rdir, round(rt, 2)])
                fp.flush()
                if s.ctl.estop:
                    s.clear_estop_interactive()
                    s.drive("G1", servo)
                    s.enable("G2", "coupling")
            s.enable("G2", "coupling", False)
    finally:
        fp.close()


def _ask_choice(msg, keys):
    while True:
        r = input(msg).strip().lower()
        if r[:1] in keys:
            return r[:1]


def _sim_response(s, dF, rng):
    """가상 참가자: 자극 중 치료사 쪽 결합력 최대값이 0.25 N(+잡음) 넘으면 감지."""
    seen = dF + rng.gauss(0, 0.08) > 0.25 or rng.random() < 0.05
    if not seen:
        return "n", ""
    true_dir = s.ctl.labels.get("cond", "+0")[0]
    rdir = ("f" if true_dir == "+" else "e") if rng.random() < 0.9 else ("e" if true_dir == "+" else "f")
    return "y", rdir


# ── 분석 ──

def fit_threshold(x, y, gamma, lam=0.02, crit=0.75):
    """p(x) = γ + (1−γ−λ)/(1+exp(−(x−α)/β)), 격자 최대우도. 반환 (α, β, x@crit)."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    best, ab = -np.inf, (np.nan, np.nan)
    for al in np.linspace(0.1, 15, 150):
        for be in np.linspace(0.1, 5, 50):
            p = gamma + (1 - gamma - lam) / (1 + np.exp(-(x - al) / be))
            p = np.clip(p, 1e-6, 1 - 1e-6)
            ll = np.sum(y * np.log(p) + (1 - y) * np.log(1 - p))
            if ll > best:
                best, ab = ll, (al, be)
    al, be = ab
    if crit <= gamma or crit >= 1 - lam:
        return al, be, np.nan
    xc = al - be * math.log((1 - gamma - lam) / (crit - gamma) - 1)
    return al, be, xc


def analyze(folder, deg_per_curl=None):
    folder, logp, meta = ac.resolve_run(folder)
    tr = pd.read_csv(os.path.join(folder, "trials.csv"), encoding="utf-8-sig")
    df = ac.load_log(logp)
    dpc = deg_per_curl or float(meta.get("args", {}).get("deg_per_curl", 1.0))
    # 실제로 일어난 G1 변위 (유지 구간 평균 − 자극 전 0.3 s 평균)
    act = []
    for _, r in tr.iterrows():
        d = df[df["trial"] == r["trial"]]
        on = d.index[d["event"] == "STIM_ON"]
        if not len(on):
            act.append(np.nan)
            continue
        t_on = d.loc[on[0], "t"]
        pre = d[(d["t"] > t_on - 0.3) & (d["t"] <= t_on)]["G1_curl"].mean()
        hold = d[(d["t"] > t_on + T_RAMP + 0.3) & (d["t"] < t_on + T_RAMP + T_HOLD)]["G1_curl"].mean()
        act.append((hold - pre) * dpc)
    tr["actual_deg"] = act
    tr["hit"] = (tr["response"] == "y").astype(int)
    tr["dir_ok"] = np.where(tr["hit"] == 1,
                            ((tr["direction"] > 0) & (tr["resp_dir"] == "f")) | ((tr["direction"] < 0) & (tr["resp_dir"] == "e")),
                            np.nan)
    rows, curves = [], []
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(5.0, 3.6))
    for K, b in tr.groupby("block_K"):
        fa = b[b["delta_deg"] == 0]["hit"].mean() if (b["delta_deg"] == 0).any() else 0.0
        stim = b[b["delta_deg"] > 0]
        g = stim.groupby("delta_deg").agg(hit_rate=("hit", "mean"), n=("hit", "size"),
                                          peak_cpl_N=("peak_cpl_G2_N", "mean"),
                                          actual_deg=("actual_deg", lambda v: np.nanmean(np.abs(v))),
                                          dir_acc=("dir_ok", "mean")).reset_index()
        g.insert(0, "K", K)
        rows.append(g)
        x = np.abs(stim["actual_deg"].fillna(stim["delta_deg"]).to_numpy(float))
        al, be, x75 = fit_threshold(x, stim["hit"].to_numpy(float), gamma=min(fa, 0.5))
        curves.append(dict(K=K, false_alarm=fa, alpha=al, beta=be, threshold75_deg=x75,
                           dir_acc_overall=float(np.nanmean(stim["dir_ok"]))))
        line = ax.plot(g["actual_deg"], g["hit_rate"], "o", label=f"K={K:g} data")[0]
        ax.plot([0], [fa], "x", ms=8, color=line.get_color(), label=f"K={K:g} false alarm (catch)")
        xs = np.linspace(0, max(10, x.max() * 1.1), 100)
        gam = min(fa, 0.5)
        ax.plot(xs, gam + (1 - gam - 0.02) / (1 + np.exp(-(xs - al) / be)), "-", color=line.get_color(),
                label=f"K={K:g} fit, 75% = {x75:.1f}°")
    ax.axhline(0.75, color="k", ls=":", lw=1)
    ax.axvspan(2, 3, color="#ffcc80", alpha=0.3, label="therapist requirement 2–3°")
    ax.set_xlabel("Actual patient-finger displacement [deg]")
    ax.set_ylabel("Detection rate")
    ax.set_ylim(-0.02, 1.02)
    ax.set_title("E7 detection threshold")
    ax.legend(fontsize=7)
    ac.savefig(fig, folder, "e7_psychometric.png")
    tab = pd.concat(rows) if rows else pd.DataFrame()
    cur = pd.DataFrame(curves)
    tr.to_csv(os.path.join(folder, "trials_scored.csv"), index=False)
    ac.write_summary(folder, "E7 지각 역치", [
        ("변위별 감지율", tab.round(3)),
        ("역치 (K별)", cur.round(3)),
        ("읽는 법", "threshold75_deg 가 2~3° 이하면 치료사 요구 충족. false_alarm 이 높으면(>0.2) 소리 등 다른 단서가 "
                  "있는 것이니 --mask 로 다시. actual_deg 가 명령값보다 작으면 더미 손가락/글러브가 변위를 흡수한 것."),
    ], {"blocks": curves})
    return cur


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ec.add_common_args(ap)
    ap.add_argument("--deg-per-curl", type=float, required=False, default=None,
                    help="E0 resolution 결과 (관절각 °/curl). 실험 시 필수")
    ap.add_argument("--K", type=float, nargs="+", default=[0.1])
    ap.add_argument("--B", type=float, default=0.002)
    ap.add_argument("--deltas", type=float, nargs="+", default=[0, 1, 2, 3, 5, 8], help="관절각 [deg]")
    ap.add_argument("--reps", type=int, default=8, help="변위·방향별 반복 (가짜 시행은 2배)")
    ap.add_argument("--c0", type=float, default=0.3, help="기준 curl")
    ap.add_argument("--mask", type=float, default=0.0, help="자극 구간 동시수축 [N] (소리 마스킹)")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--servo-kp", type=float, default=20.0)
    ap.add_argument("--servo-kd", type=float, default=0.8)
    ap.add_argument("--servo-tbase", type=float, default=0.3)
    ap.add_argument("--servo-tmax", type=float, default=6.0)
    a = ap.parse_args()
    if a.analyze_only:
        analyze(a.analyze_only, a.deg_per_curl)
        return
    if a.deg_per_curl is None:
        if a.sim:
            a.deg_per_curl = 90.0
        else:
            ap.error("--deg-per-curl 이 필요합니다 (E0 resolution 의 비전 회귀 결과)")
    s = ec.Session("e7_perception", a)
    try:
        ec.run_protocol(s, protocol, gui=not a.no_gui)
    finally:
        s.close()
    analyze(s.outdir, a.deg_per_curl)


if __name__ == "__main__":
    main()
