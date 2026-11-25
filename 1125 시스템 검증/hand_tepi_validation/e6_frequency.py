"""
E6. 주파수 응답 — 어느 속도의 움직임까지 상대 손가락이 따라오는가

셋업: 치료사 글러브(G2)를 '모터로 움직이는 가상 사람'으로 사인파 구동 (더미 손가락 또는 힘 뺀 손 위)
      환자 글러브(G1)는 더미 손가락(자유) — 결합력(KC1, BC1)으로만 따라온다.
지표: 주파수별 진폭비(G1/G2, 실제 G2 움직임 기준)와 위상 지연, −3 dB 대역폭.
      서보 자체가 목표를 얼마나 따라갔는지(G2/ref)도 같이 기록.

주의: 원래 코드의 장력 램프 제한(RAMP_UP 60 N/s, RAMP_DOWN 12 N/s)이 높은 주파수에서
      결합력 변화를 늦춘다. 대역폭이 여기서 막히는지도 이 실험에서 드러난다.

예)
  python e6_frequency.py --finger I --params params.json --K 0.1 --B 0.002 --freqs 0.25 0.5 1 1.5 2 3
  서보 게인: --servo-kp 20 --servo-kd 0.8 --servo-tmax 6 (작게 시작해서 튜닝)
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
    s.ask("G2(구동 쪽)는 더미 손가락 또는 힘을 뺀 손, G1 은 더미 손가락(자유)에 끼우세요.\n"
          "  G2 가 모터로 움직입니다. 사람 손이면 힘을 완전히 빼고 있으세요.")
    if s.sim:
        p1, p2 = s.ctl.sim.get("G1", s.fid), s.ctl.sim.get("G2", s.fid)
        p1.mode, p1.k = "free", 0.3
        p2.mode, p2.k = "free", 0.3
    servo = ec.CurlServo(s.ctl.ref, kp=a.servo_kp, kd=a.servo_kd, t_base=a.servo_tbase, t_max=a.servo_tmax)
    s.set_gains(KC1=a.K, BC1=a.B, KC2=0.0, BC2=0.0)
    # 시작 자세로 천천히
    s.label(phase="E6", trial="", msg="시작 자세로 이동")
    s.set_ref(ec.traj_ramp(s.f2.curl, a.mid, 2.0))
    s.drive("G2", servo)
    s.wait(2.5)
    for fr in a.freqs:
        trial = f"f{fr:g}"
        cyc = max(a.cycles, int(math.ceil(a.min_time * fr)))
        s.label(phase="E6", trial=trial, msg=f"{fr:g} Hz 구동 중 ({cyc}주기)")
        s.enable("G1", "coupling")
        s.set_ref(ec.traj_sine(a.mid, a.amp, fr, lead_in=1.0))
        s.event("TRIAL_START")
        ok = s.wait(1.0 + cyc / fr, show=trial)
        s.end_trial()
        s.set_ref(ec.traj_hold(a.mid))
        s.enable("G1", "coupling", False)
        if not ok:
            s.disable_all()
            s.clear_estop_interactive()
            s.drive("G2", servo)
        s.wait(1.5)
    s.disable_all()


def analyze(folder):
    folder, logp, meta = ac.resolve_run(folder)
    df = ac.load_log(logp)
    rows = []
    for trial in [t for t in dict.fromkeys(df["trial"]) if t.startswith("f")]:
        d = df[df["trial"] == trial]
        st = d.index[d["event"] == "TRIAL_START"]
        if not len(st):
            continue
        d = d.loc[st[0]:]
        d = d[d["t"] >= d["t"].iloc[0] + 1.0]
        fr = float(trial[1:])
        tu, sg = ac.uniform(d, ["ref", "G2_curl", "G1_curl"], 200.0)
        if len(tu) < 50:
            continue
        Ar, pr, _ = ac.sine_fit(tu, sg["ref"], fr)
        A2, p2, _ = ac.sine_fit(tu, sg["G2_curl"], fr)
        A1, p1, _ = ac.sine_fit(tu, sg["G1_curl"], fr)
        dph = math.degrees((p1 - p2 + math.pi) % (2 * math.pi) - math.pi)
        dph_s = math.degrees((p2 - pr + math.pi) % (2 * math.pi) - math.pi)
        rows.append(dict(freq_hz=fr, ref_amp=Ar, G2_amp=A2, G1_amp=A1,
                         gain=A1 / A2 if A2 > 0 else np.nan,
                         gain_db=20 * math.log10(A1 / A2) if A1 > 0 and A2 > 0 else np.nan,
                         phase_deg=dph, servo_gain=A2 / Ar if Ar > 0 else np.nan, servo_phase_deg=dph_s,
                         G1_cpl_max_N=float(np.nanmax(d[["G1_cpl_f", "G1_cpl_e"]].to_numpy(float)))))
    res = pd.DataFrame(rows).sort_values("freq_hz")
    res.to_csv(os.path.join(folder, "e6_results.csv"), index=False)
    bw = float("nan")
    if len(res) >= 2:
        g = res["gain_db"].to_numpy()
        f = res["freq_hz"].to_numpy()
        below = np.where(g < -3)[0]
        if len(below) and below[0] > 0:
            i = below[0]
            bw = float(np.interp(-3, [g[i], g[i - 1]], [f[i], f[i - 1]]))
        elif len(below) and below[0] == 0:
            bw = float(f[0])

    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 1, figsize=(5.2, 5.2), sharex=True)
    axes[0].semilogx(res["freq_hz"], res["gain_db"], "o-", label="follower G1 / leader G2")
    axes[0].semilogx(res["freq_hz"], 20 * np.log10(res["servo_gain"]), "s--", alpha=0.6, label="servo G2 / ref")
    axes[0].axhline(-3, color="k", ls=":", lw=1, label="−3 dB")
    axes[0].set_ylabel("Gain [dB]")
    axes[0].set_title(f"E6 frequency response (bandwidth ≈ {bw:.2f} Hz)")
    axes[0].legend()
    axes[1].semilogx(res["freq_hz"], res["phase_deg"], "o-", label="G1 vs G2")
    axes[1].semilogx(res["freq_hz"], res["servo_phase_deg"], "s--", alpha=0.6, label="G2 vs ref")
    axes[1].set_xlabel("Frequency [Hz]")
    axes[1].set_ylabel("Phase [deg]")
    axes[1].legend()
    ac.savefig(fig, folder, "e6_bode.png")
    ac.write_summary(folder, "E6 주파수 응답", [
        ("주파수별 결과", res.round(3)),
        ("대역폭", f"−3 dB 대역폭 ≈ {bw:.2f} Hz (K={meta.get('args', {}).get('K')}, B={meta.get('args', {}).get('B')})"),
        ("읽는 법", "gain 은 실제 G2 움직임 대비 G1 이 따라온 비율. servo_gain 이 1 에서 많이 벗어나면 구동 자체가 "
                  "목표를 못 따라간 것이니 G1/G2 비율로만 판단."),
    ], {"rows": rows, "bandwidth_hz": bw})
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ec.add_common_args(ap)
    ap.add_argument("--K", type=float, default=0.1)
    ap.add_argument("--B", type=float, default=0.002)
    ap.add_argument("--freqs", type=float, nargs="+", default=[0.25, 0.5, 1.0, 1.5, 2.0, 3.0])
    ap.add_argument("--mid", type=float, default=0.4)
    ap.add_argument("--amp", type=float, default=0.25, help="ROM 의 약 30%% 권장")
    ap.add_argument("--cycles", type=int, default=8)
    ap.add_argument("--min-time", type=float, default=6.0, help="주파수당 최소 측정 시간 [s]")
    ap.add_argument("--servo-kp", type=float, default=20.0)
    ap.add_argument("--servo-kd", type=float, default=0.8)
    ap.add_argument("--servo-tbase", type=float, default=0.3)
    ap.add_argument("--servo-tmax", type=float, default=6.0)
    a = ap.parse_args()
    if a.analyze_only:
        analyze(a.analyze_only)
        return
    s = ec.Session("e6_frequency", a)
    try:
        ec.run_protocol(s, protocol, gui=not a.no_gui)
    finally:
        s.close()
    analyze(s.outdir)


if __name__ == "__main__":
    main()
