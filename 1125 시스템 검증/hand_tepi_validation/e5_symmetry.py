"""
E5. 대칭성 — 같은 Δcurl 에서 두 장갑이 같은 크기의 힘을 받는가 (분석 전용, 추가 실험 없음)

입력: E2/E4 등 양방향 결합 로그 폴더들 (역할을 바꿔 다시 찍은 로그가 있으면 같이)
      + 선택: E0 tension_cal.json (--cal), E0 friction.json 들 (--friction)

중요 — 현재 코드 구조상 '같은 K' 가 '같은 힘'을 보장하지 않는다:
  couple() 의 힘 = KC × (스풀 각도 오차) = KC × |지도 스팬| × Δcurl
  → 두 장갑의 스풀 스팬(sf, se)이 다르면 같은 Δcurl 에서도 힘이 다르다.
  → 굽힘(sf)과 폄(se) 스팬이 달라도 방향별로 다르다.
  이 스크립트는 지도에서 이론 비율을 계산하고, 로그에서 실제 비율을 잰 뒤,
  대칭을 맞추는 보정 게인(KC2 = KC1 × |s1| / |s2|)을 제안한다.

예)
  python e5_symmetry.py results/e2_stiffness_xxx results/e4_resistance_xxx --cal tension_cal_all.json
"""

import os
import json
import argparse

import numpy as np
import pandas as pd

import exp_common as ec
import analysis_common as ac


def slope(x, y, db=0.02):
    m = (np.abs(x) > db) & np.isfinite(y)
    return float(np.polyfit(x[m], y[m], 1)[0]) if m.sum() > 20 else np.nan


def analyze_run(path, cal):
    folder, logp, meta = ac.resolve_run(path)
    df = ac.load_log(logp)
    outs = []
    for trial in [t for t in dict.fromkeys(df["trial"]) if t]:
        on = df[(df["trial"] == trial) & (df["G1_cpl_on"] == 1) & (df["G2_cpl_on"] == 1)]
        if len(on) >= 50:
            outs.append(analyze_one(on, meta, f"{os.path.basename(folder)}:{trial}", cal))
    return outs


def analyze_one(on, meta, name, cal):
    fid = meta.get("finger", "I")
    dc = (on["G2_curl"] - on["G1_curl"]).to_numpy(float)          # +: 치료사가 더 굽힘
    out = {"run": name, "finger": fid, "n": len(on)}
    KC1, KC2 = float(on["KC1"].median()), float(on["KC2"].median())
    spans = meta.get("maps", {})
    s1, s2 = spans.get(f"G1_{fid}", {}), spans.get(f"G2_{fid}", {})
    # 이론: dc>0 → G1 은 FLEX 로 끌려가고(굽힘), G2 는 EXT 로 끌려감(폄)
    out["theory_G1_flex_N_per_curl"] = KC1 * abs(s1.get("sf_deg", np.nan))
    out["theory_G2_ext_N_per_curl"] = KC2 * abs(s2.get("se_deg", np.nan))
    out["theory_G1_ext_N_per_curl"] = KC1 * abs(s1.get("se_deg", np.nan))
    out["theory_G2_flex_N_per_curl"] = KC2 * abs(s2.get("sf_deg", np.nan))
    # 실측 (지령 / 적용)
    f1, e1 = ac.glove_tensions(on, "G1", fid, cal)
    f2, e2 = ac.glove_tensions(on, "G2", fid, cal)
    pos, neg = dc > 0, dc < 0
    out["meas_G1_flex_N_per_curl"] = slope(dc[pos], f1[pos])
    out["meas_G2_ext_N_per_curl"] = slope(dc[pos], e2[pos])
    out["meas_G1_ext_N_per_curl"] = slope(-dc[neg], e1[neg])
    out["meas_G2_flex_N_per_curl"] = slope(-dc[neg], f2[neg])
    out["ratio_dc_pos (G1flex/G2ext)"] = out["meas_G1_flex_N_per_curl"] / out["meas_G2_ext_N_per_curl"]
    out["ratio_dc_neg (G1ext/G2flex)"] = out["meas_G1_ext_N_per_curl"] / out["meas_G2_flex_N_per_curl"]
    if s1 and s2:
        out["suggest_KC2_for_symmetry_flexside"] = KC1 * abs(s1["sf_deg"]) / abs(s2["se_deg"])
        out["suggest_KC2_for_symmetry_extside"] = KC1 * abs(s1["se_deg"]) / abs(s2["sf_deg"])
    out["_dc"], out["_f1"], out["_e2"], out["_e1"], out["_f2"] = dc, f1, e2, e1, f2
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", help="양방향 결합 로그 폴더들")
    ap.add_argument("--cal", default=None)
    ap.add_argument("--friction", nargs="*", default=[], help="E0 friction.json 들")
    ap.add_argument("--out", default="results")
    a = ap.parse_args()
    cal = ac.load_tension_cal(a.cal)
    folder = os.path.join(a.out, f"e5_symmetry_{pd.Timestamp.now():%Y%m%d_%H%M%S}")
    os.makedirs(folder, exist_ok=True)
    outs = [o for r in a.runs for o in analyze_run(r, cal)]
    if not outs:
        print("양쪽 결합이 켜진 구간이 있는 로그가 없습니다")
        return
    import matplotlib.pyplot as plt
    nc = min(4, len(outs))
    nr = (len(outs) + nc - 1) // nc
    fig, axes = plt.subplots(nr, nc, figsize=(4.2 * nc, 3.4 * nr), squeeze=False)
    for ax in axes.flat[len(outs):]:
        ax.set_visible(False)
    for ax, o in zip(axes.flat, outs):
        dc = o["_dc"]
        k = slice(None, None, 4)
        ax.plot(dc[k], o["_f1"][k], ".", ms=2, label="G1 flex (patient pulled to flex)")
        ax.plot(dc[k], o["_e2"][k], ".", ms=2, label="G2 ext (therapist pulled back)")
        ax.plot(dc[k], o["_e1"][k], ".", ms=2, label="G1 ext")
        ax.plot(dc[k], o["_f2"][k], ".", ms=2, label="G2 flex")
        ax.set_xlabel("Δcurl (G2 − G1)")
        ax.set_ylabel("Motor-side tension [N]")
        ax.set_title(f"E5 {o['run']}", fontsize=8)
        ax.legend(markerscale=4)
    ac.savefig(fig, folder, "e5_symmetry.png")
    table = pd.DataFrame([{k: v for k, v in o.items() if not k.startswith("_")} for o in outs])
    table.to_csv(os.path.join(folder, "e5_results.csv"), index=False)
    fr = []
    for p in a.friction:
        with open(p, "r", encoding="utf-8") as fp:
            d = json.load(fp)
        fr.append(dict(glove=d["glove"], finger=d["finger"], **{f"breakaway_{k}": v for k, v in d["breakaway_N"].items()}))
    blocks = [("로그별 강성·비율 (이상적 비율 = 1)", table.T.reset_index().rename(columns={"index": "item"}))]
    if fr:
        blocks.append(("E0 기동 마찰 비교 [N]", pd.DataFrame(fr)))
    blocks.append(("읽는 법", "theory = KC × 지도 스팬. meas = 로그의 적용 장력 기울기. ratio 가 1 에서 벗어난 만큼 "
                              "치료사와 환자가 받는 힘이 다르다. suggest_KC2 는 지도 스팬 차이만 보정하는 값이고 "
                              "라우팅 마찰 비대칭은 E0 friction 으로 따로 본다."))
    ac.write_summary(folder, "E5 대칭성", blocks, {"runs": table.to_dict("records")})


if __name__ == "__main__":
    main()
