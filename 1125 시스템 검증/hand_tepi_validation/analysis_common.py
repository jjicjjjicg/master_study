"""analysis_common.py — 실험 로그 분석 공통 함수 (numpy / pandas / matplotlib)."""

import os
import json
import math

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({"font.size": 9, "axes.titlesize": 10, "axes.labelsize": 9,
                     "legend.fontsize": 8, "figure.dpi": 120, "axes.grid": True,
                     "grid.alpha": 0.3})


# ── 입출력 ──

def resolve_run(path):
    """폴더 또는 log.csv 경로 → (폴더, log 경로, meta dict)."""
    if os.path.isdir(path):
        folder, log = path, os.path.join(path, "log.csv")
    else:
        folder, log = os.path.dirname(path) or ".", path
    meta = {}
    mp = os.path.join(folder, "meta.json")
    if os.path.isfile(mp):
        with open(mp, "r", encoding="utf-8") as fp:
            meta = json.load(fp)
    return folder, log, meta


def load_log(path):
    df = pd.read_csv(path, encoding="utf-8-sig", low_memory=False)
    for c in ("phase", "trial", "cond", "event", "mode"):
        if c in df:
            df[c] = df[c].fillna("").astype(str)
    return df


def savefig(fig, folder, name):
    p = os.path.join(folder, name)
    fig.tight_layout()
    fig.savefig(p, dpi=150)
    plt.close(fig)
    return p


def write_summary(folder, title, blocks, data=None):
    """blocks: [(소제목, DataFrame 또는 str)] → summary.md, data → summary.json"""
    lines = [f"# {title}", ""]
    for head, body in blocks:
        lines += [f"## {head}", ""]
        if isinstance(body, pd.DataFrame):
            lines.append(df_to_md(body))
        else:
            lines.append(str(body))
        lines.append("")
    with open(os.path.join(folder, "summary.md"), "w", encoding="utf-8") as fp:
        fp.write("\n".join(lines))
    if data is not None:
        with open(os.path.join(folder, "summary.json"), "w", encoding="utf-8") as fp:
            json.dump(data, fp, ensure_ascii=False, indent=2, default=_jsonable)
    print("\n".join(lines))


def _jsonable(o):
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    return str(o)


def df_to_md(df, floatfmt="{:.3g}"):
    cols = list(df.columns)
    out = ["| " + " | ".join(map(str, cols)) + " |", "| " + " | ".join("---" for _ in cols) + " |"]
    for _, r in df.iterrows():
        cells = []
        for c in cols:
            v = r[c]
            if isinstance(v, (float, np.floating)):
                cells.append("" if np.isnan(v) else floatfmt.format(v))
            else:
                cells.append(str(v))
        out.append("| " + " | ".join(cells) + " |")
    return "\n".join(out)


# ── 신호 처리 ──

def uniform(df, cols, fs=200.0, tcol="t"):
    t = df[tcol].to_numpy(float)
    if len(t) < 3:
        return np.array([]), {c: np.array([]) for c in cols}
    tu = np.arange(t[0], t[-1], 1.0 / fs)
    return tu, {c: np.interp(tu, t, pd.to_numeric(df[c], errors="coerce").to_numpy(float)) for c in cols}


def highpass(x, fs, fc):
    a = 1.0 / (1.0 + 2 * math.pi * fc / fs)
    y = np.zeros_like(x)
    for i in range(1, len(x)):
        y[i] = a * (y[i - 1] + x[i] - x[i - 1])
    return y


def xcorr_lag(a, b, fs, max_lag=1.0):
    """b 가 a 보다 얼마나 늦는지 [s] (양수 = b 가 뒤따름)."""
    a = a - a.mean()
    b = b - b.mean()
    n = int(max_lag * fs)
    best, lag_best = -np.inf, 0
    for k in range(-n, n + 1):
        if k >= 0:
            v = np.dot(a[:len(a) - k], b[k:])
        else:
            v = np.dot(a[-k:], b[:len(b) + k])
        if v > best:
            best, lag_best = v, k
    return lag_best / fs


def sine_fit(t, x, f):
    """x ≈ A·sin(2πft + φ) + c 최소제곱. (A, φ[rad], c)"""
    w = 2 * math.pi * f
    M = np.column_stack([np.sin(w * t), np.cos(w * t), np.ones_like(t)])
    s, c, off = np.linalg.lstsq(M, x, rcond=None)[0]
    return math.hypot(s, c), math.atan2(c, s), off


def osc_peak(x, fs, fmin=2.0):
    """detrend 후 fmin 이상 대역의 최대 스펙트럼 피크 (주파수, 진폭)."""
    if len(x) < 16:
        return float("nan"), float("nan")
    x = x - np.polyval(np.polyfit(np.arange(len(x)), x, 1), np.arange(len(x)))
    X = np.abs(np.fft.rfft(x * np.hanning(len(x)))) * 4 / len(x)
    fr = np.fft.rfftfreq(len(x), 1.0 / fs)
    m = fr >= fmin
    if not m.any():
        return float("nan"), float("nan")
    i = np.argmax(X[m])
    return fr[m][i], X[m][i]


def rmse(a, b):
    return float(np.sqrt(np.nanmean((np.asarray(a) - np.asarray(b)) ** 2)))


# ── 장력 보정 (E0 결과) ──

def load_tension_cal(path):
    if not path:
        return {}
    with open(path, "r", encoding="utf-8") as fp:
        return json.load(fp).get("motors", {})


def calibrate(T_cmd, cal, motor_key):
    """모터단 지령 장력 → 추 보정 장력. 보정이 없으면 그대로."""
    c = cal.get(motor_key)
    if not c:
        return T_cmd
    return (np.asarray(T_cmd) - c["beta"]) / c["alpha"]


def glove_tensions(df, gs, fid, cal=None):
    """(flex_N, ext_N) — 보정 json 이 있으면 보정값."""
    cal = cal or {}
    fN = calibrate(df[f"{gs}_flex_N"].to_numpy(float), cal, f"{gs}_{fid}_FLEX")
    eN = calibrate(df[f"{gs}_ext_N"].to_numpy(float), cal, f"{gs}_{fid}_EXT")
    return np.clip(fN, 0, None), np.clip(eN, 0, None)


# ── 비전 정렬 (선택) ──

def attach_vision(df, vision_csv, col, out_col="vision_deg", tcol="unix_time_s"):
    """비전 CSV(unix_time_s + 각도 열)를 로그 시간축에 보간해 붙인다."""
    v = pd.read_csv(vision_csv)
    vt = "unix_time_s" if "unix_time_s" in v else ("timestamp" if "timestamp" in v else v.columns[0])
    v = v[[vt, col]].dropna().sort_values(vt)
    df[out_col] = np.interp(df[tcol].to_numpy(float), v[vt].to_numpy(float), v[col].to_numpy(float),
                            left=np.nan, right=np.nan)
    return df
