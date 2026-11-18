"""Analysis helpers for rov-depth-sim logs: step-response metrics and plots."""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from typing import Optional

import numpy as np
import pandas as pd

STATE_ORDER = ["NORMAL", "WARNING", "SURFACING", "FAULT_LATCHED"]
STATE_COLORS = {
    "NORMAL": "#2e7d32",
    "WARNING": "#f9a825",
    "SURFACING": "#ef6c00",
    "FAULT_LATCHED": "#c62828",
}


@dataclass
class StepMetrics:
    target_m: float
    overshoot_pct: float
    rise_time_s: Optional[float]       # 10% -> 90% of the step
    settling_time_s: Optional[float]   # last time outside the +-band, None if never settles
    steady_state_error_m: float        # mean error over the final window
    peak_depth_m: float


def load_log(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    required = {"time_s", "true_depth_m", "measured_depth_m", "thrust_n", "battery_frac", "state"}
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"log is missing columns: {sorted(missing)}")
    return df


def compute_step_metrics(
    time_s,
    depth_m,
    target_m: float,
    band_fraction: float = 0.02,
    final_window_s: float = 10.0,
) -> StepMetrics:
    """Classic step-response metrics for a 0 -> target_m step.

    * overshoot: how far the peak exceeds the target, as a percentage of the step size
    * rise time: time from 10% to 90% of the target
    * settling time: the last instant the response is outside +-band of the target
    * steady-state error: mean(depth - target) over the last `final_window_s` seconds
    """
    t = np.asarray(time_s, dtype=float)
    z = np.asarray(depth_m, dtype=float)
    if t.shape != z.shape or t.size < 2:
        raise ValueError("time and depth must be equal-length arrays with at least 2 samples")
    if target_m <= 0:
        raise ValueError("target must be positive")

    peak = float(z.max())
    overshoot = max(0.0, (peak - target_m) / target_m * 100.0)

    def first_crossing(level: float) -> Optional[float]:
        idx = np.argmax(z >= level)
        return float(t[idx]) if z[idx] >= level else None

    t10, t90 = first_crossing(0.1 * target_m), first_crossing(0.9 * target_m)
    rise = (t90 - t10) if (t10 is not None and t90 is not None) else None

    band = band_fraction * target_m
    outside = np.where(np.abs(z - target_m) > band)[0]
    if outside.size == 0:
        settling: Optional[float] = float(t[0])
    elif outside[-1] == z.size - 1:
        settling = None  # still outside the band at the end: never settled
    else:
        settling = float(t[outside[-1] + 1])

    tail = t >= (t[-1] - final_window_s)
    sse = float(np.mean(z[tail] - target_m))

    return StepMetrics(target_m, overshoot, rise, settling, sse, peak)


def state_transitions(df: pd.DataFrame) -> list[tuple[float, str, str]]:
    """List of (time, from_state, to_state) for each safety-state change."""
    out = []
    prev = df["state"].iloc[0]
    for t, s in zip(df["time_s"], df["state"]):
        if s != prev:
            out.append((float(t), prev, s))
            prev = s
    return out


def plot_run(df: pd.DataFrame, title: str, out_path: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True, constrained_layout=True)
    t = df["time_s"]

    # Shade the background by safety state.
    for ax in axes:
        for state, color in STATE_COLORS.items():
            if state == "NORMAL":
                continue
            mask = (df["state"] == state).to_numpy()
            ax.fill_between(t, 0, 1, where=mask, color=color, alpha=0.12,
                            transform=ax.get_xaxis_transform(), step="post", linewidth=0)

    ax = axes[0]
    ax.plot(t, df["measured_depth_m"], ".", ms=2, color="#90a4ae", label="measured (noisy)")
    ax.plot(t, df["true_depth_m"], color="#1565c0", lw=1.8, label="true depth")
    ax.plot(t, df["effective_setpoint_m"], "--", color="#c62828", lw=1.2, label="setpoint (effective)")
    if (df["requested_setpoint_m"] != df["effective_setpoint_m"]).any():
        ax.plot(t, df["requested_setpoint_m"], ":", color="#6a1b9a", lw=1.2, label="operator request")
    ax.invert_yaxis()
    ax.set_ylabel("depth (m)")
    ax.set_title(title)
    ax.legend(loc="best", fontsize=8)
    ax.grid(alpha=0.3)

    axes[1].plot(t, df["thrust_n"], color="#00838f")
    axes[1].set_ylabel("thrust (N)\n(+ = down)")
    axes[1].grid(alpha=0.3)

    axes[2].plot(t, df["battery_frac"] * 100.0, color="#558b2f")
    axes[2].axhline(20, ls="--", color="#f9a825", lw=1)
    axes[2].axhline(10, ls="--", color="#ef6c00", lw=1)
    axes[2].set_ylabel("battery (%)")
    axes[2].set_xlabel("time (s)")
    axes[2].grid(alpha=0.3)

    fig.savefig(out_path, dpi=130)
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser(description="Analyse a rov-depth-sim CSV log")
    ap.add_argument("log")
    ap.add_argument("--target", type=float, default=5.0, help="step target depth in metres")
    ap.add_argument("--plot", help="write a PNG plot to this path")
    args = ap.parse_args()

    df = load_log(args.log)
    m = compute_step_metrics(df["time_s"], df["true_depth_m"], args.target)
    print(f"target depth        : {m.target_m:.2f} m")
    print(f"peak depth          : {m.peak_depth_m:.3f} m")
    print(f"overshoot           : {m.overshoot_pct:.2f} %")
    print(f"rise time (10-90%)  : {m.rise_time_s:.2f} s" if m.rise_time_s is not None else "rise time: n/a")
    print(f"settling time (2%)  : {m.settling_time_s:.2f} s" if m.settling_time_s is not None else "settling time: never")
    print(f"steady-state error  : {m.steady_state_error_m:+.4f} m")
    for t, a, b in state_transitions(df):
        print(f"t={t:6.1f}s  {a} -> {b}")
    if args.plot:
        plot_run(df, args.log, args.plot)
        print(f"plot written to {args.plot}")


if __name__ == "__main__":
    main()
