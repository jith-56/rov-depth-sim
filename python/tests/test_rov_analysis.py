import os
import sys

import numpy as np
import pandas as pd
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import rov_analysis as ra  # noqa: E402


def first_order(tau=1.0, target=5.0, T=30.0, dt=0.1):
    t = np.arange(0, T + dt, dt)
    return t, target * (1 - np.exp(-t / tau))


def second_order(zeta, wn=2.0, target=5.0, T=30.0, dt=0.01):
    t = np.arange(0, T + dt, dt)
    wd = wn * np.sqrt(1 - zeta**2)
    phi = np.arccos(zeta)
    y = 1 - np.exp(-zeta * wn * t) / np.sqrt(1 - zeta**2) * np.sin(wd * t + phi)
    return t, target * y


def test_overshoot_matches_theory_for_second_order_system():
    zeta = 0.5
    t, z = second_order(zeta)
    m = ra.compute_step_metrics(t, z, 5.0)
    theory = 100 * np.exp(-np.pi * zeta / np.sqrt(1 - zeta**2))  # ~16.3 %
    assert m.overshoot_pct == pytest.approx(theory, abs=0.3)


def test_overdamped_response_has_zero_overshoot():
    t, z = first_order()
    assert ra.compute_step_metrics(t, z, 5.0).overshoot_pct == 0.0


def test_rise_time_of_first_order_system():
    tau = 1.0
    t, z = first_order(tau=tau)
    m = ra.compute_step_metrics(t, z, 5.0)
    # 10% -> 90% rise time of a first-order system is tau * ln(9) ~ 2.197 tau
    assert m.rise_time_s == pytest.approx(tau * np.log(9), abs=0.15)


def test_settling_time_of_first_order_system():
    tau = 1.0
    t, z = first_order(tau=tau)
    m = ra.compute_step_metrics(t, z, 5.0, band_fraction=0.02)
    assert m.settling_time_s == pytest.approx(tau * np.log(50), abs=0.15)  # ~3.9 tau


def test_steady_state_error_detects_constant_offset():
    t = np.arange(0, 40, 0.1)
    z = np.full_like(t, 4.8)
    m = ra.compute_step_metrics(t, z, 5.0)
    assert m.steady_state_error_m == pytest.approx(-0.2)


def test_never_settling_returns_none():
    t = np.arange(0, 20, 0.1)
    z = 5.0 + 1.0 * np.sin(t)  # oscillates +-20% forever
    assert ra.compute_step_metrics(t, z, 5.0).settling_time_s is None


def test_response_that_never_leaves_band_settles_at_start():
    t = np.arange(0, 10, 0.1)
    z = np.full_like(t, 5.0)
    assert ra.compute_step_metrics(t, z, 5.0).settling_time_s == 0.0


def test_vehicle_that_never_reaches_target_has_no_rise_time():
    t = np.arange(0, 10, 0.1)
    z = np.full_like(t, 1.0)
    assert ra.compute_step_metrics(t, z, 5.0).rise_time_s is None


@pytest.mark.parametrize("bad", [(np.array([0.0]), np.array([0.0])), (np.arange(3.0), np.arange(4.0))])
def test_bad_shapes_are_rejected(bad):
    with pytest.raises(ValueError):
        ra.compute_step_metrics(bad[0], bad[1], 5.0)


def test_non_positive_target_is_rejected():
    with pytest.raises(ValueError):
        ra.compute_step_metrics([0, 1], [0, 1], 0.0)


def test_state_transitions_are_listed_in_order():
    df = pd.DataFrame(
        {"time_s": [0, 1, 2, 3, 4], "state": ["NORMAL", "NORMAL", "WARNING", "WARNING", "SURFACING"]}
    )
    assert ra.state_transitions(df) == [(2.0, "NORMAL", "WARNING"), (4.0, "WARNING", "SURFACING")]


def test_load_log_rejects_missing_columns(tmp_path):
    p = tmp_path / "bad.csv"
    p.write_text("time_s,true_depth_m\n0,0\n")
    with pytest.raises(ValueError):
        ra.load_log(str(p))


def test_plot_is_written(tmp_path):
    n = 50
    df = pd.DataFrame(
        {
            "time_s": np.arange(n) * 0.1,
            "true_depth_m": np.linspace(0, 5, n),
            "measured_depth_m": np.linspace(0, 5, n),
            "requested_setpoint_m": 5.0,
            "effective_setpoint_m": 5.0,
            "thrust_n": 1.0,
            "battery_frac": np.linspace(1, 0.9, n),
            "state": ["NORMAL"] * n,
        }
    )
    out = tmp_path / "p.png"
    ra.plot_run(df, "t", str(out))
    assert out.exists() and out.stat().st_size > 1000
