"""Data generator for ML_V2: generates clean Parquet, noisy Parquet, and mixed 10K real-machine Parquet."""
import argparse
import json
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from plot_style import apply_style, C_MAIN, C_ALT, C_FAULT, C_MUTED

ROOT = Path(__file__).resolve().parent
DATA_DIR = ROOT / "data"
FIG_DIR = ROOT / "results" / "figures" / "data_analysis"

SENSORS = ["load_g", "current_ma", "temp_c", "vib_g", "step_delay_us"]
SENSOR_LABEL = {
    "load_g": "Load (g)",
    "current_ma": "Current (mA)",
    "temp_c": "Temperature (°C)",
    "vib_g": "Vibration (g RMS)",
    "step_delay_us": "Step delay (µs)",
}
WINDOW = 5
FAULTS = ["overload", "overspeed", "excess_vibration", "overheating", "stall"]
MODES = ["TURNING", "HOLDING", "DISCHARGED"]

# ---- Operating limits (fault definitions) ---------------------------------------------
LIMITS = {
    "load_max_g": 1000.0,          # above this while motor is energised = overload
    "temp_max_c": 65.0,            # above this = overheating
    "delay_fastest_us": 1500.0,    # step delay below this (faster) = overspeed
    "delay_slowest_us": 2000.0,    # normal speed window is 1500..2000 us
    "vib_base_g": 0.06,            # load-aware vibration limit = base + per_kg * load_kg
    "vib_per_kg_g": 0.02,
    "stall_current_factor": 1.5,   # stall: commanded to turn, no motion, current >= 1.5x expected
}

# ---- Physics parameters -----------------------------------------------------------------
CUR_TURN_BASE, CUR_TURN_PER_KG, CUR_PER_SPEED = 350.0, 150.0, 0.05
CUR_HOLD, CUR_HOLD_PER_KG = 300.0, 40.0

P_INJECT = {
    "overload": np.array([0.15, 0.15, 0.10]),
    "overspeed": np.array([0.12, 0.00, 0.00]),
    "excess_vibration": np.array([0.12, 0.00, 0.00]),
    "overheating": np.array([0.10, 0.08, 0.05]),
    "stall": np.array([0.07, 0.00, 0.00]),
}
P_MODE = [0.70, 0.15, 0.15]

# ---- Measurement noise model ----------------------------------------------------------
NOISE = {
    "sigma": {"load_g": 15.0, "current_ma": 10.0, "temp_c": 0.4, "vib_g": 0.004, "step_delay_us": 12.0},
    "p_spike": 0.02,
    "p_dropout": 0.01,
    "p_error_code": 0.004,
}


def vib_limit(load_g):
    return LIMITS["vib_base_g"] + LIMITS["vib_per_kg_g"] * np.clip(load_g, 0, None) / 1000.0


def simulate_truth(n, rng):
    """Simulate noise-free physical truth and ground-truth labels."""
    mode = rng.choice(3, size=n, p=P_MODE)
    turning, holding, off = mode == 0, mode == 1, mode == 2
    inj = {f: rng.random(n) < P_INJECT[f][mode] for f in FAULTS}
    inj["overspeed"] &= ~inj["stall"]

    load = np.where(inj["overload"], rng.uniform(1000, 2000, n), rng.uniform(0, 1000, n))
    kg = load / 1000.0

    delay = np.zeros(n)
    delay[turning] = rng.uniform(LIMITS["delay_fastest_us"], LIMITS["delay_slowest_us"], turning.sum())
    os_ = turning & inj["overspeed"]
    delay[os_] = rng.uniform(1100, LIMITS["delay_fastest_us"], os_.sum())
    stall = turning & inj["stall"]
    delay[stall] = 0.0
    faster = np.where(delay > 0, LIMITS["delay_slowest_us"] - delay, 0.0)

    expected_turn = CUR_TURN_BASE + CUR_TURN_PER_KG * kg
    current = np.where(off, rng.uniform(0, 8, n), 0.0)
    current = np.where(holding, rng.normal(CUR_HOLD, 15, n) + CUR_HOLD_PER_KG * kg, current)
    current = np.where(turning, expected_turn + CUR_PER_SPEED * faster, current)
    current = np.where(stall, expected_turn * rng.uniform(1.5, 2.2, n), current)

    vib = np.where(off, rng.uniform(0.001, 0.004, n), 0.0)
    vib = np.where(holding, rng.uniform(0.003, 0.010, n), vib)
    base_run = 0.02 + 0.012 * kg + 0.00004 * faster + np.abs(rng.normal(0, 0.004, n))
    base_run += np.where(os_, 0.0001 * (LIMITS["delay_fastest_us"] - delay), 0.0)
    vib = np.where(turning, base_run, vib)
    vib = np.where(turning & inj["excess_vibration"], vib + rng.uniform(0.02, 0.30, n), vib)
    vib = np.where(stall, vib + rng.uniform(0.03, 0.12, n), vib)

    temp = rng.uniform(22, 32, n)
    temp += np.where(turning, 0.02 * current, 0.0) + np.where(holding, 0.015 * current, 0.0)
    temp += np.where(off, rng.uniform(0, 8, n), 0.0)
    temp = np.where(inj["overheating"], np.maximum(temp, rng.uniform(60, 90, n)), temp)

    truth = pd.DataFrame({
        "load_g": load,
        "current_ma": current,
        "temp_c": temp,
        "vib_g": vib,
        "step_delay_us": delay,
    })
    labels = pd.DataFrame({
        "overload": ((load > LIMITS["load_max_g"]) & ~off).astype(int),
        "overspeed": ((delay > 0) & (delay < LIMITS["delay_fastest_us"])).astype(int),
        "excess_vibration": (vib > vib_limit(load)).astype(int),
        "overheating": (temp > LIMITS["temp_max_c"]).astype(int),
        "stall": (turning & (delay == 0) & (current >= LIMITS["stall_current_factor"] * expected_turn)).astype(int),
    })
    labels["mode"] = np.array(MODES)[mode]
    return truth, labels


def truth_to_clean_windows(truth):
    """Create 5-reading clean window where each reading equals the pure digital value."""
    n = len(truth)
    out = {}
    for s in SENSORS:
        t = truth[s].to_numpy()[:, None].repeat(WINDOW, axis=1)
        for j in range(WINDOW):
            out[f"{s}_r{j + 1}"] = t[:, j]
    return pd.DataFrame(out)


def add_noise(truth, rng, level=1.0):
    """Inject realistic measurement noise, spikes, dropouts, and hardware error codes."""
    n = len(truth)
    out, stats = {}, {}
    p_spike = min(NOISE["p_spike"] * level, 0.3)
    p_drop = min(NOISE["p_dropout"] * level, 0.3)
    p_err = min(NOISE["p_error_code"] * level, 0.1)

    for s in SENSORS:
        t = truth[s].to_numpy()[:, None].repeat(WINDOW, axis=1)
        moving = t > 0
        if s == "load_g":
            t = t + rng.normal(0, 5, t.shape)
        elif s == "current_ma":
            t = t * (1 + rng.normal(0, 0.02, t.shape))
        elif s == "vib_g":
            t = t * np.clip(rng.normal(1, 0.15, t.shape), 0.3, None)
        elif s == "step_delay_us":
            t = np.where(moving, t + rng.normal(0, 5, t.shape), 0.0)

        x = t + rng.normal(0, NOISE["sigma"][s] * level, t.shape)
        if s == "step_delay_us":
            x = np.where(moving, x, 0.0)

        spike = rng.random(t.shape) < p_spike
        if s == "load_g":
            x = np.where(spike, x * rng.uniform(2, 4, t.shape) + rng.uniform(200, 1500, t.shape), x)
        elif s == "current_ma":
            x = np.where(spike, x * rng.uniform(2, 3.5, t.shape) + rng.uniform(50, 300, t.shape), x)
        elif s == "temp_c":
            x = np.where(spike, x + rng.uniform(5, 25, t.shape), x)
        elif s == "vib_g":
            x = np.where(spike, x + rng.uniform(0.2, 1.0, t.shape), x)
        else:
            x = np.where(spike, np.where(moving, x * rng.uniform(0.3, 0.7, t.shape), rng.uniform(300, 2500, t.shape)), x)

        err = np.zeros(t.shape, bool)
        if s == "temp_c":
            err = rng.random(t.shape) < p_err
            x = np.where(err, -127.0, x)  # DS18B20 disconnect error code
        elif s == "load_g":
            err = rng.random(t.shape) < p_err / 2
            x = np.where(err, 99999.0, x)  # HX711 saturation error code

        if s in ("vib_g", "step_delay_us"):
            x = np.clip(x, 0, None)

        drop = rng.random(t.shape) < p_drop
        x = np.where(drop, np.nan, x)

        for j in range(WINDOW):
            out[f"{s}_r{j + 1}"] = x[:, j]

        clean = ~(spike | err | drop)
        resid = (x - t)[clean]
        sig_var = np.nanvar(truth[s].to_numpy())
        stats[s] = {
            "spikes": int(spike.sum()),
            "error_codes": int(err.sum()),
            "dropouts": int(drop.sum()),
            "noise_std": float(np.nanstd(resid)) if resid.size else 0.0,
            "snr_db": float(10 * np.log10(sig_var / max(np.nanvar(resid), 1e-12))) if resid.size else float("inf"),
        }
    return pd.DataFrame(out), stats


def generate_all_datasets(seed=42, n_train=100_000, n_real=10_000):
    """
    1. Generate 100K digital dataset without noise -> 100K_without_noise.parquet
    2. Read 100K digital dataset and inject noise -> 100K_with_noise.parquet
    3. Generate 10K real-machine dataset with mixed clean and noisy windows in 1 file -> 10K_realmachine.parquet
    """
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    apply_style()

    print(f"[{time.strftime('%H:%M:%S')}] Step 1: Generating 100K digital dataset without noise...")
    rng_100k = np.random.default_rng(seed)
    truth_100k, labels_100k = simulate_truth(n_train, rng_100k)
    clean_readings_100k = truth_to_clean_windows(truth_100k)
    df_without_noise = pd.concat([clean_readings_100k, labels_100k], axis=1)

    path_without_noise = DATA_DIR / "100K_without_noise.parquet"
    df_without_noise.to_parquet(path_without_noise, index=False, engine="pyarrow")
    print(f"[{time.strftime('%H:%M:%S')}] Saved: {path_without_noise} ({path_without_noise.stat().st_size / 1e6:.2f} MB)")

    print(f"[{time.strftime('%H:%M:%S')}] Step 2: Reading 100K_without_noise.parquet and adding noise...")
    df_read_back = pd.read_parquet(path_without_noise)
    # Extract truth from r1 columns (since clean r1..r5 are identical pure values)
    truth_extracted = pd.DataFrame({s: df_read_back[f"{s}_r1"].to_numpy() for s in SENSORS})
    rng_noise = np.random.default_rng(seed + 100)
    noisy_readings_100k, stats_100k = add_noise(truth_extracted, rng_noise, level=1.0)
    df_with_noise = pd.concat([noisy_readings_100k, df_read_back[FAULTS + ["mode"]]], axis=1)

    path_with_noise = DATA_DIR / "100K_with_noise.parquet"
    df_with_noise.to_parquet(path_with_noise, index=False, engine="pyarrow")
    print(f"[{time.strftime('%H:%M:%S')}] Saved: {path_with_noise} ({path_with_noise.stat().st_size / 1e6:.2f} MB)")

    print(f"[{time.strftime('%H:%M:%S')}] Step 3: Generating 10K real-machine dataset (mixed clean + noisy in single file)...")
    rng_real = np.random.default_rng(seed + 999)
    truth_real, labels_real = simulate_truth(n_real, rng_real)

    # 5,000 clean windows + 5,000 noisy windows interleaved/partitioned
    is_noisy_arr = np.zeros(n_real, dtype=int)
    is_noisy_arr[n_real // 2:] = 1  # first 5K clean, second 5K noisy
    rng_real.shuffle(is_noisy_arr)

    clean_readings_real = truth_to_clean_windows(truth_real)
    noisy_readings_real, stats_real = add_noise(truth_real, rng_real, level=1.2)  # slightly harsher real plant noise

    real_readings = pd.DataFrame(index=range(n_real))
    for col in clean_readings_real.columns:
        real_readings[col] = np.where(is_noisy_arr == 1, noisy_readings_real[col], clean_readings_real[col])

    df_realmachine = pd.concat([real_readings, labels_real], axis=1)
    df_realmachine["is_noisy"] = is_noisy_arr

    path_realmachine = DATA_DIR / "10K_realmachine.parquet"
    df_realmachine.to_parquet(path_realmachine, index=False, engine="pyarrow")
    print(f"[{time.strftime('%H:%M:%S')}] Saved: {path_realmachine} ({path_realmachine.stat().st_size / 1e6:.2f} MB)")

    # Metadata saving
    meta = {
        "n_train": n_train,
        "n_real": n_real,
        "seed": seed,
        "window": WINDOW,
        "sensors": SENSORS,
        "faults": FAULTS,
        "modes": MODES,
        "limits": LIMITS,
        "noise_model": NOISE,
        "noise_stats_100k": stats_100k,
        "noise_stats_10k_real": stats_real,
        "realmachine_composition": {
            "total_rows": n_real,
            "clean_rows": int((is_noisy_arr == 0).sum()),
            "noisy_rows": int((is_noisy_arr == 1).sum()),
        },
    }
    (DATA_DIR / "dataset_meta.json").write_text(json.dumps(meta, indent=2))
    print(f"[{time.strftime('%H:%M:%S')}] Saved dataset metadata.")

    # Generate analytical visualizations
    print(f"[{time.strftime('%H:%M:%S')}] Generating dataset figures in {FIG_DIR}...")
    generate_dataset_plots(df_without_noise, df_with_noise, df_realmachine, FIG_DIR)

    return df_without_noise, df_with_noise, df_realmachine


def generate_dataset_plots(df_clean, df_noisy, df_real, out_dir):
    n = len(df_clean)

    # 1. Class Balance (Fault prevalence & modes)
    fig, axs = plt.subplots(1, 2, figsize=(11, 4), gridspec_kw={"width_ratios": [5, 3]})
    share = df_clean[FAULTS].mean() * 100
    axs[0].barh(FAULTS[::-1], share[FAULTS[::-1]], color=C_FAULT)
    for i, f in enumerate(FAULTS[::-1]):
        axs[0].text(share[f] + 0.3, i, f"{share[f]:.1f}% ({int(df_clean[f].sum()):,})", va="center", fontsize=9)
    axs[0].set_xlabel("Prevalence (%)")
    axs[0].set_title("Fault Distribution (100K dataset)")
    axs[0].set_xlim(0, share.max() * 1.45)

    ms = df_clean["mode"].value_counts().reindex(MODES)
    axs[1].bar(MODES, ms / n * 100, color=[C_MAIN, C_ALT, C_MUTED])
    for i, m in enumerate(MODES):
        axs[1].text(i, ms[m] / n * 100 + 1, f"{ms[m] / n:.1%}", ha="center", fontsize=9)
    axs[1].set_ylabel("Rows (%)")
    axs[1].set_title("Operating Mode Distribution")
    fig.tight_layout()
    fig.savefig(out_dir / "class_balance.png")
    plt.close(fig)

    # 2. Fault Co-occurrence & Simultaneous Faults
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.2), gridspec_kw={"width_ratios": [4, 3]})
    Y = df_clean[FAULTS].to_numpy()
    co = Y.T @ Y
    im = axs[0].imshow(co, cmap="Blues")
    axs[0].set_xticks(range(5), FAULTS, rotation=30, ha="right")
    axs[0].set_yticks(range(5), FAULTS)
    for i in range(5):
        for j in range(5):
            axs[0].text(j, i, f"{co[i, j]:,}", ha="center", va="center", fontsize=8,
                        color="white" if co[i, j] > co.max() * 0.5 else "black")
    axs[0].set_title("Fault Co-occurrence Matrix")
    axs[0].grid(False)

    per_row = Y.sum(1)
    vals, cnts = np.unique(per_row, return_counts=True)
    axs[1].bar(vals, cnts / n * 100, color=C_MAIN)
    for v, c in zip(vals, cnts):
        axs[1].text(v, c / n * 100 + 1, f"{c:,}\n({c / n:.1%})", ha="center", fontsize=8.5)
    axs[1].set_xlabel("Faults per Sample")
    axs[1].set_ylabel("Share (%)")
    axs[1].set_title("Simultaneous Faults Distribution")
    fig.tight_layout()
    fig.savefig(out_dir / "fault_cooccurrence.png")
    plt.close(fig)

    # 3. Clean vs Noisy Scatter Comparison
    idx = np.random.default_rng(42).choice(n, size=min(3000, n), replace=False)
    fig, axs = plt.subplots(1, 5, figsize=(18, 3.8))
    for ax, s in zip(axs, SENSORS):
        clean_vals = df_clean[f"{s}_r1"].to_numpy()[idx]
        noisy_vals = df_noisy[f"{s}_r5"].to_numpy()[idx]
        lo, hi = np.nanpercentile(clean_vals, [0.5, 99.5])
        pad = (hi - lo) * 0.08 + 1e-6
        ax.scatter(clean_vals, noisy_vals, s=4, color=C_FAULT, alpha=0.35, label="Raw Noisy Reading")
        ax.plot([lo, hi], [lo, hi], color="black", lw=1.2, ls="--", label="Ideal Clean Diagonal")
        ax.set_xlim(lo - pad, hi + pad)
        ax.set_ylim(lo - pad, hi + pad * 3.5)
        ax.set_title(SENSOR_LABEL[s], fontsize=10)
        ax.set_xlabel("Clean Digital Value")
    axs[0].set_ylabel("Measured Noisy Value")
    axs[0].legend(fontsize=8, markerscale=3)
    fig.suptitle("Clean Digital Values vs Noisy Telemetry (Spikes, Jitter, and Errors)", y=1.02)
    fig.tight_layout()
    fig.savefig(out_dir / "clean_vs_noisy_comparison.png", bbox_inches="tight")
    plt.close(fig)

    # 4. Real-machine 10K dataset composition
    fig, ax = plt.subplots(figsize=(7, 4))
    clean_cnt = (df_real["is_noisy"] == 0).sum()
    noisy_cnt = (df_real["is_noisy"] == 1).sum()
    ax.bar(["Clean Telemetry\n(no noise)", "Noisy Telemetry\n(harsh plant conditions)"],
           [clean_cnt, noisy_cnt], color=[C_MAIN, C_FAULT], width=0.5)
    ax.text(0, clean_cnt / 2, f"{clean_cnt:,} samples\n({clean_cnt / len(df_real):.0%})", ha="center", va="center", color="white", fontweight="bold")
    ax.text(1, noisy_cnt / 2, f"{noisy_cnt:,} samples\n({noisy_cnt / len(df_real):.0%})", ha="center", va="center", color="white", fontweight="bold")
    ax.set_ylabel("Number of Samples")
    ax.set_title("10K Real-Machine Dataset Composition (Single Parquet File)")
    fig.tight_layout()
    fig.savefig(out_dir / "realmachine_10k_composition.png")
    plt.close(fig)

    print(f"[{time.strftime('%H:%M:%S')}] Saved all dataset figures to {out_dir}.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate ML_V2 datasets")
    parser.add_argument("--n-train", type=int, default=100_000)
    parser.add_argument("--n-real", type=int, default=10_000)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    generate_all_datasets(seed=args.seed, n_train=args.n_train, n_real=args.n_real)
