"""Machine Learning & Random Forest (rf_noisy) based fault detector for Conveyor V2.

Replaces static thresholding with the trained Random Forest pipeline (rf_noisy).
The model receives the sliding 5-sample temporal window of sensor telemetry, applies
the NoiseFilter feature preprocessing, and simultaneously predicts 5 mechanical/electrical faults:
  1. overload          -> OVERLOAD (Belt overload)
  2. excess_vibration  -> VIBRATION (Excessive vibration)
  3. overheating       -> OVERHEAT (Over-temperature)
  4. stall             -> ABN_CURRENT / STALL (Motor stall / Jam / Roller friction)
  5. overspeed         -> OVERSPEED (Belt overspeed)
and the conveyor operating mode (TURNING, HOLDING, DISCHARGED).
"""
import os
import sys
from dataclasses import dataclass
from typing import List, Optional

import joblib
import numpy as np
import pandas as pd

# Ensure dashboard_v2 directory is on sys.path for unpickling features.py
DASH_DIR = os.path.dirname(os.path.abspath(__file__))
if DASH_DIR not in sys.path:
    sys.path.insert(0, DASH_DIR)

from config import Thresholds

WARN, CRIT = 1, 2

FAULT_INFO = {
    "OVERLOAD":     ("Belt overload", "reduce"),
    "OVER_CURRENT": ("Motor over-current", "reduce"),
    "ABN_CURRENT":  ("Motor stall / Jam / Friction", "reduce"),
    "BELT_SLIP":    ("Belt slip / Decoupled", "alert"),
    "VIBRATION":    ("Excessive vibration", "reduce"),
    "OVERHEAT":     ("Over-temperature", "reduce"),
    "OVERSPEED":    ("Belt overspeed", "reduce"),
    "STALL":        ("Motor stall / Jam", "reduce"),
}

SENSORS = ["load_g", "current_ma", "temp_c", "vib_g", "step_delay_us"]
WINDOW = 5


@dataclass
class Fault:
    code: str
    severity: int
    value: float
    limit: float
    message: str

    @property
    def name(self) -> str:
        return FAULT_INFO.get(self.code, (self.code, "alert"))[0]

    @property
    def action(self) -> str:
        return FAULT_INFO.get(self.code, (self.code, "alert"))[1]


class BaseDetector:
    def detect(self, window: list, running_for_s: float) -> List[Fault]:
        raise NotImplementedError


class RFNoisyDetector(BaseDetector):
    """Fault detector driven by the trained Random Forest Noisy Pipeline (rf_noisy)."""

    def __init__(self, model_path: Optional[str] = None):
        if model_path is None:
            # Check local models dir first, then fallback to ML_V2
            candidates = [
                os.path.join(DASH_DIR, "models", "rf_noisy_pipeline.joblib"),
                os.path.join(DASH_DIR, "..", "ML_V2", "models", "noisy", "random_forest_pipeline.joblib"),
                os.path.join(DASH_DIR, "..", "ML_V2", "models", "rf_noisy_pipeline.joblib"),
            ]
            for c in candidates:
                if os.path.exists(c):
                    model_path = os.path.abspath(c)
                    break

        if not model_path or not os.path.exists(model_path):
            raise FileNotFoundError(f"Could not locate rf_noisy_pipeline.joblib model file at {model_path}")

        self.model_path = model_path
        self.model = joblib.load(model_path)
        self.model_name = "Random Forest (rf_noisy)"
        self.last_predictions = {}
        self.last_mode = "TURNING"
        self._streak = {}
        print(f"[RFNoisyDetector] Loaded Random Forest model successfully from: {self.model_path}")

    def detect(self, window: list, running_for_s: float) -> List[Fault]:
        if not window:
            return []

        # Pad window if fewer than 5 samples are available
        w = list(window)
        while len(w) < WINDOW:
            w.insert(0, w[0])
        w5 = w[-WINDOW:]

        # Extract 5-sample temporal sequence for each sensor
        row_dict = {}
        for s in SENSORS:
            for j in range(WINDOW):
                sample = w5[j]
                val = sample.get(s)
                if s == "vib_g" and val is None:
                    val = sample.get("vib_rms_g", 0.05)
                elif val is None:
                    # Realistic baseline fallback
                    defaults = {
                        "load_g": 0.0,
                        "current_ma": 350.0,
                        "temp_c": 32.0,
                        "vib_g": 0.05,
                        "step_delay_us": 1800.0,
                    }
                    val = defaults.get(s, 0.0)
                row_dict[f"{s}_r{j + 1}"] = [float(val)]

        df_input = pd.DataFrame(row_dict)

        # Run inference through the NoiseFilter + Random Forest pipeline
        try:
            preds_df = self.model.predict(df_input)
            pred = preds_df.iloc[0].to_dict()
            self.last_predictions = pred
            self.last_mode = pred.get("mode", "TURNING")
        except Exception as e:
            print(f"[RFNoisyDetector ERROR] Model inference failure: {e}")
            return []

        latest = w5[-1]
        load_val = float(latest.get("load_g") or 0.0)
        cur_val = float(latest.get("current_ma") or 350.0)
        temp_val = float(latest.get("temp_c") or 32.0)
        vib_val = float(latest.get("vib_g", latest.get("vib_rms_g", 0.05)))
        delay_val = float(latest.get("step_delay_us") or 1800.0)

        faults = []

        # 1. Overload
        if pred.get("overload", 0) == 1:
            sev = CRIT if load_val >= 1200 else WARN
            faults.append(Fault(
                code="OVERLOAD",
                severity=sev,
                value=load_val,
                limit=1000.0,
                message=f"AI Model (rf_noisy) detected Belt Overload: {load_val:.0f} g",
            ))

        # 2. Excessive Vibration
        if pred.get("excess_vibration", 0) == 1:
            sev = CRIT if vib_val >= 0.15 else WARN
            faults.append(Fault(
                code="VIBRATION",
                severity=sev,
                value=vib_val,
                limit=0.08,
                message=f"AI Model (rf_noisy) detected Vibration anomaly: {vib_val:.3f} g RMS",
            ))

        # 3. Overheating
        if pred.get("overheating", 0) == 1:
            sev = CRIT if temp_val >= 65.0 else WARN
            faults.append(Fault(
                code="OVERHEAT",
                severity=sev,
                value=temp_val,
                limit=65.0,
                message=f"AI Model (rf_noisy) detected Over-temperature: {temp_val:.1f} °C",
            ))

        # 4. Motor Stall / Jam
        if pred.get("stall", 0) == 1:
            faults.append(Fault(
                code="ABN_CURRENT",
                severity=CRIT,
                value=cur_val,
                limit=1500.0,
                message=f"AI Model (rf_noisy) detected Motor Stall / Jam: Current {cur_val:.0f} mA",
            ))

        # 5. Overspeed
        if pred.get("overspeed", 0) == 1 or (0 < delay_val < 1500.0 and bool(latest.get("run"))):
            faults.append(Fault(
                code="OVERSPEED",
                severity=WARN,
                value=delay_val,
                limit=1500.0,
                message=f"AI Model (rf_noisy) detected Belt Overspeed: Step delay {delay_val:.0f} µs",
            ))

        return sorted(faults, key=lambda f: -f.severity)


class ThresholdDetector(BaseDetector):
    """Legacy threshold-based detector kept for fallback/reference."""

    def __init__(self, th: Thresholds):
        self.th = th
        self._streak = {}

    @staticmethod
    def _mean(rows, key) -> Optional[float]:
        vals = [r[key] for r in rows if r.get(key) is not None]
        return sum(vals) / len(vals) if vals else None

    def _raw_faults(self, window, running_for_s):
        th = self.th
        rows = window[-th.smooth_n:]
        if not rows:
            return []
        load = self._mean(rows, "load_g") or 0.0
        cur = self._mean(rows, "current_ma")
        vib = self._mean(rows, "vib_rms_g")
        temp = self._mean(rows, "temp_c")
        running = bool(rows[-1].get("run"))
        dynamic_ok = running and running_for_s >= th.grace_s
        load_kg = max(load, 0.0) / 1000.0
        out = []

        if load >= th.load_crit_g:
            out.append(Fault("OVERLOAD", CRIT, load, th.load_crit_g, f"Load {load:.0f} g exceeds {th.load_crit_g:.0f} g"))
        elif load >= th.load_warn_g:
            out.append(Fault("OVERLOAD", WARN, load, th.load_warn_g, f"Load {load:.0f} g exceeds {th.load_warn_g:.0f} g"))

        if temp is not None:
            if temp >= th.temp_crit_c:
                out.append(Fault("OVERHEAT", CRIT, temp, th.temp_crit_c, f"Temperature {temp:.1f} C above {th.temp_crit_c:.0f} C"))
            elif temp >= th.temp_warn_c:
                out.append(Fault("OVERHEAT", WARN, temp, th.temp_warn_c, f"Temperature {temp:.1f} C above {th.temp_warn_c:.0f} C"))

        if dynamic_ok and cur is not None:
            expected = th.current_idle_ma + th.current_per_kg_ma * load_kg
            tol = max(th.current_tol_floor_ma, expected * th.current_tol_ratio)
            if cur >= th.current_crit_ma:
                out.append(Fault("OVER_CURRENT", CRIT, cur, th.current_crit_ma, f"Current {cur:.0f} mA above hard limit {th.current_crit_ma:.0f} mA"))
            elif cur > expected + tol:
                out.append(Fault("ABN_CURRENT", WARN, cur, expected + tol, f"Current {cur:.0f} mA but only {expected + tol:.0f} mA expected at {load:.0f} g"))
            elif cur < expected - tol and load >= th.slip_min_load_g:
                out.append(Fault("BELT_SLIP", WARN, cur, expected - tol, f"Current {cur:.0f} mA is below {expected - tol:.0f} mA expected at {load:.0f} g"))

        if dynamic_ok and vib is not None:
            limit = th.vib_base_g + th.vib_per_kg_g * load_kg
            if vib >= limit * th.vib_crit_factor:
                out.append(Fault("VIBRATION", CRIT, vib, limit * th.vib_crit_factor, f"Vibration {vib:.3f} g (limit {limit:.3f} g at this load)"))
            elif vib >= limit:
                out.append(Fault("VIBRATION", WARN, vib, limit, f"Vibration {vib:.3f} g (limit {limit:.3f} g at this load)"))
        return out

    def detect(self, window, running_for_s):
        raw = {f.code: f for f in self._raw_faults(window, running_for_s)}
        for code in list(self._streak):
            if code not in raw:
                del self._streak[code]
        confirmed = []
        for code, f in raw.items():
            self._streak[code] = self._streak.get(code, 0) + 1
            if self._streak[code] >= self.th.persist_n:
                confirmed.append(f)
        return sorted(confirmed, key=lambda f: -f.severity)
