"""Feature engineering and noise filtering for ML_V2."""
import warnings
import numpy as np
import pandas as pd
from sklearn.base import BaseEstimator, TransformerMixin

from generator import FAULTS, MODES, SENSORS, WINDOW, vib_limit

VALID_RANGE = {
    "load_g": (-100.0, 5000.0),
    "current_ma": (-20.0, 3000.0),
    "temp_c": (-20.0, 125.0),
    "vib_g": (0.0, 4.0),
    "step_delay_us": (0.0, 100000.0),
}


def raw_block(df, sensor):
    cols = [f"{sensor}_r{j + 1}" for j in range(WINDOW)]
    return df[cols].to_numpy(dtype=float)


def validate(block, sensor):
    lo, hi = VALID_RANGE[sensor]
    out = block.copy()
    out[(out < lo) | (out > hi)] = np.nan
    return out


def window_median(df):
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        return pd.DataFrame(
            {s: np.nanmedian(validate(raw_block(df, s), s), axis=1) for s in SENSORS},
            index=df.index,
        )


class NoiseFilter(BaseEstimator, TransformerMixin):
    """Filters noisy readings and extracts window median, spread, and load-aware ratio."""

    def __init__(self, variant="filtered"):
        self.variant = variant

    def _features(self, df):
        if self.variant == "raw":
            f = pd.DataFrame(
                {s: df[f"{s}_r{WINDOW}"].to_numpy(dtype=float) for s in SENSORS},
                index=df.index,
            )
        else:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", RuntimeWarning)
                cols = {}
                for s in SENSORS:
                    b = validate(raw_block(df, s), s)
                    cols[s] = np.nanmedian(b, axis=1)
                    cols[f"{s}_spread"] = np.nanstd(b, axis=1)
                f = pd.DataFrame(cols, index=df.index)

        # Domain engineered feature: vibration relative to load limit
        f["vib_load_ratio"] = f["vib_g"] / vib_limit(f["load_g"].fillna(0).clip(lower=0, upper=5000))
        return f

    def fit(self, df, y=None):
        f = self._features(df)
        self.feature_names_ = list(f.columns)
        self.fill_ = f.median()
        return self

    def transform(self, df):
        f = self._features(df)
        if self.variant == "filtered":
            f = f.fillna(self.fill_)
        return f[self.feature_names_]

    def report(self, df):
        out = {}
        for s in SENSORS:
            b = raw_block(df, s)
            v = validate(b, s)
            out[s] = {
                "missing": int(np.isnan(b).sum()),
                "out_of_range": int((np.isnan(v) & ~np.isnan(b)).sum()),
                "empty_windows": int(np.isnan(v).all(axis=1).sum()),
            }
        return out


class MultiTask(BaseEstimator):
    """Encapsulates 6 fitted estimators (5 faults + mode)."""

    def __init__(self, models=None):
        self.models = models

    def fit(self, X, y=None):
        return self

    def __sklearn_is_fitted__(self):
        return True

    def predict(self, X):
        out = {f: self.models[f].predict(X) for f in FAULTS}
        out["mode"] = np.array(MODES)[self.models["mode"].predict(X)]
        return pd.DataFrame(out, index=getattr(X, "index", None))

    def predict_proba(self, X):
        return {t: self.models[t].predict_proba(X) for t in list(FAULTS) + ["mode"]}
