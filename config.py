"""All tunable settings in one place. Everything here can also be edited live in the dashboard sidebar."""
import os
from dataclasses import dataclass

MQTT_HOST = os.getenv("MQTT_HOST", "localhost")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))

TOPIC_TELEMETRY = "conveyor/telemetry"
TOPIC_EVENT = "conveyor/event"
TOPIC_STATUS = "conveyor/status"
TOPIC_COMMAND = "conveyor/command"

LOG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data")
HISTORY_LEN = 3000          # samples kept in memory (5 Hz -> 10 minutes)
STALE_AFTER_S = 3.0         # no telemetry for this long = data link lost


@dataclass
class Thresholds:
    # --- load (rated 1 kg capacity) ---
    load_warn_g: float = 1000.0
    load_crit_g: float = 1200.0
    # --- load-aware current model: expected_mA = idle + per_kg * load_kg ---
    current_idle_ma: float = 300.0
    current_per_kg_ma: float = 60.0
    current_tol_ratio: float = 0.35      # +/- 35 % around the expected current is "normal"
    current_tol_floor_ma: float = 80.0   # ...but never tighter than this
    current_crit_ma: float = 1800.0
    slip_min_load_g: float = 500.0       # underload/slip check only when this much load is on the belt
    # --- load-aware vibration limit: limit_g = base + per_kg * load_kg ---
    vib_base_g: float = 0.06
    vib_per_kg_g: float = 0.01
    vib_crit_factor: float = 1.6
    # --- temperature ---
    temp_warn_c: float = 50.0
    temp_crit_c: float = 65.0
    # --- filtering ---
    smooth_n: int = 5            # average the last N samples
    persist_n: int = 4           # fault must be seen in N consecutive evaluations
    grace_s: float = 4.0         # ignore dynamic faults this long after belt start


@dataclass
class ControllerConfig:
    target_speed_pct: float = 60.0   # operator set-point
    min_speed_pct: float = 15.0      # adaptive system never goes below this
    step_pct: float = 10.0           # speed change per adaptation step (x2 for critical)
    settle_s: float = 4.0            # wait this long after an action before judging again
    recover_after_s: float = 8.0     # fault-free time before speed is stepped back up
    recover_step_pct: float = 5.0
    escalate_after_s: float = 25.0   # fault still there after this long -> big alarm
    trip_after_s: float = 10.0       # critical fault at min speed for this long -> stop the belt
    load_aware_speed: bool = True    # derate set-point automatically when belt is heavily loaded
    load_derate: float = 0.30        # at load_warn the set-point is cut by 30 %
    auto_adapt: bool = True          # master switch for self-adaptive actions
