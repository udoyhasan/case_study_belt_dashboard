"""Self-adaptive controller.

speed_cmd = clamp(desired_speed - fault_offset)
  desired_speed : operator set-point, optionally derated by the current load (load-aware)
  fault_offset  : grows by `step_pct` every `settle_s` while an actionable fault persists,
                  shrinks again after `recover_after_s` of clean running.
If the fault is still present after `escalate_after_s` -> ALARM (big banner + sound).
A critical fault that survives `trip_after_s` at minimum speed -> belt is stopped (tripped).
"""
from dataclasses import dataclass
from typing import Optional

from config import ControllerConfig, Thresholds
from detector import CRIT

NORMAL, ADAPTING, ALARM, TRIPPED, ESTOP = "NORMAL", "ADAPTING", "ALARM", "TRIPPED", "E-STOP"


@dataclass
class Decision:
    run: bool
    speed_pct: float


class AdaptiveController:
    def __init__(self, cfg: ControllerConfig, th: Thresholds):
        self.cfg, self.th = cfg, th
        self.operator_run = False
        self.fault_offset = 0.0
        self.mode = NORMAL
        self.last_action_t = 0.0
        self.fault_since: Optional[float] = None
        self.clear_since: Optional[float] = None
        self.minspeed_crit_since: Optional[float] = None
        self.last_recover_t = 0.0
        self.alarm_active = False
        self.alarm_silenced = False
        self.alarm_reason = ""
        self.actions = 0
        self.safe_ceiling: Optional[float] = None   # learned: speed above which the fault came back
        self.ceiling_until = 0.0
        self.ceiling_hold_s = 120.0
        self._now = 0.0
        self.log = []          # callable set by backend: log(kind, text)

    # ---- helpers ----
    def _log(self, kind, text):
        if callable(self.log):
            self.log(kind, text)

    def desired_speed(self, load_g: float) -> float:
        c = self.cfg
        s = c.target_speed_pct
        if c.load_aware_speed and self.th.load_warn_g > 0:
            frac = min(max(load_g, 0.0) / self.th.load_warn_g, 1.0)
            s = s * (1.0 - c.load_derate * frac)
        return s

    def command_speed(self, load_g: float) -> float:
        c = self.cfg
        s = min(100.0, self.desired_speed(load_g) - self.fault_offset)
        if self.safe_ceiling is not None and self._now < self.ceiling_until:
            s = min(s, self.safe_ceiling)
        return max(c.min_speed_pct, s)

    def raise_alarm(self, reason: str):
        if not self.alarm_active:
            self.alarm_active, self.alarm_silenced, self.alarm_reason = True, False, reason
            self._log("alarm", reason)

    def acknowledge(self):
        self.alarm_silenced = True
        self._log("info", "Alarm acknowledged by operator")

    def operator_start(self):
        self.operator_run = True
        self.fault_offset = 0.0
        self.safe_ceiling = None
        self.fault_since = self.clear_since = self.minspeed_crit_since = None
        if self.mode in (TRIPPED, ALARM, ESTOP):
            self.mode = NORMAL
        self.alarm_active = False
        self._log("info", "Belt START requested")

    def operator_stop(self):
        self.operator_run = False
        self._log("info", "Belt STOP requested")

    def device_estop(self):
        self.operator_run = False
        self.mode = ESTOP
        self.raise_alarm("Physical EMERGENCY STOP pressed on the machine")

    # ---- main step ----
    def update(self, faults, now: float, load_g: float, device_estop: bool) -> Decision:
        c = self.cfg
        self._now = now
        if device_estop:
            self.mode = ESTOP
            return Decision(False, self.command_speed(load_g))
        if self.mode == ESTOP:
            self.mode = NORMAL
            self.alarm_active = False

        if not self.operator_run:
            # belt is off: nothing to adapt
            self.fault_since = self.minspeed_crit_since = None
            if self.mode != TRIPPED:
                self.mode = NORMAL
            return Decision(False, self.command_speed(load_g))

        actionable = [f for f in faults if f.action == "reduce"]
        any_fault = bool(faults)

        if any_fault:
            self.clear_since = None
            if self.fault_since is None:
                self.fault_since = now
            if self.mode == NORMAL:
                self.mode = ADAPTING
        else:
            if self.clear_since is None:
                self.clear_since = now
            self.fault_since = None
            self.minspeed_crit_since = None

        if c.auto_adapt and actionable and now - self.last_action_t >= c.settle_s:
            worst = max(f.severity for f in actionable)
            at_min = self.command_speed(load_g) <= c.min_speed_pct + 1e-6
            if not at_min:
                step = c.step_pct * (2 if worst >= CRIT else 1)
                before = self.command_speed(load_g)
                self.fault_offset += step
                after = self.command_speed(load_g)
                self.actions += 1
                top = actionable[0]
                self._log("action", f"{top.name}: speed {before:.0f}% -> {after:.0f}%")
                if now - self.last_recover_t < 15.0:      # fault came straight back after recovering
                    self.safe_ceiling = after
                    self.ceiling_until = now + self.ceiling_hold_s
                    self._log("info", f"Learned safe speed ceiling {after:.0f}% (re-test in {self.ceiling_hold_s:.0f}s)")
                self.last_action_t = now

        # trip: critical fault that survives at minimum speed
        crit_now = any(f.severity >= CRIT and f.action == "reduce" for f in faults)
        at_min = self.command_speed(load_g) <= c.min_speed_pct + 1e-6
        if crit_now and at_min:
            self.minspeed_crit_since = self.minspeed_crit_since or now
            if now - self.minspeed_crit_since >= c.trip_after_s:
                self.operator_run = False
                self.mode = TRIPPED
                self.raise_alarm(f"Belt STOPPED: {faults[0].name} persists at minimum speed")
                return Decision(False, self.command_speed(load_g))
        else:
            self.minspeed_crit_since = None

        # escalate: fault persists too long
        if self.fault_since is not None and now - self.fault_since >= c.escalate_after_s:
            self.mode = ALARM
            self.raise_alarm(f"{faults[0].name} not resolved after {now - self.fault_since:.0f}s of adaptation")

        # recover: step speed back up after clean running
        if not any_fault and self.clear_since is not None:
            capped = self.safe_ceiling is not None and now < self.ceiling_until and self.command_speed(load_g) >= self.safe_ceiling
            if self.fault_offset > 0 and not capped and now - self.clear_since >= c.recover_after_s and now - self.last_recover_t >= c.settle_s:
                self.fault_offset = max(0.0, self.fault_offset - c.recover_step_pct)
                self.last_recover_t = now
                self._log("action", f"Fault cleared, recovering speed to {self.command_speed(load_g):.0f}%")
            if now - self.clear_since >= c.recover_after_s and self.alarm_active:
                self.alarm_active = False
                self._log("info", "Alarm cleared: faults gone")
            if self.fault_offset <= 0 and self.mode in (ADAPTING, ALARM):
                self.mode = NORMAL

        return Decision(True, self.command_speed(load_g))
