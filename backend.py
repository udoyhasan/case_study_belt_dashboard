"""MQTT <-> detector <-> adaptive controller. Runs in background threads, independent of the UI."""
import csv
import json
import os
import threading
import time
from collections import deque

import paho.mqtt.client as mqtt
import pandas as pd

import config as C
from controller import AdaptiveController
from detector import RFNoisyDetector, ThresholdDetector

FIELDS = ["load_g", "bus_v", "current_ma", "power_mw", "temp_c", "vib_rms_g", "vib_peak_g",
          "ax", "ay", "az", "speed_pct", "step_delay_us", "run", "estop", "rssi"]


class Backend:
    def __init__(self, host=C.MQTT_HOST, port=C.MQTT_PORT):
        self.host, self.port = host, port
        self.lock = threading.RLock()
        self.th = C.Thresholds()
        self.cfg = C.ControllerConfig()
        # Initialize Random Forest Noisy ML detector
        self.detector = RFNoisyDetector()
        self.detector_name = getattr(self.detector, "model_name", "Random Forest (rf_noisy)")
        self.ctrl = AdaptiveController(self.cfg, self.th)
        self.ctrl.log = self.add_log

        self.rows = deque(maxlen=C.HISTORY_LEN)
        self.events = deque(maxlen=300)
        self.faults = []
        self.broker_ok = False
        self.device_status = "unknown"
        self.last_rx = 0.0
        self.run_since = None
        self.prev_run = False
        self.prev_estop = False
        self.last_cmd = None
        self.last_cmd_t = 0.0
        self.rx_count = 0
        self.log_to_csv = True
        self._csv = None
        self._csv_writer = None

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"dashboard-{os.getpid()}")
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        self.client.reconnect_delay_set(1, 5)

    # ---------------- lifecycle ----------------
    def start(self):
        try:
            self.client.connect_async(self.host, self.port, keepalive=15)
        except Exception as e:  # noqa
            self.add_log("error", f"MQTT connect error: {e}")
        self.client.loop_start()
        threading.Thread(target=self._watchdog, daemon=True).start()

    def add_log(self, kind, text):
        with self.lock:
            self.events.appendleft({"time": time.strftime("%H:%M:%S"), "kind": kind, "text": text})

    # ---------------- mqtt callbacks ----------------
    def _on_connect(self, client, userdata, flags, rc, props=None):
        self.broker_ok = (rc == 0)
        if rc == 0:
            for t in (C.TOPIC_TELEMETRY, C.TOPIC_EVENT, C.TOPIC_STATUS):
                client.subscribe(t, qos=1)
            self.add_log("info", f"Connected to broker {self.host}:{self.port}")
        else:
            self.add_log("error", f"Broker refused connection: {rc}")

    def _on_disconnect(self, client, userdata, flags, rc, props=None):
        self.broker_ok = False
        self.add_log("error", "Disconnected from broker")

    def _on_message(self, client, userdata, msg):
        try:
            if msg.topic == C.TOPIC_STATUS:
                self.device_status = msg.payload.decode()
                self.add_log("info", f"ESP32 is {self.device_status}")
            elif msg.topic == C.TOPIC_EVENT:
                self._handle_event(json.loads(msg.payload))
            elif msg.topic == C.TOPIC_TELEMETRY:
                self._handle_telemetry(json.loads(msg.payload))
        except Exception as e:  # noqa - never kill the network thread
            self.add_log("error", f"Bad message on {msg.topic}: {e}")

    def _handle_event(self, ev):
        t, d = ev.get("type"), ev.get("detail", "")
        with self.lock:
            self.add_log("device", f"ESP32 event: {t} - {d}")
            if t == "estop":
                self.ctrl.device_estop()
            elif t == "start":
                self.ctrl.operator_start()

    # ---------------- core processing ----------------
    def _handle_telemetry(self, d):
        now = time.time()
        row = {"t": now}
        for k in FIELDS:
            v = d.get(k)
            row[k] = (bool(v) if k in ("run", "estop") else (float(v) if v is not None else None))
        with self.lock:
            self.last_rx = now
            self.rx_count += 1
            running = bool(row["run"])
            if running and not self.prev_run:
                self.run_since = now
                if not self.ctrl.operator_run and not row["estop"]:
                    self.ctrl.operator_run = True        # started from the physical button
            if not running:
                self.run_since = None
            self.prev_run = running
            self.prev_estop = bool(row["estop"])

            window = list(self.rows)[-(self.th.smooth_n - 1):] + [row]
            run_for = (now - self.run_since) if self.run_since else 0.0
            self.faults = self.detector.detect(window, run_for)
            decision = self.ctrl.update(self.faults, now, row["load_g"] or 0.0, bool(row["estop"]))
            self._send_if_needed(decision, now)

            row["cmd_speed"] = decision.speed_pct if decision.run else 0.0
            row["fault"] = ",".join(f.code for f in self.faults) or "NORMAL"
            row["mode"] = self.ctrl.mode
            self.rows.append(row)
            self._log_csv(row)

    def _send_if_needed(self, dec, now):
        if self.ctrl.mode == "E-STOP":
            return
        cmd = (bool(dec.run), round(dec.speed_pct))
        changed = cmd != self.last_cmd
        if changed or now - self.last_cmd_t > 5.0:
            self.publish_command(dec.run, dec.speed_pct, "adaptive" if self.ctrl.mode in ("ADAPTING", "ALARM") else "dashboard")
            self.last_cmd = cmd

    def publish_command(self, run, speed, reason, **extra):
        payload = {"run": bool(run), "speed_pct": round(float(speed), 1), "reason": reason, **extra}
        self.client.publish(C.TOPIC_COMMAND, json.dumps(payload), qos=1)
        self.last_cmd_t = time.time()

    def _watchdog(self):
        lost = False
        while True:
            time.sleep(1)
            stale = self.last_rx and (time.time() - self.last_rx > C.STALE_AFTER_S)
            if stale and not lost:
                lost = True
                self.add_log("error", "No telemetry from ESP32 (data link lost)")
            elif not stale and lost:
                lost = False
                self.add_log("info", "Telemetry restored")

    # ---------------- logging for future ML ----------------
    def _log_csv(self, row):
        if not self.log_to_csv:
            return
        try:
            if self._csv is None:
                os.makedirs(C.LOG_DIR, exist_ok=True)
                path = os.path.join(C.LOG_DIR, "telemetry_log.csv")
                new = not os.path.exists(path)
                self._csv = open(path, "a", newline="")
                cols = ["t"] + FIELDS + ["cmd_speed", "fault", "mode"]
                self._csv_writer = csv.DictWriter(self._csv, fieldnames=cols, extrasaction="ignore")
                if new:
                    self._csv_writer.writeheader()
            self._csv_writer.writerow(row)
            self._csv.flush()
        except Exception:  # noqa
            self.log_to_csv = False

    # ---------------- API for the UI ----------------
    def df(self, seconds=120) -> pd.DataFrame:
        with self.lock:
            rows = list(self.rows)
        if not rows:
            return pd.DataFrame()
        df = pd.DataFrame(rows)
        df = df[df["t"] >= df["t"].iloc[-1] - seconds].copy()
        df["time"] = pd.to_datetime(df["t"], unit="s")
        return df

    def comm_ok(self) -> bool:
        return bool(self.last_rx) and (time.time() - self.last_rx) <= C.STALE_AFTER_S

    # operator actions
    def start_belt(self):
        with self.lock:
            self.ctrl.operator_start()
            self.last_cmd = None

    def stop_belt(self):
        with self.lock:
            self.ctrl.operator_stop()
            self.last_cmd = None

    def tare(self):
        self.publish_command(self.ctrl.operator_run, self.cfg.target_speed_pct, "tare", tare=True)

    def inject_manual_telemetry(self, fault_type: str, custom_values: dict = None) -> dict:
        """Injects simulated faulty or normal telemetry to trigger and test rf_noisy detection."""
        fault_type = (fault_type or "normal").lower().strip()
        now = time.time()

        presets = {
            "overload": {
                "load_g": 1450.0,
                "bus_v": 24.1,
                "current_ma": 580.0,
                "power_mw": 13978.0,
                "temp_c": 38.5,
                "vib_rms_g": 0.052,
                "vib_peak_g": 0.12,
                "speed_pct": 60.0,
                "step_delay_us": 1800.0,
                "run": True,
                "estop": False,
                "rssi": -55,
            },
            "vibration": {
                "load_g": 480.0,
                "bus_v": 24.0,
                "current_ma": 420.0,
                "power_mw": 10080.0,
                "temp_c": 41.0,
                "vib_rms_g": 0.285,
                "vib_peak_g": 0.65,
                "speed_pct": 60.0,
                "step_delay_us": 1800.0,
                "run": True,
                "estop": False,
                "rssi": -55,
            },
            "overheat": {
                "load_g": 400.0,
                "bus_v": 24.0,
                "current_ma": 460.0,
                "power_mw": 11040.0,
                "temp_c": 78.5,
                "vib_rms_g": 0.058,
                "vib_peak_g": 0.14,
                "speed_pct": 60.0,
                "step_delay_us": 1800.0,
                "run": True,
                "estop": False,
                "rssi": -55,
            },
            "stall": {
                "load_g": 850.0,
                "bus_v": 23.6,
                "current_ma": 1950.0,
                "power_mw": 46020.0,
                "temp_c": 56.0,
                "vib_rms_g": 0.088,
                "vib_peak_g": 0.21,
                "speed_pct": 0.0,
                "step_delay_us": 50000.0,
                "run": False,
                "estop": False,
                "rssi": -55,
            },
            "overspeed": {
                "load_g": 250.0,
                "bus_v": 24.2,
                "current_ma": 340.0,
                "power_mw": 8228.0,
                "temp_c": 35.0,
                "vib_rms_g": 0.082,
                "vib_peak_g": 0.18,
                "speed_pct": 95.0,
                "step_delay_us": 1150.0,
                "run": True,
                "estop": False,
                "rssi": -55,
            },
            "multifault": {
                "load_g": 1450.0,
                "bus_v": 23.5,
                "current_ma": 1950.0,
                "power_mw": 45825.0,
                "temp_c": 79.0,
                "vib_rms_g": 0.320,
                "vib_peak_g": 0.75,
                "speed_pct": 60.0,
                "step_delay_us": 1800.0,
                "run": True,
                "estop": False,
                "rssi": -55,
            },
            "normal": {
                "load_g": 250.0,
                "bus_v": 24.0,
                "current_ma": 360.0,
                "power_mw": 8640.0,
                "temp_c": 34.0,
                "vib_rms_g": 0.045,
                "vib_peak_g": 0.09,
                "speed_pct": 60.0,
                "step_delay_us": 1800.0,
                "run": True,
                "estop": False,
                "rssi": -55,
            },
        }

        template = presets.get(fault_type, presets["normal"]).copy()
        if custom_values:
            template.update(custom_values)

        # Inject 5 samples to fill the model's 5-window buffer
        with self.lock:
            for i in range(5):
                t_sample = now - (4 - i) * 0.2
                row = dict(template)
                row["t"] = t_sample
                # Subtle jitter on history to make realistic time-series
                if fault_type != "normal":
                    row["load_g"] += (i - 2) * 4.0
                    row["current_ma"] += (i - 2) * 2.0
                self._handle_telemetry(row)

            detected = [{"code": f.code, "name": f.name, "severity": f.severity, "message": f.message} for f in self.faults]
            self.add_log("error" if fault_type != "normal" else "info",
                         f"[MANUAL INJECT] Injected '{fault_type}'. Model detected: {', '.join(f.code for f in self.faults) or 'NORMAL'}")

        # Also publish message to MQTT broker if connected
        try:
            self.client.publish(C.TOPIC_TELEMETRY, json.dumps(template), qos=1)
        except Exception:
            pass

        return {
            "status": "success",
            "fault_injected": fault_type,
            "detected_faults": detected,
            "last_readings": template,
            "model_name": getattr(self.detector, "model_name", "Random Forest (rf_noisy)"),
            "controller_mode": self.ctrl.mode,
        }
