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

FIELDS = ["load_g", "bus_v", "current_ma", "power_mw", "temp_c", "vib_rms_g", "vib_peak_g", "vib_g",
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
        self.active_override = None
        self.active_override_custom = None

        try:
            self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"dashboard-{os.getpid()}")
        except (AttributeError, TypeError):
            self.client = mqtt.Client(client_id=f"dashboard-{os.getpid()}")
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
        threading.Thread(target=self._override_ticker, daemon=True).start()

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
            print(f"[MQTT ERROR] Bad message on {msg.topic}: {e} (payload: {msg.payload[:100]!r})")

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
        # If active fault override is engaged, replace sensor readings with persistent faulty data
        if self.active_override is not None:
            d = self._build_fault_frame(self.active_override, now, self.active_override_custom)

        row = {"t": now}
        for k in FIELDS:
            v = d.get(k)
            row[k] = (bool(v) if k in ("run", "estop") else (float(v) if v is not None else None))

        # Ensure essential telemetry data bits have valid non-null defaults
        if row.get("vib_g") is None:
            row["vib_g"] = row.get("vib_rms_g") if row.get("vib_rms_g") is not None else 0.05
        if row.get("vib_rms_g") is None:
            row["vib_rms_g"] = row.get("vib_g", 0.05)
        if row.get("temp_c") is None:
            row["temp_c"] = 25.0
        if row.get("load_g") is None:
            row["load_g"] = 0.0
        if row.get("current_ma") is None:
            row["current_ma"] = 0.0
        if row.get("bus_v") is None:
            row["bus_v"] = 12.0
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
    def _override_ticker(self):
        """Continuously streams realistic telemetry at 5 Hz while override is active if no external MQTT is arriving."""
        while True:
            time.sleep(0.2)
            if self.active_override is not None:
                now = time.time()
                # If no hardware message arrived in last 250ms, generate next override frame
                if now - self.last_rx >= 0.2:
                    frame = self._build_fault_frame(self.active_override, now, self.active_override_custom)
                    self._handle_telemetry(frame)

    def _build_fault_frame(self, fault_type, now, custom=None):
        import random
        fault_type = (fault_type or "normal").lower().strip()

        presets = {
            "overload": {
                "load_g": 1450.0 + random.uniform(-4.0, 4.0),
                "bus_v": 23.8 + random.uniform(-0.1, 0.1),
                "current_ma": 580.0 + random.uniform(-5.0, 5.0),
                "power_mw": 13800.0 + random.uniform(-40.0, 40.0),
                "temp_c": 38.5 + random.uniform(-0.2, 0.2),
                "vib_rms_g": 0.058 + random.uniform(-0.002, 0.002),
                "vib_peak_g": 0.12 + random.uniform(-0.005, 0.005),
                "vib_g": 0.058,
                "speed_pct": self.cfg.target_speed_pct,
                "step_delay_us": 1800.0,
                "run": True,
                "estop": False,
                "rssi": -55,
            },
            "vibration": {
                "load_g": 380.0 + random.uniform(-3.0, 3.0),
                "bus_v": 24.0 + random.uniform(-0.1, 0.1),
                "current_ma": 390.0 + random.uniform(-4.0, 4.0),
                "power_mw": 9360.0 + random.uniform(-40.0, 40.0),
                "temp_c": 36.0 + random.uniform(-0.2, 0.2),
                "vib_rms_g": 0.285 + random.uniform(-0.008, 0.008),
                "vib_peak_g": 0.65 + random.uniform(-0.02, 0.02),
                "vib_g": 0.285,
                "speed_pct": self.cfg.target_speed_pct,
                "step_delay_us": 1800.0,
                "run": True,
                "estop": False,
                "rssi": -55,
            },
            "overheat": {
                "load_g": 350.0 + random.uniform(-3.0, 3.0),
                "bus_v": 23.9 + random.uniform(-0.1, 0.1),
                "current_ma": 460.0 + random.uniform(-4.0, 4.0),
                "power_mw": 11000.0 + random.uniform(-50.0, 50.0),
                "temp_c": 78.5 + random.uniform(-0.3, 0.3),
                "vib_rms_g": 0.048 + random.uniform(-0.002, 0.002),
                "vib_peak_g": 0.10 + random.uniform(-0.005, 0.005),
                "vib_g": 0.048,
                "speed_pct": self.cfg.target_speed_pct,
                "step_delay_us": 1800.0,
                "run": True,
                "estop": False,
                "rssi": -55,
            },
            "stall": {
                "load_g": 350.0 + random.uniform(-3.0, 3.0),
                "bus_v": 23.4 + random.uniform(-0.1, 0.1),
                "current_ma": 1950.0 + random.uniform(-10.0, 10.0),
                "power_mw": 45600.0 + random.uniform(-100.0, 100.0),
                "temp_c": 58.0 + random.uniform(-0.2, 0.2),
                "vib_rms_g": 0.045 + random.uniform(-0.002, 0.002),
                "vib_peak_g": 0.09 + random.uniform(-0.005, 0.005),
                "vib_g": 0.045,
                "speed_pct": 0.0,
                "step_delay_us": 0.0,
                "run": True,
                "estop": False,
                "rssi": -55,
            },
            "overspeed": {
                "load_g": 250.0 + random.uniform(-3.0, 3.0),
                "bus_v": 24.1 + random.uniform(-0.1, 0.1),
                "current_ma": 420.0 + random.uniform(-4.0, 4.0),
                "power_mw": 10120.0 + random.uniform(-40.0, 40.0),
                "temp_c": 35.0 + random.uniform(-0.2, 0.2),
                "vib_rms_g": 0.048 + random.uniform(-0.002, 0.002),
                "vib_peak_g": 0.10 + random.uniform(-0.005, 0.005),
                "vib_g": 0.048,
                "speed_pct": 95.0,
                "step_delay_us": 1200.0,
                "run": True,
                "estop": False,
                "rssi": -55,
            },
            "multifault": {
                "load_g": 1450.0 + random.uniform(-5.0, 5.0),
                "bus_v": 23.2 + random.uniform(-0.1, 0.1),
                "current_ma": 1950.0 + random.uniform(-10.0, 10.0),
                "power_mw": 45240.0 + random.uniform(-100.0, 100.0),
                "temp_c": 79.0 + random.uniform(-0.3, 0.3),
                "vib_rms_g": 0.320 + random.uniform(-0.01, 0.01),
                "vib_peak_g": 0.75 + random.uniform(-0.02, 0.02),
                "vib_g": 0.320,
                "speed_pct": 60.0,
                "step_delay_us": 1800.0,
                "run": True,
                "estop": False,
                "rssi": -55,
            },
            "normal": {
                "load_g": 250.0 + random.uniform(-3.0, 3.0),
                "bus_v": 24.0 + random.uniform(-0.1, 0.1),
                "current_ma": 360.0 + random.uniform(-4.0, 4.0),
                "power_mw": 8640.0 + random.uniform(-30.0, 30.0),
                "temp_c": 32.0 + random.uniform(-0.2, 0.2),
                "vib_rms_g": 0.045 + random.uniform(-0.002, 0.002),
                "vib_peak_g": 0.09 + random.uniform(-0.005, 0.005),
                "vib_g": 0.045,
                "speed_pct": self.cfg.target_speed_pct,
                "step_delay_us": 1800.0,
                "run": self.ctrl.operator_run,
                "estop": False,
                "rssi": -55,
            },
        }

        template = presets.get(fault_type, presets["normal"]).copy()
        if custom:
            template.update(custom)
        template["t"] = now
        template["device"] = "conveyor-node-01"
        return template

    def inject_manual_telemetry(self, fault_type: str, custom_values: dict = None):
        """Activates persistent real faulty sensor data across all sensors until 'normal' or 'local' is clicked."""
        now = time.time()
        fault_type = (fault_type or "normal").lower().strip()

        if fault_type in ("normal", "local", "clear", "reset"):
            self.active_override = None
            self.active_override_custom = None
            with self.lock:
                for i in range(5):
                    frame = self._build_fault_frame("normal", now - (4 - i) * 0.2)
                    self._handle_telemetry(frame)
            return {"status": "success", "mode": "normal", "active_override": None}

        # Engage persistent faulty data override
        self.active_override = fault_type
        self.active_override_custom = custom_values

        with self.lock:
            for i in range(5):
                frame = self._build_fault_frame(fault_type, now - (4 - i) * 0.2, custom_values)
                self._handle_telemetry(frame)

        detected = [{"code": f.code, "name": f.name, "severity": f.severity, "message": f.message} for f in self.faults]
        return {
            "status": "success",
            "active_override": fault_type,
            "detected_faults": detected,
            "model_name": getattr(self.detector, "model_name", "Random Forest (rf_noisy)"),
            "controller_mode": self.ctrl.mode,
        }
