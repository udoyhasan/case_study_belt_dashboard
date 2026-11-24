"""Web dashboard server.   Run:  ./start_dashboard.sh   (or: uvicorn server:app --app-dir dashboard)

Serves dashboard/static/ and pushes live state to the browser over a WebSocket.
The MQTT backend, fault detector and adaptive controller run in background threads (backend.py).
"""
import asyncio
import json
import os
import time
from contextlib import asynccontextmanager

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, PlainTextResponse
from fastapi.staticfiles import StaticFiles

import config as C
from backend import Backend

STATIC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
PUSH_EVERY_S = 0.5
HISTORY_ON_CONNECT_S = 600

be = Backend()


@asynccontextmanager
async def lifespan(_app):
    be.start()
    yield


app = FastAPI(lifespan=lifespan)
app.mount("/static", StaticFiles(directory=STATIC), name="static")


@app.get("/")
def index():
    return FileResponse(os.path.join(STATIC, "index.html"))


@app.get("/manual")
def manual():
    """Hidden diagnostic endpoint for manual fault injection (not linked on home)."""
    return FileResponse(os.path.join(STATIC, "manual.html"))


@app.post("/api/manual/inject")
async def manual_inject(payload: dict = None):
    """Prefills sensor telemetry with the selected fault to test rf_noisy model response."""
    payload = payload or {}
    fault_type = payload.get("fault", "normal")
    custom = payload.get("custom", None)
    return be.inject_manual_telemetry(fault_type, custom)


@app.get("/api/manual/status")
def manual_status():
    with be.lock:
        return {
            "model": getattr(be, "detector_name", "Random Forest (rf_noisy)"),
            "faults": [{"code": f.code, "name": f.name, "severity": f.severity, "message": f.message} for f in be.faults],
            "mode": be.ctrl.mode,
            "last_rx": be.last_rx,
        }


@app.get("/api/log.csv")
def log_csv():
    path = os.path.join(C.LOG_DIR, "telemetry_log.csv")
    if not os.path.exists(path):
        return PlainTextResponse("No data logged yet.", status_code=404)
    return FileResponse(path, filename="conveyor_log.csv", media_type="text/csv")


def _clean(row: dict) -> dict:
    return {k: (None if isinstance(v, float) and v != v else v) for k, v in row.items()}


def snapshot(since: float) -> dict:
    th, cfg, ctrl = be.th, be.cfg, be.ctrl
    with be.lock:
        rows = [_clean(r) for r in be.rows if r["t"] > since]
        faults = [{"code": f.code, "name": f.name, "severity": f.severity, "message": f.message} for f in be.faults]
        events = list(be.events)[:40]
        return {
            "type": "state",
            "now": time.time(),
            "model": getattr(be, "detector_name", "Random Forest (rf_noisy)"),
            "broker": be.broker_ok,
            "device": be.device_status,
            "comm_ok": be.comm_ok(),
            "last_rx": be.last_rx,
            "mode": ctrl.mode,
            "alarm": {"active": ctrl.alarm_active, "silenced": ctrl.alarm_silenced, "reason": ctrl.alarm_reason},
            "faults": faults,
            "control": {"run": ctrl.operator_run, "target": cfg.target_speed_pct,
                        "auto_adapt": cfg.auto_adapt, "load_aware": cfg.load_aware_speed},
            "limits": {"load_warn": th.load_warn_g, "load_crit": th.load_crit_g,
                       "temp_warn": th.temp_warn_c, "temp_crit": th.temp_crit_c,
                       "cur_idle": th.current_idle_ma, "cur_per_kg": th.current_per_kg_ma,
                       "cur_tol": th.current_tol_ratio, "cur_floor": th.current_tol_floor_ma, "cur_crit": th.current_crit_ma,
                       "vib_base": th.vib_base_g, "vib_per_kg": th.vib_per_kg_g, "vib_crit_factor": th.vib_crit_factor},
            "rows": rows,
            "events": events,
        }


def handle_command(msg: dict):
    a, v = msg.get("action"), msg.get("value")
    cfg = be.cfg
    if a == "start":
        be.start_belt()
    elif a == "stop":
        be.stop_belt()
    elif a == "speed" and v is not None:
        cfg.target_speed_pct = max(5.0, min(100.0, float(v)))
    elif a == "auto_adapt":
        cfg.auto_adapt = bool(v)
    elif a == "load_aware":
        cfg.load_aware_speed = bool(v)
    elif a == "tare":
        be.tare()
    elif a == "ack":
        with be.lock:
            be.ctrl.acknowledge()


@app.websocket("/ws")
async def ws(socket: WebSocket):
    await socket.accept()
    since = time.time() - HISTORY_ON_CONNECT_S

    async def reader():
        while True:
            handle_command(json.loads(await socket.receive_text()))

    task = asyncio.create_task(reader())
    try:
        while not task.done():
            snap = snapshot(since)
            if snap["rows"]:
                since = snap["rows"][-1]["t"]
            await socket.send_text(json.dumps(snap, allow_nan=False, default=str))
            await asyncio.sleep(PUSH_EVERY_S)
    except (WebSocketDisconnect, RuntimeError):
        pass
    finally:
        task.cancel()
