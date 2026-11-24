# Conveyor Belt Dashboard V2 (AI/ML Driven)

Real-time conveyor monitoring, mimic visualization, and fault classification driven by the trained Random Forest (`rf_noisy`) machine learning pipeline and adaptive speed controller.

---

## Features
- **Machine Learning Fault Classification (`rf_noisy`)**: Real-time multi-label classification of Overload, Excessive Vibration, Overheating, Motor Stall / Jam, and Overspeed.
- **Noise-Resilient Processing**: Built-in `NoiseFilter` processing sliding 5-sample telemetry windows with median and spread filtering.
- **Adaptive Control**: Auto-derating and alarm escalation when mechanical/electrical anomalies persist.
- **Live SVG Mimic & Trends**: Real-time belt animation, sensor trends, power consumption, and fault logging over WebSockets.
- **Diagnostic Console (`/manual`)**: Hidden test interface to inject pre-filled sensor anomalies and verify AI detection response.
- **Production Service Deployment**: Ready-to-deploy systemd service for Raspberry Pi and LaunchAgent for macOS.

---

## Quick Start on Raspberry Pi

### 1. Clone & Setup Virtual Environment
```bash
git clone https://github.com/udoyhasan/case_study_belt_dashboard.git conveyor
cd conveyor
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

### 2. Run Interactively
```bash
./start_dashboard.sh
```
- Open Dashboard: `http://<PI_IP>:8000`
- Open Diagnostic Injection Console: `http://<PI_IP>:8000/manual`

---

## Deploy as a Service (Start on Boot)

```bash
# Automatically install and enable systemd service:
sudo ./manage_service.sh install

# Check service status:
./manage_service.sh status

# Live service logs:
sudo journalctl -u conveyor-dashboard-v2.service -f
```
