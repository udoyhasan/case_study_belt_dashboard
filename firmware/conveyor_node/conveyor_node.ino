/*
  Smart Load-Aware Conveyor - ESP32 node
  ---------------------------------------------------------------
  Sensors : HX711 load cell, INA219 (current/voltage), MPU6050 (vibration),
            temperature (DS18B20 or analog LM35/TMP36)
  Actuator: NEMA stepper via A4988 (STEP/DIR, EN wired directly)
  Inputs  : START button, EMERGENCY-STOP button (both wired 3.3V -> button -> GPIO)
  Link    : WiFi + MQTT (PubSubClient) to Mosquitto running on the laptop

  MQTT topics
    conveyor/telemetry  ESP32 -> laptop   JSON sensor frame (every PUBLISH_INTERVAL_MS)
    conveyor/event      ESP32 -> laptop   JSON button / safety events
    conveyor/status     ESP32 -> laptop   "online"/"offline" (retained, Last-Will)
    conveyor/command    laptop -> ESP32   JSON {"run":true,"speed_pct":60,"reason":"..."}

  Libraries (Arduino Library Manager):
    PubSubClient            (Nick O'Leary)
    ArduinoJson             (Benoit Blanchon, version 7.x)
    HX711                   (Bogdan Necula "bogde")
    Adafruit INA219         (+ Adafruit BusIO)
    Adafruit MPU6050        (+ Adafruit Unified Sensor)
    OneWire + DallasTemperature   (only if TEMP_SENSOR_TYPE == TEMP_DS18B20)
  Board: "ESP32 Dev Module" (esp32 core 2.x or 3.x both work)
*/

#include <WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HX711.h>
#include <Adafruit_INA219.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// =====================================================================
//  USER CONFIGURATION  -  change everything here, nothing else needed
// =====================================================================

// ---------- WiFi / MQTT ----------
const char* WIFI_SSID     = "Mission-e-Elahi";
const char* WIFI_PASSWORD = "m!ss!0n!mp0ss!ble";
const char* MQTT_BROKER   = "192.168.2.81";    // Raspberry Pi IP (MQTT Broker)
const uint16_t MQTT_PORT  = 1883;
const char* MQTT_USER     = "";                // leave "" when broker is anonymous
const char* MQTT_PASS     = "";
const char* DEVICE_ID     = "conveyor-esp32-01";

const char* TOPIC_TELEMETRY = "conveyor/telemetry";
const char* TOPIC_EVENT     = "conveyor/event";
const char* TOPIC_STATUS    = "conveyor/status";
const char* TOPIC_COMMAND   = "conveyor/command";

// ---------- PINS (all in one place) ----------
// INA219 on its own I2C bus (Wire)
const int PIN_INA_SDA   = 18;
const int PIN_INA_SCL   = 19;
// MPU6050 on its own I2C bus (Wire1)
const int PIN_MPU_SDA   = 21;
const int PIN_MPU_SCL   = 22;
// HX711 load cell amplifier
const int PIN_HX711_DT  = 16;
const int PIN_HX711_SCK = 17;
// A4988 stepper driver (ENABLE pin is wired directly, not controlled by the ESP32)
const int PIN_STEP      = 23;
const int PIN_DIR       = 2;
// Temperature sensor (3-pin: VCC / GND / signal)
const int PIN_TEMP      = 4;     // DS18B20: data pin (4.7k pull-up to 3.3V). Analog: use an ADC1 pin (32-39)
// Push buttons: one leg to 3.3V, other leg to the GPIO  => pressed = HIGH
const int PIN_BTN_START = 13;
const int PIN_BTN_ESTOP = 14;
// Optional status LED (use -1 to disable)
const int PIN_LED       = -1;

// ---------- Temperature sensor type ----------
#define TEMP_DS18B20   1          // digital 1-wire (waterproof probe / 3-pin module)
#define TEMP_ANALOG    2          // LM35 / TMP36 style analog output
#define TEMP_SENSOR_TYPE  TEMP_DS18B20
// Analog sensor conversion: temp_C = (volts - TEMP_ANALOG_OFFSET_V) / TEMP_ANALOG_V_PER_C
const float TEMP_ANALOG_V_PER_C   = 0.010f;   // LM35 = 0.010 V/C, TMP36 = 0.010 V/C
const float TEMP_ANALOG_OFFSET_V  = 0.0f;     // LM35 = 0.0 V,    TMP36 = 0.5 V

// ---------- Sensor settings ----------
const uint8_t INA219_ADDR   = 0x40;
const uint8_t MPU6050_ADDR  = 0x68;
const float   LOADCELL_CALIBRATION = 420.0f;  // raw counts per gram -> calibrate with a known weight!
const bool    LOADCELL_INVERT      = false;   // set true if weight reads negative
const uint16_t VIB_SAMPLE_PERIOD_MS = 10;     // accelerometer sampled at 100 Hz

// ---------- Stepper / belt ----------
const bool  DIR_FORWARD_LEVEL = HIGH;         // flip if the belt runs backwards
// Step delay in microseconds: STEP pin is HIGH for <delay> us, then LOW for <delay> us
// (same as: HIGH; delayMicroseconds(d); LOW; delayMicroseconds(d);).  Smaller delay = faster.
const float STEP_DELAY_US_FASTEST = 1750.0f;  // delay at speed 100 %
const float STEP_DELAY_US_SLOWEST = 2250.0f;  // delay at lowest speed (also the start-up delay)
const float RAMP_US_PER_SEC       = 1000.0f;  // how fast the delay may change (acceleration limit)
const float DEFAULT_SPEED_PCT = 50.0f;        // speed used when START is pressed
const float MIN_SPEED_PCT     = 5.0f;         // commands below this are clamped

// ---------- Timing ----------
const uint32_t PUBLISH_INTERVAL_MS = 200;     // 5 telemetry frames per second
const uint32_t DEBOUNCE_MS         = 40;

// ---------- Local hard safety limits (independent of the laptop) ----------
const float HARD_MAX_TEMP_C     = 80.0f;
const float HARD_MAX_CURRENT_MA = 2500.0f;
const float HARD_MAX_LOAD_G     = 20000.0f;

// =====================================================================
//  END OF USER CONFIGURATION
// =====================================================================

#if TEMP_SENSOR_TYPE == TEMP_DS18B20
  #include <OneWire.h>
  #include <DallasTemperature.h>
  OneWire oneWire(PIN_TEMP);
  DallasTemperature tempSensor(&oneWire);
#endif

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);
HX711        scale;
Adafruit_INA219 ina219(INA219_ADDR);
Adafruit_MPU6050 mpu;

// ---- sensor availability flags ----
bool okIna = false, okMpu = false, okScale = false;

// ---- latest readings ----
float loadG = 0, busV = 0, currentMA = 0, powerMW = 0, tempC = NAN;
float ax = 0, ay = 0, az = 0;
float vibRms = 0, vibPeak = 0;

// ---- vibration window accumulators ----
double vibSum = 0, vibSumSq = 0; uint32_t vibN = 0; float vibMax = 0;
uint32_t lastVibSample = 0;

// ---- motor state ----
bool  beltRun = false;           // requested run state
bool  estopLatched = false;      // emergency stop latched until START is pressed
float targetSpeedPct = 0;        // requested
float currentDelayUs = 0;        // actual step delay (ramped), 0 = stopped
float targetDelayUs = 0;
uint32_t lastRampMs = 0;
String lastReason = "boot";

// ---- buttons ----
struct Button { int pin; bool stable; bool lastRead; uint32_t changedAt; };
Button btnStart = {PIN_BTN_START, false, false, 0};
Button btnEstop = {PIN_BTN_ESTOP, false, false, 0};

uint32_t lastPublish = 0, lastMqttTry = 0, lastWifiTry = 0;
String topicCmdStr;

// ---------------------------------------------------------------------
//  Stepper: a hardware timer fires every <delay> us and toggles the STEP pin,
//  so the pulse timing is exact and loop() (WiFi/MQTT/sensors) is never blocked.
// ---------------------------------------------------------------------
hw_timer_t* stepTimer = nullptr;
volatile bool stepActive = false;
volatile bool stepLevel = false;
uint32_t stepPeriodUs = 0;

void IRAM_ATTR onStepTimer() {
  if (!stepActive) return;
  stepLevel = !stepLevel;
  digitalWrite(PIN_STEP, stepLevel);
}

void stepperTimerInit() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  stepTimer = timerBegin(1000000);                       // 1 MHz -> 1 tick = 1 us
  timerAttachInterrupt(stepTimer, &onStepTimer);
  timerAlarm(stepTimer, (uint64_t)STEP_DELAY_US_SLOWEST, true, 0);
#else
  stepTimer = timerBegin(0, 80, true);                   // 80 MHz / 80 = 1 MHz
  timerAttachInterrupt(stepTimer, &onStepTimer, true);
  timerAlarmWrite(stepTimer, (uint64_t)STEP_DELAY_US_SLOWEST, true);
  timerAlarmEnable(stepTimer);
#endif
}

// delay <= 0 stops the motor
void stepperSetDelay(float delayUs) {
  if (delayUs < 1.0f) {
    stepActive = false;
    stepLevel = false;
    digitalWrite(PIN_STEP, LOW);
    return;
  }
  uint32_t period = (uint32_t)(delayUs + 0.5f);
  if (period != stepPeriodUs) {
    stepPeriodUs = period;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    timerAlarm(stepTimer, period, true, 0);
#else
    timerAlarmWrite(stepTimer, period, true);
#endif
  }
  stepActive = true;
}

// speed 100 % -> fastest (smallest) delay, speed ~0 % -> slowest (largest) delay
float speedPctToDelayUs(float pct) {
  if (pct <= 0.0f) return 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return STEP_DELAY_US_SLOWEST - (STEP_DELAY_US_SLOWEST - STEP_DELAY_US_FASTEST) * (pct / 100.0f);
}

void motorUpdate() {
  uint32_t now = millis();
  float dt = (now - lastRampMs) / 1000.0f;
  lastRampMs = now;

  targetDelayUs = (beltRun && !estopLatched) ? speedPctToDelayUs(targetSpeedPct) : 0.0f;

  if (estopLatched || targetDelayUs == 0.0f) {
    currentDelayUs = 0;                                  // stop immediately
  } else {
    if (currentDelayUs == 0) currentDelayUs = STEP_DELAY_US_SLOWEST;   // start slow, then ramp
    float maxDelta = RAMP_US_PER_SEC * dt;
    if (currentDelayUs < targetDelayUs) currentDelayUs = min(currentDelayUs + maxDelta, targetDelayUs);
    else                                currentDelayUs = max(currentDelayUs - maxDelta, targetDelayUs);
  }
  stepperSetDelay(currentDelayUs);
}

// ---------------------------------------------------------------------
//  Events
// ---------------------------------------------------------------------
void publishEvent(const char* type, const char* detail) {
  if (!mqtt.connected()) return;
  JsonDocument doc;
  doc["device"] = DEVICE_ID;
  doc["ts"] = millis();
  doc["type"] = type;
  doc["detail"] = detail;
  char buf[200];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  mqtt.publish(TOPIC_EVENT, (const uint8_t*)buf, n, false);
}

void emergencyStop(const char* why) {
  if (!estopLatched) {
    estopLatched = true;
    beltRun = false;
    currentDelayUs = 0;
    stepperSetDelay(0);
    lastReason = why;
    Serial.printf("[ESTOP] %s\n", why);
    publishEvent("estop", why);
  }
}

void startBelt(const char* why) {
  estopLatched = false;
  beltRun = true;
  if (targetSpeedPct < MIN_SPEED_PCT) targetSpeedPct = DEFAULT_SPEED_PCT;
  lastReason = why;
  Serial.printf("[START] %s speed=%.0f%%\n", why, targetSpeedPct);
  publishEvent("start", why);
}

// ---------------------------------------------------------------------
//  Buttons (active HIGH, debounced, edge on press)
// ---------------------------------------------------------------------
bool buttonPressedEdge(Button &b) {
  bool raw = digitalRead(b.pin) == HIGH;
  uint32_t now = millis();
  if (raw != b.lastRead) { b.lastRead = raw; b.changedAt = now; }
  if ((now - b.changedAt) > DEBOUNCE_MS && raw != b.stable) {
    b.stable = raw;
    return raw;                       // true only on the press edge
  }
  return false;
}

void handleButtons() {
  if (buttonPressedEdge(btnEstop)) emergencyStop("emergency button");
  if (buttonPressedEdge(btnStart)) startBelt("start button");
}

// ---------------------------------------------------------------------
//  MQTT
// ---------------------------------------------------------------------
void onMqttMessage(char* topic, byte* payload, unsigned int len) {
  if (String(topic) != TOPIC_COMMAND) return;
  JsonDocument doc;
  if (deserializeJson(doc, payload, len)) { Serial.println("[CMD] bad json"); return; }

  if (doc["speed_pct"].is<float>()) {
    float s = doc["speed_pct"].as<float>();
    targetSpeedPct = constrain(s, 0.0f, 100.0f);
    if (targetSpeedPct > 0 && targetSpeedPct < MIN_SPEED_PCT) targetSpeedPct = MIN_SPEED_PCT;
  }
  if (doc["tare"] | false) { if (okScale) scale.tare(); }

  if (doc["run"].is<bool>()) {
    bool wantRun = doc["run"].as<bool>();
    if (wantRun && estopLatched) {
      // the physical e-stop is only released with the physical START button
      publishEvent("cmd_rejected", "estop latched - press START on the machine");
    } else {
      beltRun = wantRun;
    }
  }
  lastReason = doc["reason"] | "remote";
  Serial.printf("[CMD] run=%d speed=%.0f%% (%s)\n", beltRun, targetSpeedPct, lastReason.c_str());
}

void wifiEnsure() {
  if (WiFi.status() == WL_CONNECTED) return;
  if (millis() - lastWifiTry < 5000 && lastWifiTry != 0) return;
  lastWifiTry = millis();
  Serial.printf("[WiFi] connecting to %s\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void mqttEnsure() {
  if (WiFi.status() != WL_CONNECTED || mqtt.connected()) return;
  if (millis() - lastMqttTry < 3000) return;
  lastMqttTry = millis();
  Serial.printf("[MQTT] connecting to %s:%u\n", MQTT_BROKER, MQTT_PORT);
  bool ok = (strlen(MQTT_USER) > 0)
    ? mqtt.connect(DEVICE_ID, MQTT_USER, MQTT_PASS, TOPIC_STATUS, 1, true, "offline")
    : mqtt.connect(DEVICE_ID, nullptr, nullptr, TOPIC_STATUS, 1, true, "offline");
  if (ok) {
    Serial.println("[MQTT] connected");
    mqtt.publish(TOPIC_STATUS, "online", true);
    mqtt.subscribe(TOPIC_COMMAND, 1);
    publishEvent("boot", "node online");
  } else {
    Serial.printf("[MQTT] failed rc=%d\n", mqtt.state());
  }
}

// ---------------------------------------------------------------------
//  Sensors
// ---------------------------------------------------------------------
void sampleVibration() {
  if (!okMpu) return;
  uint32_t now = millis();
  if (now - lastVibSample < VIB_SAMPLE_PERIOD_MS) return;
  lastVibSample = now;
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  ax = a.acceleration.x / 9.80665f;
  ay = a.acceleration.y / 9.80665f;
  az = a.acceleration.z / 9.80665f;
  float mag = sqrtf(ax * ax + ay * ay + az * az);
  vibSum += mag; vibSumSq += (double)mag * mag; vibN++;
  float dev = fabsf(mag - 1.0f);                 // deviation from 1 g (gravity)
  if (dev > vibMax) vibMax = dev;
}

void finishVibrationWindow() {
  if (vibN > 3) {
    double mean = vibSum / vibN;
    double var = vibSumSq / vibN - mean * mean;
    vibRms = var > 0 ? sqrt(var) : 0;            // AC part of acceleration magnitude, in g
    vibPeak = vibMax;
  }
  vibSum = vibSumSq = 0; vibN = 0; vibMax = 0;
}

void readSlowSensors() {
  if (okIna) {
    busV      = ina219.getBusVoltage_V();
    currentMA = ina219.getCurrent_mA();
    powerMW   = ina219.getPower_mW();
  }
  if (okScale && scale.is_ready()) {
    float g = scale.get_units(1);
    loadG = LOADCELL_INVERT ? -g : g;
  }
#if TEMP_SENSOR_TYPE == TEMP_DS18B20
  tempSensor.requestTemperatures();               // 12-bit: ~750ms; see setWaitForConversion(false)
  float t = tempSensor.getTempCByIndex(0);
  // DS18B20 returns 85.0C on power-on reset before conversion completes, and -127.0C if disconnected.
  if (t > -55.0f && t <= 125.0f && t != 85.0f) tempC = t;
#else
  uint32_t mv = 0;
  for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(PIN_TEMP);
  float volts = (mv / 8.0f) / 1000.0f;
  tempC = (volts - TEMP_ANALOG_OFFSET_V) / TEMP_ANALOG_V_PER_C;
#endif
}

void localSafetyCheck() {
  if (!isnan(tempC) && tempC > HARD_MAX_TEMP_C)  emergencyStop("hard limit: temperature");
  if (okIna && currentMA > HARD_MAX_CURRENT_MA)  emergencyStop("hard limit: current");
  if (okScale && loadG > HARD_MAX_LOAD_G)        emergencyStop("hard limit: load");
}

void publishTelemetry() {
  if (!mqtt.connected()) return;
  JsonDocument doc;
  doc["device"]    = DEVICE_ID;
  doc["ts"]        = millis();
  doc["load_g"]    = serialized(String(loadG, 1));
  doc["bus_v"]     = serialized(String(busV, 2));
  doc["current_ma"]= serialized(String(currentMA, 1));
  doc["power_mw"]  = serialized(String(powerMW, 1));
  if (!isnan(tempC)) doc["temp_c"] = serialized(String(tempC, 2));
  doc["vib_rms_g"] = serialized(String(vibRms, 4));
  doc["vib_peak_g"]= serialized(String(vibPeak, 4));
  doc["ax"] = serialized(String(ax, 3));
  doc["ay"] = serialized(String(ay, 3));
  doc["az"] = serialized(String(az, 3));
  doc["speed_pct"] = serialized(String(targetSpeedPct, 1));
  doc["step_delay_us"] = (int)currentDelayUs;
  doc["run"]       = beltRun && !estopLatched;
  doc["estop"]     = estopLatched;
  doc["rssi"]      = WiFi.RSSI();
  char buf[512];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  mqtt.publish(TOPIC_TELEMETRY, (const uint8_t*)buf, n, false);
}

// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Smart Conveyor ESP32 node ===");

  pinMode(PIN_DIR, OUTPUT); digitalWrite(PIN_DIR, DIR_FORWARD_LEVEL);
  pinMode(PIN_STEP, OUTPUT);
  if (PIN_LED >= 0) pinMode(PIN_LED, OUTPUT);
  // Buttons are wired to 3.3V -> need a pull-DOWN. GPIO34-39 have no internal
  // pull-down: add an external 10k resistor to GND if you move a button there.
  pinMode(PIN_BTN_START, INPUT_PULLDOWN);
  pinMode(PIN_BTN_ESTOP, INPUT_PULLDOWN);
  stepperTimerInit();
  stepperSetDelay(0);

  Wire.begin(PIN_INA_SDA, PIN_INA_SCL);
  Wire.setClock(400000);
  Wire1.begin(PIN_MPU_SDA, PIN_MPU_SCL);
  Wire1.setClock(400000);

  okIna = ina219.begin(&Wire);
  Serial.printf("INA219 : %s\n", okIna ? "OK" : "NOT FOUND");

  okMpu = mpu.begin(MPU6050_ADDR, &Wire1);
  if (okMpu) {
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);                // keep vibration content
  }
  Serial.printf("MPU6050: %s\n", okMpu ? "OK" : "NOT FOUND");

  scale.begin(PIN_HX711_DT, PIN_HX711_SCK);
  okScale = scale.wait_ready_timeout(1500);
  if (okScale) { scale.set_scale(LOADCELL_CALIBRATION); scale.tare(10); }
  Serial.printf("HX711  : %s\n", okScale ? "OK (tared - keep belt EMPTY at boot)" : "NOT FOUND");

#if TEMP_SENSOR_TYPE == TEMP_DS18B20
  tempSensor.begin();
  tempSensor.setWaitForConversion(false);                       // non-blocking reads
  tempSensor.setResolution(10);
#else
  analogSetPinAttenuation(PIN_TEMP, ADC_11db);
#endif

  topicCmdStr = TOPIC_COMMAND;
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);
  mqtt.setBufferSize(768);
  mqtt.setKeepAlive(15);
  lastRampMs = millis();
}

void loop() {
  wifiEnsure();
  mqttEnsure();
  mqtt.loop();

  handleButtons();
  sampleVibration();
  motorUpdate();

  uint32_t now = millis();
  if (now - lastPublish >= PUBLISH_INTERVAL_MS) {
    lastPublish = now;
    finishVibrationWindow();
    readSlowSensors();
    localSafetyCheck();
    publishTelemetry();
    if (PIN_LED >= 0) digitalWrite(PIN_LED, (beltRun && !estopLatched) ? HIGH : (estopLatched ? (now / 150) % 2 : LOW));
  }
}
