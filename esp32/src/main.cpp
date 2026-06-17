// =============================================================================
// main.cpp  —  ESP32 4-axis robot-arm controller (Phase 1)
//
//   WiFi + MQTT (PubSubClient)  ->  setpoints
//   50 Hz rate-limited control loop (local, real-time)
//   LEDC PWM via ESP32Servo      ->  4x MG996R
//   Command watchdog             ->  hold then relax on link loss
//   Serial jog mode              ->  per-servo limit calibration
//
// Commands (MQTT):
//   arm/cmd/joints   "b s e w"     absolute target angles (deg)
//   arm/cmd/pantilt  "dpan dtilt"  relative nudge of base + wrist (deg)
//   arm/cmd/mode     "HOME"|"RELAX"
//
// Serial jog (115200, newline-terminated), for Phase 1 calibration:
//   a0|a1|a2|a3   select axis
//   + / -         nudge selected axis by JOG_STEP deg (ignores soft limits' role
//                 only for finding mechanical safe range — still constrained 0..180)
//   h             home    r  relax    p  print angles
// =============================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include "config.h"

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);
Servo        servo[N_AXES];

float    cur[N_AXES];          // current (commanded) angle, deg
float    tgt[N_AXES];          // target angle, deg
bool     attached = false;     // are servos energized?
uint32_t tCtrl = 0, tState = 0, tLastCmd = 0;
int      jogAxis = 0;
const float JOG_STEP = 2.0f;

// ---- helpers ----------------------------------------------------------------
static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

void attachAll() {
  for (int i = 0; i < N_AXES; i++) {
    servo[i].setPeriodHertz(50);
    servo[i].attach(PIN[i], US_MIN[i], US_MAX[i]);
  }
  attached = true;
}

void detachAll() {
  for (int i = 0; i < N_AXES; i++) servo[i].detach();
  attached = false;
}

void setTargets(float b, float s, float e, float w) {
  if (!attached) attachAll();
  tgt[0] = clampf(b, MIN_DEG[0], MAX_DEG[0]);
  tgt[1] = clampf(s, MIN_DEG[1], MAX_DEG[1]);
  tgt[2] = clampf(e, MIN_DEG[2], MAX_DEG[2]);
  tgt[3] = clampf(w, MIN_DEG[3], MAX_DEG[3]);
}

void nudgePanTilt(float dpan, float dtilt) {
  if (!attached) attachAll();
  tgt[0] = clampf(tgt[0] + dpan,  MIN_DEG[0], MAX_DEG[0]);
  tgt[3] = clampf(tgt[3] + dtilt, MIN_DEG[3], MAX_DEG[3]);
}

void goHome()  { setTargets(HOME[0], HOME[1], HOME[2], HOME[3]); }
void relax()   { detachAll(); if (mqtt.connected()) mqtt.publish(TOPIC_STATUS, "relaxed"); }

// parse up to 4 space-separated floats from a buffer; returns count parsed
int parseFloats(const char* s, float* out, int maxN) {
  int n = 0; char buf[96];
  strncpy(buf, s, sizeof(buf) - 1); buf[sizeof(buf) - 1] = 0;
  char* tok = strtok(buf, " \t");
  while (tok && n < maxN) { out[n++] = atof(tok); tok = strtok(nullptr, " \t"); }
  return n;
}

// ---- MQTT -------------------------------------------------------------------
void onMqtt(char* topic, byte* payload, unsigned int len) {
  char msg[96];
  unsigned int n = len < sizeof(msg) - 1 ? len : sizeof(msg) - 1;
  memcpy(msg, payload, n); msg[n] = 0;
  tLastCmd = millis();

  if (!strcmp(topic, TOPIC_JOINTS)) {
    float v[4];
    if (parseFloats(msg, v, 4) == 4) setTargets(v[0], v[1], v[2], v[3]);
  } else if (!strcmp(topic, TOPIC_PANTILT)) {
    float v[2];
    if (parseFloats(msg, v, 2) == 2) nudgePanTilt(v[0], v[1]);
  } else if (!strcmp(topic, TOPIC_MODE)) {
    if (!strncmp(msg, "HOME", 4))       goHome();
    else if (!strncmp(msg, "RELAX", 5)) relax();
  }
}

void ensureLink() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    return;  // try again next loop; don't block control
  }
  if (!mqtt.connected()) {
    if (mqtt.connect(MQTT_CLIENTID)) {
      mqtt.subscribe(TOPIC_CMD_SUB);
      mqtt.publish(TOPIC_STATUS, "ok");
    }
  }
}

// ---- 50 Hz control loop -----------------------------------------------------
void controlStep() {
  uint32_t now = millis();
  if (now - tCtrl < CONTROL_PERIOD_MS) return;
  tCtrl = now;

  // watchdog: link/command loss -> hold, then relax
  uint32_t silent = now - tLastCmd;
  if (silent > RELAX_TIMEOUT_MS) { if (attached) relax(); return; }
  if (silent > CMD_TIMEOUT_MS)   { /* hold: leave tgt as-is, keep position */ }

  if (!attached) return;
  float dt = CONTROL_PERIOD_MS / 1000.0f;
  for (int i = 0; i < N_AXES; i++) {
    float step = MAX_DPS[i] * dt;                 // max move this tick
    cur[i] += clampf(tgt[i] - cur[i], -step, step);
    servo[i].write((int)lroundf(cur[i]));
  }
}

void publishState() {
  uint32_t now = millis();
  if (now - tState < STATE_PUB_MS || !mqtt.connected()) return;
  tState = now;
  char buf[48];
  snprintf(buf, sizeof(buf), "%.1f %.1f %.1f %.1f", cur[0], cur[1], cur[2], cur[3]);
  mqtt.publish(TOPIC_STATE, buf);
}

// ---- Serial jog (calibration) -----------------------------------------------
void serviceSerial() {
  if (!Serial.available()) return;
  String ln = Serial.readStringUntil('\n');
  ln.trim();
  if (ln.length() == 0) return;
  tLastCmd = millis();
  char c = ln[0];
  if      (c == 'a' && ln.length() > 1) jogAxis = constrain(ln[1] - '0', 0, N_AXES - 1);
  else if (c == '+') { if (!attached) attachAll(); tgt[jogAxis] = clampf(tgt[jogAxis] + JOG_STEP, 0, 180); }
  else if (c == '-') { if (!attached) attachAll(); tgt[jogAxis] = clampf(tgt[jogAxis] - JOG_STEP, 0, 180); }
  else if (c == 'h') goHome();
  else if (c == 'r') relax();
  Serial.printf("[jog] axis=%d  tgt: %.1f %.1f %.1f %.1f  %s\n",
                jogAxis, tgt[0], tgt[1], tgt[2], tgt[3], attached ? "ON" : "RELAXED");
}

// ---- setup / loop -----------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  for (int i = 0; i < N_AXES; i++) { cur[i] = tgt[i] = HOME[i]; }
  attachAll();
  for (int i = 0; i < N_AXES; i++) servo[i].write((int)lroundf(HOME[i]));

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(onMqtt);
  tLastCmd = millis();
  Serial.println("[boot] arm-esp32 ready. Serial jog: a0..a3 + - h r p");
}

void loop() {
  ensureLink();
  mqtt.loop();
  serviceSerial();
  controlStep();
  publishState();
}
