// =============================================================================
// config.h  —  ESP32 robot-arm controller configuration (Phase 1)
// 4x MG996R servos driven by ESP32 LEDC PWM, commanded over WiFi/MQTT.
//
// EDIT the WiFi / broker / per-servo calibration values for YOUR hardware.
// US_MIN/US_MAX are per-servo pulse widths — calibrate them with the serial
// jog mode (see main.cpp) and write the safe values back here.
// =============================================================================
#pragma once

// ---- Network -----------------------------------------------------------------
#define WIFI_SSID     "YOUR_WIFI_SSID"      // 2.4 GHz network
#define WIFI_PASS     "YOUR_WIFI_PASSWORD"
#define MQTT_BROKER   "192.168.0.50"        // Pi static IP (Mosquitto host)
#define MQTT_PORT     1883
#define MQTT_CLIENTID "arm-esp32"

// ---- MQTT topics -------------------------------------------------------------
#define TOPIC_CMD_SUB  "arm/cmd/#"          // subscribe to all commands
#define TOPIC_JOINTS   "arm/cmd/joints"     // payload: "b s e w"  (abs deg)
#define TOPIC_PANTILT  "arm/cmd/pantilt"    // payload: "dpan dtilt" (delta deg)
#define TOPIC_MODE     "arm/cmd/mode"       // payload: "HOME" | "RELAX"
#define TOPIC_STATE    "arm/state"          // pub: "b s e w" telemetry
#define TOPIC_STATUS   "arm/status"         // pub: "ok" | "linklost" | "relaxed"

// ---- Axis / servo mapping ----------------------------------------------------
// Index: 0=Base(pan) 1=Shoulder 2=Elbow 3=Wrist(tilt)
#define N_AXES 4
const int   PIN[N_AXES]    = { 13, 12, 14, 27 };       // PWM-safe GPIOs
const float HOME[N_AXES]   = { 90.0f, 0.0f, 0.0f, 90.0f };
const float MIN_DEG[N_AXES]= {  0.0f, 0.0f, 0.0f,  0.0f };
const float MAX_DEG[N_AXES]= {180.0f,120.0f,140.0f,180.0f };
const float MAX_DPS[N_AXES]= { 60.0f, 45.0f, 45.0f, 60.0f }; // max deg/sec per axis

// Per-servo pulse calibration (microseconds). MG996R typical 500..2500us = 180deg.
// Start conservative; refine per servo via serial jog mode.
const int   US_MIN[N_AXES] = { 500, 500, 500, 500 };
const int   US_MAX[N_AXES] = { 2500, 2500, 2500, 2500 };

// ---- Control / safety timing -------------------------------------------------
#define CONTROL_PERIOD_MS  20      // 50 Hz control loop
#define STATE_PUB_MS       200     // telemetry publish interval
#define CMD_TIMEOUT_MS     1500    // no command for this long -> hold (watchdog)
#define RELAX_TIMEOUT_MS   8000    // no command for this long -> detach (limp)
