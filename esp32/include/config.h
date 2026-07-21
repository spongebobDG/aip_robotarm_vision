// =============================================================================
// config.h — ESP32 robot-arm controller configuration
// =============================================================================
#pragma once

// Keep real network values in ignored secrets.h.
// Copy secrets.h.example to secrets.h and edit it for the local network.
#if __has_include("secrets.h")
#include "secrets.h"
#else
#define WIFI_SSID     "CHANGE_ME"
#define WIFI_PASS     "CHANGE_ME"
#define MQTT_BROKER   "192.0.2.10"
#endif

#define MQTT_PORT     1883
#define MQTT_CLIENTID "arm-esp32"

// ---- MQTT topics -------------------------------------------------------------
#define TOPIC_CMD_SUB  "arm/cmd/#"
#define TOPIC_JOINTS   "arm/cmd/joints"
#define TOPIC_PANTILT  "arm/cmd/pantilt"
#define TOPIC_MODE     "arm/cmd/mode"
#define TOPIC_STATE    "arm/state"
#define TOPIC_STATUS   "arm/status"

// ---- Axis / servo mapping ----------------------------------------------------
// Index: 0=Base(pan) 1=Shoulder 2=Elbow 3=Wrist(tilt)
#define N_AXES 4
const int   PIN[N_AXES]     = {13, 12, 14, 27};
const float HOME[N_AXES]    = {90.0f, 0.0f, 0.0f, 90.0f};
const float MIN_DEG[N_AXES] = {0.0f, 0.0f, 0.0f, 0.0f};
const float MAX_DEG[N_AXES] = {180.0f, 120.0f, 140.0f, 180.0f};
const float MAX_DPS[N_AXES] = {60.0f, 45.0f, 45.0f, 60.0f};

// Per-servo pulse calibration (microseconds).
const int US_MIN[N_AXES] = {500, 500, 500, 500};
const int US_MAX[N_AXES] = {2500, 2500, 2500, 2500};

// ---- Control / safety timing -------------------------------------------------
#define CONTROL_PERIOD_MS 20
#define STATE_PUB_MS      200
#define CMD_TIMEOUT_MS    1500
#define RELAX_TIMEOUT_MS  8000
