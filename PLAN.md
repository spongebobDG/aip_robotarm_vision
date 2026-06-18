# 산업용 감시 로봇 (Pi4 + ESP32 WiFi/MQTT 분산제어, RGB/열화상 비전) — 단계별 개발 계획

## Context (왜 이 작업을 하는가)
하드웨어 조립은 끝났지만 소프트웨어는 0에서 시작한다. 목표는 **배선/핀 설정부터 비전 AI 추적까지 한 단계씩 검증하며 올라가는** 산업용 감시 로봇이다. 핵심 요구는 "동작하는 코드 + 명확한 HW 매핑"이며, 각 Phase는 독립 테스트/완료 가능한 마일스톤이어야 한다.

### 아키텍처 (사용자 확정 반영) — Pi4 + ESP32 무선(WiFi/MQTT) 분산제어
- **Raspberry Pi 4 (고수준):** CSI 카메라, 비전·퓨전·AI·FSM, **Mosquitto MQTT 브로커 호스팅**, 열화상 프레임 시리얼 수신, [Linux] ubuntu 22.04 LTS server
- **ESP32 (실시간 저수준):** **WiFi 2.4GHz로 MQTT 구독**해 셋포인트 수신 → **50Hz 제어 루프** → **LEDC 하드웨어 PWM으로 4× MG996R 직접 구동**. 펌웨어 = Arduino(C++)
- **열화상 모듈:** **UART(VIN/GND/RX/TX)로 프레임 스트리밍** → Pi GPIO UART
> **이 구조의 핵심:** WiFi는 **저(低)레이트 셋포인트(조인트 목표·pan/tilt delta)만** 전송하고, **하드 리얼타임 50Hz 루프는 ESP32 로컬**에 둔다 → WiFi 지연/지터가 모션 매끄러움에 영향 없음. 무선이라 Pi-ESP32 간 공통 GND 불필요(전기적 분리).

### 통신 연결 정리 (혼동 방지)
| 링크 | 연결 방식 | 식별자 | 비고 |
|---|---|---|---|
| **Pi ↔ ESP32** | **WiFi 2.4GHz · MQTT** (브로커=Pi의 Mosquitto) | 토픽 `arm/#` | 런타임 무선. USB는 **펌웨어 플래싱 전용** |
| **Pi ↔ 열화상** | **GPIO UART** (GPIO15 RXD ← 모듈 TX) | `/dev/serial0` | 모듈은 TTL 시리얼 핀만 → GPIO UART |
| **Pi ↔ 카메라** | **CSI 리본** | libcamera | UART/GPIO 아님 |
| **ESP32 → 서보** | **GPIO PWM** (LEDC) | GPIO13/12/14/27 | 신호선 4개. 전원은 외부 PSU |
| **네트워크** | **기존 2.4GHz 공유기 AP** | — | Pi/ESP32 모두 접속, **Pi 고정 IP 권장**(ESP32가 브로커 주소로 사용) |

> ⚠️ **열화상 센서 사실 확인:** 맨 MLX90640 칩은 원래 **I2C 전용**이다. VIN/GND/RX/TX(UART)로 나온다면 모듈에 **변환 MCU가 내장**돼 프레임을 시리얼로 내보내는 보드다. 드라이버는 I2C 호출이 아니라 **시리얼 프레임 파서** → 모듈 데이터시트의 **baud + 프레임 포맷** 확인 후 확정(아래는 템플릿).

### MQTT 토픽 스키마
| 방향 | 토픽 | 페이로드 | QoS |
|---|---|---|---|
| Pi → ESP32 | `arm/cmd/joints` | `"b s e w"` (절대 각도°) | 0 |
| Pi → ESP32 | `arm/cmd/pantilt` | `"dpan dtilt"` (delta°) | 0 |
| Pi → ESP32 | `arm/cmd/mode` | `"HOME"` / `"RELAX"` | 1 |
| ESP32 → Pi | `arm/state` | `"b s e w"` (현재각, 텔레메트리) | 0 |
| ESP32 → Pi | `arm/status` | `"ok"` / `"estop"` / `"linklost"` | 1 |

### 축 정의 (4-DOF, MG996R 180° 서보)
| 축 | 이름 | 운동 | ESP32 GPIO | 홈 각도 | 용도 |
|----|------|------|------|------|------|
| J0 | Base | 수직축 회전 = **Pan** | GPIO13 | 90° | 좌우 추적 |
| J1 | Shoulder | 수평축 회전(들어올림) | GPIO12 | 0° | 자세/높이(coarse) |
| J2 | Elbow | 수평축 회전(전완) | GPIO14 | 0° | 자세/리치(coarse) |
| J3 | Wrist | 카메라 헤드 **Tilt** | GPIO27 | 90° | 상하 추적 |
> GPIO13/12/14/27 = PWM 가능·안전 핀. 스트래핑 핀(0/2/12/15)·입력전용(34~39)은 서보 출력 금지(대체: 26/25/33/32).

> **설계 핵심:** 감시 추적은 풀 XYZ IK 불필요. **J1/J2는 "감시 자세(watch posture)"로 고정**, **빠른 추적은 J0(Pan)+J3(Tilt) 2-DOF**(이미지 기반 비주얼 서보잉). 풀 IK는 특정 3D 좌표 지향 시에만.

---

## 전체 데이터 흐름도 (Data Flow Diagram)
```
 ┌───────────────── Raspberry Pi 4 (Python + Mosquitto broker) ───────────────┐
 │  Pi Wide Cam ──CSI──► [Camera Capture] ─┐                                   │
 │                                          ├─shm─► [Fusion + AI] ─events─► [FSM] │
 │  Thermal mod ─UART──► [Thermal Parse] ──┘        upscale·MediaPipe      │   │
 │  (/dev/serial0)                                                          │   │
 │                              MQTT broker (localhost:1883) ◄──publish/sub─┘   │
 └────────────────────────────────────│──────────────────────────────────────┘
                              WiFi 2.4GHz · MQTT  (arm/cmd/*  ▲ arm/state)
                              기존 공유기 AP        joint goals / pan-tilt delta
                                                                            ▼
                              ┌──────────── ESP32 (Arduino C++) ────────────┐
                              │ WiFi+MQTT sub ─► [50Hz Control Loop] ─► LEDC │
                              │ (cmd watchdog)    rate-limited interp    PWM │
                              └──────────────────────────────────────────│──┘
                                                                          ▼
                Ext 5~6V PSU ──buck 5V──► ESP32 VIN                4× MG996R
                            └────────────V+───────────────────────► (GPIO13/12/14/27)
                            (공통 GND: PSU ↔ ESP32, Pi는 무선 분리)
```
**핵심 원칙:** 비전(Pi)·MQTT(WiFi)·50Hz제어(ESP32) 3계층이 분리되어 어느 한쪽 지연도 50Hz 모션에 영향 없음. Pi 내부도 multiprocessing + latest-frame-wins로 격리.

---

## 프로젝트 디렉터리 구조
```
aip_robotarm_vision/
├── pi/                              # Raspberry Pi 4 (Python)
│   ├── config/{arm_config.yaml, camera_config.yaml, fusion_calib.yaml, mqtt_config.yaml}
│   ├── comms/
│   │   ├── mqtt_link.py             # Pi↔ESP32 MQTT (paho), 토픽 pub/sub
│   │   └── thermal_serial.py        # 열화상 모듈 시리얼 파서 (/dev/serial0)
│   ├── kinematics/{arm_model.py(FK), ik_solver.py(2-link IK + pan/tilt)}
│   ├── vision/{camera_capture.py, fusion.py, detector.py, thermal_analysis.py}
│   ├── app/{state_machine.py, tracker.py, main.py(멀티프로세스)}
│   └── tools/fusion_calibrate.py
└── esp32/                           # ESP32 펌웨어 (Arduino / PlatformIO)
    ├── src/main.cpp                 # WiFi+MQTT + LEDC 서보 + 50Hz 루프 + 워치독
    ├── include/config.h             # SSID/브로커IP/핀/min·max us/한계/홈
    └── platformio.ini
```

## 의존성
**Pi (Raspberry Pi OS Bookworm):**
```bash
sudo apt install -y python3-picamera2 python3-serial mosquitto mosquitto-clients
sudo systemctl enable --now mosquitto             # 로컬 MQTT 브로커
sudo raspi-config   # Interface > Serial Port: 콘솔 OFF / 하드웨어 ON  (열화상 /dev/serial0)
pip install opencv-python mediapipe numpy pyyaml pyserial paho-mqtt gpiozero
# Pi 고정 IP 권장(ESP32가 브로커 주소로 사용)
```
**ESP32 (Arduino/PlatformIO):** `WiFi`, `PubSubClient`(MQTT), `ESP32Servo`. 보드: esp32dev.

---

# Phase 1 — 하드웨어 브링업 (ESP32 서보 + WiFi/MQTT 링크)

### [개발 목표]
- ESP32 Arduino: LEDC PWM로 4× MG996R 구동, 축별 **안전 각도 한계 캘리브레이션**, 홈(90/0/0/90), relax(detach)
- **WiFi 접속 + MQTT 구독/발행**: Pi에서 `arm/cmd/mode HOME` 발행 → ESP32 동작, `arm/state` 텔레메트리 수신
- **명령 워치독**: 셋포인트 끊기면(WiFi drop) 마지막 안전자세 유지 후 타임아웃 시 relax
- 전원/접지: 외부 5~6V PSU(서보) + **5V 벅으로 ESP32 급전**, 공통 GND, 벌크 캡

### [핀 맵 및 HW 연결 가이드]
**ESP32 → 4× MG996R**
| 서보 | 신호 → ESP32 | V+(빨강) | GND(갈색) |
|---|---|---|---|
| J0 Base | GPIO13 | 외부 PSU +5~6V | 공통 GND |
| J1 Shoulder | GPIO12 | 〃 | 〃 |
| J2 Elbow | GPIO14 | 〃 | 〃 |
| J3 Wrist | GPIO27 | 〃 | 〃 |

**전원:** MG996R 스톨 ≈2.5A/개 ×4 → **5~6V, ≥6A(여유 10A) 외부 PSU**. **ESP32는 USB 미연결 운용 → PSU에서 5V 벅 레귤레이터로 ESP32 VIN(5V) 급전**(또는 3.3V→3V3). USB는 **플래싱 시에만**.
**공통 GND:** PSU GND ↔ ESP32 GND ↔ 서보 GND (단일점). Pi는 무선이라 분리.
**벅 레귤레이터:** 서보 전류 변동이 ESP32에 영향 안 가도록 ESP32 급전은 **별도 벅 출력** 권장(서보 V+ 레일과 분리, GND만 공통).
**벌크 캡:** 서보 V+/GND에 1000µF(≥10V), 서보별 100nF.
**레벨시프터:** ESP32 GPIO 3.3V → MG996R 신호 임계 마진 빠듯 → **3.3→5V 레벨시프터 권장**.

### [HW 결선 주의점]
- **서보 전원을 ESP32 핀에서 끌면 안 됨** → 별도 PSU. ESP32 급전도 서보 레일과 분리(벅).
- **공통 GND 필수**(PSU·ESP32·서보). WiFi라 Pi는 제외.
- 서보 출력에 **스트래핑 핀(0/2/15) 금지**, 입력전용(34~39) PWM 불가.
- LEDC 50Hz, 16-bit로 500~2500µs 매핑. 첫 인가 시 한 축씩.
- idle 시 `detach()`로 relax → 발열/험/전류 저감.

### [핵심 SW 구조 / 의사코드]
`esp32/include/config.h`
```cpp
const char* SSID="..."; const char* PASS="..."; const char* BROKER="192.168.0.x"; // Pi 고정 IP
const int  PIN[4]={13,12,14,27};
const float HOME[4]={90,0,0,90}, MINDEG[4]={0,0,0,0}, MAXDEG[4]={180,120,140,180}, MAXDPS[4]={60,45,45,60};
const int  US_MIN=500, US_MAX=2500;            // 서보별 캘리브레이션 후 조정
const uint32_t CMD_TIMEOUT=1500;               // ms, 워치독
```
`esp32/src/main.cpp`
```cpp
#include <WiFi.h>; #include <PubSubClient.h>; #include <ESP32Servo.h>
WiFiClient wifi; PubSubClient mqtt(wifi); Servo sv[4];
float cur[4], tgt[4]; uint32_t tCtrl=0, tLastCmd=0;
void onMsg(char* topic, byte* p, unsigned n){
  String t=topic, s=String((char*)p).substring(0,n);
  if(t=="arm/cmd/joints")  parse4 -> tgt[i]=constrain(v, MINDEG[i], MAXDEG[i]);
  if(t=="arm/cmd/pantilt") parse2 -> tgt[0]=clamp(tgt[0]+dp,..); tgt[3]=clamp(tgt[3]+dt,..);
  if(t=="arm/cmd/mode"){ if(s=="HOME") for(i)tgt[i]=HOME[i]; if(s=="RELAX") relaxAll(); }
  tLastCmd=millis();
}
void ensureLink(){ if(WiFi.status()!=WL_CONNECTED){WiFi.begin(SSID,PASS);}
  if(!mqtt.connected()){ if(mqtt.connect("arm-esp32")) mqtt.subscribe("arm/cmd/#"); } }
void controlStep(){ if(millis()-tCtrl<20) return; tCtrl=millis();
  if(millis()-tLastCmd>CMD_TIMEOUT) holdSafe();             // 워치독: 정지 유지(→타임아웃 더 길면 relax)
  for(int i=0;i<4;i++){ float step=MAXDPS[i]*0.02f;
    cur[i]+=constrain(tgt[i]-cur[i],-step,step); sv[i].write(cur[i]); } }
void setup(){ for(i){sv[i].setPeriodHertz(50); sv[i].attach(PIN[i],US_MIN,US_MAX); cur[i]=tgt[i]=HOME[i]; sv[i].write(HOME[i]);}
  mqtt.setServer(BROKER,1883); mqtt.setCallback(onMsg); }
void loop(){ ensureLink(); mqtt.loop(); controlStep();
  every 200ms: mqtt.publish("arm/state", "b s e w"); }
```
`pi/comms/mqtt_link.py`
```python
import paho.mqtt.client as mqtt
cli = mqtt.Client("pi-host"); cli.connect("localhost", 1883); cli.loop_start()
def send_joints(b,s,e,w): cli.publish("arm/cmd/joints", f"{b:.1f} {s:.1f} {e:.1f} {w:.1f}", qos=0)
def pan_tilt(dp,dt):      cli.publish("arm/cmd/pantilt", f"{dp:.2f} {dt:.2f}", qos=0)
def home():  cli.publish("arm/cmd/mode", "HOME", qos=1)
def relax(): cli.publish("arm/cmd/mode", "RELAX", qos=1)
# cli.subscribe("arm/state"); on_message -> 현재각 텔레메트리
```
캘리브레이션: 플래싱 후 시리얼 모니터로 축 선택→±1° 조그→안전 min/max(US) 기록→`config.h`.

### [검증 방법]
- ESP32 시리얼 모니터: WiFi 접속·IP 획득, 브로커 연결, 각 축 min↔max 스윕/홈/relax 동작
- 동시 구동 시 PSU·ESP32 브라운아웃 없음(벅 분리 확인)
- Pi: `mosquitto_pub -t arm/cmd/mode -m HOME` 또는 `mqtt_link.home()` → 암 홈 복귀
- `mosquitto_sub -t arm/state` → 텔레메트리 수신. WiFi 끊으면 워치독으로 안전정지
- **✅ Milestone 1:** Pi가 MQTT로 명령, ESP32가 4축을 한계 내 구동 + 워치독 동작

---

# Phase 2 — 기구학 & 제어 (Pi/ESP32 역할 분담)

### [개발 목표]
- **Pi:** FK/IK(2-link planar + pan/tilt), 고수준 목표 생성 → MQTT 발행
- **ESP32:** 50Hz 속도제한 보간 + LEDC PWM (Phase 1 펌웨어 확장: 가감속/한계)
- 명령 프로토콜 확정(절대 `arm/cmd/joints` vs delta `arm/cmd/pantilt`), watch-posture 모드

### [핀 맵 및 HW 연결 가이드]
- **신규 배선 없음** — Phase 1 재사용.

### [핵심 SW 구조 / 의사코드]
링크 파라미터(실측): `L1`(shoulder→elbow), `L2`(elbow→wrist), `H`(베이스 높이).
`pi/kinematics/ik_solver.py`
```python
def ik(x, y, z, cam_pitch):
    pan=atan2(y,x); r=hypot(x,y); zp=z-H
    c2=(r*r+zp*zp-L1*L1-L2*L2)/(2*L1*L2)
    if abs(c2)>1: raise Unreachable
    th2=acos(c2)                                        # elbow
    th1=atan2(zp,r) - atan2(L2*sin(th2), L1+L2*cos(th2))
    th3=cam_pitch-(th1+th2)                             # 카메라 목표 pitch 유지
    return clamp_limits(pan, th1, th2, th3)
```
역할 분담:
- Pi: `angles=ik(x,y,z,pitch)` → `mqtt_link.send_joints(*angles)`
- Pi 추적: `mqtt_link.pan_tilt(dp,dt)` → ESP32가 `tgt[base]+=dp; tgt[wrist]+=dt`(clamp) 누적
- ESP32: 들어온 목표를 50Hz로 **속도제한 보간**(모션 매끄러움·한계 보장 책임)
> **실시간 경계:** "무엇을 겨냥"(Pi, MQTT, 느림) ↔ "어떻게 이동"(ESP32, 50Hz·로컬). WiFi가 느리거나 끊겨도 ESP32가 매끄러움·안전(워치독) 책임.

### [검증 방법]
- FK 라운드트립: 알려진 각도 명령→자로 말단 측정→FK 비교(수 cm/도). 아날로그 서보 정밀도 한계 인지
- IK: 도달 가능 (x,y,z) 도달, 불가능 graceful reject
- `pan_tilt` delta가 카메라를 예측 가능하게 이동(Phase 4 토대)
- WiFi 지연/순간 끊김 중에도 모션 매끄럽고 안전(워치독)
- **✅ Milestone 2:** `move_to_xyz()`(Pi→MQTT→ESP32) + `pan_tilt()` 부드럽게 동작

---

# Phase 3 — 비전 시스템 & 열화상(UART) 퓨전

### [개발 목표]
- 카메라 CSI 스트리밍(Picamera2)
- **열화상 프레임 = 시리얼 파서**(모듈→Pi GPIO UART `/dev/serial0`) ← I2C 아님
- 32×24 업스케일 → 정합(warpAffine) → 오버레이
- multiprocessing 격리

### [핀 맵 및 HW 연결 가이드]
- **카메라 → CSI 리본**(libcamera/Picamera2)
- **열화상 모듈 → Pi GPIO UART:**
  - VIN → 모듈 사양(3.3V or 5V) / GND → Pi GND
  - **모듈 TX → Pi GPIO15 (RXD)** / (모듈 RX ← Pi GPIO14 TXD, 필요 시)
  - ⚠️ **모듈 TX가 5V면 Pi RX(3.3V)에 직결 금지** → 분압/레벨시프터로 강압
  - raspi-config: 시리얼 콘솔 OFF / 하드웨어 ON → `/dev/serial0`
- **포트 충돌 없음:** ESP32=WiFi(무선), 열화상=GPIO UART(`/dev/serial0`)

### [핵심 SW 구조 / 의사코드]
프로세스(multiprocessing, shared_memory + Queue, latest-frame-wins):
1. **Camera Capture**(Picamera2 640×480@30) 2. **Thermal Parse**(시리얼) 3. **Fusion+AI** 4. **FSM**(Phase 4) — 제어는 ESP32(무선)

`pi/comms/thermal_serial.py` (모듈 포맷에 맞춰 확정 — 템플릿)
```python
import serial, numpy as np
ser = serial.Serial('/dev/serial0', BAUD, timeout=1)
def read_frame():                       # -> np.ndarray(24,32) °C
    sync_to_start_marker(ser)
    raw = ser.read(EXPECTED_BYTES)      # 768값(uint16/float) + 체크섬
    verify_checksum(raw)
    return np.array(unpack(raw), dtype=float).reshape(24,32)
```
`pi/vision/fusion.py`
```python
def thermal_overlay(rgb, thermal_c, calib):
    t = cv2.applyColorMap(uint8(normalize(thermal_c, TMIN, TMAX)), cv2.COLORMAP_JET)
    t = cv2.resize(t, (W,H), interpolation=cv2.INTER_CUBIC)
    t = cv2.warpAffine(t, calib.M, (rgb.W, rgb.H))           # 수직 시차+FOV 정합
    return cv2.addWeighted(rgb, 1-α, t, α, 0)
```
`pi/tools/fusion_calibrate.py`: 가열체 다지점 → RGB↔열화상 대응점 클릭 → `cv2.estimateAffinePartial2D`로 `M` → `fusion_calib.yaml`. **카메라/열화상 상하 적층(카메라가 타겟에 더 가까움) → 수직 시차 지배적**, 운용 거리에서 캘리브레이션.

### [검증 방법]
- 열화상 프레임 정상 파싱(체크섬 통과, 24×32 °C)
- RGB ≈30fps, 오버레이에서 가열체가 RGB 위치에 수 px 이내 정합
- 비전 구동 중 ESP32 제어 50Hz 무영향
- **✅ Milestone 3:** 라이브 퓨전 스트림 + ESP32 제어 공존

---

# Phase 4 — 산업 감시 AI & 추적

### [개발 목표]
1. **침입자/사람·얼굴 탐지 → Pan/Tilt 추적**(타겟 중앙 정렬)
2. **열화상 고온 이상/화재 탐지 → 경보**
3. 인식 ↔ 구동을 잇는 **고수준 상태 머신(FSM)**

### [핀 맵 및 HW 연결 가이드]
- 신규 배선 없음. (옵션) 부저/릴레이 → Pi 또는 ESP32 여유 GPIO. 네트워크 경보는 MQTT `arm/alarm` 토픽 발행.

### [핵심 SW 구조 / 의사코드]
**탐지(Pi4 CPU):** 사람/얼굴 = **MediaPipe Face/Pose**, 일반 person = **MobileNet-SSD(OpenCV DNN)** 옵션. 침입자 = 제한구역/시간 person 이벤트.

`pi/app/tracker.py` (이미지 기반 비주얼 서보잉, J0 Pan + J3 Tilt → MQTT)
```python
ex=(cx-W/2)/W; ey=(cy-H/2)/H
if abs(ex)<DEAD and abs(ey)<DEAD: return                # 데드존(지터 방지)
dpan=-Kp_pan*ex; dtilt=+Kp_tilt*ey                       # 부호는 기구 방향
mqtt_link.pan_tilt(clamp(dpan,±MAX), clamp(dtilt,±MAX))
```
`pi/vision/thermal_analysis.py`
```python
mask = thermal_c > T_ALARM               # 예: 60°C (config)
if mask.any():
    region=largest_component(mask); severity=f(max_temp, area)
    emit FIRE_ALARM(coords→calib로 RGB 매핑, severity)   # MQTT arm/alarm + 부저
```
`pi/app/state_machine.py` (이벤트 구동, 우선순위 **FIRE > INTRUDER > PATROL**)
```python
states: PATROL, TRACKING, ALARM_FIRE, RETURN_HOME
on perception_result:
    if fire_event:                    state=ALARM_FIRE
    elif PATROL and person_detected:  state=TRACKING
    elif TRACKING:
        if target_lost>timeout:       state=PATROL
        else:                         tracker.update()        # -> pan_tilt (MQTT→ESP32)
    if ALARM_FIRE: aim_at(hotspot); sustain alarm until cleared/ack
PATROL: J1/J2 watch posture 고정, J0 느린 스윕(send_joints)
```
> **통합:** FSM(느림·이벤트)이 `mqtt_link`로 발행, 인식은 Phase 3 프로세스, 50Hz 제어는 ESP32. 3계층 분리 유지.

### [검증 방법]
- 탐지 FPS(MediaPipe 목표 ~10~15fps on Pi4)
- 추적: FOV 가로질러 이동 시 암이 추종해 중앙 유지, 진동 없음(Kp·데드존 튜닝)
- 화재 테스트: 임계 초과 열원 → ALARM_FIRE, 카메라 지향, 부저/`arm/alarm` 발행, 제거 시 복귀
- FSM 전이 정확(fire가 tracking 오버라이드)
- **풀 시나리오:** PATROL→탐지→TRACK→열 주입→ALARM→해제→PATROL, ESP32 제어 무정지
- **✅ Milestone 4:** 엔드투엔드 자율 감시 데모

---

## 통합 검증 & 운용
- 각 Phase 독립 실행 스크립트로 검증(`pi/tools/`, 각 모듈 `__main__`, ESP32 시리얼 모니터, `mosquitto_sub`로 토픽 모니터).
- `pi/app/main.py`가 Pi 프로세스 기동/워치독, 종료 시 `arm/cmd/mode RELAX`+`HOME`.
- 안전: 비상 정지(키/GPIO/MQTT) → `RELAX`. SW 각도 한계는 Pi·ESP32 양쪽 이중 적용. **WiFi 끊김 = ESP32 워치독 안전정지**.

## ROS2(Humble) 이식 매핑 (향후)
| 현재 | ROS2 |
|---|---|
| MQTT `arm/cmd/*`,`arm/state` | `arm_driver` 노드 토픽(`/joint_goal`,`/pan_tilt_delta`,`/joint_states`). micro-ROS로 ESP32 직접 노드화도 가능 |
| `detector` | `perception` 노드 / pub `/detections` |
| `thermal_analysis` | pub `/thermal/alarm` |
| `state_machine` | `behavior` 노드 |
> MQTT 토픽 스키마를 ROS2 토픽과 1:1 대응되게 설계해 이식 비용 최소화.

## 리스크 / 주의
- **열화상 UART 프로토콜 미확정:** 모듈 데이터시트 baud·프레임 포맷 확인(맨 MLX90640은 I2C 전용임에 유의).
- **WiFi 신뢰성:** 2.4GHz 혼잡/끊김 → ESP32 **명령 워치독 필수**(안전정지). QoS·재연결 로직, Pi 고정 IP.
- **MG996R 토크 한계:** 하위축(J1/J2) 중력 부하 빠듯 → watch posture 보수적, 필요 시 카운터스프링.
- **ESP32 급전:** 서보 레일과 분리된 벅 출력으로 급전(서보 전류 변동 격리). 3.3V 신호 → 레벨시프터 권장.
- **아날로그 서보 정밀도/지터:** 절대 피드백 없음 → 기구학 정밀도 수도 단위, idle relax.
- **열화상-RGB 시차:** 거리 의존 → 운용 거리 캘리브레이션, 5V TX면 Pi RX 강압 필수.
```
