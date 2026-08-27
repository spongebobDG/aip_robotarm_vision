# Industrial Robot Arm Vision

Raspberry Pi 4가 비전·기구학·상태 머신을 담당하고 ESP32가 4축 MG996R 서보를 50 Hz로 제어하는 분산 감시 로봇암 프로젝트입니다.

> A Pi–ESP32 distributed robot-arm prototype with RGB/thermal perception, MQTT setpoints, local 50 Hz motion control, and link-loss safety behavior.

| 구분 | 내용 |
|---|---|
| 구성 | 개인 프로젝트 |
| 실기기 | Raspberry Pi 4, ESP32, MG996R ×4, CSI RGB camera, UART thermal module |
| 대표 증거 | 5분 18초 조그, telemetry 1,581건, 8초 link-loss relax |
| 포트폴리오 | [ROBOTIS 지원 포트폴리오 요약](https://github.com/spongebobDG/robotics-software-portfolio/blob/main/projects/aip-robotarm-vision.md) |

## 60초 요약

| 구분 | 내용 |
|---|---|
| 고수준 컴퓨팅 | Raspberry Pi 4, Ubuntu 22.04, Python, OpenCV |
| 저수준 제어 | ESP32, Arduino C++, 50 Hz rate-limited servo interpolation |
| 통신 | Wi-Fi/MQTT는 저주기 setpoint, 모션 루프는 ESP32 로컬 수행 |
| 센서 | CSI RGB camera, UART thermal module |
| 안전 | 명령 1.5초 후 hold, 8초 후 servo relax |

## 검증된 결과

- 4축 조그 **5분 18초**, `arm/state` telemetry **1,581건**, 1초 이상 gap 0건
- 명령이 멈춘 3개 구간 모두 마지막 명령 후 **8초**에 자동 `relaxed`
- 4개 자세 FK→IK round-trip 오차 **0.00°**, 도달 불가능 목표 reject
- RGB 640×480 카메라 약 **23 FPS**
- RGB–열화상 affine calibration: scale 약 0.90, rotation 약 3.8°, translation +26.7/+17.5 px
- 공개 placeholder 설정으로 PlatformIO build 성공: RAM **13.8%**, flash **58.8%**

근거와 트러블슈팅 과정은 [`docs/devlog/`](docs/devlog/)에 기록했습니다.

## 아키텍처

```text
RGB camera ───────┐
                  ├─> Pi vision / fusion / FSM ─> MQTT setpoint ─> ESP32
Thermal UART ─────┘                                           │
                                                              └─> 50 Hz local control ─> 4× MG996R
```

Wi-Fi는 목표 관절각과 pan/tilt delta만 전달합니다. 50 Hz 보간, 각도 제한과 link-loss watchdog은 ESP32에서 수행해 네트워크 지연이 저수준 제어 주기에 직접 들어오지 않도록 분리했습니다.

## 구현 상태

| Phase | 구현 | 검증 상태 |
|---|---|---|
| 1. Bringup | ESP32 PWM, MQTT, telemetry, watchdog | 실기기 검증 완료 |
| 2. Kinematics | FK/IK, reachability, axis mapping | 수학·실측 비교 완료 |
| 3. Vision | RGB/thermal capture, parser, calibration, fusion | 실기기 calibration 완료 |
| 4. Surveillance | detection, tracking, FSM, alarm, orchestrator | 코드 구현, 장시간 통합 acceptance 미완료 |

기존 README가 Phase 1만 완료된 것처럼 보이던 문제를 수정했습니다. Phase 4는 코드가 존재하지만 전체 시나리오의 장시간 실기기 acceptance가 없으므로 “완료”로 표시하지 않습니다.

## 디렉터리

```text
aip_robotarm_vision/
├── esp32/
│   ├── include/        # safe config + ignored local secrets
│   └── src/main.cpp    # MQTT, 50 Hz interpolation, watchdog, serial jog
├── pi/
│   ├── app/            # surveillance FSM, tracker, orchestrator
│   ├── comms/          # MQTT and thermal serial parser
│   ├── kinematics/     # FK and IK
│   ├── tools/          # calibration and web preview tools
│   └── vision/         # RGB/thermal capture, detection, fusion
└── docs/devlog/        # measured evidence and troubleshooting
```

## Setup

### 1. ESP32 local secrets

실제 Wi-Fi와 broker 주소는 Git에 커밋하지 않습니다.

```bash
cd esp32/include
cp secrets.h.example secrets.h
# Edit secrets.h locally.
```

`secrets.h`는 `.gitignore`에 포함되어 있습니다. 파일이 없을 때는 문서용 placeholder가 사용되므로 실제 장비 연결 전에 반드시 로컬 값을 설정해야 합니다.

### 2. ESP32 firmware

```bash
cd esp32
pio run
pio run -t upload
pio device monitor -b 115200
```

### 3. Raspberry Pi

```bash
sudo apt install -y mosquitto mosquitto-clients python3-picamera2 python3-serial
sudo systemctl enable --now mosquitto
cd pi
pip install -r requirements.txt
python -m tools.kinematics_check
```

## 주요 설계 선택

### Pi와 ESP32 역할 분리

- Pi: 카메라·열화상 처리, FK/IK, 감시 FSM, 목표 생성
- ESP32: PWM, 속도 제한 보간, angle clamp, watchdog

### Headless calibration

Ubuntu Server에서 OpenCV GUI가 동작하지 않아 RGB와 열화상 점을 브라우저에서 선택하고 서버에서 affine을 계산하는 도구를 구현했습니다.

### 안전한 종료

Pi orchestrator가 종료될 때 HOME과 RELAX를 요청하고, Pi 또는 Wi-Fi가 사라져도 ESP32의 독립 watchdog이 hold 후 detach합니다.

## 현재 한계

- MG996R은 position feedback이 없는 아날로그 서보이므로 명령각과 실제각의 오차를 직접 측정하지 못합니다.
- 단순 2-link IK 모델은 구조적 유연성과 backlash를 포함하지 않습니다.
- RGB–열화상 affine은 calibration 거리 주변에서 가장 정확하며 시차가 큰 거리 변화에는 한계가 있습니다.
- Phase 4의 장시간 실기기 acceptance와 정량 tracking 성능은 후속 과제입니다.

## Security

실제 Wi-Fi 자격정보는 `secrets.h`로 분리했습니다. 공개 이력에 과거 값이 포함됐던 경우 해당 비밀번호를 폐기하고 Git history에서도 제거해야 합니다.

## ROBOTIS 직무 연결

MG996R에는 위치·부하·온도 feedback이 없어 명령각과 실제각의 차이를 측정하지 못했습니다. 이 한계 때문에 상태를 되읽을 수 있는 DYNAMIXEL과 hardware interface에 관심을 갖게 됐습니다. 현재는 문서·코드 학습 단계이며, 직접 제어와 실기기 검증이 끝나기 전까지 DYNAMIXEL SDK 경험으로 표시하지 않습니다.

## Related Portfolio

[ROS 2 Robot Systems Software Portfolio](https://github.com/spongebobDG/robotics-software-portfolio)
