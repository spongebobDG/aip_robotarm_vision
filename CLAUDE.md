# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 프로젝트 개요

산업용 감시 로봇암: 4축 MG996R 서보암 + RGB/열화상 복합 비전, WiFi/MQTT로 연결된
두 컴퓨팅 노드로 분리되어 있다.
우분투 22.04 LTS SERVER을 사용하고 있다 화면을 확인해야하면 VNC로 확인을 할것이다
- **Raspberry Pi 4** — 비전/AI/FSM 담당, Mosquitto MQTT 브로커 호스팅, CSI 카메라 +
  열화상 모듈(UART) 수신. 코드는 `pi/`.
- **ESP32** — 로컬 50 Hz 실시간 제어 루프 실행, LEDC 하드웨어 PWM으로 4x MG996R 서보 구동,
  WiFi/MQTT로 셋포인트 수신. 코드는 `esp32/`.

전체 설계 원칙: WiFi/MQTT는 저(低)레이트 셋포인트(조인트 목표·pan/tilt delta)만 전달한다.
하드 리얼타임 50 Hz 보간/PWM 루프는 ESP32 로컬에 있어서, WiFi 지연/지터가 서보 모션의
매끄러움에 영향을 주지 않는다. Pi 쪽에서는 카메라 캡처·퓨전/AI·FSM이 각각 별도의
multiprocessing 단계로 동작하며 shared memory/queue로 연결되고 "latest-frame-wins"
방식을 따른다(`PLAN.md` Phase 3/4 참고) — 한 단계가 다른 단계를 블록하면 안 된다.

프로젝트는 단계별로 구축된다(각 단계의 상세 설계·의사코드는 `PLAN.md` 참고):
- **Phase 1 (완료)** — ESP32 펌웨어 브링업: WiFi/MQTT 링크, 4축 서보 제어, 시리얼 조그
  캘리브레이션, 명령 워치독/relax 안전장치.
- **Phase 2** — Pi 쪽 기구학(`pi/kinematics/`, 아직 미생성): 2-link planar IK/FK +
  pan/tilt. `arm_config.yaml`의 `link1_mm`/`link2_mm`/`base_height_mm`를 사용 —
  이 단계 전에 실제 암에서 실측 필요(현재는 0.0 placeholder).
- **Phase 3** — 비전/열화상 퓨전(`pi/vision/`, `pi/comms/thermal_serial.py`, 아직 미생성):
  열화상 모듈은 UART(`/dev/serial0`)로 프레임 스트리밍 — I2C 아님. 원래 MLX90640 칩은
  I2C 전용이지만, VIN/GND/RX/TX로 나온다면 모듈에 변환 MCU가 내장되어 시리얼로 프레임을
  내보내는 것. 파서 구현 전에 모듈 데이터시트에서 baud rate/프레임 포맷을 확인해야 함.
- **Phase 4** — 감시 AI/추적(`pi/app/`, 아직 미생성): FSM 우선순위
  FIRE > INTRUDER > PATROL, 이미지 기반 pan/tilt 비주얼 서보잉.

## 하드웨어/안전 불변조건 (펌웨어나 설정 수정 시 위반하지 말 것)

- 서보 전원(5~6V, ≥6A)은 외부 PSU에서 공급하며 ESP32 핀에서 절대 끌어오지 않는다.
  ESP32 자체는 같은 PSU에서 나온 별도의 벅 레귤레이터로 급전한다(GND는 공통, 전원
  레일은 분리) — 서보 전류 변동이 ESP32를 브라운아웃시키지 않도록.
- 서보 신호 GPIO는 고정: `PIN[4] = {13, 12, 14, 27}` (Base/Shoulder/Elbow/Wrist).
  의도적으로 선택된 PWM-safe 핀들 — 축을 추가할 경우 스트래핑 핀(0/2/15)과
  입력전용 핀(34~39)은 피할 것.
- 조인트 각도 한계와 홈 위치는 **두 곳에 이중 정의**되어 있으며(defense in depth)
  서로 동기화되어야 한다:
  - `esp32/include/config.h` → `MIN_DEG`/`MAX_DEG`/`HOME`/`MAX_DPS`
  - `pi/config/arm_config.yaml` → `axes[].min`/`max`/`home`/`max_dps`
- ESP32의 명령 워치독(`esp32/src/main.cpp`): `CMD_TIMEOUT_MS`(1500ms) 동안 명령이
  없으면 → 마지막 위치 유지(hold); `RELAX_TIMEOUT_MS`(8000ms) 동안 없으면 → 서보
  detach(relax/힘 빼기). 이것이 WiFi 링크 끊김 시의 안전 동작이므로, 제어 루프를
  변경할 때도 이 동작은 유지해야 한다.
- `config.h`의 US_MIN/US_MAX(서보 펄스폭)는 placeholder(500/2500µs)이며, 풀 레인지
  모션을 신뢰하기 전에 시리얼 조그 모드로 서보별 캘리브레이션이 필요하다.

## MQTT 프로토콜 (Pi <-> ESP32)

토픽은 `pi/config/mqtt_config.yaml`(Pi 쪽)과 `esp32/include/config.h`(ESP32 쪽)에
정의되어 있다 — 스키마 변경 시 양쪽 모두 동기화할 것.

| 방향 | 토픽 | 페이로드 | QoS |
|---|---|---|---|
| Pi -> ESP32 | `arm/cmd/joints` | `"b s e w"` 절대 각도(deg) | 0 |
| Pi -> ESP32 | `arm/cmd/pantilt` | `"dpan dtilt"` 상대 nudge(deg) | 0 |
| Pi -> ESP32 | `arm/cmd/mode` | `"HOME"` \| `"RELAX"` | 1 |
| ESP32 -> Pi | `arm/state` | `"b s e w"` 현재 각도(텔레메트리) | 0 |
| ESP32 -> Pi | `arm/status` | `"ok"` \| `"linklost"` \| `"relaxed"` | 1 |
| (Phase 4) | `arm/alarm` | 화재/침입자 이벤트 | — |

축 인덱스 규약(펌웨어·설정·MQTT 페이로드 전체 공통): `0=Base(pan)`,
`1=Shoulder`, `2=Elbow`, `3=Wrist(tilt)`.

## 빌드/실행 명령

### ESP32 펌웨어 (PlatformIO, `esp32/` 디렉터리)
```bash
cd esp32
pio run -t upload              # 빌드 + USB 플래싱
pio device monitor -b 115200   # 시리얼 모니터 / 조그 콘솔
```
플래싱 전에 `esp32/include/config.h`에서 WiFi SSID/비밀번호와 Pi의 고정 LAN IP
(`MQTT_BROKER`)를 환경에 맞게 수정할 것.

시리얼 조그 프로토콜(115200 시리얼 모니터에서 줄바꿈으로 종료되는 명령), 축별
안전 펄스폭을 찾고 캘리브레이션할 때 사용:
- `a0`..`a3` — 축 선택
- `+` / `-` — 선택된 축을 `JOG_STEP`(2도)만큼 조그
- `h` — 홈, `r` — relax(detach), `p` — 현재 각도 출력

### Pi 쪽 (Python, `pi/` 디렉터리)
```bash
sudo apt install -y mosquitto mosquitto-clients python3-picamera2 python3-serial
sudo systemctl enable --now mosquitto      # 로컬 MQTT 브로커
cd pi && pip install -r requirements.txt

mosquitto_sub -t 'arm/#' -v                # 모든 arm 토픽 모니터링
python -m tools.arm_jog                    # Pi -> MQTT -> ESP32 대화형 조그/검증 도구
```
`pip install` 관련: `python3-picamera2`는 pip가 아니라 `apt`로 설치해야 함
(`requirements.txt` 주석 참고).

`pi/tools/arm_jog.py` 키: `0..3` 축 선택, `+`/`-` 조그, `j b s e w` 절대 조인트
전송, `p pan tilt` pan/tilt nudge, `h` 홈, `r` relax, `s` 마지막 텔레메트리 출력,
`q` 종료.

이 리포지토리에는 아직 테스트 스위트나 린터가 구성되어 있지 않다.

## 코드 아키텍처 노트

- `pi/comms/mqtt_link.py` — Pi <-> ESP32 통신을 담당하는 유일한 transport. `MqttLink`는
  `pi/config/mqtt_config.yaml`을 로드하고, paho-mqtt를 래핑하며(`_make_client`로
  1.x/2.x 클라이언트 API 모두 지원), `send_joints`/`pan_tilt`/`home`/`relax`와
  `.state`/`.status` 속성(텔레메트리 콜백으로 갱신됨)을 제공한다. 암을 명령해야 하는
  새 Pi 코드는 paho-mqtt를 직접 다루지 말고 이 클래스를 통해야 한다.
- `esp32/src/main.cpp`는 단일 파일 Arduino 스케치로 다음 구조를 따른다: MQTT 콜백
  (`onMqtt`)이 `tgt[]`를 설정 → 50 Hz `controlStep()`이 `MAX_DPS`에 따라 `cur[]`를
  `tgt[]` 방향으로 rate-limit하며 서보에 기록. `ensureLink()`는 주기적 재시도 로깅과
  함께 WiFi/MQTT 재연결을 처리한다. `serviceSerial()`은 조그 캘리브레이션 콘솔을
  구현한다. 네 가지 관심사(링크 관리, 제어 루프, 텔레메트리 발행, 시리얼 조그)는
  단일 `loop()`에서 협력적으로(cooperative) 실행되며 RTOS 태스크 분리는 없다.
- 설정은 `esp32/include/config.h`(펌웨어에 컴파일됨)와 `pi/config/*.yaml`(런타임에
  로드됨) 양쪽에 의도적으로 중복되어 있다 — 두 쪽이 소스 파일을 공유할 수 없기 때문.
  한계값/토픽/홈 위치를 변경할 때는 양쪽 모두 업데이트할 것.
