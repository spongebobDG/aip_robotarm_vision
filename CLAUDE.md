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
- **Phase 2 (완료)** — Pi 쪽 기구학(`pi/kinematics/arm_model.py`(FK),
  `pi/kinematics/ik_solver.py`(2-link IK + pan/tilt)). `arm_config.yaml`의
  `geometry.link1_mm`/`link2_mm`/`base_height_mm`는 실측값(115/120/95mm)으로
  채워짐. `pi/tools/kinematics_check.py`로 FK/IK round-trip·도달가능성 검증, 실제
  암에서 자로 측정해 FK 출력과 일치 확인됨. `kinematics.<axis>.sign`/`offset_deg`
  보정 파라미터는 현재 전부 0/1(미보정) — 특정 자세에서 어긋남이 보이면 그때 보정.
- **Phase 3 (완료)** — 비전/열화상 퓨전. `pi/vision/camera_capture.py`(Picamera2,
  latest-frame-wins), `pi/vision/fusion.py`(colormap→업스케일→warpAffine→오버레이),
  `pi/comms/thermal_serial.py`(UART 프레임 파서), 정합 캘리브레이션은 GUI 없는
  서버라 **웹 기반** `pi/tools/fusion_calibrate_web.py`(클릭) + `vision_web_preview.py`
  (RGB+열화상 라이브 비교/조준)를 사용(cv2-창 `fusion_calibrate.py`는 X 필요라 미사용),
  설정 `pi/config/{camera_config,fusion_calib,thermal_config}.yaml` 모두 작성·캘리브됨.
  ✅ **블로커 해소(2026-06-18):** 열화상 UART 프로토콜을 실측으로 역공학해
  `confirmed: true` 완료(상세: `docs/devlog/phase3-vision.md`). 확정된 프레임:
  115200 baud, 마커 `5A 5A`, `[len:2][Ta:2]` 헤더 + 768px(24×32) uint16 LE
  (`temp_c=raw*0.01`) + 16비트 워드합 체크섬, 총 1544바이트.
  `thermal_serial.py`가 헤더 스킵·Ta(`last_ta_c`)·체크섬 검증까지 처리.
  실제 프레임 8/8 체크섬 검증 통과. 카메라도 `media_device`를 `/dev/media0`로
  바로잡아 ~23fps 정상(아래 트러블슈팅 참고). 재부팅 후 sudo 없이 두 센서 모두
  수신 확인, 실하드웨어 end-to-end 퓨전 1장(`captures/fusion_overlay.png`) 생성.
  열화상에 **죽은 픽셀 3개(항상 615°C 고정)**가 있어 — 체크섬은 통과하지만
  비물리적 — `thermal_serial`이 물리범위 밖 픽셀을 8-이웃 중앙값으로 자동 보정
  (`thermal_config.yaml`의 `bad_pixel` 섹션, `last_bad_pixels`로 카운트 노출).
  ✅ **RGB↔열화상 정합 캘리브레이션 완료(2026-06-18):** `fusion_calib.yaml`의
  `affine`이 실측값(scale≈0.90, 회전≈3.8°, 이동 +26.7/+17.5px)으로 채워짐,
  보정 오버레이 `captures/fusion_overlay_calibrated.png`로 확인. GUI 없는 서버라
  cv2-창 `fusion_calibrate.py` 대신 **브라우저 기반 `fusion_calibrate_web.py`**(클릭
  정합)와 `vision_web_preview.py`(RGB+열화상 라이브 비교, 조준용)를 사용. 열화상은
  카메라 대비 상하반전이라 `thermal_config.yaml`의 `orientation.flip_vertical:true`로
  **파서에서** 교정(affine은 반사를 못 하므로 캘리브 전 필수). **Phase 3 완료.**
  ⚠️ **선행 시스템 설정 두 가지:** (1) 모듈이 GPIO14/15(`/dev/serial0→ttyS0`)에
  물려 있는데 이 UART이 리눅스 **시리얼 콘솔**로 점유돼 있었음 — `cmdline.txt`에서
  `console=serial0,115200` 제거 + `serial-getty@ttyS0` disable로 해방함. 재부팅
  후 `/dev/ttyS0`가 `root:dialout`로 돌아와 sudo 없이 접근 가능(재부팅 전엔 sudo
  필요). (2) `camera_config.yaml`의 `media_device`는 ISP(`/dev/media1`)가 아니라
  센서가 사는 `/dev/media0`여야 함.
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
- `pi/vision/camera_capture.py`의 `CameraCapture`는 백그라운드 스레드로 Picamera2를
  계속 읽어 단일 버퍼를 덮어쓰는 "latest-frame-wins" 패턴이다 — 느린 소비자(퓨전/AI)가
  캡처율을 막거나 stale 프레임이 쌓이지 않게 함(PLAN.md Phase 3/4 원칙). 아직 별도
  multiprocessing 프로세스로 분리되어 있지 않음(소비자가 없어서) — `pi/app/main.py`
  생기면 그쪽에서 프로세스 분리.
- `pi/comms/thermal_serial.py`의 `ThermalSerial`은 이제 실측 확정된 프로토콜을
  파싱한다(`thermal_config.yaml` `confirmed: true`). 단 가드는 그대로 살아 있어서
  `confirmed: false`로 되돌리면(검증 안 된 다른 모듈로 교체 등) `start()`가
  `ProtocolUnconfirmed`를 던진다 — 검증 전 모듈로는 절대 우회하지 말 것(잘못된
  프레임 해석이 조용히 가짜 온도를 만들어낼 수 있음). 또한 센서 죽은 픽셀(체크섬은
  통과하나 비물리적 고정값)을 `_repair_bad_pixels()`로 8-이웃 중앙값 보정한다 —
  체크섬(전송 무결성)과 bad-pixel(센서 결함)은 책임이 다른 별개 방어 계층이다.
  방향 교정(`orientation` 설정)도 read_frame에서 적용해 모든 소비자가 동일 프레임을
  받는다. 연속 스트리밍 소비자(라이브뷰 등)는 `read_frame()`을 계속 호출(버퍼를
  비워 stale 없음), **가끔 읽는 온디맨드 소비자**(캡처 도구)는 `read_fresh()`를 써야
  한다 — tty 버퍼가 꽉 차면 새 데이터를 드롭하므로 stale 폐기 후 재동기화로 최신
  프레임을 얻는다(상세: `docs/devlog/phase3-vision.md`).
