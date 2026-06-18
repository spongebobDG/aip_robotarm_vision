# Phase 1 — 하드웨어 브링업 (ESP32 + WiFi/MQTT 링크)

목표 / 검증 기준은 `README.md`와 `PLAN.md`의 Phase 1 섹션 참고. 이 파일은 그
기준에 도달하는 과정의 작업 일지.

---

## 2026-06-18

**목표:** 서보 결선 + ESP32 펌웨어 업로드가 끝난 상태에서, Pi <-> ESP32
WiFi/MQTT 링크가 실제로 동작하는지 검증.

**진행:**
- 현재 머신이 로봇 Pi 본체임을 확인 (`uname` → raspi 커널, Ubuntu 22.04.5 LTS
  aarch64, `wlan0` IP `192.168.0.14` — `esp32/include/config.h`의
  `MQTT_BROKER`와 일치).
- `pip3 install --user paho-mqtt pyserial` — `pi/comms/mqtt_link.py` 실행에
  필요한 최소 의존성만 설치 (Phase 3/4용 opencv/mediapipe는 아직 불필요해서 보류).
- `mosquitto` 서비스 active 상태 확인.

**트러블슈팅:**
- **증상:** `mosquitto_sub -t 'arm/#' -v`로 8초 구독했는데 아무 메시지도 안
  들어옴. `journalctl -u mosquitto`에도 ESP32의 연결 시도 로그 자체가 없음.
- **원인:** Mosquitto **2.0**부터 보안 강화를 위해 기본 동작이 바뀌어서, `listener`
  지시자를 명시하지 않으면 **127.0.0.1(localhost)에만 바인딩**한다(1.x 시절엔
  0.0.0.0 전체 바인딩이 기본이었음 — open relay 사고가 많아서 바뀐 것). 그래서
  같은 LAN의 ESP32(WiFi)는 브로커에 TCP 연결 자체를 못 함.
  `ss -tlnp | grep 1883`로 바인딩 주소를 직접 확인하는 게 가장 빠른 진단 방법.
- **해결:** `/etc/mosquitto/conf.d/arm.conf` 생성:
  ```
  listener 1883 0.0.0.0
  allow_anonymous true
  ```
  이후 `ss -tlnp | grep 1883` → `0.0.0.0:1883` 확인, `mosquitto_sub -t 'arm/#' -v`
  재실행 → `arm/state 90.0 0.0 0.0 90.0`이 200ms 간격으로 수신됨 (HOME 위치,
  `STATE_PUB_MS=200`과 일치). 링크 정상 동작 확인.
  > 트레이드오프: LAN 내 누구나 인증 없이 브로커에 붙을 수 있음. 집 공유기
  > 내부망이라 허용했지만, 외부에 노출되는 환경이면 password file이나 TLS가
  > 필요함.

**배운 개념:**
- Mosquitto 2.0의 기본 listener 바인딩 변경 (`127.0.0.1` vs `0.0.0.0`) — 버전
  업그레이드로 인한 암묵적 동작 변화는 공식 변경 로그를 봐야 알 수 있음.
- `ss -tlnp`로 포트 바인딩 주소를 확인하는 법 — "포트가 열려 있다"와 "어느
  인터페이스에 열려 있는가"는 다른 질문.
- ESP32 쪽 텔레메트리 주기(`STATE_PUB_MS`)와 watchdog 타임아웃(`CMD_TIMEOUT_MS`,
  `RELAX_TIMEOUT_MS`)이 `esp32/include/config.h`에 정의되어 있고, 실제 동작이
  그 값과 일치하는지 텔레메트리로 검증할 수 있다는 것.

**참고:**
- `PLAN.md` Phase 1 섹션 (핀 맵, MQTT 토픽 스키마, 워치독 설계)
- `CLAUDE.md`의 "하드웨어/안전 불변조건" — 워치독 타임아웃 값 등
- Mosquitto 2.0 변경사항: 공식 changelog의 "default listener" 관련 항목

---

### [해결] MQTT 링크는 정상인데 서보가 실제로 안 움직임

**증상:** `arm/cmd/mode HOME`, `arm/cmd/joints "45 0 0 90"`을 발행하면 `arm/state`
텔레메트리는 정확히 반영됨(45.0 0.0 0.0 90.0) — 즉 ESP32 펌웨어 로직(파싱, 보간,
워치독)은 전부 정상 동작. 하지만 **물리적으로 서보는 전혀 움직이지 않음**.

**원인:** 결선 문제. 전원(18650 ×2)/공통 GND/신호선을 점검하는 과정에서 선을
재배치하면서 해결됨 — 정확히 어느 선이 헐거웠는지는 특정하지 못했으나, 점검
4가지(전원/공통GND/신호선/축별 확인) 자체에는 "이상 없음"으로 나왔던 걸 보면
헐거운 핀헤더 접촉 같은, 멀티미터로는 안 잡히고 물리적 재결선으로만 드러나는
유형의 문제였을 가능성이 높음.

**해결:** 선 재배치 + 아두이노 재업로드 후 재부팅. 이후 시리얼 조그로 모터 단독
동작 확인 → MQTT로 `arm/cmd/joints "45 0 0 90"` 발행 → base축이 실제로 45도까지
회전하는 것을 육안으로 확인. MQTT → ESP32 → 서보 전체 경로 정상.

**배운 점:** 텔레메트리(`arm/state`)가 정상 값을 보고한다고 해서 실제 모션까지
보장되는 게 아니다 — `servo[i].write()`는 호출됐지만 그 이후(LEDC PWM → 신호선 →
서보 제어칩)는 별도의 신뢰 경계. 소프트웨어 로그만 보고 "통신은 됐으니 펌웨어가
문제"라고 단정하지 말고, 항상 물리적 결선까지 내려가서 확인할 것. 헐거운 결선은
멀티미터 연속성 테스트로도 종종 안 잡히니, 의심되면 직접 재결선/재꽂기를 시도해
보는 게 빠른 경우가 있다.

---

### Milestone 1 검증 — 4축 조그 + 브라운아웃/워치독 확인

**목표:** `README.md` Milestone 1 체크리스트의 나머지 항목(4축 전체 조그, 동시
구동 브라운아웃, 워치독 자동 relax) 검증.

**진행:**
- `python -m tools.arm_jog`로 4축(base/shoulder/elbow/wrist) 전부 조그 — 한계
  범위 내에서 정상 동작 확인.
- 조그하는 동안 `mosquitto_sub -t 'arm/#' -v -F '%I %t %p'`를 백그라운드로 띄워
  타임스탬프 찍힌 전체 토픽 로그를 `/tmp/arm_monitor.log`에 수집(5분 18초,
  `arm/state` 1581건).

**검증 결과:**
- **브라운아웃:** `arm/state` 메시지 간 1초 이상 갭이 0건. ESP32가 리셋되면
  WiFi+MQTT 재연결 동안 반드시 텔레메트리 갭이 생기므로, 갭이 없다는 것은 4축
  연속 조그 중 브라운아웃이 없었다는 근거가 됨.
- **워치독(RELAX_TIMEOUT_MS=8000ms):** 조그 중 입력이 멈췄던 3개 구간 모두에서
  마지막 명령 후 정확히 8초 뒤 `arm/status relaxed`가 *자동으로* 발행됨
  (예: `11:41:36 cmd/joints` → `11:41:44 status relaxed`). 별도 RELAX 명령 없이
  명령 끊김만으로 워치독이 동작함을 확인.
- **Milestone 1 (README.md 기준) 전 항목 충족** → Phase 1 완료.

**배운 개념:**
- 워치독/브라운아웃처럼 "물리적으로 지켜봐야 할 것 같은" 안전 동작도, MQTT
  텔레메트리 타임스탬프 갭 분석만으로 충분히 검증 가능한 경우가 있다 — 굳이
  매번 실제로 WiFi를 끊거나 8초씩 기다리는 새 테스트를 만들 필요 없이, 정상
  사용 중 자연스럽게 생긴 idle 구간을 사후 분석해도 같은 증거가 나온다.

**참고:**
- `README.md`의 "Milestone 1 (완료 기준)" 체크리스트
- `esp32/include/config.h`의 `CMD_TIMEOUT_MS`/`RELAX_TIMEOUT_MS`

---

<!-- 다음 세션부터는 위 README.md의 템플릿을 복사해서 새 ## YYYY-MM-DD 항목으로 추가 -->
