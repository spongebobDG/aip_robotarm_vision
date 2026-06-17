# aip_robotarm_vision — 산업용 감시 로봇 (Pi4 + ESP32 WiFi/MQTT)

4축 로봇암(MG996R ×4) + 복합 비전(RGB + 열화상)의 산업용 감시 로봇.
Raspberry Pi 4가 비전·AI·FSM을 담당하고, ESP32가 WiFi/MQTT로 셋포인트를 받아
50 Hz 실시간 루프로 서보를 구동한다. 전체 설계는 개발 계획 문서를 따른다.

```
Pi 4 (vision/AI/FSM + Mosquitto)  ──WiFi·MQTT──►  ESP32 (50Hz LEDC PWM)  ──►  4× MG996R
        ▲ CSI 카메라 / UART 열화상                         ▲ 외부 5~6V PSU(+벅)
```

## 디렉터리
- `esp32/` — ESP32 펌웨어 (PlatformIO / Arduino C++). Phase 1 완료분.
- `pi/`    — Raspberry Pi Python 코드 (comms / kinematics / vision / app / tools).

---

## Phase 1 — 하드웨어 브링업 (현재 구현 완료)

### 1. ESP32 펌웨어 설정 & 플래싱
1. `esp32/include/config.h`에서 `WIFI_SSID`, `WIFI_PASS`, `MQTT_BROKER`(Pi 고정 IP) 수정.
2. 플래싱(USB 연결):
   ```bash
   cd esp32
   pio run -t upload
   pio device monitor -b 115200
   ```
3. 시리얼 모니터에서 `[boot] arm-esp32 ready` + WiFi/MQTT 연결 로그 확인.

### 2. 서보 한계 캘리브레이션 (시리얼 조그)
시리얼 모니터에 입력(엔터):
- `a0`/`a1`/`a2`/`a3` 축 선택 → `+`/`-`로 2°씩 조그 → 기구적 안전 min/max 확인
- 확인한 각도/펄스폭을 `config.h`의 `MIN_DEG/MAX_DEG/US_MIN/US_MAX`에 반영 후 재플래싱
- `h` 홈(90/0/0/90), `r` relax(서보 힘 빼기)

### 3. 전원 / 배선 (요약 — 자세한 내용은 계획 문서)
- 서보 V+: **외부 5~6V, ≥6A PSU** (ESP32 핀에서 급전 금지)
- ESP32 급전: PSU에서 **별도 벅(5V)** → ESP32 VIN (서보 레일과 분리, GND만 공통)
- 공통 GND: PSU ↔ ESP32 ↔ 서보 GND (단일점). Pi는 무선이라 분리
- 서보 신호: ESP32 GPIO13/12/14/27 → (3.3→5V 레벨시프터 권장) → 서보 신호선
- 서보 V+/GND에 벌크 캡 1000µF

### 4. Pi 셋업 & 검증
```bash
# 브로커 + 의존성
sudo apt install -y mosquitto mosquitto-clients python3-picamera2 python3-serial
sudo systemctl enable --now mosquitto
cd pi && pip install -r requirements.txt

# 토픽 모니터 (별도 터미널)
mosquitto_sub -t 'arm/#' -v

# 대화형 조그 도구 (Pi → MQTT → ESP32)
python -m tools.arm_jog
```
조그 도구 키: `0..3` 축선택 · `+`/`-` 조그 · `h` 홈 · `r` relax · `j b s e w` 절대각 · `p pan tilt` 팬틸트 · `q` 종료.

### Milestone 1 (완료 기준)
- [ ] ESP32가 WiFi/브로커 접속, `arm/state` 텔레메트리 발행
- [ ] `arm_jog.py`로 각 축이 한계 내에서 부드럽게 구동
- [ ] 동시 구동 시 브라운아웃 없음(벅 분리 확인)
- [ ] WiFi 끊으면 워치독으로 hold→relax(안전정지)

---

## 다음 단계
- Phase 2: 기구학(IK/FK) — `pi/kinematics/` (`arm_config.yaml`의 `L1/L2/H` 실측 필요)
- Phase 3: 비전·열화상 퓨전 — `pi/vision/`, 열화상 모듈 시리얼 프로토콜 확정 필요
- Phase 4: 감시 AI·추적 — `pi/app/`
