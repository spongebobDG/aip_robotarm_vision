# Phase 2 — 기구학 (Pi 쪽 IK/FK)

목표 / 검증 기준은 `README.md`와 `PLAN.md`의 Phase 2 섹션 참고. 이 파일은 그
기준에 도달하는 과정의 작업 일지.

---

## 2026-06-18

**목표:** `arm_config.yaml`의 링크 길이 placeholder(0.0)를 실측값으로 채우고,
`pi/kinematics/`에 2-link planar IK/FK + pan/tilt를 구현해서 Milestone 2
검증까지 끝내기.

**진행:**
- 실측: base→shoulder 95mm, shoulder→elbow 115mm, elbow→카메라 마운트 120mm.
  → `pi/config/arm_config.yaml`의 `geometry.{base_height_mm,link1_mm,link2_mm}`에
  반영.
- `pi/kinematics/arm_model.py` (`ArmModel`) 신규 작성: `arm_config.yaml`을
  로드해서 서보 각도 ↔ "이론 각도"(pan, θ1=shoulder, θ2=elbow, θ3=wrist) 변환
  + `fk(b,s,e,w) -> (x,y,z,cam_pitch)` 제공.
- `pi/kinematics/ik_solver.py` (`IkSolver`) 신규 작성: `PLAN.md` Phase 2
  의사코드 그대로(`pan=atan2(y,x)`, `th2=acos(c2)`, `th1=atan2(zp,r)-atan2(...)`,
  `th3=cam_pitch-(th1+th2)`) 구현. 한계 밖이면 `Unreachable` 예외.
- `pi/tools/kinematics_check.py` 신규 작성: 하드웨어/MQTT 없이 순수 수학
  검증만 하는 스크립트 — 알려진 서보각 → FK → IK 역산 → 원래 각도와 비교
  (round-trip), + 임의 xyz 목표의 도달가능성 체크.
- 실행 결과: 4개 테스트 자세 모두 round-trip 오차 0.00°. 도달불가 케이스(어깨가
  0° 미만으로 내려가야 하는 목표, 사거리 초과 목표)는 의도대로 reject됨.

**트러블슈팅:**
- **증상(설계 이슈, 에러는 아님):** elbow→카메라 마운트 사이가 직선 로드가
  아니라 **ㄱ자로 꺾인 브래킷**이라서, "shoulder/elbow servo가 둘 다 home(0°)일
  때 link1-link2가 일직선"이라는 단순 2-link 모델의 암묵적 가정이 깨질 수
  있음. 사용자가 잰 120mm는 그 꺾인 구간을 따라간 길이가 아니라 elbow 축에서
  카메라 마운트까지의 **대각선(직선) 거리**.
  - **원인 분석:** 평면 IK 모델이 실제로 필요로 하는 L2는 "링크의 물리적
    재질 길이"가 아니라 "두 축 사이의 유클리드 거리"이므로, 대각선 측정값
    120mm는 **그 자체로 정확한 L2**다 — 추가 변환 불필요. 단, 꺾인 지점이
    servo 0°일 때 "곧게 뻗음" 가정과 안 맞으면 θ2에 **고정 각도 오프셋**이
    생길 수 있는데, 그 오프셋 값은 브래킷의 두 다리 길이/각도를 모르면 계산할
    수 없고 실측(자로 직접 비교)으로만 알아낼 수 있음.
  - **해결:** 값 자체는 수정 없이 그대로 사용(`link2_mm: 120.0`). 대신
    `arm_model.py`의 `AxisMap`에 `sign`(이미 있던 것) 외에 `offset_rad`를
    일반화해서 추가하고, `arm_config.yaml`에 `kinematics.<axis>.{sign,
    offset_deg}` 블록을 신설(기본값 전부 0/1 = 미보정). 실제 암에서 여러
    자세를 명령하고 자로 측정해서 `kinematics_check.py`의 FK 출력과 비교한
    결과 — **테스트한 자세들에서는 일치**, 그래서 일단 offset_deg는 0으로 둔
    채 보류. 사용자 코멘트: "맞으면 나중에 문제 생기면 다시 돌아와서
    해결하자" → 선제적으로 튜닝하지 않고, 실제 불일치가 관찰될 때 그 자세의
    실측값으로 `offset_deg`를 역산하는 방향으로 결정.

**배운 개념:**
- 2-link planar IK/FK에서 "링크 길이"는 물리적 재질 길이가 아니라 **두 회전축
  사이의 유클리드 거리**다 — 링크가 곧은 로드든 ㄱ자로 꺾인 브래킷이든, 모델이
  보는 건 그 사이 거리뿐. 다만 꺾임은 "축 0°일 때 링크가 어느 방향을
  가리키는가"라는 기준점(요철/오프셋)에는 영향을 줄 수 있어서, 길이와 기준점
  오프셋은 별개로 검증해야 하는 두 가지 문제.
- 검증되지 않은 보정값을 미리 추측해서 넣지 않고, 명시적으로 "아직 0(미보정)"
  상태로 남겨두고 실측 불일치가 실제로 나타날 때만 역산하는 방식 — 추측성
  보정이 오히려 더 헷갈리는 오차를 만들 수 있음.

**참고:**
- `PLAN.md` Phase 2 섹션 (IK 의사코드, 역할 분담)
- `pi/kinematics/arm_model.py` docstring (서보각↔이론각 변환식,
  `theta = offset_rad + sign*radians(servo_deg-home)`)
- `pi/config/arm_config.yaml`의 `geometry`/`kinematics` 블록 주석

---

### Milestone 2 검증 — FK/IK 수학 + 실측 비교

**목표:** `README.md`/`PLAN.md` Milestone 2 체크리스트 중, MQTT 없이도 확인
가능한 부분(FK round-trip, 도달가능성)과 실제 암 측정 비교.

**진행:**
- `python3 -m tools.kinematics_check` (4개 자세 round-trip + 4개 xyz
  도달가능성) → 전부 기대대로 동작.
- 실제 암에 known 자세를 명령 후 자로 측정 → `ArmModel.fk()` 출력과 비교 →
  일치 확인(사용자 검증, 2026-06-18).

**검증 결과:**
- FK/IK 수학 자체는 self-consistent (round-trip 오차 0).
- 실제 하드웨어와도 테스트한 자세들에서는 일치 — `kinematics.<axis>.offset_deg`
  보정 없이도 현재 단계는 통과로 판단.
- **주의:** 아직 모든 자세(특히 elbow를 크게 굽힌 극단 자세)를 다 검증한 건
  아님 — 추후 불일치 발견 시 이 섹션에 추가 기록 예정.

**참고:**
- `pi/tools/kinematics_check.py`

---

<!-- 다음 세션부터는 docs/devlog/README.md의 템플릿을 복사해서 새 ## YYYY-MM-DD 항목으로 추가 -->
