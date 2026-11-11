# Quest 3 / Vive Tracker 하드웨어 시험 계획

이 문서는 실제 G1이 아닌 로컬 MuJoCo에서만 시험한다. 현재 코드에는 physical G1
출력 경로가 없으며 Unitree SDK도 import하지 않는다.

## 1. 시험 순서

1. Quest 3 HMD와 양손 Touch controller만으로 arm-only simulation을 확인한다.
2. Linux VIVE Hub에서 Ultimate Tracker 5개가 모두 `Ready`이고 VUT pose가 나오는지 확인한다.
3. tracker serial, 장착 위치와 좌표 offset을 기록한다.
4. full-body 입력은 dry-run과 recorded replay로 먼저 검증한다.
5. full-body simulation publish는 별도 안정성 검증을 통과하기 전까지 활성화하지 않는다.

## 2. 필요한 장비와 소프트웨어

- Meta Quest 3, 좌우 Touch controller
- ALVR client/server와 SteamVR
- 후속 full-body 시험: Vive Ultimate Tracker 5개와 Wireless Dongle 1개
- VIVE Hub Linux Beta와 udev USB 권한
- 이 저장소의 `.conda-vr` 환경
- 공식 TWIST2 ONNX checkpoint와 MuJoCo G1 XML

Quest/ALVR pose/action 입력과 stereo 출력은 SteamVR OpenVR API를 사용한다. Linux
직접 연결의 Ultimate Tracker pose는 SteamVR가 아니라 VIVE Hub VUT API를 사용한다.

## 3. 첫 번째 시험: Quest만으로 팔 움직이기

### 시작

ALVR에서 Quest가 연결되고 SteamVR가 실행된 상태에서 터미널 하나에 다음만 입력한다.

```bash
cd /home/railab/Desktop/vr_teleop/TWIST2
bash scripts/run_quest_arms_vr.sh
```

이 launcher는 전용 localhost Redis, Quest 입력 publisher, 공식 TWIST2 ONNX
controller, MuJoCo, SteamVR stereo viewer를 함께 시작한다. 실제 로봇에는 연결하지
않는다.

### 착용 후 체크리스트

- [ ] Quest에서 G1이 정면 약 2.2 m 위치에 보인다.
- [ ] 양손 controller를 들어 움직여 tracking을 깨운다.
- [ ] 양쪽 grip을 완전히 놓고 두 손을 편한 중립 위치에서 약 1초 고정한다.
- [ ] 터미널 `runtime/quest_arms_publisher.log`에 `[READY]`가 나온다.
- [ ] 양쪽 grip을 계속 누르고 한 손을 조금씩 앞/뒤 또는 위/아래로 움직인다.
- [ ] 해당 G1 어깨·팔꿈치만 부드럽게 따라 움직인다.
- [ ] 머리를 좌우로 돌리거나 숙여도 팔 목표는 움직이지 않는다.
- [ ] grip 하나를 놓으면 양팔이 마지막 목표에 멈춘다.
- [ ] grip을 놓은 채 손 위치를 옮기고 다시 양쪽을 누르면 튀지 않고 새 위치에서 시작한다.
- [ ] root, 다리, 허리, 손목 관절은 기준 자세를 유지하고 G1이 넘어지지 않는다.

첫 프로필은 controller 위치 변화만 사용한다. controller 회전, HMD의 프레임별
회전, 연결된 Vive tracker는 의도적으로 무시한다.

중립 기준 손목 목표에는 X/Y/Z 고정 범위 cap을 적용하지 않는다. 이 프로필은 pose
smoothing, 속도-jump 차단, 입력 gain, Cartesian/joint slew, 관절 한계 margin,
neutral regularization, IK residual projection도 적용하지 않는다. controller
translation은 1:1로 IK에 들어가며 MuJoCo G1 XML의 실제 관절 가동범위만 유지한다.

### 버튼

| 입력 | 결과 |
| --- | --- |
| 양쪽 grip 유지 | arm clutch 활성화 |
| grip 하나라도 release | 현재 arm 목표 hold |
| 오른쪽 joystick 클릭 | 정면 3인칭 시점 재정렬 |
| 양쪽 joystick 동시 클릭 | arm E-stop latch |
| 양쪽 grip release 후 `Y+B` | E-stop 해제, 중립 보정 재시작 |
| mirror 창 `R` | 시점 재정렬 예비 입력 |
| 터미널 `Ctrl+C` | 전체 launcher 종료 |

E-stop과 시점 재정렬은 서로 다른 계층이다. 양쪽 joystick을 누르면 화면도 재정렬될
수 있지만 arm E-stop이 우선한다.

### 예상 상태

- `WAITING_TRACKING`: HMD/Touch pose가 valid가 아님. controller를 깨운다.
- `WAITING_INPUT`: grip action을 아직 읽지 못함. SteamVR binding과 controller를 확인한다.
- `WAIT_RELEASE`: 양쪽 grip을 완전히 놓아야 함.
- `CALIBRATING`: 중립 위치에서 움직이지 않고 잠시 기다린다.
- `READY`: grip을 누르기 전 또는 release 후 hold 상태.
- `ARMED`: 양쪽 grip이 눌려 arm position 입력 중.
- `E_STOP`: grip release 후 `Y+B`로 해제한다.
- `RECONNECTED`: 장치가 다시 연결되어 중립 보정을 다시 수행해야 함.
- `TRACKING_LOST`: 표시된 HMD/controller role의 pose가 무효임.

주기 상태 줄의 `IK=L=...mm R=...mm`는 요청한 손목 위치와 실제 IK 결과 사이의
남은 거리다. 소프트웨어 projection은 없으므로 `scale`이나 `IK_PROJECTED` 상태도
없다. 이 값이 커지면서 팔이 멈추면 실제 G1 어깨·팔꿈치의 관절 가동범위에 도달한
것이다.

### 안전 확인

publisher가 중단되거나 payload가 잘못되면 MuJoCo consumer는 300 ms 뒤 neutral
reference로 전환한다. 로그에는 `Redis action stream is stale/invalid`와 복구 시
`Redis action stream is fresh`가 상태 전환마다 한 번씩 나온다.

성능 때문에 ALVR가 끊기면 다음처럼 eye render scale을 낮춘다.

```bash
bash scripts/run_quest_arms_vr.sh --vr-render-scale 0.6
```

## 4. 장치 inventory

```bash
bash scripts/run_live_inventory.sh
```

각 장치에서 다음을 기록한다.

- `serial`, `model`, `controller_type`, `tracking_system`
- `class_name`
- `connected`, `pose_valid`, `tracking_result`
- Touch의 `automatic_role`
- tracker의 `mapped_role`

Quest controller가 잠들면 등록 정보는 보이지만 `connected:false` 또는
`pose_valid:false`일 수 있다. `VRLink Hand Tracker` 같은 synthetic hand device는
Touch controller로 선택되면 안 된다.

## 5. Vive Ultimate Tracker full-body 준비

VIVE Wireless Dongle을 Linux 서버에 직접 연결하는 기본 절차는
[VIVE_LINUX_QUICKSTART.md](VIVE_LINUX_QUICKSTART.md)를 따른다. 이때 Ultimate pose는
SteamVR가 아니라 VIVE Hub의 `/tmp/vut.sock`에서 받고, Quest HMD/Touch만
ALVR→Linux SteamVR/OpenVR 경로를 사용한다. Windows relay는 필요 없다.

아래 내용은 tracker가 이미 OpenVR GenericTracker로 보이는 기존 경로의 참고 사항이다.

### 역할 매핑

OpenVR device index는 재연결 후 바뀔 수 있으므로 사용하지 않는다. inventory의
stable serial을 복사해 로컬 설정을 만든다.

```bash
mkdir -p config/local
cp config/tracker_mapping.vive_fullbody.example.json config/local/tracker_mapping.vive_fullbody.json
cp config/coordinates.vive_fullbody.example.json config/local/coordinates.vive_fullbody.json
```

`serial_to_role`에서 placeholder를 실제 serial로 바꾼다. 최소 sparse 배치는 다음이다.

```text
Quest HMD             → Head (자동)
left/right Touch      → Left_Wrist / Right_Wrist (자동)
pelvis Ultimate       → Pelvis (serial mapping)
left/right foot       → Left_Foot / Right_Foot (serial mapping)
```

Tracker가 더 있으면 `Spine3`, `Left/Right_Elbow`, `Left/Right_Knee` 등 실제 장착
위치에 대응하는 role을 추가할 수 있다. 하나의 serial을 여러 role에 쓰거나 여러
serial을 같은 role에 배정하지 않는다.

### 장착 transform

tracker origin은 anatomical joint origin이 아니다. 각 장치에 대해
`config/local/coordinates.vive_fullbody.json`의 `tracker_to_body`를 측정한다.

```text
T_world_joint = T_room_calibration × T_world_device × T_device_joint
```

zero offset은 placeholder일 뿐 실제 보정값이 아니다. 좌/우와 tracker 앞면 방향을
표시하고, 장착을 바꾸면 다시 측정한다.

### room yaw/origin 보정

실제 mapping 파일을 준 뒤 neutral pose에서 다음을 실행한다.

```bash
.conda-vr/bin/python -m twist2_vr live-calibrate \
  --mapping config/local/tracker_mapping.vive_fullbody.json \
  --coordinates config/local/coordinates.vive_fullbody.json \
  --output config/local/calibration.vive_fullbody.json
```

이 단계는 room yaw/origin만 계산한다. mount transform이나 누락된 anatomical joint를
자동으로 구하지 않는다.

## 6. Vive full-body dry-run

```bash
bash scripts/run_vive_fullbody_dry.sh \
  --mapping config/local/tracker_mapping.vive_fullbody.json \
  --coordinates config/local/coordinates.vive_fullbody.json \
  --calibration config/local/calibration.vive_fullbody.json \
  --record runtime/vive_fullbody.jsonl
```

이 명령은 pose→normalization→calibration→reconstruction→공식 GMR→35D reference를
실행하지만 Redis나 로봇에는 쓰지 않는다. 터미널에 `source=openvr`,
`reference=official_gmr`가 보여야 한다.

기록 후 같은 데이터를 replay한다.

```bash
bash scripts/run_replay.sh runtime/vive_fullbody.jsonl --realtime
```

다음 항목을 확인하기 전에는 full-body simulation publish를 열지 않는다.

- [ ] 필수 role이 모두 지속적으로 valid다.
- [ ] 좌/우와 축 방향이 실제 동작과 일치한다.
- [ ] 중립 pose에서 root와 발 높이가 맞는다.
- [ ] tracker를 회전했을 때 관절 frame이 예상 방향으로 회전한다.
- [ ] dropout/reconnect 시 즉시 disarm되고 calibration을 다시 요구한다.
- [ ] JSONL replay에서 NaN/Inf, 큰 점프, 잘못된 root 높이가 없다.
- [ ] retargeted q29가 joint limit 안이며 MuJoCo 장시간 안정성 검증을 통과한다.

현재 sparse reconstruction은 full-body를 유일하게 결정하지 못하며 기존 sparse mock은
낙상했다. 따라서 위 dry-run이 성공해도 balance-safe한 full-body teleop이 입증된 것은
아니다.

## 7. 종료와 로그

- Quest arm test: 실행 터미널에서 `Ctrl+C`
- publisher: `runtime/quest_arms_publisher.log`
- 전용 Redis: `runtime/redis-quest.log`
- 강제 종료 뒤 남은 프로세스 확인:

```bash
pgrep -a -f 'run_quest_arms_vr|twist2_vr quest-arms|server_low_level_g1_sim|redis-server.*16379'
```

정상 종료 후 위 명령은 자기 자신을 제외하고 아무것도 표시하지 않아야 한다.

## 8. Mock과 live 구분

| 항목 | Mock | Quest arm live | Vive Linux full-body |
| --- | --- | --- | --- |
| provider source | `mock` | `openvr` | `quest_openvr+vive_hub_linux` |
| 실제 HMD/Touch pose | 아니오 | 예 | 예 |
| Ultimate Tracker | 아니오 | 무시 | serial mapping |
| simulation publish | 검증 wrapper | 격리된 localhost만 허용 | 차단 |
| physical G1 | 차단 | 차단 | 차단 |

코드 검증은 언제든 다음으로 재실행한다.

```bash
bash scripts/run_vr_tests.sh
bash scripts/run_quest_arm_headless_validation.sh
```
