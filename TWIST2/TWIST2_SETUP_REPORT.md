# TWIST2 설치 및 검증 보고서

작성일: 2026-10-09, 최종 갱신: 2026-10-12  
작업 위치: `/home/railab/Desktop/vr_teleop/TWIST2`

## 결론

공식 TWIST2의 pretrained G1 student controller와 예제 모션은 정상 자산이며, 별도 Python 3.10 환경에서 다음 경로를 실제로 검증했다.

```text
full synthetic trackers
→ 좌표 변환/보정
→ XRobot 형식 전신 pose
→ 공식 GMR xrobot→G1 IK
→ 35D TWIST2 reference
→ localhost Redis
→ 공식 1432D ONNX low-level controller
→ residual joint target + PD
→ MuJoCo G1
```

5초 실시간 headless 검증에서 G1 root 높이는 `0.7814~0.7930 m`, 최대 base tilt는 `1.77°`, 최대 절대 torque는 `32.66`이었고 낙상 판정을 통과했다. 이는 **FK에서 역생성한 full mock tracker fixture** 결과이며 실물 Quest나 tracker 검증 결과가 아니다.

별도의 Quest arm-only 경로는 HMD 시점과 로봇 제어를 분리하고, 양손
controller translation을 G1 shoulder/elbow position IK로 변환하며
root·legs·waist·wrist DoF를 stand reference에 고정한다. 사용자가 Quest 3와
양손 Touch controller, 양쪽 grip clutch로 실행해 로봇의 팔과 어깨가 추종함을
확인했다. 첫 live 시험에서 수직 target이 기존 `+0.20 m` cap에 도달했고,
후속 합성 수직-envelope 검증 후 cap을 `+0.30 m`로 늘렸다. 확장된 800-frame
reference도 안정성 검증을 통과했지만, Quest 재시험에서 축별 cap이 실제 대각선 팔
궤적을 잘라내는 것을 확인했다. 현재 Quest arm 코드는 X/Y/Z Cartesian cap과 함께
pose smoothing, 속도-jump 차단, gain, Cartesian/joint slew, 관절 margin, neutral
regularization, IK projection을 모두 제거했다. controller translation을 1:1로 넣은
후속 검증에서도 root 높이 `0.779882~0.792998 m`, 최대 tilt `4.9416°`, 최대 torque
`33.4907`로 통과했다. direct-input 변경 후 Quest live 재시험은 아직 필요하다.

## 소스 상태

| 구성요소 | URL | 고정 커밋 |
|---|---|---|
| TWIST2 | `https://github.com/amazon-far/TWIST2.git` | `b06178f19a22f2138cbd31f60c6d494bc263f67d` |
| GMR | `https://github.com/YanjieZe/GMR.git` | `bb1bbe40774794fceb2a7c579a3464a28e68c844` |

기계 판독용 기록은 `SOURCE_PROVENANCE.json`에 있다. 기존 H2H/OmniH2O 및 상위 `alvr` 코드는 변경하지 않았다.

## 환경

- 격리 환경: `.conda-vr`, Python 3.10.22
- GPU: NVIDIA RTX 4070 12GB, driver 580.159.03
- VR runtime: SteamVR active runtime 및 ALVR 20.14.1 확인
- OpenVR Python: 2.12.1401
- 온라인 GMR: MuJoCo 3.15.0, Mink 1.3.0, QP solver 설치
- GMR import에 필요한 PyTorch는 CUDA 패키지를 받지 않고 `torch 2.5.1+cpu`로 설치
- 로컬 Redis: 환경 내부 `redis-server 5.0.3`; system service와 외부 bind를 사용하지 않음
- ONNX Runtime: CPU provider. GPU provider는 설치하지 않았으며 CPU 추론은 충분히 빨랐다.
- Quest 3 HMD와 좌우 Touch controller는 ALVR/SteamVR에서 실제 serial/role로 열거했다.

환경 재생성:

```bash
cd /home/railab/Desktop/vr_teleop/TWIST2
bash scripts/setup_vr_env.sh
```

## 공식 자산 검증

### Controller checkpoint

| 파일 | SHA-256 | 실제 입출력 | 결과 |
|---|---|---|---|
| `twist2_1017_20k.onnx` | `2d1fb3…5061` | `[batch,1432] → [batch,29]` | ONNX checker 및 CPU inference 통과 |
| `twist2_1017_25k.onnx` | `126ab9…f2e` | `[batch,1432] → [batch,29]` | ONNX checker 및 CPU inference 통과 |

공식 실행 스크립트는 20k checkpoint를 사용한다. README에 명시된 checkpoint는 별도 다운로드 없이 저장소에 포함되어 있었다. 이 ONNX는 README가 말하는 pretrained **student controller**이며 teacher checkpoint가 아니다.

실제 observation 계약은 다음과 같다.

```text
current:  reference35 + proprio92 = 127
history:  10 × 127               = 1270
future:   current reference 복제 = 35
total                               1432
output: 29 joint residual actions
```

공식 코드 주석 일부의 `1402`는 산술 오타이며 코드와 ONNX는 모두 1432로 일치한다.

### G1 관절

TWIST2 deploy XML과 GMR XML의 actuator 순서가 정확히 일치했다.

```text
left leg 6 → right leg 6 → waist 3 → left arm 7 → right arm 7 = 29
```

정책 출력은 `default_dof_pos + 0.5 * clipped_action`으로 적용된다.

공식 low-level controller는 Redis의 35D reference 자체에는 Cartesian cap, gain,
slew 또는 IK projection을 적용하지 않는다. 공식 ONNX output clip(`±10`),
`action_scale=0.5`, PD torque clip, XML joint range는 각각 학습 action 계약과 G1 물리
모델이므로 유지했다. 이 값들은 Quest 손목 목표를 잘라낸 원인이 아니며, 제거해도
Cartesian 도달범위는 늘지 않고 pretrained policy 안정성만 깨질 수 있다.

### 모션

- `assets/example_motions`의 10개 pickle 모두 로드·finite·29 DoF 검사를 통과했다.
- 각 파일은 이미 G1 형식인 `root_pos[T,3]`, `root_rot[T,4](xyzw)`, `dof_pos[T,29]`, `local_body_pos[T,38,3]`를 포함한다.
- 따라서 예제 파일은 단순 원본 human motion이 아니라 G1 reference로 사용 가능한 형식이다.
- 상세 hash/프레임 수는 `runtime/asset_validation.json`을 `scripts/validate_assets.sh`로 다시 생성할 수 있다.

## Simulation 결과

### 공식 example motion

`assets/example_motions/0807_yanjie_walk_003.pkl`을 공식 `MotionLib`으로 읽고,
공식 motion server와 같은 20 ms future sample 및 35D reference 구성을 사용해
pretrained controller에 공급했다. CPU/headless 5초 결과는 다음과 같다.

| 지표 | 값 |
|---|---:|
| root height 최소/최대 | 0.7765 / 0.8032 m |
| 최대 base tilt | 10.45° |
| 최대 절대 torque | 61.95 |
| 결과 | 통과 |

```bash
bash scripts/run_example_motion_validation.sh
```

upstream `server_motion_lib.py`는 CUDA를 hard-code하고 viewer를 강제로 열기 때문에,
headless 검증 wrapper는 공식 `pose/pose/utils/motion_lib_pkl.py`를 그대로 재사용하고
동일한 reference 산식을 CPU에서 적용한다. 시작에는 0.2초 default history prime과
2초 선형 ramp를 넣은 뒤 exact sample을 스트리밍했고, 종료에도 default ramp를 쓴다.
low-level controller 코드는 변경하지 않았다.

### 공식 stand reference

공식 low-level controller를 50Hz, MuJoCo physics 1kHz로 5초 실행했다.

| 지표 | 값 |
|---|---:|
| root height 최소/최대 | 0.7804 / 0.7930 m |
| 최대 base tilt | 2.04° |
| 최대 절대 torque | 34.12 |
| 결과 | 통과 |

### full mock tracker → GMR → controller

| 지표 | 값 |
|---|---:|
| root height 최소/최대 | 0.7814 / 0.7930 m |
| 최대 base tilt | 1.77° |
| 최대 절대 torque | 32.66 |
| 실제 policy loop 출력 | 약 47Hz, 목표 50Hz |
| 결과 | 통과 |

재현 명령:

```bash
bash scripts/run_headless_validation.sh
```

GUI mock simulation:

```bash
bash scripts/run_mock_sim.sh
```

공식 `sim2sim.sh`는 `--policy_frequency 100`을 전달하지만 학습 설정과 motion server는 50Hz다. 검증 래퍼는 학습 조건과 같은 50Hz를 사용한다.

### Quest controller arm-only IK → controller

공식 XRobot→G1 GMR 설정의 팔 target은 orientation cost를 사용하고 position
cost가 0이므로 controller translation을 바로 wrist position에 넣어도 팔이 따라가지
않는다. 첫 Quest 시험용 `QuestArmIKRetargeter`는 G1 MuJoCo model의 position
Jacobian으로 shoulder 3 DoF와 elbow만 해석하고 나머지 reference를 중립으로
고정한다. 이 경로는 simulation-only이며 physical G1 output을 가지지 않는다.

50 Hz, 800-frame 순수 수직 `+0.60 m` 한 팔과 실제 Quest 로그 peak 방향
(`max |X/Y/Z| = 0.238/0.213/0.600 m`)을 재현한 양팔 trajectory 결과:

| 참조 지표 | 값 |
|---|---:|
| 제거된 X/Y/Z cap 모두 초과 | true |
| 최대 frame-to-frame joint step | 0.451114 rad |
| 최대 IK residual | 0.006345 m |
| root/legs/waist/wrist protected drift | `5.96e-9` rad |
| physical joint-limit violation | 0 rad |

이 reference를 pretrained controller에 공급한 13초 실시간 headless 결과:

| 지표 | 값 |
|---|---:|
| root height 최소/최대 | 0.779882 / 0.792998 m |
| 최대 base tilt | 4.9416° |
| 최대 절대 torque | 33.4907 |
| 결과 | 통과 |

```bash
bash scripts/run_quest_arm_headless_validation.sh
```

Redis consumer에는 35D shape, finite value와 `t_action` age를 검사하는 watchdog를
추가했다. payload가 누락·손상·지연되면 exception으로 종료하지 않고 neutral
reference와 zero hand/neck command로 fallback하며, fresh stream 복구도 상태 변경 시
한 번만 로그한다.

live Quest CLI는 주기 상태 줄의 `IK=L=...mm R=...mm`에 요청 손목과 실제 IK 결과
사이의 residual을 출력하며, tracking loss 로그는 무효한 HMD/컨트롤러 role을
특정한다. 입력 shaping이나 software projection은 적용하지 않는다.

### 의도적으로 실패한 sparse test

HMD·양손·pelvis·양발만 측정하고 나머지 관절 방향을 단순 kinematic estimate로 채운 초기 mock은 5초 안에 넘어졌다(`root z` 최소 0.126m, 최대 tilt 92.9°). 따라서 sparse reconstructor는 데이터 흐름 테스트용이며 현재 balance-safe라고 간주하지 않는다. 실제 full-body tracker 배치, 더 나은 pose solver 또는 학습된 student prior가 필요하다.

## Training 및 데이터셋 상태

- 전체 TWIST2 dataset은 내려받지 않았다.
- `legged_gym/motion_data_configs/twist2_dataset.yaml`은 저자 내부 절대 경로를 가리키며 로컬 파일은 없다.
- 공식 config에는 약 19,426개 G1 motion 항목이 있지만 실제 dataset이 없으므로 training은 현재 실행할 수 없다.
- Legacy Isaac Gym도 설치되어 있지 않다. 이 서버의 Isaac Sim/Isaac Lab은 자동 대체물이 아니다.
- `nvcc`와 CUDA toolkit도 없어서 Isaac Gym extension build는 별도 준비가 필요하다.
- pretrained controller 재사용이 현재 목표이므로 PPO training은 실행하지 않았다.

## 자동 테스트

```bash
bash scripts/run_vr_tests.sh
```

결과: `61 passed`. 공식 example MotionLib reference, GMR IK smoke, 좌표·quaternion,
calibration, stale/dropout, reconstruction provenance, 35D reference, joint order, JSONL
replay, OpenVR HMD/Touch 선택·action fallback·reconnect generation, Quest grip clutch/deadman,
arm-only IK/protected DoF, IK/tracking telemetry, Redis freshness watchdog, safe fallback,
front-view HMD camera와 controller recenter edge 처리, VIVE Hub VUT protocol/state/slot
재사용 fail-closed 및 Quest/VIVE hand-eye 공간 정렬을 포함한다.

## 확인하지 않은 것

- 외부 6DoF full-body tracker의 실물 device serial/pose
- tracker mount offset과 사용자 신체 치수
- live full-body reconstruction 품질
- direct-input 제약 제거 후 Quest live 재시험과 정량 tracking 오차
- 실제 Unitree G1 통신과 동작

위 항목은 통과했다고 주장하지 않는다. 특히 `twist2_vr`에는 Unitree SDK
호출이 없고, CLI와 실행 스크립트는 physical G1 output을 의도적으로
차단한다. `quest-arms`는 전용 localhost Redis와 MuJoCo simulation만 구동한다.

Quest 3 HMD와 좌우 Touch controller는 ALVR/SteamVR에서 실제 serial/role로
열거했고, 새 MuJoCo
OpenVR viewer는 valid HMD pose를 받아 정면 시점으로 recenter했다. SteamVR
compositor 로그에서는 최종 코드 시험의 282 presents, app dropped 0을 확인했다.
2026-10-12 사용자 착용 시험에서 정면 G1 영상, 정상적인 head-tracked 시점,
controller 재정렬과 robot action 분리를 확인했다. 이후 양쪽 Touch grip clutch로
팔과 어깨가 실시간 추종하는 arm-only teleoperation도 확인했다. 이는 Quest
arm-only simulation 경로의 live 기능 검증이며 full-body teleoperation 검증은 아니다.
