# 구현 상태

## 완료 및 실제 검증

| 항목 | 상태 | 검증 |
|---|---|---|
| 공식 TWIST2 clone/commit 기록 | 완료 | commit/hash 기록 |
| 공식 GMR 고정 clone | 완료 | online IK import/실행 |
| pretrained ONNX 확인 | 완료 | checker + CPU `1432→29` inference |
| G1 joint order | 완료 | deploy XML = GMR XML = 29 |
| 예제 모션 10개 | 완료 | trusted pickle schema/finite 검사 |
| 전용 Python 3.10 환경 | 완료 | `.conda-vr` |
| Mock provider | 완료 | full/sparse fixture |
| JSONL record/replay | 완료 | round-trip test |
| OpenVR provider | 구현 완료 | Quest 3 HMD/좌우 Touch serial·role 실제 열거; GenericTracker 미검증 |
| Quest Touch action/deadman 입력 | 실제 검증 | action/legacy fallback, release-before-arm, reconnect, E-stop 자동 테스트; 양쪽 grip clutch live teleop 확인 |
| MuJoCo OpenVR stereo viewer | 완료 | 최종 코드 Quest/ALVR 282 presents, app dropped 0 |
| 정면 3인칭 HMD rig | 완료 | 실제 착용 영상·HMD 추적·controller recenter 확인 |
| axis/quaternion normalization | 완료 | unit tests |
| tracker mount offset | 완료 | unit tests; 실측값 미입력 |
| neutral calibration | 완료 | synthetic test + Quest/Touch live arm teleop |
| direct pose validity/freshness gate | 완료 | smoothing·speed rejection 제거, stale/non-monotonic/reconnect unit tests |
| strict reconstruction | 완료 | missing role fail-closed test |
| sparse reconstruction interface | 완료 | provenance/two-bone tests; balance 불합격 |
| 공식 GMR adapter | 완료 | mock frame → finite qpos36/reference35 |
| 35D TWIST2 encoder | 완료 | velocity/quaternion sign tests |
| dry-run sink | 완료 | end-to-end test |
| guarded localhost Redis sim sink | 완료 | fake Redis + actual local Redis |
| Redis action watchdog | 완료 | 35D shape/finite/`t_action` freshness 검사, stale/malformed 시 neutral fallback |
| Quest controller arm-only profile | 실제 검증 | Quest 3/Touch로 팔·어깨 live teleop; root/legs/waist/wrists lock, direct 1:1 입력·clutch·IK·reconnect 보호 |
| IK/tracking telemetry | 완료 | tracking-loss role, 좌우 IK residual 로그 + 자동 테스트 |
| Quest arm position IK | 완료 | 제거된 축별 cap을 모두 넘는 800-frame 궤적 + 공식 controller 13초 headless 안정성 통과 |
| 공식 example motion → controller | 완료 | `walk_003`, 5초 headless 안정성 통과 |
| 공식 low-level MuJoCo controller | 완료 | stand/example/full mock 각각 5초 안정성 통과 |
| Linux VIVE Hub VUT provider/공간 정렬 | 구현 완료 | fake `SOCK_SEQPACKET`, 상태·stale·slot 재사용 fail-closed, synthetic hand-eye 검증; 실물 미검증 |
| 자동 테스트 | 완료 | 61 passed |

## Mock-only

- `FULL_TARGET_POSES`는 G1 neutral FK와 공식 GMR offset을 역변환한 합성 full tracker fixture다.
- full mock → GMR → ONNX → MuJoCo 성공은 software contract 검증이다.
- Quest arm IK headless 결과는 합성 wrist trajectory로 안정성을 정량 검증한 것이다. 별도로 Quest/Touch live 팔·어깨 teleop도 사용자가 확인했다.
- 사용자 신체와 Quest tracking의 정량 오차, full-body pose reconstruction 품질까지 검증한 것은 아니다.

## 구현했고 Quest에서 runtime 확인

- ALVR/SteamVR에서 Quest 3 HMD 및 좌우 Touch controller serial enumeration
- valid HMD pose를 기준으로 front-view recenter
- MuJoCo side-by-side stereo texture의 OpenVR compositor 제출
- 오른쪽 조이스틱 클릭 및 PC `R` 재정렬, `Esc` 종료

SteamVR compositor 로그에서 최종 코드 시험 동안 282 presents와 app dropped 0을
확인했다. 2026-10-12 사용자 착용 시험에서 G1 정면 표시, 자연스러운 HMD 시점,
오른쪽 조이스틱 재정렬, HMD 움직임과 robot action의 분리를 육안 확인했다.
이후 양쪽 grip clutch를 사용한 live arm-only 시험에서 사용자의 팔과 어깨
움직임을 로봇이 추종함을 확인했다. 첫 시험에서 수직 wrist target이
기존 `+0.20 m` 제한에 도달하는 현상을 관찰했고, 후속 검증 후 상향을 `+0.30 m`로
확장했다. 다음 live 시험에서 X/Y/Z 축별 cap이 사람의 대각선 팔 궤적을 잘라내는
것을 확인했다. 현재는 해당 cap뿐 아니라 Quest arm 경로의 smoothing, gain,
Cartesian/joint slew, 관절 margin, neutral regularization, IK projection까지 제거했다.
이 direct-input 변경 후 Quest live 재시험은 아직 필요하다.

## Quest arm-only simulation 검증

`QuestArmIKRetargeter`는 controller translation만 shoulder pitch/roll/yaw와 elbow에
적용하고, root·legs·waist·wrist DoF를 stand reference로 고정한다. 합성 순수 수직
`+0.60 m` 한 팔과 실제 Quest 로그의 peak 방향을 재현한 양팔 trajectory를 50 Hz로
800 frame 공급하고 공식 ONNX controller와 MuJoCo를 13초 실시간 실행한 결과다.

| 지표 | 값 |
|---|---:|
| max requested X/Y/Z delta | 0.238 / 0.213 / 0.600 m |
| 제거된 X/Y/Z cap 모두 초과 | true |
| max reference joint step | 0.451114 rad |
| max IK residual | 0.006345 m |
| physical joint-limit violation | 0 rad |
| protected DoF drift | `5.96e-9` rad |
| root height 최소/최대 | 0.779882 / 0.792998 m |
| 최대 base tilt | 4.9416° |
| 최대 절대 torque | 33.4907 |
| 결과 | 통과 |

live 운영 중에는 주기 상태 줄의 `IK=L=...mm R=...mm`가 요청 손목과 실제 IK 결과
사이의 residual을 보고한다. tracking loss는 무효한 HMD/컨트롤러 role을 함께
표시한다. 입력 shaping이나 software projection은 적용하지 않는다.

## 구현했지만 추가 tracker 하드웨어 확인 필요

- Linux VIVE Hub 설치, 동글과 tracker 5개 pair/map/Ready
- `/tmp/vut.sock` 실물 serial/status/120 Hz pose
- Quest/VIVE rigid-fixture 공간 정렬의 실측 RMSE
- serial→body role mapping과 5개 tracker 착용
- room yaw/origin calibration
- 신체별 tracker mount transform
- tracker loss/reconnect 동작
- Vive Ultimate Tracker 실물 serial/pose 및 full-body profile 착용 검증
- 기존 대안 경로의 SteamVR GenericTracker 노출은 별도 미검증

## 의도적으로 차단

- full-body `live`/`vive-fullbody`/`vive-linux-fullbody` 입력의 Redis simulation publish
- 모든 물리 G1 통신
- Unitree SDK import/명령
- 실제 robot server 실행

`quest-arms`만 양쪽 grip clutch, reconnect 재보정, E-stop, tracking freshness 검사와
전용 localhost Redis port를 사용해 MuJoCo simulation publish를 허용한다.
full-body publish와 physical G1 output은 계속 fail-closed다.

## 알려진 blocker

1. 전체 TWIST2 training dataset이 없고 YAML root가 저자 내부 경로다.
2. legacy Isaac Gym 및 Python 3.8 `twist2` training 환경이 없다.
3. `nvcc`/CUDA toolkit이 없어 training extension build를 바로 검증할 수 없다.
4. PICO XRoboToolkit SDK는 설치하지 않았다. 이번 경로는 Quest/OpenVR다.
5. sparse body pose는 underdetermined이며 현재 naive solver는 balance-safe하지 않다.
6. VIVE Hub Linux 실물 연결, serial mapping, mount calibration과 full-body replay 안정성은 미검증이다.

## 공식 코드에서 발견한 주의점

- `sim2sim.sh`는 100Hz지만 학습/모션은 50Hz다. 새 wrapper는 50Hz 사용.
- official sim history는 zero로 시작하지만 training reset은 current obs로 채운다.
- upstream official Redis GET은 key 누락/stale을 검사하지 않았다. 현재 local simulation consumer는 35D shape/finite/`t_action` watchdog으로 neutral fallback한다.
- official emergency-stop은 현재 없는 과거 파일명을 kill하려고 한다.
- `eval.sh`와 전체 dataset YAML에는 저자 머신 절대 경로가 남아 있다.

## 바로 실행할 명령

```bash
cd /home/railab/Desktop/vr_teleop/TWIST2

# 테스트
bash scripts/run_vr_tests.sh

# 자산/ONNX 검증
bash scripts/validate_assets.sh

# 공식 GMR을 쓰는 dry-run mock
bash scripts/run_mock.sh --frames 100

# mock 기록
bash scripts/run_mock.sh --frames 100 --record runtime/mock.jsonl

# replay
bash scripts/run_replay.sh runtime/mock.jsonl --realtime

# 한 터미널 headless controller 검증
bash scripts/run_headless_validation.sh

# Quest arm-only synthetic trajectory → controller headless 검증
bash scripts/run_quest_arm_headless_validation.sh

# 공식 example motion → pretrained controller 검증
bash scripts/run_example_motion_validation.sh

# 한 터미널 GUI mock simulation
bash scripts/run_mock_sim.sh

# Quest/ALVR에서 정면 3인칭 MuJoCo stereo mock simulation
bash scripts/run_mock_vr.sh

# Quest HMD + 양손 Touch arm-only MuJoCo teleop (실물 G1 연결 없음)
bash scripts/run_quest_arms_vr.sh

# Quest/Vive/full-body 장치 inventory
bash scripts/run_live_inventory.sh

# Vive full-body dry-run (실물 mapping/calibration 완료 후; simulation publish 차단)
bash scripts/run_vive_fullbody_dry.sh

# Linux 동글/VIVE Hub 직접 연결 설치 및 전체 순서
# VIVE_LINUX_QUICKSTART.md 참조
```
