# Quest/Tracker → TWIST2 아키텍처

## SparsePoser + 양쪽 정강이 어댑터

2026-10-15 신규 경로는 기존 `sparse-kinematic` 휴리스틱과 분리되어 있다.
사람 미리보기는 **전신 추정과 시각적 검사**만 한다. 별도 G1 실행기는 아래 설명대로
GMR까지 연결하지만 Redis/정책/실제 로봇에 출력을 보내지 않는다.

```text
이미 SpaceCal 적용된 OpenVR standing 8장치
  │ T-pose 장착 보정 + 기준 신체 크기로 변환
  ├─ 허리 / 양발(Toe target) / 머리 / 양손 → 64-frame history
  │                                        ↓
  │                          공식 SparsePoser generator + learned IK
  │                                        ↓ 22개 local joint quaternion
  └─ 양쪽 정강이(Knee target) + 양발 → 별도 다리 IK 보정
                                           ↓
                                FK → OpenVR world 전신 JSONL
                                           ↓
                                데스크톱 사람 골격 미리보기
```

- 원본: [UPC-ViRVIG/SparsePoser](https://github.com/UPC-ViRVIG/SparsePoser),
  commit `6d40a03d71379d59c3b0469f87e5113c6bf0d4d2`, `external/SparsePoser`에 수정 없이 보관.
  코드 MIT, pretrained weights/data CC BY-SA 4.0. 출처/파일 해시는 `SOURCE_PROVENANCE.json`.
- `twist2_vr/sparseposer_backend.py`: 공식 `train`, `RunMotionData`, generator/IK를 직접 호출.
  Unity/TCP를 거치지 않는 래퍼이며 학습 코드나 가중치를 변경하지 않는다.
  입력 `(64,6,7)`은 오래된→최신 순서, 위치와 WXYZ quaternion이다. root만 global이고
  나머지는 root-relative. 출력은 local XYZW 22관절이며 world 변환은 FK로 계산한다.
- `sparseposer_adapter.py`: 8장치 필수, 시리얼/유효성/시간 검사, T-pose 장착 보정,
  60Hz 기본 과거 데이터 재표본화, 원래 OpenVR 공간으로의 역변환을 담당한다.
  64-frame history의 최초 준비 시간은 기본 약 1.05초이며, 매 추론은 최신 프레임을 사용한다.
  관측 공백/추적 끊김 후 history를 다시 채운다. SDK 공간 변환은 다시 적용하지 않는다.
  2026-10-29: `MotionWindow`의 WXYZ 부호를 최초 identity → 직전 수락 입력 기준으로
  정렬하고, Slerp 결과도 저장된 스트림 부호에 고정했다. 기존 창 내부 unroll만으로는
  초기/이동 창의 부호가 뒤집혀 공식 신경망 입력이 달라졌다. `q/-q`의 물리적 회전은
  같지만 네트워크 특징은 같지 않다. 매 프레임 양의 w를 강제하거나 모델을 수정하지 않는다.
  history 초기화 시 부호 기준도 초기화한다. 회귀 테스트는
  `tests/vr_adapter/test_sparseposer_quaternion_stream.py`에 있다.
- `shin_refinement.py`: 양쪽 hip/knee/ankle 회전만 최적화한다. 정강이·발의 위치와 방향을
  모두 비용 함수에 넣고, 모델 예측/기준 자세에는 약한 prior를 둔다. 뼈 길이, root,
  척추/팔 회전은 이 추가 보정에서 그대로 유지된다. 모델 자체의 상체 추정은 여전히 입력에 반응한다.
  측정하지 않은 관절은 추정값이며, 인체/로봇의 물리적 실행 가능성을 보장하지 않는다.
- 이동 범위를 잘라내는 임의의 팔/다리 clipping은 추가하지 않았다. 기본 3cm / 0.20rad
  허용 오차는 **결과 품질 표시 기준**이며 움직임을 자르는 관절 한계가 아니다.
  수렴 실패/오차 초과 결과도 검사 목적으로 저장하되 `fit_accepted=false`,
  `robot_eligible=false`를 기록한다. 누락 장치를 정상 자세로 조작해 채우지 않는다.
- T-pose는 공식 `male_170_85.txt`의 비항등 reference rotation을 그대로 사용한다.
  사람 키에 따른 균일 스케일만 적용하며 개인별 뼈 길이를 측정/학습하지 않는다.
  HMD 중심을 head joint 근처로 가정해 센서→joint 변환을 추정한다.
  정강이 센서 회전은 lower-leg 방향 제약이다. 센서 중심을 무릎 관절 중심이라고
  간주하지 않고 장착 변환을 거친다. 따라서 장착 위치 변경 후 재보정이 필요하다.
- OpenVR의 실제 추적 상태를 받아들이며 SteamVR/ALVR/VIVE를 시작·재시작하거나
  프로필을 수정하지 않는다. 저장 SpaceCal 프로필 해시 변경 시 실행을 거부한다.
  런타임 recenter처럼 파일에 나타나지 않는 공간 변경은 사용자 재확인이 필요하다.

검증: 공식 모델 실제 로딩/추론, PyMotion FK 일치, room yaw/키/장착 회전 roundtrip,
정강이별 영향, 상체/뼈 길이 보존, 좌우 오매핑/중복 시리얼 거부, 추적 손실/history 초기화.
테스트는 `tests/vr_adapter/test_sparseposer*.py`, `test_shin_refinement.py`.
합성 입력 검증은 실제 사용자 동작 정확도나 로봇 안정성 검증을 대신하지 않는다.
기존 22:06 차렷 캡처는 입력 형식 검사에는 사용 가능하지만 T-pose 보정으로 사용하지 않는다.

### G1 운동학적 미리보기

`scripts/run_sparseposer_g1.sh` → `twist2_vr/sparseposer_g1_preview.py`는
위 추정기를 공식 GMR의 `bvh_xsens → unitree_g1`에 연결한다.
`sparseposer_gmr.py`가 모델 스케일을 실제 미터로 복원하고 BVH 축을 `[Z,X,Y]`로
변환한다. world/local 회전 양쪽에 basis 변환을 적용하고, 팔의 T-pose/N-pose
segment 축 차이를 보정한다. FootMod에는 공식 Xsens 경로처럼 ankle 위치·회전을 쓴다.
GMR 소스/IK 설정은 수정하지 않는다. 기본 `offset_to_ground=True` 때문에 최저 ankle이
매 프레임 기준 높이에 정렬되며, 절대 점프 높이 보존을 주장하지 않는다.

GMR qpos36과 XML 관절 순서를 확인한 뒤 `mj_forward`만 호출한다. `mj_step`,
TWIST2 ONNX 정책, Redis/DDS 전송은 사용하지 않는다. 품질 기준 미달의 유한 자세도
경고를 붙여 표시하지만 로봇 명령으로 사용하지 않는다. 입력 소실 시 마지막 자세를 유지한다.

- 입력/추정/GMR: 별도 프로세스, OpenVR Background. raw/body/G1 JSONL 저장.
- 화면: OpenVR Scene + 기존 MuJoCo front viewer, Quest와 PC 미러 표시.
  입력과 Scene은 pyopenvr singleton 충돌을 피하도록 같은 프로세스에 두지 않는다.
- 선택 `--record`: 또 다른 프로세스에서 EGL 정면 카메라와 MP4 인코딩.
  기본은 OFF이며 사용자가 PC 화면을 수동 녹화한다. 영상 프로세스는 VR 설정을 만지지 않는다.
  켰을 때 출력은 `g1_front.mp4`, `video_frames.jsonl`, `video_recording_stats.json`이다.
  프레임 메타데이터에 입력 시각·오차·중복/누락 슬롯을 기록한다. 장시간 렌더 지연은
  무한 따라잡기 없이 슬롯을 생략하므로 동영상 시간과 실제 시간 차이는 메타데이터로 확인한다.

최근 실제 입력 120개로 G1 변환과 영상 생성/디코딩을 확인했다. 왼쪽 다리 fit 오류는
남아 있으며, 이 결과는 동적 균형 또는 실시간 VR 동시 검증을 대신하지 않는다.
기존 `xrobot` 경로에 8개 센서나 22개 관절의 이름만 바꿔 넣지 않는다.
실행 명령은 [빠른 시작](VR_QUICKSTART.md)의 맨 위에 있다.

## 설계 원칙

공식 TWIST2 low-level controller를 수정하지 않는다. Vive tracker를 포함하는 전신 경로는 Quest/SteamVR 입력을 공식 PICO `XRobotStreamer`와 동일한 **전신 anatomical pose 경계**로 변환한 뒤 공식 GMR 경로를 재사용한다.

첫 실물 입력 검증용 `quest-arms` 프로필은 전신 동작을 추정하지 않는다. HMD와 Touch controller 두 개만으로 pelvis나 다리를 발명하는 대신 root·다리·허리·손목 관절을 검증된 중립값에 고정하고, controller의 **위치 변화량만** G1 양쪽 shoulder 3축과 elbow IK로 변환한다. 이 프로필은 로컬 MuJoCo simulation 전용이다.

기존 상위 프로젝트의 `alvr_teleop_adapter.py`가 만드는 head-relative wrist pose는 TWIST2에 사용하지 않는다. 그 값은 Unitree `xr_teleoperate` 팔 IK용이며, pelvis/feet가 있는 world-frame skeleton이 아니다.

## 데이터 흐름

```text
Mock / JSONL Replay / OpenVR Live
                 │ TrackerFrame (meters, xyzw, validity, timestamp, serial)
                 ▼
           PoseNormalizer
      OpenVR +X right,+Y up,-Z forward
         → +X forward,+Y left,+Z up
                 │
                 ▼
        per-tracker mount offset
                 │
                 ▼
 direct pose pass + validity/stale checks
                 │
                 ▼
       neutral yaw/origin calibration
                 │
                 ▼
  strict-direct OR sparse-kinematic reconstruction
                 │ XRobot joint dict + measured/estimated provenance
                 ▼
       Official GMR (xrobot→unitree_g1)
                 │ MuJoCo qpos[36] = root7(wxyz)+q29
                 ▼
      Twist2ReferenceEncoder
                 │ mimic[35]
                 ▼
 dry-run sink (default) / guarded localhost Redis simulation sink
                 │
                 ▼
official TWIST2 ONNX student → residual q29 → PD → MuJoCo
```

### Quest controller-only arm 경로

```text
Quest 3 HMD + physical left/right Touch controller
                 │ raw OpenVR device pose + SteamVR actions
                 │ (grip/buttons/sticks, aim pose는 identity 확인용)
                 ▼
 axis normalization + direct pose validation
                 ▼
       QuestUpperBodyReconstructor
  startup HMD yaw 1회 고정 + clutch hand displacement
                 │ fixed canonical body + two wrist deltas
                 ▼
          QuestArmIKRetargeter
      shoulder pitch/roll/yaw + elbow only
                 │ fixed root/legs/waist/wrist DoFs, mimic[35]
                 ▼
 isolated localhost Redis → TWIST2 ONNX controller → MuJoCo/Quest view
```

controller `PoseSample`은 SteamVR `/pose/tip`이 아니라 OpenVR의 raw tracked
controller origin을 사용한다. Tip pose에는 controller-local translation offset이
있으므로 이를 손목 위치로 사용하면 controller를 제자리에서 회전하기만 해도 tip이
원을 그리며 손목 목표가 이동한다. Action aim pose는 action origin이 선택된 물리
controller와 같은 device인지 검증하는 용도로만 보존한다.

중립 보정 중에는 양쪽 grip을 놓고 HMD와 controller를 고정한다. 설정 기본값은
15 sample, 최소 0.25초, controller 위치 표준편차 최대 1.5 cm다. 이때 HMD yaw를
`fixed_room_to_body`로 한 번 저장하며 이후 프레임의 HMD 회전이나 숙임을 arm
target에 다시 적용하지 않는다. 따라서 로봇을 보려고 머리를 돌리거나 숙여도 손이
따라 움직이지 않는다.

보정 이후 양쪽 grip을 함께 누른 순간의 controller 위치와 현재 robot arm delta를
새 clutch 기준으로 저장한다. 누르고 있는 동안의 translation delta만 고정된 startup
yaw frame으로 변환한다. 한쪽 grip이라도 놓으면 양팔 목표를 현재 값에 유지하고,
다시 양쪽 grip을 누르면 그 위치에서 새 clutch가 시작되므로 controller를 편한
위치로 재배치할 수 있다. Quest arm direct-input 프로필에서는 pose smoothing,
인위적인 속도-jump 차단, Cartesian/joint 속도 제한, 관절 한계 margin, neutral
regularization, IK residual projection을 적용하지 않는다.

controller translation의 robot-frame 목표에는 축별 X/Y/Z cap을 적용하지 않는다.
grip clutch 지점에서 시작한 translation delta를 1:1로 IK에 넣는다. 따라서 사람의
대각선 팔 궤적을 직육면체 경계에서 자르거나 입력을 축소하지 않는다. IK는 MuJoCo
G1 XML의 실제 관절 가동범위 안에서 가장 가까운 pose를 풀며, 도달 불가능한 목표는
축소하지 않고 residual로 보고한다.

공식 offline example 경로는 VR 경로와 별도로 `OfficialExampleMotionStream`이
공식 `MotionLib`의 pose/velocity interpolation을 재사용해 같은 35D Redis 경계로
들어간다. 이는 pretrained controller 기준선 검증용이며 tracker reconstruction을
검증하는 경로는 아니다.

### 공식 TWIST2 controller의 한계와 입력 shaping 구분

공식 low-level 경로는 Redis의 35D reference를 Cartesian workspace cap이나 reference
slew 없이 ONNX observation에 넣는다. 따라서 Quest 팔이 특정 궤적에서 멈춘 원인은
공식 reference 전처리 제한이 아니었다. 공식 코드의 ONNX output clip(`±10`),
`action_scale=0.5`, PD torque clip과 MuJoCo XML joint range는 학습 때 사용한 action
계약 및 G1 물리 모델이다. 이것들은 손목 목표를 축소하는 입력 shaping이 아니므로
변경하지 않았다. 제거하면 pretrained policy의 계약을 깨고 simulation을 불안정하게
만들 뿐 Cartesian 도달범위를 늘리지 않는다.

## 내부 형식

### `PoseSample`

- `device_id`: OpenVR serial. Device index는 재연결 시 바뀔 수 있어 identity로 사용하지 않는다.
- `role`: `Pelvis`, `Left_Foot`, `Left_Wrist` 같은 anatomical role.
- `position_m[3]`: meters.
- `orientation_xyzw[4]`: normalized quaternion.
- `timestamp_s`, `valid`, `tracking_state`, metadata.

내부 quaternion은 항상 `xyzw`다. 공식 GMR dictionary를 만들 때만 `[w,x,y,z]`로 변환한다. MuJoCo free joint qpos 역시 `wxyz`임을 adapter에서 명시적으로 처리한다.

### GMR 입력

공식 `xrobot_to_g1.json`이 요구하는 14개 body는 다음과 같다.

```text
Pelvis
Left/Right_Hip, Left/Right_Knee, Left/Right_Foot
Spine3
Left/Right_Shoulder, Left/Right_Elbow, Left/Right_Wrist
```

각 값은 `position_xyz_m`과 `quaternion_wxyz`다. neck을 나중에 연결하려면 `Head`도 필요하다.

### TWIST2 35D reference

```text
[0:2]   root local vx, vy
[2]     root z
[3:5]   root roll, pitch
[5]     root local yaw angular velocity
[6:35]  G1 q29
```

Redis simulation key는 공식 코드와 동일하다.

- `action_body_unitree_g1_with_hands`: 35
- `action_hand_left_unitree_g1_with_hands`: 7
- `action_hand_right_unitree_g1_with_hands`: 7
- `action_neck_unitree_g1_with_hands`: 2
- `t_action`: Unix milliseconds

## 좌표계와 calibration

`config/coordinates.openvr.json`의 기본 basis는 다음과 같다.

```text
OpenVR: +X right, +Y up, -Z forward
target: +X forward, +Y left, +Z up

p_target = C p_openvr
R_target = C R_openvr C^T
```

tracker pose는 anatomical joint pose와 같지 않다. 변환 순서는 다음과 같다.

```text
T_world_joint = T_neutral_tracking × T_tracking_device × T_device_joint
```

`T_device_joint`는 tracker 장착 방향/위치이며 `coordinates.openvr.json`에 저장한다. 현재 HMD/컨트롤러 offset은 0으로 두었고 실물 측정 전에는 정확한 wrist/head frame이라고 주장하지 않는다.

`calibration.live.json`은 기본적으로 `calibrated:false`다. live pipeline은 이 상태에서 실패한다. `live-calibrate`는 room origin/yaw만 결정하며 mount offset이나 관절 위치를 자동 발명하지 않는다.

## Reconstruction 모드

### Strict direct

GMR의 14개 target이 모두 valid tracker/외부 skeleton solver 출력으로 들어올 때만 성공한다. 누락된 role이 있으면 fail closed 한다.

### Sparse kinematic estimate

최소 `Head`, `Pelvis`, 양손목, 양발을 요구한다. 누락된 hips/spine/shoulders는 neutral skeleton에서, elbows/knees는 two-bone IK로 추정하고 provenance에 `estimated_*`를 표시한다.

이 방법은 full-body 측정이 아니다. 특히 관절 orientation과 limb twist가 underdetermined다. 실제 테스트에서 단순 sparse mock은 정책을 넘어뜨렸으므로 현재 simulation experiment 용도다. 안정적인 live full-body teleop에는 다음 중 하나가 필요하다.

1. 더 많은 direct trackers와 정확한 anatomical frames
2. temporal full-body pose solver
3. Quest sparse observation을 입력으로 teacher를 distill한 student policy/prior

## Provider

- `MockVRProvider`: full FK-derived fixture 또는 명시적 sparse fixture.
- `JSONLReplayProvider`: live와 동일한 contract로 timestamped JSONL replay.
- `OpenVRProvider`: SteamVR background API에서 HMD/controller/GenericTracker를 읽는다.
- `ViveUltimateLinuxProvider`: VIVE Hub Linux의 `/tmp/vut.sock` VUT protocol v1에서
  Ultimate pose/status를 받고 stable serial로 역할을 지정한다. `Ready` 외 상태,
  stale pose, 빈 serial, 재연결 이전 세대 pose는 fail closed한다.
- `QuestViveHybridProvider`: rigid fixture로 구한 `VIVE world → OpenVR standing`
  transform을 적용한 뒤 Quest의 Head/양손목과 VIVE의 pelvis/양무릎/양발을 합친다.

Linux 직접 연결의 흐름은 다음과 같다.

```text
Quest/Touch → ALVR → Linux SteamVR/OpenVR ─┐
                                           ├→ calibrated hybrid TrackerFrame
Ultimate 5 → Linux dongle → VIVE Hub/VUT ─┘
```

두 tracking map은 같은 방에 있어도 원점이 자동으로 같아지지 않는다. tracker 하나와
Quest controller 하나를 rigid fixture로 묶어 여러 축으로 움직인 sample에서 hand-eye
transform을 구하며, 어느 한쪽 map/origin이 바뀌면 재정렬한다.

`OpenVRProvider`는 HMD와 controller index를 매 프레임 다시 찾는다. 좌우 controller는
선택한 HMD와 `tracking_system`이 같아야 하며, ALVR synthetic hand-tracking device는
제외하고 Touch를 우선하는 물리 controller를 선택한다. SteamVR action binding은 grip, trigger,
buttons, stick을 `TrackerFrame.controls[left/right]`에 넣고, 누락된 action channel만
같은 device의 legacy controller state로 보충한다. Action origin이 선택한 controller
index와 다르면 해당 action/aim pose를 거부한다. 재연결 또는 index 재사용은
`connection_generation`과 serial/model metadata로 노출되어 진행 중인 보정을
무효화한다.

OpenVR GenericTracker에는 자동 anatomical role이 없으므로 stable serial→role
mapping이 필수다. `config/tracker_mapping.vive_fullbody.example.json`에서 Ultimate
Tracker serial을 최소 `Pelvis`, `Left_Foot`, `Right_Foot`에 지정한다. HMD는 `Head`,
좌우 Touch는 `Left_Wrist`/`Right_Wrist`로 자동 연결되며 explicit serial mapping이
자동 role보다 우선한다. 이 future full-body 경로는 mount transform, SteamVR standing
space 정렬, neutral calibration 및 recorded replay 안정성을 실물로 검증하기 전에는
dry-run만 허용한다. 서로 다른 PC/runtime에서 tracker pose를 relay한다면 별도의
좌표 transform과 timestamp/clock 동기화가 추가로 필요하다.

## Controller 입력과 상태 전이

`quest-arms`의 기본 입력은 다음과 같다.

| 입력 | 동작 |
| --- | --- |
| 양쪽 grip 완전 release | 최초 중립 보정 또는 arm 목표 hold |
| 양쪽 grip 지속 press | clutch 활성화; controller translation으로 양팔 이동 |
| 한쪽 grip release | 즉시 양팔 목표 hold, 재배치 가능 |
| 양쪽 조이스틱 동시 클릭 | arm input E-stop latch |
| grip을 놓고 `Y+B` 동시 입력 | E-stop 해제 후 중립 재보정 |
| 오른쪽 조이스틱 단독 클릭 | VR 정면 시점 재정렬 |
| desktop mirror의 `R` | 동일한 VR 시점 재정렬 fallback |

Grip에는 press/release hysteresis가 적용된다(기본 0.65/0.35). 시작, tracking loss,
controller reconnect 또는 E-stop 해제 후에는 grip release와 안정된 중립 보정을 다시
통과해야 한다. 양쪽 조이스틱 E-stop에는 오른쪽 클릭도 포함되므로 view가 함께
재정렬될 수 있지만 arm E-stop latch가 우선하며, 위 reset 절차 전에는 arm 입력이
재개되지 않는다.

### Quest arm telemetry

live 로그는 매 frame을 출력하지 않고 상태가 바뀔 때만 IK/tracking event를 출력한다.

- `[TRACKING_LOST]`: 유효하지 않거나 사라진 role을 `Head`, `Left_Wrist`,
  `Right_Wrist` 이름으로 표시한다. 이때 clutch를 해제하고 마지막 적용 arm
  목표를 hold하며, 추적 복구 후 release와 중립 보정을 다시 요구한다.
- 주기 상태 줄의 `IK=L=...mm R=...mm`는 각 손목의 요청 위치와 실제 IK 결과 사이의
  residual을 표시한다. Quest arm 프로필은 목표를 축소 투영하지 않으므로
  `scale`, `[IK_PROJECTED]`, `[IK_OK]`를 출력하지 않는다.

따라서 팔이 더 움직이지 않으면서 residual이 커지면 G1 arm의 실제 관절 가동범위가
원인이고, `[TRACKING_LOST]`이면 OpenVR controller pose가 무효가 되어 hold한 것이다.

## 안전 경계

- 기본 sink는 `DryRunSink`다.
- Redis sink는 localhost만 허용하고 `--publish-sim` 및 `TWIST2_ALLOW_SIM_REDIS=1`을 동시에 요구한다.
- live hardware 입력 중 Redis publish가 허용된 것은 `quest-arms` 프로필뿐이다. `scripts/run_quest_arms_vr.sh`는 외부 Redis와 분리된 `127.0.0.1:16379`를 시작하고 MuJoCo server도 같은 port에만 연결한다.
- 일반 `live`와 future `vive-fullbody`의 `--publish-sim`은 CLI에서 명시적으로 차단한다. 이 두 경로는 calibration과 replay stability 검증 전까지 dry-run 전용이다. Mock/replay 및 공식 example publisher는 별도의 offline simulation validation 경로다.
- 일반 전신 pipeline도 smoothing과 인위적인 속도-jump 차단 없이 pose를 직접 전달한다. stale/non-monotonic/NaN/Inf/필수 tracker 누락/root height 위반 시에는 stand reference를 1회 쓰고 disarm한다. `quest-arms`는 HMD/controller tracking 또는 action 입력이 사라지면 clutch를 해제하고 마지막 적용 arm target을 hold하며, 재연결 후 중립 보정을 다시 요구한다.
- Redis 공식 서버 자체에는 TTL이나 stale rejection이 없으므로 adapter watchdog이 필수다.
- quest-arms MuJoCo server는 `t_action` timeout도 검사하며 launcher 기본값은 300 ms다.
- 물리 로봇 Unitree SDK는 이 패키지에서 import하지 않으며, physical G1 output은 모든 프로필에서 차단되어 있다.

## 파일 구조

```text
twist2_vr/
  providers/       mock, JSONL replay, OpenVR
  openvr_actions.py SteamVR action + matching-device legacy fallback
  upperbody.py     Quest-only fixed-body clutch state machine
  models.py        typed contracts
  math3d.py        quaternion/rotation helpers
  normalization.py
  calibration.py
  filtering.py
  reconstruction.py
  retargeting.py   official GMR + Quest arm IK adapters + 35D encoder
  controller.py    safety + dry-run/Redis simulation sink
  pipeline.py
  cli.py
config/            mapping, calibration, filtering, modes
tests/vr_adapter/  hardware-independent tests
scripts/           setup/start/validation wrappers
```

## MuJoCo → Quest stereo 출력

`twist2_vr/mujoco_openvr.py`는 공식 low-level controller의 물리 loop와 분리된
render thread에서 다음 경로를 실행한다.

```text
latest MuJoCo qpos snapshot
        ↓
separate MjData + mjvScene.camera[0:2]
        ↓
side-by-side OpenGL texture
        ↓ OpenVR compositor
SteamVR → ALVR → Quest 3
```

첫 valid HMD pose를 neutral로 저장하고, virtual neutral head를 G1 정면 2.2 m에
배치한다. 실제 head delta만 그 anchor에 곱하므로 시작할 때 사용자가 어느 room
방향을 보고 있었는지와 무관하게 G1이 정면에 놓인다. robot state는 50 Hz로
latest-only copy되고 HMD pose/render submit은 compositor cadence로 동작하므로
`waitGetPoses`가 1 kHz physics loop를 막지 않는다.

정면 anchor의 robot root 위치와 heading은 시작할 때 고정한다. 따라서 로봇이
이동하거나 넘어져도 카메라가 함께 끌려가지 않고 제3자 관찰 시점에서 움직임을
확인할 수 있다. 오른쪽 조이스틱 클릭 또는 `R`은 현재 HMD pose만 이 고정
anchor에 다시 맞춘다. 컨트롤러는 매 frame role/serial로 다시 찾고, release를
관측한 뒤의 press edge만 허용하므로 sleep/reconnect 중 held 버튼은 무시한다.
