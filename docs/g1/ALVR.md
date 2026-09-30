# Quest 3 · ALVR 입력과 G1 시뮬레이션 연결

이 경로는 Quest 브라우저를 사용하지 않는다. ALVR에 연결된 SteamVR에서
머리·양손 컨트롤러를 읽고, 학습한 G1 정책을 실행하는 시뮬레이터에
세 점의 목표 위치와 이동 명령을 전달한다. 기존 `quest3_probe.py`는 별도의
브라우저 연결 진단 프로그램으로 유지된다.

```text
Quest 3 + Touch 컨트롤러
    ⇅ ALVR
SteamVR ── Isaac Sim XR 렌더링 → Quest 영상
    └─ OpenVR Background / IVRInput
          ↓ 좌표 변환 · 기준 자세 보정 · 동작 허용 스위치
       UDP 127.0.0.1:8765
          ↓
       학습된 정책 → 시뮬레이션 G1 관절 제어
```

입력 프로그램은 `VRApplication_Background`로 연결하므로 SteamVR를 자동으로
켜지 않고, 영상 렌더링 앱의 compositor를 점유하지 않는다.
[Valve의 애플리케이션 종류 설명](https://github.com/ValveSoftware/openvr/wiki/API-Documentation#initialization-and-cleanup)을 참고한다.
실제 VR 영상은 Isaac Sim 쪽 XR 실행 설정이 담당한다.
입력 프로그램만 실행하면 영상이 생성되지는 않는다.

## 1. 공용 서버에 입력 환경 설치

SteamVR와 ALVR 서버가 **같은 서버의 같은 그래픽 사용자 세션**에서 실행되어야
한다. 다른 서버에서 `openvr` 라이브러리만 설치하는 것으로 원격 SteamVR에
접속할 수는 없다. 기존 ALVR·SteamVR 설치와 설정을 그대로 사용한다.

저장소 최상위에서 다음을 실행한다. Isaac 환경을 변경하지 않는 별도 Python
환경이며 GPU 라이브러리도 설치하지 않는다.

```bash
python3 -m venv .venv-alvr
.venv-alvr/bin/python -m pip install -r g1_teleop/vr/requirements.txt
```

Python 3.10에서 `numpy==1.26.4`, `openvr==2.12.1401`로 검증했다.
`openvr`는 [pyopenvr 프로젝트](https://github.com/cmbruns/pyopenvr)의 Python 바인딩이다.

SteamVR에 Quest 머리와 두 컨트롤러가 모두 표시되는지 확인한다. 제공한 기본
바인딩은 SteamVR의 `oculus_touch` 컨트롤러 프로필용이다. ALVR에서 다른
컨트롤러로 에뮬레이션하고 있다면 해당 설정을 확인하거나 맞는 action binding을
추가하고 `--manifest`로 지정해야 한다. 프로그램이 기존 ALVR 설정을 바꾸지는 않는다.

## 2. 시뮬레이터와 입력 프로그램 실행

영상도 입력과 **같은 SteamVR 런타임**을 사용해야 한다. `--xr`로 Isaac의 OpenXR
확장을 켜는 것만으로 SteamVR이 선택되는 것은 아니다. 서버에서 다른 OpenXR
런타임을 사용한 적이 있다면 특히 확인한다. Isaac을 실행할 터미널에서 다음을
설정한다. 아래는 일반적인 Steam 설치 위치이며 실제 서버 위치로 바꿀 수 있다.

```bash
G1_STEAMVR_ROOT="$HOME/.local/share/Steam/steamapps/common/SteamVR"
# 별도 Steam 라이브러리라면 예: G1_STEAMVR_ROOT="/mnt/SteamLibrary/steamapps/common/SteamVR"
if [ -f "$G1_STEAMVR_ROOT/steamxr_linux64.json" ]; then
  export XR_RUNTIME_JSON="$G1_STEAMVR_ROOT/steamxr_linux64.json"
  printf 'Isaac OpenXR runtime: %s\n' "$XR_RUNTIME_JSON"
else
  printf 'SteamVR runtime file missing; correct G1_STEAMVR_ROOT before launching Isaac.\n' >&2
fi
```

파일이 없다는 메시지가 나오면 경로를 고쳐 검사를 통과한 뒤 실행한다.
위치를 모르면 실제 Steam 라이브러리 아래에서
`rg --files /실제/SteamLibrary -g steamxr_linux64.json`으로 찾을 수 있다.
문제 진단 시 같은 터미널에서 `export XR_LOADER_DEBUG=info`를 추가하면 런타임
탐색 로그를 볼 수 있다. 이 환경 변수는 해당 셸에서 실행하는 Isaac 프로세스에
적용되며, 시스템 기본 런타임·SteamVR·ALVR 설정 파일을 수정하지 않는다.
[Khronos 공식 런타임 선택 규칙](https://registry.khronos.org/OpenXR/specs/1.1/loader.html#overriding-the-default-runtime-usage)

먼저 G1 실행 안내에 따라 체크포인트를 로드한 시뮬레이터를 `teleop` 모드로
실행한다. UDP 기본 수신 주소는 `127.0.0.1:8765`이다. 시뮬레이터와 입력
프로그램은 서로 다른 터미널에서 실행한다.

그다음 입력 터미널에서 **학습 결과에 저장된 기준 좌표 파일**을 지정한다.
아래 `RUN_DIR`를 실제 모델 실행 폴더로 바꾼다.

```bash
RUN_DIR="outputs/g1/실제_학습_폴더"
.venv-alvr/bin/python scripts/g1/alvr_input.py \
  --nominal "$RUN_DIR/nominal_targets.json" \
  --target-bounds config/g1/teleop_small_motion.json
```

`--nominal`은 `[head, left, right]` 순서의 3×3 좌표 목록 또는
`nominal_targets`/`nominal_keypoints_root_m` 키가 있는 JSON을 받는다.
G1 모델의 FK로 생성한 정책과 동일한 기준 위치를 사용해야 한다.
파일을 생략했을 때 사용하는 일반 기준 좌표는 통신 테스트용이며
다른 로봇 모델에 맞는 값이라고 가정하면 안 된다.

| 입력 | 동작 |
|---|---|
| 왼쪽 X / 터미널 `c` | 정면을 보고 편한 중립 자세에서 기준 자세 보정, 조작 허용 해제 |
| 오른쪽 A / 터미널 Enter | 보정 후 조작 허용 |
| 양손 그립을 동시에 70% 이상 누르고 유지 | 조작 허용 상태에서만 정책 입력 활성화 |
| 그립 하나라도 놓기 | 즉시 입력 비활성화, 이동 명령 0 |
| 오른쪽 B / 터미널 Space | 조작 허용 해제; 다시 A/Enter 필요 |
| 왼쪽 Y / 터미널 `r` | 시뮬레이션 리셋 요청, 조작 허용 해제 |
| 왼쪽 스틱 | 앞뒤·좌우 이동 목표 속도 |
| 오른쪽 스틱 좌우 | 회전 목표 속도 |
| 터미널 `q` / Ctrl+C | 입력 프로그램 종료, 정지 패킷 전송 |

보정 → A → 양손 그립 순으로 시작한다. `ACTIVE`와 `enabled=True`는 입력이
활성화되었다는 뜻이다. UDP에 상대방 수신 확인 응답은 없으므로 이 출력만으로
시뮬레이션이 실행되었다고 판단하지 않는다. 시뮬레이터의 수신 상태도 확인한다.
트리거 값은 기록할 수 있지만 이 세 점 정책의 그리퍼 제어 명령으로 사용하지 않는다.

옵션을 생략한 입력 속도 상한은 전후 0.5 m/s, 좌우 0.25 m/s, 회전 0.6 rad/s이다.
`--velocity-limits VX VY YAW`로 각각의 상한을 줄일 수 있다. 단위는 m/s, m/s,
rad/s이며 모든 값은 유한한 0 이상의 수여야 하고 기존 상한을 넘길 수 없다.
0으로 지정한 축은 스틱을 움직여도 목표 속도가 0이다. 기존 스틱 부호와 deadzone은
유지한다.

전후·좌우 이동을 실험하되 회전 명령을 비활성화하려면 다음처럼 실행한다.
선택한 모델이 해당 이동 범위로 학습되고 별도 평가되었는지 먼저 확인한다.

```bash
.venv-alvr/bin/python scripts/g1/alvr_input.py \
  --nominal "$RUN_DIR/nominal_targets.json" \
  --target-bounds config/g1/teleop_small_motion.json \
  --velocity-limits 0.15 0.08 0
```

이 수치는 명령 제한이며 보행·회전 성능이나 안정성의 증명이 아니다. 정지 기반
정책은 수신기에서 학습 범위에 따라 이동 명령을 모두 0으로 제한한다.
입력 프로그램의 상한 설정으로 모델의 학습 범위를 넓힐 수는 없다.

학습·추론 제어 간격은 시뮬레이션 시간 0.02초(목표 50 Hz)다. `--real-time`은
처리가 빠를 때 대기 시간을 넣지만 느린 GPU를 50 Hz로 만들지는 않는다.
v2 목표 속도도 같은 0.02초 차분으로 계산하므로, 실제 입력은 벽시계 시간으로
움직이는데 시뮬레이터가 30 Hz에 머무르면 같은 손 이동이 시뮬레이션 시간에서는
약 1.67배 빠르게 해석될 수 있다. 학습과 같은 실시간 움직임을 기대하려면
렌더링·동시 학습 부하를 줄이고 실제 제어 주기를 확인한다.

새 실행의 `result.json`에는 `mean_control_hz`, `wall_loop_seconds`,
`simulation_to_wall_time_ratio`가 기록된다. 초기화 시간이 제외된 전체 제어 루프
평균이며 활성 조작 구간만의 평균은 아니다. `--record-trace`의 `wall_time`과
`enabled`를 함께 사용하면 활성 구간의 주기도 따로 확인할 수 있다.
합성 시나리오의 단계별 확인은 [RUNTIME_SCENARIO.md](RUNTIME_SCENARIO.md)를 따른다.
이 지표는 제어 루프 성능이며 ALVR 영상이나 헤드셋 추적의 종단 간 지연 측정은 아니다.

## 3. 좌표계와 조작 범위

OpenVR는 +X 오른쪽, +Y 위쪽, -Z 전방이고 단위는 미터다.
[Valve의 좌표 정의](https://github.com/ValveSoftware/openvr/blob/master/headers/openvr.h)를 기준으로
G1의 +X 전방, +Y 왼쪽, +Z 위쪽 좌표로 변환한다.

정책 좌표 계약은 `g1-yaw-floor-relative-v1`이다. 원점 XY는 로봇 골반의 현재
수평 위치, 원점 Z는 바닥 높이 + 중립 기립 시 골반 높이다. 축은 골반의 yaw만
따라가며 현재 roll/pitch나 웅크린 골반 높이를 따라가지 않는다. 따라서 머리를
낮추는 입력이 현재 골반 좌표를 빼면서 사라지지 않는다.

머리의 초기 수평 방향을 정면으로 정하고, 각 장치의 **초기 위치에서 변화한 값**을
로봇 기준 위치에 더한다. 따라서 사람의 키가 그대로 G1 골반 기준 머리 높이로
들어가지 않는다. 기본 변화량 배율은 0.65이고 `--scale`로 조절할 수 있다.
별도 설정을 생략했을 때의 기존 기본 제한은 머리 각 축 ±0.12 m, 양손 각 축
±0.30 m이다. 첫 연결에는 위 명령처럼 `--target-bounds`로 작은 조작 범위를
명시하는 것을 권장한다.

`config/g1/teleop_small_motion.json`은 보정한 중립 위치를 기준으로 머리 X/Y는
±0.025 m, 머리 Z는 -0.04~0 m, 양손의 각 축은 ±0.05 m로 제한한다.
사람 입력에 배율을 적용하고 로봇 좌표로 변환한 뒤 이 범위로 잘라낸다.
이 설정은 **임시 조작 범위**이며 정책의 추종 성능·안정성·실제 Quest 연결을
자동으로 입증하지 않는다. 선택한 모델의 별도 평가 결과를 함께 확인한다.

사용자 설정 JSON은 `units: "m"`와 `delta_min`, `delta_max`를 포함해야 한다.
각 배열의 행 순서는 `[head, left_hand, right_hand]`, 열 순서는 `[X, Y, Z]`이며
3×3 유한 수치여야 한다. 각 축은 `delta_min <= 0 <= delta_max`와
`delta_min < delta_max`를 만족해야 하므로 중립 위치를 포함한다.
`--target-bounds`는 OpenVR 및 synthetic 입력의 변환에 적용된다.
독립적인 `runtime_scenario.py --tracking-sweep`의 ±6 cm 손 목표는 바꾸지 않는다.

모델의 관측 입력은 세 점의 **위치**이며 머리·손목 방향은 원본 기록에만 보관한다.

Quest 3와 두 컨트롤러에는 다리 추적 정보가 없다. 사람의 실제 발 움직임을
그대로 복원하는 기능은 제공하지 않는다. 하체는 학습한 정책이 균형·목표 추종을
위해 생성하며 이동·회전은 스틱으로 명령한다.

머리/컨트롤러 추적 또는 버튼 action이 끊기면 즉시 조작 허용을 해제한다.
연결 복구 후 A/Enter를 다시 눌러야 한다. 장치 역할이 바뀌거나 SteamVR 기준
공간 변경 이벤트가 들어오면 기준 자세 보정도 다시 해야 한다.

## 4. 헤드셋 없이 통합 검증

실행 중인 G1 시뮬레이터에 가상 입력을 10초간 보낸다.
`--enable-synthetic`이 있어야 실제 시뮬레이션 동작을 허용하는 패킷이 전송된다.

```bash
.venv-alvr/bin/python scripts/g1/alvr_input.py \
  --backend synthetic --enable-synthetic --duration 10 \
  --nominal "$RUN_DIR/nominal_targets.json" \
  --target-bounds config/g1/teleop_small_motion.json
```

`SYNTHETIC INPUT`은 실제 Quest가 연결되었다는 뜻이 아니다.
가상 입력은 중립 자세 보정 후 작은 양손 위치 변화만 생성한다.

기록과 재생:

```bash
# 실제 OpenVR 입력 및 변환된 목표를 새 파일에 기록한다. 기존 파일은 덮어쓰지 않는다.
.venv-alvr/bin/python scripts/g1/alvr_input.py \
  --nominal "$RUN_DIR/nominal_targets.json" \
  --target-bounds config/g1/teleop_small_motion.json \
  --record logs/g1/quest_session.jsonl

# 기본 재생은 동작을 허용하지 않는다.
.venv-alvr/bin/python scripts/g1/alvr_input.py \
  --backend replay --replay-file logs/g1/quest_session.jsonl

# 시뮬레이터에서 기록된 활성 상태까지 재생하려면 명시적으로 지정한다.
.venv-alvr/bin/python scripts/g1/alvr_input.py \
  --backend replay --replay-file logs/g1/quest_session.jsonl --enable-replay
```

재생은 기록 당시 보정·배율을 적용한 목표를 사용하며 새 세션 UUID와 현재 시각을
붙인다. 새 `--nominal`/`--scale`로 원본을 다시 보정하는 기능은 아니다.
`--target-bounds`를 replay와 함께 지정하면 오류로 종료한다. 기록된 제한 후 목표를
그대로 재생하므로 새 범위를 조용히 무시하거나 다른 기준 위치에 다시 적용하지 않는다.
`--velocity-limits`도 replay에는 지정할 수 없다. 재생은 기록된 최종 속도 명령을
사용하며 새 스틱 제한값을 적용하는 경로가 아니다.
재생 도중 Space를 누르면 그 프로세스에서는 다시 활성화되지 않는다.

자동 검사:

```bash
PYTHONDONTWRITEBYTECODE=1 .venv-alvr/bin/python -m unittest discover \
  -s tests/g1 -p 'test_vr*.py' -v
```

2026-09-30에 실제 UDP 송수신, 별도 송신 프로세스, 좌표 변환, 보정, 추적 끊김,
정지/리셋, 가상 OpenVR 런타임 API를 검사했다. 실제 Quest·ALVR 영상과 버튼
바인딩은 현장 장비로 확인해야 한다.

## 5. 연결 규약과 서버 설정

JSON `g1.vr.target.v1`은 `frame=robot_yaw_nominal_height`,
`coordinate_version=g1-yaw-floor-relative-v1`, `session` UUID, 증가하는 `seq`, 송신 시각,
`source`(`openvr`/`synthetic`/`replay`), `target_positions_m` 3×3,
`command_velocity` 3개(vx, vy, yaw-rate), `enabled`, `calibrated`,
`tracking_valid`, 증가하는 `reset_id`를 보낸다. 수신 측 `poll()`은 리셋 번호
증가를 한 번의 `reset=True` 이벤트로 바꾼다.

성공한 기준 자세 보정마다 `calibration_id`도 증가한다. X를 누르고 있는 동안은
한 번만 증가하며 보정 실패·시뮬레이션 리셋과는 별개다. XR 쪽에서 같은 세션의
새 보정 시점을 식별할 수 있도록 전달하는 값이며, 그 자체로 조작을 허용하지 않는다.
이 필드가 없는 기존 패킷은 0으로 읽는다. 수신자는 같은 세션에서 보정 번호가
작아지는 패킷을 거절하고 새 세션에서는 비교 기준을 다시 시작한다.

송신 시각 및 로컬 수신 경과 시간 모두 기본 0.25초 제한을 적용한다.
중복·역순·잘못된 크기·NaN·좌표계 불일치 패킷은 거절한다. 새로운 송신 세션은
이전 세션이 끊긴 뒤에만 연결할 수 있다. 수신자는 `enabled`와 `fresh`를 확인한
후 정책 입력에 적용해야 한다.

권장 구성은 ALVR·SteamVR·입력 프로세스·Isaac Sim을 같은 서버에서 실행하고,
입력 UDP를 localhost에만 두는 것이다. 다른 호스트를 사용할 때는 송신
`--host`와 시뮬레이터 수신 주소를 명시하고 두 호스트 시간을 동기화한다.
인증 없는 로컬 시뮬레이션 통신이므로 공용 인터넷에 포트를 직접 노출하지 않는다.

## 6. 문제 해결

| 증상 | 확인할 내용 |
|---|---|
| `NoServerForBackgroundApp` | 해당 사용자 그래픽 세션에서 SteamVR부터 실행한다. |
| `TRACKING_LOST` | 머리와 두 컨트롤러 모두 SteamVR에서 추적 중인지 확인한다. |
| `Actions inactive` | ALVR 프로필이 `oculus_touch`인지, action 바인딩과 SteamVR 대시보드 상태를 확인한다. |
| `UNCALIBRATED` | 정면을 보고 X/c를 누른다. 컨트롤러 역할 변경 후에는 다시 보정한다. |
| `HOLD_GRIPS` | 양손 그립을 동시에 누른다. |
| 송신은 `ACTIVE`인데 수신되지 않음 | 양쪽 UDP 주소/포트, 중복 송신 프로세스, 호스트 시간 차이를 확인한다. |
| 모델 자세가 기준부터 크게 어긋남 | 체크포인트와 같은 G1 모델/FK의 `nominal_targets.json`인지 확인한다. |
| Quest에서 영상이 안 보임 | ALVR 영상 연결과 Isaac Sim XR 실행을 별도로 확인한다. 입력 UDP에는 영상이 없다. |

SteamVR 입력 바인딩이 다른 앱의 포커스/대시보드 때문에 비활성화되면 프로그램은
조작을 멈춘다. 실제 실행 중 영상 앱과 입력 action이 동시에 활성인지 장비에서
확인해야 한다. 헤드셋 없는 자동 테스트는 이 동시 실행을 입증하지 않는다.

## 7. SteamVR 앱 식별과 동시 입력의 검증 범위

입력 프로세스는 `org.limchanju.vr_teleop.g1_input`이라는 고정 앱 키로
`G1 Teleop Input` 바인딩을 사용한다. 시작할 때 임시 디렉터리에 `.vrmanifest`를
생성하고 `AddApplicationManifest(path, true)` →
`IdentifyApplication(현재 PID, app_key)` → `SetActionManifestPath(절대 경로)`
순서로 등록한다. 등록 후 앱 존재와 PID 연결을 다시 확인한다. 등록·식별 실패나
같은 키의 중복 실행이 감지되면 입력을 시작하지 않고 원인과 조치 방법을 출력한다.

Linux 매니페스트 필드 이름은 `binary_path_linux`다. `launch_type="binary"`,
현재 Python 실행 파일의 절대 경로, `action_manifest_path`, 표시 이름을 기록한다.
실행 파일 경로는 앱 식별을 위한 메타데이터이며 이 코드에서 앱 실행이나 자동
시작을 요청하지 않는다. Background 역할과 액션 우선순위 0을 유지한다.
정상 종료·초기화 실패 모두 자신이 만든 매니페스트 등록 해제, OpenVR 종료,
임시 파일 삭제를 시도한다. SteamVR가 먼저 종료되어 정리 API가 실패해도 나머지
정리는 계속하고 경고를 남긴다. 임시 등록은 다음 SteamVR 실행 시 자동 로드되지
않으며 기존 ALVR 설정이나 다른 앱의 매니페스트를 수정하지 않는다.

기존 Python 앱과 구별되는 바인딩 이름은 이 식별 등록의 목적이다. Valve 공식
Unity 플러그인도 임시 매니페스트 등록 뒤 현재 PID를 명시해 앱을 식별한다.
Linux 필드 이름과 binary/action 매니페스트 조합은 설치된 Valve SteamVR의
`resources/config/runtime.vrmanifest`에서도 확인했다. 이 파일에서
`openvr.component.vrcompositor` 등의 항목이 `binary_path_linux`를 사용한다.
[Valve 공식 등록 코드](https://github.com/ValveSoftware/steamvr_unity_plugin/blob/master/Assets/SteamVR/Scripts/SteamVR.cs#L574),
[Valve 앱 식별 API 정의](https://github.com/ValveSoftware/openvr/blob/master/headers/openvr.h),
[action_manifest_path 공식 설명](https://github.com/ValveSoftware/openvr/wiki/Action-manifest#telling-steamvr-about-your-action-manifest-when-the-app-isnt-running)

`isInputAvailable()`이 거짓이거나 `shouldApplicationPause()`가 참이면 입력을
무효화한다. 필수 action이 비활성 상태일 때도 버튼·그립을 재사용하지 않으며
진단 메시지에 해당 action 이름을 표시한다. Standing 원점 재설정(808)과
Room Setup 확정(807)을 포함한 좌표계 변경 시 재보정이 필요하다.
기존 `GetControllerState`로 자동 전환하거나 overlay 입력 우선권을 강제로
높이는 코드는 없다. 이 동작과 이벤트 값은
[Valve SDK](https://github.com/ValveSoftware/openvr/blob/master/headers/openvr.h)에 근거한다.

공식 매니페스트 문서에서 action set의 `usage`는 바인딩 UI 설정이며
`leftright`·`single`·`hidden`이 정의되어 있다. 현재 `leftright`를 유지한다.
[공식 명세](https://github.com/ValveSoftware/openvr/wiki/Action-manifest#action-sets)
다른 게임 실행 중 Background 앱의 action 입력이 동작했다는
[공식 저장소의 사용자 사례](https://github.com/ValveSoftware/openvr/issues/1833#issuecomment-3999679228)는
존재하지만, 사례의 미문서화된 `usage="background"`를 필수 설정으로 해석하지
않았다. 이 사례는 Valve의 모든 런타임 조합에 대한 지원 보증도 아니다.

2026-09-30 변경은 실제 openvr Python 패키지의 ctypes 자료형을 사용하는 모의
런타임으로 검사했다. 등록·식별 순서와 현재 PID, 정상 종료, 중복 실행 거부,
등록·식별·action 초기화 실패, API가 성공을 반환해도 앱/PID가 누락된 경우,
종료 API 실패 후 파일 정리, 입력 포커스 상실, 원점 재설정, 입력 값 재사용
방지를 검사한다. **실제 SteamVR를 이 테스트에서 실행하지 않았다.**

현장에서는 Isaac Sim OpenXR 영상이 출력되는 동안 다음을 별도로 확인한다.

1. `G1 Teleop Input` 바인딩에서 모든 필수 action과 양손 아날로그 값이 활성이다.
2. 대시보드를 열면 조작이 중지되고 닫은 뒤 다시 시작 버튼을 눌러야 한다.
3. Standing 원점을 바꾸면 조작이 중지되고 새 기준으로 보정해야 한다.
4. 입력 프로세스 종료 시 Isaac 영상 앱은 계속 실행되며, 재실행 시 같은 G1
   바인딩 이름으로 정상 등록된다.

이 검사를 통과하기 전까지 Linux·Quest 3·ALVR·Isaac OpenXR의 동시 입력/영상
연결은 **실제 장비 미검증**으로 기록한다. 이번 수정 전 파일은 로컬
`backups/openvr_identity_20260930_192703/`에 보관했다.

추가 변경: SteamVR OpenXR 런타임의 프로세스별 선택 안내와 제어 주기 지표를
보완했으며, 수정 전 문서는 `backups/alvr_runtime_docs_20260930_195207/`에 있다.
보정 번호 추가 전 입력 코드·문서는 `backups/calibration_generation_20260930_195346/`에
보관했다. 보정 번호는 모의 입력과 실제 localhost UDP로 검증하며 실제 헤드셋 검증과
구별한다.
