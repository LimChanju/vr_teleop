# Quest 3 ALVR → Unitree G1 simulation teleoperation

Quest Touch 입력을 ALVR/SteamVR에서 읽어 Unitree 공식 `xr_teleoperate`의 G1 팔 IK와
`unitree_sim_isaaclab` DDS 제어에 연결합니다. 공식 시뮬레이터에 등록된 G1 태스크 14개를
하나의 런처로 선택할 수 있습니다.

```text
Quest 3 + Touch controllers
  → USB ALVR → SteamVR/OpenVR
  → aim pose, trigger, grip, buttons, sticks
  → ALVR safety/coordinate adapter
  → official G1_29 IK + arm/hand DDS
  → Unitree Isaac Lab simulator
```

현재 범위는 시뮬레이터뿐입니다. 실물 G1로 전환하는 옵션은 제공하지 않습니다.

## 지원 범위

```bash
./run_unitree_task.sh list
```

| 태스크 | Dex1 | Dex3 | Inspire |
| --- | --- | --- | --- |
| Cylinder pick/place | `pick-cylinder-dex1` | `pick-cylinder-dex3` | `pick-cylinder-inspire` |
| Red block pick/place | `pick-redblock-dex1` | `pick-redblock-dex3` | `pick-redblock-inspire` |
| R/G/Y block stack | `stack-rgy-dex1` | `stack-rgy-dex3` | `stack-rgy-inspire` |
| Red block → drawer | `drawer-dex1` | `drawer-dex3` | 공식 태스크 없음 |
| Whole-body cylinder | `move-cylinder-dex1` | `move-cylinder-dex3` | `move-cylinder-inspire` |

- Dex1 5개는 공식 controller/gripper 경로를 사용합니다.
- Dex3 5개와 Inspire 4개는 Quest Touch 트리거를 25-joint synthetic grasp로 바꾸는
  **실험적 synergy**입니다. 개별 손가락 추적과 동등하지 않습니다.
- Whole-body 3개는 스틱 명령을 시뮬레이터의 `rt/run_command/cmd`에 100 Hz로 보냅니다.
  공식 `--motion`의 LocoClient와는 다른 시뮬레이터 전용 경로입니다.

## 1. ALVR 입력 확인

ALVR Dashboard와 SteamVR을 먼저 실행하고 Quest 3를 USB wired client로 연결합니다.
이 백엔드는 **Quest Touch 컨트롤러 전용**입니다. Quest hand tracking으로 전환되지 않게 하고
양쪽 Touch 컨트롤러를 켜 둡니다.

```bash
cd /home/railab/Desktop/vr_teleop/alvr
PYTHON_BIN=/home/railab/miniconda3/envs/isaac_vr/bin/python bash scripts/setup.sh
./run_probe.sh --duration 5
```

`setup.sh`는 `PYTHON_BIN`과 기존 `.venv`가 모두 Python 3.10~3.12인지 확인합니다. 이전에
다른 Python으로 만든 `.venv`가 있으면 해당 폴더를 옮겨 둔 뒤 다시 실행하십시오.

`[READY]`이면 HMD, 좌·우 pose와 버튼 action을 모두 받고 있습니다. 전체 샘플은 다음처럼
기록할 수 있습니다.

```bash
./run_probe.sh --rate 30 --output logs/quest3-input.jsonl
```

프로브는 진단 전용이며 로봇 명령을 보내지 않습니다. 실제 teleop을 실행하기 전에 반드시
프로브를 종료하십시오. 두 프로세스가 같은 SteamVR action app을 동시에 점유하지 못하도록
런타임 lock도 적용되어 있습니다.

## 2. 공식 소스와 환경 준비

검증한 리비전과 모든 서브모듈을 `vendor/`에 받습니다. 이 폴더는 Git에서 제외됩니다.

```bash
cd /home/railab/Desktop/vr_teleop/alvr
bash scripts/setup_unitree_sources.sh
```

이 스크립트가 고정하는 버전은 다음과 같습니다.

- `xr_teleoperate`: `817fb00c63cd`
- `unitree_sim_isaaclab`: `e30c25b1dffd`
- `unitree_sdk2_python`: `814556d15970`

이미 존재하는 checkout의 `HEAD`가 고정 리비전과 다르면 스크립트는 그 내용을 바꾸지 않고
중단합니다. 해당 checkout의 변경을 먼저 확인한 뒤 정확한 리비전으로 전환하거나 다른 곳으로
옮기고 다시 실행하십시오.

이 워크스테이션에는 다음 구성을 완료해 두었습니다.

| 구성 요소 | 설치 위치/버전 |
| --- | --- |
| Isaac Sim | `/home/railab/isaac-sim-4.5.0` |
| Isaac Lab | `vendor/IsaacLab` @ `91ad4944f2b7fad29d52c04a5264a082bcaad71d` |
| Isaac Sim 연결 | `vendor/IsaacLab/_isaac_sim` → 위 Isaac Sim 설치 |
| Unitree USD asset | `vendor/unitree_sim_isaaclab/assets` (약 1.8 GB) |
| 시뮬레이터 환경 | conda `unitree_sim_env`, Python 3.10/NumPy 1.26 |
| teleop 환경 | conda `tv`, Python 3.10/Pinocchio 3.1 |

Unitree asset은 공식 패키지를 staging 디렉터리에 받은 뒤 SHA-256과 ZIP CRC를 확인하고
기존 파일을 덮어쓰지 않는 방식으로 설치했습니다. 14개 G1 태스크가 참조하는 주 USD 13개도
모두 실제 파일인지 확인했습니다.

새 PC에서 처음부터 재구성할 때만 Unitree의 `auto_setup_env.sh`를 참고하십시오. 이 스크립트는
`sudo` 사용, Isaac Sim 중복 설치, 기존 `assets/` 교체가 포함될 수 있으므로 현재 워크스테이션에서
다시 실행할 필요는 없습니다. Isaac Sim 4.5를 이미 보유한 경우에는 이 구성처럼 고정된 Isaac Lab을
연결하고 별도 `unitree_sim_env`를 쓰는 편이 안전합니다.

Teleop 환경을 다시 만들 때는 Python 3.10 또는 3.11을 사용합니다(`teleimager`가 Python 3.12
이상을 지원하지 않습니다). CycloneDDS C SDK가 없으면 설치 스크립트가 self-contained 0.10.2
wheel을 사용합니다.

```bash
conda create -n tv python=3.10 pinocchio=3.1.0 casadi=3.6.7 numpy=1.26.4 -c conda-forge
cd /home/railab/Desktop/vr_teleop/alvr
TELEOP_PYTHON=/home/railab/miniconda3/envs/tv/bin/python \
  bash scripts/setup_teleop_env.sh
```

`setup_teleop_env.sh`는 CycloneDDS 0.10.2, teleimager 의존성, 실제 `ImageClient`, G1 IK와
팔/손 controller, ALVR `TeleVuerWrapper` import까지 확인합니다. CycloneDDS Python 0.10.2가
이미 정상 설치돼 있으면 `CYCLONEDDS_HOME`을 생략할 수 있습니다.

기본 런처는 다음 conda 환경을 자동 탐색합니다.

```text
unitree_sim_env  # conda activation hook을 적용한 뒤 Isaac Sim을 exec
tv               # xr_teleoperate 실행
```

Conda가 다른 위치에 있으면 `CONDA_EXE`를 지정합니다. 완전히 다른 실행 구성이면 Python 명령을
직접 덮어쓸 수도 있습니다.

```bash
export CONDA_EXE=/path/to/miniconda3/bin/conda
# 선택 사항:
export UNITREE_SIM_PYTHON='/custom/command python'
export XR_TELEOPERATE_PYTHON=/path/to/tv/bin/python
```

## 3. 사전 점검과 실행

첫 태스크로 Dex1 cylinder를 권장합니다.

```bash
cd /home/railab/Desktop/vr_teleop/alvr
./run_unitree_task.sh doctor pick-cylinder-dex1
./run_unitree_task.sh print pick-cylinder-dex1
./run_unitree_task.sh all pick-cylinder-dex1
```

`all`은 시뮬레이터를 먼저 시작하고 DDS controller ready 문구를 확인한 다음 teleop을 실행합니다.
기본 실행은 학교망에서 무기한 대기할 수 있는 원격 S3 창고 배경과 사용하지 않는 spectator
카메라들만 생략합니다. 로봇, 태스크 물체/테이블, 머리 카메라와 양쪽 손목 카메라는 유지됩니다.
공식 배경까지 그대로 불러오려면 `--full-scene`을 추가할 수 있습니다.
세 카메라는 로컬 ZMQ로만 전달하므로 별도 WebRTC 인증서나 학교망 접속도 필요하지 않습니다.

처음 한 번은 3개 RTX 카메라의 셰이더/텍스처 cache를 만들기 때문에 수 분간 마지막 로그가
그대로일 수 있습니다. 기본 준비 제한은 15분이며 CPU/GPU 사용률이 유지되면 그대로 기다립니다.
이후 실행은 cache를 재사용합니다.
공식 teleop 화면에 시작 안내가 나오면 다음 순서를 지킵니다.

1. Touch 컨트롤러를 몸 앞의 편한 중립 위치에 둡니다.
2. 터미널에서 `r`을 눌러 공식 control loop를 시작합니다.
3. 양쪽 grip을 모두 한 번 놓습니다.
4. 양쪽 grip을 계속 잡으면 `ARMED`가 되고 팔이 따라옵니다.
5. 한쪽 grip이라도 놓으면 팔은 현재 관절에서 hold되고 base 속도는 즉시 0이 됩니다.
6. 종료는 터미널의 `q` 또는 `Ctrl+C`입니다.

개별 프로세스만 실행할 수도 있습니다.

```bash
./run_unitree_task.sh sim pick-cylinder-dex1
./run_unitree_task.sh teleop pick-cylinder-dex1
```

`--sim-arg=--option`과 `--teleop-arg=--option`으로 비관리 옵션을 추가할 수 있습니다. 반드시
옵션의 전체 이름만 사용하십시오. 태스크, 로봇, end-effector, DDS backend, `--motion` 등 런처가
관리하는 옵션과 argparse 축약형은 런처가 거부합니다.

## 조작

| 입력 | 동작 |
| --- | --- |
| 양쪽 grip hold | dead-man arm/hand/base 활성화 |
| 한쪽 grip release | 팔 현재 관절 hold, 손 입력 freeze, base zero |
| 좌·우 trigger | Dex1 gripper 또는 Dex3/Inspire grasp synergy |
| 왼쪽 stick | Whole-body 전진/후진 및 좌우 이동 |
| 오른쪽 stick X | Whole-body yaw |
| 양쪽 stick click | latch되는 soft e-stop |
| grip을 놓고 Y + B | e-stop reset; 이후 grip을 다시 잡아 재활성화 |

Whole-body 명령은 100 Hz heartbeat로 반복되며 base Python watchdog은 0.2초, arm Python
watchdog은 0.25초의 command 정지를 감지해 안전 상태로 latch합니다. 이 경우 양쪽 grip을 놓았다가
다시 잡아야 합니다. 이 watchdog은 Python process/GIL 전체가 멈추거나 runtime이 frozen-valid
입력을 계속 내보내는 경우까지 보장하지 않으므로 DDS/시뮬레이터 쪽 안전 제한도 함께 필요합니다.

## Pose 처리와 안전 장치

- `/pose/openxr_aim`을 우선 사용해 공식 TeleVuer controller 좌표 수식과 맞춥니다.
- aim action이 최소 3초 동안 오지 않으면 raw grip pose로 fallback합니다. 이때 좌우 컨트롤러를
  교차하지 말고 최소 0.4초 동안 중립 자세로 정지해 calibration을 완료합니다.
- OpenVR `+X right, +Y up, -Z forward`를 Unitree `+X forward, +Y left, +Z up`으로 변환합니다.
- HMD yaw-relative wrist pose, workspace clamp, 선속도/각속도 slew limit을 적용합니다.
- `Running_OK`가 아닌 tracking 상태는 거부하며 tracking이 끊기면 항상 dead-man이 해제됩니다.
  정상 disconnect/reconnect도 connection generation을 바꿔 calibration을 초기화하고, 매 sample
  갱신하는 controller serial/tracking-system identity가 바뀌어도 다시 calibration합니다.
- 공식 main loop 위에 검증된 safety overlay를 적용해 disarm 시 measured arm joints와 zero
  feed-forward torque를 보냅니다. 업스트림 코드 형식이 바뀌면 조용히 실행하지 않고 실패합니다.

## 화면에 보이는 것

ALVR은 Quest에 SteamVR 화면을 전송하지만, 이 구현의 `render_to_xr()`는 의도적으로 no-op입니다.
따라서 현재는 Quest의 SteamVR Desktop에서 Isaac Sim 창을 보는 방식입니다. 로봇 head camera를
stereo VR layer로 직접 합성하거나 Quest pass-through와 합성하는 compositor는 아직 구현하지
않았습니다. 입력 teleoperation과 robot-camera VR 표시는 별개의 단계입니다.

14개 항목은 고정된 공식 태스크 catalog와 명령 생성 범위를 뜻합니다. 하드웨어 없는 단위 테스트와
`doctor` 통과가 각 태스크의 Isaac 시뮬레이션 end-to-end 동작을 대신 검증하지는 않습니다.

## 문제 해결

- `[TRACKING_ONLY]`: Touch를 들어 버튼을 눌러 깨우고 SteamVR action binding을 확인합니다.
- `PARTIAL`, `L=--` 또는 `R=--`: 컨트롤러 배터리와 ALVR tracking 상태를 확인합니다.
- `aim_pose.pose_valid=false`: Touch를 사용 중인지 확인하고 SteamVR을 재시작합니다. 고정된 raw
  calibration fallback도 사용할 수 있지만 aim pose가 더 정확합니다.
- `another vr_teleop probe/adapter ...`: 실행 중인 `run_probe.sh` 또는 다른 teleop을 종료합니다.
- doctor의 `sim asset missing`: 공식 simulator 환경/asset 설치가 끝나지 않은 상태입니다.
- `teleop Python imports failed`: `TELEOP_PYTHON=... bash scripts/setup_teleop_env.sh`를 다시 확인합니다.
- 장면 출력 후 로그가 오래 멈춤: 첫 RTX 카메라/재질 warm-up일 수 있습니다. GPU 사용률이 유지되면
  최대 15분 기다립니다. 필요하면 `--startup-timeout 1200`으로 늘릴 수 있습니다.
- 학교 Wi-Fi 단말 격리: Quest↔ALVR은 USB wired connection을 유지합니다. simulator와 teleop이
  같은 호스트에서 실행되므로 기본 image server 주소는 `127.0.0.1`입니다.

테스트는 하드웨어 없이 실행할 수 있습니다.

```bash
./.venv/bin/python -m unittest discover -s tests -v
```

참고:

- [Unitree xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate)
- [Unitree unitree_sim_isaaclab](https://github.com/unitreerobotics/unitree_sim_isaaclab)
- [ALVR wired setup](https://github.com/alvr-org/ALVR/wiki/ALVR-wired-setup-%28ALVR-over-USB%29)
