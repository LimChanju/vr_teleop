# VIVE Ultimate Tracker Linux 직접 연결

> xi-ve Space Calibrator를 사용하는 새 SteamVR 경로는
> [VR_QUICKSTART.md](VR_QUICKSTART.md#새-공간-정렬-경로-xi-ve-space-calibrator--vive--steamvr)를 참고한다.
> 아래는 기존 직접 VUT 경로다. 두 경로의 공간 보정은 중복 적용하지 않는다.

이 경로는 Windows relay를 사용하지 않는다. VIVE Ultimate Tracker 5개는 Linux 서버에
꽂힌 VIVE Wireless Dongle과 VIVE Hub Linux가 읽고, TWIST2 adapter가 로컬 Unix
socket에서 pose를 직접 받는다. Quest 3와 Touch controller만 ALVR/SteamVR를 거친다.

```text
Quest 3 + Touch ─ ALVR ─ Linux SteamVR/OpenVR ─┐
                                               ├─ Quest/VIVE 공간 정렬 ─ TWIST2 adapter
Ultimate 5개 ─ 동글 ─ VIVE Hub ─ /tmp/vut.sock ┘
```

현재 서버는 Ubuntu 22.04 x86_64이며 `.conda-vr` 환경은 이미 준비돼 있다. Windows,
Windows SteamVR, 새 pip package, 별도 C++ SDK build는 필요 없다. 동글은 Windows PC가
아니라 이 Linux 서버 USB에 꽂아야 한다.

## 1. 한 번만 설치

먼저 HTC의 [VIVE Hub Linux release와 EULA](https://github.com/ViveRoboticsSoftware/vivehub-linux/releases/tag/v1.0.3)를
확인한다. 그다음 아래를 실행한다.

```bash
cd /home/railab/Desktop/vr_teleop/TWIST2

sudo apt update
sudo apt install -y curl ca-certificates usbutils \
  libgl1 libx11-6 libgl1-mesa-dri libudev1

bash scripts/setup_vive_linux.sh --accept-vive-eula
sudo external/VIVEHub-Linux-v1.0.3/install-udev.sh
```

마지막 명령 뒤에 VIVE Wireless Dongle을 뺐다가 Linux 서버에 다시 꽂는다. 설치
스크립트는 공식 v1.0.3 archive와 HTC가 게시한 SHA256을 확인하고
`external/VIVEHub-Linux-v1.0.3`에 원본 그대로 푼다.

## 2. Hub, 페어링, tracking map

한 터미널에서 Hub를 background로 시작할 수 있다.

```bash
bash scripts/run_vive_linux_hub.sh --background
```

화면에 열린 Hub에서 다음을 수행한다.

1. 동글 하나에 tracker 5개를 pair한다.
2. `Start setup`으로 tracking map을 만든다.
3. 다섯 장치가 모두 `Ready`인지 확인한다.

Hub UI가 `tracker_server`를 직접 시작하므로 별도로 daemon을 실행하지 않는다. 상태를
확인하거나 종료할 때는 다음을 쓴다.

```bash
bash scripts/run_vive_linux_hub.sh --status
bash scripts/run_vive_linux_hub.sh --stop
```

## 3. 연결 확인과 serial 역할 지정

```bash
bash scripts/vive_linux_doctor.sh
bash scripts/run_vive_linux_inventory.sh
```

정상이면 각 장치에 `serial`이 비어 있지 않고, `state_name`은 `ready`, `ready`는
`true`로 나온다. 출력의 `device_id` 1~5는 재사용되는 동글 slot이므로 역할 매핑에
절대로 쓰지 않는다.

`config/local/tracker_mapping.vive_linux.json`의 다섯 placeholder key를 실제 serial로
바꾼다. 권장 배치는 다음과 같다.

```text
허리        Pelvis
왼쪽 무릎  Left_Knee
오른쪽 무릎 Right_Knee
왼쪽 발    Left_Foot
오른쪽 발  Right_Foot
```

HMD와 양손목은 각각 Quest의 `Head`, `Left_Wrist`, `Right_Wrist`를 자동 사용한다.

## 4. Quest 공간과 VIVE 공간 정렬

### 보정 + 독립 검증 연속 실행 (현재 권장)

0314와 오른손 컨트롤러를 상대 위치·각도가 변하지 않게 고정한다. 가능하면 스트랩이나
고정대를 사용하고 두 기기 카메라를 가리지 않는다. 현재 세션에서는 다른 트래커도 모두
켜둔다. 정렬용 tracker 선택은 신체 역할 매핑을 바꾸지 않는다.

```bash
cd /home/railab/Desktop/vr_teleop/TWIST2
bash scripts/run_vive_linux_alignment_check.sh FA61N3B00314
```

별도 Enter 없이 추적 준비(최대 30초), 15초 준비, 20초 보정 데이터 수집, 12초 검증
데이터 수집이 이어진다. 두 단계 사이에 놓거나 고쳐 잡지 않는다. 보정 때는 천천히
8자 이동과 pitch/yaw/roll을 섞고, 검증 때는 같은 고정을 유지하며 다른 위치와 각도를
지나간다. 기본 수집 시간이 지나면 손을 풀어도 된다는 안내 후 계산한다. 이전의
`run_vive_linux_alignment.sh`를 두 번 실행하는 것은 이 독립 검증을 대신하지 않는다.

변환식은 `Quest_pose = world_transform × VIVE_pose × fixture_transform`이다.
컨트롤러와 tracker 원점이 실제로 떨어져 있는 거리도 fixture 변환으로 반영한다.
첫 단계만으로 두 변환을 계산하고 **검증 단계에서는 어느 변환도 다시 추정하지 않는다.**
계산 지연을 줄이기 위해 첫 단계의 유효 샘플 중 시간 전체에 고르게 분포한 최대 240개를
기존 solver에 넣고, 계산한 결과의 오차는 보정·검증 각각의 **모든 유효 샘플**로 평가한다.

자동 판정 기준 (이 저장소의 진단 기준이며 HTC 사양이나 로봇 움직임 제한이 아님):

- 보정과 검증 각각 위치 RMSE ≤ 30mm, 회전 RMSE ≤ 5도. 기존 계산 기준을 유지한다.
- 단계별 fresh/유효/중복 없는 pair 비율 ≥ 80%, 최소 8 pair.
- pose age는 -10ms~200ms, 두 provider timestamp 차이는 최대 50ms. 이 timestamp 검사는
  ALVR의 전체 센서 지연을 측정하거나 보정했다는 의미는 아니다.
- 장치 시리얼·연결 세대·VIVE 서버 세션이 바뀌거나, 검증에 보정 시점의 pose가 재사용되면 실패.
- 정지 상태만으로 검증에 통과하지 않도록 검증 단계의 양쪽 장치 각각 위치 bounding box
  대각선 ≥ 10cm, 시작 대비 회전 변화 ≥ 15도를 요구한다. 편하게 여러 위치와 방향으로
  움직이면 되며 특정 신체 동작이나 관절을 제한하지 않는다. 실제 이동 범위와 회전량도 저장한다.

매 실행은 고유한 `runtime/alignment_check/<시간>/` 아래에 저장된다:

- `settings.json`: 사용 옵션
- `poses.jsonl`: 준비·보정·검증 원본 위치/쿼터니언, 각 timestamp, 유효 상태, 탈락 사유
- `session.log`: 진행 및 오류
- `summary.json`: 보정/검증 각각의 추적 상태, 유효 비율, RMSE/p95/max, 최종 판정
- `candidate.json`: **두 단계가 모두 통과했을 때만** 생성되는 새 보정 후보
- `active_calibration_snapshot.json`: 실행 전 사용 보정 파일이 있으면 복사한 기록

추적 실패, 계산 기준 초과, Ctrl+C 때도 이미 받은 원본과 로그를 남긴다. 실패한 실행의
폴더에는 후보를 생성하지 않는다. 기존 `config/local/calibration.quest_vive.json` 및 이전
`calibration.quest_vive.candidate.json`은 변경하지 않는다. 후보의 적용은 결과 검토 후 별도로
결정한다. 이 검사는 전신 자세 변환이나 0072를 포함한 모든 tracker의 정확도를 인증하지 않는다.
HTC 공식 바이너리 및 TWIST2/GMR upstream 코드는 변경하지 않는다.

### 이전 보정 전용 명령 (검증은 하지 않음)

두 tracking system의 원점은 서로 다르다. tracker 하나를 오른쪽 Touch controller에
움직이지 않게 단단히 고정하고, 그 tracker의 실제 serial로 실행한다.

```bash
bash scripts/run_vive_linux_alignment.sh YOUR_TRACKER_SERIAL
```

countdown 뒤 fixture 전체를 넓은 8자 궤적으로 이동하면서 pitch, yaw, roll 세 축으로
충분히 회전한다. 위치 RMSE 3 cm 또는 회전 RMSE 5도를 넘으면 파일을 쓰지 않고
실패한다. 성공 파일은 다음 위치에 생긴다.

```text
config/local/calibration.quest_vive.json
```

SteamVR standing origin을 recenter하거나 VIVE tracking map을 다시 만들면 이 정렬도
다시 해야 한다. 끝나면 tracker를 controller에서 떼어 원래 신체 위치에 장착한다.

### 4-1. 재시작·추적 복구 후 가까운 두 장치의 좌표 비교

`Ready`와 `Stable capture`는 좌표가 실제 위치와 일치한다는 보장이 아니다.
장착을 유지한 채 오른손 Touch 컨트롤러 **본체**를 안내되는 tracker 바로 옆에
가까이 대고, 두 장치가 정지했을 때 보고되는 위치를 비교한다. 포인터만 tracker를
가리키는 검사가 아니다. 손이나 컨트롤러로 VIVE 카메라를 가리지 않는다.

다섯 곳을 한 번에 검사하는 명령:

```bash
cd /home/railab/Desktop/vr_teleop/TWIST2
bash scripts/run_vive_linux_validation.sh
```

`배 → 오른쪽 정강이 → 오른쪽 발등 → 왼쪽 발등 → 왼쪽 정강이` 순서로 자동 진행한다.
각 단계 10초 준비 후 5초 정지 기록으로 약 75초이며, 중간에 확인 답변이나 Enter는
필요 없다. 한 곳의 추적/좌표 검사에 실패해도 나머지를 기록하고 마지막에 합산한다.
소프트웨어 검증 결과와 신체를 직접 움직여 확인하는 결과는 구분한다.

시스템에 `spd-say`가 있으면 PC 오디오 출력으로 영어 단계 안내를 요청한다. **Quest에서
들리는지는 PC/ALVR 오디오 경로에 따라 다르며 검증되지 않았다.** 첫 안내가 들리는지
확인하고, 안 들리면 옆 사람이 터미널을 보고 순서를 알려주거나 `--no-speech`로 실행한다.
음성 안내를 위한 패키지 설치나 시스템 오디오 설정 변경은 하지 않는다. 이동 시간이
부족하면 `--countdown 20`을 사용한다. 균형을 잃지 않게 천천히 움직이고, `Ctrl+C`로
언제든 중지할 수 있다. 중지해도 완료된 단계의 기록은 남는다.

한 곳만 재검사할 때는 `bash scripts/run_vive_linux_spatial_check.sh --vive-role Pelvis`
등을 쓴다. 결과와 원본 pose는 `runtime/spatial_check/<시간>/`에 저장하며, 전체 검사는
하위 폴더에 각 단계 기록, 상위 `summary.json`에 종합 결과를 저장한다. 기존 정지 자세
기록은 덮어쓰지 않는다. 기본 15cm는 가까이 댄
두 장치의 **대략적인 좌표 불일치 검출 기준**이며, 공식 정확도 사양이나 로봇 동작
제한이 아니다. 두 장치의 보고 원점은 하우징 접촉점과 다르다. 이 검사는 큰 위치
불일치를 찾을 뿐이고, 모든 지점에서 통과해도 회전 정합·장착 보정 완료가 아니다.
잘못 댄 자세·추적 손실·움직임·좌표 오차를 구분해서 결과를 검토한다. 보정 파일,
공식 코드, Redis/로봇 출력은 변경하지 않는다.

## 5. 장착 offset과 중립 보정

장착 후 먼저 아래 명령으로 정지 자세를 기록할 수 있다. 10초 준비 시간 뒤 5초 동안
정면을 보고 서서 양발을 평행하게 두고, 컨트롤러를 양손에 든 채 양팔을 편하게 내린다.

```bash
bash scripts/run_vive_linux_mount_capture.sh
```

`runtime/mount_capture/<시간>/`에 `poses.jsonl`과 `summary.json`을 저장한다.
이는 **장착 상태 진단용 기록**이며 로봇/Redis로 데이터를 보내거나 보정 파일을
수정하지 않는다. `Stable capture`는 자세 기록이 안정적이라는 뜻이지 장착 보정
완료라는 뜻은 아니다. 위치/방향 평균과 추적 누락을 확인한 후 장착 보정을 진행한다.

공간 정렬과 tracker 장착 보정은 다른 문제다. 기본
`config/local/coordinates.vive_linux_fullbody.json`의 `tracker_to_body`는 모두 identity이고
`mounts_calibrated:false`다. 이는 통신 확인용 placeholder이지 anatomical joint 보정이
아니다.

각 tracker의 실제 장착 위치/방향을 측정해 해당 `tracker_to_body` transform을 채운 뒤에만
`mounts_calibrated:true`로 바꾼다.

```text
T_world_joint = T_room × T_world_tracker × T_tracker_joint
```

그다음 모든 tracker를 착용하고 똑바로 선 중립 자세에서 실행한다.

```bash
bash scripts/run_vive_linux_neutral_calibration.sh
```

이 명령은 방의 yaw/origin만 보정해
`config/local/calibration.vive_linux.json`을 만든다. 장착 offset을 자동으로 추정하지는
않는다.

## 6. 전신 입력 dry-run

먼저 로봇이나 Redis에 아무것도 보내지 않고 기록한다.

```bash
bash scripts/run_vive_linux_fullbody_dry.sh \
  --record runtime/vive_linux_fullbody.jsonl
```

정상일 때 주기 로그에 다음이 보이며 `SAFE STOP`이 없어야 한다.

```text
source=quest_openvr+vive_hub_linux reference=official_gmr
```

현재 이 명령은 pose→공간 정렬→normalization→sparse reconstruction→공식 GMR까지만
검증한다. 5개 tracker로 누락된 hips/spine/shoulders/elbows를 추정하는 경로는 아직
balance-safe하다고 입증되지 않았기 때문에 full-body Redis simulation publish와 실제
G1 출력은 코드에서 차단돼 있다. 기록 파일의 축, 좌우, dropout, root 높이와 MuJoCo
replay 안정성을 확인한 뒤 simulation launcher를 여는 것이 다음 단계다.

## 문제 해결

- Hub 로그: `~/.local/share/HTC/VIVEUltimateTracker/logs/`
- launcher 로그: `runtime/vivehub-launcher.log`
- `/tmp/vut.sock`이 없으면 Hub를 desktop user로 실행했는지 확인한다.
- 정상적인 pose 수집은 VIVE Hub v1의 `/tmp/vut.sock`을 사용한다. 공식
  `vive-ultimate-tracker-sdk`의 `/tmp/vive_tracker.sock`은 다른 프로토콜이므로 둘을
  동시에 실행하지 않는다.
- VIVE Hub v1.0.3에서 정지한 장치가 `unknown_18`로 표시됐던 것은 VUT 상태값에
  `PoseStable(0x10)` 비트가 포함되는 호환성 문제였다. 우리 provider는 알려진 정지
  비트만 분리해서 `0x12`를 `Ready(2)`로 읽고, 원래 값은 `raw_state`, 분리한 비트는
  `state_flags`에 보존한다. 실제 추적 손실·rotation-only·오래된 pose·그 밖의 미확인
  비트는 계속 무효로 처리하며, HTC 공식 바이너리는 수정하지 않는다.

### Hub에서 3개까지만 페어링되고 슬롯 0·3이 고정될 때

Hub의 reset 이후에도 슬롯 0과 3이 계속 `Connecting`으로 남는다면 동글에 ghost
pairing record가 남았을 수 있다. 이때만 Hub를 멈추고 공식 SDK 기반 복구 도구를 쓴다.

```bash
cd /home/railab/Desktop/vr_teleop/TWIST2
bash scripts/run_vive_linux_hub.sh --stop
bash scripts/run_vive_slot_repair.sh
```

복구 도구 안에서 다음 순서로 입력한다.

```text
status
unpair 0,3
status
pair 0
status
quit
```

펌웨어 9.17에서는 `pair 0`을 실행해도 현재 비어 있는 슬롯 전체가 pairing 상태로
열릴 수 있다. 따라서 새로 등록할 두 tracker만 파란색 점멸 pairing mode로 켜고,
이미 정상인 세 tracker는 pairing mode에 넣지 않는다. 복구 도구는 정상 슬롯 1, 2,
4에 대한 `pair`/`unpair` 명령을 거부한다.

종료한 뒤 Hub를 다시 실행하고 live inventory를 확인한다.

```bash
bash scripts/run_vive_linux_hub.sh --background
bash scripts/run_vive_linux_inventory.sh
```

서로 다른 device ID와 serial이 5개 나오면 복구 완료다. Hub의 `trackers.json` 저장이
한 장치 늦게 갱신되는 경우가 있으므로 이 파일만으로 성공 여부를 판단하지 않는다.
재시작 후에도 live inventory가 5개이고 Hub 로그에
`DeviceCountsPairedOnDongle: 5`가 남는지를 기준으로 확인한다.
