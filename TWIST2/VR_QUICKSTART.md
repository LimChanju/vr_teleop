# TWIST2 Quest 3 VR Quick Start

## 2026-11-01: 전신 입력 + TWIST2 정책 + MuJoCo 물리 테스트

**기존 `run_sparseposer_g1.sh`는 자세 미리보기다. 아래는 중력·접촉·토크 제어가
작동하는 별도 실험 경로이며 넘어질 수 있다. 실제 로봇에는 연결하지 않는다.**

기존 G1 미리보기만 `Ctrl+C`로 종료한다. VIVE Hub, 고정 SteamVR/ALVR 및 SpaceCal
전달 터미널은 유지하고 HMD·컨트롤러 2개·트래커 5개를 착용한다.

```bash
cd /home/railab/Desktop/vr_teleop/TWIST2
bash scripts/run_sparseposer_g1_physics.sh --calibration config/local/calibration.sparseposer_tpose.20261101.json
```

- 기존 공식 ONNX `twist2_1017_20k.onnx`와 시뮬레이터의 정책/PD/물리 루프를 사용한다.
  이 연결 작업은 정책, 관절 제한, PD 게인, 공식 SparsePoser/GMR 소스를 바꾸지 않는다.
- 로컬 전용 Redis `127.0.0.1:16479`를 새로 띄운다. 이미 사용 중이면 중단하며,
  종료 시 이번 실행이 생성한 입력 프로세스와 Redis만 종료한다.
- 별도 입력 프로세스에서 기존 SparsePoser/GMR를 실행한다. 유효한 목표는 추가
  gain/smoothing/clamp 없이 전달한다. 초기 2초 및 모델 준비 중에는 중립 목표를 쓰고,
  입력 손실/품질 미달/오래된 입력에서도 중립 목표로 전환한다. 이는 넘어짐 방지 보장이 아니다.
- `SIM INPUT: tracking` 이후 처음에는 정지하고, 이어 작은 팔·무릎 동작부터 확인한다.
  Quest에는 정면 3인칭 화면, PC에는 미러 창이 나온다. 오른쪽 스틱 클릭은 **시점 재정렬**이다.
- 종료 `Ctrl+C`. 자동 기립/자동 리셋은 없다. 넘어지면 종료 후 같은 명령으로 다시 시작한다.
- `runtime/sparseposer_physics/<시각>/`에 입력·G1 목표·실제 물리 상태 및 summary를 남긴다.
  `physics.jsonl`의 자세는 목표가 아니라 물리 시뮬레이터가 계산한 실제 자세다.
- `--viewer desktop`은 PC만 표시한다. `--viewer headless --seconds 15`는 유한 시간
  비표시 테스트다. `--recording <raw_poses.jsonl>`은 기록 재생이며 해당 기록의 보정을 지정해야 한다.
- 확인 범위: 기본 중립 5초, 새 정지 기록 일부를 넣은 물리 15초에서 넘어짐 지표 없음.
  후자의 최대 몸통 기울기 5.63°. 전체 동작/실시간 VR/보행 성공을 검증한 것은 아니다.
  정책 50Hz는 **시뮬레이션 시간 기준**이며 실제 벽시계 실행 속도는 CPU 부하에 따라 느릴 수 있다.

## 2026-10-29 수정: SparsePoser 입력 회전 부호

같은 회전인 `q`/`-q`가 신경망에는 다른 숫자로 들어가던 어댑터 오류를 수정했다.
공식 Unity 입력처럼 최초 부호는 identity 기준으로 선택하고, 이후에는 이전 입력과
연속성을 유지한다. 매 프레임 `w>=0`을 강제하는 방식이 아니므로 180°를 지나도
연속성이 유지된다. 공식 모델/GMR, 관절 한계, 기존 보정 파일은 변경하지 않았다.

기존 기록 `20261015T142449_449911Z` 전체를 재생하여 같은 시각의 7,219개 자세를 비교했다.
하체 피팅 기준 통과는 0개 → 6,187개(85.7%), 보정 후 왼발 방향 오차 중앙값은
33.25° → 3.58°로 개선됐다. **이 수치는 사람의 실제 동작 정확도나 균형 성공률이 아니다.**
G1 왼쪽 발목 roll은 여전히 하한 근처에 오래 머물러 리타게팅 추가 검증이 필요하다.
검증 출력: `runtime/sparseposer_g1/20261028_quaternion_stream_fix/`.
실행 명령은 아래와 같으며, 수정 반영에는 실행 중인 미리보기만 종료 후 다시 시작하면 된다.
부착 위치·사용자·추적 원점이 그대로라면 이번 코드 수정 때문에 재보정할 필요는 없다.

## 지금 실행: 전신 추정 → G1 자세 미리보기 (PC + Quest)

기존 사람 골격 미리보기가 실행 중이면 그 터미널에서 `Ctrl+C`로 종료한다.
**SteamVR/ALVR/VIVE는 그대로 유지**하고 HMD·양손 컨트롤러·트래커 5개를 착용한다.
PC 스크린 녹화는 원하는 프로그램에서 수동으로 시작한 뒤 아래 명령을 실행한다.

```bash
cd /home/railab/Desktop/vr_teleop/TWIST2
bash scripts/run_sparseposer_g1.sh
```

- Quest에 G1을 마주 보는 3인칭 화면, PC에 같은 VR 화면의 미러 창이 뜬다.
  오른쪽 스틱 클릭 또는 미러 창의 `R`은 **시점 재정렬**이다.
- 기존 `calibration.sparseposer_tpose.json`을 사용한다. 입력 8개가 모두 유효해지고
  약 1.05초의 연속 입력이 쌓이면 자세 추정을 시작한다. 모델 로딩 시간은 별도다.
- 자동 영상 녹화는 기본 **OFF**다. 종료는 `Ctrl+C` 또는 미러 창 닫기.
  입력·추정 골격·G1 관절·오차 기록은 `runtime/sparseposer_g1/<시각>/`에 저장한다.
- PC 창만 띄우려면 `bash scripts/run_sparseposer_g1.sh --viewer desktop`.
  이 경우에도 입력을 얻으려면 Quest를 착용하고 ALVR 연결을 유지해야 한다.
- 선택 사항: `--record`를 붙이면 별도 정면 카메라의 `g1_front.mp4`도 저장한다.
  `Ctrl+C`로 종료해야 영상 파일 마무리를 기다릴 수 있다. 기본 실행에는 인코더를 띄우지 않는다.

**현재는 공식 GMR의 `bvh_xsens → unitree_g1` IK 결과를 MuJoCo 모델에 적용하는
운동학적 미리보기다. TWIST2 정책·중력·균형 제어·실제 로봇 출력은 실행하지 않는다.**
동작의 방향과 관절 매핑을 보는 단계이며, 서거나 걷는 안정성 검증은 아니다.
공식 SparsePoser/GMR 소스는 수정하지 않고 별도 어댑터로 연결했다.
입력이 끊기면 마지막 자세를 유지하고 터미널에 상태를 표시한다.
최근 기록의 왼쪽 정강이/발 보정 오차는 아직 남아 있다. `fit_accepted=false`도
검사용 자세로 표시하며 정상 보정/로봇 실행 가능 판정으로 승격하지 않는다.
GMR의 기본 바닥 정렬을 사용하므로 점프 높이를 충실하게 재현하는 경로는 아니다.

검증: 실제 저장 입력에서 G1 120프레임 변환·표시 상태 갱신, 960×720/30fps
MP4 294프레임 생성 및 디코딩 통과. 현재 연결을 사용한 Quest+PC 동시 실시간
미리보기는 사용자가 실행해 확인할 단계다.

## 전신 추정: SparsePoser + 양쪽 정강이 보정 (2026-10-15 추가)

**이 절의 실행 경로는 사람 전신 22관절 추정/미리보기용이다. G1이나 실제 로봇은
움직이지 않는다.** 공식 SparsePoser 6입력 모델을 그대로 사용하고, 별도 어댑터가
정강이 2개의 위치·방향으로 양쪽 다리를 보정한다. 8입력 모델을 새로 학습한 것은 아니다.
공식 소스/가중치 추론과 오프라인 테스트는 확인했지만, 착용 상태 동작 정확도는 아직 미검증이다.

현재 PC에는 설치해 두었다. 다른 환경에서 재구성할 때만
`bash scripts/setup_sparseposer.sh`를 실행한다. 기존 `.conda-vr` 패키지를 바꾸지 않고
별도 `.venv-sparseposer`에 PyMotion을 설치하며, 공식 checkout은 수정하지 않는다.

1. 현재 정상 동작하는 ALVR/SteamVR/VIVE 세션을 유지한다. SpaceCal로 장치들이 몸 위치에
   정렬되어 있어야 하며, HMD + 물리 컨트롤러 2개 + 트래커 5개가 모두 유효해야 한다.
2. 아래 명령을 실행하고 **10초 안에 T자 자세**를 잡아 4초간 유지한다.
   정면을 보고, 발을 평행하게 두고, 두 팔을 어깨 높이로 옆으로 곧게 편다.
   컨트롤러는 양손에, 트래커는 실제 사용할 부착 위치에 둔다.

```bash
cd /home/railab/Desktop/vr_teleop/TWIST2
bash scripts/run_sparseposer_fullbody.sh calibrate --height 1.70
```

3. 완료 후 같은 터미널에서 실행한다. 팔·무릎을 천천히 움직이며 **데스크톱 사람 골격 창**을
   확인한다. 파란 선은 추정 골격, 주황 점은 실제 센서 중심이라 정확히 포개지지 않는 것이 정상이다.

```bash
bash scripts/run_sparseposer_fullbody.sh live --preview
```

Quest에 새 영상을 띄우거나 G1 시뮬레이션을 시작하는 명령은 아니다. 종료는 `Ctrl+C`.
창 없이 추정/기록만 하려면 `--preview`를 빼면 된다. 예: `live --seconds 30`.
기존 `run_quest_arms_vr.sh`는 이 신규 전신 추정기를 사용하지 않는다.

보정 성공 후에는 `calibrate`를 반복하지 않고 `live --preview`만 실행한다.
`preserving existing calibration`은 기존 성공한 보정 파일을 보호하는 메시지다.
단, 부착 위치나 공간 원점이 바뀌었다면 아래 설명대로 새 파일에 재보정한다.
실행 중에도 HMD를 착용하고 컨트롤러 2개를 손에 들어 추적을 유지한다.
창에 `Waiting: Left_Wrist, Right_Wrist`가 나오면 양손 입력이 없는 상태다.
주황 센서 점만 표시하고 골격은 만들지 않는다. 8개 입력이 복구되면 약 1.05초의
연속 추적 기록을 모은 뒤 골격이 나온다. 대기/추적 손실 중에도 창은 갱신된다.

부착 매핑은 기존 파일을 읽는다: `0072=왼발등`, `0314=오른발등`,
`0320=왼쪽 정강이`, `0086=오른쪽 정강이`, `0082=배쪽 허리`.
정강이를 허벅지로 옮기면 이 보정 모델과 맞지 않는다.

출력은 `runtime/sparseposer/<시각>/`의 `raw_poses.jsonl`, `body_estimates.jsonl`,
`run.json`, `summary.json`이다. 추정 파일에는 22관절 world 위치/회전, local 회전,
양쪽 정강이·발의 오차와 수렴 상태가 들어간다. `fit_accepted=false`는 보정 오차 초과이며,
`tracking_blocked`는 입력 누락/추적 끊김이다. 어떤 상태도 로봇에 자동 전송하지 않는다.
기록은 실행하는 동안 계속 커지므로 테스트가 끝나면 종료한다.

T-pose 프로필: `config/local/calibration.sparseposer_tpose.json`.
기존 차렷 자세 `mount_capture/summary.json`은 이 프로필 대신 쓸 수 없다.
사용자/부착 위치/공간 원점이 바뀌면 다시 보정한다. 기존 프로필은 덮어쓰지 않으므로
재보정 시 `--calibration config/local/calibration.sparseposer_tpose.v2.json`을
**calibrate와 live 두 명령에 똑같이** 붙인다. SpaceCal 프로필 파일 변경은 자동 감지하지만,
모든 런타임 원점 재설정을 자동으로 감지한다고 보장하지 않는다.

오프라인 재생(해당 기록과 맞는 T-pose 프로필 필요):

```bash
bash scripts/run_sparseposer_fullbody.sh replay --recording runtime/sparseposer/<시각>/raw_poses.jsonl --calibration config/local/calibration.sparseposer_tpose.json
```

주의: 신체 비율은 170cm 공식 템플릿을 키에 맞춰 균일 확대/축소한 근사치이고,
센서→해부학적 관절 오프셋도 T-pose 기반 추정치다. GMR/G1 연결 전 골격과 보정 오차를
확인해야 한다. 이 출력은 `xrobot` 관절 입력과 이름/로컬 축 규약이 같지 않아
그대로 이름만 바꿔 GMR에 넣으면 안 된다. 위 G1 실행기는 별도 BVH 변환을 사용한다. 상세 구조는
[아키텍처의 SparsePoser 절](VR_TELEOP_ARCHITECTURE.md#sparseposer--양쪽-정강이-어댑터)을 참고한다.

## 현재 실행 상태 — 실제 트래커 5개와 영상 동시 검증 통과 (2026-10-15 20:32)

**20:32에 실제 VIVE 트래커 5개 + HMD + 물리 Quest 컨트롤러 2개의 유효 포즈를
동시에 확인했다.** OpenVR 8초/80회 검사에서 8개 장치 모두 연결·유효 80/80,
유한한 좌표였다. VIVE 원본도 5초간 5개 모두 Ready였고 각 100개의 새 타임스탬프를
받았다(관측 포즈 나이 50ms 미만). 이전에 실패했던 0072도 이번 구간에서는 정상이다.

- 동시 ALVR 10초/718프레임 통계: 총 지연 중앙값 약 50ms, 약 72fps,
  Desync 0회, 보고 패킷 손실 0. **영상 우선 시작 후 실제 트래커 5개를 추가하는
  순서가 이번 세션에서 정상 동작함을 확인했다.** 장시간·재시작 반복 검증은 별개다.
- 공간 정렬 정확도, 신체 부착 보정, 전신 리타게팅/시뮬레이터 teleop까지 통과했다는
  뜻은 아니다. 이 검사에서는 로봇/시뮬레이터 명령을 보내지 않았다.

### 현재 세션 시작 순서 (18:08 시작)

사용자가 영상 정상임을 확인한 후 VUT 드라이버를 재등록하고 같은 고정 SteamVR을
재시작했다. **송신부 OFF 상태에서 신선한 ALVR 통계 6초(약 47ms, Desync 0회),
HMD 유효 포즈, 등록 트래커 0개를 확인한 다음에만 실제 VIVE 피더를 시작했다.**

- 현재 Space Calibrator 드라이버/UI와 VUT 드라이버/실제 피더 모두 실행 중이다.
  피더 서비스: `twist2-vut-only-ab.service`; 로그: `runtime/spacecal/vive-late-start-bridge.log`.
  보정 UI 서비스: `twist2-spacecal-only-ab.service`(이름은 이전 비교 실험에서 유지).
- 18:08 피더 시작 직후에는 약 47ms / 72fps / Desync 0회였지만 트래커가 모두
  offline이었다. 실제 유효 포즈를 포함한 후속 결과는 위의 20:32 검사와 구분한다.
- ALVR 설정/보정값 변경 없음. 백업: `runtime/spacecal/vive-late-start.8cgFmvjT/`.
- 시작 순서는 이번에 수동으로 지켰으며, 자동 시작 가드를 구현한 것은 아니다.
  SteamVR 재시작 전에는 피더를 먼저 종료하고, 재시작 후 ALVR 첫 영상 확인 뒤 다시 켠다.

## 영상 지연 재현 조건 확인 — 트래커 포즈 시작 순서 (2026-10-15 16:57)

**실제 HTC 데이터를 사용하지 않는 가상 트래커로도 영상 지연을 재현했다.**
같은 SteamVR `2.12.14` / ALVR `20.14.1` 설정으로 비교했고 Space Calibrator는
분리 실험 내내 제외했다. 아래 수치는 ALVR의 신선한 이벤트 통계 중앙값이다.

| 조건 | 총 지연 추정 | Desync 경고 |
| --- | --- | --- |
| VUT 드라이버만 로드, 송신 없음 | 48ms | 8초간 0회 |
| 가상 장치 5개, 연결됨/포즈 무효 | 48ms | 15초간 0회 |
| 정상 영상 이후 같은 5개에 고정 유효 포즈 공급 | 44ms | 12초간 0회 |
| 새 런타임에서 영상 전부터 유효 포즈 공급, 이후 재활성화 | 468ms | 25초간 70회 |
| 위 오류 이후 송신 종료 | 418ms | 6초간 32회 |

- 오류 조건에서 실제 OpenVR 가상 트래커 5개의 유효 포즈를 확인했고, 사용자도 같은
  화면 이상을 확인했다. 실제 VIVE 움직임·SDK 타임스탬프 없이도 발생한다.
- 정확한 시간 순서: 첫 30초 가상 송신은 HMD 연결 전에 끝났다. 같은 런타임에서
  HMD 연결 후 동일 장치의 유효 포즈를 다시 공급했을 때 오류를 수집했다.
  따라서 '트래커를 계속 켜둔 채 HMD 연결'의 완전한 반복 실험은 아직 아니다.
- 유력 원인은 ALVR Linux의 초기 포즈 선택/영상 타임스탬프 연결이다.
  설치된 버전은 최초로 찾은 포즈 포인터를 캐시하며 장치 ID/HMD 여부를 검사하지 않는다.
  **실제 잘못 선택된 메모리 포인터를 직접 확인한 것은 아니므로 내부 원인을 확정하지 않는다.**
- [ALVR 공식 Linux 문서](https://github.com/alvr-org/ALVR/wiki/Linux-Troubleshooting#the-view-shakes-when-using-slimevr)와
  [동일 계열 시작 순서 이슈](https://github.com/alvr-org/ALVR/issues/2737)도
  ALVR에서 첫 영상을 받은 다음 추가 트래커 서버를 시작하도록 안내한다.
- 이후 실제 VIVE 검증은 **SteamVR 시작 → Quest ALVR 첫 영상 정상 확인 → VIVE 브리지 시작**
  순서로 진행한다. 단순히 몇 초 대기하거나 SteamVR 창만 뜬 것으로 판단하지 않는다.
  브리지 자동 시작 가드는 아직 구현하지 않았다. 20:32 실제 Ready 트래커 5개를
  포함한 단기 영상 검증은 통과했으며, 재시작 반복 검증은 별개다.
- 실험 종료 후 가상 송신을 종료하고 SteamVR을 새로 시작하여 가상 장치를 제거했다.
  16:57 복구 당시에는 **Space Calibrator ON / VIVE 브리지 및 VUT 드라이버 OFF**였다.
  보정값·ALVR 설정·공식 소스는 변경하지 않았다.
  복구 직후 ALVR은 연결 해제 상태여서 새 영상 통계는 없었다. Quest 재연결 후
  정상 영상 여부를 확인해야 하며, 복구 후 화면까지 검증했다고 간주하지 않는다.
- 진단 도구만 별도 추가: `twist2_vr.vut_bridge_probe`(최대 30초 가상 송신),
  `twist2_vr.alvr_stats_probe`(읽기 전용 통계 수집). 관련 테스트 35개 통과.
  이 도구의 송신 성공 수는 OpenVR 포즈 수신 확인을 대신하지 않는다.
- 상세 수치와 실험 한계: `runtime/diagnostics/20261015_vut_isolation.json`.
  실행 전 설정 백업: `runtime/spacecal/vut-isolation.6FhDAyfI/`.

## Space Calibrator 단독 비교 (2026-10-15 16:20)

**사용자 요청으로 반대 조건을 비교한다: Space Calibrator ON / VIVE 브리지 OFF.**
외부 드라이버는 ALVR + `01spacecalibrator`뿐이며, 실제 서버에서 Space Calibrator가
로드되고 `twist2_vut`는 빠진 것을 확인했다. VUT 피더 서비스도 종료했다.
같은 고정 SteamVR `2.12.14`를 재시작했으며 VIVE Hub는 계속 실행 중이다.

- 보정 UI만 `twist2-spacecal-only-ab.service`로 직접 실행했다. 앱이 시작하면서 자동 실행을
  켜므로 실행 후 다시 `false`로 돌렸다(실행 중인 UI에는 영향 없음).
  `run_vive_spacecal.sh`는 VUT도 켜므로 이번 비교에서는 사용하지 않는다.
- ALVR 설정과 기존 보정값은 수정하지 않았다. 백업:
  `runtime/spacecal/spacecal-only-ab.k4PMwFhg/`. UI 로그:
  `runtime/spacecal/spacecal-only-overlay.log`.
- Quest에서 ALVR에 다시 연결하여 고개를 천천히 움직였을 때 화면 지연/검은 테두리가
  재현되는지 확인한다. VIVE 트래커가 SteamVR에 안 보이는 것은 정상이다.
  새 보정은 실행하지 않는다. 사용자가 이 조건에서는 화면이 정상이라고 확인했다.
  새 5초 통계에서도 약 72fps, 총 지연 중앙값 51ms, 동기화 오류 0회를 확인했다.
- 비교 결론: Quest 단독 정상, VIVE만 추가하면 재발, Space Calibrator만 추가하면 정상.
  VUT 브리지→OpenVR 드라이버→ALVR 연동으로 범위가 좁혀졌지만, 브리지 자체 결함인지
  ALVR의 추가 장치 포즈 처리 문제인지는 아직 확정하지 않았다.

## VIVE만 추가한 비교 결과 (2026-10-15 16:14–16:16)

**사용자 승인으로 `twist2_vut` 드라이버와 원본 포즈 피더만 다시 켰다.**
외부 드라이버는 ALVR + VUT이며, Space Calibrator 드라이버는 등록 해제/미로드 상태를
유지한다. 보정 앱 자동 실행도 `false`다. 같은 SteamVR `2.12.14`로 재시작했다.
ALVR 설정과 보정 파일은 변경하지 않았다. 백업: `runtime/spacecal/vive-only-ab.yUugd58C/`.

- 피더는 `twist2-vut-only-ab.service` 사용자 서비스로 실행한다. 로그:
  `runtime/spacecal/vive-only-bridge.log`. 종료는 `systemctl --user stop twist2-vut-only-ab.service`.
- 공간 보정 세션 대신 `python -u -m twist2_vr.vut_openvr_bridge`만 실행했다.
  트래커 위치가 Quest와 어긋나는 것은 이번 비교에서 예상되는 상태다. 재보정하지 않는다.
- 16:14:39 기준 피더가 포즈 패킷을 전달 중이며 0072/0320은 Ready,
  나머지 3개는 offline이었다. 5개 등록과 5개 Ready는 다르다.
- 사용자가 VIVE만 켠 상태에서 화면 이상이 다시 발생했다고 확인했다.
  16:16의 6초 수집에서 ALVR `Desync detected. Attempting recovery.` 경고가 35회,
  총 지연 추정 중앙값은 638ms였다(인코더 1.78ms, 디코더 6.77ms, 서버 약 72fps).
  Space Calibrator가 없어도 재현되므로 VUT→OpenVR→ALVR 연동 경로가 유력하지만,
  특정 코드 결함이나 트래커 하드웨어 고장까지 확정된 것은 아니다.
  이후 피더를 종료하고 VUT 드라이버 등록을 해제했다.

## Quest 단독 비교 결과 (2026-10-15 16:06)

**사용자 승인으로 VIVE 브리지와 Space Calibrator를 SteamVR에서 임시 제외했다.**
같은 SteamVR `2.12.14`를 재시작했고 실제 서버 메모리에서 두 드라이버가 빠진 것을 확인했다.
이 비교 당시 외부 드라이버 등록은 ALVR만 남았다. 따라서 SteamVR에 VIVE 트래커 5개가
보이지 않는 것이 정상이다. Space Calibrator 앱 등록은 남겨두고 자동 실행만 껐다.
프로젝트의 공간 보정 세션과 VUT 피더도 종료했다. VIVE Hub/트래커 서버는 계속 실행 중이다.

- ALVR 설정(위치/방향 재정렬 모두 `Disabled`)과 보정 파일은 변경하지 않았다.
- 백업: `runtime/spacecal/quest-only-ab.q8K0fmGi/` (드라이버 등록, 앱 자동 실행 설정,
  ALVR 설정, 보정 파일, 변경 전 SteamVR 로그).
- 16:10 사용자가 이 구성에서 화면이 정상으로 돌아왔다고 확인했다. ALVR에 남은 마지막
  평균 지연 추정값은 47ms였다(이전 약 601ms). 확인 시점에는 스트림이 끊겨 있어 새 구간별
  통계는 수집되지 않았다. 물리 Touch 컨트롤러 정상화는 아직 별도 확인이 필요하다.
- 복귀 시 기존 두 드라이버 경로를 `vrpathreg adddriver`로 재등록하고
  `spacecalibrator.linux` 자동 실행을 원래의 `true`로 복구한 뒤, 고정 SteamVR과
  `scripts/run_vive_spacecal.sh`를 다시 시작한다. 재빌드·재페어링·보정 파일 복원은 필요 없다.
- 이번 비교에는 서버 재시작도 포함되므로, 개선되더라도 추가 드라이버가 원인이라고
  즉시 확정하지 않는다. 다시 활성화했을 때 재발하는지 확인해야 한다.

## 2026년 10월 14일 추적 장애 조사

**0072 추적 손실, 트래커 공통 위치 오프셋, Quest 양손 컨트롤러 연결 끊김의 최신 진단은
[추적 장애 조사와 복구 순서](VR_TRACKING_DIAGNOSIS_20261014.md)에 정리했다.**
Quest 자체 컨트롤러와 VIVE 원본 추적을 먼저 정상화하고 공간 정렬을 확인한다.
현재 전신 teleop 검증은 완료되지 않았으며, 초기화나 재보정부터 반복하지 않는다.

## SteamVR 복구 버전 실행 — 2.12.14 고정

**2026-10-14: SteamVR `2.12.14` 복구 및 실제 실행 버전 확인 완료.**
ALVR `20.14.1`과 함께 compositor의 `Startup Complete`를 확인했고, 업데이트 후 발생한
`496 / CannotDRMLeaseDisplay` 시작 오류는 복구한 런타임에서 재발하지 않았다.
OpenVR에서 HMD와 양쪽 컨트롤러의 유효 포즈도 수신했다. Quest 안의 실제 영상 품질과
사용자 공간 보정 완료 여부는 별도다. 초기 위치 검증 실패 후 ALVR 재정렬 설정을 변경했고,
22:31에는 실제 오른손 컨트롤러와 0314의 보정 및 독립 이동 검증이 성공했다.
그 뒤 VIVE host 교체와 추적 장애가 발생했으므로 현재 전체 장치 정렬은 위 조사 문서를 따른다.

### 재실행 시 버전 고정 — 2026-10-15

**이 PC의 SteamVR 시작 경로를 `2.12.14`로 고정했다.**
오늘 `2.16.7`이 다시 표시된 것은 매번 다운로드된 것이 아니라, Steam의 시작 옵션이
관리 폴더의 `vrmonitor.sh`를 직접 실행했기 때문이다. 실행창 `2.16.7`과 서버
`2.12.14`가 섞이면서 오류 `105`도 발생했다.

- SteamVR의 Steam 시작 옵션은 `scripts/steamvr_pinned_entry.sh %command%`를 호출한다.
  이 어댑터는 Steam이 넘긴 관리 버전 명령을 실행하지 않고 검증된 고정본만 시작한다.
  ALVR `20.14.1`의 시작/재시작 버튼도 같은 Steam URI를 사용하므로 이 경로를 따른다.
- SteamVR 업데이트 설정은 **높은 우선순위 → 실행할 때만**으로 변경했다.
  이는 백그라운드 자동 업데이트를 막지만, Steam 라이브러리로 시작할 때 관리용 복사본의
  업데이트 확인·다운로드까지 금지하는 설정은 아니다. 고정본은 그 폴더 밖에 있어 별개다.
- **SteamVR 업데이트 확인을 거치지 않으려면** Steam과 ALVR Dashboard를 켠 뒤
  바탕화면의 **`SteamVR 2.12.14 (Pinned)`** 또는 아래 명령을 사용한다.
  이 설정은 Steam 클라이언트 자체의 업데이트나 다른 게임의 업데이트에는 영향을 주지 않는다.
- 변경 전 시작 옵션·SteamVR 앱 설정·바로가기 백업:
  `runtime/spacecal/launch-pin-backup.Lo2dN10z/`.
  `localconfig.vdf`에는 계정 관련 정보도 있으므로 공유하거나 Git에 추가하지 않는다.
- 검증: 2026-10-15 14:45 일반 Steam URI로 시작, 종료 후 14:46 같은 URI로 재실행했다.
  두 실행 모두 `vrmonitor`·`vrserver`·`vrcompositor`가 `2.12.14`였고 compositor의
  `Startup Complete`를 확인했다. 실행 어댑터 테스트 6개도 통과했다.

Steam과 ALVR Dashboard를 켠 뒤 실행한다.

```bash
cd /home/railab/Desktop/vr_teleop/TWIST2
bash scripts/run_steamvr_pinned.sh
```

명령은 SteamVR을 사용자 백그라운드 서비스로 시작하고 터미널로 돌아온다.
같은 터미널에서 아래 공간 정렬 명령 또는 기존 상체 테스트 명령을 실행하면 된다.
이미 복구 버전이 실행 중이면 중복 실행하지 않는다. 다른 버전이 실행 중이면
먼저 SteamVR 창에서 종료한다. 이 스크립트는 다른 실행을 강제 종료하지 않는다.

- 고정 설치: `runtime/steamvr-2.12.14/` (Valve 배포본, ALVR 공식 compositor wrapper 사용).
- Steam 관리 폴더의 `2.16.7`은 덮어쓰지 않았다. 시작 옵션으로 실행만 고정본에 연결했으며,
  Valve·ALVR 배포 파일을 개조하거나 관리 폴더를 읽기 전용으로 잠그지 않았다.
- Valve 바이너리나 xi-ve 원본 코드를 패치하지 않았다. xi-ve 빌드 의존성만
  OpenVR SDK `v2.12.14`로 맞춘다(`IVRSystem_023`; 최신 SDK 사용 시 오류 `105`).
  설정 창은 시스템 OpenGL 라이브러리를 우선 사용하도록 빌드하여 정상 실행을 확인했다.
- 기존 런타임/설정 백업: `runtime/spacecal/steamvr-before-restore.Og1MiL/`.
  원본 depot manifest/출처는 `SOURCE_PROVENANCE.json`, 핵심 파일 검증값은
  `config/steamvr_2.12.14.sha256`에 기록했다.
- 버전/프로세스 확인: `bash scripts/run_steamvr_pinned.sh --check`.
- 종료: SteamVR 창에서 종료하거나 `systemctl --user stop twist2-steamvr-pinned.service`.
- 실행 로그: `runtime/spacecal/steamvr-pinned-launch.log`.
- 이 버전 고정은 화면 과민/어지러움이나 트래커 공간 정렬의 해결을 의미하지 않는다.
  ALVR 움직임 예측 설정과 트래커 보정값은 이번 변경에서 수정하지 않았다.

## 새 공간 정렬 경로: xi-ve Space Calibrator + VIVE → SteamVR

**2026-10-14: 복구한 `2.12.14`에서 두 드라이버와 xi-ve 보정 파일 적용을 확인했다.
22:31의 단일 장치 쌍 검증은 성공했지만 이후 장애가 생겨 전체 장치 정렬은 재검증이 필요하다.**
xi-ve의 OpenVR 초기화/드라이버 연결 및 실제 데스크톱 창 표시까지 확인했다.
초기 확인용 UI/포즈 전달 세션은 종료했으며, 이후 21:52부터 다시 연 세션으로 보정했다.
앞서 Steam 재실행 시 대기 업데이트가 적용되어 `2.16.7`/ALVR `20.14.1` 조합에서
`496` 및 xi-ve compositor 연결 오류 `307`이 발생했다. 위의 별도 고정 설치로 복구했다.
Steam의 `previous` 브랜치가 원하는 버전을 가리킨다고 가정하지 않는다.

실제 `vrserver` 메모리와 로그에서 `01spacecalibrator`, `twist2_vut` 두 드라이버의 로드를 확인했고,
OpenVR에서 VIVE 시리얼 5개가 GenericTracker로 열거되는 것도 확인했다.
이는 **5개 모두 Ready 또는 공간 정렬 완료**를 뜻하지 않는다. 확인 중 `0072`/`0082`의 유효 포즈를
수신했고 나머지에는 추적 상실/오프라인 상태가 있었다. 이는 초기 복구 검사 당시의 기록이며
22:31 이후 결과와 현재 장애는 위 최신 조사 문서에 구분해 기록했다.
진단 로그: `runtime/spacecal/restart-check.Cyzti7/`.
관련 보고: [ALVR #3296](https://github.com/alvr-org/ALVR/issues/3296),
[SteamVR Linux #901](https://github.com/ValveSoftware/SteamVR-for-Linux/issues/901).

xi-ve 원본은 `external/openvr-space-calibrator-linux`에 수정 없이 보존한다.
기존 상체 실행 명령은 그대로다. 아래 명령은 **공간 정렬용이며 로봇을 움직이지 않는다.**

```bash
cd /home/railab/Desktop/vr_teleop/TWIST2
bash scripts/run_vive_spacecal.sh
```

최초 설치 후에는 위 고정 실행기로 SteamVR을 한 번 재시작해야 한다. 이미 복구 버전과
두 드라이버가 실행 중이면 다시 시작할 필요 없다. VIVE Hub/ALVR을 켜고 트래커 5개와
양손 컨트롤러를 깨운다. 이 터미널을 유지하면 원시 VUT 포즈가 SteamVR으로 전달되고
xi-ve 창이 열린다. `Ctrl+C`는 이 세션의 전달 프로세스를 종료한다. 기존에 실행 중이던
xi-ve 창은 종료하지 않는다.

- Reference: Quest **오른손 컨트롤러**. `VRLINKQ_Hand_Right` 같은 합성 손 장치가 아님.
- Target: tracking system **`twist2_vut`**, serial **`FA61N3B00314`**.
- 두 장치를 서로 움직이지 않게 고정하고 일반 `Start Calibration`을 사용한다.
  재검증 시 속도는 **`Very Slow`**로 선택한다. 두 기기를 함께 천천히 좌우·위아래·앞뒤로
  이동하고 여러 방향으로 돌린다. 손 안에서 두 기기의 상대 위치·방향은 유지한다.
  `Continuous Calibration`/`Playspace Movement`는 처음에는 사용하지 않는다.
- 정렬 후 몸에 다시 부착하기 전에, 다른 위치와 방향에서도 두 장치가 함께 맞는지 확인한다.
  트래커 장치 원점과 컨트롤러 장치 원점은 물리적으로 일치하지 않으므로 0cm가 정답은 아니다.
- 트래커 5개는 같은 tracking system으로 등록되므로 같은 공간 변환의 적용 대상이다.
  `lost_tracking`이나 `rotation_only`는 보정으로 복구되지 않는다.

연결 구조: `VIVE Hub → 별도 VUT bridge → SteamVR → xi-ve 공간 정렬 → OpenVR 입력`.
이 경로에는 기존 `calibration.quest_vive.json`을 다시 적용하지 않는다.
기존 직접 VUT 경로/보정 파일은 삭제하거나 덮어쓰지 않는다.

### 허리 트래커를 제외한 부분 자세 측정

0082(Pelvis)가 추적되지 않을 때는 아래 명령으로 HMD, 양손 컨트롤러,
양쪽 정강이·발등 트래커 **7개만 정지 자세 진단**할 수 있다.
현재 SteamVR·ALVR·VIVE 브리지·Space Calibrator 세션은 그대로 유지한다.

```bash
cd /home/railab/Desktop/vr_teleop/TWIST2
bash scripts/run_vive_spacecal_mount_capture.sh --exclude-role Pelvis
```

10초 준비 시간 뒤 정면을 보고 팔을 내린 채 5초간 정지한다.
결과는 `runtime/mount_capture/<시각>/summary.json`에 부분 측정으로 명시된다.
허리 원시 데이터가 있으면 기록에는 남지만 안정성 판정에서는 제외한다.
이 옵션은 측정 스크립트에만 적용되며, 허리 포즈를 만들어 넣거나 전신 teleop의
입력 요건을 변경하지 않는다. 기존 보정값은 변경하지 않고 로봇 명령도 보내지 않는다.

### 컨트롤러 없이 하체와 머리 6개 측정

허리(0082) 추적은 돌아왔지만 양손 컨트롤러가 없을 때는 다음 명령을 사용한다.
**HMD·허리·양쪽 정강이·양쪽 발등 6개**가 모두 유효해야 하며, 양손 컨트롤러 상태만
이번 정지 자세 판정에서 제외한다. 현재 SteamVR·ALVR·VIVE·Space Calibrator는 유지한다.

```bash
cd /home/railab/Desktop/vr_teleop/TWIST2
bash scripts/run_vive_spacecal_mount_capture.sh --lower-body-head
```

HMD를 착용하고, 10초 준비 시간 뒤 정면을 보고 발을 평행하게 놓은 채 5초간 정지한다.
`--exclude-role Pelvis`와 함께 쓰지 않는다. 결과에는 `capture_mode: lower_body_head`와
`partial_capture: true`가 기록되며, 수신된 컨트롤러 원시 데이터도 보존한다.
이것은 하체·머리 센서 측정일 뿐, 신체 부착 보정이나 하체/전신 로봇 제어를 수행하지 않는다.

### ALVR 재정렬 설정 — 2026-10-14 적용

**2026-10-15 15:52: 위치·방향 재정렬 설정을 다시 모두 `Disabled`로 저장했다.**
15:46에 `LocalFloor`/`Yaw`로 변경하여 사용자가 재연결 후 비교했으나 화면 증상이 같았고,
사용자의 요청으로 두 항목만 되돌렸다. SteamVR 버전·영상 설정·트래커 보정은 유지했다.
원래 Disabled 설정/보정 백업: `runtime/spacecal/alvr-recenter-ab.R8hp2aue/`.
실패한 A/B 설정 백업: `runtime/spacecal/alvr-recenter-return.GKsGZQay/session.before.json`.
저장된 값은 서버 API에서도 확인했지만, 기존 스트림의 원점에 적용하려면 Quest ALVR을
재연결해야 한다. Space Calibrator 재보정이나 로봇 teleop은 실행하지 않았다.
15:52–15:53 연결 중 `Desync detected. Attempting recovery.` 경고를 반복 확인했다.
인코딩/디코딩은 각각 중앙값 약 1.8/6.7ms였고 ALVR의 `game_time_s`는 약 581ms였다.
이 값은 자세 수신→영상 제출의 타임스탬프 매칭에 의존하므로 실제 GPU 렌더 시간이
581ms라는 뜻은 아니다. [측정 요약](runtime/diagnostics/20261015_alvr_desync_summary.json)을 참고한다.

아래는 10월 14일 변경 기록이다.

ALVR `20.14.1`의 위치 재정렬(`position_recentering_mode`)과 방향 재정렬
(`rotation_recentering_mode`)을 모두 **`Disabled`**로 변경했다. 이전 값은 각각
`LocalFloor`, `Yaw`였다. ALVR 실행 중인 서버 API와 저장 설정에서 변경을 확인했다.
기존 ALVR 설정과 xi-ve 프로필 백업:
`runtime/spacecal/alvr-recenter-before.P2FVvn9p/{session,profile}.json`.

이는 ALVR이 추가 적용하는 원점 보정을 끄는 설정이며, Quest/VIVE 공간 자체를 고정하거나
추적 상실을 해결하는 설정은 아니다. ALVR 재정렬이 기존 오차의 원인인지는 아직 미확정이다.
설정 변경 후 Quest의 ALVR 앱을 재연결하고 **오른손 컨트롤러 + 0314로 다시 보정**한다.
SteamVR 복구 버전이나 VIVE 공간 맵을 재설정할 필요는 없다.
xi-ve의 완료 메시지만으로 통과 처리하지 않고, 새로운 움직임에서 정렬을 다시 검증한다.

설치 재현(빌드 도구는 `.spacecal-build-env`에 격리; sudo 불필요):

```bash
bash scripts/setup_space_calibrator.sh --install
```

이미 드라이버가 로드됐다면 SteamVR을 먼저 종료해야 재빌드할 수 있다. 설치 전 등록 설정은
`runtime/spacecal/backup.*`에 보존한다. xi-ve 로그는 `~/.local/share/space-calibrator/`,
창 실행 로그는 `runtime/spacecal/overlay-launch.log`에 있다.
설치/빌드 성공과 실제 공간 보정 성공은 별개이며, 사용자 움직임을 통한 보정은 추가로 필요하다.
**신체 부착 보정과 전신 관절 방향 추정이 완료된 상태는 아니다.**

되돌리려면 `bash scripts/unregister_space_calibrator.sh` 후 SteamVR을 재시작한다.
이 프로젝트가 추가한 두 드라이버 등록만 제거하며 ALVR, 소스, 기존 보정 파일은 유지한다.

## 기존 직접 VUT 경로: 0314 + 오른손 공간 정렬과 검증을 한 번에

**트래커 5개를 켜두고, 0314(`FA61N3B00314`)와 오른손 컨트롤러를 서로 움직이지
않게 고정한 뒤 실행한다.** 카메라를 가리지 않는다. 몸을 회전시키는 검사가 아니라
손에 든 두 기기를 하나의 물체처럼 천천히 이동·회전시키는 검사다.

```bash
cd /home/railab/Desktop/vr_teleop/TWIST2
bash scripts/run_vive_linux_alignment_check.sh FA61N3B00314
```

두 장치 추적 준비 → **15초 준비 → 20초 보정 수집 → 12초 별도 검증 수집**이
자동으로 이어진다. 중간에 Enter를 누르거나 손을 고쳐 잡지 않는다. 보정 때는 8자 이동과
여러 축 회전을 하고, 검증 때도 같은 고정을 유지하며 다른 위치·각도로 천천히 움직인다.
`CAPTURE FINISHED`가 나오면 내려놓아도 된다. 계산은 두 수집 단계가 끝난 뒤 진행한다.
준비 대기와 계산 시간을 제외한 기본 진행 시간은 약 47초다.

영어 음성 안내는 PC 출력이므로 Quest에서 들리는지는 보장하지 않는다. 소리를 듣기
어려우면 옆 사람이 터미널 안내를 읽어주도록 하거나 `--no-speech`를 붙인다.
검증은 새 데이터에 **보정 때 계산한 두 변환을 그대로 적용**하며 다시 fitting하지 않는다.

실행마다 `runtime/alignment_check/<시간>/`에 `session.log`, `poses.jsonl`, `summary.json`이
남는다. 실패·Ctrl+C 때도 이미 수집한 데이터가 보존된다. 통과한 실행에만 그 폴더의
`candidate.json`이 생기며 **기존 사용 보정값과 `config/local`의 후보 파일은 덮어쓰지 않는다.**
통과는 해당 고정 상태와 움직인 범위에 대한 검사 결과이며, 0072 추적 복구·신체 장착 보정·
전신 teleop 완료를 뜻하지 않는다. 상세 기준은 [VIVE Linux 안내](VIVE_LINUX_QUICKSTART.md#4-quest-공간과-vive-공간-정렬)에 있다.

## 전신 준비: Quest + VIVE 5개 좌표 검사 한 번에

통신/정지 추적 확인 뒤에는 아래 명령 하나로 다섯 곳의 좌표 비교를 연속 수행한다.
**로봇을 움직이는 명령이 아니라 보정 전 확인 도구**다.

```bash
cd /home/railab/Desktop/vr_teleop/TWIST2
bash scripts/run_vive_linux_validation.sh
```

오른손 컨트롤러 본체를 `배 → 오른쪽 정강이 → 오른쪽 발등 → 왼쪽 발등 → 왼쪽 정강이`
tracker 바로 옆에 순서대로 가져간다. tracker는 떼지 않고 카메라를 가리지 않는다.
각 단계 10초 준비 + 5초 정지, 약 75초 자동 진행이며 중간 확인 입력은 없다.
PC 음성 안내가 Quest에서도 들리는지는 환경에 따라 달라 첫 안내를 확인한다.
안 들리면 옆 사람이 터미널 안내를 읽어주도록 한다. 자세 변경에 시간이 더 필요하면
`--countdown 20`, 음성 없이 실행하려면 `--no-speech`를 붙인다.

결과는 `runtime/spatial_check/<시간>/summary.json`에 모인다. 이 검사는 공식 코드나
보정값을 바꾸지 않는다. 통과해도 장착 보정·전신 자세 변환·MuJoCo 정책 검증이 남는다.
자세한 의미와 단일 지점 재검사는 [VIVE Linux 안내](VIVE_LINUX_QUICKSTART.md#4-1-재시작추적-복구-후-가까운-두-장치의-좌표-비교)를 참고한다.

## 첫 번째 테스트: Quest 3와 양손 컨트롤러만 사용

ALVR client가 SteamVR에 연결되고 양손 Touch controller가 켜진 상태에서 아래 명령
하나만 실행한다.

```bash
cd /home/railab/Desktop/vr_teleop/TWIST2
bash scripts/run_quest_arms_vr.sh
```

이 명령은 한 터미널에서 다음을 모두 시작한다.

- 외부에서 접근할 수 없는 전용 Redis (`127.0.0.1:16379`)
- Quest HMD/Touch controller 입력 처리기
- 공식 TWIST2 ONNX controller와 MuJoCo G1 simulation
- Quest에 제출되는 정면 3인칭 stereo 화면

실제 G1이나 Unitree SDK에는 연결하지 않는다.

### 착용 후 순서

1. 양손 컨트롤러를 움직여 깨운다.
2. 양쪽 grip을 모두 놓은 채, 편안한 중립 자세로 잠시 멈춘다.
3. 터미널에 `[READY]`가 나온 것을 확인한다.
4. 양쪽 grip을 계속 누른 채 손을 천천히 앞·뒤 또는 위·아래로 움직인다.
5. 한쪽 grip이라도 놓으면 마지막 팔 목표에서 멈춘다.
6. 종료할 때는 터미널에서 `Ctrl+C`를 누른다.

첫 테스트는 안정성을 위해 어깨와 팔꿈치만 움직인다. 로봇의 root, 다리, 허리,
손목 관절은 중립값으로 고정된다. HMD는 시작 시 room yaw와 VR 시점을 정하는 데만
쓰이며, 이후 머리를 돌리거나 숙여도 팔 목표가 따라 움직이지 않는다. 컨트롤러의
회전도 첫 테스트의 팔 위치 목표에는 사용하지 않는다.

robot-frame wrist 목표에는 X/Y/Z 고정 범위 제한을 적용하지 않는다. Quest arm
프로필은 pose smoothing, 속도-jump 차단, translation gain, Cartesian/joint slew,
관절 한계 안쪽 margin, neutral regularization, IK residual projection도 적용하지 않는다.
컨트롤러 translation은 1:1로 IK에 들어간다. 다만 MuJoCo G1 XML에 정의된 실제 관절
가동범위는 유지되므로, 물리적으로 도달할 수 없는 손목 목표에는 IK 오차가 남는다.
tracking jump도 더 이상 완화하지 않으므로 이 direct-input 프로필은 simulation에서만
사용하고, grip을 누른 뒤에는 손을 천천히 움직인다.

### 조작

- 양쪽 grip 계속 누르기: 팔 위치 입력 활성화
- grip 하나라도 놓기: 현재 팔 목표 유지
- 오른쪽 joystick 클릭: 정면 3인칭 VR 시점 재정렬
- 양쪽 joystick 동시 클릭: 팔 입력 비상정지
- grip을 모두 놓고 `Y + B`: 비상정지 해제 후 다시 중립 보정
- mirror 창의 `R`: VR 시점 재정렬 예비 입력
- mirror 창의 `Esc`, 또는 터미널 `Ctrl+C`: 종료

화면 끊김이 있으면 해상도를 낮춰 실행한다.

```bash
bash scripts/run_quest_arms_vr.sh --vr-render-scale 0.6
```

입력 로그는 `runtime/quest_arms_publisher.log`, Redis 로그는
`runtime/redis-quest.log`에 남는다. simulation consumer는 300 ms 동안 새롭고 정상인
35D action이 오지 않으면 중립 action으로 자동 전환한다.

### IK와 tracking 로그 읽기

- `[TRACKING_LOST]`: 뒤에 표시된 `Head`, `Left_Wrist`, `Right_Wrist` 중 하나 이상의
  추적이 끊겼다. controller를 다시 보이게 한 뒤 grip을 놓고 중립 보정부터 다시 한다.
- 주기적으로 표시되는 `IK=L=...mm R=...mm`는 각 손목의 요청 위치와 실제 IK 결과
  사이의 남은 거리다. 목표를 소프트웨어가 축소하지 않으므로 `scale`이나
  `[IK_PROJECTED]` 상태는 더 이상 없다.

팔이 더 움직이지 않으면서 IK 오차가 커지면 G1 어깨·팔꿈치의 실제 가동범위가
원인이다. `[TRACKING_LOST]`가 함께 나오면 Quest가 controller pose를 놓쳐 안전
hold로 전환한 것이다.

## 코드/안정성 검증

```bash
cd /home/railab/Desktop/vr_teleop/TWIST2
bash scripts/run_vr_tests.sh
bash scripts/run_quest_arm_headless_validation.sh
```

두 번째 명령은 실제 VR 없이 controller arm reference를 공급해 13초 동안 공식 ONNX와
MuJoCo 안정성을 검사한다.

## 보기만 먼저 확인

컨트롤러로 로봇을 움직이지 않고 VR 렌더링과 재정렬만 확인하려면 다음을 쓴다.

```bash
bash scripts/run_mock_vr.sh
```

## OpenVR 장치 확인

```bash
bash scripts/run_live_inventory.sh
```

Quest 3 HMD와 좌우 Touch controller의 serial/model이 보이고 `pose_valid`가 `true`인지
확인한다. 컨트롤러가 잠들어 있으면 들고 움직이거나 버튼을 눌러 깨운다.

## 다음 단계: Vive Ultimate Tracker를 포함한 전신 입력

Ultimate Tracker를 Linux 동글에서 직접 읽는 새 기본 경로는
[VIVE_LINUX_QUICKSTART.md](VIVE_LINUX_QUICKSTART.md)에 설치부터 serial 매핑, Quest/VIVE
공간 정렬, 중립 보정, dry-run까지 순서대로 정리돼 있다. Windows와 Windows SteamVR는
필요 없다.

아래의 기존 `vive-fullbody` 명령은 tracker가 이미 Linux SteamVR의 GenericTracker로
노출되는 경우에만 쓰는 구경로다. VIVE Hub의 `/tmp/vut.sock` 직접 연결과 혼용하지
않는다. 두 경로 모두 실제 tracker serial과 장착 위치를 먼저 확정해야 하므로
simulation publish는 아직 차단되어 있다.

```bash
mkdir -p config/local
cp config/tracker_mapping.vive_fullbody.example.json config/local/tracker_mapping.vive_fullbody.json
cp config/coordinates.vive_fullbody.example.json config/local/coordinates.vive_fullbody.json
```

inventory에서 확인한 serial을 mapping 파일의 `REPLACE_WITH_...` 항목에 넣고, 각
tracker의 mount transform을 coordinates 파일에 측정해 넣는다. 그 다음 출력 없는
dry-run으로만 확인한다.

```bash
bash scripts/run_vive_fullbody_dry.sh \
  --mapping config/local/tracker_mapping.vive_fullbody.json \
  --coordinates config/local/coordinates.vive_fullbody.json
```

자세한 체크리스트는 [VR_HARDWARE_TEST_PLAN.md](VR_HARDWARE_TEST_PLAN.md)에 있다.

## 기타 mock/replay 검증

```bash
bash scripts/setup_vr_env.sh
bash scripts/validate_assets.sh
bash scripts/run_mock.sh --frames 100 --record runtime/mock.jsonl
bash scripts/run_replay.sh runtime/mock.jsonl --realtime
bash scripts/run_example_motion_validation.sh
bash scripts/run_headless_validation.sh
bash scripts/run_mock_sim.sh
```
