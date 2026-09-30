# 배포 모델을 공용 서버에서 실행하기

기본 모델 `teleop_head_precision_v6`는 머리·양손의 작은 위치 변화를 추종하며
29개 몸체 관절로 균형을 잡는다. **제자리 조작용이며 스틱 이동은 비활성화한다.**
실제 GPU 학습과 가상 UDP → TorchScript → Isaac Sim 관절 제어를 검증했다.
실제 Quest 연결과 XR 영상은 아직 확인하지 않았다. 90초 시험의 방향 drift
약 −26°와 복합 동작의 부족한 머리 Y 응답은 남아 있다. [전체 결과](RESULTS.md)

## 1. 저장소와 모델 받기

기존 저장소가 있다면 새 경로 이름을 사용한다. 아래는 새 서버에서의 예다.

```bash
git clone --branch g1-whole-body-alvr-20260930 --single-branch \
  https://github.com/LimChanju/vr_teleop.git "$HOME/vr_teleop_source"
cd "$HOME/vr_teleop_source"

python3 scripts/g1/bundle.py verify artifacts/g1_stationary_v6_r2_20260930.tar.gz
python3 scripts/g1/bundle.py unpack artifacts/g1_stationary_v6_r2_20260930.tar.gz \
  --destination "$HOME/g1_teleop_run"
cd "$HOME/g1_teleop_run"
```

`g1_teleop_run`은 아직 존재하지 않는 새 폴더여야 한다. 번들에는 정책·재개
체크포인트·해당 동작 데이터·G1 USD 전체 폴더·소스·설정·평가 기록이 있다.
Conda 환경 자체는 포함하지 않으므로 다음 설치가 필요하다. 원래 PC에서
직접 옮길 때도 TAR와 `.sha256`을 함께 복사한 후 같은 검증·추출 명령을 쓴다.

## 2. 별도 실행 환경 설치

Conda와 서버의 NVIDIA 드라이버가 준비된 Ubuntu 22.04에서 실행한다.
이름이 같은 Conda 환경이 이미 있으면 새로운 `--target-prefix`를 지정한다.
설치 스크립트는 기존 환경·ALVR 설정·드라이버를 덮어쓰지 않는다.

```bash
bash scripts/g1/setup.sh --fresh \
  --target-prefix "$HOME/anaconda3/envs/g1_teleop_server"

export G1_PYTHON="$HOME/anaconda3/envs/g1_teleop_server/bin/python"
export G1_MODEL="$PWD/models/teleop_head_precision_v6"
export G1_USD_PATH="$PWD/assets/robots/g1-29dof_wholebody_dex1/g1_29dof_with_dex1_rev_1_0.usd"
```

Miniconda 등 다른 Conda 설치 위치는 `--conda /실제/경로/bin/conda`를 추가한다.
최초 Isaac 실행의 제조사 라이선스 확인은 서버 사용자가 터미널에서 처리한다.
검증 버전은 Isaac Sim 4.5.0.0, 고정 Isaac Lab commit,
Torch 2.5.1+cu121, RSL-RL 2.3.3이다. [설치 상세](INSTALL.md)

## 3. 먼저 같은 조건으로 평가

```bash
bash scripts/g1/run.sh evaluate --headless --device cuda:0 \
  --num-envs 64 --steps 3000 --seed 2026 \
  --checkpoint "$G1_MODEL/model.pt" --use-exported-policy \
  --motion-file "$PWD/data/motions/g1_teleop_v3.npz" --motion-split eval \
  --record-trace --output runs/server_eval_head1100
```

이미 존재하는 결과 폴더에는 쓰지 않으므로 재시험에는 새 `--output` 이름을
사용한다. 별도 평가의 비교 기준은 낙상 0/192, 머리 평균 오차 1.03cm,
양손 1.32cm, 완료 에피소드 평균 19.98초다. GPU와 시뮬레이터 환경에 따라
결과가 달라질 수 있으므로 `result.json`을 확인한다.

## 4. ALVR·SteamVR·Quest 연결 후 실행

서버의 **같은 그래픽 사용자 세션**에서 기존 ALVR·SteamVR를 켜고 Quest와
양쪽 컨트롤러가 추적되는 상태로 만든다. 다음 두 터미널은 모두
`$HOME/g1_teleop_run`에서 실행한다. 첫 터미널은 로봇과 영상을 담당한다.

```bash
cd "$HOME/g1_teleop_run"
export G1_PYTHON="$HOME/anaconda3/envs/g1_teleop_server/bin/python"
export G1_MODEL="$PWD/models/teleop_head_precision_v6"
export G1_USD_PATH="$PWD/assets/robots/g1-29dof_wholebody_dex1/g1_29dof_with_dex1_rev_1_0.usd"
export XR_RUNTIME_JSON="$HOME/.local/share/Steam/steamapps/common/SteamVR/steamxr_linux64.json"
# Steam 라이브러리가 다른 경로라면 XR_RUNTIME_JSON을 실제 파일로 바꾼다.
test -f "$XR_RUNTIME_JSON"

bash scripts/g1/run.sh teleop --device cuda:0 --num-envs 1 --steps 0 --xr \
  --checkpoint "$G1_MODEL/model.pt" --use-exported-policy
```

두 번째 터미널은 입력을 담당한다. 기본 바인딩은 `oculus_touch` 프로필이다.

```bash
cd "$HOME/g1_teleop_run"
.venv-alvr/bin/python scripts/g1/alvr_input.py --backend openvr \
  --nominal models/teleop_head_precision_v6/nominal_targets.json \
  --target-bounds config/g1/teleop_small_motion.json --velocity-limits 0 0 0
```

정면을 보고 편한 중립 자세에서 **X 보정 → A 시작 → 양손 그립 70% 이상 유지**
순서로 조작한다. 그립은 클러치이므로 놓으면 멈추고, 추적이 유지되면 다시
쥘 때 이어진다. B는 시작 허용 해제, Y는 시뮬레이션 리셋이며 이후 새 A가
필요하다. 추적·컨트롤러 입력 상실이나 송신기 갱신 중단도 시작 허용을 해제한다.
수신기에서만 발생한 timeout·낙상은 신선한 비활성→활성 전이로 복구한다.
송신기가 계속 시작 허용 상태라면 그립 해제→재쥐기로도 복구된다.

머리 X/Y ±2.5cm·아래 4cm, 양손 각 축 ±5cm의 로봇 목표 범위로 시작한다.
기본 사람→로봇 변화량 배율은 0.65다. 천천히 작은 동작부터 확인한다.
손목 방향·손가락·사람의 실제 다리 자세를 복사하는 모델은 아니다.
입력 UDP는 `127.0.0.1:8765`, 송신·수신 제한은 0.25초다. Quest의 IP를
이 UDP 주소에 넣지 않는다. Quest↔서버 연결은 기존 ALVR 설정이 담당한다.

## 5. 헤드셋 없이 같은 정책을 확인

새 r2 번들은 아래 한 명령으로 시뮬레이터와 합성 송신기를 실행하고 종료 후
결과를 검사한다. 기존 결과 폴더를 덮어쓰지 않는다.

```bash
python3 scripts/g1/validate_runtime.py \
  --checkpoint "$G1_MODEL/model.pt" --scenario tracking \
  --output runs/server_tracking
```

`--scenario state`는 정지·클러치·끊김·리셋을 검사한다. `whole_body`는 이동
명령도 보내므로 별도 이동 실험 정책에 사용한다. 추종 기준 미달 시 종료 코드
1을 반환하고 `scenario_analysis.json`과 물리 실행 기록을 보존한다.
아래는 같은 시험을 두 터미널에서 직접 실행하는 방법이다.

첫 터미널의 `--xr`를 `--headless`로 바꾸고 `--steps 4500 --record-trace
--output runs/server_sweep`를 지정한다. 준비 메시지가 나온 뒤 두 번째
터미널에서 실행한다.

```bash
.venv-alvr/bin/python scripts/g1/runtime_scenario.py --tracking-sweep \
  --nominal models/teleop_head_precision_v6/nominal_targets.json \
  --output runs/server_sweep/scenario.jsonl

# 시뮬레이터가 종료한 다음
.venv-alvr/bin/python scripts/g1/runtime_scenario.py \
  --output runs/server_sweep/scenario.jsonl --analyze-run runs/server_sweep \
  --require-tracking-quality
```

이것은 합성 입력 시험이다. 실제 헤드셋 연결·버튼·영상 성공으로 보고하지 않는다.
전체 상태 전이와 이동 명령 시나리오는 [시나리오 문서](RUNTIME_SCENARIO.md)에 있다.

## 6. 학습 이어서 하기

기본 모델을 추가 학습하는 예다. `--iterations`는 기존 1100에 더할 업데이트
수이며 optimizer와 보상 설정을 복원한다. 더 오래 학습한 모델이 항상 좋지는
않으므로 기존 모델을 보존하고 같은 평가·외부 입력 시험으로 다시 비교한다.

```bash
bash scripts/g1/run.sh train --headless --device cuda:0 --num-envs 2048 \
  --checkpoint "$G1_MODEL/model.pt" \
  --motion-file "$PWD/data/motions/g1_teleop_v3.npz" \
  --iterations 1000 --output runs/head1100_resume
```

이동·회전 통합 정책의 별도 체크포인트와 이어서 학습하는 방법은
[서버에서 이동 실험 실행](RUN_MOVING_EXPERIMENT.md), 비교 결과는
[이동 학습](LOCOMOTION.md), 새 데이터·Teacher/Student 설정은
[학습 안내](TRAINING.md)에 있다. 기본 제자리 모델의 스틱 제한을 해제하는
것만으로 보행 정책이 되지는 않는다.

## 현장에서 남은 확인

실제 Quest에서 머리·양손 좌표와 버튼 바인딩, 중립 보정, 그립 해제/재접속,
G1 시점의 높이·방향·양안 영상, 실측 제어 주기와 체감 지연을 확인해야 한다.
OpenVR 입력 앱과 Isaac OpenXR 렌더링의 동시 실행도 현재 PC에서는 하드웨어로
검증하지 못했다. 오류별 확인 사항은 [ALVR 문제 해결](ALVR.md)을 따른다.
