# 이동·상체 통합 실험을 서버에서 재개하기

기본 제자리 모델의 설치·실행은 [RUN_SERVER.md](RUN_SERVER.md)를 따른다.
이 문서는 이동 명령까지 학습한 **실험 정책**을 재평가하고 이어서 학습하는
방법이다. 안정적인 보행과 머리·양손 동시 추종이 완성됐다는 뜻이 아니다.
모델별 이동 응답, 방향 편향, 낙상과 상체 오차는 [결과](RESULTS.md) 및
[이동 평가](LOCOMOTION.md)를 먼저 확인한다.

| 아카이브 | 압축 해제 후 모델 | 목적 |
|---|---|---|
| `g1_velocity_v7_20260930.tar.gz` | `models/velocity_pure_axes_v7` | 순수 이동축 명령 학습의 비교 기준, 총 2500업데이트 |
| `g1_wholebody_v9_20260930.tar.gz` | `models/locomotion_balance_v9` | 상체 동작 혼합·정지 yaw·작은 선형속도 명령 보상을 추가한 실험, 총 3500업데이트 |

두 모델은 다른 장단점이 있으므로 마지막 파일이라는 이유로 좋은 모델로
간주하지 않는다. 기본 제자리 모델은 계속 `teleop_head_precision_v6`다.
원본 37액션 G1 보행 모델을 이 29액션 모델에 직접 연결하지 않는다.

## 실험 묶음 열기

다음은 기본 안내에서 `g1_teleop_server` 환경을 설치한 다음 실행한다.
압축 해제 폴더와 결과 폴더는 새로운 이름이어야 한다.

```bash
cd "$HOME/vr_teleop_source"
python3 scripts/g1/bundle.py verify artifacts/g1_wholebody_v9_20260930.tar.gz
python3 scripts/g1/bundle.py unpack artifacts/g1_wholebody_v9_20260930.tar.gz \
  --destination "$HOME/g1_wholebody_experiment"
cd "$HOME/g1_wholebody_experiment"

export G1_PYTHON="$HOME/anaconda3/envs/g1_teleop_server/bin/python"
export G1_MODEL="$PWD/models/locomotion_balance_v9"
export G1_USD_PATH="$PWD/assets/robots/g1-29dof_wholebody_dex1/g1_29dof_with_dex1_rev_1_0.usd"
```

번들에는 optimizer를 포함한 체크포인트, TorchScript, 학습 설정과 소스,
학습 데이터, 두 비교 평가 데이터, 학습 곡선과 실제 평가 기록이 들어 있다.
기본 모델의 파일이나 환경을 덮어쓰지 않는다.

## 같은 조건으로 평가하기

학습에 쓰지 않은 Cartesian 동작과 고정 이동 명령열을 함께 준다. 18환경,
4500step, seed 2027이며 환경당 90초다. 에피소드는 약 20초에서 리셋된다.

```bash
bash scripts/g1/run.sh evaluate --headless --device cuda:0 \
  --num-envs 18 --steps 4500 --seed 2027 --velocity-evaluation \
  --checkpoint "$G1_MODEL/model.pt" --use-exported-policy \
  --motion-file "$PWD/data/motions/g1_teleop_v3.npz" --motion-split eval \
  --record-trace --output runs/server_cartesian_velocity_eval

# 같은 정책에 실제 UDP 합성 컨트롤러 값을 보내 110초 동안 구동
python3 scripts/g1/validate_runtime.py \
  --checkpoint "$G1_MODEL/model.pt" --scenario whole_body \
  --output runs/server_whole_body_udp
```

평가의 `result.json`에는 전체 환경 지표가, UDP 시험의
`scenario_analysis.json`에는 각 이동 방향과 머리·손 응답이 들어 있다.
정지·입력 처리가 정상이어도 추종 기준에 미달하면 자동 검증 명령은 1로
종료한다. 생성된 기록을 삭제하지 않는다. 이는 실제 Quest 연결 시험이 아니다.

v7을 비교하려면 해당 아카이브를 **다른 새 폴더**에 풀고 `G1_MODEL`을
`$PWD/models/velocity_pure_axes_v7`로 바꾼 뒤 같은 평가 명령을 사용한다.

## 이어서 학습하기

아래의 1000은 추가 PPO 업데이트 수다. optimizer, 관측, PD, 보상과 명령
분포를 모델 설정에서 복원한다. 기존 모델을 보존하고 새 결과를 같은 기준으로
다시 평가한다.

```bash
bash scripts/g1/run.sh train --headless --device cuda:0 --num-envs 2048 \
  --checkpoint "$G1_MODEL/model.pt" --iterations 1000 \
  --motion-file "$PWD/data/motions/g1_gradual_v1.npz" \
  --output runs/wholebody_resume
```

v7을 그대로 재개하려면 위 모션 파일 대신 `g1_stand_v1.npz`를 사용한다.
새 혼합 데이터는 정지 데이터의 각 클립 4회와 Cartesian 각 클립 1회를
보존해 합친 것이다. 학습 클립 기준 비율은 40:19이며 80:20이 아니다.
반복된 평가 클립은 서로 독립된 새 동작으로 세지 않는다.

## 실제 Quest 입력으로 시험하기

같은 그래픽 세션에서 ALVR·SteamVR를 실행하고, 시뮬레이터의 `--xr` 설정은
기본 안내와 동일하게 지정한다. 처음에는 `--velocity-limits 0 0 0`으로
기립과 버튼을 확인한다. 합성 시험 결과를 검토한 뒤 이동 명령 경로를
시험할 때는 다음 상한을 사용할 수 있다.

```bash
# 터미널 1: 위의 G1_PYTHON/G1_MODEL/G1_USD_PATH와 XR_RUNTIME_JSON 설정 후
bash scripts/g1/run.sh teleop --device cuda:0 --num-envs 1 --steps 0 --xr \
  --checkpoint "$G1_MODEL/model.pt" --use-exported-policy

# 터미널 2: 기본 묶음에서 설치한 입력 venv를 재사용
cd "$HOME/g1_wholebody_experiment"
"$HOME/g1_teleop_run/.venv-alvr/bin/python" scripts/g1/alvr_input.py --backend openvr \
  --nominal models/locomotion_balance_v9/nominal_targets.json \
  --target-bounds config/g1/teleop_small_motion.json --velocity-limits 0.15 0.08 0
```

숫자는 명령 상한이며 실제 추종 속도를 보장하지 않는다. v9의 110초 합성
시험에서 전후·좌우 속도 응답은 통과했지만 회전은 미달했으므로 위 명령은
회전 스틱을 끈다. 별도 Cartesian 평가에서는 75개 완료 에피소드 중 6회
낙상이 있었고 상체 추종도 부족했다. 왼쪽 스틱이 전후·좌우, 오른쪽 스틱이
회전 명령 경로다. 사람의 실제 다리 자세는 측정하지 않으며 하체 관절
명령은 정책이 생성한다. 실제 헤드셋·양안 영상·지연은 현장에서 확인해야 한다.
