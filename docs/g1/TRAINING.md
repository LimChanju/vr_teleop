# G1 전신 정책 학습과 평가

학습·평가·텔레옵은 모두 `g1_teleop/sim/env.py`의 같은 Isaac Lab 환경을 사용한다. 따라서 Isaac Gym 정책을 다른 Isaac Sim 모델에 그대로 옮기는 단계는 없다. 로봇과 PD 값은 [ROBOT.md](ROBOT.md), 동작 리타게팅은 [MOTION.md](MOTION.md)에 기록되어 있다.

## H2O 구조의 적용 범위

[human2humanoid](https://github.com/LeCAR-Lab/human2humanoid)의 전체 동작 참조, 동작 추종 보상, privileged teacher, 머리·양손과 고유감각을 쓰는 sparse student 구조를 G1에 적용했다. 원본 H1 환경을 실행하는 것이 아니라 Isaac Lab으로 구현한 G1 환경이다. 원 논문의 학습 결과를 재현했다고 주장하지 않는다.

세 가지 학습 구성이 있다.

| `--stage` | 기본 v1 actor / critic | `--rich-observations` v2 actor / critic | 학습 방법 | Quest 실행 |
|---|---:|---:|---|---|
| `sparse` | 105 / 138 | 126 / 156 | 비대칭 actor-critic PPO | 가능 |
| `teacher` | 138 / 138 | 156 / 156 | 전체 참조를 보는 PPO teacher | 불가 |
| `student` | 105 / 138 | 126 / 156 | 학습된 teacher의 액션을 student 방문 상태에서 증류 | 가능 |

`sparse`는 처음부터 실행 중 얻을 수 있는 입력만 actor에 준다. 학습 중 critic만 참조 관절 자세, 골반 속도·높이를 본다. 이 경로는 별도 teacher 증류 없이 배포할 수 있다. `student`는 RSL-RL의 `Distillation`으로 teacher 액션에 대한 Huber loss를 최적화한다. 최종 모델이 어느 경로로 학습됐는지는 `run_config.json`과 평가 보고서에 기록한다.

105차원 actor 관측 순서는 각속도 3, 중력 방향 3, 중립 대비 관절각 29, 관절속도 29, 직전 액션 29, 머리·왼손·오른손 목표 위치 9, 이동·회전 명령 3이다. 관측 시간 이력은 없다. 각속도는 0.25, 관절속도는 0.05배로 고정 정규화한다. 경험적 running normalization은 사용하지 않는다. 실행 시에도 같은 코드를 사용한다.

v2는 여기에 시뮬레이터에서 측정한 골반 선속도 3, 목표와 현재 표식 위치의 차이 9, 목표 이동 속도 9를 추가한다. 목표 속도는 제어 스텝마다 차분하고 ±2 m/s로 제한한 뒤 EMA 0.5를 적용한다. 처음 입력·리셋·재시작 때 이력을 지우고, 정책에는 한 제어 스텝 늦게 들어간다. 모두 실제 텔레옵 실행 중 얻을 수 있는 정보다. v2 critic만 골반 높이 1과 전체 참조 관절 29를 추가로 사용한다. `policy.json`의 `observation_version`과 입력 순서를 실행 전에 검사하며, 모델을 불러오면 버전을 자동 선택한다.

기존 v1 가중치를 v2 입력으로 옮길 때는 `--warm-start 이전/model.pt --rich-observations`를 쓴다. 기존 입력의 가중치는 보존하고 추가 입력 가중치를 0으로 시작하며, actor와 critic 출력이 보존되는지 검사한다. 이때 optimizer는 새로 시작한다. 동일 구조에서 optimizer까지 그대로 재개하려면 `--checkpoint`를 쓴다.

액션은 `nominal_q + 0.5 * clamp(action, -4, 4)`인 관절 위치 목표다. 그 뒤 실제 G1의 soft joint limit를 적용하고 PD 액추에이터로 구동한다. 손가락은 0.024m로 열린 상태를 유지한다. 전체 몸 관절 29개가 정책 출력에 포함되며, 하체는 별도의 고정 애니메이션이 아니다.

## 보상과 종료

머리·양손 위치, 명령 속도, 골반 높이, 수직 자세와 전체 참조 관절을 추종한다. 발이 접촉한 상태의 미끄러짐, 수직 속도, roll/pitch 각속도, 토크, 관절속도와 액션 변화를 벌점으로 준다. 정확한 가중치는 환경 소스에 있으며, 매 실행의 `source/`에도 복사한다.

골반 높이가 0.40m 아래로 내려가거나 기울기가 60도를 넘거나 상태가 유한하지 않으면 낙상 종료로 센다. 일반 학습 에피소드는 20초이며 시간 제한 종료는 낙상과 따로 센다. 참조 상태로 초기화하고, 클립 경계를 넘는 목표 순간이동을 막기 위해 선형 보간과 끝점에서 방향을 반전하는 재생을 사용한다. 평가에서도 같은 재생 방식을 명시한다.

## 실행

설치 후 저장소 루트에서 실행한다. `G1_PYTHON`으로 별도 환경의 Python을 지정할 수 있다.

```bash
export G1_PYTHON="$HOME/anaconda3/envs/g1_teleop/bin/python"

# 합성 제자리 데이터 생성. 전달받은 data/motions가 있으면 생략할 수 있다.
"$G1_PYTHON" scripts/g1/prepare_motions.py

# 작은 실행 검증. 이것만으로 사전학습 완료라고 판단하지 않는다.
bash scripts/g1/run.sh train --headless --num-envs 64 --iterations 3 \
  --motion-file data/motions/g1_stand_v1.npz --output runs/smoke

# 초기 균형 학습
bash scripts/g1/run.sh train --headless --num-envs 2048 --iterations 500 \
  --motion-file data/motions/g1_stand_v1.npz --output runs/standing

# 목표 오차·속도 입력을 추가해 혼합 동작 학습을 시작한다.
bash scripts/g1/run.sh train --headless --num-envs 2048 --iterations 3000 \
  --warm-start runs/standing/model_final.pt --rich-observations --warm-start-noise 0.15 \
  --motion-file data/motions/g1_mixed_v1.npz --output runs/mixed

# 별도 seed와 학습에 포함하지 않은 클립으로 평가
bash scripts/g1/run.sh evaluate --headless --num-envs 64 --steps 5000 \
  --seed 2026 --motion-split eval --checkpoint runs/mixed/model_final.pt \
  --use-exported-policy --record-trace --output runs/evaluation

# 학습 재개: iterations는 추가 업데이트 수다.
bash scripts/g1/run.sh train --headless --num-envs 2048 --iterations 3000 \
  --checkpoint runs/mixed/model_latest.pt --output runs/mixed_resume
```

`--max-hours 2`처럼 시간 제한을 줄 수 있다. 100회 업데이트 단위가 끝날 때 시간을 확인하므로 제한보다 한 묶음 정도 더 실행될 수 있다. 체크포인트는 100회마다 남고, 정상 종료 시 `model_final.pt`, 추론용 `policy.pt`, 정책 계약 `policy.json`을 기록한다. Ctrl+C로 종료하면 `model_interrupted.pt`도 저장한다. 정기 체크포인트와 마지막 정상 저장본을 보존하므로 중단 후 이어서 학습할 수 있다.

같은 `--output`을 재사용하지 않고 새로운 폴더를 지정한다. `run_config.json`, 소스 사본, TensorBoard 이벤트, `progress.json`, `result.json`을 함께 보관한다. 모델 단독으로 전달하지 말고 관절·좌표·제어 주기 계약도 함께 전달한다.

`--learning-rate 0.0001`로 학습률을 명시할 수 있다. 재개 시에는 저장된 optimizer의 학습률과 PPO의 adaptive learning-rate 상태를 함께 복원한다. `progress.json`의 전체 진행 추정치를 확인한다. RSL-RL 내부 ETA는 100회 묶음 호출의 누적 시간 때문에 크게 표시될 수 있다.

`--precision-training`은 표식 추종 가중치/오차 분산을 `6/0.01`, 높이 가중치/오차 분산을 `2/0.0025`, 낙상 비용을 `5`로 바꾼다. 기본값은 각각 `4/0.04`, `1/0.01`, `2`다. 낙상 비용은 사건당 부과하며 `dt`를 곱하지 않는다. 이 설정은 자동 성능 향상을 보장하지 않으므로 원래 체크포인트를 보존하고 낙상과 축별 추종을 함께 평가한다. 저장된 `reward_contract`는 재개·평가 시 복원한다.

표식 추종만 별도로 조정하려면 학습 명령에 `--tracking-sigma 0.05`를 추가한다. `exp(-오차² / sigma²)`의 길이 척도를 m로 지정하며, 이 예시는 분산 0.0025다. 값이 작으면 작은 위치 오차에 민감해지지만 큰 오차에서 복구 신호가 약해질 수 있다. 다른 보상은 유지하고, 안정적인 체크포인트에서 짧게 추가 학습한 뒤 같은 입력으로 비교한다.

머리 반응만 보강하려면 `--head-tracking-weight 2`로 머리 항의 계수를 두 배로 한다. 양손 계수와 원래 분모 3은 유지한다. 기본값 1은 이전 보상과 동일하고, 값은 `(0,10]` 범위다. 이 값도 체크포인트 설정에서 복원하므로 평가 명령에는 지정하지 않는다. 머리·손의 평균 오차만 확인하지 말고, 독립적인 9개 축 입력의 반응과 낙상을 함께 확인한다.

완료한 학습의 전체 scalar 로그를 이전용 JSON과 그림으로 저장할 수 있다.

```bash
"$G1_PYTHON" scripts/g1/export_training_log.py runs/선택한_run \
  --plot runs/선택한_run/training_metrics.png
```

진행 중 로그는 `snapshot_only=true`로 표시하며 완료 로그로 취급하지 않는다.

## Teacher → Student 선택 경로

```bash
bash scripts/g1/run.sh train --stage teacher --headless --num-envs 2048 \
  --rich-observations --iterations 3000 --motion-file data/motions/g1_mixed_v1.npz --output runs/teacher

bash scripts/g1/run.sh train --stage student --headless --num-envs 2048 \
  --rich-observations --iterations 2000 --teacher-checkpoint runs/teacher/model_final.pt \
  --motion-file data/motions/g1_mixed_v1.npz --output runs/student

bash scripts/g1/run.sh evaluate --headless --num-envs 64 --steps 5000 \
  --seed 2026 --checkpoint runs/student/model_final.pt \
  --use-exported-policy --output runs/student_evaluation
```

수렴하지 않은 teacher를 증류해도 안정적인 student가 되는 것은 아니다. Teacher의 별도 평가를 먼저 확인한다. 이번에 실제로 실행한 학습 횟수와 성능은 결과 보고서를 기준으로 판단한다.

## 평가 수치 해석

`result.json`의 `mean_metrics`는 자동 리셋 **직전**의 모든 병렬 환경 상태를 집계한다. 목표 점의 평균 오차, 머리 오차, 양손 오차, 속도 오차를 m 또는 m/s로 기록한다. `falls`, `timeouts`, `fall_rate_completed_episodes`, `mean_completed_episode_seconds`를 함께 봐야 한다. 아직 끝나지 않은 에피소드는 `incomplete_episode_seconds`에 따로 남긴다.

입력 데이터의 IK 오차와 학습된 정책의 동역학 추종 오차는 다른 값이다. 짧은 실행 성공, 보상 상승, 학습용 클립의 결과만으로 실제 Quest 텔레옵 성능을 판단하지 않는다. `--record-trace`의 trace는 환경 0만 담으므로 전체 64대의 집계와 혼동하지 않는다.

현재 제공된 동작은 제자리 동작이다. 비영 속도 명령을 지원하는 코드가 있다는 사실만으로 보행을 학습했다고 판단할 수 없다. 이동·회전 명령의 검증 범위와 활성화 여부는 최종 모델 결과 보고서를 따른다.
