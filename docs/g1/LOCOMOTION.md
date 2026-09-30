# G1 이동·회전 명령 학습

`g1_teleop/sim/velocity_env.py`는 같은 G1 모델에 이동 속도 명령을 추가하는
선택 학습 환경이다. 기본 전신 동작 추종 환경과 **29개 몸체 액션, 물리
설정과 제어 주기**가 같다. 관측은 기본 설정에서 actor 105 / critic 138차원,
`rich_observations=True`에서는 actor 126 / critic 156차원으로 기본 환경과 같다.
따라서 같은 모델 계약으로 학습한 전신 정책을 시작점으로 사용할 수 있다.
기존 환경이나 원본 USD를 수정하지 않는다.

## 목표 생성

| 명령 | 무작위 학습 범위 | 단위 |
|---|---|---|
| 전후 이동 vx | -0.30 ~ +0.30 | m/s |
| 좌우 이동 vy | -0.15 ~ +0.15 | m/s |
| 회전 yaw rate | -0.40 ~ +0.40 | rad/s |

각 환경의 명령은 5초마다 새로 정하고 30% 확률로 세 명령을 모두 0으로
설정한다. 환경 리셋 시에도 새 명령을 선택한다. 리타게팅 데이터의 표식·관절
자세 목표를 함께 사용하되, 데이터에 기록된 속도 명령 대신 위 샘플을 적용한다.
이동 명령 좌표는 로봇 body 기준이다.

`external_mode=True`에서는 무작위 명령 갱신을 완전히 중지한다.
텔레옵 명령은 `set_external_targets()`로만 받고 학습 범위 안으로 제한한다.
머리·양손 위치는 상체와 높이 목표에 사용하고, 기본 컨트롤러의 스틱은
이동·회전 명령을 제공한다. 사람의 다리 동작을 측정하는 기능은 아니다.

## 발 동작과 보상

다리는 PPO 정책이 29개 몸체 관절의 일부로 제어한다. 가짜 보행 궤적이나
root 위치를 강제로 이동하는 스크립트를 사용하지 않는다. 기본 환경의
속도 추종, 균형 유지, 자세 추종, 토크·액션 변화·발 미끄러짐 보상에 다음을
추가한다.

이동 명령 구간(`norm(vx,vy)>0.08 m/s` 또는 `abs(yaw)>0.10 rad/s`)에서는
기존의 완만한 속도 보상을 다음 식으로 **대체**한다. 기존 항에 중복해서
더하지 않으며, 에피소드 로그의 `linear_velocity`, `yaw_velocity`도 대체된
값을 합산한다. 정지 및 이 기준 이하의 작은 명령에서는 기존 보상을 유지한다.

| 항 | 이동 명령 중 보상 | 기존 정지 보상 |
|---|---|---|
| XY 속도 | `4 * exp(-||actual_xy-command_xy||² / 0.04)` | `2 * exp(-error² / 0.25)` |
| yaw rate | `2 * exp(-(actual_yaw-command_yaw)² / 0.10)` | `exp(-error² / 0.25)` |

표의 값은 초당 가중치이며 실제 매 스텝에는 제어 주기 0.02초를 곱한다.
예를 들어 전진 명령 0.20 m/s에서 가만히 서 있으면 XY 속도 보상은 약
1.472/초이고 정확하게 이동하면 4/초다. 이전 식에서는 각각 약 1.704/초와
2/초여서 느린 이동 명령을 무시해도 손실이 작았다. 이 변경은 물리, 관측,
액션 해석을 바꾸지 않는다. 발 미끄러짐 페널티와 아래 접촉 보상은 유지한다.

- 이동 명령이 있을 때 발이 0.20초 이상 공중에 있었다가 닿으면 작은 보상을 준다.
- 한 번의 착지 보상은 공중 시간 0.50초에서 상한에 도달한다.
- 두 발이 모두 떨어지는 상태에는 작은 페널티를 준다.
- 미끄러지며 바닥에 붙어 있는 발에는 공중 시간 보상이 생기지 않는다.

관절 기준 자세는 여전히 리타게팅 데이터에서 오므로, 속도 추종과 상체
동작 추종을 동시에 학습해야 한다. 이 코드가 있다는 사실만으로 안정적인
보행이 학습됐다고 판단하면 안 된다.

## 평가와 체크포인트

메타데이터의 `environment_variant`는 `g1_commanded_velocity_v1`,
새 학습의 기본 `training_velocity_limits`는 `[0.30, 0.15, 0.40]`이다. 이 필드가 없는 기존
정적 동작 추종 정책은 별도 근거 없이 보행 정책으로 취급하지 않는다.
속도 보상 설정은 `velocity_reward_contract.version=moving_velocity_replacement_v2`
및 해당 필드의 가중치·오차 분모에 기록된다. 이번 수정 전 파일은
`backups/20260930_velocity_reward_v2/`에 보관했다.

기존 표식 추종 오차, 낙상률과 에피소드 지속 시간에 더해 환경은 리셋 전의
다음 평가 값을 제공한다.

- 명령 vx/vy/yaw rate와 실제 vx/vy/yaw rate.
- 축별 절대 이동 속도 오차와 XY 속도 오차 norm.
- yaw rate 절대 오차.
- 해당 프레임에 이동 명령이 있었는지 여부.

양·음 방향 명령이 섞이면 평균 vx 자체는 거의 0이 될 수 있으므로
명령과 실제 속도의 단순 평균만으로 성공을 판정하지 않는다. 이동 구간의
추종 오차와 낙상 통계를 함께 확인해야 한다. 이동 성능이 충분하지 않으면
텔레옵에서는 속도 명령을 0으로 제한하고 기립 상태의 상체 조작부터 사용한다.

설정 클래스는 `G1VelocityEnvCfg`, 환경 클래스는 `G1VelocityEnv`다.
학습 실행기의 `--velocity-training` 옵션 또는 해당 체크포인트의 환경
메타데이터로 이 변형을 선택한다. `--checkpoint`로 재개·평가할 때는 저장한
이동·yaw 보상의 가중치와 오차 분산, 속도 제한, 정지 비율, 명령 재선택 주기,
발 공중 시간·양발 공중 상태의 가중치를 복원한다. `--warm-start`는 가중치
초기화이므로 새로운 학습 설정을 사용한다.

`velocity_reward_contract`가 없는 예전 velocity 모델은 당시의
XY `2*exp(-error²/0.25)`, yaw `exp(-error²/0.25)`로 복원한다.
지원하지 않는 버전이나 잘못된 분산·가중치는 오류로 처리한다.
따라서 새 코드의 기본 보상이 과거 모델 평가에 조용히 적용되지 않는다.

학습에서만 `--yaw-tracking-sigma 0.2 --yaw-reward-weight 4`로 yaw 보상을
덮어쓸 수 있다. sigma 단위는 rad/s이고 분모는 제곱한 0.04다. 저장된 설정을
복원한 뒤 명시한 옵션을 적용하며 평가에서는 옵션 대신 저장값을 따른다.
이 변경은 위 이동 조건에 해당하는 yaw 속도 추종 보상에 적용된다.
절대 heading 유지 기능이 추가되는 것은 아니며 영명령 구간의 보상은 기존과 같다.

## 고정 명령열로 평가

`G1VelocityEnvCfg.velocity_evaluation=True`는 학습용 무작위 속도 대신
`g1_teleop/velocity_evaluation.py`의 고정 명령열을 사용한다. 실행기의
`--velocity-evaluation` 옵션으로 선택할 수 있다.

| 블록 | vx / vy / yaw rate |
|---|---|
| 정지 | 0 / 0 / 0 |
| 전진·후진 | ±0.20 / 0 / 0 |
| 좌·우 이동 | 0 / ±0.10 / 0 |
| 좌·우 회전 | 0 / 0 / ±0.30 |
| 대각선 전진·후진 | +0.15 / +0.08 / 0, -0.15 / -0.08 / 0 |

9개 블록은 각각 5초이며 한 주기는 45초다. 환경마다 시작 블록을 순환해
분산하고, **전역 제어 스텝**으로 다음 블록을 선택한다. 낙상이나 에피소드
리셋으로 명령열이 처음부터 다시 시작하지 않는다. 실제 명령과 블록 번호를
같은 시점에 저장하므로 블록이 바뀌는 마지막 스텝의 상태도 올바르게 집계한다.

첫 1초는 속도 점수의 정착 구간으로 제외하며, 이후 4초에서 축별 MAE/RMSE,
XY·yaw 오차의 평균과 p95, 실제 속도 크기, 명령 대비 속도 이득과 부호 일치를
보고한다. 이득은 양·음 명령을 합쳐도 상쇄되지 않는 최소제곱 비율이다.
정지 블록에는 XY drift와 yaw drift에 해당하는 실제 속도 크기를 보고한다.
낙상은 정착 구간을 포함한 모든 스텝에서 블록별로 센다. 리셋 이후 회복 구간을
추가로 제거하지 않으며, 유한하지 않은 값은 개수를 별도로 기록한다.

`VelocityEvaluationAccumulator.update(raw.metrics)`는 매 스텝 자동 리셋
이전 스냅샷을 받고 `.summary()`가 JSON으로 저장 가능한 보고서를 반환한다.
전체 에피소드 완주율·표식 오차는 실행기의 기존 집계를 함께 사용한다.
외부 텔레옵 모드에서는 이 고정 명령열도 적용하지 않는다. 이 명령열 평가는
무작위 학습 명령보다 재현하기 쉽지만, 하드웨어 검증을 대신하지 않는다.

## 완료한 velocity2000 평가와 한계

`runs/eval_velocity_2000/result.json`은 2026-09-30에 완료한 별도 평가다.
18환경 × 4500step, seed 2027, stand 데이터의 평가 2클립, 환경당 90초다.
72개 완료 에피소드 중 낙상 1회, 시간 제한 종료 71회, 평균 완료 길이 19.92초다.
블록별 점수는 정착 1초를 제외한 7200샘플씩이며 낙상은 정착 구간도 포함한다.

| 명령 축 | 양의 명령 / 음의 명령 | 양방향 gain | 양방향 MAE |
|---|---|---|---|
| vx | +0.20 / −0.20 m/s | 0.966 / 0.945 | 0.0184 / 0.0182 m/s |
| vy | +0.10 / −0.10 m/s | 0.810 / 0.696 | 0.0339 / 0.0442 m/s |
| yaw rate | +0.30 / −0.30 rad/s | 0.050 / −0.003 | 0.2887 / 0.3053 rad/s |

전후 속도는 추종하지만 회전 명령은 거의 수행하지 못했다. 정지 블록의 평균
XY 속도는 0.0157 m/s, 절대 yaw 속도는 0.0773 rad/s다. 낙상 1회는 좌회전
블록에서 발생했다. 이 결과를 회전까지 완성된 전신 텔레옵으로 해석하지 않는다.
상체 추종과 함께 평가한 전체 지표·체크포인트는 [RESULTS.md](RESULTS.md)에 있다.

추가 외부 입력 시험 `whole_body_velocity2000`은 머리·양손 동작을 함께
명령하며 110초·50Hz, 활성 72초를 실행했고 낙상 0회·상태 검사 통과였다.
각 plateau에서 시작 2초·끝 1초를 제외한 실제 속도는 전진/후진
0.1606/−0.1170m/s(명령 ±0.15), 좌/우 0.0139/−0.0783m/s(명령 ±0.08)였다.
회전 ±0.20rad/s 명령의 합친 gain은 0.00193으로 미달했고, 머리 3축과
양손 Y/Z도 응답 기준에 미달해 전체 전신 판정은 실패했다.

오프라인 명령은 ±0.20/±0.10/±0.30으로 크기가 다르고 상체 목표도 stand다.
외부 좌우 ±0.08은 학습 보상의 `norm(command_xy)>0.08` 경계에 해당한다.
따라서 상체 움직임 때문에 좌측 응답이 나빠졌다고 단정하지 않는다. 추론 때
계산되는 보상은 동작을 직접 바꾸지 않으며, 조건을 맞춘 대조 시험이 필요하다.

`unified_2048_v5`는 이 정책에서 Cartesian v3 목표와 더 좁은 위치·yaw 보상으로
이어간 별도 실험으로 1500업데이트 후 두 평가를 완료했다.
`eval_unified1500`은 64환경·3000step·seed 2026이지만 무작위 이동
명령이 활성이므로 같은 Cartesian v3를 쓰는 정지 정책 평가와 조건이 다르다.
`eval_unified_velocity1500`은 18환경·4500step·seed 2027의 고정 명령열과
Cartesian v3를 함께 사용한다. velocity2000의 stand 목표와도 차이가 있으므로
이동·표식 추종을 동시에 수행할 때의 결과로 따로 보고해야 한다.

무작위 명령 평가는 낙상 6/192, timeout 186, 평균 완료 19.63초였다.
고정 명령열은 낙상 4/75, timeout 71, 평균 완료 19.26초였고 정착 후
전진/후진 gain 1.065/1.046, 좌/우 0.163/−0.052, 좌/우회전 0.253/0.109다.
대각선까지 포함한 축별 gain은 vx 약 1.09, vy 0.032, yaw 0.181이다.
전후 평균 속도 외에 좌우·회전 응답과 순간 속도 오차가 부족하므로 기본 모델로
선택하지 않았다. `whole_body_unified1500`의 실제 UDP 추가 시험도 2.88초에
낙상했고, 전체 110초 중 활성 입력은 1.72초였다. 이후 gate가 비활성으로
유지돼 이동 블록의 정착 후 유효 표본은 전부 0이다. 이 시험의 전체 평균 오차를
이동 성공으로 보고하지 않는다.
[완료 평가 수치](RESULTS.md)

## 재현·재개 명령

저장소 루트에서 실행한다. 아래 평가 명령은 완료된 velocity2000을 재평가한다.
기존 기록을 보존하려고 출력 경로를 새로 지정했다.

```bash
bash scripts/g1/run.sh evaluate --headless \
  --checkpoint runs/velocity_2048_v2/model_final.pt --use-exported-policy \
  --motion-file data/motions/g1_stand_v1.npz --motion-split eval \
  --velocity-evaluation --num-envs 18 --steps 4500 --seed 2027 \
  --record-trace --output runs/velocity2000_recheck
```

완료 모델에서 yaw 보상을 바꾸어 추가 학습하는 명령 예시다. 이 명령 자체를
실행·평가한 결과는 아니며, `--iterations`는 추가 업데이트 수다.

```bash
bash scripts/g1/run.sh train --headless \
  --checkpoint runs/velocity_2048_v2/model_final.pt \
  --num-envs 2048 --iterations 1000 \
  --yaw-tracking-sigma 0.2 --yaw-reward-weight 4 \
  --output runs/velocity_yaw_resume

bash scripts/g1/run.sh evaluate --headless \
  --checkpoint runs/velocity_yaw_resume/model_final.pt --use-exported-policy \
  --velocity-evaluation --num-envs 18 --steps 4500 --seed 2027 \
  --record-trace --output runs/velocity_yaw_resume_eval
```

완료된 unified1500 체크포인트는 다음 조건으로 고정 명령열을 재평가할 수 있다.
이 명령은 부족한 성능을 재현·비교하는 용도이며 준비된 배포 모델이라는 뜻이 아니다.

```bash
bash scripts/g1/run.sh evaluate --headless \
  --checkpoint runs/unified_2048_v5/model_final.pt --use-exported-policy \
  --motion-file data/motions/g1_teleop_v3.npz --motion-split eval \
  --velocity-evaluation --num-envs 18 --steps 4500 --seed 2027 \
  --record-trace --output runs/unified_velocity_recheck
```

## 실제 입력에서 이동 축 제한

`scripts/g1/alvr_input.py --velocity-limits VX VY YAW`는 OpenVR/live 또는
synthetic 입력의 스틱 범위를 제한한다. 단위는 m/s, m/s, rad/s이며 0은 해당
축을 비활성화한다. 허용 상한은 `[0.5,0.25,0.6]`이다. replay에서는 이미
기록된 명령을 보존하므로 이 옵션을 사용할 수 없다. 시뮬레이터는 다시 해당
모델의 `training_velocity_limits`로 제한한다. 정지 정책은 이 값이 전부 0이다.

먼저 이동 명령을 끄고 기립 조작을 확인하는 입력 명령이다. 실제 ALVR 입력은
공용 서버의 SteamVR 환경에서 추가 검증해야 한다.

```bash
.venv-alvr/bin/python scripts/g1/alvr_input.py --backend openvr \
  --nominal runs/teleop_precision_tight_v4/nominal_targets.json \
  --host 127.0.0.1 --port 8765 --velocity-limits 0 0 0
```

velocity 모델을 실험할 때 입력을 `--velocity-limits 0.20 0.10 0`으로 정하면
전후·좌우만 명령하고 yaw 스틱 명령은 보내지 않는다. 이 제한은 자연스러운
yaw drift까지 제거하지 않으므로 화면과 로그에서 방향 변화를 확인해야 한다.
최종 서버 실행에는 결과 보고에서 선택한 모델의 `nominal_targets.json`을 사용한다.

기존 공식 보행 체크포인트의 37액션은 23몸체+14손가락 모델용이다. 현재
29몸체 모델과 관절 구성뿐 아니라 링크 좌표·질량·effort limit도 다르므로
가중치를 그대로 연결하는 방안은 채택하지 않았다. 성공한 다리 teacher 전이로
보고할 결과는 없다. 별도 순수 이동축 RL 실험은 진행 중이며 이 문서의 완료
수치에 포함되는 새 통합 정책은 아직 없다. 제자리 기본 모델과 서버 실행
명령은 [RUN_SERVER.md](RUN_SERVER.md)를 따른다.
