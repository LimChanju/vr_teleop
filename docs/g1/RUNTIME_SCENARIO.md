# 합성 입력으로 실행 상태와 실제 추종 검증

이 검사는 실제 Quest 연결 검사가 아니다. `runtime_scenario.py`가 실제
`RawFrame` → `TargetMapper` → `encode_target` → UDP 경로로 입력을 보내고,
시뮬레이터가 기록한 입력 순번·시각·활성 상태·관절 상태와 대조한다.
송신만 완료되었다고 시뮬레이터 검증 성공으로 보고하지 않는다.

먼저 사용할 모델 폴더를 지정하고 첫 터미널에서 유한 시간 실행을 시작한다.
`MODEL`에는 평가하려는 실제 모델을 지정한다. 105차원 또는 126차원 입력 구성은
모델 메타데이터에서 읽는다. 출력 경로는 기존 폴더와 겹치지 않게 정한다.

```bash
export MODEL=/absolute/path/to/evaluated_model_folder
bash scripts/g1/run.sh teleop --headless --real-time --num-envs 1 \
  --steps 2500 --checkpoint "$MODEL/model_final.pt" --use-exported-policy \
  --record-trace --output runs/runtime_scenario_check
```

번들 모델 파일 이름은 `model.pt`이므로 해당 경로를 사용한다.
`[G1] Waiting for ALVR targets`가 출력된 뒤 두 번째 터미널에서 실행한다.
다른 입력 송신기는 함께 실행하지 않는다.

```bash
export MODEL=/absolute/path/to/evaluated_model_folder
.venv-alvr/bin/python scripts/g1/runtime_scenario.py \
  --nominal "$MODEL/nominal_targets.json" \
  --host 127.0.0.1 --port 8765 \
  --output runs/runtime_scenario_check/scenario.jsonl
```

기본 주기는 60 Hz, 전체 송신 시나리오는 약 28.5초다. 환경의 2500step은
50초의 시뮬레이션 시간이며 실제 GPU 처리 속도에 따라 더 오래 걸릴 수 있다.
시나리오 시작이 늦어 마지막 단계가 기록되지 않았다면 분석은 누락으로 실패한다.

| 단계 | 기대 결과 |
|---|---|
| idle → calibrate | 추적 데이터는 도착하지만 정책 원격 입력은 비활성 |
| armed_motion | 시작 버튼과 양손 그립, 왼손 높이·오른손 전방 목표를 ±6 cm 변화 |
| deadman_release | 그립을 놓고 원격 추종 중지; 송신기의 시작 허용은 유지 |
| rearm_motion | 시나리오는 시작 버튼을 다시 누르고 조작 복구; 일반 그립 복귀의 필수 조건은 아님 |
| producer_pause | 1.5초 동안 UDP 송신과 mapper 갱신을 모두 중단; 수신 0.25초 제한 확인 |
| held_arm_after_pause | 시작 버튼을 계속 누른 상태에서도 조작이 재개되지 않아야 함 |
| arm_release → rearm_after_pause | 버튼을 놓았다 새로 눌러 조작 복구 |
| reset → post_reset_rearm | 리셋 1회, 명시적 재시작 후 조작 복구 |
| final_stop | 원격 추종 중지; 종료 시 정지 패킷도 3회 전송 |

이 시나리오의 `producer_pause`는 UDP뿐 아니라 송신기 매퍼 갱신도 중단한다.
그래서 매퍼가 시작 허용을 해제하며 복귀한 뒤 새 A/arm 입력을 기다린다.
이를 **수신기에서만 발생한 패킷 손실**이나 낙상 리셋에도 새 A가 꼭 필요하다는
검사로 해석하지 않는다. 수신기 latch는 신선한 disabled→enabled 전이로 풀리며,
송신기 허용이 남아 있으면 그립 해제→재쥐기도 이 전이를 만든다. 추적이 정상인
일반 그립 동작은 클러치이므로 재쥐면 이어진다. 전체 조작 규칙은
[ALVR 입력 안내](ALVR.md)를 따른다. 이 기록은 합성 입력이며 실제 버튼 검증은 아니다.

이동·회전 속도 명령은 0이다. 6 cm는 **변환 후 로봇 목표 위치의 진폭**이다.
기본 변환 배율 0.65를 적용하기 전 합성 컨트롤러 위치는 이에 맞춰 더 크게 움직인다.
`--phase-seconds`, `--hz`, `--scale`로 바꿀 수 있고 통신 중단은 최소 0.6초다.

JSONL에는 단계 시작·종료의 Unix 시각, 각 RawFrame, 실제 전송 TargetFrame,
매퍼 상태와 송신 결과가 들어 있다. 기존 파일은 덮어쓰지 않는다. 마지막
`producer_completed=true`는 송신기 자체 조건만 통과했다는 뜻이며,
`receiver_verified=false`와 `simulator_verified=false`를 그대로 기록한다.

시뮬레이터가 정상 종료해 `result.json`과 `trace.npz`를 저장한 뒤 분석한다.

```bash
.venv-alvr/bin/python scripts/g1/runtime_scenario.py \
  --output runs/runtime_scenario_check/scenario.jsonl \
  --analyze-run runs/runtime_scenario_check
```

기본 보고서는 `runs/runtime_scenario_check/scenario_analysis.json`이다.
기존 보고서를 덮어쓰지 않으므로 재분석 시 `--analysis-output 다른경로.json`을 지정한다.
분석은 새 데이터를 송신하지 않는다. 단계 시각과 `trace.wall_time`을 비교하므로
송신기와 시뮬레이터는 같은 PC에서 실행하는 것을 기준으로 한다.

`control_state_passed`는 다음 실제 기록이 모두 맞을 때만 참이다.
기존 API의 `simulator_checks_passed`는 같은 값으로 유지한다.

- 모든 단계에 충분한 추적 표본이 있고, 원격 활성/비활성 및 입력 신선도가 기대와 일치한다.
- UDP와 매퍼 갱신을 함께 중단한 뒤 멈추며, 복귀 시 버튼 유지 상태에서 자동 재시작하지 않는다.
- 활성 입력 순번과 목표점이 실제 송신 데이터에 대응한다.
- 시뮬레이터의 수동 리셋이 정확히 1회이며 리셋 단계 안에서 발생한다.
- 관절·액션·목표점이 유한하고 낙상이 기록되지 않았다.

기본 단계 경계 여유는 0.15초(`--grace-seconds`)이며 통신 중단의 0.25초
수신 제한은 별도로 더한다. 단계별 표본이 3개 미만이거나 실행 구간이 누락되면
실패로 기록한다. 느린 처리 속도 때문에 누락되어도 성공으로 추정하지 않는다.
오차 분석에서는 자동/수동 리셋 행과 앞뒤 1행을 제외한다. 신선한 활성 입력과
유한한 로봇 상태가 있는 행만 추종 분석에 사용한다.

`tracking_quality_passed`는 상태 전환과 별개다. 목표 범위가 5 mm 이상인 각 축에
대해 실제 위치를 목표 위치에 선형 회귀한 **gain**, 상관계수, 실제/목표 가동 범위
비율, 절대오차·RMSE·편향을 기록한다. 회귀에는 절편을 포함하므로 고정 위치 오차가
gain에 섞이지 않으며, 위치 편향은 별도 수치로 남는다. 지연을 보정하지 않은 같은
시각의 입력·출력을 비교한다.

임시 기준은 gain ≥ 0.5와 상관계수 ≥ 0.6이다. `--min-gain`,
`--min-correlation`으로 바꿀 수 있으며 사용한 기준은 결과에 기록한다.
상관이 높아도 실제로 거의 움직이지 않으면 gain 기준을 통과하지 못한다.
기준 통과는 자극한 축의 최소 응답을 확인한 것이며 작업 성공이나 실제 장비 검증을
의미하지 않는다. `control_and_tracking_passed`는 두 판정이 모두 참일 때만 참이다.

기본 분석 종료 코드는 기존과 같이 상태 검사 결과를 따른다. 품질 실패도 종료 코드에
반영하려면 `--require-tracking-quality`를 추가한다. 기본 상태 시나리오는 왼손 Z와
오른손 X만 자극하므로 다른 7개 축의 추종 능력을 증명하지 않는다.

`simulator_checks_passed=true`여도 실제 Quest 연결·영상·지연 검증은 아니다.
부하 조건과 선택한 정책에서 이 짧은 합성 입력 시나리오를 통과했다는 의미다.

## 머리와 양손 9개 축의 독립 추종 검사

`--tracking-sweep`은 기본 상태 시나리오와 다른 약 59초짜리 입력을 보낸다.
먼저 보정하고 2초간 중립 자세를 유지한 뒤, 머리 XYZ·왼손 XYZ·오른손 XYZ를
각각 5초씩 움직인다. 마지막에 10초간 작은 복합 동작을 보내고 정지한다.
이 모드에서는 리셋이나 의도적인 통신 중단을 하지 않으므로 기본 상태 검사를
대체하지 않는다.

| 목표 | 변환 후 로봇 좌표에서의 최대 변위 |
|---|---|
| 머리 X / Y | 각각 ±0.025 m |
| 머리 Z | 중립에서 아래로 최대 0.04 m |
| 양손 X / Y / Z | 각각 ±0.06 m |
| 복합 동작 | 위 변위의 절반 |

각 구간의 시작과 끝 0.5초에는 부드러운 포락선을 적용해 목표가 중립으로 돌아오고
속도도 0으로 이어진다. 활성 구간 전체에서 양손 그립과 arm을 유지한다.
이동·회전 명령은 0이다. `--phase-seconds`는 기본 상태 시나리오에만 적용되며
sweep 구간 길이는 고정이다.

주파수는 머리 XYZ 0.31/0.37/0.23 Hz, 왼손 0.43/0.29/0.47 Hz,
오른손 0.41/0.33/0.53 Hz다. 불규칙한 목표점 사이를 보간하는 IK 학습 데이터와
다른 결정적 파형이며, 주파수와 변위는 송신 기록에도 저장한다.

```bash
# 첫 터미널: 기다린 뒤 입력을 시작할 시간을 포함해 90초의 시뮬레이션 시간 확보
bash scripts/g1/run.sh teleop --headless --real-time --num-envs 1 \
  --steps 4500 --checkpoint "$MODEL/model_final.pt" --use-exported-policy \
  --record-trace --output runs/runtime_tracking_sweep

# 두 번째 터미널: Waiting for ALVR targets 출력 후 실행
.venv-alvr/bin/python scripts/g1/runtime_scenario.py \
  --tracking-sweep --nominal "$MODEL/nominal_targets.json" \
  --output runs/runtime_tracking_sweep/scenario.jsonl

# 시뮬레이터가 결과를 저장한 뒤 실행; 임시 품질 기준 실패 시 종료 코드 1
.venv-alvr/bin/python scripts/g1/runtime_scenario.py \
  --output runs/runtime_tracking_sweep/scenario.jsonl \
  --analyze-run runs/runtime_tracking_sweep --require-tracking-quality
```

분석기는 송신 파일에 저장된 구간 계획과 각 시작 이벤트의 기대 상태를 대조한다.
sweep 품질 판정에는 각 축을 **독립적으로 자극한 구간**의 결과를 사용한다.
복합 동작에서 나온 움직임이 약한 단일 축 응답을 가리지 못하도록 한 것이다.
9개 구간 중 하나라도 누락되거나 자극이 부족하면 품질 판정이 실패한다.

기존 rich400 상태 검사 기록에 새 분석을 적용한
`scenario_tracking_analysis_v2.json`에서는 상태 검사가 통과했지만, 왼손 Z의
gain은 약 0.059, 가동 범위 비율은 약 0.072로 추종 기준을 통과하지 못했다.
오른손 X는 gain 약 0.936이었다. 기존 `scenario_analysis.json`은 보존했다.
이 결과는 두 축 상태 시나리오의 기록이며, 9축 sweep이나 실제 Quest 실행 결과가 아니다.

## 머리·양손 목표와 이동·회전을 함께 입력하는 검사

`--whole-body-sweep`은 약 75초의 별도 시나리오다. `--tracking-sweep`과 동시에
지정할 수 없으며 기존 기본 상태 검사와 9축 독립 검사의 동작은 그대로 유지된다.

보정 후 6초 동안 제자리에서 머리·손 목표를 움직인다. 이어서 전진/후진 각각
±0.15 m/s, 좌/우 이동 각각 ±0.08 m/s, 좌/우 회전 각각 ±0.2 rad/s를
각각 8초씩 입력한다. 각 방향 사이에는 2초 중립 구간을 두고, 마지막에는 8초
중립 유지와 2초 정지를 수행한다. 이 모드는 리셋이나 의도적 송신 중단을 포함하지 않는다.

머리·손은 이동·회전 구간에도 함께 움직인다. 머리 XY는 ±0.018 m, Z는
-0.025~0 m, 양손 각 축은 ±0.04 m 이내다. 따라서 작은 조작 범위 설정 안에
있지만, 이 범위에서 정책 성능이 확인되었다는 의미는 아니다.
각 구간 시작·끝 0.75초에 부드러운 전환을 적용한다. 실제 RawFrame의 왼쪽·오른쪽
스틱 축을 만들어 `TargetMapper`의 부호·deadzone·배율 처리 후 UDP로 전송한다.
최종 속도 패킷을 직접 만들어 매핑을 우회하지 않는다.

```bash
# 첫 터미널: 평가할 통합 정책 폴더. 번들에서는 model_final.pt 대신 model.pt 사용
export MODEL=/absolute/path/to/unified_policy_folder
bash scripts/g1/run.sh teleop --headless --real-time --num-envs 1 \
  --steps 5500 --checkpoint "$MODEL/model_final.pt" --use-exported-policy \
  --record-trace --output runs/runtime_whole_body_sweep

# 두 번째 터미널: Waiting for ALVR targets 출력 후 시작
.venv-alvr/bin/python scripts/g1/runtime_scenario.py \
  --whole-body-sweep --nominal "$MODEL/nominal_targets.json" \
  --output runs/runtime_whole_body_sweep/scenario.jsonl

# 시뮬레이터가 result.json/trace.npz를 저장한 뒤 분석
.venv-alvr/bin/python scripts/g1/runtime_scenario.py \
  --output runs/runtime_whole_body_sweep/scenario.jsonl \
  --analyze-run runs/runtime_whole_body_sweep \
  --analysis-output runs/runtime_whole_body_sweep/scenario_analysis_whole_body.json \
  --require-tracking-quality
```

5500 step은 50 Hz에서 110초의 시뮬레이션 시간이다. 송신 시작 여유를 포함한
길이이며 실제 처리 속도에 따라 벽시계 시간은 달라진다. 시작이 늦어 전체 구간이
기록되지 않으면 분석을 통과하지 못한다. 이동·회전 명령이 정책의 학습 범위를
넘어 수신기에서 잘린 경우도 적용 명령과 요청의 불일치로 기록한다.

보고서에서 다음을 구분한다.

- `control_state_passed`: 구간별 활성 상태, 신선도, 실제 송신 목표·속도와의 일치,
  유한 상태, 리셋 횟수 및 낙상 없음.
- `tracking_quality_passed`: 활성 구간의 머리·양손 위치 응답. 이 모드에서는 여러
  위치 축을 함께 움직이므로 9축 독립 검사를 대체하지 않는다.
- `velocity_tracking`: 각 방향의 처음 2초와 마지막 1초를 제외한 구간에서
  목표·실제 평균 속도, gain, MAE, RMSE를 계산한다. XY는 `root_linear_velocity_b`,
  yaw는 `root_angular_velocity_b`의 Z 성분을 직접 사용한다. 양방향 자료를 합친
  축별 LS gain·상관도 기록하고 같은 임시 gain/상관 기준을 적용한다.
- `contact_diagnostics`: 양발 접촉 비율, 단일/양발 지지 비율, 접촉 중 발 수평 속도를
  기술한다. 이러한 수치만으로 발을 번갈아 내딛는 보행, 안정성 또는 미끄럼 없음을
  판정하지 않으며 `gait_success_certified`는 항상 false다.
- `whole_body_checks_passed`: 상태·머리/손 추종·속도 추종의 임시 기준이 모두
  충족될 때만 참이다. 실제 Quest 연결이나 보행 성공을 의미하지 않는다.

속도·접촉 trace가 없는 이전 실행도 읽을 수 있다. 새 속도 배열이 없으면
`velocity_tracking.available=false`로 표시하며, 측정하지 않은 속도 성능을
통과로 추정하지 않는다. 이 모드에서 `--require-tracking-quality`를 지정하면
속도 자료 누락이나 속도 응답 실패도 종료 코드 1을 반환한다.

```bash
# 실제 localhost UDPReceiver + TeleopGate를 이용하는 짧은 시험; GPU/SteamVR 없음
PYTHONDONTWRITEBYTECODE=1 .venv-alvr/bin/python -m unittest discover \
  -s tests/g1 -p 'test_runtime_scenario.py' -v
```
