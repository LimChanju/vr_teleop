# 합성 입력으로 실행 상태 전환 검증

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
| deadman_release | 그립을 놓고 원격 추종 중지 |
| rearm_motion | 시작 버튼을 다시 누르고 조작 복구 |
| producer_pause | 1.5초 동안 UDP 송신과 mapper 갱신을 모두 중단; 수신 0.25초 제한 확인 |
| held_arm_after_pause | 시작 버튼을 계속 누른 상태에서도 조작이 재개되지 않아야 함 |
| arm_release → rearm_after_pause | 버튼을 놓았다 새로 눌러 조작 복구 |
| reset → post_reset_rearm | 리셋 1회, 명시적 재시작 후 조작 복구 |
| final_stop | 원격 추종 중지; 종료 시 정지 패킷도 3회 전송 |

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

`simulator_checks_passed`는 다음 실제 기록이 모두 맞을 때만 참이다.

- 모든 단계에 충분한 추적 표본이 있고, 원격 활성/비활성 및 입력 신선도가 기대와 일치한다.
- 통신 중단 후 멈추며 버튼 유지 상태에서 자동 재시작하지 않는다.
- 활성 입력 순번과 목표점이 실제 송신 데이터에 대응한다.
- 시뮬레이터의 수동 리셋이 정확히 1회이며 리셋 단계 안에서 발생한다.
- 관절·액션·목표점이 유한하고 낙상이 기록되지 않았다.

기본 단계 경계 여유는 0.15초(`--grace-seconds`)이며 통신 중단의 0.25초
수신 제한은 별도로 더한다. 단계별 표본이 3개 미만이거나 실행 구간이 누락되면
실패로 기록한다. 느린 처리 속도 때문에 누락되어도 성공으로 추정하지 않는다.
손·머리 추종 오차와 움직임 범위는 별도 수치로 제시하며, 이를 임의의 작업 성공
기준으로 판정하지 않는다. 오차 분석에서는 자동/수동 리셋 행과 앞뒤 1행을 제외한다.

`simulator_checks_passed=true`여도 실제 Quest 연결·영상·지연 검증은 아니다.
부하 조건과 선택한 정책에서 이 짧은 합성 입력 시나리오를 통과했다는 의미다.

```bash
# 실제 localhost UDPReceiver + TeleopGate를 이용하는 짧은 시험; GPU/SteamVR 없음
PYTHONDONTWRITEBYTECODE=1 .venv-alvr/bin/python -m unittest discover \
  -s tests/g1 -p 'test_runtime_scenario.py' -v
```
