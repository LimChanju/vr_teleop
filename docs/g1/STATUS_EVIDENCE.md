# G1 구현·검증 근거와 남은 범위

**확인 시점: 2026-09-30. 기본 모델은 head1100의 제자리 조작이다.**
이동·회전 명령은 비활성화한다. 상세 수치·체크포인트·평가 조건은
[RESULTS.md](RESULTS.md)에 있다. 개발 PC의 `runs/`, `logs/`는 Git 기본 제외
경로이므로 서버 이전에는 선택 모델과 평가 기록을 별도로 전달해야 한다.
기본 전달은 v6 r2, 추가 실험 묶음은 v7과 v9다. v9는 평행 이동 응답을
회복했지만 낙상 증가와 회전·상체 추종 미달이 남았다. 이번 전달의 학습·평가는
종료했으며 안정적인 전신 이동 텔레옵 완료로 보고하지 않는다.
이전 19:36 스냅샷은 `backups/results_docs_20260930_210715/`에 보존했다.

## 확인한 구현과 실행

| 항목 | 확인한 근거 | 실제 기록 |
|---|---|---|
| 기존 환경과 분리한 신규 설치 | 같은 PC에서 새 Conda 환경 설치, user-site 차단, `pip check` 통과 | `logs/g1/fresh_environment_verification.json` |
| 신규 환경 GPU 학습·export | 64환경, PPO 3업데이트, 실제 가중치 변화, TorchScript 오차 4.47×10⁻⁸ | `runs/g1_server_check_smoke/` |
| 기본 모델 번들 이전 검증 | 같은 PC의 새 번들 경로·독립 Isaac Lab으로 64환경×3000step 재평가; 0/192 낙상, 기존 head1100 지표와 정확히 일치 | [결과](verification/results/head1100_portable_result.json), [경로 확인](verification/results/head1100_portable_path_receipt.json) |
| 갱신한 기본 r2 번들 | 같은 head1100 가중치, 갱신 문서·검증 도구; 새 r2 경로에서64×3000step·0/192 낙상 및7개 지표 정확히 일치 | [r2 결과](verification/results/head1100_portable_r2_result.json), [r2 경로](verification/results/head1100_portable_r2_path_receipt.json) |
| 이전 번들 렌더링 경로 | 1환경×200step·4초·낙상 0회, 완료 에피소드 0; 미리보기 이미지 생성 | [실행 기록](verification/results/head1100_portable_preview_result.json) |
| 현재 코드 단위 검사 | 고유 186개 통과: 주 환경 169개 + 다른 의존성 환경에서 나머지 17개 확인; 최초 v6는153개, 기본 r2는180개 시점 | [verification/tests.json](verification/tests.json) |
| G1 모델 계약 | 같은 USD·29관절·PD·좌표·스케일·주기로 학습/평가/추론; FK 대조 | 평가별 `run_config.json`, [ROBOT.md](ROBOT.md) |
| 실제 정책 학습 | stand500, rich2500, precision500/1500, tight800, velocity2000 완료 후 별도 평가 | [완료 결과](RESULTS.md) |
| Teacher/Student 경로 | v1/v2 teacher 3업데이트 → student 5업데이트와 export; 50step student 평가 | `runs/teacher_smoke_v*/`, `runs/student_smoke_v*/`, `runs/student_smoke_eval/` |
| 입력 상태 전이 → 실제 물리 | 실제 UDP, TorchScript, 29관절 PD, timeout·deadman·새 arm edge·수동 reset | `runs/runtime_scenario_rich400/` |
| 9축 Cartesian 외부 입력 | 90초·실측 50Hz, 활성 57초, 낙상 0회; 머리 Y/Z 응답 기준 미달 | `runs/tracking_sweep_tight800/` |
| 위치·방향 drift | 같은 90초에서 순 XY 4.80cm, yaw −27.87도; 발 들기 전환 0회 | `runs/analysis_sweep_tight800/analysis.json` |
| head1100의 최초 조건별 차이 | 오프라인 0/192 낙상, 초기화 수정 전 외부 입력 시험 7회 낙상·활성 2.22초 | `runs/eval_head1100/`, `runs/tracking_sweep_head1100/` |
| head1100 초기화 수정 후 | 같은 체크포인트 90초·50Hz, 활성 57.02초, 낙상 0회, 단독 9축 잠정 기준 통과 | `runs/tracking_sweep_head1100_grounded/` |
| head1100 전체 상태 전이 | 40초·50Hz, 낙상 0회, reset 1회, timeout·새 arm edge 포함 모든 검사 통과 | `runs/runtime_state_head1100_grounded/` |
| unified1500 이동 통합 실험 | 무작위 명령 6/192 낙상, 고정 명령열 4/75; 좌우·회전 응답 미달 | `runs/eval_unified1500/`, `runs/eval_unified_velocity1500/` |
| unified1500 외부 입력 실패 | 110초 실행 중 2.88초에 낙상, 활성 1.72초; 이동 블록 유효 표본 0 | `runs/whole_body_unified1500/` |
| velocity2000 외부 입력 미달 | 110초·낙상 0회이나 머리 3축·양손 Y/Z·회전 응답 부족 | `runs/whole_body_velocity2000/` |
| pure_axes2500 추가 실험 | 같은 오프라인 조건에서 회전 gain 개선, 1/72 낙상; 외부 입력 110초 낙상 0회이나 양의 yaw·7개 위치 축 미달, 영명령 yaw 편향 악화 | [비교 결과](RESULTS.md), [guard 분석](verification/results/pure_axes2500_guarded_comparison.json) |
| gradual3000 추가 실험 | 같은 Cartesian 평가에서 영명령 yaw와 위치 오차 개선, 좌우 gain 0.247/0.052로 하락; UDP 110초 낙상0회이나 좌우 gain0.060/0.005로 통합 기준 미달 | [평가](verification/results/gradual3000_cartesian_result.json), [guard 비교](verification/results/gradual3000_guarded_comparison.json) |
| balance3500 최종 이동 실험 | Cartesian 6/75·stand11/76 낙상; UDP110초0낙상·평행 이동4방향 기준 통과지만 회전 양방향·7개 위치 축 미달 | [평가](verification/results/balance3500_cartesian_result.json), [UDP 판정](verification/results/balance3500_whole_body_analysis.json), [guard 비교](verification/results/balance3500_guarded_comparison.json) |

신규 환경은 Python 3.10.21, Isaac Sim 4.5.0.0, PyTorch 2.5.1+cu121,
RSL-RL 2.3.3, NumPy 1.26.4, 별도 고정 Isaac Lab 체크아웃을 사용했다.
기존 Kit 라이선스 동의 기록을 재사용했으며 다른 서버의 설치 결과는 아니다.
신규 설치 당시 입력 테스트 14개 통과 기록과 이후 추가된 단위 검사를 같은
시점의 수치로 합치지 않는다. [설치 상세](INSTALL.md)

번들 재실행에서는 USD·모션·체크포인트·export를 모두
`/home/railabchan/g1_portable_validation`에서 읽었다. Isaac Lab도 원래 소스와
공유 inode·외부 symlink가 없는 별도 체크아웃을 사용했고 기존 환경의 설치
경로는 보존했다. 머리 1.030839cm·양손 1.324320cm, timeout 192, 완료 평균
19.97999954초로 원래 평가의 집계 값과 정확히 일치한다. 이는 같은 PC에서의
이전 가능성 확인이며 실제 공용 서버나 헤드셋 검증은 아니다.
[프레임워크 근거](verification/portable_framework.json),
[결과 비교](verification/portable_result_comparison.json),
[서버 실행 안내](RUN_SERVER.md)

현재 기본 전달 파일은 `artifacts/g1_stationary_v6_r2_20260930.tar.gz`이며
최초 v6 파일은 그대로 보존한다. r2도 같은 PC의 별도 환경·독립 프레임워크에서
확인했으며 실제 공용 서버나 Quest 하드웨어 검증은 아니다. 해시와 상세 비교는
[RESULTS.md](RESULTS.md)에 있다.

## 성능이 확인된 범위

최종 v7·v9 아카이브도 각각 새 경로·신규 Conda·독립 Lab으로 재평가했다.
18환경×4500step의 Cartesian+속도 평가에서 각 모델의 10개 집계 지표가
원래 결과와 정확히 일치했다. 실제 로드 경로와 번들 파일 해시도 확인했다.
[v7](verification/portable_velocity_v7_paths.json),
[v9](verification/portable_wholebody_v9_paths.json).

`human2humanoid`의 전신 참조 추종·privileged/sparse 학습 발상을 G1용
Isaac Lab에 새로 구현했다. 원본 H1 환경이나 논문 결과의 정확한 재현은 아니다.
실제 평가한 정책은 실행 가능한 sparse actor와 참조 정보를 보는 asymmetric
critic을 쓰는 PPO다. **Teacher/Student는 짧은 경로 검증만 완료**했으며,
수렴한 teacher 또는 증류된 최종 student를 제공했다고 표현하지 않는다.

v1 actor/critic은 105/138차원, 현재 v2는 126/156차원이다. v2 actor의 추가
입력은 시뮬레이터에서 얻는 상태·목표 오차·과거 입력 차분이며 전체 참조 관절이
아니다. 모델별 `observation_version`과 계약을 그대로 복원한다.
[학습 설정](TRAINING.md)

같은 Cartesian heldout에서 양손 평균 오차는 preprecision1800 4.95cm,
precision500 2.22cm, precision1500 2.48cm, tight800 1.47cm, head1100 1.32cm였다. 낙상은 각각
0/192, 0/192, 7/192, 1/192, 0/192다. 이 비교는 같은 데이터·seed·64환경·3000step에
한정한다. stand·mixed·비영 이동 명령 평가를 이 수치와 같은 조건으로 취급하지 않는다.

tight800은 29관절을 구동하는 제자리 전신 제어다. 머리 Y/Z축의 약한 응답과
방향 drift가 남아 있으며 9축 추종 종합 기준을 통과하지 않았다. 별도
velocity2000은 전후 속도를 따라갔지만 yaw 명령 gain이 0.050/−0.003으로
회전을 거의 수행하지 못했다. 사람의 실제 다리 움직임이나 완성된 보행·회전
텔레옵을 학습했다는 근거가 아니다. [이동 평가](LOCOMOTION.md)

head1100은 초기화 수정 후 단독 9축 기준을 통과했지만 복합 머리 Y gain은
0.462이고, 90초에서 yaw −26.22도·순 XY 4.67cm의 drift가 남았다. 전체 상태
전이 시험도 40초 동안 통과했으며, 제자리 후보의 이 결과를 이동·회전 성능으로
해석하지 않는다. 수정 전 실패도 [RESULTS.md](RESULTS.md)에 함께 보존했다.

상태 시험의 통신 중단은 UDP와 송신기 매퍼 갱신을 함께 멈춘 조건이다.
일반 그립 해제는 클러치라서 추적이 유지되면 재쥐기만으로 이어진다. 수신기만의
timeout·낙상 latch는 신선한 disabled→enabled 전이로 풀리며 그립 동작도
가능하다. 모든 재개에 새 A가 필요하다고 해석하지 않는다. [조작 규칙](ALVR.md)

평가 클립을 학습 업데이트에 넣지는 않았지만 개발 중 모델 선택과 보상 조정에
사용했다. 실제 사용자·새 동작에서의 독립적인 최종 검증은 추가로 필요하다.

## 남은 한계와 현장 미검증

- unified1500은 별도 평가와 실제 UDP 추가 시험에서 모두 이동·회전 통합 성능이 미달했다. head1100은 90초 추종·40초 상태 전이 시험을 근거로 제한된 제자리 기본 모델로 선택했다. 걷기·회전·머리·양손 조작을 동시에 안정적으로 수행하는 정책은 준비되지 않았다.
- 기존 37액션 보행 체크포인트는 몸체/손가락 구성과 링크 좌표·질량·effort limit이 현재 모델과 달라 직접 적용을 채택하지 않았다. 성공한 teacher 전이 근거는 없다. 순수 이동축 v7 실험도 통합 기준 미달로 기본 모델에 선정하지 않았다. 각 추가 실험은 해당 조건과 결과를 따로 기록한다.
- v8·v9의 추가 500업데이트씩과 각각의 평가도 완료했다. v9는 작은 UDP 평행 이동을 되찾았지만 다중 환경의 낙상이 늘었고 UDP 회전이 더 약해졌다. v7·v9는 실험용으로 구분하며 제자리 기본 모델의 대체 완성품이 아니다. 수동 v9 예시는 회전을 끄는 `--velocity-limits 0.15 0.08 0`이다.
- OpenVR 앱 등록·버튼·추적 실패 처리는 실제 API 자료형을 사용한 모의 런타임 검사다. 실제 Quest 연결 성공이 아니다.
- 합성 UDP 시험의 모든 기록은 `physical_quest_verified=false`다. 실제 Quest/ALVR/SteamVR와 Isaac OpenXR의 동시 작동, 영상 방향·재보정·지연은 장비에서 확인해야 한다.
- 공용 서버 설치·GPU 성능·네트워크·실시간 50Hz 여부는 미검증이다. 여러 GPU 작업이 겹친 병렬 평가는 실시간보다 느렸으며 그 처리량을 헤드셋 루프의 성능으로 사용하지 않는다.
- 현재 모션은 기립·팔 동작·펀치·합성 crouch 중심이다. 인간 다리 추적, 다양한 보행 모방, 손가락·bHaptics 제어, 광범위 자기충돌·지형 대응은 포함하지 않는다.

주요 원본 결과 사본은 [검증 기록 목록](verification/results/index.json)에 있으며
전체 trace·모델은 번들로 전달한다. 제자리 기본 모델의 서버 실행은
[RUN_SERVER.md](RUN_SERVER.md), 환경 구성은 [ALVR.md](ALVR.md),
[INSTALL.md](INSTALL.md)를 따른다.
