# G1 학습·통합 검증 결과

**기본 모델: `teleop_head_precision_v6`(head1100), 제자리 조작에 한정.**
전달 기본 파일은 `g1_stationary_v6_r2_20260930.tar.gz`다. v7과 v9는 서로
다른 장단점을 가진 **별도 실험용**으로 보관한다. 마지막 v9는 평행 이동 응답을
되찾았지만 오프라인 낙상이 늘고 실제 UDP의 회전·7개 위치 축 기준에 미달했다.
이번 전달을 위한 추가 학습과 평가는 종료했으며 안정적인 보행·회전·머리·양손
동시 텔레옵을 완성했다고 보고하지 않는다. [실험용 실행·재개](RUN_MOVING_EXPERIMENT.md)

2026-09-30의 완료된 로컬 평가 JSON을 기준으로 정리했다. 이동·회전 명령은
기본 모델에서 비활성화한다. `unified_2048_v5`는 평가를 완료했지만 이동·회전
통합 성능이 부족한 실험용 체크포인트이며 추가 외부 입력 시험도 실패했다.
걷기·회전과 머리·양손 추종을 동시에 안정적으로 수행하는 정책은 준비되지 않았다.
머리 가중치를 높인 head1100은 최초 외부 입력 시험에서 낙상 7회가 발생했고,
초기화 경로 수정 후 같은 체크포인트의 90초 시험에서 낙상 0회와 단독 9축
응답 기준 통과를 확인했다. 같은 초기화의 40초 전체 상태 시험도 통과했다.
기본 모델의 서버 실행 명령은 [RUN_SERVER.md](RUN_SERVER.md)를 따른다.
완료한 주요 `result.json`·시나리오·drift 분석 원본 사본은
[검증 기록 목록](verification/results/index.json)에 SHA-256과 함께 보관했다.
그 폴더에는 trace·모델 파일을 넣지 않았으며 전체 기록은 이전용 번들에 보관한다.

G1 29관절 제어 정책을 실제 GPU에서 학습했고, 합성 입력을 실제 UDP로 보내
TorchScript 추론과 Isaac Sim 관절 구동까지 실행했다. 양손 추종은 개선됐지만
복합 동작에서의 작은 머리 응답, 제자리 방향 drift, 회전 명령의 낮은 추종 성능이
남았다. **실제 Quest 3·ALVR·SteamVR·OpenXR 동시 연결과 공용 서버 실행은
아직 장비에서 검증하지 않았다.**

## 무엇을 학습했는가

`human2humanoid`의 참조 동작 추종과 privileged/sparse 정책 구성을 참고해
G1용 Isaac Lab 환경을 새로 구현했다. 원본 H1 Isaac Gym 구현이나 논문 결과의
정확한 재현은 아니다. 아래 성능표의 정책은 모두 **sparse actor + asymmetric
critic PPO**로 학습했다. 수렴한 teacher를 증류한 모델이라는 뜻이 아니다.

v2 actor는 실행 중 얻는 머리·양손 목표, G1 상태, 목표 오차·목표 속도와
이동 명령을 받는다(126차원). critic만 골반 높이와 전체 참조 관절을 추가로
본다(156차원). 동작 데이터의 참조 관절은 배포 actor 입력에 들어가지 않는다.
전체 몸체 29관절을 PD 위치 목표로 제어하며 손가락은 열린 자세를 유지한다.
학습·평가·텔레옵에 같은 USD, 좌표, PD, 액션 스케일, 0.02초 제어 주기를 쓴다.
[계약 상세](ROBOT.md), [학습 구조](TRAINING.md)

주요 학습은 실제로 2048환경, 환경당 rollout 24step으로 실행했다.
한 PPO 업데이트는 49,152개의 환경 전이를 수집한다. stand는 500업데이트,
rich는 새 분기에서 2500업데이트, precision은 새 분기에서 1500업데이트,
velocity는 새 분기에서 2000업데이트를 완료했다. tight800과 head1100은 각각
이전 체크포인트에서 300업데이트씩 추가했다. 모델 이름의 수는 에피소드나
시뮬레이션 step 수가 아니며, 같은 초기 가중치를 공유한 분기도 존재한다.

별도 Teacher/Student 경로는 v1/v2 각각 teacher 3업데이트와 student
5업데이트의 학습·export를 확인했다. student 평가 50step은 환경당 1초이며
완료 에피소드가 0이다. 이는 구현 경로 확인이고, 사전학습 완료나 수렴한
전신 모방 정책의 성능 근거가 아니다.

참조 데이터는 작은 합성 기립 동작, 넓은 합성 팔 동작, H1 `stable_punch`의
실제 G1 FK/IK 리타게팅, 독립 Cartesian IK 클립이다. 펀치의 하체는 제자리
자세이며 사람의 발·무릎 추적이나 다양한 보행 모션을 학습한 데이터가 아니다.
Quest 머리·양손 세 점만으로 사람의 다리 동작을 복원했다고 주장하지 않는다.
[원본 출처·제약](MOTION.md), [Cartesian 데이터](TELEOP_MOTION.md)

## 평가 방법과 분모

표의 오차는 자동 리셋 **이전**의 모든 평가 환경에서 집계한 위치 오차다.
양손 오차는 두 손목 오차의 평균이다. `낙상/완료`의 분모는 낙상 종료와
시간 제한 종료의 합이며 미완료 에피소드는 포함하지 않는다. 시간 제한은
약 20초이고, 미완료 시간은 각 `result.json`에 따로 있다.

학습용 보상이나 `mean episode length`를 별도 평가 수치로 바꾸어 쓰지 않았다.
`trace.npz`는 환경 0만 저장하므로 전체 환경 집계와 구별하며, trace 속도·응답
계산에서는 리셋 경계를 제외한다. 0회 낙상은 해당 노출 시간에서 관측하지
않았다는 뜻이다. [평가 집계 방법](VALIDATION_METHOD.md)

### 작은 기립 데이터와 넓은 혼합 데이터

| 평가 | 데이터 / seed / 환경×step | 머리 오차 | 양손 오차 | 낙상/완료 | timeout | 완료 평균 길이 |
|---|---|---:|---:|---:|---:|---:|
| 중립 PD `baseline_stand_v3` | stand / 42 / 64×1500 | 11.06 cm | 8.75 cm | 1181/1181 | 0 | 1.58 s |
| stand500 `eval_stand_500` | stand / 42 / 64×1500 | 1.11 cm | 1.37 cm | 0/64 | 64 | 19.98 s |
| rich2500 `eval_rich_2500` | mixed / 2026 / 64×3000 | 2.69 cm | 4.52 cm | 1/192 | 191 | 19.89 s |

1500step은 환경당 30초, 3000step은 환경당 60초다. stand는 평가 2클립,
mixed는 평가 11클립을 사용했다. **stand와 mixed의 숫자는 다른 과제의 결과**다.
rich2500의 mixed 오차를 아래 precision 정책의 Cartesian 오차와 직접 비교해
향상률을 계산하지 않는다. 원본은 `runs/<평가 이름>/result.json`과
`run_config.json`이며 두 학습 모델의 실행 경로는 TorchScript다.

### 같은 Cartesian heldout에서의 비교

다음 다섯 평가는 모두 `g1_teleop_v3.npz`의 동일한 평가 5클립, seed 2026,
64환경 × 3000step이다. 데이터 SHA-256도 모두
`00a55b19792d817725aa490c33e7b84b70f6eb6c2d1f676181755577dad0d675`로 같다.

| 평가 이름 | 머리 오차 | 양손 오차 | 낙상/완료 | timeout | 낙상률 | 완료 평균 길이 |
|---|---:|---:|---:|---:|---:|---:|
| `eval_teleop_preprecision1800` | 3.17 cm | 4.95 cm | 0/192 | 192 | 0% | 19.98 s |
| `eval_precision_500` | 1.85 cm | 2.22 cm | 0/192 | 192 | 0% | 19.98 s |
| `eval_precision_1500` | 1.71 cm | 2.48 cm | 7/192 | 185 | 3.65% | 19.39 s |
| `eval_precision_tight800` | 1.51 cm | 1.47 cm | 1/192 | 191 | 0.52% | 19.89 s |
| `eval_head1100` | 1.03 cm | 1.32 cm | 0/192 | 192 | 0% | 19.98 s |

preprecision1800은 rich 계열 `model_1799.pt`를 새 Cartesian 데이터로 평가한
기준점이다. RSL-RL 추론 경로를 사용했고 나머지 네 평가는 TorchScript다.
precision500에서 1000업데이트를 더 진행한 precision1500은 낙상이 늘었다.
tight800은 precision500으로 돌아가 더 좁은 위치 보상(`sigma=0.05 m`)으로
300업데이트를 추가한 분기다. 1500에서 이어 학습한 모델이 아니다.
head1100은 tight800에서 머리 보상 가중치를 2로 정해 300업데이트를 더한 모델이다.
아래 최초 런타임 실패와 초기화 수정 후 재시험을 함께 평가해야 하며,
오프라인 수치만으로 배포 모델에 선정하지 않았다.

같은 데이터에서 tight800의 양손 평균 오차는 기준점보다 약 70.3% 작다.
다만 기준점 0회에 비해 낙상 1회가 생겼고, 아래 외부 입력 시험의 머리 축은
전부 통과하지 않았다. 손 오차만으로 최종 전신 텔레옵 완료를 판정하지 않는다.

평가 클립은 PPO 업데이트에 넣지 않았지만, 중간 체크포인트 비교와 보상 조정에
사용했다. 따라서 이 표는 개발용 heldout 평가이며, 모델 선택과 독립적인
새 사용자·새 동작에서의 최종 일반화 검증을 대신하지 않는다.

## 실제 UDP → 정책 → 시뮬레이터 시험

`runtime_scenario_rich400`은 실제 제어 루프 40초·2000step에서 시작/정지,
deadman 해제, UDP·송신기 매퍼 갱신 중단, 입력 복귀 시 arm 버튼을 계속 눌러도 자동 재시작하지
않음, 버튼 해제 후 새 시작, 수동 reset 1회를 확인했다. 낙상 0회이고 활성 입력은
762step(15.24초)이다. 이 시험은 상태 전이와 입력 연결의 근거다.

`tracking_sweep_tight800`은 **학습 난수 경로와 다른 결정적 합성 경로**를
실제 UDP로 전송했다. 머리 X/Y ±2.5cm, Z 아래로 4cm, 양손 각 축 ±6cm를
축별 5초 동안 움직이고 복합 동작 10초를 추가했다. 전체 90초·4500step,
실측 49.9999 Hz, 활성 입력 2850step(57초), 낙상 0회, 수동 reset 0회다.
완료 에피소드가 없으므로 완료 에피소드 낙상률은 `null`이다.

아래는 각 축을 단독으로 움직인 구간의 235샘플씩을 사용한 결과다.
전체 활성 구간을 섞은 `axis_tracking`과 값이 다르며,
`scenario_analysis.json → tracking_quality.axis_results`가 표의 원본이다.
잠정 기준은 목표 범위 ≥5mm, 절편을 포함한 영지연 최소제곱 gain ≥0.5,
상관계수 ≥0.6이다. 절대 오차나 최대 gain에 대한 합격 기준은 아니므로
이 기준의 통과만으로 정밀 추종을 보장하지 않는다.

| 단독 입력 축 | 응답 gain | 상관계수 | 축별 MAE | 잠정 기준 |
|---|---:|---:|---:|---|
| 머리 X | 0.563 | 0.999 | 10.28 mm | 통과 |
| 머리 Y | 0.485 | 0.986 | 9.76 mm | 미달 |
| 머리 Z | 0.465 | 0.993 | 7.24 mm | 미달 |
| 왼손 X | 1.011 | 1.000 | 4.44 mm | 통과 |
| 왼손 Y | 1.006 | 0.999 | 1.18 mm | 통과 |
| 왼손 Z | 0.845 | 0.997 | 7.75 mm | 통과 |
| 오른손 X | 1.067 | 0.999 | 5.26 mm | 통과 |
| 오른손 Y | 0.952 | 0.999 | 5.21 mm | 통과 |
| 오른손 Z | 0.679 | 0.980 | 12.78 mm | 통과 |

상태 전이 검사는 통과했지만 **추종 종합 판정은 미달**이다. 복합 동작에서
머리 X/Y/Z gain은 0.350/0.415/0.413이고 머리 Y 상관계수는 0.333이다.
양손은 단독 입력보다 개선되었어도 머리가 사용자 움직임을 충분히 따라간다고
보기 어렵다. 임계값은 개발 중 진단 기준이며 외부 인증 기준이 아니다.

### 제자리 정책의 위치·방향 drift

같은 90초 시험에서 root의 순 XY 변위는 **4.80cm**, 누적 평면 이동 경로는
1.527m, 누적 yaw 순변화는 **−27.87도**였다. 양발 접촉률은 각각 99.96%였고
시작 직후 접촉을 제외한 발 들기 전환은 0회다. 이는 제자리 전신 관절 제어를
보여주며 보행 성공의 근거가 아니다. 작은 평균 손 오차나 낙상 0회와 함께
이 방향 drift를 반드시 고려해야 한다.
표식 오차는 골반 yaw를 기준으로 한 좌표계에서 측정하므로, 전역 방향이
천천히 돌아가도 손의 로컬 위치 오차는 작을 수 있다.

원본: `runs/tracking_sweep_tight800/{result.json,scenario_analysis.json,trace.npz}`,
`runs/analysis_sweep_tight800/analysis.json`. 모든 입력 source는 `synthetic`이고
`physical_quest_verified=false`다. 같은 sweep을 반복해 모델을 조정했으므로
이는 개발용 회귀 시험이며, 처음 보는 실제 사용자 동작의 평가가 아니다.

### head1100의 최초 실패와 초기화 수정

`tracking_sweep_head1100`은 90초·4500step에서 **낙상 7회, timeout 0회**였고
완료 에피소드 평균은 3.29초다. 활성 입력은 111step(2.22초)에 그쳤으며
첫 낙상 후 신선한 disabled→enabled 전이가 없어 이후 대부분 구간이 비활성 상태였다.
상태·추종 종합 검사는 모두 미달이다. 이 상태에서 나온 전체 평균 오차나
비활성 구간을 정상 9축 추종 점수로 해석하지 않는다.

조사 중인 가설은 모션 없이 실행하는 초기 root 높이와 학습 초기 높이의 차이,
그리고 공중 초기 상태에서 측정한 머리 위치를 정지 목표로 고정하는 경로다.
학습 범위보다 높은 머리 목표가 만들어질 가능성이 있어 초기화 경로를 수정했다.
모션 참조가 없는 reset도 FK 중립 골반 높이 +1.5cm의 여유 높이로 시작하며,
시작·수동 reset·낙상 reset의 목표는 바닥 기준 중립 marker와 속도 0으로 둔다.
일반적인 deadman/입력 해제에서는 현재 도달 자세를 유지한다. 낙상 후에는
다음 루프가 중립 목표를 공중의 측정 자세로 다시 덮어쓰지 않게 했다.

이 변경은 정책 가중치·액션/관측·물리 모델을 바꾸지 않으며 새 실행 메타데이터에
`reset_contract=grounded_nominal_v1`과 이전 계약에서의 migration을 기록한다.
실패한 `runs/tracking_sweep_head1100/`을 보존했고 새 기록은
`runs/tracking_sweep_head1100_grounded/`다.

### 같은 head1100의 초기화 수정 후 시험

`tracking_sweep_head1100_grounded`는 같은 체크포인트와 같은 종류의 9축 입력으로
90초·4500step을 완료했다. 실측 49.9999 Hz, 낙상 0회, 수동 reset 0회,
활성 2851step(57.02초)다. 상태 검사와 단독 9축 잠정 응답 기준이 모두 통과했다.
초기화 수정 뒤 이 재현 시험의 낙상이 사라진 근거이며, 어떤 입력에서도
안정적이라는 보증이나 각 변경의 개별 기여도를 분리한 실험은 아니다.

| 단독 입력 축 | 응답 gain | 상관계수 | 축별 MAE |
|---|---:|---:|---:|
| 머리 X | 0.855 | 0.994 | 3.17 mm |
| 머리 Y | 0.574 | 0.994 | 6.79 mm |
| 머리 Z | 0.752 | 0.967 | 4.10 mm |
| 왼손 X | 0.961 | 0.999 | 2.50 mm |
| 왼손 Y | 1.020 | 1.000 | 1.71 mm |
| 왼손 Z | 0.923 | 0.972 | 12.74 mm |
| 오른손 X | 1.048 | 0.999 | 3.95 mm |
| 오른손 Y | 0.943 | 0.998 | 3.03 mm |
| 오른손 Z | 0.636 | 0.990 | 13.03 mm |

각 행은 단독 축 구간 235샘플이며 기준은 위와 같다. 전체 90초 평균은 머리
5.97mm, 양손 7.63mm지만 약 33초의 비활성 구간도 포함한다. 복합 동작 10초의
평균은 머리 7.26mm, 양손 12.61mm다. 복합 머리 X/Y/Z gain은
0.859/0.462/0.602로 Y축은 여전히 0.5 미만이다. 단독 축 판정 통과를 복합 동작
전체의 통과나 손목 Z축의 완전한 크기 재현으로 확대하지 않는다.

같은 실행의 순 XY 변위는 **4.67cm**, 평면 경로 길이는 **1.901m**,
yaw 순변화는 **−26.22도**다. 초기화 수정 뒤에도 방향 drift는 남았다.
왼발 접촉률 100%, 오른발 99.98%이며 오른발만 1샘플의 접촉 해제·복귀가 있었다.
접촉 중 발 링크 원점의 평균 수평 속도는 왼발 0.0110m/s, 오른발 0.0090m/s다.
이는 링크 회전도 포함하므로 정확한 접촉점 미끄러짐으로 단정하지 않는다.
새 분석 기록은 `runs/tracking_sweep_head1100_grounded/analysis/analysis.json`과
같은 디렉터리의 PNG다. 모든 입력은 계속 합성이며 실제 Quest 검증은 아니다.

### head1100 최종 후보의 전체 상태 전이 시험

`runtime_state_head1100_grounded`는 40초·2000step, 실측 49.9997Hz로 완료했다.
낙상 0회, 수동 reset 1회, 활성 입력 762step(15.24초)다. deadman 해제,
0.25초 이상 UDP·송신기 매퍼 갱신 단절, 복귀 후 arm 버튼 유지 시 비활성 유지, 버튼 해제 후
새 arm edge, 수동 reset 후 재시작, 최종 정지의 검사와 실제 입력 sequence/목표
대조가 모두 통과했다. 완료 에피소드가 없어 완료 에피소드 낙상률은 `null`이다.
이 상태 시험에서 움직인 축의 응답 기준도 통과했으며, 전체 9축의 별도 근거는
위 90초 sweep이다. 원본은 같은 이름의 실행 폴더의 `result.json`과
`scenario_analysis.json`이다.

그립 해제는 송신기 시작 허용을 유지하는 클러치다. 추적 상실 없이 다시 쥐면
새 A 없이 재개한다. 수신기만의 timeout·낙상 latch도 신선한 disabled→enabled
전이를 요구하며, 송신기가 계속 armed라면 그립 동작으로 풀 수 있다. 위 시험의
새 arm 요구는 매퍼 갱신까지 중단한 조건이며 모든 통신 장애의 버튼 규칙을
대신하지 않는다. [정확한 조작 구분](ALVR.md)

## 별도 이동 정책 velocity2000

`eval_velocity_2000`은 stand 평가 2클립과 고정 속도 명령열을 사용했다.
18환경 × 4500step, seed 2027, 환경당 90초(9블록 2회), 전체 1620 환경·초다.
완료 72개 중 낙상 1회(1.39%), timeout 71회, 완료 평균 19.92초다.
각 5초 블록의 첫 1초를 제외해 블록당 7200샘플을 점수화했다.

| 명령 | 실제 해당 축 평균 | 해당 축 MAE | 명령 대비 gain |
|---|---:|---:|---:|
| 전진 +0.20 m/s | +0.193 m/s | 0.0184 m/s | 0.966 |
| 후진 −0.20 m/s | −0.189 m/s | 0.0182 m/s | 0.945 |
| 좌측 +0.10 m/s | +0.081 m/s | 0.0339 m/s | 0.810 |
| 우측 −0.10 m/s | −0.070 m/s | 0.0442 m/s | 0.696 |
| 좌회전 +0.30 rad/s | +0.0151 rad/s | 0.2887 rad/s | 0.050 |
| 우회전 −0.30 rad/s | +0.0010 rad/s | 0.3053 rad/s | −0.003 |

전후 속도는 추종했으나 회전 명령은 거의 수행하지 못했다. 정지 블록에서도
평균 XY 속도 0.0157 m/s와 절대 yaw 속도 0.0773 rad/s가 남았다. 유일한 낙상은
좌회전 블록에서 관측했다. 전후·좌우 이동 중 의도하지 않은 yaw 속도도 존재한다.
이 정책을 회전까지 완성된 전신 텔레옵 정책으로 보고하지 않는다.
재현·학습 재개·입력 축 제한 명령은 [LOCOMOTION.md](LOCOMOTION.md)를 따른다.

### velocity2000의 실제 UDP 이동·상체 동시 입력

`whole_body_velocity2000`은 1환경·seed 42, 110초·5500step, 실측 49.9999Hz로
완료했다. 활성 입력 3600step(72초), 낙상 0회, 수동 reset 0회이며 상태 검사는
통과했다. 완료 에피소드가 없어 완료 에피소드 낙상률은 `null`이다.
그러나 **위치 추종과 전신 통합 판정은 미달**이다.

각 명령 plateau의 시작 2초와 마지막 1초를 제외한 250샘플씩의 실제 속도다.

| 입력 plateau | 실제 해당 축 평균 | 개별 gain | 해당 축 MAE |
|---|---:|---:|---:|
| 전진 +0.15 m/s | +0.1606 m/s | 1.071 | 0.0240 m/s |
| 후진 −0.15 m/s | −0.1170 m/s | 0.780 | 0.0389 m/s |
| 좌측 +0.08 m/s | +0.0139 m/s | 0.174 | 0.0661 m/s |
| 우측 −0.08 m/s | −0.0783 m/s | 0.979 | 0.0342 m/s |
| 좌회전 +0.20 rad/s | +0.0186 rad/s | 0.093 | 0.1949 rad/s |
| 우회전 −0.20 rad/s | +0.0178 rad/s | −0.089 | 0.2178 rad/s |

양방향을 합친 yaw gain은 0.00193, 상관계수 0.00582로 회전 명령을 거의
구별하지 못했다. vy의 합친 gain 0.576이 잠정 축 기준을 넘더라도 개별
좌측 명령의 gain 0.174가 낮은 문제를 가리지 않는다.

동시에 움직인 머리·손 목표의 전체 활성 구간 gain은 다음과 같다.

| 목표점 | X gain | Y gain | Z gain |
|---|---:|---:|---:|
| 머리 | 0.143 | −0.007 | −0.035 |
| 왼손 | 0.818 | 0.257 | 0.016 |
| 오른손 | 0.738 | 0.103 | 0.115 |

머리 3축과 양손 Y/Z, 총 7축이 잠정 응답 기준에 미달했다. 접촉률은 왼발
93.94%, 오른발 92.92%이며 이는 접촉 진단일 뿐 올바른 보행이나 미끄럼 없는
이동을 증명하지 않는다. 원본은 `runs/whole_body_velocity2000/`의 두 JSON이다.

이 시험은 위 오프라인 평가와 속도 크기, 환경 수·seed, 상체 목표, 실행 경로가
다르다. 특히 좌우 ±0.08m/s는 학습 보상의 `norm(command_xy)>0.08` 경계에
있고 오프라인 좌우 명령은 ±0.10m/s다. 추론 중 보상 계산이 정책 동작을 직접
바꾸는 것은 아니다. 이 차이만으로 상체 움직임이나 보상 경계를 원인으로
단정하지 않으며, 원인을 분리하려면 같은 명령 크기·상태의 대조 시험이 필요하다.

## unified1500 — 이동·팔 동작 통합 실험의 미달

velocity2000에서 가중치를 옮겨 Cartesian v3와 위치 `sigma=0.05m`, 이동 중 yaw
가중치 4·`sigma=0.2rad/s`로 1500업데이트를 학습했다. 두 평가는 모두
TorchScript지만 아래와 같이 명령 조건이 다르다.

| 평가 | 조건 | 머리 / 양손 오차 | 낙상/완료 | timeout | 완료 평균 길이 |
|---|---|---:|---:|---:|---:|
| `eval_unified1500` | Cartesian v3, 64환경×3000step, seed 2026, 무작위 이동 명령 | 2.50 / 2.22 cm | 6/192 (3.13%) | 186 | 19.63 s |
| `eval_unified_velocity1500` | Cartesian v3, 18환경×4500step, seed 2027, 고정 명령열 | 2.57 / 2.33 cm | 4/75 (5.33%) | 71 | 19.26 s |

첫 unified 평가는 같은 Cartesian 파일을 사용해도 정지 정책 표와 명령 분포가
다르다. 두 번째 평가는 velocity2000과 속도 명령열이 같지만 상체 참조가
stand에서 Cartesian v3로 바뀐다. 두 결과 모두 단순한 동일 조건 향상률로
보고하지 않는다. 속도 지표는 정착 구간을 제외한 블록별 지표, 낙상·위치 추종은
모든 스텝의 집계로 나누어 기록한다.

고정 명령열의 정착 후 블록별 속도 결과는 다음과 같다. 블록당 7200샘플이다.

| 명령 | 실제 해당 축 평균 | 해당 축 MAE | 명령 대비 gain |
|---|---:|---:|---:|
| 전진 +0.20 m/s | +0.213 m/s | 0.0720 m/s | 1.065 |
| 후진 −0.20 m/s | −0.209 m/s | 0.0414 m/s | 1.046 |
| 좌측 +0.10 m/s | +0.0163 m/s | 0.0874 m/s | 0.163 |
| 우측 −0.10 m/s | +0.0052 m/s | 0.1054 m/s | −0.052 |
| 좌회전 +0.30 rad/s | +0.0758 rad/s | 0.2907 rad/s | 0.253 |
| 우회전 −0.30 rad/s | −0.0328 rad/s | 0.3039 rad/s | 0.109 |

대각선 블록까지 포함해 0이 아닌 명령축에서 합친 gain은 vx 약 1.09,
vy 0.032, yaw 0.181이다. 전후 평균 속도가 명령에 가까워도 순간 오차는 크고,
좌우 움직임과 회전은 충분히 따라가지 못했다. 정지 블록의 XY 속도도
0.0659m/s, 절대 yaw 속도 0.1879rad/s로 남았다. 낙상은 정지 1회, 전진 1회,
좌회전 2회다. **이 모델을 이동·회전 통합 텔레옵 완료 모델로 선택하지 않았다.**

### unified1500 실제 UDP 통합 시험의 실패

`whole_body_unified1500`은 110초·5500step·실측 49.9999Hz로 끝났지만
**2.88초에 낙상 1회**가 발생했다. 활성 입력은 86step(1.72초)였고 이후에는
신선한 disabled→enabled 전이가 없어 gate가 비활성으로 유지됐다. 완료 에피소드 1개 중 낙상
1개, timeout 0개이며 마지막 미완료 에피소드는 107.12초다.

이동 블록마다 시작 2초와 마지막 1초를 제외한 속도 점수화 구간에서
**유효 활성 표본이 모두 0**이다. 상태·위치 응답·전신 통합 검사가 모두 실패했다.
따라서 이 실행에서는 이동 명령의 gain/오차를 산출할 충분한 근거가 없다.
전체 실행의 양손 평균 8.82mm나 XY 속도 오차 0.0365m/s는 대부분 비활성인
상태의 값이며, 걷기 추종 성능으로 사용하지 않는다. 실패 기록을 그대로 보존했다.

## 순수 이동축 추가 학습 실험

`velocity_pure_axes_v7`은 velocity2000에서 500업데이트를 더한
pure_axes2500이다. optimizer·std를 이어 사용하고 초기 재개 learning rate를
0.0001로 정했다. stand 데이터와 다른 보상을 유지하며 단일 축 명령 샘플링,
이동 XY 조건 0.03m/s, yaw 보상 가중치 3·오차 분모 0.10을 사용했다.
새 머리·양손 Cartesian 모션은 추가하지 않았다. 설정을 함께 바꾼 실험이므로
각 변경의 독립적인 효과로 해석하지 않는다. [설정·명령 분포](LOCOMOTION.md)

`eval_pure_axes2500`은 velocity2000의 오프라인 평가와 같은 stand 데이터,
18환경×4500step, seed 2027, 첫 1초를 제외한 블록당 7200샘플로 끝났다.
낙상은 기존과 같은 1/72, timeout 71, 완료 평균 19.939초다. 회전 ±0.30rad/s
명령의 gain은 0.050/−0.003에서 **0.585/0.788**로 개선됐고 MAE는
0.2887/0.3053에서 0.1499/0.1415rad/s로 줄었다. 전후 gain은
0.966/0.945에서 0.863/0.861로 낮아졌고 좌우는 0.740/0.665였다.
정지 중 signed yaw 편향은 −0.06134rad/s로 남았다.
[원본 결과](verification/results/pure_axes2500_blocks_result.json)

외부 UDP의 `whole_body_pure_axes2500`도 완료했다. 기존 velocity2000과
같은 phase 계획·머리/손 진폭·주파수·중립 해시·이동 명령이며, 양쪽 모두
110초·5500step·50Hz, 활성 72초, 낙상 0회, 완료 에피소드 0회다. 상태 검사는
통과했지만 위치·전신 종합 판정은 미달이다. 각 이동 phase의 시작 2초와
끝 1초를 제외한 250샘플씩에서 새 gain은 vx +/− 0.890/0.797,
vy +/− 0.661/0.782, yaw +/− **0.317/1.337**였다. 합친 yaw gain 0.827에는
−0.1020rad/s의 절편이 있어 양의 회전 부족과 음의 회전 과다를 숨길 수 있다.
머리 3축과 양손 Y/Z도 미달했고 양손 X만 통과했다.

영명령 구간에 같은 guard를 적용하면 초기 기립 150샘플의 signed yaw가
velocity2000의 +0.00880에서 −0.13609rad/s로, 마지막 중립 복귀 250샘플은
+0.00999에서 −0.08273rad/s로 바뀌었다. 회전 명령 응답과 함께 이 편향 악화를
기록한다. 6개 이동 plateau의 유효 1500샘플에서 접촉 중 발 링크의 평균
평면 속도도 0.0283에서 0.0410m/s로 커졌다. 이는 발 접촉 진단이며 정확한
접촉점 미끄러짐이나 보행 성공 판정이 아니다.
[실행 기록](verification/results/pure_axes2500_whole_body_result.json),
[시나리오 판정](verification/results/pure_axes2500_whole_body_analysis.json),
[guard 적용 비교](verification/results/pure_axes2500_guarded_comparison.json)

현재 수치를 보행과 상체 조작이 통합된 최종 모델의 근거로 쓰지 않는다.
추가한 v8·v9의 결과는 아래에 따로 기록하며 기본 선택은 제자리 head1100으로 유지한다.

### 후속 후보 비교용 Cartesian 기준 평가

같은 pure_axes2500 가중치를 `g1_teleop_v3.npz`의 평가 5클립으로 바꿔
`eval_pure_axes2500_cartesian`을 별도 실행했다. 18환경×4500step, seed 2027,
동일한 고정 속도 명령열이며 낙상 0/72·timeout 72·완료 평균 19.98초다.
머리 평균 오차 4.3993cm, 양손 4.8610cm였다. 정착 후 yaw ±0.30rad/s
gain은 0.579/1.005, 영명령 signed yaw 평균은 −0.08234rad/s다.
이 기준점은 **stand 평가와 다른 데이터**이며, 후속 gradual 후보를 같은
Cartesian 조건에서 비교하기 위해 보관한다. v8의 완료 결과는 다음 절에 구분한다.
[기준 결과](verification/results/pure_axes2500_cartesian_baseline_result.json),
[조건·데이터 해시](verification/results/pure_axes2500_cartesian_baseline_config.json)

### gradual v8: 정지 회전 편향 개선, 좌우 이동 미달

`gradual_wholebody_v8`은 v7에서 500업데이트를 추가해 누적 3000으로
완료했다. stand×4와 Cartesian×1 클립을 정확히 복사한 데이터, 위치 보상
6/분산0.01·머리 가중치2·높이2/분산0.0025·낙상비용5를 사용했다.
정지 yaw 보상3/분산0.05를 추가했고 이동 yaw3/분산0.10과 pure-axis/0.03
조건은 유지했다. 재개 learning rate는 0.0001이며 optimizer·std도 이어 썼다.
실제 학습 시간 282.54초, actor 최대 변화0.06664, export 오차2.98×10⁻⁷다.
데이터의 초기 train 클립 선택 비율은 약67.8% stand/32.2% Cartesian이다.
[전체 설정](verification/results/gradual3000_train_config.json), [학습 기록](verification/results/gradual3000_train_result.json)

별도 평가는 모두18환경×4500step·seed2027·고정 속도 명령열이다. stand는
v7의 stand와, Cartesian은 위 v7 Cartesian 기준과 데이터 해시·물리·관측·평가
조건이 같은 것을 확인했다. **두 데이터의 수치는 서로 같은 조건으로 비교하지 않는다.**

| 조건 | v7 낙상/완료·timeout | v8 낙상/완료·timeout | v8 완료 평균 | 머리/손 오차 v7 → v8 |
|---|---|---|---:|---|
| stand | 1/72·71 | 1/73·72 | 19.766s | 3.087/2.706 → 1.628/1.623cm |
| Cartesian | 0/72·72 | 0/72·72 | 19.980s | 4.399/4.861 → 3.338/4.388cm |

Cartesian의 정착 후 좌우 gain은0.705/0.754 → **0.247/0.052**로 낮아졌고,
회전 gain은0.579/1.005 →0.678/0.828이었다. 영명령 signed yaw는
−0.08234 →−0.00679rad/s로 줄었다. stand에서도 좌우 gain은0.208/0.045에
그쳤고 우회전 블록에서 낙상1회가 있었다. 위치 오차 감소와 이동 손실을 함께 기록한다.
[stand 결과](verification/results/gradual3000_stand_result.json), [Cartesian 결과](verification/results/gradual3000_cartesian_result.json)

같은 외부 UDP 시나리오의 `whole_body_gradual3000`은110초·50Hz·낙상0회,
활성72초, 완료 에피소드0회였다. 상태 검사는 통과했으나 위치·전신 검사는
미달이다. 시작2초·끝1초를 제외한250샘플/방향의 gain은 vx0.809/0.699,
vy**0.060/0.005**, yaw0.131/0.743으로 좌우 명령을 거의 수행하지 못했다.
머리3축·양손Y/Z도 여전히 미달했다.

같은 guard를 쓴 영명령 초기 기립150샘플의 signed yaw는 v7−0.13609에서
v8−0.00703rad/s, 중립 복귀250샘플은−0.08273에서−0.00005rad/s로 줄었다.
절대 yaw 평균은 각각0.01926·0.02966rad/s로 흔들림은 남아 있다.
정착 후 이동1500샘플의 머리/손 오차는2.313/3.845cm이고 한 발 접촉 비율은
12.5%였다. 발 움직임 감소와 좌우 명령 무시는 함께 해석해야 한다.
[실행](verification/results/gradual3000_whole_body_result.json), [판정](verification/results/gradual3000_whole_body_analysis.json),
[동일 구간 비교](verification/results/gradual3000_guarded_comparison.json)

v8은 통합 성능에 미달한 실험이며 기본 모델로 선택하지 않았다. 이동 XY 보상을
강화한 v9의 완료 결과는 다음 절에 따로 기록한다.

### 최종 이동 실험 v9: 평행 이동 회복, 낙상 증가와 회전 미달

`locomotion_balance_v9`는 v8에서 optimizer·std를 이어 2048환경으로
500업데이트를 추가해 누적 3500으로 완료했다. 학습 데이터 해시와 위치·높이·
낙상·머리·정지/이동 yaw 보상, 명령 분포는 v8과 같음을 확인했다. 이동 XY
보상만 가중치 6·분산 0.01로 바꾸고 재개 learning rate를 0.00005로 정했다.
실행 기록 시간은 285.87초, actor 최대 변화 0.05390, export 오차 4.17×10⁻⁷다.
추가 학습과 두 XY 보상 수치를 함께 바꾼 실험이며 한 값의 독립 효과는 아니다.
[학습 기록](verification/results/balance3500_train_result.json), [설정](verification/results/balance3500_train_config.json)

아래는 각각 v8과 **같은 데이터·18환경×4500step·seed 2027·고정 명령열**이다.
속도 점수는 첫 1초를 제외한 블록당 7200샘플이며 낙상은 전 구간에서 센다.

| 데이터 | v8 낙상/완료·timeout | v9 낙상/완료·timeout | v9 낙상률 | v9 완료 평균 | v9 머리/손 오차 |
|---|---|---|---:|---:|---:|
| stand | 1/73·72 | 11/76·65 | 14.47% | 19.031s | 1.754/1.821cm |
| Cartesian | 0/72·72 | 6/75·69 | 8.00% | 19.505s | 3.480/4.434cm |

Cartesian의 좌우 gain은 0.247/0.052 → **0.918/0.976**, 전후는
0.853/0.775 → 0.968/0.954로 회복됐다. 그러나 Cartesian 낙상 6회는 모두
우측 이동 블록, stand 낙상 11회는 우측 이동 10회·우회전 1회였다. 평균 속도가
명령과 가까워졌다는 이유로 낙상 증가를 제외하지 않는다.
[stand 평가](verification/results/balance3500_stand_result.json), [Cartesian 평가](verification/results/balance3500_cartesian_result.json)

같은 실제 UDP 시나리오 `whole_body_balance3500`은 110초·5500step·50Hz,
활성 72초, 낙상 0회·완료 에피소드 0회다. 시작 2초·끝 1초를 제외한
250샘플/방향에서 전후 gain **0.964/0.920**, 좌우 **1.064/0.948**로 네 방향
평행 이동의 잠정 기준은 통과했다. 회전 +0.20/−0.20rad/s에서는 실제
−0.00383/−0.08004rad/s, gain **−0.019/0.400**으로 양방향 모두 미달했다.
v8의 회전 gain 0.131/0.743보다 낮으며 양의 회전은 평균 부호도 반대다.
상태 검사는 통과했지만 머리 3축·양손 Y/Z도 여전히 미달해 전신 종합 판정은 실패다.
[실행 기록](verification/results/balance3500_whole_body_result.json), [판정](verification/results/balance3500_whole_body_analysis.json)

영명령에 같은 guard를 적용하면 초기 기립 150샘플 signed yaw는 v8−0.00703 →
v9−0.01123rad/s, 중립 복귀 250샘플은 −0.00005 → **+0.04855rad/s**다.
후자의 절대 yaw도 0.02966 → 0.06038rad/s로 늘었다. 이동 plateau 1500샘플에서
머리/손 오차는 2.413/3.754cm, 접촉 중 발 링크 평면 속도는 0.0343m/s다.
같은 v8 값 0.0254m/s보다 크며 발 접촉 진단을 보행 성공 근거로 바꾸어 쓰지 않는다.
[동일 구간 비교 원본](verification/results/balance3500_guarded_comparison.json)

한 로봇·작은 입력의 UDP에서 낙상 0회였어도 더 큰 고정 명령과 여러 환경의
평가에서는 위 낙상이 있었다. v9는 실험용으로만 전달하며 수동 입력 예시는
`--velocity-limits 0.15 0.08 0`으로 회전 스틱을 끈다. 이는 남은 낙상·상체
추종 문제를 해결하거나 실제 Quest 성능을 보증하는 설정은 아니다.
v7도 별도 비교 실험용으로 보존한다. 기본 선택은 제자리 head1100 r2다.

## 기본 모델 번들을 새 경로에서 재실행한 결과

현재 기본 전달 파일은 `artifacts/g1_stationary_v6_r2_20260930.tar.gz`다.
크기 24,834,732바이트, SHA-256은
`a3789f470318f872859a60b0b9b8e604baba1c6cb8c863df8d6dfff925ffbb37`이다.
최초 v6와 **모델·TorchScript 가중치는 같고**, 클러치 조작 안내와 실행 검증
도구 등의 갱신을 포함한다. 최초 파일은 덮어쓰지 않고 보존했다.

r2를 `/home/railabchan/g1_portable_validation_r2`에 풀어 별도 Conda 환경·
독립 Isaac Lab에서 64환경×3000step·seed2026으로 다시 평가했다. 낙상0/192,
timeout192, 완료 평균19.97999954초, 머리1.030839cm·양손1.324320cm이며
원래 head1100의 7개 `mean_metrics` 값과 정확히 같다. 모델·모션·USD·export가
모두 새 r2 경로를 사용하고 205개 번들 파일 및 8개 실행 소스의 해시를 확인했다.
[r2 결과](verification/results/head1100_portable_r2_result.json),
[설정](verification/results/head1100_portable_r2_config.json),
[경로 영수증](verification/results/head1100_portable_r2_path_receipt.json),
[기준과 비교](verification/results/head1100_portable_r2_comparison.json)

이는 같은 PC에서의 이전 검증이며 공용 서버·Quest 성공으로 해석하지 않는다.
다음 최초 v6 기록도 이전 근거와 변경 이력으로 함께 보존한다.

head1100 번들을 `/home/railabchan/g1_portable_validation`에 풀고, 같은 PC의
별도 Conda 환경과 `/home/railabchan/g1_portable_framework/IsaacLab`에서 다시
실행했다. 모델·TorchScript export·모션·USD 모두 새 번들 경로를 실제로 사용했다.
별도 Isaac Lab은 commit `91ad4944f2b7fad29d52c04a5264a082bcaad71d`이며,
원래 체크아웃과 공유하는 inode, 외부 symlink, Git alternates가 없다.
기존 `g1_teleop` 환경의 editable 설치 경로가 보존된 것도 확인했다.
[경로·해시 영수증](verification/results/head1100_portable_path_receipt.json),
[별도 프레임워크 확인](verification/portable_framework.json)

`portable_export_check`는 TorchScript, Cartesian 평가 5클립, seed 2026,
64환경 × 3000step으로 완료했다. **낙상 0/192, timeout 192, 완료 평균
19.97999954초, 머리 1.030839cm, 양손 1.324320cm**다. 원래 `eval_head1100`과
7개 `mean_metrics` 값이 JSON을 읽은 부동소수점 표현까지 정확히 일치한다.
동일성을 확인한 대상은 집계 지표와 평가 조건이며 전체 trace를 비교한 것은 아니다.
물리 시뮬레이터와 FK의 marker 최대 오차도 1.86µm로 허용치 2mm 이내였다.
[원본 결과 사본](verification/results/head1100_portable_result.json),
[실행 설정](verification/results/head1100_portable_config.json),
[원래 평가와 비교](verification/portable_result_comparison.json)

같은 번들로 1환경·200step(시뮬레이션 4초)의 렌더링 미리보기도 완료했다.
낙상과 완료 에피소드가 모두 0이며, 이 짧은 실행은 이미지 출력 경로 확인이다.
위 64환경 평가와 같은 성능 검증이나 Quest 영상 수신 시험으로 간주하지 않는다.
[미리보기 실행 기록](verification/results/head1100_portable_preview_result.json)

이 결과는 **같은 PC에서 설치·리소스 경로를 분리한 이전 가능성 확인**이다.
64환경 headless 처리량 약 74.70Hz는 SteamVR/OpenXR 영상과 동시에 동작하는
실제 헤드셋 루프의 처리량을 보장하지 않는다. 다른 GPU·드라이버를 사용하는
공용 서버와 Quest 연결은 아직 검증하지 않았다. 번들 전달·설치·정확한 실행
명령은 [RUN_SERVER.md](RUN_SERVER.md)를 따른다.

## 체크포인트와 남은 검증

| 완료 평가 대상 | 보관 경로 |
|---|---|
| stand500 | `runs/stand_2048_v2/model_final.pt` |
| rich2500 | `runs/rich_2048_v2/model_final.pt` |
| preprecision1800 | `runs/rich_2048_v2/model_1799.pt` |
| precision500 | `runs/export_precision500/model.pt` 및 원본 `runs/teleop_precision_2048_v3/model_499.pt` |
| precision1500 | `runs/teleop_precision_2048_v3/model_final.pt` |
| tight800 | `runs/teleop_precision_tight_v4/model_final.pt` |
| head1100 — 오프라인·초기화 수정 후 9축 시험 완료 | `runs/teleop_head_precision_v6/model_final.pt` |
| velocity2000 | `runs/velocity_2048_v2/model_final.pt` |
| unified1500 — 실험용, 이동·회전 미달 | `runs/unified_2048_v5/model_final.pt` |
| pure_axes2500 — 회전 응답 추가 실험, 기본 모델 아님 | `runs/velocity_pure_axes_v7/model_final.pt` |
| gradual3000 — 제자리 yaw 개선·좌우 이동 미달 | `runs/gradual_wholebody_v8/model_final.pt` |
| balance3500 — 평행 이동 회복·낙상 증가·UDP 회전 미달 | `runs/locomotion_balance_v9/model_final.pt` |

배포에는 선택한 checkpoint와 같은 디렉터리의 `policy.pt`, `policy.json`,
`run_config.json`, `nominal_targets.json` 및 평가 기록을 함께 전달한다.
`runs/`는 Git 기본 제외 경로이므로 저장소 코드만 받아서는 모델이 생기지 않는다.
기본 선택 모델은 head1100이며 번들 위치·공용 서버 전달 명령은
[RUN_SERVER.md](RUN_SERVER.md)에 모은다. velocity2000/unified1500은 별도 실험용
체크포인트로 구별하며 기본 모델 대신 자동으로 활성화하지 않는다.

최종 전달하는 추가 실험 묶음은 `g1_velocity_v7_20260930.tar.gz`와
`g1_wholebody_v9_20260930.tar.gz`다. v7은 회전 응답과 큰 영명령 yaw 편향,
v9는 개선된 평행 이동과 증가한 낙상·약한 UDP 회전이라는 차이가 있다.
공용 서버의 모델 경로·평가·재개 명령은
[RUN_MOVING_EXPERIMENT.md](RUN_MOVING_EXPERIMENT.md)에 있다.

아카이브 생성 후 v7·v9를 각각 새 경로에 풀어 신규 Conda·독립 Isaac Lab에서
Cartesian+속도 평가(18환경×4500step, seed2027)를 반복했다. 모델·모션·USD·실행
소스가 해당 압축 해제 폴더에서 로드됐음을 해시와 실제 설정으로 확인했다.
두 모델 모두 원래 평가의 낙상·오차·속도 gain 등 10개 지표와 정확히 일치했다.
v7의 0/72 낙상, v9의 6/75 낙상이라는 성능 차이도 그대로 재현됐다.
이는 같은 PC에서의 이전 가능성 검증이며 실제 공용 서버·Quest 검증이 아니다.
[v7 확인 기록](verification/portable_velocity_v7_paths.json),
[v9 확인 기록](verification/portable_wholebody_v9_paths.json).

같은 PC의 신규 Conda 환경에서 설치와 GPU 학습/export를 확인한 근거는
`logs/g1/fresh_environment_verification.json`이다. 이는 다른 서버나 실제
헤드셋의 검증이 아니다. 코드 검사는 [verification/tests.json](verification/tests.json)에
기록된 **고유 테스트 186개**가 각 의존성 환경에서 통과했다. 주 환경은 186개 중
169개 통과·17개 skip, 별도 OpenVR venv의 32개 검사는 그중 9개 skip을 포함하고,
실제 pxr/Gf를 사용하는 8개 검사가 나머지 skip을 확인한다. 186+32+8로 합산하지
않는다. OpenVR/compositor는 모의 런타임이므로 실제 Quest 검증이 아니다.
최초 제자리 v6는 153개, 기본 r2는 180개 검사 시점의 고정 번들이다.
현재 코드·실험 묶음의 186개 검사와 해당 스냅샷을 구별한다.
아직 남은 항목은 다음과 같다.

- head1100의 같은-PC 번들 이전 검증은 완료했다. 공용 서버로의 실제 전달·실행과 실험용 체크포인트의 별도 검증은 구별한다. 방향 유지·복합 동작·보행의 한계를 함께 전달한다.
- 기존 37액션 보행 체크포인트는 23몸체+14손가락 모델용이며, 현재 29몸체 모델과 링크 좌표·질량·effort limit도 다르다. 직접 적용은 채택하지 않았고 성공한 teacher 전이로 보고하지 않는다. 순수 이동축 500업데이트 추가 실험은 평가했지만 영명령 회전 편향과 머리·손 응답 미달이 남아 기본 모델로 선택하지 않았다.
- 실제 Quest의 OpenVR action과 Isaac OpenXR 영상이 같은 SteamVR runtime에서 동시에 동작하는지, 재보정·영상 방향·지연 확인.
- 공용 서버의 GPU·드라이버·네트워크에서 설치, 실시간 50 Hz 유지, 실제 사용자 입력 검증.
- 넓은 동작·자기충돌·지형·사람 다리 모방·손가락/글러브 제어. 현재 결과가 이 범위를 포함하지 않는다.
