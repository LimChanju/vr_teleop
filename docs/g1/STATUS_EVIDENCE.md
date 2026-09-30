# G1 구현·검증 근거와 남은 범위

**확인 시점: 2026-09-30 19:36 KST.** 이 문서는 완료된 실행 기록의 스냅샷이다.
최종 배포 모델을 선택하는 문서가 아니며, 이후 학습의 성능을 추정하지 않는다.
최종 선택은 체크포인트별 별도 평가와 최종 결과 보고서를 따른다.
아래 `runs/`, `logs/`는 개발 PC의 실제 기록이며 Git 기본 제외 경로다.
서버 이전 시 선택한 모델과 해당 평가 기록도 함께 전달해야 한다.

## 구현한 범위

`human2humanoid`의 전신 참조 동작 추종, privileged teacher, sparse student
구조를 참고해 **G1 29개 몸체 관절·Dex1·free-base Isaac Lab 환경으로 새로
구현**했다. 원본 H1 코드를 그대로 실행하거나 논문의 결과를 재현한 것이 아니다.
H1 펀치는 H1 FK의 Cartesian 방향에서 G1 기구학 최적화를 거쳐 리타게팅했다.
관절 각도를 복사하지 않는다. 출처·비상업적 연구 데이터 조건은
[동작 문서](MOTION.md)를 따른다.

학습·평가·텔레옵은 같은 로봇 USD, 관절 순서, PD, 액션 스케일과 환경을 쓴다.
머리·양손 위치와 로봇 상태를 입력받고 29관절 위치 목표를 출력한다. Dex1 손가락은
열린 자세를 유지한다. 사람의 발·무릎 자세나 컨트롤러 회전·손가락 자세를 직접
복원하는 정책은 아니다. 아래 평가에 사용한 v1 관측은 actor 105/critic 138차원이다.

현재 코드에는 목표 오차·목표 속도·로봇 선속도를 더한 선택형
`sparse_tracking_v2`도 존재한다(actor 126/critic 156). **v1 평가 수치는 v2 성능을
입증하지 않는다.** 실행 계약은 해당 모델의 `run_config.json`/`policy.json`과
일치시켜야 한다. [학습 설정](TRAINING.md), [로봇 계약](ROBOT.md)

## 실제로 확인한 실행

| 항목 | 확인 결과 | 원본 기록 |
|---|---|---|
| 기존 환경과 분리한 신규 설치 | 같은 PC에서 `--fresh`로 새 Conda 환경 설치, user-site 차단, `pip check` 통과 | `logs/g1/fresh_environment_verification.json` |
| 신규 환경 GPU 학습·export | 64환경 × PPO 3업데이트, 실제 파라미터 변화 0.0113958, TorchScript 최대 오차 4.47×10⁻⁸ | `runs/g1_server_check_smoke/result.json` |
| 로봇 FK와 실제 Isaac 상태 대조 | 평가 초기 4환경의 머리·손목 marker 최대 차이 약 1.57–2.72×10⁻⁶ m | 평가별 `run_config.json`의 `kinematics_validation` |
| 초기 균형 정책 학습 | 2048환경 × PPO 500업데이트, 이후 별도 평가 실행 | `runs/stand_2048_v2/`, 아래 표 |
| Teacher → Student 실행 경로 | teacher 3업데이트 → student 5업데이트 → TorchScript 50step 실행 | `runs/teacher_smoke_v1/`, `runs/student_smoke_v1/`, `runs/student_smoke_eval/` |
| 합성 입력 → 실제 시뮬레이터 | UDP 입력, TorchScript 추론, 29관절 PD 제어, 입력 중단 후 정지 상태 확인 | `runs/teleop_synthetic_stand500/` |
| OpenVR 등록·입력·실패 처리 | 개발 입력 venv에서 VR 테스트 21개 통과; OpenVR API는 실제 ctypes 자료형을 쓰는 모의 런타임 | `tests/g1/test_vr*.py`, [ALVR 문서](ALVR.md#7-steamvr-앱-식별과-동시-입력의-검증-범위) |

새 환경은 Python 3.10.21, Isaac Sim 4.5.0.0, PyTorch 2.5.1+cu121,
RSL-RL 2.3.3, NumPy 1.26.4와 별도 고정 Isaac Lab 체크아웃을 사용했다.
기존 동일 버전 Kit의 라이선스 동의 기록을 재사용했다. 새 입력 venv 설치 시점의
14개 테스트와 이후 매니페스트 보완 후 개발 venv의 21개 테스트는 서로 다른
시점의 기록이다. **다른 공용 서버에서 설치한 결과는 아니다.** [설치 상세](INSTALL.md)

Teacher/Student의 짧은 실행은 설정·가중치 학습·export 연결 확인이다.
Student 평가 50step은 환경당 1초이고 완료 에피소드가 없어 낙상률을 산출할 수 없다.
이 모델들을 수렴한 teacher나 사전학습 완료 student로 해석하지 않는다.
성능이 확인된 아래 500업데이트 모델은 별도 증류 없이 학습한 비대칭 PPO다.

## v1 별도 평가 결과

모두 64환경, 학습에 포함하지 않은 클립 분할이다. 손 오차는 양손 위치 오차의
평균이며 전신 관절 오차가 아니다. 완료 에피소드의 시간 제한은 약 20초다.

| 평가 기록 | 참조 데이터 / seed / 환경당 실행 | 양손 오차 | 낙상 / 완료 에피소드 | 완료 에피소드 평균 |
|---|---|---:|---:|---:|
| `baseline_stand_v3` — 중립 PD | stand / 42 / 30초 | 8.75 cm | 1181 / 1181 | 1.58초 |
| `eval_stand_500` — stand500 | stand / 42 / 30초 | **1.37 cm** | **0 / 64** | 19.98초 |
| `baseline_mixed_v3` — 중립 PD | mixed / 2026 / 60초 | 26.62 cm | 2827 / 2827 | 1.34초 |
| `eval_mixed_stand500` — stand500 | mixed / 2026 / 60초 | **13.00 cm** | **2 / 192** | 19.79초 |
| `eval_mixed_1200` — mixed `model_1199.pt` | mixed / 2026 / 60초 | **16.85 cm** | **23 / 192** | 18.99초 |

각 숫자의 원본은 `runs/<평가 기록>/result.json`이다. `stand500`은
`runs/stand_2048_v2/model_final.pt`를 뜻하며 두 평가 모두 export된 TorchScript를
사용했다. mixed `model_1199.pt` 평가는 RSL-RL 추론 경로다. 같은 이름의 학습 로그
평균을 별도 평가 수치로 대체하지 않았다.

500업데이트 모델은 작은 제자리 팔 동작에서 균형과 추종을 배웠다. 같은 모델을
넓은 팔 뻗기·펀치·앉기 데이터로 평가하면 손 오차가 커진다. mixed 추가 학습의
중간 체크포인트도 이 평가에서는 개선되지 않았다. **학습 횟수를 늘렸다는 사실만으로
넓은 전신 텔레옵 준비가 완료됐다고 판단할 수 없다.**

0/64는 관측한 완료 에피소드에서 낙상이 없었다는 뜻이며 모든 입력에서의 안정성
보증이 아니다. 미완료 에피소드 시간도 원본에 보관했다. 전체 환경 지표는 리셋
직전 상태를 집계한다. `trace.npz`는 환경 0만 기록하므로 분석 시 리셋 경계를 제외하고
전체 낙상 통계와 구별한다. [평가 방법](VALIDATION_METHOD.md)

## 가상 입력과 실제 Quest를 구별

`teleop_synthetic_stand500`은 실제 시뮬레이션 45초·2250step을 실행했고,
그중 **617step(12.34초)**만 유효한 합성 조작 입력이 활성 상태였다. 45초 전체를
연속 조작 시간으로 보고하지 않는다. 낙상 0회, 전체 실행의 양손 오차 평균
2.53 cm였으며 정지 상태의 구간도 이 평균에 포함된다. 원본에는
`input_sources=["synthetic"]`, `physical_quest_verified=false`가 기록되어 있다.

실제 Quest 3의 추적·버튼, ALVR 전송, SteamVR Background action과 Isaac OpenXR의
동시 활성화, 헤드셋에서의 G1 시점 영상·지연·재보정은 **아직 실제 장비로 검증하지
않았다**. 모의 OpenVR 테스트나 합성 UDP 성공은 이를 대신하지 않는다.

현재 평가한 동작 데이터의 이동·회전 명령은 모두 0이다. 스틱 입력과 속도 명령
코드는 있어도 이 기록으로 보행·회전 정책 학습을 입증할 수 없다. 이동 성능,
큰 몸통 변화와 넓은 팔 작업 범위, 실제 사용자 입력에 대한 안정성은 별도 평가가
필요하다. 최종 모델·번들·공용 서버 명령은 최종 결과 보고서에서 확정한다.
