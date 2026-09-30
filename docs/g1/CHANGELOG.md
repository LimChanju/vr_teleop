# 변경·보존 기록

- 작업 브랜치: `g1-whole-body-alvr-20260930`, 시작 커밋 `faed634409af8565fc00d27e51bc157c7c6791c8`.
- 원래 WebXR 입력 진단 코드는 유지했다. 기존 README는 `docs/WEBXR_PROBE.md`로 보존했다.
- 기존 README와 `.gitignore`의 수정 전 사본은 로컬 `backups/20260930_initial/`에 보관했다. Git에도 변경 전 내용이 남아 있다.
- Isaac Sim 4.5, 원래 Isaac Lab, Unitree 예제, `unitree_sim_env`, `omnih2o` 환경을 수정하지 않았다.
- `unitree_sim_env`를 새 `g1_teleop` Conda 환경으로 복제했다. 기존 환경이 사용자 사이트에서 참조하던 패키지를 새 환경에 명시적으로 설치하고 `PYTHONNOUSERSITE=1`로 분리했다.
- ALVR 입력은 `.venv-alvr`에 따로 설치했다. 기존 ALVR/SteamVR 설정 파일, 드라이버와 시스템 CUDA는 변경하지 않았다.
- 원본 로봇 USD는 읽기 전용으로 사용한다. 두 hip-yaw 충돌 메시의 convex cooking 오류는 실행 중 USD stage의 `boundingCube` 설정으로 처리한다. 학습과 실행에 같은 설정을 쓴다.
- 로봇 학습·텔레옵 코드는 `g1_teleop/`, `scripts/g1/` 아래에 새로 작성했다. 기존 37-action 보행 정책은 다른 G1 형태이므로 이번 29-action 정책에 섞어 쓰지 않았다.
- 각 학습 폴더 `source/`와 `run_config.json`에 당시 환경 코드와 설정 해시를 남긴다. 수정 중 실행한 초기 실패·smoke 폴더는 최종 모델로 지정하지 않는다.
- 새 설치 경로도 `g1_server_check` 환경과 전용 `vendor/IsaacLab-g1`에서 검증했다. 원래 환경을 복제하지 않고 설치했으며, 기존 동일 버전 Kit 라이선스 동의 기록을 재사용했다. 공용 서버 자체에서 실행한 결과는 아니다.
- 관측 v2를 추가하기 전 소스는 `backups/20260930_tracking_v2/`에 보관했다. v1 105/138 입력을 유지하면서 v2 126/156 입력을 선택할 수 있으며, 가중치 이전은 함수 출력 보존 검사를 거친다.
- SteamVR 앱 식별 변경 전 소스는 `backups/openvr_identity_20260930_192703/`에 보관했다. 실제 SteamVR 설정을 실행·변경하지 않고 코드와 API mock 테스트를 수행했다.
- 텔레옵 기록에는 입력 시각·시퀀스·fresh 상태·수동 리셋을 추가했다. 변경 전 런처와 분석기는 `backups/20260930_runtime_trace/`에 보관했다.
- 정밀 추종 보상을 선택 설정으로 추가하기 전 환경은 `backups/20260930_precision_reward_config/`에 보관했다. 기본 보상은 그대로 유지하며 선택값은 모델 설정에 기록한다. 재개 시 PPO optimizer와 adaptive learning-rate 상태를 함께 복원한다.
- 독립 Cartesian 텔레옵 데이터를 새 파일로 추가했다. 초기 v1/v2의 끝점 속도를 확인한 뒤, 마지막 난수 knot 간격이 너무 짧아지는 문제를 수정한 v3를 생성했다. 기존 제자리·H1 리타게팅·혼합 데이터와 split은 보존했다.
- 입력 상태 검사와 추종 품질 검사를 분리했다. 연결·시작·정지 처리가 정상이어도 특정 축의 움직임을 충분히 따라가지 못하면 추종 성공으로 기록하지 않는다.
- 최종 문서 변경 전 사본은 `backups/20260930_final_documentation/`에 보관했다. 코드는 별도 브랜치에만 올리고 기존 main 브랜치는 변경하지 않는다.
- 낙상 진단 추가 전 환경은 `backups/20260930_failure_diagnostics/`, 런처·평가 문서는 `backups/20260930_failure_events/`에 보관했다. 평가할 때만 전체 환경의 낙상 직전 상태를 기록하며 학습 보상과 관측은 유지한다.
