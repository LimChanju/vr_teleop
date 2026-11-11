# 새 서버 환경 분리 기준 — 2026-11-11

사용자가 정한 새 서버의 목표 구조다. 기존 서버의 실행 환경을 그대로 재생성하는 절차와 구분한다. Conda 계열과 Pixi 중 최종 도구 선택은 아직 확정하지 않는다. Miniconda는 Conda를 설치하는 배포판이므로 여기서는 Conda 계열에 포함한다.

| 영역 | 환경 | 담당 범위 |
|---|---|---|
| 강화학습·시뮬레이션·계산 | Conda/Miniconda 또는 Pixi | Isaac Sim/Lab, MuJoCo, 학습, GMR/MuJoCo 기반 retargeting |
| VR·센서 트래킹 | 독립 Python venv `vr-tracking` | OpenVR/VIVE 입력, 장치 식별, 보정, 사람 pose/body 추정 및 기록 |
| 실제 로봇 연결 | 별도 독립 Python venv `robot-io` | 로봇 SDK/DDS/센서 통신, 명령 송신, 상태 수신 |

이름은 설명용이며 경로와 Python 버전은 각 dependency의 호환성을 확인한 뒤 정한다. 실제 로봇 연결은 새 서버의 목표 범위이며, 현재 보존된 TWIST2 실로봇 출력이 검증 완료됐다는 뜻은 아니다.

## 환경 생성·실행 원칙

- 두 venv는 서로 다른 환경으로 생성하며 `--system-site-packages`를 사용하지 않는다.
- venv의 base Python도 Conda/Pixi 환경에 의존시키지 않는다. 해당 환경과 별도로 설치한 버전별 Python을 사용한다.
- SparsePoser를 트래킹 venv에서 실행할 경우 필요한 Torch/NumPy/PyMotion도 그 venv에 설치한다. 부모 Conda의 site-packages를 가져오지 않는다.
- Conda와 Pixi를 같은 prefix에 겹쳐 설치하지 않는다. 시뮬레이터별 dependency가 충돌하면 시뮬레이션 영역 안에서도 별도 환경을 둔다.
- 프로세스별 Python 경로를 명시한다. Conda/Pixi를 activate한 셸에 venv를 중첩해 모든 역할을 실행하는 방식은 사용하지 않는다.
- `PYTHONPATH`, user-site, 동적 라이브러리 경로가 다른 영역의 Python 패키지를 끌어오지 않는지 package origin으로 확인한다. VR/드라이버에 필요한 라이브러리 경로는 각 프로세스에 필요한 범위로 설정한다.
- 환경마다 dependency 정의와 lock/export를 따로 관리한다. 서로 다른 Python 버전은 IPC 경계에서 분리한다.

## 프로세스 경계

```text
vr-tracking venv
  OpenVR/VIVE → 보정 → 사람 pose/body
                         │ 직렬화한 pose + timestamp + validity
                         ▼
Conda/Pixi 계산 프로세스
  GMR retargeting / MuJoCo·Isaac simulation / 학습
                         │ 명령 및 상태 프로토콜
                         ▼
robot-io venv
  로봇 SDK·DDS 통신 → 명령 전송 / 실제 상태 수신
```

역할 간에는 JSONL/소켓/Redis 등 명시적 프로토콜을 사용한다. Python 객체나 import 경로를 공유하는 방식으로 연결하지 않는다. 길이 단위·좌표계·quaternion 순서·timestamp·validity·메시지 버전을 프로토콜에 기록한다. 기존 35D reference 등 재사용 가능한 계약은 보존하고, 새 경계는 이후 구현 시 검증한다.

정책 추론을 어느 프로세스에서 실행할지는 주기·지연 요구와 dependency를 확인한 뒤 결정한다. 실제 로봇 제어용 추론을 robot-io venv에 둘 경우에도 필요한 inference package는 해당 venv 안에 독립 설치한다. 실로봇 운용에서 시뮬레이터 프로세스를 반드시 실행해야 한다는 설계는 아니다.

ALVR·SteamVR·VIVE Hub·udev·GPU driver는 Python venv 자체에 설치되는 구성 요소가 아니다. 호스트의 별도 런타임으로 관리하며 Python venv는 해당 입력/통신 클라이언트를 담당한다.

## 현재 코드에서 이후 바꿔야 할 지점

현재 `.venv-sparseposer`는 `include-system-site-packages=true`이고 `.conda-vr`의 NumPy/Torch/OpenVR를 상속한다. 이는 **원본 서버의 기록**이며 새 서버의 목표 구조로 채택하지 않는다.

또한 현재 `twist2_vr/sparseposer_sim_input.py`는 MuJoCo를 직접 import하고, SparsePoser/G1 입력 경로가 GMR 변환과 결합되어 있다. venv를 독립 생성하는 것만으로 위 분리 기준을 만족하지 않는다. 이전 후 다음을 별도로 구현해야 한다.

1. VR/사람 추정 출력과 GMR/MuJoCo retargeting 프로세스를 분리한다.
2. `setup_sparseposer.sh`의 부모 Conda 의존 생성 방식을 독립 venv 설치 절차로 바꾼다.
3. `.conda-vr`와 `.venv-sparseposer` 경로를 고정한 실행기에서 역할별 Python/프로세스 경계를 지정한다.
4. 기존 `tv` Conda 기반 teleop 경로 중 실제 로봇 I/O가 담당할 부분을 독립 `robot-io` venv로 이식하고, 계산/정책 부분과 구분한다.
5. 각 프로세스의 interpreter/package origin, 메시지 형식, 기록 재생 및 새 서버 장치 연결을 검증한다.

이번 변경은 문서에 목표 구조를 반영한 것이다. 현재 서버의 환경과 Isaac4.5/teleop 실행 코드는 수정하지 않았다. 새 서버에서 환경 생성·6.2 이식·실제 로봇 연결을 완료한 상태도 아니다.

## Drive 경량 묶음과의 관계

기존 경량 압축 파일3개는 그대로 사용한다. 데이터·모델·보정·VR 런타임은 환경 관리 방식과 독립적으로 필요하다. `environment/` export 및 압축본의 과거 설치 설명은 원본 의존성을 확인하는 자료다.

`03-restore-info.tar.gz` 안 README의 환경 복원 단계는 원본 서버를 재현하는 설명이다. **새 서버의 환경 설계는 이 문서의 사용자 합의를 우선한다.** 현재 setup 스크립트를 그대로 실행하면 여전히 과거 Conda→venv 상속 구조가 만들어질 수 있으므로, 새 구조에 맞춘 설치/실행기 변경 후 사용한다. 기존 압축본이나 SHA256SUMS를 교체할 필요는 없다.
