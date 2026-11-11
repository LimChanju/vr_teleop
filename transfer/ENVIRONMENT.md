# 실행 환경 확인

새 서버의 합의된 목표 구조는 [새 서버 환경 분리 기준](NEW_SERVER_ENVIRONMENT.md)을 따른다. 아래 표와 package origin은 **기존 서버의 재현 기록**이며 새 서버에서 그대로 사용할 환경 설계가 아니다.

실제 interpreter와 package origin을 읽어 확인했다. VR/Isaac 프로세스를 시작하지 않았다.

| 실행 경로 | Python | 패키지 공유 |
|---|---|---|
| ALVR probe | alvr/.venv | isaac_vr Conda Python으로 만든 venv. system-site-packages=false, numpy는 자체 venv, Conda torch 미노출 |
| Browser probe | browser/.venv | 위와 같은 독립 site-packages |
| TWIST2 physics/정책 | TWIST2/.conda-vr | Conda prefix 자체의 패키지 |
| SparsePoser 입력·추정 | TWIST2/.venv-sparseposer | system-site-packages=true. numpy/torch/openvr 등은 .conda-vr에서, PyMotion/Matplotlib은 venv에서 읽음 |
| Unitree Isaac simulator | Conda unitree_sim_env | activation hook이 Isaac4.5 setup_conda_env.sh를 source하여 번들 torch/Isaac 패키지 추가 |
| Unitree teleop | Conda tv | 별도 Python 프로세스 |

SparsePoser는 Conda→venv 상속 구조가 의도적으로 사용됐다. physics 실행기는 Conda에서 정책을 실행하고 SparsePoser venv의 입력 프로세스를 별도로 시작한다. 완전히 독립된 venv 두 개라고 표현하면 안 된다. SparsePoser venv는 user-site도 활성화되어 있으며, 조사 당시 sys.path와 package origin은 비공개 Drive environment 자료에 보존했다.

ALVR/Browser venv의 base_prefix가 isaac_vr인 것은 생성한 Python의 출처를 뜻한다. 두 venv는 include-system-site-packages=false이고 실제 numpy도 각 venv에서 읽었다. Conda 패키지를 공유하는 SparsePoser와 구분한다.

Isaac launcher의 conda_env_exec.sh는 VIRTUAL_ENV와 해당 PATH를 제거한 뒤 unitree_sim_env를 activate한다. 이 방어 처리는 기존 Git commit 33c03ba에 있으며, 새 이식 작업에서 만든 변경은 아니다. launcher_python override나 일반 python 수동 실행 등 모든 과거 실행이 같았음을 보장하는 검사는 아니다.

원본 재현 시에는 실행기에 지정된 Python을 사용한다. prefix가 달라지면 venv symlink, shebang, editable install, activation hook, build cache의 절대경로도 점검한다. venv binary와 Conda base/export는 원본 비교·의존성 확인 자료로 보존한다. 새 서버에서는 VR/센서 및 robot I/O에 각각 독립 venv를 만들고, 과거 SparsePoser의 Conda 상속 구조는 새 설치 절차로 교체한다.

현재 4.5 코드와 환경은 수정하지 않았다. Isaac6.2용 Python/XR 환경 구성은 이후 별도 작업이다.
