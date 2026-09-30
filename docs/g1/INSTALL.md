# 공용 서버 설치와 모델 전달

이 G1 경로는 개발 PC와 공용 서버에서 같은 Isaac Sim/Isaac Lab 환경으로
학습·평가·텔레옵을 실행하도록 구성했다. 기존 H1 Isaac Gym 환경이나 기존 G1
예제 환경을 덮어쓰지 않는다. 모델의 실제 성능과 검증 상태는 해당 체크포인트의
평가 보고서를 확인한다.

## 검증 기준 버전

| 구성 | 기준 |
|---|---|
| OS / Python | Linux x86_64 / Python 3.10 |
| Isaac Sim | 4.5.0.0 |
| Isaac Lab | `91ad4944f2b7fad29d52c04a5264a082bcaad71d` |
| PyTorch | 2.5.1+cu121 |
| RSL-RL | 2.3.3 |
| NumPy | 1.26.4 |
| ALVR 입력 Python | 별도 `.venv-alvr`, openvr 2.12.1401 |
| 로봇 | Unitree G1 29개 몸체 관절 + Dex1 손가락 4개, free-base USD |

`config/g1/requirements-supplement.txt`는 기존 환경의 user-site에만 있던 의존성을
새 환경 내부로 설치하는 고정 버전 목록이다. `environment.freeze.txt`와
`conda-explicit.txt`는 개발 환경 기록이다. 특히 freeze 파일에는 로컬 editable
경로가 있으므로 다른 서버에서 그대로 `pip install -r`하지 않는다.

## 1. 실행 환경 준비

### A. 기존 Isaac 환경 없이 새로 설치

Conda와 git이 설치된 Linux x86_64 서버에서 실행한다. 공식 Isaac Sim 4.5 pip
패키지에 필요한 GLIBC 2.34 이상을 사전 확인한다. Ubuntu 22.04의 GLIBC 2.35는
이 조건을 만족한다. NVIDIA 드라이버는 서버에서 별도로 준비되어 있어야 하며
이 설치 스크립트가 변경하지 않는다.

```bash
bash scripts/g1/setup.sh --fresh \
  --target-prefix "$HOME/anaconda3/envs/g1_teleop_server" \
  --dry-run

bash scripts/g1/setup.sh --fresh \
  --target-prefix "$HOME/anaconda3/envs/g1_teleop_server"

export G1_PYTHON="$HOME/anaconda3/envs/g1_teleop_server/bin/python"
```

`--fresh`는 새 Python 3.10 Conda 환경을 만들고 PyTorch 공식 CUDA 12.1 인덱스의
2.5.1 빌드, NVIDIA 인덱스의 `isaacsim[all,extscache]==4.5.0.0`, 고정 Isaac Lab
소스와 `config/g1/requirements-fresh.txt`의 호환 의존성을 설치한다. 패키지 다운로드와
설치 공간이 필요하며 최초 시뮬레이터 실행 시 확장 캐시 준비가 추가로 진행될 수 있다.
`--fresh`와 `--source-env`는 함께 지정하지 않는다. 설치된 환경 자체는 이 검증에서
약 16 GB였으며 다운로드 캐시 공간은 별도다.

새 Kit 설치는 첫 실행에서 NVIDIA 라이선스 동의를 확인한다. 설치 스크립트는 이를
자동으로 새로 수락하지 않는다. 첫 실행을 터미널에서 진행하면 제조사의 확인 문구가
표시된다. 이 PC의 검증에서는 기존 동일 버전 Kit의 동의 기록을 재사용했다.

### B. 기존 Isaac 환경을 복제해서 준비

서버에 **이미 실행되는 Python 3.10 / Isaac Sim 4.5 / PyTorch 2.5.1+cu121 Conda
환경**이 있는 경우를 지원한다. 아래 `unitree_sim_env`는 서버의 실제 환경 이름으로
바꿀 수 있다. 체크아웃한 `vr_teleop` 최상위에서 실행한다.

```bash
# 읽기 전용 사전 검사와 예정 명령 출력
bash scripts/g1/setup.sh \
  --source-env unitree_sim_env \
  --target-prefix "$HOME/anaconda3/envs/g1_teleop_server" \
  --dry-run

# 새 환경 설치
bash scripts/g1/setup.sh \
  --source-env unitree_sim_env \
  --target-prefix "$HOME/anaconda3/envs/g1_teleop_server"

export G1_PYTHON="$HOME/anaconda3/envs/g1_teleop_server/bin/python"
```

스크립트는 원본 환경 버전을 먼저 확인하고 다음을 수행한다.

1. 별도 `vendor/IsaacLab-g1` 폴더에 위 커밋을 체크아웃한다.
2. 원본 Conda 환경을 새로운 경로로 복제한다.
3. **복제한 환경에만** 추가 의존성과 RSL-RL을 설치하고, 전용 Isaac Lab 체크아웃으로
   editable 연결을 바꾼다.
4. `pip check`와 핵심 패키지 버전을 검사한다.
5. ALVR 입력용 `.venv-alvr`를 별도로 만든다. 이미 같은 버전의 입력 환경이 있으면
   수정 없이 재사용한다.

이미 존재하는 대상 Conda 경로, 변경된 Isaac Lab 체크아웃, 맞지 않는 기존 입력
환경은 덮어쓰지 않는다. 다른 경로를 지정한다. 설치 실패로 일부 새 환경이 남았다면
그 위치를 확인하고 새 이름으로 다시 준비한다. 설치 작업은 NVIDIA 드라이버, 시스템
CUDA, ALVR·SteamVR 설정, 원본 Conda 환경을 수정하지 않는다.

Miniconda 등 다른 경로는 `--conda`, 전용 Lab 위치는 `--isaaclab-dir`, 입력 환경
위치는 `--vr-prefix`로 지정한다. 환경 복제에는 원본과 새 환경을 위한 디스크 공간이
필요하다.

새 환경 설치 경로는 [Isaac Lab의 Isaac Sim 4.5 pip 설치 안내](https://isaac-sim.github.io/IsaacLab/v2.1.1/source/setup/installation/pip_installation.html)의
방식을 사용하되 이 프로젝트의 기준 버전을 고정한다. 최신 문서의 5.x/6.x 설치
명령으로 대신하지 않는다. 새 Conda 환경 설치 검증 결과는 아래 검증 기록에 남긴다.

## 2. 정확한 G1 로봇 에셋 준비

기존 Unitree 에셋이 있다면 약 51 MB의 해당 로봇 폴더만 복사한다.
`--source`에는 Unitree 저장소, `assets` 폴더, 해당 로봇 폴더 또는 루트 USD를
지정할 수 있다.

```bash
python3 scripts/g1/assets.py \
  --source /실제/경로/unitree_sim_isaaclab
```

기존 파일이 없다면 공식 Unitree Hugging Face 에셋 묶음을 사용한다.

```bash
python3 scripts/g1/assets.py --download
```

전체 ZIP 약 1.31 GB를 캐시에 내려받지만, 추출하는 것은 다음 폴더 하나다.

```text
assets/robots/g1-29dof_wholebody_dex1/
├── g1_29dof_with_dex1_rev_1_0.usd
├── configuration/
├── config.yaml
└── asset_manifest.json
```

공식 [Unitree 다운로드 스크립트](https://github.com/unitreerobotics/unitree_sim_isaaclab/blob/main/fetch_assets.sh)가
참조하는 [에셋 저장소](https://huggingface.co/datasets/unitreerobotics/unitree_sim_isaaclab_usds)의
커밋 `394cf2448f8a9ed815c77c701a761f3d1ff1c8fb`를 고정했다. ZIP SHA256은 다음과 같다.

```text
06fbf14549be3a81e3dbd4a8a019e7f060f48de1556b70f291a18ac8f9ead4b0
```

다운로드 크기와 해시를 확인하고 개별 파일 해시도 `asset_manifest.json`에 저장한다.
`--zip /path/assets.zip`으로 기존 ZIP을 사용할 수도 있다. ZIP 경로 이탈·심볼릭 링크는
허용하지 않는다. 기존 에셋 경로가 있으면 그대로 보존하며 종료한다. 캐시 위치는
`--cache`, 설치 위치는 `--destination`으로 바꿀 수 있다.

다른 설치 위치를 사용하면 다음처럼 루트 USD를 지정한다. USD 한 파일만 옮기면
참조하는 `configuration/`이 빠지므로 **폴더 전체**가 필요하다.

```bash
export G1_USD_PATH="/실제/에셋/폴더/g1_29dof_with_dex1_rev_1_0.usd"
```

## 3. 모델과 코드 전달용 번들 만들기

학습이 끝나고 평가한 run 폴더를 선택한다. run 폴더에는 다음 파일이 필요하다.

- `model_final.pt`: 학습 재개·시뮬레이터 실행 체크포인트
- `policy.pt`, `policy.json`: TorchScript 정책과 해시·관측/액션 계약
- `run_config.json`, `nominal_targets.json`: 실행 설정과 입력 보정 기준

번들에는 선택한 모델, 참조하는 동작 데이터, G1 코드·설정·문서와 선택한 로봇 에셋을
넣는다. 기존 README의 WebXR 안내가 이어지도록 `docs/WEBXR_PROBE.md`와 작은
브라우저 진단 소스·설치 파일도 보존한다. Conda 환경, `.venv`, 인증서·키,
브라우저 vendor 저장소는 넣지 않는다.

```bash
python3 scripts/g1/bundle.py create \
  --run runs/실제_평가한_학습_폴더 \
  --assets assets/robots/g1-29dof_wholebody_dex1 \
  --evidence runs/선택한_평가_폴더 \
  --output dist/g1_teleop_bundle.tar.gz

python3 scripts/g1/bundle.py verify dist/g1_teleop_bundle.tar.gz
```

에셋을 별도로 준비할 경우 `--assets`를 생략한다. `model_final.pt` 이외의 체크포인트는
그 체크포인트를 대상으로 export한 폴더를 함께 지정한다.

`--evidence`는 반복 지정할 수 있다. 선택한 평가·텔레옵 폴더의 `result.json`,
설정, `trace.npz`, `analysis/`의 작은 분석 결과를 `evidence/`에 넣는다.
가상 입력 검증의 `scenario.jsonl`과 최상위 `scenario_analysis*.json`도 포함한다.
각 폴더는 200개 파일·64 MiB 이하여야 하며 다른 학습 체크포인트나 원시
`input.jsonl`은 포함하지 않는다. `BUNDLE.json`에는 평가가 선택한 모델과 같은
체크포인트인지, 다른 비교 모델인지 또는 확인할 수 없는지 기록한다.

```bash
python3 scripts/g1/bundle.py create \
  --run runs/원래_학습_폴더 \
  --checkpoint runs/원래_학습_폴더/model_1000.pt \
  --policy-dir runs/해당_체크포인트_export_폴더 \
  --output dist/g1_teleop_model1000.tar.gz
```

생성물은 TAR와 외부 `.sha256` 파일이다. 내부에는 모든 파일의 `SHA256SUMS`와
`BUNDLE.json`이 들어 있다. 선택한 체크포인트는 번들에서
`models/<학습폴더이름>/model.pt`로 통일된다. 정책 SHA256과 모델 계약이 일치하지
않거나 원본 파일이 복사 중 바뀌면 번들 생성을 중단한다. 같은 출력 파일은 덮어쓰지 않는다.

모델 계약에는 관절 순서, PD·충돌 설정, 제어 주기, 기준 자세, 액션 크기와 관측
버전이 포함된다. 이전 메타데이터에서 관측 버전이 빠진 경우 실행 코드와 동일하게
`sparse_positions_v1`로 해석한다. `sparse_tracking_v2`는 추가 관측 순서와 목표
속도 추정기 설정도 일치해야 한다. 학습 당시 `source_sha256`에 기록한 소스는
해시를 검증해 모델 폴더의 `source/`에 별도로 보존한다. 실행용 코드는 번들 최상위에
있으며, 소스 스냅샷은 학습 당시 구현을 확인하기 위한 기록이다.

해시는 전달된 파일의 무결성을 확인하며 정책 성능이나 실제 Quest 연결 성공을
보증하지 않는다. 별도의 평가 보고서도 함께 확인한다.

## 4. 서버에서 번들 풀기와 실행

번들과 `.sha256` 파일을 서버로 옮긴 뒤 같은 디렉터리에서 외부 해시를 확인한다.

```bash
sha256sum -c g1_teleop_bundle.tar.gz.sha256
```

GitHub의 해당 G1 브랜치를 이미 받은 경우 제공한 검사·추출 도구를 사용할 수 있다.
대상은 새 폴더여야 하며 기존 저장소 위에 덮어쓰지 않는다.

```bash
python3 /path/to/vr_teleop/scripts/g1/bundle.py unpack \
  g1_teleop_bundle.tar.gz --destination "$HOME/vr_teleop_g1_run"
cd "$HOME/vr_teleop_g1_run"
```

도구는 TAR의 경로·파일 종류와 내부 해시를 검증하고 최상위 `vr_teleop_g1/` 이름을
제거해서 새 대상 폴더로 추출한다. `BUNDLE.json`에서 모델 폴더를 확인한다.

```bash
cat BUNDLE.json
export G1_PYTHON="$HOME/anaconda3/envs/g1_teleop_server/bin/python"

# 성능을 확인할 때는 모델과 함께 제공한 평가 조건/명령을 우선 사용한다.
bash scripts/g1/run.sh evaluate \
  --checkpoint models/실제_학습폴더이름/model.pt --headless --steps 1000

# 기존 SteamVR/ALVR 그래픽 세션에서 G1 텔레옵 실행
bash scripts/g1/run.sh teleop \
  --checkpoint models/실제_학습폴더이름/model.pt --steps 0 --real-time --xr
```

별도 터미널의 ALVR 입력 실행과 보정 순서는 [ALVR.md](ALVR.md)를 따른다.
`--nominal models/실제_학습폴더이름/nominal_targets.json`을 입력 프로그램에 전달한다.
같은 모델을 계속 학습하려면 다음처럼 실행한다.

```bash
bash scripts/g1/run.sh train \
  --checkpoint models/실제_학습폴더이름/model.pt \
  --iterations 1000 --num-envs 1024 --headless
```

환경 변수 `G1_PYTHON`, 선택 시 `G1_USD_PATH`는 새 셸에서도 설정한다.
디스플레이·XR 런타임·SteamVR가 필요한 `--xr` 실행은 그래픽 사용자 세션에서
검증해야 한다. SSH만 연결된 세션의 성공 여부와 같지 않다.

## 설치 도구 자체 검사

```bash
bash -n scripts/g1/setup.sh
python3 scripts/g1/assets.py --self-test
python3 scripts/g1/bundle.py self-test
```

에셋 도구는 임시 폴더에서 선택 추출, 경로 이탈·심볼릭 링크 거절, 덮어쓰기 방지,
해시를 검사한다. 번들 도구는 생성/검증/추출과 정책 해시, 제외 경로, 덮어쓰기 방지를
검사한다. 이 검사는 실제 GPU 시뮬레이션 또는 공용 서버 설치 검증을 대신하지 않는다.

## 새 환경 실제 실행 검증 기록

2026-09-30, 개발 PC에서 기존 환경을 복제하지 않고 `--fresh` 경로로 새
`g1_server_check` Conda 환경과 별도 입력 venv를 설치했다. Isaac Lab도 별도로
내려받은 위 고정 커밋을 사용했고 Python user-site 패키지 사용을 비활성화했다.

- Python 3.10.21, Isaac Sim 4.5.0.0, PyTorch 2.5.1+cu121, RSL-RL 2.3.3,
  NumPy 1.26.4를 확인했다.
- 첫 `pip check`에서 누락된 `pyglet`을 찾아 설치 목록에 1.5.31로 고정한 뒤
  새 환경에만 설치했다. 최종 `pip check`가 통과했다.
- 해당 새 환경에서 G1 **64개 환경 × PPO 3회 업데이트**를 실행했다.
  4,608개 샘플, 정책 파라미터 최대 변화량 0.0113958,
  TorchScript 내보내기 최대 오차 4.47×10⁻⁸, 정상 종료를 확인했다.
- 새 입력 venv에서 VR 입력 테스트 14개를 통과했다.

이것은 **설치·학습·모델 내보내기 경로의 실행 검증**이다. 3회 업데이트 모델을
사전학습 완료 모델로 제공하는 것은 아니다. 최종 사용할 정책은 별도 장기 학습과
평가 결과에 따라 선택한다. 같은 PC의 새 환경에서 검증했으며 실제 공용 서버 OS,
Quest 연결과 XR 영상은 현장에서 확인해야 한다.

개발 PC의 상세 기록은 `logs/g1/fresh_environment_verification.json`,
`logs/g1/fresh_setup.log`, `logs/g1/fresh_setup_completion.log`,
`logs/g1/fresh_smoke_retry.log`에 있다. 설치 검증 실행 결과는
`runs/g1_server_check_smoke/result.json`에 저장되어 있다.
