# vr_teleop

Meta Quest 3 기본 컨트롤러 연결 테스트.

공용 서버에서 **Quest 3의 머리와 양손 기본 컨트롤러 입력을 수신**하는 첫 단계입니다.
Quest 브라우저의 WebXR 입력을 Unitree 공식 TeleVuer로 받아 위치·자세·버튼 값을 표시합니다.
이 연결을 확인한 뒤 G1 시뮬레이터의 팔/그리퍼 제어에 연결할 수 있습니다.
이번 코드에는 G1 제어, 카메라 영상 전송, bHaptics 출력은 포함되지 않습니다.

## 공용 서버에 준비할 것

- Ubuntu/Linux, Python **3.10 권장** (3.10에서 검증), `git`, `openssl`, Python `venv` 지원.
- Quest 브라우저에서 접속할 수 있는 서버 IP 또는 DNS 이름.
- 선택한 TCP 포트 접근 가능 여부. 기본은 `8012`이고 `--port`로 바꿀 수 있습니다.
- 입력 수신만 확인하므로 GPU나 Isaac Sim 실행 없이 사용할 수 있습니다.

코드 폴더를 서버의 원하는 경로로 옮기거나 새 Git 저장소에 넣습니다.
`vendor/`, `.venv/`, `certs/`, `logs/`는 옮기지 않아도 됩니다.

### 1. 별도 Python 환경 설치

```bash
cd vr_teleop   # GitHub에서 clone한 폴더. 압축 파일을 풀었다면 실제 폴더 이름으로 변경
bash scripts/setup.sh
```

`setup.sh`는 이 폴더의 `.venv`에만 설치합니다. 기존 Conda/Isaac Sim 환경을 바꾸지 않습니다.
Python 경로를 직접 고를 때는 `PYTHON_BIN=/원하는/python3.10 bash scripts/setup.sh`를 사용합니다.
의존성은 `requirements.lock.txt`에 고정했고, 주요 직접 의존성은 `requirements.txt`에 표시했습니다.

### 2. 서버 주소로 테스트용 인증서 생성

아래 `192.168.1.20`을 **공용 서버의 실제 접속 주소**로 바꿉니다. 이 예시 IP는 실제 설정값이 아닙니다.

```bash
.venv/bin/python scripts/make_cert.py --server-host 192.168.1.20
```

`certs/cert.pem`과 `certs/key.pem`이 생깁니다. 기존 인증서가 있으면 덮어쓰지 않습니다.
서버 주소를 바꾸면 다른 `--out-dir`로 새 인증서를 만들고 실행 시 `--cert`, `--key`로 지정하세요.
기관에서 발급한 인증서가 있다면 생성 단계를 생략하고 그 인증서/키의 경로를 사용할 수 있습니다.

### 3. 컨트롤러 수신 서버 실행

```bash
bash run_probe.sh --bind 0.0.0.0 --server-host 192.168.1.20 --port 8012
```

터미널에 출력된 주소를 **Quest 3 자체 브라우저**에서 엽니다.

```text
https://192.168.1.20:8012/?ws=wss://192.168.1.20:8012
```

1. 직접 생성한 인증서를 쓰면 인증서 경고가 나올 수 있습니다. 주소가 자신의 서버인지 확인하고
   `고급 → 계속 진행`으로 해당 테스트 주소를 엽니다. 이 과정은 Unitree의 Quest 연결 절차에도 안내되어 있습니다.
2. Vuer 화면의 `Virtual Reality` / `Pass-through` 등 XR 시작 버튼을 누르고 요청되는 추적 권한을 허용합니다.
   버튼 표기는 브라우저와 모드에 따라 다를 수 있습니다. 이 프로그램은 패스스루 입력 테스트로 설정됩니다.
3. 양손 기본 컨트롤러를 움직이고, 왼쪽·오른쪽 검지 트리거를 각각 눌렀다가 놓습니다.
4. 서버 터미널의 좌표와 `trigger` 값이 실제 동작에 따라 바뀌는지 확인합니다.

PC 브라우저로 페이지를 여는 것만으로 Quest 컨트롤러 입력이 생기지는 않습니다.
기존 ALVR 연결 여부와 별개로 이 테스트는 Quest 브라우저에서 XR 세션을 시작해야 합니다.

종료: 실행한 서버 터미널에서 `Ctrl+C`.

## 출력 해석과 첫 성공 기준

```text
[Quest3] RECEIVING | head: ... (60 Hz) | left: ... trigger=0.75 | right: ... trigger=0.00
```

위 숫자는 출력 형식 예시입니다. 실제 수신율은 장치/브라우저/네트워크에 따라 달라집니다.

- `WAITING`: 아직 유효한 위치 데이터가 없습니다.
- `PARTIAL_OR_STALE`: 일부 장치만 들어오거나, 마지막 입력 이후 1초가 지났거나, 유효하지 않은 자세가 들어왔습니다.
- `RECEIVING`: 머리와 양손 컨트롤러의 유효한 자세가 모두 최근 1초 안에 들어왔습니다.
- `trigger`: 검지 트리거의 아날로그 값, 0은 놓은 상태이고 1은 끝까지 누른 상태입니다.
- `Hz`: 해당 출력 구간에 실제로 받은 유효 패킷 수/초입니다.

**실물 첫 성공 기준:** 머리를 움직이면 head 값이 바뀌고, 각 컨트롤러를 움직이면 해당 좌표가 바뀌며,
왼쪽/오른쪽 트리거를 각각 누르면 해당 값이 올라갔다가 놓을 때 내려갑니다.
`RECEIVING` 문자열만으로 장치 종류나 움직임의 정확성을 자동 인증하지는 않습니다.

`logs/<실행시간>/latest.json`에는 최근 데이터, `summary.json`에는 종료 결과가 저장됩니다.
JSON에는 4×4 자세 행렬, 위치, 트리거, 그립(squeeze), 스틱, 버튼 값과 입력 경과 시간이 포함됩니다.
`aButton`/`bButton`은 Vuer의 공통 필드 이름입니다. 왼손의 물리 버튼 표시는 X/Y일 수 있습니다.
좌표는 **OpenXR 기준: +X 오른쪽, +Y 위, -Z 앞쪽**, 길이 단위는 미터입니다.
행렬은 열 우선(column-major)이고, G1 로봇 좌표 변환은 아직 적용하지 않았습니다.

## 공용 서버 연결 팁

- 주소는 기존 PC 주소가 아니라 **수신 프로그램을 실행하는 서버 주소**여야 합니다.
- `8012`가 사용 중이면 양쪽 URL과 실행 옵션에서 다른 포트(예: `18012`)를 사용합니다.
  프로그램은 포트를 차지한 다른 사용자의 프로세스를 종료하지 않습니다.
- Quest Wi-Fi와 서버가 서로 접근 가능해야 합니다. 게스트 Wi-Fi 격리, VLAN, 서버 방화벽은 관리자 설정을 확인합니다.
- 기본 바인딩은 `127.0.0.1`입니다. `--bind 0.0.0.0`은 신뢰하는 LAN/VPN에서 접속할 때 사용합니다.
  이 진단 서버에는 사용자 인증이 없으므로 공용 인터넷에 직접 공개하지 않습니다.
- `https://서버주소:포트/healthz`가 열리면 HTTPS 서버까지 접속된 것입니다. 이것만으로 XR 추적 연결이 확인되지는 않습니다.

Quest가 **실행 서버에 USB로 연결되고 ADB가 이미 승인된 경우**에는 네트워크 대신 다음 경로도 쓸 수 있습니다.
ADB 미설치/미승인 상태는 먼저 서버 관리 절차에 따라 준비합니다.

```bash
adb devices
adb reverse tcp:8012 tcp:8012
bash run_probe.sh --server-host localhost
```

이 경우 Quest 브라우저 주소는 `https://localhost:8012/?ws=wss://localhost:8012`입니다.
종료 후 필요하면 `adb reverse --remove tcp:8012`로 해당 포워딩만 해제합니다.

## 검증 및 코드 출처

```bash
.venv/bin/python -m unittest discover -s tests -v
```

테스트는 루프백에서 HTTPS/WebSocket 서버를 잠깐 실행하고 **생성한 가상 입력 패킷**을 보냅니다.
실제 Quest 장치나 외부 서버 접속 없이 패킷 파싱·열 우선 좌표·버튼·입력 끊김·인증서·정상 종료를 확인합니다.
따라서 이 테스트 통과와 실물 Quest 연결 성공은 별도로 확인해야 합니다.

- 공식 수신/브라우저 기능: [Unitree TeleVuer](https://github.com/unitreerobotics/televuer),
  커밋 `766de45e74373ae0ea66321d942ce538385655a5`, Vuer `0.0.60`.
- 기준 공식 저장소: [Unitree xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate),
  커밋 `817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b`이 지정한 TeleVuer를 사용합니다.
- `quest3_probe.py`, 스크립트와 테스트는 이 연결 확인용으로 작성한 별도 코드입니다.
  공식 패키지 파일은 수정하지 않고 설정 가능한 포트·상태 출력·패킷 검증을 추가합니다.
- `params-proto` 3.x에서는 Vuer 0.0.60이 사용하는 `Flag`/`PrefixProto`를 가져오지 못해
  실제 import를 확인한 `2.13.2`로 고정했습니다.

현재 단계의 다음 작업은 서버에서 실물 입력을 확인한 뒤, 이 입력을 G1 팔 IK와 그리퍼 명령에 연결하는 것입니다.
