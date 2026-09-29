# ALVR / SteamVR backend

Quest 3의 머리·컨트롤러 추적과 버튼 입력을 ALVR을 통해 SteamVR로 전달하고,
Isaac Sim의 Unitree G1 제어 입력으로 변환하는 백엔드입니다.

```text
Quest 3
  → USB ALVR
  → SteamVR / OpenVR
  → pose·button probe
  → 기준 자세 보정과 좌표 변환
  → G1 팔 IK·그리퍼
  → Isaac Sim
```

## 현재 상태

`alvr_probe.py`가 SteamVR/OpenVR에서 HMD와 좌·우 컨트롤러의 포즈, 속도, 트리거, 그립,
스틱과 버튼을 읽습니다. Quest Touch용 SteamVR Action 바인딩을 제공하며, 좌표 원점이 다른 드라이버의
장치를 잘못 섞지 않도록 HMD와 같은 추적 시스템의 컨트롤러만 선택합니다. 이 프로브는 진단 전용이며
G1 명령은 보내지 않습니다.

## 설치와 실행

먼저 ALVR Dashboard와 SteamVR을 실행하고 Quest 3가 연결된 상태로 만듭니다. USB 연결이라면 ALVR의
`Settings → Connection → Wired Client Type`을 `GitHub`로 선택하고 `Devices → Wired Connection`을
활성화합니다. 그 다음 호스트 터미널에서 실행합니다.

```bash
cd /home/railab/Desktop/vr_teleop/alvr
PYTHON_BIN=/home/railab/miniconda3/envs/isaac_vr/bin/python bash scripts/setup.sh
./run_probe.sh
```

포즈와 버튼이 모두 정상이면 `[READY]` 뒤에 `HMD`, `L`, `R`의 미터 단위 좌표가 계속 출력됩니다.
`[TRACKING_ONLY]`는 세 장치의 포즈는 있지만 버튼 Action이 아직 비활성이라는 뜻입니다. Quest Touch의
왼쪽 `primary/secondary`는 X/Y, 오른쪽은 A/B입니다. `PARTIAL`과 `L=-- R=--`가 보이면 Quest 안에서
양쪽 컨트롤러를 들어 버튼을 눌러 깨우십시오. 손 추적 장치는 Quest Touch 입력을 대신하지 않습니다.

전체 값을 JSON Lines로 받거나 로그를 남길 수도 있습니다.

```bash
./run_probe.sh --format jsonl --rate 30
./run_probe.sh --rate 30 --output logs/quest3-input.jsonl
./run_probe.sh --duration 5
```

종료는 `Ctrl+C`입니다. 출력 좌표를 그대로 G1에 보내면 안 됩니다. 중립 자세 보정, 로봇 기준 좌표 변환,
작업공간·속도 제한, dead-man 입력을 다음 제어 계층에서 반드시 적용해야 합니다.

## 구현 순서

1. ALVR의 native wired connection으로 Quest 3와 SteamVR 연결
2. OpenVR background probe로 HMD·왼손·오른손 포즈 및 버튼 수신 (완료)
3. 사용자의 중립 자세를 기준으로 SteamVR 좌표를 G1 기준 좌표로 변환
4. 속도·작업공간 제한과 dead-man 입력을 적용한 뒤 G1 시뮬레이터 IK에 연결
5. Isaac Sim XR 출력을 SteamVR/ALVR로 Quest에 전송

OpenVR 좌표는 오른손 좌표계이며 `+X` 오른쪽, `+Y` 위, `-Z` 앞쪽입니다. 로봇 명령에는 이 값을
직접 사용하지 않고 시작 시점의 HMD/컨트롤러 기준 자세와 로봇 어깨 기준 변환을 적용해야 합니다.

## 운영 제약

- ALVR 실행 중에는 Quest Browser의 WebXR 세션을 함께 시작하지 않습니다.
- 학교 Wi-Fi의 단말 격리를 피하기 위해 초기 개발은 USB 유선 ALVR을 기준으로 합니다.
- 실물 G1에는 직접 연결하지 않습니다. 시뮬레이터에서 관절 제한, 속도 제한, 입력 끊김과 비상 정지를
  먼저 검증한 뒤 별도의 승인 단계로 진행합니다.

참고 자료:

- [ALVR 동작 구조](https://github.com/alvr-org/ALVR/wiki/How-ALVR-works)
- [ALVR 유선 연결](https://github.com/alvr-org/ALVR/wiki/ALVR-wired-setup-%28ALVR-over-USB%29)
- [OpenVR API](https://github.com/ValveSoftware/openvr/wiki/API-Documentation)
