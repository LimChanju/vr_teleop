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

폴더와 구현 경계를 먼저 분리한 단계입니다. ALVR 수신 코드와 G1 명령 코드는 아직 포함하지 않습니다.
현재 개발 머신에서는 ALVR 20.14.1, SteamVR, Isaac Sim 4.5.0과 NVIDIA GPU가 확인됐으며,
Quest 3 유선 클라이언트 프로필도 존재합니다. 다음 단계는 `Disconnected` 상태인 유선 세션을 복구하고
HMD와 양쪽 컨트롤러의 포즈·트리거 값을 독립 프로브로 검증하는 것입니다.

## 구현 순서

1. ALVR의 native wired connection으로 Quest 3와 SteamVR 연결
2. OpenVR background application으로 HMD·왼손·오른손 포즈 및 버튼 수신
3. 입력 유효성, 수신 주기, stale timeout과 JSON 로그 추가
4. 사용자의 중립 자세를 기준으로 SteamVR 좌표를 G1 기준 좌표로 변환
5. 속도·작업공간 제한과 dead-man 입력을 적용한 뒤 G1 시뮬레이터 IK에 연결
6. Isaac Sim XR 출력을 SteamVR/ALVR로 Quest에 전송

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
