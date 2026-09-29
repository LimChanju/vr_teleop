# vr_teleop

Meta Quest 3로 Unitree G1 시뮬레이터를 조종하기 위한 실험 저장소입니다.
입력 전송 방식이 다른 두 백엔드를 독립 폴더로 관리합니다.

| 백엔드 | 용도 | 현재 상태 |
| --- | --- | --- |
| [`browser/`](browser/) | Quest Browser WebXR → Unitree TeleVuer | 머리·양손 컨트롤러 입력 수신 프로브와 테스트 구현 완료 |
| [`alvr/`](alvr/) | Quest ALVR → SteamVR/OpenVR → G1 시뮬레이터 | 공식 G1 14개 태스크 런처·입력 어댑터·안전 게이트 구현 완료 |

## 선택 기준

- Unitree 공식 TeleVuer 입력 경로를 검증하려면 `browser/`를 사용합니다.
- Isaac Sim 화면을 Quest로 스트리밍하면서 컨트롤러 입력도 함께 사용하려면 `alvr/`를 사용합니다.
- ALVR과 Quest Browser WebXR은 동시에 immersive XR 세션을 점유할 수 없으므로 한 번에 하나만 실행합니다.

각 백엔드의 설치 및 실행 방법은 해당 폴더의 README를 따릅니다. G1 제어를 연결하기 전에는
시뮬레이터에서만 입력, 좌표 변환, 제한값과 비상 정지 동작을 검증합니다.
