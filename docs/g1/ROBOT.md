# G1 모델·정책 계약

이 구현은 **G1 29자유도 몸체 + Dex1 그리퍼**를 Isaac Lab / Isaac Sim 4.5에서
동일한 USD, 액추에이터와 제어 코드로 학습하고 실행한다. 이전 공식 G1 보행
예제의 37차원 액션은 23자유도 몸체와 14개 손가락 관절 모델용이다. 그 정책을
이 29자유도 모델에 그대로 적용하지 않는다.

## 원본 자산

Unitree 자산 저장소:
<https://huggingface.co/datasets/unitreerobotics/unitree_sim_isaaclab_usds>

선택 파일:
`assets/robots/g1-29dof_wholebody_dex1/g1_29dof_with_dex1_rev_1_0.usd`.
`configuration/` 등 같은 디렉터리 안의 참조 파일을 함께 보존해야 한다.
`G1_USD_PATH` 환경 변수로 위치를 지정할 수 있다. 프로그램은 이 변수가 없으면
저장소의 `assets/robots/`와 `assets/robot/`, 기존 로컬 설치를 순서대로 찾는다.

자산을 직접 읽어 확인한 가동 관절은 33개다. 29개 몸체 회전관절은 정책이
제어하고, 네 개 손가락 직선관절은 0.024 m 열린 위치를 유지한다. 손가락
잡기 제어는 이 전신 정책 학습에 포함되지 않는다. 선언된 전체 질량은
약 33.38665 kg이며 질량과 관성은 원본 USD 값을 그대로 사용한다.
실행 시 `policy_metadata()`가 실제 관절 이름, 인덱스, 제한과 초기 자세를 기록한다.

설정 출처는 Unitree의 `robots/unitree.py` 안
`G129_CFG_WITH_DEX1_WHOLEBODY`다. 기존 원통 집기용 `BASE_FIX` 자산은
고정 베이스이므로 사용하지 않는다. 저장소의 Dex3 URDF는 손과 질량이 다르기
때문에 본 학습·배포 모델 대신 사용하지 않는다.

## 29개 액션의 순서

1. 왼쪽 다리: hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll.
2. 오른쪽 다리: 위와 같은 순서.
3. 허리: waist_yaw, waist_roll, waist_pitch.
4. 왼팔: shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll, wrist_pitch, wrist_yaw.
5. 오른팔: 위와 같은 순서.

각 이름에는 `_joint`가 붙는다. 양측 관절에는 `left_` 또는 `right_`가 앞에
붙는다. 실행 시 이름으로 Isaac 관절 배열에 매핑하므로 USD 순서를 가정하지
않는다. 몸체는 rad, 손가락 직선관절은 m 단위다.

## 물리와 PD 제어

물리는 200 Hz (`dt=0.005`), 정책·목표 관절 갱신은 50 Hz (`decimation=4`)다.
액션을 [-4, 4]로 제한한 다음 `q_target = q_nominal + 0.5 * action`으로
계산하고, 실제 USD 관절 범위의 90%에 해당하는 제한을 적용한다.
기본 자세는 양측 hip_pitch=-0.20, knee=0.42, ankle_pitch=-0.23,
elbow=0.87, shoulder_pitch=0.35, 왼쪽 shoulder_roll=0.18,
오른쪽 shoulder_roll=-0.18이며 나머지는 0이다.

| 관절 | Kp | Kd | 최대 토크 (Nm) |
|---|---:|---:|---:|
| hip_pitch / hip_roll / hip_yaw | 200 / 150 / 150 | 5 | 88 / 139 / 88 |
| knee | 200 | 5 | 139 |
| waist_yaw / roll / pitch | 200 | 5 | 88 / 35 / 35 |
| ankle_pitch / roll | 20 | 2 | 35 |
| shoulder_pitch / roll | 100 | 2 | 25 |
| shoulder_yaw / elbow | 50 | 2 | 25 |
| wrist_roll / pitch / yaw | 40 | 2 | 25 / 5 / 5 |

몸체 armature는 0.01이다. Dex1 손가락은 Kp=800, Kd=3, 최대 힘 20 N을
사용한다. 자기 충돌은 꺼져 있고 solver position/velocity iteration은 4/1이다.
바닥 static/dynamic friction은 1/1이다. 이 설정은 실기 G1 제어 명령이 아니다.

Isaac Sim 4.5에서 원본 왼쪽 hip_yaw의 복잡한 convex hull 생성이 실패하여,
양쪽 `*_hip_yaw_link/collisions`의 collision approximation만 실행 시
`boundingCube`로 변경한다. 왼쪽과 오른쪽에 대칭적으로 적용하며 관절,
질량·관성, 발 접촉 형상은 유지한다. 소스 USD는 수정하지 않고 현재 stage에만
적용한다. 이 변경은 학습과 텔레옵 모두에서 같고 체크포인트 메타데이터에 남는다.

## 관측과 목표 좌표계

정책 입력 좌표의 버전은 `g1-yaw-floor-relative-v1`이다. **+X 전방,
+Y 왼쪽, +Z 위쪽**이며, 수평 원점은 골반을 따라가고 회전은 골반 yaw를
따른다. 수직 원점은 지면 위 기본 기립 높이(약 0.767923 m)에 고정한다.
쿼터니언은 Isaac 형식 `wxyz`, 길이는 m다. 골반을 따라 수직 원점까지
내리면 쪼그려 앉는 동작이 목표에서 사라지므로 이 고정 높이가 필요하다.

Student 관측 105차원은 순서대로 다음과 같다.

| 요소 | 차원 | 스케일 |
|---|---:|---:|
| root 각속도 | 3 | 0.25 |
| 중력 방향 | 3 | 1 |
| 몸체 관절 위치 - 기본 위치 | 29 | 1 |
| 몸체 관절 속도 | 29 | 0.05 |
| 직전 액션 | 29 | 1 |
| 머리·왼손목·오른손목 목표 위치 | 9 | 1 |
| vx, vy, yaw rate 명령 | 3 | 1 |

Critic은 root 선속도 3개, 지면 기준 root 높이 1개, reference 관절 위치 -
기본 위치 29개를 추가해 138차원이다. Teacher의 actor도 이 138차원을 사용할
수 있다. Student 텔레옵 실행에는 이 추가 특권 입력이 필요하지 않다.

머리 목표는 `head_link` 원점에 local `[0,0,0.45]` m를 더한 표식이다.
원본 USD의 `head_link` 원점은 실제 머리 형상의 중심이 아니기 때문이다.
손목 표식은 `left_wrist_yaw_link`, `right_wrist_yaw_link` 원점이다.
리타게팅 FK와 환경 측정은 같은 표식 정의를 사용한다. NPZ 원본 표식은
골반 body 좌표이며, 환경 로더가 각 프레임의 기준 root 높이와 기본 기립
높이의 차이를 표식 Z에 더해 위 정책 입력 좌표로 변환한다.
실제 표식은 세계 좌표에서 `(root_x, root_y, ground_z+기립높이)`를 빼고
골반 yaw 역회전을 적용한다. `raw_root_keypoints()`는 변환 전 root body
좌표를 반환하므로 FK와의 정확한 비교에 사용할 수 있다.
Quest 외부 입력에서는 머리 목표의 Z 변화로 목표 root 높이를 유도하며,
관절 상태와 표식 높이 보상으로 crouch를 학습한다. 머리 관절은 고정되어
있어 목을 독립적으로 회전시키는 모델이 아니다.

## 보상과 종료

3개 표식 추종, 기준 관절 자세, 이동/회전 속도, 높이와 자세 유지에 보상을
주며 발 미끄러짐, 불필요한 속도, 토크와 액션 변화를 억제한다. 에피소드는
20초다. root 높이가 0.40 m 미만이거나 수직에서 60도 이상 기울어지면
낙상으로 종료하고, 유한하지 않은 물리 상태도 종료한다. 시간 제한 종료와
낙상 종료는 별도로 반환한다. 평가용 오차는 자동 리셋 직전 값으로 기록한다.

학습 동작 클립이 있으면 기본적으로 무작위 클립 시점의 기준 관절 자세와
root 높이로 초기화한다. 실제 적용할 관절 제한을 거치고 0.015 m 위에서
시작한다. 텔레옵 외부 입력 모드에서는 기본 기립 자세로 리셋한다.
클립 표본 사이를 선형 보간하며, 비주기적인 펀치 동작의 끝에서는 반대 방향으로
재생하여 마지막 프레임에서 첫 프레임으로 갑자기 뛰지 않게 한다. 이는 원본
동작의 역방향 재생도 학습한다는 뜻이며, 원본 데이터의 새 동작이 아니다.
`validate_kinematics()`는 초기화나 물리 스텝 후 최대 네 환경에서 FK와 실제
PhysX 표식 위치 차이를 확인한다.

## ALVR 영상

선택 기능인 `g1_teleop.sim.xr_view.G1XRView`는 Isaac Sim의 native XR
렌더링을 SteamVR/ALVR로 전달한다. 브라우저나 별도 영상 서버를 사용하지
않는다. XR 좌표 원점을 G1 골반 위치와 yaw에 따라 움직이고 첫 연결에서
시점을 머리 표식 앞에 맞춘다. 헤드셋의 회전은 시야에 반영하지만, 물리적인
머리 이동은 XR anchor에서 보상해 로봇의 머리 위치를 시야 위치로 사용한다.
따라서 사용자가 crouch해서 정책이 G1을 낮출 때 카메라가 두 번 낮아지지 않는다.
몸체의 roll/pitch를 시야에 강제로 적용하지 않으므로 이 영상은 고정된
로봇 카메라 센서의 픽셀 출력과 동일하지 않다.

이 기능은 연결된 Quest 3에서 화면·양안 렌더링·지연을 확인해야 한다.
코드 실행이나 가상 패킷 검증만으로 실제 헤드셋 연결 성공을 주장하지 않는다.
제어 입력은 SteamVR의 물리 추적 좌표에서 읽어야 한다. 움직이는 XR 가상
좌표를 다시 정책 목표로 넣으면 로봇 시점과 명령 사이에 피드백이 생긴다.
