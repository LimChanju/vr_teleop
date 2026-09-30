# G1 전신 동작 데이터와 리타게팅

이 프로젝트는 **시뮬레이션에서 사용하는 동일한 Unitree G1 29DoF Dex1 USD의 관절 프레임**으로 동작을 계산한다. H1 관절 각도를 G1에 복사하는 방식이 아니다. G1 관절 각도 29개, 머리·양손 목표점과 골반 높이를 독립적으로 구하고, 강화학습의 동작 목표로 사용한다.

현재 데이터 범위는 제자리 팔 뻗기·펀치와 균형 유지다. 원본 `stable_punch.pkl`은 모든 클립의 골반 XY 이동이 0이다. 이 데이터로 보행·전신 동작 전반을 학습했다고 주장하면 안 된다.

## 파일

- `g1_teleop/motion/g1_29dof_kinematics.json`: 배포용 USD에서 추출한 숫자형 관절 프레임·축·각도 제한. 소스 USD 레이어별 SHA256 포함.
- `g1_teleop/motion/kinematics.py`: NumPy 순기구학, 선택적인 PyTorch 미분 가능한 순기구학.
- `g1_teleop/motion/retarget.py`: H1 Cartesian 방향을 G1 기구학에 맞추는 최적화.
- `g1_teleop/motion/procedural.py`: 초기 제자리 동작, 별도의 이동 명령 커리큘럼 함수.
- `g1_teleop/motion/library.py`: pickle 없이 NPZ를 읽고 클립 내에서 보간.
- `scripts/g1/prepare_motions.py`: 데이터 생성 실행 파일.
- `data/motions/g1_stand_v1.npz`: 생성한 제자리 연습 데이터, 12클립·4320프레임·30Hz. 마지막 2클립은 평가용.
- `data/motions/g1_punch_v1.npz`: 실제 H1 펀치 동작을 리타게팅한 29클립·3899프레임·30Hz. 24학습·5평가 클립.
- `data/motions/g1_reach_v1.npz`: 더 넓은 합성 팔 뻗기·몸통 기울기·얕은 앉기, 20클립·9600프레임. 16학습·4평가. 균형 유지 이후의 2단계 커리큘럼용.
- `data/motions/g1_mixed_v1.npz`: 위 세 데이터의 61클립·17819프레임을 합쳤으며 원래 평가 클립을 그대로 보존한다.
- 같은 이름의 `.json`: 입력 파일 해시, 출처, 클립별 분할과 최적화 오차.

## 관절·좌표 계약

순서는 `JOINT_NAMES`에 정의되어 있다. 왼다리 6, 오른다리 6, 허리 3, 왼팔 7, 오른팔 7이다. 각도는 rad, 길이는 m이며 +X 앞, +Y 왼쪽, +Z 위다. 그리퍼 관절은 이 29개에 포함하지 않는다.

NPZ의 `keypoints`는 **골반 body 원점에 대한 로컬 점**이다. 외부의 골반 위치·회전은 넣지 않는다. 순서는 `[head, left_wrist, right_wrist]`다.

`head_link`의 USD 원점은 중립 자세에서 골반에 가깝다. 따라서 머리 목표점은 **head_link의 로컬 `[0,0,0.45]`**를 변환한 점이다. 실제 헤드 링크 원점을 헤드셋 위치로 간주하면 안 된다. 손 목표점은 `left_wrist_yaw_link`, `right_wrist_yaw_link`의 body 원점이며 손가락 끝이 아니다.

런타임 환경이 중립 골반 높이에 기준을 둔 yaw 좌표계를 사용하는 경우, 데이터 로더에서 목표점의 z에 `root_height - nominal_root_height`를 더한다. 저장 파일 자체에는 이 변환을 중복 적용하지 않는다. 기준 자세에서 사람이 착용한 헤드셋·컨트롤러를 해당 로봇 목표점에 맞추는 보정도 별도로 필요하다.

## 리타게팅 방법

1. H1 MJCF 관절 축과 원본 `pose_aa`로 H1 순기구학을 계산한다.
2. H1 상완·전완·허벅지의 Cartesian 방향을 구하고 G1의 해당 부위 길이로 조정한다.
3. G1의 29개 관절과 골반 높이를 Adam으로 최적화한다. 손목·팔꿈치·무릎 위치, 몸통 방향, 제자리 양발 위치와 수평 발바닥을 목적 함수에 넣는다.
4. 모든 반복에서 G1의 실제 USD 관절 제한을 적용한다. 기본 자세 정규화, 손목 정규화, 관절 속도·가속도 및 골반 높이 변화 페널티를 추가한다.
5. 약한 시간 평활화를 적용한 후 G1 FK로 최종 머리·손목 목표점을 다시 계산한다. 클립 경계를 넘어서 평활화하거나 보간하지 않는다.

원본 파일에는 중요한 차이가 있다. `stable_punch.pkl`의 `pose_aa` 하체는 제자리 서기 자세로 편집되어 있지만, legacy `dof` 필드는 이전 하체 각도를 갖고 있다. 공식 MotionLib와 동일하게 **pose_aa를 기준**으로 하며, 둘의 최대 차이를 보고서에 기록한다. 원본 SMPL 점들도 저장된 H1 골반 위치와 좌표가 일치하지 않으므로 그대로 G1 목표점으로 쓰지 않는다.

현재 구현은 관절·발 위치에 관한 기구학 최적화다. 자기충돌·접촉 동역학 최적화까지 수행한 것은 아니다. 실제 안정성과 추종 성능은 학습된 정책을 Isaac Sim에서 별도로 평가해야 한다.

처음에는 `g1_stand_v1`으로 균형을 배우고, 같은 관측·액션 계약으로 `g1_mixed_v1`에서 학습을 이어갈 수 있다. 넓은 동작 데이터가 실제로 안정적인지는 별도 동역학 평가가 필요하다.

## 이번 생성 결과

250회 IK 최적화/클립으로 생성한 결과다. 평균은 클립별 평균의 평균이다.

| 항목 | 결과 |
|---|---:|
| 왼손목 Cartesian 리타게팅 오차 평균 | 3.69 mm |
| 오른손목 Cartesian 리타게팅 오차 평균 | 3.47 mm |
| 가장 큰 클립별 손목 오차 95백분위 | 11.67 mm |
| 양발 제자리 위치 오차 평균 | 0.145 mm |
| 관절 제한 위반 | 0 |
| 가장 빠른 참조 관절 속도 | 6.63 rad/s |
| 참조 골반 높이 | 약 0.71685 m |

이는 **입력 동작을 G1 기구학에 맞춘 오차**다. 강화학습 정책의 실제 추종 오차나 낙상률이 아니다. 합성 제자리 데이터의 중립 골반 높이는 약 0.76792 m다. 펀치 데이터는 무릎을 더 굽힌 자세이므로 시작 자세를 참조에 맞춰야 한다.

## 재생성

NumPy만 있으면 합성 데이터와 런타임 MotionLibrary를 사용할 수 있다. 원본 리타게팅에는 `torch`, `scipy`, `joblib`이 추가로 필요하다. 기존 Isaac 환경에 패키지를 덮어쓰지 말고, 설치 스크립트의 별도 환경 또는 이미 검증한 환경을 사용한다.

```bash
python scripts/g1/prepare_motions.py
python scripts/g1/prepare_motions.py \
  --h1-source /path/to/human2humanoid/legged_gym/resources/motions/h1/stable_punch.pkl \
  --h1-xml /path/to/human2humanoid/legged_gym/resources/robots/h1/xml/h1.xml \
  --iterations 250 --device cpu --merge
```

`--h1-source`는 pickle을 읽으므로 신뢰하는 공식 데이터 파일만 지정한다. 완성된 학습·텔레옵 실행에서는 실행 코드를 포함하지 않는 NPZ만 읽는다.

```python
from g1_teleop.motion import MotionLibrary
library = MotionLibrary('data/motions/g1_punch_v1.npz')
reference = library.sample(clip_ids=[0, 3], times=[0.4, 1.2])
# reference['q']: [2,29]
# reference['keypoints']: [2,3,3]
# reference['root_height']: [2]
# reference['command_velocity']: [2,3]
```

`sample()`은 기본적으로 클립 마지막 프레임에서 멈춘다. `loop=True`를 명시하면 반복하지만, 비주기적 펀치 동작의 끝과 시작 사이에는 자세 차이가 있을 수 있으므로 환경 리셋을 권장한다. `sample_batch(..., split='test')`는 평가 전용 클립만 선택한다.

NPZ 키는 `q`, `keypoints`, `root_height`, `command_velocity`, `clip_id`, `clip_start`, `clip_length`, `clip_names`, `split`, `joint_names`, `fps`, `schema`다. `split`은 클립별 0=학습, 1=평가다. 펀치와 제자리 데이터의 `command_velocity`는 전부 0이다.

`velocity_curriculum()`은 작은 이동·회전 속도 명령을 만드는 별도 함수다. 학습 진행도 50% 이전에는 0을 반환한다. 보행을 생성한 데이터가 아니며, 비영 명령을 사용하려면 속도 추종 보상과 실제 보행 학습·평가가 필요하다. 제자리 학습에서는 진행도를 0으로 둔다.

## 검증과 출처

```bash
python -m unittest discover -s tests/g1 -p 'test_motion*.py' -v
```

USD의 알려진 중립 관절 프레임, NumPy/PyTorch FK 일치와 수치 미분, 클립 단위 분할, 보간 경계, 관절 제한, 참조 발 높이를 검증한다. 시뮬레이터와의 FK 대조는 별도 환경 검증 결과를 참고한다.

- [LeCAR-Lab human2humanoid](https://github.com/LeCAR-Lab/human2humanoid), commit `750f1fa052641f0fde43669d50cb4e407dabe6c8`. H1 skeleton과 제공된 stable_punch에서 파생한 연구 데이터. CC BY-NC 4.0이며 원본 모션 데이터의 권리·이용 조건도 적용된다. 원본 학습 코드를 이 모듈로 복사하지 않았고, FK 리타게팅 구현을 새로 작성했다.
- [Unitree unitree_sim_isaaclab](https://github.com/unitreerobotics/unitree_sim_isaaclab), commit `e30c25b1dffdf92ada1d6c8c1fe9a47bdde0fecc`. 배포용 G1 Dex1 USD의 숫자형 기구학을 추출했다. Apache 2.0.
- 해당 라이선스 전문은 `g1_teleop/motion/licenses/`에 보존한다. 합성 데이터와 리타게팅된 연구 데이터의 출처를 혼동하지 않는다.
