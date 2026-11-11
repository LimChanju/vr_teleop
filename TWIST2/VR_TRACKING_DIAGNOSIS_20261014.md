# Quest 3와 VIVE 트래커 추적 장애 조사

2026년 10월 14일 밤 기준. 대상은 0072의 추적 손실, 다른 트래커들의 공통 위치 오프셋, 양손 Quest 컨트롤러 인식 불량이다. 현재 단계의 문제는 TWIST2 정책이나 로봇 관절 제약보다 앞에 있는 입력 장치와 공간 정렬에서 발생한다.

**확인된 장애와 아직 확정하지 못한 원인을 구분해야 한다.** 0072는 VIVE 원본 위치 추적부터 실패한다. 다른 트래커의 보정 변환은 적용되고 있지만 그 변환이 현재 물리 공간에도 맞는지는 다시 검증해야 한다. Quest 컨트롤러는 ALVR에 도달하기 전에 Quest 내부에서 연결이 반복해서 끊긴다.

| 문제 | 확인된 사실 | 원인 판단 | 다음 조치 |
| --- | --- | --- | --- |
| 0072 Ready 불안정 | 배터리 70%, 44%에서도 발생. 최근 약 12분 중 Ready는 약 9% | 저배터리만으로 설명 불가. 장치별 시야·맵 재인식·하드웨어 문제를 구분해야 함 | 정상 0314와 같은 위치·방향에서 교차 비교 |
| 트래커들이 약 2m 앞에 보임 | 보정 성공 뒤 VIVE map host 교체 및 맵 재동기화. 저장된 변환은 여전히 한 번 적용됨 | 원본 좌표 원점 변경으로 기존 보정이 맞지 않게 됐을 가능성이 가장 강한 단서. 실제 원점 점프는 미확정 | 양쪽 시스템 안정화 후 정렬 검증, 어긋나면 재보정 |
| 양손 컨트롤러 인식 실패 | Quest 자체 로그에서 연결→끊김 반복. 연결 시 양쪽 배터리 100% 보고 | Quest 내부 컨트롤러 연결 장애는 확인. 헤드셋 상태·무선 연결·펌웨어 중 근본 원인은 미확정 | 헤드셋 정상 재시작 후 Quest 자체 홈에서 먼저 확인 |

복구 순서는 **Quest 컨트롤러 정상화와 VIVE 단독 추적 확인 → 공간 정렬 → 몸에 장착한 입력 검증**이다. 앞의 두 작업은 병행할 수 있지만, 불안정한 컨트롤러나 트래커를 기준으로 재보정하면 안 된다.

## 조사 범위와 보존 상태

- ALVR 20.14.1, 별도 설치된 SteamVR 2.12.14, VIVE Hub Linux 1.0.3, xi-ve Space Calibrator를 확인했다.
- 10월 13일부터의 HTC 서버·셋업·슬롯 복구 로그, 10월 14일 Quest ADB 기록, SteamVR·ALVR 설정, 로컬 어댑터와 공식 소스를 대조했다.
- 이 조사에서는 장치 재시작, 페어링 변경, 맵 초기화, 보정값 변경, 펌웨어 설치, 로봇 실행을 하지 않았다. 공식 코드는 수정하지 않았다.
- 새로 저장한 것은 이 보고서와 진단 증거다. 사용자 부재 상태의 23:09 조회에서는 트래커 5개가 오프라인이고 Quest가 절전 상태였다. 이 시점만 보고 사용 중 고장의 원인을 판단하지 않았다.

기존 장착 매핑은 유지한다. `0072=왼발등`, `0314=오른발등`, `0320=왼쪽 정강이`, `0086=오른쪽 정강이`, `0082=배 앞쪽`이다. 비교 시험을 위해 잠시 떼더라도 역할 설정을 바꾸거나 로봇을 실행하지 않는다.

## 주요 시간 기록

시간대는 Asia/Tokyo이며 한국 시간과 같은 UTC+9이다. Quest 로그와 Linux 로그 사이의 정밀한 시계 동기화는 별도로 검증하지 않았으므로 서로 다른 장치의 수초 단위 선후 관계를 과도하게 해석하지 않는다.

| 시각 | 기록 |
| --- | --- |
| 10월 13일 22:00:27 | VIVE 공간 맵 저장 완료 기록과 `has_map=1` |
| 10월 13일 22:06 이후 | 0072 배터리 70%에서도 Ready와 Rotation_only 반복 |
| 10월 14일 14:43:42 | 0072 배터리 44%, Ready가 된 0.1초 뒤 Rotation_only |
| 22:31:04 | 실제 오른손 컨트롤러와 0314로 xi-ve 보정 저장 |
| 보정 직후 | 별도 20초 이동 검증에서 쌍의 상대 자세가 약 19mm와 1.78도 수준으로 유지됨 |
| 22:37:21 | Quest 오른손 컨트롤러의 첫 보존된 연결 끊김 |
| 22:38:02 | VIVE 기존 map host 0086에 RF drop 기록 |
| 22:38:07 | `host re-picked: slot 0 -> 1 (previous host offline)`, 새 host는 0314 |
| 22:39:15 이후 | Quest 왼손 컨트롤러도 연결 끊김 반복 |
| 22:40 이후 | VIVE client 트래커들의 맵 재동기화 |
| 22:44:47 | 0072 맵 동기화 완료 뒤에도 lost_tracking과 Rotation_only 반복 |
| 23:00:49부터 | 오른손 재연결 시 버전 경고·IMU 오류, 약 8초 뒤 다시 끊김 |
| 23:01:51부터 | 왼손 재연결 시 같은 유형의 경고·오류, 약 10초 뒤 다시 끊김 |
| 23:02:41 이후 | 헤드셋을 벗은 뒤 RUNNING→STANDBY→SLEEP 전환 |

출처는 보존한 [VIVE 로그 발췌](runtime/diagnostics/20261014_vr_incident/vive_tracker_log_excerpt.txt), [Quest 로그 발췌](runtime/diagnostics/20261014_vr_incident/quest_controller_log_excerpt.txt), [22시 31분 보정 검증](runtime/diagnostics/20261014_vr_incident/spacecal_2231_pair_validation.json)이다.

## 0072의 위치 추적 실패

### 배터리와 보정만의 문제는 아니다

0072는 충분한 배터리가 있었던 이전 기록에서도 같은 상태 전이를 반복했다. 최근 배터리 8%는 충전해야 할 이유지만, 이 장애를 처음 발생시킨 충분한 설명은 아니다.

10월 14일 22:44:48부터 22:57:00까지 732초 동안 각 장치의 상태 전이 사이 시간을 합산한 결과는 다음과 같다. 로그 이벤트 개수 비율이 아니라 시간 비율이며, Ready 시간은 실제 위치 정확도의 인증이 아니다.

| 트래커 | Ready 시간 | Ready 비율 | 나머지 상태 |
| --- | --- | --- | --- |
| 0072 | 64.10초 | 8.76% | Lost 84.90초, Rotation_only 583초 |
| 0314 | 732초 | 100% | 해당 구간 다른 상태 없음 |
| 0086 | 732초 | 100% | 해당 구간 다른 상태 없음 |
| 0320 | 732초 | 100% | 해당 구간 다른 상태 없음 |
| 0082 | 652.65초 | 89.16% | Rotation_only 79.35초 |

0072가 Ready였던 24개 구간 중 가장 긴 구간도 8.15초였다. 따라서 화면에 Ready가 잠깐 나타났다는 것만으로 복구됐다고 판단하면 안 된다. 다른 시각의 8초 측정에서는 0086과 0320도 일부 Rotation_only였으므로, 위 표를 하루 전체 상태로 일반화하지 않는다.

사용자가 트래커가 날아간다고 보고한 뒤의 8초 측정에서 0072는 위치 유효 샘플이 없었다. 위치 필드는 `[-12.0097, -2.72635, -7.75765] m`에 고정됐고, 원본 상태부터 `rotation_only`, `valid=false`였다. 0314의 위치 필드와 약 14.2m 차이가 나지만 이는 **무효 좌표의 차이이지 실제 측정 거리나 날아간 궤적이 아니다**. 날아가는 순간 자체는 포착하지 못했다. [측정 요약](runtime/diagnostics/20261014_vr_incident/vut_0072_tracking_failure.json)

현재 VIVE Hub API는 위치 추적을 잃은 상태에서도 포즈 패킷을 보낼 수 있다. 패킷이 새로 도착하거나 회전값이 변하는 것과 위치가 유효한 것은 다르다. 실제 사용하는 API 문서는 [설치된 Hub SDK 설명](external/VIVEHub-Linux-v1.0.3/sdk/README.md)의 상태·좌표계 항목이다. 별도 SDK의 상태 번호나 Isaac 좌표축 설명을 이 경로에 섞으면 안 된다.

### 원인 후보와 배제하지 말아야 할 점

1. **카메라 시야와 맵 재인식 문제:** 발등에서 실패하고 높이·장소를 바꾸면 잠시 살아나는 사용자 관찰과 부합한다. 렌즈 가림, 신발·바지·몸·책상, 반사나 무늬 없는 바닥, 낮은 각도 맵 품질이 후보이다. 단, 다른 트래커와 같은 조건의 비교가 아직 없어 확정은 아니다.
2. **0072 개별 장치 문제:** 같은 조건에서도 0314는 정상이고 0072만 실패하면 우선순위가 높아진다. 현재 로그에 카메라 하드웨어 고장이 명시되지는 않았다. 오류 로그가 없다고 카메라 정상이라고 판정할 수는 없다.
3. **맵 공유 또는 Linux beta 동작:** 맵 동기화 완료 뒤에도 실패하므로 단순히 다운로드를 더 기다리는 것으로 설명하기 어렵다. 장치별 재인식이나 맵 내용 문제까지 배제되는 것은 아니다.
4. **무선 연결:** 다른 시각에 RF drop이 있으므로 완전히 배제할 수 없다. 그러나 주요 Ready↔Rotation_only 반복 구간은 재등록·재동기화 없이 진행돼 단순 연결 끊김보다는 위치 추적 실패에 더 부합한다. `DeviceIsConnected=true`만으로 무선 정상이라고 판정하지 않는다.

HTC는 렌즈 청소, 충분한 확산 조명, 반사·단조로운 벽과 바닥 개선, 장애물과의 거리 확보를 안내한다. 하체에서 추적을 잃으면 가슴 높이에서 카메라를 바깥쪽으로 향해 복구한 뒤 천천히 낮추도록 안내한다. [HTC 추적 환경 안내](https://www.vive.com/au/support/ultimate-tracker/category_howto/tracking-is-not-stable-or-accurate.html), [HTC 하체 추적 복구](https://www.vive.com/eu/support/ultimate-tracker/category_howto/cant-see-tracker-when-attached-to-lower-body.html)

### 재페어링이나 전체 초기화를 바로 하면 안 되는 이유

- 과거 슬롯 복구 도구는 공식 SDK의 Pairing/Unpair 등을 사용했고 맵이나 카메라 내부 보정값을 쓰는 코드는 없다. 이후 5개 시리얼과 슬롯은 일관되게 구분됐다. 슬롯 복구가 0072를 손상시켰다는 증거는 없다.
- `trackers.json`에 0072가 빠진 것은 시리얼 도착 직전에 저장된 캐시와 부합한다. 실제 동글의 5개 페어링과 원본 포즈는 별도로 확인됐다. 캐시 파일만 보고 수동 수정하거나 언페어링하지 않는다.
- 공간 스캔은 저장 완료 기록이 있다. “스캔이 아예 저장되지 않음”과 “발 높이에서 맵 품질이 부족함”은 다른 문제다.
- 5개 트래커의 보고 펌웨어는 모두 `1.0.999.168`, 동글은 `9.17`이다. 0072만 버전이 다른 증거는 없다.
- 설치된 HTC 실행파일은 공식 v1.0.3 배포 압축파일의 바이너리와 해시가 일치했다. 현재 버전에는 과거 host Wi-Fi 조회 및 맵 동기화 수정이 이미 포함돼 있다. [공식 변경 기록](https://github.com/ViveRoboticsSoftware/vivehub-linux/blob/main/CHANGELOG.md)

### 해결을 가르는 한 번의 비교 시험

충전과 렌즈 청소를 준비한 뒤, 로봇 출력 없이 0072와 정상 0314를 같은 방향으로 두고 **가슴 높이 30초, 발 높이 30초, 서로 자리 교환 후 반복**한다. 두 카메라가 서로나 손에 가려지지 않게 하고 실제 장치 이동과 원본 포즈를 함께 기록한다. 이 시간은 비교 시험 제안이지 제조사가 정한 합격 기준은 아니다.

| 결과 | 다음 조치 |
| --- | --- |
| 장소를 바꿔도 실패가 0072를 따라감 | 개별 렌즈·장치·재인식 문제 우선. 복구 동작과 0072 전원 재시작 후 재비교 |
| 0314도 같은 왼발 위치에서 실패 | 장착각·가림·바닥 특징·저각도 맵 개선 우선 |
| 여러 장치가 동시에 끊기며 RF drop 동반 | 동글 배치와 개인 무선 장비 조건을 바꿔 비교. 학교 네트워크를 원인으로 단정하거나 공유 장비를 끄지 않음 |
| Linux에서만 0072 실패 | Windows 공식 VIVE Hub에서도 동일 조건 비교하여 Linux 경로와 장치 문제 분리 |
| Windows에서도 0072만 실패 | 보존한 로그와 동일 조건 비교 영상으로 HTC 지원에 장치 점검 요청 |

HTC의 위치 불일치 복구는 가슴 앞에서 카메라를 바깥으로 향한 뒤 **렌즈를 만지지 않고 5초 가렸다가 열기**, 실패 시 해당 트래커 전원 재시작이다. 이미 반복해도 실패했다면 계속 같은 동작만 시키지 말고 위 비교로 넘어간다. [HTC 위치 불일치 복구](https://business.vive.com/uk/support/ultimate-tracker/category_howto/tracker-position-in-vr-and-real-world-dont-match.html)

Windows 비교나 맵 재작성은 기존 맵·호스트·펌웨어 상태를 바꿀 수 있으므로 별도 후속 작업으로 한다. 수행 뒤에는 Quest와의 공간 정렬을 다시 확인해야 한다.

## 네 트래커가 함께 앞쪽에 보이는 문제

### 보정은 실제로 한 번 적용되고 있었다

22:31에 저장된 프로필은 실제 오른손 컨트롤러 `2G0YXX0X0000XX_Controller_Right`를 기준으로 0314를 맞춘 것이다. 직후 20초 별도 이동 측정에는 1,200쌍의 OpenVR 유효 포즈가 있었고, 앞 절반에서 추정한 장치 간 고정 관계를 뒤 절반에 적용했을 때 위치 RMS 19.18mm, 회전 RMS 1.78도였다. 이는 **그때 그 두 장치의 상대 움직임 검증**이며 전 장치·장기 안정성·절대 위치 정확도 인증은 아니다.

이후 장애 측정에서 VIVE 원본 좌표에 같은 프로필을 적용한 예측값과 실제 OpenVR 좌표를 비교했다.

| 가정 | 0314의 실제 OpenVR 위치와 비교한 중앙값 오차 |
| --- | --- |
| 보정 없이 원본을 표시 | 약 2.70m |
| 저장된 보정을 한 번 적용 | 약 0.006mm |
| 저장된 보정을 두 번 적용 | 약 2.70m |

한 번 적용한 값의 95백분위 차이는 약 0.26mm였다. 0086과 0320에서도 같은 한 번 적용 경로와 일치했다. 이 작은 숫자는 **코드가 같은 수식을 적용한다는 증거이지 VR의 실제 추적 정확도가 마이크로미터라는 뜻이 아니다**. 서로 다른 시점에 읽은 데이터에는 이동 중 지연 차이도 포함된다. [0회와 1회와 2회 적용 비교](runtime/diagnostics/20261014_vr_incident/transform_multiplicity_comparison.json)

이 비교에서 유효 샘플은 0314가 397개, 0086이 145개, 0320이 282개였다. 0082와 0072는 유효 비교가 0개여서, 코드의 공통 적용 구조와 별개로 이 두 장치의 실제 출력을 이 측정으로 검증하지는 못했다.

코드와 설정에서도 다음을 확인했다.

- [VUT bridge](twist2_vr/vut_openvr_bridge.py)는 원본 위치·회전을 전달하며 옛 `calibration.quest_vive.json`을 읽지 않는다.
- [OpenVR driver protocol](tools/vut_openvr/protocol.h)은 기본 world/head 변환이 identity이고 속도를 0으로 전달한다. 여기에서 속도 외삽으로 트래커를 날리는 경로는 없다.
- xi-ve는 `twist2_vut`에 속한 모든 장치에 같은 변환을 적용한다. 5개 모두 보정 전에 SteamVR에 등록돼 있었다.
- 현재 Standing과 Seated 기준 변환은 identity이다. Playspace Movement 활성화·이동 동작의 흔적도 없다.
- 물리 컨트롤러가 끊겼다고 이미 적용된 일반 보정이 자동으로 삭제되지는 않는다.

### 가장 중요한 단서는 보정 뒤의 기준 좌표 변경 가능성

22:38:02에 기존 map host 0086이 RF drop을 겪었고, 22:38:07에 0314가 새 host가 됐다. 이후 다른 트래커들이 맵을 다시 동기화했다. VIVE Linux는 한 트래커가 맵을 제공하고 다른 트래커들이 받는 구조다. [VIVE Hub 공식 구조와 변경 기록](https://github.com/ViveRoboticsSoftware/vivehub-linux/blob/main/CHANGELOG.md)

보정은 `Quest 공간의 위치 = R × VIVE 공간의 위치 + t`라는 관계이다. VIVE가 재인식하면서 원점이나 방향을 달리 잡으면 파일의 R과 t가 그대로여도 더 이상 현재 두 공간의 관계와 맞지 않는다. **네 개가 함께 같은 쪽으로 밀린 현상에는 이 설명이 부합한다.** 단, 호스트 교체가 반드시 원점을 바꾼다는 공식 보장은 없고 당시 전후 포즈가 없어 정확히 2m 이동을 입증한 것은 아니다.

대안은 Quest 런타임의 공간 원점 변경이다. ALVR의 위치·회전 recenter는 Disabled지만, [ALVR 20.14.1 OpenXR 처리](https://github.com/alvr-org/ALVR/blob/v20.14.1/alvr/client_openxr/src/lib.rs)와 [STAGE 갱신 코드](https://github.com/alvr-org/ALVR/blob/v20.14.1/alvr/client_openxr/src/stream.rs)는 런타임의 공간 변경 이벤트를 처리한다. ALVR의 추가 recenter를 끄는 것과 Quest 자체의 공간 재인식을 막는 것은 다르다. SteamVR chaperone 재기록만으로 실제 원점 변경을 확정하지는 않는다.

사용자가 본 약 10도와 2m를 수동 오프셋으로 바로 넣으면 안 된다. 유효한 장치 쌍으로 현재 관계를 측정해야 하며, 0072의 무효 좌표를 맞추려고 전체 공간을 움직여서도 안 된다.

### 공간 정렬 복구 방법

1. Quest 실제 컨트롤러가 자체 홈에서 안정되고 VIVE의 맵 동기화가 끝난 상태를 만든다.
2. map host의 시리얼을 기록하고 보정·검증 중 끄거나 교체하지 않는다. 나머지 장치도 사용할 구성으로 먼저 켠다.
3. 기존 보정을 유지한 채 오른손 컨트롤러와 0314를 함께 잡고, VR에서도 실제로 잡은 간격에 맞게 가까이 보이는지 확인한다. 병진 이동뿐 아니라 서로 다른 축으로 완만하게 회전할 때도 두 장치의 관계가 유지돼야 한다. 어긋날 때만 기존 프로필을 백업한 후 xi-ve 일반 보정을 다시 한다.
4. 보정 직후 제자리·앞뒤 이동·여러 축 회전에서 검증하고, 다른 트래커도 같은 물리 기준점으로 비교한다. 일정한 오프셋도 병진 이동만 하면 유지될 수 있으므로 단순히 같이 움직이는 것만으로 합격시키지 않는다. 장치 원점의 물리적 간격은 0이 아니므로 완전 중첩을 기준으로 삼지 않는다.
5. 재시작·host 교체·Quest 공간 변경 뒤에는 검증을 반복한다. 재발 시 그 사건 전후의 원본 포즈를 남겨 원점 변경 여부를 확정한다.

xi-ve의 일반 보정은 주기적인 재정렬이 필요할 수 있다. 자동 연속 보정은 **HMD에 고정한 트래커**가 필요한 별도 구성이다. 발등의 0314를 선택한 채 Continuous Calibration을 켜는 해결책은 아니다. 현재 5개를 모두 몸에 쓰므로 도입하려면 배치 변경 또는 추가 장치가 필요하다. [xi-ve 공식 사용법](https://github.com/xi-ve/openvr-space-calibrator-linux#calibration)

## Quest 컨트롤러의 연결 끊김

### Quest 자체 로그에서 확인한 내용

양쪽은 페어링 기록이 남아 있지만 연결이 반복해서 끊긴다. 따라서 ALVR binding을 바꾸거나 SteamVR를 재설치하는 것만으로 해결할 문제라고 보기 어렵다.

- 오른손은 약 1,547초 연결된 뒤 22:37:21에 끊겼고, 왼손은 약 1,663초 연결된 뒤 22:39에 끊겼다.
- 이후 재연결은 몇 초에서 수십 초 안에 다시 끊겼다. 두 장치 모두 연결 시 건전지 100%를 보고했다.
- 재연결 과정에서 `SPL version mismatch. Expected: 65.23.0, Actual: 65.100.11`, `IMU corrupt`가 기록됐다.
- 끊어진 뒤 양쪽이 `disconnected detached update-required`로 표시됐다.
- Quest는 OS branch `v207`, 컨트롤러는 `FW 207.5.0`으로 보고했다. `OVRRemoteService`에 해당 세션의 펌웨어 업데이트·언페어 기록이 없고 OTA 요청도 확인되지 않았다.

따라서 **확정된 것은 Quest 내부 컨트롤러 연결 장애이며, 펌웨어 업데이트 실패 자체는 아니다**. 정상 연결 당시의 SPL 열거 로그가 남아 있지 않아 이 버전 경고가 원래도 있었는지 비교할 수 없다. `IMU corrupt` 역시 로그의 표현이지 물리적 IMU 파손 판정이 아니다. 헤드셋 내부 무선·입력 서비스 상태, 펌웨어 호환성, 주변 무선 조건이 후보이다. [Quest 증거와 반대 증거](runtime/diagnostics/20261014_vr_incident/quest_controller_log_excerpt.txt)

조사 시 `/proc/uptime`은 약 162만 초, 즉 재부팅 후 약 18.8일이었다. 절전 시간도 포함하므로 18일 내내 사용했다는 뜻은 아니고, 장기 실행이 원인이라고 입증한 것도 아니다. 양쪽이 비슷한 시기에 끊기고 건전지 재삽입만으로 복구되지 않은 상황에서는 두 컨트롤러를 교체하기 전에 헤드셋 전체의 정상 재시작을 먼저 시험하는 것이 타당하다.

23:02:41 이후의 절전 전환과 23:08의 HMD 배터리 7%는 별도로 구분한다. 지금 벗어 둔 HMD가 자는 것은 자연스럽지만, **그 전에 이미 발생한 연결 끊김을 전부 설명하지는 못한다**.

### 두 시스템에서 비슷한 시간에 발생한 무선 단절

Quest 오른손이 22:37:21, 왼손이 22:39:15에 끊긴 근처 시각에 VIVE도 0320이 22:37:56, 0086이 22:38:02, 0082가 22:40:29에 RF drop을 기록했다. 같은 시간대 PC 커널 로그에는 USB disconnect/reset 기록이 없고 VIVE 동글도 연결 상태를 유지했다. **PC USB 전체가 끊겼다는 설명은 현재 로그로 지지되지 않는다.** [동시기 무선 단절과 커널 조회 기록](runtime/diagnostics/20261014_vr_incident/radio_cluster_and_host_checks.txt)

VIVE는 전용 2.4GHz 무선 연결을 사용한다. 무선 공존이나 주변 환경 문제는 비교할 만한 후보지만, Quest 컨트롤러는 PC 동글이 아니라 헤드셋에 연결되며 당시 Quest RSSI도 강하게 보고됐다. 비슷한 시각에 끊겼다는 이유만으로 VIVE가 Quest를 방해했다고 확정할 수는 없다. [HTC 무선 규격](https://business.vive.com/us/product/vive-ultimate-tracker/)

나중에 원인을 분리할 때 VIVE 동글·트래커를 잠시 비활성화한 조건에서 Quest 자체 홈을 확인하고, 다시 켠 조건과 비교한다. 동글은 연장 케이블로 PC에서 45cm 이상 떨어뜨리는 HTC 배치 권고를 적용할 수 있다. 이것은 무선 조건 비교이며 맵 초기화나 학교 공유기 설정 변경이 아니다. 트래커를 다시 켠 뒤에는 host와 공간 정렬을 다시 확인한다. [HTC 동글 배치 권고](https://blog.vive.com/us/how-to-set-up-vive-ultimate-tracker-for-third-party-steamvr-headsets/)

### OpenVR의 초록불만 믿으면 안 되는 이유

ALVR 20.14.1은 일부 경우 OpenXR 위치가 유효하지 않으면 마지막 컨트롤러 위치를 유지하면서 motion 객체를 전달한다. 서버는 이 객체가 존재하면 OpenVR의 connected·valid를 true로 만들 수 있다. 따라서 등록된 컨트롤러, 멈춘 포즈, OpenVR valid가 실제 Touch의 정상 동작을 보장하지 않는다. [ALVR 클라이언트 포즈 처리](https://github.com/alvr-org/ALVR/blob/v20.14.1/alvr/client_openxr/src/interaction.rs#L754-L775), [ALVR 서버 유효성 처리](https://github.com/alvr-org/ALVR/blob/v20.14.1/alvr/server_openvr/cpp/alvr_server/Controller.cpp#L183-L205)

현재 OpenVR에는 실제 Touch와 `VRLINKQ_Hand_Left/Right` 합성 손이 함께 등록돼 있다. 23:09에는 실제 Touch는 disconnected였지만 합성 손은 valid로 남아 있었다. 합성 손을 실제 컨트롤러로 착각하거나 캘리브레이션 기준으로 고르면 안 된다. [23시 09분 장치 상태](runtime/diagnostics/20261014_vr_incident/live_2309_snapshot.json)

### 컨트롤러 복구 순서

1. HMD를 충분히 충전하고 정상 재시작한다. 건전지만 뺐다 넣는 것은 헤드셋 내부 무선·입력 서비스를 재시작하지 않는다.
2. ALVR에 들어가기 전에 **Quest 자체 홈**에서 양쪽 위치 추적과 버튼이 안정되는지 확인한다. 이때 HMD를 착용하고 컨트롤러를 추적 가능한 위치에 둔다.
3. 자체 홈에서도 실패하면 헤드셋의 공식 소프트웨어 업데이트 상태와 컨트롤러 업데이트 안내를 확인한다. 업데이트가 실제 진행 중이면 완료 전에 전원을 반복해서 끄지 않는다.
4. 재시작·업데이트 확인 후에도 실패하면 커버나 액세서리의 간섭과 건전지 접촉을 확인한 뒤 Meta Horizon 앱에서 문제 컨트롤러를 한쪽씩 재페어링한다. 먼저 양쪽의 기존 상태와 버전을 기록한다. 앱에서 기기 → 해당 헤드셋 → 헤드셋 설정 → 컨트롤러로 들어가며, 페어링 버튼은 오른쪽 `Meta+B`, 왼쪽 `Menu+Y`이다. [Meta 지원 계정의 페어링 안내](https://communityforums.atmeta.com/discussions/PairingConnection/3s-vr-right-controller-not-visible/1354415/replies/1354426), [기본 점검 안내](https://communityforums.atmeta.com/discussions/PairingConnection/neither-controller-will-turn-on-for-quest-3/1138269/)
5. 자체 홈에서는 정상인데 ALVR에서만 실패하면 그때 ALVR 세션, 실제 컨트롤러 선택, 입력 포커스를 조사한다. 이번 로그의 Quest 내부 끊김과는 구분한다.
6. 같은 끊김이 계속되면 보존한 로그와 OS·FW 버전으로 Meta 지원에 문의하고 공식 소프트웨어 복구 절차를 확인한다. 출처 불명 펌웨어 다운그레이드·ADB 강제 업데이트·공장 초기화는 첫 조치가 아니다.

일반 복구와 무선 조건 비교가 모두 실패했을 때의 추가 선택지는 **컨트롤러만 초기화**하는 것이다. Meta 지원 계정은 Quest 3용으로 오른쪽은 `A`를 누른 채 `Meta`를 5회, 왼쪽은 `X`를 누른 채 `Menu`를 5회 누르는 절차를 안내한다. 주황색·흰색 점멸 뒤 부팅하며 필요하면 다시 페어링한다. 이는 헤드셋 전체 데이터 초기화와 다른 작업이다. 현 상태에서는 수행하지 않았고, Touch Pro용 버튼 조합을 혼용하지 않는다. [Meta 지원 계정의 Quest 3 컨트롤러 초기화 안내](https://communityforums.atmeta.com/discussions/PairingConnection/quest-3-controller-pairs-but-not-working-at-all/1107964/replies/1171341)

## 다음 방문 때 한 번에 확인할 항목

**추가 답변을 하나씩 기다리는 대신 다음 세 묶음으로 진행한다.**

1. **장치 자체 비교:** Quest 정상 재시작 후 자체 홈에서 양손을 확인하고, VIVE는 0072와 0314의 위치·높이 교차 시험을 한다. 이 단계에는 로봇이나 공간 재보정이 필요 없다.
2. **공간 비교:** 입력이 안정되면 HMD 착용 상태에서 실제 오른손과 0314를 같은 손에 고정해 원래 위치·앞뒤 이동을 한 번의 기록으로 확인한다. 모든 VIVE 장치를 켜고 host가 유지되는지도 같이 기록한다. 필요할 때만 재보정한다.
3. **착용 비교:** 다섯 개를 원래 위치에 붙이고 중립 자세와 작은 움직임을 한 번에 기록한다. Ready율, 위치 점프, 좌우 대응, 몸에서의 상대 위치를 함께 확인한다. 이때도 로봇 출력은 하지 않는다.

실행 상태를 읽는 기존 명령은 다음과 같다. HMD와 장치를 켜고 올바른 SteamVR와 VIVE Hub가 실행 중일 때 사용한다. 1회 inventory는 장시간 안정성 검사를 대신하지 않는다.

```bash
cd /home/railab/Desktop/vr_teleop/TWIST2
bash scripts/run_steamvr_pinned.sh --check
bash scripts/run_vive_linux_inventory.sh
bash scripts/run_live_inventory.sh --mapping config/local/tracker_mapping.vive_linux.json
```

공간 정렬 세션이 꺼져 있을 때만 `bash scripts/run_vive_spacecal.sh`로 기존 UI와 bridge를 연다. 이미 실행 중인 세션을 중복으로 띄우거나, Steam 라이브러리의 다른 SteamVR 버전을 실행하지 않는다.

공간이 맞은 뒤에도 전신 teleop 준비가 모두 끝난 것은 아니다. 현재 `coordinates.vive_linux_fullbody.json`은 `mounts_calibrated=false`이고 장치에서 관절 중심까지의 변환은 placeholder이다. `run_vive_spacecal_fullbody_dry.sh`가 요구하는 `config/local/calibration.vive_linux.json`도 아직 없다. 이 파일을 임의의 0으로 만들어 검사를 통과시키거나 이전 직접 SDK 보정을 보정된 OpenVR 포즈에 다시 적용하지 않는다.

## 보존한 증거와 소프트웨어 검사

증거 폴더는 [runtime/diagnostics/20261014_vr_incident](runtime/diagnostics/20261014_vr_incident)이다.

- [VIVE 원본 로그 발췌와 상태 시간 계산](runtime/diagnostics/20261014_vr_incident/vive_tracker_log_excerpt.txt)
- [Quest 연결 끊김과 버전 경고 및 절전 로그](runtime/diagnostics/20261014_vr_incident/quest_controller_log_excerpt.txt)
- [성공했던 22시 31분 장치 쌍 검증](runtime/diagnostics/20261014_vr_incident/spacecal_2231_pair_validation.json)
- [보정 프로필 수치](runtime/diagnostics/20261014_vr_incident/spacecal_verified_profile.json)
- [장애 후 8초 VIVE 측정 요약](runtime/diagnostics/20261014_vr_incident/vut_8s_capture_summary.json)
- [8초 측정의 포즈 샘플](runtime/diagnostics/20261014_vr_incident/vut_8s_capture_samples.json)
- [원본과 OpenVR 사이의 보정 일치 비교](runtime/diagnostics/20261014_vr_incident/vut_8s_profile_agreement.json)
- [조사 중 23시 09분 장치 스냅샷](runtime/diagnostics/20261014_vr_incident/live_2309_snapshot.json)

VIVE 포즈 상태 처리, VUT bridge 패킷, OpenVR 행렬, 실제 컨트롤러 선택 관련 기존 단위 테스트 54개가 통과했다. 모의 데이터 기반 테스트이므로 하드웨어 정상이나 공간 정렬 정확도를 대신 증명하지는 않는다.

```bash
PYTHONDONTWRITEBYTECODE=1 .conda-vr/bin/python -m pytest -q -p no:cacheprovider \
  tests/vr_adapter/test_vive_linux.py tests/vr_adapter/test_vut_openvr_bridge.py \
  tests/vr_adapter/test_openvr_matrix.py tests/vr_adapter/test_openvr_actions.py
```

이번 조사에서 확인하지 못한 물리적 원인을 해결됐다고 처리하지 않는다. 0072의 동일 조건 비교, Quest 재시작 후 자체 홈 검증, host 변경 전후 원점 검증이 남아 있다. 이 세 결과에 따라 위 복구 분기를 선택하면 된다.
