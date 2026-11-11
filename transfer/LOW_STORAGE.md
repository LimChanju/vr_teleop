# Drive 5GB 경량 이전 — 2026-11-11

이 프로필은 기존 28GB 보존용 묶음 대신 Drive 약5GB에 올릴 자료다. 기존 원본과 전체 보존본은 삭제하지 않았다. GitHub의 4.5 실행 코드는 수정하지 않았다.

## 업로드 파일

| 파일 | 크기 |
|---|---:|
| `01-data-models-calibration.tar.zst` | 1.867 GB |
| `02-pinned-vr-runtime.tar.zst` | 2.771 GB |
| `03-restore-info.tar.gz` | 환경 export·목록·검증·복원 도구(소용량) |

압축 파일 3개와 `SHA256SUMS`만 올린다. 01·02 합계는 4,637,638,668bytes(4.638GB)이며, metadata를 포함한 실제 총량은 READY.json에 기록한다. 큰 28GB용 압축 파일 4개는 이 프로필에 추가로 올리지 않는다.

## 유지한 것

- Quest/VIVE 원시 입력, body/G1/physics 기록, 영상·진단·캘리브레이션과 tracker mapping.
- TWIST2 ONNX, SparsePoser 가중치, G1 XML/mesh, Unitree manipulation 자산, 외부 라이브러리의 실제 작업 파일.
- **고정 SteamVR2.12.14 전체**(tools 포함), ALVR20.14.1 서버·보관 APK·wrapper, VIVEHub1.0.3.
- 현재 Steam Scout/Sniper의 설치된 platform과 entrypoint, SpaceCal/VUT driver 및 호스트 VR 설정.
- 기존 이전 payload의 수집/모델/보정 중요 파일129개를 대조해 누락0개를 확인한다.

## 용량을 줄이려고 제외한 것

- Isaac4.5 설치 전체와 Conda/venv 바이너리. 새 서버에서 **네트워크로 설치/환경 재생성**해야 한다.
- Git 객체/이력, Python cache, 재생성할 build cache 및 Steam pressure-vessel 임시 var.
- SteamVR의 depot 다운로드 사본·교체 전 백업·Steam 관리2.16.7. 실제 사용하는 고정2.12.14는 보존한다.
- 사용하지 않는 최신 OpenVR SDK. 호환 v2.12.14의 headers·link libraries는 보존한다.
- GMR의 G1 이외 robot 자산과 OpenVR SDK example binary/media. 필요한 경우 고정 upstream에서 다시 가져온다.

빠진 항목과 이유는 inventory/excluded.json에 있다. metadata에 있는 환경 export는 실제 설치 목록이며 모든 과거 환경의 일괄 자동 재설치 성공을 보증하는 lockfile은 아니다. 새 서버에서 Python/prefix/editable 설치를 맞춰야 한다.

## 새 서버 복원 순서

1. GitHub branch `migration/vr-teleop-transfer-20261111`을 clone한다.
2. 별도 이전 폴더의 `archives/`에 압축 파일3개를 두고 SHA256SUMS를 상위에 둔다.
3. 그 상위 폴더에서 아래를 실행한다. Python3.10+와 zstd가 필요하다.

```bash
sha256sum -c SHA256SUMS
tar -xzf archives/03-restore-info.tar.gz
sha256sum -c METADATA_SHA256SUMS
sha256sum -c PAYLOAD_SHA256SUMS
python3 verify_migration.py .
mkdir -p restore-staging
tar --use-compress-program=unzstd -xpf archives/01-data-models-calibration.tar.zst -C restore-staging
tar --use-compress-program=unzstd -xpf archives/02-pinned-vr-runtime.tar.zst -C restore-staging
```

4. archive는 기존 경로의 선행 /를 뺀 `home/railab/...` 구조다. staging의 프로젝트 파일을 새 GitHub clone으로 옮기고 호스트 VR 설정은 새 사용자/설치 prefix에 맞춰 적용한다. 새 clone의 .git를 덮어쓰지 않는다.
5. 외부 dependency의 working tree에는 용량을 줄이기 위해 .git가 없다. Git 관련 setup 스크립트 전에 아래로 고정 Git metadata와 누락된 upstream 파일을 복원한다. 인터넷을 사용하며 기존 작업 파일을 덮어쓰지 않는다. 14개 repository와 nested module의 pin을 기록했다.

```bash
python3 restore/restore_dependency_git.py --workspace /새경로/vr_teleop --restore-git
```

이 도구는 공개 upstream에서 고정 commit을 fetch하고 **없는 tracked 파일만** 채운다. 이미 다른 HEAD/origin이 있는 경로는 변경하지 않고 중단/보고한다. 기본값(`--restore-git` 생략)은 읽기 전용 검사다. 실제 old host에서는 읽기 전용 확인만 수행했고, 쓰기 동작은 임시 fixture에서 기존 파일·사용자 데이터를 보존하는지 검증했다.

6. **새 서버 환경은 [합의된 분리 기준](NEW_SERVER_ENVIRONMENT.md)을 먼저 적용한다.** simulation/RL은 Conda 계열 또는 Pixi, VR/센서와 실제 로봇 I/O는 각각 독립 venv로 구성한다. 환경 export는 과거 dependency를 확인하는 자료이며 과거 `.venv-sparseposer`의 Conda 상속을 새 서버에서 그대로 재생성하지 않는다. 현재 setup/실행기의 결합 부분은 이전 후 별도 변경이 필요하다. `_isaac_sim` 링크·editable install·build cache 등의 prefix도 새 설치 위치에 맞춘다. 원본4.5 동작 재현과6.2 이식은 구분한다.
7. ALVR/Steam client·시스템 GPU/udev 권한을 준비하고, 보존한 Scout/Sniper·고정 SteamVR와 driver/wrapper 경로를 맞춘다. `openvrpaths.vrpath`와 OpenXR active_runtime symlink는 옛 절대경로다. 특히 active_runtime은 제외한 Steam 관리본을 가리킬 수 있으므로 고정2.12.14의 `steamxr_linux64.json`과 XR_RUNTIME_JSON을 확인한다.
8. ALVR 첫 영상과 양손 입력 확인 뒤 VIVE bridge를 시작한다. 새 방/장착에서는 SpaceCal·T-pose를 재검증한다. 기존 보정의 보존이 새 공간에서의 유효성을 보장하지는 않는다.

6.2를 사용하려면 Python/XR/API 이식은 별도 단계다. 이 경량 묶음에서 설치본을 뺐다고 4.5 소스가 6.2에서 수정 없이 실행되는 것은 아니다.

현재 이 컴퓨터의 원본 코드·환경·기존 전체 백업은 그대로 남아 있다. 새 서버의 설치/네트워크/하드웨어 검증은 복원 뒤 수행한다.

최종 업로드 합계는 체크섬 포함 **4,640,207,919 bytes(4.640GB / 4.322GiB)**다. 두 payload 내부 내용 및 metadata78파일의 hash/권한/링크 검사를 통과했다.
