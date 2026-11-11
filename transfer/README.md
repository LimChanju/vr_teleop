# vr_teleop 이전 분리

연결 대상: https://github.com/LimChanju/vr_teleop.git

GitHub에는 alvr/browser와 TWIST2의 코드·설정 예제·요구사항·테스트·라이선스 및 이 문서를 포함한다. TWIST2의 로컬 변경/미추적 코드는 일반 디렉터리로 포함했으며 embedded Git repository로 추가하지 않았다. upstream source commit과 모델 provenance는 dependency-lock.json / upstream-provenance.json을 참고한다.

비공개 Drive 자료는 vendor, TWIST2 external/assets/runtime, 실제 config/local 보정, venv/Conda binary, ALVR·SteamVR·HTC·SpaceCal 설정, 인증서와 로그를 포함한다. 공통 VR runtime과 Isaac4.5/관련 Conda도 별도 archive로 제공한다. clone만으로 전체 실행을 복구할 수 없으며 Drive 자료를 함께 복원해야 한다.

Drive archive는 원본 절대경로에서 선행 /를 제거한 경로를 보존한다. staging 디렉터리로 먼저 추출한 뒤 새 repo 및 설치 위치로 옮긴다. GitHub repo의 .git를 덮어쓰지 않는다. 새 경로에서는 symlink/OpenVR driver 등록/Conda editable install/IsaacLab _isaac_sim 링크/activation hook을 점검한다.

보존 VR 기준: ALVR20.14.1, 고정 SteamVR2.12.14, OpenVR SDKv2.12.14. Steam 관리본2.16.7과 구분한다.

[환경 공유 구조](ENVIRONMENT.md)를 먼저 읽는다. 실제 캘리브레이션과 신뢰 장치/개인키는 GitHub에 포함하지 않았다.

기존 HRC 및 별도 g1_teleop_run 프로젝트는 이 vr_teleop 전용 분리본의 범위에 포함하지 않는다.

Drive 업로드 파일: `01-vr-project-data-assets-env.tar.zst`, `02-vr-runtime-and-settings.tar.zst`, `03-isaac45-related-conda.tar.zst`, `04-environment-and-verification.tar.gz` 및 `SHA256SUMS`. 네 번째 파일에는 환경 export·파일 목록·Git 이력·검증/복원 도구가 있다. 새 서버에서 네 번째 파일을 먼저 풀어 안내를 읽는다.

Drive 여유 공간이 약5GB이면 기존 보존용 압축본 대신 [5GB 경량 프로필](LOW_STORAGE.md)을 사용한다. 데이터·모델·보정과 고정 VR runtime은 유지하고 Isaac/Python 환경은 새 서버에서 설치한다.
