# 실행용 모델 묶음

| 파일 | 용도 | 검증 |
|---|---|---|
| [g1_stationary_v6_20260930.tar.gz](g1_stationary_v6_20260930.tar.gz) | 기본 제자리 머리·양손 추종 정책, 24.15 MiB | 별도 192에피소드 낙상0, 실제 UDP/9축/정지·리셋, 새 Conda·독립 Lab·압축 해제 폴더에서 재평가 |

[실행 명령](../docs/g1/RUN_SERVER.md), [정량 결과와 한계](../docs/g1/RESULTS.md),
[해시·모델 정보](manifest.json)를 먼저 확인한다. 제자리 모델은 이동 명령을
0으로 제한한다. 실제 Quest 연결과 ALVR 영상은 현장에서 확인해야 한다.

```bash
# 저장소 최상위에서
python3 scripts/g1/bundle.py verify artifacts/g1_stationary_v6_20260930.tar.gz
python3 scripts/g1/bundle.py unpack artifacts/g1_stationary_v6_20260930.tar.gz \
  --destination "$HOME/g1_teleop_run"
```

모델 위치는 압축 해제 후 `models/teleop_head_precision_v6/`다.
`model.pt`는 optimizer를 포함한 재개 체크포인트, `policy.pt`는 실행용 TorchScript다.
둘의 관측/액션 계약과 실제 actor 파라미터 일치는 실행 시 검사한다.
원본 학습 설정·소스 스냅샷·전체 scalar 로그 JSON·평가 기록·데이터·G1 USD와
출처 고지문도 포함한다. 라이선스 범위는 동봉한 출처 문서를 따른다.

이 아카이브는 실제 검증한 런타임의 불변 스냅샷이다. 이후 저장소의 실험 코드나
결과 보고서가 갱신되더라도 기존 아카이브는 교체하지 않는다. 최신 전체 실험
결과는 저장소의 `docs/g1/RESULTS.md`를 기준으로 확인한다.
