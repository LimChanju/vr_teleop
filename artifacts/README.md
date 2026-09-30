# 실행용 모델 묶음

| 파일 | 용도 | 검증 |
|---|---|---|
| [g1_stationary_v6_r2_20260930.tar.gz](g1_stationary_v6_r2_20260930.tar.gz) | 현재 기본 제자리 머리·양손 추종 정책 | 별도 192에피소드 낙상0, 실제 UDP/9축/정지·리셋, 새 Conda·독립 Lab·새 압축 해제 폴더에서 같은 평가 수치 재현 |
| [g1_stationary_v6_20260930.tar.gz](g1_stationary_v6_20260930.tar.gz) | 이전 불변 스냅샷 보관 | 모델 가중치는 r2와 같음; 새 실행에는 r2 사용 |
| [g1_velocity_v7_20260930.tar.gz](g1_velocity_v7_20260930.tar.gz) | 이동 실험 비교 기준 | Cartesian+이동 평가 0/72 낙상; UDP 회전 편향·상체 추종 미달 |
| [g1_wholebody_v9_20260930.tar.gz](g1_wholebody_v9_20260930.tar.gz) | 평행 이동 응답을 강화한 실험 | Cartesian+이동 평가 6/75 낙상; 110초 UDP 평행 이동 응답 통과·회전과 상체 추종 미달 |

[실행 명령](../docs/g1/RUN_SERVER.md), [정량 결과와 한계](../docs/g1/RESULTS.md),
[해시·모델 정보](manifest.json)를 먼저 확인한다. 제자리 모델은 이동 명령을
0으로 제한한다. 실제 Quest 연결과 ALVR 영상은 현장에서 확인해야 한다.
v7/v9의 서버 실행·평가·재개는 [이동 실험 안내](../docs/g1/RUN_MOVING_EXPERIMENT.md)를
따른다. 실험 묶음에는 원래 학습 데이터와 별도 비교 평가 데이터도 포함한다.

```bash
# 저장소 최상위에서
python3 scripts/g1/bundle.py verify artifacts/g1_stationary_v6_r2_20260930.tar.gz
python3 scripts/g1/bundle.py unpack artifacts/g1_stationary_v6_r2_20260930.tar.gz \
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

r2는 같은 모델에 자동 통합 검증 도구와 정확한 그립 클러치 안내를 추가했다.
기본 정책 가중치의 SHA256은 동일하며, r2를 별도 경로에 풀어 수행한 192개
에피소드의 7개 집계 지표도 최초 평가와 정확히 일치했다.
[경로·해시 확인](../docs/g1/verification/portable_model_v6_r2_paths.json),
[평가 비교](../docs/g1/verification/portable_result_comparison_r2.json).

v7·v9도 각각 새 경로에 압축을 풀고 독립 Isaac Lab과 새 Conda 환경으로
18환경×4500step Cartesian+속도 평가를 반복했다. 각 모델의 낙상·오차·속도
응답 10개 지표가 원래 평가와 정확히 일치했다. 부족한 성능까지 재현한 것이며,
실험 정책의 품질 기준 통과나 실제 공용 서버 검증을 뜻하지 않는다.
[v7 경로·결과 확인](../docs/g1/verification/portable_velocity_v7_paths.json),
[v9 경로·결과 확인](../docs/g1/verification/portable_wholebody_v9_paths.json).
