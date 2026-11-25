# Hand TEPI 시스템 검증 실험 스크립트 (E0–E7)

기존 제어 코드 `1015_v9_glove_cal.py` 를 **수정하지 않고** 불러와서 실험을 돌린다.
같은 폴더에 제어 코드와 `tendon_maps/G1.json`, `G2.json` (캘리브레이션 결과)을 두고 실행.

```
hand_tepi_validation/
  1015_v9_glove_cal.py    ← 기존 제어 코드 (그대로 복사)
  tendon_maps/G1.json, G2.json
  params.json             ← params.example.json 을 복사해 UI 에서 튜닝한 값 기입
  exp_common.py           공통: 제어 스레드 확장, 0점, 로그, 가이드 창, 가상 사람 서보, 시뮬레이터
  analysis_common.py      공통 분석 함수
  e0_characterize.py      E0 기초 특성화 (tension / friction / resolution / merge)
  e1_stability.py         E1 안정성 맵
  e2_stiffness.py         E2 렌더링 강성
  e3_free_motion.py       E3 자유 운동 투명성
  e4_resistance.py        E4 저항 전달 (양방향 vs 단방향)
  e5_symmetry.py          E5 대칭성 (분석 전용)
  e6_frequency.py         E6 주파수 응답
  e7_perception.py        E7 지각 역치
```

## 먼저: 하드웨어 없이 시험 (`--sim`)

모든 스크립트는 `--sim --no-gui` 로 가상 손가락 위에서 끝까지 돌아간다. 처음엔 이걸로 흐름과 출력 파일을 확인.
시뮬레이터 숫자는 의미가 없다 (로직 확인용).

```
python e1_stability.py --sim --no-gui --K 0.05 0.1 --B 0.002 --duration 4
```

## 실행 순서

| 순서 | 명령 | 필요한 것 | 사람 |
|---|---|---|---|
| 1 | `python e0_characterize.py tension --glove G1 --finger I --tendon FLEX --mass 0.1 0.2 0.5` | 추, 짧은 케이블 | 실험자 |
| 1' | (모터마다 반복) → `python e0_characterize.py merge results/e0_tension_*/tension_cal.json -o tension_cal_all.json` | | |
| 2 | `python e0_characterize.py friction --glove G1 --finger I` (G2 도) | 더미 손가락 | 실험자 |
| 3 | `python e1_stability.py --finger I --params params.json --K 0.02 0.05 0.1 0.2 --B 0 0.001 0.003` | 고정 블록 | 2명 |
| 4 | `python e2_stiffness.py --finger I --params params.json --K 0.05 0.1 0.2 --cal tension_cal_all.json` | 더미+고정 블록 | 치료사 역할 1명 |
| 5 | `python e3_free_motion.py --finger I --params params.json --K 0.1 --B 0.002 --auto` | 더미(자유) | 1명 |
| 6 | `python e4_resistance.py --finger I --params params.json --K 0.1 --loads free=0 soft=1 medium=2 stiff=4 clamp=inf` | 스프링 3종+고정 | 1명 |
| 7 | `python e5_symmetry.py results/e2_xxx results/e4_xxx --cal tension_cal_all.json --friction results/e0_friction_*/friction.json` | (로그만) | – |
| 8 | `python e6_frequency.py --finger I --params params.json --K 0.1 --B 0.002` | 더미 2개 | 실험자 |
| 9 | `python e0_characterize.py resolution --glove G1 --finger I --log results/e3_xxx --vision vision.csv --vision-col <각도열>` | 비전 기록 | – |
| 10 | `python e7_perception.py --finger I --params params.json --deg-per-curl <9번 결과> --K 0.1` | 더미, 눈가림, 이어폰 | 참가자 1명 |

E1 결과의 안정 범위 안에서 E2–E7 의 K, B 를 고른다.
모든 스크립트는 `--analyze-only <결과폴더>` 로 분석만 다시 돌릴 수 있다.

## 결과 파일

`results/<실험>_<시각>/`
- `log.csv` — 제어 루프 매 틱 (≈125 Hz): curl, 엔코더, 지령/적용 장력, 결합력, 라벨, 이벤트
- `meta.json` — 인자, 파라미터, 지도 스팬, 제어 주기
- `summary.md` / `summary.json` — 표로 정리된 결과 (보고서에 옮길 값)
- `*.png` — 그림

## 단위와 주의

- **장력은 전부 모터단 추정값** (`전류 × Kt / r`, Kt = 0.0156 Nm/A, r = 3.8 mm). 손끝 힘 아님.
  E0 보정 파일(`--cal`)을 주면 추 기준으로 보정된 값을 쓴다.
- **K 단위는 N / 스풀 deg** (코드의 KC). `× 지도 스팬[deg]` → N/curl, `÷ 0.0663` → N/mm(케이블).
- 같은 KC 라도 두 장갑의 지도 스팬이 다르면 받는 힘이 다르다 → E5 가 보정 KC2 를 제안.
- 텐던 탄성(5 N 에서 ~150°)이 엔코더 오차에 섞이므로, K 를 키울수록 실제 강성은 설정값보다 작아진다
  (대략 K_eff ≈ K / (1 + K / k_stretch)). E2 가 이걸 재는 실험.
- 원래 코드의 장력 램프 제한(RAMP_UP 60 N/s, RAMP_DOWN 12 N/s)이 빠른 움직임에서 결합력을 늦춘다 → E6 에서 확인.

## 안전

- 가이드 창에서 **Space / Esc = 비상정지**, 창이 없으면 **Ctrl+C**.
- E1 은 결합오차 고주파 RMS 가 `--thr` 를 넘으면 자동으로 결합을 끄고, 같은 B 에서 더 큰 K 는 건너뛴다.
- 가상 사람 서보(E6, E7)의 상한은 `--servo-tmax` (기본 6 N). 게인은 작게 시작.
- IRB 장력 상한 20 N. 원래 코드의 지령 상한(≈16.9 N)과 결합력 상한(15 N)이 그대로 적용된다.
- DAQ 는 한 번에 한 프로그램만 — 원래 UI 를 끄고 실행할 것.
- E7 등 사람 참가자 실험은 현재 IRB 승인 범위에 들어가는지 먼저 확인.
