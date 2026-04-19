# CW 탐지 Phase 1 — 분포 분리성 실측

일자: 2026-04-19  
매크로: `HTS_CW_DETECT_DIAG` (계측만, FEC 복조 경로 변경 없음)

## Discovery (요약)

### DISC-CW1 — Row / FWHT hook

- `HTS_FEC_HARQ.cpp` `Decode64_IR` conv 경로: `fec_ir_fwht_bin_unshift` 직후 `fec_row_consistency_record_from_fwht(fI,fQ,nc,bps)` (기존 `HTS_ROW_CONSISTENCY_DIAG`).
- Polar 분기: 동일하게 `g_fec_dec_fI/Q` + `nc` 에 대해 unshift 직후 기록.
- `Decode16_IR`: `fI`/`fQ` unshift 직후 동일.
- CW Phase1 계측은 **동일 시점**에 `CWDetectDiag::record_symbol_from_fwht(fi,fq,nc)` 호출: `nc`개 bin에 대해 \(I^2+Q^2\) 에너지, 나머지 64−`nc`는 0으로 두고 정렬·엔트로피·top4 비율 계산.

### DISC-CW2 — AJC / excision (문헌 위치만)

- 실제 CW 제거·ECCM 처리는 `HTS_V400_Dispatcher.cpp` IR 경로 주석의 “Fix A3 … excision / ECCM” 블록 등에서 수행 (본 Phase1에서는 호출 추가 없음).

### DISC-CW3 — Per-symbol 에너지

- FEC IR 루프의 `fI[r]`, `fQ[r]` (또는 polar 경로 `g_fec_dec_fI`)가 FWHT+unshift 후 **행(bin) 에너지**에 해당.

## 빌드

- 스크립트: `HTS_TEST/t6_sim/build_and_test_cw_detect.bat`
- 결과: **PASS**

## 회귀

- `HTS_T6_CW_DETECT.exe` stdout: **정량 합계 10927 / 11200 (97.6%)** — 기존 T6 baseline 유지

## 시나리오 라벨

- **non-CW 기대** (`set_expect_non_cw(true)`): S1, S3, S7 — 패킷 집계는 `pkt_total_clean` 쪽.
- **CW-heavy 기대** (`set_expect_non_cw(false)`): S8, S9 — `pkt_total_cw`.

## 시나리오별 핵심 분포 (stderr, `cw_detect_stderr.log`)

히스토그램 bin 의미: per-sym / win8 concentration 은 20구간으로 \([0,1)\) 근사 균등 분할; win8 entropy 는 0.25 bit 간격 20 bin (0~5 bit 스케일).

### S1 Clean — Baseline

- `total_symbols=6880`, `pkt_clean=20`
- Per-sym top2/top1: peak bin 0 (비율 0.00~0.05 구간)
- Per-sym concentration: peak bin 19 (~0.95–1.00), p95 bin 19
- Window(8) concentration: peak bin 19, p95 bin 19
- Window entropy: peak bin 0 (~0.00–0.25 bit), p05 bin 0
- 패킷 평균 concentration 임계 (non-CW false alarm 후보): `conc_mean>0.5` … `>0.95` 모두 **100%** (20/20 패킷)

### S3 저 SNR

- `pkt_clean=155`
- Per-sym concentration: peak bin 11 (~0.55–0.60), p95 bin 18 (~0.90–0.95)
- Win8 entropy: peak bin 19 (~4.75–5.00 bit), p05 bin 3 (~0.75–1.00 bit)
- `conc_mean>0.7`: non-CW **25.806%** / `conc_mean>0.9`: **12.903%**
- `ent_mean<2.0`: non-CW **12.903%**

### S7 Barrage

- `pkt_clean=116`
- Per-sym concentration: peak bin 11, p95 bin 13 (~0.65–0.70)
- Win8 entropy: peak bin 13 (~3.25–3.50 bit), p05 동일
- `conc_mean>0.5`: non-CW **52.586%** / `>0.7`: **0%**

### S8 CW (목표)

- `pkt_cw=60`
- Per-sym concentration: peak bin 18 (~0.90–0.95), p95 bin 19
- Win8 concentration: peak bin 18, p95 bin 19
- Win8 entropy: peak bin 7 (~1.75–2.00 bit), p05 bin 6 (~1.50–1.75 bit)
- `conc_mean>0.5` … `>0.9`: CW **100%** / `>0.95`: **33.333%**
- `ent_mean<2.0`: CW **100%**

### S9 복합 (CW 라벨)

- `pkt_cw=16` (디코드가 FEC까지 도달한 trial만 패킷 집계)
- 분포는 S7에 가깝게 나옴 (barrage/ISI 등으로 행 에너지가 CW 단독보다 분산)
- `conc_mean>0.5`: CW **81.25%** / 그 외 고 임계는 0%

## ROC 분석 (예비, 패킷 평균 기준)

| 규칙 | non-CW FP (참고 시나리오) | CW TP (S8) | 비고 |
|------|----------------------------|------------|------|
| `conc_mean>0.7` | S3: 25.8% | 100% | S1은 conc만으로 전부 걸림 |
| `conc_mean>0.8` | S3: 25.8% | 100% | 동일 |
| `conc_mean>0.9` | S3: 12.9% | 100% | |
| `conc_mean>0.95` | S3: 0% | 33.3% | TP 급락 |
| `ent_mean<2.0` | S1: 100% | 100% | 단독 사용 불가 |
| `conc_mean>0.9` AND `ent_mean<2.0` | S1·S3 교집합 필요 (Phase2에서 정량) | S8: 100% (둘 다 만족) | 복합 후보 |

**Phase 1 결론 (임시):** 단일 지표 `conc_mean`만으로는 S1 클린에서 FP가 과도하고, `ent_mean<2` 단독은 S1·S8 구분 불가. **복합(예: 고 conc + 저 ent)** 이후보다 **S7/S9와의 분리**를 Phase2에서 추가 정량할 것.

## 판정

### B. 중첩 있음 (단일 지표 한계)

- 클린 S1에서도 **per-sym / win8 concentration** 이 극단 bin에 강하게 몰릴 수 있어, concentration 단독으로는 “CW만의 지문”으로 보기 어렵다 (Q1: 클린에서도 높은 concentration 가능 — 실측 확인).
- S3·S7과 S8은 **win8 entropy** 쪽에서 차이가 더 분명하다 (S8 저엔트로피 피크 vs S7 중간 피크).
- 완벽한 하드 임계 한 개로 분리하기는 어렵고, **soft metric·복합·또는 LLR 등 추가 축**이 Phase2 이후에 필요할 가능성이 크다.

## 다음 단계

- Phase2: 패킷 단위 joint 분포( `conc_mean` × `ent_mean` ) 히트맵, S2/S4/S5 등 추가 non-CW 클래스, sliding window 길이(8) 민감도.
- Phase3 이후: 실제 excision/AJC 쪽은 `HTS_V400_Dispatcher` Fix A3 경로와의 정합 검토.

## 자진 고지

- Sliding window 길이 **8 심볼** 고정 (코드 `CWDetectStats::WINDOW_SIZE`).
- 임계값 배열: `conc` 0.5/0.7/0.8/0.9/0.95, `entropy` 2.0/1.5/1.0/0.8/0.5 (패킷 **평균** 심볼 엔트로피에 적용).
- `mark_packet_boundary`는 T6 `feed_raw_ext` **매 trial 종료** 시 호출; 디코드가 FEC 기록 없이 끝나면 해당 trial은 패킷 집계에서 제외될 수 있음 (`pkt_sym_n==0` 스킵).
- S9는 스트레스 혼합으로 **S8 순수 CW**보다 분포가 덜 극단적일 수 있음 — 라벨은 “CW 시나리오”이나 물리적으로는 broadband·ISI 성분이 크다.
