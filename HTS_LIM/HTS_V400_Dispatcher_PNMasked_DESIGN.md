# PN-masked Phase0 통합 설계 (Step 3 가이드) — INNOViD

양산 코드 변경 없음. Step 2 분석 결과를 바탕으로 Step 3 진입 시 참고.

## 1. HNS FWHT vs Phase0 FWHT (요약)

| 구분 | 위치 | 시그니처 / 형태 | 용도 |
|------|------|-----------------|------|
| HNS | `HTS_Holo_LPI.cpp` L35–47 | `static void fwht64(int32_t* d)` | LPI 스칼라 생성 (in-place, N=64) |
| Phase0 / IR | `HTS_V400_Dispatcher_Internal.hpp` | `fwht_raw(int32_t* d, int n)` 의 n==64 언롤 | 페이로드·동기 FWHT (실수부) |
| Phase0 (복소) | 동 파일 | `fwht_64_complex_inplace_(T_I, T_Q)` | I/Q 동시 64점 (진단·일부 경로) |

- **N**: 모두 **64**.
- **HNS `fwht64`**: 일반 butterfly 루프 (`len=1..32`).
- **`fwht_raw(..., 64)`**: 동일 수학적 FWHT이나 **언롤 스테이지 순서** — PN-masked **BPSK 실수 프리앰블**만 쓸 때는 `fwht_raw` 미러와 LUT 행 인덱스가 일치함을 `pn_masked_unit_test.cpp` 로 검증.
- **입력 타입**: HNS/Phase0 모두 **int32_t 누적 버퍼**; 프리앰블은 `int16_t` → **int32_t 승격** 후 FWHT (Step 3 동일 패턴).

## 2. 변경 예정 위치 (Step 3에서만 수정)

- `HTS_V400_Dispatcher_Sync_PSLTE.cpp` — Phase0 / preamble 상관·FWHT 인근 (`p0_buf128_*`, `fwht_*` 호출부).
- `HTS_V400_Dispatcher_Sync_AMI.cpp` — AMI 동등 경로.
- (필요 시) `HTS_V400_Dispatcher.hpp` / 디스패처 코어: `HTS_USE_PN_MASKED` 가드 분기.

**금지 (요구사항 유지)**:`HTS_FEC_HARQ.*`, `HTS_Holo_Tensor_4D_TX.*`, `HTS_Holo_LPI.*` 직접 수정 없음. HNS는 **호출만** 검토; 알고리즘 복사는 **Dispatcher 측** 또는 기존 `fwht_raw` 재사용.

## 3. 통합 흐름 (목표)

### 현재 (PRE_SYM0 고정, 개념)

1. RX preamble 128 chip (`p0_buf128_I_` / `Q_`).
2. 고정 템플릿(예: Walsh-63) 상관 또는 기존 Phase0 규칙.
3. 임계값·동기 확정.

### PN-masked (Step 3~)

1. 동일 128 chip 수집.
2. **첫 64 chip** (또는 정의된 블록)을 `int32_t[64]`에 적재 후 **`fwht_raw(..., 64)`** (또는 동일 정렬의 래퍼).
3. **BPTE**로 `argmax |bin|` → **Walsh row 후보** (0..63).
4. (Step 4) PTE 서브칩 보간.
5. (Step 5) ARM SIMD.
6. (Step 6) Device ID 기반 seed ↔ row 매핑.
7. 임계값·동기 확정; `HTS_USE_PN_MASKED` 미정의 시 **기존 경로 100% 유지**.

## 4. `HTS_USE_PN_MASKED` 가드

- **미정의**: 현행 Phase0, LUT 링크 생략.
- **정의**: LUT `detail::GetPnMaskedPreambleI(seed_row)` 등과 FWHT 검출 결합 (구체 식은 Step 3 구현 시 확정).

## 5. Step 별 작업 (로드맵)

| Step | 내용 |
|------|------|
| 3 | Phase0에 `fwht_raw` + BPTE max + `HTS_USE_PN_MASKED` 가드 |
| 4 | PTE 서브칩 타이밍 |
| 5 | SIMD (SMUAD/SMLAD 등, ARM 빌드) |
| 6 | Seed(Device ID) ↔ row |
| 7 | T6 + KCMVP 검증 |

## 6. 단위 테스트 (Step 2 산출물)

- `HTS_TEST/pn_masked_unit_test.cpp` — 64행 clean 검출.
- `HTS_TEST/cursor_pn_masked_unit_test.cmd` — PC `cl` 빌드.

## 7. 참고 파일 (분석만, 변경 없음)

- `HTS_Holo_LPI.cpp` / `HTS_Holo_LPI.h` — HNS FWHT·LPI 파이프라인.
- `HTS_V400_Dispatcher_Internal.hpp` — `fwht_raw`, `fwht_64_complex_inplace_`.
