# PN-masked KCMVP Timing 검증

Step 7-1: PC에서 입력 패턴별 per-call 시간 분산을 측정해 **데이터 의존 타이밍 변동**이 작은지 확인한다. 임계: 동일 함수 내 3가지 입력 대비 **평균 대비 최대 편차 < 5%** → `PASS`, 그 외 `INVESTIGATE` (OS·캐시·전력 상태에 따라 PC 단독 측정은 참고용).

## 측정 환경

- MSVC x64 Release (`/O2 /MD`)
- `pn_masked_unit_test.cpp` → `cursor_pn_masked_unit_test.cmd`
- `HTS_USE_PN_MASKED`, `HTS_ALLOW_HOST_BUILD`
- 반복: **100000** calls / 입력, 워밍업 **2000** calls
- `pn_masked_phase0_scan` 내부 FWHT: `fwht_raw` 경로 (ARM SIMD 래퍼는 PC에서 동일 butterfly fallback)

## 결과 (대표 1회 — 2026-04-27 로컬 실행)

| 함수 | per-call (avg) | max deviation | constant-time |
|------|----------------|---------------|---------------|
| `pn_masked_phase0_scan` | 127.0 ns | 2.68% | PASS |
| `pn_masked_pte_subchip` | 2.6 ns | 1.31% | PASS |
| `pn_masked_device_id_to_row` | 0.3 ns | 0.74% | PASS |
| `pn_masked_phase0_subchip_refine` | 2.6 ns | 0.96% | PASS |

### 입력 패턴 (요약)

- **phase0_scan**: I/Q 64샘플 `zero` / `all 32767` / 의사 랜덤
- **pte_subchip**: 대칭 피크, 상한 클램프, 하한 클램프 삼중
- **device_id_to_row**: `0`, `0xFFFFFFFF`, `0xDEADBEEF`
- **phase0_subchip_refine**: PTE 래퍼 — 대칭·비대칭·근접 피크 삼중

### Phase0 예산 (참고)

- 32회 coarse offset × 평균 **127 ns** ≈ **4.1 µs** (1 ms 대비 여유)

## 분석

- **BPTE argmax** (`phase0_scan`): 분기 없는 `|v|` 비교 마스크 갱신; 입력 에너지 스케일이 달라도 per-call 시간 분산이 작게 관측됨.
- **PTE** (`pte_subchip`): 정수 나눗셈 1회 + BPTE `den==0` 대체·클램프; 구현은 데이터에 따른 **제어 흐름 분기 없음**.
- **Device ID → row**: XOR fold + BPTE 클램프만; 관측 편차는 타이머 해상도에 가깝다.

## KCMVP 적합성 (구현 관점)

- 의도적 **비밀 의존 분기 없음**: 소스 상 BPTE 패턴 유지 (Step 6-1/4-1와 동일).
- **PC 타이밍 검증**: 상기 분산 기준 PASS — **양산 ARM**에서는 별도 실측·인증 루틴에 편입할 것.
- **Timing attack**: 본 측정은 호스트 OS 하에서의 휴리스틱이며, 최종 주장은 CC 인증 범위 내 하드웨어·빌드 설정으로 확정한다.

## 재현

```bat
cd HTS_TEST
cursor_pn_masked_unit_test.cmd
pn_masked_unit_test.exe
```

출력의 `=== Constant-time check (Step 7-1 KCMVP) ===` 블록을 갱신해 본 문서 표를 업데이트하면 된다.
