# Phase 3 measurement analysis (auto-generated)

Clopper-Pearson exact CI (alpha=0.05), bits/point ≥ 100000.

## Scenario S1_CLIFF — J1 AWGN cliff precision

- **Jammer**: `J1_AWGN`
- **MIL reference (prompt)**: MIL-STD-188-110D
- **Trials/point**: 1000 (~64000 bits)

### Limit estimates (linear interp, monotonic sweep)

| Threshold | Param at crossing |
|-----------|--------------------|
| BER 1e-2 | -3.03846 |
| BER 1e-3 | N/A |
| BER 1e-5 | N/A |
| BER 1e-6 | N/A |

### Per-point (first / mid / last)

- param=-3 BER=0.009 CI=[0.008283,0.009762] PER=0.009 dec_fail=9 sat=N/A
- param=-8 BER=0.45 CI=[0.4461,0.4539] PER=0.45 dec_fail=450 sat=N/A
- param=-12 BER=0.956 CI=[0.9544,0.9576] PER=0.956 dec_fail=956 sat=N/A

### Interpretation (한계 탐지)

- 본 표는 **측정 곡선**에서의 교차 추정치이며, MIL-STD **합격/불합격 판정**이 아님.
- Post-FEC 페이로드 BER (Phase 2 `measure_ber_per`) 기준.

## Scenario S1 — J1 AWGN (MIL-STD-188-110D ref sweep)

- **Jammer**: `J1`
- **MIL reference (prompt)**: MIL-STD-188-110D (voice/data BER ref: program table)
- **Trials/point**: 1563 (~100032 bits)

### Limit estimates (linear interp, monotonic sweep)

| Threshold | Param at crossing |
|-----------|--------------------|
| BER 1e-2 | -3.14935 |
| BER 1e-3 | -1.563 |
| BER 1e-5 | 0.98437 |
| BER 1e-6 | 0.998437 |

### Per-point (first / mid / last)

- param=20 BER=0 CI=[0,3.688e-05] PER=0 dec_fail=0 sat=N/A
- param=0 BER=0.0006398 CI=[0.0004928,0.0008169] PER=0.0006398 dec_fail=1 sat=N/A
- param=-20 BER=1 CI=[1,1] PER=1 dec_fail=1563 sat=N/A

### Interpretation (한계 탐지)

- 본 표는 **측정 곡선**에서의 교차 추정치이며, MIL-STD **합격/불합격 판정**이 아님.
- Post-FEC 페이로드 BER (Phase 2 `measure_ber_per`) 기준.

