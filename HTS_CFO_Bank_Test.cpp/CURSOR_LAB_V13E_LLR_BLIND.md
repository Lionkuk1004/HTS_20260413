# Lab v13e — LLR Sum Combining + 완전 Blind (V13d 동일 악랄 스윕)

## 소스

- `HTS_CFO_Bank_Test_v13e_llr_blind.cpp` — `HTS_CFO_Bank_Test_v13d_pacd_dd.cpp` 기반
- 빌드: `cursor_lab_v13e_llr_blind.cmd` → `HTS_CFO_Bank_Test_v13e_llr_blind.exe`
- 로그: `v13e_llr_blind_result.txt`

## V13d 대비 변경 (두 가지)

1. **Blind Two-Candidates**  
   정답 비트로 cand 고르지 않음.  
   `Decode_Block_Two_Candidates_With_Metric`이 주는 **단일** `metric[k]`에 대해  
   `Σ m·c0` vs `Σ m·c1`으로 후보 선택 (부호 반대 후보만 구별; Walsh 구현 특성상 보통 c0).

2. **Combined**  
   경로별 hard max 대신 **signed metric 합**  
   `llr_sum[k] = metric_legacy[k] + metric_pacd[k] + metric_dd[k]`  
   → 분기 없이 부호로 `±1` 비트 (LLR sum PoC).

## 스윕 조건

V13d와 동일: `kSnrMin=-20`, `kSnrMax=35`, 11 bin, 5000 trial, CFO/timing/MP/ψ 랜덤.

## 비교

실행 결과 하단에 **bin 10**에 대해 V13d 고정 참고값(`v13d_brutal_result.txt`)과 V13e 측정값 차이 표 출력.

## 양산

Lab 전용. 양산 소스 미변경.
