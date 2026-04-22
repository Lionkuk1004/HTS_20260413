# Step 3 DIAG 캡처 보고

## 사전 상태

- 태그: `T_HOLO_SYNC_OFF_OK` (`21c7f9a`)
- 브랜치: `opt/holographic_sync`
- **HTS_LIM 소스 수정 없음** (printf는 Step 2에서 이미 삽입된 것만 사용)

## DIAG `printf` 존재 확인

| 파일 | 패턴 | 결과 |
|------|------|------|
| `HTS_LIM/HTS_V400_Dispatcher_Sync_AMI.cpp` | `[P0-HOLO-AMI]` | L1247 근처 `#if defined(HTS_DIAG_PRINTF)` 블록 **존재** |
| `HTS_LIM/HTS_V400_Dispatcher_Sync_PSLTE.cpp` | `[P0-HOLO-PSLTE]` | L1237 근처 **동일 구조 존재** |

## 빌드 (테스트 TU만)

- **`cursor_t6_build_ami.cmd` / `cursor_t6_build.cmd`**: 임시로 `set HOLO_FLAG=/DHTS_USE_HOLOGRAPHIC_SYNC` 만 활성화.  
  **`/DHTS_DIAG_PRINTF`**: `cl` 라인에 **이미 포함**되어 있어 `HOLO_FLAG`에 중복 넣지 않음 (지시서 D-2와 동등 효과).
- PS-LTE 배치 파일: 한글-only `REM` 줄이 `cmd` 파싱을 깨는 경우가 있어, Step D 동안 **ASCII REM**으로 유지.

| 타깃 | 결과 |
|------|------|
| AMI | OK (`build_ami_diag.log`) |
| PS-LTE | OK (`build_pslte_diag.log`) |

## 로그 파일

| 파일 | 설명 |
|------|------|
| `HTS_TEST/t6_sim/ami_diag_full.log` | AMI 전체 stdout |
| `HTS_TEST/t6_sim/ami_p0_holo_lines.txt` | `[P0-HOLO-AMI]` 행만 추출 |
| `HTS_TEST/t6_sim/pslte_diag_full.log` | PS-LTE 전체 |
| `HTS_TEST/t6_sim/pslte_p0_holo_lines.txt` | `[P0-HOLO-PSLTE]` 행만 추출 |

## AMI DIAG 결과 (`ami_p0_holo_lines.txt`, **155515** 라인)

### `best_e` (로그 필드 `e=`)

| 항목 | 값 |
|------|-----|
| `e` 최소 | 0 |
| `e` 최대 | 416963155 |
| `e` 평균 | **202872949.82** |
| `threshold_12` = (1000×38)² / 8 | **180500000** |
| **평균 / threshold** | **≈ 1.124** |

### `ratio_x10`

| 항목 | 값 |
|------|-----|
| 최소 | 10 |
| 최대 | 9999 (`peak_to_median_ratio_x10` 상한 클램프) |
| 평균 | ≈ **316.36** |
| **ratio_x10 ≥ 25** (코드상 L5 통과 조건) | **7033 / 155515** (≈ **4.52%**) |

### 게이트 플래그 (문자열 `l12=1` 등 출현 횟수 / 총 라인)

| 게이트 | 통과 수 / 총 |
|--------|----------------|
| L12 (`l12=1`) | **132110 / 155515** (≈ **84.95%**) |
| L5 (`l5=1`) | **7033 / 155515** (≈ **4.52%**) |
| L3 (`l3=1`) | **1772 / 155515** (≈ **1.14%**) |
| 전체 `pass=1` | **1772 / 155515** (≈ **1.14%**, **L3 통과 수와 동일**) |

### 처음 20라인 샘플 (`ami_p0_holo_lines.txt`)

```
[P0-HOLO-AMI] off=-1 e=0 ratio_x10=9999 l12=0 l5=1 l3=0 pass=0
[P0-HOLO-AMI] off=-1 e=0 ratio_x10=9999 l12=0 l5=1 l3=0 pass=0
[P0-HOLO-AMI] off=34 e=146707582 ratio_x10=11 l12=0 l5=0 l3=0 pass=0
[P0-HOLO-AMI] off=34 e=146707582 ratio_x10=11 l12=0 l5=0 l3=0 pass=0
[P0-HOLO-AMI] off=62 e=208198796 ratio_x10=21 l12=1 l5=0 l3=0 pass=0
[P0-HOLO-AMI] off=22 e=244995811 ratio_x10=18 l12=1 l5=0 l3=0 pass=0
[P0-HOLO-AMI] off=50 e=208766827 ratio_x10=16 l12=1 l5=0 l3=0 pass=0
[P0-HOLO-AMI] off=42 e=224386998 ratio_x10=17 l12=1 l5=0 l3=0 pass=0
[P0-HOLO-AMI] off=44 e=184556972 ratio_x10=14 l12=1 l5=0 l3=0 pass=0
[P0-HOLO-AMI] off=42 e=216170613 ratio_x10=16 l12=1 l5=0 l3=0 pass=0
[P0-HOLO-AMI] off=46 e=195746191 ratio_x10=15 l12=1 l5=0 l3=0 pass=0
[P0-HOLO-AMI] off=22 e=197942364 ratio_x10=15 l12=1 l5=0 l3=0 pass=0
[P0-HOLO-AMI] off=18 e=228244654 ratio_x10=17 l12=1 l5=0 l3=0 pass=0
[P0-HOLO-AMI] off=34 e=180252414 ratio_x10=14 l12=0 l5=0 l3=0 pass=0
[P0-HOLO-AMI] off=42 e=207504031 ratio_x10=16 l12=1 l5=0 l3=0 pass=0
[P0-HOLO-AMI] off=26 e=151104349 ratio_x10=11 l12=0 l5=0 l3=0 pass=0
[P0-HOLO-AMI] off=13 e=194428241 ratio_x10=15 l12=1 l5=0 l3=0 pass=0
[P0-HOLO-AMI] off=21 e=201152070 ratio_x10=15 l12=1 l5=0 l3=0 pass=0
[P0-HOLO-AMI] off=19 e=197779091 ratio_x10=15 l12=1 l5=0 l3=0 pass=0
[P0-HOLO-AMI] off=54 e=202680691 ratio_x10=16 l12=1 l5=0 l3=0 pass=0
```

## PS-LTE DIAG 결과 (`pslte_p0_holo_lines.txt`, **185653** 라인)

| 항목 | 값 |
|------|-----|
| `e` 평균 | **192777221.73** |
| 평균 / `threshold_12` | **≈ 1.068** |
| `e` 최소 / 최대 | 0 / 309112273 |
| `ratio_x10` 평균 | ≈ **293.08** |
| **ratio_x10 ≥ 25** | **5918 / 185653** (≈ **3.19%**) |
| L12 | **151595 / 185653** (≈ **81.65%**) |
| L5 | **5918 / 185653** |
| L3 | **100 / 185653** |
| `pass=1` | **100 / 185653** (**L3와 동일**) |

### 처음 5라인 샘플 (`pslte_p0_holo_lines.txt`)

```
[P0-HOLO-PSLTE] off=-1 e=0 ratio_x10=9999 l12=0 l5=1 l3=0 pass=0
[P0-HOLO-PSLTE] off=-1 e=0 ratio_x10=9999 l12=0 l5=1 l3=0 pass=0
[P0-HOLO-PSLTE] off=34 e=146707582 ratio_x10=11 l12=0 l5=0 l3=0 pass=0
[P0-HOLO-PSLTE] off=34 e=146707582 ratio_x10=11 l12=0 l5=0 l3=0 pass=0
[P0-HOLO-PSLTE] off=62 e=208198796 ratio_x10=21 l12=1 l5=0 l3=0 pass=0
```

## D-6 S5-HOLO 구간 표식

- `ami_diag_full.log` 에서 `S5-HOLO` 헤더 문자열 검색: **존재** (hdr 출력).

## 원상복구 (Step D-7)

- `cursor_t6_build_ami.cmd` / `cursor_t6_build.cmd` 의 `HOLO_FLAG` 를 **빈 문자열 + 주석 형태**로 복구함.
- **검증**: `cursor_t6_build_ami.cmd` 재빌드 후 `정량 합계` → **14939 / 18400** 확인.

---

## 수치만 기반한 분류 (지시서 “힌트” Case 1~4 대응)

| Case | 힌트 조건 | AMI 실측 | 판단 |
|------|-----------|-----------|------|
| 1 | `best_e` 극소 (~threshold의 극소 분수) | 평균 **~2.03×10⁸** > `threshold_12` | **해당 없음** |
| 2 | `best_e` 중간, 스케일만 문제 | 에너지는 threshold 근처 이상이나 **L12가 대다수 통과** | “**스케일만**”으로는 설명 부족 |
| 3 | `best_e` 충분, **L5 또는 L3** 병목 | AMI: **L5 통과 ≈4.5%**, L3는 L5 이후라 더 적음·`pass`≡L3 | **부합 (AMI는 L5가 1차 병목)** |
| 3 (PS-LTE) | 동일 | L5 통과 ≈3.2%, **L3·`pass` 100건** | **L3가 최종 병목** |

### 추정 (가) vs (나) — **측정으로 좁힌 결론**

- **(가) “Chu descramble 때문에 에너지가 threshold의 1/10⁶”**  
  - **실측과 정반대에 가깝다.** 대표적인 `e`는 **1.5×10⁸ ~ 2.5×10⁸** 구간이 많고, 평균은 `threshold_12` **위**이다.  
  - 다만 **`e=0`, `off=-1`** 같은 퇴화 샘플이 존재하며, 이때는 `ratio_x10=9999`로 **L5만 이상하게 통과**하는 라인이 있다(코드의 median≤0 분기).

- **(나) “AMI amp×19 vs 홀로 amp×38 + AND 게이트”**  
  - **L12(에너지 절대 임계)** 단독으로 전체 실패를 설명하기 어렵다 — AMI에서 **L12 통과 ≈85%**.  
  - **L5(peak/median ≥ 2.5)** 가 AMI에서 **통과 ≈4.5%**로 가장 좁은 목.  
  - PS-LTE에서는 **L5 이후 L3**가 **`pass`와 같은 100건**으로 최종 병목.

### 체크박스 (원인 축 — **측정 기반만**)

- [ ] **(가) 단독**: TX/RX Chu 도메인 불일치만으로 “에너지 극소” 설명 → **실측으로 지지되지 않음**  
- [ ] **(나) 단독**: amp×19/38 스케일만 → **L12 대량 통과로 단독 설명 불가**  
- [x] **혼합 / 게이트 병목**: **에너지 스케일은 대체로 충분한 구간이 많으나**, **L5(AMI·공통) 및 PS-LTE의 L3**가 AND 체인에서 지배적이다.

---

## 제안 다음 단계 (해석 아닌 작업 목록)

- [ ] L5: `ratio_x10` 임계(25) 또는 median/peak 정의 재검토(시뮬 vs T6 분포)  
- [ ] PS-LTE L3: `fwht_raw` 기반 `top4` vs `threshold_3` 정합  
- [ ] `off=-1`, `e=0` 분기(버퍼 미충족/초기 상태) 별도 처리 여부 검토  

---

## 커밋

- **없음** (조사 전용). 로그·추출 텍스트는 `HTS_TEST/t6_sim/` 에 남음.
