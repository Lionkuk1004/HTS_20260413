# HTS B-CDMA 전수검사 통합 기준서
**INNOViD CORE-X Pro HTS B-CDMA 보안통신 펌웨어**
**버전 4.0 — 2026.04.01 (세션 4차 확정)**

---

## 0. 타겟 플랫폼 스펙

### 0-1. B-CDMA 보안 코프로세서 (메인 타겟)

| 항목 | 사양 |
|------|------|
| MCU | STM32F407VGT6 |
| 코어 | ARM Cortex-M4F (하드웨어 FPU) |
| 클럭 | 168 MHz (max), AHB=168MHz, APB1=42MHz, APB2=84MHz |
| Flash | 1 MB (단일 뱅크, 섹터 0~11) |
| SRAM | 192 KB (112KB 메인 + 16KB SRAM2 + 64KB CCM) |
| CCM | 64 KB (Core Coupled Memory, DMA 접근 불가, HARQ 버퍼 전용) |
| FPU | 단정밀도 FPU (float 하드웨어, double 소프트웨어 에뮬) |
| MPU | 8개 리전 (코드 실행 보호, 스택 가드, DMA 분리) |
| NVIC | 82 인터럽트, 16단계 우선순위 (4비트 프리엠션) |
| DMA | DMA1(8ch) + DMA2(8ch), 이중 버퍼 모드 |
| 암호 | 소프트웨어 KCMVP(ARIA/LEA/LSH) + FIPS 140-3(AES-256/SHA-256) |
| 버스 | AHB1(GPIO/DMA), AHB2(USB OTG), APB1(SPI2/3, UART4/5), APB2(SPI1, USART1/6) |
| 패키지 | LQFP-100 |
| 전압 | 1.8~3.6V (PVD 감시) |
| 온도 | -40~85°C (산업용) |

### 0-2. 유무선 변환 프로세서 (보조)

| 항목 | 사양 |
|------|------|
| SoC | Cortex-A55 Linux |
| 역할 | 유선↔무선 변환, 기본 보안 (TLS) |
| IPC | SPI(최대 42Mbps) + UART(115200bps) 이중 채널 |

### 0-3. SRAM 메모리 맵 (확정)

| 영역 | 주소 | 크기 | 용도 |
|------|------|------|------|
| SRAM 메인 | 0x2000_0000 | 112 KB | 코드 + BSS + 스택 + 오버레이 풀 |
| SRAM2 | 0x2001_C000 | 16 KB | ISR 스택 + 보안 버퍼 |
| CCM | 0x1000_0000 | 64 KB | HARQ 버퍼 (DMA 접근 불가) |
| 오버레이 풀 | SRAM 내 | ~80 KB | 시분할 모듈 로딩 |

### 0-4. 빌드 구성

| 프리셋 | 정의 | 암호 알고리즘 | 대상 |
|--------|------|--------------|------|
| HTS_CRYPTO_KCMVP | ARIA/LEA/LSH256/HMAC | 국내 (NIS/KT) |
| HTS_CRYPTO_FIPS | AES-256/SHA-256 | 국제 수출 |
| HTS_CRYPTO_DUAL | 양쪽 모두 | 이중 인증 |

### 0-5. Cortex-M4 하드웨어 제약 (검수 시 필수 고려)

| 제약 | 상세 | 검수 영향 |
|------|------|-----------|
| 비정렬 접근 | LDM/STM/LDRD → UsageFault | alignas 강제 |
| 64비트 원자성 없음 | atomic<uint64_t> → tearing | PRIMASK 크리티컬 섹션 |
| 소프트웨어 나눗셈 | 64비트 나눗셈 → __aeabi_uldivmod(~200cyc) | 시프트+마스크 대체 |
| 32비트 UDIV | 하드웨어 지원 (2~12cyc) | 허용 |
| FPU 단정밀도 | double → 소프트웨어 에뮬(~100cyc) | double 전면 금지 |
| 스택 크기 | MSP 2~4KB 일반 | 대형 로컬 배열 금지 |
| Flash 대기 | 5 Wait State @168MHz | constexpr → Flash 읽기 최소화 |
| ISR 지연 | 12~16사이클 진입 | ISR 내 긴 로직 금지 |

---

## 1. 필수 검수 17항 + 추가 4항 (프롬프트 기준)

### 1-1. 기본 17항

| No | 항목 | 기준 | FAIL 시 영향 |
|----|------|------|-------------|
| ① | memory_order 배리어 | seq_cst 금지, acquire/release 다운그레이드 | 사이클 낭비 |
| ② | std::abs → fast_abs | 비트마스크 절대값 | 불필요 분기 |
| ③ | 힙 할당 금지 | new/malloc/shared_ptr/unique_ptr ARM 전면 금지 | 힙 파편화 |
| ④ | double/float 금지 | ARM 코드 내 부동소수점 전면 금지 | FPU 에뮬 지연 |
| ⑤ | try-catch 금지 | -fno-exceptions | 코드 사이즈 폭증 |
| ⑥ | 스택 사용량 | 대형 로컬 배열 → static/전역 | 스택 오버플로 |
| ⑦ | 주석-코드 불일치 | 주석과 실제 로직 정합성 | 유지보수 트랩 |
| ⑧ | SRAM static_assert | 구조체 크기 빌드타임 검증 | 런타임 오버플로 |
| ⑨ | 나눗셈→시프트 | 64비트 나눗셈 전면 제거, 32비트 UDIV 허용, constexpr 허용, 비2의제곱은 Q16 역수 곱셈+시프트 | ALU 병목 |
| ⑩ | const& + zero-copy | 함수 인자 복사 방지 | 메모리 낭비 |
| ⑪ | 엔디안 독립 | 포인터 캐스팅 금지, 명시적 시프트 | 바이트 순서 오류 |
| ⑫ | static_cast | 암묵적 형변환 금지, dynamic_cast/typeid 전면 금지 | 부호 확장 오류 |
| ⑬ | CFI 상태전이 검증 | 상태 머신 전이 합법성 매번 검증 | ROP/JOP 공격 |
| ⑭ | PC 코드 삭제 | `<cstdlib>`, `<windows.h>`, `<unistd.h>`, `std::abort()` → `#ifndef HTS_PLATFORM_ARM` 가드, `<thread>/<mutex>/<condition_variable>/<future>` ARM 발견 즉시 FAIL | Flash 낭비 |
| ⑮ | HW 레지스터 폴링 타임아웃 | SPI/UART/DMA 대기루프 무한행 방지 | MCU 행업 |
| ⑯ | 전원강하/EMI 비트플립 방어 | PVD + 이중뱅크 + Flash 쓰기 원자성 | 데이터 오염 |
| ⑰ | 하드 리얼타임 데드라인 | NVIC 우선순위 + ISR WCET + 타임슬롯 마감 보장 | 통신 실패 |

### 1-2. 추가 보충 4항 (A~D)

| No | 항목 | 기준 | FAIL 시 영향 |
|----|------|------|-------------|
| A | sizeof 전파 경고 | 1KB+ 멤버 포함 래퍼 → Doxygen @warning에 sizeof + "전역/정적 배치 필수" | 스택 오버플로 |
| B | 래퍼 static_assert | 1KB+ 클래스 → sizeof≤SRAM예산 빌드타임 검증 | 런타임 오버플로 |
| C | 원자적 초기화 CAS | atomic<bool> 멱등성 가드 → compare_exchange_strong(acq_rel) | 이중 초기화 |
| D | C++20 속성 가드 매크로 | `[[likely]]`/`[[unlikely]]`/`[[nodiscard]]` → `#if __cplusplus >= 202002L` 가드 | 빌드 실패 |

---

## 2. 동시성 및 메모리 배리어 (A항)

| No | 항목 | 기준 |
|----|------|------|
| A-1 | 오더링 오버헤드 최소화 | seq_cst → relaxed/acquire/release 다운그레이드 |
| A-2 | 컴파일러/CPU 재배치 방어 | DMA/ISR 공유 영역 → atomic_thread_fence + volatile |
| A-3 | Lock-free/Wait-free 설계 | ISR/실시간루프 → mutex/spinlock 전면 배제, CAS 링 버퍼 |

---

## 3. 메모리 및 리소스 무결성 (B항)

| No | 항목 | 기준 |
|----|------|------|
| B-1 | Zero-Copy + 임시 객체 제거 | const& 또는 포인터, 런타임 동적 할당 전면 금지 |
| B-2 | 메모리 정렬(Alignment) | alignas 남용/누락 → Unaligned Access Fault 확인 |
| B-3 | 스택 무결성 | 대형 로컬 배열 → static/전역, 깊은 재귀 배제 |

---

## 4. 제어 흐름 무결성 (C항)

| No | 항목 | 기준 |
|----|------|------|
| C-1 | CFI 검증 | 상태 머신 전이 합법성 검증 (ROP/JOP 방어) |
| C-2 | 예외 처리 배제 | try-catch/throw 제거, new(std::nothrow) |
| C-3 | HardFault 폴백 | 안전 에러 핸들러, 레지스터 로깅 + 리셋 |

---

## 5. 보안 및 안티포렌식 (D항)

| No | 항목 | 기준 |
|----|------|------|
| D-1 | Constant-time 실행 | 암호 연산 내 if/switch 금지 → 비트마스크 로직 |
| D-2 | 즉각적 보안 소거 | volatile ptr + asm clobber + release fence 3중 방어 |
| D-3 | 디버그 포트 무력화 | JTAG/SWD 감지 → AIRCR 즉시 리셋 |

---

## 6. DSP 및 알고리즘 최적화 (E항)

| No | 항목 | 기준 |
|----|------|------|
| E-1 | ALU 병목 제거 | 나눗셈/모듈로 → 시프트+비트마스크 100% 대체 |
| E-2 | HW Intrinsics 활용 | __builtin_popcount, __clz 등 단일사이클 명령 |
| E-3 | 분기 예측 최적화 | Hot Path → [[likely]]/[[unlikely]] (C++20 가드) |

---

## 7. 데이터 무결성 및 통신 (F항)

| No | 항목 | 기준 |
|----|------|------|
| F-1 | 엔디안 독립성 | 포인터 캐스팅 금지 → 명시적 비트 시프트 |
| F-2 | 오류 검출 코드 | 통신 페이로드 + 설정값 → CRC-16/32 또는 해시 |

---

## 8. 빌드 타임 검증 (G항)

| No | 항목 | 기준 |
|----|------|------|
| G-1 | static_assert | 구조체 크기, 비트마스크, 배열 길이 빌드타임 검증 |
| G-2 | 엄격한 형변환 | 암묵적 캐스팅 금지 → static_cast 명시 |
| G-3 | 경고 에러화 | -Wall -Wextra -Werror (경고 0개) |

---

## 9. 포인터 및 메모리 관리 (H항 — 20항)

| No | 항목 |
|----|------|
| H-1 | NULL 포인터 역참조 방지 |
| H-2 | 댕글링 포인터 검사 |
| H-3 | 버퍼 오버플로우 (memcpy/strcpy 경계) |
| H-4 | 동적 메모리 할당 제한 (malloc/free 금지) |
| H-5 | 정렬 위반 (Cortex-M4 Fault) |
| H-6 | new/malloc 후 delete/free 쌍 |
| H-7 | 메모리 누수 (모든 경로 해제) |
| H-8 | 더블 프리 방지 |
| H-9 | 스택 버퍼 오버플로우 |
| H-10 | 힙 무결성 |
| H-11 | 초기화되지 않은 변수 |
| H-12 | strcpy/strcat 금지 → strncpy/snprintf |
| H-13 | sprintf 금지 → snprintf |
| H-14 | gets() 금지 → fgets() |
| H-15 | memcpy 크기 소스/타겟 일치 |
| H-16 | 포인터 연산 유효 범위 |
| H-17 | 상속 구조 가상 소멸자 |
| H-18 | 생성자/소멸자 내 가상 함수 호출 자제 |
| H-19 | 지역 변수 주소 반환 금지 |
| H-20 | ASan 설정 확인 |

---

## 10. 제어 흐름 및 예외 처리 (I항)

| No | 항목 |
|----|------|
| I-1 | ISR 내 긴 로직 금지 |
| I-2 | 무한 루프 탈출 조건 |
| I-3 | switch default 케이스 |
| I-4 | 재귀 호출 금지 |

---

## 11. 타입 안정성 (J항)

| No | 항목 |
|----|------|
| J-1 | 형 변환 안전성 (큰→작은 유실) |
| J-2 | Unsigned 언더/오버플로우 |
| J-3 | 매직 넘버 금지 → constexpr/enum (SWAR/CRC/LCG/비트마스크 예외) |
| J-4 | volatile 올바른 사용 |

---

## 12. Cortex-M4 전용 (K항)

| No | 항목 |
|----|------|
| K-1 | MPU 설정 (Flash 쓰기 불가) |
| K-2 | DSP 명령어 데이터 범위 검사 |
| K-3 | FPU 레지스터 컨텍스트 저장 |

---

## 13. 가독성 및 유지보수 (L항)

| No | 항목 |
|----|------|
| L-1 | 함수 복잡도 (Cyclomatic Complexity) |
| L-2 | 변수 초기화 |
| L-3 | 미사용 코드/변수 제거 |

---

## 14. 정적 분석 및 표준 준수 (M항 — 20항)

| No | 항목 |
|----|------|
| M-1 | Cppcheck 결함 탐지 |
| M-2 | Clang-tidy 모던 C++ |
| M-3 | -Wall -Wextra -Wpedantic |
| M-4 | -Werror |
| M-5 | const/constexpr 상수화 |
| M-6 | enum class 명시적 타입 |
| M-7 | explicit 생성자 |
| M-8 | static_cast/reinterpret_cast (C캐스팅 금지) |
| M-9 | nullptr (NULL 금지) |
| M-10 | auto 남용 자제 |
| M-11 | 범위 기반 for |
| M-12 | std::move 활용 |
| M-13 | 템플릿 과용 자제 |
| M-14 | DRY 원칙 (중복 코드 제거) |
| M-15 | 데드 코드 제거 |
| M-16 | 함수 길이 적정 |
| M-17 | 주석-코드 일치 |
| M-18 | 네임스페이스 (전역 오염 방지) |
| M-19 | unsigned/signed 혼용 주의 |
| M-20 | placement new (베어메탈) |

---

## 15. 동시성 (N항 — 15항)

| No | 항목 |
|----|------|
| N-1 | 데이터 레이스 방어 |
| N-2 | Mutex/Lock (PC용, ARM은 Lock-free) |
| N-3 | 데드락 방지 |
| N-4 | std::atomic 사용 |
| N-5 | 공유 상태 최소화 |
| N-6 | 스레드 비안전 함수 주의 |
| N-7 | volatile 오용 금지 (atomic 사용) |
| N-8 | 조건 변수 (PC용) |
| N-9 | 비동기 처리 (PC용) |
| N-10 | 락 범위 최소화 |
| N-11 | 재진입성 |
| N-12 | 스레드 종료 Join/Detach (PC용) |
| N-13 | shared_ptr 멀티스레드 안전 |
| N-14 | 예외 시 락 해제 (RAII) |
| N-15 | thread_local 활용 |

---

## 16. 입력 검증 및 보안 (O항 — 15항)

| No | 항목 |
|----|------|
| O-1 | 입력 값 범위 검사 |
| O-2 | 명령어 인젝션 방지 |
| O-3 | 형식 문자열 취약점 |
| O-4 | 버퍼 크기 검증 |
| O-5 | NULL 바이트 삽입 방지 |
| O-6 | UTF-8 인코딩 검증 |
| O-7 | 화이트리스트 입력 |
| O-8 | 오류 메시지 정보 노출 방지 |
| O-9 | 평문 키/비밀번호 저장 금지 |
| O-10 | 암호학적 난수 (PUF+PRNG) |
| O-11 | 세션 관리 |
| O-12 | 코드 주입 방지 |
| O-13 | 인자 경계 검사 |
| O-14 | 경로 조작 방지 |
| O-15 | 파일 크기/형식 검증 |

---

## 17. 런타임 성능 (P항 — 15항)

| No | 항목 |
|----|------|
| P-1 | assert (디버그 모드) |
| P-2 | RTTI 비용 (dynamic_cast 자제) |
| P-3 | noexcept 지정 |
| P-4 | 알고리즘 복잡도 (O(n²)+ 경보) |
| P-5 | 컨테이너 선택 (ARM: 정적 배열) |
| P-6 | 캐시 지역성 |
| P-7 | const T& 전달 |
| P-8 | RVO/NRVO 활성화 |
| P-9 | 임시 객체 최소화 |
| P-10 | 루프 불변식 외부 이동 |
| P-11 | 정적 라이브러리 불필요 링크 제거 |
| P-12 | 런타임 무결성 해시 체크 |
| P-13 | 예외 안전성 (RAII) |
| P-14 | 스트림 오버헤드 (ARM: 스트림 금지) |
| P-15 | 핫 루프 최적화 |

---

## 18. 프로젝트 구조 (Q항 — 15항)

| No | 항목 |
|----|------|
| Q-1 | 헤더 가드 (#pragma once) |
| Q-2 | include 최소화 |
| Q-3 | 전방 선언 (Pimpl) |
| Q-4 | Debug/Release 분리 |
| Q-5 | 코드 스타일 일관성 |
| Q-6 | Doxygen API 문서화 |
| Q-7 | 외부 라이브러리 버전 고정 |
| Q-8 | ARM/PC 플랫폼 분리 |
| Q-9 | Self-Contained 헤더 |
| Q-10 | ProtectedEngine 네임스페이스 |
| Q-11 | 복사/이동 = delete |
| Q-12 | [[nodiscard]] |
| Q-13 | constexpr 컴파일타임 상수화 |
| Q-14 | inline constexpr (ODR) |
| Q-15 | static_assert 아키텍처 검증 |

---

## 19. 보안 부팅 및 펌웨어 검증 (R항 — 30항)

| No | 항목 |
|----|------|
| R-1 | 부트 코드 ROM 불변성 |
| R-2 | 펌웨어 디지털 서명 (RSA/ECC) |
| R-3 | SHA-256/SHA-3 해시 무결성 |
| R-4 | OTP 안전 부팅 설정 |
| R-5 | Anti-rollback |
| R-6 | 안전 키 저장소 |
| R-7 | 부트 영역 쓰기 보호 |
| R-8 | QSPI 외부 플래시 검증 |
| R-9 | Flash ECC 쓰기 확인 |
| R-10 | 설정값 CRC-32 |
| R-11 | MPU 보안 영역 분리 |
| R-12 | 스택 Guard Value 감시 |
| R-13 | 힙 오버플로우 검출 |
| R-14 | Read-only 데이터 보호 |
| R-15 | 부팅 시 전체 CRC |
| R-16 | 런타임 코드 변경 감지 |
| R-17 | 워치독 타이머 (WDT) |
| R-18 | 윈도우 워치독 |
| R-19 | HardFault 재부팅 |
| R-20 | 클럭 감시 (CSS) |
| R-21 | 저전력 모드 상태 검사 |
| R-22 | JTAG/SWD 비활성화 |
| R-23 | 입력 경계 검사 |
| R-24 | 암호화 펌웨어 |
| R-25 | 코드 주입 방지 |
| R-26 | 보안 HW 가속기 |
| R-27 | 서명 키 주기 변경 |
| R-28 | 동적 펌웨어 갱신 차단 |
| R-29 | Secure Boot State 확인 |
| R-30 | 무결성 실패 → 안전 모드 |

---

## 20. 구조적 안전성 (U항 — 신규 4항)

| No | 항목 | 기준 |
|----|------|------|
| U-A | sizeof 전파 경고 | 1KB+ 멤버 → @warning sizeof + "전역/정적 필수" |
| U-B | 래퍼 static_assert | sizeof≤SRAM예산 빌드타임 검증 |
| U-C | 원자적 초기화 CAS | compare_exchange_strong(acq_rel), 중복 store 제거 |
| U-D | C++20 속성 가드 | [[likely]]/[[unlikely]] → `#if __cplusplus >= 202002L` |

---

## 21. 모듈별 특수 취약점 (V항 — 35항)

### V-1. BLE/NFC 게이트웨이 (10항)

| No | 항목 |
|----|------|
| V-1-1 | AT TX 전용 버퍼 독립 할당 |
| V-1-2 | 오버플로 라인 즉시 폐기 (uart_line_overflow) |
| V-1-3 | 인밴드 AT 인젝션 차단 (바이트 카운팅 + 트레일링 \n 소각) |
| V-1-4 | 프레임 밀반입 차단 (정확 길이 1:1 일치) |
| V-1-5 | 세션 암살 방지 (역순 탐색) |
| V-1-6 | 64비트 국가지점번호 검증 (오라우팅 차단) |
| V-1-7 | msg_type 프로토콜 분기 (바보 파이프 방지) |
| V-1-8 | Send_* 세션 게이트키퍼 (위조 전송 차단) |
| V-1-9 | 일괄 틱 갱신 철거 (좀비 세션 방지) |
| V-1-10 | SPSC 링 버퍼 PRIMASK 보호 (Nested ISR Race) |

### V-2. CCTV 보안 코프로세서 (8항)

| No | 항목 |
|----|------|
| V-2-1 | Event Storm DoS → Edge 트리거 |
| V-2-2 | 시계열 정합성 → 역순 탐색 |
| V-2-3 | nullptr → detail_len=0 클램프 |
| V-2-4 | MAC 키 역산 → CRC 연쇄 혼합 |
| V-2-5 | 1틱 격리 → 즉시 return |
| V-2-6 | 틱 보상 폭주 → 타이머 강제 동기화 |
| V-2-7 | 무한 락다운 → Re-baseline |
| V-2-8 | OFFLINE 전이 → from!=0 합법화 |

### V-3. CoAP 엔진 (14항)

| No | 항목 |
|----|------|
| V-3-1 | URI 버퍼 오버플로 → 클램프 |
| V-3-2 | 스택 평문 잔류 → Secure_Wipe |
| V-3-3 | next_mid/token → atomic |
| V-3-4 | TKL 미검증 → 즉각 폐기 |
| V-3-5 | Piggybacked Response → URI 파싱 철거 |
| V-3-6 | 0xFF Payload Marker 삽입 |
| V-3-7 | 파서 스머글링 → 0xFF 실존 검증 |
| V-3-8 | safe_streq → XOR 상수 시간 |
| V-3-9 | safe_shift 21 클램프 |
| V-3-10 | alloc_state 4단계 CAS |
| V-3-11 | ACK CAS(READY→WIPING) 독점 |
| V-3-12 | Enqueue → PRIMASK 외부 (로컬 복사) |
| V-3-13 | atomic 객체 Wipe(UB) 금지 → msg만 정밀 소거 |
| V-3-14 | Register URI 사전 소거 (XOR 호환) |

### V-4. 자가진단 + Config (3항)

| No | 항목 |
|----|------|
| V-4-1 | alignas(uint32_t) LEA 버퍼 |
| V-4-2 | HMAC ctx 5경로 Wipe |
| V-4-3 | 곱셈(*) → 시프트(<<) 컨벤션 |

---

## 22. 수석 아키텍트 추가 규약 (W항 — 4항)

| No | 항목 | 기준 |
|----|------|------|
| W-1 | DMA 캐시 일관성 | D-Cache Invalidate/Clean 또는 Non-cacheable 지정 |
| W-2 | WDT 펫팅 제한 | ISR/데드락 루프 내 금지, 메인 루프 최상단만 허용 |
| W-3 | Flash 마모 평준화 | NVRAM 링 버퍼 기반 Wear-Leveling |
| W-4 | 인터럽트 폭주 차단 | ISR 내 디바운싱/일시적 마스킹 |

---

## 23. 추가 정밀검사 항목 (X항 — 신규 20항)

본 항목은 기존 기준서에서 명시되지 않았으나, 양산 정밀검사 시 반드시 확인해야 하는 사항이다.

### X-1. 하드웨어 레지스터 접근

| No | 항목 | 기준 |
|----|------|------|
| X-1-1 | 레지스터 주소 constexpr 상수화 | AIRCR/VECTKEY 등 HW 주소를 매직 넘버로 사용 금지 |
| X-1-2 | 레지스터 Read-Modify-Write 원자성 | PRIMASK로 보호하거나 비트밴딩 사용 |
| X-1-3 | 페리페럴 클럭 활성화 확인 | RCC 레지스터에서 해당 페리페럴 클럭 ON 후 접근 |

### X-2. DMA 안전성

| No | 항목 | 기준 |
|----|------|------|
| X-2-1 | DMA 버퍼 Non-cacheable 또는 Cache 관리 | D-Cache 라인 경계 정렬 (32바이트) |
| X-2-2 | DMA 전송 완료 콜백 내 volatile 배리어 | 전송 완료 플래그에 DSB/DMB 발행 |
| X-2-3 | DMA 이중 버퍼 교차 오염 방지 | 핑퐁 버퍼 전환 시 이전 버퍼 잠금 |

### X-3. 전원 관리 및 복원

| No | 항목 | 기준 |
|----|------|------|
| X-3-1 | PVD 임계값 설정 검증 | STM32 2.7V 레벨로 설정 (Flash 오류 방지) |
| X-3-2 | 브라운아웃 시 NVM 쓰기 중단 | Flash 프로그래밍 중 전원 강하 → 원자적 폴백 |
| X-3-3 | 웨이크업 후 클럭 재설정 | STOP/STANDBY 복귀 시 PLL 재설정 확인 |

### X-4. 통신 버스 안전성

| No | 항목 | 기준 |
|----|------|------|
| X-4-1 | SPI NSS 관리 | 소프트웨어 NSS → 크리티컬 섹션 내 토글 |
| X-4-2 | UART 프레이밍 에러 처리 | ORE/NE/FE 플래그 클리어 + 에러 카운터 |
| X-4-3 | I2C 버스 행업 복구 | SCL 토글 10회 + SWRST 시퀀스 |
| X-4-4 | 버스 타임아웃 | 모든 페리페럴 대기에 하드웨어 타이머 연동 |

### X-5. 보안 강화

| No | 항목 | 기준 |
|----|------|------|
| X-5-1 | Secure Wipe 3중 방어 검증 | (volatile + asm clobber + release fence) 누락 없음 |
| X-5-2 | 키 유도 함수 안전성 | KDF 입력에 nonce/salt 포함 (replay 방지) |
| X-5-3 | 사이드 채널 방어 전수 확인 | 모든 암호 비교 함수 → 상수 시간 XOR |
| X-5-4 | Write Suppression 공격 방어 | 보안 비교 반환형 bool → uint32_t |
| X-5-5 | Boolean Coercion 방어 | bool 반환 보안 함수 → uint32_t (glitch 주입 방어) |
| X-5-6 | 64비트 Data Tearing 방어 | atomic<uint64_t> → 두 개 atomic<uint32_t> 또는 PRIMASK |

### X-6. 시간 관련

| No | 항목 | 기준 |
|----|------|------|
| X-6-1 | systick 래핑 안전성 | 49.7일(uint32_t ms) 래핑 시 elapsed 계산 안전 확인 |
| X-6-2 | 타이머 오버플로우 | TIM 카운터 16/32비트 경계 처리 |

---

## 24. 정오표 핵심 예외

| 예외 | 설명 |
|------|------|
| J-3 예외 | SWAR/CRC/LCG/비트마스크 알고리즘 표준상수 → 명명 없이 허용 |
| Secure Wipe 표준 | memset + asm clobber + release fence 3중 (volatile 단독 불충분) |
| std::atomic 전용 | volatile 변수를 동기화 목적 사용 금지, std::atomic만 허용 |
| constexpr 나눗셈 | 컴파일타임 상수 나눗셈은 허용 (런타임 ALU 미사용) |
| 32비트 UDIV | ARM Cortex-M4 하드웨어 UDIV 2~12cyc → 허용 |

---

## 25. 판정 체계

| 판정 | 의미 |
|------|------|
| PASS | 기준 충족 |
| PEND | 확인 보류 (추가 정보 필요) |
| FAIL | 기준 위반 → 즉시 수정 |
| CHK | 검수 중 |
| N/A | 해당 없음 |

---

## 총 항목 수

| 분류 | 항목 수 |
|------|---------|
| 필수 17항 + 보충 4항 | 21 |
| A~G (동시성~빌드) | 16 |
| H (포인터/메모리) | 20 |
| I~K (제어흐름~Cortex-M4) | 11 |
| L~M (가독성~정적분석) | 23 |
| N (동시성) | 15 |
| O (입력검증/보안) | 15 |
| P (런타임/성능) | 15 |
| Q (프로젝트 구조) | 15 |
| R (보안부팅/펌웨어) | 30 |
| U (구조적 안전성) | 4 |
| V (모듈별 특수취약점) | 35 |
| W (수석 아키텍트 규약) | 4 |
| X (추가 정밀검사) | 20 |
| **합계** | **244** |

pend검사하여 문제없게 한다. 


