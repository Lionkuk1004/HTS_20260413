// =========================================================================
// HTS_Hardware_Shield.h
// B-CDMA 하드웨어 보안 실드 — 물리 계층 보호 인터페이스
// Target: STM32F407 (Cortex-M4, 168MHz)
//
// ─────────────────────────────────────────────────────────────────────────
//  외주 업체 통합 가이드
// ─────────────────────────────────────────────────────────────────────────
//
//  [1] Lock()
//      B-CDMA PHY 레지스터 쓰기 금지 잠금.
//      세션 설립 완료 후 호출하여 런타임 중 PHY 설정 변경을 차단합니다.
//      AMI 보드 커스텀 레지스터 주소는 .cpp 내부에 은닉되어 있으므로,
//      보드 변경 시 .cpp만 수정하면 됩니다.
//
//  [2] Execute_Tensor_Decoherence_Shredding(uint32_t* node)
//      보안 위반 감지 시 단일 워드(4바이트) 물리적 파쇄. TRNG+DWT 틱 1회.
//
//  [2b] Execute_Tensor_Decoherence_Shredding_Range(uint32_t* first, size_t word_count)
//      연속 버퍼 전체 파쇄. 하드웨어 TRNG(Extract_Quantum_Seed)는 1회만 호출하고,
//      워드별 패턴은 주소·인덱스 기반 결정적 혼합으로 분기 — 대량 루프에서
//      워드마다 TRNG를 호출하지 않아 MAC/PHY ISR 기아를 방지합니다.
//      대량 텐서 소거 시 반드시 본 API를 사용하십시오 (단일 워드 API를 N회 호출 금지).
//
//  [3] Get_Hardware_Clock()
//      CPU 물리 사이클 카운터를 반환합니다.
//      ARM: DWT CYCCNT (32비트, 168MHz ≈ 25.56초 래핑)
//      PC:  TSC (64비트, 래핑 거의 없음)
//      내부적으로 Hardware_Bridge::Get_Physical_CPU_Tick()에 위임합니다.
//
// ─────────────────────────────────────────────────────────────────────────
#pragma once

#include <cstdint>
#include <cstddef>

namespace ProtectedEngine {

    class Hardware_Shield {
    public:
        /// B-CDMA PHY 레지스터 쓰기 잠금 (세션 설립 후 호출)
        static void Lock() noexcept;

        /// 보안 위반 시 텐서 요소 물리적 파쇄 (3단계 덮어쓰기 + 소거)
        /// @param node  파쇄 대상 uint32_t 포인터 (nullptr 안전)
        /// @post  *node == 0 (파쇄 완료)
        static void Execute_Tensor_Decoherence_Shredding(uint32_t* node) noexcept;

        /// 연속 uint32_t 버퍼 물리적 파쇄 — TRNG 1회만 (실시간 경로용)
        /// @param first      버퍼 시작 (nullptr 또는 word_count==0 이면 no-op)
        /// @param word_count 워드 개수
        /// @post  [first, first+word_count) 가 0으로 소거
        static void Execute_Tensor_Decoherence_Shredding_Range(
            uint32_t* first, size_t word_count) noexcept;

        /// CPU 물리 사이클 카운터 반환
        /// @return ARM: DWT CYCCNT (하위 32비트) / PC: TSC (64비트)
        static uint64_t Get_Hardware_Clock() noexcept;
    };

} // namespace ProtectedEngine