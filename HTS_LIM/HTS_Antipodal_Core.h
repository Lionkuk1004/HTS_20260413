// =========================================================================
/// @file  HTS_Antipodal_Core.h
/// @brief 안티포달 텐서 변환 유틸리티 (바이너리 ↔ ±1 변환 + 직교성 내적)
/// @target STM32F407 (Cortex-M4, 168MHz, SRAM 192KB) / PC
///
///  [메모리] 힙 할당 0, 스택 지역변수만 사용
///  [성능]  32비트 워드 병렬 처리 + SMLAD DSP 활용 + 브랜치리스 ALU
// =========================================================================
#pragma once
#include <cstdint>
#include <cstddef>

namespace ProtectedEngine {

    /// 안티포달 텐서 유틸리티 (인스턴스 불필요 — static 함수만)
    struct AntipodalTensor {
        AntipodalTensor() = delete;

        /// @brief 바이너리(0/1) → 안티포달(±1) 변환
        /// @note  수학 정확성 보장을 위해 바이트 단위 변환 경로를 고정 사용
        ///        (Parallel Subtraction 경로는 borrow 전파 오류로 제거됨)
        static void convertToAntipodal(const uint8_t* __restrict in,
            int8_t* __restrict out, size_t len) noexcept;

        /// @brief 안티포달 텐서 내적 (직교성 판별)
        /// @note  std::memcpy 기반 워드 로드로 정렬 여부와 무관하게
        ///        동일한 고속 경로를 사용합니다.
        [[nodiscard]]
        static int32_t calculateOrthogonality(const int8_t* __restrict a,
            const int8_t* __restrict b, size_t len) noexcept;
    };

} // namespace ProtectedEngine