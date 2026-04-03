// =========================================================================
// HTS_Orbital_Mapper.hpp
// 파울리 배타 원리 기반 LCM 2D 직교 인터리버 + 오비탈 텐서 폴딩
// Target: STM32F407 (Cortex-M4)
//
// [Fail-Closed 보안 설계]
//  Apply_Orbital_Clouding / Reverse_Orbital_Collapse:
//    OOM 시 텐서 전체를 보안 소거하여 평문 유출 원천 차단
//  Generate_Pauli_State_Map:
//    OOM 시 빈 벡터 반환 → 호출부에서 size 불일치로 자동 거부
// =========================================================================
#pragma once

#include <cstdint>
#include <cstddef>

#if (defined(__arm__) || defined(__TARGET_ARCH_ARM) || defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)) && !defined(__aarch64__)
#define HTS_ORBITAL_MAPPER_ARM 1
#else
#define HTS_ORBITAL_MAPPER_ARM 0
#endif

#if (HTS_ORBITAL_MAPPER_ARM == 0)
#include <vector>
#endif

namespace ProtectedEngine {

    class Orbital_Mapper {
    public:
#if (HTS_ORBITAL_MAPPER_ARM == 0)
        /// @brief LCM 2D 직교 인터리버 상태 지도 생성
        /// @return 상태 지도 (OOM 시 빈 벡터 → 호출부 size 불일치로 거부)
        static std::vector<uint32_t> Generate_Pauli_State_Map(
            size_t tensor_size, uint64_t pqc_session_id) noexcept;

        // ── vector API (기존 호환) ──────────────────────────────

        /// @brief 오비탈 텐서 폴딩 (정방향 인터리빙)
        /// @note  OOM 시 Fail-Closed: 텐서 전체 보안 소거
        static void Apply_Orbital_Clouding(
            std::vector<uint32_t>& tensor,
            const std::vector<uint32_t>& state_map) noexcept;

        /// @brief 오비탈 파동 함수 수렴 (역방향 디인터리빙)
        /// @note  OOM 시 Fail-Closed: 텐서 전체 보안 소거
        static void Reverse_Orbital_Collapse(
            std::vector<uint32_t>& tensor,
            const std::vector<uint32_t>& state_map) noexcept;
#endif

        // ── raw 포인터 API (BB1 정적 배열용) ───────────────────
        //
        // vector 래핑 없이 직접 호출. inplace_scatter/gather 동일 사용.
        // 실패 시 텐서를 0으로 보안 소거 (Fail-Closed).

        /// @brief 정방향 인터리빙 (raw 포인터)
        /// @param tensor     텐서 배열 (in-place 변환)
        /// @param t_size     텐서 원소 수
        /// @param state_map  상태 지도 배열
        /// @param m_size     상태 지도 원소 수 (t_size와 동일해야 함)
        static void Apply_Orbital_Clouding(
            uint32_t* tensor, size_t t_size,
            const uint32_t* state_map, size_t m_size) noexcept;

        /// @brief 역방향 디인터리빙 (raw 포인터)
        static void Reverse_Orbital_Collapse(
            uint32_t* tensor, size_t t_size,
            const uint32_t* state_map, size_t m_size) noexcept;
    };

} // namespace ProtectedEngine