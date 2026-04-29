#pragma once
/// @file  HTS_Holo_Tensor_4D_Common.h
/// @brief 4D 홀로 — Walsh ROM(단일 copy), PRNG/순열/위상 마스크, CFI(공용 논리).
///        TX/RX 각자 impl_buf(1024B) + 시드/슬롯/프로파일 — 엔진 단일화 없음.
#include "HTS_Holo_Tensor_4D_Defs.h"
#include <cstdint>

namespace ProtectedEngine {

    /// @brief 64×64 Walsh-Hadamard LUT — `Holo4D_Walsh_Code` 로만 접근 (Common TU 단일 constexpr ROM).
    int8_t Holo4D_Walsh_Code(uint32_t row, uint32_t col) noexcept;

    /// @brief 열/행(Fisher–Yates) + 분할 Walsh 행 선택(출력: row_workspace[0..L*K-1], col_perm[0..N-1]).
    void Holo4D_Generate_Partitioned_Params(
        const uint32_t       master_seed[4],
        uint32_t              t_slot,
        uint16_t*             row_workspace,
        uint16_t*             col_perm,
        uint16_t              K,
        uint16_t              N,
        uint8_t               L) noexcept;

    /// @brief 이진 위상 마스크 64비트(Encode/Decode 공통, 동일 master_seed+time).
    uint64_t Holo4D_Generate_Phase_Mask(
        const uint32_t        master_seed[4],
        uint32_t              time_slot) noexcept;

    /// @brief CFI (HOLO_4D) — HTS_Holo_Tensor_4D_*::SECURE_* 와 동일 수치.
    uint32_t Holo4D_Cfi_Transition(
        HoloState&            state,
        uint8_t&              cfi_violation_count,
        HoloState             target) noexcept;

    /// Cordic atan2(y,x) in Q16 radians per turn (π = 32768). Shared by 4D RX / sync ref.
    int32_t Holo4D_Atan2_Q16(int32_t y, int32_t x) noexcept;

} // namespace ProtectedEngine
