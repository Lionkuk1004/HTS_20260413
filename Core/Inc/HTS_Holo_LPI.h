#pragma once
#include <cstdint>

namespace ProtectedEngine {

/// @brief 홀로그램 LPI 소프트 스칼라 생성 + 적용
/// @note  V400 Dispatcher Build_Packet 출력에 후처리로 적용
///        FWHT 혼합 + 4D 회전 → 가우시안 진폭 분포
///        Walsh PG 보존, 수신측 역스칼라로 복원
///
/// 연산: ~1000 사이클/블록 (Cortex-M4)
/// SRAM: 128 bytes (scalars[64])

class HTS_Holo_LPI final {
public:
    static constexpr uint32_t SECURE_TRUE  = 0x5A5A5A5Au;
    static constexpr uint32_t SECURE_FALSE = 0xA5A5A5A5u;

    /// 스칼라 생성 (64개, Q13)
    /// @param master_seed  128비트 시드 (TX/RX 공유)
    /// @param time_slot    프레임 번호 (TX/RX 동기)
    /// @param mix_ratio_q8 혼합 비율 Q8 (128=0.50, 0=원본유지)
    /// @param out_scalars  출력 [64] int16_t
    static uint32_t Generate_Scalars(
        const uint32_t master_seed[4],
        uint32_t time_slot,
        uint8_t mix_ratio_q8,
        int16_t* out_scalars) noexcept;

    /// V400 칩에 스칼라 적용 (in-place)
    /// @param chips_I/Q  V400 Build_Packet 출력
    /// @param chip_count 칩 수
    /// @param scalars    Generate_Scalars 출력 [64]
    static void Apply(
        int16_t* chips_I,
        int16_t* chips_Q,
        uint16_t chip_count,
        const int16_t* scalars) noexcept;

    HTS_Holo_LPI() = delete;
};

} // namespace ProtectedEngine
