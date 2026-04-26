// =========================================================================
/// @file  HTS_Dynamic_Config.h
/// @brief 시스템 체급(Tier)별 파라미터 프로파일
/// @target STM32F407 (Cortex-M4) / PC / Server
// =========================================================================
#pragma once
#include <cstdint>
#include "HTS_CXX17_Atomic_Safe.h"
namespace ProtectedEngine {

    enum class HTS_Phy_Tier : uint8_t {
        TIER_32_IQ = 0u,
        TIER_64_ECCM = 1u,
    };

    struct HTS_Phy_Config {
        uint32_t noise_floor_init_q16 = 0u;
        uint32_t calib_frames = 0u;
        int32_t  kp = 0;
        int32_t  ki = 0;
        uint32_t jamming_margin = 0u;
        int32_t  squelch_threshold = 0;
        int32_t  cfar_default_mult = 0;
        uint8_t  chip_count = 32u;
        uint8_t  min_valid_chips = 16u;
        uint8_t  reserved[2] = { 0u, 0u };
    };

    static_assert(sizeof(HTS_Phy_Config) == 32u, "HTS_Phy_Config must be exactly 32 bytes");

    struct HTS_Phy_Config_Factory {
        [[nodiscard]]
        static HTS_Phy_Config make(HTS_Phy_Tier tier) noexcept;
    };

    enum class HTS_Sys_Tier : uint8_t {
        EMBEDDED_MINI = 0u,
        STANDARD_CHIP = 1u,
        WORKSTATION = 2u,
        HYPER_SERVER = 3u
    };

    struct HTS_Sys_Config {
        uint32_t node_count = 0u;
        uint32_t vdf_iterations = 0u;
        uint32_t temporal_slice_chunk = 0u;
        uint8_t  anchor_ratio_percent = 5u;
        uint8_t  reserved[3] = { 0u, 0u, 0u };
    };

    static_assert(sizeof(HTS_Sys_Config) == 16u, "HTS_Sys_Config must be exactly 16 bytes");

    class HTS_Sys_Config_Factory {
    private:
        static std::atomic<uint8_t> g_admin_ram_ratio;

    public:
        static void Override_RAM_Ratio(uint8_t ratio_percent) noexcept;

        [[nodiscard]]
        static HTS_Sys_Config Get_Tier_Profile(HTS_Sys_Tier tier) noexcept;
    };

} // namespace ProtectedEngine