#pragma once
/// @file  HTS_Holo_Tensor_4D_TX.h
/// @brief 4D 홀로 — TX 전용(독립 1024B impl + 시드/슬롯/프로파일)
#include "HTS_Holo_Tensor_4D_Defs.h"
#include <atomic>
#include <cstdint>

namespace ProtectedEngine {

    class HTS_Holo_Tensor_4D_TX final {
    public:
        static constexpr uint32_t SECURE_TRUE = 0x5A5A5A5Au;
        static constexpr uint32_t SECURE_FALSE = 0xA5A5A5A5u;
        static constexpr uint32_t IMPL_BUF_SIZE = 1024u;

        HTS_Holo_Tensor_4D_TX() noexcept;
        ~HTS_Holo_Tensor_4D_TX() noexcept;

        uint32_t Initialize(
            const uint32_t         master_seed[4],
            const HoloTensor_Profile* profile) noexcept;
        void Shutdown() noexcept;
        void Rotate_Seed(const uint32_t new_seed[4]) noexcept;
        [[nodiscard]] uint32_t Set_Profile(
            const HoloTensor_Profile* profile) noexcept;

        uint32_t Encode_Block(
            const int8_t*         data_bits,
            uint16_t                K,
            int8_t*                 output_chips,
            uint16_t                N) noexcept;

        [[nodiscard]] uint32_t Advance_Time_Slot() noexcept;
        [[nodiscard]] uint32_t Set_Time_Slot(uint32_t frame_no) noexcept;

        HoloState Get_State() const noexcept;
        uint32_t Get_Encode_Count() const noexcept;
        uint32_t Get_Time_Slot() const noexcept;

        HTS_Holo_Tensor_4D_TX(const HTS_Holo_Tensor_4D_TX&) = delete;
        HTS_Holo_Tensor_4D_TX& operator=(const HTS_Holo_Tensor_4D_TX&) = delete;
        HTS_Holo_Tensor_4D_TX(HTS_Holo_Tensor_4D_TX&&) = delete;
        HTS_Holo_Tensor_4D_TX& operator=(HTS_Holo_Tensor_4D_TX&&) = delete;

    private:
        struct Impl;
        alignas(4) uint8_t  impl_buf_[IMPL_BUF_SIZE];
        std::atomic<bool>  initialized_{ false };
        mutable std::atomic_flag op_busy_ = ATOMIC_FLAG_INIT;
        uint32_t             master_seed_[4];
        uint32_t             time_slot_;
        HoloTensor_Profile  profile_{};
    };

    static_assert(sizeof(HTS_Holo_Tensor_4D_TX) <= 2048u,
        "HTS_Holo_Tensor_4D_TX exceeds 2KB SRAM budget");

} // namespace ProtectedEngine
