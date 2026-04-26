#pragma once
/// @file  HTS_Holo_Tensor_4D_RX.h
/// @brief 4D 홀로 — RX 전용(독립 1024B impl + 시드/슬롯/프로파일)
#include "HTS_Holo_Tensor_4D_Defs.h"
#include "HTS_CXX17_Atomic_Safe.h"
#include <cstdint>

namespace ProtectedEngine {

    class HTS_Holo_Tensor_4D_RX final {
    public:
        static constexpr uint32_t SECURE_TRUE = 0x5A5A5A5Au;
        static constexpr uint32_t SECURE_FALSE = 0xA5A5A5A5u;
        static constexpr uint32_t IMPL_BUF_SIZE = 1024u;

        HTS_Holo_Tensor_4D_RX() noexcept;
        ~HTS_Holo_Tensor_4D_RX() noexcept;

        uint32_t Initialize(
            const uint32_t         master_seed[4],
            const HoloTensor_Profile* profile) noexcept;
        void Shutdown() noexcept;
        void Rotate_Seed(const uint32_t new_seed[4]) noexcept;
        [[nodiscard]] uint32_t Set_Profile(
            const HoloTensor_Profile* profile) noexcept;

        uint32_t Decode_Block(
            const int16_t*        rx_chips,
            uint16_t                N,
            uint64_t                valid_mask,
            int8_t*                 output_bits,
            uint16_t                K) noexcept;

        /// CRC 검증 콜백 — bits[k] ∈ {+1,-1}, K 비트; true=PASS
        using CrcVerifyFn = bool (*)(const int8_t* bits, uint16_t K, void* ctx);

        /// I/Q → (선택) derotate → (I'+Q')/2 → Walsh; φ 및 φ+π 후보를 동시 반환.
        /// 패킷 단위 CRC는 호출측(Dispatcher)에서 조립 후 검증.
        uint32_t Decode_Block_Two_Candidates(
            const int16_t*        rx_I,
            const int16_t*        rx_Q,
            uint16_t                N,
            uint64_t                valid_mask,
            int8_t*                 output_bits_cand0,
            int8_t*                 output_bits_cand1,
            uint16_t                K) noexcept;

        /// Two_Candidates 후 CRC 콜백으로 단일 출력 선택(레거시·단위 테스트용).
        uint32_t Decode_Block_With_Phase(
            const int16_t*        rx_I,
            const int16_t*        rx_Q,
            uint16_t                N,
            uint64_t                valid_mask,
            int8_t*                 output_bits,
            uint16_t                K,
            CrcVerifyFn             crc_fn,
            void*                   crc_ctx) noexcept;

        [[nodiscard]] uint32_t Advance_Time_Slot() noexcept;
        [[nodiscard]] uint32_t Set_Time_Slot(uint32_t frame_no) noexcept;

        HoloState Get_State() const noexcept;
        uint32_t Get_Decode_Count() const noexcept;
        uint32_t Get_Time_Slot() const noexcept;

        HTS_Holo_Tensor_4D_RX(const HTS_Holo_Tensor_4D_RX&) = delete;
        HTS_Holo_Tensor_4D_RX& operator=(const HTS_Holo_Tensor_4D_RX&) = delete;
        HTS_Holo_Tensor_4D_RX(HTS_Holo_Tensor_4D_RX&&) = delete;
        HTS_Holo_Tensor_4D_RX& operator=(HTS_Holo_Tensor_4D_RX&&) = delete;

    private:
        struct Impl;
        friend uint32_t detail_holo4d_decode_two_candidates(
            HTS_Holo_Tensor_4D_RX& o,
            Impl* im,
            const int16_t* rx_I,
            const int16_t* rx_Q,
            uint16_t N,
            uint64_t valid_mask,
            int8_t* output_bits_cand0,
            int8_t* output_bits_cand1,
            uint16_t K) noexcept;
        alignas(4) uint8_t  impl_buf_[IMPL_BUF_SIZE];
        std::atomic<bool>  initialized_{ false };
        mutable std::atomic_flag op_busy_ = ATOMIC_FLAG_INIT;
        uint32_t             master_seed_[4];
        uint32_t             time_slot_;
        HoloTensor_Profile  profile_{};
    };

    static_assert(sizeof(HTS_Holo_Tensor_4D_RX) <= 2048u,
        "HTS_Holo_Tensor_4D_RX exceeds 2KB SRAM budget");

} // namespace ProtectedEngine
