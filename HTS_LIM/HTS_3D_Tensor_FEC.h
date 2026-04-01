#pragma once
// =========================================================================
// HTS_3D_Tensor_FEC.h
// B-CDMA DIOC 항재밍 코어 + LTE HARQ 시뮬레이션 — 공개 인터페이스
// Target: STM32F407 (Cortex-M4, 168MHz, SRAM 192KB) / PC
//
// ─────────────────────────────────────────────────────────────────────────
//  외주 업체 통합 가이드
// ─────────────────────────────────────────────────────────────────────────
//
//  이 파일은 2개 계층으로 구성됩니다:
//
//  [Layer 1] ProtectedEngine::HTS16_DIOC_Core
//   — Dual Independent Optimal Code + OS-CFAR 물리 계층 항재밍 코어
//   — PSL=4 최적 균형 코드북 64개 + ARX128 CSPRNG
//   — I/Q 독립 코드 배정 → 가짜 피크 확률 제곱 감소
//   — ⚠ 현재 .cpp는 PC 시뮬레이션 전용 (ARM 빌드 제외)
//   — ARM 실장: HTS64_Native_ECCM_Core 참조
//
//  [Layer 2] HTS_Engine (PC 시뮬레이션 전용)
//   — LTE HARQ 시뮬레이션, Text Codec, CRC16, Tensor FEC
//   — ARM 빌드에서는 이 네임스페이스를 사용하지 마십시오  
//   — #if !defined(HTS_3D_ARM_EXCLUDE) 가드로 ARM 노출 차단
//
//  [사용법 — HTS16_DIOC_Core]       
//   1. 생성: HTS16_DIOC_Core(seed) — seed=0 시 0xDEADBEEF 대체
//   2. 송신: Transmit_4Bit(data_4bit) → SparseChip[16] I/Q 독립 극성
//   3. 수신: Decode_4Bit(rx_I, rx_Q) → 복호된 4비트 (실패 시 -1)
//   4. TX/RX 간 PRNG 동기: 동일 seed + 동일 호출 순서 필수
//
//  [메모리 요구량]
//   sizeof(HTS16_DIOC_Core) ≈ 264B (impl_buf_[256] + impl_valid_ + padding)
//   Impl(SRAM In-Place): arx_state[4] = 16B — placement new, 힙 할당 0회
//       
//  [보안 설계]
//   arx_state: Impl 소멸자에서 보안 소거 (128비트 PRNG 키 잔존 방지)
//   impl_buf_: 소멸자에서 SecWipe — Impl 전체 이중 소거
//   복사/이동: = delete (PRNG 상태 복제 원천 차단)
//
//  [양산 수정 이력 — 세션 5: 10건 + BUG-15]
//   BUG-01~09 (arx_state 소거, Pimpl, 헤더 분리, copy/move,
//             noexcept, rotl32 가드, Self-Contained, nodiscard, 이중 가드)
//   BUG-10 (Cognitive CFAR 이중 모드 — 광대역 재밍 통신 생존)
//   BUG-11~14 (나눗셈 제거, SecWipe, idx_Q, Soft_Tensor_FEC 방어)
//   BUG-15 [CRIT] unique_ptr + make_unique + try-catch → placement new
//          · impl_buf_[256] alignas(8) — uint32_t[4] 정렬 수용
//          · make_unique 예외 경로 및 try-catch 완전 제거
//          · 힙 OOM 위험 원천 제거 / 결정론적 SRAM 배치 보장
//
// ─────────────────────────────────────────────────────────────────────────
#pragma once

#include <cstdint>
#include <cstddef>
#include <array>

// =========================================================================
//  [Layer 2] HTS_Engine — PC 시뮬레이션 전용
//  [BUG-03] ARM 빌드에서 <random>, <string> 등 STL 헤더 전파 차단
// =========================================================================
#if !defined(__arm__) && !defined(__TARGET_ARCH_ARM) && \
    !defined(__TARGET_ARCH_THUMB) && !defined(__ARM_ARCH)

#include <string>
#include <random>

namespace HTS_Engine {

    // ── [BUG-22] PC 시뮬: std::vector 제거, float + 파일 정적(BSS) 버퍼 ──
    static constexpr size_t TEXT_CODEC_MAX_CHARS = 4096u;
    static constexpr size_t TEXT_CODEC_MAX_BITS = TEXT_CODEC_MAX_CHARS * 8u;

    class Text_Codec {
    public:
        /// 내부 정적 버퍼에 기록(재진입 불가). 반환: 비트 개수.
        static size_t String_To_Bits(const std::string& text);
        static const float* String_To_Bits_Data() noexcept;
        static size_t String_To_Bits_Count() noexcept;
        static std::string Bits_To_String(const float* bits, size_t num_bits);
    };

    class CRC16 {
    public:
        /// out에 n+16 소프트 비트 기록. out_cap >= n+16 필수. 반환: n+16 또는 0.
        static size_t Append(const float* data, size_t n,
            float* out, size_t out_cap);
        static bool Check(const float* data_with_crc, size_t len);
    private:
        static uint16_t compute(const float* data, size_t n);
    };

    class Tensor_Interleaver {
    public:
        explicit Tensor_Interleaver(size_t dim = 64);
        size_t Get_Size() const;
        /// out_cap >= Get_Size() 권장. 실제 기록은 min(in_len,total_size) 등.
        void Interleave_To(const float* in, size_t in_len,
            float* out, size_t out_cap) const;
        void Deinterleave_To(const float* in, size_t in_len,
            float* out, size_t out_cap) const;

        // ── [BUG-21] ARM Raw API (int8_t, zero-heap) ──
        size_t Interleave_Raw(const int8_t* in, size_t in_len,
            int8_t* out, size_t out_max) const noexcept;
        size_t Deinterleave_Raw(const int8_t* in, size_t in_len,
            int8_t* out, size_t out_max) const noexcept;
    private:
        size_t dim;
        size_t total_size;
    };

    class Soft_Tensor_FEC {
    public:
        static constexpr size_t TENSOR_CAPACITY = 262144u;

        // ── [BUG-21] ARM Raw API (int8_t ±1, zero-heap) ──
        size_t Encode_Raw(const int8_t* bits, size_t n_bits,
            int8_t* out, size_t out_max,
            unsigned int frame_seed) const noexcept;

        /// out_cap ≥ TENSOR_CAPACITY 권장 (PC 시뮬). float 소프트 비트.
        void Encode_To(const float* bits, size_t n_bits,
            unsigned int frame_seed,
            float* out, size_t out_cap) const;
        void Decode_Soft_To(const float* tensor, size_t tensor_len,
            size_t num_bits, unsigned int frame_seed,
            float* out, size_t out_cap) const;

    private:
        static constexpr size_t TENSOR_SIZE = TENSOR_CAPACITY;
        static float Tag(unsigned int seed, size_t index) noexcept;
        static int8_t Tag_i8(unsigned int seed, size_t index) noexcept;
    };

    class LTE_Channel {
    public:
        static constexpr float JS_DB = 50.0f;
        static constexpr int   NUM_CHIPS = 128;
        static constexpr float EMP_RATE = 0.03f;
        static constexpr float EMP_AMP = 99999.0f;
        static void Transmit_To(const float* tensor, size_t N,
            std::mt19937& rng, float* out, size_t out_cap);
    };

    class LTE_HARQ_Controller {
    public:
        static constexpr int    MTU = 512;
        static constexpr int    CRC_BITS = 16;
        static constexpr int    REP_FACTOR = 5;
        static constexpr int    MAX_HARQ = 12;
        static constexpr float  HARQ_RTT_MS = 8.0f;
        static constexpr int    PROTECTED_BITS = MTU / REP_FACTOR;
        static constexpr int    INFO_PER_BLOCK = PROTECTED_BITS - CRC_BITS;

        struct BlockResult {
            std::array<float, static_cast<size_t>(INFO_PER_BLOCK)> info_bits{};
            int    harq_rounds = 0;
            bool   success = false;
            float  latency_ms = 0.0f;
        };

        /// info_len ≤ INFO_PER_BLOCK. 내부 파이프라인은 파일 정적 버퍼 사용.
        static BlockResult TransmitBlock(
            const float* info_bits, size_t info_len,
            unsigned int block_seed,
            Soft_Tensor_FEC& fec,
            Tensor_Interleaver& interleaver,
            std::mt19937& rng,
            int block_id);
    };

    void Print_LTE_Analysis();

} // namespace HTS_Engine

#endif // !ARM — Layer 2 PC 전용 끝

// =========================================================================
//  [Layer 1] ProtectedEngine::HTS16_DIOC_Core
//  Dual Independent Optimal Code + OS-CFAR 물리 계층 항재밍 코어
// =========================================================================
namespace ProtectedEngine {

    class HTS16_DIOC_Core {
    public:
        /// @brief DIOC 코어 생성 (128비트 ARX PRNG 초기화)
        /// @param seed  32비트 시드 (0 시 0xDEADBEEF 대체)
        /// @note  TX/RX 동일 seed 필수 (PRNG 동기)
        explicit HTS16_DIOC_Core(uint32_t seed) noexcept;

        /// @brief 소멸자 — arx_state 보안 소거 후 impl_buf_ SecWipe
        ~HTS16_DIOC_Core() noexcept;

        /// PRNG 상태 복제 원천 차단
        HTS16_DIOC_Core(const HTS16_DIOC_Core&) = delete;
        HTS16_DIOC_Core& operator=(const HTS16_DIOC_Core&) = delete;
        HTS16_DIOC_Core(HTS16_DIOC_Core&&) = delete;
        HTS16_DIOC_Core& operator=(HTS16_DIOC_Core&&) = delete;

        struct SparseChip {
            uint16_t slot_index;
            int8_t   polarity_I;  ///< I채널 극성 (+1/-1)
            int8_t   polarity_Q;  ///< Q채널 극성 (+1/-1, I와 독립)
        };

        /// @brief DIOC 4비트 송신 (I/Q 독립 최적 코드 배정)
        /// @param data_4bit  4비트 데이터 (0~15)
        /// @return 16칩 SparseChip 배열 (슬롯 인덱스 + I/Q 극성)
        [[nodiscard]]
        std::array<SparseChip, 16> Transmit_4Bit(
            uint8_t data_4bit) noexcept;

        /// @brief DIOC 4비트 수신 (OS-CFAR + 16-회전 상관기)
        /// @param rx_universe_I  I채널 1024슬롯 수신 배열 (nullptr 불가)
        /// @param rx_universe_Q  Q채널 1024슬롯 수신 배열 (nullptr 불가)
        /// @return 0~15: 복호 성공, -1: 유효 칩 부족 (재밍 과다)
        [[nodiscard]]
        int16_t Decode_4Bit(
            const int16_t* rx_universe_I,
            const int16_t* rx_universe_Q) noexcept;

    private:
        // ── [BUG-15] Pimpl In-Place Storage (zero-heap) ──────────────
        // Impl = arx_state[4](16B) + static methods (데이터 없음)
        // alignof(Impl) = 4 (uint32_t) → alignas(8) 안전 초과 정렬
        static constexpr size_t IMPL_BUF_SIZE = 256u;
        static constexpr size_t IMPL_BUF_ALIGN = 8u;

        struct Impl;  ///< ARX PRNG + 코드북 인덱스 은닉 (ABI 안정성 보장)

        alignas(IMPL_BUF_ALIGN) uint8_t impl_buf_[IMPL_BUF_SIZE];
        bool impl_valid_ = false;  ///< placement new 성공 여부

        /// @brief impl_buf_에서 Impl 포인터 반환 (컴파일 타임 크기·정렬 검증 포함)
        Impl* get_impl() noexcept;
        /// @overload
        const Impl* get_impl() const noexcept;
    };

} // namespace ProtectedEngine