// =========================================================================
// HTS_Quantum_Decoy_VDF.h
// 양자 디코이 VDF (Verifiable Delay Function) 기반 텐서 난독화
// Target: STM32F407 (Cortex-M4, 168MHz)
//
// [설계 목적]
//  VDF: 의도적으로 순차 연산만 가능한 지연 함수
//    → 적군이 FPGA/GPU 병렬화로 키를 사전 계산하는 것을 차단
//    → 64비트 상태: 2^64 Brute-force 불가 (GPU 1억년+)
//
// [양산 주의]
//  · 대량 elements + 기본 iteration(50000)은 장시간 CPU 점유 → ISR/WDG 컨텍스트·
//    PRIMASK 장시간 구간에서 호출 금지.
//  · Apply/Reverse: 동일 session_id로 XOR 마스크가 대칭(자기역)임.
// =========================================================================
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#define HTS_QUANTUM_DECOY_ARM 1
#else
#define HTS_QUANTUM_DECOY_ARM 0
#endif

namespace ProtectedEngine {

    class Quantum_Decoy_VDF {
    private:
        static constexpr uint32_t QUANTUM_NOISE_SEED = 0x5C4E3D2FUL;

        /// 요소 단위 XOR (memcpy — strict aliasing 회피, LTO 안전)
        template <typename T>
        static void xor_element_inplace(T& v, uint64_t mix) noexcept {
            static_assert(sizeof(T) <= 32u,
                "tensor element too large for stack xor buffer");
            alignas(T) unsigned char buf[sizeof(T)];
            std::memcpy(buf, &v, sizeof(T));
            for (size_t b = 0u; b < sizeof(T); ++b) {
                const unsigned shift = static_cast<unsigned>(b & 7u) * 8u;
                buf[b] ^= static_cast<unsigned char>((mix >> shift) & 0xFFu);
            }
            std::memcpy(&v, buf, sizeof(T));
        }

    public:
        /// @brief VDF 코어: 64비트 순차 해시 시간 잠금 퍼즐
        /// @return 64비트 VDF 출력 (2^64 Brute-force 내성)
        [[nodiscard]]
        static uint64_t Execute_Time_Lock_Puzzle(
            uint64_t session_id,
            uint32_t iterations = 50000u) noexcept;

        template <typename T>
        static void Apply_Quantum_Decoy(
            T* tensor_data,
            size_t elements,
            uint64_t true_session_id) noexcept;

        template <typename T>
        static void Reverse_Quantum_Decoy(
            T* damaged_tensor,
            size_t elements,
            uint64_t input_session_id) noexcept;
    };

    // -----------------------------------------------------------------
    //  템플릿 정의 (헤더 인라인 — ODR)
    // -----------------------------------------------------------------
    template <typename T>
    inline void Quantum_Decoy_VDF::Apply_Quantum_Decoy(
        T* tensor_data,
        size_t elements,
        uint64_t true_session_id) noexcept {

#if HTS_QUANTUM_DECOY_ARM
        static_assert(
            !std::is_same<typename std::remove_cv<T>::type, double>::value,
            "B-CDMA: ARM 경로 double 텐서 요소 금지");
#endif
        if (tensor_data == nullptr || elements == 0u) {
            return;
        }

        constexpr uint32_t kDefaultIterations = 50000u;
        const uint64_t lock = Execute_Time_Lock_Puzzle(true_session_id, kDefaultIterations);

        for (size_t i = 0u; i < elements; ++i) {
            uint64_t mix = lock ^ static_cast<uint64_t>(i) ^ true_session_id;
            mix ^= static_cast<uint64_t>(QUANTUM_NOISE_SEED);
            xor_element_inplace(tensor_data[i], mix);
        }
        std::atomic_thread_fence(std::memory_order_release);
    }

    template <typename T>
    inline void Quantum_Decoy_VDF::Reverse_Quantum_Decoy(
        T* damaged_tensor,
        size_t elements,
        uint64_t input_session_id) noexcept {
        // XOR 자기역: Apply 와 동일 세션 ID 로 재적용하면 원복
        Apply_Quantum_Decoy(damaged_tensor, elements, input_session_id);
    }

} // namespace ProtectedEngine
