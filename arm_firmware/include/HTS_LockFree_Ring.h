// HTS/3DPOWNS Phase 2 — SPSC 바이트 링 (Mutex 없음, std::atomic + 명시적 memory_order).
// 용량은 2의 거듭제곱; 인덱스는 마스크( &) 로 래핑 — 나눗셈/모듈로 미사용.
#pragma once

#include "../../HTS_LIM/HTS_CXX17_Atomic_Safe.h"
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace HTS_LockFree {

/// 단일 생산자·단일 소비자(SPSC) 바이트 링. ISR ↔ 메인루프 한쪽이 생산·다른 쪽이 소비일 때 사용.
/// 다중 생산자가 필요하면 별도 슬롯 큐 + CAS 확장.
template <std::uint32_t CapacityPow2>
class ByteRingSpsc final
{
    static_assert((CapacityPow2 & (CapacityPow2 - 1u)) == 0u && CapacityPow2 >= 2u,
                  "CapacityPow2 must be power of 2 and >= 2");
    static constexpr std::uint32_t kMask = CapacityPow2 - 1u;

public:
    ByteRingSpsc() noexcept = default;
    ByteRingSpsc(const ByteRingSpsc&) = delete;
    ByteRingSpsc& operator=(const ByteRingSpsc&) = delete;

    /// 연속 구간 복사 push. 실패 시 false (공간 부족).
    [[nodiscard]] bool TryPush(const std::uint8_t* data, std::uint32_t len) noexcept
    {
        if (data == nullptr || len == 0u) {
            return false;
        }
        const std::uint32_t h = head_.load(std::memory_order_relaxed);
        const std::uint32_t t = tail_.load(std::memory_order_acquire);
        const std::uint32_t used = h - t;
        // 한 슬롯 비워 full/empty 모호성 제거 (최대 저장 바이트 = CapacityPow2 - 1)
        if (used + len >= CapacityPow2) {
            return false;
        }
        std::uint32_t pos = h & kMask;
        const std::uint32_t til_end = CapacityPow2 - pos;
        std::uint32_t first = len;
        if (first > til_end) {
            first = til_end;
        }
        std::memcpy(buf_ + pos, data, static_cast<std::size_t>(first));
        const std::uint32_t second = len - first;
        if (second != 0u) {
            std::memcpy(buf_, data + first, static_cast<std::size_t>(second));
        }
        head_.store(h + len, std::memory_order_release);
        return true;
    }

    /// 최대 max_len 바이트를 dst로 복사. 반환: 실제 복사한 길이.
    [[nodiscard]] std::uint32_t TryPop(std::uint8_t* dst, std::uint32_t max_len) noexcept
    {
        if (dst == nullptr || max_len == 0u) {
            return 0u;
        }
        const std::uint32_t t = tail_.load(std::memory_order_relaxed);
        const std::uint32_t h = head_.load(std::memory_order_acquire);
        const std::uint32_t used = h - t;
        if (used == 0u) {
            return 0u;
        }
        std::uint32_t len = used;
        if (len > max_len) {
            len = max_len;
        }
        std::uint32_t pos = t & kMask;
        const std::uint32_t til_end = CapacityPow2 - pos;
        std::uint32_t first = len;
        if (first > til_end) {
            first = til_end;
        }
        std::memcpy(dst, buf_ + pos, static_cast<std::size_t>(first));
        const std::uint32_t second = len - first;
        if (second != 0u) {
            std::memcpy(dst + first, buf_, static_cast<std::size_t>(second));
        }
        tail_.store(t + len, std::memory_order_release);
        return len;
    }

    [[nodiscard]] std::uint32_t AvailableBytes() const noexcept
    {
        const std::uint32_t h = head_.load(std::memory_order_acquire);
        const std::uint32_t t = tail_.load(std::memory_order_acquire);
        return h - t;
    }

private:
    alignas(4) std::uint8_t buf_[CapacityPow2]{};
    std::atomic<std::uint32_t> head_{0u};
    std::atomic<std::uint32_t> tail_{0u};
};

} // namespace HTS_LockFree
