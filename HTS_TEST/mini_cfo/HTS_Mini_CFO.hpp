// HTS_Mini_CFO.hpp — 격리 미니 CFO 테스트 모듈
// 양산 HTS_CFO_V5a 와 링크 분리. 본진 격리·비교용.
#pragma once

#include <cstdint>

namespace hts {
namespace mini_cfo {

inline constexpr int kMiniChipRateHz = 1000000;
inline constexpr int kMiniSegSize = 16;
inline constexpr int kMiniNumSeg = 8;
inline constexpr int kMiniPreambleChips = 128;

class Mini_CFO {
public:
    Mini_CFO() noexcept;

    void Init() noexcept;

    /// 영준님 정통 L&R (M-1 인접 lag 합산, 세그먼트는 칩 합·Walsh 없음)
    int32_t Estimate(const int16_t* rx_I, const int16_t* rx_Q) noexcept;

    /// chip별 e^(-j·ω·n) 보정 (double, std::sin/cos)
    void Apply_Per_Chip(const int16_t* in_I, const int16_t* in_Q, int16_t* out_I,
                        int16_t* out_Q, int n_chips, int32_t cfo_hz) noexcept;

    int32_t GetLastCfoHz() const noexcept { return last_cfo_hz_; }

private:
    int32_t last_cfo_hz_;
};

}  // namespace mini_cfo
}  // namespace hts
