// HTS_V400_Dispatcher_Gravity.cpp
// 6면 Gravity Cube 구현 - DPTE 적용
// G-3-1: skeleton (always reject)
// G-3-2: 실제 알고리즘 이식

#include "HTS_V400_Dispatcher_Gravity.hpp"

#if defined(HTS_USE_GRAVITY) && HTS_USE_GRAVITY

namespace HTS_LIM {
namespace detail_gravity {

bool gravity_evaluate_cube(
    const int16_t* rx_pre_I,
    const int16_t* rx_pre_Q,
    const int16_t* tx_pre_tmpl_I,
    GravityCube6* out_cube) noexcept {
    if (rx_pre_I == nullptr || rx_pre_Q == nullptr ||
        tx_pre_tmpl_I == nullptr || out_cube == nullptr) {
        return false;
    }

    // G-3-1: skeleton (always reject)
    *out_cube = GravityCube6{};

    return false;
}

} // namespace detail_gravity
} // namespace HTS_LIM

#endif // HTS_USE_GRAVITY
