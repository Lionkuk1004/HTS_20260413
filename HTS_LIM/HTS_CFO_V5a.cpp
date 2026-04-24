// =============================================================================
// HTS_CFO_V5a.cpp — INNOViD HTS Rx CFO V5a (Phase 1-1 stub implementation)
// =============================================================================
#include "HTS_CFO_V5a.hpp"
#include "HTS_Rx_CFO_SinCos_Table.hpp"

namespace hts {
namespace rx_cfo {

CFO_V5a::CFO_V5a() noexcept
    : last_cfo_hz_(0),
      runtime_enabled_(false) {
    for (int i = 0; i < kPreambleChips; ++i) {
        work_I_[i] = 0;
        work_Q_[i] = 0;
    }
    for (int j = 0; j < 32; ++j) {
        fine_energies_[j] = 0;
    }
}

void CFO_V5a::Init() noexcept {
    Build_SinCos_Table();
    last_cfo_hz_ = 0;
    runtime_enabled_ = (HTS_CFO_V5A_ENABLE != 0);
}

CFO_Result CFO_V5a::Estimate(const int16_t* rx_I,
                             const int16_t* rx_Q) noexcept {
    (void)rx_I;
    (void)rx_Q;
    CFO_Result res{};
    res.cfo_hz = 0;
    res.peak_energy = 0;
    res.valid = false;
    last_cfo_hz_ = 0;
    return res;
}

void CFO_V5a::ApplyDerotate(const int16_t* in_I, const int16_t* in_Q,
                            int16_t* out_I, int16_t* out_Q, int chips,
                            int32_t cfo_hz) noexcept {
    (void)cfo_hz;
    for (int k = 0; k < chips; ++k) {
        out_I[k] = in_I[k];
        out_Q[k] = in_Q[k];
    }
}

}  // namespace rx_cfo
}  // namespace hts
