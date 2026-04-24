// =============================================================================
// HTS_Rx_CFO_SinCos_Table.cpp — INNOViD HTS Rx CFO sin/cos table (stub init)
// Phase 1-1: compile-only; Phase 1-2 fills table with validated values.
// =============================================================================
#include "HTS_Rx_CFO_SinCos_Table.hpp"

namespace hts {
namespace rx_cfo {

int16_t g_sin_table[kSinCosTableSize] = {};
int16_t g_cos_table[kSinCosTableSize] = {};

namespace {
bool g_initialized = false;
}  // namespace

void Build_SinCos_Table() noexcept {
    if (g_initialized) {
        return;
    }
    for (int i = 0; i < kSinCosTableSize; ++i) {
        g_sin_table[i] = 0;
        g_cos_table[i] = static_cast<int16_t>(kQ14One);
    }
    g_initialized = true;
}

}  // namespace rx_cfo
}  // namespace hts
