// =============================================================================
// HTS_Session_Derive_Stub.cpp — HTS_Jammer_STD PC 하네스 전용
// Walsh_Row_Permuter 가 Session_Gateway::Derive_Session_Material 을 링크하되
// 전체 Session_Gateway.cpp 없이 빌드한다. (T6/t6_sim 스텁과 동등 동작)
//
// @warning 프로덕션 금지. 고정 시드는 시험 전용.
// =============================================================================
#include "HTS_Session_Gateway.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace ProtectedEngine {

namespace {
static constexpr const char kWalshPermDomain[] = "HTS_WALSH_ROW_PERM";

[[nodiscard]] static bool domain_is_walsh_row_perm(const char *domain) noexcept {
    if (domain == nullptr) {
        return false;
    }
    for (size_t i = 0;; ++i) {
        if (kWalshPermDomain[i] != domain[i]) {
            return false;
        }
        if (kWalshPermDomain[i] == '\0') {
            return true;
        }
    }
}
} // namespace

size_t Session_Gateway::Derive_Session_Material(const char *domain_label,
                                              uint8_t *out_buf,
                                              size_t out_len) noexcept {
    if (domain_label == nullptr || out_buf == nullptr || out_len == 0u) {
        return 0u;
    }
    if (!domain_is_walsh_row_perm(domain_label)) {
        return 0u;
    }

    static constexpr uint8_t kTestSeed[16] = {
        0xA5u, 0x3Cu, 0xF1u, 0x29u, 0x84u, 0x7Bu, 0xD6u, 0x0Eu,
        0x12u, 0xBCu, 0x4Fu, 0x90u, 0x68u, 0xE3u, 0x57u, 0xABu,
    };
    const size_t n =
        (out_len < sizeof(kTestSeed)) ? out_len : sizeof(kTestSeed);
    std::memcpy(static_cast<void *>(out_buf), static_cast<const void *>(kTestSeed),
                n);
    return n;
}

} // namespace ProtectedEngine
