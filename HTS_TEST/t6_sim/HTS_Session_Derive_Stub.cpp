// =============================================================================
// HTS_Session_Derive_Stub.cpp — PC 하네스 전용
// Walsh_Row_Permuter 등이 Session_Gateway::Derive_Session_Material 을 링크하되
// 전체 Session_Gateway.cpp(물리 신뢰·SHA256·로거 체인) 없이 T6/벤치를 구성.
//
// 기본: "HTS_WALSH_ROW_PERM" 이외 도메인 → 0 반환 (기존 동작).
// T6 효과 검증: 해당 도메인에 한해 16 byte 고정 시드 반환 → Walsh Initialize
// 성공 → mask ≠ 0 (permutation 활성).
//
// @warning 프로덕션 금지. 고정 시드는 실환경·공개 저장소에서 보안 위협.
//          실배포 시 본 TU 를 링크에서 제외하고 정식 Session_Gateway 구현 사용.
// =============================================================================
#include "../../HTS_LIM/HTS_Session_Gateway.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace ProtectedEngine {

namespace {
/// Walsh_Row_Permuter.cpp 의 DOMAIN_WALSH_ROW_PERM 과 바이트 단위 동일해야 함
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

    // Walsh_Row_Permuter 용 테스트 고정 시드 (16 byte)
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
