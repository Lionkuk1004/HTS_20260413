// =============================================================================
// HTS_Session_Derive_Stub.cpp — PC 하네스 전용
// Walsh_Row_Permuter 등이 Session_Gateway::Derive_Session_Material 을 링크하되
// 전체 Session_Gateway.cpp(물리 신뢰·SHA256·로거 체인) 없이 T6/벤치를 구성.
// 반환 0 → Walsh Initialize SECURE_FALSE → mask=0 (항등, 레거시 동작).
// =============================================================================
#include "../../HTS_LIM/HTS_Session_Gateway.hpp"

namespace ProtectedEngine {

size_t Session_Gateway::Derive_Session_Material(const char *domain_label,
                                                uint8_t *out_buf,
                                                size_t out_len) noexcept {
    (void)domain_label;
    (void)out_buf;
    (void)out_len;
    return 0u;
}

} // namespace ProtectedEngine
