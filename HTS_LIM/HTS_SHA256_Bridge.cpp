// =========================================================================
// HTS_SHA256_Bridge.cpp
// FIPS 180-4 SHA-256 래퍼 구현부
// Target: STM32F407 (Cortex-M4) / Cortex-A55 / PC
//
// [KISA SHA-256 C 라이브러리 연결]
//  SHA256_Init → SHA256_Process → SHA256_Close (원샷 API 미사용 — 스택 잔류 방지)
//
// [제약] try-catch 0, float/double 0, heap 0
// =========================================================================
#include "HTS_SHA256_Bridge.h"
#include "HTS_Secure_Memory.h"

// KISA SHA-256 C 라이브러리 extern "C" 링크
extern "C" {
#include "KISA_SHA256.h"
}

namespace ProtectedEngine {

    static_assert(
        static_cast<size_t>(SHA256_DIGEST_VALUELEN) == SHA256_Bridge::DIGEST_LEN,
        "KISA SHA256_DIGEST_VALUELEN must match DIGEST_LEN");

    // =====================================================================
    //  Hash — SHA-256 (Init+Process+Close, 단일 버퍼)
    // =====================================================================
    bool SHA256_Bridge::Hash(
        const uint8_t* data, size_t data_len,
        uint8_t* output_32) noexcept {

        if (output_32 == nullptr) { return false; }
        if (data == nullptr && data_len != 0u) {
            SecureMemory::secureWipe(output_32, DIGEST_LEN);
            return false;
        }

        // data_len → UINT 범위 검사 (KISA Process/Close 인자)
        if (data_len > 0xFFFFFFFFu) {
            SecureMemory::secureWipe(output_32, DIGEST_LEN);
            return false;
        }

        // Init → Process(길이>0) → Close; KISA 원샷 SHA256_Encrpyt 미사용(내부 스택 비파쇄 방지)
        SHA256_INFO info = {};
        SHA256_Init(&info);
        if (data_len > 0u) {
            SHA256_Process(&info, data, static_cast<UINT>(data_len));
        }
        SHA256_Close(&info, output_32);
        SecureMemory::secureWipe(&info, sizeof(info));
        return true;
    }

} // namespace ProtectedEngine