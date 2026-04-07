// =========================================================================
// HTS_Layer345_Sequential_Audit_PC_Test.cpp
// 기준서 «# HTS B-CDMA 전수검사 통합 기준서.md» Layer 3→4→5 순 스모크
//
// Layer 3 — KCMVP/FIPS 암호 엔진 (LEA → LSH → HMAC → ARIA, 기준서 §3·§8-5 TU 나열 정합)
// Layer 4 — 조건부 자가진단·Flash 무결성 API (HTS_Conditional_SelfTest)
// Layer 5 — 키 로테이션 (HTS_Key_Rotator / DynamicKeyRotator)
//
// [범위 밖 — 별도 TU/POST]
//  · HTS_Crypto_KAT::Run_All_Crypto_KAT / CTR_DRBG — SecureLogger·Physical 엔트로피 등
//    추가 링크 필요 → PC 전용 «KCMVP_암호_4종_종합_테스트» / 펌웨어 POST 권장.
//  · EntropyMonitor — RCT/APT 실패 시 PC에서 무한 루프(Auto_Rollback) 경로 진입 가능.
//  · HTS_Key_Provisioning / HTS_Secure_Boot_Verify — AES·Flash/OTP 스텁 대량 필요.
//
// 빌드: CMake 타깃 HTS_Layer345_Sequential_Audit_Run (HTS_검증_KCMVP.vcxproj TU + CondSelfTest + KeyRotator)
// =========================================================================
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || \
    defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#error "[HTS_FATAL] HTS_Layer345_Sequential_Audit_PC_Test — PC 전용"
#endif

#include "HTS_ARIA_Bridge.hpp"
#include "HTS_Conditional_SelfTest.h"
#include "HTS_HMAC_Bridge.hpp"
#include "HTS_Key_Rotator.h"
#include "HTS_LEA_Bridge.h"
#include "HTS_LSH256_Bridge.h"

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>

using namespace ProtectedEngine;

namespace {

alignas(64) static uint8_t g_key32[32] = {};
alignas(64) static uint8_t g_iv16[16] = {};
alignas(64) static uint8_t g_buf[1024] = {};
alignas(64) static uint8_t g_digest[64] = {};
alignas(64) static uint8_t g_hmac_out[32] = {};

static LEA_Bridge g_lea;

struct TestResult {
    const char* id;
    bool passed;
};

// ── Layer 3 — LEA ─────────────────────────────────────────────────────
static bool L3_1_LEA_Null_Key()
{
    const uint32_t r = g_lea.Initialize(
        nullptr, 16u, g_iv16, 16u);
    return r == LEA_Bridge::SECURE_FALSE;
}

// ── Layer 3 — LSH-256 ───────────────────────────────────────────────────
static bool L3_2_LSH_Null_Output()
{
    const uint32_t r = LSH256_Bridge::Hash_256(g_buf, 0u, nullptr);
    return r == LSH_SECURE_FALSE;
}

static bool L3_3_LSH_Null_Data_Nonzero_Len()
{
    const uint32_t r = LSH256_Bridge::Hash_256(nullptr, 16u, g_digest);
    return r == LSH_SECURE_FALSE;
}

// ── Layer 3 — HMAC ──────────────────────────────────────────────────────
static bool L3_4_HMAC_Null_And_Huge_Len()
{
    HMAC_Context ctx{};
    const uint32_t i0 = HMAC_Bridge::Init(ctx, nullptr, 32u);
    const uint32_t i1 = HMAC_Bridge::Init(ctx, g_key32, 32u);
    const uint32_t u0 = HMAC_Bridge::Update(ctx, g_buf, 1u);
    const uint32_t u1 = HMAC_Bridge::Update(ctx, nullptr, 8u);
    // data_len > uint32_t 범위는 64-bit size_t에서만 표현 가능 (32-bit에서는 +1 오버플로)
    bool huge_rejected = true;
#if SIZE_MAX > UINT32_MAX
    constexpr size_t kHuge =
        (static_cast<size_t>(std::numeric_limits<uint32_t>::max)()) + 1u;
    const uint32_t u2 = HMAC_Bridge::Update(ctx, g_buf, kHuge);
    huge_rejected = (u2 == HMAC_Bridge::SECURE_FALSE);
#endif
    return i0 == HMAC_Bridge::SECURE_FALSE && i1 == HMAC_Bridge::SECURE_TRUE &&
        u0 == HMAC_Bridge::SECURE_TRUE && u1 == HMAC_Bridge::SECURE_FALSE &&
        huge_rejected;
}

// ── Layer 3 — ARIA ──────────────────────────────────────────────────────
static bool L3_5_ARIA_Null_Invalid_Process()
{
    ARIA_Bridge aria;
    const bool a0 = aria.Initialize_Encryption(nullptr, 128);
    const bool a1 = aria.Initialize_Encryption(g_key32, 130);
    const bool a2 = aria.Initialize_Encryption(g_key32, 128);
    const bool p0 = aria.Process_Block(nullptr, g_buf);
    const bool p1 = a2 ? aria.Process_Block(g_buf, nullptr) : false;
    aria.Reset();
    return !a0 && !a1 && !p0 && !p1;
}

// ── Layer 4 — Conditional_SelfTest ───────────────────────────────────────
static bool L4_6_Cond_ARIA_Null_Key()
{
    return !Conditional_SelfTest::Verify_ARIA_Key(nullptr, 128);
}

static bool L4_7_Cond_LEA_Null_Iv()
{
    return !Conditional_SelfTest::Verify_LEA_Key(g_key32, 16u, nullptr);
}

static bool L4_8_Cond_Flash_Read_Null()
{
    static uint8_t expect[32] = {};
    return !Conditional_SelfTest::Verify_Flash_Integrity(
        nullptr, 0u, 256u, g_key32, expect);
}

// ── Layer 5 — DynamicKeyRotator ─────────────────────────────────────────
static bool L5_9_Rotator_Null_Out()
{
    DynamicKeyRotator rot(g_key32, sizeof(g_key32));
    size_t out_len = 0u;
    return !rot.deriveNextSeed(0u, nullptr, 32u, out_len);
}

static bool L5_10_Rotator_Buffer_Too_Small_And_Stress()
{
    DynamicKeyRotator rot(g_key32, sizeof(g_key32));
    size_t out_len = 0u;
    uint8_t small[8] = {};
    if (rot.deriveNextSeed(0u, small, sizeof(small), out_len)) {
        return false;
    }
    uint8_t out[32] = {};
    for (int i = 0; i < 500; ++i) {
        if (!rot.deriveNextSeed(static_cast<uint32_t>(i), out, sizeof(out), out_len)) {
            return false;
        }
        if (out_len != 32u) {
            return false;
        }
    }
    return true;
}

static void Run_Layer345_Audit()
{
    // 기준서 파이프라인 순서: Layer 3 → 4 → 5
    const TestResult results[] = {
        { "L3.1_LEA_null_key", L3_1_LEA_Null_Key() },
        { "L3.2_LSH_null_out", L3_2_LSH_Null_Output() },
        { "L3.3_LSH_null_data", L3_3_LSH_Null_Data_Nonzero_Len() },
        { "L3.4_HMAC_bounds", L3_4_HMAC_Null_And_Huge_Len() },
        { "L3.5_ARIA_null_inv", L3_5_ARIA_Null_Invalid_Process() },
        { "L4.6_CondARIA_null", L4_6_Cond_ARIA_Null_Key() },
        { "L4.7_CondLEA_nullIv", L4_7_Cond_LEA_Null_Iv() },
        { "L4.8_CondFlash_cb", L4_8_Cond_Flash_Read_Null() },
        { "L5.9_Rot_null_out", L5_9_Rotator_Null_Out() },
        { "L5.10_Rot_stress", L5_10_Rotator_Buffer_Too_Small_And_Stress() },
    };
    const int n = static_cast<int>(sizeof(results) / sizeof(results[0]));

    std::cout << "\n[HTS_Layer345_Audit] Layer 3→4→5 sequential (" << n << ")\n";

    int fail_count = 0;
    for (int i = 0; i < n; ++i) {
        if (!results[i].passed) {
            std::cerr << "[FAIL] " << results[i].id << '\n';
            fail_count++;
        } else {
            std::cout << "[PASS] " << results[i].id << '\n';
        }
    }

    if (fail_count != 0) {
        std::cerr << "\n[ALERT] " << fail_count << " test(s) failed.\n";
        std::exit(EXIT_FAILURE);
    }
    std::cout << "\n[SUCCESS] Layer 3–5 audit passed.\n";
}

} // namespace

int main()
{
    std::cout << "========================================================\n";
    std::cout << " HTS Layer 3–5 (Crypto / CondSelfTest / KeyRotator) PC Audit\n";
    std::cout << "========================================================\n";
    Run_Layer345_Audit();
    return EXIT_SUCCESS;
}
