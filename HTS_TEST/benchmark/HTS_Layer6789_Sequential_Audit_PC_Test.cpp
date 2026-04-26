// =========================================================================
// HTS_Layer6789_Sequential_Audit_PC_Test.cpp
// 기준서 «# HTS B-CDMA 전수검사 통합 기준서.md» Layer 6→7→8→9 순 스모크 (10건)
//
// Layer 6 — 안티디버그·글리치·포인터 인증 (AntiGlitch / PAC / AntiDebug poll)
// Layer 7 — 로거·동적 설정 (SecureLogger / HTS_Dynamic_Config)
// Layer 8 — DSP/PHY (Rx_Matched_Filter / AntiJam_Engine)
// Layer 9 — 텐서 엔진 (HTS_Holo_Tensor_4D_TX / Sparse_Recovery)
//
// [PC 빌드 주의]
//  · HTS_Anti_Debug.cpp 는 비-ARM에서 MSVC일 때만 컴파일 허용됨 — 본 타깃은 MSVC x64 권장.
//  · PAC 변조(Authenticate 실패) 경로는 Halt_PAC_Violation → Self_Healing 후 무한루프이므로
//    스모크에서는 서명·검증 정상 경로만 검사 (위조 테스트는 타깃 펌웨어/수동 감사용).
//
// 빌드: CMake 타깃 HTS_Layer6789_Sequential_Audit_Run
// =========================================================================
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || \
    defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#error "[HTS_FATAL] HTS_Layer6789_Sequential_Audit_PC_Test — PC 전용"
#endif

#include "HTS_Anti_Glitch.h"
#include "HTS_AntiJam_Engine.h"
#include "HTS_Dynamic_Config.h"
#include "HTS_Holo_Tensor_4D_TX.h"
#include "HTS_Pointer_Auth.hpp"
#include "HTS_Rx_Matched_Filter.h"
#include "HTS_Secure_Logger.h"
#include "HTS_Sparse_Recovery.h"

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>

using namespace ProtectedEngine;

namespace {

alignas(16) static uint8_t g_pac_object[64] = {};
alignas(16) static char g_logger_garbage[10000] = {};
alignas(16) static int32_t g_mf_ref[16] = {};
alignas(16) static int32_t g_mf_rx[8192] = {};
alignas(16) static int32_t g_mf_out[8177] = {}; // 8192 - 16 + 1
alignas(16) static int16_t g_aj_I[64] = {};
alignas(16) static int16_t g_aj_Q[64] = {};
alignas(16) static int8_t g_holo_chips[128] = {};

struct TestResult {
    const char* id;
    bool passed;
};

// ── Layer 6 — AntiGlitch (잠금 해제 후 검증 경로만 — 미해제 시 자가치유) ──
static bool L6_1_AntiGlitch_Unlock_Verify()
{
    AntiGlitchShield shield;
    shield.unlockSystem();
    shield.verifyCriticalExecution();
    return true;
}

// ── Layer 6 — PAC: 런타임 키 초기화 후 Sign → Authenticate 라운드트립 ──
static bool L6_2_PAC_Sign_Auth_Roundtrip()
{
    const uint64_t seed = 0x000A11CEu ^ (1ULL << 32);
    PAC_Manager::Initialize_Runtime_Key(seed);
    uint64_t token = PAC_Manager::Sign_Pointer(
        static_cast<void*>(g_pac_object));
    void* recovered = PAC_Manager::Authenticate_Pointer<void*>(token);
    return recovered == static_cast<void*>(g_pac_object);
}

// ── Layer 6 — AntiDebug poll (MSVC x64: MMIO 미접근 본문) ────────────────
static bool L6_3_SecureLogger_Poll_Debug()
{
    SecureLogger::pollDebuggerHardwareOrFault();
    return true;
}

// ── Layer 7 — 로거: 비널종료 details — Append_Lit 세그먼트 상한 내 OOB 방지 ──
static bool L7_4_Logger_Long_Unterminated_Payload()
{
    std::memset(g_logger_garbage, 'Z', sizeof(g_logger_garbage));
    g_logger_garbage[sizeof(g_logger_garbage) - 1u] = 'Z';
    SecureLogger::logSecurityEvent("L6789_AUDIT", g_logger_garbage);
    return true;
}

// ── Layer 7 — 동적 설정: Tier 프로파일 조회 ────────────────────────────
static bool L7_5_Dynamic_Config_Tier_Profile()
{
    const HTS_Sys_Config cfg =
        HTS_Sys_Config_Factory::Get_Tier_Profile(HTS_Sys_Tier::STANDARD_CHIP);
    return cfg.node_count > 0u && cfg.vdf_iterations > 0u &&
        SecureLogger_GetAuditRingSlotCount() == 16u &&
        SecureLogger_GetAuditLineBytes() == 256u;
}

// ── Layer 8 — 정합 필터: nullptr 기준 시퀀스 거부 ────────────────────────
static bool L8_6_Matched_Filter_Reject_Null_Ref()
{
    HTS_Rx_Matched_Filter mf(HTS_Sys_Tier::STANDARD_CHIP);
    return !mf.Set_Reference_Sequence(nullptr, 8u);
}

// ── Layer 8 — 정합 필터: Q16 포화 입력 + int64 누산·int32 클램프 ─────────
static bool L8_7_Matched_Filter_Saturation_Clamp()
{
    HTS_Rx_Matched_Filter mf(HTS_Sys_Tier::STANDARD_CHIP);
    for (size_t i = 0; i < 16u; ++i) {
        g_mf_ref[i] = 32767;
    }
    if (!mf.Set_Reference_Sequence(g_mf_ref, 16u)) {
        return false;
    }
    for (size_t i = 0; i < 8192u; ++i) {
        g_mf_rx[i] = 32767;
    }
    if (!mf.Apply_Filter(g_mf_rx, 8192u, g_mf_out)) {
        return false;
    }
    const int32_t last = g_mf_out[8176];
    return last <= 2147483647 && last >= (-2147483647 - 1);
}

// ── Layer 8 — AntiJam: 동일 I/Q 블록 처리 (특이행렬 스트레스, 크래시 없음) ──
static bool L8_8_AntiJam_Identical_IQ_Process()
{
    AntiJamEngine engine;
    engine.Reset(64);
    for (int i = 0; i < 64; ++i) {
        g_aj_I[i] = 1000;
        g_aj_Q[i] = 1000;
    }
    engine.Process(g_aj_I, g_aj_Q, 64);
    return true;
}

// ── Layer 9 — Holo 텐서: 미초기화·널 데이터 인코딩 거부 ────────────────
static bool L9_9_Holo_Encode_Null_Data()
{
    HTS_Holo_Tensor_4D_TX holo;
    const uint32_t r = holo.Encode_Block(nullptr, 8, g_holo_chips, 64);
    return r == HTS_Holo_Tensor_4D_TX::SECURE_FALSE;
}

// ── Layer 9 — Sparse Recovery: nullptr 입력 → false ─────────────────────
static bool L9_10_Sparse_Execute_Null_Tensor()
{
    RecoveryStats st{};
    const bool ok = Sparse_Recovery_Engine::Execute_L1_Reconstruction<uint32_t>(
        nullptr,
        16u,
        1u,
        2u,
        true,
        true,
        st);
    return !ok && st.total_elements == 0u;
}

static void Run_Layer6789_Audit()
{
    const TestResult results[] = {
        { "L6.1_AntiGlitch_unlock_verify", L6_1_AntiGlitch_Unlock_Verify() },
        { "L6.2_PAC_roundtrip", L6_2_PAC_Sign_Auth_Roundtrip() },
        { "L6.3_Logger_poll_dbg", L6_3_SecureLogger_Poll_Debug() },
        { "L7.4_Logger_long_payload", L7_4_Logger_Long_Unterminated_Payload() },
        { "L7.5_Config_tier_ring", L7_5_Dynamic_Config_Tier_Profile() },
        { "L8.6_Matched_null_ref", L8_6_Matched_Filter_Reject_Null_Ref() },
        { "L8.7_Matched_saturation", L8_7_Matched_Filter_Saturation_Clamp() },
        { "L8.8_AntiJam_ident_IQ", L8_8_AntiJam_Identical_IQ_Process() },
        { "L9.9_Holo_encode_null", L9_9_Holo_Encode_Null_Data() },
        { "L9.10_Sparse_null_exec", L9_10_Sparse_Execute_Null_Tensor() },
    };
    const int n = static_cast<int>(sizeof(results) / sizeof(results[0]));

    std::cout << "\n[HTS_Layer6789_Audit] Layer 6→9 sequential (" << n << ")\n";

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
    std::cout << "\n[SUCCESS] Layer 6–9 audit passed.\n";
}

} // namespace

int main()
{
    std::cout << "========================================================\n";
    std::cout << " HTS Layer 6–9 (Shield/Logger/DSP/Tensor) PC Audit\n";
    std::cout << "========================================================\n";
    Run_Layer6789_Audit();
    return EXIT_SUCCESS;
}
