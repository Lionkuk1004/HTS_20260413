// =========================================================================
// HTS_Layer10111213_Sequential_Audit_PC_Test.cpp
// 기준서 «# HTS B-CDMA 전수검사 통합 기준서.md» Layer 10→11→12→13 순 스모크 (10건)
//
// Layer 10 — FEC/HARQ + TX 스케줄러 (FEC_HARQ / HTS_Tx_Scheduler)
// Layer 11 — 디스패처 (HTS_V400_Dispatcher)
// Layer 12 — 보안 파이프라인·역할 인증 (AEAD_Integrity_Vault / Security_Pipeline / Role_Auth)
// Layer 13 — 스토리지 계약: 저장소 TU 미구현 시 Universal_API 경계로 대체
//   · HTS_Storage_Adapter.hpp 의 Restore 계약(nullptr∧word_count≠0 → false)은
//     현재 트리에 .cpp 가 없어 런타임 호출 불가 — 링크 시 별도 TU 추가 후 동일 시나리오 권장.
//
// [PC 빌드] MSVC x64 권장 (HTS_Anti_Debug.cpp 비-ARM 허용). CMake: HTS_Layer10111213_Sequential_Audit_Run
// [주의] Security_Pipeline 은 세션 게이트·AntiAnalysis 경로가 있으나, 본 테스트는
//        abort_signal=true 로 CFI 진입 전 조기 반환만 검증합니다.
// =========================================================================
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || \
    defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#error "[HTS_FATAL] HTS_Layer10111213_Sequential_Audit_PC_Test — PC 전용"
#endif

#include "HTS_AEAD_Integrity.hpp"
#include "HTS_Dynamic_Config.h"
#include "HTS_FEC_HARQ.hpp"
#include "HTS_Role_Auth.h"
#include "HTS_Security_Pipeline.h"
#include "HTS_Tx_Scheduler.hpp"
#include "HTS_Universal_API.h"
#include "HTS_V400_Dispatcher.hpp"

#include "../../HTS_LIM/HTS_CXX17_Atomic_Safe.h"
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>

using namespace ProtectedEngine;

namespace {

alignas(16) static int32_t g_q32[256] = {};
alignas(16) static uint8_t g_info[8] = {};
alignas(16) static int16_t g_outI[16384] = {};
alignas(16) static int16_t g_outQ[16384] = {};
alignas(16) static uint8_t g_fec_syms[512] = {};
alignas(16) static uint32_t g_pipe_words[16] = {};
alignas(16) static FEC_HARQ::WorkBuf g_fec_wb{};

struct TestResult {
    const char* id;
    bool passed;
};

// ── Layer 10 — TX 스케줄러: nullptr 페이로드 거부 ───────────────────────
static bool L10_1_TxScheduler_Reject_Null_Push()
{
    HTS_Tx_Scheduler sched(HTS_Sys_Tier::STANDARD_CHIP);
    if (!sched.Initialize()) {
        return false;
    }
    return !sched.Push_Waveform_Chunk(nullptr, 32u);
}

// ── Layer 10 — TX 스케줄러: 링 백프레셔(적재 거부) 발생 여부 ───────────
static bool L10_2_TxScheduler_Ring_Backpressure()
{
    HTS_Tx_Scheduler sched(HTS_Sys_Tier::STANDARD_CHIP);
    if (!sched.Initialize()) {
        return false;
    }
    for (size_t i = 0; i < 256u; ++i) {
        g_q32[i] = 0;
    }
    bool saw_reject = false;
    for (int n = 0; n < 200000; ++n) {
        if (!sched.Push_Waveform_Chunk(g_q32, 32u)) {
            saw_reject = true;
            break;
        }
    }
    return saw_reject;
}

// ── Layer 10 — FEC: Encode64 nullptr info → 0 ───────────────────────────
static bool L10_3_FEC_Encode_Reject_Null_Info()
{
    const int n = FEC_HARQ::Encode64(
        nullptr, 1, g_fec_syms, 0x111u, g_fec_wb);
    return n == 0;
}

// ── Layer 11 — V400: info_len < 0 거부 ───────────────────────────────────
static bool L11_4_V400_Reject_Negative_Ilen()
{
    HTS_V400_Dispatcher disp;
    disp.Set_Seed(0x55AA55AAu);
    const int r = disp.Build_Packet(
        PayloadMode::DATA,
        g_info,
        -1,
        300,
        g_outI,
        g_outQ,
        16000);
    return r <= 0;
}

// ── Layer 11 — V400: info_len == 0 → DATA 인코드 실패로 칩 수 0 ─────────
static bool L11_5_V400_Reject_Zero_Ilen_Data()
{
    HTS_V400_Dispatcher disp;
    disp.Set_Seed(0xA5A5A5A5u);
    const int r = disp.Build_Packet(
        PayloadMode::DATA,
        g_info,
        0,
        300,
        g_outI,
        g_outQ,
        16000);
    return r <= 0;
}

// ── Layer 12 — AEAD 상수시간 태그: 불일치 시 non-zero ────────────────────
static bool L12_6_AEAD_Tag_Mismatch_NonZero()
{
    const uint32_t d = AEAD_Integrity_Vault::Constant_Time_Compare(
        0x1111222233334444ULL,
        0xFFFFEEEEAAAA5555ULL);
    return d != 0u;
}

// ── Layer 12 — Security_Pipeline: abort 선행 시 조기 반환(무 heavy 경로) ─
static bool L12_7_Security_Pipeline_Abort_Before_CFI()
{
    Security_Pipeline pipe;
    std::atomic<bool> abort_flag{ true };
    for (size_t i = 0; i < 16u; ++i) {
        g_pipe_words[i] = i;
    }
    pipe.Secure_Master_Worker(
        g_pipe_words, 0u, 4u, abort_flag, 16u);
    return true;
}

// ── Layer 12 — Role_Auth: nullptr 비밀번호 거부 ─────────────────────────
static bool L12_8_Role_Auth_Reject_Null_Password()
{
    return !Role_Auth::Authenticate(
        nullptr, 8u, Role::USER);
}

// ── Layer 13 — Universal_API: 잘못된 세션 → 게이트 실패 마스크 ──────────
static bool L13_9_Universal_Gate_Wrong_Session()
{
    const uint32_t m = Universal_API::Secure_Gate_Open(0u);
    return m != Universal_API::SECURE_GATE_MASK_OK;
}

// ── Layer 13 — Universal_API: nullptr 소거 경로 무동작 ─────────────────
static bool L13_10_Universal_Erasure_Null_NoCrash()
{
    Universal_API::Absolute_Trace_Erasure(nullptr, 64u);
    return true;
}

static void Run_Layer10111213_Audit()
{
    const TestResult results[] = {
        { "L10.1_TxSched_null_push", L10_1_TxScheduler_Reject_Null_Push() },
        { "L10.2_TxSched_backpressure", L10_2_TxScheduler_Ring_Backpressure() },
        { "L10.3_FEC_encode_null", L10_3_FEC_Encode_Reject_Null_Info() },
        { "L11.4_V400_ilen_neg", L11_4_V400_Reject_Negative_Ilen() },
        { "L11.5_V400_ilen_zero", L11_5_V400_Reject_Zero_Ilen_Data() },
        { "L12.6_AEAD_tag_diff", L12_6_AEAD_Tag_Mismatch_NonZero() },
        { "L12.7_SecPipe_abort", L12_7_Security_Pipeline_Abort_Before_CFI() },
        { "L12.8_RoleAuth_null_pw", L12_8_Role_Auth_Reject_Null_Password() },
        { "L13.9_UAPI_gate_fail", L13_9_Universal_Gate_Wrong_Session() },
        { "L13.10_UAPI_erase_null", L13_10_Universal_Erasure_Null_NoCrash() },
    };
    const int n = static_cast<int>(sizeof(results) / sizeof(results[0]));

    std::cout << "\n[HTS_Layer10111213_Audit] Layer 10→13 sequential (" << n << ")\n";

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
    std::cout << "\n[SUCCESS] Layer 10–13 audit passed.\n";
}

} // namespace

int main()
{
    std::cout << "========================================================\n";
    std::cout << " HTS Layer 10–13 (FEC/Dispatcher/Pipeline/Boundary) PC Audit\n";
    std::cout << "========================================================\n";
    Run_Layer10111213_Audit();
    return EXIT_SUCCESS;
}
