// =========================================================================
// HTS_RS_FEC_Sequential_Audit_PC_Test.cpp
// RS(15,8) + FEC_HARQ 경계·널 포인터·상태 재초기화 스모크 (PC 전용)
//
// [아키텍트 브리핑]
// · RxState64, IR_RxState, WorkBuf 등 거대 상태는 스택 배치 금지 → 정적 전역
// · try-catch·new/malloc 배제 (펌웨어 규약과 동일)
// · 순차 검사로 SegFault·OOB·무한 루프(수동 관찰) 유발 지점 색출
//
// [주의] Decode64_IR(sym_I,sym_Q, nsym, …) 는 길이 nsym×nc 평면을 읽음.
//        nsym=NSYM64 일 때 int16[NSYM64×C64] 필수 — int16[64] 단독이면 OOB.
// =========================================================================
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || \
    defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#error "[HTS_FATAL] HTS_RS_FEC_Sequential_Audit_PC_Test — PC 전용"
#endif

#include "HTS_FEC_HARQ.hpp"
#include "HTS_RS_GF16.h"

#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <iostream>

using namespace ProtectedEngine;

namespace {

constexpr int k_ir_chip_count = FEC_HARQ::NSYM64 * FEC_HARQ::C64;

// -------------------------------------------------------------------------
// [보안 지침] 거대 상태·IR 평면 버퍼 — 정적 배치만
// -------------------------------------------------------------------------
alignas(64) static FEC_HARQ::RxState64 g_rx64_state;
alignas(64) static FEC_HARQ::IR_RxState g_ir_state;
alignas(64) static FEC_HARQ::WorkBuf g_wb;

alignas(64) static uint8_t g_dummy_info[FEC_HARQ::MAX_INFO] = {};
alignas(64) static uint8_t g_dummy_out[FEC_HARQ::MAX_INFO] = {};
alignas(64) static int16_t g_dummy_I[FEC_HARQ::C64] = {};
alignas(64) static int16_t g_dummy_Q[FEC_HARQ::C64] = {};

alignas(64) static int16_t g_ir_flat_I[k_ir_chip_count] = {};
alignas(64) static int16_t g_ir_flat_Q[k_ir_chip_count] = {};

struct TestResult {
    const char* id;
    bool passed;
};

// -------------------------------------------------------------------------
// [1.1] Null Pointer Bomb (RS)
// -------------------------------------------------------------------------
static bool Test_1_1_Null_Pointer()
{
    uint8_t out15[15] = {};
    HTS_RS_GF16_Encode15_8(nullptr, out15);
    HTS_RS_GF16_Encode15_8(g_dummy_info, nullptr);
    const bool decode_null = HTS_RS_GF16_Decode15_8(nullptr);
    return !decode_null;
}

// -------------------------------------------------------------------------
// [1.2] OOB Burst — t=3 초과(4 심볼 손상) → 복호 실패
// -------------------------------------------------------------------------
static bool Test_1_2_OOB_Burst()
{
    const uint8_t valid_info[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    uint8_t cw[15] = {};
    HTS_RS_GF16_Encode15_8(valid_info, cw);
    cw[0] = static_cast<uint8_t>(cw[0] ^ 0x0Fu);
    cw[1] = static_cast<uint8_t>(cw[1] ^ 0x0Fu);
    cw[2] = static_cast<uint8_t>(cw[2] ^ 0x0Fu);
    cw[3] = static_cast<uint8_t>(cw[3] ^ 0x0Fu);
    return HTS_RS_GF16_Decode15_8(cw) == false;
}

// -------------------------------------------------------------------------
// [1.3] Syndrome / 잡입력 — 다회 Decode 가 종료·크래시 없음
//      (특정 패턴 == false 단정은 우연 성공에 취약하므로 호출만 검증)
// -------------------------------------------------------------------------
static bool Test_1_3_Syndrome_Poison()
{
    uint8_t garbage[15] = {
        15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1
    };
    for (int i = 0; i < 15; ++i) {
        garbage[static_cast<std::size_t>(i)] = static_cast<uint8_t>(
            (garbage[static_cast<std::size_t>(i)] * 7u) & 0x0Fu);
    }
    for (int r = 0; r < 500; ++r) {
        uint8_t buf[15];
        for (int i = 0; i < 15; ++i) {
            buf[static_cast<std::size_t>(i)] = static_cast<uint8_t>(
                (r + i * 13) & 0x0Fu);
        }
        (void)HTS_RS_GF16_Decode15_8(buf);
        (void)HTS_RS_GF16_Decode15_8(garbage);
    }
    return true;
}

// -------------------------------------------------------------------------
// [1.4] Zero-Energy — Chase Decode64_A 실패(무결성 미달)
// -------------------------------------------------------------------------
static bool Test_1_4_Zero_Energy()
{
    FEC_HARQ::Init64(g_rx64_state);
    FEC_HARQ::Feed64_1sym(g_rx64_state, g_dummy_I, g_dummy_Q, 0);
    FEC_HARQ::Advance_Round_64(g_rx64_state);
    int olen = 0;
    const bool ok = FEC_HARQ::Decode64_A(
        g_rx64_state,
        g_dummy_out,
        &olen,
        0x12345678u,
        FEC_HARQ::BPS64,
        g_wb);
    return !ok;
}

// -------------------------------------------------------------------------
// [1.5] IR Decode 반복 — nsym=NSYM64 평면 OOB 없이 반환만 검증
// -------------------------------------------------------------------------
static bool Test_1_5_HARQ_IR_Stress()
{
    std::memset(g_ir_flat_I, 0, sizeof(g_ir_flat_I));
    std::memset(g_ir_flat_Q, 0, sizeof(g_ir_flat_Q));
    FEC_HARQ::IR_Init(g_ir_state);
    const int nsym = FEC_HARQ::NSYM64;
    const int bps = FEC_HARQ::BPS64;
    for (int i = 0; i < 100; ++i) {
        const int rv = i & 3;
        int olen = 0;
        (void)FEC_HARQ::Decode64_IR(
            g_ir_flat_I,
            g_ir_flat_Q,
            nsym,
            FEC_HARQ::C64,
            bps,
            0x12345678u,
            rv,
            g_ir_state,
            g_dummy_out,
            &olen,
            g_wb);
    }
    return true;
}

// -------------------------------------------------------------------------
// [1.6] State Desync — Init64 중간 삽입 후 Feed 재개
// -------------------------------------------------------------------------
static bool Test_1_6_State_Desync()
{
    FEC_HARQ::Init64(g_rx64_state);
    FEC_HARQ::Feed64_1sym(g_rx64_state, g_dummy_I, g_dummy_Q, 0);
    FEC_HARQ::Init64(g_rx64_state);
    FEC_HARQ::Feed64_1sym(g_rx64_state, g_dummy_I, g_dummy_Q, 0);
    return true;
}

// -------------------------------------------------------------------------
// [1.7] Output Pointer Destruction (Decode64_A)
// -------------------------------------------------------------------------
static bool Test_1_7_Output_Ptr_Destroy()
{
    FEC_HARQ::Init64(g_rx64_state);
    int olen = 0;
    const bool ok1 = FEC_HARQ::Decode64_A(
        g_rx64_state,
        nullptr,
        &olen,
        0x12345678u,
        FEC_HARQ::BPS64,
        g_wb);
    const bool ok2 = FEC_HARQ::Decode64_A(
        g_rx64_state,
        g_dummy_out,
        nullptr,
        0x12345678u,
        FEC_HARQ::BPS64,
        g_wb);
    return !ok1 && !ok2;
}

// -------------------------------------------------------------------------
// [1.8] Feed64_1sym — nullptr I/Q 무시
// -------------------------------------------------------------------------
static bool Test_1_8_Feed64_Null_IQ()
{
    FEC_HARQ::Init64(g_rx64_state);
    FEC_HARQ::Feed64_1sym(g_rx64_state, nullptr, nullptr, 0);
    return true;
}

// -------------------------------------------------------------------------
// [1.9] Feed64_1sym — sym_idx 범위 밖 무시
// -------------------------------------------------------------------------
static bool Test_1_9_Feed64_Bad_SymIdx()
{
    FEC_HARQ::Init64(g_rx64_state);
    FEC_HARQ::Feed64_1sym(g_rx64_state, g_dummy_I, g_dummy_Q, -1);
    FEC_HARQ::Feed64_1sym(
        g_rx64_state, g_dummy_I, g_dummy_Q, FEC_HARQ::NSYM64);
    return true;
}

// -------------------------------------------------------------------------
// [1.10] Decode64_IR — nsym 초과 시 방어
// -------------------------------------------------------------------------
static bool Test_1_10_IR_Nsym_Over()
{
    FEC_HARQ::IR_Init(g_ir_state);
    int olen = 0;
    const bool ok = FEC_HARQ::Decode64_IR(
        g_ir_flat_I,
        g_ir_flat_Q,
        FEC_HARQ::NSYM64 + 1,
        FEC_HARQ::C64,
        FEC_HARQ::BPS64,
        0u,
        0,
        g_ir_state,
        g_dummy_out,
        &olen,
        g_wb);
    return !ok;
}

static void Run_Sequential_Audit()
{
    const TestResult results[] = {
        { "1.1_Null_RS", Test_1_1_Null_Pointer() },
        { "1.2_OOB_RS", Test_1_2_OOB_Burst() },
        { "1.3_RS_Term", Test_1_3_Syndrome_Poison() },
        { "1.4_ZeroChase", Test_1_4_Zero_Energy() },
        { "1.5_IR_Stress", Test_1_5_HARQ_IR_Stress() },
        { "1.6_Init64_x2", Test_1_6_State_Desync() },
        { "1.7_DecOutNull", Test_1_7_Output_Ptr_Destroy() },
        { "1.8_FeedNull", Test_1_8_Feed64_Null_IQ() },
        { "1.9_FeedIdx", Test_1_9_Feed64_Bad_SymIdx() },
        { "1.10_IRnsym+", Test_1_10_IR_Nsym_Over() },
    };
    const int n = static_cast<int>(sizeof(results) / sizeof(results[0]));

    std::cout << "\n[HTS_RS_FEC_Audit] Sequential checks (" << n << ")\n";

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
    std::cout << "\n[SUCCESS] All audit steps passed.\n";
}

} // namespace

int main(int argc, char** argv)
{
    int cycles = 1;
    if (argc > 1) {
        cycles = std::atoi(argv[1]);
    }
    if (cycles < 1) {
        cycles = 1;
    }
    if (cycles > 10000) {
        cycles = 10000;
    }

    for (int c = 1; c <= cycles; ++c) {
        std::cout << "--- Cycle " << c << " ---\n";
        Run_Sequential_Audit();
    }
    return EXIT_SUCCESS;
}
