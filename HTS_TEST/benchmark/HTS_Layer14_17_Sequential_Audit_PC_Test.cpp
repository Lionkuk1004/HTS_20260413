// =========================================================================
// HTS_Layer14_17_Sequential_Audit_PC_Test.cpp
// Layer 14~17 (Protocol, OTA, IoT, Scheduler/API) 10건 순차 스모크
//
// [PC 빌드] MSVC x64 권장 — CMake: HTS_Layer1417_Sequential_Audit_Run
// [주의] IPC 엔진은 Initialize 시 MMIO 를 건드리므로 호스트에서는 미초기화 상태 경로만 검증.
//        INVALID_LEN 등 초기화 후 페이로드 상한은 타깃 보드 통합 테스트로 확인.
// =========================================================================
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || \
    defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#error "[HTS_FATAL] HTS_Layer14_17_Sequential_Audit_PC_Test — PC 전용"
#endif

#include "HTS_Console_Manager.h"
#include "HTS_Emergency_Beacon.h"
#include "HTS_IoT_Codec.h"
#include "HTS_IPC_Protocol.h"
#include "HTS_IPC_Protocol_Defs.h"
#include "HTS_Mesh_Sync.h"
#include "HTS_Modbus_Gateway.h"
#include "HTS_OTA_Manager.h"
#include "HTS_OTA_Manager_Defs.h"
#include "HTS_Priority_Scheduler.h"
#include "HTS_Sensor_Fusion.h"
#include "HTS_Universal_API.h"

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>

using namespace ProtectedEngine;

namespace {

alignas(8) static uint8_t g_fuzz_payload[8192] = {};

struct TestResult {
    const char* id;
    bool passed;
};

// ── Layer 14 — IPC: 미초기화 + 페이로드 길이 상한 초과 요청 → NOT_INITIALIZED (MMIO Initialize 회피)
static bool L14_1_IPC_Send_NotInit_Or_LenGuard()
{
    HTS_IPC_Protocol ipc;
    const uint16_t over =
        static_cast<uint16_t>(static_cast<uint32_t>(IPC_MAX_PAYLOAD) + 32u);
    const IPC_Error e =
        ipc.Send_Frame(IPC_Command::PING, g_fuzz_payload, over);
    return e == IPC_Error::NOT_INITIALIZED;
}

// ── Layer 14 — IoT Codec: nullptr 와이어 버퍼 거부
static bool L14_2_IoT_Codec_Parse_Null_Wire()
{
    HTS_IoT_Codec codec;
    IoT_Frame_Header hdr{};
    IoT_TLV_Item items[4]{};
    uint8_t n = 0u;
    const uint32_t r = codec.Parse(
        nullptr, 0u, hdr, items, static_cast<uint8_t>(4u), n);
    return r == HTS_IoT_Codec::SECURE_FALSE;
}

// ── Layer 14 — Mesh: 타이머 래핑·극단 상관값 — 예외 없이 반환
static bool L14_3_Mesh_Beacon_Timing_Stress()
{
    HTS_Mesh_Sync mesh(1u);
    mesh.On_Beacon_Timing(
        2u,
        0xFFFFFFF0u,
        0x00000010u,
        0u,
        0xFFFFFFFEu,
        -32768,
        32767,
        -32768);
    return true;
}

// ── Layer 15 — OTA: 미초기화 상태에서 CHUNK 페이로드 주입 — 조용히 무시(Flash 미접근)
static bool L15_1_OTA_Process_NotInitialized()
{
    HTS_OTA_Manager ota;
    g_fuzz_payload[0] = static_cast<uint8_t>(OTA_Command::CHUNK_DATA);
    std::memset(g_fuzz_payload + 1, 0x5Au, 64u);
    ota.Process_OTA_Command(g_fuzz_payload, 66u);
    return true;
}

// ── Layer 15 — Modbus GW: 헤더 미만 길이 — 조기 반환
static bool L15_2_Modbus_GW_Undersize()
{
    HTS_Modbus_Gateway gw;
    g_fuzz_payload[0] = 0x01u;
    g_fuzz_payload[1] = 0x03u;
    gw.Process_GW_Command(g_fuzz_payload, static_cast<uint16_t>(4u));
    return true;
}

// ── Layer 16 — Sensor Fusion: 극단 raw + Tick (호스트: 물리 신뢰 검사 생략)
static bool L16_1_Sensor_Fusion_Extreme_Integers()
{
    HTS_Sensor_Fusion fus;
    fus.Feed_Temperature(static_cast<int16_t>(-32768));
    fus.Feed_Smoke(0xFFFFu);
    fus.Feed_Wind(0xFFFFu);
    fus.Feed_Accel(0xFFFFu);
    fus.Tick();
    return true;
}

// ── Layer 16 — Emergency Beacon: 트리거 폭주 + Tick(스케줄러 인큐 경로)
static bool L16_2_Emergency_Beacon_Trigger_Storm()
{
    HTS_Priority_Scheduler sched;
    HTS_Emergency_Beacon beacon(0xA501u);
    beacon.Set_GPS(375665, 1269780);
    for (int i = 0; i < 10000; ++i) {
        beacon.Trigger(AlertFlag::SOS_ALARM);
    }
    uint32_t t = 0u;
    for (int k = 0; k < 120; ++k) {
        t += 500u;
        beacon.Tick(t, sched);
    }
    return true;
}

// ── Layer 17 — Priority Scheduler: 잘못된 enum 값 → NULL_INPUT
static bool L17_1_Priority_Enqueue_Invalid_Priority()
{
    HTS_Priority_Scheduler sched;
    uint8_t pkt[HTS_Priority_Scheduler::MAX_PACKET_DATA] = {
        1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u };
    const EnqueueResult r = sched.Enqueue(
        static_cast<PacketPriority>(static_cast<uint8_t>(0xEEu)),
        pkt,
        HTS_Priority_Scheduler::MAX_PACKET_DATA,
        0u);
    return r == EnqueueResult::NULL_INPUT;
}

// ── Layer 17 — Console: 미초기화 Set_Channel_Config → NOT_INITIALIZED
static bool L17_2_Console_SetConfig_NotInitialized()
{
    HTS_Console_Manager cm;
    ChannelConfig cfg{};
    const IPC_Error e = cm.Set_Channel_Config(cfg);
    return e == IPC_Error::NOT_INITIALIZED;
}

// ── Layer 17 — Universal API: 잘못된 세션 키 → 게이트 실패 마스크
static bool L17_3_Universal_API_Gate_Reject_Wrong_Key()
{
    const uint32_t m = Universal_API::Secure_Gate_Open(0u);
    return m != Universal_API::SECURE_GATE_MASK_OK;
}

static void Run_Layer14_17_Sequential_Audit()
{
    TestResult results[10];
    int n = 0;

    results[n++] = { "14.1 IPC (no-init / len guard)     ", L14_1_IPC_Send_NotInit_Or_LenGuard() };
    results[n++] = { "14.2 IoT Codec null wire           ", L14_2_IoT_Codec_Parse_Null_Wire() };
    results[n++] = { "14.3 Mesh beacon timing stress     ", L14_3_Mesh_Beacon_Timing_Stress() };
    results[n++] = { "15.1 OTA chunk (not initialized)   ", L15_1_OTA_Process_NotInitialized() };
    results[n++] = { "15.2 Modbus GW undersize           ", L15_2_Modbus_GW_Undersize() };
    results[n++] = { "16.1 Sensor fusion extreme int     ", L16_1_Sensor_Fusion_Extreme_Integers() };
    results[n++] = { "16.2 Emergency beacon storm        ", L16_2_Emergency_Beacon_Trigger_Storm() };
    results[n++] = { "17.1 Priority invalid priority     ", L17_1_Priority_Enqueue_Invalid_Priority() };
    results[n++] = { "17.2 Console setcfg (not init)     ", L17_2_Console_SetConfig_NotInitialized() };
    results[n++] = { "17.3 Universal API gate wrong key  ", L17_3_Universal_API_Gate_Reject_Wrong_Key() };

    int fail_count = 0;
    for (int i = 0; i < n; ++i) {
        if (!results[i].passed) {
            std::cerr << "[CRITICAL] Test " << results[i].id << " FAILED!\n";
            ++fail_count;
        } else {
            std::cout << "[PASS] Test " << results[i].id << "\n";
        }
    }

    if (fail_count == 0) {
        std::cout << "\n[SUCCESS] Layer 14~17 Protocol/OTA/IoT/Scheduler audit passed.\n";
    } else {
        std::cerr << "\n[ALERT] Layer 14~17 audit failed.\n";
        std::exit(EXIT_FAILURE);
    }
}

} // namespace

int main()
{
    std::cout << "========================================================\n";
    std::cout << " HTS B-CDMA Layer 14~17 (Protocol/OTA/IoT/API) PC Audit\n";
    std::cout << "========================================================\n";
    Run_Layer14_17_Sequential_Audit();
    return EXIT_SUCCESS;
}
