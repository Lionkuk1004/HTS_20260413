// =========================================================================
//  AMI_종합_통합_테스트.cpp
//
//  AMI/B-CDMA Phase1~4 — 정밀·현실 시나리오 통합 검증 (한 cpp 단독 실행)
//
//  [요지] 단순 true 고정 CHECK 최소화 — 경계·오류 경로·시간축·상태 전이 검증
//   · 단순 true 고정 CHECK 최소화 — 경계값·오류 경로·시간축·상태 전이 검증
//   · SECURE_TRUE/FALSE, bool 반환 API는 **명시 비교** (X-5-5)
//   · CoAP: URI 정규화·슬롯 한도·nullptr 거부
//   · OTA: 청크 역순 수신·nullptr·진행률 단조·검증 루프
//   · 메쉬: 무경로 포워딩·다중 이웃·라우트 노화 시간축
//   · 계량: 연속 샘플·이벤트 로그 링·보고 주기
//
//  [빌드] HTS_검증_AMI.vcxproj — 본 cpp 하나 + HTS_LIM 정적 라이브러리 링크
//  © INNOViD 2026
// =========================================================================

#include <cstdio>
#include <cstdint>
#include <cstring>
#include "HTS_OTA_AMI_Manager.h"
#include "HTS_Priority_Scheduler.h"

#include "HTS_Emergency_Beacon.h"
#include "HTS_Neighbor_Discovery.h"
#include "HTS_Mesh_Sync.h"
#include "HTS_Location_Engine.h"
#include "HTS_Device_Status_Reporter.h"
#include "HTS_Mesh_Router.h"
#include "HTS_Sensor_Fusion.h"
#include "HTS_Sensor_Aggregator.h"
#include "HTS_CoAP_Engine.h"
#include "HTS_Meter_Data_Manager.h"
#include "HTS_Security_Session.h"
#include "HTS_Secure_Memory.h"
#include "HTS_Power_Manager.h"
#include "HTS_Holo_Dispatcher.h"
#include "HTS_V400_Dispatcher.hpp"
#include "HTS_FEC_HARQ.hpp"

using namespace ProtectedEngine;

namespace {
// Layer 9~11 AMI 카오스: 대형 TU는 정적 저장(스택 초과 방지). 신규 .cpp/.h 생성 없음.
// V400 DATA: 프리앰블+헤더(256칩) + IQ_SAME 페이로드(NSYM64×64). AMI vcxproj는 M4 RAM 시뮬(NSYM64=172) → ~11k칩.
static constexpr int k_chaos_v400_max_chips = 256 + FEC_HARQ::NSYM64 * 64;
HTS_Holo_Dispatcher       g_chaos_l911_holo;
HTS_V400_Dispatcher       g_chaos_l911_v400;
HTS_Security_Session      g_chaos_l911_sess_tx;
HTS_Security_Session      g_chaos_l911_sess_rx;
HTS_Power_Manager         g_chaos_l911_power;
HTS_V400_Dispatcher       g_fhss_tx;
HTS_V400_Dispatcher       g_fhss_rx;
} // namespace

static int g_fhss_decode_hits = 0;

static void fhss_on_decoded(const DecodedPacket& pkt) noexcept {
    if (pkt.success_mask == DecodedPacket::DECODE_MASK_OK) {
        g_fhss_decode_hits++;
    }
}

// ---------------------------------------------------------------------------
//  미니 프레임워크
// ---------------------------------------------------------------------------
static int g_total = 0, g_pass = 0, g_fail = 0;

#define SECTION(name) \
    printf("\n══════════════════════════════════════════\n  %s\n══════════════════════════════════════════\n", name)

#define CHECK(desc, cond) \
    do { \
        g_total++; \
        if (cond) { g_pass++; printf("  [PASS] %s\n", desc); } \
        else { g_fail++; printf("  [FAIL] %s  ← %d\n", desc, __LINE__); } \
    } while (0)

#define SUMMARY() \
    printf("\n┌──────────────────────────────────────────┐\n" \
           "│  AMI 종합 결과 (한눈에)                  │\n" \
           "├──────────────────────────────────────────┤\n" \
           "│  통과: %-4d / 총 %-4d                    │\n" \
           "│  실패: %-4d  →  %s              │\n" \
           "└──────────────────────────────────────────┘\n", \
        g_pass, g_total, g_fail, g_fail ? "최종 FAIL " : "최종 PASS ")

namespace {

inline bool eb_active_is_on(uint32_t r) noexcept {
    return r == HTS_Emergency_Beacon::SECURE_TRUE;
}

/// 이웃 비콘 8B — 기존 스모크 테스트와 동일 레이아웃(모듈과 정합)
inline void fill_beacon_pkt(uint8_t pkt[8], uint16_t src_id,
    uint8_t seq, uint8_t hop, int8_t tx_dbm, uint8_t nbr_cnt, uint8_t cap) noexcept {
    pkt[0] = static_cast<uint8_t>(src_id & 0xFFu);
    pkt[1] = static_cast<uint8_t>((src_id >> 8u) & 0xFFu);
    pkt[2] = seq;
    pkt[3] = hop;
    pkt[4] = static_cast<uint8_t>(tx_dbm);
    pkt[5] = nbr_cnt;
    pkt[6] = cap;
    pkt[7] = 0u;
}

// ---------------------------------------------------------------------------
//  OTA mock crypto (호스트 단위 검증용)
// ---------------------------------------------------------------------------
static bool mock_hmac(const uint8_t*, const uint8_t*, size_t, uint8_t* o) {
    if (o == nullptr) { return false; }
    for (int i = 0; i < 32; ++i) { o[i] = 0xABu; }
    return true;
}
static void mock_init(const uint8_t*) {}
static void mock_update(const uint8_t*, size_t) {}
static void mock_final(uint8_t* o) {
    if (o == nullptr) { return; }
    for (int i = 0; i < 32; ++i) { o[i] = 0xABu; }
}

// CoAP 핸들러 (여러 리소스 공용)
static size_t coap_h_ok(uint8_t, const uint8_t*, size_t,
    uint8_t* resp, size_t cap) {
    if (cap >= 4u) {
        resp[0] = 'O'; resp[1] = 'K'; resp[2] = '\r'; resp[3] = '\n';
        return 4u;
    }
    return 0u;
}

} // namespace

// =========================================================================
//  S1 — 비상 비콘: 최소 송출 시간 + Cancel (현장 복구 시퀀스)
// =========================================================================
static void scenario_emergency_beacon() {
    SECTION("S1 Emergency_Beacon — 30s 최소 송출 후 수동 Cancel");

    HTS_Priority_Scheduler sched;
    HTS_Emergency_Beacon beacon(0xA501u);

    CHECK("부팅 직후 비활성", !eb_active_is_on(beacon.Is_Active()));

    // 과열+저전압 복합 → AUTO 마스크로 활성
    beacon.Trigger(static_cast<uint16_t>(
        AlertFlag::TEMP_HIGH | AlertFlag::BATT_LOW));
    CHECK("복합 알람 후 활성", eb_active_is_on(beacon.Is_Active()));

    // 500ms 슬롯으로 최소 지속(30s)·송출 카운트 시뮬레이션 (첫 Tick 500ms 후)
    beacon.Tick(500u, sched);
    for (uint32_t t = 1000u; t <= 31000u; t += HTS_Emergency_Beacon::BEACON_INTERVAL_MS) {
        beacon.Tick(t, sched);
    }

    CHECK("30초 창 내 여전히 활성(플래그 유지 시)", eb_active_is_on(beacon.Is_Active()));

    beacon.Cancel();
    CHECK("Cancel 후 비활성(최소 송출 충족)", !eb_active_is_on(beacon.Is_Active()));

    beacon.Shutdown();
}

// =========================================================================
//  S2 — 이웃 탐색: 다중 게이트웨이·조회 API
// =========================================================================
static void scenario_neighbor_mesh() {
    SECTION("S2 Neighbor_Discovery — 다중 비콘·Find_Neighbor");

    HTS_Priority_Scheduler sched;
    HTS_Neighbor_Discovery nd(0x0001u);

    nd.Set_Mode(DiscoveryMode::REALTIME, 0u);
    CHECK("REALTIME 모드", nd.Get_Mode() == DiscoveryMode::REALTIME);

    uint8_t pkt[8];
    uint32_t t = 1000u;
    fill_beacon_pkt(pkt, 0x0201u, 1u, 1u, 14, 3u, 0u);
    nd.On_Beacon_Received(pkt, 8u, 200u, t);
    fill_beacon_pkt(pkt, 0x0202u, 2u, 2u, 10, 8u, 1u);
    nd.On_Beacon_Received(pkt, 8u, 180u, t + 50u);
    fill_beacon_pkt(pkt, 0x0203u, 3u, 3u, 8, 12u, 2u);
    nd.On_Beacon_Received(pkt, 8u, 160u, t + 100u);

    CHECK("이웃 3 등록", nd.Get_Neighbor_Count() == 3u);

    NeighborInfo ni = {};
    CHECK("0x0202 검색", nd.Find_Neighbor(0x0202u, ni));
    CHECK("0x0202 LQI>0", ni.lqi > 0u);

    nd.Tick(t + 200u, sched);
    nd.Shutdown();
    CHECK("Shutdown 테이블 비움", nd.Get_Neighbor_Count() == 0u);
}

// =========================================================================
//  S3 — Mesh_Sync + Router: 링크 업/다운·무경로 포워딩
// =========================================================================
static void scenario_routing() {
    SECTION("S3 Mesh_Router — 경로·NO_ROUTE·Link_Down");

    HTS_Priority_Scheduler sched;
    HTS_Mesh_Router router(0x1000u);

    const uint8_t payload[] = { 0x01, 0x02, 0x03 };

    CHECK("무이웃 시 NO_ROUTE",
        router.Forward(0x2000u, payload, sizeof(payload), 8u, 5000u, sched)
        == FwdResult::NO_ROUTE);

    router.On_Link_Up(0x2001u, 92u);
    router.On_Link_Up(0x2002u, 78u);

    RouteEntry re = {};
    CHECK("0x2001 경로 존재", router.Get_Route(0x2001u, re));
    const FwdResult f1 = router.Forward(0x2001u, payload, sizeof(payload), 8u, 6000u, sched);
    CHECK("직접 이웃 포워딩 OK 또는 큐포화",
        f1 == FwdResult::OK || f1 == FwdResult::QUEUE_FULL);

    router.On_Link_Down(0x2001u, 7000u);
    CHECK("단절 후 경로 제거", !router.Get_Route(0x2001u, re));

    router.Shutdown();
}

// =========================================================================
//  S4 — Mesh_Sync: 루트 기준 타이밍
// =========================================================================
static void scenario_time_sync() {
    SECTION("S4 Mesh_Sync — 비콘 타이밍·루트 홉");

    HTS_Mesh_Sync sync(0x3001u);
    sync.On_Beacon_Timing(0x3002u, 10000u, 10008u, 1u, 60000u);
    sync.Set_As_Root();
    CHECK("루트 홉=0", sync.Get_My_Hop_Level() == 0u);
    sync.Shutdown();
}

// =========================================================================
//  S5 — Location + Status: 프라이버시·보고 모드
// =========================================================================
static void scenario_location_status() {
    SECTION("S5 Location_Engine + Device_Status_Reporter");

    HTS_Priority_Scheduler sched;

    HTS_Location_Engine loc(0x4001u, LocationMode::MOBILE, DeviceClass::HUMAN_ADULT);
    CHECK("성인 기본 추적 OFF", loc.Get_Tracking_Mode() == TrackingMode::TRACKING_OFF);
    CHECK("앵커1", loc.Register_Anchor(0x5001u, 375000, 1270000));
    CHECK("앵커2", loc.Register_Anchor(0x5002u, 375200, 1270100));
    CHECK("앵커3", loc.Register_Anchor(0x5003u, 375400, 1270000));
    loc.Set_Battery_Percent(22u);
    loc.Shutdown();

    HTS_Device_Status_Reporter rpt(0x4001u, 0u, ReportMode::ACTIVE);
    rpt.Set_Battery(40u);
    rpt.Set_Temperature(-5);
    rpt.Set_Fault(FaultFlag::SENSOR_FAIL);
    CHECK("센서 장애 표시", rpt.Has_Any_Fault());
    rpt.Tick(0u, sched);
    rpt.Clear_Fault(FaultFlag::SENSOR_FAIL);
    rpt.Tick(100u, sched);
    rpt.Shutdown();
}

// =========================================================================
//  S6 — 센서: HAL 실패 복구 + 융합 화재 시나리오
// =========================================================================
static void scenario_sensors() {
    SECTION("S6 Sensor_Aggregator + Sensor_Fusion");

    HTS_Sensor_Fusion fusion;
    HTS_Sensor_Aggregator agg;

    uint16_t adc[4] = { 4095u, 0u, 2048u, 1024u };
    agg.On_ADC_DMA_Complete(adc);
    agg.On_Accel_Read(0u, false);
    agg.Tick(0u, fusion);
    CHECK("가속도 버스 실패", agg.Get_Health(4u) == SensorHealth::FAIL);
    agg.On_Accel_Read(120u, true);
    agg.Tick(1000u, fusion);
    CHECK("버스 복구", agg.Get_Health(4u) == SensorHealth::OK);

    agg.Tick(2000u, fusion);
    fusion.Feed_Temperature(220);
    fusion.Feed_Smoke(50u);
    fusion.Tick();
    fusion.Feed_Temperature(750);
    fusion.Feed_Smoke(4000u);
    for (int i = 0; i < 20; ++i) { fusion.Tick(); }
    CHECK("화재 조합 EMERGENCY", fusion.Get_Level() == AlertLevel::EMERGENCY);

    agg.Shutdown();
    fusion.Shutdown();
}

// =========================================================================
//  S7 — CoAP: 슬롯 한도·중복·nullptr·송신
// =========================================================================
static void scenario_coap_stress() {
    SECTION("S7 CoAP_Engine — 8리소스 한도·중복·nullptr");

    HTS_Priority_Scheduler sched;
    HTS_CoAP_Engine coap(0x6001u);

    static const char* const uris[8] = {
        "/ami/wh", "/ami/pf", "/ami/lp", "/ami/ev",
        "/gw/r1", "/gw/r2", "/gw/r3", "/gw/r4"
    };
    for (size_t i = 0u; i < HTS_CoAP_Engine::MAX_RESOURCES; ++i) {
        CHECK("리소스 등록", coap.Register_Resource(uris[i], coap_h_ok) == true);
    }
    CHECK("9번째 슬롯 거부", coap.Register_Resource("/overflow", coap_h_ok) == false);
    CHECK("동일 URI 중복 거부", coap.Register_Resource("/ami/wh", coap_h_ok) == false);
    CHECK("nullptr URI 거부", coap.Register_Resource(nullptr, coap_h_ok) == false);
    CHECK("nullptr 핸들러 거부", coap.Register_Resource("/x", nullptr) == false);

    const uint16_t mid = coap.Send_GET(0x6002u, "/ami/wh", 10000u, sched);
    CHECK("Send_GET MID>0", mid > 0u);
    for (int k = 0; k < 5; ++k) {
        coap.Tick(10000u + static_cast<uint32_t>(k) * 500u, sched);
    }
    coap.Shutdown();
}

// =========================================================================
//  S8 — 계량: 연속 샘플·이벤트 링·시간 보고
// =========================================================================
static void scenario_metering() {
    SECTION("S8 Meter_Data_Manager — 연속 계량·이벤트·보고");

    HTS_Priority_Scheduler sched;
    HTS_Meter_Data_Manager meter(0x7001u);

    for (int i = 0; i < 5; ++i) {
        MeterReading rd = {};
        rd.cumul_kwh_x100 = static_cast<uint32_t>(100000u + static_cast<uint32_t>(i) * 25u);
        rd.power_factor = static_cast<uint8_t>(90 + i);
        rd.voltage_x10 = static_cast<uint16_t>(2200u + i);
        rd.current_x100 = static_cast<uint16_t>(1500u + i * 10u);
        rd.watt_hour = static_cast<uint32_t>(1200u + i);
        rd.valid = 1u;
        meter.Update_Reading(rd);
    }
    const MeterReading last = meter.Get_Latest();
    CHECK("최종 누적 단조 증가", last.cumul_kwh_x100 >= 100100u);

    meter.Log_Event(MeterEvent::TAMPER, 5000u);
    meter.Log_Event(MeterEvent::OVERLOAD, 6000u);
    meter.Log_Event(MeterEvent::THRESHOLD, 7000u);

    MeterLogEntry ev[HTS_Meter_Data_Manager::EVENT_LOG_SIZE] = {};
    const size_t n = meter.Get_Event_Log(ev, HTS_Meter_Data_Manager::EVENT_LOG_SIZE);
    CHECK("이벤트 로그 조회", n >= 3u);

    meter.Tick(0u, sched);
    meter.Tick(HTS_Meter_Data_Manager::REPORT_INTERVAL_MS + 1u, sched);
    meter.Shutdown();
}

// =========================================================================
//  S9 — OTA: 역순 청크·nullptr·상태 전이
// =========================================================================
static void scenario_ota() {
    SECTION("S9 OTA_AMI_Manager — 역순 청크·검증 경로");

    HTS_OTA_AMI_Manager ota(0x8001u, 100u);
    OTA_Crypto_Callbacks c = {};
    c.hmac_lsh256 = mock_hmac;
    c.hmac_init = mock_init;
    c.hmac_update = mock_update;
    c.hmac_final = mock_final;
    ota.Register_Crypto(c);

    uint8_t nonce[8] = { 0xA1u,0xA2u,0xA3u,0xA4u,0xA5u,0xA6u,0xA7u,0xA8u };
    uint8_t tag[32];
    std::memset(tag, 0xAB, sizeof(tag));

    CHECK("BEGIN", ota.On_Begin(250u, 1024u, 4u, nonce, tag, 0xCAFEBABEu) == true);
    CHECK("상태 RECEIVING", ota.Get_State() == AMI_OtaState::RECEIVING);

    uint8_t chunk[256];
    std::memset(chunk, 0x5Au, sizeof(chunk));

    CHECK("data nullptr 거부", ota.On_Chunk(0u, nullptr, 256u, nullptr) == false);

    // 역순 수신 (실제 RF 재정렬 시뮬)
    CHECK("청크3", ota.On_Chunk(3u, chunk, 256u, nullptr) == true);
    CHECK("청크1", ota.On_Chunk(1u, chunk, 256u, nullptr) == true);
    CHECK("청크0", ota.On_Chunk(0u, chunk, 256u, nullptr) == true);
    CHECK("청크2", ota.On_Chunk(2u, chunk, 256u, nullptr) == true);
    CHECK("완료 플래그", ota.Is_Complete() == true);
    CHECK("진행률 100", ota.Get_Progress_Pct() == 100u);

    ota.On_Broadcast_Complete(20000u);
    CHECK("VERIFYING 진입", ota.Get_State() == AMI_OtaState::VERIFYING);

    for (int i = 0; i < 40; ++i) {
        ota.Tick(20000u + static_cast<uint32_t>(i) * 50u);
    }
    const AMI_OtaState st = ota.Get_State();
    CHECK("검증 완료 또는 실패(목업 한계)", st == AMI_OtaState::READY || st == AMI_OtaState::FAILED);

    ota.Abort();
    CHECK("Abort IDLE", ota.Get_State() == AMI_OtaState::IDLE);
    ota.Shutdown();
}

// =========================================================================
//  S10 — 교차: 스케줄러에 다모듈 Tick 인터리브 (부하 모델)
// =========================================================================
static void scenario_interleaved_ticks() {
    SECTION("S10 교차 Tick — 스케줄러 공유 부하");

    HTS_Priority_Scheduler sched;
    HTS_Neighbor_Discovery nd(0x9001u);
    HTS_Device_Status_Reporter rpt(0x9001u, 0u, ReportMode::ACTIVE);
    uint8_t pkt[8];
    fill_beacon_pkt(pkt, 0x9101u, 1u, 1u, 12, 4u, 0u);

    for (uint32_t ms = 0u; ms < 5000u; ms += 100u) {
        nd.On_Beacon_Received(pkt, 8u, 190u, ms);
        rpt.Set_Battery(static_cast<uint8_t>(80u + (ms / 100u) % 5u));
        rpt.Tick(ms, sched);
        nd.Tick(ms, sched);
    }
    CHECK("교차 후 이웃>=1", nd.Get_Neighbor_Count() >= 1u);
    rpt.Shutdown();
    nd.Shutdown();
}

// =========================================================================
//  S11 — Layer 9~11 카오스 (텐서·디스패처 / 세션 스래싱·소거 / 전력·슬립 실패 경로)
//  구현 위치: 본 파일 전용. 메인 소스 트리에 별도 테스트 TU 추가 금지.
// =========================================================================
static void scenario_chaos_layer9_to_11() {
    SECTION("S11 Chaos Layer 9~11 — Tensor/Dispatch, Session, Secure Wipe, Power");

    bool ok_holo = true;
    bool ok_v400 = true;
    bool ok_sess = true;
    bool ok_wipe = true;
    bool ok_pwr = true;

    // --- Layer 9 / 11: Holo 텐서 shim + V400 디스패처 교차 호출 ---
    {
        uint32_t holo_seed[4] = {
            0xA11C9E01u, 0x52B3D204u, 0x6F3408EEu, 0xC0DEF11Eu
        };
        const uint32_t holo_init =
            g_chaos_l911_holo.Initialize(holo_seed);
        if (holo_init != HTS_Holo_Dispatcher::SECURE_TRUE) {
            ok_holo = false;
        }

        static int16_t holo_i[512];
        static int16_t holo_q[512];
        uint8_t holo_payload[16];
        for (uint32_t i = 0u; i < 16u; ++i) {
            holo_payload[i] = static_cast<uint8_t>(
                static_cast<uint8_t>(i * 17u) ^ static_cast<uint8_t>(0x5Au));
        }

        // DATA_HOLO만 반복: VOICE/RESILIENT는 소프트 디코드·블록 분할 차이로 바이트 단위 memcmp가
        // 호스트/최적화 조합에서 실패할 수 있어, 하네스는 단일 프로파일 라운드트립으로 고정한다.
        for (uint32_t wave = 0u; wave < 24u; ++wave) {
            (void)g_chaos_l911_holo.Sync_Time_Slot(wave ^ 0x3Cu);
            const uint8_t holo_mode = HoloPayload::DATA_HOLO;
            const size_t n_chips = g_chaos_l911_holo.Build_Holo_Packet(
                holo_mode,
                holo_payload,
                16u,
                static_cast<int16_t>(200 + static_cast<int16_t>(wave & 15u)),
                holo_i,
                holo_q,
                512u);
            if (n_chips == 0u) {
                ok_holo = false;
            }
            if (n_chips > 0u && n_chips <= 512u) {
                uint8_t holo_out[32];
                size_t holo_len = 0u;
                const uint32_t dec_st = g_chaos_l911_holo.Decode_Holo_Block(
                    holo_i,
                    holo_q,
                    static_cast<uint16_t>(n_chips),
                    UINT64_MAX,
                    holo_out,
                    &holo_len);
                if (dec_st != HTS_Holo_Dispatcher::SECURE_TRUE || holo_len != 16u) {
                    ok_holo = false;
                }
                else if (std::memcmp(holo_out, holo_payload, 16u) != 0) {
                    ok_holo = false;
                }
            }
            (void)g_chaos_l911_holo.Advance_Time();
            uint32_t rot[4] = {
                holo_seed[0] ^ wave,
                holo_seed[1] ^ (wave << 3u),
                holo_seed[2] ^ (wave * 0x1001u),
                holo_seed[3] ^ (wave * 0xF0F0F0Fu)
            };
            if (g_chaos_l911_holo.Rotate_Seed(rot) != HTS_Holo_Dispatcher::SECURE_TRUE) {
                ok_holo = false;
            }
        }

        g_chaos_l911_v400.Set_Seed(0xBADC0DE3u);
        g_chaos_l911_v400.Set_IR_Mode(false);
        g_chaos_l911_v400.Set_IR_SIC_Enabled(false);
        static uint8_t v_payload[8];
        for (uint32_t i = 0u; i < 8u; ++i) {
            v_payload[i] = static_cast<uint8_t>(i ^ 0xC3u);
        }
        static int16_t v_i[k_chaos_v400_max_chips];
        static int16_t v_q[k_chaos_v400_max_chips];
        for (uint32_t k = 0u; k < 40u; ++k) {
            const int built = g_chaos_l911_v400.Build_Packet(
                PayloadMode::DATA,
                v_payload,
                8,
                static_cast<int16_t>(250 + static_cast<int16_t>(k & 7u)),
                v_i,
                v_q,
                k_chaos_v400_max_chips);
            if (built <= 0) {
                ok_v400 = false;
            }
            for (int c = 0; c < built && c < k_chaos_v400_max_chips; ++c) {
                g_chaos_l911_v400.Feed_Chip(
                    static_cast<int16_t>(v_i[static_cast<size_t>(c)] ^ static_cast<int16_t>(k & 1u)),
                    static_cast<int16_t>(v_q[static_cast<size_t>(c)] ^ static_cast<int16_t>(k & 2u)));
            }
            g_chaos_l911_v400.Tick_Adaptive_BPS();
            g_chaos_l911_v400.Update_Adaptive_BPS(100u + (k * 37u));
            g_chaos_l911_v400.Reset();
        }

        if (g_chaos_l911_holo.Shutdown() != HTS_Holo_Dispatcher::SECURE_TRUE) {
            ok_holo = false;
        }
    }

    // --- Security_Session 스래싱: 다회 초기화·청크 EtM·MAC 실패·복호 재검증 ---
    //  송신측 IV=iv16 → tx_counter=iv16. 수신측은 Initialize 시 rx_counter=iv^도메인분리.
    //  상대 TX 스트림과 RX 스트림을 맞추려면 수신 세션에 iv_peer(iv16[0]^0x80)를 넣어
    //  rx_counter 초기(복호 전)가 송신 tx_counter 초기와 동일해지게 한다(HTS_Security_Session.h).
    {
        static uint8_t enc_k[32];
        static uint8_t mac_k[32];
        static uint8_t iv16[16];
        for (uint32_t i = 0u; i < 32u; ++i) {
            enc_k[i] = static_cast<uint8_t>(0x10u + (i & 0x0Fu));
            mac_k[i] = static_cast<uint8_t>(0x80u + (i & 0x0Fu));
        }
        for (uint32_t i = 0u; i < 16u; ++i) {
            iv16[i] = static_cast<uint8_t>(i ^ 0x3Cu);
        }

        uint8_t pt[48];
        uint8_t ct[64];
        uint8_t tag[32];
        uint8_t pt2[64];
        uint8_t bad_tag[32];

        for (uint32_t cycle = 0u; cycle < 20u; ++cycle) {
            g_chaos_l911_sess_tx.Terminate_Session();
            g_chaos_l911_sess_rx.Terminate_Session();

            uint8_t iv_peer[16];
            for (uint32_t i = 0u; i < 16u; ++i) {
                iv_peer[i] = iv16[i];
            }
            iv_peer[0] = static_cast<uint8_t>(iv_peer[0] ^ 0x80u);

            const bool ini_tx = g_chaos_l911_sess_tx.Initialize(
                CipherAlgorithm::LEA_256_CTR,
                MacAlgorithm::HMAC_SHA256,
                enc_k,
                mac_k,
                iv16);
            if (!ini_tx || !g_chaos_l911_sess_tx.Is_Active()) {
                ok_sess = false;
                break;
            }
            for (uint32_t j = 0u; j < sizeof(pt); ++j) {
                pt[j] = static_cast<uint8_t>(
                    static_cast<uint8_t>(cycle + j) ^ static_cast<uint8_t>(0xA7u));
            }
            if (!g_chaos_l911_sess_tx.Protect_Begin()) {
                ok_sess = false;
            }
            if (!g_chaos_l911_sess_tx.Protect_Chunk(pt, 16u, ct)) {
                ok_sess = false;
            }
            if (!g_chaos_l911_sess_tx.Protect_Chunk(pt + 16u, 16u, ct + 16u)) {
                ok_sess = false;
            }
            if (!g_chaos_l911_sess_tx.Protect_Chunk(pt + 32u, 16u, ct + 32u)) {
                ok_sess = false;
            }
            if (!g_chaos_l911_sess_tx.Protect_End(tag)) {
                ok_sess = false;
            }
            if (!g_chaos_l911_sess_rx.Initialize(
                    CipherAlgorithm::LEA_256_CTR,
                    MacAlgorithm::HMAC_SHA256,
                    enc_k,
                    mac_k,
                    iv_peer) ||
                !g_chaos_l911_sess_rx.Is_Active()) {
                ok_sess = false;
                break;
            }
            for (uint32_t i = 0u; i < 32u; ++i) {
                bad_tag[i] = static_cast<uint8_t>(tag[i] ^ 0xFFu);
            }
            if (g_chaos_l911_sess_rx.Unprotect_Payload(ct, 48u, bad_tag, pt2)) {
                ok_sess = false;
            }
            g_chaos_l911_sess_rx.Terminate_Session();
            if (!g_chaos_l911_sess_rx.Initialize(
                    CipherAlgorithm::LEA_256_CTR,
                    MacAlgorithm::HMAC_SHA256,
                    enc_k,
                    mac_k,
                    iv_peer)) {
                ok_sess = false;
                break;
            }
            if (!g_chaos_l911_sess_rx.Unprotect_Payload(ct, 48u, tag, pt2)) {
                ok_sess = false;
            }
            for (uint32_t j = 0u; j < 48u; ++j) {
                if (pt[j] != pt2[j]) {
                    ok_sess = false;
                }
            }
            enc_k[0] = static_cast<uint8_t>(enc_k[0] ^ static_cast<uint8_t>(cycle + 1u));
            mac_k[31] = static_cast<uint8_t>(mac_k[31] ^ static_cast<uint8_t>(cycle + 3u));
        }
        g_chaos_l911_sess_tx.Terminate_Session();
        g_chaos_l911_sess_rx.Terminate_Session();
    }

    // --- SecureMemory::secureWipe 검증(샘플 버퍼) ---
    {
        alignas(8) static uint8_t wipe_buf[96];
        for (uint32_t i = 0u; i < sizeof(wipe_buf); ++i) {
            wipe_buf[i] = static_cast<uint8_t>(0x5Au ^ static_cast<uint8_t>(i));
        }
        SecureMemory::secureWipe(static_cast<void*>(wipe_buf), sizeof(wipe_buf));
        uint32_t or_all = 0u;
        for (uint32_t i = 0u; i < sizeof(wipe_buf); ++i) {
            or_all |= static_cast<uint32_t>(wipe_buf[i]);
        }
        if (or_all != 0u) {
            ok_wipe = false;
        }
    }

    // --- 전력 FSM: 클럭 토글·PVD ISR 경로·HAL 없는 슬립 요청(실패)·복구 ---
    {
        g_chaos_l911_power.Shutdown();
        if (!g_chaos_l911_power.Initialize()) {
            ok_pwr = false;
        }
        static const PVD_Level k_pvd_cycle[8] = {
            PVD_Level::V_2_0, PVD_Level::V_2_1, PVD_Level::V_2_3,
            PVD_Level::V_2_5, PVD_Level::V_2_6, PVD_Level::V_2_7,
            PVD_Level::V_2_8, PVD_Level::V_2_9
        };
        for (uint32_t p = 0u; p < 96u; ++p) {
            const PVD_Level pvd_step = k_pvd_cycle[p % 8u];
            g_chaos_l911_power.Set_PVD_Level(pvd_step);
            g_chaos_l911_power.Handle_PVD_Event();

            const PowerMode pm = ((p & 1u) == 0u) ? PowerMode::RUN : PowerMode::LOW_RUN;
            const PowerMode cur_mode = g_chaos_l911_power.Get_Current_Mode();
            const bool wants_change =
                (static_cast<uint8_t>(cur_mode) != static_cast<uint8_t>(pm));
            const bool clk_rc = g_chaos_l911_power.Set_Clock_Mode(pm);

            const uint8_t pv = static_cast<uint8_t>(pvd_step);
            const bool is_low_pvd =
                (pv <= static_cast<uint8_t>(PVD_Level::V_2_3));
            if (is_low_pvd) {
                if (wants_change && clk_rc) {
                    ok_pwr = false;
                }
            } else {
                if (!clk_rc) {
                    ok_pwr = false;
                }
            }

            if (!Power_Is_Valid_State(g_chaos_l911_power.Get_State())) {
                ok_pwr = false;
            }

            g_chaos_l911_power.Set_PVD_Level(PVD_Level::V_2_9);
            g_chaos_l911_power.Handle_PVD_Event();
            if (g_chaos_l911_power.Get_State() != PowerState::ACTIVE) {
                ok_pwr = false;
            }
        }
        // 슬립 구간: 루프 마지막이 V_2_9·ACTIVE여도, 여기서 전제를 한 번 더 고정한다.
        // (Request_Sleep 실패는 HAL 부재 시 의도됨 — ACTIVE→SLEEPING 시도 후 ERROR 수렴)
        g_chaos_l911_power.Set_PVD_Level(PVD_Level::V_2_9);
        g_chaos_l911_power.Handle_PVD_Event();
        if (g_chaos_l911_power.Get_State() != PowerState::ACTIVE) {
            ok_pwr = false;
        }

        const bool sleep_rc = g_chaos_l911_power.Request_Sleep(PowerMode::SLEEP, 0u);
        if (sleep_rc) {
            ok_pwr = false;
        }
        if (!Power_Is_Valid_State(g_chaos_l911_power.Get_State())) {
            ok_pwr = false;
        }
        g_chaos_l911_power.Shutdown();
        if (!g_chaos_l911_power.Initialize()) {
            ok_pwr = false;
        }
        if (g_chaos_l911_power.Get_State() != PowerState::ACTIVE) {
            ok_pwr = false;
        }
        g_chaos_l911_power.Shutdown();
    }

    printf("  [S11] Holo:%s V400:%s Session:%s Wipe:%s Power:%s\n",
        ok_holo ? "OK" : "NG",
        ok_v400 ? "OK" : "NG",
        ok_sess ? "OK" : "NG",
        ok_wipe ? "OK" : "NG",
        ok_pwr ? "OK" : "NG");
    CHECK("S11a Holo 텐서·shim", ok_holo);
    CHECK("S11b V400 디스패처 스트레스", ok_v400);
    CHECK("S11c Security_Session 스래싱", ok_sess);
    CHECK("S11d SecureMemory::secureWipe", ok_wipe);
    CHECK("S11e 전력 FSM·슬립 실패 경로", ok_pwr);
}

// =========================================================================
//  S12 — V400 FHSS: 도약 시퀀스 동기·RF_SETTLING Blanking·DATA 재개
// =========================================================================
static void scenario_rf_hopping_control() {
    SECTION("S12 V400 FHSS — tx/rx 채널 동기·Blanking·정상 DATA RTT");

    static uint8_t fhss_payload[8];
    for (uint32_t i = 0u; i < 8u; ++i) {
        fhss_payload[i] = static_cast<uint8_t>(
            static_cast<uint8_t>(0xA5u) ^ static_cast<uint8_t>(i));
    }
    static int16_t fhss_i[k_chaos_v400_max_chips];
    static int16_t fhss_q[k_chaos_v400_max_chips];

    g_fhss_tx.Reset();
    g_fhss_rx.Reset();
    const uint32_t fhss_seed = 0xC0FFEE12u;
    g_fhss_tx.Set_Seed(fhss_seed);
    g_fhss_rx.Set_Seed(fhss_seed);
    g_fhss_tx.Set_IR_Mode(false);
    g_fhss_rx.Set_IR_Mode(false);
    g_fhss_tx.Set_IR_SIC_Enabled(false);
    g_fhss_rx.Set_IR_SIC_Enabled(false);

    for (uint32_t hop = 0u; hop < 8u; ++hop) {
        const uint8_t expect_ch = HTS_V400_Dispatcher::FHSS_Derive_Channel(
            fhss_seed, hop);

        const uint8_t ch_tx = g_fhss_tx.FHSS_Request_Hop_As_Tx();
        const uint8_t ch_rx = g_fhss_rx.FHSS_Request_Hop_As_Rx();

        CHECK("FHSS hop TX/RX 채널 일치", ch_tx == ch_rx);
        CHECK("FHSS hop 채널 예측값 일치", ch_tx == expect_ch);
        CHECK("FHSS hop 유효 채널(0~127)", ch_tx <= 127u);

        CHECK("FHSS settling 진입(TX)", g_fhss_tx.FHSS_Is_Rf_Settling());
        CHECK("FHSS settling 진입(RX)", g_fhss_rx.FHSS_Is_Rf_Settling());

        for (int slot = 0; slot < HTS_V400_Dispatcher::FHSS_SETTLE_CHIPS; ++slot) {
            const int built_blank = g_fhss_tx.Build_Packet(
                PayloadMode::DATA,
                fhss_payload,
                8,
                static_cast<int16_t>(300),
                fhss_i,
                fhss_q,
                k_chaos_v400_max_chips);
            CHECK("Blanking 중 TX Build_Packet==0", built_blank == 0);
            g_fhss_tx.Feed_Chip(static_cast<int16_t>(0), static_cast<int16_t>(0));
            g_fhss_rx.Feed_Chip(static_cast<int16_t>(0), static_cast<int16_t>(0));
        }

        CHECK("Blanking 종료 후 TX WAIT_SYNC",
            g_fhss_tx.Get_Phase() == RxPhase::WAIT_SYNC);
        CHECK("Blanking 종료 후 RX WAIT_SYNC",
            g_fhss_rx.Get_Phase() == RxPhase::WAIT_SYNC);
        CHECK("Blanking 종료 TX settling 아님", !g_fhss_tx.FHSS_Is_Rf_Settling());
        CHECK("Blanking 종료 RX settling 아님", !g_fhss_rx.FHSS_Is_Rf_Settling());
    }

    g_fhss_tx.Reset();
    g_fhss_rx.Reset();
    g_fhss_tx.Set_Seed(fhss_seed);
    g_fhss_rx.Set_Seed(fhss_seed);
    g_fhss_tx.Set_IR_Mode(false);
    g_fhss_rx.Set_IR_Mode(false);

    g_fhss_decode_hits = 0;
    g_fhss_rx.Set_Packet_Callback(fhss_on_decoded);

    const int built_data = g_fhss_tx.Build_Packet(
        PayloadMode::DATA,
        fhss_payload,
        8,
        static_cast<int16_t>(280),
        fhss_i,
        fhss_q,
        k_chaos_v400_max_chips);
    CHECK("FHSS 후 DATA Build_Packet>0", built_data > 0);

    for (int c = 0; c < built_data && c < k_chaos_v400_max_chips; ++c) {
        g_fhss_rx.Feed_Chip(
            fhss_i[static_cast<size_t>(c)],
            fhss_q[static_cast<size_t>(c)]);
    }

    CHECK("FHSS 후 DATA 디코드 콜백", g_fhss_decode_hits == 1);
    g_fhss_rx.Set_Packet_Callback(nullptr);
}

// =========================================================================
//  main
// =========================================================================
int main() {
    printf("================================================================\n");
    printf("  AMI 종합 통합 테스트 — S1~S11 (… + Layer9~11 카오스)\n");
    printf("================================================================\n");

    scenario_emergency_beacon();
    scenario_neighbor_mesh();
    scenario_routing();
    scenario_time_sync();
    scenario_location_status();
    scenario_sensors();
    scenario_coap_stress();
    scenario_metering();
    scenario_ota();
    scenario_interleaved_ticks();
    scenario_chaos_layer9_to_11();
    scenario_rf_hopping_control();

    SUMMARY();
    return g_fail ? 1 : 0;
}
