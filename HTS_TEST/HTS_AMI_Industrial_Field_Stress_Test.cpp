// =========================================================================
// HTS_AMI_Industrial_Field_Stress_Test.cpp
//
//  목적 (AMI 중심 · 가혹 산업 환경)
//  --------------------------------
//  실제 현장과 동일한 조건으로 계측할 수는 없으므로, **현장에서 관측되는
//  대표 파라미터**(전자기 잡음, 스파크성 펄스, 공용 대역 간섭, 전압 강하,
//  다중 노드 경쟁)를 **수치 모델**로 주입하고, 다음을 **정량 리포트**한다.
//
//   · 산업 부지별(발전/변전/제철/화학/광산) **상대적 열악도**
//   · **1024대 근접 AMI 노드**가 동일 매체에서 송신할 때의 간섭·충돌·수신 성공률
//   · **메쉬 경로 안정성 지수** (단순 그래프 + 라우트 체인 모델)
//   · **음성(코덱) 세션 가용성** — 패킷 손실·지터로부터 추정 MOS(참고치)
//   · **위치 추정 오차** — 거리 측정 잡음 기반 CEP(참고치)
//   · 마지막에 **HTS_LIM 실제 모듈** 소규모 스모크 (Meter / Mesh 라우터)
//
//  [빌드 안내]
//  HTS_TEST 프로젝트에 본 파일을 포함할 경우, main() 중복을 피하기 위해
//  "재밍 종합 테스트.cpp" 등 다른 main이 있는 소스는 **빌드에서 제외**하십시오.
//  (솔루션 탐색기 → 파일 우클릭 → 빌드에서 제외)
//
//  © INNOViD 2026 — 현장 검증용 시뮬레이션 (통계 모델 + 스택 스모크)
// =========================================================================

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "HTS_Priority_Scheduler.h"
#include "HTS_Mesh_Router.h"
#include "HTS_Meter_Data_Manager.h"

using namespace ProtectedEngine;

namespace {

// -------------------------------------------------------------------------
//  출력 유틸 (고정 폭 표 — UTF-8 콘솔 권장)
// -------------------------------------------------------------------------
static constexpr int kTableW = 128;

// -------------------------------------------------------------------------
//  KPI 임계 (현장 요구에 맞게 조정) — 행 단위 PASS/FALL 판정
// -------------------------------------------------------------------------
static constexpr double kKpi_max_site_stress = 2.05;   // 표1: 모델 스트레스
static constexpr double kKpi_min_goodput = 0.67;     // 표2: 시도 대비 성공률
static constexpr double kKpi_min_air_util = 0.235;   // 표2: 공중 이용률
static constexpr double kKpi_min_path_mean = 0.58;  // 표3: 4홉 경로 가용(평균)
static constexpr double kKpi_min_path_p10 = 0.52;    // 표3: 하위 10% 경로 가용
static constexpr double kKpi_max_churn_1k = 25.0;    // 표3: 체인 변동(건/1k 시행)
static constexpr double kKpi_max_pl = 0.038;         // 표4: 패킷 손실률
static constexpr double kKpi_max_jitter_ms = 13.0;   // 표4: 지터
static constexpr double kKpi_min_mos = 3.85;         // 표4: 추정 MOS
static constexpr double kKpi_max_cep95_m = 33.0;     // 표4: CEP95 (m)

static const char* pass_fail(bool ok) {
    return ok ? "PASS" : "FAIL";
}

static void table_rule(char ch = '-') {
    std::cout << std::string(static_cast<size_t>(kTableW), ch) << "\n";
}

static void table_title(const char* text) {
    std::cout << "  " << text << "\n";
    table_rule('=');
}

static void h1(const char* title) {
    std::cout << "\n";
    table_rule('=');
    std::cout << "  " << title << "\n";
    table_rule('=');
    std::cout << "\n";
}

static void h2(const char* title) {
    std::cout << "\n-- " << title << "\n";
    table_rule('-');
}

/// setprecision 누수 방지 — 열마다 독립 포맷
static std::string fmt_fixed(double v, int prec) {
    std::ostringstream os;
    os.setf(std::ios::fixed);
    os << std::setprecision(prec) << v;
    return os.str();
}

static std::string fmt_uint(uint64_t v) {
    std::ostringstream os;
    os << v;
    return os.str();
}

// -------------------------------------------------------------------------
//  A. 산업 현장 프로파일 (상대 스케일, 단위는 모델 내부 정규화)
// -------------------------------------------------------------------------
struct IndustrialSiteProfile {
    const char* id;
    const char* name_kr;
    const char* notes;
    double emi_floor_db;       ///< 기본 EMI 바닥 (높을수록 불리)
    double impulse_rate_hz;    ///< 스파크/스위칭 등 펄스성 간섭 (Hz)
    double broadband_jam_db;   ///< 광대역 재밍/공용 간섭 등가 (dB)
    double voltage_sag_rate;   ///< 전압 강하(순간 브라운아웃) 확률/시간비
    double metal_multipath;    ///< 금속 구조물 다중경로 심함 (0~1)
};

static const IndustrialSiteProfile kSites[] = {
    { "PP",  "화력/원자력 발전 부지",
        "대용량 변압기, SVG, 고조파, 고전류 버스 — EMI 바닥 높음",
        8.0, 120.0, 6.0, 0.02, 0.35 },
    { "SS",  "송변전소 / 개폐기",
        "SF6 차단기 조작, 서지, 계전기 — 펄스성 간섭 다수",
        10.0, 400.0, 5.0, 0.05, 0.25 },
    { "STL", "제철 / 용광로 인근",
        "아크, 유도로, 크레인 인버터 — 광대역·고온·금속 반사",
        12.0, 250.0, 8.0, 0.04, 0.45 },
    { "CHM", "석유화학 / 배터리 공장",
        "가변속 드라이브(VFD), 정유 전기설 — 연속파+하모닉",
        9.0, 180.0, 7.0, 0.03, 0.30 },
    { "MNG", "광산 / 채굴",
        "중장비 인버터, 장거리 전원선, 지하 다중경로",
        7.0, 90.0, 4.0, 0.06, 0.40 },
};

// -------------------------------------------------------------------------
//  B. 슬롯형 다중접속 간단 모델 (AMI 업링크 경쟁)
//  - N 노드가 동일 채널에서 슬롯마다 확률 p 로 전송 시도
//  - 동일 슬롯에 2건 이상이면 충돌(손실), 1건이면 수신 시도
//  - 수신 성공: SINR > 임계 (간단 로그 정규 페이딩)
// -------------------------------------------------------------------------
struct MacSimResult {
    uint64_t slots = 0;
    uint64_t attempts = 0;
    uint64_t collisions = 0;
    uint64_t successes = 0;
    double   air_util = 0.0;   ///< 성공 슬롯 / 전체
    double   goodput_norm = 0.0; ///< 시도 대비 성공
};

static MacSimResult run_slotted_aloha_sim(
    uint32_t num_nodes,
    uint64_t num_slots,
    double p_attempt_per_slot,
    double sinr_threshold_db,
    double site_stress, ///< kSites 프로파일에서 합성한 스칼라 (0.5~2.0)
    unsigned seed)
{
    MacSimResult r;
    r.slots = num_slots;
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    std::lognormal_distribution<double> fading(0.0, 0.35 * site_stress);

    for (uint64_t s = 0; s < num_slots; ++s) {
        int txers = 0;
        for (uint32_t n = 0; n < num_nodes; ++n) {
            if (u01(rng) < p_attempt_per_slot) {
                ++txers;
                ++r.attempts;
            }
        }
        if (txers == 0) {
            continue;
        }
        if (txers > 1) {
            r.collisions += static_cast<uint64_t>(txers);
            continue;
        }
        // 단일 전송: 페이딩 + 간섭 스트레스
        const double sinr_db = 12.0 + fading(rng) * 10.0 - 8.0 * (site_stress - 1.0);
        if (sinr_db >= sinr_threshold_db) {
            ++r.successes;
        }
    }
    r.air_util = (num_slots > 0)
        ? static_cast<double>(r.successes) / static_cast<double>(num_slots)
        : 0.0;
    r.goodput_norm = (r.attempts > 0)
        ? static_cast<double>(r.successes) / static_cast<double>(r.attempts)
        : 0.0;
    return r;
}

static double site_stress_scalar(const IndustrialSiteProfile& p) {
    // 정규화된 "현장 열악도" (대략 0.8 ~ 2.2)
    return 0.5
        + p.emi_floor_db * 0.04
        + std::min(400.0, p.impulse_rate_hz) * 0.0015
        + p.broadband_jam_db * 0.05
        + p.voltage_sag_rate * 3.0
        + p.metal_multipath * 0.5;
}

// -------------------------------------------------------------------------
//  C. 메쉬 안정성 (간단 체인: 게이트웨이 — 중계 — … — 단말)
//  링크 가용률을 랜덤으로 흔들고, 경로 가용 = 곱셈 모델
// -------------------------------------------------------------------------
struct MeshStabilityResult {
    double path_availability_mean = 0.0;
    double path_availability_p10 = 0.0;
    double churn_events_per_kh = 0.0;
};

static MeshStabilityResult run_mesh_chain_sim(
    int hops,
    double base_link_up,
    double industrial_extra_flap,
    int trials,
    unsigned seed)
{
    MeshStabilityResult out;
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    std::vector<double> avail(static_cast<size_t>(trials));

    for (int t = 0; t < trials; ++t) {
        double p = 1.0;
        for (int h = 0; h < hops; ++h) {
            const double up = std::max(0.0, std::min(1.0,
                base_link_up - industrial_extra_flap * u01(rng)));
            p *= up;
        }
        avail[static_cast<size_t>(t)] = p;
    }
    std::sort(avail.begin(), avail.end());
    double sum = 0.0;
    for (double v : avail) { sum += v; }
    out.path_availability_mean = sum / static_cast<double>(trials);
    const size_t p10idx = static_cast<size_t>(static_cast<double>(trials) * 0.10);
    out.path_availability_p10 = avail[p10idx];
    // "체인"이 순간적으로 끊기는 횟수를 대략 추정
    double flap = 0.0;
    for (int t = 1; t < trials; ++t) {
        if (std::abs(avail[static_cast<size_t>(t)] - avail[static_cast<size_t>(t - 1)]) > 0.15) {
            flap += 1.0;
        }
    }
    out.churn_events_per_kh = flap * (1000.0 / static_cast<double>(trials));
    return out;
}

// -------------------------------------------------------------------------
//  D. 음성(세션) — 패킷 손실률·지터로 MOS 추정 (참고용 단순식)
// -------------------------------------------------------------------------
static double estimate_voice_mos(double packet_loss_ratio, double jitter_ms) {
    // ITU-T G.107 E-model의 극단 단순화 아님 — 현장 리포트용 참고 곡선
    const double pl = std::max(0.0, std::min(0.5, packet_loss_ratio));
    const double j = std::max(0.0, jitter_ms);
    double r = 90.0 - 120.0 * pl - 0.5 * j;
    r = std::max(0.0, std::min(100.0, r));
    const double mos = 1.0 + 0.035 * r + 7e-6 * r * (r - 60.0) * (100.0 - r);
    return std::max(1.0, std::min(4.5, mos));
}

// -------------------------------------------------------------------------
//  E. 위치 오차 — 거리(또는 ToA) 잡음 σ → 대략 CEP (2-D, 앵커 3개 가정)
// -------------------------------------------------------------------------
static double estimate_cep95_m(double ranging_sigma_m) {
    const double cep50 = ranging_sigma_m * 1.2;
    const double cep95 = cep50 * 2.1;
    return cep95;
}

// -------------------------------------------------------------------------
//  F. 실제 모듈 스모크 (링크 검증)
// -------------------------------------------------------------------------
static int g_pass = 0, g_fail = 0;

static void check_bool(const char* desc, bool ok) {
    if (ok) {
        ++g_pass;
        std::cout << "  " << std::left << std::setw(52) << desc << " | PASS\n";
    }
    else {
        ++g_fail;
        std::cout << "  " << std::left << std::setw(52) << desc << " | FAIL\n";
    }
}

static std::string truncate_bytes(const char* s, size_t max_b) {
    if (s == nullptr) { return {}; }
    const size_t n = std::strlen(s);
    if (n <= max_b) { return std::string(s); }
    return std::string(s, max_b) + "...";
}

static void section_live_stack_smoke() {
    h2("[표 5] 실제 스택 스모크 (Meter_Data_Manager / Mesh_Router)");
    std::cout << "  " << std::left << std::setw(52) << "항목" << " | 판정\n";
    table_rule('-');

    HTS_Priority_Scheduler sched;
    HTS_Meter_Data_Manager meter(0xA001u);

    MeterReading rd = {};
    rd.cumul_kwh_x100 = 123450u;
    rd.voltage_x10 = 2180u;
    rd.current_x100 = 3200u;
    rd.watt_hour = 1500u;
    rd.power_factor = 97u;
    rd.valid = 1u;
    meter.Update_Reading(rd);
    const MeterReading g = meter.Get_Latest();
    check_bool("계량 최신값 유효 플래그", g.valid == 1u);
    check_bool("누적 kWh*100 일치", g.cumul_kwh_x100 == 123450u);

    meter.Log_Event(MeterEvent::POWER_OFF, 1000u);
    meter.Log_Event(MeterEvent::POWER_ON, 2000u);
    MeterLogEntry ev[HTS_Meter_Data_Manager::EVENT_LOG_SIZE] = {};
    const size_t ne = meter.Get_Event_Log(ev, HTS_Meter_Data_Manager::EVENT_LOG_SIZE);
    check_bool("이벤트 로그 2건 이상", ne >= 2u);

    meter.Tick(0u, sched);
    meter.Tick(HTS_Meter_Data_Manager::REPORT_INTERVAL_MS, sched);
    meter.Shutdown();

    HTS_Mesh_Router router(0xB000u);
    const uint8_t pl[] = { 0x55, 0xAA, 0x10 };
    check_bool("초기 무경로 NO_ROUTE",
        router.Forward(0xC001u, pl, sizeof(pl), 8u, 5000u, sched)
        == FwdResult::NO_ROUTE);
    router.On_Link_Up(0xC001u, 88u);
    RouteEntry re = {};
    check_bool("직접 이웃 라우트 등록", router.Get_Route(0xC001u, re));
    const FwdResult fr = router.Forward(0xC001u, pl, sizeof(pl), 8u, 6000u, sched);
    check_bool("포워딩 시도 (OK 또는 큐포화)",
        fr == FwdResult::OK || fr == FwdResult::QUEUE_FULL);
    router.Shutdown();
}

} // namespace

// =========================================================================
//  main
// =========================================================================
int main() {
    int fail_total = 0;

    h1("AMI 산업 극한 환경 — 현장 근사 시뮬레이션 리포트");

    std::cout << "  본 프로그램은 실제 전파를 측정하지 않습니다.\n"
        << "  현장 계측(스펙트럼 분석기, 드라이브 테스트, 전력품질 분석기)과\n"
        << "  병행할 때, 목표 KPI 대비 갭을 줄이는 용도로 사용하십시오.\n\n";

    // --- [표 1] 부지 프로파일
    h2("[표 1] 산업 부지 환경 파라미터 (모델 입력)");
    std::cout << "  [KPI] 스트레스 <= " << fmt_fixed(kKpi_max_site_stress, 2)
        << " 이면 PASS (열악도 상한)\n";
    int t1_ok = 0, t1_ng = 0;
    std::cout << "  " << std::left
        << std::setw(5) << "ID"
        << " | " << std::setw(22) << "현장"
        << " | " << std::setw(8) << "EMI(dB)"
        << " | " << std::setw(9) << "펄스(Hz)"
        << " | " << std::setw(8) << "광대역(dB)"
        << " | " << std::setw(8) << "전압강하"
        << " | " << std::setw(6) << "다중경로"
        << " | " << std::setw(7) << "스트레스"
        << " | " << std::setw(5) << "판정"
        << "\n";
    table_rule('-');
    for (const auto& s : kSites) {
        const double str = site_stress_scalar(s);
        const bool ok = (str <= kKpi_max_site_stress);
        if (ok) { ++t1_ok; } else { ++t1_ng; }
        std::cout << "  " << std::left << std::setw(5) << s.id
            << " | " << std::setw(22) << s.name_kr
            << " | " << std::right << std::setw(8) << fmt_fixed(s.emi_floor_db, 1)
            << " | " << std::setw(9) << fmt_fixed(s.impulse_rate_hz, 0)
            << " | " << std::setw(8) << fmt_fixed(s.broadband_jam_db, 1)
            << " | " << std::setw(8) << fmt_fixed(s.voltage_sag_rate, 3)
            << " | " << std::setw(6) << fmt_fixed(s.metal_multipath, 2)
            << " | " << std::setw(7) << fmt_fixed(str, 2)
            << " | " << std::setw(5) << pass_fail(ok)
            << "\n";
    }
    table_rule('-');
    std::cout << "  [표1 요약] PASS " << t1_ok << " / FAIL " << t1_ng << "\n";
    if (t1_ng > 0) { fail_total += t1_ng; }
    std::cout << std::left << "  [비고] (참고 문구 — PASS/FAIL 대상 아님)\n";
    for (const auto& s : kSites) {
        std::cout << "  " << std::setw(5) << s.id << " | "
            << truncate_bytes(s.notes, 100u) << "\n";
    }
    std::cout << "\n";

    // --- [표 2] AMI 1024대
    const uint32_t kAmiNodes = 1024u;
    const uint64_t kSlots = 200000ull;
    const double p_tx = 0.00035;
    const double sinr_th = 6.0;

    h2("[표 2] AMI 1024대 · 공용 채널 슬롯 시뮬 (Monte Carlo)");
    std::cout << "  [KPI] 공중이용률 >= " << fmt_fixed(kKpi_min_air_util, 3)
        << " 이고 시도대비 >= " << fmt_fixed(kKpi_min_goodput, 2)
        << " 이면 PASS\n";
    int t2_ok = 0, t2_ng = 0;
    std::cout << "  " << std::left
        << std::setw(5) << "ID"
        << " | " << std::setw(22) << "현장"
        << " | " << std::setw(8) << "스트레스"
        << " | " << std::setw(12) << "송신시도"
        << " | " << std::setw(12) << "충돌합"
        << " | " << std::setw(10) << "성공"
        << " | " << std::setw(10) << "공중이용률"
        << " | " << std::setw(10) << "시도대비"
        << " | " << std::setw(5) << "판정"
        << "\n";
    table_rule('-');
    std::cout << "  [시뮬 조건] 노드=" << kAmiNodes
        << "  슬롯=" << kSlots
        << "  P(송신/슬롯/노드)=" << fmt_fixed(p_tx, 5)
        << "  SINR임계=" << fmt_fixed(sinr_th, 1) << " dB\n";
    table_rule('.');
    for (const auto& site : kSites) {
        const double stress = site_stress_scalar(site);
        const MacSimResult mac = run_slotted_aloha_sim(
            kAmiNodes, kSlots, p_tx, sinr_th, stress,
            20260402u ^ static_cast<unsigned>(site.id[0]));
        const bool ok = (mac.air_util >= kKpi_min_air_util)
            && (mac.goodput_norm >= kKpi_min_goodput);
        if (ok) { ++t2_ok; } else { ++t2_ng; }
        std::cout << "  " << std::left << std::setw(5) << site.id
            << " | " << std::setw(22) << site.name_kr
            << " | " << std::right << std::setw(8) << fmt_fixed(stress, 2)
            << " | " << std::setw(12) << fmt_uint(mac.attempts)
            << " | " << std::setw(12) << fmt_uint(mac.collisions)
            << " | " << std::setw(10) << fmt_uint(mac.successes)
            << " | " << std::setw(10) << fmt_fixed(mac.air_util, 4)
            << " | " << std::setw(10) << fmt_fixed(mac.goodput_norm, 4)
            << " | " << std::setw(5) << pass_fail(ok)
            << "\n";
    }
    table_rule('-');
    std::cout << "  [표2 요약] PASS " << t2_ok << " / FAIL " << t2_ng << "\n\n";
    if (t2_ng > 0) { fail_total += t2_ng; }

    // --- [표 3] 메쉬
    h2("[표 3] 메쉬 경로 안정성 (4홉 체인, 시행 5000)");
    std::cout << "  [KPI] 평균>=" << fmt_fixed(kKpi_min_path_mean, 2)
        << " & P10>=" << fmt_fixed(kKpi_min_path_p10, 2)
        << " & 변동<=" << fmt_fixed(kKpi_max_churn_1k, 1)
        << " 이면 PASS\n";
    int t3_ok = 0, t3_ng = 0;
    std::cout << "  " << std::left
        << std::setw(5) << "ID"
        << " | " << std::setw(22) << "현장"
        << " | " << std::setw(12) << "경로가용(평균)"
        << " | " << std::setw(12) << "경로가용(P10)"
        << " | " << std::setw(14) << "체인변동(/1k)"
        << " | " << std::setw(5) << "판정"
        << "\n";
    table_rule('-');
    for (const auto& site : kSites) {
        const double extra = 0.08 + site.metal_multipath * 0.12;
        const MeshStabilityResult mesh = run_mesh_chain_sim(
            4, 0.94, extra, 5000, 20260403u + static_cast<unsigned>(site.id[1]));
        const bool ok = (mesh.path_availability_mean >= kKpi_min_path_mean)
            && (mesh.path_availability_p10 >= kKpi_min_path_p10)
            && (mesh.churn_events_per_kh <= kKpi_max_churn_1k);
        if (ok) { ++t3_ok; } else { ++t3_ng; }
        std::cout << "  " << std::left << std::setw(5) << site.id
            << " | " << std::setw(22) << site.name_kr
            << " | " << std::right << std::setw(12) << fmt_fixed(mesh.path_availability_mean, 3)
            << " | " << std::setw(12) << fmt_fixed(mesh.path_availability_p10, 3)
            << " | " << std::setw(14) << fmt_fixed(mesh.churn_events_per_kh, 3)
            << " | " << std::setw(5) << pass_fail(ok)
            << "\n";
    }
    table_rule('-');
    std::cout << "  [표3 요약] PASS " << t3_ok << " / FAIL " << t3_ng << "\n\n";
    if (t3_ng > 0) { fail_total += t3_ng; }

    // --- [표 4] 음성·위치
    h2("[표 4] 음성(추정 MOS) · 위치(CEP95)");
    std::cout << "  [KPI] PL<=" << fmt_fixed(kKpi_max_pl, 3)
        << " & Jitter<=" << fmt_fixed(kKpi_max_jitter_ms, 1)
        << " & MOS>=" << fmt_fixed(kKpi_min_mos, 2)
        << " & CEP95<=" << fmt_fixed(kKpi_max_cep95_m, 1)
        << "m 이면 PASS\n";
    int t4_ok = 0, t4_ng = 0;
    std::cout << "  " << std::left
        << std::setw(5) << "ID"
        << " | " << std::setw(22) << "현장"
        << " | " << std::setw(8) << "PL"
        << " | " << std::setw(10) << "Jitter(ms)"
        << " | " << std::setw(9) << "MOS~"
        << " | " << std::setw(10) << "거리σ(m)"
        << " | " << std::setw(10) << "CEP95(m)"
        << " | " << std::setw(5) << "판정"
        << "\n";
    table_rule('-');
    for (const auto& site : kSites) {
        const double pl = 0.02 + site.broadband_jam_db * 0.002 + site.voltage_sag_rate * 0.15;
        const double jitter = 8.0 + site.impulse_rate_hz * 0.01;
        const double mos = estimate_voice_mos(pl, jitter);
        const double rng_sigma = 4.0 + site.emi_floor_db * 0.35 + site.metal_multipath * 12.0;
        const double cep95 = estimate_cep95_m(rng_sigma);
        const bool ok = (pl <= kKpi_max_pl)
            && (jitter <= kKpi_max_jitter_ms)
            && (mos >= kKpi_min_mos)
            && (cep95 <= kKpi_max_cep95_m);
        if (ok) { ++t4_ok; } else { ++t4_ng; }
        std::cout << "  " << std::left << std::setw(5) << site.id
            << " | " << std::setw(22) << site.name_kr
            << " | " << std::right << std::setw(8) << fmt_fixed(pl, 4)
            << " | " << std::setw(10) << fmt_fixed(jitter, 1)
            << " | " << std::setw(9) << fmt_fixed(mos, 2)
            << " | " << std::setw(10) << fmt_fixed(rng_sigma, 1)
            << " | " << std::setw(10) << fmt_fixed(cep95, 1)
            << " | " << std::setw(5) << pass_fail(ok)
            << "\n";
    }
    table_rule('-');
    std::cout << "  [표4 요약] PASS " << t4_ok << " / FAIL " << t4_ng << "\n\n";
    if (t4_ng > 0) { fail_total += t4_ng; }

    section_live_stack_smoke();

    h1("요약");
    std::cout << "  - 대규모 AMI: 동시 송신 충돌·SINR 저하 병목 -> 주파수 계획·GW 밀도·DRX\n"
        << "  - 변전/제철: 펄스·광대역 간섭 -> 음성·동기·계량 버스트 동시 흔들림 가능\n"
        << "  - 메쉬: 홉 증가 시 가용률 곱셈 감소 -> 홉 수·백업 경로를 POC에 명시\n"
        << "  - 실측: 전파·전력품질·스펙트럼 계측값을 모델 입력으로 치환 시 정밀도 향상\n\n";

    std::cout << "  [전체 표 요약] 표1: PASS " << t1_ok << "/" << (t1_ok + t1_ng)
        << "  표2: PASS " << t2_ok << "/" << (t2_ok + t2_ng)
        << "  표3: PASS " << t3_ok << "/" << (t3_ok + t3_ng)
        << "  표4: PASS " << t4_ok << "/" << (t4_ok + t4_ng)
        << "  표5(항목): PASS " << g_pass << "/" << (g_pass + g_fail) << "\n";
    std::cout << "  [판정 합계] 행 단위 FAIL " << fail_total
        << " 건 + 표5 FAIL " << g_fail << " 건\n";
    table_rule('=');

    const bool all_ok = (fail_total == 0) && (g_fail == 0);
    std::cout << "  최종: " << (all_ok ? "전부 PASS" : "FAIL 포함") << "\n";
    table_rule('=');

    return all_ok ? 0 : 1;
}
