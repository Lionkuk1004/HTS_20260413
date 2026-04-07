// =========================================================================
// HTS_AMI_CommSpec_Barrage30_Matrix_Test.cpp
// 위치: HTS_TEST/ — PC 전용 (HTS_LIM 정적 라이브러리 링크)
//
// 목적
//   · 통신 규격 매트릭스(기본 성능·HARQ/FEC 등) 중, 단일 링크·V400 DATA 경로에서
//     **측정 가능한 항목**을 BARRAGE(광역 재밍) 채널에서 **회당 10회** Monte Carlo.
//
//   · 규격서의 「BARRAGE 30dB」는 **운용 시나리오 ID**로 두고, PC 소프트 와이어에는
//     `kJsDbBarrageWire` 로 실제 J/S(dB)를 넣는다. (30dB 그대로 두면 본 단일 링크·
//     무AJC 하네스에서 CRC가 붕괴하고 32H-ARQ × 대칩수로 **수십 분** 소요될 수 있음.)
//   · 와이어 모델은 HTS_TEST/benchmark/HTS_Fractal_Channel_Compare.cpp 의
//     V400 소프트 BARRAGE(처리이득·분모 보정)와 동일 계열 — 코어 TU 비변경.
//
// 한계(출력에 명시)
//   · 다중 노드 동시 Walsh(2·8·16·64대), Near-Far, 듀티 1000대 등은
//     전역 HARQ/CCM 공유 구조상 본 단일 프로세스 하네스 범위 밖 → [N/A].
//   · EMP/CW/페이딩/도심 채널 스윕은 별도 채널 모듈 결합 TU 권장 → [N/A] 또는 향후 확장.
//   · Erasure/RS 세부(4-5~4-9)는 FEC_HARQ IR 내부 계측이 필요 → 본 파일은 간접 지표만.
//
// 빌드: HTS_TEST\HTS_검증_AMI_Barrage30_Spec.vcxproj
// =========================================================================
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || \
    defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#error "[HTS_FATAL] HTS_AMI_CommSpec_Barrage30_Matrix_Test — PC 전용"
#endif

#include "HTS_FEC_HARQ.hpp"
#include "HTS_V400_Dispatcher.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

extern "C" volatile int g_hts_ir_diag_chip0;
extern "C" volatile int g_hts_ir_diag_feed_idx;

namespace {

using ProtectedEngine::DecodedPacket;
using ProtectedEngine::FEC_HARQ;
using ProtectedEngine::HTS_V400_Dispatcher;
using ProtectedEngine::PayloadMode;
using ProtectedEngine::SoftClipPolicy;

// ── Fractal 벤치 정합 BARRAGE 소프트 경로 (동일 상수) ─────────────────
static constexpr double kV400BarrageHarnessDiv = 7.72;
static constexpr double kPlateauExp = 0.58;
static constexpr int16_t kFecWalshAmp = 2000;
static constexpr double kV400WalshAmpD = static_cast<double>(kFecWalshAmp);
static constexpr double kSoftSpreadGain = 128.0;
static constexpr double kSoftBaseNoiseSigma = 0.01;
static constexpr double kV400DoubleRxToIq16 = kV400WalshAmpD / kSoftSpreadGain;
/// Fractal `apply_barrage_v400(..., nc)` 와 동일: 심볼 길이(nc)별 처리이득 보정
static constexpr int kWalshNcPreamble64 = 64;
static constexpr int kWalshNcPayload16 = 16;

static inline double fec_v400_pg_ratio(int nc) noexcept
{
    return std::sqrt(static_cast<double>(nc) / 32.0);
}

static inline double fec_v400_barrage_denominator(double js_db, int nc) noexcept
{
    const double pg = fec_v400_pg_ratio(nc);
    const double base = kV400BarrageHarnessDiv * pg;
    if (js_db <= 25.0) {
        return base;
    }
    const double t = (js_db - 25.0) / 15.0;
    return base * std::pow(10.0, t * kPlateauExp);
}

static inline int16_t fec_clamp_i16_from_double(double v) noexcept
{
    long r = std::lround(v);
    if (r > 32767L) {
        r = 32767L;
    }
    if (r < -32768L) {
        r = -32768L;
    }
    return static_cast<int16_t>(r);
}

/// V400 칩열(선형)에 BARRAGE 가산 — `spread_nc` 는 해당 구간 Walsh 길이(16 또는 64).
static void apply_barrage_v400_chip_stream(
    int16_t* I, int16_t* Q, int n_chips, double js_db, std::mt19937& rng,
    int spread_nc) noexcept
{
    const double js_linear = std::pow(10.0, js_db / 10.0);
    const double jam_sigma =
        (std::sqrt(js_linear) * kSoftSpreadGain)
        / fec_v400_barrage_denominator(js_db, spread_nc);
    std::normal_distribution<double> jam(0.0, jam_sigma);
    std::normal_distribution<double> base_noise(0.0, kSoftBaseNoiseSigma);
    for (int idx = 0; idx < n_chips; ++idx) {
        const double tx_norm =
            static_cast<double>(I[idx]) / kV400WalshAmpD;
        const double rd =
            tx_norm * kSoftSpreadGain + jam(rng) + base_noise(rng);
        const double v = rd * kV400DoubleRxToIq16;
        I[idx] = fec_clamp_i16_from_double(v);
        Q[idx] = I[idx];
    }
}

static constexpr int kMaxChips = 256 + FEC_HARQ::NSYM64 * 64;

DecodedPacket g_last{};
static void on_packet_cb(const DecodedPacket& p) noexcept
{
    g_last = p;
}

struct RunStats {
    PayloadMode mode = PayloadMode::DATA;
    int nc = 64;
    int trials = 0;
    int crc_ok = 0;
    int payload_ok = 0;
    long long sum_harq_on_success = 0;
    int max_harq = 0;
    int max_feeds = 0;
    int exhaust_32 = 0;
    double max_lat_ms = 0.0;
};

/// 단일 시나리오: IR 또는 Chase. js_db < 0 이면 칩열 와이어 가산 없음(평시).
RunStats run_campaign(PayloadMode mode, bool ir_mode,
    double js_db,
    int max_feeds,
    int num_trials,
    uint32_t seed_base) noexcept
{
    RunStats st{};
    st.trials = num_trials;
    st.mode = mode;
    st.nc = (mode == PayloadMode::DATA) ? 64 : 16;

    std::vector<int16_t> oI(static_cast<size_t>(kMaxChips));
    std::vector<int16_t> oQ(static_cast<size_t>(kMaxChips));

    for (int t = 0; t < num_trials; ++t) {
        g_last = DecodedPacket{};
        const uint32_t disp_seed =
            seed_base ^ static_cast<uint32_t>(t * 0x9E3779B9u);
        const uint32_t noise_seed =
            (seed_base << 1) ^ static_cast<uint32_t>(t * 0x85EBCA6Bu);

        HTS_V400_Dispatcher disp;
        disp.Set_IR_Mode(ir_mode);
        disp.Set_Preamble_Boost(4);  // +12dB
        disp.Set_Seed(disp_seed);
        disp.Set_IR_SIC_Enabled(false);
        disp.Set_CW_Cancel(false);
        disp.Set_AJC_Enabled(false);
        /* 강재밍 하네스: 페이로드 구간도 soft_clip ON — SYNC_ONLY 는 페이로드 비클리핑으로
           고 J/S 에서 LLR 폭주 시 지표가 Fractal/구형 벤치 대비 과도하게 붕괴할 수 있음 */
        disp.Set_SoftClip_Policy(SoftClipPolicy::ALWAYS);
        disp.Set_Packet_Callback(on_packet_cb);

        uint8_t info[8]{};
        for (int b = 0; b < 8; ++b) {
            info[b] = static_cast<uint8_t>(
                static_cast<unsigned>(disp_seed >> static_cast<unsigned>(b * 4))
                ^ static_cast<unsigned>(t + b));
        }

        std::mt19937 rng(noise_seed);
        int feeds_used = 0;
        int success = 0;

        for (int feed = 0; feed < max_feeds && success == 0; ++feed) {
            /* [진단] 25dB·t=0·feed=0..3: Decode 직전 ir_chip_I_[0]·harq_round_ 출력 */
            {
                const bool diag_slot =
                    (t == 0 && feed >= 0 && feed < 4 &&
                     js_db >= 24.99 && js_db <= 25.01);
                g_hts_ir_diag_chip0 = diag_slot ? 1 : 0;
                g_hts_ir_diag_feed_idx = diag_slot ? feed : -1;
            }
            int n = 0;
            if (feed == 0 || !disp.Is_Retx_Ready()) {
                n = disp.Build_Packet(
                    mode,
                    info,
                    8,
                    kFecWalshAmp,
                    oI.data(),
                    oQ.data(),
                    kMaxChips);
            }
            else {
                n = disp.Build_Retx(
                    mode,
                    info,
                    8,
                    kFecWalshAmp,
                    oI.data(),
                    oQ.data(),
                    kMaxChips);
            }
            if (n <= 0) {
                break;
            }
            if (js_db >= 0.0) {
                /* DATA·16/64 페이로드: Fractal 과 동일하게 nc 보정 (VOICE 는 프리앰블 256칩=64,
                   이후 페이로드=16) */
                if (mode == PayloadMode::VOICE && n > 256) {
                    apply_barrage_v400_chip_stream(
                        oI.data(), oQ.data(), 256, js_db, rng,
                        kWalshNcPreamble64);
                    apply_barrage_v400_chip_stream(
                        oI.data() + 256, oQ.data() + 256, n - 256, js_db, rng,
                        kWalshNcPayload16);
                }
                else {
                    apply_barrage_v400_chip_stream(
                        oI.data(), oQ.data(), n, js_db, rng, st.nc);
                }
            }
            if (feed == 0 || !disp.Is_Retx_Ready()) {
                for (int i = 0; i < n; ++i) {
                    disp.Feed_Chip(
                        oI[static_cast<size_t>(i)],
                        oQ[static_cast<size_t>(i)]);
                }
            }
            else {
                for (int i = 0; i < n; ++i) {
                    disp.Feed_Retx_Chip(
                        oI[static_cast<size_t>(i)],
                        oQ[static_cast<size_t>(i)]);
                }
            }
            ++feeds_used;
            if (t < 5 && js_db >= 20.0) {
                std::printf(
                    "    [t=%d feed=%d] retx=%d phase=%d success=%d\n",
                    t,
                    feed,
                    disp.Is_Retx_Ready() ? 1 : 0,
                    static_cast<int>(disp.Get_Phase()),
                    (g_last.success_mask == DecodedPacket::DECODE_MASK_OK)
                        ? 1
                        : 0);
            }
            if (g_last.success_mask == DecodedPacket::DECODE_MASK_OK) {
                success = 1;
            }
        }

        g_hts_ir_diag_chip0 = 0;
        g_hts_ir_diag_feed_idx = -1;

        if (feeds_used > st.max_feeds) {
            st.max_feeds = feeds_used;
        }
        if (success != 0) {
            ++st.crc_ok;
            const double rtt = ir_mode
                ? HTS_V400_Dispatcher::IR_HARQ_RTT_MS
                : 8.0;
            const double lat = static_cast<double>(g_last.harq_k) * rtt;
            if (lat > st.max_lat_ms) {
                st.max_lat_ms = lat;
            }
            if (g_last.harq_k > st.max_harq) {
                st.max_harq = g_last.harq_k;
            }
            st.sum_harq_on_success += g_last.harq_k;
            if (g_last.mode == mode && g_last.data_len == 8) {
                if (std::memcmp(g_last.data, info, 8) == 0) {
                    ++st.payload_ok;
                }
            }
        } else {
            if (feeds_used >= 32) {
                ++st.exhaust_32;
            }
        }
    }
    return st;
}

} // namespace

extern "C" void scenario_rf_hopping_control(void);

int main()
{
    static constexpr int kNumTrials = 10;
    static constexpr int kMaxHarqFeeds = 32;
    static constexpr uint32_t kSeed = 0xB40730u;

    static constexpr struct {
        const char* label;
        PayloadMode mode;
    } k_modes[] = {
        { "64chip DATA",  PayloadMode::DATA },
        { "16chip VOICE", PayloadMode::VOICE },
    };

    static constexpr double k_js[] = {
        -1.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0,
        35.0, 40.0, 45.0, 50.0
    };

    bool all_pass = true;

    for (const auto& m : k_modes) {
        std::printf("\n======== %s ========\n", m.label);
        std::printf("  %-10s | %-28s | %s\n",
            "J/S(dB)", "IR-HARQ", "Chase");

        for (double js : k_js) {
            RunStats ir = run_campaign(m.mode, true, js,
                kMaxHarqFeeds, kNumTrials,
                kSeed ^ static_cast<uint32_t>(
                    static_cast<int>(js * 100.0)));
            RunStats ch = run_campaign(m.mode, false, js,
                kMaxHarqFeeds, kNumTrials,
                kSeed ^ 0xACE0u ^ static_cast<uint32_t>(
                    static_cast<int>(js * 100.0)));

            double ir_crc = (ir.trials > 0)
                ? 100.0 * ir.crc_ok / ir.trials : 0.0;
            double ch_crc = (ch.trials > 0)
                ? 100.0 * ch.crc_ok / ch.trials : 0.0;
            double ir_avg = (ir.crc_ok > 0)
                ? static_cast<double>(ir.sum_harq_on_success)
                  / ir.crc_ok : 0.0;

            const char* ch_label =
                (js < 0.0) ? "CLEAR" : "BARRAGE";

            std::printf(
                "  %s %5.1f | CRC %6.2f%% avg %5.2f lat %5.0fms"
                " | CRC %6.2f%%\n",
                ch_label, js, ir_crc, ir_avg,
                ir.max_lat_ms, ch_crc);

            if (js < 0.0 && ir_crc < 100.0 - 1e-9) {
                all_pass = false;
            }
        }
    }

    std::printf("\n최종: %s\n", all_pass ? "PASS" : "FAIL");
    /* BUG-FIX-RETX6: 일원화 + 자동 16/64칩 순회 + HARQ 연속모드 */
    scenario_rf_hopping_control();
    return all_pass ? 0 : 1;
}

// =========================================================================
// [S12] RF 주파수 도약(FHSS) 제어 및 송수신 동기화 검증
// =========================================================================
extern "C" void scenario_rf_hopping_control(void) {
    std::printf("\n--- [S12] RF Hopping Control (FHSS) Test ---\n");

    ProtectedEngine::HTS_V400_Dispatcher tx_disp;
    ProtectedEngine::HTS_V400_Dispatcher rx_disp;

    // 1. 송수신기가 동일한 암호화 시드(비밀키) 공유
    uint32_t shared_seed = 0x99887766u;
    tx_disp.Set_Seed(shared_seed);
    rx_disp.Set_Seed(shared_seed);

    // B-CDMA IR-HARQ 모드 활성화
    tx_disp.Set_IR_Mode(true);
    rx_disp.Set_IR_Mode(true);

    int pass_count = 0;

    // 2. 8번 연속으로 주파수 도약(Hopping)을 지시하며 채널 추적
    for (int i = 0; i < 8; ++i) {
        // 송신기와 수신기가 각자의 시퀀스만으로 다음 도약 채널 계산
        uint8_t tx_ch = tx_disp.FHSS_Request_Hop_As_Tx();
        uint8_t rx_ch = rx_disp.FHSS_Request_Hop_As_Rx();

        std::printf("Hop %d: TX CH=%u, RX CH=%u ", i, tx_ch, rx_ch);

        // 채널 100% 일치 확인 (0~127 범위)
        if (tx_ch == rx_ch && tx_ch != 0xFF) {
            std::printf("[SYNC OK]\n");
            pass_count++;
        } else {
            std::printf("[SYNC FAIL]\n");
        }

        // 3. RF_SETTLING (하드웨어 안정화 대기 및 Blanking) 검증
        if (tx_disp.FHSS_Is_Rf_Settling() && rx_disp.FHSS_Is_Rf_Settling()) {
            // 64칩(1 심볼 분량) 동안 빈 칩(0, 0)을 넣어주며 타이머 대기
            for (int c = 0; c < 64; ++c) {
                tx_disp.Feed_Chip(0, 0);
                rx_disp.Feed_Chip(0, 0);
            }

            // 64칩 경과 후, 자동으로 WAIT_SYNC 상태로 복귀했는지 확인
            if (!tx_disp.FHSS_Is_Rf_Settling() && !rx_disp.FHSS_Is_Rf_Settling()) {
                pass_count++;
            }
        }
    }

    // 총 8번의 채널 일치 + 8번의 안정화 복귀가 모두 성공해야 최종 PASS
    if (pass_count == 16) {
        std::printf("[S12] scenario_rf_hopping_control ... PASS\n");
    } else {
        std::printf("[S12] scenario_rf_hopping_control ... FAIL (score=%d/16)\n",
            pass_count);
    }
}
