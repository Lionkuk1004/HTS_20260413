// HTS_Extended_Endurance_Test.cpp — AMI V2 FEC 하네스 확장 (BPS 래더 + 내구도)
// HTS_AMI_Barrage30_Test_V2.cpp의 Inject + Feed 패턴을 따르되,
// run_trial을 BPS 가변(DATA 3~6)으로 일반화.
// DATA: BPS 3~6 × J/S 래더 × trials=100
// VOICE: 16칩 × J/S 래더 × trials=100
#if defined(__arm__) || defined(__TARGET_ARCH_ARM)
#error "[HTS_FATAL] PC 전용"
#endif

#include "HTS_FEC_HARQ.hpp"
#include "HTS_V400_Dispatcher.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

namespace {

using ProtectedEngine::DecodedPacket;
using ProtectedEngine::FEC_HARQ;
using ProtectedEngine::HTS_V400_Dispatcher;
using ProtectedEngine::PayloadMode;
using ProtectedEngine::SoftClipPolicy;

static constexpr int16_t kAmp = 500;
static constexpr int kMaxChips = 16384;
static constexpr int kMaxFeeds = 32;
static constexpr int kTrials = 100;

static DecodedPacket g_last{};
static void on_pkt(const DecodedPacket& p) noexcept { g_last = p; }

static void add_barrage(int16_t* sI, int16_t* sQ, int n, double js_db,
                        std::mt19937& rng) noexcept {
    if (js_db < 0.01) {
        return;
    }
    const double ja =
        static_cast<double>(kAmp) * std::sqrt(std::pow(10.0, js_db / 10.0));
    const double sigma = ja / std::sqrt(2.0);
    std::normal_distribution<double> nd(0.0, sigma);
    for (int i = 0; i < n; ++i) {
        int32_t vi =
            static_cast<int32_t>(sI[i]) + static_cast<int32_t>(std::lround(nd(rng)));
        int32_t vq =
            static_cast<int32_t>(sQ[i]) + static_cast<int32_t>(std::lround(nd(rng)));
        if (vi > 32767) {
            vi = 32767;
        }
        if (vi < -32768) {
            vi = -32768;
        }
        if (vq > 32767) {
            vq = 32767;
        }
        if (vq < -32768) {
            vq = -32768;
        }
        sI[i] = static_cast<int16_t>(vi);
        sQ[i] = static_cast<int16_t>(vq);
    }
}

// Barrage30 V2와 동일 구조: bps는 DATA 전용(3~6). VOICE는 bps 무시(16칩 경로).
static bool run_trial(PayloadMode mode, int bps, double js_db, uint32_t seed,
                      int& out_harq_k) noexcept {
    const uint32_t il = seed ^ (0u * 0xA5A5A5A5u);

    uint8_t info[8]{};
    for (int b = 0; b < 8; ++b) {
        info[b] = static_cast<uint8_t>((seed >> (b * 4)) ^ b);
    }

    FEC_HARQ::WorkBuf wb{};
    const int nsym = (mode == PayloadMode::DATA) ? FEC_HARQ::nsym_for_bps(bps)
                                                 : FEC_HARQ::NSYM16;
    const int nc = (mode == PayloadMode::DATA) ? 64 : 16;

    std::mt19937 rng(seed ^ 0xBEEF0000u);
    g_last = DecodedPacket{};
    out_harq_k = 0;

    HTS_V400_Dispatcher rx;
    rx.Set_IR_Mode(true);
    rx.Set_Seed(seed);
    rx.Set_Preamble_Reps(4);
    rx.Set_CW_Cancel(false);
    rx.Set_AJC_Enabled(false);
    rx.Set_SoftClip_Policy(SoftClipPolicy::NEVER);
    rx.Set_Packet_Callback(on_pkt);
    rx.Set_Lab_IQ_Mode_Jam_Harness();
    if (mode == PayloadMode::DATA) {
        if (bps < FEC_HARQ::BPS64_MIN_OPERABLE || bps > FEC_HARQ::BPS64_MAX) {
            return false;
        }
        rx.Set_Lab_BPS64(bps);
        rx.Update_Adaptive_BPS(1000u);
    } else {
        rx.Set_Lab_BPS64(4);
        rx.Update_Adaptive_BPS(1000u);
    }

    rx.Inject_Payload_Phase(mode, (mode == PayloadMode::DATA) ? bps : 4);

    std::vector<int16_t> symI(static_cast<size_t>(nsym * nc));
    std::vector<int16_t> symQ(static_cast<size_t>(nsym * nc));

    for (int feed = 0; feed < kMaxFeeds; ++feed) {
        uint8_t syms[FEC_HARQ::NSYM64]{};
        int enc_n = 0;

        std::memset(&wb, 0, sizeof(wb));
        if (mode == PayloadMode::DATA) {
            enc_n = FEC_HARQ::Encode64_IR(info, 8, syms, il, bps, feed & 3, wb);
        } else {
            enc_n = FEC_HARQ::Encode16_IR(info, 8, syms, il, feed & 3, wb);
        }
        if (enc_n <= 0) {
            break;
        }

        for (int s = 0; s < enc_n && s < nsym; ++s) {
            const uint8_t sym = syms[s];
            for (int c = 0; c < nc; ++c) {
                const uint32_t sv = static_cast<uint32_t>(sym);
                const uint32_t cv = static_cast<uint32_t>(c);
                uint32_t x = sv & cv;
                x = x - ((x >> 1u) & 0x55555555u);
                x = (x & 0x33333333u) + ((x >> 2u) & 0x33333333u);
                const uint32_t p =
                    (((x + (x >> 4u)) & 0x0F0F0F0Fu) * 0x01010101u) >> 24u;
                const int16_t chip = static_cast<int16_t>(
                    static_cast<int32_t>(kAmp) *
                    (1 - 2 * static_cast<int32_t>(p & 1u)));
                symI[static_cast<size_t>(s * nc + c)] = chip;
                symQ[static_cast<size_t>(s * nc + c)] = chip;
            }
        }

        add_barrage(symI.data(), symQ.data(), enc_n * nc, js_db, rng);

        if (feed == 0) {
            for (int i = 0; i < enc_n * nc; ++i) {
                rx.Feed_Chip(symI[static_cast<size_t>(i)],
                             symQ[static_cast<size_t>(i)]);
            }
        } else {
            if (rx.Is_Retx_Ready()) {
                for (int i = 0; i < enc_n * nc; ++i) {
                    rx.Feed_Retx_Chip(symI[static_cast<size_t>(i)],
                                      symQ[static_cast<size_t>(i)]);
                }
            } else {
                break;
            }
        }

        if (g_last.success_mask == DecodedPacket::DECODE_MASK_OK) {
            out_harq_k = g_last.harq_k;
            return true;
        }
    }
    return false;
}

// V2와 동일 J/S 격자 (래더)
static constexpr double kJs[] = {0,  5,  10, 15, 20, 25, 28, 30,
                                 32, 35, 38, 40, 42, 45};

// DATA PG: bps4 기준 18.06 dB, 심볼 수 비율로 1차 근사 (표시용)
static double pg_data_nominal(int bps) noexcept {
    const int n = FEC_HARQ::nsym_for_bps(bps);
    const int n4 = FEC_HARQ::nsym_for_bps(4);
    if (n4 <= 0 || n <= 0) {
        return 18.06;
    }
    return 18.06 + 10.0 * std::log10(static_cast<double>(n) /
                                     static_cast<double>(n4));
}

} // namespace

int main() {
    std::printf("═══════════════════════════════════════════════════\n");
    std::printf("  HTS Extended Endurance — BPS 3~6 + VOICE (AMI V2 harness)\n");
    std::printf("  trials=%d max_feeds=%d amp=%d\n", kTrials, kMaxFeeds, kAmp);
    const int bps_lo = FEC_HARQ::BPS64_MIN_OPERABLE;
    const int bps_hi = 6;
    if (bps_lo > 3) {
        std::printf("  [note] BPS64_MIN_OPERABLE=%d — BPS 3..%d skipped on this binary\n",
                    bps_lo, bps_lo - 1);
    }
    std::printf("═══════════════════════════════════════════════════\n\n");

    int total_cells = 0;
    int total_ok = 0;

    for (int bps = bps_lo; bps <= bps_hi; ++bps) {
        const double pg = pg_data_nominal(bps);
        std::printf("── DATA BPS=%d  (nominal PG~%.2f dB) ──\n", bps, pg);
        std::printf("  %5s %7s  %5s %7s %5s\n", "J/S", "eff", "CRC%", "avgH",
                    "maxH");

        for (double js : kJs) {
            int ok = 0;
            int sumh = 0;
            int maxh = 0;
            for (int t = 0; t < kTrials; ++t) {
                const uint32_t seed =
                    0xE501u ^ static_cast<uint32_t>(bps * 0x10000u) ^
                    static_cast<uint32_t>(t * 0x9E3779B9u) ^
                    static_cast<uint32_t>(static_cast<int>(js * 100));
                int hk = 0;
                ++total_cells;
                if (run_trial(PayloadMode::DATA, bps, js, seed, hk)) {
                    ++ok;
                    ++total_ok;
                    sumh += hk;
                    if (hk > maxh) {
                        maxh = hk;
                    }
                }
            }
            std::printf("  %5.0f %+6.1f  %4.0f%% %6.1f  %4d\n", js,
                        js - pg, 100.0 * ok / kTrials,
                        ok > 0 ? static_cast<double>(sumh) / ok : 0.0, maxh);
        }
        std::printf("\n");
    }

    static constexpr double kPgVoice = 12.04;
    std::printf("── VOICE 16chip  PG=%.2f dB ──\n", kPgVoice);
    std::printf("  %5s %7s  %5s %7s %5s\n", "J/S", "eff", "CRC%", "avgH",
                "maxH");
    for (double js : kJs) {
        int ok = 0;
        int sumh = 0;
        int maxh = 0;
        for (int t = 0; t < kTrials; ++t) {
            const uint32_t seed =
                0xE502u ^ static_cast<uint32_t>(t * 0x9E3779B9u) ^
                static_cast<uint32_t>(static_cast<int>(js * 100));
            int hk = 0;
            ++total_cells;
            if (run_trial(PayloadMode::VOICE, 4, js, seed, hk)) {
                ++ok;
                ++total_ok;
                sumh += hk;
                if (hk > maxh) {
                    maxh = hk;
                }
            }
        }
        std::printf("  %5.0f %+6.1f  %4.0f%% %6.1f  %4d\n", js,
                    js - kPgVoice, 100.0 * ok / kTrials,
                    ok > 0 ? static_cast<double>(sumh) / ok : 0.0, maxh);
    }

    std::printf("\n═══════════════════════════════════════════════════\n");
    std::printf("  Aggregate: %d / %d decode successes (cell = J/S point x trial)\n",
                total_ok, total_cells);
    std::printf("  동기/HDR: Inject 경로 (Barrage30 V2와 동일 계열)\n");
    std::printf("═══════════════════════════════════════════════════\n");
    return 0;
}
