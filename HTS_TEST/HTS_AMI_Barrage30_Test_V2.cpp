// HTS_AMI_Barrage30_Test_V2.cpp — FEC/HARQ 직접 테스트 (동기/HDR bypass)
// 동기+HDR은 T6-SIM 40/40으로 검증 완료.
// 이 하네스는 FEC+IR-HARQ 성능만 측정.
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
static constexpr int kTrials = 20;

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

// FEC 직접 테스트: Inject_Payload_Phase로 동기/HDR 우회
// TX는 Build_Packet의 페이로드 부분만 사용
// RX는 Inject_Payload_Phase 후 Feed_Chip/Feed_Retx_Chip
static bool run_trial(PayloadMode mode, double js_db, uint32_t seed,
                      int& out_harq_k) noexcept {
    const int bps = 4;
    const uint32_t il = seed ^ (0u * 0xA5A5A5A5u); // tx_seq=0

    uint8_t info[8]{};
    for (int b = 0; b < 8; ++b) {
        info[b] = static_cast<uint8_t>((seed >> (b * 4)) ^ b);
    }

    // ── TX: Encode만 (Dispatcher 불필요) ──
    FEC_HARQ::WorkBuf wb{};
    const int nsym = (mode == PayloadMode::DATA) ? FEC_HARQ::nsym_for_bps(bps)
                                                 : FEC_HARQ::NSYM16;
    const int nc = (mode == PayloadMode::DATA) ? 64 : 16;

    // IR-HARQ: 각 라운드마다 다른 RV salt로 인코딩
    std::mt19937 rng(seed ^ 0xBEEF0000u);
    g_last = DecodedPacket{};
    out_harq_k = 0;

    // RX Dispatcher (Inject 모드)
    HTS_V400_Dispatcher rx;
    rx.Set_IR_Mode(true);
    rx.Set_Seed(seed);
    rx.Set_Preamble_Reps(4);
    rx.Set_CW_Cancel(false);
    rx.Set_AJC_Enabled(false);
    rx.Set_SoftClip_Policy(SoftClipPolicy::NEVER);
    rx.Set_Packet_Callback(on_pkt);
    rx.Set_Lab_IQ_Mode_Jam_Harness();
    rx.Set_Lab_BPS64(bps);
    rx.Update_Adaptive_BPS(1000u);

    // Inject: 동기/HDR 건너뛰고 READ_PAYLOAD 직접 진입
    rx.Inject_Payload_Phase(mode, bps);

    // 각 라운드: TX encode → 잡음 추가 → RX Feed
    std::vector<int16_t> symI(static_cast<size_t>(nsym * nc));
    std::vector<int16_t> symQ(static_cast<size_t>(nsym * nc));

    for (int feed = 0; feed < kMaxFeeds; ++feed) {
        // TX: IR encode (RV = feed & 3)
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

        // Walsh 변조
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
                symQ[static_cast<size_t>(s * nc + c)] = chip; // I=Q
            }
        }

        // 잡음 추가
        add_barrage(symI.data(), symQ.data(), enc_n * nc, js_db, rng);

        // RX Feed (페이로드 칩만, 동기/HDR 없음)
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
                break; // RX가 retx 준비 안 됨 → 포기
            }
        }

        if (g_last.success_mask == DecodedPacket::DECODE_MASK_OK) {
            out_harq_k = g_last.harq_k;
            return true;
        }
    }
    return false;
}

} // namespace

int main() {
    std::printf("═══════════════════════════════════════════════════\n");
    std::printf("  HTS AMI Barrage30 V2 — FEC/IR-HARQ 직접 테스트\n");
    std::printf("  동기/HDR: T6-SIM 40/40 검증 완료 (bypass)\n");
    std::printf("  trials=%d max_feeds=%d amp=%d\n", kTrials, kMaxFeeds, kAmp);
    std::printf("═══════════════════════════════════════════════════\n\n");

    static constexpr double kJs[] = {0,  5,  10, 15, 20, 25, 28, 30,
                                     32, 35, 38, 40, 42, 45};
    static constexpr PayloadMode kM[] = {PayloadMode::DATA, PayloadMode::VOICE};
    static constexpr const char* kN[] = {"64chip(DATA)", "16chip(VOICE)"};
    static constexpr double kPg[] = {18.06, 12.04};

    for (int mi = 0; mi < 2; ++mi) {
        std::printf("── %s  PG=%.2f dB ──\n", kN[mi], kPg[mi]);
        std::printf("  %5s %7s  %5s %7s %5s\n", "J/S", "eff", "CRC%", "avgH",
                    "maxH");

        for (double js : kJs) {
            int ok = 0;
            int sumh = 0;
            int maxh = 0;
            for (int t = 0; t < kTrials; ++t) {
                const uint32_t seed =
                    0xB40730u ^ static_cast<uint32_t>(t * 0x9E3779B9u) ^
                    static_cast<uint32_t>(static_cast<int>(js * 100));
                int hk = 0;
                if (run_trial(kM[mi], js, seed, hk)) {
                    ++ok;
                    sumh += hk;
                    if (hk > maxh) {
                        maxh = hk;
                    }
                }
            }
            std::printf("  %5.0f %+6.1f  %4.0f%% %6.1f  %4d\n", js,
                        js - kPg[mi], 100.0 * ok / kTrials,
                        ok > 0 ? static_cast<double>(sumh) / ok : 0.0, maxh);
        }
        std::printf("\n");
    }

    std::printf("═══════════════════════════════════════════════════\n");
    std::printf("  동기 한계: T6-SIM Monte Carlo 참조\n");
    std::printf("  AWGN 90%%=+8dB, EMP 90%%=+12dB, Combined 90%%=+12dB\n");
    std::printf("═══════════════════════════════════════════════════\n");
    return 0;
}
