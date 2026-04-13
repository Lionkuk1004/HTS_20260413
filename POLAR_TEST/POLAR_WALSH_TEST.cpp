// =============================================================================
// POLAR_WALSH_TEST.cpp — Polar + Walsh 64칩 통합 테스트
//
// [목적] HTS B-CDMA 파이프라인에서 Polar(512,80) SCL-4 검증
//        Conv+REP4 대비 성능 비교
//
// [파이프라인]
//   TX: data → Polar Encode → BPS 심볼 매핑 → Walsh 64칩 변조
//   CH: Walsh 칩 + AWGN 바라지 잡음
//   RX: FWHT → Bin_To_LLR → Polar Decode
//
// [빌드] POLAR_TEST 프로젝트에 추가:
//   - HTS_Polar_Codec.h / .cpp
//   - 이 파일 (POLAR_WALSH_TEST.cpp) 을 main()으로 사용
//     기존 POLAR_TEST.cpp의 main()은 주석 처리 또는 제거
// =============================================================================
#include "HTS_Polar_Codec.h"
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
using Polar = ProtectedEngine::HTS_Polar_Codec;
// ── 시스템 파라미터 ──────────────────────────────────────
static constexpr int NC = 64;           // Walsh 칩 수
static constexpr int BPS = 4;           // bits per symbol
static constexpr int TOTAL_CODED = 688; // Conv+REP4 호환 (172심볼 × BPS4)
static constexpr int NSYM = TOTAL_CODED / BPS; // 172 심볼
static constexpr int AMP = 300;                // Walsh 진폭
// 688/512 = 1.344× 반복 확장 → +1.3dB 소프트 결합 이득
static_assert(NSYM == 172, "NSYM must match Conv+REP4 system");
static_assert(TOTAL_CODED > Polar::N, "Must have repetition expansion");
// ── Walsh 인코딩 (FWHT 기반) ─────────────────────────────
static uint32_t popc32(uint32_t x) noexcept {
    x -= (x >> 1u) & 0x55555555u;
    x = (x & 0x33333333u) + ((x >> 2u) & 0x33333333u);
    return (((x + (x >> 4u)) & 0x0F0F0F0Fu) * 0x01010101u) >> 24u;
}
static void walsh_enc(uint8_t sym, int nc, int16_t amp, int16_t *oI,
                      int16_t *oQ) {
    const uint32_t su = static_cast<uint32_t>(sym);
    for (int j = 0; j < nc; ++j) {
        const int16_t ch = (popc32(su & static_cast<uint32_t>(j)) & 1u)
                               ? static_cast<int16_t>(-amp)
                               : static_cast<int16_t>(amp);
        oI[j] = ch;
        oQ[j] = ch; // I=Q 동일
    }
}
// ── FWHT (Fast Walsh-Hadamard Transform) ─────────────────
static void fwht(int32_t *d, int n) {
    for (int half = 1; half < n; half <<= 1) {
        for (int i = 0; i < n; i += 2 * half) {
            for (int j = 0; j < half; ++j) {
                const int32_t u = d[i + j];
                const int32_t v = d[i + j + half];
                d[i + j] = u + v;
                d[i + j + half] = u - v;
            }
        }
    }
}
// ── Bin_To_LLR: Sum 방식 (HARQ 누적 호환) + TPE branchless ──
//  Sum: 잡음 8빈 상쇄 (기대값=0) → HARQ 32R 누적에 최적
//  Max: 고 J/S에서 잡음 편향 누적 → HARQ 악화 → 폐기
static void bin_to_llr(const int32_t *fI, const int32_t *fQ, int nc, int bps,
                       int32_t *llr) {
    const int nsym_max = 1 << bps;
    for (int b = 0; b < bps; ++b) {
        int32_t pos_sum = 0, neg_sum = 0;
        const int sh = bps - 1 - b;
        for (int m = 0; m < nsym_max && m < nc; ++m) {
            const int32_t corr = fI[m] + fQ[m];
            // TPE branchless: 비트마스크로 pos/neg 분배
            const uint32_t is_one =
                0u -
                ((static_cast<uint32_t>(m) >> static_cast<uint32_t>(sh)) & 1u);
            pos_sum += corr & static_cast<int32_t>(~is_one);
            neg_sum += corr & static_cast<int32_t>(is_one);
        }
        llr[b] = (pos_sum - neg_sum) >> 10;
    }
}
// ── 바라지 잡음 추가 ────────────────────────────────────
static void add_barrage(int16_t *I, int16_t *Q, int n, double js_db,
                        int16_t amp, std::mt19937 &rng) {
    if (js_db < -0.5)
        return;
    const double sigma =
        static_cast<double>(amp) * std::sqrt(std::pow(10.0, js_db / 10.0));
    std::normal_distribution<double> nd(0.0, sigma);
    for (int i = 0; i < n; ++i) {
        double vi = static_cast<double>(I[i]) + nd(rng);
        double vq = static_cast<double>(Q[i]) + nd(rng);
        if (vi > 32767)
            vi = 32767;
        if (vi < -32768)
            vi = -32768;
        if (vq > 32767)
            vq = 32767;
        if (vq < -32768)
            vq = -32768;
        I[i] = static_cast<int16_t>(std::lround(vi));
        Q[i] = static_cast<int16_t>(std::lround(vq));
    }
}
// ── 랜덤 데이터 생성 ────────────────────────────────────
static void gen_info(uint32_t seed, uint8_t *info, int len) {
    uint32_t s = seed;
    for (int i = 0; i < len; ++i) {
        s ^= s << 13u;
        s ^= s >> 17u;
        s ^= s << 5u;
        info[i] = static_cast<uint8_t>(s & 0xFFu);
    }
}
// ═════════════════════════════════════════════════════════
//  Polar + Walsh 단일 패킷 테스트
// ═════════════════════════════════════════════════════════
struct TestResult {
    bool success;
    int rounds; // HARQ 라운드 (1 = 단발 성공)
};
static TestResult test_one_packet(const uint8_t *info, double js_db,
                                  int max_rounds, std::mt19937 &rng,
                                  bool use_scl) {
    TestResult res = {false, 0};
    // Step 1: Polar 인코딩 (512비트)
    uint8_t coded[Polar::N / 8] = {};
    if (Polar::Encode(info, 8, coded) != Polar::N)
        return res;
    // Step 2: 반복 확장 512 → 688 비트 (cyclic repetition)
    //  coded_688[i] = coded_512[i % 512]
    //  172심볼 × BPS4 = 688비트 = Conv+REP4 동일 슬롯
    uint8_t syms[NSYM] = {};
    for (int s = 0; s < NSYM; ++s) {
        uint8_t val = 0;
        for (int b = 0; b < BPS; ++b) {
            const int bi_688 = s * BPS + b;       // 0..687
            const int bi_512 = bi_688 % Polar::N; // 0..511 cyclic wrap
            const int byte_idx = bi_512 >> 3;
            const int bit_idx = 7 - (bi_512 & 7);
            val |= ((coded[byte_idx] >> bit_idx) & 1u) << (BPS - 1 - b);
        }
        syms[s] = val;
    }
    // Step 3: HARQ 루프
    // int32 누적 → 포화 없음 → 디코딩 전 int16 스케일 다운
    static int32_t polar_llr_acc32[Polar::N];
    std::memset(polar_llr_acc32, 0, sizeof(polar_llr_acc32));
    for (int rv = 0; rv < max_rounds; ++rv) {
        // Walsh 변조 (172심볼 × 64칩)
        static int16_t chipI[NSYM * NC], chipQ[NSYM * NC];
        for (int s = 0; s < NSYM; ++s) {
            walsh_enc(syms[s], NC, AMP, &chipI[s * NC], &chipQ[s * NC]);
        }
        // 바라지 잡음 추가
        add_barrage(chipI, chipQ, NSYM * NC, js_db, static_cast<int16_t>(AMP),
                    rng);
        // FWHT + LLR 추출 + 688→512 폴딩
        static int32_t fI[NC], fQ[NC];
        for (int s = 0; s < NSYM; ++s) {
            for (int c = 0; c < NC; ++c) {
                fI[c] = static_cast<int32_t>(chipI[s * NC + c]);
                fQ[c] = static_cast<int32_t>(chipQ[s * NC + c]);
            }
            fwht(fI, NC);
            fwht(fQ, NC);
            int32_t llr_sym[BPS] = {};
            bin_to_llr(fI, fQ, NC, BPS, llr_sym);
            // int32 누적 (포화 없음)
            for (int b = 0; b < BPS; ++b) {
                const int bi_688 = s * BPS + b;
                const int bi_512 = bi_688 % Polar::N;
                polar_llr_acc32[bi_512] += llr_sym[b];
            }
        }
        // int32 → int16 스케일 다운 (디코딩용)
        // 최대값 탐색 후 적응 시프트
        int32_t max_abs = 1;
        for (int i = 0; i < Polar::N; ++i) {
            int32_t a = polar_llr_acc32[i];
            if (a < 0)
                a = -a;
            if (a > max_abs)
                max_abs = a;
        }
        // max_abs를 ~8000 수준으로 스케일 (int16 50% 활용)
        int shift = 0;
        while ((max_abs >> shift) > 8000 && shift < 20)
            shift++;
        static int16_t polar_llr_dec[Polar::N];
        for (int i = 0; i < Polar::N; ++i) {
            int32_t v = polar_llr_acc32[i] >> shift;
            if (v > 32767)
                v = 32767;
            if (v < -32768)
                v = -32768;
            polar_llr_dec[i] = static_cast<int16_t>(v);
        }
        // Polar 디코딩
        uint8_t out[8] = {};
        int olen = 0;
        bool ok;
        if (use_scl) {
            ok = Polar::Decode_SCL(polar_llr_dec, out, &olen);
        } else {
            ok = Polar::Decode_SC(polar_llr_dec, out, &olen);
        }
        res.rounds = rv + 1;
        if (ok && olen == 8 && std::memcmp(info, out, 8) == 0) {
            res.success = true;
            return res;
        }
    }
    return res;
}
// ═════════════════════════════════════════════════════════
//  메인 테스트
// ═════════════════════════════════════════════════════════
int main() {
    std::printf("╔══════════════════════════════════════════════╗\n");
    std::printf("║  Polar + Walsh-64 통합 테스트                ║\n");
    std::printf("║  N=%d, K=%d, BPS=%d, NC=%d, SCL-L=%d       ║\n", Polar::N,
                Polar::K, BPS, NC, Polar::SCL_L);
    std::printf("╚══════════════════════════════════════════════╝\n");
    // ── T1: 클린 채널 왕복 + 진단 ──
    {
        std::printf("\n=== T1: 클린 Walsh 왕복 ===\n");
        // T1-A: Walsh 우회 (직접 LLR) — Polar 디코더 검증
        {
            int pass = 0;
            for (int t = 0; t < 20; ++t) {
                uint8_t info[8];
                gen_info(static_cast<uint32_t>(t + 1), info, 8);
                uint8_t coded[Polar::N / 8] = {};
                Polar::Encode(info, 8, coded);
                static int16_t llr[Polar::N];
                for (int i = 0; i < Polar::N; ++i) {
                    const uint8_t bit = (coded[i >> 3] >> (7 - (i & 7))) & 1u;
                    llr[i] = (bit == 0) ? static_cast<int16_t>(150)
                                        : static_cast<int16_t>(-150);
                }
                uint8_t out[8] = {};
                int olen = 0;
                bool ok = Polar::Decode_SCL(llr, out, &olen);
                if (ok && olen == 8 && std::memcmp(info, out, 8) == 0)
                    pass++;
            }
            std::printf("  T1-A (직접 LLR ±150): %d/20 %s\n", pass,
                        (pass == 20) ? "OK" : "FAIL ← 디코더 문제");
        }
        // T1-B: Walsh 파이프라인 LLR 부호 검증 (688슬롯 반복 확장)
        {
            int total_mismatch = 0;
            for (int t = 0; t < 5; ++t) {
                uint8_t info[8];
                gen_info(static_cast<uint32_t>(t + 1), info, 8);
                uint8_t coded[Polar::N / 8] = {};
                Polar::Encode(info, 8, coded);
                // 반복 확장 심볼 추출 (688비트)
                uint8_t syms[NSYM] = {};
                for (int s = 0; s < NSYM; ++s) {
                    uint8_t val = 0;
                    for (int b = 0; b < BPS; ++b) {
                        const int bi_688 = s * BPS + b;
                        const int bi_512 = bi_688 % Polar::N;
                        val |= ((coded[bi_512 >> 3] >> (7 - (bi_512 & 7))) & 1u)
                               << (BPS - 1 - b);
                    }
                    syms[s] = val;
                }
                // Walsh → FWHT → LLR
                static int16_t cI[NSYM * NC], cQ[NSYM * NC];
                for (int s = 0; s < NSYM; ++s)
                    walsh_enc(syms[s], NC, AMP, &cI[s * NC], &cQ[s * NC]);
                int mismatch = 0;
                for (int s = 0; s < NSYM; ++s) {
                    static int32_t fI[NC], fQ[NC];
                    for (int c = 0; c < NC; ++c) {
                        fI[c] = static_cast<int32_t>(cI[s * NC + c]);
                        fQ[c] = static_cast<int32_t>(cQ[s * NC + c]);
                    }
                    fwht(fI, NC);
                    fwht(fQ, NC);
                    int32_t llr_sym[BPS] = {};
                    bin_to_llr(fI, fQ, NC, BPS, llr_sym);
                    for (int b = 0; b < BPS; ++b) {
                        const int bi_688 = s * BPS + b;
                        const int bi_512 = bi_688 % Polar::N;
                        const uint8_t coded_bit =
                            (coded[bi_512 >> 3] >> (7 - (bi_512 & 7))) & 1u;
                        const bool sign_ok =
                            (coded_bit == 0 && llr_sym[b] > 0) ||
                            (coded_bit == 1 && llr_sym[b] < 0);
                        if (!sign_ok)
                            mismatch++;
                    }
                }
                if (mismatch > 0)
                    std::printf("  T1-B seed %d: %d/%d LLR 부호 불일치\n",
                                t + 1, mismatch, TOTAL_CODED);
                total_mismatch += mismatch;
            }
            std::printf("  T1-B (LLR 부호검증): %s\n",
                        (total_mismatch == 0) ? "OK" : "FAIL");
        }
        // T1-C: Walsh 파이프라인 전체 왕복
        {
            int pass = 0;
            std::mt19937 rng(42u);
            for (int t = 0; t < 20; ++t) {
                uint8_t info[8];
                gen_info(static_cast<uint32_t>(t + 1), info, 8);
                auto r = test_one_packet(info, -10.0, 1, rng, true);
                if (r.success)
                    pass++;
                else
                    std::printf("  T1-C [%d] FAIL\n", t + 1);
            }
            std::printf("  T1-C (Walsh 왕복): %d/20 %s\n", pass,
                        (pass == 20) ? "PASS" : "FAIL");
        }
    }
    // ── T2: J/S 스윕 (SC, 단발) ──
    {
        std::printf("\n=== T2: SC 단발 — J/S 스윕 ===\n");
        std::printf("  J/S(dB)  |   CRC%%   |  Frames\n");
        std::printf("  ---------+---------+--------\n");
        const double js[] = {-1, 0, 5, 10, 15, 20, 25};
        for (double j : js) {
            int ok = 0;
            const int frames = 100;
            std::mt19937 rng(12345u);
            for (int t = 0; t < frames; ++t) {
                uint8_t info[8];
                gen_info(static_cast<uint32_t>(t), info, 8);
                auto r = test_one_packet(info, j, 1, rng, false);
                if (r.success)
                    ok++;
            }
            std::printf("  %5.0f dB  |  %5.1f%%  |  %d\n", j,
                        100.0 * ok / frames, frames);
            if (ok == 0 && j > 0)
                break;
        }
    }
    // ── T3: SCL-4 + HARQ 32라운드 — J/S 스윕 ──
    {
        std::printf("\n=== T3: SCL-4 + HARQ 32R — J/S 스윕 ===\n");
        std::printf("  J/S(dB)  |   CRC%%   | avg R  |  Frames\n");
        std::printf("  ---------+---------+--------+--------\n");
        const double js[] = {-1, 0, 5, 10, 15, 20, 25, 30};
        for (double j : js) {
            int ok = 0;
            double total_r = 0;
            const int frames = 100;
            std::mt19937 rng(67890u);
            for (int t = 0; t < frames; ++t) {
                uint8_t info[8];
                gen_info(static_cast<uint32_t>(t), info, 8);
                auto r = test_one_packet(info, j, 32, rng, true);
                if (r.success) {
                    ok++;
                    total_r += r.rounds;
                }
            }
            double avg_r = (ok > 0) ? total_r / ok : 0;
            std::printf("  %5.0f dB  |  %5.1f%%  | %4.1fR  |  %d\n", j,
                        100.0 * ok / frames, avg_r, frames);
            if (ok == 0 && j > 5)
                break;
        }
    }
    // ── T4: CPU 타이밍 ──
    {
        std::printf("\n=== T4: 파이프라인 타이밍 ===\n");
        uint8_t info[8] = {0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45, 0x67, 0x89};
        std::mt19937 rng(99u);
        const int iters = 1000;
        auto t0 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < iters; ++i) {
            auto r = test_one_packet(info, 10.0, 1, rng, false);
            (void)r;
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        double sc_us =
            std::chrono::duration<double, std::micro>(t1 - t0).count() /
            static_cast<double>(iters);
        t0 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < iters; ++i) {
            auto r = test_one_packet(info, 10.0, 1, rng, true);
            (void)r;
        }
        t1 = std::chrono::high_resolution_clock::now();
        double scl_us =
            std::chrono::duration<double, std::micro>(t1 - t0).count() /
            static_cast<double>(iters);
        std::printf("  Walsh+Polar SC:   %.0f µs/pkt\n", sc_us);
        std::printf("  Walsh+Polar SCL4: %.0f µs/pkt\n", scl_us);
    }
    std::printf("\n═══════════════════════════════════════════\n");
    std::printf("  완료\n");
    std::printf("═══════════════════════════════════════════\n");
    return 0;
}
