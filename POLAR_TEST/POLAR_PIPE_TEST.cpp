// =============================================================================
// POLAR_PIPE_TEST.cpp — 실 파이프라인 궁합 테스트
//
// [3가지 파이프라인 비교]
//   A: LLR 누적 (기존 테스트) — Polar 이론 최적
//   B: 칩 누적 + soft_clip + FWHT → Polar — 실 양산 구조
//   C: 칩 누적 + soft_clip + FWHT → Tensor 8×Polar(64) — 다차원 대안
//
// [빌드] HTS_Polar_Codec.h/cpp + 이 파일
// =============================================================================
#include "HTS_Polar_Codec.h"
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
using Polar = ProtectedEngine::HTS_Polar_Codec;
// ── 시스템 파라미터 ──────────────────────────────────────
static constexpr int NC = 64;
static constexpr int BPS = 4;
static constexpr int TOTAL_CODED = 688;
static constexpr int NSYM = TOTAL_CODED / BPS; // 172
static constexpr int AMP = 300;
// ── Walsh / FWHT / LLR (기존과 동일) ─────────────────────
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
        oQ[j] = ch;
    }
}
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
// Max-Log-MAP LLR
static void bin_to_llr_max(const int32_t *fI, const int32_t *fQ, int nc,
                           int bps, int32_t *llr) {
    const int nsym_max = 1 << bps;
    for (int b = 0; b < bps; ++b) {
        int32_t pos_max = -2147483647;
        int32_t neg_max = -2147483647;
        const int sh = bps - 1 - b;
        for (int m = 0; m < nsym_max && m < nc; ++m) {
            const int32_t corr = fI[m] + fQ[m];
            if (((m >> sh) & 1) == 0) {
                if (corr > pos_max)
                    pos_max = corr;
            } else {
                if (corr > neg_max)
                    neg_max = corr;
            }
        }
        llr[b] = (pos_max - neg_max) >> 10;
    }
}
// ── ECCM: soft_clip (비선형 클리핑) ─────────────────────
//  실 양산의 soft_clip_ 간이 버전
//  임계값 = 라운드 수 × AMP × 1.5 (신호 피크의 1.5배)
static void soft_clip(int32_t *buf, int n, int32_t threshold) {
    for (int i = 0; i < n; ++i) {
        if (buf[i] > threshold)
            buf[i] = threshold;
        if (buf[i] < -threshold)
            buf[i] = -threshold;
    }
}
// ── 바라지 잡음 ─────────────────────────────────────────
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
static void gen_info(uint32_t seed, uint8_t *info, int len) {
    uint32_t s = seed;
    for (int i = 0; i < len; ++i) {
        s ^= s << 13u;
        s ^= s >> 17u;
        s ^= s << 5u;
        info[i] = static_cast<uint8_t>(s & 0xFFu);
    }
}
// ── Polar 인코딩 + 688 반복 확장 → 심볼 ─────────────────
static bool polar_encode_to_syms(const uint8_t *info, uint8_t *syms_out) {
    uint8_t coded[Polar::N / 8] = {};
    if (Polar::Encode(info, 8, coded) != Polar::N)
        return false;
    for (int s = 0; s < NSYM; ++s) {
        uint8_t val = 0;
        for (int b = 0; b < BPS; ++b) {
            const int bi_688 = s * BPS + b;
            const int bi_512 = bi_688 % Polar::N;
            const int byte_idx = bi_512 >> 3;
            const int bit_idx = 7 - (bi_512 & 7);
            val |= ((coded[byte_idx] >> bit_idx) & 1u) << (BPS - 1 - b);
        }
        syms_out[s] = val;
    }
    return true;
}
// ── 512 LLR에서 Polar 디코딩 공통 루틴 ──────────────────
static bool polar_decode_from_llr(const int32_t *llr32, uint8_t *out,
                                  bool use_scl) {
    // max_abs → 적응 시프트
    int32_t max_abs = 1;
    for (int i = 0; i < Polar::N; ++i) {
        int32_t a = llr32[i] < 0 ? -llr32[i] : llr32[i];
        if (a > max_abs)
            max_abs = a;
    }
    int shift = 0;
    while ((max_abs >> shift) > 8000 && shift < 20)
        shift++;
    static int16_t llr16[Polar::N];
    for (int i = 0; i < Polar::N; ++i) {
        int32_t v = llr32[i] >> shift;
        if (v > 32767)
            v = 32767;
        if (v < -32768)
            v = -32768;
        llr16[i] = static_cast<int16_t>(v);
    }
    int olen = 0;
    if (use_scl)
        return Polar::Decode_SCL(llr16, out, &olen) && olen == 8;
    else
        return Polar::Decode_SC(llr16, out, &olen) && olen == 8;
}
// ═════════════════════════════════════════════════════════
//  파이프라인 A: LLR 누적 (기존 — 이론 최적)
// ═════════════════════════════════════════════════════════
struct Result {
    bool success;
    int rounds;
};
static Result pipe_A_llr_harq(const uint8_t *info, const uint8_t *syms,
                              double js_db, int max_rounds, std::mt19937 &rng) {
    Result res = {false, 0};
    static int32_t llr_acc[Polar::N];
    std::memset(llr_acc, 0, sizeof(llr_acc));
    for (int rv = 0; rv < max_rounds; ++rv) {
        static int16_t cI[NSYM * NC], cQ[NSYM * NC];
        for (int s = 0; s < NSYM; ++s)
            walsh_enc(syms[s], NC, AMP, &cI[s * NC], &cQ[s * NC]);
        add_barrage(cI, cQ, NSYM * NC, js_db, static_cast<int16_t>(AMP), rng);
        static int32_t fI[NC], fQ[NC];
        for (int s = 0; s < NSYM; ++s) {
            for (int c = 0; c < NC; ++c) {
                fI[c] = static_cast<int32_t>(cI[s * NC + c]);
                fQ[c] = static_cast<int32_t>(cQ[s * NC + c]);
            }
            fwht(fI, NC);
            fwht(fQ, NC);
            int32_t llr_sym[BPS] = {};
            bin_to_llr_max(fI, fQ, NC, BPS, llr_sym);
            for (int b = 0; b < BPS; ++b) {
                const int bi = (s * BPS + b) % Polar::N;
                llr_acc[bi] += llr_sym[b];
            }
        }
        uint8_t out[8] = {};
        if (polar_decode_from_llr(llr_acc, out, true) &&
            std::memcmp(info, out, 8) == 0) {
            res.success = true;
            res.rounds = rv + 1;
            return res;
        }
        res.rounds = rv + 1;
    }
    return res;
}
// ═════════════════════════════════════════════════════════
//  파이프라인 B: 칩 누적 + soft_clip + FWHT → Polar
//  (실 양산 구조)
// ═════════════════════════════════════════════════════════
static Result pipe_B_chip_harq(const uint8_t *info, const uint8_t *syms,
                               double js_db, int max_rounds,
                               std::mt19937 &rng) {
    Result res = {false, 0};
    // 칩 도메인 누적 버퍼: [NSYM][NC] × I/Q (int32)
    static int32_t harq_I[NSYM][NC], harq_Q[NSYM][NC];
    std::memset(harq_I, 0, sizeof(harq_I));
    std::memset(harq_Q, 0, sizeof(harq_Q));
    for (int rv = 0; rv < max_rounds; ++rv) {
        // ① Walsh 변조 + 잡음
        static int16_t cI[NSYM * NC], cQ[NSYM * NC];
        for (int s = 0; s < NSYM; ++s)
            walsh_enc(syms[s], NC, AMP, &cI[s * NC], &cQ[s * NC]);
        add_barrage(cI, cQ, NSYM * NC, js_db, static_cast<int16_t>(AMP), rng);
        // ② 칩 도메인 누적 (실 양산과 동일)
        for (int s = 0; s < NSYM; ++s) {
            for (int c = 0; c < NC; ++c) {
                harq_I[s][c] += static_cast<int32_t>(cI[s * NC + c]);
                harq_Q[s][c] += static_cast<int32_t>(cQ[s * NC + c]);
            }
        }
        // ③ ECCM: soft_clip (비선형 — 핵심 차이점)
        //    임계값 = (rv+1) × AMP × NC × 1.5
        //    → 신호 피크의 1.5배, 잡음 피크 제거
        const int32_t clip_th =
            static_cast<int32_t>(static_cast<double>(rv + 1) * AMP * 1.5);
        static int32_t clipped_I[NC], clipped_Q[NC];
        // ④ 심볼별: clip → FWHT → LLR → 폴딩
        static int32_t llr_512[Polar::N];
        std::memset(llr_512, 0, sizeof(llr_512));
        for (int s = 0; s < NSYM; ++s) {
            // 누적 칩 복사 + soft_clip
            for (int c = 0; c < NC; ++c) {
                clipped_I[c] = harq_I[s][c];
                clipped_Q[c] = harq_Q[s][c];
            }
            soft_clip(clipped_I, NC, clip_th);
            soft_clip(clipped_Q, NC, clip_th);
            // FWHT
            fwht(clipped_I, NC);
            fwht(clipped_Q, NC);
            // Max-Log-MAP LLR
            int32_t llr_sym[BPS] = {};
            bin_to_llr_max(clipped_I, clipped_Q, NC, BPS, llr_sym);
            // 688→512 폴딩 (매 라운드 새로 계산, LLR 누적 아님)
            for (int b = 0; b < BPS; ++b) {
                const int bi = (s * BPS + b) % Polar::N;
                llr_512[bi] += llr_sym[b];
            }
        }
        // ⑤ Polar 디코딩
        uint8_t out[8] = {};
        if (polar_decode_from_llr(llr_512, out, true) &&
            std::memcmp(info, out, 8) == 0) {
            res.success = true;
            res.rounds = rv + 1;
            return res;
        }
        res.rounds = rv + 1;
    }
    return res;
}
// ═════════════════════════════════════════════════════════
//  파이프라인 C: 칩 누적 + soft_clip → Tensor 8×Polar(64)
//  (다차원 대안 — ECCM 후 독립 행 디코딩)
// ═════════════════════════════════════════════════════════
static Result pipe_C_tensor(const uint8_t *info, const uint8_t *syms,
                            double js_db, int max_rounds, std::mt19937 &rng) {
    Result res = {false, 0};
    // 동일한 칩 누적
    static int32_t harq_I[NSYM][NC], harq_Q[NSYM][NC];
    std::memset(harq_I, 0, sizeof(harq_I));
    std::memset(harq_Q, 0, sizeof(harq_Q));
    for (int rv = 0; rv < max_rounds; ++rv) {
        static int16_t cI[NSYM * NC], cQ[NSYM * NC];
        for (int s = 0; s < NSYM; ++s)
            walsh_enc(syms[s], NC, AMP, &cI[s * NC], &cQ[s * NC]);
        add_barrage(cI, cQ, NSYM * NC, js_db, static_cast<int16_t>(AMP), rng);
        for (int s = 0; s < NSYM; ++s) {
            for (int c = 0; c < NC; ++c) {
                harq_I[s][c] += static_cast<int32_t>(cI[s * NC + c]);
                harq_Q[s][c] += static_cast<int32_t>(cQ[s * NC + c]);
            }
        }
        const int32_t clip_th =
            static_cast<int32_t>(static_cast<double>(rv + 1) * AMP * 1.5);
        static int32_t cl_I[NC], cl_Q[NC];
        // 688 LLR → 512 폴딩 → Polar 디코딩 (B와 동일)
        // C는 동일 파이프라인이지만 Polar 대신 단순 SC 사용
        // (Tensor 8×64 구현은 별도 코덱 필요 → 여기서는 SC로 대체)
        static int32_t llr_512[Polar::N];
        std::memset(llr_512, 0, sizeof(llr_512));
        for (int s = 0; s < NSYM; ++s) {
            for (int c = 0; c < NC; ++c) {
                cl_I[c] = harq_I[s][c];
                cl_Q[c] = harq_Q[s][c];
            }
            soft_clip(cl_I, NC, clip_th);
            soft_clip(cl_Q, NC, clip_th);
            fwht(cl_I, NC);
            fwht(cl_Q, NC);
            int32_t llr_sym[BPS] = {};
            bin_to_llr_max(cl_I, cl_Q, NC, BPS, llr_sym);
            for (int b = 0; b < BPS; ++b) {
                const int bi = (s * BPS + b) % Polar::N;
                llr_512[bi] += llr_sym[b];
            }
        }
        // SC 디코딩 (Tensor 대리 — 속도 우위 확인용)
        uint8_t out[8] = {};
        if (polar_decode_from_llr(llr_512, out, false) &&
            std::memcmp(info, out, 8) == 0) {
            res.success = true;
            res.rounds = rv + 1;
            return res;
        }
        res.rounds = rv + 1;
    }
    return res;
}
// ═════════════════════════════════════════════════════════
//  메인 비교 테스트
// ═════════════════════════════════════════════════════════
int main() {
    std::printf("╔══════════════════════════════════════════════════╗\n");
    std::printf("║  파이프라인 궁합 테스트: A(LLR) vs B(칩+ECCM)  ║\n");
    std::printf("║  N=%d, K=%d, BPS=%d, NC=%d, 688rep            ║\n", Polar::N,
                Polar::K, BPS, NC);
    std::printf("╚══════════════════════════════════════════════════╝\n");
    // ── 클린 채널 검증 ──
    {
        std::printf("\n=== 클린 채널 ===\n");
        int pass_a = 0, pass_b = 0;
        for (int t = 0; t < 20; ++t) {
            uint8_t info[8], syms[NSYM];
            gen_info(static_cast<uint32_t>(t + 1), info, 8);
            polar_encode_to_syms(info, syms);
            std::mt19937 rng_a(42u + t), rng_b(42u + t);
            if (pipe_A_llr_harq(info, syms, -10, 1, rng_a).success)
                pass_a++;
            if (pipe_B_chip_harq(info, syms, -10, 1, rng_b).success)
                pass_b++;
        }
        std::printf("  A(LLR):      %d/20 %s\n", pass_a,
                    pass_a == 20 ? "OK" : "FAIL");
        std::printf("  B(칩+ECCM):  %d/20 %s\n", pass_b,
                    pass_b == 20 ? "OK" : "FAIL");
    }
    // ── J/S 스윕 비교 (SCL-4 + HARQ 32R) ──
    {
        std::printf("\n=== SCL-4 + HARQ 32R — A vs B 비교 ===\n");
        std::printf("  J/S    |  A(LLR)     |  B(칩+ECCM) |  차이\n");
        std::printf("  -------+-------------+-------------+------\n");
        const double js_list[] = {5, 10, 15, 20, 25, 30};
        for (double js : js_list) {
            int ok_a = 0, ok_b = 0;
            double rnd_a = 0, rnd_b = 0;
            const int frames = 100;
            for (int t = 0; t < frames; ++t) {
                uint8_t info[8], syms[NSYM];
                gen_info(static_cast<uint32_t>(t), info, 8);
                polar_encode_to_syms(info, syms);
                std::mt19937 rng_a(12345u + t);
                std::mt19937 rng_b(12345u + t); // 동일 잡음
                auto ra = pipe_A_llr_harq(info, syms, js, 32, rng_a);
                auto rb = pipe_B_chip_harq(info, syms, js, 32, rng_b);
                if (ra.success) {
                    ok_a++;
                    rnd_a += ra.rounds;
                }
                if (rb.success) {
                    ok_b++;
                    rnd_b += rb.rounds;
                }
            }
            double avg_a = ok_a > 0 ? rnd_a / ok_a : 0;
            double avg_b = ok_b > 0 ? rnd_b / ok_b : 0;
            const char *diff = (ok_b >= ok_a) ? "B>=A" : "B<A ⚠️";
            std::printf("  %4.0fdB  | %3d%% %4.1fR  | %3d%% %4.1fR  | %s\n", js,
                        ok_a, avg_a, ok_b, avg_b, diff);
        }
    }
    std::printf("\n═══════════════════════════════════════════\n");
    std::printf("  완료\n");
    std::printf("═══════════════════════════════════════════\n");
    return 0;
}
