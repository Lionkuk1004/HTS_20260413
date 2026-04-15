// =============================================================================
// HTS_FEC_HARQ.cpp — V400 3모드 (1칩/16칩/64칩)
// Target: STM32F407VGT6 (Cortex-M4F) / PC
//
#include "HTS_FEC_HARQ.hpp"
#include "HTS_RS_GF16.h"
#include "HTS_Secure_Memory.h"
#include <array>
#include <atomic>
#include <climits>
#include <cstdint>
#include <cstring>
#if defined(HTS_FEC_POLAR_ENABLE)
#include "HTS_DWT_Profiler.h"
#endif
#if defined(_MSC_VER)
#include <intrin.h>
#endif
#if defined(HTS_FEC_PROFILE)
#include <chrono>
#if defined(_WIN32)
#include <intrin.h>
#endif
#endif
namespace ProtectedEngine {
#if defined(HTS_FEC_PROFILE)
namespace {
struct FecProfG {
    uint64_t sym{};
    uint64_t deint{};
    uint64_t rep{};
    uint64_t vit{};
    uint64_t tail{};
    uint64_t calls{};
};
FecProfG g_fec_prof{};
static inline uint64_t fec_prof_now() noexcept {
#if defined(_WIN32)
    return static_cast<uint64_t>(__rdtsc());
#elif ((defined(__arm__) || defined(__TARGET_ARCH_ARM) ||                      \
        defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)) &&                \
       !defined(HTS_ALLOW_HOST_BUILD))
    return static_cast<uint64_t>(
        *reinterpret_cast<volatile uint32_t *>(0xE0001004u));
#else
    using clock = std::chrono::steady_clock;
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            clock::now().time_since_epoch())
            .count());
#endif
}
} // namespace
void FEC_HARQ::Profile_Reset() noexcept { g_fec_prof = {}; }
void FEC_HARQ::Profile_Get(DecodeProfileStats &out) noexcept {
    out.ticks_sym_prep_and_loop = g_fec_prof.sym;
    out.ticks_bit_deinterleave = g_fec_prof.deint;
    out.ticks_rep_combine = g_fec_prof.rep;
    out.ticks_viterbi = g_fec_prof.vit;
    out.ticks_tail = g_fec_prof.tail;
    out.calls = g_fec_prof.calls;
}
#endif // HTS_FEC_PROFILE
// 컴파일 타임 고정 스택 버퍼 — VLA/alloca 경로 배제 (임베디드 규약)
static constexpr std::size_t k_conv_out_sz =
    static_cast<std::size_t>(FEC_HARQ::CONV_OUT);
static constexpr std::size_t k_fwht_buf_sz =
    static_cast<std::size_t>(FEC_HARQ::C64);
static constexpr std::size_t k_llr_buf_sz =
    static_cast<std::size_t>(FEC_HARQ::BPS64_MAX);
static_assert(FEC_HARQ::BPS64_MAX >= 1, "BPS64_MAX must be positive");
static_assert(FEC_HARQ::BPS64_MAX <= FEC_HARQ::C64,
              "Bin_To_LLR bps exceeds scratch");
// Decode_Core·Decode64_IR: fI/fQ/llr BSS 단일화 (스택 512B 초과 방지)
// Decode64_IR 동일 버퍼 사용 — 비재진입·호출부 직렬화 필수
alignas(64) static std::array<int32_t, k_fwht_buf_sz> g_fec_dec_fI{};
alignas(64) static std::array<int32_t, k_fwht_buf_sz> g_fec_dec_fQ{};
alignas(64) static std::array<int32_t, k_llr_buf_sz> g_fec_dec_llr{};
static std::atomic<uint8_t> g_ir_erasure_en{1u};
static std::atomic<uint8_t> g_ir_rs_post_en{1u};
static inline int32_t fec_ir_fast_abs_i32(int32_t v) noexcept {
    const int32_t m = v >> 31;
    return (v ^ m) - m;
}
/// RS(15,8) 니블 매핑: rx[0..7]의 15니블 정정, rx[7] 하위 니블 보존
static bool try_ir_rs_recover_rx8(uint8_t *rx_head8) noexcept {
    if (!rx_head8) {
        return false;
    }
    uint8_t sym15[15];
    const uint8_t lo7 = static_cast<uint8_t>(rx_head8[7] & 0x0Fu);
    for (int i = 0; i < 7; ++i) {
        sym15[static_cast<std::size_t>(2 * i)] =
            static_cast<uint8_t>(rx_head8[static_cast<std::size_t>(i)] >> 4);
        sym15[static_cast<std::size_t>(2 * i + 1)] =
            static_cast<uint8_t>(rx_head8[static_cast<std::size_t>(i)] & 0x0Fu);
    }
    sym15[14] = static_cast<uint8_t>(rx_head8[7] >> 4);
    if (!HTS_RS_GF16_Decode15_8(sym15)) {
        return false;
    }
    for (int i = 0; i < 7; ++i) {
        rx_head8[static_cast<std::size_t>(i)] =
            static_cast<uint8_t>((sym15[static_cast<std::size_t>(2 * i)] << 4) |
                                 sym15[static_cast<std::size_t>(2 * i + 1)]);
    }
    rx_head8[7] = static_cast<uint8_t>((sym15[14] << 4) | lo7);
    return true;
}
// LTO/DCE가 스택 평문 소거를 제거하지 못하도록 secureWipe 직후 컴파일러·동기화
// 펜스
static inline void fec_fence_after_stack_wipe() noexcept {
#if defined(__GNUC__) || defined(__clang__)
    __asm__ __volatile__("" ::: "memory");
#elif defined(_MSC_VER)
    _ReadWriteBarrier();
#endif
    std::atomic_thread_fence(std::memory_order_seq_cst);
}
static inline void fec_secure_wipe_stack(void *p, std::size_t n) noexcept {
    SecureMemory::secureWipe(p, n);
    fec_fence_after_stack_wipe();
}
// ── [수정 1/6] 브랜치리스 LLR 포화 가산·클램프 (constant-time, ±500000) ──
//  기존 sat_add_i32: if 분기 2개 → 타이밍 부채널 + Cortex-M4 파이프라인 스톨
//  tpe_sat_add_llr: ±500000 범위 — Viterbi 경로 메트릭 오버플로 방지
//  tpe_clamp_llr: LLR 단일 값 클램프 (Decode_Core·IR 경로에서 사용)
static inline int32_t tpe_clamp_llr(int32_t v) noexcept {
    const int32_t lim = 500000;
    const int32_t m_pos = -static_cast<int32_t>(v > lim);
    const int32_t m_neg = -static_cast<int32_t>(v < -lim);
    return (lim & m_pos) | (-lim & m_neg) | (v & ~(m_pos | m_neg));
}
static inline int32_t tpe_sat_add_llr(int32_t a, int32_t b) noexcept {
    const int64_t s = static_cast<int64_t>(a) + static_cast<int64_t>(b);
    const int64_t lim = 500000;
    const int64_t m_pos = -(s > lim);
    const int64_t m_neg = -(s < -lim);
    const int64_t out = (lim & m_pos) | (-lim & m_neg) | (s & ~(m_pos | m_neg));
    return static_cast<int32_t>(out);
}
// FWHT 나비 연산 — 루프 전개용 (데이터 의존 분기 없음)
#define HTS_FWHT_BF(d_, i_, j_)                                                \
    do {                                                                       \
        int32_t _u = (d_)[(i_)];                                               \
        int32_t _v = (d_)[(j_)];                                               \
        (d_)[(i_)] = static_cast<int32_t>(_u + _v);                            \
        (d_)[(j_)] = static_cast<int32_t>(_u - _v);                            \
    } while (0)
// N=16: 기존 삼중 for 와 동일 순서로 32회 나비 전개 (분기/루프 없음)
static inline void FWHT_Unroll16(int32_t *d) noexcept {
    HTS_FWHT_BF(d, 0, 1);
    HTS_FWHT_BF(d, 2, 3);
    HTS_FWHT_BF(d, 4, 5);
    HTS_FWHT_BF(d, 6, 7);
    HTS_FWHT_BF(d, 8, 9);
    HTS_FWHT_BF(d, 10, 11);
    HTS_FWHT_BF(d, 12, 13);
    HTS_FWHT_BF(d, 14, 15);
    HTS_FWHT_BF(d, 0, 2);
    HTS_FWHT_BF(d, 1, 3);
    HTS_FWHT_BF(d, 4, 6);
    HTS_FWHT_BF(d, 5, 7);
    HTS_FWHT_BF(d, 8, 10);
    HTS_FWHT_BF(d, 9, 11);
    HTS_FWHT_BF(d, 12, 14);
    HTS_FWHT_BF(d, 13, 15);
    HTS_FWHT_BF(d, 0, 4);
    HTS_FWHT_BF(d, 1, 5);
    HTS_FWHT_BF(d, 2, 6);
    HTS_FWHT_BF(d, 3, 7);
    HTS_FWHT_BF(d, 8, 12);
    HTS_FWHT_BF(d, 9, 13);
    HTS_FWHT_BF(d, 10, 14);
    HTS_FWHT_BF(d, 11, 15);
    HTS_FWHT_BF(d, 0, 8);
    HTS_FWHT_BF(d, 1, 9);
    HTS_FWHT_BF(d, 2, 10);
    HTS_FWHT_BF(d, 3, 11);
    HTS_FWHT_BF(d, 4, 12);
    HTS_FWHT_BF(d, 5, 13);
    HTS_FWHT_BF(d, 6, 14);
    HTS_FWHT_BF(d, 7, 15);
}
// 열 c (0..15): 인덱스 c, c+16, c+32, c+48 에 대한 in-place WHT₄
#define HTS_FWHT_WHT4_COL(d_, c_)                                              \
    do {                                                                       \
        HTS_FWHT_BF(d_, c_, (c_) + 16);                                        \
        HTS_FWHT_BF(d_, (c_) + 32, (c_) + 48);                                 \
        HTS_FWHT_BF(d_, c_, (c_) + 32);                                        \
        HTS_FWHT_BF(d_, (c_) + 16, (c_) + 48);                                 \
    } while (0)
// N=64 = H₄ ⊗ H₁₆: 행(연속 16)별 FWHT₁₆ ×4 → 열 16개에 stride-16 WHT₄ (총 나비
// 192 = 32×6)
static inline void FWHT_Unroll64(int32_t *d) noexcept {
    FWHT_Unroll16(d + 0);
    FWHT_Unroll16(d + 16);
    FWHT_Unroll16(d + 32);
    FWHT_Unroll16(d + 48);
    HTS_FWHT_WHT4_COL(d, 0);
    HTS_FWHT_WHT4_COL(d, 1);
    HTS_FWHT_WHT4_COL(d, 2);
    HTS_FWHT_WHT4_COL(d, 3);
    HTS_FWHT_WHT4_COL(d, 4);
    HTS_FWHT_WHT4_COL(d, 5);
    HTS_FWHT_WHT4_COL(d, 6);
    HTS_FWHT_WHT4_COL(d, 7);
    HTS_FWHT_WHT4_COL(d, 8);
    HTS_FWHT_WHT4_COL(d, 9);
    HTS_FWHT_WHT4_COL(d, 10);
    HTS_FWHT_WHT4_COL(d, 11);
    HTS_FWHT_WHT4_COL(d, 12);
    HTS_FWHT_WHT4_COL(d, 13);
    HTS_FWHT_WHT4_COL(d, 14);
    HTS_FWHT_WHT4_COL(d, 15);
}
#undef HTS_FWHT_WHT4_COL
#undef HTS_FWHT_BF
// ── CRC-16/CCITT ──
uint16_t FEC_HARQ::CRC16(const uint8_t *d, int len) noexcept {
    if (!d || len <= 0)
        return 0u;
    uint16_t crc = 0xFFFFu;
    for (int i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(d[i]) << 8u;
        for (int b = 0; b < 8; ++b) {
            const uint16_t poly_mask =
                static_cast<uint16_t>(0u - ((crc >> 15u) & 1u));
            crc = static_cast<uint16_t>((crc << 1u) ^ (0x1021u & poly_mask));
        }
    }
    return crc;
}
// ── FWHT (int32_t, 가변 크기: 16 또는 64) ───────────────────
void FEC_HARQ::FWHT(int32_t *d, int n) noexcept {
    if (d == nullptr || n <= 1) {
        return;
    }
    if (n == 16) {
        FWHT_Unroll16(d);
        return;
    }
    if (n == 64) {
        FWHT_Unroll64(d);
        return;
    }
    for (int len = 1; len < n; len <<= 1) {
        for (int i = 0; i < n; i += (len << 1)) {
            for (int j = 0; j < len; ++j) {
                const int32_t u = d[i + j];
                const int32_t v = d[i + len + j];
                d[i + j] = u + v;
                d[i + len + j] = u - v;
            }
        }
    }
}
// ── 7비트 Popcount LUT ──
static constexpr uint8_t k_pc7_lut[128] = {
    0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4, 1, 2, 2, 3, 2, 3,
    3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4,
    3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 1, 2,
    2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5,
    3, 4, 4, 5, 4, 5, 5, 6, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5,
    5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7};
static constexpr int pc7(uint8_t x) noexcept { return k_pc7_lut[x & 0x7Fu]; }
// ── Conv Encoder ──
void FEC_HARQ::Conv_Encode(const uint8_t *in, int n, uint8_t *out) noexcept {
    uint8_t sr = 0u;
    for (int i = 0; i < n; ++i) {
        uint8_t r = static_cast<uint8_t>(((in[i] & 1u) << 6u) | sr);
        out[2 * i] = static_cast<uint8_t>(pc7(r & G0) & 1);
        out[2 * i + 1] = static_cast<uint8_t>(pc7(r & G1) & 1);
        sr = static_cast<uint8_t>((r >> 1u) & 0x3Fu);
    }
}
// ── Soft Viterbi ───────────────────────────────────────────
//
void FEC_HARQ::Viterbi_Decode(const int32_t *soft, int nc, uint8_t *out, int no,
                              WorkBuf &wb) noexcept {
    if (!soft || !out || nc < 2 || no < 1)
        return;
    // ⑨ T = nc>>1
    const int T = nc >> 1;
    //  TPE min: steps 상한 VIT_STEPS
    const int32_t T_lt =
        static_cast<int32_t>(0u - static_cast<uint32_t>(T < VIT_STEPS));
    const int steps = (T & T_lt) | (VIT_STEPS & ~T_lt);
    static constexpr int32_t DEAD_STATE = -1000000000;
    for (int s = 0; s < 64; ++s)
        wb.pm[0][s] = DEAD_STATE;
    wb.pm[0][0] = 0;
    int cur = 0;
    for (int t = 0; t < steps; ++t) {
        int nxt = 1 - cur;
        for (int s = 0; s < 64; ++s)
            wb.pm[nxt][s] = DEAD_STATE;
        int32_t s0 = soft[2 * t], s1 = soft[2 * t + 1];
        for (int st = 0; st < 64; ++st) {
            const int32_t pm_st = wb.pm[cur][st];
            const uint32_t m_alive =
                0u - static_cast<uint32_t>(pm_st > DEAD_STATE);
            for (int bit = 0; bit <= 1; ++bit) {
                uint8_t r =
                    static_cast<uint8_t>((static_cast<uint8_t>(bit) << 6u) |
                                         static_cast<uint8_t>(st));
                int ns = static_cast<int>((r >> 1u) & 0x3Fu);
                int e0 = pc7(static_cast<uint8_t>(r & G0)) & 1;
                int e1 = pc7(static_cast<uint8_t>(r & G1)) & 1;
                int32_t bm = s0 * (1 - 2 * e0) + s1 * (1 - 2 * e1);
                int32_t np = pm_st + bm;
                const int32_t old_pm = wb.pm[nxt][ns];
                const uint32_t take =
                    (0u - static_cast<uint32_t>(np > old_pm)) & m_alive;
                wb.pm[nxt][ns] = static_cast<int32_t>(
                    (static_cast<uint32_t>(np) & take) |
                    (static_cast<uint32_t>(old_pm) & ~take));
                const uint32_t new_st_u = static_cast<uint32_t>(st);
                const uint32_t old_st_u = static_cast<uint32_t>(wb.surv[t][ns]);
                wb.surv[t][ns] = static_cast<uint8_t>((new_st_u & take) |
                                                      (old_st_u & ~take));
            }
        }
        cur = nxt;
    }
    int state = 0;
    for (int t = steps - 1; t >= 0; --t) {
        state &= 63;
        wb.tb[t] = static_cast<uint8_t>((state >> 5) & 1);
        state = static_cast<int>(wb.surv[t][state]) & 63;
    }
    for (int i = 0; i < no && i < steps; ++i)
        out[i] = wb.tb[i];
}
// ── 부호 상관 LLR + 경로별 고정 시프트 정규화 ──────
//  부호 상관: corr[m] = fI[m] + fQ[m] (부호 보존)
//  Conv: 비트별 LLR = Σ corr(bit=0) − Σ corr(bit=1), 출력 >>12 (Viterbi 메트릭)
//  Polar: Max-Log-MAP raw, 출력 >>10 (SCL min-sum 해상도)
void FEC_HARQ::Bin_To_LLR(const int32_t *fI, const int32_t *fQ, int nc, int bps,
                          int32_t *llr) noexcept {
    const int nsym = 1 << bps;
    const int32_t ns_lt =
        static_cast<int32_t>(0u - static_cast<uint32_t>(nsym < nc));
    const int valid = (nsym & ns_lt) | (nc & ~ns_lt);
    int32_t raw[BPS64_MAX]{};
    for (int b = 0; b < bps; ++b) {
        const int sh_bit = bps - 1 - b;
#if defined(HTS_FEC_POLAR_ENABLE)
        // Max-Log-MAP: max(corr|bit=0) - max(corr|bit=1) — Sum 대비 잡음 빈 누적 완화
        int32_t pos_max = INT32_MIN;
        int32_t neg_max = INT32_MIN;
        for (int m = 0; m < valid; ++m) {
            const int32_t corr = fI[m] + fQ[m];
            const uint32_t is_one = 0u - ((static_cast<uint32_t>(m) >>
                                           static_cast<uint32_t>(sh_bit)) &
                                          1u);
            const uint32_t is_zero = ~is_one;
            if (is_zero != 0u && corr > pos_max) {
                pos_max = corr;
            }
            if (is_one != 0u && corr > neg_max) {
                neg_max = corr;
            }
        }
        raw[b] = pos_max - neg_max;
#else
        int32_t pos_sum = 0;
        int32_t neg_sum = 0;
        for (int m = 0; m < valid; ++m) {
            const int32_t corr = fI[m] + fQ[m];
            const uint32_t is_one = 0u - ((static_cast<uint32_t>(m) >>
                                           static_cast<uint32_t>(sh_bit)) &
                                          1u);
            pos_sum += corr & static_cast<int32_t>(~is_one);
            neg_sum += corr & static_cast<int32_t>(is_one);
        }
        raw[b] = pos_sum - neg_sum;
#endif
    }
    // 고정 시프트 정규화: Polar >>10 / Conv >>12
    // max|raw| ≈ valid × max|corr|; 시프트 후 tpe_clamp_llr 범위 내
    // UDIV 제거 → ARM 사이클 절감
    for (int b = 0; b < bps; ++b) {
#if defined(HTS_FEC_POLAR_ENABLE)
        // Polar용: >>10 (SCL min-sum 해상도 확보)
        // ±38400 >> 10 = ±37, HARQ 32R 누적 = ±1184 < int16 ✓
        llr[b] = raw[b] >> 10;
#else
        // Conv용: >>12 (Viterbi 경로 메트릭 최적화)
        llr[b] = raw[b] >> 12;
#endif
    }
}
// ── Xorshift PRNG ──
static uint32_t xs(uint32_t s) noexcept {
    s ^= s << 13u;
    s ^= s >> 17u;
    s ^= s << 5u;
    return s;
}
// Lemire fast range reduction: [0, range) 균등 매핑 (mod/div 회피)
static inline uint32_t fast_range32(uint32_t x, uint32_t range) noexcept {
    return static_cast<uint32_t>(
        (static_cast<uint64_t>(x) * static_cast<uint64_t>(range)) >> 32u);
}
// =====================================================================
//  [항목⑨ 주석] Fisher-Yates 셔플 — 모듈로(%) 불가피 사유
//
//  분모 (i+1)은 매 반복마다 1씩 감소하는 가변값이므로
//  2의 거듭제곱 시프트/마스크로 대체할 수 없습니다.
//  균등 분포 보장을 위해 정확한 나머지 연산이 필수입니다.
//
//  ARM UDIV: 2~12cyc/회 × TOTAL_CODED(688) ≈ 최대 8,256cyc
//  이는 패킷당 1회 실행 (TX 인코딩 또는 RX 디코딩 시)이므로
//  168MHz 기준 ~49µs — 실시간 제약(1ms 프레임) 내 충분합니다.
//
//  대안 검토:
//   · 비트 마스크 + 리젝션: 균등 분포 보장하나 루프 비결정론
//   · 곱셈 기반 (Lemire): 64비트 곱셈 필요 → ARM __aeabi_lmul
//   → 현재 UDIV가 가장 단순하고 결정론적 (양산 안정성 우선)
// =====================================================================
static_assert(FEC_HARQ::TOTAL_CODED <= 1024,
              "TOTAL_CODED > 1024: Fisher-Yates UDIV 오버헤드 재검토 필요");
void FEC_HARQ::Bit_Interleave(uint8_t *bits, int n, uint32_t seed) noexcept {
    if (!bits || n < 2)
        return;
    if (n > TOTAL_CODED)
        return;
    uint32_t s = (seed == 0u) ? 0xDEADBEEFu : seed;
    for (int i = n - 1; i > 0; --i) {
        s = xs(s);
        const uint32_t range = static_cast<uint32_t>(i + 1);
        const int j = static_cast<int>(fast_range32(s, range));
        uint8_t t = bits[i];
        bits[i] = bits[j];
        bits[j] = t;
    }
}
void FEC_HARQ::Bit_Deinterleave(int32_t *soft, int n, uint32_t seed,
                                WorkBuf &wb) noexcept {
    if (!soft || n < 2)
        return;
    if (n > TOTAL_CODED)
        return;
    for (int i = 0; i < n; ++i) {
        wb.perm[i] = static_cast<uint16_t>(i);
    }
    uint32_t s = (seed == 0u) ? 0xDEADBEEFu : seed;
    for (int i = n - 1; i > 0; --i) {
        s = xs(s);
        const uint32_t range = static_cast<uint32_t>(i + 1);
        const int j = static_cast<int>(fast_range32(s, range));
        const uint16_t t = wb.perm[i];
        wb.perm[i] = wb.perm[static_cast<size_t>(j)];
        wb.perm[static_cast<size_t>(j)] = t;
    }
    // perm 이 0..n-1 순열이므로 아래 루프가 tmp_soft[0..n-1] 전부를 한 번씩
    // 덮어씀 — memset 불필요
    for (int i = 0; i < n; ++i) {
        wb.tmp_soft[static_cast<size_t>(wb.perm[i])] = soft[i];
    }
    for (int i = 0; i < n; ++i)
        soft[i] = wb.tmp_soft[i];
}
void FEC_HARQ::Gen_Perm(uint32_t seed, uint8_t *p, int n) noexcept {
    if (!p || n <= 0 || n > C64)
        return;
    for (int i = 0; i < n; ++i)
        p[i] = static_cast<uint8_t>(i);
    uint32_t s = (seed == 0u) ? 0xDEADBEEFu : seed;
    for (int i = n - 1; i > 0; --i) {
        s = xs(s);
        const uint32_t range = static_cast<uint32_t>(i + 1);
        const int j = static_cast<int>(fast_range32(s, range));
        uint8_t t = p[i];
        p[i] = p[j];
        p[j] = t;
    }
}
void FEC_HARQ::Interleave(int16_t *I, int16_t *Q, const uint8_t *p,
                          int n) noexcept {
    if (!I || !Q || !p || n <= 0 || n > C64)
        return;
    int16_t tI[C64] = {}, tQ[C64] = {};
    for (int i = 0; i < n; ++i) {
        tI[p[i]] = I[i];
        tQ[p[i]] = Q[i];
    }
    for (int i = 0; i < n; ++i) {
        I[i] = tI[i];
        Q[i] = tQ[i];
    }
}
void FEC_HARQ::Deinterleave(int16_t *I, int16_t *Q, const uint8_t *p,
                            int n) noexcept {
    if (!I || !Q || !p || n <= 0 || n > C64)
        return;
    int16_t tI[C64] = {}, tQ[C64] = {};
    for (int i = 0; i < n; ++i) {
        tI[i] = I[p[i]];
        tQ[i] = Q[p[i]];
    }
    for (int i = 0; i < n; ++i) {
        I[i] = tI[i];
        Q[i] = tQ[i];
    }
}
#if defined(HTS_FEC_POLAR_ENABLE)
// Polar(512,80) 인코딩 + 688 cyclic repetition → 심볼 배열 (64칩 전용)
static int polar_encode_to_syms(const uint8_t *info, int len, uint8_t *syms,
                                int bps, int nsym) noexcept {
    static constexpr int k_pn = 512;
    if (info == nullptr || syms == nullptr || len <= 0 ||
        len > FEC_HARQ::MAX_INFO) {
        return 0;
    }
    uint8_t coded[static_cast<std::size_t>(k_pn / 8)] = {};
    if (HTS_Polar_Codec::Encode(info, len, coded) != k_pn) {
        return 0;
    }
    for (int s = 0; s < nsym; ++s) {
        uint8_t val = 0u;
        for (int b = 0; b < bps; ++b) {
            const int bi_688 = s * bps + b;
            const int bi_512 = bi_688 % k_pn;
            const int byte_idx = bi_512 >> 3;
            const int bit_idx = 7 - (bi_512 & 7);
            val |= static_cast<uint8_t>(
                ((coded[static_cast<std::size_t>(byte_idx)] >>
                  static_cast<unsigned>(bit_idx)) &
                 1u)
                << static_cast<unsigned>(bps - 1 - b));
        }
        syms[s] = val;
    }
    return nsym;
}
#endif
// =================================================================
//  Encode Core
// =================================================================
int FEC_HARQ::Encode_Core(const uint8_t *info, int len, uint8_t *syms,
                          uint32_t il, int bps, int nsym,
                          WorkBuf &wb) noexcept {
    const uint32_t p_ok = (static_cast<uint32_t>(info != nullptr) &
                           static_cast<uint32_t>(syms != nullptr));
    const uint32_t len_ok = (static_cast<uint32_t>(len >= 1) &
                             static_cast<uint32_t>(len <= MAX_INFO));
    const uint32_t bps_ok = (static_cast<uint32_t>(bps >= 1) &
                             static_cast<uint32_t>(bps <= BPS64_MAX));
    const uint32_t nsym_ok = static_cast<uint32_t>(nsym > 0);
    const int64_t slots =
        static_cast<int64_t>(nsym) * static_cast<int64_t>(bps);
    const uint32_t grid_ok = static_cast<uint32_t>(
        slots >= static_cast<int64_t>(TOTAL_CODED));
    const uint32_t full_ok =
        p_ok & len_ok & bps_ok & nsym_ok & grid_ok;

    int idx = 0;
    for (int pass = 0; pass < static_cast<int>(full_ok); ++pass) {
        std::array<uint8_t, static_cast<std::size_t>(MAX_INFO + 2)> coded{};
        for (int i = 0; i < len; ++i)
            coded[static_cast<std::size_t>(i)] = info[i];
        uint16_t crc = CRC16(coded.data(), MAX_INFO);
        coded[static_cast<std::size_t>(MAX_INFO)] =
            static_cast<uint8_t>(crc >> 8u);
        coded[static_cast<std::size_t>(MAX_INFO + 1)] =
            static_cast<uint8_t>(crc & 0xFFu);
        std::array<uint8_t, static_cast<std::size_t>(CONV_IN)> in_bits{};
        for (int i = 0; i < INFO_BITS; ++i)
            in_bits[static_cast<std::size_t>(i)] = static_cast<uint8_t>(
                (coded[static_cast<std::size_t>(i >> 3)] >> (7 - (i & 7))) &
                1u);
        std::array<uint8_t, k_conv_out_sz> conv{};
        Conv_Encode(in_bits.data(), CONV_IN, conv.data());
        for (int r = 0; r < REP; ++r)
            for (int i = 0; i < CONV_OUT; ++i)
                wb.ru.rep[r * CONV_OUT + i] = conv[static_cast<std::size_t>(i)];
        Bit_Interleave(wb.ru.rep, TOTAL_CODED, il);
        idx = 0;
        for (int s = 0; s < nsym; ++s) {
            uint8_t sym = 0u;
            for (int b = 0; b < bps; ++b) {
                int bi = s * bps + b;
                // TPE: OOB → safe_bi=0 읽고 마스크로 소거
                const uint32_t in_range =
                    0u - static_cast<uint32_t>(bi < TOTAL_CODED);
                const int safe_bi = bi & static_cast<int>(in_range);
                sym |= static_cast<uint8_t>(
                    (wb.ru.rep[safe_bi] << (bps - 1 - b)) &
                    static_cast<uint8_t>(in_range));
            }
            syms[idx++] = sym;
        }
    }
    return idx;
}
// =================================================================
//  Decode Core
// =================================================================
bool FEC_HARQ::Decode_Core(const int32_t *accI, const int32_t *accQ, int nsym,
                           int nc, int bps, uint8_t *out, int *olen,
                           uint32_t il, WorkBuf &wb) noexcept {
    if (olen == nullptr) {
        return false;
    }
    *olen = 0;

    const uint32_t ptr_ok = (static_cast<uint32_t>(accI != nullptr) &
                             static_cast<uint32_t>(accQ != nullptr) &
                             static_cast<uint32_t>(out != nullptr));
    const uint32_t dim_ok = (static_cast<uint32_t>(nsym > 0) &
                             static_cast<uint32_t>(nc > 0) &
                             static_cast<uint32_t>(bps > 0));
    const uint32_t bps_ok = static_cast<uint32_t>(bps <= BPS64_MAX);
    const uint32_t nsym_ok = static_cast<uint32_t>(nsym <= NSYM64);
    const uint32_t nc_ok = static_cast<uint32_t>(nc <= FEC_HARQ::C64);
    const int64_t llr_slots =
        static_cast<int64_t>(nsym) * static_cast<int64_t>(bps);
    const uint32_t slot_ok = static_cast<uint32_t>(
        llr_slots >= static_cast<int64_t>(TOTAL_CODED));
    // TPE: single validity mask — pass loop runs 0× when any check fails
    const uint32_t full_ok =
        ptr_ok & dim_ok & bps_ok & nsym_ok & nc_ok & slot_ok;

    std::array<uint8_t, static_cast<std::size_t>(CONV_IN)> dec{};
    std::array<uint8_t, static_cast<std::size_t>(MAX_INFO + 2)> rx{};
    bool dec_ok = false;

    for (int pass = 0; pass < static_cast<int>(full_ok); ++pass) {
#if defined(HTS_FEC_PROFILE)
        ++g_fec_prof.calls;
        uint64_t fec_t0 = fec_prof_now();
#endif
        std::array<int32_t, k_fwht_buf_sz> &fI = g_fec_dec_fI;
        std::array<int32_t, k_fwht_buf_sz> &fQ = g_fec_dec_fQ;
        std::array<int32_t, k_llr_buf_sz> &llr = g_fec_dec_llr;
        llr.fill(static_cast<int32_t>(0));
        for (int sym = 0; sym < nsym; ++sym) {
            const int base = sym * nc;
            std::memcpy(fI.data(), accI + base,
                        static_cast<std::size_t>(nc) * sizeof(int32_t));
            std::memcpy(fQ.data(), accQ + base,
                        static_cast<std::size_t>(nc) * sizeof(int32_t));
            FWHT(fI.data(), nc);
            FWHT(fQ.data(), nc);
            // DC bin 제거: Walsh bin 0 = Σchips (DC 성분)
            // Pluto LO leakage / ADC offset → bin 0에 집중 → LLR bias 방지
            fI[0] = 0;
            fQ[0] = 0;
            Bin_To_LLR(fI.data(), fQ.data(), nc, bps, llr.data());
            for (int b = 0; b < bps; ++b) {
                const int bi = sym * bps + b;
                // TPE: bi < TOTAL_CODED 마스크 — OOB 방지
                const uint32_t in_range =
                    0u - static_cast<uint32_t>(bi < TOTAL_CODED);
                const int32_t val =
                    tpe_clamp_llr(llr[static_cast<std::size_t>(b)]) &
                    static_cast<int32_t>(in_range);
                wb.ru.all_llr[static_cast<std::size_t>(bi)] = val;
            }
        }
#if defined(HTS_FEC_PROFILE)
        uint64_t fec_t1 = fec_prof_now();
        g_fec_prof.sym += (fec_t1 - fec_t0);
        fec_t0 = fec_t1;
#endif
        Bit_Deinterleave(wb.ru.all_llr, TOTAL_CODED, il, wb);
#if defined(HTS_FEC_PROFILE)
        fec_t1 = fec_prof_now();
        g_fec_prof.deint += (fec_t1 - fec_t0);
        fec_t0 = fec_t1;
#endif
        for (int i = 0; i < CONV_OUT; ++i) {
            int32_t acc = wb.ru.all_llr[i];
            for (int r = 1; r < REP; ++r) {
                acc = tpe_sat_add_llr(acc, wb.ru.all_llr[r * CONV_OUT + i]);
            }
            wb.ru.all_llr[i] = acc;
        }
#if defined(HTS_FEC_PROFILE)
        fec_t1 = fec_prof_now();
        g_fec_prof.rep += (fec_t1 - fec_t0);
        fec_t0 = fec_t1;
#endif
        dec.fill(static_cast<uint8_t>(0));
        Viterbi_Decode(wb.ru.all_llr, CONV_OUT, dec.data(), CONV_IN, wb);
#if defined(HTS_FEC_PROFILE)
        fec_t1 = fec_prof_now();
        g_fec_prof.vit += (fec_t1 - fec_t0);
        fec_t0 = fec_t1;
#endif
        rx.fill(static_cast<uint8_t>(0));
        for (int i = 0; i < INFO_BITS; ++i) {
            const uint32_t bit =
                static_cast<uint32_t>(dec[static_cast<std::size_t>(i)]) & 1u;
            rx[static_cast<std::size_t>(i >> 3)] |=
                static_cast<uint8_t>(bit << static_cast<unsigned>(7 - (i & 7)));
        }
        uint16_t calc = CRC16(rx.data(), MAX_INFO);
        uint16_t stored =
            (static_cast<uint16_t>(rx[static_cast<std::size_t>(MAX_INFO)])
             << 8u) |
            static_cast<uint16_t>(rx[static_cast<std::size_t>(MAX_INFO + 1)]);
#if defined(HTS_FEC_PROFILE)
        fec_t1 = fec_prof_now();
        g_fec_prof.tail += (fec_t1 - fec_t0);
#endif
        const uint32_t ok_mask = 0u - static_cast<uint32_t>(calc == stored);
        for (int i = 0; i < MAX_INFO; ++i) {
            out[i] = static_cast<uint8_t>(
                static_cast<uint32_t>(rx[static_cast<std::size_t>(i)]) &
                ok_mask);
        }
        *olen = static_cast<int>(static_cast<uint32_t>(MAX_INFO) & ok_mask);
        dec_ok = (ok_mask != 0u);
    }

    fec_secure_wipe_stack(static_cast<void *>(dec.data()), dec.size());
    fec_secure_wipe_stack(static_cast<void *>(rx.data()), rx.size());
    return dec_ok;
}
// ── 16칩 래퍼 ──
int FEC_HARQ::Encode16(const uint8_t *info, int len, uint8_t *syms, uint32_t il,
                       WorkBuf &wb) noexcept {
    return Encode_Core(info, len, syms, il, BPS16, NSYM16, wb);
}
void FEC_HARQ::Init16(RxState16 &s) noexcept {
    std::memset(static_cast<void *>(&s), 0, sizeof(s));
}
void FEC_HARQ::Feed16(RxState16 &s, const int16_t I[][C16],
                      const int16_t Q[][C16]) noexcept {
    if (s.ok)
        return;
    for (int sym = 0; sym < NSYM16; ++sym)
        for (int c = 0; c < C16; ++c) {
            s.aI[sym][c] += static_cast<int32_t>(I[sym][c]);
            s.aQ[sym][c] += static_cast<int32_t>(Q[sym][c]);
        }
    s.k++;
}
bool FEC_HARQ::Decode16(const RxState16 &s, uint8_t *out, int *len, uint32_t il,
                        WorkBuf &wb) noexcept {
    return Decode_Core(&s.aI[0][0], &s.aQ[0][0], NSYM16, C16, BPS16, out, len,
                       il, wb);
}
// ── 64칩 래퍼 ──
int FEC_HARQ::Encode64(const uint8_t *info, int len, uint8_t *syms, uint32_t il,
                       WorkBuf &wb) noexcept {
#if defined(HTS_FEC_POLAR_ENABLE)
    (void)il;
    (void)wb;
    return polar_encode_to_syms(info, len, syms, BPS64, NSYM64);
#else
    return Encode_Core(info, len, syms, il, BPS64, NSYM64, wb);
#endif
}
void FEC_HARQ::Init64(RxState64 &s) noexcept {
    std::memset(static_cast<void *>(&s), 0, sizeof(s));
}
void FEC_HARQ::Feed64(RxState64 &s, const int16_t I[][C64],
                      const int16_t Q[][C64]) noexcept {
    if (s.ok)
        return;
    for (int sym = 0; sym < NSYM64; ++sym)
        for (int c = 0; c < C64; ++c) {
            s.aI[sym][c] += static_cast<int32_t>(I[sym][c]);
            s.aQ[sym][c] += static_cast<int32_t>(Q[sym][c]);
        }
    s.k++;
}
bool FEC_HARQ::Decode64(const RxState64 &s, uint8_t *out, int *len, uint32_t il,
                        WorkBuf &wb) noexcept {
    return Decode_Core(&s.aI[0][0], &s.aQ[0][0], NSYM64, C64, BPS64, out, len,
                       il, wb);
}
// ── 적응형 64칩 API ──
int FEC_HARQ::Encode64_A(const uint8_t *info, int len, uint8_t *syms,
                         uint32_t il, int bps, WorkBuf &wb) noexcept {
    if (bps < BPS64_MIN_OPERABLE || bps > BPS64_MAX)
        return 0;
    if (nsym_for_bps(bps) > NSYM64)
        return 0;
#if defined(HTS_FEC_POLAR_ENABLE)
    (void)il;
    (void)wb;
    return polar_encode_to_syms(info, len, syms, bps, nsym_for_bps(bps));
#else
    return Encode_Core(info, len, syms, il, bps, nsym_for_bps(bps), wb);
#endif
}
void FEC_HARQ::Feed64_A(RxState64 &s, const int16_t I[][C64],
                        const int16_t Q[][C64], int nsym) noexcept {
    if (s.ok)
        return;
    if (nsym > NSYM64)
        nsym = NSYM64;
    for (int sym = 0; sym < nsym; ++sym)
        for (int c = 0; c < C64; ++c) {
            s.aI[sym][c] += static_cast<int32_t>(I[sym][c]);
            s.aQ[sym][c] += static_cast<int32_t>(Q[sym][c]);
        }
    s.k++;
}
// ── Feed16_1sym — 16칩 심볼 1개 즉시 HARQ 누적 ─────────────
void FEC_HARQ::Feed16_1sym(RxState16 &s, const int16_t *I, const int16_t *Q,
                           int sym_idx) noexcept {
    if (s.ok)
        return;
    if (sym_idx < 0 || sym_idx >= NSYM16)
        return;
    if (!I || !Q)
        return;
    for (int c = 0; c < C16; ++c) {
        s.aI[sym_idx][c] += static_cast<int32_t>(I[c]);
        s.aQ[sym_idx][c] += static_cast<int32_t>(Q[c]);
    }
}
// ── Feed64_1sym — 64칩 심볼 1개 즉시 HARQ 누적 ─────────────
void FEC_HARQ::Feed64_1sym(RxState64 &s, const int16_t *I, const int16_t *Q,
                           int sym_idx) noexcept {
    if (s.ok)
        return;
    if (sym_idx < 0 || sym_idx >= NSYM64)
        return;
    if (!I || !Q)
        return;
    for (int c = 0; c < C64; ++c) {
        s.aI[sym_idx][c] += static_cast<int32_t>(I[c]);
        s.aQ[sym_idx][c] += static_cast<int32_t>(Q[c]);
    }
}
// ── Advance_Round — 스트리밍 Feed 후 라운드 카운터 증가 ─────
void FEC_HARQ::Advance_Round_16(RxState16 &s) noexcept {
    if (!s.ok)
        s.k++;
}
void FEC_HARQ::Advance_Round_64(RxState64 &s) noexcept {
    if (!s.ok)
        s.k++;
}
bool FEC_HARQ::Decode64_A(const RxState64 &s, uint8_t *out, int *len,
                          uint32_t il, int bps, WorkBuf &wb) noexcept {
    if (bps < BPS64_MIN_OPERABLE || bps > BPS64_MAX)
        return false;
    if (nsym_for_bps(bps) > NSYM64)
        return false;
    return Decode_Core(&s.aI[0][0], &s.aQ[0][0], nsym_for_bps(bps), C64, bps,
                       out, len, il, wb);
}
// ── Decode_Core_Split — I/Q 분리 배치용 Decode 래퍼 ──────────
bool FEC_HARQ::Decode_Core_Split(const int32_t *accI, const int32_t *accQ,
                                 int nsym, int nc, int bps, uint8_t *out,
                                 int *len, uint32_t il, WorkBuf &wb) noexcept {
    if (!accI || !accQ || !out || !len)
        return false;
    if (nsym <= 0 || nc <= 0)
        return false;
    if (bps < BPS64_MIN_OPERABLE || bps > BPS64_MAX)
        return false;
    if (nsym > NSYM64) {
        *len = 0;
        return false;
    }
    return Decode_Core(accI, accQ, nsym, nc, bps, out, len, il, wb);
}
// ── 1칩 BPSK ──
int FEC_HARQ::Encode1(const uint8_t *info, int len, uint8_t *syms) noexcept {
    if (!info || !syms || len < 1 || len > MAX_INFO)
        return 0;
    uint8_t coded[MAX_INFO + 2] = {};
    for (int i = 0; i < len; ++i)
        coded[i] = info[i];
    uint16_t crc = CRC16(coded, MAX_INFO);
    coded[MAX_INFO] = static_cast<uint8_t>(crc >> 8u);
    coded[MAX_INFO + 1] = static_cast<uint8_t>(crc & 0xFFu);
    for (int i = 0; i < INFO_BITS; ++i)
        syms[i] = static_cast<uint8_t>((coded[i >> 3] >> (7 - (i & 7))) & 1u);
    return INFO_BITS;
}
bool FEC_HARQ::Decode1(const int16_t *rx_I, uint8_t *out, int *len) noexcept {
    if (!rx_I || !out || !len)
        return false;
    uint8_t rx[MAX_INFO + 2] = {};
    for (int i = 0; i < INFO_BITS; ++i) {
        const int32_t si = static_cast<int32_t>(rx_I[i]);
        const uint32_t sign_mask = static_cast<uint32_t>(si >> 31) & 1u;
        rx[static_cast<std::size_t>(i >> 3)] |= static_cast<uint8_t>(
            sign_mask << static_cast<unsigned>(7 - (i & 7)));
    }
    uint16_t calc = CRC16(rx, MAX_INFO);
    uint16_t stored = (static_cast<uint16_t>(rx[MAX_INFO]) << 8u) |
                      static_cast<uint16_t>(rx[MAX_INFO + 1]);
    const bool ok = (calc == stored);
    if (ok) {
        for (int i = 0; i < MAX_INFO; ++i) {
            out[i] = rx[static_cast<std::size_t>(i)];
        }
        *len = MAX_INFO;
    } else {
        *len = 0;
    }
    fec_secure_wipe_stack(static_cast<void *>(rx), sizeof(rx));
    return ok;
}
// =================================================================
//  IR-HARQ (RV 인터리브 + LLR 누적) — 64칩 적응형과 분리 섹션
// =================================================================
void FEC_HARQ::IR_Init(IR_RxState &s) noexcept {
    SecureMemory::secureWipe(static_cast<void *>(&s), sizeof(s));
}
void FEC_HARQ::Set_IR_Erasure_Enabled(bool enable) noexcept {
    g_ir_erasure_en.store(enable ? 1u : 0u, std::memory_order_relaxed);
}
bool FEC_HARQ::Get_IR_Erasure_Enabled() noexcept {
    return g_ir_erasure_en.load(std::memory_order_relaxed) != 0u;
}
void FEC_HARQ::Set_IR_Rs_Post_Enabled(bool enable) noexcept {
    g_ir_rs_post_en.store(enable ? 1u : 0u, std::memory_order_relaxed);
}
bool FEC_HARQ::Get_IR_Rs_Post_Enabled() noexcept {
    return g_ir_rs_post_en.load(std::memory_order_relaxed) != 0u;
}
int FEC_HARQ::Encode64_IR(const uint8_t *info, int len, uint8_t *syms,
                          uint32_t il_seed, int bps, int rv,
                          WorkBuf &wb) noexcept {
    if (bps < BPS64_MIN_OPERABLE || bps > BPS64_MAX)
        return 0;
    const int ns = nsym_for_bps(bps);
    if (ns > NSYM64)
        return 0;
#if defined(HTS_FEC_POLAR_ENABLE)
    (void)wb;
    (void)il_seed;
    (void)rv;
    return polar_encode_to_syms(info, len, syms, bps, ns);
#else
    const uint32_t il_eff = il_seed ^ RV_SALT[static_cast<std::size_t>(rv & 3)];
    return Encode_Core(info, len, syms, il_eff, bps, ns, wb);
#endif
}
bool FEC_HARQ::Decode64_IR(const int16_t *sym_I, const int16_t *sym_Q, int nsym,
                           int nc, int bps, uint32_t il_seed, int rv,
                           IR_RxState &ir_state, uint8_t *out, int *olen,
                           WorkBuf &wb) noexcept {
    if (!out || !olen)
        return false;
    if (ir_state.ok) {
        *olen = MAX_INFO;
        return true;
    }
    if (!sym_I || !sym_Q)
        return false;
    if (nsym <= 0 || nc <= 0 || bps <= 0)
        return false;
    if (bps < BPS64_MIN_OPERABLE || bps > BPS64_MAX) {
        *olen = 0;
        return false;
    }
    if (nsym_for_bps(bps) > NSYM64) {
        *olen = 0;
        return false;
    }
    if (nsym > NSYM64) {
        *olen = 0;
        return false;
    }
    if (nc > C64) {
        *olen = 0;
        return false;
    }
    const int64_t llr_slots =
        static_cast<int64_t>(nsym) * static_cast<int64_t>(bps);
    if (llr_slots < static_cast<int64_t>(TOTAL_CODED)) {
        *olen = 0;
        return false;
    }
    std::array<int32_t, k_fwht_buf_sz> &fI = g_fec_dec_fI;
    std::array<int32_t, k_fwht_buf_sz> &fQ = g_fec_dec_fQ;
    std::array<int32_t, k_llr_buf_sz> &llr = g_fec_dec_llr;
    llr.fill(static_cast<int32_t>(0));
    const uint32_t er_en =
        static_cast<uint32_t>(g_ir_erasure_en.load(std::memory_order_relaxed));
    for (int sym = 0; sym < nsym; ++sym) {
        const int base = sym * nc;
        for (int c = 0; c < nc; ++c) {
            const int32_t Ii = static_cast<int32_t>(sym_I[base + c]);
            const int32_t Qi = static_cast<int32_t>(sym_Q[base + c]);
            static constexpr int32_t kErasureMagTh = 20000;
            const int32_t mag =
                fec_ir_fast_abs_i32(Ii) + fec_ir_fast_abs_i32(Qi);
            const uint32_t allow_chip =
                (1u - er_en) | static_cast<uint32_t>(mag <= kErasureMagTh);
            const int32_t mask = -static_cast<int32_t>(allow_chip);
            fI[static_cast<std::size_t>(c)] = Ii & mask;
            fQ[static_cast<std::size_t>(c)] = Qi & mask;
        }
        FWHT(fI.data(), nc);
        FWHT(fQ.data(), nc);
        // DC bin 제거: Walsh bin 0 = Σchips (DC 성분)
        fI[0] = 0;
        fQ[0] = 0;
        Bin_To_LLR(fI.data(), fQ.data(), nc, bps, llr.data());
        for (int b = 0; b < bps; ++b) {
            const int bi = sym * bps + b;
            const uint32_t in_range =
                0u - static_cast<uint32_t>(bi < TOTAL_CODED);
            wb.ru.all_llr[static_cast<std::size_t>(bi)] =
                tpe_clamp_llr(llr[static_cast<std::size_t>(b)]) &
                static_cast<int32_t>(in_range);
        }
    }
#if defined(HTS_FEC_POLAR_ENABLE)
    (void)il_seed;
    (void)rv;
    // ── Polar 칩 누적 디코딩 경로 (CURSOR_POLAR_REVERT_BASELINE) ──
    // 칩 도메인 coherent 누적(I+Q) → 단일 FWHT → Max-Log LLR → Polar
    // (위 심볼 루프의 wb.ru.all_llr는 Polar에서 미사용; Conv #else만 사용)
    // T18: DWT CYCCNT / TSC 구간 계측 (로직 불변)

    static int32_t s_chip_acc[NSYM64 * C64];
    static int16_t polar_llr16[POLAR_N];

    const uint32_t t18_total_start = HTS_DWT::Read();

    if (ir_state.rounds_done == 0) {
        std::memset(static_cast<void *>(s_chip_acc), 0, sizeof(s_chip_acc));
    }

    for (int s = 0; s < nsym; ++s) {
        const int base = s * nc;
        for (int c = 0; c < nc; ++c) {
            const int32_t Ii = static_cast<int32_t>(sym_I[base + c]);
            const int32_t Qi = static_cast<int32_t>(sym_Q[base + c]);
            static constexpr int32_t kErasureMagTh = 20000;
            const int32_t mag =
                fec_ir_fast_abs_i32(Ii) + fec_ir_fast_abs_i32(Qi);
            const uint32_t allow_chip =
                (1u - er_en) | static_cast<uint32_t>(mag <= kErasureMagTh);
            const int32_t mask = -static_cast<int32_t>(allow_chip);
            s_chip_acc[base + c] += (Ii & mask) + (Qi & mask);
        }
    }
    ++ir_state.rounds_done;

    int32_t polar_llr_coded[TOTAL_CODED] = {};

    const uint32_t t18_fwht_start = HTS_DWT::Read();
    for (int sym = 0; sym < nsym; ++sym) {
        const int sym_base = sym * nc;
        for (int c = 0; c < nc; ++c) {
            g_fec_dec_fI[static_cast<std::size_t>(c)] =
                s_chip_acc[sym_base + c];
        }
        FWHT(g_fec_dec_fI.data(), nc);
        // DC bin 제거: Walsh bin 0 = Σchips (DC 성분)
        g_fec_dec_fI[0] = 0;
        const int valid = 1 << bps;
        for (int b = 0; b < bps; ++b) {
            const int sh_bit = bps - 1 - b;
            int32_t pos_max = INT32_MIN;
            int32_t neg_max = INT32_MIN;
            for (int m = 0; m < valid; ++m) {
                const int32_t corr =
                    g_fec_dec_fI[static_cast<std::size_t>(m)];
                const uint32_t is_one =
                    0u - ((static_cast<uint32_t>(m) >>
                           static_cast<uint32_t>(sh_bit)) &
                          1u);
                const uint32_t is_zero = ~is_one;
                if (is_zero != 0u && corr > pos_max) {
                    pos_max = corr;
                }
                if (is_one != 0u && corr > neg_max) {
                    neg_max = corr;
                }
            }
            const int bi = sym * bps + b;
            if (bi < TOTAL_CODED) {
                polar_llr_coded[bi] = (pos_max - neg_max) >> 10;
            }
        }
    }
    const uint32_t t18_fwht_end = HTS_DWT::Read();

    std::memset(static_cast<void *>(ir_state.llr_accum), 0,
                static_cast<std::size_t>(POLAR_N) * sizeof(int32_t));
    static_assert((static_cast<unsigned>(POLAR_N) &
                    (static_cast<unsigned>(POLAR_N) - 1u)) == 0u,
                  "POLAR_N must be power of 2 for AND mask");
    for (int i = 0; i < TOTAL_CODED; ++i) {
        ir_state.llr_accum[static_cast<std::size_t>(
            i & (POLAR_N - 1))] += polar_llr_coded[i];
    }

    int32_t max_abs = 1;
    for (int i = 0; i < POLAR_N; ++i) {
        int32_t a = ir_state.llr_accum[static_cast<std::size_t>(i)];
        if (a < 0) {
            a = -a;
        }
        if (a > max_abs) {
            max_abs = a;
        }
    }
    // ── constant-time shift (while 동치): 최대 20스텝, 데이터 독립 ──
    // 단일 CLZ식(18-clz 등)은 8001·비2^k 경계에서 while과 불일치 가능.
    const uint32_t umx = static_cast<uint32_t>(max_abs);
    int shift = 0;
    for (int k = 0; k < 20; ++k) {
        shift += static_cast<int>(
            ((umx >> static_cast<unsigned>(shift)) > 8000u) &
            static_cast<unsigned>(shift < 20));
    }

    for (int i = 0; i < POLAR_N; ++i) {
        const int32_t v =
            ir_state.llr_accum[static_cast<std::size_t>(i)] >> shift;
        const int32_t m_hi = -static_cast<int32_t>(v > 32767);
        const int32_t m_lo = -static_cast<int32_t>(v < -32768);
        const int32_t clamped =
            (32767 & m_hi) | (-32768 & m_lo) | (v & ~(m_hi | m_lo));
        polar_llr16[static_cast<std::size_t>(i)] =
            static_cast<int16_t>(clamped);
    }
    const uint32_t t18_fold_end = HTS_DWT::Read();

    uint8_t rx[static_cast<std::size_t>(MAX_INFO)] = {};
    int olen_p = 0;
    bool dec_ok =
        HTS_Polar_Codec::Decode_SCL(polar_llr16, rx, &olen_p);
    dec_ok = dec_ok && (olen_p == MAX_INFO);
    const uint32_t t18_scl_end = HTS_DWT::Read();

    const uint32_t ok_mask = 0u - static_cast<uint32_t>(dec_ok);
    const uint32_t fail_mask = ~ok_mask;
    for (int i = 0; i < MAX_INFO; ++i) {
        const uint32_t rxv =
            static_cast<uint32_t>(rx[static_cast<std::size_t>(i)]);
        out[i] = static_cast<uint8_t>(rxv & ok_mask);
        ir_state.sic_tentative[static_cast<std::size_t>(i)] =
            static_cast<uint8_t>(rxv & fail_mask);
    }
    *olen = static_cast<int>(static_cast<uint32_t>(MAX_INFO) & ok_mask);
    ir_state.ok = (ok_mask != 0u);
    ir_state.sic_tentative_valid = fail_mask & 1u;
    fec_secure_wipe_stack(static_cast<void *>(rx), sizeof(rx));

    const uint32_t t18_total_end = HTS_DWT::Read();
    // Step 2: fwht 구간 = nsym×(FWHT+Max-Log); llr는 동일 구간에 포함 → 0
    HTS_DWT::g_prof.fwht_cycles = t18_fwht_end - t18_fwht_start;
    HTS_DWT::g_prof.llr_cycles = 0u;
    HTS_DWT::g_prof.fold_cycles = t18_fold_end - t18_fwht_end;
    HTS_DWT::g_prof.scl_cycles = t18_scl_end - t18_fold_end;
    HTS_DWT::g_prof.total_cycles = t18_total_end - t18_total_start;
    HTS_DWT::g_prof.rounds =
        static_cast<uint32_t>(ir_state.rounds_done);
    HTS_DWT::Stats_Update(HTS_DWT::g_prof);

    return dec_ok;
#else
    const uint32_t il_eff = il_seed ^ RV_SALT[static_cast<std::size_t>(rv & 3)];
    Bit_Deinterleave(wb.ru.all_llr, TOTAL_CODED, il_eff, wb);
    for (int i = 0; i < TOTAL_CODED; ++i) {
        int32_t &slot = ir_state.llr_accum[static_cast<std::size_t>(i)];
        slot =
            tpe_sat_add_llr(slot, wb.ru.all_llr[static_cast<std::size_t>(i)]);
    }
    ++ir_state.rounds_done;
    for (int i = 0; i < CONV_OUT; ++i) {
        int32_t acc = ir_state.llr_accum[static_cast<std::size_t>(i)];
        for (int r = 1; r < REP; ++r) {
            acc = tpe_sat_add_llr(
                acc,
                ir_state.llr_accum[static_cast<std::size_t>(r * CONV_OUT + i)]);
        }
        wb.ru.all_llr[static_cast<std::size_t>(i)] = acc;
    }
    std::array<uint8_t, static_cast<std::size_t>(CONV_IN)> dec{};
    Viterbi_Decode(wb.ru.all_llr, CONV_OUT, dec.data(), CONV_IN, wb);
    std::array<uint8_t, static_cast<std::size_t>(MAX_INFO + 2)> rx{};
    for (int i = 0; i < INFO_BITS; ++i) {
        const uint32_t bit =
            static_cast<uint32_t>(dec[static_cast<std::size_t>(i)]) & 1u;
        rx[static_cast<std::size_t>(i >> 3)] |=
            static_cast<uint8_t>(bit << static_cast<unsigned>(7 - (i & 7)));
    }
    uint16_t calc = CRC16(rx.data(), MAX_INFO);
    uint16_t stored =
        (static_cast<uint16_t>(rx[static_cast<std::size_t>(MAX_INFO)]) << 8u) |
        static_cast<uint16_t>(rx[static_cast<std::size_t>(MAX_INFO + 1)]);
    bool dec_ok = (calc == stored);
    // RS 후처리: 희소 경로 (보안 무관, 분기 유지)
    if (!dec_ok && g_ir_rs_post_en.load(std::memory_order_relaxed) != 0u) {
        if (try_ir_rs_recover_rx8(rx.data())) {
            calc = CRC16(rx.data(), MAX_INFO);
            dec_ok = (calc == stored);
        }
    }
    // TPE: 출력 복사 + SIC tentative (상수 시간)
    const uint32_t ok_mask = 0u - static_cast<uint32_t>(dec_ok);
    const uint32_t fail_mask = ~ok_mask;
    for (int i = 0; i < MAX_INFO; ++i) {
        const uint32_t rxv =
            static_cast<uint32_t>(rx[static_cast<std::size_t>(i)]);
        out[i] = static_cast<uint8_t>(rxv & ok_mask);
        ir_state.sic_tentative[static_cast<std::size_t>(i)] =
            static_cast<uint8_t>(rxv & fail_mask);
    }
    *olen = static_cast<int>(static_cast<uint32_t>(MAX_INFO) & ok_mask);
    ir_state.ok = (ok_mask != 0u);
    ir_state.sic_tentative_valid = fail_mask & 1u;
    fec_secure_wipe_stack(static_cast<void *>(dec.data()), dec.size());
    fec_secure_wipe_stack(static_cast<void *>(rx.data()), rx.size());
    return dec_ok;
#endif
}
int FEC_HARQ::Encode16_IR(const uint8_t *info, int len, uint8_t *syms,
                          uint32_t il_seed, int rv, WorkBuf &wb) noexcept {
    const uint32_t il_eff = il_seed ^ RV_SALT[static_cast<std::size_t>(rv & 3)];
    return Encode_Core(info, len, syms, il_eff, BPS16, NSYM16, wb);
}
bool FEC_HARQ::Decode16_IR(const int16_t *sym_I, const int16_t *sym_Q, int nsym,
                           int nc, int bps, uint32_t il_seed, int rv,
                           IR_RxState &ir_state, uint8_t *out, int *olen,
                           WorkBuf &wb) noexcept {
    if (!out || !olen)
        return false;
    if (ir_state.ok) {
        *olen = MAX_INFO;
        return true;
    }
    if (!sym_I || !sym_Q)
        return false;
    if (nsym != NSYM16 || nc != C16 || bps != BPS16) {
        *olen = 0;
        return false;
    }
    const int64_t llr_slots =
        static_cast<int64_t>(nsym) * static_cast<int64_t>(bps);
    if (llr_slots < static_cast<int64_t>(TOTAL_CODED)) {
        *olen = 0;
        return false;
    }
    std::array<int32_t, k_fwht_buf_sz> &fI = g_fec_dec_fI;
    std::array<int32_t, k_fwht_buf_sz> &fQ = g_fec_dec_fQ;
    std::array<int32_t, k_llr_buf_sz> &llr = g_fec_dec_llr;
    llr.fill(static_cast<int32_t>(0));
    const uint32_t er_en =
        static_cast<uint32_t>(g_ir_erasure_en.load(std::memory_order_relaxed));
    for (int sym = 0; sym < nsym; ++sym) {
        const int base = sym * nc;
        for (int c = 0; c < nc; ++c) {
            const int32_t Ii = static_cast<int32_t>(sym_I[base + c]);
            const int32_t Qi = static_cast<int32_t>(sym_Q[base + c]);
            static constexpr int32_t kErasureMagTh = 20000;
            const int32_t mag =
                fec_ir_fast_abs_i32(Ii) + fec_ir_fast_abs_i32(Qi);
            const uint32_t allow_chip =
                (1u - er_en) | static_cast<uint32_t>(mag <= kErasureMagTh);
            const int32_t mask = -static_cast<int32_t>(allow_chip);
            fI[static_cast<std::size_t>(c)] = Ii & mask;
            fQ[static_cast<std::size_t>(c)] = Qi & mask;
        }
        FWHT(fI.data(), nc);
        FWHT(fQ.data(), nc);
        // DC bin 제거: Walsh bin 0 = Σchips (DC 성분)
        fI[0] = 0;
        fQ[0] = 0;
        Bin_To_LLR(fI.data(), fQ.data(), nc, bps, llr.data());
        for (int b = 0; b < bps; ++b) {
            const int bi = sym * bps + b;
            const uint32_t in_range =
                0u - static_cast<uint32_t>(bi < TOTAL_CODED);
            wb.ru.all_llr[static_cast<std::size_t>(bi)] =
                tpe_clamp_llr(llr[static_cast<std::size_t>(b)]) &
                static_cast<int32_t>(in_range);
        }
    }
    const uint32_t il_eff = il_seed ^ RV_SALT[static_cast<std::size_t>(rv & 3)];
    Bit_Deinterleave(wb.ru.all_llr, TOTAL_CODED, il_eff, wb);
    for (int i = 0; i < TOTAL_CODED; ++i) {
        int32_t &slot = ir_state.llr_accum[static_cast<std::size_t>(i)];
        slot =
            tpe_sat_add_llr(slot, wb.ru.all_llr[static_cast<std::size_t>(i)]);
    }
    ++ir_state.rounds_done;
    for (int i = 0; i < CONV_OUT; ++i) {
        int32_t acc = ir_state.llr_accum[static_cast<std::size_t>(i)];
        for (int r = 1; r < REP; ++r) {
            acc = tpe_sat_add_llr(
                acc,
                ir_state.llr_accum[static_cast<std::size_t>(r * CONV_OUT + i)]);
        }
        wb.ru.all_llr[static_cast<std::size_t>(i)] = acc;
    }
    std::array<uint8_t, static_cast<std::size_t>(CONV_IN)> dec{};
    Viterbi_Decode(wb.ru.all_llr, CONV_OUT, dec.data(), CONV_IN, wb);
    std::array<uint8_t, static_cast<std::size_t>(MAX_INFO + 2)> rx{};
    for (int i = 0; i < INFO_BITS; ++i) {
        const uint32_t bit =
            static_cast<uint32_t>(dec[static_cast<std::size_t>(i)]) & 1u;
        rx[static_cast<std::size_t>(i >> 3)] |=
            static_cast<uint8_t>(bit << static_cast<unsigned>(7 - (i & 7)));
    }
    uint16_t calc = CRC16(rx.data(), MAX_INFO);
    uint16_t stored =
        (static_cast<uint16_t>(rx[static_cast<std::size_t>(MAX_INFO)]) << 8u) |
        static_cast<uint16_t>(rx[static_cast<std::size_t>(MAX_INFO + 1)]);
    bool dec_ok = (calc == stored);
    if (!dec_ok && g_ir_rs_post_en.load(std::memory_order_relaxed) != 0u) {
        if (try_ir_rs_recover_rx8(rx.data())) {
            calc = CRC16(rx.data(), MAX_INFO);
            dec_ok = (calc == stored);
        }
    }
    // TPE: 출력 복사 (상수 시간)
    const uint32_t ok16_mask = 0u - static_cast<uint32_t>(dec_ok);
    for (int i = 0; i < MAX_INFO; ++i) {
        out[i] = static_cast<uint8_t>(
            static_cast<uint32_t>(rx[static_cast<std::size_t>(i)]) & ok16_mask);
    }
    *olen = static_cast<int>(static_cast<uint32_t>(MAX_INFO) & ok16_mask);
    ir_state.ok = (ok16_mask != 0u);
    ir_state.sic_tentative_valid = 0u;
    fec_secure_wipe_stack(static_cast<void *>(dec.data()), dec.size());
    fec_secure_wipe_stack(static_cast<void *>(rx.data()), rx.size());
    return dec_ok;
}
} // namespace ProtectedEngine
