// =============================================================================
// HTS_V400_Dispatcher.cpp — V400 동적 모뎀 디스패처 + FEC-HARQ 통합
//
// [동작 메모]
//  try_decode_: Q채널 HARQ는 harq_Q_[0] (행 포인터)로 Decode_Core_Split에 전달
//               (&harq_Q_[0][0] 금지 — 2차원 배열 타입과 혼동 시 HardFault)
//
#include "HTS_V400_Dispatcher.hpp"
#include "HTS_Holo_LPI.h"
#include "HTS_RF_Metrics.h" // Tick_Adaptive_BPS 용
#include "HTS_Secure_Memory.h"
#include <climits>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cstddef>
// ── AMI/PS-LTE Target Selector ─────────────────────────────
// AMI 200 kcps 환경 (T6 하네스 기본): 32-chip Phase 0/1 coherent + non-coherent 합산
// PS-LTE 1 Mcps 환경: 64-chip Phase 0/1 coherent (기존 유지)
// 빌드 시 정확히 하나만 활성화. 둘 다 미정의 시 PS-LTE(기존 검증 경로) 기본.
// AMI 32-chip 실험(T6 등): 컴파일에 /DHTS_TARGET_AMI 명시.
#if !defined(HTS_TARGET_AMI) && !defined(HTS_TARGET_PSLTE)
#  define HTS_TARGET_PSLTE
#endif
#if defined(HTS_TARGET_AMI) && defined(HTS_TARGET_PSLTE)
#  error "HTS_TARGET_AMI 와 HTS_TARGET_PSLTE 는 동시 정의 불가"
#endif
// 2026-04-18: AMI 32-chip T6 붕괴(43/11200) — 원인 규명 전 컴파일 차단
#if defined(HTS_TARGET_AMI)
#  error "HTS_TARGET_AMI 32-chip 경로는 2026-04-18 T6 붕괴 (43/11200) 로 비활성. 원인 규명 전 빌드 금지."
#endif
// ── Walsh Bank Phase 0 Experimental Path ─────────────────
// 정의 시: Phase 0 detection 을 FWHT Max Row 방식으로 전환
// 정의 미정 시: 기존 경로 (baseline 10810 보존)
// 실험 빌드: /DHTS_PHASE0_WALSH_BANK
// [진단] Barrage30 등 PC 하네스 — ir_chip_I_[0]·harq_round_ 스냅샷 (기본
// 0=비활성)
extern "C" volatile int g_hts_ir_diag_chip0 = 0;
extern "C" volatile int g_hts_ir_diag_feed_idx = -1;
extern "C" void Mock_RF_Synth_Set_Channel(uint8_t channel) noexcept {
    const unsigned ch = static_cast<unsigned>(channel) & 0x7Fu;
    std::printf("[Mock_RF_Synth] ch=%u\n", ch);
}
namespace ProtectedEngine {
// 파일 범위 스크래치: 단일 디스패처 실행 컨텍스트(반이중)·해당 경로 재진입 없음
// 가정. NSYM64 ≥ NSYM16 이므로 Encode 심볼 버퍼 겸용.
alignas(64) static uint8_t g_v400_sym_scratch[FEC_HARQ::NSYM64];
static_assert(sizeof(g_v400_sym_scratch) >= FEC_HARQ::NSYM16,
              "sym scratch must cover NSYM16 encode");
// ── HARQ CCM union: Chase Q 누적 ↔ IR 칩 버퍼 + IR_RxState (SRAM 추가 0) ─
//  Chase: harq_Q[NSYM64][C64] int32 — 기존 g_harq_Q_ccm 와 동일 레이아웃
//  IR:    chip_I/chip_Q int16 + IR_RxState — 모드 전환 시 full_reset_ 에서 전체
//  wipe
struct V400HarqCcmChase {
    int32_t harq_Q[FEC_HARQ::NSYM64][FEC_HARQ::C64];
};
struct V400HarqCcmIr {
    int16_t chip_I[FEC_HARQ::NSYM64][FEC_HARQ::C64];
    int16_t chip_Q[FEC_HARQ::NSYM64][FEC_HARQ::C64];
    FEC_HARQ::IR_RxState ir_state;
};
alignas(64) HTS_CCM_SECTION static union {
    V400HarqCcmChase chase;
    V400HarqCcmIr ir;
} g_harq_ccm_union;
static_assert(sizeof(g_harq_ccm_union) >= sizeof(V400HarqCcmChase),
              "CCM union must hold Chase harq_Q");
static_assert(sizeof(g_harq_ccm_union) >= sizeof(V400HarqCcmIr),
              "CCM union must hold IR chip buffers + IR_RxState");
static_assert(sizeof(int16_t) == 2, "int16_t must be 2 bytes");
static_assert(sizeof(int32_t) == 4, "int32_t must be 4 bytes");
static_assert(sizeof(uint64_t) == 8, "uint64_t required for FWHT energy");
// SIC 1비트 압축: Walsh 칩 = ±amp (I=Q 동일), 부호만 저장
// bit c = 1 → 음(−amp), bit c = 0 → 양(+amp)
// NSYM64 × 8B (기존 int16 I/Q 각 NSYM64×64 대비 약 97% 절감)
alignas(64) static uint64_t g_sic_bits[FEC_HARQ::NSYM64];
static_assert(sizeof(g_sic_bits) == FEC_HARQ::NSYM64 * 8u,
              "SIC bit-pack size mismatch");
static_assert(FEC_HARQ::C64 == 64, "SIC bit-pack requires C64==64");
static constexpr uint32_t popc32(uint32_t x) noexcept {
    x = x - ((x >> 1u) & 0x55555555u);
    x = (x & 0x33333333u) + ((x >> 2u) & 0x33333333u);
    return (((x + (x >> 4u)) & 0x0F0F0F0Fu) * 0x01010101u) >> 24u;
}
static constexpr uint32_t fast_abs(int32_t x) noexcept {
    const uint32_t m = 0u - static_cast<uint32_t>(x < 0);
    return (static_cast<uint32_t>(x) ^ m) + m;
}
/// @brief Walsh-Hadamard 행렬 H_64의 63번 행.
///        w63[j] = (-1)^popcount(j).  ROM 배치.
static constexpr int8_t k_w63[64] = {
    +1, -1, -1, +1, -1, +1, +1, -1,
    -1, +1, +1, -1, +1, -1, -1, +1,
    -1, +1, +1, -1, +1, -1, -1, +1,
    +1, -1, -1, +1, -1, +1, +1, -1,
    -1, +1, +1, -1, +1, -1, -1, +1,
    +1, -1, -1, +1, -1, +1, +1, -1,
    +1, -1, -1, +1, -1, +1, +1, -1,
    -1, +1, +1, -1, +1, -1, -1, +1
};

/// @brief Fast Walsh-Hadamard Transform 64-point (complex I/Q)
/// @details radix-2 butterfly, in-place, 6 stages (Baseline Stage 2 + WBANK)
/// @note Hadamard natural ordering (row idx = sequency)
static inline void fwht_64_complex_inplace_(int32_t* __restrict T_I,
                                            int32_t* __restrict T_Q) noexcept {
    for (int stride = 1; stride < 64; stride <<= 1) {
        for (int i = 0; i < 64; i += (stride << 1)) {
            for (int j = 0; j < stride; ++j) {
                const int a = i + j;
                const int b = a + stride;
                const int32_t tI = T_I[a];
                const int32_t tQ = T_Q[a];
                T_I[a] = tI + T_I[b];
                T_Q[a] = tQ + T_Q[b];
                T_I[b] = tI - T_I[b];
                T_Q[b] = tQ - T_Q[b];
            }
        }
    }
}

/// @brief 64-bin 에너지 배열 argmax — Fix A3 Walsh-row excision
static inline void find_argmax_64(const int64_t* __restrict e,
                                  int* __restrict out_bin,
                                  int64_t* __restrict out_e) noexcept {
    int best_bin = 0;
    int64_t best_e = e[0];
    for (int m = 1; m < 64; ++m) {
        if (e[m] > best_e) {
            best_e = e[m];
            best_bin = m;
        }
    }
    *out_bin = best_bin;
    *out_e = best_e;
}

/// @brief 8-point FWHT (복소 I/Q, in-place, 3 stage)
static inline void fwht_8_complex_inplace_(int32_t* T_I,
                                            int32_t* T_Q) noexcept {
    for (int stride = 1; stride < 8; stride <<= 1) {
        for (int i = 0; i < 8; i += (stride << 1)) {
            for (int j = 0; j < stride; ++j) {
                const int a = i + j;
                const int b = a + stride;
                const int32_t tI = T_I[a];
                const int32_t tQ = T_Q[a];
                T_I[a] = tI + T_I[b];
                T_Q[a] = tQ + T_Q[b];
                T_I[b] = tI - T_I[b];
                T_Q[b] = tQ - T_Q[b];
            }
        }
    }
}

/// @brief 64-chip 블록의 8×8 non-coh 에너지 (각 8칩 FWHT row 7 누적)
/// @note k_w63 자기유사: 8칩 H8[7] 패턴 반복 → CFO 짧은 구간에서 row 7 누적
static inline int64_t energy_8x8_noncoh_row7_(const int16_t* chip_I,
                                                const int16_t* chip_Q) noexcept {
    int64_t e_row7 = 0;
    for (int s = 0; s < 8; ++s) {
        alignas(32) int32_t t_I[8];
        alignas(32) int32_t t_Q[8];
        for (int i = 0; i < 8; ++i) {
            t_I[i] = static_cast<int32_t>(chip_I[s * 8 + i]);
            t_Q[i] = static_cast<int32_t>(chip_Q[s * 8 + i]);
        }
        fwht_8_complex_inplace_(t_I, t_Q);
        e_row7 += static_cast<int64_t>(t_I[7]) * t_I[7] +
                  static_cast<int64_t>(t_Q[7]) * t_Q[7];
    }
    return e_row7;
}

/// @brief Sylvester natural: k_w63 ↔ FWHT row 63 (Stage 2 XOR 기준)
static constexpr uint8_t k_W63_FWHT_ROW_NATURAL = 63u;

#if defined(HTS_PHASE0_WALSH_BANK)
/// @brief k_w63 의 FWHT dominant row (self-calibrated)
/// @details 시작 시 full_reset_ 에서 1회 계산, 이후 read-only
/// @note TX/RX 공통 k_w63 → 동일 row 자동 보장 (ordering 무관)
static uint8_t k_W63_FWHT_ROW = 255u;  // 미초기화 sentinel

/// @brief k_w0 의 FWHT dominant row (항상 0)
/// @details k_w0 = all +1 → FWHT Sylvester/Walsh 모든 ordering 에서 row 0
static constexpr uint8_t k_W0_FWHT_ROW [[maybe_unused]] = 0u;

/// @brief k_w63 의 FWHT dominant row 계산 (1회)
/// @return k_w63 의 FWHT 결과 max row index
static uint8_t calc_kw63_fwht_row_() noexcept {
    int32_t T_I[64];
    int32_t T_Q[64];
    for (int i = 0; i < 64; ++i) {
        T_I[i] = static_cast<int32_t>(k_w63[static_cast<std::size_t>(i)]) * 1000;
        T_Q[i] = 0;
    }
    fwht_64_complex_inplace_(T_I, T_Q);

    int max_row = 0;
    int64_t max_e = 0;
    for (int r = 0; r < 64; ++r) {
        const int64_t e = static_cast<int64_t>(T_I[r]) * T_I[r];
        if (e > max_e) {
            max_e = e;
            max_row = r;
        }
    }
    return static_cast<uint8_t>(max_row);
}

#endif  // HTS_PHASE0_WALSH_BANK

/// Walsh-63 bin dot (ADD/SUB만) — Phase 0 스캔용
static void walsh63_dot_(const int16_t *chip_I, const int16_t *chip_Q,
                         int32_t &dot_I, int32_t &dot_Q) noexcept {
    int32_t dI = 0;
    int32_t dQ = 0;
    for (int j = 0; j < 64; ++j) {
        const int32_t sI = static_cast<int32_t>(chip_I[j]);
        const int32_t sQ = static_cast<int32_t>(chip_Q[j]);
        if (k_w63[static_cast<std::size_t>(j)] > 0) {
            dI += sI;
            dQ += sQ;
        } else {
            dI -= sI;
            dQ -= sQ;
        }
    }
    dot_I = dI;
    dot_Q = dQ;
}
static void fwht_raw(int32_t *d, int n) noexcept {
    if (n == 64) {
        for (int i = 0; i < 64; i += 2) {
            int32_t u = d[i], v = d[i + 1];
            d[i] = u + v;
            d[i + 1] = u - v;
        }
        for (int i = 0; i < 64; i += 4) {
            int32_t u = d[i], v = d[i + 2];
            d[i] = u + v;
            d[i + 2] = u - v;
            u = d[i + 1];
            v = d[i + 3];
            d[i + 1] = u + v;
            d[i + 3] = u - v;
        }
        for (int i = 0; i < 64; i += 8) {
            for (int k = 0; k < 4; ++k) {
                int32_t u = d[i + k], v = d[i + 4 + k];
                d[i + k] = u + v;
                d[i + 4 + k] = u - v;
            }
        }
        for (int i = 0; i < 64; i += 16) {
            for (int k = 0; k < 8; ++k) {
                int32_t u = d[i + k], v = d[i + 8 + k];
                d[i + k] = u + v;
                d[i + 8 + k] = u - v;
            }
        }
        for (int i = 0; i < 64; i += 32) {
            for (int k = 0; k < 16; ++k) {
                int32_t u = d[i + k], v = d[i + 16 + k];
                d[i + k] = u + v;
                d[i + 16 + k] = u - v;
            }
        }
        for (int k = 0; k < 32; ++k) {
            int32_t u = d[k], v = d[k + 32];
            d[k] = u + v;
            d[k + 32] = u - v;
        }
        return;
    }
    if (n == 16) {
        for (int i = 0; i < 16; i += 2) {
            int32_t u = d[i], v = d[i + 1];
            d[i] = u + v;
            d[i + 1] = u - v;
        }
        for (int i = 0; i < 16; i += 4) {
            int32_t u = d[i], v = d[i + 2];
            d[i] = u + v;
            d[i + 2] = u - v;
            u = d[i + 1];
            v = d[i + 3];
            d[i + 1] = u + v;
            d[i + 3] = u - v;
        }
        for (int i = 0; i < 16; i += 8) {
            for (int k = 0; k < 4; ++k) {
                int32_t u = d[i + k], v = d[i + 4 + k];
                d[i + k] = u + v;
                d[i + 4 + k] = u - v;
            }
        }
        for (int k = 0; k < 8; ++k) {
            int32_t u = d[k], v = d[k + 8];
            d[k] = u + v;
            d[k + 8] = u - v;
        }
        return;
    }
    for (int len = 1; len < n; len <<= 1) {
        for (int i = 0; i < n; i += 2 * len) {
            for (int j = 0; j < len; ++j) {
                int32_t u = d[i + j], v = d[i + len + j];
                d[i + j] = u + v;
                d[i + len + j] = u - v;
            }
        }
    }
}
//  스택 512B(sI[64]+sQ[64]) 제거, dec_wI_/dec_wQ_ 멤버 재활용
alignas(64) static const int16_t k_walsh_dummy_iq_[64] = {};
// ── Walsh 피크 탐색: I²+Q² 에너지 (위상 불변, RF 90° 회전에도 신호 소멸 없음) ──
//  best_e/second_e: 검출된 빈의 I²+Q² 그대로 산출
HTS_V400_Dispatcher::SymDecResult
HTS_V400_Dispatcher::walsh_dec_full_(const int16_t *I, const int16_t *Q, int n,
                                     bool cap_search_to_bps) noexcept {
    const uint32_t p_ok =
        static_cast<uint32_t>((I != nullptr) & (Q != nullptr) &
                              static_cast<uint32_t>((n == 16) | (n == 64)));
    const int n_eff = ((n == 16) | (n == 64)) ? n : 64;
    const int16_t *srcI = (p_ok != 0u) ? I : k_walsh_dummy_iq_;
    const int16_t *srcQ = (p_ok != 0u) ? Q : k_walsh_dummy_iq_;
    for (int i = 0; i < n_eff; ++i) {
        dec_wI_[i] = srcI[i];
        dec_wQ_[i] = srcQ[i];
    }
    fwht_raw(dec_wI_, n_eff);
    fwht_raw(dec_wQ_, n_eff);
    int search = n_eff;
    if (cap_search_to_bps && n_eff == 64) {
        const int bps = cur_bps64_;
        const int valid = 1 << bps;
        search = (valid < n_eff) ? valid : n_eff;
    }
    // ── 피크 탐색: I²+Q² 에너지 (branchless) ──
    int32_t best_c = INT32_MIN;
    int32_t second_c = INT32_MIN;
    uint8_t dec = 0xFFu;
    uint8_t dec2 = 0xFFu;
    for (int m = 0; m < search; ++m) {
        const int64_t eI = static_cast<int64_t>(dec_wI_[m]) * dec_wI_[m];
        const int64_t eQ = static_cast<int64_t>(dec_wQ_[m]) * dec_wQ_[m];
        const int32_t c = static_cast<int32_t>((eI + eQ) >> 16);
        // 0/1 플래그 → 0x00000000/0xFFFFFFFF 풀 마스크 변환
        const uint32_t m_gt_best = 0u - static_cast<uint32_t>(c > best_c);
        const uint32_t m_gt_sec =
            0u - ((~m_gt_best & 1u) & static_cast<uint32_t>(c > second_c));
        const uint32_t m_none = ~(m_gt_best | m_gt_sec);
        // second 갱신: best가 밀려오거나 sec 직접 갱신
        second_c =
            static_cast<int32_t>((static_cast<uint32_t>(best_c) & m_gt_best) |
                                 (static_cast<uint32_t>(c) & m_gt_sec) |
                                 (static_cast<uint32_t>(second_c) & m_none));
        dec2 = static_cast<uint8_t>((static_cast<uint32_t>(dec) & m_gt_best) |
                                    (static_cast<uint32_t>(m) & m_gt_sec) |
                                    (static_cast<uint32_t>(dec2) & m_none));
        // best 갱신
        best_c =
            static_cast<int32_t>((static_cast<uint32_t>(c) & m_gt_best) |
                                 (static_cast<uint32_t>(best_c) & ~m_gt_best));
        dec = static_cast<uint8_t>((static_cast<uint32_t>(m) & m_gt_best) |
                                   (static_cast<uint32_t>(dec) & ~m_gt_best));
    }
    const int32_t mk = -static_cast<int32_t>(p_ok);
    const int8_t sym_raw = (best_c == INT32_MIN) ? static_cast<int8_t>(-1)
                                                 : static_cast<int8_t>(dec);
    const int8_t sym_out =
        static_cast<int8_t>((static_cast<int32_t>(sym_raw) & mk) |
                            (static_cast<int32_t>(-1) & ~mk));
    // 검출된 빈의 에너지 산출
    const uint32_t bi = static_cast<uint32_t>(dec) & 63u;
    const uint32_t si = static_cast<uint32_t>(dec2) & 63u;
    const uint64_t be64 =
        static_cast<uint64_t>(static_cast<int64_t>(dec_wI_[bi]) * dec_wI_[bi] +
                              static_cast<int64_t>(dec_wQ_[bi]) * dec_wQ_[bi]);
    const uint64_t se64 =
        static_cast<uint64_t>(static_cast<int64_t>(dec_wI_[si]) * dec_wI_[si] +
                              static_cast<int64_t>(dec_wQ_[si]) * dec_wQ_[si]);
    uint32_t be = static_cast<uint32_t>(be64 >> 16u);
    uint32_t se = static_cast<uint32_t>(se64 >> 16u);
    be &= static_cast<uint32_t>(mk);
    se &= static_cast<uint32_t>(mk);
    return {sym_out, be, se};
}
HTS_V400_Dispatcher::SymDecResult
HTS_V400_Dispatcher::walsh_dec_dot_proj_full_(const int16_t *I, const int16_t *Q,
                                              bool cap_search_to_bps) noexcept {
    const uint32_t p_ok =
        static_cast<uint32_t>((I != nullptr) & (Q != nullptr));
    const int16_t *srcI = (p_ok != 0u) ? I : k_walsh_dummy_iq_;
    const int16_t *srcQ = (p_ok != 0u) ? Q : k_walsh_dummy_iq_;
    for (int i = 0; i < 64; ++i) {
        dec_wI_[i] = srcI[i];
        dec_wQ_[i] = srcQ[i];
    }
    fwht_raw(dec_wI_, 64);
    fwht_raw(dec_wQ_, 64);
    int search = 64;
    if (cap_search_to_bps) {
        const int bps = cur_bps64_;
        const int valid = 1 << bps;
        search = (valid < 64) ? valid : 64;
    }
    // Non-coherent 전용: est_I_/est_Q_/est_count_ 기반 coherent 분기 없음
    // (est_* 는 derot_shift 등 다른 경로 전용). CFO 시 projection 왜곡 방지.
    // 피크 탐색은 walsh_dec_full_ 과 동일 — I²+Q² 에너지 + branchless + second_e.
    int32_t best_c = INT32_MIN;
    int32_t second_c = INT32_MIN;
    uint8_t dec = 0xFFu;
    uint8_t dec2 = 0xFFu;
    for (int m = 0; m < search; ++m) {
        const int64_t eI = static_cast<int64_t>(dec_wI_[m]) * dec_wI_[m];
        const int64_t eQ = static_cast<int64_t>(dec_wQ_[m]) * dec_wQ_[m];
        const int32_t c = static_cast<int32_t>((eI + eQ) >> 16);
        const uint32_t m_gt_best = 0u - static_cast<uint32_t>(c > best_c);
        const uint32_t m_gt_sec =
            0u - ((~m_gt_best & 1u) & static_cast<uint32_t>(c > second_c));
        const uint32_t m_none = ~(m_gt_best | m_gt_sec);
        second_c =
            static_cast<int32_t>((static_cast<uint32_t>(best_c) & m_gt_best) |
                                 (static_cast<uint32_t>(c) & m_gt_sec) |
                                 (static_cast<uint32_t>(second_c) & m_none));
        dec2 = static_cast<uint8_t>((static_cast<uint32_t>(dec) & m_gt_best) |
                                    (static_cast<uint32_t>(m) & m_gt_sec) |
                                    (static_cast<uint32_t>(dec2) & m_none));
        best_c =
            static_cast<int32_t>((static_cast<uint32_t>(c) & m_gt_best) |
                                 (static_cast<uint32_t>(best_c) & ~m_gt_best));
        dec = static_cast<uint8_t>((static_cast<uint32_t>(m) & m_gt_best) |
                                   (static_cast<uint32_t>(dec) & ~m_gt_best));
    }
    const int32_t mk = -static_cast<int32_t>(p_ok);
    const int8_t sym_raw = (best_c == INT32_MIN) ? static_cast<int8_t>(-1)
                                                 : static_cast<int8_t>(dec);
    const int8_t sym_out =
        static_cast<int8_t>((static_cast<int32_t>(sym_raw) & mk) |
                            (static_cast<int32_t>(-1) & ~mk));
    const uint32_t bi = static_cast<uint32_t>(dec) & 63u;
    const uint32_t si = static_cast<uint32_t>(dec2) & 63u;
    const uint64_t be64 =
        static_cast<uint64_t>(static_cast<int64_t>(dec_wI_[bi]) * dec_wI_[bi] +
                              static_cast<int64_t>(dec_wQ_[bi]) * dec_wQ_[bi]);
    const uint64_t se64 =
        static_cast<uint64_t>(static_cast<int64_t>(dec_wI_[si]) * dec_wI_[si] +
                              static_cast<int64_t>(dec_wQ_[si]) * dec_wQ_[si]);
    uint32_t be = static_cast<uint32_t>(be64 >> 16u);
    uint32_t se = static_cast<uint32_t>(se64 >> 16u);
    be &= static_cast<uint32_t>(mk);
    se &= static_cast<uint32_t>(mk);
    return {sym_out, be, se};
}
// ── I=Q 동일 모드 (재밍 방어) ──
// ── [적응형 I/Q] I/Q 독립 디코딩 ──────────────────────────
//  I 채널과 Q 채널을 분리하여 각각 FWHT 수행
//  한 칩 구간에서 2개 심볼 획득 → 처리량 2배
//  각 채널의 에너지는 I² 또는 Q² 단독 (합산하지 않음)
//  → I=Q 동일 대비 −3dB, 평시(NF<10dB) 충분
HTS_V400_Dispatcher::SymDecResultSplit
HTS_V400_Dispatcher::walsh_dec_split_(const int16_t *I, const int16_t *Q,
                                      int n) noexcept {
    const uint32_t p_ok =
        static_cast<uint32_t>((I != nullptr) & (Q != nullptr) &
                              static_cast<uint32_t>((n == 16) | (n == 64)));
    const int n_eff = ((n == 16) | (n == 64)) ? n : 64;
    const int16_t *srcI = (p_ok != 0u) ? I : k_walsh_dummy_iq_;
    const int16_t *srcQ = (p_ok != 0u) ? Q : k_walsh_dummy_iq_;
    for (int i = 0; i < n_eff; ++i) {
        dec_wI_[i] = srcI[i];
    }
    fwht_raw(dec_wI_, n_eff);
    for (int i = 0; i < n_eff; ++i) {
        dec_wQ_[i] = srcQ[i];
    }
    fwht_raw(dec_wQ_, n_eff);
    const int bps = (n_eff == 64) ? cur_bps64_ : 4;
    const int valid = 1 << bps;
    const int search = (valid < n_eff) ? valid : n_eff;
    // I 채널 최대 에너지 빈 탐색
    uint64_t bestI = 0u, secI = 0u;
    uint8_t decI = 0xFFu;
    for (int m = 0; m < search; ++m) {
        const uint64_t e = static_cast<uint64_t>(
            static_cast<int64_t>(dec_wI_[m]) * dec_wI_[m]);
        const uint32_t c_gt_best = static_cast<uint32_t>(e > bestI);
        const uint32_t c_gt_sec =
            static_cast<uint32_t>(e <= bestI) & static_cast<uint32_t>(e > secI);
        secI = secI * (1ull - static_cast<uint64_t>(c_gt_best)) *
                   (1ull - static_cast<uint64_t>(c_gt_sec)) +
               bestI * static_cast<uint64_t>(c_gt_best) +
               e * static_cast<uint64_t>(c_gt_sec);
        bestI = bestI * (1ull - static_cast<uint64_t>(c_gt_best)) +
                e * static_cast<uint64_t>(c_gt_best);
        decI = static_cast<uint8_t>(static_cast<uint32_t>(m) * c_gt_best +
                                    static_cast<uint32_t>(decI) *
                                        (1u - c_gt_best));
    }
    uint64_t bestQ = 0u, secQ = 0u;
    uint8_t decQ = 0xFFu;
    for (int m = 0; m < search; ++m) {
        const uint64_t e = static_cast<uint64_t>(
            static_cast<int64_t>(dec_wQ_[m]) * dec_wQ_[m]);
        const uint32_t c_gt_best = static_cast<uint32_t>(e > bestQ);
        const uint32_t c_gt_sec =
            static_cast<uint32_t>(e <= bestQ) & static_cast<uint32_t>(e > secQ);
        secQ = secQ * (1ull - static_cast<uint64_t>(c_gt_best)) *
                   (1ull - static_cast<uint64_t>(c_gt_sec)) +
               bestQ * static_cast<uint64_t>(c_gt_best) +
               e * static_cast<uint64_t>(c_gt_sec);
        bestQ = bestQ * (1ull - static_cast<uint64_t>(c_gt_best)) +
                e * static_cast<uint64_t>(c_gt_best);
        decQ = static_cast<uint8_t>(static_cast<uint32_t>(m) * c_gt_best +
                                    static_cast<uint32_t>(decQ) *
                                        (1u - c_gt_best));
    }
    const int32_t mk = -static_cast<int32_t>(p_ok);
    const int8_t symI_raw =
        (bestI == 0u) ? static_cast<int8_t>(-1) : static_cast<int8_t>(decI);
    const int8_t symQ_raw =
        (bestQ == 0u) ? static_cast<int8_t>(-1) : static_cast<int8_t>(decQ);
    uint32_t bIe = static_cast<uint32_t>(bestI >> 16u);
    uint32_t sIe = static_cast<uint32_t>(secI >> 16u);
    uint32_t bQe = static_cast<uint32_t>(bestQ >> 16u);
    uint32_t sQe = static_cast<uint32_t>(secQ >> 16u);
    bIe &= static_cast<uint32_t>(mk);
    sIe &= static_cast<uint32_t>(mk);
    bQe &= static_cast<uint32_t>(mk);
    sQe &= static_cast<uint32_t>(mk);
    return {static_cast<int8_t>((static_cast<int32_t>(symI_raw) & mk) |
                                (static_cast<int32_t>(-1) & ~mk)),
            static_cast<int8_t>((static_cast<int32_t>(symQ_raw) & mk) |
                                (static_cast<int32_t>(-1) & ~mk)),
            bIe,
            sIe,
            bQe,
            sQe};
}
static void walsh_enc(uint8_t sym, int n, int16_t amp, int16_t *oI,
                      int16_t *oQ) noexcept {
    const int32_t ampi = static_cast<int32_t>(amp);
    for (int j = 0; j < n; ++j) {
        const uint32_t p =
            popc32(static_cast<uint32_t>(sym) & static_cast<uint32_t>(j)) & 1u;
        const int16_t ch =
            static_cast<int16_t>(ampi * (1 - 2 * static_cast<int32_t>(p)));
        oI[j] = ch;
        oQ[j] = ch;
    }
}
// ── [적응형 I/Q] I/Q 독립 모드 (평시 2배 처리량) ──
//  sym_I → I 채널, sym_Q → Q 채널에 독립 Walsh 인코딩
//  처리량 2배: 동일 칩 수로 2개 심볼 전송
static void walsh_enc_split(uint8_t sym_I, uint8_t sym_Q, int n, int16_t amp,
                            int16_t *oI, int16_t *oQ) noexcept {
    const int32_t ampi = static_cast<int32_t>(amp);
    for (int j = 0; j < n; ++j) {
        const uint32_t pI =
            popc32(static_cast<uint32_t>(sym_I) & static_cast<uint32_t>(j)) &
            1u;
        const uint32_t pQ =
            popc32(static_cast<uint32_t>(sym_Q) & static_cast<uint32_t>(j)) &
            1u;
        oI[j] = static_cast<int16_t>(ampi * (1 - 2 * static_cast<int32_t>(pI)));
        oQ[j] = static_cast<int16_t>(ampi * (1 - 2 * static_cast<int32_t>(pQ)));
    }
}
/// 인접 교환 정렬 — N=64, 고정 2016 인접 스왑(브랜치리스 트립).
static constexpr int kMagSort64N = 64;
static_assert(static_cast<std::size_t>(kMagSort64N) *
                      static_cast<std::size_t>(kMagSort64N - 1) / 2u ==
                  2016u,
              "mag sort trip count");
static void sort_u32_ct_adjacent_64_dispatch(uint32_t *a) noexcept {
    if (a == nullptr) {
        return;
    }
    for (int pass = 0; pass < kMagSort64N - 1; ++pass) {
        const int imax = kMagSort64N - 1 - pass;
        for (int i = 0; i < imax; ++i) {
            const uint32_t x = a[i];
            const uint32_t y = a[i + 1];
            const uint32_t gt = static_cast<uint32_t>(x > y);
            const uint32_t m = 0u - gt;
            a[i] = (x & ~m) | (y & m);
            a[i + 1] = (y & ~m) | (x & m);
        }
    }
}
static void fill_u32_pad_max_(uint32_t *work, const uint32_t *src,
                              int nc) noexcept {
    for (int i = 0; i < nc; ++i) {
        work[static_cast<std::size_t>(i)] = src[static_cast<std::size_t>(i)];
    }
    for (int i = nc; i < kMagSort64N; ++i) {
        work[static_cast<std::size_t>(i)] = 0xFFFFFFFFu;
    }
}
static inline int16_t ssat16_dispatch_(int32_t v) noexcept {
#if defined(__GNUC__) && defined(__ARM_ARCH) && (__ARM_ARCH >= 6)
    return static_cast<int16_t>(__builtin_arm_ssat(v, 16));
#else
    const uint32_t ov_hi = static_cast<uint32_t>(v > 32767);
    const uint32_t ov_lo = static_cast<uint32_t>(v < -32768);
    const int32_t msk = -static_cast<int32_t>(ov_hi | ov_lo);
    const int32_t repl = (32767 & -static_cast<int32_t>(ov_hi)) |
                         (-32768 & -static_cast<int32_t>(ov_lo));
    return static_cast<int16_t>((v & ~msk) | (repl & msk));
#endif
}
// [REMOVED Step1] I/Q 클립 경로 제거 — Gaussian noise 에 무효 확정 (T6 96.5% 유지).
//  T6 진단 시 outlier 발생 비율 0.003%, abs_sum=0. 완전 제거.

static constexpr uint32_t k_BH_NOISE_FLOOR = 50u; // baseline 하한 (무간섭 판별)
static constexpr uint32_t k_BH_SATURATION =
    8000u; // baseline 상한 (ADC 포화 방어)
void HTS_V400_Dispatcher::blackhole_(int16_t *I, int16_t *Q, int nc) noexcept {
    if (I == nullptr || Q == nullptr || nc <= 0)
        return;
    if (nc > 64)
        return;
    for (int i = 0; i < nc; ++i) {
        scratch_mag_[i] = 0u;
        scratch_sort_[i] = 0u;
    }
    for (int i = 0; i < nc; ++i) {
        scratch_mag_[i] = fast_abs(static_cast<int32_t>(I[i])) +
                          fast_abs(static_cast<int32_t>(Q[i]));
        scratch_sort_[i] = scratch_mag_[i];
    }
    int q25 = nc >> 2;
    if (q25 < 1)
        q25 = 1;
    fill_u32_pad_max_(scratch_sort_, scratch_mag_, nc);
    sort_u32_ct_adjacent_64_dispatch(scratch_sort_);
    uint32_t bl = scratch_sort_[static_cast<std::size_t>(q25 - 1)];
    if (bl < 1u)
        bl = 1u;
    if (bl < k_BH_NOISE_FLOOR || bl > k_BH_SATURATION)
        return;
    uint32_t punch = bl << 3u;
    for (int i = 0; i < nc; ++i) {
        const uint32_t kill = static_cast<uint32_t>(scratch_mag_[i] > punch);
        const int32_t km = -static_cast<int32_t>(kill);
        I[i] = static_cast<int16_t>(static_cast<int32_t>(I[i]) & ~km);
        Q[i] = static_cast<int16_t>(static_cast<int32_t>(Q[i]) & ~km);
    }
}
// [REMOVED Step2] cw_cancel_64_ — T6 에서 0.003% 효과, abs_sum=56 (~1 LSB).
//  1.87M 호출 중 56회만 I[0] 변경. SNR 기여 측정 불가 수준. 경로 삭제.

// =====================================================================
//  생성자 / 소멸자
// =====================================================================
HTS_V400_Dispatcher::HTS_V400_Dispatcher() noexcept
    : phase_(RxPhase::WAIT_SYNC), cur_mode_(PayloadMode::UNKNOWN),
      active_video_(PayloadMode::VIDEO_1), seed_(0x12345678u), tx_seq_(0u),
      rx_seq_(0u), on_pkt_(nullptr), on_ctrl_(nullptr), buf_I_{}, buf_Q_{},
      buf_idx_(0), pre_phase_(0), pre_reps_(1), pre_boost_(1), hdr_syms_{},
      hdr_count_(0), hdr_fail_(0), pay_cps_(0), pay_total_(0), pay_recv_(0),
      harq_round_(0), max_harq_(0), vid_fail_(0), vid_succ_(0), v1_rx_{},
      v1_idx_(0), rx_{}, sym_idx_(0), harq_inited_(false), retx_ready_(false),
      harq_Q_(g_harq_ccm_union.chase.harq_Q),
      ir_chip_I_(&g_harq_ccm_union.ir.chip_I[0][0]),
      ir_chip_Q_(&g_harq_ccm_union.ir.chip_Q[0][0]),
      ir_state_(&g_harq_ccm_union.ir.ir_state), sic_ir_enabled_(false),
      sic_expect_valid_(false), sic_walsh_amp_(300),
      wb_{} // wb 유니온 (반이중 TDM)
      ,
      orig_acc_{}, orig_I_{}, orig_Q_{},
      holo_lpi_en_(HTS_Holo_LPI::SECURE_FALSE),
      holo_lpi_seed_{}, holo_lpi_mix_q8_(128), holo_lpi_scalars_{},
      holo_lpi_rx_slot_(0u), holo_lpi_rx_chip_idx_(0u),
      holo_lpi_rx_scalars_seq_(0xFFFFFFFFu), dec_wI_{}, dec_wQ_{}
{
#if defined(HTS_DIAG_PRINTF)
    // PROMPT 38: AntiJamEngine 본문은 Step3 제거 — 플래그·대체 경로만 실측
    std::printf(
        "[AJC-STATE] anti_jam_engine_present=0 reason=removed_step3 "
        "notch_path=n/a iq_mode=%d ir_mode=%d lpi_on=%d "
        "p_metrics_bound=%d nf_q16_hi=-1(tx_ctor) tx_amp=%d\n",
        static_cast<int>(iq_mode_),
        ir_mode_ ? 1 : 0,
        (holo_lpi_en_ == HTS_Holo_LPI::SECURE_TRUE) ? 1 : 0,
        (p_metrics_ != nullptr) ? 1 : 0,
        static_cast<int>(tx_amp_));
#endif
}
HTS_V400_Dispatcher::~HTS_V400_Dispatcher() noexcept {
    // [CRIT] sizeof(*this) 통째 wipe 금지 — 멤버 역순 소멸 전 다른 서브객체
    // 손상. 순서: CCM → full_reset_(HARQ/work 버퍼) → 칩/워킹 스크래치 →
    // 시퀀스/시드 → 콜백/메트릭 무효화
    SecureMemory::secureWipe(static_cast<void *>(&g_harq_ccm_union),
                             sizeof(g_harq_ccm_union));
    SecureMemory::secureWipe(static_cast<void *>(g_sic_bits),
                             sizeof(g_sic_bits));
    full_reset_();
    SecureMemory::secureWipe(static_cast<void *>(buf_I_), sizeof(buf_I_));
    SecureMemory::secureWipe(static_cast<void *>(buf_Q_), sizeof(buf_Q_));
    SecureMemory::secureWipe(static_cast<void *>(dec_wI_), sizeof(dec_wI_));
    SecureMemory::secureWipe(static_cast<void *>(dec_wQ_), sizeof(dec_wQ_));
    SecureMemory::secureWipe(static_cast<void *>(scratch_mag_),
                             sizeof(scratch_mag_));
    SecureMemory::secureWipe(static_cast<void *>(scratch_sort_),
                             sizeof(scratch_sort_));
    holo_lpi_en_ = HTS_Holo_LPI::SECURE_FALSE;
    holo_lpi_rx_slot_ = 0u;
    holo_lpi_rx_chip_idx_ = 0u;
    holo_lpi_rx_scalars_seq_ = 0xFFFFFFFFu;
    SecureMemory::secureWipe(static_cast<void *>(holo_lpi_seed_),
                             sizeof(holo_lpi_seed_));
    SecureMemory::secureWipe(static_cast<void *>(holo_lpi_scalars_),
                             sizeof(holo_lpi_scalars_));
    SecureMemory::secureWipe(static_cast<void *>(&seed_), sizeof(seed_));
    SecureMemory::secureWipe(static_cast<void *>(&tx_seq_), sizeof(tx_seq_));
    SecureMemory::secureWipe(static_cast<void *>(&rx_seq_), sizeof(rx_seq_));
    SecureMemory::secureWipe(static_cast<void *>(hdr_syms_), sizeof(hdr_syms_));
    on_pkt_ = nullptr;
    on_ctrl_ = nullptr;
    p_metrics_ = nullptr;
}
void HTS_V400_Dispatcher::Set_Seed(uint32_t s) noexcept { seed_ = s; }

void HTS_V400_Dispatcher::Enable_Holo_LPI(
    const uint32_t lpi_seed[4]) noexcept {
    if (lpi_seed == nullptr) {
        Disable_Holo_LPI();
        return;
    }
    holo_lpi_seed_[0] = lpi_seed[0];
    holo_lpi_seed_[1] = lpi_seed[1];
    holo_lpi_seed_[2] = lpi_seed[2];
    holo_lpi_seed_[3] = lpi_seed[3];
    holo_lpi_rx_slot_ = 0u;
    holo_lpi_rx_chip_idx_ = 0u;
    holo_lpi_rx_scalars_seq_ = 0xFFFFFFFFu;
    holo_lpi_en_ = HTS_Holo_LPI::SECURE_TRUE;
}

void HTS_V400_Dispatcher::Disable_Holo_LPI() noexcept {
    holo_lpi_en_ = HTS_Holo_LPI::SECURE_FALSE;
    holo_lpi_rx_slot_ = 0u;
    holo_lpi_rx_chip_idx_ = 0u;
    holo_lpi_rx_scalars_seq_ = 0xFFFFFFFFu;
    SecureMemory::secureWipe(static_cast<void *>(holo_lpi_seed_),
                             sizeof(holo_lpi_seed_));
    SecureMemory::secureWipe(static_cast<void *>(holo_lpi_scalars_),
                             sizeof(holo_lpi_scalars_));
}

void HTS_V400_Dispatcher::apply_holo_lpi_inverse_rx_chip_(
    int16_t& chip_I, int16_t& chip_Q,
    uint32_t scalar_time_slot) noexcept {
    if (holo_lpi_en_ != HTS_Holo_LPI::SECURE_TRUE) {
        return;
    }
    holo_lpi_rx_slot_ = scalar_time_slot;
    if (holo_lpi_rx_scalars_seq_ != scalar_time_slot) {
        if (HTS_Holo_LPI::Generate_Scalars(holo_lpi_seed_, scalar_time_slot,
                                           holo_lpi_mix_q8_,
                                           holo_lpi_scalars_) !=
            HTS_Holo_LPI::SECURE_TRUE) {
            return;
        }
        holo_lpi_rx_scalars_seq_ = scalar_time_slot;
    }
    const int16_t sc =
        holo_lpi_scalars_[static_cast<std::size_t>(holo_lpi_rx_chip_idx_) &
                           63u];
    ++holo_lpi_rx_chip_idx_;
    if (sc == 0) {
        return;
    }
    const int64_t i64 =
        (static_cast<int64_t>(chip_I) << 13) / static_cast<int64_t>(sc);
    const int64_t q64 =
        (static_cast<int64_t>(chip_Q) << 13) / static_cast<int64_t>(sc);
    const int64_t hi = 32767;
    const int64_t lo = -32768;
    const int64_t ci = (i64 > hi) ? hi : ((i64 < lo) ? lo : i64);
    const int64_t cq = (q64 > hi) ? hi : ((q64 < lo) ? lo : q64);
    chip_I = static_cast<int16_t>(ci);
    chip_Q = static_cast<int16_t>(cq);
}
void HTS_V400_Dispatcher::Set_Packet_Callback(PacketCB cb) noexcept {
    on_pkt_ = cb;
}
void HTS_V400_Dispatcher::Set_Control_Callback(ControlCB cb) noexcept {
    on_ctrl_ = cb;
}
RxPhase HTS_V400_Dispatcher::Get_Phase() const noexcept { return phase_; }
PayloadMode HTS_V400_Dispatcher::Get_Mode() const noexcept { return cur_mode_; }
int HTS_V400_Dispatcher::Get_Video_Fail_Count() const noexcept {
    return vid_fail_;
}
int HTS_V400_Dispatcher::Get_Current_BPS64() const noexcept {
    return cur_bps64_;
}
IQ_Mode HTS_V400_Dispatcher::Get_IQ_Mode() const noexcept { return iq_mode_; }
void HTS_V400_Dispatcher::Set_IR_Mode(bool enable) noexcept {
    if (ir_mode_ != enable) {
        ir_mode_ = enable;
        full_reset_();
    }
}
bool HTS_V400_Dispatcher::Get_IR_Mode() const noexcept { return ir_mode_; }
void HTS_V400_Dispatcher::Set_IR_SIC_Enabled(bool enable) noexcept {
    if (sic_ir_enabled_ == enable) {
        return;
    }
    sic_ir_enabled_ = enable;
    sic_expect_valid_ = false;
    std::memset(g_sic_bits, 0, sizeof(g_sic_bits));
}
bool HTS_V400_Dispatcher::Get_IR_SIC_Enabled() const noexcept {
    return sic_ir_enabled_;
}
void HTS_V400_Dispatcher::Set_SIC_Walsh_Amp(int16_t amp) noexcept {
    sic_walsh_amp_ = amp;
}
void HTS_V400_Dispatcher::Set_Tx_Amp(int16_t amp) noexcept {
    if (amp < 64) {
        amp = 64;
    }
    if (amp > 1024) {
        amp = 1024;
    }
    tx_amp_ = amp;
}
HTS_TPC_Controller& HTS_V400_Dispatcher::Get_TPC() noexcept { return tpc_; }
const HTS_TPC_Controller& HTS_V400_Dispatcher::Get_TPC() const noexcept {
    return tpc_;
}
void HTS_V400_Dispatcher::tpc_rx_feedback_after_decode_(
    DecodedPacket& pkt) noexcept {
    // ── P0-FIX-005 Stage 0: TPC feedback 임시 격리 ──
    //  구(舊): Embed_Feedback 이 pkt.data[7] 상위 2 bit 를 RSSI 로 덮어써
    //   info payload 의 bit 56, 57 을 파괴 (DIAG-CORRUPT 실증).
    //   Extract_Feedback 은 TX 가 사전 embed 하지 않은 payload 를 오독하여
    //   잘못된 TPC state 로 갱신.
    //  신(新): 함수 전체를 no-op 화. TPC 기능은 Stage 2 에서 헤더 기반으로
    //   재구현 예정 (info payload 무결성 보장).
    (void)pkt;
}
void HTS_V400_Dispatcher::fill_sic_expected_64_() noexcept {
    sic_expect_valid_ = false;
    if (!sic_ir_enabled_ || ir_state_ == nullptr) {
        return;
    }
    if (ir_state_->sic_tentative_valid == 0u) {
        return;
    }
    std::memset(g_sic_bits, 0, sizeof(g_sic_bits));
    uint8_t *const syms = g_v400_sym_scratch;
    const int rv_fb = (ir_rv_ + 3) & 3;
    const uint32_t il = seed_ ^ (rx_seq_ * 0xA5A5A5A5u);
    const int enc_n =
        FEC_HARQ::Encode64_IR(ir_state_->sic_tentative, FEC_HARQ::MAX_INFO,
                              syms, il, cur_bps64_, rv_fb, wb_);
    if (enc_n <= 0) {
        SecureMemory::secureWipe(static_cast<void *>(syms),
                                 sizeof(g_v400_sym_scratch));
        return;
    }
    // Walsh 부호 비트맵 — walsh_enc()와 동일: chip = amp×(1−2×(popc(sym&j)&1))
    for (int s = 0; s < enc_n; ++s) {
        const uint32_t sym_u = static_cast<uint32_t>(
            syms[static_cast<std::size_t>(s)]);
        uint64_t bits = 0u;
        for (int c = 0; c < 64; ++c) {
            const uint32_t neg =
                popc32(sym_u & static_cast<uint32_t>(c)) & 1u;
            bits |= (static_cast<uint64_t>(neg)
                     << static_cast<uint32_t>(c));
        }
        g_sic_bits[static_cast<std::size_t>(s)] = bits;
    }
    sic_expect_valid_ = true;
    SecureMemory::secureWipe(static_cast<void *>(syms),
                             sizeof(g_v400_sym_scratch));
}
void HTS_V400_Dispatcher::Update_Adaptive_BPS(uint32_t nf) noexcept {
    // HTS_RF_Metrics + HTS_Adaptive_BPS_Controller 경로가 연결된 경우
    // current_bps의 단일 진실은 컨트롤러(히스테리시스)이다.
    // bps_from_nf(nf)로 cur_bps64_를 즉시 덮어쓰면 Tick_Adaptive_BPS()가
    // 방금 올린 BPS를 한 프레임 만에 되돌리는 이중 경로 충돌이 난다.
    if (p_metrics_ != nullptr) {
        (void)nf;
        return;
    }
    const int new_bps = FEC_HARQ::bps_from_nf(nf);
    if (new_bps >= FEC_HARQ::BPS64_MIN_OPERABLE &&
        new_bps <= FEC_HARQ::BPS64_MAX) {
        cur_bps64_ = new_bps;
    }
    // IQ 모드 전환은 Tick_Adaptive_BPS()에서만 수행 (히스테리시스 보장)
}
void HTS_V400_Dispatcher::Set_Lab_BPS64(int bps) noexcept {
    cur_bps64_ = FEC_HARQ::bps_clamp_runtime(bps);
}
void HTS_V400_Dispatcher::Set_Lab_IQ_Mode_Jam_Harness() noexcept {
    iq_mode_ = IQ_Mode::IQ_SAME;
    iq_upgrade_count_ = 0u;
}
void HTS_V400_Dispatcher::Set_RF_Metrics(HTS_RF_Metrics *p) noexcept {
    // 비소유 포인터 저장 — nullptr 허용 (Tick 무동작 모드)
    p_metrics_ = p;
}
void HTS_V400_Dispatcher::Tick_Adaptive_BPS() noexcept {
    if (p_metrics_ == nullptr) {
        return;
    }
    const RxPhase prev_phase = phase_;
    const int prev_bps = cur_bps64_;
    const IQ_Mode prev_iq = iq_mode_;
    uint32_t need_reset = 0u;
    const uint8_t bps = p_metrics_->current_bps.load(std::memory_order_acquire);
    if (bps >= static_cast<uint8_t>(FEC_HARQ::BPS64_MIN) &&
        bps <= static_cast<uint8_t>(FEC_HARQ::BPS64_MAX)) {
        const int new_bps = FEC_HARQ::bps_clamp_runtime(static_cast<int>(bps));
        if (new_bps != prev_bps) {
            cur_bps64_ = new_bps;
            need_reset |=
                static_cast<uint32_t>(prev_phase != RxPhase::WAIT_SYNC);
        }
    }
    // ── 적응형 I/Q 모드 전환 (히스테리시스) ──────────────
    const uint32_t nf = p_metrics_->ajc_nf.load(std::memory_order_acquire);
    if (nf >= NF_IQ_SAME_TH) {
        iq_mode_ = IQ_Mode::IQ_SAME;
        iq_upgrade_count_ = 0u;
        if (cur_bps64_ > FEC_HARQ::BPS64_MIN_OPERABLE) {
            cur_bps64_ = FEC_HARQ::BPS64_MIN_OPERABLE;
            need_reset |=
                static_cast<uint32_t>(prev_phase != RxPhase::WAIT_SYNC);
        }
        if (prev_iq != IQ_Mode::IQ_SAME) {
            need_reset |=
                static_cast<uint32_t>(prev_phase != RxPhase::WAIT_SYNC);
        }
    } else if (nf < NF_IQ_SPLIT_TH) {
        if (iq_upgrade_count_ < IQ_UPGRADE_GUARD) {
            iq_upgrade_count_++;
        }
        if (iq_upgrade_count_ >= IQ_UPGRADE_GUARD) {
            iq_mode_ = IQ_Mode::IQ_INDEPENDENT;
            if (cur_bps64_ < IQ_BPS_PEACETIME) {
                cur_bps64_ = IQ_BPS_PEACETIME;
                need_reset |=
                    static_cast<uint32_t>(prev_phase != RxPhase::WAIT_SYNC);
            }
            if (prev_iq != IQ_Mode::IQ_INDEPENDENT) {
                need_reset |=
                    static_cast<uint32_t>(prev_phase != RxPhase::WAIT_SYNC);
            }
        }
    } else {
        iq_upgrade_count_ = 0u;
    }
    if (ir_mode_) {
        iq_mode_ = IQ_Mode::IQ_SAME;
        iq_upgrade_count_ = 0u;
    }
    if (need_reset != 0u) {
        full_reset_();
    }
}
void HTS_V400_Dispatcher::full_reset_() noexcept {
    // WAIT_SYNC 전이는 모든 상태에서 무조건 합법
    phase_ = RxPhase::WAIT_SYNC;
    rf_settle_chips_remaining_ = 0;
    cur_mode_ = PayloadMode::UNKNOWN;
    buf_idx_ = 0;
    pre_phase_ = 0;
    first_c63_ = 0;
    m63_gap_ = 0;
    dc_est_I_ = 0;
    dc_est_Q_ = 0;
    cfo_.Init();
    tpc_.Init();
    pre_agc_.Init();
#if defined(HTS_PHASE0_WALSH_BANK)
    if (k_W63_FWHT_ROW == 255u) {
        k_W63_FWHT_ROW = calc_kw63_fwht_row_();
#if defined(HTS_DIAG_PRINTF)
        std::printf("[DIAG-SELF-CAL] k_W63_FWHT_ROW = %u (Walsh ordering 무관)\n",
                    static_cast<unsigned>(k_W63_FWHT_ROW));
#endif
    }
#endif
    p0_chip_count_ = 0;
    p0_carry_count_ = 0;
    p1_carry_pending_ = 0;
    p1_carry_prefix_ = 0;
    p1_tail_collect_rem_ = 0;
    p1_tail_idx_ = 0;
    psal_pending_ = false;
    psal_off_ = 0;
    psal_e63_ = 0;
    dominant_row_ = 63u;
    est_I_ = 0;
    est_Q_ = 0;
    est_count_ = 0;
    derot_shift_ = 17;
    std::memset(p0_buf128_I_, 0, sizeof(p0_buf128_I_));
    std::memset(p0_buf128_Q_, 0, sizeof(p0_buf128_Q_));
    std::memset(p0_carry_I_, 0, sizeof(p0_carry_I_));
    std::memset(p0_carry_Q_, 0, sizeof(p0_carry_Q_));
    wait_sync_head_ = 0;
    wait_sync_count_ = 0;
    hdr_count_ = 0;
    hdr_fail_ = 0;
    pay_recv_ = 0;
    v1_idx_ = 0;
    sym_idx_ = 0;
    harq_round_ = 0;
    harq_inited_ = false;
    SecureMemory::secureWipe(static_cast<void *>(&rx_), sizeof(rx_));
    SecureMemory::secureWipe(static_cast<void *>(v1_rx_), sizeof(v1_rx_));
    SecureMemory::secureWipe(static_cast<void *>(orig_I_), sizeof(orig_I_));
    SecureMemory::secureWipe(static_cast<void *>(orig_Q_), sizeof(orig_Q_));
    SecureMemory::secureWipe(static_cast<void *>(&orig_acc_),
                             sizeof(orig_acc_));
    SecureMemory::secureWipe(static_cast<void *>(&wb_), sizeof(wb_));
    // CCM union 전체 — file-scope `g_harq_ccm_union` 직접 참조 (Chase/IR 공용)
    SecureMemory::secureWipe(static_cast<void *>(&g_harq_ccm_union),
                             sizeof(g_harq_ccm_union));
    ir_rv_ = 0;
    sic_expect_valid_ = false;
    retx_ready_ = false;
    std::memset(g_sic_bits, 0, sizeof(g_sic_bits));
    std::memset(buf_I_, 0, sizeof(buf_I_));
    std::memset(buf_Q_, 0, sizeof(buf_Q_));
    if (ir_mode_ && ir_state_ != nullptr) {
        FEC_HARQ::IR_Init(*ir_state_);
    }
    holo_lpi_rx_chip_idx_ = 0u;
    holo_lpi_rx_scalars_seq_ = 0xFFFFFFFFu;
}
void HTS_V400_Dispatcher::update_derot_shift_from_est_() noexcept {
    const int32_t mI = est_I_ >> 31;
    const int32_t mQ = est_Q_ >> 31;
    const int32_t abs_eI = (est_I_ + mI) ^ mI;
    const int32_t abs_eQ = (est_Q_ + mQ) ^ mQ;
    const uint64_t est_mag64 =
        static_cast<uint64_t>(static_cast<uint32_t>(abs_eI)) +
        static_cast<uint64_t>(static_cast<uint32_t>(abs_eQ));
    if (est_mag64 == 0ull) {
        derot_shift_ = 17;
        return;
    }
    const uint32_t mag_u =
        (est_mag64 > static_cast<uint64_t>(UINT32_MAX))
            ? UINT32_MAX
            : static_cast<uint32_t>(est_mag64);
    int sh = 0;
    for (int b = 31; b >= 0; --b) {
        if ((mag_u & (1u << static_cast<unsigned>(b))) != 0u) {
            sh = b;
            break;
        }
    }
    derot_shift_ = sh;
}
// =====================================================================
//
//  CFI: key=(from<<2)|to, 16슬롯 LUT(RF_SETTLING 포함) + 클램프
// =====================================================================
uint32_t HTS_V400_Dispatcher::set_phase_(RxPhase target) noexcept {
    const uint32_t f = static_cast<uint32_t>(phase_);
    const uint32_t t = static_cast<uint32_t>(target);
    const uint32_t key = (f << 2u) | t;
    static constexpr uint8_t k_trans_legal[16] = {
        1u, 1u, 0u, 1u, 1u, 0u, 1u, 1u, 1u, 0u, 0u, 1u, 1u, 0u, 0u, 0u};
    const uint32_t k_ok = static_cast<uint32_t>(key < 16u);
    const uint32_t idx = key * k_ok + 15u * (1u - k_ok);
    const uint32_t legal_u = static_cast<uint32_t>(k_trans_legal[idx]);
    const uint32_t p = static_cast<uint32_t>(phase_);
    phase_ = static_cast<RxPhase>(t * legal_u + p * (1u - legal_u));
#if defined(HTS_DIAG_PRINTF)
    if (legal_u != 0u) {
        static int s_phase_trans = 0;
        ++s_phase_trans;
        if (s_phase_trans <= 50) {
            std::printf("[PHASE] old=%d new=%d count=%d\n",
                        static_cast<int>(p), static_cast<int>(phase_),
                        s_phase_trans);
        }
    }
#endif
    if (legal_u == 0u) {
        full_reset_();
        return PHASE_TRANSFER_MASK_FAIL;
    }
    return PHASE_TRANSFER_MASK_OK;
}
void HTS_V400_Dispatcher::fhss_abort_rx_for_hop_() noexcept {
    buf_idx_ = 0;
    wait_sync_head_ = 0;
    wait_sync_count_ = 0;
    pre_phase_ = 0;
    first_c63_ = 0;
    m63_gap_ = 0;
    p0_chip_count_ = 0;
    p0_carry_count_ = 0;
    p1_carry_pending_ = 0;
    p1_carry_prefix_ = 0;
    p1_tail_collect_rem_ = 0;
    p1_tail_idx_ = 0;
    psal_pending_ = false;
    psal_off_ = 0;
    psal_e63_ = 0;
    est_I_ = 0;
    est_Q_ = 0;
    est_count_ = 0;
    derot_shift_ = 17;
    hdr_count_ = 0;
    hdr_fail_ = 0;
    pay_recv_ = 0;
    pay_total_ = 0;
    pay_cps_ = 0;
    v1_idx_ = 0;
    sym_idx_ = 0;
    harq_round_ = 0;
    harq_inited_ = false;
    retx_ready_ = false;
    cur_mode_ = PayloadMode::UNKNOWN;
}
uint8_t HTS_V400_Dispatcher::FHSS_Derive_Channel(uint32_t seed,
                                                 uint32_t seq) noexcept {
    uint32_t x = seed ^ seq;
    x ^= x << 13u;
    x ^= x >> 17u;
    x ^= x << 5u;
    x ^= seq << 15u;
    x ^= seed >> 3u;
    x ^= (seq << 7u) ^ (seed << 11u);
    return static_cast<uint8_t>(x & static_cast<uint32_t>(0x7Fu));
}
uint8_t HTS_V400_Dispatcher::FHSS_Request_Hop_As_Tx() noexcept {
    const uint32_t in_settle =
        static_cast<uint32_t>(phase_ == RxPhase::RF_SETTLING);
    if (in_settle != 0u) {
        return static_cast<uint8_t>(0xFFu);
    }
    const uint8_t ch = FHSS_Derive_Channel(seed_, tx_seq_);
    Mock_RF_Synth_Set_Channel(ch);
    tx_seq_ = tx_seq_ + 1u;
    fhss_abort_rx_for_hop_();
    rf_settle_chips_remaining_ = FHSS_SETTLE_CHIPS;
    const uint32_t ph_ok = set_phase_(RxPhase::RF_SETTLING);
    if (ph_ok == PHASE_TRANSFER_MASK_FAIL) {
        return static_cast<uint8_t>(0xFFu);
    }
    return ch;
}
uint8_t HTS_V400_Dispatcher::FHSS_Request_Hop_As_Rx() noexcept {
    const uint32_t in_settle =
        static_cast<uint32_t>(phase_ == RxPhase::RF_SETTLING);
    if (in_settle != 0u) {
        return static_cast<uint8_t>(0xFFu);
    }
    const uint8_t ch = FHSS_Derive_Channel(seed_, rx_seq_);
    Mock_RF_Synth_Set_Channel(ch);
    rx_seq_ = rx_seq_ + 1u;
    fhss_abort_rx_for_hop_();
    rf_settle_chips_remaining_ = FHSS_SETTLE_CHIPS;
    const uint32_t ph_ok = set_phase_(RxPhase::RF_SETTLING);
    if (ph_ok == PHASE_TRANSFER_MASK_FAIL) {
        return static_cast<uint8_t>(0xFFu);
    }
    return ch;
}
bool HTS_V400_Dispatcher::FHSS_Is_Rf_Settling() const noexcept {
    return static_cast<uint32_t>(phase_) ==
           static_cast<uint32_t>(RxPhase::RF_SETTLING);
}
void HTS_V400_Dispatcher::Reset() noexcept {
    full_reset_();
    // [REMOVED Step3] ajc_.Reset(16), ajc_last_nc_ — AntiJam 엔진 제거
}
uint32_t HTS_V400_Dispatcher::parse_hdr_(PayloadMode &mode,
                                         int &plen) noexcept {
    const uint16_t hdr = (static_cast<uint16_t>(hdr_syms_[0]) << 6u) |
                         static_cast<uint16_t>(hdr_syms_[1]);
    const uint8_t mb = static_cast<uint8_t>((hdr >> 10u) & 0x03u);
    const uint32_t rx_iq_split =
        static_cast<uint32_t>((hdr & HDR_IQ_BIT) != 0u);
    plen = static_cast<int>(hdr & 0x01FFu);
    iq_mode_ = static_cast<IQ_Mode>(
        static_cast<uint32_t>(IQ_Mode::IQ_INDEPENDENT) * rx_iq_split +
        static_cast<uint32_t>(IQ_Mode::IQ_SAME) * (1u - rx_iq_split));
    static constexpr PayloadMode k_hdr_modes[4] = {
        PayloadMode::VIDEO_1,
        PayloadMode::VIDEO_16,
        PayloadMode::VOICE,
        PayloadMode::DATA,
    };
    mode = k_hdr_modes[static_cast<size_t>(mb & 3u)];
    const uint32_t m0 = static_cast<uint32_t>(mb == 0u);
    const uint32_t m1 = static_cast<uint32_t>(mb == 1u);
    const uint32_t m2 = static_cast<uint32_t>(mb == 2u);
    const uint32_t m3 = static_cast<uint32_t>(mb == 3u);
    const uint32_t plen_ok_v1 = static_cast<uint32_t>(plen == FEC_HARQ::NSYM1);
    const uint32_t plen_ok_v16 =
        static_cast<uint32_t>(plen == FEC_HARQ::NSYM16);
    const int bps = FEC_HARQ::bps_from_nsym(plen);
    const uint32_t bps_ge =
        static_cast<uint32_t>(bps >= FEC_HARQ::BPS64_MIN_OPERABLE);
    const uint32_t bps_le = static_cast<uint32_t>(bps <= FEC_HARQ::BPS64_MAX);
    const uint32_t plen_sym =
        static_cast<uint32_t>(plen == FEC_HARQ::nsym_for_bps(bps));
    const uint32_t data_ok = m3 & bps_ge & bps_le & plen_sym;
    const uint32_t ok_u = (m0 & plen_ok_v1) | (m1 & plen_ok_v16) |
                          (m2 & plen_ok_v16) | data_ok;
    const int d_int = static_cast<int>(data_ok);
    cur_bps64_ =
        (bps * static_cast<int>(data_ok)) + (cur_bps64_ * (1 - d_int));
    if (ir_mode_) {
        iq_mode_ = IQ_Mode::IQ_SAME;
        iq_upgrade_count_ = 0u;
    }
    return static_cast<uint32_t>(0u - ok_u);
}
void HTS_V400_Dispatcher::on_sym_() noexcept {
    pay_recv_++;
    if (cur_mode_ == PayloadMode::VIDEO_1) {
        if (v1_idx_ < 80) {
            v1_rx_[v1_idx_++] = buf_I_[0];
        } else {
            full_reset_();
            return;
        }
    } else if (cur_mode_ == PayloadMode::VIDEO_16 ||
               cur_mode_ == PayloadMode::VOICE ||
               cur_mode_ == PayloadMode::DATA) {
        const int nc = (cur_mode_ == PayloadMode::DATA) ? 64 : 16;
        /* IR-HARQ: RV마다 송신 파형이 다르므로 칩 도메인 += 금지.
           결합은 FEC Decode*_IR 의 ir_state_(LLR)에서만 수행; 수신 칩은 매 심볼
           덮어쓰기. */
        std::memcpy(orig_I_, buf_I_, nc * sizeof(int16_t));
        std::memcpy(orig_Q_, buf_Q_, nc * sizeof(int16_t));
        // [Walsh_Row_Permuter] TX: enc = raw ^ mask 후 walsh_enc(enc).
        //  RX: chip[j] *= (−1)^{popc(mask∧j)} — Walsh-Hadamard 행 XOR 성질로
        //  FEC(Feed*) 가 기대하는 raw 도메인 칩으로 복원 (mask=0 이면 항등).
        if (nc == 16) {
            const uint8_t m4 = walsh_permuter_.Encode_4bit(0u);
            for (int c = 0; c < 16; ++c) {
                const uint32_t pu =
                    popc32(static_cast<uint32_t>(m4) &
                           static_cast<uint32_t>(c)) &
                    1u;
                const int32_t sgn = 1 - 2 * static_cast<int32_t>(pu);
                buf_I_[c] = static_cast<int16_t>(
                    static_cast<int32_t>(buf_I_[c]) * sgn);
                buf_Q_[c] = static_cast<int16_t>(
                    static_cast<int32_t>(buf_Q_[c]) * sgn);
            }
        } else {
            const uint8_t m6 = walsh_permuter_.Encode_6bit(0u);
            for (int c = 0; c < 64; ++c) {
                const uint32_t pu =
                    popc32(static_cast<uint32_t>(m6) &
                           static_cast<uint32_t>(c)) &
                    1u;
                const int32_t sgn = 1 - 2 * static_cast<int32_t>(pu);
                buf_I_[c] = static_cast<int16_t>(
                    static_cast<int32_t>(buf_I_[c]) * sgn);
                buf_Q_[c] = static_cast<int16_t>(
                    static_cast<int32_t>(buf_Q_[c]) * sgn);
            }
        }
        // [REMOVED Step2] cw_cancel_64_(buf_I_, buf_Q_) — NOP 확정
        // [REMOVED Step3] if (ajc_enabled_) ajc_.Process(...) — 학습 구조 불일치로 NOP
        // [REMOVED Step1] soft_clip_iq — Gaussian noise 에 무효 확정
        if (nc == 16) {
            if (sym_idx_ < FEC_HARQ::NSYM16) {
                if (ir_mode_) {
                    /* IR 칩: ECCM 후 버퍼 저장 (CW 제거 효과 반영) */
                    const int base = sym_idx_ * FEC_HARQ::C16;
                    for (int c = 0; c < nc; ++c) {
                        ir_chip_I_[base + c] = buf_I_[c];
                        ir_chip_Q_[base + c] = buf_Q_[c];
                    }
                } else {
                    FEC_HARQ::Feed16_1sym(rx_.m16, buf_I_, buf_Q_, sym_idx_);
                }
                for (int c = 0; c < nc; ++c) {
                    const uint8_t hiI =
                        static_cast<uint8_t>((orig_I_[c] >> 12) & 0x0Fu);
                    const uint8_t hiQ =
                        static_cast<uint8_t>((orig_Q_[c] >> 12) & 0x0Fu);
                    orig_acc_.acc16.iq4[sym_idx_][c] =
                        static_cast<uint8_t>((hiI << 4u) | hiQ);
                }
                sym_idx_++;
            } else {
                full_reset_();
                return;
            }
        } else {
            const int nsym64 = cur_nsym64_();
            if (iq_mode_ == IQ_Mode::IQ_INDEPENDENT && !ir_mode_) {
                // ── [적응형 I/Q] I/Q 독립 RX: 칩슬롯당 2심볼 ──
                //  I 채널 → 짝수 sym_idx_, Q 채널 → 홀수 sym_idx_
                //  HARQ I 누적기: 짝수 심볼 전용
                //  HARQ Q 누적기: 홀수 심볼 전용
                const int si_I = sym_idx_;     // I 채널 심볼 인덱스
                const int si_Q = sym_idx_ + 1; // Q 채널 심볼 인덱스
                if (si_Q < nsym64) {
                    // I 채널 HARQ 누적 (짝수 심볼)
                    for (int c = 0; c < nc; ++c) {
                        rx_.m64_I.aI[si_I][c] +=
                            static_cast<int32_t>(buf_I_[c]);
                    }
                    // Q 채널 HARQ 누적 (홀수 심볼)
                    for (int c = 0; c < nc; ++c) {
                        rx_.m64_I.aI[si_Q][c] +=
                            static_cast<int32_t>(buf_Q_[c]);
                    }
                    for (int c = 0; c < nc; ++c) {
                        const uint8_t hiI =
                            static_cast<uint8_t>((orig_I_[c] >> 12) & 0x0Fu);
                        orig_acc_.acc64.iq4[si_I][c] =
                            static_cast<uint8_t>(hiI << 4u);
                        const uint8_t hiQ =
                            static_cast<uint8_t>((orig_Q_[c] >> 12) & 0x0Fu);
                        orig_acc_.acc64.iq4[si_Q][c] =
                            static_cast<uint8_t>(hiQ << 4u);
                    }
                    sym_idx_ += 2;
                    pay_recv_++; // 칩슬롯 기준 카운트
                } else {
                    full_reset_();
                    return;
                }
            } else {
                // ── I=Q 동일 또는 IR-HARQ (칩 보관) ──
                if (sym_idx_ < nsym64) {
                    if (ir_mode_) {
                        /* IR 칩: ECCM 후 버퍼 + SIC (CW 제거 효과 반영) */
                        const int base = sym_idx_ * FEC_HARQ::C64;
                        const uint32_t use_sic_u =
                            static_cast<uint32_t>(sic_expect_valid_) & 1u;
                        // SIC: 1비트 부호 비트맵에서 Walsh 칩 복원 (I=Q 동일)
                        const uint64_t sic_sym_bits =
                            g_sic_bits[static_cast<std::size_t>(sym_idx_)];
                        const int32_t sic_amp =
                            static_cast<int32_t>(sic_walsh_amp_);
                        {
                            // Derotation 제거: FEC_HARQ::Decode64_IR이 위상 불변
                            // 디코딩(I²+Q² 에너지 기반)을 수행하므로 Dispatcher
                            // 레벨 derotation은 불필요하며, RF 환경에서 est 벡터의
                            // 노이즈로 인해 스케일 왜곡을 유발하여 FEC 실패 원인이 됨.
                            // T9(FEC 직접 호출)가 99/100 성공하는 것이 증명.
                            for (int c = 0; c < nc; ++c) {
                                int32_t vi = static_cast<int32_t>(buf_I_[c]);
                                int32_t vq = static_cast<int32_t>(buf_Q_[c]);
                                // SIC 차감: 기저대역 직접 차감
                                const uint32_t neg = static_cast<uint32_t>(
                                    (sic_sym_bits >>
                                     static_cast<uint32_t>(c)) & 1u);
                                const int32_t sic_chip =
                                    sic_amp - 2 * sic_amp *
                                    static_cast<int32_t>(neg);
                                const int32_t sub =
                                    sic_chip *
                                    static_cast<int32_t>(use_sic_u);
                                vi -= sub;
                                vq -= sub;
                                ir_chip_I_[base + c] = ssat16_dispatch_(vi);
                                ir_chip_Q_[base + c] = ssat16_dispatch_(vq);
                            }
                        }
                        for (int c = 0; c < nc; ++c) {
                            const uint8_t hiI = static_cast<uint8_t>(
                                (orig_I_[c] >> 12) & 0x0Fu);
                            const uint8_t hiQ = static_cast<uint8_t>(
                                (orig_Q_[c] >> 12) & 0x0Fu);
                            orig_acc_.acc64.iq4[sym_idx_][c] =
                                static_cast<uint8_t>((hiI << 4u) | hiQ);
                        }
                        sym_idx_++;
                    } else {
                        // Derotation 제거 (위 IR 경로 주석 참조)
                        for (int c = 0; c < nc; ++c) {
                            rx_.m64_I.aI[sym_idx_][c] +=
                                static_cast<int32_t>(buf_I_[c]);
                        }
                        for (int c = 0; c < nc; ++c) {
                            harq_Q_[sym_idx_][c] +=
                                static_cast<int32_t>(buf_Q_[c]);
                        }
                        for (int c = 0; c < nc; ++c) {
                            const uint8_t hiI = static_cast<uint8_t>(
                                (orig_I_[c] >> 12) & 0x0Fu);
                            const uint8_t hiQ = static_cast<uint8_t>(
                                (orig_Q_[c] >> 12) & 0x0Fu);
                            orig_acc_.acc64.iq4[sym_idx_][c] =
                                static_cast<uint8_t>((hiI << 4u) | hiQ);
                        }
                        sym_idx_++;
                    }
                } else {
                    full_reset_();
                    return;
                }
            }
        }
        // [REMOVED Step3] AJC walsh 피드백 블록 (Update_AJC × N) — 구조적 NOP
    }
    buf_idx_ = 0;
    if (pay_recv_ >= pay_total_)
        try_decode_();
}
void HTS_V400_Dispatcher::try_decode_() noexcept {
    // [BUG-FIX-IR5] wb_ 초기화: Encode 잔류 데이터가 Decode 경로를 오염시키는
    // 것을 방지. 16칩(NSYM16=172)은 wb_ 사용 영역이 작아 영향 없으나,
    // 64칩(NSYM64=172, BPS=4)은 전체 TOTAL_CODED(688) 슬롯을 사용하므로 필수.
    std::memset(&wb_, 0, sizeof(wb_));
    const uint8_t walsh_shift_payload = current_walsh_shift_();
    DecodedPacket pkt = {};
    pkt.mode = cur_mode_;
    pkt.success_mask = DecodedPacket::DECODE_MASK_FAIL;
    uint32_t il = seed_ ^ (rx_seq_ * 0xA5A5A5A5u);
    /* WorkBuf: try_decode_ 진입 시 memset 초기화 완료 (BUG-FIX-IR5). */
    if (cur_mode_ == PayloadMode::VIDEO_1) {
        pkt.success_mask =
            static_cast<uint32_t>(0u - static_cast<uint32_t>(FEC_HARQ::Decode1(
                                           v1_rx_, pkt.data, &pkt.data_len)));
        pkt.harq_k = 1;
        handle_video_(pkt.success_mask);
        if (pkt.success_mask != 0u && pkt.data_len >= 8) {
            tpc_rx_feedback_after_decode_(pkt);
        }
        if (on_pkt_ != nullptr) {
            on_pkt_(pkt);
        }
        rx_seq_++;
        full_reset_();
    } else if (cur_mode_ == PayloadMode::VIDEO_16 ||
               cur_mode_ == PayloadMode::VOICE) {
        if (ir_mode_) {
            harq_round_++;
            const int rv = ir_rv_;
#if defined(HTS_IR_DIAG_ENABLE)
            if (g_hts_ir_diag_chip0 != 0 && ir_chip_I_ != nullptr &&
                ir_state_ != nullptr) {
                std::printf("[IR-DIAG] pre-Decode16_IR feed=%d harq_round_=%d "
                            "ir_chip_I_[0]=%d ir_state.rounds_done=%d\n",
                            static_cast<int>(g_hts_ir_diag_feed_idx),
                            harq_round_, static_cast<int>(ir_chip_I_[0]),
                            ir_state_->rounds_done);
            }
#endif
            const bool ir16_ok = (ir_state_ != nullptr &&
                                  ir_chip_I_ != nullptr &&
                                  ir_chip_Q_ != nullptr);
            pkt.success_mask = static_cast<uint32_t>(
                0u -
                static_cast<uint32_t>(
                    ir16_ok &&
                    FEC_HARQ::Decode16_IR(
                        ir_chip_I_, ir_chip_Q_, FEC_HARQ::NSYM16,
                        FEC_HARQ::C16, FEC_HARQ::BPS16, il, rv, *ir_state_,
                        pkt.data, &pkt.data_len, wb_, walsh_shift_payload)));
#if defined(HTS_DIAG_PRINTF)
            if (ir16_ok) {
                std::printf(
                    "[PAYLOAD-SHIFT] walsh_shift=%u dom=%u\n",
                    static_cast<unsigned>(walsh_shift_payload),
                    static_cast<unsigned>(dominant_row_));
            }
#endif
            ir_rv_ = (ir_rv_ + 1) & 3;
            sic_expect_valid_ = false;
        } else {
            FEC_HARQ::Advance_Round_16(rx_.m16);
            harq_round_++;
            pkt.success_mask = static_cast<uint32_t>(
                0u - static_cast<uint32_t>(FEC_HARQ::Decode16(
                         rx_.m16, pkt.data, &pkt.data_len, il, wb_)));
        }
        pkt.harq_k = harq_round_;
        const uint32_t dec_ok = static_cast<uint32_t>(pkt.success_mask != 0u);
        const uint32_t harq_ex =
            static_cast<uint32_t>(harq_round_ >= max_harq_);
        // 연속모드에서는 harq 소진을 하네스 외부(feeds 루프)에서 관리
        // max_harq_는 DATA_K(32)로 통일하여 VOICE도 32라운드 누적 허용
        const uint32_t finish = dec_ok | harq_ex;
        if (finish != 0u) {
            if (dec_ok != 0u) {
                harq_feedback_seed_(pkt.data, pkt.data_len, 16, il);
            }
            if (cur_mode_ == PayloadMode::VIDEO_16) {
                handle_video_(pkt.success_mask);
            }
            if (pkt.success_mask != 0u && pkt.data_len >= 8) {
                tpc_rx_feedback_after_decode_(pkt);
            }
            if (on_pkt_ != nullptr) {
                on_pkt_(pkt);
            }
            rx_seq_++;
            full_reset_();
        } else {
            pay_recv_ = 0;
            sym_idx_ = 0;
            /* IR 칩: on_sym_ 라운드마다 덮어쓰기 — memset 불필요 */
            /* BUG-FIX-RETX5: 실패 시 READ_PAYLOAD 유지, 재동기 회피 */
            retx_ready_ = true;
            buf_idx_ = 0;
            // set_phase_(RxPhase::WAIT_SYNC);
        }
    } else if (cur_mode_ == PayloadMode::DATA) {
        harq_round_++;
        if (ir_mode_) {
                const int bps = cur_bps64_;
                if (bps >= FEC_HARQ::BPS64_MIN_OPERABLE &&
                    bps <= FEC_HARQ::BPS64_MAX && ir_state_ != nullptr &&
                    ir_chip_I_ != nullptr && ir_chip_Q_ != nullptr) {
                    const int nsym_ir = FEC_HARQ::nsym_for_bps(bps);
                    const int rv = ir_rv_;
#if defined(HTS_IR_DIAG_ENABLE)
                    if (g_hts_ir_diag_chip0 != 0) {
                        std::printf(
                            "[IR-DIAG] pre-Decode64_IR feed=%d harq_round_=%d "
                            "ir_chip_I_[0]=%d ir_state.rounds_done=%d\n",
                            static_cast<int>(g_hts_ir_diag_feed_idx),
                            harq_round_, static_cast<int>(ir_chip_I_[0]),
                            ir_state_->rounds_done);
                        std::printf(
                            "[IR-DIAG] Decode64_IR args: bps=%d nsym_ir=%d "
                            "il=0x%08x rv=%d rounds_done=%d\n",
                            bps, nsym_ir, static_cast<unsigned>(il), rv,
                            ir_state_->rounds_done);
                    }
#endif
                    pkt.success_mask = static_cast<uint32_t>(
                        0u -
                        static_cast<uint32_t>(FEC_HARQ::Decode64_IR(
                            ir_chip_I_, ir_chip_Q_, nsym_ir, FEC_HARQ::C64, bps,
                            il, rv, *ir_state_, pkt.data, &pkt.data_len, wb_,
                            walsh_shift_payload)));
#if defined(HTS_DIAG_PRINTF)
                    std::printf(
                        "[PAYLOAD-SHIFT] walsh_shift=%u dom=%u\n",
                        static_cast<unsigned>(walsh_shift_payload),
                        static_cast<unsigned>(dominant_row_));
#endif
                }
                ir_rv_ = (ir_rv_ + 1) & 3;
                {
                    const int bps_sic = cur_bps64_;
                    const bool ir64_attempt =
                        (bps_sic >= FEC_HARQ::BPS64_MIN_OPERABLE &&
                         bps_sic <= FEC_HARQ::BPS64_MAX &&
                         ir_state_ != nullptr && ir_chip_I_ != nullptr &&
                         ir_chip_Q_ != nullptr);
                    if (ir64_attempt) {
                        if (pkt.success_mask != 0u) {
                            sic_expect_valid_ = false;
                        } else if (sic_ir_enabled_) {
                            fill_sic_expected_64_();
                        } else {
                            sic_expect_valid_ = false;
                        }
                    } else {
                        sic_expect_valid_ = false;
                    }
                }
            } else {
                sic_expect_valid_ = false;
                if (!rx_.m64_I.ok)
                    rx_.m64_I.k++;
                {
                    const int bps = cur_bps64_;
                    if (bps >= FEC_HARQ::BPS64_MIN_OPERABLE &&
                        bps <= FEC_HARQ::BPS64_MAX) {
                        const int nsym = FEC_HARQ::nsym_for_bps(bps);
                        pkt.success_mask = static_cast<uint32_t>(
                            0u -
                            static_cast<uint32_t>(FEC_HARQ::Decode_Core_Split(
                                &rx_.m64_I.aI[0][0], harq_Q_[0], nsym,
                                FEC_HARQ::C64, bps, pkt.data, &pkt.data_len, il,
                                wb_)));
                    }
                }
            }
        pkt.harq_k = harq_round_;
        const uint32_t dec_ok = static_cast<uint32_t>(pkt.success_mask != 0u);
        const uint32_t harq_ex =
            static_cast<uint32_t>(harq_round_ >= max_harq_);
        const uint32_t finish = dec_ok | harq_ex;
        if (finish != 0u) {
            if (dec_ok != 0u) {
                if (!ir_mode_) {
                    rx_.m64_I.ok = true;
                }
                harq_feedback_seed_(pkt.data, pkt.data_len, 64, il);
            }
            if (pkt.success_mask != 0u && pkt.data_len >= 8) {
                tpc_rx_feedback_after_decode_(pkt);
            }
            if (on_pkt_ != nullptr) {
                on_pkt_(pkt);
            }
            rx_seq_++;
            full_reset_();
        } else {
            pay_recv_ = 0;
            sym_idx_ = 0;
            /* IR 칩: on_sym_ 라운드마다 덮어쓰기 — memset 불필요 */
            /* BUG-FIX-RETX5: 실패 시 READ_PAYLOAD 유지, 재동기 회피 */
            retx_ready_ = true;
            buf_idx_ = 0;
            // set_phase_(RxPhase::WAIT_SYNC);
        }
    }
}
void HTS_V400_Dispatcher::harq_feedback_seed_(const uint8_t *data, int data_len,
                                              int nc, uint32_t il) noexcept {
    if (!data || data_len <= 0)
        return;
    if (nc == 16) {
        uint8_t *const correct_syms = g_v400_sym_scratch;
        int enc_n = 0;
        if (ir_mode_) {
            const int rv_fb = (ir_rv_ + 3) & 3;
            enc_n = FEC_HARQ::Encode16_IR(data, data_len, correct_syms, il,
                                          rv_fb, wb_);
        } else {
            enc_n = FEC_HARQ::Encode16(data, data_len, correct_syms, il, wb_);
        }
        if (enc_n <= 0) {
            SecureMemory::secureWipe(static_cast<void *>(correct_syms),
                                     sizeof(g_v400_sym_scratch));
            return;
        }
        // [REMOVED Step3] HARQ 피드백 루프의 AJC Update_AJC — AJC 제거
        SecureMemory::secureWipe(static_cast<void *>(correct_syms),
                                 sizeof(g_v400_sym_scratch));
    } else if (nc == 64) {
        uint8_t *const correct_syms = g_v400_sym_scratch;
        int enc_n = 0;
        if (ir_mode_) {
            // try_decode_: Decode64_IR 직후 ir_rv_ 가 +1 되므로
            // 피드백용 직전 라운드 RV ≡ (ir_rv_ + 3) & 3
            const int rv_fb = (ir_rv_ + 3) & 3;
            enc_n = FEC_HARQ::Encode64_IR(data, data_len, correct_syms, il,
                                          cur_bps64_, rv_fb, wb_);
        } else {
            enc_n = FEC_HARQ::Encode64_A(data, data_len, correct_syms, il,
                                         cur_bps64_, wb_);
        }
        if (enc_n <= 0) {
            SecureMemory::secureWipe(static_cast<void *>(correct_syms),
                                     sizeof(g_v400_sym_scratch));
            return;
        }
        // [REMOVED Step3] HARQ 피드백 루프의 AJC Update_AJC — AJC 제거
        SecureMemory::secureWipe(static_cast<void *>(correct_syms),
                                 sizeof(g_v400_sym_scratch));
    }
}
void HTS_V400_Dispatcher::handle_video_(uint32_t decode_ok_mask) noexcept {
    const uint32_t ok = decode_ok_mask & 1u;
    if (ok != 0u) {
        vid_succ_++;
        vid_fail_ = 0;
        if (active_video_ == PayloadMode::VIDEO_16 &&
            vid_succ_ >= VIDEO_RECOVER_TH) {
            active_video_ = PayloadMode::VIDEO_1;
            vid_succ_ = 0;
            if (on_ctrl_ != nullptr) {
                on_ctrl_(PayloadMode::VIDEO_1);
            }
        }
    } else {
        vid_fail_++;
        vid_succ_ = 0;
        if (active_video_ == PayloadMode::VIDEO_1 &&
            vid_fail_ >= VIDEO_FAIL_TH) {
            active_video_ = PayloadMode::VIDEO_16;
            vid_fail_ = 0;
            if (on_ctrl_ != nullptr) {
                on_ctrl_(PayloadMode::VIDEO_16);
            }
        }
    }
}
namespace {
alignas(64) static int16_t g_bp_sink_i[64];
alignas(64) static int16_t g_bp_sink_q[64];
static inline int16_t *bp_dst_i(int16_t *oI, int pos,
                                std::uintptr_t okm) noexcept {
    const std::uintptr_t p = reinterpret_cast<std::uintptr_t>(&oI[pos]);
    return reinterpret_cast<int16_t *>(
        (p & okm) |
        (reinterpret_cast<std::uintptr_t>(g_bp_sink_i) & ~okm));
}
static inline int16_t *bp_dst_q(int16_t *oQ, int pos,
                                std::uintptr_t okm) noexcept {
    const std::uintptr_t p = reinterpret_cast<std::uintptr_t>(&oQ[pos]);
    return reinterpret_cast<int16_t *>(
        (p & okm) |
        (reinterpret_cast<std::uintptr_t>(g_bp_sink_q) & ~okm));
}
} // namespace
int HTS_V400_Dispatcher::Build_Packet(PayloadMode mode, const uint8_t *info,
                                      int ilen, int16_t amp, int16_t *oI,
                                      int16_t *oQ, int max_c) noexcept {
    if (phase_ == RxPhase::RF_SETTLING) {
        return 0;
    }
    if (info == nullptr || oI == nullptr || oQ == nullptr)
        return 0;
    if (ilen < 0 || max_c <= 0)
        return 0;
    // [Walsh_Row_Permuter] payload 스크램블 키 — TX tx_seq_·첫 라운드(ir_rv_=0 아래)
    if (!walsh_permuter_.Is_Initialized()) {
        (void)walsh_permuter_.Initialize(tx_seq_, 0u);
    } else {
        (void)walsh_permuter_.Update_Key(tx_seq_, 0u);
    }
    // TPC는 외부에서 명시적으로 사용할 때만 적용
    // 기본: caller의 amp 파라미터 사용 (기존 호환)
    tx_amp_ = amp;
    /* 신규 PDU 송신: 인코드 RV는 0부터 (try_decode_ 첫 라운드 rv=0 과 정합) */
    ir_rv_ = 0;
    /* BUG-FIX-PRE5: 프리앰블 반복 폐기, amp 부스트로 교체.
       프리앰블+헤더를 boost 배 진폭으로 전송.
       RX: 동기·헤더는 Walsh 인덱스 0..63 — walsh_dec_full_(…, cap=false)로
       전빈 탐색. 페이로드만 2^BPS 제한(cap=true)으로 FEC 심볼 집합과 정합. */
    const int16_t pre_amp =
        static_cast<int16_t>(static_cast<int32_t>(amp) * pre_boost_);
    // [BUG-FIX-PRE2] 프리앰블 반복 전송 — pre_reps_ × PRE_SYM0 + 1 × PRE_SYM1
    const int pre_chips = (pre_reps_ + 1) * 64;
    const uint32_t il = seed_ ^ (tx_seq_ * 0xA5A5A5A5u);
    static constexpr uint8_t k_tx_mb[4] = {0u, 1u, 2u, 3u};
    static constexpr int k_tx_psyms[4] = {FEC_HARQ::NSYM1, FEC_HARQ::NSYM16,
                                          FEC_HARQ::NSYM16, 0};
    const uint32_t mi = static_cast<uint32_t>(mode);
    if (mi > 3u) {
        return 0;
    }
    const uint32_t u0 = static_cast<uint32_t>(mi == 0u);
    const uint32_t u1 = static_cast<uint32_t>(mi == 1u);
    const uint32_t u2 = static_cast<uint32_t>(mi == 2u);
    const uint32_t u3 = static_cast<uint32_t>(mi == 3u);
    const uint32_t u16 = u1 | u2;
    const uint8_t mb = k_tx_mb[mi];
    const int nsym64_live = cur_nsym64_();
    // TPE: header psyms — LUT + DATA runtime nsym64
    const int psyms =
        k_tx_psyms[mi] + static_cast<int>(u3) * nsym64_live;
    const uint32_t ir_hdr_iq_same_u = static_cast<uint32_t>(ir_mode_);
    const uint32_t iq_ind_u =
        static_cast<uint32_t>(iq_mode_ == IQ_Mode::IQ_INDEPENDENT);
    const uint16_t iq_bit =
        static_cast<uint16_t>((iq_ind_u & (1u - ir_hdr_iq_same_u)) *
                              static_cast<uint32_t>(HDR_IQ_BIT));
    uint16_t hdr = (static_cast<uint16_t>(mb & 0x03u) << 10u) | iq_bit |
                   (static_cast<uint16_t>(psyms) & 0x01FFu);

    uint8_t syms_v1[80] = {};
    uint8_t syms16_ir[FEC_HARQ::NSYM16] = {};
    uint8_t syms16_pl[FEC_HARQ::NSYM16] = {};
    uint8_t syms64_ir[FEC_HARQ::NSYM64] = {};
    uint8_t syms64_pl[FEC_HARQ::NSYM64] = {};
    const int irb = static_cast<int>(ir_mode_);

    const int il_v1 = ilen * static_cast<int>(u0);
    const int il_16 = ilen * static_cast<int>(u16);
    const int il_64 = ilen * static_cast<int>(u3);

    const int n_v1 = FEC_HARQ::Encode1(info, il_v1, syms_v1);
    const int enc16_ir =
        FEC_HARQ::Encode16_IR(info, il_16, syms16_ir, il, ir_rv_, wb_);
    const int enc16_pl = FEC_HARQ::Encode16(info, il_16, syms16_pl, il, wb_);
    const int enc16 = enc16_ir * irb + enc16_pl * (1 - irb);
    const int enc64_ir = FEC_HARQ::Encode64_IR(info, il_64, syms64_ir, il,
                                               cur_bps64_, ir_rv_, wb_);
    const int enc64_pl =
        FEC_HARQ::Encode64_A(info, il_64, syms64_pl, il, cur_bps64_, wb_);
    const int enc64 = enc64_ir * irb + enc64_pl * (1 - irb);

    // IR/plain 선택 (TPE 비트마스크 — 분기 없음)
    uint8_t syms16[FEC_HARQ::NSYM16] = {};
    uint8_t syms64[FEC_HARQ::NSYM64] = {};
    const uint32_t ir_mask = 0u - static_cast<uint32_t>(irb);
    const uint32_t pl_mask = ~ir_mask;
    for (int i = 0; i < FEC_HARQ::NSYM16; ++i) {
        syms16[i] = static_cast<uint8_t>(
            (static_cast<uint32_t>(syms16_ir[i]) & ir_mask) |
            (static_cast<uint32_t>(syms16_pl[i]) & pl_mask));
    }
    for (int i = 0; i < FEC_HARQ::NSYM64; ++i) {
        syms64[i] = static_cast<uint8_t>(
            (static_cast<uint32_t>(syms64_ir[i]) & ir_mask) |
            (static_cast<uint32_t>(syms64_pl[i]) & pl_mask));
    }

    // TPE: per-mode encode validity — inactive mode is always “ok”
    const uint32_t ok_v1 =
        (1u - u0) | (0u - static_cast<uint32_t>(n_v1 > 0));
    const uint32_t ok_16 =
        (1u - u16) | (0u - static_cast<uint32_t>(enc16 > 0));
    const uint32_t ok_64 =
        (1u - u3) | (0u - static_cast<uint32_t>(enc64 > 0));
    uint32_t go_enc = ~(0u);
    go_enc &= ok_v1 & ok_16 & ok_64;

    // TPE: pay chip budget from masked mode contributions
    int pay_raw = (n_v1 * static_cast<int>(u0)) +
                  (FEC_HARQ::NSYM16 * 16 * static_cast<int>(u16)) +
                  (nsym64_live * 64 * static_cast<int>(u3));
    pay_raw &= -static_cast<int>(go_enc & 1u);
    const int total_need = pre_chips + 128 + pay_raw;
    uint32_t go = go_enc;
    go &= static_cast<uint32_t>(total_need <= max_c);

    // TPE: pointer sink mask — intptr_t sign, no 0ull−bit wrap
    const std::uintptr_t okm = static_cast<std::uintptr_t>(
        -static_cast<std::intptr_t>(go & 1u));
    const int inc = static_cast<int>(go & 1u);

    int pos = 0;
    for (int r = 0; r < pre_reps_; ++r) {
        walsh_enc(PRE_SYM0, 64, pre_amp, bp_dst_i(oI, pos, okm),
                  bp_dst_q(oQ, pos, okm));
        pos += 64 * inc;
    }
    walsh_enc(PRE_SYM1, 64, pre_amp, bp_dst_i(oI, pos, okm),
              bp_dst_q(oQ, pos, okm));
    pos += 64 * inc;
    walsh_enc(static_cast<uint8_t>((hdr >> 6u) & 0x3Fu), 64, pre_amp,
              bp_dst_i(oI, pos, okm), bp_dst_q(oQ, pos, okm));
    pos += 64 * inc;
    walsh_enc(static_cast<uint8_t>(hdr & 0x3Fu), 64, pre_amp,
              bp_dst_i(oI, pos, okm), bp_dst_q(oQ, pos, okm));
    pos += 64 * inc;

    const int n_send_v1 = n_v1 * inc * static_cast<int>(u0);
    for (int s = 0; s < n_send_v1; ++s) {
        const uint8_t sv = syms_v1[static_cast<std::size_t>(s)];
        const uint32_t nz = 0u - static_cast<uint32_t>(sv != 0u);
        const int32_t amp32 = static_cast<int32_t>(amp);
        const int32_t v32 =
            amp32 + (static_cast<int32_t>(nz) & (-2 * amp32));
        const int16_t v = static_cast<int16_t>(v32);
        int16_t *const di = bp_dst_i(oI, pos, okm);
        int16_t *const dq = bp_dst_q(oQ, pos, okm);
        di[0] = v;
        dq[0] = v;
        pos += inc;
    }
    SecureMemory::secureWipe(static_cast<void *>(syms_v1), sizeof(syms_v1));

    const int n_send16 = FEC_HARQ::NSYM16 * inc * static_cast<int>(u16);
    for (int s = 0; s < n_send16; ++s) {
        const uint8_t raw_sym4 = syms16[static_cast<std::size_t>(s)];
        const uint8_t enc_sym4 = walsh_permuter_.Encode_4bit(raw_sym4);
        walsh_enc(enc_sym4, 16, amp, bp_dst_i(oI, pos, okm),
                  bp_dst_q(oQ, pos, okm));
        pos += 16 * inc;
    }
    SecureMemory::secureWipe(static_cast<void *>(syms16), sizeof(syms16));
    SecureMemory::secureWipe(static_cast<void *>(syms16_ir), sizeof(syms16_ir));
    SecureMemory::secureWipe(static_cast<void *>(syms16_pl), sizeof(syms16_pl));

    // TPE: DATA split path gated by u3 — non-DATA ⇒ 0 Walsh iterations
    const uint32_t split_u =
        static_cast<uint32_t>(static_cast<uint32_t>(!ir_mode_) &
                              iq_ind_u & u3);
    const int npairs = ((nsym64_live + 1) / 2) * inc * static_cast<int>(u3);
    const int nwal = nsym64_live * inc * static_cast<int>(u3);
    const int n_spl = npairs * static_cast<int>(split_u & 1u);
    const int n_sim = nwal * static_cast<int>((split_u ^ 1u) & 1u);
    for (int p = 0; p < n_spl; ++p) {
        const int s = p * 2;
        const uint8_t sI = syms64[static_cast<std::size_t>(s)];
        const uint32_t s2u = static_cast<uint32_t>(s + 1);
        const uint32_t nsymu = static_cast<uint32_t>(nsym64_live);
        const uint32_t have2 = 0u - static_cast<uint32_t>(s2u < nsymu);
        const size_t idx2 =
            static_cast<size_t>(s2u) & static_cast<size_t>(have2);
        const uint8_t sQ = static_cast<uint8_t>(
            static_cast<uint32_t>(syms64[idx2]) & (have2 & 0xFFu));
        const uint8_t enc_sI = walsh_permuter_.Encode_6bit(sI);
        const uint8_t enc_sQ = walsh_permuter_.Encode_6bit(sQ);
        walsh_enc_split(enc_sI, enc_sQ, 64, amp, bp_dst_i(oI, pos, okm),
                        bp_dst_q(oQ, pos, okm));
        pos += 64 * inc;
    }
    for (int s = 0; s < n_sim; ++s) {
        const uint8_t raw_sym6 = syms64[static_cast<std::size_t>(s)];
        const uint8_t enc_sym6 = walsh_permuter_.Encode_6bit(raw_sym6);
        walsh_enc(enc_sym6, 64, amp, bp_dst_i(oI, pos, okm),
                  bp_dst_q(oQ, pos, okm));
        pos += 64 * inc;
    }
    SecureMemory::secureWipe(static_cast<void *>(syms64), sizeof(syms64));
    SecureMemory::secureWipe(static_cast<void *>(syms64_ir), sizeof(syms64_ir));
    SecureMemory::secureWipe(static_cast<void *>(syms64_pl), sizeof(syms64_pl));

    if ((go & 1u) != 0u && pos > 0 &&
        holo_lpi_en_ == HTS_Holo_LPI::SECURE_TRUE) {
        if (HTS_Holo_LPI::Generate_Scalars(holo_lpi_seed_, tx_seq_,
                                           holo_lpi_mix_q8_,
                                           holo_lpi_scalars_) ==
            HTS_Holo_LPI::SECURE_TRUE) {
            const int pc = (pos > 65535) ? 65535 : pos;
            HTS_Holo_LPI::Apply(oI, oQ, static_cast<uint16_t>(pc),
                                holo_lpi_scalars_);
        }
    }

    tx_seq_ += static_cast<uint32_t>(go & 1u);
    return pos;
}
/* BUG-FIX-RETX3: HARQ 연속모드 TX — 프리앰블/헤더 생략 */
int HTS_V400_Dispatcher::Build_Retx(PayloadMode mode, const uint8_t *info,
                                    int ilen, int16_t amp, int16_t *oI,
                                    int16_t *oQ, int max_c) noexcept {
    if (phase_ == RxPhase::RF_SETTLING) {
        return 0;
    }
    if (info == nullptr || oI == nullptr || oQ == nullptr)
        return 0;
    if (ilen < 0 || max_c <= 0)
        return 0;
    // TPC는 외부에서 명시적으로 사용할 때만 적용
    // 기본: caller의 amp 파라미터 사용 (기존 호환)
    tx_amp_ = amp;
    int pos = 0;
    /* Build_Packet가 tx_seq_++ 한 뒤이므로 Retx il은 직전 송신
       시퀀스(tx_seq_-1)와 수신 il(seed_ ^ rx_seq_*0xA5A5A5A5) 정합 */
    const uint32_t tx_seq_prev = (tx_seq_ > 0u) ? (tx_seq_ - 1u) : 0u;
    // [Walsh_Row_Permuter] 재전송 — 직전 PDU tx_seq_prev + 현재 IR RV
    if (!walsh_permuter_.Is_Initialized()) {
        (void)walsh_permuter_.Initialize(
            tx_seq_prev, static_cast<uint8_t>(ir_rv_ & 0xFF));
    } else {
        (void)walsh_permuter_.Update_Key(
            tx_seq_prev, static_cast<uint8_t>(ir_rv_ & 0xFF));
    }
    const uint32_t il = seed_ ^ (tx_seq_prev * 0xA5A5A5A5u);
    const uint32_t mi = static_cast<uint32_t>(mode);
    if (mi > 3u || mi == 0u) {
        return 0;
    }
    const uint32_t u1 = static_cast<uint32_t>(mi == 1u);
    const uint32_t u2 = static_cast<uint32_t>(mi == 2u);
    const uint32_t u3 = static_cast<uint32_t>(mi == 3u);
    const uint32_t u16 = u1 | u2;
    const uint32_t iq_ind_u =
        static_cast<uint32_t>(iq_mode_ == IQ_Mode::IQ_INDEPENDENT);

    uint8_t syms16_ir[FEC_HARQ::NSYM16] = {};
    uint8_t syms16_pl[FEC_HARQ::NSYM16] = {};
    uint8_t syms64_ir[FEC_HARQ::NSYM64] = {};
    uint8_t syms64_pl[FEC_HARQ::NSYM64] = {};
    const int irb = static_cast<int>(ir_mode_);

    const int il_16 = ilen * static_cast<int>(u16);
    const int il_64 = ilen * static_cast<int>(u3);

    const int enc16_ir =
        FEC_HARQ::Encode16_IR(info, il_16, syms16_ir, il, ir_rv_, wb_);
    const int enc16_pl = FEC_HARQ::Encode16(info, il_16, syms16_pl, il, wb_);
    const int enc16 = enc16_ir * irb + enc16_pl * (1 - irb);

    const int enc64_ir = FEC_HARQ::Encode64_IR(info, il_64, syms64_ir, il,
                                               cur_bps64_, ir_rv_, wb_);
    const int enc64_pl =
        FEC_HARQ::Encode64_A(info, il_64, syms64_pl, il, cur_bps64_, wb_);
    const int enc64 = enc64_ir * irb + enc64_pl * (1 - irb);

    // IR/plain 선택 (TPE 비트마스크)
    uint8_t syms16[FEC_HARQ::NSYM16] = {};
    uint8_t syms64[FEC_HARQ::NSYM64] = {};
    const uint32_t ir_mask = 0u - static_cast<uint32_t>(irb);
    const uint32_t pl_mask = ~ir_mask;
    for (int i = 0; i < FEC_HARQ::NSYM16; ++i) {
        syms16[i] = static_cast<uint8_t>(
            (static_cast<uint32_t>(syms16_ir[i]) & ir_mask) |
            (static_cast<uint32_t>(syms16_pl[i]) & pl_mask));
    }
    for (int i = 0; i < FEC_HARQ::NSYM64; ++i) {
        syms64[i] = static_cast<uint8_t>(
            (static_cast<uint32_t>(syms64_ir[i]) & ir_mask) |
            (static_cast<uint32_t>(syms64_pl[i]) & pl_mask));
    }

    const uint32_t bad_enc =
        (u16 & (0u - static_cast<uint32_t>(enc16 <= 0))) |
        (u3 & (0u - static_cast<uint32_t>(enc64 <= 0)));
    if (bad_enc != 0u) {
        SecureMemory::secureWipe(static_cast<void *>(syms16), sizeof(syms16));
        SecureMemory::secureWipe(static_cast<void *>(syms16_ir), sizeof(syms16_ir));
        SecureMemory::secureWipe(static_cast<void *>(syms16_pl), sizeof(syms16_pl));
        SecureMemory::secureWipe(static_cast<void *>(syms64), sizeof(syms64));
        SecureMemory::secureWipe(static_cast<void *>(syms64_ir), sizeof(syms64_ir));
        SecureMemory::secureWipe(static_cast<void *>(syms64_pl), sizeof(syms64_pl));
        return 0;
    }

    const int n16_loop = FEC_HARQ::NSYM16 * static_cast<int>(u16);
    for (int s = 0; s < n16_loop; ++s) {
        const int space = max_c - pos;
        if (space < 16) {
            SecureMemory::secureWipe(static_cast<void *>(syms16),
                                     sizeof(syms16));
            SecureMemory::secureWipe(static_cast<void *>(syms16_ir),
                                     sizeof(syms16_ir));
            SecureMemory::secureWipe(static_cast<void *>(syms16_pl),
                                     sizeof(syms16_pl));
            SecureMemory::secureWipe(static_cast<void *>(syms64),
                                     sizeof(syms64));
            SecureMemory::secureWipe(static_cast<void *>(syms64_ir),
                                     sizeof(syms64_ir));
            SecureMemory::secureWipe(static_cast<void *>(syms64_pl),
                                     sizeof(syms64_pl));
            return 0;
        }
        const uint8_t raw_sym4 = syms16[static_cast<std::size_t>(s)];
        const uint8_t enc_sym4 = walsh_permuter_.Encode_4bit(raw_sym4);
        walsh_enc(enc_sym4, 16, amp, &oI[pos], &oQ[pos]);
        pos += 16;
    }

    const int nsym = cur_nsym64_() * static_cast<int>(u3);
    const uint32_t split_u =
        static_cast<uint32_t>(static_cast<uint32_t>(!ir_mode_) & iq_ind_u &
                              u3);
    const int npairs = ((nsym + 1) / 2) * static_cast<int>(u3);
    const int n_spl = npairs * static_cast<int>(split_u & 1u);
    const int n_sim =
        nsym * static_cast<int>(u3) * static_cast<int>((split_u ^ 1u) & 1u);

    for (int p = 0; p < n_spl; ++p) {
        const int s = p * 2;
        const int space = max_c - pos;
        if (space < 64) {
            SecureMemory::secureWipe(static_cast<void *>(syms16),
                                     sizeof(syms16));
            SecureMemory::secureWipe(static_cast<void *>(syms16_ir),
                                     sizeof(syms16_ir));
            SecureMemory::secureWipe(static_cast<void *>(syms16_pl),
                                     sizeof(syms16_pl));
            SecureMemory::secureWipe(static_cast<void *>(syms64),
                                     sizeof(syms64));
            SecureMemory::secureWipe(static_cast<void *>(syms64_ir),
                                     sizeof(syms64_ir));
            SecureMemory::secureWipe(static_cast<void *>(syms64_pl),
                                     sizeof(syms64_pl));
            return 0;
        }
        const uint8_t sI = syms64[static_cast<std::size_t>(s)];
        const uint32_t s2u = static_cast<uint32_t>(s + 1);
        const uint32_t nsymu = static_cast<uint32_t>(nsym);
        const uint32_t have2 = 0u - static_cast<uint32_t>(s2u < nsymu);
        const size_t idx2 =
            static_cast<size_t>(s2u) & static_cast<size_t>(have2);
        const uint8_t sQ = static_cast<uint8_t>(
            static_cast<uint32_t>(syms64[idx2]) & (have2 & 0xFFu));
        const uint8_t enc_sI = walsh_permuter_.Encode_6bit(sI);
        const uint8_t enc_sQ = walsh_permuter_.Encode_6bit(sQ);
        walsh_enc_split(enc_sI, enc_sQ, 64, amp, &oI[pos], &oQ[pos]);
        pos += 64;
    }
    for (int s = 0; s < n_sim; ++s) {
        const int space = max_c - pos;
        if (space < 64) {
            SecureMemory::secureWipe(static_cast<void *>(syms16),
                                     sizeof(syms16));
            SecureMemory::secureWipe(static_cast<void *>(syms16_ir),
                                     sizeof(syms16_ir));
            SecureMemory::secureWipe(static_cast<void *>(syms16_pl),
                                     sizeof(syms16_pl));
            SecureMemory::secureWipe(static_cast<void *>(syms64),
                                     sizeof(syms64));
            SecureMemory::secureWipe(static_cast<void *>(syms64_ir),
                                     sizeof(syms64_ir));
            SecureMemory::secureWipe(static_cast<void *>(syms64_pl),
                                     sizeof(syms64_pl));
            return 0;
        }
        const uint8_t raw_sym6 = syms64[static_cast<std::size_t>(s)];
        const uint8_t enc_sym6 = walsh_permuter_.Encode_6bit(raw_sym6);
        walsh_enc(enc_sym6, 64, amp, &oI[pos], &oQ[pos]);
        pos += 64;
    }

    SecureMemory::secureWipe(static_cast<void *>(syms16), sizeof(syms16));
    SecureMemory::secureWipe(static_cast<void *>(syms16_ir), sizeof(syms16_ir));
    SecureMemory::secureWipe(static_cast<void *>(syms16_pl), sizeof(syms16_pl));
    SecureMemory::secureWipe(static_cast<void *>(syms64), sizeof(syms64));
    SecureMemory::secureWipe(static_cast<void *>(syms64_ir), sizeof(syms64_ir));
    SecureMemory::secureWipe(static_cast<void *>(syms64_pl), sizeof(syms64_pl));

    if (pos > 0 && holo_lpi_en_ == HTS_Holo_LPI::SECURE_TRUE) {
        if (HTS_Holo_LPI::Generate_Scalars(
                holo_lpi_seed_, tx_seq_prev, holo_lpi_mix_q8_,
                holo_lpi_scalars_) == HTS_Holo_LPI::SECURE_TRUE) {
            const int pc = (pos > 65535) ? 65535 : pos;
            HTS_Holo_LPI::Apply(oI, oQ, static_cast<uint16_t>(pc),
                                holo_lpi_scalars_);
        }
    }

    return pos;
}
void HTS_V400_Dispatcher::psal_commit_align_() noexcept {
    psal_pending_ = false;
    const int commit_off = psal_off_;
    const int carry = 64 - commit_off;
    p0_carry_count_ = carry;
    if (carry > 0) {
        const int src = commit_off + 128;
        for (int j = 0; j < carry; ++j) {
            p0_carry_I_[j] = p0_buf128_I_[src + j];
            p0_carry_Q_[j] = p0_buf128_Q_[src + j];
        }
    }
    pre_phase_ = 1;
    p1_carry_pending_ = (carry > 0) ? 1 : 0;
    p1_tail_collect_rem_ = 0;
    p1_tail_idx_ = 0;
    p1_carry_prefix_ = 0;
    buf_idx_ = 0;
    wait_sync_head_ = 0;
    wait_sync_count_ = 0;
    p0_chip_count_ = 0;
}

void HTS_V400_Dispatcher::phase0_scan_() noexcept {
    int32_t best_e63 = 0, second_e63 = 0;
    int best_off = -1;
    int64_t sum_all = 0;
    int best_dom_row = 63;
    int32_t best_seed_I = 0;
    int32_t best_seed_Q = 0;
#if defined(HTS_DIAG_PRINTF) && !defined(HTS_PHASE0_WALSH_BANK) && \
    !defined(HTS_TARGET_AMI)
    static int s_stage4_scan_idx = 0;
    ++s_stage4_scan_idx;
    const bool diag_this_scan = (s_stage4_scan_idx <= 3);
    int cons_pass_count = 0;
    int best_cons_off = -1;
    int best_cons_r0 = -1;
    int best_cons_r1 = -1;
    int best_cons_r2 = -1;
    int64_t best_cons_e_sum = 0;
    {
        static int s_p0_buf_dump_idx = 0;
        ++s_p0_buf_dump_idx;
        if (s_p0_buf_dump_idx <= 3) {
            std::printf(
                "[BUF#%d] p0_buf128_I snapshot (at scan entry):\n",
                s_p0_buf_dump_idx);
            for (int blk = 0; blk < 3; ++blk) {
                int nz = 0;
                int abs_max = 0;
                int abs_sum = 0;
                for (int i = 0; i < 64; ++i) {
                    const int v = static_cast<int>(p0_buf128_I_[blk * 64 + i]);
                    const int a = (v >= 0) ? v : -v;
                    if (a > 100) {
                        ++nz;
                    }
                    if (a > abs_max) {
                        abs_max = a;
                    }
                    abs_sum += a;
                }
                std::printf("  blk%d: nonzero=%d/64 abs_max=%d abs_avg=%d\n",
                            blk, nz, abs_max, abs_sum / 64);
            }
            std::printf("  buf[0..15]_I:");
            for (int i = 0; i < 16; ++i) {
                std::printf(" %d", static_cast<int>(p0_buf128_I_[i]));
            }
            std::printf("\n");
            std::printf("  buf[64..79]_I:");
            for (int i = 0; i < 16; ++i) {
                std::printf(" %d", static_cast<int>(p0_buf128_I_[64 + i]));
            }
            std::printf("\n");
            std::printf("  buf[128..143]_I:");
            for (int i = 0; i < 16; ++i) {
                std::printf(" %d", static_cast<int>(p0_buf128_I_[128 + i]));
            }
            std::printf("\n");
        }
    }
    if (s_stage4_scan_idx >= 3 && s_stage4_scan_idx <= 5) {
        std::printf("[DEEP-BUF#%d] entire 192 chip I dump:\n",
                    s_stage4_scan_idx);
        for (int b = 0; b < 3; ++b) {
            std::printf("  blk%d I: ", b);
            for (int i = 0; i < 64; ++i) {
                std::printf("%d ",
                            static_cast<int>(p0_buf128_I_[b * 64 + i]));
            }
            std::printf("\n");
        }
        std::printf("[DEEP-BUF#%d] entire 192 chip Q dump:\n",
                    s_stage4_scan_idx);
        for (int b = 0; b < 3; ++b) {
            std::printf("  blk%d Q: ", b);
            for (int i = 0; i < 64; ++i) {
                std::printf("%d ",
                            static_cast<int>(p0_buf128_Q_[b * 64 + i]));
            }
            std::printf("\n");
        }
    }
    if (s_stage4_scan_idx == 3) {
        alignas(32) int32_t diag_T0_I[64];
        alignas(32) int32_t diag_T0_Q[64];
        alignas(32) int32_t diag_T1_I[64];
        alignas(32) int32_t diag_T1_Q[64];
        for (int i = 0; i < 64; ++i) {
            diag_T0_I[i] = static_cast<int32_t>(p0_buf128_I_[i]);
            diag_T0_Q[i] = static_cast<int32_t>(p0_buf128_Q_[i]);
            diag_T1_I[i] = static_cast<int32_t>(p0_buf128_I_[i + 64]);
            diag_T1_Q[i] = static_cast<int32_t>(p0_buf128_Q_[i + 64]);
        }
        fwht_64_complex_inplace_(diag_T0_I, diag_T0_Q);
        fwht_64_complex_inplace_(diag_T1_I, diag_T1_Q);
        std::printf(
            "[DEEP-FWHT] off=0 block 0 FWHT energy (selected rows):\n");
        {
            const int rows[] = {0,  1,  15, 30, 31, 32,
                                33, 47, 62, 63};
            for (unsigned ri = 0; ri < sizeof(rows) / sizeof(rows[0]); ++ri) {
                const int r = rows[ri];
                const int64_t e =
                    static_cast<int64_t>(diag_T0_I[r]) * diag_T0_I[r] +
                    static_cast<int64_t>(diag_T0_Q[r]) * diag_T0_Q[r];
                std::printf("  blk0 row%d energy=%lld\n", r,
                            static_cast<long long>(e));
            }
        }
        std::printf("[DEEP-FWHT] off=0 block 1 FWHT energy:\n");
        {
            const int rows[] = {0,  1,  15, 30, 31, 32,
                                33, 47, 62, 63};
            for (unsigned ri = 0; ri < sizeof(rows) / sizeof(rows[0]); ++ri) {
                const int r = rows[ri];
                const int64_t e =
                    static_cast<int64_t>(diag_T1_I[r]) * diag_T1_I[r] +
                    static_cast<int64_t>(diag_T1_Q[r]) * diag_T1_Q[r];
                std::printf("  blk1 row%d energy=%lld\n", r,
                            static_cast<long long>(e));
            }
        }
        auto print_top3 = [](const int32_t* T_I, const int32_t* T_Q,
                             const char* lbl) noexcept {
            int idx[3] = {-1, -1, -1};
            int64_t vals[3] = {0, 0, 0};
            for (int r = 0; r < 64; ++r) {
                const int64_t e =
                    static_cast<int64_t>(T_I[r]) * T_I[r] +
                    static_cast<int64_t>(T_Q[r]) * T_Q[r];
                if (e > vals[0]) {
                    vals[2] = vals[1];
                    idx[2] = idx[1];
                    vals[1] = vals[0];
                    idx[1] = idx[0];
                    vals[0] = e;
                    idx[0] = r;
                } else if (e > vals[1]) {
                    vals[2] = vals[1];
                    idx[2] = idx[1];
                    vals[1] = e;
                    idx[1] = r;
                } else if (e > vals[2]) {
                    vals[2] = e;
                    idx[2] = r;
                }
            }
            std::printf("  %s top3: row=[%d,%d,%d] e=[%lld,%lld,%lld]\n", lbl,
                        idx[0], idx[1], idx[2],
                        static_cast<long long>(vals[0]),
                        static_cast<long long>(vals[1]),
                        static_cast<long long>(vals[2]));
        };
        print_top3(diag_T0_I, diag_T0_Q, "blk0");
        print_top3(diag_T1_I, diag_T1_Q, "blk1");
    }
#endif

#if defined(HTS_DIAG_PRINTF) && !defined(HTS_PHASE0_WALSH_BANK) && \
    !defined(HTS_TARGET_AMI)
    {
        static int s_p0_full_dump_idx = 0;
        ++s_p0_full_dump_idx;
        if (s_p0_full_dump_idx <= 5) {
            std::printf("[P0-DUMP#%d] p0_buf128_I (192 chips):\n",
                        s_p0_full_dump_idx);
            for (int i = 0; i < 192; ++i) {
                std::printf("%d%s", static_cast<int>(p0_buf128_I_[i]),
                            (i % 16 == 15) ? "\n" : ",");
            }
            std::printf("[P0-DUMP#%d] p0_buf128_Q (192 chips):\n",
                        s_p0_full_dump_idx);
            for (int i = 0; i < 192; ++i) {
                std::printf("%d%s", static_cast<int>(p0_buf128_Q_[i]),
                            (i % 16 == 15) ? "\n" : ",");
            }
        }
    }
#endif

    for (int off = 0; off < 64; ++off) {
        int32_t accum = 0;
        int max_row = 0;
        int32_t seed_I_fwht = 0;
        int32_t seed_Q_fwht = 0;
        int stage4_b1_row = 0;
#if defined(HTS_PHASE0_WALSH_BANK)
        // ── Walsh Bank Phase 0 — Full FWHT + Max Row + per-block T 보존 ──
        // 2-block FWHT 결과를 dominant row 에서 coherent 합산 → Phase 1 과
        // 동일 Walsh 도메인 시드 (walsh63_dot_ 재계산 경로와 분리).
        alignas(4) int32_t T_I[2][64];
        alignas(4) int32_t T_Q[2][64];
        int64_t row_e[64] = {0};
        for (int blk = 0; blk < 2; ++blk) {
            const int base = off + (blk << 6);
            for (int i = 0; i < 64; ++i) {
                T_I[blk][i] = static_cast<int32_t>(p0_buf128_I_[base + i]);
                T_Q[blk][i] = static_cast<int32_t>(p0_buf128_Q_[base + i]);
            }
            fwht_64_complex_inplace_(T_I[blk], T_Q[blk]);
            for (int r = 0; r < 64; ++r) {
                const int64_t e_I =
                    static_cast<int64_t>(T_I[blk][r]) * T_I[blk][r];
                const int64_t e_Q =
                    static_cast<int64_t>(T_Q[blk][r]) * T_Q[blk][r];
                row_e[r] += (e_I + e_Q);
            }
        }
        // Tie-break: row 63 (PRE_SYM0 / k_w63 FWHT) 동점 시 우선
        int64_t row_max_e = row_e[63];
        max_row = 63;
        for (int r = 0; r < 64; ++r) {
            if (r == 63) {
                continue;
            }
            if (row_e[r] > row_max_e) {
                row_max_e = row_e[r];
                max_row = r;
            }
        }
        accum = static_cast<int32_t>(row_max_e >> 16);
        seed_I_fwht = T_I[0][max_row] + T_I[1][max_row];
        seed_Q_fwht = T_Q[0][max_row] + T_Q[1][max_row];
#else
#if defined(HTS_TARGET_AMI)
        // ── AMI: 32-chip × 2 non-coherent per block (기존) ──
        int64_t e_nc = 0;
        for (int blk = 0; blk < 2; ++blk) {
            const int base = off + (blk << 6);
            for (int half = 0; half < 2; ++half) {
                int32_t dI = 0, dQ = 0;
                for (int j = 0; j < 32; ++j) {
                    const int idx = base + (half << 5) + j;
                    const int32_t sI =
                        static_cast<int32_t>(p0_buf128_I_[idx]);
                    const int32_t sQ =
                        static_cast<int32_t>(p0_buf128_Q_[idx]);
                    const int widx = (half << 5) + j;
                    if (k_w63[static_cast<std::size_t>(widx)] > 0) {
                        dI += sI;
                        dQ += sQ;
                    } else {
                        dI -= sI;
                        dQ -= sQ;
                    }
                }
                e_nc += static_cast<int64_t>(dI) * dI +
                        static_cast<int64_t>(dQ) * dQ;
            }
        }
        accum = static_cast<int32_t>(e_nc >> 16);
#else
        // Stage 4b (PS-LTE): 3×64 FWHT 동일 row 일관성 (pre_reps_>1 시 PRE_SYM0 반복)
        // p0_buf128_*_[192]: off+191<192 일 때만 3블록 안전 → off==0 만 4b 전체.
        // 그 외 off 는 기존 2블록 XOR+fallback (슬라이드 OOB 방지).
        alignas(32) int32_t T0_I[64];
        alignas(32) int32_t T0_Q[64];
        alignas(32) int32_t T1_I[64];
        alignas(32) int32_t T1_Q[64];
        alignas(32) int32_t T2_I[64];
        alignas(32) int32_t T2_Q[64];
        int r0 = 63;
        int r1 = 63;
        int r2 = 63;
        int max_row0 = 63;
        int max_row1 = 63;
        for (int i = 0; i < 64; ++i) {
            T0_I[i] = static_cast<int32_t>(p0_buf128_I_[off + i]);
            T0_Q[i] = static_cast<int32_t>(p0_buf128_Q_[off + i]);
        }
        fwht_64_complex_inplace_(T0_I, T0_Q);
        for (int i = 0; i < 64; ++i) {
            T1_I[i] = static_cast<int32_t>(p0_buf128_I_[off + 64 + i]);
            T1_Q[i] = static_cast<int32_t>(p0_buf128_Q_[off + 64 + i]);
        }
        fwht_64_complex_inplace_(T1_I, T1_Q);
        int64_t max_e0 = static_cast<int64_t>(T0_I[63]) * T0_I[63] +
                         static_cast<int64_t>(T0_Q[63]) * T0_Q[63];
        max_row0 = 63;
        for (int r = 0; r < 64; ++r) {
            if (r == 63) {
                continue;
            }
            const int64_t e = static_cast<int64_t>(T0_I[r]) * T0_I[r] +
                              static_cast<int64_t>(T0_Q[r]) * T0_Q[r];
            if (e > max_e0) {
                max_e0 = e;
                max_row0 = r;
            }
        }
        int64_t max_e1 = static_cast<int64_t>(T1_I[63]) * T1_I[63] +
                         static_cast<int64_t>(T1_Q[63]) * T1_Q[63];
        max_row1 = 63;
        for (int r = 0; r < 64; ++r) {
            if (r == 63) {
                continue;
            }
            const int64_t e = static_cast<int64_t>(T1_I[r]) * T1_I[r] +
                              static_cast<int64_t>(T1_Q[r]) * T1_Q[r];
            if (e > max_e1) {
                max_e1 = e;
                max_row1 = r;
            }
        }
        r0 = max_row0;
        r1 = max_row1;
        int64_t max_e2 = 0;
        if ((off + 128 + 63) < 192) {
            for (int i = 0; i < 64; ++i) {
                T2_I[i] = static_cast<int32_t>(p0_buf128_I_[off + 128 + i]);
                T2_Q[i] = static_cast<int32_t>(p0_buf128_Q_[off + 128 + i]);
            }
            fwht_64_complex_inplace_(T2_I, T2_Q);
#if defined(HTS_DIAG_PRINTF) && !defined(HTS_PHASE0_WALSH_BANK) && \
    !defined(HTS_TARGET_AMI)
            {
                static int s_p0_fwht_dump_idx = 0;
                // PROMPT 39: 전역 한도가 S8 이전에 고갈되지 않도록 상향 (DIAG 전용)
                if (off == 0 && s_p0_fwht_dump_idx < 5000) {
                    ++s_p0_fwht_dump_idx;
                    auto print_top5 = [](const char *label, const int32_t *I,
                                         const int32_t *Q) noexcept {
                        int64_t e[64];
                        for (int m = 0; m < 64; ++m) {
                            e[m] = static_cast<int64_t>(I[m]) * I[m] +
                                   static_cast<int64_t>(Q[m]) * Q[m];
                        }
                        int top_bin[5] = {0, 0, 0, 0, 0};
                        int64_t top_e[5] = {INT64_MIN, INT64_MIN, INT64_MIN,
                                            INT64_MIN, INT64_MIN};
                        for (int m = 0; m < 64; ++m) {
                            for (int k = 0; k < 5; ++k) {
                                if (e[m] > top_e[k]) {
                                    for (int j = 4; j > k; --j) {
                                        top_e[j] = top_e[j - 1];
                                        top_bin[j] = top_bin[j - 1];
                                    }
                                    top_e[k] = e[m];
                                    top_bin[k] = m;
                                    break;
                                }
                            }
                        }
                        std::printf("[P0-FWHT-RAW] %s: ", label);
                        for (int k = 0; k < 5; ++k) {
                            std::printf(
                                "bin%d=%lld ", top_bin[k],
                                static_cast<long long>(top_e[k]));
                        }
                        std::printf("\n");
                    };
                    print_top5("block0", T0_I, T0_Q);
                    print_top5("block1", T1_I, T1_Q);
                    print_top5("block2", T2_I, T2_Q);
                    {
                        auto block_total_e = [](const int32_t *I,
                                                const int32_t *Q) noexcept
                            -> int64_t {
                            int64_t s = 0;
                            for (int m = 0; m < 64; ++m) {
                                s += static_cast<int64_t>(I[m]) * I[m] +
                                     static_cast<int64_t>(Q[m]) * Q[m];
                            }
                            return s;
                        };
                        auto row_energy = [](const int32_t *I, const int32_t *Q,
                                             int r) noexcept -> int64_t {
                            return static_cast<int64_t>(I[r]) * I[r] +
                                   static_cast<int64_t>(Q[r]) * Q[r];
                        };
                        std::printf(
                            "[P0-BLOCK-TOTAL] block=0 total_e=%lld\n",
                            static_cast<long long>(
                                block_total_e(T0_I, T0_Q)));
                        std::printf(
                            "[P0-BLOCK-TOTAL] block=1 total_e=%lld\n",
                            static_cast<long long>(
                                block_total_e(T1_I, T1_Q)));
                        std::printf(
                            "[P0-BLOCK-TOTAL] block=2 total_e=%lld\n",
                            static_cast<long long>(
                                block_total_e(T2_I, T2_Q)));
                        for (int b = 0; b < 3; ++b) {
                            const int32_t *Ib =
                                (b == 0) ? T0_I : ((b == 1) ? T1_I : T2_I);
                            const int32_t *Qb =
                                (b == 0) ? T0_Q : ((b == 1) ? T1_Q : T2_Q);
                            std::printf(
                                "[P0-PINPOINT] block=%d e_bin0=%lld "
                                "e_bin6=%lld e_bin63=%lld\n",
                                b,
                                static_cast<long long>(
                                    row_energy(Ib, Qb, 0)),
                                static_cast<long long>(
                                    row_energy(Ib, Qb, 6)),
                                static_cast<long long>(
                                    row_energy(Ib, Qb, 63)));
                        }
                    }
                }
            }
#endif
            max_e2 = static_cast<int64_t>(T2_I[63]) * T2_I[63] +
                     static_cast<int64_t>(T2_Q[63]) * T2_Q[63];
            r2 = 63;
            for (int r = 0; r < 64; ++r) {
                if (r == 63) {
                    continue;
                }
                const int64_t e = static_cast<int64_t>(T2_I[r]) * T2_I[r] +
                                  static_cast<int64_t>(T2_Q[r]) * T2_Q[r];
                if (e > max_e2) {
                    max_e2 = e;
                    r2 = r;
                }
            }
            const bool cons_ok = (r0 == r1) && (r1 == r2);
            int32_t accum_cohr = 0;
            if (cons_ok) {
                // 기존 2블록 (e0+e1)>>16 판정 스케일에 맞춤: 동일 E면 2E/65536 ≡
                // (2/3)·(3E)/65536
                const int64_t e_scaled =
                    ((max_e0 + max_e1 + max_e2) * 2) / 3;
                accum_cohr = static_cast<int32_t>(e_scaled >> 16);
            } else {
                const int64_t e63_0 =
                    static_cast<int64_t>(T0_I[63]) * T0_I[63] +
                    static_cast<int64_t>(T0_Q[63]) * T0_Q[63];
                const int64_t e63_1 =
                    static_cast<int64_t>(T1_I[63]) * T1_I[63] +
                    static_cast<int64_t>(T1_Q[63]) * T1_Q[63];
                const int64_t e63_2 =
                    static_cast<int64_t>(T2_I[63]) * T2_I[63] +
                    static_cast<int64_t>(T2_Q[63]) * T2_Q[63];
                const int64_t e63_scaled = ((e63_0 + e63_1 + e63_2) * 2) / 3;
                accum_cohr = static_cast<int32_t>(e63_scaled >> 16);
            }
            // Stage 5: 8×8 non-coh (row 7) — 고 CFO에서 coherent 대비 이득
            const int64_t e8x8_0 =
                energy_8x8_noncoh_row7_(&p0_buf128_I_[off],
                                         &p0_buf128_Q_[off]);
            const int64_t e8x8_1 =
                energy_8x8_noncoh_row7_(&p0_buf128_I_[off + 64],
                                         &p0_buf128_Q_[off + 64]);
            const int64_t e8x8_2 =
                energy_8x8_noncoh_row7_(&p0_buf128_I_[off + 128],
                                         &p0_buf128_Q_[off + 128]);
            const int64_t e8x8_sum = e8x8_0 + e8x8_1 + e8x8_2;
            // >>14: 8×8 합을 64칩 coherent(3블록) 스케일에 근사 정합
            const int32_t accum_8x8 =
                static_cast<int32_t>(((e8x8_sum * 2) / 3) >> 14);
            accum = (accum_8x8 > accum_cohr) ? accum_8x8 : accum_cohr;
#if defined(HTS_DIAG_PRINTF) && !defined(HTS_PHASE0_WALSH_BANK) && \
    !defined(HTS_TARGET_AMI)
            if (diag_this_scan) {
                if (cons_ok) {
                    ++cons_pass_count;
                    const int64_t e_triple = max_e0 + max_e1 + max_e2;
                    if (best_cons_off < 0 || e_triple > best_cons_e_sum) {
                        best_cons_e_sum = e_triple;
                        best_cons_off = off;
                        best_cons_r0 = r0;
                        best_cons_r1 = r1;
                        best_cons_r2 = r2;
                    }
                }
            }
            {
                static int s_stage5_diag = 0;
                if (off == 0 && s_stage5_diag < 5) {
                    ++s_stage5_diag;
                    std::printf(
                        "[STAGE5] off=0 cons=%d e_cohr=%d e_8x8_scaled=%d "
                        "accum=%d\n",
                        cons_ok ? 1 : 0, accum_cohr, accum_8x8, accum);
                }
            }
#endif
        } else {
            // 2블록 레거시: XOR=63 (PRE_SYM0/1) + row63 fallback
            int64_t max_e1b = static_cast<int64_t>(T1_I[0]) * T1_I[0] +
                              static_cast<int64_t>(T1_Q[0]) * T1_Q[0];
            max_row1 = 0;
            for (int r = 0; r < 64; ++r) {
                if (r == 0) {
                    continue;
                }
                const int64_t e = static_cast<int64_t>(T1_I[r]) * T1_I[r] +
                                  static_cast<int64_t>(T1_Q[r]) * T1_Q[r];
                if (e > max_e1b) {
                    max_e1b = e;
                    max_row1 = r;
                }
            }
            r1 = max_row1;
            r2 = max_row1;
            const bool xor_ok = ((max_row0 ^ max_row1) == 63);
            if (xor_ok) {
                const int64_t e_sum = max_e0 + max_e1b;
                accum = static_cast<int32_t>(e_sum >> 16);
            } else {
                const int64_t e63_0 =
                    static_cast<int64_t>(T0_I[63]) * T0_I[63] +
                    static_cast<int64_t>(T0_Q[63]) * T0_Q[63];
                const int64_t e63_1 =
                    static_cast<int64_t>(T1_I[63]) * T1_I[63] +
                    static_cast<int64_t>(T1_Q[63]) * T1_Q[63];
                accum = static_cast<int32_t>((e63_0 + e63_1) >> 16);
            }
        }
#if defined(HTS_DIAG_PRINTF) && !defined(HTS_PHASE0_WALSH_BANK) && \
    !defined(HTS_TARGET_AMI)
        {
            static int s_stage4b_diag_line = 0;
            if ((off == 0 || off == 32 || off == 63) &&
                s_stage4b_diag_line < 30) {
                ++s_stage4b_diag_line;
                const bool full3 = ((off + 128 + 63) < 192);
                const int cons_show =
                    full3 ? (((r0 == r1) && (r1 == r2)) ? 1 : 0) : -1;
                std::printf(
                    "[STAGE4B] off=%d full3=%d r0=%d r1=%d r2=%d cons=%d "
                    "accum=%d\n",
                    off, full3 ? 1 : 0, r0, r1, r2, cons_show, accum);
            }
        }
#endif
        max_row = max_row0;
        stage4_b1_row = max_row1;
#endif
#endif
        sum_all += static_cast<int64_t>(accum);
        if (accum > best_e63) {
            second_e63 = best_e63;
            best_e63 = accum;
            best_off = off;
#if defined(HTS_PHASE0_WALSH_BANK)
            best_dom_row = max_row;
            best_seed_I = seed_I_fwht;
            best_seed_Q = seed_Q_fwht;
#if defined(HTS_DIAG_PRINTF)
            std::printf(
                "[P0-ROW-NEW] off=%d dom_row=%d max_e=%d seed=(%d,%d)\n",
                off, max_row, accum, seed_I_fwht, seed_Q_fwht);
#endif
#elif !defined(HTS_TARGET_AMI)
            // Fix A1 (V28): 3-way 일치 시 r0, 아니면 r2.
            // Fix A3 (V34): block-2 FWHT 에너지에서 coherent pair 순차 excision
            // (bin63 하드코딩 없음). pair1=(top1,top2) → pair2=(top3,top4) 검사,
            // 2-level jam이면 top5, 1-level이면 top3를 preamble 후보로 채택.
            const bool full3blk_dom = ((off + 128 + 63) < 192);
            int best_dom_pick = (r0 == r1 && r1 == r2) ? r0 : r2;
            if (full3blk_dom) {
                int64_t e_b2[64];
                for (int m = 0; m < 64; ++m) {
                    e_b2[m] = static_cast<int64_t>(T2_I[m]) * T2_I[m] +
                              static_cast<int64_t>(T2_Q[m]) * T2_Q[m];
                }
                int t1_bin = 0;
                int64_t t1_e = 0;
                find_argmax_64(e_b2, &t1_bin, &t1_e);
                int t2_bin = 0;
                int64_t t2_e = 0;
                int final_pick = best_dom_pick;
                int a3_level = 0;
                int t3_bin_dbg = -1;
                int t4_bin_dbg = -1;
                int t5_bin_dbg = -1;
                int64_t t3_e_dbg = -1;
                int64_t t4_e_dbg = -1;
                int64_t t5_e_dbg = -1;
                if (t1_e > 0) {
                    const int64_t t1_saved = e_b2[t1_bin];
                    e_b2[t1_bin] = 0;
                    find_argmax_64(e_b2, &t2_bin, &t2_e);
                    const bool pair1_coherent =
                        (t2_e >= (t1_e - (t1_e >> 4))) &&
                        (t1_bin != t2_bin);
                    if (pair1_coherent) {
                        const int64_t t2_saved = e_b2[t2_bin];
                        e_b2[t2_bin] = 0;
                        int t3_bin = 0;
                        int64_t t3_e = 0;
                        find_argmax_64(e_b2, &t3_bin, &t3_e);
                        t3_bin_dbg = t3_bin;
                        t3_e_dbg = t3_e;
                        const int64_t t3_saved = e_b2[t3_bin];
                        e_b2[t3_bin] = 0;
                        int t4_bin = 0;
                        int64_t t4_e = 0;
                        find_argmax_64(e_b2, &t4_bin, &t4_e);
                        t4_bin_dbg = t4_bin;
                        t4_e_dbg = t4_e;
                        const bool pair2_coherent =
                            (t3_e > 0) &&
                            (t4_e >= (t3_e - (t3_e >> 4))) &&
                            (t3_bin != t4_bin);
                        if (pair2_coherent) {
                            e_b2[t4_bin] = 0;
                            int t5_bin = 0;
                            int64_t t5_e = 0;
                            find_argmax_64(e_b2, &t5_bin, &t5_e);
                            t5_bin_dbg = t5_bin;
                            t5_e_dbg = t5_e;
                            final_pick = t5_bin;
                            a3_level = 2;
                            e_b2[t3_bin] = t3_saved;
                            e_b2[t2_bin] = t2_saved;
                        } else {
                            final_pick = t3_bin;
                            a3_level = 1;
                            e_b2[t3_bin] = t3_saved;
                            e_b2[t2_bin] = t2_saved;
                        }
                    }
                    e_b2[t1_bin] = t1_saved;
                }
                best_dom_pick = final_pick;
#if defined(HTS_DIAG_PRINTF)
                if (a3_level > 0) {
                    static int s_fixa3_dump = 0;
                    if (s_fixa3_dump < 50) {
                        ++s_fixa3_dump;
                        std::printf(
                            "[FIXA3] off=%d r0=%d r1=%d r2=%d level=%d "
                            "t1=(%d,%lld) t2=(%d,%lld) t3=(%d,%lld) t4=(%d,%lld) "
                            "t5=(%d,%lld) pick=%d (A1=%d)\n",
                            off, r0, r1, r2, a3_level, t1_bin,
                            static_cast<long long>(t1_e), t2_bin,
                            static_cast<long long>(t2_e), t3_bin_dbg,
                            static_cast<long long>(t3_e_dbg), t4_bin_dbg,
                            static_cast<long long>(t4_e_dbg), t5_bin_dbg,
                            static_cast<long long>(t5_e_dbg), final_pick,
                            (r0 == r1 && r1 == r2) ? r0 : r2);
                    }
                }
#endif
            }
            best_dom_row = best_dom_pick;
#if defined(HTS_DIAG_PRINTF)
            std::printf(
                "[STAGE4-P0] off=%d r0=%d r1=%d r2=%d xor_legacy=%d accum=%d "
                "bdr=%d\n",
                off, max_row, stage4_b1_row, r2,
                (max_row ^ stage4_b1_row), accum, best_dom_row);
#endif
#endif
        } else if (accum > second_e63) {
            second_e63 = accum;
        }
    }

    const int64_t sum_others = sum_all - static_cast<int64_t>(best_e63);

    // r_avg >= 5: (best<<6)-best >= (so<<3)+(so<<1)+(so) = so*5...
    // 아니, 5 = (1<<2)+1: so*5 = (so<<2)+so
    // r_avg >= 5 ↔ best*63 >= so*5
    const int64_t best_x63 = (static_cast<int64_t>(best_e63) << 6) -
                              static_cast<int64_t>(best_e63);
    const int64_t so_x5    = (sum_others << 2) + sum_others;
    const bool r_avg_ok = (sum_others <= 0) || (best_x63 >= so_x5);

    // r_avg >= 8 판정 (sep_ok 활성화용, 곱셈 0)
    // best*63 >= so*8 ↔ best_x63 >= so<<3
    const bool r_avg_high = (sum_others <= 0) ||
        (best_x63 >= (sum_others << 3));

    // 적응형 sep_ok: r_avg≥8일 때만 alias 방어
    const int64_t best_x4 = static_cast<int64_t>(best_e63) << 2;
    const int64_t sec_x5  = (static_cast<int64_t>(second_e63) << 2) +
                             static_cast<int64_t>(second_e63);
    const bool sep_ok = (!r_avg_high) ||
        (second_e63 == 0) || (best_x4 > sec_x5);

    // 적응형 e63_min: noise_floor × 5 (하한 5000), amp×38 하한 (TPC tx_amp_ 연동)
    const int32_t avg_others =
        (sum_others > 0)
            ? static_cast<int32_t>((sum_others * 1040LL) >> 16)
            : 0;
    // avg_others * 5 = (avg<<2)+avg [SHIFT]
    const int32_t adaptive_min =
        (avg_others > 0)
            ? static_cast<int32_t>((static_cast<int64_t>(avg_others) << 2) +
                                    avg_others)
            : 5000;
    // 적응형: AMI amp×19 / PS-LTE amp×38 (shift 조합)
    const int32_t amp32 = static_cast<int32_t>(tx_amp_);
#if defined(HTS_TARGET_AMI)
    // AMI 32-chip × 4 non-coherent: clean peak 약 1/2 → amp × 19
    const int32_t k_E63_ALIGN_MIN =
        (amp32 << 4) + (amp32 << 1) + amp32;  // amp × 19
#else
    // PS-LTE 64-chip × 2 non-coherent: 기존 amp × 38
    const int32_t k_E63_ALIGN_MIN =
        (amp32 << 5) + (amp32 << 2) + (amp32 << 1);  // amp × 38
#endif
    const int32_t e63_min =
        (adaptive_min > k_E63_ALIGN_MIN) ? adaptive_min : k_E63_ALIGN_MIN;

    const bool pass = (best_off >= 0 && r_avg_ok &&
                       best_e63 >= e63_min && sep_ok);

#if defined(HTS_DIAG_PRINTF) && !defined(HTS_PHASE0_WALSH_BANK) && \
    !defined(HTS_TARGET_AMI)
    if (diag_this_scan) {
        std::printf(
            "[STAGE4-SCAN#%d] cons_pass=%d best_cons_off=%d r0=%d r1=%d r2=%d "
            "best_cons_esum>>16=%lld final_best_off=%d best_e63=%d second_e63=%d\n",
            s_stage4_scan_idx, cons_pass_count, best_cons_off, best_cons_r0,
            best_cons_r1, best_cons_r2,
            static_cast<long long>(
                (best_cons_e_sum > 0) ? (best_cons_e_sum >> 16) : 0LL),
            best_off, static_cast<int>(best_e63),
            static_cast<int>(second_e63));
        std::printf(
            "[STAGE4-JUDGE#%d] best_off=%d best_e63=%d e63_min=%d "
            "r_avg_ok=%d sep_ok=%d pass=%d sum_others=%lld\n",
            s_stage4_scan_idx, best_off, static_cast<int>(best_e63), e63_min,
            r_avg_ok ? 1 : 0, sep_ok ? 1 : 0, pass ? 1 : 0,
            static_cast<long long>(sum_others));
    }
#endif

#if defined(HTS_DIAG_PRINTF) && defined(HTS_PHASE0_WALSH_BANK)
    // ─── DIAG-SCAN-STAT: scan pass/fail 누적 (로직 무변경) ───
    {
        static int s_scan_total = 0;
        static int s_scan_pass = 0;
        static int s_scan_fail = 0;
        ++s_scan_total;
        if (pass) {
            ++s_scan_pass;
        } else {
            ++s_scan_fail;
        }
        if (s_scan_total > 0 && (s_scan_total % 20) == 0) {
            const int ratio_pct = (s_scan_pass * 100) / s_scan_total;
            std::printf(
                "[DIAG-SCAN-STAT] total=%d pass=%d fail=%d pass_ratio=%d%%\n",
                s_scan_total, s_scan_pass, s_scan_fail, ratio_pct);
        }
    }
#endif

#if defined(HTS_DIAG_PRINTF) && defined(HTS_PHASE0_WALSH_BANK)
    // ─── DIAG-SCAN-FULL: pass 확정 후에만 (빈 버퍼 소모 방지), 최대 2회 ───
    {
        static int s_scan_full_count = 0;
        if (pass && s_scan_full_count < 2) {
            ++s_scan_full_count;
            std::printf("[DIAG-SCAN-FULL] dump#%d — all off scan result\n",
                        s_scan_full_count);
            for (int off = 0; off < 64; ++off) {
                alignas(4) int32_t T_I[2][64];
                alignas(4) int32_t T_Q[2][64];
                int64_t row_e[64] = {0};
                for (int blk = 0; blk < 2; ++blk) {
                    const int base = off + (blk << 6);
                    for (int i = 0; i < 64; ++i) {
                        T_I[blk][i] = static_cast<int32_t>(
                            p0_buf128_I_[base + i]);
                        T_Q[blk][i] = static_cast<int32_t>(
                            p0_buf128_Q_[base + i]);
                    }
                    fwht_64_complex_inplace_(T_I[blk], T_Q[blk]);
                    for (int r = 0; r < 64; ++r) {
                        row_e[r] +=
                            static_cast<int64_t>(T_I[blk][r]) * T_I[blk][r] +
                            static_cast<int64_t>(T_Q[blk][r]) * T_Q[blk][r];
                    }
                }
                int64_t max_e = row_e[0];
                int max_r = 0;
                for (int r = 1; r < 64; ++r) {
                    if (row_e[r] > max_e) {
                        max_e = row_e[r];
                        max_r = r;
                    }
                }
                const int64_t e63 = row_e[63];
                if ((off & 3) == 0) {
                    std::printf(
                        "  off=%2d row_max=%2d e_max=%10lld e63=%10lld\n",
                        off, max_r,
                        static_cast<long long>(max_e >> 16),
                        static_cast<long long>(e63 >> 16));
                }
            }
        }
    }
#endif

#if defined(HTS_DIAG_PRINTF)
    {
        const int32_t r_avg =
            (avg_others > 0)
                ? static_cast<int32_t>(static_cast<int64_t>(best_e63) /
                                       static_cast<int64_t>(avg_others))
                : 999;
        const int32_t r_sep =
            (second_e63 > 0)
                ? static_cast<int32_t>(static_cast<int64_t>(best_e63) /
                                       static_cast<int64_t>(second_e63))
                : 999;
        std::printf("[P0-SCAN] off=%d e63=%d avg_o=%d r_avg=%d r_sep=%d emin=%d sep=%d\n",
                    best_off, best_e63, avg_others, r_avg, r_sep,
                    e63_min, static_cast<int>(sep_ok));
    }
#endif

    if (pass) {
#if defined(HTS_DIAG_PRINTF) && defined(HTS_PHASE0_WALSH_BANK)
        // ─── DIAG-BUF: buf128 실측 dump (Phase 0 detect 직전, 로직 무변경) ───
        {
            static int s_buf_dump_count = 0;
            if (s_buf_dump_count < 3) {
                ++s_buf_dump_count;
                std::printf(
                    "[DIAG-BUF] dump#%d best_off=%d best_dom_row=%d best_e63=%d\n",
                    s_buf_dump_count, best_off, best_dom_row, best_e63);
                std::printf("[DIAG-BUF] buf[0..15]_I:");
                for (int i = 0; i < 16; ++i) {
                    std::printf(" %d", static_cast<int>(p0_buf128_I_[i]));
                }
                std::printf("\n");
                std::printf("[DIAG-BUF] buf[0..15]_Q:");
                for (int i = 0; i < 16; ++i) {
                    std::printf(" %d", static_cast<int>(p0_buf128_Q_[i]));
                }
                std::printf("\n");
                std::printf("[DIAG-BUF] buf[16..31]_I:");
                for (int i = 16; i < 32; ++i) {
                    std::printf(" %d", static_cast<int>(p0_buf128_I_[i]));
                }
                std::printf("\n");
                if (best_off >= 0) {
                    std::printf("[DIAG-BUF] buf[off+0..+15]_I (off=%d):",
                                best_off);
                    for (int i = 0; i < 16; ++i) {
                        std::printf(" %d",
                                    static_cast<int>(
                                        p0_buf128_I_[best_off + i]));
                    }
                    std::printf("\n");
                    std::printf("[DIAG-BUF] buf[off+0..+15]_Q:");
                    for (int i = 0; i < 16; ++i) {
                        std::printf(" %d",
                                    static_cast<int>(
                                        p0_buf128_Q_[best_off + i]));
                    }
                    std::printf("\n");
                    std::printf("[DIAG-BUF] buf[off+64..+79]_I:");
                    for (int i = 0; i < 16; ++i) {
                        std::printf(" %d",
                                    static_cast<int>(
                                        p0_buf128_I_[best_off + 64 + i]));
                    }
                    std::printf("\n");
                    std::printf(
                        "[DIAG-BUF] buf[off+128..+143]_I (header?, <192):");
                    for (int i = 0; i < 16; ++i) {
                        const int idx = best_off + 128 + i;
                        if (idx >= 192) {
                            std::printf(" NA");
                        } else {
                            std::printf(" %d",
                                        static_cast<int>(p0_buf128_I_[idx]));
                        }
                    }
                    std::printf("\n");
                }
                std::printf(
                    "[DIAG-BUF] k_w63[0..15] expected (assume amp=1000):");
                for (int i = 0; i < 16; ++i) {
                    std::printf(" %d",
                                static_cast<int>(k_w63[static_cast<std::size_t>(
                                    i)]) * 1000);
                }
                std::printf("\n");
            }
        }
#endif
        dominant_row_ = static_cast<uint8_t>(best_dom_row);
#if defined(HTS_DIAG_PRINTF) && !defined(HTS_PHASE0_WALSH_BANK) && \
    !defined(HTS_TARGET_AMI)
        std::printf(
            "[STAGE4-P0] lock dominant_row=%u off=%d (seed=walsh63 non-coh)\n",
            static_cast<unsigned>(dominant_row_), best_off);
#endif
        {
            int32_t seed_dot_I = 0, seed_dot_Q = 0;
#if defined(HTS_PHASE0_WALSH_BANK)
            seed_dot_I = best_seed_I;
            seed_dot_Q = best_seed_Q;
#elif defined(HTS_TARGET_AMI)
            for (int blk = 0; blk < 2; ++blk) {
                int32_t di = 0, dq = 0;
                walsh63_dot_(&p0_buf128_I_[best_off + (blk << 6)],
                             &p0_buf128_Q_[best_off + (blk << 6)],
                             di, dq);
                seed_dot_I += di;
                seed_dot_Q += dq;
            }
#else
            // Stage 4: 스캔은 XOR=63 지문만; 시드는 블록별 walsh63 non-coherent 합
            for (int blk = 0; blk < 2; ++blk) {
                int32_t di = 0, dq = 0;
                walsh63_dot_(&p0_buf128_I_[best_off + (blk << 6)],
                             &p0_buf128_Q_[best_off + (blk << 6)],
                             di, dq);
                seed_dot_I += di;
                seed_dot_Q += dq;
            }
#endif
            est_I_ = seed_dot_I;
            est_Q_ = seed_dot_Q;
            est_count_ = 2;
            // B-2: est seed 확정 직후 mag/recip 즉시 계산
            //   이후 on_sym_ 심볼당 정규화 derotation 사용
            update_derot_shift_from_est_();
            // CFO 는 Walsh 도메인에서 처리 예정 — MCE 철거 (Stage 1)
            // 프리앰블 AGC: P0 피크에서 수신 진폭 측정
            {
                int32_t mag_sum = 0;
                for (int j = 0; j < 64; ++j) {
                    const int32_t ai = p0_buf128_I_[best_off + j];
                    const int32_t aq = p0_buf128_Q_[best_off + j];
                    const int32_t si = ai >> 31;
                    mag_sum += (ai ^ si) - si;
                    const int32_t sq = aq >> 31;
                    mag_sum += (aq ^ sq) - sq;
                }
                const int32_t peak_avg = mag_sum >> 6;
                pre_agc_.Set_From_Peak(peak_avg);
            }
#if defined(HTS_DIAG_PRINTF)
            std::printf("[P0-SEED] dot=(%d,%d) est=(%d,%d) n=%d\n",
                        seed_dot_I, seed_dot_Q, est_I_, est_Q_,
                        est_count_);
#endif
#if defined(HTS_PHASE0_WALSH_BANK) && defined(HTS_DIAG_PRINTF)
            std::printf("[P0-WBANK] commit_off=%d dominant_row=%u seed=(%d,%d)\n",
                        best_off, static_cast<unsigned>(dominant_row_),
                        seed_dot_I, seed_dot_Q);
#endif
        }
#if defined(HTS_DIAG_PRINTF)
        std::printf("[P0-ALIGNED] off=%d carry=%d\n",
                    best_off, 64 - best_off);
#endif
        psal_off_ = best_off;
        psal_e63_ = best_e63;
        psal_commit_align_();
    } else {
#if defined(HTS_DIAG_PRINTF) && defined(HTS_PHASE0_WALSH_BANK)
        {
            static int s_shift_count = 0;
            ++s_shift_count;
            if (s_shift_count <= 3) {
                std::printf(
                    "[DIAG-SHIFT] count=%d before_shift buf[128..143]_I:",
                    s_shift_count);
                for (int i = 0; i < 16; ++i) {
                    std::printf(" %d",
                                static_cast<int>(p0_buf128_I_[128 + i]));
                }
                std::printf("\n");
            }
        }
#endif
        std::memcpy(p0_buf128_I_, p0_buf128_I_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        std::memcpy(p0_buf128_Q_, p0_buf128_Q_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        p0_chip_count_ = 64;
    }
}
void HTS_V400_Dispatcher::Feed_Chip(int16_t rx_I, int16_t rx_Q) noexcept {
#if defined(HTS_DIAG_PRINTF) && !defined(HTS_PHASE0_WALSH_BANK) && \
    !defined(HTS_TARGET_AMI)
    static int s_lab_feed_chip_idx = 0;
    ++s_lab_feed_chip_idx;
    if (s_lab_feed_chip_idx <= 300) {
        std::printf("[HARNESS] chip#%d rx_I=%d rx_Q=%d\n",
                    s_lab_feed_chip_idx, static_cast<int>(rx_I),
                    static_cast<int>(rx_Q));
    }
#endif
#if defined(HTS_DIAG_PRINTF) && defined(HTS_PHASE0_WALSH_BANK)
    {
        static int s_feed_call_count = 0;
        static int s_last_logged_count = -1;
        ++s_feed_call_count;
        if (s_feed_call_count < 500 &&
            (s_feed_call_count - s_last_logged_count) >= 50) {
            s_last_logged_count = s_feed_call_count;
            std::printf(
                "[DIAG-FEED] call#%d phase=%d pre_phase=%d p0_count=%d "
                "carry_pend=%d buf_idx=%d rx_I=%d rx_Q=%d\n",
                s_feed_call_count,
                static_cast<int>(phase_),
                pre_phase_,
                p0_chip_count_,
                p1_carry_pending_,
                buf_idx_,
                static_cast<int>(rx_I), static_cast<int>(rx_Q));
        }
    }
#endif
    int16_t chip_I = rx_I;
    int16_t chip_Q = rx_Q;
    // DC 제거 IIR: α=1/128 (shift 기반, 곱셈 0)
    // dc_est = dc_est - (dc_est >> 7) + (chip >> 7)
    dc_est_I_ = dc_est_I_ - (dc_est_I_ >> 7) +
                (static_cast<int32_t>(chip_I) >> 7);
    dc_est_Q_ = dc_est_Q_ - (dc_est_Q_ >> 7) +
                (static_cast<int32_t>(chip_Q) >> 7);
    chip_I = static_cast<int16_t>(static_cast<int32_t>(chip_I) - dc_est_I_);
    chip_Q = static_cast<int16_t>(static_cast<int32_t>(chip_Q) - dc_est_Q_);
#if defined(HTS_DIAG_PRINTF) && !defined(HTS_PHASE0_WALSH_BANK) && \
    !defined(HTS_TARGET_AMI)
    if (s_lab_feed_chip_idx >= 250 && s_lab_feed_chip_idx <= 450 &&
        (s_lab_feed_chip_idx % 16) == 0) {
        std::printf(
            "[DC-AGC] chip#%d rx_I=%d after_DC=%d dc_est_I=%d dc_est_Q=%d\n",
            s_lab_feed_chip_idx, static_cast<int>(rx_I),
            static_cast<int>(chip_I), static_cast<int>(dc_est_I_),
            static_cast<int>(dc_est_Q_));
    }
#endif
    // CFO 는 Walsh 도메인에서 처리 예정 (시간도메인 Apply 철거, Stage 1)
    // 프리앰블 AGC
    pre_agc_.Apply(chip_I, chip_Q);
    apply_holo_lpi_inverse_rx_chip_(chip_I, chip_Q, rx_seq_);
    if (phase_ == RxPhase::RF_SETTLING) {
        (void)chip_I;
        (void)chip_Q;
        const uint32_t nz =
            static_cast<uint32_t>(rf_settle_chips_remaining_ > 0);
        rf_settle_chips_remaining_ -= static_cast<int>(nz);
        if (rf_settle_chips_remaining_ == 0) {
            (void)set_phase_(RxPhase::WAIT_SYNC);
        }
        return;
    }
    if (phase_ == RxPhase::WAIT_SYNC) {
        if (pre_phase_ == 0) {
            p0_buf128_I_[p0_chip_count_] = chip_I;
            p0_buf128_Q_[p0_chip_count_] = chip_Q;
            ++p0_chip_count_;
            if (p0_chip_count_ < 192)
                return;
#if defined(HTS_DIAG_PRINTF) && defined(HTS_PHASE0_WALSH_BANK)
            {
                static int s_fullbuf_count = 0;
                ++s_fullbuf_count;
                if (s_fullbuf_count <= 5) {
                    std::printf(
                        "[DIAG-FULLBUF] #%d (192 chip reached, about to scan)\n",
                        s_fullbuf_count);
                    for (int b = 0; b < 3; ++b) {
                        int nz = 0;
                        int mag_max = 0;
                        for (int i = 0; i < 64; ++i) {
                            const int mag =
                                (p0_buf128_I_[b * 64 + i] > 0)
                                    ? static_cast<int>(p0_buf128_I_[b * 64 + i])
                                    : -static_cast<int>(
                                          p0_buf128_I_[b * 64 + i]);
                            if (mag > 100) {
                                ++nz;
                            }
                            if (mag > mag_max) {
                                mag_max = mag;
                            }
                        }
                        std::printf("  blk%d: nonzero=%d/64 mag_max=%d\n", b,
                                    nz, mag_max);
                    }
                }
            }
#endif
            phase0_scan_();
            return;
        }
        // ── Phase 1: carry + Walsh-63/0 dual dot ──
        if (p1_tail_collect_rem_ > 0) {
            p0_carry_I_[p1_tail_idx_] = chip_I;
            p0_carry_Q_[p1_tail_idx_] = chip_Q;
            ++p1_tail_idx_;
            if (p1_tail_idx_ < p1_tail_collect_rem_)
                return;
            p0_carry_count_ = p1_tail_collect_rem_;
            p1_carry_pending_ = 1;
            p1_tail_collect_rem_ = 0;
            p1_tail_idx_ = 0;
            return;
        }
        bool carry_only_full = false;
        bool p1_rx_in_buf = false;
        if (buf_idx_ == 0 && p1_carry_pending_) {
            const int cc = p0_carry_count_;
            for (int j = 0; j < cc; ++j) {
                buf_I_[j] = p0_carry_I_[j];
                buf_Q_[j] = p0_carry_Q_[j];
            }
            buf_idx_ = cc;
            p1_carry_prefix_ = cc;
            p1_carry_pending_ = 0;
            if (buf_idx_ == 64) carry_only_full = true;
        }
        if (!carry_only_full) {
            buf_I_[buf_idx_] = chip_I;
            buf_Q_[buf_idx_] = chip_Q;
            ++buf_idx_;
            p1_rx_in_buf = true;
            if (buf_idx_ < 64) return;
        }
        buf_idx_ = 0;
        std::memcpy(orig_I_, buf_I_, 64 * sizeof(int16_t));
        std::memcpy(orig_Q_, buf_Q_, 64 * sizeof(int16_t));
        // [REMOVED Step1] I/Q 클립 경로 제거 — Gaussian noise 에 무효 확정.
        // [REMOVED Step3] if (ajc_enabled_) ajc_.Process(orig_I_, orig_Q_, 64) — NOP
#if defined(HTS_PHASE0_WALSH_BANK)
        // ── Walsh Bank Phase 1: FWHT + dominant_row 매칭 ──
        int32_t dot63_I = 0, dot63_Q = 0;
        int32_t dot0_I  = 0, dot0_Q  = 0;
        alignas(4) int32_t T_I[64];
        alignas(4) int32_t T_Q[64];
        for (int j = 0; j < 64; ++j) {
            T_I[j] = static_cast<int32_t>(orig_I_[j]);
            T_Q[j] = static_cast<int32_t>(orig_Q_[j]);
        }
        fwht_64_complex_inplace_(T_I, T_Q);
        {
            const int dr = static_cast<int>(dominant_row_);
            dot63_I = T_I[dr];
            dot63_Q = T_Q[dr];
        }
        {
            const uint8_t sym1_row_u = static_cast<uint8_t>(
                static_cast<unsigned>(dominant_row_) ^
                static_cast<unsigned>(k_W63_FWHT_ROW));
            const int sym1_row = static_cast<int>(sym1_row_u);
            dot0_I = T_I[sym1_row];
            dot0_Q = T_Q[sym1_row];
#if defined(HTS_DIAG_PRINTF)
            static bool s_p1_map_logged = false;
            if (!s_p1_map_logged) {
                std::printf("[DIAG-P1-MAP] dominant_row=%u k_W63_ROW=%u sym1_row=%u\n",
                            static_cast<unsigned>(dominant_row_),
                            static_cast<unsigned>(k_W63_FWHT_ROW),
                            static_cast<unsigned>(sym1_row_u));
                s_p1_map_logged = true;
            }
#endif
        }
        const int64_t e63_64 = static_cast<int64_t>(dot63_I) * dot63_I
                             + static_cast<int64_t>(dot63_Q) * dot63_Q;
        const int64_t e0_64  = static_cast<int64_t>(dot0_I) * dot0_I
                             + static_cast<int64_t>(dot0_Q) * dot0_Q;
#elif defined(HTS_TARGET_AMI)
        // ── AMI: 32-chip × 2 non-coherent ──
        int32_t dot63a_I = 0, dot63a_Q = 0;
        int32_t dot63b_I = 0, dot63b_Q = 0;
        int32_t dot0a_I  = 0, dot0a_Q  = 0;
        int32_t dot0b_I  = 0, dot0b_Q  = 0;
        for (int j = 0; j < 32; ++j) {
            const int32_t cI = static_cast<int32_t>(orig_I_[j]);
            const int32_t cQ = static_cast<int32_t>(orig_Q_[j]);
            dot0a_I += cI;
            dot0a_Q += cQ;
            if (k_w63[static_cast<std::size_t>(j)] > 0) {
                dot63a_I += cI;
                dot63a_Q += cQ;
            } else {
                dot63a_I -= cI;
                dot63a_Q -= cQ;
            }
        }
        for (int j = 32; j < 64; ++j) {
            const int32_t cI = static_cast<int32_t>(orig_I_[j]);
            const int32_t cQ = static_cast<int32_t>(orig_Q_[j]);
            dot0b_I += cI;
            dot0b_Q += cQ;
            if (k_w63[static_cast<std::size_t>(j)] > 0) {
                dot63b_I += cI;
                dot63b_Q += cQ;
            } else {
                dot63b_I -= cI;
                dot63b_Q -= cQ;
            }
        }
        const int64_t e63_64 =
            static_cast<int64_t>(dot63a_I) * dot63a_I
          + static_cast<int64_t>(dot63a_Q) * dot63a_Q
          + static_cast<int64_t>(dot63b_I) * dot63b_I
          + static_cast<int64_t>(dot63b_Q) * dot63b_Q;
        const int64_t e0_64 =
            static_cast<int64_t>(dot0a_I) * dot0a_I
          + static_cast<int64_t>(dot0a_Q) * dot0a_Q
          + static_cast<int64_t>(dot0b_I) * dot0b_I
          + static_cast<int64_t>(dot0b_Q) * dot0b_Q;
        // est 누적용 단일 dot (두 half 합, CFO phase cancel 영향 있음)
        const int32_t dot63_I = dot63a_I + dot63b_I;
        const int32_t dot63_Q = dot63a_Q + dot63b_Q;
#else
        // ── Stage 2 (PS-LTE): Walsh-domain 매칭 (FWHT + dominant_row) ──
        int32_t dot63_I = 0, dot63_Q = 0;
        int32_t dot0_I  = 0, dot0_Q  = 0;
        alignas(4) int32_t T_I[64];
        alignas(4) int32_t T_Q[64];
        for (int j = 0; j < 64; ++j) {
            T_I[j] = static_cast<int32_t>(orig_I_[j]);
            T_Q[j] = static_cast<int32_t>(orig_Q_[j]);
        }
        fwht_64_complex_inplace_(T_I, T_Q);
        {
            // P0 fingerprint only — 블록마다 argmax 하면 XOR 기하 붕괴(PRE_SYM1 미검출)
            const int dom = static_cast<int>(dominant_row_);
            const int sym1_row =
                dom ^ static_cast<int>(k_W63_FWHT_ROW_NATURAL);
            dot63_I = T_I[dom];
            dot63_Q = T_Q[dom];
            dot0_I  = T_I[sym1_row];
            dot0_Q  = T_Q[sym1_row];
        }
        const int64_t e63_64 = static_cast<int64_t>(dot63_I) * dot63_I
                             + static_cast<int64_t>(dot63_Q) * dot63_Q;
        const int64_t e0_64  = static_cast<int64_t>(dot0_I) * dot0_I
                             + static_cast<int64_t>(dot0_Q) * dot0_Q;
#endif
        const int32_t e63_sh = static_cast<int32_t>(e63_64 >> 16);
        const int32_t e0_sh  = static_cast<int32_t>(e0_64 >> 16);
#if defined(HTS_PHASE0_WALSH_BANK)
        static constexpr int32_t k_P1_MIN_E = 1000;  // FWHT row — PS-LTE 동일 하한
#elif defined(HTS_TARGET_AMI)
        static constexpr int32_t k_P1_MIN_E = 500;   // 32-chip × 2 non-coherent (peak 1/2)
#else
        static constexpr int32_t k_P1_MIN_E = 1000;  // 64-chip coherent (기존)
#endif
        const int32_t max_e_sh = (e63_sh >= e0_sh) ? e63_sh : e0_sh;
#if defined(HTS_DIAG_PRINTF) && !defined(HTS_PHASE0_WALSH_BANK)
        {
            static int s_p1_e_count = 0;
            ++s_p1_e_count;
            if (s_p1_e_count <= 500) {
                std::printf(
                    "[P1-E] call#%d e63_sh=%d e0_sh=%d max_e=%d k_P1_MIN=%d "
                    "pass=%s dom=%u\n",
                    s_p1_e_count, e63_sh, e0_sh, max_e_sh, k_P1_MIN_E,
                    (max_e_sh >= k_P1_MIN_E) ? "YES" : "NO",
                    static_cast<unsigned>(dominant_row_));
            }
        }
#endif
#if defined(HTS_DIAG_PRINTF) && defined(HTS_PHASE0_WALSH_BANK)
        // ─── DIAG-P1: Phase 1 FWHT / silent reset 경계 (로직 무변경) ───
        {
            static int s_p1_dump_count = 0;
            if (s_p1_dump_count < 3) {
                ++s_p1_dump_count;
                const uint8_t sym1_row_u = static_cast<uint8_t>(
                    static_cast<unsigned>(dominant_row_) ^
                    static_cast<unsigned>(k_W63_FWHT_ROW));
                std::printf(
                    "[DIAG-P1-DUMP] dump#%d dom_row=%u k_W63_ROW=%u "
                    "sym1_row=%u e63_sh=%d e0_sh=%d max_e_sh=%d "
                    "k_P1_MIN_E=%d pass=%d\n",
                    s_p1_dump_count,
                    static_cast<unsigned>(dominant_row_),
                    static_cast<unsigned>(k_W63_FWHT_ROW),
                    static_cast<unsigned>(sym1_row_u),
                    e63_sh, e0_sh, max_e_sh, k_P1_MIN_E,
                    static_cast<int>(max_e_sh >= k_P1_MIN_E));
                int64_t e_all[64];
                for (int r = 0; r < 64; ++r) {
                    e_all[r] = static_cast<int64_t>(T_I[r]) * T_I[r] +
                             static_cast<int64_t>(T_Q[r]) * T_Q[r];
                }
                std::printf("[DIAG-P1-TOP5]");
                for (int k = 0; k < 5; ++k) {
                    int best_r = 0;
                    int64_t best_ev = INT64_MIN;
                    for (int r = 0; r < 64; ++r) {
                        if (e_all[r] > best_ev) {
                            best_ev = e_all[r];
                            best_r = r;
                        }
                    }
                    std::printf(" row%d=%lld", best_r,
                                static_cast<long long>(best_ev >> 16));
                    e_all[best_r] = INT64_MIN;
                }
                std::printf("\n");
                std::printf("[DIAG-P1-IN] orig[0..15]_I:");
                for (int i = 0; i < 16; ++i) {
                    std::printf(" %d", static_cast<int>(orig_I_[i]));
                }
                std::printf("\n");
                std::printf("[DIAG-P1-IN] carry_prefix=%d\n",
                            p1_carry_prefix_);
            }
        }
#endif
        if (max_e_sh < k_P1_MIN_E) {
#if defined(HTS_DIAG_PRINTF) && defined(HTS_PHASE0_WALSH_BANK)
            {
                static int s_silent_reset_count = 0;
                ++s_silent_reset_count;
                if ((s_silent_reset_count % 5) == 0) {
                    std::printf(
                        "[DIAG-SILENT-RESET] count=%d (dom_row=%u "
                        "carry_prefix=%d)\n",
                        s_silent_reset_count,
                        static_cast<unsigned>(dominant_row_),
                        p1_carry_prefix_);
                }
            }
#endif
#if defined(HTS_DIAG_PRINTF) && !defined(HTS_PHASE0_WALSH_BANK)
            {
                static int s_silent_count = 0;
                ++s_silent_count;
                if ((s_silent_count % 10) == 0) {
                    std::printf("[SILENT] total=%d last_e=%d\n", s_silent_count,
                                max_e_sh);
                }
            }
#endif
            pre_phase_ = 0;
            psal_pending_ = false; psal_off_ = 0; psal_e63_ = 0;
            p0_chip_count_ = 0; p0_carry_count_ = 0;
            p1_carry_pending_ = 0; p1_tail_collect_rem_ = 0;
            p1_tail_idx_ = 0; p1_carry_prefix_ = 0;
            buf_idx_ = 0; wait_sync_head_ = 0; wait_sync_count_ = 0;
            if (carry_only_full && !p1_rx_in_buf) {
                p0_buf128_I_[0] = chip_I; p0_buf128_Q_[0] = chip_Q;
                p0_chip_count_ = 1;
            }
            return;
        }
        uint8_t best_m = 0u;
        if (e63_64 >= e0_64) {
            best_m = PRE_SYM0;
            if (first_c63_ == 0) {
                first_c63_ = e63_sh;
            }
            est_I_ += dot63_I;
            est_Q_ += dot63_Q;
            ++est_count_;
            // B-2: Phase 1 est 갱신 시 mag/recip 동기 갱신
            //   est_count_ 증가 → 매 갱신 시 정규화 상수 재계산
            update_derot_shift_from_est_();
        } else {
            best_m = PRE_SYM1;
        }
        const int8_t sym = static_cast<int8_t>(best_m);
        const int tail_rem = p1_carry_prefix_;
        p1_carry_prefix_ = 0;
#if defined(HTS_DIAG_PRINTF)
        std::printf("[P1-NC] sym=%d e63=%d e0=%d est=(%d,%d) n=%d carry_pend=%d\n",
                    static_cast<int>(sym), e63_sh, e0_sh,
                    est_I_, est_Q_, est_count_, p1_carry_pending_);
        {
            static int s_p1_gate_dump = 0;
            if (s_p1_gate_dump < 50000) {
                ++s_p1_gate_dump;
                const int gate_e_ok =
                    (max_e_sh >= k_P1_MIN_E) ? 1 : 0;
                const int gate_ge_dom = (e63_64 >= e0_64) ? 1 : 0;
                const int gate_sym_pre1 =
                    (sym == static_cast<int8_t>(PRE_SYM1)) ? 1 : 0;
                const int gate_sym_pre0 =
                    (sym == static_cast<int8_t>(PRE_SYM0)) ? 1 : 0;
                const int enter_hdr = gate_sym_pre1;
                std::printf(
                    "[P1-GATE] #%d n=%d e63_sh=%d e0_sh=%d max_e=%d "
                    "k_P1_MIN_E=%d gate_e_ok=%d ge_dom=%d "
                    "sym_pre1=%d sym_pre0=%d carry_pend=%d enter_hdr=%d\n",
                    s_p1_gate_dump, est_count_, e63_sh, e0_sh, max_e_sh,
                    k_P1_MIN_E, gate_e_ok, gate_ge_dom, gate_sym_pre1,
                    gate_sym_pre0, p1_carry_pending_, enter_hdr);
            }
        }
#endif
        if (sym == static_cast<int8_t>(PRE_SYM1)) {
            p1_tail_collect_rem_ = 0; p1_tail_idx_ = 0;
            p1_carry_pending_ = 0; p0_carry_count_ = 0;
            update_derot_shift_from_est_();
            set_phase_(RxPhase::READ_HEADER);
            // [REMOVED Step2] CW EMA 상태 제거됨
            hdr_count_ = 0; hdr_fail_ = 0;
            wait_sync_head_ = 0; wait_sync_count_ = 0;
#if defined(HTS_DIAG_PRINTF)
            std::printf("[P1→HDR] est=(%d,%d) n=%d\n",
                        est_I_, est_Q_, est_count_);
#endif
            if (carry_only_full && !p1_rx_in_buf) {
                buf_I_[0] = chip_I; buf_Q_[0] = chip_Q; buf_idx_ = 1;
            } else {
                buf_idx_ = 0;
            }
            return;
        }
        if (sym == static_cast<int8_t>(PRE_SYM0)) {
            if (tail_rem > 0) {
                p1_tail_collect_rem_ = tail_rem; p1_tail_idx_ = 0;
                if (carry_only_full) {
                    p0_carry_I_[0] = chip_I; p0_carry_Q_[0] = chip_Q;
                    p1_tail_idx_ = 1;
                    if (p1_tail_idx_ >= p1_tail_collect_rem_) {
                        p0_carry_count_ = tail_rem; p1_carry_pending_ = 1;
                        p1_tail_collect_rem_ = 0; p1_tail_idx_ = 0;
                    }
                }
            }
            buf_idx_ = 0;
            return;
        }
        pre_phase_ = 0;
        psal_pending_ = false; psal_off_ = 0; psal_e63_ = 0;
        p0_chip_count_ = 0; p0_carry_count_ = 0;
        p1_carry_pending_ = 0; p1_tail_collect_rem_ = 0;
        p1_tail_idx_ = 0; p1_carry_prefix_ = 0;
        buf_idx_ = 0; wait_sync_head_ = 0; wait_sync_count_ = 0;
        if (carry_only_full && !p1_rx_in_buf) {
            p0_buf128_I_[0] = chip_I; p0_buf128_Q_[0] = chip_Q;
            p0_chip_count_ = 1;
        }
        return;
    }
    if (buf_idx_ >= 64)
        return;
    buf_I_[buf_idx_] = chip_I;
    buf_Q_[buf_idx_] = chip_Q;
    buf_idx_++;
    if (phase_ == RxPhase::READ_HEADER) {
        if (buf_idx_ == 64) {
            std::memcpy(orig_I_, buf_I_, 64 * sizeof(int16_t));
            std::memcpy(orig_Q_, buf_Q_, 64 * sizeof(int16_t));
            // [REMOVED Step1] I/Q 클립 경로 제거 — Gaussian noise 에 무효 확정.
            static int16_t s_hdr_blk0_I[64], s_hdr_blk0_Q[64];
            if (hdr_count_ == 0) {
                const SymDecResult rh =
                    walsh_dec_full_(orig_I_, orig_Q_, 64, false);
                if (rh.sym >= 0 && rh.sym < 64) {
                    const uint8_t walsh_shift = current_walsh_shift_();
                    const uint8_t corrected_sym = static_cast<uint8_t>(
                        (static_cast<unsigned>(rh.sym) & 63u) ^ walsh_shift);
                    hdr_syms_[hdr_count_] = corrected_sym;
#if defined(HTS_DIAG_PRINTF)
                    std::printf(
                        "[HDR-SHIFT] raw_sym=%d shift=%u corrected=%u dom=%u\n",
                        static_cast<int>(rh.sym),
                        static_cast<unsigned>(walsh_shift),
                        static_cast<unsigned>(corrected_sym),
                        static_cast<unsigned>(dominant_row_));
#endif
                    std::memcpy(s_hdr_blk0_I, orig_I_, 64 * sizeof(int16_t));
                    std::memcpy(s_hdr_blk0_Q, orig_Q_, 64 * sizeof(int16_t));
                    hdr_count_++;
                } else {
                    hdr_fail_++;
                    if (hdr_fail_ >= HDR_FAIL_MAX) {
#if defined(HTS_DIAG_PRINTF)
                        std::printf("[HDR-FAIL] max fail, back to P1\n");
#endif
                        hdr_count_ = 0;
                        hdr_fail_ = 0;
                        buf_idx_ = 0;
                        set_phase_(RxPhase::WAIT_SYNC);
                    }
                }
            } else if (hdr_count_ == 1) {
                // 6-bit 패리티 LUT (popcount&1 대체, ARM 1사이클)
                static constexpr uint8_t k_par6[64] = {
                    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
                    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
                    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
                    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
                };
                static constexpr struct {
                    uint8_t mb;
                    uint16_t plen;
                } k_hdr_defs[] = {
                    {0u, static_cast<uint16_t>(FEC_HARQ::NSYM1)},
                    {0u, static_cast<uint16_t>(FEC_HARQ::NSYM1)},
                    {1u, static_cast<uint16_t>(FEC_HARQ::NSYM16)},
                    {1u, static_cast<uint16_t>(FEC_HARQ::NSYM16)},
                    {2u, static_cast<uint16_t>(FEC_HARQ::NSYM16)},
                    {2u, static_cast<uint16_t>(FEC_HARQ::NSYM16)},
                    {3u, static_cast<uint16_t>(FEC_HARQ::nsym_for_bps(4))},
                    {3u, static_cast<uint16_t>(FEC_HARQ::nsym_for_bps(4))},
                    {3u, static_cast<uint16_t>(FEC_HARQ::nsym_for_bps(5))},
                    {3u, static_cast<uint16_t>(FEC_HARQ::nsym_for_bps(5))},
                    {3u, static_cast<uint16_t>(FEC_HARQ::nsym_for_bps(6))},
                    {3u, static_cast<uint16_t>(FEC_HARQ::nsym_for_bps(6))},
                };
                // ── P0-FIX-002: 위상 독립 템플릿 매칭 ──
                // 기존: v = I[j] + Q[j] projection 후 부호 변조 누적.
                //   채널 회전 θ에서 |corr| ∝ |cosθ − sinθ| + |cosθ + sinθ|
                //   → θ=90°/270° 에서 corr_sum 소멸, S2 전멸의 주 원인.
                // 수정: I 채널 상관 cI, Q 채널 상관 cQ 를 각각 구하고
                //   에너지 e = cI² + cQ² 로 비교. 블록 0, 블록 1 의
                //   에너지 합을 템플릿 점수로 사용 — θ 불변.
                //   CRC 없는 헤더이므로 separation gate 는 에너지 도메인
                //   3 dB (best > second × 2) 로 엄격화.
                int best_idx = -1;
                int64_t best_e = 0;
                int64_t second_e = 0;
                // Stage 6A+: CFO 시 RX chip 열이 k_w_{sym XOR shift} 형태로 보임
                const uint8_t walsh_shift = current_walsh_shift_();
                for (int ti = 0; ti < 12; ++ti) {
                    const uint8_t mb = k_hdr_defs[static_cast<size_t>(ti)].mb;
                    const uint16_t pl = k_hdr_defs[static_cast<size_t>(ti)].plen;
                    const uint16_t iq =
                        (static_cast<unsigned>(ti) & 1u) != 0u
                            ? static_cast<uint16_t>(HDR_IQ_BIT)
                            : 0u;
                    const uint16_t hdr_val =
                        (static_cast<uint16_t>(mb) << 10u) | iq | pl;
                    const uint8_t sym0 =
                        static_cast<uint8_t>((hdr_val >> 6u) & 0x3Fu);
                    const uint8_t sym1 =
                        static_cast<uint8_t>(hdr_val & 0x3Fu);
                    const uint8_t sym0_shifted =
                        static_cast<uint8_t>(sym0 ^ walsh_shift);
                    const uint8_t sym1_shifted =
                        static_cast<uint8_t>(sym1 ^ walsh_shift);
                    // 블록 0 (s_hdr_blk0_I/Q) — I 채널 / Q 채널 각각 상관
                    int64_t cI0 = 0;
                    int64_t cQ0 = 0;
                    for (int j = 0; j < 64; ++j) {
                        const int32_t sm = -static_cast<int32_t>(
                            k_par6[static_cast<uint8_t>(sym0_shifted & j)]);
                        const int32_t vI =
                            static_cast<int32_t>(s_hdr_blk0_I[j]);
                        const int32_t vQ =
                            static_cast<int32_t>(s_hdr_blk0_Q[j]);
                        cI0 += static_cast<int64_t>((vI ^ sm) - sm);
                        cQ0 += static_cast<int64_t>((vQ ^ sm) - sm);
                    }
                    // 블록 1 (orig_I_/orig_Q_)
                    int64_t cI1 = 0;
                    int64_t cQ1 = 0;
                    for (int j = 0; j < 64; ++j) {
                        const int32_t sm = -static_cast<int32_t>(
                            k_par6[static_cast<uint8_t>(sym1_shifted & j)]);
                        const int32_t vI = static_cast<int32_t>(orig_I_[j]);
                        const int32_t vQ = static_cast<int32_t>(orig_Q_[j]);
                        cI1 += static_cast<int64_t>((vI ^ sm) - sm);
                        cQ1 += static_cast<int64_t>((vQ ^ sm) - sm);
                    }
                    // 에너지 도메인 (Q16 스케일 다운 — overflow 방어)
                    const int64_t e0 =
                        ((cI0 * cI0) + (cQ0 * cQ0)) >> 16;
                    const int64_t e1 =
                        ((cI1 * cI1) + (cQ1 * cQ1)) >> 16;
                    const int64_t e = e0 + e1;
                    if (e > best_e) {
                        second_e = best_e;
                        best_e = e;
                        best_idx = ti;
                    } else if (e > second_e) {
                        second_e = e;
                    }
                }
                // Fix B: ratio 2.0x -> 1.5x (best >= second + second/2).
                // V29b: linear floor best_e > first_c63_ (same >>16 scale as
                // e63_sh). Squared floor (first_c63_^2) vs template best_e was
                // mismatched and suppressed [HDR-TMPL] globally (V29 sim).
                const int64_t fc63 =
                    static_cast<int64_t>(first_c63_);
                const int64_t second_64 =
                    static_cast<int64_t>(second_e);
                const int64_t best_64 =
                    static_cast<int64_t>(best_e);
                const int64_t best_ge_15x =
                    best_64 >= (second_64 + (second_64 >> 1));
                const uint32_t floor_ok_u =
                    static_cast<uint32_t>((fc63 <= 0) | (best_64 > fc63));
                const bool tmpl_ok =
                    (best_idx >= 0) &&
                    ((second_e == 0) ||
                     (best_ge_15x && (floor_ok_u != 0u)));
                if (tmpl_ok) {
                    const uint8_t mb =
                        k_hdr_defs[static_cast<size_t>(best_idx)].mb;
                    const uint16_t pl =
                        k_hdr_defs[static_cast<size_t>(best_idx)].plen;
                    const uint16_t iq =
                        (static_cast<unsigned>(best_idx) & 1u) != 0u
                            ? static_cast<uint16_t>(HDR_IQ_BIT)
                            : 0u;
                    const uint16_t hdr_val =
                        (static_cast<uint16_t>(mb) << 10u) | iq | pl;
                    hdr_syms_[0] =
                        static_cast<uint8_t>((hdr_val >> 6u) & 0x3Fu);
                    hdr_syms_[1] = static_cast<uint8_t>(hdr_val & 0x3Fu);
                    hdr_count_ = 2;
#if defined(HTS_DIAG_PRINTF)
                    std::printf(
                        "[HDR-TMPL] matched idx=%d hdr=0x%03X shift=%u dom=%u\n",
                        best_idx, static_cast<unsigned>(hdr_val),
                        static_cast<unsigned>(walsh_shift),
                        static_cast<unsigned>(dominant_row_));
#endif
                } else {
                    SymDecResult rh =
                        walsh_dec_full_(orig_I_, orig_Q_, 64, false);
                    int8_t sym = rh.sym;
                    if (sym >= 0 && sym < 64) {
                        if (rh.second_e > 0u &&
                            rh.best_e < rh.second_e * 2u) {
#if defined(HTS_DIAG_PRINTF)
                            std::printf("[HDR-SOFT] unreliable, reset\n");
#endif
                            hdr_count_ = 0;
                            buf_idx_ = 0;
                            return;
                        }
                        const uint8_t sym_cor = static_cast<uint8_t>(
                            (static_cast<unsigned>(sym) & 63u) ^ walsh_shift);
                        hdr_syms_[hdr_count_] = sym_cor;
#if defined(HTS_DIAG_PRINTF)
                        std::printf(
                            "[HDR-SHIFT] raw_sym=%d shift=%u corrected=%u dom=%u\n",
                            static_cast<int>(sym),
                            static_cast<unsigned>(walsh_shift),
                            static_cast<unsigned>(sym_cor),
                            static_cast<unsigned>(dominant_row_));
#endif
                        hdr_count_++;
                    } else {
                        hdr_fail_++;
                        if (hdr_fail_ >= HDR_FAIL_MAX) {
#if defined(HTS_DIAG_PRINTF)
                            std::printf("[HDR-FAIL] max fail, back to P1\n");
#endif
                            hdr_count_ = 0;
                            hdr_fail_ = 0;
                            buf_idx_ = 0;
                            set_phase_(RxPhase::WAIT_SYNC);
                        }
                    }
                }
            } else {
                SymDecResult rh =
                    walsh_dec_full_(orig_I_, orig_Q_, 64, false);
                int8_t sym = rh.sym;
                if (sym >= 0 && sym < 64) {
                    if (rh.second_e > 0u &&
                        rh.best_e < rh.second_e * 2u) {
#if defined(HTS_DIAG_PRINTF)
                        std::printf("[HDR-SOFT] unreliable e=%u/%u, skip\n",
                                    rh.best_e, rh.second_e);
#endif
                        hdr_count_ = 0;
                        buf_idx_ = 0;
                        return;
                    }
                    const uint8_t walsh_shift = current_walsh_shift_();
                    const uint8_t sym_cor = static_cast<uint8_t>(
                        (static_cast<unsigned>(sym) & 63u) ^ walsh_shift);
                    hdr_syms_[hdr_count_] = sym_cor;
#if defined(HTS_DIAG_PRINTF)
                    std::printf(
                        "[HDR-SHIFT] raw_sym=%d shift=%u corrected=%u dom=%u\n",
                        static_cast<int>(sym),
                        static_cast<unsigned>(walsh_shift),
                        static_cast<unsigned>(sym_cor),
                        static_cast<unsigned>(dominant_row_));
#endif
                    hdr_count_++;
                } else {
                    hdr_fail_++;
                    if (hdr_fail_ >= HDR_FAIL_MAX) {
#if defined(HTS_DIAG_PRINTF)
                        std::printf("[HDR-FAIL] max fail, back to P1\n");
#endif
                        hdr_count_ = 0;
                        hdr_fail_ = 0;
                        buf_idx_ = 0;
                        set_phase_(RxPhase::WAIT_SYNC);
                        return;
                    }
                }
            }
            if (hdr_count_ >= HDR_SYMS) {
                PayloadMode mode;
                int plen = 0;
                if (parse_hdr_(mode, plen) != 0u) {
                    cur_mode_ = mode;
                    pay_cps_ = (mode == PayloadMode::VIDEO_1) ? 1
                               : (mode == PayloadMode::DATA)  ? 64
                                                              : 16;
                    pay_total_ = plen;
                    pay_recv_ = 0;
                    v1_idx_ = 0;
                    sym_idx_ = 0;
                    max_harq_ = FEC_HARQ::DATA_K; // 모든 모드에서 32
                    set_phase_(RxPhase::READ_PAYLOAD);
                    buf_idx_ = 0;
                    // [Walsh_Row_Permuter] 헤더 완료 후 payload 직전 — TX Build_Packet
                    // 의 tx_seq_(= rx_seq_ 미증가 시점) 및 첫 RV 라운드(0) 와 정합
                    if (!walsh_permuter_.Is_Initialized()) {
                        (void)walsh_permuter_.Initialize(rx_seq_, 0u);
                    } else {
                        (void)walsh_permuter_.Update_Key(rx_seq_, 0u);
                    }
                    //  try_decode_ 내부에서 Init → Feed 이후라 데이터 파괴
                    //  READ_PAYLOAD 진입 시 첫 라운드만 Init
                    if (!harq_inited_) {
                        if (mode == PayloadMode::VIDEO_16 ||
                            mode == PayloadMode::VOICE) {
                            if (ir_mode_) {
                                SecureMemory::secureWipe(
                                    static_cast<void *>(&g_harq_ccm_union),
                                    sizeof(g_harq_ccm_union));
                                if (ir_state_ != nullptr) {
                                    FEC_HARQ::IR_Init(*ir_state_);
                                }
                                ir_rv_ = 0;
                            } else {
                                FEC_HARQ::Init16(rx_.m16);
                            }
                        } else if (mode == PayloadMode::DATA) {
                            // DATA READ_PAYLOAD: CCM union 전체 한 회 wipe
                            SecureMemory::secureWipe(
                                static_cast<void *>(&g_harq_ccm_union),
                                sizeof(g_harq_ccm_union));
                            if (ir_mode_) {
                                if (ir_state_ != nullptr) {
                                    FEC_HARQ::IR_Init(*ir_state_);
                                }
                                ir_rv_ = 0;
                            } else {
                                SecureMemory::secureWipe(
                                    static_cast<void *>(rx_.m64_I.aI),
                                    sizeof(rx_.m64_I.aI));
                                rx_.m64_I.k = 0;
                                rx_.m64_I.ok = false;
                            }
                        }
                        harq_inited_ = true;
                    }
                    // [REMOVED Step3] pay_cps_ 변경 시 ajc_.Reset — AJC 제거
                } else {
                    // HDR parse 실패: P0 재시작 대신 Phase 1로 복귀
                    // est/derot 유지 — 다음 프리앰블(HARQ 라운드)에서 재시도
#if defined(HTS_DIAG_PRINTF)
                    std::printf("[HDR-RETRY] parse fail, back to P1\n");
#endif
                    hdr_count_ = 0;
                    buf_idx_ = 0;
                    phase_ = RxPhase::WAIT_SYNC;
                    // pre_phase_ 유지 → Phase 1 계속
                }
            }
            if (phase_ == RxPhase::READ_HEADER)
                buf_idx_ = 0;
        }
    } else if (phase_ == RxPhase::READ_PAYLOAD) {
        if (buf_idx_ >= pay_cps_)
            on_sym_();
    }
}
/* BUG-FIX-RETX4: HARQ 연속모드 RX */
void HTS_V400_Dispatcher::Feed_Retx_Chip(int16_t rx_I, int16_t rx_Q) noexcept {
    if (phase_ == RxPhase::RF_SETTLING) {
        return;
    }
    if (!retx_ready_ || phase_ != RxPhase::READ_PAYLOAD)
        return;
    if (buf_idx_ >= 64)
        return;
    if (buf_idx_ == 0) {
        holo_lpi_rx_chip_idx_ = 0u;
        holo_lpi_rx_scalars_seq_ = 0xFFFFFFFFu;
        // [Walsh_Row_Permuter] HARQ 재전송 슬롯 — Build_Retx 의 (tx_seq_prev, ir_rv_)
        // 와 동일 (tx_seq_prev == 미완료 PDU 에 대한 rx_seq_)
        if (walsh_permuter_.Is_Initialized()) {
            (void)walsh_permuter_.Update_Key(
                rx_seq_, static_cast<uint8_t>(ir_rv_ & 0xFF));
        }
    }
    int16_t ti = rx_I;
    int16_t tq = rx_Q;
    const uint32_t retx_slot =
        (rx_seq_ > 0u) ? (rx_seq_ - 1u) : 0u;
    apply_holo_lpi_inverse_rx_chip_(ti, tq, retx_slot);
    buf_I_[buf_idx_] = ti;
    buf_Q_[buf_idx_] = tq;
    buf_idx_++;
    if (buf_idx_ >= pay_cps_)
        on_sym_();
}
void HTS_V400_Dispatcher::Inject_Payload_Phase(PayloadMode mode,
                                               int bps) noexcept {
    // 동기/헤더 단계를 건너뛰고 READ_PAYLOAD로 직접 진입
    // IR 상태 초기화 포함
    full_reset_();

    cur_mode_ = mode;
    if (mode == PayloadMode::DATA) {
        cur_bps64_ = FEC_HARQ::bps_clamp_runtime(bps);
        pay_cps_ = 64;
        pay_total_ = FEC_HARQ::nsym_for_bps(cur_bps64_);
    } else if (mode == PayloadMode::VOICE || mode == PayloadMode::VIDEO_16) {
        pay_cps_ = 16;
        pay_total_ = FEC_HARQ::NSYM16;
    } else {
        return; // VIDEO_1은 미지원
    }

    pay_recv_ = 0;
    sym_idx_ = 0;
    max_harq_ = FEC_HARQ::DATA_K;
    phase_ = RxPhase::READ_PAYLOAD;
    buf_idx_ = 0;

    // HARQ/IR 초기화
    SecureMemory::secureWipe(static_cast<void *>(&g_harq_ccm_union),
                             sizeof(g_harq_ccm_union));
    if (ir_mode_ && ir_state_ != nullptr) {
        FEC_HARQ::IR_Init(*ir_state_);
    }
    ir_rv_ = 0;
    harq_inited_ = true;
    retx_ready_ = false;

    // [Walsh_Row_Permuter] 동기/헤더 우회 시험 경로 — 일반 RX 와 동일 키
    if (!walsh_permuter_.Is_Initialized()) {
        (void)walsh_permuter_.Initialize(rx_seq_, 0u);
    } else {
        (void)walsh_permuter_.Update_Key(rx_seq_, 0u);
    }

    // [REMOVED Step3] pay_cps_ 변경 시 ajc_.Reset — AJC 제거
}
} // namespace ProtectedEngine
