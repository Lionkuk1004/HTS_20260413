// =============================================================================
// HTS_V400_Dispatcher_Decode.cpp — Walsh decode / blackhole
// =============================================================================
#include "HTS_V400_Dispatcher.hpp"
#include "HTS_V400_Dispatcher_Internal.hpp"
#include <climits>
#include <cstdint>
namespace ProtectedEngine {
using namespace detail;
alignas(64) extern const int16_t k_walsh_dummy_iq_[64] = {};
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
#if defined(HTS_DIAG_FWHT_INTERNAL) || defined(HTS_DIAG_AGC_TRACE)
    {
        const int32_t pk =
            fwht_post_peak_max_abs_(dec_wI_, dec_wQ_, n_eff);
        const int sh = fwht_int16_store_downshift_(pk);
        fwht_post_shift_hist_record(sh);
    }
#endif
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
} // namespace ProtectedEngine
