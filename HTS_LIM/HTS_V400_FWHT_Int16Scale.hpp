#ifndef HTS_V400_FWHT_INT16_SCALE_HPP
#define HTS_V400_FWHT_INT16_SCALE_HPP
/**
 * @file HTS_V400_FWHT_Int16Scale.hpp
 * @brief Phase 4-B-1: FWHT 스펙트럼 피크 기반 IR 칩 int16 저장용 산술 시프트 (`chip ≫ shift`).
 *
 * @details `HTS_V400_Dispatcher.hpp`에 의존하지 않음. 펌웨어 메인트리와 호스트 `Local` TU가 동일
 * 수학을 공유한다. 양산 경로는 정수만 사용; DIAG 전용 히스토그램·호스트 `fprintf`는
 * `HTS_DIAG_FWHT_INTERNAL` 또는 `HTS_DIAG_AGC_TRACE`에서만 컴파일된다.
 */

#include <cstdint>
#if defined(HTS_DIAG_FWHT_INTERNAL) || defined(HTS_DIAG_AGC_TRACE)
#  include <atomic>
#  if defined(HTS_ALLOW_HOST_BUILD)
#    include <cstdio>
#  endif
#endif

namespace HtsFwhtInt16Scale {

/**
 * @brief `fwht_raw` 직후 스펙트럼 bin에서 |I|,|Q| 절댓값의 최대.
 * @param wI,wQ 길이 `n`인 FWHT 출력 버퍼 (read-only).
 * @param n FWHT 크기 (호출부에서 16 또는 64).
 * @return 피크 절댓값; `n<=0`이면 호출부에서 방지 (현재 TU는 양수만 전달).
 */
inline int32_t post_peak_max_abs(const int32_t *wI, const int32_t *wQ,
                                   int n) noexcept {
    int32_t m = 0;
    for (int r = 0; r < n; ++r) {
        const int32_t aI = wI[r] < 0 ? -wI[r] : wI[r];
        const int32_t aQ = wQ[r] < 0 ? -wQ[r] : wQ[r];
        if (aI > m) {
            m = aI;
        }
        if (aQ > m) {
            m = aQ;
        }
    }
    return m;
}

/**
 * @brief int16 SRAM 저장 직전 산술 우시프트량 (0…5).
 * @param peak_abs `post_peak_max_abs` 결과 (비음수).
 * @return `peak_abs <= 262143`(NONE)이면 0; 그 위 구간은 Phase 4-B-1 튜닝 임계값에 따른
 *         누적 비교로 1…5.
 * @note TIM 등 고피크 경로만 시프트; NONE(저피크) 동작은 시프트 0으로 기존과 동일.
 * @warning 나눗셈 없음. 임계값 변경 시 양산 회귀 재검증 필요.
 */
inline int downshift_for_int16_store(int32_t peak_abs) noexcept {
    int shift = 0;
    if (peak_abs > 262143) {
        shift = 1;
    }
    if (peak_abs > 524287) {
        shift = 2;
    }
    if (peak_abs > 1048575) {
        shift = 3;
    }
    if (peak_abs > 2097151) {
        shift = 4;
    }
    if (peak_abs > 4194303) {
        shift = 5;
    }
    return shift;
}

#if defined(HTS_DIAG_FWHT_INTERNAL) || defined(HTS_DIAG_AGC_TRACE)
extern std::atomic<uint32_t> g_post_shift_hist[8];
inline void post_shift_hist_record(int shift) noexcept {
    g_post_shift_hist[static_cast<std::size_t>(shift & 7)].fetch_add(
        1u, std::memory_order_relaxed);
}
inline void post_shift_hist_reset() noexcept {
    for (int i = 0; i < 8; ++i) {
        g_post_shift_hist[static_cast<std::size_t>(i)].store(
            0u, std::memory_order_relaxed);
    }
}
#  if defined(HTS_ALLOW_HOST_BUILD)
inline void post_shift_hist_dump(const char *label) noexcept {
    uint32_t vals[8];
    uint32_t total = 0u;
    for (int s = 0; s < 8; ++s) {
        vals[static_cast<std::size_t>(s)] =
            g_post_shift_hist[static_cast<std::size_t>(s)].load(
                std::memory_order_relaxed);
        total += vals[static_cast<std::size_t>(s)];
    }
    if (total == 0u) {
        return;
    }
    std::fprintf(stderr, "\n[FWHT-SHIFT] === %s post-FWHT int16 downshift ===\n",
                 label != nullptr ? label : "");
    for (int s = 0; s < 8; ++s) {
        const uint32_t v = vals[static_cast<std::size_t>(s)];
        if (v != 0u) {
            std::fprintf(stderr, "  shift=%d: %u (%.2f%%)\n", s,
                         static_cast<unsigned>(v),
                         100.0 * static_cast<double>(v) /
                             static_cast<double>(total));
        }
    }
    std::fprintf(stderr, "[FWHT-SHIFT] === end ===\n\n");
}
#  else
inline void post_shift_hist_dump(const char *) noexcept {}
#  endif
#else
inline void post_shift_hist_record(int) noexcept {}
inline void post_shift_hist_reset() noexcept {}
inline void post_shift_hist_dump(const char *) noexcept {}
#endif

} // namespace HtsFwhtInt16Scale

#if defined(HTS_DIAG_FWHT_INTERNAL) || defined(HTS_DIAG_AGC_TRACE)
extern "C" inline void HTS_FWHT_SHIFT_DIAG_Reset(void) noexcept {
    HtsFwhtInt16Scale::post_shift_hist_reset();
}
extern "C" inline void HTS_FWHT_SHIFT_DIAG_Dump(const char *label) noexcept {
    HtsFwhtInt16Scale::post_shift_hist_dump(label);
}
#endif

#endif // HTS_V400_FWHT_INT16_SCALE_HPP
