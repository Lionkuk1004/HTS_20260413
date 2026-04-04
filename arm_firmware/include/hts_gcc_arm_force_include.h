/* GCC ARM 크로스 빌드용 강제 include — HTS_LIM 코어 파일 비수정 */
#pragma once

#if defined(__GNUC__) && !defined(__SSAT)
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

static inline int32_t __hts_ssat_gcc(int32_t x, unsigned n)
{
    if (n == 0u || n > 32u) {
        return 0;
    }
    int32_t smax;
    int32_t smin;
    if (n == 32u) {
        smax = (int32_t)0x7FFFFFFF;
        smin = (int32_t)(-2147483647 - 1);
    } else {
        smax = (int32_t)((1u << (n - 1u)) - 1u);
        smin = (int32_t)(-1 - smax);
    }
    if (x > smax) {
        return smax;
    }
    if (x < smin) {
        return smin;
    }
    return x;
}

#ifdef __cplusplus
}
#endif

#define __SSAT(X, N) __hts_ssat_gcc((int32_t)(X), (unsigned)(N))
#endif
