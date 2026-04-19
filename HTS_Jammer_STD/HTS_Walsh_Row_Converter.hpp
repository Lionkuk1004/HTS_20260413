#ifndef HTS_WALSH_ROW_CONVERTER_HPP
#define HTS_WALSH_ROW_CONVERTER_HPP

#include <cstdint>

// ============================================================================
// WRC 방식 선택 매크로
//   0 = OFF (baseline, WRC 함수 호출 해도 no-op)
//   1 = 방식 A: zero-out
//   2 = 방식 B: attenuate
//   3 = 방식 C: top1-only
// ============================================================================
#ifndef HTS_WRC_METHOD
#define HTS_WRC_METHOD 0
#endif

namespace ProtectedEngine {
namespace WRC {

// 방식 A: sigma 임계 (이상치 탐지)
static constexpr int SIGMA_NUM = 3;   // 3σ
static constexpr int SIGMA_DEN = 1;

// 방식 B: attenuation 강도
static constexpr int32_t ALPHA_Q8 = 128;  // 0.5
static constexpr int32_t W_MAX_Q8 = 256;  // 1.0
static constexpr int32_t W_MIN_Q8 = 16;   // 0.0625

// 최대 처리 가능 길이
static constexpr int MAX_N = 64;

// ============================================================================
// clean_chips
//   입력: chips[N] (int16_t, in-place)
//   N = 16 또는 64
//   HTS_WRC_METHOD 에 따라 분기 (0 이면 no-op)
// ============================================================================
void clean_chips(int16_t* chips, int N) noexcept;

// ============================================================================
// 진단 카운터 (WRC 실행 검증용)
// ============================================================================
struct DiagCounters {
    uint64_t call_count_16;    // N=16 호출 수
    uint64_t call_count_64;    // N=64 호출 수
    uint64_t total_delta_sum;  // sum of |output - input| over all chips
    uint64_t method;           // 실제 사용된 method (0~3)
};

// 전역 카운터 접근
DiagCounters& get_diag() noexcept;

// 카운터 리셋 + 최종 출력
void reset_diag() noexcept;
void print_diag() noexcept;

} // namespace WRC
} // namespace ProtectedEngine

#endif // HTS_WALSH_ROW_CONVERTER_HPP
