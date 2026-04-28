#pragma once
// T6 host 전용: HTS_V5A_DIAG_PER_SCENARIO 시 CFO_V5a::Estimate 가 시나리오 라벨을 읽음.
// 정의·초기값은 HTS_T6_SIM_Test.cpp (또는 동일 exe를 링크하는 TU 한 곳).

#if defined(HTS_V5A_DIAG_PER_SCENARIO)
extern const char* g_t6_current_scenario_label;
extern int g_t6_current_phase_deg;
extern int g_t6_current_cfo_hz;
#endif
