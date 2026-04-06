// =========================================================================
// HTS_Adaptive_BPS_Controller.cpp
// HTS 적응형 BPS 히스테리시스 컨트롤러 구현부
// Target: STM32F407 (Cortex-M4, 168MHz)
//
//
// [판단 로직 요약]
//
//  HEAVY → BPS_MIN 즉시 (재밍 감지 → 즉각 반응, 지연 허용 안 됨)
//  QUIET → quiet_count_++ → HYST_UP_COUNT 도달 시 BPS++ → count 리셋
//  HOLD  → quiet_count_ 절반 감쇠 (QUIET↔HOLD 진동 시 상향 진행 보존)
//          + ajc ≥ AJC_DRIFT_DOWN_THR 연속 시 drift_down_count_ → BPS--
//
// [HOLD 절반 감쇠]
//  완전 리셋(0)이면 AJC가 IDLE 근처에서만 요동할 때 8연속 QUIET을
//  다시 채우기 어렵다. 절반 감쇠로 ‘거의 조용’ 프레임 누적을 일부 보존한다.
//
// [DRIFT_DOWN]
//  500~2000 사이에 SNR은 아직 붕괴(5 미만)가 아닐 때, HEAVY까지 기다리지 않고
//  한 단계씩만 BPS를 내린다 (처리량·강인도 절충, 교착 완화).
//
// [왜 BPS를 HEAVY에서 즉시 BPS_MIN으로 점프하는가]
//  재밍 환경에서 BPS가 4→3→4→3으로 진동하면 HARQ 파라미터가 매 프레임 바뀌고
//  FEC 디코더가 일관된 상태를 유지할 수 없습니다.
//  BPS_MIN(3)이 변전소 +20dB 재밍에서 검증된 안전 최솟값이므로,
//  재밍 감지 즉시 확정 최솟값으로 떨어뜨리는 것이 가장 안전합니다.
//
// [구현]
//  Update() — 제어 흐름 분기 없이 플래그(0/1) + 산술 마스킹으로 동일 의미 유지.
//  current_bps: 함수 상단 1회 relaxed load, 종료 직전 1회 release store.
// =========================================================================
#include "HTS_Adaptive_BPS_Controller.h"

namespace ProtectedEngine {

    HTS_Adaptive_BPS_Controller::HTS_Adaptive_BPS_Controller(
        HTS_RF_Metrics& metrics) noexcept
        : metrics_(metrics)
        , quiet_count_(0u)
        , drift_down_count_(0u)
    {
        // 초기 상태: 안전 최솟값으로 강제 설정
        // 시스템 부팅 직후 채널 상태를 알 수 없으므로 보수적 출발
        metrics_.current_bps.store(BPS_MIN, std::memory_order_release);
    }

    void HTS_Adaptive_BPS_Controller::Update() noexcept {
        // 단일 코어(Cortex-M): std::atomic_flag try-lock은 사용하지 않음.
        // 메인이 Set한 뒤 ISR이 동일 Update()를 호출하면 조기 반환으로
        // BPS/히스테리시스 갱신이 통째로 건너뛰어질 수 있음(실시간 적응 실패).
        // quiet_count_는 메인 루프 단일 컨텍스트에서만 갱신 — ISR 호출 금지(헤더 계약).

        const uint8_t cur_bps = metrics_.current_bps.load(
            std::memory_order_relaxed);
        const int32_t  snr = metrics_.snr_proxy.load(
            std::memory_order_acquire);
        const uint32_t ajc = metrics_.ajc_nf.load(
            std::memory_order_acquire);

        const uint8_t qc0 = quiet_count_;
        const uint8_t dc0 = drift_down_count_;

        // ── 모드 플래그 (0/1), | & 만 사용 — 단축 평가 없음 ─────────────
        const uint32_t c_heavy = static_cast<uint32_t>(
            static_cast<uint32_t>(ajc >= AJC_HEAVY_THR)
            | static_cast<uint32_t>(snr < NOISY_SNR_THR));

        const uint32_t c_quiet_raw = static_cast<uint32_t>(
            static_cast<uint32_t>(ajc < AJC_IDLE_THR)
            & static_cast<uint32_t>(snr >= QUIET_SNR_THR));
        const uint32_t c_quiet = c_quiet_raw & (1u - c_heavy);
        const uint32_t c_hold = (1u - c_heavy) & (1u - c_quiet);

        // ── HEAVY → BPS_MIN (마스크), 분기 없음 ─────────────────────────
        const uint8_t m_heavy_u8 = static_cast<uint8_t>(0u - c_heavy);
        uint8_t bps_work = static_cast<uint8_t>(
            static_cast<uint8_t>(BPS_MIN & m_heavy_u8)
            | static_cast<uint8_t>(cur_bps & static_cast<uint8_t>(~m_heavy_u8)));

        // ── QUIET: quiet 카운트 + 상향 (항상 산출, c_quiet로 가중) ───────
        const uint8_t add_q = static_cast<uint8_t>(
            (0u - static_cast<uint32_t>(qc0 < HYST_UP_COUNT)) & 1u);
        const uint8_t qc_try = static_cast<uint8_t>(qc0 + add_q);
        const uint32_t bump_up = static_cast<uint32_t>(
            qc_try >= HYST_UP_COUNT);
        const uint8_t qc_quiet_next = static_cast<uint8_t>(
            static_cast<uint32_t>(qc_try) * (1u - bump_up));

        const uint8_t inc_bps_quiet = static_cast<uint8_t>(
            c_quiet & bump_up
            & static_cast<uint32_t>(bps_work < BPS_MAX));
        bps_work = static_cast<uint8_t>(bps_work + inc_bps_quiet);

        // ── HOLD + DRIFT: drift 카운트 + 하향 (c_hold·c_drift로 가중) ───
        const uint32_t c_drift = static_cast<uint32_t>(
            ajc >= AJC_DRIFT_DOWN_THR);
        const uint8_t add_d = static_cast<uint8_t>(
            (0u - static_cast<uint32_t>(dc0 < HYST_DRIFT_DOWN_COUNT)) & 1u);
        const uint8_t dc1 = static_cast<uint8_t>(dc0 + add_d);
        const uint32_t bump_d = static_cast<uint32_t>(
            dc1 >= HYST_DRIFT_DOWN_COUNT);
        const uint8_t dc_in_zone = static_cast<uint8_t>(
            static_cast<uint32_t>(dc1) * (1u - bump_d));
        const uint8_t dc_hold_next = static_cast<uint8_t>(c_drift * dc_in_zone);

        const uint8_t dec_bps_hold = static_cast<uint8_t>(
            c_hold & c_drift & bump_d
            & static_cast<uint32_t>(bps_work > BPS_MIN));
        bps_work = static_cast<uint8_t>(bps_work - dec_bps_hold);

        // ── quiet_count_: HEAVY=0 / QUIET=qc_quiet_next / HOLD=절반 감쇠 ─
        const uint8_t qc_hold_next = static_cast<uint8_t>(qc0 >> 1u);
        quiet_count_ = static_cast<uint8_t>(
            static_cast<uint32_t>(c_heavy) * 0u
            + static_cast<uint32_t>(c_quiet)
                * static_cast<uint32_t>(qc_quiet_next)
            + static_cast<uint32_t>(c_hold)
                * static_cast<uint32_t>(qc_hold_next));

        // ── drift_down_count_: HEAVY=0 / QUIET=0 / HOLD=drift 경로 값 ──
        drift_down_count_ = static_cast<uint8_t>(
            static_cast<uint32_t>(c_heavy) * 0u
            + static_cast<uint32_t>(c_quiet) * 0u
            + static_cast<uint32_t>(c_hold)
                * static_cast<uint32_t>(dc_hold_next));

        metrics_.current_bps.store(bps_work, std::memory_order_release);
    }

    void HTS_Adaptive_BPS_Controller::Reset() noexcept {
        metrics_.current_bps.store(BPS_MIN, std::memory_order_release);
        quiet_count_ = 0u;
        drift_down_count_ = 0u;
    }

} // namespace ProtectedEngine
