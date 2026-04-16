// =============================================================================
// HTS_TPC_Controller.h — Transmit Power Control (AND/OR/SHIFT 전용)
// =============================================================================
// LPI 은닉 마진 극대화를 위한 적응형 TX 전력 제어.
// 수신측 RSSI 피드백 → 송신 진폭을 최소 필요량으로 자동 축소.
//
// 설계 원칙:
//   - 곱셈/나눗셈/float/double 전면 금지
//   - 전력 레벨은 LUT (shift 기반 사전 계산)
//   - RSSI 측정은 |I|+|Q| MAG1 + shift 정규화
//   - 히스테리시스: 진동 방지 (up 문턱 ≠ down 문턱)
//   - Zero heap, static 전용
//
// @warning sizeof(HTS_TPC_Controller) ≈ 32 bytes
// =============================================================================
#ifndef HTS_TPC_CONTROLLER_H
#define HTS_TPC_CONTROLLER_H

#include <cstdint>

namespace ProtectedEngine {

class HTS_TPC_Controller {
public:
    // ── 전력 레벨 정의 (3dB 스텝, 9단계) ──
    // 각 레벨의 amp 값: amp[n] ≈ MAX_AMP >> (n/2)
    // 3dB = 10^(3/10) ≈ 2.0 (전력), √2 ≈ 1.414 (진폭)
    // shift 근사: ×1.414 ≈ x + (x>>1) - (x>>3) - (x>>5)
    //             ÷1.414 ≈ (x>>1) + (x>>2) - (x>>5)
    // → LUT로 사전 계산하여 런타임 연산 제거
    static constexpr int32_t kLevelCount = 9;
    static constexpr int32_t kLevelMax = 0;   // 최대 출력 (0dB)
    static constexpr int32_t kLevelMin = 3;   // 최소 출력 (-9dB), 은닉 +27dB
    static constexpr int32_t kLevelInit = 4;  // 초기 레벨 (-12dB)

    // 전력 레벨 LUT (인덱스 = 레벨, 값 = 진폭)
    // Level 0: 1024 (0dB)     Level 4: 256 (-12dB)
    // Level 1:  724 (-3dB)    Level 5: 181 (-15dB)
    // Level 2:  512 (-6dB)    Level 6: 128 (-18dB)
    // Level 3:  362 (-9dB)    Level 7:  90 (-21dB)
    //                         Level 8:  64 (-24dB)
    //
    // 각 값 = 1024 / (√2)^n, 정수 반올림
    // 검증: 724² / 1024² ≈ 0.500 = -3.01dB ✓
    static constexpr int16_t kAmpLut[kLevelCount] = {
        1024, // Level 0:   0 dB
        724,  // Level 1:  -3 dB  (1024×0.707)
        512,  // Level 2:  -6 dB  (1024>>1)
        362,  // Level 3:  -9 dB  (512×0.707)
        256,  // Level 4: -12 dB  (1024>>2)
        181,  // Level 5: -15 dB  (256×0.707)
        128,  // Level 6: -18 dB  (1024>>3)
        90,   // Level 7: -21 dB  (128×0.707)
        64,   // Level 8: -24 dB  (1024>>4)
    };

    // dB 오프셋 LUT (보고용, 연산에는 미사용)
    static constexpr int32_t kDbLut[kLevelCount] = {
        0, -3, -6, -9, -12, -15, -18, -21, -24};

    // ── RSSI 목표 및 히스테리시스 ──
    // RSSI = Σ(|I|+|Q|) >> shift (MAG1 근사, 칩 N개)
    // 목표: kAmpLut[level] × 64칩 기준
    //
    // 히스테리시스 (shift 기반):
    //   상향 문턱 = target + (target >> 2)  = target × 1.25
    //   하향 문턱 = target - (target >> 2)  = target × 0.75
    //   → 0.75~1.25 범위에서 유지, 진동 방지
    static constexpr int32_t kRssiWindowChips = 64; // RSSI 측정 윈도우
    static constexpr int32_t kRssiShift = 0;        // 정규화 시프트

    // ── 피드백 인코딩 (2비트, ACK에 삽입) ──
    // 0b00 = HOLD   (현재 유지)
    // 0b01 = DOWN_1 (1단계 감소 = -3dB)
    // 0b10 = DOWN_2 (2단계 감소 = -6dB)
    // 0b11 = UP_1   (1단계 증가 = +3dB)
    static constexpr uint8_t kFbHold = 0x00u;
    static constexpr uint8_t kFbDown1 = 0x01u;
    static constexpr uint8_t kFbDown2 = 0x02u;
    static constexpr uint8_t kFbUp1 = 0x03u;
    static constexpr uint8_t kFbMask = 0x03u;

    // ── API ──

    /// @brief 초기화 (부팅 시 1회)
    void Init() noexcept;

    /// @brief 현재 TX 진폭 반환 (Build_Packet의 amp 인자로 사용)
    /// @return kAmpLut[current_level_]
    int16_t Get_Tx_Amp() const noexcept;

    /// @brief 현재 레벨 반환 (0=최대, 8=최소)
    int32_t Get_Level() const noexcept;

    /// @brief 현재 레벨의 dB 오프셋 반환
    int32_t Get_Level_dB() const noexcept;

    /// @brief [RX측] 수신 칩 에너지로 RSSI 측정 → 2비트 피드백 생성
    /// @param chip_I  수신 I 칩 배열
    /// @param chip_Q  수신 Q 칩 배열
    /// @param n_chips 칩 수 (64 권장)
    /// @param target_amp TX측이 사용한 amp 값 (HDR에서 파싱 또는 협상)
    /// @return 2비트 피드백 코드 (kFbHold/Down1/Down2/Up1)
    /// @note 모든 연산 AND/OR/SHIFT, 곱셈 0회
    static uint8_t Measure_RSSI_Feedback(const int16_t *chip_I,
                                         const int16_t *chip_Q,
                                         int32_t n_chips,
                                         int16_t target_amp) noexcept;

    /// @brief [TX측] 피드백 수신 → 전력 레벨 조정
    /// @param fb 2비트 피드백 코드
    void Apply_Feedback(uint8_t fb) noexcept;

    /// @brief [TX측] 레벨 직접 설정 (테스트용)
    /// @param level 0~8
    void Set_Level(int32_t level) noexcept;

    /// @brief 최대 출력으로 리셋 (링크 재설정 시)
    void Reset_To_Max() noexcept;

    /// @brief 최소 출력으로 설정 (LPI 극대화)
    void Set_Min_Power() noexcept;

    // ── 페이로드 피드백 삽입/추출 (info[7] 상위 2비트) ──

    /// @brief [RX→TX] 디코딩 성공한 info 배열에 TPC 피드백 삽입
    /// @param info 8바이트 페이로드 (info[7] 상위 2비트 덮어쓰기)
    /// @param fb 2비트 피드백 코드
    /// @note 원본 info[7]의 상위 2비트는 손실됨 → 프로토콜 예약
    static void Embed_Feedback(uint8_t* info, uint8_t fb) noexcept {
        if (info == nullptr) return;
        info[7] = static_cast<uint8_t>(
            (info[7] & 0x3Fu) |
            (static_cast<uint8_t>(fb & kFbMask) << 6u));
    }

    /// @brief [TX측] 수신한 ACK 페이로드에서 TPC 피드백 추출
    /// @param info 8바이트 수신 페이로드
    /// @return 2비트 피드백 코드
    static uint8_t Extract_Feedback(const uint8_t* info) noexcept {
        if (info == nullptr) return kFbHold;
        return static_cast<uint8_t>((info[7] >> 6u) & kFbMask);
    }

    // ── LPI 은닉 마진 계산 (보고용, 연산에 shift만 사용) ──
    /// @brief 현재 레벨의 칩 SNR 마진 (dB, 정수)
    /// @param walsh_pg_db Walsh 처리이득 (18 for 64칩, 12 for 16칩)
    /// @return 은닉마진 = walsh_pg_db - |level_dB| (양수=잡음 아래)
    int32_t Get_Hiding_Margin_dB(int32_t walsh_pg_db) const noexcept;

private:
    int32_t current_level_ = kLevelInit;
};

// ── 인라인 구현 (헤더 전용, 링크 의존성 없음) ──

inline void HTS_TPC_Controller::Init() noexcept { current_level_ = kLevelInit; }

inline int16_t HTS_TPC_Controller::Get_Tx_Amp() const noexcept {
    const int32_t idx = current_level_ & 0x0F;
    return (idx < kLevelCount) ? kAmpLut[idx] : kAmpLut[kLevelMin];
}

inline int32_t HTS_TPC_Controller::Get_Level() const noexcept {
    return current_level_;
}

inline int32_t HTS_TPC_Controller::Get_Level_dB() const noexcept {
    const int32_t idx = current_level_ & 0x0F;
    return (idx < kLevelCount) ? kDbLut[idx] : kDbLut[kLevelMin];
}

inline uint8_t
HTS_TPC_Controller::Measure_RSSI_Feedback(const int16_t *chip_I,
                                           const int16_t *chip_Q,
                                           int32_t n_chips,
                                           int16_t target_amp) noexcept {
    if (n_chips <= 0)
        return kFbHold;

    int32_t sum = 0;
    for (int32_t i = 0; i < n_chips; ++i) {
        const int32_t vi = static_cast<int32_t>(chip_I[i]);
        const int32_t vq = static_cast<int32_t>(chip_Q[i]);
        const int32_t si = vi >> 31;
        const int32_t ai = (vi ^ si) - si;
        const int32_t sq = vq >> 31;
        const int32_t aq = (vq ^ sq) - sq;
        sum += ai + aq;
    }

    int32_t shift = 0;
    {
        int32_t tmp = n_chips;
        while (tmp > 1) {
            tmp >>= 1;
            ++shift;
        }
    }
    const int32_t rssi_avg = sum >> shift;

    const int32_t target = static_cast<int32_t>(target_amp);

    const int32_t th_up = target + (target >> 2);
    const int32_t th_down = target - (target >> 2);

    const int32_t th_down2 = target + (target >> 1) + (target >> 2);

    if (rssi_avg > th_down2) {
        return kFbDown2;
    }
    if (rssi_avg > th_up) {
        return kFbDown1;
    }
    if (rssi_avg < th_down) {
        return kFbUp1;
    }
    return kFbHold;
}

inline void HTS_TPC_Controller::Apply_Feedback(uint8_t fb) noexcept {
    const uint8_t cmd = fb & kFbMask;

    if (cmd == kFbDown1) {
        if (current_level_ < kLevelMin) {
            current_level_ = current_level_ + 1;
        }
    } else if (cmd == kFbDown2) {
        int32_t next = current_level_ + 2;
        if (next > kLevelMin)
            next = kLevelMin;
        current_level_ = next;
    } else if (cmd == kFbUp1) {
        if (current_level_ > kLevelMax) {
            current_level_ = current_level_ - 1;
        }
    }
}

inline void HTS_TPC_Controller::Set_Level(int32_t level) noexcept {
    if (level < kLevelMax)
        level = kLevelMax;
    if (level > kLevelMin)
        level = kLevelMin;
    current_level_ = level;
}

inline void HTS_TPC_Controller::Reset_To_Max() noexcept {
    current_level_ = kLevelMax;
}

inline void HTS_TPC_Controller::Set_Min_Power() noexcept {
    current_level_ = kLevelMin;
}

inline int32_t
HTS_TPC_Controller::Get_Hiding_Margin_dB(int32_t walsh_pg_db) const noexcept {
    const int32_t idx = current_level_ & 0x0F;
    const int32_t backoff = (idx < kLevelCount) ? -kDbLut[idx] : 24;
    return walsh_pg_db + backoff;
}

} // namespace ProtectedEngine

#endif // HTS_TPC_CONTROLLER_H
