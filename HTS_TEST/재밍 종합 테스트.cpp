// =========================================================================
// main.cpp — HTS B-CDMA DIOC 항재밍 극한 생존성 검증 시뮬레이션
// Target: PC 전용 (Visual Studio / GCC / Clang)
//
// [테스트 시나리오]
//  1. 정상 AWGN       (SNR 5~50dB)
//  2. 바라지 재밍     (J/S 5~50dB)
//  3. 연속파(CW) 재밍  (J/S 5~50dB)
//  4. EMP 폭격        (파괴율 5%~50%)
//
// [측정 항목]
//  - 비트 에러율 (BER)
//  - CRC 성공/실패 (ACK/NACK)
//  - HARQ 재전송 횟수
//  - 최종 지연 시간 (Latency_ms)
//  - 송/수신 텍스트 비교 (한글 깨짐 육안 확인)
//
// [빌드]
//  g++ -std=c++17 -O2 -o hts_test main.cpp HTS_3D_Tensor_FEC.cpp -I.
//  또는 Visual Studio 프로젝트에 추가
// =========================================================================
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || \
    defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#error "[HTS_FATAL] 이 테스트는 PC 전용입니다. ARM 빌드에서 제외하십시오."
#endif

#include "HTS_3D_Tensor_FEC.h"

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <cmath>
#include <random>
#include <chrono>
#include <algorithm>
#include <numeric>
#include <sstream>

using namespace HTS_Engine;

// =========================================================================
//  채널 환경 타입
// =========================================================================
enum class ChannelType {
    AWGN,       // 백색 가우시안 잡음
    BARRAGE,    // 전대역 바라지 재밍
    CW,         // 연속파 (톤) 재밍
    EMP         // 전자기 펄스 버스트
};

static const char* channel_name_kr(ChannelType t) {
    switch (t) {
    case ChannelType::AWGN:    return "정상 AWGN";
    case ChannelType::BARRAGE: return "바라지 재밍";
    case ChannelType::CW:     return "연속파(CW) 재밍";
    case ChannelType::EMP:    return "EMP 폭격";
    }
    return "미정의";
}

static const char* channel_unit(ChannelType t) {
    switch (t) {
    case ChannelType::AWGN:    return "SNR";
    case ChannelType::BARRAGE: return "J/S";
    case ChannelType::CW:     return "J/S";
    case ChannelType::EMP:    return "파괴율";
    }
    return "dB";
}

// =========================================================================
//  파라메트릭 채널 모델 — 4가지 환경 × 가변 강도
//
//  기존 LTE_Channel::Transmit_To의 고정 파라미터를 해체하고
//  실시간으로 환경 파라미터를 주입하는 구조.
//  _To 패턴: 루프 내 힙 할당 0회.
// =========================================================================
static void apply_channel(
    const std::vector<double>& tx,
    std::mt19937& rng,
    std::vector<double>& rx,
    ChannelType type,
    double intensity_db)
{
    const size_t N = tx.size();
    rx.resize(N);

    static constexpr int NUM_CHIPS = 128;
    const double spread_gain = static_cast<double>(NUM_CHIPS);

    // 기본 열잡음 (SNR 40dB 기준)
    const double base_noise_sigma = 0.01;
    std::normal_distribution<double> base_noise(0.0, base_noise_sigma);

    switch (type) {

    case ChannelType::AWGN: {
        // SNR(dB) → 신호 대비 잡음 전력
        // 낮은 SNR = 높은 잡음 → intensity_db가 SNR이므로 역수
        const double snr_linear = std::pow(10.0, intensity_db / 10.0);
        const double signal_power = spread_gain * spread_gain;
        const double noise_sigma =
            std::sqrt(signal_power / snr_linear);
        std::normal_distribution<double> awgn(0.0, noise_sigma);
        for (size_t i = 0u; i < N; ++i) {
            rx[i] = tx[i] * spread_gain + awgn(rng);
        }
        break;
    }

    case ChannelType::BARRAGE: {
        // J/S(dB) → 재밍 전력 / 신호 전력
        const double js_linear = std::pow(10.0, intensity_db / 10.0);
        const double jam_sigma =
            std::sqrt(js_linear) * spread_gain;
        std::normal_distribution<double> jam(0.0, jam_sigma);
        for (size_t i = 0u; i < N; ++i) {
            rx[i] = tx[i] * spread_gain + jam(rng) + base_noise(rng);
        }
        break;
    }

    case ChannelType::CW: {
        // CW 재밍: 특정 주파수 톤 (center = N/4, width = N/16)
        const double js_linear = std::pow(10.0, intensity_db / 10.0);
        const double cw_amp =
            std::sqrt(js_linear) * spread_gain * 2.0;
        const size_t cw_center = N / 4u;
        const size_t cw_width = N / 16u;
        std::uniform_real_distribution<double> phase_dist(0.0, 6.2831853);
        const double cw_phase = phase_dist(rng);

        for (size_t i = 0u; i < N; ++i) {
            double cw = 0.0;
            // CW 간섭: 중심 주파수 ± 대역폭 내 사인파
            if (i >= (cw_center - cw_width) &&
                i < (cw_center + cw_width)) {
                const double t = static_cast<double>(i - cw_center + cw_width)
                    / static_cast<double>(cw_width * 2u);
                cw = cw_amp * std::sin(2.0 * 3.14159265 * 8.0 * t + cw_phase);
            }
            rx[i] = tx[i] * spread_gain + cw + base_noise(rng);
        }
        break;
    }

    case ChannelType::EMP: {
        // EMP: 파괴율(%) = intensity_db를 % 단위로 사용
        const double destroy_rate = intensity_db / 100.0;
        const double emp_amp = 99999.0;
        std::uniform_real_distribution<double> u01(0.0, 1.0);
        for (size_t i = 0u; i < N; ++i) {
            if (u01(rng) < destroy_rate) {
                // 칩 완전 파괴 → 랜덤 극단값
                rx[i] = ((u01(rng) > 0.5) ? 1.0 : -1.0) * emp_amp;
            }
            else {
                rx[i] = tx[i] * spread_gain + base_noise(rng);
            }
        }
        break;
    }
    } // switch
}

// =========================================================================
//  HARQ 블록 전송 (파라메트릭 채널 사용)
// =========================================================================
struct TestBlockResult {
    std::vector<double> info_bits;
    int    harq_rounds = 0;
    bool   crc_pass = false;
    double latency_ms = 0.0;
    int    bit_errors = 0;
    int    total_bits = 0;
    double ber = 0.0;
};

static TestBlockResult transmit_block_parametric(
    const std::vector<double>& info_bits,
    unsigned int block_seed,
    Soft_Tensor_FEC& fec,
    Tensor_Interleaver& interleaver,
    std::mt19937& rng,
    ChannelType ch_type,
    double ch_intensity,
    // 사전 할당 버퍼 (루프 내 힙 0회)
    std::vector<double>& tensor_buf,
    std::vector<double>& tx_buf,
    std::vector<double>& rx_buf,
    std::vector<double>& rx_dint_buf,
    std::vector<double>& soft_buf,
    std::vector<double>& harq_accum,
    std::vector<double>& combined,
    std::vector<double>& last_hard)
{
    static constexpr int MTU = LTE_HARQ_Controller::MTU;
    static constexpr int REP = LTE_HARQ_Controller::REP_FACTOR;
    static constexpr int PROT = LTE_HARQ_Controller::PROTECTED_BITS;
    static constexpr int MAX_K = LTE_HARQ_Controller::MAX_HARQ;
    static constexpr double RTT = LTE_HARQ_Controller::HARQ_RTT_MS;

    TestBlockResult result;

    // CRC 부착 + 반복 코딩
    std::vector<double> prot = CRC16::Append(info_bits);
    std::vector<double> coded(MTU, 1.0);
    for (int r = 0; r < REP; ++r)
        for (size_t i = 0u; i < prot.size(); ++i) {
            size_t pos = static_cast<size_t>(r) * prot.size() + i;
            if (pos < static_cast<size_t>(MTU))
                coded[pos] = prot[i];
        }

    std::fill(harq_accum.begin(), harq_accum.end(), 0.0);

    for (int k = 1; k <= MAX_K; ++k) {
        result.harq_rounds = k;
        result.latency_ms = static_cast<double>(k) * RTT;

        unsigned int fseed =
            block_seed + static_cast<unsigned int>(k) * 7919u;

        // _To API: 힙 할당 0회
        fec.Encode_To(coded, fseed, tensor_buf);
        interleaver.Interleave_To(tensor_buf, tx_buf);

        // ★ 파라메트릭 채널 적용
        apply_channel(tx_buf, rng, rx_buf, ch_type, ch_intensity);

        interleaver.Deinterleave_To(rx_buf, rx_dint_buf);
        fec.Decode_Soft_To(rx_dint_buf, MTU, fseed, soft_buf);

        for (int i = 0; i < MTU; ++i)
            harq_accum[static_cast<size_t>(i)] += soft_buf[static_cast<size_t>(i)];

        // 반복 결합 + 경판정
        std::fill(combined.begin(), combined.end(), 0.0);
        for (int r = 0; r < REP; ++r)
            for (int i = 0; i < PROT; ++i) {
                size_t pos = static_cast<size_t>(r) * PROT + i;
                if (pos < static_cast<size_t>(MTU))
                    combined[static_cast<size_t>(i)] += harq_accum[pos];
            }
        for (int i = 0; i < PROT; ++i)
            last_hard[static_cast<size_t>(i)] =
            (combined[static_cast<size_t>(i)] > 0.0) ? 1.0 : -1.0;

        if (CRC16::Check(last_hard)) {
            result.info_bits.assign(
                last_hard.begin(), last_hard.begin() + (PROT - 16));
            result.crc_pass = true;
            return result;
        }
    }

    // MAX_HARQ 초과: 최종 경판정 결과 반환 (CRC 실패)
    result.info_bits.assign(
        last_hard.begin(), last_hard.begin() + (PROT - 16));
    result.crc_pass = false;
    return result;
}

// =========================================================================
//  BER 계산
// =========================================================================
static void compute_ber(
    const std::vector<double>& original,
    const std::vector<double>& received,
    int& errors, int& total, double& ber)
{
    errors = 0;
    total = static_cast<int>(
        std::min(original.size(), received.size()));
    for (int i = 0; i < total; ++i) {
        const int orig = (original[static_cast<size_t>(i)] > 0.0) ? 1 : 0;
        const int recv = (received[static_cast<size_t>(i)] > 0.0) ? 1 : 0;
        if (orig != recv) { ++errors; }
    }
    ber = (total > 0) ? static_cast<double>(errors) / total : 1.0;
}

// =========================================================================
//  전체 메시지 송수신 (블록 분할)
// =========================================================================
struct MessageResult {
    std::string rx_text;
    int    total_blocks = 0;
    int    success_blocks = 0;
    int    total_harq = 0;
    double max_latency_ms = 0.0;
    int    total_bit_errors = 0;
    int    total_bits = 0;
    double ber = 0.0;
};

static MessageResult send_full_message(
    const std::string& message,
    Soft_Tensor_FEC& fec,
    Tensor_Interleaver& interleaver,
    std::mt19937& rng,
    ChannelType ch_type,
    double ch_intensity)
{
    static constexpr int INFO = LTE_HARQ_Controller::INFO_PER_BLOCK;
    static constexpr int MTU = LTE_HARQ_Controller::MTU;
    static constexpr int PROT = LTE_HARQ_Controller::PROTECTED_BITS;

    MessageResult msg;

    // 텍스트 → 비트 변환
    std::vector<double> all_bits = Text_Codec::String_To_Bits(message);
    const int num_blocks =
        (static_cast<int>(all_bits.size()) + INFO - 1) / INFO;
    msg.total_blocks = num_blocks;

    // ── 사전 할당 (루프 내 힙 0회) ────────────────────────────
    const size_t tensor_size = interleaver.Get_Size();
    std::vector<double> tensor_buf(tensor_size, 0.0);
    std::vector<double> tx_buf(tensor_size, 0.0);
    std::vector<double> rx_buf(tensor_size, 0.0);
    std::vector<double> rx_dint_buf(tensor_size, 0.0);
    std::vector<double> soft_buf(MTU, 0.0);
    std::vector<double> harq_accum(MTU, 0.0);
    std::vector<double> combined(PROT, 0.0);
    std::vector<double> last_hard(PROT, 0.0);

    std::vector<double> all_rx_bits;
    all_rx_bits.reserve(all_bits.size());

    for (int b = 0; b < num_blocks; ++b) {
        // 블록 추출
        const int start = b * INFO;
        const int end = std::min(start + INFO,
            static_cast<int>(all_bits.size()));
        std::vector<double> block_bits(
            all_bits.begin() + start,
            all_bits.begin() + end);

        // 패딩 (INFO 미만 시)
        while (static_cast<int>(block_bits.size()) < INFO)
            block_bits.push_back(1.0);

        unsigned int bseed = static_cast<unsigned int>(b * 31337u + 42u);

        TestBlockResult br = transmit_block_parametric(
            block_bits, bseed, fec, interleaver, rng,
            ch_type, ch_intensity,
            tensor_buf, tx_buf, rx_buf, rx_dint_buf,
            soft_buf, harq_accum, combined, last_hard);

        if (br.crc_pass) { msg.success_blocks++; }
        msg.total_harq += br.harq_rounds;
        if (br.latency_ms > msg.max_latency_ms)
            msg.max_latency_ms = br.latency_ms;

        // BER 계산
        int err = 0, tot = 0; double blk_ber = 0.0;
        compute_ber(block_bits, br.info_bits, err, tot, blk_ber);
        msg.total_bit_errors += err;
        msg.total_bits += tot;

        // 수신 비트 누적
        const int copy_len = end - start;
        for (int i = 0; i < copy_len &&
            i < static_cast<int>(br.info_bits.size()); ++i)
            all_rx_bits.push_back(br.info_bits[static_cast<size_t>(i)]);
    }

    msg.rx_text = Text_Codec::Bits_To_String(all_rx_bits);
    msg.ber = (msg.total_bits > 0)
        ? static_cast<double>(msg.total_bit_errors) / msg.total_bits
        : 1.0;

    return msg;
}

// =========================================================================
//  리포트 출력
// =========================================================================
static void print_header() {
    std::cout << "\n";
    std::cout << std::string(100, '=') << "\n";
    std::cout << "  HTS B-CDMA DIOC 항재밍 극한 생존성 검증 시뮬레이션\n";
    std::cout << "  INNOViD CORE-X Pro | Tensor FEC + HARQ + 3층 항재밍\n";
    std::cout << std::string(100, '=') << "\n\n";
}

static void print_scenario_header(ChannelType type) {
    std::cout << "\n" << std::string(100, '-') << "\n";
    std::cout << "  [시나리오] " << channel_name_kr(type) << "\n";
    std::cout << std::string(100, '-') << "\n";
    std::cout << std::setw(8) << "강도"
        << std::setw(10) << "단위"
        << std::setw(10) << "블록수"
        << std::setw(10) << "성공"
        << std::setw(10) << "CRC"
        << std::setw(12) << "BER"
        << std::setw(10) << "HARQ"
        << std::setw(12) << "지연(ms)"
        << "  결과\n";
    std::cout << std::string(100, '-') << "\n";
}

static void print_row(double intensity, const char* unit,
    const MessageResult& r)
{
    const char* verdict =
        (r.success_blocks == r.total_blocks) ? "PASS" :
        (r.success_blocks > 0) ? "PARTIAL" : "FAIL";

    std::cout << std::fixed
        << std::setw(8) << std::setprecision(0) << intensity
        << std::setw(10) << unit
        << std::setw(10) << r.total_blocks
        << std::setw(10) << r.success_blocks
        << std::setw(10)
        << ((r.success_blocks == r.total_blocks) ? "ACK" : "NACK")
        << std::setw(12) << std::scientific << std::setprecision(2)
        << r.ber
        << std::setw(10) << std::fixed << std::setprecision(0)
        << r.total_harq
        << std::setw(12) << std::setprecision(1) << r.max_latency_ms
        << "  " << verdict << "\n";
}

// =========================================================================
//  메인
// =========================================================================
int main() {
    // ── 타이머 ──
    auto t0 = std::chrono::steady_clock::now();

    print_header();

    // ── 테스트 페이로드 ──
    const std::string payload =
        "HTS-V400 Tensor Engine Test 2026! "
        "[긴급] 고도 15,000ft에서 적대적 재밍 공격 감지. "
        "바라지 50dB 환경 생존율 100% 검증 프로토콜 가동 중. "
        "Target Coordinates: N 37.5665, E 126.9780. "
        "Payload Confirmed. "
        "재난안전망 B-CDMA 통신 무결성 검증: "
        "ARIA-256/LEA-256 이중 암호화 적용. "
        "홀로그래픽 텐서 자가치유 활성. "
        "EMP 내성 테스트 #7-Alpha 진행 중.";

    std::cout << "  [송신 원본]\n";
    std::cout << "  " << payload << "\n";
    std::cout << "  (길이: " << payload.size() << " 바이트, "
        << payload.size() * 8 << " 비트)\n\n";

    // ── 엔진 초기화 (1회 할당) ──
    Soft_Tensor_FEC fec;
    Tensor_Interleaver interleaver(64);
    std::mt19937 rng(20260331u);  // 고정 시드 (재현성)

    // ── 강도 스텝 정의 ──
    const double steps[] = { 5, 10, 15, 20, 25, 30, 35, 40, 45, 50 };
    const int num_steps = sizeof(steps) / sizeof(steps[0]);

    // ── 시나리오별 결과 누적 ──
struct ScenarioSummary {
    ChannelType type = ChannelType::AWGN; // [FIX] 초기값 명시적 할당 (경고 C26495 완벽 소멸)
    int pass_count = 0;
    int partial_count = 0;
    int fail_count = 0;
    double max_survivable_db = 0.0;
};
    ScenarioSummary summaries[4];
    summaries[0].type = ChannelType::AWGN;
    summaries[1].type = ChannelType::BARRAGE;
    summaries[2].type = ChannelType::CW;
    summaries[3].type = ChannelType::EMP;

    // ═══════════════════════════════════════════════════════
    //  4 시나리오 × 10 강도 = 40회 테스트
    // ═══════════════════════════════════════════════════════
    ChannelType scenarios[] = {
        ChannelType::AWGN,
        ChannelType::BARRAGE,
        ChannelType::CW,
        ChannelType::EMP
    };

    for (int s = 0; s < 4; ++s) {
        ChannelType ct = scenarios[s];
        print_scenario_header(ct);

        for (int step = 0; step < num_steps; ++step) {
            double intensity = steps[step];

            // RNG 리셋 (각 테스트 독립)
            rng.seed(20260331u + static_cast<unsigned>(s * 1000 + step));

            MessageResult mr = send_full_message(
                payload, fec, interleaver, rng, ct, intensity);

            print_row(intensity, channel_unit(ct), mr);

            // 텍스트 비교 (첫 스텝, 마지막 스텝에서 출력)
            if (step == 0 || step == num_steps - 1 ||
                mr.success_blocks != mr.total_blocks) {
                std::cout << "    [수신] "
                    << mr.rx_text.substr(0,
                        std::min<size_t>(80, mr.rx_text.size()));
                if (mr.rx_text.size() > 80) std::cout << "...";
                std::cout << "\n";
            }

            // 요약 누적
            if (mr.success_blocks == mr.total_blocks) {
                summaries[s].pass_count++;
                summaries[s].max_survivable_db = intensity;
            }
            else if (mr.success_blocks > 0) {
                summaries[s].partial_count++;
            }
            else {
                summaries[s].fail_count++;
            }
        }
    }

    // ═══════════════════════════════════════════════════════
    //  최종 요약
    // ═══════════════════════════════════════════════════════
    auto t1 = std::chrono::steady_clock::now();
    double elapsed =
        std::chrono::duration<double>(t1 - t0).count();

    std::cout << "\n" << std::string(100, '=') << "\n";
    std::cout << "  최종 생존성 분석 결과\n";
    std::cout << std::string(100, '=') << "\n\n";

    std::cout << std::setw(20) << "환경"
        << std::setw(10) << "PASS"
        << std::setw(12) << "PARTIAL"
        << std::setw(10) << "FAIL"
        << std::setw(20) << "최대 생존 강도"
        << "\n";
    std::cout << std::string(72, '-') << "\n";

    for (int s = 0; s < 4; ++s) {
        std::cout << std::setw(20) << channel_name_kr(summaries[s].type)
            << std::setw(10) << summaries[s].pass_count
            << std::setw(12) << summaries[s].partial_count
            << std::setw(10) << summaries[s].fail_count
            << std::setw(15) << std::fixed << std::setprecision(0)
            << summaries[s].max_survivable_db
            << " " << channel_unit(summaries[s].type) << "\n";
    }

    std::cout << "\n  총 테스트: 40회 (4환경 x 10단계)\n";
    std::cout << "  실행 시간: " << std::fixed << std::setprecision(1)
        << elapsed << "초\n";
    std::cout << "  페이로드:  " << payload.size() << " 바이트 ("
        << payload.size() * 8 << " 비트)\n";
    std::cout << std::string(100, '=') << "\n";

    return 0;
}