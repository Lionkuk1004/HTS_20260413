// =========================================================================
// 재밍 종합 테스트.cpp — HTS B-CDMA DIOC 항재밍 생존성 검증 (PC 전용)
//
//  목적
//  ----
//  Tensor FEC + 인터리버 + (가상) LTE HARQ 루프를 한 파이프라인으로 묶은 뒤,
//  네 가지 채널 모델에서 강도를 바꿔 가며 블록 성공률·BER·HARQ·지연을 본다.
//
//  데이터 경로 (블록 단위)
//  ----------------------
//  UTF-8 텍스트 → Text_Codec::String_To_Bits (±1 비트)
//       → INFO 비트씩 분할 → CRC16 부착 → 반복(REP) 코딩 → MTU
//       → Soft_Tensor_FEC::Encode_To → 텐서
//       → Tensor_Interleaver::Interleave_To
//       → apply_channel (AWGN / 바라지 / CW / EMP)
//       → Deinterleave → Decode_Soft_To (소프트)
//       → HARQ 누적·경판정 → CRC 검사 (최대 MAX_HARQ 라운드)
//
//  시나리오 매트릭스
//  ----------------
//  4종 채널 × 10단계 강도 = 총 40회 (각 회는 페이로드 전체 재전송 시뮬)
//
//  빌드: HTS_TEST 프로젝트 (HTS_LIM 정적 라이브러리 링크)
// =========================================================================
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || \
    defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#error "[HTS_FATAL] 이 테스트는 PC 전용입니다. ARM 빌드에서 제외하십시오."
#endif

#include "HTS_3D_Tensor_FEC.h"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>
#include <algorithm>

using namespace HTS_Engine;

namespace {

// -------------------------------------------------------------------------
//  실행 상수 (표·설명 문자열과 공유)
// -------------------------------------------------------------------------
constexpr unsigned kRngSeedBase = 20260331u;

/// 강도 스윕: AWGN·바라지·CW는 dB 스케일, EMP는 파괴율 %로 동일 배열 재사용
const double kIntensitySteps[] = {
    5, 10, 15, 20, 25, 30, 35, 40, 45, 50
};
const int kNumIntensitySteps =
    static_cast<int>(sizeof(kIntensitySteps) / sizeof(kIntensitySteps[0]));

const int kNumScenarios = 4;

// LTE_HARQ_Controller 기준 (헤더와 동일 값 — 리포트에 숫자로 출력)
constexpr int kMtu = LTE_HARQ_Controller::MTU;
constexpr int kRep = LTE_HARQ_Controller::REP_FACTOR;
constexpr int kProt = LTE_HARQ_Controller::PROTECTED_BITS;
constexpr int kMaxHarq = LTE_HARQ_Controller::MAX_HARQ;
constexpr int kInfoPerBlock = LTE_HARQ_Controller::INFO_PER_BLOCK;
constexpr double kHarqRttMs = LTE_HARQ_Controller::HARQ_RTT_MS;

// -------------------------------------------------------------------------
//  채널 종류
// -------------------------------------------------------------------------
enum class ChannelType {
    AWGN,    ///< 백색 가우시안 잡음 (intensity = SNR dB)
    BARRAGE, ///< 전대역 바라지 (intensity = J/S dB)
    CW,      ///< 연속파 톤 재밍 (intensity = J/S dB)
    EMP      ///< 펄스 버스트 (intensity = 파괴율 % , 0~100)
};

const char* channel_name_kr(ChannelType t) {
    switch (t) {
    case ChannelType::AWGN:    return "정상 AWGN";
    case ChannelType::BARRAGE: return "바라지 재밍";
    case ChannelType::CW:      return "연속파(CW) 재밍";
    case ChannelType::EMP:     return "EMP 폭격";
    }
    return "미정의";
}

const char* channel_intensity_unit(ChannelType t) {
    switch (t) {
    case ChannelType::AWGN:    return "SNR(dB)";
    case ChannelType::BARRAGE: return "J/S(dB)";
    case ChannelType::CW:      return "J/S(dB)";
    case ChannelType::EMP:     return "파괴(%)";
    }
    return "";
}

/// 표 헤더용 짧은 단위 라벨
const char* channel_unit_short(ChannelType t) {
    switch (t) {
    case ChannelType::AWGN:    return "SNR";
    case ChannelType::BARRAGE: return "J/S";
    case ChannelType::CW:      return "J/S";
    case ChannelType::EMP:     return "%";
    }
    return "";
}

const char* channel_intensity_hint_kr(ChannelType t) {
    switch (t) {
    case ChannelType::AWGN:
        return "높을수록 신호 대 잡음비 양호 (수신 유리)";
    case ChannelType::BARRAGE:
    case ChannelType::CW:
        return "높을수록 재밍 전력 우세 (수신 불리)";
    case ChannelType::EMP:
        return "칩 일부를 극단값으로 파괴하는 비율(%) — 높을수록 불리";
    }
    return "";
}

// -------------------------------------------------------------------------
//  파라메트릭 채널 (고정 NUM_CHIPS=128, 스프레드 이득 반영)
//  — 루프 내 힙 할당 없음 (rx는 호출자가 크기 맞춤)
// -------------------------------------------------------------------------
void apply_channel(
    const std::vector<double>& tx,
    std::mt19937& rng,
    std::vector<double>& rx,
    ChannelType type,
    double intensity_db)
{
    const size_t N = tx.size();
    rx.resize(N);

    static constexpr int kNumChips = 128;
    const double spread_gain = static_cast<double>(kNumChips);
    const double base_noise_sigma = 0.01;
    std::normal_distribution<double> base_noise(0.0, base_noise_sigma);

    switch (type) {

    case ChannelType::AWGN: {
        const double snr_linear = std::pow(10.0, intensity_db / 10.0);
        const double signal_power = spread_gain * spread_gain;
        const double noise_sigma = std::sqrt(signal_power / snr_linear);
        std::normal_distribution<double> awgn(0.0, noise_sigma);
        for (size_t i = 0u; i < N; ++i)
            rx[i] = tx[i] * spread_gain + awgn(rng);
        break;
    }

    case ChannelType::BARRAGE: {
        const double js_linear = std::pow(10.0, intensity_db / 10.0);
        const double jam_sigma = std::sqrt(js_linear) * spread_gain;
        std::normal_distribution<double> jam(0.0, jam_sigma);
        for (size_t i = 0u; i < N; ++i)
            rx[i] = tx[i] * spread_gain + jam(rng) + base_noise(rng);
        break;
    }

    case ChannelType::CW: {
        const double js_linear = std::pow(10.0, intensity_db / 10.0);
        const double cw_amp = std::sqrt(js_linear) * spread_gain * 2.0;
        const size_t cw_center = N / 4u;
        const size_t cw_width = N / 16u;
        std::uniform_real_distribution<double> phase_dist(0.0, 6.2831853);
        const double cw_phase = phase_dist(rng);

        for (size_t i = 0u; i < N; ++i) {
            double cw = 0.0;
            if (i >= (cw_center - cw_width) && i < (cw_center + cw_width)) {
                const double t = static_cast<double>(i - cw_center + cw_width)
                    / static_cast<double>(cw_width * 2u);
                cw = cw_amp * std::sin(2.0 * 3.14159265 * 8.0 * t + cw_phase);
            }
            rx[i] = tx[i] * spread_gain + cw + base_noise(rng);
        }
        break;
    }

    case ChannelType::EMP: {
        const double destroy_rate = intensity_db / 100.0;
        const double emp_amp = 99999.0;
        std::uniform_real_distribution<double> u01(0.0, 1.0);
        for (size_t i = 0u; i < N; ++i) {
            if (u01(rng) < destroy_rate)
                rx[i] = ((u01(rng) > 0.5) ? 1.0 : -1.0) * emp_amp;
            else
                rx[i] = tx[i] * spread_gain + base_noise(rng);
        }
        break;
    }

    } // switch
}

// -------------------------------------------------------------------------
//  단일 블록: HARQ 루프 (버퍼 재사용)
// -------------------------------------------------------------------------
struct TestBlockResult {
    std::vector<double> info_bits;
    int    harq_rounds = 0;
    bool   crc_pass = false;
    double latency_ms = 0.0;
    int    bit_errors = 0;
    int    total_bits = 0;
    double ber = 0.0;
};

TestBlockResult transmit_block_parametric(
    const std::vector<double>& info_bits,
    unsigned int block_seed,
    Soft_Tensor_FEC& fec,
    Tensor_Interleaver& interleaver,
    std::mt19937& rng,
    ChannelType ch_type,
    double ch_intensity,
    std::vector<double>& tensor_buf,
    std::vector<double>& tx_buf,
    std::vector<double>& rx_buf,
    std::vector<double>& rx_dint_buf,
    std::vector<double>& soft_buf,
    std::vector<double>& harq_accum,
    std::vector<double>& combined,
    std::vector<double>& last_hard)
{
    TestBlockResult result;

    // (1) 보호 비트 = 정보 + CRC16, (2) 반복 슬롯에 복사 → coded[MTU]
    std::vector<double> prot = CRC16::Append(info_bits);
    std::vector<double> coded(kMtu, 1.0);
    for (int r = 0; r < kRep; ++r)
        for (size_t i = 0u; i < prot.size(); ++i) {
            size_t pos = static_cast<size_t>(r) * prot.size() + i;
            if (pos < static_cast<size_t>(kMtu))
                coded[pos] = prot[i];
        }

    std::fill(harq_accum.begin(), harq_accum.end(), 0.0);

    for (int k = 1; k <= kMaxHarq; ++k) {
        result.harq_rounds = k;
        result.latency_ms = static_cast<double>(k) * kHarqRttMs;

        const unsigned int fseed =
            block_seed + static_cast<unsigned int>(k) * 7919u;

        fec.Encode_To(coded, fseed, tensor_buf);
        interleaver.Interleave_To(tensor_buf, tx_buf);
        apply_channel(tx_buf, rng, rx_buf, ch_type, ch_intensity);
        interleaver.Deinterleave_To(rx_buf, rx_dint_buf);
        fec.Decode_Soft_To(rx_dint_buf, kMtu, fseed, soft_buf);

        for (int i = 0; i < kMtu; ++i)
            harq_accum[static_cast<size_t>(i)] += soft_buf[static_cast<size_t>(i)];

        std::fill(combined.begin(), combined.end(), 0.0);
        for (int r = 0; r < kRep; ++r)
            for (int i = 0; i < kProt; ++i) {
                size_t pos = static_cast<size_t>(r) * kProt + i;
                if (pos < static_cast<size_t>(kMtu))
                    combined[static_cast<size_t>(i)] += harq_accum[pos];
            }
        for (int i = 0; i < kProt; ++i)
            last_hard[static_cast<size_t>(i)] =
                (combined[static_cast<size_t>(i)] > 0.0) ? 1.0 : -1.0;

        if (CRC16::Check(last_hard)) {
            result.info_bits.assign(
                last_hard.begin(), last_hard.begin() + (kProt - 16));
            result.crc_pass = true;
            return result;
        }
    }

    result.info_bits.assign(
        last_hard.begin(), last_hard.begin() + (kProt - 16));
    result.crc_pass = false;
    return result;
}

// -------------------------------------------------------------------------
//  BER (±1 비트를 0/1로 접어 비교)
// -------------------------------------------------------------------------
void compute_ber(
    const std::vector<double>& original,
    const std::vector<double>& received,
    int& errors, int& total, double& ber)
{
    errors = 0;
    total = static_cast<int>(std::min(original.size(), received.size()));
    for (int i = 0; i < total; ++i) {
        const int orig = (original[static_cast<size_t>(i)] > 0.0) ? 1 : 0;
        const int recv = (received[static_cast<size_t>(i)] > 0.0) ? 1 : 0;
        if (orig != recv) { ++errors; }
    }
    ber = (total > 0) ? static_cast<double>(errors) / total : 1.0;
}

// -------------------------------------------------------------------------
//  전체 메시지: INFO 단위 블록 분할 후 블록별 HARQ
// -------------------------------------------------------------------------
struct MessageResult {
    std::string rx_text;
    int    total_blocks = 0;
    int    success_blocks = 0;
    int    total_harq = 0;
    double max_latency_ms = 0.0;
    int    total_bit_errors = 0;
    int    total_bits = 0;
    double ber = 0.0;
    double avg_harq_per_block = 0.0;
};

MessageResult send_full_message(
    const std::string& message,
    Soft_Tensor_FEC& fec,
    Tensor_Interleaver& interleaver,
    std::mt19937& rng,
    ChannelType ch_type,
    double ch_intensity)
{
    MessageResult msg;

    std::vector<double> all_bits = Text_Codec::String_To_Bits(message);
    const int num_blocks =
        (static_cast<int>(all_bits.size()) + kInfoPerBlock - 1) / kInfoPerBlock;
    msg.total_blocks = num_blocks;

    const size_t tensor_size = interleaver.Get_Size();
    std::vector<double> tensor_buf(tensor_size, 0.0);
    std::vector<double> tx_buf(tensor_size, 0.0);
    std::vector<double> rx_buf(tensor_size, 0.0);
    std::vector<double> rx_dint_buf(tensor_size, 0.0);
    std::vector<double> soft_buf(kMtu, 0.0);
    std::vector<double> harq_accum(kMtu, 0.0);
    std::vector<double> combined(kProt, 0.0);
    std::vector<double> last_hard(kProt, 0.0);

    std::vector<double> all_rx_bits;
    all_rx_bits.reserve(all_bits.size());

    for (int b = 0; b < num_blocks; ++b) {
        const int start = b * kInfoPerBlock;
        const int end = std::min(start + kInfoPerBlock,
            static_cast<int>(all_bits.size()));
        std::vector<double> block_bits(
            all_bits.begin() + start,
            all_bits.begin() + end);

        while (static_cast<int>(block_bits.size()) < kInfoPerBlock)
            block_bits.push_back(1.0);

        const unsigned int bseed = static_cast<unsigned int>(b * 31337u + 42u);

        TestBlockResult br = transmit_block_parametric(
            block_bits, bseed, fec, interleaver, rng,
            ch_type, ch_intensity,
            tensor_buf, tx_buf, rx_buf, rx_dint_buf,
            soft_buf, harq_accum, combined, last_hard);

        if (br.crc_pass) { msg.success_blocks++; }
        msg.total_harq += br.harq_rounds;
        if (br.latency_ms > msg.max_latency_ms)
            msg.max_latency_ms = br.latency_ms;

        int err = 0, tot = 0;
        double blk_ber = 0.0;
        compute_ber(block_bits, br.info_bits, err, tot, blk_ber);
        msg.total_bit_errors += err;
        msg.total_bits += tot;

        const int copy_len = end - start;
        for (int i = 0; i < copy_len &&
            i < static_cast<int>(br.info_bits.size()); ++i)
            all_rx_bits.push_back(br.info_bits[static_cast<size_t>(i)]);
    }

    msg.rx_text = Text_Codec::Bits_To_String(all_rx_bits);
    msg.ber = (msg.total_bits > 0)
        ? static_cast<double>(msg.total_bit_errors) / msg.total_bits
        : 1.0;
    msg.avg_harq_per_block = (msg.total_blocks > 0)
        ? static_cast<double>(msg.total_harq) /
            static_cast<double>(msg.total_blocks)
        : 0.0;

    return msg;
}

// -------------------------------------------------------------------------
//  시나리오 요약 (최종 표용)
// -------------------------------------------------------------------------
struct ScenarioSummary {
    ChannelType type = ChannelType::AWGN;
    int pass_count = 0;
    int partial_count = 0;
    int fail_count = 0;
    double max_survivable_intensity = 0.0;
};

// -------------------------------------------------------------------------
//  콘솔 출력
// -------------------------------------------------------------------------
void print_line(char ch, int width) {
    std::cout << std::string(static_cast<size_t>(width), ch) << "\n";
}

void print_banner() {
    std::cout << "\n";
    print_line('=', 100);
    std::cout << "  HTS B-CDMA DIOC  항재밍 생존성 검증 (Tensor FEC + 인터리버 + HARQ)\n";
    std::cout << "  시뮬레이션 리포트\n";
    print_line('=', 100);
    std::cout << "\n";
}

void print_pipeline_params(int interleaver_dim) {
    std::cout << "  [파이프라인 파라미터 — LTE_HARQ_Controller / Soft_Tensor_FEC]\n";
    print_line('-', 100);
    std::cout << "    MTU(코드 비트)           : " << kMtu << "\n";
    std::cout << "    반복 인자 REP            : " << kRep << "\n";
    std::cout << "    보호 비트 수 PROT        : " << kProt << " (= MTU/REP)\n";
    std::cout << "    블록당 정보 비트 INFO    : " << kInfoPerBlock
        << " (= PROT - CRC16)\n";
    std::cout << "    HARQ 최대 라운드         : " << kMaxHarq << "\n";
    std::cout << "    HARQ RTT (가정, ms/라운드): " << kHarqRttMs << "\n";
    std::cout << "    인터리버 차원            : " << interleaver_dim
        << " (텐서 슬롯 " << interleaver_dim * interleaver_dim * interleaver_dim << ")\n";
    print_line('-', 100);
    std::cout << "\n";
}

void print_test_matrix_legend() {
    std::cout << "  [실행 매트릭스]\n";
    print_line('-', 100);
    std::cout << "    · 채널 4종 × 강도 " << kNumIntensitySteps << "단계 = 총 "
        << (kNumScenarios * kNumIntensitySteps) << "회\n";
    std::cout << "    · 각 회: 동일 페이로드 전체 전송 (블록 수는 페이로드 길이에 따름)\n";
    std::cout << "    · 강도 스윕 값: ";
    for (int i = 0; i < kNumIntensitySteps; ++i) {
        std::cout << kIntensitySteps[i];
        if (i + 1 < kNumIntensitySteps) { std::cout << ", "; }
    }
    std::cout << "\n";
    print_line('-', 100);
    std::cout << "\n";
}

void print_scenario_header(ChannelType type) {
    std::cout << "\n";
    print_line('-', 100);
    std::cout << "  시나리오: " << channel_name_kr(type) << "\n";
    std::cout << "  강도 의미: " << channel_intensity_hint_kr(type) << "\n";
    print_line('-', 100);
    std::cout
        << std::setw(6) << std::right << "강도"
        << std::setw(8) << "단위"
        << std::setw(7) << "블록"
        << std::setw(7) << "성공"
        << std::setw(6) << "CRC"
        << std::setw(12) << "BER"
        << std::setw(8) << "총HARQ"
        << std::setw(8) << "평균H"
        << std::setw(10) << "지연(ms)"
        << "  판정\n";
    print_line('-', 100);
}

void print_row(double intensity, ChannelType ct, const MessageResult& r) {
    const char* verdict =
        (r.success_blocks == r.total_blocks) ? "PASS" :
        (r.success_blocks > 0) ? "PARTIAL" : "FAIL";

    std::cout << std::fixed
        << std::setw(6) << std::setprecision(0) << intensity
        << std::setw(8) << channel_unit_short(ct)
        << std::setw(7) << r.total_blocks
        << std::setw(7) << r.success_blocks
        << std::setw(6)
        << ((r.success_blocks == r.total_blocks) ? "ACK" : "NACK")
        << std::setw(12) << std::scientific << std::setprecision(2)
        << r.ber
        << std::setw(8) << std::fixed << std::setprecision(0)
        << r.total_harq
        << std::setw(8) << std::setprecision(1) << r.avg_harq_per_block
        << std::setw(10) << std::setprecision(1) << r.max_latency_ms
        << "  " << verdict << "\n";
}

void print_rx_snippet(const MessageResult& mr, int step_idx, int num_steps) {
    if (step_idx != 0 && step_idx != num_steps - 1 &&
        mr.success_blocks == mr.total_blocks) {
        return;
    }
    std::cout << "      [수신 미리보기] ";
    const size_t n = std::min<size_t>(80, mr.rx_text.size());
    std::cout << mr.rx_text.substr(0, n);
    if (mr.rx_text.size() > 80) { std::cout << "..."; }
    std::cout << "\n";
}

} // namespace

// -------------------------------------------------------------------------
//  main
// -------------------------------------------------------------------------
int main() {
    const auto t0 = std::chrono::steady_clock::now();

    print_banner();

    constexpr int kInterDim = 64;
    Soft_Tensor_FEC fec;
    Tensor_Interleaver interleaver(static_cast<size_t>(kInterDim));
    std::mt19937 rng(kRngSeedBase);

    print_pipeline_params(kInterDim);
    print_test_matrix_legend();

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

    std::cout << "  [송신 페이로드]\n";
    print_line('-', 100);
    std::cout << "  " << payload << "\n";
    std::cout << "  바이트: " << payload.size()
        << "  |  비트(문자): " << (payload.size() * 8)
        << "  |  블록 수(대략): "
        << ((static_cast<int>(payload.size()) * 8 + kInfoPerBlock - 1) / kInfoPerBlock)
        << " (INFO=" << kInfoPerBlock << " 비트/블록)\n\n";

    ScenarioSummary summaries[kNumScenarios];
    summaries[0].type = ChannelType::AWGN;
    summaries[1].type = ChannelType::BARRAGE;
    summaries[2].type = ChannelType::CW;
    summaries[3].type = ChannelType::EMP;

    const ChannelType scenarios[kNumScenarios] = {
        ChannelType::AWGN,
        ChannelType::BARRAGE,
        ChannelType::CW,
        ChannelType::EMP
    };

    for (int s = 0; s < kNumScenarios; ++s) {
        const ChannelType ct = scenarios[s];
        print_scenario_header(ct);

        for (int step = 0; step < kNumIntensitySteps; ++step) {
            const double intensity = kIntensitySteps[step];
            rng.seed(kRngSeedBase + static_cast<unsigned>(s * 1000 + step));

            const MessageResult mr = send_full_message(
                payload, fec, interleaver, rng, ct, intensity);

            print_row(intensity, ct, mr);
            print_rx_snippet(mr, step, kNumIntensitySteps);

            if (mr.success_blocks == mr.total_blocks) {
                summaries[s].pass_count++;
                summaries[s].max_survivable_intensity = intensity;
            }
            else if (mr.success_blocks > 0) {
                summaries[s].partial_count++;
            }
            else {
                summaries[s].fail_count++;
            }
        }
    }

    const auto t1 = std::chrono::steady_clock::now();
    const double elapsed_sec =
        std::chrono::duration<double>(t1 - t0).count();

    std::cout << "\n";
    print_line('=', 100);
    std::cout << "  최종 요약\n";
    print_line('=', 100);
    std::cout << "\n";
    std::cout << std::setw(22) << std::left << "환경"
        << std::setw(8) << std::right << "PASS"
        << std::setw(10) << "PARTIAL"
        << std::setw(8) << "FAIL"
        << std::setw(18) << "최대 생존 강도"
        << "\n";
    print_line('-', 72);

    for (int s = 0; s < kNumScenarios; ++s) {
        std::cout << std::setw(22) << std::left << channel_name_kr(summaries[s].type)
            << std::setw(8) << summaries[s].pass_count
            << std::setw(10) << summaries[s].partial_count
            << std::setw(8) << summaries[s].fail_count
            << std::setw(12) << std::fixed << std::setprecision(0)
            << summaries[s].max_survivable_intensity
            << " " << channel_intensity_unit(summaries[s].type) << "\n";
    }

    std::cout << "\n";
    std::cout << "  총 실행: " << (kNumScenarios * kNumIntensitySteps) << "회\n";
    std::cout << "  경과 시간: " << std::fixed << std::setprecision(1)
        << elapsed_sec << " 초\n";
    print_line('=', 100);
    std::cout << "\n";

    return 0;
}
