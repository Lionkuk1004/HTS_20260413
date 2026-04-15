// =============================================================================
// HTS_Pluto_Test.cpp — V18: T2 DMA priming + Chase HARQ rv=0
// =============================================================================
#if defined(__arm__) || defined(__TARGET_ARCH_ARM)
#error "[HTS_FATAL] PC 전용"
#endif
#include "HTS_FEC_HARQ.hpp"
#include "HTS_V400_Dispatcher.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iio.h>
#include <random>
#include <thread>
#include <vector>
using ProtectedEngine::DecodedPacket;
using ProtectedEngine::FEC_HARQ;
using ProtectedEngine::HTS_V400_Dispatcher;
using ProtectedEngine::PayloadMode;
using ProtectedEngine::RxPhase;
using ProtectedEngine::SoftClipPolicy;
// ── PS-LTE 재난안전통신망 (Band 28, 700 MHz) ──
// 하향: 773~783 MHz, 상향: 718~728 MHz
// HTS B-CDMA 칩레이트 200 kc/s → 점유 BW ~200 kHz
static constexpr long long PLUTO_FREQ_HZ =
    778000000LL; // PS-LTE DL 중심 (TX/RX 동일 LO, 근거리 루프백)
static constexpr long long PLUTO_SAMPLE_RATE = 1000000LL; // 1 MSPS
static constexpr long long PLUTO_BW_HZ = 1000000LL;       // 1 MHz RF BW
static constexpr int PLUTO_BUF_SAMPLES = 8192; // 1 MSPS × 8 ms
static constexpr long long PLUTO_TX_GAIN_INIT = -50LL; // 근거리 테스트
static constexpr long long PLUTO_RX_GAIN_INIT = 40LL;   // 포화 방지
static long long g_tx_gain = PLUTO_TX_GAIN_INIT;
static constexpr int16_t kAmp = 1000;
static constexpr double kAmpD = 1000.0;
struct PlutoCtx {
    struct iio_context *ctx = nullptr;
    struct iio_device *phy = nullptr;
    struct iio_device *tx_dev = nullptr;
    struct iio_device *rx_dev = nullptr;
    struct iio_channel *tx_i = nullptr;
    struct iio_channel *tx_q = nullptr;
    struct iio_channel *rx_i = nullptr;
    struct iio_channel *rx_q = nullptr;
    struct iio_buffer *tx_buf = nullptr;
    struct iio_buffer *rx_buf = nullptr;
};
static bool wr_lli(struct iio_channel *c, const char *a, long long v) {
    return iio_channel_attr_write_longlong(c, a, v) >= 0;
}
static bool wr_str(struct iio_channel *c, const char *a, const char *v) {
    return iio_channel_attr_write(c, a, v) >= 0;
}
static void pluto_disable_dds(struct iio_device *d) {
    const char *alt[] = {"altvoltage0", "altvoltage1", "altvoltage2",
                         "altvoltage3"};
    for (auto nm : alt) {
        auto *ch = iio_device_find_channel(d, nm, true);
        if (ch)
            iio_channel_attr_write(ch, "raw", "0");
    }
}
static bool pluto_open(PlutoCtx &p) {
    p.ctx = iio_create_context_from_uri("usb:");
    if (!p.ctx) {
        std::printf("[PLUTO] USB 연결 실패. iio_info -s 로 URI 확인\n");
        return false;
    }
    p.phy = iio_context_find_device(p.ctx, "ad9361-phy");
    if (!p.phy) {
        std::printf("[PLUTO] PHY 미발견\n");
        return false;
    }
    auto *tx_lo = iio_device_find_channel(p.phy, "altvoltage1", true);
    auto *tx_ph = iio_device_find_channel(p.phy, "voltage0", true);
    if (tx_lo)
        wr_lli(tx_lo, "frequency", PLUTO_FREQ_HZ);
    if (tx_ph) {
        wr_lli(tx_ph, "rf_bandwidth", PLUTO_BW_HZ);
        wr_lli(tx_ph, "sampling_frequency", PLUTO_SAMPLE_RATE);
        wr_lli(tx_ph, "hardwaregain", PLUTO_TX_GAIN_INIT);
    }
    auto *rx_lo = iio_device_find_channel(p.phy, "altvoltage0", true);
    auto *rx_ph = iio_device_find_channel(p.phy, "voltage0", false);
    if (rx_lo)
        wr_lli(rx_lo, "frequency", PLUTO_FREQ_HZ);
    if (rx_ph) {
        wr_lli(rx_ph, "rf_bandwidth", PLUTO_BW_HZ);
        wr_lli(rx_ph, "sampling_frequency", PLUTO_SAMPLE_RATE);
        wr_str(rx_ph, "gain_control_mode", "manual");
        wr_lli(rx_ph, "hardwaregain", PLUTO_RX_GAIN_INIT);
    }
    p.tx_dev = iio_context_find_device(p.ctx, "cf-ad9361-dds-core-lpc");
    p.rx_dev = iio_context_find_device(p.ctx, "cf-ad9361-lpc");
    if (!p.tx_dev || !p.rx_dev)
        return false;
    p.tx_i = iio_device_find_channel(p.tx_dev, "voltage0", true);
    p.tx_q = iio_device_find_channel(p.tx_dev, "voltage1", true);
    p.rx_i = iio_device_find_channel(p.rx_dev, "voltage0", false);
    p.rx_q = iio_device_find_channel(p.rx_dev, "voltage1", false);
    if (!p.tx_i || !p.tx_q || !p.rx_i || !p.rx_q)
        return false;
    pluto_disable_dds(p.tx_dev);
    iio_channel_enable(p.tx_i);
    iio_channel_enable(p.tx_q);
    iio_channel_enable(p.rx_i);
    iio_channel_enable(p.rx_q);
    p.tx_buf = iio_device_create_buffer(p.tx_dev, PLUTO_BUF_SAMPLES, true);
    p.rx_buf = iio_device_create_buffer(p.rx_dev, PLUTO_BUF_SAMPLES, false);
    if (!p.tx_buf || !p.rx_buf)
        return false;
    std::printf(
        "[PLUTO] USB 연결 OK: LO=%.0f MHz (PS-LTE DL), BW=%.0f MHz, "
        "%.0f MSPS, TX=%lld dB, RX=%lld dB, Cyclic TX\n",
        PLUTO_FREQ_HZ / 1e6, PLUTO_BW_HZ / 1e6,
        PLUTO_SAMPLE_RATE / 1e6, PLUTO_TX_GAIN_INIT, PLUTO_RX_GAIN_INIT);
    return true;
}
static void pluto_close(PlutoCtx &p) {
    if (p.tx_buf)
        iio_buffer_destroy(p.tx_buf);
    if (p.rx_buf)
        iio_buffer_destroy(p.rx_buf);
    if (p.ctx)
        iio_context_destroy(p.ctx);
    p = PlutoCtx{};
}
static void pluto_set_tx_gain(PlutoCtx &p, long long g) {
    if (g < -89)
        g = -89;
    if (g > 0)
        g = 0; // AD9361 TX 범위: -89 ~ 0 dB
    auto *ch = iio_device_find_channel(p.phy, "voltage0", true);
    if (ch)
        wr_lli(ch, "hardwaregain", g);
    g_tx_gain = g;
}
static void pluto_flush_rx(PlutoCtx &p) {
    // 8192 samples @ 1 MSPS → ~8.192 ms/refill. 20회 클리어
    for (int i = 0; i < 20; ++i)
        iio_buffer_refill(p.rx_buf);
}
/// @brief AD9361 TX/RX DMA 체인 완전 리셋
/// @note  연속 실행 시 AD9361 DMA FSM이 stuck되는 현상 복구용.
///        TX·RX 버퍼 파괴 → 500ms 대기(PLL re-lock) → 버퍼 재생성.
///        RX 채널 재활성화로 cf-ad9361-lpc 스트림 엔진도 리셋.
/// @return 버퍼 재생성 성공 여부
static bool pluto_reset_tx_chain(PlutoCtx &p) {
    std::printf("  [RECOVERY] TX chain reset...\n");
    // ① TX 버퍼 파괴
    if (p.tx_buf) {
        iio_buffer_destroy(p.tx_buf);
        p.tx_buf = nullptr;
    }
    // ② RX 버퍼 파괴 (RX DMA도 함께 리셋)
    if (p.rx_buf) {
        iio_buffer_destroy(p.rx_buf);
        p.rx_buf = nullptr;
    }
    // ③ AD9361 PLL re-lock + DMA FSM 초기화 대기
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // ④ RX 버퍼 재생성
    p.rx_buf = iio_device_create_buffer(p.rx_dev, PLUTO_BUF_SAMPLES, false);
    if (!p.rx_buf) {
        std::printf("  [RECOVERY] RX buffer recreate FAIL\n");
        return false;
    }
    // ⑤ TX 버퍼 재생성 (cyclic)
    p.tx_buf = iio_device_create_buffer(p.tx_dev, PLUTO_BUF_SAMPLES, true);
    if (!p.tx_buf) {
        std::printf("  [RECOVERY] TX buffer recreate FAIL\n");
        return false;
    }
    // ⑥ 재생성 직후 RX 파이프라인 안정화
    pluto_flush_rx(p);
    std::printf("  [RECOVERY] TX chain reset OK\n");
    return true;
}
static bool pluto_tx_start(PlutoCtx &p, const int16_t *I, const int16_t *Q,
                           int n) {
    if (p.tx_buf) {
        iio_buffer_destroy(p.tx_buf);
        p.tx_buf = nullptr;
    }
    p.tx_buf = iio_device_create_buffer(p.tx_dev, PLUTO_BUF_SAMPLES, true);
    if (!p.tx_buf)
        return false;
    char *s = (char *)iio_buffer_start(p.tx_buf);
    char *e = (char *)iio_buffer_end(p.tx_buf);
    ptrdiff_t step = iio_buffer_step(p.tx_buf);
    int idx = 0;
    for (char *ptr = s; ptr < e; ptr += step) {
        ((int16_t *)ptr)[0] = (idx < n) ? I[idx] : 0;
        ((int16_t *)ptr)[1] = (idx < n) ? Q[idx] : 0;
        idx++;
    }
    return iio_buffer_push(p.tx_buf) > 0;
}
static int pluto_rx(PlutoCtx &p, int16_t *I, int16_t *Q, int max_n) {
    if (iio_buffer_refill(p.rx_buf) < 0)
        return 0;
    char *s = (char *)iio_buffer_start(p.rx_buf);
    char *e = (char *)iio_buffer_end(p.rx_buf);
    ptrdiff_t step = iio_buffer_step(p.rx_buf);
    int idx = 0;
    for (char *ptr = s; ptr < e && idx < max_n; ptr += step) {
        I[idx] = ((int16_t *)ptr)[0];
        Q[idx] = ((int16_t *)ptr)[1];
        idx++;
    }
    return idx;
}
static int pluto_tx_then_rx(PlutoCtx &p, const int16_t *txI, const int16_t *txQ,
                            int n_tx, int16_t *rxI, int16_t *rxQ, int max_rx) {
    // ① Pre-flush: 이전 반복에서 쌓인 잔류 RX 데이터 제거
    pluto_flush_rx(p);
    // ② Cyclic TX 시작 (버퍼 파괴→재생성→push)
    if (!pluto_tx_start(p, txI, txQ, n_tx))
        return 0;
    // ③ 최소 안정화 대기: PLL lock + DMA 파이프라인 (cyclic 반복 개시)
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    // ④ Post-flush: 전환 구간 노이즈 제거 (50ms 분량 = ~7 refill)
    pluto_flush_rx(p);
    // ⑤ 실제 수신: cyclic TX가 안정 반복 중인 상태에서 읽기
    return pluto_rx(p, rxI, rxQ, max_rx);
}
static void apply_digital_agc(int16_t *I, int16_t *Q, int n,
                              int target = 1000) {
    int32_t mx = 1;
    for (int i = 0; i < n; ++i) {
        int32_t ai = I[i] < 0 ? -I[i] : I[i];
        int32_t aq = Q[i] < 0 ? -Q[i] : Q[i];
        if (ai > mx)
            mx = ai;
        if (aq > mx)
            mx = aq;
    }
    if (mx < 30)
        return;
    float gain = (float)target / mx;
    for (int i = 0; i < n; ++i) {
        int32_t vi = (int32_t)(I[i] * gain);
        int32_t vq = (int32_t)(Q[i] * gain);
        if (vi > 32767)
            vi = 32767;
        if (vi < -32768)
            vi = -32768;
        if (vq > 32767)
            vq = 32767;
        if (vq < -32768)
            vq = -32768;
        I[i] = (int16_t)vi;
        Q[i] = (int16_t)vq;
    }
}
// 기존 성공했던 함수로 교체
static void apply_phase_correction(int16_t *I, int16_t *Q, int n) {
    int64_t sumI = 0, sumQ = 0;
    for (int i = 0; i < n; ++i) {
        int32_t mag2 = (int32_t)I[i] * I[i] + (int32_t)Q[i] * Q[i];
        if (mag2 > 3500) {
            sumI += I[i];
            sumQ += Q[i];
        }
    }
    double theta = std::atan2((double)sumQ, (double)sumI);
    double cosT = std::cos(theta);
    double sinT = std::sin(theta);
    for (int i = 0; i < n; ++i) {
        double di = (double)I[i];
        double dq = (double)Q[i];
        double ri = di * cosT + dq * sinT;
        double rq = -di * sinT + dq * cosT;
        if (ri > 32767)
            ri = 32767;
        if (ri < -32768)
            ri = -32768;
        if (rq > 32767)
            rq = 32767;
        if (rq < -32768)
            rq = -32768;
        I[i] = (int16_t)std::lround(ri);
        Q[i] = (int16_t)std::lround(rq);
    }
}
static int find_signal_start(const int16_t *rxI, const int16_t *rxQ, int n_rx,
                             const int16_t *sigI, const int16_t *sigQ,
                             int sig_len) {
    if (n_rx <= sig_len)
        return 0;
    int best = 0;
    int64_t best_c = -1;
    int cl = sig_len > 1024 ? 1024 : sig_len;
    for (int i = 0; i <= n_rx - sig_len; ++i) {
        int64_t cI = 0, cQ = 0;
        for (int j = 0; j < cl; ++j) {
            cI += (int64_t)rxI[i + j] * sigI[j] + (int64_t)rxQ[i + j] * sigQ[j];
            cQ += (int64_t)rxQ[i + j] * sigI[j] - (int64_t)rxI[i + j] * sigQ[j];
        }
        int64_t cs = (cI / 256), qs = (cQ / 256);
        int64_t c = cs * cs + qs * qs;
        if (c > best_c) {
            best_c = c;
            best = i;
        }
    }
    return best;
}
static constexpr int GUARD_CHIPS = 512;
static void fill_tx_repeated(int16_t *txI, int16_t *txQ, int buf_len,
                             const int16_t *sigI, const int16_t *sigQ,
                             int sig_len) {
    std::memset(txI, 0, buf_len * sizeof(int16_t));
    std::memset(txQ, 0, buf_len * sizeof(int16_t));
    int block = sig_len + GUARD_CHIPS;
    int offset = GUARD_CHIPS;
    while (offset + sig_len <= buf_len) {
        std::memcpy(&txI[offset], sigI, sig_len * sizeof(int16_t));
        std::memcpy(&txQ[offset], sigQ, sig_len * sizeof(int16_t));
        offset += block;
    }
}
static uint32_t popc32(uint32_t x) {
    x -= ((x >> 1) & 0x55555555u);
    x = (x & 0x33333333u) + ((x >> 2) & 0x33333333u);
    return (((x + (x >> 4)) & 0x0F0F0F0Fu) * 0x01010101u) >> 24;
}
static void walsh_enc(uint8_t sym, int nc, int16_t amp, int16_t *oI,
                      int16_t *oQ) {
    for (int j = 0; j < nc; ++j) {
        int p = popc32((uint32_t)sym & (uint32_t)j) & 1;
        int16_t ch = (int16_t)(amp * (1 - 2 * p));
        oI[j] = ch;
        oQ[j] = ch;
    }
}
static void gen_info(uint32_t seed, uint8_t *info) {
    uint32_t s = seed;
    for (int b = 0; b < 8; ++b) {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        info[b] = (uint8_t)(s & 0xFF);
    }
}
static void add_barrage(int16_t *I, int16_t *Q, int n, double js_db,
                        std::mt19937 &rng) {
    if (js_db < 0)
        return;
    double sigma = kAmpD * std::sqrt(std::pow(10.0, js_db / 10.0));
    std::normal_distribution<double> nd(0.0, sigma);
    for (int i = 0; i < n; ++i) {
        double vi = (double)I[i] + nd(rng);
        double vq = (double)Q[i] + nd(rng);
        if (vi > 2047)
            vi = 2047;
        if (vi < -2048)
            vi = -2048;
        if (vq > 2047)
            vq = 2047;
        if (vq < -2048)
            vq = -2048;
        I[i] = (int16_t)std::lround(vi);
        Q[i] = (int16_t)std::lround(vq);
    }
}
static void add_cw(int16_t *I, int16_t *Q, int n, double js_db) {
    if (js_db < 0)
        return;
    double a = kAmpD * std::sqrt(std::pow(10.0, js_db / 10.0));
    for (int i = 0; i < n; ++i) {
        double ph = 2.0 * 3.14159265358979 * i / 8.0;
        double vi = (double)I[i] + a * std::cos(ph);
        double vq = (double)Q[i] + a * std::sin(ph);
        if (vi > 2047)
            vi = 2047;
        if (vi < -2048)
            vi = -2048;
        if (vq > 2047)
            vq = 2047;
        if (vq < -2048)
            vq = -2048;
        I[i] = (int16_t)std::lround(vi);
        Q[i] = (int16_t)std::lround(vq);
    }
}
static DecodedPacket g_last{};
static void on_pkt(const DecodedPacket &pkt) { g_last = pkt; }
// ════════════════════════════════════════════════════════════
//  공통: FEC 직접 TX→RX→정렬→디코딩
// ════════════════════════════════════════════════════════════
static bool fec_txrx_decode(PlutoCtx &p, const uint8_t *info, uint32_t il,
                            int bps, int nc, int rv, FEC_HARQ::IR_RxState &ir,
                            FEC_HARQ::WorkBuf &wb, uint8_t *out, int *olen,
                            const int16_t *jamI = nullptr,
                            const int16_t *jamQ = nullptr, int jam_len = 0) {
    const int nsym = FEC_HARQ::nsym_for_bps(bps);
    const int total = nsym * nc;
    uint8_t syms[FEC_HARQ::NSYM64]{};
    std::memset(&wb, 0, sizeof(wb));
    if (FEC_HARQ::Encode64_IR(info, 8, syms, il, bps, rv & 3, wb) <= 0)
        return false;
    std::vector<int16_t> sigI(total), sigQ(total);
    for (int s = 0; s < nsym; ++s)
        walsh_enc(syms[s], nc, kAmp, &sigI[s * nc], &sigQ[s * nc]);
    std::vector<int16_t> refI = sigI, refQ = sigQ;
    if (jamI && jamQ && jam_len == total) {
        for (int i = 0; i < total; ++i) {
            sigI[i] = jamI[i];
            sigQ[i] = jamQ[i];
        }
    }
    std::vector<int16_t> txI(PLUTO_BUF_SAMPLES), txQ(PLUTO_BUF_SAMPLES);
    fill_tx_repeated(txI.data(), txQ.data(), PLUTO_BUF_SAMPLES, sigI.data(),
                     sigQ.data(), total);
    std::vector<int16_t> rxI(PLUTO_BUF_SAMPLES), rxQ(PLUTO_BUF_SAMPLES);
    int n_rx = pluto_tx_then_rx(p, txI.data(), txQ.data(), PLUTO_BUF_SAMPLES,
                                rxI.data(), rxQ.data(), PLUTO_BUF_SAMPLES);
    if (n_rx < total)
        return false;
    apply_digital_agc(rxI.data(), rxQ.data(), n_rx);
    apply_phase_correction(rxI.data(), rxQ.data(), n_rx);
    int ss = find_signal_start(rxI.data(), rxQ.data(), n_rx, refI.data(),
                               refQ.data(), total);
    if (ss + total > n_rx)
        ss = 0;
    std::memset(&wb, 0, sizeof(wb));
    bool dec = FEC_HARQ::Decode64_IR(&rxI[ss], &rxQ[ss], nsym, nc, bps, il,
                                     rv & 3, ir, out, olen, wb);
    return dec && *olen == 8 && std::memcmp(out, info, 8) == 0;
}
static bool fec16_txrx_decode(PlutoCtx &p, const uint8_t *info, uint32_t il,
                              int rv, FEC_HARQ::IR_RxState &ir,
                              FEC_HARQ::WorkBuf &wb, uint8_t *out, int *olen) {
    const int nc = 16, nsym = FEC_HARQ::NSYM16;
    const int total = nsym * nc;
    uint8_t syms[FEC_HARQ::NSYM16]{};
    std::memset(&wb, 0, sizeof(wb));
    if (FEC_HARQ::Encode16_IR(info, 8, syms, il, rv & 3, wb) <= 0)
        return false;
    std::vector<int16_t> sigI(total), sigQ(total);
    for (int s = 0; s < nsym; ++s)
        walsh_enc(syms[s], nc, kAmp, &sigI[s * nc], &sigQ[s * nc]);
    std::vector<int16_t> refI = sigI, refQ = sigQ;
    std::vector<int16_t> txI(PLUTO_BUF_SAMPLES), txQ(PLUTO_BUF_SAMPLES);
    fill_tx_repeated(txI.data(), txQ.data(), PLUTO_BUF_SAMPLES, sigI.data(),
                     sigQ.data(), total);
    std::vector<int16_t> rxI(PLUTO_BUF_SAMPLES), rxQ(PLUTO_BUF_SAMPLES);
    int n_rx = pluto_tx_then_rx(p, txI.data(), txQ.data(), PLUTO_BUF_SAMPLES,
                                rxI.data(), rxQ.data(), PLUTO_BUF_SAMPLES);
    if (n_rx < total)
        return false;
    apply_digital_agc(rxI.data(), rxQ.data(), n_rx);
    apply_phase_correction(rxI.data(), rxQ.data(), n_rx);
    int ss = find_signal_start(rxI.data(), rxQ.data(), n_rx, refI.data(),
                               refQ.data(), total);
    if (ss + total > n_rx)
        ss = 0;
    std::memset(&wb, 0, sizeof(wb));
    bool dec =
        FEC_HARQ::Decode16_IR(&rxI[ss], &rxQ[ss], nsym, nc, FEC_HARQ::BPS16, il,
                              rv & 3, ir, out, olen, wb);
    return dec && *olen == 8 && std::memcmp(out, info, 8) == 0;
}
// ════════════════════════════════════════════════════════════
//  T0~T9 + 진단 printf 추가
// ════════════════════════════════════════════════════════════
static long long test_T0(PlutoCtx &p) {
    std::printf("\n══ T0: 수신 레벨 탐색 ══\n");
    std::vector<int16_t> txI(PLUTO_BUF_SAMPLES), txQ(PLUTO_BUF_SAMPLES);
    for (int i = 0; i < PLUTO_BUF_SAMPLES; ++i) {
        int16_t v = ((i / 32) & 1) ? 1000 : -1000;
        txI[i] = v;
        txQ[i] = v;
    }
    // 최대 2회 시도 (0: 정상, 1: TX chain reset 후 재시도)
    for (int attempt = 0; attempt < 2; ++attempt) {
        if (attempt == 1) {
            std::printf("  [T0] 전 구간 미감지 — TX chain reset 후 재스캔\n");
            if (!pluto_reset_tx_chain(p)) {
                std::printf(
                    "  [T0] TX chain reset 실패 — 디폴트 -40 dB 사용\n");
                pluto_set_tx_gain(p, -35);
                return -35;
            }
        }
        long long best = -80;
        bool any_detected = false;
        for (long long gain = -80; gain <= -40; gain += 5) {
            pluto_set_tx_gain(p, gain);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            std::vector<int16_t> rxI(PLUTO_BUF_SAMPLES), rxQ(PLUTO_BUF_SAMPLES);
            int n =
                pluto_tx_then_rx(p, txI.data(), txQ.data(), PLUTO_BUF_SAMPLES,
                                 rxI.data(), rxQ.data(), PLUTO_BUF_SAMPLES);
            int32_t mx = 0;
            for (int i = 0; i < n; ++i) {
                int32_t a = rxI[i] < 0 ? -rxI[i] : rxI[i];
                if (a > mx)
                    mx = a;
            }
            std::printf("  TX=%3lld dB → RX max=%5d", gain, (int)mx);
            if (mx > 1800) {
                std::printf(" ← 포화!\n");
                any_detected = true;
                break;
            }
            if (mx >= 300 && mx < 1500) {
                best = gain;
                std::printf(" ← 적정 ✓\n");
                any_detected = true;
                break;
            }
            if (mx >= 50) {
                best = gain;
                std::printf(" ← 감지\n");
                any_detected = true;
            } else
                std::printf(" ← 미감지\n");
        }
        if (any_detected) {
            long long boosted = best + 5;
            if (boosted > 0) {
                boosted = 0;
            }
            std::printf("  TX gain: %lld dB → 고정 %lld dB (+5 dB, cap 0)\n",
                        best, boosted);
            pluto_set_tx_gain(p, boosted);
            return boosted;
        }
        // 전 구간 미감지 — attempt 0이면 루프 계속(재시도), 1이면 폴백
    }
    // 2회 시도 후에도 전 구간 미감지 — 디폴트 폴백
    std::printf("  [T0] 2회 시도 모두 미감지 — 디폴트 -35 dB 폴백\n");
    pluto_set_tx_gain(p, -35);
    return -35;
}
static bool test_T1(PlutoCtx &p) {
    std::printf("\n══ T1: 연결 확인 ══\n");
    pluto_flush_rx(p);
    std::vector<int16_t> rxI(PLUTO_BUF_SAMPLES), rxQ(PLUTO_BUF_SAMPLES);
    int n = pluto_rx(p, rxI.data(), rxQ.data(), PLUTO_BUF_SAMPLES);
    std::printf("  RX %d samples\n  T1: %s\n", n, n > 0 ? "PASS" : "FAIL");
    return n > 0;
}
static void test_T2(PlutoCtx &p) {
    std::printf("\n══ T2: 클린 루프백 ══\n");
    FEC_HARQ::Set_IR_Erasure_Enabled(false);
    FEC_HARQ::Set_IR_Rs_Post_Enabled(true);
    // ── T2 DMA 프라이밍: 실 TX→RX 사이클로 파이프라인 활성화 ──
    // V17의 flush+sleep은 RX만 정리. TX DMA FSM 복구에는
    // buffer destroy→recreate→push 사이클이 필요하므로
    // pluto_tx_then_rx 를 직접 호출하여 TX 경로를 프라이밍.
    // (fec_txrx_decode 미호출 → s_chip_acc/IR_state 오염 없음)
    {
        std::vector<int16_t> w_txI(PLUTO_BUF_SAMPLES),
            w_txQ(PLUTO_BUF_SAMPLES);
        for (int i = 0; i < PLUTO_BUF_SAMPLES; ++i) {
            int16_t v = ((i / 32) & 1) ? 1000 : -1000;
            w_txI[i] = v;
            w_txQ[i] = v;
        }
        std::vector<int16_t> w_rxI(PLUTO_BUF_SAMPLES),
            w_rxQ(PLUTO_BUF_SAMPLES);
        int32_t prime_mx = 0;
        int prime_cycles = 0;
        // Phase 1: 최대 5회 TX→RX, RX max >= 200 이면 조기 종료
        for (int w = 0; w < 5; ++w) {
            int n = pluto_tx_then_rx(p, w_txI.data(), w_txQ.data(),
                                     PLUTO_BUF_SAMPLES, w_rxI.data(),
                                     w_rxQ.data(), PLUTO_BUF_SAMPLES);
            prime_mx = 0;
            for (int i = 0; i < n; ++i) {
                int32_t a = w_rxI[i] < 0 ? -w_rxI[i] : w_rxI[i];
                if (a > prime_mx)
                    prime_mx = a;
            }
            prime_cycles = w + 1;
            if (prime_mx >= 200)
                break;
        }
        // Phase 2: 5회 후에도 미복구 → chain reset + 추가 10회
        if (prime_mx < 200) {
            std::printf("  [T2-PRIME] RX max=%d after %d cycles — chain reset\n",
                        static_cast<int>(prime_mx), prime_cycles);
            pluto_reset_tx_chain(p);
            pluto_set_tx_gain(p, g_tx_gain);
            for (int w = 0; w < 10; ++w) {
                int n = pluto_tx_then_rx(p, w_txI.data(), w_txQ.data(),
                                         PLUTO_BUF_SAMPLES, w_rxI.data(),
                                         w_rxQ.data(), PLUTO_BUF_SAMPLES);
                prime_mx = 0;
                for (int i = 0; i < n; ++i) {
                    int32_t a = w_rxI[i] < 0 ? -w_rxI[i] : w_rxI[i];
                    if (a > prime_mx)
                        prime_mx = a;
                }
                prime_cycles += 1;
                if (prime_mx >= 200)
                    break;
            }
        }
        std::printf("  [T2-PRIME] %d cycles, RX max=%d%s\n",
                    prime_cycles, static_cast<int>(prime_mx),
                    prime_mx >= 200 ? " — ready" : " — RF unstable");
    }
    int ok = 0;
    for (int t = 0; t < 10; ++t) {
        if (t > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        uint32_t ts = 0xB40730u ^ (uint32_t)(t * 0x9E3779B9u);
        uint8_t info[8];
        gen_info(ts, info);
        FEC_HARQ::IR_RxState ir{};
        FEC_HARQ::IR_Init(ir);
        FEC_HARQ::WorkBuf wb{};
        uint8_t out[8]{};
        int olen = 0;
        bool dec = false;
        int used_rounds = 0;
        // ── Chase HARQ: rv=0 고정 (최대 4R) ──
        // Polar 칩누적(s_chip_acc)은 coherent 결합이므로
        // 매 라운드 동일 Walsh 심볼을 TX해야 함.
        // rv를 변경하면 Encode64_IR의 인터리브가 바뀌어
        // 다른 Walsh 심볼이 생성 → 칩 합산 시 직교 상쇄.
        for (int round = 0; round < 4 && !dec; ++round) {
            dec = fec_txrx_decode(p, info, 0xA5A5A5A5u ^ ts, 4, 64, 0, ir, wb,
                                  out, &olen);
            used_rounds = round + 1;
        }
        std::printf("  [%2d] %s (%dR)\n", t, dec ? "OK" : "FAIL", used_rounds);
        if (dec)
            ok++;
    }
    std::printf("  T2: %d/10\n", ok);
}
static void test_T3(PlutoCtx &p, long long base) {
    std::printf("\n══ T3: TX gain 스윕 ══\n");
    for (long long off = 0; off >= -30; off -= 5) {
        long long g = base + off;
        if (g < -89)
            g = -89;
        pluto_set_tx_gain(p, g);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        int ok = 0;
        for (int t = 0; t < 10; ++t) {
            uint32_t ts = 0xCAFE00u ^ (uint32_t)(t * 0x9E3779B9u);
            uint8_t info[8];
            gen_info(ts, info);
            FEC_HARQ::IR_RxState ir{};
            FEC_HARQ::IR_Init(ir);
            FEC_HARQ::WorkBuf wb{};
            uint8_t out[8]{};
            int olen = 0;
            if (fec_txrx_decode(p, info, 0xA5A5A5A5u ^ ts, 4, 64, 0, ir, wb,
                                out, &olen))
                ok++;
        }
        std::printf("  TX=%3lld dB (%+lld) %d/10\n", g, off, ok);
    }
    pluto_set_tx_gain(p, base);
}
static void test_T4(PlutoCtx &p) {
    std::printf("\n══ T4: 바라지 재밍 ══\n");
    const double js[] = {0, 5, 10, 15, 20, 25, 30};
    const int bps = 3, nc = 64;
    const int nsym = FEC_HARQ::nsym_for_bps(bps);
    const int total = nsym * nc;
    for (double j : js) {
        // DAC 클리핑 방지: 3σ < 2048 조건
        // safe = 2047 / (1 + 3×√(10^(j/10)))
        const double lin = std::pow(10.0, j / 10.0);
        const double denom = 1.0 + 3.0 * std::sqrt(lin);
        int16_t amp_t4 = static_cast<int16_t>(
            std::min(static_cast<double>(kAmp), 2047.0 / denom));
        if (amp_t4 < 30)
            amp_t4 = 30; // Pluto 채널 SNR 하한
        int ok = 0;
        double ar = 0;
        for (int t = 0; t < 10; ++t) {
            uint32_t ts = 0xDEAD00u ^ (uint32_t)(t * 0x9E3779B9u);
            uint32_t il = 0xA5A5A5A5u ^ ts;
            uint8_t info[8];
            gen_info(ts, info);
            HTS_V400_Dispatcher disp;
            disp.Set_Seed(0xA5A5A5A5u ^ ts);
            disp.Set_IR_Mode(true);
            disp.Set_CW_Cancel(true);
            disp.Set_AJC_Enabled(true);
            disp.Set_SoftClip_Policy(SoftClipPolicy::ALWAYS);
            disp.Set_Packet_Callback(on_pkt);
            disp.Set_Lab_BPS64(bps);
            disp.Set_Lab_IQ_Mode_Jam_Harness();
            bool dec = false;
            int rnd = 0;
            for (int rv = 0; rv < 32 && !dec; ++rv) {
                g_last = DecodedPacket{};
                // 페이로드만 인코딩
                FEC_HARQ::WorkBuf wb{};
                uint8_t syms[FEC_HARQ::NSYM64]{};
                std::memset(&wb, 0, sizeof(wb));
                FEC_HARQ::Encode64_IR(info, 8, syms, il, bps, rv & 3, wb);
                std::vector<int16_t> sI(total), sQ(total);
                for (int s = 0; s < nsym; ++s)
                    walsh_enc(syms[s], nc, amp_t4, &sI[s * nc], &sQ[s * nc]);
                // 재밍 추가 (페이로드에만) — amp_t4 기준 바라지
                std::mt19937 rng(ts ^ (uint32_t)(rv * 0x85EBCA6Bu));
                {
                    double sigma_t4 =
                        static_cast<double>(amp_t4) * std::sqrt(lin);
                    std::normal_distribution<double> nd(0.0, sigma_t4);
                    for (int i = 0; i < total; ++i) {
                        double vi = (double)sI[i] + nd(rng);
                        double vq = (double)sQ[i] + nd(rng);
                        if (vi > 2047)
                            vi = 2047;
                        if (vi < -2048)
                            vi = -2048;
                        if (vq > 2047)
                            vq = 2047;
                        if (vq < -2048)
                            vq = -2048;
                        sI[i] = (int16_t)std::lround(vi);
                        sQ[i] = (int16_t)std::lround(vq);
                    }
                }
                // TX → RX
                std::vector<int16_t> txI(PLUTO_BUF_SAMPLES),
                    txQ(PLUTO_BUF_SAMPLES);
                fill_tx_repeated(txI.data(), txQ.data(), PLUTO_BUF_SAMPLES,
                                 sI.data(), sQ.data(), total);
                std::vector<int16_t> rxI(PLUTO_BUF_SAMPLES),
                    rxQ(PLUTO_BUF_SAMPLES);
                int nr = pluto_tx_then_rx(p, txI.data(), txQ.data(),
                                          PLUTO_BUF_SAMPLES, rxI.data(),
                                          rxQ.data(), PLUTO_BUF_SAMPLES);
                if (nr <= 0)
                    break;
                apply_digital_agc(rxI.data(), rxQ.data(), nr);
                apply_phase_correction(rxI.data(), rxQ.data(), nr);
                // 클린 레퍼런스로 정렬 (재밍 전 원본, amp_t4와 동일 진폭)
                std::vector<int16_t> refI(total), refQ(total);
                for (int s = 0; s < nsym; ++s)
                    walsh_enc(syms[s], nc, amp_t4, &refI[s * nc],
                              &refQ[s * nc]);
                int ss = find_signal_start(rxI.data(), rxQ.data(), nr,
                                           refI.data(), refQ.data(), total);
                if (ss + total > nr)
                    ss = 0;
                // 디스패처: 동기 건너뛰고 페이로드 직접 진입
                if (rv == 0) {
                    disp.Inject_Payload_Phase(PayloadMode::DATA, bps);
                }
                // Feed — ECCM 파이프라인 경유
                for (int i = 0; i < total; ++i) {
                    if (rv == 0) {
                        disp.Feed_Chip(rxI[ss + i], rxQ[ss + i]);
                    } else {
                        disp.Feed_Retx_Chip(rxI[ss + i], rxQ[ss + i]);
                    }
                }
                dec = (g_last.success_mask == DecodedPacket::DECODE_MASK_OK);
                rnd = rv + 1;
                if (dec && (g_last.data_len != 8 ||
                            std::memcmp(g_last.data, info, 8) != 0))
                    dec = false;
            }
            if (dec) {
                ok++;
                ar += rnd;
            }
        }
        if (ok > 0)
            ar /= ok;
        std::printf("  J/S=%2.0f dB %d/10 avg %.1fR\n", j, ok, ar);
        if (ok == 0)
            break;
    }
}
static void test_T5(PlutoCtx &p) {
    std::printf("\n══ T5: CW 재밍 ══\n");
    const double js[] = {0, 5, 10, 15, 20};
    const int bps = 4, nc = 64;
    const int nsym = FEC_HARQ::nsym_for_bps(bps);
    const int total = nsym * nc;
    for (double j : js) {
        int ok = 0;
        double ar = 0;
        for (int t = 0; t < 10; ++t) {
            uint32_t ts = 0xCCCC00u ^ (uint32_t)(t * 0x9E3779B9u);
            uint32_t il = 0xA5A5A5A5u ^ ts;
            uint8_t info[8];
            gen_info(ts, info);
            HTS_V400_Dispatcher disp;
            disp.Set_Seed(0xA5A5A5A5u ^ ts);
            disp.Set_IR_Mode(true);
            disp.Set_CW_Cancel(true);
            disp.Set_AJC_Enabled(true);
            disp.Set_SoftClip_Policy(SoftClipPolicy::ALWAYS);
            disp.Set_Packet_Callback(on_pkt);
            disp.Set_Lab_BPS64(bps);
            disp.Set_Lab_IQ_Mode_Jam_Harness();
            bool dec = false;
            int rnd = 0;
            for (int rv = 0; rv < 32 && !dec; ++rv) {
                g_last = DecodedPacket{};
                FEC_HARQ::WorkBuf wb{};
                uint8_t syms[FEC_HARQ::NSYM64]{};
                std::memset(&wb, 0, sizeof(wb));
                FEC_HARQ::Encode64_IR(info, 8, syms, il, bps, rv & 3, wb);
                std::vector<int16_t> sI(total), sQ(total);
                for (int s = 0; s < nsym; ++s)
                    walsh_enc(syms[s], nc, kAmp, &sI[s * nc], &sQ[s * nc]);
                std::vector<int16_t> refI = sI, refQ = sQ;
                // 매 라운드 동일 CW (위상 연속)
                add_cw(sI.data(), sQ.data(), total, j);
                std::vector<int16_t> txI(PLUTO_BUF_SAMPLES),
                    txQ(PLUTO_BUF_SAMPLES);
                fill_tx_repeated(txI.data(), txQ.data(), PLUTO_BUF_SAMPLES,
                                 sI.data(), sQ.data(), total);
                std::vector<int16_t> rxI(PLUTO_BUF_SAMPLES),
                    rxQ(PLUTO_BUF_SAMPLES);
                int nr = pluto_tx_then_rx(p, txI.data(), txQ.data(),
                                          PLUTO_BUF_SAMPLES, rxI.data(),
                                          rxQ.data(), PLUTO_BUF_SAMPLES);
                if (nr <= 0)
                    break;
                apply_digital_agc(rxI.data(), rxQ.data(), nr);
                apply_phase_correction(rxI.data(), rxQ.data(), nr);
                int ss = find_signal_start(rxI.data(), rxQ.data(), nr,
                                           refI.data(), refQ.data(), total);
                if (ss + total > nr)
                    ss = 0;
                if (rv == 0) {
                    disp.Inject_Payload_Phase(PayloadMode::DATA, bps);
                }
                for (int i = 0; i < total; ++i) {
                    if (rv == 0)
                        disp.Feed_Chip(rxI[ss + i], rxQ[ss + i]);
                    else
                        disp.Feed_Retx_Chip(rxI[ss + i], rxQ[ss + i]);
                }
                dec = (g_last.success_mask == DecodedPacket::DECODE_MASK_OK);
                rnd = rv + 1;
                if (dec && (g_last.data_len != 8 ||
                            std::memcmp(g_last.data, info, 8) != 0))
                    dec = false;
            }
            if (dec) {
                ok++;
                ar += rnd;
            }
        }
        if (ok > 0)
            ar /= ok;
        std::printf("  CW J/S=%2.0f dB %d/10 avg %.1fR\n", j, ok, ar);
        if (ok == 0)
            break;
    }
}
// ════════════════════════════════════════════════════════════
//  T6-SIM: 소프트웨어 채널 디스패처 동기화 시뮬레이션
//  Pluto 하드웨어 미사용. Build_Packet → Feed_Chip 직접 연결.
// ════════════════════════════════════════════════════════════
static void test_T6_sim() {
    std::printf("\n══ T6-SIM: 소프트웨어 디스패처 동기화 ══\n");

    static constexpr int kPreReps = 4;
    static constexpr int kPreBoost = 1;
    static constexpr int kMaxC =
        2048 + (FEC_HARQ::NSYM64 + kPreReps + 4) * 64;

    int total_ok = 0;

    // ── 테스트 1: 클린 채널 (노이즈 0, 위상 0) ──
    {
        std::printf("  -- Test 1: Clean channel --\n");
        int ok = 0;
        for (int t = 0; t < 10; ++t) {
            g_last = DecodedPacket{};
            uint32_t ds = 0xF00D00u ^ (uint32_t)(t * 0x9E3779B9u);

            // TX
            HTS_V400_Dispatcher tx_disp;
            tx_disp.Set_IR_Mode(true);
            tx_disp.Set_Seed(ds);
            tx_disp.Set_Preamble_Boost(kPreBoost);
            tx_disp.Set_Preamble_Reps(kPreReps);
            tx_disp.Set_CW_Cancel(false);
            tx_disp.Set_AJC_Enabled(false);
            tx_disp.Set_SoftClip_Policy(SoftClipPolicy::NEVER);
            tx_disp.Set_Packet_Callback(on_pkt);
            tx_disp.Update_Adaptive_BPS(1000);
            tx_disp.Set_Lab_IQ_Mode_Jam_Harness();

            uint8_t info[8]{};
            for (int b = 0; b < 8; ++b)
                info[b] = (uint8_t)((ds >> (b * 4)) ^ (unsigned)(t + b));

            std::vector<int16_t> sigI(kMaxC), sigQ(kMaxC);
            int n = tx_disp.Build_Packet(PayloadMode::DATA, info, 8, kAmp,
                                         sigI.data(), sigQ.data(), kMaxC);
            if (n <= 0) {
                std::printf("    [%d] Build FAIL\n", t);
                continue;
            }

            // RX — 가드(256칩 0) + 패킷 + 가드(256칩 0)
            HTS_V400_Dispatcher rx_disp;
            rx_disp.Set_IR_Mode(true);
            rx_disp.Set_Seed(ds);
            rx_disp.Set_Preamble_Boost(kPreBoost);
            rx_disp.Set_Preamble_Reps(kPreReps);
            rx_disp.Set_CW_Cancel(false);
            rx_disp.Set_AJC_Enabled(false);
            rx_disp.Set_SoftClip_Policy(SoftClipPolicy::NEVER);
            rx_disp.Set_Packet_Callback(on_pkt);
            rx_disp.Update_Adaptive_BPS(1000);

            // 가드 256칩 (0)
            for (int i = 0; i < 256; ++i)
                rx_disp.Feed_Chip(0, 0);
            // 패킷 전체
            for (int i = 0; i < n; ++i)
                rx_disp.Feed_Chip(sigI[i], sigQ[i]);
            // 후미 가드 256칩
            for (int i = 0; i < 256; ++i)
                rx_disp.Feed_Chip(0, 0);

            bool success =
                (g_last.success_mask == DecodedPacket::DECODE_MASK_OK);
            int phase_val = static_cast<int>(rx_disp.Get_Phase());
            std::printf("    [%d] n=%d phase=%d %s\n", t, n, phase_val,
                        success ? "OK" : "FAIL");
            if (success)
                ok++;
        }
        std::printf("  Test 1 (clean): %d/10\n", ok);
        total_ok += ok;
    }

    // ── 테스트 2: 위상 90° 회전 (I→Q, Q→-I) ──
    {
        std::printf("  -- Test 2: Phase 90° --\n");
        int ok = 0;
        for (int t = 0; t < 10; ++t) {
            g_last = DecodedPacket{};
            uint32_t ds = 0xF00D00u ^ (uint32_t)(t * 0x9E3779B9u);

            HTS_V400_Dispatcher tx_disp;
            tx_disp.Set_IR_Mode(true);
            tx_disp.Set_Seed(ds);
            tx_disp.Set_Preamble_Boost(kPreBoost);
            tx_disp.Set_Preamble_Reps(kPreReps);
            tx_disp.Set_CW_Cancel(false);
            tx_disp.Set_AJC_Enabled(false);
            tx_disp.Set_SoftClip_Policy(SoftClipPolicy::NEVER);
            tx_disp.Set_Packet_Callback(on_pkt);
            tx_disp.Update_Adaptive_BPS(1000);

            uint8_t info[8]{};
            for (int b = 0; b < 8; ++b)
                info[b] = (uint8_t)((ds >> (b * 4)) ^ (unsigned)(t + b));

            std::vector<int16_t> sigI(kMaxC), sigQ(kMaxC);
            int n = tx_disp.Build_Packet(PayloadMode::DATA, info, 8, kAmp,
                                         sigI.data(), sigQ.data(), kMaxC);
            if (n <= 0)
                continue;

            HTS_V400_Dispatcher rx_disp;
            rx_disp.Set_IR_Mode(true);
            rx_disp.Set_Seed(ds);
            rx_disp.Set_Preamble_Boost(kPreBoost);
            rx_disp.Set_Preamble_Reps(kPreReps);
            rx_disp.Set_CW_Cancel(false);
            rx_disp.Set_AJC_Enabled(false);
            rx_disp.Set_SoftClip_Policy(SoftClipPolicy::NEVER);
            rx_disp.Set_Packet_Callback(on_pkt);
            rx_disp.Update_Adaptive_BPS(1000);

            // 가드
            for (int i = 0; i < 256; ++i)
                rx_disp.Feed_Chip(0, 0);
            // 위상 90° 회전: I'=Q, Q'=-I
            for (int i = 0; i < n; ++i)
                rx_disp.Feed_Chip(sigQ[i],
                                  static_cast<int16_t>(-sigI[i]));
            for (int i = 0; i < 256; ++i)
                rx_disp.Feed_Chip(0, 0);

            bool success =
                (g_last.success_mask == DecodedPacket::DECODE_MASK_OK);
            int phase_val = static_cast<int>(rx_disp.Get_Phase());
            std::printf("    [%d] phase=%d %s\n", t, phase_val,
                        success ? "OK" : "FAIL");
            if (success)
                ok++;
        }
        std::printf("  Test 2 (90°): %d/10\n", ok);
        total_ok += ok;
    }

    // ── 테스트 3: AWGN 10dB J/S ──
    {
        std::printf("  -- Test 3: AWGN 10dB --\n");
        int ok = 0;
        std::mt19937 rng(0xABCD1234u);
        for (int t = 0; t < 10; ++t) {
            g_last = DecodedPacket{};
            uint32_t ds = 0xF00D00u ^ (uint32_t)(t * 0x9E3779B9u);

            HTS_V400_Dispatcher tx_disp;
            tx_disp.Set_IR_Mode(true);
            tx_disp.Set_Seed(ds);
            tx_disp.Set_Preamble_Boost(kPreBoost);
            tx_disp.Set_Preamble_Reps(kPreReps);
            tx_disp.Set_CW_Cancel(false);
            tx_disp.Set_AJC_Enabled(false);
            tx_disp.Set_SoftClip_Policy(SoftClipPolicy::NEVER);
            tx_disp.Set_Packet_Callback(on_pkt);
            tx_disp.Update_Adaptive_BPS(1000);

            uint8_t info[8]{};
            for (int b = 0; b < 8; ++b)
                info[b] = (uint8_t)((ds >> (b * 4)) ^ (unsigned)(t + b));

            std::vector<int16_t> sigI(kMaxC), sigQ(kMaxC);
            int n = tx_disp.Build_Packet(PayloadMode::DATA, info, 8, kAmp,
                                         sigI.data(), sigQ.data(), kMaxC);
            if (n <= 0)
                continue;

            // 송신 패킷 전체 n칩: 칩당 평균 I²+Q² (스케일/정규화 없음)
            double P_signal = 0.0;
            for (int i = 0; i < n; ++i) {
                const double di = static_cast<double>(sigI[i]);
                const double dq = static_cast<double>(sigQ[i]);
                P_signal += di * di + dq * dq;
            }
            P_signal /= static_cast<double>(n);

            // SNR = P_signal / (2σ²), I·Q 각 N(0,σ²) 가산 → σ = √(P_signal/(2·snr_linear))
            constexpr double kTest3SnrDb = 10.0;
            const double snr_linear =
                std::pow(10.0, kTest3SnrDb / 10.0);
            const double sigma =
                std::sqrt(P_signal / (2.0 * snr_linear));
            // 순수 가산 AWGN: tx 칩에 곱/나눔 없음 (σ만으로 잡음 크기 결정)
            std::normal_distribution<double> gauss(0.0, sigma);
            std::vector<int16_t> nI(n), nQ(n);
            for (int i = 0; i < n; ++i) {
                const long long ri =
                    static_cast<long long>(sigI[i]) +
                    static_cast<long long>(std::llround(gauss(rng)));
                const long long rq =
                    static_cast<long long>(sigQ[i]) +
                    static_cast<long long>(std::llround(gauss(rng)));
                nI[i] = static_cast<int16_t>(
                    std::max(-32768LL, std::min(32767LL, ri)));
                nQ[i] = static_cast<int16_t>(
                    std::max(-32768LL, std::min(32767LL, rq)));
            }

            HTS_V400_Dispatcher rx_disp;
            rx_disp.Set_IR_Mode(true);
            rx_disp.Set_Seed(ds);
            rx_disp.Set_Preamble_Boost(kPreBoost);
            rx_disp.Set_Preamble_Reps(kPreReps);
            rx_disp.Set_CW_Cancel(false);
            rx_disp.Set_AJC_Enabled(false);
            rx_disp.Set_SoftClip_Policy(SoftClipPolicy::NEVER);
            rx_disp.Set_Packet_Callback(on_pkt);
            rx_disp.Update_Adaptive_BPS(1000);

            // 선행 가드: Test 1/2와 동일 0칩 (AWGN 가드는 Phase0 배경만 키워 avg_o 악화)
            for (int i = 0; i < 256; ++i)
                rx_disp.Feed_Chip(0, 0);
            for (int i = 0; i < n; ++i)
                rx_disp.Feed_Chip(nI[i], nQ[i]);
            for (int i = 0; i < 256; ++i)
                rx_disp.Feed_Chip(0, 0);

            bool success =
                (g_last.success_mask == DecodedPacket::DECODE_MASK_OK);
            int phase_val = static_cast<int>(rx_disp.Get_Phase());
            std::printf("    [%d] phase=%d %s\n", t, phase_val,
                        success ? "OK" : "FAIL");
            if (success)
                ok++;
        }
        std::printf("  Test 3 (AWGN 10dB): %d/10\n", ok);
        total_ok += ok;
    }

    // ── 테스트 4: 타이밍 오프셋 δ=17칩 ──
    {
        std::printf("  -- Test 4: Timing offset +17 chips --\n");
        int ok = 0;
        for (int t = 0; t < 10; ++t) {
            g_last = DecodedPacket{};
            uint32_t ds = 0xF00D00u ^ (uint32_t)(t * 0x9E3779B9u);

            HTS_V400_Dispatcher tx_disp;
            tx_disp.Set_IR_Mode(true);
            tx_disp.Set_Seed(ds);
            tx_disp.Set_Preamble_Boost(kPreBoost);
            tx_disp.Set_Preamble_Reps(kPreReps);
            tx_disp.Set_CW_Cancel(false);
            tx_disp.Set_AJC_Enabled(false);
            tx_disp.Set_SoftClip_Policy(SoftClipPolicy::NEVER);
            tx_disp.Set_Packet_Callback(on_pkt);
            tx_disp.Update_Adaptive_BPS(1000);

            uint8_t info[8]{};
            for (int b = 0; b < 8; ++b)
                info[b] = (uint8_t)((ds >> (b * 4)) ^ (unsigned)(t + b));

            std::vector<int16_t> sigI(kMaxC), sigQ(kMaxC);
            int n = tx_disp.Build_Packet(PayloadMode::DATA, info, 8, kAmp,
                                         sigI.data(), sigQ.data(), kMaxC);
            if (n <= 0)
                continue;

            HTS_V400_Dispatcher rx_disp;
            rx_disp.Set_IR_Mode(true);
            rx_disp.Set_Seed(ds);
            rx_disp.Set_Preamble_Boost(kPreBoost);
            rx_disp.Set_Preamble_Reps(kPreReps);
            rx_disp.Set_CW_Cancel(false);
            rx_disp.Set_AJC_Enabled(false);
            rx_disp.Set_SoftClip_Policy(SoftClipPolicy::NEVER);
            rx_disp.Set_Packet_Callback(on_pkt);
            rx_disp.Update_Adaptive_BPS(1000);

            // 가드 256 + 오프셋 17칩
            for (int i = 0; i < 256 + 17; ++i)
                rx_disp.Feed_Chip(0, 0);
            for (int i = 0; i < n; ++i)
                rx_disp.Feed_Chip(sigI[i], sigQ[i]);
            for (int i = 0; i < 256; ++i)
                rx_disp.Feed_Chip(0, 0);

            bool success =
                (g_last.success_mask == DecodedPacket::DECODE_MASK_OK);
            int phase_val = static_cast<int>(rx_disp.Get_Phase());
            std::printf("    [%d] phase=%d %s\n", t, phase_val,
                        success ? "OK" : "FAIL");
            if (success)
                ok++;
        }
        std::printf("  Test 4 (offset +17): %d/10\n", ok);
        total_ok += ok;
    }

    std::printf("  T6-SIM total: %d/40\n", total_ok);
}
static void test_T6(PlutoCtx &p) {
    std::printf("\n══ T6: 디스패처 동기화 ══\n");
    static constexpr int kPreReps = 4;
    static constexpr int kPreBoost = 1;
    static constexpr int kMaxC = 1024 + (FEC_HARQ::NSYM64 + kPreReps + 4) * 64;
    // DMA 프라이밍
    {
        std::vector<int16_t> wI(PLUTO_BUF_SAMPLES), wQ(PLUTO_BUF_SAMPLES);
        for (int i = 0; i < PLUTO_BUF_SAMPLES; ++i) {
            int16_t v = ((i / 32) & 1) ? 1000 : -1000;
            wI[i] = v;
            wQ[i] = v;
        }
        std::vector<int16_t> rI(PLUTO_BUF_SAMPLES), rQ(PLUTO_BUF_SAMPLES);
        for (int w = 0; w < 3; ++w)
            pluto_tx_then_rx(p, wI.data(), wQ.data(), PLUTO_BUF_SAMPLES,
                             rI.data(), rQ.data(), PLUTO_BUF_SAMPLES);
    }
    int sync_ok = 0;
    for (int t = 0; t < 10; ++t) {
        if (t > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        g_last = DecodedPacket{};
        uint32_t ds = 0xF00D00u ^ (uint32_t)(t * 0x9E3779B9u);
        // TX 디스패처
        HTS_V400_Dispatcher tx_disp;
        tx_disp.Set_IR_Mode(true);
        tx_disp.Set_Seed(ds);
        tx_disp.Set_Preamble_Boost(kPreBoost);
        tx_disp.Set_Preamble_Reps(kPreReps);
        tx_disp.Set_CW_Cancel(false);
        tx_disp.Set_AJC_Enabled(false);
        tx_disp.Set_SoftClip_Policy(SoftClipPolicy::NEVER);
        tx_disp.Set_Packet_Callback(on_pkt);
        tx_disp.Update_Adaptive_BPS(1000);
        uint8_t info[8]{};
        for (int b = 0; b < 8; ++b)
            info[b] = (uint8_t)((ds >> (b * 4)) ^ (unsigned)(t + b));
        std::vector<int16_t> sigI(kMaxC), sigQ(kMaxC);
        int n = tx_disp.Build_Packet(PayloadMode::DATA, info, 8, kAmp,
                                     sigI.data(), sigQ.data(), kMaxC);
        if (n <= 0) {
            std::printf("  [%2d] Build FAIL\n", t);
            continue;
        }
        // TX→RX
        std::vector<int16_t> txI(PLUTO_BUF_SAMPLES), txQ(PLUTO_BUF_SAMPLES);
        fill_tx_repeated(txI.data(), txQ.data(), PLUTO_BUF_SAMPLES,
                         sigI.data(), sigQ.data(), n);
        std::vector<int16_t> rxI(PLUTO_BUF_SAMPLES), rxQ(PLUTO_BUF_SAMPLES);
        int nr = pluto_tx_then_rx(p, txI.data(), txQ.data(), PLUTO_BUF_SAMPLES,
                                  rxI.data(), rxQ.data(), PLUTO_BUF_SAMPLES);
        if (nr <= 0) {
            std::printf("  [%2d] RX FAIL\n", t);
            continue;
        }
        // AGC만 (phase correction 제거)
        apply_digital_agc(rxI.data(), rxQ.data(), nr);
        // 정렬
        int ss = find_signal_start(rxI.data(), rxQ.data(), nr,
                                   sigI.data(), sigQ.data(), n);
        if (ss + n > nr)
            ss = 0;
        // RX 디스패처 (TX와 분리)
        HTS_V400_Dispatcher rx_disp;
        rx_disp.Set_IR_Mode(true);
        rx_disp.Set_Seed(ds);
        rx_disp.Set_Preamble_Boost(kPreBoost);
        rx_disp.Set_Preamble_Reps(kPreReps);
        rx_disp.Set_CW_Cancel(false);
        rx_disp.Set_AJC_Enabled(false);
        rx_disp.Set_SoftClip_Policy(SoftClipPolicy::NEVER);
        rx_disp.Set_Packet_Callback(on_pkt);
        rx_disp.Update_Adaptive_BPS(1000);
        // 프리앰블 전 256칩 포함 → Phase 0 에너지 상승·피크 검출 여유
        int feed_start = ss - 256;
        if (feed_start < 0)
            feed_start = 0;
        int feed_end = ss + n + 256;
        if (feed_end > nr)
            feed_end = nr;
        for (int i = feed_start; i < feed_end; ++i)
            rx_disp.Feed_Chip(rxI[i], rxQ[i]);
        bool ok = (g_last.success_mask == DecodedPacket::DECODE_MASK_OK);
        int phase_val = static_cast<int>(rx_disp.Get_Phase());
        std::printf("  [%2d] tx=%d rx=%d ss=%d phase=%d %s\n",
                    t, n, nr, ss, phase_val, ok ? "OK" : "FAIL");
        if (ok)
            sync_ok++;
    }
    std::printf("  T6: %d/10\n", sync_ok);
}
static void test_T7(PlutoCtx &p) {
    std::printf("\n══ T7: IR-HARQ 바라지 20dB ══\n");
    const int bps = 3, nc = 64;
    const int nsym = FEC_HARQ::nsym_for_bps(bps);
    const int total = nsym * nc;
    int ok = 0;
    double ar = 0;
    for (int t = 0; t < 10; ++t) {
        uint32_t ts = 0xBEEF00u ^ (uint32_t)(t * 0x9E3779B9u);
        uint32_t il = 0xA5A5A5A5u ^ ts;
        uint8_t info[8];
        gen_info(ts, info);
        HTS_V400_Dispatcher disp;
        disp.Set_Seed(0xA5A5A5A5u ^ ts);
        disp.Set_IR_Mode(true);
        disp.Set_CW_Cancel(true);
        disp.Set_AJC_Enabled(true);
        disp.Set_SoftClip_Policy(SoftClipPolicy::ALWAYS);
        disp.Set_Packet_Callback(on_pkt);
        disp.Set_Lab_BPS64(bps);
        disp.Set_Lab_IQ_Mode_Jam_Harness();
        bool dec = false;
        int rnd = 0;
        for (int rv = 0; rv < 32 && !dec; ++rv) {
            g_last = DecodedPacket{};
            FEC_HARQ::WorkBuf wb{};
            uint8_t syms[FEC_HARQ::NSYM64]{};
            std::memset(&wb, 0, sizeof(wb));
            FEC_HARQ::Encode64_IR(info, 8, syms, il, bps, rv & 3, wb);
            std::vector<int16_t> sI(total), sQ(total);
            // DAC 클리핑 방지: J/S=20dB에서 3σ=1800 < 2048
            static constexpr int16_t kAmpT7 = 60;
            for (int s = 0; s < nsym; ++s)
                walsh_enc(syms[s], nc, kAmpT7, &sI[s * nc], &sQ[s * nc]);
            std::vector<int16_t> refI = sI, refQ = sQ;
            std::mt19937 rng(ts ^ (uint32_t)(rv * 0x85EBCA6Bu));
            // kAmpT7 기준 바라지 (kAmpD 대신 60.0)
            {
                double sigma_t7 = 60.0 * std::sqrt(std::pow(10.0, 20.0 / 10.0));
                std::normal_distribution<double> nd(0.0, sigma_t7);
                for (int i = 0; i < total; ++i) {
                    double vi = (double)sI[i] + nd(rng);
                    double vq = (double)sQ[i] + nd(rng);
                    if (vi > 2047)
                        vi = 2047;
                    if (vi < -2048)
                        vi = -2048;
                    if (vq > 2047)
                        vq = 2047;
                    if (vq < -2048)
                        vq = -2048;
                    sI[i] = (int16_t)std::lround(vi);
                    sQ[i] = (int16_t)std::lround(vq);
                }
            }
            std::vector<int16_t> txI(PLUTO_BUF_SAMPLES), txQ(PLUTO_BUF_SAMPLES);
            fill_tx_repeated(txI.data(), txQ.data(), PLUTO_BUF_SAMPLES,
                             sI.data(), sQ.data(), total);
            std::vector<int16_t> rxI(PLUTO_BUF_SAMPLES), rxQ(PLUTO_BUF_SAMPLES);
            int nr =
                pluto_tx_then_rx(p, txI.data(), txQ.data(), PLUTO_BUF_SAMPLES,
                                 rxI.data(), rxQ.data(), PLUTO_BUF_SAMPLES);
            if (nr <= 0)
                break;
            apply_digital_agc(rxI.data(), rxQ.data(), nr);
            apply_phase_correction(rxI.data(), rxQ.data(), nr);
            int ss = find_signal_start(rxI.data(), rxQ.data(), nr, refI.data(),
                                       refQ.data(), total);
            if (ss + total > nr)
                ss = 0;
            if (rv == 0) {
                disp.Inject_Payload_Phase(PayloadMode::DATA, bps);
            }
            for (int i = 0; i < total; ++i) {
                if (rv == 0)
                    disp.Feed_Chip(rxI[ss + i], rxQ[ss + i]);
                else
                    disp.Feed_Retx_Chip(rxI[ss + i], rxQ[ss + i]);
            }
            dec = (g_last.success_mask == DecodedPacket::DECODE_MASK_OK);
            rnd = rv + 1;
            if (dec && (g_last.data_len != 8 ||
                        std::memcmp(g_last.data, info, 8) != 0))
                dec = false;
        }
        std::printf("  [%2d] %s R=%d\n", t, dec ? "OK" : "FAIL", rnd);
        if (dec) {
            ok++;
            ar += rnd;
        }
    }
    if (ok > 0)
        ar /= ok;
    std::printf("  T7: %d/10 avg %.1fR\n", ok, ar);
}
static void test_T8(PlutoCtx &p) {
    std::printf("\n══ T8: 16칩 VOICE ══\n");
    int ok = 0;
    for (int t = 0; t < 10; ++t) {
        uint32_t ts = 0xAAAA00u ^ (uint32_t)(t * 0x9E3779B9u);
        uint8_t info[8];
        gen_info(ts, info);
        FEC_HARQ::IR_RxState ir{};
        FEC_HARQ::IR_Init(ir);
        FEC_HARQ::WorkBuf wb{};
        uint8_t out[8]{};
        int ol = 0;
        bool dec =
            fec16_txrx_decode(p, info, 0xA5A5A5A5u ^ ts, 0, ir, wb, out, &ol);
        std::printf("  [%2d] %s\n", t, dec ? "OK" : "FAIL");
        if (dec)
            ok++;
    }
    std::printf("  T8: %d/10\n", ok);
}
static void test_T9(PlutoCtx &p) {
    std::printf("\n══ T9: 내구 (100패킷) ══\n");
    int ok = 0;
    auto t0 = std::chrono::steady_clock::now();
    for (int t = 0; t < 100; ++t) {
        uint32_t ts = 0x999900u ^ (uint32_t)(t * 0x9E3779B9u);
        uint8_t info[8];
        gen_info(ts, info);
        FEC_HARQ::IR_RxState ir{};
        FEC_HARQ::IR_Init(ir);
        FEC_HARQ::WorkBuf wb{};
        uint8_t out[8]{};
        int ol = 0;
        if (fec_txrx_decode(p, info, 0xA5A5A5A5u ^ ts, 4, 64, 0, ir, wb, out,
                            &ol))
            ok++;
        if ((t + 1) % 25 == 0)
            std::printf("  %d/100 (ok=%d)\n", t + 1, ok);
    }
    auto t1 = std::chrono::steady_clock::now();
    double sec = std::chrono::duration<double>(t1 - t0).count();
    std::printf("  T9: %d/100 %.1fs\n", ok, sec);
}
// ════════════════════════════════════════════════════════════
//  T10~T20: 추가 검증 테스트
// ════════════════════════════════════════════════════════════
// ── 재밍 합성 헬퍼 ──
/// @brief 다중 톤 CW 재밍 합성 (주파수 f_i = base + i*spacing, 칩 주기)
static void add_multi_cw(int16_t *I, int16_t *Q, int n, double js_db,
                         int num_tones, double base_period, double spacing) {
    if (js_db < 0 || num_tones <= 0)
        return;
    // 각 톤의 에너지 합 = 총 에너지 → 톤당 진폭 = a_total / sqrt(num_tones)
    double a_total = kAmpD * std::sqrt(std::pow(10.0, js_db / 10.0));
    double a_per = a_total / std::sqrt(static_cast<double>(num_tones));
    for (int i = 0; i < n; ++i) {
        double vi = static_cast<double>(I[i]);
        double vq = static_cast<double>(Q[i]);
        for (int t = 0; t < num_tones; ++t) {
            double freq = 1.0 / (base_period + t * spacing);
            double ph = 2.0 * 3.14159265358979 * i * freq;
            vi += a_per * std::cos(ph);
            vq += a_per * std::sin(ph);
        }
        if (vi > 2047)
            vi = 2047;
        if (vi < -2048)
            vi = -2048;
        if (vq > 2047)
            vq = 2047;
        if (vq < -2048)
            vq = -2048;
        I[i] = static_cast<int16_t>(std::lround(vi));
        Q[i] = static_cast<int16_t>(std::lround(vq));
    }
}
/// @brief EMP/펄스 재밍 합성 — duty_pct: 펄스 ON 비율(%), burst_period: 칩 단위
/// 주기
static void add_emp_pulse(int16_t *I, int16_t *Q, int n, double js_db,
                          int duty_pct, int burst_period, std::mt19937 &rng) {
    if (js_db < 0)
        return;
    // 펄스 ON 구간의 에너지 = 전체 E_jam → ON 구간 σ 보정: σ_on = σ_avg /
    // sqrt(duty)
    double duty = static_cast<double>(duty_pct) / 100.0;
    if (duty < 0.01)
        duty = 0.01;
    double sigma_avg = kAmpD * std::sqrt(std::pow(10.0, js_db / 10.0));
    double sigma_on = sigma_avg / std::sqrt(duty);
    std::normal_distribution<double> nd(0.0, sigma_on);
    for (int i = 0; i < n; ++i) {
        int phase_in_burst = i % burst_period;
        int on_len = static_cast<int>(burst_period * duty);
        if (on_len < 1)
            on_len = 1;
        if (phase_in_burst < on_len) {
            double vi = static_cast<double>(I[i]) + nd(rng);
            double vq = static_cast<double>(Q[i]) + nd(rng);
            if (vi > 2047)
                vi = 2047;
            if (vi < -2048)
                vi = -2048;
            if (vq > 2047)
                vq = 2047;
            if (vq < -2048)
                vq = -2048;
            I[i] = static_cast<int16_t>(std::lround(vi));
            Q[i] = static_cast<int16_t>(std::lround(vq));
        }
    }
}
/// @brief CFO 적용 (주파수 오프셋 시뮬레이션)
/// @param cfo_hz 주파수 오프셋 (Hz)
/// @param sample_rate 샘플 레이트 (Hz)
static void apply_cfo(int16_t *I, int16_t *Q, int n, double cfo_hz,
                      double sample_rate) {
    for (int i = 0; i < n; ++i) {
        double ph = 2.0 * 3.14159265358979 * cfo_hz * i / sample_rate;
        double cosP = std::cos(ph);
        double sinP = std::sin(ph);
        double di = static_cast<double>(I[i]);
        double dq = static_cast<double>(Q[i]);
        double ri = di * cosP - dq * sinP;
        double rq = di * sinP + dq * cosP;
        if (ri > 32767)
            ri = 32767;
        if (ri < -32768)
            ri = -32768;
        if (rq > 32767)
            rq = 32767;
        if (rq < -32768)
            rq = -32768;
        I[i] = static_cast<int16_t>(std::lround(ri));
        Q[i] = static_cast<int16_t>(std::lround(rq));
    }
}
/// @brief Partial-band 재밍 (지정 칩 범위에만 바라지)
static void add_partial_band(int16_t *I, int16_t *Q, int n, double js_db,
                             double band_fraction, std::mt19937 &rng) {
    if (js_db < 0 || band_fraction <= 0)
        return;
    double sigma = kAmpD * std::sqrt(std::pow(10.0, js_db / 10.0));
    // partial-band: 에너지 집중 → σ_partial = σ / sqrt(fraction)
    double sigma_p = sigma / std::sqrt(band_fraction);
    std::normal_distribution<double> nd(0.0, sigma_p);
    int jam_len = static_cast<int>(n * band_fraction);
    if (jam_len < 1)
        jam_len = 1;
    // 시작 위치: 재밍 영역을 반복 구간 전반부에 배치
    for (int i = 0; i < jam_len && i < n; ++i) {
        double vi = static_cast<double>(I[i]) + nd(rng);
        double vq = static_cast<double>(Q[i]) + nd(rng);
        if (vi > 2047)
            vi = 2047;
        if (vi < -2048)
            vi = -2048;
        if (vq > 2047)
            vq = 2047;
        if (vq < -2048)
            vq = -2048;
        I[i] = static_cast<int16_t>(std::lround(vi));
        Q[i] = static_cast<int16_t>(std::lround(vq));
    }
}
// ════════════════════════════════════════════════════════════
//  T10: CW 재밍 ECCM 검증 (단일 톤 / 다중 톤)
// ════════════════════════════════════════════════════════════
static void test_T10(PlutoCtx &p) {
    std::printf("\n══ T10: CW 재밍 ECCM (단일톤+다중톤) ══\n");
    const int bps = 3, nc = 64;
    const int nsym = FEC_HARQ::nsym_for_bps(bps);
    const int total = nsym * nc;
    struct CwCase {
        const char *label;
        int num_tones;
        double base_period; // 칩 주기
        double spacing;     // 톤 간 간격
    };
    // cw_cancel_64_()는 8칩 주기 단일 톤에 최적화 → 비-8주기 및 다중톤 검증
    const CwCase cases[] = {
        {"1-tone(8chip)", 1, 8.0, 0.0},
        {"1-tone(12chip)", 1, 12.0, 0.0},
        {"2-tone(8+16)", 2, 8.0, 8.0},
        {"3-tone(8+12+20)", 3, 8.0, 4.0},
    };
    const double js[] = {10, 15, 20};
    for (auto &cc : cases) {
        std::printf("  [%s]\n", cc.label);
        for (double j : js) {
            int ok = 0;
            double ar = 0;
            for (int t = 0; t < 10; ++t) {
                uint32_t ts =
                    0xC01000u ^ static_cast<uint32_t>(t * 0x9E3779B9u);
                uint32_t il = 0xA5A5A5A5u ^ ts;
                uint8_t info[8];
                gen_info(ts, info);
                HTS_V400_Dispatcher disp;
                disp.Set_Seed(il);
                disp.Set_IR_Mode(true);
                disp.Set_CW_Cancel(true);
                disp.Set_AJC_Enabled(true);
                disp.Set_SoftClip_Policy(SoftClipPolicy::ALWAYS);
                disp.Set_Packet_Callback(on_pkt);
                disp.Set_Lab_BPS64(bps);
                disp.Set_Lab_IQ_Mode_Jam_Harness();
                bool dec = false;
                int rnd = 0;
                for (int rv = 0; rv < 32 && !dec; ++rv) {
                    g_last = DecodedPacket{};
                    FEC_HARQ::WorkBuf wb{};
                    uint8_t syms[FEC_HARQ::NSYM64]{};
                    std::memset(&wb, 0, sizeof(wb));
                    FEC_HARQ::Encode64_IR(info, 8, syms, il, bps, rv & 3, wb);
                    std::vector<int16_t> sI(total), sQ(total);
                    for (int s = 0; s < nsym; ++s)
                        walsh_enc(syms[s], nc, kAmp, &sI[s * nc], &sQ[s * nc]);
                    std::vector<int16_t> refI = sI, refQ = sQ;
                    add_multi_cw(sI.data(), sQ.data(), total, j, cc.num_tones,
                                 cc.base_period, cc.spacing);
                    std::vector<int16_t> txI(PLUTO_BUF_SAMPLES),
                        txQ(PLUTO_BUF_SAMPLES);
                    fill_tx_repeated(txI.data(), txQ.data(), PLUTO_BUF_SAMPLES,
                                     sI.data(), sQ.data(), total);
                    std::vector<int16_t> rxI(PLUTO_BUF_SAMPLES),
                        rxQ(PLUTO_BUF_SAMPLES);
                    int nr = pluto_tx_then_rx(p, txI.data(), txQ.data(),
                                              PLUTO_BUF_SAMPLES, rxI.data(),
                                              rxQ.data(), PLUTO_BUF_SAMPLES);
                    if (nr <= 0)
                        break;
                    apply_digital_agc(rxI.data(), rxQ.data(), nr);
                    apply_phase_correction(rxI.data(), rxQ.data(), nr);
                    int ss = find_signal_start(rxI.data(), rxQ.data(), nr,
                                               refI.data(), refQ.data(), total);
                    if (ss + total > nr)
                        ss = 0;
                    if (rv == 0)
                        disp.Inject_Payload_Phase(PayloadMode::DATA, bps);
                    for (int i = 0; i < total; ++i) {
                        if (rv == 0)
                            disp.Feed_Chip(rxI[ss + i], rxQ[ss + i]);
                        else
                            disp.Feed_Retx_Chip(rxI[ss + i], rxQ[ss + i]);
                    }
                    dec =
                        (g_last.success_mask == DecodedPacket::DECODE_MASK_OK);
                    rnd = rv + 1;
                    if (dec && (g_last.data_len != 8 ||
                                std::memcmp(g_last.data, info, 8) != 0))
                        dec = false;
                }
                if (dec) {
                    ok++;
                    ar += rnd;
                }
            }
            if (ok > 0)
                ar /= ok;
            std::printf("    J/S=%2.0fdB %d/10 avg %.1fR\n", j, ok, ar);
        }
    }
}
// ════════════════════════════════════════════════════════════
//  T11: EMP/펄스 재밍 내성
// ════════════════════════════════════════════════════════════
static void test_T11(PlutoCtx &p) {
    std::printf("\n══ T11: EMP/펄스 재밍 ══\n");
    const int bps = 3, nc = 64;
    const int nsym = FEC_HARQ::nsym_for_bps(bps);
    const int total = nsym * nc;
    const int duties[] = {10, 30, 50};
    const double js_db = 20.0;
    static constexpr int BURST_PERIOD = 128; // 128칩 주기
    for (int duty : duties) {
        int ok = 0;
        double ar = 0;
        for (int t = 0; t < 10; ++t) {
            uint32_t ts = 0xE0B100u ^ static_cast<uint32_t>(t * 0x9E3779B9u);
            uint32_t il = 0xA5A5A5A5u ^ ts;
            uint8_t info[8];
            gen_info(ts, info);
            HTS_V400_Dispatcher disp;
            disp.Set_Seed(il);
            disp.Set_IR_Mode(true);
            disp.Set_CW_Cancel(true);
            disp.Set_AJC_Enabled(true);
            disp.Set_SoftClip_Policy(SoftClipPolicy::ALWAYS);
            disp.Set_Packet_Callback(on_pkt);
            disp.Set_Lab_BPS64(bps);
            disp.Set_Lab_IQ_Mode_Jam_Harness();
            bool dec = false;
            int rnd = 0;
            for (int rv = 0; rv < 32 && !dec; ++rv) {
                g_last = DecodedPacket{};
                FEC_HARQ::WorkBuf wb{};
                uint8_t syms[FEC_HARQ::NSYM64]{};
                std::memset(&wb, 0, sizeof(wb));
                FEC_HARQ::Encode64_IR(info, 8, syms, il, bps, rv & 3, wb);
                std::vector<int16_t> sI(total), sQ(total);
                for (int s = 0; s < nsym; ++s)
                    walsh_enc(syms[s], nc, kAmp, &sI[s * nc], &sQ[s * nc]);
                std::vector<int16_t> refI = sI, refQ = sQ;
                std::mt19937 rng(ts ^ static_cast<uint32_t>(rv * 0x85EBCA6Bu));
                add_emp_pulse(sI.data(), sQ.data(), total, js_db, duty,
                              BURST_PERIOD, rng);
                std::vector<int16_t> txI(PLUTO_BUF_SAMPLES),
                    txQ(PLUTO_BUF_SAMPLES);
                fill_tx_repeated(txI.data(), txQ.data(), PLUTO_BUF_SAMPLES,
                                 sI.data(), sQ.data(), total);
                std::vector<int16_t> rxI(PLUTO_BUF_SAMPLES),
                    rxQ(PLUTO_BUF_SAMPLES);
                int nr = pluto_tx_then_rx(p, txI.data(), txQ.data(),
                                          PLUTO_BUF_SAMPLES, rxI.data(),
                                          rxQ.data(), PLUTO_BUF_SAMPLES);
                if (nr <= 0)
                    break;
                apply_digital_agc(rxI.data(), rxQ.data(), nr);
                apply_phase_correction(rxI.data(), rxQ.data(), nr);
                int ss = find_signal_start(rxI.data(), rxQ.data(), nr,
                                           refI.data(), refQ.data(), total);
                if (ss + total > nr)
                    ss = 0;
                if (rv == 0)
                    disp.Inject_Payload_Phase(PayloadMode::DATA, bps);
                for (int i = 0; i < total; ++i) {
                    if (rv == 0)
                        disp.Feed_Chip(rxI[ss + i], rxQ[ss + i]);
                    else
                        disp.Feed_Retx_Chip(rxI[ss + i], rxQ[ss + i]);
                }
                dec = (g_last.success_mask == DecodedPacket::DECODE_MASK_OK);
                rnd = rv + 1;
                if (dec && (g_last.data_len != 8 ||
                            std::memcmp(g_last.data, info, 8) != 0))
                    dec = false;
            }
            if (dec) {
                ok++;
                ar += rnd;
            }
        }
        if (ok > 0)
            ar /= ok;
        std::printf("  duty=%d%% J/S=%.0fdB %d/10 avg %.1fR\n", duty, js_db, ok,
                    ar);
    }
}
// ════════════════════════════════════════════════════════════
//  T12: 다중 사용자 CDMA (Walsh 코드 분리)
// ════════════════════════════════════════════════════════════
static void test_T12(PlutoCtx &p) {
    std::printf("\n══ T12: 다중 사용자 CDMA ══\n");
    // 2~4 사용자 동시 전송: 각 사용자 독립 Walsh 심볼 → 합성 → 디코딩
    // 단일 Pluto 루프백이므로 SW 합성 후 RF 전송
    const int bps = 4, nc = 64;
    const int nsym = FEC_HARQ::nsym_for_bps(bps);
    const int total = nsym * nc;
    const int user_counts[] = {2, 3, 4};
    for (int num_users : user_counts) {
        int ok_target = 0; // 타겟 사용자(user 0) 디코딩 성공
        for (int t = 0; t < 10; ++t) {
            // 각 사용자별 독립 데이터 + 인터리브 시드
            struct UserCtx {
                uint8_t info[8];
                uint32_t il;
                uint8_t syms[FEC_HARQ::NSYM64];
            };
            std::vector<UserCtx> users(static_cast<size_t>(num_users));
            for (int u = 0; u < num_users; ++u) {
                uint32_t ts =
                    0xCD0A00u ^
                    static_cast<uint32_t>((t * num_users + u) * 0x9E3779B9u);
                gen_info(ts, users[static_cast<size_t>(u)].info);
                users[static_cast<size_t>(u)].il = 0xA5A5A5A5u ^ ts;
                FEC_HARQ::WorkBuf wb{};
                std::memset(&wb, 0, sizeof(wb));
                FEC_HARQ::Encode64_IR(users[static_cast<size_t>(u)].info, 8,
                                      users[static_cast<size_t>(u)].syms,
                                      users[static_cast<size_t>(u)].il, bps, 0,
                                      wb);
            }
            // 모든 사용자 신호 합산
            std::vector<int16_t> sI(total, 0), sQ(total, 0);
            for (int u = 0; u < num_users; ++u) {
                for (int s = 0; s < nsym; ++s) {
                    int16_t chipI[64], chipQ[64];
                    walsh_enc(users[static_cast<size_t>(u)].syms[s], nc, kAmp,
                              chipI, chipQ);
                    for (int c = 0; c < nc; ++c) {
                        int32_t vi =
                            static_cast<int32_t>(sI[s * nc + c]) + chipI[c];
                        int32_t vq =
                            static_cast<int32_t>(sQ[s * nc + c]) + chipQ[c];
                        if (vi > 2047)
                            vi = 2047;
                        if (vi < -2048)
                            vi = -2048;
                        if (vq > 2047)
                            vq = 2047;
                        if (vq < -2048)
                            vq = -2048;
                        sI[s * nc + c] = static_cast<int16_t>(vi);
                        sQ[s * nc + c] = static_cast<int16_t>(vq);
                    }
                }
            }
            // 타겟 사용자(user 0)에 대한 클린 레퍼런스 (정렬용)
            std::vector<int16_t> refI(total), refQ(total);
            for (int s = 0; s < nsym; ++s)
                walsh_enc(users[0].syms[s], nc, kAmp, &refI[s * nc],
                          &refQ[s * nc]);
            std::vector<int16_t> txI(PLUTO_BUF_SAMPLES), txQ(PLUTO_BUF_SAMPLES);
            fill_tx_repeated(txI.data(), txQ.data(), PLUTO_BUF_SAMPLES,
                             sI.data(), sQ.data(), total);
            std::vector<int16_t> rxI(PLUTO_BUF_SAMPLES), rxQ(PLUTO_BUF_SAMPLES);
            int nr =
                pluto_tx_then_rx(p, txI.data(), txQ.data(), PLUTO_BUF_SAMPLES,
                                 rxI.data(), rxQ.data(), PLUTO_BUF_SAMPLES);
            if (nr <= 0)
                continue;
            apply_digital_agc(rxI.data(), rxQ.data(), nr);
            apply_phase_correction(rxI.data(), rxQ.data(), nr);
            int ss = find_signal_start(rxI.data(), rxQ.data(), nr, refI.data(),
                                       refQ.data(), total);
            if (ss + total > nr)
                ss = 0;
            // FEC 직접 디코딩 (user 0의 인터리브 시드)
            FEC_HARQ::IR_RxState ir{};
            FEC_HARQ::IR_Init(ir);
            FEC_HARQ::WorkBuf wb{};
            std::memset(&wb, 0, sizeof(wb));
            uint8_t out[8]{};
            int olen = 0;
            bool dec =
                FEC_HARQ::Decode64_IR(&rxI[ss], &rxQ[ss], nsym, nc, bps,
                                      users[0].il, 0, ir, out, &olen, wb);
            if (dec && olen == 8 && std::memcmp(out, users[0].info, 8) == 0)
                ok_target++;
            std::printf(
                "  [%2d] %d-user %s\n", t, num_users,
                (dec && olen == 8 && std::memcmp(out, users[0].info, 8) == 0)
                    ? "OK"
                    : "FAIL");
        }
        std::printf("  T12(%d-user): %d/10\n", num_users, ok_target);
    }
}
// ════════════════════════════════════════════════════════════
//  T13: TX↔RX 주파수 오프셋(CFO) 내성
// ════════════════════════════════════════════════════════════
static void test_T13(PlutoCtx &p) {
    std::printf("\n══ T13: CFO 내성 ══\n");
    const int bps = 4, nc = 64;
    const int nsym = FEC_HARQ::nsym_for_bps(bps);
    const int total = nsym * nc;
    const double cfo_list[] = {100, 500, 1000, 2000, 5000, 10000}; // Hz
    for (double cfo : cfo_list) {
        int ok = 0;
        for (int t = 0; t < 10; ++t) {
            uint32_t ts = 0xCF0000u ^ static_cast<uint32_t>(t * 0x9E3779B9u);
            uint32_t il = 0xA5A5A5A5u ^ ts;
            uint8_t info[8];
            gen_info(ts, info);
            FEC_HARQ::WorkBuf wb{};
            uint8_t syms[FEC_HARQ::NSYM64]{};
            std::memset(&wb, 0, sizeof(wb));
            FEC_HARQ::Encode64_IR(info, 8, syms, il, bps, 0, wb);
            std::vector<int16_t> sI(total), sQ(total);
            for (int s = 0; s < nsym; ++s)
                walsh_enc(syms[s], nc, kAmp, &sI[s * nc], &sQ[s * nc]);
            std::vector<int16_t> refI = sI, refQ = sQ;
            // CFO 적용 (디지털 도메인)
            apply_cfo(sI.data(), sQ.data(), total, cfo,
                      static_cast<double>(PLUTO_SAMPLE_RATE));
            std::vector<int16_t> txI(PLUTO_BUF_SAMPLES), txQ(PLUTO_BUF_SAMPLES);
            fill_tx_repeated(txI.data(), txQ.data(), PLUTO_BUF_SAMPLES,
                             sI.data(), sQ.data(), total);
            std::vector<int16_t> rxI(PLUTO_BUF_SAMPLES), rxQ(PLUTO_BUF_SAMPLES);
            int nr =
                pluto_tx_then_rx(p, txI.data(), txQ.data(), PLUTO_BUF_SAMPLES,
                                 rxI.data(), rxQ.data(), PLUTO_BUF_SAMPLES);
            if (nr < total)
                continue;
            apply_digital_agc(rxI.data(), rxQ.data(), nr);
            apply_phase_correction(rxI.data(), rxQ.data(), nr);
            int ss = find_signal_start(rxI.data(), rxQ.data(), nr, refI.data(),
                                       refQ.data(), total);
            if (ss + total > nr)
                ss = 0;
            std::memset(&wb, 0, sizeof(wb));
            FEC_HARQ::IR_RxState ir{};
            FEC_HARQ::IR_Init(ir);
            uint8_t out[8]{};
            int olen = 0;
            bool dec = FEC_HARQ::Decode64_IR(&rxI[ss], &rxQ[ss], nsym, nc, bps,
                                             il, 0, ir, out, &olen, wb);
            if (dec && olen == 8 && std::memcmp(out, info, 8) == 0)
                ok++;
        }
        std::printf("  CFO=%+6.0fHz %d/10\n", cfo, ok);
    }
}
// ════════════════════════════════════════════════════════════
//  T14: 타이밍 드리프트 / 클럭 불일치
// ════════════════════════════════════════════════════════════
static void test_T14(PlutoCtx &p) {
    std::printf("\n══ T14: 타이밍 드리프트 ══\n");
    // 장시간 연속 수신 대신 심볼 타이밍 드리프트를 디지털 리샘플링으로 시뮬
    // ppm_offset: 클럭 편차(ppm) → 리샘플링 비율 = 1 + ppm*1e-6
    const int bps = 4, nc = 64;
    const int nsym = FEC_HARQ::nsym_for_bps(bps);
    const int total = nsym * nc;
    const double ppm_list[] = {1, 5, 10, 20, 50};
    for (double ppm : ppm_list) {
        int ok = 0;
        for (int t = 0; t < 10; ++t) {
            uint32_t ts = 0xD4F700u ^ static_cast<uint32_t>(t * 0x9E3779B9u);
            uint32_t il = 0xA5A5A5A5u ^ ts;
            uint8_t info[8];
            gen_info(ts, info);
            FEC_HARQ::WorkBuf wb{};
            uint8_t syms[FEC_HARQ::NSYM64]{};
            std::memset(&wb, 0, sizeof(wb));
            FEC_HARQ::Encode64_IR(info, 8, syms, il, bps, 0, wb);
            std::vector<int16_t> sI(total), sQ(total);
            for (int s = 0; s < nsym; ++s)
                walsh_enc(syms[s], nc, kAmp, &sI[s * nc], &sQ[s * nc]);
            std::vector<int16_t> refI = sI, refQ = sQ;
            // 리샘플링: rate = 1 + ppm*1e-6, 선형 보간
            double rate = 1.0 + ppm * 1e-6;
            int drift_len = static_cast<int>(total * rate) + 2;
            std::vector<int16_t> dI(drift_len), dQ(drift_len);
            for (int i = 0; i < drift_len; ++i) {
                double src = i / rate;
                int idx = static_cast<int>(src);
                double frac = src - idx;
                if (idx + 1 < total) {
                    dI[i] = static_cast<int16_t>(std::lround(
                        sI[idx] * (1.0 - frac) + sI[idx + 1] * frac));
                    dQ[i] = static_cast<int16_t>(std::lround(
                        sQ[idx] * (1.0 - frac) + sQ[idx + 1] * frac));
                } else if (idx < total) {
                    dI[i] = sI[idx];
                    dQ[i] = sQ[idx];
                } else {
                    dI[i] = 0;
                    dQ[i] = 0;
                }
            }
            // TX: 드리프트된 신호 (최대 total 칩만)
            int tx_len = (drift_len < total + 64) ? drift_len : total + 64;
            std::vector<int16_t> txI(PLUTO_BUF_SAMPLES), txQ(PLUTO_BUF_SAMPLES);
            fill_tx_repeated(txI.data(), txQ.data(), PLUTO_BUF_SAMPLES,
                             dI.data(), dQ.data(), tx_len);
            std::vector<int16_t> rxI(PLUTO_BUF_SAMPLES), rxQ(PLUTO_BUF_SAMPLES);
            int nr =
                pluto_tx_then_rx(p, txI.data(), txQ.data(), PLUTO_BUF_SAMPLES,
                                 rxI.data(), rxQ.data(), PLUTO_BUF_SAMPLES);
            if (nr < total)
                continue;
            apply_digital_agc(rxI.data(), rxQ.data(), nr);
            apply_phase_correction(rxI.data(), rxQ.data(), nr);
            int ss = find_signal_start(rxI.data(), rxQ.data(), nr, refI.data(),
                                       refQ.data(), total);
            if (ss + total > nr)
                ss = 0;
            std::memset(&wb, 0, sizeof(wb));
            FEC_HARQ::IR_RxState ir{};
            FEC_HARQ::IR_Init(ir);
            uint8_t out[8]{};
            int olen = 0;
            bool dec = FEC_HARQ::Decode64_IR(&rxI[ss], &rxQ[ss], nsym, nc, bps,
                                             il, 0, ir, out, &olen, wb);
            if (dec && olen == 8 && std::memcmp(out, info, 8) == 0)
                ok++;
        }
        std::printf("  drift=%+4.0fppm %d/10\n", ppm, ok);
    }
}
// ════════════════════════════════════════════════════════════
//  T15: 패킷 연속 전송 스트레스 (200 왕복)
// ════════════════════════════════════════════════════════════
static void test_T15(PlutoCtx &p) {
    std::printf("\n══ T15: 스트레스 테스트 (200 패킷) ══\n");
    static constexpr int STRESS_COUNT = 200;
    const int bps = 4, nc = 64;
    int ok = 0;
    int state_reset_count = 0;
    auto t0 = std::chrono::steady_clock::now();
    // 단일 디스패처 인스턴스를 재사용 — 상태 머신 누수/stuck 검증
    HTS_V400_Dispatcher disp;
    disp.Set_Seed(0x57DE5515u);
    disp.Set_IR_Mode(true);
    disp.Set_CW_Cancel(true);
    disp.Set_AJC_Enabled(true);
    disp.Set_SoftClip_Policy(SoftClipPolicy::ALWAYS);
    disp.Set_Packet_Callback(on_pkt);
    disp.Set_Lab_BPS64(bps);
    disp.Set_Lab_IQ_Mode_Jam_Harness();
    for (int t = 0; t < STRESS_COUNT; ++t) {
        uint32_t ts = 0x515000u ^ static_cast<uint32_t>(t * 0x9E3779B9u);
        uint32_t il = 0xA5A5A5A5u ^ ts;
        uint8_t info[8];
        gen_info(ts, info);
        // 매 패킷마다 Reset → 연속 전송 후 상태 정상 복귀 확인
        disp.Reset();
        disp.Set_Seed(il);
        disp.Set_IR_Mode(true);
        disp.Set_CW_Cancel(true);
        disp.Set_AJC_Enabled(true);
        disp.Set_SoftClip_Policy(SoftClipPolicy::ALWAYS);
        disp.Set_Packet_Callback(on_pkt);
        disp.Set_Lab_BPS64(bps);
        disp.Set_Lab_IQ_Mode_Jam_Harness();
        // 상태 확인: WAIT_SYNC 복귀 검증
        if (static_cast<int>(disp.Get_Phase()) != 0) {
            state_reset_count++;
        }
        FEC_HARQ::IR_RxState ir{};
        FEC_HARQ::IR_Init(ir);
        FEC_HARQ::WorkBuf wb{};
        uint8_t out[8]{};
        int olen = 0;
        bool dec = fec_txrx_decode(p, info, il, bps, nc, 0, ir, wb, out, &olen);
        if (dec)
            ok++;
        if ((t + 1) % 50 == 0) {
            auto tn = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(tn - t0).count();
            std::printf("  %d/%d (ok=%d) %.1fs\n", t + 1, STRESS_COUNT, ok,
                        elapsed);
        }
    }
    auto t1 = std::chrono::steady_clock::now();
    double sec = std::chrono::duration<double>(t1 - t0).count();
    std::printf("  T15: %d/%d %.1fs state_reset_anomaly=%d\n", ok, STRESS_COUNT,
                sec, state_reset_count);
}
// ════════════════════════════════════════════════════════════
//  T16: 전력 레벨 변동 (근/원거리 문제)
// ════════════════════════════════════════════════════════════
static void test_T16(PlutoCtx &p) {
    std::printf("\n══ T16: 전력 레벨 스윕 ══\n");
    // Pluto TX gain 범위: -89 ~ -40 (본 테스트에서는 -80 ~ -40)
    const long long gains[] = {-80, -75, -70, -65, -60, -55, -50, -45, -40};
    const int bps = 4, nc = 64;
    for (long long g : gains) {
        pluto_set_tx_gain(p, g);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        int ok = 0;
        for (int t = 0; t < 10; ++t) {
            uint32_t ts = 0xB04000u ^ static_cast<uint32_t>(t * 0x9E3779B9u);
            uint8_t info[8];
            gen_info(ts, info);
            FEC_HARQ::IR_RxState ir{};
            FEC_HARQ::IR_Init(ir);
            FEC_HARQ::WorkBuf wb{};
            uint8_t out[8]{};
            int olen = 0;
            if (fec_txrx_decode(p, info, 0xA5A5A5A5u ^ ts, bps, nc, 0, ir, wb,
                                out, &olen))
                ok++;
        }
        std::printf("  TX=%3lld dB  %d/10\n", g, ok);
    }
    // 원래 게인 복원
    pluto_set_tx_gain(p, -40);
}
// ════════════════════════════════════════════════════════════
//  T17: FHSS + Partial-Band 재밍 결합
// ════════════════════════════════════════════════════════════
static void test_T17(PlutoCtx &p) {
    std::printf("\n══ T17: FHSS + Partial-Band 재밍 ══\n");
    // FHSS 도약은 단일 Pluto 루프백에서는 실제 주파수 도약 불가
    // → 디지털 도메인: FHSS hop을 시드 기반으로 시뮬, partial-band 재밍 적용
    const int bps = 3, nc = 64;
    const int nsym = FEC_HARQ::nsym_for_bps(bps);
    const int total = nsym * nc;
    const double band_fracs[] = {0.10, 0.20, 0.30, 0.50};
    const double js_db = 25.0;
    for (double frac : band_fracs) {
        int ok = 0;
        double ar = 0;
        for (int t = 0; t < 10; ++t) {
            uint32_t ts = 0xF455017u ^ static_cast<uint32_t>(t * 0x9E3779B9u);
            uint32_t il = 0xA5A5A5A5u ^ ts;
            uint8_t info[8];
            gen_info(ts, info);
            HTS_V400_Dispatcher disp;
            disp.Set_Seed(il);
            disp.Set_IR_Mode(true);
            disp.Set_CW_Cancel(true);
            disp.Set_AJC_Enabled(true);
            disp.Set_SoftClip_Policy(SoftClipPolicy::ALWAYS);
            disp.Set_Packet_Callback(on_pkt);
            disp.Set_Lab_BPS64(bps);
            disp.Set_Lab_IQ_Mode_Jam_Harness();
            bool dec = false;
            int rnd = 0;
            for (int rv = 0; rv < 32 && !dec; ++rv) {
                g_last = DecodedPacket{};
                FEC_HARQ::WorkBuf wb{};
                uint8_t syms[FEC_HARQ::NSYM64]{};
                std::memset(&wb, 0, sizeof(wb));
                FEC_HARQ::Encode64_IR(info, 8, syms, il, bps, rv & 3, wb);
                std::vector<int16_t> sI(total), sQ(total);
                for (int s = 0; s < nsym; ++s)
                    walsh_enc(syms[s], nc, kAmp, &sI[s * nc], &sQ[s * nc]);
                std::vector<int16_t> refI = sI, refQ = sQ;
                std::mt19937 rng(ts ^ static_cast<uint32_t>(rv * 0x85EBCA6Bu));
                add_partial_band(sI.data(), sQ.data(), total, js_db, frac, rng);
                std::vector<int16_t> txI(PLUTO_BUF_SAMPLES),
                    txQ(PLUTO_BUF_SAMPLES);
                fill_tx_repeated(txI.data(), txQ.data(), PLUTO_BUF_SAMPLES,
                                 sI.data(), sQ.data(), total);
                std::vector<int16_t> rxI(PLUTO_BUF_SAMPLES),
                    rxQ(PLUTO_BUF_SAMPLES);
                int nr = pluto_tx_then_rx(p, txI.data(), txQ.data(),
                                          PLUTO_BUF_SAMPLES, rxI.data(),
                                          rxQ.data(), PLUTO_BUF_SAMPLES);
                if (nr <= 0)
                    break;
                apply_digital_agc(rxI.data(), rxQ.data(), nr);
                apply_phase_correction(rxI.data(), rxQ.data(), nr);
                int ss = find_signal_start(rxI.data(), rxQ.data(), nr,
                                           refI.data(), refQ.data(), total);
                if (ss + total > nr)
                    ss = 0;
                if (rv == 0)
                    disp.Inject_Payload_Phase(PayloadMode::DATA, bps);
                for (int i = 0; i < total; ++i) {
                    if (rv == 0)
                        disp.Feed_Chip(rxI[ss + i], rxQ[ss + i]);
                    else
                        disp.Feed_Retx_Chip(rxI[ss + i], rxQ[ss + i]);
                }
                dec = (g_last.success_mask == DecodedPacket::DECODE_MASK_OK);
                rnd = rv + 1;
                if (dec && (g_last.data_len != 8 ||
                            std::memcmp(g_last.data, info, 8) != 0))
                    dec = false;
            }
            if (dec) {
                ok++;
                ar += rnd;
            }
        }
        if (ok > 0)
            ar /= ok;
        std::printf("  band=%.0f%% J/S=%.0fdB %d/10 avg %.1fR\n", frac * 100.0,
                    js_db, ok, ar);
    }
}
// ════════════════════════════════════════════════════════════
//  T18: DWT CYCCNT 실행시간 측정 (PC 프록시)
// ════════════════════════════════════════════════════════════
static void test_T18(PlutoCtx & /*p*/) {
    std::printf("\n══ T18: 실행시간 측정 (PC steady_clock → Cortex-M4 168MHz "
                "환산) ══\n");
    // PC에서 steady_clock으로 Decode64_IR 1회 처리 시간 측정
    // Cortex-M4 환산: 경험적으로 PC(x64) 대비 ~8~15배 느림 (ISA/파이프라인
    // 차이) 양산 WCET 확인은 실칩 DWT CYCCNT 필수 — 본 테스트는 프록시 지표
    static constexpr int MEASURE_ROUNDS = 50;
    static constexpr double M4_FACTOR = 12.0; // PC→M4 보수적 추정 계수
    static constexpr double M4_CLOCK_HZ = 168e6;
    const int bps = 4, nc = 64;
    const int nsym = FEC_HARQ::nsym_for_bps(bps);
    const int total = nsym * nc;
    double sum_us = 0;
    double max_us = 0;
    for (int t = 0; t < MEASURE_ROUNDS; ++t) {
        uint32_t ts = 0xCE70018u ^ static_cast<uint32_t>(t * 0x9E3779B9u);
        uint32_t il = 0xA5A5A5A5u ^ ts;
        uint8_t info[8];
        gen_info(ts, info);
        FEC_HARQ::WorkBuf wb{};
        uint8_t syms[FEC_HARQ::NSYM64]{};
        std::memset(&wb, 0, sizeof(wb));
        FEC_HARQ::Encode64_IR(info, 8, syms, il, bps, 0, wb);
        // 합성 수신 IQ (클린, 재밍 없음)
        std::vector<int16_t> sI(total), sQ(total);
        for (int s = 0; s < nsym; ++s)
            walsh_enc(syms[s], nc, kAmp, &sI[s * nc], &sQ[s * nc]);
        // Decode64_IR 실행 시간 측정
        FEC_HARQ::IR_RxState ir{};
        FEC_HARQ::IR_Init(ir);
        uint8_t out[8]{};
        int olen = 0;
        std::memset(&wb, 0, sizeof(wb));
        auto t0 = std::chrono::steady_clock::now();
        FEC_HARQ::Decode64_IR(sI.data(), sQ.data(), nsym, nc, bps, il, 0, ir,
                              out, &olen, wb);
        auto t1 = std::chrono::steady_clock::now();
        double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
        sum_us += us;
        if (us > max_us)
            max_us = us;
    }
    double avg_us = sum_us / MEASURE_ROUNDS;
    // M4 환산
    double m4_avg_us = avg_us * M4_FACTOR;
    double m4_max_us = max_us * M4_FACTOR;
    double m4_avg_cycles = m4_avg_us * (M4_CLOCK_HZ / 1e6);
    double m4_max_cycles = m4_max_us * (M4_CLOCK_HZ / 1e6);
    std::printf("  PC Decode64_IR: avg=%.1fus max=%.1fus (%d회)\n", avg_us,
                max_us, MEASURE_ROUNDS);
    std::printf("  M4 환산(x%.0f): avg=%.0fus(%.0f cyc) max=%.0fus(%.0f cyc)\n",
                M4_FACTOR, m4_avg_us, m4_avg_cycles, m4_max_us, m4_max_cycles);
    // 실시간 마진: 64칩 1심볼 @ PLUTO_SAMPLE_RATE (1 MSPS → 64us)
    double sym_time_us = 64.0 / (static_cast<double>(PLUTO_SAMPLE_RATE) / 1e6);
    double frame_time_us = sym_time_us * nsym;
    std::printf("  프레임 시간: %.0fus, M4 마진: %.1f%%\n", frame_time_us,
                (1.0 - m4_max_us / frame_time_us) * 100.0);
}
// ════════════════════════════════════════════════════════════
//  T19: AMI 무결성 테스트 (DLMS/COSEM 프레임 모의)
// ════════════════════════════════════════════════════════════
static void test_T19(PlutoCtx &p) {
    std::printf("\n══ T19: AMI 무결성 ══\n");
    // DLMS/COSEM 프레임 구조 모의: 8바이트 INFO = [TAG(1) | LEN(1) | DATA(4) |
    // CRC16(2)] CRC16은 FEC_HARQ::CRC16 사용 — 상위 레이어 무결성 검증
    int ok = 0;
    for (int t = 0; t < 20; ++t) {
        uint32_t ts = 0xA010019u ^ static_cast<uint32_t>(t * 0x9E3779B9u);
        // DLMS-like 프레임 구성
        uint8_t frame[8] = {};
        frame[0] = 0x0F; // TAG: Get-Response-Normal
        frame[1] = 0x04; // LEN: 4바이트 데이터
        // DATA: 전력 계량값 (4바이트 BCD)
        uint32_t meter_val = ts & 0x00FFFFFFu;
        frame[2] = static_cast<uint8_t>((meter_val >> 24) & 0xFFu);
        frame[3] = static_cast<uint8_t>((meter_val >> 16) & 0xFFu);
        frame[4] = static_cast<uint8_t>((meter_val >> 8) & 0xFFu);
        frame[5] = static_cast<uint8_t>(meter_val & 0xFFu);
        // CRC16 over [TAG+LEN+DATA]
        uint16_t crc = FEC_HARQ::CRC16(frame, 6);
        frame[6] = static_cast<uint8_t>((crc >> 8) & 0xFFu);
        frame[7] = static_cast<uint8_t>(crc & 0xFFu);
        // FEC TX→RX 루프백
        FEC_HARQ::IR_RxState ir{};
        FEC_HARQ::IR_Init(ir);
        FEC_HARQ::WorkBuf wb{};
        uint8_t out[8]{};
        int olen = 0;
        bool dec = fec_txrx_decode(p, frame, 0xA5A5A5A5u ^ ts, 4, 64, 0, ir, wb,
                                   out, &olen);
        // FEC 디코딩 성공 + 상위 CRC 재검증
        bool ami_ok = false;
        if (dec && olen == 8) {
            uint16_t rx_crc = FEC_HARQ::CRC16(out, 6);
            uint16_t rx_crc_rx = (static_cast<uint16_t>(out[6]) << 8) | out[7];
            ami_ok =
                (rx_crc == rx_crc_rx) && (out[0] == 0x0F) && (out[1] == 0x04);
        }
        if (ami_ok)
            ok++;
        if ((t + 1) % 10 == 0)
            std::printf("  %d/20 (ok=%d)\n", t + 1, ok);
    }
    std::printf("  T19: %d/20\n", ok);
}
// ════════════════════════════════════════════════════════════
//  T20: S11 Chaos 테스트 (상태 머신 비정상 전이 추적)
// ════════════════════════════════════════════════════════════
static void test_T20(PlutoCtx &p) {
    std::printf("\n══ T20: S11 Chaos — 상태 머신 스트레스 ══\n");
    // 랜덤 시드·타이밍으로 디스패처에 무작위 칩 주입 후 상태 머신 정상성 확인
    // 목표: full_reset_()으로 WAIT_SYNC 복귀 보장, stuck/leak 없음
    static constexpr int CHAOS_ROUNDS = 100;
    int stuck_count = 0;
    int crash_count = 0;
    int normal_count = 0;
    std::mt19937 chaos_rng(0xCA050020u);
    for (int t = 0; t < CHAOS_ROUNDS; ++t) {
        uint32_t seed = static_cast<uint32_t>(chaos_rng());
        HTS_V400_Dispatcher disp;
        disp.Set_Seed(seed);
        disp.Set_IR_Mode((seed & 1u) != 0u);
        disp.Set_CW_Cancel((seed & 2u) != 0u);
        disp.Set_AJC_Enabled((seed & 4u) != 0u);
        disp.Set_SoftClip_Policy(static_cast<SoftClipPolicy>((seed >> 3) % 3u));
        disp.Set_Packet_Callback(on_pkt);
        disp.Set_Lab_BPS64(3 + static_cast<int>((seed >> 5) & 3u));
        disp.Set_Lab_IQ_Mode_Jam_Harness();
        // 무작위 칩 수 주입 (100~5000칩)
        int num_chips = 100 + static_cast<int>(chaos_rng() % 4901u);
        std::uniform_int_distribution<int16_t> ud(-2048, 2047);
        for (int c = 0; c < num_chips; ++c) {
            int16_t ri = ud(chaos_rng);
            int16_t rq = ud(chaos_rng);
            disp.Feed_Chip(ri, rq);
        }
        // 상태 확인: WAIT_SYNC(0), READ_HEADER(1), READ_PAYLOAD(2),
        // RF_SETTLING(3)
        RxPhase phase = disp.Get_Phase();
        int pv = static_cast<int>(phase);
        // 추가 칩 주입 후에도 리셋 가능한지 확인
        disp.Reset();
        RxPhase after_reset = disp.Get_Phase();
        if (static_cast<int>(after_reset) != 0) {
            stuck_count++;
            std::printf("  [%3d] STUCK: phase=%d after Reset\n", t, pv);
        } else {
            normal_count++;
        }
    }
    // Pluto RF 경유 chaos: 실 RF 노이즈로 동일 테스트
    std::printf("  -- RF chaos (Pluto 노이즈 직접 주입) --\n");
    int rf_stuck = 0;
    for (int t = 0; t < 20; ++t) {
        HTS_V400_Dispatcher disp;
        disp.Set_Seed(static_cast<uint32_t>(chaos_rng()));
        disp.Set_IR_Mode(true);
        disp.Set_CW_Cancel(true);
        disp.Set_AJC_Enabled(true);
        disp.Set_SoftClip_Policy(SoftClipPolicy::ALWAYS);
        disp.Set_Packet_Callback(on_pkt);
        disp.Set_Lab_BPS64(4);
        disp.Set_Lab_IQ_Mode_Jam_Harness();
        // TX=순수 노이즈 (gain 최소), RX 주입
        pluto_set_tx_gain(p, -89);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        std::vector<int16_t> rxI(PLUTO_BUF_SAMPLES), rxQ(PLUTO_BUF_SAMPLES);
        int nr = pluto_rx(p, rxI.data(), rxQ.data(), PLUTO_BUF_SAMPLES);
        for (int i = 0; i < nr; ++i)
            disp.Feed_Chip(rxI[i], rxQ[i]);
        disp.Reset();
        if (static_cast<int>(disp.Get_Phase()) != 0) {
            rf_stuck++;
            std::printf("  [RF-%2d] STUCK after noise injection\n", t);
        }
    }
    pluto_set_tx_gain(p, -40); // 복원
    std::printf("  T20: SW chaos normal=%d stuck=%d | RF chaos stuck=%d/20\n",
                normal_count, stuck_count, rf_stuck);
    std::printf("  T20: %s\n",
                (stuck_count == 0 && rf_stuck == 0) ? "PASS" : "FAIL");
}
// ════════════════════════════════════════════════════════════
int main() {
    // ── T6-SIM: Pluto 미사용, 즉시 실행 ──
    test_T6_sim();

    // ── 이하 기존 Pluto 초기화 + 테스트 ──
    std::printf("╔══════════════════════════════════════════════╗\n");
    std::printf("║  HTS B-CDMA ADALM-PLUTO V18                 ║\n");
    std::printf("║  T0~T20 Full Test Suite                      ║\n");
    std::printf("╚══════════════════════════════════════════════╝\n\n");
    PlutoCtx pluto{};
    if (!pluto_open(pluto)) {
        std::printf("[FATAL] 초기화 실패\n");
        return 1;
    }
    pluto_flush_rx(pluto);
    // ── Pluto RF 웜업 (T0 직전: PLL/RX 경로 정착) ──
    std::printf("══ RF 웜업 ══\n");
    pluto_set_tx_gain(pluto, -40);
    {
        std::vector<int16_t> wtx(PLUTO_BUF_SAMPLES), wtq(PLUTO_BUF_SAMPLES);
        std::memset(wtx.data(), 0, PLUTO_BUF_SAMPLES * sizeof(int16_t));
        std::memset(wtq.data(), 0, PLUTO_BUF_SAMPLES * sizeof(int16_t));
        std::vector<int16_t> wrx(PLUTO_BUF_SAMPLES), wrq(PLUTO_BUF_SAMPLES);
        for (int w = 0; w < 3; ++w) {
            pluto_tx_then_rx(pluto, wtx.data(), wtq.data(), PLUTO_BUF_SAMPLES,
                             wrx.data(), wrq.data(), PLUTO_BUF_SAMPLES);
            std::printf("  warmup %d/3\n", w + 1);
        }
    }
    test_T0(pluto);
    const long long tx_gain = -40;
    pluto_set_tx_gain(pluto, tx_gain);
    // 안정성 확인: TX=-40 dB에서 RX max (T0와 동일 칩 패턴), 200 미만이면
    // TX chain reset 후 재측정
    {
        std::vector<int16_t> chk_txI(PLUTO_BUF_SAMPLES),
            chk_txQ(PLUTO_BUF_SAMPLES);
        for (int i = 0; i < PLUTO_BUF_SAMPLES; ++i) {
            int16_t v = ((i / 32) & 1) ? 1000 : -1000;
            chk_txI[i] = v;
            chk_txQ[i] = v;
        }
        std::vector<int16_t> chk_rxI(PLUTO_BUF_SAMPLES),
            chk_rxQ(PLUTO_BUF_SAMPLES);
        int n_chk = pluto_tx_then_rx(pluto, chk_txI.data(), chk_txQ.data(),
                                     PLUTO_BUF_SAMPLES, chk_rxI.data(),
                                     chk_rxQ.data(), PLUTO_BUF_SAMPLES);
        int32_t rx_max = 0;
        for (int i = 0; i < n_chk; ++i) {
            int32_t a = chk_rxI[i] < 0 ? -chk_rxI[i] : chk_rxI[i];
            if (a > rx_max)
                rx_max = a;
        }
        // 1차 재측정
        if (rx_max < 200) {
            n_chk = pluto_tx_then_rx(pluto, chk_txI.data(), chk_txQ.data(),
                                     PLUTO_BUF_SAMPLES, chk_rxI.data(),
                                     chk_rxQ.data(), PLUTO_BUF_SAMPLES);
            rx_max = 0;
            for (int i = 0; i < n_chk; ++i) {
                int32_t a = chk_rxI[i] < 0 ? -chk_rxI[i] : chk_rxI[i];
                if (a > rx_max)
                    rx_max = a;
            }
        }
        // 2차: TX chain reset 후 재측정 (AD9361 DMA stuck 복구)
        if (rx_max < 200) {
            std::printf("  [WARN] RX max=%d — TX chain reset 시도\n",
                        static_cast<int>(rx_max));
            if (pluto_reset_tx_chain(pluto)) {
                pluto_set_tx_gain(pluto, tx_gain);
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                n_chk = pluto_tx_then_rx(pluto, chk_txI.data(), chk_txQ.data(),
                                         PLUTO_BUF_SAMPLES, chk_rxI.data(),
                                         chk_rxQ.data(), PLUTO_BUF_SAMPLES);
                rx_max = 0;
                for (int i = 0; i < n_chk; ++i) {
                    int32_t a = chk_rxI[i] < 0 ? -chk_rxI[i] : chk_rxI[i];
                    if (a > rx_max)
                        rx_max = a;
                }
            }
        }
        std::printf("  TX gain 고정: %lld dB, RX max 확인: %d\n", tx_gain,
                    static_cast<int>(rx_max));
    }
    // ── 기존 테스트 (T0~T9) ──
    test_T1(pluto);
    test_T2(pluto);
    test_T6(pluto);
    test_T3(pluto, tx_gain);
    test_T8(pluto);
    test_T9(pluto);
    test_T4(pluto);
    test_T5(pluto);
    test_T7(pluto);
    // ── 추가 테스트 (T10~T20) ──
    std::printf("\n╔══════════════════════════════════════════════╗\n");
    std::printf("║  T10~T20 추가 검증                            ║\n");
    std::printf("╚══════════════════════════════════════════════╝\n");
    test_T10(pluto); // CW ECCM (단일톤+다중톤)
    test_T11(pluto); // EMP/펄스 재밍
    test_T12(pluto); // 다중 사용자 CDMA
    test_T13(pluto); // CFO 내성
    test_T14(pluto); // 타이밍 드리프트
    test_T15(pluto); // 패킷 연속 스트레스 (200회)
    test_T16(pluto); // 전력 레벨 변동
    test_T17(pluto); // FHSS + Partial-Band 재밍
    test_T18(pluto); // 실행시간 측정 (WCET 프록시)
    test_T19(pluto); // AMI 무결성
    test_T20(pluto); // S11 Chaos 상태 머신
    pluto_close(pluto);
    std::printf("\n══ V18 전체 완료 (T0~T20) ══\n");
    return 0;
}
