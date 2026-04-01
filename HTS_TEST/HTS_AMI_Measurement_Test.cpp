// =========================================================================
//  HTS_AMI_Measurement_Test.cpp
//
//  **단일 소스 TU** — 별도 .obj·HTS_LIM 링크 없이 빌드 가능:
//    cl /std:c++17 /EHsc /O2 /W3 /Fe:ami_measurement.exe HTS_AMI_Measurement_Test.cpp
//    g++ -std=c++17 -O2 -Wall -o ami_measurement HTS_AMI_Measurement_Test.cpp
//
//  B-CDMA AMI 측정·스트레스: 시나리오 A/B/C (Zero-copy 뷰, CAS 링, 악성 입력 방어)
//  new/malloc·try/catch·Mutex 금지. 링 길이는 2의 거듭제곱(마스크 &, >>).
//  © INNOViD 2026
// =========================================================================

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace {

// ---------------------------------------------------------------------------
//  D-2 호환 소거 (TU 내부 — 외부 SecureMemory 링크 불필요)
// ---------------------------------------------------------------------------
static void secure_wipe(void* p, std::size_t n) noexcept {
    if (p == nullptr || n == 0u) {
        return;
    }
    volatile std::uint8_t* q = static_cast<volatile std::uint8_t*>(p);
    for (std::size_t i = 0u; i < n; ++i) {
        q[i] = 0u;
    }
#if defined(__GNUC__) || defined(__clang__)
    __asm__ __volatile__("" : : "r"(p) : "memory");
#endif
    std::atomic_thread_fence(std::memory_order_release);
}

// ---------------------------------------------------------------------------
//  테스트 카운터
// ---------------------------------------------------------------------------
static int g_total = 0;
static int g_pass = 0;
static int g_fail = 0;

static volatile std::uint32_t g_defensive_hits = 0u;

#define SECTION(name) \
    std::printf("\n══════════════════════════════════════════\n  %s\n══════════════════════════════════════════\n", name)

#define CHECK(desc, cond) \
    do { \
        g_total++; \
        if (cond) { \
            g_pass++; \
            std::printf("  [PASS] %s\n", desc); \
        } else { \
            g_fail++; \
            std::printf("  [FAIL] %s  ← %d\n", desc, __LINE__); \
        } \
    } while (false)

#define SUMMARY() \
    std::printf("\n══════════════════════════════════════════\n" \
                "  결과: %d/%d 통과%s\n" \
                "  방어 경로 누적: %u\n" \
                "══════════════════════════════════════════\n", \
        g_pass, g_total, g_fail ? " ⛔" : " ✅", \
        static_cast<unsigned>(g_defensive_hits))

// ---------------------------------------------------------------------------
//  AMI 프레임 (고정 24B) — 빅엔디안, CRC32는 바이트 0..16
// ---------------------------------------------------------------------------
static constexpr std::size_t AMI_FRAME_BYTES = 24u;
static constexpr std::uint8_t k_magic0 = 0x41u;
static constexpr std::uint8_t k_magic1 = 0x4Du;
static constexpr std::uint8_t k_magic2 = 0x49u;
static constexpr std::uint8_t k_magic3 = 0x01u;

static constexpr std::uint8_t k_ch_electric = 0u;
static constexpr std::uint8_t k_ch_gas = 1u;
static constexpr std::uint8_t k_ch_water = 2u;

static constexpr std::uint32_t k_crc_poly = 0xEDB88320u;

static constexpr std::uint32_t k_err_ok = 0u;
static constexpr std::uint32_t k_err_null = 1u;
static constexpr std::uint32_t k_err_len = 2u;
static constexpr std::uint32_t k_err_magic = 3u;
static constexpr std::uint32_t k_err_crc = 4u;
static constexpr std::uint32_t k_err_replay = 5u;
static constexpr std::uint32_t k_err_range = 6u;

static std::uint32_t g_last_seq[3] = { 0u, 0u, 0u };

static std::uint32_t crc32_block(const std::uint8_t* data, std::size_t len,
    std::uint32_t crc) noexcept {
    if (data == nullptr) {
        return crc;
    }
    for (std::size_t i = 0u; i < len; ++i) {
        crc ^= static_cast<std::uint32_t>(data[i]);
        for (std::uint32_t bit = 0u; bit < 8u; ++bit) {
            const std::uint32_t lsb = crc & 1u;
            crc = static_cast<std::uint32_t>(crc >> 1u);
            crc ^= static_cast<std::uint32_t>(0u - lsb) & k_crc_poly;
        }
    }
    return crc;
}

static std::uint32_t read_be32(const std::uint8_t* p) noexcept {
    return (static_cast<std::uint32_t>(p[0]) << 24)
        | (static_cast<std::uint32_t>(p[1]) << 16)
        | (static_cast<std::uint32_t>(p[2]) << 8)
        | static_cast<std::uint32_t>(p[3]);
}

static void write_be32(std::uint8_t* p, std::uint32_t v) noexcept {
    p[0] = static_cast<std::uint8_t>((v >> 24) & 0xFFu);
    p[1] = static_cast<std::uint8_t>((v >> 16) & 0xFFu);
    p[2] = static_cast<std::uint8_t>((v >> 8) & 0xFFu);
    p[3] = static_cast<std::uint8_t>(v & 0xFFu);
}

struct AmiParsedView {
    const std::uint8_t* base;
    std::size_t len;
    std::uint8_t channel;
    std::uint32_t seq;
    std::uint32_t ts_ms;
    std::uint32_t value_q100;
};

static void build_frame(std::uint8_t* f, std::uint8_t ch,
    std::uint32_t seq, std::uint32_t ts_ms, std::uint32_t val_q100) noexcept {
    f[0] = k_magic0;
    f[1] = k_magic1;
    f[2] = k_magic2;
    f[3] = k_magic3;
    f[4] = ch;
    write_be32(f + 5u, seq);
    write_be32(f + 9u, ts_ms);
    write_be32(f + 13u, val_q100);
    const std::uint32_t c = crc32_block(f, 17u, 0xFFFFFFFFu) ^ 0xFFFFFFFFu;
    write_be32(f + 17u, c);
    f[21] = 0u;
    f[22] = 0u;
    f[23] = 0u;
}

/// Zero-copy: out->base 는 입력 버퍼 포인터만 보관
static std::uint32_t parse_ami_zero_copy(const std::uint8_t* p, std::size_t len,
    AmiParsedView* out) noexcept {
    if (p == nullptr || out == nullptr) {
        return k_err_null;
    }
    if (len < AMI_FRAME_BYTES) {
        return k_err_len;
    }
    std::uint32_t d = 0u;
    d |= static_cast<std::uint32_t>(p[0] ^ k_magic0);
    d |= static_cast<std::uint32_t>(p[1] ^ k_magic1);
    d |= static_cast<std::uint32_t>(p[2] ^ k_magic2);
    d |= static_cast<std::uint32_t>(p[3] ^ k_magic3);
    if (d != 0u) {
        return k_err_magic;
    }

    const std::uint8_t ch = p[4];
    const std::uint32_t seq = read_be32(p + 5u);
    const std::uint32_t ts = read_be32(p + 9u);
    const std::uint32_t val = read_be32(p + 13u);
    const std::uint32_t crc_in = read_be32(p + 17u);
    const std::uint32_t crc_comp = crc32_block(p, 17u, 0xFFFFFFFFu) ^ 0xFFFFFFFFu;
    if ((crc_comp ^ crc_in) != 0u) {
        return k_err_crc;
    }

    const std::uint32_t is_e = static_cast<std::uint32_t>(ch == k_ch_electric);
    const std::uint32_t is_g = static_cast<std::uint32_t>(ch == k_ch_gas);
    const std::uint32_t is_w = static_cast<std::uint32_t>(ch == k_ch_water);
    const std::uint32_t ch_ok = is_e | is_g | is_w;
    if (ch_ok == 0u) {
        return k_err_range;
    }
    const std::uint32_t ch_idx = is_g | (is_w << 1u);

    const std::uint32_t prev = g_last_seq[ch_idx];
    const std::uint32_t replay = static_cast<std::uint32_t>(
        (seq == prev) & (prev != 0u));
    if (replay != 0u) {
        return k_err_replay;
    }

    const std::uint32_t neg = val & 0x80000000u;
    if (neg != 0u) {
        return k_err_range;
    }
    const std::uint32_t val_masked = val & 0x7FFFFFFFu;

    g_last_seq[ch_idx] = seq;

    out->base = p;
    out->len = AMI_FRAME_BYTES;
    out->channel = ch;
    out->seq = seq;
    out->ts_ms = ts;
    out->value_q100 = val_masked;
    return k_err_ok;
}

// ---------------------------------------------------------------------------
//  Lock-free 링: MP 프로듀서(CAS tail) + 단일 컨슈머 — Mutex 없음
// ---------------------------------------------------------------------------
static constexpr std::size_t RING_CAP = 64u;
static constexpr std::size_t RING_MASK = RING_CAP - 1u;
static constexpr std::size_t SLOT_BYTES = 64u;

alignas(64) static std::uint8_t g_ring[RING_CAP][SLOT_BYTES];
static std::atomic<std::size_t> g_head{0u};
static std::atomic<std::size_t> g_tail{0u};

static bool ring_try_push_mp(const std::uint8_t* src, std::size_t len) noexcept {
    if (src == nullptr || len == 0u || len > AMI_FRAME_BYTES) {
        return false;
    }
    std::size_t t = g_tail.load(std::memory_order_relaxed);
    for (;;) {
        const std::size_t h = g_head.load(std::memory_order_acquire);
        const std::size_t next = (t + 1u) & RING_MASK;
        const std::uint32_t full = static_cast<std::uint32_t>(next == h);
        if (full != 0u) {
            return false;
        }
        if (!g_tail.compare_exchange_weak(t, next,
                std::memory_order_acq_rel,
                std::memory_order_relaxed)) {
            continue;
        }
        std::uint8_t* const dst = reinterpret_cast<std::uint8_t*>(g_ring[t & RING_MASK]);
        for (std::size_t i = 0u; i < len; ++i) {
            dst[i] = src[i];
        }
        for (std::size_t i = len; i < SLOT_BYTES; ++i) {
            dst[i] = 0u;
        }
        return true;
    }
}

static bool ring_try_pop_sc(std::uint8_t* dst) noexcept {
    if (dst == nullptr) {
        return false;
    }
    const std::size_t h = g_head.load(std::memory_order_relaxed);
    const std::size_t t = g_tail.load(std::memory_order_acquire);
    if (h == t) {
        return false;
    }
    const std::uint8_t* const src = reinterpret_cast<const std::uint8_t*>(g_ring[h & RING_MASK]);
    for (std::size_t i = 0u; i < AMI_FRAME_BYTES; ++i) {
        dst[i] = src[i];
    }
    g_head.store((h + 1u) & RING_MASK, std::memory_order_release);
    return true;
}

// ---------------------------------------------------------------------------
//  시나리오 A — 이종 3종 동시 유입·Zero-copy
// ---------------------------------------------------------------------------
static void scenario_a() noexcept {
    SECTION("A — 전력·가스·수도 이종 페이로드 (Zero-copy)");

    alignas(8) static std::uint8_t s_elec[AMI_FRAME_BYTES];
    alignas(8) static std::uint8_t s_gas[AMI_FRAME_BYTES];
    alignas(8) static std::uint8_t s_water[AMI_FRAME_BYTES];

    g_last_seq[0] = 0u;
    g_last_seq[1] = 0u;
    g_last_seq[2] = 0u;

    build_frame(s_elec, k_ch_electric, 1u, 100u, 500000u);
    build_frame(s_gas, k_ch_gas, 1u, 5000u, 120u);
    build_frame(s_water, k_ch_water, 1u, 10000u, 33u);

    AmiParsedView ve = {};
    AmiParsedView vg = {};
    AmiParsedView vw = {};

    const std::uint32_t re = parse_ami_zero_copy(s_elec, AMI_FRAME_BYTES, &ve);
    const std::uint32_t rg = parse_ami_zero_copy(s_gas, AMI_FRAME_BYTES, &vg);
    const std::uint32_t rw = parse_ami_zero_copy(s_water, AMI_FRAME_BYTES, &vw);

    CHECK("전력 OK", re == k_err_ok);
    CHECK("가스 OK", rg == k_err_ok);
    CHECK("수도 OK", rw == k_err_ok);
    CHECK("ZC 전력", (ve.base == s_elec) && (ve.len == AMI_FRAME_BYTES));
    CHECK("ZC 가스", (vg.base == s_gas) && (vg.len == AMI_FRAME_BYTES));
    CHECK("ZC 수도", (vw.base == s_water) && (vw.len == AMI_FRAME_BYTES));
    CHECK("스케줄(전력→가스→수도 ts)", (ve.ts_ms < vg.ts_ms) && (vg.ts_ms < vw.ts_ms));

    secure_wipe(reinterpret_cast<void*>(s_elec), AMI_FRAME_BYTES);
    secure_wipe(reinterpret_cast<void*>(s_gas), AMI_FRAME_BYTES);
    secure_wipe(reinterpret_cast<void*>(s_water), AMI_FRAME_BYTES);
}

// ---------------------------------------------------------------------------
//  시나리오 B — 버스트·중첩(많은 노드 순차 푸시)·오버플로 한계
// ---------------------------------------------------------------------------
static void scenario_b() noexcept {
    SECTION("B — B-CDMA 혼잡 시뮬 (CAS 링·오버플로)");

    g_head.store(0u, std::memory_order_release);
    g_tail.store(0u, std::memory_order_release);

    alignas(8) std::uint8_t frame[AMI_FRAME_BYTES];
    std::uint32_t drops = 0u;
    constexpr std::uint32_t k_burst = 400u;

    for (std::uint32_t n = 0u; n < k_burst; ++n) {
        const std::uint8_t nid = static_cast<std::uint8_t>(n & 3u);
        build_frame(frame, k_ch_electric, n, 1000u + n, 100u + n);
        frame[21] = nid;
        const bool ok = ring_try_push_mp(frame, AMI_FRAME_BYTES);
        drops += static_cast<std::uint32_t>(ok ? 0u : 1u);
    }

    CHECK("한계 초과 시 드롭 발생", drops > 0u);

    std::uint32_t drained = 0u;
    alignas(8) std::uint8_t popb[AMI_FRAME_BYTES];
    while (ring_try_pop_sc(popb)) {
        drained++;
    }
    CHECK("드레인>0", drained > 0u);

    secure_wipe(reinterpret_cast<void*>(frame), sizeof(frame));
    secure_wipe(reinterpret_cast<void*>(popb), sizeof(popb));
}

// ---------------------------------------------------------------------------
//  시나리오 C — CRC·절단·Replay·비정상 값
// ---------------------------------------------------------------------------
static void scenario_c() noexcept {
    SECTION("C — 악성/노이즈 방어");

    alignas(8) static std::uint8_t good[AMI_FRAME_BYTES];
    alignas(8) static std::uint8_t bad_crc[AMI_FRAME_BYTES];
    alignas(8) static std::uint8_t shortbuf[8];

    g_last_seq[0] = 0u;
    build_frame(good, k_ch_electric, 7u, 1u, 100u);
    std::memcpy(bad_crc, good, AMI_FRAME_BYTES);
    bad_crc[19] = static_cast<std::uint8_t>(bad_crc[19] ^ 0xFFu);

    AmiParsedView v = {};

    std::uint32_t r = parse_ami_zero_copy(good, AMI_FRAME_BYTES, &v);
    CHECK("정상", r == k_err_ok);

    r = parse_ami_zero_copy(bad_crc, AMI_FRAME_BYTES, &v);
    CHECK("CRC 실패", r == k_err_crc);
    g_defensive_hits++;

    r = parse_ami_zero_copy(good, 8u, &v);
    CHECK("절단", r == k_err_len);
    g_defensive_hits++;

    r = parse_ami_zero_copy(nullptr, AMI_FRAME_BYTES, &v);
    CHECK("nullptr", r == k_err_null);
    g_defensive_hits++;

    std::memset(shortbuf, 0, sizeof(shortbuf));
    r = parse_ami_zero_copy(shortbuf, sizeof(shortbuf), &v);
    CHECK("짧은 버퍼", r == k_err_len);
    g_defensive_hits++;

    g_last_seq[0] = 5u;
    build_frame(good, k_ch_electric, 5u, 2u, 50u);
    r = parse_ami_zero_copy(good, AMI_FRAME_BYTES, &v);
    CHECK("Replay", r == k_err_replay);
    g_defensive_hits++;

    build_frame(good, k_ch_electric, 6u, 3u, 0xFFFFFFFFu);
    r = parse_ami_zero_copy(good, AMI_FRAME_BYTES, &v);
    CHECK("부호비트(비정상) 거부", r == k_err_range);
    g_defensive_hits++;

    secure_wipe(reinterpret_cast<void*>(good), sizeof(good));
    secure_wipe(reinterpret_cast<void*>(bad_crc), sizeof(bad_crc));
    secure_wipe(reinterpret_cast<void*>(shortbuf), sizeof(shortbuf));
}

} // namespace

int main() {
    std::printf("═══════════════════════════════════════════════\n");
    std::printf("  HTS AMI Measurement — **단일 cpp** (외부 .obj 불필요)\n");
    std::printf("═══════════════════════════════════════════════\n");

    scenario_a();
    scenario_b();
    scenario_c();

    SUMMARY();
    return g_fail ? 1 : 0;
}
