// =========================================================================
// TensorCodec.cpp
// 3D 텐서 FEC 코덱 구현부 (Pimpl 은닉)
// Target: Cortex-A55 (CORE-X Pro 메인CPU) / Server
//
// [양산 수정 이력 — 25건]
//  BUG-01~19 (이전 세션)
//  BUG-20 [CRIT] unique_ptr + make_unique + try-catch(ctor) → placement new
//         · impl_buf_[2048] alignas(64) 정적 배치 (BUG-39)
//         · Impl = AnchorManager(값) + AnchorEncoder(값) + AnchorDecoder(값)
//           sizeof(Impl): get_impl() static_assert로 컴파일 타임 검증
//           초과 시 즉시 빌드 실패 → IMPL_BUF_SIZE를 늘릴 것
//         · 생성자: try-catch 제거 → impl_buf_ SecWipe → ::new Impl()
//           Impl 생성자(AnchorManager/Encoder/Decoder 초기화)는
//           예외를 던질 수 있으므로 noexcept + terminate 허용
//           (-fno-exceptions ARM: 예외 시 abort — 코덱 없이 동작 불가)
//         · 소멸자: = default 제거 → 명시적 p->~Impl() + SecWipe_Tensor
//  BUG-24 [CRIT] DecodePacket 터보 루프 내 힙 할당 폭풍 제거
//         · 기존: anc_slice + fixed 벡터를 루프 내부에서 매회 생성/소멸
//           → 3 iter × 6144 라인 × 2 alloc = ~36,864회 malloc/free
//         · 수정: anc_slice를 루프 외부 사전 할당 → assign() 재사용
//           → anc_slice malloc 0회 (reserve 1회 + assign 재사용)
//         · 잔여: fixed = decode() 반환값 — AnchorDecoder API 변경 없이
//           할당 제거 불가. 향후 decode(out_buf) 오버로드 추가 시 완전 제거
//  BUG-25 [HIGH] decode_inplace 전환 (PENDING-1 해결)
//         · AnchorDecoder::decode_inplace() API 추가 완료
//         · 터보 루프 내 fixed 벡터 할당 완전 제거 → fixed_buf 사전 할당
//         · 힙 할당: 36,864 → 2 (reserve 2회) = 99.99% 절감
//  BUG-39 [CRIT] Secure_Wipe_Tensor → SecureMemory::secureWipe, impl_buf_ alignas(64),
//         impl_valid_ atomic, DecodePacket turbo_iters<0 방어, provideFeedback 음수 클램프
//  BUG-40 [CRIT] vector secureWipe: size→capacity 전체 힙 블록; DecodePacket 앵커 평탄
//         count×len ↔ flat.size() 무결성 검증 후 reserve (거짓 메타 DoS 차단)
//  BUG-41 [CRIT] RAII secureWipe: ptr/size 명시 가드(H-1); u16_vec wipe: cap·data 검증
//  BUG-26 [CRIT] STM32 빌드 가드 → 프로젝트 표준 4종 매크로
//         · 기존: __arm__ 단독 → ARMCC(Keil)/IAR 누락
//         · 수정: __arm__ + __TARGET_ARCH_ARM + __TARGET_ARCH_THUMB + __ARM_ARCH
//         · _MSC_VER 예외 삭제 (MSVC는 __arm__ 미정의 → 불필요)
// =========================================================================

// [BUG-15] AnchorManager.h = 전역 네임스페이스 → ProtectedEngine 밖에서 include
#include "AnchorManager.h"

// [BUG-26] STM32 (Cortex-M) 빌드 차단 — 프로젝트 표준 4종 매크로
// 기존: __arm__ 단독 + _MSC_VER 예외 → ARMCC(Keil)/IAR/GCC Thumb-only 누락
// A55 (aarch64): __aarch64__ 정의 → 정상 통과
#if (defined(__arm__) || defined(__TARGET_ARCH_ARM) || \
     defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)) && \
    !defined(__aarch64__)
#error "[HTS_FATAL] TensorCodec는 A55/서버 전용. STM32 빌드에서 제외하십시오."
#endif

#include "TensorCodec.hpp"

// [BUG-15] AnchorEncoder/Decoder = ProtectedEngine 안
#include "AnchorEncoder.h"
#include "AnchorDecoder.h"
#include "HTS_Secure_Memory.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <new>
#include <vector>

namespace ProtectedEngine {

    namespace {

        /// @brief size_t 곱 오버플로 없이 a*b — [BUG-40] 평탄 배열 기대 크기
        static bool size_mul_ok(size_t a, size_t b, size_t* out) noexcept {
            if (out == nullptr) { return false; }
            if (a == 0u || b == 0u) { *out = 0u; return true; }
            if (a > SIZE_MAX / b) { return false; }
            *out = a * b;
            return true;
        }

        static void secure_wipe_u16_vec(std::vector<uint16_t>& v) noexcept {
            const size_t cap = v.capacity();
            if (cap == 0u) { return; }
            uint16_t* p = v.data();
            if (p == nullptr) { return; }
            SecureMemory::secureWipe(
                static_cast<void*>(p), cap * sizeof(uint16_t));
        }
    } // namespace

    // [BUG-39] 보안 소거 — SecureMemory::secureWipe 단일화 (D-2)

    // [BUG-14] RAII 보안 소거
    struct RAII_Secure_Wiper_Tensor {
        void* ptr;
        size_t size;
        RAII_Secure_Wiper_Tensor(void* p, size_t s) noexcept
            : ptr(p), size(s) {
        }
        ~RAII_Secure_Wiper_Tensor() noexcept {
            if (ptr != nullptr && size != 0u) {
                SecureMemory::secureWipe(static_cast<void*>(ptr), size);
            }
        }
        void update(void* p, size_t s) noexcept { ptr = p; size = s; }
        RAII_Secure_Wiper_Tensor(const RAII_Secure_Wiper_Tensor&) = delete;
        RAII_Secure_Wiper_Tensor& operator=(const RAII_Secure_Wiper_Tensor&) = delete;
        RAII_Secure_Wiper_Tensor(RAII_Secure_Wiper_Tensor&&) = delete;
        RAII_Secure_Wiper_Tensor& operator=(RAII_Secure_Wiper_Tensor&&) = delete;
    };

    // =====================================================================
    //  TensorPacket 소멸자 [BUG-01]
    // =====================================================================
    TensorPacket::~TensorPacket() noexcept {
        // [BUG-40] 할당 블록 전체(capacity) 소거 — size만 지우면 잉여 capacity에 잔상
        secure_wipe_u16_vec(tensor_data);
        secure_wipe_u16_vec(row_anchors_flat);
        secure_wipe_u16_vec(col_anchors_flat);
        secure_wipe_u16_vec(depth_anchors_flat);
    }

    // =====================================================================
    //  텐서 차원 상수
    // =====================================================================
    static constexpr size_t ROWS = 64u;
    static constexpr size_t COLS = 64u;
    static constexpr size_t DEPTH = 16u;

    // =====================================================================
    //  [BUG-02/15] Pimpl 구현 구조체
    //  AnchorManager는 전역 네임스페이스(::AnchorManager)
    //  AnchorEncoder/Decoder는 ProtectedEngine 네임스페이스
    // =====================================================================
    struct TensorCodec::Impl {
        ::AnchorManager anchorManager;
        AnchorEncoder   encoder;
        AnchorDecoder   decoder;

        Impl() : encoder(anchorManager), decoder(anchorManager) {}

        ~Impl() noexcept = default;

        // ── extract (크기 검증 없음 — 호출자가 차원 보장) ────────────

        static void extractRowFast(
            const std::vector<uint16_t>& tensor,
            size_t d, size_t r,
            std::vector<uint16_t>& out) noexcept {
            const size_t base = d * ROWS * COLS + r * COLS;
            for (size_t c = 0u; c < COLS; ++c) {
                out[c] = tensor[base + c];
            }
        }

        static void extractColFast(
            const std::vector<uint16_t>& tensor,
            size_t d, size_t c,
            std::vector<uint16_t>& out) noexcept {
            const size_t base_d = d * ROWS * COLS;
            for (size_t r = 0u; r < ROWS; ++r) {
                const size_t spiral_c = (c + r) % COLS;
                out[r] = tensor[base_d + r * COLS + spiral_c];
            }
        }

        static void extractDepthFast(
            const std::vector<uint16_t>& tensor,
            size_t r, size_t c,
            std::vector<uint16_t>& out) noexcept {
            const size_t plane_size = ROWS * COLS;
            for (size_t d = 0u; d < DEPTH; ++d) {
                const size_t spiral_r = (r + d) % ROWS;
                const size_t spiral_c = (c + d) % COLS;
                out[d] = tensor[d * plane_size + spiral_r * COLS + spiral_c];
            }
        }

        // ── insert [BUG-13] 크기 검증: 불일치 시 false 반환 ──────────

        static bool insertRowFast(
            std::vector<uint16_t>& tensor,
            size_t d, size_t r,
            const std::vector<uint16_t>& data) noexcept {
            if (data.size() != COLS) { return false; }
            const size_t base = d * ROWS * COLS + r * COLS;
            bool changed = false;
            for (size_t c = 0u; c < COLS; ++c) {
                if (tensor[base + c] != data[c]) {
                    tensor[base + c] = data[c];
                    changed = true;
                }
            }
            return changed;
        }

        static bool insertColFast(
            std::vector<uint16_t>& tensor,
            size_t d, size_t c,
            const std::vector<uint16_t>& data) noexcept {
            if (data.size() != ROWS) { return false; }
            const size_t base_d = d * ROWS * COLS;
            bool changed = false;
            for (size_t r = 0u; r < ROWS; ++r) {
                const size_t spiral_c = (c + r) % COLS;
                const size_t idx = base_d + r * COLS + spiral_c;
                if (tensor[idx] != data[r]) {
                    tensor[idx] = data[r];
                    changed = true;
                }
            }
            return changed;
        }

        static bool insertDepthFast(
            std::vector<uint16_t>& tensor,
            size_t r, size_t c,
            const std::vector<uint16_t>& data) noexcept {
            if (data.size() != DEPTH) { return false; }
            const size_t plane_size = ROWS * COLS;
            bool changed = false;
            for (size_t d = 0u; d < DEPTH; ++d) {
                const size_t spiral_r = (r + d) % ROWS;
                const size_t spiral_c = (c + d) % COLS;
                const size_t idx =
                    d * plane_size + spiral_r * COLS + spiral_c;
                if (tensor[idx] != data[d]) {
                    tensor[idx] = data[d];
                    changed = true;
                }
            }
            return changed;
        }
    };

    // =====================================================================
    //  [BUG-20] 컴파일 타임 크기·정렬 검증 + get_impl()
    //
    //  static_assert가 get_impl() 내부 → Impl 완전 정의 후 평가 안전
    //  sizeof(Impl) 초과 시 빌드 실패 → IMPL_BUF_SIZE 값을 늘릴 것
    // =====================================================================
    TensorCodec::Impl* TensorCodec::get_impl() noexcept {
        static_assert(sizeof(Impl) <= IMPL_BUF_SIZE,
            "Impl이 IMPL_BUF_SIZE(2048B)를 초과합니다 — 버퍼 크기를 늘려주세요");
        static_assert(alignof(Impl) <= IMPL_BUF_ALIGN,
            "Impl 정렬 요구가 impl_buf_(IMPL_BUF_ALIGN)을 초과합니다");
        return impl_valid_.load(std::memory_order_acquire)
            ? reinterpret_cast<Impl*>(impl_buf_)
            : nullptr;
    }

    const TensorCodec::Impl* TensorCodec::get_impl() const noexcept {
        return impl_valid_.load(std::memory_order_acquire)
            ? reinterpret_cast<const Impl*>(impl_buf_)
            : nullptr;
    }

    // =====================================================================
    //  [BUG-20] 생성자 — placement new (zero-heap)
    //
    //  기존: std::make_unique<Impl>() + try-catch
    //  수정: impl_buf_ SecWipe → ::new Impl() → impl_valid_ = true
    //
    //  Impl() 생성자(AnchorManager/Encoder/Decoder 초기화)가 예외를
    //  던질 수 있습니다. noexcept + 예외 = std::terminate — 의도적:
    //    ARM -fno-exceptions: bad_alloc → abort (코덱 없이 동작 불가)
    //    PC: std::terminate → 스택 트레이스로 원인 즉시 진단 가능
    //  기존 make_unique + try-catch도 실질적으로 동일 효과였습니다.
    // =====================================================================
    TensorCodec::TensorCodec() noexcept {
        SecureMemory::secureWipe(static_cast<void*>(impl_buf_), sizeof(impl_buf_));
        ::new (static_cast<void*>(impl_buf_)) Impl();
        impl_valid_.store(true, std::memory_order_release);
    }

    // =====================================================================
    //  [BUG-20] 소멸자 — 명시적 (= default 제거)
    // =====================================================================
    TensorCodec::~TensorCodec() noexcept {
        Impl* p = get_impl();
        impl_valid_.store(false, std::memory_order_release);
        if (p != nullptr) { p->~Impl(); }
        SecureMemory::secureWipe(static_cast<void*>(impl_buf_), sizeof(impl_buf_));
    }

    // =====================================================================
    //  EncodeChunk — [BUG-12] 평탄화 앵커 + [BUG-14] RAII 소거
    // =====================================================================
    TensorPacket TensorCodec::EncodeChunk(
        const std::vector<char>& buffer,
        size_t offset, size_t total_size) noexcept
    {
        TensorPacket pkt;
        Impl* p = get_impl();
        if (p == nullptr) { return pkt; }

        // [BUG-05] OOB 방어
        if (buffer.empty() || offset >= buffer.size() ||
            total_size > buffer.size() || offset >= total_size) {
            return pkt;
        }

        // [BUG-21] try-catch 삭제 (-fno-exceptions)
        auto& impl = *p;
        const size_t chunk_size = DEPTH * ROWS * COLS;
        const size_t current_chunk =
            std::min<size_t>(chunk_size, total_size - offset);

        if (offset + current_chunk > buffer.size()) { return pkt; }

        pkt.valid_bytes = current_chunk;
        pkt.is_last = (offset + current_chunk >= total_size);
        pkt.tensor_data.assign(chunk_size, 0u);

        for (size_t i = 0u; i < current_chunk; ++i) {
            pkt.tensor_data[i] = static_cast<uint16_t>(
                static_cast<unsigned char>(buffer[offset + i]));
        }

        // [BUG-14] RAII 바인딩 — reserve 직후 즉시
        std::vector<uint16_t> temp_line;
        temp_line.reserve(std::max({ ROWS, COLS, DEPTH }));
        RAII_Secure_Wiper_Tensor wipe_temp(
            temp_line.data(),
            temp_line.capacity() * sizeof(uint16_t));

        // ── [BUG-12] 평탄화 앵커 생성 ───────────────────────────
        pkt.row_anchor_count = DEPTH * ROWS;  // 1024
        pkt.col_anchor_count = DEPTH * COLS;  // 1024
        pkt.depth_anchor_count = ROWS * COLS;   // 4096

        // 첫 번째 앵커로 길이 결정
        temp_line.resize(COLS);
        Impl::extractRowFast(pkt.tensor_data, 0u, 0u, temp_line);
        auto first_anchor = impl.encoder.encode(temp_line);
        pkt.row_anchor_len = first_anchor.size();

        temp_line.resize(ROWS);
        Impl::extractColFast(pkt.tensor_data, 0u, 0u, temp_line);
        auto first_col_anc = impl.encoder.encode(temp_line);
        pkt.col_anchor_len = first_col_anc.size();

        temp_line.resize(DEPTH);
        Impl::extractDepthFast(pkt.tensor_data, 0u, 0u, temp_line);
        auto first_dep_anc = impl.encoder.encode(temp_line);
        pkt.depth_anchor_len = first_dep_anc.size();

        // 평탄 배열 사전 할당
        pkt.row_anchors_flat.resize(
            pkt.row_anchor_count * pkt.row_anchor_len, 0u);
        pkt.col_anchors_flat.resize(
            pkt.col_anchor_count * pkt.col_anchor_len, 0u);
        pkt.depth_anchors_flat.resize(
            pkt.depth_anchor_count * pkt.depth_anchor_len, 0u);

        // 행 앵커
        for (size_t i = 0u; i < pkt.row_anchor_count; ++i) {
            const size_t d = i / ROWS;
            const size_t r = i % ROWS;
            temp_line.resize(COLS);
            Impl::extractRowFast(pkt.tensor_data, d, r, temp_line);

            std::vector<uint16_t> anc;
            if (i == 0u) { anc = std::move(first_anchor); }
            else { anc = impl.encoder.encode(temp_line); }

            // [BUG-22/40] D-2: 패리티 벡터 힙 잔상 — capacity 전체
            RAII_Secure_Wiper_Tensor wipe_anc(
                anc.data(), anc.capacity() * sizeof(uint16_t));

            const size_t off = i * pkt.row_anchor_len;
            const size_t cplen = std::min(anc.size(), pkt.row_anchor_len);
            for (size_t k = 0u; k < cplen; ++k) {
                pkt.row_anchors_flat[off + k] = anc[k];
            }
        }

        // 열 앵커
        for (size_t i = 0u; i < pkt.col_anchor_count; ++i) {
            const size_t d = i / COLS;
            const size_t c = i % COLS;
            temp_line.resize(ROWS);
            Impl::extractColFast(pkt.tensor_data, d, c, temp_line);

            std::vector<uint16_t> anc;
            if (i == 0u) { anc = std::move(first_col_anc); }
            else { anc = impl.encoder.encode(temp_line); }

            // [BUG-22/40] D-2: 패리티 벡터 힙 잔상 — capacity 전체
            RAII_Secure_Wiper_Tensor wipe_anc(
                anc.data(), anc.capacity() * sizeof(uint16_t));

            const size_t off = i * pkt.col_anchor_len;
            const size_t cplen = std::min(anc.size(), pkt.col_anchor_len);
            for (size_t k = 0u; k < cplen; ++k) {
                pkt.col_anchors_flat[off + k] = anc[k];
            }
        }

        // 깊이 앵커
        for (size_t i = 0u; i < pkt.depth_anchor_count; ++i) {
            const size_t r = i / COLS;
            const size_t c = i % COLS;
            temp_line.resize(DEPTH);
            Impl::extractDepthFast(pkt.tensor_data, r, c, temp_line);

            std::vector<uint16_t> anc;
            if (i == 0u) { anc = std::move(first_dep_anc); }
            else { anc = impl.encoder.encode(temp_line); }

            // [BUG-22/40] D-2: 패리티 벡터 힙 잔상 — capacity 전체
            RAII_Secure_Wiper_Tensor wipe_anc(
                anc.data(), anc.capacity() * sizeof(uint16_t));

            const size_t off = i * pkt.depth_anchor_len;
            const size_t cplen = std::min(anc.size(), pkt.depth_anchor_len);
            for (size_t k = 0u; k < cplen; ++k) {
                pkt.depth_anchors_flat[off + k] = anc[k];
            }
        }
        // wipe_temp 소멸자: temp_line 자동 보안 소거

        return pkt;
    }

    // =====================================================================
    //  DecodePacket — [BUG-17] 크기 검증 + [BUG-18] OOB 방어
    //                 [BUG-14] RAII 소거 + [BUG-19] capacity→size
    // =====================================================================
    void TensorCodec::DecodePacket(
        TensorPacket& pkt, int turbo_iterations) noexcept
    {
        Impl* p = get_impl();
        if (p == nullptr) { return; }
        // [BUG-17] 기형 패킷 방어: 텐서 크기 불일치 → 즉시 탈출
        if (pkt.tensor_data.size() != ROWS * COLS * DEPTH) { return; }
        int turbo_iters = turbo_iterations;
        if (turbo_iters < 0) { turbo_iters = 0; }
        auto& impl = *p;

        // [BUG-40] 수신 메타 vs 평탄 벡터 무결성 — 불일치·오버플로 시 즉시 반환
        // (거짓 depth/col/row_anchor_len → max_anc_len 폭주 → bad_alloc/DoS 차단)
        {
            size_t expected = 0u;
            if (!size_mul_ok(pkt.row_anchor_count, pkt.row_anchor_len, &expected)
                || expected != pkt.row_anchors_flat.size()) {
                return;
            }
            if (!size_mul_ok(pkt.col_anchor_count, pkt.col_anchor_len, &expected)
                || expected != pkt.col_anchors_flat.size()) {
                return;
            }
            if (!size_mul_ok(pkt.depth_anchor_count, pkt.depth_anchor_len, &expected)
                || expected != pkt.depth_anchors_flat.size()) {
                return;
            }
        }

        // [BUG-21] try-catch 삭제 (-fno-exceptions)
        std::vector<uint16_t> temp_line;
        temp_line.reserve(std::max({ ROWS, COLS, DEPTH }));

        // [BUG-14/19] reserve 직후 size()가 0이므로 capacity() 기준
        RAII_Secure_Wiper_Tensor wipe_temp(
            temp_line.data(),
            temp_line.capacity() * sizeof(uint16_t));

        // [BUG-24] 앵커 슬라이스 + 복원 버퍼 사전 할당 — 루프 내 malloc 0회
        //  기존: 매 반복 anc_slice 복사 + fixed 반환 = ~36,000 malloc/free
        //  수정: reserve(max_dim) 후 assign/decode_inplace 재사용 → 2회 할당
        const size_t max_anc_len = std::max({
            pkt.depth_anchor_len,
            pkt.col_anchor_len,
            pkt.row_anchor_len });
        std::vector<uint16_t> anc_slice;
        anc_slice.reserve(max_anc_len);

        // [PENDING-1 해결] decode() 반환 벡터 → decode_inplace() 재사용 버퍼
        const size_t max_line_len = std::max({ ROWS, COLS, DEPTH });
        std::vector<uint16_t> fixed_buf;
        fixed_buf.reserve(max_line_len);

        static constexpr uint16_t ERASURE_MARKER = 0xFFFFu;
        bool avalanche_triggered = true;

        for (int iter = 0;
            iter < turbo_iters && avalanche_triggered; ++iter) {
            avalanche_triggered = false;

            // ── [단계 1] Z축 (Depth) 3D 스파이럴 ─────────────────
            for (size_t i = 0u; i < pkt.depth_anchor_count; ++i) {
                const size_t r = i / COLS;
                const size_t c = i % COLS;
                temp_line.resize(DEPTH);
                Impl::extractDepthFast(pkt.tensor_data, r, c, temp_line);

                const size_t erasures = static_cast<size_t>(
                    std::count(temp_line.begin(), temp_line.end(),
                        ERASURE_MARKER));

                if (erasures > 0u && erasures <= pkt.depth_anchor_len) {
                    const size_t off = i * pkt.depth_anchor_len;
                    // [BUG-18] 앵커 슬라이스 OOB 방어
                    if (off + pkt.depth_anchor_len >
                        pkt.depth_anchors_flat.size()) {
                        continue;
                    }
                    // [BUG-24] assign 재사용
                    anc_slice.assign(
                        pkt.depth_anchors_flat.begin() +
                        static_cast<ptrdiff_t>(off),
                        pkt.depth_anchors_flat.begin() +
                        static_cast<ptrdiff_t>(
                            off + pkt.depth_anchor_len));

                    // [PENDING-1] decode_inplace: fixed_buf capacity 재사용
                    if (impl.decoder.decode_inplace(
                        temp_line, anc_slice, fixed_buf) == ProtectedEngine::AnchorDecoder::SECURE_TRUE) {
                        if (Impl::insertDepthFast(
                            pkt.tensor_data, r, c, fixed_buf)) {
                            avalanche_triggered = true;
                        }
                    }
                }
            }

            // ── [단계 2] Y축 (Col) 2D 대각선 ─────────────────────
            for (size_t i = 0u; i < pkt.col_anchor_count; ++i) {
                const size_t d = i / COLS;
                const size_t c = i % COLS;
                temp_line.resize(ROWS);
                Impl::extractColFast(pkt.tensor_data, d, c, temp_line);

                const size_t erasures = static_cast<size_t>(
                    std::count(temp_line.begin(), temp_line.end(),
                        ERASURE_MARKER));

                if (erasures > 0u && erasures <= pkt.col_anchor_len) {
                    const size_t off = i * pkt.col_anchor_len;
                    if (off + pkt.col_anchor_len >
                        pkt.col_anchors_flat.size()) {
                        continue;
                    }
                    anc_slice.assign(
                        pkt.col_anchors_flat.begin() +
                        static_cast<ptrdiff_t>(off),
                        pkt.col_anchors_flat.begin() +
                        static_cast<ptrdiff_t>(
                            off + pkt.col_anchor_len));

                    if (impl.decoder.decode_inplace(
                        temp_line, anc_slice, fixed_buf) == ProtectedEngine::AnchorDecoder::SECURE_TRUE) {
                        if (Impl::insertColFast(
                            pkt.tensor_data, d, c, fixed_buf)) {
                            avalanche_triggered = true;
                        }
                    }
                }
            }

            // ── [단계 3] X축 (Row) 1D 선형 ───────────────────────
            for (size_t i = 0u; i < pkt.row_anchor_count; ++i) {
                const size_t d = i / ROWS;
                const size_t r = i % ROWS;
                temp_line.resize(COLS);
                Impl::extractRowFast(pkt.tensor_data, d, r, temp_line);

                const size_t erasures = static_cast<size_t>(
                    std::count(temp_line.begin(), temp_line.end(),
                        ERASURE_MARKER));

                if (erasures > 0u && erasures <= pkt.row_anchor_len) {
                    const size_t off = i * pkt.row_anchor_len;
                    if (off + pkt.row_anchor_len >
                        pkt.row_anchors_flat.size()) {
                        continue;
                    }
                    anc_slice.assign(
                        pkt.row_anchors_flat.begin() +
                        static_cast<ptrdiff_t>(off),
                        pkt.row_anchors_flat.begin() +
                        static_cast<ptrdiff_t>(
                            off + pkt.row_anchor_len));

                    if (impl.decoder.decode_inplace(
                        temp_line, anc_slice, fixed_buf) == ProtectedEngine::AnchorDecoder::SECURE_TRUE) {
                        if (Impl::insertRowFast(
                            pkt.tensor_data, d, r, fixed_buf)) {
                            avalanche_triggered = true;
                        }
                    }
                }
            }
        } // for iter

        // [BUG-24/40] 사전 할당 버퍼 보안 소거 — capacity 전체 (마지막 assign 길이 ≠ 힙 블록)
        secure_wipe_u16_vec(anc_slice);
        secure_wipe_u16_vec(fixed_buf);
        // wipe_temp 소멸자: temp_line 자동 보안 소거
    }

    // =====================================================================
    //  AMC 피드백 브릿지
    // =====================================================================
    uint8_t TensorCodec::getCurrentRatio() const noexcept {
        const Impl* p = get_impl();
        return (p != nullptr) ? p->anchorManager.getCurrentRatio() : 0u;
    }

    void TensorCodec::provideFeedback(
        int residual_errors, int loops_used) noexcept {
        Impl* p = get_impl();
        if (p != nullptr) {
            const int re = (residual_errors < 0) ? 0 : residual_errors;
            const int lu = (loops_used < 0) ? 0 : loops_used;
            p->anchorManager.autoScaleRatio(re, lu);
        }
    }

} // namespace ProtectedEngine