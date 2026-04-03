// =========================================================================
// HTS_Unified_Scheduler.cpp
// DMA 핑퐁 이중 버퍼 기반 통합 송신 스케줄러 구현부
// Target: STM32F407 (Cortex-M4)
//
// [보안] ARM Release: Schedule_Next_Transfer·Trigger_DMA_Hardware 진입 시 DHCSR·OPTCR(RDP)
// [제약] 힙 0, try-catch 0
// =========================================================================
#include "HTS_Unified_Scheduler.hpp"
#include "HTS_Hardware_Init.h"
#include "HTS_Secure_Memory.h"
#include <atomic>
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cstdlib>
#if defined(_MSC_VER)
#include <intrin.h>
#endif
#if defined(__GNUC__) || defined(__clang__)
typedef uint32_t __attribute__((__may_alias__)) uni_sched_u32_alias_t;
#else
typedef uint32_t uni_sched_u32_alias_t;
#endif
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || \
    defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#define HTS_PLATFORM_ARM
#include "HTS_Anti_Debug.h"
#endif
namespace ProtectedEngine {
#if !defined(HTS_UNIFIED_SCHEDULER_SKIP_PHYS_TRUST)
#if defined(HTS_ALLOW_OPEN_DEBUG) || !defined(NDEBUG)
#define HTS_UNIFIED_SCHEDULER_SKIP_PHYS_TRUST 1
#else
#define HTS_UNIFIED_SCHEDULER_SKIP_PHYS_TRUST 0
#endif
#endif
#if HTS_UNIFIED_SCHEDULER_SKIP_PHYS_TRUST == 0 && defined(HTS_PLATFORM_ARM)
    [[noreturn]] static void UnifiedScheduler_PhysicalTrust_Fault() noexcept {
        Hardware_Init_Manager::Terminal_Fault_Action();
    }
    static void UnifiedScheduler_AssertPhysicalTrustOrFault() noexcept {
        volatile const uint32_t* const dhcsr =
            reinterpret_cast<volatile const uint32_t*>(ADDR_DHCSR);
        const uint32_t d0 = *dhcsr;
#if defined(__GNUC__) || defined(__clang__)
        __asm__ __volatile__("dsb sy" ::: "memory");
#endif
        const uint32_t d1 = *dhcsr;
        if (d0 != d1) {
            UnifiedScheduler_PhysicalTrust_Fault();
        }
        if ((d0 & DHCSR_DEBUG_MASK) != 0u) {
            UnifiedScheduler_PhysicalTrust_Fault();
        }
        volatile const uint32_t* const optcr =
            reinterpret_cast<volatile const uint32_t*>(HTS_FLASH_OPTCR_ADDR);
        const uint32_t o0 = *optcr;
#if defined(__GNUC__) || defined(__clang__)
        __asm__ __volatile__("dsb sy" ::: "memory");
#endif
        const uint32_t o1 = *optcr;
        if (o0 != o1) {
            UnifiedScheduler_PhysicalTrust_Fault();
        }
        const uint32_t rdp = (o0 & HTS_RDP_OPTCR_MASK) >> 8u;
        if (rdp != HTS_RDP_EXPECTED_BYTE_VAL) {
            UnifiedScheduler_PhysicalTrust_Fault();
        }
    }
#else
    static void UnifiedScheduler_AssertPhysicalTrustOrFault() noexcept {}
#endif

    // =====================================================================
    //  DMA 레지스터 절대 주소 (보드별 커스텀 B-CDMA 모뎀)
    //  [주의] STM32F407 내장 DMA(0x4002_6000)와는 별개
    //         외부 B-CDMA 모뎀 FPGA 맵드 주소 — 보드 설계서 참조
    // =====================================================================
    static constexpr uint32_t DMA_BASE_ADDR = 0x80000000u;
    static constexpr uint32_t DMA_SRC_ADDR_REG = DMA_BASE_ADDR + 0x100u;
    static constexpr uint32_t DMA_TRANS_LEN_REG = DMA_BASE_ADDR + 0x104u;
    static constexpr uint32_t DMA_CTRL_STAT_REG = DMA_BASE_ADDR + 0x108u;
    static constexpr uint32_t DMA_DEST_ADDR_REG = DMA_BASE_ADDR + 0x10Cu;
    static constexpr uint32_t BCDMA_TX_FIFO_ADDR = 0x90000000u;

    static constexpr uint32_t DMA_START_BIT = 0x01u;  ///< bit[0] DMA 전송 시작
    //  FPGA 커스텀 DMA 컨트롤러 control_status 레지스터 비트맵:
    //    bit[0] = START (W: 전송 시작, R: 0)
    //    bit[1] = BUSY  (R: 1=전송 중, 0=IDLE)
    //  ⚠ BUSY=1 상태에서 source_address/transfer_length 쓰기 금지
    //    → FPGA 내부 낸드 게이트 꼬임 → 시스템 락업/HardFault
    //  보드별 비트맵이 다를 수 있음 — FPGA RTL 또는 보드 설계서 참조
    static constexpr uint32_t DMA_BUSY_BIT = 0x02u;   ///< bit[1] FPGA DMA 전송 중
    static constexpr uintptr_t AIRCR_ADDR = 0xE000ED0Cu;
    static constexpr uint32_t  AIRCR_VECTKEY = 0x05FA0000u;
    static constexpr uint32_t  AIRCR_SYSRST = 0x04u;

    // =====================================================================
    //  생성자 — pipeline==nullptr: AIRCR 시도 후 Terminal_Fault_Action(무한 루프 제거)
    // =====================================================================
    Unified_Scheduler::Unified_Scheduler(Dual_Tensor_Pipeline* pipeline) noexcept
        : core_pipeline(pipeline)
        , buffer_size(0)
        , ping_buffer{}
        , pong_buffer{}
        , current_dma_buffer(0)
        , packet_sequence_nonce(0)
        , dma_hw{} {

        const uint32_t bad_pipe = static_cast<uint32_t>(core_pipeline == nullptr);
        if (bad_pipe != 0u) {
#if defined(HTS_PLATFORM_ARM)
#if defined(__GNUC__) || defined(__clang__)
            __asm__ __volatile__("dsb" ::: "memory");
#endif
            volatile uint32_t* const aircr = reinterpret_cast<volatile uint32_t*>(
                reinterpret_cast<void*>(static_cast<uintptr_t>(AIRCR_ADDR)));
            *aircr = (AIRCR_VECTKEY | AIRCR_SYSRST);
#if defined(__GNUC__) || defined(__clang__)
            __asm__ __volatile__("dsb" ::: "memory");
            __asm__ __volatile__("isb" ::: "memory");
#endif
            Hardware_Init_Manager::Terminal_Fault_Action();
#else
            std::abort();
#endif
        }

        // Output buffer capacity는 ping/pong 배열 크기(MAX_DMA_FRAME)로 고정.
        // 실제 송신 길이는 generated_len(dl_len_)로만 제한한다.
        buffer_size = MAX_DMA_FRAME;

        // DMA 레지스터 매핑 — ARM에서만 유효
#if defined(HTS_PLATFORM_ARM)
        dma_hw.source_address = reinterpret_cast<volatile uint32_t*>(static_cast<uintptr_t>(DMA_SRC_ADDR_REG));
        dma_hw.transfer_length = reinterpret_cast<volatile uint32_t*>(static_cast<uintptr_t>(DMA_TRANS_LEN_REG));
        dma_hw.control_status = reinterpret_cast<volatile uint32_t*>(static_cast<uintptr_t>(DMA_CTRL_STAT_REG));
        dma_hw.dest_address = reinterpret_cast<volatile uint32_t*>(static_cast<uintptr_t>(DMA_DEST_ADDR_REG));
#else
        // PC 시뮬레이션: DMA 레지스터 = nullptr (Trigger_DMA_Hardware에서 no-op)
        dma_hw.source_address = nullptr;
        dma_hw.transfer_length = nullptr;
        dma_hw.control_status = nullptr;
        dma_hw.dest_address = nullptr;
#endif
    }

    // =====================================================================
    //
    //  소멸자 미정의 (= default 암시)
    //   → 32KB 핑퐁 버퍼에 텐서 데이터 평문 잔존
    //   → 콜드부트/힙 스캔 공격으로 직전 송신 데이터 복원 가능
    //
    //  volatile 소거 + asm clobber + release fence
    //   → BB1, Anchor_Vault 소멸자 보안 소거 표준과 통일
    // =====================================================================
    Unified_Scheduler::~Unified_Scheduler() noexcept {
        SecureMemory::secureWipe(static_cast<void*>(ping_buffer), sizeof(ping_buffer));
        SecureMemory::secureWipe(static_cast<void*>(pong_buffer), sizeof(pong_buffer));
        SecureMemory::secureWipe(static_cast<void*>(ping_buffer), sizeof(ping_buffer));
        SecureMemory::secureWipe(static_cast<void*>(pong_buffer), sizeof(pong_buffer));
        SecureMemory::secureWipe(
            static_cast<void*>(&packet_sequence_nonce), sizeof(packet_sequence_nonce));
        SecureMemory::secureWipe(static_cast<void*>(&dma_hw), sizeof(dma_hw));
        std::atomic_thread_fence(std::memory_order_release);
        current_dma_buffer.store(0, std::memory_order_release);
        buffer_size = 0u;
        core_pipeline = nullptr;
    }

    // =====================================================================
    //  Schedule_Next_Transfer — 센서 → 듀얼 텐서 → 핑퐁 버퍼 → DMA
    //
    //   relaxed — ISR/다른 컨텍스트에서 stale 값 가능
    //   acquire — store(release)와 쌍을 이루어 가시성 보장
    //         현재 ISR이 비어있어 실질 영향 없으나, 향후 ISR에서
    //         버퍼 스왑 로직 추가 시 데이터 레이스 예방
    //
    //   data_len: raw_sensor_data의 uint16_t 원소 개수
    //   buffer_size/generated_len: uint32_t 원소 개수
    //   듀얼 텐서 파이프라인이 16→32비트 패킹을 내부 수행하므로
    //   safe_len 계산은 32비트 단위 출력 기준으로 수렴함
    //   입력 data_len이 부족하면 core_pipeline이 generated_len을 축소하며,
    //   Schedule_Next_Transfer는 generated_len만큼만 ping/pong에 복사 후 DMA를 수행함
    // =====================================================================
    bool Unified_Scheduler::Schedule_Next_Transfer(
        uint16_t* raw_sensor_data, size_t data_len,
        std::atomic<bool>& abort_signal) noexcept {

        UnifiedScheduler_AssertPhysicalTrustOrFault();
        const uint32_t bad_pre =
            static_cast<uint32_t>(raw_sensor_data == nullptr)
            | static_cast<uint32_t>(core_pipeline == nullptr);
        if (bad_pre != 0u) {
            return false;
        }

        const int active_dma = current_dma_buffer.load(std::memory_order_acquire);
        const uint32_t ad_u = static_cast<uint32_t>(
            static_cast<uint32_t>(active_dma) & 0x7FFFFFFFu);
        const uint32_t ad_ok = static_cast<uint32_t>(ad_u < 2u);
        if (ad_ok == 0u) {
            return false;
        }
        const int target_cpu_buffer = 1 - static_cast<int>(ad_u);

        const bool success = core_pipeline->Execute_Dual_Processing(
            raw_sensor_data, data_len, packet_sequence_nonce++, abort_signal);
        if (!success) {
            return false;
        }

        const uint32_t* generated_data = core_pipeline->Get_Dual_Lane_Data();
        const size_t generated_len = core_pipeline->Get_Dual_Lane_Size();
        const uint32_t bad_gen =
            static_cast<uint32_t>(generated_data == nullptr)
            | static_cast<uint32_t>(generated_len == 0u);
        if (bad_gen != 0u) {
            return false;
        }

        uint32_t* const banks[2] = { ping_buffer, pong_buffer };
        uint32_t* const active_cpu_buffer =
            banks[static_cast<size_t>(static_cast<uint32_t>(target_cpu_buffer) & 1u)];

        const uint32_t take_gen = static_cast<uint32_t>(generated_len < buffer_size);
        const size_t safe_len =
            generated_len * static_cast<size_t>(take_gen)
            + buffer_size * static_cast<size_t>(1u - take_gen);
        const uint32_t copy_safe =
            static_cast<uint32_t>(safe_len <= MAX_DMA_FRAME)
            & static_cast<uint32_t>(safe_len <= buffer_size);
        const size_t copy_words = safe_len * static_cast<size_t>(copy_safe);
        if (copy_safe == 0u) {
            return false;
        }
        const size_t copy_bytes = copy_words * sizeof(uint32_t);
        std::memcpy(
            static_cast<void*>(static_cast<uni_sched_u32_alias_t*>(active_cpu_buffer)),
            static_cast<const void*>(
                static_cast<const uni_sched_u32_alias_t*>(generated_data)),
            copy_bytes);

        if (!Trigger_DMA_Hardware(active_cpu_buffer, copy_words)) {
            return false;
        }
        current_dma_buffer.store(target_cpu_buffer, std::memory_order_release);

        return true;
    }

    // =====================================================================
    //  DMA 전송 완료 ISR
    //  TODO: 향후 버퍼 스왑 알림, 세마포어 시그널 등 추가
    // =====================================================================
    void Unified_Scheduler::DMA_Transfer_Complete_ISR() noexcept {
        // 실제 환경: 다음 스왑 준비 또는 슬립 해제
    }

    // =====================================================================
    //  Trigger_DMA_Hardware — DMA 레지스터 장전 + 전송 시작
    //
    //  3단 플랫폼 분기
    //
    //  ⚠ CRITICAL: FPGA 커스텀 DMA 컨트롤러는 BUSY=1 상태에서
    //     source_address / dest_address / transfer_length 레지스터를
    //     덮어쓰면 FPGA 내부 낸드 게이트가 꼬이면서 하드웨어 락업 발생.
    //     (STM32 내장 DMA는 EN=0으로 비활성화 후 쓰기 가능하지만,
    //      외부 FPGA는 이 보호 메커니즘이 없음)
    //
    //  동작 순서 (비차단):
    //   ① BUSY 1회 샘플 — 1이면 즉시 false (스핀 금지, 상위 재시도)
    //   ② Cache_Clean_Tx → DSB — SRAM 반영 후에만 레지스터 장전
    //   ③ source/dest/length → START
    // =====================================================================
    bool Unified_Scheduler::Trigger_DMA_Hardware(
        uint32_t* buffer_ptr, size_t length) noexcept {

#if defined(HTS_PLATFORM_ARM)
        UnifiedScheduler_AssertPhysicalTrustOrFault();
        const uint32_t bad_ptr = static_cast<uint32_t>(buffer_ptr == nullptr);
        const uint32_t ok_len =
            static_cast<uint32_t>(length != 0u)
            & static_cast<uint32_t>(length <= MAX_DMA_FRAME);
        const uint32_t bad_regs =
            static_cast<uint32_t>(dma_hw.source_address == nullptr)
            | static_cast<uint32_t>(dma_hw.transfer_length == nullptr)
            | static_cast<uint32_t>(dma_hw.control_status == nullptr)
            | static_cast<uint32_t>(dma_hw.dest_address == nullptr);
        const uint32_t bad = bad_ptr | (1u - ok_len) | bad_regs;
        if (bad != 0u) {
            return false;
        }
        const size_t safe_length = length * static_cast<size_t>(ok_len);

        const uint32_t ctrl0 = *dma_hw.control_status;
        const uint32_t busy_now =
            static_cast<uint32_t>((ctrl0 & DMA_BUSY_BIT) != 0u);
        if (busy_now != 0u) {
            return false;
        }

        Hardware_Init_Manager::Cache_Clean_Tx(buffer_ptr, safe_length);
#if defined(__GNUC__) || defined(__clang__)
        __asm__ __volatile__("dsb sy" ::: "memory");
#elif defined(_MSC_VER)
        _ReadWriteBarrier();
#endif

        *dma_hw.source_address = static_cast<uint32_t>(
            reinterpret_cast<uintptr_t>(buffer_ptr));
        *dma_hw.dest_address = static_cast<uint32_t>(BCDMA_TX_FIFO_ADDR);
        *dma_hw.transfer_length = static_cast<uint32_t>(safe_length);
#if defined(__GNUC__) || defined(__clang__)
        __asm__ __volatile__("dsb sy" ::: "memory");
#elif defined(_MSC_VER)
        _ReadWriteBarrier();
#endif

        *dma_hw.control_status |= DMA_START_BIT;
        return true;
#else
        // PC 시뮬레이션: DMA 없음 — 데이터는 이미 버퍼에 준비됨 (소유권 갱신 허용)
        (void)buffer_ptr;
        (void)length;
        return true;
#endif
    }

} // namespace ProtectedEngine
