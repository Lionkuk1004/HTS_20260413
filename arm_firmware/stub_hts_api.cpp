/* M4 펌웨어 1차 링크용 HTS_API.cpp 대체 스텁 — Dual_Tensor/Unified 경로 미포함 빌드 전용 */
#define HTS_API_BUILD
#include "HTS_API.h"

#include <atomic>
#include <cstddef>

namespace HTS_API {

namespace {

std::atomic<uint32_t> g_unified_tx_period_ms{kDefaultUnifiedTxServicePeriodMs};
std::atomic<uint32_t> g_unified_tx_run_count{0u};
std::atomic<uint32_t> g_unified_tx_skip_count{0u};

} // namespace

HTS_Status Initialize_Core(
    volatile uint32_t* /*hw_irq_status_reg*/,
    volatile uint32_t* /*hw_irq_clear_reg*/,
    volatile int16_t* /*hw_rx_fifo_addr*/,
    HTS_CommMedium /*target_medium*/) noexcept
{
    return HTS_Status::ERR_NOT_INITIALIZED;
}

HTS_Status Fetch_And_Heal_Rx_Payload(
    uint32_t* /*out_buffer*/,
    size_t /*required_size*/) noexcept
{
    return HTS_Status::ERR_NOT_INITIALIZED;
}

uint32_t Is_System_Operational() noexcept
{
    return SECURE_FALSE;
}

void Set_Unified_Tx_Service_Period_Ms(uint32_t period_ms) noexcept
{
    g_unified_tx_period_ms.store(period_ms, std::memory_order_relaxed);
}

uint32_t Get_Unified_Tx_Service_Period_Ms() noexcept
{
    return g_unified_tx_period_ms.load(std::memory_order_relaxed);
}

uint32_t Get_Unified_Tx_Service_Run_Count() noexcept
{
    return g_unified_tx_run_count.load(std::memory_order_relaxed);
}

uint32_t Get_Unified_Tx_Service_Skip_Count() noexcept
{
    return g_unified_tx_skip_count.load(std::memory_order_relaxed);
}

HTS_Status Service_Unified_Tx_If_Due(
    uint32_t /*monotonic_ms*/,
    uint16_t* sensor_samples,
    size_t sample_count) noexcept
{
    if (!sensor_samples || sample_count == 0u) {
        return HTS_Status::ERR_NULL_POINTER;
    }
    g_unified_tx_skip_count.fetch_add(1u, std::memory_order_relaxed);
    return HTS_Status::OK;
}

HTS_Status Service_Unified_Tx_Main_Tick_Default(uint32_t monotonic_ms) noexcept
{
    static uint16_t scratch[1024];
    return Service_Unified_Tx_If_Due(
        monotonic_ms,
        scratch,
        sizeof(scratch) / sizeof(scratch[0]));
}

HTS_Status Schedule_Unified_Tx_And_Queue(
    uint16_t* /*sensor_samples*/,
    size_t /*sample_count*/) noexcept
{
    return HTS_Status::ERR_NOT_INITIALIZED;
}

} // namespace HTS_API
