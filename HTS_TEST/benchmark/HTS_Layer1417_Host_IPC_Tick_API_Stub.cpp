// =========================================================================
// HTS_Layer1417_Host_IPC_Tick_API_Stub.cpp
// PC 전용: HTS_IPC_Protocol::Tick(initialized) 본문이 HTS_API::Service_Unified_Tx_Main_Tick_Default
// 를 호출하나, Layer 14~17 호스트 스모크에서는 IPC 를 Initialize 하지 않음(MMIO 접근 회피).
// 링커가 해당 심볼을 요구하므로 본 TU 에서 최소 구현만 제공한다.
// =========================================================================
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || \
    defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#error "[HTS_FATAL] HTS_Layer1417 Host IPC Tick API Stub — PC 전용"
#endif

#include "HTS_API.h"

namespace HTS_API {

    HTS_Status Service_Unified_Tx_Main_Tick_Default(
        uint32_t /*monotonic_ms*/) noexcept
    {
        return HTS_Status::ERR_NOT_INITIALIZED;
    }

} // namespace HTS_API
