// HTS_Modbus_Gateway contract stub test
// Enable with: HTS_ENABLE_MODBUS_GATEWAY_CONTRACT_TEST

#include "HTS_Modbus_Gateway.h"

namespace {
    struct GatewayContractCase {
        ProtectedEngine::Modbus_PHY phy;
        uint8_t slave_addr;
        uint8_t func_code;
        const uint8_t* data;
        uint8_t data_len;
        uint8_t* rsp;
        uint16_t rsp_len;
        bool expect_ok;
    };
}

#if defined(HTS_ENABLE_MODBUS_GATEWAY_CONTRACT_TEST)
namespace ProtectedEngine {

// Returns 0 on pass, non-zero on first failure.
int HTS_Modbus_Gateway_Contract_Stub_Run() noexcept
{
    static_assert(MODBUS_RESPONSE_TIMEOUT > 0u, "timeout must be non-zero");
    static_assert(MODBUS_RX_POLL_MAX_ATTEMPTS > 0u, "attempt cap must be non-zero");

    HTS_Modbus_Gateway gw;
    // Intentionally keep ipc nullptr: input contract checks must reject invalid
    // requests regardless of runtime transport readiness.
    (void)gw.Initialize(nullptr);

    uint8_t rsp_buf[8] = { 0u };
    uint8_t req_buf[4] = { 0x00u, 0x10u, 0x00u, 0x01u };

    static const GatewayContractCase k_cases[] = {
        { Modbus_PHY::RS485, 0u, ModbusFC::READ_HOLDING_REGS, req_buf, 4u, rsp_buf, 8u, false }, // invalid slave
        { Modbus_PHY::RS485, 1u, 0x7Fu, req_buf, 4u, rsp_buf, 8u, false }, // unsupported FC
        { Modbus_PHY::RS485, 1u, ModbusFC::READ_HOLDING_REGS, nullptr, 1u, rsp_buf, 8u, false }, // null data with len
        { Modbus_PHY::RS485, 1u, ModbusFC::READ_HOLDING_REGS, req_buf, 4u, nullptr, 8u, false }, // null rsp with size
    };

    for (uint32_t i = 0u; i < (sizeof(k_cases) / sizeof(k_cases[0])); ++i) {
        const GatewayContractCase& c = k_cases[i];
        const uint16_t out = gw.Send_Request(
            c.phy, c.slave_addr, c.func_code, c.data, c.data_len, c.rsp, c.rsp_len);
        const bool ok = (out > 0u);
        if (ok != c.expect_ok) {
            return static_cast<int>(i + 1u);
        }
    }

    Modbus_PollItem item{};
    item.active = 1u;
    item.interval_sec = 1u;
    item.slave_addr = 0u; // invalid
    item.reg_count = 1u;
    item.func_code = ModbusFC::READ_HOLDING_REGS;
    item.phy_type = static_cast<uint8_t>(Modbus_PHY::RS485);
    if (gw.Add_Poll_Item(item) != 0xFFu) {
        return 101;
    }

    item.slave_addr = 1u;
    item.func_code = 0x7Fu; // invalid
    if (gw.Add_Poll_Item(item) != 0xFFu) {
        return 102;
    }

    return 0;
}

} // namespace ProtectedEngine
#endif

