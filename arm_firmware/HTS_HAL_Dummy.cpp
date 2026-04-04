#include "HTS_HAL.h"

extern "C" {

void HTS_HAL_Cycles_Init(void)
{
    volatile uint32_t *const demcr =
        reinterpret_cast<volatile uint32_t *>(0xE000EDFCu);
    volatile uint32_t *const dwt_ctrl =
        reinterpret_cast<volatile uint32_t *>(0xE0001000u);
    volatile uint32_t *const dwt_cyccnt =
        reinterpret_cast<volatile uint32_t *>(0xE0001004u);
    *demcr |= (1u << 24);   /* TRCENA */
    *dwt_ctrl |= 1u;        /* CYCCNTENA */
    *dwt_cyccnt = 0u;
}

uint32_t HTS_HAL_Cycles_Read32(void)
{
    volatile uint32_t *const dwt_cyccnt =
        reinterpret_cast<volatile uint32_t *>(0xE0001004u);
    return *dwt_cyccnt;
}

int HTS_HAL_TRNG_Fill(uint8_t *out, size_t len)
{
    if (out == nullptr || len == 0u) {
        return -1;
    }
    /* 스텁: 보드 TRNG 레지스터 미연동 — 결정론적 패턴(부트스트랩 전용) */
    uint32_t s = HTS_HAL_Cycles_Read32();
    for (size_t i = 0u; i < len; ++i) {
        s = s * 1664525u + 1013904223u;
        out[i] = static_cast<uint8_t>(s >> 24);
    }
    return 0;
}

int HTS_HAL_Flash_Erase_Sector(uint32_t sector_index)
{
    (void)sector_index;
    return -1;
}

int HTS_HAL_Flash_Program(
    uint32_t flash_addr,
    const uint8_t *data,
    size_t len)
{
    (void)flash_addr;
    (void)data;
    (void)len;
    return -1;
}

}
