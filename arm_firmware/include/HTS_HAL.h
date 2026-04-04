#pragma once

#include <cstddef>
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

/** DWT CYCCNT 활성화 (DEMCR.TRCENA + CTRL.CYCCNTENA). idempotent. */
void HTS_HAL_Cycles_Init(void);

/** 현재 사이클 카운트(32비트; CM4 DWT 래핑 주의). */
uint32_t HTS_HAL_Cycles_Read32(void);

/** 하드웨어 TRNG 스텁 — 양산 보드에서 레지스터 구현으로 교체. */
int HTS_HAL_TRNG_Fill(uint8_t *out, size_t len);

/** 플래시 스텁 — 지정 섹터 지우기(보드 HAL로 교체). @return 0 성공, 비0 실패 */
int HTS_HAL_Flash_Erase_Sector(uint32_t sector_index);

/** 플래시 스텁 — 프로그램(보드 HAL로 교체). @return 0 성공, 비0 실패 */
int HTS_HAL_Flash_Program(
    uint32_t flash_addr,
    const uint8_t *data,
    size_t len);

#ifdef __cplusplus
}
#endif
