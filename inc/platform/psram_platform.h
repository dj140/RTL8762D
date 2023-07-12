#ifndef _PSRAM_PLATFORM_H_
#define _PSRAM_PLATFORM_H_

#include <stdint.h>
#include <stdbool.h>
#include "flash_device.h"

#ifdef __cplusplus
extern "C" {
#endif

void psram_init(void);

T_FLASH_RET psram_try_high_speed(T_FLASH_MODE bit_mode);

void psram_seq_trans_enable(uint8_t enable);

bool psram_read(uint32_t addr, uint32_t len, uint8_t *data);

bool psram_write(uint32_t addr, uint32_t len, uint8_t *data);

bool psram_user_dma_write(uint32_t addr, uint32_t len, uint8_t *data);

void psram_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* _PSRAM_PLATFORM_H_ */
