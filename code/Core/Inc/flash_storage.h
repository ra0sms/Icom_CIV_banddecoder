#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include <stdint.h>
#include "band_rules.h"

#define FLASH_STORAGE_ADDR   0x0800F800

// API
uint8_t flash_save_rules(const band_rule_t *rules, uint32_t count);
uint8_t flash_load_rules(band_rule_t *rules, uint32_t max_count);

#endif
