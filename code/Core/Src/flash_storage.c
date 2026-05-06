#include "flash_storage.h"
#include "stm32g0xx_hal.h"
#include <string.h>

#define FLASH_MAGIC 0xBEEF1234

typedef struct {
    uint32_t magic;
    uint32_t count;
    band_rule_t rules[MAX_RULES];
} flash_data_t;

static flash_data_t flash_buf;


// ===================== LOAD =====================
uint8_t flash_load_rules(band_rule_t *rules, uint32_t max_count)
{
    const flash_data_t *ptr = (const flash_data_t*)FLASH_STORAGE_ADDR;

    // проверка сигнатуры
    if (ptr->magic != FLASH_MAGIC) {
        return 0;
    }

    // проверка количества
    if (ptr->count == 0 || ptr->count > MAX_RULES) {
        return 0;
    }

    uint32_t count = ptr->count;

    if (count > max_count) {
        count = max_count;
    }

    memcpy(rules, ptr->rules, sizeof(band_rule_t) * count);

    return count;
}


// ===================== SAVE =====================
uint8_t flash_save_rules(const band_rule_t *rules, uint32_t count)
{
    if (count == 0 || count > MAX_RULES) {
        return 0;
    }

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase;
    uint32_t error;

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Page = (FLASH_STORAGE_ADDR - FLASH_BASE) / FLASH_PAGE_SIZE;
    erase.NbPages = 1;

    if (HAL_FLASHEx_Erase(&erase, &error) != HAL_OK) {
        HAL_FLASH_Lock();
        return 0;
    }

    // подготовка буфера
    memset(&flash_buf, 0xFF, sizeof(flash_buf));

    flash_buf.magic = FLASH_MAGIC;
    flash_buf.count = count;

    memcpy(flash_buf.rules, rules, sizeof(band_rule_t) * count);

    // запись по 8 байт
    uint64_t *src = (uint64_t*)&flash_buf;
    uint32_t addr = FLASH_STORAGE_ADDR;

    uint32_t size = sizeof(flash_buf);
    uint32_t dwords = (size + 7) / 8;

    for (uint32_t i = 0; i < dwords; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, src[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return 0;
        }
        addr += 8;
    }

    HAL_FLASH_Lock();
    return 1;
}
