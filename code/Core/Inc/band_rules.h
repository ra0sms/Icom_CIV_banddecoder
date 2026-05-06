#ifndef BAND_RULES_H
#define BAND_RULES_H

#include <stdint.h>

#define MAX_RULES 32

typedef struct {
    uint32_t start_khz;
    uint32_t end_khz;
    uint16_t band;
    char mode;
} band_rule_t;

extern band_rule_t band_rules[MAX_RULES];
extern uint8_t rules_count;

#endif
