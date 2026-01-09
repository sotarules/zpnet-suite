#pragma once
#include <stdint.h>

void gpt_clock_init();
void gpt_clock_start();
void gpt_clock_stop();
uint32_t gpt_clock_read();
