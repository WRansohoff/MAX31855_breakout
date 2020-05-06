#ifndef _VVC_MAIN_H
#define _VVC_MAIN_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "stm32l4xx.h"

// Pre-defined memory locations for program initialization.
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;

// Global variables.
uint32_t core_clock_hz;
volatile uint32_t systick;

#endif
