/*
 * USB Serial
 * 
 * Copyright (c) 2020 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 * 
 * Common functions
 */

#include "common.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>

static volatile uint32_t millis_count;

uint32_t millis()
{
	return millis_count;
}

void delay(uint32_t ms)
{
	int32_t target_time = millis_count + ms;
	while (target_time - (int32_t)millis_count > 0)
		;
}

bool has_expired(uint32_t timeout)
{
    return (int32_t)timeout - (int32_t)millis_count <= 0;
}

void rcc_clock_setup_in_hsebyp_16mhz_out_48mhz(void)
{
	RCC_CR |= RCC_CR_HSEBYP;

	rcc_osc_on(RCC_HSE);
	rcc_wait_for_osc_ready(RCC_HSE);
	rcc_set_sysclk_source(RCC_HSE);

	rcc_set_hpre(RCC_CFGR_HPRE_NODIV);
	rcc_set_ppre(RCC_CFGR_PPRE_NODIV);

	flash_prefetch_enable();
	flash_set_ws(FLASH_ACR_LATENCY_024_048MHZ);

	/* PLL: 16MHz * 3 = 48MHz */
	rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_MUL3);
	rcc_set_pll_source(RCC_CFGR_PLLSRC_HSE_CLK);
	rcc_set_pllxtpre(RCC_CFGR_PLLXTPRE_HSE_CLK);

	rcc_osc_on(RCC_PLL);
	rcc_wait_for_osc_ready(RCC_PLL);
	rcc_set_sysclk_source(RCC_PLL);

	rcc_apb1_frequency = 48000000;
	rcc_ahb_frequency = 48000000;
}

void common_init()
{
	// Initialize SysTick

	rcc_clock_setup_in_hsebyp_16mhz_out_48mhz();
	// rcc_clock_setup_in_hsi_out_48mhz();

	// Interrupt every 1ms
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_set_reload(rcc_ahb_frequency / 1000 - 1);

	// Enable and start
	systick_interrupt_enable();
	systick_counter_enable();
}

// System tick timer interrupt handler
extern "C" void sys_tick_handler()
{
	millis_count++;
}

