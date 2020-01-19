#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/systick.h>

#include <core/types.h>

#include "millis.h"

// 32 bits so that loads are atomic
static volatile uint32_t _millis = 0;

extern "C" void sys_tick_handler() {
	_millis++;
}

void millis_init() {
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_set_reload(rcc_ahb_frequency/1000 - 1);
	
	STK_CVR = 0;
	
	systick_counter_enable();
	systick_interrupt_enable();
}

uint32_t millis() {
	return _millis;
}

void delay_ms(uint32_t ms) {
	uint32_t t = millis();
	while(millis() - t < ms);
}
