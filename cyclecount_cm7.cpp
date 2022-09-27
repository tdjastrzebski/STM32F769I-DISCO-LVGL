#include "cyclecount_cm7.h"

#include "stm32f769xx.h"  // Applies to all Cortex-M7

// Source: https://stackoverflow.com/questions/38355831/measuring-clock-cycle-count-on-cortex-m7

// Not defined in CMSIS 4.00 headers - check if defined
// to allow for possible correction in later versions

#if !defined DWT_LSR_Present_Msk
#define DWT_LSR_Present_Msk ITM_LSR_Present_Msk
#endif
#if !defined DWT_LSR_Access_Msk
#define DWT_LSR_Access_Msk ITM_LSR_Access_Msk
#endif
#define DWT_LAR_KEY 0xC5ACCE55

void dwt_access_enable(bool enable) {
	uint32_t lsr = DWT->LSR;

	if ((lsr & DWT_LSR_Present_Msk) != 0) {
		if (enable) {
			if ((lsr & DWT_LSR_Access_Msk) != 0) {
				// locked: access need unlock
				DWT->LAR = DWT_LAR_KEY;
			}
		} else {
			if ((lsr & DWT_LSR_Access_Msk) == 0) {
				// unlocked
				DWT->LAR = 0;
			}
		}
	}
}

void reset_cnt() {
	CoreDebug->DEMCR |= 0x01000000;
	DWT->CYCCNT = 0;  // reset the counter
	DWT->CTRL = 0;
}

void start_cnt() {
	DWT->CTRL |= 0x00000001;  // enable the counter
}

void stop_cnt() {
	DWT->CTRL &= 0xFFFFFFFE;  // disable the counter
}

unsigned int getCycles() {
	return DWT->CYCCNT;
}