#include "LPC13xx.h"
#include "math_utils.h"

/**
 * value1: current msticks
 * value2: past msticks
 */
uint32_t math_calc_diff(uint32_t value1, uint32_t value2) {
	if (value1 == value2) {
		return 0;
	}
	if (value1 > value2) {
		return (value1 - value2);
	}
	else {
		// check for overflow
		return (UINT32_MAX - value2 + value1);
	}
}
