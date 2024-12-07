#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hx_drv_scu.h"
#include "hx_drv_uart.h"
#include "driver_interface.h"

#define WE2_CHIP_VERSION_C	      0x8538000c

void *_sbrk_r() { return NULL; }
void _exit() { return 0; }
int _read_r() { return 0; }
int _write_r() { return 0; }
int _close_r() { return 0; }
int _fstat_r() { return 0; }
int _isatty_() { return 0; }
int _isatty() { return 0; }
int _lseek_r() { return 0; }

int main(void)
{
	SCU_PINMUX_CFG_T pinmux_cfg;
	DEV_UART *console_uart = hx_drv_uart_get_dev(DW_UART_0_ID);
	uint32_t freq;

	hx_drv_scu_get_all_pinmux_cfg(&pinmux_cfg);
	pinmux_cfg.pin_pb0 = SCU_PB0_PINMUX_UART0_RX_1;
	pinmux_cfg.pin_pb1 = SCU_PB1_PINMUX_UART0_TX_1;
	hx_drv_scu_set_all_pinmux_cfg(&pinmux_cfg, 1);
	hx_drv_scu_init();
	hx_drv_scu_get_freq(SCU_CLK_FREQ_TYPE_HSC_CLK, &freq);
	SystemCoreClockUpdate(freq);

	hx_drv_uart_init(USE_DW_UART_0, HX_UART0_BASE);
	console_uart->uart_open(UART_BAUDRATE_921600);

	while (1) {
		console_uart->uart_write("hello world\n", 12);
	}
	return 0;
}

extern void EPII_NVIC_SetVector(IRQn_Type IRQn, uint32_t vector)
{
	uint32_t *vectors = (uint32_t *)SCB->VTOR;
	uint32_t addr = (uint32_t)&vectors[(int32_t)IRQn + NVIC_USER_IRQ_OFFSET];

	NVIC_SetVector(IRQn, vector);

	if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
		SCB_CleanDCache_by_Addr((volatile void *)addr, 4);
	}

	if ((SCB->CCR & SCB_CCR_IC_Msk) != 0U) {
		SCB_InvalidateICache_by_Addr((volatile void *)addr, 4);
	}
}

void hx_set_memory(unsigned int addr, unsigned int val) {
	(*((volatile unsigned int*) addr)) = val;
}

unsigned int hx_get_memory(unsigned int addr) {
	unsigned int val;
	val = (*((volatile unsigned int*) addr));
	return val;
}

extern void hx_CleanDCache_by_Addr(volatile void *addr, int32_t dsize)
{
	uint32_t dtcm_start;
	uint32_t dtcm_end;
	uint32_t itcm_start;
	uint32_t itcm_end;

	dtcm_start = BASE_ADDR_DTCM_ALIAS;
	dtcm_end = BASE_ADDR_DTCM_ALIAS + DTCM_SIZE;
	itcm_start = BASE_ADDR_ITCM_ALIAS;
	itcm_end = BASE_ADDR_ITCM_ALIAS + ITCM_SIZE;

	if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
		if (((uint32_t)addr >= dtcm_start) && ((uint32_t)addr <= dtcm_end)) {
			return;
		}
		if (((uint32_t)addr >= itcm_start) && ((uint32_t)addr <= itcm_end)) {
			return;
		}
		SCB_CleanDCache_by_Addr(addr, dsize);
	}
}

DRIVER_INTERFACE_E drv_interface_get_freq(SCU_CLK_FREQ_TYPE_E type, uint32_t *freq)
{
	SCU_ERROR_E ret;

	ret = hx_drv_scu_get_freq(type, freq);
	return (ret == SCU_NO_ERROR) ? DRIVER_INTERFACE_NO_ERROR : DRIVER_INTERFACE_UNKNOWN_ERROR;
}

DRIVER_INTERFACE_E drv_interface_get_pdlsc_clken_cfg(SCU_PDLSC_CLKEN_CFG_T *cfg)
{
	hx_drv_scu_get_pdlsc_clken_cfg(cfg);
	return DRIVER_INTERFACE_NO_ERROR;
}
