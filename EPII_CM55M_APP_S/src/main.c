#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "WE2_device.h"
#include "WE2_core.h"
#include "xprintf.h"
#include "hx_drv_pmu.h"
#include "hx_drv_scu.h"
#include "hx_drv_uart.h"
#include "console_io.h"
#include "trustzone_cfg.h"

#define WE2_CHIP_VERSION_C	      0x8538000c

void *_sbrk_r() { return NULL; }
int _exit() { return 0; }
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
