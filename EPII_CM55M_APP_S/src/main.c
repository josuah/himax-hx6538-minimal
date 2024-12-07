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

int main(void)
{
	SCU_PINMUX_CFG_T pinmux_cfg;
	uint32_t wakeup_event;
	uint32_t wakeup_event1;
	uint32_t freq;

	hx_drv_scu_get_all_pinmux_cfg(&pinmux_cfg);
	pinmux_cfg.pin_pb0 = SCU_PB0_PINMUX_UART0_RX_1;
	pinmux_cfg.pin_pb1 = SCU_PB1_PINMUX_UART0_TX_1;
	hx_drv_scu_set_all_pinmux_cfg(&pinmux_cfg, 1);
	hx_drv_pmu_get_ctrl(PMU_pmu_wakeup_EVT, &wakeup_event);
	hx_drv_pmu_get_ctrl(PMU_pmu_wakeup_EVT1, &wakeup_event1);
	TZ_Set_ALL_Secure();
	hx_drv_scu_init();
	hx_drv_scu_get_freq(SCU_CLK_FREQ_TYPE_HSC_CLK, &freq);
	SystemCoreClockUpdate(freq);
	hx_drv_uart_init(USE_DW_UART_0, HX_UART0_BASE);
	hx_drv_uart_init(USE_DW_UART_1, HX_UART1_BASE);
	hx_drv_uart_init(USE_DW_UART_2, HX_UART2_BASE);
	console_setup(DW_UART_0_ID, UART_BAUDRATE_921600);
	xprintf_setup();

	while (1) {
		xprintf("wakeup_event=0x%x,WakeupEvt1=0x%x\n", wakeup_event, wakeup_event1);
	}
	return 0;
}
