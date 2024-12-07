#include <stdio.h>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "powermode_export.h"
#include "WE2_device.h"
#include "board.h"
#include "xprintf.h"
#include "board.h"
#include "WE2_core.h"
#include "hx_drv_scu.h"
#include "hx_drv_swreg_aon.h"
#include "hx_drv_pmu_export.h"
#include "hx_drv_pmu.h"
#include "powermode.h"

#define WE2_CHIP_VERSION_C		0x8538000c
#define FRAME_CHECK_DEBUG		1

int app_main(void) {
	uint32_t wakeup_event;
	uint32_t wakeup_event1;

	hx_drv_pmu_get_ctrl(PMU_pmu_wakeup_EVT, &wakeup_event);
	hx_drv_pmu_get_ctrl(PMU_pmu_wakeup_EVT1, &wakeup_event1);

	while (1) {
		xprintf("wakeup_event=0x%x,WakeupEvt1=0x%x\n", wakeup_event, wakeup_event1);
	}

	return 0;
}
