/*
 * platform_driver_init.h
 *
 *  Created on: 2023¦~9¤ë8¤é
 *      Author: 902447
 */

#ifndef BOARD_EPII_EVB_PLATFORM_DRIVER_INIT_H_
#define BOARD_EPII_EVB_PLATFORM_DRIVER_INIT_H_

#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "WE2_device.h"
#ifdef IP_uart
#include"hx_drv_uart.h"
#endif
#ifdef IP_scu
#include"hx_drv_scu.h"
#endif
#ifdef IP_timer
#include "hx_drv_timer.h"
#endif
#ifdef IP_rtc
#include "hx_drv_rtc.h"
#endif
#ifdef IP_watchdog
#include "hx_drv_watchdog.h"
#endif
#ifdef IP_gpio
#include"hx_drv_gpio.h"
#endif
#ifdef IP_dma
#include "hx_drv_dmac.h"
#endif
#ifdef IP_timer
#include "timer_interface.h"
#endif

void platform_driver_init();

#endif /* BOARD_EPII_EVB_PLATFORM_DRIVER_INIT_H_ */
