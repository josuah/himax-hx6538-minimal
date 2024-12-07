/*
 * platform_driver_init.c
 *
 *  Created on: 2023¦~9¤ë8¤é
 *      Author: 902447
 */

#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "WE2_device.h"

#if defined (__ARM_FEATURE_CMSE) &&  (__ARM_FEATURE_CMSE == 3U)

   #include "trustzone_cfg.h"
#endif

#include "platform_driver_init.h"


void __attribute__((weak)) platform_driver_init()
{
	uint32_t freq;


#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
#ifdef TRUSTZONE_SEC_ONLY
    TZ_Set_ALL_Secure();
#else
    TZ_Set_Secure_ByCFG();
#endif
#endif

#if defined(__GNU__) && defined(SEMIHOST)
	initialise_monitor_handles();
#endif


#ifdef IP_scu
	hx_drv_scu_init();
#endif
	hx_drv_scu_get_freq(SCU_CLK_FREQ_TYPE_HSC_CLK, &freq);
	SystemCoreClockUpdate(freq);

#ifdef IP_dma
#if defined(IP_INST_DMA0) || defined(IP_INST_NS_DMA0)
    hx_drv_dmac_init(USE_HX_DMAC_0, DMAC0_BASE);
#endif
#if defined(IP_INST_DMA1) || defined(IP_INST_NS_DMA1)
    hx_drv_dmac_init(USE_HX_DMAC_1, DMAC1_BASE);
#endif
#if defined(IP_INST_DMA2) || defined(IP_INST_NS_DMA2)
    hx_drv_dmac_init(USE_HX_DMAC_2, DMAC2_BASE);
#endif
#if defined(IP_INST_DMA3) || defined(IP_INST_NS_DMA3)
    hx_drv_dmac_init(USE_HX_DMAC_3, DMAC3_BASE);
#endif
#endif

#ifdef IP_timer
#if defined(IP_INST_TIMER0) || defined(IP_INST_NS_TIMER0)
	hx_drv_timer_init(TIMER_ID_0, HX_TIMER0_BASE);
#endif
#if defined(IP_INST_TIMER1) || defined(IP_INST_NS_TIMER1)
	hx_drv_timer_init(TIMER_ID_1, HX_TIMER1_BASE);
#endif
#if defined(IP_INST_TIMER2) || defined(IP_INST_NS_TIMER2)
	hx_drv_timer_init(TIMER_ID_2, HX_TIMER2_BASE);
#endif
#if defined(IP_INST_TIMER3) || defined(IP_INST_NS_TIMER3)
	hx_drv_timer_init(TIMER_ID_3, HX_TIMER3_BASE);
#endif
#if defined(IP_INST_TIMER4) || defined(IP_INST_NS_TIMER4)
	hx_drv_timer_init(TIMER_ID_4, HX_TIMER4_BASE);
#endif
#if defined(IP_INST_TIMER5) || defined(IP_INST_NS_TIMER5)
	hx_drv_timer_init(TIMER_ID_5, HX_TIMER5_BASE);
#endif
#if defined(IP_INST_TIMER6) || defined(IP_INST_NS_TIMER6)
	hx_drv_timer_init(TIMER_ID_6, HX_TIMER6_BASE);
#endif
#if defined(IP_INST_TIMER7) || defined(IP_INST_NS_TIMER7)
	hx_drv_timer_init(TIMER_ID_7, HX_TIMER7_BASE);
#endif
#if defined(IP_INST_TIMER8) || defined(IP_INST_NS_TIMER8)
	hx_drv_timer_init(TIMER_ID_8, HX_TIMER8_BASE);
#endif
#endif
#ifdef IP_watchdog
#if defined(IP_INST_WDT0) || defined(IP_INST_NS_WDT0)
	hx_drv_watchdog_init(WATCHDOG_ID_0, HX_WDG0_BASE);
#endif
#if defined(IP_INST_WDT1) || defined(IP_INST_NS_WDT1)
	hx_drv_watchdog_init(WATCHDOG_ID_1, HX_WDG1_BASE);
#endif
#endif
#ifdef IP_rtc
#if defined(IP_INST_RTC0) || defined(IP_INST_NS_RTC0)
	hx_drv_rtc_init(RTC_ID_0, HX_RTC0_BASE);
#endif
#if defined(IP_INST_RTC1) || defined(IP_INST_NS_RTC1)
	hx_drv_rtc_init(RTC_ID_1, HX_RTC1_BASE);
#endif
#if defined(IP_INST_RTC2) || defined(IP_INST_NS_RTC2)
	hx_drv_rtc_init(RTC_ID_2, HX_RTC2_BASE);
#endif
#endif
#ifdef IP_uart
	#if defined(IP_INST_UART0) || defined(IP_INST_NS_UART0)
	hx_drv_uart_init(USE_DW_UART_0, HX_UART0_BASE);
	#endif
	#if defined(IP_INST_UART1) || defined(IP_INST_NS_UART1)
	hx_drv_uart_init(USE_DW_UART_1, HX_UART1_BASE);
	#endif
	#if defined(IP_INST_UART2) || defined(IP_INST_NS_UART2)
	hx_drv_uart_init(USE_DW_UART_2, HX_UART2_BASE);
	#endif
#endif
#ifdef IP_gpio
	#if defined(IP_INST_GPIO_G0) || defined(IP_INST_NS_GPIO_G0)
	hx_drv_gpio_init(GPIO_GROUP_0, HX_GPIO_GROUP_0_BASE);
	#endif
	#if defined(IP_INST_GPIO_G1) || defined(IP_INST_NS_GPIO_G1)
	hx_drv_gpio_init(GPIO_GROUP_1, HX_GPIO_GROUP_1_BASE);
	#endif
	#if defined(IP_INST_GPIO_G2) || defined(IP_INST_NS_GPIO_G2)
	hx_drv_gpio_init(GPIO_GROUP_2, HX_GPIO_GROUP_2_BASE);
	#endif
	#if defined(IP_INST_GPIO_G3) || defined(IP_INST_NS_GPIO_G3)
	hx_drv_gpio_init(GPIO_GROUP_3, HX_GPIO_GROUP_3_BASE);
	#endif
	#if defined(IP_INST_SB_GPIO) || defined(IP_INST_NS_SB_GPIO)
	hx_drv_gpio_init(GPIO_GROUP_4, HX_GPIO_GROUP_4_BASE);
	#endif
	#if defined(IP_INST_AON_GPIO) || defined(IP_INST_NS_AON_GPIO)
	hx_drv_gpio_init(GPIO_GROUP_5, HX_GPIO_GROUP_5_BASE);
	#endif
#endif
}
