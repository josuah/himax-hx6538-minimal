#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hx_drv_scu.h"
#include "hx_drv_uart.h"
#include "driver_interface.h"
#include "board.h"

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
	DEV_UART *console_uart = hx_drv_uart_get_dev(DW_UART_0_ID);

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

void hx_set_memory(unsigned int addr, unsigned int val)
{
	(*((volatile unsigned int*) addr)) = val;
}

unsigned int hx_get_memory(unsigned int addr)
{
	return (*((volatile unsigned int*) addr));
}

extern void hx_CleanDCache_by_Addr(volatile void *addr, int32_t dsize)
{
	uint32_t dtcm_start = BASE_ADDR_DTCM_ALIAS;
	uint32_t dtcm_end = BASE_ADDR_DTCM_ALIAS + DTCM_SIZE;
	uint32_t itcm_start = BASE_ADDR_ITCM_ALIAS;
	uint32_t itcm_end = BASE_ADDR_ITCM_ALIAS + ITCM_SIZE;

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

extern uint32_t __INITIAL_SP;
extern uint32_t __STACK_LIMIT;

__NO_RETURN void Reset_Handler(void)
{
	__set_MSP((uint32_t)(&__INITIAL_SP));
	__set_MSPLIM((uint32_t) (&__STACK_LIMIT));

	SystemInit(); /* CMSIS System Initialization */

	SCB_DisableICache();
	SCB_DisableDCache();
	SCB_EnableICache();
	SCB_EnableDCache();


	__PROGRAM_START();
}

__NO_RETURN void Default_Handler(void)
{
}

void NMI_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler(void) __attribute__ ((weak));
void MemManage_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void BusFault_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void SecureFault_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void DebugMon_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void Interrupt0_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void Interrupt1_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void Interrupt2_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void Interrupt3_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void Interrupt4_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void Interrupt5_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void Interrupt6_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void Interrupt7_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void Interrupt8_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void Interrupt9_Handler(void) __attribute__ ((weak, alias("Default_Handler")));

const VECTOR_TABLE_Type __VECTOR_TABLE[496] __VECTOR_TABLE_ATTRIBUTE = {
	   (VECTOR_TABLE_Type)(&__INITIAL_SP), /*	 Initial Stack Pointer */
	   Reset_Handler, /*	 Reset Handler */
	   NMI_Handler, /* -14 NMI Handler */
	   HardFault_Handler, /* -13 Hard Fault Handler */
	   MemManage_Handler, /* -12 MPU Fault Handler */
	   BusFault_Handler, /* -11 Bus Fault Handler */
	   UsageFault_Handler, /* -10 Usage Fault Handler */
	   SecureFault_Handler, /*  -9 Secure Fault Handler */
	   0, /*	 Reserved */
	   0, /*	 Reserved */
	   0, /*	 Reserved */
	   SVC_Handler, /*  -5 SVCall Handler */
	   DebugMon_Handler, /*  -4 Debug Monitor Handler */
	   0, /*	 Reserved */
	   PendSV_Handler, /*  -2 PendSV Handler */
	   SysTick_Handler, /*  -1 SysTick Handler */

	   /* Interrupts */
	   Interrupt0_Handler, /*   0 Interrupt 0 */
	   Interrupt1_Handler, /*   1 Interrupt 1 */
	   Interrupt2_Handler, /*   2 Interrupt 2 */
	   Interrupt3_Handler, /*   3 Interrupt 3 */
	   Interrupt4_Handler, /*   4 Interrupt 4 */
	   Interrupt5_Handler, /*   5 Interrupt 5 */
	   Interrupt6_Handler, /*   6 Interrupt 6 */
	   Interrupt7_Handler, /*   7 Interrupt 7 */
	   Interrupt8_Handler, /*   8 Interrupt 8 */
	   Interrupt9_Handler /*   9 Interrupt 9 */
	   /* Interrupts 10 .. 480 are left out */
};

uint32_t SystemCoreClock = SYSTEM_CLOCK;

volatile unsigned long g_time_loop = 0;

void SystemCoreClockInit(void)
{
	uint32_t val = SysTick_LOAD_RELOAD_Msk+1;
	SystemCoreClock = SYSTEM_CLOCK;
	g_time_loop = 0;
#if !defined(ENABLE_OS) && !defined(RTE_CMSIS_RTOS2) && !defined(RTE_RTOS_FreeRTOS_CORE)
	//SYSTICK MAX
	if (SysTick_Config(val)) {

		while (1)
			;
	}
#endif
}

void SystemCoreClockUpdate(uint32_t clock) {
	uint32_t val = SysTick_LOAD_RELOAD_Msk+1;
	SystemCoreClock = clock;
	g_time_loop = 0;
#if !defined(ENABLE_OS) && !defined(RTE_CMSIS_RTOS2) && !defined(RTE_RTOS_FreeRTOS_CORE)
	//SYSTICK MAX
	if (SysTick_Config(val)) {

		while (1)
			;
	}
#endif
}

/**
 \brief  Get Current Tick.

 */
void SystemGetTick(uint32_t *systick, uint32_t *loop_cnt)
{
	*systick = SysTick->VAL;
	*loop_cnt = g_time_loop;
}

#if !defined(HX_TFM)

void SystemInit(void) {

#if defined (__VTOR_PRESENT) && (__VTOR_PRESENT == 1U)
  SCB->VTOR = (uint32_t)(&__VECTOR_TABLE[0]);
#endif

#if (defined (__FPU_USED) && (__FPU_USED == 1U)) || \
    (defined (__ARM_FEATURE_MVE) && (__ARM_FEATURE_MVE > 0U))
  SCB->CPACR |= ((3U << 10U*2U) |           /* enable CP10 Full Access */
                 (3U << 11U*2U)  );         /* enable CP11 Full Access */
#if defined (__ARM_FEATURE_CMSE) &&  (__ARM_FEATURE_CMSE == 3U)
  SCB_NS->CPACR |= ((3U << 10U*2U) |           /* enable CP10 Full Access */
                 (3U << 11U*2U)  );         /* enable CP11 Full Access */
#endif
#endif
	SCB->SHCSR |= (1 << SCB_SHCSR_MEMFAULTENA_Pos);
	SCB->SHCSR |= (1 << SCB_SHCSR_BUSFAULTENA_Pos);
	SCB->SHCSR |= (1 << SCB_SHCSR_USGFAULTENA_Pos);
#if defined (__ARM_FEATURE_CMSE) &&  (__ARM_FEATURE_CMSE == 3U)
  SCB->SHCSR |= (1 << SCB_SHCSR_SECUREFAULTENA_Pos);
#endif

#ifdef UNALIGNED_SUPPORT_DISABLE
  SCB->CCR |= SCB_CCR_UNALIGN_TRP_Msk;
#endif

// Enable Loop and branch info cache
	SCB->CCR |= SCB_CCR_LOB_Msk;
	__ISB();


	for(IRQn_Type idx = 0; idx <=200; idx++)
	{
		NVIC_DisableIRQ(idx);
	}

#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
#ifndef TRUSTZONE_SEC_ONLY
  hx_drv_scu_set_CM55M_IDAU(3, 0, 0, 1);
#endif
#endif

  SystemCoreClockInit();
}

#endif //#if !defined(HX_TFM)
