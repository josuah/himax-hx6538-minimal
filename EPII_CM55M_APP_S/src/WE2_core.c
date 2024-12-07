#include "WE2_device.h"
#include "cachel1_armv7.h"
#include "board.h"
#include "BITOPS.h"
#include "WE2_core.h"
#include "xprintf.h"
#include "hx_drv_scu.h"
#define MSCR_CTRL_ENABLE     1
#define MPU_QSPI_REGION    14
#define MPU_OSPI_REGION    15
#define MPU_DEVICE_MEMORY_nGnRnE						( 0x00 ) /* 0000 0000 */
#define MPU_DEVICE_MEMORY_nGnRE							( 0x04 ) /* 0000 0100 */
#define MPU_DEVICE_MEMORY_nGRE							( 0x08 ) /* 0000 1000 */
#define MPU_DEVICE_MEMORY_GRE							( 0x0C ) /* 0000 1100 */
#define MPU_NORMAL_MEMORY_NON_CACHEABLE					( 0x44 ) /* Non-cacheable. */
#define MPU_NORMAL_MEMORY_BUFFERABLE_CACHEABLE			( 0xFF ) /* Non-Transient, Write-back, Read-Allocate and Write-Allocate. */
#define MPU_NORMAL_MEMORY_FLASH							( 0x44  )
#define MPU_REGION_NON_SHAREABLE						( 0UL << 3UL )
#define MPU_REGION_INNER_SHAREABLE						( 1UL << 3UL )
#define MPU_REGION_OUTER_SHAREABLE						( 2UL << 3UL )
#define MPU_REGION_PRIVILEGED_READ_WRITE				( 0UL << 1UL )
#define MPU_REGION_READ_WRITE							( 1UL << 1UL )
#define MPU_REGION_PRIVILEGED_READ_ONLY					( 2UL << 1UL )
#define MPU_REGION_READ_ONLY							( 3UL << 1UL )
#define TCMCR_ENABLE_Pos                0U                                            /*!< MPU TYPE: DREGION Position */
#define TCMCR_ENABLE_Msk               (0x1UL << TCMCR_ENABLE_Pos)
#define TCMCR_ENABLE_Size               1
#define TCMCR_SIZE_Pos                3U                                            /*!< MPU TYPE: DREGION Position */
#define TCMCR_SIZE_Msk               (0xFUL << TCMCR_ENABLE_Pos)
#define TCMCR_SIZE_Size                4U
#define TGUCTRL_DBFEN_Pos                0U
#define TGUCTRL_DBFEN_Msk               (0x1UL << TGUCTRL_DBFEN_Pos)
#define TGUCTRL_DBFEN_Size                1U
#define TGUCTRL_DEREN_Pos                1U
#define TGUCTRL_DEREN_Msk               (0x1UL << TGUCTRL_DEREN_Pos)
#define TGUCTRL_DEREN_Size                1U
#define TGUCFG_PRESENT_Pos               31U
#define TGUCFG_PRESENT_Msk               (0x1UL << TGUCFG_PRESENT_Pos)
#define TGUCFG_PRESENT_Size               1U
#define TGUCFG_NUMBLKS_Pos               8U
#define TGUCFG_NUMBLKS_Msk               (0xFUL << TGUCFG_NUMBLKS_Pos)
#define TGUCFG_NUMBLKS_Size               4U
#define TGUCFG_BLKSZ_Pos               0U
#define TGUCFG_BLKSZ_Msk               (0xFUL << TGUCFG_BLKSZ_Pos)
#define TGUCFG_BLKSZ_Size               4U
#define REG_MSCR_ADDR	0xE001E000
#define REG_MSCR_ICACTIVE_POS	13
#define REG_MSCR_ICACTIVE_SIZE	1
#define REG_MSCR_DCACTIVE_POS	12
#define REG_MSCR_DCACTIVE_SIZE	1
extern uint32_t SystemCoreClock;

#define EPII_SYS_TIMER_US_HZ		(1000000)

void EPII_set_reg(unsigned int addr, unsigned int val) {
	(*((volatile unsigned int*) addr)) = val;
}

unsigned int EPII_get_reg(unsigned int addr) {
	unsigned int val;
	val = (*((volatile unsigned int*) addr));
	return val;
}

__STATIC_INLINE uint32_t EPII_NVIC_GetVectorAddr(IRQn_Type IRQn)
{
	  uint32_t *vectors = (uint32_t *)SCB->VTOR;
	  return (uint32_t)&vectors[(int32_t)IRQn + NVIC_USER_IRQ_OFFSET];
}

extern void EPII_NVIC_SetVector(IRQn_Type IRQn, uint32_t vector)
{
#ifdef BOOTROM
	return;
#else
	uint32_t addr;
	NVIC_SetVector(IRQn, vector);
	addr = EPII_NVIC_GetVectorAddr(IRQn);
	if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
	{
		SCB_CleanDCache_by_Addr((volatile void *)addr, 4);
	}
	if ((SCB->CCR & SCB_CCR_IC_Msk) != 0U)
	{
		SCB_InvalidateICache_by_Addr((volatile void *)addr, 4);
	}
#endif
}



void hx_set_memory(unsigned int addr, unsigned int val) {
	(*((volatile unsigned int*) addr)) = val;
}

unsigned int hx_get_memory(unsigned int addr) {
	unsigned int val;
	val = (*((volatile unsigned int*) addr));
	return val;
}

void EPII_cpu_nop_us(uint32_t period_us)
{
	uint32_t freq, delay_nop_cnt;

#ifdef CM55_BIG
#ifdef NSC
	freq = veneer_clk_get_freq(SCU_CLK_FREQ_TYPE_HSC_CM55M);
#else
	hx_drv_scu_get_freq(SCU_CLK_FREQ_TYPE_HSC_CM55M, &freq);
#endif
#else
#ifdef NSC
	freq = veneer_clk_get_freq(SCU_CLK_FREQ_TYPE_LSC_CM55S);
#else
	hx_drv_scu_get_freq(SCU_CLK_FREQ_TYPE_LSC_CM55S, &freq);
#endif
#endif
	delay_nop_cnt = (freq/EPII_SYS_TIMER_US_HZ) * period_us;

	for(uint32_t idx = 0; idx < delay_nop_cnt; idx++)
	{
	    __NOP();
	}
}


/**
 \brief  Enable/Disable I Cache

 Enable/Disable I Cache
 */
extern void hx_enable_ICache(uint32_t enable)
{
	uint32_t val;
	if(enable == 0)
	{
		SCB_DisableICache();
#ifdef BOOTROM
		val = EPII_get_reg(REG_MSCR_ADDR);
		HX_BIT_SET(val, REG_MSCR_ICACTIVE_SIZE, REG_MSCR_ICACTIVE_POS, 1);
		EPII_set_reg(REG_MSCR_ADDR, val);
#else
#if MSCR_CTRL_ENABLE
		val = EPII_get_reg(REG_MSCR_ADDR);
		HX_BIT_SET(val, REG_MSCR_ICACTIVE_SIZE, REG_MSCR_ICACTIVE_POS, 0);
		EPII_set_reg(REG_MSCR_ADDR, val);
#endif
#endif
	}else{
#if MSCR_CTRL_ENABLE
		val = EPII_get_reg(REG_MSCR_ADDR);
		HX_BIT_SET(val, REG_MSCR_ICACTIVE_SIZE, REG_MSCR_ICACTIVE_POS, 1);
		EPII_set_reg(REG_MSCR_ADDR, val);
#endif
		SCB_EnableICache();
	}
}

/**
 \brief  Enable/Disable D Cache

 Enable/Disable D Cache
 */
extern void hx_enable_DCache(uint32_t enable)
{
	uint32_t val;
	if(enable == 0)
	{
		SCB_DisableDCache();
#if MSCR_CTRL_ENABLE
#ifdef BOOTROM
		val = EPII_get_reg(REG_MSCR_ADDR);
		HX_BIT_SET(val, REG_MSCR_DCACTIVE_SIZE, REG_MSCR_DCACTIVE_POS, 1);
		EPII_set_reg(REG_MSCR_ADDR, val);
#else
		val = EPII_get_reg(REG_MSCR_ADDR);
		HX_BIT_SET(val, REG_MSCR_DCACTIVE_SIZE, REG_MSCR_DCACTIVE_POS, 0);
		EPII_set_reg(REG_MSCR_ADDR, val);
#endif
#endif
	}else{
#if MSCR_CTRL_ENABLE
		val = EPII_get_reg(REG_MSCR_ADDR);
		HX_BIT_SET(val, REG_MSCR_DCACTIVE_SIZE, REG_MSCR_DCACTIVE_POS, 1);
		EPII_set_reg(REG_MSCR_ADDR, val);
#endif
		SCB_EnableDCache();
	}
}

/**
 \brief  D-Cache clean by address

 D-Cache clean by address
 */
extern void hx_CleanDCache_by_Addr(volatile void *addr, int32_t dsize)
{
	uint32_t dtcm_start;
	uint32_t dtcm_end;
	uint32_t itcm_start;
	uint32_t itcm_end;
#ifdef TRUSTZONE_SEC
	dtcm_start = BASE_ADDR_DTCM_ALIAS;
	dtcm_end = BASE_ADDR_DTCM_ALIAS + DTCM_SIZE;
	itcm_start = BASE_ADDR_ITCM_ALIAS;
	itcm_end = BASE_ADDR_ITCM_ALIAS + ITCM_SIZE;
#else
	dtcm_start = BASE_ADDR_DTCM;
	dtcm_end = BASE_ADDR_DTCM + DTCM_SIZE;
	itcm_start = BASE_ADDR_ITCM;
	itcm_end = BASE_ADDR_ITCM + ITCM_SIZE;
#endif

	if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
	{
		if(((uint32_t)addr >= dtcm_start) && ((uint32_t)addr <= dtcm_end))
		{
			return;
		}
		if(((uint32_t)addr >= itcm_start) && ((uint32_t)addr <= itcm_end))
		{
			return;
		}
		SCB_CleanDCache_by_Addr(addr, dsize);
	}
}

/**
 \brief  D-Cache Invalidate by address

 D-Cache Invalidate by address
 */
extern void hx_InvalidateDCache_by_Addr(volatile void *addr, int32_t dsize)
{
	uint32_t dtcm_start;
	uint32_t dtcm_end;
	uint32_t itcm_start;
	uint32_t itcm_end;
#ifdef TRUSTZONE_SEC
	dtcm_start = BASE_ADDR_DTCM_ALIAS;
	dtcm_end = BASE_ADDR_DTCM_ALIAS + DTCM_SIZE;
	itcm_start = BASE_ADDR_ITCM_ALIAS;
	itcm_end = BASE_ADDR_ITCM_ALIAS + ITCM_SIZE;
#else
	dtcm_start = BASE_ADDR_DTCM;
	dtcm_end = BASE_ADDR_DTCM + DTCM_SIZE;
	itcm_start = BASE_ADDR_ITCM;
	itcm_end = BASE_ADDR_ITCM + ITCM_SIZE;
#endif
	if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
	{
		if(((uint32_t)addr >= dtcm_start) && ((uint32_t)addr <= dtcm_end))
		{
			return;
		}
		if(((uint32_t)addr >= itcm_start) && ((uint32_t)addr <= itcm_end))
		{
			return;
		}
		SCB_InvalidateDCache_by_Addr(addr, dsize);
	}
}

extern void hx_InvalidateICache_by_Addr(volatile void *addr, int32_t isize)
{
	uint32_t itcm_start;
	uint32_t itcm_end;
	uint32_t dtcm_start;
	uint32_t dtcm_end;
#ifdef TRUSTZONE_SEC
	dtcm_start = BASE_ADDR_DTCM_ALIAS;
	dtcm_end = BASE_ADDR_DTCM_ALIAS + DTCM_SIZE;
	itcm_start = BASE_ADDR_ITCM_ALIAS;
	itcm_end = BASE_ADDR_ITCM_ALIAS + ITCM_SIZE;
#else
	dtcm_start = BASE_ADDR_DTCM;
	dtcm_end = BASE_ADDR_DTCM + DTCM_SIZE;
	itcm_start = BASE_ADDR_ITCM;
	itcm_end = BASE_ADDR_ITCM + ITCM_SIZE;
#endif
	if ((SCB->CCR & SCB_CCR_IC_Msk) != 0U)
	{
		if(((uint32_t)addr >= itcm_start) && ((uint32_t)addr <= itcm_end))
		{
			return;
		}
		if(((uint32_t)addr >= dtcm_start) && ((uint32_t)addr <= dtcm_end))
		{
			return;
		}
		SCB_InvalidateICache_by_Addr(addr, isize);
	}
}

/**
 \brief  D-Cache Clean Invalidate by address

 D-Cache Invalidate by address
 */
extern void hx_CleanInvalidateDCache_by_Addr(volatile void *addr, int32_t dsize)
{
	uint32_t mod_val;
	uint32_t dtcm_start;
	uint32_t dtcm_end;
	uint32_t itcm_start;
	uint32_t itcm_end;
#ifdef TRUSTZONE_SEC
	dtcm_start = BASE_ADDR_DTCM_ALIAS;
	dtcm_end = BASE_ADDR_DTCM_ALIAS + DTCM_SIZE;
	itcm_start = BASE_ADDR_ITCM_ALIAS;
	itcm_end = BASE_ADDR_ITCM_ALIAS + ITCM_SIZE;
#else
	dtcm_start = BASE_ADDR_DTCM;
	dtcm_end = BASE_ADDR_DTCM + DTCM_SIZE;
	itcm_start = BASE_ADDR_ITCM;
	itcm_end = BASE_ADDR_ITCM + ITCM_SIZE;
#endif
	if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
	{
		if(((uint32_t)addr >= dtcm_start) && ((uint32_t)addr <= dtcm_end))
		{
			return;
		}
		if(((uint32_t)addr >= itcm_start) && ((uint32_t)addr <= itcm_end))
		{
			return;
		}
		SCB_CleanInvalidateDCache_by_Addr(addr, dsize);
	}
}
