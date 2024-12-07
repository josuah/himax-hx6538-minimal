/*
 * mpu.c
 *
 *  Created on: 2021
 *      Author: 902447
 */
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
/*-----------------------------------------------------------*/
/* Device memory attributes used in MPU_MAIR registers.
 *
 * 8-bit values encoded as follows:
 *  Bit[7:4] - 0000 - Device Memory
 *  Bit[3:2] - 00 --> Device-nGnRnE
 *				01 --> Device-nGnRE
 *				10 --> Device-nGRE
 *				11 --> Device-GRE
 *  Bit[1:0] - 00, Reserved.
 */
#define MPU_DEVICE_MEMORY_nGnRnE						( 0x00 ) /* 0000 0000 */
#define MPU_DEVICE_MEMORY_nGnRE							( 0x04 ) /* 0000 0100 */
#define MPU_DEVICE_MEMORY_nGRE							( 0x08 ) /* 0000 1000 */
#define MPU_DEVICE_MEMORY_GRE							( 0x0C ) /* 0000 1100 */
/* Normal memory attributes used in MPU_MAIR registers. */
#define MPU_NORMAL_MEMORY_NON_CACHEABLE					( 0x44 ) /* Non-cacheable. */
#define MPU_NORMAL_MEMORY_BUFFERABLE_CACHEABLE			( 0xFF ) /* Non-Transient, Write-back, Read-Allocate and Write-Allocate. */
/*
 * Memory region  MAIR_ATTR.Outer MAIR_ATTR.Inner Shareability
 * Flash memory 	!=0000 			0b1010 			0
 */
#define MPU_NORMAL_MEMORY_FLASH							( 0x44  )

/* Attributes used in MPU_RBAR registers. */
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

/**
 \brief  Get system clock

system clock
 */
void EPII_Get_Systemclock(uint32_t *val)
{
	*val = SystemCoreClock;
}


/**
 \brief  Set ITGU Error enable

 Set ITGU Error enable.
 */
void EPII_Set_ITGU_Err_En(uint8_t DBFEN, uint8_t DEREN) {
	HX_BIT_SET(ITGU_CTRL_REG, TGUCTRL_DBFEN_Size, TGUCTRL_DBFEN_Pos, DBFEN);

	HX_BIT_SET(ITGU_CTRL_REG, TGUCTRL_DEREN_Size, TGUCTRL_DEREN_Pos, DEREN);
}

/**
 \brief  Get ITGU Error enable

 Get ITGU Error enable.
 */
void EPII_Get_ITGU_Err_En(uint8_t *DBFEN, uint8_t *DEREN) {
	*DBFEN = HX_BIT_GET(ITGU_CTRL_REG, TGUCTRL_DBFEN_Size, TGUCTRL_DBFEN_Pos);
	*DEREN = HX_BIT_GET(ITGU_CTRL_REG, TGUCTRL_DEREN_Size, TGUCTRL_DEREN_Pos);
}

/**
 \brief  Get ITGU CFG

 Get ITGU Configuration
 */
void EPII_Get_ITGU_CFG(TGU_PRESENT_E *present, uint8_t *NUMBLKS, uint8_t *BLKSZ) {
	*present = HX_BIT_GET(ITGU_CFG_REG, TGUCFG_PRESENT_Size,
			TGUCFG_PRESENT_Pos);

	*NUMBLKS = HX_BIT_GET(ITGU_CFG_REG, TGUCFG_NUMBLKS_Size,
			TGUCFG_NUMBLKS_Pos);

	*BLKSZ = HX_BIT_GET(ITGU_CFG_REG, TGUCFG_BLKSZ_Size, TGUCFG_BLKSZ_Pos);
}

/**
 \brief  Set ITGU LUT configuration

 Set ITGU LUT configuration
 */
void EPII_Set_ITGU_LUT(uint8_t block_no, TGU_LUT_E lut) {
	HX_BIT_SET(ITGU_LUT0_CFG_REG, 1, block_no, lut);
}
/**
 \brief  Get ITGU LUT configuration

 Get ITGU LUT configuration
 */
void EPII_Get_ITGU_LUT(uint8_t block_no, TGU_LUT_E *lut) {
	*lut = HX_BIT_GET(ITGU_LUT0_CFG_REG, 1, block_no);
}
/**
 \brief  Set DTGU Error enable

 Set DTGU Error enable.
 */
void EPII_Set_DTGU_Err_En(uint8_t DBFEN, uint8_t DEREN) {
	HX_BIT_SET(DTGU_CTRL_REG, TGUCTRL_DBFEN_Size, TGUCTRL_DBFEN_Pos, DBFEN);

	HX_BIT_SET(DTGU_CTRL_REG, TGUCTRL_DEREN_Size, TGUCTRL_DEREN_Pos, DEREN);
}

/**
 \brief  Get DTGU Error enable

 Get DTGU Error enable.
 */
void EPII_Get_DTGU_Err_En(uint8_t *DBFEN, uint8_t *DEREN) {
	*DBFEN = HX_BIT_GET(DTGU_CTRL_REG, TGUCTRL_DBFEN_Size, TGUCTRL_DBFEN_Pos);
	*DEREN = HX_BIT_GET(DTGU_CTRL_REG, TGUCTRL_DEREN_Size, TGUCTRL_DEREN_Pos);
}

/**
 \brief  Get DTGU CFG

 Get DTGU Configuration
 */
void EPII_Get_DTGU_CFG(TGU_PRESENT_E *present, uint8_t *NUMBLKS, uint8_t *BLKSZ) {
	*present = HX_BIT_GET(DTGU_CFG_REG, TGUCFG_PRESENT_Size,
			TGUCFG_PRESENT_Pos);

	*NUMBLKS = HX_BIT_GET(DTGU_CFG_REG, TGUCFG_NUMBLKS_Size,
			TGUCFG_NUMBLKS_Pos);

	*BLKSZ = HX_BIT_GET(DTGU_CFG_REG, TGUCFG_BLKSZ_Size, TGUCFG_BLKSZ_Pos);
}

/**
 \brief  Set DTGU LUT configuration

 Set DTGU LUT configuration
 */
void EPII_Set_DTGU_LUT(uint8_t block_no, TGU_LUT_E lut) {
	HX_BIT_SET(DTGU_LUT0_CFG_REG, 1, block_no, lut);
}
/**
 \brief  Get DTGU LUT configuration

 Get DTGU LUT configuration
 */
void EPII_Get_DTGU_LUT(uint8_t block_no, TGU_LUT_E *lut) {
	*lut = HX_BIT_GET(DTGU_LUT0_CFG_REG, 1, block_no);
}
/**
 \brief  Set systick load

 Set systick load
 */
extern void EPII_Set_Systick_enable(uint32_t enable) {
	if (enable == 0) {
		SysTick->VAL = 0;
		SysTick->CTRL = 0;
	} else {
		SysTick->VAL = 0;
		SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
		SysTick_CTRL_ENABLE_Msk; /* Enable SysTick IRQ and SysTick Timer */
	}
}
/**
 \brief  NVIC Set Vector

 Set NVIC Vector
 */
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
