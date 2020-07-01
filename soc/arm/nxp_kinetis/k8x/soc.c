 /*
 * Copyright (c) 2019 SEAL AG
 *
 * Based on NXP K6x soc.c, which is:
 * Copyright (c) 2014-2015 Wind River Systems, Inc.
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <fsl_common.h>
#include <fsl_clock.h>
#include <fsl_smc.h>
#include <fsl_rtc.h>

#define PLLFLLSEL_MCGFLLCLK	(0)
#define PLLFLLSEL_MCGPLLCLK	(1)
#define PLLFLLSEL_IRC48MHZ	(3)

#define ER32KSEL_OSC32KCLK	(0)
#define ER32KSEL_LPO1KHZ	(3)

#define PERIPH_CLK_PLLFLLSEL	(1)
#define PERIPH_CLK_OSCERCLK	(2)
#define PERIPH_CLK_MCGIRCLK	(3)

#define RUNM_RUN		(0)
#define RUNM_VLPR		(2)
#define RUNM_HSRUN		(3)

#define BOARD_FRDIV                                     0x0U
#define BOARD_XTAL32K_CLK_HZ                          32768U  /*!< Board RTC xtal frequency in Hz */
#define RTC_RTC32KCLK_PERIPHERALS_ENABLED                 1U  /*!< RTC32KCLK to other peripherals: enabled */
#define RTC_OSC_CAP_LOAD_0PF                            0x0U  /*!< RTC oscillator capacity load: 0pF */
#define SIM_OSC32KSEL_RTC32KCLK_CLK                       2U  /*!< OSC32KSEL select: RTC32KCLK clock (32.768kHz) */
#define SIM_RTC_CLKOUT_SEL_RTC32KCLK_CLK                  1U  /*!< RTC clock output select: RTC32KCLK clock (32.768kHz) */
#define SIM_SDHC_CLK_SEL_CORE_SYSTEM_CLK                  0U  /*!< SDHC clock select: Core/system clock */
#define SIM_LPUART_CLK_SEL_PLLFLLSEL_CLK                  1U  /*!< LPUART clock select: PLLFLLSEL output clock */
#define SIM_FLEXIO_CLK_SEL_OSCERCLK_CLK                   2U  /*!< FLEXIO clock select: OSCERCLK clock */
#define SIM_CLKOUT_SEL_FLEXBUS_CLK                        0U  /*!< CLKOUT pin clock select: FlexBus clock */

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_CONFIG_SetRtcClock
 * Description   : This function is used to configuring RTC clock including
 * enabling RTC oscillator.
 * Param capLoad : RTC oscillator capacity load
 * Param enableOutPeriph : Enable (1U)/Disable (0U) clock to peripherals
 *
 *END**************************************************************************/
static void CLOCK_CONFIG_SetRtcClock(uint32_t capLoad, uint8_t enableOutPeriph)
{
  /* RTC clock gate enable */
  CLOCK_EnableClock(kCLOCK_Rtc0);
  if ((RTC->CR & RTC_CR_OSCE_MASK) == 0u) { /* Only if the Rtc oscillator is not already enabled */
    /* Set the specified capacitor configuration for the RTC oscillator */
    RTC_SetOscCapLoad(RTC, capLoad);
    /* Enable the RTC 32KHz oscillator */
    RTC->CR |= RTC_CR_OSCE_MASK;
  }
  /* Output to other peripherals */
  if (enableOutPeriph) {
    RTC->CR &= ~RTC_CR_CLKO_MASK;
  }
  else {
    RTC->CR |= RTC_CR_CLKO_MASK;
  }
  /* Set the XTAL32/RTC_CLKIN frequency based on board setting. */
  CLOCK_SetXtal32Freq(BOARD_XTAL32K_CLK_HZ);
  /* Set RTC_TSR if there is fault value in RTC */
  if (RTC->SR & RTC_SR_TIF_MASK) {
    RTC -> TSR = RTC -> TSR;
  }
  /* RTC clock gate disable */
  CLOCK_DisableClock(kCLOCK_Rtc0);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_CONFIG_SetFllExtRefDiv
 * Description   : Configure FLL external reference divider (FRDIV).
 * Param frdiv   : The value to set FRDIV.
 *
 *END**************************************************************************/
static void CLOCK_CONFIG_SetFllExtRefDiv(uint8_t frdiv)
{
    MCG->C1 = ((MCG->C1 & ~MCG_C1_FRDIV_MASK) | MCG_C1_FRDIV(frdiv));
}

static const osc_config_t osc_config = {
	.freq = CONFIG_OSC_XTAL0_FREQ,
	.capLoad = 0,

#if defined(CONFIG_OSC_EXTERNAL)
	.workMode = kOSC_ModeExt,
#elif defined(CONFIG_OSC_LOW_POWER)
	.workMode = kOSC_ModeOscLowPower,
#elif defined(CONFIG_OSC_HIGH_GAIN)
	.workMode = kOSC_ModeOscHighGain,
#else
#error "An oscillator mode must be defined"
#endif

	.oscerConfig = {
		.enableMode = kOSC_ErClkEnable,
#if (defined(FSL_FEATURE_OSC_HAS_EXT_REF_CLOCK_DIVIDER) && \
	FSL_FEATURE_OSC_HAS_EXT_REF_CLOCK_DIVIDER)
		.erclkDiv = 0U,
#endif
	},
};

static const mcg_pll_config_t pll0_config = {
	.enableMode = 0U,
	.prdiv = CONFIG_MCG_PRDIV0,
	.vdiv = CONFIG_MCG_VDIV0,
};

static const sim_clock_config_t sim_config = {
	/* PLLFLLSEL: select PLL. */
	.pllFllSel = PERIPH_CLK_MCGIRCLK,
	/* ERCLK32K selection: use system oscillator. */
	.er32kSrc = SIM_OSC32KSEL_RTC32KCLK_CLK,
	.clkdiv1 = SIM_CLKDIV1_OUTDIV1(CONFIG_K8X_CORE_CLOCK_DIVIDER - 1) |
		   SIM_CLKDIV1_OUTDIV2(CONFIG_K8X_BUS_CLOCK_DIVIDER - 1) |
		   SIM_CLKDIV1_OUTDIV3(CONFIG_K8X_FLEXBUS_CLOCK_DIVIDER - 1) |
		   SIM_CLKDIV1_OUTDIV4(CONFIG_K8X_FLASH_CLOCK_DIVIDER - 1),
};

static ALWAYS_INLINE void clk_init(void)
{
	CLOCK_SetSimSafeDivs();

    /* Configure RTC clock including enabling RTC oscillator. */
    CLOCK_CONFIG_SetRtcClock(RTC_OSC_CAP_LOAD_0PF, RTC_RTC32KCLK_PERIPHERALS_ENABLED);

	CLOCK_InitOsc0(&osc_config);
	CLOCK_SetXtal0Freq(CONFIG_OSC_XTAL0_FREQ);

    /* Configure FLL external reference divider (FRDIV). */
    CLOCK_CONFIG_SetFllExtRefDiv(BOARD_FRDIV);

	CLOCK_BootToPeeMode(kMCG_OscselIrc, kMCG_PllClkSelPll0, &pll0_config);

	CLOCK_SetInternalRefClkConfig(kMCG_IrclkEnable, kMCG_IrcSlow,
				      CONFIG_MCG_FCRDIV);

	CLOCK_SetSimConfig(&sim_config);

    /* Set RTC_CLKOUT source. */
    CLOCK_SetRtcClkOutClock(SIM_RTC_CLKOUT_SEL_RTC32KCLK_CLK);

    /* Set SDHC clock source. */
    CLOCK_SetSdhc0Clock(SIM_SDHC_CLK_SEL_CORE_SYSTEM_CLK);
    /* Set LPUART clock source. */
    CLOCK_SetLpuartClock(SIM_LPUART_CLK_SEL_PLLFLLSEL_CLK);
    /* Set FLEXIO clock source. */
    CLOCK_SetFlexio0Clock(SIM_FLEXIO_CLK_SEL_OSCERCLK_CLK);
    /* Set CLKOUT source. */
    CLOCK_SetClkOutClock(SIM_CLKOUT_SEL_FLEXBUS_CLK);

	/* Divide PLL output frequency by 2 for peripherals */
	CLOCK_SetPllFllSelClock(PLLFLLSEL_MCGPLLCLK, 1, 0);

#if CONFIG_UART_MCUX_LPUART
	CLOCK_SetLpuartClock(PERIPH_CLK_PLLFLLSEL);
#endif

#if CONFIG_USB_KINETIS
	CLOCK_EnableUsbfs0Clock(kCLOCK_UsbSrcPll0, 120000000UL);
#endif
}

static int k8x_init(struct device *arg)
{
	ARG_UNUSED(arg);

	unsigned int old_level; /* old interrupt lock level */
#if !defined(CONFIG_ARM_MPU)
	u32_t temp_reg;
#endif /* !CONFIG_ARM_MPU */

	/* Disable interrupts */
	old_level = irq_lock();

	/* release I/O power hold to allow normal run state */
	PMC->REGSC |= PMC_REGSC_ACKISO_MASK;

#if !defined(CONFIG_ARM_MPU)
	/*
	 * Disable memory protection and clear slave port errors.
	 * Note that the K8x does not implement the optional ARMv7-M memory
	 * protection unit (MPU), specified by the architecture (PMSAv7), in the
	 * Cortex-M4 core.  Instead, the processor includes its own MPU module.
	 */
	temp_reg = SYSMPU->CESR;
	temp_reg &= ~SYSMPU_CESR_VLD_MASK;
	temp_reg |= SYSMPU_CESR_SPERR_MASK;
	SYSMPU->CESR = temp_reg;
#endif /* !CONFIG_ARM_MPU */

	/* Initialize system clocks and PLL */
	clk_init();

	/*
	 * Install default handler that simply resets the CPU if
	 * configured in the kernel, NOP otherwise
	 */
	NMI_INIT();

	/* Restore interrupt state */
	irq_unlock(old_level);

	return 0;
}

SYS_INIT(k8x_init, PRE_KERNEL_1, 0);
