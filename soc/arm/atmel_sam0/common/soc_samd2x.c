/*
 * Copyright (c) 2017 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Atmel SAMD MCU series initialization code
 */

#include <arch/cpu.h>
#include <arch/arm/aarch32/cortex_m/cmsis.h>
#include <device.h>
#include <init.h>
#include <kernel.h>
#include <soc.h>

static void flash_waitstates_init(void)
{
	/* One wait state at 48 MHz. */
	NVMCTRL->CTRLB.bit.RWS = NVMCTRL_CTRLB_RWS_HALF_Val;
}

/* Follows the same structure as the Arduino Zero, namely:
 *  XOSC32K -> GCLK1 -> DFLL48M -> GCLK0
 *  OSC8M -> 8 MHz -> GCLK3
 */

static void xosc_init(void)
{
#ifdef CONFIG_SOC_ATMEL_SAMD_XOSC
#error External oscillator support is not implemented.
#endif
}

static void wait_gclk_synchronization(void)
{
	while (GCLK->STATUS.bit.SYNCBUSY) {
	}
}

static void xosc32k_init(void)
{
	SYSCTRL->XOSC32K.reg = SYSCTRL_XOSC32K_STARTUP(6) |
			       SYSCTRL_XOSC32K_XTALEN | SYSCTRL_XOSC32K_EN32K;

	SYSCTRL->XOSC32K.bit.ENABLE = 1;
	/* Wait for the crystal to stabalise. */
	while (!SYSCTRL->PCLKSR.bit.XOSC32KRDY) {
	}
}

static void osc32k_init(void)
{
#ifdef FUSES_OSC32K_CAL_ADDR
	uint32_t fuse = *(uint32_t *)FUSES_OSC32K_CAL_ADDR;
	uint32_t calib = (fuse & FUSES_OSC32K_CAL_Msk) >> FUSES_OSC32K_CAL_Pos;
#else
	uint32_t fuse = *(uint32_t *)FUSES_OSC32KCAL_ADDR;
	uint32_t calib = (fuse & FUSES_OSC32KCAL_Msk) >> FUSES_OSC32KCAL_Pos;
#endif

	SYSCTRL->OSC32K.reg = SYSCTRL_OSC32K_CALIB(calib) |
			      SYSCTRL_OSC32K_STARTUP(0x6u) |
			      SYSCTRL_OSC32K_EN32K | SYSCTRL_OSC32K_ENABLE;

	/* Wait for the oscillator to stabalise. */
	while (!SYSCTRL->PCLKSR.bit.OSC32KRDY) {
	}
}

static void dfll_init(void)
{
	/* No prescaler */
	GCLK->GENDIV.reg = GCLK_GENDIV_ID(1) | GCLK_GENDIV_DIV(0);
	wait_gclk_synchronization();


#if defined(CONFIG_SOC_ATMEL_SAMD_XOSC32K_AS_MAIN)
	/* Route XOSC32K to GCLK1 */
	GCLK->GENCTRL.reg =
	    GCLK_GENCTRL_ID(1) | GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_GENEN;
#elif defined(CONFIG_SOC_ATMEL_SAMD_OSC8M_AS_MAIN)
	/* Route OSC8M to GCLK1 */
	GCLK->GENCTRL.reg =
	    GCLK_GENCTRL_ID(1) | GCLK_GENCTRL_SRC_OSC8M | GCLK_GENCTRL_GENEN;
#elif defined(CONFIG_SOC_ATMEL_SAMD_OSC32K_AS_MAIN)
	/* Route OSC8M to GCLK1 */
	GCLK->GENCTRL.reg =
	    GCLK_GENCTRL_ID(1) | GCLK_GENCTRL_SRC_OSC32K | GCLK_GENCTRL_GENEN;
#else
#error Unsupported main clock source.
#endif

	wait_gclk_synchronization();

	/* Route GCLK1 to multiplexer 0 */
	GCLK->CLKCTRL.reg =
	    GCLK_CLKCTRL_ID(0) | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_CLKEN;
	wait_gclk_synchronization();

	SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE;
	while (!SYSCTRL->PCLKSR.bit.DFLLRDY) {
	}

	uint32_t mul = (SOC_ATMEL_SAM0_MCK_FREQ_HZ +
		     SOC_ATMEL_SAM0_GCLK1_FREQ_HZ / 2) /
		    SOC_ATMEL_SAM0_GCLK1_FREQ_HZ;

	SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_CSTEP(31) |
			       SYSCTRL_DFLLMUL_FSTEP(511) |
			       SYSCTRL_DFLLMUL_MUL(mul);
	while (!SYSCTRL->PCLKSR.bit.DFLLRDY) {
	}


	// 32K xosc code

	#if defined(CONFIG_SOC_ATMEL_SAMD_XOSC32K_AS_MAIN) || defined(CONFIG_SOC_ATMEL_SAMD_OSC8M_AS_MAIN)

	SYSCTRL->DFLLCTRL.reg |= SYSCTRL_DFLLCTRL_MODE |
#ifdef SYSCTRL_DFLLCTRL_WAITLOCK
				 SYSCTRL_DFLLCTRL_WAITLOCK |
#endif
				 SYSCTRL_DFLLCTRL_QLDIS;
	while (!SYSCTRL->PCLKSR.bit.DFLLRDY) {
	}

	/* Enable the DFLL */
	SYSCTRL->DFLLCTRL.bit.ENABLE = 1;

	while (!SYSCTRL->PCLKSR.bit.DFLLLCKC || !SYSCTRL->PCLKSR.bit.DFLLLCKF) {
	}
	#elif defined(CONFIG_SOC_ATMEL_SAMD_OSC32K_AS_MAIN)

	 #define NVM_SW_CALIB_DFLL48M_COARSE_VAL 58

  // Turn on DFLL
	uint32_t coarse =( *((uint32_t *)(NVMCTRL_OTP4) + (NVM_SW_CALIB_DFLL48M_COARSE_VAL / 32)) >> (NVM_SW_CALIB_DFLL48M_COARSE_VAL % 32) )
			& ((1 << 6) - 1);
	if (coarse == 0x3f) {
	coarse = 0x1f;
	}
	// TODO(tannewt): Load this value from memory we've written previously. There
	// isn't a value from the Atmel factory.
	uint32_t fine = 0x1ff;

	SYSCTRL->DFLLVAL.bit.COARSE = coarse;
	SYSCTRL->DFLLVAL.bit.FINE = fine;
	/* Write full configuration to DFLL control register */
	SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_CSTEP( 0x1f / 4 ) | // Coarse step is 31, half of the max value
				SYSCTRL_DFLLMUL_FSTEP( 10 ) |
				SYSCTRL_DFLLMUL_MUL( (48000) ) ;

	SYSCTRL->DFLLCTRL.reg = 0;

	while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0 )
	{
	/* Wait for synchronization */
	}

	SYSCTRL->DFLLCTRL.reg =  SYSCTRL_DFLLCTRL_MODE |
				SYSCTRL_DFLLCTRL_CCDIS |
				SYSCTRL_DFLLCTRL_USBCRM | /* USB correction */
				SYSCTRL_DFLLCTRL_BPLCKC;

	while (!SYSCTRL->PCLKSR.bit.DFLLRDY) {
	}

	/* Enable the DFLL */
	SYSCTRL->DFLLCTRL.bit.ENABLE = 1;


	#else
	#error Unsupported main clock source.
	#endif

	// end xosc code

	while (!SYSCTRL->PCLKSR.bit.DFLLRDY) {
	}
}

static void osc8m_init(void)
{
	/* Turn off the prescaler */
	SYSCTRL->OSC8M.bit.PRESC = SYSCTRL_OSC8M_PRESC(0);
	SYSCTRL->OSC8M.bit.ONDEMAND = 0;
}

static void gclks_init(void)
{
	/* DFLL/1 -> GCLK0 */
	GCLK->GENDIV.reg = GCLK_GENDIV_ID(0) | GCLK_GENDIV_DIV(0);
	wait_gclk_synchronization();

	GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_SRC_DFLL48M |
			    GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN;
	wait_gclk_synchronization();

	/* OSC8M/1 -> GCLK3 */
	GCLK->GENCTRL.reg =
	    GCLK_GENCTRL_ID(3) | GCLK_GENCTRL_SRC_OSC8M | GCLK_GENCTRL_GENEN;
	wait_gclk_synchronization();

	/* OSCULP32K/32 -> GCLK2 */
	GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(32 - 1);
	wait_gclk_synchronization();

	GCLK->GENCTRL.reg =
	    GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_SRC_OSC32K | GCLK_GENCTRL_GENEN;
	wait_gclk_synchronization();
}

static void dividers_init(void)
{
	/* Set the CPU, APBA, B, and C dividers */
	PM->CPUSEL.reg = PM_CPUSEL_CPUDIV_DIV1;
	PM->APBASEL.reg = PM_APBASEL_APBADIV_DIV1_Val;
	PM->APBBSEL.reg = PM_APBBSEL_APBBDIV_DIV1_Val;
	PM->APBCSEL.reg = PM_APBCSEL_APBCDIV_DIV1_Val;

	/* TODO(mlhx): enable clock failure detection? */
}

static void enable_bod33() {
	/* Disable the brown-out detector during configuration,
	otherwise it might misbehave and reset the
	microcontroller. */
	SYSCTRL->BOD33.bit.ENABLE = 0;
	while (!SYSCTRL->PCLKSR.bit.B33SRDY) {};

	/* Configure the brown-out detector so that the
	program can use it to watch the power supply
	voltage */
	SYSCTRL->BOD33.reg = (
	/* This sets the minimum voltage level to 3.0v - 3.2v.
	See datasheet table 37-21. */
	SYSCTRL_BOD33_LEVEL(48) |
	/* Since the program is waiting for the voltage to rise,
	don't reset the microcontroller if the voltage is too
	low. */
	SYSCTRL_BOD33_ACTION_NONE |
	/* Enable hysteresis to better deal with noisy power
	supplies and voltage transients. */
	SYSCTRL_BOD33_HYST);

	/* Enable the brown-out detector and then wait for
	the voltage level to settle. */
	SYSCTRL->BOD33.bit.ENABLE = 1;
	while (!SYSCTRL->PCLKSR.bit.BOD33RDY) {}

	/* BOD33DET is set when the voltage is *too low*,
	so wait for it to be cleared. */
	while (SYSCTRL->PCLKSR.bit.BOD33DET) {}

	/* Let the brown-out detector automatically reset the microcontroller
	if the voltage drops too low. */
	SYSCTRL->BOD33.bit.ENABLE = 0;
	while (!SYSCTRL->PCLKSR.bit.B33SRDY) {};

	SYSCTRL->BOD33.reg |= SYSCTRL_BOD33_ACTION_RESET;

	SYSCTRL->BOD33.bit.ENABLE = 1;
}

static int atmel_samd_init(const struct device *arg)
{
	uint32_t key;

	ARG_UNUSED(arg);

	key = irq_lock();

#if defined(CONFIG_SOC_ATMEL_SAMD_ENABLE_BOD33)
	enable_bod33();
#endif

	flash_waitstates_init();
	osc8m_init();
	osc32k_init();
	xosc_init();
	#ifdef CONFIG_SOC_ATMEL_SAMD_XOSC32K
	xosc32k_init();
	#endif
	dfll_init();
	gclks_init();
	dividers_init();

	/* Install default handler that simply resets the CPU
	 * if configured in the kernel, NOP otherwise
	 */
	NMI_INIT();

	irq_unlock(key);

	return 0;
}

SYS_INIT(atmel_samd_init, PRE_KERNEL_1, 0);
