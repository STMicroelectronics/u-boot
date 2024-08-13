// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018, STMicroelectronics - All Rights Reserved
 * Author: Fabrice Gasnier <fabrice.gasnier@st.com>
 *
 * Originally based on the Linux kernel v4.18 drivers/iio/adc/stm32-adc.c.
 */

#include <common.h>
#include <adc.h>
#include <dm.h>
#include <env.h>
#include <asm/io.h>
#include <dm/device_compat.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <power/regulator.h>
#include "stm32-adc-core.h"

/* STM32H7 - Registers for each ADC instance */
#define STM32H7_ADC_ISR			0x00
#define STM32H7_ADC_CR			0x08
#define STM32H7_ADC_CFGR		0x0C
#define STM32H7_ADC_SMPR1		0x14
#define STM32H7_ADC_SMPR2		0x18
#define STM32H7_ADC_PCSEL		0x1C
#define STM32H7_ADC_SQR1		0x30
#define STM32H7_ADC_DR			0x40
#define STM32H7_ADC_DIFSEL		0xC0
#define STM32H7_ADC_CALFACT		0xC4
#define STM32H7_ADC_CALFACT2		0xC8

/* STM32MP25 - Registers for each ADC instance */
#define STM32MP25_ADC_CALFACT STM32H7_ADC_CALFACT

/* STM32H7_ADC_ISR - bit fields */
#define STM32MP1_VREGREADY		BIT(12)
#define STM32H7_EOC			BIT(2)
#define STM32H7_ADRDY			BIT(0)

/* STM32H7_ADC_CR - bit fields */
#define STM32H7_ADCAL			BIT(31)
#define STM32H7_ADCALDIF		BIT(30)
#define STM32H7_DEEPPWD			BIT(29)
#define STM32H7_ADVREGEN		BIT(28)
#define STM32H7_LINCALRDYW6		BIT(27)
#define STM32H7_LINCALRDYW5		BIT(26)
#define STM32H7_LINCALRDYW4		BIT(25)
#define STM32H7_LINCALRDYW3		BIT(24)
#define STM32H7_LINCALRDYW2		BIT(23)
#define STM32H7_LINCALRDYW1		BIT(22)
#define STM32H7_ADCALLIN		BIT(16)
#define STM32H7_BOOST			BIT(8)
#define STM32H7_ADSTP			BIT(4)
#define STM32H7_ADSTART			BIT(2)
#define STM32H7_ADDIS			BIT(1)
#define STM32H7_ADEN			BIT(0)

/* STM32MP25_ADC_CFGR - bit fields */
#define STM32MP25_RES_MASK		GENMASK(3, 2)

/* STM32MP25_ADC_CALFACT - bit fields */
#define STM32MP25_CALADDOS		BIT(31)
#define STM32MP25_CALFACT_S		GENMASK(8, 0)
#define STM32MP25_CALFACT_D		GENMASK(24, 16)

/* STM32H7_ADC_CALFACT2 - bit fields */
#define STM32H7_LINCALFACT_SHIFT	0
#define STM32H7_LINCALFACT_MASK		GENMASK(29, 0)

/* STM32H7_ADC_CFGR bit fields */
#define STM32H7_EXTEN			GENMASK(11, 10)
#define STM32H7_DMNGT			GENMASK(1, 0)

/* STM32H7_ADC_SQR1 - bit fields */
#define STM32H7_SQ1_SHIFT		6

/* STM32H7_ADC_DIFSEL - bit fields */
#define STM32H7_DIFSEL_SHIFT	0
#define STM32H7_DIFSEL_MASK		GENMASK(19, 0)

/* BOOST bit must be set on STM32H7 when ADC clock is above 20MHz */
#define STM32H7_BOOST_CLKRATE		20000000UL

/* STM32MP13 - Registers for each ADC instance */
#define STM32MP13_ADC_DIFSEL	0xB0

/* STM32MP13_ADC_CFGR specific bit fields */
#define STM32MP13_DMAEN			BIT(0)
#define STM32MP13_DMACFG		BIT(1)

/* STM32MP13_ADC_DIFSEL - bit fields */
#define STM32MP13_DIFSEL_SHIFT	0
#define STM32MP13_DIFSEL_MASK	GENMASK(18, 0)

#define STM32_ADC_CH_MAX		20	/* max number of channels */
#define STM32_ADC_TIMEOUT_US		100000
/* Number of linear calibration shadow registers / LINCALRDYW control bits */
#define STM32H7_LINCALFACT_NUM		6
#define STM32H7_LINCAL_NAME_LEN		32

/* Number of loops in the calibration procedure to average data on STM32MP25 */
#define STM32MP25_CALIB_LOOP		8

struct stm32_adc_cfg {
	const struct stm32_adc_regspec	*regs;
	unsigned int max_channels;
	unsigned int num_bits;
	int (*calib)(struct udevice *dev);
	bool has_vregready;
	bool has_boostmode;
	bool has_linearcal;
	bool has_presel;
};

struct stm32_adc {
	void __iomem *regs;
	int active_channel;
	const struct stm32_adc_cfg *cfg;
	u32 lincalfact[STM32H7_LINCALFACT_NUM];
};

struct stm32_adc_regs {
	int reg;
	int mask;
	int shift;
};

struct stm32_adc_regspec {
	const struct stm32_adc_regs difsel;
};

static const struct stm32_adc_regspec stm32h7_adc_regspec = {
	.difsel = { STM32H7_ADC_DIFSEL, STM32H7_DIFSEL_MASK },
};

static const struct stm32_adc_regspec stm32mp13_adc_regspec = {
	.difsel = { STM32MP13_ADC_DIFSEL, STM32MP13_DIFSEL_MASK },
};
static void stm32_adc_enter_pwr_down(struct udevice *dev)
{
	struct stm32_adc *adc = dev_get_priv(dev);

	if (adc->cfg->has_boostmode)
		clrbits_le32(adc->regs + STM32H7_ADC_CR, STM32H7_BOOST);

	/* Setting DEEPPWD disables ADC vreg and clears ADVREGEN */
	setbits_le32(adc->regs + STM32H7_ADC_CR, STM32H7_DEEPPWD);
}

static int stm32_adc_exit_pwr_down(struct udevice *dev)
{
	struct stm32_adc_common *common = dev_get_priv(dev_get_parent(dev));
	struct stm32_adc *adc = dev_get_priv(dev);
	int ret;
	u32 val;

	/* return immediately if ADC is not in deep power down mode */
	if (!(readl(adc->regs + STM32H7_ADC_CR) & STM32H7_DEEPPWD))
		return 0;

	/* Exit deep power down, then enable ADC voltage regulator */
	clrbits_le32(adc->regs + STM32H7_ADC_CR, STM32H7_DEEPPWD);
	setbits_le32(adc->regs + STM32H7_ADC_CR, STM32H7_ADVREGEN);
	if (adc->cfg->has_boostmode && common->rate > STM32H7_BOOST_CLKRATE)
		setbits_le32(adc->regs + STM32H7_ADC_CR, STM32H7_BOOST);

	/* Wait for startup time */
	if (!adc->cfg->has_vregready) {
		udelay(20);
		return 0;
	}

	ret = readl_poll_timeout(adc->regs + STM32H7_ADC_ISR, val,
				 val & STM32MP1_VREGREADY,
				 STM32_ADC_TIMEOUT_US);
	if (ret < 0) {
		stm32_adc_enter_pwr_down(dev);
		dev_err(dev, "Failed to enable vreg: %d\n", ret);
	}

	return ret;
}

static int stm32_adc_stop(struct udevice *dev)
{
	struct stm32_adc *adc = dev_get_priv(dev);

	setbits_le32(adc->regs + STM32H7_ADC_CR, STM32H7_ADDIS);
	stm32_adc_enter_pwr_down(dev);
	adc->active_channel = -1;

	return 0;
}

static int stm32_adc_enable(struct udevice *dev)
{
	struct stm32_adc *adc = dev_get_priv(dev);
	int ret;
	u32 val;

	setbits_le32(adc->regs + STM32H7_ADC_CR, STM32H7_ADEN);
	ret = readl_poll_timeout(adc->regs + STM32H7_ADC_ISR, val,
				 val & STM32H7_ADRDY, STM32_ADC_TIMEOUT_US);
	if (ret < 0) {
		stm32_adc_stop(dev);
		dev_err(dev, "Failed to enable ADC: %d\n", ret);
	}

	return ret;
}

static int stm32_adc_start_channel(struct udevice *dev, int channel)
{
	struct adc_uclass_plat *uc_pdata = dev_get_uclass_plat(dev);
	struct stm32_adc *adc = dev_get_priv(dev);
	int ret;

	ret = stm32_adc_exit_pwr_down(dev);
	if (ret < 0)
		return ret;

	/* Only use single ended channels */
	clrbits_le32(adc->regs + adc->cfg->regs->difsel.reg, adc->cfg->regs->difsel.mask);

	ret = stm32_adc_enable(dev);
	if (ret)
		return ret;

	/* Preselect channels */
	if (adc->cfg->has_presel)
		writel(uc_pdata->channel_mask, adc->regs + STM32H7_ADC_PCSEL);

	/* Set sampling time to max value by default */
	writel(0xffffffff, adc->regs + STM32H7_ADC_SMPR1);
	writel(0xffffffff, adc->regs + STM32H7_ADC_SMPR2);

	/* Program regular sequence: chan in SQ1 & len = 0 for one channel */
	writel(channel << STM32H7_SQ1_SHIFT, adc->regs + STM32H7_ADC_SQR1);

	/*
	 * Trigger detection disabled (conversion can be launched in SW)
	 * STM32H7_DMNGT is equivalent to STM32MP13_DMAEN & STM32MP13_DMACFG
	 */
	clrbits_le32(adc->regs + STM32H7_ADC_CFGR, STM32H7_EXTEN | STM32H7_DMNGT);
	adc->active_channel = channel;

	return 0;
}

static int stm32_adc_channel_data(struct udevice *dev, int channel,
				  unsigned int *data)
{
	struct stm32_adc *adc = dev_get_priv(dev);
	int ret;
	u32 val;

	if (channel != adc->active_channel) {
		dev_err(dev, "Requested channel is not active!\n");
		return -EINVAL;
	}

	setbits_le32(adc->regs + STM32H7_ADC_CR, STM32H7_ADSTART);
	ret = readl_poll_timeout(adc->regs + STM32H7_ADC_ISR, val,
				 val & STM32H7_EOC, STM32_ADC_TIMEOUT_US);
	if (ret < 0) {
		dev_err(dev, "conversion timed out: %d\n", ret);
		return ret;
	}

	*data = readl(adc->regs + STM32H7_ADC_DR);

	ret = readl_poll_timeout(adc->regs + STM32H7_ADC_CR, val,
				 !(val & (STM32H7_ADSTART)), STM32_ADC_TIMEOUT_US);
	if (ret)
		dev_warn(dev, "conversion stop timed out\n");

	if (adc->cfg->has_presel)
		setbits_le32(adc->regs + STM32H7_ADC_PCSEL, 0);

	return ret;
}

/**
 * Fixed timeout value for ADC calibration.
 * worst cases:
 * - low clock frequency (0.12 MHz min)
 * - maximum prescalers
 * Calibration requires:
 * - 16384 ADC clock cycle for the linear calibration
 * - 20 ADC clock cycle for the offset calibration
 *
 * Set to 100ms for now
 */
#define STM32H7_ADC_CALIB_TIMEOUT_US		100000

static int stm32_adc_run_selfcalib(struct udevice *dev, int do_lincal)
{
	struct stm32_adc *adc = dev_get_priv(dev);
	int ret;
	u32 val, mask;

	/*
	 * Select calibration mode:
	 * - Offset calibration for single ended inputs
	 * - No linearity calibration.
	 */
	clrbits_le32(adc->regs + STM32H7_ADC_CR, STM32H7_ADCALDIF | STM32H7_ADCALLIN);

	/* Start calibration, then wait for completion */
	setbits_le32(adc->regs + STM32H7_ADC_CR, STM32H7_ADCAL);
	ret = readl_poll_sleep_timeout(adc->regs + STM32H7_ADC_CR, val,
				       !(val & STM32H7_ADCAL), 100,
				       STM32H7_ADC_CALIB_TIMEOUT_US);
	if (ret) {
		dev_err(dev, "calibration (offset single-ended) failed\n");
		goto out;
	}

	/*
	 * Select calibration mode, then start calibration:
	 * - Offset calibration for differential input
	 * - Linearity calibration if not already done.
	 *   will run simultaneously with offset calibration.
	 */
	mask = STM32H7_ADCALDIF;
	if (adc->cfg->has_linearcal && do_lincal)
		mask |= STM32H7_ADCALLIN;
	setbits_le32(adc->regs + STM32H7_ADC_CR, mask);

	/* Start calibration, then wait for completion */
	setbits_le32(adc->regs + STM32H7_ADC_CR, STM32H7_ADCAL);
	ret = readl_poll_sleep_timeout(adc->regs + STM32H7_ADC_CR, val,
				       !(val & STM32H7_ADCAL), 100,
				       STM32H7_ADC_CALIB_TIMEOUT_US);
	if (ret)
		dev_err(dev, "calibration (offset diff%s) failed\n",
			(mask & STM32H7_ADCALLIN) ? "+linear" : "");

out:
	clrbits_le32(adc->regs + STM32H7_ADC_CR, STM32H7_ADCALDIF | STM32H7_ADCALLIN);

	return ret;
}

static int stm32mp25_adc_calfact_data(struct udevice *dev, u32 *average)
{
	struct stm32_adc *adc = dev_get_priv(dev);
	u32 val, avg = 0;
	int i, ret;

	for (i = 0; i < STM32MP25_CALIB_LOOP; i++) {
		setbits_le32(adc->regs + STM32H7_ADC_CR, STM32H7_ADSTART);

		/*
		 * Wait for end of conversion by polling ADSTART bit until it is cleared
		 * Also work if waiting for EOC to be set in STM32H7_ADC_ISR
		 */
		ret = readl_poll_sleep_timeout(adc->regs + STM32H7_ADC_CR, val,
					       !(val & STM32H7_ADSTART), 100, STM32_ADC_TIMEOUT_US);
		if (ret < 0) {
			dev_err(dev, "Conversion failed: %d\n", ret);
			return ret;
		}

		val = readl(adc->regs + STM32H7_ADC_DR);
		avg += val;
	}

	*average = DIV_ROUND_CLOSEST(avg, STM32MP25_CALIB_LOOP);

	return 0;
}

static int stm32mp25_adc_run_calib(struct udevice *dev)
{
	struct stm32_adc *adc = dev_get_priv(dev);
	u32 average_data = 0, calfact = 0;
	int ret;

	setbits_le32(adc->regs + STM32H7_ADC_CR, STM32H7_ADCAL);
	/* Clears CALADDOS (and old calibration data if any) */
	writel_relaxed(0, adc->regs + STM32MP25_ADC_CALFACT);
	/* Select single ended input calibration */
	clrbits_le32(adc->regs + STM32H7_ADC_CR, STM32H7_ADCALDIF);
	/* Use default resolution (e.g. 12 bits) */
	clrbits_le32(adc->regs + STM32H7_ADC_CFGR, STM32MP25_RES_MASK);

retry:
	ret = stm32mp25_adc_calfact_data(dev, &average_data);
	if (ret)
		goto out;

	/* If the averaged data is zero, retry with additional offset (set CALADDOS) */
	if (!average_data) {
		if (!calfact) {
			/* Averaged data is zero, retry with additional offset */
			calfact = STM32MP25_CALADDOS;
			writel_relaxed(STM32MP25_CALADDOS,
				       adc->regs + STM32MP25_ADC_CALFACT);
			goto retry;
		}
		/* Averaged data is still zero with additional offset, just warn about it */
		dev_warn(dev, "Single-ended calibration average: 0\n");
	}

	calfact |= FIELD_PREP(STM32MP25_CALFACT_S, average_data);

	/* Select differential input calibration (keep previous CALADDOS value) */
	setbits_le32(adc->regs + STM32H7_ADC_CR, STM32H7_ADCALDIF);

	ret = stm32mp25_adc_calfact_data(dev, &average_data);
	if (ret)
		goto out;
	/*
	 * If the averaged data is below 0x800 (half value in 12-bits mode),
	 * retry with additional offset
	 */
	if (average_data < 0x800) {
		if (!(calfact & STM32MP25_CALADDOS)) {
			/* Retry the whole calibration with additional offset */
			clrbits_le32(adc->regs + STM32H7_ADC_CR, STM32H7_ADCALDIF);
			calfact = STM32MP25_CALADDOS;
			writel(calfact, adc->regs + STM32MP25_ADC_CALFACT);
			goto retry;
		}
		/*
		 * Averaged data is still below center value. It needs to be clamped to zero,
		 * so don't use the result here, warn about it.
		 */
		dev_warn(dev, "Differential calibration clamped(0): 0x%x\n", average_data);
	} else {
		calfact |= FIELD_PREP(STM32MP25_CALFACT_D, average_data);
	}

	writel_relaxed(calfact, adc->regs + STM32MP25_ADC_CALFACT);

	dev_dbg(dev, "Set calfact_s=0x%03lx, calfact_d=0x%03lx, calados=%ld\n",
		FIELD_GET(STM32MP25_CALFACT_S, calfact),
		FIELD_GET(STM32MP25_CALFACT_D, calfact),
		FIELD_GET(STM32MP25_CALADDOS, calfact));

out:
	clrbits_le32(adc->regs + STM32H7_ADC_CR, STM32H7_ADCAL);

	return ret;
}

/* Retrieve calibration data from env variables */
static bool stm32_adc_getenv_selfcalib(struct udevice *dev)
{
	struct stm32_adc *adc = dev_get_priv(dev);
	char env_name[STM32H7_LINCAL_NAME_LEN];
	char *env;
	int i;

	memset(&adc->lincalfact, 0, STM32H7_LINCALFACT_NUM * sizeof(u32));
	for (i = STM32H7_LINCALFACT_NUM - 1; i >= 0; i--) {
		/*
		 * Save ADC linear calibration factors in U-boot environment variables
		 * Variables are instantiated according to the adc address through
		 * adcx_ prefix.
		 */
		snprintf(env_name, sizeof(env_name), "adc%x_lincalfact%d", (u32)adc->regs, i + 1);
		env = env_get(env_name);
		if (!env)
			return false;
		adc->lincalfact[i] = env_get_hex(env_name, 0);
	}
	return true;
}

/* Save calibration data to env variables */
static void stm32_adc_save_selfcalib(struct udevice *dev)
{
	struct stm32_adc *adc = dev_get_priv(dev);
	char env_name[STM32H7_LINCAL_NAME_LEN];
	int i;

	for (i = STM32H7_LINCALFACT_NUM - 1; i >= 0; i--) {
		snprintf(env_name, sizeof(env_name), "adc%x_lincalfact%d", (u32)adc->regs, i + 1);
		if (env_set_hex(env_name, adc->lincalfact[i]))
			dev_warn(dev, "Failed to save %s\n", env_name);
	}
}

/* Read calibration data from ADC */
static int stm32_adc_read_selfcalib(struct udevice *dev)
{
	struct stm32_adc *adc = dev_get_priv(dev);
	u32 lincalrdyw_mask, val;
	int i, ret;

	/* Read linearity calibration */
	lincalrdyw_mask = STM32H7_LINCALRDYW6;
	for (i = STM32H7_LINCALFACT_NUM - 1; i >= 0; i--) {
		/* Clear STM32H7_LINCALRDYW[6..1]: transfer calib to CALFACT2 */
		clrbits_le32(adc->regs + STM32H7_ADC_CR, lincalrdyw_mask);

		/* Poll: wait calib data to be ready in CALFACT2 register */
		ret = readl_poll_sleep_timeout(adc->regs + STM32H7_ADC_CR, val,
					       !(val & lincalrdyw_mask), 100,
					       STM32_ADC_TIMEOUT_US);
		if (ret) {
			dev_err(dev, "Failed to read calfact\n");
			return ret;
		}

		val = readl(adc->regs + STM32H7_ADC_CALFACT2);
		adc->lincalfact[i] = (val & STM32H7_LINCALFACT_MASK);
		adc->lincalfact[i] >>= STM32H7_LINCALFACT_SHIFT;

		lincalrdyw_mask >>= 1;
	}

	return 0;
}

/* Write calibration data to ADC */
static int stm32_adc_write_selfcalib(struct udevice *dev)
{
	struct stm32_adc *adc = dev_get_priv(dev);
	u32 lincalrdyw_mask, val;
	int i, ret;

	lincalrdyw_mask = STM32H7_LINCALRDYW6;
	for (i = STM32H7_LINCALFACT_NUM - 1; i >= 0; i--) {
		/*
		 * Write saved calibration data to shadow registers:
		 * Write CALFACT2, and set LINCALRDYW[6..1] bit to trigger
		 * data write. Then poll to wait for complete transfer.
		 */
		val = adc->lincalfact[i] << STM32H7_LINCALFACT_SHIFT;
		writel(val, adc->regs + STM32H7_ADC_CALFACT2);
		setbits_le32(adc->regs + STM32H7_ADC_CR, lincalrdyw_mask);
		ret = readl_poll_sleep_timeout(adc->regs + STM32H7_ADC_CR, val,
					       val & lincalrdyw_mask,
					       100, STM32_ADC_TIMEOUT_US);
		if (ret) {
			dev_err(dev, "Failed to write calfact\n");
			return ret;
		}

		lincalrdyw_mask >>= 1;
	}

	return 0;
}

static int stm32_adc_selfcalib(struct udevice *dev)
{
	struct stm32_adc *adc = dev_get_priv(dev);
	struct stm32_adc_common *common = dev_get_priv(dev_get_parent(dev));
	int ret;
	bool lincal_done = false;

	ret = regulator_set_enable_if_allowed(common->vdda, true);
	if (ret) {
		dev_err(dev, "Failed to enable vdd_supply: %s",
			common->vdda->name);
		return ret;
	}

	ret = regulator_set_enable_if_allowed(common->vref, true);
	if (ret) {
		dev_err(dev, "Failed to enable Vref: %s", common->vref->name);
		return ret;
	}

	/* Try to restore linear calibration */
	if (adc->cfg->has_linearcal)
		lincal_done = stm32_adc_getenv_selfcalib(dev);

	/*
	 * Run offset calibration unconditionally.
	 * Run linear calibration if not already available.
	 */
	ret = stm32_adc_run_selfcalib(dev, !lincal_done);
	if (ret)
		return ret;

	ret = stm32_adc_enable(dev);
	if (ret)
		return ret;

	if (adc->cfg->has_linearcal) {
		if (!lincal_done) {
			ret = stm32_adc_read_selfcalib(dev);
			if (ret)
				goto disable;

			stm32_adc_save_selfcalib(dev);
		}

		/*
		 * Always write linear calibration data to ADC.
		 * This allows to ensure that LINCALRDYWx bits are set when entering kernel
		 *
		 * - First boot:
		 *   U-boot performs ADC linear calibration (& offset calibration)
		 *   U-boot reads & saves linear calibration result in environment variable
		 *   (Here LINCALRDYWx have been cleared due to the read procedure)
		 *   U-boot writes back ADC linear calibration to set LINCALRDYWx bits,
		 *   making the linear calibration available for the kernel.
		 *
		 * - Subsequent boot (environment set earlier):
		 *   U-boot performs ADC offset calibration only
		 *   U-boot reads ADC linear calibration from environment variable
		 *   and writes back ADC linear calibration.
		 *
		 * - All boot: kernel steps
		 *   * Case1: ADC calibrated by U-boot (LINCALRDYWx bits set)
		 *     Read back the linear calibration from ADC registers and save it.
		 *   * Case2: ADC not calibrated by U-boot
		 *     Run a linear calibration and save it.
		 */
		ret = stm32_adc_write_selfcalib(dev);
		if (ret)
			goto disable;
	}

	return ret;

disable:
	stm32_adc_stop(dev);
	return ret;
}

static int stm32mp25_adc_calib(struct udevice *dev)
{
	struct stm32_adc_common *common = dev_get_priv(dev_get_parent(dev));
	int ret;

	ret = regulator_set_enable_if_allowed(common->vdda, true);
	if (ret) {
		dev_err(dev, "Failed to enable vdd_supply: %s",
			common->vdda->name);
		return ret;
	}

	ret = regulator_set_enable_if_allowed(common->vref, true);
	if (ret) {
		dev_err(dev, "Failed to enable Vref: %s", common->vref->name);
		return ret;
	}

	ret = stm32_adc_enable(dev);
	if (ret < 0)
		return ret;

	/* Run offset calibration unconditionally. */
	ret = stm32mp25_adc_run_calib(dev);
	if (ret)
		stm32_adc_stop(dev);

	return ret;
}

static int stm32_adc_get_legacy_chan_count(struct udevice *dev)
{
	int ret;

	/* Retrieve single ended channels listed in device tree */
	ret = dev_read_size(dev, "st,adc-channels");
	if (ret < 0) {
		dev_err(dev, "can't get st,adc-channels: %d\n", ret);
		return ret;
	}

	return (ret / sizeof(u32));
}

static int stm32_adc_legacy_chan_init(struct udevice *dev, unsigned int num_channels)
{
	struct adc_uclass_plat *uc_pdata = dev_get_uclass_plat(dev);
	struct stm32_adc *adc = dev_get_priv(dev);
	u32 chans[STM32_ADC_CH_MAX];
	int i, ret;

	ret = dev_read_u32_array(dev, "st,adc-channels", chans, num_channels);
	if (ret < 0) {
		dev_err(dev, "can't read st,adc-channels: %d\n", ret);
		return ret;
	}

	for (i = 0; i < num_channels; i++) {
		if (chans[i] >= adc->cfg->max_channels) {
			dev_err(dev, "bad channel %u\n", chans[i]);
			return -EINVAL;
		}
		uc_pdata->channel_mask |= 1 << chans[i];
	}

	return ret;
}

static int stm32_adc_generic_chan_init(struct udevice *dev, unsigned int num_channels)
{
	struct adc_uclass_plat *uc_pdata = dev_get_uclass_plat(dev);
	struct stm32_adc *adc = dev_get_priv(dev);
	ofnode child;
	int val, ret;

	ofnode_for_each_subnode(child, dev_ofnode(dev)) {
		ret = ofnode_read_u32(child, "reg", &val);
		if (ret) {
			dev_err(dev, "Missing channel index %d\n", ret);
			return ret;
		}

		if (val >= adc->cfg->max_channels) {
			dev_err(dev, "Invalid channel %d\n", val);
			return -EINVAL;
		}

		uc_pdata->channel_mask |= 1 << val;
	}

	return 0;
}

static int stm32_adc_chan_of_init(struct udevice *dev)
{
	struct adc_uclass_plat *uc_pdata = dev_get_uclass_plat(dev);
	struct stm32_adc *adc = dev_get_priv(dev);
	unsigned int num_channels;
	int ret;
	bool legacy = false;

	num_channels = dev_get_child_count(dev);
	/* If no channels have been found, fallback to channels legacy properties. */
	if (!num_channels) {
		legacy = true;

		ret = stm32_adc_get_legacy_chan_count(dev);
		if (!ret) {
			dev_err(dev, "No channel found\n");
			return -ENODATA;
		} else if (ret < 0) {
			return ret;
		}
		num_channels = ret;
	}

	if (num_channels > adc->cfg->max_channels) {
		dev_err(dev, "too many st,adc-channels: %d\n", num_channels);
		return -EINVAL;
	}

	if (legacy)
		ret = stm32_adc_legacy_chan_init(dev, num_channels);
	else
		ret = stm32_adc_generic_chan_init(dev, num_channels);
	if (ret < 0)
		return ret;

	uc_pdata->data_mask = (1 << adc->cfg->num_bits) - 1;
	uc_pdata->data_format = ADC_DATA_FORMAT_BIN;
	uc_pdata->data_timeout_us = 100000;

	return 0;
}

static int stm32_adc_probe(struct udevice *dev)
{
	struct adc_uclass_plat *uc_pdata = dev_get_uclass_plat(dev);
	struct stm32_adc_common *common = dev_get_priv(dev_get_parent(dev));
	struct stm32_adc *adc = dev_get_priv(dev);
	int offset, ret;

	offset = dev_read_u32_default(dev, "reg", -ENODATA);
	if (offset < 0) {
		dev_err(dev, "Can't read reg property\n");
		return offset;
	}
	adc->regs = common->base + offset;
	adc->cfg = (const struct stm32_adc_cfg *)dev_get_driver_data(dev);

	/*
	 * VDDA and Vref can be two separate ADC supplies.
	 * Provide the VDDA as VDD to the framework, that supplies the ADC.
	 * Provide Vref value used by SAR ADC for conversion (can be different),
	 * the u-class use it for raw to microvolt conversion.
	 */
	uc_pdata->vdd_supply = common->vdda;
	uc_pdata->vdd_microvolts = common->vref_uv;
	uc_pdata->vss_microvolts = 0;

	ret = stm32_adc_chan_of_init(dev);
	if (ret < 0)
		return ret;

	ret = stm32_adc_exit_pwr_down(dev);
	if (ret < 0)
		return ret;

	ret = adc->cfg->calib(dev);
	if (ret)
		stm32_adc_enter_pwr_down(dev);

	return ret;
}

static const struct adc_ops stm32_adc_ops = {
	.start_channel = stm32_adc_start_channel,
	.channel_data = stm32_adc_channel_data,
	.stop = stm32_adc_stop,
};

static const struct stm32_adc_cfg stm32h7_adc_cfg = {
	.regs = &stm32h7_adc_regspec,
	.num_bits = 16,
	.max_channels = STM32_ADC_CH_MAX,
	.calib = &stm32_adc_selfcalib,
	.has_boostmode = true,
	.has_linearcal = true,
	.has_presel = true,
};

static const struct stm32_adc_cfg stm32mp1_adc_cfg = {
	.regs = &stm32h7_adc_regspec,
	.num_bits = 16,
	.max_channels = STM32_ADC_CH_MAX,
	.calib = &stm32_adc_selfcalib,
	.has_vregready = true,
	.has_boostmode = true,
	.has_linearcal = true,
	.has_presel = true,
};

static const struct stm32_adc_cfg stm32mp13_adc_cfg = {
	.regs = &stm32mp13_adc_regspec,
	.num_bits = 12,
	.max_channels = STM32_ADC_CH_MAX - 1,
	.calib = &stm32_adc_selfcalib,
};

static const struct stm32_adc_cfg stm32mp25_adc_cfg = {
	.regs = &stm32h7_adc_regspec,
	.num_bits = 12,
	.max_channels = STM32_ADC_CH_MAX,
	.calib = &stm32mp25_adc_calib,
	.has_presel = true,
};

static const struct udevice_id stm32_adc_ids[] = {
	{ .compatible = "st,stm32h7-adc",
	  .data = (ulong)&stm32h7_adc_cfg },
	{ .compatible = "st,stm32mp1-adc",
	  .data = (ulong)&stm32mp1_adc_cfg },
	{ .compatible = "st,stm32mp13-adc",
	  .data = (ulong)&stm32mp13_adc_cfg },
	{ .compatible = "st,stm32mp21-adc",
	  .data = (ulong)&stm32mp25_adc_cfg },
	{ .compatible = "st,stm32mp23-adc",
	  .data = (ulong)&stm32mp25_adc_cfg },
	{ .compatible = "st,stm32mp25-adc",
	  .data = (ulong)&stm32mp25_adc_cfg },
	{}
};

U_BOOT_DRIVER(stm32_adc) = {
	.name  = "stm32-adc",
	.id = UCLASS_ADC,
	.of_match = stm32_adc_ids,
	.probe = stm32_adc_probe,
	.ops = &stm32_adc_ops,
	.priv_auto	= sizeof(struct stm32_adc),
};
