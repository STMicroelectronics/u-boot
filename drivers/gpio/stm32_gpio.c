// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2017, STMicroelectronics - All Rights Reserved
 * Author(s): Vikas Manocha, <vikas.manocha@st.com> for STMicroelectronics.
 */

#define LOG_CATEGORY UCLASS_GPIO

#include <common.h>
#include <clk.h>
#include <dm.h>
#include <fdtdec.h>
#include <log.h>
#include <asm/arch/stm32.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <dm/device_compat.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/io.h>

#include "stm32_gpio_priv.h"

#define STM32_GPIOS_PER_BANK		16

#define MODE_BITS(gpio_pin)		((gpio_pin) * 2)
#define MODE_BITS_MASK			3
#define BSRR_BIT(gpio_pin, value)	BIT((gpio_pin) + (value ? 0 : 16))

#define PUPD_BITS(gpio_pin)		((gpio_pin) * 2)
#define PUPD_MASK			3

#define OTYPE_BITS(gpio_pin)		(gpio_pin)
#define OTYPE_MSK			1

#define SECCFG_BITS(gpio_pin)		(gpio_pin)
#define SECCFG_MSK			1

#define STM32_GPIO_CID1			1

#define STM32_GPIO_CIDCFGR_CFEN		BIT(0)
#define STM32_GPIO_CIDCFGR_SEMEN	BIT(1)
#define STM32_GPIO_CIDCFGR_SCID_MASK	GENMASK(5, 4)
#define STM32_GPIO_CIDCFGR_SEMWL_CID1	BIT(16 + STM32_GPIO_CID1)

#define STM32_GPIO_SEMCR_SEM_MUTEX	BIT(0)
#define STM32_GPIO_SEMCR_SEMCID_MASK	GENMASK(5, 4)

static void stm32_gpio_set_moder(struct stm32_gpio_regs *regs,
				 int idx,
				 int mode)
{
	int bits_index;
	int mask;

	bits_index = MODE_BITS(idx);
	mask = MODE_BITS_MASK << bits_index;

	clrsetbits_le32(&regs->moder, mask, mode << bits_index);
}

static int stm32_gpio_get_moder(struct stm32_gpio_regs *regs, int idx)
{
	return (readl(&regs->moder) >> MODE_BITS(idx)) & MODE_BITS_MASK;
}

static void stm32_gpio_set_otype(struct stm32_gpio_regs *regs,
				 int idx,
				 enum stm32_gpio_otype otype)
{
	int bits;

	bits = OTYPE_BITS(idx);
	clrsetbits_le32(&regs->otyper, OTYPE_MSK << bits, otype << bits);
}

static enum stm32_gpio_otype stm32_gpio_get_otype(struct stm32_gpio_regs *regs,
						  int idx)
{
	return (readl(&regs->otyper) >> OTYPE_BITS(idx)) & OTYPE_MSK;
}

static void stm32_gpio_set_pupd(struct stm32_gpio_regs *regs,
				int idx,
				enum stm32_gpio_pupd pupd)
{
	int bits;

	bits = PUPD_BITS(idx);
	clrsetbits_le32(&regs->pupdr, PUPD_MASK << bits, pupd << bits);
}

static enum stm32_gpio_pupd stm32_gpio_get_pupd(struct stm32_gpio_regs *regs,
						int idx)
{
	return (readl(&regs->pupdr) >> PUPD_BITS(idx)) & PUPD_MASK;
}

static bool stm32_gpio_is_mapped(struct udevice *dev, int offset)
{
	struct stm32_gpio_priv *priv = dev_get_priv(dev);

	return !!(priv->gpio_range & BIT(offset));
}

bool stm32_gpio_rif_valid(struct stm32_gpio_regs *regs, unsigned int offset)
{
	u32 cid, sem;

	cid = readl(&regs->rif[offset].cidcfgr);

	if (!(cid & STM32_GPIO_CIDCFGR_CFEN))
		return true;

	if (!(cid & STM32_GPIO_CIDCFGR_SEMEN)) {
		if (FIELD_GET(STM32_GPIO_CIDCFGR_SCID_MASK, cid) == STM32_GPIO_CID1)
			return true;

		return false;
	}

	if (!(cid & STM32_GPIO_CIDCFGR_SEMWL_CID1))
		return false;

	sem = readl(&regs->rif[offset].semcr);

	if (sem & STM32_GPIO_SEMCR_SEM_MUTEX) {
		if (FIELD_GET(STM32_GPIO_SEMCR_SEMCID_MASK, sem) == STM32_GPIO_CID1)
			return true;

		return false;
	}

	writel(STM32_GPIO_SEMCR_SEM_MUTEX, &regs->rif[offset].semcr);
	sem = readl(&regs->rif[offset].semcr);
	if (sem & STM32_GPIO_SEMCR_SEM_MUTEX &&
	    FIELD_GET(STM32_GPIO_SEMCR_SEMCID_MASK, sem) == STM32_GPIO_CID1)
		return true;

	return false;
}

static void stm32_gpio_rif_release(struct stm32_gpio_regs *regs, unsigned int offset)
{
	u32 cid;

	cid = readl(&regs->rif[offset].cidcfgr);

	if (!(cid & STM32_GPIO_CIDCFGR_CFEN))
		return;

	if (cid & STM32_GPIO_CIDCFGR_SEMEN && cid & STM32_GPIO_CIDCFGR_SEMWL_CID1)
		writel(0, &regs->rif[offset].semcr);
}

static int stm32_gpio_request(struct udevice *dev, unsigned offset, const char *label)
{
	struct stm32_gpio_priv *priv = dev_get_priv(dev);
	struct gpio_dev_priv *uc_priv = dev_get_uclass_priv(dev);
	struct stm32_gpio_regs *regs = priv->regs;
	ulong drv_data = dev_get_driver_data(dev);

	if (!stm32_gpio_is_mapped(dev, offset))
		return -ENXIO;

	/* Deny request access if IO is secured */
	if ((drv_data & STM32_GPIO_FLAG_SEC_CTRL) &&
	    ((readl(&regs->seccfgr) >> SECCFG_BITS(offset)) & SECCFG_MSK)) {
		dev_err(dev, "Failed to get secure IO %s %d @ %p\n",
			uc_priv->bank_name, offset, regs);
		return -EACCES;
	}

	/* Deny request access if IO RIF semaphore is not available */
	if ((drv_data & STM32_GPIO_FLAG_RIF_CTRL) &&
	    !stm32_gpio_rif_valid(regs, offset)) {
		dev_err(dev, "Failed to take RIF semaphore on IO %s %d @ %p\n",
			uc_priv->bank_name, offset, regs);
		return -EACCES;
	}

	return 0;
}

static int stm32_gpio_rfree(struct udevice *dev, unsigned int offset)
{
	struct stm32_gpio_priv *priv = dev_get_priv(dev);
	struct stm32_gpio_regs *regs = priv->regs;
	ulong drv_data = dev_get_driver_data(dev);

	if (drv_data & STM32_GPIO_FLAG_RIF_CTRL)
		stm32_gpio_rif_release(regs, offset);

	return 0;
}

static int stm32_gpio_direction_input(struct udevice *dev, unsigned offset)
{
	struct stm32_gpio_priv *priv = dev_get_priv(dev);
	struct stm32_gpio_regs *regs = priv->regs;

	if (!stm32_gpio_is_mapped(dev, offset))
		return -ENXIO;

	stm32_gpio_set_moder(regs, offset, STM32_GPIO_MODE_IN);

	return 0;
}

static int stm32_gpio_direction_output(struct udevice *dev, unsigned offset,
				       int value)
{
	struct stm32_gpio_priv *priv = dev_get_priv(dev);
	struct stm32_gpio_regs *regs = priv->regs;

	if (!stm32_gpio_is_mapped(dev, offset))
		return -ENXIO;

	stm32_gpio_set_moder(regs, offset, STM32_GPIO_MODE_OUT);

	writel(BSRR_BIT(offset, value), &regs->bsrr);

	return 0;
}

static int stm32_gpio_get_value(struct udevice *dev, unsigned offset)
{
	struct stm32_gpio_priv *priv = dev_get_priv(dev);
	struct stm32_gpio_regs *regs = priv->regs;

	if (!stm32_gpio_is_mapped(dev, offset))
		return -ENXIO;

	return readl(&regs->idr) & BIT(offset) ? 1 : 0;
}

static int stm32_gpio_set_value(struct udevice *dev, unsigned offset, int value)
{
	struct stm32_gpio_priv *priv = dev_get_priv(dev);
	struct stm32_gpio_regs *regs = priv->regs;

	if (!stm32_gpio_is_mapped(dev, offset))
		return -ENXIO;

	writel(BSRR_BIT(offset, value), &regs->bsrr);

	return 0;
}

static int stm32_gpio_get_function(struct udevice *dev, unsigned int offset)
{
	struct stm32_gpio_priv *priv = dev_get_priv(dev);
	struct stm32_gpio_regs *regs = priv->regs;
	ulong drv_data = dev_get_driver_data(dev);
	int bits_index;
	int mask;
	u32 mode;

	if (!stm32_gpio_is_mapped(dev, offset))
		return GPIOF_UNKNOWN;

	/* Return 'protected' if the IO is secured */
	if ((drv_data & STM32_GPIO_FLAG_SEC_CTRL) &&
	    ((readl(&regs->seccfgr) >> SECCFG_BITS(offset)) & SECCFG_MSK))
		return GPIOF_PROTECTED;

	/* Return 'protected' if the IO RIF semaphore is not available */
	if ((drv_data & STM32_GPIO_FLAG_RIF_CTRL) &&
	    !stm32_gpio_rif_valid(regs, offset))
		return GPIOF_PROTECTED;

	bits_index = MODE_BITS(offset);
	mask = MODE_BITS_MASK << bits_index;

	mode = (readl(&regs->moder) & mask) >> bits_index;
	if (mode == STM32_GPIO_MODE_OUT)
		return GPIOF_OUTPUT;
	if (mode == STM32_GPIO_MODE_IN)
		return GPIOF_INPUT;
	if (mode == STM32_GPIO_MODE_AN)
		return GPIOF_UNUSED;

	return GPIOF_FUNC;
}

static int stm32_gpio_set_flags(struct udevice *dev, unsigned int offset,
				ulong flags)
{
	struct stm32_gpio_priv *priv = dev_get_priv(dev);
	struct stm32_gpio_regs *regs = priv->regs;

	if (!stm32_gpio_is_mapped(dev, offset))
		return -ENXIO;

	if (flags & GPIOD_IS_OUT) {
		bool value = flags & GPIOD_IS_OUT_ACTIVE;

		if (flags & GPIOD_OPEN_DRAIN)
			stm32_gpio_set_otype(regs, offset, STM32_GPIO_OTYPE_OD);
		else
			stm32_gpio_set_otype(regs, offset, STM32_GPIO_OTYPE_PP);

		stm32_gpio_set_moder(regs, offset, STM32_GPIO_MODE_OUT);
		writel(BSRR_BIT(offset, value), &regs->bsrr);

	} else if (flags & GPIOD_IS_IN) {
		stm32_gpio_set_moder(regs, offset, STM32_GPIO_MODE_IN);
	}
	if (flags & GPIOD_PULL_UP)
		stm32_gpio_set_pupd(regs, offset, STM32_GPIO_PUPD_UP);
	else if (flags & GPIOD_PULL_DOWN)
		stm32_gpio_set_pupd(regs, offset, STM32_GPIO_PUPD_DOWN);

	return 0;
}

static int stm32_gpio_get_flags(struct udevice *dev, unsigned int offset,
				ulong *flagsp)
{
	struct stm32_gpio_priv *priv = dev_get_priv(dev);
	struct stm32_gpio_regs *regs = priv->regs;
	ulong dir_flags = 0;

	if (!stm32_gpio_is_mapped(dev, offset))
		return -ENXIO;

	switch (stm32_gpio_get_moder(regs, offset)) {
	case STM32_GPIO_MODE_OUT:
		dir_flags |= GPIOD_IS_OUT;
		if (stm32_gpio_get_otype(regs, offset) == STM32_GPIO_OTYPE_OD)
			dir_flags |= GPIOD_OPEN_DRAIN;
		if (readl(&regs->idr) & BIT(offset))
			dir_flags |= GPIOD_IS_OUT_ACTIVE;
		break;
	case STM32_GPIO_MODE_IN:
		dir_flags |= GPIOD_IS_IN;
		break;
	default:
		break;
	}
	switch (stm32_gpio_get_pupd(regs, offset)) {
	case STM32_GPIO_PUPD_UP:
		dir_flags |= GPIOD_PULL_UP;
		break;
	case STM32_GPIO_PUPD_DOWN:
		dir_flags |= GPIOD_PULL_DOWN;
		break;
	default:
		break;
	}
	*flagsp = dir_flags;

	return 0;
}

static const struct dm_gpio_ops gpio_stm32_ops = {
	.request		= stm32_gpio_request,
	.rfree			= stm32_gpio_rfree,
	.direction_input	= stm32_gpio_direction_input,
	.direction_output	= stm32_gpio_direction_output,
	.get_value		= stm32_gpio_get_value,
	.set_value		= stm32_gpio_set_value,
	.get_function		= stm32_gpio_get_function,
	.set_flags		= stm32_gpio_set_flags,
	.get_flags		= stm32_gpio_get_flags,
};

static int gpio_stm32_probe(struct udevice *dev)
{
	struct stm32_gpio_priv *priv = dev_get_priv(dev);
	struct gpio_dev_priv *uc_priv = dev_get_uclass_priv(dev);
	struct ofnode_phandle_args args;
	const char *name;
	struct clk clk;
	fdt_addr_t addr;
	int ret, i;

	addr = dev_read_addr(dev);
	if (addr == FDT_ADDR_T_NONE)
		return -EINVAL;

	priv->regs = (struct stm32_gpio_regs *)addr;

	name = dev_read_string(dev, "st,bank-name");
	if (!name)
		return -EINVAL;
	uc_priv->bank_name = name;

	i = 0;
	ret = dev_read_phandle_with_args(dev, "gpio-ranges",
					 NULL, 3, i, &args);

	if (!ret && args.args_count < 3)
		return -EINVAL;

	uc_priv->gpio_count = STM32_GPIOS_PER_BANK;
	if (ret == -ENOENT)
		priv->gpio_range = GENMASK(STM32_GPIOS_PER_BANK - 1, 0);

	while (ret != -ENOENT) {
		priv->gpio_range |= GENMASK(args.args[2] + args.args[0] - 1,
				    args.args[0]);

		ret = dev_read_phandle_with_args(dev, "gpio-ranges", NULL, 3,
						 ++i, &args);
		if (!ret && args.args_count < 3)
			return -EINVAL;
	}

	dev_dbg(dev, "addr = 0x%p bank_name = %s gpio_count = %d gpio_range = 0x%x\n",
		(u32 *)priv->regs, uc_priv->bank_name, uc_priv->gpio_count,
		priv->gpio_range);

	ret = clk_get_by_index(dev, 0, &clk);
	if (ret < 0)
		return ret;

	ret = clk_enable(&clk);

	if (ret) {
		dev_err(dev, "failed to enable clock\n");
		return ret;
	}
	dev_dbg(dev, "clock enabled\n");

	return 0;
}

static int gpio_stm32_remove(struct udevice *dev)
{
	struct stm32_gpio_priv *priv = dev_get_priv(dev);
	struct stm32_gpio_regs *regs = priv->regs;
	ulong drv_data = dev_get_driver_data(dev);
	unsigned int offset;
	u32 seccfgr = 0;

	if (!(drv_data & STM32_GPIO_FLAG_RIF_CTRL))
		return 0;

	if (drv_data & STM32_GPIO_FLAG_SEC_CTRL)
		seccfgr = readl(&regs->seccfgr);

	for (offset = 0; offset < STM32_GPIOS_PER_BANK; offset++) {
		if (!stm32_gpio_is_mapped(dev, offset))
			continue;

		if ((seccfgr >> SECCFG_BITS(offset)) & SECCFG_MSK)
			continue;

		stm32_gpio_rif_release(regs, offset);
	}

	return 0;
}

U_BOOT_DRIVER(gpio_stm32) = {
	.name	= "gpio_stm32",
	.id	= UCLASS_GPIO,
	.probe	= gpio_stm32_probe,
	.remove	= gpio_stm32_remove,
	.ops	= &gpio_stm32_ops,
	.flags	= DM_UC_FLAG_SEQ_ALIAS | DM_FLAG_OS_PREPARE,
	.priv_auto	= sizeof(struct stm32_gpio_priv),
};
