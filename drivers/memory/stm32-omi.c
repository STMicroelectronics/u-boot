// SPDX-License-Identifier: GPL-2.0-or-later OR BSD-3-Clause
/*
 * Copyright (C) 2021, STMicroelectronics - All Rights Reserved
 */

#include <common.h>
#include <dm.h>
#include <log.h>
#include <misc.h>
#include <regmap.h>
#include <stm32_omi.h>
#include <syscon.h>
#include <watchdog.h>
#include <dm/of_access.h>
#include <dm/device-internal.h>
#include <dm/device_compat.h>
#include <dm/lists.h>
#include <dm/read.h>
#include <linux/bitfield.h>
#include <linux/ioport.h>

static int stm32_omi_dlyb_set_tap(struct udevice *dev, u8 tap, bool rx_tap)
{
	struct stm32_omi_plat *omi_plat = dev_get_plat(dev);
	u32 sr, mask, ack;
	int ret;
	u8 shift;

	if (!omi_plat->regmap || !omi_plat->dlyb_base)
		return -EINVAL;

	if (rx_tap) {
		mask = DLYBOS_CR_RXTAPSEL_MASK;
		shift = DLYBOS_CR_RXTAPSEL_SHIFT;
		ack = DLYBOS_SR_RXTAPSEL_ACK;
	} else {
		mask = DLYBOS_CR_TXTAPSEL_MASK;
		shift = DLYBOS_CR_TXTAPSEL_SHIFT;
		ack = DLYBOS_SR_TXTAPSEL_ACK;
	}

	regmap_update_bits(omi_plat->regmap,
			   omi_plat->dlyb_base + SYSCFG_DLYBOS_CR,
			   mask, mask & (tap << shift));

	ret = regmap_read_poll_timeout(omi_plat->regmap,
				       omi_plat->dlyb_base + SYSCFG_DLYBOS_SR,
				       sr, sr & ack, 1,
				       STM32_DLYBOS_TIMEOUT_MS);
	if (ret)
		dev_err(dev, "%s delay block phase configuration timeout\n",
			rx_tap ? "RX" : "TX");

	return ret;
}

int stm32_omi_dlyb_find_tap(struct udevice *dev, bool rx_only, u8 *window_len)
{
	struct stm32_omi_priv *omi_priv = dev_get_priv(dev);
	struct stm32_tap_window rx_tap_w[DLYBOS_TAPSEL_NB];
	int ret;
	u8 rx_len, rx_window_len, rx_window_end;
	u8 tx_len, tx_window_len, tx_window_end;
	u8 rx_tap, tx_tap, tx_tap_max, tx_tap_min, best_tx_tap = 0;
	u8 score, score_max;

	tx_len = 0;
	tx_window_len = 0;
	tx_window_end = 0;
	omi_priv->is_calibrating = true;

	for (tx_tap = 0;
	     tx_tap < (rx_only ? 1 : DLYBOS_TAPSEL_NB);
	     tx_tap++) {
		ret = stm32_omi_dlyb_set_tap(dev, tx_tap, false);
		if (ret)
			return ret;

		rx_len = 0;
		rx_window_len = 0;
		rx_window_end = 0;

		for (rx_tap = 0; rx_tap < DLYBOS_TAPSEL_NB; rx_tap++) {
			ret = stm32_omi_dlyb_set_tap(dev, rx_tap, true);
			if (ret)
				return ret;

			ret = omi_priv->check_transfer(dev);
			if (ret) {
				if ((!rx_only && ret == -ETIMEDOUT) ||
				    ret == -EOPNOTSUPP)
					break;

				rx_len = 0;
			} else {
				rx_len++;
				if (rx_len > rx_window_len) {
					rx_window_len = rx_len;
					rx_window_end = rx_tap;
				}
			}
		}

		if (ret == -EOPNOTSUPP)
			break;

		rx_tap_w[tx_tap].end = rx_window_end;
		rx_tap_w[tx_tap].length = rx_window_len;

		if (!rx_window_len) {
			tx_len = 0;
		} else {
			tx_len++;
			if (tx_len > tx_window_len) {
				tx_window_len = tx_len;
				tx_window_end = tx_tap;
			}
		}

		dev_dbg(dev, "rx_tap_w[%02d].end = %d rx_tap_w[%02d].length = %d\n",
			tx_tap, rx_tap_w[tx_tap].end, tx_tap, rx_tap_w[tx_tap].length);
	}

	omi_priv->is_calibrating = false;

	if (ret == -EOPNOTSUPP) {
		dev_err(dev, "Calibration can not be done on this device\n");
		return ret;
	}

	if (rx_only) {
		if (!rx_window_len) {
			dev_err(dev, "Can't find RX phase settings\n");
			return -EIO;
		}

		rx_tap = rx_window_end - rx_window_len / 2;
		*window_len = rx_window_len;
		dev_dbg(dev, "RX_TAP_SEL set to %d\n", rx_tap);

		return stm32_omi_dlyb_set_tap(dev, rx_tap, true);
	}

	if (!tx_window_len) {
		dev_err(dev, "Can't find TX phase settings\n");
		return -EIO;
	}

	/* find the best duet TX/RX TAP */
	tx_tap_min = tx_window_end - tx_window_len + 1;
	tx_tap_max = tx_window_end;
	score_max = 0;
	for (tx_tap = tx_tap_min; tx_tap <= tx_tap_max; tx_tap++) {
		score = min_t(u8, tx_tap - tx_tap_min, tx_tap_max - tx_tap) +
			rx_tap_w[tx_tap].length;
		if (score > score_max) {
			score_max = score;
			best_tx_tap = tx_tap;
		}
	}

	rx_tap = rx_tap_w[best_tx_tap].end - rx_tap_w[best_tx_tap].length / 2;

	dev_dbg(dev, "RX_TAP_SEL set to %d\n", rx_tap);
	ret = stm32_omi_dlyb_set_tap(dev, rx_tap, true);
	if (ret)
		return ret;

	dev_dbg(dev, "TX_TAP_SEL set to %d\n", best_tx_tap);

	return stm32_omi_dlyb_set_tap(dev, best_tx_tap, false);
}

int stm32_omi_dlyb_set_cr(struct udevice *dev, u32 dlyb_cr)
{
	bool bypass_mode = false;
	int ret;
	u16 period_ps;
	u8 rx_tap, tx_tap;

	period_ps = FIELD_GET(DLYBOS_BYP_CMD_MASK, dlyb_cr);
	if (dlyb_cr & DLYBOS_BYP_EN)
		bypass_mode = true;

	ret = stm32_omi_dlyb_configure(dev, bypass_mode, period_ps);
	if (ret)
		return ret;

	/* restore Rx and TX tap */
	rx_tap = FIELD_GET(DLYBOS_CR_RXTAPSEL_MASK, dlyb_cr);
	ret = stm32_omi_dlyb_set_tap(dev, rx_tap, true);
	if (ret)
		return ret;

	tx_tap = FIELD_GET(DLYBOS_CR_TXTAPSEL_MASK, dlyb_cr);
	return stm32_omi_dlyb_set_tap(dev, tx_tap, false);
}

void stm32_omi_dlyb_get_cr(struct udevice *dev, u32 *dlyb_cr)
{
	struct stm32_omi_plat *omi_plat = dev_get_plat(dev);

	regmap_read(omi_plat->regmap, omi_plat->dlyb_base + SYSCFG_DLYBOS_CR,
		    dlyb_cr);
}

/* ½ memory clock period in pico second */
static const u16 dlybos_delay_ps[STM32_DLYBOS_DELAY_NB] = {
2816, 4672, 6272, 7872, 9472, 11104, 12704, 14304, 15904, 17536, 19136, 20736,
22336, 23968, 25568, 27168, 28768, 30400, 32000, 33600, 35232, 36832, 38432, 40032
};

static u32 stm32_omi_find_byp_cmd(u16 period_ps)
{
	u16 half_period_ps = period_ps / 2;
	u8 max = STM32_DLYBOS_DELAY_NB - 1;
	u8 i, min = 0;

	/* find closest value in dlybos_delay_ps[] with half_period_ps*/
	if (half_period_ps < dlybos_delay_ps[0])
		return FIELD_PREP(DLYBOS_BYP_CMD_MASK, 1);

	if (half_period_ps > dlybos_delay_ps[max])
		return FIELD_PREP(DLYBOS_BYP_CMD_MASK, STM32_DLYBOS_DELAY_NB);

	while (max - min > 1) {
		i = DIV_ROUND_UP(min + max, 2);
		if (half_period_ps > dlybos_delay_ps[i])
			min = i;
		else
			max = i;
	}

	if ((dlybos_delay_ps[max] - half_period_ps) >
	    (half_period_ps - dlybos_delay_ps[min]))
		return FIELD_PREP(DLYBOS_BYP_CMD_MASK, min + 1);
	else
		return FIELD_PREP(DLYBOS_BYP_CMD_MASK, max + 1);
}

void stm32_omi_dlyb_stop(struct udevice *dev)
{
	struct stm32_omi_plat *omi_plat = dev_get_plat(dev);

	/* disable delay block */
	regmap_write(omi_plat->regmap,
		     omi_plat->dlyb_base + SYSCFG_DLYBOS_CR,
		     0x0);
}

int stm32_omi_dlyb_configure(struct udevice *dev,
			     bool bypass_mode, u16 period_ps)
{
	struct stm32_omi_plat *omi_plat = dev_get_plat(dev);
	u32 sr, mask, val;
	int ret;

	if (!omi_plat->regmap || !omi_plat->dlyb_base)
		return -EINVAL;

	if (bypass_mode) {
		val = DLYBOS_BYP_EN | stm32_omi_find_byp_cmd(period_ps);
		mask = DLYBOS_BYP_EN | DLYBOS_BYP_CMD_MASK;
	} else {
		val = DLYBOS_CR_EN;
		mask = DLYBOS_CR_EN;
	}

	regmap_update_bits(omi_plat->regmap,
			  omi_plat->dlyb_base + SYSCFG_DLYBOS_CR,
			  mask, val);

	if (bypass_mode)
		return 0;

	/* in lock mode, wait for lock status bit */
	ret = regmap_read_poll_timeout(omi_plat->regmap,
				       omi_plat->dlyb_base + SYSCFG_DLYBOS_SR,
				       sr, sr & DLYBOS_SR_LOCK, 1,
				       STM32_DLYBOS_TIMEOUT_MS);
	if (ret) {
		dev_err(dev, "Delay Block lock timeout\n");
		stm32_omi_dlyb_stop(dev);
	}

	return ret;
}

static void stm32_omi_read_fifo(u8 *val, phys_addr_t addr)
{
	*val = readb(addr);
	schedule();
}

static void stm32_omi_write_fifo(u8 *val, phys_addr_t addr)
{
	writeb(*val, addr);
}

int stm32_omi_tx_poll(struct udevice *dev, u8 *buf, u32 len, bool read)
{
	struct stm32_omi_plat *omi_plat = dev_get_plat(dev);
	struct stm32_omi_priv *omi_priv = dev_get_priv(dev);
	phys_addr_t regs_base = omi_plat->regs_base;
	void (*fifo)(u8 *val, phys_addr_t addr);
	u32 sr;
	int ret;

	if (read)
		fifo = stm32_omi_read_fifo;
	else
		fifo = stm32_omi_write_fifo;

	while (len--) {
		ret = readl_poll_timeout(regs_base + OSPI_SR, sr,
					 sr & OSPI_SR_FTF,
					 OSPI_FIFO_TIMEOUT_US);
		if (ret) {
			if (!omi_priv->is_calibrating)
				dev_err(dev, "fifo timeout (len:%d stat:%#x)\n",
					len, sr);
			return ret;
		}

		fifo(buf++, regs_base + OSPI_DR);
	}

	return 0;
}

int stm32_omi_wait_for_not_busy(struct udevice *dev)
{
	struct stm32_omi_plat *omi_plat = dev_get_plat(dev);
	phys_addr_t regs_base = omi_plat->regs_base;
	u32 sr;
	int ret;

	ret = readl_poll_timeout(regs_base + OSPI_SR, sr, !(sr & OSPI_SR_BUSY),
				 OSPI_BUSY_TIMEOUT_US);
	if (ret)
		dev_err(dev, "busy timeout (stat:%#x)\n", sr);

	return ret;
}

int stm32_omi_wait_cmd(struct udevice *dev)
{
	struct stm32_omi_plat *omi_plat = dev_get_plat(dev);
	phys_addr_t regs_base = omi_plat->regs_base;
	u32 sr;
	int ret = 0;

	ret = readl_poll_timeout(regs_base + OSPI_SR, sr,
				 sr & OSPI_SR_TCF,
				 OSPI_CMD_TIMEOUT_US);
	if (ret) {
		dev_err(dev, "cmd timeout (stat:%#x)\n", sr);
	} else if (readl(regs_base + OSPI_SR) & OSPI_SR_TEF) {
		dev_err(dev, "transfer error (stat:%#x)\n", sr);
		ret = -EIO;
	}

	/* clear flags */
	writel(OSPI_FCR_CTCF | OSPI_FCR_CTEF, regs_base + OSPI_FCR);

	if (!ret)
		ret = stm32_omi_wait_for_not_busy(dev);

	return ret;
}

static int stm32_omi_bind(struct udevice *dev)
{
	struct stm32_omi_plat *omi_plat = dev_get_plat(dev);
	struct driver *drv;
	const char *name;
	ofnode flash_node;
	u8 hyperflash_count = 0;
	u8 spi_flash_count = 0;
	u8 child_count = 0;

	/*
	 * Flash subnodes sanity check:
	 *        2 spi-nand/spi-nor flashes			=> supported
	 *        1 HyperFlash					=> supported
	 *	  All other flash node configuration		=> not supported
	 */
	omi_plat->jedec_flash = false;

	dev_for_each_subnode(flash_node, dev) {
		if (ofnode_device_is_compatible(flash_node, "cfi-flash"))
			hyperflash_count++;

		if (ofnode_device_is_compatible(flash_node, "jedec-flash")) {
			hyperflash_count++;
			omi_plat->jedec_flash = true;
		}

		if (ofnode_device_is_compatible(flash_node, "jedec,spi-nor") ||
		    ofnode_device_is_compatible(flash_node, "spi-nand"))
			spi_flash_count++;

		child_count++;
	}

	if (!child_count) {
		dev_err(dev, "Missing flash node\n");
		return -ENODEV;
	}

	if ((!hyperflash_count && !spi_flash_count) ||
	    child_count != (hyperflash_count + spi_flash_count)) {
		dev_warn(dev, "Unknown flash type\n");
		return -ENODEV;
	}

	if ((hyperflash_count && spi_flash_count) ||
	    hyperflash_count > 1) {
		dev_err(dev, "Flash node configuration not supported\n");
		return -EINVAL;
	}

	if (spi_flash_count)
		name = "stm32_ospi";
	else
		name = "stm32_hyperbus";

	drv = lists_driver_lookup_name(name);
	if (!drv) {
		dev_err(dev, "Cannot find driver '%s'\n", name);
		return -ENOENT;
	}

	return device_bind(dev, drv, dev_read_name(dev), NULL, dev_ofnode(dev), NULL);
}

static int stm32_omi_of_to_plat(struct udevice *dev)
{
	struct stm32_omi_plat *plat = dev_get_plat(dev);
	struct resource res;
	struct ofnode_phandle_args args;
	const fdt32_t *reg;
	int ret, len;

	reg = dev_read_prop(dev, "reg", &len);
	if (!reg) {
		dev_err(dev, "Can't get regs base address\n");
		return -ENOENT;
	}

	plat->regs_base = (phys_addr_t)dev_translate_address(dev, reg);

	/* optional */
	ret = dev_read_phandle_with_args(dev, "memory-region", NULL, 0, 0, &args);
	if (!ret) {
		ret = ofnode_read_resource(args.node, 0, &res);
		if (ret) {
			dev_err(dev, "Can't get mmap base address(%d)\n", ret);
			return ret;
		}

		plat->mm_base = res.start;
		plat->mm_size = resource_size(&res);

		if (plat->mm_size > OSPI_MAX_MMAP_SZ) {
			dev_err(dev, "Incorrect memory-map size: %lld Bytes\n", plat->mm_size);
			return -EINVAL;
		}

		dev_dbg(dev, "%s: regs_base=<0x%llx> mm_base=<0x%llx> mm_size=<0x%x>\n",
			__func__, plat->regs_base, plat->mm_base, (u32)plat->mm_size);
	} else {
		plat->mm_base = 0;
		plat->mm_size = 0;
		dev_info(dev, "memory-region property not found (%d)\n", ret);
	}

	ret = clk_get_by_index(dev, 0, &plat->clk);
	if (ret < 0) {
		dev_err(dev, "Failed to get clock\n");
		return ret;
	}

	ret = reset_get_bulk(dev, &plat->rst_ctl);
	if (ret && ret != -ENOENT) {
		dev_err(dev, "Failed to get reset\n");
		clk_free(&plat->clk);
		return ret;
	}

	plat->clock_rate = clk_get_rate(&plat->clk);
	if (!plat->clock_rate) {
		clk_free(&plat->clk);
		return -EINVAL;
	}

	plat->regmap = syscon_regmap_lookup_by_phandle(dev, "st,syscfg-dlyb");
	if (IS_ERR(plat->regmap)) {
		dev_err(dev, "Can't find st,syscfg-dlyb property\n");
		ret = PTR_ERR(plat->regmap);
	} else {
		ret = dev_read_u32_index(dev, "st,syscfg-dlyb", 1, &plat->dlyb_base);
		if (ret)
			dev_err(dev, "Can't read delay block base address\n");
	}

	return ret;
};

static const struct udevice_id stm32_omi_ids[] = {
	{.compatible = "st,stm32mp25-omi" },
	{ }
};

U_BOOT_DRIVER(stm32_omi) = {
	.name		= "stm32-omi",
	.id		= UCLASS_NOP,
	.of_match	= stm32_omi_ids,
	.of_to_plat	= stm32_omi_of_to_plat,
	.plat_auto	= sizeof(struct stm32_omi_plat),
	.priv_auto	= sizeof(struct stm32_omi_priv),
	.bind		= stm32_omi_bind,
};
