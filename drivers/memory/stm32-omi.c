// SPDX-License-Identifier: GPL-2.0-or-later OR BSD-3-Clause
/*
 * Copyright (C) 2021, STMicroelectronics - All Rights Reserved
 */

#include <common.h>
#include <dm.h>
#include <log.h>
#include <misc.h>
#include <stm32_omi.h>
#include <watchdog.h>
#include <dm/of_access.h>
#include <dm/device-internal.h>
#include <dm/device_compat.h>
#include <dm/lists.h>
#include <dm/read.h>
#include <linux/ioport.h>

static void stm32_omi_read_fifo(u8 *val, phys_addr_t addr)
{
	*val = readb(addr);
	schedule();
}

static void stm32_omi_write_fifo(u8 *val, phys_addr_t addr)
{
	writeb(*val, addr);
}

int stm32_omi_tx_poll(struct stm32_omi_plat *omi, u8 *buf, u32 len, bool read)
{
	phys_addr_t regs_base = omi->regs_base;
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
			dev_err(omi->dev, "fifo timeout (len:%d stat:%#x)\n",
				len, sr);
			return ret;
		}

		fifo(buf++, regs_base + OSPI_DR);
	}

	return 0;
}

int stm32_omi_wait_for_not_busy(struct stm32_omi_plat *omi)
{
	phys_addr_t regs_base = omi->regs_base;
	u32 sr;
	int ret;

	ret = readl_poll_timeout(regs_base + OSPI_SR, sr, !(sr & OSPI_SR_BUSY),
				 OSPI_BUSY_TIMEOUT_US);
	if (ret)
		dev_err(omi->dev, "busy timeout (stat:%#x)\n", sr);

	return ret;
}

int stm32_omi_wait_cmd(struct stm32_omi_plat *omi)
{
	phys_addr_t regs_base = omi->regs_base;
	u32 sr;
	int ret = 0;

	ret = readl_poll_timeout(regs_base + OSPI_SR, sr,
				 sr & OSPI_SR_TCF,
				 OSPI_CMD_TIMEOUT_US);
	if (ret) {
		dev_err(omi->dev, "cmd timeout (stat:%#x)\n", sr);
	} else if (readl(regs_base + OSPI_SR) & OSPI_SR_TEF) {
		dev_err(omi->dev, "transfer error (stat:%#x)\n", sr);
		ret = -EIO;
	}

	/* clear flags */
	writel(OSPI_FCR_CTCF | OSPI_FCR_CTEF, regs_base + OSPI_FCR);

	if (!ret)
		ret = stm32_omi_wait_for_not_busy(omi);

	return ret;
}

static int stm32_omi_bind(struct udevice *dev)
{
	struct stm32_omi_plat *plat = dev_get_plat(dev);
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
	dev_for_each_subnode(flash_node, dev) {
		if (ofnode_device_is_compatible(flash_node, "cfi-flash"))
			hyperflash_count++;

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

	return 0;
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
	.bind		= stm32_omi_bind,
};
