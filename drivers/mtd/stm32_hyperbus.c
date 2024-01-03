// SPDX-License-Identifier: GPL-2.0-or-later OR BSD-3-Clause

/*
 * STMicroelectronics hyperflash driver
 * Copyright (C) 2021 STMicroelectronics - All Rights Reserved
 *
 */

#include <common.h>
#include <flash.h>
#include <mtd.h>
#include <stm32_omi.h>
#include <asm/io.h>
#include <dm/device_compat.h>
#include <dm/read.h>
#include <linux/bitfield.h>
#include <mtd/cfi_flash.h>

#define WRITE	0
#define READ	1
#define NSEC_PER_SEC		1000000000L

/* HyperBus Commands */
#define HYPERBUS_ADDR_UNLOCK1	0x555
#define HYPERBUS_ADDR_UNLOCK2	0x2AA
#define HYPERBUS_CMD_UNLOCK1	0xAA
#define HYPERBUS_CMD_UNLOCK2	0x55
#define HYPERBUS_CMD_RDSFDP	0x90

struct stm32_hb_priv {
	struct udevice *dev;
	struct udevice *omi_dev;
	ulong real_flash_freq;		/* real flash freq = bus_freq x prescaler */
};

struct stm32_hb_plat {
	ulong flash_freq;		/* flash max supported frequency */
	u32 tacc;
	u32 cs;
	bool wzl;
};

static struct stm32_hb_priv *g_stm32_hb_priv;

static int stm32_hb_xfer(void *addr, u16 wdata, u16 *rdata,
			 bool read)
{
	struct stm32_hb_priv *priv = g_stm32_hb_priv;
	struct stm32_omi_plat *omi_plat = dev_get_plat(priv->omi_dev);
	phys_addr_t regs_base = omi_plat->regs_base;
	int ret;
	u32 cr;
	u32 offset;

	/* exit from memory map mode by setting ABORT bit */
	setbits_le32(regs_base + OSPI_CR, OSPI_CR_ABORT);

	/* Wait clear of abort bit by hw */
	ret = readl_poll_timeout(regs_base + OSPI_CR, cr, !(cr & OSPI_CR_ABORT),
				 OSPI_ABT_TIMEOUT_US);

	if (ret) {
		dev_err(priv->dev, "%s abort timeout:%d\n", __func__, ret);
		return ret;
	}

	offset = (u32)(long)addr - omi_plat->mm_base;

	clrsetbits_le32(regs_base + OSPI_CR, OSPI_CR_FMODE_MASK,
			FIELD_PREP(OSPI_CR_FMODE_MASK,
				   read ? OSPI_CR_FMODE_IND_READ :
				   OSPI_CR_FMODE_IND_WRITE));

	writel((uintptr_t)offset, regs_base + OSPI_AR);

	ret = stm32_omi_tx_poll(priv->omi_dev, read ? (u8 *)rdata : (u8 *)&wdata, 2, read);
	if (ret)
		return ret;

	/* Wait end of tx in indirect mode */
	ret = stm32_omi_wait_cmd(priv->omi_dev);
	if (ret)
		return ret;

	dev_dbg(priv->dev, "%s: %s 0x%x @ 0x%x\n", __func__,
		read ? "read" : "write",
		read ? *rdata : wdata, offset >> 1);

	clrsetbits_le32(regs_base + OSPI_CR, OSPI_CR_FMODE_MASK,
			FIELD_PREP(OSPI_CR_FMODE_MASK, OSPI_CR_FMODE_MMAP));

	return ret;
}

u16 flash_read16(void *addr)
{
	struct stm32_hb_priv *priv = g_stm32_hb_priv;
	struct stm32_omi_plat *omi_plat = dev_get_plat(priv->omi_dev);
	phys_addr_t mm_base = omi_plat->mm_base;
	resource_size_t mm_size = omi_plat->mm_size;
	int ret;
	u16 rdata = 0;

	/*
	 * Before going further, check if this read is accessing DDR or Flash
	 */
	if (((u32)(long)addr < mm_base) ||
	    ((u32)(long)addr > mm_base + mm_size))
		return readw(addr);

	ret = stm32_hb_xfer(addr, 0, &rdata, READ);
	if (ret)
		dev_err(priv->dev, "%s failed, ret=%i\n", __func__, ret);

	return rdata;
}

void flash_write16(u16 value, void *addr)
{
	struct stm32_hb_priv *priv = g_stm32_hb_priv;
	int ret;

	ret = stm32_hb_xfer(addr, value, 0, WRITE);
	if (ret)
		dev_err(priv->dev, "%s failed, ret=%i\n", __func__, ret);
};

static int stm32_hb_test_sfdp(struct udevice *omi_dev)
{
	struct stm32_omi_plat *omi_plat = dev_get_plat(omi_dev);
	phys_addr_t mm_base = omi_plat->mm_base;
	int ret = -EIO;
	u16 sfdp[2];

	/* Reset */
	flash_write16(AMD_CMD_RESET, (void *)mm_base);
	flash_write16(HYPERBUS_CMD_UNLOCK1, (void *)mm_base + (HYPERBUS_ADDR_UNLOCK1 << 1));
	flash_write16(HYPERBUS_CMD_UNLOCK2, (void *)mm_base + (HYPERBUS_ADDR_UNLOCK2 << 1));
	flash_write16(HYPERBUS_CMD_RDSFDP, (void *)mm_base + (HYPERBUS_ADDR_UNLOCK1 << 1));

	sfdp[0] = readw(mm_base);
	sfdp[1] = readw(mm_base + 0x2);

	/* compare with "SF" & "DP" */
	if (sfdp[0] == 0x4653 && sfdp[1] == 0x5044)
		ret = 0;

	/* Reset CFI */
	flash_write16(AMD_CMD_RESET, (void *)mm_base);

	return ret;
}

static int stm32_hb_test_cfi(struct udevice *omi_dev)
{
	struct stm32_omi_plat *omi_plat = dev_get_plat(omi_dev);
	phys_addr_t mm_base = omi_plat->mm_base;
	int ret = -EIO;
	u16 qry[3];

	/* Reset/Enter in CFI */
	flash_write16(AMD_CMD_RESET, (void *)mm_base);
	flash_write16(FLASH_CMD_CFI, (void *)mm_base + 0xaa);

	qry[0] = readw(mm_base + 0x20);
	qry[1] = readw(mm_base + 0x22);
	qry[2] = readw(mm_base + 0x24);
	if (qry[0] == 'Q' && qry[1] == 'R' && qry[2] == 'Y')
		ret = 0;

	/* Reset/Exit from CFI */
	flash_write16(AMD_CMD_RESET, (void *)mm_base);
	flash_write16(FLASH_CMD_RESET, (void *)mm_base);

	return ret;
}

static int stm32_hb_check_transfer(struct udevice *omi_dev)
{
	struct stm32_omi_plat *omi_plat = dev_get_plat(omi_dev);

	if (omi_plat->jedec_flash)
		return stm32_hb_test_sfdp(omi_dev);
	else
		return stm32_hb_test_cfi(omi_dev);
}

static int stm32_hb_calibrate(struct stm32_hb_priv *priv)
{
	struct stm32_omi_plat *omi_plat = dev_get_plat(priv->omi_dev);
	u32 prescaler;
	u16 period_ps = 0;
	u8 window_len = 0;
	int ret;
	bool bypass_mode = false;

	prescaler = FIELD_GET(OSPI_DCR2_PRESC_MASK,
			      readl(omi_plat->regs_base + OSPI_DCR2));
	if (prescaler)
		setbits_le32(omi_plat->regs_base + OSPI_TCR, OSPI_TCR_DHQC);

	if (priv->real_flash_freq <= STM32_DLYB_FREQ_THRESHOLD) {
		bypass_mode = true;
		period_ps = NSEC_PER_SEC / (priv->real_flash_freq / 1000);
	}

	ret = stm32_omi_dlyb_configure(priv->omi_dev, bypass_mode, period_ps);
	if (ret)
		return ret;

	if (bypass_mode || prescaler)
		/* perform only RX TAP selection */
		ret = stm32_omi_dlyb_find_tap(priv->omi_dev, true, &window_len);
	else
		/* perform RX/TX TAP selection */
		ret = stm32_omi_dlyb_find_tap(priv->omi_dev, false, &window_len);

	if (ret) {
		dev_err(priv->omi_dev, "Calibration failed\n");
		if (!bypass_mode)
			/* stop delay block when configured in lock mode */
			stm32_omi_dlyb_stop(priv->omi_dev);
	}

	return ret;
}

static void stm32_hb_init(struct udevice *dev)
{
	struct stm32_hb_priv *priv = dev_get_priv(dev);
	struct stm32_hb_plat *plat = dev_get_plat(dev);
	struct stm32_omi_plat *omi_plat = dev_get_plat(priv->omi_dev);
	phys_addr_t regs_base = omi_plat->regs_base;
	unsigned long period;
	u32 ccr, dcr1, hlcr, prescaler;

	/* enable IP */
	setbits_le32(regs_base + OSPI_CR, OSPI_CR_EN);

	clrsetbits_le32(regs_base + OSPI_CR, OSPI_CR_CSSEL,
			FIELD_PREP(OSPI_CR_CSSEL, plat->cs));

	/* set MTYP to HyperBus memory-map mode */
	dcr1 = FIELD_PREP(OSPI_DCR1_MTYP_MASK, OSPI_DCR1_MTYP_HP_MEMMODE);
	/* set DEVSIZE to memory map size */
	dcr1 |= FIELD_PREP(OSPI_DCR1_DEVSIZE_MASK, ffs(omi_plat->mm_size) - 1);
	writel(dcr1, regs_base + OSPI_DCR1);

	prescaler = DIV_ROUND_UP(omi_plat->clock_rate, plat->flash_freq) - 1;
	if (prescaler > 255)
		prescaler = 255;

	clrsetbits_le32(regs_base + OSPI_DCR2, OSPI_DCR2_PRESC_MASK,
			FIELD_PREP(OSPI_DCR2_PRESC_MASK, prescaler));
	priv->real_flash_freq = omi_plat->clock_rate / (prescaler + 1);

	writel(1, regs_base + OSPI_DLR);

	/* set access time latency */
	period = NSEC_PER_SEC / priv->real_flash_freq;
	hlcr = FIELD_PREP(OSPI_HLCR_TACC_MASK, DIV_ROUND_UP(plat->tacc, period));
	/* set write zero latency */
	if (plat->wzl)
		hlcr |= OSPI_HLCR_WZL;

	writel(hlcr, regs_base + OSPI_HLCR);

	ccr = OSPI_CCR_DQSE | OSPI_CCR_DDTR | OSPI_CCR_ADDTR;
	ccr |= FIELD_PREP(OSPI_CCR_DMODE_MASK, OSPI_CCR_DMODE_8LINES);
	ccr |= FIELD_PREP(OSPI_CCR_ADSIZE_MASK, OSPI_CCR_ADSIZE_32BITS);
	ccr |= FIELD_PREP(OSPI_CCR_ADMODE_MASK, OSPI_CCR_ADMODE_8LINES);
	writel(ccr, regs_base + OSPI_CCR);

	/* Set FMODE to memory map mode */
	clrsetbits_le32(regs_base + OSPI_CR, OSPI_CR_FMODE_MASK,
			FIELD_PREP(OSPI_CR_FMODE_MASK, OSPI_CR_FMODE_MMAP));
}

static int stm32_hb_probe(struct udevice *dev)
{
	struct stm32_hb_priv *priv = dev_get_priv(dev);
	struct stm32_omi_plat *omi_plat;
	struct stm32_omi_priv *omi_priv;
	int ret;

	priv->omi_dev = dev->parent;
	omi_plat = dev_get_plat(priv->omi_dev);
	omi_priv = dev_get_priv(priv->omi_dev);
	omi_priv->dev = dev;
	priv->dev = dev;

	g_stm32_hb_priv = priv;

	/* mandatory for HyperFlash */
	if (!omi_plat->mm_size) {
		dev_err(dev, "Memory-map region not found\n");
		return -EINVAL;
	}

	/* mandatory for HyperFlash */
	if (!omi_plat->dlyb_base) {
		dev_err(dev, "Incorrect delay block base address\n");
		return -EINVAL;
	}

	ret = clk_enable(&omi_plat->clk);
	if (ret) {
		dev_err(dev, "failed to enable HyperBus clock\n");
		return ret;
	}

	reset_assert_bulk(&omi_plat->rst_ctl);
	udelay(2);
	reset_deassert_bulk(&omi_plat->rst_ctl);

	omi_priv->check_transfer = stm32_hb_check_transfer;
	stm32_hb_init(dev);

	return stm32_hb_calibrate(priv);
}

static int stm32_hb_of_to_plat(struct udevice *dev)
{
	struct stm32_hb_plat *plat = dev_get_plat(dev);
	ofnode flash_node;
	int ret;

	flash_node = dev_read_first_subnode(dev);

	ret = ofnode_read_u32(flash_node, "reg", &plat->cs);
	if (ret) {
		dev_err(dev, "could not retrieve reg property: %d\n", ret);
		return ret;
	}

	plat->flash_freq = ofnode_read_u32_default(flash_node, "st,max-frequency", 0);
	if (!plat->flash_freq) {
		dev_err(dev, "Can't find st,max-frequency property\n");
		return -ENOENT;
	}

	plat->tacc = ofnode_read_u32_default(flash_node, "st,tacc-ns", 0);
	plat->wzl = ofnode_read_bool(flash_node, "st,wzl");

	return 0;
}

U_BOOT_DRIVER(stm32_hyperbus) = {
	.name		= "stm32_hyperbus",
	.id		= UCLASS_MTD,
	.bind		= dm_scan_fdt_dev,
	.of_to_plat	= stm32_hb_of_to_plat,
	.plat_auto	= sizeof(struct stm32_hb_plat),
	.priv_auto	= sizeof(struct stm32_hb_priv),
	.probe		= stm32_hb_probe,
};
