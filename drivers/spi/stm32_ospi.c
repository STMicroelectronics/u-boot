// SPDX-License-Identifier: GPL-2.0-or-later OR BSD-3-Clause
/*
 * Copyright (C) 2021, STMicroelectronics - All Rights Reserved
 */

#define LOG_CATEGORY UCLASS_SPI

#include <common.h>
#include <dm.h>
#include <log.h>
#include <regmap.h>
#include <spi.h>
#include <spi-mem.h>
#include <stm32_omi.h>
#include <syscon.h>
#include <dm/device_compat.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/sizes.h>

struct stm32_ospi_flash {
	u32 cr;
	u32 dcr;
	u32 dcr2;
	bool initialized;
};

struct stm32_ospi_priv {
	struct stm32_ospi_flash flash[OSPI_MAX_CHIP];
	struct udevice *omi_dev;
	int cs_used;
};

static int stm32_ospi_mm(struct stm32_ospi_priv *priv,
			 const struct spi_mem_op *op)
{
	struct stm32_omi_plat *omi_plat = dev_get_plat(priv->omi_dev);

	memcpy_fromio(op->data.buf.in,
		      (void __iomem *)omi_plat->mm_base + op->addr.val,
		      op->data.nbytes);

	return 0;
}

static int stm32_ospi_tx(struct stm32_ospi_priv *priv,
			 const struct spi_mem_op *op,
			 u8 mode)
{
	u8 *buf;

	if (!op->data.nbytes)
		return 0;

	if (mode == OSPI_CCR_MEM_MAP)
		return stm32_ospi_mm(priv, op);

	if (op->data.dir == SPI_MEM_DATA_IN)
		buf = op->data.buf.in;
	else
		buf = (u8 *)op->data.buf.out;

	return stm32_omi_tx_poll(priv->omi_dev, buf, op->data.nbytes,
				 op->data.dir == SPI_MEM_DATA_IN);
}

static int stm32_ospi_get_mode(u8 buswidth)
{
	if (buswidth == 8)
		return 4;

	if (buswidth == 4)
		return 3;

	return buswidth;
}

static int stm32_ospi_exec_op(struct spi_slave *slave,
			      const struct spi_mem_op *op)
{
	struct stm32_ospi_priv *priv = dev_get_priv(slave->dev->parent);
	struct stm32_omi_plat *omi_plat = dev_get_plat(priv->omi_dev);
	phys_addr_t regs_base = omi_plat->regs_base;
	u32 cr, ccr = 0, addr_max;
	int timeout, ret;
	int dmode;
	u8 mode = OSPI_CCR_IND_WRITE;
	u8 dcyc = 0;

	dev_dbg(slave->dev, "%s: cmd:%#x mode:%d.%d.%d.%d addr:%#llx len:%#x\n",
		__func__, op->cmd.opcode, op->cmd.buswidth, op->addr.buswidth,
		op->dummy.buswidth, op->data.buswidth,
		op->addr.val, op->data.nbytes);

	addr_max = op->addr.val + op->data.nbytes + 1;

	if (op->data.dir == SPI_MEM_DATA_IN && op->data.nbytes) {
		if (addr_max < omi_plat->mm_size && op->addr.buswidth)
			mode = OSPI_CCR_MEM_MAP;
		else
			mode = OSPI_CCR_IND_READ;
	}

	if (op->data.nbytes)
		writel(op->data.nbytes - 1, regs_base + OSPI_DLR);

	clrsetbits_le32(regs_base + OSPI_CR, OSPI_CR_FMODE_MASK,
			mode << OSPI_CR_FMODE_SHIFT);

	ccr |= (stm32_ospi_get_mode(op->cmd.buswidth) << OSPI_CCR_IMODE_SHIFT) &
		OSPI_CCR_IMODE_MASK;

	if (op->addr.nbytes) {
		ccr |= ((op->addr.nbytes - 1) << OSPI_CCR_ADSIZE_SHIFT);
		ccr |= (stm32_ospi_get_mode(op->addr.buswidth)
			<< OSPI_CCR_ADMODE_SHIFT) & OSPI_CCR_ADMODE_MASK;
	}

	if (op->dummy.buswidth && op->dummy.nbytes)
		dcyc = op->dummy.nbytes * 8 / op->dummy.buswidth;

	clrsetbits_le32(regs_base + OSPI_TCR, OSPI_TCR_DCYC_MASK,
			dcyc << OSPI_TCR_DCYC_SHIFT);

	if (op->data.nbytes) {
		dmode = stm32_ospi_get_mode(op->data.buswidth);
		ccr |= (dmode << OSPI_CCR_DMODE_SHIFT) & OSPI_CCR_DMODE_MASK;
	}

	writel(ccr, regs_base + OSPI_CCR);

	/* set instruction, must be set after ccr register update */
	writel(op->cmd.opcode, regs_base + OSPI_IR);

	if (op->addr.nbytes && mode != OSPI_CCR_MEM_MAP)
		writel(op->addr.val, regs_base + OSPI_AR);

	ret = stm32_ospi_tx(priv, op, mode);
	/*
	 * Abort in:
	 * -error case
	 * -read memory map: prefetching must be stopped if we read the last
	 *  byte of device (device size - fifo size). like device size is not
	 *  knows, the prefetching is always stop.
	 */
	if (ret || mode == OSPI_CCR_MEM_MAP)
		goto abort;

	/* Wait end of tx in indirect mode */
	ret = stm32_omi_wait_cmd(priv->omi_dev);
	if (ret)
		goto abort;

	return 0;

abort:
	setbits_le32(regs_base + OSPI_CR, OSPI_CR_ABORT);

	/* Wait clear of abort bit by hw */
	timeout = readl_poll_timeout(regs_base + OSPI_CR, cr,
				     !(cr & OSPI_CR_ABORT),
				     OSPI_ABT_TIMEOUT_US);

	writel(OSPI_FCR_CTCF, regs_base + OSPI_FCR);

	if (ret || timeout)
		dev_err(slave->dev, "%s ret:%d abort timeout:%d\n", __func__,
			ret, timeout);

	return ret;
}

static int stm32_ospi_probe(struct udevice *bus)
{
	struct stm32_ospi_priv *priv = dev_get_priv(bus);
	struct stm32_omi_plat *omi_plat;
	phys_addr_t regs_base;
	int ret;

	priv->omi_dev = bus->parent;
	omi_plat = dev_get_plat(priv->omi_dev);
	regs_base = omi_plat->regs_base;

	ret = clk_enable(&omi_plat->clk);
	if (ret) {
		dev_err(bus, "failed to enable clock\n");
		return ret;
	}

	/* Reset OSPI controller */
	reset_assert_bulk(&omi_plat->rst_ctl);
	udelay(2);
	reset_deassert_bulk(&omi_plat->rst_ctl);

	priv->cs_used = -1;

	setbits_le32(regs_base + OSPI_TCR, OSPI_TCR_SSHIFT);

	/* Set dcr devsize to max address */
	setbits_le32(regs_base + OSPI_DCR1,
		     OSPI_DCR1_DEVSIZE_MASK | OSPI_DCR1_DLYBYP);

	return 0;
}

static int stm32_ospi_claim_bus(struct udevice *dev)
{
	struct stm32_ospi_priv *priv = dev_get_priv(dev->parent);
	struct dm_spi_slave_plat *slave_plat = dev_get_parent_plat(dev);
	struct stm32_omi_plat *omi_plat = dev_get_plat(priv->omi_dev);
	phys_addr_t regs_base = omi_plat->regs_base;
	int slave_cs = slave_plat->cs;

	if (slave_cs >= OSPI_MAX_CHIP)
		return -ENODEV;

	if (priv->cs_used != slave_cs) {
		struct stm32_ospi_flash *flash = &priv->flash[slave_cs];

		priv->cs_used = slave_cs;

		if (flash->initialized) {
			/* Set the configuration: speed + cs */
			writel(flash->cr, regs_base + OSPI_CR);
			writel(flash->dcr, regs_base + OSPI_DCR1);
			writel(flash->dcr2, regs_base + OSPI_DCR2);
		} else {
			/* Set chip select */
			clrsetbits_le32(regs_base + OSPI_CR,
					OSPI_CR_CSSEL,
					priv->cs_used ? OSPI_CR_CSSEL : 0);

			/* Save the configuration: speed + cs */
			flash->cr = readl(regs_base + OSPI_CR);
			flash->dcr = readl(regs_base + OSPI_DCR1);
			flash->dcr2 = readl(regs_base + OSPI_DCR2);
			flash->initialized = true;
		}
	}

	setbits_le32(regs_base + OSPI_CR, OSPI_CR_EN);

	return 0;
}

static int stm32_ospi_release_bus(struct udevice *dev)
{
	struct stm32_ospi_priv *priv = dev_get_priv(dev->parent);
	struct stm32_omi_plat *omi_plat = dev_get_plat(priv->omi_dev);
	phys_addr_t regs_base = omi_plat->regs_base;

	clrbits_le32(regs_base + OSPI_CR, OSPI_CR_EN);

	return 0;
}

static int stm32_ospi_set_speed(struct udevice *bus, uint speed)
{
	struct stm32_ospi_priv *priv = dev_get_priv(bus);
	struct stm32_omi_plat *omi_plat = dev_get_plat(priv->omi_dev);
	phys_addr_t regs_base = omi_plat->regs_base;
	u32 ospi_clk = omi_plat->clock_rate;
	u32 prescaler = 255;
	u32 csht;
	int ret;

	if (speed > 0) {
		prescaler = 0;
		if (ospi_clk) {
			prescaler = DIV_ROUND_UP(ospi_clk, speed) - 1;
			if (prescaler > 255)
				prescaler = 255;
		}
	}

	csht = (DIV_ROUND_UP((5 * ospi_clk) / (prescaler + 1), 100000000)) - 1;

	ret = stm32_omi_wait_for_not_busy(priv->omi_dev);
	if (ret)
		return ret;

	clrsetbits_le32(regs_base + OSPI_DCR2, OSPI_DCR2_PRESC_MASK,
			prescaler << OSPI_DCR2_PRESC_SHIFT);

	clrsetbits_le32(regs_base + OSPI_DCR1, OSPI_DCR1_CSHT_MASK,
			csht << OSPI_DCR1_CSHT_SHIFT);


	return 0;
}

static int stm32_ospi_set_mode(struct udevice *bus, uint mode)
{
	struct stm32_ospi_priv *priv = dev_get_priv(bus);
	struct stm32_omi_plat *omi_plat = dev_get_plat(priv->omi_dev);
	phys_addr_t regs_base = omi_plat->regs_base;
	const char *str_rx, *str_tx;
	int ret;

	ret = stm32_omi_wait_for_not_busy(priv->omi_dev);
	if (ret)
		return ret;

	if ((mode & SPI_CPHA) && (mode & SPI_CPOL))
		setbits_le32(regs_base + OSPI_DCR1, OSPI_DCR1_CKMODE);
	else if (!(mode & SPI_CPHA) && !(mode & SPI_CPOL))
		clrbits_le32(regs_base + OSPI_DCR1, OSPI_DCR1_CKMODE);
	else
		return -ENODEV;

	if (mode & SPI_CS_HIGH)
		return -ENODEV;

	if (mode & SPI_RX_OCTAL)
		str_rx = "octal";
	else if (mode & SPI_RX_QUAD)
		str_rx = "quad";
	else if (mode & SPI_RX_DUAL)
		str_rx = "dual";
	else
		str_rx = "single";

	if (mode & SPI_TX_OCTAL)
		str_tx = "octal";
	else if (mode & SPI_TX_QUAD)
		str_tx = "quad";
	else if (mode & SPI_TX_DUAL)
		str_tx = "dual";
	else
		str_tx = "single";

	dev_dbg(bus, "mode=%d rx: %s, tx: %s\n", mode, str_rx, str_tx);

	return 0;
}

static const struct spi_controller_mem_ops stm32_ospi_mem_ops = {
	.exec_op = stm32_ospi_exec_op,
};

static const struct dm_spi_ops stm32_ospi_ops = {
	.claim_bus	= stm32_ospi_claim_bus,
	.release_bus	= stm32_ospi_release_bus,
	.set_speed	= stm32_ospi_set_speed,
	.set_mode	= stm32_ospi_set_mode,
	.mem_ops	= &stm32_ospi_mem_ops,
};

static const struct udevice_id stm32_ospi_ids[] = {
	{ .compatible = "st,stm32mp25-ospi" },
	{ }
};

U_BOOT_DRIVER(stm32_ospi) = {
	.name = "stm32_ospi",
	.id = UCLASS_SPI,
	.of_match = stm32_ospi_ids,
	.ops = &stm32_ospi_ops,
	.priv_auto = sizeof(struct stm32_ospi_priv),
	.probe = stm32_ospi_probe,
};
