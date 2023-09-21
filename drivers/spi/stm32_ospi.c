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

#define NSEC_PER_SEC		1000000000L
#define MACRONIX_ID		0xc2

struct stm32_ospi_flash {
	u64 str_idcode;
	u64 dtr_idcode;
	bool is_spi_nor;
	bool is_str_calibration;
	bool dtr_calibration_done_once;
	bool octal_dtr;
};

struct stm32_ospi_priv {
	struct udevice *omi_dev;
	struct stm32_ospi_flash flash[OSPI_MAX_CHIP];
	int cs_used;
};

static int stm32_ospi_mm(struct udevice *omi_dev,
			 const struct spi_mem_op *op)
{
	struct stm32_omi_plat *omi_plat = dev_get_plat(omi_dev);

	memcpy_fromio(op->data.buf.in,
		      (void __iomem *)omi_plat->mm_base + op->addr.val,
		      op->data.nbytes);

	return 0;
}

static int stm32_ospi_tx(struct udevice *omi_dev,
			 const struct spi_mem_op *op,
			 u8 mode)
{
	struct stm32_omi_priv *omi_priv = dev_get_priv(omi_dev);
	struct stm32_ospi_priv *priv = dev_get_priv(omi_priv->dev);
	struct stm32_ospi_flash *flash = &priv->flash[priv->cs_used];
	u8 *buf;
	u8 dummy = 0xff;
	int ret;

	if (!op->data.nbytes)
		return 0;

	if (mode == OSPI_CCR_MEM_MAP)
		return stm32_ospi_mm(omi_dev, op);

	if (op->data.dir == SPI_MEM_DATA_IN)
		buf = op->data.buf.in;
	else
		buf = (u8 *)op->data.buf.out;

	if (flash->octal_dtr && op->addr.val % 2) {
		/* Read/write dummy byte */
		ret = stm32_omi_tx_poll(omi_dev, &dummy, 1,
					op->data.dir == SPI_MEM_DATA_IN);
		if (ret)
			return ret;
	}

	ret = stm32_omi_tx_poll(omi_dev, buf, op->data.nbytes,
				op->data.dir == SPI_MEM_DATA_IN);
	if (ret)
		return ret;

	if (flash->octal_dtr && (op->addr.val + op->data.nbytes) % 2) {
		/* Read/write dummy byte */
		ret = stm32_omi_tx_poll(omi_dev, &dummy, 1,
					op->data.dir == SPI_MEM_DATA_IN);
		if (ret)
			return ret;
	}

	return ret;
}

static int stm32_ospi_get_mode(u8 buswidth)
{
	if (buswidth == 8)
		return 4;

	if (buswidth == 4)
		return 3;

	return buswidth;
}

static int stm32_ospi_send(struct udevice *omi_dev,
			   const struct spi_mem_op *op, u8 mode)
{
	struct stm32_omi_plat *omi_plat = dev_get_plat(omi_dev);
	struct stm32_omi_priv *omi_priv = dev_get_priv(omi_dev);
	struct stm32_ospi_priv *priv = dev_get_priv(omi_priv->dev);
	struct stm32_ospi_flash *flash = &priv->flash[priv->cs_used];
	phys_addr_t regs_base = omi_plat->regs_base;
	u32 cr, ccr = 0;
	int timeout, ret;
	int dmode;
	u8 dcyc = 0;
	u64 addr = op->addr.val;
	unsigned int nbytes = op->data.nbytes;

	dev_dbg(omi_priv->dev, "%s: cmd:%#x dtr: %d mode:%d.%d.%d.%d addr:%#llx len:%#x\n",
		__func__, op->cmd.opcode, op->cmd.dtr, op->cmd.buswidth,
		op->addr.buswidth, op->dummy.buswidth, op->data.buswidth,
		op->addr.val, op->data.nbytes);

	/*
	 * When DTR mode and indirect read/write mode are set, there is a
	 * constraint on the address and the number of bytes read or write
	 * that should be even.
	 */
	if (flash->octal_dtr && mode != OSPI_CCR_MEM_MAP && op->data.nbytes) {
		if (op->addr.val % 2) {
			addr--;
			nbytes++;
		}

		if ((op->addr.val + op->data.nbytes) % 2)
			nbytes++;
	}

	clrbits_le32(regs_base + OSPI_DCR1, OSPI_DCR1_MTYP_MASK);

	if (op->data.nbytes && mode != OSPI_CCR_MEM_MAP)
		writel(nbytes - 1, regs_base + OSPI_DLR);

	clrsetbits_le32(regs_base + OSPI_CR, OSPI_CR_FMODE_MASK,
			mode << OSPI_CR_FMODE_SHIFT);

	ccr |= ((op->cmd.nbytes - 1) << OSPI_CCR_ISIZE_SHIFT) &
	       OSPI_CCR_ISIZE_MASK;

	ccr |= (stm32_ospi_get_mode(op->cmd.buswidth) << OSPI_CCR_IMODE_SHIFT) &
		OSPI_CCR_IMODE_MASK;

	if (op->cmd.dtr) {
		ccr |= OSPI_CCR_IDTR;
		ccr |= OSPI_CCR_DQSE;
	}

	if (op->addr.dtr)
		ccr |= OSPI_CCR_ADDTR;

	if (op->data.dtr)
		ccr |= OSPI_CCR_DDTR;

	if (op->data.dtr_swab16)
		clrsetbits_le32(regs_base + OSPI_DCR1, OSPI_DCR1_MTYP_MASK,
				OSPI_DCR1_MTYP_MX_MODE << OSPI_DCR1_MTYP_SHIFT);

	if (op->addr.nbytes) {
		ccr |= ((op->addr.nbytes - 1) << OSPI_CCR_ADSIZE_SHIFT);
		ccr |= (stm32_ospi_get_mode(op->addr.buswidth)
			<< OSPI_CCR_ADMODE_SHIFT) & OSPI_CCR_ADMODE_MASK;
	}

	if (op->dummy.buswidth && op->dummy.nbytes) {
		dcyc = op->dummy.nbytes * 8 / op->dummy.buswidth;

		if (op->dummy.dtr)
			dcyc /= 2;
	}

	clrsetbits_le32(regs_base + OSPI_TCR, OSPI_TCR_DCYC_MASK,
			dcyc << OSPI_TCR_DCYC_SHIFT);

	if (op->data.nbytes) {
		dmode = stm32_ospi_get_mode(op->data.buswidth);
		ccr |= (dmode << OSPI_CCR_DMODE_SHIFT) & OSPI_CCR_DMODE_MASK;
	}

	writel(ccr, regs_base + OSPI_CCR);

	/* Set instruction, must be set after ccr register update */
	writel(op->cmd.opcode, regs_base + OSPI_IR);

	if (op->addr.nbytes && mode != OSPI_CCR_MEM_MAP)
		writel(addr, regs_base + OSPI_AR);

	ret = stm32_ospi_tx(omi_dev, op, mode);
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
	ret = stm32_omi_wait_cmd(omi_dev);
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

	if (!omi_priv->is_calibrating && (ret || timeout))
		dev_err(omi_priv->dev, "%s ret:%d abort timeout:%d\n", __func__,
			ret, timeout);

	return ret;
}

static int stm32_ospi_set_speed(struct udevice *bus, uint speed)
{
	struct stm32_ospi_priv *priv = dev_get_priv(bus);
	struct stm32_omi_plat *omi_plat = dev_get_plat(priv->omi_dev);
	phys_addr_t regs_base = omi_plat->regs_base;
	u32 ospi_clk = omi_plat->clock_rate;
	u32 prescaler = 255;
	u32 csht;
	uint bus_freq;
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

	bus_freq = DIV_ROUND_UP(ospi_clk, prescaler + 1);
	if (bus_freq <= STM32_DLYB_FREQ_THRESHOLD)
		setbits_le32(regs_base + OSPI_DCR1, OSPI_DCR1_DLYBYP);
	else
		clrbits_le32(regs_base + OSPI_DCR1, OSPI_DCR1_DLYBYP);

	return 0;
}

static int stm32_ospi_readid(struct udevice *omi_dev)
{
	struct stm32_omi_priv *omi_priv = dev_get_priv(omi_dev);
	struct stm32_ospi_priv *priv = dev_get_priv(omi_priv->dev);
	struct stm32_ospi_flash *flash = &priv->flash[priv->cs_used];
	u64 rx_buf;
	struct spi_mem_op readid_op;
	int ret;

	if (flash->is_str_calibration) {
		u8 nb_dummy_bytes = flash->is_spi_nor ? 0 : 1;

		readid_op = (struct spi_mem_op)
			    SPI_MEM_OP(SPI_MEM_OP_CMD(0x9f, 1),
				       SPI_MEM_OP_NO_ADDR,
				       SPI_MEM_OP_DUMMY(nb_dummy_bytes, 1),
				       SPI_MEM_OP_DATA_IN(8, (u8 *)&rx_buf, 1));
	} else {
		if (flash->octal_dtr && flash->is_spi_nor) {
			u16 opcode;
			u8 nb_addr_bytes;
			u8 nb_dummy_bytes;

			if ((flash->dtr_idcode & 0xff) == MACRONIX_ID) {
				opcode = 0x9f60;
				nb_addr_bytes = 4;
				nb_dummy_bytes = 8;
			} else {
				/*
				 * All memory providers are not currently
				 * supported, feel free to add them
				 */
				return -EOPNOTSUPP;
			}

			readid_op = (struct spi_mem_op)
				    SPI_MEM_OP(SPI_MEM_OP_CMD(opcode, 8),
					       SPI_MEM_OP_ADDR(nb_addr_bytes, 0, 8),
					       SPI_MEM_OP_DUMMY(nb_dummy_bytes, 8),
					       SPI_MEM_OP_DATA_IN(8, (u8 *)&rx_buf, 8));
			readid_op.cmd.dtr = true;
			readid_op.addr.dtr = true;
			readid_op.dummy.dtr = true;
			readid_op.data.dtr = true;
			readid_op.cmd.nbytes = 2;
		} else {
			/*
			 * Only OCTAL DTR calibration on SPI NOR devices
			 * is currently supported
			 */
			return -EOPNOTSUPP;
		}
	}

	ret = stm32_ospi_send(omi_dev, &readid_op, OSPI_CCR_IND_READ);
	if (ret)
		return ret;

	dev_dbg(omi_dev, "Flash ID 0x%08llx\n", rx_buf);

	if (flash->is_str_calibration) {
		/*
		 * On stm32_ospi_readid() first execution, save the golden
		 * read id
		 */
		if (flash->str_idcode == 0) {
			flash->str_idcode = rx_buf;

			if (flash->is_spi_nor) {
				/* Build DTR id code */
				if ((rx_buf & 0xff) == MACRONIX_ID) {
					/*
					 * Retrieve odd array and re-sort id
					 * because of read id format will be
					 * A-A-B-B-C-C after enter into octal
					 * dtr mode for Macronix flashes.
					 */
					flash->dtr_idcode = rx_buf & 0xff;
					flash->dtr_idcode |= (rx_buf & 0xff) << 8;
					flash->dtr_idcode |= (rx_buf & 0xff00) << 8;
					flash->dtr_idcode |= (rx_buf & 0xff00) << 16;
					flash->dtr_idcode |= (rx_buf & 0xff0000) << 16;
					flash->dtr_idcode |= (rx_buf & 0xff0000) << 24;
					flash->dtr_idcode |= (rx_buf & 0xff000000) << 24;
					flash->dtr_idcode |= (rx_buf & 0xff000000) << 32;
				} else {
					flash->dtr_idcode = rx_buf;
				}
			}
		}

		if (rx_buf == flash->str_idcode)
			return 0;
	} else if (rx_buf == flash->dtr_idcode) {
		return 0;
	}

	return -EIO;
}

static int stm32_ospi_str_calibration(struct udevice *bus)
{
	struct stm32_ospi_priv *priv = dev_get_priv(bus);
	struct stm32_omi_plat *omi_plat = dev_get_plat(priv->omi_dev);
	struct stm32_ospi_flash *flash = &priv->flash[priv->cs_used];
	phys_addr_t regs_base = omi_plat->regs_base;
	u32 dlyb_cr;
	u8 window_len_tcr0 = 0, window_len_tcr1 = 0;
	int ret, ret_tcr0, ret_tcr1;
	u32 dcr2, prescaler;
	uint bus_freq;

	dcr2 = readl(regs_base + OSPI_DCR2);
	prescaler = (dcr2 & OSPI_DCR2_PRESC_MASK) >> OSPI_DCR2_PRESC_SHIFT;
	bus_freq = DIV_ROUND_UP(omi_plat->clock_rate, prescaler + 1);

	/*
	 * Set memory device at low frequency (50MHz) and sent
	 * READID (0x9F) command, save the answer as golden answer
	 */
	ret = stm32_ospi_set_speed(bus, STM32_DLYB_FREQ_THRESHOLD);
	if (ret)
		return ret;

	flash->str_idcode = 0;
	ret = stm32_ospi_readid(priv->omi_dev);
	if (ret)
		return ret;

	/* Set frequency at requested value */
	ret = stm32_ospi_set_speed(bus, bus_freq);
	if (ret)
		return ret;

	/* Calibration needed above 50MHz */
	if (bus_freq <= STM32_DLYB_FREQ_THRESHOLD)
		return 0;

	/* Perform calibration */
	ret = stm32_omi_dlyb_configure(priv->omi_dev, false, 0);
	if (ret)
		return ret;

	ret_tcr0 = stm32_omi_dlyb_find_tap(priv->omi_dev, true, &window_len_tcr0);
	if (!ret_tcr0)
		stm32_omi_dlyb_get_cr(priv->omi_dev, &dlyb_cr);

	stm32_omi_dlyb_stop(priv->omi_dev);

	ret = stm32_omi_dlyb_configure(priv->omi_dev, false, 0);
	if (ret)
		return ret;

	setbits_le32(regs_base + OSPI_TCR, OSPI_TCR_SSHIFT);

	ret_tcr1 = stm32_omi_dlyb_find_tap(priv->omi_dev, true, &window_len_tcr1);
	if (ret_tcr0 && ret_tcr1) {
		dev_info(bus, "Calibration phase failed\n");
		return ret_tcr0;
	}

	if (window_len_tcr0 >= window_len_tcr1) {
		clrbits_le32(regs_base + OSPI_TCR, OSPI_TCR_SSHIFT);

		stm32_omi_dlyb_stop(priv->omi_dev);

		ret = stm32_omi_dlyb_set_cr(priv->omi_dev, dlyb_cr);
		if (ret)
			return ret;
	}

	return 0;
}

static int stm32_ospi_dtr_calibration(struct udevice *bus)
{
	struct stm32_ospi_priv *priv = dev_get_priv(bus);
	struct stm32_omi_plat *omi_plat = dev_get_plat(priv->omi_dev);
	phys_addr_t regs_base = omi_plat->regs_base;
	u32 dcr2, prescaler;
	uint bus_freq;
	u16 period_ps = 0;
	u8 window_len = 0;
	int ret;
	bool bypass_mode = false;

	clrbits_le32(regs_base + OSPI_DCR1, OSPI_DCR1_DLYBYP);

	dcr2 = readl(regs_base + OSPI_DCR2);
	prescaler = (dcr2 & OSPI_DCR2_PRESC_MASK) >> OSPI_DCR2_PRESC_SHIFT;
	bus_freq = DIV_ROUND_UP(omi_plat->clock_rate, prescaler + 1);

	if (prescaler)
		setbits_le32(regs_base + OSPI_TCR, OSPI_TCR_DHQC);

	if (bus_freq <= STM32_DLYB_FREQ_THRESHOLD) {
		bypass_mode = true;
		period_ps = NSEC_PER_SEC / (bus_freq / 1000);
	}

	ret = stm32_omi_dlyb_configure(priv->omi_dev, bypass_mode, period_ps);
	if (ret)
		return ret;

	if (bypass_mode || prescaler)
		/* Perform only RX TAP selection */
		ret = stm32_omi_dlyb_find_tap(priv->omi_dev, true, &window_len);
	else
		/* Perform RX/TX TAP selection */
		ret = stm32_omi_dlyb_find_tap(priv->omi_dev, false, &window_len);

	if (ret) {
		dev_err(bus, "Calibration failed\n");
		if (!bypass_mode)
			/* Stop delay block when configured in lock mode */
			stm32_omi_dlyb_stop(priv->omi_dev);
	}

	return ret;
}

static int stm32_ospi_exec_op(struct spi_slave *slave,
			      const struct spi_mem_op *op)
{
	struct stm32_ospi_priv *priv = dev_get_priv(slave->dev->parent);
	struct stm32_omi_plat *omi_plat = dev_get_plat(priv->omi_dev);
	struct stm32_ospi_flash *flash = &priv->flash[priv->cs_used];
	phys_addr_t regs_base = omi_plat->regs_base;
	u32 addr_max;
	u8 mode = OSPI_CCR_IND_WRITE;
	int ret;

	if (op->cmd.dtr && !flash->dtr_calibration_done_once) {
		stm32_omi_dlyb_stop(priv->omi_dev);
		clrbits_le32(regs_base + OSPI_TCR, OSPI_TCR_SSHIFT);
		flash->octal_dtr = (op->cmd.nbytes == 2);

		ret = stm32_ospi_dtr_calibration(slave->dev->parent);
		if (ret)
			return ret;

		flash->dtr_calibration_done_once = true;
	}

	addr_max = op->addr.val + op->data.nbytes + 1;

	if (op->data.dir == SPI_MEM_DATA_IN && op->data.nbytes) {
		if (addr_max < omi_plat->mm_size && op->addr.buswidth)
			mode = OSPI_CCR_MEM_MAP;
		else
			mode = OSPI_CCR_IND_READ;
	}

	return stm32_ospi_send(priv->omi_dev, op, mode);
}

static int stm32_ospi_probe(struct udevice *bus)
{
	struct stm32_ospi_priv *priv = dev_get_priv(bus);
	struct stm32_omi_plat *omi_plat;
	struct stm32_omi_priv *omi_priv;
	phys_addr_t regs_base;
	ofnode child;
	int ret;

	priv->omi_dev = bus->parent;
	omi_plat = dev_get_plat(priv->omi_dev);
	omi_priv = dev_get_priv(priv->omi_dev);
	omi_priv->dev = bus;
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

	/* Set dcr devsize to max address */
	setbits_le32(regs_base + OSPI_DCR1, OSPI_DCR1_DEVSIZE_MASK);

	priv->cs_used = -1;
	omi_priv->check_transfer = stm32_ospi_readid;

	/* Find memory model on each child node (SPI NOR or SPI NAND) */
	dev_for_each_subnode(child, priv->omi_dev) {
		u32 cs;

		ret = ofnode_read_u32(child, "reg", &cs);
		if (ret) {
			dev_err(bus, "could not retrieve reg property: %d\n",
				ret);
			return ret;
		}

		if (cs >= OSPI_MAX_CHIP) {
			dev_err(bus, "invalid reg value: %d\n", cs);
			return -EINVAL;
		}

		if (ofnode_device_is_compatible(child, "jedec,spi-nor")) {
			struct stm32_ospi_flash *flash = &priv->flash[cs];

			flash->is_spi_nor = true;
		}
	}

	return 0;
}

static int stm32_ospi_claim_bus(struct udevice *dev)
{
	struct stm32_ospi_priv *priv = dev_get_priv(dev->parent);
	struct dm_spi_slave_plat *slave_plat = dev_get_parent_plat(dev);
	struct stm32_omi_plat *omi_plat = dev_get_plat(priv->omi_dev);
	struct stm32_ospi_flash *flash;
	phys_addr_t regs_base = omi_plat->regs_base;
	int slave_cs = slave_plat->cs;
	int ret;

	if (slave_cs >= OSPI_MAX_CHIP)
		return -ENODEV;

	setbits_le32(regs_base + OSPI_CR, OSPI_CR_EN);

	if (priv->cs_used == slave_cs)
		return 0;

	priv->cs_used = slave_cs;
	flash = &priv->flash[priv->cs_used];

	stm32_omi_dlyb_stop(priv->omi_dev);

	/* Set chip select */
	clrsetbits_le32(regs_base + OSPI_CR, OSPI_CR_CSSEL,
			priv->cs_used ? OSPI_CR_CSSEL : 0);
	clrbits_le32(regs_base + OSPI_TCR, OSPI_TCR_SSHIFT);

	if (flash->dtr_calibration_done_once) {
		ret = stm32_ospi_dtr_calibration(dev->parent);
	} else {
		flash->is_str_calibration = true;

		ret = stm32_ospi_str_calibration(dev->parent);
		if (ret) {
			dev_info(dev->parent, "Set flash frequency to a safe value (%d Hz)\n",
				 STM32_DLYB_FREQ_THRESHOLD);

			stm32_omi_dlyb_stop(priv->omi_dev);

			clrbits_le32(regs_base + OSPI_TCR, OSPI_TCR_SSHIFT);
			ret = stm32_ospi_set_speed(dev->parent,
						   STM32_DLYB_FREQ_THRESHOLD);
		}

		flash->is_str_calibration = false;
	}

	return ret;
}

static int stm32_ospi_release_bus(struct udevice *dev)
{
	struct stm32_ospi_priv *priv = dev_get_priv(dev->parent);
	struct stm32_omi_plat *omi_plat = dev_get_plat(priv->omi_dev);
	phys_addr_t regs_base = omi_plat->regs_base;

	clrbits_le32(regs_base + OSPI_CR, OSPI_CR_EN);

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

static bool stm32_ospi_mem_supports_op(struct spi_slave *slave,
				       const struct spi_mem_op *op)
{
	if (op->data.buswidth > 8 || op->addr.buswidth > 8 ||
	    op->dummy.buswidth > 8 || op->cmd.buswidth > 8)
		return false;

	if (op->cmd.nbytes > 4 || op->addr.nbytes > 4)
		return false;

	if ((!op->dummy.dtr && op->dummy.nbytes > 32) ||
	    (op->dummy.dtr && op->dummy.nbytes > 64))
		return false;

	if (!op->cmd.dtr && !op->addr.dtr && !op->dummy.dtr &&
	    !op->data.dtr && op->cmd.nbytes == 1)
		return spi_mem_default_supports_op(slave, op);

	return spi_mem_dtr_supports_op(slave, op);
}

static const struct spi_controller_mem_ops stm32_ospi_mem_ops = {
	.exec_op = stm32_ospi_exec_op,
	.supports_op = stm32_ospi_mem_supports_op,
};

static const struct dm_spi_ops stm32_ospi_ops = {
	.claim_bus	= stm32_ospi_claim_bus,
	.release_bus	= stm32_ospi_release_bus,
	.set_speed	= stm32_ospi_set_speed,
	.set_mode	= stm32_ospi_set_mode,
	.mem_ops	= &stm32_ospi_mem_ops,
};

U_BOOT_DRIVER(stm32_ospi) = {
	.name = "stm32_ospi",
	.id = UCLASS_SPI,
	.ops = &stm32_ospi_ops,
	.priv_auto = sizeof(struct stm32_ospi_priv),
	.probe = stm32_ospi_probe,
};
