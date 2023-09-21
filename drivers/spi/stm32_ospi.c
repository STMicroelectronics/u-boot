// SPDX-License-Identifier: GPL-2.0-or-later OR BSD-3-Clause
/*
 * Copyright (C) 2025, STMicroelectronics - All Rights Reserved
 */

#define LOG_CATEGORY UCLASS_SPI

#include <clk.h>
#include <dm.h>
#include <log.h>
#include <regmap.h>
#include <reset.h>
#include <spi.h>
#include <spi-mem.h>
#include <syscon.h>
#include <asm/io.h>
#include <dm/device_compat.h>
#include <linux/bitops.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>
#include <linux/sizes.h>

/* OCTOSPI control register */
#define OSPI_CR			0x00
#define OSPI_CR_EN		BIT(0)
#define OSPI_CR_ABORT		BIT(1)
#define OSPI_CR_TCEN		BIT(3)
#define OSPI_CR_FSEL		BIT(7)
#define OSPI_CR_FTHRES_MASK	GENMASK(13, 8)
#define OSPI_CR_FTHRES_SHIFT	8
#define OSPI_CR_CSSEL		BIT(24)
#define OSPI_CR_FMODE_SHIFT	28
#define OSPI_CR_FMODE_MASK	GENMASK(29, 28)

/* OCTOSPI device configuration register */
#define OSPI_DCR1		0x08
#define OSPI_DCR1_CKMODE	BIT(0)
#define OSPI_DCR1_DLYBYP	BIT(3)
#define OSPI_DCR1_CSHT_SHIFT	8
#define OSPI_DCR1_CSHT_MASK	GENMASK(13, 8)
#define OSPI_DCR1_DEVSIZE_MASK	GENMASK(20, 16)
#define OSPI_DCR1_MTYP_MASK	GENMASK(26, 24)
#define OSPI_DCR1_MTYP_SHIFT	24
#define OSPI_DCR1_MTYP_MX_MODE	1

/* OCTOSPI device configuration register 2 */
#define OSPI_DCR2		0x0c
#define OSPI_DCR2_PRESC_SHIFT	0
#define OSPI_DCR2_PRESC_MASK	GENMASK(7, 0)

/* OCTOSPI status register */
#define OSPI_SR			0x20
#define OSPI_SR_TEF		BIT(0)
#define OSPI_SR_TCF		BIT(1)
#define OSPI_SR_FTF		BIT(2)
#define OSPI_SR_BUSY		BIT(5)

/* OCTOSPI flag clear register */
#define OSPI_FCR		0x24
#define OSPI_FCR_CTEF		BIT(0)
#define OSPI_FCR_CTCF		BIT(1)

/* OCTOSPI data length register */
#define OSPI_DLR		0x40

/* OCTOSPI address register */
#define OSPI_AR			0x48

/* OCTOSPI data configuration register */
#define OSPI_DR			0x50

/* OCTOSPI communication configuration register */
#define OSPI_CCR		0x100
#define OSPI_CCR_IMODE_SHIFT	0
#define OSPI_CCR_IMODE_MASK	GENMASK(2, 0)
#define OSPI_CCR_IDTR		BIT(3)
#define OSPI_CCR_ISIZE_SHIFT	4
#define OSPI_CCR_ISIZE_MASK	GENMASK(5, 4)
#define OSPI_CCR_ADMODE_SHIFT	8
#define OSPI_CCR_ADMODE_MASK	GENMASK(10, 8)
#define OSPI_CCR_ADDTR		BIT(11)
#define OSPI_CCR_ADSIZE_SHIFT	12
#define OSPI_CCR_DMODE_SHIFT	24
#define OSPI_CCR_DMODE_MASK	GENMASK(26, 24)
#define OSPI_CCR_IND_WRITE	0
#define OSPI_CCR_IND_READ	1
#define OSPI_CCR_MEM_MAP	3
#define OSPI_CCR_DDTR		BIT(27)
#define OSPI_CCR_DQSE		BIT(29)

/* OCTOSPI timing configuration register */
#define OSPI_TCR		0x108
#define OSPI_TCR_DCYC_SHIFT	0x0
#define OSPI_TCR_DCYC_MASK	GENMASK(4, 0)
#define OSPI_TCR_DHQC		BIT(28)
#define OSPI_TCR_SSHIFT		BIT(30)

/* OCTOSPI instruction register */
#define OSPI_IR			0x110

#define SYSCFG_DLYBOS_CR		0
#define DLYBOS_CR_EN			BIT(0)
#define DLYBOS_CR_RXTAPSEL_SHIFT	1
#define DLYBOS_CR_RXTAPSEL_MASK		GENMASK(6, 1)
#define DLYBOS_CR_TXTAPSEL_SHIFT	7
#define DLYBOS_CR_TXTAPSEL_MASK		GENMASK(12, 7)
#define DLYBOS_TAPSEL_NB		33
#define DLYBOS_BYP_EN			BIT(16)
#define DLYBOS_BYP_CMD_MASK		GENMASK(21, 17)

#define SYSCFG_DLYBOS_SR	4
#define DLYBOS_SR_LOCK		BIT(0)
#define DLYBOS_SR_RXTAPSEL_ACK	BIT(1)
#define DLYBOS_SR_TXTAPSEL_ACK	BIT(2)

#define OSPI_MAX_MMAP_SZ	SZ_256M
#define OSPI_MAX_CHIP		2

#define OSPI_FIFO_TIMEOUT_US	30000
#define OSPI_ABT_TIMEOUT_US	100000
#define OSPI_BUSY_TIMEOUT_US	100000
#define OSPI_CMD_TIMEOUT_US	1000000

#define STM32_DLYB_FREQ_THRESHOLD	50000000
#define STM32_DLYBOS_TIMEOUT_MS		1000
#define STM32_DLYBOS_DELAY_NB		24

#define NSEC_PER_SEC		1000000000L
#define MACRONIX_ID		0xc2

struct stm32_tap_window {
	u8 end;
	u8 length;
};

struct stm32_ospi_flash {
	u64 str_idcode;
	u64 dtr_idcode;
	bool is_spi_nor;
	bool is_str_calibration;
	bool dtr_calibration_done_once;
	bool octal_dtr;
};

struct stm32_ospi_priv {
	struct stm32_ospi_flash flash[OSPI_MAX_CHIP];
	int cs_used;
	u64 idcode;
	u32 prescaler[OSPI_MAX_CHIP];
	int (*check_transfer)(struct udevice * dev);
	bool is_calibrating;
};

struct stm32_ospi_plat {
	phys_addr_t regs_base;		/* register base address */
	phys_addr_t mm_base;		/* memory map base address */
	resource_size_t mm_size;
	struct clk clk;
	struct reset_ctl_bulk rst_ctl;
	struct regmap *regmap;
	ulong clock_rate;
	u32 dlyb_base;
};

static int stm32_ospi_mm(struct udevice *dev,
			 const struct spi_mem_op *op)
{
	struct stm32_ospi_plat *ospi_plat = dev_get_plat(dev);

	memcpy_fromio(op->data.buf.in,
		      (void __iomem *)ospi_plat->mm_base + op->addr.val,
		      op->data.nbytes);

	return 0;
}

static void stm32_ospi_read_fifo(void *val, phys_addr_t addr, u8 len)
{
	switch (len) {
	case sizeof(u32):
		*((u32 *)val) = readl_relaxed(addr);
		break;
	case sizeof(u16):
		*((u16 *)val) = readw_relaxed(addr);
		break;
	case sizeof(u8):
		*((u8 *)val) = readb_relaxed(addr);
	};
}

static void stm32_ospi_write_fifo(void *val, phys_addr_t addr, u8 len)
{
	switch (len) {
	case sizeof(u32):
		writel_relaxed(*((u32 *)val), addr);
		break;
	case sizeof(u16):
		writew_relaxed(*((u16 *)val), addr);
		break;
	case sizeof(u8):
		writeb_relaxed(*((u8 *)val), addr);
	};
}

int stm32_ospi_tx_poll(struct udevice *dev, void *buf, u32 len, bool read)
{
	struct stm32_ospi_plat *ospi_plat = dev_get_plat(dev);
	struct stm32_ospi_priv *priv = dev_get_priv(dev);
	phys_addr_t regs_base = ospi_plat->regs_base;
	void (*fifo)(void *val, phys_addr_t addr, u8 len);
	u32 sr;
	int ret;
	u8 step = 1;

	if (read)
		fifo = stm32_ospi_read_fifo;
	else
		fifo = stm32_ospi_write_fifo;

	while (len) {
		ret = readl_poll_timeout(regs_base + OSPI_SR, sr,
					 sr & OSPI_SR_FTF,
					 OSPI_FIFO_TIMEOUT_US);
		if (ret) {
			if (!priv->is_calibrating)
				dev_err(dev, "fifo timeout (len:%d stat:%#x)\n", len, sr);
			return ret;
		}

		if (!IS_ALIGNED((uintptr_t)buf, sizeof(u32))) {
			if (!IS_ALIGNED((uintptr_t)buf, sizeof(u16)))
				step = sizeof(u8);
			else
				step = min((u32)len, (u32)sizeof(u16));
		}
		/* Buf is aligned */
		else if (len >= sizeof(u32))
			step = sizeof(u32);
		else if (len >= sizeof(u16))
			step = sizeof(u16);
		else if (len)
			step = sizeof(u8);

		fifo(buf, regs_base + OSPI_DR, step);
		len -= step;
		buf += step;

		if (!(len % SZ_1M))
			schedule();
	}

	return 0;
}

static int stm32_ospi_tx(struct udevice *dev,
			 const struct spi_mem_op *op,
			 u8 mode)
{
	struct stm32_ospi_priv *priv = dev_get_priv(dev);
	struct stm32_ospi_flash *flash = &priv->flash[priv->cs_used];
	void *buf;
	u8 dummy = 0xff;
	int ret;

	if (!op->data.nbytes)
		return 0;

	if (mode == OSPI_CCR_MEM_MAP)
		return stm32_ospi_mm(dev, op);

	if (op->data.dir == SPI_MEM_DATA_IN)
		buf = op->data.buf.in;
	else
		buf = (void *)op->data.buf.out;

	if (flash->octal_dtr && op->addr.val % 2) {
		/* Read/write dummy byte */
		ret = stm32_ospi_tx_poll(dev, &dummy, 1,
					 op->data.dir == SPI_MEM_DATA_IN);
		if (ret)
			return ret;
	}

	ret = stm32_ospi_tx_poll(dev, buf, op->data.nbytes,
				op->data.dir == SPI_MEM_DATA_IN);
	if (ret)
		return ret;

	if (flash->octal_dtr && (op->addr.val + op->data.nbytes) % 2) {
		/* Read/write dummy byte */
		ret = stm32_ospi_tx_poll(dev, &dummy, 1,
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

int stm32_ospi_wait_for_not_busy(struct udevice *dev)
{
	struct stm32_ospi_plat *ospi_plat = dev_get_plat(dev);
	phys_addr_t regs_base = ospi_plat->regs_base;
	u32 sr;
	int ret;

	ret = readl_poll_timeout(regs_base + OSPI_SR, sr, !(sr & OSPI_SR_BUSY),
				 OSPI_BUSY_TIMEOUT_US);
	if (ret)
		dev_err(dev, "busy timeout (stat:%#x)\n", sr);

	return ret;
}

int stm32_ospi_wait_cmd(struct udevice *dev)
{
	struct stm32_ospi_plat *ospi_plat = dev_get_plat(dev);
	phys_addr_t regs_base = ospi_plat->regs_base;
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
		ret = stm32_ospi_wait_for_not_busy(dev);

	return ret;
}

static int stm32_ospi_send(struct udevice *dev, const struct spi_mem_op *op, u8 mode)
{
	struct stm32_ospi_plat *plat = dev_get_plat(dev);
	struct stm32_ospi_priv *priv = dev_get_priv(dev);
	phys_addr_t regs_base = plat->regs_base;
	struct stm32_ospi_flash *flash = &priv->flash[priv->cs_used];
	u32 cr, ccr = 0;
	int timeout, ret;
	int dmode;
	u8 dcyc = 0;
	u64 addr = op->addr.val;
	unsigned int nbytes = op->data.nbytes;

	dev_dbg(dev, "%s: cmd:%#x dtr: %d mode:%d.%d.%d.%d addr:%#llx len:%#x\n",
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

	if (op->data.swap16)
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

	ret = stm32_ospi_tx(dev, op, mode);
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
	ret = stm32_ospi_wait_cmd(dev);
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

	if (!priv->is_calibrating && (ret || timeout))
		dev_err(dev, "%s ret:%d abort timeout:%d\n", __func__, ret, timeout);

	return ret;
}

static int stm32_ospi_readid(struct udevice *dev)
{
	struct stm32_ospi_priv *priv = dev_get_priv(dev);
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

	ret = stm32_ospi_send(dev, &readid_op, OSPI_CCR_IND_READ);
	if (ret)
		return ret;

	dev_dbg(dev, "Flash ID 0x%08llx\n", rx_buf);

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

static int stm32_ospi_set_speed(struct udevice *bus, uint speed)
{
	struct stm32_ospi_plat *ospi_plat = dev_get_plat(bus);
	phys_addr_t regs_base = ospi_plat->regs_base;
	u32 ospi_clk = ospi_plat->clock_rate;
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

	ret = stm32_ospi_wait_for_not_busy(bus);
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

static int stm32_ospi_dlyb_set_tap(struct udevice *dev, u8 tap, bool rx_tap)
{
	struct stm32_ospi_plat *plat = dev_get_plat(dev);
	u32 sr, mask, ack;
	int ret;
	u8 shift;

	if (!plat->regmap || !plat->dlyb_base)
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

	regmap_update_bits(plat->regmap,
			   plat->dlyb_base + SYSCFG_DLYBOS_CR,
			   mask, mask & (tap << shift));

	ret = regmap_read_poll_timeout(plat->regmap,
				       plat->dlyb_base + SYSCFG_DLYBOS_SR,
				       sr, sr & ack, 1,
				       STM32_DLYBOS_TIMEOUT_MS);
	if (ret)
		dev_err(dev, "%s delay block phase configuration timeout\n",
			rx_tap ? "RX" : "TX");

	return ret;
}

int stm32_ospi_dlyb_find_tap(struct udevice *dev, bool rx_only, u8 *window_len)
{
	struct stm32_ospi_priv *priv = dev_get_priv(dev);
	struct stm32_tap_window rx_tap_w[DLYBOS_TAPSEL_NB];
	int ret;
	u8 rx_len, rx_window_len, rx_window_end;
	u8 tx_len, tx_window_len, tx_window_end;
	u8 rx_tap, tx_tap, tx_tap_max, tx_tap_min, best_tx_tap = 0;
	u8 score, score_max;

	tx_len = 0;
	tx_window_len = 0;
	tx_window_end = 0;
	priv->is_calibrating = true;

	for (tx_tap = 0;
	     tx_tap < (rx_only ? 1 : DLYBOS_TAPSEL_NB);
	     tx_tap++) {
		ret = stm32_ospi_dlyb_set_tap(dev, tx_tap, false);
		if (ret)
			return ret;

		rx_len = 0;
		rx_window_len = 0;
		rx_window_end = 0;

		for (rx_tap = 0; rx_tap < DLYBOS_TAPSEL_NB; rx_tap++) {
			ret = stm32_ospi_dlyb_set_tap(dev, rx_tap, true);
			if (ret)
				return ret;

			ret = priv->check_transfer(dev);
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

	priv->is_calibrating = false;

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

		return stm32_ospi_dlyb_set_tap(dev, rx_tap, true);
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
	ret = stm32_ospi_dlyb_set_tap(dev, rx_tap, true);
	if (ret)
		return ret;

	dev_dbg(dev, "TX_TAP_SEL set to %d\n", best_tx_tap);

	return stm32_ospi_dlyb_set_tap(dev, best_tx_tap, false);
}

static void stm32_ospi_dlyb_stop(struct udevice *dev)
{
	struct stm32_ospi_plat *plat = dev_get_plat(dev);

	/* disable delay block */
	regmap_write(plat->regmap,
		     plat->dlyb_base + SYSCFG_DLYBOS_CR,
		     0x0);
}

/* ½ memory clock period in pico second */
static const u16 dlybos_delay_ps[STM32_DLYBOS_DELAY_NB] = {
2816, 4672, 6272, 7872, 9472, 11104, 12704, 14304, 15904, 17536, 19136, 20736,
22336, 23968, 25568, 27168, 28768, 30400, 32000, 33600, 35232, 36832, 38432, 40032
};

static u32 stm32_ospi_find_byp_cmd(u16 period_ps)
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

static int stm32_ospi_dlyb_configure(struct udevice *dev,
				     bool bypass_mode, u16 period_ps)
{
	struct stm32_ospi_plat *plat = dev_get_plat(dev);
	u32 sr, mask, val;
	int ret;

	if (!plat->regmap || !plat->dlyb_base)
		return -EINVAL;

	if (bypass_mode) {
		val = DLYBOS_BYP_EN | stm32_ospi_find_byp_cmd(period_ps);
		mask = DLYBOS_BYP_EN | DLYBOS_BYP_CMD_MASK;
	} else {
		val = DLYBOS_CR_EN;
		mask = DLYBOS_CR_EN;
	}

	regmap_update_bits(plat->regmap,
			   plat->dlyb_base + SYSCFG_DLYBOS_CR,
			   mask, val);

	if (bypass_mode)
		return 0;

	/* in lock mode, wait for lock status bit */
	ret = regmap_read_poll_timeout(plat->regmap,
				       plat->dlyb_base + SYSCFG_DLYBOS_SR,
				       sr, sr & DLYBOS_SR_LOCK, 1,
				       STM32_DLYBOS_TIMEOUT_MS);
	if (ret) {
		dev_err(dev, "Delay Block lock timeout\n");
		stm32_ospi_dlyb_stop(dev);
	}

	return ret;
}

int stm32_ospi_dlyb_set_cr(struct udevice *dev, u32 dlyb_cr)
{
	bool bypass_mode = false;
	int ret;
	u16 period_ps;
	u8 rx_tap, tx_tap;

	period_ps = FIELD_GET(DLYBOS_BYP_CMD_MASK, dlyb_cr);
	if (dlyb_cr & DLYBOS_BYP_EN)
		bypass_mode = true;

	ret = stm32_ospi_dlyb_configure(dev, bypass_mode, period_ps);
	if (ret)
		return ret;

	/* restore Rx and TX tap */
	rx_tap = FIELD_GET(DLYBOS_CR_RXTAPSEL_MASK, dlyb_cr);
	ret = stm32_ospi_dlyb_set_tap(dev, rx_tap, true);
	if (ret)
		return ret;

	tx_tap = FIELD_GET(DLYBOS_CR_TXTAPSEL_MASK, dlyb_cr);
	return stm32_ospi_dlyb_set_tap(dev, tx_tap, false);
}

void stm32_ospi_dlyb_get_cr(struct udevice *dev, u32 *dlyb_cr)
{
	struct stm32_ospi_plat *plat = dev_get_plat(dev);

	regmap_read(plat->regmap, plat->dlyb_base + SYSCFG_DLYBOS_CR, dlyb_cr);
}

static int stm32_ospi_str_calibration(struct udevice *bus)
{
	struct stm32_ospi_priv *priv = dev_get_priv(bus);
	struct stm32_ospi_plat *plat = dev_get_plat(bus);
	struct stm32_ospi_flash *flash = &priv->flash[priv->cs_used];
	phys_addr_t regs_base = plat->regs_base;
	u32 dlyb_cr;
	u8 window_len_tcr0 = 0, window_len_tcr1 = 0;
	int ret, ret_tcr0, ret_tcr1;
	u32 dcr2, prescaler;
	uint bus_freq;

	dcr2 = readl(regs_base + OSPI_DCR2);
	prescaler = (dcr2 & OSPI_DCR2_PRESC_MASK) >> OSPI_DCR2_PRESC_SHIFT;
	bus_freq = DIV_ROUND_UP(plat->clock_rate, prescaler + 1);

	/*
	 * Set memory device at low frequency (50MHz) and sent
	 * READID (0x9F) command, save the answer as golden answer
	 */
	ret = stm32_ospi_set_speed(bus, STM32_DLYB_FREQ_THRESHOLD);
	if (ret)
		return ret;

	flash->str_idcode = 0;
	ret = stm32_ospi_readid(bus);
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
	ret = stm32_ospi_dlyb_configure(bus, false, 0);
	if (ret)
		return ret;

	ret_tcr0 = stm32_ospi_dlyb_find_tap(bus, true, &window_len_tcr0);
	if (!ret_tcr0)
		stm32_ospi_dlyb_get_cr(bus, &dlyb_cr);

	stm32_ospi_dlyb_stop(bus);

	ret = stm32_ospi_dlyb_configure(bus, false, 0);
	if (ret)
		return ret;

	setbits_le32(regs_base + OSPI_TCR, OSPI_TCR_SSHIFT);

	ret_tcr1 = stm32_ospi_dlyb_find_tap(bus, true, &window_len_tcr1);
	if (ret_tcr0 && ret_tcr1) {
		dev_info(bus, "Calibration phase failed\n");
		return ret_tcr0;
	}

	if (window_len_tcr0 >= window_len_tcr1) {
		clrbits_le32(regs_base + OSPI_TCR, OSPI_TCR_SSHIFT);

		stm32_ospi_dlyb_stop(bus);

		ret = stm32_ospi_dlyb_set_cr(bus, dlyb_cr);
		if (ret)
			return ret;
	}

	return 0;
}

static int stm32_ospi_dtr_calibration(struct udevice *bus)
{
	struct stm32_ospi_plat *plat = dev_get_plat(bus);
	phys_addr_t regs_base = plat->regs_base;
	u32 dcr2, prescaler;
	uint bus_freq;
	u16 period_ps = 0;
	u8 window_len = 0;
	int ret;
	bool bypass_mode = false;

	clrbits_le32(regs_base + OSPI_DCR1, OSPI_DCR1_DLYBYP);

	dcr2 = readl(regs_base + OSPI_DCR2);
	prescaler = (dcr2 & OSPI_DCR2_PRESC_MASK) >> OSPI_DCR2_PRESC_SHIFT;
	bus_freq = DIV_ROUND_UP(plat->clock_rate, prescaler + 1);

	if (prescaler)
		setbits_le32(regs_base + OSPI_TCR, OSPI_TCR_DHQC);

	if (bus_freq <= STM32_DLYB_FREQ_THRESHOLD) {
		bypass_mode = true;
		period_ps = NSEC_PER_SEC / (bus_freq / 1000);
	}

	ret = stm32_ospi_dlyb_configure(bus, bypass_mode, period_ps);
	if (ret)
		return ret;

	if (bypass_mode || prescaler)
		/* Perform only RX TAP selection */
		ret = stm32_ospi_dlyb_find_tap(bus, true, &window_len);
	else
		/* Perform RX/TX TAP selection */
		ret = stm32_ospi_dlyb_find_tap(bus, false, &window_len);

	if (ret) {
		dev_err(bus, "Calibration failed\n");
		if (!bypass_mode)
			/* Stop delay block when configured in lock mode */
			stm32_ospi_dlyb_stop(bus);
	}

	return ret;
}

static int stm32_ospi_exec_op(struct spi_slave *slave,
			      const struct spi_mem_op *op)
{
	struct stm32_ospi_priv *priv = dev_get_priv(slave->dev->parent);
	struct stm32_ospi_plat *plat = dev_get_plat(slave->dev->parent);
	struct stm32_ospi_flash *flash = &priv->flash[priv->cs_used];
	phys_addr_t regs_base = plat->regs_base;
	u32 addr_max;
	u8 mode = OSPI_CCR_IND_WRITE;
	int ret;

	if (op->cmd.dtr && !flash->dtr_calibration_done_once) {
		stm32_ospi_dlyb_stop(slave->dev->parent);
		clrbits_le32(regs_base + OSPI_TCR, OSPI_TCR_SSHIFT);
		flash->octal_dtr = (op->cmd.nbytes == 2);

		ret = stm32_ospi_dtr_calibration(slave->dev->parent);
		if (ret)
			return ret;

		flash->dtr_calibration_done_once = true;
	}

	addr_max = op->addr.val + op->data.nbytes + 1;

	if (op->data.dir == SPI_MEM_DATA_IN && op->data.nbytes) {
		if (addr_max < plat->mm_size && op->addr.buswidth)
			mode = OSPI_CCR_MEM_MAP;
		else
			mode = OSPI_CCR_IND_READ;
	}

	return stm32_ospi_send(slave->dev->parent, op, mode);
}

static int stm32_ospi_probe(struct udevice *bus)
{
	struct stm32_ospi_priv *priv = dev_get_priv(bus);
	struct stm32_ospi_plat *plat = dev_get_plat(bus);
	phys_addr_t regs_base = plat->regs_base;
	ofnode child;
	int ret;

	ret = clk_enable(&plat->clk);
	if (ret) {
		dev_err(bus, "failed to enable clock\n");
		return ret;
	}

	/* Reset OSPI controller */
	reset_assert_bulk(&plat->rst_ctl);
	udelay(2);
	reset_deassert_bulk(&plat->rst_ctl);

	/* Set dcr devsize to max address */
	setbits_le32(regs_base + OSPI_DCR1, OSPI_DCR1_DEVSIZE_MASK);

	priv->cs_used = -1;
	priv->check_transfer = stm32_ospi_readid;

	/* Find memory model on each child node (SPI NOR or SPI NAND) */
	dev_for_each_subnode(child, bus) {
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
	struct stm32_ospi_plat *plat = dev_get_plat(dev->parent);
	struct dm_spi_slave_plat *slave_plat = dev_get_parent_plat(dev);
	struct stm32_ospi_flash *flash;
	phys_addr_t regs_base = plat->regs_base;
	int slave_cs = slave_plat->cs[0];
	int ret;

	if (slave_cs >= OSPI_MAX_CHIP)
		return -ENODEV;

	setbits_le32(regs_base + OSPI_CR, OSPI_CR_EN);

	if (priv->cs_used == slave_cs)
		return 0;

	priv->cs_used = slave_cs;
	flash = &priv->flash[priv->cs_used];

	stm32_ospi_dlyb_stop(dev->parent);

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

			stm32_ospi_dlyb_stop(dev->parent);

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
	struct stm32_ospi_plat *plat = dev_get_plat(dev->parent);
	phys_addr_t regs_base = plat->regs_base;

	clrbits_le32(regs_base + OSPI_CR, OSPI_CR_EN);

	return 0;
}

static int stm32_ospi_set_mode(struct udevice *bus, uint mode)
{
	struct stm32_ospi_plat *ospi_plat = dev_get_plat(bus);
	phys_addr_t regs_base = ospi_plat->regs_base;
	const char *str_rx, *str_tx;
	int ret;

	ret = stm32_ospi_wait_for_not_busy(bus);
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
	.claim_bus = stm32_ospi_claim_bus,
	.release_bus = stm32_ospi_release_bus,
	.set_speed = stm32_ospi_set_speed,
	.set_mode = stm32_ospi_set_mode,
	.mem_ops = &stm32_ospi_mem_ops,
};

static int stm32_ospi_of_to_plat(struct udevice *dev)
{
	struct stm32_ospi_plat *plat = dev_get_plat(dev);
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
		return ret;
	}

	plat->clock_rate = clk_get_rate(&plat->clk);
	if (!plat->clock_rate)
		return -EINVAL;

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

static const struct udevice_id stm32_ospi_ids[] = {
	{ .compatible = "st,stm32mp25-ospi" },
	{ }
};

U_BOOT_DRIVER(stm32_ospi) = {
	.name = "stm32_ospi",
	.id = UCLASS_SPI,
	.of_match = stm32_ospi_ids,
	.of_to_plat = stm32_ospi_of_to_plat,
	.ops = &stm32_ospi_ops,
	.plat_auto = sizeof(struct stm32_ospi_plat),
	.priv_auto = sizeof(struct stm32_ospi_priv),
	.probe = stm32_ospi_probe,
};
