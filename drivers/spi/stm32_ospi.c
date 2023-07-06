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
#define OSPI_CCR_ADMODE_SHIFT	8
#define OSPI_CCR_ADMODE_MASK	GENMASK(10, 8)
#define OSPI_CCR_ADSIZE_SHIFT	12
#define OSPI_CCR_DMODE_SHIFT	24
#define OSPI_CCR_DMODE_MASK	GENMASK(26, 24)
#define OSPI_CCR_IND_WRITE	0
#define OSPI_CCR_IND_READ	1
#define OSPI_CCR_MEM_MAP	3

/* OCTOSPI timing configuration register */
#define OSPI_TCR		0x108
#define OSPI_TCR_DCYC_SHIFT	0x0
#define OSPI_TCR_DCYC_MASK	GENMASK(4, 0)
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

struct stm32_tap_window {
	u8 end;
	u8 length;
};

struct stm32_ospi_priv {
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
	void *buf;

	if (!op->data.nbytes)
		return 0;

	if (mode == OSPI_CCR_MEM_MAP)
		return stm32_ospi_mm(dev, op);

	if (op->data.dir == SPI_MEM_DATA_IN)
		buf = op->data.buf.in;
	else
		buf = (void *)op->data.buf.out;

	return stm32_ospi_tx_poll(dev, buf, op->data.nbytes,
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
	phys_addr_t regs_base = plat->regs_base;
	u32 cr, ccr = 0;
	int timeout, ret;
	int dmode;
	u8 dcyc = 0;

	dev_dbg(dev, "%s: cmd:%#x mode:%d.%d.%d.%d addr:%#llx len:%#x\n",
		__func__, op->cmd.opcode, op->cmd.buswidth, op->addr.buswidth,
		op->dummy.buswidth, op->data.buswidth,
		op->addr.val, op->data.nbytes);

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

	if (ret || timeout)
		dev_err(dev, "%s ret:%d abort timeout:%d\n", __func__,
			ret, timeout);

	return ret;
}

static int stm32_ospi_exec_op(struct spi_slave *slave,
			      const struct spi_mem_op *op)
{
	struct stm32_ospi_plat *plat = dev_get_plat(slave->dev->parent);
	u32 addr_max;
	u8 mode = OSPI_CCR_IND_WRITE;

	addr_max = op->addr.val + op->data.nbytes + 1;

	if (op->data.dir == SPI_MEM_DATA_IN && op->data.nbytes) {
		if (addr_max < plat->mm_size && op->addr.buswidth)
			mode = OSPI_CCR_MEM_MAP;
		else
			mode = OSPI_CCR_IND_READ;
	}

	return stm32_ospi_send(slave->dev->parent, op, mode);
}

static int stm32_ospi_readid(struct udevice *dev)
{
	struct stm32_ospi_priv *priv = dev_get_priv(dev);
	u64 rx_buf;
	struct spi_mem_op readid_op = SPI_MEM_OP(SPI_MEM_OP_CMD(0x9F, 1),
						 SPI_MEM_OP_NO_ADDR,
						 SPI_MEM_OP_NO_DUMMY,
						 SPI_MEM_OP_DATA_IN(8, (u8 *)&rx_buf, 1));
	int ret;

	ret = stm32_ospi_send(dev, &readid_op, OSPI_CCR_IND_READ);
	if (ret)
		return ret;
	dev_dbg(dev, "Flash ID 0x%08llx\n", rx_buf);
	/*
	 * In case of SNAND, the first byte is a dummy byte. Depending of
	 * memory device, its value can be different in function of frequency.
	 * Ignore this byte and force its value to 0.
	 */
	rx_buf &= 0xffffffffffffff00;

	/* On stm32_ospi_readid() first execution, save the golden READID command's answer */
	if (priv->idcode == 0)
		priv->idcode = rx_buf;

	if (rx_buf == priv->idcode)
		return 0;

	return -EIO;
}

static int stm32_ospi_probe(struct udevice *bus)
{
	struct stm32_ospi_priv *priv = dev_get_priv(bus);
	struct stm32_ospi_plat *ospi_plat;
	phys_addr_t regs_base;
	int ret;

	ospi_plat = dev_get_plat(bus);
	regs_base = ospi_plat->regs_base;

	ret = clk_enable(&ospi_plat->clk);
	if (ret) {
		dev_err(bus, "failed to enable clock\n");
		return ret;
	}

	/* Reset OSPI controller */
	reset_assert_bulk(&ospi_plat->rst_ctl);
	udelay(2);
	reset_deassert_bulk(&ospi_plat->rst_ctl);

	priv->cs_used = -1;
	memset(priv->prescaler, -1, OSPI_MAX_CHIP * sizeof(u32));

	setbits_le32(regs_base + OSPI_TCR, OSPI_TCR_SSHIFT);

	clrsetbits_le32(regs_base + OSPI_CR, OSPI_CR_FTHRES_MASK,
			3 << OSPI_CR_FTHRES_SHIFT);

	/* Set dcr devsize to max address */
	setbits_le32(regs_base + OSPI_DCR1, OSPI_DCR1_DEVSIZE_MASK);

	priv->check_transfer = stm32_ospi_readid;

	return 0;
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

	bus_freq = ospi_clk / (prescaler + 1);

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

int stm32_ospi_dlyb_find_tap(struct udevice *dev, bool rx_only)
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

static void stm32_ospi_dlyb_stop(struct udevice *dev)
{
	struct stm32_ospi_plat *plat = dev_get_plat(dev);

	/* disable delay block */
	regmap_write(plat->regmap,
		     plat->dlyb_base + SYSCFG_DLYBOS_CR,
		     0x0);
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

static int stm32_ospi_calibration(struct udevice *bus, uint freq)
{
	struct stm32_ospi_priv *priv = dev_get_priv(bus);
	int ret;

	/*
	 * Set memory device at low frequency (50MHz) and sent
	 * READID (0x9F) command, save the answer as golden answer
	 */
	ret = stm32_ospi_set_speed(bus, STM32_DLYB_FREQ_THRESHOLD);
	if (ret)
		return ret;

	priv->idcode = 0;
	ret = stm32_ospi_readid(bus);
	if (ret)
		return ret;

	/* set frequency at requested value and perform calibration */
	ret = stm32_ospi_set_speed(bus, freq);
	if (ret)
		return ret;

	ret = stm32_ospi_dlyb_configure(bus, false, 0);
	if (ret)
		return ret;

	/* perform only RX TAP selection */
	ret = stm32_ospi_dlyb_find_tap(bus, true);
	if (ret)
		dev_info(bus, "Calibration phase failed\n");

	return ret;
}

static int stm32_ospi_claim_bus(struct udevice *dev)
{
	struct stm32_ospi_priv *priv = dev_get_priv(dev->parent);
	struct stm32_ospi_plat *plat = dev_get_plat(dev->parent);
	struct dm_spi_slave_plat *slave_plat = dev_get_parent_plat(dev);
	phys_addr_t regs_base = plat->regs_base;
	u32 ospi_clk = plat->clock_rate;
	u32 dcr2, prescaler;
	unsigned int slave_cs = slave_plat->cs[0];
	uint bus_freq;
	int ret = 0;

	if (slave_cs >= OSPI_MAX_CHIP)
		return -ENODEV;

	setbits_le32(regs_base + OSPI_CR, OSPI_CR_EN);

	if (priv->cs_used == slave_cs)
		return 0;

	priv->cs_used = slave_cs;

	/* Set chip select */
	clrsetbits_le32(regs_base + OSPI_CR, OSPI_CR_CSSEL,
			priv->cs_used ? OSPI_CR_CSSEL : 0);

	dcr2 = readl(regs_base + OSPI_DCR2);
	prescaler = (dcr2 & OSPI_DCR2_PRESC_MASK) >> OSPI_DCR2_PRESC_SHIFT;

	if (prescaler == priv->prescaler[priv->cs_used])
		return 0;

	 stm32_ospi_dlyb_stop(dev->parent);

	priv->prescaler[priv->cs_used] = prescaler;
	bus_freq = ospi_clk / (prescaler + 1);

	/* calibration needed above 50MHz */
	if (bus_freq > STM32_DLYB_FREQ_THRESHOLD) {
		ret = stm32_ospi_calibration(dev->parent, bus_freq);
		if (ret) {
			dev_info(dev->parent, "Set flash frequency to a safe value (%d Hz)\n",
				 STM32_DLYB_FREQ_THRESHOLD);

			stm32_ospi_dlyb_stop(dev->parent);

			ret = stm32_ospi_set_speed(dev->parent, STM32_DLYB_FREQ_THRESHOLD);
		}
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

static const struct spi_controller_mem_ops stm32_ospi_mem_ops = {
	.exec_op = stm32_ospi_exec_op,
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
