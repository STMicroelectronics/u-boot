/* SPDX-License-Identifier: GPL-2.0-or-later OR BSD-3-Clause */
/*
 * Copyright (C) 2021, STMicroelectronics - All Rights Reserved
 */

#include <clk.h>
#include <reset.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>

/*
 * OCTOSPI control register
 */
#define OSPI_CR			0x00
#define OSPI_CR_EN		BIT(0)
#define OSPI_CR_ABORT		BIT(1)
#define OSPI_CR_TCEN		BIT(3)
#define OSPI_CR_FSEL		BIT(7)
#define OSPI_CR_FTHRES_MASK	GENMASK(12,8)
#define OSPI_CR_CSSEL		BIT(24)
#define OSPI_CR_FMODE_SHIFT	28
#define OSPI_CR_FMODE_MASK	GENMASK(29, 28)
#define OSPI_CR_FMODE_IND_WRITE	0
#define OSPI_CR_FMODE_IND_READ	1
#define OSPI_CR_FMODE_MMAP	3
/*
 * OCTOSPI device configuration register
 */
#define OSPI_DCR1		0x08
#define OSPI_DCR1_CKMODE	BIT(0)
#define OSPI_DCR1_DLYBYP	BIT(3)
#define OSPI_DCR1_CSHT_SHIFT	8
#define OSPI_DCR1_CSHT_MASK	GENMASK(13, 8)
#define OSPI_DCR1_DEVSIZE_MASK	GENMASK(20,16)
#define OSPI_DCR1_MTYP_MASK	GENMASK(26, 24)
#define OSPI_DCR1_MTYP_SHIFT	24
#define OSPI_DCR1_MTYP_MX_MODE	1
#define OSPI_DCR1_MTYP_HP_MEMMODE	4
/*
 * OCTOSPI device configuration register 2
 */
#define OSPI_DCR2		0x0c
#define OSPI_DCR2_PRESC_SHIFT	0
#define OSPI_DCR2_PRESC_MASK	GENMASK(7, 0)
/*
 * OCTOSPI status register
 */
#define OSPI_SR			0x20
#define OSPI_SR_TEF		BIT(0)
#define OSPI_SR_TCF		BIT(1)
#define OSPI_SR_FTF		BIT(2)
#define OSPI_SR_BUSY		BIT(5)
/*
 * OCTOSPI flag clear register
 */
#define OSPI_FCR		0x24
#define OSPI_FCR_CTEF		BIT(0)
#define OSPI_FCR_CTCF		BIT(1)
/*
 * OCTOSPI data length register
 */
#define OSPI_DLR		0x40
/*
 * OCTOSPI address register
 */
#define OSPI_AR			0x48
/*
 * OCTOSPI data configuration register
 */
#define OSPI_DR			0x50
/*
 * OCTOSPI communication configuration register
 */
#define OSPI_CCR		0x100
#define OSPI_CCR_IMODE_SHIFT	0
#define OSPI_CCR_IMODE_MASK	GENMASK(2, 0)
#define OSPI_CCR_IDTR		BIT(3)
#define OSPI_CCR_ISIZE_SHIFT	4
#define OSPI_CCR_ISIZE_MASK	GENMASK(5, 4)
#define OSPI_CCR_ADMODE_SHIFT	8
#define OSPI_CCR_ADMODE_MASK	GENMASK(10, 8)
#define OSPI_CCR_ADMODE_8LINES	4
#define OSPI_CCR_ADDTR		BIT(11)
#define OSPI_CCR_ADSIZE_SHIFT	12
#define OSPI_CCR_ADSIZE_MASK	GENMASK(13,12)
#define OSPI_CCR_ADSIZE_32BITS	3
#define OSPI_CCR_DMODE_SHIFT	24
#define OSPI_CCR_DMODE_MASK	GENMASK(26, 24)
#define OSPI_CCR_DMODE_8LINES	4
#define OSPI_CCR_IND_WRITE	0
#define OSPI_CCR_IND_READ	1
#define OSPI_CCR_MEM_MAP	3
#define OSPI_CCR_DDTR		BIT(27)
#define OSPI_CCR_DQSE		BIT(29)
/*
 * OCTOSPI timing configuration register
 */
#define OSPI_TCR		0x108
#define OSPI_TCR_DCYC_SHIFT	0x0
#define OSPI_TCR_DCYC_MASK	GENMASK(4, 0)
#define OSPI_TCR_DHQC		BIT(28)
#define OSPI_TCR_SSHIFT		BIT(30)
/*
 * OCTOSPI instruction register
 */
#define OSPI_IR			0x110
/*
 * OCTOSPI low power timeout register
 */
#define OSPI_LPTR		0x130
#define OSPI_LPTR_TIMEOUT_MASK	GENMASK(15, 0)

/*
 * OCTOSPI write communication configuration register
 */
#define OSPI_WCCR		0x180
/*
 * HyperBus latency configuration register
 */
#define OSPI_HLCR		0x200
#define OSPI_HLCR_WZL		BIT(1)
#define OSPI_HLCR_TACC_MASK	GENMASK(15,8)
#define OSPI_HLCR_TRWR_MASK	GENMASK(23,16)

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

#define OSPI_ABT_TIMEOUT_US		100000
#define OSPI_BUSY_TIMEOUT_US		100000
#define OSPI_CMD_TIMEOUT_US		1000000
#define OSPI_FIFO_TIMEOUT_US		30000
#define STM32_DLYB_FREQ_THRESHOLD	50000000
#define STM32_DLYBOS_TIMEOUT_MS		1000
#define STM32_DLYBOS_DELAY_NB		24

struct stm32_omi_plat {
	struct regmap *regmap;
	phys_addr_t regs_base;		/* register base address */
	phys_addr_t mm_base;		/* memory map base address */
	resource_size_t mm_size;
	struct clk clk;
	struct reset_ctl_bulk rst_ctl;
	ulong clock_rate;
	u32 dlyb_base;
	bool jedec_flash;
};

struct stm32_omi_priv {
	int (*check_transfer)(struct udevice * omi_dev);
	struct udevice *dev;
	bool is_calibrating;
};

struct stm32_tap_window {
	u8 end;
	u8 length;
};

int stm32_omi_dlyb_configure(struct udevice *dev,
			     bool bypass_mode, u16 period_ps);
int stm32_omi_dlyb_find_tap(struct udevice *dev, bool rx_only, u8 *window_len);
int stm32_omi_dlyb_set_cr(struct udevice *dev, u32 dlyb_cr);
void stm32_omi_dlyb_get_cr(struct udevice *dev, u32 *dlyb_cr);
void stm32_omi_dlyb_stop(struct udevice *dev);
int stm32_omi_tx_poll(struct udevice *dev, u8 *buf, u32 len, bool read);
int stm32_omi_wait_cmd(struct udevice *dev);
int stm32_omi_wait_for_not_busy(struct udevice *dev);
