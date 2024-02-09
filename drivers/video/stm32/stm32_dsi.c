// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2019 STMicroelectronics - All Rights Reserved
 * Author(s): Philippe Cornu <philippe.cornu@st.com> for STMicroelectronics.
 *	      Yannick Fertre <yannick.fertre@st.com> for STMicroelectronics.
 *
 * This MIPI DSI controller driver is based on the Linux Kernel driver from
 * drivers/gpu/drm/stm/dw_mipi_dsi-stm.c.
 */

#define LOG_CATEGORY UCLASS_VIDEO_BRIDGE

#include <common.h>
#include <clk.h>
#include <dm.h>
#include <dsi_host.h>
#include <log.h>
#include <mipi_dsi.h>
#include <panel.h>
#include <reset.h>
#include <video.h>
#include <video_bridge.h>
#include <asm/io.h>
#include <dm/device-internal.h>
#include <dm/device_compat.h>
#include <dm/lists.h>
#include <linux/bitops.h>
#include <linux/iopoll.h>
#include <power/regulator.h>

#define HWVER_130			0x31333000	/* IP version 1.30 */
#define HWVER_131			0x31333100	/* IP version 1.31 */
#define HWVER_141			0x31343100	/* IP version 1.41 */

/* DSI digital registers & bit definitions */
#define DSI_VERSION			0x00
#define VERSION				GENMASK(31, 8)

/*
 * DSI wrapper registers & bit definitions
 * Note: registers are named as in the Reference Manual
 */
#define DSI_WCFGR	0x0400		/* Wrapper ConFiGuration Reg */
#define WCFGR_DSIM	BIT(0)		/* DSI Mode */
#define WCFGR_COLMUX	GENMASK(3, 1)	/* COLor MUltipleXing */

#define DSI_WCR		0x0404		/* Wrapper Control Reg */
#define WCR_DSIEN	BIT(3)		/* DSI ENable */

#define DSI_WISR	0x040C		/* Wrapper Interrupt and Status Reg */
#define WISR_PLLLS	BIT(8)		/* PLL Lock Status */
#define WISR_RRS	BIT(12)		/* Regulator Ready Status */

#define DSI_WPCR0	0x0418		/* Wrapper Phy Conf Reg 0 */
#define WPCR0_UIX4	GENMASK(5, 0)	/* Unit Interval X 4 */
#define WPCR0_TDDL	BIT(16)		/* Turn Disable Data Lanes */

#define DSI_WRPCR	0x0430		/* Wrapper Regulator & Pll Ctrl Reg */
#define WRPCR_PLLEN	BIT(0)		/* PLL ENable */
#define WRPCR_NDIV	GENMASK(8, 2)	/* pll loop DIVision Factor */
#define WRPCR_IDF	GENMASK(14, 11)	/* pll Input Division Factor */
#define WRPCR_ODF	GENMASK(17, 16)	/* pll Output Division Factor */
#define WRPCR_REGEN	BIT(24)		/* REGulator ENable */
#define WRPCR_BGREN	BIT(28)		/* BandGap Reference ENable */
#define IDF_MIN		1
#define IDF_MAX		7
#define NDIV_MIN	10
#define NDIV_MAX	125
#define ODF_MIN		1
#define ODF_MAX		8

#define IDF_PHY_141_MIN		1
#define IDF_PHY_141_MAX		16
#define NDIV_PHY_141_MIN	64
#define NDIV_PHY_141_MAX	625

/* dsi color format coding according to the datasheet */
enum dsi_color {
	DSI_RGB565_CONF1,
	DSI_RGB565_CONF2,
	DSI_RGB565_CONF3,
	DSI_RGB666_CONF1,
	DSI_RGB666_CONF2,
	DSI_RGB888,
};

#define LANE_MIN_KBPS	31250
#define LANE_MAX_KBPS	500000

/* specific registers for hardware version 1.41 */
#define DSI_WPCR1	0x0430		/* Wrapper Phy Conf Reg 1 */
#define WPCR1_CCF	GENMASK(5, 0)	/* Configuration clock frequency */
#define WPCR1_HSFR	GENMASK(14, 8)	/* High speed frequency range in Tx mode */
#define WPCR1_DLD	BIT(16)		/* Data lanes 0 direction */

#define DSI_WRPCR0	0x0434		/* Wrapper regulator and PLL configuration register 0 */
#define WRPCR0_IDF	GENMASK(3, 0)	/* PLL input division ratio */
#define WRPCR0_NDIV	GENMASK(13, 4)	/* PLL InLoop division ratio */
#define WRPCR0_PSC	BIT(16)		/* PLL shadow control */

#define DSI_WRPCR1	0x0438		/* Wrapper regulator and PLL configuration register 1 */
#define WRPCR1_PROP	GENMASK(5, 0)	/* Proportional charge pump */
#define WRPCR1_GMP	GENMASK(7, 6)	/* Loop filter resistance */
#define WRPCR1_INT	GENMASK(13, 8)	/* Integral of charge pump */
#define WRPCR1_BIAS	GENMASK(22, 16)	/* Charge pump bias */
#define WRPCR1_VCO	GENMASK(27, 24)	/* VCO operating range */
#define WRPCR1_ODF	GENMASK(29, 28)	/* Output division factor */

#define DSI_WRPCR2	0x043C		/* Wrapper regulator and PLL configuration register 2 */
#define WRPCR2_SEL	GENMASK(1, 0)	/* Output selection for PLL Clock */
#define WRPCR2_PLLEN	BIT(8)		/* PLL ENable */
#define WRPCR2_UPD	BIT(16)		/* Update (copies) the PLL shadow registers */
#define WRPCR2_CLR	BIT(24)		/* Clears the PLL shadow registers to their reset values */
#define WRPCR2_FPLL	BIT(28)		/* Force PLL lock signal */

#define DSI_PTCR0	0x00B4		/* Host PHY test control register 0 */
#define PTCR0_TRSEN	BIT(0)		/* Test-interface reset enable for the TDI bus */
#define PTCR0_TCKEN	BIT(1)		/* Test-interface clock enable for the TDI bus */

#define DSI_PCTLR	0x00A0		/* Host PHY control register */
#define PCTLR_PWEN	BIT(0)		/* Power enable */
#define PCTLR_DEN	BIT(1)		/* Digital enable */
#define PCTLR_CKEN	BIT(2)		/* Clock enable */
#define PCTLR_UCKEN	BIT(3)		/* ULPS clock enable */

#define DSI_PSR		0x00B0		/* Host PHY status register */
#define PSR_PSSC	BIT(2)		/* PHY stop state clock lane */

#define LANE_MIN_PHY_141_KBPS	80000
#define LANE_MAX_PHY_141_KBPS	2500000
#define FVCO_MIN_PHY_141_KBPS	320000
#define FVCO_MAX_PHY_141_KBPS	1250000

/* Timeout for regulator on/off, pll lock/unlock & fifo empty */
#define TIMEOUT_US	200000

struct stm32_dsi_priv {
	struct mipi_dsi_device device;
	void __iomem *base;
	struct udevice *panel;
	u32 pllref_clk;
	u32 hw_version;
	int lane_min_kbps;
	int lane_max_kbps;
	struct udevice *vdd_reg;
	struct udevice *vdda18_reg;
	struct udevice *dsi_host;
	unsigned int lane_mbps;
	u32 format;
};

static inline void dsi_write(struct stm32_dsi_priv *dsi, u32 reg, u32 val)
{
	writel(val, dsi->base + reg);
}

static inline u32 dsi_read(struct stm32_dsi_priv *dsi, u32 reg)
{
	return readl(dsi->base + reg);
}

static inline void dsi_set(struct stm32_dsi_priv *dsi, u32 reg, u32 mask)
{
	dsi_write(dsi, reg, dsi_read(dsi, reg) | mask);
}

static inline void dsi_clear(struct stm32_dsi_priv *dsi, u32 reg, u32 mask)
{
	dsi_write(dsi, reg, dsi_read(dsi, reg) & ~mask);
}

static inline void dsi_update_bits(struct stm32_dsi_priv *dsi, u32 reg,
				   u32 mask, u32 val)
{
	dsi_write(dsi, reg, (dsi_read(dsi, reg) & ~mask) | val);
}

static enum dsi_color dsi_color_from_mipi(u32 fmt)
{
	switch (fmt) {
	case MIPI_DSI_FMT_RGB888:
		return DSI_RGB888;
	case MIPI_DSI_FMT_RGB666:
		return DSI_RGB666_CONF2;
	case MIPI_DSI_FMT_RGB666_PACKED:
		return DSI_RGB666_CONF1;
	case MIPI_DSI_FMT_RGB565:
		return DSI_RGB565_CONF1;
	default:
		log_err("MIPI color invalid, so we use rgb888\n");
	}
	return DSI_RGB888;
}

static int dsi_pll_get_clkout_khz(int clkin_khz, int idf, int ndiv, int odf)
{
	int divisor = idf * odf;

	/* prevent from division by 0 */
	if (!divisor)
		return 0;

	return DIV_ROUND_CLOSEST(clkin_khz * ndiv, divisor);
}

static int dsi_pll_get_params(struct stm32_dsi_priv *dsi,
			      int clkin_khz, int clkout_khz,
			      int *idf, int *ndiv, int *odf)
{
	int i, o, n, n_min, n_max;
	int fvco_min, fvco_max, delta, best_delta; /* all in khz */

	/* Early checks preventing division by 0 & odd results */
	if (clkin_khz <= 0 || clkout_khz <= 0)
		return -EINVAL;

	fvco_min = dsi->lane_min_kbps * 2 * ODF_MAX;
	fvco_max = dsi->lane_max_kbps * 2 * ODF_MIN;

	best_delta = 1000000; /* big started value (1000000khz) */

	for (i = IDF_MIN; i <= IDF_MAX; i++) {
		/* Compute ndiv range according to Fvco */
		n_min = ((fvco_min * i) / (2 * clkin_khz)) + 1;
		n_max = (fvco_max * i) / (2 * clkin_khz);

		/* No need to continue idf loop if we reach ndiv max */
		if (n_min >= NDIV_MAX)
			break;

		/* Clamp ndiv to valid values */
		if (n_min < NDIV_MIN)
			n_min = NDIV_MIN;
		if (n_max > NDIV_MAX)
			n_max = NDIV_MAX;

		for (o = ODF_MIN; o <= ODF_MAX; o *= 2) {
			n = DIV_ROUND_CLOSEST(i * o * clkout_khz, clkin_khz);
			/* Check ndiv according to vco range */
			if (n < n_min || n > n_max)
				continue;
			/* Check if new delta is better & saves parameters */
			delta = dsi_pll_get_clkout_khz(clkin_khz, i, n, o) -
				clkout_khz;
			if (delta < 0)
				delta = -delta;
			if (delta < best_delta) {
				*idf = i;
				*ndiv = n;
				*odf = o;
				best_delta = delta;
			}
			/* fast return in case of "perfect result" */
			if (!delta)
				return 0;
		}
	}

	return 0;
}

static int dsi_phy_init(void *priv_data)
{
	struct mipi_dsi_device *device = priv_data;
	struct udevice *dev = device->dev;
	struct stm32_dsi_priv *dsi = dev_get_priv(dev);
	u32 val;
	int ret;

	dev_dbg(dev, "Initialize DSI physical layer\n");

	/* Enable the regulator */
	dsi_set(dsi, DSI_WRPCR, WRPCR_REGEN | WRPCR_BGREN);
	ret = readl_poll_timeout(dsi->base + DSI_WISR, val, val & WISR_RRS,
				 TIMEOUT_US);
	if (ret) {
		dev_dbg(dev, "!TIMEOUT! waiting REGU\n");
		return ret;
	}

	/* Enable the DSI PLL & wait for its lock */
	dsi_set(dsi, DSI_WRPCR, WRPCR_PLLEN);
	ret = readl_poll_timeout(dsi->base + DSI_WISR, val, val & WISR_PLLLS,
				 TIMEOUT_US);
	if (ret) {
		dev_dbg(dev, "!TIMEOUT! waiting PLL\n");
		return ret;
	}

	return 0;
}

static void dsi_phy_post_set_mode(void *priv_data, unsigned long mode_flags)
{
	struct mipi_dsi_device *device = priv_data;
	struct udevice *dev = device->dev;
	struct stm32_dsi_priv *dsi = dev_get_priv(dev);

	dev_dbg(dev, "Set mode %p enable %ld\n", dsi,
		mode_flags & MIPI_DSI_MODE_VIDEO);

	if (!dsi)
		return;

	/*
	 * DSI wrapper must be enabled in video mode & disabled in command mode.
	 * If wrapper is enabled in command mode, the display controller
	 * register access will hang.
	 */

	if (mode_flags & MIPI_DSI_MODE_VIDEO)
		dsi_set(dsi, DSI_WCR, WCR_DSIEN);
	else
		dsi_clear(dsi, DSI_WCR, WCR_DSIEN);
}

static int dsi_get_lane_mbps(void *priv_data, struct display_timing *timings,
			     u32 lanes, u32 format, unsigned int *lane_mbps)
{
	struct mipi_dsi_device *device = priv_data;
	struct udevice *dev = device->dev;
	struct stm32_dsi_priv *dsi = dev_get_priv(dev);
	int idf, ndiv, odf, pll_in_khz, pll_out_khz;
	int ret, bpp;
	u32 val;

	/* Update lane capabilities according to hw version */
	dsi->lane_min_kbps = LANE_MIN_KBPS;
	dsi->lane_max_kbps = LANE_MAX_KBPS;
	if (dsi->hw_version == HWVER_131) {
		dsi->lane_min_kbps *= 2;
		dsi->lane_max_kbps *= 2;
	}

	pll_in_khz = dsi->pllref_clk / 1000;

	/* Compute requested pll out */
	bpp = mipi_dsi_pixel_format_to_bpp(format);
	pll_out_khz = (timings->pixelclock.typ / 1000) * bpp / lanes;
	/* Add 20% to pll out to be higher than pixel bw (burst mode only) */
	pll_out_khz = (pll_out_khz * 12) / 10;
	if (pll_out_khz > dsi->lane_max_kbps) {
		pll_out_khz = dsi->lane_max_kbps;
		dev_warn(dev, "Warning max phy mbps is used\n");
	}
	if (pll_out_khz < dsi->lane_min_kbps) {
		pll_out_khz = dsi->lane_min_kbps;
		dev_warn(dev, "Warning min phy mbps is used\n");
	}

	/* Compute best pll parameters */
	idf = 0;
	ndiv = 0;
	odf = 0;
	ret = dsi_pll_get_params(dsi, pll_in_khz, pll_out_khz,
				 &idf, &ndiv, &odf);
	if (ret) {
		dev_err(dev, "Warning dsi_pll_get_params(): bad params\n");
		return ret;
	}

	/* Get the adjusted pll out value */
	pll_out_khz = dsi_pll_get_clkout_khz(pll_in_khz, idf, ndiv, odf);

	/* Set the PLL division factors */
	dsi_update_bits(dsi, DSI_WRPCR,	WRPCR_NDIV | WRPCR_IDF | WRPCR_ODF,
			(ndiv << 2) | (idf << 11) | ((ffs(odf) - 1) << 16));

	/* Compute uix4 & set the bit period in high-speed mode */
	val = 4000000 / pll_out_khz;
	dsi_update_bits(dsi, DSI_WPCR0, WPCR0_UIX4, val);

	/* Select video mode by resetting DSIM bit */
	dsi_clear(dsi, DSI_WCFGR, WCFGR_DSIM);

	/* Select the color coding */
	dsi_update_bits(dsi, DSI_WCFGR, WCFGR_COLMUX,
			dsi_color_from_mipi(format) << 1);

	*lane_mbps = pll_out_khz / 1000;

	dev_dbg(dev, "pll_in %ukHz pll_out %ukHz lane_mbps %uMHz\n",
		pll_in_khz, pll_out_khz, *lane_mbps);

	return 0;
}

static const struct mipi_dsi_phy_ops dsi_stm_phy_ops = {
	.init = dsi_phy_init,
	.get_lane_mbps = dsi_get_lane_mbps,
	.post_set_mode = dsi_phy_post_set_mode,
};

struct hstt {
	unsigned int maxfreq;
	struct mipi_dsi_phy_timing timing;
};

#define HSTT(_maxfreq, _c_lp2hs, _c_hs2lp, _d_lp2hs, _d_hs2lp)	\
{					\
	.maxfreq = _maxfreq,		\
	.timing = {			\
		.clk_lp2hs = _c_lp2hs,	\
		.clk_hs2lp = _c_hs2lp,	\
		.data_lp2hs = _d_lp2hs,	\
		.data_hs2lp = _d_hs2lp,	\
	}				\
}

/* Table High-Speed Transition Times */
static const struct hstt hstt_phy_141_table[] = {
	HSTT(80, 21, 17, 15, 10),
	HSTT(90, 23, 17, 16, 10),
	HSTT(100, 22, 17, 16, 10),
	HSTT(110, 25, 18, 17, 11),
	HSTT(120, 26, 20, 18, 11),
	HSTT(130, 27, 19, 19, 11),
	HSTT(140, 27, 19, 19, 11),
	HSTT(150, 28, 20, 20, 12),
	HSTT(160, 30, 21, 22, 13),
	HSTT(170, 30, 21, 23, 13),
	HSTT(180, 31, 21, 23, 13),
	HSTT(190, 32, 22, 24, 13),
	HSTT(205, 35, 22, 25, 13),
	HSTT(220, 37, 26, 27, 15),
	HSTT(235, 38, 28, 27, 16),
	HSTT(250, 41, 29, 30, 17),
	HSTT(275, 43, 29, 32, 18),
	HSTT(300, 45, 32, 35, 19),
	HSTT(325, 48, 33, 36, 18),
	HSTT(350, 51, 35, 40, 20),
	HSTT(400, 59, 37, 44, 21),
	HSTT(450, 65, 40, 49, 23),
	HSTT(500, 71, 41, 54, 24),
	HSTT(550, 77, 44, 57, 26),
	HSTT(600, 82, 46, 64, 27),
	HSTT(650, 87, 48, 67, 28),
	HSTT(700, 94, 52, 71, 29),
	HSTT(750, 99, 52, 75, 31),
	HSTT(800, 105, 55, 82, 32),
	HSTT(850, 110, 58, 85, 32),
	HSTT(900, 115, 58, 88, 35),
	HSTT(950, 120, 62, 93, 36),
	HSTT(1000, 128, 63, 99, 38),
	HSTT(1050, 132, 65, 102, 38),
	HSTT(1100, 138, 67, 106, 39),
	HSTT(1150, 146, 69, 112, 42),
	HSTT(1200, 151, 71, 117, 43),
	HSTT(1250, 153, 74, 120, 45),
	HSTT(1300, 160, 73, 124, 46),
	HSTT(1350, 165, 76, 130, 47),
	HSTT(1400, 172, 78, 134, 49),
	HSTT(1450, 177, 80, 138, 49),
	HSTT(1500, 183, 81, 143, 52),
	HSTT(1550, 191, 84, 147, 52),
	HSTT(1600, 194, 85, 152, 52),
	HSTT(1650, 201, 86, 155, 53),
	HSTT(1700, 208, 88, 161, 53),
	HSTT(1750, 212, 89, 165, 53),
	HSTT(1800, 220, 90, 171, 54),
	HSTT(1850, 223, 92, 175, 55),
	HSTT(1900, 231, 91, 180, 56),
	HSTT(1950, 236, 95, 185, 56),
	HSTT(2000, 243, 97, 190, 58),
	HSTT(2050, 248, 99, 194, 59),
	HSTT(2100, 252, 100, 199, 61),
	HSTT(2150, 259, 102, 204, 62),
	HSTT(2200, 266, 105, 210, 63),
	HSTT(2250, 269, 109, 213, 65),
	HSTT(2300, 272, 109, 217, 66),
	HSTT(2350, 281, 112, 225, 66),
	HSTT(2400, 283, 115, 226, 67),
	HSTT(2450, 282, 115, 226, 67),
	HSTT(2500, 281, 118, 227, 68)
};

struct dphy_pll_parameter_map {
	u32 data_rate;	/* upper margin of frequency range */
	u8 hs_freq;	/* hsfreqrange */
	u8 odf;
	u8 vco;
	u8 prop;
};

static const struct dphy_pll_parameter_map dppa_map_phy_141[] = {
	{80, 0x00, 0x03, 0x0F, 0x0B},
	{90, 0x10, 0x03, 0x0F, 0x0B},
	{100, 0x20, 0x03, 0x0F, 0x0B},
	{110, 0x30, 0x03, 0x09, 0x0B},
	{120, 0x01, 0x03, 0x09, 0x0B},
	{130, 0x11, 0x03, 0x09, 0x0B},
	{140, 0x21, 0x03, 0x09, 0x0B},
	{150, 0x31, 0x03, 0x09, 0x0B},
	{160, 0x02, 0x02, 0x0F, 0x0B},
	{170, 0x12, 0x02, 0x0F, 0x0B},
	{180, 0x22, 0x02, 0x0F, 0x0B},
	{190, 0x32, 0x02, 0x0F, 0x0B},
	{205, 0x03, 0x02, 0x0F, 0x0B},
	{220, 0x13, 0x02, 0x09, 0x0B},
	{235, 0x23, 0x02, 0x09, 0x0B},
	{250, 0x33, 0x02, 0x09, 0x0B},
	{275, 0x04, 0x02, 0x09, 0x0B},
	{300, 0x14, 0x02, 0x09, 0x0B},
	{325, 0x25, 0x01, 0x0F, 0x0B},
	{350, 0x35, 0x01, 0x0F, 0x0B},
	{400, 0x05, 0x01, 0x0F, 0x0B},
	{450, 0x16, 0x01, 0x09, 0x0B},
	{500, 0x26, 0x01, 0x09, 0x0B},
	{550, 0x37, 0x01, 0x09, 0x0B},
	{600, 0x07, 0x01, 0x09, 0x0B},
	{650, 0x18, 0x00, 0x0F, 0x0B},
	{700, 0x28, 0x00, 0x0F, 0x0B},
	{750, 0x39, 0x00, 0x0F, 0x0B},
	{800, 0x09, 0x00, 0x0F, 0x0B},
	{850, 0x19, 0x00, 0x09, 0x0B},
	{900, 0x29, 0x00, 0x09, 0x0B},
	{950, 0x3A, 0x00, 0x09, 0x0B},
	{1000, 0x0A, 0x00, 0x09, 0x0B},
	{1050, 0x1A, 0x00, 0x09, 0x0B},
	{1100, 0x2A, 0x00, 0x09, 0x0B},
	{1150, 0x3B, 0x00, 0x09, 0x0B},
	{1200, 0x0B, 0x00, 0x09, 0x0B},
	{1250, 0x1B, 0x00, 0x09, 0x0B},
	{1300, 0x2B, 0x00, 0x03, 0x0B},
	{1350, 0x3C, 0x00, 0x03, 0x0B},
	{1400, 0x0C, 0x00, 0x03, 0x0B},
	{1450, 0x1C, 0x00, 0x03, 0x0B},
	{1500, 0x2C, 0x00, 0x03, 0x0B},
	{1550, 0x3D, 0x00, 0x03, 0x0B},
	{1600, 0x0D, 0x00, 0x03, 0x0B},
	{1650, 0x1D, 0x00, 0x03, 0x0B},
	{1700, 0x2E, 0x00, 0x03, 0x0B},
	{1750, 0x3E, 0x00, 0x03, 0x0B},
	{1800, 0x0E, 0x00, 0x03, 0x0B},
	{1850, 0x1E, 0x00, 0x03, 0x0B},
	{1900, 0x2F, 0x00, 0x03, 0x0B},
	{1950, 0x3F, 0x00, 0x03, 0x0B},
	{2000, 0x0F, 0x00, 0x03, 0x0B},
	{2050, 0x40, 0x00, 0x03, 0x0B},
	{2100, 0x41, 0x00, 0x03, 0x0B},
	{2150, 0x42, 0x00, 0x03, 0x0B},
	{2200, 0x43, 0x00, 0x01, 0x0B},
	{2250, 0x44, 0x00, 0x01, 0x0B},
	{2300, 0x45, 0x00, 0x01, 0x0C},
	{2350, 0x46, 0x00, 0x01, 0x0C},
	{2400, 0x47, 0x00, 0x01, 0x0C},
	{2450, 0x48, 0x00, 0x01, 0x0C},
	{2500, 0x49, 0x00, 0x01, 0x0C}
};

static int dsi_phy_141_pll_get_params(struct stm32_dsi_priv *dsi,
				      int clkin_khz, int clkout_khz,
				      int *idf, int *ndiv, int *odf)
{
	int i, n;
	int delta, best_delta; /* all in khz */

	/* Early checks preventing division by 0 & odd results */
	if (clkin_khz <= 0 || clkout_khz <= 0)
		return -EINVAL;

	best_delta = 1000000; /* big started value (1000000khz) */

	for (i = IDF_PHY_141_MIN; i <= IDF_PHY_141_MAX; i++) {
		for (n = NDIV_PHY_141_MIN; n <= NDIV_PHY_141_MAX; n++) {
			/* Check if new delta is better & saves parameters */
			delta = dsi_pll_get_clkout_khz(clkin_khz, i, n, *odf) - clkout_khz;

			if (delta < 0)
				delta = -delta;
			if (delta < best_delta) {
				*idf = i;
				*ndiv = n;
				best_delta = delta;
			}
			/* fast return in case of "perfect result" */
			if (!delta)
				return 0;
		}
	}

	return 0;
}

static int dsi_phy_141_init(void *priv_data)
{
	struct mipi_dsi_device *device = priv_data;
	struct udevice *dev = device->dev;
	struct stm32_dsi_priv *dsi = dev_get_priv(dev);
	u32 val, ccf, prop, gmp, int1, bias, vco, ndiv, odf, idf;
	unsigned int pll_in_khz, pll_out_khz, hsfreq;
	int ret, i;

	dev_dbg(dev, "Initialize DSI physical layer\n");

	/* Select video mode by resetting DSIM bit */
	dsi_clear(dsi, DSI_WCFGR, WCFGR_DSIM);

	/* Select the color coding */
	dsi_update_bits(dsi, DSI_WCFGR, WCFGR_COLMUX,
			dsi_color_from_mipi(dsi->format) << 1);

	dsi_write(dsi, DSI_PCTLR, 0x00);

	/* clear the pll shadow regs */
	dsi_set(dsi, DSI_WRPCR2, WRPCR2_CLR);
	mdelay(1);

	dsi_clear(dsi, DSI_WRPCR2, WRPCR2_CLR);
	mdelay(1);

	/* set testclr = 1 */
	dsi_set(dsi, DSI_PTCR0, PTCR0_TRSEN);
	mdelay(1);

	dsi_clear(dsi, DSI_PTCR0, PTCR0_TRSEN);
	mdelay(1);

	/* Compute requested pll out, pll out is the half of the lane data rate */
	pll_out_khz = dsi->lane_mbps * 1000 / 2;
	pll_in_khz = dsi->pllref_clk / 1000;

	/* find frequency mapping */
	for (i = 0; i < ARRAY_SIZE(dppa_map_phy_141); i++) {
		if (dsi->lane_mbps < dppa_map_phy_141[i].data_rate) {
			i--;
			break;
		}
	}

	switch (dppa_map_phy_141[i].odf) {
	case(3):
		odf = 8;
		break;
	case(2):
		odf = 4;
		break;
	case(1):
		odf = 2;
		break;
	default:
		odf = 1;
		break;
	}

	dsi_phy_141_pll_get_params(dsi, pll_in_khz, pll_out_khz, &idf, &ndiv, &odf);

	ccf = ((pll_in_khz / 1000 - 17)) * 4;
	hsfreq = dppa_map_phy_141[i].hs_freq;

	vco = dppa_map_phy_141[i].vco;
	bias = 0x10;
	int1 = 0x00;
	gmp = 0x01;
	prop = dppa_map_phy_141[i].prop;

	/* set DLD, HSFR & CCF */
	val = (hsfreq << 8) | ccf;
	dsi_write(dsi, DSI_WPCR1, val);

	val = ((ndiv - 2) << 4) | (idf - 1);
	dsi_write(dsi, DSI_WRPCR0, val);

	val = ((odf - 1) << 28) | (vco << 24) | (bias << 16) | (int1 << 8) | (gmp << 6) | prop;
	dsi_write(dsi, DSI_WRPCR1, val);

	dsi_write(dsi, DSI_PCTLR, PCTLR_CKEN);

	dsi_update_bits(dsi, DSI_WRPCR2, WRPCR2_SEL, 0x01);

	dsi_set(dsi, DSI_WRPCR2, WRPCR2_UPD);
	mdelay(1);

	dsi_clear(dsi, DSI_WRPCR2, WRPCR2_UPD);
	mdelay(1);

	dsi_set(dsi, DSI_PCTLR, PCTLR_PWEN | PCTLR_DEN);

	ret = readl_poll_timeout(dsi->base + DSI_PSR, val, val & PSR_PSSC, TIMEOUT_US);
	if (ret)
		dev_err(dev, "!TIMEOUT! waiting PLL, let's continue\n");

	dev_dbg(dev, "IDF %d ODF %d NDIV %d\n", idf, odf, ndiv);
	dev_dbg(dev, "VCO %d BIAS %d INT %d GMP %d PROP %d\n", vco, bias, int1, gmp, prop);

	dsi_set(dsi, DSI_WRPCR2, WRPCR2_PLLEN);

	return 0;
}

static void dsi_phy_141_post_set_mode(void *priv_data, unsigned long mode_flags)
{
	struct mipi_dsi_device *device = priv_data;
	struct udevice *dev = device->dev;
	struct stm32_dsi_priv *dsi = dev_get_priv(dev);

	dev_dbg(dev, "Set mode %p enable %ld\n", dsi,
		mode_flags & MIPI_DSI_MODE_VIDEO);

	if (!dsi)
		return;

	/*
	 * DSI wrapper must be enabled in video mode & disabled in command mode.
	 * If wrapper is enabled in command mode, the display controller
	 * register access will hang.
	 */

	if (mode_flags & MIPI_DSI_MODE_VIDEO)
		dsi_set(dsi, DSI_WCR, WCR_DSIEN);
	else
		dsi_clear(dsi, DSI_WCR, WCR_DSIEN);
}

static int dsi_phy_141_get_lane_mbps(void *priv_data, struct display_timing *timings,
				     u32 lanes, u32 format, unsigned int *lane_mbps)
{
	struct mipi_dsi_device *device = priv_data;
	struct udevice *dev = device->dev;
	struct stm32_dsi_priv *dsi = dev_get_priv(dev);
	int idf, ndiv, odf, pll_in_khz, pll_out_khz;
	int bpp, i;

	/* Update lane capabilities according to hw version */
	dsi->lane_min_kbps = LANE_MIN_PHY_141_KBPS;
	dsi->lane_max_kbps = LANE_MAX_PHY_141_KBPS;

	pll_in_khz = dsi->pllref_clk / 1000;

	/* Compute requested pll out */
	bpp = mipi_dsi_pixel_format_to_bpp(format);
	pll_out_khz = (timings->pixelclock.typ / 1000) * bpp / (lanes * 2);
	/* Add 20% to pll out to be higher than pixel bw (burst mode only) */
	pll_out_khz = (pll_out_khz * 12) / 10;
	if (pll_out_khz > dsi->lane_max_kbps) {
		pll_out_khz = dsi->lane_max_kbps;
		dev_warn(dev, "Warning max phy mbps is used\n");
	}
	if (pll_out_khz < dsi->lane_min_kbps) {
		pll_out_khz = dsi->lane_min_kbps;
		dev_warn(dev, "Warning min phy mbps is used\n");
	}

	/* find frequency mapping */
	for (i = 0; i < ARRAY_SIZE(dppa_map_phy_141); i++) {
		if (dsi->lane_mbps < dppa_map_phy_141[i].data_rate) {
			i--;
			break;
		}
	}

	switch (dppa_map_phy_141[i].odf) {
	case(3):
		odf = 8;
		break;
	case(2):
		odf = 4;
		break;
	case(1):
		odf = 2;
		break;
	default:
		odf = 1;
		break;
	}

	dsi_phy_141_pll_get_params(dsi, pll_in_khz, pll_out_khz, &idf, &ndiv, &odf);

	/* Get the adjusted lane data rate value, lane data rate = 2 * pll output */
	*lane_mbps = 2 * dsi_pll_get_clkout_khz(pll_in_khz, idf, ndiv, odf) / 1000;
	dsi->lane_mbps = *lane_mbps;

	dev_dbg(dev, "pll_in %ukHz pll_out %ukHz lane_mbps %uMHz\n",
		pll_in_khz, pll_out_khz, *lane_mbps);

	return 0;
}

static int dsi_phy_141_get_timing(void *priv_data, unsigned int lane_mbps,
				  struct mipi_dsi_phy_timing *timing)
{
	struct mipi_dsi_device *device = priv_data;
	struct udevice *dev = device->dev;
	int i;

	for (i = 0; i < ARRAY_SIZE(hstt_phy_141_table); i++)
		if (lane_mbps < hstt_phy_141_table[i].maxfreq)
			break;

	if (i == ARRAY_SIZE(hstt_phy_141_table))
		i--;

	*timing = hstt_phy_141_table[i].timing;

	dev_dbg(dev, "data hs2lp %d data lp2hs %d\n", timing->data_hs2lp, timing->data_lp2hs);
	dev_dbg(dev, "data hs2lp %d data lp2hs %d\n", timing->clk_hs2lp, timing->clk_lp2hs);

	return 0;
}

static const struct mipi_dsi_phy_ops dsi_stm_phy_141_ops = {
	.init = dsi_phy_141_init,
	.get_lane_mbps = dsi_phy_141_get_lane_mbps,
	.post_set_mode = dsi_phy_141_post_set_mode,
	.get_timing = dsi_phy_141_get_timing,
};

static int stm32_dsi_get_panel(struct udevice *dev, struct udevice **panel)
{
	ofnode ep_node, node, ports, remote;
	u32 remote_phandle;
	int ret;

	ports = ofnode_find_subnode(dev_ofnode(dev), "ports");
	if (!ofnode_valid(ports)) {
		dev_dbg(dev, "Remote bridge subnode\n");
		return ret;
	}

	for (node = ofnode_first_subnode(ports);
	     ofnode_valid(node);
	     node = dev_read_next_subnode(node)) {
		ep_node = ofnode_first_subnode(node);
		if (!ofnode_valid(ep_node))
			continue;

		ret = ofnode_read_u32(ep_node, "remote-endpoint", &remote_phandle);
		if (ret) {
			dev_dbg(dev, "%s(%s): Could not find remote-endpoint property\n",
				__func__, dev_read_name(dev));
			return ret;
		}

		remote = ofnode_get_by_phandle(remote_phandle);
		if (!ofnode_valid(remote)) {
			dev_dbg(dev, "%s(%s): Remote is not valid\n", __func__, dev_read_name(dev));
			return -EINVAL;
		}

		while (ofnode_valid(remote)) {
			remote = ofnode_get_parent(remote);
			if (!ofnode_valid(remote)) {
				dev_dbg(dev, "%s(%s): no UCLASS_DISPLAY for remote-endpoint\n",
					__func__, dev_read_name(dev));
				continue;
			}

			uclass_get_device_by_ofnode(UCLASS_PANEL, remote, panel);
			if (*panel)
				break;
		}
	}

	/* Sanity check, we can get out of the loop without having a clean ofnode */
	if (!(*panel))
		ret = -EINVAL;
	else
		if (!ofnode_valid(dev_ofnode(*panel)))
			ret = -EINVAL;

	return ret;
}

static int stm32_dsi_attach(struct udevice *dev)
{
	struct stm32_dsi_priv *priv = dev_get_priv(dev);
	struct mipi_dsi_device *device = &priv->device;
	struct mipi_dsi_panel_plat *mplat;
	struct display_timing timings;
	int ret;

	ret = stm32_dsi_get_panel(dev, &priv->panel);
	if (ret) {
		dev_err(dev, "No panel found %d\n", ret);
		return ret;
	}

	mplat = dev_get_plat(priv->panel);

	/* check that the panel contains platform data */
	if (!mplat)
		return -EINVAL;

	mplat->device = &priv->device;
	device->lanes = mplat->lanes;
	device->format = mplat->format;
	device->mode_flags = mplat->mode_flags;

	ret = panel_get_display_timing(priv->panel, &timings);
	if (ret) {
		ret = ofnode_decode_display_timing(dev_ofnode(priv->panel),
						   0, &timings);
		if (ret) {
			dev_err(dev, "decode display timing error %d\n", ret);
			return ret;
		}
	}

	ret = uclass_get_device(UCLASS_DSI_HOST, 0, &priv->dsi_host);
	if (ret) {
		dev_err(dev, "No video dsi host detected %d\n", ret);
		return ret;
	}

	if (priv->hw_version == HWVER_141 &&
	    (IS_ENABLED(CONFIG_STM32MP25X) || IS_ENABLED(CONFIG_STM32MP23X))) {
		ret = dsi_host_init(priv->dsi_host, device, &timings, 4,
				    &dsi_stm_phy_141_ops);
		if (ret) {
			dev_err(dev, "failed to initialize mipi dsi host\n");
			return ret;
		}
	} else if ((priv->hw_version == HWVER_131 || priv->hw_version == HWVER_130) &&
		   IS_ENABLED(CONFIG_STM32MP15X)) {
		ret = dsi_host_init(priv->dsi_host, device, &timings, 2,
				    &dsi_stm_phy_ops);
		if (ret) {
			dev_err(dev, "failed to initialize mipi dsi host\n");
			return ret;
		}
	} else {
		dev_err(dev, "Hardware version not supported\n");
		return -EINVAL;
	}

	return 0;
}

static int stm32_dsi_set_backlight(struct udevice *dev, int percent)
{
	struct stm32_dsi_priv *priv = dev_get_priv(dev);
	int ret;

	ret = panel_enable_backlight(priv->panel);
	if (ret) {
		dev_err(dev, "panel %s enable backlight error %d\n",
			priv->panel->name, ret);
		return ret;
	}

	ret = dsi_host_enable(priv->dsi_host);
	if (ret) {
		dev_err(dev, "failed to enable mipi dsi host\n");
		return ret;
	}

	return 0;
}

static int stm32_dsi_bind(struct udevice *dev)
{
	int ret;

	ret = device_bind_driver_to_node(dev, "dw_mipi_dsi", "dsihost",
					 dev_ofnode(dev), NULL);
	if (ret)
		return ret;

	return dm_scan_fdt_dev(dev);
}

static int stm32_dsi_probe(struct udevice *dev)
{
	struct stm32_dsi_priv *priv = dev_get_priv(dev);
	struct mipi_dsi_device *device = &priv->device;
	struct reset_ctl rst;
	struct clk clk;
	int ret;

	device->dev = dev;

	priv->base = dev_read_addr_ptr(dev);
	if (!priv->base) {
		dev_err(dev, "dsi dt register address error\n");
		return -EINVAL;
	}

	if (device_is_compatible(dev, "st,stm32-dsi")) {
		ret =  device_get_supply_regulator(dev, "phy-dsi-supply",
						   &priv->vdd_reg);
		if (ret && ret != -ENOENT) {
			dev_err(dev, "Warning: cannot get phy dsi supply\n");
			return -ENODEV;
		}

		if (ret != -ENOENT) {
			ret = regulator_set_enable(priv->vdd_reg, true);
			if (ret)
				return ret;
		}
	}

	if (device_is_compatible(dev, "st,stm32mp25-dsi")) {
		ret =  device_get_supply_regulator(dev, "vdd-supply",
						   &priv->vdd_reg);
		if (ret && ret != -ENOENT) {
			dev_err(dev, "Warning: cannot get phy dsi supply\n");
			return -ENODEV;
		}

		if (ret != -ENOENT) {
			ret = regulator_set_enable(priv->vdd_reg, true);
			if (ret)
				return ret;
		}

		ret =  device_get_supply_regulator(dev, "vdda18",
						   &priv->vdda18_reg);
		if (ret && ret != -ENOENT) {
			dev_err(dev, "Warning: cannot get vdda18 supply\n");
			return ret;
		}

		if (ret != -ENOENT) {
			ret = regulator_set_enable(priv->vdda18_reg, true);
			if (ret)
				return ret;
		}
	}

	ret = clk_get_by_name(device->dev, "pclk", &clk);
	if (ret) {
		dev_err(dev, "peripheral clock get error %d\n", ret);
		goto err_reg;
	}

	ret = clk_enable(&clk);
	if (ret) {
		dev_err(dev, "peripheral clock enable error %d\n", ret);
		goto err_reg;
	}

	ret = clk_get_by_name(dev, "ref", &clk);
	if (ret) {
		dev_err(dev, "pll reference clock get error %d\n", ret);
		goto err_clk;
	}

	priv->pllref_clk = (unsigned int)clk_get_rate(&clk);

	ret = reset_get_by_index(device->dev, 0, &rst);
	if (ret) {
		dev_err(dev, "missing dsi hardware reset\n");
		goto err_clk;
	}

	/* Reset */
	reset_deassert(&rst);

	/* check hardware version */
	priv->hw_version = dsi_read(priv, DSI_VERSION) & VERSION;
	if (priv->hw_version != HWVER_130 &&
	    priv->hw_version != HWVER_131 &&
	    priv->hw_version != HWVER_141) {
		dev_err(dev, "DSI version 0x%x not supported\n", priv->hw_version);
		dev_dbg(dev, "remove and unbind all DSI child\n");
		device_chld_remove(dev, NULL, DM_REMOVE_NORMAL);
		device_chld_unbind(dev, NULL);
		ret = -ENODEV;
		goto err_clk;
	}

	return 0;
err_clk:
	clk_disable(&clk);
err_reg:
	regulator_set_enable(priv->vdda18_reg, false);
	regulator_set_enable(priv->vdd_reg, false);

	return ret;
}

struct video_bridge_ops stm32_dsi_ops = {
	.attach = stm32_dsi_attach,
	.set_backlight = stm32_dsi_set_backlight,
};

static const struct udevice_id stm32_dsi_ids[] = {
	{ .compatible = "st,stm32-dsi"},
	{ .compatible = "st,stm32mp25-dsi"},
	{ }
};

U_BOOT_DRIVER(stm32_dsi) = {
	.name				= "stm32-display-dsi",
	.id				= UCLASS_VIDEO_BRIDGE,
	.of_match			= stm32_dsi_ids,
	.bind				= stm32_dsi_bind,
	.probe				= stm32_dsi_probe,
	.ops				= &stm32_dsi_ops,
	.priv_auto		= sizeof(struct stm32_dsi_priv),
};
