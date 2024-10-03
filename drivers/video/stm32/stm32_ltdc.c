// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2017-2018 STMicroelectronics - All Rights Reserved
 * Author(s): Philippe Cornu <philippe.cornu@st.com> for STMicroelectronics.
 *	      Yannick Fertre <yannick.fertre@st.com> for STMicroelectronics.
 */
#define LOG_CATEGORY UCLASS_VIDEO

#include <common.h>
#include <clk.h>
#include <display.h>
#include <dm.h>
#include <log.h>
#include <panel.h>
#include <regmap.h>
#include <reset.h>
#include <syscon.h>
#include <video.h>
#include <video_bridge.h>
#include <asm/io.h>
#include <dm/device-internal.h>
#include <dm/uclass-internal.h>
#include <dm/device_compat.h>
#include <dm/pinctrl.h>
#include <linux/bitops.h>

#if CONFIG_IS_ENABLED(ARCH_STM32MP)
/* direct access to RIFSC function, waiting firewall uclass */
#include <mach/rif.h>
static int stm32_check_access_by_id(ofnode device_node, u32 id)
{
	return stm32_rifsc_check_access_by_id(device_node, id);
}
#else
static int stm32_check_access_by_id(ofnode device_node, u32 id)
{
	return -EACCES;
}
#endif

struct stm32_ltdc_priv {
	void __iomem *regs;
	enum video_log2_bpp l2bpp;
	u32 bg_col_argb;
	const u32 *layer_regs;
	const u32 *pix_fmt_hw;
	u32 crop_x, crop_y, crop_w, crop_h;
	u32 alpha;
	u32 hw_version;
	struct udevice *bridge;
	struct udevice *panel;
};

/* Layer register offsets */
static const u32 layer_regs_a0[] = {
	0x80,	/* L1 configuration 0 */
	0x00,	/* not available */
	0x00,	/* not available */
	0x84,	/* L1 control register */
	0x88,	/* L1 window horizontal position configuration */
	0x8c,	/* L1 window vertical position configuration */
	0x90,	/* L1 color keying configuration */
	0x94,	/* L1 pixel format configuration */
	0x98,	/* L1 constant alpha configuration */
	0x9c,	/* L1 default color configuration */
	0xa0,	/* L1 blending factors configuration */
	0x00,	/* not available */
	0x00,	/* not available */
	0xac,	/* L1 color frame buffer address */
	0xb0,	/* L1 color frame buffer length */
	0xb4,	/* L1 color frame buffer line number */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0xc4,	/* L1 CLUT write */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00	/* not available */
};

static const u32 layer_regs_a1[] = {
	0x80,	/* L1 configuration 0 */
	0x84,	/* L1 configuration 1 */
	0x00,	/* L1 reload control */
	0x88,	/* L1 control register */
	0x8c,	/* L1 window horizontal position configuration */
	0x90,	/* L1 window vertical position configuration */
	0x94,	/* L1 color keying configuration */
	0x98,	/* L1 pixel format configuration */
	0x9c,	/* L1 constant alpha configuration */
	0xa0,	/* L1 default color configuration */
	0xa4,	/* L1 blending factors configuration */
	0xa8,	/* L1 burst length configuration */
	0x00,	/* not available */
	0xac,	/* L1 color frame buffer address */
	0xb0,	/* L1 color frame buffer length */
	0xb4,	/* L1 color frame buffer line number */
	0xb8,	/* L1 auxiliary frame buffer address 0 */
	0xbc,	/* L1 auxiliary frame buffer address 1 */
	0xc0,	/* L1 auxiliary frame buffer length */
	0xc4,	/* L1 auxiliary frame buffer line number */
	0xc8,	/* L1 CLUT write */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00	/* not available */
};

static const u32 layer_regs_a2[] = {
	0x100,	/* L1 configuration 0 */
	0x104,	/* L1 configuration 1 */
	0x108,	/* L1 reload control */
	0x10c,	/* L1 control register */
	0x110,	/* L1 window horizontal position configuration */
	0x114,	/* L1 window vertical position configuration */
	0x118,	/* L1 color keying configuration */
	0x11c,	/* L1 pixel format configuration */
	0x120,	/* L1 constant alpha configuration */
	0x124,	/* L1 default color configuration */
	0x128,	/* L1 blending factors configuration */
	0x12c,	/* L1 burst length configuration */
	0x130,	/* L1 planar configuration */
	0x134,	/* L1 color frame buffer address */
	0x138,	/* L1 color frame buffer length */
	0x13c,	/* L1 color frame buffer line number */
	0x140,	/* L1 auxiliary frame buffer address 0 */
	0x144,	/* L1 auxiliary frame buffer address 1 */
	0x148,	/* L1 auxiliary frame buffer length */
	0x14c,	/* L1 auxiliary frame buffer line number */
	0x150,	/* L1 CLUT write */
	0x154,	/* not available */
	0x158,	/* not available */
	0x15c,	/* not available */
	0x160,	/* not available */
	0x164,	/* not available */
	0x168,	/* not available */
	0x16c,	/* L1 Conversion YCbCr RGB 0 */
	0x170,	/* L1 Conversion YCbCr RGB 1 */
	0x174,	/* L1 Flexible Pixel Format 0 */
	0x178	/* L1 Flexible Pixel Format 1 */
};

/* LTDC main registers */
#define LTDC_IDR	0x00	/* IDentification */
#define LTDC_LCR	0x04	/* Layer Count */
#define LTDC_SSCR	0x08	/* Synchronization Size Configuration */
#define LTDC_BPCR	0x0C	/* Back Porch Configuration */
#define LTDC_AWCR	0x10	/* Active Width Configuration */
#define LTDC_TWCR	0x14	/* Total Width Configuration */
#define LTDC_GCR	0x18	/* Global Control */
#define LTDC_GC1R	0x1C	/* Global Configuration 1 */
#define LTDC_GC2R	0x20	/* Global Configuration 2 */
#define LTDC_SRCR	0x24	/* Shadow Reload Configuration */
#define LTDC_GACR	0x28	/* GAmma Correction */
#define LTDC_BCCR	0x2C	/* Background Color Configuration */
#define LTDC_IER	0x34	/* Interrupt Enable */
#define LTDC_ISR	0x38	/* Interrupt Status */
#define LTDC_ICR	0x3C	/* Interrupt Clear */
#define LTDC_LIPCR	0x40	/* Line Interrupt Position Conf. */
#define LTDC_CPSR	0x44	/* Current Position Status */
#define LTDC_CDSR	0x48	/* Current Display Status */
#define LTDC_RB0AR	0x80	/* Rotation Buffer 0 address */
#define LTDC_RB1AR	0x84	/* Rotation Buffer 1 address */
#define LTDC_RBPR	0x88	/* Rotation Buffer Pitch */

/* Layer register offsets */
#define LTDC_L1C0R	(priv->layer_regs[0])	/* L1 configuration 0 */
#define LTDC_L1C1R	(priv->layer_regs[1])	/* L1 configuration 1 */
#define LTDC_L1RCR	(priv->layer_regs[2])	/* L1 reload control */
#define LTDC_L1CR	(priv->layer_regs[3])	/* L1 control register */
#define LTDC_L1WHPCR	(priv->layer_regs[4])	/* L1 window horizontal position configuration */
#define LTDC_L1WVPCR	(priv->layer_regs[5])	/* L1 window vertical position configuration */
#define LTDC_L1CKCR	(priv->layer_regs[6])	/* L1 color keying configuration */
#define LTDC_L1PFCR	(priv->layer_regs[7])	/* L1 pixel format configuration */
#define LTDC_L1CACR	(priv->layer_regs[8])	/* L1 constant alpha configuration */
#define LTDC_L1DCCR	(priv->layer_regs[9])	/* L1 default color configuration */
#define LTDC_L1BFCR	(priv->layer_regs[10])	/* L1 blending factors configuration */
#define LTDC_L1BLCR	(priv->layer_regs[11])	/* L1 burst length configuration */
#define LTDC_L1PCR	(priv->layer_regs[12])	/* L1 planar configuration */
#define LTDC_L1CFBAR	(priv->layer_regs[13])	/* L1 color frame buffer address */
#define LTDC_L1CFBLR	(priv->layer_regs[14])	/* L1 color frame buffer length */
#define LTDC_L1CFBLNR	(priv->layer_regs[15])	/* L1 color frame buffer line number */
#define LTDC_L1AFBA0R	(priv->layer_regs[16])	/* L1 auxiliary frame buffer address 0 */
#define LTDC_L1AFBA1R	(priv->layer_regs[17])	/* L1 auxiliary frame buffer address 1 */
#define LTDC_L1AFBLR	(priv->layer_regs[18])	/* L1 auxiliary frame buffer length */
#define LTDC_L1AFBLNR	(priv->layer_regs[19])	/* L1 auxiliary frame buffer line number */
#define LTDC_L1CLUTWR	(priv->layer_regs[20])	/* L1 CLUT write */
#define LTDC_L1SISR	(priv->layer_regs[21])	/* L1 scaler input size */
#define LTDC_L1SOSR	(priv->layer_regs[22])	/* L1 scaler output size */
#define LTDC_L1SVSFR	(priv->layer_regs[23])	/* L1 scaler vertical scaling factor */
#define LTDC_L1SVSPR	(priv->layer_regs[24])	/* L1 scaler vertical scaling phase */
#define LTDC_L1SHSFR	(priv->layer_regs[25])	/* L1 scaler horizontal scaling factor */
#define LTDC_L1SHSPR	(priv->layer_regs[26])	/* L1 scaler horizontal scaling phase */
#define LTDC_L1CYR0R	(priv->layer_regs[27])	/* L1 Conversion YCbCr RGB 0 */
#define LTDC_L1CYR1R	(priv->layer_regs[28])	/* L1 Conversion YCbCr RGB 1 */
#define LTDC_L1FPF0R	(priv->layer_regs[29])	/* L1 Flexible Pixel Format 0 */
#define LTDC_L1FPF1R	(priv->layer_regs[30])	/* L1 Flexible Pixel Format 1 */

/* Bit definitions */
#define SSCR_VSH	GENMASK(10, 0)	/* Vertical Synchronization Height */
#define SSCR_HSW	GENMASK(27, 16)	/* Horizontal Synchronization Width */

#define BPCR_AVBP	GENMASK(10, 0)	/* Accumulated Vertical Back Porch */
#define BPCR_AHBP	GENMASK(27, 16)	/* Accumulated Horizontal Back Porch */

#define AWCR_AAH	GENMASK(10, 0)	/* Accumulated Active Height */
#define AWCR_AAW	GENMASK(27, 16)	/* Accumulated Active Width */

#define TWCR_TOTALH	GENMASK(10, 0)	/* TOTAL Height */
#define TWCR_TOTALW	GENMASK(27, 16)	/* TOTAL Width */

#define GCR_LTDCEN	BIT(0)		/* LTDC ENable */
#define GCR_ROTEN	BIT(2)		/* ROTation ENable */
#define GCR_DEN		BIT(16)		/* Dither ENable */
#define GCR_PCPOL	BIT(28)		/* Pixel Clock POLarity-Inverted */
#define GCR_DEPOL	BIT(29)		/* Data Enable POLarity-High */
#define GCR_VSPOL	BIT(30)		/* Vertical Synchro POLarity-High */
#define GCR_HSPOL	BIT(31)		/* Horizontal Synchro POLarity-High */

#define GC1R_WBCH	GENMASK(3, 0)	/* Width of Blue CHannel output */
#define GC1R_WGCH	GENMASK(7, 4)	/* Width of Green Channel output */
#define GC1R_WRCH	GENMASK(11, 8)	/* Width of Red Channel output */
#define GC1R_PBEN	BIT(12)		/* Precise Blending ENable */
#define GC1R_DT		GENMASK(15, 14)	/* Dithering Technique */
#define GC1R_GCT	GENMASK(19, 17)	/* Gamma Correction Technique */
#define GC1R_SHREN	BIT(21)		/* SHadow Registers ENabled */
#define GC1R_BCP	BIT(22)		/* Background Colour Programmable */
#define GC1R_BBEN	BIT(23)		/* Background Blending ENabled */
#define GC1R_LNIP	BIT(24)		/* Line Number IRQ Position */
#define GC1R_TP		BIT(25)		/* Timing Programmable */
#define GC1R_IPP	BIT(26)		/* IRQ Polarity Programmable */
#define GC1R_SPP	BIT(27)		/* Sync Polarity Programmable */
#define GC1R_DWP	BIT(28)		/* Dither Width Programmable */
#define GC1R_STREN	BIT(29)		/* STatus Registers ENabled */
#define GC1R_BMEN	BIT(31)		/* Blind Mode ENabled */

#define GC2R_EDCA	BIT(0)		/* External Display Control Ability  */
#define GC2R_STSAEN	BIT(1)		/* Slave Timing Sync Ability ENabled */
#define GC2R_DVAEN	BIT(2)		/* Dual-View Ability ENabled */
#define GC2R_DPAEN	BIT(3)		/* Dual-Port Ability ENabled */
#define GC2R_BW		GENMASK(6, 4)	/* Bus Width (log2 of nb of bytes) */
#define GC2R_EDCEN	BIT(7)		/* External Display Control ENabled */

#define SRCR_IMR	BIT(0)		/* IMmediate Reload */
#define SRCR_VBR	BIT(1)		/* Vertical Blanking Reload */

#define LXCR_LEN	BIT(0)		/* Layer ENable */
#define LXCR_COLKEN	BIT(1)		/* Color Keying Enable */
#define LXCR_CLUTEN	BIT(4)		/* Color Look-Up Table ENable */
#define LXCR_HMEN	BIT(8)		/* Horizontal Mirroring ENable */
#define LXCR_SCEN	BIT(10)		/* SCaler ENable */

#define LXWHPCR_WHSTPOS	GENMASK(11, 0)	/* Window Horizontal StarT POSition */
#define LXWHPCR_WHSPPOS	GENMASK(27, 16)	/* Window Horizontal StoP POSition */

#define LXWVPCR_WVSTPOS	GENMASK(10, 0)	/* Window Vertical StarT POSition */
#define LXWVPCR_WVSPPOS	GENMASK(26, 16)	/* Window Vertical StoP POSition */

#define LXPFCR_PF	GENMASK(2, 0)	/* Pixel Format */

#define LXCACR_CONSTA	GENMASK(7, 0)	/* CONSTant Alpha */

#define LXBFCR_BF2	GENMASK(2, 0)	/* Blending Factor 2 */
#define LXBFCR_BF1	GENMASK(10, 8)	/* Blending Factor 1 */

#define LXCFBLR_CFBLL	GENMASK(12, 0)	/* Color Frame Buffer Line Length */
#define LXCFBLR_CFBP	GENMASK(28, 16)	/* Color Frame Buffer Pitch in bytes */

#define LXCFBLNR_CFBLN	GENMASK(10, 0)	/* Color Frame Buffer Line Number */

#define BF1_PAXCA	0x600		/* Pixel Alpha x Constant Alpha */
#define BF1_CA		0x400		/* Constant Alpha */
#define BF2_1PAXCA	0x007		/* 1 - (Pixel Alpha x Constant Alpha) */
#define BF2_1CA		0x005		/* 1 - Constant Alpha */

#define NB_PF		8		/* Max nb of HW pixel format */

#define HWVER_10200 0x010200
#define HWVER_10300 0x010300
#define HWVER_20101 0x020101
#define HWVER_40100 0x040100
#define HWVER_40101 0x040101

#define SYSCFG_DISPLAYCLKCR 0x5000
#define DISPLAYCLKCR_LVDS	0x01
#define DISPLAYCLKCR_DPI	0x02

enum stm32_ltdc_pix_fmt {
	PF_ARGB8888 = 0,	/* ARGB [32 bits] */
	PF_ABGR8888,		/* ABGR [32 bits] */
	PF_BGRA8888,		/* BGRA [32 bits] */
	PF_RGBA8888,		/* RGBA [32 bits] */
	PF_RGB888,		/* RGB [24 bits] */
	PF_BGR565,		/* RGB [16 bits] */
	PF_RGB565,		/* RGB [16 bits] */
	PF_ARGB1555,		/* ARGB A:1 bit RGB:15 bits [16 bits] */
	PF_ARGB4444,		/* ARGB A:4 bits R/G/B: 4 bits each [16 bits] */
	PF_AL44,		/* Alpha:4 bits + indexed 4 bits [8 bits] */
	PF_AL88,		/* Alpha:8 bits + indexed 8 bits [16 bits] */
	PF_L8,			/* Indexed 8 bits [8 bits] */
	PF_NONE
};

static const enum stm32_ltdc_pix_fmt pix_fmt_a0[NB_PF] = {
	PF_ARGB8888,		/* 0x00 */
	PF_RGB888,		/* 0x01 */
	PF_RGB565,		/* 0x02 */
	PF_ARGB1555,		/* 0x03 */
	PF_ARGB4444,		/* 0x04 */
	PF_L8,			/* 0x05 */
	PF_AL44,		/* 0x06 */
	PF_AL88			/* 0x07 */
};

static const enum stm32_ltdc_pix_fmt pix_fmt_a1[NB_PF] = {
	PF_ARGB8888,		/* 0x00 */
	PF_RGB888,		/* 0x01 */
	PF_RGB565,		/* 0x02 */
	PF_RGBA8888,		/* 0x03 */
	PF_AL44,		/* 0x04 */
	PF_L8,			/* 0x05 */
	PF_ARGB1555,		/* 0x06 */
	PF_ARGB4444		/* 0x07 */
};

static const enum stm32_ltdc_pix_fmt pix_fmt_a2[NB_PF] = {
	PF_ARGB8888,		/* 0x00 */
	PF_ABGR8888,		/* 0x01 */
	PF_RGBA8888,		/* 0x02 */
	PF_BGRA8888,		/* 0x03 */
	PF_RGB565,		/* 0x04 */
	PF_BGR565,		/* 0x05 */
	PF_RGB888,		/* 0x06 */
	PF_NONE			/* 0x07 (flexible pixel format) */
};

/* TODO add more color format support */
static u32 stm32_ltdc_get_pixel_format(enum video_log2_bpp l2bpp)
{
	enum stm32_ltdc_pix_fmt pf;

	switch (l2bpp) {
	case VIDEO_BPP16:
		pf = PF_RGB565;
		break;

	case VIDEO_BPP32:
		pf = PF_ARGB8888;
		break;

	case VIDEO_BPP8:
		pf = PF_L8;
		break;

	case VIDEO_BPP1:
	case VIDEO_BPP2:
	case VIDEO_BPP4:
	default:
		log_warning("warning %dbpp not supported yet, %dbpp instead\n",
			    VNBITS(l2bpp), VNBITS(VIDEO_BPP16));
		pf = PF_RGB565;
		break;
	}

	log_debug("%d bpp -> ltdc pf %d\n", VNBITS(l2bpp), pf);

	return (u32)pf;
}

static bool has_alpha(u32 fmt)
{
	switch (fmt) {
	case PF_ARGB8888:
	case PF_ARGB1555:
	case PF_ARGB4444:
	case PF_AL44:
	case PF_AL88:
		return true;
	case PF_RGB888:
	case PF_RGB565:
	case PF_L8:
	default:
		return false;
	}
}

static void stm32_ltdc_enable(struct stm32_ltdc_priv *priv)
{
	/* Reload configuration immediately & enable LTDC */
	setbits_le32(priv->regs + LTDC_SRCR, SRCR_IMR);
	setbits_le32(priv->regs + LTDC_GCR, GCR_LTDCEN);
}

static void stm32_ltdc_set_mode(struct udevice *dev,
				struct display_timing *timings)
{
	struct stm32_ltdc_priv *priv = dev_get_priv(dev);
	void __iomem *regs = priv->regs;
	u32 hsync, vsync, acc_hbp, acc_vbp, acc_act_w, acc_act_h;
	u32 pitch, rota0_buf, rota1_buf;
	u32 total_w, total_h;
	u32 out_values[4];
	u32 rotation;
	u32 phandle;
	u32 base;
	u32 val;
	int size;
	ofnode remote;

	/* Rotation supported only by mp25 SOCs */
	if (ofnode_device_is_compatible(dev_ofnode(dev), "st,stm32mp25-ltdc"))
		rotation = dev_read_u32_default(priv->panel, "rotation", 0);

	/* Convert video timings to ltdc timings */
	hsync = timings->hsync_len.typ - 1;
	vsync = timings->vsync_len.typ - 1;
	acc_hbp = hsync + timings->hback_porch.typ;
	acc_vbp = vsync + timings->vback_porch.typ;
	acc_act_w = acc_hbp + timings->hactive.typ;
	acc_act_h = acc_vbp + timings->vactive.typ;
	total_w = acc_act_w + timings->hfront_porch.typ;
	total_h = acc_act_h + timings->vfront_porch.typ;

	/* check that an output rotation is required */
	if (rotation == 90 || rotation == 270) {
		if (ofnode_read_u32(dev_ofnode(dev), "rotation-memory", &phandle)) {
			dev_err(dev, "%s(%s): Could not find rotation-memory property\n",
				__func__, dev_read_name(dev));
			return;
		}

		remote = ofnode_get_by_phandle(phandle);
		if (!ofnode_valid(remote)) {
			dev_err(dev, "%s(%s): Could not get rotation memory handle\n",
				__func__, dev_read_name(dev));
			return;
		}

		if (ofnode_read_u32_array(remote, "reg", out_values, 4)) {
			dev_err(dev, "%s(%s): Could not get rotation memory reg property\n",
				__func__, dev_read_name(dev));
			return;
		}

		/* get base & size of memory rotation buffer */
		base = out_values[1];
		size = out_values[3];

		/*
		 * Size of the rotation buffer must be larger than the size
		 * of two frames (format RGB24).
		 */
		if (size <  timings->hactive.typ *  timings->vactive.typ * 2 * 3) {
			dev_err(dev, "%s(%s): Rotation buffer too small: %d\n",
				__func__, dev_read_name(dev), size);
			return;
		}

		/* Panel width should not exceed 1366 pixels */
		if (timings->hactive.typ > 1366) {
			dev_err(dev, "%s(%s): Panel width should not exceed 1366 pixels: %d\n",
				__func__, dev_read_name(dev), size);
			return;
		}

		rota0_buf = (u32)base;
		rota1_buf = (u32)base + (size >> 1);

		writel(rota0_buf, regs + LTDC_RB0AR);
		writel(rota1_buf, regs + LTDC_RB1AR);

		/*
		 * LTDC_RBPR register is used define the pitch (line-to-line address increment)
		 * of the stored rotation buffer. The pitch is proportional to the width of the
		 * composed display (before rotation) and,(after rotation) proportional to the
		 * non-raster dimension of the display panel.
		 */
		pitch = ((timings->hactive.typ - 1 + 9) / 10) * 64;
		writel(pitch, regs + LTDC_RBPR);

		/* Synchronization sizes */
		val = (vsync << 16) | hsync;
		clrsetbits_le32(regs + LTDC_SSCR, SSCR_VSH | SSCR_HSW, val);

		/* Accumulated back porch */
		val = (acc_vbp << 16) | acc_hbp;
		clrsetbits_le32(regs + LTDC_BPCR, BPCR_AVBP | BPCR_AHBP, val);

		/* Accumulated active width */
		val = (acc_act_h << 16) | acc_act_w;
		clrsetbits_le32(regs + LTDC_AWCR, AWCR_AAW | AWCR_AAH, val);

		/* Total width & height */
		val = (total_h << 16) | total_w;
		clrsetbits_le32(regs + LTDC_TWCR, TWCR_TOTALH | TWCR_TOTALW, val);

		setbits_le32(regs + LTDC_LIPCR, acc_act_w + 1);
	} else {
		/* Synchronization sizes */
		val = (hsync << 16) | vsync;
		clrsetbits_le32(regs + LTDC_SSCR, SSCR_VSH | SSCR_HSW, val);

		/* Accumulated back porch */
		val = (acc_hbp << 16) | acc_vbp;
		clrsetbits_le32(regs + LTDC_BPCR, BPCR_AVBP | BPCR_AHBP, val);

		/* Accumulated active width */
		val = (acc_act_w << 16) | acc_act_h;
		clrsetbits_le32(regs + LTDC_AWCR, AWCR_AAW | AWCR_AAH, val);

		/* Total width & height */
		val = (total_w << 16) | total_h;
		clrsetbits_le32(regs + LTDC_TWCR, TWCR_TOTALH | TWCR_TOTALW, val);

		setbits_le32(regs + LTDC_LIPCR, acc_act_h + 1);
	}

	/* Signal polarities */
	val = 0;
	log_debug("timing->flags 0x%08x\n", timings->flags);
	if (timings->flags & DISPLAY_FLAGS_HSYNC_HIGH)
		val |= GCR_HSPOL;
	if (timings->flags & DISPLAY_FLAGS_VSYNC_HIGH)
		val |= GCR_VSPOL;
	if (timings->flags & DISPLAY_FLAGS_DE_LOW)
		val |= GCR_DEPOL;
	if (timings->flags & DISPLAY_FLAGS_PIXDATA_NEGEDGE)
		val |= GCR_PCPOL;

	if (rotation == 90 || rotation == 270)
		val |= GCR_ROTEN;

	clrsetbits_le32(regs + LTDC_GCR,
			GCR_HSPOL | GCR_VSPOL | GCR_DEPOL | GCR_PCPOL, val);

	/* Overall background color */
	writel(priv->bg_col_argb, priv->regs + LTDC_BCCR);
}

static void stm32_ltdc_set_layer1(struct udevice *dev, ulong fb_addr)
{
	struct stm32_ltdc_priv *priv = dev_get_priv(dev);
	void __iomem *regs = priv->regs;
	u32 x0, x1, y0, y1;
	u32 pitch_in_bytes;
	u32 line_length;
	u32 bus_width;
	u32 val, avbp, ahbp, bpp;
	u32 format;
	u32 rotation;

	x0 = priv->crop_x;
	x1 = priv->crop_x + priv->crop_w - 1;
	y0 = priv->crop_y;
	y1 = priv->crop_y + priv->crop_h - 1;

	if (ofnode_device_is_compatible(dev_ofnode(dev), "st,stm32mp25-ltdc"))
		rotation = dev_read_u32_default(priv->panel, "rotation", 0);

	/* check that an output rotation is required */
	if (rotation == 90 || rotation == 270) {
		/* Horizontal start and stop position */
		ahbp = (readl(regs + LTDC_BPCR) & BPCR_AVBP);
		val = ((x1 + 1 + ahbp) << 16) + (x0 + 1 + ahbp);
		clrsetbits_le32(regs + LTDC_L1WHPCR, LXWHPCR_WHSTPOS | LXWHPCR_WHSPPOS, val);

		/* Vertical start & stop position */
		avbp = (readl(regs + LTDC_BPCR) & BPCR_AHBP) >> 16;
		val = ((y1 + 1 + avbp) << 16) + (y0 + 1 + avbp);
		clrsetbits_le32(regs + LTDC_L1WVPCR, LXWVPCR_WVSTPOS | LXWVPCR_WVSPPOS,	val);
	} else {
		/* Horizontal start and stop position */
		ahbp = (readl(regs + LTDC_BPCR) & BPCR_AHBP) >> 16;
		val = ((x1 + 1 + ahbp) << 16) + (x0 + 1 + ahbp);
		clrsetbits_le32(regs + LTDC_L1WHPCR, LXWHPCR_WHSTPOS | LXWHPCR_WHSPPOS, val);

		/* Vertical start & stop position */
		avbp = readl(regs + LTDC_BPCR) & BPCR_AVBP;
		val = ((y1 + 1 + avbp) << 16) + (y0 + 1 + avbp);
		clrsetbits_le32(regs + LTDC_L1WVPCR, LXWVPCR_WVSTPOS | LXWVPCR_WVSPPOS,	val);
	}

	/* Layer background color */
	writel(priv->bg_col_argb, regs + LTDC_L1DCCR);

	/* Color frame buffer pitch in bytes & line length */
	bpp = VNBITS(priv->l2bpp);
	pitch_in_bytes = priv->crop_w * (bpp >> 3);
	bus_width = 8 << ((readl(regs + LTDC_GC2R) & GC2R_BW) >> 4);
	line_length = ((bpp >> 3) * priv->crop_w) + (bus_width >> 3) - 1;
	if (rotation == 270 || rotation == 180)
		/* Compute negative value (signed on 16 bits) for the picth */
		val = ((0x10000 - pitch_in_bytes) << 16) | line_length;
	else
		val = (pitch_in_bytes << 16) | line_length;
	clrsetbits_le32(regs + LTDC_L1CFBLR, LXCFBLR_CFBLL | LXCFBLR_CFBP, val);

	/* Pixel format */
	format = stm32_ltdc_get_pixel_format(priv->l2bpp);
	for (val = 0; val < NB_PF; val++)
		if (priv->pix_fmt_hw[val] == format)
			break;

	if (val >= NB_PF) {
		log_err("invalid pixel format\n");
		return;
	}

	clrsetbits_le32(regs + LTDC_L1PFCR, LXPFCR_PF, val);

	/* Constant alpha value */
	clrsetbits_le32(regs + LTDC_L1CACR, LXCACR_CONSTA, priv->alpha);

	/* Specifies the blending factors : with or without pixel alpha */
	/* Manage hw-specific capabilities */
	val = has_alpha(format) ? BF1_PAXCA | BF2_1PAXCA : BF1_CA | BF2_1CA;

	/* Blending factors */
	clrsetbits_le32(regs + LTDC_L1BFCR, LXBFCR_BF2 | LXBFCR_BF1, val);

	/* Frame buffer line number */
	clrsetbits_le32(regs + LTDC_L1CFBLNR, LXCFBLNR_CFBLN, priv->crop_h);

	/* Frame buffer address */
	switch (rotation) {
	case 270:
		writel(fb_addr + (pitch_in_bytes * (y1 - y0 + 1) - 1), regs + LTDC_L1CFBAR);
		break;
	case 180:
		writel(fb_addr + (pitch_in_bytes * (y1 - y0 + 1) - 1) +
		       (bpp >> 3) * (x1 - x0 + 1) - 1, regs + LTDC_L1CFBAR);
		break;
	case 90:
		writel(fb_addr + (bpp >> 3) * (x1 - x0 + 1) - 1, regs + LTDC_L1CFBAR);
		break;
	default:
		writel(fb_addr, regs + LTDC_L1CFBAR);
	}

	/* Enable layer 1 & set mirroring */
	if (rotation == 90 || rotation == 180)
		setbits_le32(priv->regs + LTDC_L1CR, LXCR_LEN | LXCR_HMEN);
	else
		setbits_le32(priv->regs + LTDC_L1CR, LXCR_LEN);
}

static int stm32_ltdc_get_panel(struct udevice *dev, struct udevice **panel)
{
	ofnode ep_node, node, ports, remote;
	u32 phandle;
	int ret = 0;

	if (!dev)
		return -EINVAL;

	ports = ofnode_find_subnode(dev_ofnode(dev), "ports");
	if (!ofnode_valid(ports)) {
		dev_err(dev, "Remote bridge subnode\n");
		return ret;
	}

	for (node = ofnode_first_subnode(ports);
	     ofnode_valid(node);
	     node = dev_read_next_subnode(node)) {
		ep_node = ofnode_first_subnode(node);
		if (!ofnode_valid(ep_node))
			continue;

		ret = ofnode_read_u32(ep_node, "remote-endpoint", &phandle);
		if (ret) {
			dev_err(dev, "%s(%s): Could not find remote-endpoint property\n",
				__func__, dev_read_name(dev));
			return ret;
		}

		remote = ofnode_get_by_phandle(phandle);
		if (!ofnode_valid(remote))
			return -EINVAL;

		while (ofnode_valid(remote)) {
			remote = ofnode_get_parent(remote);
			if (!ofnode_valid(remote)) {
				dev_dbg(dev, "%s(%s): no UCLASS_DISPLAY for remote-endpoint\n",
					__func__, dev_read_name(dev));
				continue;
			}

			uclass_find_device_by_ofnode(UCLASS_PANEL, remote, panel);
			if (*panel)
				if (ofnode_valid(dev_ofnode(*panel)))
					return 0;
		};
	}

	/* Sanity check, we can get out of the loop without having a clean ofnode */
	if (!(*panel))
		ret = -EINVAL;

	return ret;
}

static int stm32_ltdc_display_init(struct udevice *dev, ofnode *ep_node,
				   struct udevice **panel, struct udevice **bridge)
{
	ofnode remote;
	u32 phandle;
	int ret;

	if (*panel)
		return -EINVAL;

	if (IS_ENABLED(CONFIG_VIDEO_BRIDGE)) {
		ret = ofnode_read_u32(*ep_node, "remote-endpoint", &phandle);
		if (ret) {
			dev_dbg(dev, "%s(%s): Could not find remote-endpoint property\n",
				__func__, dev_read_name(dev));
			return ret;
		}

		remote = ofnode_get_by_phandle(phandle);
		if (!ofnode_valid(remote))
			return -EINVAL;

		while (ofnode_valid(remote)) {
			remote = ofnode_get_parent(remote);
			if (!ofnode_valid(remote)) {
				dev_dbg(dev, "%s(%s): no UCLASS_VIDEO_BRIDGE for remote-endpoint\n",
					__func__, dev_read_name(dev));
				return -EINVAL;
			}

			uclass_find_device_by_ofnode(UCLASS_VIDEO_BRIDGE, remote, bridge);
			if (*bridge && !ret) {
				ret = uclass_get_device_by_ofnode(UCLASS_VIDEO_BRIDGE,
								  remote, bridge);
				if (ret)
					dev_dbg(dev,
						"No video bridge, or no backlight on bridge\n");
				break;
			}
		}

		ret = stm32_ltdc_get_panel(*bridge, panel);
	} else {
		/* no bridge , search a panel from display controller node */
		ret = ofnode_read_u32(*ep_node, "remote-endpoint", &phandle);
		if (ret) {
			dev_dbg(dev, "%s(%s): Could not find remote-endpoint property\n",
				__func__, dev_read_name(dev));
			return ret;
		}

		remote = ofnode_get_by_phandle(phandle);
		if (!ofnode_valid(remote))
			return -EINVAL;

		while (ofnode_valid(remote)) {
			remote = ofnode_get_parent(remote);
			if (!ofnode_valid(remote)) {
				dev_dbg(dev, "%s(%s): no UCLASS_VIDEO_BRIDGE for remote-endpoint\n",
					__func__, dev_read_name(dev));
				return -EINVAL;
			}

			ret = uclass_find_device_by_ofnode(UCLASS_PANEL, remote, panel);
			if (*panel && !ret) {
				ret = uclass_get_device_by_ofnode(UCLASS_PANEL, remote, panel);
				break;
			}
		}
	}

	return ret;
}

static int stm32_ltdc_probe(struct udevice *dev)
{
	struct video_uc_plat *uc_plat = dev_get_uclass_plat(dev);
	struct video_priv *uc_priv = dev_get_uclass_priv(dev);
	struct stm32_ltdc_priv *priv = dev_get_priv(dev);
	struct display_timing timings;
	struct clk pclk, bclk;
	struct reset_ctl rst;
	struct regmap *regmap = NULL;
	struct udevice *syscon;
	ofnode node, port;
	ulong rate;
	int ret, idx;

	if (IS_ENABLED(CONFIG_STM32MP25X) || IS_ENABLED(CONFIG_STM32MP23X) ||
	    IS_ENABLED(CONFIG_STM32MP21X)) {
		struct ofnode_phandle_args args;

		node = dev_ofnode(dev);

		idx = ofnode_stringlist_search(node, "access-controller-names", "cmn");
		if (idx < 0)
			return idx;

		ret = ofnode_parse_phandle_with_args(node, "access-controllers",
						     "#access-controller-cells",
						     0, idx, &args);
		if (ret < 0) {
			dev_err(dev, "Can not get access-controllers to common registers\n");
			return ret;
		}

		ret = stm32_check_access_by_id(dev_ofnode(dev), args.args[0]);
		if (ret < 0) {
			dev_err(dev, "Fail to get access to common registers\n");
			return ret;
		}

		node = dev_read_subnode(dev, "l1l2");

		idx = ofnode_stringlist_search(node, "access-controller-names", "l1l2");
		if (idx < 0)
			return idx;

		ret = ofnode_parse_phandle_with_args(node, "access-controllers",
						     "#access-controller-cells",
						     0, idx, &args);
		if (ret < 0) {
			dev_err(dev, "Can not get access-controllers to l1l2 registers\n");
			return ret;
		}

		ret = stm32_check_access_by_id(dev_ofnode(dev), args.args[0]);
		if (ret < 0) {
			dev_err(dev, "Fail to get access to l1l2 registers\n");
			return ret;
		}
	}

	if (IS_ENABLED(CONFIG_SYSCON) &&
	    (IS_ENABLED(CONFIG_STM32MP25X) || IS_ENABLED(CONFIG_STM32MP23X))) {
		ret = uclass_get_device_by_phandle(UCLASS_SYSCON, dev, "st,syscon", &syscon);
		if (ret) {
			if (ret != -ENOENT) {
				dev_err(dev, "unable to find syscon device\n");
				return ret;
			}
		} else {
			regmap = syscon_get_regmap(syscon);
			if (IS_ERR(regmap)) {
				dev_err(dev, "Fail to get Syscon regmap\n");
				return PTR_ERR(regmap);
			}

			/* Set default pixel clock to enable register access */
			regmap_write(regmap, SYSCFG_DISPLAYCLKCR, DISPLAYCLKCR_DPI);
		}
	}

	priv->regs = dev_read_addr_ptr(dev);
	if (!priv->regs) {
		dev_err(dev, "ltdc dt register address error\n");
		return -EINVAL;
	}

	ret = clk_get_by_name(dev, "bus", &bclk);
	if (ret) {
		if (ret != -ENODATA) {
			dev_err(dev, "bus clock get error %d\n", ret);
			return ret;
		}
	} else {
		ret = clk_enable(&bclk);
		if (ret) {
			dev_err(dev, "bus clock enable error %d\n", ret);
			return ret;
		}
	}

	ret = clk_get_by_name(dev, "lcd", &pclk);
	if (ret) {
		dev_err(dev, "peripheral clock get error %d\n", ret);
		return ret;
	}

	ret = clk_enable(&pclk);
	if (ret) {
		dev_err(dev, "peripheral clock enable error %d\n", ret);
		return ret;
	}

	priv->hw_version = readl(priv->regs + LTDC_IDR);
	dev_dbg(dev, "%s: LTDC hardware 0x%x\n", __func__, priv->hw_version);

	switch (priv->hw_version) {
	case HWVER_10200:
	case HWVER_10300:
		priv->layer_regs = layer_regs_a0;
		priv->pix_fmt_hw = pix_fmt_a0;
		break;
	case HWVER_20101:
		priv->layer_regs = layer_regs_a1;
		priv->pix_fmt_hw = pix_fmt_a1;
		break;
	case HWVER_40100:
	case HWVER_40101:
		priv->layer_regs = layer_regs_a2;
		priv->pix_fmt_hw = pix_fmt_a2;
		break;
	default:
		return -ENODEV;
	}

	/*
	 * Try all the ports until one working.
	 *
	 * This means that it will search first for the DSI node
	 * and then for the LVDS.
	 * This is done in two times. First is checks for the
	 * UCLASS_VIDEO_BRIDGE available, and then for this bridge
	 * it scans for a UCLASS_PANEL.
	 */

	port = dev_read_subnode(dev, "port");
	if (!ofnode_valid(port)) {
		dev_err(dev, "%s(%s): 'port' subnode not found\n",
			__func__, dev_read_name(dev));
		return -EINVAL;
	}

	for (node = ofnode_first_subnode(port);
	     ofnode_valid(node);
	     node = dev_read_next_subnode(node)) {
		ret = stm32_ltdc_display_init(dev, &node, &priv->panel, &priv->bridge);
		if (ret)
			dev_dbg(dev, "Device failed ret=%d\n", ret);
		else
			break;
	}

	/* Sanity check */
	if (ret)
		return ret;

	ret = panel_get_display_timing(priv->panel, &timings);
	if (ret) {
		ret = ofnode_decode_display_timing(dev_ofnode(priv->panel),
						   0, &timings);
		if (ret) {
			dev_err(dev, "decode display timing error %d\n", ret);
			return ret;
		}
	}

	rate = clk_set_rate(&pclk, timings.pixelclock.typ);
	if (IS_ERR_VALUE(rate))
		dev_warn(dev, "fail to set pixel clock %d hz, ret=%ld\n",
			 timings.pixelclock.typ, rate);

	dev_dbg(dev, "Set pixel clock req %d hz get %ld hz\n",
		timings.pixelclock.typ, rate);

	ret = reset_get_by_index(dev, 0, &rst);
	if (ret) {
		dev_err(dev, "missing ltdc hardware reset\n");
		return ret;
	}

	/* Reset */
	reset_deassert(&rst);

	if (IS_ENABLED(CONFIG_VIDEO_BRIDGE)) {
		if (priv->bridge) {
			/* Set the pixel clock according to the encoder */
			if (IS_ENABLED(CONFIG_SYSCON) &&
			    (IS_ENABLED(CONFIG_STM32MP25X) || IS_ENABLED(CONFIG_STM32MP23X))) {
				if (!strcmp(priv->bridge->name, "stm32-display-dsi"))
					regmap_write(regmap, SYSCFG_DISPLAYCLKCR,
						     DISPLAYCLKCR_DPI);
				else if (!strncmp(priv->bridge->name, "lvds", 4))
					regmap_write(regmap, SYSCFG_DISPLAYCLKCR,
						     DISPLAYCLKCR_LVDS);
			}

			ret = video_bridge_attach(priv->bridge);
			if (ret) {
				dev_dbg(priv->bridge, "fail to attach bridge\n");
				return ret;
			}

			/* set state the pinctrl to sleep to avoid noise */
			pinctrl_select_state(dev, "sleep");
		}
	}

	/* TODO Below parameters are hard-coded for the moment... */
	priv->l2bpp = VIDEO_BPP16;
	priv->bg_col_argb = 0xFFFFFFFF; /* white no transparency */
	priv->crop_x = 0;
	priv->crop_y = 0;
	priv->crop_w = timings.hactive.typ;
	priv->crop_h = timings.vactive.typ;
	priv->alpha = 0xFF;

	dev_dbg(dev, "%dx%d %dbpp frame buffer at 0x%lx\n",
		timings.hactive.typ, timings.vactive.typ,
		VNBITS(priv->l2bpp), uc_plat->base);
	dev_dbg(dev, "crop %d,%d %dx%d bg 0x%08x alpha %d\n",
		priv->crop_x, priv->crop_y, priv->crop_w, priv->crop_h,
		priv->bg_col_argb, priv->alpha);

	/* Configure & start LTDC */
	stm32_ltdc_set_mode(dev, &timings);
	stm32_ltdc_set_layer1(dev, uc_plat->base);
	stm32_ltdc_enable(priv);

	uc_priv->xsize = timings.hactive.typ;
	uc_priv->ysize = timings.vactive.typ;
	uc_priv->bpix = priv->l2bpp;

	if (!priv->bridge) {
		ret = panel_enable_backlight(priv->panel);
		if (ret) {
			dev_err(dev, "panel %s enable backlight error %d\n",
				priv->panel->name, ret);
			return ret;
		}
	} else if (IS_ENABLED(CONFIG_VIDEO_BRIDGE)) {
		ret = video_bridge_set_backlight(priv->bridge, 80);
		if (ret) {
			dev_err(dev, "fail to set backlight\n");
			return ret;
		}
	}

	video_set_flush_dcache(dev, true);

	return 0;
}

static int stm32_ltdc_bind(struct udevice *dev)
{
	struct video_uc_plat *uc_plat = dev_get_uclass_plat(dev);

	uc_plat->size = CONFIG_VIDEO_STM32_MAX_XRES *
			CONFIG_VIDEO_STM32_MAX_YRES *
			(CONFIG_VIDEO_STM32_MAX_BPP >> 3);
	/* align framebuffer on kernel MMU_SECTION_SIZE = max 2MB for LPAE */
	uc_plat->align = SZ_2M;
	dev_dbg(dev, "frame buffer max size %d bytes align %x\n",
		uc_plat->size, uc_plat->align);

	return 0;
}

static const struct udevice_id stm32_ltdc_ids[] = {
	{ .compatible = "st,stm32-ltdc" },
	{ .compatible = "st,stm32mp21-ltdc" },
	{ .compatible = "st,stm32mp25-ltdc" },
	{ }
};

U_BOOT_DRIVER(stm32_ltdc) = {
	.name			= "stm32_display",
	.id			= UCLASS_VIDEO,
	.of_match		= stm32_ltdc_ids,
	.probe			= stm32_ltdc_probe,
	.bind			= stm32_ltdc_bind,
	.priv_auto	= sizeof(struct stm32_ltdc_priv),
};
