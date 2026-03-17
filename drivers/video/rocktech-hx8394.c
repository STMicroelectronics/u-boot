// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2022 STMicroelectronics - All Rights Reserved
 * Author: Yannick Fertre <yannick.fertre@foss.st.com> for STMicroelectronics.
 *
 * This hx8394 panel driver is inspired from the Linux Kernel driver
 * drivers/gpu/drm/panel/panel-rocktech-hx8394.c.
 */

#include <backlight.h>
#include <dm.h>
#include <mipi_dsi.h>
#include <panel.h>
#include <asm/gpio.h>
#include <dm/device_compat.h>
#include <linux/delay.h>
#include <power/regulator.h>

#define MCS_SETPOWER	0xB1
#define MCS_SETDISP	0xB2
#define MCS_SETCYC	0xB4
#define MCS_SETVCOM	0xB6
#define MCS_SETEXTC	0xB9
#define MCS_SETMIPI	0xBA
#define MCS_SET_BANK	0xBD
#define MCS_NO_DOC1	0xBF
#define MCS_NO_DOC2	0xC0
#define MCS_NO_DOC3	0xC6
#define MCS_NO_DOC4	0xD8
#define MCS_NO_DOC5	0xD4
#define MCS_SETPANEL	0xCC
#define MCS_SETGIP_0	0xD3
#define MCS_SETGIP_1	0xD5
#define MCS_SETGIP_2	0xD6

#define MCS_SETGAMMA	0xE0
#define MCS_READ_ID1	0xDA
#define MCS_READ_ID2	0xDB
#define MCS_READ_ID3	0xDC

#define MY	BIT(7)	/* Row Address Order */
#define MX	BIT(6)	/* Column Address Order */
#define MV	BIT(5)	/* Row/Column Exchange */
#define ML	BIT(4)	/* Vertical Refresh Order */
#define RGB	BIT(3)	/* RGB-BGR Order */
#define DDL	BIT(2)	/* Display Data Latch Order */
#define FH	BIT(1)	/* Flip Horizontal */
#define FV	BIT(0)	/* Flip Vertical */

struct hx8394_panel_priv {
	struct udevice *reg;
	struct udevice *backlight;
	struct gpio_desc reset;
};

static const struct display_timing default_timing = {
	.pixelclock.typ		= 54000000,
	.hactive.typ		= 720,
	.hfront_porch.typ	= 48,
	.hback_porch.typ	= 48,
	.hsync_len.typ		= 9,
	.vactive.typ		= 1280,
	.vfront_porch.typ	= 12,
	.vback_porch.typ	= 12,
	.vsync_len.typ		= 5,
};

static void hx8394_dcs_write_buf(struct udevice *dev, const void *data,
				 size_t len)
{
	struct mipi_dsi_panel_plat *plat = dev_get_plat(dev);
	struct mipi_dsi_device *device = plat->device;
	int err;

	err = mipi_dsi_dcs_write_buffer(device, data, len);
	if (err < 0)
		dev_err(dev, "MIPI DSI DCS write buffer failed: %d\n", err);
}

#define dcs_write_seq(dev, seq...)				\
({								\
	static const u8 d[] = { seq };				\
								\
	hx8394_dcs_write_buf(dev, d, ARRAY_SIZE(d));	\
})

#define dcs_write_cmd_seq(dev, cmd, seq...)				\
({									\
	static const u8 d[] = { seq };					\
	struct mipi_dsi_panel_plat *plat = dev_get_plat(dev);		\
	struct mipi_dsi_device *device = plat->device;			\
	int err;							\
	err = mipi_dsi_dcs_write(device, cmd, d, ARRAY_SIZE(d));	\
	if (err < 0)							\
		dev_err(dev, "MIPI DSI DCS write failed: %d\n", err);	\
})

static void hx8394_init_sequence(struct udevice *dev)
{
	dcs_write_cmd_seq(dev, MCS_SETEXTC, 0xFF, 0x83, 0x94);
	dcs_write_cmd_seq(dev, MCS_SETMIPI, 0x61, 0x03, 0x68, 0x6B, 0xB2, 0xC0);
	dcs_write_seq(dev, MCS_SETPOWER, 0x48, 0x12, 0x72, 0x09, 0x32, 0x54, 0x71, 0x71, 0x57,
		      0x47);
	dcs_write_cmd_seq(dev, MCS_SETDISP, 0x00, 0x80, 0x64, 0x0C, 0x0D, 0x2F);
	dcs_write_seq(dev, MCS_SETCYC, 0x73, 0x74, 0x73, 0x74, 0x73, 0x74, 0x01, 0x0C, 0x86, 0x75,
		      0x00, 0x3F, 0x73, 0x74, 0x73, 0x74, 0x73, 0x74, 0x01, 0x0C, 0x86);
	dcs_write_seq(dev, MCS_SETGIP_0, 0x00, 0x00, 0x07, 0x07, 0x40, 0x07, 0x0C, 0x00, 0x08, 0x10,
		      0x08, 0x00, 0x08, 0x54, 0x15, 0x0A, 0x05, 0x0A, 0x02, 0x15, 0x06, 0x05, 0x06,
		      0x47, 0x44, 0x0A, 0x0A, 0x4B, 0x10, 0x07, 0x07, 0x0C, 0x40);
	dcs_write_seq(dev, MCS_SETGIP_1, 0x1C, 0x1C, 0x1D, 0x1D, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
		      0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x24, 0x25, 0x18, 0x18, 0x26, 0x27, 0x18,
		      0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
		      0x18, 0x18, 0x20, 0x21, 0x18, 0x18, 0x18, 0x18);
	dcs_write_seq(dev, MCS_SETGIP_2, 0x1C, 0x1C, 0x1D, 0x1D, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02,
		      0x01, 0x00, 0x0B, 0x0A, 0x09, 0x08, 0x21, 0x20, 0x18, 0x18, 0x27, 0x26, 0x18,
		      0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
		      0x18, 0x18, 0x25, 0x24, 0x18, 0x18, 0x18, 0x18);
	dcs_write_cmd_seq(dev, MCS_SETVCOM, 0x92, 0x92);
	dcs_write_seq(dev, MCS_SETGAMMA, 0x00, 0x0A, 0x15, 0x1B, 0x1E, 0x21, 0x24, 0x22, 0x47, 0x56,
		      0x65, 0x66, 0x6E, 0x82, 0x88, 0x8B, 0x9A, 0x9D, 0x98, 0xA8, 0xB9, 0x5D, 0x5C,
		      0x61, 0x66, 0x6A, 0x6F, 0x7F, 0x7F, 0x00, 0x0A, 0x15, 0x1B, 0x1E, 0x21, 0x24,
		      0x22, 0x47, 0x56, 0x65, 0x65, 0x6E, 0x81, 0x87, 0x8B, 0x98, 0x9D, 0x99, 0xA8,
		      0xBA, 0x5D, 0x5D, 0x62, 0x67, 0x6B, 0x72, 0x7F,  0x7F);
	dcs_write_cmd_seq(dev, MCS_NO_DOC2, 0x1F, 0x31);
	dcs_write_cmd_seq(dev, MCS_SETPANEL, 0x03);
	dcs_write_cmd_seq(dev, MCS_NO_DOC5, 0x02);
	dcs_write_cmd_seq(dev, MCS_SET_BANK, 0x02);
	dcs_write_seq(dev, MCS_NO_DOC4, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		      0xFF, 0xFF);
	dcs_write_cmd_seq(dev, MCS_SET_BANK, 0x00);
	dcs_write_cmd_seq(dev, MCS_SET_BANK, 0x01);
	dcs_write_cmd_seq(dev, MCS_SETPOWER, 0x00);
	dcs_write_cmd_seq(dev, MCS_SET_BANK, 0x00);
	dcs_write_cmd_seq(dev, MCS_NO_DOC1, 0x40, 0x81, 0x50, 0x00, 0x1A, 0xFC, 0x01);
	dcs_write_cmd_seq(dev, MCS_NO_DOC3, 0xED);
	dcs_write_cmd_seq(dev, MIPI_DCS_SET_ADDRESS_MODE, FH);
}

static int hx8394_panel_enable_backlight(struct udevice *dev)
{
	struct mipi_dsi_panel_plat *plat = dev_get_plat(dev);
	struct mipi_dsi_device *device = plat->device;
	struct hx8394_panel_priv *priv = dev_get_priv(dev);
	int ret;

	ret = mipi_dsi_attach(device);
	if (ret < 0)
		return ret;

	hx8394_init_sequence(dev);

	ret = mipi_dsi_dcs_exit_sleep_mode(device);
	if (ret)
		return ret;

	mdelay(120);

	ret = mipi_dsi_dcs_set_display_on(device);
	if (ret)
		return ret;

	mdelay(50);

	ret = backlight_enable(priv->backlight);

	return ret;
}

static int hx8394_panel_get_display_timing(struct udevice *dev,
					   struct display_timing *timings)
{
	memcpy(timings, &default_timing, sizeof(*timings));

	return 0;
}

static int hx8394_panel_of_to_plat(struct udevice *dev)
{
	struct hx8394_panel_priv *priv = dev_get_priv(dev);
	int ret;

	ret =  device_get_supply_regulator(dev, "power-supply",
					   &priv->reg);
	if (ret) {
		dev_err(dev, "Warning: cannot get power supply\n");
		return ret;
	}

	ret = gpio_request_by_name(dev, "reset-gpios", 0, &priv->reset,
				   GPIOD_IS_OUT);
	if (ret) {
		dev_err(dev, "Warning: cannot get reset GPIO\n");
		if (ret != -ENOENT)
			return ret;
	}

	ret = uclass_get_device_by_phandle(UCLASS_PANEL_BACKLIGHT, dev,
					   "backlight", &priv->backlight);
	if (ret)
		dev_err(dev, "Cannot get backlight: ret=%d\n", ret);

	return ret;
}

static int hx8394_panel_probe(struct udevice *dev)
{
	struct hx8394_panel_priv *priv = dev_get_priv(dev);
	struct mipi_dsi_panel_plat *plat = dev_get_plat(dev);
	int ret;

	ret = regulator_set_enable(priv->reg, true);
	if (ret)
		return ret;

	/* reset panel */
	dm_gpio_set_value(&priv->reset, true);
	mdelay(1);
	dm_gpio_set_value(&priv->reset, false);
	mdelay(50);

	/* fill characteristics of DSI data link */
	plat->lanes = 2;
	plat->format = MIPI_DSI_FMT_RGB888;
	plat->mode_flags = MIPI_DSI_MODE_VIDEO |
			   MIPI_DSI_MODE_VIDEO_BURST |
			   MIPI_DSI_MODE_LPM;

	return 0;
}

static const struct panel_ops hx8394_panel_ops = {
	.enable_backlight = hx8394_panel_enable_backlight,
	.get_display_timing = hx8394_panel_get_display_timing,
};

static const struct udevice_id hx8394_panel_ids[] = {
	{ .compatible = "rocktech,hx8394" },
	{ }
};

U_BOOT_DRIVER(hx8394_panel) = {
	.name			  = "hx8394_panel",
	.id			  = UCLASS_PANEL,
	.of_match		  = hx8394_panel_ids,
	.ops			  = &hx8394_panel_ops,
	.of_to_plat	  = hx8394_panel_of_to_plat,
	.probe			  = hx8394_panel_probe,
	.plat_auto	= sizeof(struct mipi_dsi_panel_plat),
	.priv_auto	= sizeof(struct hx8394_panel_priv),
};
