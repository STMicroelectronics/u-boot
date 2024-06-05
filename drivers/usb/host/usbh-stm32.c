// SPDX-License-Identifier: GPL-2.0-only
/*
 * STM32 USB2 EHCI/OHCI (USBH) controller glue driver
 *
 * Copyright (C) 2023 STMicroelectronics – All Rights Reserved
 *
 * Author: Pankaj Dev <pankaj.dev@st.com>
 */

#define LOG_CATEGORY UCLASS_USB

#include <dm/device.h>
#include <dm/device_compat.h>
#include <dm/devres.h>
#include <dm/ofnode.h>
#include <linux/bitfield.h>
#include <regmap.h>
#include <reset.h>
#include <syscon.h>

#define SYSCFG_USBHCR_OVRCUR_POLARITY_MASK      BIT(0)
#define SYSCFG_USBHCR_VBUSEN_POLARITY_MASK      BIT(1)

/**
 * struct stm32_usbh_data - usbh-stm32 driver private structure
 * @dev:		device pointer
 * @regmap:		regmap pointer for getting syscfg
 * @syscfg_usbhcr_reg_off:	usbhcr syscfg control offset
 * @vbusen_polarity_low:	vbusen signal polarity
 * @ovrcur_polarity_low:	ovrcur signal polarity
 */
struct stm32_usbh_data {
	struct udevice *dev;
	struct regmap *regmap;
	int syscfg_usbhcr_reg_off;
	bool vbusen_polarity_low;
	bool ovrcur_polarity_low;
};

/**
 * stm32_usbh_init: init the controller via glue logic
 * @usbh_data: driver private structure
 */
static int stm32_usbh_init(struct stm32_usbh_data *usbh_data)
{
	return regmap_update_bits(usbh_data->regmap, usbh_data->syscfg_usbhcr_reg_off,
				  SYSCFG_USBHCR_OVRCUR_POLARITY_MASK |
				  SYSCFG_USBHCR_VBUSEN_POLARITY_MASK,
				  FIELD_PREP(SYSCFG_USBHCR_OVRCUR_POLARITY_MASK,
					     usbh_data->ovrcur_polarity_low) |
				  FIELD_PREP(SYSCFG_USBHCR_VBUSEN_POLARITY_MASK,
					     usbh_data->vbusen_polarity_low));
}

static int stm32_usbh_probe(struct udevice *dev)
{
	ofnode node = dev_ofnode(dev);
	struct regmap *regmap;
	struct stm32_usbh_data *usbh_data = dev_get_plat(dev);
	int ret;

	regmap = syscon_regmap_lookup_by_phandle(dev, "st,syscfg");
	if (IS_ERR(regmap)) {
		dev_err(dev, "no st,syscfg node found(%ld)\n", PTR_ERR(regmap));
		ret = PTR_ERR(regmap);
		return ret;
	}

	ret = ofnode_read_u32_index(node, "st,syscfg", 1, &usbh_data->syscfg_usbhcr_reg_off);
	if (ret) {
		dev_err(dev, "can't get usbhcr offset(%d)\n", ret);
		return ret;
	}

	dev_vdbg(dev, "syscfg-usbhcr-reg offset 0x%x\n", usbh_data->syscfg_usbhcr_reg_off);

	usbh_data->dev = dev;
	usbh_data->regmap = regmap;

	if (ofnode_read_bool(node, "st,vbusen-active-low"))
		usbh_data->vbusen_polarity_low = true;
	if (ofnode_read_bool(node, "st,ovrcur-active-low"))
		usbh_data->ovrcur_polarity_low = true;

	/* ST USBH glue logic init */
	ret = stm32_usbh_init(usbh_data);
	if (ret) {
		dev_err(dev, "err setting syscfg_usbhcr_reg(%d)\n", ret);
		return ret;
	}

	return 0;
}

static const struct udevice_id stm32_usbh_ids[] = {
	{ .compatible = "st,stm32mp25-usbh" },
	{ .compatible = "st,stm32mp21-usbh" },
	{ /* sentinel */ },
};

U_BOOT_DRIVER(stm32_usbh_glue) = {
	.name = "stm32-usbh-glue",
	.id = UCLASS_NOP,
	.of_match = stm32_usbh_ids,
	.bind = dm_scan_fdt_dev,
	.probe = stm32_usbh_probe,
	.plat_auto = sizeof(struct stm32_usbh_data),
};
