// SPDX-License-Identifier: GPL-2.0-only OR BSD-3-Clause
/*
 * Copyright (C) 2022, STMicroelectronics - All Rights Reserved
 */

#define LOG_CATEGORY UCLASS_PHY

#include <asm/io.h>
#include <common.h>
#include <clk.h>
#include <clk-uclass.h>
#include <div64.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <dm/lists.h>
#include <dm/of_access.h>
#include <fdtdec.h>
#include <generic-phy.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/usb/otg.h>
#include <log.h>
#include <power/regulator.h>
#include <regmap.h>
#include <reset.h>
#include <syscon.h>
#include <usb.h>

#define PHY1CR_OFFSET		0x2400
#define PHY1TRIM1_OFFSET	0x240C
#define PHY1TRIM2_OFFSET	0x2410
#define PHY2CR_OFFSET		0x2800
#define PHY2TRIM1_OFFSET	0x2808
#define PHY2TRIM2_OFFSET	0x280C

#define PHYCR_REG      1

/* Retention mode enable (active low) */
#define SYSCFG_USB2PHY2CR_RETENABLEN2_MASK		BIT(0)
/*
 * Auto-resume mode enable. Enables auto-resume logic in USB2PHY so that the PHY automatically
 * responds to a remote wake-up without initial involvement of the Host controller.
 */
#define SYSCFG_USB2PHY2CR_AUTORSMENB2_MASK		BIT(1)
/* Controls the power down of analog blocks during Suspend and Sleep. */
#define SYSCFG_USB2PHY2CR_USB2PHY2CMN_MASK		BIT(2)
/* Controls vbus valid input of USB3 DRD controller when in Host mode */
#define SYSCFG_USB2PHY2CR_VBUSVALID_MASK		BIT(4)
/* Selects VBUS valid comparator that is used when USB3 DRD controller is in Device mode */
#define SYSCFG_USB2PHY2CR_VBUSVLDEXTSEL_MASK		BIT(5)
/* Voltage comparison result when an external voltage comparator is used */
#define SYSCFG_USB2PHY2CR_VBUSVLDEXT_MASK		BIT(6)
/*
 * 0: internal debounce logic is enabled (default).
 * Bit0: applies to utmiotg_vbusvalid,
 * Bit1: applies to pipe3_PowerPresent,
 * Bit2: applies to utmisrp_bvalid,
 * Bit3: applies to utmiotg_iddig]. (default)
 */
#define SYSCFG_USB2PHY2CR_FILTER_BYPASS_MASK		GENMASK(10, 7)
/* Voltage comparison result when an external voltage comparator is used */
#define SYSCFG_USB2PHY2CR_OTGDISABLE0_MASK		BIT(16)
/* Voltage comparison result when an external voltage comparator is used */
#define SYSCFG_USB2PHY2CR_DRVVBUS0_MASK			BIT(17)

#define SYSCFG_USB2PHYTRIM1_PLLITUNE_MASK		GENMASK(1, 0)
#define SYSCFG_USB2PHYTRIM1_PLLPTUNE_MASK		GENMASK(5, 2)
#define SYSCFG_USB2PHYTRIM1_COMPDISTUNE_MASK		GENMASK(8, 6)
#define SYSCFG_USB2PHYTRIM1_SQRXTUNE_MASK		GENMASK(11, 9)
#define SYSCFG_USB2PHYTRIM1_VDATREFTUNE_MASK		GENMASK(13, 12)
#define SYSCFG_USB2PHYTRIM1_OTGTUNE_MASK		GENMASK(16, 14)
#define SYSCFG_USB2PHYTRIM1_TXHSXVTUNE_MASK		GENMASK(18, 17)
#define SYSCFG_USB2PHYTRIM1_TXFSLSTUNE_MASK		GENMASK(22, 19)
#define SYSCFG_USB2PHYTRIM1_TXVREFTUNE_MASK		GENMASK(26, 23)
#define SYSCFG_USB2PHYTRIM1_TXRISETUNE_MASK		GENMASK(28, 27)
#define SYSCFG_USB2PHYTRIM1_TXRESTUNE_MASK		GENMASK(30, 29)

#define SYSCFG_USB2PHYTRIM2_TXPREEMPAMPTUNE_MASK	GENMASK(1, 0)
#define SYSCFG_USB2PHYTRIM2_TXPREEMPPULSETUNE_MASK	BIT(2)

struct stm32_usb2phy {
	struct regmap *regmap;
	struct clk clk;
	struct reset_ctl reset;
	struct udevice *vdd33;
	struct udevice *vdda18;
	struct udevice *vbus;
	uint init;
	const struct stm32mp2_usb2phy_hw_data *hw_data;
};

enum stm32_usb2phy_mode {
	USB2_MODE_HOST_ONLY = 0x1,
	USB2_MODE_DRD = 0x3,
	USB2_MODE_OTG,
};

struct stm32mp2_usb2phy_hw_data {
	u32 phyrefsel_mask, phyrefsel_bitpos, cr_offset, trim1_offset, trim2_offset;
	enum stm32_usb2phy_mode valid_mode;
};

static const struct stm32mp2_usb2phy_hw_data stm32mp21_usb2phy_hwdata[] = {
	{
		.cr_offset = PHY1CR_OFFSET,
		.trim1_offset = PHY1TRIM1_OFFSET,
		.trim2_offset = PHY1TRIM2_OFFSET,
		.valid_mode = USB2_MODE_HOST_ONLY,
		.phyrefsel_mask = 0x7,
		.phyrefsel_bitpos = 4,
	},
	{
		.cr_offset = PHY2CR_OFFSET,
		.trim1_offset = PHY2TRIM1_OFFSET,
		.trim2_offset = PHY2TRIM2_OFFSET,
		.valid_mode = USB2_MODE_OTG,
		.phyrefsel_mask = 0x7,
		.phyrefsel_bitpos = 4,
	},
	{ }
};

static const struct stm32mp2_usb2phy_hw_data stm32mp25_usb2phy_hwdata[] = {
	{
		.cr_offset = PHY1CR_OFFSET,
		.trim1_offset = PHY1TRIM1_OFFSET,
		.trim2_offset = PHY1TRIM2_OFFSET,
		.valid_mode = USB2_MODE_HOST_ONLY,
		.phyrefsel_mask = 0x7,
		.phyrefsel_bitpos = 4,
	},
	{
		.cr_offset = PHY2CR_OFFSET,
		.trim1_offset = PHY2TRIM1_OFFSET,
		.trim2_offset = PHY2TRIM2_OFFSET,
		.valid_mode = USB2_MODE_DRD,
		.phyrefsel_mask = 0x7,
		.phyrefsel_bitpos = 12,
	},
	{ }
};

/*
 * Two phy instances are found in mp25, and some bitfields are a bit different (shift...)
 * depending on the instance. So identify the instance by using CR offset to report
 * the correct bitfields & modes to use
 */
static const struct stm32mp2_usb2phy_hw_data *stm32_usb2phy_get_hwdata(struct udevice *dev,
								       unsigned long offset)
{
	int i;
	struct stm32mp2_usb2phy_hw_data *hwdata;

	hwdata = (struct stm32mp2_usb2phy_hw_data *)dev_get_driver_data(dev);
	if (!hwdata)
		return NULL;
	for (i = 0; (hwdata[i].cr_offset != offset) && hwdata[i].cr_offset; i++)
		;
	if (hwdata[i].cr_offset)
		return &hwdata[i];

	return NULL;
}

static int stm32_usb2phy_getrefsel(unsigned long rate)
{
	switch (rate) {
	case 19200000:
		return 0;
	case 20000000:
		return 1;
	case 24000000:
		return 2;
	default:
		return -EINVAL;
	}
}

static int stm32_usb2phy_regulators_enable(struct phy *phy)
{
	int ret;
	struct stm32_usb2phy *phy_dev = dev_get_priv(phy->dev);

	ret = regulator_set_enable_if_allowed(phy_dev->vdd33, true);
	if (ret)
		return ret;

	if (phy_dev->vdda18) {
		ret = regulator_set_enable_if_allowed(phy_dev->vdda18, true);
		if (ret)
			goto vdd33_disable;
	}

	return 0;

vdd33_disable:
	regulator_set_enable(phy_dev->vdd33, false);

	return ret;
}

static int stm32_usb2phy_regulators_disable(struct phy *phy)
{
	int ret;
	struct stm32_usb2phy *phy_dev = dev_get_priv(phy->dev);

	if (phy_dev->vdda18) {
		ret = regulator_set_enable_if_allowed(phy_dev->vdda18, false);
		if (ret)
			return ret;
	}

	ret = regulator_set_enable_if_allowed(phy_dev->vdd33, false);
	if (ret)
		return ret;

	return 0;
}

static int stm32_usb2phy_init(struct phy *phy)
{
	int ret;
	struct stm32_usb2phy *phy_dev = dev_get_priv(phy->dev);
	struct udevice *dev = phy->dev;
	unsigned long phyref_rate;
	u32 phyrefsel;
	const struct stm32mp2_usb2phy_hw_data *phy_data = phy_dev->hw_data;

	if (phy_dev->init) {
		phy_dev->init++;
		return 0;
	}

	ret = stm32_usb2phy_regulators_enable(phy);
	if (ret) {
		dev_err(dev, "can't enable regulators (%d)\n", ret);
		return ret;
	}

	ret = clk_enable(&phy_dev->clk);
	if (ret) {
		dev_err(dev, "could not enable clock: %d\n", ret);
		goto error_regl_dis;
	}

	phyref_rate = clk_get_rate(&phy_dev->clk);

	ret = stm32_usb2phy_getrefsel(phyref_rate);
	if (ret < 0) {
		dev_err(dev, "invalid phyref clk rate\n");
		goto error_clk_dis;
	}
	phyrefsel = (u32)ret;
	dev_dbg(dev, "phyrefsel value (%d)\n", phyrefsel);

	ret = regmap_update_bits(phy_dev->regmap,
				 phy_data->cr_offset,
				 phy_data->phyrefsel_mask << phy_data->phyrefsel_bitpos,
				 phyrefsel << phy_data->phyrefsel_bitpos);
	if (ret) {
		dev_err(dev, "can't set phyrefclksel (%d)\n", ret);
		goto error_clk_dis;
	}

	ret = reset_deassert(&phy_dev->reset);
	if (ret) {
		dev_err(dev, "can't release reset (%d)\n", ret);
		goto error_clk_dis;
	}

	phy_dev->init++;

	return 0;

error_clk_dis:
	clk_disable(&phy_dev->clk);
error_regl_dis:
	stm32_usb2phy_regulators_disable(phy);

	return ret;
}

static int stm32_usb2phy_exit(struct phy *phy)
{
	struct stm32_usb2phy *phy_dev = dev_get_priv(phy->dev);
	int ret;

        if (!phy_dev->init) {
		dev_err(phy->dev, "Invalid ref-count for phy\n");
		return -EINVAL;
	}

	phy_dev->init--;

	if (phy_dev->init)
		return 0;

	ret = clk_disable(&phy_dev->clk);
	if (ret)
		return ret;

	ret = stm32_usb2phy_regulators_disable(phy);
	if (ret) {
		dev_err(phy->dev, "can't disable regulators (%d)\n", ret);
		return ret;
	}

	return reset_assert(&phy_dev->reset);
}

static int stm32_usb2phy_phy_power_on(struct phy *phy)
{
	struct stm32_usb2phy *phy_dev = dev_get_priv(phy->dev);
	int ret;

	if (phy_dev->vbus) {
		ret = regulator_set_enable_if_allowed(phy_dev->vbus, true);
		if (ret)
			return ret;
	}

	return 0;
}

static int stm32_usb2phy_phy_power_off(struct phy *phy)
{
	struct stm32_usb2phy *phy_dev = dev_get_priv(phy->dev);
	int ret;

	if (phy_dev->vbus) {
		ret = regulator_set_enable_if_allowed(phy_dev->vbus, false);
		if (ret)
			return ret;
	}

	return 0;
}

static int stm32_usb2phy_get_regulator(ofnode node,
				       char *supply_name,
				       struct udevice **regulator)
{
	struct ofnode_phandle_args regulator_phandle;
	int ret;

	ret = ofnode_parse_phandle_with_args(node, supply_name,
					     NULL, 0, 0, &regulator_phandle);
	if (ret)
		return ret;

	ret = uclass_get_device_by_ofnode(UCLASS_REGULATOR,
					  regulator_phandle.node, regulator);
	if (ret)
		return ret;

	return 0;
}

static int stm32_usb2phy_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	int ret;
	struct stm32_usb2phy *phy_dev = dev_get_priv(phy->dev);
	const struct stm32mp2_usb2phy_hw_data *phy_data = phy_dev->hw_data;
	struct udevice *dev = phy->dev;

	switch (mode) {
	case PHY_MODE_USB_HOST:
		if (phy_data->valid_mode == USB2_MODE_HOST_ONLY ||
		    phy_data->valid_mode == USB2_MODE_OTG)
			/*
			 * CMN bit cleared since OHCI-ctrl registers are inaccessible
			 * when clocks (clk12+clk48) are turned off in Suspend which
			 * makes it impossible to resume
			 */
			ret = regmap_update_bits(phy_dev->regmap,
						 phy_data->cr_offset,
						 SYSCFG_USB2PHY2CR_USB2PHY2CMN_MASK,
						 0);
		else {
			/*
			 * CMN bit cleared since when running in usb3speed with dwc3-usb
			 * xHCI ctrl is (most likely) suspending the (unused) usb2phy2
			 * and when the clocks (utmi_clk) input to usb3dr-ctrl from usb2phy2
			 * are turned off, there is some internal error inside the usb3dr-ctrl
			 * while running in usb3-speed
			 */
			if (submode == USB_ROLE_NONE) {
				ret = regmap_update_bits(phy_dev->regmap,
							 phy_data->cr_offset,
							 SYSCFG_USB2PHY2CR_USB2PHY2CMN_MASK |
							 SYSCFG_USB2PHY2CR_VBUSVLDEXT_MASK |
							 SYSCFG_USB2PHY2CR_VBUSVALID_MASK |
							 SYSCFG_USB2PHY2CR_VBUSVLDEXT_MASK, 0);
			} else {
				ret = regmap_update_bits(phy_dev->regmap,
							 phy_data->cr_offset,
							 SYSCFG_USB2PHY2CR_USB2PHY2CMN_MASK |
							 SYSCFG_USB2PHY2CR_VBUSVLDEXT_MASK |
							 SYSCFG_USB2PHY2CR_VBUSVALID_MASK,
							 SYSCFG_USB2PHY2CR_VBUSVALID_MASK);
			}
		}
		if (ret) {
			dev_err(dev, "can't set usb2phycr (%d)\n", ret);
			return ret;
		}
		break;

	case PHY_MODE_USB_DEVICE:
		/*
		 * USB DWC3 gadget driver (in uboot) first sets the RUN bit, and
		 * later it performs the device-mode conf init.
		 * Hence USB2PHY2CMN bit of PHY needs to be cleared, since incase
		 * VBUS is not present then usb-ctrl puts PHY in suspend and inturn
		 * PHY turns off clocks to ctrl which makes the device-mode init fail
		 */
		if (phy_data->valid_mode == USB2_MODE_OTG) {
			ret = regmap_update_bits(phy_dev->regmap,
						 phy_data->cr_offset,
						 SYSCFG_USB2PHY2CR_USB2PHY2CMN_MASK,
						 0);
		} else {
			if (submode == USB_ROLE_NONE) {
				ret = regmap_update_bits(phy_dev->regmap,
							 phy_data->cr_offset,
							 SYSCFG_USB2PHY2CR_USB2PHY2CMN_MASK |
							 SYSCFG_USB2PHY2CR_VBUSVALID_MASK |
							 SYSCFG_USB2PHY2CR_VBUSVLDEXTSEL_MASK |
							 SYSCFG_USB2PHY2CR_VBUSVLDEXT_MASK,
							 SYSCFG_USB2PHY2CR_VBUSVLDEXTSEL_MASK);
			} else {
				ret = regmap_update_bits(phy_dev->regmap,
							 phy_data->cr_offset,
							 SYSCFG_USB2PHY2CR_USB2PHY2CMN_MASK |
							 SYSCFG_USB2PHY2CR_VBUSVALID_MASK |
							 SYSCFG_USB2PHY2CR_VBUSVLDEXTSEL_MASK |
							 SYSCFG_USB2PHY2CR_VBUSVLDEXT_MASK,
							 SYSCFG_USB2PHY2CR_VBUSVLDEXTSEL_MASK |
							 SYSCFG_USB2PHY2CR_VBUSVLDEXT_MASK);
			}
		}
		if (ret) {
			dev_err(dev, "can't set usb2phycr (%d)\n", ret);
			return ret;
		}
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int stm32_usb2phy_tuning(struct udevice *dev, ofnode node)
{
	struct stm32_usb2phy *phy_dev = dev_get_priv(dev);
	u32 mask_trim1 = 0, value_trim1 = 0, mask_trim2 = 0, value_trim2 = 0, val;
	int ret;

	ret = ofnode_read_u32(node, "st,pll-ipath-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && val <= SYSCFG_USB2PHYTRIM1_PLLITUNE_MASK) {
			mask_trim1 |= SYSCFG_USB2PHYTRIM1_PLLITUNE_MASK;
			value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_PLLITUNE_MASK, val);
		} else {
			dev_err(dev, "Error getting pll-ipath-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = ofnode_read_u32(node, "st,pll-ppath-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && val <= SYSCFG_USB2PHYTRIM1_PLLPTUNE_MASK) {
			mask_trim1 |= SYSCFG_USB2PHYTRIM1_PLLPTUNE_MASK;
			value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_PLLPTUNE_MASK, val);
		} else {
			dev_err(dev, "Error getting pll-ppath-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = ofnode_read_u32(node, "st,comp-dis-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM1_COMPDISTUNE_MASK, val)) {
			mask_trim1 |= SYSCFG_USB2PHYTRIM1_COMPDISTUNE_MASK;
			value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_COMPDISTUNE_MASK, val);
		} else {
			dev_err(dev, "Error getting comp-dis-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = ofnode_read_u32(node, "st,sqrx-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM1_SQRXTUNE_MASK, val)) {
			mask_trim1 |= SYSCFG_USB2PHYTRIM1_SQRXTUNE_MASK;
			value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_SQRXTUNE_MASK, val);
		} else {
			dev_err(dev, "Error getting sqrx-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = ofnode_read_u32(node, "st,vdatref-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM1_VDATREFTUNE_MASK, val)) {
			mask_trim1 |= SYSCFG_USB2PHYTRIM1_VDATREFTUNE_MASK;
			value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_VDATREFTUNE_MASK, val);
		} else {
			dev_err(dev, "Error getting vdatref-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = ofnode_read_u32(node, "st,otg-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM1_OTGTUNE_MASK, val)) {
			mask_trim1 |= SYSCFG_USB2PHYTRIM1_OTGTUNE_MASK;
			value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_OTGTUNE_MASK, val);
		} else {
			dev_err(dev, "Error getting otg-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = ofnode_read_u32(node, "st,txhsxv-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM1_TXHSXVTUNE_MASK, val)) {
			mask_trim1 |= SYSCFG_USB2PHYTRIM1_TXHSXVTUNE_MASK;
			value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_TXHSXVTUNE_MASK, val);
		} else {
			dev_err(dev, "Error getting txhsxv-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = ofnode_read_u32(node, "st,txfsls-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM1_TXFSLSTUNE_MASK, val)) {
			mask_trim1 |= SYSCFG_USB2PHYTRIM1_TXFSLSTUNE_MASK;
			value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_TXFSLSTUNE_MASK, val);
		} else {
			dev_err(dev, "Error getting txfsls-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = ofnode_read_u32(node, "st,txvref-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM1_TXVREFTUNE_MASK, val)) {
			mask_trim1 |= SYSCFG_USB2PHYTRIM1_TXVREFTUNE_MASK;
			value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_TXVREFTUNE_MASK, val);
		} else {
			dev_err(dev, "Error getting txvref-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = ofnode_read_u32(node, "st,txrise-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM1_TXRISETUNE_MASK, val)) {
			mask_trim1 |= SYSCFG_USB2PHYTRIM1_TXRISETUNE_MASK;
			value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_TXRISETUNE_MASK, val);
		} else {
			dev_err(dev, "Error getting txrise-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = ofnode_read_u32(node, "st,txres-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM1_TXRESTUNE_MASK, val)) {
			mask_trim1 |= SYSCFG_USB2PHYTRIM1_TXRESTUNE_MASK;
			value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_TXRESTUNE_MASK, val);
		} else {
			dev_err(dev, "Error getting txres-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = ofnode_read_u32(node, "st,txpreempamp-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM2_TXPREEMPAMPTUNE_MASK, val)) {
			mask_trim2 |= SYSCFG_USB2PHYTRIM2_TXPREEMPAMPTUNE_MASK;
			value_trim2 |= FIELD_PREP(SYSCFG_USB2PHYTRIM2_TXPREEMPAMPTUNE_MASK, val);
		} else {
			dev_err(dev, "Error getting txpreempamp-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = ofnode_read_u32(node, "st,txpreemppulse-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM2_TXPREEMPPULSETUNE_MASK, val)) {
			mask_trim2 |= SYSCFG_USB2PHYTRIM2_TXPREEMPPULSETUNE_MASK;
			value_trim2 |= FIELD_PREP(SYSCFG_USB2PHYTRIM2_TXPREEMPPULSETUNE_MASK, val);
		} else {
			dev_err(dev, "Error getting txpreemppulse-tune property (%d)\n", ret);
			return ret;
		}
	}

	if (mask_trim1) {
		ret = regmap_update_bits(phy_dev->regmap,
					 phy_dev->hw_data->trim1_offset,
					 mask_trim1,
					 value_trim1);
		if (ret) {
			dev_err(dev, "can't set usb2phytrim1 (%d)\n", ret);
			return ret;
		}
		dev_dbg(dev, "usb2phytrim1 mask = %x value = %x\n", mask_trim1, value_trim1);
	}

	if (mask_trim2) {
		ret = regmap_update_bits(phy_dev->regmap,
					 phy_dev->hw_data->trim2_offset,
					 mask_trim2,
					 value_trim2);
		if (ret) {
			dev_err(dev, "can't set usb2phytrim2 (%d)\n", ret);
			return ret;
		}
		dev_dbg(dev, "usb2phytrim2 mask = %x value = %x\n", mask_trim2, value_trim2);
	}

	return 0;
}

static const struct phy_ops stm32_usb2phy_ops = {
	.init = stm32_usb2phy_init,
	.exit = stm32_usb2phy_exit,
	.power_on = stm32_usb2phy_phy_power_on,
	.power_off = stm32_usb2phy_phy_power_off,
	.set_mode = stm32_usb2phy_set_mode,
};

static int stm32_usb2phy_probe(struct udevice *dev)
{
	struct stm32_usb2phy *phy_dev = dev_get_priv(dev);
	ofnode node = dev_ofnode(dev), connector;
	int ret;
	u32 phycr;

	phy_dev->regmap = syscon_regmap_lookup_by_phandle(dev, "st,syscfg");
	if (IS_ERR(phy_dev->regmap)) {
		dev_err(dev, "unable to find regmap\n");
		return PTR_ERR(phy_dev->regmap);
	}

	ret = dev_read_u32_index(dev, "st,syscfg", PHYCR_REG, &phycr);
	if (ret) {
		dev_err(dev, "Can't get sysconfig phycr offset (%d)\n", ret);
		return ret;
	}
	dev_dbg(dev, "phycr offset 0x%x\n", phycr);

	ret = clk_get_by_index(dev, 0, &phy_dev->clk);
	if (ret)
		return ret;

	ret = reset_get_by_index(dev, 0, &phy_dev->reset);
	if (ret)
		return ret;

	ret = device_get_supply_regulator(dev, "vdd33-supply", &phy_dev->vdd33);
	if (ret) {
		dev_err(dev, "Can't get vdd33-supply regulator\n");
		return ret;
	}

	/* Vdda18 regulator is optional */
	ret = device_get_supply_regulator(dev, "vdda18-supply", &phy_dev->vdda18);
	if (ret) {
		if (ret != -ENOENT)
			return ret;
		dev_dbg(dev, "Can't get vdda18-supply regulator\n");
	}

	phy_dev->hw_data = stm32_usb2phy_get_hwdata(dev, phycr);
	if (!phy_dev->hw_data) {
		dev_err(dev, "can't get matching stm32mp2_usb2_of_data\n");
		return -EINVAL;
	}

	/* Configure phy tuning */
	ret = stm32_usb2phy_tuning(dev, node);
	if (ret) {
		dev_err(dev, "can't set tuning parameters: %d\n", ret);
		return ret;
	}

	connector = ofnode_find_subnode(node, "connector");
	if (ofnode_valid(connector)) {
		ret = stm32_usb2phy_get_regulator(connector, "vbus-supply",
						  &phy_dev->vbus);
		if (ret) {
			if (ret != -ENOENT) {
				dev_err(dev, "Can't get vbus regulator\n");
				return ret;
			}
			phy_dev->vbus = NULL;
		}
	} else {
		phy_dev->vbus = NULL;
	}

	return 0;
}

static const struct udevice_id stm32_usb2phy_of_match[] = {
	{ .compatible = "st,stm32mp25-usb2phy", .data = (ulong)stm32mp25_usb2phy_hwdata },
	{ .compatible = "st,stm32mp21-usb2phy", .data = (ulong)stm32mp21_usb2phy_hwdata },
	{ },
};

U_BOOT_DRIVER(stm32_usb2phy) = {
	.name = "stm32-usb2phy",
	.id = UCLASS_PHY,
	.of_match = stm32_usb2phy_of_match,
	.ops = &stm32_usb2phy_ops,
	.probe = stm32_usb2phy_probe,
	.priv_auto	= sizeof(struct stm32_usb2phy),
};
