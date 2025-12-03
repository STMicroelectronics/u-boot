// SPDX-License-Identifier: GPL-2.0-or-later OR BSD-3-Clause
/*
 * Copyright (C) 2021, STMicroelectronics - All Rights Reserved
 */

#define LOG_CATEGORY UCLASS_USB_TYPEC

#include <common.h>
#include <dm.h>
#include <i2c.h>
#include <typec.h>
#include <dm/device_compat.h>
#include <linux/delay.h>
#include <power/regulator.h>

#define STUSB160X_ALERT_STATUS			0x0B /* RC */
#define STUSB160X_CC_CONNECTION			BIT(6)

#define STUSB160X_CC_CONNECTION_STATUS_TRANS	0x0D /* RC */
#define STUSB160X_CC_ATTACH_TRANS		BIT(0)

#define STUSB160X_CC_CONNECTION_STATUS		0x0E /* RO */
#define STUSB160X_CC_ATTACH			BIT(0)
#define STUSB160X_CC_DATA_ROLE			BIT(2)

#define STUSB160X_CC_POWER_MODE_CTRL		0x28 /* RW */
#define STUSB160X_DUAL_WITH_ACCESSORY		3

struct stusb160x_priv {
	enum typec_state attached;
	enum typec_data_role data_role;
};

static int stusb160x_get_status(struct udevice *dev, bool force)
{
	struct stusb160x_priv *priv = dev_get_priv(dev);
	int alert, trans, status;

	alert = dm_i2c_reg_read(dev, STUSB160X_ALERT_STATUS);
	if (alert < 0)
		return alert;

	/* If no update, exit */
	if ((!(alert & STUSB160X_CC_CONNECTION)) && !force)
		goto exit;

	trans = dm_i2c_reg_read(dev, STUSB160X_CC_CONNECTION_STATUS_TRANS);
	if (trans < 0)
		return trans;

	status = dm_i2c_reg_read(dev, STUSB160X_CC_CONNECTION_STATUS);
	if (status < 0)
		return status;

	priv->data_role = status & STUSB160X_CC_DATA_ROLE ? TYPEC_HOST : TYPEC_DEVICE;
	priv->attached = status & STUSB160X_CC_ATTACH ? TYPEC_ATTACHED : TYPEC_UNATTACHED;
exit:
	dev_dbg(dev, "status: %s data role: %s\n",
		priv->attached == TYPEC_ATTACHED ? "Attached" : "Unattached",
		priv->data_role == TYPEC_HOST ? "Host" : "Device");

	return 0;
}

static int stusb160x_get_data_role(struct udevice *dev, u8 con_idx)
{
	struct stusb160x_priv *priv = dev_get_priv(dev);
	int ret;

	ret = stusb160x_get_status(dev, false);
	if (ret < 0)
		return ret;

	return priv->data_role;
}

static int stusb160x_is_attached(struct udevice *dev, u8 con_idx)
{
	struct stusb160x_priv *priv = dev_get_priv(dev);
	int ret;

	ret = stusb160x_get_status(dev, false);
	if (ret < 0)
		return ret;

	return priv->attached;
}

static u8 stusb160x_get_nb_connector(struct udevice *dev)
{
	/* only one connector supported */
	return 1;
}

static int stusb160x_probe(struct udevice *dev)
{
	struct udevice *vsys_supply, *vdd_supply, *vconn_supply;
	int power_mode_ctrl;
	int ret;

	/* Get "vsys", "vdd" and "vconn" optional regulators, enable them. */
	ret = device_get_supply_regulator(dev, "vsys-supply", &vsys_supply);
	if (ret && ret != -ENOENT)
		return ret;

	if (vsys_supply) {
		ret = regulator_set_enable_if_allowed(vsys_supply, true);
		if (ret) {
			dev_err(dev, "Error enabling vsys supply (ret=%d)\n", ret);
			return ret;
		}
	}

	ret = device_get_supply_regulator(dev, "vdd-supply", &vdd_supply);
	if (ret && ret != -ENOENT)
		return ret;

	if (vdd_supply) {
		ret = regulator_set_enable_if_allowed(vdd_supply, true);
		if (ret) {
			dev_err(dev, "Error enabling vdd supply (ret=%d)\n", ret);
			return ret;
		}
	}

	ret = device_get_supply_regulator(dev, "vconn-supply", &vconn_supply);
	if (ret && ret != -ENOENT)
		return ret;

	if (vconn_supply) {
		ret = regulator_set_enable_if_allowed(vconn_supply, true);
		if (ret) {
			dev_err(dev, "Error enabling vconn supply (ret=%d)\n", ret);
			return ret;
		}
	}

	/* Tload: I2C registers loading time from NVM */
	mdelay(30);

	/* configure STUSB160X_CC_POWER_MODE_CTRL */
	power_mode_ctrl = dm_i2c_reg_read(dev, STUSB160X_CC_POWER_MODE_CTRL);
	if (power_mode_ctrl < 0)
		return power_mode_ctrl;

	power_mode_ctrl |= STUSB160X_DUAL_WITH_ACCESSORY;
	ret = dm_i2c_reg_write(dev, STUSB160X_CC_POWER_MODE_CTRL, power_mode_ctrl);
	if (ret < 0)
		return ret;

	/* get current status : attached/unattached, device/host */
	return stusb160x_get_status(dev, true);
}

static const struct typec_ops stusb160x_typec_ops = {
	.is_attached = stusb160x_is_attached,
	.get_data_role = stusb160x_get_data_role,
	.get_nb_connector = stusb160x_get_nb_connector,
};

static const struct udevice_id typec_of_match[] = {
	{ .compatible = "st,stusb1600"},
	{}
};

U_BOOT_DRIVER(typec_stusb160x) = {
	.id			= UCLASS_USB_TYPEC,
	.name			= "typec_stusb160x",
	.of_match		= typec_of_match,
	.ops			= &stusb160x_typec_ops,
	.priv_auto		= sizeof(struct stusb160x_priv),
	.probe			= stusb160x_probe,
};
