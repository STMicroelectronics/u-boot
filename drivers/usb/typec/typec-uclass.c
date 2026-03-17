// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021, STMicroelectronics - All Rights Reserved
 */

#define LOG_CATEGORY UCLASS_USB_TYPEC

#include <dm.h>
#include <errno.h>
#include <typec.h>
#include <dm/device_compat.h>

int typec_get_data_role(struct udevice *dev, u8 con_idx)
{
	const struct typec_ops *ops = device_get_ops(dev);
	int ret;

	if (!ops->get_data_role)
		return -ENOSYS;

	ret = ops->get_data_role(dev, con_idx);
	dev_dbg(dev, "%s\n", ret == TYPEC_HOST ? "Host" : "Device");

	return ret;
}

int typec_is_attached(struct udevice *dev, u8 con_idx)
{
	const struct typec_ops *ops = device_get_ops(dev);
	int ret;

	if (!ops->is_attached)
		return -ENOSYS;

	ret = ops->is_attached(dev, con_idx);
	dev_dbg(dev, "%s\n", ret == TYPEC_ATTACHED ? "Attached" : "Not attached");

	return ret;
}

int typec_get_nb_connector(struct udevice *dev)
{
	const struct typec_ops *ops = device_get_ops(dev);
	int ret;

	if (!ops->get_nb_connector)
		return -ENOSYS;

	ret = ops->get_nb_connector(dev);
	dev_dbg(dev, "%d connector(s)\n", ret);

	return ret;
}

UCLASS_DRIVER(typec) = {
	.id	= UCLASS_USB_TYPEC,
	.name	= "typec",
};
