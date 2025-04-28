// SPDX-License-Identifier: GPL-2.0-or-later OR BSD-3-Clause
/*
 * Copyright (C) 2021, STMicroelectronics - All Rights Reserved
 */

#define LOG_CATEGORY UCLASS_USB_TYPEC

#include <dm.h>
#include <errno.h>
#include <typec.h>
#include <dm/device_compat.h>
#include <dm/device-internal.h>
#include <dm/uclass-internal.h>

int typec_get_device_from_usb(struct udevice *dev, struct udevice **typec, u8 index)
{
	ofnode node, child;
	u32 endpoint_phandle;
	u32 reg;
	int ret;

	/* 'port' nodes can be grouped under an optional 'ports' node */
	node = dev_read_subnode(dev, "ports");
	if (!ofnode_valid(node)) {
		node = dev_read_subnode(dev, "port");
	} else {
		/* several 'port' nodes, found the requested port@index one */
		ofnode_for_each_subnode(child, node) {
			ofnode_read_u32(child, "reg", &reg);
			if (index == reg) {
				node = child;
				break;
			}
		}
		node = child;
	}

	if (!ofnode_valid(node)) {
		dev_dbg(dev, "connector port or port@%d subnode not found\n", index);
		return -ENODEV;
	}

	/* get endpoint node */
	node = ofnode_first_subnode(node);
	if (!ofnode_valid(node))
		return -EINVAL;

	ret = ofnode_read_u32(node, "remote-endpoint", &endpoint_phandle);
	if (ret)
		return ret;

	/* retrieve connector endpoint phandle */
	node = ofnode_get_by_phandle(endpoint_phandle);
	if (!ofnode_valid(node))
		return -EINVAL;
	/*
	 * Use a while to retrieve an USB Type-C device either at connector
	 * level or just above (depending if UCSI uclass is used or not)
	 */
	while (ofnode_valid(node)) {
		node = ofnode_get_parent(node);
		if (!ofnode_valid(node)) {
			dev_err(dev, "No UCLASS_USB_TYPEC for remote-endpoint\n");
			return -EINVAL;
		}

		uclass_find_device_by_ofnode(UCLASS_USB_TYPEC, node, typec);
		if (*typec)
			break;
	}

	ret = device_probe(*typec);
	if (ret) {
		dev_err(dev, "Type-C won't probe (ret=%d)\n", ret);
		return ret;
	}

	return 0;
}

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
