// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021, STMicroelectronics - All Rights Reserved
 *
 * Code inspired from kernel drivers/usb/typec/ucsi/ucsi.c
 *
 */

#define LOG_CATEGORY UCLASS_UCSI

#include <dm.h>
#include <errno.h>
#include <typec.h>
#include <ucsi.h>
#include <dm/device.h>
#include <dm/device_compat.h>
#include <linux/delay.h>

/*
 * UCSI_TIMEOUT_US - PPM communication timeout
 *
 * Ideally we could use MIN_TIME_TO_RESPOND_WITH_BUSY (which is defined in UCSI
 * specification) here as reference, but unfortunately we can't. It is very
 * difficult to estimate the time it takes for the system to process the command
 * before it is actually passed to the PPM.
 */
#define UCSI_TIMEOUT_US		50000000

struct connector {
	enum typec_state attached;
	enum typec_data_role data_role;
};

struct ucsi_priv {
	struct connector *con;
	u8 nb_connector;
};

static int ucsi_read(struct udevice *dev, int offset, void *buf, int size)
{
	const struct ucsi_ops *ops = device_get_ops(dev);

	if (!ops->read)
		return -ENOSYS;

	return ops->read(dev, offset, buf, size);
}

static int ucsi_write(struct udevice *dev, int offset, void *buf, int size)
{
	const struct ucsi_ops *ops = device_get_ops(dev);

	if (!ops->write)
		return -ENOSYS;

	return ops->write(dev, offset, buf, size);
}

static int ucsi_acknowledge_command(struct udevice *dev)
{
	u64 ctrl;

	ctrl = UCSI_ACK_CC_CI;
	ctrl |= UCSI_ACK_COMMAND_COMPLETE;

	return ucsi_write(dev, UCSI_CONTROL, &ctrl, sizeof(ctrl));
}

static int ucsi_acknowledge_connector_change(struct udevice *dev)
{
	u64 ctrl;

	ctrl = UCSI_ACK_CC_CI;
	ctrl |= UCSI_ACK_CONNECTOR_CHANGE;

	return ucsi_write(dev, UCSI_CONTROL, &ctrl, sizeof(ctrl));
}

static int ucsi_exec_command(struct udevice *dev, u64 command);

static int ucsi_read_error(struct udevice *dev)
{
	u16 error;
	int ret;

	/* Acknowlege the command that failed */
	ret = ucsi_acknowledge_command(dev);

	if (ret)
		return ret;

	ret = ucsi_exec_command(dev, UCSI_GET_ERROR_STATUS);

	if (ret < 0)
		return ret;

	ret = ucsi_read(dev, UCSI_MESSAGE_IN, &error, sizeof(error));
	if (ret)
		return ret;

	switch (error) {
	case UCSI_ERROR_INCOMPATIBLE_PARTNER:
		return -EOPNOTSUPP;
	case UCSI_ERROR_CC_COMMUNICATION_ERR:
		return -ECOMM;
	case UCSI_ERROR_CONTRACT_NEGOTIATION_FAIL:
		return -EPROTO;
	case UCSI_ERROR_DEAD_BATTERY:
		dev_warn(dev, "Dead battery condition!\n");
		return -EPERM;
	case UCSI_ERROR_INVALID_CON_NUM:
	case UCSI_ERROR_UNREGONIZED_CMD:
	case UCSI_ERROR_INVALID_CMD_ARGUMENT:
		dev_err(dev, "possible UCSI driver bug %u\n", error);
		return -EINVAL;
	case UCSI_ERROR_OVERCURRENT:
		dev_warn(dev, "Overcurrent condition\n");
		break;
	case UCSI_ERROR_PARTNER_REJECTED_SWAP:
		dev_warn(dev, "Partner rejected swap\n");
		break;
	case UCSI_ERROR_HARD_RESET:
		dev_warn(dev, "Hard reset occurred\n");
		break;
	case UCSI_ERROR_PPM_POLICY_CONFLICT:
		dev_warn(dev, "PPM Policy conflict\n");
		break;
	case UCSI_ERROR_SWAP_REJECTED:
		dev_warn(dev, "Swap rejected\n");
		break;
	case UCSI_ERROR_UNDEFINED:
	default:
		dev_err(dev, "unknown error %u\n", error);
		break;
	}

	return -EIO;
}

static int ucsi_exec_command(struct udevice *dev, u64 cmd)
{
	u32 cci;
	int ret;

	ret = ucsi_write(dev, UCSI_CONTROL, &cmd, sizeof(cmd));
	if (ret)
		return ret;

	ret = ucsi_read(dev, UCSI_CCI, &cci, sizeof(cci));
	if (ret)
		return ret;

	if (cci & UCSI_CCI_BUSY)
		return -EBUSY;

	if (!(cci & UCSI_CCI_COMMAND_COMPLETE))
		return -EIO;

	if (cci & UCSI_CCI_NOT_SUPPORTED)
		return -EOPNOTSUPP;

	if (cci & UCSI_CCI_ERROR) {
		if (cmd == UCSI_GET_ERROR_STATUS)
			return -EIO;
		return ucsi_read_error(dev);
	}

	return UCSI_CCI_LENGTH(cci);
}

static int ucsi_send_command(struct udevice *dev, u64 command,
			     void *data, size_t size)
{
	u8 length;
	int ret;

	ret = ucsi_exec_command(dev, command);
	if (ret < 0)
		goto out;

	length = ret;

	if (data) {
		ret = ucsi_read(dev, UCSI_MESSAGE_IN, data, size);
		if (ret)
			goto out;
	}

	ret = ucsi_acknowledge_command(dev);
	if (ret)
		goto out;

	ret = length;
out:

	return ret;
}

static int ucsi_reset_ppm(struct udevice *dev)
{
	u64 command = UCSI_PPM_RESET;
	unsigned long tmo;
	u32 cci;
	int ret;

	ret = ucsi_write(dev, UCSI_CONTROL, &command, sizeof(command));
	if (ret < 0)
		goto out;

	tmo = timer_get_us() + UCSI_TIMEOUT_US;

	do {
		if (time_before(tmo, timer_get_us())) {
			ret = -ETIMEDOUT;
			goto out;
		}

		ret = ucsi_read(dev, UCSI_CCI, &cci, sizeof(cci));
		if (ret)
			goto out;

		/* If the PPM is still doing something else, reset it again. */
		if (cci & ~UCSI_CCI_RESET_COMPLETE) {
			ret = ucsi_write(dev, UCSI_CONTROL, &command,
					 sizeof(command));
			if (ret < 0)
				goto out;
		}

		mdelay(20);
	} while (!(cci & UCSI_CCI_RESET_COMPLETE));

out:
	return ret;
}

static int ucsi_get_status(struct udevice *child, u8 con_idx, bool force)
{
	struct udevice *parent = dev_get_parent(child);
	struct ucsi_priv *priv = dev_get_priv(child);
	struct ucsi_connector_status status;
	u64 command;
	u32 cci;
	int ret = 0;

	if (con_idx > (priv->nb_connector - 1))
		return -EINVAL;

	ret = ucsi_read(parent, UCSI_CCI, &cci, sizeof(cci));
	if (ret)
		return ret;

	/* is there any change ? */
	if (!UCSI_CCI_CONNECTOR(cci) && !force)
		goto exit;

	command = UCSI_GET_CONNECTOR_STATUS | UCSI_CONNECTOR_NUMBER(con_idx + 1);
	ret = ucsi_send_command(parent, command, &status, sizeof(status));
	if (ret < 0)
		return ret;

	priv->con[con_idx].attached = status.flags & UCSI_CONSTAT_CONNECTED ?
		TYPEC_ATTACHED : TYPEC_UNATTACHED;

	switch (UCSI_CONSTAT_PARTNER_TYPE(status.flags)) {
	case UCSI_CONSTAT_PARTNER_TYPE_UFP:
	case UCSI_CONSTAT_PARTNER_TYPE_CABLE_AND_UFP:
	case UCSI_CONSTAT_PARTNER_TYPE_CABLE:
		priv->con[con_idx].data_role = TYPEC_HOST;
		break;
	case UCSI_CONSTAT_PARTNER_TYPE_DFP:
		priv->con[con_idx].data_role = TYPEC_DEVICE;
		break;
	}

	ret = ucsi_acknowledge_connector_change(parent);
exit:
	dev_dbg(child, "connector[%d] status: %s data role: %s\n",
		con_idx,
		priv->con[con_idx].attached == TYPEC_ATTACHED ? "Attached" : "Unattached",
		priv->con[con_idx].data_role == TYPEC_HOST ? "Host" : "Device");

	return ret;
}

int ucsi_post_probe(struct udevice *dev)
{
	struct connector *con;
	struct ucsi_priv *priv;
	struct udevice *child;
	struct ucsi_capability cap;
	u64 command;
	int ret;
	u8 i;

	/* Reset the PPM */
	ret = ucsi_reset_ppm(dev);
	if (ret) {
		dev_err(dev, "failed to reset PPM!\n");
		return ret;
	}

	/* enable connector change notification */
	command = UCSI_SET_NOTIFICATION_ENABLE | UCSI_ENABLE_NTFY_CONNECTOR_CHANGE;
	ret = ucsi_send_command(dev, command, NULL, 0);
	if (ret < 0)
		return ret;

	/* get current status : attached/unattached, device/host */
	ret = device_get_child(dev, 0, &child);
	if (ret < 0)
		return ret;

	/* Get PPM capabilities */
	command = UCSI_GET_CAPABILITY;
	ret = ucsi_send_command(dev, command, &cap, sizeof(cap));
	if (ret < 0)
		return ret;

	if (!cap.num_connectors)
		return -ENODEV;

	priv = dev_get_priv(child);
	priv->nb_connector = cap.num_connectors;
	priv->con = kcalloc(priv->nb_connector, sizeof(*con), GFP_KERNEL);
	if (!priv->con)
		return -ENOMEM;

	for (i = 0; i < priv->nb_connector; i++) {
		ret = ucsi_get_status(child, i, true);
		if (ret < 0)
			return ret;
	}

	return 0;
}

UCLASS_DRIVER(ucsi) = {
	.id		= UCLASS_UCSI,
	.name		= "ucsi",
	.post_probe	= ucsi_post_probe,
};

static int ucsi_is_attached(struct udevice *dev, u8 con_idx)
{
	struct ucsi_priv *priv = dev_get_priv(dev);
	int ret;

	ret = ucsi_get_status(dev, con_idx, false);
	if (ret < 0)
		return ret;

	return priv->con[con_idx].attached;
}

static int ucsi_get_data_role(struct udevice *dev, u8 con_idx)
{
	struct ucsi_priv *priv = dev_get_priv(dev);
	int ret;

	ret = ucsi_get_status(dev, con_idx, false);
	if (ret < 0)
		return ret;

	return priv->con[con_idx].data_role;
}

static u8 usci_get_nb_connector(struct udevice *dev)
{
	struct ucsi_priv *priv = dev_get_priv(dev);

	return priv->nb_connector;
}

static const struct typec_ops ucsi_typec_ops = {
	.is_attached = ucsi_is_attached,
	.get_data_role = ucsi_get_data_role,
	.get_nb_connector = usci_get_nb_connector,
};

static const struct udevice_id typec_of_match[] = {
	{ .compatible = "usb-c-connector"},
	{}
};

U_BOOT_DRIVER(typec_ucsi) = {
	.id		= UCLASS_USB_TYPEC,
	.name		= "typec_ucsi",
	.of_match	= typec_of_match,
	.ops		= &ucsi_typec_ops,
	.priv_auto	= sizeof(struct ucsi_priv),
};
