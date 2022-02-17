// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021, STMicroelectronics - All Rights Reserved
 */

#define LOG_CATEGORY UCLASS_UCSI

#include <dm.h>
#include <errno.h>
#include <i2c.h>
#include <ucsi.h>
#include <dm/device.h>
#include <dm/device_compat.h>
#include <linux/delay.h>

static int stm32_ucsi_read(struct udevice *dev, unsigned int offset, void *val, size_t len)
{
	struct dm_i2c_chip *chip = dev_get_parent_plat(dev);
	u8 reg = offset;
	struct i2c_msg msg[] = {
		{
			.addr	= chip->chip_addr,
			.flags  = 0,
			.len	= 1,
			.buf	= &reg,
		},
		{
			.addr	= chip->chip_addr,
			.flags  = I2C_M_RD,
			.len	= len,
			.buf	= val,
		},
	};
	int ret;

	ret = dm_i2c_xfer(dev, msg, ARRAY_SIZE(msg));
	if (ret)
		dev_err(dev, "i2c read failed @offset 0x%x (%d)\n", offset, ret);

	/*
	 * Add this delay to ensure that PPM has completed the current command,
	 * before sending it another one.
	 */
	udelay(20);

	return ret;
}

static int stm32_ucsi_write(struct udevice *dev, unsigned int offset,
			    const void *val, size_t len)
{
	struct dm_i2c_chip *chip = dev_get_parent_plat(dev);
	struct i2c_msg msg[] = {
		{
			.addr	= chip->chip_addr,
			.flags  = 0,
		}
	};
	unsigned char *buf;
	int ret;

	buf = kzalloc(len + 1, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf[0] = offset;
	memcpy(&buf[1], val, len);
	msg[0].len = len + 1;
	msg[0].buf = buf;

	ret = dm_i2c_xfer(dev, msg, ARRAY_SIZE(msg));
	kfree(buf);
	if (ret)
		dev_err(dev, "i2c write failed @offset 0x%x (%d)\n", offset, ret);

	/*
	 * Add this delay to ensure that PPM has completed the current command,
	 * before sending it another one.
	 */
	mdelay(2);

	return ret;
}

int stm32_ucsi_probe(struct udevice *dev)
{
	u16 ucsi_version;
	int ret;

	ret = stm32_ucsi_read(dev, UCSI_VERSION, &ucsi_version, sizeof(ucsi_version));
	if (ret < 0)
		return ret;

	dev_dbg(dev, "STM32G0 version 0x%x\n", ucsi_version);

	return 0;
}

static const struct ucsi_ops stm32_ucsi_ops = {
	.read = stm32_ucsi_read,
	.write = stm32_ucsi_write,
};

static const struct udevice_id stm32_ucsi_of_match[] = {
	{ .compatible = "st,stm32g0-typec"},
	{}
};

U_BOOT_DRIVER(ucsi_stm32g0) = {
	.id		= UCLASS_UCSI,
	.name		= "ucsi-stm32g0",
	.of_match	= stm32_ucsi_of_match,
	.probe		= stm32_ucsi_probe,
	.ops		= &stm32_ucsi_ops,
	.bind		= dm_scan_fdt_dev,
};
