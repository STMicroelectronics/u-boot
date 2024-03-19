// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/*
 * Copyright (C) 2019, STMicroelectronics - All Rights Reserved
 */
#define LOG_CATEGORY UCLASS_REMOTEPROC

#include <common.h>
#include <dm.h>
#include <errno.h>
#include <fdtdec.h>
#include <log.h>
#include <nvmem.h>
#include <remoteproc.h>
#include <rproc_optee.h>
#include <reset.h>
#include <asm/io.h>
#include <dm/device_compat.h>
#include <linux/err.h>

#define STM32MP15_M4_FW_ID	0
#define STM32MP25_M33_FW_ID	1

/* TAMP_COPRO_STATE register values */
#define TAMP_COPRO_STATE_OFF		0
#define TAMP_COPRO_STATE_INIT		1
#define TAMP_COPRO_STATE_CRUN		2
#define TAMP_COPRO_STATE_CSTOP		3
#define TAMP_COPRO_STATE_STANDBY	4
#define TAMP_COPRO_STATE_CRASH		5

/**
 * struct stm32_copro_privdata - power processor private data
 * @reset_ctl:		reset controller handle
 * @hold_boot:		hold boot controller handle
 * @rsc_table_addr:	resource table address
 */
struct stm32_copro_privdata {
	struct reset_ctl reset_ctl;
	struct reset_ctl hold_boot;
	ulong rsc_table_addr;
	ulong rsc_table_size;
	struct nvmem_cell rsc_t_addr_cell;
	struct nvmem_cell rsc_t_size_cell;
	struct rproc_optee trproc;
};

static int stm32_copro_stop(struct udevice *dev);

/**
 * stm32_copro_probe() - Basic probe
 * @dev:	remote processor device
 * Return: 0 if all went ok, else corresponding -ve error
 */
static int stm32_copro_probe(struct udevice *dev)
{
	struct stm32_copro_privdata *priv = dev_get_priv(dev);
	struct rproc_optee *trproc = &priv->trproc;
	int ret;

	trproc->proc_id = (u32)dev_get_driver_data(dev);

	if (trproc->proc_id == STM32MP25_M33_FW_ID) {
		ret = nvmem_cell_get_by_name(dev, "rsc-tbl-addr", &priv->rsc_t_addr_cell);
		if (ret && ret != -ENODATA)
			return ret;
		ret = nvmem_cell_get_by_name(dev, "rsc-tbl-size", &priv->rsc_t_size_cell);
		if (ret && ret != -ENODATA)
			return ret;
	}

	if (device_is_compatible(dev, "st,stm32mp1-m4-tee") ||
	    device_is_compatible(dev, "st,stm32mp2-m33-tee")) {
		ret = rproc_optee_open(trproc);
		if (ret) {
			dev_err(dev, "failed to delegate to OP-TEE\n");
			return ret;
		}

		dev_info(dev, "delegate the firmware management to OP-TEE\n");
	} else {
		ret = reset_get_by_name(dev, "mcu_rst", &priv->reset_ctl);
		if (ret) {
			dev_err(dev, "failed to get reset (%d)\n", ret);
			return ret;
		}

		ret = reset_get_by_name(dev, "hold_boot", &priv->hold_boot);
		if (ret) {
			dev_err(dev, "failed to get hold boot (%d)\n", ret);
			return ret;
		}
	}

	dev_dbg(dev, "probed\n");

	return 0;
}

/**
 * stm32_copro_optee_remove() - Close the rproc trusted application session
 * @dev:	 remote processor device
 * @return 0 if all went ok, else corresponding -ve error
 */
static int stm32_copro_remove(struct udevice *dev)
{
	struct stm32_copro_privdata *priv = dev_get_priv(dev);
	struct rproc_optee *trproc = &priv->trproc;

	if (trproc->tee)
		return rproc_optee_close(trproc);

	return 0;
}

/**
 * stm32_copro_device_to_phys() - Convert device address to physical address
 * @dev:	remote processor device
 * @da:		device address
 * @size:	Size of the memory region @da is pointing to
 * Return: converted physical address
 */
static phys_addr_t stm32_copro_device_to_phys(struct udevice *dev, ulong da,
					      ulong size)
{
	fdt32_t in_addr = cpu_to_be32(da), end_addr;
	phys_addr_t paddr;

	paddr = dev_translate_dma_address(dev, &in_addr);
	if (paddr == OF_BAD_ADDR) {
		dev_err(dev, "Unable to convert address %ld\n", da);
		return 0;
	}
	end_addr = cpu_to_be32(da + size - 1);
	if (dev_translate_dma_address(dev, &end_addr) == OF_BAD_ADDR) {
		dev_err(dev, "Unable to convert address %ld\n", da + size - 1);
		return 0;
	}

	return paddr;
}
/**
 * stm32_copro_device_to_virt() - Convert device address to virtual address
 * @dev:	remote processor device
 * @da:		device address
 * @size:	Size of the memory region @da is pointing to
 * Return: converted virtual address
 */
static void *stm32_copro_device_to_virt(struct udevice *dev, ulong da,
					ulong size)
{
	phys_addr_t paddr;

	paddr = stm32_copro_device_to_phys(dev, da, size);
	if (!paddr)
		return NULL;

	return phys_to_virt(paddr);
}

/**
 * stm32_copro_load() - Loadup the STM32 remote processor
 * @dev:	remote processor device
 * @addr:	Address in memory where image is stored
 * @size:	Size in bytes of the image
 * Return: 0 if all went ok, else corresponding -ve error
 */
static int stm32_copro_load(struct udevice *dev, ulong addr, ulong size)
{
	struct stm32_copro_privdata *priv = dev_get_priv(dev);
	struct rproc_optee *trproc = &priv->trproc;
	ulong rsc_table_size = 0;
	ulong rsc_table_addr = 0;
	phys_addr_t paddr;
	int ret;

	if (trproc->tee)
		return rproc_optee_load(trproc, addr, size);

	ret = reset_assert(&priv->hold_boot);
	if (ret) {
		dev_err(dev, "Unable to assert hold boot (ret=%d)\n", ret);
		return ret;
	}

	ret = reset_assert(&priv->reset_ctl);
	if (ret) {
		dev_err(dev, "Unable to assert reset line (ret=%d)\n", ret);
		return ret;
	}

	priv->rsc_table_addr = 0;
	ret = rproc_elf32_load_rsc_table(dev, addr, size, &rsc_table_addr,
					 &rsc_table_size);
	if (ret) {
		if (ret != -ENODATA)
			return ret;
		dev_dbg(dev, "No resource table for this firmware\n");
	}

	paddr = stm32_copro_device_to_phys(dev, rsc_table_addr, rsc_table_size);

	priv->rsc_table_addr = (ulong)paddr;
	priv->rsc_table_size = rsc_table_size;

	return rproc_elf32_load_image(dev, addr, size);
}

/**
 * stm32_copro_start() - Start the STM32 remote processor
 * @dev:	remote processor device
 * Return: 0 if all went ok, else corresponding -ve error
 */
static int stm32_copro_start(struct udevice *dev)
{
	struct stm32_copro_privdata *priv = dev_get_priv(dev);
	struct rproc_optee *trproc = &priv->trproc;
	unsigned int proc_id = (u32)dev_get_driver_data(dev);
	phys_size_t rsc_size;
	phys_addr_t rsc_addr;
	int ret;

	if (trproc->tee) {
		ret = rproc_optee_get_rsc_table(trproc, &rsc_addr, &rsc_size);
		if (ret)
			return ret;

		priv->rsc_table_size = (ulong)rsc_size;
		priv->rsc_table_addr = (ulong)rsc_addr;

		ret = rproc_optee_start(trproc);
		if (ret)
			return ret;

	} else {
		ret = reset_deassert(&priv->hold_boot);
		if (ret) {
			dev_err(dev, "Unable to deassert hold boot (ret=%d)\n",
				ret);
			return ret;
		}

		/*
		 * Once copro running, reset hold boot flag to avoid copro
		 * rebooting autonomously (error should never occur)
		 */
		ret = reset_assert(&priv->hold_boot);
		if (ret)
			dev_err(dev, "Unable to assert hold boot (ret=%d)\n",
				ret);
	}

	if (proc_id == STM32MP15_M4_FW_ID) {
#ifdef CONFIG_STM32MP15X
		/* Indicates that copro is running */
		writel(TAMP_COPRO_STATE_CRUN, TAMP_COPRO_STATE);

		/* Store rsc_address in bkp register */
		writel(priv->rsc_table_addr, TAMP_COPRO_RSC_TBL_ADDRESS);
#else
		return -EOPNOTSUPP;
#endif
	} else if (proc_id == STM32MP25_M33_FW_ID) {
		/* Store the resource table address and size in 32-bit registers*/
		ret = nvmem_cell_write(&priv->rsc_t_addr_cell, &priv->rsc_table_addr, sizeof(u32));
		if (ret)
			goto error;

		ret = nvmem_cell_write(&priv->rsc_t_size_cell, &priv->rsc_table_size, sizeof(u32));
		if (ret)
			goto error;
	}

	return 0;

error:
	stm32_copro_stop(dev);

	return ret;
}

/**
 * stm32_copro_reset() - Reset the STM32 remote processor
 * @dev:	remote processor device
 * Return: 0 if all went ok, else corresponding -ve error
 */
static int stm32_copro_reset(struct udevice *dev)
{
	struct stm32_copro_privdata *priv = dev_get_priv(dev);
	struct rproc_optee *trproc = &priv->trproc;
	unsigned int proc_id = (u32)dev_get_driver_data(dev);
	int ret;


	if (trproc->tee) {
		ret = rproc_optee_stop(trproc);
		if (ret)
			return ret;
	} else {
		ret = reset_assert(&priv->hold_boot);
		if (ret) {
			dev_err(dev, "Unable to assert hold boot (ret=%d)\n",
				ret);
			return ret;
		}

		ret = reset_assert(&priv->reset_ctl);
		if (ret) {
			dev_err(dev, "Unable to assert reset line (ret=%d)\n",
				ret);
			return ret;
		}
	}

	/* Clean-up backup registers */
	priv->rsc_table_addr = 0;
	priv->rsc_table_size = 0;

	if (proc_id == STM32MP15_M4_FW_ID) {
#ifdef CONFIG_STM32MP15X
		writel(TAMP_COPRO_STATE_OFF, TAMP_COPRO_STATE);
		writel(priv->rsc_table_addr, TAMP_COPRO_RSC_TBL_ADDRESS);
#else
		return -EOPNOTSUPP;
#endif
	} else if (proc_id == STM32MP25_M33_FW_ID) {
		ret = nvmem_cell_write(&priv->rsc_t_addr_cell, &priv->rsc_table_addr, sizeof(u32));
		if (ret)
			return ret;

		ret = nvmem_cell_write(&priv->rsc_t_size_cell, &priv->rsc_table_size, sizeof(u32));
		if (ret)
			return ret;
	}

	return 0;
}

/**
 * stm32_copro_stop() - Stop the STM32 remote processor
 * @dev:	remote processor device
 * Return: 0 if all went ok, else corresponding -ve error
 */
static int stm32_copro_stop(struct udevice *dev)
{
	return stm32_copro_reset(dev);
}

/**
 * stm32_copro_is_running() - Is the STM32 remote processor running
 * @dev:	remote processor device
 * Return: 0 if the remote processor is running, 1 otherwise
 */
static int stm32_copro_is_running(struct udevice *dev)
{
#ifdef CONFIG_STM32MP15X
	unsigned int proc_id = (u32)dev_get_driver_data(dev);

	if (proc_id == STM32MP15_M4_FW_ID)
		return (readl(TAMP_COPRO_STATE) == TAMP_COPRO_STATE_OFF);
#endif
	return -EOPNOTSUPP;
}

static const struct dm_rproc_ops stm32_copro_ops = {
	.load = stm32_copro_load,
	.start = stm32_copro_start,
	.stop =  stm32_copro_stop,
	.reset = stm32_copro_reset,
	.is_running = stm32_copro_is_running,
	.device_to_virt = stm32_copro_device_to_virt,
};

static const struct udevice_id stm32_copro_ids[] = {
	{ .compatible = "st,stm32mp1-m4", .data = STM32MP15_M4_FW_ID },
	{ .compatible = "st,stm32mp1-m4-tee", .data = STM32MP15_M4_FW_ID },
	{ .compatible = "st,stm32mp2-m33-tee", .data = STM32MP25_M33_FW_ID },
	{}
};

U_BOOT_DRIVER(stm32_copro) = {
	.name = "stm32_copro",
	.of_match = stm32_copro_ids,
	.id = UCLASS_REMOTEPROC,
	.ops = &stm32_copro_ops,
	.probe = stm32_copro_probe,
	.remove = stm32_copro_remove,
	.priv_auto = sizeof(struct stm32_copro_privdata),
	.flags = DM_FLAG_OS_PREPARE,
};
