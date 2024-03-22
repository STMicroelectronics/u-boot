// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021, STMicroelectronics - All Rights Reserved
 */

#include <common.h>
#include <clk.h>
#include <dm.h>
#include <regmap.h>
#include <stm32_omi.h>
#include <syscon.h>
#include <dm/device_compat.h>
#include <dm/device-internal.h>
#include <dm/lists.h>
#include <dm/of_addr.h>
#include <dm/of_access.h>
#include <linux/bitfield.h>
#include <linux/ioport.h>
#include <mach/rif.h>

#define OCTOSPIM_CR		0
#define CR_MUXEN		BIT(0)
#define CR_MUXENMODE_MASK	GENMASK(1, 0)
#define CR_CSSEL_OVR_EN		BIT(4)
#define CR_CSSEL_OVR_MASK	GENMASK(6, 5)
#define CR_REQ2ACK_MASK		GENMASK(23, 16)

#define OMM_CHILD_NB		2

#define NSEC_PER_SEC		1000000000L

struct stm32_omm_plat {
	struct regmap *omm_regmap;
	struct regmap *syscfg_regmap;
	struct clk clk;
	struct reset_ctl reset_ctl;
	resource_size_t mm_ospi2_size;
	u32 mux;
	u32 cssel_ovr;
	u32 req2ack;
	u32 amcr_base;
	u32 amcr_mask;
	unsigned long clk_rate_max;
};

static int stm32_omm_set_amcr(struct udevice *dev, bool set)
{
	struct stm32_omm_plat *plat = dev_get_plat(dev);
	unsigned int amcr, read_amcr;

	amcr = plat->mm_ospi2_size / SZ_64M;

	if (set)
		regmap_update_bits(plat->syscfg_regmap, plat->amcr_base,
				   plat->amcr_mask, amcr);

	/* read AMCR and check coherency with memory-map areas defined in DT */
	regmap_read(plat->syscfg_regmap, plat->amcr_base, &read_amcr);
	read_amcr = read_amcr >> (ffs(plat->amcr_mask) - 1);

	if (amcr != read_amcr) {
		dev_err(dev, "AMCR value not coherent with DT memory-map areas\n");
		return -EINVAL;
	}

	return 0;
}

static int stm32_omm_configure(struct udevice *dev)
{
	struct stm32_omm_plat *plat = dev_get_plat(dev);
	int ret;
	u32 mux = 0;
	u32 cssel_ovr = 0;
	u32 req2ack = 0;

	ret = clk_enable(&plat->clk);
	if (ret) {
		dev_err(dev, "Failed to enable OMM clock (%d)\n", ret);
		return ret;
	}

	reset_assert(&plat->reset_ctl);
	udelay(2);
	reset_deassert(&plat->reset_ctl);

	if (plat->mux & CR_MUXEN) {
		if (!plat->req2ack) {
			req2ack = DIV_ROUND_UP(plat->req2ack,
					       NSEC_PER_SEC / plat->clk_rate_max) - 1;
			if (req2ack > 256)
				req2ack = 256;
		}

		req2ack = FIELD_PREP(CR_REQ2ACK_MASK, req2ack);
		regmap_update_bits(plat->omm_regmap, OCTOSPIM_CR,
					 CR_REQ2ACK_MASK, req2ack);
	}

	if (plat->cssel_ovr != 0xff) {
		cssel_ovr = FIELD_PREP(CR_CSSEL_OVR_MASK, cssel_ovr);
		cssel_ovr |= CR_CSSEL_OVR_EN;
		regmap_update_bits(plat->omm_regmap, OCTOSPIM_CR,
				   CR_CSSEL_OVR_MASK, cssel_ovr);
	}

	mux = FIELD_PREP(CR_MUXENMODE_MASK, plat->mux);
	regmap_update_bits(plat->omm_regmap, OCTOSPIM_CR,
			   CR_MUXENMODE_MASK, mux);

	clk_disable(&plat->clk);

	return stm32_omm_set_amcr(dev, true);
}

static int stm32_omm_disable_child(struct udevice *dev, ofnode child)
{
	struct regmap *omi_regmap;
	struct clk omi_clk;
	int ret;

	ret = regmap_init_mem(child, &omi_regmap);
	if (ret) {
		dev_err(dev, "Regmap failed for node %s\n", ofnode_get_name(child));
		return ret;
	}

	/* retrieve OMI clk */
	ret = clk_get_by_index_nodev(child, 0, &omi_clk);
	if (ret) {
		dev_err(dev, "Failed to get clock for %s\n", ofnode_get_name(child));
		return ret;
	}

	ret = clk_enable(&omi_clk);
	if (ret) {
		dev_err(dev, "Failed to enable clock for %s\n", ofnode_get_name(child));
		goto clk_free;
	}

	regmap_update_bits(omi_regmap, OSPI_CR, OSPI_CR_EN, 0);

	clk_disable(&omi_clk);
clk_free:
	clk_free(&omi_clk);

	return ret;
}

static int stm32_omm_enable_child_clock(struct udevice *dev, ofnode child)
{
	struct clk omi_clk;
	int ret;

	ret = clk_get_by_index_nodev(child, 0, &omi_clk);
	if (ret) {
		dev_err(dev, "Failed to get clock for %s\n", ofnode_get_name(child));
		return ret;
	}

	ret = clk_enable(&omi_clk);
	if (ret)
		dev_err(dev, "Failed to enable clock for %s\n", ofnode_get_name(child));

	clk_free(&omi_clk);

	return ret;
}

static int stm32_omm_probe(struct udevice *dev) {
	struct stm32_omm_plat *plat = dev_get_plat(dev);
	ofnode child_list[OMM_CHILD_NB];
	ofnode child;
	int ret;
	u8 nb_child = 0;
	u8 child_access_granted = 0;
	u8 i;
	bool child_access[OMM_CHILD_NB];

	/* check child's access */
	for (child = ofnode_first_subnode(dev_ofnode(dev));
	     ofnode_valid(child);
	     child = ofnode_next_subnode(child)) {

		if (nb_child > OMM_CHILD_NB) {
			dev_err(dev, "Bad DT, found too much children\n");
			return -E2BIG;
		}

		if (!ofnode_device_is_compatible(child, "st,stm32mp25-omi"))
			return -EINVAL;

		ret = stm32_rifsc_check_access(child);
		if (ret < 0 && ret != -EACCES)
			return ret;

		child_access[nb_child] = false;
		if (!ret) {
			child_access_granted++;
			child_access[nb_child] = true;
		}

		child_list[nb_child] = child;
		nb_child++;
	}

	if (nb_child != OMM_CHILD_NB)
		return -EINVAL;

	/* check if OMM's ressource access is granted */
	ret = stm32_rifsc_check_access(dev_ofnode(dev));
	if (ret < 0 && ret != -EACCES)
		return ret;

	/* All child's access are granted ? */
	if (!ret && child_access_granted == nb_child) {
		/* Ensure both OSPI instance are disabled before configuring OMM */
		for (i = 0; i < nb_child; i++) {
			ret = stm32_omm_disable_child(dev, child_list[i]);
			if (ret)
				return ret;
		}

		ret = stm32_omm_configure(dev);
		if (ret)
			return ret;

		if (plat->mux & CR_MUXEN) {
			/*
			 * If the mux is enabled, the 2 OSPI clocks have to be
			 * always enabled
			 */

			for (i = 0; i < nb_child; i++) {
				ret = stm32_omm_enable_child_clock(dev, child_list[i]);
				if (ret)
					return ret;
			}
		}
	} else {
		dev_dbg(dev, "Octo Memory Manager resource's access not granted\n");
		/*
		 * AMCR can't be set, so check if current value is coherent
		 * with memory-map areas defined in DT
		 */
		ret = stm32_omm_set_amcr(dev, false);
	}

	return ret;
}

static int stm32_omm_of_to_plat(struct udevice *dev)
{
	struct stm32_omm_plat *plat = dev_get_plat(dev);
	static const char *mm_name[] = { "mm_ospi1", "mm_ospi2" };
	struct resource res, res1, mm_res;
	struct ofnode_phandle_args args;
	struct udevice *child;
	unsigned long clk_rate;
	struct clk child_clk;
	u32 mm_size = 0;
	int ret, idx;
	u8 i;

	ret = regmap_init_mem(dev_ofnode(dev), &plat->omm_regmap);
	if (ret) {
		dev_err(dev, "I/O manager regmap failed\n");
		return ret;
	}

	ret = dev_read_resource_byname(dev, "omm_mm", &mm_res);
	if (ret) {
		dev_err(dev, "can't get omm_mm mmap resource(ret = %d)!\n", ret);
		return ret;
	}

	ret = reset_get_by_index(dev, 0, &plat->reset_ctl);
	if (ret)
		return ret;

	ret = clk_get_by_index(dev, 0, &plat->clk);
	if (ret < 0) {
		dev_err(dev, "Can't find I/O manager clock\n");
		return ret;
	}

	/* parse children's clock */
	plat->clk_rate_max = 0;
	device_foreach_child(child, dev) {
		ret = clk_get_by_index(child, 0, &child_clk);
		if (ret) {
			dev_err(dev, "Failed to get clock for %s\n",
				dev_read_name(child));
			return ret;
		}

		clk_rate = clk_get_rate(&child_clk);
		clk_free(&child_clk);
		if (!clk_rate) {
			dev_err(dev, "Invalid clock rate\n");
			return -EINVAL;
		}

		if (clk_rate > plat->clk_rate_max)
			plat->clk_rate_max = clk_rate;
	}

	plat->mux = dev_read_u32_default(dev, "st,omm-mux", 0);
	plat->req2ack = dev_read_u32_default(dev, "st,omm-req2ack-ns", 0);
	plat->cssel_ovr = dev_read_u32_default(dev, "st,omm-cssel-ovr", 0xff);
	plat->mm_ospi2_size = 0;

	for (i = 0; i < 2; i++) {
		idx = dev_read_stringlist_search(dev, "memory-region-names",
						 mm_name[i]);
		if (idx < 0)
			continue;

		/* res1 only used on second loop iteration */
		res1.start = res.start;
		res1.end = res.end;

		dev_read_phandle_with_args(dev, "memory-region", NULL, 0, idx,
					   &args);
		ret = ofnode_read_resource(args.node, 0, &res);
		if (ret) {
			dev_err(dev, "unable to resolve memory region\n");
			goto clk_free;
		}

		/* check that memory region fits inside OMM memory map area */
		if (!resource_contains(&mm_res, &res)) {
			dev_err(dev, "%s doesn't fit inside OMM memory map area\n",
				mm_name[i]);
			dev_err(dev, "[0x%llx-0x%llx] doesn't fit inside [0x%llx-0x%llx]\n",
				res.start, res.end,
				mm_res.start, mm_res.end);

			return -EFAULT;
		}

		if (i == 1) {
			plat->mm_ospi2_size = resource_size(&res);

			/* check that OMM memory region 1 doesn't overlap memory region 2 */
			if (resource_overlaps(&res, &res1)) {
				dev_err(dev, "OMM memory-region %s overlaps memory region %s\n",
					mm_name[0], mm_name[1]);
				dev_err(dev, "[0x%llx-0x%llx] overlaps [0x%llx-0x%llx]\n",
					res1.start, res1.end, res.start, res.end);

				return -EFAULT;
			}
		}

		mm_size += resource_size(&res);
	}

	plat->syscfg_regmap = syscon_regmap_lookup_by_phandle(dev, "st,syscfg-amcr");
	if (IS_ERR(plat->syscfg_regmap)) {
		dev_err(dev, "Failed to get st,syscfg-amcr property\n");
		ret = PTR_ERR(plat->syscfg_regmap);
		goto clk_free;
	}

	ret = dev_read_u32_index(dev, "st,syscfg-amcr", 1, &plat->amcr_base);
	if (ret) {
		dev_err(dev, "Failed to get st,syscfg-amcr base\n");
		goto clk_free;
	}

	ret = dev_read_u32_index(dev, "st,syscfg-amcr", 2, &plat->amcr_mask);
	if (ret) {
		dev_err(dev, "Failed to get st,syscfg-amcr mask\n");
		goto clk_free;
	}

	return 0;

clk_free:
	clk_free(&plat->clk);

	return ret;
};

static int stm32_omm_bind(struct udevice *dev)
{
	int ret = 0, err = 0;
	ofnode node;

	for (node = ofnode_first_subnode(dev_ofnode(dev));
	     ofnode_valid(node);
	     node = ofnode_next_subnode(node)) {
		const char *node_name = ofnode_get_name(node);

		if (!ofnode_is_enabled(node) || stm32_rifsc_check_access(node)) {
			dev_dbg(dev, "%s failed to bind\n", node_name);
			continue;
		}

		err = lists_bind_fdt(dev, node, NULL, NULL,
				     gd->flags & GD_FLG_RELOC ? false : true);
		if (err && !ret) {
			ret = err;
			dev_dbg(dev, "%s: ret=%d\n", node_name, ret);
		}
	}

	if (ret)
		dev_dbg(dev, "Some drivers failed to bind\n");

	return ret;
}

static const struct udevice_id stm32_omm_ids[] = {
	{ .compatible = "st,stm32mp25-omm", },
	{},
};

U_BOOT_DRIVER(stm32_omm) = {
	.name		= "stm32_omm",
	.id		= UCLASS_NOP,
	.probe		= stm32_omm_probe,
	.of_match	= stm32_omm_ids,
	.of_to_plat	= stm32_omm_of_to_plat,
	.plat_auto	= sizeof(struct stm32_omm_plat),
	.bind		= stm32_omm_bind,
};
