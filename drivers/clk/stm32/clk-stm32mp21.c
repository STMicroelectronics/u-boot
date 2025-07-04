// SPDX-License-Identifier: GPL-2.0-or-later OR BSD-3-Clause
/*
 * Copyright (C) 2024, STMicroelectronics - All Rights Reserved
 */

#include <clk-uclass.h>
#include <common.h>
#include <dm.h>
#include <dt-bindings/clock/st,stm32mp21-rcc.h>
#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <mach/rif.h>

#include "clk-stm32-core.h"
#include "stm32mp21_rcc.h"

/* Clock security definition */
#define SECF_NONE		-1

#define RCC_REG_SIZE	32
#define RCC_SECCFGR(x)	(((x) / RCC_REG_SIZE) * 0x4 + RCC_SECCFGR0)
#define RCC_CIDCFGR(x)	((x) * 0x8 + RCC_R0CIDCFGR)
#define RCC_SEMCR(x)	((x) * 0x8 + RCC_R0SEMCR)
#define RCC_CID1	1

/* Register: RIFSC_CIDCFGR */
#define RCC_CIDCFGR_CFEN	BIT(0)
#define RCC_CIDCFGR_SEM_EN	BIT(1)
#define RCC_CIDCFGR_SEMWLC1_EN	BIT(17)
#define RCC_CIDCFGR_SCID_MASK	GENMASK(6, 4)

/* Register: RIFSC_SEMCR */
#define RCC_SEMCR_SEMCID_MASK	GENMASK(6, 4)

#define STM32MP21_RIFRCC_DBG_ID		73
#define STM32MP21_RIFRCC_MCO1_ID	108
#define STM32MP21_RIFRCC_MCO2_ID	109
#define STM32MP21_RIFRCC_OSPI1_ID	110

#define SEC_RIFSC_FLAG		BIT(31)
#define SEC_RIFRCC(_id)		(STM32MP21_RIFRCC_##_id##_ID)
#define SEC_RIFSC(_id)		((_id) | SEC_RIFSC_FLAG)

static const char * const adc1_src[] = {
	"ck_flexgen_46", "ck_icn_ls_mcu"
};

static const char * const adc2_src[] = {
	"ck_flexgen_47", "ck_icn_ls_mcu", "ck_flexgen_46"
};

static const char * const usb2phy1_src[] = {
	"ck_flexgen_57", "hse_div2_ck"
};

static const char * const usb2phy2_src[] = {
	"ck_flexgen_58", "hse_div2_ck"
};

static const char * const dts_src[] = {
	"hsi_ck", "hse_ck", "msi_ck"
};

static const char * const mco1_src[] = {
	"ck_flexgen_61", "ck_obs0"
};

static const char * const mco2_src[] = {
	"ck_flexgen_62", "ck_obs1"
};

enum enum_mux_cfg {
	MUX_MCO1,
	MUX_MCO2,
	MUX_ADC1,
	MUX_ADC2,
	MUX_USB2PHY1,
	MUX_USB2PHY2,
	MUX_DTS,
	MUX_NB
};

#define MUX_CFG(id, src, _offset, _shift, _witdh)[id] = {\
		.num_parents	= ARRAY_SIZE(src),\
		.parent_names	= src,\
		.reg_off	= (_offset),\
		.shift		= (_shift),\
		.width		= (_witdh),\
}

static const struct stm32_mux_cfg stm32mp21_muxes[MUX_NB] = {
	MUX_CFG(MUX_ADC1,		adc1_src,	RCC_ADC1CFGR,		12,	1),
	MUX_CFG(MUX_ADC2,		adc2_src,	RCC_ADC2CFGR,		12,	2),
	MUX_CFG(MUX_DTS,		dts_src,	RCC_DTSCFGR,		12,	2),
	MUX_CFG(MUX_MCO1,		mco1_src,	RCC_MCO1CFGR,		0,	1),
	MUX_CFG(MUX_MCO2,		mco2_src,	RCC_MCO2CFGR,		0,	1),
	MUX_CFG(MUX_USB2PHY1,		usb2phy1_src,	RCC_USB2PHY1CFGR,	15,	1),
	MUX_CFG(MUX_USB2PHY2,		usb2phy2_src,	RCC_USB2PHY2CFGR,	15,	1),
};

enum enum_gate_cfg {
	GATE_ADC1,
	GATE_ADC2,
	GATE_CRC,
	GATE_CRYP1,
	GATE_CRYP2,
	GATE_CSI,
	GATE_DBG,
	GATE_DCMIPP,
	GATE_DTS,
	GATE_ETH1,
	GATE_ETH1MAC,
	GATE_ETH1RX,
	GATE_ETH1STP,
	GATE_ETH1TX,
	GATE_ETH2,
	GATE_ETH2MAC,
	GATE_ETH2RX,
	GATE_ETH2STP,
	GATE_ETH2TX,
	GATE_ETR,
	GATE_FDCAN,
	GATE_HASH1,
	GATE_HASH2,
	GATE_HDP,
	GATE_I2C1,
	GATE_I2C2,
	GATE_I2C3,
	GATE_I3C1,
	GATE_I3C2,
	GATE_I3C3,
	GATE_IWDG1,
	GATE_IWDG2,
	GATE_IWDG3,
	GATE_IWDG4,
	GATE_LPTIM1,
	GATE_LPTIM2,
	GATE_LPTIM3,
	GATE_LPTIM4,
	GATE_LPTIM5,
	GATE_LPUART1,
	GATE_LTDC,
	GATE_MCO1,
	GATE_MCO2,
	GATE_MDF1,
	GATE_OSPI1,
	GATE_PKA,
	GATE_RNG1,
	GATE_RNG2,
	GATE_SAES,
	GATE_SAI1,
	GATE_SAI2,
	GATE_SAI3,
	GATE_SAI4,
	GATE_SDMMC1,
	GATE_SDMMC2,
	GATE_SDMMC3,
	GATE_SERC,
	GATE_SPDIFRX,
	GATE_SPI1,
	GATE_SPI2,
	GATE_SPI3,
	GATE_SPI4,
	GATE_SPI5,
	GATE_SPI6,
	GATE_STGEN,
	GATE_STM,
	GATE_TIM1,
	GATE_TIM2,
	GATE_TIM3,
	GATE_TIM4,
	GATE_TIM5,
	GATE_TIM6,
	GATE_TIM7,
	GATE_TIM8,
	GATE_TIM10,
	GATE_TIM11,
	GATE_TIM12,
	GATE_TIM13,
	GATE_TIM14,
	GATE_TIM15,
	GATE_TIM16,
	GATE_TIM17,
	GATE_TRACE,
	GATE_UART4,
	GATE_UART5,
	GATE_UART7,
	GATE_USART1,
	GATE_USART2,
	GATE_USART3,
	GATE_USART6,
	GATE_USBH,
	GATE_USBOTG,
	GATE_USB2PHY1,
	GATE_USB2PHY2,
	GATE_VREF,
	GATE_WWDG1,
	GATE_NB
};

#define GATE_CFG(id, _offset, _bit_idx, _offset_clr)[id] = {\
	.reg_off	= (_offset),\
	.bit_idx	= (_bit_idx),\
	.set_clr	= (_offset_clr),\
}

static const struct stm32_gate_cfg stm32mp21_gates[GATE_NB] = {
	GATE_CFG(GATE_MCO1,		RCC_MCO1CFGR,		8,	0),
	GATE_CFG(GATE_MCO2,		RCC_MCO2CFGR,		8,	0),
	GATE_CFG(GATE_OSPI1,		RCC_OSPI1CFGR,		1,	0),
	GATE_CFG(GATE_DBG,		RCC_DBGCFGR,		8,	0),
	GATE_CFG(GATE_TRACE,		RCC_DBGCFGR,		9,	0),
	GATE_CFG(GATE_STM,		RCC_STMCFGR,		1,	0),
	GATE_CFG(GATE_ETR,		RCC_ETRCFGR,		1,	0),
	GATE_CFG(GATE_TIM1,		RCC_TIM1CFGR,		1,	0),
	GATE_CFG(GATE_TIM2,		RCC_TIM2CFGR,		1,	0),
	GATE_CFG(GATE_TIM3,		RCC_TIM3CFGR,		1,	0),
	GATE_CFG(GATE_TIM4,		RCC_TIM4CFGR,		1,	0),
	GATE_CFG(GATE_TIM5,		RCC_TIM5CFGR,		1,	0),
	GATE_CFG(GATE_TIM6,		RCC_TIM6CFGR,		1,	0),
	GATE_CFG(GATE_TIM7,		RCC_TIM7CFGR,		1,	0),
	GATE_CFG(GATE_TIM8,		RCC_TIM8CFGR,		1,	0),
	GATE_CFG(GATE_TIM10,		RCC_TIM10CFGR,		1,	0),
	GATE_CFG(GATE_TIM11,		RCC_TIM11CFGR,		1,	0),
	GATE_CFG(GATE_TIM12,		RCC_TIM12CFGR,		1,	0),
	GATE_CFG(GATE_TIM13,		RCC_TIM13CFGR,		1,	0),
	GATE_CFG(GATE_TIM14,		RCC_TIM14CFGR,		1,	0),
	GATE_CFG(GATE_TIM15,		RCC_TIM15CFGR,		1,	0),
	GATE_CFG(GATE_TIM16,		RCC_TIM16CFGR,		1,	0),
	GATE_CFG(GATE_TIM17,		RCC_TIM17CFGR,		1,	0),
	GATE_CFG(GATE_LPTIM1,		RCC_LPTIM1CFGR,		1,	0),
	GATE_CFG(GATE_LPTIM2,		RCC_LPTIM2CFGR,		1,	0),
	GATE_CFG(GATE_LPTIM3,		RCC_LPTIM3CFGR,		1,	0),
	GATE_CFG(GATE_LPTIM4,		RCC_LPTIM4CFGR,		1,	0),
	GATE_CFG(GATE_LPTIM5,		RCC_LPTIM5CFGR,		1,	0),
	GATE_CFG(GATE_SPI1,		RCC_SPI1CFGR,		1,	0),
	GATE_CFG(GATE_SPI2,		RCC_SPI2CFGR,		1,	0),
	GATE_CFG(GATE_SPI3,		RCC_SPI3CFGR,		1,	0),
	GATE_CFG(GATE_SPI4,		RCC_SPI4CFGR,		1,	0),
	GATE_CFG(GATE_SPI5,		RCC_SPI5CFGR,		1,	0),
	GATE_CFG(GATE_SPI6,		RCC_SPI6CFGR,		1,	0),
	GATE_CFG(GATE_SPDIFRX,		RCC_SPDIFRXCFGR,	1,	0),
	GATE_CFG(GATE_USART1,		RCC_USART1CFGR,		1,	0),
	GATE_CFG(GATE_USART2,		RCC_USART2CFGR,		1,	0),
	GATE_CFG(GATE_USART3,		RCC_USART3CFGR,		1,	0),
	GATE_CFG(GATE_UART4,		RCC_UART4CFGR,		1,	0),
	GATE_CFG(GATE_UART5,		RCC_UART5CFGR,		1,	0),
	GATE_CFG(GATE_USART6,		RCC_USART6CFGR,		1,	0),
	GATE_CFG(GATE_UART7,		RCC_UART7CFGR,		1,	0),
	GATE_CFG(GATE_LPUART1,		RCC_LPUART1CFGR,	1,	0),
	GATE_CFG(GATE_I2C1,		RCC_I2C1CFGR,		1,	0),
	GATE_CFG(GATE_I2C2,		RCC_I2C2CFGR,		1,	0),
	GATE_CFG(GATE_I2C3,		RCC_I2C3CFGR,		1,	0),
	GATE_CFG(GATE_SAI1,		RCC_SAI1CFGR,		1,	0),
	GATE_CFG(GATE_SAI2,		RCC_SAI2CFGR,		1,	0),
	GATE_CFG(GATE_SAI3,		RCC_SAI3CFGR,		1,	0),
	GATE_CFG(GATE_SAI4,		RCC_SAI4CFGR,		1,	0),
	GATE_CFG(GATE_MDF1,		RCC_MDF1CFGR,		1,	0),
	GATE_CFG(GATE_FDCAN,		RCC_FDCANCFGR,		1,	0),
	GATE_CFG(GATE_HDP,		RCC_HDPCFGR,		1,	0),
	GATE_CFG(GATE_ADC1,		RCC_ADC1CFGR,		1,	0),
	GATE_CFG(GATE_ADC2,		RCC_ADC2CFGR,		1,	0),
	GATE_CFG(GATE_ETH1MAC,		RCC_ETH1CFGR,		1,	0),
	GATE_CFG(GATE_ETH1STP,		RCC_ETH1CFGR,		4,	0),
	GATE_CFG(GATE_ETH1,		RCC_ETH1CFGR,		5,	0),
	GATE_CFG(GATE_ETH1TX,		RCC_ETH1CFGR,		8,	0),
	GATE_CFG(GATE_ETH1RX,		RCC_ETH1CFGR,		10,	0),
	GATE_CFG(GATE_ETH2MAC,		RCC_ETH2CFGR,		1,	0),
	GATE_CFG(GATE_ETH2STP,		RCC_ETH2CFGR,		4,	0),
	GATE_CFG(GATE_ETH2,		RCC_ETH2CFGR,		5,	0),
	GATE_CFG(GATE_ETH2TX,		RCC_ETH2CFGR,		8,	0),
	GATE_CFG(GATE_ETH2RX,		RCC_ETH2CFGR,		10,	0),
	GATE_CFG(GATE_USBH,		RCC_USBHCFGR,		1,	0),
	GATE_CFG(GATE_USB2PHY1,		RCC_USB2PHY1CFGR,	1,	0),
	GATE_CFG(GATE_USBOTG,		RCC_OTGCFGR,		1,	0),
	GATE_CFG(GATE_USB2PHY2,		RCC_USB2PHY2CFGR,	1,	0),
	GATE_CFG(GATE_STGEN,		RCC_STGENCFGR,		1,	0),
	GATE_CFG(GATE_SDMMC1,		RCC_SDMMC1CFGR,		1,	0),
	GATE_CFG(GATE_SDMMC2,		RCC_SDMMC2CFGR,		1,	0),
	GATE_CFG(GATE_SDMMC3,		RCC_SDMMC3CFGR,		1,	0),
	GATE_CFG(GATE_LTDC,		RCC_LTDCCFGR,		1,	0),
	GATE_CFG(GATE_CSI,		RCC_CSICFGR,		1,	0),
	GATE_CFG(GATE_DCMIPP,		RCC_DCMIPPCFGR,		1,	0),
	GATE_CFG(GATE_RNG1,		RCC_RNG1CFGR,		1,	0),
	GATE_CFG(GATE_RNG2,		RCC_RNG2CFGR,		1,	0),
	GATE_CFG(GATE_PKA,		RCC_PKACFGR,		1,	0),
	GATE_CFG(GATE_SAES,		RCC_SAESCFGR,		1,	0),
	GATE_CFG(GATE_HASH1,		RCC_HASH1CFGR,		1,	0),
	GATE_CFG(GATE_HASH2,		RCC_HASH2CFGR,		1,	0),
	GATE_CFG(GATE_CRYP1,		RCC_CRYP1CFGR,		1,	0),
	GATE_CFG(GATE_CRYP2,		RCC_CRYP2CFGR,		1,	0),
	GATE_CFG(GATE_IWDG1,		RCC_IWDG1CFGR,		1,	0),
	GATE_CFG(GATE_IWDG2,		RCC_IWDG2CFGR,		1,	0),
	GATE_CFG(GATE_IWDG3,		RCC_IWDG3CFGR,		1,	0),
	GATE_CFG(GATE_IWDG4,		RCC_IWDG4CFGR,		1,	0),
	GATE_CFG(GATE_WWDG1,		RCC_WWDG1CFGR,		1,	0),
	GATE_CFG(GATE_VREF,		RCC_VREFCFGR,		1,	0),
	GATE_CFG(GATE_DTS,		RCC_DTSCFGR,		1,	0),
	GATE_CFG(GATE_CRC,		RCC_CRCCFGR,		1,	0),
	GATE_CFG(GATE_SERC,		RCC_SERCCFGR,		1,	0),
	GATE_CFG(GATE_I3C1,		RCC_I3C1CFGR,		1,	0),
	GATE_CFG(GATE_I3C2,		RCC_I3C2CFGR,		1,	0),
	GATE_CFG(GATE_I3C3,		RCC_I3C3CFGR,		1,	0),
};

static int stm32_rcc_get_access(struct udevice *dev, u32 index)
{
	fdt_addr_t rcc_base = dev_read_addr(dev->parent);
	u32 seccfgr, cidcfgr, semcr;
	int bit, cid;

	bit = index % RCC_REG_SIZE;

	seccfgr = readl(rcc_base + RCC_SECCFGR(index));
	if (seccfgr & BIT(bit))
		return -EACCES;

	cidcfgr = readl(rcc_base + RCC_CIDCFGR(index));
	if (!(cidcfgr & RCC_CIDCFGR_CFEN))
		/* CID filtering is turned off: access granted */
		return 0;

	if (!(cidcfgr & RCC_CIDCFGR_SEM_EN)) {
		/* Static CID mode */
		cid = FIELD_GET(RCC_CIDCFGR_SCID_MASK, cidcfgr);
		if (cid != RCC_CID1)
			return -EACCES;
		return 0;
	}

	/* Pass-list with semaphore mode */
	if (!(cidcfgr & RCC_CIDCFGR_SEMWLC1_EN))
		return -EACCES;

	semcr = readl(rcc_base + RCC_SEMCR(index));

	cid = FIELD_GET(RCC_SEMCR_SEMCID_MASK, semcr);
	if (cid != RCC_CID1)
		return -EACCES;

	return 0;
}

static int stm32mp21_check_security(struct udevice *dev, void __iomem *base,
				    const struct clock_config *cfg)
{
	int ret = 0;

	if (cfg->sec_id != SECF_NONE) {
		u32 index = (u32)cfg->sec_id;

		if (index & SEC_RIFSC_FLAG)
			ret = stm32_rifsc_grant_access_by_id(dev_ofnode(dev),
							     index & ~SEC_RIFSC_FLAG);
		else
			ret = stm32_rcc_get_access(dev, index);
	}

	return ret;
}

#define STM32_COMPOSITE_NODIV(_id, _name, _flags, _sec_id, _gate_id, _mux_id)\
	STM32_COMPOSITE(_id, _name, _flags, _sec_id, _gate_id, _mux_id, NO_STM32_DIV)

static const struct clock_config stm32mp21_clock_cfg[] = {
	/* ADC */
	STM32_GATE(CK_BUS_ADC1, "ck_icn_p_adc1", "ck_icn_ls_mcu", 0, GATE_ADC1,
		   SEC_RIFSC(58)),
	STM32_COMPOSITE_NODIV(CK_KER_ADC1, "ck_ker_adc12", 0, SEC_RIFSC(58),
			      GATE_ADC1, MUX_ADC1),
	STM32_GATE(CK_BUS_ADC2, "ck_icn_p_adc2", "ck_icn_ls_mcu", 0, GATE_ADC2, SEC_RIFSC(59)),
	STM32_COMPOSITE_NODIV(CK_KER_ADC2, "ck_ker_adc2", 0, SEC_RIFSC(59), GATE_ADC2, MUX_ADC2),

	/*TODO: check csi gate for all clocks ? */
	/* CSI-HOST */
	STM32_GATE(CK_BUS_CSI, "ck_icn_p_csi", "ck_icn_apb4", 0, GATE_CSI, SEC_RIFSC(86)),
	STM32_GATE(CK_KER_CSI, "ck_ker_csi", "ck_flexgen_29", 0, GATE_CSI, SEC_RIFSC(86)),
	STM32_GATE(CK_KER_CSITXESC, "ck_ker_csitxesc", "ck_flexgen_30", 0, GATE_CSI,
		   SEC_RIFSC(86)),

	/* CSI-PHY */
	STM32_GATE(CK_KER_CSIPHY, "ck_ker_csiphy", "ck_flexgen_31", 0, GATE_CSI,
		   SEC_RIFSC(86)),

	/* DCMIPP */
	STM32_GATE(CK_BUS_DCMIPP, "ck_icn_p_dcmipp", "ck_icn_apb4", 0, GATE_DCMIPP,
		   SEC_RIFSC(87)),

	/* CRC */
	STM32_GATE(CK_BUS_CRC, "ck_icn_p_crc", "ck_icn_ls_mcu", 0, GATE_CRC, SEC_RIFSC(109)),

	/* CRYP */
	STM32_GATE(CK_BUS_CRYP1, "ck_icn_p_cryp1", "ck_icn_ls_mcu", 0, GATE_CRYP1,
		   SEC_RIFSC(98)),
	STM32_GATE(CK_BUS_CRYP2, "ck_icn_p_cryp2", "ck_icn_ls_mcu", 0, GATE_CRYP2,
		   SEC_RIFSC(99)),

	/* DBG & TRACE*/
	/* Trace and debug clocks are managed by SCMI */

	/* Display subsystem */
	/* LTDC */
	STM32_GATE(CK_BUS_LTDC, "ck_icn_p_ltdc", "ck_icn_apb4", 0, GATE_LTDC, SEC_RIFSC(80)),
	STM32_GATE(CK_KER_LTDC, "ck_ker_ltdc", "ck_flexgen_27", CLK_SET_RATE_PARENT, GATE_LTDC,
		   SEC_RIFSC(80)),

	/* DTS */
	STM32_COMPOSITE_NODIV(CK_KER_DTS, "ck_ker_dts", 0, SEC_RIFSC(107), GATE_DTS, MUX_DTS),

	/* ETHERNET */
	STM32_GATE(CK_BUS_ETH1, "ck_icn_p_eth1", "ck_icn_ls_mcu", 0, GATE_ETH1, SEC_RIFSC(60)),
	STM32_GATE(CK_ETH1_STP, "ck_ker_eth1stp", "ck_icn_ls_mcu", 0, GATE_ETH1STP,
		   SEC_RIFSC(60)),
	STM32_GATE(CK_KER_ETH1, "ck_ker_eth1", "ck_flexgen_54", 0, GATE_ETH1, SEC_RIFSC(60)),
	STM32_GATE(CK_KER_ETH1, "ck_ker_eth1ptp", "ck_flexgen_56", 0, GATE_ETH1, SEC_RIFSC(60)),
	STM32_GATE(CK_ETH1_MAC, "ck_ker_eth1mac", "ck_icn_ls_mcu", 0, GATE_ETH1MAC,
		   SEC_RIFSC(60)),
	STM32_GATE(CK_ETH1_TX, "ck_ker_eth1tx", "ck_icn_ls_mcu", 0, GATE_ETH1TX, SEC_RIFSC(60)),
	STM32_GATE(CK_ETH1_RX, "ck_ker_eth1rx", "ck_icn_ls_mcu", 0, GATE_ETH1RX, SEC_RIFSC(60)),

	STM32_GATE(CK_BUS_ETH2, "ck_icn_p_eth2", "ck_icn_ls_mcu", 0, GATE_ETH2, SEC_RIFSC(61)),
	STM32_GATE(CK_ETH2_STP, "ck_ker_eth2stp", "ck_icn_ls_mcu", 0, GATE_ETH2STP,
		   SEC_RIFSC(61)),
	STM32_GATE(CK_KER_ETH2, "ck_ker_eth2", "ck_flexgen_54", 0, GATE_ETH2, SEC_RIFSC(61)),
	STM32_GATE(CK_KER_ETH2, "ck_ker_eth2ptp", "ck_flexgen_56", 0, GATE_ETH2, SEC_RIFSC(61)),
	STM32_GATE(CK_ETH2_MAC, "ck_ker_eth2mac", "ck_icn_ls_mcu", 0, GATE_ETH2MAC,
		   SEC_RIFSC(61)),
	STM32_GATE(CK_ETH2_TX, "ck_ker_eth2tx", "ck_icn_ls_mcu", 0, GATE_ETH2TX, SEC_RIFSC(61)),
	STM32_GATE(CK_ETH2_RX, "ck_ker_eth2rx", "ck_icn_ls_mcu", 0, GATE_ETH2RX, SEC_RIFSC(61)),

	/* FDCAN */
	STM32_GATE(CK_BUS_FDCAN, "ck_icn_p_fdcan", "ck_icn_apb2", 0, GATE_FDCAN, SEC_RIFSC(56)),
	STM32_GATE(CK_KER_FDCAN, "ck_ker_fdcan", "ck_flexgen_26", 0, GATE_FDCAN, SEC_RIFSC(56)),

	/* HASH */
	STM32_GATE(CK_BUS_HASH1, "ck_icn_p_hash1", "ck_icn_ls_mcu", 0, GATE_HASH1, SEC_RIFSC(96)),
	STM32_GATE(CK_BUS_HASH2, "ck_icn_p_hash2", "ck_icn_ls_mcu", 0, GATE_HASH2, SEC_RIFSC(97)),

	/* HDP */
	STM32_GATE(CK_BUS_HDP, "ck_icn_p_hdp", "ck_icn_apb3", 0, GATE_HDP, SEC_RIFSC(57)),

	/* I2C */
	STM32_GATE(CK_KER_I2C1, "ck_ker_i2c1", "ck_flexgen_13", 0, GATE_I2C1, SEC_RIFSC(41)),
	STM32_GATE(CK_KER_I2C2, "ck_ker_i2c2", "ck_flexgen_13", 0, GATE_I2C2, SEC_RIFSC(42)),
	STM32_GATE(CK_KER_I2C3, "ck_ker_i2c3", "ck_flexgen_38", 0, GATE_I2C3, SEC_RIFSC(43)),

	/* I3C */
	STM32_GATE(CK_KER_I3C1, "ck_ker_i3c1", "ck_flexgen_14", 0, GATE_I3C1, SEC_RIFSC(114)),
	STM32_GATE(CK_KER_I3C2, "ck_ker_i3c2", "ck_flexgen_14", 0, GATE_I3C2, SEC_RIFSC(115)),
	STM32_GATE(CK_KER_I3C3, "ck_ker_i3c3", "ck_flexgen_36", 0, GATE_I3C3, SEC_RIFSC(116)),

	/* IWDG */
	STM32_GATE(CK_BUS_IWDG1, "ck_icn_p_iwdg1", "ck_icn_apb3", 0, GATE_IWDG1, SEC_RIFSC(100)),
	STM32_GATE(CK_BUS_IWDG2, "ck_icn_p_iwdg2", "ck_icn_apb3", 0, GATE_IWDG2, SEC_RIFSC(101)),
	STM32_GATE(CK_BUS_IWDG3, "ck_icn_p_iwdg3", "ck_icn_apb3", 0, GATE_IWDG3, SEC_RIFSC(102)),
	STM32_GATE(CK_BUS_IWDG4, "ck_icn_p_iwdg4", "ck_icn_apb3", 0, GATE_IWDG4, SEC_RIFSC(103)),

	/* LPTIM */
	STM32_GATE(CK_KER_LPTIM1, "ck_ker_lptim1", "ck_flexgen_07", 0, GATE_LPTIM1,
		   SEC_RIFSC(17)),
	STM32_GATE(CK_KER_LPTIM2, "ck_ker_lptim2", "ck_flexgen_07", 0, GATE_LPTIM2,
		   SEC_RIFSC(18)),
	STM32_GATE(CK_KER_LPTIM3, "ck_ker_lptim3", "ck_flexgen_40", 0, GATE_LPTIM3,
		   SEC_RIFSC(19)),
	STM32_GATE(CK_KER_LPTIM4, "ck_ker_lptim4", "ck_flexgen_41", 0, GATE_LPTIM4,
		   SEC_RIFSC(20)),
	STM32_GATE(CK_KER_LPTIM5, "ck_ker_lptim5", "ck_flexgen_42", 0, GATE_LPTIM5,
		   SEC_RIFSC(21)),

	/* LPUART */
	STM32_GATE(CK_KER_LPUART1, "ck_ker_lpuart1", "ck_flexgen_39", 0, GATE_LPUART1,
		   SEC_RIFSC(40)),

	/* MCO1 & MCO2 */
	STM32_COMPOSITE_NODIV(CK_MCO1, "ck_mco1", 0, SEC_RIFRCC(MCO1), GATE_MCO1, MUX_MCO1),
	STM32_COMPOSITE_NODIV(CK_MCO2, "ck_mco2", 0, SEC_RIFRCC(MCO2), GATE_MCO2, MUX_MCO2),

	/* MDF */
	STM32_GATE(CK_KER_MDF1, "ck_ker_mdf1", "ck_flexgen_21", 0, GATE_MDF1, SEC_RIFSC(54)),

	/* PKA */
	STM32_GATE(CK_BUS_PKA, "ck_icn_p_pka", "ck_icn_ls_mcu", 0, GATE_PKA, SEC_RIFSC(94)),

	/* RNG */
	STM32_GATE(CK_BUS_RNG1, "ck_icn_p_rng1", "ck_icn_ls_mcu", CLK_IGNORE_UNUSED, GATE_RNG1,
		   SEC_RIFSC(92)),
	STM32_GATE(CK_BUS_RNG2, "ck_icn_p_rng2", "ck_icn_ls_mcu", CLK_IGNORE_UNUSED, GATE_RNG2,
		   SEC_RIFSC(93)),

	/* SAES */
	STM32_GATE(CK_BUS_SAES, "ck_icn_p_saes", "ck_icn_ls_mcu", 0, GATE_SAES, SEC_RIFSC(95)),

	/* SAI [1..4] */
	STM32_GATE(CK_BUS_SAI1, "ck_icn_p_sai1", "ck_icn_apb2", 0, GATE_SAI1, SEC_RIFSC(49)),
	STM32_GATE(CK_BUS_SAI2, "ck_icn_p_sai2", "ck_icn_apb2", 0, GATE_SAI2, SEC_RIFSC(50)),
	STM32_GATE(CK_BUS_SAI3, "ck_icn_p_sai3", "ck_icn_apb2", 0, GATE_SAI3, SEC_RIFSC(51)),
	STM32_GATE(CK_BUS_SAI4, "ck_icn_p_sai4", "ck_icn_apb2", 0, GATE_SAI4, SEC_RIFSC(52)),
	STM32_GATE(CK_KER_SAI1, "ck_ker_sai1", "ck_flexgen_22", 0, GATE_SAI1, SEC_RIFSC(49)),
	STM32_GATE(CK_KER_SAI2, "ck_ker_sai2", "ck_flexgen_23", 0, GATE_SAI2, SEC_RIFSC(50)),
	STM32_GATE(CK_KER_SAI3, "ck_ker_sai3", "ck_flexgen_24", 0, GATE_SAI3, SEC_RIFSC(51)),
	STM32_GATE(CK_KER_SAI4, "ck_ker_sai4", "ck_flexgen_25", 0, GATE_SAI4, SEC_RIFSC(52)),

	/* SDMMC */
	STM32_GATE(CK_KER_SDMMC1, "ck_ker_sdmmc1", "ck_flexgen_51", 0, GATE_SDMMC1,
		   SEC_RIFSC(76)),
	STM32_GATE(CK_KER_SDMMC2, "ck_ker_sdmmc2", "ck_flexgen_52", 0, GATE_SDMMC2,
		   SEC_RIFSC(77)),
	STM32_GATE(CK_KER_SDMMC3, "ck_ker_sdmmc3", "ck_flexgen_53", 0, GATE_SDMMC3,
		   SEC_RIFSC(78)),

	/* SERC */
	STM32_GATE(CK_BUS_SERC, "ck_icn_p_serc", "ck_icn_apb3", 0, GATE_SERC, SEC_RIFSC(110)),

	/* SPDIF */
	STM32_GATE(CK_KER_SPDIFRX, "ck_ker_spdifrx", "ck_flexgen_12", 0, GATE_SPDIFRX,
		   SEC_RIFSC(30)),

	/* SPI */
	STM32_GATE(CK_KER_SPI1, "ck_ker_spi1", "ck_flexgen_16", 0, GATE_SPI1, SEC_RIFSC(22)),
	STM32_GATE(CK_KER_SPI2, "ck_ker_spi2", "ck_flexgen_10", 0, GATE_SPI2, SEC_RIFSC(23)),
	STM32_GATE(CK_KER_SPI3, "ck_ker_spi3", "ck_flexgen_11", 0, GATE_SPI3, SEC_RIFSC(24)),
	STM32_GATE(CK_KER_SPI4, "ck_ker_spi4", "ck_flexgen_17", 0, GATE_SPI4, SEC_RIFSC(25)),
	STM32_GATE(CK_KER_SPI5, "ck_ker_spi5", "ck_flexgen_17", 0, GATE_SPI5, SEC_RIFSC(26)),
	STM32_GATE(CK_KER_SPI6, "ck_ker_spi6", "ck_flexgen_37", 0, GATE_SPI6, SEC_RIFSC(27)),

	/* STGEN */
	STM32_GATE(CK_KER_STGEN, "ck_ker_stgen", "ck_flexgen_33", CLK_IGNORE_UNUSED, GATE_STGEN,
		   SEC_RIFSC(73)),

	/* Timers */
	STM32_GATE(CK_KER_TIM2, "ck_ker_tim2", "timg1_ck", 0, GATE_TIM2, SEC_RIFSC(1)),
	STM32_GATE(CK_KER_TIM3, "ck_ker_tim3", "timg1_ck", 0, GATE_TIM3, SEC_RIFSC(2)),
	STM32_GATE(CK_KER_TIM4, "ck_ker_tim4", "timg1_ck", 0, GATE_TIM4, SEC_RIFSC(3)),
	STM32_GATE(CK_KER_TIM5, "ck_ker_tim5", "timg1_ck", 0, GATE_TIM5, SEC_RIFSC(4)),
	STM32_GATE(CK_KER_TIM6, "ck_ker_tim6", "timg1_ck", 0, GATE_TIM6, SEC_RIFSC(5)),
	STM32_GATE(CK_KER_TIM7, "ck_ker_tim7", "timg1_ck", 0, GATE_TIM7, SEC_RIFSC(6)),
	STM32_GATE(CK_KER_TIM10, "ck_ker_tim10", "timg1_ck", 0, GATE_TIM10, SEC_RIFSC(8)),
	STM32_GATE(CK_KER_TIM11, "ck_ker_tim11", "timg1_ck", 0, GATE_TIM11, SEC_RIFSC(9)),
	STM32_GATE(CK_KER_TIM12, "ck_ker_tim12", "timg1_ck", 0, GATE_TIM12, SEC_RIFSC(10)),
	STM32_GATE(CK_KER_TIM13, "ck_ker_tim13", "timg1_ck", 0, GATE_TIM13, SEC_RIFSC(11)),
	STM32_GATE(CK_KER_TIM14, "ck_ker_tim14", "timg1_ck", 0, GATE_TIM14, SEC_RIFSC(12)),

	STM32_GATE(CK_KER_TIM1, "ck_ker_tim1", "timg2_ck", 0, GATE_TIM1, SEC_RIFSC(0)),
	STM32_GATE(CK_KER_TIM8, "ck_ker_tim8", "timg2_ck", 0, GATE_TIM8, SEC_RIFSC(7)),
	STM32_GATE(CK_KER_TIM15, "ck_ker_tim15", "timg2_ck", 0, GATE_TIM15, SEC_RIFSC(13)),
	STM32_GATE(CK_KER_TIM16, "ck_ker_tim16", "timg2_ck", 0, GATE_TIM16, SEC_RIFSC(14)),
	STM32_GATE(CK_KER_TIM17, "ck_ker_tim17", "timg2_ck", 0, GATE_TIM17, SEC_RIFSC(15)),

	/* UART/USART */
	STM32_GATE(CK_KER_USART2, "ck_ker_usart2", "ck_flexgen_08", 0, GATE_USART2,
		   SEC_RIFSC(32)),
	STM32_GATE(CK_KER_UART4, "ck_ker_uart4", "ck_flexgen_08", 0, GATE_UART4,
		   SEC_RIFSC(34)),
	STM32_GATE(CK_KER_USART3, "ck_ker_usart3", "ck_flexgen_09", 0, GATE_USART3,
		   SEC_RIFSC(33)),
	STM32_GATE(CK_KER_UART5, "ck_ker_uart5", "ck_flexgen_09", 0, GATE_UART5,
		   SEC_RIFSC(35)),
	STM32_GATE(CK_KER_USART1, "ck_ker_usart1", "ck_flexgen_18", 0, GATE_USART1,
		   SEC_RIFSC(31)),
	STM32_GATE(CK_KER_USART6, "ck_ker_usart6", "ck_flexgen_19", 0, GATE_USART6,
		   SEC_RIFSC(36)),
	STM32_GATE(CK_KER_UART7, "ck_ker_uart7", "ck_flexgen_20", 0, GATE_UART7,
		   SEC_RIFSC(37)),

	/* USB2PHY1 */
	STM32_COMPOSITE_NODIV(CK_KER_USB2PHY1, "ck_ker_usb2phy1", 0, SEC_RIFSC(63),
			      GATE_USB2PHY1, MUX_USB2PHY1),

	/* USB2H */
	STM32_GATE(CK_BUS_USBHOHCI, "ck_icn_m_usbhohci", "ck_icn_hsl", 0, GATE_USBH,
		   SEC_RIFSC(63)),
	STM32_GATE(CK_BUS_USBHEHCI, "ck_icn_m_usbhehci", "ck_icn_hsl", 0, GATE_USBH,
		   SEC_RIFSC(63)),

	/* USBOTG */
	STM32_GATE(CK_BUS_OTG, "ck_icn_m_otg", "ck_icn_hsl", 0, GATE_USBOTG,
		   SEC_RIFSC(66)),

	/* USB2PHY2 */
	STM32_COMPOSITE_NODIV(CK_KER_USB2PHY2EN, "ck_ker_usb2phy2_en", 0, SEC_RIFSC(66),
			      GATE_USB2PHY2, MUX_USB2PHY2),

	/* VREF */
	STM32_GATE(CK_BUS_VREF, "ck_icn_p_vref", "ck_icn_apb3", 0, RCC_VREFCFGR,
		   SEC_RIFSC(106)),

	/* WWDG */
	STM32_GATE(CK_BUS_WWDG1, "ck_icn_p_wwdg1", "ck_icn_apb3", 0, GATE_WWDG1,
		   SEC_RIFSC(104)),
};

static const struct stm32_clock_match_data stm32mp21_data = {
	.tab_clocks	= stm32mp21_clock_cfg,
	.num_clocks	= ARRAY_SIZE(stm32mp21_clock_cfg),
	.clock_data = &(const struct clk_stm32_clock_data) {
		.num_gates	= ARRAY_SIZE(stm32mp21_gates),
		.gates		= stm32mp21_gates,
		.muxes		= stm32mp21_muxes,
	},
	.check_security = stm32mp21_check_security,

};

static int stm32mp21_clk_probe(struct udevice *dev)
{
	fdt_addr_t base = dev_read_addr(dev->parent);
	struct udevice *scmi;

	if (base == FDT_ADDR_T_NONE)
		return -EINVAL;

	/* force SCMI probe to register all SCMI clocks */
	uclass_get_device_by_driver(UCLASS_CLK, DM_DRIVER_GET(scmi_clock), &scmi);

	stm32_rcc_init(dev, &stm32mp21_data);

	return 0;
}

U_BOOT_DRIVER(stm32mp21_clock) = {
	.name = "stm32mp21_clk",
	.id = UCLASS_CLK,
	.ops = &stm32_clk_ops,
	.priv_auto = sizeof(struct stm32mp_rcc_priv),
	.probe = stm32mp21_clk_probe,
};
