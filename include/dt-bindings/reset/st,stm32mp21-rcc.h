/* SPDX-License-Identifier: GPL-2.0-only OR BSD-3-Clause */
/*
 * Copyright (C) STMicroelectronics 2024 - All Rights Reserved
 * Author(s): Gabriel Fernandez <gabriel.fernandez@foss.st.com>
 */

#ifndef _DT_BINDINGS_STM32MP21_RESET_H_
#define _DT_BINDINGS_STM32MP21_RESET_H_

#define TIM1_R		0
#define TIM2_R		1
#define TIM3_R		2
#define TIM4_R		3
#define TIM5_R		4
#define TIM6_R		5
#define TIM7_R		6
#define TIM8_R		7
#define TIM10_R		8
#define TIM11_R		9
#define TIM12_R		10
#define TIM13_R		11
#define TIM14_R		12
#define TIM15_R		13
#define TIM16_R		14
#define TIM17_R		15
#define LPTIM1_R	16
#define LPTIM2_R	17
#define LPTIM3_R	18
#define LPTIM4_R	19
#define LPTIM5_R	20
#define SPI1_R		21
#define SPI2_R		22
#define SPI3_R		23
#define SPI4_R		24
#define SPI5_R		25
#define SPI6_R		26
#define SPDIFRX_R	27
#define USART1_R	28
#define USART2_R	29
#define USART3_R	30
#define UART4_R		31
#define UART5_R		32
#define USART6_R	33
#define UART7_R		34
#define LPUART1_R	35
#define I2C1_R		36
#define I2C2_R		37
#define I2C3_R		38
#define SAI1_R		39
#define SAI2_R		40
#define SAI3_R		41
#define SAI4_R		42
#define MDF1_R		43
#define FDCAN_R		44
#define HDP_R		45
#define ADC1_R		46
#define ADC2_R		47
#define ETH1_R		48
#define ETH2_R		49
#define USBH_R		50
#define USB2PHY1_R	51
#define USB2PHY2_R	52
#define SDMMC1_R	53
#define SDMMC1DLL_R	54
#define SDMMC2_R	55
#define SDMMC2DLL_R	56
#define SDMMC3_R	57
#define SDMMC3DLL_R	58
#define LTDC_R		59
#define CSI_R		60
#define DCMIPP_R	61
#define DCMIPSSI_R	62
#define WWDG1_R		63
#define VREF_R		64
#define DTS_R		65
#define CRC_R		66
#define SERC_R		67
#define I3C1_R		68
#define I3C2_R		69
#define I3C3_R		70
#define IWDG2_KER_R	71
#define IWDG4_KER_R	72
#define RNG1_R		73
#define RNG2_R		74
#define PKA_R		75
#define SAES_R		76
#define HASH1_R		77
#define HASH2_R		78
#define CRYP1_R		79
#define CRYP2_R		80
#define OSPI1_R		81
#define OSPI1DLL_R	82
#define OTG_R		83
#define FMC_R		84
#define DBG_R		85
#define GPIOA_R		86
#define GPIOB_R		87
#define GPIOC_R		88
#define GPIOD_R		89
#define GPIOE_R		90
#define GPIOF_R		91
#define GPIOG_R		92
#define GPIOH_R		93
#define GPIOI_R		94
#define GPIOZ_R		95
#define HPDMA1_R	96
#define HPDMA2_R	97
#define HPDMA3_R	98
#define IPCC1_R		99
#define C2_HOLDBOOT_R	100
#define C1_HOLDBOOT_R	101
#define C1_R		102
#define C1P1POR_R	103
#define C1P1_R		104
#define C2_R		105
#define SYS_R		106
#define VSW_R		107
#define C1MS_R		108
#define DDRCP_R		109
#define DDRCAPB_R	110
#define DDRPHYCAPB_R	111
#define DDRCFG_R	112
#define DDR_R		113
#define DDRPERFM_R	114
#define CCB_R		115
#define IWDG1_SYS_R	116
#define IWDG2_SYS_R	117
#define IWDG3_SYS_R	118
#define IWDG4_SYS_R	119

#define STM32MP21_LAST_RESET	120

#define RST_SCMI_C1_R		0
#define RST_SCMI_C2_R		1
#define RST_SCMI_C1_HOLDBOOT_R	2
#define RST_SCMI_C2_HOLDBOOT_R	3
#define RST_SCMI_FMC		4
#define RST_SCMI_OSPI1		5
#define RST_SCMI_OSPI1DLL	6

#endif /* _DT_BINDINGS_STM32MP2_RESET_H_ */