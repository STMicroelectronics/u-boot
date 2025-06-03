// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/*
 * Copyright (C) 2018, STMicroelectronics - All Rights Reserved
 */

#define LOG_CATEGORY LOGC_BOARD

#include <common.h>
#include <adc.h>
#include <bootm.h>
#include <clk.h>
#include <config.h>
#include <dm.h>
#include <efi_loader.h>
#include <env.h>
#include <env_internal.h>
#include <fdt_simplefb.h>
#include <fdt_support.h>
#include <g_dnl.h>
#include <generic-phy.h>
#include <hang.h>
#include <i2c.h>
#include <regmap.h>
#include <init.h>
#include <led.h>
#include <log.h>
#include <malloc.h>
#include <misc.h>
#include <net.h>
#include <netdev.h>
#include <phy.h>
#include <remoteproc.h>
#include <reset.h>
#include <syscon.h>
#include <typec.h>
#include <usb.h>
#include <watchdog.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/arch/stm32.h>
#include <asm/arch/sys_proto.h>
#include <dm/device.h>
#include <dm/device-internal.h>
#include <dm/ofnode.h>
#include <jffs2/load_kernel.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/iopoll.h>
#include <linux/stringify.h>
#include <power/regulator.h>
#include <usb/dwc2_udc.h>

/* SYSCFG registers */
#define SYSCFG_BOOTR		0x00
#define SYSCFG_PMCSETR		0x04
#define SYSCFG_IOCTRLSETR	0x18
#define SYSCFG_ICNR		0x1C
#define SYSCFG_CMPCR		0x20
#define SYSCFG_CMPENSETR	0x24
#define SYSCFG_PMCCLRR		0x08
#define SYSCFG_MP13_PMCCLRR	0x44

#define SYSCFG_BOOTR_BOOT_MASK		GENMASK(2, 0)
#define SYSCFG_BOOTR_BOOTPD_SHIFT	4

#define SYSCFG_IOCTRLSETR_HSLVEN_TRACE		BIT(0)
#define SYSCFG_IOCTRLSETR_HSLVEN_QUADSPI	BIT(1)
#define SYSCFG_IOCTRLSETR_HSLVEN_ETH		BIT(2)
#define SYSCFG_IOCTRLSETR_HSLVEN_SDMMC		BIT(3)
#define SYSCFG_IOCTRLSETR_HSLVEN_SPI		BIT(4)

#define SYSCFG_CMPCR_SW_CTRL		BIT(1)
#define SYSCFG_CMPCR_READY		BIT(8)

#define SYSCFG_CMPENSETR_MPU_EN		BIT(0)

#define GOODIX_REG_ID		0x8140
#define GOODIX_ID_LEN		4

#define USB_LOW_THRESHOLD_UV		200000
#define USB_WARNING_LOW_THRESHOLD_UV	660000
#define USB_START_LOW_THRESHOLD_UV	1230000
#define USB_START_HIGH_THRESHOLD_UV	2150000

#if IS_ENABLED(CONFIG_EFI_HAVE_CAPSULE_SUPPORT)
struct efi_fw_image fw_images[1];

struct efi_capsule_update_info update_info = {
	.num_images = ARRAY_SIZE(fw_images),
	.images = fw_images,
};

#endif /* EFI_HAVE_CAPSULE_SUPPORT */

int board_early_init_f(void)
{
	/* nothing to do, only used in SPL */
	return 0;
}

int checkboard(void)
{
	int ret;
	char *mode;
	u32 otp;
	struct udevice *dev;
	const char *fdt_compat;
	int fdt_compat_len;

	if (IS_ENABLED(CONFIG_TFABOOT)) {
		if (IS_ENABLED(CONFIG_STM32MP15X_STM32IMAGE))
			mode = "trusted - stm32image";
		else
			mode = "trusted";
	} else {
		mode = "basic";
	}

	fdt_compat = ofnode_get_property(ofnode_root(), "compatible",
					 &fdt_compat_len);

	log_info("Board: stm32mp1 in %s mode (%s)\n", mode,
		 fdt_compat && fdt_compat_len ? fdt_compat : "");

	/* display the STMicroelectronics board identification */
	if (IS_ENABLED(CONFIG_CMD_STBOARD)) {
		ret = uclass_get_device_by_driver(UCLASS_MISC,
						  DM_DRIVER_GET(stm32mp_bsec),
						  &dev);
		if (!ret)
			ret = misc_read(dev, STM32_BSEC_SHADOW(BSEC_OTP_BOARD),
					&otp, sizeof(otp));
		if (ret > 0 && otp)
			log_info("Board: MB%04x Var%d.%d Rev.%c-%02d\n",
				 otp >> 16,
				 (otp >> 12) & 0xF,
				 (otp >> 4) & 0xF,
				 ((otp >> 8) & 0xF) - 1 + 'A',
				 otp & 0xF);
	}

	return 0;
}

static void board_key_check(void)
{
	ofnode node;
	struct gpio_desc gpio;
	enum forced_boot_mode boot_mode = BOOT_NORMAL;

	if (!IS_ENABLED(CONFIG_FASTBOOT) && !IS_ENABLED(CONFIG_CMD_STM32PROG))
		return;

	node = ofnode_path("/config");
	if (!ofnode_valid(node)) {
		log_debug("no /config node?\n");
		return;
	}
	if (IS_ENABLED(CONFIG_FASTBOOT)) {
		if (gpio_request_by_name_nodev(node, "st,fastboot-gpios", 0,
					       &gpio, GPIOD_IS_IN)) {
			log_debug("could not find a /config/st,fastboot-gpios\n");
		} else {
			udelay(20);
			if (dm_gpio_get_value(&gpio)) {
				log_notice("Fastboot key pressed, ");
				boot_mode = BOOT_FASTBOOT;
			}

			dm_gpio_free(NULL, &gpio);
		}
	}
	if (IS_ENABLED(CONFIG_CMD_STM32PROG)) {
		if (gpio_request_by_name_nodev(node, "st,stm32prog-gpios", 0,
					       &gpio, GPIOD_IS_IN)) {
			log_debug("could not find a /config/st,stm32prog-gpios\n");
		} else {
			udelay(20);
			if (dm_gpio_get_value(&gpio)) {
				log_notice("STM32Programmer key pressed, ");
				boot_mode = BOOT_STM32PROG;
			}
			dm_gpio_free(NULL, &gpio);
		}
	}
	if (boot_mode != BOOT_NORMAL) {
		log_notice("entering download mode...\n");
		clrsetbits_le32(TAMP_BOOT_CONTEXT,
				TAMP_BOOT_FORCED_MASK,
				boot_mode);
	}
}

static int typec_usb_cable_connected(void)
{
	struct udevice *dev;
	int ret;
	u8 connector = 0;

	ret = uclass_get_device(UCLASS_USB_TYPEC, 0, &dev);
	if (ret < 0)
		return ret;

	return (typec_is_attached(dev, connector) == TYPEC_ATTACHED) &&
	       (typec_get_data_role(dev, connector) == TYPEC_DEVICE);
}

int g_dnl_board_usb_cable_connected(void)
{
	struct udevice *dwc2_udc_otg;
	int ret;

	if (!IS_ENABLED(CONFIG_USB_GADGET_DWC2_OTG))
		return -ENODEV;

	/*
	 * In case of USB boot device is detected, consider USB cable is
	 * connected
	 */
	if ((get_bootmode() & TAMP_BOOT_DEVICE_MASK) == BOOT_SERIAL_USB)
		return true;

	/* if Type-C is present, it means DK1 or DK2 board */
	ret = typec_usb_cable_connected();
	if (ret >= 0)
		return ret;

	ret = uclass_get_device_by_driver(UCLASS_USB_GADGET_GENERIC,
					  DM_DRIVER_GET(dwc2_udc_otg),
					  &dwc2_udc_otg);
	if (ret) {
		log_debug("dwc2_udc_otg init failed\n");
		return ret;
	}

	return dwc2_udc_B_session_valid(dwc2_udc_otg);
}

#ifdef CONFIG_USB_GADGET_DOWNLOAD
#define STM32MP1_G_DNL_DFU_PRODUCT_NUM 0xdf11
#define STM32MP1_G_DNL_FASTBOOT_PRODUCT_NUM 0x0afb

int g_dnl_bind_fixup(struct usb_device_descriptor *dev, const char *name)
{
	if (IS_ENABLED(CONFIG_DFU_OVER_USB) &&
	    !strcmp(name, "usb_dnl_dfu"))
		put_unaligned(STM32MP1_G_DNL_DFU_PRODUCT_NUM, &dev->idProduct);
	else if (IS_ENABLED(CONFIG_FASTBOOT) &&
		 !strcmp(name, "usb_dnl_fastboot"))
		put_unaligned(STM32MP1_G_DNL_FASTBOOT_PRODUCT_NUM,
			      &dev->idProduct);
	else
		put_unaligned(CONFIG_USB_GADGET_PRODUCT_NUM, &dev->idProduct);

	return 0;
}
#endif /* CONFIG_USB_GADGET_DOWNLOAD */

static int get_led(struct udevice **dev, char *led_string)
{
	const char *led_name;
	int ret;

	led_name = ofnode_conf_read_str(led_string);
	if (!led_name) {
		log_debug("could not find %s config string\n", led_string);
		return -ENOENT;
	}
	ret = led_get_by_label(led_name, dev);
	if (ret) {
		log_debug("get=%d\n", ret);
		return ret;
	}

	return 0;
}

static int setup_led(enum led_state_t cmd)
{
	struct udevice *dev;
	int ret;

	if (!CONFIG_IS_ENABLED(LED))
		return 0;

	ret = get_led(&dev, "u-boot,boot-led");
	if (ret)
		return ret;

	ret = led_set_state(dev, cmd);
	return ret;
}

static void __maybe_unused led_error_blink(u32 nb_blink)
{
	int ret;
	struct udevice *led;
	u32 i;

	if (!nb_blink)
		return;

	if (CONFIG_IS_ENABLED(LED)) {
		ret = get_led(&led, "u-boot,error-led");
		if (!ret) {
			/* make u-boot,error-led blinking */
			/* if U32_MAX and 125ms interval, for 17.02 years */
			for (i = 0; i < 2 * nb_blink; i++) {
				led_set_state(led, LEDST_TOGGLE);
				mdelay(125);
				schedule();
			}
			led_set_state(led, LEDST_ON);
		}
	}

	/* infinite: the boot process must be stopped */
	if (nb_blink == U32_MAX)
		hang();
}

static int adc_measurement(ofnode node, int adc_count, int *min_uV, int *max_uV)
{
	struct ofnode_phandle_args adc_args;
	struct udevice *adc;
	unsigned int raw;
	int ret, uV;
	int i;

	for (i = 0; i < adc_count; i++) {
		if (ofnode_parse_phandle_with_args(node, "st,adc_usb_pd",
						   "#io-channel-cells", 0, i,
						   &adc_args)) {
			log_debug("can't find /config/st,adc_usb_pd\n");
			return 0;
		}

		ret = uclass_get_device_by_ofnode(UCLASS_ADC, adc_args.node,
						  &adc);

		if (ret) {
			log_err("Can't get adc device(%d)\n", ret);
			return ret;
		}

		ret = adc_channel_single_shot(adc->name, adc_args.args[0],
					      &raw);
		if (ret) {
			log_err("single shot failed for %s[%d]!\n",
				adc->name, adc_args.args[0]);
			return ret;
		}
		/* Convert to uV */
		if (!adc_raw_to_uV(adc, raw, &uV)) {
			if (uV > *max_uV)
				*max_uV = uV;
			if (uV < *min_uV)
				*min_uV = uV;
			log_debug("%s[%02d] = %u, %d uV\n",
				  adc->name, adc_args.args[0], raw, uV);
		} else {
			log_err("Can't get uV value for %s[%d]\n",
				adc->name, adc_args.args[0]);
		}
	}

	return 0;
}

static int board_check_usb_power(void)
{
	ofnode node;
	int max_uV = 0;
	int min_uV = USB_START_HIGH_THRESHOLD_UV;
	int adc_count, ret;
	u32 nb_blink;
	u8 i;

	node = ofnode_path("/config");
	if (!ofnode_valid(node)) {
		log_debug("no /config node?\n");
		return -ENOENT;
	}

	/*
	 * Retrieve the ADC channels devices and get measurement
	 * for each of them
	 */
	adc_count = ofnode_count_phandle_with_args(node, "st,adc_usb_pd",
						   "#io-channel-cells", 0);
	if (adc_count < 0) {
		if (adc_count == -ENOENT)
			return 0;

		log_err("Can't find adc channel (%d)\n", adc_count);

		return adc_count;
	}

	/* perform maximum of 2 ADC measurements to detect power supply current */
	for (i = 0; i < 2; i++) {
		ret = adc_measurement(node, adc_count, &min_uV, &max_uV);
		if (ret)
			return ret;

		/*
		 * If highest value is inside 1.23 Volts and 2.10 Volts, that means
		 * board is plugged on an USB-C 3A power supply and boot process can
		 * continue.
		 */
		if (max_uV > USB_START_LOW_THRESHOLD_UV &&
		    max_uV <= USB_START_HIGH_THRESHOLD_UV &&
		    min_uV <= USB_LOW_THRESHOLD_UV)
			return 0;

		if (i == 0) {
			log_err("Previous ADC measurements was not the one expected, retry in 20ms\n");
			mdelay(20);  /* equal to max tPDDebounce duration (min 10ms - max 20ms) */
		}
	}

	log_notice("****************************************************\n");
	/*
	 * If highest and lowest value are either both below
	 * USB_LOW_THRESHOLD_UV or both above USB_LOW_THRESHOLD_UV, that
	 * means USB TYPE-C is in unattached mode, this is an issue, make
	 * u-boot,error-led blinking and stop boot process.
	 */
	if ((max_uV > USB_LOW_THRESHOLD_UV &&
	     min_uV > USB_LOW_THRESHOLD_UV) ||
	     (max_uV <= USB_LOW_THRESHOLD_UV &&
	     min_uV <= USB_LOW_THRESHOLD_UV)) {
		log_notice("* ERROR USB TYPE-C connection in unattached mode   *\n");
		log_notice("* Check that USB TYPE-C cable is correctly plugged *\n");
		/* with 125ms interval, led will blink for 17.02 years ....*/
		nb_blink = U32_MAX;
	}

	if (max_uV > USB_LOW_THRESHOLD_UV &&
	    max_uV <= USB_WARNING_LOW_THRESHOLD_UV &&
	    min_uV <= USB_LOW_THRESHOLD_UV) {
		log_notice("*        WARNING 500mA power supply detected       *\n");
		nb_blink = 2;
	}

	if (max_uV > USB_WARNING_LOW_THRESHOLD_UV &&
	    max_uV <= USB_START_LOW_THRESHOLD_UV &&
	    min_uV <= USB_LOW_THRESHOLD_UV) {
		log_notice("*       WARNING 1.5A power supply detected        *\n");
		nb_blink = 3;
	}

	/*
	 * If highest value is above 2.15 Volts that means that the USB TypeC
	 * supplies more than 3 Amp, this is not compliant with TypeC specification
	 */
	if (max_uV > USB_START_HIGH_THRESHOLD_UV) {
		log_notice("*      USB TYPE-C charger not compliant with       *\n");
		log_notice("*                   specification                  *\n");
		log_notice("****************************************************\n\n");
		/* with 125ms interval, led will blink for 17.02 years ....*/
		nb_blink = U32_MAX;
	} else {
		log_notice("*     Current too low, use a 3A power supply!      *\n");
		log_notice("****************************************************\n\n");
	}

	led_error_blink(nb_blink);

	return 0;
}

static void sysconf_init(void)
{
	u8 *syscfg;
	struct udevice *pwr_dev;
	struct udevice *pwr_reg;
	struct udevice *dev;
	u32 otp = 0;
	int ret;
	u32 bootr, val;

	syscfg = (u8 *)syscon_get_first_range(STM32MP_SYSCON_SYSCFG);

	/* interconnect update : select master using the port 1 */
	/* LTDC = AXI_M9 */
	/* GPU  = AXI_M8 */
	/* today information is hardcoded in U-Boot */
	writel(BIT(9), syscfg + SYSCFG_ICNR);

	/* disable Pull-Down for boot pin connected to VDD */
	bootr = readl(syscfg + SYSCFG_BOOTR);
	bootr &= ~(SYSCFG_BOOTR_BOOT_MASK << SYSCFG_BOOTR_BOOTPD_SHIFT);
	bootr |= (bootr & SYSCFG_BOOTR_BOOT_MASK) << SYSCFG_BOOTR_BOOTPD_SHIFT;
	writel(bootr, syscfg + SYSCFG_BOOTR);

	/* High Speed Low Voltage Pad mode Enable for SPI, SDMMC, ETH, QSPI
	 * and TRACE. Needed above ~50MHz and conditioned by AFMUX selection.
	 * The customer will have to disable this for low frequencies
	 * or if AFMUX is selected but the function not used, typically for
	 * TRACE. Otherwise, impact on power consumption.
	 *
	 * WARNING:
	 *   enabling High Speed mode while VDD>2.7V
	 *   with the OTP product_below_2v5 (OTP 18, BIT 13)
	 *   erroneously set to 1 can damage the IC!
	 *   => U-Boot set the register only if VDD < 2.7V (in DT)
	 *      but this value need to be consistent with board design
	 */
	ret = uclass_get_device_by_driver(UCLASS_PMIC,
					  DM_DRIVER_GET(stm32mp_pwr_pmic),
					  &pwr_dev);
	if (!ret) {
		ret = uclass_get_device_by_driver(UCLASS_MISC,
						  DM_DRIVER_GET(stm32mp_bsec),
						  &dev);
		if (ret) {
			log_err("Can't find stm32mp_bsec driver\n");
			return;
		}

		ret = misc_read(dev, STM32_BSEC_SHADOW(18), &otp, 4);
		if (ret > 0)
			otp = otp & BIT(13);

		/* get VDD = vdd-supply */
		ret = device_get_supply_regulator(pwr_dev, "vdd-supply",
						  &pwr_reg);

		/* check if VDD is Low Voltage */
		if (!ret) {
			if (regulator_get_value(pwr_reg) < 2700000) {
				writel(SYSCFG_IOCTRLSETR_HSLVEN_TRACE |
				       SYSCFG_IOCTRLSETR_HSLVEN_QUADSPI |
				       SYSCFG_IOCTRLSETR_HSLVEN_ETH |
				       SYSCFG_IOCTRLSETR_HSLVEN_SDMMC |
				       SYSCFG_IOCTRLSETR_HSLVEN_SPI,
				       syscfg + SYSCFG_IOCTRLSETR);

				if (!otp)
					log_err("product_below_2v5=0: HSLVEN protected by HW\n");
			} else {
				if (otp)
					log_err("product_below_2v5=1: HSLVEN update is destructive, no update as VDD>2.7V\n");
			}
		} else {
			log_debug("VDD unknown");
		}
	}

	/* activate automatic I/O compensation
	 * warning: need to ensure CSI enabled and ready in clock driver
	 */
	writel(SYSCFG_CMPENSETR_MPU_EN, syscfg + SYSCFG_CMPENSETR);

	/* poll until ready (1s timeout) */
	ret = readl_poll_timeout(syscfg + SYSCFG_CMPCR, val,
				 val & SYSCFG_CMPCR_READY,
				 1000000);
	if (ret) {
		log_err("SYSCFG: I/O compensation failed, timeout.\n");
		led_error_blink(10);
	}

	clrbits_le32(syscfg + SYSCFG_CMPCR, SYSCFG_CMPCR_SW_CTRL);
}

static int board_stm32mp15x_dk2_init(void)
{
	ofnode node;
	struct gpio_desc hdmi, audio;
	int ret = 0;

	/* Fix to make I2C1 usable on DK2 for touchscreen usage in kernel */
	node = ofnode_path("/soc/i2c@40012000/hdmi-transmitter@39");
	if (!ofnode_valid(node)) {
		log_debug("no hdmi-transmitter@39 ?\n");
		return -ENOENT;
	}

	if (gpio_request_by_name_nodev(node, "reset-gpios", 0,
				       &hdmi, GPIOD_IS_OUT)) {
		log_debug("could not find reset-gpios\n");
		return -ENOENT;
	}

	node = ofnode_path("/soc/i2c@40012000/cs42l51@4a");
	if (!ofnode_valid(node)) {
		log_debug("no cs42l51@4a ?\n");
		return -ENOENT;
	}

	if (gpio_request_by_name_nodev(node, "reset-gpios", 0,
				       &audio, GPIOD_IS_OUT)) {
		log_debug("could not find reset-gpios\n");
		return -ENOENT;
	}

	/* before power up, insure that HDMI and AUDIO IC is under reset */
	ret = dm_gpio_set_value(&hdmi, 1);
	if (ret) {
		log_err("can't set_value for hdmi_nrst gpio");
		goto error;
	}
	ret = dm_gpio_set_value(&audio, 1);
	if (ret) {
		log_err("can't set_value for audio_nrst gpio");
		goto error;
	}

	/* power-up audio IC */
	regulator_autoset_by_name("v1v8_audio", NULL);

	/* power-up HDMI IC */
	regulator_autoset_by_name("v1v2_hdmi", NULL);
	regulator_autoset_by_name("v3v3_hdmi", NULL);

error:
	return ret;
}

static bool board_is_stm32mp15x_dk2(void)
{
	if (CONFIG_IS_ENABLED(TARGET_ST_STM32MP15X) &&
	    (of_machine_is_compatible("st,stm32mp157c-dk2") ||
	     of_machine_is_compatible("st,stm32mp157f-dk2")))
		return true;

	return false;
}

static bool board_is_stm32mp15x_ev1(void)
{
	if (CONFIG_IS_ENABLED(TARGET_ST_STM32MP15X) &&
	    (of_machine_is_compatible("st,stm32mp157a-ev1") ||
	     of_machine_is_compatible("st,stm32mp157c-ev1") ||
	     of_machine_is_compatible("st,stm32mp157d-ev1") ||
	     of_machine_is_compatible("st,stm32mp157f-ev1")))
		return true;

	return false;
}

/* touchscreen driver: used for focaltech touchscreen detection */
static const struct udevice_id edt_ft6236_ids[] = {
	{ .compatible = "focaltech,ft6236", },
	{ }
};

U_BOOT_DRIVER(edt_ft6236) = {
	.name		= "edt_ft6236",
	.id		= UCLASS_I2C_GENERIC,
	.of_match	= edt_ft6236_ids,
};

/* touchscreen driver: only used for pincontrol configuration */
static const struct udevice_id goodix_ids[] = {
	{ .compatible = "goodix,gt911", },
	{ .compatible = "goodix,gt9147", },
	{ }
};

U_BOOT_DRIVER(goodix) = {
	.name		= "goodix",
	.id		= UCLASS_I2C_GENERIC,
	.of_match	= goodix_ids,
};

static int goodix_i2c_read(struct udevice *dev, u16 reg, u8 *buf, int len)
{
	struct i2c_msg msgs[2];
	__be16 wbuf = cpu_to_be16(reg);
	int ret;

	msgs[0].flags = 0;
	msgs[0].addr  = 0x5d;
	msgs[0].len   = 2;
	msgs[0].buf   = (u8 *)&wbuf;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = 0x5d;
	msgs[1].len   = len;
	msgs[1].buf   = buf;

	ret = dm_i2c_xfer(dev, msgs, 2);

	return ret;
}

/* HELPER: search detected driver */
struct detect_info_t {
	bool (*detect)(void);
	struct driver *drv;
};

static struct driver *detect_device(struct detect_info_t *info, u8 size)
{
	struct driver *drv = NULL;
	u8 i;

	for (i = 0; i < size && !drv; i++)
		if (info[i].detect())
			drv = info[i].drv;

	return drv;
}

/* HELPER: force new driver binding, replace the existing one */
static void bind_driver(struct driver *drv, const char *path)
{
	ofnode node;
	struct udevice *dev;
	struct udevice *parent;
	int ret;

	node = ofnode_path(path);
	if (!ofnode_valid(node))
		return;
	if (!ofnode_is_enabled(node))
		return;

	ret = device_find_global_by_ofnode(ofnode_get_parent(node), &parent);
	if (!parent || ret) {
		log_debug("Unable to found parent. err:%d\n", ret);
		return;
	}

	ret = device_find_global_by_ofnode(node, &dev);
	/* remove the driver previously binded */
	if (dev && !ret) {
		if (dev->driver == drv) {
			log_debug("nothing to do, %s already binded.\n", drv->name);
			return;
		}
		log_debug("%s unbind\n", dev->driver->name);
		device_remove(dev, DM_REMOVE_NORMAL);
		device_unbind(dev);
	}
	/* bind the new driver */
	ret = device_bind_with_driver_data(parent, drv, ofnode_get_name(node),
					   0, node, &dev);
	if (ret)
		log_debug("Unable to bind %s, err:%d\n", drv->name, ret);
}

bool stm32mp15x_ev1_rm68200(void)
{
	struct udevice *dev;
	struct udevice *bus;
	struct dm_i2c_chip *chip;
	char id[GOODIX_ID_LEN];
	int ret;

	ret = uclass_get_device_by_driver(UCLASS_I2C_GENERIC, DM_DRIVER_GET(goodix), &dev);
	if (ret)
		return false;

	bus = dev_get_parent(dev);
	chip = dev_get_parent_plat(dev);
	ret = dm_i2c_probe(bus, chip->chip_addr, 0, &dev);
	if (ret)
		return false;

	ret = goodix_i2c_read(dev, GOODIX_REG_ID, id, sizeof(id));
	if (ret)
		return false;

	if (!strncmp(id, "9147", sizeof(id)))
		return true;

	return false;
}

bool stm32mp15x_ev1_hx8394(void)
{
	return true;
}

extern U_BOOT_DRIVER(rm68200_panel);
extern U_BOOT_DRIVER(hx8394_panel);

struct detect_info_t stm32mp15x_ev1_panels[] = {
	CONFIG_IS_ENABLED(VIDEO_LCD_RAYDIUM_RM68200,
			  ({ .detect = stm32mp15x_ev1_rm68200,
			   .drv = DM_DRIVER_REF(rm68200_panel)
			   },
			   ))
	CONFIG_IS_ENABLED(VIDEO_LCD_ROCKTECH_HX8394,
			  ({ .detect = stm32mp15x_ev1_hx8394,
			   .drv = DM_DRIVER_REF(hx8394_panel)
			   },
			   ))
};

static void board_stm32mp15x_ev1_init(void)
{
	struct udevice *dev;
	struct driver *drv;
	struct gpio_desc reset_gpio;
	char path[40];
	int ret;

	/* configure IRQ line on EV1 for touchscreen before LCD reset */
	ret = uclass_get_device_by_driver(UCLASS_I2C_GENERIC, DM_DRIVER_GET(goodix), &dev);
	if (ret)
		return;

	/* get & set reset gpio for panel */
	ret = uclass_get_device_by_driver(UCLASS_PANEL, DM_DRIVER_GET(rm68200_panel), &dev);
	if (ret)
		return;

	gpio_request_by_name(dev, "reset-gpios", 0, &reset_gpio, GPIOD_IS_OUT);

	if (!dm_gpio_is_valid(&reset_gpio))
		return;

	dm_gpio_set_value(&reset_gpio, true);
	mdelay(1);
	dm_gpio_set_value(&reset_gpio, false);
	mdelay(10);

	/* auto detection of connected panel-dsi */
	drv = detect_device(stm32mp15x_ev1_panels, ARRAY_SIZE(stm32mp15x_ev1_panels));
	if (!drv)
		return;
	/* save the detected compatible in environment */
	env_set("panel-dsi", drv->of_match->compatible);

	dm_gpio_free(NULL, &reset_gpio);

	/* select the driver for the detected PANEL */
	ofnode_get_path(dev_ofnode(dev), path, sizeof(path));
	bind_driver(drv, path);
}

/* board dependent setup after realloc */
int board_init(void)
{
	board_key_check();

	regulators_enable_boot_on(_DEBUG);

	/*
	 * sysconf initialisation done only when U-Boot is running in secure
	 * done in TF-A for TFABOOT.
	 */
	if (IS_ENABLED(CONFIG_ARMV7_NONSEC))
		sysconf_init();

	setup_led(LEDST_ON);

#if IS_ENABLED(CONFIG_EFI_HAVE_CAPSULE_SUPPORT)
	efi_guid_t image_type_guid = STM32MP_FIP_IMAGE_GUID;

	guidcpy(&fw_images[0].image_type_id, &image_type_guid);
	fw_images[0].fw_name = u"STM32MP-FIP";
	fw_images[0].image_index = 1;
#endif
	return 0;
}

int board_late_init(void)
{
	const void *fdt_compat;
	int fdt_compat_len;
	int ret;
	u32 otp;
	struct udevice *dev;
	char buf[10];
	char dtb_name[256];
	int buf_len;

	if (IS_ENABLED(CONFIG_VIDEO_LCD_RAYDIUM_RM68200) && board_is_stm32mp15x_ev1())
		board_stm32mp15x_ev1_init();

	if (board_is_stm32mp15x_dk2())
		board_stm32mp15x_dk2_init();

	if (IS_ENABLED(CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG)) {
		fdt_compat = ofnode_get_property(ofnode_root(), "compatible",
						 &fdt_compat_len);
		if (fdt_compat && fdt_compat_len) {
			if (strncmp(fdt_compat, "st,", 3) != 0) {
				env_set("board_name", fdt_compat);
			} else {
				env_set("board_name", fdt_compat + 3);

				buf_len = sizeof(dtb_name);
				strncpy(dtb_name, fdt_compat + 3, buf_len);
				buf_len -= strlen(fdt_compat + 3);
				strncat(dtb_name, ".dtb", buf_len);
				env_set("fdtfile", dtb_name);
			}
		}
		ret = uclass_get_device_by_driver(UCLASS_MISC,
						  DM_DRIVER_GET(stm32mp_bsec),
						  &dev);

		if (!ret)
			ret = misc_read(dev, STM32_BSEC_SHADOW(BSEC_OTP_BOARD),
					&otp, sizeof(otp));
		if (ret > 0 && otp) {
			snprintf(buf, sizeof(buf), "0x%04x", otp >> 16);
			env_set("board_id", buf);

			snprintf(buf, sizeof(buf), "0x%04x",
				 ((otp >> 8) & 0xF) - 1 + 0xA);
			env_set("board_rev", buf);
		}
	}

	if (IS_ENABLED(CONFIG_ADC)) {
		/* probe all ADC for calibration */
		uclass_foreach_dev_probe(UCLASS_ADC, dev) {
			log_debug("ACD probe for calibration: %s\n", dev->name);
		}
		/* for DK1/DK2 boards */
		board_check_usb_power();
	}

	if (CONFIG_IS_ENABLED(FASTBOOT_FLASH_MMC))
		env_set("fastboot.mmcdev", __stringify(CONFIG_FASTBOOT_FLASH_MMC_DEV));

	return 0;
}

void board_quiesce_devices(void)
{
	setup_led(LEDST_OFF);
}

/* CLOCK feed to PHY*/
#define ETH_CK_F_25M	25000000
#define ETH_CK_F_50M	50000000
#define ETH_CK_F_125M	125000000

struct stm32_syscfg_pmcsetr {
	u32 syscfg_clr_off;
	u32 eth1_clk_sel;
	u32 eth1_ref_clk_sel;
	u32 eth1_sel_mii;
	u32 eth1_sel_rgmii;
	u32 eth1_sel_rmii;
	u32 eth2_clk_sel;
	u32 eth2_ref_clk_sel;
	u32 eth2_sel_rgmii;
	u32 eth2_sel_rmii;
};

const struct stm32_syscfg_pmcsetr stm32mp15_syscfg_pmcsetr = {
	.syscfg_clr_off		= 0x44,
	.eth1_clk_sel		= BIT(16),
	.eth1_ref_clk_sel	= BIT(17),
	.eth1_sel_mii		= BIT(20),
	.eth1_sel_rgmii		= BIT(21),
	.eth1_sel_rmii		= BIT(23),
	.eth2_clk_sel		= 0,
	.eth2_ref_clk_sel	= 0,
	.eth2_sel_rgmii		= 0,
	.eth2_sel_rmii		= 0
};

const struct stm32_syscfg_pmcsetr stm32mp13_syscfg_pmcsetr = {
	.syscfg_clr_off		= 0x08,
	.eth1_clk_sel		= BIT(16),
	.eth1_ref_clk_sel	= BIT(17),
	.eth1_sel_mii		= 0,
	.eth1_sel_rgmii		= BIT(21),
	.eth1_sel_rmii		= BIT(23),
	.eth2_clk_sel		= BIT(24),
	.eth2_ref_clk_sel	= BIT(25),
	.eth2_sel_rgmii		= BIT(29),
	.eth2_sel_rmii		= BIT(31)
};

#define SYSCFG_PMCSETR_ETH_MASK		GENMASK(23, 16)
#define SYSCFG_PMCR_ETH_SEL_GMII	0

/* eth init function : weak called in eqos driver */
int board_interface_eth_init(struct udevice *dev,
			     phy_interface_t interface_type, ulong rate)
{
	struct regmap *regmap;
	uint regmap_mask;
	int ret;
	u32 value;
	bool ext_phyclk, eth_clk_sel_reg, eth_ref_clk_sel_reg;
	const struct stm32_syscfg_pmcsetr *pmcsetr;

	/* Ethernet PHY have no crystal */
	ext_phyclk = dev_read_bool(dev, "st,ext-phyclk");

	/* Gigabit Ethernet 125MHz clock selection. */
	eth_clk_sel_reg = dev_read_bool(dev, "st,eth-clk-sel");

	/* Ethernet 50Mhz RMII clock selection */
	eth_ref_clk_sel_reg = dev_read_bool(dev, "st,eth-ref-clk-sel");

	if (device_is_compatible(dev, "st,stm32mp13-dwmac"))
		pmcsetr = &stm32mp13_syscfg_pmcsetr;
	else
		pmcsetr = &stm32mp15_syscfg_pmcsetr;

	regmap = syscon_regmap_lookup_by_phandle(dev, "st,syscon");
	if (!IS_ERR(regmap)) {
		u32 fmp[3];

		ret = dev_read_u32_array(dev, "st,syscon", fmp, 3);
		if (ret)
			/*  If no mask in DT, it is MP15 (backward compatibility) */
			regmap_mask = SYSCFG_PMCSETR_ETH_MASK;
		else
			regmap_mask = fmp[2];
	} else {
		return -ENODEV;
	}

	switch (interface_type) {
	case PHY_INTERFACE_MODE_MII:
		value = pmcsetr->eth1_sel_mii;
		log_debug("PHY_INTERFACE_MODE_MII\n");
		break;
	case PHY_INTERFACE_MODE_GMII:
		value = SYSCFG_PMCR_ETH_SEL_GMII;
		log_debug("PHY_INTERFACE_MODE_GMII\n");
		break;
	case PHY_INTERFACE_MODE_RMII:
		value = pmcsetr->eth1_sel_rmii | pmcsetr->eth2_sel_rmii;
		if (rate == ETH_CK_F_50M && (eth_clk_sel_reg || ext_phyclk))
			value |= pmcsetr->eth1_ref_clk_sel | pmcsetr->eth2_ref_clk_sel;
		log_debug("PHY_INTERFACE_MODE_RMII\n");
		break;
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
		value = pmcsetr->eth1_sel_rgmii | pmcsetr->eth2_sel_rgmii;
		if (rate == ETH_CK_F_125M && (eth_clk_sel_reg || ext_phyclk))
			value |= pmcsetr->eth1_clk_sel | pmcsetr->eth2_clk_sel;
		log_debug("PHY_INTERFACE_MODE_RGMII\n");
		break;
	default:
		log_debug("Do not manage %d interface\n",
			  interface_type);
		/* Do not manage others interfaces */
		return -EINVAL;
	}

	/* Need to update PMCCLRR (clear register) */
	regmap_write(regmap, pmcsetr->syscfg_clr_off, regmap_mask);

	ret = regmap_update_bits(regmap, SYSCFG_PMCSETR, regmap_mask, value);

	return ret;
}

enum env_location env_get_location(enum env_operation op, int prio)
{
	u32 bootmode = get_bootmode();

	if (prio)
		return ENVL_UNKNOWN;

	switch (bootmode & TAMP_BOOT_DEVICE_MASK) {
	case BOOT_FLASH_SD:
	case BOOT_FLASH_EMMC:
		if (CONFIG_IS_ENABLED(ENV_IS_IN_MMC))
			return ENVL_MMC;
		else if (CONFIG_IS_ENABLED(ENV_IS_IN_EXT4))
			return ENVL_EXT4;
		else
			return ENVL_NOWHERE;

	case BOOT_FLASH_NAND:
	case BOOT_FLASH_SPINAND:
		if (IS_ENABLED(CONFIG_ENV_IS_IN_UBI))
			return ENVL_UBI;
		else
			return ENVL_NOWHERE;

	case BOOT_FLASH_NOR:
		if (CONFIG_IS_ENABLED(ENV_IS_IN_SPI_FLASH))
			return ENVL_SPI_FLASH;
		else
			return ENVL_NOWHERE;

	default:
		return ENVL_NOWHERE;
	}
}

const char *env_ext4_get_intf(void)
{
	u32 bootmode = get_bootmode();

	switch (bootmode & TAMP_BOOT_DEVICE_MASK) {
	case BOOT_FLASH_SD:
	case BOOT_FLASH_EMMC:
		return "mmc";
	default:
		return "";
	}
}

int mmc_get_boot(void)
{
	struct udevice *dev;
	u32 boot_mode = get_bootmode();
	unsigned int instance = (boot_mode & TAMP_BOOT_INSTANCE_MASK) - 1;
	char cmd[20];
	const u32 sdmmc_addr[] = {
		STM32_SDMMC1_BASE,
		STM32_SDMMC2_BASE,
		STM32_SDMMC3_BASE
	};

	if (instance >= ARRAY_SIZE(sdmmc_addr))
		return 0;

	/* search associated sdmmc node in devicetree */
	snprintf(cmd, sizeof(cmd), "mmc@%x", sdmmc_addr[instance]);
	if (uclass_get_device_by_name(UCLASS_MMC, cmd, &dev)) {
		log_err("mmc%d = %s not found in device tree!\n", instance, cmd);
		return 0;
	}

	return dev_seq(dev);
};

const char *env_ext4_get_dev_part(void)
{
	static char *const env_dev_part =
#ifdef CONFIG_ENV_EXT4_DEVICE_AND_PART
		CONFIG_ENV_EXT4_DEVICE_AND_PART;
#else
		"";
#endif
	static char *const dev_part[] = {"0:auto", "1:auto", "2:auto"};

	if (strlen(env_dev_part) > 0)
		return env_dev_part;

	return dev_part[mmc_get_boot()];
}

int mmc_get_env_dev(void)
{
	const int mmc_env_dev = CONFIG_IS_ENABLED(ENV_IS_IN_MMC, (CONFIG_SYS_MMC_ENV_DEV), (-1));

	if (mmc_env_dev >= 0)
		return mmc_env_dev;

	/* use boot instance to select the correct mmc device identifier */
	return mmc_get_boot();
}

#if defined(CONFIG_OF_BOARD_SETUP)
void stm32mp15x_dk2_fdt_update(void *new_blob)
{
	struct udevice *dev;
	struct udevice *bus;
	int nodeoff = 0;
	int ret;

	ret = uclass_get_device_by_driver(UCLASS_I2C_GENERIC, DM_DRIVER_GET(edt_ft6236), &dev);
	if (ret)
		return;

	bus = dev_get_parent(dev);

	ret = dm_i2c_probe(bus, 0x38, 0, &dev);
	if (ret < 0) {
		nodeoff = fdt_node_offset_by_compatible(new_blob, -1, "focaltech,ft6236");
		if (nodeoff < 0) {
			log_warning("touchscreen@38 node not found\n");
		} else {
			fdt_set_name(new_blob, nodeoff, "touchscreen@2a");
			fdt_setprop_u32(new_blob, nodeoff, "reg", 0x2a);
			log_debug("touchscreen@38 node updated to @2a\n");
		}
	}
}

void fdt_update_panel_dsi(void *new_blob)
{
	char const *panel = env_get("panel-dsi");
	int nodeoff = 0;

	if (!panel)
		return;

	if (!strcmp(panel, "rocktech,rk055mhd042a0")) {
		nodeoff = fdt_node_offset_by_compatible(new_blob, -1, "raydium,rm68200");
		if (nodeoff < 0) {
			log_warning("panel-dsi node not found");
			return;
		}
		fdt_setprop_string(new_blob, nodeoff, "compatible", panel);

		nodeoff = fdt_node_offset_by_compatible(new_blob, -1, "goodix,gt9147");
		if (nodeoff < 0) {
			log_warning("touchscreen node not found");
			return;
		}
		fdt_setprop_string(new_blob, nodeoff, "compatible", "goodix,gt911");
	}
}

int ft_board_setup(void *blob, struct bd_info *bd)
{
	fdt_copy_fixed_partitions(blob);

	if (IS_ENABLED(CONFIG_FDT_SIMPLEFB))
		fdt_simplefb_enable_and_mem_rsv(blob);

	if (board_is_stm32mp15x_dk2())
		stm32mp15x_dk2_fdt_update(blob);

	if (board_is_stm32mp15x_ev1())
		fdt_update_panel_dsi(blob);

	return 0;
}
#endif

#if defined(CONFIG_OF_BOARD_FIXUP)

int fdt_update_fwu_properties(void *blob, int nodeoff,
			      const char *compat_str,
			      const char *storage_path)
{
	int ret;
	int storage_off;

	ret = fdt_increase_size(blob, 100);
	if (ret) {
		printf("fdt_increase_size: err=%s\n", fdt_strerror(ret));
		return ret;
	}

	ret = fdt_setprop_string(blob, nodeoff, "compatible", compat_str);
	if (ret) {
		log_err("Can't set compatible property\n");
		return ret;
	}

	storage_off = fdt_path_offset(blob, storage_path);
	if (storage_off < 0) {
		log_err("Can't find %s path\n", storage_path);
		return nodeoff;
	}

	ret = fdt_setprop_string(blob, nodeoff, "fwu-mdata-store", storage_path);

	if (ret < 0)
		log_err("Can't set fwu-mdata-store property\n");

	return ret;
}

int fdt_update_fwu_mdata(void *blob)
{
	int nodeoff, ret = 0;
	u32 bootmode;

	nodeoff = fdt_path_offset(blob, "/fwu-mdata");
	if (nodeoff < 0) {
		log_info("no /fwu-mdata node ?\n");

		return 0;
	}

	bootmode = get_bootmode() & TAMP_BOOT_DEVICE_MASK;

	switch (bootmode) {
	case BOOT_FLASH_SD:
		/* sdmmc1 : nothing to do, already the default device tree configuration */
		break;
	case BOOT_FLASH_EMMC:
		/* sdmmc2 */
		ret = fdt_update_fwu_properties(blob, nodeoff, "u-boot,fwu-mdata-gpt",
						"/soc/mmc@58007000");
		break;

	case BOOT_FLASH_NAND:
		/* nand@0 */
		ret = fdt_update_fwu_properties(blob, nodeoff, "u-boot,fwu-mdata-mtd",
						"/soc/bus@5c007000/memory-controller@58002000/nand-controller@4,0/nand@0");
		break;

	case BOOT_FLASH_SPINAND:
	case BOOT_FLASH_NOR:
		/* flash0 */
		ret = fdt_update_fwu_properties(blob, nodeoff, "u-boot,fwu-mdata-mtd",
						"/soc/bus@5c007000/spi@58003000/flash@0");
		break;
	default:
		/* TF-A firmware update not supported for other boot device */
		ret = fdt_del_node(blob, nodeoff);
	}

	return ret;
}

int board_fix_fdt(void *blob)
{
	int ret = 0;

	if (CONFIG_IS_ENABLED(FWU_MDATA))
		ret = fdt_update_fwu_mdata(blob);

	return ret;
}
#endif /* CONFIG_OF_BOARD_FIXUP */

static void board_copro_image_process(ulong fw_image, size_t fw_size)
{
	int ret, id = 0; /* Copro id fixed to 0 as only one coproc on mp1 */

	if (!rproc_is_initialized())
		if (rproc_init()) {
			log_err("Remote Processor %d initialization failed\n",
				id);
			return;
		}

	ret = rproc_load(id, fw_image, fw_size);
	log_err("Load Remote Processor %d with data@addr=0x%08lx %u bytes:%s\n",
		id, fw_image, fw_size, ret ? " Failed!" : " Success!");

	if (!ret)
		rproc_start(id);
}

U_BOOT_FIT_LOADABLE_HANDLER(IH_TYPE_COPRO, board_copro_image_process);

#if defined(CONFIG_FWU_MULTI_BANK_UPDATE)

#include <fwu.h>

/**
 * fwu_plat_get_bootidx() - Get the value of the boot index
 * @boot_idx: Boot index value
 *
 * Get the value of the bank(partition) from which the platform
 * has booted. This value is passed to U-Boot from the earlier
 * stage bootloader which loads and boots all the relevant
 * firmware images
 *
 */
void fwu_plat_get_bootidx(uint *boot_idx)
{
	*boot_idx = (readl(TAMP_FWU_BOOT_INFO_REG) >>
		    TAMP_FWU_BOOT_IDX_OFFSET) & TAMP_FWU_BOOT_IDX_MASK;
}
#endif /* CONFIG_FWU_MULTI_BANK_UPDATE */
