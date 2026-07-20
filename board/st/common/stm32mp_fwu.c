// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/*
 * Copyright (C) 2026 Amarula Solutions, Dario Binacchi <dario.binacchi@amarulasolutions.com>
 */

#include <env.h>
#include <fwu.h>
#include <part_efi.h>
#include <asm/io.h>
#include <asm/arch/stm32.h>

/* METADA UID identifying the image type for STM32MP platform */

/* STM32MP bootfs = 8da7c9d3-d368-487b-b28b-71e1b0c68cf0 */
#define STM32MP_BOOTFS \
	EFI_GUID(0x8DA7C9D3, 0xD368, 0x487B, \
		 0xB2, 0x8B, 0x71, 0xE1, 0xB0, 0xC6, 0x8C, 0xF0)

/* STM32MP rootfs = b28961f8-c72c-4088-bbdb-c785861473fa */
#define STM32MP_ROOTFS \
	EFI_GUID(0xB28961F8, 0xC72C, 0x4088, \
		 0xBB, 0xDB, 0xC7, 0x85, 0x86, 0x14, 0x73, 0xFA)

/* STM32MP vendorfs = 3f427af9-e80f-4811-bd33-29aaf35a28f8 */
#define STM32MP_VENDORFS \
	EFI_GUID(0x3F427AF9, 0xE80F, 0x4811, \
		 0xBD, 0x33, 0x29, 0xAA, 0xF3, 0x5A, 0x28, 0xF8)

/* STM32MP userfs = bf9b43d9-fcb3-4253-8297-afd9c68df1ef */
#define STM32MP_USERFS \
	EFI_GUID(0xBF9B43D9, 0xFCB3, 0x4253, \
		 0x82, 0x97, 0xAF, 0xD9, 0xC6, 0x8D, 0xF1, 0xEF)

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

int fwu_platform_hook(struct udevice *dev, struct fwu_data *data)
{
	uint boot_idx;
	efi_guid_t uuid;
	const efi_guid_t boot_uid = STM32MP_BOOTFS;
	const efi_guid_t root_uid = STM32MP_ROOTFS;
	const efi_guid_t vendor_uid = STM32MP_VENDORFS;
	const efi_guid_t user_uid = STM32MP_USERFS;
	char uuidbuf[UUID_STR_LEN + 1];
	int ret;

	fwu_plat_get_bootidx(&boot_idx);

	ret = fwu_mdata_get_image_guid(&uuid, &boot_uid, boot_idx);
	if (!ret) {
		uuid_bin_to_str(uuid.b, uuidbuf, UUID_STR_FORMAT_GUID);
		env_set("boot_partuuid", uuidbuf);
	}
	ret = fwu_mdata_get_image_guid(&uuid, &root_uid, boot_idx);
	if (!ret) {
		uuid_bin_to_str(uuid.b, uuidbuf, UUID_STR_FORMAT_GUID);
		env_set("root_partuuid", uuidbuf);
	}
	ret = fwu_mdata_get_image_guid(&uuid, &vendor_uid, boot_idx);
	if (!ret) {
		uuid_bin_to_str(uuid.b, uuidbuf, UUID_STR_FORMAT_GUID);
		env_set("vendor_partuuid", uuidbuf);
	}
	ret = fwu_mdata_get_image_guid(&uuid, &user_uid, boot_idx);
	if (!ret) {
		uuid_bin_to_str(uuid.b, uuidbuf, UUID_STR_FORMAT_GUID);
		env_set("user_partuuid", uuidbuf);
	}

	return 0;
}
