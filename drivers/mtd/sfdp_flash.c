// SPDX-License-Identifier: GPL-2.0-or-later OR BSD-3-Clause
/*
 * (C) Copyright 2022
 * Takahiro Kuwano, <takahiro.kuwano@infineon.com>
 */

#include <common.h>
#include <flash.h>
#include <asm/byteorder.h>
#include <mtd/cfi_flash.h>

#define SFDP_SIGNATURE		0x50444653U
#define SFDP_JESD216_MAJOR	1
#define SFDP_OFFSET_S26_ID	(0x800)

struct sfdp_header {
	u32		signature; /* Ox50444653U <=> "SFDP" */
	u8		minor;
	u8		major;
	u8		nph; /* 0-base number of parameter headers */
	u8		prot;
};

struct sfdp_flash_info {
	const ushort mfr_id;
	const ushort dev_id;
	const ushort dev_id2;
	const char *name;
};

static const struct sfdp_flash_info s26_table[] = {
	{ 0x0034, 0x006a, 0x001a, "s26hl512t" },
	{ 0x0034, 0x006a, 0x001b, "s26hl01gt" },
	{ 0x0034, 0x007b, 0x001a, "s26hs512t" },
	{ 0x0034, 0x007b, 0x001b, "s26hs01gt" },
};

static int detect_s26(flash_info_t *info, const void *sfdp)
{
	const ushort *id = (const ushort *)sfdp + SFDP_OFFSET_S26_ID;
	int i;

	for (i = 0; i < ARRAY_SIZE(s26_table); i++) {
		if (s26_table[i].mfr_id == le16_to_cpu(id[0]) &&
		    s26_table[i].dev_id == le16_to_cpu(id[1]) &&
		    s26_table[i].dev_id2 == le16_to_cpu(id[2])) {
			info->manufacturer_id = s26_table[i].mfr_id;
			info->device_id = s26_table[i].dev_id;
			info->device_id2 = s26_table[i].dev_id2;
			info->name = s26_table[i].name;
			info->vendor = CFI_CMDSET_AMD_STANDARD;
			info->flash_id = FLASH_MAN_CFI;
			info->interface = FLASH_CFI_X16;
			info->addr_unlock1 = 0x555;
			info->addr_unlock2 = 0x2aa;
			info->sr_supported = 1;
			info->buffer_size = 512;
			info->cmd_reset = AMD_CMD_RESET;
			info->cmd_erase_sector = AMD_CMD_ERASE_SECTOR;
			info->erase_blk_tout = 3072;
			info->write_tout = 1;
			info->buffer_write_tout = 4;
			info->size = 1ULL << info->device_id2;
#ifdef CONFIG_SYS_FLASH_PROTECTION
			info->legacy_unlock = 1;
#endif
			return 1;
		}
	}

	return 0;
}

int sfdp_flash_scan(flash_info_t *info, const void *sfdp)
{
	const struct sfdp_header *header = (const struct sfdp_header *)sfdp;

	/* Check the SFDP signature and header version. */
	if (le32_to_cpu(header->signature) == SFDP_SIGNATURE &&
	    header->major == SFDP_JESD216_MAJOR) {
		return detect_s26(info, sfdp);
	}

	return 0;
}
