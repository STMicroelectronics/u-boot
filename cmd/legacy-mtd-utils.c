// SPDX-License-Identifier: GPL-2.0+

#include <mtd.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/string.h>
#include <jffs2/load_kernel.h>

/* mapping between legacy parameter and MTD device type */
bool check_devtype(int devtype, u_char mtdtype)
{
	if (devtype == MTD_DEV_TYPE_NOR && mtdtype == MTD_NORFLASH)
		return true;

	if ((devtype == MTD_DEV_TYPE_NAND || devtype == MTD_DEV_TYPE_ONENAND) &&
	    (mtdtype == MTD_NANDFLASH || mtdtype == MTD_NANDFLASH))
		return true;

	return false;
}


static int get_part(const char *partname, int *idx, loff_t *off, loff_t *size,
		    loff_t *maxsize, int devtype)
{
	struct mtd_info *mtd;
	struct mtd_info *partition;
	bool part_found = false;
	int part_num;

	if (!IS_ENABLED(CONFIG_MTD)) {
		puts("mtd support missing.\n");
		return -1;
	}
	/* register partitions with MTDIDS/MTDPARTS or OF fallback */
	mtd_probe_devices();

	mtd_for_each_device(mtd) {
		printf("%s:%d(%d, %s)\n", __func__, __LINE__, mtd->type, mtd->name);
		if (mtd_is_partition(mtd) &&
		    check_devtype(devtype, mtd->type) &&
		    (!strcmp(partname, mtd->name))) {
			part_found = true;
			break;
		}
	}
	if (!part_found)
		return -1;

	*off = mtd->offset;
	*size = mtd->size;
	*maxsize = mtd->size;

	/* loop on partition list as index is not accessbile in MTD */
	part_num = 0;
	list_for_each_entry(partition, &mtd->parent->partitions, node) {
		part_num++;
		if (partition == mtd)
			break;
	}

	*idx = part_num;
	return 0;
}

int mtd_arg_off(const char *arg, int *idx, loff_t *off, loff_t *size,
		loff_t *maxsize, int devtype, uint64_t chipsize)
{
	if (!str2off(arg, off))
		return get_part(arg, idx, off, size, maxsize, devtype);

	if (*off >= chipsize) {
		puts("Offset exceeds device limit\n");
		return -1;
	}

	*maxsize = chipsize - *off;
	*size = *maxsize;
	return 0;
}

int mtd_arg_off_size(int argc, char *const argv[], int *idx, loff_t *off,
		     loff_t *size, loff_t *maxsize, int devtype,
		     uint64_t chipsize)
{
	int ret;

	if (argc == 0) {
		*off = 0;
		*size = chipsize;
		*maxsize = *size;
		goto print;
	}

	ret = mtd_arg_off(argv[0], idx, off, size, maxsize, devtype,
			  chipsize);
	if (ret)
		return ret;

	if (argc == 1)
		goto print;

	if (!str2off(argv[1], size)) {
		printf("'%s' is not a number\n", argv[1]);
		return -1;
	}

	if (*size > *maxsize) {
		puts("Size exceeds partition or device limit\n");
		return -1;
	}

	if (*size == 0) {
		debug("ERROR: Invalid size 0\n");
		return -1;
	}

print:
	printf("device %d ", *idx);
	if (*size == chipsize)
		puts("whole chip\n");
	else
		printf("offset 0x%llx, size 0x%llx\n",
		       (unsigned long long)*off, (unsigned long long)*size);
	return 0;
}
