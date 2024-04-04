// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2000-2010
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * (C) Copyright 2001 Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Andreas Heppel <aheppel@sysgo.de>
 */

/* #define DEBUG */

#include <common.h>
#include <command.h>
#include <env.h>
#include <env_internal.h>
#include <log.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <linux/stddef.h>
#include <malloc.h>
#include <search.h>
#include <errno.h>
#include <u-boot/crc.h>

DECLARE_GLOBAL_DATA_PTR;

#ifndef CONFIG_SPL_BUILD
# if defined(CONFIG_CMD_SAVEENV) && defined(CONFIG_CMD_FLASH)
#  include <flash.h>
#  define CMD_SAVEENV
# elif defined(CONFIG_ENV_ADDR_REDUND)
#  error CONFIG_ENV_ADDR_REDUND must have CONFIG_CMD_SAVEENV & CONFIG_CMD_FLASH
# endif
#endif

/* TODO(sjg@chromium.org): Figure out all these special cases */
#if (!defined(CONFIG_MICROBLAZE) && !defined(CONFIG_ARCH_ZYNQ) && \
	!defined(CONFIG_TARGET_MCCMON6) && !defined(CONFIG_TARGET_X600) && \
	!defined(CONFIG_TARGET_EDMINIV2)) || \
	!defined(CONFIG_SPL_BUILD)
#define LOADENV
#endif

#if !defined(CONFIG_TARGET_X600) || !defined(CONFIG_SPL_BUILD)
#define INITENV
#endif

#if defined(CONFIG_ENV_ADDR_REDUND) && defined(CMD_SAVEENV) || \
	!defined(CONFIG_ENV_ADDR_REDUND) && defined(INITENV)
#ifdef ENV_IS_EMBEDDED
static env_t *env_ptr = &embedded_environment;
#else /* ! ENV_IS_EMBEDDED */

static env_t *env_ptr = (env_t *)CONFIG_ENV_ADDR;
#endif /* ENV_IS_EMBEDDED */
#endif
static __maybe_unused env_t *flash_addr = (env_t *)CONFIG_ENV_ADDR;

/* CONFIG_ENV_ADDR is supposed to be on sector boundary */
static ulong __maybe_unused end_addr =
		CONFIG_ENV_ADDR + CONFIG_ENV_SECT_SIZE - 1;

#ifdef CONFIG_ENV_ADDR_REDUND

static env_t __maybe_unused *flash_addr_new = (env_t *)CONFIG_ENV_ADDR_REDUND;

/* CONFIG_ENV_ADDR_REDUND is supposed to be on sector boundary */
static ulong __maybe_unused end_addr_new =
		CONFIG_ENV_ADDR_REDUND + CONFIG_ENV_SECT_SIZE - 1;
#endif /* CONFIG_ENV_ADDR_REDUND */

#ifdef CONFIG_ENV_ADDR_REDUND
#ifdef INITENV
static int env_flash_init(void)
{
	int crc1_ok = 0, crc2_ok = 0;
	uchar flag1, flag2;
	ulong addr1, addr2;
	env_t *tmp_env1, *tmp_env2;

	gd->env_valid = ENV_INVALID;

	if (!is_flash_available())
		return 0;

	tmp_env1 = (env_t *)malloc(CONFIG_ENV_SIZE);
	tmp_env2 = (env_t *)malloc(CONFIG_ENV_SIZE);

	if (!tmp_env1 || !tmp_env2) {
		env_set_default("malloc() failed", 0);
		return -EIO;
	}

	memcpy_fromio(tmp_env1, flash_addr, CONFIG_ENV_SIZE);
	memcpy_fromio(tmp_env2, flash_addr_new, CONFIG_ENV_SIZE);

	flag1 = tmp_env1->flags;
	flag2 = tmp_env2->flags;

	addr1 = (ulong)&(flash_addr->data);
	addr2 = (ulong)&(flash_addr_new->data);

	crc1_ok = crc32(0, tmp_env1->data, ENV_SIZE) == tmp_env1->crc;
	crc2_ok = crc32(0, tmp_env2->data, ENV_SIZE) == tmp_env2->crc;

	if (crc1_ok && !crc2_ok) {
		gd->env_addr	= addr1;
		gd->env_valid	= ENV_VALID;
	} else if (!crc1_ok && crc2_ok) {
		gd->env_addr	= addr2;
		gd->env_valid	= ENV_VALID;
	} else if (!crc1_ok && !crc2_ok) {
		gd->env_valid	= ENV_INVALID;
	} else if (flag1 == ENV_REDUND_ACTIVE &&
		   flag2 == ENV_REDUND_OBSOLETE) {
		gd->env_addr	= addr1;
		gd->env_valid	= ENV_VALID;
	} else if (flag1 == ENV_REDUND_OBSOLETE &&
		   flag2 == ENV_REDUND_ACTIVE) {
		gd->env_addr	= addr2;
		gd->env_valid	= ENV_VALID;
	} else if (flag1 == flag2) {
		gd->env_addr	= addr1;
		gd->env_valid	= ENV_REDUND;
	} else if (flag1 == 0xFF) {
		gd->env_addr	= addr1;
		gd->env_valid	= ENV_REDUND;
	} else if (flag2 == 0xFF) {
		gd->env_addr	= addr2;
		gd->env_valid	= ENV_REDUND;
	}

	free(tmp_env1);
	free(tmp_env2);

	return 0;
}

#if CONFIG_ENV_SECT_SIZE > CONFIG_ENV_SIZE
static int save_rest_of_sector(ulong start, ulong end, char **saved_data)
{
	ulong up_data = 0;

	up_data = end + 1 - (start + CONFIG_ENV_SIZE);
	debug("Data to save 0x%lX\n", up_data);
	if (up_data) {
		*saved_data = malloc(up_data);
		if (!saved_data) {
			printf("Can't allocate the rest of sector\n");
			return -ENOMEM;
		}
		memcpy_fromio(*saved_data, (void *)(start + CONFIG_ENV_SIZE),
			      up_data);
		debug("Data (start 0x%lX, len 0x%lX) saved at 0x%p\n",
		      start + CONFIG_ENV_SIZE, up_data, *saved_data);
	}

	return 0;
}

static int restore_rest_of_sector(ulong start, ulong end, char *saved_data)
{
	ulong up_data;
	int ret = 0;

	up_data = end + 1 - (start + CONFIG_ENV_SIZE);
	if (up_data) {
		debug("Restoring the rest of data to 0x%lX len 0x%lX\n",
		      start + CONFIG_ENV_SIZE, up_data);
		ret = flash_write(saved_data, start + CONFIG_ENV_SIZE, up_data);
	}

	return ret;
}
#endif
#endif

static int env_update_flag(char flag, env_t *start, ulong end)
{
	env_t tmp_env;
	char *saved_data = NULL;
	int ret;

	memcpy_fromio(&tmp_env, start, CONFIG_ENV_SIZE);
	tmp_env.flags = flag;

#if CONFIG_ENV_SECT_SIZE > CONFIG_ENV_SIZE
	ret = save_rest_of_sector((ulong)start, end, &saved_data);
	if (ret)
		return ret;
#endif
	puts("Erasing Flash...");
	debug(" %08lX ... %08lX ...", (ulong)start, end);

	if (flash_sect_erase((ulong)start, end))
		goto done;

	puts("Writing to Flash... ");
	debug(" %08lX ... %08lX ...",
	      (ulong)&start,
	      sizeof(tmp_env.data) + (ulong)&start->data);

	ret = flash_write((char *)&tmp_env, (ulong)start, sizeof(tmp_env));
	if (ret)
		goto perror;

#if CONFIG_ENV_SECT_SIZE > CONFIG_ENV_SIZE
	ret = restore_rest_of_sector((long)start, end, saved_data);
#endif

perror:
	flash_perror(ret);
done:
	free(saved_data);

	return ret;
}

#ifdef CMD_SAVEENV
static int env_flash_save(void)
{
	env_t	env_new;
	char	*saved_data = NULL;
	char	flag = ENV_REDUND_OBSOLETE, new_flag = ENV_REDUND_ACTIVE;
	int	rc = 1;

	debug("Protect off %08lX ... %08lX\n", (ulong)flash_addr, end_addr);

	if (flash_sect_protect(0, (ulong)flash_addr, end_addr))
		goto done;

	debug("Protect off %08lX ... %08lX\n",
		(ulong)flash_addr_new, end_addr_new);

	if (flash_sect_protect(0, (ulong)flash_addr_new, end_addr_new))
		goto done;

	rc = env_export(&env_new);
	if (rc)
		return rc;
	env_new.flags	= new_flag;

#if CONFIG_ENV_SECT_SIZE > CONFIG_ENV_SIZE
	rc = save_rest_of_sector((long)flash_addr_new, end_addr_new, &saved_data);
	if (rc)
		goto done;

#endif
	puts("Erasing Flash...");
	debug(" %08lX ... %08lX ...", (ulong)flash_addr_new, end_addr_new);

	if (flash_sect_erase((ulong)flash_addr_new, end_addr_new))
		goto done;

	puts("Writing to Flash... ");
	debug(" %08lX ... %08lX ...",
		(ulong)&(flash_addr_new),
		sizeof(env_ptr->data) + (ulong)&(flash_addr_new->data));
	rc = flash_write((char *)&env_new, (ulong)flash_addr_new,
			 sizeof(env_new));
	if (rc)
		goto perror;

	rc = env_update_flag(flag, flash_addr, end_addr);
	if (rc)
		goto perror;

#if CONFIG_ENV_SECT_SIZE > CONFIG_ENV_SIZE
	rc = restore_rest_of_sector((long)flash_addr_new, end_addr_new, saved_data);
	if (rc)
		goto perror;

#endif
	puts("done\n");

	{
		env_t *etmp = flash_addr;
		ulong ltmp = end_addr;

		flash_addr = flash_addr_new;
		flash_addr_new = etmp;

		end_addr = end_addr_new;
		end_addr_new = ltmp;
	}

	rc = 0;
	goto done;
perror:
	flash_perror(rc);
done:
	free(saved_data);
	/* try to re-protect */
	flash_sect_protect(1, (ulong)flash_addr, end_addr);
	flash_sect_protect(1, (ulong)flash_addr_new, end_addr_new);

	return rc;
}
#endif /* CMD_SAVEENV */

#else /* ! CONFIG_ENV_ADDR_REDUND */

#ifdef INITENV
static int env_flash_init(void)
{
	env_t *tmp_env;

	gd->env_valid = ENV_INVALID;

	if (!is_flash_available())
		return 0;

	tmp_env = malloc(CONFIG_ENV_SIZE);
	if (!tmp_env) {
		env_set_default("malloc() failed", 0);
		return -EIO;
	}

	memcpy_fromio(tmp_env, env_ptr, CONFIG_ENV_SIZE);

	if (crc32(0, tmp_env->data, ENV_SIZE) == tmp_env->crc) {
		gd->env_addr = (ulong)&(env_ptr->data);
		gd->env_valid = ENV_VALID;
	}

	free(tmp_env);

	return 0;
}
#endif

#ifdef CMD_SAVEENV
static int env_flash_save(void)
{
	env_t	env_new;
	int	rc = 1;
	char	*saved_data = NULL;
#if CONFIG_ENV_SECT_SIZE > CONFIG_ENV_SIZE
	ulong	up_data = 0;

	up_data = end_addr + 1 - ((long)flash_addr + CONFIG_ENV_SIZE);
	debug("Data to save 0x%lx\n", up_data);
	if (up_data) {
		saved_data = malloc(up_data);
		if (saved_data == NULL) {
			printf("Unable to save the rest of sector (%ld)\n",
				up_data);
			goto done;
		}
		memcpy(saved_data,
			(void *)((long)flash_addr + CONFIG_ENV_SIZE), up_data);
		debug("Data (start 0x%lx, len 0x%lx) saved at 0x%lx\n",
			(ulong)flash_addr + CONFIG_ENV_SIZE,
			up_data,
			(ulong)saved_data);
	}
#endif	/* CONFIG_ENV_SECT_SIZE */

	debug("Protect off %08lX ... %08lX\n", (ulong)flash_addr, end_addr);

	if (flash_sect_protect(0, (long)flash_addr, end_addr))
		goto done;

	rc = env_export(&env_new);
	if (rc)
		goto done;

	puts("Erasing Flash...");
	if (flash_sect_erase((long)flash_addr, end_addr))
		goto done;

	puts("Writing to Flash... ");
	rc = flash_write((char *)&env_new, (long)flash_addr, CONFIG_ENV_SIZE);
	if (rc != 0)
		goto perror;

#if CONFIG_ENV_SECT_SIZE > CONFIG_ENV_SIZE
	if (up_data) {	/* restore the rest of sector */
		debug("Restoring the rest of data to 0x%lx len 0x%lx\n",
			(ulong)flash_addr + CONFIG_ENV_SIZE, up_data);
		if (flash_write(saved_data,
				(long)flash_addr + CONFIG_ENV_SIZE,
				up_data))
			goto perror;
	}
#endif
	puts("done\n");
	rc = 0;
	goto done;
perror:
	flash_perror(rc);
done:
	free(saved_data);
	/* try to re-protect */
	flash_sect_protect(1, (long)flash_addr, end_addr);
	return rc;
}
#endif /* CMD_SAVEENV */

#endif /* CONFIG_ENV_ADDR_REDUND */

#ifdef LOADENV
static int env_flash_load(void)
{
	env_t *tmp_env;
	int ret = 0;

	tmp_env = malloc(CONFIG_ENV_SIZE);
	if (!tmp_env) {
		env_set_default("malloc() failed", 0);
		return -EIO;
	}

	memcpy_fromio(tmp_env, flash_addr, CONFIG_ENV_SIZE);

#ifdef CONFIG_ENV_ADDR_REDUND
	if (gd->env_addr != (ulong)&(flash_addr->data)) {
		env_t *etmp = flash_addr;
		ulong ltmp = end_addr;

		flash_addr = flash_addr_new;
		flash_addr_new = etmp;

		end_addr = end_addr_new;
		end_addr_new = ltmp;
	}

	if (flash_addr_new->flags != ENV_REDUND_OBSOLETE &&
	    crc32(0, tmp_env->data, ENV_SIZE) == tmp_env->crc) {
		gd->env_valid = ENV_REDUND;
		flash_sect_protect(0, (ulong)flash_addr_new, end_addr_new);
		ret = env_update_flag(ENV_REDUND_OBSOLETE, flash_addr_new, end_addr_new);
		flash_sect_protect(1, (ulong)flash_addr_new, end_addr_new);
	}

	if (flash_addr->flags != ENV_REDUND_ACTIVE &&
	    (flash_addr->flags & ENV_REDUND_ACTIVE) == ENV_REDUND_ACTIVE) {
		gd->env_valid = ENV_REDUND;
		flash_sect_protect(0, (ulong)flash_addr, end_addr);
		ret = env_update_flag(ENV_REDUND_ACTIVE, flash_addr, end_addr);
		flash_sect_protect(1, (ulong)flash_addr, end_addr);
	}

	if (gd->env_valid == ENV_REDUND) {
		if (ret)
			puts("*** Error - some problems detected "
			     "environment can't be recovered\n\n");
		else
			puts("*** Warning - some problems detected "
			     "reading environment; recovered successfully\n\n");
	}

#endif /* CONFIG_ENV_ADDR_REDUND */

	ret = env_import((char *)tmp_env, 1, H_EXTERNAL);
	free(tmp_env);

	return ret;
}
#endif /* LOADENV */

U_BOOT_ENV_LOCATION(flash) = {
	.location	= ENVL_FLASH,
	ENV_NAME("Flash")
#ifdef LOADENV
	.load		= env_flash_load,
#endif
#ifdef CMD_SAVEENV
	.save		= env_save_ptr(env_flash_save),
#endif
#ifdef INITENV
	.init		= env_flash_init,
#endif
};
