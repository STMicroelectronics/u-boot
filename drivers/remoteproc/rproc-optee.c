// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) STMicroelectronics 2020 - All Rights Reserved
 * Authors: Arnaud Pouliquen <arnaud.pouliquen@st.com>
 */

#define LOG_CATEGORY UCLASS_REMOTEPROC

#include <dm.h>
#include <errno.h>
#include <remoteproc.h>
#include <rproc_optee.h>
#include <tee.h>
#include <dm/device_compat.h>

#define TA_REMOTEPROC_UUID  { 0x80a4c275, 0x0a47, 0x4905, \
		   { 0x82, 0x85, 0x14, 0x86, 0xa9, 0x77, 0x1a, 0x08} }

/* The function IDs implemented in the associated TA */

/*
 * Authentication of the firmware and load in the remote processor memory.
 *
 * [in]  params[0].value.a:	unique 32bit identifier of the firmware
 * [in]  params[1].memref:	buffer containing the image of the firmware
 */
#define TA_RPROC_FW_CMD_LOAD_FW		1

/*
 * Start the remote processor.
 *
 * [in]  params[0].value.a:	unique 32bit identifier of the firmware
 */
#define TA_RPROC_FW_CMD_START_FW	2

/*
 * Stop the remote processor.
 *
 * [in]  params[0].value.a:	unique 32bit identifier of the firmware
 */
#define TA_RPROC_FW_CMD_STOP_FW		3

/*
 * Return the physical address of the resource table, or 0 if not found
 * No check is done to verify that the address returned is accessible by the
 * non secure world. If the resource table is loaded in a protected memory,
 * then accesses from non-secure world will likely fail.
 *
 * [in]  params[0].value.a:	unique 32bit identifier of the firmware
 * [out] params[1].value.a:	32bit LSB resource table memory address
 * [out] params[1].value.b:	32bit MSB resource table memory address
 * [out] params[2].value.a:	32bit LSB resource table memory size
 * [out] params[2].value.b:	32bit MSB resource table memory size
 */
#define TA_RPROC_FW_CMD_GET_RSC_TABLE	4

/*
 * Get remote processor firmware core dump. If found, return either
 * TEE_SUCCESS on successful completion or TEE_ERROR_SHORT_BUFFER if output
 * buffer is too short to store the core dump.
 *
 * [in]  params[0].value.a:	unique 32bit identifier of the firmware
 * [out] params[1].memref:	Core dump, if found
 */
#define TA_RPROC_FW_CMD_GET_COREDUMP	5

static void prepare_args(struct rproc_optee *trproc, int cmd,
			 struct tee_invoke_arg *arg, uint num_param,
			 struct tee_param *param)
{
	memset(arg, 0, sizeof(*arg));
	memset(param, 0, num_param * sizeof(*param));

	arg->func = cmd;
	arg->session = trproc->session;

	param[0] = (struct tee_param) {
		.attr = TEE_PARAM_ATTR_TYPE_VALUE_INPUT,
		.u.value.a = trproc->proc_id,
	};
}

int rproc_optee_load(struct rproc_optee *trproc, ulong addr, ulong size)
{
	struct tee_invoke_arg arg;
	struct tee_param param[2];
	struct tee_shm *fw_shm;
	int rc;

	rc = tee_shm_register(trproc->tee, (void *)addr, size, 0, &fw_shm);
	if (rc)
		return rc;

	prepare_args(trproc, TA_RPROC_FW_CMD_LOAD_FW, &arg, 2, param);

	/* Provide the address and size of the firmware image */
	param[1] = (struct tee_param){
		.attr = TEE_PARAM_ATTR_TYPE_MEMREF_INPUT,
		.u.memref = {
			.shm = fw_shm,
			.size = size,
			.shm_offs = 0,
		},
	};

	rc = tee_invoke_func(trproc->tee, &arg, 2, param);
	if (rc < 0 || arg.ret != 0) {
		dev_err(trproc->tee,
			"TA_RPROC_FW_CMD_LOAD_FW invoke failed TEE err: %x, err:%x\n",
			arg.ret, rc);
		if (!rc)
			rc = -EIO;
	}

	tee_shm_free(fw_shm);

	return rc;
}

int rproc_optee_get_rsc_table(struct rproc_optee *trproc, phys_addr_t *rsc_addr,
			      phys_size_t *rsc_size)
{
	struct tee_invoke_arg arg;
	struct tee_param param[3];
	int rc;

	prepare_args(trproc, TA_RPROC_FW_CMD_GET_RSC_TABLE, &arg, 3, param);

	param[1].attr = TEE_PARAM_ATTR_TYPE_VALUE_OUTPUT;
	param[2].attr = TEE_PARAM_ATTR_TYPE_VALUE_OUTPUT;

	rc = tee_invoke_func(trproc->tee, &arg, 3, param);
	if (rc < 0 || arg.ret != 0) {
		dev_err(trproc->tee,
			"TA_RPROC_FW_CMD_GET_RSC_TABLE invoke failed TEE err: %x, err:%x\n",
			arg.ret, rc);
		if (!rc)
			rc = -EIO;

		return rc;
	}

	*rsc_size = (phys_size_t)
			(param[2].u.value.b << 32 | param[2].u.value.a);
	*rsc_addr = (phys_addr_t)
			(param[1].u.value.b << 32 | param[1].u.value.a);

	return 0;
}

int rproc_optee_start(struct rproc_optee *trproc)
{
	struct tee_invoke_arg arg;
	struct tee_param param;
	int rc;

	prepare_args(trproc, TA_RPROC_FW_CMD_START_FW, &arg, 1, &param);

	rc =  tee_invoke_func(trproc->tee, &arg, 1, &param);
	if (rc < 0 || arg.ret != 0) {
		dev_err(trproc->tee,
			"TA_RPROC_FW_CMD_START_FW invoke failed TEE err: %x, err:%x\n",
			arg.ret, rc);
		if (!rc)
			rc = -EIO;
	}

	return rc;
}

int rproc_optee_stop(struct rproc_optee *trproc)
{
	struct tee_invoke_arg arg;
	struct tee_param param;
	int rc;

	prepare_args(trproc, TA_RPROC_FW_CMD_STOP_FW, &arg, 1, &param);

	rc =  tee_invoke_func(trproc->tee, &arg, 1, &param);
	if (rc < 0 || arg.ret != 0) {
		dev_err(trproc->tee,
			"TA_RPROC_FW_CMD_STOP_FW invoke failed TEE err: %x, err:%x\n",
			arg.ret, rc);
		if (!rc)
			rc = -EIO;
	}

	return rc;
}

int rproc_optee_open(struct rproc_optee *trproc)
{
	struct udevice *tee = NULL;
	const struct tee_optee_ta_uuid uuid = TA_REMOTEPROC_UUID;
	struct tee_open_session_arg arg = { };
	struct tee_param param;
	int rc;

	if (!trproc)
		return -EINVAL;

	tee = tee_find_device(tee, NULL, NULL, NULL);
	if (!tee)
		return -ENODEV;

	tee_optee_ta_uuid_to_octets(arg.uuid, &uuid);

	param.attr = TEE_PARAM_ATTR_TYPE_VALUE_INPUT;
	param.u.value.a = trproc->proc_id;

	rc = tee_open_session(tee, &arg, 1, &param);
	if (rc < 0 || arg.ret != 0) {
		if (!rc)
			rc = -EIO;
		return rc;
	}

	trproc->tee = tee;
	trproc->session = arg.session;

	return 0;
}

int rproc_optee_close(struct rproc_optee *trproc)
{
	int rc;

	if (!trproc->tee)
		return -ENODEV;

	rc = tee_close_session(trproc->tee, trproc->session);
	if (rc)
		return rc;

	trproc->tee = NULL;
	trproc->session = 0;

	return 0;
}
