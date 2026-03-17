/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) STMicroelectronics 2020 - All Rights Reserved
 */

#ifndef _RPROC_OPTEE_H_
#define _RPROC_OPTEE_H_

/**
 * struct rproc_optee - TEE remoteproc structure
 * @tee:	TEE device
 * @fw_id:	Identifier of the target firmware
 * @session:	TEE session identifier
 */
struct rproc_optee {
	struct udevice *tee;
	u32 fw_id;
	u32 session;
};

#if IS_ENABLED(CONFIG_REMOTEPROC_OPTEE)

/**
 * rproc_optee_open() - open a rproc tee session
 *
 * Open a session towards the trusted application in charge of the remote
 * processor.
 *
 * @trproc: OPTEE remoteproc context structure
 *
 * @return 0 if the session is opened, or an appropriate error value.
 */
int rproc_optee_open(struct rproc_optee *trproc);

/**
 * rproc_optee_close() - close a rproc tee session
 *
 * Close the trusted application session in charge of the remote processor.
 *
 * @trproc: OPTEE remoteproc context structure
 *
 * @return 0 on success, or an appropriate error value.
 */
int rproc_optee_close(struct rproc_optee *trproc);

/**
 * rproc_optee_start() - Request OP-TEE to start a remote processor
 *
 * @trproc: OPTEE remoteproc context structure
 *
 * @return 0 on success, or an appropriate error value.
 */
int rproc_optee_start(struct rproc_optee *trproc);

/**
 * rproc_optee_stop() - Request OP-TEE to stop a remote processor
 *
 * @trproc: OPTEE remoteproc context structure
 *
 * @return 0 on success, or an appropriate error value.
 */
int rproc_optee_stop(struct rproc_optee *trproc);

/**
 * rproc_optee_get_rsc_table() - Request OP-TEE the resource table
 *
 * Get the address and the size of the resource table. If no resource table is
 * found, the size and address are null.
 *
 * @trproc: OPTEE remoteproc context structure
 * @rsc_addr:  out the physical address of the resource table returned
 * @rsc_size:  out the size of the resource table
 *
 * @return 0 on success, or an appropriate error value.
 */
int rproc_optee_get_rsc_table(struct rproc_optee *trproc, phys_addr_t *rsc_addr,
			      phys_size_t *rsc_size);

/**
 * rproc_optee_load() - load an signed ELF image
 *
 * @trproc: OPTEE remoteproc context structure
 * @addr:	valid ELF image address
 * @size:	size of the image
 *
 * @return 0 if the image is successfully loaded, else appropriate error value.
 */
int rproc_optee_load(struct rproc_optee *trproc, ulong addr, ulong size);

#else

static inline int  rproc_optee_open(struct rproc_optee *trproc)
{
	return -ENOSYS;
}

static inline int  rproc_optee_close(struct rproc_optee *trproc)
{
	return -ENOSYS;
}

static inline int  rproc_optee_start(struct rproc_optee *trproc)
{
	return -ENOSYS;
}

static inline int  rproc_optee_stop(struct rproc_optee *trproc)
{
	return -ENOSYS;
}

static inline int  rproc_optee_get_rsc_table(struct rproc_optee *trproc,
					     phys_addr_t *rsc_addr,
					     phys_size_t *rsc_size)
{
	return -ENOSYS;
}

static inline int  rproc_optee_load(struct rproc_optee *trproc, ulong addr,
				    ulong size)
{
	return -ENOSYS;
}

#endif

#endif	/* _RPROC_OPTEE_H_ */
