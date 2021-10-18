/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef _UCSI_H_
#define _UCSI_H_

/* UCSI offsets (Bytes) */
#define UCSI_VERSION			0
#define UCSI_CCI			4
#define UCSI_CONTROL			8
#define UCSI_MESSAGE_IN			16

/* Commands */
#define UCSI_PPM_RESET			0x01
#define UCSI_ACK_CC_CI			0x04
#define UCSI_SET_NOTIFICATION_ENABLE	0x05
#define UCSI_GET_CAPABILITY		0x06
#define UCSI_GET_ALTERNATE_MODES	0x0c
#define UCSI_GET_CONNECTOR_STATUS	0x12
#define UCSI_GET_ERROR_STATUS		0x13

#define UCSI_CONNECTOR_NUMBER(_num_)		((u64)(_num_) << 16)

/* ACK_CC_CI bits */
#define UCSI_ACK_CONNECTOR_CHANGE		BIT(16)
#define UCSI_ACK_COMMAND_COMPLETE		BIT(17)

/* SET_NOTIFICATION_ENABLE command bits */
#define UCSI_ENABLE_NTFY_CONNECTOR_CHANGE	BIT(30)

/* Command Status and Connector Change Indication (CCI) bits */
#define UCSI_CCI_CONNECTOR(_c_)		(((_c_) & GENMASK(7, 1)) >> 1)
#define UCSI_CCI_LENGTH(_c_)		(((_c_) & GENMASK(15, 8)) >> 8)
#define UCSI_CCI_NOT_SUPPORTED		BIT(25)
#define UCSI_CCI_RESET_COMPLETE		BIT(27)
#define UCSI_CCI_BUSY			BIT(28)
#define UCSI_CCI_ERROR			BIT(30)
#define UCSI_CCI_COMMAND_COMPLETE	BIT(31)

/* Error information returned by PPM in response to GET_ERROR_STATUS command. */
#define UCSI_ERROR_UNREGONIZED_CMD		BIT(0)
#define UCSI_ERROR_INVALID_CON_NUM		BIT(1)
#define UCSI_ERROR_INVALID_CMD_ARGUMENT		BIT(2)
#define UCSI_ERROR_INCOMPATIBLE_PARTNER		BIT(3)
#define UCSI_ERROR_CC_COMMUNICATION_ERR		BIT(4)
#define UCSI_ERROR_DEAD_BATTERY			BIT(5)
#define UCSI_ERROR_CONTRACT_NEGOTIATION_FAIL	BIT(6)
#define UCSI_ERROR_OVERCURRENT			BIT(7)
#define UCSI_ERROR_UNDEFINED			BIT(8)
#define UCSI_ERROR_PARTNER_REJECTED_SWAP	BIT(9)
#define UCSI_ERROR_HARD_RESET			BIT(10)
#define UCSI_ERROR_PPM_POLICY_CONFLICT		BIT(11)
#define UCSI_ERROR_SWAP_REJECTED		BIT(12)

/* Data structure filled by PPM in response to GET_CAPABILITY command. */
struct ucsi_capability {
	u32 attributes;
	u8 num_connectors;
	u8 features;
	u16 reserved_1;
	u8 num_alt_modes;
	u8 reserved_2;
	u16 bc_version;
	u16 pd_version;
	u16 typec_version;
} __packed;

/* Data structure filled by PPM in response to GET_CONNECTOR_STATUS command. */
struct ucsi_connector_status {
	u16 change;
	u16 flags;
#define UCSI_CONSTAT_CONNECTED			BIT(3)
#define UCSI_CONSTAT_PARTNER_TYPE(_f_)		(((_f_) & GENMASK(15, 13)) >> 13)
#define   UCSI_CONSTAT_PARTNER_TYPE_DFP		1
#define   UCSI_CONSTAT_PARTNER_TYPE_UFP		2
#define   UCSI_CONSTAT_PARTNER_TYPE_CABLE	3 /* Powered Cable */
#define   UCSI_CONSTAT_PARTNER_TYPE_CABLE_AND_UFP	4 /* Powered Cable */
#define   UCSI_CONSTAT_PARTNER_TYPE_DEBUG	5
#define   UCSI_CONSTAT_PARTNER_TYPE_AUDIO	6
	u32 request_data_obj;
	u8 pwr_status;
} __packed;

/**
 * struct ucsi_ops - driver I/O operations for UCSI uclass
 *
 * Drivers should support 2 operations. These operations are intended to be used
 * by uclass code, not directly from other code.
 */
struct ucsi_ops {
	/**
	 * read() - Read operation
	 *
	 * @ucsi:	UCSI device to read from
	 * @offset:	UCSI data structure offset
	 * @buf:	Buffer to receive the data
	 * @len		Number of bytes to read
	 * @return 0 on success, -ve on failure
	 */
	int (*read)(struct udevice *ucsi, unsigned int offset, void *val, size_t len);

	/**
	 * write() - Write operation
	 *
	 * @ucsi:	UCSI device to write to
	 * @offset:	UCSI data structure offset
	 * @buf:	Buffer data to write
	 * @len		Number of bytes to write
	 * @return 0 on success, -ve on failure
	 */
	int (*write)(struct udevice *ucsi, unsigned int offset, const void *val, size_t len);
};
#endif /* _UCSI_H_ */
