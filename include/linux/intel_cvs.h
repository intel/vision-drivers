/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *
 * Copyright (C) 2024 Intel Corporation.
 *
 */

#ifndef __INTEL_CVS_H__
#define __INTEL_CVS_H__

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>

#define DEBUG_CVS

/* ICVS # of GPIOs */
#define ICVS_FULL 4
#define ICVS_LIGHT 2

/* WDT timeout in ms */
#define WDT_TIMEOUT 5000

/* RESET values */
#define RST_TIME 100
#define RST_RETRY 5

#define GPIO_READ_DELAY_MS 100
#define GPIO_WRITE_DELAY_MS 100
#define GPIO_RESET_MS 2
#define FW_MAX_RETRY 5
/* Below two macros are WA for "JIRA ID ISCVS-13" */
#define WA_FW_DL_WAIT 100
#define WA_FW_DL_WAIT_2 1000
#define _MAX_PATH 260
#define I2C_PKT_SIZE 256
#define CV_FW_DL_MAX_TRY_DEFAULT 5
#define FW_PREPARE_MS 100
#define WAIT_HOST_RELEASE_MS 10
#define WAIT_HOST_WAKE_NORMAL_MS 1000
#define WAIT_HOST_WAKE_RESET_MS 1000
#define WAIT_NORMAL_MS 500
#define WAIT_HOST_WAKE_FLASH_LONG_MS 200000L
#define MAGICNUMSIZE 8
#define MAGICNUM                                       \
	{                                              \
		'V', 'I', 'S', 'S', 'O', 'C', 'F', 'W' \
	}
#define CVMAGICNUMSIZE              sizeof(u32)
#define CVMAGICNUM                  0xCAFEB0BA
#define CVMAGICNUM_REVERSE          0xBAB0FECA
#define CVMAGICNUM_UNKNOWN          0xDEADBEEF

/* ICVS capability */
enum icvs_cap { ICVS_NOTSUP = 0, ICVS_LIGHTCAP, ICVS_FULLCAP };

/* Supported commands by CV SoC */
enum cvs_command {
	GET_DEVICE_STATE = 0x0800,
	GET_FW_VERSION = 0x0801,
	GET_VID_PID = 0x0802,
	GET_DEV_CAPABILITY = 0x0804,
	SET_HOST_IDENTIFIER = 0x0805,
	FW_LOADER_START = 0x0820,
	FW_LOADER_DATA = 0x0821,
	FW_LOADER_END = 0x0822,
	HOST_SET_MIPI_CONFIG = 0x0830,
	FACTORY_RESET = 0x0831,
};

/* CV SoC device status */
enum cvs_state {
	DEVICE_OFF_STATE = 0x00,
	PRIVACY_ON_BIT_MASK = (1 << 0),
	DEVICE_ON_BIT_MASK = (1 << 1),
	SENSOR_OWNER_BIT_MASK = (1 << 2),
	DEVICE_DWNLD_STATE_MASK = (1 << 4),
	DEVICE_DWNLD_ERROR_MASK = (1 << 6),
	DEVICE_DWNLD_BUSY_MASK = (1 << 7),
};

/* CVS sensor Status */
enum icvs_sensor_state {
	CV_SENSOR_RELEASED_STATE = 0,
	CV_SENSOR_VISION_ACQUIRED_STATE = (1 << 0),
	CV_SENSOR_IPU_ACQUIRED_STATE = (1 << 1)
};

/* CVS driver Status */
enum icvs_state {
	CV_INIT_STATE = 0,
	CV_FW_DOWNLOADING_STATE = (1 << 1),
	CV_FW_FLASHING_STATE = (1 << 3),
	CV_STOPPING = (1 << 7)
};

enum cvs_camera_owner {
	CVS_CAMERA_NONE = 0,
	CVS_CAMERA_CVS,
	CVS_CAMERA_IPU,
};

union cv_host_identifiers {
	u32      value;
	struct {
		u32      Reserved : 27;
		u32      VisionSensing : 1;
		u32      DevicePowerSetting : 2;
		u32      PrivacyLedHost : 1;
		u32      rgbCameraPwrUpHost : 1;
	} field;
};

enum cv_dev_capability_mask

{
	HOST_MIPI_CONFIG_REQUIRED_MASK  = (1 << 15),
	FW_ANTIROLLBACK_MASK            = (1 << 14),
	PRIVACY2VISIONDRIVER_MASK       = (1 << 13),
	FWUPDATE_RESET_REQUIRED_MASK    = (1 << 12),
	NOCAMERA_DURING_FWUPDATE_MASK   = (1 << 11),
	CV_POWER_DOMAIN_MASK            = (1 << 10),
	CAPABILITY_RESERVED_BYTE_MASK   = (0xFF),
};

struct cv_ver_capability {
	u8 protocol_ver_major;
	u8 protocol_ver_minor;
	u16  dev_capability;
} __attribute__((__packed__));

/* CV-returned data from i2c */
struct cvs_fw {
	u32 major;
	u32 minor;
	u32 hotfix;
	u32 build;
} __attribute__((__packed__));

struct cvs_id {
	u16 vid;
	u16 pid;
} __attribute__((__packed__));

struct cvs_fw_header {
	u8 magic_number[8];
	struct cvs_fw fw_ver;
	struct cvs_id vid_pid;
	/* offset of the FW binary from the beginning of the file */
	u32 fw_offset;
	/* pad with 0 */
	u8 reserved[256 - sizeof(u8) * 8 /*magic_number*/ -
		    sizeof(struct cvs_fw) - sizeof(struct cvs_id) -
		    sizeof(u32) /*fw_offset*/ - sizeof(u32) /*header_checksum*/];
	/* 4bytes CRC checksum of the header, not include this header_checksum itself */
	u32 header_checksum;
};

struct intel_cvs {
	struct device *dev;
	enum icvs_cap cap;

	int irq;
	struct gpio_desc *rst;
	struct gpio_desc *req;
	struct gpio_desc *resp;

	/* CVS Status */
	struct cvs_id id;
	struct cvs_fw ver;
	enum cvs_state cvs_state;
	enum icvs_state icvs_state;
	enum icvs_sensor_state icvs_sensor_state;
	union cv_host_identifiers host_identifiers;
	bool magic_num_support;
	u32  magic_num_received;
	struct cv_ver_capability cv_fw_capability;

	int i2c_shared;
	unsigned long long oem_prod_id;
	enum cvs_camera_owner owner;
	int int_ref_count;

	/* FW update info */
	u8 in_buf[I2C_PKT_SIZE];
	u8 out_buf[I2C_PKT_SIZE];
	struct work_struct fw_dl_task;
	const struct firmware *file;
	u32 fw_update_retries;
	u32 fw_file_path[_MAX_PATH];
	void *fw_buffer;
	u32 fw_buffer_size;
	u32 max_flashtime_ms;
	u8 cv_fw_state;
	char fw_filename[24];
	bool fw_dl_task_finished;
	bool fw_dl_needed;
	bool close_fw_dl_task;
	wait_queue_head_t hostwake_event;
	wait_queue_head_t update_complete_event;
	int hostwake_event_arg;
	int update_complete_event_arg;
};

#ifdef DEBUG_CVS
int cvs_sysfs_dump(char *buf);
int cvs_exec_cmd(enum cvs_command command);
#endif

#endif // __INTEL_CVS_H__
