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

#define FW_BIN_HDR_SIZE 256
#define GPIO_READ_DELAY_MS 100
#define GPIO_WRITE_DELAY_MS 100
#define GPIO_RESET_MS 2
#define FW_MAX_RETRY 5
#define I2C_PKT_SIZE 256
#define FW_PREPARE_MS 100
#define WAIT_HOST_RELEASE_MS 10
#define WAIT_HOST_WAKE_NORMAL_MS 1000
#define WAIT_HOST_WAKE_RESET_MS 1000
#define WAIT_NORMAL_MS 500
#define CVMAGICNUMSIZE sizeof(u32)
#define CVMAGICNUM 0xCAFEB0BA
#define CVMAGICNUM_UNKNOWN 0xDEADBEEF

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
	u32 value;
	struct {
		u32 reserved : 27;
		u32 vision_sensing : 1;
		u32 device_power_setting : 2;
		u32 privacy_led_host : 1;
		u32 rgbcamera_pwrup_host : 1;
	} field;
};

enum cv_dev_capability_mask {
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
} __packed;

/* CV-returned data from i2c */
struct cvs_fw {
	u32 major;
	u32 minor;
	u32 hotfix;
	u32 build;
} __packed;

struct cvs_id {
	u16 vid;
	u16 pid;
} __packed;

/* Params updated by vision driver */
struct cvs_to_plugin_interface {
	/* Device FW Version */
	u32 major;
	u32 minor;
	u32 hotfix;
	u32 build;
	/* Device vid,pid */
	u16 vid;
	u16 pid;
	int opid;
	int dev_capabilities;
};

/* Params updated by CVS plugin */
struct plugin_to_cvs_interface {
	int max_download_time; /*milli sec*/
	int max_flash_time; /*milli sec*/
	int max_fwupd_retry_count;
	int fw_bin_fd; /*file descriptor*/
};

/* FW update status Info */
struct ctrl_data_fwupd {
	u8 dev_state;
	int fw_upd_retries;
	int total_packets;
	int num_packets_sent;
	/*0:In progress, 1: Finished */
	bool fw_dl_finshed;
	/* 0:PASS, Non-zero error code. Read only after fw_dl finished=1 */
	int fw_dl_status_code;
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
	enum icvs_state icvs_state;
	enum icvs_sensor_state icvs_sensor_state;
	bool magic_num_support;
	struct cv_ver_capability cv_fw_capability;

	unsigned long long oem_prod_id;
	enum cvs_camera_owner owner;
	int int_ref_count;

	/* FW update info */
	struct work_struct fw_dl_task;
	u32 fw_update_retries;
	void *fw_buffer;
	u32 fw_buffer_size;
	u32 max_flashtime_ms;
	u8 cv_fw_state;
	bool fw_dl_task_finished;
	bool close_fw_dl_task;
	wait_queue_head_t hostwake_event;
	wait_queue_head_t update_complete_event;
	wait_queue_head_t lvfs_fwdl_complete_event;
	int hostwake_event_arg;
	int update_complete_event_arg;
	int lvfs_fwdl_complete_event_arg;
	bool cv_suspend;
	bool fw_dl_task_started;
	spinlock_t buffer_lock;
	struct cvs_to_plugin_interface cvs_to_plugin;
	struct plugin_to_cvs_interface plugin_to_cvs;
	struct ctrl_data_fwupd info_fwupd;
};

#ifdef DEBUG_CVS
int cvs_sysfs_dump(char *buf);
int cvs_exec_cmd(enum cvs_command command);
#endif

#endif // __INTEL_CVS_H__
