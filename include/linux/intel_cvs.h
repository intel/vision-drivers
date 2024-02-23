/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * Intel Computer Vision System driver
 *
 * Copyright (C) 2024 Intel Corporation.
 *
 */

#ifndef __INTEL_CVS_H__
#define __INTEL_CVS_H__

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>

#define DEBUG_CVS

/* ICVS # of GPIOs */
#define ICVS_FULL	4
#define ICVS_LIGHT	2

/* WDT timeout in ms */
#define WDT_TIMEOUT	1600

/* RESET values */
#define RST_TIME	100
#define RST_RETRY	5

#define GPIO_READ_DELAY_MS 2


/* ICVS capability */
enum icvs_cap {
	ICVS_NOTSUP = 0,
	ICVS_LIGHTCAP,
	ICVS_FULLCAP
};

/* Supported commands by CV SoC */
enum cvs_command {
	GET_DEVICE_STATE     = 0x0800,
	GET_FW_VERSION       = 0x0801,
	GET_VID_PID          = 0x0802,
	FW_LOADER_START      = 0x0820,
	FW_LOADER_DATA       = 0x0821,
	FW_LOADER_END        = 0x0822,
	HOST_SET_MIPI_CONFIG = 0x0830,
	FACTORY_RESET        = 0x0831,
};

/* CV SoC device status */
enum cvs_state {
	DEVICE_OFF_STATE        = 0x00,
	PRIVACY_ON_BIT_MASK     = (1 << 0),
	DEVICE_ON_BIT_MASK      = (1 << 1),
	SENSOR_OWNER_BIT_MASK   = (1 << 2),
	DEVICE_DWNLD_STATE_MASK = (1 << 4),
	DEVICE_DWNLD_ERROR_MASK = (1 << 6),
	DEVICE_DWNLD_BUSY_MASK  = (1 << 7),
};

/* CVS driver Status */
enum icvs_state {
	CV_INIT_STATE = 0,
	CV_FW_DOWNLOADING_STATE = (1 << 1),
	CV_FW_FLASHING_STATE = (2 << 2),
	CV_STOPPING = (1 << 7)
};

struct cvs_mipi_config  {
	u32 freq;
	u32 lane_num;
};

enum cvs_privacy_status {
	CVS_PRIVACY_ON = 0,
	CVS_PRIVACY_OFF,
};

enum cvs_camera_owner {
	CVS_CAMERA_NONE = 0, 
	CVS_CAMERA_CVS,
	CVS_CAMERA_IPU,
};

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

struct intel_cvs {
	struct device *dev;
	enum icvs_cap cap;
	spinlock_t lock;

	int irq;
	struct gpio_desc *rst;
	struct gpio_desc *req;
	struct gpio_desc *resp;

	int rst_retry;
	struct hrtimer wdt;
	struct work_struct rst_task;

	/* CVS Status */
	struct cvs_id id;
	struct cvs_fw ver;
	enum cvs_state cvs_state;   
	enum icvs_state icvs_state;  

	int i2c_shared;
	enum cvs_camera_owner owner;
	u64 cnt;
	int ref_count;
	int int_ref_count;    

	/* FW pointer */
	void *fw_buffer;
	u32 fw_buffer_size;
};

struct cvs_camera_status {
	enum cvs_camera_owner owner;
	enum cvs_privacy_status status;
	u32 exposure_level;
};

typedef void (*cvs_privacy_callback_t)(void *handle,
					enum cvs_privacy_status status);

int cvs_acquire_camera_sensor(struct cvs_mipi_config *config,
				cvs_privacy_callback_t callback,
				void *handle,
				struct cvs_camera_status *status);
int cvs_release_camera_sensor(struct cvs_camera_status *status);

#ifdef DEBUG_CVS     
int cvs_sysfs_dump(char *buf);
int cvs_exec_cmd(enum cvs_command command);
int cvs_get_state(void);
#endif

#endif // __INTEL_CVS_H__
