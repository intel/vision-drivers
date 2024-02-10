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

#include <linux/i2c.h>
#include <linux/gpio/consumer.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/workqueue.h>

/* ICVS # of GPIOs */
#define ICVS_FULL	4
#define ICVS_LIGHT	2

/* WDT timeout in ms */
#define WDT_TIMEOUT	1600

/* RESET values */
#define RST_TIME	100
#define RST_RETRY	5

/* ICVS capability */
enum icvs_cap {
	ICVS_NOTSUP = 0,
	ICVS_LIGHTCAP,
	ICVS_FULLCAP
};

struct intel_cvs {
	struct device *dev;
	enum icvs_cap cap;

	int irq;
	struct gpio_desc *rst;
	struct gpio_desc *req;
	struct gpio_desc *resp;

	int rst_retry;
	struct hrtimer wdt;
	struct work_struct rst_task;
};

#endif // __INTEL_CVS_H__
