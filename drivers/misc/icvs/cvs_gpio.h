/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * Intel Computer Vision System driver
 *
 * Copyright (C) 2024 Intel Corporation.
 *
 */

#ifndef __CVS_GPIO_H__
#define __CVS_GPIO_H__

#include <linux/gpio.h>

/* GPIO Full Resources */
const static struct acpi_gpio_params gpio_wake = {0, 0, false};
const static struct acpi_gpio_params gpio_rst = {1, 0, false};
const static struct acpi_gpio_params gpio_req = {2, 0, false};
const static struct acpi_gpio_params gpio_resp = {3, 0, false};
const static struct acpi_gpio_mapping icvs_acpi_gpios[] = {
	{"wake-gpio", &gpio_wake, 1},
	{"rst-gpio", &gpio_rst, 1},
	{"req-gpio", &gpio_req, 1},
	{"resp-gpio", &gpio_resp, 1},
	{ }
};

/* GPIO Light Resources */
const static struct acpi_gpio_params lgpio_req = {0, 0, false};
const static struct acpi_gpio_params lgpio_resp = {1, 0, false};
const static struct acpi_gpio_mapping icvs_acpi_lgpios[] = {
	{"req-gpio", &lgpio_req, 1},
	{"resp-gpio", &lgpio_resp, 1},
	{ }
};

#endif // __CVS_GPIO_H__
