/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * Intel Computer Vision System driver
 *
 * Copyright (C) 2024 Intel Corporation.
 *
 */

#include <linux/module.h>
#include <linux/acpi.h>
#include <linux/i2c.h>

static int cvs_i2c_probe(struct i2c_client *i2c)
{
	int ret = 0;

	if (i2c)
		dev_info(&i2c->dev, "%s\n", __func__);
	else
		ret = -ENODEV;

	return ret;
}

static void cvs_i2c_remove(struct i2c_client *i2c)
{
	if (i2c)
		dev_info(&i2c->dev, "%s\n", __func__);
}

static struct acpi_device_id acpi_cvs_ids[] = {
	{ "INTC10CF" }, /* MTL */
	{ "INTC10DE" }, /* LNL */
	{ "INTC10E0" }, /* ARL */
	{ "INTC10E1" }, /* PTL */
	{ /* END OF LIST */ }
};
MODULE_DEVICE_TABLE(acpi, acpi_cvs_ids);

static struct i2c_driver cvs_i2c_driver = {
	.driver = {
		.name = "Intel CVS driver",
		.owner = THIS_MODULE,
		.acpi_match_table = ACPI_PTR(acpi_cvs_ids),
	},
	.probe = cvs_i2c_probe,
	.remove = cvs_i2c_remove
};

static int __init icvs_init(void)
{
	int ret;

	ret = i2c_add_driver(&cvs_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register I2C driver: %d\n", ret);

	return ret;
}
module_init(icvs_init);

static void __exit icvs_exit(void)
{
	i2c_del_driver(&cvs_i2c_driver);
}
module_exit(icvs_exit);

MODULE_AUTHOR("Israel Cepeda <israel.a.cepeda.lopez@intel.com>");
MODULE_DESCRIPTION("Intel CVS driver");
MODULE_LICENSE("GPL v2");
