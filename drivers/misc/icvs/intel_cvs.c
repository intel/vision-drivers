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

#include "intel_cvs.h"
#include "cvs_gpio.h"

static irqreturn_t cvs_irq_handler(int irq, void *devid)
{
	struct intel_cvs *icvs = devid;
	bool ret = false;

	if (!icvs || !icvs->dev)
		goto exit;

	ret = true;

exit:
	return IRQ_RETVAL(ret);
}

static int cvs_init(struct intel_cvs *icvs)
{
	int ret = -EINVAL;

	if (!icvs || !icvs->dev || !icvs->rst)
		goto exit;

	gpiod_set_value_cansleep(icvs->rst, 1);

	ret = devm_request_irq(icvs->dev, icvs->irq, cvs_irq_handler,
			IRQF_ONESHOT | IRQF_NO_SUSPEND, dev_name(icvs->dev), icvs);
	if (ret)
		dev_err(icvs->dev, "Failed to request irq\n");

exit:
	return ret;
}

static int cvs_i2c_probe(struct i2c_client *i2c)
{
	struct intel_cvs *icvs;
	int ret = -ENODEV;

	if (!i2c) {
		pr_err("No I2C device\n");
		goto exit;
	}

	dev_info(&i2c->dev, "%s\n", __func__);
	icvs = devm_kzalloc(&i2c->dev, sizeof(struct intel_cvs), GFP_KERNEL);
	if (!icvs) {
		ret = -ENOMEM;
		goto exit;
	}

	icvs->dev = &i2c->dev;
	i2c_set_clientdata(i2c, icvs);

	ret = gpiod_count(icvs->dev, NULL);
	switch (ret) {
		case ICVS_LIGHT:
			icvs->cap = ICVS_LIGHTCAP;
			break;
		case ICVS_FULL:
			icvs->cap = ICVS_FULLCAP;
			break;
		default:
			dev_err(icvs->dev, "Number of GPIOs not supported: %d\n", ret);
			devm_kfree(icvs->dev, icvs);
			ret = -EINVAL;
			goto exit;
	}

	ret = devm_acpi_dev_add_driver_gpios(icvs->dev, icvs->cap == ICVS_FULLCAP ?
										icvs_acpi_gpios : icvs_acpi_lgpios);
	if (ret) {
		dev_err(icvs->dev, "Failed to add driver gpios\n");
		goto exit;
	}

	/* Request GPIO */
	icvs->req = devm_gpiod_get(icvs->dev, "req", GPIOD_OUT_HIGH);
	if (IS_ERR(icvs->req)) {
		dev_err(icvs->dev, "Failed to get REQUEST gpiod\n");
		ret = -EIO;
		goto exit;
	}

	/* Response GPIO */
	icvs->resp = devm_gpiod_get(icvs->dev, "resp", GPIOD_IN);
	if (IS_ERR(icvs->resp)) {
		dev_err(icvs->dev, "Failed to get RESPONSE gpiod\n");
		ret = -EIO;
		goto exit;
	}

	if (icvs->cap == ICVS_FULLCAP) {
		/* Reset GPIO */
		icvs->rst = devm_gpiod_get(icvs->dev, "rst", GPIOD_OUT_LOW);
		if (IS_ERR(icvs->rst)) {
			dev_err(icvs->dev, "Failed to get RESET gpiod\n");
			ret = -EIO;
			goto exit;
		}

		/* Wake Interrupt */
		ret = acpi_dev_gpio_irq_get_by(ACPI_COMPANION(icvs->dev), "wake-gpio", 0);
		if (ret > 0)
			icvs->irq = ret;
		else {
			dev_err(icvs->dev, "Failed to get WAKE interrupt: %d\n", ret);
			goto exit;
		}

		ret = cvs_init(icvs);
		if (ret)
			dev_err(icvs->dev, "Failed to initialize\n");
	}

exit:
	if (ret)
		devm_kfree(icvs->dev, icvs);

	return ret;
}

static void cvs_exit(struct intel_cvs *icvs)
{
	if (icvs && icvs->dev && icvs->rst) {
		devm_free_irq(icvs->dev, icvs->irq, icvs);
		gpiod_set_value_cansleep(icvs->rst, 0);
	}
}

static void cvs_i2c_remove(struct i2c_client *i2c)
{
	struct intel_cvs *icvs;

	if (i2c) {
		dev_info(&i2c->dev, "%s\n", __func__);
		icvs = i2c_get_clientdata(i2c);
		if (icvs) {
			if (icvs->cap == ICVS_FULLCAP)
				cvs_exit(icvs);
			devm_kfree(&i2c->dev, icvs);
		}
	}
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
