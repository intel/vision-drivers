/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * Intel Computer Vision System driver
 *
 * Copyright (C) 2024 Intel Corporation.
 *
 */

#include <linux/acpi.h>
#include <linux/intel_cvs.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>

#include "cvs_gpio.h"
#include "intel_cvs_update.h"

struct intel_cvs *cvs = NULL;

static irqreturn_t cvs_irq_handler(int irq, void *devid)
{
	struct intel_cvs *icvs = devid;
	bool ret = false;

	if (!icvs || !icvs->dev)
		goto exit;

	icvs->rst_retry = RST_RETRY;
	icvs->hostwake_event_arg = 1;
	wake_up_interruptible(&icvs->hostwake_event);

	if (icvs->fw_dl_task_finished == true)
		hrtimer_start(&icvs->wdt, ms_to_ktime(WDT_TIMEOUT),
			      HRTIMER_MODE_REL);
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
			       IRQF_ONESHOT | IRQF_NO_SUSPEND,
			       dev_name(icvs->dev), icvs);
	if (ret) {
		dev_err(icvs->dev, "Failed to request irq\n");
		goto exit;
	}

	icvs->rst_retry = RST_RETRY;

exit:
	return ret;
}

static void cvs_wdt_reset_thread(struct work_struct *work)
{
	struct intel_cvs *icvs = container_of(work, struct intel_cvs, rst_task);

	if (!icvs)
		return;

	dev_info(icvs->dev, "%s\n", __func__);
	gpiod_set_value_cansleep(icvs->rst, 0);

	if (icvs->rst_retry--) {
		icvs->owner = CVS_CAMERA_NONE;
		msleep(RST_TIME);
		gpiod_set_value_cansleep(icvs->rst, 1);
		hrtimer_start(&icvs->wdt, ms_to_ktime(WDT_TIMEOUT),
			      HRTIMER_MODE_REL);
	} else
		dev_err(icvs->dev, "%s:Device unresponsive!\n", __func__);
}

static enum hrtimer_restart cvs_wdt_reset(struct hrtimer *t)
{
	struct intel_cvs *icvs = container_of(t, struct intel_cvs, wdt);

	if (!icvs)
		goto exit;

	dev_warn(icvs->dev, "%s\n", __func__);
	schedule_work(&icvs->rst_task);

exit:
	return HRTIMER_NORESTART;
}

static int find_shared_i2c(acpi_handle handle, const char *method_name)
{
	struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
	acpi_status status;

	status = acpi_evaluate_object(handle, (acpi_string)method_name, NULL,
				      &buffer);
	if (ACPI_FAILURE(status))
		return -ENODEV;

	if (!buffer.pointer)
		return -ENODEV;

	ACPI_FREE(buffer.pointer);
	pr_info("%s:ACPI method %s found for i2c_shared\n", __func__,
		method_name);
	return 0;
}

static int cvs_i2c_probe(struct i2c_client *i2c)
{
	struct intel_cvs *icvs;
	acpi_handle handle;
	int ret = -ENODEV;

	if (!i2c) {
		pr_err("No I2C device\n");
		goto exit;
	}

	dev_info(&i2c->dev, "%s with i2c_client:%p\n", __func__, i2c);
	icvs = devm_kzalloc(&i2c->dev, sizeof(struct intel_cvs), GFP_KERNEL);
	if (!icvs) {
		ret = -ENOMEM;
		goto exit;
	}
	icvs->dev = &i2c->dev;
	i2c_set_clientdata(i2c, icvs);
	cvs = icvs;

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

	ret = devm_acpi_dev_add_driver_gpios(
		icvs->dev,
		icvs->cap == ICVS_FULLCAP ? icvs_acpi_gpios : icvs_acpi_lgpios);
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
		ret = acpi_dev_gpio_irq_get_by(ACPI_COMPANION(icvs->dev),
					       "wake-gpio", 0);
		if (ret > 0)
			icvs->irq = ret;
		else {
			dev_err(icvs->dev, "Failed to get WAKE interrupt: %d\n",
				ret);
			goto exit;
		}

		INIT_WORK(&icvs->rst_task, cvs_wdt_reset_thread);
		INIT_WORK(&icvs->fw_dl_task, cvs_fw_dl_thread);
		init_waitqueue_head(&icvs->hostwake_event);
		init_waitqueue_head(&icvs->update_complete_event);
		icvs->fw_dl_task_finished = false;

		hrtimer_init(&icvs->wdt, CLOCK_REALTIME, HRTIMER_MODE_REL);
		icvs->wdt.function = cvs_wdt_reset;

		ret = cvs_init(icvs);
		if (ret)
			dev_err(icvs->dev, "Failed to initialize\n");
	}

	handle = ACPI_HANDLE(&i2c->dev);
	if (!handle) {
		dev_err(icvs->dev, "Failed to get ACPI handle\n");
		goto exit;
	}
	icvs->i2c_shared = (find_shared_i2c(handle, "IICS") < 0) ? 0 : 1;

	if (icvs->cap == ICVS_FULLCAP) {
		/* Start FW D/L task cvs_fw_dl_thread() */
		mdelay(FW_PREPARE_MS);
		cvs_release_camera_sensor_internal();
		mdelay(FW_PREPARE_MS);
		cvs_reset_cv_device();
		mdelay(FW_PREPARE_MS);
		schedule_work(&icvs->fw_dl_task);
	}
exit:
	if (ret)
		devm_kfree(icvs->dev, icvs);

	return ret;
}

static void cvs_exit(struct intel_cvs *icvs)
{
	if (icvs && icvs->dev && icvs->rst) {
		hrtimer_cancel(&icvs->wdt);
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
			release_firmware(cvs->file);
			cvs->close_fw_dl_task = true;

			while (cvs->ref_count) {
				pr_info("%s:Camera is used by IPU. check again after %dms",
					__func__, WAIT_HOST_RELEASE_MS);
				mdelay(WAIT_HOST_RELEASE_MS);
			}

			if (icvs->cap == ICVS_FULLCAP) {
				if (!cvs->fw_dl_task_finished) {
					pr_info("%s:signal cvs_fw_dl_thread() to stop",
						__func__);
					cvs->hostwake_event_arg = 1;
					wake_up_interruptible(
						&cvs->hostwake_event);

					pr_info("%s:Wait for cvs_fw_dl_thread() to stop",
						__func__);
					wait_event_interruptible(
						cvs->update_complete_event,
						cvs->update_complete_event_arg ==
							1);
					pr_info("%s:cvs_fw_dl_thread() stopped",
						__func__);
					mdelay(WAIT_HOST_RELEASE_MS);
				}
				cvs_exit(icvs);
			}
			devm_kfree(&i2c->dev, icvs);
		}
	}
}

int cvs_read_i2c(u16 cmd, char *data, int size)
{
	struct i2c_client *i2c = container_of(cvs->dev, struct i2c_client, dev);
	int cnt;
	u16 cvs_cmd = (((cmd) >> 8) & 0x00ff) | (((cmd) << 8) & 0xff00);

	if (size < 0 || !data)
		return -EINVAL;

	cnt = i2c_master_send(i2c, (const char *)&cvs_cmd, sizeof(u16));
	if (cnt != sizeof(u16)) {
		dev_err(&i2c->dev, "%s:cmd:%x count:%d (!=2)\n", __func__, cmd,
			cnt);
		return -EIO;
	}

	cnt = i2c_master_recv(i2c, (char *)data, size);
	return cnt;
}

int cvs_acquire_camera_sensor_internal(void)
{
	int val, retry = 20;

	if (!cvs)
		return -EINVAL;

	if (cvs->icvs_state == CV_FW_DOWNLOADING_STATE)
		return -EBUSY;

	if (cvs->owner != CVS_CAMERA_IPU) {
		do {
			gpiod_set_value_cansleep(cvs->req, 0);
			mdelay(GPIO_READ_DELAY_MS);
			val = gpiod_get_value_cansleep(cvs->resp);
		} while (val != 0 && retry--);

		if (val != 0)
			goto err_out;
	}
	cvs->int_ref_count++;
	return 0;

err_out:
	dev_err(cvs->dev, "%s:error! val %d (!=0)\n", __func__, val);
	return -EIO;
}

int cvs_release_camera_sensor_internal(void)
{
	int val, retry = 20;

	if (!cvs)
		return -EINVAL;

	if (cvs->icvs_state == CV_FW_DOWNLOADING_STATE)
		return -EBUSY;

	if (cvs->owner != CVS_CAMERA_CVS) {
		do {
			gpiod_set_value_cansleep(cvs->req, 1);
			mdelay(GPIO_READ_DELAY_MS);
			val = gpiod_get_value_cansleep(cvs->resp);
		} while (val != 1 && retry--);

		if (val != 1)
			goto err_out;
	}
	cvs->int_ref_count--;
	return 0;
err_out:
	dev_err(cvs->dev, "%s:error! val %d (!=1)\n", __func__, val);
	return -EIO;
}

int cvs_acquire_camera_sensor(struct cvs_mipi_config *config,
			      cvs_privacy_callback_t callback, void *handle,
			      struct cvs_camera_status *status)
{
	int ret;

	if (!cvs)
		return -EINVAL;

	ret = cvs_acquire_camera_sensor_internal();
	if (!ret) {
		spin_lock(&cvs->lock);
		cvs->owner = (cvs->ref_count <= 0) ? CVS_CAMERA_IPU :
						     cvs->owner;
		cvs->ref_count = (cvs->ref_count <= 0) ? 1 :
							 (cvs->ref_count + 1);
		spin_unlock(&cvs->lock);
	}
	return (ret) ? ret : 0;
}
EXPORT_SYMBOL_GPL(cvs_acquire_camera_sensor);

int cvs_release_camera_sensor(struct cvs_camera_status *status)
{
	int ret;

	if (!cvs)
		return -EINVAL;

	ret = cvs_release_camera_sensor_internal();
	if (!ret) {
		spin_lock(&cvs->lock);
		cvs->ref_count = (cvs->ref_count > 0) ? (cvs->ref_count - 1) :
							0;
		cvs->owner = (cvs->ref_count == 0) ? CVS_CAMERA_CVS :
						     cvs->owner;
		spin_unlock(&cvs->lock);
	}
	return (ret) ? ret : 0;
}
EXPORT_SYMBOL_GPL(cvs_release_camera_sensor);

#ifdef DEBUG_CVS
int cvs_get_state()
{
	if (!cvs)
		return -EINVAL;

	if (cvs_acquire_camera_sensor_internal())
		return -EINVAL;

	if (cvs_read_i2c(GET_DEVICE_STATE, (char *)&cvs->cvs_state,
			 sizeof(char)) <= 0)
		return -EIO;

	cvs_release_camera_sensor_internal();
	return 0;
}

int cvs_exec_cmd(enum cvs_command command)
{
	int rc;

	if (!cvs)
		return -EINVAL;

	if (cvs->i2c_shared && cvs->icvs_state != CV_FW_DOWNLOADING_STATE)
		if (cvs_acquire_camera_sensor_internal())
			return -EINVAL;

	switch (command) {
	case GET_DEVICE_STATE:
		rc = cvs_read_i2c(GET_DEVICE_STATE, (char *)&cvs->cvs_state,
				  sizeof(char));
		if (rc <= 0)
			goto err_out;
		break;

	case GET_FW_VERSION:
		rc = cvs_read_i2c(GET_FW_VERSION, (char *)&cvs->ver,
				  sizeof(struct cvs_fw));
		if (rc <= 0)
			goto err_out;
		break;

	case GET_VID_PID:
		rc = cvs_read_i2c(GET_VID_PID, (char *)&cvs->id,
				  sizeof(struct cvs_id));
		if (rc <= 0)
			goto err_out;
		break;

	default:
		pr_err("%s:command %x not implemented\n", __func__, command);
	}

	if (cvs->i2c_shared && cvs->icvs_state != CV_FW_DOWNLOADING_STATE)
		if(cvs_release_camera_sensor_internal())
			return -EINVAL;

	return 0;

err_out:
	if (cvs->i2c_shared && cvs->icvs_state != CV_FW_DOWNLOADING_STATE)
		cvs_release_camera_sensor_internal();


	pr_err("%s:CVs command 0x%x failed, cvs_read_i2c: %d\n", __func__,
	       command, rc);
	return -EIO;
}

static void cvs_state_str(const enum cvs_state stat, u8 *buf)
{
	int n = 0;
	
	*buf = 0;
	if (stat == DEVICE_OFF_STATE) {
		sprintf(buf, "%s", "DEVICE_OFF_STATE");
	}
	else {
		if (stat & PRIVACY_ON_BIT_MASK)
			n += sprintf(buf + n, "%s, ", "PRIVACY_ON_BIT_MASK");
		if (stat & DEVICE_ON_BIT_MASK)
			n += sprintf(buf + n, "%s, ", "DEVICE_ON_BIT_MASK");
		if (stat & SENSOR_OWNER_BIT_MASK)
			n += sprintf(buf + n, "%s, ", "SENSOR_OWNER_BIT_MASK");
		if (stat & DEVICE_DWNLD_STATE_MASK)
			n += sprintf(buf + n, "%s, ", "DEVICE_DWNLD_STATE_MASK");
		if (stat & DEVICE_DWNLD_ERROR_MASK)
			n += sprintf(buf + n, "%s, ", "DEVICE_DWNLD_ERROR_MASK");
		if (stat & DEVICE_DWNLD_BUSY_MASK)
			n += sprintf(buf + n, "%s, ", "DEVICE_DWNLD_BUSY_MASK");
	}
	return;
}

static ssize_t coredump_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	u8 stat_name[256] = "";

	cvs_state_str(cvs->cvs_state, stat_name);

	return sysfs_emit(
		buf,
		"CVS VID/PID     : 0x%x 0x%x\n"
		"CVS Firmware Ver: %d.%d.%d.%d (0x%x.0x%x.0x%x.0x%x)\n"
		"CVS Device State: 0x%x (%s)\n"
		"Reference Count : %d (internal %d)\n"
		"CVS Owner       : %s\n"
		"I2C Shared      : %d\n",
		cvs->id.vid, cvs->id.pid, cvs->ver.major, cvs->ver.minor,
		cvs->ver.hotfix, cvs->ver.build, cvs->ver.major, cvs->ver.minor,
		cvs->ver.hotfix, cvs->ver.build, cvs->cvs_state, stat_name,
		cvs->ref_count, cvs->int_ref_count,
		(cvs->owner == CVS_CAMERA_CVS) ?
			"CVS" :
			((cvs->owner == CVS_CAMERA_IPU) ? "IPU" : "none"),
		cvs->i2c_shared);
}
static DEVICE_ATTR_RO(coredump);

static ssize_t cmd_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	if (sysfs_streq(buf, "reset"))
		schedule_work(&cvs->rst_task);
	else if (sysfs_streq(buf, "acquire"))
		cvs_acquire_camera_sensor(NULL, NULL, NULL, NULL);
	else if (sysfs_streq(buf, "release"))
		cvs_release_camera_sensor(NULL);
	else if (sysfs_streq(buf, "state"))
		cvs_get_state();
	else if (sysfs_streq(buf, "version"))
		cvs_exec_cmd(GET_FW_VERSION);
	else if (sysfs_streq(buf, "id"))
		cvs_exec_cmd(GET_VID_PID);
	else if (sysfs_streq(buf, "update"))
		pr_err("%s:update not implemented\n", __func__);
	else
		pr_err("%s:invalid %s command\n", __func__, buf);

	return count;
}

static ssize_t cmd_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return sysfs_emit(
		buf, "command: %s\n",
		"[coredump, reset, acquire, release, state, version, id, update]");
}
static DEVICE_ATTR_RW(cmd);

static struct attribute *cvs_attrs[] = {
	&dev_attr_coredump.attr,
	&dev_attr_cmd.attr,
	NULL,
};
ATTRIBUTE_GROUPS(cvs);
#endif

#ifdef CONFIG_PM
static int cvs_suspend(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct intel_cvs *icvs = i2c_get_clientdata(i2c);
	int ret = 0; 
	u64 timeout;

	/* Wait for FW update complete */
	if (icvs->cap == ICVS_FULLCAP && cvs->fw_dl_task_finished != true) {	
		cvs->close_fw_dl_task = true;
		cvs->hostwake_event_arg = 1;
		wake_up_interruptible(&cvs->hostwake_event);
		timeout = msecs_to_jiffies(WAIT_HOST_WAKE_FLASH_LONG_MS * FW_MAX_RETRY);
		ret = wait_event_interruptible_timeout(cvs->update_complete_event,
							cvs->update_complete_event_arg == 1,
							timeout);
		if (ret <= 0) {
			dev_err(icvs->dev, "%s: Failed to wait FW update complete\n", __func__);
			ret = -EAGAIN;
		}
	}
	if (icvs->cap == ICVS_FULLCAP) {	
		/* Disable wdt & IRQ */
		hrtimer_cancel(&icvs->wdt);
		disable_irq(icvs->irq);
	}

	dev_info(icvs->dev, "%s completed\n", __func__);
	return ret;
}

static int cvs_resume(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct intel_cvs *icvs = i2c_get_clientdata(i2c);
	int val = -1, retry = FW_MAX_RETRY;

	/* Wait for bridge ready */
	while (val < 0 && retry--) {
		val = gpiod_get_value_cansleep(cvs->resp);
		mdelay(WAIT_HOST_WAKE_NORMAL_MS);
	}

	if (val < 0) 
		dev_err(icvs->dev, "%s: Failed to read gpio via usb bridge\n", __func__);

	if (icvs->cap == ICVS_FULLCAP) {	
		/* Restart IRQ and wdt, hrtimer_start would start by fw_dl_task */
		enable_irq(icvs->irq);
		schedule_work(&icvs->fw_dl_task);
	}

	dev_info(icvs->dev, "%s completed\n", __func__);
	return 0;
}

static const struct dev_pm_ops icvs_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(cvs_suspend, cvs_resume)
};
#define ICVS_DEV_PM_OPS (&icvs_pm_ops)
#else
#define ICVS_DEV_PM_OPS NULL 
#endif /* CONFIG_PM */

static struct acpi_device_id acpi_cvs_ids[] = { { "INTC10CF" }, /* MTL */
						{ "INTC10DE" }, /* LNL */
						{ "INTC10E0" }, /* ARL */
						{ "INTC10E1" }, /* PTL */
						{ /* END OF LIST */ } };
MODULE_DEVICE_TABLE(acpi, acpi_cvs_ids);


static struct i2c_driver cvs_i2c_driver = {
	.driver = {
		.name = "Intel CVS driver",
		.owner = THIS_MODULE,
		.acpi_match_table = ACPI_PTR(acpi_cvs_ids),
		.dev_groups = cvs_groups,
		.pm = pm_ptr(ICVS_DEV_PM_OPS),
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

MODULE_AUTHOR("Lifu Wang <lifu.wang@intel.com>");
MODULE_AUTHOR("Israel Cepeda <israel.a.cepeda.lopez@intel.com>");
MODULE_AUTHOR("Hemanth Rachakonda <hemanth.rachakonda@intel.com>");
MODULE_AUTHOR("Srinivas Alla <alla.srinivas@intel.com>");
MODULE_DESCRIPTION("Intel CVS driver");
MODULE_LICENSE("GPL v2");
