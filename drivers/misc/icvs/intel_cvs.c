// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 * Copyright (C) 2024 Intel Corporation.
 *
 */

#include <linux/acpi.h>
#include <linux/intel_cvs.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/vmalloc.h>

#include "cvs_gpio.h"
#include "intel_cvs_update.h"

struct intel_cvs *cvs;

static irqreturn_t cvs_irq_handler(int irq, void *devid)
{
	struct intel_cvs *icvs = devid;

	icvs->hostwake_event_arg = 1;
	wake_up_interruptible(&icvs->hostwake_event);
	return IRQ_RETVAL(true);
}

static int cvs_init(struct intel_cvs *icvs)
{
	int ret = -EINVAL;

	if (!icvs || !icvs->dev || !icvs->rst)
		return -EINVAL;

	ret = devm_request_irq(icvs->dev, icvs->irq, cvs_irq_handler,
						   IRQF_ONESHOT | IRQF_NO_SUSPEND,
						   dev_name(icvs->dev), icvs);
	if (ret)
		dev_err(icvs->dev, "Failed to request irq");
	return ret;
}

static int find_oem_prod_id(acpi_handle handle, const char *method_name,
			    unsigned long long *value)
{
	acpi_status status;

	status = acpi_evaluate_integer(handle, (acpi_string)method_name, NULL,
								   value);

	if (ACPI_FAILURE(status)) {
		dev_err(cvs->dev, "%s: ACPI method %s not found", __func__,
				method_name);
		return status;
	}

	dev_info(cvs->dev, "%s: ACPI method %s returned oem_prod_id:0x%llx",
			 __func__, method_name, *value);
	return 0;
}

static int cvs_common_probe(struct device *dev, bool is_i2c)
{
	struct intel_cvs *icvs;
	acpi_handle handle;
	int ret = -ENODEV;

	if (!dev)
		return -ENODEV;

	icvs = devm_kzalloc(dev, sizeof(struct intel_cvs), GFP_KERNEL);
	if (!icvs)
		return -ENOMEM;
	icvs->dev = dev;
	icvs->has_i2c = is_i2c;
	if (is_i2c) {
		struct i2c_client *i2c = to_i2c_client(dev);
		i2c_set_clientdata(i2c, icvs);
		dev_set_drvdata(dev, icvs);
		dev_info(dev, "%s: probed as i2c device", __func__);
	} else {
		dev_set_drvdata(dev, icvs);
		dev_info(dev, "%s: probed as platform device (GPIO-only)", __func__);
	}
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
		dev_err(icvs->dev, "Number of GPIOs not supported: %d", ret);
		ret = -EINVAL;
		goto exit;
	}

	ret = devm_acpi_dev_add_driver_gpios(icvs->dev,
			icvs->cap == ICVS_FULLCAP ?
			icvs_acpi_gpios : icvs_acpi_lgpios);
	if (ret) {
		dev_err(icvs->dev, "Failed to add driver gpios");
		goto exit;
	}

	/* Request GPIO */
	icvs->req = devm_gpiod_get(icvs->dev, "req", GPIOD_OUT_HIGH);
	if (IS_ERR(icvs->req)) {
		dev_err(icvs->dev, "Get request gpiod failed. Do deferred probing");
		return dev_err_probe(icvs->dev, -EPROBE_DEFER,
							 "Do deferred probing as request gpiod failed");
	}

	/* Response GPIO */
	icvs->resp = devm_gpiod_get(icvs->dev, "resp", GPIOD_IN);
	if (IS_ERR(icvs->resp)) {
		dev_err(icvs->dev, "Get response gpiod failed. Do deferred probing");
		return dev_err_probe(icvs->dev, -EPROBE_DEFER,
							 "Do deferred probing as response gpiod failed");
	}

	if (icvs->cap == ICVS_FULLCAP) {
		/* Reset GPIO */
		icvs->rst = devm_gpiod_get(icvs->dev, "rst", GPIOD_OUT_HIGH);
		if (IS_ERR(icvs->rst)) {
			dev_err(icvs->dev, "Get reset gpiod failed. Do deferred probing");
			return dev_err_probe(icvs->dev, -EPROBE_DEFER,
								 "Do deferred probing as reset gpiod failed");
		}

		/* Wake Interrupt */
		ret = acpi_dev_gpio_irq_get_by(ACPI_COMPANION(icvs->dev),
									   "wake-gpio", 0);
		if (ret > 0) {
			icvs->irq = ret;
		} else {
			dev_err(icvs->dev, "Failed to get right wake interrupt:%d", ret);
			ret = -EINVAL;
			goto exit;
		}

		INIT_WORK(&icvs->fw_dl_task, cvs_fw_dl_thread);
		init_waitqueue_head(&icvs->hostwake_event);
		init_waitqueue_head(&icvs->update_complete_event);
		init_waitqueue_head(&icvs->lvfs_fwdl_complete_event);
		icvs->fw_dl_task_finished = false;

		if (icvs->has_i2c) {
			ret = cvs_init(icvs);
			if (ret)
				dev_err(icvs->dev, "Failed to initialize\n");

			find_oem_prod_id(handle, "OPID", &icvs->oem_prod_id);

			ret = cvs_find_magic_num_support(icvs);
			if (ret)
				goto exit;

			if (icvs->magic_num_support) {
				ret = cvs_get_device_cap(&icvs->cv_fw_capability);
				if (ret)
					goto exit;
			}

			ret = cvs_write_i2c(SET_HOST_IDENTIFIER, NULL, 0);
			if (ret) {
				dev_err(cvs->dev, "%s:set_host_identifier cmd failed", __func__);
				goto exit;
			}
		}
	}

	mdelay(FW_PREPARE_MS);

	ret = cvs_acquire_camera_sensor_internal();
	if (ret) {
		dev_err(cvs->dev, "%s:Acquire sensor fail", __func__);
		goto exit;
	} else {
		dev_info(cvs->dev, "%s:Transfer of ownership success",
			 __func__);
	}

	icvs->icvs_sensor_state = CV_SENSOR_VISION_ACQUIRED_STATE;
	acpi_dev_clear_dependencies(ACPI_COMPANION(icvs->dev));

exit:
	if (ret)
		devm_kfree(icvs->dev, icvs);

	return ret;
}

static int cvs_i2c_probe(struct i2c_client *i2c)
{
	return cvs_common_probe(&i2c->dev, true);
}

static int cvs_platform_probe(struct platform_device *pdev)
{
	return cvs_common_probe(&pdev->dev, false);
}

static void cvs_exit(struct intel_cvs *icvs)
{
	if (icvs && icvs->dev && icvs->rst)
		devm_free_irq(icvs->dev, icvs->irq, icvs);
}

static void cvs_common_remove(struct device *dev, bool is_i2c)
{
	struct intel_cvs *icvs = dev_get_drvdata(dev);
	if (is_i2c) {
		dev_info(dev, "%s (i2c)\n", __func__);
		icvs = i2c_get_clientdata(to_i2c_client(dev));
	} else {
		dev_info(dev, "%s (platform)\n", __func__);
	}
	if (icvs) {
		cvs->close_fw_dl_task = true;

		if (icvs->cap == ICVS_FULLCAP) {
			if (cvs->fw_dl_task_started && !cvs->fw_dl_task_finished) {
				dev_info(cvs->dev, "%s:signal cvs_fw_dl_thread() to stop",
						 __func__);
				cvs->hostwake_event_arg = 1;
				wake_up_interruptible(&cvs->hostwake_event);

				dev_info(cvs->dev, "%s:Wait for cvs_fw_dl_thread() to stop",
						 __func__);
				wait_event_interruptible(cvs->update_complete_event,
					cvs->update_complete_event_arg == 1);
				dev_info(cvs->dev, "%s:cvs_fw_dl_thread() stopped", __func__);
				mdelay(WAIT_HOST_RELEASE_MS);
			}

			/* reset vision chip */
			if (cvs_reset_cv_device()) {
				dev_err(cvs->dev, "%s:CV reset fail after flash", __func__);
				return;
			}
			cvs->icvs_state = CV_INIT_STATE;

			cvs_exit(icvs);
		}
		if (cvs->fw_buffer)
			vfree(cvs->fw_buffer);

		devm_kfree(dev, icvs);
	}
}

static void cvs_i2c_remove(struct i2c_client *i2c)
{
	cvs_common_remove(&i2c->dev, true);
}

static void cvs_platform_remove(struct platform_device *pdev)
{
	cvs_common_remove(&pdev->dev, false);
}

int cvs_read_i2c(u16 cmd, char *data, int size)
{
	struct intel_cvs *ctx = cvs;
	if (!ctx || !ctx->has_i2c)
		return -EOPNOTSUPP;
	struct i2c_client *i2c = container_of(cvs->dev, struct i2c_client, dev);
	int cnt;
	char *in_data;

	in_data = devm_kzalloc(ctx->dev, (CVMAGICNUMSIZE + size), GFP_KERNEL);
	if (!in_data) {
		dev_err(cvs->dev, "%s:Buffer allocation failed", __func__);
		return -ENOMEM;
	}
	u16 cvs_cmd = (((cmd) >> 8) & 0x00ff) | (((cmd) << 8) & 0xff00);

	if (size < 0 || !data)
		return -EINVAL;

	cnt = i2c_master_send(i2c, (const char *)&cvs_cmd, sizeof(u16));
	if (cnt != sizeof(u16)) {
		dev_err(&i2c->dev, "%s:cmd:%x count:%d (!=2)\n", __func__, cmd,
				cnt);
		return -EIO;
	}

	if (ctx->magic_num_support) {
		u32 magic_num_received;

		cnt = i2c_master_recv(i2c, in_data, size + CVMAGICNUMSIZE);
		magic_num_received = ((u32 *)in_data)[0];
		if (magic_num_received ==  CVMAGICNUM) {
			dev_dbg(&i2c->dev, "%s:Valid magic number", __func__);
			memcpy(data, in_data + CVMAGICNUMSIZE, size);
		} else {
			dev_dbg(&i2c->dev, "%s:Invalid magic number", __func__);
			return -EIO;
		}
	} else {
		cnt = i2c_master_recv(i2c, (char *)data, size);
	}

	return cnt;
}

int cvs_acquire_camera_sensor_internal(void)
{
	int val, retry = 20;

	if (!cvs)
		return -EINVAL;

	if (cvs->icvs_state == CV_FW_DOWNLOADING_STATE ||
		cvs->icvs_state == CV_FW_FLASHING_STATE)
		return -EBUSY;

	if (cvs->owner != CVS_CAMERA_IPU) {
		do {
			gpiod_set_value_cansleep(cvs->req, 0);
			mdelay(GPIO_READ_DELAY_MS);
			val = gpiod_get_value_cansleep(cvs->resp);
		} while (val != 0 && retry--);

		if (val != 0) {
			dev_err(cvs->dev, "%s:error! val %d (!=0)", __func__, val);
			return -EIO;
		}
	}

	cvs->owner = CVS_CAMERA_IPU;
	cvs->int_ref_count++;
	return 0;
}

int cvs_release_camera_sensor_internal(void)
{
	int val, retry = 20;

	if (!cvs)
		return -EINVAL;

	if (cvs->icvs_state == CV_FW_DOWNLOADING_STATE)
		return -EBUSY;

	if (cvs->int_ref_count == 0)
		return 0;

	if (cvs->owner != CVS_CAMERA_CVS && cvs->int_ref_count == 1) {
		do {
			gpiod_set_value_cansleep(cvs->req, 1);
			mdelay(GPIO_READ_DELAY_MS);
			val = gpiod_get_value_cansleep(cvs->resp);
		} while (val != 1 && retry--);

		if (val != 1) {
			dev_err(cvs->dev, "%s:error! val %d (!=1)", __func__, val);
			return -EIO;
		}
	}

	cvs->int_ref_count--;
	cvs->owner = (cvs->int_ref_count == 0) ? CVS_CAMERA_CVS : CVS_CAMERA_IPU;
	return 0;
}

#ifdef DEBUG_CVS
enum cvs_state cvs_state;
int cvs_exec_cmd(enum cvs_command command)
{
	int rc;

	if (!cvs)
		return -EINVAL;

	if (cvs->icvs_state == CV_FW_DOWNLOADING_STATE ||
		cvs->icvs_state == CV_FW_FLASHING_STATE) {
		dev_err(cvs->dev, "%s:Device busy, cmd:0X%x can't be queried now",
				__func__, command);
		cvs_state = cvs->cv_fw_state;
		return -EBUSY;
	}

	switch (command) {
	case GET_DEVICE_STATE:
		rc = cvs_read_i2c(GET_DEVICE_STATE, (char *)&cvs_state,
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
		dev_err(cvs->dev, "%s:command %x not implemented\n", __func__,
			command);
	}

	return 0;

err_out:
	dev_err(cvs->dev, "%s:CVS command 0x%x failed, cvs_read_i2c: %d",
			__func__, command, rc);
	return -EIO;
}

static void cvs_state_str(const enum cvs_state stat, u8 *buf)
{
	int n = 0;

	*buf = 0;
	if (stat == DEVICE_OFF_STATE) {
		sprintf(buf, "%s", "DEVICE_OFF_STATE");
	} else {
		if (stat & PRIVACY_ON_BIT_MASK)
			n += sprintf(buf + n, "%s, ", "PRIVACY_ON_BIT_MASK");
		if (stat & DEVICE_ON_BIT_MASK)
			n += sprintf(buf + n, "%s, ", "DEVICE_ON_BIT_MASK");
		if (stat & SENSOR_OWNER_BIT_MASK)
			n += sprintf(buf + n, "%s, ", "SENSOR_OWNER_BIT_MASK");
		if (stat & DEVICE_DWNLD_STATE_MASK)
			n += sprintf(buf + n, "%s, ",
				     "DEVICE_DWNLD_STATE_MASK");
		if (stat & DEVICE_DWNLD_ERROR_MASK)
			n += sprintf(buf + n, "%s, ",
				     "DEVICE_DWNLD_ERROR_MASK");
		if (stat & DEVICE_DWNLD_BUSY_MASK)
			n += sprintf(buf + n, "%s, ", "DEVICE_DWNLD_BUSY_MASK");
	}
}

static ssize_t coredump_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	u8 stat_name[256] = "";

	cvs_state_str(cvs_state, stat_name);
	return sysfs_emit(buf,
		"CVS VID/PID     : 0x%x 0x%x\n"
		"CVS Firmware Ver: %d.%d.%d.%d (0x%x.0x%x.0x%x.0x%x)\n"
		"CVS Device State: 0x%x (%s)\n"
		"Reference Count : %d\n"
		"CVS Owner       : %s\n",
		cvs->id.vid, cvs->id.pid, cvs->ver.major, cvs->ver.minor,
		cvs->ver.hotfix, cvs->ver.build, cvs->ver.major, cvs->ver.minor,
		cvs->ver.hotfix, cvs->ver.build, cvs_state, stat_name,
		cvs->int_ref_count,
		((cvs->owner == CVS_CAMERA_NONE) ? "NONE" :
		 (cvs->owner == CVS_CAMERA_CVS)	 ? "CVS" :
		 (cvs->owner == CVS_CAMERA_IPU)	 ? "HOST" : "UNKNOWN"));
}
static DEVICE_ATTR_RO(coredump);

static ssize_t cmd_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	if (sysfs_streq(buf, "state"))
		cvs_exec_cmd(GET_DEVICE_STATE);
	else if (sysfs_streq(buf, "version"))
		cvs_exec_cmd(GET_FW_VERSION);
	else if (sysfs_streq(buf, "id"))
		cvs_exec_cmd(GET_VID_PID);
	else
		dev_err(cvs->dev, "%s:invalid %s command\n", __func__, buf);

	return count;
}

static ssize_t cmd_show(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	return sysfs_emit(buf, "command: %s\n",
			  "[coredump, state, version, id]");
}
static DEVICE_ATTR_RW(cmd);
#endif //DEBUG_CVS

static ssize_t cvs_ctrl_data_pre_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	unsigned long flags;
	int count;

	if (!buf) {
		dev_err(cvs->dev, "%s: buff is null", __func__);
		return -EINVAL;
	}

	count = sizeof(struct cvs_to_plugin_interface);
	memset(&cvs->info_fwupd, 0, sizeof(struct ctrl_data_fwupd));
	cvs->fw_dl_task_finished = false;
	cvs->fw_dl_task_started = false;

	if (cvs_get_fwver_vid_pid()) {
		dev_err(cvs->dev, "%s:Not able to read vid/pid", __func__);
		return -EIO;
	}
	dev_info(cvs->dev, "%s:Device fw version is %d.%d.%d.%d", __func__,
			 cvs->ver.major, cvs->ver.minor, cvs->ver.hotfix, cvs->ver.build);

	/* Device vid,pid */
	cvs->cvs_to_plugin.vid = cvs->id.vid;
	cvs->cvs_to_plugin.pid = cvs->id.pid;
	cvs->cvs_to_plugin.opid = cvs->oem_prod_id;
	cvs->cvs_to_plugin.dev_capabilities =
		cvs->cv_fw_capability.dev_capability;

	/* Device FW version */
	cvs->cvs_to_plugin.major = cvs->ver.major;
	cvs->cvs_to_plugin.minor = cvs->ver.minor;
	cvs->cvs_to_plugin.hotfix = cvs->ver.hotfix;
	cvs->cvs_to_plugin.build = cvs->ver.build;

	spin_lock_irqsave(&cvs->buffer_lock, flags);
	memcpy(buf, &cvs->cvs_to_plugin, sizeof(struct cvs_to_plugin_interface));
	spin_unlock_irqrestore(&cvs->buffer_lock, flags);
	return count;
}

static ssize_t cvs_ctrl_data_pre_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long flags;
	struct file *f;
	int fw_bin_size;
	ssize_t bytes_read = 0;

	if (!buf) {
		dev_err(cvs->dev, "%s:buff is null", __func__);
		return -EINVAL;
	}

	if (count == 0 || count != sizeof(struct plugin_to_cvs_interface)) {
		dev_err(cvs->dev, "%s:Wrong count:%x", __func__, (unsigned int)count);
		return -EINVAL;
	}

	spin_lock_irqsave(&cvs->buffer_lock, flags);
	memcpy(&cvs->plugin_to_cvs, buf, count);
	spin_unlock_irqrestore(&cvs->buffer_lock, flags);

	dev_dbg(cvs->dev, "%s:dl_time:%x, fl_time:%x, retry_cnt:%x, fw_bin_fd:%x",
			__func__, cvs->plugin_to_cvs.max_download_time,
			cvs->plugin_to_cvs.max_flash_time,
			cvs->plugin_to_cvs.max_fwupd_retry_count,
			cvs->plugin_to_cvs.fw_bin_fd);

	/* Get the file structure from the file descriptor */
	f = fget(cvs->plugin_to_cvs.fw_bin_fd);
	if (!f) {
		dev_err(cvs->dev, "%s:Bad file descriptor", __func__);
		return -EBADF;
	}

	fw_bin_size = generic_file_llseek(f, 0, SEEK_END);
	dev_dbg(cvs->dev, "%s:Calculated fw_bin size:%x bytes",
			__func__, fw_bin_size);
	generic_file_llseek(f, -fw_bin_size, SEEK_CUR);

	cvs->fw_buffer_size = fw_bin_size;
	cvs->max_flashtime_ms = cvs->plugin_to_cvs.max_flash_time;
	cvs->fw_update_retries = cvs->plugin_to_cvs.max_fwupd_retry_count;
	cvs->fw_buffer = vmalloc(cvs->fw_buffer_size);

	if (IS_ERR_OR_NULL(cvs->fw_buffer)) {
		dev_err(cvs->dev, "%s:No memory for fw_buffer", __func__);
		fput(f);
		return -ENOMEM;
	}
	dev_dbg(cvs->dev, "%s:fw_buffer allocated at:%p",
			__func__, cvs->fw_buffer);
	/* Read FW binary using file descriptor */
	bytes_read = kernel_read(f, cvs->fw_buffer, cvs->fw_buffer_size,
							 &f->f_pos);
	fput(f);
	if (bytes_read != cvs->fw_buffer_size) {
		dev_err(cvs->dev, "%s:kernel_read failed with bytes_read:%lx",
				__func__, bytes_read);
		return -EIO;
	} else {
		dev_info(cvs->dev, "%s:Full fw_buffer received. Start fw_download",
				 __func__);
		schedule_work(&cvs->fw_dl_task);
		cvs->fw_dl_task_started = true;
	}
	return count;
}

static ssize_t cvs_ctrl_data_fwupd_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned long flags;
	unsigned int count = sizeof(struct ctrl_data_fwupd);

	if (!buf) {
		dev_err(cvs->dev, "%s:buff is null", __func__);
		return -EINVAL;
	}
	cvs->info_fwupd.fw_dl_finshed = cvs->fw_dl_task_finished;
	cvs->info_fwupd.dev_state     = cvs->cv_fw_state;
	cvs->info_fwupd.fw_upd_retries = cvs->fw_update_retries;

	spin_lock_irqsave(&cvs->buffer_lock, flags);
	memcpy(buf, &cvs->info_fwupd, count);
	spin_unlock_irqrestore(&cvs->buffer_lock, flags);

	if (cvs->info_fwupd.total_packets == cvs->info_fwupd.num_packets_sent) {
		cvs->info_fwupd.fw_dl_finshed = true;
		cvs->lvfs_fwdl_complete_event_arg = 1;
		wake_up_interruptible(&cvs->lvfs_fwdl_complete_event);
	}
	return count;
}

static DEVICE_ATTR_RW(cvs_ctrl_data_pre);
static DEVICE_ATTR_RO(cvs_ctrl_data_fwupd);

static struct attribute *cvs_attrs[] = {
#ifdef DEBUG_CVS
	&dev_attr_coredump.attr,
	&dev_attr_cmd.attr,
#endif
	&dev_attr_cvs_ctrl_data_pre.attr,
	&dev_attr_cvs_ctrl_data_fwupd.attr,
	NULL,
};
ATTRIBUTE_GROUPS(cvs);

#ifdef CONFIG_PM
static int cvs_suspend(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct intel_cvs *icvs = i2c_get_clientdata(i2c);
	int ret = 0;

	dev_info(icvs->dev, "%s:entered\n", __func__);
	if (icvs->cap == ICVS_FULLCAP && cvs->fw_dl_task_finished != true) {
		icvs->cv_suspend = true;
		cvs->close_fw_dl_task = true;
		cvs->hostwake_event_arg = 1;
		wake_up_interruptible(&cvs->hostwake_event);
		/* Wait for fw update to cancel */
		dev_info(icvs->dev, "%s:wait for fw update cancel", __func__);
		flush_work(&icvs->fw_dl_task);
		dev_info(icvs->dev, "%s:fw update cancelled", __func__);
	}

	if (icvs->cap == ICVS_FULLCAP) {
		/* Disable IRQ */
		disable_irq(icvs->irq);
	}

	dev_info(icvs->dev, "%s:completed", __func__);
	return ret;
}

static int cvs_resume(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct intel_cvs *icvs = i2c_get_clientdata(i2c);
	int val = -1;

	dev_info(icvs->dev, "%s entered", __func__);
	/* Check if bridge is ready */
	val = gpiod_get_value_cansleep(icvs->resp);
	if (val != 0)
		dev_err(icvs->dev, "%s:Wrong gpio_response val:%x read via bridge",
				__func__, val);

	if (icvs->cap == ICVS_FULLCAP) {
		icvs->cv_suspend = false;
		icvs->close_fw_dl_task = false;

		/* Start IRQ */
		enable_irq(icvs->irq);
		if (cvs->fw_dl_task_started && !cvs->fw_dl_task_finished) {
			schedule_work(&icvs->fw_dl_task);
			cvs->fw_dl_task_started = true;
		}
	}

	dev_info(icvs->dev, "%s:completed", __func__);
	return 0;
}

static const struct dev_pm_ops icvs_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(cvs_suspend, cvs_resume) };

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

static struct platform_driver cvs_platform_driver = {
	.probe	= cvs_platform_probe,
	.remove	= cvs_platform_remove,
	.driver = {
		.name = "intel-cvs-platform",
		.acpi_match_table = ACPI_PTR(acpi_cvs_ids),
		.dev_groups = cvs_groups,
		.pm = pm_ptr(ICVS_DEV_PM_OPS),
	},
};

static int __init icvs_init(void)
{
	int ret, pret;

	ret = i2c_add_driver(&cvs_i2c_driver);
	if (ret)
		pr_err("Failed to register I2C driver: %d\n", ret);

	pret = platform_driver_register(&cvs_platform_driver);
	if (pret)
		pr_err("Failed to register platform driver: %d\n", pret);

	/* Return 0 if at least one succeeds */
	return (ret && pret) ? ret : 0;
}
module_init(icvs_init);

static void __exit icvs_exit(void)
{
	i2c_del_driver(&cvs_i2c_driver);
	platform_driver_unregister(&cvs_platform_driver);
}
module_exit(icvs_exit);

MODULE_AUTHOR("Lifu Wang <lifu.wang@intel.com>");
MODULE_AUTHOR("Israel Cepeda <israel.a.cepeda.lopez@intel.com>");
MODULE_AUTHOR("Hemanth Rachakonda <hemanth.rachakonda@intel.com>");
MODULE_AUTHOR("Srinivas Alla <alla.srinivas@intel.com>");
MODULE_DESCRIPTION("Intel CVS driver");
MODULE_LICENSE("GPL v2");
MODULE_SOFTDEP("pre: usbio gpio-usbio i2c-usbio");
