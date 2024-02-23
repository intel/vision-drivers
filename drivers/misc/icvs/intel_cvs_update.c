/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * Intel Computer Vision System driver
 *
 * Copyright (C) 2024 Intel Corporation.
 *
 */

#include <linux/intel_cvs.h>
#include "intel_cvs_update.h"

extern struct intel_cvs *cvs;

int cvs_write_i2c(u16 cmd, u8 *data, u32 len)
{
	struct i2c_client *client =
		container_of(cvs->dev, struct i2c_client, dev);
	int count;
	u16 cv_cmd = (((cmd) >> 8) & 0x00ff) | (((cmd) << 8) & 0xff00);

	dev_dbg(cvs->dev, "%s with client:%p, cmd:%x, len:%x", __func__, client,
		cmd, len);

	switch (cmd) {
	case FW_LOADER_START:
		dev_dbg(cvs->dev, "%s:Executing FW_loader_start", __func__);
		count = i2c_master_send(client, (const char *)&cv_cmd,
					sizeof(u16));
		if (count != sizeof(u16)) {
			dev_err(cvs->dev,
				"%s:FW_loader_start fail with count:%x",
				__func__, count);
			return -EIO;
		}
		break;
	case FW_LOADER_DATA:
		dev_dbg(cvs->dev, "%s:FW_loader_data with len:%x", __func__,
			len);
		count = i2c_master_send(client, data, len);
		if (count != len) {
			dev_err(cvs->dev,
				"%s:FW_loader_data fail with count:%x",
				__func__, count);
			return -EIO;
		}
		break;
	case FW_LOADER_END:
		dev_dbg(cvs->dev, "%s:Executing FW_loader_end", __func__);
		count = i2c_master_send(client, (const char *)&cv_cmd,
					sizeof(u16));
		mdelay(GPIO_WRITE_DELAY_MS);
		if (count != sizeof(u16)) {
			dev_err(cvs->dev, "%s:FW_loader_end fail with count:%x",
				__func__, count);
			return -EIO;
		}
		break;
	default:
		dev_err(cvs->dev, "%s:Invalid cmd type", __func__);
		return -EINVAL;
	}

	return 0;
}

int cvs_get_device_state(u8 *cv_fw_state)
{
	if (!cvs)
		return -EINVAL;

	if (cvs_read_i2c(GET_DEVICE_STATE, cv_fw_state, sizeof(char)) <= 0)
		return -EIO;

	return 0;
}

int cvs_wait_for_host_wake(u64 time_ms)
{
	int ret = -1;
	int64_t timeout = 0;

	dev_dbg(cvs->dev, "%s:Enter", __func__);

	/* wait for HOST_WAKE signal, timeout in time_ms */
	timeout = msecs_to_jiffies(time_ms);
	dev_dbg(cvs->dev, "%s with time_ms:%lld, timeout:%lld", __func__,
		time_ms, timeout);

	ret = wait_event_interruptible_timeout(
		cvs->hostwake_event, cvs->hostwake_event_arg == 1, timeout);

	if (ret <= 0) {
		dev_err(cvs->dev, "%s:hostwake wait timeout", __func__);
		return -ETIMEDOUT;
	} else {
		pr_debug("hostwake wait pass with ret:%d", ret);
		cvs->hostwake_event_arg = 0;
		return 0;
	}
}

int cvs_reset_cv_device(void)
{
	if (IS_ERR_OR_NULL(cvs)) {
		dev_err(cvs->dev, "%s:Invalid cvs context", __func__);
		return -EINVAL;
	}

	gpiod_set_value_cansleep(cvs->rst, 0);
	mdelay(GPIO_RESET_MS);
	gpiod_set_value_cansleep(cvs->rst, 1);
	mdelay(GPIO_RESET_MS);
	return 0;
}

int cvs_get_dev_state(void)
{
	int status = 0;
	struct intel_cvs *ctx = cvs;
	struct i2c_client *client =
		container_of(cvs->dev, struct i2c_client, dev);
	u8 fw_state = 0;

	if (IS_ERR_OR_NULL(ctx) || IS_ERR_OR_NULL(client)) {
		status = -EINVAL;
		dev_err(cvs->dev, "%s:Invalid cvs_context or client",
			__func__);
		goto xit;
	} else
		dev_dbg(cvs->dev, "%s:cvs_context:%p\n", __func__, ctx);

	if (ctx->i2c_shared) {
		if (ctx->icvs_sensor_state != CV_SENSOR_IPU_ACQUIRED_STATE) {
			if (cvs_acquire_camera_sensor_internal()) {
				dev_err(cvs->dev, "%s:Acquire sensor fail",__func__);
				return -EIO;
			} else {
				dev_dbg(cvs->dev, "%s:Acquire sensor pass",__func__);
				status = 0;
				ctx->icvs_sensor_state =
					CV_SENSOR_VISION_ACQUIRED_STATE;
			}
		} else {
			status = -EBUSY;
		}
	}

	if (!ctx->i2c_shared || (!status)) {
		status = cvs_get_device_state(&fw_state);
		dev_dbg(cvs->dev, "%s:I2C check FW state:%x, status:%x",
			__func__, fw_state, status);
	}

	if ((!status) && (fw_state & SENSOR_OWNER_BIT_MASK)) {
		ctx->cv_fw_state = fw_state;
		dev_dbg(cvs->dev, "%s:get fw_state success:%x", __func__,
			status);
	} else {
		dev_err(cvs->dev,
			"%s:firmware state error, try reboot. FW state:%x. - "
			"status:%x",
			__func__, fw_state, status);
	}

	if (ctx->icvs_sensor_state != CV_SENSOR_IPU_ACQUIRED_STATE) {
		if (ctx->i2c_shared) {
			if (cvs_release_camera_sensor_internal()) {
				dev_err(cvs->dev, "%s:Release sensor fail",__func__);
				return -EIO;
			} else {
				dev_dbg(cvs->dev, "%s:Release sensor pass",__func__);
				ctx->icvs_sensor_state = CV_SENSOR_RELEASED_STATE;
				status = 0;
			}
		} else {
			ctx->icvs_sensor_state = CV_SENSOR_RELEASED_STATE;
			status = 0;
		}
	}

xit:
	return status;
}

int cvs_dev_fw_dl_start(void)
{
	int status = 0;
	struct intel_cvs *ctx = cvs;
	u8 fw_state = 0;

	dev_info(cvs->dev, "%s:Enter", __func__);

	/* check CV FW state */
	status = cvs_get_device_state(&fw_state);
	dev_dbg(cvs->dev, "%s:cvs_get_device_state() fw_state:%x, status:%x",
		__func__, fw_state, status);

	if ((!status) && (fw_state & SENSOR_OWNER_BIT_MASK)) {
		status = cvs_write_i2c(FW_LOADER_START, NULL, 0);
		if (status)
			dev_err(cvs->dev, "%s:cvs_write_i2c() failed",
				__func__);
		else
			ctx->icvs_state = CV_FW_DOWNLOADING_STATE;
	}

	/* check CV FW state */
	if (!status) {
		status = cvs_get_device_state(&fw_state);
		dev_dbg(cvs->dev,
			"%s:cvs_get_device_state() fw_state:%x, status:%x",
			__func__, fw_state, status);
		if ((status) || ((fw_state & DEVICE_DWNLD_STATE_MASK) !=
				 DEVICE_DWNLD_STATE_MASK)) {
			status = -EIO;
			dev_err(cvs->dev,
				"%s:fail to enter download state. fwstate:%x, status:%x",
				__func__, fw_state, status);
		}
	}

	ctx->cv_fw_state = fw_state;
	dev_info(cvs->dev, "%s:Exit with status:%x", __func__, status);
	return status;
}

int cvs_dev_fw_dl_data(void)
{
	int status = 0;
	struct intel_cvs *ctx = cvs;
	u8 fw_state = DEVICE_DWNLD_STATE_MASK;
	u8 *fw_buff_ptr = NULL;
	u32 fw_size = 0;

	dev_info(cvs->dev, "%s:Enter", __func__);

	fw_buff_ptr = (u8 *)ctx->fw_buffer + sizeof(struct cvs_fw_header);
	fw_size = ctx->fw_buffer_size - sizeof(struct cvs_fw_header);

	dev_dbg(cvs->dev, "%s:fwBuff_ptr:%p, fw_size:%x", __func__,
		fw_buff_ptr, fw_size);

	while ((fw_size > 0) && (ctx->icvs_state != CV_STOPPING)) {
		int retry = FW_MAX_RETRY;

		if (ctx->close_fw_dl_task == true) {
			dev_err(cvs->dev, "%s:Received close_fw_dl_task true",
				__func__);
			goto err_exit;
		}

		do {
			if (ctx->close_fw_dl_task == true) {
				dev_info(cvs->dev,
					 "%s:Received close_fw_dl_task true",
					 __func__);
				goto err_exit;
			}

			/* copy data to outbuf */
			memcpy(ctx->out_buf, fw_buff_ptr,
			       I2C_PKT_SIZE < fw_size ?
				       I2C_PKT_SIZE :
				       fw_size);
			wmb(); /* Flush WC buffers after writing out_buf */

			dev_dbg(cvs->dev, "%s:memcpy() src:%p, dst:%p",
				__func__, fw_buff_ptr, ctx->out_buf);

			if ((!status) && (fw_state & DEVICE_DWNLD_STATE_MASK)) {
				status = cvs_write_i2c(FW_LOADER_DATA, ctx->out_buf,
						       I2C_PKT_SIZE);
				dev_dbg(cvs->dev,"%s:I2C write done with status:%x",
					__func__, status);
			}

			/* check CV FW state */
			if (!status) {
				/* Wait for Host Wake */
				cvs_wait_for_host_wake(
					WAIT_HOST_WAKE_NORMAL_MS);
				/* Check device state */
				status = cvs_get_device_state(&fw_state);
				dev_dbg(cvs->dev,
					"%s:I2C check fw_state:%x, status:%x",
					__func__, fw_state, status);

				if ((!status) &&
				    (fw_state & DEVICE_DWNLD_STATE_MASK) &&
				    (fw_state & DEVICE_DWNLD_BUSY_MASK)) {
					/* Check device state again */
					status = cvs_get_device_state(&fw_state);
					dev_err(cvs->dev,
						"%s:I2C re-check fw_state:%x, status:%x",
						__func__, fw_state, status);

					if (fw_state & DEVICE_DWNLD_BUSY_MASK) {
						dev_err(cvs->dev,
							"%s:I2C is busy for too long! fw_state:%x, status:%x",
							__func__, fw_state, status);
						break;
					}
				} else {
					dev_dbg(cvs->dev,
						"%s:got WRONG_1 fw_state:%x, cv_state:%x, status:%x ",
						__func__, fw_state, ctx->icvs_state, status);
				}
			}

			if (ctx->icvs_state == CV_STOPPING) {
				dev_err(cvs->dev, "%s:cv_state is CV_STOPPING",
					__func__);
				ctx->fw_update_retries = 0;
				break;
			}
		} while (--retry && (fw_state & DEVICE_DWNLD_ERROR_MASK));

		if ((fw_state & DEVICE_DWNLD_BUSY_MASK) ||
		    (fw_state & DEVICE_DWNLD_ERROR_MASK) ||
		    (ctx->icvs_state == CV_STOPPING) || (!fw_state)) {
			dev_err(cvs->dev,
				"%s:got WRONG_2 fw_state:%x, cv_state:%x, status:%x ",
				__func__, fw_state, ctx->icvs_state, status);
			status = -EIO;
			break;
		}

		fw_size -= I2C_PKT_SIZE;
		fw_buff_ptr += I2C_PKT_SIZE;
		ctx->cv_fw_state = fw_state;
	}

err_exit:
	dev_info(cvs->dev, "%s:Exit with status:%x, fw_state:%x, cv_state:%x",
		 __func__, status, fw_state, ctx->icvs_state);

	return status;
}

int cvs_dev_fw_dl_end(void)
{
	int status = 0;
	struct intel_cvs *ctx = cvs;
	u8 fw_state = 0;

	dev_info(cvs->dev, "%s:Enter", __func__);
	status = cvs_write_i2c(FW_LOADER_END, NULL, 0);
	dev_dbg(cvs->dev, "%s:I2C fw_loader_end status:%x", __func__, status);

	if (!status) {
		ctx->icvs_state = CV_FW_FLASHING_STATE;
		status = cvs_get_device_state(&fw_state);
		dev_dbg(cvs->dev,
			"%s:device_state() with fw_state = 0x%02x, status:%x",
			__func__, fw_state, status);
	}

	dev_info(cvs->dev, "%s:Exit with status:%x", __func__, status);
	return status;
}

int cvs_dev_fw_dl(void)
{
	int status = 0;
	struct intel_cvs *ctx = cvs;
	u8 retries = 0;
	const u8 max_retry = 5;

	dev_info(cvs->dev, "%s:Enter", __func__);

	status = cvs_dev_fw_dl_start();
	dev_dbg(cvs->dev, "%s:fw_download_start status:%x, fw_size= %x",
		__func__, status, ctx->fw_buffer_size);

	if (!status) {
		status = cvs_dev_fw_dl_data();
		dev_dbg(cvs->dev, "%s:fw_download_data status:%x", __func__,
			status);
	}

	/* End FW download, no matter if it's pass or fail */
	status = cvs_dev_fw_dl_end();
	dev_dbg(cvs->dev, "%s:fw_download_end status:%x", __func__, status);

	if (ctx->i2c_shared) {
		do {
			status = cvs_release_camera_sensor_internal();
			if (status) {
				dev_err(cvs->dev, "%s:Release sensor fail",
					__func__);
			} else {
				dev_dbg(cvs->dev, "%s:Release sensor pass",
					__func__);
				ctx->icvs_sensor_state =
					CV_SENSOR_RELEASED_STATE;
				break;
			}
			mdelay(WAIT_NORMAL_MS);
		} while (ctx->icvs_sensor_state ==
				 CV_SENSOR_VISION_ACQUIRED_STATE &&
			 retries++ < max_retry);
	}

	if (!status) {
		status = cvs_wait_for_host_wake(ctx->max_flashtime_ms);
	}

	while (ctx->ref_count) {
		dev_info(cvs->dev,
			 "%s:Camera is used by IPU, check again after %dms",
			 __func__, WAIT_NORMAL_MS);
		mdelay(WAIT_NORMAL_MS);
	}

	do {
		if (!status) {
			status = cvs_wait_for_host_wake(ctx->max_flashtime_ms);
			dev_info(cvs->dev,
				 "%s:wait HOST_WAKE signal status:%x",
				 __func__, status);

			status = cvs_get_dev_state();
			dev_info(
				cvs->dev,
				"%s:cvs_get_dev_state() status:%x, cv_fw_state:%x",
				__func__, status, ctx->cv_fw_state);
		}
	} while ((!ctx->cv_fw_state) ||
		 (ctx->cv_fw_state & DEVICE_DWNLD_BUSY_MASK));

	if (ctx->icvs_sensor_state == CV_SENSOR_RELEASED_STATE) {
		/* reset Vision chip */
		dev_info(cvs->dev,
			 "%s:Do cv reset only if sensor is released\n",
			 __func__);
		status = cvs_reset_cv_device();
		dev_info(cvs->dev, "%s:cvs_reset_cv_device status:%x",
			 __func__, status);
		ctx->icvs_state = CV_INIT_STATE;
	}

	if (!status) {
		status = cvs_wait_for_host_wake(WAIT_HOST_WAKE_RESET_MS);
		dev_info(cvs->dev, "%s:wait HOST_WAKE signal status:%x",
			 __func__, status);
	}

	dev_info(cvs->dev, "%s:Exit with status:%x", __func__, status);
	return status;
}

static int cvs_get_fwver_vid_pid(void)
{
	int rc;
	if (!cvs)
		dev_info(cvs->dev, "%s:Invalid cvs context", __func__);

	rc = cvs_read_i2c(GET_FW_VERSION, (char *)&cvs->ver,
			  sizeof(struct cvs_fw));
	if (rc <= 0) {
		dev_err(cvs->dev,
			"%s:CV command:0x%x failed, cvs_i2c_read ret:%d\n",
			__func__, GET_FW_VERSION, rc);
		cvs_release_camera_sensor_internal();
		return -EIO;
	}

	rc = cvs_read_i2c(GET_VID_PID, (char *)&cvs->id, sizeof(struct cvs_id));
	if (rc <= 0) {
		dev_err(cvs->dev,
			"%s:CV command:0x%x failed, cvs_i2c_read ret:%d\n",
			__func__, GET_VID_PID, rc);
		cvs_release_camera_sensor_internal();
		return -EIO;
	}

	return 0;
}

static bool iterate_files(struct dir_context *fdir, const char *name,
			  int namelen, loff_t offset, u64 ino,
			  unsigned int d_type)
{
	if (d_type == DT_REG &&
	    strncmp(name, cvs->file_prefix, strlen(cvs->file_prefix)) == 0) {
		dev_dbg(cvs->dev, "%s:File found: %s ", __func__, name);
		cvs->fw_filename = (char *)devm_kzalloc(
			cvs->dev, strlen(name) + 1, GFP_KERNEL);
		strcpy(cvs->fw_filename, name);
		cvs->fw_bin_file_found = true;

		/* Return 'false' to stop (OR) if there are no more entries */
		return false;
	}

	/* Return 'true' to keep going */
	return true;
}

static bool cvs_search_fw_file(void)
{
	struct file *filp;
	struct dir_context fdir = { 0 };
	int ret;
	const char *path_to_search = "/lib/firmware/";

	sprintf(cvs->file_prefix, "%d-%d", cvs->id.vid, cvs->id.pid);
	dev_dbg(cvs->dev, "%s:VID-PID as string: %s", __func__,
		cvs->file_prefix);

	filp = filp_open(path_to_search, O_RDONLY | O_DIRECTORY, 0);
	if (IS_ERR_OR_NULL(filp)) {
		dev_err(cvs->dev, "%s:Failed to open directory: %ld", __func__,
			PTR_ERR(filp));
		return PTR_ERR(filp);
	}

	fdir.actor = iterate_files;
	fdir.pos = 0;
	ret = iterate_dir(filp, &fdir);
	filp_close(filp, NULL);
	return cvs->fw_bin_file_found;
}

static u32 cvs_calc_checksum(void *data, int size)
{
	int i;
	u32 chksum = 0;

	for (i = 0; i < size / sizeof(u32); i++) {
		chksum += *((u32 *)data + i);
	}

	return chksum;
}

static int cvs_fw_parse(void)
{
	int status = -1;
	u8 magic[MAGICNUMSIZE + 1] = MAGICNUM;
	struct cvs_fw_header *ptr_fw_header;
	int fw_bin_header_size = sizeof(struct cvs_fw_header);
	u32 chksum = -1;

	if (!cvs) {
		dev_err(cvs->dev, "%s:cvs is NULL", __func__);
		return -EINVAL;
	}

	if (cvs->fw_buffer && (cvs->fw_buffer_size > fw_bin_header_size)) {
		ptr_fw_header = (struct cvs_fw_header *)(cvs->fw_buffer);

		if (memcmp(magic, ptr_fw_header, MAGICNUMSIZE) == 0) {
			dev_dbg(cvs->dev,
				"%s:Lib Firmware file has valid magic number",
				__func__);

			dev_info(cvs->dev, "%s:Lib FW version is %d.%d.%d.%d",
				 __func__, ptr_fw_header->fw_ver.major,
				 ptr_fw_header->fw_ver.minor,
				 ptr_fw_header->fw_ver.hotfix,
				 ptr_fw_header->fw_ver.build);
			dev_info(cvs->dev,
				 "%s:Lib VID:%d, PID:%d, fw_offset:%d",
				 __func__, ptr_fw_header->vid_pid.vid,
				 ptr_fw_header->vid_pid.pid,
				 ptr_fw_header->fw_offset);

			if (ptr_fw_header->fw_offset == fw_bin_header_size) {
				chksum = cvs_calc_checksum(ptr_fw_header,
							   fw_bin_header_size);
			} else {
				dev_err(cvs->dev,
					"%s:Wrong FW header offset:%x",
					__func__, ptr_fw_header->fw_offset);
				return -EINVAL;
			}

			if (chksum == 0) {
				dev_dbg(cvs->dev,
					"%s:Firmware header CRC verified",
					__func__);
				status = 0;

				if ((cvs->ver.major !=
				     ptr_fw_header->fw_ver.major) ||
				    (cvs->ver.minor !=
				     ptr_fw_header->fw_ver.minor) ||
				    (cvs->ver.hotfix !=
				     ptr_fw_header->fw_ver.hotfix) ||
				    (cvs->ver.build !=
				     ptr_fw_header->fw_ver.build)) {
					cvs->fw_dl_needed = true;
					dev_info(cvs->dev,
						 "%s:FW update needed",
						 __func__);
				} else
					dev_info(cvs->dev,
						 "%s:FW update not needed",
						 __func__);
			} else {
				dev_err(cvs->dev, "%s:FW header CRC fail",
					__func__);
				status = -EINVAL;
			}
		} else {
			dev_err(cvs->dev, "%s:FW has invalid magic number",
				__func__);
			status = -EINVAL;
		}
	}
	return status;
}

static bool evaluate_fw(void)
{
	int ret, status = 1;
	dev_info(cvs->dev, "%s:file name is %s : %p\n", __func__,
		 cvs->fw_filename, cvs->dev);
	ret = firmware_request_nowarn(&cvs->file, cvs->fw_filename, cvs->dev);
	if (ret)
		dev_err(cvs->dev,
			"%s:firmware_request_nowarn() fail with ret:%d\n",
			__func__, ret);
	else
		dev_dbg(cvs->dev,
			"%s:firmware_request_nowarn() pass with name:%s, data:%p, size:%d\n",
			__func__, cvs->fw_filename, cvs->file->data,
			(int)cvs->file->size);

	/* Alloc memory for FW Image Buffer */
	cvs->fw_buffer_size = cvs->file->size;
	cvs->fw_buffer =
		devm_kzalloc(cvs->dev, cvs->fw_buffer_size, GFP_KERNEL);
	if (IS_ERR_OR_NULL(cvs->fw_buffer)) {
		dev_err(cvs->dev, "%s:NO memory for fw_buffer", __func__);
		status = -ENOMEM;
		goto err_fw_release;
	} else
		dev_dbg(cvs->dev, "%s:fw_buff:%p size:%x, out_buf:%p",
			__func__, cvs->fw_buffer, cvs->fw_buffer_size,
			cvs->out_buf);

	memcpy(cvs->fw_buffer, cvs->file->data, cvs->fw_buffer_size);
	wmb(); /* Flush WC buffers after writing fw_buffer */

	status = cvs_fw_parse();
	if (status) {
		dev_err(cvs->dev, "%s:cvs_fw_parse() fail with status:%d\n",
			__func__, status);
		goto err_fw_release;
	}

	return 0;

err_fw_release:
	dev_err(cvs->dev, "%s:Calling release_firmware()\n", __func__);
	release_firmware(cvs->file);
	return status;
}

void cvs_fw_dl_thread(struct work_struct *arg)
{
	int status = 0;
	u8 fw_state = 0;
	struct intel_cvs *ctx = cvs;
	struct i2c_client *client =
		container_of(cvs->dev, struct i2c_client, dev);

	if (IS_ERR_OR_NULL(ctx) || IS_ERR_OR_NULL(client)) {
		status = -EINVAL;
		dev_err(cvs->dev, "%s:Invalid cvs_context or client",
			__func__);
		goto xit;
	} else
		dev_info(cvs->dev, "%s:Enter with cvs_context:%p", __func__,
			 ctx);

	while (ctx->ref_count) {
		dev_info(cvs->dev, "%s:Camera is used by IPU", __func__);
		if (ctx->close_fw_dl_task == true) {
			dev_info(cvs->dev, "%s:Got close_fw_dl_task as true",
				 __func__);
			goto err_exit;
		}
		cvs_wait_for_host_wake(WAIT_HOST_WAKE_NORMAL_MS);
	}

	ctx->max_flashtime_ms = WAIT_HOST_WAKE_FLASH_LONG_MS;
	ctx->fw_update_retries = CV_FW_DL_MAX_TRY_DEFAULT;

	if (cvs->i2c_shared &&
	    ctx->icvs_sensor_state != CV_SENSOR_IPU_ACQUIRED_STATE) {
		if (cvs_acquire_camera_sensor_internal())
			goto err_exit;
	}
	cvs->icvs_sensor_state = CV_SENSOR_VISION_ACQUIRED_STATE;

	if (!cvs_get_fwver_vid_pid() && cvs_search_fw_file()) {
		dev_info(cvs->dev, "%s:FW file found", __func__);
		if (!evaluate_fw())
			dev_dbg(cvs->dev, "%s:FW file found is valid",
				__func__);
		else {
			dev_err(cvs->dev, "%s:FW file found is invalid",
				__func__);
			goto err_exit;
		}
	} else {
		dev_info(cvs->dev, "%s:FW file not found", __func__);
		goto err_exit;
	}

	ctx->icvs_state = CV_INIT_STATE;
	do {
		int rebootRetry = 0;

		if (ctx->close_fw_dl_task == true) {
			dev_info(cvs->dev, "%s:Received close_fw_dl_task true",
				 __func__);
			goto err_exit;
		}

		while (rebootRetry <= FW_MAX_RETRY) {
			fw_state = 0;

			if (ctx->close_fw_dl_task == true) {
				dev_info(cvs->dev,
					 "%s:Received close_fw_dl_task true",
					 __func__);
				goto err_exit;
			}

			if (ctx->icvs_sensor_state !=
				    CV_SENSOR_IPU_ACQUIRED_STATE &&
			    ctx->i2c_shared) {
				status = cvs_acquire_camera_sensor_internal();
				if (status) {
					dev_err(cvs->dev,
						"%s:Acquire sensor fail",
						__func__);
					goto err_exit;
				} else {
					ctx->icvs_sensor_state =
						CV_SENSOR_VISION_ACQUIRED_STATE;
					dev_dbg(cvs->dev,
						"%s:Acquire sensor pass",
						__func__);
				}
			}

			cvs_wait_for_host_wake(WAIT_HOST_WAKE_NORMAL_MS);
			status = cvs_get_device_state(&fw_state);
			dev_dbg(cvs->dev,
				"%s:I2C check FW state:%x, status:%x",
				__func__, fw_state, status);

			if ((!status) && (fw_state & SENSOR_OWNER_BIT_MASK)) {
				dev_dbg(cvs->dev, "%s:fw_state Success",
					__func__);
				break;
			} else {
				dev_err(cvs->dev,
					"%s:fw error with state:%x. Try reboot",
					__func__, fw_state);

				if (ctx->i2c_shared) {
					if (cvs_release_camera_sensor_internal()) {
						dev_err(cvs->dev,
							"%s:Release sensor fail", __func__);
					} else {
						ctx->icvs_sensor_state = CV_SENSOR_RELEASED_STATE;
						dev_dbg(cvs->dev,
							"%s:Release sensor pass", __func__);
					}
				} else {
					ctx->icvs_sensor_state = CV_SENSOR_RELEASED_STATE;
				}

				/* reboot */
				if (rebootRetry < FW_MAX_RETRY) {
					if (ctx->icvs_sensor_state ==
					    CV_SENSOR_RELEASED_STATE) {
						dev_info(
							cvs->dev,
							"%s:reset only if sensor released",
							__func__);
						if (cvs_reset_cv_device())
							dev_err(cvs->dev,
								"%s:Reset CV Soc Fail", __func__);
						else
							dev_info(
								cvs->dev,
								"%s:Reset CV Soc Pass", __func__);
					}
					cvs_wait_for_host_wake(
						WAIT_HOST_WAKE_RESET_MS);
					rebootRetry++;
				}
			}
		}

		/* max reboot retry achieved and not able to set up I2C communication */
		if ((rebootRetry >= FW_MAX_RETRY) && (ctx->fw_update_retries) &&
		    (ctx->icvs_state != CV_STOPPING) && (!fw_state)) {
			dev_err(cvs->dev,
				"%s:Not able to set up i2c after reboot", __func__);
			goto err_exit;
		}

		if (!((!status) && (fw_state & SENSOR_OWNER_BIT_MASK))) {
			dev_err(cvs->dev,
				"%s:firmware error with FW state:%x, status:%x",
				__func__, fw_state, status);
			goto err_exit;
		}

		if ((!status) && ctx->fw_dl_needed && ctx->fw_update_retries) {
			dev_info(
				cvs->dev,
				"%s:dev & lib versions differ. Start FW update",
				__func__);

			status = cvs_dev_fw_dl();
			if (status) {
				dev_err(cvs->dev, "%s:cvs_dev_fw_dl fail", __func__);
			} else {
				dev_err(cvs->dev, "%s:cvs_dev_fw_dl pass", __func__);
				if (ctx->fw_update_retries)
					ctx->fw_update_retries--;
				goto err_exit;
			}
		}

		if (ctx->fw_update_retries)
			ctx->fw_update_retries--;

	} while (ctx->fw_update_retries);

err_exit:
	if (ctx->icvs_sensor_state == CV_SENSOR_VISION_ACQUIRED_STATE) {
		if (ctx->i2c_shared) {
			if (cvs_release_camera_sensor_internal()) {
				dev_err(cvs->dev, "%s:Release sensor fail",__func__);
			} else {
				ctx->icvs_sensor_state =
					CV_SENSOR_RELEASED_STATE;
				dev_dbg(cvs->dev, "%s:Release sensor pass",__func__);
			}
		} else {
			ctx->icvs_sensor_state = CV_SENSOR_RELEASED_STATE;
		}
	}

	ctx->update_complete_event_arg = 1;
	wake_up_interruptible(&ctx->update_complete_event);
	ctx->fw_dl_task_finished = true;

	dev_info(cvs->dev, "%s:Starting watchdog timer", __func__);
	hrtimer_start(&ctx->wdt, ms_to_ktime(WDT_TIMEOUT), HRTIMER_MODE_REL);

xit:
	dev_info(cvs->dev, "%s:Exit with status:%x", __func__, status);
	return;
}