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
#define FW_BIN_HDR_SIZE sizeof(struct cvs_fw_header)

int cvs_write_i2c(u16 cmd, u8 *data, u32 len)
{
	struct i2c_client *client =
		container_of(cvs->dev, struct i2c_client, dev);
	int count;
	u16 cv_cmd = (((cmd) >> 8) & 0x00ff) | (((cmd) << 8) & 0xff00);

	switch (cmd) {
	case FW_LOADER_START:
		count = i2c_master_send(client, (const char *)&cv_cmd,
					sizeof(u16));
		if (count != sizeof(u16))
			return -EIO;
		break;
	case FW_LOADER_DATA:
		count = i2c_master_send(client, data, len);
		if (count != len)
			return -EIO;
		break;
	case FW_LOADER_END:
		count = i2c_master_send(client, (const char *)&cv_cmd,
					sizeof(u16));
		mdelay(GPIO_WRITE_DELAY_MS);
		if (count != sizeof(u16))
			return -EIO;
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

	dev_dbg(cvs->dev, "%s: fw_state:0x%x", __func__, *cv_fw_state);
	return 0;
}

int cvs_wait_for_host_wake(u64 time_ms)
{
	int ret = -1;
	int64_t timeout = 0;


	/* wait for HOST_WAKE signal, timeout in time_ms */
	timeout = msecs_to_jiffies(time_ms);
	ret = wait_event_interruptible_timeout(
		cvs->hostwake_event, cvs->hostwake_event_arg == 1, timeout);

	if (ret <= 0) {
		dev_err(cvs->dev, "%s:hostwake wait timeout", __func__);
		return -ETIMEDOUT;
	}

	cvs->hostwake_event_arg = 0;
	return 0;
}

int cvs_reset_cv_device(void)
{
	if (IS_ERR_OR_NULL(cvs)) {
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
		return -EINVAL;
	}

	status = cvs_get_device_state(&fw_state);

	if ((!status) && (fw_state & SENSOR_OWNER_BIT_MASK)) {
		ctx->cv_fw_state = fw_state;
	} else {
		dev_err(cvs->dev,
			"%s:firmware state error, try reboot. FW state:0x%x. - "
			"status:0x%x", __func__, fw_state, status);
	}

	return status;
}

int cvs_dev_fw_dl_start(void)
{
	int status = 0;
	struct intel_cvs *ctx = cvs;
	u8 fw_state = 0;


	/* check CV FW state */
	status = cvs_get_device_state(&fw_state);
	if ((!status) && (fw_state & SENSOR_OWNER_BIT_MASK)) {
		if (cvs_write_i2c(FW_LOADER_START, NULL, 0)) {
			dev_err(cvs->dev, "%s:cvs_write_i2c() failed", __func__);
			return -EIO;
		}
	}

	ctx->icvs_state = CV_FW_DOWNLOADING_STATE;
	/* check CV FW state */
	status = cvs_get_device_state(&fw_state);
	if ((status) || ((fw_state & DEVICE_DWNLD_STATE_MASK) !=
				DEVICE_DWNLD_STATE_MASK)) {
		status = -EIO;
		dev_err(cvs->dev,
			"%s:fail to enter download state. fwstate:0x%x, status:0x%x",
			__func__, fw_state, status);
	}

	ctx->cv_fw_state = fw_state;
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

	fw_buff_ptr = (u8 *)ctx->fw_buffer + FW_BIN_HDR_SIZE;
	fw_size = ctx->fw_buffer_size - FW_BIN_HDR_SIZE;

	while ((fw_size > 0) && (ctx->icvs_state != CV_STOPPING)) {
		int retry = FW_MAX_RETRY;

		if (ctx->close_fw_dl_task == true) {
			dev_err(cvs->dev, "%s:Received close_fw_dl_task true",
				__func__);
			status = -EPERM;
			goto err_exit;
		}

		do {
			if (ctx->close_fw_dl_task == true) {
				dev_info(cvs->dev, "%s:Received close_fw_dl_task",
					 __func__);
				status = -EPERM;
				goto err_exit;
			}

			/* copy data to outbuf */
			memcpy(ctx->out_buf, fw_buff_ptr,
			       I2C_PKT_SIZE < fw_size ?
				       I2C_PKT_SIZE : fw_size);
			wmb(); /* Flush WC buffers after writing out_buf */

			if ((!status) && (fw_state & DEVICE_DWNLD_STATE_MASK)) {
				if (cvs_write_i2c(FW_LOADER_DATA, ctx->out_buf,I2C_PKT_SIZE)) {
					dev_err(cvs->dev, "%s:I2C fw_loader_data failed",
						__func__);
					fw_state = DEVICE_DWNLD_ERROR_MASK;
					goto i2c_packet_loop_end;
				}
			}

			/* Wait for Host Wake */
			cvs_wait_for_host_wake(WAIT_HOST_WAKE_NORMAL_MS);
			/* Check device state */
			status = cvs_get_device_state(&fw_state);

			if ((!status) &&
				(fw_state & DEVICE_DWNLD_STATE_MASK) &&
				(fw_state & DEVICE_DWNLD_BUSY_MASK)) {
				/* Check device state again */
				status = cvs_get_device_state(&fw_state);
				dev_err(cvs->dev,
					"%s:I2C re-check fw_state:0x%x, status:0x%x",
					__func__, fw_state, status);

				if (fw_state & DEVICE_DWNLD_BUSY_MASK) {
					dev_err(cvs->dev,
						"%s:I2C is busy for too long! fw_state:0x%x, status:0x%x",
						__func__, fw_state, status);
					break;
				}
			} else {
				dev_dbg(cvs->dev,
					"%s:got WRONG_1 fw_state:0x%x, cv_state:0x%x, status:0x%x ",
					__func__, fw_state, ctx->icvs_state, status);
			}

			if (ctx->icvs_state == CV_STOPPING) {
				dev_err(cvs->dev, "%s:cv_state is CV_STOPPING",
					__func__);
				ctx->fw_update_retries = 0;
				break;
			}
i2c_packet_loop_end:
		} while (--retry && (fw_state & DEVICE_DWNLD_ERROR_MASK));

		if ((fw_state & DEVICE_DWNLD_BUSY_MASK) ||
		    (fw_state & DEVICE_DWNLD_ERROR_MASK) ||
		    (ctx->icvs_state == CV_STOPPING) || (!fw_state)) {
			dev_err(cvs->dev,
				"%s:got WRONG_2 fw_state:0x%x, cv_state:0x%x, status:0x%x ",
				__func__, fw_state, ctx->icvs_state, status);
			status = -EIO;
			break;
		}

		fw_size -= I2C_PKT_SIZE;
		fw_buff_ptr += I2C_PKT_SIZE;
		ctx->cv_fw_state = fw_state;
	}

err_exit:
	dev_info(cvs->dev, "%s:Exit with status:0x%x, fw_state:0x%x, cv_state:0x%x",
		 __func__, status, fw_state, ctx->icvs_state);

	return status;
}

int cvs_dev_fw_dl_end(void)
{
	int status = 0;
	struct intel_cvs *ctx = cvs;
	u8 fw_state = 0;

	if (cvs_write_i2c(FW_LOADER_END, NULL, 0)) {
		dev_err(cvs->dev, "%s:I2C fw_loader_end failed", __func__);
		return -EIO;
	}

	ctx->icvs_state = CV_FW_FLASHING_STATE;
	status = cvs_get_device_state(&fw_state);
	ctx->cv_fw_state = fw_state;
	return status;
}

int cvs_dev_fw_dl(void)
{
	int status = 0;
	struct intel_cvs *ctx = cvs;
	u8 retries = 0;
	const u8 max_retry = 5;

	dev_info(cvs->dev, "%s:Enter", __func__);

	if (cvs_dev_fw_dl_start()) {
		dev_err(cvs->dev, "%s:cvs_dev_fw_dl_start() fail", __func__);
	}
	else {
		status = cvs_dev_fw_dl_data();
		if(status)
			dev_err(cvs->dev, "%s:cvs_dev_fw_dl_data() fail",
				__func__);
	}

	/* End FW download, no matter if it's pass or fail */
	if (cvs_dev_fw_dl_end()){
		dev_err(cvs->dev, "%s:cvs_dev_fw_dl_end() fail", __func__);
		return -EIO;
	}

	if (cvs_wait_for_host_wake(WAIT_HOST_WAKE_NORMAL_MS)) {
		dev_err(cvs->dev, "%s:Loader End flash hostwake error", __func__);
		return -ETIMEDOUT;
	}
	
	if (cvs_wait_for_host_wake(ctx->max_flashtime_ms)) {
		dev_err(cvs->dev, "%s: FW flash hostwake error",
		__func__);
		return -ETIMEDOUT;
	}
	ctx->icvs_state = CV_INIT_STATE;
	cvs_get_dev_state();
	if((!ctx->cv_fw_state) ||
	(ctx->cv_fw_state & DEVICE_DWNLD_BUSY_MASK)) {
		dev_err(cvs->dev, "%s: Post Flash device still in DWNLD_BUSY",
			__func__);
			return -EBUSY;
	}

	if (!status && cvs->close_fw_dl_task) {
		status = -EINTR;
		dev_info(cvs->dev, "%s:Exit with status:0x%x", __func__, status);
		return status;
	}

	if (ctx->i2c_shared) {
		do {
			status = cvs_release_camera_sensor_internal();
			if (status) {
				dev_err(cvs->dev, "%s:Release sensor fail",
					__func__);
			} else {
				ctx->icvs_sensor_state =
					CV_SENSOR_RELEASED_STATE;
				break;
			}
			mdelay(WAIT_NORMAL_MS);
		} while (ctx->icvs_sensor_state ==
				 CV_SENSOR_VISION_ACQUIRED_STATE &&
			 retries++ < max_retry);
	}
	
	if (ctx->icvs_sensor_state == CV_SENSOR_RELEASED_STATE) {
		/* reset Vision chip */
		if (cvs_reset_cv_device()) {
			dev_err(cvs->dev, "%s:CV reset post flash fail",
				__func__);
			return -EIO;
		}

		ctx->icvs_state = CV_INIT_STATE;
		if (cvs_wait_for_host_wake(WAIT_HOST_WAKE_RESET_MS)) {
			dev_err(cvs->dev, "%s:CV reset FW boot hostwake error",
				__func__);
			return -ETIMEDOUT;
		}

	}

	dev_info(cvs->dev, "%s:Exit with status:0x%x", __func__, status);
	return status;
}

static int cvs_get_fwver_vid_pid(void)
{
	int rc;
	if (!cvs) {
		return -EINVAL;
	}

	rc = cvs_read_i2c(GET_FW_VERSION, (char *)&cvs->ver,
			  sizeof(struct cvs_fw));
	if (rc <= 0) {
		cvs_release_camera_sensor_internal();
		return -EIO;
	}

	rc = cvs_read_i2c(GET_VID_PID, (char *)&cvs->id, sizeof(struct cvs_id));
	if (rc <= 0) {
		cvs_release_camera_sensor_internal();
		return -EIO;
	}

	return 0;
}

static u32 cvs_calc_checksum(void *data)
{
	int i;
	u32 chksum = 0;

	if (IS_ERR_OR_NULL(data))
		return PTR_ERR(data);

	for (i = 0; i < FW_BIN_HDR_SIZE / sizeof(u32); i++) {
		chksum += *((u32 *)data + i);
	}

	return chksum;
}

static int cvs_fw_parse(void)
{
	u8 magic[MAGICNUMSIZE + 1] = MAGICNUM;
	struct cvs_fw_header *ptr_fw_header;
	int fw_bin_header_size = FW_BIN_HDR_SIZE;

	if (!cvs)
		return -EINVAL;

	if (!(cvs->fw_buffer && (cvs->fw_buffer_size > fw_bin_header_size))) {
		dev_err(cvs->dev, "%s:Invalid fw_buff params", __func__);
		return -EINVAL;
	}

	ptr_fw_header = (struct cvs_fw_header *)(cvs->fw_buffer);
	if (memcmp(magic, ptr_fw_header, MAGICNUMSIZE) != 0) {
		dev_err(cvs->dev, "%s:FW has invalid magic number",
			__func__);
		return -EINVAL;
	}

	if ((ptr_fw_header->vid_pid.vid != cvs->id.vid) ||
		(ptr_fw_header->vid_pid.pid != cvs->id.pid)) {
		dev_err(cvs->dev, "%s:dev & lib vid, pid mismatch",
			__func__);
		return -EINVAL;
	}

	dev_info(cvs->dev, "%s:Lib FW version is %d.%d.%d.%d",
			__func__, ptr_fw_header->fw_ver.major,
			ptr_fw_header->fw_ver.minor,
			ptr_fw_header->fw_ver.hotfix,
			ptr_fw_header->fw_ver.build);

	dev_info(cvs->dev, "%s:Lib VID:0X%x, PID:0x%x, fw_offset:0x%x",
			__func__, ptr_fw_header->vid_pid.vid,
			ptr_fw_header->vid_pid.pid,
			ptr_fw_header->fw_offset);

	if (ptr_fw_header->fw_offset == fw_bin_header_size) {
		if (cvs_calc_checksum(ptr_fw_header)) {
			dev_err(cvs->dev, "%s:FW header CRC fail",	__func__);
			return -EINVAL;
		}
	} else {
		dev_err(cvs->dev, "%s:Wrong FW header offset:0x%x",
			__func__, ptr_fw_header->fw_offset);
		return -EINVAL;
	}

	if ((cvs->ver.major  != ptr_fw_header->fw_ver.major) ||
		(cvs->ver.minor  != ptr_fw_header->fw_ver.minor) ||
		(cvs->ver.hotfix != ptr_fw_header->fw_ver.hotfix) ||
		(cvs->ver.build  != ptr_fw_header->fw_ver.build)) {
		cvs->fw_dl_needed = true;
		dev_dbg(cvs->dev, "%s:FW update needed", __func__);
	}
	else {
		cvs->fw_dl_needed = false;
		dev_info(cvs->dev, "%s:FW update not needed", __func__);
	}

	return 0;
}

static bool evaluate_fw(void)
{
	int ret, status = 1;

	if (cvs->oem_prod_id) {
		sprintf(cvs->fw_filename, "cvs/%04X%04X-%04llX.bin",
			cvs->id.vid, cvs->id.pid, cvs->oem_prod_id);
	}
	else
		sprintf(cvs->fw_filename, "cvs/%04X%04X.bin",
			cvs->id.vid, cvs->id.pid);

	ret = request_firmware(&cvs->file, cvs->fw_filename, cvs->dev);
	if (ret) {
		dev_err(cvs->dev,
			"%s:request_firmware() fail with ret:%d",
			__func__, ret);
		return ret;
	}

	dev_dbg(cvs->dev,
		"%s: FW bin file found with file_ptr:%p, size:0x%x",
		__func__, cvs->file->data, (int)cvs->file->size);

	/* Alloc memory for FW Image Buffer */
	cvs->fw_buffer_size = cvs->file->size;
	cvs->fw_buffer =
		devm_kzalloc(cvs->dev, cvs->fw_buffer_size, GFP_KERNEL);
	if (IS_ERR_OR_NULL(cvs->fw_buffer)) {
		dev_err(cvs->dev, "%s:No memory for fw_buffer", __func__);
		release_firmware(cvs->file);
		return -ENOMEM;
	}

	dev_dbg(cvs->dev, "%s:fw_buff:%p size:0x%x, out_buf:%p",
		__func__, cvs->fw_buffer, cvs->fw_buffer_size,
		cvs->out_buf);

	memcpy(cvs->fw_buffer, cvs->file->data, cvs->fw_buffer_size);
	wmb(); /* Flush WC buffers after writing fw_buffer */

	status = cvs_fw_parse();
	if (status)
		dev_err(cvs->dev, "%s: FW bin file is invalid", __func__);

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
		return;
	}

	ctx->max_flashtime_ms = WAIT_HOST_WAKE_FLASH_LONG_MS;
	ctx->fw_update_retries = CV_FW_DL_MAX_TRY_DEFAULT;

	if (cvs->i2c_shared &&
		ctx->icvs_sensor_state == CV_SENSOR_RELEASED_STATE) {
		if (cvs_acquire_camera_sensor_internal()) {
			dev_err(cvs->dev,
				"%s:Acquire sensor fail", __func__);
			goto xit;
		}
	}
	cvs->icvs_sensor_state = CV_SENSOR_VISION_ACQUIRED_STATE;

	if (!cvs_get_fwver_vid_pid()) {
		dev_info(cvs->dev, "%s:Device FW version is %d.%d.%d.%d",
			__func__, cvs->ver.major, cvs->ver.minor,
			cvs->ver.hotfix, cvs->ver.build);

		if (evaluate_fw()) {
			dev_err(cvs->dev, "%s:FW file not found",
				__func__);
			goto xit;
		}
	}
	else {
		dev_err(cvs->dev, "%s:I2C error. Not able to read vid/pid",
			__func__);
		goto xit;
	}

	ctx->icvs_state = CV_INIT_STATE;
	do {
		int rebootRetry = 0;

		if (ctx->close_fw_dl_task == true) {
			dev_info(cvs->dev, "%s:Received close_fw_dl_task true",
				 __func__);
			goto xit;
		}

		while (rebootRetry <= FW_MAX_RETRY) {
			fw_state = 0;

			if (ctx->close_fw_dl_task == true) {
				dev_info(cvs->dev,
					 "%s:Received close_fw_dl_task true",
					 __func__);
				goto xit;
			}

			if (ctx->icvs_sensor_state == CV_SENSOR_RELEASED_STATE &&
					ctx->i2c_shared) {
				status = cvs_acquire_camera_sensor_internal();
				if (status) {
					dev_err(cvs->dev,
						"%s:Acquire sensor fail", __func__);
					goto xit;
				} else {
					ctx->icvs_sensor_state =
						CV_SENSOR_VISION_ACQUIRED_STATE;
				}
			}

			status = cvs_get_device_state(&fw_state);
			if ((!status) && (fw_state & SENSOR_OWNER_BIT_MASK)) {
				break;
			} else {
				dev_err(cvs->dev,
					"%s:fw error with state:0x%x. Try reboot",
					__func__, fw_state);

				if (ctx->i2c_shared) {
					if (cvs_release_camera_sensor_internal()) {
						dev_err(cvs->dev,
							"%s:Release sensor fail", __func__);
					} else {
						ctx->icvs_sensor_state = CV_SENSOR_RELEASED_STATE;
					}
				} else {
					ctx->icvs_sensor_state = CV_SENSOR_RELEASED_STATE;
				}

				/* reboot */
				if (rebootRetry < FW_MAX_RETRY) {
					if (ctx->icvs_sensor_state ==
					    CV_SENSOR_RELEASED_STATE) {
						if (cvs_reset_cv_device())
							dev_err(cvs->dev,
								"%s:Reset CV Soc Fail", __func__);
					}
					cvs_wait_for_host_wake(WAIT_HOST_WAKE_RESET_MS);
					rebootRetry++;
				}
			}
		}

		/* max reboot retry achieved and not able to set up I2C communication */
		if ((rebootRetry >= FW_MAX_RETRY) && (ctx->fw_update_retries) &&
		    (ctx->icvs_state != CV_STOPPING) && (!fw_state)) {
			dev_err(cvs->dev,
				"%s:Not able to set up i2c after reboot", __func__);
			goto xit;
		}

		if (!((!status) && (fw_state & SENSOR_OWNER_BIT_MASK))) {
			dev_err(cvs->dev,
				"%s:firmware error with FW state:0x%x, status:0x%x",
				__func__, fw_state, status);
			goto xit;
		}

		if ((!status) && ctx->fw_dl_needed && ctx->fw_update_retries) {
			dev_info(cvs->dev,
				"%s:dev & lib versions differ. Start FW update",
				__func__);

			status = cvs_dev_fw_dl();
			if (ctx->close_fw_dl_task && status == -EINTR)
				dev_info(cvs->dev,
				"%s:flash interrupted,FW reset to factory ver",
				__func__);
			else if (ctx->close_fw_dl_task)
				dev_info(cvs->dev, "%s:cvs_dev_fw_dl cancelled", __func__);
			else if (status)
				dev_err(cvs->dev, "%s:cvs_dev_fw_dl fail", __func__);
			else {
				dev_info(cvs->dev, "%s:cvs_dev_fw_dl pass", __func__);
				if (ctx->fw_update_retries)
					ctx->fw_update_retries--;
				goto xit;
			}
		}

		if (ctx->fw_update_retries)
			ctx->fw_update_retries--;

	} while (ctx->fw_update_retries);

xit:
	/* After FW download acquire sensor to keep sensor ownserhip
	  with host(IPU) always.This makes IPU-Vision driver interface
	  simple w/o need of IPU calling vision driver interface API's */
	if (ctx->icvs_sensor_state != CV_SENSOR_VISION_ACQUIRED_STATE) {
		if (cvs_acquire_camera_sensor_internal()) {
			dev_err(cvs->dev, "%s:Acquire sensor fail", __func__);
		}
		else {
			ctx->icvs_sensor_state =
				CV_SENSOR_VISION_ACQUIRED_STATE;
		}
	}
	else {
		ctx->icvs_sensor_state = CV_SENSOR_VISION_ACQUIRED_STATE;
	}

	ctx->update_complete_event_arg = 1;
	wake_up_interruptible(&ctx->update_complete_event);
	ctx->fw_dl_task_finished = true;

	dev_info(cvs->dev, "%s:Exiting fw_dl thread", __func__);
	return;
}