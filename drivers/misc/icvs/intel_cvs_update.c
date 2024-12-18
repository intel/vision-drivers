// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024 Intel Corporation.
 *
 */

#include <linux/intel_cvs.h>
#include "intel_cvs_update.h"

extern struct intel_cvs *cvs;


int cvs_write_i2c(u16 cmd, u8 *data, u32 len)
{
	struct intel_cvs *ctx = cvs;
	struct i2c_client *client =
		container_of(cvs->dev, struct i2c_client, dev);
	int count;
	u16 cv_cmd = (((cmd) >> 8) & 0x00ff) | (((cmd) << 8) & 0xff00);
	u32 cv_host_identifier_size = sizeof(union cv_host_identifiers);

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
	case SET_HOST_IDENTIFIER:
		u8 *out_buff;

		out_buff = devm_kzalloc(ctx->dev,
								(cv_host_identifier_size + sizeof(cmd)),
								GFP_KERNEL);
		if (!out_buff) {
			dev_err(cvs->dev, "%s:Buffer alloc failed", __func__);
			return -ENOMEM;
		}
		out_buff[0] = (cmd >> 8) & 0x00ff;
		out_buff[1] = cmd & 0x00ff;
		ctx->host_identifiers.field.vision_sensing = 0;
		ctx->host_identifiers.field.device_power_setting = 0;
		ctx->host_identifiers.field.privacy_led_host = 1;
		ctx->host_identifiers.field.rgbcamera_pwrup_host = 1;

		memcpy(&out_buff[2], &ctx->host_identifiers.value,
			   cv_host_identifier_size);

		count = i2c_master_send(client, (const char *)out_buff,
								sizeof(cmd) + cv_host_identifier_size);

		if (count != cv_host_identifier_size + sizeof(cmd))
			return -EIO;
		dev_dbg(cvs->dev, "%s:set_host_identifier cmd pass", __func__);
		break;
	default:
		dev_err(cvs->dev, "%s:Invalid command type", __func__);
		return -EINVAL;
	}

	return 0;
}

int cvs_get_device_state(u8 *cv_fw_state)
{
	if (!cvs)
		return -EINVAL;

	if (cvs_read_i2c(GET_DEVICE_STATE, cv_fw_state, sizeof(char)) <= 0) {
		dev_err(cvs->dev, "%s:cvs_read_i2c() failed", __func__);
		return -EIO;
	}

	if (!((*cv_fw_state) & DEVICE_ON_BIT_MASK)) {
		dev_err(cvs->dev, "%s:device_on bit not set", __func__);
		return -EINVAL;
	}

	if (cvs->i2c_shared && !((*cv_fw_state) & SENSOR_OWNER_BIT_MASK)) {
		dev_err(cvs->dev, "%s:sensor owner bit not set", __func__);
		return -EINVAL;
	}

	dev_dbg(cvs->dev, "%s: fw_state:0x%x", __func__, *cv_fw_state);
	return 0;
}

int cvs_get_device_cap(struct cv_ver_capability *cv_fw_cap)
{
	if (!cvs)
		return -EINVAL;
	cvs->magic_num_support = true;

	if (cvs_read_i2c(GET_DEV_CAPABILITY, (char *)cv_fw_cap,
					 sizeof(struct cv_ver_capability)) <= 0) {
		dev_info(cvs->dev, "%s:Device protocol is 1.0", __func__);
		cvs->magic_num_support = false;
	} else {
		dev_info(cvs->dev, "%s:Device protocol is %d.%d", __func__,
				 cvs->cv_fw_capability.protocol_ver_major,
				 cvs->cv_fw_capability.protocol_ver_minor);
		dev_info(cvs->dev, "%s:Device capability is 0x%x", __func__,
				 cvs->cv_fw_capability.dev_capability);
	}

	return 0;
}

int cvs_wait_for_host_wake(u64 time_ms)
{
	int ret = -1;
	s64 timeout = 0;

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
	if (IS_ERR_OR_NULL(cvs))
		return -EINVAL;

	gpiod_set_value_cansleep(cvs->rst, 0);
	mdelay(GPIO_RESET_MS);
	gpiod_set_value_cansleep(cvs->rst, 1);
	mdelay(FW_PREPARE_MS);
	return 0;
}

int cvs_dev_fw_dl_start(void)
{
	struct intel_cvs *ctx = cvs;
	u8 fw_state = 0;

	/* check CV FW state */
	if (cvs_get_device_state(&fw_state)) {
		dev_err(cvs->dev, "%s:cvs_get_device_state() failed", __func__);
		return -EIO;
	}
	if (cvs_write_i2c(FW_LOADER_START, NULL, 0)) {
		dev_err(cvs->dev, "%s:cvs_write_i2c() failed", __func__);
		return -EIO;
	}

	/* Wait for Host Wake */
	if (cvs_wait_for_host_wake(WAIT_HOST_WAKE_NORMAL_MS)) {
		dev_err(cvs->dev, "%s:Host wake timeout", __func__);
		return -ETIMEDOUT;
	}
	ctx->icvs_state = CV_FW_DOWNLOADING_STATE;
	/* check CV FW state */
	if (cvs_get_device_state(&fw_state)) {
		dev_err(cvs->dev, "%s:cvs_get_device_state() failed", __func__);
		return -EIO;
	}
	ctx->cv_fw_state = fw_state;
	if ((fw_state & DEVICE_DWNLD_STATE_MASK) != DEVICE_DWNLD_STATE_MASK) {
		dev_err(cvs->dev, "%s:fail to enter download state. fwstate:0x%x",
				__func__, fw_state);
		return -EIO;
	}
	return 0;
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
			dev_err(cvs->dev, "%s:Received close_fw_dl_task true", __func__);
			status = -EPERM;
			goto err_exit;
		}

		do {
			u8 out_buf[I2C_PKT_SIZE];

			if (ctx->close_fw_dl_task == true) {
				dev_info(cvs->dev, "%s:Received close_fw_dl_task", __func__);
				status = -EPERM;
				goto err_exit;
			}

			/* copy data to outbuf */
			memcpy(out_buf, fw_buff_ptr,
				   fw_size > I2C_PKT_SIZE ? I2C_PKT_SIZE : fw_size);
			wmb(); /* Flush WC buffers after writing out_buf */

			if (fw_state & DEVICE_DWNLD_STATE_MASK) {
				if (cvs_write_i2c(FW_LOADER_DATA, out_buf, I2C_PKT_SIZE)) {
					dev_err(cvs->dev, "%s:fw_loader_data failed", __func__);
					fw_state = DEVICE_DWNLD_ERROR_MASK;
					goto i2c_packet_loop_end;
				}
			}

			/* Wait for Host Wake */
			if (cvs_wait_for_host_wake(WAIT_HOST_WAKE_NORMAL_MS)) {
				dev_err(cvs->dev, "%s:Host wake timeout", __func__);
				return -EIO;
			}

			/* Check device state */
			if (cvs_get_device_state(&fw_state))
				return -EIO;

			if (!(fw_state & DEVICE_DWNLD_STATE_MASK)) {
				dev_err(cvs->dev, "%s:Device not in download_state",
						__func__);
				return -EIO;
			}

			if (fw_state & DEVICE_DWNLD_BUSY_MASK) {
				dev_err(cvs->dev, "%s:I2C is busy for too long! fw_state:0x%x",
						__func__, fw_state);
				return -EIO;
			}

			if (ctx->icvs_state == CV_STOPPING) {
				dev_err(cvs->dev, "%s:cv_state is CV_STOPPING", __func__);
				ctx->fw_update_retries = 0;
				break;
			}
i2c_packet_loop_end:
		} while (--retry && (fw_state & DEVICE_DWNLD_ERROR_MASK));

		if ((fw_state & DEVICE_DWNLD_BUSY_MASK) ||
			(fw_state & DEVICE_DWNLD_ERROR_MASK) ||
			ctx->icvs_state == CV_STOPPING || !fw_state) {
			dev_err(cvs->dev, "%s:Wrong fw_state:0x%x, cv_state:0x%x",
					__func__, fw_state, ctx->icvs_state);
			status = -EIO;
			break;
		}
		ctx->info_fwupd.num_packets_sent++;
		fw_size -= I2C_PKT_SIZE;
		fw_buff_ptr += I2C_PKT_SIZE;
		ctx->cv_fw_state = fw_state;
	}

err_exit:
	dev_info(cvs->dev, "%s:Exit with status:0x%x, fw_st:0x%x, cv_st:0x%x",
			 __func__, status, fw_state, ctx->icvs_state);

	return status;
}

int cvs_dev_fw_dl_end(void)
{
	struct intel_cvs *ctx = cvs;
	u8 fw_state = 0;

	if (cvs_write_i2c(FW_LOADER_END, NULL, 0)) {
		dev_err(cvs->dev, "%s:fw_loader_end failed", __func__);
		return -EIO;
	}

	if (cvs_wait_for_host_wake(WAIT_HOST_WAKE_NORMAL_MS)) {
		dev_err(cvs->dev, "%s:Loader_end hostwake error", __func__);
		return -ETIMEDOUT;
	}
	ctx->icvs_state = CV_FW_FLASHING_STATE;

	/* check CV FW state */
	if (cvs_get_device_state(&fw_state)) {
		dev_err(cvs->dev, "%s:cvs_get_device_state() failed", __func__);
		return -EIO;
	}
	ctx->cv_fw_state = fw_state;
	return 0;
}

int cvs_dev_fw_dl(void)
{
	int status = 0;
	struct intel_cvs *ctx = cvs;
	u8 retries = 0, fw_state = 0;
	const u8 max_retry = 5;

	dev_info(cvs->dev, "%s:Enter", __func__);
	status = cvs_dev_fw_dl_start();
	if (status) {
		dev_err(cvs->dev, "%s:cvs_dev_fw_dl_start() fail", __func__);
	} else {
		status = cvs_dev_fw_dl_data();
		if (status)
			dev_err(cvs->dev, "%s:cvs_dev_fw_dl_data() fail", __func__);
	}

	/* End FW download, no matter if it's pass or fail */
	if (cvs_dev_fw_dl_end()) {
		dev_err(cvs->dev, "%s:cvs_dev_fw_dl_end() fail", __func__);
		return -EIO;
	}

	if (status)
		return status;

	if (cvs_wait_for_host_wake(ctx->max_flashtime_ms)) {
		dev_err(cvs->dev, "%s:Firmware flash hostwake error", __func__);
		return -ETIMEDOUT;
	}

	ctx->icvs_state = CV_INIT_STATE;
	if (cvs_get_device_state(&fw_state)) {
		dev_err(cvs->dev, "%s:cvs_get_device_state() failed", __func__);
		return -EIO;
	}
	ctx->cv_fw_state = fw_state;
	if (ctx->cv_fw_state & DEVICE_DWNLD_BUSY_MASK) {
		dev_err(cvs->dev, "%s: Device is still busy after flash", __func__);
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
				dev_err(cvs->dev, "%s:Release sensor fail", __func__);
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

	wait_event_interruptible(cvs->lvfs_fwdl_complete_event,
							 cvs->lvfs_fwdl_complete_event_arg == 1);
	cvs->lvfs_fwdl_complete_event_arg = 0;

	/* reset Vision chip */
	if (cvs_reset_cv_device()) {
		dev_err(cvs->dev, "%s:CV reset fail after flash", __func__);
		return -EIO;
	}

	ctx->icvs_state = CV_INIT_STATE;
	if (cvs_wait_for_host_wake(WAIT_HOST_WAKE_RESET_MS)) {
		dev_err(cvs->dev, "%s:hostwake error after CV reset FW boot",
				__func__);
		return -ETIMEDOUT;
	}

	dev_info(cvs->dev, "%s:Exit with status:0x%x", __func__, status);
	return status;
}

int cvs_get_fwver_vid_pid(void)
{
	if (!cvs)
		return -EINVAL;

	if (cvs_read_i2c(GET_FW_VERSION, (char *)&cvs->ver,
					 sizeof(struct cvs_fw)) <= 0)
		goto err_xit;

	if (cvs_read_i2c(GET_VID_PID, (char *)&cvs->id,
					 sizeof(struct cvs_id)) <= 0)
		goto err_xit;

	return 0;

err_xit:
	cvs_release_camera_sensor_internal();
	return -EIO;
}

void cvs_fw_dl_thread(struct work_struct *arg)
{
	int status = 0;
	u8 fw_state = 0;
	u32 fw_size = 0;
	struct intel_cvs *ctx = cvs;

	if (IS_ERR_OR_NULL(ctx)) {
		dev_err(cvs->dev, "%s:Invalid ctx. Exit firmware download", __func__);
		return;
	}

	fw_size = ctx->fw_buffer_size - FW_BIN_HDR_SIZE;
	ctx->info_fwupd.total_packets = fw_size / I2C_PKT_SIZE;
	ctx->info_fwupd.total_packets += (fw_size % I2C_PKT_SIZE) ? 1 : 0;
	ctx->icvs_state = CV_INIT_STATE;

	do {
		if (ctx->close_fw_dl_task == true) {
			dev_info(cvs->dev, "%s:Received close_fw_dl_task true", __func__);
			goto xit;
		}
		if (cvs_get_device_state(&fw_state)) {
			dev_err(cvs->dev, "%s:cvs_get_device_state() failed", __func__);
			goto xit;
		}
		status = cvs_dev_fw_dl();
		cvs->info_fwupd.fw_dl_status_code = status;
		if (ctx->close_fw_dl_task && status == -EINTR) {
			dev_info(cvs->dev, "%s:flash interrupted,fw reset to factory ver",
					 __func__);
		} else if (ctx->close_fw_dl_task) {
			dev_info(cvs->dev, "%s:cvs_dev_fw_dl cancelled", __func__);
		} else if (status) {
			dev_err(cvs->dev, "%s:cvs_dev_fw_dl fail", __func__);
		} else {
			dev_info(cvs->dev, "%s:cvs_dev_fw_dl pass", __func__);
			ctx->fw_update_retries--;
			break;
		}
	} while (--ctx->fw_update_retries);

xit:
	/* After FW download acquire sensor to keep sensor ownserhip
	 * with host(IPU) always.This makes IPU-Vision driver interface
	 * simple w/o need of IPU calling vision driver interface API's
	 */
	if (ctx->icvs_sensor_state != CV_SENSOR_VISION_ACQUIRED_STATE) {
		if (cvs_acquire_camera_sensor_internal()) {
			dev_err(cvs->dev, "%s:Acquire sensor fail", __func__);
		} else {
			ctx->icvs_sensor_state = CV_SENSOR_VISION_ACQUIRED_STATE;
			dev_info(cvs->dev, "%s:Ownership transfer after fw_dl success",
					 __func__);
		}
	} else {
		ctx->icvs_sensor_state = CV_SENSOR_VISION_ACQUIRED_STATE;
	}

	ctx->update_complete_event_arg = 1;
	wake_up_interruptible(&ctx->update_complete_event);
	ctx->fw_dl_task_finished = true;
	dev_info(cvs->dev, "%s:Exiting fw_dl thread", __func__);
}
