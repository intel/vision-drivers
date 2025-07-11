/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *
 * Copyright (C) 2024 Intel Corporation.
 *
 */

#ifndef __INTEL_CVS_UPDATE_H__
#define __INTEL_CVS_UPDATE_H__

int cvs_reset_cv_device(void);
void cvs_fw_dl_thread(struct work_struct *arg);
int cvs_get_fwver_vid_pid(void);
int cvs_find_magic_num_support(struct intel_cvs *ctx);

int cvs_acquire_camera_sensor_internal(void);
int cvs_release_camera_sensor_internal(void);
int cvs_read_i2c(u16 cmd, char *data, int size);
int cvs_write_i2c(u16 cmd, u8 *data, u32 len);
int cvs_get_device_state(u8 *cv_fw_state);
int cvs_get_device_cap(struct cv_ver_capability *cv_fw_cap);
int cvs_wait_for_host_wake(u64 time_ms);
int cvs_dev_fw_dl(void);
int cvs_dev_fw_dl_start(void);
int cvs_dev_fw_dl_data(void);
int cvs_dev_fw_dl_end(void);
#endif // __INTEL_CVS_UPDATE_H__
