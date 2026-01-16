/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2026 Deepak Meena <who53@disroot.org> */

#ifndef _UAPI_MEMBRANE_H_
#define _UAPI_MEMBRANE_H_

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#define MEMBRANE_PRESENT_UPDATED (1 << 0)
#define MEMBRANE_DPMS_UPDATED (1 << 1)

#define MEMBRANE_DPMS_OFF 0
#define MEMBRANE_DPMS_ON 1
#define MEMBRANE_DPMS_NO_COMP 2

#define MEMBRANE_MAX_FDS 4

struct membrane_event {
    __u32 flags;
    __u32 value;
};

struct membrane_u2k_cfg {
    int32_t w;
    int32_t h;
    int32_t r;
    int32_t __reserved;
};

struct membrane_get_present_fd {
    __u32 buffer_id;
    __u32 num_fds;
    __s32 fds[MEMBRANE_MAX_FDS];
};

#define DRM_MEMBRANE_GET_PRESENT_FD 0x23
#define DRM_MEMBRANE_CONFIG 0x24
#define DRM_MEMBRANE_SIGNAL 0x25

#define DRM_IOCTL_MEMBRANE_GET_PRESENT_FD                                                          \
    DRM_IOWR(DRM_COMMAND_BASE + DRM_MEMBRANE_GET_PRESENT_FD, struct membrane_get_present_fd)

#define DRM_IOCTL_MEMBRANE_CONFIG                                                                  \
    DRM_IOW(DRM_COMMAND_BASE + DRM_MEMBRANE_CONFIG, struct membrane_u2k_cfg)

#define DRM_IOCTL_MEMBRANE_SIGNAL                                                                  \
    DRM_IOWR(DRM_COMMAND_BASE + DRM_MEMBRANE_SIGNAL, struct membrane_event)

#endif
