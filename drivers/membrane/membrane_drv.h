/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2026 Deepak Meena <who53@disroot.org> */

#ifndef _MEMBRANE_DRV_H_
#define _MEMBRANE_DRV_H_

#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/file.h>
#include <linux/spinlock.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 13, 0)
#else
#include <drm/drm_drv.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_vblank.h>
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 1, 0)
#else
#include <drm/drm_probe_helper.h>
#endif

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_encoder.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_gem.h>
#include <drm/drm_plane_helper.h>
#include "uapi/membrane.h"



struct membrane_gem_object {
    struct drm_gem_object base;
    struct file* dmabuf_file;
};

static inline struct membrane_gem_object* to_membrane_gem(struct drm_gem_object* obj) {
    return container_of(obj, struct membrane_gem_object, base);
}

struct membrane_framebuffer {
    struct drm_framebuffer base;
    struct drm_gem_object* objs[MEMBRANE_MAX_FDS];
};

static inline struct membrane_framebuffer* to_membrane_fb(struct drm_framebuffer* fb) {
    return container_of(fb, struct membrane_framebuffer, base);
}

struct membrane_device {
    struct drm_device dev;
    struct drm_plane plane;
    struct drm_crtc crtc;
    struct drm_encoder encoder;
    struct drm_connector connector;

    struct drm_file* event_consumer;

    struct drm_framebuffer* active_state;
    struct drm_framebuffer* pending_state;

    struct hrtimer vblank_timer;

    int w, h, r;

    struct completion event_done;
    struct membrane_event pending_event;
    atomic_t dpms_state;
    atomic_t stopping;
};

int membrane_config(struct drm_device* dev, void* data, struct drm_file* file_priv);
int membrane_signal(struct drm_device* dev, void* data, struct drm_file* file_priv);
void membrane_send_event(struct membrane_device* mdev, u32 flags, u32 num_fds);
enum hrtimer_restart membrane_vblank_timer_fn(struct hrtimer* timer);

struct drm_framebuffer* membrane_fb_create(
    struct drm_device* dev, struct drm_file* file_priv, const struct drm_mode_fb_cmd2* mode_cmd);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0)
void membrane_crtc_enable(struct drm_crtc* crtc);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(5, 11, 0)
void membrane_crtc_enable(struct drm_crtc* crtc, struct drm_crtc_state* state);
#else
void membrane_crtc_enable(struct drm_crtc* crtc, struct drm_atomic_state* state);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 11, 0)
void membrane_crtc_disable(struct drm_crtc* crtc, struct drm_crtc_state* old_state);
void membrane_crtc_atomic_flush(struct drm_crtc* crtc, struct drm_crtc_state* old_crtc_state);
#else
void membrane_crtc_disable(struct drm_crtc* crtc, struct drm_atomic_state* state);
void membrane_crtc_atomic_flush(struct drm_crtc* crtc, struct drm_atomic_state* state);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 13, 0)
void membrane_plane_atomic_update(struct drm_plane* plane, struct drm_plane_state* old_state);
void membrane_plane_atomic_disable(struct drm_plane* plane, struct drm_plane_state* old_state);
#else
void membrane_plane_atomic_update(struct drm_plane* plane, struct drm_atomic_state* state);
void membrane_plane_atomic_disable(struct drm_plane* plane, struct drm_atomic_state* state);
#endif

int membrane_prime_fd_to_handle(
    struct drm_device* dev, struct drm_file* file_priv, int prime_fd, uint32_t* handle);
int membrane_prime_handle_to_fd(struct drm_device* dev, struct drm_file* file_priv, uint32_t handle,
    uint32_t flags, int* prime_fd);
int membrane_get_present_fd(struct drm_device* dev, void* data, struct drm_file* file);

u32 membrane_get_vblank_counter(struct drm_device* dev, unsigned int pipe);
int membrane_enable_vblank(struct drm_device* dev, unsigned int pipe);
void membrane_disable_vblank(struct drm_device* dev, unsigned int pipe);

void membrane_gem_free_object(struct drm_gem_object* obj);

static const struct drm_ioctl_desc membrane_ioctls[] = {
    DRM_IOCTL_DEF_DRV(
        MEMBRANE_GET_PRESENT_FD, membrane_get_present_fd, DRM_UNLOCKED | DRM_RENDER_ALLOW),
    DRM_IOCTL_DEF_DRV(MEMBRANE_CONFIG, membrane_config, DRM_UNLOCKED),
    DRM_IOCTL_DEF_DRV(MEMBRANE_SIGNAL, membrane_signal, DRM_UNLOCKED),
};

#define membrane_debug(fmt, ...) pr_debug("membrane: %s: " fmt "\n", __func__, ##__VA_ARGS__)
#define membrane_err(fmt, ...) pr_err("membrane: %s: " fmt "\n", __func__, ##__VA_ARGS__)

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
#define drm_dev_put(dev) drm_dev_unref(dev)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 12, 0)
#define drm_gem_object_put(obj) drm_gem_object_unreference_unlocked(obj)
#define drm_framebuffer_get(obj) drm_framebuffer_reference(obj)
#define drm_framebuffer_put(obj) drm_framebuffer_unreference(obj)
#endif

#endif
