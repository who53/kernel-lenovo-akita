/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2026 Deepak Meena <who53@disroot.org> */

#include "membrane_drv.h"

void membrane_send_event(struct membrane_device* mdev, u32 flags, u32 value) {
    if (atomic_read(&mdev->stopping))
        return;

    if (flags & MEMBRANE_DPMS_UPDATED)
        atomic_set(&mdev->dpms_state, value);

    mdev->pending_event.flags = flags;
    mdev->pending_event.value = value;
    complete(&mdev->event_done);
}

int membrane_signal(struct drm_device* dev, void* data, struct drm_file* file_priv) {
    struct membrane_device* mdev = container_of(dev, struct membrane_device, dev);
    struct membrane_event* arg = data;

    if (wait_for_completion_interruptible(&mdev->event_done))
        return -ERESTARTSYS;

    *arg = mdev->pending_event;
    reinit_completion(&mdev->event_done);

    return 0;
}

enum hrtimer_restart membrane_vblank_timer_fn(struct hrtimer* timer) {
    struct membrane_device* mdev = container_of(timer, struct membrane_device, vblank_timer);
    struct drm_framebuffer *fb, *old;

    fb = xchg(&mdev->pending_state, NULL);
    if (fb) {
        struct membrane_framebuffer* mfb = to_membrane_fb(fb);
        unsigned int count = 0;
        unsigned int i;

        old = xchg(&mdev->active_state, fb);
        if (old)
            drm_framebuffer_put(old);

        for (i = 0; i < MEMBRANE_MAX_FDS; i++)
            if (mfb->objs[i])
                count++;

        membrane_send_event(mdev, MEMBRANE_PRESENT_UPDATED, count);
    }

    drm_crtc_handle_vblank(&mdev->crtc);

    if (READ_ONCE(mdev->pending_state)) {
        int r = READ_ONCE(mdev->r);
        if (r <= 0)
            r = 60;
        hrtimer_forward_now(timer, ns_to_ktime(1000000000ULL / r));
        return HRTIMER_RESTART;
    }

    return HRTIMER_NORESTART;
}

int membrane_config(struct drm_device* dev, void* data, struct drm_file* file_priv) {
    struct membrane_device* mdev = container_of(dev, struct membrane_device, dev);
    struct membrane_u2k_cfg* cfg = data;
    bool mode_changed = false;

    if (!READ_ONCE(mdev->event_consumer)) {
        WRITE_ONCE(mdev->event_consumer, file_priv);
        atomic_set(&mdev->stopping, 0);
    }

    if (READ_ONCE(mdev->w) != cfg->w || READ_ONCE(mdev->h) != cfg->h
        || READ_ONCE(mdev->r) != cfg->r) {
        WRITE_ONCE(mdev->w, cfg->w);
        WRITE_ONCE(mdev->h, cfg->h);
        WRITE_ONCE(mdev->r, cfg->r);
        mode_changed = true;
    }

    if (mode_changed)
        drm_kms_helper_hotplug_event(&mdev->dev);

    return 0;
}

static void membrane_fb_destroy(struct drm_framebuffer* fb) {
    struct membrane_framebuffer* mfb = to_membrane_fb(fb);
    unsigned int i;

    for (i = 0; i < MEMBRANE_MAX_FDS; i++) {
        if (mfb->objs[i])
            drm_gem_object_put(mfb->objs[i]);
    }

    drm_framebuffer_cleanup(fb);
    kfree(mfb);
}

static const struct drm_framebuffer_funcs membrane_fb_funcs = {
    .destroy = membrane_fb_destroy,
};

struct drm_framebuffer* membrane_fb_create(
    struct drm_device* dev, struct drm_file* file_priv, const struct drm_mode_fb_cmd2* mode_cmd) {
    struct membrane_framebuffer* mfb;
    unsigned int i;
    int ret = 0;

    mfb = kzalloc(sizeof(*mfb), GFP_KERNEL);
    if (!mfb)
        return ERR_PTR(-ENOMEM);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 11, 0)
    drm_helper_mode_fill_fb_struct(&mfb->base, mode_cmd);
#else
    drm_helper_mode_fill_fb_struct(dev, &mfb->base, mode_cmd);
#endif

    for (i = 0; i < MEMBRANE_MAX_FDS; i++) {

        if (!mode_cmd->handles[i])
            continue;

        mfb->objs[i] = drm_gem_object_lookup(file_priv, mode_cmd->handles[i]);

        if (!mfb->objs[i]) {
            ret = -ENOENT;
            goto err;
        }
    }

    ret = drm_framebuffer_init(dev, &mfb->base, &membrane_fb_funcs);
    if (ret)
        goto err;

    return &mfb->base;

err:
    while (i--) {
        if (mfb->objs[i])
            drm_gem_object_put(mfb->objs[i]);
    }

    kfree(mfb);
    return ERR_PTR(ret);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0)
void membrane_crtc_enable(struct drm_crtc* crtc) {
#elif LINUX_VERSION_CODE < KERNEL_VERSION(5, 11, 0)
void membrane_crtc_enable(struct drm_crtc* crtc, struct drm_crtc_state* state) {
#else
void membrane_crtc_enable(struct drm_crtc* crtc, struct drm_atomic_state* state) {
#endif
    struct membrane_device* mdev = container_of(crtc->dev, struct membrane_device, dev);

    membrane_send_event(mdev, MEMBRANE_DPMS_UPDATED, MEMBRANE_DPMS_ON);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 11, 0)
void membrane_crtc_disable(struct drm_crtc* crtc, struct drm_crtc_state* old_state) {
#else
void membrane_crtc_disable(struct drm_crtc* crtc, struct drm_atomic_state* state) {
#endif
    struct membrane_device* mdev = container_of(crtc->dev, struct membrane_device, dev);
    struct drm_framebuffer* old;

    old = xchg(&mdev->active_state, NULL);
    if (old)
        drm_framebuffer_put(old);
    old = xchg(&mdev->pending_state, NULL);
    if (old)
        drm_framebuffer_put(old);

    hrtimer_cancel(&mdev->vblank_timer);

    if (crtc->dev->master) {
        membrane_send_event(mdev, MEMBRANE_DPMS_UPDATED, MEMBRANE_DPMS_OFF);
    } else {
        membrane_send_event(mdev, MEMBRANE_DPMS_UPDATED, MEMBRANE_DPMS_NO_COMP);
    }
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 13, 0)
void membrane_plane_atomic_update(struct drm_plane* plane, struct drm_plane_state* old_state) {
    struct drm_plane_state* new_state = plane->state;
#else
void membrane_plane_atomic_update(struct drm_plane* plane, struct drm_atomic_state* state) {
    struct drm_plane_state* new_state = drm_atomic_get_new_plane_state(state, plane);
#endif
    struct drm_framebuffer* fb = new_state->fb;
    struct membrane_device* mdev = container_of(plane->dev, struct membrane_device, dev);
    struct drm_framebuffer* old;

    if (!fb)
        return;

    drm_framebuffer_get(fb);
    old = xchg(&mdev->pending_state, fb);
    if (old)
        drm_framebuffer_put(old);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 13, 0)
void membrane_plane_atomic_disable(struct drm_plane* plane, struct drm_plane_state* old_state) {
#else
void membrane_plane_atomic_disable(struct drm_plane* plane, struct drm_atomic_state* state) {
#endif
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 11, 0)
void membrane_crtc_atomic_flush(struct drm_crtc* crtc, struct drm_crtc_state* old_crtc_state) {
#else
void membrane_crtc_atomic_flush(struct drm_crtc* crtc, struct drm_atomic_state* state) {
#endif
    struct membrane_device* mdev = container_of(crtc->dev, struct membrane_device, dev);
    struct drm_pending_vblank_event* event = crtc->state->event;

    if (READ_ONCE(mdev->pending_state) && !hrtimer_active(&mdev->vblank_timer)) {
        int r = READ_ONCE(mdev->r);
        if (r <= 0)
            r = 60;
        hrtimer_start(&mdev->vblank_timer, ns_to_ktime(1000000000ULL / r), HRTIMER_MODE_REL);
    }

    if (event) {
        crtc->state->event = NULL;
        spin_lock_irq(&crtc->dev->event_lock);
        drm_crtc_send_vblank_event(crtc, event);
        spin_unlock_irq(&crtc->dev->event_lock);
    }
}

int membrane_get_present_fd(struct drm_device* dev, void* data, struct drm_file* file) {
    struct membrane_device* mdev = container_of(dev, struct membrane_device, dev);
    struct membrane_get_present_fd* args = data;
    struct drm_framebuffer* fb;
    struct membrane_framebuffer* mfb;
    unsigned int i;
    int count = 0;

    fb = xchg(&mdev->active_state, NULL);
    if (!fb) {
        args->buffer_id = 0;
        args->num_fds = 0;
        for (i = 0; i < MEMBRANE_MAX_FDS; i++)
            args->fds[i] = -1;
        return 0;
    }

    mfb = to_membrane_fb(fb);
    args->buffer_id = fb->base.id;

    for (i = 0; i < MEMBRANE_MAX_FDS; i++) {

        struct drm_gem_object* obj = mfb->objs[i];
        struct membrane_gem_object* mobj;
        int fd;

        if (!obj) {
            args->fds[i] = -1;
            continue;
        }

        mobj = to_membrane_gem(obj);

        get_file(mobj->dmabuf_file);

        fd = get_unused_fd_flags(O_CLOEXEC);
        if (fd < 0) {
            fput(mobj->dmabuf_file);
            args->fds[i] = -1;
            continue;
        }

        fd_install(fd, mobj->dmabuf_file);
        args->fds[i] = fd;
        count++;
    }

    args->num_fds = count;

    drm_framebuffer_put(fb);
    return 0;
}

u32 membrane_get_vblank_counter(struct drm_device* dev, unsigned int pipe) { return 0; }

int membrane_enable_vblank(struct drm_device* dev, unsigned int pipe) { return 0; }

void membrane_disable_vblank(struct drm_device* dev, unsigned int pipe) { }
