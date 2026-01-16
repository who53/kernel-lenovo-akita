/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2026 Deepak Meena <who53@disroot.org> */

#include <linux/module.h>
#include <linux/platform_device.h>

#include "membrane_drv.h"

static const struct drm_mode_config_funcs membrane_mode_config_funcs = {
    .fb_create = membrane_fb_create,
    .atomic_check = drm_atomic_helper_check,
    .atomic_commit = drm_atomic_helper_commit,
};

static const struct drm_plane_funcs membrane_plane_funcs = {
    .update_plane = drm_atomic_helper_update_plane,
    .disable_plane = drm_atomic_helper_disable_plane,
    .destroy = drm_plane_cleanup,
    .reset = drm_atomic_helper_plane_reset,
    .atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
    .atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

static const struct drm_plane_helper_funcs membrane_plane_helper_funcs = {
    .atomic_update = membrane_plane_atomic_update,
    .atomic_disable = membrane_plane_atomic_disable,
};

static const struct drm_crtc_helper_funcs membrane_crtc_helper_funcs = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0)
    .enable = membrane_crtc_enable,
#else
    .atomic_enable = membrane_crtc_enable,
#endif
    .atomic_disable = membrane_crtc_disable,
    .atomic_flush = membrane_crtc_atomic_flush,
};

static const struct drm_crtc_funcs membrane_crtc_funcs = {
    .destroy = drm_crtc_cleanup,
    .set_config = drm_atomic_helper_set_config,
    .page_flip = drm_atomic_helper_page_flip,
    .reset = drm_atomic_helper_crtc_reset,
    .atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
    .atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
};

static const struct drm_encoder_funcs membrane_encoder_funcs = {
    .destroy = drm_encoder_cleanup,
};

void membrane_atomic_commit_tail(struct drm_atomic_state* state) {
    struct drm_device* dev = state->dev;

    drm_atomic_helper_commit_modeset_disables(dev, state);
    drm_atomic_helper_commit_planes(dev, state, 0);
    drm_atomic_helper_commit_modeset_enables(dev, state);
    drm_atomic_helper_commit_hw_done(state);

    drm_atomic_helper_cleanup_planes(dev, state);
}

static struct drm_mode_config_helper_funcs membrane_mode_config_helper_funcs = {
    .atomic_commit_tail = membrane_atomic_commit_tail,
};

static int membrane_connector_get_modes(struct drm_connector* connector) {
    struct drm_display_mode* mode;
    struct membrane_device* mdev = container_of(connector->dev, struct membrane_device, dev);

    mode = drm_cvt_mode(connector->dev, mdev->w, mdev->h, mdev->r, false, false, false);
    if (!mode) {
        membrane_err("drm_cvt_mode failed");
        return 0;
    }

    mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
    drm_mode_set_name(mode);
    drm_mode_probed_add(connector, mode);

    return 1;
}

static enum drm_mode_status membrane_connector_mode_valid(
    struct drm_connector* connector, struct drm_display_mode* mode) {
    return MODE_OK;
}

static const struct drm_connector_helper_funcs membrane_connector_helper_funcs = {
    .get_modes = membrane_connector_get_modes,
    .mode_valid = membrane_connector_mode_valid,
};

static enum drm_connector_status membrane_connector_detect(
    struct drm_connector* connector, bool force) {
    return connector_status_connected;
}

static const struct drm_connector_funcs membrane_connector_funcs = {
    .detect = membrane_connector_detect,
    .fill_modes = drm_helper_probe_single_connector_modes,
    .destroy = drm_connector_cleanup,
    .reset = drm_atomic_helper_connector_reset,
    .atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
    .atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const uint32_t membrane_formats[] = {
    DRM_FORMAT_ARGB8888,
    DRM_FORMAT_XRGB8888,
};

static int membrane_load(struct membrane_device* mdev) {
    struct drm_device* dev = &mdev->dev;
    int ret;

    mdev->w = 1920;
    mdev->h = 1080;
    mdev->r = 60;

    init_completion(&mdev->event_done);

    hrtimer_init(&mdev->vblank_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    mdev->vblank_timer.function = membrane_vblank_timer_fn;

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 8, 0)
    drm_mode_config_init(dev);
#else
    ret = drm_mode_config_init(dev);
    if (ret) {
        membrane_err("drm_mode_config_init failed: %d", ret);
        return ret;
    }
#endif

    dev->mode_config.min_width = 0;
    dev->mode_config.min_height = 0;
    dev->mode_config.max_width = 4096;
    dev->mode_config.max_height = 4096;
    dev->mode_config.funcs = &membrane_mode_config_funcs;
    dev->mode_config.helper_private = &membrane_mode_config_helper_funcs;

    ret = drm_universal_plane_init(dev, &mdev->plane, 1, &membrane_plane_funcs, membrane_formats,
        ARRAY_SIZE(membrane_formats),
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0)
        DRM_PLANE_TYPE_PRIMARY, NULL);
#else
        NULL, DRM_PLANE_TYPE_PRIMARY, NULL);
#endif
    if (ret) {
        membrane_err("drm_universal_plane_init failed: %d", ret);
        return ret;
    }

    drm_plane_helper_add(&mdev->plane, &membrane_plane_helper_funcs);

    drm_crtc_helper_add(&mdev->crtc, &membrane_crtc_helper_funcs);
    ret = drm_crtc_init_with_planes(
        dev, &mdev->crtc, &mdev->plane, NULL, &membrane_crtc_funcs, NULL);
    if (ret) {
        membrane_err("drm_crtc_init_with_planes failed: %d", ret);
        return ret;
    }

    ret = drm_encoder_init(
        dev, &mdev->encoder, &membrane_encoder_funcs, DRM_MODE_ENCODER_DSI, NULL);
    if (ret) {
        membrane_err("drm_encoder_init failed: %d", ret);
        return ret;
    }

    mdev->encoder.possible_crtcs = 1 << drm_crtc_index(&mdev->crtc);

    ret = drm_connector_init(
        dev, &mdev->connector, &membrane_connector_funcs, DRM_MODE_CONNECTOR_DSI);
    if (ret) {
        membrane_err("drm_connector_init failed: %d", ret);
        return ret;
    }

    drm_connector_helper_add(&mdev->connector, &membrane_connector_helper_funcs);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 19, 0)
    ret = drm_mode_connector_attach_encoder(&mdev->connector, &mdev->encoder);
#else
    ret = drm_connector_attach_encoder(&mdev->connector, &mdev->encoder);
#endif
    if (ret) {
        membrane_err("drm_mode_connector_attach_encoder failed: %d", ret);
        return ret;
    }

    ret = drm_vblank_init(dev, 1);
    if (ret) {
        membrane_err("drm_vblank_init failed: %d", ret);
        return ret;
    }

    drm_mode_config_reset(dev);

    return 0;
}

static void membrane_postclose(struct drm_device* dev, struct drm_file* file) {
    struct membrane_device* mdev = container_of(dev, struct membrane_device, dev);

    if (READ_ONCE(mdev->event_consumer) == file) {
        struct drm_framebuffer* old;

        WRITE_ONCE(mdev->event_consumer, NULL);
        atomic_set(&mdev->stopping, 1);
        complete_all(&mdev->event_done);

        hrtimer_cancel(&mdev->vblank_timer);

        old = xchg(&mdev->active_state, NULL);
        if (old)
            drm_framebuffer_put(old);
        old = xchg(&mdev->pending_state, NULL);
        if (old)
            drm_framebuffer_put(old);
    }
}

static const struct file_operations membrane_fops = {
    .owner = THIS_MODULE,
    .open = drm_open,
    .release = drm_release,
    .unlocked_ioctl = drm_ioctl,
    .compat_ioctl = drm_compat_ioctl,
    .poll = drm_poll,
    .read = drm_read,
    .llseek = noop_llseek,
};

static struct drm_driver membrane_driver = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)
    .driver_features = DRIVER_MODESET | DRIVER_PRIME | DRIVER_GEM | DRIVER_ATOMIC,
#else
    .driver_features = DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,
#endif
    .fops = &membrane_fops,
    .name = "membrane",
    .desc = "membrane",
    .date = "20260119",
    .major = 1,
    .minor = 0,
    .patchlevel = 0,
    .prime_fd_to_handle = membrane_prime_fd_to_handle,
    .prime_handle_to_fd = membrane_prime_handle_to_fd,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 9, 0)
    .gem_free_object = membrane_gem_free_object,
#else
    .gem_free_object_unlocked = membrane_gem_free_object,
#endif
    .ioctls = membrane_ioctls,
    .num_ioctls = ARRAY_SIZE(membrane_ioctls),
    .postclose = membrane_postclose,
    .get_vblank_counter = membrane_get_vblank_counter,
    .enable_vblank = membrane_enable_vblank,
    .disable_vblank = membrane_disable_vblank,
};

static int membrane_probe(struct platform_device* pdev) {
    struct membrane_device* mdev;
    struct drm_device* dev;
    int ret;

    membrane_debug("probe %s", pdev->name);

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
    mdev = devm_kzalloc(&pdev->dev, sizeof(*mdev), GFP_KERNEL);
    if (!mdev) {
        membrane_err("devm_kzalloc failed");
        return -ENOMEM;
    }
#else
    mdev = devm_drm_dev_alloc(&pdev->dev, &membrane_driver, typeof(*mdev), dev);
    if (IS_ERR(mdev)) {
        membrane_err("devm_drm_dev_alloc failed");
        return PTR_ERR(mdev);
    }
#endif

    dev = &mdev->dev;

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
    membrane_debug("drm_dev_init");
    ret = drm_dev_init(dev, &membrane_driver, &pdev->dev);
    if (ret) {
        membrane_err("drm_dev_init failed: %d", ret);
        return ret;
    }
    membrane_debug("drm_dev_init done");
#endif

    platform_set_drvdata(pdev, dev);

    ret = membrane_load(mdev);
    if (ret) {
        membrane_err("membrane_load failed: %d", ret);
        goto err_free;
    }

    ret = drm_dev_register(dev, 0);
    if (ret) {
        membrane_err("drm_dev_register failed: %d", ret);
        goto err_free;
    }

    membrane_debug("probe success");
    return 0;

err_free:
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
    drm_dev_put(dev);
#endif
    membrane_err("probe failed");
    return ret;
}

static int membrane_remove(struct platform_device* pdev) {
    struct drm_device* drm = platform_get_drvdata(pdev);
    membrane_debug("remove");

    drm_dev_unregister(drm);
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
    drm_dev_put(drm);
#endif

    return 0;
}

static struct platform_driver membrane_platform_driver = {
    .probe = membrane_probe,
    .remove = membrane_remove,
    .driver = {
        .name = "membrane",
    },
};

static struct platform_device* membrane_pdev;

static int __init membrane_init(void) {
    int ret;

    membrane_debug("init");

    ret = platform_driver_register(&membrane_platform_driver);
    if (ret) {
        membrane_err("platform_driver_register failed");
        return ret;
    }

    membrane_pdev = platform_device_register_simple("membrane", -1, NULL, 0);
    if (IS_ERR(membrane_pdev)) {
        membrane_err("platform_device_register_simple failed");
        platform_driver_unregister(&membrane_platform_driver);
        return PTR_ERR(membrane_pdev);
    }

    return 0;
}

static void __exit membrane_exit(void) {
    membrane_debug("exit");
    platform_device_unregister(membrane_pdev);
    platform_driver_unregister(&membrane_platform_driver);
}

module_init(membrane_init);
module_exit(membrane_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Deepak Meena <who53@disroot.org>");
MODULE_DESCRIPTION("Membrane DRM Driver");
