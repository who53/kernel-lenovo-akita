/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2026 Deepak Meena <who53@disroot.org> */

#include "membrane_drv.h"

void membrane_gem_free_object(struct drm_gem_object* gem_obj) {
    struct membrane_gem_object* obj = to_membrane_gem(gem_obj);

    if (obj->dmabuf_file)
        fput(obj->dmabuf_file);

    drm_gem_object_release(gem_obj);
    kfree(obj);
}

int membrane_prime_fd_to_handle(
    struct drm_device* dev, struct drm_file* file_priv, int prime_fd, uint32_t* handle) {
    struct membrane_gem_object* obj;
    struct file* dmabuf_file;
    int ret;

    dmabuf_file = fget(prime_fd);
    if (!dmabuf_file) {
        membrane_err("prime_fd_to_handle: fget failed");
        return -EBADF;
    }

    obj = kzalloc(sizeof(*obj), GFP_KERNEL);
    if (!obj) {
        fput(dmabuf_file);
        return -ENOMEM;
    }

    ret = drm_gem_object_init(dev, &obj->base, 0);
    if (ret) {
        membrane_err("prime_fd_to_handle: drm_gem_object_init failed");
        kfree(obj);
        fput(dmabuf_file);
        return ret;
    }

    obj->dmabuf_file = dmabuf_file;

    ret = drm_gem_handle_create(file_priv, &obj->base, handle);
    if (ret) {
        membrane_err("prime_fd_to_handle: drm_gem_handle_create failed");
        drm_gem_object_release(&obj->base);
        kfree(obj);
        fput(dmabuf_file);
        return ret;
    }

    drm_gem_object_put(&obj->base);

    return 0;
}

int membrane_prime_handle_to_fd(struct drm_device* dev, struct drm_file* file_priv, uint32_t handle,
    uint32_t flags, int* prime_fd) {
    membrane_err("shouldnt get called");
    return -ENOSYS;
}
