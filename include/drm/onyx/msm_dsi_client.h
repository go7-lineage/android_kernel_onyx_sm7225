/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _DRM_ONYX_MSM_DSI_CLIENT_H_
#define _DRM_ONYX_MSM_DSI_CLIENT_H_

struct device_node;
struct drm_device;

#if IS_REACHABLE(CONFIG_DRM_MSM)
struct drm_device *msm_dsi_drm_device(struct device_node *np);
#else
static inline struct drm_device *msm_dsi_drm_device(struct device_node *np) { return NULL; }
#endif

#endif
