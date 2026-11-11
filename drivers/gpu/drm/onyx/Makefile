# SPDX-License-Identifier: GPL-2.0-only
obj-$(CONFIG_DRM_ONYX_EPD) += onyx-epd.o
onyx-epd-y := onyx_epd_drm.o epd_wbf.o epd_tables.o epd_frame.o epd_engine.o epd_convert.o epd_policy.o epd_ui.o epd_neon.o
ccflags-y += -DEPD_NEON
CFLAGS_epd_neon.o += $(CC_FLAGS_FPU)
CFLAGS_REMOVE_epd_neon.o += $(CC_FLAGS_NO_FPU)
