/* SPDX-License-Identifier: GPL-2.0-only */
/* Onyx Boox Go 7 e-ink */
#ifndef _LINUX_ONYX_EPD_H
#define _LINUX_ONYX_EPD_H

#include <linux/types.h>

struct onyx_epdc_upd {
	__s32 top, left, width, height;
	__s32 waveform_mode;
	__s32 update_mode;
	__s32 update_marker;
	__s32 temp;
	__s32 flag;
	__s32 reserved;
};

#endif
