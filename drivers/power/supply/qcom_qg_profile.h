/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __QCOM_QG_PROFILE_H__
#define __QCOM_QG_PROFILE_H__

#include <linux/types.h>

struct device;
struct fwnode_handle;

enum qg_table {
	QG_TABLE_OCV1,
	QG_TABLE_OCV2,
	QG_TABLE_FCC1,
	QG_TABLE_FCC2,
	QG_TABLE_Z1,
	QG_TABLE_Z2,
	QG_TABLE_Z3,
	QG_TABLE_Z4,
	QG_TABLE_Z5,
	QG_TABLE_Z6,
	QG_TABLE_Y1,
	QG_TABLE_Y2,
	QG_TABLE_Y3,
	QG_TABLE_Y4,
	QG_TABLE_Y5,
	QG_TABLE_Y6,
	QG_TABLE_MAX,
};

struct qg_lut {
	unsigned int rows;
	unsigned int cols;
	const s32 *row;
	const s32 *col;
	const s32 *data;
};

struct qg_profile {
	struct qg_lut lut[QG_TABLE_MAX];
};

int qg_profile_parse(struct device *dev, struct fwnode_handle *node, struct qg_profile *profile);
int qg_profile_soc(const struct qg_profile *profile, enum qg_table table, int temp, int ocv_uv);
int qg_profile_ocv(const struct qg_profile *profile, enum qg_table table, int temp, int soc);
int qg_profile_fcc(const struct qg_profile *profile, enum qg_table table, int temp);
int qg_profile_slope(const struct qg_profile *profile, enum qg_table table, int temp, int soc);
int qg_profile_var(const struct qg_profile *profile, enum qg_table table, int temp, int soc);

#endif
