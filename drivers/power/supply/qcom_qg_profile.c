// SPDX-License-Identifier: GPL-2.0-only

#include <linux/device.h>
#include <linux/minmax.h>
#include <linux/property.h>

#include "qcom_qg_profile.h"

static const char * const qg_table_names[QG_TABLE_MAX] = {
	[QG_TABLE_OCV1] = "qcom,pc-temp-v1-lut",
	[QG_TABLE_OCV2] = "qcom,pc-temp-v2-lut",
	[QG_TABLE_FCC1] = "qcom,fcc1-temp-lut",
	[QG_TABLE_FCC2] = "qcom,fcc2-temp-lut",
	[QG_TABLE_Z1] = "qcom,pc-temp-z1-lut",
	[QG_TABLE_Z2] = "qcom,pc-temp-z2-lut",
	[QG_TABLE_Z3] = "qcom,pc-temp-z3-lut",
	[QG_TABLE_Z4] = "qcom,pc-temp-z4-lut",
	[QG_TABLE_Z5] = "qcom,pc-temp-z5-lut",
	[QG_TABLE_Z6] = "qcom,pc-temp-z6-lut",
	[QG_TABLE_Y1] = "qcom,pc-temp-y1-lut",
	[QG_TABLE_Y2] = "qcom,pc-temp-y2-lut",
	[QG_TABLE_Y3] = "qcom,pc-temp-y3-lut",
	[QG_TABLE_Y4] = "qcom,pc-temp-y4-lut",
	[QG_TABLE_Y5] = "qcom,pc-temp-y5-lut",
	[QG_TABLE_Y6] = "qcom,pc-temp-y6-lut",
};

static int qg_lerp(int y0, int x0, int y1, int x1, int x)
{
	if (y0 == y1 || x == x0)
		return y0;
	if (x1 == x0 || x == x1)
		return y1;

	return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
}

static bool qg_between(int left, int right, int value)
{
	return (left >= right && left >= value && value >= right) ||
	       (left <= right && left <= value && value <= right);
}

static unsigned int qg_lut_col(const struct qg_lut *lut, int *temp)
{
	unsigned int j;

	*temp = clamp(*temp, lut->col[0] * 10, lut->col[lut->cols - 1] * 10);
	for (j = 0; j < lut->cols - 1; j++)
		if (*temp <= lut->col[j] * 10)
			break;

	return j;
}

static int qg_lut_at(const struct qg_lut *lut, unsigned int row, unsigned int j, int temp)
{
	const s32 *d = lut->data + row * lut->cols;

	if (temp == lut->col[j] * 10)
		return d[j];

	return qg_lerp(d[j - 1], lut->col[j - 1] * 10, d[j], lut->col[j] * 10, temp);
}

static int qg_lut_var(const struct qg_lut *lut, int temp, int soc)
{
	unsigned int i, j, r1, r2;

	if (soc > lut->row[0]) {
		r1 = 0;
		r2 = 0;
	} else if (soc < lut->row[lut->rows - 1]) {
		r1 = lut->rows - 1;
		r2 = lut->rows - 1;
	} else {
		for (i = 0; soc < lut->row[i]; i++)
			;
		r2 = i;
		r1 = soc == lut->row[i] ? i : i - 1;
	}

	j = qg_lut_col(lut, &temp);

	return qg_lerp(qg_lut_at(lut, r1, j, temp), lut->row[r1],
		       qg_lut_at(lut, r2, j, temp), lut->row[r2], soc);
}

int qg_profile_var(const struct qg_profile *profile, enum qg_table table, int temp, int soc)
{
	return clamp(qg_lut_var(&profile->lut[table], temp, soc), 0, 65535);
}

int qg_profile_ocv(const struct qg_profile *profile, enum qg_table table, int temp, int soc)
{
	return clamp(qg_lut_var(&profile->lut[table], temp, soc) * 100, 2000000, 5000000);
}

int qg_profile_fcc(const struct qg_profile *profile, enum qg_table table, int temp)
{
	const struct qg_lut *lut = &profile->lut[table];
	unsigned int j = qg_lut_col(lut, &temp);

	return clamp(qg_lut_at(lut, 0, j, temp), 100, 16000);
}

int qg_profile_slope(const struct qg_profile *profile, enum qg_table table, int temp, int soc)
{
	const struct qg_lut *lut = &profile->lut[table];
	unsigned int j, r1, r2;
	int slope;

	if (soc >= lut->row[0]) {
		r2 = 1;
	} else if (soc <= lut->row[lut->rows - 1]) {
		r2 = lut->rows - 1;
	} else {
		for (r2 = 0; soc < lut->row[r2]; r2++)
			;
	}
	r1 = r2 - 1;

	j = qg_lut_col(lut, &temp);
	slope = max(qg_lut_at(lut, r1, j, temp) - qg_lut_at(lut, r2, j, temp), 1);
	slope = slope * 10000 / (lut->row[r1] - lut->row[r2]);

	return clamp(slope, 1, 50000);
}

static int qg_lut_soc(const struct qg_lut *lut, int temp, int ocv)
{
	unsigned int i, j, last = lut->rows - 1;
	int high = 0, low = 0;
	const s32 *d = lut->data;

	j = qg_lut_col(lut, &temp);

	if (ocv >= d[j])
		return lut->row[0];

	if (temp == lut->col[j] * 10) {
		if (ocv <= d[last * lut->cols + j])
			return lut->row[last];

		for (i = 1; ocv < d[i * lut->cols + j]; i++)
			;
		if (ocv == d[i * lut->cols + j])
			return lut->row[i];

		return qg_lerp(lut->row[i], d[i * lut->cols + j],
			       lut->row[i - 1], d[(i - 1) * lut->cols + j], ocv);
	}

	if (ocv <= d[last * lut->cols + j - 1])
		return lut->row[last];

	for (i = 0; i < last; i++) {
		const s32 *a = d + i * lut->cols + j, *b = a + lut->cols;

		if (!high && qg_between(a[0], b[0], ocv))
			high = qg_lerp(lut->row[i], a[0], lut->row[i + 1], b[0], ocv);
		if (!low && qg_between(a[-1], b[-1], ocv))
			low = qg_lerp(lut->row[i], a[-1], lut->row[i + 1], b[-1], ocv);
		if (high && low)
			return qg_lerp(low, lut->col[j - 1] * 10, high, lut->col[j] * 10, temp);
	}

	if (high)
		return high;
	if (low)
		return low;

	return 10000;
}

int qg_profile_soc(const struct qg_profile *profile, enum qg_table table, int temp, int ocv_uv)
{
	return clamp(qg_lut_soc(&profile->lut[table], temp, ocv_uv / 100), 0, 10000);
}

static const s32 *qg_lut_read(struct device *dev, struct fwnode_handle *node, const char *name,
			      unsigned int count)
{
	u32 *values;
	int ret;

	values = devm_kcalloc(dev, count, sizeof(*values), GFP_KERNEL);
	if (!values)
		return ERR_PTR(-ENOMEM);

	ret = fwnode_property_read_u32_array(node, name, values, count);
	if (ret)
		return ERR_PTR(ret);

	return (const s32 *)values;
}

static int qg_lut_parse(struct device *dev, struct fwnode_handle *node, struct qg_lut *lut,
			bool has_rows)
{
	unsigned int i;
	int cols, rows = 1, count;

	cols = fwnode_property_count_u32(node, "qcom,lut-col-legend");
	if (has_rows)
		rows = fwnode_property_count_u32(node, "qcom,lut-row-legend");
	count = fwnode_property_count_u32(node, "qcom,lut-data");
	if (cols < 1 || cols > 16 || rows < (has_rows ? 2 : 1) || rows > 128 || count != rows * cols)
		return -EINVAL;

	lut->cols = cols;
	lut->rows = rows;
	lut->col = qg_lut_read(dev, node, "qcom,lut-col-legend", cols);
	if (IS_ERR(lut->col))
		return PTR_ERR(lut->col);
	lut->data = qg_lut_read(dev, node, "qcom,lut-data", count);
	if (IS_ERR(lut->data))
		return PTR_ERR(lut->data);

	for (i = 1; i < lut->cols; i++)
		if (lut->col[i] <= lut->col[i - 1])
			return -EINVAL;

	if (!has_rows)
		return 0;

	lut->row = qg_lut_read(dev, node, "qcom,lut-row-legend", rows);
	if (IS_ERR(lut->row))
		return PTR_ERR(lut->row);

	for (i = 1; i < lut->rows; i++)
		if (lut->row[i] >= lut->row[i - 1])
			return -EINVAL;

	return 0;
}

int qg_profile_parse(struct device *dev, struct fwnode_handle *node, struct qg_profile *profile)
{
	unsigned int i;
	int ret;

	for (i = 0; i < QG_TABLE_MAX; i++) {
		struct fwnode_handle *child = fwnode_get_named_child_node(node, qg_table_names[i]);

		if (!child)
			return dev_err_probe(dev, -EINVAL, "%s missing in the battery profile\n",
					     qg_table_names[i]);

		ret = qg_lut_parse(dev, child, &profile->lut[i],
				   i != QG_TABLE_FCC1 && i != QG_TABLE_FCC2);
		fwnode_handle_put(child);
		if (ret)
			return dev_err_probe(dev, ret, "bad table %s in the battery profile\n",
					     qg_table_names[i]);
	}

	return 0;
}
