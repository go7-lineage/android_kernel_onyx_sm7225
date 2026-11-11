// SPDX-License-Identifier: GPL-2.0-only
/*
 * .wbf parser.
 */
#include "epd_core.h"

static u32 u24(const u8 *d, size_t o)
{
	return d[o] | d[o + 1] << 8 | d[o + 2] << 16;
}

static u8 sum8(const u8 *d, size_t from, size_t to)
{
	u8 s = 0;

	while (from < to)
		s += d[from++];
	return s;
}

static int wbf_ptr(const struct epd_wbf *w, size_t o, u32 *out)
{
	const u8 *d = w->data;

	if (o + 4 > w->size || (u8)(d[o] + d[o + 1] + d[o + 2]) != d[o + 3])
		return -EINVAL;
	*out = u24(d, o);
	return 0;
}

static long wbf_rle(const struct epd_wbf *w, u32 off, u8 *out, size_t cap)
{
	const u8 *d = w->data;
	size_t i = off, n = 0, max = (size_t)EPD_MAX_FRAMES * w->levels * w->levels;
	bool literal = false;
	u32 total = 0;
	u32 run, k;
	u8 b;

	for (;;) {
		if (i >= w->size)
			return -EINVAL;
		b = d[i];
		if (b == w->end_byte)
			break;
		if (b == w->toggle_byte) {
			literal = !literal;
			total += b;
			i++;
			continue;
		}
		if (literal) {
			run = 1;
			total += b;
			i++;
		} else {
			if (i + 1 >= w->size)
				return -EINVAL;
			run = d[i + 1] + 1;
			total += b + d[i + 1];
			i += 2;
		}
		if (n + 4 * run > max)
			return -E2BIG;
		if (out) {
			if (n + 4 * run > cap)
				return -ENOSPC;
			for (k = 0; k < run; k++) {
				out[n + 4 * k] = b & 3;
				out[n + 4 * k + 1] = b >> 2 & 3;
				out[n + 4 * k + 2] = b >> 4 & 3;
				out[n + 4 * k + 3] = b >> 6;
			}
		}
		n += 4 * run;
	}
	if (i + 1 >= w->size || d[i + 1] != (u8)(total + w->end_byte))
		return -EINVAL;
	return n;
}

int epd_wbf_parse(struct epd_wbf *w, const u8 *d, size_t size)
{
	u32 mode_table, temp_table, lut_fmt, wfm_rev, n, m, t, l2;
	size_t o;
	long ret;

	memset(w, 0, sizeof(*w));
	w->data = d;
	w->size = size;
	if (size < 0x30)
		return -EINVAL;
	if (sum8(d, 8, 31) != d[31] || sum8(d, 32, 47) != d[47])
		return -EINVAL;

	w->mode_version = d[0x10];
	wfm_rev = d[0x16];
	w->frame_rate = wfm_rev == 2 ? (d[0x17] & 0xf) + (d[0x17] >> 4) * 10 : d[0x18];
	mode_table = u24(d, 0x20);
	lut_fmt = d[0x24];
	w->n_modes = d[0x25] + 1;
	w->n_temps = d[0x26] + 1;
	w->end_byte = d[0x28];
	w->toggle_byte = d[0x29];
	w->adv_flags = d[0x2a];
	w->levels = (lut_fmt == 4 || w->mode_version == 0x90) ? 32 : 16;
	l2 = w->levels * w->levels;

	o = 0x30 + w->n_temps + 3;
	if (o > size)
		return -EINVAL;
	memcpy(w->temp_bounds, d + 0x30, w->n_temps + 1);
	if (sum8(d, 0x30, 0x30 + w->n_temps + 1) != d[0x30 + w->n_temps + 1])
		return -EINVAL;
	n = d[0x30 + w->n_temps + 2];
	if (o + n > size)
		return -EINVAL;
	memcpy(w->name, d + o, n);
	w->name[n] = 0;

	w->block = epd_zalloc(sizeof(*w->block) * w->n_modes * w->n_temps);
	w->nframes = epd_zalloc(sizeof(*w->nframes) * w->n_modes * w->n_temps);
	if (!w->block || !w->nframes) {
		ret = -ENOMEM;
		goto err;
	}
	for (m = 0; m < w->n_modes; m++) {
		ret = wbf_ptr(w, (size_t)mode_table + 4 * m, &temp_table);
		if (ret)
			goto err;
		for (t = 0; t < w->n_temps; t++) {
			u32 *blk = &w->block[m * w->n_temps + t];

			ret = wbf_ptr(w, (size_t)temp_table + 4 * t, blk);
			if (ret)
				goto err;
			ret = wbf_rle(w, *blk, NULL, 0);
			if (ret < 0)
				goto err;
			if (ret % l2) {
				ret = -EINVAL;
				goto err;
			}
			w->nframes[m * w->n_temps + t] = ret / l2;
		}
	}
	return 0;

err:
	epd_wbf_release(w);
	return ret;
}

void epd_wbf_release(struct epd_wbf *w)
{
	epd_free(w->block);
	epd_free(w->nframes);
	w->block = NULL;
	w->nframes = NULL;
}

int epd_wbf_decode(const struct epd_wbf *w, int temp, int mode, u8 *out, size_t cap)
{
	long ret;

	if (temp < 0 || temp >= w->n_temps || mode < 0 || mode >= w->n_modes)
		return -EINVAL;
	ret = wbf_rle(w, w->block[mode * w->n_temps + temp], out, cap);
	return ret < 0 ? ret : ret / (w->levels * w->levels);
}

int epd_temp_index(const struct epd_wbf *w, int temp_c)
{
	int i, idx = 0;

	for (i = 0; i < w->n_temps; i++)
		if (w->temp_bounds[i] <= temp_c)
			idx = i;
	return idx;
}

/* stock wf_trans_info */
static const u8 wf_trans_info[][19] = {
	{ 3, 0, 0, 1, 2, 3, 4, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 255, 255 },
	{ 4, 0, 0, 1, 2, 3, 4, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 255, 255 },
	{ 9, 0, 0, 1, 2, 255, 4, 255, 255, 3, 255, 255, 255, 255, 255, 255, 0, 255, 255 },
	{ 19, 0, 0, 1, 2, 255, 4, 3, 255, 255, 255, 255, 255, 255, 255, 255, 0, 255, 255 },
	{ 21, 0, 0, 1, 2, 5, 4, 255, 255, 3, 255, 255, 255, 255, 255, 255, 0, 255, 255 },
	{ 24, 1, 0, 1, 2, 255, 6, 4, 5, 3, 255, 255, 255, 255, 255, 255, 0, 255, 255 },
	{ 25, 2, 0, 1, 2, 255, 6, 4, 5, 3, 7, 255, 255, 255, 255, 255, 0, 255, 255 },
	{ 25, 0, 0, 1, 2, 255, 6, 4, 5, 3, 7, 255, 255, 255, 255, 255, 0, 9, 8 },
	{ 32, 0, 0, 1, 2, 255, 6, 4, 5, 3, 1, 255, 255, 255, 255, 255, 0, 255, 255 },
	{ 35, 0, 0, 1, 2, 255, 4, 255, 255, 3, 5, 255, 255, 255, 255, 255, 0, 255, 255 },
	{ 72, 0, 0, 1, 2, 255, 6, 4, 255, 3, 255, 255, 255, 255, 255, 255, 0, 8, 7 },
	{ 88, 0, 0, 1, 2, 255, 6, 4, 255, 255, 255, 255, 255, 255, 255, 255, 0, 9, 10 },
	{ 7, 0, 0, 1, 2, 3, 1, 255, 255, 5, 255, 255, 255, 255, 255, 255, 0, 255, 255 },
	{ 48, 0, 0, 1, 2, 3, 1, 5, 255, 255, 255, 255, 255, 255, 255, 255, 0, 255, 255 },
	{ 144, 0, 0, 1, 2, 3, 1, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 255, 255 },
	{ 83, 0, 0, 1, 2, 255, 6, 255, 255, 3, 255, 255, 255, 5, 255, 255, 0, 255, 255 },
	{ 84, 1, 0, 1, 2, 255, 5, 255, 255, 3, 255, 255, 255, 4, 255, 255, 0, 255, 255 },
	{ 22, 1, 0, 1, 2, 255, 6, 4, 255, 3, 255, 255, 255, 5, 255, 4, 0, 8, 7 },
	{ 255, 0, 0, 1, 5, 255, 6, 255, 255, 3, 255, 255, 255, 5, 255, 255, 0, 255, 255 },
	{ 153, 0, 0, 1, 2, 255, 6, 3, 5, 4, 7, 255, 255, 255, 255, 255, 0, 255, 255 },
};

int epd_mode_slot(u8 mode_version, int onyx_mode)
{
	const u8 *row = NULL;
	size_t i;

	if (onyx_mode == 14)
		return 0;
	for (i = 0; i < sizeof(wf_trans_info) / sizeof(wf_trans_info[0]); i++)
		if (wf_trans_info[i][0] == mode_version)
			row = wf_trans_info[i];
	if (!row)
		return onyx_mode;
	return onyx_mode >= 0 && onyx_mode < 17 ? row[2 + onyx_mode] : 0xff;
}

int epd_wbf_fixup_variant(const struct epd_wbf *w, int slot)
{
	if (slot == epd_mode_slot(w->mode_version, 5))
		return 0;
	if (slot == epd_mode_slot(w->mode_version, 15))
		return 1;
	if (slot == epd_mode_slot(w->mode_version, 16) && (w->adv_flags & 0x80))
		return 2;
	return -1;
}
