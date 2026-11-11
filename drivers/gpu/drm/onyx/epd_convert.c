// SPDX-License-Identifier: GPL-2.0-only
/*
 * Input conversion (E3)
 */
#include "epd_core.h"

/* rgb32_to_4bit_32_neon / ggg32_to_4bit_32_neon: Y = (77 R + 151 G + 28 B) >> 8, then & mask */
void epd_rgba_to_grey_scalar(const u8 *src, size_t sstride, u8 *dst, size_t dstride, u32 w, u32 h, u8 mask, bool ggg){
	u32 x, y;

	for (y = 0; y < h; y++, src += sstride, dst += dstride)
		for (x = 0; x < w; x++) {
			const u8 *p = src + 4 * x;

			dst[x] = (ggg ? p[0] : (77 * p[0] + 151 * p[1] + 28 * p[2]) >> 8) & mask;
		}
}

/* onyx_gamma_correction */
void epd_gamma(u8 *buf, size_t stride, u32 w, u32 h, const u8 *lut)
{
	u32 x, y;

	for (y = 0; y < h; y++, buf += stride)
		for (x = 0; x < w; x++)
			buf[x] = lut[buf[x]];
}

#define ERR(row, x)	err[((row) % 3) * (w + 3) + (x) + 1]

static void spread_next_row(int *e1, const int *v, u32 w)
{
	int x;

	e1[-1] += v[0];
	for (x = 0; x < (int)w; x++)
		e1[x] += (x ? v[x - 1] : 0) + v[x] + (x + 1 < (int)w ? v[x + 1] : 0);
	e1[w] += v[w - 1];
}

/* one Y1 pixel: threshold s, return the error share (an eighth) */
static inline int y1_px(u8 *o, int s, int pix_level, int white)
{
	int dark = s < pix_level;

	*o = dark ? 0 : 0xff;
	return (s - (dark ? 0 : white)) >> 3;
}

struct y1_row {
	u8 *buf;
	int *e0, *e1, *e2;
	int c0, c1, p1, p2;
};

static inline void y1_start(struct y1_row *r, u8 *buf, int *e0, int *e1, int *e2)
{
	r->buf = buf;
	r->e0 = e0;
	r->e1 = e1;
	r->e2 = e2;
	r->c0 = e0[0];
	r->c1 = e0[1];
	r->p1 = r->p2 = 0;
}

static inline void y1_step(struct y1_row *r, u32 x, int pix_level, int white)
{
	int v = y1_px(&r->buf[x], r->c0 + r->buf[x], pix_level, white);

	r->c0 = r->c1 + v;
	r->c1 = r->e0[x + 2] + v;
	r->e2[x] = v;
	r->e1[(int)x - 1] += r->p2 + r->p1 + v;
	r->p2 = r->p1;
	r->p1 = v;
}

static inline void y1_finish(struct y1_row *r, u32 w)
{
	r->e1[w - 1] += r->p2 + r->p1;
}

void epd_dither_y1(u8 *buf, u32 w, u32 h, size_t stride, int *err, int pix_level, bool grey_white)
{
	const int white = grey_white ? 0xf0 : 0xff;
	struct y1_row a, b;
	u32 x, y;

	for (y = 0; y + 1 < h; y += 2, buf += 2 * stride) {
		int *ea = &ERR(y, 0), *eb = &ERR(y + 1, 0), *ec = &ERR(y + 2, 0);

		ea[w] = ea[w + 1] = 0;
		y1_start(&a, buf, ea, eb, ec);
		for (x = 0; x < w; x++) {
			y1_step(&a, x, pix_level, white);
			if (x == 3)
				y1_start(&b, buf + stride, eb, ec, ea);
			if (x >= 3)
				y1_step(&b, x - 3, pix_level, white);
		}
		y1_finish(&a, w);
		eb[w] = eb[w + 1] = 0;
		if (w <= 3)
			y1_start(&b, buf + stride, eb, ec, ea);
		for (x = w > 3 ? w - 3 : 0; x < w; x++)
			y1_step(&b, x, pix_level, white);
		y1_finish(&b, w);
	}
	if (y < h) {
		int *ea = &ERR(y, 0);

		ea[w] = ea[w + 1] = 0;
		y1_start(&a, buf, ea, &ERR(y + 1, 0), &ERR(y + 2, 0));
		for (x = 0; x < w; x++)
			y1_step(&a, x, pix_level, white);
	}
}

void epd_dither_y4(u8 *buf, u32 w, u32 h, size_t stride, int *err)
{
	u32 x, y;

	for (y = 0; y < h; y++, buf += stride) {
		int *e0 = &ERR(y, 0), *e1 = &ERR(y + 1, 0), *e2 = &ERR(y + 2, 0);
		int c0 = e0[0], c1 = w > 1 ? e0[1] : 0;

		for (x = 0; x < w; x++) {
			int b = buf[x], s = c0 + b, c = s < 0 ? 0 : s > 255 ? 255 : s, v;

			v = (b - (c & 0xf0)) >> 3;
			buf[x] = c & 0xf0;
			c0 = c1 + v;
			c1 = (x + 2 < w ? e0[x + 2] : 0) + v;
			e2[x] = v;
		}
		spread_next_row(e1, e2, w);
	}
}

#undef ERR

void epd_dither_y5(u8 *buf, u32 w, u32 h, size_t stride, int *err)
{
	u32 x, y;

	for (y = 0; y < h; y++, buf += stride)
		for (x = 0; x < w; x++) {
			int *e = &err[w * (y & 1) + x];
			int s = *e + buf[x], q = s >> 5, l = q * 36, d = 0;

			if ((u8)(buf[x] + 1) > 1)
				d = (s - l) >> 1;
			buf[x] = l < 0 ? 0 : l > 255 ? 255 : l;
			e[1] += d;
			err[w * ((y + 1) & 1) + x] = d;
		}
}

void epd_dither_flyd(u8 *buf, u32 w, u32 h, size_t stride)
{
	u32 x, y;

	for (y = 0; y < h; y++, buf += stride)
		for (x = 0; x < w; x++) {
			u8 b = buf[x];
			u32 e = b < 0x80 ? (u32)(b & 0xf0) : (u32)(b & 0xf0) - 0xf0;

			buf[x] = b & 0x80 ? 0xf0 : 0;
			if (x != 0 && y != h - 1 && x != w - 1) {
				buf[x + 1] += (u8)(e * 7 >> 4);
				buf[stride + x + 1] += (u8)(e >> 4);
				buf[stride + x] += (u8)(e * 5 >> 4);
				buf[stride + x - 1] += (u8)(e * 3 >> 4);
			}
		}
}

static const u8 bayer8[8][8] = {
	{ 0, 32, 8, 40, 2, 34, 10, 42 },	{ 48, 16, 56, 24, 50, 18, 58, 26 },
	{ 12, 44, 4, 36, 14, 46, 6, 38 },	{ 60, 28, 52, 20, 62, 30, 54, 22 },
	{ 3, 35, 11, 43, 1, 33, 9, 41 },	{ 51, 19, 59, 27, 49, 17, 57, 25 },
	{ 15, 47, 7, 39, 13, 45, 5, 37 },	{ 63, 31, 55, 23, 61, 29, 53, 21 },
};

void epd_dither_y1_ordered(u8 *buf, u32 w, u32 h, size_t stride, u32 x0, u32 y0)
{
	u32 y, i;

	for (y = 0; y < h; y++, buf += stride) {
		u8 t[16];

		for (i = 0; i < 16; i++)
			t[i] = (2 * bayer8[(y0 + y) & 7][(x0 + i) & 7] + 1) * 255 >> 7;
#ifdef EPD_NEON
		epd_threshold16_neon(buf, w, t);
#else
		for (u32 x = 0; x < w; x++)
			buf[x] = buf[x] > t[x & 15] ? 0xff : 0;
#endif
	}
}

int epd_dither(u8 *buf, u32 w, u32 h, size_t stride, u32 flags, int *err)
{
	memset(err, 0, (3 * (size_t)w + 12) * sizeof(*err));
	if (flags & 0x2000)
		epd_dither_y1(buf, w, h, stride, err, 0xff, false);
	else if (flags & 0x4000)
		epd_dither_y4(buf, w, h, stride, err);
	else if (flags & 0x0800)
		epd_dither_flyd(buf, w, h, stride);
	else if (flags & 0x0008)
		epd_dither_y5(buf, w, h, stride, err);
	else
		return 0;
	return 1;
}

void epd_regal_aa_mark(const u8 *levels, size_t stride, u32 w, u32 h, u8 *mark, int shift)
{
	u32 x, y;

	if (w < 2 || h < 2) {
		memset(mark, 0, (size_t)w * h);
		return;
	}
	for (y = 0; y < h; y++) {
		const u8 *r0 = levels + (size_t)(y ? y - 1 : y) * stride;
		const u8 *r1 = levels + (size_t)y * stride;
		const u8 *r2 = levels + (size_t)(y + 1 < h ? y + 1 : y) * stride;
		u8 *m = mark + (size_t)y * w;

		for (x = 0; x < w; x++)
			m[x] = (r0[x] >> shift < 15) | (r1[x] >> shift < 15) | (r2[x] >> shift < 15);
		{
			u8 left = 0, cur;

			for (x = 0; x < w; x++) {
				cur = m[x];
				m[x] = left | cur | (x + 1 < w ? m[x + 1] : 0);
				left = cur;
			}
		}
		for (x = 0; x < w; x++)
			m[x] &= r1[x] >> shift == 15;
	}
}

void epd_convert_update(const u8 *rgba, size_t rgba_stride, u8 *grey, size_t grey_stride, u32 left,
			u32 top, u32 w, u32 h, u32 flags, bool ggg, const u8 *gamma, int *err)
{
	u8 mask = flags & EPD_DITHER_FLAGS ? 0xff : 0xf0;
	const u8 *src = rgba + top * rgba_stride + 4 * (size_t)left;
	u8 *dst = grey + top * grey_stride + left;
	EPD_SIMD_STATE;

#ifdef EPD_NEON
	epd_simd_begin();
	epd_rgba_to_grey_neon(src, rgba_stride, dst, grey_stride, w, h, mask, ggg);
	epd_simd_end();
#else
	epd_rgba_to_grey_scalar(src, rgba_stride, dst, grey_stride, w, h, mask, ggg);
#endif
	if (gamma)
		epd_gamma(dst, grey_stride, w, h, gamma);
	if ((flags & (EPD_CONV_Y1_ORDERED | 0x2000)) == (EPD_CONV_Y1_ORDERED | 0x2000)) {
#ifdef EPD_NEON
		epd_simd_begin();
		epd_dither_y1_ordered(dst, w, h, grey_stride, left, top);
		epd_simd_end();
#else
		epd_dither_y1_ordered(dst, w, h, grey_stride, left, top);
#endif
	} else if (flags & EPD_DITHER_FLAGS) {
		epd_dither(dst, w, h, grey_stride, flags, err);
	}
}
