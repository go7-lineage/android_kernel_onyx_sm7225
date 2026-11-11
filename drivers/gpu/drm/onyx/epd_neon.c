// SPDX-License-Identifier: GPL-2.0-only
/*
 * NEON kernels
 */
#include <arm_neon.h>

#define EPD_IDLE	0x7f

/* same as in epd_core.h */
void epd_row_neon(const uint8_t *state, const uint8_t *owner, const uint8_t *const *cur, uint32_t *out, uint32_t npix);
void epd_rgba_to_grey_neon(const uint8_t *src, unsigned long sstride, uint8_t *dst, unsigned long dstride,
			   uint32_t w, uint32_t h, uint8_t mask, int ggg);
void epd_regal_aa_mark_neon(const uint8_t *levels, unsigned long stride, uint32_t w, uint32_t h, uint8_t *mark, int shift);
void epd_threshold16_neon(uint8_t *buf, uint32_t w, const uint8_t *t16);
void epd_threshold16_neon(uint8_t *buf, uint32_t w, const uint8_t *t16)
{
	uint8x16_t t = vld1q_u8(t16);
	uint32_t x;

	for (x = 0; x + 16 <= w; x += 16)
		vst1q_u8(buf + x, vcgtq_u8(vld1q_u8(buf + x), t));
	for (; x < w; x++)
		buf[x] = buf[x] > t16[x & 15] ? 0xff : 0;
}

static inline __attribute__((always_inline)) void
row_scalar(const uint8_t *state, const uint8_t *owner, const uint8_t *const *cur, uint32_t *out,
	   uint32_t npix)
{
	uint32_t x, k;

	for (x = 0; x < npix; x += 8) {
		uint32_t w = 0;

		for (k = 0; k < 8; k++)
			w |= (uint32_t)cur[owner[x + k]][state[x + k]] << (14 - 2 * k);
		out[x / 8] = (out[x / 8] & ~0xffffu) | w;
	}
}

static inline __attribute__((always_inline)) int uniform16(const uint8_t *owner, uint8_t o)
{
	return vminvq_u8(vceqq_u8(vld1q_u8(owner), vdupq_n_u8(o))) == 0xff;
}

static inline __attribute__((always_inline)) uint32x2_t pack16(uint8x16_t r)
{
	uint16x8_t l = vreinterpretq_u16_u8(r);
	uint16x4_t l2;
	uint8x8_t b;

	b = vmovn_u16(vorrq_u16(vshlq_n_u16(l, 2), vshrq_n_u16(l, 8)));	/* d(2k) << 2 | d(2k+1) */
	l2 = vreinterpret_u16_u8(b);
	l2 = vorr_u16(vshl_n_u16(l2, 4), vshr_n_u16(l2, 8));	/* 4 pixels per byte */
	b = vrev16_u8(vmovn_u16(vcombine_u16(l2, l2)));		/* q0 << 8 | q1, .. */
	return vget_low_u32(vmovl_u16(vreinterpret_u16_u8(b)));
}

void epd_row_neon(const uint8_t *state, const uint8_t *owner, const uint8_t *const *cur,
		  uint32_t *out, uint32_t npix)
{
	const uint8_t *zero = cur[EPD_IDLE];
	const uint32x2_t ctl = vdup_n_u32(0xffff0000u);
	uint32_t x = 0, xe;

	while (x + 16 <= npix) {
		const uint8_t o = owner[x];
		const uint8_t *tab = cur[o];

		for (xe = x; xe + 16 <= npix && uniform16(owner + xe, o); xe += 16)
			;
		if (xe == x) {
			row_scalar(state + x, owner + x, cur, out + x / 8, 16);
			x += 16;
			continue;
		}
		if (tab == zero) {
			for (; x < xe; x += 16)
				vst1_u32(out + x / 8, vand_u32(vld1_u32(out + x / 8), ctl));
			continue;
		}
		{
			const uint8x16x4_t t0 = vld1q_u8_x4(tab), t1 = vld1q_u8_x4(tab + 64);
			const uint8x16x4_t t2 = vld1q_u8_x4(tab + 128), t3 = vld1q_u8_x4(tab + 192);
			const uint8x16_t k64 = vdupq_n_u8(64);

			for (; x + 32 <= xe; x += 32) {
				uint8x16_t s = vld1q_u8(state + x), u = vld1q_u8(state + x + 16), r, q;

				r = vqtbl4q_u8(t0, s);
				q = vqtbl4q_u8(t0, u);
				s = vsubq_u8(s, k64);
				u = vsubq_u8(u, k64);
				r = vqtbx4q_u8(r, t1, s);
				q = vqtbx4q_u8(q, t1, u);
				s = vsubq_u8(s, k64);
				u = vsubq_u8(u, k64);
				r = vqtbx4q_u8(r, t2, s);
				q = vqtbx4q_u8(q, t2, u);
				s = vsubq_u8(s, k64);
				u = vsubq_u8(u, k64);
				r = vqtbx4q_u8(r, t3, s);
				q = vqtbx4q_u8(q, t3, u);
				vst1q_u32(out + x / 8, vorrq_u32(vandq_u32(vld1q_u32(out + x / 8), vcombine_u32(ctl, ctl)),
								 vcombine_u32(pack16(r), pack16(q))));
			}
			for (; x < xe; x += 16) {
				uint8x16_t s = vld1q_u8(state + x), r;

				r = vqtbl4q_u8(t0, s);
				s = vsubq_u8(s, k64);
				r = vqtbx4q_u8(r, t1, s);
				s = vsubq_u8(s, k64);
				r = vqtbx4q_u8(r, t2, s);
				s = vsubq_u8(s, k64);
				r = vqtbx4q_u8(r, t3, s);
				vst1_u32(out + x / 8, vorr_u32(vand_u32(vld1_u32(out + x / 8), ctl), pack16(r)));
			}
		}
	}
	if (x < npix)
		row_scalar(state + x, owner + x, cur, out + x / 8, npix - x);
}

void epd_rgba_to_grey_neon(const uint8_t *src, unsigned long sstride, uint8_t *dst, unsigned long dstride,
			   uint32_t w, uint32_t h, uint8_t mask, int ggg)
{
	const uint8x8_t kr = vdup_n_u8(77), kg = vdup_n_u8(151), kb = vdup_n_u8(28);
	const uint8x16_t m = vdupq_n_u8(mask);
	uint32_t x, y;

	for (y = 0; y < h; y++, src += sstride, dst += dstride) {
		for (x = 0; x + 16 <= w; x += 16) {
			uint8x16x4_t p = vld4q_u8(src + 4 * x);
			uint8x16_t g;

			if (ggg) {
				g = p.val[0];
			} else {
				uint16x8_t lo = vmull_u8(vget_low_u8(p.val[0]), kr);
				uint16x8_t hi = vmull_u8(vget_high_u8(p.val[0]), kr);

				lo = vmlal_u8(lo, vget_low_u8(p.val[1]), kg);
				hi = vmlal_u8(hi, vget_high_u8(p.val[1]), kg);
				lo = vmlal_u8(lo, vget_low_u8(p.val[2]), kb);
				hi = vmlal_u8(hi, vget_high_u8(p.val[2]), kb);
				g = vcombine_u8(vshrn_n_u16(lo, 8), vshrn_n_u16(hi, 8));
			}
			vst1q_u8(dst + x, vandq_u8(g, m));
		}
		for (; x < w; x++) {
			const uint8_t *q = src + 4 * x;

			dst[x] = (ggg ? q[0] : (77 * q[0] + 151 * q[1] + 28 * q[2]) >> 8) & mask;
		}
	}
}

static inline __attribute__((always_inline)) uint8x16_t load_pad(const uint8_t *p, uint32_t n, uint8_t fill)
{
	uint8_t t[16];
	uint32_t i;

	if (n >= 16)
		return vld1q_u8(p);
	for (i = 0; i < 16; i++)
		t[i] = i < n ? p[i] : fill;
	return vld1q_u8(t);
}

void epd_regal_aa_mark_neon(const uint8_t *levels, unsigned long stride, uint32_t w, uint32_t h, uint8_t *mark,
			    int shift)
{
	const int8x16_t sh = vdupq_n_s8(-shift);
	const uint8x16_t k15 = vdupq_n_u8(15), one = vdupq_n_u8(1), zero = vdupq_n_u8(0);
	const uint8_t fill = 15 << shift;
	uint32_t x, y;

	for (y = 0; y < h; y++) {
		const uint8_t *r0 = levels + (unsigned long)(y ? y - 1 : y) * stride;
		const uint8_t *r1 = levels + (unsigned long)y * stride;
		const uint8_t *r2 = levels + (unsigned long)(y + 1 < h ? y + 1 : y) * stride;
		uint8_t *m = mark + (unsigned long)y * w;
		uint8x16_t prev = zero, cur, next;

#define AA_DARK(x, n)	vorrq_u8(vorrq_u8( \
			vcltq_u8(vshlq_u8(load_pad(r0 + (x), (n), fill), sh), k15), \
			vcltq_u8(vshlq_u8(load_pad(r1 + (x), (n), fill), sh), k15)), \
			vcltq_u8(vshlq_u8(load_pad(r2 + (x), (n), fill), sh), k15))
		cur = AA_DARK(0, w);
		for (x = 0; x < w; x += 16) {
			uint32_t n = w - x;
			uint8x16_t r;

			next = x + 16 < w ? AA_DARK(x + 16, w - x - 16) : zero;
			r = vorrq_u8(vorrq_u8(vextq_u8(prev, cur, 15), cur), vextq_u8(cur, next, 1));
			r = vandq_u8(vandq_u8(r, vceqq_u8(vshlq_u8(load_pad(r1 + x, n, fill), sh), k15)), one);
			if (n >= 16) {
				vst1q_u8(m + x, r);
			} else {
				uint8_t t[16];
				uint32_t i;

				vst1q_u8(t, r);
				for (i = 0; i < n; i++)
					m[x + i] = t[i];
			}
			prev = cur;
			cur = next;
		}
#undef AA_DARK
	}
}
