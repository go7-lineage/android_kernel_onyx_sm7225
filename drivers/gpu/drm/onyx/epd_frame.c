// SPDX-License-Identifier: GPL-2.0-only
/*
 * Go 7 e-ink DSI frame (E1).
 */
#include "epd_core.h"

#define GDCLK	(1u << 17)
#define GDSP	(1u << 18)
#define SDSP	(1u << 19)
#define SDOE	(1u << 20)
#define GDOE	(1u << 21)
#define SDLE	(1u << 22)

const struct epd_timing epd_timing_go7 = {
	.width = 1264, .height = 1680, .bitspanel = 16,
	.lsl = 15, .lbl = 10, .lel = 30, .ldl = 158,
	.fsl = 1, .fbl = 4, .fel = 14,
};

struct tbuf {
	u32 *b;
	long size;
};

static long orr(struct tbuf *t, long start, long n, u32 bit)
{
	long i;

	if (n <= 0)
		return start;
	for (i = start; i < start + n && i < t->size; i++)
		t->b[i] |= bit;
	return start + n;
}

void epd_frame_template(const struct epd_timing *t, u32 *buf)
{
	long lt = epd_lt(t), low, high, after_low, half, shift, p, r;
	struct tbuf tb = { buf, (long)epd_frame_words(t) };

	memset(buf, 0, tb.size * sizeof(*buf));
	if (t->lgonl == 0 && t->gdck_sta == 0) {
		low = lt >> 3;
		high = lt - low;
	} else {
		low = (t->lsl - (t->lbl >> 1)) + t->lbl * 2 + t->gdck_sta;
		high = t->lgonl;
	}
	after_low = lt - low;

	for (r = 0; r < t->fsl + t->fbl + t->height; r++)
		orr(&tb, r * lt + low, high, GDCLK);

	half = high >> 1;
	p = orr(&tb, 0, half + low, GDSP);
	p = orr(&tb, p, t->fsl * lt, GDSP);
	p += t->fsl * lt;
	orr(&tb, p, after_low - half - 1 + ((long)t->fbl - t->fsl - 1 + t->height + t->fel) * lt, GDSP);

	orr(&tb, t->fsl * lt, ((long)t->height + t->fbl) * lt, GDOE);
	orr(&tb, t->fsl * lt, ((long)t->fbl + t->height) * lt, SDOE);

	shift = t->lbl - (t->lbl >> 1);
	p = shift + ((long)t->fbl + t->fsl) * lt;
	for (r = 0; r < t->height; r++) {
		p = orr(&tb, p, t->lsl, SDLE);
		p += t->ldl + t->lbl + t->lel;
	}

	p = orr(&tb, 0, shift, SDSP);
	p = orr(&tb, p, ((long)t->fbl + t->fsl) * lt, SDSP);
	for (r = 0; r < t->height; r++) {
		p = orr(&tb, p, t->lbl + t->lsl, SDSP);
		if (!t->sdsp_pulse_enable)
			p += t->ldl;
		else
			p = orr(&tb, p + 1, t->ldl - 1, SDSP);
		p = orr(&tb, p, t->lel, SDSP);
	}
	orr(&tb, p, ((long)t->fel - 1) * lt + (t->lbl >> 1) + lt - t->lbl - 1, SDSP);
}

void epd_frame_encode(const struct epd_timing *t, const u32 *tmpl, const u8 *drive, u32 *buf)
{
	u32 lt = epd_lt(t), start = epd_data_start(t), x, y, k, w;
	size_t n = epd_frame_words(t);

	memcpy(buf, tmpl, n * sizeof(*buf));
	for (y = 0; y < t->height; y++) {
		const u8 *d = drive + (size_t)y * t->width;
		u32 *row = buf + start + (size_t)y * lt;

		for (x = 0; x < t->width / 8u; x++) {
			for (w = 0, k = 0; k < 8; k++)
				w |= (u32)(d[8 * x + k] & 3) << (14 - 2 * k);
			row[x] |= w;
		}
	}
}
