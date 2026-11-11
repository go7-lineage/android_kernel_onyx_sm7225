// SPDX-License-Identifier: GPL-2.0-only
#include "epd_core.h"

#define E(new, old)	((new) << 4 | (old))

static void a2_clean_mode2(u8 *full, int n)
{
	static const u8 to_black[] = { 1, 2, 3, 8, 12, 13, 14 };
	static const u8 to_white[] = { 1, 2, 3, 7, 12, 13, 14 };
	int f, i;

	for (f = 0; f < n - 1; f++) {
		for (i = 0; i < sizeof(to_black); i++)
			full[f * 256 + E(0, to_black[i])] = 1;
		for (i = 0; i < sizeof(to_white); i++)
			full[f * 256 + E(15, to_white[i])] = 2;
	}
	for (f = n - 4; f <= n - 2; f++)
		if (f >= 0)
			full[f * 256 + E(15, 8)] = 2;
	for (f = n - 3; f <= n - 2; f++)
		if (f >= 0)
			full[f * 256 + E(0, 7)] = 1;
}

int epd_tables4(const u8 *src, int nfr, int levels, int slot, const struct epd_table_cfg *c,
		u8 *full, u8 *part)
{
	bool cutting = c->cut && (slot == c->a2_slot || slot == c->du_slot);
	int n = nfr, f, nw, old;

	if (levels != 32 || nfr < 1)
		return -EINVAL;
	if (cutting) {
		n -= c->cut;
		if (n < 1)
			n = 1;
	}
	for (f = 0; f < n; f++)
		for (nw = 0; nw < 16; nw++)
			for (old = 0; old < 16; old++)
				full[f * 256 + E(nw, old)] = src[f * 1024 + 2 * nw * 32 + 2 * old];
	if (cutting && c->fill_zero)
		memset(full + (n - 1) * 256, 0, 256);
	memcpy(part, full, (size_t)n * 256);
	for (f = 0; f < n; f++)
		for (nw = 0; nw < 16; nw++)
			part[f * 256 + E(nw, nw)] = 0;
	if (slot == c->a2_slot && c->a2_clean_mode == 2 && n > 1)
		a2_clean_mode2(full, n);
	else if (slot == c->a2_slot && c->a2_clean_mode == 1)
		return -EOPNOTSUPP;
	return n;
}

void epd_regal_fixup(u8 *f, int n, int variant)
{
	int i, k;

	for (i = 0; i < n; i++, f += 1024) {
		f[0x3de] = 0;		/* 30 <- 30 */
		f[0x3df] = f[0x3ff];	/* 30 <- 31 = 31 <- 31 */
		f[0x3dd] = f[0x3bd];	/* 30 <- 29 = 29 <- 29 */
		if (variant != 2)
			f[0] = 0;	/* 0 <- 0 */
		f[1] = f[0x20];		/* 0 <- 1 = 1 <- 0 */
		f[3] = f[0x60];		/* 0 <- 3 = 3 <- 0 */
		if (variant == 1)
			for (k = 0; k < 32; k++)
				f[k * 33] = 0;
	}
}

int epd_mode14_frames(u8 *f, size_t cap)
{
	static const struct { u8 n, drive; } seq[] = { { 7, 1 }, { 7, 2 }, { 7, 1 }, { 8, 2 }, { 1, 0 } };
	int i, k, n = 0;

	if (cap < 30 * 1024)
		return -ENOSPC;
	for (i = 0; i < 5; i++)
		for (k = 0; k < seq[i].n; k++, n++)
			memset(f + n * 1024, seq[i].drive, 1024);
	return n;
}
