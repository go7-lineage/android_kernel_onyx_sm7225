// SPDX-License-Identifier: GPL-2.0-only
/*
 * Update engine (stock: onyx_epdc_update_wb_sg, mergeWbBySg*Update_*).
 */
#include "epd_core.h"

struct epd_table {
	u8 *t;		/* n x 256, indexed by state */
	u16 n;
	int users;	/* active LUTs */
	bool retired;
};

struct epd_lut {
	bool active;
	u32 marker;
	struct epd_table *table;
	u16 frame;
	u16 y0, y1, x0, x1;
	u32 seq;
};

struct epd_req {
	struct epd_req *next;
	struct epd_update upd;
	u8 *target;		/* width x height, 4-bit target grey */
	u8 *mask;		/* 1 = still to merge */
	u16 y0, y1, x0, x1;	/* part of the rect that still has mask set */
	u64 wait[2];		/* LUTs this request waits for */
	bool wait_any;		/* ran out of LUTs, retry when any finish */
};

struct epd_engine {
	struct epd_engine_cfg cfg;
	struct epd_wbf wf;
	int t;
	u16 w, h;
	u8 *state;
	u8 *owner;
	u8 *mark;		/* Regal AA scratch */
	struct epd_lut lut[EPD_NLUTS];
	u32 seq;
	int active;
	struct epd_req *wait_head, **wait_tail;
	struct epd_table *mode14[2];	/* partial, full */
	struct epd_table **cache;	/* see cache_slot() */
	u8 zero[256];
	const u8 *cur[EPD_IDLE + 1];
	int finished[EPD_NLUTS];
};

static bool lut_bit(const u64 *m, int i)
{
	return m[i >> 6] >> (i & 63) & 1;
}

#define CACHE_VARIANTS	8

static struct epd_table **cache_slot(struct epd_engine *e, int t, int slot, bool full, bool regal, bool aa)
{
	return &e->cache[(t * e->wf.n_modes + slot) * CACHE_VARIANTS + full * 4 + regal * 2 + aa];
}

static void table_free(struct epd_table *t)
{
	epd_free(t->t);
	epd_free(t);
}

static void cache_retire(struct epd_table **slot)
{
	struct epd_table *t = *slot;

	if (!t)
		return;
	*slot = NULL;
	if (t->users)
		t->retired = true;
	else
		table_free(t);
}

static void table_put(struct epd_table *t)
{
	if (--t->users == 0 && t->retired)
		table_free(t);
}

static int build_table(struct epd_engine *e, struct epd_table **slotp, int slot, bool full,
		       bool regal, bool aa)
{
	struct epd_table *ct;
	bool mode14 = slot < 0;
	int nfr = mode14 ? 30 : epd_wbf_frames(&e->wf, e->t, slot), n, f, nw, old, fix;
	u8 *src, *p4 = NULL;

	if (nfr < 1)
		return -EINVAL;
	ct = epd_zalloc(sizeof(*ct));
	if (!ct)
		return -ENOMEM;
	src = epd_zalloc((size_t)nfr * 1024);
	ct->t = epd_zalloc((size_t)nfr * 256);
	if (!src || !ct->t) {
		n = -ENOMEM;
		goto out;
	}
	if (mode14) {
		n = epd_mode14_frames(src, (size_t)nfr * 1024);
	} else {
		n = epd_wbf_decode(&e->wf, e->t, slot, src, (size_t)nfr * 1024);
		fix = epd_wbf_fixup_variant(&e->wf, slot);
		if (n > 0 && fix >= 0)
			epd_regal_fixup(src, n, fix);
	}
	if (n < 0)
		goto out;
	if (regal) {
		for (f = 0; f < n; f++)
			for (nw = 0; nw < 16; nw++)
				for (old = 0; old < 16; old++) {
					bool edge = aa && old == 15;
					int o5 = edge ? 31 : 2 * old;

					ct->t[f * 256 + (nw << 4 | old)] = full || nw != old || edge ?
						src[f * 1024 + 2 * nw * 32 + o5] : 0;
				}
	} else {
		p4 = epd_zalloc((size_t)nfr * 256);
		if (!p4) {
			n = -ENOMEM;
			goto out;
		}
		n = epd_tables4(src, nfr, e->wf.levels, slot, &e->cfg.tables, ct->t, p4);
		if (n > 0 && !full)
			memcpy(ct->t, p4, (size_t)n * 256);
	}
	if (n > 0)
		ct->n = n;
out:
	if (n < 0)
		table_free(ct);
	else
		*slotp = ct;
	epd_free(p4);
	epd_free(src);
	return n < 0 ? n : 0;
}

static int get_table(struct epd_engine *e, int mode, bool full, bool aa, struct epd_table **out)
{
	int slot = epd_mode_slot(e->wf.mode_version, mode), ret;
	bool regal = epd_mode_is_regal(mode);
	struct epd_table **ct;

	if (mode == 14) {
		ct = &e->mode14[full];
		slot = -1;
		regal = aa = false;
	} else if (slot < 0 || slot >= e->wf.n_modes) {
		return -EINVAL;
	} else {
		aa &= regal;
		ct = cache_slot(e, e->t, slot, full, regal, aa);
	}
	if (!*ct) {
		ret = build_table(e, ct, slot, full, regal, aa);
		if (ret)
			return ret;
	}
	*out = *ct;
	return 0;
}

static void req_free(struct epd_req *r)
{
	epd_free(r->target);
	epd_free(r->mask);
	epd_free(r);
}

static void marker_complete(struct epd_engine *e, u32 marker)
{
	struct epd_req *r;
	int i;

	for (i = 0; i < EPD_NLUTS; i++)
		if (e->lut[i].active && e->lut[i].marker == marker)
			return;
	for (r = e->wait_head; r; r = r->next)
		if (r->upd.marker == marker)
			return;
	if (e->cfg.marker_done)
		e->cfg.marker_done(e->cfg.ctx, marker);
}

static void wait_append(struct epd_engine *e, struct epd_req *r)
{
	r->next = NULL;
	*e->wait_tail = r;
	e->wait_tail = &r->next;
}

struct claim {
	int lut;
	int n;
	u16 y0, y1, x0, x1;
};

static int free_lut(const struct epd_engine *e, int skip)
{
	int i;

	for (i = 0; i < EPD_NLUTS; i++)
		if (!e->lut[i].active && i != skip)
			return i;
	return -1;
}

static void claim_add(struct claim *c, u32 x, u32 y)
{
	c->n++;
	c->y0 = y < c->y0 ? y : c->y0;
	c->y1 = y + 1;
	c->x0 = x < c->x0 ? x : c->x0;
	c->x1 = x + 1 > c->x1 ? x + 1 : c->x1;
}

static void activate(struct epd_engine *e, const struct claim *c, struct epd_table *table,
		     const struct epd_update *u)
{
	struct epd_lut *l = &e->lut[c->lut];

	l->active = true;
	l->marker = u->marker;
	l->table = table;
	table->users++;
	l->frame = 0;
	l->y0 = u->top + c->y0;
	l->y1 = u->top + c->y1;
	l->x0 = u->left + c->x0;
	l->x1 = u->left + c->x1;
	l->seq = e->seq++;
	e->active++;
}

static int merge(struct epd_engine *e, struct epd_req *r)
{
	const struct epd_update *u = &r->upd;
	struct epd_table *table, *aa_table = NULL;
	struct claim c = { .y0 = 0xffff, .x0 = 0xffff }, ca = { .lut = -1, .y0 = 0xffff, .x0 = 0xffff };
	bool aa = e->cfg.regal_aa && epd_mode_is_regal(u->mode);
	u16 by0 = 0xffff, by1 = 0, bx0 = 0xffff, bx1 = 0;
	int busy = 0, ret;
	u32 x, y;

	ret = get_table(e, u->mode, u->full, false, &table);
	if (!ret && aa)
		ret = get_table(e, u->mode, u->full, true, &aa_table);
	if (ret) {
		req_free(r);
		return ret;
	}
	if (aa) {
		const u8 *lv = e->state + (size_t)u->top * e->w + u->left;
		EPD_SIMD_STATE;

#ifdef EPD_NEON
		if (u->width >= 2 && u->height >= 2) {
			epd_simd_begin();
			epd_regal_aa_mark_neon(lv, e->w, u->width, u->height, e->mark, 4);
			epd_simd_end();
		} else
#endif
			epd_regal_aa_mark(lv, e->w, u->width, u->height, e->mark, 4);
	}
	c.lut = free_lut(e, -1);
	r->wait[0] = r->wait[1] = 0;
	r->wait_any = false;

	for (y = r->y0; y < r->y1; y++) {
		size_t row = (size_t)(u->top + y) * e->w + u->left;
		size_t rrow = (size_t)y * u->width;

		for (x = r->x0; x < r->x1; x++) {
			size_t p = row + x;
			bool edge = aa && e->mark[rrow + x];
			struct claim *cl = edge ? &ca : &c;
			u8 o, tgt;

			if (!r->mask[rrow + x])
				continue;
			o = e->owner[p];
			tgt = r->target[rrow + x];
			if (o == EPD_IDLE && !u->full && !edge && (e->state[p] >> 4) == tgt) {
				r->mask[rrow + x] = 0;	/* partial: target unchanged */
				continue;
			}
			if (o == EPD_IDLE && edge && ca.lut < 0)
				ca.lut = free_lut(e, c.lut);
			if (o != EPD_IDLE || cl->lut < 0) {
				if (o != EPD_IDLE)
					r->wait[o >> 6] |= 1ull << (o & 63);
				else
					r->wait_any = true;
				busy++;
				by0 = y < by0 ? y : by0;
				by1 = y + 1;
				bx0 = x < bx0 ? x : bx0;
				bx1 = x + 1 > bx1 ? x + 1 : bx1;
				continue;
			}
			e->state[p] = tgt << 4 | e->state[p] >> 4;
			e->owner[p] = cl->lut;
			r->mask[rrow + x] = 0;
			claim_add(cl, x, y);
		}
	}

	if (c.n)
		activate(e, &c, table, u);
	if (ca.n)
		activate(e, &ca, aa_table, u);
	if (busy) {
		r->y0 = by0;
		r->y1 = by1;
		r->x0 = bx0;
		r->x1 = bx1;
		wait_append(e, r);
	} else {
		u32 marker = u->marker;

		req_free(r);
		if (!c.n && !ca.n)
			marker_complete(e, marker);
	}
	return 0;
}

int epd_engine_submit(struct epd_engine *e, const struct epd_update *u, const u8 *grey8,
		      size_t stride)
{
	struct epd_table *table;
	struct epd_req *r;
	size_t n;
	u32 x, y;
	int ret;

	if ((u32)u->top + u->height > e->h || (u32)u->left + u->width > e->w)
		return -EINVAL;
	ret = get_table(e, u->mode, u->full, false, &table);
	if (ret)
		return ret;
	n = (size_t)u->width * u->height;
	r = epd_zalloc(sizeof(*r));
	if (!r)
		return -ENOMEM;
	r->upd = *u;
	r->target = epd_zalloc(n);
	r->mask = epd_zalloc(n);
	if (!r->target || !r->mask) {
		req_free(r);
		return -ENOMEM;
	}
	for (y = 0; y < u->height; y++) {
		const u8 *src = grey8 + (size_t)(u->top + y) * stride + u->left;

		for (x = 0; x < u->width; x++)
			r->target[(size_t)y * u->width + x] = src[x] >> 4;
	}
	memset(r->mask, 1, n);
	r->y1 = u->height;
	r->x1 = u->width;
	return merge(e, r);
}

bool epd_engine_busy(const struct epd_engine *e)
{
	return e->active || e->wait_head;
}

static void release_owner(struct epd_engine *e, const struct epd_lut *l, u8 id)
{
	u32 y, x, n = l->x1 - l->x0;

	for (y = l->y0; y < l->y1; y++) {
		u8 *o = e->owner + (size_t)y * e->w + l->x0;

		for (x = 0; x < n; x++)
			o[x] = o[x] == id ? EPD_IDLE : o[x];
	}
}

static void clear_words(u32 *row, u32 from, u32 to)
{
	for (; from < to; from++)
		row[from] &= ~0xffffu;
}

void epd_row_scalar(const u8 *state, const u8 *owner, const u8 *const *cur, u32 *out, u32 npix)
{
	u32 x, k;

	for (x = 0; x < npix; x += 8) {
		u32 w = 0;

		for (k = 0; k < 8; k++)
			w |= (u32)cur[owner[x + k]][state[x + k]] << (14 - 2 * k);
		out[x / 8] = (out[x / 8] & ~0xffffu) | w;
	}
}

#ifdef EPD_NEON
#define epd_row epd_row_neon
#else
#define epd_row epd_row_scalar
#endif

int epd_engine_tick(struct epd_engine *e, u32 *buf)
{
	const struct epd_timing *t = e->cfg.timing;
	u32 lt = epd_lt(t), start = epd_data_start(t), y;
	const u8 **cur = e->cur;
	int *finished = e->finished;
	u32 y0 = 0xffff, y1 = 0, x0 = 0xffff, x1 = 0;
	int nfin = 0, i, j, ret = 0;
	struct epd_req *ready = NULL, **ready_tail = &ready, **pp, *r;
	EPD_SIMD_STATE;

	if (!epd_engine_busy(e))
		return 0;

	for (i = 0; i <= EPD_IDLE; i++)
		cur[i] = e->zero;
	for (i = 0; i < EPD_NLUTS; i++) {
		struct epd_lut *l = &e->lut[i];

		if (!l->active)
			continue;
		cur[i] = l->table->t + (size_t)l->frame * 256;
		y0 = l->y0 < y0 ? l->y0 : y0;
		y1 = l->y1 > y1 ? l->y1 : y1;
		x0 = l->x0 < x0 ? l->x0 : x0;
		x1 = l->x1 > x1 ? l->x1 : x1;
	}
	x0 &= ~15u;
	x1 = (x1 + 15) & ~15u;
	if (x1 > e->w)
		x1 = e->w;
	epd_simd_begin();
	for (y = 0; y < e->h; y++) {
		u32 *row = buf + start + (size_t)y * lt;
		size_t p = (size_t)y * e->w;

		if (y < y0 || y >= y1) {
			clear_words(row, 0, e->w / 8u);
			continue;
		}
		clear_words(row, 0, x0 / 8);
		epd_row(e->state + p + x0, e->owner + p + x0, cur, row + x0 / 8, x1 - x0);
		clear_words(row, x1 / 8, e->w / 8u);
		if ((y & 63) == 63) {
			epd_simd_end();
			epd_simd_begin();
		}
	}
	epd_simd_end();

	for (i = 0; i < EPD_NLUTS; i++) {
		struct epd_lut *l = &e->lut[i];

		if (l->active && ++l->frame == l->table->n)
			finished[nfin++] = i;
	}
	for (i = 1; i < nfin; i++)
		for (j = i; j > 0 && e->lut[finished[j]].seq < e->lut[finished[j - 1]].seq; j--) {
			int tmp = finished[j];

			finished[j] = finished[j - 1];
			finished[j - 1] = tmp;
		}
	for (i = 0; i < nfin; i++) {
		struct epd_lut *l = &e->lut[finished[i]];

		release_owner(e, l, finished[i]);
		table_put(l->table);
		l->active = false;
		e->active--;
		marker_complete(e, l->marker);
	}
	if (!nfin)
		return 1;

	for (pp = &e->wait_head; (r = *pp);) {
		bool wait = false;

		for (i = 0; i < EPD_NLUTS && !wait; i++)
			wait = lut_bit(r->wait, i) && e->lut[i].active;
		if (wait) {
			pp = &r->next;
			continue;
		}
		*pp = r->next;
		r->next = NULL;
		*ready_tail = r;
		ready_tail = &r->next;
	}
	e->wait_tail = pp;
	while (ready) {
		r = ready;
		ready = r->next;
		i = merge(e, r);
		if (i && !ret)
			ret = i;
	}
	return ret ? ret : 1;
}

int epd_engine_mode_frames(struct epd_engine *e, int mode, bool full)
{
	struct epd_table *table;
	int ret = get_table(e, mode, full, false, &table);

	return ret ? ret : table->n;
}

int epd_engine_set_temp(struct epd_engine *e, int temp_c)
{
	int t = epd_temp_index(&e->wf, temp_c);
	size_t per_t = (size_t)e->wf.n_modes * CACHE_VARIANTS, i;

	if (t != e->t) {
		for (i = 0; i < (size_t)e->wf.n_temps * per_t; i++)
			if (i / per_t == (size_t)e->t)
				cache_retire(&e->cache[i]);
		e->t = t;
	}
	return 0;
}

int epd_engine_set_cut(struct epd_engine *e, int cut)
{
	int slots[2] = { e->cfg.tables.a2_slot, e->cfg.tables.du_slot }, t, k, v;

	if (cut < 0 || cut > 255)
		return -EINVAL;
	e->cfg.tables.cut = cut;
	for (t = 0; t < e->wf.n_temps; t++)
		for (k = 0; k < 2; k++)
			if (slots[k] >= 0 && slots[k] < e->wf.n_modes)
				for (v = 0; v < CACHE_VARIANTS; v++)
					cache_retire(&e->cache[(t * e->wf.n_modes + slots[k]) * CACHE_VARIANTS + v]);
	return 0;
}

int epd_engine_get_cut(const struct epd_engine *e)
{
	return e->cfg.tables.cut;
}

struct epd_engine *epd_engine_create(const struct epd_engine_cfg *cfg, const u8 *wbf, size_t size,
				     int *err)
{
	struct epd_engine *e;
	size_t px;
	int ret;

	e = epd_zalloc(sizeof(*e));
	if (!e) {
		*err = -ENOMEM;
		return NULL;
	}
	e->cfg = *cfg;
	e->wait_tail = &e->wait_head;
	ret = epd_wbf_parse(&e->wf, wbf, size);
	if (ret)
		goto err_free;
	ret = -EINVAL;
	if (e->wf.levels != 32 || cfg->timing->width % 8)
		goto err_wbf;
	e->cfg.tables.a2_slot = epd_mode_slot(e->wf.mode_version, 4);
	e->cfg.tables.du_slot = epd_mode_slot(e->wf.mode_version, 1);
	e->t = epd_temp_index(&e->wf, cfg->temp_c);
	e->w = cfg->timing->width;
	e->h = cfg->timing->height;
	px = (size_t)e->w * e->h;
	ret = -ENOMEM;
	e->state = epd_zalloc(px);
	e->owner = epd_zalloc(px);
	e->cache = epd_zalloc(sizeof(*e->cache) * e->wf.n_temps * e->wf.n_modes * CACHE_VARIANTS);
	e->mark = epd_zalloc(px);
	if (!e->state || !e->owner || !e->cache || !e->mark)
		goto err_mem;
	memset(e->owner, EPD_IDLE, px);
	return e;

err_mem:
	epd_free(e->state);
	epd_free(e->owner);
	epd_free(e->mark);
	epd_free(e->cache);
err_wbf:
	epd_wbf_release(&e->wf);
err_free:
	epd_free(e);
	*err = ret;
	return NULL;
}

void epd_engine_destroy(struct epd_engine *e)
{
	struct epd_req *r;
	size_t k, n;
	int i;

	if (!e)
		return;
	while ((r = e->wait_head)) {
		e->wait_head = r->next;
		req_free(r);
	}
	for (i = 0; i < EPD_NLUTS; i++)
		if (e->lut[i].active)
			table_put(e->lut[i].table);
	n = (size_t)e->wf.n_temps * e->wf.n_modes * CACHE_VARIANTS;
	for (k = 0; k < n; k++)
		cache_retire(&e->cache[k]);
	cache_retire(&e->mode14[0]);
	cache_retire(&e->mode14[1]);
	epd_free(e->cache);
	epd_free(e->state);
	epd_free(e->owner);
	epd_free(e->mark);
	epd_wbf_release(&e->wf);
	epd_free(e);
}
