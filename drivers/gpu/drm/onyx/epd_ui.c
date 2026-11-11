// SPDX-License-Identifier: GPL-2.0-only
/*
 * UI refresh policy.
 */
#include "epd_port.h"
#include "epd_core.h"

/* frames closer than this belong to one burst */
#define UI_BURST_MS		100
/* cooldown before a settle pass */
#define UI_SETTLE_MS		150
/* cooldown before a big burst's GC16 */
#define UI_BIG_QUIET_MS		400
/* a frame after this much quiet counts towards gc_interval */
#define UI_INTERACTION_MS	500
/* bursts longer than this never get GC16 */
#define UI_BIG_MAX_FRAMES	10

#define UI_STOCK_TEMP		20

struct ui_mode {
	const char *name;
	int wf;
	u32 flag;
	bool debounced;
};

static const struct ui_mode ui_modes[EPD_UI_NMODES] = {
	[EPD_UI_SPEED] = { "speed", 4, 0x2000, false },
	[EPD_UI_NORMAL] = { "normal", 255, 0, true },
	[EPD_UI_REGAL] = { "regal", 5, 0x1000, true },
	[EPD_UI_REGALPLUS] = { "regalplus", 16, 0, true },
};

static const struct ui_mode ui_a2 = { "a2", 4, 0x2000, false };

#define UI_GC_WF	2
#define UI_GC_FLAG	0x1000

int epd_ui_mode_by_name(const char *name)
{
	int i;

	for (i = 0; i < EPD_UI_NMODES; i++)
		if (!strcmp(name, ui_modes[i].name))
			return i;
	return EPD_UI_SPEED;
}

const char *epd_ui_mode_name(int mode)
{
	return mode >= 0 && mode < EPD_UI_NMODES ? ui_modes[mode].name : "?";
}

static const struct ui_mode *ui_mode(int mode)
{
	return &ui_modes[mode >= 0 && mode < EPD_UI_NMODES ? mode : EPD_UI_SPEED];
}

int epd_ui_init(struct epd_ui *u, u32 w, u32 h)
{
	memset(u, 0, sizeof(*u));
	if (w < 2 || w % 2 || !h || w > 0xffff)
		return -EINVAL;
	u->w = w;
	u->h = h;
	u->shadow = epd_zalloc((size_t)w / 2 * h * sizeof(u64));
	u->rows = epd_zalloc(h * sizeof(*u->rows));
	u->burst = epd_zalloc(h * sizeof(*u->burst));
	if (!u->shadow || !u->rows || !u->burst) {
		epd_ui_release(u);
		return -ENOMEM;
	}
	u->settle_mode = -1;
	return 0;
}

void epd_ui_release(struct epd_ui *u)
{
	epd_free(u->shadow);
	epd_free(u->rows);
	epd_free(u->burst);
	u->shadow = NULL;
	u->rows = NULL;
	u->burst = NULL;
}

void epd_ui_invalidate(struct epd_ui *u)
{
	u->valid = false;
}

enum ui_diff { UI_UNCHANGED, UI_CHANGED, UI_UNREADABLE };

struct ui_rect {
	u32 x, y, w, h;
};

static enum ui_diff ui_diff(struct epd_ui *u, const u8 *frame, u32 pitch, size_t len, struct ui_rect *r)
{
	u32 words = u->w / 2, y, x0 = 0, x1 = 0, y0 = 0, y1 = 0;
	size_t row_bytes = (size_t)words * 8;
	bool any = false;

	if (pitch < row_bytes || (size_t)pitch * u->h > len)
		return UI_UNREADABLE;
	for (y = 0; y < u->h; y++) {
		const u8 *src = frame + (size_t)y * pitch;
		u64 *shadow = u->shadow + (size_t)y * words;
		u32 a, z, i;

		if (u->valid) {
			u64 v;

			for (a = 0; a < words; a++) {
				memcpy(&v, src + (size_t)a * 8, 8);
				if (v != shadow[a])
					break;
			}
			if (a == words) {
				u->rows[y].end = 0;
				continue;
			}
			for (z = words - 1; z > a; z--) {
				memcpy(&v, src + (size_t)z * 8, 8);
				if (v != shadow[z])
					break;
			}
		} else {
			a = 0;
			z = words - 1;
		}
		for (i = a; i <= z; i++)
			memcpy(&shadow[i], src + (size_t)i * 8, 8);
		u->rows[y].start = a * 2;
		u->rows[y].end = (z + 1) * 2;
		if (!any) {
			x0 = a * 2;
			x1 = (z + 1) * 2;
			y0 = y;
			any = true;
		} else {
			x0 = x0 < a * 2 ? x0 : a * 2;
			x1 = x1 > (z + 1) * 2 ? x1 : (z + 1) * 2;
		}
		y1 = y + 1;
	}
	u->valid = true;
	if (!any)
		return UI_UNCHANGED;
	x0 &= ~7u;
	x1 = (x1 + 7) & ~7u;
	if (x1 > u->w)
		x1 = u->w;
	r->x = x0;
	r->y = y0;
	r->w = x1 - x0;
	r->h = y1 - y0;
	return UI_CHANGED;
}

static void ui_burst_reset(struct epd_ui *u)
{
	memset(u->burst, 0, u->h * sizeof(*u->burst));
	u->burst_frames = 0;
}

static void ui_burst_merge(struct epd_ui *u)
{
	u32 y;

	for (y = 0; y < u->h; y++) {
		const struct epd_ui_span *f = &u->rows[y];
		struct epd_ui_span *b = &u->burst[y];

		if (!f->end)
			continue;
		if (!b->end) {
			*b = *f;
		} else {
			b->start = b->start < f->start ? b->start : f->start;
			b->end = b->end > f->end ? b->end : f->end;
		}
	}
}

static bool ui_burst_add(struct epd_ui *u, bool new_burst, int pct)
{
	u64 area = 0;
	u32 y;

	if (new_burst)
		ui_burst_reset(u);
	ui_burst_merge(u);
	if (pct <= 0)
		return false;
	if (u->burst_frames < 0xffffffffu)
		u->burst_frames++;
	for (y = 0; y < u->h; y++)
		area += u->burst[y].end - u->burst[y].start;
	return u->burst_frames <= UI_BIG_MAX_FRAMES && area * 100 >= (u64)pct * u->w * u->h;
}

enum ui_kind { UI_GC16, UI_TRANSIENT, UI_NORMAL };

struct ui_decision {
	enum ui_kind kind;
	bool mode_changed;
	bool new_burst;
};

static void ui_decide(struct epd_ui *u, int mode, int gc_interval, u64 now, struct ui_decision *d)
{
	bool had = u->have_frame, gc = false;
	u64 since = had && now > u->last_frame ? now - u->last_frame : 0;
#define QUIET_FOR(ms) (!had || since >= (ms))

	u->last_frame = now;
	u->have_frame = true;
	d->mode_changed = !u->have_mode || u->last_mode != mode;
	if (d->mode_changed) {
		gc = u->have_mode;
		u->last_mode = mode;
		u->have_mode = true;
		u->settled = 0;
	}
	if (QUIET_FOR(UI_INTERACTION_MS) && gc_interval > 0) {
		if (u->settled < 0x7fffffff)
			u->settled++;
		gc |= u->settled > gc_interval;
	}
	if (gc) {
		u->settled = 0;
		d->kind = UI_GC16;
	} else if (ui_mode(mode)->debounced && !QUIET_FOR(UI_BURST_MS)) {
		d->kind = UI_TRANSIENT;
	} else {
		d->kind = UI_NORMAL;
	}
	d->new_burst = QUIET_FOR(UI_BIG_QUIET_MS);
#undef QUIET_FOR
}

static int ui_marker(struct epd_ui *u)
{
	u->marker = u->marker == 0x7fffffff ? 1 : u->marker + 1;
	return u->marker;
}

static void ui_upd(struct epd_request *r, u32 x, u32 y, u32 w, u32 h, int wf, u32 flag, bool full, int marker)
{
	r->top = y;
	r->left = x;
	r->width = w;
	r->height = h;
	r->mode = wf;
	r->update_mode = full;
	r->marker = marker;
	r->flags = flag;
}

int epd_ui_frame(struct epd_ui *u, const struct epd_ui_cfg *cfg, const u8 *rgba, u32 pitch, size_t len,
		 u64 now, struct epd_request *out, int *cut)
{
	const struct ui_mode *m = ui_mode(cfg->mode);
	struct ui_decision d;
	struct ui_rect r = { 0, 0, u->w, u->h };
	enum ui_diff diff = ui_diff(u, rgba, pitch, len, &r);
	bool tracked = diff == UI_CHANGED;

	*cut = -1;
	if (diff == UI_UNCHANGED)
		return 0;
	ui_decide(u, cfg->mode, cfg->gc_interval, now, &d);
	if (d.mode_changed)
		*cut = m->debounced ? 0 : cfg->a2cut < 0 ? -1 : cfg->a2cut > 9 ? 9 : cfg->a2cut;

	switch (d.kind) {
	case UI_GC16:
		u->settle_mode = -1;
		u->big_pending = false;
		ui_burst_reset(u);
		if (tracked)
			ui_burst_merge(u);
		ui_upd(out, 0, 0, u->w, u->h, UI_GC_WF, UI_GC_FLAG, true, ui_marker(u));
		break;
	case UI_TRANSIENT:
	case UI_NORMAL:
		if (d.kind == UI_TRANSIENT)
			u->settle_mode = cfg->mode;
		if (tracked)
			u->big_pending = ui_burst_add(u, d.new_burst, cfg->big_pct);
		if (d.kind == UI_TRANSIENT)
			m = &ui_a2;
		ui_upd(out, r.x, r.y, r.w, r.h, m->wf, m->flag, false, ui_marker(u));
		break;
	}
	return 1;
}

enum epd_ui_step epd_ui_step(struct epd_ui *u, u64 now, struct epd_request *out, u64 *wait)
{
	int wf;
	u32 flag;
	u64 quiet;

	if (u->big_pending) {
		wf = UI_GC_WF;
		flag = UI_GC_FLAG;
		quiet = UI_BIG_QUIET_MS;
	} else if (u->settle_mode >= 0) {
		wf = ui_mode(u->settle_mode)->wf;
		flag = ui_mode(u->settle_mode)->flag;
		quiet = UI_SETTLE_MS;
	} else {
		return EPD_UI_IDLE;
	}
	if (u->have_frame && now < u->last_frame + quiet) {
		*wait = u->last_frame + quiet - now;
		return EPD_UI_WAIT;
	}
	u->settle_mode = -1;
	u->big_pending = false;
	ui_upd(out, 0, 0, u->w, u->h, wf, flag, true, ui_marker(u));
	return EPD_UI_SEND;
}
