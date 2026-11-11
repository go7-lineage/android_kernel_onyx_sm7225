/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * E-ink core for the Onyx Boox Go 7.
 */
#ifndef EPD_CORE_H
#define EPD_CORE_H

#include "epd_port.h"

#define EPD_MAX_FRAMES	1024

struct epd_wbf {
	const u8 *data;
	size_t size;
	u8 mode_version;
	u8 frame_rate;
	u8 levels;		/* 16 or 32 */
	u16 n_modes, n_temps;
	u8 end_byte, toggle_byte;
	u8 adv_flags;		/* bit 7 enables the Regal plus fixup */
	u8 temp_bounds[257];
	char name[256];
	u32 *block;
	u16 *nframes;
};

int epd_wbf_parse(struct epd_wbf *w, const u8 *data, size_t size);
void epd_wbf_release(struct epd_wbf *w);
static inline int epd_wbf_frames(const struct epd_wbf *w, int temp, int mode)
{
	return w->nframes[mode * w->n_temps + temp];
}
int epd_wbf_decode(const struct epd_wbf *w, int temp, int mode, u8 *out, size_t cap);

int epd_temp_index(const struct epd_wbf *w, int temp_c);
int epd_wbf_fixup_variant(const struct epd_wbf *w, int slot);
int epd_mode_slot(u8 mode_version, int onyx_mode);

static inline bool epd_mode_is_regal(int onyx_mode)
{
	return onyx_mode == 5 || onyx_mode == 15 || onyx_mode == 16;
}

struct epd_table_cfg {
	u8 cut;
	bool fill_zero;
	u8 a2_clean_mode;
	int a2_slot, du_slot;
};

int epd_tables4(const u8 *src, int nfr, int levels, int slot, const struct epd_table_cfg *c, u8 *full, u8 *part);
void epd_regal_fixup(u8 *frames, int n, int variant);
int epd_mode14_frames(u8 *frames, size_t cap);

struct epd_timing {
	u16 width, height, bitspanel;
	u16 lsl, lbl, lel, ldl;
	u16 fsl, fbl, fel;
	u16 gdck_sta, lgonl;
	u16 width_dummy_ldl, height_dummy;
	u8 sdsp_pulse_enable;
};

extern const struct epd_timing epd_timing_go7;

static inline u32 epd_lt(const struct epd_timing *t)
{
	return t->lsl + t->lbl + t->ldl + t->lel;
}

static inline u32 epd_rows(const struct epd_timing *t)
{
	return t->fsl + t->fbl + t->height + t->fel;
}

static inline size_t epd_frame_words(const struct epd_timing *t)
{
	return (size_t)epd_lt(t) * epd_rows(t);
}

static inline u32 epd_data_start(const struct epd_timing *t)
{
	u32 lt = epd_lt(t);

	return (t->lbl - (t->lbl >> 1)) + lt * (t->fbl + t->fsl + (t->height_dummy >> 1)) +
	       t->lbl + t->lsl + (t->width_dummy_ldl >> 1);
}

void epd_frame_template(const struct epd_timing *t, u32 *buf);
void epd_frame_encode(const struct epd_timing *t, const u32 *tmpl, const u8 *drive, u32 *buf);

#define EPD_NLUTS	127
#define EPD_IDLE	0x7f

struct epd_update {
	u16 top, left, width, height;
	int mode;
	bool full;
	u32 marker;
};

struct epd_engine_cfg {
	const struct epd_timing *timing;
	int temp_c;
	struct epd_table_cfg tables;
	void (*marker_done)(void *ctx, u32 marker);
	void *ctx;
	bool regal_aa;
};

struct epd_engine;
struct epd_engine *epd_engine_create(const struct epd_engine_cfg *cfg, const u8 *wbf, size_t size, int *err);
void epd_engine_destroy(struct epd_engine *e);
int epd_engine_set_temp(struct epd_engine *e, int temp_c);
int epd_engine_set_cut(struct epd_engine *e, int cut);
int epd_engine_get_cut(const struct epd_engine *e);
int epd_engine_submit(struct epd_engine *e, const struct epd_update *u, const u8 *grey8, size_t stride);
bool epd_engine_busy(const struct epd_engine *e);
int epd_engine_tick(struct epd_engine *e, u32 *buf);
void epd_row_scalar(const u8 *state, const u8 *owner, const u8 *const *cur, u32 *out, u32 npix);
void epd_row_neon(const u8 *state, const u8 *owner, const u8 *const *cur, u32 *out, u32 npix);
void epd_rgba_to_grey_scalar(const u8 *src, size_t sstride, u8 *dst, size_t dstride, u32 w, u32 h, u8 mask, bool ggg);
void epd_rgba_to_grey_neon(const u8 *src, unsigned long sstride, u8 *dst, unsigned long dstride, u32 w, u32 h, u8 mask, int ggg);
void epd_regal_aa_mark_neon(const u8 *levels, unsigned long stride, u32 w, u32 h, u8 *mark, int shift);
void epd_gamma(u8 *buf, size_t stride, u32 w, u32 h, const u8 *lut);
void epd_dither_y1(u8 *buf, u32 w, u32 h, size_t stride, int *err, int pix_level, bool grey_white);
void epd_dither_y4(u8 *buf, u32 w, u32 h, size_t stride, int *err);
void epd_dither_y5(u8 *buf, u32 w, u32 h, size_t stride, int *err);
void epd_dither_flyd(u8 *buf, u32 w, u32 h, size_t stride);
int epd_dither(u8 *buf, u32 w, u32 h, size_t stride, u32 flags, int *err);
#define EPD_DITHER_FLAGS	0x6808
#define EPD_CONV_Y1_ORDERED	0x80000000u
void epd_dither_y1_ordered(u8 *buf, u32 w, u32 h, size_t stride, u32 x0, u32 y0);
void epd_threshold16_neon(u8 *buf, u32 w, const u8 *t16);
void epd_regal_aa_mark(const u8 *levels, size_t stride, u32 w, u32 h, u8 *mark, int shift);
void epd_convert_update(const u8 *rgba, size_t rgba_stride, u8 *grey, size_t grey_stride, u32 left,
			u32 top, u32 w, u32 h, u32 flags, bool ggg, const u8 *gamma, int *err);

struct epd_request {
	u16 top, left, width, height;
	int mode;
	int update_mode;
	u32 marker;
	u32 flags;
};

struct epd_decision {
	struct epd_update upd;
	u32 flags;
	bool barrier;
};

struct epd_policy {
	int last_mode;
	u32 last_flags;
	bool du_a2_clean;
};

#define EPD_QMAX	16

struct epd_queue {
	struct epd_decision q[EPD_QMAX];
	int head, n;
	u32 coalesced;
};

void epd_policy_init(struct epd_policy *p, bool du_a2_clean);
void epd_policy_apply(struct epd_policy *p, const struct epd_request *in, struct epd_decision *out);
void epd_queue_init(struct epd_queue *q);
void epd_queue_push(struct epd_queue *q, const struct epd_decision *d);
bool epd_queue_pop(struct epd_queue *q, bool engine_busy, struct epd_decision *out);

int epd_engine_mode_frames(struct epd_engine *e, int mode, bool full);

enum epd_ui_mode { EPD_UI_SPEED, EPD_UI_NORMAL, EPD_UI_REGAL, EPD_UI_REGALPLUS, EPD_UI_NMODES };

struct epd_ui_cfg {
	int mode;
	int gc_interval;
	int big_pct;
	int a2cut;
};

struct epd_ui_span {
	u16 start, end;
};

struct epd_ui {
	u32 w, h;
	u64 *shadow;
	bool valid;
	struct epd_ui_span *rows;
	struct epd_ui_span *burst;
	u32 burst_frames;
	bool have_mode, have_frame;
	int last_mode;
	u64 last_frame;
	int settled;
	int settle_mode;
	bool big_pending;
	int marker;
};

enum epd_ui_step { EPD_UI_IDLE, EPD_UI_WAIT, EPD_UI_SEND };

int epd_ui_init(struct epd_ui *u, u32 w, u32 h);
void epd_ui_release(struct epd_ui *u);
void epd_ui_invalidate(struct epd_ui *u);
int epd_ui_mode_by_name(const char *name);
const char *epd_ui_mode_name(int mode);
int epd_ui_frame(struct epd_ui *u, const struct epd_ui_cfg *cfg, const u8 *rgba, u32 pitch, size_t len,
		 u64 now, struct epd_request *out, int *cut);
enum epd_ui_step epd_ui_step(struct epd_ui *u, u64 now, struct epd_request *out, u64 *wait);

#endif
