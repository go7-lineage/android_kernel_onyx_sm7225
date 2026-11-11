// SPDX-License-Identifier: GPL-2.0-only
/*
 * Update policy and queue between userspace requests and engine. 
 */
#include "epd_core.h"

#define M_DU	1
#define M_GC16	2
#define M_A2	4
#define M_AUTO	0xff

void epd_policy_init(struct epd_policy *p, bool du_a2_clean)
{
	memset(p, 0, sizeof(*p));
	p->last_mode = -1;
	p->du_a2_clean = du_a2_clean;
}

static bool is_regal(int m)
{
	return epd_mode_is_regal(m);
}

void epd_policy_apply(struct epd_policy *p, const struct epd_request *in, struct epd_decision *out)
{
	int mode = in->mode, last = p->last_mode;
	bool full = in->update_mode == 1, barrier;

	barrier = (last == M_DU && mode != M_DU) ||
		  (last == M_GC16 && !(p->last_flags & 8) && (mode == M_DU || mode == M_A2)) ||
		  (mode == M_GC16 && !(in->flags & 8) && (last == M_DU || last == M_A2)) ||
		  (p->du_a2_clean && mode != M_A2 && last == M_A2);
	if (mode == M_AUTO)
		mode = M_GC16;

	if (mode == M_A2 || (in->flags & 0x8000))
		full = true;
	if ((last == M_A2 && mode != M_A2) || (is_regal(last) && !is_regal(mode)) ||
	    (last == M_DU && mode != M_DU))
		full = true;
	if (p->du_a2_clean && mode == M_A2 && last != M_A2 && last != -1) {
		mode = M_DU;
		full = true;
	}

	out->upd.top = in->top;
	out->upd.left = in->left;
	out->upd.width = in->width;
	out->upd.height = in->height;
	out->upd.mode = mode;
	out->upd.full = full;
	out->upd.marker = in->marker;
	out->flags = in->flags;
	out->barrier = barrier;

	p->last_mode = in->mode == M_AUTO ? M_GC16 : in->mode;
	p->last_flags = in->flags;
}

void epd_queue_init(struct epd_queue *q)
{
	memset(q, 0, sizeof(*q));
}

void epd_queue_push(struct epd_queue *q, const struct epd_decision *d)
{
	struct epd_decision *t;
	u32 r, b;

	if (q->n < EPD_QMAX) {
		q->q[(q->head + q->n++) % EPD_QMAX] = *d;
		return;
	}
	t = &q->q[(q->head + q->n - 1) % EPD_QMAX];
	r = t->upd.left + t->upd.width > d->upd.left + d->upd.width ?
		t->upd.left + t->upd.width : d->upd.left + d->upd.width;
	b = t->upd.top + t->upd.height > d->upd.top + d->upd.height ?
		t->upd.top + t->upd.height : d->upd.top + d->upd.height;
	t->upd.left = t->upd.left < d->upd.left ? t->upd.left : d->upd.left;
	t->upd.top = t->upd.top < d->upd.top ? t->upd.top : d->upd.top;
	t->upd.width = r - t->upd.left;
	t->upd.height = b - t->upd.top;
	t->upd.mode = d->upd.mode;
	t->upd.full |= d->upd.full;
	t->upd.marker = d->upd.marker;
	t->flags = d->flags;
	t->barrier |= d->barrier;
	q->coalesced++;
}

bool epd_queue_pop(struct epd_queue *q, bool engine_busy, struct epd_decision *out)
{
	struct epd_decision *h = &q->q[q->head];

	if (!q->n || (h->barrier && engine_busy))
		return false;
	*out = *h;
	q->head = (q->head + 1) % EPD_QMAX;
	q->n--;
	return true;
}
