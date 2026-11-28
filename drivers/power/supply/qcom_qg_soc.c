// SPDX-License-Identifier: GPL-2.0-only

#include <linux/minmax.h>
#include <linux/string.h>

#include "qcom_qg_profile.h"
#include "qcom_qg_soc.h"

enum qg_param {
	QG_PARAM_ESR,
	QG_PARAM_CSHORT,
	QG_PARAM_RSHORT,
	QG_PARAM_RLONG,
	QG_PARAM_CLONG,
	QG_PARAM_RBAT,
};

static const struct {
	enum qg_table table[2];
	int offset[2];
	int divisor[2];
} qg_params[] = {
	[QG_PARAM_ESR] = { { QG_TABLE_Y1, QG_TABLE_Z1 }, { 3724, 8924 }, { 19, 35 } },
	[QG_PARAM_CSHORT] = { { QG_TABLE_Y2, QG_TABLE_Z2 }, { 9643, 8936 }, { 1, 1 } },
	[QG_PARAM_RSHORT] = { { QG_TABLE_Y3, QG_TABLE_Z3 }, { 13258, 19256 }, { 3, 12 } },
	[QG_PARAM_RLONG] = { { QG_TABLE_Y4, QG_TABLE_Z4 }, { 16395, 14578 }, { 7, 14 } },
	[QG_PARAM_CLONG] = { { QG_TABLE_Y5, QG_TABLE_Z5 }, { 8712, 9581 }, { 1, 1 } },
	[QG_PARAM_RBAT] = { { QG_TABLE_Y6, QG_TABLE_Z6 }, { 4825, 14027 }, { 2, 6 } },
};

static double qg_exp(double x)
{
	static const double ln2_hi = 6.93147180369123816490e-01;
	static const double ln2_lo = 1.90821492927058770002e-10;
	static const double inv_ln2 = 1.44269504088896338700e+00;
	static const double c[] = {
		1.0 / 6227020800.0, 1.0 / 479001600.0, 1.0 / 39916800.0, 1.0 / 3628800.0,
		1.0 / 362880.0, 1.0 / 40320.0, 1.0 / 5040.0, 1.0 / 720.0, 1.0 / 120.0,
		1.0 / 24.0, 1.0 / 6.0, 1.0 / 2.0, 1.0, 1.0,
	};
	union {
		double d;
		u64 u;
	} scale;
	double r, p;
	unsigned int i;
	int k;

	if (x < -700.0)
		return 0.0;
	if (x > 700.0)
		x = 700.0;

	k = (int)(x * inv_ln2 + (x < 0.0 ? -0.5 : 0.5));
	r = (x - (double)k * ln2_hi) - (double)k * ln2_lo;

	p = c[0];
	for (i = 1; i < ARRAY_SIZE(c); i++)
		p = p * r + c[i];

	scale.u = (u64)(k + 1023) << 52;

	return p * scale.d;
}

static float qg_clampf(float x, float lo, float hi)
{
	if (x < lo)
		return lo;
	if (x > hi)
		return hi;

	return x;
}

static int qg_soc_param(const struct qg_soc *s, enum qg_param param, int soc, int temp,
			bool charging)
{
	int raw = qg_profile_var(s->profile, qg_params[param].table[charging], temp, soc);

	return clamp((raw - qg_params[param].offset[charging]) / qg_params[param].divisor[charging],
		     1, 65535);
}

static int qg_soc_sys(const struct qg_soc_vars *v)
{
	int span = v->full_soc - v->cutoff_soc;

	if (!span)
		return 0;

	return max(v->b_soc - v->cutoff_soc, 0) * 10000 / span;
}

static float qg_soc_vbat(const struct qg_soc_vars *v, float fifo_v)
{
	if (fifo_v < 2.0f)
		return 2.0f;

	return fifo_v <= v->v_cap_f ? fifo_v : v->v_cap_f;
}

static float qg_soc_branch(float branch, float current, float r, float c, int ms)
{
	float e = (float)qg_exp((double)((float)ms / -1000.0f) / (double)(r * c));

	return branch + (float)((1.0 - (double)e) * (double)(current - branch));
}

static void qg_soc_init(struct qg_soc *s, int ocv_uv, int temp)
{
	struct qg_soc_vars *v = &s->v;
	int soc;

	ocv_uv = clamp(ocv_uv, 2000000, 5000000);
	soc = qg_profile_soc(s->profile, QG_TABLE_OCV2, temp, ocv_uv);

	memset(v, 0, sizeof(*v));
	v->init_soc = (float)soc;
	v->voltage_soc = soc;
	v->cc_soc = soc;
	v->cc_soc_sw = soc;
	v->cc_soc_sw_f = (float)soc;
	v->cutoff_soc = s->coff_soc;
	v->cutoff_soc_f = (float)s->coff_soc;
	v->full_soc = s->fullsoc;
	v->full_soc_f = (float)s->fullsoc;
	v->v_ocv = (float)ocv_uv / 1e6f;
	v->temp = temp;
}

void qg_soc_start(struct qg_soc *s, const struct qg_profile *profile, const struct qg_soc_cfg *cfg,
		  int capacity, int temp)
{
	int soc;

	s->profile = profile;
	s->cfg = *cfg;
	s->fifo_delta_ms = 0;
	s->charge_done_seen = false;

	s->coff_soc = 200;
	if (cfg->vcutoff_uv > 0)
		s->coff_soc = qg_profile_soc(profile, QG_TABLE_OCV2, temp, cfg->vcutoff_uv);
	if (temp < 100)
		s->coff_soc = 500;

	s->fullsoc = 9900;
	if (cfg->vfloat_uv > 0)
		s->fullsoc = min(qg_profile_soc(profile, QG_TABLE_OCV1, temp, cfg->vfloat_uv), 9900);

	soc = (capacity * (s->fullsoc - s->coff_soc) + 50) / 100 + s->coff_soc;
	qg_soc_init(s, qg_profile_ocv(profile, QG_TABLE_OCV2, temp, soc), temp);
}

static void qg_soc_good_ocv(struct qg_soc *s, int ocv_uv, int temp)
{
	struct qg_soc_vars *v = &s->v;
	int cutoff_soc = v->cutoff_soc, full_soc = v->full_soc, v_cap = v->v_cap;
	int cc_soc_sw = v->cc_soc_sw;

	qg_soc_init(s, ocv_uv, temp);

	if (cutoff_soc) {
		v->cutoff_soc = cutoff_soc;
		v->cutoff_soc_f = (float)cutoff_soc;
	}
	if (full_soc) {
		v->full_soc = full_soc;
		v->full_soc_f = (float)full_soc;
	}
	if (v_cap) {
		v->v_cap = v_cap;
		v->v_cap_f = (float)v_cap / 1e6f;
	}
	if (cc_soc_sw) {
		v->cc_soc_sw = cc_soc_sw;
		v->cc_soc_sw_f = (float)cc_soc_sw;
	}

	v->r_soc = clamp((v->voltage_soc * 50 + v->cc_soc * 50) / 100, 0, 10000);
	v->b_soc = v->r_soc;
	v->b_ocv = qg_profile_ocv(s->profile, QG_TABLE_OCV2, temp, v->b_soc);
	v->s_soc = clamp(qg_soc_sys(v), 0, 10000);
}

static void qg_soc_sample(struct qg_soc *s, const struct qg_soc_in *in,
			  const struct qg_sample *sample)
{
	const struct qg_profile *p = s->profile;
	struct qg_soc_vars *v = &s->v;
	int temp = in->temp, i_ua = sample->i_ua;
	int ms = sample->interval_ms * sample->count + s->fifo_delta_ms;
	bool charging = i_ua < -5000, cold = temp < 0;
	enum qg_table ocv_table = charging ? QG_TABLE_OCV1 : QG_TABLE_OCV2;
	float fifo_i = (float)i_ua / 1e6f;
	float fifo_v = (float)(int)sample->v_uv / 1e6f;
	float iterm = (float)s->cfg.iterm_ua / 1e6f;
	float seconds = (float)ms / 1000.0f;
	float fcc_ah, vbat_100, below, top, slope, rbat, step, diff, ibat, charge, soc;
	float rshort, cshort, rlong, clong, esr, extra, predicted, error, gain;
	int fcc, b_soc, cutoff_ma;

	v->fifo_i = i_ua;
	v->temp = temp;
	if (!sample->v_uv)
		return;

	fcc = qg_profile_fcc(p, charging ? QG_TABLE_FCC1 : QG_TABLE_FCC2, temp);
	v->fcc = fcc;
	fcc_ah = (float)fcc / 1000.0f;

	rbat = (float)qg_soc_param(s, QG_PARAM_RBAT, 10000, temp, true) / 1000.0f;
	vbat_100 = (float)qg_profile_ocv(p, QG_TABLE_OCV1, temp, 10000) / 1e6f - rbat * iterm;
	below = (float)((double)vbat_100 + -0.005);
	if (!v->v_cap) {
		v->v_cap_f = below;
		v->v_cap = (int)(below * 1e6f);
	}
	if (iterm > (float)i_ua) {
		top = (float)s->cfg.vfloat_uv / 1e6f;
		if (!(top > fifo_v))
			top = fifo_v;
		v->v_cap_f = (double)(vbat_100 - top) < 0.005 ? below : top;
		v->v_cap = (int)(v->v_cap_f * 1e6f);
	}

	slope = (float)qg_profile_slope(p, QG_TABLE_OCV2, temp, v->voltage_soc) / 10000.0f;
	v->t_rbat = qg_soc_param(s, QG_PARAM_RBAT, v->voltage_soc, temp, charging);
	rbat = (float)v->t_rbat / 1000.0f;
	step = (seconds * slope) / (fcc_ah * 3600.0f);
	diff = v->v_ocv - qg_soc_vbat(v, fifo_v);
	ibat = (diff + diff) / ((rbat + rbat) + step);
	v->v_ocv = v->v_ocv - step * ibat;
	v->voltage_soc = qg_profile_soc(p, QG_TABLE_OCV2, temp, (int)(v->v_ocv * 1e6f));

	charge = (seconds * (float)i_ua) / 3600.0f;
	v->cc_uah = qg_clampf(v->cc_uah + charge, (float)(fcc * -2000), (float)(fcc * 2000));
	v->cc_soc_sw_f = qg_clampf(v->cc_soc_sw_f - (charge * 10.0f) / (float)fcc, 0.0f, 10000.0f);
	v->cc_soc_sw = (int)v->cc_soc_sw_f;
	soc = qg_clampf(v->init_soc - (v->cc_uah * 10.0f) / (float)fcc, -10000.0f, 20000.0f);
	v->cc_soc = (int)soc;

	v->r_soc = clamp((clamp(v->voltage_soc, 0, 10000) * 50 +
			  clamp(v->cc_soc, -1500, 11500) * 50) / 100, -1500, 11500);

	b_soc = clamp(v->r_soc + v->soc_corr, 0, 10000);
	rshort = (float)qg_soc_param(s, QG_PARAM_RSHORT, b_soc, temp, charging) / 1000.0f;
	rlong = (float)qg_soc_param(s, QG_PARAM_RLONG, b_soc, temp, charging) / 1000.0f;
	clong = (float)qg_soc_param(s, QG_PARAM_CLONG, b_soc, temp, charging);
	cshort = (float)qg_soc_param(s, QG_PARAM_CSHORT, b_soc, temp, charging);
	esr = (float)qg_soc_param(s, QG_PARAM_ESR, b_soc, temp, charging) / 1000.0f;
	v->t_rbat = qg_soc_param(s, QG_PARAM_RBAT, b_soc, temp, charging);
	rbat = (float)v->t_rbat / 1000.0f;

	v->i_rshort = qg_soc_branch(v->i_rshort, fifo_i, rshort, cshort, ms);
	v->i_rlong = qg_soc_branch(v->i_rlong, fifo_i, rlong, clong, ms);
	extra = rbat - ((rlong + rshort) + esr);
	if (!(extra > 0.0f))
		extra = 0.0f;
	predicted = (float)qg_profile_ocv(p, ocv_table, temp, b_soc) / 1e6f;
	predicted = ((predicted - fifo_i * (esr + extra)) - rlong * v->i_rlong) -
		    rshort * v->i_rshort;
	error = qg_soc_vbat(v, fifo_v) - predicted;
	v->soc_corr_f = v->soc_corr_f + (0.35f * error) * 100.0f;
	v->soc_corr = (int)v->soc_corr_f;
	v->b_soc = (int)qg_clampf(v->soc_corr_f + (float)v->r_soc, 0.0f, 10000.0f);

	if (!in->charge_done) {
		s->charge_done_seen = false;
	} else if (!s->charge_done_seen) {
		v->cc_soc_sw = v->b_soc;
		v->cc_soc_sw_f = (float)v->b_soc;
		s->charge_done_seen = true;
	}

	v->b_ocv = qg_profile_ocv(p, ocv_table, temp, v->b_soc);

	cutoff_ma = max(cold ? 500 : 250, i_ua / 1000);
	rbat = (float)qg_soc_param(s, QG_PARAM_RBAT, v->cutoff_soc, temp, false) / 1000.0f;
	predicted = (float)qg_profile_ocv(p, QG_TABLE_OCV2, temp, v->cutoff_soc) / 1e6f -
		    rbat * ((float)(cutoff_ma * 1000) / 1e6f);
	error = (float)s->cfg.vcutoff_uv / 1e6f - predicted;
	gain = cold ? 10.0f : 2.0f;
	v->cutoff_soc_f = qg_clampf(v->cutoff_soc_f + (gain * error) * 100.0f, 0.0f, 10000.0f);
	v->cutoff_soc = (int)v->cutoff_soc_f;

	rbat = (float)qg_soc_param(s, QG_PARAM_RBAT, v->full_soc, temp, true) / 1000.0f;
	predicted = (float)qg_profile_ocv(p, QG_TABLE_OCV1, temp, v->full_soc) / 1e6f - rbat * iterm;
	gain = temp >= 0 && i_ua < 0 ? 2.0f : 0.0f;
	v->full_soc_f = qg_clampf(v->full_soc_f + (gain * (v->v_cap_f - predicted)) * 100.0f,
				  200.0f, 10000.0f);
	v->full_soc = (int)v->full_soc_f;
	if (i_ua < 0 && v->s_soc > 9999) {
		v->full_soc = v->b_soc;
		v->full_soc_f = (float)v->b_soc;
	}
	if (v->full_soc <= v->cutoff_soc) {
		v->cutoff_soc = v->full_soc - 100;
		v->cutoff_soc_f = (float)v->cutoff_soc;
	}

	v->s_soc = qg_soc_sys(v);
}

bool qg_soc_event(struct qg_soc *s, const struct qg_soc_in *in, const struct qg_sample *fifo,
		  unsigned int n, int time_delta_ms, int good_ocv_uv)
{
	unsigned int i;
	int nominal, margin;

	s->coff_soc = 200;
	s->fullsoc = 9900;
	s->v.temp = in->temp;
	s->fifo_delta_ms = 0;
	if (time_delta_ms >= 0 && n) {
		nominal = n * fifo[0].interval_ms * fifo[0].count;
		margin = nominal * 5 / 100;
		s->fifo_delta_ms = clamp(time_delta_ms - nominal, -margin, margin) / (int)n;
	}

	if (good_ocv_uv > 0)
		qg_soc_good_ocv(s, good_ocv_uv, in->temp);

	if (n) {
		s->v.fifo_length = n;
		for (i = 0; i < n; i++)
			qg_soc_sample(s, in, &fifo[i]);
	}

	return n || good_ocv_uv > 0;
}

int qg_soc_report(const struct qg_soc *s, const struct qg_soc_in *in)
{
	const struct qg_soc_vars *v = &s->v;
	int soc = v->s_soc + 50;

	soc = soc >= 10100 ? 100 : soc > -100 ? soc / 100 : 0;

	if (!soc) {
		if (in->vbat_uv > 0 && s->cfg.vcutoff_uv > 0)
			soc = in->vbat_uv >= s->cfg.vcutoff_uv + 20000;
	} else if (soc > 98 && v->fifo_length && v->fifo_i < 0) {
		soc = v->fifo_i >= s->cfg.iterm_ua - 30000 ? 100 : 99;
	}

	return soc;
}

int qg_soc_charge_counter(const struct qg_soc *s)
{
	return s->v.fcc * ((s->v.cc_soc_sw + 50) / 100) * 10;
}
