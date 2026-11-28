/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __QCOM_QG_SOC_H__
#define __QCOM_QG_SOC_H__

#include <linux/types.h>

struct qg_profile;

struct qg_soc_cfg {
	int vcutoff_uv;
	int vfloat_uv;
	int iterm_ua;
};

struct qg_soc_in {
	int temp;
	int vbat_uv;
	bool charge_done;
};

struct qg_sample {
	u32 v_uv;
	s32 i_ua;
	u32 count;
	u32 interval_ms;
};

struct qg_soc_vars {
	float init_soc;
	float v_ocv;
	float cc_uah;
	float cc_soc_sw_f;
	float soc_corr_f;
	float i_rshort;
	float i_rlong;
	float cutoff_soc_f;
	float full_soc_f;
	float v_cap_f;
	int cc_soc_sw;
	int soc_corr;
	int cutoff_soc;
	int full_soc;
	int v_cap;
	int voltage_soc;
	int cc_soc;
	int r_soc;
	int b_soc;
	int s_soc;
	int fifo_i;
	int fifo_length;
	int b_ocv;
	int t_rbat;
	int fcc;
	int temp;
};

struct qg_soc {
	const struct qg_profile *profile;
	struct qg_soc_cfg cfg;
	int coff_soc;
	int fullsoc;
	int fifo_delta_ms;
	bool charge_done_seen;
	struct qg_soc_vars v;
};

void qg_soc_start(struct qg_soc *s, const struct qg_profile *profile, const struct qg_soc_cfg *cfg,
		  int capacity, int temp);
bool qg_soc_event(struct qg_soc *s, const struct qg_soc_in *in, const struct qg_sample *fifo,
		  unsigned int n, int time_delta_ms, int good_ocv_uv);
int qg_soc_report(const struct qg_soc *s, const struct qg_soc_in *in);
int qg_soc_charge_counter(const struct qg_soc *s);

#endif
