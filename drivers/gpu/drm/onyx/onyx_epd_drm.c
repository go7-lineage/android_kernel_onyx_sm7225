// SPDX-License-Identifier: GPL-2.0-only
/*
 * Onyx Boox Go 7 e-ink display.
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/regulator/consumer.h>
#include <linux/sched/topology.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_client.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_edid.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_property.h>
#include <drm/drm_vblank.h>
#include <drm/onyx/msm_dsi_client.h>
#include <linux/onyx_epd.h>

#include "epd_core.h"

#define EPD_W		1264
#define EPD_H		1680
#define EPD_NBUF	3

static char *waveform = "eink_waveform.wbf";
module_param(waveform, charp, 0444);
MODULE_PARM_DESC(waveform, "waveform file");
static unsigned int idle_ms = 500;
module_param(idle_ms, uint, 0644);
static unsigned int link_idle_ms = 2000;
module_param(link_idle_ms, uint, 0644);
MODULE_PARM_DESC(link_idle_ms, "switch DSI link off after this long without frames");
static int temp_c = 24;
module_param(temp_c, int, 0644);
static bool swap_rg = true;
module_param(swap_rg, bool, 0644);
MODULE_PARM_DESC(swap_rg, "swap the two data bytes per word");
static bool hv = true;
module_param(hv, bool, 0644);
MODULE_PARM_DESC(hv, "switch the panel rails on for updates");
static bool y1_ordered;
module_param(y1_ordered, bool, 0644);
MODULE_PARM_DESC(y1_ordered, "Y1 (speed mode 0x2000) as ordered dither instead of stock Atkinson");
static bool ui = true;
module_param(ui, bool, 0644);
MODULE_PARM_DESC(ui, "refresh policy for commits without EPD_UPDATES");

struct onyx_plane_state {
	struct drm_shadow_plane_state base;
	struct drm_property_blob *updates;
};

static struct onyx_plane_state *to_onyx_plane_state(struct drm_plane_state *s)
{
	return container_of(to_drm_shadow_plane_state(s), struct onyx_plane_state, base);
}

struct onyx_epd {
	struct drm_device drm;
	struct drm_plane plane;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_connector connector;
	struct drm_property *updates_prop;

	struct mutex lock;
	struct epd_engine *eng;
	const struct firmware *fw;
	struct epd_policy pol;
	struct epd_queue q;
	u8 *grey;
	int *err;
	u32 cut;

	struct epd_ui ui;
	bool ui_ok;
	struct epd_ui_cfg ui_cfg;
	struct delayed_work ui_work;
	u64 ui_frames, ui_updates, ui_passes;
	u32 ui_us_last, ui_us_max;

	struct epd_stat { u32 last, max; u64 sum, n; } st_tick, st_copy, st_commit, st_link, st_power, st_first;
	u64 t_start;
	int tick_cpu;

	struct device_node *dsi_np;
	struct drm_client_dev client;
	bool client_ok;
	struct mutex link_lock;
	bool link_on;
	struct drm_client_buffer *buf[EPD_NBUF];
	void *buf_vaddr[EPD_NBUF];
	u32 *frame;

	struct task_struct *out_thread;
	wait_queue_head_t out_wq;
	spinlock_t out_lock;
	int shown, inflight, pending, filling;
	bool link_req;
	u32 *tmpl;

	struct task_struct *thread;
	wait_queue_head_t wq;
	bool kick;

	struct regulator *display, *vcom, *v3p3;
	bool powered;
	struct delayed_work power_off;
	struct delayed_work link_off;
	struct delayed_work setup;
	int tries;
	u64 frames;

	struct miscdevice misc;
	struct class *cls;
	struct device *sysdev;
	struct wakeup_source *ws;
};

static struct onyx_epd *to_epd(struct drm_device *drm)
{
	return container_of(drm, struct onyx_epd, drm);
}

static void epd_stat(struct epd_stat *s, u64 t0)
{
	u32 us = div_u64(ktime_get_ns() - t0, 1000);

	s->last = us;
	s->max = max(s->max, us);
	s->sum += us;
	s->n++;
}

static void epd_power_up(struct onyx_epd *p)
{
	u64 t0 = ktime_get_ns();

	cancel_delayed_work(&p->power_off);
	if (p->powered || !hv)
		return;
	__pm_stay_awake(p->ws);
	if (p->v3p3 && regulator_enable(p->v3p3))
		drm_warn(&p->drm, "v3p3 on failed\n");
	if (p->display && regulator_enable(p->display))
		drm_warn(&p->drm, "display on failed\n");
	if (p->vcom && regulator_enable(p->vcom))
		drm_warn(&p->drm, "vcom on failed\n");
	p->powered = true;
	epd_stat(&p->st_power, t0);
}

static void epd_power_down(struct onyx_epd *p)
{
	if (!p->powered)
		return;
	if (p->vcom)
		regulator_disable(p->vcom);
	if (p->display)
		regulator_disable(p->display);
	if (p->v3p3)
		regulator_disable(p->v3p3);
	p->powered = false;
	__pm_relax(p->ws);
}

static void epd_link_off(struct onyx_epd *p)
{
	mutex_lock(&p->link_lock);
	if (p->client_ok && p->link_on) {
		drm_client_modeset_dpms(&p->client, DRM_MODE_DPMS_OFF);
		WRITE_ONCE(p->link_on, false);
	}
	mutex_unlock(&p->link_lock);
}

static bool epd_out_idle(struct onyx_epd *p)
{
	bool idle;

	spin_lock(&p->out_lock);
	idle = p->pending < 0 && p->inflight < 0;
	spin_unlock(&p->out_lock);
	return idle;
}

static void epd_power_off_work(struct work_struct *w)
{
	struct onyx_epd *p = container_of(to_delayed_work(w), struct onyx_epd, power_off);
	bool idle;

	mutex_lock(&p->lock);
	idle = (!p->eng || !epd_engine_busy(p->eng)) && epd_out_idle(p);
	if (idle)
		epd_power_down(p);
	else if (!p->eng || !epd_engine_busy(p->eng))	/* last frames still going out */
		schedule_delayed_work(&p->power_off, msecs_to_jiffies(idle_ms));
	mutex_unlock(&p->lock);
}

static void epd_link_off_work(struct work_struct *w)
{
	struct onyx_epd *p = container_of(to_delayed_work(w), struct onyx_epd, link_off);
	bool idle;

	mutex_lock(&p->lock);
	idle = (!p->eng || !epd_engine_busy(p->eng)) && !p->q.n && epd_out_idle(p);
	mutex_unlock(&p->lock);
	if (idle)
		epd_link_off(p);
	else
		mod_delayed_work(system_wq, &p->link_off, msecs_to_jiffies(link_idle_ms));
}

static void epd_link_hint(struct onyx_epd *p)
{
	if (!p->client_ok || READ_ONCE(p->link_on))
		return;
	spin_lock(&p->out_lock);
	p->link_req = true;
	spin_unlock(&p->out_lock);
	wake_up(&p->out_wq);
}

static void epd_set_vcom(struct onyx_epd *p)
{
	const char *args, *s;
	char num[16];
	long uv;
	int i;

	if (!p->vcom || !of_chosen || of_property_read_string(of_chosen, "bootargs", &args))
		return;
	s = strstr(args, "fp9931:vcom=");
	if (!s)
		return;
	s += strlen("fp9931:vcom=");
	for (i = 0; i < sizeof(num) - 1 && s[i] && s[i] != ' '; i++)
		num[i] = s[i];
	num[i] = 0;
	if (kstrtol(num, 10, &uv))
		return;
	uv = abs(uv);
	if (regulator_set_voltage(p->vcom, uv - (long)regulator_get_linear_step(p->vcom) + 1, uv))
		drm_warn(&p->drm, "can't set VCOM %ld uV\n", uv);
	else
		drm_info(&p->drm, "VCOM %ld uV from the bootloader\n", uv);
}

/* ---------------------------------------------------------------- requests */

static bool epd_upd_to_req(const struct onyx_epdc_upd *u, struct epd_request *r)
{
	s32 top = max(u->top, 0), left = max(u->left, 0);
	s32 w = min_t(s32, u->width, EPD_W - left), h = min_t(s32, u->height, EPD_H - top);

	if ((u32)u->flag & (7u << 16) || w <= 0 || h <= 0)
		return false;
	r->top = top;
	r->left = left;
	r->width = w;
	r->height = h;
	r->mode = u->waveform_mode;
	r->update_mode = u->update_mode;
	r->marker = u->update_marker;
	r->flags = u->flag;
	return true;
}

static void epd_submit(struct onyx_epd *p, const struct epd_request *r, const u8 *rgba, u32 pitch)
{
	struct epd_decision d;

	epd_policy_apply(&p->pol, r, &d);
	epd_convert_update(rgba, pitch, p->grey, EPD_W, r->left, r->top, r->width, r->height,
			   d.flags | (y1_ordered ? EPD_CONV_Y1_ORDERED : 0), false, NULL, p->err);
	epd_queue_push(&p->q, &d);
	p->kick = true;
}

static void epd_request(struct onyx_epd *p, const struct onyx_epdc_upd *u, const u8 *rgba, u32 pitch)
{
	struct epd_request r;

	if (epd_upd_to_req(u, &r)) {
		epd_link_hint(p);
		epd_submit(p, &r, rgba, pitch);
	}
}

static u64 epd_now_ms(void)
{
	return ktime_to_ms(ktime_get());
}

static void epd_set_cut(struct onyx_epd *p, u32 cut)
{
	p->cut = cut;
	if (p->eng)
		epd_engine_set_cut(p->eng, cut);
}

static bool epd_out_can_queue(struct onyx_epd *p)
{
	bool ok;

	spin_lock(&p->out_lock);
	ok = p->pending < 0;
	spin_unlock(&p->out_lock);
	return ok || kthread_should_stop();
}

static void epd_copy_frame(struct onyx_epd *p, int f, const u32 *src)
{
	const struct epd_timing *t = &epd_timing_go7;
	u32 lt = epd_lt(t), rows = epd_rows(t), y, x;
	struct drm_client_buffer *b = p->buf[f];
	u8 *dst = p->buf_vaddr[f];
	u64 t0 = ktime_get_ns();

	for (y = 0; y < rows; y++) {
		u32 *d = (u32 *)(dst + (size_t)y * b->pitch);
		const u32 *s = src + (size_t)y * lt;

		if (!swap_rg) {
			memcpy(d, s, lt * 4);
			continue;
		}
		for (x = 0; x < lt; x++)
			d[x] = (s[x] & 0xffff0000u) | (s[x] & 0xff) << 8 | (s[x] >> 8 & 0xff);
	}
	epd_stat(&p->st_copy, t0);
}

static int epd_free_buf(struct onyx_epd *p)
{
	int f;

	for (f = 0; f < EPD_NBUF; f++)
		if (f != p->shown && f != p->inflight && f != p->pending && f != p->filling)
			return f;
	return -1;
}

static void epd_queue_frame(struct onyx_epd *p)
{
	int f;

	wait_event(p->out_wq, epd_out_can_queue(p));
	if (kthread_should_stop())
		return;
	spin_lock(&p->out_lock);
	f = epd_free_buf(p);
	p->filling = f;
	spin_unlock(&p->out_lock);
	if (WARN_ON_ONCE(f < 0))
		return;
	epd_copy_frame(p, f, p->frame);
	spin_lock(&p->out_lock);
	p->filling = -1;
	p->pending = f;
	spin_unlock(&p->out_lock);
	wake_up(&p->out_wq);
}

static bool epd_out_has_work(struct onyx_epd *p)
{
	bool work;

	spin_lock(&p->out_lock);
	work = p->pending >= 0 || (p->link_req && p->inflight < 0);
	spin_unlock(&p->out_lock);
	return work || kthread_should_stop();
}

static int epd_out_thread(void *data)
{
	struct onyx_epd *p = data;

	while (!kthread_should_stop()) {
		struct drm_mode_set *ms;
		u64 t0, t_start;
		bool up, null = false;
		int c;

		wait_event_interruptible(p->out_wq, epd_out_has_work(p));
		spin_lock(&p->out_lock);
		c = p->pending;
		if (c >= 0) {
			p->pending = -1;
		} else if (p->link_req) {
			p->link_req = false;
			if (!READ_ONCE(p->link_on)) {
				c = epd_free_buf(p);
				null = c >= 0;
			}
		}
		p->inflight = c;
		spin_unlock(&p->out_lock);
		if (c < 0)
			continue;
		if (null)
			epd_copy_frame(p, c, p->tmpl);
		else
			wake_up(&p->out_wq);

		mutex_lock(&p->client.modeset_mutex);
		drm_client_for_each_modeset(ms, &p->client)
			if (ms->mode)
				ms->fb = p->buf[c]->fb;
		mutex_unlock(&p->client.modeset_mutex);
		mutex_lock(&p->link_lock);
		t0 = ktime_get_ns();
		up = !p->link_on;
		if (drm_client_modeset_commit(&p->client))
			dev_warn_ratelimited(p->drm.dev, "frame commit failed\n");
		else
			WRITE_ONCE(p->link_on, true);
		epd_stat(up ? &p->st_link : &p->st_commit, t0);
		mutex_unlock(&p->link_lock);
		mod_delayed_work(system_wq, &p->link_off, msecs_to_jiffies(link_idle_ms));
		t_start = READ_ONCE(p->t_start);
		if (t_start && !null) {
			epd_stat(&p->st_first, t_start);
			WRITE_ONCE(p->t_start, 0);
		}

		spin_lock(&p->out_lock);
		p->shown = c;
		p->inflight = -1;
		spin_unlock(&p->out_lock);
		if (!null)
			p->frames++;
		wake_up(&p->out_wq);
	}
	return 0;
}

static void epd_thread_place(struct task_struct *t)
{
	unsigned long cap = 0;
	cpumask_var_t big;
	int cpu;

	sched_set_fifo_low(t);
	if (!zalloc_cpumask_var(&big, GFP_KERNEL))
		return;
	for_each_possible_cpu(cpu)
		cap = max(cap, arch_scale_cpu_capacity(cpu));
	for_each_possible_cpu(cpu)
		if (arch_scale_cpu_capacity(cpu) == cap)
			cpumask_set_cpu(cpu, big);
	set_cpus_allowed_ptr(t, big);
	free_cpumask_var(big);
}

static int epd_thread(void *data)
{
	struct onyx_epd *p = data;

	while (!kthread_should_stop()) {
		struct epd_decision d;
		bool busy, was_busy;
		u64 t0;
		int ret;

		wait_event_interruptible(p->wq, kthread_should_stop() || READ_ONCE(p->kick) ||
					 (p->eng && epd_engine_busy(p->eng) && p->client_ok));
		mutex_lock(&p->lock);
		p->kick = false;
		was_busy = p->eng && epd_engine_busy(p->eng);
		while (p->eng && epd_queue_pop(&p->q, epd_engine_busy(p->eng), &d)) {
			ret = epd_engine_submit(p->eng, &d.upd, p->grey, EPD_W);
			if (ret)
				drm_dbg(&p->drm, "update mode %d dropped: %d\n", d.upd.mode, ret);
		}
		busy = p->eng && epd_engine_busy(p->eng);
		if (busy && !was_busy && !READ_ONCE(p->t_start))
			WRITE_ONCE(p->t_start, ktime_get_ns());
		if (busy && p->client_ok) {
			if (!READ_ONCE(p->link_on)) {
				spin_lock(&p->out_lock);
				p->link_req = true;
				spin_unlock(&p->out_lock);
				wake_up(&p->out_wq);
			}
			epd_power_up(p);
			t0 = ktime_get_ns();
			ret = epd_engine_tick(p->eng, p->frame);
			epd_stat(&p->st_tick, t0);
			p->tick_cpu = raw_smp_processor_id();
			busy = epd_engine_busy(p->eng);
			mutex_unlock(&p->lock);
			if (ret == 1)
				epd_queue_frame(p);
			mutex_lock(&p->lock);
		}
		if (!busy && p->powered)
			schedule_delayed_work(&p->power_off, msecs_to_jiffies(idle_ms));
		mutex_unlock(&p->lock);
	}
	return 0;
}

static void epd_marker_done(void *ctx, u32 marker) { }

static int epd_setup_output(struct onyx_epd *p)
{
	struct drm_device *out = msm_dsi_drm_device(p->dsi_np);
	struct iosys_map map;
	int i, ret;

	if (!out || !out->registered)
		return -EPROBE_DEFER;
	ret = drm_client_init(out, &p->client, "onyx-epd", NULL);
	if (ret)
		return ret;
	ret = drm_client_modeset_probe(&p->client, 213, 1699);
	if (ret)
		goto err;
	for (i = 0; i < EPD_NBUF; i++) {
		p->buf[i] = drm_client_framebuffer_create(&p->client, 213, 1699, DRM_FORMAT_ABGR8888);
		if (IS_ERR(p->buf[i])) {
			ret = PTR_ERR(p->buf[i]);
			p->buf[i] = NULL;
			goto err;
		}
		ret = drm_client_buffer_vmap(p->buf[i], &map);
		if (ret)
			goto err;
		p->buf_vaddr[i] = map.vaddr;
	}
	p->client_ok = true;
	drm_info(&p->drm, "output on %s, buffer pitch %u\n", out->driver->name, p->buf[0]->pitch);
	return 0;
err:
	for (i = 0; i < EPD_NBUF; i++)
		if (p->buf[i]) {
			drm_client_framebuffer_delete(p->buf[i]);
			p->buf[i] = NULL;
		}
	drm_client_release(&p->client);
	return ret;
}

static void epd_setup_work(struct work_struct *w)
{
	struct onyx_epd *p = container_of(to_delayed_work(w), struct onyx_epd, setup);
	struct epd_engine_cfg cfg = {
		.timing = &epd_timing_go7,
		.temp_c = temp_c,
		.tables = { .cut = p->cut, .fill_zero = true, .a2_clean_mode = 2 },
		.marker_done = epd_marker_done,
		.ctx = p,
		.regal_aa = true,
	};
	int ret;

	if (!p->eng) {
		const struct firmware *fw;

		ret = request_firmware_direct(&fw, waveform, p->drm.dev);
		if (!ret) {
			struct epd_engine *eng = epd_engine_create(&cfg, fw->data, fw->size, &ret);

			if (eng) {
				mutex_lock(&p->lock);
				p->fw = fw;
				p->eng = eng;
				mutex_unlock(&p->lock);
				drm_info(&p->drm, "waveform %s loaded\n", waveform);
			} else {
				drm_err(&p->drm, "bad waveform %s: %d\n", waveform, ret);
				release_firmware(fw);
			}
		}
	}
	if (!p->client_ok) {
		ret = epd_setup_output(p);
		if (ret && ret != -EPROBE_DEFER)
			drm_err(&p->drm, "output setup failed: %d\n", ret);
	}
	if ((!p->eng || !p->client_ok) && ++p->tries < 240)
		schedule_delayed_work(&p->setup, HZ / 2);
	wake_up(&p->wq);
}

static const u32 epd_formats[] = { DRM_FORMAT_ABGR8888, DRM_FORMAT_XBGR8888 };

static void epd_plane_reset(struct drm_plane *plane)
{
	struct onyx_plane_state *s;

	if (plane->state) {
		plane->funcs->atomic_destroy_state(plane, plane->state);
		plane->state = NULL;
	}
	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (s)
		__drm_gem_reset_shadow_plane(plane, &s->base);
}

static struct drm_plane_state *epd_plane_duplicate(struct drm_plane *plane)
{
	struct onyx_plane_state *s;

	if (!plane->state)
		return NULL;
	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s)
		return NULL;
	__drm_gem_duplicate_shadow_plane_state(plane, &s->base);
	s->updates = NULL;
	return &s->base.base;
}

static void epd_plane_destroy_state(struct drm_plane *plane, struct drm_plane_state *state)
{
	struct onyx_plane_state *s = to_onyx_plane_state(state);

	drm_property_blob_put(s->updates);
	__drm_gem_destroy_shadow_plane_state(&s->base);
	kfree(s);
}

static int epd_plane_set_property(struct drm_plane *plane, struct drm_plane_state *state,
				  struct drm_property *property, u64 val)
{
	struct onyx_epd *p = to_epd(plane->dev);
	struct onyx_plane_state *s = to_onyx_plane_state(state);
	bool replaced;

	if (property != p->updates_prop)
		return -EINVAL;
	return drm_property_replace_blob_from_id(plane->dev, &s->updates, val, -1,
						 sizeof(struct onyx_epdc_upd), &replaced);
}

static int epd_plane_get_property(struct drm_plane *plane, const struct drm_plane_state *state,
				  struct drm_property *property, u64 *val)
{
	struct onyx_epd *p = to_epd(plane->dev);
	struct onyx_plane_state *s = to_onyx_plane_state((struct drm_plane_state *)state);

	if (property != p->updates_prop)
		return -EINVAL;
	*val = s->updates ? s->updates->base.id : 0;
	return 0;
}

static const struct drm_plane_funcs epd_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	.reset = epd_plane_reset,
	.atomic_duplicate_state = epd_plane_duplicate,
	.atomic_destroy_state = epd_plane_destroy_state,
	.atomic_set_property = epd_plane_set_property,
	.atomic_get_property = epd_plane_get_property,
};

static int epd_plane_atomic_check(struct drm_plane *plane, struct drm_atomic_state *state)
{
	struct drm_plane_state *ps = drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *cs = NULL;

	if (ps->crtc)
		cs = drm_atomic_get_new_crtc_state(state, ps->crtc);
	return drm_atomic_helper_check_plane_state(ps, cs, DRM_PLANE_NO_SCALING, DRM_PLANE_NO_SCALING,
						   false, false);
}

static void epd_plane_atomic_update(struct drm_plane *plane, struct drm_atomic_state *state)
{
	struct onyx_epd *p = to_epd(plane->dev);
	struct drm_plane_state *ps = drm_atomic_get_new_plane_state(state, plane);
	struct drm_plane_state *old = drm_atomic_get_old_plane_state(state, plane);
	struct onyx_plane_state *s = to_onyx_plane_state(ps);
	struct drm_framebuffer *fb = ps->fb;
	const u8 *vaddr;
	int idx;

	if (!fb || !ps->visible || drm_gem_fb_begin_cpu_access(fb, DMA_FROM_DEVICE))
		return;
	if (!drm_dev_enter(plane->dev, &idx))
		goto out;
	vaddr = s->base.data[0].vaddr;
	epd_link_hint(p);
	mutex_lock(&p->lock);
	if (s->updates) {
		const struct onyx_epdc_upd *u = s->updates->data;
		size_t i, n = min_t(size_t, s->updates->length / sizeof(*u), 8);

		for (i = 0; i < n; i++)
			epd_request(p, &u[i], vaddr, fb->pitches[0]);
		if (p->ui_ok)
			epd_ui_invalidate(&p->ui);
	} else if (ui && p->ui_ok) {
		struct epd_request r;
		int cut;

		u64 t0 = ktime_get_ns();
		int n = fb->format->cpp[0] == 4 &&
			epd_ui_frame(&p->ui, &p->ui_cfg, vaddr, fb->pitches[0],
				     fb->obj[0]->size - fb->offsets[0], epd_now_ms(), &r, &cut);

		p->ui_us_last = div_u64(ktime_get_ns() - t0, 1000);
		p->ui_us_max = max(p->ui_us_max, p->ui_us_last);
		p->ui_frames++;
		if (n) {
			p->ui_updates++;
			if (cut >= 0)
				epd_set_cut(p, cut);
			epd_submit(p, &r, vaddr, fb->pitches[0]);
			mod_delayed_work(system_unbound_wq, &p->ui_work, 0);
		}
	} else {
		struct drm_atomic_helper_damage_iter iter;
		struct drm_rect r;

		drm_atomic_helper_damage_iter_init(&iter, old, ps);
		drm_atomic_for_each_plane_damage(&iter, &r) {
			struct onyx_epdc_upd u = {
				.top = r.y1, .left = r.x1, .width = drm_rect_width(&r),
				.height = drm_rect_height(&r), .waveform_mode = 0xff,
			};

			epd_request(p, &u, vaddr, fb->pitches[0]);
		}
	}
	mutex_unlock(&p->lock);
	wake_up(&p->wq);
	drm_dev_exit(idx);
out:
	drm_gem_fb_end_cpu_access(fb, DMA_FROM_DEVICE);
}

static const struct drm_plane_helper_funcs epd_plane_helper_funcs = {
	DRM_GEM_SHADOW_PLANE_HELPER_FUNCS,
	.atomic_check = epd_plane_atomic_check,
	.atomic_update = epd_plane_atomic_update,
};

static void epd_crtc_atomic_flush(struct drm_crtc *crtc, struct drm_atomic_state *state)
{
	struct drm_pending_vblank_event *event = crtc->state->event;

	if (event) {
		crtc->state->event = NULL;
		spin_lock_irq(&crtc->dev->event_lock);
		drm_crtc_send_vblank_event(crtc, event);
		spin_unlock_irq(&crtc->dev->event_lock);
	}
}

static const struct drm_crtc_helper_funcs epd_crtc_helper_funcs = {
	.atomic_check = drm_crtc_helper_atomic_check,
	.atomic_flush = epd_crtc_atomic_flush,
};

static const struct drm_crtc_funcs epd_crtc_funcs = {
	.reset = drm_atomic_helper_crtc_reset,
	.destroy = drm_crtc_cleanup,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
};

static const struct drm_encoder_funcs epd_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static const struct drm_display_mode epd_mode = {
	DRM_MODE_INIT(40, EPD_W, EPD_H, 107, 142),
};

static int epd_connector_get_modes(struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &epd_mode);
}

static const struct drm_connector_helper_funcs epd_connector_helper_funcs = {
	.get_modes = epd_connector_get_modes,
};

static const struct drm_connector_funcs epd_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_mode_config_funcs epd_mode_config_funcs = {
	.fb_create = drm_gem_fb_create_with_dirty,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

DEFINE_DRM_GEM_FOPS(epd_fops);

static const struct drm_driver epd_drm_driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,
	.name = "onyx-epd",
	.desc = "Onyx Boox Go 7 e-ink",
	.major = 1,
	.minor = 0,
	DRM_GEM_SHMEM_DRIVER_OPS,
	.fops = &epd_fops,
};

static int epd_modeset_init(struct onyx_epd *p)
{
	struct drm_device *drm = &p->drm;
	int ret;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;
	drm->mode_config.min_width = EPD_W;
	drm->mode_config.max_width = EPD_W;
	drm->mode_config.min_height = EPD_H;
	drm->mode_config.max_height = EPD_H;
	drm->mode_config.preferred_depth = 24;
	drm->mode_config.funcs = &epd_mode_config_funcs;

	ret = drm_universal_plane_init(drm, &p->plane, 0, &epd_plane_funcs, epd_formats,
				       ARRAY_SIZE(epd_formats), NULL, DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;
	drm_plane_helper_add(&p->plane, &epd_plane_helper_funcs);
	drm_plane_enable_fb_damage_clips(&p->plane);
	p->updates_prop = drm_property_create(drm, DRM_MODE_PROP_BLOB, "EPD_UPDATES", 0);
	if (!p->updates_prop)
		return -ENOMEM;
	drm_object_attach_property(&p->plane.base, p->updates_prop, 0);

	ret = drm_crtc_init_with_planes(drm, &p->crtc, &p->plane, NULL, &epd_crtc_funcs, NULL);
	if (ret)
		return ret;
	drm_crtc_helper_add(&p->crtc, &epd_crtc_helper_funcs);

	ret = drm_encoder_init(drm, &p->encoder, &epd_encoder_funcs, DRM_MODE_ENCODER_NONE, NULL);
	if (ret)
		return ret;
	p->encoder.possible_crtcs = drm_crtc_mask(&p->crtc);

	ret = drm_connector_init(drm, &p->connector, &epd_connector_funcs, DRM_MODE_CONNECTOR_DPI);
	if (ret)
		return ret;
	drm_connector_helper_add(&p->connector, &epd_connector_helper_funcs);
	ret = drm_connector_attach_encoder(&p->connector, &p->encoder);
	if (ret)
		return ret;
	drm_mode_config_reset(drm);
	return 0;
}

static ssize_t cut_frame_num_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct onyx_epd *p = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", p->cut);
}

static ssize_t cut_frame_num_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct onyx_epd *p = dev_get_drvdata(dev);
	unsigned int v;
	int ret = kstrtouint(buf, 0, &v);

	if (ret)
		return ret;
	if (v > 255)
		return -EINVAL;
	mutex_lock(&p->lock);
	p->cut = v;
	if (p->eng)
		epd_engine_set_cut(p->eng, v);
	mutex_unlock(&p->lock);
	return count;
}
static DEVICE_ATTR_RW(cut_frame_num);

#define EPD_ST(s) (s).last, (u32)((s).n ? div64_u64((s).sum, (s).n) : 0), (s).max

static ssize_t status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct onyx_epd *p = dev_get_drvdata(dev);

	return sysfs_emit(buf, "waveform %s, output %s, powered %d, frames %llu, swap_rg %d\n"
			  "ui %s: commits %llu, updates %llu, passes %llu, diff %u us (max %u)\n"
			  "us last/avg/max (cpu %d): tick %u/%u/%u, copy %u/%u/%u, commit %u/%u/%u\n"
			  "  link up %u/%u/%u, power up %u/%u/%u, first frame %u/%u/%u\n",
			  p->eng ? "loaded" : "missing", p->client_ok ? "ready" : "waiting",
			  p->powered, p->frames, swap_rg,
			  !p->ui_ok ? "unavailable" : ui ? epd_ui_mode_name(p->ui_cfg.mode) : "off",
			  p->ui_frames, p->ui_updates, p->ui_passes, p->ui_us_last, p->ui_us_max,
			  p->tick_cpu, EPD_ST(p->st_tick), EPD_ST(p->st_copy), EPD_ST(p->st_commit),
			  EPD_ST(p->st_link), EPD_ST(p->st_power), EPD_ST(p->st_first));
}
static DEVICE_ATTR_RO(status);

static ssize_t ui_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct onyx_epd *p = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%s\n", epd_ui_mode_name(p->ui_cfg.mode));
}

static ssize_t ui_mode_store(struct device *dev, struct device_attribute *attr, const char *buf,
			     size_t count)
{
	struct onyx_epd *p = dev_get_drvdata(dev);
	char name[16];

	strscpy(name, buf, sizeof(name));
	mutex_lock(&p->lock);
	p->ui_cfg.mode = epd_ui_mode_by_name(strim(name));
	mutex_unlock(&p->lock);
	return count;
}
static DEVICE_ATTR_RW(ui_mode);

#define EPD_UI_INT_ATTR(name, field)								\
static ssize_t name##_show(struct device *dev, struct device_attribute *attr, char *buf)	\
{												\
	struct onyx_epd *p = dev_get_drvdata(dev);						\
												\
	return sysfs_emit(buf, "%d\n", p->ui_cfg.field);					\
}												\
static ssize_t name##_store(struct device *dev, struct device_attribute *attr,			\
			    const char *buf, size_t count)					\
{												\
	struct onyx_epd *p = dev_get_drvdata(dev);						\
	int v, ret = kstrtoint(buf, 0, &v);							\
												\
	if (ret)										\
		return ret;									\
	mutex_lock(&p->lock);									\
	p->ui_cfg.field = v;									\
	mutex_unlock(&p->lock);									\
	return count;										\
}												\
static DEVICE_ATTR_RW(name)

EPD_UI_INT_ATTR(ui_gc, gc_interval);
EPD_UI_INT_ATTR(ui_full, big_pct);
EPD_UI_INT_ATTR(ui_a2cut, a2cut);

static ssize_t test_pattern_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct onyx_epd *p = dev_get_drvdata(dev);
	struct onyx_epdc_upd u = { .width = EPD_W, .height = EPD_H, .waveform_mode = 2, .update_mode = 1 };
	char name[16];
	int mode, n, x, y;
	u32 *img;

	n = sscanf(buf, "%15s %d", name, &mode);
	if (n < 1)
		return -EINVAL;
	if (n == 2)
		u.waveform_mode = mode;
	img = vmalloc(EPD_W * EPD_H * 4);
	if (!img)
		return -ENOMEM;
	for (y = 0; y < EPD_H; y++)
		for (x = 0; x < EPD_W; x++) {
			u8 g;

			if (!strcmp(name, "white"))
				g = 0xff;
			else if (!strcmp(name, "black"))
				g = 0;
			else if (!strcmp(name, "stripes"))
				g = (x * 8 / EPD_W) * 0x24;
			else if (!strcmp(name, "checker"))
				g = ((x / 79) ^ (y / 105)) & 1 ? 0xff : 0;
			else if (!strcmp(name, "diagonal"))
				g = (x + y) % 64 < 2 || (x - y + 64 * EPD_H) % 64 < 2 ? 0 : 0xff;
			else {
				vfree(img);
				return -EINVAL;
			}
			img[y * EPD_W + x] = 0xff000000u | g << 16 | g << 8 | g;
		}
	mutex_lock(&p->lock);
	epd_request(p, &u, (const u8 *)img, EPD_W * 4);
	mutex_unlock(&p->lock);
	wake_up(&p->wq);
	vfree(img);
	return count;
}
static DEVICE_ATTR_WO(test_pattern);

static struct attribute *epd_attrs[] = {
	&dev_attr_cut_frame_num.attr,
	&dev_attr_test_pattern.attr,
	&dev_attr_status.attr,
	&dev_attr_ui_mode.attr,
	&dev_attr_ui_gc.attr,
	&dev_attr_ui_full.attr,
	&dev_attr_ui_a2cut.attr,
	NULL
};
ATTRIBUTE_GROUPS(epd);


#define EBC_SEND_UPDATE		0x700c

static int epd_resend(struct onyx_epd *p, const struct epd_request *r)
{
	struct iosys_map map[DRM_FORMAT_MAX_PLANES], data[DRM_FORMAT_MAX_PLANES];
	struct drm_plane *plane = &p->plane;
	struct drm_framebuffer *fb = NULL;
	int idx, ret;

	if (!drm_dev_enter(&p->drm, &idx))
		return -ENODEV;
	drm_modeset_lock(&plane->mutex, NULL);
	if (plane->state && plane->state->visible && plane->state->fb) {
		fb = plane->state->fb;
		drm_framebuffer_get(fb);
	}
	drm_modeset_unlock(&plane->mutex);
	ret = -EAGAIN;
	if (!fb)
		goto out;
	ret = drm_gem_fb_vmap(fb, map, data);
	if (ret)
		goto put;
	ret = drm_gem_fb_begin_cpu_access(fb, DMA_FROM_DEVICE);
	if (!ret) {
		mutex_lock(&p->lock);
		if (r)
			epd_submit(p, r, data[0].vaddr, fb->pitches[0]);
		mutex_unlock(&p->lock);
		wake_up(&p->wq);
		drm_gem_fb_end_cpu_access(fb, DMA_FROM_DEVICE);
	}
	drm_gem_fb_vunmap(fb, map);
put:
	drm_framebuffer_put(fb);
out:
	drm_dev_exit(idx);
	return ret;
}

static long ebc_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	struct onyx_epd *p = container_of(f->private_data, struct onyx_epd, misc);
	struct onyx_epdc_upd u;
	struct epd_request r;

	if (cmd != EBC_SEND_UPDATE)
		return -ENOTTY;
	if (copy_from_user(&u, (void __user *)arg, sizeof(u)))
		return -EFAULT;
	pm_wakeup_ws_event(p->ws, 750, false);	/* stock: 0x2ee ms */
	return epd_resend(p, epd_upd_to_req(&u, &r) ? &r : NULL);
}

static void epd_ui_work(struct work_struct *work)
{
	struct onyx_epd *p = container_of(to_delayed_work(work), struct onyx_epd, ui_work);
	struct epd_request r;
	enum epd_ui_step st;
	u64 wait = 0;

	mutex_lock(&p->lock);
	st = epd_ui_step(&p->ui, epd_now_ms(), &r, &wait);
	mutex_unlock(&p->lock);
	if (st == EPD_UI_WAIT)
		mod_delayed_work(system_unbound_wq, &p->ui_work, msecs_to_jiffies(wait));
	else if (st == EPD_UI_SEND && !epd_resend(p, &r))
		p->ui_passes++;
}

static const struct file_operations ebc_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = ebc_ioctl,
	.compat_ioctl = ebc_ioctl,
	.llseek = noop_llseek,
};

static int onyx_epd_suspend(struct device *dev)
{
	struct onyx_epd *p = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < 200; i++) {
		bool busy;

		mutex_lock(&p->lock);
		busy = (p->eng && epd_engine_busy(p->eng)) || p->q.n;
		mutex_unlock(&p->lock);
		if (!busy || !p->client_ok)
			break;
		msleep(10);
	}
	cancel_delayed_work_sync(&p->power_off);
	cancel_delayed_work_sync(&p->link_off);
	mutex_lock(&p->lock);
	epd_power_down(p);
	mutex_unlock(&p->lock);
	epd_link_off(p);
	return 0;
}

static int onyx_epd_resume(struct device *dev)
{
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(onyx_epd_pm, onyx_epd_suspend, onyx_epd_resume);

static struct regulator *epd_supply(struct device *dev, const char *name)
{
	struct regulator *r = devm_regulator_get_optional(dev, name);

	if (IS_ERR(r) && PTR_ERR(r) == -ENODEV)
		return NULL;
	return r;
}

static void epd_ui_free(void *data)
{
	epd_ui_release(data);
}

static void epd_vfree(void *data)
{
	vfree(data);
}

static void epd_node_put(void *data)
{
	of_node_put(data);
}

static int epd_link_tcon(struct device *dev)
{
	struct device_node *np = of_parse_phandle(dev->of_node, "onyx,tcon", 0);
	struct i2c_client *tcon;
	int ret = 0;

	if (!np)
		return 0;
	tcon = of_find_i2c_device_by_node(np);
	of_node_put(np);
	if (!tcon || !tcon->dev.driver)
		ret = -EPROBE_DEFER;
	else if (!device_link_add(dev, &tcon->dev, DL_FLAG_AUTOREMOVE_CONSUMER))
		ret = -EINVAL;
	if (tcon)
		put_device(&tcon->dev);
	return ret;
}

static int onyx_epd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct epd_timing *t = &epd_timing_go7;
	struct drm_device *out;
	struct onyx_epd *p;
	int ret;

	p = devm_drm_dev_alloc(dev, &epd_drm_driver, struct onyx_epd, drm);
	if (IS_ERR(p))
		return PTR_ERR(p);
	platform_set_drvdata(pdev, p);
	mutex_init(&p->lock);
	mutex_init(&p->link_lock);
	init_waitqueue_head(&p->wq);
	init_waitqueue_head(&p->out_wq);
	spin_lock_init(&p->out_lock);
	p->shown = p->inflight = p->pending = p->filling = -1;
	INIT_DELAYED_WORK(&p->power_off, epd_power_off_work);
	INIT_DELAYED_WORK(&p->link_off, epd_link_off_work);
	INIT_DELAYED_WORK(&p->setup, epd_setup_work);
	INIT_DELAYED_WORK(&p->ui_work, epd_ui_work);
	p->ui_cfg = (struct epd_ui_cfg){ .mode = EPD_UI_SPEED, .gc_interval = 20, .big_pct = 50, .a2cut = 3 };
	p->cut = 3;
	epd_policy_init(&p->pol, true);
	epd_queue_init(&p->q);

	p->dsi_np = of_parse_phandle(dev->of_node, "onyx,dsi", 0);
	if (!p->dsi_np)
		return dev_err_probe(dev, -EINVAL, "onyx,dsi\n");
	ret = devm_add_action_or_reset(dev, epd_node_put, p->dsi_np);
	if (ret)
		return ret;
	out = msm_dsi_drm_device(p->dsi_np);
	if (!out || !out->registered)
		return dev_err_probe(dev, -EPROBE_DEFER, "waiting for the DSI host's DRM device\n");
	ret = epd_link_tcon(dev);
	if (ret)
		return dev_err_probe(dev, ret, "onyx,tcon\n");
	p->display = epd_supply(dev, "display");
	p->vcom = epd_supply(dev, "vcom");
	p->v3p3 = epd_supply(dev, "v3p3");
	if (IS_ERR(p->display) || IS_ERR(p->vcom) || IS_ERR(p->v3p3))
		return -EPROBE_DEFER;
	epd_set_vcom(p);

	p->grey = devm_kzalloc(dev, EPD_W * EPD_H, GFP_KERNEL);
	p->err = devm_kcalloc(dev, 3 * EPD_W + 12, sizeof(*p->err), GFP_KERNEL);
	if (!p->grey || !p->err)
		return -ENOMEM;
	p->frame = vzalloc(epd_frame_words(t) * 4);
	if (!p->frame)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, epd_vfree, p->frame);
	if (ret)
		return ret;
	memset(p->grey, 0xff, EPD_W * EPD_H);
	epd_frame_template(t, p->frame);
	p->tmpl = vzalloc(epd_frame_words(t) * 4);
	if (!p->tmpl)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, epd_vfree, p->tmpl);
	if (ret)
		return ret;
	epd_frame_template(t, p->tmpl);
	p->ui_ok = !epd_ui_init(&p->ui, EPD_W, EPD_H);
	if (p->ui_ok) {
		ret = devm_add_action_or_reset(dev, epd_ui_free, &p->ui);
		if (ret)
			return ret;
	}

	ret = epd_modeset_init(p);
	if (ret)
		return ret;
	p->ws = wakeup_source_register(dev, "epdc_power");
	if (!p->ws)
		return -ENOMEM;
	ret = drm_dev_register(&p->drm, 0);
	if (ret) {
		wakeup_source_unregister(p->ws);
		return ret;
	}
	p->out_thread = kthread_run(epd_out_thread, p, "onyx-epd-out");
	if (IS_ERR(p->out_thread)) {
		ret = PTR_ERR(p->out_thread);
		goto err_ws;
	}
	epd_thread_place(p->out_thread);
	p->thread = kthread_run(epd_thread, p, "onyx-epd");
	if (IS_ERR(p->thread)) {
		ret = PTR_ERR(p->thread);
		kthread_stop(p->out_thread);
		goto err_ws;
	}
	epd_thread_place(p->thread);

	p->misc.minor = MISC_DYNAMIC_MINOR;
	p->misc.name = "ebc";
	p->misc.fops = &ebc_fops;
	p->misc.parent = dev;
	ret = misc_register(&p->misc);
	if (ret)
		goto err_thread;
	p->cls = class_create("sepdc");
	if (IS_ERR(p->cls)) {
		ret = PTR_ERR(p->cls);
		goto err_misc;
	}
	p->sysdev = device_create_with_groups(p->cls, NULL, 0, p, epd_groups, "debug");
	if (IS_ERR(p->sysdev)) {
		ret = PTR_ERR(p->sysdev);
		goto err_cls;
	}
	schedule_delayed_work(&p->setup, 0);
	return 0;

err_cls:
	class_destroy(p->cls);
err_misc:
	misc_deregister(&p->misc);
err_thread:
	kthread_stop(p->thread);
	kthread_stop(p->out_thread);
err_ws:
	drm_dev_unregister(&p->drm);
	wakeup_source_unregister(p->ws);
	return ret;
}

static void onyx_epd_remove(struct platform_device *pdev)
{
	struct onyx_epd *p = platform_get_drvdata(pdev);
	int i;

	device_unregister(p->sysdev);
	class_destroy(p->cls);
	misc_deregister(&p->misc);
	drm_dev_unplug(&p->drm);
	cancel_delayed_work_sync(&p->setup);
	cancel_delayed_work_sync(&p->ui_work);
	kthread_stop(p->thread);
	kthread_stop(p->out_thread);
	cancel_delayed_work_sync(&p->power_off);
	cancel_delayed_work_sync(&p->link_off);
	epd_power_down(p);
	if (p->client_ok) {
		for (i = 0; i < EPD_NBUF; i++)
			drm_client_framebuffer_delete(p->buf[i]);
		drm_client_release(&p->client);
	}
	epd_engine_destroy(p->eng);
	release_firmware(p->fw);
	drm_atomic_helper_shutdown(&p->drm);
	wakeup_source_unregister(p->ws);
}

static const struct of_device_id onyx_epd_of_match[] = {
	{ .compatible = "onyx,go7-epd" },
	{ }
};
MODULE_DEVICE_TABLE(of, onyx_epd_of_match);

static struct platform_driver onyx_epd_driver = {
	.probe = onyx_epd_probe,
	.remove = onyx_epd_remove,
	.driver = {
		.name = "onyx-epd",
		.of_match_table = onyx_epd_of_match,
		.dev_groups = epd_groups,
		.pm = pm_sleep_ptr(&onyx_epd_pm),
	},
};
module_platform_driver(onyx_epd_driver);

MODULE_DESCRIPTION("Onyx Boox Go 7 e-ink display");
MODULE_LICENSE("GPL");
