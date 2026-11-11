// SPDX-License-Identifier: GPL-2.0-only
/*
 * DSI side of the Onyx Boox Go 7 e-ink panel
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

static bool tear_on = true;
module_param(tear_on, bool, 0644);
MODULE_PARM_DESC(tear_on, "send DCS set_tear_on at each link-up");

static bool reinit;
module_param(reinit, bool, 0644);
MODULE_PARM_DESC(reinit, "reset and re-init TCON at every link-up instead of only the first");

struct go7_eink {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset;
	bool inited;
};

static struct go7_eink *to_go7(struct drm_panel *panel)
{
	return container_of(panel, struct go7_eink, panel);
}

static int go7_prepare(struct drm_panel *panel)
{
	struct go7_eink *ctx = to_go7(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	if (ctx->inited && !reinit)
		return 0;
	if (ctx->reset) {
		gpiod_set_value_cansleep(ctx->reset, 0);
		usleep_range(10000, 11000);
		gpiod_set_value_cansleep(ctx->reset, 1);
		usleep_range(10000, 11000);
		gpiod_set_value_cansleep(ctx->reset, 0);
		usleep_range(10000, 11000);
	}
	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_PIXEL_FORMAT, 0x07);
	mipi_dsi_msleep(&dsi_ctx, 9);
	if (tear_on)
		mipi_dsi_dcs_set_tear_on_multi(&dsi_ctx, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
	ctx->inited = !dsi_ctx.accum_err;
	return dsi_ctx.accum_err;
}

static int go7_unprepare(struct drm_panel *panel)
{
	struct go7_eink *ctx = to_go7(panel);

	if (!reinit)
		return 0;
	if (ctx->reset)
		gpiod_set_value_cansleep(ctx->reset, 1);
	ctx->inited = false;
	return 0;
}

static const struct drm_display_mode go7_mode = {
	.clock = 35791,
	.hdisplay = 213,
	.hsync_start = 213 + 1,
	.hsync_end = 213 + 1 + 1,
	.htotal = 213 + 1 + 1 + 1,
	.vdisplay = 1699,
	.vsync_start = 1699 + 287,
	.vsync_end = 1699 + 287 + 1,
	.vtotal = 1699 + 287 + 1 + 1,
	.width_mm = 107,
	.height_mm = 142,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static int go7_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &go7_mode);
}

static const struct drm_panel_funcs go7_funcs = {
	.prepare = go7_prepare,
	.unprepare = go7_unprepare,
	.get_modes = go7_get_modes,
};

static int go7_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct go7_eink *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct go7_eink, panel, &go7_funcs, DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);
	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset))
		return dev_err_probe(dev, PTR_ERR(ctx->reset), "reset-gpios\n");

	ctx->panel.prepare_prev_first = true;
	drm_panel_add(&ctx->panel);

	dsi->lanes = 2;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS;
	ret = mipi_dsi_attach(dsi);
	if (ret) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "DSI attach\n");
	}
	return 0;
}

static void go7_remove(struct mipi_dsi_device *dsi)
{
	struct go7_eink *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id go7_of_match[] = {
	{ .compatible = "onyx,go7-eink-dsi" },
	{ }
};
MODULE_DEVICE_TABLE(of, go7_of_match);

static struct mipi_dsi_driver go7_driver = {
	.driver = {
		.name = "panel-onyx-go7-eink",
		.of_match_table = go7_of_match,
	},
	.probe = go7_probe,
	.remove = go7_remove,
};
module_mipi_dsi_driver(go7_driver);

MODULE_DESCRIPTION("Onyx Boox Go 7 e-ink DSI link");
MODULE_LICENSE("GPL");
