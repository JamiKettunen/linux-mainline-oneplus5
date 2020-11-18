// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2020 Jami Kettunen <jami.kettunen@protonmail.com>
 * Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct s6e3fc1 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct regulator *supply;
	struct gpio_desc *reset_gpio;
	bool prepared;
};

static inline struct s6e3fc1 *to_s6e3fc1_panel(struct drm_panel *panel)
{
	return container_of(panel, struct s6e3fc1, panel);
}

#define dsi_dcs_write_seq(dsi, seq...) do {				\
		static const u8 d[] = { seq };				\
		int ret;						\
		ret = mipi_dsi_dcs_write_buffer(dsi, d, ARRAY_SIZE(d));	\
		if (ret < 0)						\
			return ret;					\
	} while (0)

static void s6e3fc1_reset(struct s6e3fc1 *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(2000, 3000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(2000, 3000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(2000, 3000);
}

static int s6e3fc1_on(struct s6e3fc1 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to exit sleep mode: %d\n", ret);
		return ret;
	}
	msleep(25);

	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0) {
		dev_err(dev, "Failed to set tear on: %d\n", ret);
		return ret;
	}

	dsi_dcs_write_seq(dsi, 0xfc, 0x5a, 0x5a);
	dsi_dcs_write_seq(dsi, 0xe8, 0x64, 0x08, 0x0c);
	dsi_dcs_write_seq(dsi, 0xfc, 0xa5, 0xa5);
	dsi_dcs_write_seq(dsi, 0xf0, 0x5a, 0x5a);
	dsi_dcs_write_seq(dsi, 0xb0, 0x01);
	dsi_dcs_write_seq(dsi, 0xed, 0x04);
	dsi_dcs_write_seq(dsi, 0xf0, 0xa5, 0xa5);
	dsi_dcs_write_seq(dsi, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x20);
	dsi_dcs_write_seq(dsi, MIPI_DCS_WRITE_POWER_SAVE, 0x00);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display on: %d\n", ret);
		return ret;
	}

	return 0;
}

static int s6e3fc1_off(struct s6e3fc1 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display off: %d\n", ret);
		return ret;
	}
	usleep_range(10000, 11000);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to enter sleep mode: %d\n", ret);
		return ret;
	}
	msleep(160);

	return 0;
}

static int s6e3fc1_prepare(struct drm_panel *panel)
{
	struct s6e3fc1 *ctx = to_s6e3fc1_panel(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	if (ctx->prepared)
		return 0;

	ret = regulator_enable(ctx->supply);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulator: %d\n", ret);
		return ret;
	}

	s6e3fc1_reset(ctx);

	ret = s6e3fc1_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		regulator_disable(ctx->supply);
		return ret;
	}

	ctx->prepared = true;
	return 0;
}

static int s6e3fc1_unprepare(struct drm_panel *panel)
{
	struct s6e3fc1 *ctx = to_s6e3fc1_panel(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	if (!ctx->prepared)
		return 0;

	ret = s6e3fc1_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_disable(ctx->supply);

	ctx->prepared = false;
	return 0;
}

static const struct drm_display_mode s6e3fc1_mode = {
	.clock = (1080 + 128 + 16 + 64) * (2160 + 18 + 2 + 4) * 60 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 128,
	.hsync_end = 1080 + 128 + 16,
	.htotal = 1080 + 128 + 16 + 64,
	.vdisplay = 2160,
	.vsync_start = 2160 + 18,
	.vsync_end = 2160 + 18 + 2,
	.vtotal = 2160 + 18 + 2 + 4,
	.width_mm = 68,
	.height_mm = 137,
};

static int s6e3fc1_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &s6e3fc1_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs s6e3fc1_panel_funcs = {
	.prepare = s6e3fc1_prepare,
	.unprepare = s6e3fc1_unprepare,
	.get_modes = s6e3fc1_get_modes,
};

static int s6e3fc1_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness(dsi, brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return 0;
}

static int s6e3fc1_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness(dsi, &brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return brightness;
}

static const struct backlight_ops s6e3fc1_bl_ops = {
	.update_status = s6e3fc1_bl_update_status,
	.get_brightness = s6e3fc1_bl_get_brightness,
};

static struct backlight_device *
s6e3fc1_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 1023,
		.max_brightness = 1023,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &s6e3fc1_bl_ops, &props);
}

static int s6e3fc1_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct s6e3fc1 *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->supply = devm_regulator_get(dev, "vddio");
	if (IS_ERR(ctx->supply))
		return dev_err_probe(dev, PTR_ERR(ctx->supply),
				     "Failed to get vddio regulator\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO_BURST | MIPI_DSI_MODE_VIDEO_HSE |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS;

	drm_panel_init(&ctx->panel, dev, &s6e3fc1_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	ctx->panel.backlight = s6e3fc1_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to attach to DSI host: %d\n", ret);
		return ret;
	}

	return 0;
}

static int s6e3fc1_remove(struct mipi_dsi_device *dsi)
{
	struct s6e3fc1 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id s6e3fc1_of_match[] = {
	{ .compatible = "samsung,s6e3fc1" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s6e3fc1_of_match);

static struct mipi_dsi_driver s6e3fc1_driver = {
	.probe = s6e3fc1_probe,
	.remove = s6e3fc1_remove,
	.driver = {
		.name = "panel-samsung-s6e3fc1",
		.of_match_table = s6e3fc1_of_match,
	},
};
module_mipi_dsi_driver(s6e3fc1_driver);

MODULE_AUTHOR("Jami Kettunen <jami.kettunen@protonmail.com>");
MODULE_DESCRIPTION("DRM driver for Samsung S6E3FC1 AMOLED DSI cmd mode panel found in OnePlus 5T phones");
MODULE_LICENSE("GPL v2");
