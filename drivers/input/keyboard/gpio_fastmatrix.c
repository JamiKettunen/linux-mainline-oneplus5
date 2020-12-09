// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Fast GPIO driven keyboard/keypad matrix driver
 *
 *  Copyright (c) 2020 AngeloGioacchino Del Regno
 *                     <angelogioacchino.delregno@somainline.org>
 *  Based on matrix_keypad.c
 */

#include <linux/types.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/input/matrix_keypad.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>

struct gpio_keyboard {
	struct device *dev;
	struct input_dev *input_dev;
	unsigned int row_shift;

	struct gpio_descs *row_gpios;
	struct gpio_descs *col_gpios;
	u32 autorescan_ms;
	u32 debounce_ms;
	u32 col_scan_us;
	int clustered_irq;

	DECLARE_BITMAP(disabled_gpios, MATRIX_MAX_ROWS);

	u32 last_key_state[MATRIX_MAX_COLS];
	struct delayed_work work;
	struct mutex lock;

	bool drive_inactive_cols;
	bool gpio_all_disabled;
	bool scan_pending;
	bool stopped;
};

static int activate_one_column(struct gpio_keyboard *gkb, int col, bool wait)
{
	int ret;

	ret = gpiod_direction_output(gkb->col_gpios->desc[col], 1);
	if (ret)
		return ret;
	if (wait && gkb->col_scan_us)
		udelay(gkb->col_scan_us);
	return 0;
}

/*
 * NOTE: If drive_inactive_cols is false, then the GPIO has to be put into
 * HiZ when de-activated to cause minmal side effect when scanning other
 * columns. In that case it is configured here to be input, otherwise it is
 * driven with the inactive value.
 */
static int deactivate_one_column(struct gpio_keyboard *gkb, int col)
{
	gpiod_set_value_cansleep(gkb->col_gpios->desc[col], 0);
	if (!gkb->drive_inactive_cols)
		return gpiod_direction_input(gkb->col_gpios->desc[col]);
	return 0;
}

static int activate_all_cols(struct gpio_keyboard *gkb)
{
	unsigned long val = ULONG_MAX;
	int ret, col;

	/*
	 * Shortcut! If we don't have to set direction, we can use
	 * the way faster gpiod array setting instead.
	 */
	if (gkb->drive_inactive_cols) {
		return gpiod_set_array_value_cansleep(gkb->col_gpios->ndescs,
						      gkb->col_gpios->desc,
						      gkb->col_gpios->info,
						      &val);
	}

	for (col = 0; col < gkb->col_gpios->ndescs; col++) {
		ret = activate_one_column(gkb, col, false);
		if (ret)
			return ret;
	}

	return 0;
}

static int deactivate_all_cols(struct gpio_keyboard *gkb)
{
	unsigned long val = 0;
	int col, ret;

	/*
	 * If the GPIO controller supports setting all pins at once it
	 * is going to be way faster, otherwise this function will fall
	 * back to setting all pins one at a time.
	 */
	ret = gpiod_set_array_value_cansleep(gkb->col_gpios->ndescs,
					     gkb->col_gpios->desc,
					     gkb->col_gpios->info, &val);
	if (ret)
		return ret;

	if (!gkb->drive_inactive_cols) {
		for (col = 0; col < gkb->col_gpios->ndescs; col++)
			gpiod_direction_input(gkb->col_gpios->desc[col]);
	}
	return ret;
}

static void enable_row_irqs(struct gpio_keyboard *gkb)
{
	int i;

	if (gkb->clustered_irq > 0)
		enable_irq(gkb->clustered_irq);
	else {
		for (i = 0; i < gkb->row_gpios->ndescs; i++)
			enable_irq(gpiod_to_irq(gkb->row_gpios->desc[i]));
	}
}

static void disable_row_irqs(struct gpio_keyboard *gkb)
{
	int i;

	if (gkb->clustered_irq > 0)
		disable_irq_nosync(gkb->clustered_irq);
	else {
		for (i = 0; i < gkb->row_gpios->ndescs; i++)
			disable_irq_nosync(gpiod_to_irq(gkb->row_gpios->desc[i]));
	}
}

/*
 * This gets the keys from keyboard and reports it to input subsystem
 */
static void gpio_keyboard_scan(struct work_struct *work)
{
	struct gpio_keyboard *gkb =
		container_of(work, struct gpio_keyboard, work.work);
	struct input_dev *input_dev = gkb->input_dev;
	const unsigned short *keycodes = input_dev->keycode;
	u32 *new_state;
	unsigned long row_values;
	int ret, row, col, code;
	u32 keymask = 0;

	new_state = kzalloc(gkb->col_gpios->ndescs * sizeof(*new_state),
			    GFP_KERNEL);
	if (!new_state)
		return;

	/* de-activate all columns for scanning */
	deactivate_all_cols(gkb);

	/* assert each column and read the row status out */
	for (col = 0; col < gkb->col_gpios->ndescs; col++) {
		activate_one_column(gkb, col, true);

		ret = gpiod_get_array_value_cansleep(gkb->row_gpios->ndescs,
						     gkb->row_gpios->desc,
						     gkb->row_gpios->info,
						     &row_values);
		new_state[col] = row_values;
		keymask |= new_state[col];

		if (deactivate_one_column(gkb, col)) {
			activate_all_cols(gkb);
			goto end;
		}
	}

	activate_all_cols(gkb);

	for (col = 0; col < gkb->col_gpios->ndescs; col++) {
		u32 bits_changed = gkb->last_key_state[col] ^ new_state[col];
		if (bits_changed == 0)
			continue;

		for (row = 0; bits_changed; row++, bits_changed >>=1 ) {
			if (!(bits_changed & BIT(0)))
				continue;

			code = MATRIX_SCAN_CODE(row, col, gkb->row_shift);
			input_event(input_dev, EV_MSC, MSC_SCAN, code);
			input_report_key(input_dev, keycodes[code],
					 new_state[col] & (1 << row));
		}
	}
	input_sync(input_dev);

	memcpy(gkb->last_key_state, new_state, sizeof(gkb->last_key_state));
	kfree(new_state);

	/* Avoid missing key release events on quirky hardware */
	if (gkb->autorescan_ms && keymask) {
		queue_delayed_work(system_highpri_wq, &gkb->work,
				   msecs_to_jiffies(gkb->autorescan_ms));
		return;
	}
end:
	/* Enable IRQs again */
	mutex_lock(&gkb->lock);
	gkb->scan_pending = false;
	enable_row_irqs(gkb);
	mutex_unlock(&gkb->lock);
}

static irqreturn_t gpio_keyboard_interrupt(int irq, void *id)
{
	struct gpio_keyboard *gkb = id;

	/*
	 * See if another IRQ beaten us to it and scheduled the
	 * scan already. In that case we should not try to
	 * disable IRQs again.
	 */
	if (unlikely(gkb->scan_pending || gkb->stopped))
		goto out;

	mutex_lock(&gkb->lock);

	disable_row_irqs(gkb);
	gkb->scan_pending = true;
	mod_delayed_work(system_highpri_wq, &gkb->work,
			 msecs_to_jiffies(gkb->debounce_ms));

out:
	mutex_unlock(&gkb->lock);
	return IRQ_HANDLED;
}

static int gpio_keyboard_start(struct input_dev *dev)
{
	struct gpio_keyboard *gkb = input_get_drvdata(dev);

	gkb->stopped = false;

	/*
	 * Schedule an immediate key scan to capture current key state;
	 * columns will be activated and IRQs be enabled after the scan.
	 */
	schedule_delayed_work(&gkb->work, 0);

	return 0;
}

static void gpio_keyboard_stop(struct input_dev *dev)
{
	struct gpio_keyboard *gkb = input_get_drvdata(dev);

	mutex_lock(&gkb->lock);
	gkb->stopped = true;
	mutex_unlock(&gkb->lock);

	flush_delayed_work(&gkb->work);
	/*
	 * gpio_keyboard_scan() will leave IRQs enabled;
	 * we should disable them now.
	 */
	disable_row_irqs(gkb);
}

static void __maybe_unused gpio_keyboard_wakeup_en(struct gpio_keyboard *gkb)
{
	int irq, i;

	if (gkb->clustered_irq > 0) {
		if (enable_irq_wake(gkb->clustered_irq) == 0)
			gkb->gpio_all_disabled = true;
	} else {

		for (i = 0; i < gkb->row_gpios->ndescs; i++) {
			if (!test_bit(i, gkb->disabled_gpios)) {
				irq = gpiod_to_irq(gkb->row_gpios->desc[i]);

				if (enable_irq_wake(irq) == 0)
					__set_bit(i, gkb->disabled_gpios);
			}
		}
	}
}

static void __maybe_unused gpio_keyboard_wakeup_dis(struct gpio_keyboard *gkb)
{
	int irq, i;

	if (gkb->clustered_irq > 0) {
		if (gkb->gpio_all_disabled) {
			disable_irq_wake(gkb->clustered_irq);
			gkb->gpio_all_disabled = false;
		}
	} else {
		for (i = 0; i < gkb->row_gpios->ndescs; i++) {
			if (test_and_clear_bit(i, gkb->disabled_gpios)) {
				irq = gpiod_to_irq(gkb->row_gpios->desc[i]);
				disable_irq_wake(irq);
			}
		}
	}
}

static int __maybe_unused gpio_keyboard_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_keyboard *gkb = platform_get_drvdata(pdev);

	gpio_keyboard_stop(gkb->input_dev);

	if (device_may_wakeup(&pdev->dev))
		gpio_keyboard_wakeup_en(gkb);

	return 0;
}

static int __maybe_unused gpio_keyboard_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_keyboard *gkb = platform_get_drvdata(pdev);

	if (device_may_wakeup(&pdev->dev))
		gpio_keyboard_wakeup_dis(gkb);

	gpio_keyboard_start(gkb->input_dev);

	return 0;
}
static SIMPLE_DEV_PM_OPS(gpio_keyboard_pm_ops,
			 gpio_keyboard_suspend, gpio_keyboard_resume);

static int gpio_keyboard_init_gpio(struct platform_device *pdev,
				   struct gpio_keyboard *gkb)
{
	int i, ret;

	if (gkb->clustered_irq > 0) {
		ret = devm_request_threaded_irq(gkb->dev, gkb->clustered_irq,
						NULL, gpio_keyboard_interrupt,
						IRQF_ONESHOT, "gpio-keyboard", gkb);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"Cannot get IRQ %d\n", gkb->clustered_irq);
			return ret;
		}
	} else {
		for (i = 0; i < gkb->row_gpios->ndescs; i++) {
			ret = devm_request_threaded_irq(
					gkb->dev,
					gpiod_to_irq(gkb->row_gpios->desc[i]),
					NULL, gpio_keyboard_interrupt,
					IRQF_TRIGGER_HIGH |
					IRQF_TRIGGER_LOW,
					"gpio-keyboard", gkb);
			if (ret < 0) {
				dev_err(&pdev->dev,
					"Cannot get IRQ for gpio%d\n",
					desc_to_gpio(gkb->row_gpios->desc[i]));
				return ret;
			}
		}
	}

	/* initialized as disabled - enabled by input->open */
	disable_row_irqs(gkb);
	return 0;
}

static int gpio_keyboard_parse_dt(struct gpio_keyboard *gkb)
{
	struct device *dev = gkb->dev;
	int rc;

	/*
	 * Get as GPIOD_ASIS to use the configuration that comes from
	 * device-tree. Anyway, in many cases the row is configured as
	 * input, while the column is configured as output-high.
	 *
	 * This may vary depending on the hardware.
	 */
	gkb->row_gpios = devm_gpiod_get_array(dev, "row", GPIOD_ASIS);
	if (IS_ERR(gkb->row_gpios))
		return PTR_ERR(gkb->row_gpios);

	gkb->col_gpios = devm_gpiod_get_array(dev, "col", GPIOD_ASIS);
	if (IS_ERR(gkb->col_gpios))
		return PTR_ERR(gkb->col_gpios);

	/* All of these additional properties are optional */
	device_property_read_string(dev, "label", &gkb->input_dev->name);

	if (device_property_read_bool(dev, "autorepeat"))
		__set_bit(EV_REP, gkb->input_dev->evbit);

	gkb->drive_inactive_cols =
		device_property_read_bool(dev, "drive-inactive-cols");

	rc = device_property_read_u32(dev, "autorescan-ms",
				      &gkb->autorescan_ms);
	if (rc < 0)
		gkb->autorescan_ms = 0;

	rc = device_property_read_u32(dev, "debounce-delay-ms",
				      &gkb->debounce_ms);
	if (rc < 0)
		gkb->debounce_ms = 0;

	rc = device_property_read_u32(dev, "col-scan-delay-us",
				      &gkb->col_scan_us);
	if (rc < 0)
		gkb->col_scan_us = 0;

	return 0;
}

static int gpio_keyboard_probe(struct platform_device *pdev)
{
	struct gpio_keyboard *gkb;
	bool wake;
	int irq, ret;

	gkb = devm_kmalloc(&pdev->dev, sizeof(*gkb), GFP_KERNEL);
	if (!gkb)
		return -ENOMEM;

	gkb->dev = &pdev->dev;
	gkb->input_dev = devm_input_allocate_device(&pdev->dev);
	if (!gkb->input_dev)
		return -ENOMEM;

	ret = gpio_keyboard_parse_dt(gkb);
	if (ret)
		return ret;

	irq = platform_get_irq_optional(pdev, 0);
	gkb->clustered_irq = (irq > 0) ? irq : 0;
	gkb->row_shift = get_count_order(gkb->col_gpios->ndescs);
	gkb->stopped = true;
	INIT_DELAYED_WORK(&gkb->work, gpio_keyboard_scan);
	memset(gkb->last_key_state, 0, sizeof(gkb->last_key_state));
	mutex_init(&gkb->lock);

	if (!gkb->input_dev->name)
		gkb->input_dev->name = pdev->name;
	gkb->input_dev->id.bustype = BUS_HOST;
	gkb->input_dev->dev.parent = &pdev->dev;
	gkb->input_dev->open = gpio_keyboard_start;
	gkb->input_dev->close = gpio_keyboard_stop;

	ret = matrix_keypad_build_keymap(NULL, NULL,
					 gkb->row_gpios->ndescs,
					 gkb->col_gpios->ndescs,
					 NULL, gkb->input_dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to build keymap\n");
		return ret;
	}

	input_set_capability(gkb->input_dev, EV_MSC, MSC_SCAN);
	input_set_drvdata(gkb->input_dev, gkb);

	ret = gpio_keyboard_init_gpio(pdev, gkb);
	if (ret)
		return ret;

	ret = input_register_device(gkb->input_dev);
	if (ret)
		return ret;

	wake = device_property_read_bool(&pdev->dev, "wakeup-source");
	device_init_wakeup(&pdev->dev, wake);

	platform_set_drvdata(pdev, gkb);

	return 0;
}

static int gpio_keyboard_remove(struct platform_device *pdev)
{
	struct gpio_keyboard *gkb = platform_get_drvdata(pdev);
	int i;

	if (gkb->clustered_irq > 0) {
		free_irq(gkb->clustered_irq, gkb);
	} else {
		for (i = 0; i < gkb->row_gpios->ndescs; i++)
			free_irq(gpiod_to_irq(gkb->row_gpios->desc[i]), gkb);
	}
	input_unregister_device(gkb->input_dev);

	return 0;
}

static const struct of_device_id gpio_keyboard_dt_match[] = {
	{ .compatible = "gpio-fastmatrix-keyboard" },
	{ }
};
MODULE_DEVICE_TABLE(of, gpio_keyboard_dt_match);

static struct platform_driver gpio_keyboard_driver = {
	.probe		= gpio_keyboard_probe,
	.remove		= gpio_keyboard_remove,
	.driver		= {
		.name	= "gpio-fastmatrix-keyboard",
		.pm	= &gpio_keyboard_pm_ops,
		.of_match_table = gpio_keyboard_dt_match,
	},
};
module_platform_driver(gpio_keyboard_driver);

MODULE_AUTHOR("AngeloGioacchino Del Regno <angelogioacchino.delregno@somainline.org>");
MODULE_DESCRIPTION("Fast GPIO driven keyboard/keypad matrix driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:gpio-fastmatrix-keyboard");
