// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2010 - 2017 Novatek, Inc.
 */

#include <linux/delay.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#include "nt36xxx.h"

static int nvt_i2c_read(struct i2c_client *client, uint16_t address,
			uint8_t *buf, uint16_t len)
{
	int ret;
	int retries;
	struct i2c_msg msg[2] = {
		/* first write slave position to i2c devices */
		{
			.addr = address,
			.len = 1,
			.buf = &buf[0]
		},
		/* Second read data from position */
		{
			.addr = address,
			.flags = I2C_M_RD,
			.len = len - 1,
			.buf = &buf[1]
		}
	};

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, msg, 2);
		if (ret < 0)
			return ret;
		if (ret == 2)
			break;
		retries++;
	}

	if (retries == 5)
		return -EIO;

	return 0;
}

static int nvt_i2c_write(struct i2c_client *client, uint16_t address,
			 uint8_t *buf, uint16_t len)
{
	int ret;
	int retries;
	struct i2c_msg msg = {
		.addr = address,
		.flags = !I2C_M_RD,
		.len = len,
		.buf = buf
	};

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret < 0)
			return ret;
		if (ret == 1)
			break;
		retries++;
	}

	if (retries == 5)
		return -EIO;

	return 0;
}

static void nvt_sw_reset_idle(struct nvt_i2c *nvt_i2c)
{
	uint8_t buf[4] = { 0 };

	buf[0] = 0x00;
	buf[1] = 0xA5;
	nvt_i2c_write(nvt_i2c->client, I2C_HW_Address, buf, 2);

	msleep(15);
}

static int nvt_bootloader_reset(struct nvt_i2c *nvt_i2c)
{
	int ret = 0;
	uint8_t buf[8] = { 0 };

	buf[0] = 0x00;
	buf[1] = 0x69;
	ret = nvt_i2c_write(nvt_i2c->client, I2C_HW_Address, buf, 2);

	msleep(35);

	return ret;
}

static int nvt_check_fw_reset_state(struct nvt_i2c *nvt_i2c,
				    RST_COMPLETE_STATE check_reset_state)
{
	uint8_t buf[8] = { 0 };
	int ret = 0;
	int retry = 0;

	while (1) {
		msleep(10);

		buf[0] = EVENT_MAP_RESET_COMPLETE;
		buf[1] = 0x00;
		nvt_i2c_read(nvt_i2c->client, I2C_FW_Address, buf, 6);

		if ((buf[1] >= check_reset_state) &&
		    (buf[1] <= RESET_STATE_MAX)) {
			ret = 0;
			break;
		}

		retry++;

		if (retry > 100) {
			dev_err(&nvt_i2c->client->dev,
				"error, retry=%d, buf[1]=0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
				retry, buf[1], buf[2], buf[3], buf[4], buf[5]);
			ret = -1;
			break;
		}
	}

	return ret;
}

int nvt_read_pid(struct nvt_i2c *nvt_i2c)
{
	uint8_t buf[3] = { 0 };
	int ret = 0;

	/* Set xdata index to EVENT BUF ADDR */
	buf[0] = 0xFF;
	buf[1] = (nvt_i2c->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
	buf[2] = (nvt_i2c->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
	nvt_i2c_write(nvt_i2c->client, I2C_FW_Address, buf, 3);

	/* Read project id */
	buf[0] = EVENT_MAP_PROJECTID;
	buf[1] = 0x00;
	buf[2] = 0x00;
	nvt_i2c_read(nvt_i2c->client, I2C_FW_Address, buf, 3);

	nvt_i2c->nvt_pid = (buf[2] << 8) + buf[1];

	dev_info(&nvt_i2c->client->dev, "PID=%04X\n", nvt_i2c->nvt_pid);

	return ret;
}

static int nvt_get_fw_info(struct nvt_i2c *nvt_i2c)
{
	uint8_t buf[64] = { 0 };
	unsigned int retry_count = 0;
	int ret = 0;

info_retry:
	/* Set xdata index to EVENT BUF ADDR */
	buf[0] = 0xFF;
	buf[1] = (nvt_i2c->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
	buf[2] = (nvt_i2c->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
	nvt_i2c_write(nvt_i2c->client, I2C_FW_Address, buf, 3);

	/* Read fw info */
	buf[0] = EVENT_MAP_FWINFO;
	nvt_i2c_read(nvt_i2c->client, I2C_FW_Address, buf, 17);
	nvt_i2c->fw_ver = buf[1];
	nvt_i2c->x_num = buf[3];
	nvt_i2c->y_num = buf[4];
	nvt_i2c->abs_x_max = (uint16_t)((buf[5] << 8) | buf[6]);
	nvt_i2c->abs_y_max = (uint16_t)((buf[7] << 8) | buf[8]);
	nvt_i2c->max_button_num = buf[11];

	/* Clear x_num, y_num if fw info is broken */
	if ((buf[1] + buf[2]) != 0xFF) {
		dev_err(&nvt_i2c->client->dev,
			"FW info is broken! fw_ver=0x%02X, ~fw_ver=0x%02X\n",
			buf[1], buf[2]);
		nvt_i2c->fw_ver = 0;
		nvt_i2c->x_num = 18;
		nvt_i2c->y_num = 32;
		nvt_i2c->abs_x_max = TOUCH_DEFAULT_MAX_WIDTH;
		nvt_i2c->abs_y_max = TOUCH_DEFAULT_MAX_HEIGHT;
		nvt_i2c->max_button_num = 0;

		if (retry_count < 3) {
			retry_count++;
			dev_info(&nvt_i2c->client->dev, "retry_count=%d\n", retry_count);
			goto info_retry;
		}

		dev_info(&nvt_i2c->client->dev, "Set default fw_ver=%d, x_num=%d, y_num=%d, \
			 abs_x_max=%d, abs_y_max=%d, max_button_num=%d!\n",
			 nvt_i2c->fw_ver, nvt_i2c->x_num, nvt_i2c->y_num,
			 nvt_i2c->abs_x_max, nvt_i2c->abs_y_max,
			 nvt_i2c->max_button_num);
		ret = -1;
	} else {
		ret = 0;
	}

	/* Get Novatek PID */
	nvt_read_pid(nvt_i2c);

	return ret;
}

#ifdef CONFIG_OF
static int nvt_parse_dt(struct device *dev)
{
	struct nvt_i2c *nvt_i2c = i2c_get_clientdata(to_i2c_client(dev));
	struct device_node *np = dev->of_node;
	int ret;
	const char *name;

	nvt_i2c->irq_gpio =
		of_get_named_gpio_flags(np, "novatek,irq-gpio", 0, NULL);
	dev_info(&nvt_i2c->client->dev,
		 "novatek,irq-gpio=%d\n", nvt_i2c->irq_gpio);

	nvt_i2c->reset_gpio =
		of_get_named_gpio_flags(np, "novatek,reset-gpio", 0, NULL);
	dev_info(&nvt_i2c->client->dev,
		 "novatek,reset-gpio=%d\n", nvt_i2c->reset_gpio);

	ret = of_property_read_string(np, "novatek,vddio-reg-name", &name);
	if (ret == -EINVAL)
		nvt_i2c->vddio_reg_name = NULL;
	else if (ret < 0)
		return -EINVAL;
	else {
		nvt_i2c->vddio_reg_name = name;
		dev_info(&nvt_i2c->client->dev, "vddio_reg_name = %s\n", name);
	}

	ret = of_property_read_string(np, "novatek,lab-reg-name", &name);
	if (ret == -EINVAL)
		nvt_i2c->lab_reg_name = NULL;
	else if (ret < 0)
		return -EINVAL;
	else {
		nvt_i2c->lab_reg_name = name;
		dev_info(&nvt_i2c->client->dev, "lab_reg_name = %s\n", name);
	}

	ret = of_property_read_string(np, "novatek,ibb-reg-name", &name);
	if (ret == -EINVAL)
		nvt_i2c->ibb_reg_name = NULL;
	else if (ret < 0)
		return -EINVAL;
	else {
		nvt_i2c->ibb_reg_name = name;
		dev_info(&nvt_i2c->client->dev, "ibb_reg_name = %s\n", name);
	}

	return 0;
}
#else
static int nvt_parse_dt(struct device *dev)
{
	struct nvt_i2c *nvt_i2c = i2c_get_clientdata(to_i2c_client(dev));
	nvt_i2c->irq_gpio = NVTTOUCH_INT_PIN;

	return 0;
}
#endif

static int nvt_get_reg(struct nvt_i2c *nvt_i2c, bool get)
{
	int ret;

	if (!get) {
		ret = 0;
		goto regulator_put;
	}

	if (nvt_i2c->vddio_reg_name && (*nvt_i2c->vddio_reg_name != 0)) {
		nvt_i2c->vddio_reg = regulator_get(&nvt_i2c->client->dev,
						   nvt_i2c->vddio_reg_name);
		if (IS_ERR(nvt_i2c->vddio_reg)) {
			dev_err(&nvt_i2c->client->dev, "Failed to get power regulator\n");
			ret = PTR_ERR(nvt_i2c->vddio_reg);
			goto regulator_put;
		}
	}

	if (nvt_i2c->lab_reg_name && (*nvt_i2c->lab_reg_name != 0)) {
		nvt_i2c->lab_reg = regulator_get(&nvt_i2c->client->dev,
						 nvt_i2c->lab_reg_name);
		if (IS_ERR(nvt_i2c->lab_reg)) {
			dev_err(&nvt_i2c->client->dev, "Failed to get lab regulator\n");
			ret = PTR_ERR(nvt_i2c->lab_reg);
			goto regulator_put;
		}
	}

	if (nvt_i2c->ibb_reg_name && (*nvt_i2c->ibb_reg_name != 0)) {
		nvt_i2c->ibb_reg = regulator_get(&nvt_i2c->client->dev,
						 nvt_i2c->ibb_reg_name);
		if (IS_ERR(nvt_i2c->ibb_reg)) {
			dev_err(&nvt_i2c->client->dev, "Failed to get ibb regulator\n");
			ret = PTR_ERR(nvt_i2c->ibb_reg);
			goto regulator_put;
		}
	}

	return 0;

regulator_put:
	if (nvt_i2c->vddio_reg) {
		regulator_put(nvt_i2c->vddio_reg);
		nvt_i2c->vddio_reg = NULL;
	}
	if (nvt_i2c->lab_reg) {
		regulator_put(nvt_i2c->lab_reg);
		nvt_i2c->lab_reg = NULL;
	}
	if (nvt_i2c->ibb_reg) {
		regulator_put(nvt_i2c->ibb_reg);
		nvt_i2c->ibb_reg = NULL;
	}

	return ret;
}

static int nvt_enable_reg(struct nvt_i2c *nvt_i2c, bool enable)
{
	int ret;

	if (!enable) {
		ret = 0;
		goto disable_ibb_reg;
	}

	if (nvt_i2c->vddio_reg) {
		ret = regulator_enable(nvt_i2c->vddio_reg);
		if (ret < 0) {
			dev_err(&nvt_i2c->client->dev, "Failed to enable vddio regulator\n");
			goto exit;
		}
	}

	if (nvt_i2c->lab_reg && nvt_i2c->lab_reg) {
		ret = regulator_enable(nvt_i2c->lab_reg);
		if (ret < 0) {
			dev_err(&nvt_i2c->client->dev, "Failed to enable lab regulator\n");
			goto disable_vddio_reg;
		}
	}

	if (nvt_i2c->ibb_reg) {
		ret = regulator_enable(nvt_i2c->ibb_reg);
		if (ret < 0) {
			dev_err(&nvt_i2c->client->dev, "Failed to enable ibb regulator\n");
			goto disable_lab_reg;
		}
	}

	return 0;

disable_ibb_reg:
	if (nvt_i2c->ibb_reg)
		regulator_disable(nvt_i2c->ibb_reg);

disable_lab_reg:
	if (nvt_i2c->lab_reg)
		regulator_disable(nvt_i2c->lab_reg);

disable_vddio_reg:
	if (nvt_i2c->vddio_reg)
		regulator_disable(nvt_i2c->vddio_reg);

exit:
	return ret;
}

static int nvt_gpio_config(struct nvt_i2c *nvt_i2c)
{
	int ret = 0;

	/* Request INT-pin (Input) */
	if (gpio_is_valid(nvt_i2c->irq_gpio)) {
		ret = gpio_request_one(nvt_i2c->irq_gpio, GPIOF_IN, "NVT-int");
		if (ret) {
			dev_err(&nvt_i2c->client->dev, "Failed to request NVT-int GPIO\n");
			goto err_request_irq_gpio;
		}
	}

	if (gpio_is_valid(nvt_i2c->reset_gpio)) {
		ret = gpio_request_one(nvt_i2c->reset_gpio, GPIOF_OUT_INIT_HIGH,
				       "NVT-reset");
		if (ret) {
			dev_err(&nvt_i2c->client->dev, "Failed to request reset-int GPIO\n");
			goto err_request_reset_gpio;
		}
		gpio_direction_output(nvt_i2c->reset_gpio, 1);
	}

	return ret;

err_request_reset_gpio:
err_request_irq_gpio:
	return ret;
}

static void nvt_ts_work_func(struct work_struct *work)
{
	struct nvt_i2c *nvt_i2c = container_of(work, struct nvt_i2c, ts_work);

	int ret = -1;
	uint8_t point_data[POINT_DATA_LEN + 1] = { 0 };
	unsigned int position = 0;
	unsigned int input_x = 0;
	unsigned int input_y = 0;
	unsigned int input_w = 0;
	unsigned int input_p = 0;
	uint8_t input_id = 0;
	uint8_t press_id[TOUCH_MAX_FINGER_NUM] = { 0 };
	int i = 0;
	int finger_cnt = 0;

	mutex_lock(&nvt_i2c->lock);

	if (nvt_i2c->dev_pm_suspend) {
		ret = wait_for_completion_timeout(
			&nvt_i2c->dev_pm_suspend_completion,
			msecs_to_jiffies(500));
		if (!ret) {
			dev_err(&nvt_i2c->client->dev,
				"system(i2c) can't finished resuming procedure, skip it\n");
			goto xfer_error;
		}
	}

	ret = nvt_i2c_read(nvt_i2c->client, I2C_FW_Address, point_data,
			   POINT_DATA_LEN + 1);
	if (ret < 0) {
		dev_err(&nvt_i2c->client->dev, "nvt_i2c_read failed.(%d)\n", ret);
		goto xfer_error;
	}

	for (i = 0; i < nvt_i2c->max_touch_num; i++) {
		position = 1 + 6 * i;
		input_id = (uint8_t)(point_data[position + 0] >> 3);
		if ((input_id == 0) || (input_id > nvt_i2c->max_touch_num))
			continue;

		if (((point_data[position] & 0x07) == 0x01) ||
		    ((point_data[position] & 0x07) == 0x02)) {
			input_x = (uint)(point_data[position + 1] << 4) +
				  (uint)(point_data[position + 3] >> 4);
			input_y = (uint)(point_data[position + 2] << 4) +
				  (uint)(point_data[position + 3] & 0x0F);
			if ((input_x < 0) || (input_y < 0))
				continue;
			if ((input_x > nvt_i2c->abs_x_max) ||
			    (input_y > nvt_i2c->abs_y_max))
				continue;
			input_w = (uint)(point_data[position + 4]);
			if (input_w == 0)
				input_w = 1;
			if (i < 2) {
				input_p = (uint)(point_data[position + 5]) +
					  (uint)(point_data[i + 63] << 8);
				if (input_p > TOUCH_FORCE_NUM)
					input_p = TOUCH_FORCE_NUM;
			} else {
				input_p = (uint)(point_data[position + 5]);
			}
			if (input_p == 0)
				input_p = 1;

			press_id[input_id - 1] = 1;
			input_mt_slot(nvt_i2c->input, input_id - 1);
			input_mt_report_slot_state(nvt_i2c->input, MT_TOOL_FINGER, true);

			input_report_abs(nvt_i2c->input, ABS_MT_POSITION_X, input_x);
			input_report_abs(nvt_i2c->input, ABS_MT_POSITION_Y, input_y);
			input_report_abs(nvt_i2c->input, ABS_MT_TOUCH_MAJOR, input_w);
			input_report_abs(nvt_i2c->input, ABS_MT_PRESSURE, input_p);

			finger_cnt++;
		}
	}

	for (i = 0; i < nvt_i2c->max_touch_num; i++) {
		if (press_id[i] != 1) {
			input_mt_slot(nvt_i2c->input, i);
			input_report_abs(nvt_i2c->input, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(nvt_i2c->input, ABS_MT_PRESSURE, 0);
			input_mt_report_slot_state(nvt_i2c->input,
						   MT_TOOL_FINGER, false);
		}
	}

	input_report_key(nvt_i2c->input, BTN_TOUCH, (finger_cnt > 0));

	input_sync(nvt_i2c->input);

xfer_error:
	enable_irq(nvt_i2c->client->irq);

	mutex_unlock(&nvt_i2c->lock);
}

static irqreturn_t nvt_ts_irq_handler(int irq, void *dev_id)
{
	struct nvt_i2c *nvt_i2c = dev_id;

	disable_irq_nosync(nvt_i2c->client->irq);
	queue_work(nvt_i2c->ts_workq, &nvt_i2c->ts_work);

	return IRQ_HANDLED;
}

static void nvt_stop_crc_reboot(struct nvt_i2c *nvt_i2c)
{
	uint8_t buf[8] = { 0 };
	int retry = 0;

	/* Read dummy buffer to check CRC fail reboot is happening or not */

	/* Change I2C index to prevent geting 0xFF, but not 0xFC */
	buf[0] = 0xFF;
	buf[1] = 0x01;
	buf[2] = 0xF6;
	nvt_i2c_write(nvt_i2c->client, I2C_BLDR_Address, buf, 3);

	/* Read to check if buf is 0xFC which means IC is in CRC reboot */
	buf[0] = 0x4E;
	nvt_i2c_read(nvt_i2c->client, I2C_BLDR_Address, buf, 4);

	if (((buf[1] == 0xFC) && (buf[2] == 0xFC) && (buf[3] == 0xFC)) ||
	    ((buf[1] == 0xFF) && (buf[2] == 0xFF) && (buf[3] == 0xFF))) {
		/* IC is in CRC fail reboot loop, needs to be stopped! */
		for (retry = 5; retry > 0; retry--) {
			/* Write i2c cmds to reset idle - part #1 */
			buf[0] = 0x00;
			buf[1] = 0xA5;
			nvt_i2c_write(nvt_i2c->client, I2C_HW_Address, buf, 2);

			/* Write i2c cmds to reset idle - part #2 */
			buf[0] = 0x00;
			buf[1] = 0xA5;
			nvt_i2c_write(nvt_i2c->client, I2C_HW_Address, buf, 2);
			msleep(1);

			/* Clear CRC_ERR_FLAG */
			buf[0] = 0xFF;
			buf[1] = 0x03;
			buf[2] = 0xF1;
			nvt_i2c_write(nvt_i2c->client, I2C_BLDR_Address, buf, 3);

			buf[0] = 0x35;
			buf[1] = 0xA5;
			nvt_i2c_write(nvt_i2c->client, I2C_BLDR_Address, buf, 2);

			/* Check CRC_ERR_FLAG */
			buf[0] = 0xFF;
			buf[1] = 0x03;
			buf[2] = 0xF1;
			nvt_i2c_write(nvt_i2c->client, I2C_BLDR_Address, buf, 3);

			buf[0] = 0x35;
			buf[1] = 0x00;
			nvt_i2c_read(nvt_i2c->client, I2C_BLDR_Address, buf, 2);

			if (buf[1] == 0xA5)
				break;
		}

		if (retry == 0)
			 dev_err(&nvt_i2c->client->dev,
				 "CRC auto reboot is not able to be stopped! buf[1]=0x%02X\n",
				 buf[1]);
	}

	return;
}

static int8_t nvt_ts_check_chip_ver_trim(struct nvt_i2c *nvt_i2c)
{
	uint8_t buf[8] = { 0 };
	int retry = 0;
	int list = 0;
	int i = 0;
	int found_nvt_chip = 0;
	int ret = -1;

	ret = nvt_bootloader_reset(nvt_i2c);
	if (ret < 0) {
		dev_err(&nvt_i2c->client->dev, "Can't reset the nvt IC\n");
		return ret;
	}

	for (retry = 5; retry > 0; retry--) {
		nvt_sw_reset_idle(nvt_i2c);

		buf[0] = 0x00;
		buf[1] = 0x35;
		nvt_i2c_write(nvt_i2c->client, I2C_HW_Address, buf, 2);
		msleep(10);

		buf[0] = 0xFF;
		buf[1] = 0x01;
		buf[2] = 0xF6;
		nvt_i2c_write(nvt_i2c->client, I2C_BLDR_Address, buf, 3);

		buf[0] = 0x4E;
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		buf[6] = 0x00;
		nvt_i2c_read(nvt_i2c->client, I2C_BLDR_Address, buf, 7);

		/* Compare read chip id on supported list */
		for (list = 0; list < (sizeof(trim_id_table) /
				       sizeof(struct nvt_ts_trim_id_table));
		     list++) {
			found_nvt_chip = 0;

			/* Compare each byte */
			for (i = 0; i < NVT_ID_BYTE_MAX; i++) {
				if (trim_id_table[list].mask[i] &&
					buf[i + 1] != trim_id_table[list].id[i])
					break;
			}

			if (i == NVT_ID_BYTE_MAX)
				found_nvt_chip = 1;

			if (found_nvt_chip) {
				nvt_i2c->mmap = trim_id_table[list].mmap;
				nvt_i2c->carrier_system =
					trim_id_table[list].carrier_system;
				ret = 0;
				return ret;
			}

			nvt_i2c->mmap = NULL;
			ret = -1;
		}

		/* Stop CRC check to prevent IC auto reboot */
		if (((buf[1] == 0xFC) && (buf[2] == 0xFC) &&
		     (buf[3] == 0xFC)) ||
		    ((buf[1] == 0xFF) && (buf[2] == 0xFF) &&
		     (buf[3] == 0xFF))) {
			nvt_stop_crc_reboot(nvt_i2c);
		}

		msleep(10);
	}

	return ret;
}

static int nvt_ts_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct nvt_i2c *nvt_i2c;
	struct input_dev *input;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		return -EIO;
	}

	nvt_i2c = kzalloc(sizeof(*nvt_i2c), GFP_KERNEL);
	input = input_allocate_device();
	if (!nvt_i2c || !input)
		return -ENOMEM;

	nvt_i2c->client = client;
	nvt_i2c->input = input;
	i2c_set_clientdata(client, nvt_i2c);

	nvt_parse_dt(&client->dev);

	ret = nvt_gpio_config(nvt_i2c);
	if (ret) {
		dev_err(&client->dev, "GPIO config error\n");
		return ret;
	}

	nvt_get_reg(nvt_i2c, true);
	nvt_enable_reg(nvt_i2c, true);

	msleep(10);

	ret = nvt_ts_check_chip_ver_trim(nvt_i2c);
	if (ret) {
		dev_err(&client->dev, "Failed to check chip version\n");
		return -EINVAL;
	}

	mutex_init(&nvt_i2c->mdata_lock);
	mutex_init(&nvt_i2c->lock);

	mutex_lock(&nvt_i2c->lock);
	nvt_bootloader_reset(nvt_i2c);
	nvt_check_fw_reset_state(nvt_i2c, RESET_STATE_INIT);
	nvt_get_fw_info(nvt_i2c);
	mutex_unlock(&nvt_i2c->lock);

	nvt_i2c->ts_workq = alloc_ordered_workqueue("nt36xxx", 0);
	if (!nvt_i2c->ts_workq) {
		dev_err(&client->dev, "Failed to create workqueue\n");
		return -EINVAL;
	}

	nvt_i2c->int_trigger_type = INT_TRIGGER_TYPE;
	nvt_i2c->max_touch_num = TOUCH_MAX_FINGER_NUM;
	INIT_WORK(&nvt_i2c->ts_work, nvt_ts_work_func);

	snprintf(nvt_i2c->phys, sizeof(nvt_i2c->phys), "%s/input0",
		 dev_name(&client->dev));
	input->name = NVT_TS_NAME;
	input->phys = nvt_i2c->phys;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;

	input->evbit[0] =
		BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input->propbit[0] = BIT(INPUT_PROP_DIRECT);

	input_mt_init_slots(input, nvt_i2c->max_touch_num, 0);

	input_set_abs_params(input, ABS_MT_PRESSURE, 0, TOUCH_FORCE_NUM, 0, 0);

#if TOUCH_MAX_FINGER_NUM > 1
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	input_set_abs_params(input, ABS_MT_POSITION_X, 0,
			     nvt_i2c->abs_x_max - 1, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0,
			     nvt_i2c->abs_y_max - 1, 0, 0);
#endif

	input_set_drvdata(input, nvt_i2c);

	ret = input_register_device(nvt_i2c->input);
	if (ret) {
		dev_err(&client->dev, "Failed to register input device: %d\n",
			ret);
		return ret;
	}

	client->irq = gpio_to_irq(nvt_i2c->irq_gpio);
	if (client->irq) {
		ret = request_irq(client->irq, nvt_ts_irq_handler,
				  nvt_i2c->int_trigger_type, client->name,
				  nvt_i2c);
		if (ret != 0) {
			dev_err(&client->dev, "request irq failed. ret=%d\n",
				ret);
			return ret;
		} else {
			disable_irq(client->irq);
		}
	}

	device_init_wakeup(&client->dev, 1);
	nvt_i2c->dev_pm_suspend = false;
	init_completion(&nvt_i2c->dev_pm_suspend_completion);

	enable_irq(client->irq);

	return 0;
}

static int nvt_ts_remove(struct i2c_client *client)
{
	struct nvt_i2c *nvt_i2c = i2c_get_clientdata(client);

	free_irq(client->irq, nvt_i2c);

	nvt_get_reg(nvt_i2c, false);
	nvt_enable_reg(nvt_i2c, false);

	mutex_destroy(&nvt_i2c->lock);

	input_unregister_device(nvt_i2c->input);
	kfree(nvt_i2c);

	return 0;
}

static int __maybe_unused nvt_ts_suspend(struct device *dev)
{
	struct nvt_i2c *nvt_i2c = i2c_get_clientdata(to_i2c_client(dev));
	uint8_t buf[4] = { 0 };
	int i;

	mutex_lock(&nvt_i2c->lock);

	disable_irq(nvt_i2c->client->irq);

	/* Write i2c command to enter "deep sleep mode" */
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x11;
	nvt_i2c_write(nvt_i2c->client, I2C_FW_Address, buf, 2);

	/* Release all touches */
	for (i = 0; i < nvt_i2c->max_touch_num; i++) {
		input_mt_slot(nvt_i2c->input, i);
		input_report_abs(nvt_i2c->input, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(nvt_i2c->input, ABS_MT_PRESSURE, 0);
		input_mt_report_slot_state(nvt_i2c->input, MT_TOOL_FINGER, 0);
	}

	input_sync(nvt_i2c->input);

	msleep(50);

	mutex_unlock(&nvt_i2c->lock);

	return 0;
}

static int __maybe_unused nvt_ts_resume(struct device *dev)
{
	struct nvt_i2c *nvt_i2c = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&nvt_i2c->lock);

	if (gpio_is_valid(nvt_i2c->reset_gpio)) {
		gpio_set_value(nvt_i2c->reset_gpio, 1);
	}

	nvt_bootloader_reset(nvt_i2c);
	nvt_check_fw_reset_state(nvt_i2c, RESET_STATE_REK);

	enable_irq(nvt_i2c->client->irq);

	mutex_unlock(&nvt_i2c->lock);

	return 0;
}

static SIMPLE_DEV_PM_OPS(nvt_ts_pm, nvt_ts_suspend, nvt_ts_resume);

#ifdef CONFIG_OF
static const struct of_device_id nvt_match_table[] = {
	{ .compatible = "novatek,nt36xxx" },
	{ }
};
MODULE_DEVICE_TABLE(of, nvt_match_table);
#endif

static const struct i2c_device_id nvt_ts_id[] = {
	{ NVT_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nvt_ts_id);

static struct i2c_driver nvt_ts_driver = {
	.driver = {
		.name	= NVT_I2C_NAME,
		.of_match_table = of_match_ptr(nvt_match_table),
		.pm	= &nvt_ts_pm,
	},
	.probe		= nvt_ts_probe,
	.remove		= nvt_ts_remove,
	.id_table	= nvt_ts_id,
};
module_i2c_driver(nvt_ts_driver);

MODULE_DESCRIPTION("Novatek Touchscreen Driver");
