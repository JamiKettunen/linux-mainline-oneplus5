// FIXME: Too much stuff is hardcoded in currently!
#include <linux/input.h>
#include <linux/rmi.h>
#include <linux/of.h>
#include "rmi_driver.h"

/* Query 1 */
//#define F1A_BUTTONS_COUNT	0b00011111 /* BIT(1) */

#define MAX_NAME_LEN		256
#define F1A_MAX_BUTTONS		8

struct f1a_data {
	struct rmi_function *fn;

	unsigned char button_count;
	u32 button_map[F1A_MAX_BUTTONS];
	struct input_dev *input;
	char input_name[MAX_NAME_LEN];
	char input_phys[MAX_NAME_LEN];
	u8 button_data_buffer;
};

static int rmi_f1a_initialize(struct f1a_data *f1a)
{
	struct rmi_function *fn = f1a->fn;
	struct device *dev = &fn->dev;
	//u8 query[2];
	int error;

	/*error = rmi_read_block(fn->rmi_dev, fn->fd.query_base_addr, query, 2);
	if (error) {
		dev_err(dev, "Failed to read query register (%d).\n", error);
		return error;
	}
	printk("%s: query0: 0x%x, query1: 0x%x\n", __func__, query[0], query[1]);*/

	error = of_property_read_variable_u32_array(dev_of_node(dev),
		"syna,codes", f1a->button_map, 1, F1A_MAX_BUTTONS);
	if (error < 0) {
		dev_err(dev, "Failed to parse syna,codes from OF device tree (%d).\n", error);
		return error;
	}
	// FIXME: button_count = query[1] & F1A_BUTTONS_COUNT;
	f1a->button_count = 0;
	for (f1a->button_count = 0; f1a->button_count < F1A_MAX_BUTTONS; f1a->button_count++)
		if (f1a->button_map[f1a->button_count] == 0)
			break;
	rmi_dbg(RMI_DEBUG_FN, dev, "%s: %d button codes defined\n", __func__, f1a->button_count);

	return 0;
}

static int rmi_f1a_register_device(struct f1a_data *f1a)
{
	struct rmi_function *fn = f1a->fn;
	struct device *dev = &fn->dev;
	struct rmi_device *rmi_dev = fn->rmi_dev;
	struct input_dev *input_dev;
	int i, rc;

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(dev, "Failed to allocate input device.\n");
		return -ENOMEM;
	}

	f1a->input = input_dev;
	snprintf(f1a->input_name, MAX_NAME_LEN, "%s.fn%02x",
			dev_name(&rmi_dev->dev), fn->fd.function_number);
	input_dev->name = f1a->input_name;
	snprintf(f1a->input_phys, MAX_NAME_LEN, "%s/input0", input_dev->name);
	input_dev->phys = f1a->input_phys;
	input_dev->dev.parent = &rmi_dev->dev;
	input_set_drvdata(input_dev, f1a);

	/* set up any input events */
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);

	/* manage button map using input subsystem */
	input_dev->keycode = f1a->button_map;
	input_dev->keycodesize = sizeof(f1a->button_map); /* f1a->button_count */
	input_dev->keycodemax = f1a->button_count;

	/* set bits for each button */
	for (i = 0; i < f1a->button_count; i++) {
		set_bit(f1a->button_map[i], input_dev->keybit);
		input_set_capability(input_dev, EV_KEY, f1a->button_map[i]);
	}

	rc = input_register_device(input_dev);
	if (rc < 0) {
		dev_err(dev, "Failed to register input device.\n");
		goto error_free_device;
	}

	return 0;

error_free_device:
	input_free_device(input_dev);
	return rc;
}

static int rmi_f1a_probe(struct rmi_function *fn)
{
	struct device *dev = &fn->dev;
	struct f1a_data *f1a;
	int error;

	rmi_dbg(RMI_DEBUG_FN, dev, "%s\n", __func__);

	f1a = devm_kzalloc(dev, sizeof(struct f1a_data), GFP_KERNEL);
	if (!f1a)
		return -ENOMEM;

	f1a->fn = fn;

	error = rmi_f1a_initialize(f1a);
	if (error < 0)
		return error;

	error = rmi_f1a_register_device(f1a);
	if (error < 0)
		return error;

	dev_set_drvdata(dev, f1a);
	return 0;
}

static int rmi_f1a_config(struct rmi_function *fn)
{
	fn->rmi_dev->driver->set_irq_bits(fn->rmi_dev, fn->irq_mask);
	return 0;
}

static irqreturn_t rmi_f1a_attention(int irq, void *ctx)
{
	struct rmi_function *fn = ctx;
	struct device *dev = &fn->dev;
	struct rmi_device *rmi_dev = fn->rmi_dev;
	struct f1a_data *f1a = dev_get_drvdata(dev);
	int error, button;

	// TODO: use rmi_read_block() to accomodate up to 8 buttons?
	error = rmi_read(rmi_dev, fn->fd.data_base_addr, &(f1a->button_data_buffer));
	if (error < 0) {
		dev_err(dev, "Failed to read button data registers (%d).\n", error);
		return error;
	}
	/*rmi_dbg(RMI_DEBUG_FN, dev, "%s: button_data=0x%x\n", __func__, f1a->button_data_buffer);*/

	/* generate events for buttons that change state */
	// TODO: Implement button_data_buffer as array + button_reg = button / 8
	for (button = 0; button < f1a->button_count; button++) {
		int button_shift;
		bool button_status;
		/* bit shift to get button's status */
		button_shift = button % 8;
		button_status = ((f1a->button_data_buffer >> button_shift) & 0x01) != 0;

		rmi_dbg(RMI_DEBUG_FN, dev, "button %d (code %d) -> %d\n",
			button, f1a->button_map[button], button_status);
		/* generate an event here */
		input_report_key(f1a->input, f1a->button_map[button], button_status);
	}
	input_sync(f1a->input); /* sync after groups of events */

	return IRQ_HANDLED;
}

struct rmi_function_handler rmi_f1a_handler = {
	.driver = {
		.name = "rmi4_f1a",
	},
	.func = 0x1a,
	.probe = rmi_f1a_probe,
	.config = rmi_f1a_config,
	.attention = rmi_f1a_attention,
};
