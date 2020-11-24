// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
 * Copyright (c) 2019, Linaro Limited
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_opp.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/nvmem-consumer.h>
#include "cpr-common.h"

int cpr_read_efuse(struct device *dev, const char *cname, u32 *data)
{
	struct nvmem_cell *cell;
	ssize_t len;
	char *ret;
	int i;

	*data = 0;

	cell = nvmem_cell_get(dev, cname);
	if (IS_ERR(cell)) {
		if (PTR_ERR(cell) != -EPROBE_DEFER)
			dev_err(dev, "undefined cell %s\n", cname);
		return PTR_ERR(cell);
	}

	ret = nvmem_cell_read(cell, &len);
	nvmem_cell_put(cell);
	if (IS_ERR(ret)) {
		dev_err(dev, "can't read cell %s\n", cname);
		return PTR_ERR(ret);
	}

	for (i = 0; i < len; i++)
		*data |= ret[i] << (8 * i);

	kfree(ret);
	dev_dbg(dev, "efuse read(%s) = %x, bytes %zd\n", cname, *data, len);

	return 0;
}

int cpr_populate_ring_osc_idx(struct device *dev,
			      struct fuse_corner *fuse_corner,
			      const struct cpr_fuse *cpr_fuse,
			      int num_fuse_corners)
{
	struct fuse_corner *end = fuse_corner + num_fuse_corners;
	u32 data;
	int ret;

	for (; fuse_corner < end; fuse_corner++, cpr_fuse++) {
		ret = cpr_read_efuse(dev, cpr_fuse->ring_osc,
				     &data);
		if (ret)
			return ret;
		fuse_corner->ring_osc_idx = data;
	}

	return 0;
}

int cpr_read_fuse_uV(int init_v_width, int step_size_uV, int ref_uV,
		     int adj, int step_volt, const char *init_v_efuse,
		     struct device *dev)
{
	int steps, uV;
	u32 bits = 0;
	int ret;

	ret = cpr_read_efuse(dev, init_v_efuse, &bits);
	if (ret)
		return ret;

	steps = bits & (BIT(init_v_width - 1) - 1);
	/* Not two's complement.. instead highest bit is sign bit */
	if (bits & BIT(init_v_width - 1))
		steps = -steps;

	uV = ref_uV + steps * step_size_uV;

	/* Apply open-loop fixed adjustments to fused values */
	uV += adj;

	return DIV_ROUND_UP(uV, step_volt) * step_volt;
}

const struct cpr_fuse *cpr_get_fuses(struct device *dev, int tid,
				     int num_fuse_corners)
{
	struct cpr_fuse *fuses;
	int i;

	fuses = devm_kcalloc(dev, num_fuse_corners,
			     sizeof(struct cpr_fuse),
			     GFP_KERNEL);
	if (!fuses)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < num_fuse_corners; i++) {
		char tbuf[50];

		snprintf(tbuf, sizeof(tbuf), "cpr%d_ring_osc%d", tid, i + 1);
		fuses[i].ring_osc = devm_kstrdup(dev, tbuf, GFP_KERNEL);
		if (!fuses[i].ring_osc)
			return ERR_PTR(-ENOMEM);

		snprintf(tbuf, sizeof(tbuf),
			 "cpr%d_init_voltage%d", tid, i + 1);
		fuses[i].init_voltage = devm_kstrdup(dev, tbuf,
						     GFP_KERNEL);
		if (!fuses[i].init_voltage)
			return ERR_PTR(-ENOMEM);

		snprintf(tbuf, sizeof(tbuf), "cpr%d_quotient%d", tid, i + 1);
		fuses[i].quotient = devm_kstrdup(dev, tbuf, GFP_KERNEL);
		if (!fuses[i].quotient)
			return ERR_PTR(-ENOMEM);

		snprintf(tbuf, sizeof(tbuf),
			 "cpr%d_quotient_offset%d", tid, i + 1);
		fuses[i].quotient_offset = devm_kstrdup(dev, tbuf,
							GFP_KERNEL);
		if (!fuses[i].quotient_offset)
			return ERR_PTR(-ENOMEM);
	}

	return fuses;
}

int cpr_populate_fuse_common(struct device *dev,
			     struct fuse_corner_data *fdata,
			     const struct cpr_fuse *cpr_fuse,
			     struct fuse_corner *fuse_corner,
			     int step_volt, int init_v_width,
			     int init_v_step)
{
	int uV, ret;

	/* Populate uV */
	uV = cpr_read_fuse_uV(init_v_width, init_v_step,
			      fdata->ref_uV, fdata->volt_oloop_adjust,
			      step_volt, cpr_fuse->init_voltage, dev);
	if (uV < 0)
		return uV;

	/*
	 * Update SoC voltages: platforms might choose a different
	 * regulators than the one used to characterize the algorithms
	 * (ie, init_voltage_step).
	 */
	fdata->min_uV = roundup(fdata->min_uV, step_volt);
	fdata->max_uV = roundup(fdata->max_uV, step_volt);

	fuse_corner->min_uV = fdata->min_uV;
	fuse_corner->max_uV = fdata->max_uV;
	fuse_corner->uV = clamp(uV, fuse_corner->min_uV, fuse_corner->max_uV);

	/* Populate target quotient by scaling */
	ret = cpr_read_efuse(dev, cpr_fuse->quotient, &fuse_corner->quot);
	if (ret)
		return ret;

	fuse_corner->quot *= fdata->quot_scale;
	fuse_corner->quot += fdata->quot_offset;
	fuse_corner->quot += fdata->quot_adjust;

	return 0;
}

/*
 * Returns: Index of the initial corner or negative number for error.
 */
int cpr_find_initial_corner(struct device *dev, struct clk *cpu_clk,
			    struct corner *corners, int num_corners)
{
	unsigned long rate;
	struct corner *iter, *corner;
	const struct corner *end;
	unsigned int ret = 0;

	if (!cpu_clk)
		return -EINVAL;

	end = &corners[num_corners - 1];
	rate = clk_get_rate(cpu_clk);

	/*
	 * Some bootloaders set a CPU clock frequency that is not defined
	 * in the OPP table. When running at an unlisted frequency,
	 * cpufreq_online() will change to the OPP which has the lowest
	 * frequency, at or above the unlisted frequency.
	 * Since cpufreq_online() always "rounds up" in the case of an
	 * unlisted frequency, this function always "rounds down" in case
	 * of an unlisted frequency. That way, when cpufreq_online()
	 * triggers the first ever call to cpr_set_performance_state(),
	 * it will correctly determine the direction as UP.
	 */
	for (iter = corners; iter <= end; iter++) {
		if (iter->freq > rate)
			break;
		ret++;
		if (iter->freq == rate) {
			corner = iter;
			break;
		}
		if (iter->freq < rate)
			corner = iter;
	}

	if (!corner) {
		dev_err(dev, "boot up corner not found\n");
		return -EINVAL;
	}

	dev_dbg(dev, "boot up perf state: %u\n", ret);

	return ret;
}

u32 cpr_get_fuse_corner(struct dev_pm_opp *opp, u32 tid)
{
	struct device_node *np;
	u32 fc;

	np = dev_pm_opp_get_of_node(opp);
	if (of_property_read_u32_index(np, "qcom,opp-fuse-level", tid, &fc)) {
		pr_debug("%s: missing 'qcom,opp-fuse-level' property\n",
			 __func__);
		fc = 0;
	}

	of_node_put(np);

	return fc;
}

unsigned long cpr_get_opp_hz_for_req(struct dev_pm_opp *ref,
				     struct device *cpu_dev)
{
	u64 rate = 0;
	struct device_node *ref_np;
	struct device_node *desc_np;
	struct device_node *child_np = NULL;
	struct device_node *child_req_np = NULL;

	desc_np = dev_pm_opp_of_get_opp_desc_node(cpu_dev);
	if (!desc_np)
		return 0;

	ref_np = dev_pm_opp_get_of_node(ref);
	if (!ref_np)
		goto out_ref;

	do {
		of_node_put(child_req_np);
		child_np = of_get_next_available_child(desc_np, child_np);
		child_req_np = of_parse_phandle(child_np, "required-opps", 0);
	} while (child_np && child_req_np != ref_np);

	if (child_np && child_req_np == ref_np)
		of_property_read_u64(child_np, "opp-hz", &rate);

	of_node_put(child_req_np);
	of_node_put(child_np);
	of_node_put(ref_np);
out_ref:
	of_node_put(desc_np);

	return (unsigned long) rate;
}

int cpr_calculate_scaling(const char *quot_offset,
			  struct device *dev,
			  const struct fuse_corner_data *fdata,
			  const struct corner *corner)
{
	u32 quot_diff = 0;
	unsigned long freq_diff;
	int scaling;
	const struct fuse_corner *fuse, *prev_fuse;
	int ret;

	fuse = corner->fuse_corner;
	prev_fuse = fuse - 1;

	if (quot_offset) {
		ret = cpr_read_efuse(dev, quot_offset, &quot_diff);
		if (ret)
			return ret;

		quot_diff *= fdata->quot_offset_scale;
		quot_diff += fdata->quot_offset_adjust;
	} else {
		quot_diff = fuse->quot - prev_fuse->quot;
	}

	freq_diff = fuse->max_freq - prev_fuse->max_freq;
	freq_diff /= 1000000; /* Convert to MHz */
	scaling = 1000 * quot_diff / freq_diff;
	return min(scaling, fdata->max_quot_scale);
}

int cpr_interpolate(const struct corner *corner, int step_volt,
		    const struct fuse_corner_data *fdata)
{
	unsigned long f_high, f_low, f_diff;
	int uV_high, uV_low, uV;
	u64 temp, temp_limit;
	const struct fuse_corner *fuse, *prev_fuse;

	fuse = corner->fuse_corner;
	prev_fuse = fuse - 1;

	f_high = fuse->max_freq;
	f_low = prev_fuse->max_freq;
	uV_high = fuse->uV;
	uV_low = prev_fuse->uV;
	f_diff = fuse->max_freq - corner->freq;

	/*
	 * Don't interpolate in the wrong direction. This could happen
	 * if the adjusted fuse voltage overlaps with the previous fuse's
	 * adjusted voltage.
	 */
	if (f_high <= f_low || uV_high <= uV_low || f_high <= corner->freq)
		return corner->uV;

	temp = f_diff * (uV_high - uV_low);
	do_div(temp, f_high - f_low);

	/*
	 * max_volt_scale has units of uV/MHz while freq values
	 * have units of Hz.  Divide by 1000000 to convert to.
	 */
	temp_limit = f_diff * fdata->max_volt_scale;
	do_div(temp_limit, 1000000);

	uV = uV_high - min(temp, temp_limit);
	return roundup(uV, step_volt);
}

int cpr_check_vreg_constraints(struct device *dev, struct regulator *vreg,
			       struct fuse_corner *f)
{
	int ret;

	ret = regulator_is_supported_voltage(vreg, f->min_uV, f->min_uV);
	if (!ret) {
		dev_err(dev, "min uV: %d not supported by regulator\n",
			f->min_uV);
		return -EINVAL;
	}

	ret = regulator_is_supported_voltage(vreg, f->max_uV, f->max_uV);
	if (!ret) {
		dev_err(dev, "max uV: %d not supported by regulator\n",
			f->max_uV);
		return -EINVAL;
	}

	return 0;
}
