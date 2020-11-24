/* SPDX-License-Identifier: GPL-2.0 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/regulator/consumer.h>

enum voltage_change_dir {
	NO_CHANGE,
	DOWN,
	UP,
};

struct fuse_corner_data {
	int ref_uV;
	int max_uV;
	int min_uV;
	int range_uV;
	/* fuse volt: closed/open loop */
	int volt_cloop_adjust;
	int volt_oloop_adjust;
	int max_volt_scale;
	int max_quot_scale;
	/* fuse quot */
	int quot_offset;
	int quot_scale;
	int quot_adjust;
	/* fuse quot_offset */
	int quot_offset_scale;
	int quot_offset_adjust;
};

struct cpr_fuse {
	char *ring_osc;
	char *init_voltage;
	char *quotient;
	char *quotient_offset;
};

struct fuse_corner {
	int min_uV;
	int max_uV;
	int uV;
	int quot;
	int step_quot;
	const struct reg_sequence *accs;
	int num_accs;
	unsigned long max_freq;
	u8 ring_osc_idx;
};

struct corner {
	int min_uV;
	int max_uV;
	int uV;
	int last_uV;
	int quot_adjust;
	u32 save_ctl;
	u32 save_irq;
	unsigned long freq;
	bool is_open_loop;
	struct fuse_corner *fuse_corner;
};

struct corner_data {
	unsigned int fuse_corner;
	unsigned long freq;
	int oloop_vadj;
	int cloop_vadj;
};

struct acc_desc {
	unsigned int	enable_reg;
	u32		enable_mask;

	struct reg_sequence	*config;
	struct reg_sequence	*settings;
	int			num_regs_per_fuse;
};

struct cpr_acc_desc {
	const struct cpr_desc *cpr_desc;
	const struct acc_desc *acc_desc;
};


int cpr_read_efuse(struct device *dev, const char *cname, u32 *data);
int cpr_populate_ring_osc_idx(struct device *dev,
			      struct fuse_corner *fuse_corner,
			      const struct cpr_fuse *cpr_fuse,
			      int num_fuse_corners);
int cpr_read_fuse_uV(int init_v_width, int step_size_uV, int ref_uV,
		     int adj, int step_volt, const char *init_v_efuse,
		     struct device *dev);
const struct cpr_fuse *cpr_get_fuses(struct device *dev, int tid,
				     int num_fuse_corners);
int cpr_populate_fuse_common(struct device *dev,
			     struct fuse_corner_data *fdata,
			     const struct cpr_fuse *cpr_fuse,
			     struct fuse_corner *fuse_corner,
			     int step_volt, int init_v_width,
			     int init_v_step);
int cpr_find_initial_corner(struct device *dev, struct clk *cpu_clk,
			    struct corner *corners, int num_corners);
u32 cpr_get_fuse_corner(struct dev_pm_opp *opp, u32 tid);
void cpr_get_corner_post_vadj(struct dev_pm_opp *opp, u32 tid,
			      s32 *open_loop, s32 *closed_loop);
unsigned long cpr_get_opp_hz_for_req(struct dev_pm_opp *ref,
				     struct device *cpu_dev);
int cpr_calculate_scaling(const char *quot_offset,
			  struct device *dev,
			  const struct fuse_corner_data *fdata,
			  const struct corner *corner);
int cpr_interpolate(const struct corner *corner, int step_volt,
		    const struct fuse_corner_data *fdata);
int cpr_check_vreg_constraints(struct device *dev, struct regulator *vreg,
			       struct fuse_corner *f);
