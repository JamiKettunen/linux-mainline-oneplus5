// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm MSM8998 Network-on-Chip (NoC) QoS driver
 * Copyright (c) 2020, AngeloGioacchino Del Regno
 *                     <angelogioacchino.delregno@somainline.org>
 * Copyright (C) 2020, Konrad Dybcio <konrad.dybcio@somainline.org>
 *
 */

#include <dt-bindings/interconnect/qcom,msm8998.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/interconnect-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include "smd-rpm.h"

#define RPM_BUS_MASTER_REQ	0x73616d62
#define RPM_BUS_SLAVE_REQ	0x766c7362

/* BIMC QoS */
#define M_BKE_REG_BASE(n)		(0x300 + (0x4000 * n))
#define M_BKE_EN_ADDR(n)		(M_BKE_REG_BASE(n))
#define M_BKE_HEALTH_CFG_ADDR(i, n)	(M_BKE_REG_BASE(n) + 0x40 + (0x4 * i))

#define M_BKE_HEALTH_CFG_LIMITCMDS_MASK	0x80000000
#define M_BKE_HEALTH_CFG_AREQPRIO_MASK	0x300
#define M_BKE_HEALTH_CFG_PRIOLVL_MASK	0x3
#define M_BKE_HEALTH_CFG_AREQPRIO_SHIFT	0x8
#define M_BKE_HEALTH_CFG_LIMITCMDS_SHIFT 0x1f

#define M_BKE_EN_EN_BMASK		0x1

/* Valid for both NoC and BIMC */
#define NOC_QOS_MODE_FIXED		0x0
#define NOC_QOS_MODE_LIMITER		0x1
#define NOC_QOS_MODE_BYPASS		0x2

/* NoC QoS */
#define NOC_PERM_MODE_FIXED		1
#define NOC_PERM_MODE_BYPASS		(1 << NOC_QOS_MODE_BYPASS)

#define NOC_QOS_PRIORITYn_ADDR(n)	(0x8 + (n * 0x1000))
#define NOC_QOS_PRIORITY_MASK		0xf
#define NOC_QOS_PRIORITY_P1_SHIFT	0x2
#define NOC_QOS_PRIORITY_P0_SHIFT	0x3

#define NOC_QOS_MODEn_ADDR(n)		(0xc + (n * 0x1000))
#define NOC_QOS_MODEn_MASK		0x3

enum {
	MSM8998_MASTER_IPA = 1,
	MSM8998_MASTER_CNOC_A2NOC,
	MSM8998_MASTER_SDCC_2,
	MSM8998_MASTER_SDCC_4,
	MSM8998_MASTER_BLSP_1,
	MSM8998_MASTER_BLSP_2,
	MSM8998_MASTER_UFS,
	MSM8998_MASTER_USB_HS,
	MSM8998_MASTER_USB3,
	MSM8998_MASTER_CRYPTO_C0,
	MSM8998_MASTER_GNOC_BIMC,
	MSM8998_MASTER_OXILI,
	MSM8998_MASTER_MNOC_BIMC,
	MSM8998_MASTER_SNOC_BIMC,
	MSM8998_MASTER_PIMEM,
	MSM8998_MASTER_SNOC_CNOC,
	MSM8998_MASTER_QDSS_DAP,
	MSM8998_MASTER_APPS_PROC,
	MSM8998_MASTER_CNOC_MNOC_MMSS_CFG,
	MSM8998_MASTER_CNOC_MNOC_CFG,
	MSM8998_MASTER_CPP,
	MSM8998_MASTER_JPEG,
	MSM8998_MASTER_MDP_P0,
	MSM8998_MASTER_MDP_P1,
	MSM8998_MASTER_VENUS,
	MSM8998_MASTER_VFE,
	MSM8998_MASTER_QDSS_ETR,
	MSM8998_MASTER_QDSS_BAM,
	MSM8998_MASTER_SNOC_CFG,
	MSM8998_MASTER_BIMC_SNOC,
	MSM8998_MASTER_A1NOC_SNOC,
	MSM8998_MASTER_A2NOC_SNOC,
	MSM8998_MASTER_GNOC_SNOC,
	MSM8998_MASTER_PCIE_0,
	MSM8998_MASTER_A2NOC_TSIF,
	MSM8998_MASTER_CRVIRT_A2NOC,
	MSM8998_MASTER_ROTATOR,
	MSM8998_MASTER_VENUS_VMEM,
	MSM8998_MASTER_HMSS,
	MSM8998_MASTER_BIMC_SNOC_0,
	MSM8998_MASTER_BIMC_SNOC_1,

	MSM8998_SLAVE_A1NOC_SNOC,
	MSM8998_SLAVE_A2NOC_SNOC,
	MSM8998_SLAVE_EBI,
	MSM8998_SLAVE_HMSS_L3,
	MSM8998_SLAVE_CNOC_A2NOC,
	MSM8998_SLAVE_MPM,
	MSM8998_SLAVE_PMIC_ARB,
	MSM8998_SLAVE_TLMM_NORTH,
	MSM8998_SLAVE_TCSR,
	MSM8998_SLAVE_PIMEM_CFG,
	MSM8998_SLAVE_IMEM_CFG,
	MSM8998_SLAVE_MESSAGE_RAM,
	MSM8998_SLAVE_GLM,
	MSM8998_SLAVE_BIMC_CFG,
	MSM8998_SLAVE_PRNG,
	MSM8998_SLAVE_SPDM,
	MSM8998_SLAVE_QDSS_CFG,
	MSM8998_SLAVE_CNOC_MNOC_CFG,
	MSM8998_SLAVE_SNOC_CFG,
	MSM8998_SLAVE_QM_CFG,
	MSM8998_SLAVE_CLK_CTL,
	MSM8998_SLAVE_MSS_CFG,
	MSM8998_SLAVE_UFS_CFG,
	MSM8998_SLAVE_A2NOC_CFG,
	MSM8998_SLAVE_A2NOC_SMMU_CFG,
	MSM8998_SLAVE_GPUSS_CFG,
	MSM8998_SLAVE_AHB2PHY,
	MSM8998_SLAVE_BLSP_1,
	MSM8998_SLAVE_SDCC_2,
	MSM8998_SLAVE_SDCC_4,
	MSM8998_SLAVE_BLSP_2,
	MSM8998_SLAVE_PDM,
	MSM8998_SLAVE_CNOC_MNOC_MMSS_CFG,
	MSM8998_SLAVE_USB_HS,
	MSM8998_SLAVE_USB3_0,
	MSM8998_SLAVE_SRVC_CNOC,
	MSM8998_SLAVE_GNOC_BIMC,
	MSM8998_SLAVE_GNOC_SNOC,
	MSM8998_SLAVE_CAMERA_CFG,
	MSM8998_SLAVE_CAMERA_THROTTLE_CFG,
	MSM8998_SLAVE_MISC_CFG,
	MSM8998_SLAVE_VENUS_THROTTLE_CFG,
	MSM8998_SLAVE_VENUS_CFG,
	MSM8998_SLAVE_MMSS_CLK_XPU_CFG,
	MSM8998_SLAVE_MMSS_CLK_CFG,
	MSM8998_SLAVE_MNOC_MPU_CFG,
	MSM8998_SLAVE_DISPLAY_CFG,
	MSM8998_SLAVE_CSI_PHY_CFG,
	MSM8998_SLAVE_DISPLAY_THROTTLE_CFG,
	MSM8998_SLAVE_SMMU_CFG,
	MSM8998_SLAVE_MNOC_BIMC,
	MSM8998_SLAVE_SRVC_MNOC,
	MSM8998_SLAVE_HMSS,
	MSM8998_SLAVE_LPASS,
	MSM8998_SLAVE_WLAN,
	MSM8998_SLAVE_CDSP,
	MSM8998_SLAVE_IPA,
	MSM8998_SLAVE_SNOC_BIMC,
	MSM8998_SLAVE_SNOC_CNOC,
	MSM8998_SLAVE_IMEM,
	MSM8998_SLAVE_PIMEM,
	MSM8998_SLAVE_QDSS_STM,
	MSM8998_SLAVE_SRVC_SNOC,
	MSM8998_SLAVE_BIMC_SNOC_0,
	MSM8998_SLAVE_BIMC_SNOC_1,
	MSM8998_SLAVE_SSC_CFG,
	MSM8998_SLAVE_SKL,
	MSM8998_SLAVE_TLMM_WEST,
	MSM8998_SLAVE_A1NOC_CFG,
	MSM8998_SLAVE_A1NOC_SMMU_CFG,
	MSM8998_SLAVE_TSIF,
	MSM8998_SLAVE_TLMM_EAST,
	MSM8998_SLAVE_CRVIRT_A2NOC,
	MSM8998_SLAVE_VMEM_CFG,
	MSM8998_SLAVE_VMEM,
	MSM8998_SLAVE_PCIE_0,
};

#define to_qcom_provider(_provider) \
	container_of(_provider, struct qcom_icc_provider, provider)

static const struct clk_bulk_data bus_clocks[] = {
	{ .id = "bus" },
	{ .id = "bus_a" },
};

static const struct clk_bulk_data bus_mm_clocks[] = {
	{ .id = "bus" },
	{ .id = "bus_a" },
	{ .id = "iface" },
};

/**
 * struct qcom_icc_provider - Qualcomm specific interconnect provider
 * @provider: generic interconnect provider
 * @bus_clks: the clk_bulk_data table of bus clocks
 * @num_clks: the total number of clk_bulk_data entries
 * @is_bimc_node: indicates whether to use bimc specific setting
 * @mmio: NoC base iospace
 */
struct qcom_icc_provider {
	struct icc_provider provider;
	struct clk_bulk_data *bus_clks;
	int num_clks;
	bool is_bimc_node;
	struct regmap *regmap;
	void __iomem *mmio;
};

#define MSM8998_MAX_LINKS	38

/**
 * struct qcom_icc_qos - Qualcomm specific interconnect QoS parameters
 * @areq_prio: node requests priority
 * @prio_level: priority level for bus communication
 * @limit_commands: activate/deactivate limiter mode during runtime
 * @ap_owned: indicates if the node is owned by the AP or by the RPM
 * @qos_mode: default qos mode for this node
 * @qos_port: qos port number for finding qos registers of this node
 */
struct qcom_icc_qos {
	u32 areq_prio;
	u32 prio_level;
	bool limit_commands;
	bool ap_owned;
	int qos_mode;
	int qos_port;
};

/**
 * struct qcom_icc_node - Qualcomm specific interconnect nodes
 * @name: the node name used in debugfs
 * @id: a unique node identifier
 * @links: an array of nodes where we can go next while traversing
 * @num_links: the total number of @links
 * @buswidth: width of the interconnect between a node and the bus (bytes)
 * @mas_rpm_id: RPM id for devices that are bus masters
 * @slv_rpm_id: RPM id for devices that are bus slaves
 * @qos: NoC QoS setting parameters
 * @rate: current bus clock rate in Hz
 */
struct qcom_icc_node {
	unsigned char *name;
	u16 id;
	u16 links[MSM8998_MAX_LINKS];
	u16 num_links;
	u16 buswidth;
	int mas_rpm_id;
	int slv_rpm_id;
	struct qcom_icc_qos qos;
	u64 rate;
};

struct qcom_icc_desc {
	struct qcom_icc_node **nodes;
	size_t num_nodes;
	const struct regmap_config *regmap_cfg;
};

#define DEFINE_QNODE(_name, _id, _buswidth, _mas_rpm_id, _slv_rpm_id,	\
		     _ap_owned, _qos_mode, _qos_prio, _qos_port, ...)	\
		static struct qcom_icc_node _name = {			\
		.name = #_name,						\
		.id = _id,						\
		.buswidth = _buswidth,					\
		.mas_rpm_id = _mas_rpm_id,				\
		.slv_rpm_id = _slv_rpm_id,				\
		.qos.ap_owned = _ap_owned,				\
		.qos.qos_mode = _qos_mode,				\
		.qos.areq_prio = _qos_prio,				\
		.qos.prio_level = _qos_prio,				\
		.qos.qos_port = _qos_port,				\
		.num_links = ARRAY_SIZE(((int[]){ __VA_ARGS__ })),	\
		.links = { __VA_ARGS__ },				\
	}

/* masters */
DEFINE_QNODE(mas_pcie_0, MSM8998_MASTER_PCIE_0, 16, 45, -1, true, NOC_QOS_MODE_FIXED, 1, 1, MSM8998_SLAVE_A1NOC_SNOC);
DEFINE_QNODE(mas_usb3, MSM8998_MASTER_USB3, 16, 32, -1, true, NOC_QOS_MODE_FIXED, 1, 2, MSM8998_SLAVE_A1NOC_SNOC);
DEFINE_QNODE(mas_ufs, MSM8998_MASTER_UFS, 16, 68, -1, true, NOC_QOS_MODE_FIXED, 1, 2, MSM8998_SLAVE_A1NOC_SNOC);
DEFINE_QNODE(mas_blsp_2, MSM8998_MASTER_BLSP_2, 16, 39, -1, false, NOC_QOS_MODE_FIXED, 0, 4, MSM8998_SLAVE_A1NOC_SNOC);
DEFINE_QNODE(mas_cnoc_a2noc, MSM8998_MASTER_CNOC_A2NOC, 8, 146, -1, true, -1, 0, -1, MSM8998_SLAVE_A2NOC_SNOC);
DEFINE_QNODE(mas_ipa, MSM8998_MASTER_IPA, 8, 59, -1, true, NOC_QOS_MODE_FIXED, 1, 1, MSM8998_SLAVE_A2NOC_SNOC);
DEFINE_QNODE(mas_sdcc_2, MSM8998_MASTER_SDCC_2, 8, 35, -1, false, NOC_QOS_MODE_FIXED, 0, 6, MSM8998_SLAVE_A2NOC_SNOC);
DEFINE_QNODE(mas_sdcc_4, MSM8998_MASTER_SDCC_4, 8, 36, -1, false, NOC_QOS_MODE_FIXED, 0, 7, MSM8998_SLAVE_A2NOC_SNOC);
DEFINE_QNODE(mas_tsif, MSM8998_MASTER_A2NOC_TSIF, 4, 37, -1, true, -1, 0, -1, MSM8998_SLAVE_A2NOC_SNOC);
DEFINE_QNODE(mas_blsp_1, MSM8998_MASTER_BLSP_1, 16, 41, -1, false, NOC_QOS_MODE_FIXED, 0, 8, MSM8998_SLAVE_A2NOC_SNOC);
DEFINE_QNODE(mas_crvirt_a2noc, MSM8998_MASTER_CRVIRT_A2NOC, 8, 145, -1, false, NOC_QOS_MODE_FIXED, 0, 9, MSM8998_SLAVE_A2NOC_SNOC);
DEFINE_QNODE(mas_gnoc_bimc, MSM8998_MASTER_GNOC_BIMC, 8, 144, -1, true, NOC_QOS_MODE_FIXED, 0, 0, MSM8998_SLAVE_EBI, MSM8998_SLAVE_BIMC_SNOC_0);
DEFINE_QNODE(mas_oxili, MSM8998_MASTER_OXILI, 8, 6, -1, true, NOC_QOS_MODE_BYPASS, 0, 1, MSM8998_SLAVE_BIMC_SNOC_1, MSM8998_SLAVE_HMSS_L3, MSM8998_SLAVE_EBI, MSM8998_SLAVE_BIMC_SNOC_0);
DEFINE_QNODE(mas_mnoc_bimc, MSM8998_MASTER_MNOC_BIMC, 8, 2, -1, true, NOC_QOS_MODE_BYPASS, 0, 2, MSM8998_SLAVE_BIMC_SNOC_1, MSM8998_SLAVE_HMSS_L3, MSM8998_SLAVE_EBI, MSM8998_SLAVE_BIMC_SNOC_0);
DEFINE_QNODE(mas_snoc_bimc, MSM8998_MASTER_SNOC_BIMC, 8, 3, -1, false, NOC_QOS_MODE_BYPASS, 0, 3, MSM8998_SLAVE_HMSS_L3, MSM8998_SLAVE_EBI);
DEFINE_QNODE(mas_snoc_cnoc, MSM8998_MASTER_SNOC_CNOC, 8, 52, -1, true, -1, 0, -1, MSM8998_SLAVE_SKL, MSM8998_SLAVE_BLSP_2, MSM8998_SLAVE_MESSAGE_RAM, MSM8998_SLAVE_TLMM_WEST, MSM8998_SLAVE_TSIF, MSM8998_SLAVE_MPM, MSM8998_SLAVE_BIMC_CFG, MSM8998_SLAVE_TLMM_EAST, MSM8998_SLAVE_SPDM, MSM8998_SLAVE_PIMEM_CFG, MSM8998_SLAVE_A1NOC_SMMU_CFG, MSM8998_SLAVE_BLSP_1, MSM8998_SLAVE_CLK_CTL, MSM8998_SLAVE_PRNG, MSM8998_SLAVE_USB3_0, MSM8998_SLAVE_QDSS_CFG, MSM8998_SLAVE_QM_CFG, MSM8998_SLAVE_A2NOC_CFG, MSM8998_SLAVE_PMIC_ARB, MSM8998_SLAVE_UFS_CFG, MSM8998_SLAVE_SRVC_CNOC, MSM8998_SLAVE_AHB2PHY, MSM8998_SLAVE_IPA, MSM8998_SLAVE_GLM, MSM8998_SLAVE_SNOC_CFG, MSM8998_SLAVE_SSC_CFG, MSM8998_SLAVE_SDCC_2, MSM8998_SLAVE_SDCC_4, MSM8998_SLAVE_PDM, MSM8998_SLAVE_CNOC_MNOC_MMSS_CFG, MSM8998_SLAVE_CNOC_MNOC_CFG, MSM8998_SLAVE_MSS_CFG, MSM8998_SLAVE_IMEM_CFG, MSM8998_SLAVE_A1NOC_CFG, MSM8998_SLAVE_GPUSS_CFG, MSM8998_SLAVE_TCSR, MSM8998_SLAVE_TLMM_NORTH);
DEFINE_QNODE(mas_qdss_dap, MSM8998_MASTER_QDSS_DAP, 8, 49, -1, true, -1, 0, -1, MSM8998_SLAVE_SKL, MSM8998_SLAVE_BLSP_2, MSM8998_SLAVE_MESSAGE_RAM, MSM8998_SLAVE_TLMM_WEST, MSM8998_SLAVE_TSIF, MSM8998_SLAVE_MPM, MSM8998_SLAVE_BIMC_CFG, MSM8998_SLAVE_TLMM_EAST, MSM8998_SLAVE_SPDM, MSM8998_SLAVE_PIMEM_CFG, MSM8998_SLAVE_A1NOC_SMMU_CFG, MSM8998_SLAVE_BLSP_1, MSM8998_SLAVE_CLK_CTL, MSM8998_SLAVE_PRNG, MSM8998_SLAVE_USB3_0, MSM8998_SLAVE_QDSS_CFG, MSM8998_SLAVE_QM_CFG, MSM8998_SLAVE_A2NOC_CFG, MSM8998_SLAVE_PMIC_ARB, MSM8998_SLAVE_UFS_CFG, MSM8998_SLAVE_SRVC_CNOC, MSM8998_SLAVE_AHB2PHY, MSM8998_SLAVE_IPA, MSM8998_SLAVE_GLM, MSM8998_SLAVE_SNOC_CFG, MSM8998_SLAVE_SDCC_2, MSM8998_SLAVE_SDCC_4, MSM8998_SLAVE_PDM, MSM8998_SLAVE_CNOC_MNOC_MMSS_CFG, MSM8998_SLAVE_CNOC_MNOC_CFG, MSM8998_SLAVE_MSS_CFG, MSM8998_SLAVE_IMEM_CFG, MSM8998_SLAVE_A1NOC_CFG, MSM8998_SLAVE_GPUSS_CFG, MSM8998_SLAVE_SSC_CFG, MSM8998_SLAVE_TCSR, MSM8998_SLAVE_TLMM_NORTH, MSM8998_SLAVE_CNOC_A2NOC);
DEFINE_QNODE(mas_crypto, MSM8998_MASTER_CRYPTO_C0, 650, 23, -1, false, -1, 0, -1, MSM8998_MASTER_CRVIRT_A2NOC);
DEFINE_QNODE(mas_apss_proc, MSM8998_MASTER_APPS_PROC, 32, 0, -1, true, -1, 0, -1, MSM8998_SLAVE_GNOC_BIMC);
DEFINE_QNODE(mas_cnoc_mnoc_mmss_cfg, MSM8998_MASTER_CNOC_MNOC_MMSS_CFG, 8, 4, -1, true, -1, 0, -1, MSM8998_SLAVE_CAMERA_THROTTLE_CFG, MSM8998_SLAVE_VENUS_CFG, MSM8998_SLAVE_MISC_CFG, MSM8998_SLAVE_CAMERA_CFG, MSM8998_SLAVE_DISPLAY_THROTTLE_CFG, MSM8998_SLAVE_VENUS_THROTTLE_CFG, MSM8998_SLAVE_DISPLAY_CFG, MSM8998_SLAVE_MMSS_CLK_CFG, MSM8998_SLAVE_VMEM_CFG, MSM8998_SLAVE_MMSS_CLK_XPU_CFG, MSM8998_SLAVE_SMMU_CFG);
DEFINE_QNODE(mas_cnoc_mnoc_cfg, MSM8998_MASTER_CNOC_MNOC_CFG, 8, 5, -1, true, -1, 0, -1, MSM8998_SLAVE_SRVC_MNOC);
DEFINE_QNODE(mas_cpp, MSM8998_MASTER_CPP, 32, 115, -1, true, NOC_QOS_MODE_BYPASS, 0, 5, MSM8998_SLAVE_MNOC_BIMC);
DEFINE_QNODE(mas_jpeg, MSM8998_MASTER_JPEG, 32, 7, -1, true, NOC_QOS_MODE_BYPASS, 0, 7, MSM8998_SLAVE_MNOC_BIMC);
DEFINE_QNODE(mas_mdp_p0, MSM8998_MASTER_MDP_P0, 32, 8, -1, true, NOC_QOS_MODE_BYPASS, 0, 1, MSM8998_SLAVE_MNOC_BIMC); /* vrail-comp???? */
DEFINE_QNODE(mas_mdp_p1, MSM8998_MASTER_MDP_P1, 32, 61, -1, true, NOC_QOS_MODE_BYPASS, 0, 2, MSM8998_SLAVE_MNOC_BIMC); /* vrail-comp??? */
DEFINE_QNODE(mas_rotator, MSM8998_MASTER_ROTATOR, 32, 120, -1, true, NOC_QOS_MODE_BYPASS, 0, 0, MSM8998_SLAVE_MNOC_BIMC);
DEFINE_QNODE(mas_venus, MSM8998_MASTER_VENUS, 32, 9, -1, true, NOC_QOS_MODE_BYPASS, 0, 3, MSM8998_SLAVE_MNOC_BIMC);
DEFINE_QNODE(mas_vfe, MSM8998_MASTER_VFE, 32, 11, -1, true, NOC_QOS_MODE_BYPASS, 0, 6, MSM8998_SLAVE_MNOC_BIMC);
DEFINE_QNODE(mas_venus_vmem, MSM8998_MASTER_VENUS_VMEM, 32, 121, -1, true, -1, 0, -1, MSM8998_SLAVE_VMEM);
DEFINE_QNODE(mas_hmss, MSM8998_MASTER_HMSS, 16, 118, -1, true, NOC_QOS_MODE_FIXED, 1, 3, MSM8998_SLAVE_PIMEM, MSM8998_SLAVE_IMEM, MSM8998_SLAVE_SNOC_BIMC);
DEFINE_QNODE(mas_qdss_bam, MSM8998_MASTER_QDSS_BAM, 16, 19, -1, true, NOC_QOS_MODE_FIXED, 1, 1, MSM8998_SLAVE_IMEM, MSM8998_SLAVE_PIMEM, MSM8998_SLAVE_SNOC_CNOC, MSM8998_SLAVE_SNOC_BIMC);
DEFINE_QNODE(mas_snoc_cfg, MSM8998_MASTER_SNOC_CFG, 16, 20, -1, false, -1, 0, -1, MSM8998_SLAVE_SRVC_SNOC);
DEFINE_QNODE(mas_bimc_snoc_0, MSM8998_MASTER_BIMC_SNOC_0, 16, 21, -1, false, -1, 0, -1, MSM8998_SLAVE_PIMEM, MSM8998_SLAVE_LPASS, MSM8998_SLAVE_HMSS, MSM8998_SLAVE_WLAN, MSM8998_SLAVE_SNOC_CNOC, MSM8998_SLAVE_IMEM, MSM8998_SLAVE_QDSS_STM);
DEFINE_QNODE(mas_bimc_snoc_1, MSM8998_MASTER_BIMC_SNOC_1, 16, 109, -1, true, -1, 0, -1, MSM8998_SLAVE_PCIE_0);
DEFINE_QNODE(mas_a1noc_snoc, MSM8998_MASTER_A1NOC_SNOC, 16, 111, -1, false, -1, 0, -1, MSM8998_SLAVE_PIMEM, MSM8998_SLAVE_PCIE_0, MSM8998_SLAVE_LPASS, MSM8998_SLAVE_HMSS, MSM8998_SLAVE_SNOC_BIMC, MSM8998_SLAVE_SNOC_CNOC, MSM8998_SLAVE_IMEM, MSM8998_SLAVE_QDSS_STM);
DEFINE_QNODE(mas_a2noc_snoc, MSM8998_MASTER_A2NOC_SNOC, 16, 112, -1, false, -1, 0, -1, MSM8998_SLAVE_PIMEM, MSM8998_SLAVE_PCIE_0, MSM8998_SLAVE_LPASS, MSM8998_SLAVE_HMSS, MSM8998_SLAVE_SNOC_BIMC, MSM8998_SLAVE_WLAN, MSM8998_SLAVE_SNOC_CNOC, MSM8998_SLAVE_IMEM, MSM8998_SLAVE_QDSS_STM);
DEFINE_QNODE(mas_qdss_etr, MSM8998_MASTER_QDSS_ETR, 16, 31, -1, true, NOC_QOS_MODE_FIXED, 1, 2, MSM8998_SLAVE_IMEM, MSM8998_MASTER_PIMEM, MSM8998_SLAVE_SNOC_CNOC, MSM8998_SLAVE_SNOC_BIMC);

/* slaves */
DEFINE_QNODE(slv_a1noc_snoc, MSM8998_SLAVE_A1NOC_SNOC, 16, -1, 142, false, -1, 0, -1, MSM8998_MASTER_A1NOC_SNOC);
DEFINE_QNODE(slv_a2noc_snoc, MSM8998_SLAVE_A2NOC_SNOC, 16, -1, 143, false, -1, 0, -1, MSM8998_MASTER_A2NOC_SNOC);
DEFINE_QNODE(slv_ebi, MSM8998_SLAVE_EBI, 8, -1, 0, false, -1, 0, -1, 0);
DEFINE_QNODE(slv_hmss_l3, MSM8998_SLAVE_HMSS_L3, 8, -1, 160, false, -1, 0, -1, 0);
DEFINE_QNODE(slv_bimc_snoc_0, MSM8998_SLAVE_BIMC_SNOC_0, 8, -1, 2, false, -1, 0, -1, MSM8998_MASTER_BIMC_SNOC_0);
DEFINE_QNODE(slv_bimc_snoc_1, MSM8998_SLAVE_BIMC_SNOC_1, 8, -1, 138, true, -1, 0, -1, MSM8998_MASTER_BIMC_SNOC_1);
DEFINE_QNODE(slv_cnoc_a2noc, MSM8998_SLAVE_CNOC_A2NOC, 4, -1, 208, true, -1, 0, -1, MSM8998_MASTER_CNOC_A2NOC);
DEFINE_QNODE(slv_ssc_cfg, MSM8998_SLAVE_SSC_CFG, 4, -1, 177, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_mpm, MSM8998_SLAVE_MPM, 4, -1, 62, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_pmic_arb, MSM8998_SLAVE_PMIC_ARB, 4, -1, 59, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_tlmm_north, MSM8998_SLAVE_TLMM_NORTH, 4, -1, 214, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_pimem_cfg, MSM8998_SLAVE_PIMEM_CFG, 4, -1, 167, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_imem_cfg, MSM8998_SLAVE_IMEM_CFG, 4, -1, 54, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_message_ram, MSM8998_SLAVE_MESSAGE_RAM, 4, -1, 55, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_skl, MSM8998_SLAVE_SKL, 4, -1, 196, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_bimc_cfg, MSM8998_SLAVE_BIMC_CFG, 4, -1, 56, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_prng, MSM8998_SLAVE_PRNG, 4, -1, 44, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_a2noc_cfg, MSM8998_SLAVE_A2NOC_CFG, 4, -1, 150, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_ipa, MSM8998_SLAVE_IPA, 4, -1, 183, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_tcsr, MSM8998_SLAVE_TCSR, 4, -1, 50, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_snoc_cfg, MSM8998_SLAVE_SNOC_CFG, 4, -1, 70, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_clk_ctl, MSM8998_SLAVE_CLK_CTL, 4, -1, 47, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_glm, MSM8998_SLAVE_GLM, 4, -1, 209, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_spdm, MSM8998_SLAVE_SPDM, 4, -1, 60, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_gpuss_cfg, MSM8998_SLAVE_GPUSS_CFG, 4, -1, 11, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_cnoc_mnoc_cfg, MSM8998_SLAVE_BLSP_1, 4, -1, 66, true, -1, 0, -1, MSM8998_MASTER_CNOC_MNOC_CFG);
DEFINE_QNODE(slv_qm_cfg, MSM8998_SLAVE_QM_CFG, 4, -1, 212, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_mss_cfg, MSM8998_SLAVE_MSS_CFG, 4, -1, 48, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_ufs_cfg, MSM8998_SLAVE_UFS_CFG, 4, -1, 92, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_tlmm_west, MSM8998_SLAVE_TLMM_WEST, 4, -1, 215, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_a1noc_cfg, MSM8998_SLAVE_A1NOC_CFG, 4, -1, 147, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_ahb2phy, MSM8998_SLAVE_AHB2PHY, 4, -1, 163, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_blsp_2, MSM8998_SLAVE_BLSP_2, 4, -1, 37, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_pdm, MSM8998_SLAVE_PDM, 4, -1, 41, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_usb3_0, MSM8998_SLAVE_USB3_0, 4, -1, 22, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_a1noc_smmu_cfg, MSM8998_SLAVE_A1NOC_SMMU_CFG, 8, -1, 149, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_blsp_1, MSM8998_SLAVE_BLSP_1, 4, -1, 39, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_sdcc_2, MSM8998_SLAVE_SDCC_2, 4, -1, 33, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_sdcc_4, MSM8998_SLAVE_SDCC_4, 4, -1, 34, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_tsif, MSM8998_SLAVE_TSIF, 4, -1, 35, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_qdss_cfg, MSM8998_SLAVE_QDSS_CFG, 4, -1, 63, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_tlmm_east, MSM8998_SLAVE_TLMM_EAST, 4, -1, 213, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_cnoc_mnoc_mmss_cfg, MSM8998_SLAVE_CNOC_MNOC_MMSS_CFG, 8, -1, 58, true, -1, 0, -1, MSM8998_MASTER_CNOC_MNOC_MMSS_CFG);
DEFINE_QNODE(slv_srvc_cnoc, MSM8998_SLAVE_SRVC_CNOC, 4, -1, 76, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_crvirt_a2noc, MSM8998_SLAVE_CRVIRT_A2NOC, 8, -1, 207, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_gnoc_bimc, MSM8998_SLAVE_GNOC_BIMC, 32, -1, 210, true, -1, 0, -1, MSM8998_MASTER_GNOC_BIMC);
DEFINE_QNODE(slv_camera_cfg, MSM8998_SLAVE_CAMERA_CFG, 8, -1, 3, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_camera_throttle_cfg, MSM8998_SLAVE_CAMERA_THROTTLE_CFG, 8, -1, 154, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_misc_cfg, MSM8998_SLAVE_MISC_CFG, 8, -1, 8, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_venus_throttle_cfg, MSM8998_SLAVE_VENUS_THROTTLE_CFG, 8, -1, 178, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_venus_cfg, MSM8998_SLAVE_VENUS_CFG, 8, -1, 10, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_vmem_cfg, MSM8998_SLAVE_VMEM_CFG, 8, -1, 180, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_mmss_clk_xpu_cfg, MSM8998_SLAVE_MMSS_CLK_XPU_CFG, 8, -1, 13, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_mmss_clk_cfg, MSM8998_SLAVE_MMSS_CLK_CFG, 8, -1, 12, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_display_cfg, MSM8998_SLAVE_DISPLAY_CFG, 8, -1, 4, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_display_throttle_cfg, MSM8998_SLAVE_DISPLAY_THROTTLE_CFG, 4, -1, 156, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_smmu_cfg, MSM8998_SLAVE_SMMU_CFG, 8, -1, 205, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_mnoc_bimc, MSM8998_SLAVE_MNOC_BIMC, 32, -1, 16, true, -1, 0, -1, MSM8998_MASTER_MNOC_BIMC);
DEFINE_QNODE(slv_vmem, MSM8998_SLAVE_VMEM, 32, -1, 179, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_srvc_mnoc, MSM8998_SLAVE_SRVC_MNOC, 8, -1, 17, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_hmss, MSM8998_SLAVE_HMSS, 16, -1, 20, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_lpass, MSM8998_SLAVE_LPASS, 16, -1, 21, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_wlan, MSM8998_SLAVE_WLAN, 16, -1, 206, false, -1, 0, -1, 0);
DEFINE_QNODE(slv_snoc_bimc, MSM8998_SLAVE_SNOC_BIMC, 32, -1, 24, false, -1, 0, -1, MSM8998_MASTER_SNOC_BIMC);
DEFINE_QNODE(slv_snoc_cnoc, MSM8998_SLAVE_SNOC_CNOC, 16, -1, 25, false, -1, 0, -1, MSM8998_MASTER_SNOC_CNOC);
DEFINE_QNODE(slv_imem, MSM8998_SLAVE_IMEM, 16, -1, 26, false, -1, 0, -1, 0);
DEFINE_QNODE(slv_pimem, MSM8998_SLAVE_PIMEM, 16, -1, 166, false, -1, 0, -1, 0);
DEFINE_QNODE(slv_qdss_stm, MSM8998_SLAVE_QDSS_STM, 16, -1, 30, false, -1, 0, -1, 0);
DEFINE_QNODE(slv_pcie_0, MSM8998_SLAVE_PCIE_0, 16, -1, 84, true, -1, 0, -1, 0);
DEFINE_QNODE(slv_srvc_snoc, MSM8998_SLAVE_SRVC_SNOC, 16, -1, 29, false, -1, 0, -1, 0);

static struct qcom_icc_node *msm8998_a1noc_nodes[] = {
	[MASTER_PCIE_0] = &mas_pcie_0,
	[MASTER_USB3] = &mas_usb3,
	[MASTER_UFS] = &mas_ufs,
	[MASTER_BLSP_2] = &mas_blsp_2,
	[SLAVE_A1NOC_SNOC] = &slv_a1noc_snoc,
};

static const struct regmap_config msm8998_a1noc_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0x60000,
	.fast_io	= true,
};

static struct qcom_icc_desc msm8998_a1noc = {
	.nodes = msm8998_a1noc_nodes,
	.num_nodes = ARRAY_SIZE(msm8998_a1noc_nodes),
	.regmap_cfg = &msm8998_a1noc_regmap_config,
};

static struct qcom_icc_node *msm8998_a2noc_nodes[] = {
	[MASTER_IPA] = &mas_ipa,
	[MASTER_CNOC_A2NOC] = &mas_cnoc_a2noc,
	[MASTER_SDCC_2] = &mas_sdcc_2,
	[MASTER_SDCC_4] = &mas_sdcc_4,
	[MASTER_TSIF] = &mas_tsif,
	[MASTER_BLSP_1] = &mas_blsp_1,
	[MASTER_CRVIRT_A2NOC] = &mas_crvirt_a2noc,
	[MASTER_CRYPTO_C0] = &mas_crypto,
	[SLAVE_A2NOC_SNOC] = &slv_a2noc_snoc,
	[SLAVE_CRVIRT_A2NOC] = &slv_crvirt_a2noc,
};

static const struct regmap_config msm8998_a2noc_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0x60000,
	.fast_io	= true,
};

static struct qcom_icc_desc msm8998_a2noc = {
	.nodes = msm8998_a2noc_nodes,
	.num_nodes = ARRAY_SIZE(msm8998_a2noc_nodes),
	.regmap_cfg = &msm8998_a2noc_regmap_config,
};

static struct qcom_icc_node *msm8998_bimc_nodes[] = {
	[MASTER_GNOC_BIMC] = &mas_gnoc_bimc,
	[MASTER_OXILI] = &mas_oxili,
	[MASTER_MNOC_BIMC] = &mas_mnoc_bimc,
	[MASTER_SNOC_BIMC] = &mas_snoc_bimc,
	[SLAVE_EBI] = &slv_ebi,
	[SLAVE_HMSS_L3] = &slv_hmss_l3,
	[SLAVE_BIMC_SNOC_0] = &slv_bimc_snoc_0,
	[SLAVE_BIMC_SNOC_1] = &slv_bimc_snoc_1,
};

static const struct regmap_config msm8998_bimc_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0x80000,
	.fast_io	= true,
};

static struct qcom_icc_desc msm8998_bimc = {
	.nodes = msm8998_bimc_nodes,
	.num_nodes = ARRAY_SIZE(msm8998_bimc_nodes),
	.regmap_cfg = &msm8998_bimc_regmap_config,
};

static struct qcom_icc_node *msm8998_cnoc_nodes[] = {
	[MASTER_SNOC_CNOC] = &mas_snoc_cnoc,
	[MASTER_QDSS_DAP] = &mas_qdss_dap,
	[SLAVE_CNOC_A2NOC] = &slv_cnoc_a2noc,
	[SLAVE_SSC_CFG] = &slv_ssc_cfg,
	[SLAVE_MPM] = &slv_mpm,
	[SLAVE_PMIC_ARB] = &slv_pmic_arb,
	[SLAVE_TLMM_NORTH] = &slv_tlmm_north,
	[SLAVE_PIMEM_CFG] = &slv_pimem_cfg,
	[SLAVE_IMEM_CFG] = &slv_imem_cfg,
	[SLAVE_MESSAGE_RAM] = &slv_message_ram,
	[SLAVE_SKL] = &slv_skl,
	[SLAVE_BIMC_CFG] = &slv_bimc_cfg,
	[SLAVE_PRNG] = &slv_prng,
	[SLAVE_A2NOC_CFG] = &slv_a2noc_cfg,
	[SLAVE_IPA] = &slv_ipa,
	[SLAVE_TCSR] = &slv_tcsr,
	[SLAVE_SNOC_CFG] = &slv_snoc_cfg,
	[SLAVE_CLK_CTL] = &slv_clk_ctl,
	[SLAVE_GLM] = &slv_glm,
	[SLAVE_SPDM] = &slv_spdm,
	[SLAVE_GPUSS_CFG] = &slv_gpuss_cfg,
	[SLAVE_CNOC_MNOC_CFG] = &slv_cnoc_mnoc_cfg,
	[SLAVE_QM_CFG] = &slv_qm_cfg,
	[SLAVE_MSS_CFG] = &slv_mss_cfg,
	[SLAVE_UFS_CFG] = &slv_ufs_cfg,
	[SLAVE_TLMM_WEST] = &slv_tlmm_west,
	[SLAVE_A1NOC_CFG] = &slv_a1noc_cfg,
	[SLAVE_AHB2PHY] = &slv_ahb2phy,
	[SLAVE_BLSP_2] = &slv_blsp_2,
	[SLAVE_PDM] = &slv_pdm,
	[SLAVE_USB3_0] = &slv_usb3_0,
	[SLAVE_A1NOC_SMMU_CFG] = &slv_a1noc_smmu_cfg,
	[SLAVE_BLSP_1] = &slv_blsp_1,
	[SLAVE_SDCC_2] = &slv_sdcc_2,
	[SLAVE_SDCC_4] = &slv_sdcc_4,
	[SLAVE_TSIF] = &slv_tsif,
	[SLAVE_QDSS_CFG] = &slv_qdss_cfg,
	[SLAVE_TLMM_EAST] = &slv_tlmm_east,
	[SLAVE_CNOC_MNOC_MMSS_CFG] = &slv_cnoc_mnoc_mmss_cfg,
	[SLAVE_SRVC_CNOC] = &slv_srvc_cnoc,
};

static const struct regmap_config msm8998_cnoc_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0x10000,
	.fast_io	= true,
};

static struct qcom_icc_desc msm8998_cnoc = {
	.nodes = msm8998_cnoc_nodes,
	.num_nodes = ARRAY_SIZE(msm8998_cnoc_nodes),
	.regmap_cfg = &msm8998_cnoc_regmap_config,
};

static struct qcom_icc_node *msm8998_gnoc_nodes[] = {
	[MASTER_APSS_PROC] = &mas_apss_proc,
	[SLAVE_GNOC_BIMC] = &slv_gnoc_bimc,
};

static const struct regmap_config msm8998_gnoc_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0x10000,
	.fast_io	= true,
};

static struct qcom_icc_desc msm8998_gnoc = {
	.nodes = msm8998_gnoc_nodes,
	.num_nodes = ARRAY_SIZE(msm8998_gnoc_nodes),
	.regmap_cfg = &msm8998_gnoc_regmap_config,
};

static struct qcom_icc_node *msm8998_mnoc_nodes[] = {
	[MASTER_CNOC_MNOC_CFG] = &mas_cnoc_mnoc_cfg,
	[MASTER_CPP] = &mas_cpp,
	[MASTER_JPEG] = &mas_jpeg,
	[MASTER_MDP_P0] = &mas_mdp_p0,
	[MASTER_MDP_P1] = &mas_mdp_p1,
	[MASTER_ROTATOR] = &mas_rotator,
	[MASTER_VENUS] = &mas_venus,
	[MASTER_VFE] = &mas_vfe,
	[MASTER_VENUS_VMEM] = &mas_venus_vmem,
	[SLAVE_MNOC_BIMC] = &slv_mnoc_bimc,
	[SLAVE_VMEM] = &slv_vmem,
	[SLAVE_SRVC_MNOC] = &slv_srvc_mnoc,
	[MASTER_CNOC_MNOC_MMSS_CFG] = &mas_cnoc_mnoc_mmss_cfg,
	[SLAVE_CAMERA_CFG] = &slv_camera_cfg,
	[SLAVE_CAMERA_THROTTLE_CFG] = &slv_camera_throttle_cfg,
	[SLAVE_MISC_CFG] = &slv_misc_cfg,
	[SLAVE_VENUS_THROTTLE_CFG] = &slv_venus_throttle_cfg,
	[SLAVE_VENUS_CFG] = &slv_venus_cfg,
	[SLAVE_VMEM_CFG] = &slv_vmem_cfg,
	[SLAVE_MMSS_CLK_XPU_CFG] = &slv_mmss_clk_xpu_cfg,
	[SLAVE_MMSS_CLK_CFG] = &slv_mmss_clk_cfg,
	[SLAVE_DISPLAY_CFG] = &slv_display_cfg,
	[SLAVE_DISPLAY_THROTTLE_CFG] = &slv_display_throttle_cfg,
	[SLAVE_SMMU_CFG] = &slv_smmu_cfg,
};

static const struct regmap_config msm8998_mnoc_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0x10000,
	.fast_io	= true,
};

static struct qcom_icc_desc msm8998_mnoc = {
	.nodes = msm8998_mnoc_nodes,
	.num_nodes = ARRAY_SIZE(msm8998_mnoc_nodes),
	.regmap_cfg = &msm8998_mnoc_regmap_config,
};

static struct qcom_icc_node *msm8998_snoc_nodes[] = {
	[MASTER_HMSS] = &mas_hmss,
	[MASTER_QDSS_BAM] = &mas_qdss_bam,
	[MASTER_SNOC_CFG] = &mas_snoc_cfg,
	[MASTER_BIMC_SNOC_0] = &mas_bimc_snoc_0,
	[MASTER_BIMC_SNOC_1] = &mas_bimc_snoc_1,
	[MASTER_A1NOC_SNOC] = &mas_a1noc_snoc,
	[MASTER_A2NOC_SNOC] = &mas_a2noc_snoc,
	[MASTER_QDSS_ETR] = &mas_qdss_etr,
	[SLAVE_HMSS] = &slv_hmss,
	[SLAVE_LPASS] = &slv_lpass,
	[SLAVE_WLAN] = &slv_wlan,
	[SLAVE_SNOC_BIMC] = &slv_snoc_bimc,
	[SLAVE_SNOC_CNOC] = &slv_snoc_cnoc,
	[SLAVE_IMEM] = &slv_imem,
	[SLAVE_PIMEM] = &slv_pimem,
	[SLAVE_QDSS_STM] = &slv_qdss_stm,
	[SLAVE_PCIE_0] = &slv_pcie_0,
	[SLAVE_SRVC_SNOC] = &slv_srvc_snoc,
};

static const struct regmap_config msm8998_snoc_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0x40000,
	.fast_io	= true,
};

static struct qcom_icc_desc msm8998_snoc = {
	.nodes = msm8998_snoc_nodes,
	.num_nodes = ARRAY_SIZE(msm8998_snoc_nodes),
	.regmap_cfg = &msm8998_snoc_regmap_config,
};

static int qcom_icc_bimc_set_qos_health(struct regmap *rmap,
					struct qcom_icc_qos *qos,
					int regnum)
{
	u32 val;
	u32 mask;

	val = qos->prio_level;
	mask = M_BKE_HEALTH_CFG_PRIOLVL_MASK;

	val |= qos->areq_prio << M_BKE_HEALTH_CFG_AREQPRIO_SHIFT;
	mask |= M_BKE_HEALTH_CFG_AREQPRIO_MASK;

	/* LIMITCMDS is not present on M_BKE_HEALTH_3 */
	if (regnum != 3) {
		val |= qos->limit_commands << M_BKE_HEALTH_CFG_LIMITCMDS_SHIFT;
		mask |= M_BKE_HEALTH_CFG_LIMITCMDS_MASK;
	}

	return regmap_update_bits(rmap,
				  M_BKE_HEALTH_CFG_ADDR(regnum, qos->qos_port),
				  mask, val);
}

static int qcom_icc_set_bimc_qos(struct icc_node *src, u64 max_bw,
				 bool bypass_mode)
{
	struct qcom_icc_provider *qp;
	struct qcom_icc_node *qn;
	struct icc_provider *provider;
	u32 mode = NOC_QOS_MODE_BYPASS;
	u32 val = 0;
	int i, rc = 0;

	qn = src->data;
	provider = src->provider;
	qp = to_qcom_provider(provider);

	if (qn->qos.qos_mode != -1)
		mode = qn->qos.qos_mode;

	/*
	 * QoS Priority: The QoS Health parameters are getting considered
	 * only if we are NOT in Bypass Mode.
	 */
	if (mode != NOC_QOS_MODE_BYPASS) {
		for (i = 3; i >= 0; i--) {
			rc = qcom_icc_bimc_set_qos_health(qp->regmap,
							  &qn->qos, i);
			if (rc)
				return rc;
		}

		/* Set BKE_EN to 1 when Fixed, Regulator or Limiter Mode */
		val = 1;
	}

	return regmap_update_bits(qp->regmap, M_BKE_EN_ADDR(qn->qos.qos_port),
				  M_BKE_EN_EN_BMASK, val);
}

static int qcom_icc_noc_set_qos_priority(struct regmap *rmap,
					 struct qcom_icc_qos *qos)
{
	u32 val;
	int rc;

	/* Must be updated one at a time, P1 first, P0 last */
	val = qos->areq_prio << NOC_QOS_PRIORITY_P1_SHIFT;
	rc = regmap_update_bits(rmap, NOC_QOS_PRIORITYn_ADDR(qos->qos_port),
				NOC_QOS_PRIORITY_MASK, val);
	if (rc)
		return rc;

	val = qos->prio_level << NOC_QOS_PRIORITY_P0_SHIFT;
	return regmap_update_bits(rmap, NOC_QOS_PRIORITYn_ADDR(qos->qos_port),
				  NOC_QOS_PRIORITY_MASK, val);
}

static int qcom_icc_set_noc_qos(struct icc_node *src, u64 max_bw)
{
	struct qcom_icc_provider *qp;
	struct qcom_icc_node *qn;
	struct icc_provider *provider;
	u32 mode = NOC_QOS_MODE_BYPASS;
	int rc = 0;

	qn = src->data;
	provider = src->provider;
	qp = to_qcom_provider(provider);

	if (qn->qos.qos_port < 0) {
		dev_dbg(src->provider->dev,
			"NoC QoS: Skipping %s: vote aggregated on parent.\n",
			qn->name);
		return 0;
	}

	if (qn->qos.qos_mode != -1)
		mode = qn->qos.qos_mode;

	if (mode == NOC_QOS_MODE_FIXED) {
		dev_dbg(src->provider->dev, "NoC QoS: %s: Set Fixed mode\n",
			qn->name);
		rc = qcom_icc_noc_set_qos_priority(qp->regmap, &qn->qos);
		if (rc)
			return rc;
	} else if (mode == NOC_QOS_MODE_BYPASS) {
		dev_dbg(src->provider->dev, "NoC QoS: %s: Set Bypass mode\n",
			qn->name);
	}

	return regmap_update_bits(qp->regmap,
				  NOC_QOS_MODEn_ADDR(qn->qos.qos_port),
				  NOC_QOS_MODEn_MASK, mode);
}

static int qcom_icc_qos_set(struct icc_node *node, u64 sum_bw)
{
	struct qcom_icc_provider *qp = to_qcom_provider(node->provider);
	struct qcom_icc_node *qn = node->data;

	dev_dbg(node->provider->dev, "Setting QoS for %s\n", qn->name);

	if (qp->is_bimc_node)
		return qcom_icc_set_bimc_qos(node, sum_bw,
				(qn->qos.qos_mode == NOC_QOS_MODE_BYPASS));

	return qcom_icc_set_noc_qos(node, sum_bw);
}

static int qcom_icc_rpm_set(int mas_rpm_id, int slv_rpm_id, u64 sum_bw)
{
	int ret = 0;

	if (mas_rpm_id != -1) {
		ret = qcom_icc_rpm_smd_send(QCOM_SMD_RPM_ACTIVE_STATE,
					    RPM_BUS_MASTER_REQ,
					    mas_rpm_id,
					    sum_bw);
		if (ret) {
			pr_err("qcom_icc_rpm_smd_send mas %d error %d\n",
			       mas_rpm_id, ret);
			return ret;
		}
	}

	if (slv_rpm_id != -1) {
		ret = qcom_icc_rpm_smd_send(QCOM_SMD_RPM_ACTIVE_STATE,
					    RPM_BUS_SLAVE_REQ,
					    slv_rpm_id,
					    sum_bw);
		if (ret) {
			pr_err("qcom_icc_rpm_smd_send slv %d error %d\n",
			       slv_rpm_id, ret);
			return ret;
		}
	}

	return ret;
}

static int qcom_icc_set(struct icc_node *src, struct icc_node *dst)
{
	struct qcom_icc_provider *qp;
	struct qcom_icc_node *qn;
	struct icc_provider *provider;
	struct icc_node *n;
	u64 sum_bw;
	u64 max_peak_bw;
	u64 rate;
	u32 agg_avg = 0;
	u32 agg_peak = 0;
	int ret, i;

	qn = src->data;
	provider = src->provider;
	qp = to_qcom_provider(provider);

	list_for_each_entry(n, &provider->nodes, node_list)
		provider->aggregate(n, 0, n->avg_bw, n->peak_bw,
				    &agg_avg, &agg_peak);

	sum_bw = icc_units_to_bps(agg_avg);
	max_peak_bw = icc_units_to_bps(agg_peak);

	if (!qn->qos.ap_owned) {
		/* send bandwidth request message to the RPM processor */
		ret = qcom_icc_rpm_set(qn->mas_rpm_id, qn->slv_rpm_id, sum_bw);
		if (ret)
			return ret;
	} else if (qn->qos.qos_mode != -1) {
		/* set bandwidth directly from the AP */
		ret = qcom_icc_qos_set(src, sum_bw);
		if (ret)
			return ret;
	}

	rate = max(sum_bw, max_peak_bw);

	do_div(rate, qn->buswidth);

	if (qn->rate == rate)
		return 0;

	for (i = 0; i < qp->num_clks; i++) {
		ret = clk_set_rate(qp->bus_clks[i].clk, rate);
		if (ret) {
			pr_err("%s clk_set_rate error: %d\n",
			       qp->bus_clks[i].id, ret);
			return ret;
		}
	}

	qn->rate = rate;

	return 0;
}

static int qnoc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct qcom_icc_desc *desc;
	struct icc_onecell_data *data;
	struct icc_provider *provider;
	struct qcom_icc_node **qnodes;
	struct qcom_icc_provider *qp;
	struct icc_node *node;
	struct resource *res;
	size_t num_nodes, i;
	int ret;

	/* wait for the RPM proxy */
	if (!qcom_icc_rpm_smd_available())
		return -EPROBE_DEFER;

	desc = of_device_get_match_data(dev);
	if (!desc)
		return -EINVAL;

	qnodes = desc->nodes;
	num_nodes = desc->num_nodes;

	qp = devm_kzalloc(dev, sizeof(*qp), GFP_KERNEL);
	if (!qp)
		return -ENOMEM;

	data = devm_kzalloc(dev, struct_size(data, nodes, num_nodes),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	if (of_device_is_compatible(dev->of_node, "qcom,msm8998-mnoc")) {
		qp->bus_clks = devm_kmemdup(dev, bus_mm_clocks,
					    sizeof(bus_mm_clocks), GFP_KERNEL);
		qp->num_clks = ARRAY_SIZE(bus_mm_clocks);
	} else {
		if (of_device_is_compatible(dev->of_node, "qcom,msm8998-bimc"))
			qp->is_bimc_node = true;

		qp->bus_clks = devm_kmemdup(dev, bus_clocks, sizeof(bus_clocks),
					    GFP_KERNEL);
		qp->num_clks = ARRAY_SIZE(bus_clocks);
	}
	if (!qp->bus_clks)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	qp->mmio = devm_ioremap_resource(dev, res);
	if (IS_ERR(qp->mmio)) {
		dev_err(dev, "Cannot ioremap interconnect bus resource\n");
		return PTR_ERR(qp->mmio);
	}

	qp->regmap = devm_regmap_init_mmio(dev, qp->mmio, desc->regmap_cfg);
	if (IS_ERR(qp->regmap)) {
		dev_err(dev, "Cannot regmap interconnect bus resource\n");
		return PTR_ERR(qp->regmap);
	}

	ret = devm_clk_bulk_get(dev, qp->num_clks, qp->bus_clks);
	if (ret)
		return ret;

	ret = clk_bulk_prepare_enable(qp->num_clks, qp->bus_clks);
	if (ret)
		return ret;

	provider = &qp->provider;
	INIT_LIST_HEAD(&provider->nodes);
	provider->dev = dev;
	provider->set = qcom_icc_set;
	provider->aggregate = icc_std_aggregate;
	provider->xlate = of_icc_xlate_onecell;
	provider->data = data;

	ret = icc_provider_add(provider);
	if (ret) {
		dev_err(dev, "error adding interconnect provider: %d\n", ret);
		clk_bulk_disable_unprepare(qp->num_clks, qp->bus_clks);
		return ret;
	}

	for (i = 0; i < num_nodes; i++) {
		size_t j;

		node = icc_node_create(qnodes[i]->id);
		if (IS_ERR(node)) {
			ret = PTR_ERR(node);
			goto err;
		}

		node->name = qnodes[i]->name;
		node->data = qnodes[i];
		icc_node_add(node, provider);

		for (j = 0; j < qnodes[i]->num_links; j++)
			icc_link_create(node, qnodes[i]->links[j]);

		data->nodes[i] = node;
	}
	data->num_nodes = num_nodes;
	platform_set_drvdata(pdev, qp);

	return 0;
err:
	icc_nodes_remove(provider);
	clk_bulk_disable_unprepare(qp->num_clks, qp->bus_clks);
	icc_provider_del(provider);

	return ret;
}

static int qnoc_remove(struct platform_device *pdev)
{
	struct qcom_icc_provider *qp = platform_get_drvdata(pdev);

	icc_nodes_remove(&qp->provider);
	clk_bulk_disable_unprepare(qp->num_clks, qp->bus_clks);
	return icc_provider_del(&qp->provider);
}

static const struct of_device_id msm8998_noc_of_match[] = {
	{ .compatible = "qcom,msm8998-a1noc", .data = &msm8998_a1noc },
	{ .compatible = "qcom,msm8998-a2noc", .data = &msm8998_a2noc },
	{ .compatible = "qcom,msm8998-bimc", .data = &msm8998_bimc },
	{ .compatible = "qcom,msm8998-cnoc", .data = &msm8998_cnoc },
	{ .compatible = "qcom,msm8998-gnoc", .data = &msm8998_gnoc },
	{ .compatible = "qcom,msm8998-mnoc", .data = &msm8998_mnoc },
	{ .compatible = "qcom,msm8998-snoc", .data = &msm8998_snoc },
	{ },
};
MODULE_DEVICE_TABLE(of, msm8998_noc_of_match);

static struct platform_driver msm8998_noc_driver = {
	.probe = qnoc_probe,
	.remove = qnoc_remove,
	.driver = {
		.name = "qnoc-msm8998",
		.of_match_table = msm8998_noc_of_match,
		.sync_state = icc_sync_state,
	},
};
module_platform_driver(msm8998_noc_driver);
MODULE_DESCRIPTION("Qualcomm msm8998 NoC driver");
MODULE_LICENSE("GPL v2");
