/* SPDX-License-Identifier: GPL-2.0 */
/* MSM8998 interconnect IDs */

#ifndef __DT_BINDINGS_INTERCONNECT_QCOM_MSM8998_H
#define __DT_BINDINGS_INTERCONNECT_QCOM_MSM8998_H

/* A1NOC */
#define MASTER_PCIE_0			0
#define MASTER_USB3                     1
#define MASTER_UFS                      2
#define MASTER_BLSP_2                   3
#define SLAVE_A1NOC_SNOC		4

/* A2NOC */
#define MASTER_IPA			0
#define MASTER_CNOC_A2NOC		1
#define MASTER_SDCC_2			2
#define MASTER_SDCC_4			3
#define MASTER_TSIF			4
#define MASTER_BLSP_1			5
#define MASTER_CRVIRT_A2NOC		6
#define MASTER_CRYPTO_C0		7
#define SLAVE_A2NOC_SNOC		8
#define SLAVE_CRVIRT_A2NOC		9

/* BIMC */
#define MASTER_GNOC_BIMC		0
#define MASTER_OXILI			1
#define MASTER_MNOC_BIMC		2
#define MASTER_SNOC_BIMC		3
#define SLAVE_EBI			4
#define SLAVE_HMSS_L3			5
#define SLAVE_BIMC_SNOC_0		6
#define SLAVE_BIMC_SNOC_1		7

/* CNOC */
#define MASTER_SNOC_CNOC		0
#define MASTER_QDSS_DAP			1
#define SLAVE_CNOC_A2NOC		2
#define SLAVE_SSC_CFG			3
#define SLAVE_MPM			4
#define SLAVE_PMIC_ARB			5
#define SLAVE_TLMM_NORTH		6
#define SLAVE_PIMEM_CFG			7
#define SLAVE_IMEM_CFG			8
#define SLAVE_MESSAGE_RAM		9
#define SLAVE_SKL			10
#define SLAVE_BIMC_CFG			11
#define SLAVE_PRNG			12
#define SLAVE_A2NOC_CFG			13
#define SLAVE_IPA			14
#define SLAVE_TCSR			15
#define SLAVE_SNOC_CFG			16
#define SLAVE_CLK_CTL			17
#define SLAVE_GLM			18
#define SLAVE_SPDM			19
#define SLAVE_GPUSS_CFG			20
#define SLAVE_CNOC_MNOC_CFG		21
#define SLAVE_QM_CFG			22
#define SLAVE_MSS_CFG			23
#define SLAVE_UFS_CFG			24
#define SLAVE_TLMM_WEST			25
#define SLAVE_A1NOC_CFG			26
#define SLAVE_AHB2PHY			27
#define SLAVE_BLSP_2			28
#define SLAVE_PDM			29
#define SLAVE_USB3_0			30
#define SLAVE_A1NOC_SMMU_CFG		31
#define SLAVE_BLSP_1			32
#define SLAVE_SDCC_2			33
#define SLAVE_SDCC_4			34
#define SLAVE_TSIF			35
#define SLAVE_QDSS_CFG			36
#define SLAVE_TLMM_EAST			37
#define SLAVE_CNOC_MNOC_MMSS_CFG	38
#define SLAVE_SRVC_CNOC			39

/* GNOC */
#define MASTER_APSS_PROC		0
#define SLAVE_GNOC_BIMC			1

/* MNOC */
#define MASTER_CNOC_MNOC_CFG		0
#define MASTER_CPP			1
#define MASTER_JPEG			2
#define MASTER_MDP_P0			3
#define MASTER_MDP_P1			4
#define MASTER_ROTATOR			5
#define MASTER_VENUS			6
#define MASTER_VFE			7
#define MASTER_VENUS_VMEM		8
#define SLAVE_MNOC_BIMC			9
#define SLAVE_VMEM			10
#define SLAVE_SRVC_MNOC			11
#define MASTER_CNOC_MNOC_MMSS_CFG	12
#define SLAVE_CAMERA_CFG		13
#define SLAVE_CAMERA_THROTTLE_CFG	14
#define SLAVE_MISC_CFG			15
#define SLAVE_VENUS_THROTTLE_CFG	16
#define SLAVE_VENUS_CFG			17
#define SLAVE_VMEM_CFG			18
#define SLAVE_MMSS_CLK_XPU_CFG		19
#define SLAVE_MMSS_CLK_CFG		20
#define SLAVE_DISPLAY_CFG		21
#define SLAVE_DISPLAY_THROTTLE_CFG	22
#define SLAVE_SMMU_CFG			23

/* SNOC */
#define MASTER_HMSS			0
#define MASTER_QDSS_BAM			1
#define MASTER_SNOC_CFG			2
#define MASTER_BIMC_SNOC_0		3
#define MASTER_BIMC_SNOC_1		4
#define MASTER_A1NOC_SNOC		5
#define MASTER_A2NOC_SNOC		6
#define MASTER_QDSS_ETR			7
#define SLAVE_HMSS			8
#define SLAVE_LPASS			9
#define SLAVE_WLAN			10
#define SLAVE_SNOC_BIMC			11
#define SLAVE_SNOC_CNOC			12
#define SLAVE_IMEM			13
#define SLAVE_PIMEM			14
#define SLAVE_QDSS_STM			15
#define SLAVE_PCIE_0			16
#define SLAVE_SRVC_SNOC			17

#endif
