/*
 * Copyright (C) 2023 Nuvoton Technology Corp.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef MA35H0_SIP_SVC_H
#define MA35H0_SIP_SVC_H

/* SMC function IDs for SiP Service queries */
#define SIP_SVC_GET_REGISTER		0xC2000007
#define SIP_SVC_PMIC			0xC2000008
#define SIP_CPU_CLK			0xC2000009
#define SIP_SET_EPLL			0xC200000A
#define SIP_LOW_SPEED			0xC200000B
#define SIP_DDR_SELFREF			0xC200000C
#define SIP_CHIP_RESET			0xC200000D
#define SIP_SVC_VERSION			0xC200000F

/* MA35H0 SiP Service Calls PLL setting */
#define NVT_SIP_SVC_EPLL_DIV_BY_2	0x2
#define NVT_SIP_SVC_EPLL_DIV_BY_4	0x4
#define NVT_SIP_SVC_EPLL_DIV_BY_8	0x8
#define NVT_SIP_SVC_EPLL_RESTORE	0xF

/* MA35H0 SiP Service Calls version numbers */
#define NVT_SIP_SVC_VERSION_MAJOR	0x0
#define NVT_SIP_SVC_VERSION_MINOR	0x2

/* Number of MA35H0 SiP Calls implemented */
#define NVT_COMMON_SIP_NUM_CALLS	0x3

enum {
	RK_SIP_E_SUCCESS = 0,
	RK_SIP_E_INVALID_PARAM = -1
};

#endif /* MA35H0_SIP_SVC_H */
