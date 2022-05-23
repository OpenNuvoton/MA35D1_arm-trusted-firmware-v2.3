/*
 * Copyright (C) 2021 Nuvoton Technology Corp.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>

#include <common/debug.h>
#include <common/runtime_svc.h>
#include <lib/mmio.h>
#include <tools_share/uuid.h>

#include <ma35d1_sip_svc.h>
#include <drivers/nuvoton/ma35d1_pmic.h>
#include "ma35d1_private.h"

#pragma weak ma35d1_plat_sip_handler
#define CPU_1000MHZ   1
#define CPU_800MHZ    2
#define CPU_700MHZ    3
#define CPU_500MHZ    4

uintptr_t ma35d1_plat_sip_handler(uint32_t smc_fid,
				    u_register_t x1,
				    u_register_t x2,
				    u_register_t x3,
				    u_register_t x4,
				    void *cookie,
				    void *handle,
				    u_register_t flags)
{
	ERROR("%s: unhandled SMC (0x%x)\n", __func__, smc_fid);
	SMC_RET1(handle, SMC_UNK);
}

/*
 * This function is responsible for handling all SiP calls from the NS world
 */
uintptr_t sip_smc_handler(uint32_t smc_fid,
			  u_register_t x1,
			  u_register_t x2,
			  u_register_t x3,
			  u_register_t x4,
			  void *cookie,
			  void *handle,
			  u_register_t flags)
{
	uint32_t ns;
	uint32_t volt;
	int rev;
	int CPU_CLK = 0;

	/* Determine which security state this SMC originated from */
	ns = is_caller_non_secure(flags);
	if (!ns)
		SMC_RET1(handle, SMC_UNK);

	switch (smc_fid) {
	case SIP_SVC_PMIC:
		/* Return the number of ma35d1 SiP Service Calls. */
		volt = (uint32_t)x2;
		if (volt == 0)
			volt = ma35d1_get_pmic(x1);
		else {
			if (volt != ma35d1_get_pmic(x1))
				ma35d1_set_pmic(x1, x2);
		}
		SMC_RET1(handle, volt);

	case SIP_CPU_CLK:
		if ((uint32_t)x1 == CPU_1000MHZ)
			CPU_CLK = CPU_PLL_1G;
		else if ((uint32_t)x1 == CPU_800MHZ)
			CPU_CLK = CPU_PLL_800;
		else if ((uint32_t)x1 == CPU_700MHZ)
			CPU_CLK = CPU_PLL_700;
		else if ((uint32_t)x1 == CPU_500MHZ)
			CPU_CLK = CPU_PLL_500;

		rev = ma35d1_change_pll(CPU_CLK);
		if (rev == 1) {
			WARN("Set CPU clock Fail !!\n");
		}

		SMC_RET1(handle, rev);

	case SIP_SVC_VERSION:
		/* Return the version of current implementation */
		SMC_RET2(handle, NVT_SIP_SVC_VERSION_MAJOR,
			NVT_SIP_SVC_VERSION_MINOR);

	default:
		return ma35d1_plat_sip_handler(smc_fid, x1, x2, x3, x4,
			cookie, handle, flags);
	};

}

/* Define a runtime service descriptor for fast SMC calls */
DECLARE_RT_SVC(
	ma35d1_sip_svc,
	OEN_SIP_START,
	OEN_SIP_END,
	SMC_TYPE_FAST,
	NULL,
	sip_smc_handler
);
