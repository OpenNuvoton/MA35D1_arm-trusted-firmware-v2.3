/*
 * Copyright (C) 2021 Nuvoton Technology Corp.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>

#include <common/debug.h>
#include <common/runtime_svc.h>
#include <lib/mmio.h>
#include <drivers/delay_timer.h>
#include <tools_share/uuid.h>

#include <ma35d1_sip_svc.h>
#include <drivers/nuvoton/ma35d1_pmic.h>
#include "ma35d1_private.h"
#include "include/ma35d1_crypto.h"
#include "include/tsi_cmd.h"

#pragma weak ma35d1_plat_sip_handler

static unsigned int CAPLL_MODE[8] = {
	0x00000364,	/* 800 MHz */
	0x000006AF,	/* 700 MHz */
	0x000006A2,	/* 650 MHz */
	0x00001396,	/* 600 MHz */
	0x0000137D,	/* 500 MHz */
	0x0000237D,	/* 250 MHz */
	0x0000337D,	/* 125 MHz */
};

static uint32_t eppl_div_restore = 0xFFFFFFFF;
static uint32_t eppl_ctl1 = 0x1, vppl_ctl1 = 0x1;

static __inline void ma35d1_UnlockReg(void)
{
	do {
		mmio_write_32(0x404601A0, 0x59UL);
		mmio_write_32(0x404601A0, 0x16UL);
		mmio_write_32(0x404601A0, 0x88UL);
	} while (mmio_read_32(0x404601A0) == 0UL);
}

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
 * This function changes the CPU PLL
 */
static int32_t ma35d1_set_cpu_clock(int cpu_freq)
{
	uint64_t timeout;
	uint32_t reg, index;

	/* CA-PLL */
	switch (cpu_freq) {
	case CPU_PLL_800:
		index = 0;
		INFO("CA-PLL is 800 MHz\n");
		break;
	case CPU_PLL_700:
		index = 1;
		INFO("CA-PLL is 700 MHz\n");
		break;
	case CPU_PLL_650:
		index = 2;
		INFO("CA-PLL is 600 MHz\n");
		break;
	case CPU_PLL_600:
		index = 3;
		INFO("CA-PLL is 600 MHz\n");
		break;
	case CPU_PLL_500:
		index = 4;
		INFO("CA-PLL is 500 MHz\n");
		break;
	case CPU_PLL_250:
		index = 5;
		INFO("CA-PLL is 250 MHz\n");
		break;
	case CPU_PLL_125:
		index = 6;
		INFO("CA-PLL is 125 MHz\n");
		break;
	default:
		WARN("Invaild CA-PLL !!\n");
		return 1;
	};

	reg = (mmio_read_32(SYS_BA) >> 16) & 0xff;
	if (reg == 0xa1 || reg == 0x81 || reg == 0x82)
		return -1;

	/* set CA35 to DDRPLL */
	mmio_write_32(CLK_CLKSEL0, (mmio_read_32(CLK_CLKSEL0) & ~0x3) | 0x2);

	mmio_write_32(CLK_PLL0CTL0, CAPLL_MODE[index]);

	timeout = timeout_init_us(12000);	/* 1ms */
	/* check PLL stable */
	while (1) {
		if ((mmio_read_32(CLK_STATUS) & 0x40) == 0x40)
			break;

		if (timeout_elapsed(timeout)) {
			ERROR("PLL wait stable timeout!\n");
			return 1;
		}
	}
	/* set CA35 to CA-PLL */
	mmio_write_32(CLK_CLKSEL0, (mmio_read_32(CLK_CLKSEL0) & ~0x3) | 0x1);
	return 0;
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
	uint32_t ns, reg;
	uint32_t volt;
	int ret;
	int CPU_CLK = 0;

	/* unlock */
	mmio_write_32(SYS_RLKTZS, 0x59);
	mmio_write_32(SYS_RLKTZS, 0x16);
	mmio_write_32(SYS_RLKTZS, 0x88);

	// INFO("TFA SIP - 0x%x, %ld, %ld\n", smc_fid, x1, x2);

	/* Determine which security state this SMC originated from */
	ns = is_caller_non_secure(flags);
	if (!ns)
		SMC_RET1(handle, SMC_UNK);

	switch (smc_fid) {
	case SIP_SVC_PMIC:
		volt = (uint32_t)x2;
		if (volt == 0)
                       volt = ma35d1_get_pmic(x1);
		else {
			if (volt != ma35d1_get_pmic(x1))
				ma35d1_set_pmic(x1, x2);
		}
		mmio_write_32(SYS_RLKTZS, 0);
		SMC_RET1(handle, volt);

	case SIP_CPU_CLK:
		if ((uint32_t)x1 == 800)
			CPU_CLK = CPU_PLL_800;
		else if ((uint32_t)x1 == 700)
			CPU_CLK = CPU_PLL_700;
		else if ((uint32_t)x1 == 650)
			CPU_CLK = CPU_PLL_650;
		else if ((uint32_t)x1 == 600)
			CPU_CLK = CPU_PLL_600;
		else if ((uint32_t)x1 == 500)
			CPU_CLK = CPU_PLL_500;
		else if ((uint32_t)x1 == 250)
			CPU_CLK = CPU_PLL_250;
		else if ((uint32_t)x1 == 125)
			CPU_CLK = CPU_PLL_125;
		else {
			WARN("Set CPU clock %ld Fail !!\n", x1);
			SMC_RET1(handle, 1);
		}

		reg = (mmio_read_32(SYS_BA) >> 16) & 0xff;
		if (reg == 0xa1 || reg == 0x81 || reg == 0x82)
			SMC_RET1(handle, 1);

		ret = ma35d1_set_cpu_clock(CPU_CLK);
		if (ret == 1) {
			WARN("Set CPU clock %ld Fail !!\n", x1);
		}
		mmio_write_32(SYS_RLKTZS, 0);
		SMC_RET1(handle, ret);

	case SIP_SET_EPLL:
		// INFO("TFA SIP - CLK_PLL4CTL1 = 0x%x, restore = 0x%x\n", mmio_read_32(CLK_PLL4CTL1), eppl_div_restore);
		if (eppl_div_restore == 0xFFFFFFFF)
			eppl_div_restore = mmio_read_32(CLK_PLL4CTL1);
		if ((uint32_t)x1 == NVT_SIP_SVC_EPLL_DIV_BY_2)
			mmio_write_32(CLK_PLL4CTL1, eppl_div_restore + 0x10);
		else if ((uint32_t)x1 == NVT_SIP_SVC_EPLL_DIV_BY_4)
			mmio_write_32(CLK_PLL4CTL1, eppl_div_restore + 0x20);
		else if ((uint32_t)x1 == NVT_SIP_SVC_EPLL_DIV_BY_8)
			mmio_write_32(CLK_PLL4CTL1, eppl_div_restore + 0x30);
		else {
			mmio_write_32(CLK_PLL4CTL1, eppl_div_restore);
			eppl_div_restore = 0xFFFFFFFF;
		}
		mmio_write_32(SYS_RLKTZS, 0);
		SMC_RET2(handle, 0, mmio_read_32(CLK_PLL4CTL1));

	case SIP_LOW_SPEED:
		if ((uint32_t)x1 == 0) {
			// INFO("Leave low speed mode\n");
			ma35d1_set_cpu_clock(CPU_PLL_500);
			if ((mmio_read_32(SYS_CHIPCFG) & (1 << 8)) == 0)
				TSI_Set_Clock(0x80235A);
			mmio_write_32(CLK_PLL4CTL1, eppl_ctl1);
			mmio_write_32(CLK_PLL5CTL1, vppl_ctl1);
		} else {
			// INFO("Enter low speed mode\n");
			ma35d1_set_cpu_clock(CPU_PLL_125);
			if ((mmio_read_32(SYS_CHIPCFG) & (1 << 8)) == 0)
				TSI_Set_Clock(0x802312);
			eppl_ctl1 = mmio_read_32(CLK_PLL4CTL1);
			vppl_ctl1 = mmio_read_32(CLK_PLL5CTL1);
			mmio_write_32(CLK_PLL4CTL1, eppl_ctl1 | 0x70);
			mmio_write_32(CLK_PLL5CTL1, vppl_ctl1 | 0x70);
		}
		mmio_write_32(SYS_RLKTZS, 0);
		SMC_RET1(handle, 0);

	case SIP_DDR_SELFREF:
		if ((uint32_t)x1 == 0) {
			/*
			 * disable SDRAM auto-refresh
			 */
			mmio_write_32(UMCTL2_BA + 0x30, mmio_read_32(UMCTL2_BA + 0x30) & ~0x1);
		} else {
			/*
			 * enable SDRAM auto-refresh
			 */
			mmio_write_32(UMCTL2_BA + 0x30, mmio_read_32(UMCTL2_BA + 0x30) | 0x1);
		}
		SMC_RET1(handle, 0);

	case SIP_CHIP_RESET:
		ma35d1_UnlockReg();
		mmio_write_32(SYS_IPRST0, 0x1);
		mmio_write_32(SYS_IPRST0, 0x0);
		WARN("SIP_CHIP_RESET not work!\n");
		SMC_RET1(handle, 0);

	case SIP_BL2_OFFSET:
		// INFO("BL2 offset: 0x%x\n", mmio_read_32(0x2803FFF0));
		SMC_RET1(handle, mmio_read_32(0x2803FFF0));

	case SIP_SVC_GET_REGISTER:
		if (((uint32_t)x1 < 0x40000000) || ((uint32_t)x1 > 0x50808000)) {
			WARN("SIP_SVC_GET_REGISTER - invalid register 0x%x!\n", (uint32_t)x1);
			SMC_RET1(handle, 1);
		}
		// INFO("SIP_SVC_GET_REGISTER read 0x%x = 0x%x\n", (uint32_t)x1, mmio_read_32((uint32_t)x1));
		SMC_RET1(handle, mmio_read_32((uint32_t)x1));

	case SIP_SVC_VERSION:
		/* Return the version of current implementation */
		SMC_RET3(handle, 0, NVT_SIP_SVC_VERSION_MAJOR,
			NVT_SIP_SVC_VERSION_MINOR);

	default:
		mmio_write_32(SYS_RLKTZS, 0);
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
