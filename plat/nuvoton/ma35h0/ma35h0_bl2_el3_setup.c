/*
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <common/debug.h>
#include <lib/mmio.h>
#include <plat/arm/common/plat_arm.h>

#include "ma35h0_private.h"

void bl2_el3_early_platform_setup(u_register_t arg0 __unused,
				  u_register_t arg1 __unused,
				  u_register_t arg2 __unused,
				  u_register_t arg3 __unused)
{
	/* Initialize the platform config for future decision making */
	ma35h0_config_setup();

#if OTP_ANTI_ROLLBACK
	mmio_write_32(0x2803EC20, 0);
	mmio_write_32(0x2803EC24, 0);
	mmio_write_32(0x2803EC28, 0);
#endif

	ma35h0_ddr_init();

#if MA35_ANTI_ROLLBACK_AB_BOOT
	NOTICE("BL2: version %d\n", MA35_BL2_VERSION);

	if (MA35_BL2_VERSION < mmio_read_32(0x2803EC20)) {
		ERROR("BL2 firmware version rollback detected, halting (version=%d, min=%d)\n",
		      MA35_BL2_VERSION, mmio_read_32(0x2803EC20));
		panic();
	}

	mmio_write_32(0x2803EC04, MA35_BL2_VERSION);
#endif

	/*
	 * Initialize Interconnect for this cluster during cold boot.
	 * No need for locks as no other CPU is active.
	 */
	ma35h0_interconnect_init();
	/*
	 * Enable coherency in Interconnect for the primary CPU's cluster.
	 */
	ma35h0_interconnect_enable();

}
