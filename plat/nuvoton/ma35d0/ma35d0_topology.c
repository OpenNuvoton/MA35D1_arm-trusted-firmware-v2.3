/*
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <platform_def.h>

#include <arch.h>
#include <drivers/arm/fvp/fvp_pwrc.h>
#include <lib/cassert.h>
#include <plat/arm/common/arm_config.h>
#include <plat/arm/common/plat_arm.h>
#include <plat/common/platform.h>

/* The MA35D0 power domain tree descriptor */
static unsigned char ma35d0_power_domain_tree_desc[MA35D0_CLUSTER_COUNT + 2];


CASSERT(((MA35D0_CLUSTER_COUNT > 0) && (MA35D0_CLUSTER_COUNT <= 256)),
			assert_invalid_ma35d0_cluster_count);

/*******************************************************************************
 * This function dynamically constructs the topology according to
 * MA35D0_CLUSTER_COUNT and returns it.
 ******************************************************************************/
const unsigned char *plat_get_power_domain_tree_desc(void)
{
	int i;

	/*
	 * The highest level is the system level. The next level is constituted
	 * by clusters and then cores in clusters.
	 */
	ma35d0_power_domain_tree_desc[0] = 1;
	ma35d0_power_domain_tree_desc[1] = MA35D0_CLUSTER_COUNT;

	for (i = 0; i < MA35D0_CLUSTER_COUNT; i++)
		ma35d0_power_domain_tree_desc[i + 2] = MA35D0_MAX_CPUS_PER_CLUSTER;


	return ma35d0_power_domain_tree_desc;
}

/*******************************************************************************
 * This function returns the core count within the cluster corresponding to
 * `mpidr`.
 ******************************************************************************/
unsigned int plat_arm_get_cluster_core_count(u_register_t mpidr)
{
	return MA35D0_MAX_CPUS_PER_CLUSTER;
}

/*******************************************************************************
 * This function implements a part of the critical interface between the psci
 * generic layer and the platform that allows the former to query the platform
 * to convert an MPIDR to a unique linear index. An error code (-1) is returned
 * in case the MPIDR is invalid.
 ******************************************************************************/
int plat_core_pos_by_mpidr(u_register_t mpidr)
{
	unsigned int cluster_id, cpu_id;

	mpidr &= MPIDR_AFFINITY_MASK;

	if ((mpidr & ~(MPIDR_CLUSTER_MASK | MPIDR_CPU_MASK)) != 0U) {
		return -1;
	}

	cluster_id = (mpidr >> MPIDR_AFF1_SHIFT) & MPIDR_AFFLVL_MASK;
	cpu_id = (mpidr >> MPIDR_AFF0_SHIFT) & MPIDR_AFFLVL_MASK;

	if (cluster_id >= PLATFORM_CLUSTER_COUNT) {
		return -1;
	}

	/*
	 * Validate cpu_id by checking whether it represents a CPU in one
	 * of the two clusters present on the platform.
	 */
	if (cpu_id >= PLATFORM_CORE_COUNT) {
		return -1;
	}

	return (int)cpu_id;
}
