/*
 * Copyright (C) 2026 Nuvoton Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include <stdint.h>

#include <common/debug.h>

#include <ma35d1_otp_fuse.h>
#include <tsi_cmd.h>

#if OTP_ANTI_ROLLBACK

int ma35d1_otp_fuse_ctr_read(uint32_t *ctr)
{
	uint32_t addr;
	uint32_t data;
	uint32_t total = 0;
	int ret;

	if (ctr == NULL)
		return -EINVAL;

	/*
	 * Bit-walk code within a word is always contiguous from LSB, so the
	 * total counter value is simply the sum of set bits (popcount) over
	 * every word in the region, regardless of which word is currently
	 * being filled.
	 */
	for (addr = OTP_FUSE_CTR_ADDR_HIGH; addr >= OTP_FUSE_CTR_ADDR_LOW;
	     addr -= OTP_FUSE_CTR_WORD_SIZE) {
		ret = TSI_OTP_Read(addr, &data);
		if (ret != 0) {
			ERROR("otp_fuse: read addr 0x%x failed (%d)\n", addr, ret);
			return ret;
		}
		total += __builtin_popcount(data);
	}

	*ctr = total;
	return 0;
}

int ma35d1_otp_fuse_ctr_set(uint32_t new_ctr)
{
	uint32_t cur_ctr;
	uint32_t word_idx, bit_idx, addr, data;
	int ret;

	if (new_ctr > OTP_FUSE_CTR_MAX_VALUE)
		return -EINVAL;

	ret = ma35d1_otp_fuse_ctr_read(&cur_ctr);
	if (ret != 0)
		return ret;

	/* OTP fuse bits can only be blown (0 -> 1), never rolled back. */
	if (new_ctr < cur_ctr)
		return -EPERM;

	while (cur_ctr < new_ctr) {
		word_idx = cur_ctr / OTP_FUSE_CTR_BITS_PER_WORD;
		bit_idx  = cur_ctr % OTP_FUSE_CTR_BITS_PER_WORD;
		addr = OTP_FUSE_CTR_ADDR_HIGH - (word_idx * OTP_FUSE_CTR_WORD_SIZE);

		/* Blow only the next bit; already-set bits stay untouched. */
		data = (1U << bit_idx);
		ret = TSI_OTP_Program(addr, data);
		if (ret != 0) {
			ERROR("otp_fuse: program addr 0x%x bit %u failed (%d)\n",
			      addr, bit_idx, ret);
			return ret;
		}
		cur_ctr++;
	}
	return 0;
}

void ma35d1_otp_dump_secure_region(void)
{
	uint32_t addr, data;
	int ret;

	printf("OTP secure region dump (0x%x ~ 0x%x):\n",
	       OTP_FUSE_CTR_ADDR_LOW, OTP_FUSE_CTR_ADDR_HIGH);

	for (addr = OTP_FUSE_CTR_ADDR_LOW; addr <= OTP_FUSE_CTR_ADDR_HIGH;
	     addr += OTP_FUSE_CTR_WORD_SIZE) {
		ret = TSI_OTP_Read(addr, &data);
		if (ret != 0) {
			printf("  0x%03x: <read error %d>\n", addr, ret);
			continue;
		}
		printf("  0x%03x: 0x%08x\n", addr, data);
	}
}

#else /* !OTP_ANTI_ROLLBACK */

int ma35d1_otp_fuse_ctr_read(uint32_t *ctr)
{
	return -ENOTSUP;
}

int ma35d1_otp_fuse_ctr_set(uint32_t new_ctr)
{
	return -ENOTSUP;
}

void ma35d1_otp_dump_secure_region(void)
{
}

#endif /* OTP_ANTI_ROLLBACK */
