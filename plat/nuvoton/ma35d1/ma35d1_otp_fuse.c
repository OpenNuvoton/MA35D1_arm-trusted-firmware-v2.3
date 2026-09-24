/*
 * Copyright (C) 2026 Nuvoton Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include <stdint.h>

#include <common/debug.h>
#include <lib/mmio.h>
#include <platform_def.h>

#include <ma35d1_otp_fuse.h>
#include <tsi_cmd.h>

#define OTP_BASE		0x40350000U
#define OTP_CTL			(OTP_BASE + 0x000U)
#define OTP_STS			(OTP_BASE + 0x004U)
#define OTP_ADDR		(OTP_BASE + 0x008U)
#define OTP_DATA		(OTP_BASE + 0x00CU)

#define OTP_CTL_START		(1U << 0)
#define OTP_CTL_PROGRAM		(1U << 4)

#define OTP_STS_BUSY		(1U << 0)
#define OTP_STS_PFF		(1U << 1)
#define OTP_STS_ADDRFF		(1U << 2)
#define OTP_STS_CMDFF		(1U << 4)

#define OTP_BUSY_POLL_LIMIT	12000000U

static int otp_wait_ready(void)
{
	uint32_t poll_count = 0;

	while ((mmio_read_32(OTP_STS) & OTP_STS_BUSY) != 0U) {
		if (poll_count++ > OTP_BUSY_POLL_LIMIT)
			return -ETIMEDOUT;
	}
	return 0;
}

int ma35d1_otp_read(uint32_t addr, uint32_t *data)
{
	uint32_t status;
	int ret;

	if (data == NULL)
		return -EINVAL;

	if ((mmio_read_32(SYS_CHIPCFG) & 0x100U) == 0U)
		return TSI_OTP_Read(addr, data);

	ret = otp_wait_ready();
	if (ret != 0)
		return ret;

	mmio_write_32(OTP_STS, OTP_STS_ADDRFF | OTP_STS_CMDFF);
	mmio_write_32(OTP_ADDR, addr);
	mmio_write_32(OTP_CTL, OTP_CTL_START);

	ret = otp_wait_ready();
	if (ret != 0)
		return ret;

	status = mmio_read_32(OTP_STS);
	if ((status & (OTP_STS_ADDRFF | OTP_STS_CMDFF)) != 0U) {
		mmio_write_32(OTP_STS, OTP_STS_ADDRFF | OTP_STS_CMDFF);
		return -EIO;
	}

	*data = mmio_read_32(OTP_DATA);
	return 0;
}

int ma35d1_otp_program(uint32_t addr, uint32_t data)
{
	uint32_t status;
	int ret;

	if ((mmio_read_32(SYS_CHIPCFG) & 0x100U) == 0U)
		return TSI_OTP_Program(addr, data);

	ret = otp_wait_ready();
	if (ret != 0)
		return ret;

	mmio_write_32(OTP_STS,
		      OTP_STS_PFF | OTP_STS_ADDRFF | OTP_STS_CMDFF);
	mmio_write_32(OTP_ADDR, addr);
	mmio_write_32(OTP_DATA, data);
	mmio_write_32(OTP_CTL, OTP_CTL_PROGRAM | OTP_CTL_START);

	ret = otp_wait_ready();
	if (ret != 0)
		return ret;

	status = mmio_read_32(OTP_STS);
	if ((status & (OTP_STS_PFF | OTP_STS_ADDRFF |
		       OTP_STS_CMDFF)) != 0U) {
		mmio_write_32(OTP_STS,
			      OTP_STS_PFF | OTP_STS_ADDRFF | OTP_STS_CMDFF);
		return -EIO;
	}

	return 0;
}

#if OTP_ANTI_ROLLBACK

static int otp_fuse_ctr_read(uint32_t low, uint32_t high, uint32_t *ctr)
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
	for (addr = high; addr >= low;
	     addr -= OTP_FUSE_CTR_WORD_SIZE) {
		ret = ma35d1_otp_read(addr, &data);
		if (ret != 0) {
			ERROR("otp_fuse: read addr 0x%x failed (%d)\n", addr, ret);
			return ret;
		}
		total += __builtin_popcount(data);
	}

	*ctr = total;
	return 0;
}

int ma35d1_otp_fuse_ctr_read(uint32_t *ctr)
{
	return otp_fuse_ctr_read(OTP_BL2_CTR_ADDR_LOW,
				 OTP_BL2_CTR_ADDR_HIGH, ctr);
}

int ma35d1_otp_fip_ctr_read(uint32_t *ctr)
{
	return otp_fuse_ctr_read(OTP_FIP_CTR_ADDR_LOW,
				 OTP_FIP_CTR_ADDR_HIGH, ctr);
}

int ma35d1_otp_kernel_ctr_read(uint32_t *ctr)
{
	return otp_fuse_ctr_read(OTP_KERNEL_CTR_ADDR_LOW,
				 OTP_KERNEL_CTR_ADDR_HIGH, ctr);
}

int ma35d1_otp_fuse_ctr_set(uint32_t new_ctr)
{
	uint32_t cur_ctr;
	uint32_t word_idx, bit_idx, addr, data;
	int ret;

	if (new_ctr > OTP_BL2_CTR_MAX_VALUE)
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
		addr = OTP_BL2_CTR_ADDR_HIGH - (word_idx * OTP_FUSE_CTR_WORD_SIZE);

		/* Blow only the next bit; already-set bits stay untouched. */
		data = (1U << bit_idx);
		ret = ma35d1_otp_program(addr, data);
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

	printf("OTP BL2 counter region dump (0x%x ~ 0x%x):\n",
	       OTP_BL2_CTR_ADDR_LOW, OTP_BL2_CTR_ADDR_HIGH);

	for (addr = OTP_BL2_CTR_ADDR_LOW; addr <= OTP_BL2_CTR_ADDR_HIGH;
	     addr += OTP_FUSE_CTR_WORD_SIZE) {
		ret = ma35d1_otp_read(addr, &data);
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

int ma35d1_otp_fip_ctr_read(uint32_t *ctr)
{
	return -ENOTSUP;
}

int ma35d1_otp_kernel_ctr_read(uint32_t *ctr)
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
