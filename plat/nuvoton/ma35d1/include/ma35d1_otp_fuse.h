/*
 * Copyright (C) 2026 Nuvoton Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __MA35D1_OTP_FUSE_H__
#define __MA35D1_OTP_FUSE_H__

#include <stdint.h>

/*
 * OTP anti-rollback counters, each occupying 44 bytes (11 32-bit words):
 * FIP:          0x120 ~ 0x14B
 * BL2:          0x14C ~ 0x177
 * Linux kernel: 0x1A4 ~ 0x1CF
 *
 * ADDR_HIGH/LOW are word-aligned addresses, not inclusive byte endpoints.
 * Each counter uses bit-walk encoding: burn bits LSB to MSB in each word,
 * starting at ADDR_HIGH and moving toward ADDR_LOW. Each has a lifetime
 * maximum of 352 increments. Only the BL2 counter has a programming API.
 */
#define OTP_FUSE_CTR_WORD_SIZE		4U
#define OTP_FUSE_CTR_BITS_PER_WORD	32U

#define OTP_FIP_CTR_ADDR_LOW		0x120U
#define OTP_FIP_CTR_ADDR_HIGH		0x148U
#define OTP_FIP_CTR_WORD_COUNT		(((OTP_FIP_CTR_ADDR_HIGH - \
					   OTP_FIP_CTR_ADDR_LOW) / \
					  OTP_FUSE_CTR_WORD_SIZE) + 1U)
#define OTP_FIP_CTR_MAX_VALUE		(OTP_FIP_CTR_WORD_COUNT * \
					 OTP_FUSE_CTR_BITS_PER_WORD)

#define OTP_BL2_CTR_ADDR_LOW		0x14CU
#define OTP_BL2_CTR_ADDR_HIGH		0x174U
#define OTP_BL2_CTR_WORD_COUNT		(((OTP_BL2_CTR_ADDR_HIGH - \
					   OTP_BL2_CTR_ADDR_LOW) / \
					  OTP_FUSE_CTR_WORD_SIZE) + 1U)
#define OTP_BL2_CTR_MAX_VALUE		(OTP_BL2_CTR_WORD_COUNT * \
					 OTP_FUSE_CTR_BITS_PER_WORD)

#define OTP_KERNEL_CTR_ADDR_LOW		0x1A4U
#define OTP_KERNEL_CTR_ADDR_HIGH	0x1CCU
#define OTP_KERNEL_CTR_WORD_COUNT	(((OTP_KERNEL_CTR_ADDR_HIGH - \
					   OTP_KERNEL_CTR_ADDR_LOW) / \
					   OTP_FUSE_CTR_WORD_SIZE) + 1U)
#define OTP_KERNEL_CTR_MAX_VALUE	(OTP_KERNEL_CTR_WORD_COUNT * \
					 OTP_FUSE_CTR_BITS_PER_WORD)

/*
 * Read back the current BL2 anti-rollback counter value.
 *
 * ctr: output, current counter value (0 ~ OTP_BL2_CTR_MAX_VALUE).
 * Returns 0 on success, negative errno on failure.
 */
int ma35d1_otp_fuse_ctr_read(uint32_t *ctr);

/* Read the FIP / Linux kernel counter; same return convention as BL2. */
int ma35d1_otp_fip_ctr_read(uint32_t *ctr);
int ma35d1_otp_kernel_ctr_read(uint32_t *ctr);

/*
 * Raise the BL2 anti-rollback counter up to new_ctr by burning the
 * necessary OTP bits. If the current counter value is already >= new_ctr,
 * this is a no-op (returns 0) as long as new_ctr equals the current value;
 * requesting a value lower than the current counter fails since OTP fuse
 * bits cannot be un-blown.
 *
 * new_ctr: target counter value (0 ~ OTP_BL2_CTR_MAX_VALUE).
 * Returns 0 on success, negative errno on failure.
 */
int ma35d1_otp_fuse_ctr_set(uint32_t new_ctr);

/*
 * Dump every OTP word in the BL2 counter region (word-aligned addresses
 * 0x14C ~ 0x174) for debugging.
 */
void ma35d1_otp_dump_secure_region(void);

#endif /* __MA35D1_OTP_FUSE_H__ */
