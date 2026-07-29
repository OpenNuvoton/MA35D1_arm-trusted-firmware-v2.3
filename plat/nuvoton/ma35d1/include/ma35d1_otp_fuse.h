/*
 * Copyright (C) 2026 Nuvoton Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __MA35D1_OTP_FUSE_H__
#define __MA35D1_OTP_FUSE_H__

#include <stdint.h>

/*
 * OTP secure region reserved for the firmware anti-rollback counter.
 *
 * OTP read/program operations are word-aligned and operate on one 32-bit
 * word at a time. The byte range 0x120 ~ 0x177 therefore maps to 22
 * word-aligned addresses: 0x120, 0x124, 0x128, ... 0x174.
 *
 * The counter is encoded with a "bit-walk" (thermometer) code across the
 * full 32 bits of each word: bits are burned one at a time from LSB to
 * MSB (bit0 -> bit1 -> ... -> bit31). Programming starts at the highest
 * address of the region (0x174) and moves toward the lowest address
 * (0x120) as the counter value grows, i.e. the word at 0x174 is filled
 * first (starting from its LSB), then 0x170, and so on.
 */
#define OTP_FUSE_CTR_WORD_SIZE		4U	/* bytes per OTP word   */
#define OTP_FUSE_CTR_ADDR_HIGH		0x174U	/* first word written   */
#define OTP_FUSE_CTR_ADDR_LOW		0x120U	/* last word of region  */
#define OTP_FUSE_CTR_WORD_COUNT		(((OTP_FUSE_CTR_ADDR_HIGH - \
					   OTP_FUSE_CTR_ADDR_LOW) / \
					  OTP_FUSE_CTR_WORD_SIZE) + 1U)	/* 22 */
#define OTP_FUSE_CTR_BITS_PER_WORD	32U
#define OTP_FUSE_CTR_MAX_VALUE		(OTP_FUSE_CTR_WORD_COUNT * \
					 OTP_FUSE_CTR_BITS_PER_WORD)	/* 704 */

/*
 * Read back the current firmware anti-rollback counter value.
 *
 * ctr: output, current counter value (0 ~ OTP_FUSE_CTR_MAX_VALUE).
 * Returns 0 on success, negative errno on failure.
 */
int ma35d1_otp_fuse_ctr_read(uint32_t *ctr);

/*
 * Raise the firmware anti-rollback counter up to new_ctr by burning the
 * necessary OTP bits. If the current counter value is already >= new_ctr,
 * this is a no-op (returns 0) as long as new_ctr equals the current value;
 * requesting a value lower than the current counter fails since OTP fuse
 * bits cannot be un-blown.
 *
 * new_ctr: target counter value (0 ~ OTP_FUSE_CTR_MAX_VALUE).
 * Returns 0 on success, negative errno on failure.
 */
int ma35d1_otp_fuse_ctr_set(uint32_t new_ctr);

/*
 * Dump every OTP word in the secure region (word-aligned addresses
 * 0x120 ~ 0x174) for debugging.
 */
void ma35d1_otp_dump_secure_region(void);

#endif /* __MA35D1_OTP_FUSE_H__ */
