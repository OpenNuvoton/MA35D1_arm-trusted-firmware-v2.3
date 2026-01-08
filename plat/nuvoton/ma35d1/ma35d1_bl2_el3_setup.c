/*
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <common/debug.h>
#include <plat/arm/common/plat_arm.h>
#include <drivers/generic_delay_timer.h>
#include <drivers/delay_timer.h>
#include <lib/mmio.h>

#include "ma35d1_private.h"
#include <ma35d1_crypto.h>
#include <tsi_cmd.h>

#if FIP_DE_AES
#define UART_DAT                U(0x40700000)
#define UART_FIFOSTS            U(0x40700018)
#define RX_EMPTY                (1 << 14)
#define TX_EMPTY                (1 << 22)

#define TIMEOUT_US_10_MS	120000U

static char uart0_getc(void)
{
	while (1) {
		if (!(mmio_read_32(UART_FIFOSTS) & RX_EMPTY))
			return mmio_read_32(UART_DAT);
	}
}

static void uart0_putc(char c)
{
	while (!(mmio_read_32(UART_FIFOSTS) & TX_EMPTY)) ;
	mmio_write_32(UART_DAT, c);
}

static void uart0_puts(char data[])
{
	for (int i = 0; i < 64; i++) {
		if ((data[i] == 0) || (data[i] == 0xd))
			break;
		else
			uart0_putc(data[i]);
	}
}

static char ch2hex(char ch)
{
	if (ch <= '9')
		ch = ch - '0';
	else if ((ch <= 'z') && (ch >= 'a'))
		ch = ch - 'a' + 10U;
	else
		ch = ch - 'A' + 10U;
	return ch;
}

static void Hex2Reg(char input[], unsigned int volatile reg[])
{
	char      hex;
	int       si, ri;
	unsigned int  i, val32;

	si = 64 - 1;
	ri = 7;

	while (si >= 0) {
		val32 = 0UL;
		for (i = 0UL; (i < 8UL) && (si >= 0); i++) {
			hex = ch2hex(input[si]);
			val32 |= (unsigned int)hex << (i * 4UL);
			si--;
		}
		reg[ri--] = val32;
	}
}

static int EnterRMAState(void)
{
	int sid, j, status = 0;
	volatile unsigned char *param = (volatile unsigned char *)TSI_PARAM_BASE;
	char ch;
	static char data[64];
	static unsigned int data0[8];
	static unsigned int data1[8];
	uint64_t timeout;

	printf("please press the 'Esc' first\n");
	/* reset RX FIFO */
	mmio_write_32(0x40700008, mmio_read_32(0x40700008) | 0x2);
	while((mmio_read_32(0x40700008) & 0x2) == 0x2);
	j = 0;
	timeout = read_cntpct_el0();
	while (1) {
		ch = uart0_getc();
		if (ch == 0x1b)
			break;
		if ((read_cntpct_el0() - timeout) > 60000000) //5000ms
			break;
	}
	printf("please input 64 char raw data\n");
	memset(data, 0, 64);
	/* reset RX FIFO */
	mmio_write_32(0x40700008, mmio_read_32(0x40700008) | 0x2);
	while((mmio_read_32(0x40700008) & 0x2) == 0x2);
	j = 0;
	while (1) {
		ch = uart0_getc();
		if (ch == 0xd) {
			printf("done\n");
			break;
		}
		data[j++] = ch;
	}
	uart0_puts(data);
	Hex2Reg(data, data0);

	printf("\n\nplease input 64 char encrypt data\n");
	memset(data, 0, 64);
	/* reset RX FIFO */
	mmio_write_32(0x40700008, mmio_read_32(0x40700008) | 0x2);
	while((mmio_read_32(0x40700008) & 0x2) == 0x2);
	j = 0;
	while (1) {
		ch = uart0_getc();
		if (ch == 0xd) {
			printf("done\n");
			break;
		}
		data[j++] = ch;
	}
	uart0_puts(data);
	Hex2Reg(data, data1);
	printf("\n\n");

	/* verify encrypt data */
	if ((mmio_read_32(SYS_CHIPCFG) & 0x100) == 0x000) {
		/* Enable whc0 clock */
		mmio_write_32(CLK_SYSCLK1, (mmio_read_32(CLK_SYSCLK1) | 0x10));
		/* connect with TSI */
		while (1) {
			if (TSI_Sync() == 0)
				break;
		}
		if ((status = TSI_Open_Session(C_CODE_AES, &sid)) != 0)
			return status;
		inv_dcache_range(TSI_PARAM_BASE, 32);
		for (j=0; j<32; j++)
			*(param+j) = 0;
		inv_dcache_range(TSI_PARAM_BASE, 32);
		if ((status = TSI_AES_Set_IV(sid, (unsigned int)TSI_PARAM_BASE)) != 0)
			return status;
		if ((status = TSI_AES_Set_Mode(sid, 0, 0, 1, 1, 0, 0, 2, 2, 5, 8)) != 0)
			return status;
		if ((status = TSI_AES_Run(sid, 1, 32, (unsigned int)(long)data1, (unsigned int)TSI_PARAM_BASE)) != 0)
			return status;
		TSI_Close_Session(C_CODE_AES, sid);
		inv_dcache_range(TSI_PARAM_BASE, 32);
		for (j=0; j<8; j++) {
			if (*(unsigned int *)((unsigned long)TSI_PARAM_BASE+j*4) != data0[j]) {
				printf("data compare error\n");
				return 1;
			}
		}
		/* PLM = RMA */
		status = TSI_OTP_Program(0x108, 0x7);
		TSI_Reset();
		if (status != 0)
			return status;
		else
			mmio_write_32(0x40460020, 0x1);
	}
	return 0;
}

#endif


void bl2_el3_early_platform_setup(u_register_t arg0 __unused,
				  u_register_t arg1 __unused,
				  u_register_t arg2 __unused,
				  u_register_t arg3 __unused)
{
#if FIP_DE_AES
	uint64_t timeout;
	unsigned int volatile state = 0;
#endif

	/* Initialize the platform config for future decision making */
	ma35d1_config_setup();

	/* version B + PLM = deployed + secure boot */
#if FIP_DE_AES
	if (((mmio_read_32(SYS_BA + 0x1f0) & 0x0f000000) == 0x1000000) &&
	    ((mmio_read_32(SSPCC_BASE + 0x704) & 0xf) == 0x3)) {

		/* GPE15 internal pull-up */
		mmio_write_32(0x40460208, mmio_read_32(0x40460208) | 0x100000);
		mmio_write_32(0x40040130, (mmio_read_32(0x40040130) & 0x3fffffff) | 0x40000000);
		printf("version 0x%x, plm 0x%x\n", mmio_read_32(SYS_BA + 0x1f0), mmio_read_32(SSPCC_BASE + 0x704) & 0xf);

		/* reset RX FIFO */
		mmio_write_32(0x40700008, mmio_read_32(0x40700008) | 0x2);
		while((mmio_read_32(0x40700008) & 0x2) == 0x2);
		state = 0;
		/* 10ms timeout */
		generic_delay_timer_init();
		timeout = read_cntpct_el0();
		while (1) {
			/* RX FIFO >= 16 or RX full */
			if ((mmio_read_32(UART_FIFOSTS) & 0x3000) || (mmio_read_32(UART_FIFOSTS) & 0x8000)) {
				state = 1;
				break;
			}
			if ((read_cntpct_el0() - timeout) > 2400000) //200ms
				break;
		}
		if (state) {
		int ret;
		ret = EnterRMAState();
			if (ret)
				printf("enter RMA fail. 0x%x\n", ret);
		}
	}
#endif


	ma35d1_ddr_init();

	/*
	 * Initialize Interconnect for this cluster during cold boot.
	 * No need for locks as no other CPU is active.
	 */
	ma35d1_interconnect_init();
	/*
	 * Enable coherency in Interconnect for the primary CPU's cluster.
	 */
	ma35d1_interconnect_enable();

}
