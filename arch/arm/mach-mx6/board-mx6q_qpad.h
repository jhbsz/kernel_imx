/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _BOARD_MX6Q_QPAD_H
#define _BOARD_MX6Q_QPAD_H
#include <mach/iomux-mx6q.h>

static iomux_v3_cfg_t mx6q_qpad_pads[] = {
	/* AUDMUX */
	MX6Q_PAD_CSI0_DAT4__AUDMUX_AUD3_TXC,
	MX6Q_PAD_CSI0_DAT5__AUDMUX_AUD3_TXD,
	MX6Q_PAD_CSI0_DAT6__AUDMUX_AUD3_TXFS,
	MX6Q_PAD_CSI0_DAT7__AUDMUX_AUD3_RXD,


	/* CCM  */
	MX6Q_PAD_GPIO_0__CCM_CLKO,		/* Clock output 1: Camera MCLK */
	MX6Q_PAD_NANDF_CS2__CCM_CLKO2,	/* Clock output 2: Audio Codec MCLK*/


	/* GPIO1 */
	MX6Q_PAD_GPIO_18__GPIO_7_13,	/*PMIC INT*/

	/* GPIO2 */
	/* MX6Q_PAD_NANDF_D1__GPIO_2_1,*/	/* J14 - Menu Button */
	/* MX6Q_PAD_NANDF_D2__GPIO_2_2,*/	/* J14 - Back Button */
	/* MX6Q_PAD_NANDF_D3__GPIO_2_3,*/	/* J14 - Search Button */
	/* MX6Q_PAD_NANDF_D4__GPIO_2_4,*/	/* J14 - Home Button */
	MX6Q_PAD_EIM_A22__GPIO_2_16,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A21__GPIO_2_17,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A20__GPIO_2_18,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A19__GPIO_2_19,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A18__GPIO_2_20,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A17__GPIO_2_21,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A16__GPIO_2_22,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_RW__GPIO_2_26,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_LBA__GPIO_2_27,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_EB0__GPIO_2_28,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_EB1__GPIO_2_29,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_EB3__GPIO_2_31,	/* J12 - Boot Mode Select */

	/* GPIO3 */
	MX6Q_PAD_EIM_DA0__GPIO_3_0,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA1__GPIO_3_1,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA2__GPIO_3_2,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA3__GPIO_3_3,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA4__GPIO_3_4,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA5__GPIO_3_5,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA6__GPIO_3_6,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA7__GPIO_3_7,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA8__GPIO_3_8,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA12__GPIO_3_12,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA13__GPIO_3_13,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA14__GPIO_3_14,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA15__GPIO_3_15,	/* J12 - Boot Mode Select */

	/* GPIO5 */

	/* GPIO6 */

	/* SW4 , SW5 & SW1 */
	MX6Q_PAD_EIM_DA9__GPIO_3_9,		/* KEY1*/
	MX6Q_PAD_EIM_DA10__GPIO_3_10,	/* KEY2*/
	MX6Q_PAD_EIM_DA11__GPIO_3_11,	/* KEY3*/


	/* I2C1 */
	MX6Q_PAD_CSI0_DAT9__I2C1_SCL,
	MX6Q_PAD_CSI0_DAT8__I2C1_SDA,

	/* I2C2 */
	MX6Q_PAD_KEY_COL3__I2C2_SCL,
	MX6Q_PAD_KEY_ROW3__I2C2_SDA,

	/* I2C3 */
	MX6Q_PAD_GPIO_3__I2C3_SCL,	
	MX6Q_PAD_GPIO_6__I2C3_SDA,

	/* DISPLAY ,MIPI is proprietary PINs*/


	/* PWM */
	MX6Q_PAD_SD1_DAT3__PWM1_PWMO,		/* GPIO1[21] */

	/* UART */
	MX6Q_PAD_CSI0_DAT10__UART1_TXD,
	MX6Q_PAD_CSI0_DAT11__UART1_RXD,
	MX6Q_PAD_EIM_D26__UART2_TXD,
	MX6Q_PAD_EIM_D27__UART2_RXD,
	MX6Q_PAD_EIM_D24__UART3_TXD,
	MX6Q_PAD_EIM_D25__UART3_RXD,
	MX6Q_PAD_KEY_COL0__UART4_TXD,
	MX6Q_PAD_KEY_ROW0__UART4_RXD,
	MX6Q_PAD_KEY_COL1__UART5_TXD,
	MX6Q_PAD_KEY_ROW1__UART5_RXD,

	/* USB OTG */
	MX6Q_PAD_GPIO_1__USBOTG_ID,
//	MX6Q_PAD_EIM_D22__USBOH3_USBOTG_PWR,

	/* USDHC2 */
	MX6Q_PAD_SD2_CLK__USDHC2_CLK,
	MX6Q_PAD_SD2_CMD__USDHC2_CMD,
	MX6Q_PAD_SD2_DAT0__USDHC2_DAT0,
	MX6Q_PAD_SD2_DAT1__USDHC2_DAT1,
	MX6Q_PAD_SD2_DAT2__USDHC2_DAT2,
	MX6Q_PAD_SD2_DAT3__USDHC2_DAT3,

	/* USDHC3 4bit*/
	MX6Q_PAD_SD3_CLK__USDHC3_CLK_50MHZ,
	MX6Q_PAD_SD3_CMD__USDHC3_CMD_50MHZ,
	MX6Q_PAD_SD3_DAT0__USDHC3_DAT0_50MHZ,
	MX6Q_PAD_SD3_DAT1__USDHC3_DAT1_50MHZ,
	MX6Q_PAD_SD3_DAT2__USDHC3_DAT2_50MHZ,
	MX6Q_PAD_SD3_DAT3__USDHC3_DAT3_50MHZ,

	/* USDHC4 8bit*/
	MX6Q_PAD_SD4_CLK__USDHC4_CLK_50MHZ,
	MX6Q_PAD_SD4_CMD__USDHC4_CMD_50MHZ,
	MX6Q_PAD_SD4_DAT0__USDHC4_DAT0_50MHZ,
	MX6Q_PAD_SD4_DAT1__USDHC4_DAT1_50MHZ,
	MX6Q_PAD_SD4_DAT2__USDHC4_DAT2_50MHZ,
	MX6Q_PAD_SD4_DAT3__USDHC4_DAT3_50MHZ,
	MX6Q_PAD_SD4_DAT4__USDHC4_DAT4_50MHZ,
	MX6Q_PAD_SD4_DAT5__USDHC4_DAT5_50MHZ,
	MX6Q_PAD_SD4_DAT6__USDHC4_DAT6_50MHZ,
	MX6Q_PAD_SD4_DAT7__USDHC4_DAT7_50MHZ,

	/* Charger */
	MX6Q_PAD_EIM_A25__GPIO_5_2,  /* FLT_1_B */
	MX6Q_PAD_EIM_D23__GPIO_3_23, /* CHG_1_B */
	MX6Q_PAD_EIM_D17__GPIO_3_17,  /* UOK_B */
	MX6Q_PAD_EIM_CS1__GPIO_2_24,   /* DOK_B */

	/* Audio Codec */
	MX6Q_PAD_KEY_COL2__GPIO_4_10,		/* CODEC_PWR_EN */
	MX6Q_PAD_SD3_RST__GPIO_7_8,			/* HEADPHONE_DET */
	MX6Q_PAD_GPIO_9__GPIO_1_9,			/* MICROPHONE_DET */


};

static iomux_v3_cfg_t mx6q_sabresd_csi0_sensor_pads[] = {
	/* IPU1 Camera */
	MX6Q_PAD_CSI0_DAT12__IPU1_CSI0_D_12,
	MX6Q_PAD_CSI0_DAT13__IPU1_CSI0_D_13,
	MX6Q_PAD_CSI0_DAT14__IPU1_CSI0_D_14,
	MX6Q_PAD_CSI0_DAT15__IPU1_CSI0_D_15,
	MX6Q_PAD_CSI0_DAT16__IPU1_CSI0_D_16,
	MX6Q_PAD_CSI0_DAT17__IPU1_CSI0_D_17,
	MX6Q_PAD_CSI0_DAT18__IPU1_CSI0_D_18,
	MX6Q_PAD_CSI0_DAT19__IPU1_CSI0_D_19,
	MX6Q_PAD_CSI0_DATA_EN__IPU1_CSI0_DATA_EN,
	MX6Q_PAD_CSI0_MCLK__IPU1_CSI0_HSYNC,
	MX6Q_PAD_CSI0_PIXCLK__IPU1_CSI0_PIXCLK,
	MX6Q_PAD_CSI0_VSYNC__IPU1_CSI0_VSYNC,

	MX6Q_PAD_GPIO_0__CCM_CLKO,		/* camera clk */

	MX6Q_PAD_SD1_DAT0__GPIO_1_16,		/* camera PWDN */
	MX6Q_PAD_SD1_DAT1__GPIO_1_17,		/* camera RESET */
};

static iomux_v3_cfg_t mx6q_sabresd_mipi_sensor_pads[] = {
	MX6Q_PAD_GPIO_0__CCM_CLKO,		/* camera clk */

	MX6Q_PAD_SD1_DAT2__GPIO_1_19,		/* camera PWDN */
	MX6Q_PAD_SD1_CLK__GPIO_1_20,		/* camera RESET */
};


#endif
