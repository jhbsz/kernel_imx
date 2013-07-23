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

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/ion.h>
#include <linux/etherdevice.h>
#include <linux/power/sabresd_battery.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/max17135.h>
#include <linux/mfd/wm8994/pdata.h>
#include <linux/mfd/wm8994/gpio.h>
#include <sound/wm8962.h>
#include <linux/mfd/mxc-hdmi-core.h>
#include <linux/wlan_plat.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>
#include <mach/mipi_dsi.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-mx6q_sparkauto.h"
#include "board-mx6dl_sparkauto.h"
#include <mach/imx_rfkill.h>

#define SABRESD_USR_DEF_GRN_LED	IMX_GPIO_NR(1, 1)
#define SABRESD_BT_RESET	IMX_GPIO_NR(1, 2)
#define SABRESD_USR_DEF_RED_LED	IMX_GPIO_NR(1, 2)
#define SABRESD_VOLUME_UP	IMX_GPIO_NR(1, 4)
#define SABRESD_VOLUME_DN	IMX_GPIO_NR(1, 5)
#define SABRESD_MICROPHONE_DET	IMX_GPIO_NR(1, 9)
#define SABRESD_CSI0_PWN	IMX_GPIO_NR(1, 16)
#define SABRESD_CSI0_RST	IMX_GPIO_NR(1, 17)
#define SABRESD_ACCL_INT	IMX_GPIO_NR(1, 18)
#define SABRESD_MIPICSI_PWN	IMX_GPIO_NR(1, 19)
#define SABRESD_MIPICSI_RST	IMX_GPIO_NR(1, 20)
#define SABRESD_RGMII_RST	IMX_GPIO_NR(1, 25)
#define SABRESD_RGMII_INT	IMX_GPIO_NR(1, 26)
#define SABRESD_CHARGE_UOK_B	IMX_GPIO_NR(1, 27)
#define SABRESD_USBH1_PWR_EN	IMX_GPIO_NR(1, 29)
#define SABRESD_DISP0_PWR_EN	IMX_GPIO_NR(1, 30)

#define SABRESD_SD3_CD		IMX_GPIO_NR(7, 8)
#define SABRESD_SD3_WP		IMX_GPIO_NR(2, 1)
#define SABRESD_CHARGE_DOK_B	IMX_GPIO_NR(2, 24)
#define SABRESD_GPS_RESET	IMX_GPIO_NR(2, 28)
#define SABRESD_SENSOR_EN	IMX_GPIO_NR(2, 31)

#define SABRESD_GPS_EN	IMX_GPIO_NR(3, 0)
#define SABRESD_DISP0_RST_B	IMX_GPIO_NR(3, 8)
#define SABRESD_ALS_INT		IMX_GPIO_NR(3, 9)
#define SABRESD_CHARGE_CHG_2_B	IMX_GPIO_NR(3, 13)
#define SABRESD_CHARGE_FLT_2_B	IMX_GPIO_NR(3, 14)
#define SABRESD_BAR0_INT	IMX_GPIO_NR(3, 15)
#define SABRESD_eCOMPASS_INT	IMX_GPIO_NR(3, 16)
#define SABRESD_GPS_PPS		IMX_GPIO_NR(3, 18)
#define SABRESD_PCIE_PWR_EN	IMX_GPIO_NR(3, 19)
#define SABRESD_USB_OTG_PWR	IMX_GPIO_NR(3, 22)
#define SABRESD_USB_H1_PWR	IMX_GPIO_NR(1, 29)
#define SABRESD_CHARGE_CHG_1_B	IMX_GPIO_NR(3, 23)
#define SABRESD_TS_INT		IMX_GPIO_NR(3, 26)
#define SABRESD_DISP0_RD	IMX_GPIO_NR(3, 28)
#define SABRESD_POWER_OFF	IMX_GPIO_NR(3, 29)

#define SABRESD_CAN1_STBY	IMX_GPIO_NR(4, 5)
#define SABRESD_ECSPI1_CS0  IMX_GPIO_NR(4, 9)
#define SABRESD_CODEC_PWR_EN	IMX_GPIO_NR(4, 10)
#define SABRESD_HDMI_CEC_IN	IMX_GPIO_NR(4, 11)
#define SABRESD_PCIE_DIS_B	IMX_GPIO_NR(4, 14)

#define SABRESD_DI0_D0_CS	IMX_GPIO_NR(5, 0)
#define SABRESD_CHARGE_FLT_1_B	IMX_GPIO_NR(5, 2)
#define SABRESD_PCIE_WAKE_B	IMX_GPIO_NR(5, 20)

#define SABRESD_CAP_TCH_INT1	IMX_GPIO_NR(6, 7)
#define SABRESD_CAP_TCH_INT0	IMX_GPIO_NR(6, 8)
#define SABRESD_DISP_RST_B	IMX_GPIO_NR(6, 11)
#define SABRESD_DISP_PWR_EN	IMX_GPIO_NR(6, 14)
#define SABRESD_CABC_EN0	IMX_GPIO_NR(6, 15)
#define SABRESD_CABC_EN1	IMX_GPIO_NR(6, 16)
#define SABRESD_AUX_3V15_EN	IMX_GPIO_NR(6, 9)
#define SABRESD_DISP0_WR_REVB	IMX_GPIO_NR(6, 9)
#define SABRESD_AUX_5V_EN	IMX_GPIO_NR(6, 10)
#define SABRESD_DI1_D0_CS	IMX_GPIO_NR(6, 31)

#define SABRESD_HEADPHONE_DET	IMX_GPIO_NR(7, 8)
#define SABRESD_PCIE_RST_B_REVB	IMX_GPIO_NR(7, 12)
#define SABRESD_PMIC_INT_B	IMX_GPIO_NR(7, 13)
#define SABRESD_PFUZE_INT	IMX_GPIO_NR(7, 13)


#define SABRESD_EPDC_PMIC_WAKE	IMX_GPIO_NR(3, 20)
#define SABRESD_EPDC_PMIC_INT	IMX_GPIO_NR(2, 25)
#define SABRESD_EPDC_VCOM	IMX_GPIO_NR(3, 17)
#define SABRESD_CHARGE_NOW	IMX_GPIO_NR(1, 2)
#define SABRESD_CHARGE_DONE	IMX_GPIO_NR(1, 1)

#define BL_ON				IMX_GPIO_NR(4, 10)
#define LCD_POWER			IMX_GPIO_NR(4, 11)
#define MX6_VDD_3V3_EN		IMX_GPIO_NR(3, 26)
#define MX6_VDD_5V_EN		IMX_GPIO_NR(6, 15)

#define WIFI_PWR			IMX_GPIO_NR(2, 29)
#define WIFI_HOST_WAKE		IMX_GPIO_NR(7, 8)
#define WIFI_RESET			IMX_GPIO_NR(6, 18)


#define FEC_PHY_ADDR		IMX_GPIO_NR(1, 24)
#define FEC_PHY_RESET 		IMX_GPIO_NR(5, 4)


#define GPIO_KEY_VOLUP		IMX_GPIO_NR(3,8)
#define GPIO_KEY_VOLDOWN	IMX_GPIO_NR(2,23)
#define GPIO_KEY_RETURN		IMX_GPIO_NR(2,25)


#define SPARKAUTO_SD1_CD		IMX_GPIO_NR(5, 2)
#define SPARKAUTO_SD1_WP		IMX_GPIO_NR(6, 31)


#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
#define MX6_ENET_IRQ		IMX_GPIO_NR(1, 6)
#define IOMUX_OBSRV_MUX1_OFFSET	0x3c
#define OBSRV_MUX1_MASK			0x3f
#define OBSRV_MUX1_ENET_IRQ		0x9
#endif

static struct clk *sata_clk;
static int mag3110_position = 1;
static int caam_enabled;

extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;

#define MX6Q_USDHC_PAD_SETTING(id, speed)	\
mx6q_sd##id##_##speed##mhz[] = {		\
	MX6Q_PAD_SD##id##_CLK__USDHC##id##_CLK_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_CMD__USDHC##id##_CMD_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT0__USDHC##id##_DAT0_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT1__USDHC##id##_DAT1_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT2__USDHC##id##_DAT2_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT3__USDHC##id##_DAT3_##speed##MHZ,	\
}

static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(1, 50);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(1, 100);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(1, 200);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(3, 50);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(3, 100);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(3, 200);

enum sd_pad_mode {
	SD_PAD_MODE_LOW_SPEED,
	SD_PAD_MODE_MED_SPEED,
	SD_PAD_MODE_HIGH_SPEED,
};

static int plt_sd_pad_change(unsigned int index, int clock)
{
	/* LOW speed is the default state of SD pads */
	static enum sd_pad_mode pad_mode = SD_PAD_MODE_LOW_SPEED;

	iomux_v3_cfg_t *sd_pads_200mhz = NULL;
	iomux_v3_cfg_t *sd_pads_100mhz = NULL;
	iomux_v3_cfg_t *sd_pads_50mhz = NULL;

	u32 sd_pads_200mhz_cnt;
	u32 sd_pads_100mhz_cnt;
	u32 sd_pads_50mhz_cnt;

	printk("%s slot%d to cll %dkHz\n",__func__,index,clock/1000);

	switch (index) {
	case 2:
		sd_pads_200mhz = mx6q_sd3_200mhz;
		sd_pads_100mhz = mx6q_sd3_100mhz;
		sd_pads_50mhz = mx6q_sd3_50mhz;

		sd_pads_200mhz_cnt = ARRAY_SIZE(mx6q_sd3_200mhz);
		sd_pads_100mhz_cnt = ARRAY_SIZE(mx6q_sd3_100mhz);
		sd_pads_50mhz_cnt = ARRAY_SIZE(mx6q_sd3_50mhz);
		break;
	case 0:
		sd_pads_200mhz = mx6q_sd1_200mhz;
		sd_pads_100mhz = mx6q_sd1_100mhz;
		sd_pads_50mhz = mx6q_sd1_50mhz;

		sd_pads_200mhz_cnt = ARRAY_SIZE(mx6q_sd1_200mhz);
		sd_pads_100mhz_cnt = ARRAY_SIZE(mx6q_sd1_100mhz);
		sd_pads_50mhz_cnt = ARRAY_SIZE(mx6q_sd1_50mhz);
		break;
	default:
		printk(KERN_ERR "no such SD host controller index %d\n", index);
		return -EINVAL;
	}

	if (clock > 100000000) {
		if (pad_mode == SD_PAD_MODE_HIGH_SPEED)
			return 0;
		BUG_ON(!sd_pads_200mhz);
		pad_mode = SD_PAD_MODE_HIGH_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_200mhz,
							sd_pads_200mhz_cnt);
	} else if (clock > 52000000) {
		if (pad_mode == SD_PAD_MODE_MED_SPEED)
			return 0;
		BUG_ON(!sd_pads_100mhz);
		pad_mode = SD_PAD_MODE_MED_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_100mhz,
							sd_pads_100mhz_cnt);
	} else {
		if (pad_mode == SD_PAD_MODE_LOW_SPEED)
			return 0;
		BUG_ON(!sd_pads_50mhz);
		pad_mode = SD_PAD_MODE_LOW_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_50mhz,
							sd_pads_50mhz_cnt);
	}
}


/*Micro SD*/
static const struct esdhc_platform_data mx6q_sd1_data __initconst = {	
	.cd_gpio = SPARKAUTO_SD1_CD,
	.keep_power_at_suspend = 1,
	.platform_pad_change = plt_sd_pad_change,
	.cd_type 			= ESDHC_CD_CONTROLLER,
};

/*WIFI SDIO*/
static const struct esdhc_platform_data mx6q_sd3_data __initconst = {
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.cd_type = ESDHC_CD_PERMANENT,
	//.platform_pad_change = plt_sd_pad_change,
};

/*eMMC*/
static const struct esdhc_platform_data mx6q_sd4_data __initconst = {
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.support_8bit = 1,
	.delay_line = 0,
	.cd_type = ESDHC_CD_PERMANENT,
};

static const struct anatop_thermal_platform_data
	mx6q_sabresd_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

static const struct imxuart_platform_data mx6q_uart3_data __initconst = {
	.flags      = IMXUART_HAVE_RTSCTS,
	.dma_req_rx = MX6Q_DMA_REQ_UART3_RX,
	.dma_req_tx = MX6Q_DMA_REQ_UART3_TX,
};

static inline void mx6q_sparkauto_init_uart(void)
{	
	imx6q_add_imx_uart(0, NULL);	
	imx6q_add_imx_uart(1, NULL);
	imx6q_add_imx_uart(2, &mx6q_uart3_data);
	imx6q_add_imx_uart(3, NULL);
	imx6q_add_imx_uart(4, NULL);	
}

static void max6q_fec_reset(void){
	/* reset phy */
	gpio_request(FEC_PHY_RESET, "phy-rst");
	gpio_direction_output(FEC_PHY_RESET, 0);
	mdelay(1);
	gpio_direction_output(FEC_PHY_RESET, 1);
	gpio_free(FEC_PHY_RESET);
}
static int mx6q_fec_phy_init(struct phy_device *phydev)
{
	int val;

	/* set phy addr to 0 */
	gpio_request(FEC_PHY_ADDR, "phy-addr");
	gpio_direction_output(FEC_PHY_ADDR, 0);
	gpio_free(FEC_PHY_ADDR);

	/* reset phy */
	max6q_fec_reset();
	/* check phy power */
	val = phy_read(phydev, 0x0);
	if (val & BMCR_PDOWN) {
		phy_write(phydev, 0x0, (val & ~BMCR_PDOWN));
	}

	return 0;
}
static struct fec_platform_data fec_data __initdata = {
	.init = mx6q_fec_phy_init,
	.phy = PHY_INTERFACE_MODE_RMII,
};


static int mx6q_sabresd_spi_cs[] = {
	SABRESD_ECSPI1_CS0,
};

static const struct spi_imx_master mx6q_sabresd_spi_data __initconst = {
	.chipselect     = mx6q_sabresd_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6q_sabresd_spi_cs),
};

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct mtd_partition imx6_sabresd_spi_nor_partitions[] = {
	{
	 .name = "bootloader",
	 .offset = 0,
	 .size = 0x00100000,
	},
	{
	 .name = "kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL,
	},
};

static struct flash_platform_data imx6_sabresd__spi_flash_data = {
	.name = "m25p80",
	.parts = imx6_sabresd_spi_nor_partitions,
	.nr_parts = ARRAY_SIZE(imx6_sabresd_spi_nor_partitions),
	.type = "sst25vf016b",
};
#endif

static struct spi_board_info imx6_sabresd_spi_nor_device[] __initdata = {
#if defined(CONFIG_MTD_M25P80)
	{
		.modalias = "m25p80",
		.max_speed_hz = 20000000, /* max spi clock (SCK) speed in HZ */
		.bus_num = 0,
		.chip_select = 0,
		.platform_data = &imx6_sabresd__spi_flash_data,
	},
#endif
};

static void spi_device_init(void)
{
	spi_register_board_info(imx6_sabresd_spi_nor_device,
				ARRAY_SIZE(imx6_sabresd_spi_nor_device));
}

static struct imx_ssi_platform_data mx6_sabresd_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct platform_device mx6_sabresd_audio_rt5633_device = {
	.name = "imx-rt5633",
};

static struct mxc_audio_platform_data rt5633_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
//	.hp_gpio = SABRESD_HEADPHONE_DET,
//	.hp_active_low = 1,
};



static void mx6q_csi0_cam_powerdown(int powerdown)
{
	if (powerdown)
		gpio_set_value(SABRESD_CSI0_PWN, 1);
	else
		gpio_set_value(SABRESD_CSI0_PWN, 0);

	msleep(2);
}

static void mx6q_csi0_io_init(void)
{
	if (cpu_is_mx6q())
		mxc_iomux_v3_setup_multiple_pads(mx6q_sabresd_csi0_sensor_pads,
			ARRAY_SIZE(mx6q_sabresd_csi0_sensor_pads));
	else if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_sabresd_csi0_sensor_pads,
			ARRAY_SIZE(mx6dl_sabresd_csi0_sensor_pads));

	/* Camera reset */
	gpio_request(SABRESD_CSI0_RST, "cam-reset");
	gpio_direction_output(SABRESD_CSI0_RST, 1);

	/* Camera power down */
	gpio_request(SABRESD_CSI0_PWN, "cam-pwdn");
	gpio_direction_output(SABRESD_CSI0_PWN, 1);
	msleep(5);
	gpio_set_value(SABRESD_CSI0_PWN, 0);
	msleep(5);
	gpio_set_value(SABRESD_CSI0_RST, 0);
	msleep(1);
	gpio_set_value(SABRESD_CSI0_RST, 1);
	msleep(5);
	gpio_set_value(SABRESD_CSI0_PWN, 1);

	/* For MX6Q:
	 * GPR1 bit19 and bit20 meaning:
	 * Bit19:       0 - Enable mipi to IPU1 CSI0
	 *                      virtual channel is fixed to 0
	 *              1 - Enable parallel interface to IPU1 CSI0
	 * Bit20:       0 - Enable mipi to IPU2 CSI1
	 *                      virtual channel is fixed to 3
	 *              1 - Enable parallel interface to IPU2 CSI1
	 * IPU1 CSI1 directly connect to mipi csi2,
	 *      virtual channel is fixed to 1
	 * IPU2 CSI0 directly connect to mipi csi2,
	 *      virtual channel is fixed to 2
	 *
	 * For MX6DL:
	 * GPR13 bit 0-2 IPU_CSI0_MUX
	 *   000 MIPI_CSI0
	 *   100 IPU CSI0
	 */
	if (cpu_is_mx6q())
		mxc_iomux_set_gpr_register(1, 19, 1, 1);
	else if (cpu_is_mx6dl())
		mxc_iomux_set_gpr_register(13, 0, 3, 4);
}

static struct fsl_mxc_camera_platform_data camera_data = {
	.mclk = 12000000,
	.mclk_source = 0,
	.csi = 0,
	.io_init = mx6q_csi0_io_init,
	.pwdn = mx6q_csi0_cam_powerdown,
};

static void mx6q_mipi_powerdown(int powerdown)
{
	if (powerdown)
		gpio_set_value(SABRESD_MIPICSI_PWN, 1);
	else
		gpio_set_value(SABRESD_MIPICSI_PWN, 0);

	msleep(2);
}

static void mx6q_mipi_sensor_io_init(void)
{
	if (cpu_is_mx6q())
		mxc_iomux_v3_setup_multiple_pads(mx6q_sabresd_mipi_sensor_pads,
			ARRAY_SIZE(mx6q_sabresd_mipi_sensor_pads));
	else if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_sabresd_mipi_sensor_pads,
			ARRAY_SIZE(mx6dl_sabresd_mipi_sensor_pads));

	/* Camera reset */
	gpio_request(SABRESD_MIPICSI_RST, "cam-reset");
	gpio_direction_output(SABRESD_MIPICSI_RST, 1);

	/* Camera power down */
	gpio_request(SABRESD_MIPICSI_PWN, "cam-pwdn");
	gpio_direction_output(SABRESD_MIPICSI_PWN, 1);
	msleep(5);
	gpio_set_value(SABRESD_MIPICSI_PWN, 0);
	msleep(5);
	gpio_set_value(SABRESD_MIPICSI_RST, 0);
	msleep(1);
	gpio_set_value(SABRESD_MIPICSI_RST, 1);
	msleep(5);
	gpio_set_value(SABRESD_MIPICSI_PWN, 1);

	/*for mx6dl, mipi virtual channel 1 connect to csi 1*/
	if (cpu_is_mx6dl())
		mxc_iomux_set_gpr_register(13, 3, 3, 1);
}

static struct fsl_mxc_camera_platform_data mipi_csi2_data = {
	.mclk = 24000000,
	.mclk_source = 0,
	.csi = 1,
	.io_init = mx6q_mipi_sensor_io_init,
	.pwdn = mx6q_mipi_powerdown,
};


static struct imxi2c_platform_data mx6q_sabresd_i2c_data = {
	.bitrate = 100000,
};

static struct fsl_mxc_lightsensor_platform_data ls_data = {
	.rext = 499,	/* calibration: 499K->700K */
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("wm89**", 0x1a),
	},
	{
		I2C_BOARD_INFO("rt5633", 0x1c),
	},
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
	{
		I2C_BOARD_INFO("ov5640", 0x3c),
		.platform_data = (void *)&camera_data,
	},	
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("egalax_ts", 0x4),
		.irq = gpio_to_irq(SABRESD_CAP_TCH_INT1),
	},
	{
		I2C_BOARD_INFO("mag3110", 0x0e),
		.irq = gpio_to_irq(SABRESD_eCOMPASS_INT),
		.platform_data = (void *)&mag3110_position,
	},
	{
		I2C_BOARD_INFO("isl29023", 0x44),
		.irq  = gpio_to_irq(SABRESD_ALS_INT),
		.platform_data = &ls_data,
	}, 
	{
		I2C_BOARD_INFO("mxc_ldb_i2c", 0x50),
		.platform_data = (void *)0,
	},
};


static void imx6q_sabresd_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(SABRESD_USB_OTG_PWR, 1);
	else
		gpio_set_value(SABRESD_USB_OTG_PWR, 0);
}

static void imx6q_sabresd_host1_vbus(bool on)
{
	if (on)
		gpio_set_value(SABRESD_USB_H1_PWR, 1);
	else
		gpio_set_value(SABRESD_USB_H1_PWR, 0);
}

static void __init imx6q_sabresd_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	/* disable external charger detect,
	 * or it will affect signal quality at dp .
	 */
	ret = gpio_request(SABRESD_USB_OTG_PWR, "usb-pwr");
	if (ret) {
		pr_err("failed to get GPIO SABRESD_USB_OTG_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(SABRESD_USB_OTG_PWR, 0);
	/* keep USB host1 VBUS always on */
	ret = gpio_request(SABRESD_USB_H1_PWR, "usb-h1-pwr");
	if (ret) {
		pr_err("failed to get GPIO SABRESD_USB_H1_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(SABRESD_USB_H1_PWR, 1);
	if (board_is_mx6_reva())
		mxc_iomux_set_gpr_register(1, 13, 1, 1);
	else
		mxc_iomux_set_gpr_register(1, 13, 1, 1);

	mx6_set_otghost_vbus_func(imx6q_sabresd_usbotg_vbus);
	mx6_set_host1_vbus_func(imx6q_sabresd_host1_vbus);

}

/* HW Initialization, if return 0, initialization is successful. */
static int mx6q_sabresd_sata_init(struct device *dev, void __iomem *addr)
{
	u32 tmpdata;
	int ret = 0;
	struct clk *clk;

	sata_clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(sata_clk)) {
		dev_err(dev, "no sata clock.\n");
		return PTR_ERR(sata_clk);
	}
	ret = clk_enable(sata_clk);
	if (ret) {
		dev_err(dev, "can't enable sata clock.\n");
		goto put_sata_clk;
	}

	/* Set PHY Paremeters, two steps to configure the GPR13,
	 * one write for rest of parameters, mask of first write is 0x07FFFFFD,
	 * and the other one write for setting the mpll_clk_off_b
	 *.rx_eq_val_0(iomuxc_gpr13[26:24]),
	 *.los_lvl(iomuxc_gpr13[23:19]),
	 *.rx_dpll_mode_0(iomuxc_gpr13[18:16]),
	 *.sata_speed(iomuxc_gpr13[15]),
	 *.mpll_ss_en(iomuxc_gpr13[14]),
	 *.tx_atten_0(iomuxc_gpr13[13:11]),
	 *.tx_boost_0(iomuxc_gpr13[10:7]),
	 *.tx_lvl(iomuxc_gpr13[6:2]),
	 *.mpll_ck_off(iomuxc_gpr13[1]),
	 *.tx_edgerate_0(iomuxc_gpr13[0]),
	 */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x07FFFFFD) | 0x0593A044), IOMUXC_GPR13);

	/* enable SATA_PHY PLL */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x2) | 0x2), IOMUXC_GPR13);

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb");
	if (IS_ERR(clk)) {
		dev_err(dev, "no ahb clock.\n");
		ret = PTR_ERR(clk);
		goto release_sata_clk;
	}
	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

#ifdef CONFIG_SATA_AHCI_PLATFORM
	ret = sata_init(addr, tmpdata);
	if (ret == 0)
		return ret;
#else
	usleep_range(1000, 2000);
	/* AHCI PHY enter into PDDQ mode if the AHCI module is not enabled */
	tmpdata = readl(addr + PORT_PHY_CTL);
	writel(tmpdata | PORT_PHY_CTL_PDDQ_LOC, addr + PORT_PHY_CTL);
	pr_info("No AHCI save PWR: PDDQ %s\n", ((readl(addr + PORT_PHY_CTL)
					>> 20) & 1) ? "enabled" : "disabled");
#endif

release_sata_clk:
	/* disable SATA_PHY PLL */
	writel((readl(IOMUXC_GPR13) & ~0x2), IOMUXC_GPR13);
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);

	return ret;
}

#ifdef CONFIG_SATA_AHCI_PLATFORM
static void mx6q_sabresd_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);
}

static struct ahci_platform_data mx6q_sabresd_sata_data = {
	.init = mx6q_sabresd_sata_init,
	.exit = mx6q_sabresd_sata_exit,
};
#endif

static void mx6q_sabresd_flexcan0_switch(int enable)
{
	if (enable) {
		gpio_set_value(SABRESD_CAN1_STBY, 1);
	} else {
		gpio_set_value(SABRESD_CAN1_STBY, 0);
	}
}

static const struct flexcan_platform_data
	mx6q_sabresd_flexcan0_pdata __initconst = {
	.transceiver_switch = mx6q_sabresd_flexcan0_switch,
};

static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M + SZ_64M - SZ_16M,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static void mx6_reset_mipi_dsi(void)
{
	gpio_set_value(SABRESD_DISP_PWR_EN, 1);
	gpio_set_value(SABRESD_DISP_RST_B, 1);
	udelay(10);
	gpio_set_value(SABRESD_DISP_RST_B, 0);
	udelay(50);
	gpio_set_value(SABRESD_DISP_RST_B, 1);

	/*
	 * it needs to delay 120ms minimum for reset complete
	 */
	msleep(120);
}

static struct mipi_dsi_platform_data mipi_dsi_pdata = {
	.ipu_id		= 0,
	.disp_id	= 1,
	.lcd_panel	= "TRULY-WVGA",
	.reset		= mx6_reset_mipi_dsi,
};

static struct ipuv3_fb_platform_data sparkauto_fb_data[] = {
	{ /*fb0*/
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	}, {
	.disp_dev = "hdmi",
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
	.mode_str = "1920x1080M@60",
	.default_bpp = 32,
	.int_clk = false,
	.late_init = false,
	}, {
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	},
};

static void hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;

	if ((ipu_id > 1) || (ipu_id < 0)) {
		pr_err("Invalid IPU select for HDMI: %d. Set to 0\n", ipu_id);
		ipu_id = 0;
	}

	if ((disp_id > 1) || (disp_id < 0)) {
		pr_err("Invalid DI select for HDMI: %d. Set to 0\n", disp_id);
		disp_id = 0;
	}

	/* Configure the connection between IPU1/2 and HDMI */
	hdmi_mux_setting = 2*ipu_id + disp_id;

	/* GPR3, bits 2-3 = HDMI_MUX_CTL */
	mxc_iomux_set_gpr_register(3, 2, 2, hdmi_mux_setting);

	/* Set HDMI event as SDMA event2 while Chip version later than TO1.2 */
	if (hdmi_SDMA_check())
		mxc_iomux_set_gpr_register(0, 0, 1, 1);
}

/* On mx6x sabresd board i2c2 iomux with hdmi ddc,
 * the pins default work at i2c2 function,
 when hdcp enable, the pins should work at ddc function */

static void hdmi_enable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_sabresd_hdmi_ddc_pads,
			ARRAY_SIZE(mx6dl_sabresd_hdmi_ddc_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_sabresd_hdmi_ddc_pads,
			ARRAY_SIZE(mx6q_sabresd_hdmi_ddc_pads));
}

static void hdmi_disable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_sabresd_i2c2_pads,
			ARRAY_SIZE(mx6dl_sabresd_i2c2_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_sabresd_i2c2_pads,
			ARRAY_SIZE(mx6q_sabresd_i2c2_pads));
}

static struct fsl_mxc_hdmi_platform_data hdmi_data = {
	.init = hdmi_init,
	.enable_pins = hdmi_enable_ddc_pin,
	.disable_pins = hdmi_disable_ddc_pin,
};

static struct fsl_mxc_hdmi_core_platform_data hdmi_core_data = {
	.ipu_id = 1,
	.disp_id = 0,
};

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_RGB565,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.ext_ref = 1,
	.mode = LDB_SEP0,
	.sec_ipu_id = 0,
	.sec_disp_id = 1,
};

static struct max8903_pdata charger1_data = {
	.dok = SABRESD_CHARGE_DOK_B,
	.uok = SABRESD_CHARGE_UOK_B,
	.chg = SABRESD_CHARGE_CHG_1_B,
	.flt = SABRESD_CHARGE_FLT_1_B,
	.dcm_always_high = true,
	.dc_valid = true,
	.usb_valid = true,
};

static struct platform_device sabresd_max8903_charger_1 = {
	.name	= "max8903-charger",
	.id	= 1,
	.dev	= {
		.platform_data = &charger1_data,
	},
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	}, {
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	},
};

static struct ion_platform_data imx_ion_data = {
	.nr = 1,
	.heaps = {
		{
		.id = 0,
		.type = ION_HEAP_TYPE_CARVEOUT,
		.name = "vpu_ion",
		.size = SZ_16M,
		.cacheable = 1,
		},
	},
};

static struct fsl_mxc_capture_platform_data capture_data[] = {
	{
		.csi = 0,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 0,
	}, {
		.csi = 1,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 1,
	},
};

static void mx6q_sd_bt_reset(void)
{
	printk(KERN_INFO "mx6q_sd_bt_reset");
	gpio_request(SABRESD_BT_RESET, "bt-reset");
	gpio_direction_output(SABRESD_BT_RESET, 0);
	/* pull down reset pin at least >5ms */
	mdelay(6);
	/* pull up after power supply BT */
	gpio_direction_output(SABRESD_BT_RESET, 1);
	gpio_free(SABRESD_BT_RESET);
	msleep(100);
}

static int mx6q_sd_bt_power_change(int status)
{
	if (status)
		mx6q_sd_bt_reset();
	return 0;
}

static struct platform_device mxc_bt_rfkill = {
	.name = "mxc_bt_rfkill",
};

static struct imx_bt_rfkill_platform_data mxc_bt_rfkill_data = {
	.power_change = mx6q_sd_bt_power_change,
};
static void sabresd_suspend_enter(void)
{
	/* suspend preparation */
	/* Disable AUX 5V */
	gpio_set_value(SABRESD_AUX_5V_EN, 0);
}

static void sabresd_suspend_exit(void)
{
	/* resume restore */
	/* Enable AUX 5V */
	gpio_set_value(SABRESD_AUX_5V_EN, 1);
}
static const struct pm_platform_data mx6q_sabresd_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = sabresd_suspend_enter,
	.suspend_exit = sabresd_suspend_exit,
};

static struct regulator_consumer_supply sabresd_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data sabresd_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(sabresd_vmmc_consumers),
	.consumer_supplies = sabresd_vmmc_consumers,
};

static struct fixed_voltage_config sabresd_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &sabresd_vmmc_init,
};

static struct platform_device sabresd_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &sabresd_vmmc_reg_config,
	},
};

static int __init imx6q_init_audio(void)
{
	mxc_register_device(&mx6_sabresd_audio_rt5633_device,
				    &rt5633_data);
	imx6q_add_imx_ssi(1, &mx6_sabresd_ssi_pdata);

	//mxc_rt5633_init();
	return 0;
}

#if defined(CONFIG_LEDS_TRIGGER) || defined(CONFIG_LEDS_GPIO)

#define GPIO_LED(gpio_led, name_led, act_low, state_suspend, trigger)	\
{									\
	.gpio			= gpio_led,				\
	.name			= name_led,				\
	.active_low		= act_low,				\
	.retain_state_suspended = state_suspend,			\
	.default_state		= 0,					\
	.default_trigger	= "max8903-"trigger,		\
}

/* use to show a external power source is connected
 * GPIO_LED(SABRESD_CHARGE_DONE, "chg_detect", 0, 1, "ac-online"),
 */
static struct gpio_led imx6q_gpio_leds[] = {
	GPIO_LED(SABRESD_CHARGE_NOW, "chg_now_led", 0, 1,
		"charger-charging"),
/* For the latest B4 board, this GPIO_1 is connected to POR_B,
which will reset the whole board if this pin's level is changed,
so, for the latest board, we have to avoid using this pin as
GPIO.
	GPIO_LED(SABRESD_CHARGE_DONE, "chg_done_led", 0, 1,
			"charger-full"),
*/
};

static struct gpio_led_platform_data imx6q_gpio_leds_data = {
	.leds		= imx6q_gpio_leds,
	.num_leds	= ARRAY_SIZE(imx6q_gpio_leds),
};

static struct platform_device imx6q_gpio_led_device = {
	.name		= "leds-gpio",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &imx6q_gpio_leds_data,
	}
};

/* For BT_PWD_L is conflict with charger's LED trigger gpio on sabresd_revC.
 * add mutual exclusion here to be decided which one to be used by board config
 */
static void __init imx6q_add_device_gpio_leds(void)
{
	platform_device_register(&imx6q_gpio_led_device);
}
#else
static void __init imx6q_add_device_gpio_leds(void) {}
#endif

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#define GPIO_BUTTON(gpio_num, ev_code, act_low, descr, wake, debounce)	\
{								\
	.gpio		= gpio_num,				\
	.type		= EV_KEY,				\
	.code		= ev_code,				\
	.active_low	= act_low,				\
	.desc		= "btn " descr,				\
	.wakeup		= wake,					\
	.debounce_interval = debounce,				\
}


static struct gpio_keys_button sparkauto_buttons[] = {
	GPIO_BUTTON(GPIO_KEY_VOLUP, KEY_VOLUMEUP, 1, "volume-up", 0, 1),
	GPIO_BUTTON(GPIO_KEY_VOLDOWN, KEY_VOLUMEDOWN, 1, "volume-down", 0, 1),
	GPIO_BUTTON(GPIO_KEY_RETURN, KEY_BACK, 1, "power-key", 0, 1),
};

static struct gpio_keys_platform_data sparkauto_button_data = {
	.buttons	= sparkauto_buttons,
	.nbuttons	= ARRAY_SIZE(sparkauto_buttons),
};

static struct platform_device sparkauto_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &sparkauto_button_data,
	}
	
};

static void __init imx6q_add_device_buttons(void)
{
	platform_device_register(&sparkauto_button_device);
}
#endif

static struct platform_pwm_backlight_data mx6_sabresd_pwm_backlight_data = {
	.pwm_id = 1,
	.max_brightness = 248,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

static struct mxc_dvfs_platform_data sabresd_dvfscore_data = {
	.reg_id = "VDDCORE",
	.soc_id	= "VDDSOC",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 80,
};

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	char *str;
	struct tag *t;
	int i = 0;
	struct ipuv3_fb_platform_data *pdata_fb = sparkauto_fb_data;

	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "fbmem=");
			if (str != NULL) {
				str += 6;
				pdata_fb[i++].res_size[0] = memparse(str, &str);
				while (*str == ',' &&
					i < ARRAY_SIZE(sparkauto_fb_data)) {
					str++;
					pdata_fb[i++].res_size[0] = memparse(str, &str);
				}
			}
			/* ION reserved memory */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "ionmem=");
			if (str != NULL) {
				str += 7;
				imx_ion_data.heaps[0].size = memparse(str, &str);
			}
			/* Primary framebuffer base address */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "fb0base=");
			if (str != NULL) {
				str += 8;
				pdata_fb[0].res_base[0] =
						simple_strtol(str, &str, 16);
			}
			/* GPU reserved memory */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpumem=");
			if (str != NULL) {
				str += 7;
				imx6q_gpu_pdata.reserved_mem_size = memparse(str, &str);
			}
			break;
		}
	}
}

static struct mipi_csi2_platform_data mipi_csi2_pdata = {
	.ipu_id	 = 0,
	.csi_id = 1,
	.v_channel = 0,
	.lanes = 2,
	.dphy_clk = "mipi_pllref_clk",
	.pixel_clk = "emi_clk",
};

static int __init caam_setup(char *__unused)
{
	caam_enabled = 1;
	return 1;
}
early_param("caam", caam_setup);

#define SNVS_LPCR 0x38
static void mx6_snvs_poweroff(void)
{

	void __iomem *mx6_snvs_base =  MX6_IO_ADDRESS(MX6Q_SNVS_BASE_ADDR);
	u32 value;
	value = readl(mx6_snvs_base + SNVS_LPCR);
	/*set TOP and DP_EN bit*/
	writel(value | 0x60, mx6_snvs_base + SNVS_LPCR);
}


#ifdef CONFIG_ANDROID_RAM_CONSOLE
static struct resource ram_console_resource = {
	.name = "android ram console",
	.flags = IORESOURCE_MEM,
};

static struct platform_device android_ram_console = {
	.name = "ram_console",
	.num_resources = 1,
	.resource = &ram_console_resource,
};

static int __init imx6x_add_ram_console(void)
{
	return platform_device_register(&android_ram_console);
}
#endif


#if  defined(CONFIG_BCMDHD)||defined(CONFIG_BCMDHD_MODULE)
static int bcm4330_wifi_power(int on){	
	gpio_request(WIFI_PWR, "wifi_pwr");
	gpio_direction_output(WIFI_PWR, on?0:1);
	gpio_free(WIFI_PWR);
	return 0;
}
static int bcm4330_wifi_reset(int on){	
	gpio_request(WIFI_RESET, "wifi_pwr");
	gpio_direction_output(WIFI_RESET, 0);
	msleep(10);
	gpio_direction_output(WIFI_RESET, 1);
	gpio_free(WIFI_RESET);
	return 0;
}
int bcm4330_carddetect(int val){
	return 0;
}

static struct resource brcm_wifi_resources[] = {
	[0] = {
		.name		= "bcmdhd_wlan_irq",
		.start		= gpio_to_irq(WIFI_HOST_WAKE),
		.end		= gpio_to_irq(WIFI_HOST_WAKE),
		.flags          = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL
	},
};

static struct wifi_platform_data brcm_wifi_control = {
	.set_power      = bcm4330_wifi_power,
	.set_reset      = bcm4330_wifi_reset,
	.set_carddetect = bcm4330_carddetect,
};

static struct platform_device brcm_wifi_device = {
        .name           = "bcmdhd_wlan",
        .id             = -1,
        .num_resources  = ARRAY_SIZE(brcm_wifi_resources),
        .resource       = brcm_wifi_resources,
        .dev            = {
                .platform_data = &brcm_wifi_control,
        },
};

int __init broadcom_wifi_init(void)
{
	return platform_device_register(&brcm_wifi_device);
}
late_initcall(broadcom_wifi_init);
#endif

/*!
 * Board specific initialization.
 */
static void __init mx6_sparkauto_board_init(void)
{
	int i;
	int ret;
	struct clk *clko2;
	int rate;
	struct clk *parent;

	if (cpu_is_mx6q())
		mxc_iomux_v3_setup_multiple_pads(mx6q_sparkauto_pads,
			ARRAY_SIZE(mx6q_sparkauto_pads));
	else if (cpu_is_mx6dl()) {
		mxc_iomux_v3_setup_multiple_pads(mx6dl_sparkauto_pads,
			ARRAY_SIZE(mx6dl_sparkauto_pads));
	}

	gp_reg_id = sabresd_dvfscore_data.reg_id;
	soc_reg_id = sabresd_dvfscore_data.soc_id;
	mx6q_sparkauto_init_uart();

	
	#ifdef CONFIG_ANDROID_RAM_CONSOLE
	imx6x_add_ram_console();
	#endif

	/*add bt support*/
	mxc_register_device(&mxc_bt_rfkill, &mxc_bt_rfkill_data);
	/*
	 * MX6DL/Solo only supports single IPU
	 * The following codes are used to change ipu id
	 * and display id information for MX6DL/Solo. Then
	 * register 1 IPU device and up to 2 displays for
	 * MX6DL/Solo
	 */
	if (cpu_is_mx6dl()) {
		ldb_data.ipu_id = 0;
		ldb_data.disp_id = 1;
		hdmi_core_data.ipu_id = 0;
		hdmi_core_data.disp_id = 0;
		mipi_dsi_pdata.ipu_id = 0;
		mipi_dsi_pdata.disp_id = 1;
		ldb_data.sec_ipu_id = 0;
	}
	imx6q_add_mxc_hdmi_core(&hdmi_core_data);

	imx6q_add_ipuv3(0, &ipu_data[0]);
	if (cpu_is_mx6q()) {
		imx6q_add_ipuv3(1, &ipu_data[1]);
		for (i = 0; i < 4 && i < ARRAY_SIZE(sparkauto_fb_data); i++)
			imx6q_add_ipuv3fb(i, &sparkauto_fb_data[i]);
	} else
		for (i = 0; i < 2 && i < ARRAY_SIZE(sparkauto_fb_data); i++)
			imx6q_add_ipuv3fb(i, &sparkauto_fb_data[i]);

	imx6q_add_vdoa();
	imx6q_add_mipi_dsi(&mipi_dsi_pdata);
	imx6q_add_lcdif(&lcdif_data);
	imx6q_add_ldb(&ldb_data);
	imx6q_add_v4l2_output(0);
	imx6q_add_v4l2_capture(0, &capture_data[0]);
	imx6q_add_v4l2_capture(1, &capture_data[1]);
	imx6q_add_mipi_csi2(&mipi_csi2_pdata);
	imx6q_add_imx_snvs_rtc();

	if (1 == caam_enabled)
		imx6q_add_imx_caam();

	imx6q_add_device_gpio_leds();

	imx6q_add_imx_i2c(0, &mx6q_sabresd_i2c_data);
	imx6q_add_imx_i2c(1, &mx6q_sabresd_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_sabresd_i2c_data);
	i2c_register_board_info(0, mxc_i2c0_board_info,
			ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));
	ret = gpio_request(SABRESD_PFUZE_INT, "pFUZE-int");
	if (ret) {
		printk(KERN_ERR"request pFUZE-int error!!\n");
		return;
	} else {
		gpio_direction_input(SABRESD_PFUZE_INT);
		mx6q_sabresd_init_pfuze100(SABRESD_PFUZE_INT);
	}
	/* SPI */
	imx6q_add_ecspi(0, &mx6q_sabresd_spi_data);
	spi_device_init();

	imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &mx6q_sabresd_anatop_thermal_data);
	
	/*enable ethernet after debugging with lan8720a okay*/
	/* Set RGMII_TX_CTL output for RMII reference clock */
	mxc_iomux_set_gpr_register(1, 21, 1, 1);	
	imx6_init_fec(fec_data);
	max6q_fec_reset();

	imx6q_add_pm_imx(0, &mx6q_sabresd_pm_data);

	/* Move sd4 to first because sd4 connect to emmc.
	   Mfgtools want emmc is mmcblk0 and other sd card is mmcblk1.
	   mmc0 is emmc
	   mmc1 is sd card
	   mmc2 is wifi
	*/
	imx6q_add_sdhci_usdhc_imx(3, &mx6q_sd4_data);
	//imx6q_add_sdhci_usdhc_imx(0, &mx6q_sd1_data);
	imx6q_add_sdhci_usdhc_imx(2, &mx6q_sd3_data);
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	imx6q_sabresd_init_usb();
	/* SATA is not supported by MX6DL/Solo */
	if (cpu_is_mx6q()) {
#ifdef CONFIG_SATA_AHCI_PLATFORM
		imx6q_add_ahci(0, &mx6q_sabresd_sata_data);
#else
		mx6q_sabresd_sata_init(NULL,
			(void __iomem *)ioremap(MX6Q_SATA_BASE_ADDR, SZ_4K));
#endif
	}
	imx6q_add_vpu();
	imx6q_init_audio();
	platform_device_register(&sabresd_vmmc_reg_devices);
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm(1);
	imx6q_add_mxc_pwm(2);
	imx6q_add_mxc_pwm(3);
	imx6q_add_mxc_pwm_backlight(0, &mx6_sabresd_pwm_backlight_data);

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();

	imx6q_add_dvfs_core(&sabresd_dvfscore_data);

	if (imx_ion_data.heaps[0].size)
		imx6q_add_ion(0, &imx_ion_data,
			sizeof(imx_ion_data) + sizeof(struct ion_platform_heap));

	
	#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
	imx6q_add_device_buttons();
	#endif

	/* enable sensor 3v3 and 1v8 */
	gpio_request(SABRESD_SENSOR_EN, "sensor-en");
	gpio_direction_output(SABRESD_SENSOR_EN, 1);

	/* enable ecompass intr */
	gpio_request(SABRESD_eCOMPASS_INT, "ecompass-int");
	gpio_direction_input(SABRESD_eCOMPASS_INT);
	/* enable light sensor intr */
	gpio_request(SABRESD_ALS_INT, "als-int");
	gpio_direction_input(SABRESD_ALS_INT);

	gpio_request(BL_ON, "bl_on");
	gpio_direction_output(BL_ON, 1);
	gpio_request(LCD_POWER, "lcd_power");
	gpio_direction_output(LCD_POWER, 1);
	gpio_request(MX6_VDD_3V3_EN, "vdd_3v3_en");
	gpio_direction_output(MX6_VDD_3V3_EN, 1);
	gpio_request(MX6_VDD_5V_EN, "vdd_5v_en");
	gpio_direction_output(MX6_VDD_5V_EN, 1);


	
	#if  defined(CONFIG_BCMDHD)||defined(CONFIG_BCMDHD_MODULE)
	//turn on wifi power
	bcm4330_wifi_power(1);
	#endif
	
	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();

	if (cpu_is_mx6dl()) {
		imx6dl_add_imx_pxp();
		imx6dl_add_imx_pxp_client();
	}

	/* Enable Aux_5V */
	gpio_request(SABRESD_AUX_5V_EN, "aux_5v_en");
	gpio_direction_output(SABRESD_AUX_5V_EN, 1);
	gpio_set_value(SABRESD_AUX_5V_EN, 1);

	/* Register charger chips */
	platform_device_register(&sabresd_max8903_charger_1);
	pm_power_off = mx6_snvs_poweroff;
	imx6q_add_busfreq();

	imx6_add_armpmu();
	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);

		

	clko2 = clk_get(NULL, "clko2_clk");
	if (IS_ERR(clko2))
		pr_err("can't get CLKO2 clock.\n");

	parent = clk_get(NULL, "osc_clk");
	if (!IS_ERR(parent)) {
		clk_set_parent(clko2, parent);
		clk_put(parent);
	}
	rate = clk_round_rate(clko2, 12000000);
	clk_set_rate(clko2, rate);

	rt5633_data.sysclk = rate;
	clk_enable(clko2);
}

extern void __iomem *twd_base;
static void __init mx6_sparkauto_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART1_BASE_ADDR, uart_clk);
}

static struct sys_timer mx6_sparkauto_timer = {
	.init   = mx6_sparkauto_timer_init,
};

static void __init mx6q_sparkauto_reserve(void)
{
	phys_addr_t phys;
	int i, fb0_reserved = 0, fb_array_size;

	/*
	 * Reserve primary framebuffer memory if its base address
	 * is set by kernel command line.
	 */
	fb_array_size = ARRAY_SIZE(sparkauto_fb_data);
	if (fb_array_size > 0 && sparkauto_fb_data[0].res_base[0] &&
	    sparkauto_fb_data[0].res_size[0]) {
		memblock_reserve(sparkauto_fb_data[0].res_base[0],
				 sparkauto_fb_data[0].res_size[0]);
		memblock_remove(sparkauto_fb_data[0].res_base[0],
				sparkauto_fb_data[0].res_size[0]);
		sparkauto_fb_data[0].late_init = true;
		ipu_data[ldb_data.ipu_id].bypass_reset = true;
		fb0_reserved = 1;
	}
	for (i = fb0_reserved; i < fb_array_size; i++)
		if (sparkauto_fb_data[i].res_size[0]) {
			/* Reserve for other background buffer. */
			phys = memblock_alloc(sparkauto_fb_data[i].res_size[0],
						SZ_4K);
			memblock_remove(phys, sparkauto_fb_data[i].res_size[0]);
			sparkauto_fb_data[i].res_base[0] = phys;
		}

#ifdef CONFIG_ANDROID_RAM_CONSOLE
	phys = memblock_alloc_base(SZ_128K, SZ_4K, SZ_1G);
	memblock_remove(phys, SZ_128K);
	memblock_free(phys, SZ_128K);
	ram_console_resource.start = phys;
	ram_console_resource.end   = phys + SZ_128K - 1;
#endif

#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
					   SZ_4K, SZ_1G);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
#endif

#if defined(CONFIG_ION)
	if (imx_ion_data.heaps[0].size) {
		phys = memblock_alloc(imx_ion_data.heaps[0].size, SZ_4K);
		memblock_remove(phys, imx_ion_data.heaps[0].size);
		imx_ion_data.heaps[0].base = phys;
	}
#endif
}

/*
 * initialize __mach_desc_MX6Q_SABRESD data structure.
 */
MACHINE_START(MX6Q_SPARKAUTO, "Freescale i.MX 6Quad/DualLite/Solo SparkAuto Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = mx6_sparkauto_board_init,
	.timer = &mx6_sparkauto_timer,
	.reserve = mx6q_sparkauto_reserve,
MACHINE_END
