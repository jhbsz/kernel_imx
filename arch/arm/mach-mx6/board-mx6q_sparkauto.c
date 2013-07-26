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


#include <mach/imx_rfkill.h>

#include <linux/i2c/at24.h>
#include <linux/i2c/tsc2007.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-mx6q_sparkauto.h"
#include "board-mx6dl_sparkauto.h"
#include "generic_devices.h"

#define SABRESD_BT_RESET	IMX_GPIO_NR(1, 2)


#define SABRESD_DISP_RST_B	IMX_GPIO_NR(6, 11)
#define SABRESD_DISP_PWR_EN	IMX_GPIO_NR(6, 14)


/*
\*Below IO is configured for programing facility ,properly iomux should be done in header file xxxx_sparkauto.h
*/
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

#define SPARKAUTO_USB_HUB_PWR	IMX_GPIO_NR(1, 14)
#define SPARKAUTO_USB_HUB_RESET	IMX_GPIO_NR(7, 13)

#define SPARKAUTO_CSI0_PWN	IMX_GPIO_NR(7, 12)
#define SPARKAUTO_CSI0_RST	IMX_GPIO_NR(4, 15)

#define SPARKAUTO_CODEC_PWR_EN	IMX_GPIO_NR(3, 22)


#define SPARKAUTO_TVIN_RST		IMX_GPIO_NR(5, 0)
#define SPARKAUTO_TVIN_PWR_EN	IMX_GPIO_NR(2, 28)
#define SPARKAUTO_TVIN_INT		IMX_GPIO_NR(3, 28)

#define TSC2007_IRQGPIO			IMX_GPIO_NR(4, 5)

#define SPARKAUTO_ECSPI1_CS0 	IMX_GPIO_NR(3, 19)

#define W1_EMULATED_IO		SPARKAUTO_SD1_WP
#define MODEM_PWR_EN		IMX_GPIO_NR(3, 13)
#define MODEM_RST			IMX_GPIO_NR(3, 11)
#define MODEM_WAKEUP		IMX_GPIO_NR(2, 24)
extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;



/*Micro SD*/
static const struct esdhc_platform_data mx6q_sd1_data __initconst = {	
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.cd_type = ESDHC_CD_PERMANENT,
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


static int mx6q_sparkauto_spi_cs[] = {
	SPARKAUTO_ECSPI1_CS0,
};

static const struct spi_imx_master mx6q_sparkauto_spi_data __initconst = {
	.chipselect     = mx6q_sparkauto_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6q_sparkauto_spi_cs),
};

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct mtd_partition imx6_sparkauto_spi_nor_partitions[] = {
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

static struct flash_platform_data imx6_sparkauto__spi_flash_data = {
	.name = "m25p80",
	.parts = imx6_sparkauto_spi_nor_partitions,
	.nr_parts = ARRAY_SIZE(imx6_sparkauto_spi_nor_partitions),
	.type = "sst25vf032b",
};
#endif

static struct spi_board_info imx6_sparkauto_spi_nor_device[] __initdata = {
#if defined(CONFIG_MTD_M25P80)
	{
		.modalias = "m25p80",
		.max_speed_hz = 20000000, /* max spi clock (SCK) speed in HZ */
		.bus_num = 0,
		.chip_select = 0,
		.platform_data = &imx6_sparkauto__spi_flash_data,
	},
#endif
};

static void spi_device_init(void)
{
	spi_register_board_info(imx6_sparkauto_spi_nor_device,
				ARRAY_SIZE(imx6_sparkauto_spi_nor_device));
}

static struct imx_ssi_platform_data mx6_sparkauto_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct platform_device mx6_sparkauto_audio_rt5633_device = {
	.name = "imx-rt5633",
};

static struct clk *clko2;
static struct mxc_audio_platform_data rt5633_data;

static int rt5633_clk_enable(int enable)
{
	if (enable)
		clk_enable(clko2);
	else
		clk_disable(clko2);

	return 0;
}

static int mxc_rt5633_init(void)
{
	int rate;
	struct clk *parent;

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

	//enable clko2 since somtimes codec require it for initialization
	clk_enable(clko2);

	return 0;
}

static struct mxc_audio_platform_data rt5633_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.init = mxc_rt5633_init,
	.clock_enable = rt5633_clk_enable,

};



static void mx6q_csi0_cam_powerdown(int powerdown)
{
	if (powerdown)
		gpio_set_value(SPARKAUTO_CSI0_PWN, 1);
	else
		gpio_set_value(SPARKAUTO_CSI0_PWN, 0);

	msleep(2);
}

static void mx6q_csi0_io_init(void)
{
	if (cpu_is_mx6q())
		mxc_iomux_v3_setup_multiple_pads(mx6q_sparkauto_csi0_sensor_pads,
			ARRAY_SIZE(mx6q_sparkauto_csi0_sensor_pads));
	else if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_sabresd_csi0_sensor_pads,
			ARRAY_SIZE(mx6dl_sabresd_csi0_sensor_pads));

	/* Camera reset */
	gpio_request(SPARKAUTO_CSI0_RST, "cam-reset");
	gpio_direction_output(SPARKAUTO_CSI0_RST, 1);

	/* Camera power down */
	gpio_request(SPARKAUTO_CSI0_PWN, "cam-pwdn");
	gpio_direction_output(SPARKAUTO_CSI0_PWN, 1);
	msleep(5);
	gpio_set_value(SPARKAUTO_CSI0_PWN, 0);
	msleep(5);
	gpio_set_value(SPARKAUTO_CSI0_RST, 0);
	msleep(1);
	gpio_set_value(SPARKAUTO_CSI0_RST, 1);
	msleep(5);
	gpio_set_value(SPARKAUTO_CSI0_PWN, 1);

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

static void mx6q_csi1_io_init(void)
{
	iomux_v3_cfg_t *tvin_pads = NULL;
	u32 tvin_pads_cnt;

	if (cpu_is_mx6q()) {
		tvin_pads = mx6q_sparkauto_csi1_sensor_pads;
		tvin_pads_cnt = ARRAY_SIZE(mx6q_sparkauto_csi1_sensor_pads);
	}else {
		BUG();
	}

	BUG_ON(!tvin_pads);
	mxc_iomux_v3_setup_multiple_pads(tvin_pads, tvin_pads_cnt);
	/* Tvin reset */
	gpio_request(SPARKAUTO_TVIN_RST, "tvin-reset");
	gpio_direction_output(SPARKAUTO_TVIN_RST, 1);

	/* Tvin power down */
	gpio_request(SPARKAUTO_TVIN_PWR_EN, "cam-pwdn");
	gpio_direction_output(SPARKAUTO_TVIN_PWR_EN, 0);
	msleep(1);
	gpio_set_value(SPARKAUTO_TVIN_PWR_EN, 1);

	if (cpu_is_mx6q())
		mxc_iomux_set_gpr_register(1, 20, 1, 1);
	else if (cpu_is_mx6dl())
		mxc_iomux_set_gpr_register(13, 0, 3, 4);

}


static struct fsl_mxc_tvin_platform_data tvin_data = {
	.io_init = mx6q_csi1_io_init,
	.cvbs = true,
};


static struct at24_platform_data eeprom_data = {
	.byte_len	= SZ_32K / 8,	
	.page_size	= 32,	
	.flags		= AT24_FLAG_ADDR16|AT24_FLAG_IRUGO,
};


static struct tsc2007_platform_data tsc2007_info = {
	.model			= 2007,
	.x_plate_ohms	= 180,
};

static struct imxi2c_platform_data mx6q_sparkauto_i2c_data = {
	.bitrate = 100000,
};

/*
 * There are two version of IO board v1 and v2.
 * For v1 ,eeprom is connected on i2c bus 0 (on my hand)
 * For v2 ,eeprom is connected on i2c bus 1
*/
static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("adv7180", 0x21),
		.platform_data = (void *)&tvin_data,
	},{
		I2C_BOARD_INFO("at24", 0x54),
		.platform_data	= &eeprom_data,	
	},
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("ov5640", 0x3c),
		.platform_data = (void *)&camera_data,
	},{
		I2C_BOARD_INFO("rt5633", 0x1c),
	},{
		I2C_BOARD_INFO("mxc_ldb_i2c", 0x50),
	},	
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	/*
	{
		I2C_BOARD_INFO("tsc2007", 0x48),
		.platform_data	= &tsc2007_info,
		.irq		= gpio_to_irq(TSC2007_IRQGPIO),
	}
	*/
};


static void imx6q_sparkauto_usbotg_vbus(bool on)
{
	if(on){}
}

static void imx6q_sparkauto_host1_vbus(bool on)
{
	if (on)
		gpio_set_value(SPARKAUTO_USB_HUB_PWR, 1);
	else
		gpio_set_value(SPARKAUTO_USB_HUB_PWR, 0);
}

static void __init imx6q_sparkauto_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	/* keep USB host1 VBUS always on */
	ret = gpio_request(SPARKAUTO_USB_HUB_PWR, "usb-hub-pwr");
	if (ret) {
		pr_err("failed to get GPIO SPARKAUTO_USB_HUB_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(SPARKAUTO_USB_HUB_PWR, 1);
	
	ret = gpio_request(SPARKAUTO_USB_HUB_RESET, "usb-hub-reset");
	if (ret) {
		pr_err("failed to get GPIO SPARKAUTO_USB_HUB_RESET: %d\n",
			ret);
		return;
	}
	gpio_direction_output(SPARKAUTO_USB_HUB_RESET, 0);
	mdelay(5);
	gpio_direction_output(SPARKAUTO_USB_HUB_RESET, 1);
	
	if (board_is_mx6_reva())
		mxc_iomux_set_gpr_register(1, 13, 1, 1);
	else
		mxc_iomux_set_gpr_register(1, 13, 1, 1);

	mx6_set_otghost_vbus_func(imx6q_sparkauto_usbotg_vbus);
	mx6_set_host1_vbus_func(imx6q_sparkauto_host1_vbus);

}




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
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	},
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
}

static void sabresd_suspend_exit(void)
{
	/* resume restore */
}
static const struct pm_platform_data mx6q_sabresd_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = sabresd_suspend_enter,
	.suspend_exit = sabresd_suspend_exit,
};

static struct regulator_consumer_supply sparkauto_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.0"),	
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data sparkauto_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(sparkauto_vmmc_consumers),
	.consumer_supplies = sparkauto_vmmc_consumers,
};

static struct fixed_voltage_config sparkauto_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &sparkauto_vmmc_init,
};

static struct platform_device sparkauto_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &sparkauto_vmmc_reg_config,
	},
};

static int __init imx6q_init_audio(void)
{
	mxc_register_device(&mx6_sparkauto_audio_rt5633_device,
				    &rt5633_data);
	imx6q_add_imx_ssi(1, &mx6_sparkauto_ssi_pdata);

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

static struct gpio_led imx6q_gpio_leds[] = {

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

#warning FIXME:add modem wakeup support
int __init generic_modem_init(void){
	gpio_request(MODEM_PWR_EN, "modem_pwr");
	gpio_direction_output(MODEM_PWR_EN, 0);
	gpio_free(MODEM_PWR_EN);
	
	gpio_request(MODEM_RST, "modem_rst");
	gpio_direction_output(MODEM_RST, 0);
	msleep(10);
	gpio_direction_output(MODEM_RST, 1);	
	gpio_free(MODEM_RST);
	
	return 0;
	
}


/*!
 * Board specific initialization.
 */
static void __init mx6_sparkauto_board_init(void)
{
	int i;

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
		mipi_dsi_pdata.ipu_id = 0;
		mipi_dsi_pdata.disp_id = 1;
		ldb_data.sec_ipu_id = 0;
	}

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

	imx6q_add_imx_caam();

	imx6q_add_device_gpio_leds();

	imx6q_add_imx_i2c(0, &mx6q_sparkauto_i2c_data);
	imx6q_add_imx_i2c(1, &mx6q_sparkauto_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_sparkauto_i2c_data);
	i2c_register_board_info(0, mxc_i2c0_board_info,
			ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));

	/* SPI */
	imx6q_add_ecspi(0, &mx6q_sparkauto_spi_data);
	spi_device_init();


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
	imx6q_add_sdhci_usdhc_imx(0, &mx6q_sd1_data);
	imx6q_add_sdhci_usdhc_imx(2, &mx6q_sd3_data);
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	imx6q_sparkauto_init_usb();

	imx6q_add_vpu();
	imx6q_init_audio();
	platform_device_register(&sparkauto_vmmc_reg_devices);
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
	
	if (cpu_is_mx6dl()) {
		imx6dl_add_imx_pxp();
		imx6dl_add_imx_pxp_client();
	}

	/* Register charger chips */
	pm_power_off = mx6_snvs_poweroff;
	imx6q_add_busfreq();

	imx6_add_armpmu();
	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);


	generic_modem_init();

	//Uncomment following to enable w1 bus emulation on SD1_WP io(SD1 not used pin)
	generic_add_w1(W1_EMULATED_IO);

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
