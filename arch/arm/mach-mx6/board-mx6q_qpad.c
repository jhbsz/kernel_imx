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
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/max17135.h>
#include <linux/mfd/wm8994/pdata.h>
#include <linux/mfd/wm8994/gpio.h>
#include <sound/wm8962.h>
#include <linux/mfd/mxc-hdmi-core.h>
#include <linux/power/qpower.h>
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
#include <mach/system.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-mx6q_qpad.h"
#include "board-mx6dl_qpad.h"
#include <mach/imx_rfkill.h>


#include "generic_devices.h"
#include <linux/pwm.h>


#define QPAD_PFUZE_INT		IMX_GPIO_NR(7, 13)


#define QPAD_CHARGE_DOK_B	IMX_GPIO_NR(2, 24)
#define QPAD_CHARGE_UOK_B	IMX_GPIO_NR(3, 17)
#define QPAD_CHARGE_CHG_1_B	IMX_GPIO_NR(3, 23)
#define QPAD_CHARGE_FLT_1_B	IMX_GPIO_NR(5, 2)
#define QPAD_BATTERY_ALERT	IMX_GPIO_NR(4, 14)
#define QPAD_BATTERY_DET	IMX_GPIO_NR(4, 11)

#define QPAD_CAM_PWR		IMX_GPIO_NR(1, 16)
#define QPAD_CAM_RST		IMX_GPIO_NR(5, 20)


#define QPAD_W1_IO			IMX_GPIO_NR(3, 28)


#define GPIO_KEY_POWER		IMX_GPIO_NR(3,29)
#define GPIO_KEY_MENU		IMX_GPIO_NR(3,9)
#define GPIO_KEY_HOME		IMX_GPIO_NR(3,10)
#define GPIO_KEY_BACK		IMX_GPIO_NR(3,11)
#define GPIO_KEY_F1			IMX_GPIO_NR(6,9)
#define GPIO_KEY_F2			IMX_GPIO_NR(6,10)

#define QPAD_USB_OTG_PWR	IMX_GPIO_NR(3, 22)

#define QPAD_DISP_PWR_EN		IMX_GPIO_NR(2, 6)
#define QPAD_DISP_BL_PWR_EN	IMX_GPIO_NR(4, 15)
#define QPAD_DISP_RST_B		IMX_GPIO_NR(6, 16)

#define QPAD_CSI0_PWN		IMX_GPIO_NR(1, 16)
#define QPAD_CSI0_RST		IMX_GPIO_NR(5, 20)

#define QPAD_MODEM_ONOFF		IMX_GPIO_NR(2, 2)
#define QPAD_MODEM_RST			IMX_GPIO_NR(2, 3)
#define QPAD_MODEM_PWR			IMX_GPIO_NR(2, 4)
#define QPAD_MODEM_WAKEMODEM	IMX_GPIO_NR(2, 5)
#define QPAD_MODEM_WAKEAP		IMX_GPIO_NR(1, 18)

#define QPAD_AUDIO_RST					IMX_GPIO_NR(7, 11)
#define QPAD_AUDIO_INT					IMX_GPIO_NR(1, 20)
#define QPAD_AUDIO_HEADPHONE_DET		IMX_GPIO_NR(3, 21)

//QR Engine
#define QPAD_QRE_RST		IMX_GPIO_NR(6, 31)
#define QPAD_QRE_TRIG		IMX_GPIO_NR(3, 30)

//FLASHLIGHT
#define QPAD_FL_PWR_EN		IMX_GPIO_NR(3, 31)
#define QPAD_FL_EN			IMX_GPIO_NR(4, 5)

//Touch Panel
#define QPAD_TP_PWR_EN		IMX_GPIO_NR(2, 28)
#define QPAD_TP_RST			IMX_GPIO_NR(6, 8)
#define QPAD_TP_IRQ			IMX_GPIO_NR(6, 7)

//Sensor
#define QPAD_SENSOR_RST		IMX_GPIO_NR(2, 23)
#define QPAD_SENSOR_INT1	IMX_GPIO_NR(3, 16)
#define QPAD_SENSOR_INT2	IMX_GPIO_NR(3, 15)

//WiFi
#define QPAD_WIFI_RST		IMX_GPIO_NR(7, 8)


//SDHC
#define QPAD_SD2_CD			IMX_GPIO_NR(2, 0)



#define NFC_VEN			IMX_GPIO_NR(1,17)
#define NFC_IRQ			IMX_GPIO_NR(6,17)
#define NFC_UPE			IMX_GPIO_NR(1,19)

static iomux_v3_cfg_t nfc_pads[] = {
	NEW_PAD_CTRL(MX6Q_PAD_SD1_DAT1__GPIO_1_17,MX6Q_GENERIC_PAD_CTRL),/*ven*/
	MX6Q_PAD_SD3_DAT7__GPIO_6_17,/*int*/
	NEW_PAD_CTRL(MX6Q_PAD_SD1_DAT2__GPIO_1_19,MX6Q_GENERIC_PAD_CTRL),/*upgrade*/
};
static int __init nfc_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(nfc_pads,
		ARRAY_SIZE(nfc_pads));
	return generic_add_device_pn544(0,NFC_IRQ,NFC_VEN,NFC_UPE);
}

enum {
 eBootModeNormal=0,
 eBootModeRecovery,
 eBootModeCharger,
 eBootModeFastboot,
 eBootModeAutoupdate,
 eBootModeFactory,
 eBootModeMax,
};

static int android_bootmode;
static int __init  detect_bootmode(char *arg)
{
	//if(arg&&!strcmp(arg,"charger"))
	//	android_bootmode = eBootModeCharger;

    return 0;
}

__setup("androidboot.mode=", detect_bootmode);


//
//
//


extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;


/*Micro SD*/
static const struct esdhc_platform_data qpad_sd2_data __initconst = {
	.cd_gpio = QPAD_SD2_CD,
	.keep_power_at_suspend = 1,
	.cd_type = ESDHC_CD_CONTROLLER,
	.runtime_pm = 1,
};

/*WIFI SDIO*/
static const struct esdhc_platform_data qpad_sd3_data __initconst = {
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.cd_type = ESDHC_CD_PERMANENT,
};

/*eMMC*/
static const struct esdhc_platform_data qpad_sd4_data __initconst = {
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.support_8bit = 1,
	.delay_line = 0,
	.cd_type = ESDHC_CD_PERMANENT,
};

static const struct anatop_thermal_platform_data
	qpad_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

static struct pwm_device       *pwm_uart_mod=NULL; ;
static int pwm_uart_mod_enable=1;
module_param_named(uart_modulation, pwm_uart_mod_enable, int, S_IRUGO | S_IWUSR | S_IWGRP);
void uart_modulation_enable(int en){
       if(!pwm_uart_mod){
               pwm_uart_mod = pwm_request(1, "IRC");
       }
       if(pwm_uart_mod&&pwm_uart_mod_enable){
               if(en){
                       pwm_config(pwm_uart_mod, 13158, 26315/*38kHz*/);
                       pwm_enable(pwm_uart_mod);  
               }else{
                       pwm_config(pwm_uart_mod, 0, 26315);
                       pwm_disable(pwm_uart_mod);              
               }
               
       }
       
}

/*
 * IMX-78,For Infrared modulation communication,we need using half duplex,Never enable DMA mode since 
 * IMX serial driver does not support half duplex mode in DMA mode.
 */
static const struct imxuart_platform_data mx6q_uart4_data __initconst = {
       .flags      = IMXUART_MODULATION,
       .modulation_enable = uart_modulation_enable,

};

static inline void mx6q_qpad_init_uart(void)
{
	imx6q_add_imx_uart(0, NULL);	
	imx6q_add_imx_uart(1, NULL);
	imx6q_add_imx_uart(2, NULL);
	imx6q_add_imx_uart(3, &mx6q_uart4_data);
	imx6q_add_imx_uart(4, NULL);	
}

static void mx6q_csi0_cam_powerdown(int powerdown)
{
	if (powerdown)
		gpio_set_value(QPAD_CSI0_PWN, 1);
	else
		gpio_set_value(QPAD_CSI0_PWN, 0);
}

static void mx6q_csi0_io_init(void)
{
	if (cpu_is_mx6q())
		mxc_iomux_v3_setup_multiple_pads(mx6q_qpad_csi0_sensor_pads,
			ARRAY_SIZE(mx6q_qpad_csi0_sensor_pads));
	else if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_qpad_csi0_sensor_pads,
			ARRAY_SIZE(mx6dl_qpad_csi0_sensor_pads));

	/* Camera reset */
	gpio_request(QPAD_CSI0_RST, "cam-reset");
	gpio_direction_output(QPAD_CSI0_RST, 1);

	/* Camera power down */
	gpio_request(QPAD_CSI0_PWN, "cam-pwdn");
	gpio_direction_output(QPAD_CSI0_PWN, 1);
	msleep(5);
	gpio_set_value(QPAD_CSI0_PWN, 0);
	msleep(5);
	gpio_set_value(QPAD_CSI0_RST, 0);
	msleep(1);
	gpio_set_value(QPAD_CSI0_RST, 1);
	msleep(5);
	gpio_set_value(QPAD_CSI0_PWN, 1);
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
	.mclk = 24000000,
	.mclk_source = 0,
	.csi = 0,
	.io_init = mx6q_csi0_io_init,
	.pwdn = mx6q_csi0_cam_powerdown,
};

#include <linux/i2c/eup2471.h>


static int eup2471_enable(int on)
{
	int en = QPAD_FL_PWR_EN;

	if (gpio_request(en, "eup2471 enable")) {
		printk(KERN_INFO "gpio %d request failed\n", en);
		return -EINVAL;
	}

	printk("eup2471 %s\n", on?"enable":"disable");
	if (on)
	{
		gpio_direction_output(en, 1);
		udelay(10);
	}
	else
	{
		gpio_direction_output(en, 0);
	}

	gpio_free(en);

	return 0;
}

static int eup2471_flash(int on)
{
	int en = QPAD_FL_EN;

	if (gpio_request(en, "eup2471 flash on")) {
		printk(KERN_INFO "gpio %d request failed\n", en);
		return -EINVAL;
	}

	printk("eup2471 flash %s\n", on?"on":"off");
	if (on)
	{
		gpio_direction_output(en, 0);
		udelay(10);

		gpio_direction_output(en, 1);
		udelay(10);
	}
	else
	{
		gpio_direction_output(en, 0);
	}

	gpio_free(en);

	return 0;
}

static struct eup2471_platform_data eup2471_pdata = 
{
	.enable 	= eup2471_enable,
	.flash 		= eup2471_flash,
};

#include <linux/ft5x0x_ts.h>

static int ft5x0x_plat_init(void){
	int ret=0;	
	ret = gpio_request(QPAD_TP_PWR_EN, "tp-pwr");
	if (ret) {
		pr_err("failed to get GPIO tp-pwr: %d\n",
			ret);
		goto err;
	}
	gpio_direction_output(QPAD_TP_PWR_EN, 1);
	gpio_free(QPAD_TP_PWR_EN);

	ret = gpio_request(QPAD_TP_RST, "tp-rst");
	if (ret) {
		pr_err("failed to get GPIO tp-rst: %d\n",
			ret);
		goto err;
	}
	gpio_direction_output(QPAD_TP_RST, 0);
	msleep(50);
	gpio_direction_output(QPAD_TP_RST, 1);
	gpio_free(QPAD_TP_RST);
	msleep(50);

err:
	return ret;
	
}
static struct ft5x0x_ts_platform_data ft5x0x_data=
{
	.plat_init	= ft5x0x_plat_init,
};



static struct imxi2c_platform_data mx6q_i2c_data = {
	.bitrate = 100000,
};


static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("rt5625", 0x1f),
	},
	{
		I2C_BOARD_INFO("ov564x", 0x3c),
		.platform_data = (void *)&camera_data,
	},
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		.type			= "ft5x0x_ts",
		.addr			= 0x38,
		.irq			= gpio_to_irq(QPAD_TP_IRQ),
		.platform_data	= &ft5x0x_data,
	},
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
		.type	= "eup2471",
		.addr	= 0x37,
		.platform_data = &eup2471_pdata,
	},	
	{
		.type	= "fxos8700",
		.addr	= 0x1E,
	},
};


static void qpad_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(QPAD_USB_OTG_PWR, 1);
	else
		gpio_set_value(QPAD_USB_OTG_PWR, 0);

}

static void qpad_host1_vbus(bool on)
{
	//dummy function since we don't have any usb hub
}

static void __init imx6q_qpad_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	/* disable external charger detect,
	 * or it will affect signal quality at dp .
	 */
	ret = gpio_request(QPAD_USB_OTG_PWR, "otg-vbus");
	if (ret) {
		pr_err("failed to get GPIO otg-vbus: %d\n",
			ret);
		return;
	}
	gpio_direction_output(QPAD_USB_OTG_PWR, 0);
	/*
	 *
	 *OTG ID daisy chain set
	 * 0: ENET_RX_ER as USB_OTG_ID 
	 * 1: GPIO_1 as USB_OTG_ID
	 */
	mxc_iomux_set_gpr_register(1, 13, 1, 1);
	mx6_set_otghost_vbus_func(qpad_usbotg_vbus);
	mx6_set_host1_vbus_func(qpad_host1_vbus);

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
	#if 0
	int ret;
	//return if bootloader already enable LCD???	
	ret = gpio_request(QPAD_DISP_RST_B, "disp_reset");	
	if (ret) {
		pr_err("failed to request gpio: %d\n",QPAD_DISP_RST_B);
		return;
	}
	gpio_direction_output(QPAD_DISP_RST_B,1);
	udelay(10);
	gpio_direction_output(QPAD_DISP_RST_B, 0);
	udelay(50);
	gpio_direction_output(QPAD_DISP_RST_B, 1);
	
	gpio_free(QPAD_DISP_RST_B);

	/*
	 * it needs to delay 120ms minimum for reset complete
	 */
	msleep(120);
	#endif
}

static void mipi_dsi_power(int en){	
	/*
	int ret;
	ret = gpio_request(QPAD_DISP_PWR_EN, "disp_pwren");	
	if (ret) {
		pr_err("failed to request gpio: %d\n",QPAD_DISP_PWR_EN);
		return;
	}
	gpio_direction_output(QPAD_DISP_PWR_EN,en?0:1);
	gpio_free(QPAD_DISP_PWR_EN);
	*/
}
static void mipi_dsi_baclight_power(int en){
	/*
	int ret;
	ret = gpio_request(QPAD_DISP_BL_PWR_EN, "disp_bl_pwren");	
	if (ret) {
		pr_err("failed to request gpio: %d\n",QPAD_DISP_BL_PWR_EN);
		return;
	}
	gpio_direction_output(QPAD_DISP_BL_PWR_EN,en?1:0);
	gpio_free(QPAD_DISP_BL_PWR_EN);
	*/
}
static struct mipi_dsi_platform_data mipi_dsi_pdata = {
	.ipu_id		= 0,
	.disp_id	= 1,
	.lcd_panel	= "NT-QHD",
	.reset		= mx6_reset_mipi_dsi,
	.lcd_power	= mipi_dsi_power,
	.backlight_power	= mipi_dsi_baclight_power,

};

static struct ipuv3_fb_platform_data qpad_fb_data[] = {
	{ /*fb0*/
	.disp_dev = "mipi_dsi",
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
	.mode_str = "NT-QHD",
	.default_bpp = 32,
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

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_RGB565,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 0,
	.disp_id = 1,
	.ext_ref = 1,
	.mode = LDB_SEP1,
	.sec_ipu_id = 0,
	.sec_disp_id = 0,
};



static struct qpower_battery_pdata qbp = {
	.alert = QPAD_BATTERY_ALERT,
	.busid = 0,
};
static struct qpower_charger_pdata qcp = {
	.dok = QPAD_CHARGE_DOK_B,
	.uok = QPAD_CHARGE_UOK_B,
	.chg = QPAD_CHARGE_CHG_1_B,
	.flt = QPAD_CHARGE_FLT_1_B,	
	.dcm_always_high = true,
	.dc_valid = true,
	.usb_valid = false,//true,
	.feature_flag = QPOWER_CHARGER_FEATURE_SHORT_MODE,
	//for battery detection
	.det = QPAD_BATTERY_DET,
	.bat_det_active = 0,
	
};
static struct qpower_pdata qp = {
	.bp = &qbp,
	.cp = &qcp,
	.flags = QPOWER_FEATURE_CHARGER|QPOWER_FEATURE_BATTERY,  
};

static int __init qpad_power_init(void){
	
	imx_add_platform_device("qpower", -1,
			NULL, 0, &qp, sizeof(qp));
	return 0;
}


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


static void qpad_suspend_enter(void)
{
}

static void qpad_suspend_exit(void)
{
}
static const struct pm_platform_data qpad_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = qpad_suspend_enter,
	.suspend_exit = qpad_suspend_exit,
};

static struct regulator_consumer_supply qpad_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.0"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),	
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data qpad_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(qpad_vmmc_consumers),
	.consumer_supplies = qpad_vmmc_consumers,
};

static struct fixed_voltage_config qpad_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &qpad_vmmc_init,
};

static struct platform_device qpad_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &qpad_vmmc_reg_config,
	},
};



static struct imx_ssi_platform_data mx6_qpad_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct platform_device mx6_qpad_audio_rt5625_device = {
	.name = "imx-rt5625",
};

static struct mxc_audio_platform_data rt5625_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.hp_gpio = QPAD_AUDIO_HEADPHONE_DET,
	.hp_active_low = 0,
};

static int __init imx6q_init_audio(void)
{
	int ret,rate;	
	struct clk *parent,*clko2;
	

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

	rt5625_data.sysclk = rate;

	//enable clko2 since somtimes codec require it for initialization
	clk_enable(clko2);

	//reset audio codec
	ret = gpio_request(QPAD_AUDIO_RST, "audio-rst");
	if (ret) {
		pr_err("failed to get GPIO audio-rst: %d\n",
			ret);
		return -EINVAL;
	}
	gpio_direction_output(QPAD_AUDIO_RST, 0);
	mdelay(5);
	gpio_direction_output(QPAD_AUDIO_RST, 1);

	
	ret = gpio_request(QPAD_AUDIO_HEADPHONE_DET, "hp_jack");
	if (ret) {
		pr_err("failed to get GPIO hp_jack: %d\n",
			ret);
		return -EINVAL;
	}
	gpio_direction_input(QPAD_AUDIO_HEADPHONE_DET);
	gpio_free(QPAD_AUDIO_HEADPHONE_DET);

	mxc_register_device(&mx6_qpad_audio_rt5625_device,
			&rt5625_data);
	imx6q_add_imx_ssi(1, &mx6_qpad_ssi_pdata);

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

static struct gpio_keys_button qpad_buttons[] = {
	GPIO_BUTTON(GPIO_KEY_MENU, KEY_MENU, 1, "menu", 1, 1),
	GPIO_BUTTON(GPIO_KEY_HOME, KEY_HOME, 1, "home", 1, 1),
	GPIO_BUTTON(GPIO_KEY_BACK, KEY_BACK, 1, "back", 1, 1),	
	GPIO_BUTTON(GPIO_KEY_F1, KEY_F1, 1, "F1", 1, 1),
	GPIO_BUTTON(GPIO_KEY_F2, KEY_F2, 1, "F2", 1, 1),
	GPIO_BUTTON(GPIO_KEY_POWER, KEY_POWER, 1, "power", 1, 1),	
};

static struct gpio_keys_platform_data qpad_button_data = {
	.buttons	= qpad_buttons,
	.nbuttons	= ARRAY_SIZE(qpad_buttons),
};

static struct platform_device qpad_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources  = 0,
};

static void __init imx6q_add_device_buttons(void)
{
	platform_device_add_data(&qpad_button_device,
			&qpad_button_data,
			sizeof(qpad_button_data));

	platform_device_register(&qpad_button_device);
}
#endif

static struct platform_pwm_backlight_data mx6_qpad_pwm_backlight_data = {
	.pwm_id = 0,
	.max_brightness = 248,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

static struct mxc_dvfs_platform_data qpad_dvfscore_data = {
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
	struct ipuv3_fb_platform_data *pdata_fb = qpad_fb_data;

	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "fbmem=");
			if (str != NULL) {
				str += 6;
				pdata_fb[i++].res_size[0] = memparse(str, &str);
				while (*str == ',' &&
					i < ARRAY_SIZE(qpad_fb_data)) {
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



static void __set_switch_linux_flag(void __iomem * snvs_base)
{
#define ANDROID_LINUX_BOOT    (1 << 11)
	u32 reg;
	reg = readl(snvs_base + SNVS_LPGPR);
	reg |= ANDROID_LINUX_BOOT;
	writel(reg, snvs_base + SNVS_LPGPR);
}


static void mx6_snvs_poweroff(void)
{
#define SNVS_LPCR 0x38
	void __iomem *mx6_snvs_base =  MX6_IO_ADDRESS(MX6Q_SNVS_BASE_ADDR);
	u32 value;
	//set linux boot flag before power off
	__set_switch_linux_flag(mx6_snvs_base);
	
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



#define MODEM_PIN_POWER QPAD_MODEM_PWR
#define MODEM_PIN_RESET QPAD_MODEM_RST
#define MODEM_PIN_ONOFF QPAD_MODEM_ONOFF
#define MODEM_PIN_WAKEAP QPAD_MODEM_WAKEAP
#define MODEM_PIN_WAKEMODEM QPAD_MODEM_WAKEMODEM
#include "modem.c"
static int __init modem_init(void){
	//dummy init for modem
	board_modem_init();
	return 0;
	
}


static int __init board_misc_init(void){
	int ret;
	//QR Engine 
	//Set Reset On,Trigger Off
	ret = gpio_request(QPAD_QRE_RST, "qre-rst");
	if (ret) {
		pr_err("failed to get GPIO qre-rst: %d\n",
			ret);
		return -EINVAL;
	}
	gpio_direction_output(QPAD_QRE_RST,0);
	mdelay(5);
	gpio_direction_output(QPAD_QRE_RST,1);
	gpio_free(QPAD_QRE_RST);

	ret = gpio_request(QPAD_QRE_TRIG, "qre-trig");
	if (ret) {
		pr_err("failed to get GPIO qre-trig: %d\n",
			ret);
		return -EINVAL;
	}
	gpio_direction_output(QPAD_QRE_TRIG,1);
	gpio_free(QPAD_QRE_TRIG);

	//Sensor ,FXO8700 driver don't use interrupt but using polling
	//default RST is high ,and no power control
	ret = gpio_request(QPAD_SENSOR_RST, "sensor-rst");
	if (ret) {
		pr_err("failed to get GPIO sensor-pwr: %d\n",
			ret);
		return -EINVAL;
	}
	gpio_direction_output(QPAD_SENSOR_RST,1);
	mdelay(5);
	gpio_direction_output(QPAD_SENSOR_RST,0);
	gpio_free(QPAD_SENSOR_RST);


	//reset WIFI chip
	ret = gpio_request(QPAD_WIFI_RST, "wifi-rst");
	if (ret) {
		pr_err("failed to get GPIO wifi-rst: %d\n",
			ret);
		return -EINVAL;
	}
	gpio_direction_output(QPAD_WIFI_RST,0);
	mdelay(5);
	gpio_direction_output(QPAD_WIFI_RST,1);
	gpio_free(QPAD_WIFI_RST);

	return 0;
}

/*
* Disable unused regulator
*/
static int __init qpad_regulator_late_init(void){
		struct reg_map {
		  struct regulator* regulator;
		  char* supply;
		};
		struct reg_map maps[] = {
			{.regulator=NULL,.supply="VGEN1_1V5"},
			{.regulator=NULL,.supply="VGEN2_1V5"},
			{.regulator=NULL,.supply="VGEN3_2V8"},
		};
		int i=0;
		int count = ARRAY_SIZE(maps);
		do{
			maps[i].regulator = regulator_get(NULL,maps[i].supply);
			if(IS_ERR(maps[i].regulator)){
				printk("failed to get regulator[%s]\n",maps[i].supply);
			}else {
				regulator_enable(maps[i].regulator);
				regulator_disable(maps[i].regulator);
				regulator_put(maps[i].regulator);
			}
			i++;
		}while(i<count);
	
	return 0;
}

late_initcall(qpad_regulator_late_init);

extern u32 enable_ldo_mode;

static int __init qpad_mipi_regulator_init(void){
	if(LDO_MODE_BYPASSED==enable_ldo_mode){
		mipi_dsi_pdata.core_regulator = "MIPI_DSI";
		mipi_dsi_pdata.core_volt	= 2800000;
	}	
	imx6q_add_mipi_dsi(&mipi_dsi_pdata);
	return 0;
}
device_initcall(qpad_mipi_regulator_init);

/*!
 * Board specific initialization.
 */
static void __init mx6_qpad_board_init(void)
{
	int i;
	int ret;

	if (cpu_is_mx6q())
		mxc_iomux_v3_setup_multiple_pads(mx6q_qpad_pads,
			ARRAY_SIZE(mx6q_qpad_pads));
	else if (cpu_is_mx6dl()) {
		mxc_iomux_v3_setup_multiple_pads(mx6dl_qpad_pads,
			ARRAY_SIZE(mx6dl_qpad_pads));
	}

	gp_reg_id = qpad_dvfscore_data.reg_id;
	soc_reg_id = qpad_dvfscore_data.soc_id;
	mx6q_qpad_init_uart();
	
	#ifdef CONFIG_ANDROID_RAM_CONSOLE
	imx6x_add_ram_console();
	#endif

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
		for (i = 0; i < 4 && i < ARRAY_SIZE(qpad_fb_data); i++)
			imx6q_add_ipuv3fb(i, &qpad_fb_data[i]);
	} else
		for (i = 0; i < 2 && i < ARRAY_SIZE(qpad_fb_data); i++)
			imx6q_add_ipuv3fb(i, &qpad_fb_data[i]);

	imx6q_add_vdoa();
	imx6q_add_lcdif(&lcdif_data);
	imx6q_add_ldb(&ldb_data);
	imx6q_add_v4l2_output(0);
	imx6q_add_v4l2_capture(0, &capture_data[0]);
	imx6q_add_v4l2_capture(1, &capture_data[1]);
	imx6q_add_imx_snvs_rtc();

	imx6q_add_imx_caam();

	imx6q_add_device_gpio_leds();

	imx6q_add_imx_i2c(0, &mx6q_i2c_data);
	imx6q_add_imx_i2c(1, &mx6q_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_i2c_data);

	
	if(eBootModeCharger!=android_bootmode){
		i2c_register_board_info(0, mxc_i2c0_board_info,
				ARRAY_SIZE(mxc_i2c0_board_info));
		i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));
		i2c_register_board_info(2, mxc_i2c2_board_info,
				ARRAY_SIZE(mxc_i2c2_board_info));
	}	

	ret = gpio_request(QPAD_PFUZE_INT, "pFUZE-int");
	if (ret) {
		printk(KERN_ERR"request pFUZE-int error!!\n");
		return;
	} else {
		gpio_direction_input(QPAD_PFUZE_INT);
		mx6q_qpad_init_pfuze100(QPAD_PFUZE_INT);
	}
	
	imx6q_add_anatop_thermal_imx(1, &qpad_anatop_thermal_data);
	
	imx6q_add_pm_imx(0, &qpad_pm_data);

	/* Move sd4 to first because sd4 connect to emmc.
	   Mfgtools want emmc is mmcblk0 and other sd card is mmcblk1.
	   mmc0 is emmc
	   mmc1 is sd card
	   mmc2 is wifi
	*/
	imx6q_add_sdhci_usdhc_imx(3, &qpad_sd4_data);
	
	imx6q_qpad_init_usb();
	
	if(eBootModeCharger!=android_bootmode){
		imx6q_add_sdhci_usdhc_imx(1, &qpad_sd2_data);
		imx6q_add_sdhci_usdhc_imx(2, &qpad_sd3_data);

		imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);

		imx6q_add_vpu();
		imx6q_init_audio();
	}
	platform_device_register(&qpad_vmmc_reg_devices);
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm(1);
	imx6q_add_mxc_pwm(2);
	imx6q_add_mxc_pwm(3);
	imx6q_add_mxc_pwm_backlight(0, &mx6_qpad_pwm_backlight_data);

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(1, NULL);
	imx6q_add_dma();

	imx6q_add_dvfs_core(&qpad_dvfscore_data);

	if (imx_ion_data.heaps[0].size)
		imx6q_add_ion(0, &imx_ion_data,
			sizeof(imx_ion_data) + sizeof(struct ion_platform_heap));

	
	#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
	imx6q_add_device_buttons();
	#endif


	if (cpu_is_mx6dl()) {
		imx6dl_add_imx_pxp();
		imx6dl_add_imx_pxp_client();
	}

	/* Register charger chips */
	pm_power_off = mx6_snvs_poweroff;
	imx6q_add_busfreq();

	imx6_add_armpmu();
	
	qpad_power_init();
	
	if(eBootModeCharger!=android_bootmode){
		imx6q_add_perfmon(0);
		imx6q_add_perfmon(1);
		imx6q_add_perfmon(2);
		
		nfc_init();

		modem_init();

		generic_add_w1(QPAD_W1_IO);
	}

	board_misc_init();
	
}

extern void __iomem *twd_base;
static void __init mx6_qpad_timer_init(void)
{
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	early_console_setup(UART1_BASE_ADDR, clk_get_sys("imx-uart.0", NULL));
}

static struct sys_timer mx6_qpad_timer = {
	.init   = mx6_qpad_timer_init,
};



static void __init mx6q_qpad_reserve(void)
{
	phys_addr_t phys;
	int i, fb0_reserved = 0, fb_array_size;

	/*
	 * Reserve primary framebuffer memory if its base address
	 * is set by kernel command line.
	 */
	fb_array_size = ARRAY_SIZE(qpad_fb_data);
	if (fb_array_size > 0 && qpad_fb_data[0].res_base[0] &&
	    qpad_fb_data[0].res_size[0]) {
		if (qpad_fb_data[0].res_base[0] > SZ_2G)
			printk(KERN_INFO"UI Performance downgrade with FB phys address %x!\n",
			    qpad_fb_data[0].res_base[0]);
		memblock_reserve(qpad_fb_data[0].res_base[0],
				 qpad_fb_data[0].res_size[0]);
		memblock_remove(qpad_fb_data[0].res_base[0],
				qpad_fb_data[0].res_size[0]);
		qpad_fb_data[0].late_init = true;
		ipu_data[ldb_data.ipu_id].bypass_reset = true;
		fb0_reserved = 1;
	}
	for (i = fb0_reserved; i < fb_array_size; i++)
		if (qpad_fb_data[i].res_size[0]) {
			/* Reserve for other background buffer. */
			phys = memblock_alloc_base(qpad_fb_data[i].res_size[0],
						SZ_4K, SZ_2G);
			memblock_remove(phys, qpad_fb_data[i].res_size[0]);
			qpad_fb_data[i].res_base[0] = phys;
		}

#ifdef CONFIG_ANDROID_RAM_CONSOLE
	phys = memblock_alloc_base(SZ_1M, SZ_4K, SZ_1G);
	memblock_remove(phys, SZ_1M);
	memblock_free(phys, SZ_1M);
	ram_console_resource.start = phys;
	ram_console_resource.end   = phys + SZ_1M - 1;
#endif

#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
					   SZ_4K, SZ_2G);
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
 * initialize __mach_desc_MX6Q_QPAD data structure.
 */
MACHINE_START(MX6Q_QPAD, "Freescale i.MX 6Quad/DualLite/Solo QPAD")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = mx6_qpad_board_init,
	.timer = &mx6_qpad_timer,
	.reserve = mx6q_qpad_reserve,
MACHINE_END
