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
#include <linux/hrtimer.h>
#include <linux/timed_output.h>
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
#include <linux/i2c/at24.h>
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
#include "board-mx6q_tdh.h"
#include "board-mx6dl_tdh.h"
#include <mach/imx_rfkill.h>
#include <linux/mmc/host.h>
#include <linux/wakelock.h>


#include "generic_devices.h"
#include <linux/pwm.h>

#define BOARD_TDH_REVA 0x1
#define BOARD_TDH_REVB 0x2
#define BOARD_TDH_REVC 0x3


#define FT5X0X_XY_PATCH

typedef struct peripheral_power_state{
	unsigned int sam:1;
	unsigned int wlan:1;
}PERIPHERAL_POWER_STATE_T;

#define TDH_PFUZE_INT		IMX_GPIO_NR(7, 13)


#define TDH_CHARGE_DOK_B	IMX_GPIO_NR(2, 24)
#define TDH_CHARGE_CHG_1_B	IMX_GPIO_NR(3, 23)
#define TDH_CHARGE_FLT_1_B	IMX_GPIO_NR(5, 2)
#define TDH_BATTERY_ALERT	IMX_GPIO_NR(3, 16)

#define TDH_CSI0_PWDN		IMX_GPIO_NR(1, 16)
#define TDH_CSI0_RST		IMX_GPIO_NR(5, 20)

#define GPIO_KEY_POWER		IMX_GPIO_NR(3,29)
#define GPIO_KEY_VOLUP		IMX_GPIO_NR(6,10)
#define GPIO_KEY_VOLDOWN	IMX_GPIO_NR(6,9)
#define GPIO_KEY_PTT		IMX_GPIO_NR(4,10)
#define GPIO_KEY_HOT1		IMX_GPIO_NR(4,14)
#define GPIO_KEY_HOT2		IMX_GPIO_NR(4,11)

#define TDH_DISP_PWR_EN		IMX_GPIO_NR(2, 6)
#define TDH_DISP_BL_PWR_EN	IMX_GPIO_NR(4, 15)
#define TDH_DISP_RST_B		IMX_GPIO_NR(6, 16)


#define TDH_AUDIO_INT				IMX_GPIO_NR(1, 20)
#define TDH_AUDIO_HEADPHONE_DET		IMX_GPIO_NR(7, 12)


//FLASHLIGHT
#define TDH_FL_PWR_EN		IMX_GPIO_NR(3, 22)
#define TDH_FL_EN			IMX_GPIO_NR(3, 31)

//Touch Panel
#define TDH_TP_PWR_EN		IMX_GPIO_NR(2, 28)
#define TDH_TP_RST			IMX_GPIO_NR(6, 8)
#define TDH_TP_IRQ			IMX_GPIO_NR(6, 7)

//WiFi
#define TDH_WIFI_RST		IMX_GPIO_NR(7, 8)
#define TDH_WIFI_PDN		IMX_GPIO_NR(6, 11)
#define TDH_WIFI_WAKEUP	IMX_GPIO_NR(7, 5)

//SDHC
#define TDH_SD2_CD			IMX_GPIO_NR(2, 0)

//USB 
#define TDH_USB_HUB_RST		IMX_GPIO_NR(1, 2)

//SAM
#define TDH_SAM_PWR_EN		IMX_GPIO_NR(1, 5)

//Motor 
#define TDH_MOTOR_PWR_EN	IMX_GPIO_NR(1, 4)

enum {
 eBootModeNormal=0,
 eBootModeRecovery,
 eBootModeCharger,
 eBootModeFastboot,
 eBootModeAutoupdate,
 eBootModeFactory,
 eBootModeMax,
};

//bootmode support
static int android_bootmode __initdata;
static int __init  detect_bootmode(char *arg)
{
	//if(arg&&!strcmp(arg,"charger"))
	//	android_bootmode = eBootModeCharger;

    return 0;
}

__setup("androidboot.mode=", detect_bootmode);

//armpmu support
static int enable_armpmu __initdata;
static int __init  detect_armpmu(char *arg)
{
	enable_armpmu++;
    return 0;
}

__setup("armpmu", detect_armpmu);

//console support
static int console_assigned __initdata = 0;
static int __init  detect_console_assigned(char *str)
{
    console_assigned++;
    return 0;
}

__setup("console=", detect_console_assigned);



//
//
//


extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;

static PERIPHERAL_POWER_STATE_T pps;

/*Micro SD*/
static const struct esdhc_platform_data tdh_sd2_data __initconst = {
	.cd_gpio = TDH_SD2_CD,
	.keep_power_at_suspend = 1,
	.cd_type = ESDHC_CD_CONTROLLER,
	.runtime_pm = 1,
};

/*WIFI SDIO*/
static struct sdhci_host* wlan_sdhc;
static int wlan_wakeup_init=0;
struct wake_lock wlan_wakelock;

static iomux_v3_cfg_t mx6q_wlan_wakeup_pads_func[] = {
	MX6Q_PAD_SD3_DAT1__USDHC3_DAT1_50MHZ,
};
static iomux_v3_cfg_t mx6dl_wlan_wakeup_pads_func[] = {
	MX6DL_PAD_SD3_DAT1__USDHC3_DAT1_50MHZ,
};

static iomux_v3_cfg_t mx6q_wlan_wakeup_pads_io[] = {
	NEW_PAD_CTRL(MX6Q_PAD_SD3_DAT1__GPIO_7_5,MX6Q_GENERIC_PAD_CTRL),
};
static iomux_v3_cfg_t mx6dl_wlan_wakeup_pads_io[] = {
	NEW_PAD_CTRL(MX6DL_PAD_SD3_DAT1__GPIO_7_5, MX6DL_GENERIC_PAD_CTRL),
};

static irqreturn_t wlan_wakup_handler(int irq, void *data){
	   wake_lock_timeout(&wlan_wakelock, HZ * 2);
       return IRQ_HANDLED;
}
static int wlan_wakeup_add(void){
       int ret=0;
	   if(!wlan_wakeup_init){
	       gpio_request(TDH_WIFI_WAKEUP,"wifi-wakeup");
	       gpio_direction_input(TDH_WIFI_WAKEUP);
		   wake_lock_init(&wlan_wakelock , WAKE_LOCK_SUSPEND, "wlan wakelock");
		   wlan_wakeup_init++;
	   }
	   //we can't use both edge trigger ,otherwise GPC will be waken up immediately and we 
	   //will enter an infinite loop
	   ret = request_any_context_irq(gpio_to_irq(TDH_WIFI_WAKEUP), wlan_wakup_handler,
			   IRQF_NO_SUSPEND|IRQF_TRIGGER_FALLING ,
			   "wlan wakeap", 0);
	   if (ret) {
			   pr_warning("Request wlan wakeup failed %d\n", ret);
	   }else {
			   enable_irq_wake(gpio_to_irq(TDH_WIFI_WAKEUP));
	   }
       return ret;
}
static int wlan_wakeup_enable(void){
	//switch to gpio mode
	if (cpu_is_mx6q())
		mxc_iomux_v3_setup_multiple_pads(mx6q_wlan_wakeup_pads_io,1);
	else if(cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_wlan_wakeup_pads_io,1);
	return 0;
}

static int wlan_wakeup_disable(void)
{
	if (cpu_is_mx6q())
		mxc_iomux_v3_setup_multiple_pads(mx6q_wlan_wakeup_pads_func,1);
	else if(cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_wlan_wakeup_pads_func,1);
	return 0;
}

static int wlan_wakeup_remove(void){
	disable_irq_wake(gpio_to_irq(TDH_WIFI_WAKEUP));
	free_irq(gpio_to_irq(TDH_WIFI_WAKEUP),0);
	return 0;
}

static int wlan_host_init_cb(struct sdhci_host* host){
	wlan_sdhc = host;
	return 0;
}
static const struct esdhc_platform_data tdh_sd3_data __initconst= {
	.keep_power_at_suspend = 1,
	.cd_type = ESDHC_CD_NONE,
	.sdhc_host_init_cb = wlan_host_init_cb,
};

/*eMMC*/
static const struct esdhc_platform_data tdh_sd4_data __initconst = {
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.support_8bit = 1,
	.delay_line = 0,
	.cd_type = ESDHC_CD_PERMANENT,
};

static const struct anatop_thermal_platform_data
	tdh_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

static void qpad_uart_io_switch(int uart_idx/*0 based*/,int switch2uart)
{
	iomux_v3_cfg_t mx6q_uart_gpio_pads[] = {
		MX6Q_PAD_CSI0_DAT10__GPIO_5_28,
		MX6Q_PAD_CSI0_DAT11__GPIO_5_29,
		MX6Q_PAD_EIM_D26__GPIO_3_26,
		MX6Q_PAD_EIM_D27__GPIO_3_27,
		MX6Q_PAD_EIM_D24__GPIO_3_24,
		MX6Q_PAD_EIM_D25__GPIO_3_25,
		MX6Q_PAD_KEY_COL0__GPIO_4_6,
		MX6Q_PAD_KEY_ROW0__GPIO_4_7,		
		MX6Q_PAD_KEY_COL1__GPIO_4_8,
		MX6Q_PAD_KEY_ROW1__GPIO_4_9,
	};
	iomux_v3_cfg_t mx6q_uart_func_pads[] = {
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
	};
	
	iomux_v3_cfg_t mx6dl_uart_gpio_pads[] = {
		MX6DL_PAD_CSI0_DAT10__GPIO_5_28,
		MX6DL_PAD_CSI0_DAT11__GPIO_5_29,
		MX6DL_PAD_EIM_D26__GPIO_3_26,
		MX6DL_PAD_EIM_D27__GPIO_3_27,
		MX6DL_PAD_EIM_D24__GPIO_3_24,
		MX6DL_PAD_EIM_D25__GPIO_3_25,
		MX6DL_PAD_KEY_COL0__GPIO_4_6,
		MX6DL_PAD_KEY_ROW0__GPIO_4_7,		
		MX6DL_PAD_KEY_COL1__GPIO_4_8,
		MX6DL_PAD_KEY_ROW1__GPIO_4_9,
	};
	iomux_v3_cfg_t mx6dl_uart_func_pads[] = {
		MX6DL_PAD_CSI0_DAT10__UART1_TXD,
		MX6DL_PAD_CSI0_DAT11__UART1_RXD,
		MX6DL_PAD_EIM_D26__UART2_TXD,
		MX6DL_PAD_EIM_D27__UART2_RXD,
		MX6DL_PAD_EIM_D24__UART3_TXD,
		MX6DL_PAD_EIM_D25__UART3_RXD,
		MX6DL_PAD_KEY_COL0__UART4_TXD,
		MX6DL_PAD_KEY_ROW0__UART4_RXD,
		MX6DL_PAD_KEY_COL1__UART5_TXD,
		MX6DL_PAD_KEY_ROW1__UART5_RXD,
	};
	
	iomux_v3_cfg_t* io_pads_cfg = switch2uart ? &mx6q_uart_func_pads[uart_idx*2] : &mx6q_uart_gpio_pads[uart_idx*2];
	if(cpu_is_mx6dl())
		io_pads_cfg = switch2uart ? &mx6dl_uart_func_pads[uart_idx*2] : &mx6dl_uart_gpio_pads[uart_idx*2];
	mxc_iomux_v3_setup_multiple_pads(io_pads_cfg,2/*fixed 2 pins*/);
}


static struct imxuart_platform_data mx6_uart_pdata __initdata = {
       .flags      = IMXUART_CON_DISABLE,
};

static inline void mx6q_tdh_init_uart(void)
{
	if(console_assigned)
		mx6_uart_pdata.flags&=~IMXUART_CON_DISABLE;
	imx6q_add_imx_uart(0, &mx6_uart_pdata);	
	imx6q_add_imx_uart(1, &mx6_uart_pdata);
	imx6q_add_imx_uart(2, &mx6_uart_pdata);
	imx6q_add_imx_uart(3, &mx6_uart_pdata);
	imx6q_add_imx_uart(4, &mx6_uart_pdata);	
}


static struct fsl_mxc_camera_platform_data camera_data;
static struct regulator* camera_regulator;

static void mx6q_csi0_mclk_on(int on){
	struct clk *mclk = clk_get(NULL, "clko_clk");
	if(!IS_ERR(mclk)){		
		on?clk_enable(mclk):clk_disable(mclk);
		clk_put(mclk);
	}
}

static void mx6q_csi0_cam_powerdown(int pdn){	
	if (pdn){
		gpio_set_value(TDH_CSI0_PWDN, 1);
	}else{
		gpio_set_value(TDH_CSI0_PWDN, 0);
	}

	msleep(2);
}

static void mx6q_csi0_io_init(void)
{	
	int cam_pdn = TDH_CSI0_PWDN;
	int cam_rst = TDH_CSI0_RST;
	struct clk *clko2 = clk_get(NULL, "clko2_clk");
	struct clk *clko = clk_get(NULL, "clko_clk");
	if(!IS_ERR(clko)&&!IS_ERR(clko2)){
		//share the same clk of clko and clko2
		clk_set_parent(clko, clko2);
		clk_put(clko);
		clk_put(clko2);
	}

	if (cpu_is_mx6q())
		mxc_iomux_v3_setup_multiple_pads(mx6q_tdh_csi0_sensor_pads,
			ARRAY_SIZE(mx6q_tdh_csi0_sensor_pads));
	else if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_tdh_csi0_sensor_pads,
			ARRAY_SIZE(mx6dl_tdh_csi0_sensor_pads));

	camera_regulator = regulator_get(NULL,"VGEN3_2V8");
	if(IS_ERR(camera_regulator)){
		printk("failed to get regulator[VGEN3_2V8]\n");
		camera_regulator = NULL;
	}else {
		regulator_enable(camera_regulator);
	}

	if (gpio_request(cam_rst, "cam-rst")) {
		printk(KERN_ERR"Request GPIO failed,"
				"gpio: %d \n", cam_rst);
	}else {
		gpio_direction_output(cam_rst,1);
	}

	if (gpio_request(cam_pdn, "cam-pdn")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", cam_pdn);
	}else {
		gpio_direction_output(cam_pdn,1);
	}
	msleep(5);
	gpio_set_value(cam_pdn, 0);
	gpio_set_value(cam_rst, 0);
	msleep(1);
	gpio_set_value(cam_rst, 1);
	msleep(5);
	gpio_set_value(cam_pdn, 1);
		
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
	.mclk = 24000000,/* 6 - 54 MHz, typical 24MHz */
	.mclk_source = 0,
	.csi = 0,
	.io_init = mx6q_csi0_io_init,
	.mclk_on = mx6q_csi0_mclk_on,
	.pwdn = mx6q_csi0_cam_powerdown,
	.trigger_led_flash = "camera",
};

#include <linux/i2c/eup2471.h>


static int fl_pwren=TDH_FL_PWR_EN;
static int fl_en=TDH_FL_EN;
static int eup2471_enable(int on)
{
	int en = fl_pwren;

	if (gpio_request(en, "eup2471 enable")) {
		printk(KERN_INFO "gpio %d request failed\n", en);
		return -EINVAL;
	}

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
	int en = fl_en;

	if (gpio_request(en, "eup2471 flash on")) {
		printk(KERN_INFO "gpio %d request failed\n", en);
		return -EINVAL;
	}

	if (on)
	{
		gpio_direction_output(en, 1);
		udelay(10);

		gpio_direction_output(en, 0);
	}
	else
	{
		gpio_direction_output(en, 1);
	}

	gpio_free(en);

	return 0;
}

static struct eup2471_platform_data eup2471_pdata = 
{
	.default_trigger = "camera",
	.name		= "flashlight:camera",
	.enable 	= eup2471_enable,
	.flash 		= eup2471_flash,
};

#include <linux/ft5x0x_ts.h>

static int ft5x0x_set_power(int on){
	int ret=0;
	ret = gpio_request(TDH_TP_PWR_EN, "tp-pwr");
	if (ret) {
		pr_err("failed to get GPIO tp-pwr: %d\n",
			ret);
		goto err;
	}
	gpio_direction_output(TDH_TP_PWR_EN, on?1:0);
	gpio_free(TDH_TP_PWR_EN);

	ret = gpio_request(TDH_TP_RST, "tp-rst");
	if (ret) {
		pr_err("failed to get GPIO tp-rst: %d\n",
			ret);
		goto err;
	}
	
	if(1==on){
		gpio_direction_output(TDH_TP_RST, 1);
		msleep(50);
	}else {
		gpio_direction_output(TDH_TP_RST, 0);
	}
	
	gpio_free(TDH_TP_RST);
err:
	return ret;
}
static int ft5x0x_plat_init(void){
	int ret=0;
	ret = gpio_request(TDH_TP_PWR_EN, "tp-pwr");
	if (ret) {
		pr_err("failed to get GPIO tp-pwr: %d\n",
			ret);
		goto err;
	}
	gpio_direction_output(TDH_TP_PWR_EN, 1);
	gpio_free(TDH_TP_PWR_EN);

	ret = gpio_request(TDH_TP_RST, "tp-rst");
	if (ret) {
		pr_err("failed to get GPIO tp-rst: %d\n",
			ret);
		goto err;
	}
	
	gpio_direction_output(TDH_TP_RST, 0);
	msleep(50);
	gpio_direction_output(TDH_TP_RST, 1);
	msleep(50);
	
	gpio_free(TDH_TP_RST);
err:
	return ret;
}

static struct ft5x0x_ts_platform_data ft5x0x_data=
{
	.setpower	= ft5x0x_set_power,
	.plat_init	= ft5x0x_plat_init,
};


static struct at24_platform_data eeprom_data = {
	.byte_len	= SZ_32K / 8,	
	.page_size	= 32,	
	.flags		= AT24_FLAG_ADDR16|AT24_FLAG_IRUGO,
};


static struct imxi2c_platform_data mx6q_i2c_data = {
	.bitrate = 100000,
};


static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("rt5625", 0x1f),
	},
	{
		I2C_BOARD_INFO("HM5065", 0x10),
		.platform_data = (void *)&camera_data,
	},
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		.type			= "ft5x0x_ts",
		.addr			= 0x38,
		.irq			= gpio_to_irq(TDH_TP_IRQ),
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
		I2C_BOARD_INFO("at24", 0x54),
		.platform_data	= &eeprom_data,	
	},
};



static void tdh_usbotg_vbus(bool on)
{
	pr_warn("no otg host vbus power switch feature on tdh\n");
}

static void tdh_host1_vbus(bool on)
{
	pr_warn("no host1 vbus power switch feature on tdh\n");
}

static void __init imx6q_tdh_init_usb(void)
{
	//NO OC input
	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	/*
	 *
	 *OTG ID daisy chain set
	 * 0: ENET_RX_ER as USB_OTG_ID 
	 * 1: GPIO_1 as USB_OTG_ID
	 */
	mxc_iomux_set_gpr_register(1, 13, 1, 1);
	mx6_set_otghost_vbus_func(tdh_usbotg_vbus);
	mx6_set_host1_vbus_func(tdh_host1_vbus);

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
	static int skip_first_reset=1;
	int ret;

	if(skip_first_reset){
		skip_first_reset=0;
		return;
	}
	//return if bootloader already enable LCD???	
	ret = gpio_request(TDH_DISP_RST_B, "disp_reset");	
	if (ret) {
		pr_err("failed to request gpio: %d\n",TDH_DISP_RST_B);
		return;
	}
	gpio_direction_output(TDH_DISP_RST_B, 0);
	msleep(10);
	gpio_direction_output(TDH_DISP_RST_B, 1);
	
	gpio_free(TDH_DISP_RST_B);

	/*
	 * it needs to delay 120ms minimum for reset complete
	 */
	msleep(10);
}

static void mipi_dsi_power(int en){	
	int ret;
	ret = gpio_request(TDH_DISP_PWR_EN, "disp_pwren");	
	if (ret) {
		pr_err("failed to request gpio: %d\n",TDH_DISP_PWR_EN);
		return;
	}
	gpio_direction_output(TDH_DISP_PWR_EN,en?0:1);
	gpio_free(TDH_DISP_PWR_EN);
}
static void mipi_dsi_baclight_power(int en){
	int ret;
	ret = gpio_request(TDH_DISP_BL_PWR_EN, "disp_bl_pwren");	
	if (ret) {
		pr_err("failed to request gpio: %d\n",TDH_DISP_BL_PWR_EN);
		return;
	}
	gpio_direction_output(TDH_DISP_BL_PWR_EN,en?1:0);
	gpio_free(TDH_DISP_BL_PWR_EN);
}
static struct mipi_dsi_platform_data mipi_dsi_pdata = {
	.ipu_id		= 0,
	.disp_id	= 1,
	.lcd_panel	= "NT-QHD",
	.reset		= mx6_reset_mipi_dsi,
	.lcd_power	= mipi_dsi_power,
	.backlight_power	= mipi_dsi_baclight_power,

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

static struct ipuv3_fb_platform_data tdh_fb_data[] = {
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




static struct qpower_battery_pdata qbp = {
	.alert = TDH_BATTERY_ALERT,
	.busid = 0,
};
static struct qpower_charger_pdata qcp = {
	.dok = TDH_CHARGE_DOK_B,
	.chg = TDH_CHARGE_CHG_1_B,
	.flt = TDH_CHARGE_FLT_1_B,	
	.dcm_always_high = true,
	.dc_valid = true,
	.usb_valid = false,
	.feature_flag = QPOWER_CHARGER_FEATURE_SHORT_MODE,
};
static struct qpower_pdata qp = {
	.bp = &qbp,
	.cp = &qcp,
	.flags = QPOWER_FEATURE_CHARGER|QPOWER_FEATURE_BATTERY,  
};

static int __init tdh_power_init(void){
	if (cpu_is_mx6q())
		mxc_iomux_v3_setup_pad(MX6Q_PAD_EIM_D16__GPIO_3_16);
	else if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_pad(MX6DL_PAD_EIM_D16__GPIO_3_16);

	qbp.alert = IMX_GPIO_NR(3,16);
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
		.size = SZ_32M,
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
		.flag = MXC_CAMERA_FLAG_POWER_UP_SEQUENCE_MCLK_FIRST|MXC_CAMERA_FLAG_POWER_DOWN_SEQUENCE_POWER_FIRST,
		.reserved_mem_size = SZ_128M,
	}
};

struct imx_vout_mem {
	resource_size_t res_mbase;
	resource_size_t res_msize;
};

static struct imx_vout_mem vout_mem __initdata = {
	.res_msize = 0,
};

static void tdh_suspend_prepare(void){
	if(pps.wlan) wlan_wakeup_add();
}
static void tdh_suspend_wake(void){
	if(pps.wlan) wlan_wakeup_remove();
}
static void tdh_suspend_enter(void)
{
	if(pps.wlan) wlan_wakeup_enable();
}

static void tdh_suspend_exit(void)
{
	if(pps.wlan) wlan_wakeup_disable();
}
static const struct pm_platform_data tdh_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_prepare = tdh_suspend_prepare,
	.suspend_wake = tdh_suspend_wake,
	.suspend_enter = tdh_suspend_enter,
	.suspend_exit = tdh_suspend_exit,
};

static struct regulator_consumer_supply tdh_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.0"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),	
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data tdh_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(tdh_vmmc_consumers),
	.consumer_supplies = tdh_vmmc_consumers,
};

static struct fixed_voltage_config tdh_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &tdh_vmmc_init,
};

static struct platform_device tdh_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &tdh_vmmc_reg_config,
	},
};



static struct imx_ssi_platform_data mx6_tdh_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct platform_device mx6_tdh_audio_rt5625_device = {
	.name = "imx-rt5625",
};

static struct mxc_audio_platform_data rt5625_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.hp_gpio = TDH_AUDIO_HEADPHONE_DET,
	.hp_active_low = 0,
};



static int __init imx6q_init_audio(void)
{
	int ret,rate;	
	struct clk *clko2;

	clko2 = clk_get(NULL, "clko2_clk");
	if (IS_ERR(clko2)){
		pr_err("can't get clko2 clock.\n");
	}
	else {
		rate = clk_round_rate(clko2, 24000000);
		rt5625_data.sysclk = rate;
		clk_set_rate(clko2, rate);
		clk_put(clko2);
	}
	
	ret = gpio_request(TDH_AUDIO_HEADPHONE_DET, "hp_jack");
	if (ret) {
		pr_err("failed to get GPIO hp_jack: %d\n",
			ret);
		return -EINVAL;
	}
	gpio_direction_input(TDH_AUDIO_HEADPHONE_DET);
	gpio_free(TDH_AUDIO_HEADPHONE_DET);

	mxc_register_device(&mx6_tdh_audio_rt5625_device,
			&rt5625_data);
	imx6q_add_imx_ssi(1, &mx6_tdh_ssi_pdata);

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
#define GPIO_BUTTON2(gpio_num, ev_code, act_low, descr, wake, debounce,callback)	\
{								\
	.gpio		= gpio_num,				\
	.type		= EV_KEY,				\
	.code		= ev_code,				\
	.active_low	= act_low,				\
	.desc		= "btn " descr,				\
	.wakeup		= wake,					\
	.debounce_interval = debounce,				\
	.event_callback = callback, \
}


//to fix powerbutton force powered off may trigger rtc alarm event not clear issue
static void mx6_snvs_fixup(void);
static void snvs_fixup_work(struct work_struct *work){
	mx6_snvs_fixup();
}

static DECLARE_DELAYED_WORK(snvs_work, snvs_fixup_work);

static void gpio_key_event(unsigned int code,int value){
	if(KEY_POWER==code){
		if(value)
			schedule_delayed_work(&snvs_work,msecs_to_jiffies(4000));
		else
			cancel_delayed_work_sync(&snvs_work);
	}
}


static struct gpio_keys_button tdh_buttons[] = {
	GPIO_BUTTON(GPIO_KEY_VOLUP, KEY_VOLUMEUP, 1, "vol+", 1, 1),
	GPIO_BUTTON(GPIO_KEY_VOLDOWN, KEY_VOLUMEDOWN, 1, "vol-", 1, 1),
	GPIO_BUTTON(GPIO_KEY_PTT, KEY_RADIO, 1, "ptt", 1, 1),
	GPIO_BUTTON(GPIO_KEY_HOT1, KEY_CHANNELUP, 1, "channel+", 1, 1),
	GPIO_BUTTON(GPIO_KEY_HOT2, KEY_CHANNELDOWN, 1, "channel-", 1, 1),
	GPIO_BUTTON2(GPIO_KEY_POWER, KEY_POWER, 1, "power", 1, 1,gpio_key_event),
};

static struct gpio_keys_platform_data tdh_button_data = {
	.buttons	= tdh_buttons,
	.nbuttons	= ARRAY_SIZE(tdh_buttons),
};


static struct platform_device tdh_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources  = 0,
};

static void __init imx6q_add_device_buttons(void)
{
	platform_device_add_data(&tdh_button_device,
			&tdh_button_data,
			sizeof(tdh_button_data));

	platform_device_register(&tdh_button_device);
}
#endif

static struct platform_pwm_backlight_data mx6_tdh_pwm_backlight_data = {
	.pwm_id = 0,
	.max_brightness = 248,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

static struct mxc_dvfs_platform_data tdh_dvfscore_data = {
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
	struct ipuv3_fb_platform_data *pdata_fb = tdh_fb_data;

	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "fbmem=");
			if (str != NULL) {
				str += 6;
				pdata_fb[i++].res_size[0] = memparse(str, &str);
				while (*str == ',' &&
					i < ARRAY_SIZE(tdh_fb_data)) {
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

static void mx6_snvs_fixup(void)
{
#define SNVS_LPCR 0x38
	void __iomem *mx6_snvs_base =  MX6_IO_ADDRESS(MX6Q_SNVS_BASE_ADDR);
	u32 value;
	value = readl(mx6_snvs_base + SNVS_LPCR);
	//clear alarm enable and wakeup enable
	value &=~0xA;
	writel(value, mx6_snvs_base + SNVS_LPCR);
}

static void mx6_snvs_poweroff(void)
{
#define SNVS_LPCR 0x38
	void __iomem *mx6_snvs_base =  MX6_IO_ADDRESS(MX6Q_SNVS_BASE_ADDR);
	u32 value;
	//set linux boot flag before power off
	__set_switch_linux_flag(mx6_snvs_base);
	
	value = readl(mx6_snvs_base + SNVS_LPCR);
	//clear alarm enable and wakeup enable
	value &=~0xA;
	printk("SNVS_LPCR=0x%x\n",value);
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

static int wlan_bt_power_change(int status)
{
	if (status){
			int ret = gpio_request(TDH_WIFI_PDN, "wifi-pdn");		
			if (ret) {
				pr_err("failed to get GPIO wifi-pdn: %d\n",
					ret);
				return -EINVAL;
			}
			gpio_direction_output(TDH_WIFI_PDN,1);
			gpio_free(TDH_WIFI_PDN);
			msleep(10);
			ret = gpio_request(TDH_WIFI_RST, "wifi-rst");
			if (ret) {
				pr_err("failed to get GPIO wifi-rst: %d\n",
					ret);
				return -EINVAL;
			}
			gpio_direction_output(TDH_WIFI_RST,1);
			gpio_free(TDH_WIFI_RST);
			pps.wlan = 1;
			msleep(200);
		}
		else {
			//always put wifi chip into reset state to save power???
			int ret = gpio_request(TDH_WIFI_PDN, "wifi-pdn");		
			if (ret) {
				pr_err("failed to get GPIO wifi-pdn: %d\n",
					ret);
				return -EINVAL;
			}
			gpio_direction_output(TDH_WIFI_PDN,0);
			gpio_free(TDH_WIFI_PDN);

			ret = gpio_request(TDH_WIFI_RST, "wifi-rst");
			if (ret) {
				pr_err("failed to get GPIO wifi-rst: %d\n",
					ret);
				return -EINVAL;
			}
			gpio_direction_output(TDH_WIFI_RST,0);
			gpio_free(TDH_WIFI_RST);
			pps.wlan=0;
		}

	if(wlan_sdhc&&wlan_sdhc->mmc){
		mmc_detect_change(wlan_sdhc->mmc,0);
		msleep(100);
	}

	return 0;
}


static struct platform_device wlan_bt_rfkill = {
	.name = "mxc_bt_rfkill",
};

static struct imx_bt_rfkill_platform_data wlan_bt_rfkill_data = {
	.name = "bluetooth",
	.power_change = wlan_bt_power_change,
};



static struct work_struct vibrator_work;
static struct hrtimer vibe_timer;
static spinlock_t vibe_lock;
static int vibe_state;

static void set_vibrator(int on)
{
	int gpio = TDH_MOTOR_PWR_EN;
	if (gpio_request(gpio, "vibrator enable gpio")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio);
		return;
	}

	if (on) 
	{
		gpio_direction_output(gpio, 1);
	} 
	else 
	{
		gpio_direction_output(gpio, 0);
	}

	gpio_free(gpio);

}

static void update_vibrator(struct work_struct *work)
{
      set_vibrator(vibe_state);
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
      unsigned long   flags;

      spin_lock_irqsave(&vibe_lock, flags);
      hrtimer_cancel(&vibe_timer);

      if (value == 0)
              vibe_state = 0;
      else {
              value = (value > 15000 ? 15000 : value);
			  printk("enable vibrator %d ms\n",value);
              vibe_state = 1;
              hrtimer_start(&vibe_timer,
                      ktime_set(value / 1000, (value % 1000) * 1000000),
                      HRTIMER_MODE_REL);
      }
      spin_unlock_irqrestore(&vibe_lock, flags);

      schedule_work(&vibrator_work);
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
      if (hrtimer_active(&vibe_timer)) {
              ktime_t r = hrtimer_get_remaining(&vibe_timer);
			  struct timeval t = ktime_to_timeval(r);
              return t.tv_sec * 1000 + t.tv_usec / 1000;
      } else
              return 0;
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
      vibe_state = 0;
      schedule_work(&vibrator_work);
      return HRTIMER_NORESTART;
}

static struct timed_output_dev vibrator = {
      .name = "vibrator",
      .get_time = vibrator_get_time,
      .enable = vibrator_enable,
};

static void board_vibrator_init(void)
{
      INIT_WORK(&vibrator_work, update_vibrator);

      spin_lock_init(&vibe_lock);
      vibe_state = 0;
      hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
      vibe_timer.function = vibrator_timer_func;

      timed_output_dev_register(&vibrator);
}


#define TDH_MODEM_ONOFF		IMX_GPIO_NR(2, 2)
#define TDH_MODEM_RST			IMX_GPIO_NR(2, 3)
#define TDH_MODEM_PWR			IMX_GPIO_NR(2, 4)
#define TDH_MODEM_WAKEMODEM	IMX_GPIO_NR(2, 5)
#define TDH_MODEM_WAKEAP		IMX_GPIO_NR(1, 18)
#define TDH_MODEM_AP_SLEEP_STATUS	IMX_GPIO_NR(1, 19)
#define TDH_MODEM_CP_SLEEP_STATUS	IMX_GPIO_NR(1, 17)

#define MODEM_PIN_POWER TDH_MODEM_PWR
#define MODEM_PIN_RESET TDH_MODEM_RST
#define MODEM_PIN_ONOFF TDH_MODEM_ONOFF
#define MODEM_PIN_WAKEAP TDH_MODEM_WAKEAP
#define MODEM_PIN_WAKEMODEM TDH_MODEM_WAKEMODEM
#define MODEM_AP_SLEEP_STATUS TDH_MODEM_AP_SLEEP_STATUS
#define MODEM_CP_SLEEP_STATUS TDH_MODEM_CP_SLEEP_STATUS

#define MODEM_INIT_EXCLUDE 1
#include "modem.c"
static int __init modem_init(void){
	//dummy init for modem
	board_modem_init();
	return 0;
}

static int read_board_revision(char *page, char **start,
			     off_t off, int count,
			     int *eof, void *data)
{
	return sprintf(page, "v%d",mx6_board_rev());
}

static void sam_power(int on){
	gpio_set_value(TDH_SAM_PWR_EN,on?1:0);
	pps.sam=on?1:0;
	qpad_uart_io_switch(3,pps.sam);
}
static int read_sam(char *page, char **start,
			     off_t off, int count,
			     int *eof, void *data)
{
	return sprintf(page, "current sam state %s\n"
						"echo 1 > sam -->enable sam reader power\r\n"
						 "echo 0 > sam -->disable smartcard read power\r\n"
						 "\n",pps.sam?"on":"off");
}

static int write_sam(struct file *file, const char *buffer,
                      unsigned long count, void *data) 
{
	char kbuf[256];
	int action;

	if (count >= 256)
		return -EINVAL;
	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;

	action = (int)simple_strtoul(kbuf, NULL, 10);
	sam_power(action);
	
    return count;
}


static int __init board_misc_init(void){
	int ret;
	struct proc_dir_entry *entry;
	eup2471_enable(0);
	eup2471_flash(0);

	
	ret = gpio_request(TDH_SAM_PWR_EN, "sam power");
	if (ret) {
		pr_err("failed to get GPIO SmartCardPwr %d\n",
			ret);
		return -EINVAL;
	}
	pps.sam=0;
	gpio_direction_output(TDH_SAM_PWR_EN,0); 	
	//default state is power off ,to fix current leak issue,switch uart to io mode
	qpad_uart_io_switch(3,pps.sam);
	entry = create_proc_entry("driver/sam", S_IFREG | S_IRUGO | S_IWUGO, NULL);
	if (entry) {
		entry->read_proc = read_sam;
		entry->write_proc = write_sam;
	}

	entry = create_proc_entry("boardrev", S_IFREG | S_IRUGO | S_IWUSR, NULL);
	if (entry) {
		entry->read_proc = read_board_revision;
	}

	board_vibrator_init();
	
	return 0;
}

/*
* Disable unused regulator
*/
static int __init tdh_regulator_late_init(void){
		struct reg_map {
		  struct regulator* regulator;
		  char* supply;
		};
		struct reg_map maps[] = {
			{.regulator=NULL,.supply="VGEN1_1V5"},
			{.regulator=NULL,.supply="VGEN2_1V5"},
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

late_initcall(tdh_regulator_late_init);

extern u32 enable_ldo_mode;

static int __init tdh_mipi_regulator_init(void){
	if(LDO_MODE_BYPASSED==enable_ldo_mode){
		mipi_dsi_pdata.core_regulator = "MIPI_DSI";
		mipi_dsi_pdata.core_volt	= 2800000;
	}	
	imx6q_add_mipi_dsi(&mipi_dsi_pdata);
	return 0;
}
device_initcall(tdh_mipi_regulator_init);

/*!
 * Board specific initialization.
 */
static void __init mx6_tdh_board_init(void)
{
	int i;
	int ret;
	struct platform_device *voutdev;
	struct platform_device *capdev;

	if (cpu_is_mx6q())
		mxc_iomux_v3_setup_multiple_pads(mx6q_tdh_pads,
			ARRAY_SIZE(mx6q_tdh_pads));
	else if (cpu_is_mx6dl()) {
		mxc_iomux_v3_setup_multiple_pads(mx6dl_tdh_pads,
			ARRAY_SIZE(mx6dl_tdh_pads));
	}
	printk("TDH board id=0x%x rev=0x%x\n",mx6_board_id(),mx6_board_rev());
	gp_reg_id = tdh_dvfscore_data.reg_id;
	soc_reg_id = tdh_dvfscore_data.soc_id;
	mx6q_tdh_init_uart();
	
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
		for (i = 0; i < 4 && i < ARRAY_SIZE(tdh_fb_data); i++)
			imx6q_add_ipuv3fb(i, &tdh_fb_data[i]);
	} else
		for (i = 0; i < 2 && i < ARRAY_SIZE(tdh_fb_data); i++)
			imx6q_add_ipuv3fb(i, &tdh_fb_data[i]);

	imx6q_add_vdoa();
	imx6q_add_lcdif(&lcdif_data);
	imx6q_add_ldb(&ldb_data);
	voutdev = imx6q_add_v4l2_output(0);
	if (vout_mem.res_msize && voutdev) {
		dma_declare_coherent_memory(&voutdev->dev,
					    vout_mem.res_mbase,
					    vout_mem.res_mbase,
					    vout_mem.res_msize,
					    (DMA_MEMORY_MAP |
					     DMA_MEMORY_EXCLUSIVE));
	}
	capdev = imx6q_add_v4l2_capture(0, &capture_data[0]);
	imx6q_add_imx_snvs_rtc();
	if (capture_data[0].reserved_mem_size && capdev) {
		int dma = dma_declare_coherent_memory(&capdev->dev,
						capture_data[0].reserved_mem_base,
						capture_data[0].reserved_mem_base,
						capture_data[0].reserved_mem_size,
						DMA_MEMORY_MAP | DMA_MEMORY_EXCLUSIVE);
		if ((dma & DMA_MEMORY_MAP) && !capdev->dev.dma_mask) {
			capdev->dev.dma_mask =
				kmalloc(sizeof(*capdev->dev.dma_mask), GFP_KERNEL);
			if (capdev->dev.dma_mask)
				*capdev->dev.dma_mask = DMA_BIT_MASK(32);
			capdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
		}
	}

	imx6q_add_imx_caam();

	imx6q_add_device_gpio_leds();

	imx6q_add_imx_i2c(0, &mx6q_i2c_data);
	imx6q_add_imx_i2c(1, &mx6q_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_i2c_data);

	
	if(eBootModeCharger!=android_bootmode){
		#ifdef FT5X0X_XY_PATCH
		//patch for TP y axis inverted
		if(BOARD_TDH_REVB==mx6_board_rev()){
			ft5x0x_data.y_inverted=1;
		}
		#endif
		i2c_register_board_info(0, mxc_i2c0_board_info,
				ARRAY_SIZE(mxc_i2c0_board_info));
		i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));
		i2c_register_board_info(2, mxc_i2c2_board_info,
				ARRAY_SIZE(mxc_i2c2_board_info));
	}	

	ret = gpio_request(TDH_PFUZE_INT, "pFUZE-int");
	if (ret) {
		printk(KERN_ERR"request pFUZE-int error!!\n");
		return;
	} else {
		gpio_direction_input(TDH_PFUZE_INT);
		mx6q_tdh_init_pfuze100(TDH_PFUZE_INT);
	}
	
	imx6q_add_anatop_thermal_imx(1, &tdh_anatop_thermal_data);
	
	imx6q_add_pm_imx(0, &tdh_pm_data);

	/* Move sd4 to first because sd4 connect to emmc.
	   Mfgtools want emmc is mmcblk0 and other sd card is mmcblk1.
	   mmc0 is emmc
	   mmc1 is sd card
	   mmc2 is wifi
	*/
	imx6q_add_sdhci_usdhc_imx(3, &tdh_sd4_data);
	
	imx6q_tdh_init_usb();
	
	if(eBootModeCharger!=android_bootmode){
		
		mxc_register_device(&wlan_bt_rfkill, &wlan_bt_rfkill_data);
		imx6q_add_sdhci_usdhc_imx(1, &tdh_sd2_data);
		imx6q_add_sdhci_usdhc_imx(2, &tdh_sd3_data);

		

		imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);

		imx6q_add_vpu();
		imx6q_init_audio();
	}
	platform_device_register(&tdh_vmmc_reg_devices);
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm_backlight(0, &mx6_tdh_pwm_backlight_data);

	//For fast boot
	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(1, NULL);
	imx6q_add_dma();

	imx6q_add_dvfs_core(&tdh_dvfscore_data);

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

	
	tdh_power_init();

	//enable armpmu if necessary
	if(enable_armpmu){
		imx6_add_armpmu();
		imx6q_add_perfmon(0);
		imx6q_add_perfmon(1);
		imx6q_add_perfmon(2);
		
	}
	if(eBootModeCharger!=android_bootmode){
		modem_init();
	}

	board_misc_init();
	
}

extern void __iomem *twd_base;


static void __init mx6_tdh_timer_init(void)
{
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	if(console_assigned){
		early_console_setup(UART1_BASE_ADDR, clk_get_sys("imx-uart.0", NULL));
	}
}

static struct sys_timer mx6_tdh_timer = {
	.init   = mx6_tdh_timer_init,
};



static void __init mx6q_tdh_reserve(void)
{
	phys_addr_t phys;
	int i, fb0_reserved = 0, fb_array_size;

	/*
	 * Reserve primary framebuffer memory if its base address
	 * is set by kernel command line.
	 */
	fb_array_size = ARRAY_SIZE(tdh_fb_data);
	if (fb_array_size > 0 && tdh_fb_data[0].res_base[0] &&
	    tdh_fb_data[0].res_size[0]) {
		if (tdh_fb_data[0].res_base[0] > SZ_2G)
			printk(KERN_INFO"UI Performance downgrade with FB phys address %x!\n",
			    tdh_fb_data[0].res_base[0]);
		memblock_reserve(tdh_fb_data[0].res_base[0],
				 tdh_fb_data[0].res_size[0]);
		memblock_remove(tdh_fb_data[0].res_base[0],
				tdh_fb_data[0].res_size[0]);
		tdh_fb_data[0].late_init = true;
		ipu_data[ldb_data.ipu_id].bypass_reset = true;
		fb0_reserved = 1;
	}
	for (i = fb0_reserved; i < fb_array_size; i++)
		if (tdh_fb_data[i].res_size[0]) {
			/* Reserve for other background buffer. */
			phys = memblock_alloc_base(tdh_fb_data[i].res_size[0],
						SZ_4K, SZ_2G);
			memblock_remove(phys, tdh_fb_data[i].res_size[0]);
			tdh_fb_data[i].res_base[0] = phys;
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

	if (vout_mem.res_msize) {
		phys = memblock_alloc_base(vout_mem.res_msize,
					   SZ_4K, SZ_1G);
		memblock_remove(phys, vout_mem.res_msize);
		vout_mem.res_mbase = phys;
	}
	if (capture_data[0].reserved_mem_size) {
		phys = memblock_alloc_base(capture_data[0].reserved_mem_size,
					   SZ_4K, SZ_1G);
		memblock_remove(phys, capture_data[0].reserved_mem_size);
		capture_data[0].reserved_mem_base = phys;
	}

}

/*
 * initialize __mach_desc_MX6Q_TDH data structure.
 */
MACHINE_START(MX6Q_TDH, "Freescale i.MX 6Quad/DualLite/Solo TDH")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = mx6_tdh_board_init,
	.timer = &mx6_tdh_timer,
	.reserve = mx6q_tdh_reserve,
MACHINE_END
