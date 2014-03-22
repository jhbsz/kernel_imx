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
#include <linux/mmc/host.h>
#include <linux/wakelock.h>


#include "generic_devices.h"
#include <linux/pwm.h>

#define BOARD_QPAD_REVA 0x1
#define BOARD_QPAD_REVB 0x2
#define BOARD_QPAD_REVC 0x3


typedef struct peripheral_power_state{
	unsigned int barcode:1;
	unsigned int smartcard:1;
	unsigned int wlan:1;
}PERIPHERAL_POWER_STATE_T;

#define QPAD_PFUZE_INT		IMX_GPIO_NR(7, 13)


#define QPAD_CHARGE_DOK_B	IMX_GPIO_NR(2, 24)
#define QPAD_CHARGE_UOK_B	IMX_GPIO_NR(3, 17)
#define QPAD_CHARGE_CHG_1_B	IMX_GPIO_NR(3, 23)
#define QPAD_CHARGE_FLT_1_B	IMX_GPIO_NR(5, 2)
#define QPAD_BATTERY_ALERT	IMX_GPIO_NR(4, 14)
#define QPAD_BATTERY_DET	IMX_GPIO_NR(4, 11)

#define QPAD_CSI0_PWDN		IMX_GPIO_NR(1, 16)
#define QPAD_CSI0_RST		IMX_GPIO_NR(5, 20)
#define QPAD_CSI0_POWER		IMX_GPIO_NR(2, 7)


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

#define QPAD_MODEM_ONOFF		IMX_GPIO_NR(2, 2)
#define QPAD_MODEM_RST			IMX_GPIO_NR(2, 3)
#define QPAD_MODEM_PWR			IMX_GPIO_NR(2, 4)
#define QPAD_MODEM_WAKEMODEM	IMX_GPIO_NR(2, 5)
#define QPAD_MODEM_WAKEAP		IMX_GPIO_NR(1, 18)

#define QPAD_AUDIO_RST					IMX_GPIO_NR(7, 11)
#define QPAD_AUDIO_INT					IMX_GPIO_NR(1, 20)
#define QPAD_AUDIO_HEADPHONE_DET		IMX_GPIO_NR(3, 21)

//QR Engine
#define QPAD_QRE_PWR		IMX_GPIO_NR(1, 7)
#define QPAD_QRE_RST		IMX_GPIO_NR(6, 31)
#define QPAD_QRE_TRIG		IMX_GPIO_NR(3, 30)
#define QPAD_QRE_WAKE		IMX_GPIO_NR(6, 31)
#define QPAD_QRE_STATE		IMX_GPIO_NR(2, 25)

//FLASHLIGHT
#define QPAD_FL_PWR_EN		IMX_GPIO_NR(4, 5)
#define QPAD_FL_EN			IMX_GPIO_NR(3, 31)

//Touch Panel
#define QPAD_TP_PWR_EN		IMX_GPIO_NR(2, 28)
#define QPAD_TP_RST			IMX_GPIO_NR(6, 8)
#define QPAD_TP_IRQ			IMX_GPIO_NR(6, 7)

//Sensor
#define QPAD_SENSOR_RST		IMX_GPIO_NR(2, 23)

//WiFi
#define QPAD_WIFI_RST		IMX_GPIO_NR(7, 8)
#define QPAD_WIFI_PDN		IMX_GPIO_NR(6, 11)
#define QPAD_WIFI_WAKEUP	IMX_GPIO_NR(7, 5)

//SDHC
#define QPAD_SD2_CD			IMX_GPIO_NR(2, 0)

//SmartCard Reader
#define QPAD_SMARTCARD_PWR_EN IMX_GPIO_NR(7,12)


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
static const struct esdhc_platform_data qpad_sd2_data __initconst = {
	.cd_gpio = QPAD_SD2_CD,
	.keep_power_at_suspend = 1,
	.cd_type = ESDHC_CD_CONTROLLER,
	.runtime_pm = 1,
};

/*WIFI SDIO*/
static struct sdhci_host* wlan_sdhc;
static int wlan_wakeup_init=0;
struct wake_lock wlan_wakelock;
static iomux_v3_cfg_t wlan_wakeup_pads_func[] = {
	MX6Q_PAD_SD3_DAT1__USDHC3_DAT1_50MHZ,
};
static iomux_v3_cfg_t wlan_wakeup_pads_io[] = {
	MX6Q_PAD_SD3_DAT1__GPIO_7_5,
};

static irqreturn_t wlan_wakup_handler(int irq, void *data){
	   printk("wlan wakeup\n");
	   wake_lock_timeout(&wlan_wakelock, HZ * 5);
       return IRQ_HANDLED;
}
static int wlan_wakeup_add(void){
       int ret;
	   if(!wlan_wakeup_init){
	       gpio_request(QPAD_WIFI_WAKEUP,"wifi-wakeup");
	       gpio_direction_input(QPAD_WIFI_WAKEUP);
		   wake_lock_init(&wlan_wakelock , WAKE_LOCK_SUSPEND, "wlan wakelock");
		   wlan_wakeup_init++;
	   }
	   //switch to gpio mode
	   mxc_iomux_v3_setup_multiple_pads(wlan_wakeup_pads_io,1);
       ret = request_any_context_irq(gpio_to_irq(QPAD_WIFI_WAKEUP), wlan_wakup_handler,
               IRQF_NO_SUSPEND|IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING ,
               "wlan wakeap", 0);
       if (ret) {
               pr_warning("Request wlan wakeup failed %d\n", ret);
       }else {
               enable_irq_wake(gpio_to_irq(QPAD_WIFI_WAKEUP));
       }
       printk("%s\n",__func__);
       return ret;
}
static int wlan_wakeup_remove(void){
       printk("%s\n",__func__);
       disable_irq_wake(gpio_to_irq(QPAD_WIFI_WAKEUP));
       free_irq(gpio_to_irq(QPAD_WIFI_WAKEUP),0);
	   mxc_iomux_v3_setup_multiple_pads(wlan_wakeup_pads_func,1);
       return 0;
}

static int wlan_host_init_cb(struct sdhci_host* host){
	wlan_sdhc = host;
	//enable_irq_wake(host->irq);
	return 0;
}
static const struct esdhc_platform_data qpad_sd3_data __initconst= {
	.keep_power_at_suspend = 1,
	.cd_type = ESDHC_CD_NONE,
	.sdhc_host_init_cb = wlan_host_init_cb,
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

static void qpad_uart_io_switch(int uart_idx/*0 based*/,int switch2uart)
{
	iomux_v3_cfg_t uart_gpio_pads[] = {
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
	iomux_v3_cfg_t uart_func_pads[] = {
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
	iomux_v3_cfg_t* io_pads_cfg = switch2uart?&uart_func_pads[uart_idx*2]:&uart_gpio_pads[uart_idx*2];
	mxc_iomux_v3_setup_multiple_pads(io_pads_cfg,2/*fixed 2 pins*/);
}



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
static struct imxuart_platform_data mx6_uart4_pdata __initdata = {
       .flags      = IMXUART_MODULATION|IMXUART_CON_DISABLE,
       .modulation_enable = uart_modulation_enable,
       .transceiver_delay = 5000,

};
static struct imxuart_platform_data mx6_uart_pdata __initdata = {
       .flags      = IMXUART_CON_DISABLE,
};

static inline void mx6q_qpad_init_uart(void)
{
	if(console_assigned)
		mx6_uart_pdata.flags&=~IMXUART_CON_DISABLE;
	imx6q_add_imx_uart(0, &mx6_uart_pdata);	
	imx6q_add_imx_uart(1, &mx6_uart_pdata);
	imx6q_add_imx_uart(2, &mx6_uart_pdata);
	imx6q_add_imx_uart(3, &mx6_uart4_pdata);
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
		gpio_set_value(QPAD_CSI0_PWDN, 1);
	}else{
		gpio_set_value(QPAD_CSI0_PWDN, 0);
	}

	msleep(2);
}

static void mx6q_csi0_io_init(void)
{	
	int cam_pdn = QPAD_CSI0_PWDN;
	int cam_rst = QPAD_CSI0_RST;
	int cam_pwr_en = QPAD_CSI0_POWER;	
	struct clk *clko2 = clk_get(NULL, "clko2_clk");
	struct clk *clko = clk_get(NULL, "clko_clk");
	if(!IS_ERR(clko)&&!IS_ERR(clko2)){
		//share the same clk of clko and clko2
		clk_set_parent(clko, clko2);
		clk_put(clko);
		clk_put(clko2);
	}

	if (cpu_is_mx6q())
		mxc_iomux_v3_setup_multiple_pads(mx6q_qpad_csi0_sensor_pads,
			ARRAY_SIZE(mx6q_qpad_csi0_sensor_pads));
	else if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_qpad_csi0_sensor_pads,
			ARRAY_SIZE(mx6dl_qpad_csi0_sensor_pads));

	camera_regulator = regulator_get(NULL,"VGEN3_2V8");
	if(IS_ERR(camera_regulator)){
		printk("failed to get regulator[VGEN3_2V8]\n");
		camera_regulator = NULL;
	}else {
		regulator_enable(camera_regulator);
	}

	if(BOARD_QPAD_REVA==mx6_board_rev()){
		iomux_v3_cfg_t csi0_power = MX6Q_PAD_NANDF_D7__GPIO_2_7;/*camera 2.8v and 1.8v enable*/
		mxc_iomux_v3_setup_pad(csi0_power);
		if (gpio_request(cam_pwr_en, "cam-pen")) {
			printk(KERN_ERR "Request GPIO failed,"
					"gpio: %d \n", cam_pwr_en);
		}else {		
			gpio_direction_output(cam_pwr_en, 0);			
		}
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


static int fl_pwren=QPAD_FL_PWR_EN;
static int fl_en=QPAD_FL_EN;
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
	ret = gpio_request(QPAD_TP_PWR_EN, "tp-pwr");
	if (ret) {
		pr_err("failed to get GPIO tp-pwr: %d\n",
			ret);
		goto err;
	}
	gpio_direction_output(QPAD_TP_PWR_EN, on?1:0);
	gpio_free(QPAD_TP_PWR_EN);

	ret = gpio_request(QPAD_TP_RST, "tp-rst");
	if (ret) {
		pr_err("failed to get GPIO tp-rst: %d\n",
			ret);
		goto err;
	}
	
	if(1==on){
		gpio_direction_output(QPAD_TP_RST, 1);
		msleep(50);
	}else {
		gpio_direction_output(QPAD_TP_RST, 0);
	}
	
	gpio_free(QPAD_TP_RST);
err:
	return ret;
}
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
	msleep(50);
	
	gpio_free(QPAD_TP_RST);
err:
	return ret;
}

static struct ft5x0x_ts_platform_data ft5x0x_data=
{
	.setpower	= ft5x0x_set_power,
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
		I2C_BOARD_INFO("mt9p111", 0x3c),
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


static void qpad_usbotg_vbus_v1(bool on){
	if(on){}
}

static void qpad_usbotg_vbus(bool on)
{
	int otg_pwren = QPAD_USB_OTG_PWR;
	if (on)
		gpio_set_value(otg_pwren, 1);
	else
		gpio_set_value(otg_pwren, 0);

}

static void qpad_host1_vbus(bool on)
{
	//dummy function since we don't have any usb hub
}

static void __init imx6q_qpad_init_usb(void)
{
	int ret = 0;

	if(BOARD_QPAD_REVA<mx6_board_rev()){
		mxc_iomux_v3_setup_pad(MX6Q_PAD_KEY_COL4__USBOH3_USBOTG_OC);
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
	}	

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	/*
	 *
	 *OTG ID daisy chain set
	 * 0: ENET_RX_ER as USB_OTG_ID 
	 * 1: GPIO_1 as USB_OTG_ID
	 */
	mxc_iomux_set_gpr_register(1, 13, 1, 1);
	if(BOARD_QPAD_REVA==mx6_board_rev())
		mx6_set_otghost_vbus_func(qpad_usbotg_vbus_v1);
	else
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
	static int skip_first_reset=1;
	int ret;

	if(skip_first_reset){
		skip_first_reset=0;
		return;
	}
	//return if bootloader already enable LCD???	
	ret = gpio_request(QPAD_DISP_RST_B, "disp_reset");	
	if (ret) {
		pr_err("failed to request gpio: %d\n",QPAD_DISP_RST_B);
		return;
	}
	gpio_direction_output(QPAD_DISP_RST_B, 0);
	msleep(10);
	gpio_direction_output(QPAD_DISP_RST_B, 1);
	
	gpio_free(QPAD_DISP_RST_B);

	/*
	 * it needs to delay 120ms minimum for reset complete
	 */
	msleep(10);
}

static void mipi_dsi_power(int en){	
	int ret;
	ret = gpio_request(QPAD_DISP_PWR_EN, "disp_pwren");	
	if (ret) {
		pr_err("failed to request gpio: %d\n",QPAD_DISP_PWR_EN);
		return;
	}
	gpio_direction_output(QPAD_DISP_PWR_EN,en?0:1);
	gpio_free(QPAD_DISP_PWR_EN);
}
static void mipi_dsi_baclight_power(int en){
	int ret;
	ret = gpio_request(QPAD_DISP_BL_PWR_EN, "disp_bl_pwren");	
	if (ret) {
		pr_err("failed to request gpio: %d\n",QPAD_DISP_BL_PWR_EN);
		return;
	}
	gpio_direction_output(QPAD_DISP_BL_PWR_EN,en?1:0);
	gpio_free(QPAD_DISP_BL_PWR_EN);
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
	.dc_valid = false,
	.usb_valid = true,//true,
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
	if(BOARD_QPAD_REVA<mx6_board_rev()){
		mxc_iomux_v3_setup_pad(MX6Q_PAD_EIM_D16__GPIO_3_16);
		qbp.alert = IMX_GPIO_NR(3,16);
	}
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
	}
};

struct imx_vout_mem {
	resource_size_t res_mbase;
	resource_size_t res_msize;
};

static struct imx_vout_mem vout_mem __initdata = {
	.res_msize = SZ_128M,
};

static void qpad_suspend_prepare(void){
	if(pps.wlan) wlan_wakeup_add();
}
static void qpad_suspend_wake(void){
	if(pps.wlan) wlan_wakeup_remove();
}
static void qpad_suspend_enter(void)
{
	if(pps.barcode)	qpad_uart_io_switch(1,0);
	if(pps.smartcard) qpad_uart_io_switch(4,0);
}

static void qpad_suspend_exit(void)
{
	if(pps.barcode)	qpad_uart_io_switch(1,1);
	if(pps.smartcard) qpad_uart_io_switch(4,1);
}
static const struct pm_platform_data qpad_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_prepare = qpad_suspend_prepare,
	.suspend_wake = qpad_suspend_wake,
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


//
//mode: 0->default headphone ,1->uart mode
//
#define UART_SWITCH_SELECT_IN1 IMX_GPIO_NR(6,14) /*Default low->hp,high->uart*/
#define UART_SWITCH_SELECT_IN2 IMX_GPIO_NR(3,19) /*Default High->hp,low->uart ,power control ,be careful*/

static int uart_switch_granted=0;
static int headphone_switch_to_mode(int mode){
	if(!mode){
		gpio_set_value(UART_SWITCH_SELECT_IN2,1);		
		gpio_set_value(UART_SWITCH_SELECT_IN1,0);
	}else {
		if(!uart_switch_granted){
			printk(KERN_WARNING "uart switcher not granted\n");
		}else {
			gpio_set_value(UART_SWITCH_SELECT_IN1,1);
			gpio_set_value(UART_SWITCH_SELECT_IN2,0);				
		}
	}
	return mode;
}
static int headphone_switch(int state){
	//switch state changed,	
	/*
	 *	state meaning
	 *	0: no headset plug in
	 *	1: headset with microphone plugged
	 *	2: headset without microphone plugged
	*/
	if(!state){
		//can't switch hardware to uart
		headphone_switch_to_mode(0);
		uart_switch_granted=0;
	}else if(state){
		headphone_switch_to_mode(0);
		uart_switch_granted++;
		printk(KERN_WARNING "FIXME:hp state->1 or state->2\n");		
	}
	return 0;
}

static int read_uartswitcher(char *page, char **start,
			     off_t off, int count,
			     int *eof, void *data)
{
	return sprintf(page, "uart_switch_granted : %s\r\n"
						 "echo 1 > uartswitcher -->switch to uart mode\r\n"
						 "echo 0 > uartswitcher -->switch to hp mode\r\n"
						 "\n",
                      uart_switch_granted?"yes":"no");
}

static int write_uartswitcher (struct file *file, const char *buffer,
                      unsigned long count, void *data) 
{
	char kbuf[256];
	int action;

	if (count >= 256)
		return -EINVAL;
	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;

	action = (int)simple_strtoul(kbuf, NULL, 10);
	headphone_switch_to_mode(action);
	
    return count;
}


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

	//uart switcher support on v2
	if(BOARD_QPAD_REVA<mx6_board_rev()){
		mxc_iomux_v3_setup_multiple_pads(mx6q_qpad_hp_uart_switcher_pads_v2,ARRAY_SIZE(mx6q_qpad_hp_uart_switcher_pads_v2));
		rt5625_data.headphone_switch=headphone_switch;

		
		ret = gpio_request(UART_SWITCH_SELECT_IN1, "UART_SWITCH_SELECT_IN1");
		if (ret) {
			pr_err("failed to get GPIO UART_SWITCH_SELECT_IN2: %d\n",
				ret);
			return -EINVAL;
		}
		gpio_direction_output(UART_SWITCH_SELECT_IN1, 0);
		ret = gpio_request(UART_SWITCH_SELECT_IN2, "UART_SWITCH_SELECT_IN2");
		if (ret) {
			pr_err("failed to get GPIO UART_SWITCH_SELECT_IN2: %d\n",
				ret);
			return -EINVAL;
		}
		gpio_direction_output(UART_SWITCH_SELECT_IN2, 1);


		//init proc interface 
		{
			struct proc_dir_entry *entry;			
			entry = create_proc_entry("driver/uartswitcher", S_IFREG | S_IRUGO | S_IWUGO/*default mode*/, NULL);
			if (entry) {
				entry->read_proc = read_uartswitcher;
				entry->write_proc = write_uartswitcher;
			}
		}
		
	}

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

static struct gpio_keys_button qpad_buttons_v2[] = {
	GPIO_BUTTON(IMX_GPIO_NR(1,2), KEY_MENU, 1, "menu", 1, 1),
	GPIO_BUTTON(IMX_GPIO_NR(1,4), KEY_HOME, 1, "home", 1, 1),
	GPIO_BUTTON(IMX_GPIO_NR(1,5), KEY_BACK, 1, "back", 1, 1),	
	GPIO_BUTTON(GPIO_KEY_F1, KEY_F1, 1, "F1", 1, 1),
	GPIO_BUTTON(GPIO_KEY_F2, KEY_F2, 1, "F2", 1, 1),
	GPIO_BUTTON(GPIO_KEY_POWER, KEY_POWER, 1, "power", 1, 1),	
};

static struct gpio_keys_platform_data qpad_button_v2_data = {
	.buttons	= qpad_buttons_v2,
	.nbuttons	= ARRAY_SIZE(qpad_buttons_v2),
};


static struct platform_device qpad_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources  = 0,
};

static void __init imx6q_add_device_buttons(void)
{
	if(BOARD_QPAD_REVA<mx6_board_rev()){		
		iomux_v3_cfg_t mx6q_qpad_keypad[] = {
			NEW_PAD_CTRL(MX6Q_PAD_GPIO_2__GPIO_1_2,MX6Q_GENERIC_PAD_CTRL),
			NEW_PAD_CTRL(MX6Q_PAD_GPIO_4__GPIO_1_4,MX6Q_GENERIC_PAD_CTRL),
			NEW_PAD_CTRL(MX6Q_PAD_GPIO_5__GPIO_1_5,MX6Q_GENERIC_PAD_CTRL),
		};
		mxc_iomux_v3_setup_multiple_pads(mx6q_qpad_keypad,ARRAY_SIZE(mx6q_qpad_keypad));
		platform_device_add_data(&qpad_button_device,
				&qpad_button_v2_data,
				sizeof(qpad_button_v2_data));		
	}else {
		iomux_v3_cfg_t mx6q_qpad_keypad[] = {
			NEW_PAD_CTRL(MX6Q_PAD_EIM_DA9__GPIO_3_9,MX6Q_KEYPAD_PAD_CTRL),	
			NEW_PAD_CTRL(MX6Q_PAD_EIM_DA10__GPIO_3_10,MX6Q_KEYPAD_PAD_CTRL),
			NEW_PAD_CTRL(MX6Q_PAD_EIM_DA11__GPIO_3_11,MX6Q_KEYPAD_PAD_CTRL),
		};
		mxc_iomux_v3_setup_multiple_pads(mx6q_qpad_keypad,ARRAY_SIZE(mx6q_qpad_keypad));
		platform_device_add_data(&qpad_button_device,
				&qpad_button_data,
				sizeof(qpad_button_data));
	}

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
			int ret = gpio_request(QPAD_WIFI_PDN, "wifi-pdn");		
			if (ret) {
				pr_err("failed to get GPIO wifi-pdn: %d\n",
					ret);
				return -EINVAL;
			}
			gpio_direction_output(QPAD_WIFI_PDN,1);
			gpio_free(QPAD_WIFI_PDN);
			msleep(10);
			ret = gpio_request(QPAD_WIFI_RST, "wifi-rst");
			if (ret) {
				pr_err("failed to get GPIO wifi-rst: %d\n",
					ret);
				return -EINVAL;
			}
			gpio_direction_output(QPAD_WIFI_RST,1);
			gpio_free(QPAD_WIFI_RST);
			pps.wlan = 1;
			msleep(200);
		}
		else {
			//always put wifi chip into reset state to save power???
			int ret = gpio_request(QPAD_WIFI_PDN, "wifi-pdn");		
			if (ret) {
				pr_err("failed to get GPIO wifi-pdn: %d\n",
					ret);
				return -EINVAL;
			}
			gpio_direction_output(QPAD_WIFI_PDN,0);
			gpio_free(QPAD_WIFI_PDN);

			ret = gpio_request(QPAD_WIFI_RST, "wifi-rst");
			if (ret) {
				pr_err("failed to get GPIO wifi-rst: %d\n",
					ret);
				return -EINVAL;
			}
			gpio_direction_output(QPAD_WIFI_RST,0);
			gpio_free(QPAD_WIFI_RST);
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



#define MODEM_PIN_POWER QPAD_MODEM_PWR
#define MODEM_PIN_RESET QPAD_MODEM_RST
#define MODEM_PIN_ONOFF QPAD_MODEM_ONOFF
#define MODEM_PIN_WAKEAP QPAD_MODEM_WAKEAP
#define MODEM_PIN_WAKEMODEM QPAD_MODEM_WAKEMODEM
#define MODEM_INIT_EXCLUDE 1
#include "modem.c"
static int __init modem_init(void){
	//dummy init for modem
	board_modem_init();
	return 0;
}



static void smartcard_reader_power(int on){
	gpio_set_value(QPAD_SMARTCARD_PWR_EN,on?1:0);
	pps.smartcard=on?1:0;
	qpad_uart_io_switch(4,on);
}
static int read_smartcard(char *page, char **start,
			     off_t off, int count,
			     int *eof, void *data)
{
	return sprintf(page, "echo 1 > smartcard -->enable smartcard reader power\r\n"
						 "echo 0 > smartcard -->disable smartcard read power\r\n"
						 "\n");
}

static int write_smartcard(struct file *file, const char *buffer,
                      unsigned long count, void *data) 
{
	char kbuf[256];
	int action;

	if (count >= 256)
		return -EINVAL;
	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;

	action = (int)simple_strtoul(kbuf, NULL, 10);
	smartcard_reader_power(action);
	
    return count;
}

static int read_barcode(char *page, char **start,
			     off_t off, int count,
			     int *eof, void *data)
{
	return sprintf(page, "echo 'power [On|off]' > barcode -->enable/disable barcode engine power\r\n"
						 "echo 'trig [on|off|Once]' > barcode -->trigger hardware pin\r\n"
 						 "echo 'wake [on|off|Once]' > barcode -->wake up module\r\n"
 						 "command without sub command is default sub command with capital prefix\r\n"
						 "cat barcode -->display current barcode engine status\r\n"
						 "\n"
						 "power[%s]wake[%s]trig[%s]state[%s]\r\n"
						 "\n",
						 gpio_get_value(QPAD_QRE_PWR)?"high":"low",
						 gpio_get_value(QPAD_QRE_WAKE)?"high":"low",
						 gpio_get_value(QPAD_QRE_TRIG)?"high":"low",
						 gpio_get_value(QPAD_QRE_STATE)?"high":"low");
}

static int write_barcode(struct file *file, const char *buffer,
                      unsigned long count, void *data) 
{
	char kbuf[256];
	char func[48]={0};
	char state[48]={0};

	if (count >= 256)
		return -EINVAL;
	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;


    sscanf(kbuf,"%s %s",func,state);
	if(func[0]){
		if(!strcmp(func,"power")){
			int on=1;
			if(!strcmp(state,"off"))
				on=0;			
			gpio_set_value(QPAD_QRE_PWR,on?0:1);		
			pps.barcode = on?1:0;
			qpad_uart_io_switch(1,on?1:0);
		}else if(!strcmp(func,"trig")){
				if(!strcmp(state,"on")){
					gpio_set_value(QPAD_QRE_TRIG,1);
				}else if(!strcmp(state,"off")){
					gpio_set_value(QPAD_QRE_TRIG,0);
				}else {
					gpio_set_value(QPAD_QRE_TRIG,1);
					msleep(10);
					gpio_set_value(QPAD_QRE_TRIG,0);
					msleep(10);
				}
		}else if(!strcmp(func,"wake")){
				if(!strcmp(state,"on")){
					gpio_set_value(QPAD_QRE_WAKE,1);
				}else if(!strcmp(state,"off")){
					gpio_set_value(QPAD_QRE_WAKE,0);
				}else {
					gpio_set_value(QPAD_QRE_WAKE,1);
					msleep(10);
					gpio_set_value(QPAD_QRE_WAKE,0);
				}
		}
	}

	
    return count;
}


static int read_board_revision(char *page, char **start,
			     off_t off, int count,
			     int *eof, void *data)
{
	return sprintf(page, "v%d",mx6_board_rev());
}


static int __init board_misc_init(void){
	int ret;	
	struct proc_dir_entry *entry;
	if(BOARD_QPAD_REVA==mx6_board_rev()){
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
	}else if(BOARD_QPAD_REVA<mx6_board_rev()){
		int qr_trig = QPAD_QRE_TRIG;
		int qr_wake = QPAD_QRE_WAKE;
		int qr_state = QPAD_QRE_STATE;
		int qr_pwr_en = QPAD_QRE_PWR;
		mxc_iomux_v3_setup_multiple_pads(mx6q_qpad_barcode_pads_v2,ARRAY_SIZE(mx6q_qpad_barcode_pads_v2));
		ret = gpio_request(qr_pwr_en, "BarcodePwr");
		if (ret) {
			pr_err("failed to get GPIO BarcodePwr %d\n",
				ret);
			return -EINVAL;
		}
		gpio_direction_output(qr_pwr_en,1);
		//default state is power off ,to fix current leak issue,switch uart to io mode
		qpad_uart_io_switch(1,0);
		ret = gpio_request(qr_wake, "BarcodeWake");
		if (ret) {
			pr_err("failed to get GPIO BarcodeWake %d\n",
				ret);
			return -EINVAL;
		}
		gpio_direction_output(qr_wake,0);
		
		ret = gpio_request(qr_trig, "BarcodeTrig");
		if (ret) {
			pr_err("failed to get GPIO BarcodeTrig %d\n",
				ret);
			return -EINVAL;
		}
		gpio_direction_output(qr_trig,1);
		
		ret = gpio_request(qr_state, "BarcodeState");
		if (ret) {
			pr_err("failed to get GPIO BarcodeState %d\n",
				ret);
			return -EINVAL;
		}
		gpio_direction_input(qr_state);
		
		entry = create_proc_entry("driver/barcode", S_IFREG | S_IRUGO | S_IWUGO, NULL);
		if (entry) {
			entry->read_proc = read_barcode;
			entry->write_proc = write_barcode;
		}

	}

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


	if(BOARD_QPAD_REVA==mx6_board_rev()){
		fl_pwren = IMX_GPIO_NR(3,31);
		fl_en = IMX_GPIO_NR(3,22);
	}else {
		fl_pwren = IMX_GPIO_NR(3,31);
		fl_en = IMX_GPIO_NR(4,5);
		
	}
	eup2471_enable(0);
	eup2471_flash(0);

	//for v2,smartcard reader power enable support
	if(BOARD_QPAD_REVA<mx6_board_rev()){
		mxc_iomux_v3_setup_pad(MX6Q_PAD_GPIO_17__GPIO_7_12);
		
		ret = gpio_request(QPAD_SMARTCARD_PWR_EN, "SmartCardPwr");
		if (ret) {
			pr_err("failed to get GPIO SmartCardPwr %d\n",
				ret);
			return -EINVAL;
		}
		gpio_direction_output(QPAD_SMARTCARD_PWR_EN,0);		
		//default state is power off ,to fix current leak issue,switch uart to io mode
		//qpad_uart_io_switch(4,0);
		entry = create_proc_entry("driver/smartcard", S_IFREG | S_IRUGO | S_IWUGO, NULL);
		if (entry) {
			entry->read_proc = read_smartcard;
			entry->write_proc = write_smartcard;
		}
		
	}
	
	entry = create_proc_entry("boardrev", S_IFREG | S_IRUGO | S_IWUSR, NULL);
	if (entry) {
		entry->read_proc = read_board_revision;
	}
	
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
	struct platform_device *voutdev;

	if (cpu_is_mx6q())
		mxc_iomux_v3_setup_multiple_pads(mx6q_qpad_pads,
			ARRAY_SIZE(mx6q_qpad_pads));
	else if (cpu_is_mx6dl()) {
		mxc_iomux_v3_setup_multiple_pads(mx6dl_qpad_pads,
			ARRAY_SIZE(mx6dl_qpad_pads));
	}
	printk("QPAD board id=0x%x rev=0x%x\n",mx6_board_id(),mx6_board_rev());
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
	voutdev = imx6q_add_v4l2_output(0);
	if (vout_mem.res_msize && voutdev) {
		dma_declare_coherent_memory(&voutdev->dev,
					    vout_mem.res_mbase,
					    vout_mem.res_mbase,
					    vout_mem.res_msize,
					    (DMA_MEMORY_MAP |
					     DMA_MEMORY_EXCLUSIVE));
	}
	imx6q_add_v4l2_capture(0, &capture_data[0]);
	imx6q_add_imx_snvs_rtc();

	imx6q_add_imx_caam();

	imx6q_add_device_gpio_leds();

	imx6q_add_imx_i2c(0, &mx6q_i2c_data);
	imx6q_add_imx_i2c(1, &mx6q_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_i2c_data);

	
	if(eBootModeCharger!=android_bootmode){
		//patch for TP y axis inverted
		if(BOARD_QPAD_REVB==mx6_board_rev()){
			ft5x0x_data.y_inverted=1;
		}
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
		
		mxc_register_device(&wlan_bt_rfkill, &wlan_bt_rfkill_data);
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
	//For fast boot
	/*imx6q_add_mxc_pwm(2);
	imx6q_add_mxc_pwm(3);*/
	imx6q_add_mxc_pwm_backlight(0, &mx6_qpad_pwm_backlight_data);

	//For fast boot
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

	
	qpad_power_init();

	//enable armpmu if necessary
	if(enable_armpmu){
		imx6_add_armpmu();
		imx6q_add_perfmon(0);
		imx6q_add_perfmon(1);
		imx6q_add_perfmon(2);
		
	}
	if(eBootModeCharger!=android_bootmode){
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

	if(console_assigned){
		early_console_setup(UART1_BASE_ADDR, clk_get_sys("imx-uart.0", NULL));
	}
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

	if (vout_mem.res_msize) {
		phys = memblock_alloc_base(vout_mem.res_msize,
					   SZ_4K, SZ_1G);
		memblock_remove(phys, vout_mem.res_msize);
		vout_mem.res_mbase = phys;
	}
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
