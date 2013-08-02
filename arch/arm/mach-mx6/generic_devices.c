#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/w1-gpio.h>
#include <linux/pn544.h>
#include <mach/devices-common.h>
#include "generic_devices.h"


static struct w1_gpio_platform_data w1_gpio_pdata=
{
	.is_open_drain = 0,
};
struct platform_device *__init generic_add_device_w1(
		int w1_io)
{
	struct w1_gpio_platform_data *pdata = &w1_gpio_pdata;
	pdata->pin = w1_io;
	return imx_add_platform_device("w1-gpio", -1,
			NULL, 0, pdata, sizeof(*pdata));
}


static struct pn544_i2c_platform_data pn544_pdata = {
	.irq_gpio = 0,//NFC_IRQ,
	.ven_gpio = 0,//NFC_EN,
	.firm_gpio = 0,//NFC_FIRM,
};


static struct i2c_board_info pn544_device = {
	I2C_BOARD_INFO("pn544", 0x28),
	.platform_data = &pn544_pdata,
};
int __init generic_add_device_pn544(
		int bus,int irq,int ven,int fw)
{
	pn544_pdata.irq_gpio = irq;
	pn544_pdata.ven_gpio = ven;
	pn544_pdata.firm_gpio = fw;
	pn544_device.irq = gpio_to_irq(irq);

	return i2c_register_board_info(bus, &pn544_device,1);
}


