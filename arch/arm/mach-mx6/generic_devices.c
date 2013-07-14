#include <linux/w1-gpio.h>
#include "generic_devices.h"


static struct w1_gpio_platform_data w1_gpio_pdata=
{
	.is_open_drain = 1,
};
struct platform_device *__init generic_add_device_w1(
		int w1_io)
{
	struct w1_gpio_platform_data *pdata = &w1_gpio_pdata;
	pdata->pin = w1_io;
	return imx_add_platform_device("w1-gpio", -1,
			NULL, 0, pdata, sizeof(*pdata));
}

