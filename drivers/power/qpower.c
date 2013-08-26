/*
 * Copyright 2013 Quester Technology,Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/power/qpower.h>

struct qpower_data {
	struct qpower_pdata *pdata;
	struct qpower_ops* ops;

};


static struct i2c_board_info qpower_battery_device = {
	I2C_BOARD_INFO("max17058", 0x36),
};



static struct platform_device qpower_charger_device = {
	.name = "qpower-charger",
};

static int register_qpower_charger(void *pdata)
{
	int ret;

	qpower_charger_device.dev.platform_data = pdata;

	ret = platform_device_register(&qpower_charger_device);
	if (ret)
		pr_debug("Unable to register platform device '%s': %d\n",
			 qpower_charger_device.name, ret);

	return ret;
}

static int register_qpower_battery(void *pdata)
{
	struct qpower_battery_pdata* bp = pdata;
	int ret;

	qpower_battery_device.platform_data = pdata;

	ret = i2c_register_board_info(bp->busid, &qpower_battery_device, 1);
	if (ret)
		pr_debug("Unable to register qpower batter '%s': %d\n",
			 qpower_battery_device.type, ret);

	return ret;
}



static __devinit int qpower_probe(struct platform_device *pdev)
{
	struct qpower_pdata *pdata = pdev->dev.platform_data;
	struct qpower_data *data;
	struct device *dev = &pdev->dev;
	int ret = 0;
	data = kzalloc(sizeof(struct qpower_data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(dev, "Cannot allocate memory.\n");
		return -ENOMEM;
	}
	data->pdata = pdata;
	platform_set_drvdata(pdev, data);

	if(pdata->flags&QPOWER_FEATURE_CHARGER){
		if(pdata->cp){
			if(pdata->cp->ops)
				data->ops = pdata->cp->ops;
		}
	}
	if(pdata->flags&QPOWER_FEATURE_BATTERY){
		if(pdata->bp){
			if(pdata->bp->ops&&!data->ops)
				data->ops = pdata->bp->ops;
		}
	}
	if(!data->ops){
		data->ops = kzalloc(sizeof(struct qpower_ops), GFP_KERNEL);
	}
	if(pdata->flags&QPOWER_FEATURE_CHARGER){
		if(!pdata->cp->ops)
			pdata->cp->ops = data->ops;
		ret = register_qpower_charger(pdata->cp);
	}
	if(pdata->flags&QPOWER_FEATURE_BATTERY){
		if(!pdata->bp->ops)
			pdata->bp->ops = data->ops;
		//register platform data device
		ret = register_qpower_battery(pdata->bp);
	}



	return ret;
}

static __devexit int qpower_remove(struct platform_device *pdev)
{
	struct qpower_data *data = platform_get_drvdata(pdev);
	if (data) {
		kfree(data);
	}
	return 0;
}

static int qpower_suspend(struct platform_device *pdev,
				  pm_message_t state)
{
	return 0;
}

static int qpower_resume(struct platform_device *pdev)
{

	return 0;

}

static struct platform_driver qpower_driver = {
	.probe	= qpower_probe,
	.remove	= __devexit_p(qpower_remove),
	.suspend = qpower_suspend,
	.resume = qpower_resume,
	.driver = {
		.name	= "qpower",
		.owner	= THIS_MODULE,
	},
};

static int __init qpower_init(void)
{
	return platform_driver_register(&qpower_driver);
}
module_init(qpower_init);

static void __exit qpower_exit(void)
{
	platform_driver_unregister(&qpower_driver);
}
module_exit(qpower_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("raymond1860@gmail.com");
MODULE_DESCRIPTION("qpower supply driver");
MODULE_ALIAS("qpad_power");
