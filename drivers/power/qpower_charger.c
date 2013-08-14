/*
 * qpower_charger.c - Maxim 8903 based USB/Adapter Charger Driver
 *
 * Copyright (C) 2011 Samsung Electronics
 * MyungJoo Ham <myungjoo.ham@samsung.com>
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
#include <linux/power/qpower.h>

#define QPOWER_CHARGER_POLLING_DELAY		(50 * HZ)
struct qpower_charger_data {
	struct qpower_charger_pdata *pdata;
	struct device *dev;
	struct power_supply psy_dc;
	struct power_supply psy_usb;

	bool chg;
	bool fault;
	bool usb_in;
	bool dc_in;
	struct delayed_work work;
};

static enum power_supply_property psy_usb_props[] = {
	POWER_SUPPLY_PROP_STATUS, /* Charger status output */
	POWER_SUPPLY_PROP_HEALTH, /* Fault or OK */
};

static enum power_supply_property psy_dc_props[] = {
	POWER_SUPPLY_PROP_ONLINE, /* External power source */
};


//return battery is online or not
static int psy_battery_online(void* drvdata){
	struct qpower_charger_data *qcd = drvdata;
	return 1;
}

//return battery is faulty or not
static int psy_charger_faulty(void* drvdata){
	struct qpower_charger_data *qcd = drvdata;
	return (qcd->fault);
}

//return charger is online or not
static int psy_charger_online(void* drvdata){
	struct qpower_charger_data *qcd = drvdata;
	return (qcd->usb_in || qcd->dc_in);
}

//return charger is enabled
static int psy_charger_enable(void* drvdata){
	struct qpower_charger_data *qcd = drvdata;
	return qcd->chg;
}


static int psy_dc_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct qpower_charger_data *data = container_of(psy,
			struct qpower_charger_data, psy_dc);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 0;
		if (data->usb_in || data->dc_in)
			val->intval = 1;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int psy_usb_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct qpower_charger_data *data = container_of(psy,
			struct qpower_charger_data, psy_usb);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		if (data->pdata->chg) {
			if (gpio_get_value(data->pdata->chg) == 0)
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			else if (data->usb_in || data->dc_in) {
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			} else
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		}
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		if (data->fault)
			val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 0;
		if (data->usb_in || data->dc_in)
			val->intval = 1;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void qpower_charger_work(struct work_struct *work)
{
	struct qpower_charger_data *data = container_of(work,
			struct qpower_charger_data, work.work);
	//FIXME,polling work?
	schedule_delayed_work(&data->work, QPOWER_CHARGER_POLLING_DELAY);
}

static irqreturn_t qpower_charger_dcin(int irq, void *_data)
{
	struct qpower_charger_data *data = _data;
	struct qpower_charger_pdata *pdata = data->pdata;
	bool dc_in;

	dc_in = gpio_get_value(pdata->dok) ? false : true;

	if (dc_in == data->dc_in)
		return IRQ_HANDLED;

	data->dc_in = dc_in;

	/* Set Current-Limit-Mode 1:DC 0:USB */
	if (pdata->dcm)
		gpio_set_value(pdata->dcm, dc_in ? 1 : 0);

	/* Charger Enable / Disable (cen is negated) */
	if (pdata->cen)
		gpio_set_value(pdata->cen, dc_in ? 0 :
				(data->usb_in ? 0 : 1));

	dev_dbg(data->dev, "TA(DC-IN) Charger %s.\n", dc_in ?
			"Connected" : "Disconnected");

	power_supply_changed(&data->psy_dc);
	if(pdata->usb_valid)
		power_supply_changed(&data->psy_usb);

	return IRQ_HANDLED;
}

static irqreturn_t qpower_charger_usbin(int irq, void *_data)
{
	struct qpower_charger_data *data = _data;
	struct qpower_charger_pdata *pdata = data->pdata;
	bool usb_in;

	usb_in = gpio_get_value(pdata->uok) ? false : true;

	if (usb_in == data->usb_in)
		return IRQ_HANDLED;

	data->usb_in = usb_in;

	/* Do not touch Current-Limit-Mode */

	/* Charger Enable / Disable (cen is negated) */
	if (pdata->cen)
		gpio_set_value(pdata->cen, usb_in ? 0 :
				(data->dc_in ? 0 : 1));

	dev_dbg(data->dev, "USB Charger %s.\n", usb_in ?
			"Connected" : "Disconnected");

	if(pdata->dc_valid)
		power_supply_changed(&data->psy_dc);
	power_supply_changed(&data->psy_usb);

	return IRQ_HANDLED;
}

static irqreturn_t qpower_charger_fault(int irq, void *_data)
{
	struct qpower_charger_data *data = _data;
	struct qpower_charger_pdata *pdata = data->pdata;
	bool fault;

	fault = gpio_get_value(pdata->flt) ? false : true;

	if (fault == data->fault)
		return IRQ_HANDLED;

	data->fault = fault;

	if (fault)
		dev_err(data->dev, "Charger suffers a fault and stops.\n");
	else
		dev_err(data->dev, "Charger recovered from a fault.\n");

	return IRQ_HANDLED;
}

static irqreturn_t qpower_charger_status(int irq, void *_data)
{
	struct qpower_charger_data *data = _data;
	struct qpower_charger_pdata *pdata = data->pdata;
	bool chg;

	chg = gpio_get_value(pdata->chg) ? false : true;

	if (chg == data->chg)
		return IRQ_HANDLED;

	data->chg = chg;

	if (chg)
		dev_err(data->dev, "Charger charging\n");
	else
		dev_err(data->dev, "Charger charging stopped.\n");

	return IRQ_HANDLED;
}


static __devinit int qpower_charger_probe(struct platform_device *pdev)
{
	struct qpower_charger_data *data;
	struct device *dev = &pdev->dev;
	struct qpower_charger_pdata *pdata = pdev->dev.platform_data;
	int ret = 0;
	int gpio = 0;
	int dc_in = 0;
	int usb_in = 0;
	int error;

	data = kzalloc(sizeof(struct qpower_charger_data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(dev, "Cannot allocate memory.\n");
		return -ENOMEM;
	}
	data->pdata = pdata;
	data->dev = dev;
	platform_set_drvdata(pdev, data);

	if (pdata->dc_valid == false && pdata->usb_valid == false) {
		dev_err(dev, "No valid power sources.\n");
		ret = -EINVAL;
		goto err;
	}

	if (pdata->dc_valid) {
		if (pdata->dok && gpio_is_valid(pdata->dok) &&
				pdata->dcm && gpio_is_valid(pdata->dcm)) {
			gpio = pdata->dok; /* PULL_UPed Interrupt */
			dc_in = gpio_get_value(gpio) ? 0 : 1;

			gpio = pdata->dcm; /* Output */
			gpio_set_value(gpio, dc_in);
		} else if (pdata->dok && gpio_is_valid(pdata->dok) &&
			   pdata->dcm_always_high) {
			gpio = pdata->dok; /* PULL_UPed Interrupt */

			error = gpio_request(gpio, "chg_dc");
			if (error < 0) {
				dev_err(dev, "failed to configure"
					" request/direction for GPIO %d, error %d\n",
					gpio, error);
				goto err;
			}
			gpio_direction_input(gpio);

			dc_in = gpio_get_value(gpio) ? 0 : 1;

			if (dc_in)
				data->dc_in = true;
			else
				data->dc_in = false;
		} else {
			dev_err(dev, "When DC is wired, DOK and DCM should"
					" be wired as well."
					" or set dcm always high\n");
			ret = -EINVAL;
			goto err;
		}
	} else {
		if (pdata->dcm) {
			if (gpio_is_valid(pdata->dcm))
				gpio_set_value(pdata->dcm, 0);
			else {
				dev_err(dev, "Invalid pin: dcm.\n");
				ret = -EINVAL;
				goto err;
			}
		}
	}

	if (pdata->usb_valid) {
		if (pdata->uok && gpio_is_valid(pdata->uok)) {
			gpio = pdata->uok;
			error = gpio_request(gpio, "chg_usb");
			if (error < 0) {
				dev_err(dev, "failed to configure"
					" request/direction for GPIO %d, error %d\n",
					gpio, error);
				goto err;
			}

			gpio_direction_input(gpio);
			usb_in = gpio_get_value(gpio) ? 0 : 1;
			if (usb_in)
				data->usb_in = true;
			else
				data->usb_in = false;
		} else {
			dev_err(dev, "When USB is wired, UOK should be wired."
					"as well.\n");
			ret = -EINVAL;
			goto err;
		}
	}

	if (pdata->cen) {
		if (gpio_is_valid(pdata->cen)) {
			gpio_set_value(pdata->cen, (dc_in || usb_in) ? 0 : 1);
		} else {
			dev_err(dev, "Invalid pin: cen.\n");
			ret = -EINVAL;
			goto err;
		}
	}

	if (pdata->chg) {
		if (!gpio_is_valid(pdata->chg)) {
			dev_err(dev, "Invalid pin: chg.\n");
			ret = -EINVAL;
			goto err;
		}
		error = gpio_request(pdata->chg, "chg_status");
		if (error < 0) {
			dev_err(dev, "failed to configure"
				" request/direction for GPIO %d, error %d\n",
				pdata->chg, error);
			goto err;
		}
		error = gpio_direction_input(pdata->chg);
	}

	if (pdata->flt) {
		if (!gpio_is_valid(pdata->flt)) {
			dev_err(dev, "Invalid pin: flt.\n");
			ret = -EINVAL;
			goto err;
		}
		error = gpio_request(pdata->flt, "chg_fault");
		if (error < 0) {
			dev_err(dev, "failed to configure"
				" request/direction for GPIO %d, error %d\n",
				pdata->flt, error);
			goto err;
		}
		error = gpio_direction_input(pdata->flt);
	}

	if (pdata->usus) {
		if (!gpio_is_valid(pdata->usus)) {
			dev_err(dev, "Invalid pin: usus.\n");
			ret = -EINVAL;
			goto err;
		}
	}

	data->fault = false;
	data->dc_in = dc_in;
	data->usb_in = usb_in;

	if(pdata->dc_valid||pdata->feature_flag&QPOWER_CHARGER_FEATURE_SHORT_MODE){
		data->psy_dc.name = "qpower-dc";
		data->psy_dc.type = POWER_SUPPLY_TYPE_MAINS;
		data->psy_dc.get_property = psy_dc_get_property;
		data->psy_dc.properties = psy_dc_props;
		data->psy_dc.num_properties = ARRAY_SIZE(psy_dc_props);

		ret = power_supply_register(dev, &data->psy_dc);
		if (ret) {
			dev_err(dev, "failed: power supply register.\n");
			goto err;
		}
		ret = request_threaded_irq(gpio_to_irq(pdata->dok),
				NULL, qpower_charger_dcin,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				"qpower_charger DC IN", data);
		if (ret) {
			dev_err(dev, "Cannot request irq %d for DC (%d)\n",
					gpio_to_irq(pdata->dok), ret);
			goto err_psy;
		}

		
		power_supply_changed(&data->psy_dc);
	}


	if (pdata->usb_valid) {
		data->psy_usb.name = "qpower-usb";
		data->psy_usb.type = POWER_SUPPLY_TYPE_USB;
		data->psy_usb.get_property = psy_usb_get_property;
		data->psy_usb.properties = psy_usb_props;
		data->psy_usb.num_properties = ARRAY_SIZE(psy_usb_props);

		ret = power_supply_register(dev, &data->psy_usb);
		if (ret) {
			dev_err(dev, "failed: power supply register.\n");
			goto err_psy;
		}
		
		ret = request_threaded_irq(gpio_to_irq(pdata->uok),
				NULL, qpower_charger_usbin,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				"qpower_charger USB IN", data);
		if (ret) {
			dev_err(dev, "Cannot request irq %d for USB (%d)\n",
					gpio_to_irq(pdata->uok), ret);
			goto err_dc_irq;
		}

		
		power_supply_changed(&data->psy_usb);
	}

	if (pdata->flt) {
		ret = request_threaded_irq(gpio_to_irq(pdata->flt),
				NULL, qpower_charger_fault,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				"qpower_charger Fault", data);
		if (ret) {
			dev_err(dev, "Cannot request irq %d for Fault (%d)\n",
					gpio_to_irq(pdata->flt), ret);
			goto err_usb_irq;
		}
	}

	if (pdata->chg) {
		ret = request_threaded_irq(gpio_to_irq(pdata->chg),
				NULL, qpower_charger_status,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				"qpower_charger status", data);
		if (ret) {
			dev_err(dev, "Cannot request irq %d for status (%d)\n",
					gpio_to_irq(pdata->flt), ret);
			goto err_usb_irq;
		}
	}
	
	
	//enable our polling work? is this necessary?
	INIT_DELAYED_WORK_DEFERRABLE(&data->work, qpower_charger_work);
	//schedule_delayed_work(&data->work, QPOWER_CHARGER_POLLING_DELAY);

	if(pdata->ops){
		pdata->ops->drv_data = data;
		if(!pdata->ops->qco)
			pdata->ops->qco = psy_charger_online;
		if(!pdata->ops->qce)
			pdata->ops->qce = psy_charger_enable;
		if(!pdata->ops->qbo)
			pdata->ops->qbo = psy_battery_online;		
		if(!pdata->ops->qbf)
			pdata->ops->qbf = psy_charger_faulty;		
	}

	return 0;

err_usb_irq:
	if (pdata->usb_valid)
		free_irq(gpio_to_irq(pdata->uok), data);
err_dc_irq:
	if (pdata->dc_valid)
		free_irq(gpio_to_irq(pdata->dok), data);
err_psy:
	power_supply_unregister(&data->psy_dc);
	power_supply_unregister(&data->psy_usb);
err:
	if (pdata->uok)
		gpio_free(pdata->uok);
	if (pdata->dok)
		gpio_free(pdata->dok);
	if (pdata->flt)
		gpio_free(pdata->flt);
	if (pdata->chg)
		gpio_free(pdata->chg);
	kfree(data);
	return ret;
}

static __devexit int qpower_charger_remove(struct platform_device *pdev)
{
	struct qpower_charger_data *data = platform_get_drvdata(pdev);

	if (data) {
		struct qpower_charger_pdata *pdata = data->pdata;

		if (pdata->flt) {
			free_irq(gpio_to_irq(pdata->flt), data);
			gpio_free(pdata->flt);
		}
		if (pdata->usb_valid) {
			free_irq(gpio_to_irq(pdata->uok), data);
			gpio_free(pdata->uok);
		}
		if (pdata->dc_valid) {
			free_irq(gpio_to_irq(pdata->dok), data);
			gpio_free(pdata->dok);
		}
		power_supply_unregister(&data->psy_dc);
		power_supply_unregister(&data->psy_usb);
		if (pdata->chg)
			gpio_free(pdata->chg);
		kfree(data);
	}

	return 0;
}

static struct platform_driver qpower_charger_driver = {
	.probe	= qpower_charger_probe,
	.remove	= __devexit_p(qpower_charger_remove),
	.driver = {
		.name	= "qpower-charger",
		.owner	= THIS_MODULE,
	},
};

static int __init qpower_charger_init(void)
{
	return platform_driver_register(&qpower_charger_driver);
}

static void __exit qpower_charger_exit(void)
{
	platform_driver_unregister(&qpower_charger_driver);
}
module_init(qpower_charger_init);
module_exit(qpower_charger_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("qpower_charger driver");
MODULE_AUTHOR("MyungJoo Ham <myungjoo.ham@samsung.com>");
MODULE_ALIAS("qpower_charger-charger");
