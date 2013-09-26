/*
 *  qpower_battery.c based max17058_battery.c
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 *
 *  Copyright (C) 2009 Samsung Electronics
 *  Minkyu Kang <mk7.kang@samsung.com> 
 * Copyright 2013 Quester Technology,Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/power/qpower.h>
#include <linux/firmware.h>
#include <linux/gpio.h>

/*
 *All max17058 register is 16bit,
*/
#define REG_OFFSET_VCELL	0x02
#define REG_OFFSET_SOC		0x04
#define REG_OFFSET_MODE		0x06
#define REG_OFFSET_VER		0x08
#define REG_OFFSET_CONFIG	0x0C
#define REG_OFFSET_VRESET	0x18
#define REG_OFFSET_STATUS	0x1A
#define REG_OFFSET_CMD		0xFE


#define REG_OFFSET_TABLE_ACCESS	0x3E
#define REG_OFFSET_TABLE_START	0x40
#define REG_OFFSET_TABLE_END	0x7F


//REG VCELL
#define REG_VCELL_SCALE 		78125 /*78.125uV/cell*/

//REG SOC
#define REG_SOC_SCALE			256		/*1%/256*/

//REG MODE
#define REG_MODE_QUICKSTART 	0x4000
#define REG_MODE_SLEEPENABLE    0x2000

//REG CONFIG
#define REG_CONFIG_RCOMP_MASK		0xFF00
#define REG_CONFIG_SLEEP			0x80
#define REG_CONFIG_ALERT			0x20
#define REG_CONFIG_ALERT_THRS_MASK	0x1F

//REG VRESET

//REG STATUS
#define REG_STATUS_RI				0x100	/*Reset Indicator*/	


//REG TABLE LOCK/UNLOCK
#define REG_TABLE_LOCK_PATTERN		0x4A57
#define REG_TABLE_UNLOCK_PATTERN	0x0000
#define REG_TABLE_SIZE				32

#define MAX17058_POLLING_DELAY		(1000)   
#define MAX17058_BATTERY_FULL		100


struct battery_chip {
	struct i2c_client		*client;
	struct delayed_work		pollwork;
	struct power_supply		battery;
	struct qpower_battery_pdata	*pdata;

	struct wake_lock wl;

	int irq;

	/* State Of Connect */
	int online;
	/* battery voltage */
	int vcell;
	/* battery ocv */
	int ocv;
	/* battery capacity */
	int soc;
	/* State Of Charge */
	int status;
	/* state of health */
	int health;

	
	int prev_soc;
	int counter;

	
};

static uint16_t default_table[] = {
	0xAA00, 0xB1F0, 
	0xB7E0, 0xB960, 
	0xBB80, 0xBC40,
	0xBD30, 0xBD50,
	0xBDF0, 0xBE40,
	0xBFD0, 0xC090,
	0xC430, 0xC7C0,
	0xCA60, 0xCF30, 
	0x0120, 0x09C0,
	0x1FC0, 0x2BE0,
	0x4FC0, 0x3000,
	0x4780, 0x4FE0,
	0x7700, 0x1560,
	0x4620, 0x1380, 
	0x1A60, 0x1220,
	0x14A0, 0x14A0
}; 

static int chip_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct battery_chip *chip = container_of(psy,struct battery_chip,battery);
	if(chip->prev_soc==0)
		chip->prev_soc = chip->soc;//init prev_soc

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = chip->status;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chip->online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = chip->vcell*1000;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if(chip->pdata&&
			chip->pdata->ops&&
			chip->pdata->ops->qco&&
			charger_online(chip->pdata->ops)){ //charger is online
			chip->prev_soc = chip->soc;		
		}
		else{
			if(chip->soc < chip->prev_soc) 
				chip->prev_soc = chip->soc;
		}
		val->intval = chip->prev_soc;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = chip->health;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}


static int chip_write_word(struct i2c_client *client, int reg, u16 value)
{
    return i2c_smbus_write_word_data(client,reg,value);
}

static int chip_read_word(struct i2c_client *client, int reg)
{
	return i2c_smbus_read_word_data(client, reg);
//    return swab16((uint16_t)(i2c_smbus_read_word_data(client, reg)&0xffff);
}



static void chip_reset(struct i2c_client *client)
{
	 chip_write_word(client, REG_OFFSET_CMD, 0x5400); //compeletly reset	 
}

static void chip_table_lock(struct i2c_client *client,int lock)
{
	if(!lock)
		chip_write_word(client, REG_OFFSET_TABLE_ACCESS, REG_TABLE_UNLOCK_PATTERN);//unlock
	else
		chip_write_word(client, REG_OFFSET_TABLE_ACCESS, REG_TABLE_LOCK_PATTERN);//lock
}


static int chip_load_table(struct battery_chip *chip ,uint16_t *table,int tsize){
	int i;
	if(tsize<REG_TABLE_SIZE)
		return -EINVAL;
	chip_table_lock(chip->client,1);
	for(i=0;i<REG_TABLE_SIZE;i++){
		chip_write_word(chip->client,REG_OFFSET_TABLE_START+i*2,table[i]);
	}
	chip_table_lock(chip->client,0);

	return 0;
	
}

static int chip_get_version(struct i2c_client *client)
{
	return chip_read_word(client, REG_OFFSET_VER); 	  	 
}


static void chip_update(struct battery_chip *chip){
	int  ret;
 	ret = chip_read_word(chip->client, REG_OFFSET_VCELL);//16bit  read
 	if(ret>=0){ 	 
	 	chip->vcell = ((ret>>8)&0xff)* 20+(ret&0xff)*10/128; //mc,78.125uV/cell  78.125uV*2^8=20mV
	}

 	ret = chip_read_word(chip->client, REG_OFFSET_SOC);//16bit  read
 	if(ret>=0){
		int alrt;
		int soc;
		if(((ret>>8)&0xff) == 1){
			alrt = chip_read_word(chip->client, REG_OFFSET_CONFIG);
			if(alrt&REG_CONFIG_ALERT)
				chip_write_word(chip->client,REG_OFFSET_CONFIG,alrt&(~REG_CONFIG_ALERT));//clear alrt bit
		}
	    soc = min(((ret>>8)&0xff)/2, 100); //mc,uboot-load custom model,1%/512cell,
		chip->soc=min(soc,100);
	}

	

}





static void chip_get_status(struct i2c_client *client)
{
	struct battery_chip *chip = i2c_get_clientdata(client);
	struct qpower_ops* ops = chip->pdata->ops;
	if(!ops) {
		dev_warn(&client->dev,"no battery ops found\n");
		return;
	}

	chip_update(chip);


	//status
	if (charger_online(ops)) {
		if (charger_enable(ops))
			chip->status = POWER_SUPPLY_STATUS_CHARGING;
		else
			chip->status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	} else {
		chip->status = POWER_SUPPLY_STATUS_DISCHARGING;
	}
	//FIXME: check if status full
	if (chip->soc >= MAX17058_BATTERY_FULL){
		
		chip->status = POWER_SUPPLY_STATUS_FULL;
	}

	//battery health
	chip->health = battery_faulty(ops)?POWER_SUPPLY_HEALTH_DEAD:POWER_SUPPLY_HEALTH_GOOD;

	//online
	chip->online = battery_online(ops);

	
}
		

static void chip_work(struct work_struct *work)
{
	struct battery_chip *chip = container_of(work, struct battery_chip, pollwork.work);

	chip_get_status(chip->client);
	
	power_supply_changed(&chip->battery);

	schedule_delayed_work(&chip->pollwork, msecs_to_jiffies(MAX17058_POLLING_DELAY));
}

static irqreturn_t chip_irq_handler(int irq, void *data)
{
	struct battery_chip *chip = (struct battery_chip *)data;
	wake_lock_timeout(&chip->wl, 5*HZ);
	//max17058_get_soc(chip->client);
	cancel_delayed_work(&chip->pollwork);
	schedule_delayed_work(&chip->pollwork,2*HZ);
	return IRQ_HANDLED;
}

static ssize_t chip_regs_show(struct device *dev,
                          struct device_attribute *attr, char *buf)
{
	struct battery_chip *chip = dev_get_drvdata(dev);
	int i,length=0;
	int value;
	int ret=0;

	ret = chip_read_word(chip->client,REG_OFFSET_VCELL);
	if(ret>=0){
		value = ((ret>>8)&0xff)* 20+(ret&0xff)*10/128;//78.125uV/vcell
		length+= sprintf(buf+length,"vcell[%04x,%duV]\n",ret,value);
	}
	ret = chip_read_word(chip->client,REG_OFFSET_SOC);
	if(ret>=0){
		value = ret/REG_SOC_SCALE;
		length+= sprintf(buf+length,"soc[%04x,%d]\n",ret,value);
	}
	ret = chip_read_word(chip->client,REG_OFFSET_MODE);
	if(ret>=0){
		length+= sprintf(buf+length,"soc[%04x]\n",ret);
	}
	ret = chip_read_word(chip->client,REG_OFFSET_VER);
	if(ret>=0){
		length+= sprintf(buf+length,"ver[%04x]\n",ret);
	}
	ret = chip_read_word(chip->client,REG_OFFSET_CONFIG);
	if(ret>=0){
		length+= sprintf(buf+length,"config[%04x]\n",ret);
	}
	ret = chip_read_word(chip->client,REG_OFFSET_VRESET);
	if(ret>=0){
		length+= sprintf(buf+length,"vreset[%04x]\n",ret);
	}
	ret = chip_read_word(chip->client,REG_OFFSET_STATUS);
	if(ret>=0){
		length+= sprintf(buf+length,"status[%04x]\n",ret);
	}
	ret = chip_read_word(chip->client,REG_OFFSET_CMD);
	if(ret>=0){
		length+= sprintf(buf+length,"cmd[%04x]\n",ret);
	}
	for(i=0;i<REG_TABLE_SIZE;i++){
		ret = chip_read_word(chip->client,REG_OFFSET_TABLE_START+i*2);
		if(ret>=0){
			length+= sprintf(buf+length,"tables[%02x:%04x]\n",i,ret);
		}
		
	}
	if(!length)
		length+= sprintf(buf+length,"invalid access\n");

	return length;
}


static DEVICE_ATTR(regs, S_IRUGO|S_IWUSR,chip_regs_show, NULL);

static ssize_t table_show(struct device *dev,
                          struct device_attribute *attr, char *buf){
    int length=0;
	length+= sprintf(buf+length,"echo <firmware> > table -- to load custom table data\n");


	return length;
}
static ssize_t table_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count){
	struct battery_chip *chip = dev_get_drvdata(dev);	
	const struct firmware *fw;	
	char firmware_name[256];
	int ret;
	
	sscanf(buf,"%s",firmware_name);
	
	dev_info(dev,"loading firmware %s\n",firmware_name);

	
	ret  = request_firmware(&fw,firmware_name,dev);
	if(ret){		
		dev_err(dev, "load firmware %s fail %d\n", firmware_name,ret);
		goto bail;
	}

	//retrieve table data
	if((fw->size/2) < REG_TABLE_SIZE){
		dev_err(dev, "invalid firmware %s fail,binary size too small\n", firmware_name);	
	}else {
		chip_load_table(chip,(uint16_t*)fw->data,fw->size);
	}

	dev_info(dev,"table store okay\n");
	release_firmware(fw);
bail:	
	return count;
}

static DEVICE_ATTR(table, S_IRUGO|S_IWUSR,table_show, table_store);

static enum power_supply_property battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_HEALTH,
};


static int chip_init(struct battery_chip *chip,struct i2c_client *client){
	int ret;
	int alert_threshold = 2;

	wake_lock_init(&chip->wl, WAKE_LOCK_SUSPEND, "low_bat");


	chip->client = client;
	chip->pdata = client->dev.platform_data;

	i2c_set_clientdata(client, chip);

	ret = device_create_file(&client->dev,&dev_attr_regs);
	ret = device_create_file(&client->dev,&dev_attr_table);


	//clear alert/sleep flags,2% alert warning
	//alert is 1%~32%
	if((chip->pdata->alert_threshold<=32)&&(chip->pdata->alert_threshold>0)){
		alert_threshold = chip->pdata->alert_threshold;
	}
	alert_threshold = 32 - alert_threshold;
	chip_write_word(client,REG_OFFSET_CONFIG,((alert_threshold&0x1F) |(chip_read_word(client,0x0C) & 0xFF00)));

	chip_load_table(chip,default_table,ARRAY_SIZE(default_table));
	
	INIT_DELAYED_WORK(&chip->pollwork, chip_work);

	if(chip->pdata->alert&&gpio_is_valid(chip->pdata->alert)){		
		
		ret = gpio_request(chip->pdata->alert, "bat_alert");
		if (ret < 0) {
			dev_err(&client->dev, "failed to configure"
				" request/direction for GPIO %d, error %d\n",
				chip->pdata->alert, ret);
			goto err;
		}
		gpio_direction_input(chip->pdata->alert);
		chip->irq = gpio_to_irq(chip->pdata->alert);
		ret = request_threaded_irq(chip->irq, NULL, chip_irq_handler,
					  IRQF_TRIGGER_FALLING , client->name, chip);  
		device_init_wakeup(&client->dev, 1);
	}

	chip->health = POWER_SUPPLY_HEALTH_GOOD;

	chip->battery.name		= "battery";
	chip->battery.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->battery.get_property	= chip_get_property;
	chip->battery.properties	= battery_props;
	chip->battery.num_properties	= ARRAY_SIZE(battery_props);

	ret = power_supply_register(&client->dev, &chip->battery);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		return ret;
	}
	schedule_delayed_work(&chip->pollwork, msecs_to_jiffies(HZ));
err:
	return ret;
}

static int max17058_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct battery_chip *chip;
	int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -EIO;

	ret = chip_get_version(client);
	if(ret<0){
		return -ENODEV;
	}
 	dev_info(&client->dev, "QPower Fuel-Gauge Ver %.4x\n", ret);

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	ret = chip_init(chip,client);
	if(ret<0){
		kfree(chip);
	}


	return ret;
}

static int __devexit max17058_remove(struct i2c_client *client)
{
	struct battery_chip *chip = i2c_get_clientdata(client);

	power_supply_unregister(&chip->battery);	
	cancel_delayed_work(&chip->pollwork);
	device_remove_file(&client->dev,&dev_attr_regs);
	device_remove_file(&client->dev,&dev_attr_table);
	kfree(chip);
	return 0;
}

#ifdef CONFIG_PM

static int max17058_suspend(struct i2c_client *client,
		pm_message_t state)
{
	struct battery_chip *chip = i2c_get_clientdata(client);
	cancel_delayed_work(&chip->pollwork);
	return 0;
}

static int max17058_resume(struct i2c_client *client)
{
	struct battery_chip *chip = i2c_get_clientdata(client);
	schedule_delayed_work(&chip->pollwork, 0);
	return 0;
}

#else

#define max17058_suspend NULL
#define max17058_resume NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id max17058_id[] = {
	{ "max17058", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17058_id);

static struct i2c_driver max17058_i2c_driver = {
	.driver	= {
		.name	= "max17058",
		.owner	= THIS_MODULE,			
	},
	.probe		= max17058_probe,
	.remove		= __devexit_p(max17058_remove),
	.suspend	= max17058_suspend,
	.resume		= max17058_resume,
	.id_table	= max17058_id,
};

static int __init qpower_battery_init(void)
{
	return i2c_add_driver(&max17058_i2c_driver);
}
module_init(qpower_battery_init);

static void __exit qpower_battery_exit(void)
{
	i2c_del_driver(&max17058_i2c_driver);
}
module_exit(qpower_battery_exit);

MODULE_AUTHOR("raymond wang<raymond1860@gmail.com>");
MODULE_DESCRIPTION("maxim 17058 Fuel Gauge");
MODULE_LICENSE("GPL");
