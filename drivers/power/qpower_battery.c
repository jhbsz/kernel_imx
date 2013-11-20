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
#define REG_OFFSET_HIBERNATE 0x0A
#define REG_OFFSET_CONFIG	0x0C
#define REG_OFFSET_OCV		0x0E
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
#define REG_TABLE_UNLOCK_PATTERN		0x4A57
#define REG_TABLE_LOCK_PATTERN			0x0000
#define REG_TABLE_SIZE				64

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

static int debug = 0;
module_param_named(debug, debug, int, S_IRUGO | S_IWUSR | S_IWGRP);


static uint8_t default_table[] = {
	0xAA, 0x00, 0xB1, 0xF0, 0xB7, 0xE0, 0xB9, 0x60, 0xBB, 0x80,
	0xBC, 0x40, 0xBD, 0x30, 0xBD, 0x50, 0xBD, 0xF0, 0xBE, 0x40,
	0xBF, 0xD0, 0xC0, 0x90, 0xC4, 0x30, 0xC7, 0xC0, 0xCA, 0x60,
	0xCF, 0x30, 0x01, 0x20, 0x09, 0xC0, 0x1F, 0xC0, 0x2B, 0xE0,
	0x4F, 0xC0, 0x30, 0x00, 0x47, 0x80, 0x4F, 0xE0, 0x77, 0x00,
	0x15, 0x60, 0x46, 0x20, 0x13, 0x80, 0x1A, 0x60, 0x12, 0x20,
	0x14, 0xA0, 0x14, 0xA0,
}; 
static uint8_t vmodel_table_1[] = {
	0x82, 0x20, 0xAA, 0xB0, 0xB4, 0x70, 0xBB, 0x40, 0xBC, 0x10, 
	0xBC, 0x70, 0xBC, 0xF0, 0xBD, 0x70, 0xBE, 0x30, 0xBF, 0x00, 
	0xC1, 0x00, 0xC3, 0x00,	0xC6, 0x60, 0xC8, 0x50, 0xCC, 0x30,
	0xD0, 0x90, 0x00, 0x40, 0x08, 0x00,	0x0E, 0x60, 0x34, 0xE0,
	0x48, 0x20, 0x7A, 0x60, 0x70, 0x40, 0x40, 0x20,	0x3B, 0xE0, 
	0x13, 0xE0, 0x19, 0x60, 0x16, 0x00, 0x19, 0x20, 0x0F, 0xE0,
	0x0E ,0x20, 0x0E, 0x20, 
}; 


static int chip_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct battery_chip *chip = container_of(psy,struct battery_chip,battery);

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
		val->intval = chip->soc;
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
    return i2c_smbus_write_word_data(client,reg,swab16(value));
}

static int chip_read_word(struct i2c_client *client, int reg)
{
	int ret = i2c_smbus_read_word_data(client, reg);
	if(ret>=0)
	    return swab16((uint16_t)(ret&0xffff));
	dev_warn(&client->dev,"read reg failed[%d]\n",ret);
	return ret;
}


/*

static void chip_reset(struct i2c_client *client)
{
	 chip_write_word(client, REG_OFFSET_CMD, 0x5400); //compeletly reset	 
}
static int chip_write_rcomp_seg(struct i2c_client *client,
                                                uint16_t rcomp_seg)
{
	uint8_t rs1, rs2;
	int ret;


	rs2 = rcomp_seg | 0x00FF;
	rs1 = rcomp_seg >> 8;

	uint8_t rcomp_seg_table[16] = { rs1, rs2, rs1, rs2,
	                                rs1, rs2, rs1, rs2,
	                                rs1, rs2, rs1, rs2,
	                                rs1, rs2, rs1, rs2};


	ret = i2c_smbus_write_i2c_block_data(client, MAX17048_RCOMPSEG1,
	                        16, (uint8_t *)rcomp_seg_table);
	if (ret < 0) {
	        dev_err(&client->dev, "%s: err %d\n", __func__, ret);
	        return ret;
	}

	ret = i2c_smbus_write_i2c_block_data(client, MAX17048_RCOMPSEG2,
	                        16, (uint8_t *)rcomp_seg_table);
	if (ret < 0) {
	        dev_err(&client->dev, "%s: err %d\n", __func__, ret);
	        return ret;
	}

	return 0;
}
*/
static int chip_load_table(struct battery_chip *chip ,uint8_t *table,int tsize){
	struct i2c_client		*client=chip->client;
	int i;	
	uint16_t soc_tst, ocv;
	int ret;
	if(tsize<REG_TABLE_SIZE){
		dev_err(&chip->client->dev, "%s: error of table size:\n",
												__func__);		
		return -EINVAL;
	}
unlock_retry:	
	//step 1
	chip_write_word(client, REG_OFFSET_TABLE_ACCESS, REG_TABLE_UNLOCK_PATTERN);
	
	//step 2
	ret = chip_read_word(client, REG_OFFSET_OCV);
	if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return ret;
	}
	ocv = (uint16_t)ret;
	//step 2.5
	if (ocv == 0xffff) {
			dev_err(&client->dev, "%s: Failed in unlocking"
									"max17058 err: %d\n", __func__, ocv);
			msleep(10);
			goto unlock_retry;
	}
	//step 3
	//step 4 17040/1/3/4 only
	/*
	ret = chip_write_rcomp_seg(client, 0xffff);
	if (ret < 0)
			return ret;
	*/
	//step 5:load model table data
	for (i = 0; i < 4; i++) {
		if (i2c_smbus_write_i2c_block_data(chip->client,
				(REG_OFFSET_TABLE_START+i*16), 16,
						&table[i*0x10]) < 0) {
				dev_err(&chip->client->dev, "%s: error writing model data:\n",
														__func__);
				break;
		}
	}
	
	/* Delay between 150ms to 600ms */
	msleep(200);

	#if 0

	//step 7:  Write OCV Test value
	ret = chip_write_word(client, REG_OFFSET_OCV, 0xa1ff);
	if (ret < 0)
			return ret;
	
	
	/*step 7.1: Disable hibernate */
	ret = chip_write_word(client, REG_OFFSET_HIBERNATE, 0x0000);
	if (ret < 0)
			return ret;
	#endif
	/*step 7.2: Lock model access */
	ret = chip_write_word(client, REG_OFFSET_TABLE_ACCESS, REG_TABLE_LOCK_PATTERN);
	if (ret < 0)
			return ret;
		
	/*step 8 delay at least 150ms*/
	msleep(200);

	soc_tst=0;
	#if 0
	/*step 9 Read SOC Register and compare to expected result */
	ret = chip_read_word(client, REG_OFFSET_SOC);
	if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return ret;
	}
	soc_tst = (uint16_t)ret;
	/*
	if (!((soc_tst >> 8) >= mdata->soccheck_A &&
							(soc_tst >> 8) <=  mdata->soccheck_B)) {
			dev_err(&client->dev, "%s: soc comparison failed %d\n",
									__func__, ret);
			return ret;
	} else {
			dev_info(&client->dev, "MAX17058 Custom data"
											" loading successfull\n");
	}*/
	
	/* step 9.1 unlock model access */
	ret = chip_write_word(client, REG_OFFSET_TABLE_ACCESS,
									REG_TABLE_UNLOCK_PATTERN);
	if (ret < 0)
			return ret;
	
	/*step 10 Restore config and OCV */
	ret = chip_write_word(client, REG_OFFSET_OCV, ocv);
	if (ret < 0)
			return ret;

	//step 10.1 restore hibernate

	//step 11 lock model access
	chip_write_word(client, REG_OFFSET_TABLE_ACCESS, REG_TABLE_LOCK_PATTERN);

	//step 12 delay at least 150ms
	msleep(200);	
	#endif
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
		int soc;
		int alrt = chip_read_word(chip->client, REG_OFFSET_CONFIG);
		if(alrt&REG_CONFIG_ALERT)
			chip_write_word(chip->client,REG_OFFSET_CONFIG,alrt&(~REG_CONFIG_ALERT));//clear alrt bit
		soc = ret >> 9;
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
		if (charger_enable(ops)){
			chip->status = POWER_SUPPLY_STATUS_CHARGING;
		}
		else {
			chip->status = POWER_SUPPLY_STATUS_NOT_CHARGING;
			if (chip->soc >= MAX17058_BATTERY_FULL)
				chip->status = POWER_SUPPLY_STATUS_FULL;
		}
	} else {
		chip->status = POWER_SUPPLY_STATUS_DISCHARGING;
	}

	//battery health
	chip->health = battery_faulty(ops)?POWER_SUPPLY_HEALTH_DEAD:POWER_SUPPLY_HEALTH_GOOD;

	//online
	chip->online = battery_online(ops);

	if(debug){
		pr_info("status=%d,online=%d,soc=%d,vcell=%d\n",chip->status,chip->online,chip->soc,chip->vcell);		
		pr_info("charger_online=%d,charger_enable=%d\n",charger_online(ops),charger_enable(ops));
	}
	
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
	int length=0;
	int value;
	int ret=0;

	ret = chip_read_word(chip->client,REG_OFFSET_VCELL);
	if(ret>=0){
		value = ((ret>>8)&0xff)* 20+(ret&0xff)*10/128;//78.125uV/vcell
		length+= sprintf(buf+length,"vcell[%04x,%dmV]\n",ret,value);
	}
	ret = chip_read_word(chip->client,REG_OFFSET_SOC);
	if(ret>=0){
		value = ret>>9;
		length+= sprintf(buf+length,"soc[%04x,%d]\n",ret,value);
	}
	ret = chip_read_word(chip->client,REG_OFFSET_MODE);
	if(ret>=0){
		length+= sprintf(buf+length,"mode[%04x]\n",ret);
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
	if((fw->size) < REG_TABLE_SIZE){
		dev_err(dev, "invalid firmware %s fail,binary size too small\n", firmware_name);	
	}else {
		chip_load_table(chip,(uint8_t*)fw->data,fw->size);
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


	if(default_table[0]){}
	if(vmodel_table_1[0]){}
	chip_load_table(chip,vmodel_table_1,ARRAY_SIZE(vmodel_table_1));


	//clear alert/sleep flags,2% alert warning
	//alert is 1%~32%
	if((chip->pdata->alert_threshold<=32)&&(chip->pdata->alert_threshold>0)){
		alert_threshold = chip->pdata->alert_threshold;
	}
	alert_threshold = 32 - alert_threshold;
	chip_write_word(client,REG_OFFSET_CONFIG,((alert_threshold&0x1F) |(chip_read_word(client,0x0C) & 0xFF00)));

	
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
	schedule_delayed_work(&chip->pollwork, msecs_to_jiffies(50));
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
		dev_err(&client->dev, "QPower Fuel-Gauge init failed [%d]\n", ret);
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
