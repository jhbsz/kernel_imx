/*
 * eup2471.c
 * EUP2471 flash light driver
 *
 * Copyright (C) 2011 Questers Tech,Ltd.ALL RIGHTS RESERVED.
 *  
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/i2c/eup2471.h>
#include <linux/leds.h>
#include <linux/workqueue.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif


#define EUP2471_REG_0	0x00
#define EUP2471_REG_1	0x01

#define MIN_EUP2471_FLASH_TIMEOUT  1
#define MAX_EUP2471_FLASH_TIMEOUT  16

#define MIN_EUP2471_MOVIE_CURRENT_LEVEL  1
#define MAX_EUP2471_MOVIE_CURRENT_LEVEL  16

#define MIN_EUP2471_FLASH_MOVIE_RATIO  1
#define MAX_EUP2471_FLASH_MOVIE_RATIO  16

#define EUP2471_FLASH_TIMEOUT_MASK 0xF	
#define EUP2471_MOVIE_CURRENT_LEVEL_MASK 0xF	
#define EUP2471_MOVIE_CURRENT_LEVEL_OFFSET 4	
#define EUP2471_FLASH_MOVIE_RATIO_MASK 0xF	

#define EUP2471_OUTEN_A 0x20	
#define EUP2471_OUTEN_B 0x10

#define MAX_EUP2471_BRIGHTNESS 10
#define MIN_EUP2471_BRIGHTNESS 0

#define ldev_to_led(c)       container_of(c, struct eup2471_device, ldev)


enum flash_state{
	kStateOff,
	kStatePreFlash,
	kStateFlash,
	kStatePostFlash,
	kStateCompleted,	
	kStateMax,
};

struct eup2471_device{
	struct i2c_client* client;

	unsigned char timeout;
	unsigned char level;
	unsigned char ratio;
	unsigned char outen;

	struct eup2471_platform_data* pdata;

	
	struct led_classdev ldev;
	struct delayed_work work;
	int new_brightness;

	int state;
	
};

struct eup2471_ratio_level_table
{
	unsigned char ratio;
	unsigned char level;
};

static struct eup2471_ratio_level_table rlt[MAX_EUP2471_BRIGHTNESS+1]={
	{MIN_EUP2471_FLASH_MOVIE_RATIO,MIN_EUP2471_MOVIE_CURRENT_LEVEL}, /* 0 */
	{2,2},   /* 23.4/17  =  1.4 */
	{13,3},  /* 25.9/6.1 =  4.2 */
	{13,9},  /* 48.4/6.1 =  7.9 */
	{13,13}, /* 73.1/6.1 = 12   */
	{13,16}, /* 100/6.1 = 16.4 */
	{15,13}, /* 73.1/3.5 = 20.9 */
	{16,10}, /* 53.7/2  = 26.9 */
	{16,12}, /* 65.7/2  = 32.9 */
	{16,14}, /* 80.8/2  = 40.4 */
	{16,16}, /* 100/2   = 50 */
};


//first is brightness,2nd is timing in ms
static int flash_brightness_timing[kStateMax][2]={
	{LED_OFF,0},
	{200,1000},
	{255,200},
	{LED_OFF,0},
	{LED_OFF,0},
};
static int eup2471_regwrite(struct i2c_client *client,u8 reg,u8 value)
{
	u8 buf[4];
	int ret;
	buf[0]=reg;
	buf[1]=value;
	ret=i2c_master_send(client,buf,2);
	if(ret!=2)
	{
		dev_err(&client->dev,"eup2471_regwrite:i2c_master_send ret %d\n",ret);
		return -1;
	}
	return 0;

}
static int eup2471_sync(struct eup2471_device* eup2471)
{	
	int ret;
	ret = eup2471_regwrite(eup2471->client,EUP2471_REG_0,((MAX_EUP2471_FLASH_TIMEOUT-eup2471->timeout)&EUP2471_FLASH_TIMEOUT_MASK)| \
		(((MAX_EUP2471_MOVIE_CURRENT_LEVEL-eup2471->level)&EUP2471_MOVIE_CURRENT_LEVEL_MASK)<<EUP2471_MOVIE_CURRENT_LEVEL_OFFSET));
	if(ret != 0)
		return ret;
	return eup2471_regwrite(eup2471->client,EUP2471_REG_1,((MAX_EUP2471_FLASH_MOVIE_RATIO-eup2471->ratio)& \
		EUP2471_FLASH_MOVIE_RATIO_MASK)| eup2471->outen);
}


static int eup2471_set_movie_brightness(struct eup2471_device* eup2471, int brightness)
{	
	struct eup2471_platform_data* pdata=eup2471->pdata;

	if(brightness>MAX_EUP2471_BRIGHTNESS)
		brightness=MAX_EUP2471_BRIGHTNESS;
	if(brightness<MIN_EUP2471_BRIGHTNESS)
		brightness=MIN_EUP2471_BRIGHTNESS;

	if(brightness!=MIN_EUP2471_BRIGHTNESS)
	{
		if(pdata->enable)
			pdata->enable(1);
	}

	eup2471->ratio=rlt[brightness].ratio;
	eup2471->level=rlt[brightness].level;

	if(brightness==MIN_EUP2471_BRIGHTNESS)
	{
		eup2471->outen=0;
	}
	else
	{
		eup2471->outen=EUP2471_OUTEN_A|EUP2471_OUTEN_B;
		eup2471_sync(eup2471);
	}

	if(brightness==MIN_EUP2471_BRIGHTNESS)
	{
		if(pdata->enable)
			pdata->enable(0);
	}
	return 0;
}

static int eup2471_set_brightness(struct eup2471_device* eup2471,int brightness)
{	
	if(LED_FULL==brightness){
		//start flash mode
		if(eup2471->pdata&&eup2471->pdata->flash)
			eup2471->pdata->flash(1);
	}else {
		if(eup2471->pdata&&eup2471->pdata->flash)
			eup2471->pdata->flash(0);
		//256->10
		brightness = brightness/25;
		eup2471_set_movie_brightness(eup2471,brightness);
	}
	return 0;
}

static ssize_t eup2471_set_flashlight_brightness(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int brightness;
	struct i2c_client* c = container_of(dev, struct i2c_client, dev);
	struct eup2471_device *eup2471 = i2c_get_clientdata(c);	

	sscanf(buf, "%d", &brightness);
	printk("flashlight brightness: %d\n", brightness);
	eup2471_set_brightness(eup2471,brightness);
	return count;
}


static struct device_attribute eup2471_attrs[] = {
	__ATTR(brightness, 0666, NULL, eup2471_set_flashlight_brightness),
};

static void eup2471_led_work(struct work_struct *work)
{
	struct eup2471_device *led = container_of(work, struct eup2471_device, work.work);

	if(LED_FULL==led->new_brightness){
		led->state++;
		eup2471_set_brightness(led, flash_brightness_timing[led->state][0]);
		if(kStateCompleted!=led->state)
			schedule_delayed_work(&led->work,msecs_to_jiffies(flash_brightness_timing[led->state][1]));
		else
			led->state=kStateOff;
	}else {
		eup2471_set_brightness(led, led->new_brightness);
	}
}

static void eup2471_led_set_brightness(struct led_classdev *led_cdev,
				      enum led_brightness brightness)
{
	struct eup2471_device *led = ldev_to_led(led_cdev);

	dev_dbg(&led->client->dev, "%s: %s, %d\n",
		__func__, led_cdev->name, brightness);

	cancel_delayed_work_sync(&led->work);

	led->new_brightness = brightness;
	if(LED_FULL==brightness){
		led->state = kStateOff;
		schedule_delayed_work(&led->work,msecs_to_jiffies(flash_brightness_timing[kStateOff][1]));
	}else 
		schedule_delayed_work(&led->work,0);
}

static int eup2471_suspend(struct i2c_client* client,pm_message_t state)
{
	struct eup2471_device *eup2471 = i2c_get_clientdata(client);	
	struct eup2471_platform_data* pdata=eup2471->pdata;

	if(pdata&&pdata->flash)	pdata->flash(0);
	if(pdata&&pdata->enable) pdata->enable(0);
	
	return 0;
}

static int eup2471_resume(struct i2c_client* client)
{
	struct eup2471_device *eup2471 = i2c_get_clientdata(client);	
	eup2471->level = MIN_EUP2471_MOVIE_CURRENT_LEVEL;
	eup2471->ratio = MIN_EUP2471_FLASH_MOVIE_RATIO;
	eup2471->outen= 0;
	return 0;
}


static int __devinit eup2471_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct eup2471_device* eup2471=NULL;
	struct eup2471_platform_data* pdata = (struct eup2471_platform_data*)client->dev.platform_data;
	struct device *dev = &client->dev;
	int ret = 0,i;		

	if(pdata)
	{
		if(pdata->flash)
			pdata->flash(0);
		if(pdata->enable)
			pdata->enable(0);
	}
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk(KERN_ERR"eup2471_probe: need I2C_FUNC_I2C\n");
		return -ENODEV;
	}
	
	eup2471 = kzalloc(sizeof(struct eup2471_device),GFP_KERNEL);
	if(!eup2471)
	{
		dev_err(dev,"failed to create eup2471_device\n");
		ret=-1;
		goto err;
	}
	eup2471->client=client;
	eup2471->pdata = pdata;
	eup2471->timeout = MAX_EUP2471_FLASH_TIMEOUT;
	eup2471->level = MIN_EUP2471_MOVIE_CURRENT_LEVEL;
	eup2471->ratio = MIN_EUP2471_FLASH_MOVIE_RATIO;
	eup2471->outen= 0;

	//write default value
	/*ret = eup2471_idle(eup2471);
	if(ret!=0)
	{
		dev_err(dev,"eup2471 not exist or state error ret=%d\n",ret);
		goto err1;
	}*/

	eup2471->ldev.name = pdata->name?pdata->name:"eup2471";
	eup2471->ldev.max_brightness = LED_FULL;
	eup2471->ldev.brightness_set = eup2471_led_set_brightness;
	if(pdata->default_trigger)
		eup2471->ldev.default_trigger = pdata->default_trigger;
	eup2471->ldev.flags = LED_CORE_SUSPENDRESUME;

	ret = led_classdev_register(&client->dev, &eup2471->ldev);
	if (ret < 0) {
		dev_err(&client->dev,
			"couldn't register LED %s\n",
			eup2471->ldev.name);
		goto err;
	}

	i2c_set_clientdata(client, eup2471);

	INIT_DELAYED_WORK(&eup2471->work, eup2471_led_work);

	for (i=0; i<ARRAY_SIZE(eup2471_attrs); i++)
	{
		ret = device_create_file(&client->dev, &eup2471_attrs[i]);
		if (ret) printk("eup2471_probe: failed creating device file\n");
	}

	return 0;
err:
	if(eup2471)
		kfree(eup2471);
	/*if(pdata)
	{
		if(pdata->enable)
			pdata->enable(0);
	}*/
	return ret;
}

static int eup2471_remove(struct i2c_client *client)
{
	struct eup2471_device *eup2471 = i2c_get_clientdata(client);	
	struct eup2471_platform_data* pdata=eup2471->pdata;
	int i;

	for (i=0; i<ARRAY_SIZE(eup2471_attrs); i++)
	{
		device_remove_file(&client->dev, &eup2471_attrs[i]);
	}

	if(pdata&&pdata->flash)	pdata->flash(0);
	if(pdata&&pdata->enable) pdata->enable(0);
	
	cancel_delayed_work_sync(&eup2471->work);
	led_classdev_unregister(&eup2471->ldev);

	kfree(eup2471);

	dev_set_drvdata(&client->dev, NULL);
	return 0;
}
 

static struct i2c_device_id eup2471_idtable[] = { 
	{ "eup2471", 0 }, 
	{ } 
}; 

static struct i2c_driver eup2471_driver = {
	.driver = {
		.name 	= "eup2471",
	},
	.id_table       = eup2471_idtable,
	.probe		= eup2471_probe,
	.remove		= __devexit_p(eup2471_remove),
	
	#ifdef CONFIG_PM
	.suspend  = eup2471_suspend,
	.resume   = eup2471_resume,
	#endif
};

static int __init eup2471_init(void)
{	
	return i2c_add_driver(&eup2471_driver);
}

static void __exit eup2471_exit(void)
{	
	i2c_del_driver(&eup2471_driver);
}

module_init(eup2471_init);
module_exit(eup2471_exit);

MODULE_DESCRIPTION("EUP2471 flash light driver");
MODULE_AUTHOR("Ellie Cao <elliejcao@gmail.com>");
MODULE_LICENSE("GPL");

