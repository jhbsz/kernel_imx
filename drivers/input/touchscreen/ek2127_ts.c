/*
 * EKTF2040 TP driver
 *
 * Copyright (C) 2011 Eavoo. Inc
 * Author: Jun.Zhou <Michael.Zhou@eavoo.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 *
 * Author				Date				Description
 * Zhoujun				2011.12.13			Created
 *
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/input/touchscreen.h>

#include <linux/slab.h>
#include <asm/uaccess.h>


// for linux 2.6.36.3
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#undef CONFIG_HAS_EARLYSUSPEND
//ADD  IAP
#define IAP_IN_DRIVER_MODE 	1
#define IAP_PORTION            	0
#define PAGERETRY  30
#define IAPRESTART 5
#define CMD_54001234	1
//ADD IAP END


int tp_rst_default_value;

// Normal Package Information
#define PACKET_SIZE						18
#define FINGER_NUM						2

#define FLAG_FINGER_UP					0x0
#define FLAG_FIRST_FINGER				0x1
#define FLAG_SECOND_FINGER				0x2
#define FLAG_DOUBLE_FINGER				0x3
#define FLAG_MENU						0x4
#define FLAG_HOME						0x8
#define FLAG_BACK						0x10

// Power control
#define PWR_STATE_DEEP_SLEEP			0
#define PWR_STATE_NORMAL				1
#define PWR_STATE_MASK					BIT(3)

// Command
#define CMD_S_PKT						0x52
#define CMD_R_PKT						0x53
#define CMD_W_PKT						0x54
#define HELLO_PKT						0x55


#define NORMAL_PKT			0x5D

#define TWO_FINGERS_PKT      0x5A
#define FIVE_FINGERS_PKT       0x6D
#define TEN_FINGERS_PKT	0x62



#define TS_X_MAX		         480
#define TS_Y_MAX		         854

#define PRESS_MAX               255

#define ELAN_X_MAX 448//1186	

#define ELAN_Y_MAX 832//1792

#define ELAN_BUTTON
#define BUTTON_ID_INDEX 7


//#define SOFTKEY_AXIS_VER 1


#define INPUT_DEV_NAME		    "currency_tp"
#define VIRTUAL_KEY_NAME	    "virtualkeys.currency_tp"

#define TP_DEVICE_TYPE
//#ifdef SOFTKEY_AXIS_VER


#define ELAN_KEY_HOME	0x08
#define ELAN_KEY_MENU	0x04
#define ELAN_KEY_BACK	0x10
//#define ELAN_KEY_SEARCH  0x11

#if 1
static int key_pressed = -1;

struct osd_offset{
	int left_x;
	int right_x;
	unsigned int key_event;
};

static struct osd_offset OSD_mapping[] = { // Range need define by Case!
	{40, 500,  ELAN_KEY_MENU},	//menu_left_x, menu_right_x, KEY_MENU
	{120, 500, ELAN_KEY_HOME},	//home_left_x, home_right_x, KEY_HOME
	{200, 500, ELAN_KEY_BACK},	//back_left_x, back_right_x, KEY_BACK
//	{600, 779, ELAN_KEY_SEARCH},	//search_left_x, search_right_x, KEY_SEARCH
};
#endif 

#if IAP_PORTION
uint8_t ic_status=0x00;	//0:OK 1:master fail 2:slave fail
int update_progree=0;
uint8_t I2C_DATA[3] = {0x15, 0x20, 0x21};/*I2C devices address*/  
int is_OldBootCode = 0; // 0:new 1:old



/*The newest firmware, if update must be changed here*/
static uint8_t file_fw_data[] = {
#include "fw_data.i"
};


enum
{
	PageSize		= 132,
	PageNum		    = 249,
	ACK_Fail		= 0x00,
	ACK_OK			= 0xAA,
	ACK_REWRITE		= 0x55,
};

enum
{
	E_FD			= -1,
};
#endif


// Debug function
#define pr_red_info(fmt, arg ...)		printk(KERN_ERR "\033[31m" fmt "\033[0m\n", ##arg)
#define pr_green_info(fmt, arg ...)		printk(KERN_INFO "\033[32m" fmt "\033[0m\n", ##arg)
#define pr_pos_info()					printk(KERN_INFO "%s [%d]\n", __FUNCTION__, __LINE__)

/*Device info*/
#define EKTF2040_DEV_NAME				"EKFT2K"
#define EKTF2040_ADDR					0X15

#define EKTF2040_GPIO_RST				98
#define EKTF2040_GPIO_IRQ				94

#define ANDROID_KEY_MENU				139
#define ANDROID_KEY_HOME				102
#define ANDROID_KEY_BACK				158

#define I2C_STATIC_BUS_NUM  (0)

struct elan_ktf2k_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *elan_wq;
	struct work_struct work;

	int intr_gpio;
	struct early_suspend early_suspend;

	struct touchscreen_platform_data* pdata;
// Firmware Information
	int fw_ver;
	int x_resolution;
	int y_resolution;
	int fw_id;
// For Firmare Update
	struct miscdevice firmware;
};

// Globle variable
static uint8_t RECOVERY = 0x00;
static int FW_VERSION = 0x00;
static int X_RESOLUTION = 0x00;
static int Y_RESOLUTION = 0x00;
static int FW_ID = 0x00;
static int work_lock=0x00;
static int file_fops_addr=0x15;
static int power_lock=0x00;
static int button_state = 0;

static struct kobject *properties_kobj;

static unsigned int ektf2040_key_map[] = {ANDROID_KEY_MENU, ANDROID_KEY_HOME, ANDROID_KEY_BACK};

static struct elan_ktf2k_ts_data *private_ts;
static int __fw_packet_handler(struct i2c_client *client);
static int elan_ktf2k_ts_resume(struct i2c_client *client);
#ifdef CONFIG_HAS_EARLYSUSPEND
static void elan_ktf2k_ts_early_suspend(struct early_suspend *h);
static void elan_ktf2k_ts_late_resume(struct early_suspend *h);
#endif

#if IAP_PORTION
int Update_FW_One(/*struct file *filp,*/ struct i2c_client *client, int 
recovery);
static int __hello_packet_handler(struct i2c_client *client);
int IAPReset(struct i2c_client *client);
#endif


static void ektf2040_reset(void)
{
	pr_pos_info();

	if(private_ts&&private_ts->pdata&&private_ts->pdata->reset){
		private_ts->pdata->reset();
	}
}

static int get_interrupt_level(int irq){	
	int irqgpio = irq_to_gpio(irq);
	int irqlevel;
	gpio_request(irqgpio,"tsirq");
	irqlevel = gpio_get_value(irqgpio);
	gpio_free(irqgpio);
	return irqlevel;
}



/******************************** For Firmware Update ******************************************/
static int elan_iap_open(struct inode *inode, struct file *filp)
{
	printk("[ELAN]into elan_iap_open\n");
		if (private_ts == NULL)  printk("private_ts is NULL~~~");
		
	return 0;

}

static int elan_iap_release(struct inode *inode, struct file *filp)
{
	pr_green_info("[ELAN]into elan_iap_release");

	return 0;
}

static ssize_t elan_iap_write(struct file *filp, const char *buff, size_t count, loff_t *offp)
{
    int ret;
    char *tmp;

    /*++++i2c transfer start+++++++*/    	
    struct i2c_adapter *adap = private_ts->client->adapter;    	
    struct i2c_msg msg;
    /*++++i2c transfer end+++++++*/	

    printk("[ELAN]into elan_iap_write\n");
    if (count > 8192)
        count = 8192;

    tmp = kmalloc(count, GFP_KERNEL);
    
    if (tmp == NULL)
        return -ENOMEM;

    if (copy_from_user(tmp, buff, count)) {
        return -EFAULT;
    }

/*++++i2c transfer start+++++++*/
	//down(&worklock);
	msg.addr = file_fops_addr;
	msg.flags = 0x00;// 0x00
	msg.len = count;
	msg.buf = (char *)tmp;
	//up(&worklock);
	ret = i2c_transfer(adap, &msg, 1);
/*++++i2c transfer end+++++++*/

    //if (ret != count) printk("ELAN i2c_master_send fail, ret=%d \n", ret);
    kfree(tmp);
    //return ret;
    return (ret == 1) ? count : ret;
}

static ssize_t elan_iap_read(struct file *filp, char *buff, size_t count, loff_t *offp)
{
    char *tmp;
    int ret;  
    long rc;
   /*++++i2c transfer start+++++++*/
    	struct i2c_adapter *adap = private_ts->client->adapter;
    	struct i2c_msg msg;
/*++++i2c transfer end+++++++*/
    printk("[ELAN]into elan_iap_read\n");
    if (count > 8192)
        count = 8192;

    tmp = kmalloc(count, GFP_KERNEL);

    if (tmp == NULL)
        return -ENOMEM;
/*++++i2c transfer start+++++++*/
#if 1
	//down(&worklock);
	msg.addr = file_fops_addr;
	//msg.flags |= I2C_M_RD;
	msg.flags = 0x00;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = tmp;
	//up(&worklock);
	ret = i2c_transfer(adap, &msg, 1);
#else
    ret = i2c_master_recv(private_ts->client, tmp, count);
#endif
/*++++i2c transfer end+++++++*/
    if (ret >= 0)
        rc = copy_to_user(buff, tmp, count);
    
    kfree(tmp);

    //return ret;
    return (ret == 1) ? count : ret;
}


static struct file_operations elan_touch_fops = {
	.owner	 = THIS_MODULE,
	.open	 = elan_iap_open,
	.write	 = elan_iap_write,
	.read    = elan_iap_read,
	.release = elan_iap_release,
};

// End Firmware Update

 
#if IAP_PORTION
int EnterISPMode(struct i2c_client *client, uint8_t  *isp_cmd)
{
	char buff[4] = {0};
	int len = 0;

#if 0
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
   	 mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	msleep(10);

     	 mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
     	 mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
      	 mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(10);
#endif	
	len = i2c_master_send(private_ts->client, isp_cmd,  sizeof(isp_cmd));
	if (len != sizeof(buff)) {
		printk("[ELAN] ERROR: EnterISPMode fail! len=%d\r\n", len);
		return -1;
	}
	else
		printk("[ELAN] IAPMode write data successfully! cmd = [%2x, %2x, %2x, %2x]\n", isp_cmd[0], isp_cmd[1], isp_cmd[2], isp_cmd[3]);
	return 0;
}

int ExtractPage(struct file *filp, uint8_t * szPage, int byte)
{
	int len = 0;

	len = filp->f_op->read(filp, szPage,byte, &filp->f_pos);
	if (len != byte) 
	{
		printk("[ELAN] ExtractPage ERROR: read page error, read error. len=%d\r\n", len);
		return -1;
	}

	return 0;
}

int WritePage(uint8_t * szPage, int byte)
{
	int len = 0;

	len = i2c_master_send(private_ts->client, szPage,  byte);
	if (len != byte) 
	{
		printk("[ELAN] ERROR: write page error, write error. len=%d\r\n", len);
		return -1;
	}

	return 0;
}

int GetAckData(struct i2c_client *client)
{
	int len = 0;

	char buff[2] = {0};
	
	len=i2c_master_recv(private_ts->client, buff, sizeof(buff));
	if (len != sizeof(buff)) {
		printk("[ELAN] ERROR: read data error, write 50 times error. len=%d\r\n", len);
		return -1;
	}

	pr_info("[ELAN] GetAckData:%x,%x\n",buff[0],buff[1]);
	if (buff[0] == 0xaa/* && buff[1] == 0xaa*/) 
		return ACK_OK;
	else if (buff[0] == 0x55 && buff[1] == 0x55)
		return ACK_REWRITE;
	else
		return ACK_Fail;

	return 0;
}

void print_progress(int page, int ic_num, int j)
{
	int i, percent,page_tatol,percent_tatol;
	char str[256];
	str[0] = '\0';
	for (i=0; i<((page)/10); i++) {
		str[i] = '#';
		str[i+1] = '\0';
	}
	
	page_tatol=page+249*(ic_num-j);
	percent = ((100*page)/(249));
	percent_tatol = ((100*page_tatol)/(249*ic_num));

	if ((page) == (249))
		percent = 100;

	if ((page_tatol) == (249*ic_num))
		percent_tatol = 100;		

	printk("\rprogress %s| %d%%", str, percent);
	
	if (page == (249))
		printk("\n");
}
/* 
* Restet and (Send normal_command ?)
* Get Hello Packet
*/
int IAPReset(struct i2c_client *client)
{
	int res;

	ektf2040_reset();

	printk("[ELAN] read Hello packet data!\n"); 	  
	res= __hello_packet_handler(client);
	return res;
}

/* Check Master & Slave is "55 aa 33 cc" */
int CheckIapMode(void)
{
	char buff[4] = {0},len = 0;
	//WaitIAPVerify(1000000);
	//len = read(fd, buff, sizeof(buff));
	len=i2c_master_recv(private_ts->client, buff, sizeof(buff));
	if (len != sizeof(buff)) 
	{
		printk("[ELAN] CheckIapMode ERROR: read data error,len=%d\r\n", len);
		return -1;
	}
	else
	{
		if (buff[0] == 0x55 && buff[1] == 0xaa && buff[2] == 0x33 && buff[3] == 0xcc)
		{
			//printk("[ELAN] CheckIapMode is 55 aa 33 cc\n");
			return 0;
		}
		else// if ( j == 9 )
		{
			printk("[ELAN] Mode= 0x%x 0x%x 0x%x 0x%x\r\n", buff[0], buff[1], buff[2], buff[3]);
			printk("[ELAN] ERROR:  CheckIapMode error\n");
			return -1;
		}
	}
	printk("\n");	
}

int Update_FW_One(struct i2c_client *client, int recovery)
{
	int res = 0,ic_num = 1;
	int iPage = 0, rewriteCnt = 0; //rewriteCnt for PAGE_REWRITE
	int i = 0;
	uint8_t data;
	private_ts = i2c_get_clientdata(client);
	
	//client->addr=0x2a;
	//client->addr |= I2C_ENEXT_FLAG;

	int restartCnt = 0, checkCnt = 0; // For IAP_RESTART
	//uint8_t recovery_buffer[4] = {0};
	int byte_count;
	uint8_t *szBuff = NULL;
	int curIndex = 0;
#ifdef CMD_54001234
	uint8_t isp_cmd[] = {0x54, 0x00, 0x12, 0x34};	 //54 00 12 34
#else
	uint8_t isp_cmd[] = {0x45, 0x49, 0x41, 0x50};	 //45 49 41 50
#endif


IAP_RESTART:	

	data=I2C_DATA[0];//Master
	dev_dbg(&client->dev, "[ELAN] %s: address data=0x%x \r\n", __func__, data);

	if(recovery != 0x80)
	{
		printk("[ELAN] Firmware upgrade normal mode !\n");

		res = EnterISPMode(private_ts->client, isp_cmd);	 //enter ISP mode

	//res = i2c_master_recv(private_ts->client, recovery_buffer, 4);   //55 aa 33 cc 
	//printk("[ELAN] recovery byte data:%x,%x,%x,%x \n",recovery_buffer[0],recovery_buffer[1],recovery_buffer[2],recovery_buffer[3]);			

		mdelay(10);
	#if 1
		//Check IC's status is IAP mode(55 aa 33 cc) or not
		res = CheckIapMode();	 //Step 1 enter ISP mode
		if (res == -1) //CheckIapMode fail
		{	
			checkCnt ++;
			if (checkCnt >= 5)
			{
				printk("[ELAN] ERROR: CheckIapMode %d times fails!\n", IAPRESTART);
				return E_FD;
			}
			else
			{
				printk("[ELAN] CheckIapMode retry %dth times! And restart IAP~~~\n\n", checkCnt);
				goto IAP_RESTART;
			}
		}
		else
			printk("[ELAN]  CheckIapMode ok!\n");
#endif
	} 
	else
		printk("[ELAN] Firmware upgrade recovery mode !\n");
	// Send Dummy Byte	
	printk("[ELAN] send one byte data:%x,%x",private_ts->client->addr,data);
	res = i2c_master_send(private_ts->client, &data,  sizeof(data));
	if(res!=sizeof(data))
	{
		printk("[ELAN] dummy error code = %d\n",res);
	}	
	mdelay(50);


	// Start IAP
	for( iPage = 1; iPage <= PageNum; iPage++ ) 
	{
PAGE_REWRITE:
#if 1 
		// 8byte mode
		//szBuff = fw_data + ((iPage-1) * PageSize); 
		for(byte_count=1;byte_count<=17;byte_count++)
		{
			if(byte_count!=17)
			{		
	//			printk("[ELAN] byte %d\n",byte_count);	
	//			printk("curIndex =%d\n",curIndex);
				szBuff = file_fw_data + curIndex;
				curIndex =  curIndex + 8;

				//ioctl(fd, IOCTL_IAP_MODE_LOCK, data);
				res = WritePage(szBuff, 8);
			}
			else
			{
	//			printk("byte %d\n",byte_count);
	//			printk("curIndex =%d\n",curIndex);
				szBuff = file_fw_data + curIndex;
				curIndex =  curIndex + 4;
				//ioctl(fd, IOCTL_IAP_MODE_LOCK, data);
				res = WritePage(szBuff, 4); 
			}
		} // end of for(byte_count=1;byte_count<=17;byte_count++)
#endif 
#if 0 // 132byte mode		
		szBuff = file_fw_data + curIndex;
		curIndex =  curIndex + PageSize;
		res = WritePage(szBuff, PageSize);
#endif

#if 1
		if(iPage==249 || iPage==1)
		{
			mdelay(300); 			  
		}
		else
		{
			mdelay(50); 			 
		}
#endif	
		res = GetAckData(private_ts->client);

		if (ACK_OK != res) 
		{
			mdelay(50); 
			printk("[ELAN] ERROR: GetAckData fail! res=%d\r\n", res);
			if ( res == ACK_REWRITE ) 
			{
				rewriteCnt = rewriteCnt + 1;
				if (rewriteCnt == PAGERETRY)
				{
					printk("[ELAN] ID 0x%02x %dth page ReWrite %d times fails!\n", data, iPage, PAGERETRY);
					return E_FD;
				}
				else
				{
					printk("[ELAN] ---%d--- page ReWrite %d times!\n",  iPage, rewriteCnt);
					curIndex = curIndex - PageSize;
					goto PAGE_REWRITE;
				}
			}
			else
			{
				restartCnt = restartCnt + 1;
				if (restartCnt >= 5)
				{
					printk("[ELAN] ID 0x%02x ReStart %d times fails!\n", data, IAPRESTART);
					return E_FD;
				}
				else
				{
					printk("[ELAN] ===%d=== page ReStart %d times!\n",  iPage, restartCnt);
					goto IAP_RESTART;
				}
			}
		}
		else
		{       printk("  data : 0x%02x ",  data);  
			rewriteCnt=0;
			print_progress(iPage,ic_num,i);
		}

		mdelay(10);
	} // end of for(iPage = 1; iPage <= PageNum; iPage++)

	if (IAPReset(client) > 0)
		printk("[ELAN] Update ALL Firmware successfully!\n");
	return 0;
}

#endif





/********************************** Sysfs interface ********************************************/
static ssize_t elan_ktf2k_gpio_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;

	ret = get_interrupt_level(private_ts->intr_gpio);
	sprintf(buf, "GPIO_TP_INT_N=%d\n", ret);

	return (strlen(buf) + 1);
}
static DEVICE_ATTR(gpio, S_IRUGO, elan_ktf2k_gpio_show, NULL);

static ssize_t elan_ktf2k_vendor_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct elan_ktf2k_ts_data *ts = private_ts;
	ssize_t ret = 0;

	sprintf(buf, "%s_x%4.4x\n", "ELAN_KTF2K", ts->fw_ver);
	ret = strlen(buf) + 1;
	return ret;
}
static DEVICE_ATTR(vendor, S_IRUGO, elan_ktf2k_vendor_show, NULL);

static struct kobject *android_touch_kobj;
static int elan_ktf2k_touch_sysfs_init(void)
{
	int ret ;

	android_touch_kobj = kobject_create_and_add("android_touch", NULL) ;
	if (android_touch_kobj == NULL) {
		printk(KERN_ERR "[elan]%s: subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		goto kobject_create_failed;
	}

	ret = sysfs_create_file(android_touch_kobj, &dev_attr_gpio.attr);
	if (ret) {
		printk(KERN_ERR "[elan]%s: sysfs_create_file failed\n", __func__);
		goto create_irq_gpio_sysfs_failed;
	}

	ret = sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr);
	if (ret) {
		printk(KERN_ERR "[elan]%s: sysfs_create_group failed\n", __func__);
		goto create_firmware_vendor_sysfs_failed;
	}

	return 0;

create_firmware_vendor_sysfs_failed:
	sysfs_remove_file(android_touch_kobj, &dev_attr_gpio.attr);
create_irq_gpio_sysfs_failed:
	kobject_put(android_touch_kobj);
kobject_create_failed:
	return ret;
}

static void elan_touch_sysfs_deinit(void)
{
	sysfs_remove_file(android_touch_kobj, &dev_attr_vendor.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_gpio.attr);
	kobject_del(android_touch_kobj);
}
/*******************************End Sysfs interface*********************************************/

static int elan_ktf2k_ts_poll(struct i2c_client *client)
{
	int status = 1, retry = 10;

	do {
		status = get_interrupt_level(private_ts->intr_gpio);
		//printk("[%s] int pin %d, value %d\n", __func__, private_ts->intr_gpio, status);
		retry--;
		mdelay(20);
	} while ((status != 0) && (retry > 0));

	return status == 0 ? 0 : -ETIMEDOUT;
}

static int elan_ktf2k_ts_get_data(struct i2c_client *client, uint8_t *cmd, uint8_t *buf, size_t size)
{
	int rc;

	if (buf == NULL)
		return -EINVAL;

	if ((i2c_master_send(client, cmd, 4)) != 4) {
		return -EFAULT;
	}

	rc = elan_ktf2k_ts_poll(client);
	if (rc < 0) {
		return -EINVAL;
	}

	if ((i2c_master_recv(client, buf, size) != size) || (buf[0] != CMD_S_PKT)) {
		
		return -EFAULT;
	}

	return 0;
}

static int __hello_packet_handler(struct i2c_client *client)
{
	uint8_t buf_recv[4] = {0};
	uint8_t buf_recv1[4] = {0};
	int rc;

	rc = elan_ktf2k_ts_poll(client);
	if (rc < 0) {
		pr_red_info("poll failed in %s: %d", __func__, __LINE__);
		return -EINVAL;
	}

	rc = i2c_master_recv(client, buf_recv, 4);
	if (rc < 0) {
		pr_red_info("Read hello packet failed");
		return -EFAULT;
	}
	pr_green_info("hello packet %2x:%2X:%2x:%2x", buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);

	if ((buf_recv[0] == 0x55) && (buf_recv[1] == 0x55) && (buf_recv[2] == 0x80) && (buf_recv[3] == 0x80)) {
		rc = elan_ktf2k_ts_poll(client); //No send but poll???????
		if (rc < 0) {
			pr_red_info("poll failed in %s: %d", __func__, __LINE__);
			return -EINVAL;
		}

		rc = i2c_master_recv(client, buf_recv1, 4);
		if (rc < 0) {
			pr_red_info("i2c_master_recv failed in %s: %d", __func__, __LINE__);
			return -EFAULT;
		}
		pr_green_info("%s: recovery hello packet %2x:%2X:%2x:%2x\n", __func__, buf_recv1[0], buf_recv1[1], buf_recv1[2], buf_recv1[3]);
		RECOVERY = 0x80;
		return RECOVERY;
	}

	return 0;
}

static int __fw_packet_handler(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int major, minor;
	uint8_t cmd_version[] = {CMD_R_PKT, 0x00, 0x00, 0x01}; /*Get firmware version*/
	uint8_t cmd_x[] = {CMD_R_PKT, 0x60, 0x00, 0x01}; /*Get x resolution*/
	uint8_t cmd_y[] = {CMD_R_PKT, 0x63, 0x00, 0x01}; /*Get y resolution*/
	uint8_t cmd_id[] = {CMD_R_PKT, 0xf0, 0x00, 0x01}; /*Get firmware ID*/
	uint8_t buf_recv[4] = {0};
	int rc;

// Firmware version
	rc = elan_ktf2k_ts_get_data(client, cmd_version, buf_recv, 4);
	if (rc < 0)  return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_ver = major << 8 | minor;
	FW_VERSION = ts->fw_ver;

// X Resolution
	rc = elan_ktf2k_ts_get_data(client, cmd_x, buf_recv, 4);
	if (rc < 0) return rc;

	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	ts->x_resolution = minor;
	X_RESOLUTION = ts->x_resolution;

// Y Resolution
	rc = elan_ktf2k_ts_get_data(client, cmd_y, buf_recv, 4);
	if (rc < 0)
		return rc;

	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	ts->y_resolution = minor;
	Y_RESOLUTION = ts->y_resolution;

// Firmware ID
	rc = elan_ktf2k_ts_get_data(client, cmd_id, buf_recv, 4);
	if (rc < 0)
		return rc;

	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_id = major << 8 | minor;
	FW_ID = ts->fw_id;

	pr_green_info("%s: firmware version: 0x%4.4x", __func__, ts->fw_ver);
	pr_green_info("%s: firmware ID: 0x%4.4x", __func__, ts->fw_id);
	pr_green_info("%s: x max resolution: %d, y max resolution: %d", __func__, ts->x_resolution, ts->y_resolution);

	return 0;
}

static int elan_ktf2k_ts_setup(struct i2c_client *client)
{
	int rc;

	rc = __hello_packet_handler(client);
	if (rc < 0) {
		pr_red_info("receive hello packet failed!");
		goto hand_shake_failed;
	}

	if (rc == 0x80) {
		pr_red_info("no firmware!!!!!!!!!!!!!!!!!");
		return rc;
	}

	rc = __fw_packet_handler(client);
	if (rc < 0) {
		goto hand_shake_failed;
	}
	pr_green_info("%s: firmware checking done.", __func__);

	return 0;

hand_shake_failed:
	return rc;
}

static int elan_ktf2k_ts_set_power_state(struct i2c_client *client, int state)
{
	uint8_t cmd[] = {CMD_W_PKT, 0x50, 0x00, 0x01};

	cmd[1] |= (state << 3);

	pr_green_info("dump cmd: %02x, %02x, %02x, %02x", cmd[0], cmd[1], cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		pr_red_info("%s: i2c_master_send failed", __func__);
		return -EINVAL;
	}

	return 0;
}

static int elan_ktf2k_ts_get_power_state(struct i2c_client *client)
{
	uint8_t cmd[] = {CMD_R_PKT, 0x50, 0x00, 0x01};
	uint8_t buf[4], power_state;
	int rc = 0;

	rc = elan_ktf2k_ts_get_data(client, cmd, buf, 4);
	if (rc < 0) {
		pr_red_info("get data failed in elan_get_power_state!");
		return rc;
	}

	power_state = buf[1];
	pr_green_info("buf[1] : 0x%x", power_state);
	power_state = (power_state & PWR_STATE_MASK) >> 3;
	pr_green_info("power_state_flag : %d", power_state);

	return power_state;
}

static void show_buff(const void *buff, size_t size)
{
	const void *buff_end;

	printk("\033[1mbuff = ");

	for (buff_end = buff + size; buff < buff_end; buff++)
	{
		printk("%02x ", *(u8 *)buff);
	}

	printk("\033[0m\n");
}

static void elan_ktf2k_touch_up(struct elan_ktf2k_ts_data *ts)
{
	struct input_dev *ts_input = ts->input_dev;
	int i;

	for (i = 0; i < ARRAY_SIZE(ektf2040_key_map); i++) {
		input_event(ts_input, EV_KEY, ektf2040_key_map[i], 0);
	}

	input_mt_sync(ts_input);
	input_sync(ts_input);
}

static int elan_ktf2k_recv_normal_packet(struct i2c_client *client, uint8_t *buf)
{

	int rc, normal_pack_size = PACKET_SIZE;

	if (buf == NULL)
		return -EINVAL;

	rc = i2c_master_recv(client, buf, normal_pack_size);
	if (rc != normal_pack_size) {
		pr_red_info("[elan] %s: i2c_master_recv error", __func__);
		rc = i2c_master_recv(client, buf, normal_pack_size);
		return -EINVAL;
	}

	return rc;
}

static inline int elan_ktf2k_ts_parse_xy(uint8_t *data,
			uint16_t *x, uint16_t *y)
{
	*x = *y = 0;

	*x = (data[0] & 0xf0);
	*x <<= 4;
	*x |= data[1];

	*y = (data[0] & 0x0f);
	*y <<= 8;
	*y |= data[2];

	return 0;
}

#ifdef SOFTKEY_AXIS_VER //SOFTKEY is reported via AXI
static void elan_ktf2k_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	
	struct input_dev *idev = ts->input_dev;
	uint16_t x, y;
	uint16_t fbits=0;
	uint8_t i, num, reported = 0;
	uint8_t idx;
	int finger_num;
	int limitY = Y_RESOLUTION-100;//Y_RESOLUTION - 100; // limitY need define by Case!

// for 5 fingers	
	if ((buf[0] == NORMAL_PKT) || (buf[0] == FIVE_FINGERS_PKT))
	{
	    	finger_num = 5;
	    	num = buf[1] & 0x07; 
        	fbits = buf[1] >>3;
	    	idx=2;
	}
	else
	{
// for 2 fingers      
		finger_num = 2;
		num = buf[7] & 0x03; 
		fbits = buf[7] & 0x03;
		idx=1;
	}
//printk("%s,%d \n",__func__,__LINE__);		
	switch (buf[0]) {
		case NORMAL_PKT:
		case TWO_FINGERS_PKT:
		case FIVE_FINGERS_PKT:	
		case TEN_FINGERS_PKT:
			printk("%s,%d \n",__func__,__LINE__);
			input_report_key(idev, BTN_TOUCH, 1);
			if (num==0)
			{	
				dev_dbg(&client->dev, "no press\n");
				
				if(key_pressed < 0){
					input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 0);
					input_report_abs(idev, ABS_MT_WIDTH_MAJOR, 0);
					input_mt_sync(idev);
				}
				else{
					dev_err(&client->dev, "[elan] KEY_RELEASE: key_code:%d\n",OSD_mapping[key_pressed].key_event);
					printk("[elan] %d KEY_UP: key_code:%d\n", key_pressed, OSD_mapping[key_pressed].key_event);
					input_report_key(idev, OSD_mapping[key_pressed].key_event, 0);
					key_pressed = -1;
				}
			}
			else 
			{	
				dev_dbg(&client->dev, "[elan] %d fingers\n", num);                        
				input_report_key(idev, BTN_TOUCH, 1);
				for (i = 0; i < finger_num; i++) 
				{	
					if ((fbits & 0x01)) 
					{
						elan_ktf2k_ts_parse_xy(&buf[idx], &x, &y);  
						x = x * TS_X_MAX/X_RESOLUTION;
						printk("x=%d  \n",x);
						if(y>limitY)
						  {
						  if(buf[18]==0x81)
						  	{
						  	printk("(buf[18]==0x81)");
						  	x=60;
							y=820;
						  	}
						  else 
						  	//if(buf[18]==0x41)
						  	{
						  	printk("(buf[18]==0x41)");
						  	x=120;
							y=820;						  	
						  	}
						   }
						else
						// y = y * TS_Y_MAX/Y_RESOLUTION;
						 y = y * TS_Y_MAX/ELAN_Y_MAX;
						if (!((x<=0) || (y<=0) || (x>=TS_X_MAX) || (y>(TS_Y_MAX+20)))) 
						{   
							if ( y < TS_Y_MAX )
						     	{
								printk("if ( y < TS_Y_MAX )  [11111111elan_debug] %s, x=%d, y=%d, limitY=%d \n",__func__, x , y, limitY);
							    input_report_key(idev, BTN_TOUCH, 1);
							  //input_report_abs(data->input_dev,ABS_MT_TRACKING_ID,i);
							    input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 1);
							    input_report_abs(idev, ABS_MT_POSITION_X, x);
							    input_report_abs(idev, ABS_MT_POSITION_Y, y);
							    input_mt_sync(idev);								
							}
							else
							{
								printk("else ( y < TS_Y_MAX ) [11111111elan_debug] %s, x=%d, y=%d, limitY=%d \n",__func__, x , y, limitY);
			    					input_report_abs(idev, ABS_MT_TRACKING_ID, i);
								input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 8);
								input_report_abs(idev, ABS_MT_POSITION_X, x);
								input_report_abs(idev, ABS_MT_POSITION_Y, y);
								input_mt_sync(idev);								
							}
							reported++;
							
					  	} // end if border
					} // end if finger status
				  	fbits = fbits >> 1;
				  	idx += 3;
				} // end for
			}

			if (reported)
				{
				printk("reported>=1");
				input_sync(idev);
				}
			else 
			{
				printk("reported<1");
				input_mt_sync(idev);
				input_sync(idev);
			}

			break;
	   	default:
				dev_err(&client->dev,
					"[elan] %s: unknown packet type: %0x\n", __func__, buf[0]);
				break;
	} // end switch
	return;
}
#else //SOFTKEY is reported via BTN bit
static void elan_ktf2k_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *idev = ts->input_dev;
	uint16_t x, y;
	uint16_t fbits=0;
	uint8_t i, num, reported = 0;
	uint8_t idx;
	int finger_num;

// for 5 fingers	
	if ((buf[0] == NORMAL_PKT) || (buf[0] == FIVE_FINGERS_PKT)){
	    	finger_num = 5;
	    	num = buf[1] & 0x07; 
        	fbits = buf[1] >>3;
	    	idx=2;
	}else{
// for 2 fingers      
		finger_num = 2;
		num = buf[7] & 0x03; 
		fbits = buf[7] & 0x03;
		idx=1;
	}
		
	switch (buf[0]) {
		case NORMAL_PKT:
		case TWO_FINGERS_PKT:
		case FIVE_FINGERS_PKT:	
		case TEN_FINGERS_PKT:
			//input_report_key(idev, BTN_TOUCH, 1);
			if (num == 0)
			{
				dev_dbg(&client->dev, "no press\n");
				printk("tp button_state0 = %x\n",button_state);
             			printk("tp buf[BUTTON_ID_INDEX] = %x KEY_MENU=%x KEY_HOME=%x KEY_BACK=%x KEY_SEARCH =%x\n",buf[BUTTON_ID_INDEX], KEY_MENU, KEY_HOME, KEY_BACK, KEY_SEARCH);
#ifdef ELAN_BUTTON
				if (buf[BUTTON_ID_INDEX] == ELAN_KEY_MENU) 
				{
					button_state = ELAN_KEY_MENU;
					input_report_key(idev,ANDROID_KEY_MENU,1);
					input_report_key(idev,ANDROID_KEY_MENU,0);
					printk("[elan]menu key is press ELAN_KEY_MENU = %x\n",ELAN_KEY_MENU);

				/*	button_state = ELAN_KEY_MENU;
				
				#if 0//defined(MACRO_LCD_SIZE_HVGA)
					x = 40;
					y = 500;
				#else
					x = 60;
					//y = 820;
					y = 884;
				#endif

				    input_report_abs(idev, ABS_PRESSURE,255);
				    input_report_key(idev, BTN_TOUCH, 1);
				    input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 1);
				    input_report_abs(idev, ABS_MT_POSITION_X, x);
				    input_report_abs(idev, ABS_MT_POSITION_Y, y);
				*/
				} 
				#if  0
				else if (buf[BUTTON_ID_INDEX] == ELAN_KEY_HOME) 
				{
					button_state = ELAN_KEY_HOME;
				#if 0 //defined(MACRO_LCD_SIZE_HVGA)
					x = 120;
					y = 500;
				#else
					x = 180;
					//y = 820;
					y = 884;
				#endif
				    input_report_abs(idev, ABS_PRESSURE,255);
				    input_report_key(idev, BTN_TOUCH, 1);
				    input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 1);
				    input_report_abs(idev, ABS_MT_POSITION_X, x);
				    input_report_abs(idev, ABS_MT_POSITION_Y, y);
				} 
				#endif	
				else if (buf[BUTTON_ID_INDEX] == ELAN_KEY_BACK) 
				{
					button_state = ELAN_KEY_BACK;
					input_report_key(idev,ANDROID_KEY_BACK,1);
					input_report_key(idev,ANDROID_KEY_BACK,0);
				/*	button_state = ELAN_KEY_BACK;
				#if 0//defined(MACRO_LCD_SIZE_HVGA)
					x = 200;
					y = 500;
				#else
					x = 420;
					//y = 820;
					y = 884;
				#endif
				    input_report_abs(idev, ABS_PRESSURE,255);
				    input_report_key(idev, BTN_TOUCH, 1);
				    input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 1);
				    input_report_abs(idev, ABS_MT_POSITION_X, x);
				    input_report_abs(idev, ABS_MT_POSITION_Y, y);
				*/
				} 
				#if  0
				else if (buf[BUTTON_ID_INDEX] == ELAN_KEY_SEARCH) 
				{
					button_state = ELAN_KEY_SEARCH;
				#if defined(MACRO_LCD_SIZE_HVGA)
					x = 280;
					y = 500;
				#else
					x = 420;
					y = 820;
				#endif
				    input_report_abs(idev, ABS_PRESSURE,255);
				    input_report_key(idev, BTN_TOUCH, 1);
				    input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 1);
				    input_report_abs(idev, ABS_MT_POSITION_X, x);
				    input_report_abs(idev, ABS_MT_POSITION_Y, y);
				}
				#endif
				else // TOUCH release
				{	
					dev_dbg(&client->dev, "no press\n");
					input_report_key(idev, BTN_TOUCH, 0);
					input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 0);
					input_report_abs(idev, ABS_MT_WIDTH_MAJOR, 0);
					input_mt_sync(idev);
				}
#endif		      
				printk("tp button_state1 = %x\n",button_state);
			}
			else 
			{			
				//dev_dbg(&client->dev, "[elan] %d fingers\n", num);                        
				input_report_key(idev, BTN_TOUCH, 1);
				for (i = 0; i < finger_num; i++) 
				{	
					if ((fbits & 0x01)) 
					{
						elan_ktf2k_ts_parse_xy(&buf[idx], &x, &y);  
						//elan_ktf2k_ts_parse_xy(&buf[idx], &y, &x);  		 
						
						x=x*480/733;
						y=y*854/1280;
						//printk("[elan_debug] %s, x=%d, y=%d\n",__func__, x , y);
						//x = X_RESOLUTION-x;	 
						//y = Y_RESOLUTION-y;			     
						//if (!((x<=0) || (y<=0) || (x>=X_RESOLUTION) || (y>=Y_RESOLUTION))) 
						if(x<=TS_X_MAX&&y<=TS_Y_MAX)
						{   
							input_report_abs(idev, ABS_MT_TRACKING_ID, i);
							input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 8);
							
							input_report_abs(idev, ABS_MT_POSITION_X, TS_X_MAX-x);
							input_report_abs(idev, ABS_MT_POSITION_Y, TS_Y_MAX-y);
							input_mt_sync(idev);
							reported++;
					  	} // end if border
					} // end if finger status
				  	fbits = fbits >> 1;
				  	idx += 3;
				} // end for
			}


			
			if (reported)
				input_sync(idev);
			else 
			{
				input_mt_sync(idev);
				input_sync(idev);
			}
			break;
	   	default:
				dev_err(&client->dev,
					"[elan] %s: unknown packet type: %0x\n", __func__, buf[0]);
				break;
	} // end switch
	return;
}
#endif



static int elan_ktf2k_ts_recv_data(struct i2c_client *client, uint8_t *buf)
{

	int rc, bytes_to_recv=PACKET_SIZE;
	if (buf == NULL)
		return -EINVAL;

	memset(buf, 0, bytes_to_recv);
	
	rc = i2c_master_recv(client, buf, 8);
	//printk("[elan_recv] %x %x %x %x %x %x %x %x\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
	//printk("[elan_recv] %x %x %x %x %x %x %x %x\n", buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);
	//printk("[elan_recv] %x %x \n", buf[16], buf[17]);
	/*
	if (rc != 8)
		printk("[elan_debug] The first package error.\n");
		printk("[elan_recv] %x %x %x %x %x %x %x %x\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
	mdelay(1);
	
        if (buf[0] == 0x5D){    //for five finger
	      rc = i2c_master_recv(client, buf+ 8, 8);	
	      if (rc != 8)
		printk("[elan_debug] The second package error.\n");
printk("[elan_recv] %x %x %x %x %x %x %x %x\n", buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);
	        rc = i2c_master_recv(client, buf+ 16, 2);
               if (rc != 2)
		    printk("[elan_debug] The third package error.\n");
	       mdelay(1);
printk("[elan_recv] %x %x \n", buf[16], buf[17]);
        }
        */
	return rc;
}

static void elan_ktf2k_ts_work_func(struct work_struct *work)
{
	struct elan_ktf2k_ts_data *ts = container_of(work, struct elan_ktf2k_ts_data, work);
	uint8_t buf[PACKET_SIZE] = {0};
	int rc;

	if (get_interrupt_level(ts->intr_gpio)) {
		//pr_red_info("irq pin is high, seems it's a fake irq");
		enable_irq(ts->intr_gpio);
		return;
	}
	
		rc = elan_ktf2k_ts_recv_data(ts->client, buf);
 
		if (rc < 0)
		{
			enable_irq(ts->client->irq);
			return;
		}

		//printk("[elan] %2x,%2x,%2x,%2x,%2x,%2x\n",buf[0],buf[1],buf[2],buf[3],buf[5],buf[6]);
		elan_ktf2k_ts_report_data(ts->client, buf);
	enable_irq(ts->intr_gpio);
}

static irqreturn_t elan_ktf2k_ts_irq_handler(int irq, void *dev_id)
{
	struct elan_ktf2k_ts_data *ts = dev_id;

	//pr_green_info("[elan] %s", __func__);
	disable_irq_nosync(ts->intr_gpio);
	queue_work(ts->elan_wq, &ts->work);

	return IRQ_HANDLED;
}

static int elan_ktf2k_ts_register_interrupt(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int err = 0;

	err = request_irq(ts->intr_gpio, elan_ktf2k_ts_irq_handler, IRQF_TRIGGER_LOW, client->name, ts);
	if (err) {
		pr_red_info("[elan] %s: request_irq %d failed", __func__, ts->intr_gpio);
		return err;
	}

	return 0;
}


#if 1 //def VIRTUAL_KEYS
static ssize_t virtual_keys_show(struct kobject *kobj,
             struct kobj_attribute *attr, char *buf)
{
    if (1) {
#if defined(CONFIG_LCD_SIZE_HVGA)

	return sprintf(buf,
	 __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":40:500:20:15"
	 ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME) ":120:500:20:15"
	 ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":200:500:20:15"
	 ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":280:500:20:15"
	 "\n");
#else
	return sprintf(buf,
	 __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":60:820:60:15"
	 ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME) ":180:820:60:15"
	 ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":300:820:60:15"
	 ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":420:820:60:15"
	 "\n");

#endif

} else {

    }
}

static struct kobj_attribute virtual_keys_attr = {
    .attr = {
        .name = VIRTUAL_KEY_NAME,
        .mode = S_IRUGO,
    },
    .show = &virtual_keys_show,
};

static struct attribute *properties_attrs[] = {
    &virtual_keys_attr.attr,
    NULL
};

static struct attribute_group properties_attr_group = {
    .attrs = properties_attrs,
};

static void virtual_keys_init(void)
{
    int ret=0;
    
    properties_kobj = kobject_create_and_add("board_properties", NULL);
    if (properties_kobj)
        ret = sysfs_create_group(properties_kobj,&properties_attr_group);
    if (!properties_kobj || ret)
        pr_err("failed to create board_properties\n");    
}

static void virtual_keys_destroy(void)
{
	kobject_del(properties_kobj);
}
#endif


#if defined(TP_DEVICE_TYPE)
static ssize_t tp_type_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s\n", EKTF2040_DEV_NAME);	
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t tp_type_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    	return size;
}

static DEVICE_ATTR(tp_type, S_IRUGO | S_IWUSR, tp_type_show, tp_type_store);

static struct attribute *tp_type_attributes[] = {
	&dev_attr_tp_type.attr,
	NULL
};

static const struct attribute_group tp_type_attr_group = {
	.attrs = tp_type_attributes,
};
#endif

static void input_cleanup(struct elan_ktf2k_ts_data *EK2048_ts)
{
	input_unregister_device(EK2048_ts->input_dev);
	input_free_device(EK2048_ts->input_dev);
}

static int elan_ktf2k_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct elan_ktf2k_ts_data *ts;	
	struct touchscreen_platform_data* pdata = client->dev.platform_data;
	int err;
	int New_FW_ID;	
	int New_FW_VER;

	printk("ELAN probe is here!");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_red_info("ektf2040 i2c_check_functionality failed!");
		err = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct elan_ktf2k_ts_data), GFP_KERNEL);
	if (!ts) {
		pr_red_info("ektf2040 allocate elan_ktf2k_ts_data failed!");
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}

	
	if(pdata->plat_init)
		pdata->plat_init();

	msleep(300);

	ts->intr_gpio= client->irq;
    ts->pdata = pdata;
	ts->elan_wq = create_singlethread_workqueue("elan_wq");
	if (!ts->elan_wq) {
		pr_red_info("ektf2040 create workqueue failed");
		err = -ENOMEM;
		goto err_create_wq_failed;
	}

	INIT_WORK(&ts->work, elan_ktf2k_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);

	private_ts = ts;


	err = elan_ktf2k_ts_setup(client);
	if(err<0)
	{
		pr_red_info("ektf2040 check false");
		err = -ENOMEM;
		goto err_create_wq_failed;
	}		

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		err = -ENOMEM;
		pr_red_info("[elan] Failed to allocate input device");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = INPUT_DEV_NAME;   // for andorid2.2 Froyo

	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, TS_X_MAX, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, TS_Y_MAX, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 1, 0, 0);	

	set_bit(EV_ABS,     ts->input_dev->evbit);
	set_bit(EV_KEY,     ts->input_dev->evbit);
	set_bit(BTN_TOUCH,  ts->input_dev->keybit);
	set_bit(EV_SYN,     ts->input_dev->evbit);

	set_bit(KEY_HOME,  ts->input_dev->keybit);
	set_bit(KEY_MENU,  ts->input_dev->keybit);
	set_bit(KEY_BACK,  ts->input_dev->keybit);
	set_bit(KEY_SEARCH,  ts->input_dev->keybit);

	input_set_capability(ts->input_dev, EV_KEY, KEY_HOME);
	input_set_capability(ts->input_dev, EV_KEY, KEY_MENU);
	input_set_capability(ts->input_dev, EV_KEY, KEY_BACK);
	input_set_capability(ts->input_dev, EV_KEY, KEY_SEARCH);
	err = input_register_device(ts->input_dev);
	
	if (err) {
		pr_red_info("%s: unable to register %s input device", __func__, ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	err = elan_ktf2k_ts_register_interrupt(ts->client);
	if (err < 0) {
		pr_red_info("[elan]%s: request irq failed: %d", __func__, err);
		goto err_request_irq_failed;
	}

	if (get_interrupt_level(ts->intr_gpio) == 0) {
		pr_red_info("[elan]%s: handle missed interrupt", __func__);
		elan_ktf2k_ts_irq_handler(ts->intr_gpio, ts);
	}
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
	ts->early_suspend.suspend = elan_ktf2k_ts_early_suspend;
	ts->early_suspend.resume = elan_ktf2k_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
	err = elan_ktf2k_touch_sysfs_init();
	if (err < 0) {
		pr_red_info("%s: create sysfs failed: %d", __func__, err);
		goto err_sysfs_init_failed;
	}

	pr_green_info("Start touchscreen %s in interrupt mode", ts->input_dev->name);

	// Firmware Update
	ts->firmware.minor = MISC_DYNAMIC_MINOR;
	ts->firmware.name = "elan-iap";
	ts->firmware.mode = S_IFREG | S_IRWXUGO;
	ts->firmware.fops = &elan_touch_fops;

	err = misc_register(&ts->firmware);
	if (err < 0) {
		pr_red_info("Misc register failed!");
		goto err_misc_register_failed;
	}
	// End Firmware Update
	pr_green_info("End of ektf2040 probe!!!!!!!!!!!!!!!!!!!!!!!!!!");

	virtual_keys_init();

	#if defined(TP_DEVICE_TYPE)
		/*create device group in sysfs as user interface */
		err = sysfs_create_group(&ts->input_dev->dev.kobj, &tp_type_attr_group);
		if (err) {
			dev_err(&client->dev, "create device file failed!\n");
			err = -EINVAL;
			goto err_input_cleanup;
		}
	#endif


#if IAP_PORTION
	if(1)
	{
		work_lock=1;
		disable_irq(ts->intr_gpio);
		cancel_work_sync(&ts->work);
		power_lock = 1;
               /* FW ID & FW VER*/
               printk("[ELAN]  [7bd0]=0x%02x,  [7bd1]=0x%02x, [7bd2]=0x%02x, [7bd3]=0x%02x\n",  file_fw_data[31696],file_fw_data[31697],file_fw_data[31698],file_fw_data[31699]);
		New_FW_ID = file_fw_data[31699]<<8  | file_fw_data[31698] ;	       
		New_FW_VER = file_fw_data[31697]<<8  | file_fw_data[31696] ;
	 	printk("[ELAN]  FW_ID=0x%x,   New_FW_ID=0x%x \n",  FW_ID, New_FW_ID);   	       
		printk("[ELAN]  FW_VERSION=0x%x,   New_FW_VER=0x%x \n",  FW_VERSION  , New_FW_VER);  

	       // for firmware auto-upgrade   
		//Update_FW_One(client, RECOVERY); 

 
#if 1      
	      if (New_FW_ID   ==  FW_ID){
		   	if (New_FW_VER > FW_VERSION) 
		                Update_FW_One(client, RECOVERY);
		} else {                        
		                printk("FW_ID is different!");		
		}
        	
		if (FW_ID == 0)  
		{
			RECOVERY=0x80;		   
			printk("[ELAN] RECOVERY=%x \n",RECOVERY);  
			Update_FW_One(client, RECOVERY);
		}
#endif
		work_lock=0;
		enable_irq(ts->intr_gpio);
		//mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

	}
#endif


	return 0;
err_input_cleanup:
	input_cleanup(ts);
	
err_misc_register_failed:
	elan_touch_sysfs_deinit();
err_sysfs_init_failed:
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
	free_irq(ts->intr_gpio, ts);
err_request_irq_failed:
	input_unregister_device(ts->input_dev);
err_input_register_device_failed:
	if (ts->input_dev)
		input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
err_detect_failed:
	if (ts->elan_wq)
		destroy_workqueue(ts->elan_wq);
err_create_wq_failed:
err_alloc_irq_pin:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	pr_red_info("%s: Probe Fail!\n",__func__);
	
	return err;
}

static int elan_ktf2k_ts_remove(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);

	misc_deregister(&ts->firmware);
	elan_touch_sysfs_deinit();
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
	free_irq(ts->intr_gpio, ts);
	input_unregister_device(ts->input_dev);

	if (ts->input_dev)
		input_free_device(ts->input_dev);

	if (ts->elan_wq)
		destroy_workqueue(ts->elan_wq);
	kfree(ts);

	return 0;
}

static int elan_ktf2k_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int rc = 0;
	if(power_lock==0) /* The power_lock can be removed when firmware upgrade procedure will not be enter into suspend mode.  */
	{
		printk(KERN_INFO "[elan] %s: enter\n", __func__);

		disable_irq(client->irq);

		rc = cancel_work_sync(&ts->work);
	//	if (rc)
		//	enable_irq(client->irq);

		//rc = elan_ktf2k_ts_set_power_state(client, PWR_STATE_DEEP_SLEEP);
	}
	return 0;
}

static int elan_ktf2k_ts_resume(struct i2c_client *client)
{
	int rc = 0, retry = 5;
	if(power_lock==0)   /* The power_lock can be removed when firmware upgrade procedure will not be enter into suspend mode.  */
	{
		printk(KERN_INFO "[elan] %s: enter\n", __func__);

		do {
			rc = elan_ktf2k_ts_set_power_state(client, PWR_STATE_NORMAL);
			msleep(100);
			rc = elan_ktf2k_ts_get_power_state(client);
			if (rc != PWR_STATE_NORMAL)
				printk(KERN_ERR "[elan] %s: wake up tp failed! err = %d\n",
					__func__, rc);
			else
				break;
		} while (--retry);

		enable_irq(client->irq);
	}
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void elan_ktf2k_ts_early_suspend(struct early_suspend *h)
{
	struct elan_ktf2k_ts_data *ts = container_of(h, struct elan_ktf2k_ts_data, early_suspend);

	elan_ktf2k_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void elan_ktf2k_ts_late_resume(struct early_suspend *h)
{
	struct elan_ktf2k_ts_data *ts = container_of(h, struct elan_ktf2k_ts_data, early_suspend);

	elan_ktf2k_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id elan_ktf2k_ts_id[] = {
	[0] = {
		.name = EKTF2040_DEV_NAME,
		.driver_data = 0,
	},
	[1] = {}
};

static struct i2c_driver ektf2k_ts_driver = {
	.probe		= elan_ktf2k_ts_probe,
	.remove		= elan_ktf2k_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= elan_ktf2k_ts_suspend,
	.resume		= elan_ktf2k_ts_resume,
#endif
	.id_table	= elan_ktf2k_ts_id,
	.driver		= {
		.owner  = THIS_MODULE,
		.name	= EKTF2040_DEV_NAME,
	},
};


static int __init elan_ktf2k_ts_init(void)
{
	int err = i2c_add_driver(&ektf2k_ts_driver);
	if (err < 0) {
		pr_red_info("i2c_add_driver failed: %s", __func__);
		return err;
	}

	return 0;
}

static void __exit elan_ktf2k_ts_exit(void)
{
	i2c_del_driver(&ektf2k_ts_driver);

}

module_init(elan_ktf2k_ts_init);
module_exit(elan_ktf2k_ts_exit);

MODULE_DESCRIPTION("ELAN EKTF2K Touchscreen Driver");
MODULE_LICENSE("GPL");



