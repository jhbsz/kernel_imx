/* 
 * drivers/input/touchscreen/ft5x0x_ts.c
 *
 * FocalTech ft5x0x TouchScreen driver. 
 *
 * Copyright (c) 2010  Focal tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * VERSION      	DATE			AUTHOR        Note
 *    1.0		  2010-01-05			WenFS    only support mulititouch	Wenfs 2010-10-01
 *    2.0          2011-09-05                   Duxx      Add touch key, and project setting update, auto CLB command
 *     
 *
 */
//#define DEBUG
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/firmware.h>
#include <linux/ft5x0x_ts.h>
#include "ft5x0x_ts.h"

#define CONFIG_FT5X0X_MULTITOUCH 1

#define TP_ID_5X0X 0 //first tp from GW
#define TP_ID_G5     1

struct ts_event {
    u16 au16_x[CFG_MAX_TOUCH_POINTS];              //x coordinate
    u16 au16_y[CFG_MAX_TOUCH_POINTS];              //y coordinate
    u8  au8_touch_event[CFG_MAX_TOUCH_POINTS];     //touch event:  0 -- down; 1-- contact; 2 -- contact
    u8  au8_finger_id[CFG_MAX_TOUCH_POINTS];       //touch ID
	u16	pressure;
    u8  touch_point;
};


struct ft5x0x_ts_data {
	struct input_dev	*input_dev;
	struct ts_event		event;
	struct work_struct 	pen_event_work;
	struct workqueue_struct *ts_workqueue;
	struct early_suspend	early_suspend;
	struct i2c_client *		client;
	
	int irq;
	int x_max,y_max;
	int xy_swap;
	int x_inverted;
	int y_inverted;

	u32 identifer;

	//firmware update	
	#if CFG_SUPPORT_AUTO_UPG 
	struct firmware *firmware;
	#endif

	
};


#if CFG_SUPPORT_TOUCH_KEY
static int tsp_keycodes[CFG_NUMOFKEYS] ={

        KEY_MENU,
        KEY_HOME,
        KEY_BACK,
        KEY_SEARCH
};

static char *tsp_keyname[CFG_NUMOFKEYS] ={

        "Menu",
        "Home",
        "Back",
        "Search"
};

static bool tsp_keystatus[CFG_NUMOFKEYS];
#endif
/***********************************************************************************************
Name	:	ft5x0x_i2c_rxdata 

Input	:	*rxdata
                     *length

Output	:	ret

function	:	

***********************************************************************************************/
static int ft5x0x_i2c_read(struct i2c_client *client,u8 addr,u8 *data, int length)
{
	struct i2c_msg msgs[] = {
	{
		.addr	= client->addr,
		.flags	= 0,
		.len	= 1,
		.buf	= &addr,
	},
	{
		.addr	= client->addr,
		.flags	= I2C_M_RD,
		.len	= length,
		.buf	= data,
	},
	};
	return  i2c_transfer(client->adapter, msgs, 2);
}

static int ft5x0x_i2c_write(struct i2c_client *client,u8 *data, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= data,
		},
	};
	return i2c_transfer(client->adapter, msg, 1);
}

static u8 fts_chip_firmware_ver(struct ft5x0x_ts_data* ft5x0x)
{
	u8 ver;
	ft5x0x_i2c_read(ft5x0x->client,FT5X0X_REG_FIRMID, &ver,1);
	return ver;
}

#if CFG_SUPPORT_AUTO_UPG  //upgrade related
typedef enum
{
    ERR_OK,
    ERR_MODE,
    ERR_READID,
    ERR_ERASE,
    ERR_STATUS,
    ERR_ECC,
    ERR_DL_ERASE_FAIL,
    ERR_DL_PROGRAM_FAIL,
    ERR_DL_VERIFY_FAIL
}E_UPGRADE_ERR_TYPE;



static int fts_read(struct ft5x0x_ts_data* ft5x0x, u8* buf, int length)
{
   return i2c_master_recv(ft5x0x->client, buf,length);
}

static int fts_write(struct ft5x0x_ts_data* fts,u8* buf,int length)
{
  //,u8 ctpm_addr
   return ft5x0x_i2c_write(fts->client,buf,length);
}
static int fts_write_reg(struct ft5x0x_ts_data* fts,u8 reg,u8 val)
{
	u8 buf[2];
	buf[0]=reg;
	buf[1]=val;
	return fts_write(fts,buf,2);
}
static int fts_read_reg(struct ft5x0x_ts_data* fts,u8 reg,u8* val)
{
	return ft5x0x_i2c_read(fts->client,reg,val,1);
}


static int fts_enter_fwupdate_mode(struct ft5x0x_ts_data *ts)
{
    struct device *dev = &ts->client->dev;
	int  retval = 0;
	int  i      = 0;
	u8  temp_buffer[4];

    dev_info(dev, "%s() - Step 1: Reset the CTPM\n", __FUNCTION__);
    retval = i2c_smbus_write_byte_data(ts->client, 0xfc, 0xaa);
    if (0 > retval) {
        dev_err(dev,"%s() - ERROR: Could not write the CTPM Reset command (high byte) to the CTPM.\n", __FUNCTION__);
        goto error_return;
    }

    msleep(50);

    retval = i2c_smbus_write_byte_data(ts->client, 0xfc, 0x55);
    if (0 > retval) {
         dev_err(dev, "%s() - ERROR: Could not write the CTPM Reset command (low byte) to the CTPM.\n", __FUNCTION__);
        goto error_return;
    }

    msleep(30);

    dev_info(dev,"%s() - Step 2: Put the CTPM in Firmware Upgrade mode\n", __FUNCTION__);
    temp_buffer[0] = 0x55;
    temp_buffer[1] = 0xAA;

    for (i = 0; ((i < 5) && (0 >= retval)); i++) {
        struct i2c_msg msg =    {
                                    .addr   = ts->client->addr,
                                    .flags  = 0,
                                    .len    = 2,
                                    .buf    = temp_buffer,
                                };
        msleep(5);

        retval = i2c_transfer(ts->client->adapter, &msg, 1);
    }

    if (0 >= retval) {
         dev_err(dev,"%s() - ERROR: Could not put the CTPM in Firmware Update mode.\n", __FUNCTION__);
        goto error_return;
    }
    return 0;

error_return:
    return -1;
}




static E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(struct ft5x0x_ts_data* ft5x0x ,u8* firmware_src_buffer, u32 firmware_size)
{
#define    PACKET_SIZE        128
	struct device *dev = &ft5x0x->client->dev;
	int i = 0,j=0;
	u32 num_packets = 0;
	u16 temp_val    = 0;
	u8  checksum    = 0;
	u8  packet_buffer[PACKET_SIZE + 6];	
	 u8  temp_buffer[4];	
	int ret;

	/*********Step 1:Reset  CTPM *****/
	dev_info(dev,"[FTS] Step 1: Reset CTPM\n");
	ret = fts_enter_fwupdate_mode(ft5x0x); if(ret) goto err;    
	msleep(300);	


    /*********Step 3:check READ-ID***********************/        
    dev_info(dev,"[FTS]  Step 3: Read the CTPM ID\n");	
    temp_buffer[0] = 0x90;
    temp_buffer[1] = 0x00;
    temp_buffer[2] = 0x00;
    temp_buffer[3] = 0x00;
    ret = fts_write(ft5x0x, temp_buffer, 4);
    if (0 > ret)
    {
        dev_err(dev, "[FTS] - ERROR: Could not write the Get ID command to the CTPM.\n");
        goto err;;
    }
    msleep(100);	
    ret = fts_read(ft5x0x,temp_buffer, 2);
    if (0 > ret)
    {
        ret = ERR_READID;
        dev_err(dev, "[FTS] - ERROR: Could not read the ID from the CTPM.\n");
        goto err;
    }
    if ((0x79 != temp_buffer[0]) || (0x03 != temp_buffer[1]))
    {
        ret = ERR_READID;
         dev_err(dev, "[FTS] - ERROR: Invalid CPTM ID. Expected 0x%02X%02X, got 0x%02X%02X.\n",
		 	0x79, 0x03, temp_buffer[0], temp_buffer[1]);
		goto err;
    }

    temp_buffer[0] = 0xcd;
    temp_buffer[1] = 0x00;
    temp_buffer[2] = 0x00;
    temp_buffer[3] = 0x01;
    ret = fts_write(ft5x0x, temp_buffer, 4);
    if (0 > ret)
    {
        dev_err(dev, "[FTS] - ERROR: Could not write the get bootloader version cmd to CTPM.\n");
        goto err;;
    }
    msleep(100);	
    ret = fts_read(ft5x0x,temp_buffer, 1);
    if (0 > ret)
    {
        ret = ERR_READID;
        dev_err(dev, "[FTS] - ERROR: Could not read the ID from the CTPM.\n");
        goto err;
    }
    dev_info(dev,"[FTS]  bootloader version = 0x%x\n", temp_buffer[0]);
    dev_info(dev, "[FTS]  Step 4: Erase the old CTPM firmware\n");

    temp_buffer[0] = 0x61;
    ret = fts_write(ft5x0x, temp_buffer, 1);
    if (0 > ret)
    {
        dev_err(dev,"[FTS] - ERROR: Could not write the Erase Firmware command to the CTPM.\n");
        goto err;
    }
    msleep(1500);

    temp_buffer[0] = 0x63;
    ret = fts_write(ft5x0x, temp_buffer, 1);
    if (0 > ret)
    {
        dev_err(dev,"[FTS] - ERROR: Could not write the Erase Panel Parameters command to the CTPM.\n");
        goto err;
    }  
    msleep(100);

    /*********Step 5:write firmware(FW) to ctpm flash*********/
    dev_info(dev, "[FTS] Step 5: Write the new CTPM firmware to CTPM flash\n");

    /* We write everything but the last 8 bytes in packets.
     * The first 6 of the last 8 bytes will be written in the footer.
     * The final 2 bytes (which seem to be always 0xFF and 0x00) don't get written.
     */
    firmware_size = firmware_size - 8;
    num_packets   = (firmware_size) / PACKET_SIZE;

    packet_buffer[0] = 0xBF;
    packet_buffer[1] = 0x00;

    /* Write whole packets */
    for (i = 0; i < num_packets; i++)
    {
        /* Target offset */
        temp_val = i * PACKET_SIZE;

        packet_buffer[2] = (u8)(0x00FF & (temp_val >> 8));
        packet_buffer[3] = (u8)(0x00FF & (temp_val));

        /* Num bytes following header */
        temp_val = PACKET_SIZE;

        packet_buffer[4] = (u8)(0x00FF & (temp_val >> 8));
        packet_buffer[5] = (u8)(0x00FF & (temp_val));

        for (j = 0; j < PACKET_SIZE; j++)
        {
            /* Process byte j of packet i... */
            packet_buffer[6 + j] = firmware_src_buffer[(i * PACKET_SIZE) + j];
            checksum ^= packet_buffer[6 + j];
        }

        ret = i2c_master_send(ft5x0x->client, packet_buffer, PACKET_SIZE + 6);
        if (0 > ret)
        {
             dev_err(dev,"[FTS] - ERROR: Could not write packet %u of %u to the CTPM.\n", i, num_packets);
            goto err;
        }

        msleep(20);

        if (0 == ((i * PACKET_SIZE) % 1024))
        {
            dev_info(dev, "[FTS] - Uploaded %6d of %6u bytes.\n", (i * PACKET_SIZE), firmware_size);
        }
    }

    dev_info(dev, "[FTS] - Uploaded %6d of %6u bytes.\n", (i * PACKET_SIZE), firmware_size);

    /* Write a partial packet if necessary */
    if (0 != (firmware_size % PACKET_SIZE))
    {
        /* Target offset */
        temp_val = num_packets * PACKET_SIZE;

        packet_buffer[2] = (u8)(0x00FF & (temp_val >> 8));
        packet_buffer[3] = (u8)(0x00FF & (temp_val));

        /* Num bytes following header */
        temp_val = (firmware_size % PACKET_SIZE);

        packet_buffer[4] = (u8)(0x00FF & (temp_val >> 8));
        packet_buffer[5] = (u8)(0x00FF & (temp_val));

        for (j = 0; j < temp_val; j++)
        {
            packet_buffer[6 + j] = firmware_src_buffer[(num_packets * PACKET_SIZE) + j];
            checksum ^= packet_buffer[6 + j];
        }

        ret = i2c_master_send(ft5x0x->client, packet_buffer, temp_val + 6);
        if (0 > ret)
        {
            dev_err(dev, "[FTS] - ERROR: Could not write partial packet to the CTPM.\n");
            goto err;
        }

        msleep(20);
    }

    dev_info(dev, "[FTS] - Uploaded %6d of %6u bytes.\n", (i * PACKET_SIZE) + temp_val, firmware_size);

    /* Write the firmware footer */
    for (i = 0; i < 6; i++)
    {
        packet_buffer[2] = 0x6F;
        packet_buffer[3] = 0xFA + i;
        packet_buffer[4] = 0x00;
        packet_buffer[5] = 0x01;

        packet_buffer[6] = firmware_src_buffer[firmware_size + i];
        checksum ^= packet_buffer[6];

        ret = i2c_master_send(ft5x0x->client, packet_buffer, 7);
        if (0 > ret)
        {
            dev_err(dev, "[FTS] - ERROR: Could not write partial packet to the CTPM.\n");
            goto err;
        }

        msleep(20);
    }

    /*********************************************************************************************/
    dev_info(dev, "[FTS] Step 6: Checksum verification\n");
    temp_buffer[0] = 0xcc;
    ret = i2c_master_send(ft5x0x->client, temp_buffer, 1);
    if (0 > ret)
    {
        dev_err(dev, "[FTS] - ERROR: Could not write the Get Checksum command to the CTPM.\n");
        goto err;
    }
    msleep(50);
    ret = i2c_master_recv(ft5x0x->client,temp_buffer, 1);
    if (0 > ret)
    {
        dev_err(dev, "[FTS] - ERROR: Could not read the Checksum from the CTPM.\n");
        goto err;
    }
    if (checksum != temp_buffer[0])
    {
       dev_err(dev,  "[FTS] - ERROR: Checksum (0x%02X) did not match calculated value (0x%02X).\n", temp_buffer[0], checksum);
        ret = ERR_ECC;
        goto err;
    }

    /*********Step 7: reset the new FW***********************/
    dev_info(dev, "[FTS] Step 7: Reset the CTPM firmware\n");
    temp_buffer[0] = 0x07;
    ret = i2c_master_send(ft5x0x->client, temp_buffer, 1);
    if (0 > ret)
    {
        dev_err(dev, "[FTS] - ERROR: Could not write the Reset Firmware command to the CTPM.\n");
        goto err;
    }


    msleep(300);  //make sure CTP startup normally
    
    return ERR_OK;
err:
	if(ret){
		dev_info(dev,"[FTS] firmware upgrade failed with error %d \n",ret);
	}
	return ret;
}


//
//TP auto self calibration
//
static int fts_ctpm_auto_clb(struct ft5x0x_ts_data* ft5x0x)
{
    struct device *dev = &ft5x0x->client->dev;
    u8 uc_temp;
    u8 i ;

    dev_info(dev,"[FTS] start auto calibration.\n");
    msleep(200);
    fts_write_reg(ft5x0x,0, 0x40);  
    msleep(100);   //make sure already enter factory mode
    fts_write_reg(ft5x0x,2, 0x4);  //write command to start calibration
    msleep(300);
    for(i=0;i<100;i++)
    {
        fts_read_reg(ft5x0x,0,&uc_temp);
        if ( ((uc_temp&0x70)>>4) == 0x0)  //return to normal mode, calibration finish
        {
            break;
        }
        msleep(200);
        dev_info(dev,"[FTS] waiting calibration %d\n",i);        
    }
	dev_info(dev,"[FTS] calibration OK.\n");

	msleep(300);
	fts_write_reg(ft5x0x,0, 0x40);  //goto factory mode
	msleep(100);   //make sure already enter factory mode
	fts_write_reg(ft5x0x,2, 0x5);  //store CLB result
	msleep(300);
	fts_write_reg(ft5x0x,0, 0x0); //return to normal mode 
	msleep(300);
	dev_info(dev,"[FTS] store calibration result OK.\n");
	
	 return 0;
}

static int fts_ctpm_upgrade_with_firmware(struct ft5x0x_ts_data* ft5x0x )
{
  struct device *dev = &ft5x0x->client->dev;
   int i_ret;
   if(!ft5x0x->firmware)
   	return -1;

   /*call the upgrade function*/
   i_ret =  fts_ctpm_fw_upgrade(ft5x0x,(u8*)ft5x0x->firmware->data,ft5x0x->firmware->size);
   if (i_ret != 0)
   {
       dev_err(dev,"[FTS] upgrade failed i_ret = %d.\n", i_ret);
       //error handling ...
       //TBD
   }
   else
   {
       dev_info(dev,"[FTS] upgrade successfully.\n");
       fts_ctpm_auto_clb(ft5x0x);  //start auto CLB
       fts_ctpm_auto_clb(ft5x0x);  //start auto CLB
   }

   return i_ret;
}

static u8 fts_host_firmware_ver(struct firmware *firmware)
{
   const u8* buf=firmware->data;
   u32 size = firmware->size;
    if (size > 2)
    {
        return buf[size - 2];
    }
    else
    {
        //TBD, error handling?
        return 0xff; //default value
    }
}



static int fts_ctpm_auto_upg(struct ft5x0x_ts_data* ft5x0x,char* name)
{
	struct device *dev = &ft5x0x->client->dev;
	const struct firmware *firmware;
	char* firmware_name;
	u8 uc_host_fm_ver;
	u8 uc_tp_fm_ver;
	int           ret;

	if(name)
		firmware_name = name;
	else
		firmware_name = "ft5x0x.bin";
	dev_info(dev,"loading firmware %s\n",firmware_name);
	ret  = request_firmware(&firmware,firmware_name,dev);
	if (ret){
		dev_err(dev, "%s:load firmware %s fail %d\n", __func__, firmware_name,ret);
		return ret;
	}
	ft5x0x->firmware =  (struct firmware *)firmware;
	uc_tp_fm_ver =    fts_chip_firmware_ver(ft5x0x);
	uc_host_fm_ver = fts_host_firmware_ver(ft5x0x->firmware);
	if ( uc_tp_fm_ver == 0xa6  ||   //the firmware in touch panel maybe corrupted
		uc_tp_fm_ver <= uc_host_fm_ver //the firmware in host flash is new, need upgrade
	)
	{
		msleep(200);
		dev_info(dev,"[FTS] uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n", uc_tp_fm_ver, uc_host_fm_ver);
		ret = fts_ctpm_upgrade_with_firmware(ft5x0x);    
		if (ret == 0)
		{
			msleep(500);
			uc_host_fm_ver = fts_host_firmware_ver(ft5x0x->firmware);
			dev_info(dev,"[FTS] upgrade to new version 0x%x\n", uc_host_fm_ver);
		}
		else
		{
			dev_info(dev,"[FTS] upgrade failed ret=%d.\n", ret);
		}
		
	}
	else
	{
		dev_warn(dev,"[FTS] tp firmware version[%#x],host firmware version[%#x]\n",
			uc_tp_fm_ver,uc_host_fm_ver);
		dev_warn(dev,"[FTS] don't require to upgrade\n");
		
	}

	//anyway release firmware
	release_firmware(ft5x0x->firmware);
	ft5x0x->firmware = 0;

	return 0;
}


#endif


/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static void ft5x0x_ts_release(struct ft5x0x_ts_data* ft5x0x)
{
	input_report_abs(ft5x0x->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	input_sync(ft5x0x->input_dev);
}


//read touch point information
static int ft5x0x_read_data(struct ft5x0x_ts_data* ft5x0x)
{
	struct device *dev = &ft5x0x->client->dev;
	struct ts_event *event = &ft5x0x->event;
	u8 buf[CFG_POINT_READ_BUF] = {0};
	u16 uSwap;
	int ret = -1;
	int i;

	ret = ft5x0x_i2c_read(ft5x0x->client,0,buf, CFG_POINT_READ_BUF);
	if (ret < 0) {
		dev_err(dev,"%s ft5x0x_i2c_read failed: %d\n", __func__, ret);
		return ret;
	}
	memset(event, 0, sizeof(struct ts_event));
	event->touch_point = buf[2] & 0x07; 

	if (event->touch_point > CFG_MAX_TOUCH_POINTS)
	{
	event->touch_point = CFG_MAX_TOUCH_POINTS;
	}

	for (i = 0; i < event->touch_point; i++)
	{
	    event->au16_x[i] = (s16)(buf[3 + 6*i] & 0x0F)<<8 | (s16)buf[4 + 6*i];
	    event->au16_y[i] = (s16)(buf[5 + 6*i] & 0x0F)<<8 | (s16)buf[6 + 6*i];
		if(ft5x0x->xy_swap)
		{
			uSwap = event->au16_x[i];
			event->au16_x[i] = event->au16_y[i];
			event->au16_y[i] = uSwap;
		}
		if(ft5x0x->x_inverted)
		{
			event->au16_x[i] = ft5x0x->x_max-event->au16_x[i];
		}
		if(ft5x0x->y_inverted)
		{
			event->au16_y[i] = ft5x0x->y_max-event->au16_y[i];
		}

		event->au8_touch_event[i] = buf[0x3 + 6*i] >> 6;
		event->au8_finger_id[i] = (buf[5 + 6*i])>>4;
	}

	event->pressure = 200;

    return 0;
}

/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/


#if CFG_SUPPORT_TOUCH_KEY
static int ft5x0x_touch_key_process(struct ft5x0x_ts_data*  ts,struct input_dev *dev, int x, int y, int touch_event)
{
	int i;
	int key_id=-1;
	if(ts->identifer==TP_ID_5X0X)
	{
		if ( x < 797 &&x > 757)		   key_id = 1;
		else if ( x < 757&&x > 737)	   key_id = 0;	   
		else if ( x < 737&&x > 717)	   key_id = 2;
		else if (x < 67&&x > 47)	   key_id = 3;
		else	   key_id = 0xf;
	}
	else
	{
		if ( y < 450&&y > 410)            key_id = 2;    
		else if ( y < 360&&y > 280)   key_id = 0;    
		else if ( y < 220&&y > 160)   key_id = 1;   
		else if (y < 100&&y > 20)       key_id = 3;  
		else            key_id = 0xf;    
	}
    
    for(i = 0; i <CFG_NUMOFKEYS; i++ )
    {
        if(tsp_keystatus[i])
        {
            input_report_key(dev, tsp_keycodes[i], 0);
      
            printk("[FTS] %s key is release. Keycode : %d\n", tsp_keyname[i], tsp_keycodes[i]);

            tsp_keystatus[i] = KEY_RELEASE;
        }
        else if( key_id == i )
        {
            if( touch_event == 0)                                  // detect
            {
                input_report_key(dev, tsp_keycodes[i], 1);
                printk( "[FTS] %s key is pressed. Keycode : %d\n", tsp_keyname[i], tsp_keycodes[i]);
                tsp_keystatus[i] = KEY_PRESS;
            }
        }
    }
    return 0;
    
}    
#endif

static void ft5x0x_report_value(struct ft5x0x_ts_data* ft5x0x)
{
	struct ft5x0x_ts_data *data = ft5x0x;
	struct ts_event *event = &data->event;
	int i;


	for (i  = 0; i < event->touch_point; i++)
	{
	   //dev_info(dev,"tp[%d,%d]\n",event->au16_x[i],event->au16_y[i]);
	    if (event->au16_x[i] < SCREEN_MAX_X && event->au16_y[i] < SCREEN_MAX_Y)
	    // LCD view area
	    {
	        input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->au16_x[i]);
    		input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->au16_y[i]);
    		input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
    		input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->au8_finger_id[i]);
    		if (event->au8_touch_event[i]== 0 || event->au8_touch_event[i] == 2)
    		{
    		    input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
    		}
    		else
    		{
    		    input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
    		}
	    }
	    else //maybe the touch key area
	    {
		#if CFG_SUPPORT_TOUCH_KEY
		//special case for 5x0x
		if(ft5x0x->identifer==TP_ID_5X0X)
		{
			if (event->au16_y[i] >= SCREEN_MAX_Y)
			{
				ft5x0x_touch_key_process(ft5x0x,data->input_dev, event->au16_x[i], event->au16_y[i], event->au8_touch_event[i]);
			}
		}
		else if (event->au16_x[i] >= SCREEN_MAX_X)
		{
			ft5x0x_touch_key_process(ft5x0x,data->input_dev, event->au16_x[i], event->au16_y[i], event->au8_touch_event[i]);
		}		
		#endif
	    }
	    
			
		input_mt_sync(data->input_dev);
	}
	input_sync(data->input_dev);

    if (event->touch_point == 0) {
        ft5x0x_ts_release(ft5x0x);
    }



}	/*end ft5x0x_report_value*/


#if CFG_SUPPORT_AUTO_UPG

static int upgrade_show(struct device *dev, struct device_attribute *attr,
		    char *buf)
{
	struct i2c_client* client = to_i2c_client(dev);
	struct ft5x0x_ts_data *ft5x0x = i2c_get_clientdata(client);
	int ret=0;
	
	ret+= sprintf(buf+ret,"firwmare is %s running\n",ft5x0x->firmware?"":"not");

	return ret;	
}
static int upgrade_store(struct device *dev, struct device_attribute *attr,
		   const char *buf, size_t count)
{
	struct i2c_client* client = to_i2c_client(dev);
	struct ft5x0x_ts_data *ft5x0x = i2c_get_clientdata(client);
    	char name[256];

	strncpy(name, buf, count - 1);

	if(ft5x0x->firmware)
	{	
		dev_warn(dev,"ft5x0x firmware upgrade is already running\n ");
		return count;
	}

	dev_warn(dev,"prepare to upgrade ft5x0x firmware\n ");
	
	cancel_work_sync(&ft5x0x->pen_event_work);
	//disable irq
	disable_irq_nosync(ft5x0x->irq);

	//prepare to run upgrade
	if(strncmp(name,"ft",2)) 
		fts_ctpm_auto_upg(ft5x0x,NULL);
	else
		fts_ctpm_auto_upg(ft5x0x,name);	

	//resume 
	enable_irq(ft5x0x->irq);
	

	return count;
}

static DEVICE_ATTR(upgrade, S_IRUSR|S_IWUSR, upgrade_show, upgrade_store);

#endif

/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static void ft5x0x_ts_pen_irq_work(struct work_struct *work)
{
	struct ft5x0x_ts_data *ft5x0x = (struct ft5x0x_ts_data *)container_of(work,struct ft5x0x_ts_data,pen_event_work);

	if(! ft5x0x_read_data(ft5x0x))
		ft5x0x_report_value(ft5x0x);
	enable_irq(ft5x0x->irq);
}
/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static irqreturn_t ft5x0x_ts_interrupt(int irq, void *dev_id)
{
	struct ft5x0x_ts_data *data = dev_id;

	disable_irq_nosync(data->irq);
	if (!work_pending(&data->pen_event_work)) {
		queue_work(data->ts_workqueue, &data->pen_event_work);
	}

	return IRQ_HANDLED;
}

static int ft5x0x_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct ft5x0x_ts_data *priv = (struct ft5x0x_ts_data *)i2c_get_clientdata(client);
	//supress compiler warning
	if(priv){}
	return 0;
	
}

static int ft5x0x_ts_resume(struct i2c_client *client)
{
	struct ft5x0x_ts_data *priv = (struct ft5x0x_ts_data *)i2c_get_clientdata(client);
	//supress compiler warning
	if(priv){}
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static void ft5x0x_ts_early_suspend(struct early_suspend *handler)
{
	struct ft5x0x_ts_data *priv = container_of(handler, struct ft5x0x_ts_data, early_suspend);
	ft5x0x_ts_suspend(priv->client, PMSG_SUSPEND);
}
/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static void ft5x0x_ts_late_resume(struct early_suspend *handler)
{
	struct ft5x0x_ts_data *priv = container_of(handler, struct ft5x0x_ts_data, early_suspend);
	ft5x0x_ts_resume(priv->client);
}
#endif  //CONFIG_HAS_EARLYSUSPEND
/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static int 
ft5x0x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	struct input_dev *input_dev;	
	struct device *dev = &client->dev;
	struct ft5x0x_ts_platform_data* pdata = client->dev.platform_data;
	int err = 0;
	u8 uc_reg_value; 
#if CFG_SUPPORT_TOUCH_KEY
    int i;
#endif
	
	printk("[FTS] ft5x0x_ts_probe, driver version is %s.\n", CFG_FTS_CTP_DRIVER_VERSION);
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	
	// probe the device
	if(i2c_smbus_read_byte_data(client,0)<0)
	{
	    dev_err(dev, "failed to access %s[%s] in addr %02x\n",client->name,id->name,client->addr);
	    return -ENODEV;
	}

	printk("ft5x0x_ts_probe,%s detected\n", id->name);
		

	ft5x0x_ts = kzalloc(sizeof(*ft5x0x_ts), GFP_KERNEL);
	if (!ft5x0x_ts)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	ft5x0x_ts->irq = client->irq;
	ft5x0x_ts->client = client;
	i2c_set_clientdata(client, ft5x0x_ts);
	if(pdata)
	{
		ft5x0x_ts->xy_swap = pdata->xyswap;
		ft5x0x_ts->x_inverted = pdata->x_inverted;
		ft5x0x_ts->y_inverted = pdata->y_inverted;
	}

	//set identifier
	ft5x0x_ts->identifer = id->driver_data;


	INIT_WORK(&ft5x0x_ts->pen_event_work, ft5x0x_ts_pen_irq_work);
	

	ft5x0x_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!ft5x0x_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}


	err = request_irq(ft5x0x_ts->irq, ft5x0x_ts_interrupt, IRQF_NO_SUSPEND|IRQF_TRIGGER_FALLING, "ft5x0x_ts", ft5x0x_ts);
	if (err < 0) {
		dev_err(&client->dev, "ft5x0x_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}

	disable_irq_nosync(ft5x0x_ts->irq);
	device_init_wakeup(&client->dev, 1);

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	
	ft5x0x_ts->input_dev = input_dev;

	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);

	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_TRACKING_ID, 0, 5, 0, 0);

	ft5x0x_ts->x_max = SCREEN_MAX_X;
	ft5x0x_ts->y_max = SCREEN_MAX_Y;

	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);

#if CFG_SUPPORT_TOUCH_KEY
    //setup key code area
    set_bit(EV_SYN, input_dev->evbit);
    set_bit(BTN_TOUCH, input_dev->keybit);
    input_dev->keycode = tsp_keycodes;
    for(i = 0; i < CFG_NUMOFKEYS; i++)
    {
	input_set_capability(input_dev, EV_KEY, ((int*)input_dev->keycode)[i]);    
	tsp_keystatus[i] = KEY_RELEASE;
    }
#endif


	input_dev->name		= FT5X0X_NAME;		//dev_name(&client->dev)
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
		"ft5x0x_ts_probe: failed to register input device: %s\n",
		dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}


#ifdef CONFIG_HAS_EARLYSUSPEND
	ft5x0x_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ft5x0x_ts->early_suspend.suspend = ft5x0x_ts_early_suspend;
	ft5x0x_ts->early_suspend.resume	= ft5x0x_ts_late_resume;
	register_early_suspend(&ft5x0x_ts->early_suspend);
#endif

    msleep(150);  //make sure CTP already finish startup process
    
    //get some register information
    uc_reg_value = fts_chip_firmware_ver(ft5x0x_ts);
    dev_info(dev,"[FTS] Firmware version = 0x%x\n", uc_reg_value);
    fts_read_reg(ft5x0x_ts,FT5X0X_REG_PERIODACTIVE, &uc_reg_value);
    dev_info(dev,"[FTS] report rate is %dHz.\n", uc_reg_value * 10);
    fts_read_reg(ft5x0x_ts,FT5X0X_REG_THGROUP, &uc_reg_value);
    dev_info(dev,"[FTS] touch threshold is %d.\n", uc_reg_value * 4);

    i2c_set_clientdata(client, ft5x0x_ts);


#if CFG_SUPPORT_AUTO_UPG
	err = device_create_file(&client->dev,&dev_attr_upgrade);
#endif    


    enable_irq(ft5x0x_ts->irq);

    return 0;

exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
//	free_irq(client->irq, ft5x0x_ts);
	free_irq(ft5x0x_ts->irq, ft5x0x_ts);
exit_irq_request_failed:
//exit_platform_data_null:
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
exit_create_singlethread:
	printk("==singlethread error =\n");
	i2c_set_clientdata(client, NULL);
	kfree(ft5x0x_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}
/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static int __devexit ft5x0x_ts_remove(struct i2c_client *client)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	printk("==ft5x0x_ts_remove=\n");
	ft5x0x_ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ft5x0x_ts->early_suspend);
#if CFG_SUPPORT_AUTO_UPG
	device_remove_file(&client->dev,&dev_attr_upgrade);
#endif

	free_irq(client->irq, ft5x0x_ts);
	input_unregister_device(ft5x0x_ts->input_dev);
	kfree(ft5x0x_ts);
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct i2c_device_id ft5x0x_ts_id[] = {
	{ FT5X0X_NAME, TP_ID_5X0X },
	{ "ep-g5", TP_ID_G5 },		
	{ }
};


MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);

static struct i2c_driver ft5x0x_ts_driver = {
	.probe		= ft5x0x_ts_probe,
	.remove		= __devexit_p(ft5x0x_ts_remove),
	.id_table	= ft5x0x_ts_id,
	.driver	= {
		.name	= FT5X0X_NAME,
		.owner	= THIS_MODULE,
	},
#ifdef CONFIG_PM
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend  = so340010_suspend,
	.resume   = so340010_resume,
#endif
#endif
	
};

/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static int __init ft5x0x_ts_init(void)
{
	int ret;
	printk("==ft5x0x_ts_init==\n");
	ret = i2c_add_driver(&ft5x0x_ts_driver);
	printk("ret=%d\n",ret);
	return ret;
//	return i2c_add_driver(&ft5x0x_ts_driver);
}

/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static void __exit ft5x0x_ts_exit(void)
{
	printk("==ft5x0x_ts_exit==\n");
	i2c_del_driver(&ft5x0x_ts_driver);
}

module_init(ft5x0x_ts_init);
module_exit(ft5x0x_ts_exit);

MODULE_AUTHOR("<wenfs@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");

