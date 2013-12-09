/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-int-device.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/leds.h>
#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <mach/hardware.h>


#include "mt9p111.h"
#include "mxc_v4l2_capture.h"

/* Debug MT9P111 register Read/Write */
//#define MT9P111_DEBUG


#define MIN_FPS 5
#define MAX_FPS 30
#define DEFAULT_FPS 30

#define mt9p111_XCLK_MIN  12000000
#define mt9p111_XCLK_MAX 27000000

enum mt9p111_frame_rate {
	mt9p111_30_fps,
	mt9p111_5_fps
};

static int mt9p111_framerates[] = {
	[mt9p111_30_fps] = 30,
	[mt9p111_5_fps] = 5
};

/*!
 * Maintains the information on the current state of the sesor.
 */
struct sensor_data mt9p111_data;

DEFINE_LED_TRIGGER(ledtrig_flash);


struct mt9p111_format_struct;  /* coming later */

struct mt9p111_info {
	struct mt9p111_format_struct *fmt;  /* Current format */
	unsigned char sat;		/* Saturation value */
	int hue;			/* Hue value */
};

/*
 * Low-level register I/O.
 */
static int mt9p111_read(struct i2c_client *client, u16 reg, u16 *value, u16 num_bytes)
{
	int ret = 0;
	u16 data;
	u8 address[2];

	address[0] = reg>>8;
	address[1] = reg;
	ret = i2c_smbus_write_byte_data(client, address[0], address[1]);
	if (ret)
		return ret;
	if(num_bytes==2)
	{
		ret = i2c_master_recv(client,(char *)&data,2);
		if(ret<0)
			return ret;
		*value = ((data&0xff)<<8)|((data>>8)&0xff);
	}
	else if(num_bytes==1)
	{
		ret = i2c_master_recv(client,(char *)&data,1);
		if(ret<0)
			return ret;
		*value = data&0xff;
	}
	else
		return -1;
	return 0;
}

static int mt9p111_write(struct i2c_client *client, u16 reg, u16 value, u16 num_bytes)
{
#ifdef MT9P111_DEBUG
	unsigned short verify = 0;
#endif
	int ret = 0;
	u8 data[4];

	data[0] = (reg>>8)&0xff;
	data[1] = reg;
	if(num_bytes == 2)
	{
		data[2]=  (value>>8)&0xff;
		data[3]=  value&0xff;
		ret = i2c_master_send(client, data, 4);
	}
	else
	{
		data[2]=  value&0xff;
		ret = i2c_master_send(client, data, 3);
	}
	if (ret < 0)
		return ret;

#ifdef MT9P111_DEBUG
	/* verify write regs for debugging */
	ret = mt9p111_read(client, reg, &verify, num_bytes);
	if (ret)
		return ret;
	if (verify != value)
		printk(KERN_ERR "mt9p111 register 0x%x: write = 0x%x, but read = 0x%x\n", reg, value, verify);
#endif
	return 0;
}

static int mt9p111_sequential_write_reg(struct i2c_client *client, unsigned char *data, u16 datasize)
{
	int err;
	struct i2c_msg msg;
	int retry = 3;

	if (datasize==0)
		return 0;
	if (!client->adapter)
		return -ENODEV;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = datasize;
	msg.buf = data;
	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		printk(KERN_ERR "mt9p111_sequential_write_reg: i2c transfer failed, count %x \n",
		       msg.addr);
	} while (retry--);
	printk(KERN_ERR "write sequential: 0x%x\n", *((u16 *)data));

	return err;
}

static int build_sequential_buffer(unsigned char *pBuf, u16 width, u16 value) {
	u32 count = 0;

	switch (width)
	{
	  case 0:
	  // possibly no address, some focusers use this
	  break;

	  // cascading switch
	  case 32:
	    pBuf[count++] = (u8)((value>>24) & 0xFF);
	  case 24:
	    pBuf[count++] = (u8)((value>>16) & 0xFF);
	  case 16:
	    pBuf[count++] = (u8)((value>>8) & 0xFF);
	  case 8:
	    pBuf[count++] = (u8)(value & 0xFF);
	    break;

	  default:
	    printk("Unsupported Bit Width %d\n", width);
	    break;
	}
	return count;

}

/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int mt9p111_write_array(struct i2c_client *client, struct mt9p111_regval_list *vals)
{
	int ret;

	while (vals->reg_num != 0xffff || vals->value != 0xffff) {
		if (vals->reg_num == 0xdddd)
		{
			msleep(vals->value);
		}
		else
		{
			ret = mt9p111_write(client, vals->reg_num, vals->value, vals->num_bytes);
			if (ret < 0){
				pr_err("failed to write mt9p111 reg %d\n",vals->reg_num);
				return ret;
			}
			
		}
		vals++;
	}

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int mt9p111_g_register(struct i2c_client *client, struct v4l2_dbg_register * reg)
{
	return mt9p111_read(client, (u16)reg->reg, (u16 *)&(reg->val), (u16)reg->size);
}

static int mt9p111_s_register(struct i2c_client *client, struct v4l2_dbg_register * reg)
{
	return mt9p111_write(client, (u16)reg->reg, (u16)reg->val, (u16)reg->size);
}
#endif

static int mt9p111_detect(struct i2c_client *client)
{
	u16 model_id=0;
	int ret = 0;
	ret = mt9p111_read(client, REG_CHIPID, &model_id, 2);
	if (ret < 0 || model_id != 0x2880)
	{
		return -ENODEV;
	}
	dev_info(&client->dev,"model_id=0x%x\n",model_id);

	return 0;
}

static void ledtrig_camera_flash(int brightness)
{
	led_trigger_event(ledtrig_flash, brightness);
}



enum mt9p111_mode {
	mt9p111_mode_MIN = 0,
	mt9p111_mode_VGA_640_480 = 0,
	mt9p111_mode_QVGA_320_240 = 1,
	mt9p111_mode_NTSC_720_480 = 2,
	mt9p111_mode_PAL_720_576 = 3,
	mt9p111_mode_720P_1280_720 = 4,
	mt9p111_mode_1080P_1920_1080 = 5,
	mt9p111_mode_QSXGA_2592_1944 = 6,
	mt9p111_mode_QCIF_176_144 = 7,
	mt9p111_mode_XGA_1024_768 = 8,
	mt9p111_mode_MAX = 8
};

#define mt9p111_exp_comp_MIN -2
#define mt9p111_exp_comp_MAX 3

static u16 mt9p111_exposure_value[mt9p111_exp_comp_MAX-mt9p111_exp_comp_MIN+1] = 
{0x2C,0x38,0x44,0x50,0x5C,0x68};

/*
 * Then there is the issue of window sizes.  Try to capture the info here.
 */
struct mt9p111_win_size {
	int	width;
	int	height;
	int	fps;
	struct mt9p111_regval_list *regs;
};

/* Preview Size */
static struct mt9p111_win_size mt9p111_win_sizes[mt9p111_mode_MAX-mt9p111_mode_MIN+1] = {
	/* 640x480 */
	[mt9p111_mode_VGA_640_480]={
		.width		= 640,
		.height		= 480,
		.fps		= 30,
		.regs		= mt9p111_reg_640x480,
	},
	[mt9p111_mode_QVGA_320_240]=/* 320x240 */
	{
		.width		= 320,
		.height		= 240,
		.fps		= 30,
		.regs		= mt9p111_reg_320x240,
	},
#ifndef PICTURE_JPEG
	[mt9p111_mode_QSXGA_2592_1944]=/* 5M */
	{
		.width		= 2592,
		.height 	= 1944,
		.fps		= 5,
		.regs		= mt9p111_reg_2592x1944,
	},
#endif
};

#define N_WIN_SIZES (ARRAY_SIZE(mt9p111_win_sizes))

/* Capture JPEG Size */
static struct mt9p111_win_size mt9p111_win_jpg_sizes[] = {
#ifdef PICTURE_JPEG
	/* 5M */
	{
		.width		= 2592,
		.height 	= 1944,
		.fps		= 15,
		.regs		= mt9p111_reg_2592x1944,
	},
#endif
};

#define N_WIN_JPEG_SIZES (ARRAY_SIZE(mt9p111_win_jpg_sizes))

static int mt9p111_fw_down(struct i2c_client *client)
{
	int ret;
	pr_info("Write af fw start...\n");
	ret=mt9p111_write_array(client, mt9p111_reg_af_fw);
	pr_info("Write af fw end\n");
	return ret;
}


static int mt9p111_int_reset(struct i2c_client *client)
{
	/* Initialize settings */
	mt9p111_write_array(client, mt9p111_reg_init);
	pr_info("Write initial setting done.\n");
	mt9p111_fw_down(client);
	mt9p111_data.pix.width = 640;
	mt9p111_data.pix.height = 480;
	return 0;
}

#if 0
static struct mt9p111_control *mt9p111_find_control(__u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++)
		if (mt9p111_controls[i].qc.id == id)
			return mt9p111_controls + i;
	return NULL;
}

static int mt9p111_s_ctrl(struct i2c_client *client, struct v4l2_control *ctrl)
{
	struct mt9p111_control *octrl = mt9p111_find_control(ctrl->id);
	int ret = 0;

	if (octrl == NULL)
		return -EINVAL;
	ret =  octrl->tweak(client, ctrl->value);
	if (ret >= 0)
		return 0;
	return ret;
}

static int mt9p111_q_ctrl(struct i2c_client *client, struct v4l2_queryctrl *qc)
{
	struct mt9p111_control *ctrl = mt9p111_find_control(qc->id);

	if (ctrl == NULL)
		return -EINVAL;
	*qc = ctrl->qc;
	return 0;
}

static int mt9p111_g_ctrl(struct i2c_client *client, struct v4l2_control *ctrl)
{
	struct mt9p111_control *octrl = mt9p111_find_control(ctrl->id);
	int ret = 0;

	if (octrl == NULL)
		return -EINVAL;
	ret = octrl->query(client, &ctrl->value);
	if (ret >= 0)
		return 0;
	return ret;
}
#endif

static int wait_for_state(struct i2c_client *client, u16 value, int cnt, int interval_ms)
{
	int times,ret;
	u16 val;

	times=0;
	while(times<cnt)
	{
		/* verify write regs for debugging */
		ret = mt9p111_read(client, 0x8405, &val, 1);
		if (ret)
		{
			printk(KERN_ERR "wait for state %d fail\n",value);
			return -1;
		}
		//printk(KERN_ERR "0x8405: 0x%x\n", val);
		if(val==value)
		{
			printk(KERN_ERR "wait %d ms for state %d\n", times*interval_ms,value);
			return times;
		}
		msleep(interval_ms);
		times++;
	}
	printk(KERN_ERR "wait for state %d fail\n",value);
	return -1;
}



struct struct_reg_dump_range{
	u16 start;
	u16 end;
	u16 step;
};
static ssize_t mt9p111_get_reg(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client* c = container_of(dev, struct i2c_client, dev);
	struct struct_reg_dump_range reg_dump_ranges[] = {
		//registers
		{0x3000,0x300c,0x2},
		{0x3012,0x303c,0x2},
		{0x3040,0x31fc,0x2},
		{0x3210,0x33c2,0x2},

		//variables
		{0x8001,0x8018,0x2},
		{0x8401,0x8440,0x2},
		
	};
	struct struct_reg_dump_range *reg_dump_range;
	u16 reg,val;
	int i;
	//sensor core
	for(i=0;i<sizeof(reg_dump_ranges)/sizeof(reg_dump_ranges[0]);i++)
	{
		reg_dump_range = &reg_dump_ranges[i];
		for(reg = reg_dump_range->start;
			reg<=reg_dump_range->end;
			reg+=reg_dump_range->step)
		{
			mt9p111_read(c, reg, &val,reg_dump_range->step);
			pr_info("0x%04x ---  0x%02x\n",reg,val);
		}
	}
	
	return sprintf(buf, "Dumped all registers.\n");
}

static ssize_t mt9p111_set_reg(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client* c = container_of(dev, struct i2c_client, dev);
	u16 reg ,val,size=2;
	int ret = sscanf(buf, "%x %d %x", &reg, &val, &size);
	if (ret<2)
	{
		mt9p111_read(c,reg,&val,size);
		pr_debug("read mt9p111 reg[0x%x]=0x%x size0x%x\n",reg,val,size);
		
	}
	else 
	{
		pr_debug("write mt9p111 reg[0x%x]=0x%x size0x%x\n",reg,val,size);
		mt9p111_write(c, reg, val, size);
	}

	return count;
}

static int mt9p111_start_focus(struct i2c_client *client)
{
	return mt9p111_write_array(client,mt9p111_reg_start_focus);
}

static int mt9p111_stop_focus(struct i2c_client *client)
{
	return mt9p111_write_array(client,mt9p111_reg_stop_focus);
}

static int mt9p111_check_focus(struct i2c_client *client)
{
	u16 val;
	int ret;
	ret = mt9p111_read(client, 0xb006, &val, 1);
	if (ret)
	{
		printk(KERN_ERR "reading focus state fail\n");
		return -1;
	}
	return (val==0);
}

static ssize_t mt9p111_set_focus(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int start;
	int ret;
	struct i2c_client* c = container_of(dev, struct i2c_client, dev);

	ret = sscanf(buf, "%d", &start);
	if (ret == 1)
	{
		if(start)
			mt9p111_start_focus(c);
		else
			mt9p111_stop_focus(c);
	}

	return count;
}

static ssize_t mt9p111_set_flash(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int brightness;
	int ret;

	ret = sscanf(buf, "%d", &brightness);
	if (ret == 1)
	{
		ledtrig_camera_flash(brightness);	
	}

	return count;
}


static struct device_attribute static_attrs[] = {
	__ATTR(reg, 0666, mt9p111_get_reg, mt9p111_set_reg),
	__ATTR(focus, 0666, NULL, mt9p111_set_focus),
	__ATTR(flash, 0666, NULL, mt9p111_set_flash),	
};


static struct fsl_mxc_camera_platform_data *camera_plat;

static int mt9p111_probe(struct i2c_client *client,
				const struct i2c_device_id *device_id);
static int mt9p111_remove(struct i2c_client *client);

static const struct i2c_device_id mt9p111_id[] = {
	{"mt9p111", 0},
	{"mt9p111x", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, mt9p111_id);

static struct i2c_driver mt9p111_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "mt9p111",
		  },
	.probe  = mt9p111_probe,
	.remove = mt9p111_remove,
	.id_table = mt9p111_id,
};


/* --------------- IOCTL functions from v4l2_int_ioctl_desc --------------- */

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	memset(p, 0, sizeof(*p));
	p->u.bt656.clock_curr = mt9p111_data.mclk;
	pr_debug("   clock_curr=mclk=%d\n", mt9p111_data.mclk);
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.clock_min = mt9p111_XCLK_MIN;
	p->u.bt656.clock_max = mt9p111_XCLK_MAX;
	//p->u.bt656.bt_sync_correct = 1;  /* Indicate external vsync */
	p->u.bt656.latch_clk_inv=1;

	return 0;
}

/*!
 * ioctl_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct sensor_data *sensor = s->priv;

	sensor->on = on;
	
	if (camera_plat->pwdn){
		camera_plat->pwdn(on?0:1);
	}


	return 0;
}

/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int mt9p111_change_mode(enum mt9p111_frame_rate frame_rate,
			    enum mt9p111_mode mode)
{
	int ret = 0, cnt = 0;
	u16 val;

	if (mode != mt9p111_mode_VGA_640_480 && mode != mt9p111_mode_QVGA_320_240
		&& mode != mt9p111_mode_QSXGA_2592_1944 ) {
		pr_err("Wrong mt9p111 mode %d!\n",mode);
		return -1;
	}

	if(mt9p111_data.pix.width == mt9p111_win_sizes[mode].width &&
		mt9p111_data.pix.height == mt9p111_win_sizes[mode].height)
		return 0;

	if (mode == mt9p111_mode_VGA_640_480)
	{
		ret= mt9p111_read(mt9p111_data.i2c_client,0x8405, &val, 1);
		if(ret)
		{
			return ret;
		}
		if(val==7) // still->preview
		{
			mt9p111_write_array(mt9p111_data.i2c_client,mt9p111_reg_preview);
			mt9p111_data.pix.width = 640;
			mt9p111_data.pix.height = 480;
			return ((wait_for_state(mt9p111_data.i2c_client,3,2000,50)>=0)?0:-1);
		}
	}
again:
	mt9p111_write_array(mt9p111_data.i2c_client, mt9p111_win_sizes[mode].regs);
	mt9p111_data.pix.width = mt9p111_win_sizes[mode].width;
	mt9p111_data.pix.height = mt9p111_win_sizes[mode].height;
	printk(KERN_ERR "Writting resolution [%dx%d] done\n",  mt9p111_win_sizes[mode].width,  mt9p111_win_sizes[mode].height);
	if (mode == mt9p111_mode_QSXGA_2592_1944)
	{
		return ((wait_for_state(mt9p111_data.i2c_client,7,2000,50)>=0)?0:-1);
	}
	else
	{
		ret=wait_for_state(mt9p111_data.i2c_client,2,100,5);
		if(ret<=0)
		{
			if(ret==0)
				wait_for_state(mt9p111_data.i2c_client,3,2000,10);
			if(cnt++<5)
			{
				printk(KERN_ERR "ret %d try again\n",ret);
				goto again;
			}
			return -1;
		}
		return ((wait_for_state(mt9p111_data.i2c_client,3,2000,50)>=0)?0:-1);
	}
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	u32 tgt_fps;	/* target frames per secound */
	enum mt9p111_frame_rate frame_rate;
	int ret = 0;
	struct sensor_data *sensor = s->priv;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	pr_debug("In mt9p111:ioctl_s_parm %d\n",
		 a->type);

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		/* Check that the new frame rate is allowed. */
		if ((timeperframe->numerator == 0) ||
		    (timeperframe->denominator == 0)) {
			timeperframe->denominator = DEFAULT_FPS;
			timeperframe->numerator = 1;
		}

		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps >= MAX_FPS) {
			timeperframe->denominator = MAX_FPS;
			timeperframe->numerator = 1;
		} else /*if (tgt_fps < MIN_FPS)*/ {
			timeperframe->denominator = MIN_FPS;
			timeperframe->numerator = 1;
		}

		/* Actual frame rate we use */
		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps == 5)
			frame_rate = mt9p111_5_fps;
		else if (tgt_fps == 30)
			frame_rate = mt9p111_30_fps;
		else {
			pr_err(" The camera frame rate is not supported!\n");
			printk(" The camera frame rate is %d,not supported!\n",tgt_fps);
			return -EINVAL;
		}

		ret = mt9p111_change_mode(frame_rate,
				a->parm.capture.capturemode);
		if (ret < 0)
			return ret;

		printk(" new frame_rate is %d,capturemode is %d!\n",frame_rate,a->parm.capture.capturemode);

		sensor->streamcap.timeperframe = *timeperframe;
		sensor->streamcap.capturemode = a->parm.capture.capturemode;

		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_debug("   type is not " \
			"V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
			a->type);
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;

}

/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor_data *sensor = s->priv;

	f->fmt.pix = sensor->pix;

	return 0;
}

/*!
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int ret = 0;

	switch (vc->id) {
	case V4L2_CID_MXC_AUTOFOCUS:
		{
			ret=mt9p111_check_focus(mt9p111_data.i2c_client);
			if(ret>=0)
			{
				vc->value = ret;
				ret=0;
			}
			break;
		}
	case V4L2_CID_MXC_FLASH:
		{
			ledtrig_camera_flash(LED_FULL);
			break;
		}
	case V4L2_CID_BRIGHTNESS:
		vc->value = mt9p111_data.brightness;
		break;
	case V4L2_CID_HUE:
		vc->value = mt9p111_data.hue;
		break;
	case V4L2_CID_CONTRAST:
		vc->value = mt9p111_data.contrast;
		break;
	case V4L2_CID_SATURATION:
		vc->value = mt9p111_data.saturation;
		break;
	case V4L2_CID_RED_BALANCE:
		vc->value = mt9p111_data.red;
		break;
	case V4L2_CID_BLUE_BALANCE:
		vc->value = mt9p111_data.blue;
		break;
	case V4L2_CID_EXPOSURE:
		vc->value = mt9p111_data.ae_mode;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}


/*!
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int retval = 0;

	pr_debug("In mt9p111:ioctl_s_ctrl %d\n",
		 vc->id);

	switch (vc->id) {
	case V4L2_CID_MXC_AUTOFOCUS:
		{
			if(vc->value)
				retval=mt9p111_start_focus(mt9p111_data.i2c_client);
			else
				retval=mt9p111_stop_focus(mt9p111_data.i2c_client);
			break;
		}
	case V4L2_CID_BRIGHTNESS:
		break;
	case V4L2_CID_CONTRAST:
		break;
	case V4L2_CID_SATURATION:
		break;
	case V4L2_CID_HUE:
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		break;
	case V4L2_CID_RED_BALANCE:
		break;
	case V4L2_CID_BLUE_BALANCE:
		break;
	case V4L2_CID_GAMMA:
		break;
	case V4L2_CID_EXPOSURE:
		{
			if(vc->value<mt9p111_exp_comp_MIN||vc->value>mt9p111_exp_comp_MAX)
				retval = -EPERM;
			else
				retval=mt9p111_write(mt9p111_data.i2c_client, 0xA409, mt9p111_exposure_value[vc->value-mt9p111_exp_comp_MIN], 1);
		}
		break;
	case V4L2_CID_AUTOGAIN:
		break;
	case V4L2_CID_GAIN:
		break;
	case V4L2_CID_HFLIP:
		break;
	case V4L2_CID_VFLIP:
		break;
	default:
		retval = -EPERM;
		break;
	}

	return retval;
}

/*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *argp)
{
	switch(argp->index)
	{
		case 0:
			argp->discrete.width =mt9p111_win_sizes[mt9p111_mode_VGA_640_480].width;
			argp->discrete.height = mt9p111_win_sizes[mt9p111_mode_VGA_640_480].height;
			return 0;
		case 1:
			argp->discrete.width =mt9p111_win_sizes[mt9p111_mode_QVGA_320_240].width;
			argp->discrete.height = mt9p111_win_sizes[mt9p111_mode_QVGA_320_240].height;
			return 0;
#ifndef PICTURE_JPEG
		case 2:
			argp->discrete.width =mt9p111_win_sizes[mt9p111_mode_QSXGA_2592_1944].width;
			argp->discrete.height = mt9p111_win_sizes[mt9p111_mode_QSXGA_2592_1944].height;
			return 0;
#endif
		default:
			return -EINVAL;
	}
}

/*!
 * ioctl_enum_frameintervals - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
					 struct v4l2_frmivalenum *argp)
{
	switch (argp->pixel_format) {
		case V4L2_PIX_FMT_VYUY:
		case V4L2_PIX_FMT_UYVY:
		case V4L2_PIX_FMT_YUV420:
			if (argp->index == N_WIN_SIZES)
				return -EINVAL;
			else if (argp->index > N_WIN_SIZES) {
				printk(KERN_ERR "camera: mt9p111 enum unsupported preview format frame rate!\n");
				return -EINVAL;
			} else {
				argp->type = V4L2_FRMIVAL_TYPE_DISCRETE;
				argp->discrete.numerator = 1;
				argp->discrete.denominator = mt9p111_win_sizes[argp->index].fps;
				break;
			}
		case V4L2_PIX_FMT_JPEG:
			if (argp->index == N_WIN_JPEG_SIZES)
				return -EINVAL;
			else if (argp->index > N_WIN_JPEG_SIZES) {
				printk(KERN_ERR "camera: mt9p111 enum unsupported jpeg format frame rate!\n");
				return -EINVAL;
			} else {
				argp->type = V4L2_FRMIVAL_TYPE_DISCRETE;
				argp->discrete.numerator = 1;
				argp->discrete.denominator = mt9p111_win_jpg_sizes[argp->index].fps;
				break;
			}
		default:
			printk(KERN_ERR "camera: mt9p111 enum format frame rate with unsupported format!\n");
			return -EINVAL;
	}

	return 0;
}

/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	((struct v4l2_dbg_chip_ident *)id)->match.type =
					V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name, "mt9p111_camera");

	return 0;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{

	return 0;
}

/*!
 * ioctl_enum_fmt_cap - V4L2 sensor interface handler for VIDIOC_ENUM_FMT
 * @s: pointer to standard V4L2 device structure
 * @fmt: pointer to standard V4L2 fmt description structure
 *
 * Return 0.
 */
static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
			      struct v4l2_fmtdesc *fmt)
{
	if (fmt->index > 0)
		return -EINVAL;

	fmt->pixelformat = mt9p111_data.pix.pixelformat;

	return 0;
}

/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct sensor_data *sensor = s->priv;
	int ret=0;
	//ret = ov5640_init_mode();
	ret=mt9p111_int_reset(sensor->i2c_client);
	
	return ret;
}

/*!
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the device when slave detaches to the master.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	return 0;
}

/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
static struct v4l2_int_ioctl_desc mt9p111_ioctl_desc[] = {
	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func *)ioctl_dev_init},
	{vidioc_int_dev_exit_num, ioctl_dev_exit},
	{vidioc_int_s_power_num, (v4l2_int_ioctl_func *)ioctl_s_power},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func *)ioctl_g_ifparm},
/*	{vidioc_int_g_needs_reset_num,
				(v4l2_int_ioctl_func *)ioctl_g_needs_reset}, */
/*	{vidioc_int_reset_num, (v4l2_int_ioctl_func *)ioctl_reset}, */
	{vidioc_int_init_num, (v4l2_int_ioctl_func *)ioctl_init},
	{vidioc_int_enum_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_enum_fmt_cap},
/*	{vidioc_int_try_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_try_fmt_cap}, */
	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_g_fmt_cap},
/*	{vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_s_fmt_cap}, */
	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func *)ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func *)ioctl_s_parm},
/*	{vidioc_int_queryctrl_num, (v4l2_int_ioctl_func *)ioctl_queryctrl}, */
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *)ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *)ioctl_s_ctrl},
	{vidioc_int_enum_framesizes_num,
				(v4l2_int_ioctl_func *)ioctl_enum_framesizes},
	{vidioc_int_enum_frameintervals_num,
				(v4l2_int_ioctl_func *)ioctl_enum_frameintervals},
	{vidioc_int_g_chip_ident_num,
				(v4l2_int_ioctl_func *)ioctl_g_chip_ident},
};

static struct v4l2_int_slave mtp9111_slave = {
	.ioctls = mt9p111_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(mt9p111_ioctl_desc),
};

static struct v4l2_int_device mtp9111_int_device = {
	.module = THIS_MODULE,
	.name = "mtp9111",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &mtp9111_slave,
	},
};


/*!
 * ov5640 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int mt9p111_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret;
	struct fsl_mxc_camera_platform_data *plat_data = client->dev.platform_data;
	int i;

	/* Set initial values for the sensor struct. */
	memset(&mt9p111_data, 0, sizeof(mt9p111_data));
	mt9p111_data.mclk = plat_data->mclk;
	mt9p111_data.mclk_source = plat_data->mclk_source;
	mt9p111_data.csi = plat_data->csi;

	mt9p111_data.i2c_client = client;
	mt9p111_data.pix.pixelformat = V4L2_PIX_FMT_UYVY;
	mt9p111_data.pix.width = 640;
	mt9p111_data.pix.height = 480;
	mt9p111_data.streamcap.capability = V4L2_MODE_HIGHQUALITY |
					   V4L2_CAP_TIMEPERFRAME;
	mt9p111_data.streamcap.capturemode = 0;
	mt9p111_data.streamcap.timeperframe.denominator = DEFAULT_FPS;
	mt9p111_data.streamcap.timeperframe.numerator = 1;

	//init 
	if (plat_data->io_init)
		plat_data->io_init();

	if (plat_data->mclk_on)
		plat_data->mclk_on(1);
	
	if (plat_data->pwdn)
		plat_data->pwdn(0);	

	ret = mt9p111_detect(client);

	if(ret)
	{
		dev_warn(&client->dev,"MT9P111 not found!!\n");
		goto err;
	}
	camera_plat = plat_data;

	mtp9111_int_device.priv = &mt9p111_data;

	led_trigger_register_simple(plat_data->trigger_led_flash?
		plat_data->trigger_led_flash:"camera",
		&ledtrig_flash);


	ret = v4l2_int_device_register(&mtp9111_int_device);
	for (i=0; i<ARRAY_SIZE(static_attrs); i++)
	{
		ret = device_create_file(&client->dev, &static_attrs[i]);
		if (ret) printk("camera: failed creating device file\n");
	}

	dev_info(&client->dev,"camera mtp9111 is found\n");
err:
	if (plat_data->pwdn)
		plat_data->pwdn(1);	
	if (plat_data->mclk_on)
		plat_data->mclk_on(0);
	return ret;
}

/*!
 * ov5640 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int mt9p111_remove(struct i2c_client *client)
{
	struct fsl_mxc_camera_platform_data *plat_data = client->dev.platform_data;
	
	if(plat_data->trigger_led_flash){
		led_trigger_unregister_simple(ledtrig_flash);
	}
	v4l2_int_device_unregister(&mtp9111_int_device);


	return 0;
}

/*!
 * ov5640 init function
 * Called by insmod ov5640_camera.ko.
 *
 * @return  Error code indicating success or failure
 */
static __init int mt9p111_init(void)
{
	u8 err;

	err = i2c_add_driver(&mt9p111_i2c_driver);
	if (err != 0)
		pr_err("%s:driver registration failed, error=%d \n",
			__func__, err);

	return err;
}

/*!
 * OV5640 cleanup function
 * Called on rmmod ov5640_camera.ko
 *
 * @return  Error code indicating success or failure
 */
static void __exit mt9p111_clean(void)
{
	i2c_del_driver(&mt9p111_i2c_driver);
}

module_init(mt9p111_init);
module_exit(mt9p111_clean);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("mt9p111 Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
