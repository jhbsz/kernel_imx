#ifndef __EK2127_TS_H__
#define __EK2127_TS_H__


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif 
#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include <mach/mfp.h>

#include <linux/ioctl.h>

#define EK2048_TS_DEVICE		    "EK2127_ts"
#define EK2048_TS_NAME	   	    "EK2127_ts"
#define EK2048_TS_ADDR			0x15
#endif

