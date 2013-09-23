/*
 * sound/soc/codecs/rt5625.c
 *
 *   support for realtek ALC5625 codec device
 * Copyright (C) 2010 Realtek Semiconductor Inc.
 * Copyright (C) 2011 Marvell International Ltd.
 * Copyright (C) 2011 Quester Technology Inc.
 * All rights reserved.
 *
 *   original code from realteak
 *   2011-03-04: Leo Yan <leoy@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
//#define DEBUG

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/jiffies.h>
#include <linux/regulator/consumer.h>
#include <asm/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>


#include "rt5625.h"

#define RT5625_VERSION 	"1.0"

#define RT5625_NUM_SUPPLIES 6
static const char *rt5625_supply_names[RT5625_NUM_SUPPLIES] = {
	"DBVDD",
	"DCVDD",
	"AVDD1",
	"AVDD2",
	"HPVDD",
	"SPKVDD",
};


struct rt5625_priv {
	enum snd_soc_control_type control_type;
	void* control_data;
	struct snd_soc_codec *codec;
	unsigned int stereo_sysclk;
	unsigned int voice_sysclk;

	
	struct regulator_bulk_data supplies[RT5625_NUM_SUPPLIES];
};


/*
 * Virtual Registers
*/
/*
 * bit[0]:	linein playback switch
 * bit[1]:	phone
 * bit[2]:	mic1
 * bit[3]:	mic2
 * bit[4]:	vopcm
 */
#define RT5625_HPL_MIXER 0x80
#define RT5625_HPR_MIXER 0x82

/*
 * bit[0..1]:	aec control
 * bit[2..3]:	adcr func
 * bit[4]:	spkl pga
 * bit[5]:	spkr pga
 * bit[6]:	hpl pga
 * bit[7]:	hpr pga
 * bit[8]:	dump dsp
 */
#define RT5625_VIRTUAL_MISC_FUNC 0x84

/*
 * bit[0] hp depop done
*/
#define RT5625_VIRTUAL_MISC2 	0x86

static u16 rt5625_reg[] = {
	0x59b4, 0x8080, 0x8080, 0x8080,		/* R00 - R06 */
	0xc800, 0xe808, 0x1010, 0x0808,		/* R08 - R0e */
	0xe0ef, 0xcbcb, 0x7f7f, 0x0000,		/* R10 - R16 */
	0xe010, 0x0000, 0x8008, 0x2007,		/* R18 - R1e */
	0x0000, 0x0000, 0x00c0, 0xef00,		/* R20 - R26 */
	0x0000, 0x0000, 0x0000, 0x0000,		/* R28 - R2e */
	0x0000, 0x0000, 0x0000, 0x0000,		/* R30 - R36 */
	0x0000, 0x0000, 0x0000, 0x0000,		/* R38 - R3e */
	0x0c0a, 0x0000, 0x0000, 0x0000,		/* R40 - R46 */
	0x0029, 0x0000, 0xbe3e, 0x3e3e,		/* R48 - R4e */
	0x0000, 0x0000, 0x803a, 0x0000,		/* R50 - R56 */
	0x0000, 0x0009, 0x0000, 0x0000,		/* R58 - R5e */
	0x3075, 0x1010, 0x3110, 0x0000,		/* R60 - R66 */
	0x0553, 0x0000, 0x0000, 0x0000,		/* R68 - R6e */
	0x0000, 0x0000, 0x0000, 0x0000,		/* R70 - R76 */
	0x0000, 0x0000, 0x0000, 0x0000,		/* R76 - R7e */
	0x0000, 0x0000, 0x0003, 0x0000,		/* R80 - R86*/
};


static struct rt5625_vodsp_reg rt5625_vodsp_init[] = {
#if 0
	#if 0
	{0x232C, 0x0025},
	{0x230B, 0x0001},
	{0x2308, 0x007F},
	{0x23F8, 0x4003},
	{0x2301, 0x0002},
	{0x2328, 0x0001},
	{0x2304, 0x00FA},
	{0x2305, 0x0500},
	{0x2306, 0x4000},
	{0x230D, 0x0900},
	{0x230E, 0x0280},
	{0x2312, 0x00B1},
	{0x2314, 0xC000},
	{0x2316, 0x0041},
	{0x2317, 0x2200},
	{0x2318, 0x0C00},
	{0x231D, 0x0050},
	{0x231F, 0x4000},
	{0x2330, 0x0008},
	{0x2335, 0x000A},
	{0x2336, 0x0004},
	{0x2337, 0x5000},
	{0x233A, 0x0300},
	{0x233B, 0x0030},
	{0x2341, 0x0008},
	{0x2343, 0x0800},
	{0x2352, 0x7FFF},
	{0x237F, 0x0400},
	{0x23A7, 0x2800},
	{0x22CE, 0x0400},
	{0x22D3, 0x1500},
	{0x22D4, 0x2800},
	{0x22D5, 0x3000},
	{0x2399, 0x2800},
	{0x230C, 0x0000},	/* enable VODSP AEC function */
	#else
	{0x22CE, 0x0400},
	{0x22D3, 0x1500},
	{0x22D4, 0x2800},
	{0x22D5, 0x3000},
	{0x22D6, 0x6800},
	{0x2301, 0x0002},
	{0x2304, 0x00FA},
	{0x2305, 0x0800},
	//{0x2305, 0x1200},
	//{0x2306, 0x7F00},
	{0x2306, 0x4000},
	{0x2307, 0x7fff},
	//{0x2307, 0x3FFF},
	{0x2308, 0x007F},
	{0x230B, 0x0001},
	//{0x230D, 0x0800},
	{0x230D, 0x0100},
	//{0x230D, 0x0400},
	{0x230E, 0x0200},
	//{0x230E, 0x0200},
	{0x2311, 0x0101},
	{0x2312, 0x00B1},
	{0x2314, 0xC000},
	{0x2315, 0x43CF},
	{0x2316, 0x0075},
	{0x2317, 0x2800},
	{0x2318, 0x0a00},
	{0x231D, 0x0050},
	{0x231F, 0x4000},
	{0x2328, 0x0002},
	{0x232C, 0x0025},
	{0x2330, 0x0008},
	{0x2335, 0x0009},
	{0x2336, 0x0005},
	{0x2337, 0x5000},
	{0x233A, 0x0120},
	{0x233B, 0x0015},
	{0x2341, 0x0008},
	{0x2343, 0x0800}, 
	{0x2352, 0x7F00},
	//{0x2352, 0x4000},
	{0x2353, 0x2600},
	{0x237F, 0x2200},
	{0x2399, 0x1800},
	{0x239A, 0x0180},
	{0x23A0, 0x2800},
	{0x23A3, 0x1200},
	{0x23A7, 0x1800},
	{0x23F8, 0x4003}, 
	{0x23FF, 0x0000},
	{0x23E7, 0x2800},
	{0x23E8, 0x0a00},
	{0x23E9, 0x0a00},
	{0x230C, 0x0000}, //to enable VODSP AEC functionginal Message ----- 
	#endif
#endif
	{0x22CE, 0x0400},
	{0x22D3, 0x1500},
	{0x22D4, 0x2800},
	{0x22D5, 0x3000},
	{0x22D6, 0x6800},
	{0x2301, 0x0002},
	{0x2304, 0x00FA},
	{0x2305, 0x3600},
	{0x2306, 0x4000},
	{0x2307, 0x7FFF},
	{0x2308, 0x007F},
	{0x230B, 0x0001},
	{0x230D, 0x0100},
	//{0x230D, 0x0500},
	{0x230E, 0x0100},
	{0x2311, 0x0101},
	{0x2312, 0x00B1},
	{0x2314, 0xC000},
	{0x2315, 0x43CF},
	{0x2316, 0x0075},
	{0x2317, 0x1800},
	{0x2318, 0x0C00},
	{0x231D, 0x0050},
	{0x231F, 0x4000},
	{0x2328, 0x0002},
	{0x232C, 0x0025},
	{0x2330, 0x0008},
	{0x2335, 0x0008},
	{0x2336, 0x0003},
	{0x2337, 0x5000},
	{0x233A, 0x0120},
	{0x233B, 0x0015},
	{0x2341, 0x0008},
	{0x2343, 0x0800},
	{0x2352, 0x7F00},
	{0x2353, 0x2600},
	{0x237F, 0x2200},
	{0x2399, 0x1800},
	{0x239A, 0x0180},
	{0x23A0, 0x2800},
	{0x23A3, 0x1200},
	{0x23A7, 0x1800},
	{0x23F8, 0x4003},
	{0x23FF, 0x0000},
	{0x23E7, 0x1800},
	{0x23E8, 0x0C00},
	{0x23E9, 0x0C00},
	{0x230C, 0x0000}, //to enable VODSP AEC functionginal Message -----

};


#define RT5625_VODSP_REG_NUM	ARRAY_SIZE(rt5625_vodsp_init)

static unsigned int rt5625_read_reg_cache(struct snd_soc_codec *codec,
				 unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	reg >>=1;
	return cache[reg];
}

static void rt5625_write_reg_cache(struct snd_soc_codec *codec,
				   unsigned int reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;

	reg >>= 1;

	cache[reg] = value;
}

static int rt5625_direct_read(struct snd_soc_codec *codec, unsigned int reg)
{
	if(reg<=0x74)
	{
		u8 data[2] = {0};
		int value;
		data[0] = reg;		
		if (i2c_master_send(codec->control_data, data, 1) == 1) {
			if(2!=i2c_master_recv(codec->control_data, data, 2))
				goto failed;
			
			value = (data[0] << 8) | data[1];
		
			//special case for RT5625_PWR_MANAG_ADD1
			//for issue codec may be in short current status
			if(RT5625_PWR_MANAG_ADD1==reg)
			{
				if(0==(value&PWR_MAIN_BIAS))
				{
					printk("\n\n %s bogus status of RT5625_PWR_MANAG_ADD1\n\n",__func__);
					value|=PWR_MAIN_BIAS;
				}
			}
			
			pr_debug("rt5625 read reg %#x=%#x\n",reg,value);
			return value;
		}
	}	
failed:
	printk(KERN_ERR "%s: read reg failed\n", __func__);
	return -EIO;
	
}
static int rt5625_direct_write(struct snd_soc_codec *codec, unsigned int reg,
			unsigned int value)
{
	if(reg<=0x74)
	{
		u8 data[3];
		//special case for RT5625_PWR_MANAG_ADD1
		//for issue codec may be in short current status
		if(RT5625_PWR_MANAG_ADD1==reg)
		{
			if(0==(value&PWR_MAIN_BIAS))
			{
				printk("\n\n %s bogus status of RT5625_PWR_MANAG_ADD1\n\n",__func__);
				value|=PWR_MAIN_BIAS;
			}
		}
		
		pr_debug("rt5625 write reg %#x=%#x\n",reg,value);
		data[0] = reg;
		data[1] = (value & 0xff00) >> 8;
		data[2] = (value & 0x00ff);

		if (i2c_master_send(codec->control_data, data, 3) != 3) {
			printk(KERN_ERR "rt5625_write fail\n");
			return -EIO;
		}
		
	}
	return 0;
	
}


static unsigned int rt5625_read(struct snd_soc_codec *codec,
				unsigned int reg)
{
	//always return cache
	if(reg<=0x74)
		return rt5625_direct_read(codec,reg);
	return rt5625_read_reg_cache(codec,reg);
}



static int rt5625_write(struct snd_soc_codec *codec, unsigned int reg,
			unsigned int value)
{
	pr_debug("rt5625 write reg:%2.2x value:%4.4x\n",reg,value);
	rt5625_write_reg_cache(codec, reg, value);
	if(reg<=0x74)
	{
		u8 data[3];
		//special case for RT5625_PWR_MANAG_ADD1
		//for issue codec may be in short current status
		if(RT5625_PWR_MANAG_ADD1==reg)
		{
			if(0==(value&PWR_MAIN_BIAS))
			{
				printk("\n\n %s bogus status of RT5625_PWR_MANAG_ADD1\n\n",__func__);
				value|=PWR_MAIN_BIAS;
			}
		}

		data[0] = reg;
		data[1] = (value & 0xff00) >> 8;
		data[2] = (value & 0x00ff);		
		
		if (i2c_master_send(codec->control_data, data, 3) != 3) {
			printk(KERN_ERR "rt5625_write fail\n");
			return -EIO;
		}
	}
	return 0;
	
}



/* read/write dsp reg */
static int rt5625_wait_vodsp_i2c_done(struct snd_soc_codec *codec)
{
	unsigned int count = 0, data;

	do {
		if (count > 10)
			return -EBUSY;

		data = rt5625_direct_read(codec, RT5625_VODSP_REG_CMD);
		count++;

		mdelay(1);
	} while (data & VODSP_BUSY);

	return 0;
}

static int rt5625_write_vodsp_reg(struct snd_soc_codec *codec,
				  unsigned int vodsp_reg,
				  unsigned int value)
{
	int ret = 0;

	ret = rt5625_wait_vodsp_i2c_done(codec);
	if (ret) {
		printk(KERN_ERR "%s: wait for vodsp free\n", __func__);
		return ret;
	}

	rt5625_write(codec, RT5625_VODSP_REG_ADDR, vodsp_reg);
	rt5625_write(codec, RT5625_VODSP_REG_DATA, value);
	rt5625_write(codec, RT5625_VODSP_REG_CMD,
		     VODSP_WRITE_ENABLE | VODSP_CMD_MW);

	mdelay(10);
	return ret;
}

static unsigned int rt5625_read_vodsp_reg(struct snd_soc_codec *codec,
					  unsigned int vodsp_reg)
{
	int ret = 0;
	unsigned int data_h, data_l;
	unsigned int value;

	ret = rt5625_wait_vodsp_i2c_done(codec);
	if (ret)
		goto bus_busy;
	rt5625_write(codec, RT5625_VODSP_REG_ADDR, vodsp_reg);
	rt5625_write(codec, RT5625_VODSP_REG_CMD,
		     VODSP_READ_ENABLE | VODSP_CMD_MR);

	ret = rt5625_wait_vodsp_i2c_done(codec);
	if (ret)
		goto bus_busy;
	rt5625_write(codec, RT5625_VODSP_REG_ADDR, 0x26);
	rt5625_write(codec, RT5625_VODSP_REG_CMD,
		     VODSP_READ_ENABLE | VODSP_CMD_RR);

	ret = rt5625_wait_vodsp_i2c_done(codec);
	if (ret)
		goto bus_busy;
	data_h = rt5625_direct_read(codec, RT5625_VODSP_REG_DATA);

	rt5625_write(codec, RT5625_VODSP_REG_ADDR, 0x25);
	rt5625_write(codec, RT5625_VODSP_REG_CMD,
		     VODSP_READ_ENABLE | VODSP_CMD_RR);

	ret = rt5625_wait_vodsp_i2c_done(codec);
	if (ret)
		goto bus_busy;
	data_l = rt5625_direct_read(codec, RT5625_VODSP_REG_DATA);

	value = ((data_h & 0xff) << 8) | (data_l & 0xff);

	pr_debug("vodsp_reg=0x%x, value=0x%x\n", vodsp_reg, value);
	return value;

bus_busy:
	printk(KERN_ERR "%s: wait for vodsp free\n", __func__);
	return ret;
}



static int rt5625_dump_dsp_reg(struct snd_soc_codec *codec)
{
	int i, index;

	snd_soc_update_bits(codec, RT5625_VODSP_CTL,
			  VODSP_NO_PD_MODE_ENA,
			  VODSP_NO_PD_MODE_ENA);

	for (i = 0; i < RT5625_VODSP_REG_NUM; i++) {
		index = rt5625_vodsp_init[i].vodsp_index;
		rt5625_read_vodsp_reg(codec, index);
	}

	return 0;
}

static int rt5625_dump_dsp_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int mode = rt5625_read(codec, RT5625_VIRTUAL_MISC_FUNC);

	mode &= ~(0x01 << 8);
	mode |= (ucontrol->value.integer.value[0] << 8);
	rt5625_write(codec, RT5625_VIRTUAL_MISC_FUNC, mode);
	rt5625_dump_dsp_reg(codec);

	return 0;
}

static const char *rt5625_aec_path_sel[] = {
	"pcm in pcm out", "analog in analog out",
	"dac in adc out", "disable"
};

static const char *rt5625_spk_out_sel[] = {
	"Class AB", "Class D"
};

static const char *rt5625_spk_l_source_sel[] = {
	"LPRN", "LPRP", "LPLN", "MM"
};

static const char *rt5625_spkmux_source_sel[] = {
	"VMID", "HP Mixer", "SPK Mixer", "Mono Mixer"
};

static const char *rt5625_hplmux_source_sel[] = {
	"VMID", "HPL Mixer"
};

static const char *rt5625_hprmux_source_sel[] = {
	"VMID", "HPR Mixer"
};

static const char *rt5625_auxmux_source_sel[] = {
	"VMID", "HP Mixer", "SPK Mixer", "Mono Mixer"
};

static const char *rt5625_spkamp_ratio_sel[] = {
	"2.25 Vdd", "2.00 Vdd", "1.75 Vdd",
	"1.50 Vdd", "1.25 Vdd", "1.00 Vdd"
};

static const char *rt5625_mic1_boost_sel[] = {
	"Bypass", "+20db", "+30db", "+40db"
};

static const char *rt5625_mic2_boost_sel[] = {
	"Bypass", "+20db", "+30db", "+40db"
};

static const char *rt5625_dmic_boost_sel[] = {
	"Bypass", "+6db", "+12db", "+18db",
	"+24db", "+30db", "+36db", "+42db"
};

static const char *rt5625_adcr_func_sel[] = {
	"Stereo ADC", "Voice ADC",
	"VoDSP Interface", "PDM Slave Interface"
};
static const char *rt5625_dacr_func_sel[] = {
	"Stereo ADC", "SRC2_Out",
	"VoDSP_TxDP", "VoDSP_TxDC"
};
static const struct soc_enum aec_cfg =
	SOC_ENUM_SINGLE(RT5625_VIRTUAL_MISC_FUNC, 0, 4,
			rt5625_aec_path_sel);
static const struct soc_enum spk_out =
	SOC_ENUM_SINGLE(RT5625_OUTPUT_MIXER_CTRL, 13, 2,
			rt5625_spk_out_sel);
static const struct soc_enum spk_l_source =
	SOC_ENUM_SINGLE(RT5625_OUTPUT_MIXER_CTRL, 14, 4,
			rt5625_spk_l_source_sel);
static const struct soc_enum spkmux_source =
	SOC_ENUM_SINGLE(RT5625_OUTPUT_MIXER_CTRL, 10, 4,
			rt5625_spkmux_source_sel);
static const struct soc_enum hplmux_source =
	SOC_ENUM_SINGLE(RT5625_OUTPUT_MIXER_CTRL, 9, 2,
			rt5625_hplmux_source_sel);
static const struct soc_enum hprmux_source =
	SOC_ENUM_SINGLE(RT5625_OUTPUT_MIXER_CTRL, 8, 2,
			rt5625_hprmux_source_sel);
static const struct soc_enum auxmux_source =
	SOC_ENUM_SINGLE(RT5625_OUTPUT_MIXER_CTRL, 6, 4,
			rt5625_auxmux_source_sel);
static const struct soc_enum spkamp_ratio =
	SOC_ENUM_SINGLE(RT5625_GEN_CTRL_REG1, 1, 6,
			rt5625_spkamp_ratio_sel);
static const struct soc_enum mic1_boost =
	SOC_ENUM_SINGLE(RT5625_MIC_CTRL, 10, 4,
			rt5625_mic1_boost_sel);
static const struct soc_enum mic2_boost =
	SOC_ENUM_SINGLE(RT5625_MIC_CTRL, 8, 4,
			rt5625_mic2_boost_sel);
static const struct soc_enum dmic_boost =
	SOC_ENUM_SINGLE(RT5625_DMIC_CTRL, 0, 8,
			rt5625_dmic_boost_sel);
static const struct soc_enum adcr_func =
	SOC_ENUM_SINGLE(RT5625_DAC_ADC_VODAC_FUN_SEL, 4, 4,
			rt5625_adcr_func_sel);
static const struct soc_enum dacr_func =
	SOC_ENUM_SINGLE(RT5625_DAC_ADC_VODAC_FUN_SEL, 12, 4,
			rt5625_dacr_func_sel);
static int rt5625_get_aec_mode(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int mode;

	/* * bits[0..1] store the mode type */
	mode = rt5625_read(codec, RT5625_VIRTUAL_MISC_FUNC) & 0x03;
	ucontrol->value.integer.value[0] = mode;

	return 0;
}


static int init_vodsp_aec(struct snd_soc_codec *codec)
{

	int i;
	int ret = 0;
	
	rt5625_write(codec,0x3a,0xcf8f);
	rt5625_write(codec,0x3c,0xa7ff);
	rt5625_write(codec,0x3e,0xff38);
	
	#if 0
	int dsp_inited = 0;

	if(!dsp_inited) dsp_inited++;
	else
		return 0;
	#endif

	/* enable LDO power and set output voltage to 1.2V */
	snd_soc_update_bits(codec, RT5625_LDO_CTRL,
			  LDO_ENABLE | LDO_OUT_VOL_CTRL_MASK,
			  LDO_ENABLE | LDO_OUT_VOL_CTRL_1_20V);
	mdelay(20);

	/* enable power of VODSP I2C interface */
	snd_soc_update_bits(codec, RT5625_PWR_MANAG_ADD3,
			  PWR_VODSP_INTERFACE | PWR_I2C_FOR_VODSP,
			  PWR_VODSP_INTERFACE | PWR_I2C_FOR_VODSP);
	mdelay(1);

	/* reset VODSP */
	snd_soc_update_bits(codec, RT5625_VODSP_CTL,
			   VODSP_NO_RST_MODE_ENA,0);
	mdelay(1);

	/* set VODSP to non-reset status */
	snd_soc_update_bits(codec, RT5625_VODSP_CTL,
			  VODSP_NO_RST_MODE_ENA,
			  VODSP_NO_RST_MODE_ENA);
	mdelay(20);

	/* initize AEC paramter */
	for (i = 0; i < RT5625_VODSP_REG_NUM; i++) {
		ret = rt5625_write_vodsp_reg(codec,
					     rt5625_vodsp_init[i].vodsp_index,
					     rt5625_vodsp_init[i].vodsp_value);
		if (ret)
			return -EIO;
	}

	schedule_timeout_uninterruptible(msecs_to_jiffies(100));

	/* set VODSP to pown down mode */
	snd_soc_update_bits(codec, RT5625_VODSP_CTL, VODSP_NO_PD_MODE_ENA,0);

	//rt5625_dump_dsp_reg(codec);

	return 0;
}


static int rt5625_vodsp_set_pipo_mode(struct snd_soc_codec *codec)
{
	/*
	 * 1. far end setting
	 *    far end device pcm out -> vodac pcm in -> vodsp_rxdp
	 *     (a) enable rxdp power and select rxdp source
	 *         from "voice to stereo digital path"
	 *     (b) voice pcm out from vodsp txdp
	 *         vodsp txdp -> vodac pcm out -> far end devie pcm out
	 */
	snd_soc_update_bits(codec, RT5625_VODSP_PDM_CTL,			  
			  VODSP_RXDP_PWR | VODSP_RXDP_S_SEL_MASK |
			  VOICE_PCM_S_SEL_MASK,
			 VODSP_RXDP_PWR | VODSP_RXDP_S_SEL_VOICE |
			  VOICE_PCM_S_SEL_AEC_TXDP );

	/*
	 * 2. near end setting
	 *    (a) adcr function select pdm slave interface
	 *        mic-->adcr-->pdm interface
	 *    (b) voice dac source select vodsp_txdc
	 */
	snd_soc_update_bits(codec, RT5625_DAC_ADC_VODAC_FUN_SEL,			  
			  ADCR_FUNC_SEL_MASK|VODAC_SOUR_SEL_MASK,
			 ADCR_FUNC_SEL_PDM | VODAC_SOUR_SEL_VODSP_TXDC);

	/* 3. setting vodsp lrck to 8k */
	snd_soc_update_bits(codec, RT5625_VODSP_CTL,
			  VODSP_LRCK_SEL_MASK,
			  VODSP_LRCK_SEL_8K);

	return 0;
}


static int rt5625_vodsp_set_aiao_mode(struct snd_soc_codec *codec)
{
	/*
	 * far end setting:
	 * far end device -> analog in -> adc_l -> vodsp_rxdp
	 * enable rxdp power and select rxdp source from adc_l
	 */
	rt5625_write(codec,0x26,0x0300);
	
	snd_soc_update_bits(codec, RT5625_VODSP_PDM_CTL,
			  VODSP_RXDP_PWR | VODSP_RXDP_S_SEL_MASK | VOICE_PCM_S_SEL_MASK | REC_S_SEL_MASK,
			  VODSP_RXDP_PWR | VODSP_RXDP_S_SEL_ADCL | VOICE_PCM_S_SEL_AEC_TXDP | REC_S_SEL_MASK);

	/*
	 * near end setting:
	 * vodsp txdp -> vodac -> analog out -> to far end analog input
	 * adcr function select pdm slave interface
	 * (mic-->adcr-->pdm interface)
	 */
	snd_soc_update_bits(codec, RT5625_DAC_ADC_VODAC_FUN_SEL,
			  DAC_FUNC_SEL_MASK | VODAC_SOUR_SEL_MASK | ADCR_FUNC_SEL_MASK | ADCL_FUNC_SEL_MASK,
			  DAC_FUNC_SEL_VODSP_TXDP | VODAC_SOUR_SEL_VODSP_TXDC | ADCR_FUNC_SEL_PDM | ADCL_FUNC_SEL_VODSP);

	rt5625_write(codec,0x26,0x0);

	/* 3.setting VODSP LRCK to 16k */
	snd_soc_update_bits(codec, RT5625_VODSP_CTL,
			  VODSP_LRCK_SEL_MASK,VODSP_LRCK_SEL_16K);



	return 0;
}


static int rt5625_vodsp_set_diao_mode(struct snd_soc_codec *codec)
{
	/*
	 * far end setting: playback -> src1 -> vodsp_rxdp
	 *    (a) enable src1 and vodsp_rxdp source select src1
	 */

	/*
	 * near end setting: vodsp_txdp -> src2 -> stereo record
	 * adcr function select pdm slave interface:
	 * mic -> adcr -> pdm interface
	 */
	snd_soc_update_bits(codec, RT5625_DAC_ADC_VODAC_FUN_SEL,
			  DAC_FUNC_SEL_MASK,
			  DAC_FUNC_SEL_VODSP_TXDC);
	
	snd_soc_update_bits(codec, RT5625_DAC_ADC_VODAC_FUN_SEL,
			  ADCR_FUNC_SEL_MASK,
			  ADCR_FUNC_SEL_PDM);

	/* enable src2 and select record source from src2 */
	snd_soc_update_bits(codec, RT5625_VODSP_PDM_CTL,
			  VODSP_SRC1_PWR | VODSP_SRC2_PWR |
			  VODSP_RXDP_PWR | VODSP_RXDP_S_SEL_MASK |
			  REC_S_SEL_MASK,
 			  VODSP_SRC1_PWR | VODSP_SRC2_PWR |
			  VODSP_RXDP_PWR | VODSP_RXDP_S_SEL_SRC1 |
			  REC_S_SEL_SRC2	  );

	/* setting vodsp lrck to 16k */
	snd_soc_update_bits(codec, RT5625_VODSP_CTL,
			  VODSP_LRCK_SEL_MASK,
 			  VODSP_LRCK_SEL_16K );

	return 0;
}

static int rt5625_set_vodsp_aec_path(struct snd_soc_codec *codec,
				     unsigned int mode)
{
	/*
	struct snd_soc_dapm_widget *w;

	list_for_each_entry(w, &codec->dapm_widgets, list) {
		if (!w->sname)
			continue;

		if (!strcmp(w->name, "Right ADC")) {
			if (mode != RT5625_VODSP_AEC_DISABLE)
				w->active = 1;
			else
				w->active = 0;
			}
		}
	*/
		switch (mode) {

		/*
		 * far end signal is from voice interface
		 * near end signal is from mic1/mic2
		 */
		case RT5625_PCM_IN_PCM_OUT:

			rt5625_vodsp_set_pipo_mode(codec);
			break;

		/*
		 * far end signal is from analog input
		 * near end signal is from mic1/mic2
		 */
		case RT5625_ANALOG_IN_ANALOG_OUT:
			
			rt5625_vodsp_set_aiao_mode(codec);
			break;

		/*
		 * far end signal is from playback
		 * near end signal is from mic1/mic2
		 */
		case RT5625_DAC_IN_ADC_OUT:

			rt5625_vodsp_set_diao_mode(codec);
			break;

		/* disable AEC */
		case RT5625_VODSP_AEC_DISABLE:

			/*
			 * set stereo dac, voice dac and stereo adc
			 * function select to default
			 */
			rt5625_write(codec, RT5625_DAC_ADC_VODAC_FUN_SEL, 0);

			/* set vodsp & pdm control to default */
			rt5625_write(codec, RT5625_VODSP_PDM_CTL, 0);
			break;

		default:
			printk(KERN_ERR "%s: unknown mode %d\n",
				__func__, mode);
			break;

		}

		return 0;
}

static int rt5625_enable_vodsp_aec(struct snd_soc_codec *codec,
				   unsigned int vodsp_aec_en,
				   unsigned int aec_mode)
{

	int ret = 0;

	if (vodsp_aec_en != 0) {
		/* select input/output of VODSP AEC */
		rt5625_set_vodsp_aec_path(codec, aec_mode);

		/* enable power of VODSP I2C interface & VODSP interface */
		snd_soc_update_bits(codec, RT5625_PWR_MANAG_ADD3,
				  PWR_VODSP_INTERFACE | PWR_I2C_FOR_VODSP,
				  PWR_VODSP_INTERFACE | PWR_I2C_FOR_VODSP);

		/* enable power of VODSP I2S interface */
		snd_soc_update_bits(codec, RT5625_PWR_MANAG_ADD1,
				  PWR_I2S_INTERFACE,
				  PWR_I2S_INTERFACE);

		/* set VODSP to active */
		snd_soc_update_bits(codec, RT5625_VODSP_CTL,
				  VODSP_NO_PD_MODE_ENA,
				  VODSP_NO_PD_MODE_ENA);

		mdelay(50);
	} else {
		/* set VODSP AEC to power down mode */
		snd_soc_update_bits(codec, RT5625_VODSP_CTL,
				  VODSP_NO_PD_MODE_ENA,0);
		/* disable power of VODSP I2C interface & VODSP interface */
		snd_soc_update_bits(codec, RT5625_PWR_MANAG_ADD3,
				  PWR_VODSP_INTERFACE | PWR_I2C_FOR_VODSP,
				  0);

		/* disable VODSP AEC path */
		rt5625_set_vodsp_aec_path(codec, RT5625_VODSP_AEC_DISABLE);
	}

	return ret;
}



static void rt5625_aec_config(struct snd_soc_codec *codec, unsigned int mode)
{

	int ret;
	unsigned int aec_enable = 1;

	ret = init_vodsp_aec(codec);
	if(ret)
	{
		printk("init vodsp failed\n");
	}
	if (mode == RT5625_VODSP_AEC_DISABLE)
		aec_enable = 0;

	rt5625_enable_vodsp_aec(codec, aec_enable, mode);

}

static int rt5625_set_aec_mode(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int mode;

	mode = rt5625_read(codec, RT5625_VIRTUAL_MISC_FUNC);
	if (ucontrol->value.integer.value[0] != 0) {
		if ((mode & 0x03) == ucontrol->value.integer.value[0])
			return 0;
	}

	mode &= 0xfffc;
	mode |= ucontrol->value.integer.value[0];
	rt5625_write(codec, RT5625_VIRTUAL_MISC_FUNC, mode);
	rt5625_aec_config(codec, ucontrol->value.integer.value[0]);

	return 1;
}

static const struct snd_kcontrol_new rt5625_snd_controls[] = {
SOC_ENUM_EXT("AEC Mode", aec_cfg, rt5625_get_aec_mode, rt5625_set_aec_mode),
SOC_ENUM("SPK Amp Type", spk_out),
SOC_ENUM("Left SPK Source", spk_l_source),
SOC_ENUM("SPK Amp Ratio", spkamp_ratio),
SOC_ENUM("Mic1 Amp Boost Type", mic1_boost),
SOC_ENUM("Mic2 Amp Boost Type", mic2_boost),
SOC_ENUM("Dmic Boost", dmic_boost),
SOC_ENUM("ADCR Func", adcr_func),
SOC_ENUM("DACR Func", dacr_func),

SOC_DOUBLE("PCM Playback Volume", RT5625_STEREO_DAC_VOL, 8, 0, 63, 1),
SOC_DOUBLE("LineIn Playback Volume", RT5625_LINE_IN_VOL, 8, 0, 31, 1),
SOC_SINGLE("Phone Playback Volume", RT5625_PHONEIN_VOL, 8, 31, 1),
SOC_SINGLE("Mic1 Playback Volume", RT5625_MIC_VOL, 8, 31, 1),
SOC_SINGLE("Mic2 Playback Volume", RT5625_MIC_VOL, 0, 31, 1),
SOC_DOUBLE("PCM Capture Volume", RT5625_ADC_REC_GAIN, 8, 0, 31, 0),
SOC_DOUBLE("SPKOUT Playback Volume", RT5625_SPK_OUT_VOL, 8, 0, 31, 1),
SOC_DOUBLE("SPKOUT Playback Switch", RT5625_SPK_OUT_VOL, 15, 7, 1, 1),
SOC_DOUBLE("HPOUT Playback Volume", RT5625_HP_OUT_VOL, 8, 0, 31, 1),
SOC_DOUBLE("HPOUT Playback Switch", RT5625_HP_OUT_VOL, 15, 7, 1, 1),
SOC_DOUBLE("AUXOUT Playback Volume", RT5625_AUX_OUT_VOL, 8, 0, 31, 1),
SOC_DOUBLE("AUXOUT Playback Switch", RT5625_AUX_OUT_VOL, 15, 7, 1, 1),
SOC_SINGLE_EXT("VoDSP Dump", RT5625_VIRTUAL_MISC_FUNC, 8, 1, 0,
	       snd_soc_get_volsw, rt5625_dump_dsp_put),
SOC_SINGLE("Voice Volume",RT5625_VOICE_DAC_OUT_VOL,0,63,1),	       
SOC_SINGLE("Voice CLK Power",RT5625_PWR_MANAG_ADD2,10,1,0),	  
SOC_SINGLE("Voice DAC Power",RT5625_PWR_MANAG_ADD1,7,1,0), 	  
SOC_SINGLE("Voice DAC Mute",RT5625_VOICE_DAC_OUT_VOL,12,1,0),	  
SOC_SINGLE("Voice DAC Playback Switch",RT5625_VOICE_DAC_OUT_VOL,14,1,0),	  
SOC_SINGLE("Voice DAC Enable",RT5625_EXTEND_SDP_CTRL,15,1,0),	  

};

static void hp_depop_mode2(struct snd_soc_codec *codec)
{
//        snd_soc_update_bits(codec, RT5625_PWR_MANAG_ADD1,
//			  PWR_MAIN_BIAS, PWR_MAIN_BIAS);
        snd_soc_update_bits(codec, RT5625_HP_OUT_VOL,
			  M_HP_L|M_HP_R, M_HP_L|M_HP_R);
		//rt5625_write(codec,0x04,0x0707);

        snd_soc_update_bits(codec, RT5625_PWR_MANAG_ADD2,
			  PWR_MIXER_VREF, PWR_MIXER_VREF);
        snd_soc_update_bits(codec, RT5625_PWR_MANAG_ADD1,
			  PWR_SOFTGEN_EN, PWR_SOFTGEN_EN);
        snd_soc_update_bits(codec, RT5625_PWR_MANAG_ADD3,
			  PWR_HP_R_OUT_VOL | PWR_HP_L_OUT_VOL,
			  PWR_HP_R_OUT_VOL | PWR_HP_L_OUT_VOL);
        snd_soc_update_bits(codec, RT5625_MISC_CTRL,
			  HP_DEPOP_MODE2_EN,
			  HP_DEPOP_MODE2_EN);
        schedule_timeout_uninterruptible(msecs_to_jiffies(300));

}


/* _dapm_ Controls */

/* left adc rec mixer */
static const struct snd_kcontrol_new rt5625_left_adc_rec_mixer_controls[] = {
SOC_DAPM_SINGLE("Mic1 Capture Switch", RT5625_ADC_REC_MIXER, 14, 1, 1),
SOC_DAPM_SINGLE("Mic2 Capture Switch", RT5625_ADC_REC_MIXER, 13, 1, 1),
SOC_DAPM_SINGLE("LineIn Capture Switch", RT5625_ADC_REC_MIXER, 12, 1, 1),
SOC_DAPM_SINGLE("Phone Capture Switch", RT5625_ADC_REC_MIXER, 11, 1, 1),
SOC_DAPM_SINGLE("HP Mixer Capture Switch", RT5625_ADC_REC_MIXER, 10, 1, 1),
SOC_DAPM_SINGLE("MoNo Mixer Capture Switch", RT5625_ADC_REC_MIXER, 8, 1, 1),
SOC_DAPM_SINGLE("SPK Mixer Capture Switch", RT5625_ADC_REC_MIXER, 9, 1, 1),
};

/* right adc rec mixer */
static const struct snd_kcontrol_new rt5625_right_adc_rec_mixer_controls[] = {
SOC_DAPM_SINGLE("Mic1 Capture Switch", RT5625_ADC_REC_MIXER, 6, 1, 1),
SOC_DAPM_SINGLE("Mic2 Capture Switch", RT5625_ADC_REC_MIXER, 5, 1, 1),
SOC_DAPM_SINGLE("LineIn Capture Switch", RT5625_ADC_REC_MIXER, 4, 1, 1),
SOC_DAPM_SINGLE("Phone Capture Switch", RT5625_ADC_REC_MIXER, 3, 1, 1),
SOC_DAPM_SINGLE("HP Mixer Capture Switch", RT5625_ADC_REC_MIXER, 2, 1, 1),
SOC_DAPM_SINGLE("MoNo Mixer Capture Switch", RT5625_ADC_REC_MIXER, 0, 1, 1),
SOC_DAPM_SINGLE("SPK Mixer Capture Switch", RT5625_ADC_REC_MIXER, 1, 1, 1),
};

static const struct snd_kcontrol_new rt5625_left_hp_mixer_controls[] = {
SOC_DAPM_SINGLE("ADC Playback Switch", RT5625_ADC_REC_GAIN, 15, 1, 1),
SOC_DAPM_SINGLE("LineIn Playback Switch", RT5625_HPL_MIXER, 0, 1, 0),
SOC_DAPM_SINGLE("Phone Playback Switch", RT5625_HPL_MIXER, 1, 1, 0),
SOC_DAPM_SINGLE("Mic1 Playback Switch", RT5625_HPL_MIXER, 2, 1, 0),
SOC_DAPM_SINGLE("Mic2 Playback Switch", RT5625_HPL_MIXER, 3, 1, 0),
SOC_DAPM_SINGLE("HIFI DAC Playback Switch", RT5625_DAC_AND_MIC_CTRL, 3, 1, 1),
SOC_DAPM_SINGLE("Voice DAC Playback Switch", RT5625_HPL_MIXER, 4, 1, 0),
};

static const struct snd_kcontrol_new rt5625_right_hp_mixer_controls[] = {
SOC_DAPM_SINGLE("ADC Playback Switch", RT5625_ADC_REC_GAIN, 7, 1, 1),
SOC_DAPM_SINGLE("LineIn Playback Switch", RT5625_HPR_MIXER, 0, 1, 0),
SOC_DAPM_SINGLE("Phone Playback Switch", RT5625_HPR_MIXER, 1, 1, 0),
SOC_DAPM_SINGLE("Mic1 Playback Switch", RT5625_HPR_MIXER, 2, 1, 0),
SOC_DAPM_SINGLE("Mic2 Playback Switch", RT5625_HPR_MIXER, 3, 1, 0),
SOC_DAPM_SINGLE("HIFI DAC Playback Switch", RT5625_DAC_AND_MIC_CTRL, 2, 1, 1),
SOC_DAPM_SINGLE("Voice DAC Playback Switch", RT5625_HPR_MIXER, 4, 1, 0),
};

static const struct snd_kcontrol_new rt5625_mono_mixer_controls[] = {
SOC_DAPM_SINGLE("ADCL Playback Switch", RT5625_ADC_REC_GAIN, 14, 1, 1),
SOC_DAPM_SINGLE("ADCR Playback Switch", RT5625_ADC_REC_GAIN, 6, 1, 1),
SOC_DAPM_SINGLE("Line Mixer Playback Switch", RT5625_LINE_IN_VOL, 13, 1, 1),
SOC_DAPM_SINGLE("Mic1 Playback Switch", RT5625_DAC_AND_MIC_CTRL, 13, 1, 1),
SOC_DAPM_SINGLE("Mic2 Playback Switch", RT5625_DAC_AND_MIC_CTRL, 9, 1, 1),
SOC_DAPM_SINGLE("DAC Mixer Playback Switch", RT5625_DAC_AND_MIC_CTRL, 0, 1, 1),
SOC_DAPM_SINGLE("Voice DAC Playback Switch",
	        RT5625_VOICE_DAC_OUT_VOL, 13, 1, 1),
};

static const struct snd_kcontrol_new rt5625_spk_mixer_controls[] = {
SOC_DAPM_SINGLE("Line Mixer Playback Switch", RT5625_LINE_IN_VOL, 14, 1, 1),
SOC_DAPM_SINGLE("Phone Playback Switch", RT5625_PHONEIN_VOL, 14, 1, 1),
SOC_DAPM_SINGLE("Mic1 Playback Switch", RT5625_DAC_AND_MIC_CTRL, 14, 1, 1),
SOC_DAPM_SINGLE("Mic2 Playback Switch", RT5625_DAC_AND_MIC_CTRL, 10, 1, 1),
SOC_DAPM_SINGLE("DAC Mixer Playback Switch", RT5625_DAC_AND_MIC_CTRL, 1, 1, 1),
SOC_DAPM_SINGLE("Voice DAC Playback Switch",
		RT5625_VOICE_DAC_OUT_VOL, 14, 1, 1),
};

static int mixer_event(struct snd_soc_dapm_widget *w,
		       struct snd_kcontrol *k,
		       int event)
{
	struct snd_soc_codec *codec = w->codec;
	unsigned int l, r;

	pr_debug("enter %s\n", __func__);

	l = rt5625_read(codec, RT5625_HPL_MIXER);
	r = rt5625_read(codec, RT5625_HPR_MIXER);

	if ((l & 0x1) || (r & 0x1))
		snd_soc_update_bits(codec, RT5625_LINE_IN_VOL,
				  M_LINEIN_TO_HP_MIXER,
				  0);
	else
		snd_soc_update_bits(codec, RT5625_LINE_IN_VOL,
				  M_LINEIN_TO_HP_MIXER,
				  M_LINEIN_TO_HP_MIXER);
	/*
	if ((l & 0x2) || (r & 0x2))
		snd_soc_update_bits(codec, RT5625_PHONEIN_VOL,
				  0,
				  M_PHONEIN_TO_HP_MIXER);
	else
		snd_soc_update_bits(codec, RT5625_PHONEIN_VOL,
				  M_PHONEIN_TO_HP_MIXER,
				  M_PHONEIN_TO_HP_MIXER);
	*/
	if ((l & 0x4) || (r & 0x4))
		snd_soc_update_bits(codec, RT5625_DAC_AND_MIC_CTRL,
				  M_MIC1_TO_HP_MIXER,
				  0);
	else
		snd_soc_update_bits(codec, RT5625_DAC_AND_MIC_CTRL,
				  M_MIC1_TO_HP_MIXER,
				  M_MIC1_TO_HP_MIXER);

	if ((l & 0x8) || (r & 0x8))
		snd_soc_update_bits(codec, RT5625_DAC_AND_MIC_CTRL,
				  M_MIC2_TO_HP_MIXER,0);
	else
		snd_soc_update_bits(codec, RT5625_DAC_AND_MIC_CTRL,
				  M_MIC2_TO_HP_MIXER,
				  M_MIC2_TO_HP_MIXER);

	if ((l & 0x10) || (r & 0x10))
		snd_soc_update_bits(codec, RT5625_VOICE_DAC_OUT_VOL,
			          0x8000, M_V_DAC_TO_HP_MIXER);
	else
		snd_soc_update_bits(codec, RT5625_VOICE_DAC_OUT_VOL,
			          0x8000, M_V_DAC_TO_HP_MIXER);

	return 0;
}

/*
 * bit[0][1]:	use for aec control
 * bit[2][3]:	for ADCR func
 * bit[4]:	for SPKL pga
 * bit[5]:	for SPKR pga
 * bit[6]:	for hpl pga
 * bit[7]:	for hpr pga
 */
static int spk_pga_event(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *k, int event)
{
	struct snd_soc_codec *codec = w->codec;
	int val;

	pr_debug("enter %s\n", __func__);

	val = rt5625_read(codec, RT5625_VIRTUAL_MISC_FUNC);
	val = (val & (0x3 << 4)) >> 4;
	pr_debug("%s: val = %d, event = %d\n", __func__, val, event);
	//if (val != 0x3 && !val)
	//	return 0;

	switch (event) {

	case SND_SOC_DAPM_POST_PMU:
		pr_debug("after virtual spk power up!\n");

		snd_soc_update_bits(codec, RT5625_PWR_MANAG_ADD3,
				  PWR_SPK_L_OUT_VOL |
				  PWR_SPK_R_OUT_VOL,
				  PWR_SPK_L_OUT_VOL |
				  PWR_SPK_R_OUT_VOL);
		snd_soc_update_bits(codec, RT5625_SPK_OUT_VOL,
				  M_SPK_L | M_SPK_R,
				  0);
		/* power on spk amp */
		snd_soc_update_bits(codec, RT5625_PWR_MANAG_ADD1,
				  PWR_AMP_POWER, PWR_AMP_POWER);
		break;

	case SND_SOC_DAPM_POST_PMD:
		pr_debug("aftet virtual spk power down!\n");

		/* power off spk amp */
		snd_soc_update_bits(codec, RT5625_PWR_MANAG_ADD1,
				  PWR_AMP_POWER,0);
		snd_soc_update_bits(codec, RT5625_SPK_OUT_VOL,
				  M_SPK_L | M_SPK_R,
				  M_SPK_L | M_SPK_R);
		snd_soc_update_bits(codec, RT5625_PWR_MANAG_ADD3,
				  PWR_SPK_L_OUT_VOL |
				  PWR_SPK_R_OUT_VOL,0);
		break;

	default:
		break;
	}

	return 0;
}

static int hp_pga_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *k,
			int event)
{
	struct snd_soc_codec *codec = w->codec;
	int val,misc;

	pr_debug("enter %s\n", __func__);

	val = rt5625_read(codec, RT5625_VIRTUAL_MISC_FUNC);
	val = (val & (0x3 << 6)) >> 6;
	pr_debug("%s: val = %d, event = %d\n", __func__, val, event);
	//if (val != 0x3 && !val)
	//	return 0;

	misc = rt5625_read(codec, RT5625_VIRTUAL_MISC2);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMD:
		{
			if(misc&0x01)
			{
				hp_depop_mode2(codec);
				snd_soc_update_bits(codec,RT5625_VIRTUAL_MISC2,0x01,0x00);				
			}				
		}
		break;

	case SND_SOC_DAPM_POST_PMD:
		{
			//snd_soc_update_bits(codec, RT5625_PWR_MANAG_ADD3,
			//		  M_HP_L|M_HP_R,0);
		}
		break;

	case SND_SOC_DAPM_POST_PMU:	break;

	case SND_SOC_DAPM_PRE_PMU:
		{
			if(0==(misc&0x01))
			{
				hp_depop_mode2(codec);
				snd_soc_update_bits(codec,RT5625_VIRTUAL_MISC2,0x01,0x01);				
			}
				
		}
		break;

	default:break;
	}

	return 0;
}

static int aux_pga_event(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *k, int event)
{
	return 0;
}
static int mic_pga_event(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *k, int event)
{
	struct snd_soc_codec *codec = w->codec;

	//because we bypass mic pga(boost) so power up/down pga 
	switch (event) {
	case SND_SOC_DAPM_POST_PMD:
		{
		}
		break;
	case SND_SOC_DAPM_POST_PMU:
		{
			unsigned int mic_reg = rt5625_read(codec,RT5625_MIC_CTRL);
			unsigned int boost_power = rt5625_read(codec,RT5625_PWR_MANAG_ADD3);
			if(!strcmp(w->name,"Mic1 Boost"))
			{
				if((MIC1_BOOST_CONTROL_BYPASS==(mic_reg&MIC1_BOOST_CONTROL_MASK))&&
					(PWR_MIC1_BOOST&boost_power))
				{
					snd_soc_update_bits(codec,RT5625_PWR_MANAG_ADD3,0x02,0x00);
				}
			}else if(!strcmp(w->name,"Mic2 Boost"))
			{
				if((MIC2_BOOST_CONTROL_BYPASS==(mic_reg&MIC2_BOOST_CONTROL_MASK))&&
					(PWR_MIC2_BOOST&boost_power))
				{
					snd_soc_update_bits(codec,RT5625_PWR_MANAG_ADD3,0x01,0x00);
				}				
			}
			//schedule_timeout_uninterruptible(msecs_to_jiffies(300));
			
		}	
		break;

	case SND_SOC_DAPM_PRE_PMD:
	case SND_SOC_DAPM_PRE_PMU:
	default:break;
	}


	return 0;
}

static int mic_bias_event(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *k, int event)
{
	switch (event) {
	case SND_SOC_DAPM_POST_PMD:
		{
			/*
			if(!strcmp(w->name,"Mic1 Bias"))
			{
				printk("power down mic1 boost\n");
				snd_soc_update_bits(codec,RT5625_PWR_MANAG_ADD3,0x02,0x00);
			}
			else if(!strcmp(w->name,"Mic2 Bias"))
			{
				printk("power down mic2 boost\n");
				snd_soc_update_bits(codec,RT5625_PWR_MANAG_ADD3,0x01,0x00);
			}*/			
			//schedule_timeout_uninterruptible(msecs_to_jiffies(500));
		}
		break;
	case SND_SOC_DAPM_PRE_PMU:
		{
			/*
			if(!strcmp(w->name,"Mic1 Bias"))
			{
				printk("power up mic1 boost\n");
				snd_soc_update_bits(codec,RT5625_PWR_MANAG_ADD3,0x02,0x02);
			}
			else if(!strcmp(w->name,"Mic2 Bias"))
			{
				printk("power up mic2 boost\n");
				snd_soc_update_bits(codec,RT5625_PWR_MANAG_ADD3,0x01,0x01);
			}			
			schedule_timeout_uninterruptible(msecs_to_jiffies(300));
			*/
		}	
		break;
		case SND_SOC_DAPM_POST_PMU:
			//schedule_timeout_uninterruptible(msecs_to_jiffies(300));
			break;

	case SND_SOC_DAPM_PRE_PMD:
	default:break;
	}

	return 0;
}

/* spkout mux */
static const struct snd_kcontrol_new rt5625_spkout_mux_out_controls =
SOC_DAPM_ENUM("Route", spkmux_source);

/* hplout mux */
static const struct snd_kcontrol_new rt5625_hplout_mux_out_controls =
SOC_DAPM_ENUM("Route", hplmux_source);

/* hprout mux */
static const struct snd_kcontrol_new rt5625_hprout_mux_out_controls =
SOC_DAPM_ENUM("Route", hprmux_source);

/* auxout mux */
static const struct snd_kcontrol_new rt5625_auxout_mux_out_controls =
SOC_DAPM_ENUM("Route", auxmux_source);

static const struct snd_soc_dapm_widget rt5625_dapm_widgets[] = {
SND_SOC_DAPM_INPUT("Left LineIn"),
SND_SOC_DAPM_INPUT("Right LineIn"),
SND_SOC_DAPM_INPUT("Phone"),
SND_SOC_DAPM_INPUT("Mic1"),
SND_SOC_DAPM_INPUT("Mic2"),

SND_SOC_DAPM_PGA_E("Mic1 Boost", RT5625_PWR_MANAG_ADD3, 1, 0, NULL, 0,
	mic_pga_event,
	SND_SOC_DAPM_POST_PMD | SND_SOC_DAPM_PRE_PMD|
	SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMU),
SND_SOC_DAPM_PGA_E("Mic2 Boost", RT5625_PWR_MANAG_ADD3, 0, 0, NULL, 0,
	mic_pga_event,
	SND_SOC_DAPM_POST_PMD | SND_SOC_DAPM_PRE_PMD|
	SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMU),

SND_SOC_DAPM_DAC("Left DAC", "Left HiFi Playback DAC",
		 RT5625_PWR_MANAG_ADD2, 9, 0),
SND_SOC_DAPM_DAC("Right DAC", "Right HiFi Playback DAC",
		 RT5625_PWR_MANAG_ADD2, 8, 0),
SND_SOC_DAPM_DAC("Voice DAC", "Voice Playback DAC",
		 RT5625_PWR_MANAG_ADD2, 10, 0),

SND_SOC_DAPM_PGA("Left LineIn PGA", RT5625_PWR_MANAG_ADD3, 7, 0, NULL, 0),
SND_SOC_DAPM_PGA("Right LineIn PGA", RT5625_PWR_MANAG_ADD3, 6, 0, NULL, 0),
SND_SOC_DAPM_PGA("Phone PGA", RT5625_PWR_MANAG_ADD3, 5, 0, NULL, 0),
SND_SOC_DAPM_PGA("Mic1 PGA", RT5625_PWR_MANAG_ADD3, 3, 0, NULL, 0),
SND_SOC_DAPM_PGA("Mic2 PGA", RT5625_PWR_MANAG_ADD3, 2, 0, NULL, 0),
SND_SOC_DAPM_PGA("Left DAC PGA", RT5625_PWR_MANAG_ADD1, 15, 0, NULL, 0),
SND_SOC_DAPM_PGA("Right DAC PGA", RT5625_PWR_MANAG_ADD1, 14, 0, NULL, 0),
SND_SOC_DAPM_PGA("VoDAC PGA", RT5625_PWR_MANAG_ADD1, 7, 0, NULL, 0),

SND_SOC_DAPM_MIXER("Left Rec Mixer", RT5625_PWR_MANAG_ADD2, 1, 0,
		   &rt5625_left_adc_rec_mixer_controls[0],
		   ARRAY_SIZE(rt5625_left_adc_rec_mixer_controls)),
SND_SOC_DAPM_MIXER("Right Rec Mixer", RT5625_PWR_MANAG_ADD2, 0, 0,
		   &rt5625_right_adc_rec_mixer_controls[0],
		   ARRAY_SIZE(rt5625_right_adc_rec_mixer_controls)),
SND_SOC_DAPM_MIXER_E("Left HP Mixer", RT5625_PWR_MANAG_ADD2, 5, 0,
		     &rt5625_left_hp_mixer_controls[0],
		     ARRAY_SIZE(rt5625_left_hp_mixer_controls),
		     mixer_event, SND_SOC_DAPM_POST_REG),
SND_SOC_DAPM_MIXER_E("Right HP Mixer", RT5625_PWR_MANAG_ADD2, 4, 0,
		     &rt5625_right_hp_mixer_controls[0],
		     ARRAY_SIZE(rt5625_right_hp_mixer_controls),
		     mixer_event, SND_SOC_DAPM_POST_REG),
SND_SOC_DAPM_MIXER("MoNo Mixer", RT5625_PWR_MANAG_ADD2, 2, 0,
		   &rt5625_mono_mixer_controls[0],
		   ARRAY_SIZE(rt5625_mono_mixer_controls)),
SND_SOC_DAPM_MIXER("SPK Mixer", RT5625_PWR_MANAG_ADD2, 3, 0,
		   &rt5625_spk_mixer_controls[0],
		   ARRAY_SIZE(rt5625_spk_mixer_controls)),

/*
 * hpl mixer -> hp mixer -> spkout mux
 * hpr mixer -> hp mixer -> spkout mux
 * hpl mixer -> hp mixer -> auxout mux
 * hpr muxer -> hp mixer -> auxout mux
 */
SND_SOC_DAPM_MIXER("HP Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
SND_SOC_DAPM_MIXER("DAC Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
SND_SOC_DAPM_MIXER("Line Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),

SND_SOC_DAPM_MUX("SPKOUT Mux", SND_SOC_NOPM, 0, 0,
		 &rt5625_spkout_mux_out_controls),
SND_SOC_DAPM_MUX("HPLOUT Mux", SND_SOC_NOPM, 0, 0,
		 &rt5625_hplout_mux_out_controls),
SND_SOC_DAPM_MUX("HPROUT Mux", SND_SOC_NOPM, 0, 0,
		 &rt5625_hprout_mux_out_controls),
SND_SOC_DAPM_MUX("AUXOUT Mux", SND_SOC_NOPM, 0, 0,
		 &rt5625_auxout_mux_out_controls),

SND_SOC_DAPM_PGA_E("SPKL Out PGA",
		   RT5625_VIRTUAL_MISC_FUNC, 4, 0, NULL, 0,
		   spk_pga_event,
		   SND_SOC_DAPM_POST_PMU |
		   SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_PGA_E("SPKR Out PGA",
		   RT5625_VIRTUAL_MISC_FUNC, 5, 0, NULL, 0,
		   spk_pga_event,
		   SND_SOC_DAPM_POST_PMU |
		   SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_PGA_E("HPL Out PGA",
		   RT5625_VIRTUAL_MISC_FUNC, 6, 0, NULL, 0,
		   hp_pga_event,
		   SND_SOC_DAPM_POST_PMD | SND_SOC_DAPM_PRE_PMD|
		   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMU),
SND_SOC_DAPM_PGA_E("HPR Out PGA",
		   RT5625_VIRTUAL_MISC_FUNC, 7, 0, NULL, 0,
		   hp_pga_event,
		   SND_SOC_DAPM_POST_PMD | SND_SOC_DAPM_PRE_PMD|
		   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMU),
SND_SOC_DAPM_PGA_E("AUX Out PGA",
		   RT5625_PWR_MANAG_ADD3, 14, 0, NULL, 0,
		   aux_pga_event,
		   SND_SOC_DAPM_PRE_PMD |
		   SND_SOC_DAPM_POST_PMU),

SND_SOC_DAPM_ADC("Left ADC", "Left ADC HiFi Capture",
		 RT5625_PWR_MANAG_ADD2, 7, 0),
SND_SOC_DAPM_ADC("Right ADC", "Right ADC HiFi Capture",
		 RT5625_PWR_MANAG_ADD2, 6, 0),

SND_SOC_DAPM_OUTPUT("SPKL"),
SND_SOC_DAPM_OUTPUT("SPKR"),
SND_SOC_DAPM_OUTPUT("HPL"),
SND_SOC_DAPM_OUTPUT("HPR"),
SND_SOC_DAPM_OUTPUT("AUX"),
SND_SOC_DAPM_INPUT("PCM"),



SND_SOC_DAPM_MICBIAS_E("Mic1 Bias", RT5625_PWR_MANAG_ADD1, 3, 0,
	mic_bias_event,
	SND_SOC_DAPM_POST_PMD | SND_SOC_DAPM_PRE_PMD|
	SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMU),
SND_SOC_DAPM_MICBIAS_E("Mic2 Bias", RT5625_PWR_MANAG_ADD1, 2, 0,
	mic_bias_event,
	SND_SOC_DAPM_POST_PMD | SND_SOC_DAPM_PRE_PMD|
	SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMU),
};



static const struct snd_soc_dapm_route rt5625_dapm_routes[] = {

	/* input pga */
	{"Left LineIn PGA", NULL, "Left LineIn"},
	{"Right LineIn PGA", NULL, "Right LineIn"},
	{"Phone PGA", NULL, "Phone"},
	//pga<--boost<--bias<--mic
	{"Mic1 Bias", NULL, "Mic1"},
	{"Mic2 Bias", NULL, "Mic2"},
	{"Mic1 Boost", NULL, "Mic1 Bias"},
	{"Mic2 Boost", NULL, "Mic2 Bias"},	
	{"Mic1 PGA", NULL, "Mic1 Boost"},
	{"Mic2 PGA", NULL, "Mic2 Boost"},
	{"VoDAC PGA", NULL, "Voice DAC"},
	{"VoDAC PGA",NULL,"PCM"},
	/* left ADC mixer */
	{"Left Rec Mixer", "LineIn Capture Switch", "Left LineIn"},
	{"Left Rec Mixer", "Phone Capture Switch", "Phone"},
	{"Left Rec Mixer", "Mic1 Capture Switch", "Mic1 Boost"},
	{"Left Rec Mixer", "Mic2 Capture Switch", "Mic2 Boost"},
	{"Left Rec Mixer", "HP Mixer Capture Switch", "Left HP Mixer"},
	{"Left Rec Mixer", "SPK Mixer Capture Switch", "SPK Mixer"},
	{"Left Rec Mixer", "MoNo Mixer Capture Switch", "MoNo Mixer"},

	/* right adc mixer */
	{"Right Rec Mixer", "LineIn Capture Switch", "Right LineIn"},
	{"Right Rec Mixer", "Phone Capture Switch", "Phone"},
	{"Right Rec Mixer", "Mic1 Capture Switch", "Mic1 Boost"},
	{"Right Rec Mixer", "Mic2 Capture Switch", "Mic2 Boost"},
	{"Right Rec Mixer", "HP Mixer Capture Switch", "Right HP Mixer"},
	{"Right Rec Mixer", "SPK Mixer Capture Switch", "SPK Mixer"},
	{"Right Rec Mixer", "MoNo Mixer Capture Switch", "MoNo Mixer"},

	/* hpl mixer */
	{"Left HP Mixer", "ADC Playback Switch", "Left Rec Mixer"},
	{"Left HP Mixer", "LineIn Playback Switch", "Left LineIn PGA"},
	{"Left HP Mixer", "Phone Playback Switch", "Phone PGA"},
	{"Left HP Mixer", "Mic1 Playback Switch", "Mic1 PGA"},
	{"Left HP Mixer", "Mic2 Playback Switch", "Mic2 PGA"},
	{"Left HP Mixer", "HIFI DAC Playback Switch", "Left DAC PGA"},
	{"Left HP Mixer", "Voice DAC Playback Switch", "VoDAC PGA"},

	/* hpr mixer */
	{"Right HP Mixer", "ADC Playback Switch", "Right Rec Mixer"},
	{"Right HP Mixer", "LineIn Playback Switch", "Right LineIn PGA"},
	{"Right HP Mixer", "HIFI DAC Playback Switch", "Right DAC PGA"},
	{"Right HP Mixer", "Phone Playback Switch", "Phone PGA"},
	{"Right HP Mixer", "Mic1 Playback Switch", "Mic1 PGA"},
	{"Right HP Mixer", "Mic2 Playback Switch", "Mic2 PGA"},
	{"Right HP Mixer", "Voice DAC Playback Switch", "VoDAC PGA"},

	/* dac mixer */
	{"Left DAC PGA", NULL, "Left DAC"},
	{"Right DAC PGA", NULL, "Right DAC"},

	{"DAC Mixer", NULL, "Left DAC PGA"},
	{"DAC Mixer", NULL, "Right DAC PGA"},
	

	/*line mixer*/
	{"Line Mixer", NULL, "Left LineIn PGA"},
	{"Line Mixer", NULL, "Right LineIn PGA"},

	/*spk mixer*/
	{"SPK Mixer", "Line Mixer Playback Switch", "Line Mixer"},
	{"SPK Mixer", "Phone Playback Switch", "Phone PGA"},
	{"SPK Mixer", "Mic1 Playback Switch", "Mic1 PGA"},
	{"SPK Mixer", "Mic2 Playback Switch", "Mic2 PGA"},
	{"SPK Mixer", "DAC Mixer Playback Switch", "DAC Mixer"},
	{"SPK Mixer", "Voice DAC Playback Switch", "VoDAC PGA"},

	/*mono mixer*/
	{"MoNo Mixer", "Line Mixer Playback Switch", "Line Mixer"},
	{"MoNo Mixer", "ADCL Playback Switch","Left Rec Mixer"},
	{"MoNo Mixer", "ADCR Playback Switch","Right Rec Mixer"},
	{"MoNo Mixer", "Mic1 Playback Switch", "Mic1 PGA"},
	{"MoNo Mixer", "Mic2 Playback Switch", "Mic2 PGA"},
	{"MoNo Mixer", "DAC Mixer Playback Switch", "DAC Mixer"},
	{"MoNo Mixer", "Voice DAC Playback Switch", "VoDAC PGA"},

	/* hp mixer */
	{"HP Mixer", NULL, "Left HP Mixer"},
	{"HP Mixer", NULL, "Right HP Mixer"},

	/* spkout mux */
	{"SPKOUT Mux", "HP Mixer", "HP Mixer"},
	{"SPKOUT Mux", "SPK Mixer", "SPK Mixer"},
	{"SPKOUT Mux", "Mono Mixer", "MoNo Mixer"},

	/* hpl out mux */
	{"HPLOUT Mux", "HPL Mixer", "Left HP Mixer"},

	/* hpr out mux */
	{"HPROUT Mux", "HPR Mixer", "Right HP Mixer"},

	/* aux out mux */
	{"AUXOUT Mux", "HP Mixer", "HP Mixer"},
	{"AUXOUT Mux", "SPK Mixer", "SPK Mixer"},
	{"SPKOUT Mux", "Mono Mixer", "MoNo Mixer"},

	/* spkl out pga */
	{"SPKL Out PGA", NULL, "SPKOUT Mux"},

	/* spkr out pga */
	{"SPKR Out PGA", NULL, "SPKOUT Mux"},

	/* hpl out pga */
	{"HPL Out PGA", NULL, "HPLOUT Mux"},

	/* hpr out pga */
	{"HPR Out PGA", NULL, "HPROUT Mux"},

	/* aux out pga */
	{"AUX Out PGA", NULL, "AUXOUT Mux"},

	/* left adc */
	{"Left ADC", NULL, "Left Rec Mixer"},

	/* right adc */
	{"Right ADC", NULL, "Right Rec Mixer"},

	/* output */
	{"SPKL", NULL, "SPKL Out PGA"},
	{"SPKR", NULL, "SPKR Out PGA"},
	{"HPL", NULL, "HPL Out PGA"},
	{"HPR", NULL, "HPR Out PGA"},
	{"AUX", NULL, "AUX Out PGA"},
};



struct rt5625_pll_div {
	u32 pll_in;
	u32 pll_out;
	u16 pll_ctrl;
};

/*
 * Note: the codec support to select different source as pll input;
 * but if use both of the I2S audio interface
 * and pcm interface instantially,
 * the two DAI must have the same pll setting params,
 * so you have to offer the same pll input,
 * and set our codec's sysclk the same one.
 */
static const struct rt5625_pll_div pll1_div_from_mclk[] = {
	{  2048000, 24576000, 0x2ea0 },
	{  3686400, 24576000, 0xee27 },
	{ 12000000, 24576000, 0x2915 },//48*512
	{ 13000000, 24576000, 0x772e },
	{ 13100000, 24576000, 0x0d20 },
	{ 18432000, 24576000, 0x0290 },
	{ 12288000, 24576000, 0x0490 },
	{  7872000, 24576000, 0x6519 },
	{  6144000, 24576000, 0x0a90 },
	{  4096000, 24576000, 0x1090 },
	{ 12300000, 24576000, 0x06a0 },
	{ 12000000, 22579200, 0x7e2f },//44.1*512
	{ 12000000, 11289600, 0x7d7d },//44.1*256
	
};

static const struct rt5625_pll_div pll1_div_from_bclk[] = {
	{  1536000, 24576000, 0x3ea0 },
	{  3072000, 24576000, 0x1ea0 },
};

static const struct rt5625_pll_div pll1_div_from_vbclk[] = {
	{  1536000, 24576000, 0x3ea0 },
	{  3072000, 24576000, 0x1ea0 },
};


struct rt5625_coeff_div_stereo {
	unsigned int mclk;
	unsigned int rate;
	unsigned int dac_clk_ctrl1;
	unsigned int dac_clk_ctrl2;
};

struct rt5625_coeff_div_voice {
	unsigned int mclk;
	unsigned int rate;
	unsigned int dac_pcmclk_ctrl1;
};

/*
 * if codec is choose to be slave mode,
 * input bclk should be 32*fs
 */
static const struct rt5625_coeff_div_stereo coeff_div_stereo[] = {
	{ 24576000, 48000, 0x3174, 0x1010 },/*512Fs*/
	{ 12288000, 48000, 0x1174, 0x3030 }, /*256Fs*/
	{ 18432000, 48000, 0x2174, 0x1111 },
	{ 36864000, 48000, 0x2274, 0x2020 },
	{ 49152000, 48000, 0xf074, 0x3030 },

	//BCLK 1.4114
	{ 22579200, 44100, 0x3174, 0x1010 },/*512Fs*/
	
	//BCLK 2.8224
	{ 22579200, 44100, 0x3075, 0x1010 },/*512Fs*/
	
	
	{ 22579200, 44100, 0x3174, 0x1010 },/*512Fs*/
	{ 11289600, 44100, 0x1174, 0x3030 }, /*256Fs*/
	{        0,     0,      0,      0 },
};

static const struct rt5625_coeff_div_voice coeff_div_voice[] = {
	{ 24576000, 16000, 0x2622 },
	{ 24576000,  8000, 0x2824 },
	{ 12288000, 16000, 0x5310 },
	{ 12288000, 8000,  0x5313 },
	
	{        0,     0,      0 },
};

static int rt5625_get_coeff(unsigned int mclk, unsigned int rate, int mode)
{
	int i;

	pr_debug("%s: mclk = %d, rate = %d\n", __func__,
		 mclk, rate);

	if (!mode) {
		for (i = 0; i < ARRAY_SIZE(coeff_div_stereo); i++) {
			if ((coeff_div_stereo[i].rate == rate) &&
			    (coeff_div_stereo[i].mclk == mclk))
				return i;
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(coeff_div_voice); i++) {
			if ((coeff_div_voice[i].rate == rate) &&
			    (coeff_div_voice[i].mclk == mclk))
				return i;
		}
	}

	printk(KERN_ERR "can't find a matched mclk and rate in %s\n",
		(mode ? "coeff_div_voice[]" : "coeff_div_stereo[]"));

	return -EINVAL;
}


static int rt5625_set_dai_pll(struct snd_soc_dai *codec_dai,
			      int pll_id, int source,
			      unsigned int freq_in,
			      unsigned int freq_out)
{
	int i, found = 0;
	struct snd_soc_codec *codec = codec_dai->codec;

	pr_debug("enter %s\n", __func__);

	pr_debug("%s: pll_id = %d, freq_in = %d, freq_out = %d\n",
	       __func__, pll_id, freq_in, freq_out);

	if (pll_id < RT5625_PLL1_FROM_MCLK ||
	    pll_id > RT5625_PLL1_FROM_VBCLK)
		return -EINVAL;

	if (!freq_in || !freq_out)
		return -EINVAL;

	switch (pll_id) {

	case RT5625_PLL1_FROM_MCLK:

		for (i = 0; i < ARRAY_SIZE(pll1_div_from_mclk); i++) {
			if ((freq_in  == pll1_div_from_mclk[i].pll_in) &&
			    (freq_out == pll1_div_from_mclk[i].pll_out)) {
				found = 1;
				break;
			}
		}

		if (found) {
			/* pll source from mclk */
			snd_soc_update_bits(codec, RT5625_GEN_CTRL_REG2,
			          GP2_PLL1_SOUR_SEL_MASK,GP2_PLL1_SOUR_SEL_MCLK);
			/* set pll code */
			rt5625_write(codec, RT5625_PLL_CTRL,
				     pll1_div_from_mclk[i].pll_ctrl);
			/*enable pll1 power*/
			snd_soc_update_bits(codec, RT5625_PWR_MANAG_ADD2,
					  PWR_PLL1, PWR_PLL1);
		} else
			goto not_found;

		break;

	case RT5625_PLL1_FROM_BCLK:

		for (i = 0; i < ARRAY_SIZE(pll1_div_from_bclk); i++) {
			if ((freq_in  == pll1_div_from_bclk[i].pll_in) &&
			    (freq_out == pll1_div_from_bclk[i].pll_out)) {
				found = 1;
				break;
			}
		}

		if (found) {
			/* pll source from bclk */
			snd_soc_update_bits(codec, RT5625_GEN_CTRL_REG2,
					  GP2_PLL1_SOUR_SEL_MASK,GP2_PLL1_SOUR_SEL_BCLK);
			/* set pll1 code */
			rt5625_write(codec, RT5625_PLL_CTRL,
				     pll1_div_from_bclk[i].pll_ctrl);
			/* enable pll1 power */
			snd_soc_update_bits(codec, RT5625_PWR_MANAG_ADD2,
					  PWR_PLL1, PWR_PLL1);
		} else
			goto not_found;

		break;

	case RT5625_PLL1_FROM_VBCLK:

		for (i = 0; i < ARRAY_SIZE(pll1_div_from_vbclk); i++) {
			if ((freq_in  == pll1_div_from_vbclk[i].pll_in) &&
			    (freq_out == pll1_div_from_vbclk[i].pll_out)) {
				found = 1;
				break;
			}
		}

		if (found) {
			/* pll source from bclk */
			snd_soc_update_bits(codec, RT5625_GEN_CTRL_REG2,
					   GP2_PLL1_SOUR_SEL_MASK,
					   GP2_PLL1_SOUR_SEL_VBCLK);
			/* set pll1 code */
			rt5625_write(codec, RT5625_PLL_CTRL,
				     pll1_div_from_vbclk[i].pll_ctrl);
			/*enable pll1 power*/
			snd_soc_update_bits(codec, RT5625_PWR_MANAG_ADD2,
					  PWR_PLL1, PWR_PLL1);
		} else
			goto not_found;

		break;

	default:
		goto not_found;
	}

	snd_soc_update_bits(codec, RT5625_GEN_CTRL_REG1,
			  GP_CLK_FROM_PLL,
			  GP_CLK_FROM_PLL);
	return 0;

not_found:
	printk(KERN_ERR "%s: not found the correct setting for pll1\n",
	       __func__);
	return -EINVAL;
}

static int rt5625_hifi_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				      int clk_id, unsigned int freq,
				      int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct rt5625_priv *rt5625 = snd_soc_codec_get_drvdata(codec);

	if ((freq < (256 * 8000)) || (freq > (512 * 48000))) {
		printk(KERN_ERR "unsupported sysclk freq %u for audio i2s\n",
		       freq);
		return -EINVAL;
	}

	rt5625->stereo_sysclk = freq;
	return 0;
}

static int rt5625_voice_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				       int clk_id, unsigned int freq,
				       int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct rt5625_priv *rt5625 = snd_soc_codec_get_drvdata(codec);

	if ((freq < (256 * 8000)) || (freq > (512 * 48000))) {
		printk(KERN_ERR "unsupported sysclk freq %u for voice pcm\n",
		       freq);
		return -EINVAL;
	}

	rt5625->voice_sysclk = freq;
	return 0;
}

static int rt5625_hifi_pcm_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params,
			struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;	
	struct rt5625_priv *rt5625 = snd_soc_codec_get_drvdata(codec);
	unsigned int iface = 0;
	int rate, coeff;

	pr_debug("enter %s rate=%d format=%d\n", __func__,params_rate(params),params_format(params));

	rate  = params_rate(params);
	coeff = rt5625_get_coeff(rt5625->stereo_sysclk, rate, 0);


	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		iface = MAIN_I2S_DL_16;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		iface = MAIN_I2S_DL_20;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		iface = MAIN_I2S_DL_24;
		break;
	case SNDRV_PCM_FORMAT_S8:
		iface = MAIN_I2S_DL_32;
		break;
	}

	snd_soc_update_bits(codec, RT5625_MAIN_SDP_CTRL,
			  MAIN_I2S_DL_MASK,
			  iface);
	/* power i2s and dac ref*/
	snd_soc_update_bits(codec, RT5625_PWR_MANAG_ADD1,
			  PWR_I2S_INTERFACE | PWR_DAC_REF,
			  PWR_I2S_INTERFACE | PWR_DAC_REF);
	if (coeff >= 0) {
		rt5625_write(codec, RT5625_STEREO_DAC_CLK_CTRL1,
			     coeff_div_stereo[coeff].dac_clk_ctrl1);
		rt5625_write(codec, RT5625_STEREO_DAC_CLK_CTRL2,
			     coeff_div_stereo[coeff].dac_clk_ctrl2);
	}

	return 0;
}

static int rt5625_voice_pcm_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params,
			struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct rt5625_priv *rt5625 = snd_soc_codec_get_drvdata(codec);
	struct snd_soc_dapm_widget *w;
	unsigned int iface = 0;
	int rate, coeff;

	pr_debug(KERN_DEBUG "enter %s\n", __func__);

	rate  = params_rate(params);
	coeff = rt5625_get_coeff(rt5625->voice_sysclk, rate, 1);

	list_for_each_entry(w, &codec->card->widgets, list) {
		if (!w->sname)
			continue;
		if (!strcmp(w->name, "Right ADC"))
			strcpy(w->sname, "Right ADC Voice Capture");
	}

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		iface = EXT_I2S_DL_16;
		break;

	case SNDRV_PCM_FORMAT_S20_3LE:
		iface = EXT_I2S_DL_20;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		iface = EXT_I2S_DL_24;
		break;
	case SNDRV_PCM_FORMAT_S8:
		iface = EXT_I2S_DL_32;
		break;
	}

	/* power i2s and dac ref */
	snd_soc_update_bits(codec, RT5625_PWR_MANAG_ADD1,
			  PWR_I2S_INTERFACE | PWR_DAC_REF,
			  PWR_I2S_INTERFACE | PWR_DAC_REF);
	snd_soc_update_bits(codec, RT5625_EXTEND_SDP_CTRL,
			  EXT_I2S_DL_MASK,
			  iface);
	if (coeff >= 0)
		rt5625_write(codec, RT5625_VOICE_DAC_PCMCLK_CTRL1,
			     coeff_div_voice[coeff].dac_pcmclk_ctrl1);
	else
		return coeff;

	return 0;
}


static int rt5625_hifi_set_dai_fmt(struct snd_soc_dai *codec_dai,
				   unsigned int fmt)
{

	struct snd_soc_codec *codec = codec_dai->codec;
	u16 iface = 0;

	pr_debug("enter %s\n", __func__);

	/* set master/slave interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		iface &= ~MAIN_I2S_MODE_SEL;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		iface |= MAIN_I2S_MODE_SEL;
		break;
	default:
		printk(KERN_ERR "%s: error master/slave interface",
			__func__);
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface &= ~MAIN_I2S_DF_MASK;
		iface |= MAIN_I2S_DF_I2S;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface &= ~MAIN_I2S_DF_MASK;
		iface |= MAIN_I2S_DF_LEFT;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface &= ~MAIN_I2S_DF_MASK;
		iface |= MAIN_I2S_DF_PCM_A;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		iface &= ~MAIN_I2S_DF_MASK;
		iface |= MAIN_I2S_DF_PCM_B;
		break;
	default:
		printk(KERN_ERR "%s: error format interface",
			__func__);
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		iface &= ~MAIN_I2S_DL_MASK;
		iface |= MAIN_I2S_DL_16;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface &= ~MAIN_I2S_DL_MASK;
		iface |= MAIN_I2S_DL_24;
		break;
	default:
		printk(KERN_ERR "%s: error clock inversion interface",
			__func__);
		return -EINVAL;
	}

	rt5625_write(codec, RT5625_MAIN_SDP_CTRL, iface);
	return 0;
}

static int rt5625_voice_set_dai_fmt(struct snd_soc_dai *codec_dai,
				    unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	int iface = 0;

	pr_debug("enter %s\n", __func__);

	/* set slave/master mode */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		iface &= ~EXT_I2S_MODE_SEL;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		iface |= EXT_I2S_MODE_SEL;
		break;
	default:
		printk(KERN_ERR "%s: error slave/master interface",
			__func__);
		return -EINVAL;
	}

	switch(fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface &= ~EXT_I2S_DF_MASK;
		iface |= EXT_I2S_DF_I2S;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface &= ~EXT_I2S_DF_MASK;
		iface |= EXT_I2S_DF_LEFT;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface &= ~EXT_I2S_DF_MASK;
		iface |= EXT_I2S_DF_PCM_A;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		iface &= ~EXT_I2S_DF_MASK;
		iface |= EXT_I2S_DF_PCM_B;
		break;
	default:
		printk(KERN_ERR "%s: error format interface",
			__func__);
		return -EINVAL;
	}

	/*clock inversion*/
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		iface &= ~EXT_I2S_DL_MASK;
		iface |= EXT_I2S_DL_16;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface &= ~EXT_I2S_DL_MASK;
		iface |= EXT_I2S_DL_24;
		break;
	default:
		printk(KERN_ERR "%s: error clock inversion interface",
			__func__);
		return -EINVAL;
	}

	/* enable vopcm */
	iface |= EXT_I2S_FUNC_ENABLE;
	rt5625_write(codec, RT5625_EXTEND_SDP_CTRL, iface);

	return 0;
}


static int rt5625_hifi_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;

	if (mute)
		snd_soc_update_bits(codec, RT5625_STEREO_DAC_VOL,
				  M_MAIN_L_INPUT | M_MAIN_R_INPUT,
				  0);
	else
		snd_soc_update_bits(codec, RT5625_STEREO_DAC_VOL,
				  M_MAIN_L_INPUT | M_MAIN_R_INPUT,
				  0);

	return 0;
}

static int rt5625_voice_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;

	if (mute)
		snd_soc_update_bits(codec, RT5625_VOICE_DAC_OUT_VOL,
				  M_V_DAC, M_V_DAC);
	else
		snd_soc_update_bits(codec, RT5625_VOICE_DAC_OUT_VOL,
				  M_V_DAC,0);

	return 0;
}


static int rt5625_set_bias_level(struct snd_soc_codec *codec,
			enum snd_soc_bias_level level)
{
	pr_debug("%s level=%s\n",__func__,
		(SND_SOC_BIAS_ON==level)?"on":
		(SND_SOC_BIAS_PREPARE==level)?"prepare":
		(SND_SOC_BIAS_STANDBY==level)?"standby":
		(SND_SOC_BIAS_OFF==level)?"off":"unknown"	);
	/*
	switch(level) {
	case SND_SOC_BIAS_ON:
		{
			snd_soc_update_bits(codec,RT5625_PWR_MANAG_ADD1,
			PWR_MIC_BIAS1|PWR_MIC_BIAS2,
			PWR_MIC_BIAS1|PWR_MIC_BIAS2);			
		}
		break;
	case SND_SOC_BIAS_OFF:
		snd_soc_update_bits(codec,RT5625_PWR_MANAG_ADD1,
			0,
			PWR_MIC_BIAS2|PWR_MIC_BIAS1);
		break;
	case SND_SOC_BIAS_STANDBY:
	case SND_SOC_BIAS_PREPARE:
	default:
		break;
	}
	*/
	return 0;
}


#define RT5625_STEREO_RATES (SNDRV_PCM_RATE_8000_48000)
#define RT5626_VOICE_RATES (SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_8000)

#define RT5625_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE |\
			SNDRV_PCM_FMTBIT_S8)

static struct snd_soc_dai_ops rt5625_dai_ops_hifi = {
	.hw_params	= rt5625_hifi_pcm_hw_params,
	.digital_mute	= rt5625_hifi_mute,
	.set_fmt	= rt5625_hifi_set_dai_fmt,
	.set_sysclk	= rt5625_hifi_set_dai_sysclk,
	.set_pll	= rt5625_set_dai_pll,
};

static struct snd_soc_dai_ops rt5625_dai_ops_voice = {
	.hw_params	= rt5625_voice_pcm_hw_params,
	.digital_mute	= rt5625_voice_mute,
	.set_fmt	= rt5625_voice_set_dai_fmt,
	.set_sysclk	= rt5625_voice_set_dai_sysclk,
	.set_pll	= rt5625_set_dai_pll,
};

static struct snd_soc_dai_driver rt5625_dai[] = {
	/* hifi codec dai */
	{
		.name = "rt5625_hifi",
		.id = 0,
		.playback = {
			.stream_name  = "HiFi Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates 	      = RT5625_STEREO_RATES,
			.formats      = RT5625_FORMATS,
		},
		.capture = {
			.stream_name  = "HiFi Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates        = RT5625_STEREO_RATES,
			.formats      = RT5625_FORMATS,
		},
		.ops = &rt5625_dai_ops_hifi,
	},

	/* voice codec dai */
	{
		.name = "rt5625_voice",
		.id = 1,
		.playback = {
			.stream_name  = "Voice Playback",
			.channels_min = 1,
			.channels_max = 1,
			.rates        = RT5626_VOICE_RATES,
			.formats      = RT5625_FORMATS,
		},
		.capture = {
			.stream_name  = "Voice Capture",
			.channels_min = 1,
			.channels_max = 1,
			.rates        = RT5626_VOICE_RATES,
			.formats      = RT5625_FORMATS,
		},
		.ops = &rt5625_dai_ops_voice,
	},
};


static int rt5625_reg_init(struct snd_soc_codec *codec)
{
#define RT5625_INIT_REG_NUM ARRAY_SIZE(rt5625_init_list)
	struct rt5625_init_reg {
		char name[30];
		u16  reg_value;
		u8	 reg_index;
	}rt5625_init_list[] = {
		{"HP Output Volume",	 0x8080, RT5625_HP_OUT_VOL},
		{"SPK Output Volume",	 0x8080, RT5625_SPK_OUT_VOL},
		{"Aux out volume",	 0x0e0e, RT5625_AUX_OUT_VOL},
		{"Phone Diff Input Ctrl",	 0x2000, RT5625_PHONEIN_VOL},
		{"DAC Mic Route",	 0xee0f, RT5625_DAC_AND_MIC_CTRL},
		{"Output Mixer Control", 0x2008, RT5625_OUTPUT_MIXER_CTRL},
		{"Mic Control", 	 0x0000, RT5625_MIC_CTRL},//mic1 boost 20dB,mic2 bias 20dB
		{"Voice DAC Volume",	 0x6035, RT5625_VOICE_DAC_OUT_VOL},
		{"ADC Rec Mixer",	 0x3f3f, RT5625_ADC_REC_MIXER},
		{"General Control",  0x0c0a, RT5625_GEN_CTRL_REG1},//main system clock souce from PLL1
		{"Gain 15db ADC",	 0xcbcb, RT5625_ADC_REC_GAIN},
		{"PCM Output Volume",	 0x1010, RT5625_STEREO_DAC_VOL},//0x5050 is 0dB
		{"Mic Input Volume",	 0x0808, RT5625_MIC_VOL},	
		{"PM1", 				 0x0f43, RT5625_PWR_MANAG_ADD1},
		{"PM2", 				 0xa009, RT5625_PWR_MANAG_ADD2},
		{"PM3", 				 0xf000, RT5625_PWR_MANAG_ADD3},
	};
	

	int i;

	for (i = 0; i < RT5625_INIT_REG_NUM; i++)
	{
		snd_soc_write(codec,
			     rt5625_init_list[i].reg_index,
			     rt5625_init_list[i].reg_value);
	}


	return 0;
}

static int init_voicepcm_clock(struct snd_soc_codec *codec,int frame_rate,int frame_width)
{	
	struct rt5625_priv *rt5625 = snd_soc_codec_get_drvdata(codec);
	int coeff;
	unsigned int iface;

	rt5625->voice_sysclk = 24576000;

	coeff= rt5625_get_coeff(rt5625->voice_sysclk, frame_rate, 1);

	//format is S16_LE
	iface=EXT_I2S_DL_16;	
	//master
	iface &= ~EXT_I2S_MODE_SEL;
	//mode A
	iface &= ~EXT_I2S_DF_MASK;
	iface |= EXT_I2S_DF_PCM_A;

	//voice PCM enable
	iface |= EXT_I2S_FUNC_ENABLE;

	//use invert bit clock
	iface |= EXT_I2S_BCLK_POLARITY;

	snd_soc_update_bits(codec, RT5625_EXTEND_SDP_CTRL,
			  EXT_I2S_DL_MASK,iface);
	if (coeff >= 0)
		snd_soc_write(codec, RT5625_VOICE_DAC_PCMCLK_CTRL1,
				 coeff_div_voice[coeff].dac_pcmclk_ctrl1);
	else
	{
		printk(KERN_ERR"failed to found matched voice coef\n");
	}

	return 0;

}



static int rt5625_probe(struct snd_soc_codec *codec)
{
	struct rt5625_priv *rt5625 = (struct rt5625_priv *)snd_soc_codec_get_drvdata(codec);
	int i=0;
	unsigned int val;

	rt5625->codec = codec;

	codec->control_type = SND_SOC_I2C;

	codec->control_data = rt5625->control_data;
	
	snd_soc_write(codec,RT5625_RESET,0);
	do{
		val=snd_soc_read(codec,RT5625_RESET);
		if(0x59b4==val)
			break;
		msleep(5);	
	}while(i++<100);

	snd_soc_write(codec, RT5625_PD_CTRL_STAT, 0);
	snd_soc_write(codec, RT5625_PWR_MANAG_ADD1, PWR_MAIN_BIAS);
	snd_soc_write(codec, RT5625_PWR_MANAG_ADD2, PWR_MIXER_VREF);
	rt5625_reg_init(codec);

	

	//init voice part (voice codec is controlled by ourselves)
	init_voicepcm_clock(codec,8000,64);

	rt5625_set_bias_level(codec, SND_SOC_BIAS_OFF);
	//codec->pop_time = 1;

	//codec->bias_level = SND_SOC_BIAS_STANDBY;
	//schedule_delayed_work(&codec->delayed_work, msecs_to_jiffies(2000));

	//init depop
	//hp_depop_mode2(codec);

	
	for (i = 0; i < ARRAY_SIZE(rt5625->supplies); i++)
		rt5625->supplies[i].supply = rt5625_supply_names[i];
	#warning "FIXME: rt5625 regulator get and enable"
	#if 0
	ret = regulator_bulk_get(codec->dev, ARRAY_SIZE(rt5625->supplies),
				 rt5625->supplies);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to request supplies: %d\n", ret);
		goto err;
	}

	
	ret = regulator_bulk_enable(ARRAY_SIZE(rt5625->supplies),
				    rt5625->supplies);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to enable supplies: %d\n", ret);
		goto err_get;
	}
	#endif

	return 0;
	
//err_get:
	//regulator_bulk_free(ARRAY_SIZE(rt5625->supplies), rt5625->supplies);
//err:
//	return ret;


}



static int rt5625_remove(struct snd_soc_codec *codec)
{
	rt5625_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}


static int rt5625_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
	rt5625_direct_write(codec, RT5625_PWR_MANAG_ADD1, 0);
	rt5625_direct_write(codec, RT5625_PWR_MANAG_ADD2, 0);
	rt5625_direct_write(codec, RT5625_PWR_MANAG_ADD3, 0);
	rt5625_direct_write(codec, RT5625_VODSP_CTL, 0);
	rt5625_direct_write(codec, RT5625_PD_CTRL_STAT, 0xffff);	
	rt5625_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int rt5625_resume(struct snd_soc_codec *codec)
{
	int i;
	u16 *cache = codec->reg_cache;

	/* Sync reg_cache with the hardware
	 * Could use auto incremented writes to speed this up
	 */
	for (i = 0; i < ARRAY_SIZE(rt5625_reg); i++) {
		if (i == RT5625_RESET)
			continue;
		rt5625_direct_write(codec,i<<1,cache[i]);
	}

	rt5625_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	/* charge rt5625 caps */
	if (codec->dapm.suspend_bias_level == SND_SOC_BIAS_ON) {
		rt5625_set_bias_level(codec, SND_SOC_BIAS_PREPARE);
	}
	
	return 0;
}


static struct snd_soc_codec_driver soc_codec_dev_rt5625 = {
	.probe = rt5625_probe,
	.remove = rt5625_remove,
	.suspend = rt5625_suspend,
	.resume = rt5625_resume,
	.set_bias_level = rt5625_set_bias_level,	
	.controls = rt5625_snd_controls,
	.num_controls = ARRAY_SIZE(rt5625_snd_controls),
	.dapm_widgets = rt5625_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(rt5625_dapm_widgets),
	.dapm_routes = rt5625_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(rt5625_dapm_routes),
	.reg_cache_size = sizeof(rt5625_reg),
	.reg_word_size = sizeof(u16),
	.reg_cache_default = rt5625_reg,
	.reg_cache_step = 2,	
	.read = rt5625_read,
	.write = rt5625_write,
	
};
static int rt5625_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct rt5625_priv *rt5625;
	int ret = 0;

	rt5625 = kzalloc(sizeof(struct rt5625_priv), GFP_KERNEL);
	if (rt5625 == NULL)	return -ENOMEM;

	rt5625->control_type = SND_SOC_I2C;
	rt5625->control_data = i2c;
	i2c_set_clientdata(i2c, rt5625);

	ret = snd_soc_register_codec(&i2c->dev, &soc_codec_dev_rt5625,
			rt5625_dai, ARRAY_SIZE(rt5625_dai));
	if (ret < 0)
		kfree(rt5625);

	return ret;
}

static int rt5625_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);	
	kfree(i2c_get_clientdata(client));
	return 0;
	
}

static const struct i2c_device_id rt5625_i2c_id[] = {
	{"rt5625", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, rt5625_i2c_id);

static struct i2c_driver rt5625_i2c_driver = {
	.driver = {
		.name  = "rt5625",
		.owner = THIS_MODULE,
	},
	.probe    = rt5625_i2c_probe,
	.remove   = rt5625_i2c_remove,
	.id_table = rt5625_i2c_id,
};


static int __init rt5625_module_init(void)
{
	int ret = i2c_add_driver(&rt5625_i2c_driver);
	if (ret)
		printk(KERN_ERR "Failed to register rt5625 I2C driver: %d\n",
		       ret);
	return ret;
}

static void __exit rt5625_module_exit(void)
{
	i2c_del_driver(&rt5625_i2c_driver);
}

module_init(rt5625_module_init);
module_exit(rt5625_module_exit);

MODULE_DESCRIPTION("ASoC realtek 5625 codec driver");
MODULE_AUTHOR("Raymond Wang<raymond1860@gmail.com>");
MODULE_LICENSE("GPL");

