#define DEBUG
#include <linux/types.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/spinlock.h>
#include <linux/mxcfb.h>
#include <linux/backlight.h>
#include <video/mipi_display.h>

#include <mach/hardware.h>
#include <mach/clock.h>
#include <mach/mipi_dsi.h>

#include "mipi_dsi.h"

#define MIPI_DSI_MAX_RET_PACK_SIZE				(0x4)

#define CHECK_RETCODE(ret)					\
do {								\
	if (ret < 0) {						\
		dev_err(&mipi_dsi->pdev->dev,			\
			"%s ERR: ret:%d, line:%d.\n",		\
			__func__, ret, __LINE__);		\
		return ret;					\
	}							\
} while (0)

static struct fb_videomode sii450_lcd_modedb[] = {
	{
	 "540x960@60", 
	60,/*refresh*/
	540,960,/*xres,yres*/
	30500,/*pixclock ps*/
	10, 10, /*left margin,right margin*/
	50, 30,/*upper margin,lower margin*/
	10,10,/*hsync len,vsync len*/
	FB_SYNC_OE_LOW_ACT,/*sync*/
	FB_VMODE_NONINTERLACED,/*vmode*/
	0,/*flag*/
	},
};

static struct mipi_lcd_config lcd_config = {
	.virtual_ch		= 0x0,
	.data_lane_num  = 0x2,
	.max_phy_clk    = 500,
	.dpi_fmt		= MIPI_RGB888,
};
void mipid_sii450_get_lcd_videomode(struct fb_videomode **mode, int *size,
		struct mipi_lcd_config **data)
{
	*mode = &sii450_lcd_modedb[0];
	*size = ARRAY_SIZE(sii450_lcd_modedb);
	*data = &lcd_config;
}

static inline int mipi_generic_write(struct mipi_dsi_info *mipi_dsi,u32* buf,int l){	
	int err;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
					buf, l);
	CHECK_RETCODE(err);
	return err;
}
#define W_COM_1A_0P(o) \
	do{ \
		u32 buf[DSI_CMD_BUF_MAXSIZE];\
		buf[0]=o;\
		mipi_generic_write(mipi_dsi,buf,1);\
	}while(0)

#define W_COM_1A_1P(o,p) \
	do{ \
		u32 buf[DSI_CMD_BUF_MAXSIZE];\
		buf[0]=o|(p<<8);\
		mipi_generic_write(mipi_dsi,buf,2);\
	}while(0)
#define W_COM_1A_2P(o,p1,p2) \
	do{ \
		u32 buf[DSI_CMD_BUF_MAXSIZE];\
		buf[0]=o|(p1<<8)|(p2<<16);\
		mipi_generic_write(mipi_dsi,buf,3);\
	}while(0)
#define W_COM_1A_3P(o,p1,p2,p3) \
		do{ \
			u32 buf[DSI_CMD_BUF_MAXSIZE];\
			buf[0]=o|(p1<<8)|(p2<<16)|(p3<<24);\
			mipi_generic_write(mipi_dsi,buf,4);\
		}while(0)
#define W_COM_1A_4P(o,p1,p2,p3,p4) \
	do{ \
		u32 buf[DSI_CMD_BUF_MAXSIZE];\
		buf[0]=o|(p1<<8)|(p2<<16)|(p3<<24);\
		buf[1]=p4;\
		mipi_generic_write(mipi_dsi,buf,5);\
	}while(0)

#define W_COM_1A_10P(o,p1,p2,p3,p4,p5,p6,p7,p8,p9,p10) \
	do{ \
		u32 buf[DSI_CMD_BUF_MAXSIZE];\
		buf[0]=o|(p1<<8)|(p2<<16)|(p3<<24);\
		buf[1]=p4|(p5<<8)|(p6<<16)|(p7<<24);\
		buf[2]=p8|(p9<<8)|(p10<<16);\
		mipi_generic_write(mipi_dsi,buf,11);\
	}while(0)
#define W_COM_1A_12P(o,p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12) \
	do{ \
		u32 buf[DSI_CMD_BUF_MAXSIZE];\
		buf[0]=o|(p1<<8)|(p2<<16)|(p3<<24);\
		buf[1]=p4|(p5<<8)|(p6<<16)|(p7<<24);\
		buf[2]=p8|(p9<<8)|(p10<<16)|(p11<<24);\
		buf[3]=p12;\
		mipi_generic_write(mipi_dsi,buf,13);\
	}while(0)
#define W_COM_1A_14P(o,p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12,p13,p14) \
	do{ \
		u32 buf[DSI_CMD_BUF_MAXSIZE];\
		buf[0]=o|(p1<<8)|(p2<<16)|(p3<<24);\
		buf[1]=p4|(p5<<8)|(p6<<16)|(p7<<24);\
		buf[2]=p8|(p9<<8)|(p10<<16)|(p11<<24);\
		buf[3]=p12|(p13<<8)|(p14<<16);\
		mipi_generic_write(mipi_dsi,buf,15);\
	}while(0)

#define W_COM_1A_15P(o,p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12,p13,p14,p15) \
	do{ \
		u32 buf[DSI_CMD_BUF_MAXSIZE];\
		buf[0]=o|(p1<<8)|(p2<<16)|(p3<<24);\
		buf[1]=p4|(p5<<8)|(p6<<16)|(p7<<24);\
		buf[2]=p8|(p9<<8)|(p10<<16)|(p11<<24);\
		buf[3]=p12|(p13<<8)|(p14<<16)|(p15<<24);\
		mipi_generic_write(mipi_dsi,buf,16);\
	}while(0)
#define W_COM_1A_16P(o,p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12,p13,p14,p15,p16) \
	do{ \
		u32 buf[DSI_CMD_BUF_MAXSIZE];\
		buf[0]=o|(p1<<8)|(p2<<16)|(p3<<24);\
		buf[1]=p4|(p5<<8)|(p6<<16)|(p7<<24);\
		buf[2]=p8|(p9<<8)|(p10<<16)|(p11<<24);\
		buf[3]=p12|(p13<<8)|(p14<<16)|(p15<<24);\
		buf[4]=p16;\
		mipi_generic_write(mipi_dsi,buf,17);\
	}while(0)

static int mipi_write_array(struct mipi_dsi_info *mipi_dsi,unsigned char* s,int l){
		u32 buf[DSI_CMD_BUF_MAXSIZE];
		int i=0;		
		int err;
		int bytes=l;
		//memset(buf,0,sizeof(u32)*DSI_CMD_BUF_MAXSIZE);
		do{
			if(l>=4){
			    buf[i++]=s[0]+(s[1]<<8)+(s[2]<<16)+(s[3]<<24);
				l-=4;s+=4;
			}else{
				if(3==l){
					buf[i]=s[0]+(s[1]<<8)+(s[2]<<16);
					l-=3;
				}else if(2==l){
					buf[i]=s[0]+(s[1]<<8);
					l-=2;
				}else if(1==l){
					buf[i]=s[0];
					l-=1;
				}

				break;
			}
		}while(1);
		
		err = mipi_dsi_pkt_write(mipi_dsi,MIPI_DSI_DCS_LONG_WRITE,
						buf, bytes);
		CHECK_RETCODE(err);
		return err;
}


	
#define Delayms(n) \
	mdelay(n)
static int mipid_sii450_setup(struct mipi_dsi_info *mipi_dsi)
{
	dev_dbg(&mipi_dsi->pdev->dev, "SII450 LCD setup.\n");
 //W_COM_1A_3P(0xb9,0xff,0x83,0x89);
 {
	 unsigned char dcs0[]={
		 0xb9,0xff,0x83,0x89,
	 };
	 mipi_write_array(mipi_dsi,dcs0,sizeof(dcs0)/sizeof(dcs0[0]));	 
 }
 Delayms(1);

 //W_COM_1A_2P(0xba,0x41,0x93);
 {
	 unsigned char dcs0[]={
		 0xba,0x41,0x93
	 };
	 mipi_write_array(mipi_dsi,dcs0,sizeof(dcs0)/sizeof(dcs0[0]));	 
 }
 //W_COM_1A_1P(0xc6,0x08); 
 {
	 unsigned char dcs0[]={
		 0xc6,0x08,
	 };
	 mipi_write_array(mipi_dsi,dcs0,sizeof(dcs0)/sizeof(dcs0[0]));	 
 }
 Delayms(1);	

 //W_COM_1A_2P(0xde,0x05,0x58); 
 {
	 unsigned char dcs0[]={
		 0xde,0x05,0x58,
	 };
	 mipi_write_array(mipi_dsi,dcs0,sizeof(dcs0)/sizeof(dcs0[0]));	 
 }
 Delayms(1);			


 //W_COM_1A_19P(0xb1,0x00,0x00,0x04,0xe3,0x9a,0x10,0x11,0x8f,0xef,0x28,0x30,0x23,0x23,0x42,0x00,0x3a,0xf5,0x20,0x00);
 {
	unsigned char dcs0[]={
		0xb1,0x00,0x00,0x04,0xe3,0x9a,0x10,0x11,0x8f,0xef,0x28,0x30,0x23,0x23,0x42,0x00,
		0x3a,0xf5,0x20,0x00,
	};
	mipi_write_array(mipi_dsi,dcs0,sizeof(dcs0)/sizeof(dcs0[0]));	
 }	

  //W_COM_1A_23P(0xb4,0x80,0x08,0x00,0x32,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x37,0x0a,0x40,
 			 // 0x04,0x37,0x0a,0x40,0x14,0xff,0xff,0x0a);
  {
	 unsigned char dcs0[]={
		 0xb4,0x80,0x08,0x00,0x32,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x37,0x0a,0x40,
 			 0x04,0x37,0x0a,0x40,0x14,0xff,0xff,0x0a,
	 };
	 mipi_write_array(mipi_dsi,dcs0,sizeof(dcs0)/sizeof(dcs0[0]));	 
  }


 //W_COM_1A_7P(0xb2,0x00,0x00,0x78,0x08,0x03,0x3f,0x80);
 {
	 unsigned char dcs0[]={
		 0xb2,0x00,0x00,0x78,0x08,0x03,0x3f,0x80,
	 };
	 mipi_write_array(mipi_dsi,dcs0,sizeof(dcs0)/sizeof(dcs0[0]));	 
  }

{
	unsigned char dcs0[]={
		0xc1,0x01,0x02,0x08,0x17,0x20,0x27,0x2d,0x32,0x38,0x40,0x48,0x4f,0x57,0x5f,0x67,
		0x70,0x79,0x81,0x89,0x91,0x99,0xa1,0xa8,0xb0,0xb9,0xc2,0xc9,0xd0,0xd9,0xe2,0xe9,
	};
	unsigned char dcs1[]={
		0xf3,0xfa,0xff,0xd0,0xaf,0xdf,0xa9,0xa9,0x3d,0x79,0x92,0xc0,0x02,0x08,0x17,0x20,
		0x27,0x2d,0x32,0x38,0x40,0x48,0x4f,0x57,0x5f,0x67,0x70,0x79,0x81,0x89,0x91,0x99,
	};
	unsigned char dcs2[]={
		0xa1,0xa8,0xb0,0xb9,0xc2,0xc9,0xd0,0xd9,0xe2,0xe9,0xf3,0xfa,0xff,0xd0,0xaf,0xdf,
		0xa9,0xa9,0x3d,0x79,0x92,0xc0,0x02,0x08,0x17,0x20,0x27,0x2d,0x32,0x38,0x40,0x48,
	};
	unsigned char dcs3[]={
		0x4f,0x57,0x5f,0x67,0x70,0x79,0x81,0x89,0x91,0x99,0xa1,0xa8,0xb0,0xb9,0xc2,0xc9,
		0xd0,0xd9,0xe2,0xe9,0xf3,0xfa,0xff,0xd0,0xaf,0xdf,0xa9,0xa9,0x3d,0x79,0x92,0xc0,
	};
	mipi_write_array(mipi_dsi,dcs0,sizeof(dcs0)/sizeof(dcs0[0]));
	
	mipi_write_array(mipi_dsi,dcs1,sizeof(dcs1)/sizeof(dcs1[0]));
	
	mipi_write_array(mipi_dsi,dcs2,sizeof(dcs2)/sizeof(dcs2[0]));

	mipi_write_array(mipi_dsi,dcs3,sizeof(dcs3)/sizeof(dcs3[0]));
}

 {
	unsigned char dcs0[]={
		0xd5,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x60,0x00,0x99,0x88,0x99,0x88,0x88,
		0x32,0x88,0x10,0x88,0x76,0x88,0x54,0x10,0x32,0x88,0x88,0x88,0x88,0x88,0x88,0x99,
	};
	unsigned char dcs1[]={
		0x88,0x99,0x88,0x45,0x88,0x67,0x88,0x01,0x88,0x23,0x23,0x01,0x88,0x88,0x88,0x88,
		0x88,
	};

	mipi_write_array(mipi_dsi,dcs0,sizeof(dcs0)/sizeof(dcs0[0]));
	
	mipi_write_array(mipi_dsi,dcs1,sizeof(dcs1)/sizeof(dcs1[0]));
 }

 {
	 unsigned char dcs0[]={
	 	0xe0,0x05,0x14,0x18,0x2d,0x34,0x3f,0x20,0x3c,0x08,0x0e,0x0e,0x11,0x13,0x10,0x12,
		0x1a,0x1c,0x05,0x14,0x18,0x2d,0x34,0x3f,0x20,0x3c,0x08,0x0e,0x0e,0x11,0x13,0x10,
	 };
	 unsigned char dcs1[]={
		 0x12,0x1a,0x1c,
	 };
	 
	 mipi_write_array(mipi_dsi,dcs0,sizeof(dcs0)/sizeof(dcs0[0]));
	 
	 mipi_write_array(mipi_dsi,dcs1,sizeof(dcs1)/sizeof(dcs1[0]));

	 
  }
 //W_COM_1A_2P(0xb6,0x00,0x96);
 {
	 unsigned char dcs0[]={
		 0xb6,0x00,0x96
	 };
	 mipi_write_array(mipi_dsi,dcs0,sizeof(dcs0)/sizeof(dcs0[0]));	 
 }
 
 //W_COM_1A_1P(0xcc,0x02);
 {
	 unsigned char dcs0[]={
		 0xcc,0x02,
	 };
	 mipi_write_array(mipi_dsi,dcs0,sizeof(dcs0)/sizeof(dcs0[0]));	 
 }
 //W_COM_1A_2P(0xcb,0x07,0x07);
 {
	 unsigned char dcs0[]={
		 0xcb,0x07,0x07
	 };
	 mipi_write_array(mipi_dsi,dcs0,sizeof(dcs0)/sizeof(dcs0[0]));	 
 }
 
 //W_COM_1A_4P(0xbb,0x00,0x00,0xff,0x80);
 {
	 unsigned char dcs0[]={
		 0xbb,0x00,0x00,0xff,0x80,
	 };
	 mipi_write_array(mipi_dsi,dcs0,sizeof(dcs0)/sizeof(dcs0[0]));	 
 }
 //W_COM_1A_0P(0x11);
 {
	 unsigned char dcs0[]={
		 0x11,
	 };
	 mipi_write_array(mipi_dsi,dcs0,sizeof(dcs0)/sizeof(dcs0[0]));	 
 }
 Delayms(120);	
 

 //W_COM_1A_0P(0x29);
 {
	 unsigned char dcs0[]={
		 0x29,
	 };
	 mipi_write_array(mipi_dsi,dcs0,sizeof(dcs0)/sizeof(dcs0[0]));	 
 }
 Delayms(10);


	return 0;

}

int mipid_sii450_lcd_suspend(struct mipi_dsi_info *mipi_dsi){	
	int err=0;	
	if(mipi_dsi->backlight_power){
		mipi_dsi->backlight_power(0);
	}
	if(mipi_dsi->lcd_power){
		mipi_dsi->lcd_power(0);
	}else {
		u32 buf[DSI_CMD_BUF_MAXSIZE];
		buf[0] = 0x22 |(0x00 << 8) ;
		err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
			buf, 0x2);
		CHECK_RETCODE(err);
		msleep(50);
		err = mipi_dsi_dcs_cmd(mipi_dsi, MIPI_DCS_SET_DISPLAY_OFF,
					NULL, 0);
		CHECK_RETCODE(err);
		msleep(50);
		err = mipi_dsi_dcs_cmd(mipi_dsi, MIPI_DCS_ENTER_SLEEP_MODE,
						NULL, 0);
		CHECK_RETCODE(err);
	}
	
	return err;
}

int mipid_sii450_lcd_resume(struct mipi_dsi_info *mipi_dsi){	
	int err=0;	
	
	if(mipi_dsi->lcd_power){
		mipi_dsi->lcd_power(1);
		if(mipi_dsi->reset)
			mipi_dsi->reset();
		mipid_sii450_setup(mipi_dsi);
	}else {
		u32 buf[DSI_CMD_BUF_MAXSIZE];
		buf[0] = 0x22 |(0x00 << 8) ;
		err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
			buf, 0x2);
		CHECK_RETCODE(err);
		msleep(120);
		err = mipi_dsi_dcs_cmd(mipi_dsi, MIPI_DCS_EXIT_SLEEP_MODE,
					NULL, 0);
		CHECK_RETCODE(err);

		msleep(120);
		err = mipi_dsi_dcs_cmd(mipi_dsi, MIPI_DCS_SET_DISPLAY_ON,
						NULL, 0);
		CHECK_RETCODE(err);
	}
	
	if(mipi_dsi->backlight_power){
		mipi_dsi->backlight_power(1);
	}
	
	return err;
}


int mipid_sii450_lcd_setup(struct mipi_dsi_info *mipi_dsi)
{
	//FIXME skip setup since bootloader already init
	return 0;

}

