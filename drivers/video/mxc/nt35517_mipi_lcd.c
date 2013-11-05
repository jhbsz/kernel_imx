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

static struct fb_videomode nt35517_lcd_modedb[] = {
	{
	 "NT-QHD", 60, 540, 960, 30500/*ps*/,  //945,30500
	 3, 3,
	 60, 35,//5,20
	 8,20,//18
	 FB_SYNC_OE_LOW_ACT,//ori is  FB_SYNC_OE_LOW_ACT
	 FB_VMODE_NONINTERLACED,//ori is  FB_VMODE_NONINTERLACED
	 0,
	},
};

static struct mipi_lcd_config lcd_config = {
	.virtual_ch		= 0x0,
	.data_lane_num  = 0x2,
	.max_phy_clk    = 550,
	.dpi_fmt		= MIPI_RGB888,
};
void mipid_nt35517_get_lcd_videomode(struct fb_videomode **mode, int *size,
		struct mipi_lcd_config **data)
{
	if (cpu_is_mx6dl())
		nt35517_lcd_modedb[0].pixclock = 37037; /* 27M clock*/
	*mode = &nt35517_lcd_modedb[0];
	*size = ARRAY_SIZE(nt35517_lcd_modedb);
	*data = &lcd_config;
}

static int mipid_nt35517_setup(struct mipi_dsi_info *mipi_dsi)
{
	u32 buf[DSI_CMD_BUF_MAXSIZE];
	int err;
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI LCD setup.\n");
	//*************************************
	// Select CMD2, Page 0
	//*************************************
	//REGW 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00
	buf[0] = 0xF0 | (0x55 << 8) | (0xAA<<16) |(0x52 << 24) ;
	buf[1] = 0x08 | (0x00<<8);
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
					buf, 0x6);
	CHECK_RETCODE(err);

	#if 0
	//read ID
	buf[0] = MIPI_DSI_MAX_RET_PACK_SIZE;
	err = mipi_dsi_pkt_write(mipi_dsi,
				MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE,
				buf, 0);
	CHECK_RETCODE(err);
	buf[0] = 0x4 ;
	err =  mipi_dsi_pkt_read(mipi_dsi,
			MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM,
			buf, 0x4);

	if (!err && ((buf[0] & 0xffff) == 0x8000)) {
		dev_info(&mipi_dsi->pdev->dev,
				"MIPI DSI LCD ID:0x%x.\n", buf[0]);
	} else {
		dev_err(&mipi_dsi->pdev->dev,
			"mipi_dsi_pkt_read err:%d, data:0x%x.\n",
			err, buf[0]);
		dev_info(&mipi_dsi->pdev->dev,
				"MIPI DSI LCD not detected!\n");
		return err;
	}
	#endif

	/* 
	 *Forward Scan		CTB=CRL=0
	 */
	buf[0] = 0xB1 | (0x7C << 8) |(0x00 << 16) ;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
						buf, 0x3);
	CHECK_RETCODE(err);


	/*Source hold time */
	buf[0] = 0xB6 | (0x05 << 8);
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
						buf, 0x2);
	CHECK_RETCODE(err);

	// Gate EQ control
	buf[0] = 0xB7 | (0x72 << 8) | (0x72<<16);
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE, buf,
				0x3);
	CHECK_RETCODE(err);

	// Source EQ control (Mode 2)
	buf[0] = 0xB8 | (0x01 << 8) | (0x06 << 16) | (0x05 << 24) ;
	buf[1] = 0x04;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE, buf,
				0x5);
	CHECK_RETCODE(err);

	// Bias Current
	buf[0] = 0xBB | (0x55 << 8);
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE, buf,
				0x2);
	CHECK_RETCODE(err);

	// Inversion
	buf[0] = 0xBC | (0x00 << 8) | (0x00 << 16) | (0x00 << 24);
	err = mipi_dsi_pkt_write(mipi_dsi,
		MIPI_DSI_GENERIC_LONG_WRITE, buf, 0x4);
	CHECK_RETCODE(err);

	//Frame Rate
	buf[0] = 0xBD | (0x01 << 8)| (0x4E << 16)| (0x10 << 24);
	buf[1] = 0x20 | (0x01<<8);
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE, buf,
				0x6);
	CHECK_RETCODE(err);

#if 0 //have fix in pannel
	/* Set MIPI: DPHYCMD & DSICMD, data lane number */
	buf[0] = HX8369_CMD_SETMIPI | (HX8369_CMD_SETMIPI_PARAM_1 << 8);
	buf[1] = HX8369_CMD_SETMIPI_PARAM_2;
	buf[2] = HX8369_CMD_SETMIPI_PARAM_3;
	if (lcd_config.data_lane_num == HX8369_ONE_DATA_LANE)
		buf[2] |= HX8369_CMD_SETMIPI_ONELANE;
	else
		buf[2] |= HX8369_CMD_SETMIPI_TWOLANE;
	buf[3] = HX8369_CMD_SETMIPI_PARAM_4;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE, buf,
				HX8369_CMD_SETMIPI_LEN);
	CHECK_RETCODE(err);
#endif

#if 1
	/* Set pixel format:24bpp ,add by allenyao*/
	buf[0] = 0x3A;
	switch (lcd_config.dpi_fmt) {
	case MIPI_RGB565_PACKED:
	case MIPI_RGB565_LOOSELY:
	case MIPI_RGB565_CONFIG3:
		buf[0] |= (0x50 << 8);
		break;

	case MIPI_RGB666_LOOSELY:
	case MIPI_RGB666_PACKED:
		buf[0] |= (0x60 << 8);
		break;

	case MIPI_RGB888:
		buf[0] |= (0x70 << 8);
		break;

	default:
		buf[0] |= (0x70 << 8);
		break;
	}
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM,
			buf, 0);
	CHECK_RETCODE(err);
#endif

	// Display Timing: Dual 8-phase 4-overlap
	//REGW 0xC9, 0x61, 0x06, 0x0D, 0x17, 0x17, 0x00
	buf[0] = 0xC9 |(0x61 << 8) |(0x06 << 16) |(0x0D << 24)	;
	buf[1] = 0x17 | (0x17 << 8)| (0x00 << 8);
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
				buf, 0x7);
	CHECK_RETCODE(err);

	//*************************************
	// Select CMD2, Page 1
	//*************************************
	//REGW 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x01
	buf[0] = 0xF0 |(0x55 << 8) |(0xAA << 16)|(0x52 << 24);
	buf[1] = 0x08 |(0x01 <<8);
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
					buf, 0x6);
	CHECK_RETCODE(err);

	// AVDD: 5.5V
	//REGW 0xB0, 0x0A, 0x0A, 0x0A
	buf[0] = 0xB0 | (0x0A << 8) |(0x0A << 16) |(0x0A << 24);
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x4);
	CHECK_RETCODE(err);

	// AVEE: -5.5V
	//REGW 0xB1, 0x0A, 0x0A, 0x0A
	buf[0] = 0xB1 | (0x0A << 8)| (0x0A << 16)| (0x0A << 24);
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x4);
	CHECK_RETCODE(err);
	
	// VCL: -3.5V
	//REGW 0xB2, 0x02, 0x02, 0x02
	buf[0] = 0xB2 |(0x02 << 8) |(0x02 << 16)|(0x02 << 24);
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x4);
	CHECK_RETCODE(err);

	// VGH: 15.0V
	//REGW 0xB3, 0x10, 0x10, 0x10
	buf[0] = 0xB3 |(0x10 << 8) |(0x10 << 16)|(0x10 << 24);
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x4);
	CHECK_RETCODE(err); 

	// VGLX: -10.0V
	//REGW 0xB4, 0x06, 0x06, 0x06
	buf[0] = 0xB4 |(0x06 << 8) |(0x06 << 16)|(0x06 << 24);
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x4);
	CHECK_RETCODE(err); 

	// AVDD: 3xVDDB
	//REGW 0xB6, 0x54, 0x54, 0x54
	buf[0] = 0xB6 |(0x54 << 8) |(0x54 << 16)|(0x54 << 24);
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x4);
	CHECK_RETCODE(err); 

	// AVEE: -2xVDDB
	//REGW 0xB7, 0x24, 0x24, 0x24
	buf[0] = 0xB7 |(0x24 << 8) |(0x24 << 16)|(0x24 << 24);
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x4);
	CHECK_RETCODE(err); 

	// VCL: -2xVDDB   pump clock : Hsync
	//REGW 0xB8, 0x34, 0x34, 0x34
	buf[0] = 0xB8 |(0x34 << 8) |(0x34 << 16)|(0x34 << 24);
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x4);
	CHECK_RETCODE(err); 

	// VGH: 2xAVDD-AVEE
	//REGW 0xB9, 0x34, 0x34, 0x34
	buf[0] = 0xB9 |(0x34 << 8) |(0x34 << 16)|(0x34 << 24);
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x4);
	CHECK_RETCODE(err);

	// VGLX: AVEE-AVDD
	//REGW 0xBA, 0x14, 0x14, 0x144
	buf[0] = 0xBA |(0x14 << 8) |(0x14 << 16)|(0x14 << 24);
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x4);
	CHECK_RETCODE(err);

	// Set VGMP = 5.2V / VGSP = 0V
	//REGW 0xBC, 0x00, 0xB0 , 0x00
	buf[0] = 0xBC |(0x00 << 8) |(0xB0 << 16)|(0x00 << 24);
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x4);
	CHECK_RETCODE(err);

	// Set VGMN = -5.2V / VGSN = 0V
	//REGW 0xBD, 0x00, 0xB0, 0x00
	buf[0] = 0xBD |(0x00 << 8) |(0xB0 << 16)|(0x00 << 24);
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x4);
	CHECK_RETCODE(err);

	// VMSEL 0: 0xBE00	;  1 : 0xBF00 
	//REGW 0xC1, 0x00
	buf[0] = 0xC1 |(0x00 << 8) ;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x2);
	CHECK_RETCODE(err);

	// Set VCOM_offset
	//REGW 0xBE, 0x46
	buf[0] = 0xBE |(0x46 << 8) ;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x2);
	CHECK_RETCODE(err);

	// Pump:0x00 or PFM:0x50 control
	//REGW 0xC2, 0x00
	buf[0] = 0xC2 |(0x00 << 8) ;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x2);
	CHECK_RETCODE(err);

	// Gamma Gradient Control
	//REGW 0xD0, 0x0F, 0x0F, 0x10, 0x10
	buf[0] = 0xD0 |(0x0F << 8) |(0x0F << 16)|(0x10 << 24);
	buf[0] = 0x10 ;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x5);
	CHECK_RETCODE(err);

//R(+) MCR cmd
	//REGW 0xD1,0x00,0x00,0x00, 0x62,0x00,0x90,0x00,	0xAE,0x00,0xC5,0x00,	0xEA,0x01,0x07,0x01,	0x34
	buf[0] = 0xD1 |(0x00 << 8) |(0x00 << 16)|(0x00 << 24);
	buf[1] = 0x62 |(0x00 << 8) |(0x90 << 16)|(0x00 << 24);
	buf[2] = 0xAE |(0x00 << 8) |(0xC5 << 16)|(0x00 << 24);
	buf[3] = 0xEA |(0x01 << 8) |(0x07 << 16)|(0x01 << 24);
	buf[4] = 0x34 ;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x11);
	CHECK_RETCODE(err);
	//REGW 0xD2,0x01,0x58,0x01, 0x8F,0x01,0xBB,0x01,	0xFF,0x02,0x37,0x02,	0x39,0x02,0x6E,0x02,	0xA9
	buf[0] = 0xD2 |(0x01 << 8) |(0x58 << 16)|(0x01 << 24);
	buf[1] = 0x8F |(0x01 << 8) |(0xBB << 16)|(0x01 << 24);
	buf[2] = 0xFF |(0x02 << 8) |(0x37 << 16)|(0x02 << 24);
	buf[3] = 0x39 |(0x02 << 8) |(0x6E << 16)|(0x02 << 24);
	buf[4] = 0xA9 ;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x11);
	CHECK_RETCODE(err);
	//REGW 0xD3,0x02,0xD0,0x03, 0x07,0x03,0x2C,0x03,	0x5F,0x03,0x81,0x03 ,0xAC,0x03,0xC3,0x03,	0xDD
	buf[0] = 0xD3 |(0x02 << 8) |(0xD0 << 16)|(0x03 << 24);
	buf[1] = 0x07 |(0x03 << 8) |(0x2C << 16)|(0x03 << 24);
	buf[2] = 0x5F |(0x03 << 8) |(0x81 << 16)|(0x03 << 24);
	buf[3] = 0xAC |(0x03 << 8) |(0xC3 << 16)|(0x03 << 24);
	buf[4] = 0xDD ;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x11);
	CHECK_RETCODE(err);
	//REGW 0xD4,0x03,0xF6,0x03, 0xFB
	buf[0] = 0xD4 |(0x03 << 8) |(0xF6 << 16)|(0x03 << 24);
	buf[1] = 0xFB ;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x5);
	CHECK_RETCODE(err);

//G(+) MCR cmd
	//REGW 0xD5,0x00,0x00,0x00, 0x62,0x00,0x90,0x00,	0xAE,0x00,0xC5,0x00,	0xEA,0x01,0x07,0x01,	0x34
	buf[0] = 0xD5 |(0x00 << 8) |(0x00 << 16)|(0x00 << 24);
	buf[1] = 0x62 |(0x00 << 8) |(0x90 << 16)|(0x00 << 24);
	buf[2] = 0xAE |(0x00 << 8) |(0xC5 << 16)|(0x00 << 24);
	buf[3] = 0xEA |(0x01 << 8) |(0x07 << 16)|(0x01 << 24);
	buf[4] = 0x34 ;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x11);
	CHECK_RETCODE(err);
	//REGW 0xD6,0x01,0x58,0x01, 0x8F,0x01,0xBB,0x01,	0xFF,0x02,0x37,0x02,	0x39,0x02,0x6E,0x02,	0xA9
	buf[0] = 0xD6 |(0x01 << 8) |(0x58 << 16)|(0x01 << 24);
	buf[1] = 0x8F |(0x01 << 8) |(0xBB << 16)|(0x01 << 24);
	buf[2] = 0xFF |(0x02 << 8) |(0x37 << 16)|(0x02 << 24);
	buf[3] = 0x39 |(0x02 << 8) |(0x6E << 16)|(0x02 << 24);
	buf[4] = 0xA9 ;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x11);
	CHECK_RETCODE(err);
	//REGW 0xD7,0x02,0xD0,0x03, 0x07,0x03,0x2C,0x03,	0x5F,0x03,0x81,0x03,	0xAC,0x03,0xC3,0x03,	0xDD
	buf[0] = 0xD7 |(0x02 << 8) |(0xD0 << 16)|(0x03 << 24);
	buf[1] = 0x07 |(0x03 << 8) |(0x2C << 16)|(0x03 << 24);
	buf[2] = 0x5F |(0x03 << 8) |(0x81 << 16)|(0x03 << 24);
	buf[3] = 0xAC |(0x03 << 8) |(0xC3 << 16)|(0x03 << 24);
	buf[4] = 0xDD ;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x11);
	CHECK_RETCODE(err);
	//REGW 0xD8,0x03,0xF6,0x03, 0xFB
	buf[0] = 0xD8 |(0x03 << 8) |(0xF6 << 16)|(0x03 << 24);
	buf[1] = 0xFB ;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x5);
	CHECK_RETCODE(err);

//B(+) MCR cmd
	//REGW 0xD9,0x00,0x00,0x00, 0x62,0x00,0x90,0x00,	0xAE,0x00,0xC5,0x00,	0xEA,0x01,0x07,0x01,	0x34
	buf[0] = 0xD9 |(0x00 << 8) |(0x00 << 16)|(0x00 << 24);
	buf[1] = 0x62 |(0x00 << 8) |(0x90 << 16)|(0x00 << 24);
	buf[2] = 0xAE |(0x00 << 8) |(0xC5 << 16)|(0x00 << 24);
	buf[3] = 0xEA |(0x01 << 8) |(0x07 << 16)|(0x01 << 24);
	buf[4] = 0x34 ;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x11);
	CHECK_RETCODE(err);
	//REGW 0xDD,0x01,0x58,0x01, 0x8F,0x01,0xBB,0x01,	0xFF,0x02,0x37,0x02,	0x39,0x02,0x6E,0x02,	0xA9
	buf[0] = 0xDD |(0x01 << 8) |(0x58 << 16)|(0x01 << 24);
	buf[1] = 0x8F |(0x01 << 8) |(0xBB << 16)|(0x01 << 24);
	buf[2] = 0xFF |(0x02 << 8) |(0x37 << 16)|(0x02 << 24);
	buf[3] = 0x39 |(0x02 << 8) |(0x6E << 16)|(0x02 << 24);
	buf[4] = 0xA9 ;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x11);
	CHECK_RETCODE(err);
	//REGW 0xDE,0x02,0xD0,0x03, 0x07,0x03,0x2C,0x03,	0x5F,0x03,0x81,0x03,	0xAC,0x03,0xC3,0x03,	0xDD
	buf[0] = 0xDE |(0x02 << 8) |(0xD0 << 16)|(0x03 << 24);
	buf[1] = 0x07 |(0x03 << 8) |(0x2C << 16)|(0x03 << 24);
	buf[2] = 0x5F |(0x03 << 8) |(0x81 << 16)|(0x03 << 24);
	buf[3] = 0xAC |(0x03 << 8) |(0xC3 << 16)|(0x03 << 24);
	buf[4] = 0xDD ;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x11);
	CHECK_RETCODE(err);
	//REGW 0xDF,0x03,0xF6,0x03, 0xFB
	buf[0] = 0xDF |(0x03 << 8) |(0xF6 << 16)|(0x03 << 24);
	buf[1] = 0xFB ;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x5);
	CHECK_RETCODE(err);

//R(-) MCR cmd
	//REGW 0xE0,0x00,0x00,0x00, 0x62,0x00,0x90,0x00,	0xAE,0x00,0xC5,0x00,	0xEA,0x01,0x07,0x01,	0x34
	buf[0] = 0xE0 |(0x00 << 8) |(0x00 << 16)|(0x00 << 24);
	buf[1] = 0x62 |(0x00 << 8) |(0x90 << 16)|(0x00 << 24);
	buf[2] = 0xAE |(0x00 << 8) |(0xC5 << 16)|(0x00 << 24);
	buf[3] = 0xEA |(0x01 << 8) |(0x07 << 16)|(0x01 << 24);
	buf[4] = 0x34 ;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x11);
	CHECK_RETCODE(err);
	//REGW 0xE1,0x01,0x58,0x01, 0x8F,0x01,0xBB,0x01,	0xFF,0x02,0x37,0x02,	0x39,0x02,0x6E,0x02,	0xA9
	buf[0] = 0xE1 |(0x01 << 8) |(0x58 << 16)|(0x01 << 24);
	buf[1] = 0x8F |(0x01 << 8) |(0xBB << 16)|(0x01 << 24);
	buf[2] = 0xFF |(0x02 << 8) |(0x37 << 16)|(0x02 << 24);
	buf[3] = 0x39 |(0x02 << 8) |(0x6E << 16)|(0x02 << 24);
	buf[4] = 0xA9 ;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x11);
	CHECK_RETCODE(err);
	//REGW 0xE2,0x02,0xD0,0x03, 0x07,0x03,0x2C,0x03,	0x5F,0x03,0x81,0x03,	0xAC,0x03,0xC3,0x03,	0xDD
	buf[0] = 0xE2 |(0x02 << 8) |(0xD0 << 16)|(0x03 << 24);
	buf[1] = 0x07 |(0x03 << 8) |(0x2C << 16)|(0x03 << 24);
	buf[2] = 0x5F |(0x03 << 8) |(0x81 << 16)|(0x03 << 24);
	buf[3] = 0xAC |(0x03 << 8) |(0xC3 << 16)|(0x03 << 24);
	buf[4] = 0xDD ;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x11);
	CHECK_RETCODE(err);
	//REGW 0xE3,0x03,0xF6,0x03,0xFB
	buf[0] = 0xE3 |(0x03 << 8) |(0xF6 << 16)|(0x03 << 24);
	buf[1] = 0xFB ;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x5);
	CHECK_RETCODE(err);

//G(-) MCR cmd
	//REGW 0xE4,0x00,0x00,0x00, 0x62,0x00,0x90,0x00,	0xAE,0x00,0xC5,0x00,	0xEA,0x01,0x07,0x01,	0x34
	buf[0] = 0xE4 |(0x00 << 8) |(0x00 << 16)|(0x00 << 24);
	buf[1] = 0x62 |(0x00 << 8) |(0x90 << 16)|(0x00 << 24);
	buf[2] = 0xAE |(0x00 << 8) |(0xC5 << 16)|(0x00 << 24);
	buf[3] = 0xEA |(0x01 << 8) |(0x07 << 16)|(0x01 << 24);
	buf[4] = 0x34 ;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x11);
	CHECK_RETCODE(err);
	//REGW 0xE5,0x01,0x58,0x01, 0x8F,0x01,0xBB,0x01,	0xFF,0x02,0x37,0x02,	0x39,0x02,0x6E,0x02,	0xA9
	buf[0] = 0xE5 |(0x01 << 8) |(0x58 << 16)|(0x01 << 24);
	buf[1] = 0x8F |(0x01 << 8) |(0xBB << 16)|(0x01 << 24);
	buf[2] = 0xFF |(0x02 << 8) |(0x37 << 16)|(0x02 << 24);
	buf[3] = 0x39 |(0x02 << 8) |(0x6E << 16)|(0x02 << 24);
	buf[4] = 0xA9 ;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x11);
	CHECK_RETCODE(err);
	//REGW 0xE6,0x02,0xD0,0x03, 0x07,0x03,0x2C,0x03,	0x5F,0x03,0x81,0x03,	0xAC,0x03,0xC3,0x03,	0xDD
	buf[0] = 0xE6 |(0x02 << 8) |(0xD0 << 16)|(0x03 << 24);
	buf[1] = 0x07 |(0x03 << 8) |(0x2C << 16)|(0x03 << 24);
	buf[2] = 0x5F |(0x03 << 8) |(0x81 << 16)|(0x03 << 24);
	buf[3] = 0xAC |(0x03 << 8) |(0xC3 << 16)|(0x03 << 24);
	buf[4] = 0xDD ;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x11);
	CHECK_RETCODE(err);
	//REGW 0xE7,0x03,0xF6,0x03,0xFB
	buf[0] = 0xE7 |(0x03 << 8) |(0xF6 << 16)|(0x03 << 24);
	buf[1] = 0xFB ;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x5);
	CHECK_RETCODE(err);

//B(-) MCR cmd
	//REGW 0xE8,0x00,0x00,0x00, 0x62,0x00,0x90,0x00,	0xAE,0x00,0xC5,0x00,	0xEA,0x01,0x07,0x01,	0x34
	buf[0] = 0xE8 |(0x00 << 8) |(0x00 << 16)|(0x00 << 24);
	buf[1] = 0x62 |(0x00 << 8) |(0x90 << 16)|(0x00 << 24);
	buf[2] = 0xAE |(0x00 << 8) |(0xC5 << 16)|(0x00 << 24);
	buf[3] = 0xEA |(0x01 << 8) |(0x07 << 16)|(0x01 << 24);
	buf[4] = 0x34 ;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x11);
	CHECK_RETCODE(err);
	//REGW 0xE9,0x01,0x58,0x01, 0x8F,0x01,0xBB,0x01,	0xFF,0x02,0x37,0x02,	0x39,0x02,0x6E,0x02,	0xA9
	buf[0] = 0xE9 |(0x01 << 8) |(0x58 << 16)|(0x01 << 24);
	buf[1] = 0x8F |(0x01 << 8) |(0xBB << 16)|(0x01 << 24);
	buf[2] = 0xFF |(0x02 << 8) |(0x37 << 16)|(0x02 << 24);
	buf[3] = 0x39 |(0x02 << 8) |(0x6E << 16)|(0x02 << 24);
	buf[4] = 0xA9 ;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x11);
	CHECK_RETCODE(err);
	//REGW 0xEA,0x02,0xD0,0x03, 0x07,0x03,0x2C,0x03,	0x5F,0x03,0x81,0x03,	0xAC,0x03,0xC3,0x03,	0xDD
	buf[0] = 0xEA |(0x02 << 8) |(0xD0 << 16)|(0x03 << 24);
	buf[1] = 0x07 |(0x03 << 8) |(0x2C << 16)|(0x03 << 24);
	buf[2] = 0x5F |(0x03 << 8) |(0x81 << 16)|(0x03 << 24);
	buf[3] = 0xAC |(0x03 << 8) |(0xC3 << 16)|(0x03 << 24);
	buf[4] = 0xDD ;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x11);
	CHECK_RETCODE(err);
	//REGW 0xEB,0x03,0xF6,0x03,0xFB
	buf[0] = 0xEB |(0x03 << 8) |(0xF6 << 16)|(0x03 << 24);
	buf[1] = 0xFB ;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x5);
	CHECK_RETCODE(err);
	//*************************************
	// TE On							   
	//*************************************
	//REGW 0x35,0x00
	buf[0] = 0x35 |(0x00 << 8) ;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
		buf, 0x2);
	CHECK_RETCODE(err);
	
	/* exit sleep mode and set display on */
	buf[0] = MIPI_DCS_EXIT_SLEEP_MODE;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM,
		buf, 0);
	CHECK_RETCODE(err);
	/* To allow time for the supply voltages
	 * and clock circuits to stabilize.
	 */
	msleep(10);
	buf[0] = MIPI_DCS_SET_DISPLAY_ON;
	err = mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM,
		buf, 0);
	CHECK_RETCODE(err);

	return err;

}

int mipid_nt35517_lcd_suspend(struct mipi_dsi_info *mipi_dsi){	
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

int mipid_nt35517_lcd_resume(struct mipi_dsi_info *mipi_dsi){	
	int err=0;	
	
	if(mipi_dsi->lcd_power){
		mipi_dsi->lcd_power(1);
		if(mipi_dsi->reset)
			mipi_dsi->reset();
		mipid_nt35517_setup(mipi_dsi);
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


int mipid_nt35517_lcd_setup(struct mipi_dsi_info *mipi_dsi)
{
	//FIXME skip setup since bootloader already init
	return 0;

}

