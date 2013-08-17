#ifndef __FT5X0X_TS_H__
#define __FT5X0X_TS_H__

#define FT5X0X_QUIRK_TOUCHKEY_ENABLE 0x1
struct ft5x0x_ts_platform_data
{
	int quirks;
	int (*plat_init)(void);
	int xyswap;
	int x_inverted;
	int y_inverted;
};

#endif

