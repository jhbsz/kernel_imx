#ifndef TOUCHSCREEN_H
#define TOUCHSCREEN_H
struct touchscreen_platform_data
{
	int (*plat_init)(void);
	int (*setpower)(int on);	
	int (*reset)(void);
	int xyswap;
	int x_inverted;
	int y_inverted;
	int quirks;
};

#endif

