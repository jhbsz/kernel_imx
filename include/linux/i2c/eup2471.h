#ifndef I2C_EUP2471_H
#define I2C_EUP2471_H
struct eup2471_platform_data
{
	const char* name;
	const char* default_trigger;
	int (*enable)(int on);
	int (*flash)(int on);
};
#endif

