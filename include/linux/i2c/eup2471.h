struct eup2471_platform_data
{
	int (*enable)(int on);
	int (*flash)(int on);
};

