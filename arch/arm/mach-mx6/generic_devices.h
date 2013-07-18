
#include <mach/devices-common.h>

#define generic_add_w1(gpio)	\
	generic_add_device_w1(gpio)

#define generic_add_pn544(bus,irq,ven,fw) \
	generic_add_device_pn544(bus,irq,ven,fw)

