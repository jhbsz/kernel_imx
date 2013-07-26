#ifndef GENERIC_DEVICES_H
#define GENERIC_DEVICES_H

struct platform_device *__init generic_add_device_w1(
		int w1_io);
int __init generic_add_device_pn544(
		int bus,int irq,int ven,int fw);


#define generic_add_w1(gpio)	\
	generic_add_device_w1(gpio)

#define generic_add_pn544(bus,irq,ven,fw) \
	generic_add_device_pn544(bus,irq,ven,fw)

#endif
