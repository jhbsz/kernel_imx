/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef __QPOWER_H__
#define __QPOWER_H__

/*
 * We define shared qpower ops resources and pass it to low level driver.
 * Low level may access it and assign it to proper operation.
 * And it's very important that the shared resources must be defined as existed.
 *
 * Example1:
	struct qpower_ops qops = {
		.qbo = xxx,
		.qco = xxx,
		.qce = xxx,
	};
	struct qpower_battery_pdata qbp = {
		 .alert = xx,
		 .alert_threshold = xx,
		 .ops = &qops,
	};
	struct qpower_charger_pdata qcp = {
		 .xx = xx,
		 .ops = &qops,
	};
	struct qpower_pdata qp = {
		.bp = &qbp,
		.cp = &qcp,
		.flags = QPOWER_FEATURE_CHARGER|QPOWER_FEATURE_BATTERY,   	
	};

   Example2 (no ops defined):
	struct qpower_battery_pdata qbp = {
		.alert = xx,
		.alert_threshold = xx,
	};
	struct qpower_charger_pdata qcp = {
		.xx = xx,
	};
	struct qpower_pdata qp = {
		.bp = &qbp,
		.cp = &qcp,
		.flags = QPOWER_FEATURE_CHARGER|QPOWER_FEATURE_BATTERY,	 
	};

	For all gpio definitions of platform data, zero means invalid gpio pin

*/

//return 1:battery is online ,0:battery is offline
typedef int (*qpower_battery_online)(void* drvdata);

//return 1:battery is faulty,0: battery is okay
typedef int (*qpower_battery_faulty)(void* drvdata);

//return 1:charger is online,0:charger is offline
typedef int (*qpower_charger_online)(void* drvdata);

//return 1:charger is enabled,0:charger is disabled
typedef int (*qpower_charger_enable)(void* drvdata);



#define battery_faulty(ops) \
	ops->qbf(ops->drv_data)
#define battery_online(ops) \
		ops->qbo(ops->drv_data)
#define charger_online(ops) \
		ops->qco(ops->drv_data)
#define charger_enable(ops) \
		ops->qce(ops->drv_data)


struct qpower_ops{
	qpower_battery_online qbo;
	qpower_charger_online qco;
	qpower_charger_enable qce;
	qpower_battery_faulty qbf;

	void* drv_data;
};

struct qpower_battery_pdata{	
	int alert_irq;
	int alert_threshold;

	//bus id ,e.g i2c-0 
	int busid;
	
	//common of qpower driver	
	struct qpower_ops*   ops;
};

#define QPOWER_CHARGER_FEATURE_SHORT_MODE	0x1 /*DC & USB source shorted*/

struct qpower_charger_pdata{
	/*
	 * GPIOs
	 * cen, chg, flt, and usus are optional.
	 * dok, dcm, and uok are not optional depending on the status of
	 * dc_valid and usb_valid.
	 */
	int cen;	/* Charger Enable input */
	int dok;	/* DC(Adapter) Power OK output */
	int uok;	/* USB Power OK output */
	int chg;	/* Charger status output */
	int flt;	/* Fault output */
	int dcm;	/* Current-Limit Mode input (1: DC, 2: USB) */
	int usus;	/* USB Suspend Input (1: suspended) */
	
	int feature_flag;

    /* DCM wired to Logic High
       Set this true when DCM pin connect to Logic high.*/
	bool dcm_always_high;

	/*
	 * DC(Adapter/TA) is wired
	 * When dc_valid is true,
	 *	dok and dcm should be valid.
	 *
	 * At least one of dc_valid or usb_valid should be true.
	 */
	bool dc_valid;
	/*
	 * USB is wired
	 * When usb_valid is true,
	 *	uok should be valid.
	 */
	bool usb_valid;


	//common of qpower driver	
	struct qpower_ops*   ops;
};


#define QPOWER_FEATURE_CHARGER  0x01
#define QPOWER_FEATURE_BATTERY	0x02

struct qpower_pdata {
	struct qpower_battery_pdata* bp;
	struct qpower_charger_pdata* cp;

	//XORed flags to indicate valid pdata
	unsigned int flags;
	
};

#endif /* __QPOWER_H__ */
