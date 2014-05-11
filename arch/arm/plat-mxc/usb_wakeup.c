/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/fsl_devices.h>
#include <linux/suspend.h>
#include <linux/io.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <mach/arc_otg.h>
#ifdef CONFIG_PM
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#endif

struct wakeup_ctrl {
	int wakeup_irq;
	int usb_irq;
	bool thread_close;
	struct fsl_usb2_wakeup_platform_data *pdata;
	struct task_struct *thread;
	struct completion  event;

	#ifdef CONFIG_ANDROID
	//only for otg event
	struct early_suspend es;
	struct input_dev	*idev;
	struct delayed_work input_work;
	int report_enabled;
	#endif
};


#ifdef CONFIG_ANDROID

static void usb_otg_input_work(struct work_struct *work){
	struct wakeup_ctrl* ctrl = (struct wakeup_ctrl*)container_of(work,struct wakeup_ctrl,input_work.work);
	if(ctrl->idev){
		printk("report otg input key\n");
		input_report_key(ctrl->idev, KEY_POWER, 1);
		input_report_key(ctrl->idev, KEY_POWER, 0);
		input_sync(ctrl->idev);
	}
}
static void submit_input_work(struct wakeup_ctrl* ctrl){
	if(ctrl->report_enabled){
		schedule_delayed_work(&ctrl->input_work,msecs_to_jiffies(500));
		ctrl->report_enabled=0;
	}
}
static void usb_wakeup_early_suspend(struct early_suspend *h){
	struct wakeup_ctrl* ctrl = (struct wakeup_ctrl*)container_of(h,struct wakeup_ctrl,es);
	ctrl->report_enabled++;
}
static void usb_wakeup_late_resume(struct early_suspend *h){
	struct wakeup_ctrl* ctrl = (struct wakeup_ctrl*)container_of(h,struct wakeup_ctrl,es);
	if(ctrl->report_enabled>0)
		ctrl->report_enabled--;
}
#endif


extern int usb_event_is_otg_wakeup(struct fsl_usb2_platform_data *pdata);
extern void usb_debounce_id_vbus(void);

static void wakeup_clk_gate(struct fsl_usb2_wakeup_platform_data *pdata, bool on)
{
	if (pdata->usb_clock_for_pm)
		pdata->usb_clock_for_pm(on);
}
static bool phy_in_lowpower_mode(struct fsl_usb2_platform_data *pdata)
{
	unsigned long flags;
	bool ret = true;

	spin_lock_irqsave(&pdata->lock, flags);

	if (!pdata->lowpower)
		ret = false;

	spin_unlock_irqrestore(&pdata->lock, flags);

	return ret;
}

static bool usb2_is_in_lowpower(struct wakeup_ctrl *ctrl)
{
	int i;
	struct fsl_usb2_wakeup_platform_data *pdata = ctrl->pdata;
	/* all the usb module related the wakeup is in lowpower mode */
	for (i = 0; i < 3; i++) {
		if (pdata->usb_pdata[i] && pdata->usb_pdata[i]->phy_lowpower_suspend) {
			if (!phy_in_lowpower_mode(pdata->usb_pdata[i]))
				return false;
		}
	}

	return true;
}

static void delay_process_wakeup(struct wakeup_ctrl *ctrl)
{
	int i;
	struct fsl_usb2_wakeup_platform_data *pdata = ctrl->pdata;
	disable_irq_nosync(ctrl->wakeup_irq);
	if ((ctrl->usb_irq > 0) && (ctrl->wakeup_irq != ctrl->usb_irq))
		disable_irq_nosync(ctrl->usb_irq);

	for (i = 0; i < 3; i++) {
		if (pdata->usb_pdata[i]) {
			pdata->usb_pdata[i]->irq_delay = 1;
		}
	}

	pdata->usb_wakeup_is_pending = true;
	complete(&ctrl->event);
}

static irqreturn_t usb_wakeup_handler(int irq, void *_dev)
{
	struct wakeup_ctrl *ctrl = (struct wakeup_ctrl *)_dev;
	irqreturn_t ret = IRQ_NONE;
	if (usb2_is_in_lowpower(ctrl)) {
		pr_debug("usb wakeup is here\n");
		delay_process_wakeup(ctrl);
		ret = IRQ_HANDLED;
	}
	return ret;
}

static enum usb_wakeup_event is_wakeup(struct fsl_usb2_platform_data *pdata)
{
	if (pdata->is_wakeup_event)
		return pdata->is_wakeup_event(pdata);
	else
		return WAKEUP_EVENT_INVALID;
}

static void wakeup_event_handler(struct wakeup_ctrl *ctrl)
{
	struct fsl_usb2_wakeup_platform_data *pdata = ctrl->pdata;
	int already_waked = 0;
	enum usb_wakeup_event wakeup_evt;
	int i;

	wakeup_clk_gate(ctrl->pdata, true);

	for (i = 0; i < 3; i++) {
		struct fsl_usb2_platform_data *usb_pdata = pdata->usb_pdata[i];
		if (usb_pdata) {
			/* In order to get the real id/vbus value */
			if (usb_event_is_otg_wakeup(usb_pdata)){
				usb_debounce_id_vbus();
			}

			usb_pdata->irq_delay = 0;
			wakeup_evt = is_wakeup(usb_pdata);
			usb_pdata->wakeup_event = wakeup_evt;
			#if defined(CONFIG_ANDROID)&&defined(CONFIG_EARLYSUSPEND)
			if(!strcmp(pdata->name,"DR wakeup")){
				submit_input_work(ctrl);
			}
			#endif
			if (wakeup_evt != WAKEUP_EVENT_INVALID) {
				if (usb2_is_in_lowpower(ctrl))
					if (usb_pdata->usb_clock_for_pm)
						usb_pdata->usb_clock_for_pm(true);
				usb_pdata->lowpower = 0;
				already_waked = 1;
				if (usb_pdata->wakeup_handler) {
					usb_pdata->wakeup_handler(usb_pdata);
				}
			}
		}
	}

	/* If nothing to wakeup, clear wakeup event */
	if ((already_waked == 0) && pdata->usb_wakeup_exhandle)
		pdata->usb_wakeup_exhandle();

	wakeup_clk_gate(ctrl->pdata, false);
	pdata->usb_wakeup_is_pending = false;
	wake_up(&pdata->wq);
}

static int wakeup_event_thread(void *param)
{
	struct wakeup_ctrl *ctrl = (struct wakeup_ctrl *)param;
	struct sched_param sch_param = {.sched_priority = 1};
	u32 timeout = 0;

	sched_setscheduler(current, SCHED_RR, &sch_param);
	while (1) {
		wait_for_completion_interruptible(&ctrl->event);
		if (ctrl->thread_close) {
			while (!kthread_should_stop() && (timeout < 1000)) {
				timeout++;
				msleep(1);
			}
			break;
		}
		wakeup_event_handler(ctrl);
		enable_irq(ctrl->wakeup_irq);
		if ((ctrl->usb_irq > 0) && (ctrl->wakeup_irq != ctrl->usb_irq))
			enable_irq(ctrl->usb_irq);
	}
	return 0;
}

static int wakeup_dev_probe(struct platform_device *pdev)
{
	struct fsl_usb2_wakeup_platform_data *pdata;
	struct wakeup_ctrl *ctrl = NULL;
	int status;
	unsigned long interrupt_flag;

	printk(KERN_INFO "IMX usb wakeup probe\n");

	if (!pdev || !pdev->dev.platform_data)
		return -ENODEV;
	ctrl = kzalloc(sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;
	pdata = pdev->dev.platform_data;
	ctrl->pdata = pdata;
	init_waitqueue_head(&pdata->wq);
	pdata->usb_wakeup_is_pending = false;

	init_completion(&ctrl->event);
	/* Currently, both mx5x and mx6q uses usb controller's irq
	 * as wakeup irq.
	 */
	ctrl->wakeup_irq = platform_get_irq(pdev, 1);
	ctrl->usb_irq = platform_get_irq(pdev, 1);
	ctrl->thread_close = false;
	if (ctrl->wakeup_irq != ctrl->usb_irq)
		interrupt_flag = IRQF_DISABLED;
	else
		interrupt_flag = IRQF_SHARED;
	status = request_irq(ctrl->wakeup_irq, usb_wakeup_handler, interrupt_flag, "usb_wakeup", (void *)ctrl);
	if (status)
		goto error1;

	ctrl->thread = kthread_run(wakeup_event_thread, (void *)ctrl, "usb_wakeup thread");
	status = IS_ERR(ctrl->thread) ? -1 : 0;
	if (status)
		goto error2;
	platform_set_drvdata(pdev, ctrl);


#ifdef CONFIG_ANDROID
	INIT_DELAYED_WORK(&ctrl->input_work,usb_otg_input_work);
	ctrl->idev= input_allocate_device();
	if (!ctrl->idev) {
		dev_err(&pdev->dev, "Failed to allocate input dev\n");
		return -ENOMEM;
	}

	ctrl->idev->name = "usb_wakeup";
	ctrl->idev->phys = "usb_wakeup/input0";
	ctrl->idev->dev.parent = &pdev->dev;
	ctrl->idev->evbit[0] = BIT_MASK(EV_KEY);
	ctrl->idev->keybit[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER);
	status = input_register_device(ctrl->idev);
	if (status) {
		dev_err(&pdev->dev, "Can't register usb dr input device: %d\n", status);
		input_free_device(ctrl->idev);
		return -ENODEV;
	}
	#ifdef CONFIG_PM
	#ifdef CONFIG_HAS_EARLYSUSPEND
	ctrl->es.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	ctrl->es.suspend = usb_wakeup_early_suspend;
	ctrl->es.resume = usb_wakeup_late_resume;
	register_early_suspend(&ctrl->es);
	#endif
	#endif
#endif

	printk(KERN_DEBUG "the wakeup pdata is 0x%p\n", pdata);
	return 0;
error2:
	free_irq(ctrl->wakeup_irq, (void *)ctrl);
error1:
	kfree(ctrl);
	return status;
}

static int  wakeup_dev_exit(struct platform_device *pdev)
{
	struct wakeup_ctrl *wctrl = platform_get_drvdata(pdev);

	wctrl->thread_close = true;
	complete(&wctrl->event);
	kthread_stop(wctrl->thread);
	free_irq(wctrl->wakeup_irq, (void *)wctrl);
	kfree(wctrl);

	return 0;
}
static struct platform_driver wakeup_d = {
	.probe   = wakeup_dev_probe,
	.remove  = wakeup_dev_exit,
	.driver = {
		.name = "usb-wakeup",
	},
};

static int __init wakeup_dev_init(void)
{
	return platform_driver_register(&wakeup_d);
}
static void __exit wakeup_dev_uninit(void)
{
	platform_driver_unregister(&wakeup_d);
}

subsys_initcall(wakeup_dev_init);
module_exit(wakeup_dev_uninit);

