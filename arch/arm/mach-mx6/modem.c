#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/wakelock.h>

#ifndef MODEM_WAKEUP_ENABLED
#define MODEM_WAKEUP_ENABLED
#endif

#ifndef MODEM_PIN_POWER
#define MODEM_PIN_POWER 121
#endif

#ifndef MODEM_PIN_RESET
#define MODEM_PIN_RESET 127
#endif

#ifndef MODEM_PIN_ONOFF
#define MODEM_PIN_ONOFF 126
#endif

#ifndef MODEM_PIN_WAKEAP
#define MODEM_PIN_WAKEAP 46
#endif

#ifndef MODEM_PIN_WAKEMODEM
#define MODEM_PIN_WAKEMODEM	142
#endif

//AP status
#ifdef MODEM_AP_SLEEP_STATUS
#define AP_SLEEP_STATUS MODEM_AP_SLEEP_STATUS
#else
#define AP_SLEEP_STATUS (-1)
#endif
#ifndef AP_SLEEP_STATUS_ALIVE
#define AP_SLEEP_STATUS_ALIVE 0
#endif
#ifndef AP_SLEEP_STATUS_SLEEP
#define AP_SLEEP_STATUS_SLEEP 1
#endif

//CP status
#ifdef MODEM_CP_SLEEP_STATUS
#define CP_SLEEP_STATUS MODEM_CP_SLEEP_STATUS
#else
#define CP_SLEEP_STATUS (-1)
#endif
#ifndef CP_SLEEP_STATUS_ALIVE
#define CP_SLEEP_STATUS_ALIVE 0
#endif
#ifndef CP_SLEEP_STATUS_SLEEP
#define CP_SLEEP_STATUS_SLEEP 1
#endif

//push to talk
#ifdef MODEM_CP_PUSH_TO_TALK
#define CP_PUSH_TO_TALK MODEM_CP_PUSH_TO_TALK
#else
#define CP_PUSH_TO_TALK (-1)
#endif


#define MODEM_STATE_INVALID 0
#define MODEM_STATE_ON      1
#define MODEM_STATE_OFF		2


struct usb_modem;

struct modem_work
{
	struct delayed_work work;
	void (*work_callback)(struct usb_modem* modem);	
};

struct usb_modem
{
	struct modem_work reset_work;
	struct modem_work enable_work;
	struct modem_work command_work;

	int gpio_reset;
	int gpio_onoff;
	int gpio_power;  	//negative valid
	int gpio_wakeap;	//modem wake up host
	int gpio_wakemodem;		//host wakeup modem
	int gpio_cp_sleep_status;
	int gpio_ap_sleep_status;
	int gpio_cp_ptt;

	unsigned long mfpr_pin_power;
	unsigned long mfpr_pin_onoff;	
	unsigned long mfpr_pin_reset;

	int modem_on_timing_ms;
	int modem_off_timing_ms;
	//
	int current_state;
	int request_state;


	struct input_dev	*idev;
	int link_state_change;
	int screen_state;


	struct platform_device* pdev;

	int host_suspend;

	//bit masked
	unsigned long commands;

	wait_queue_head_t	wakeup_wqueue;	

	int irq_enable_count;

	
	struct wake_lock modem_wakelock;

	unsigned int no_pin_onoff:1;

};

struct usb_modem_plat_data
{
	struct usb_modem* modem;
};


static void set_ap_status(struct usb_modem* modem,int awake){
	if(gpio_is_valid(modem->gpio_ap_sleep_status)){
		gpio_set_value(modem->gpio_ap_sleep_status,
			awake?AP_SLEEP_STATUS_ALIVE:AP_SLEEP_STATUS_SLEEP);
	}
}

static void modem_set_cp_awake(struct usb_modem* modem){
	if(gpio_is_valid(modem->gpio_cp_sleep_status)){
		int sleep_state = gpio_get_value(modem->gpio_cp_sleep_status)?1:0;
		if(sleep_state!=CP_SLEEP_STATUS_ALIVE){
			//wakeup cp now via wakemodem rising trigger
			if(modem->gpio_wakemodem>0){
				gpio_set_value(modem->gpio_wakemodem,0);
				udelay(1);
				gpio_set_value(modem->gpio_wakemodem,1);
				udelay(1);
				gpio_set_value(modem->gpio_wakemodem,0);
			}
		}
	}
}


static void modem_command_work(struct work_struct *work)
{
	struct usb_modem *modem = (struct usb_modem*)container_of(work,struct usb_modem,command_work.work.work);	
	if(modem->commands)
	{
		//TODO
	}


	if(modem->command_work.work_callback)
		modem->command_work.work_callback(modem);

	
}


static int request_modem_state(struct usb_modem* modem,int state)
{
	int ret=0;
	//if(modem->current_state!=state)
	{
		modem->request_state = state;
		
		/*printk("request usb modem state to %s time=%dms\n",(MODEM_STATE_ON==state)?"on":
			(MODEM_STATE_OFF==state)?"off":"invalid",
			(MODEM_STATE_ON==state)?modem->modem_on_timing_ms:modem->modem_off_timing_ms);*/
		 
		if(MODEM_STATE_ON==state)
		{
			//power on
			gpio_direction_output(modem->gpio_power,1);//power on	
 			if(!modem->no_pin_onoff){
				gpio_direction_output(modem->gpio_onoff,1);
			}
			msleep(10);//delay some time to reset
			gpio_direction_output(modem->gpio_reset,0);
			msleep(50);//delay some time to reset
			gpio_direction_output(modem->gpio_reset,1);	
			if(!modem->no_pin_onoff){
				gpio_direction_output(modem->gpio_onoff,0);
			}
			schedule_delayed_work(&modem->enable_work.work, msecs_to_jiffies(modem->modem_on_timing_ms));
		}
		else if(MODEM_STATE_OFF==state)
		{
			if(!modem->no_pin_onoff){
				gpio_direction_output(modem->gpio_onoff,1);
				gpio_direction_output(modem->gpio_onoff,0);
				schedule_delayed_work(&modem->enable_work.work, msecs_to_jiffies(modem->modem_off_timing_ms));
			}
			//just power off	
			gpio_direction_output(modem->gpio_power,0);//power off
 		}
		
	}
	return ret;

}

static void modem_enable_callback(struct usb_modem* modem)
{
	modem->current_state = modem->request_state;

	modem_set_cp_awake(modem);
}


//per MG3732 spec, on is 500~1000ms low level pulse
//off is 2500~4000ms low level
//so we use deferred work to switch on 
static void modem_reset_callback(struct usb_modem* modem)
{
	//delay some time to request state on
	msleep(500);
	request_modem_state(modem,MODEM_STATE_ON);
}

static void modem_reset_work(struct work_struct *work)
{
	struct usb_modem *modem = (struct usb_modem*)container_of(work,struct usb_modem,reset_work.work.work);	

	gpio_direction_output(modem->gpio_reset,1);

	if(modem->reset_work.work_callback)
		(*modem->reset_work.work_callback)(modem);


}
static void modem_enable_work(struct work_struct *work)
{
	struct usb_modem *modem = (struct usb_modem*)container_of(work,struct usb_modem,enable_work.work.work);	
	 //per MG3732 spec, on is 500~1000ms low level pulse
	 //off is 2500~4000ms low level
	 if(!modem->no_pin_onoff){
		 gpio_direction_output(modem->gpio_onoff,1);
 	 }
	 if(modem->enable_work.work_callback)
		 (*modem->enable_work.work_callback)(modem);
}


//
//this is some a litte noisy,we have to set mfpr for power pin config
//
static void usb_modem_set_pin(struct usb_modem* modem,const char* func,const char* state)
{
	if(!strcmp(func,"power")){
		if(!strcmp(state,"off"))
		{		
			//gpio_set_value(modem->gpio_power,1);
			gpio_direction_output(modem->gpio_power,0);//power off
 		}
		else if(!strcmp(state,"on"))
		{
			//gpio_set_value(modem->gpio_power,0);		
			gpio_direction_output(modem->gpio_power,1);//power on
 		}
	}else if(!strcmp(func,"reset")){
		if(!strcmp(state,"off"))
		{
			gpio_set_value(modem->gpio_reset,1);
		}
		else if(!strcmp(state,"on"))
		{
			gpio_set_value(modem->gpio_reset,0);
		}
	}else if(!strcmp(func,"onoff")){
		if(!strcmp(state,"on"))
		{
			gpio_set_value(modem->gpio_onoff,1);
		}
		else if(!strcmp(state,"off"))
		{
			gpio_set_value(modem->gpio_onoff,0);
		}else if(!strcmp(state,"skip")){
			modem->no_pin_onoff=1; 			
		}
	}else if(!strcmp(func,"wake")){
		if(gpio_is_valid(modem->gpio_wakemodem))
		{
			if(!strcmp(state,"on"))
			{
				gpio_set_value(modem->gpio_wakemodem,1);
			}
			else if(!strcmp(state,"off"))
			{
				gpio_set_value(modem->gpio_wakemodem,0);
			}
		}
	}else if(!strcmp(func,"ptt")){
		if(gpio_is_valid(modem->gpio_cp_ptt))
		{
			if(!strcmp(state,"on"))
			{
				gpio_set_value(modem->gpio_cp_ptt,1);
			}
			else if(!strcmp(state,"off"))
			{
				gpio_set_value(modem->gpio_cp_ptt,0);
			}
		}
	}

}
static ssize_t usb_modem_show(struct device *dev,
                            struct device_attribute *attr, char *buf)
{
	struct usb_modem* modem = platform_get_drvdata(to_platform_device(dev));
	int pin_state;
	ssize_t len=0;
	len += sprintf(buf+len, "power [%s]\t\n", (gpio_get_value(modem->gpio_power)>0)?"on":"off");
	len += sprintf(buf+len, "reset [%s]\t\n", (gpio_get_value(modem->gpio_reset)>0)?"off":"on");
	len += sprintf(buf+len, "onoff [%s]\t\n", (gpio_get_value(modem->gpio_onoff)>0)?"on":"off");
	if(gpio_is_valid(modem->gpio_ap_sleep_status)){
		pin_state =  gpio_get_value(modem->gpio_ap_sleep_status);
		len += sprintf(buf+len, "ap_sleep [%s]\t\n", (pin_state==AP_SLEEP_STATUS_SLEEP)?"sleep":"alive");
	}
	if(gpio_is_valid(modem->gpio_cp_sleep_status)){
		pin_state =  gpio_get_value(modem->gpio_cp_sleep_status);
		len += sprintf(buf+len, "cp_sleep [%s]\t\n", (pin_state==CP_SLEEP_STATUS_SLEEP)?"sleep":"alive");
	}
	if(gpio_is_valid(modem->gpio_cp_ptt)){
		pin_state =  gpio_get_value(modem->gpio_cp_ptt);
		len += sprintf(buf+len, "cp_ptt [%s]\t\n", (pin_state>0)?"high":"low");
	}

	if(gpio_is_valid(modem->gpio_wakemodem))
		len += sprintf(buf+len, "wake [%s]\t\n", (gpio_get_value(modem->gpio_wakemodem)>0)?"on":"off");
	
	if(gpio_is_valid(modem->gpio_wakeap))
		len += sprintf(buf+len, "hostwake [%s]\t\n", (gpio_get_value(modem->gpio_wakeap)>0)?"high":"low");
	len += sprintf(buf+len, "modem state \"%s\"\t\n", (MODEM_STATE_ON==modem->current_state)?"on":
		(MODEM_STATE_OFF==modem->current_state)?"off":"invalid");
	len += sprintf(buf+len, "\n\necho state <on|off> [time_ms]\trequest state on or off with optional time_ms \n");
	len += sprintf(buf+len, "echo <power|reset|onoff|wake|ptt> <on|off|skip>\tset control pin on off\n");

	return len;	
}

static ssize_t usb_modem_store(struct device* dev,	struct device_attribute* attr,
	const char* buf,	size_t count)
{
	struct usb_modem* modem = platform_get_drvdata(to_platform_device(dev));
	char func[48];
	char state[48];
	int filled;
	int time_ms = -1;
    filled = sscanf(buf,"%s %s,%d",func,state,&time_ms);
	if(3!=filled)
		time_ms = -1;
	if(time_ms>10000)
		time_ms = 10000;
	if(filled>=2)
	{
		if(!strcmp(func,"state"))
		{
			if(!strcmp(state,"on"))
			{
				if(-1!=time_ms)
					modem->modem_on_timing_ms = time_ms;
				request_modem_state(modem,MODEM_STATE_ON);
			}
			else if(!strcmp(state,"off"))
			{
				if(-1!=time_ms)
					modem->modem_off_timing_ms = time_ms;
				request_modem_state(modem,MODEM_STATE_OFF);			
			}
				
		}
		else
			usb_modem_set_pin(modem,func,state);

	}
	
	return count;
}



static ssize_t link_state_show(struct device *dev,
                            struct device_attribute *attr, char *buf)
{
	struct usb_modem* modem = platform_get_drvdata(to_platform_device(dev));
	int link_state=0;
	wait_event_interruptible(modem->wakeup_wqueue,modem->link_state_change!=0);
	link_state=modem->link_state_change;
	modem->link_state_change=0;
	return sprintf(buf,"%d\n",link_state);
}
static ssize_t link_state_store(struct device* dev,	struct device_attribute* attr,
	const char* buf,	size_t count)
{
	struct usb_modem* modem = platform_get_drvdata(to_platform_device(dev));
	int new_link_state;
	if(1==sscanf(buf,"%d",&new_link_state))
	{
		modem->link_state_change = new_link_state;
	}
	return count;
	
}

static ssize_t screen_state_show(struct device *dev,
                            struct device_attribute *attr, char *buf)
{
	struct usb_modem* modem = platform_get_drvdata(to_platform_device(dev));
	return sprintf(buf,"%d\n",modem->screen_state);
}
static ssize_t screen_state_store(struct device* dev,	struct device_attribute* attr,
	const char* buf,	size_t count)
{
	struct usb_modem* modem = platform_get_drvdata(to_platform_device(dev));
	int new_link_state;
	if(1==sscanf(buf,"%d",&new_link_state))
	{
		modem->screen_state = new_link_state;
	}
	return count;
	
}

static ssize_t wake_state_show(struct device *dev,
                            struct device_attribute *attr, char *buf)
{
	struct usb_modem* modem = platform_get_drvdata(to_platform_device(dev));
	
	if(gpio_is_valid(modem->gpio_wakemodem))
		return sprintf(buf,"%d\n",(gpio_get_value(modem->gpio_wakemodem)>0)?1:0);
	else
		return sprintf(buf,"wake up pin invalid\n");
		
}
static ssize_t wake_state_store(struct device* dev,	struct device_attribute* attr,
	const char* buf,	size_t count)
{
	struct usb_modem* modem = platform_get_drvdata(to_platform_device(dev));
	int new_state;
	if(1==sscanf(buf,"%d",&new_state))
	{
		if(gpio_is_valid(modem->gpio_wakemodem))
			gpio_direction_output(modem->gpio_wakemodem,new_state);
	}
	return count;
	
}


static DEVICE_ATTR(usb_modem, 0666, usb_modem_show, usb_modem_store);
static DEVICE_ATTR(link_state, 0666, link_state_show, link_state_store);
static DEVICE_ATTR(screen_state, 0666, screen_state_show, screen_state_store);
static DEVICE_ATTR(wake_state, 0666, wake_state_show, wake_state_store);


#ifdef MODEM_WAKEUP_ENABLED

static int acquire_wakeup_time;
struct usb_modem* mymodem;

static int pm_suspend_notifier(struct notifier_block *nb,
				unsigned long event,
				void *dummy)
{
	switch (event) {
	case PM_SUSPEND_PREPARE:
		return NOTIFY_OK;
	case PM_POST_SUSPEND:{
			if(acquire_wakeup_time&&mymodem){
				acquire_wakeup_time=0;				
				wake_lock_timeout(&mymodem->modem_wakelock, HZ * 30);
			}
		}
		return NOTIFY_OK;
	default:
		return NOTIFY_DONE;
	}
}
static struct notifier_block pm_notify_block = {
	.notifier_call = pm_suspend_notifier,
};

static irqreturn_t usb_modem_irq_handler(int irq, void *data)
{
	struct usb_modem* modem = data;
	//struct usb_modem* m = data;
	//just for wakeup AP,dummy impl?
	printk("\n\n\nmodem wakeup\n\n\n");	

	if(!modem->link_state_change)
	{
		if(modem->host_suspend)
		{
			modem->link_state_change = 2;
			modem->host_suspend=0;
		}
		else
			modem->link_state_change = 1;
		wake_up_interruptible(&modem->wakeup_wqueue);
		
	}
	
	if(0==modem->screen_state)
	{
		acquire_wakeup_time=1;
		mymodem = modem;
	}
	
	return IRQ_HANDLED;
}

#endif

static int __devinit usb_modem_probe(struct platform_device *pdev)
{
	struct usb_modem_plat_data* pdata = pdev->dev.platform_data;
	struct usb_modem* modem;
	int ret;

	if(!pdata) return -ENODEV;

	modem = pdata->modem;
	

	platform_set_drvdata(pdev,modem);
	modem->pdev = pdev;

	//register device attributes
    ret = device_create_file(&pdev->dev,&dev_attr_usb_modem);
    ret = device_create_file(&pdev->dev,&dev_attr_link_state);
    ret = device_create_file(&pdev->dev,&dev_attr_screen_state);
    ret = device_create_file(&pdev->dev,&dev_attr_wake_state);
	
	init_waitqueue_head(&modem->wakeup_wqueue);
	//request modem host wakeup irq	
	#ifdef MODEM_WAKEUP_ENABLED

	//set host wake as input
	gpio_direction_input(modem->gpio_wakeap);
	ret = request_any_context_irq(gpio_to_irq(modem->gpio_wakeap), usb_modem_irq_handler,
		IRQF_NO_SUSPEND | IRQF_EARLY_RESUME | IRQF_TRIGGER_FALLING ,
		"modem wakeap", modem);
	if (ret) {
		pr_warning("Request modem hostwake irq failed %d\n", ret);				
	}
	else
	{
		modem->irq_enable_count = 0;
	}
	register_pm_notifier(&pm_notify_block);
	#endif
	

	return ret;
}


#ifdef CONFIG_PM
static int modem_suspend(struct device *dev)
{
	
	#ifdef MODEM_WAKEUP_ENABLED
	struct platform_device *pdev = to_platform_device(dev);	
	struct usb_modem* modem = platform_get_drvdata(pdev);	
	modem->irq_enable_count++;
	enable_irq_wake(gpio_to_irq(modem->gpio_wakeap));
	#endif
	modem->host_suspend = 1;
	set_ap_status(modem,0);
 	return 0;
}

static int modem_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);	
	struct usb_modem* modem = platform_get_drvdata(pdev);	
	
 	modem->link_state_change = 1;
	wake_up_interruptible(&modem->wakeup_wqueue);	 

	
	#ifdef MODEM_WAKEUP_ENABLED
	modem->irq_enable_count--;
	disable_irq_wake(gpio_to_irq(modem->gpio_wakeap));
	#endif

	set_ap_status(modem,1);
	modem_set_cp_awake(modem);
	return 0;
}

static const struct dev_pm_ops modem_pm_ops = {
	.suspend	= modem_suspend,
	.resume		= modem_resume,
};
#endif


static struct platform_driver usb_modem_driver =
{
	.probe		= usb_modem_probe,	
	.driver		= {			
		.name	= "usb_modem",
		.owner	= THIS_MODULE,	
		#ifdef CONFIG_PM
		.pm = &modem_pm_ops,
		#endif
	},	
};

static struct platform_device usb_modem_device = 
{
	.name	= "usb_modem",
	.id = -1,
};

static int __init board_modem_init(void)
{
	struct usb_modem* modem = kzalloc(sizeof(struct usb_modem),GFP_KERNEL);
	struct usb_modem_plat_data* modem_pdata=kzalloc(sizeof(struct usb_modem_plat_data),GFP_KERNEL);

	if(!modem||!modem_pdata)
	{
		pr_warning("%s out of memory\n",__func__);
		return -ENOMEM;
	}
	modem_pdata->modem = modem;
	//request gpio pins
	modem->gpio_power = MODEM_PIN_POWER;
 	modem->gpio_reset = MODEM_PIN_RESET;
 	modem->gpio_onoff = MODEM_PIN_ONOFF;
 #ifdef MODEM_WAKEUP_ENABLED	
	modem->gpio_wakeap  	= MODEM_PIN_WAKEAP;
	modem->gpio_wakemodem	= MODEM_PIN_WAKEMODEM;
#else
	modem->gpio_wakeap = -1;
	modem->gpio_wakemodem	= -1;
#endif
	modem->gpio_ap_sleep_status = AP_SLEEP_STATUS;
	modem->gpio_cp_sleep_status = CP_SLEEP_STATUS;
	modem->gpio_cp_ptt 			= CP_PUSH_TO_TALK;
	modem->modem_on_timing_ms = 3000;
	modem->modem_off_timing_ms = 3000;
	modem->host_suspend = 0;
	modem->screen_state = 1;

	
	wake_lock_init(&modem->modem_wakelock , WAKE_LOCK_SUSPEND, "modem_wl");
	if (gpio_request(modem->gpio_power, "modem power")) {
	   pr_warning("request modem power GPIO failed\n");
	   kfree(modem);
	   return -EBUSY;
	}else if (gpio_request(modem->gpio_onoff, "modem onoff")) {
	  pr_warning("request modem onoff GPIO failed\n");
	  kfree(modem);
	  return -EBUSY;
	}else if (gpio_request(modem->gpio_reset, "modem reset")) {
	  pr_warning("request modem reset GPIO failed\n");
	  kfree(modem);
	  return -EBUSY;
	}	
#ifdef MODEM_WAKEUP_ENABLED
	else if (gpio_request(modem->gpio_wakeap, "modem wakeap")) 
	{
	  pr_warning("request modem hostwake GPIO failed\n");
	  kfree(modem);
	  return -EBUSY;
	}	
	else if (gpio_request(modem->gpio_wakemodem, "modem wakemodem")) 
	{
	  pr_warning("request modem wake GPIO failed\n");
	  kfree(modem);
	  return -EBUSY;
	}	

	gpio_direction_output(modem->gpio_wakemodem,0);//wake
	
#endif
	if(gpio_is_valid(modem->gpio_ap_sleep_status)){
		 if (gpio_request(modem->gpio_ap_sleep_status, "ap_sleep_status")) {
			  pr_warning("request modem ap_sleep_status GPIO failed\n");
			  kfree(modem);
			  return -EBUSY;
		}
		gpio_direction_output(modem->gpio_ap_sleep_status,AP_SLEEP_STATUS_ALIVE);//sleep status alive
	}
	if(gpio_is_valid(modem->gpio_cp_sleep_status)){
		 if (gpio_request(modem->gpio_cp_sleep_status, "cp_sleep_status")) {
			  pr_warning("request modem cp_sleep_status GPIO failed\n");
			  kfree(modem);
			  return -EBUSY;
		}
		gpio_direction_input(modem->gpio_cp_sleep_status);
	}
	if(gpio_is_valid(modem->gpio_cp_ptt)){
		 if (gpio_request(modem->gpio_cp_ptt, "cp_ptt")) {
			  pr_warning("request modem cp_ptt GPIO failed\n");
			  kfree(modem);
			  return -EBUSY;
		}
		gpio_direction_output(modem->gpio_cp_ptt,1);
	}

	INIT_DELAYED_WORK(&modem->reset_work.work,modem_reset_work);
	INIT_DELAYED_WORK(&modem->enable_work.work,modem_enable_work);
	INIT_DELAYED_WORK(&modem->command_work.work,modem_command_work);

	//set work callback
	modem->enable_work.work_callback= modem_enable_callback;
	modem->reset_work.work_callback = modem_reset_callback;	

	//default is power off
	gpio_direction_output(modem->gpio_onoff,1);
	gpio_direction_output(modem->gpio_power,1);//power on

	
	/*first time init ,we use reset callback to initialise enable work*/
	//reset start
	gpio_direction_output(modem->gpio_reset,0);
	schedule_delayed_work(&modem->reset_work.work, msecs_to_jiffies(100));

	/*usb modem device for user space access*/
	usb_modem_device.dev.platform_data = modem_pdata;	
	platform_driver_register(&usb_modem_driver);
	platform_device_register(&usb_modem_device);

	return 0;
	
}


/* 
 * Set modem as late init to ensure usb subsystem already initialized 
 *
 */
#ifndef MODEM_INIT_EXCLUDE
late_initcall(board_modem_init);
#endif
