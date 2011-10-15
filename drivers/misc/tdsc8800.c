#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/circ_buf.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <mach/iomux.h>
#include <mach/gpio.h>
//#include <mach/spi_fpga.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/wait.h>
//#include <linux/android_power.h>
//#include <asm/arch/gpio_extend.h>
#include <linux/workqueue.h>
#include <linux/mtk23d.h>
#include <linux/wakelock.h>
#include "../mtd/rknand/api_flash.h"

MODULE_LICENSE("GPL");

#define DEBUG
#ifdef DEBUG
#define MODEMDBG(x...) printk(x)
#else
#define MODEMDBG(fmt,argss...)
#endif

#define MTK23D_RESET 0x01
#define MTK23D_POWERON  0x02
#define MTK23D_POWER_HIGH 0x03
#define MTK23D_IMEI_READ  0x04


static bool bpstatus_irq_enable = false;
static bool wakelock_inited;
static struct wake_lock tdsc8800_wakelock;

#define SLEEP 1
#define READY 0
struct rk2818_23d_data *gpdata = NULL;

static int  get_bp_statue(struct platform_device *pdev)
{
	struct rk2818_23d_data *pdata = pdev->dev.platform_data;
	
	if(gpio_get_value(pdata->bp_statue))
		return SLEEP;
	else
		return READY;
}
static void ap_sleep(struct platform_device *pdev)
{
	struct rk2818_23d_data *pdata = pdev->dev.platform_data;
	
	MODEMDBG("ap sleep!\n");
	gpio_set_value(pdata->ap_statue,GPIO_HIGH);
}
static void ap_wakeup(struct platform_device *pdev)
{
	struct rk2818_23d_data *pdata = pdev->dev.platform_data;
	
	MODEMDBG("ap wakeup!\n");
	gpio_set_value(pdata->ap_statue,GPIO_LOW);
}
/* */
static void ap_wakeup_bp(struct platform_device *pdev, int wake)//low to wakeup bp
{
	struct rk2818_23d_data *pdata = pdev->dev.platform_data;
        struct modem_dev *mt6223d_data = platform_get_drvdata(pdev);
	MODEMDBG("ap_wakeup_bp\n");

	gpio_set_value(pdata->ap_bp_wakeup, wake);  // phc
	//gpio_set_value(RK2818_PIN_PF5, wake);
}

static void bpwakeup_work_func_work(struct work_struct *work)
{
	struct modem_dev *bdata = container_of(work, struct modem_dev, work);
	
	MODEMDBG("%s\n", __FUNCTION__);
	
}
/*  */
static irqreturn_t  bpwakeup_work_func(int irq, void *data)
{
	struct modem_dev *mt6223d_data = (struct modem_dev *)data;
	
   	MODEMDBG("bpwakeup_work_func\n");
	schedule_work(&mt6223d_data->work);
	return IRQ_HANDLED;
}
static irqreturn_t  bp_apwakeup_work_func(int irq, void *data)
{
	//struct modem_dev *dev = &mtk23d_misc;
	
   	MODEMDBG("bp_apwakeup_work_func\n");
	//wake_up_interruptible(&dev->wakeup);
	return IRQ_HANDLED;
}

static irqreturn_t BBwakeup_isr(int irq, void *dev_id)
{
	struct rk2818_23d_data *pdata = dev_id;
	
	MODEMDBG("%s \n", __FUNCTION__);
	
	if(bpstatus_irq_enable == true)
	{
		MODEMDBG("tdsc8800_wakelock 3s \n");
		wake_lock_timeout(&tdsc8800_wakelock, 3 * HZ);
	}
		

	return IRQ_HANDLED;
}

int modem_poweron_off(int on_off)
{
	struct rk2818_23d_data *pdata = gpdata;
	int result, error = 0, irq = 0;	
	
	if(on_off)
	{
		printk("tdsc8800_poweron\n");
		gpio_set_value(pdata->bp_power, pdata->bp_power_active_low? GPIO_LOW:GPIO_HIGH);  // power on enable
		mdelay(300);
		gpio_set_value(pdata->bp_reset, pdata->bp_reset_active_low? GPIO_HIGH:GPIO_LOW);  // release reset
		msleep(3000);
		gpio_set_value(pdata->bp_power, pdata->bp_power_active_low? GPIO_HIGH:GPIO_LOW);  // power on relase

	}
	else
	{
		printk("tdsc8800_poweroff\n");
		gpio_set_value(pdata->bp_power, pdata->bp_power_active_low? GPIO_LOW:GPIO_HIGH);
		mdelay(100);
		gpio_set_value(pdata->bp_power, pdata->bp_power_active_low? GPIO_HIGH:GPIO_LOW);
	}
}
static int tdsc8800_open(struct inode *inode, struct file *file)
{
	struct rk2818_23d_data *pdata = gpdata;
	//struct rk2818_23d_data *pdata = gpdata = pdev->dev.platform_data;
	struct platform_data *pdev = container_of(pdata, struct device, platform_data);

	MODEMDBG("tdsc8800_open\n");

	int ret = 0;
	modem_poweron_off(1);
	device_init_wakeup(&pdev, 1);

	return 0;
}

static int tdsc8800_release(struct inode *inode, struct file *file)
{
	MODEMDBG("tdsc8800_release\n");

	//gpio_free(pdata->bp_power);
	return 0;
}

//extern char imei_value[16]; // phc, no find 'imei_value' in rk29 project
//char imei_value[16] = {0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5};

static int tdsc8800_ioctl(struct inode *inode,struct file *file, unsigned int cmd, unsigned long arg)
{
	struct rk2818_23d_data *pdata = gpdata;
	int i;
	void __user *argp = (void __user *)arg;
	
	char SectorBuffer[512];
	
	switch(cmd)
	{
		case MTK23D_RESET:		
			printk("tdsc8800_RESET\n");
			gpio_set_value(pdata->bp_reset, pdata->bp_reset_active_low? GPIO_LOW:GPIO_HIGH);
			mdelay(100);
			gpio_set_value(pdata->bp_power, pdata->bp_power_active_low? GPIO_LOW:GPIO_HIGH);
			mdelay(300);
			gpio_set_value(pdata->bp_reset, pdata->bp_reset_active_low? GPIO_HIGH:GPIO_LOW);
			msleep(3000);
			gpio_set_value(pdata->bp_power, pdata->bp_power_active_low? GPIO_HIGH:GPIO_LOW);
			break;
		case MTK23D_IMEI_READ:
			printk("tdsc8800_IMEI_READ\n");
			
			GetSNSectorInfo(SectorBuffer); // phc,20110624
			
			if(copy_to_user(argp, &(SectorBuffer[451]), 16))  // IMEI后从451偏移开始的16bytes，第一个byte为长度固定为15
			{
				printk("ERROR: copy_to_user---%s\n", __FUNCTION__);
				return -EFAULT;
			}
			//printk("IMEI:%d %d %d %d\n", SectorBuffer[451], SectorBuffer[452], SectorBuffer[453], SectorBuffer[454]);
			break;
		default:
			break;
	}
	return 0;
}

static struct file_operations tdsc8800_fops = {
	.owner = THIS_MODULE,
	.open =tdsc8800_open,
	.release =tdsc8800_release,
	.ioctl = tdsc8800_ioctl
};

static struct miscdevice tdsc8800_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = MODEM_NAME,
	.fops = &tdsc8800_fops
};

static int tdsc8800_probe(struct platform_device *pdev)
{
	struct rk2818_23d_data *pdata = gpdata = pdev->dev.platform_data;
	struct modem_dev *tdsc8800_data = NULL;
	int result, error = 0, irq = 0;	
	
	MODEMDBG("mtk23d_probe\n");

	//pdata->io_init();

	tdsc8800_data = kzalloc(sizeof(struct modem_dev), GFP_KERNEL);
	if(NULL == tdsc8800_data)
	{
		printk("failed to request tdsc8800_data\n");
		goto err6;
	}
	platform_set_drvdata(pdev, tdsc8800_data);


	result = gpio_request(pdata->bp_reset, "tdsc8800");
	if (result) {
		printk("failed to request BP_RESET gpio\n");
		goto err2;
	}		
	result = gpio_request(pdata->bp_power, "tdsc8800");
	if (result) {
		printk("failed to request BP_POW_EN gpio\n");
		goto err1;
	}
	
	
        gpio_direction_output(pdata->bp_power, GPIO_LOW);
	gpio_direction_output(pdata->bp_reset, GPIO_LOW);

	gpio_set_value(pdata->bp_reset, pdata->bp_reset_active_low? GPIO_LOW:GPIO_HIGH);

	INIT_WORK(&tdsc8800_data->work, bpwakeup_work_func_work);
	result = misc_register(&tdsc8800_misc);
	if(result)
	{
		MODEMDBG("misc_register err\n");
	}
	MODEMDBG("mtk23d_probe ok\n");
	
	return result;
err0:
	cancel_work_sync(&tdsc8800_data->work);
	gpio_free(pdata->bp_ap_wakeup);
err1:
	gpio_free(pdata->bp_power);
err2:
	gpio_free(pdata->bp_reset);
err3:
	gpio_free(pdata->ap_bp_wakeup);
err4:
	gpio_free(pdata->ap_statue);
err5:
	gpio_free(pdata->bp_statue);
err6:
	kfree(tdsc8800_data);
ret:
	return result;
}

int tdsc8800_suspend(struct platform_device *pdev)
{
	int irq, error;
	struct rk2818_23d_data *pdata = pdev->dev.platform_data;
	
	MODEMDBG("%s \n", __FUNCTION__);
	
	//enable_irq_wake(irq);
	ap_sleep(pdev);
	ap_wakeup_bp(pdev, 0);

	irq = gpio_to_irq(pdata->bp_statue);
	if (irq < 0) {
		printk("can't get pdata->bp_statue irq \n");
	}
	else
	{
		printk("enable pdata->bp_statue irq_wake!! \n");
		bpstatus_irq_enable = true;
		enable_irq_wake(irq);
	}
	
	return 0;
}

int tdsc8800_resume(struct platform_device *pdev)
{
	struct rk2818_23d_data *pdata = pdev->dev.platform_data;
	int irq = 0;
	
	MODEMDBG("%s \n", __FUNCTION__);
	
	irq = gpio_to_irq(pdata->bp_statue);
	if(irq)
	{
		printk("disable pdata->bp_statue irq_wake!! \n");
		bpstatus_irq_enable = false;
		disable_irq_wake(irq);
	}
	
	ap_wakeup(pdev);
	ap_wakeup_bp(pdev, 1);
	
	return 0;
}

void tdsc8800_shutdown(struct platform_device *pdev, pm_message_t state)
{
	struct rk2818_23d_data *pdata = pdev->dev.platform_data;
	struct modem_dev *mt6223d_data = platform_get_drvdata(pdev);
	
	MODEMDBG("%s \n", __FUNCTION__);

	modem_poweron_off(0);  // power down

	cancel_work_sync(&mt6223d_data->work);
	gpio_free(pdata->bp_ap_wakeup);
	gpio_free(pdata->bp_power);
	gpio_free(pdata->bp_reset);
	gpio_free(pdata->ap_bp_wakeup);
	gpio_free(pdata->ap_statue);
	gpio_free(pdata->bp_statue);
	kfree(mt6223d_data);
}

static struct platform_driver tdsc8800_driver = {
	.probe	= tdsc8800_probe,
	.shutdown	= tdsc8800_shutdown,
	.suspend  	= tdsc8800_suspend,
	.resume		= tdsc8800_resume,
	.driver	= {
		.name	= "tdsc8800",
		.owner	= THIS_MODULE,
	},
};

static int __init tdsc8800_init(void)
{
	MODEMDBG("tdsc8800_init ret=%d\n");
	return platform_driver_register(&tdsc8800_driver);
}

static void __exit tdsc8800_exit(void)
{
	MODEMDBG("tdsc8800_exit\n");
	platform_driver_unregister(&tdsc8800_driver);
}

module_init(tdsc8800_init);
//late_initcall_sync(mtk23d_init);
module_exit(tdsc8800_exit);
