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
#include <asm/gpio.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/mi900.h>

MODULE_LICENSE("GPL");

#define DEBUG
#ifdef DEBUG
#define MODEMDBG(x...) printk(x)
#else
#define MODEMDBG(fmt,argss...)
#endif
#define SLEEP 1
#define READY 0
static struct wake_lock modem_wakelock;
#define IRQ_BB_WAKEUP_AP_TRIGGER    IRQF_TRIGGER_FALLING
//#define IRQ_BB_WAKEUP_AP_TRIGGER    IRQF_TRIGGER_RISING
#define airplane_mode RK29_PIN6_PC1
#define MI900_RESET 0x01
#define AIRPLANE_MODE_OFF 0x03
#define AIRPLANE_MODE_ON 0x00
struct rk29_mi900_data *gpdata = NULL;
static int do_wakeup_irq = 0;

static void uart_ctl(void);

static void ap_wakeup_bp(struct platform_device *pdev, int wake)
{
	struct rk29_mi900_data *pdata = pdev->dev.platform_data;
	gpio_set_value(pdata->ap_wakeup_bp, wake);  

}
extern void rk28_send_wakeup_key(void);

static void do_wakeup(struct work_struct *work)
{
    //MODEMDBG("%s[%d]: %s\n", __FILE__, __LINE__, __FUNCTION__);
    rk28_send_wakeup_key();
}

static DECLARE_DELAYED_WORK(wakeup_work, do_wakeup);
static irqreturn_t detect_irq_handler(int irq, void *dev_id)
{
    if(do_wakeup_irq)
    {
        do_wakeup_irq = 0;
        //MODEMDBG("%s[%d]: %s\n", __FILE__, __LINE__, __FUNCTION__);
        wake_lock_timeout(&modem_wakelock, 10 * HZ);
        schedule_delayed_work(&wakeup_work, HZ / 10);
    }
    return IRQ_HANDLED;
}
int modem_poweron_off(int on_off)
{
	struct rk29_mi900_data *pdata = gpdata;		
  if(on_off)
  {
		MODEMDBG("------------modem_poweron\n");
		gpio_set_value(pdata->bp_power, GPIO_HIGH);
		msleep(700);		
		//msleep(1000);
		//msleep(1000);
		gpio_set_value(pdata->ap_wakeup_bp, GPIO_LOW);
		gpio_set_value(airplane_mode, GPIO_HIGH);
		msleep(700);
		
  }
  else
  {
		MODEMDBG("------------modem_poweroff\n");
		
		#if 0
		
		#else
		//mdelay(2500);
		#endif
  }
  return 0;
}
static int mi900_open(struct inode *inode, struct file *file)
{
	//MODEMDBG("-------------%s\n",__FUNCTION__);
	struct rk29_mi900_data *pdata = gpdata;
	struct platform_data *pdev = container_of(pdata, struct device, platform_data);
	device_init_wakeup(&pdev, 1);
	return 0;
}

static int mi900_release(struct inode *inode, struct file *file)
{
	//MODEMDBG("-------------%s\n",__FUNCTION__);
	return 0;
}

static int mi900_ioctl(struct inode *inode,struct file *file, unsigned int cmd, unsigned long arg)
{
	struct rk29_mi900_data *pdata = gpdata;
	MODEMDBG("-------------%s\n",__FUNCTION__);
	switch(cmd)
	{
		case MI900_RESET:	
			gpio_direction_output(pdata->bp_reset,GPIO_LOW);
			//msleep(1000);
			msleep(700);
			gpio_set_value(pdata->ap_wakeup_bp, GPIO_LOW);
			gpio_set_value(airplane_mode, GPIO_HIGH);
			break;
		case AIRPLANE_MODE_ON:
			gpio_set_value(airplane_mode, GPIO_LOW);
			break;
		case AIRPLANE_MODE_OFF:
			gpio_set_value(airplane_mode, GPIO_HIGH);
			break;
		default:
			break;
	}
	return 0;
}

static struct file_operations mi900_fops = {
	.owner = THIS_MODULE,
	.open = mi900_open,
	.release = mi900_release,
	.ioctl = mi900_ioctl
};

static struct miscdevice mi900_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = MODEM_NAME,
	.fops = &mi900_fops
};

static void uart_ctl(void)
{
	#if 0
	rk29_mux_api_set(GPIO1B7_UART0SOUT_NAME, GPIO1L_GPIO1B7); 			
	gpio_request(RK29_PIN1_PB7, NULL);
	gpio_direction_output(RK29_PIN1_PB7,GPIO_LOW);
	gpio_pull_updown(RK29_PIN1_PB7, PullDisable);  // 下拉禁止

	rk29_mux_api_set(GPIO1B6_UART0SIN_NAME, GPIO1L_GPIO1B6); 		
	gpio_request(RK29_PIN1_PB6, NULL);
	gpio_direction_output(RK29_PIN1_PB6,GPIO_LOW);	
	gpio_pull_updown(RK29_PIN1_PB6, PullDisable);  // 下拉禁止

	rk29_mux_api_set(GPIO1C1_UART0RTSN_SDMMC1WRITEPRT_NAME, GPIO1H_GPIO1C1); 			
	gpio_request(RK29_PIN1_PC1, NULL);
	gpio_direction_output(RK29_PIN1_PC1,GPIO_LOW);

	rk29_mux_api_set(GPIO1C0_UART0CTSN_SDMMC1DETECTN_NAME, GPIO1H_GPIO1C0); 		
	gpio_request(RK29_PIN1_PC0, NULL);
	gpio_direction_output(RK29_PIN1_PC0,GPIO_LOW);
	#else
	rk29_mux_api_set(GPIO1B7_UART0SOUT_NAME, GPIO1L_GPIO1B7); 			
	gpio_request(RK29_PIN1_PB7, NULL);
	gpio_direction_input(RK29_PIN1_PB7);

	rk29_mux_api_set(GPIO1B6_UART0SIN_NAME, GPIO1L_GPIO1B6); 		
	gpio_request(RK29_PIN1_PB6, NULL);
	gpio_direction_input(RK29_PIN1_PB6);	

	rk29_mux_api_set(GPIO1C1_UART0RTSN_SDMMC1WRITEPRT_NAME, GPIO1H_GPIO1C1); 			
	gpio_request(RK29_PIN1_PC1, NULL);
	gpio_direction_input(RK29_PIN1_PC1);

	rk29_mux_api_set(GPIO1C0_UART0CTSN_SDMMC1DETECTN_NAME, GPIO1H_GPIO1C0); 		
	gpio_request(RK29_PIN1_PC0, NULL);
	gpio_direction_input(RK29_PIN1_PC0);
	#endif
}

static int mi900_probe(struct platform_device *pdev)
{
	struct rk29_mi900_data *pdata = gpdata = pdev->dev.platform_data;
	struct modem_dev *mi900_data = NULL;
	int result, irq = 0;	
	//MODEMDBG("-------------%s\n",__FUNCTION__);
	gpio_request(pdata->bp_reset,"bp_reset");
	uart_ctl();
	modem_poweron_off(1);
	mi900_data = kzalloc(sizeof(struct modem_dev), GFP_KERNEL);
	if(mi900_data == NULL)
	{
		printk("failed to request mi900_data\n");
		goto err2;
	}
	platform_set_drvdata(pdev, mi900_data);		
	
	result = gpio_request(pdata->ap_wakeup_bp, "mi900");
	if (result) {
		printk("failed to request AP_BP_WAKEUP gpio\n");
		goto err1;
	}	

	wake_lock_init(&modem_wakelock, WAKE_LOCK_SUSPEND, "bp_wakeup_ap");

	result = gpio_request(pdata->bp_wakeup_ap, "bp_wakeup_ap");
	if (result < 0) {
		printk("%s: gpio_request(%d) failed\n", __func__, pdata->bp_wakeup_ap);
	}
	gpio_direction_input(pdata->bp_wakeup_ap);
    gpio_pull_updown(pdata->bp_wakeup_ap, 1);	
    irq	= gpio_to_irq(pdata->bp_wakeup_ap);
	if(irq < 0)
	{
		gpio_free(pdata->bp_wakeup_ap);
		printk("failed to request bp_wakeup_ap\n");
	}
	result = request_irq(irq, detect_irq_handler, IRQ_BB_WAKEUP_AP_TRIGGER, "bp_wakeup_ap", NULL);
	if (result < 0) {
		printk("%s: request_irq(%d) failed\n", __func__, irq);
		gpio_free(irq);
		goto err0;
	}
	enable_irq_wake(gpio_to_irq(pdata->bp_wakeup_ap)); 

	result = misc_register(&mi900_misc);
	if(result)
	{
		printk("misc_register err\n");
	}	
	return result;
err0:
	cancel_work_sync(&mi900_data->work);
	gpio_free(pdata->bp_wakeup_ap);
err1:
	gpio_free(pdata->ap_wakeup_bp);
err2:
	kfree(mi900_data);
	return 0;
}

int mi900_suspend(struct platform_device *pdev)
{
	do_wakeup_irq = 1;
	//MODEMDBG("-------------%s\n",__FUNCTION__);
	ap_wakeup_bp(pdev, 1);
	rk29_mux_api_set(GPIO1C1_UART0RTSN_SDMMC1WRITEPRT_NAME, GPIO1H_GPIO1C1);
	return 0;
}

int mi900_resume(struct platform_device *pdev)
{
	//MODEMDBG("-------------%s\n",__FUNCTION__);
	ap_wakeup_bp(pdev, 0);
	rk29_mux_api_set(GPIO1C1_UART0RTSN_SDMMC1WRITEPRT_NAME, GPIO1H_UART0_RTS_N);
	return 0;
}

void mi900_shutdown(struct platform_device *pdev, pm_message_t state)
{
	struct rk29_mi900_data *pdata = pdev->dev.platform_data;
	struct modem_dev *mi900_data = platform_get_drvdata(pdev);
	
	//MODEMDBG("-------------%s\n",__FUNCTION__);
	modem_poweron_off(0);
	cancel_work_sync(&mi900_data->work);
	gpio_free(pdata->bp_power);
	gpio_free(pdata->bp_reset);
	gpio_free(pdata->ap_wakeup_bp);
	gpio_free(pdata->bp_wakeup_ap);
	kfree(mi900_data);
}

static struct platform_driver mi900_driver = {
	.probe	= mi900_probe,
	.shutdown	= mi900_shutdown,
	.suspend  	= mi900_suspend,
	.resume		= mi900_resume,
	.driver	= {
		.name	= "mi900",
		.owner	= THIS_MODULE,
	},
};

static int __init mi900_init(void)
{
	//MODEMDBG("-------------%s\n",__FUNCTION__);
	return platform_driver_register(&mi900_driver);
}

static void __exit mi900_exit(void)
{
	//MODEMDBG("-------------%s\n",__FUNCTION__);
	platform_driver_unregister(&mi900_driver);
}

module_init(mi900_init);

module_exit(mi900_exit);
