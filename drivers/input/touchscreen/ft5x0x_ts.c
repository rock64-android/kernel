/*
 * Driver for Pixcir I2C touchscreen controllers.
 *
 * Copyright (C) 2010-2011 Pixcir, Inc.
 *
 * ft5x0x_i2c_ts.c V3.0  from v3.0 support TangoC solution and remove the previous soltutions
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <mach/iomux.h>
#include <linux/platform_device.h>

#include <linux/slab.h>
#include <asm/uaccess.h>

#include "ft5x0x_ts.h"

#define FT5X0X_DEBUG			0
#if FT5X0X_DEBUG
	#define ft5x0x_dbg(msg...)	printk(msg);
#else
	#define ft5x0x_dbg(msg...)
#endif

static struct workqueue_struct *ft5x0x_wq;
static struct ft5x0x_i2c_ts_data *this_data;
static struct i2c_client *this_client;

struct point_data{
	unsigned char	brn;  //broken line number
	unsigned char	brn_pre;
	unsigned char	id;    //finger ID
	int	posx;
	int	posy;
};

static struct point_data point[5];



//static struct i2c_driver ft5x0x_i2c_ts_driver;




struct ft5x0x_i2c_ts_data {
	struct i2c_client *client;
	struct input_dev *input,*input_key_dev;
	int use_irq;
	int 	gpio_pendown;
	int 	gpio_reset;
	int 	gpio_reset_active_low;
	int		pendown_iomux_mode;	
	int		resetpin_iomux_mode;
	char	pendown_iomux_name[IOMUX_NAME_SIZE];	
	char	resetpin_iomux_name[IOMUX_NAME_SIZE];		
	struct 	work_struct  work;
	//const struct ft5x0x_ts_platform_data *chip;
	bool exiting;
};


static int finger_id[5];

/***********************************************************************
  [function]: 
callback:                 read touch  data ftom ctpm by i2c interface;
[parameters]:
rxdata[in]:              data buffer which is used to store touch data;
length[in]:              the length of the data buffer;
[return]:
FTS_TRUE:              success;
FTS_FALSE:             fail;
 ************************************************************************/
static int fts_i2c_rxdata(u8 *rxdata, int length)
{
	int ret;
	struct i2c_msg msg;


	msg.addr = this_client->addr;
	msg.flags = 0;
	msg.len = 1;
	msg.buf = rxdata;
	ret = i2c_transfer(this_client->adapter, &msg, 1);

	if (ret < 0)
		pr_err("msg %s i2c write error: %d\n", __func__, ret);

	msg.addr = this_client->addr;
	msg.flags = I2C_M_RD;
	msg.len = length;
	msg.buf = rxdata;
	ret = i2c_transfer(this_client->adapter, &msg, 1);
	if (ret < 0)
		pr_err("msg %s i2c write error: %d\n", __func__, ret);

	return ret;
}



static void ft5x0x_ts_poscheck(struct ft5x0x_i2c_ts_data *data)
{
	struct ft5x0x_i2c_ts_data *tsdata = data;
	
	u8 *p;
	u8 touch, button;
	u8 rdbuf[27], wrbuf[1] = { 0 };
	u16 posx[5],posy[5],i;
	u8 ind;

	u8 buf[32] = {0};
	int ret = -1;

ft5x0x_dbg("=================%s\n", __func__ );

	ret = fts_i2c_rxdata(buf, 31);
    	if (ret < 0) {
		printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
    	}
	
	touch = buf[2] & 0x07;

    	if (touch == 0) {
    		 //   ft5x0x_ts_release();
		ft5x0x_dbg("ft5x0x_ts_release\n");
		input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, 0);
		input_sync(tsdata->input);
       		 return 1; 
   	 }
	ind = touch;
    	switch (touch) 
	{
		case 5:
			ind--;
			point[ind].posx = (s16)(buf[0x1b] & 0x0F)<<8 | (s16)buf[0x1c];
			point[ind].posy = (s16)(buf[0x1d] & 0x0F)<<8 | (s16)buf[0x1e];
			point[ind].id = ind;
		//	printk(" tp_pos_5 =(%d, %d)\n", event->x5, event->y5);
		case 4:
			ind--;
			point[ind].posx = (s16)(buf[0x15] & 0x0F)<<8 | (s16)buf[0x16];
			point[ind].posy = (s16)(buf[0x17] & 0x0F)<<8 | (s16)buf[0x18];
			point[ind].id = ind;
		//	printk(" tp_pos_4 =(%d, %d)\n", event->x4, event->y4);
		case 3:
			ind--;
			point[ind].posx = (s16)(buf[0x0f] & 0x0F)<<8 | (s16)buf[0x10];
			point[ind].posy = (s16)(buf[0x11] & 0x0F)<<8 | (s16)buf[0x12];
			point[ind].id = ind;
		//	printk(" tp_pos_3 =(%d, %d)\n", event->x3, event->y3);
		case 2:
			ind--;
			point[ind].posx = (s16)(buf[9] & 0x0F)<<8 | (s16)buf[10];
			point[ind].posy = (s16)(buf[11] & 0x0F)<<8 | (s16)buf[12];
			point[ind].id = ind;
		//	printk(" tp_pos_2 =(%d, %d)\n", event->x2, event->y2);
		case 1:
			ind--;
			point[ind].posx = (s16)(buf[3] & 0x0F)<<8 | (s16)buf[4];
			point[ind].posy = (s16)(buf[5] & 0x0F)<<8 | (s16)buf[6];
			point[ind].id = ind;
		//	printk(" tp_pos_1 =(%d, %d)\n", event->x1, event->y1);
            break;
		default:
		    return -1;
	}

// huangg
//	if(touch == 1 && (point[0].brn != point[0].brn_pre)) {
//		input_report_key(tsdata->input, BTN_TOUCH, 0);
//		input_sync(tsdata->input);
//	} 
//	else 
// end
	{
		if (touch) 
		{
			input_report_key(tsdata->input, BTN_TOUCH, 1);
			input_report_abs(tsdata->input, ABS_X, point[0].posx);
			input_report_abs(tsdata->input, ABS_Y, point[0].posy);

			for(i=0; i<touch; i++) 
			{
			//huanggq
			//	if(point[i].brn != point[i].brn_pre) {
			//		continue;
			//	} 
			//	else
			// end
				{
				//	input_report_key(tsdata->input, ABS_MT_TRACKING_ID, point[i].id);
					input_report_abs(tsdata->input, ABS_MT_POSITION_X, point[i].posx);
					input_report_abs(tsdata->input, ABS_MT_POSITION_Y, point[i].posy);
					//input_report_key(tsdata->input, ABS_MT_WIDTH_MAJOR, i);
					input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, 1);
					input_mt_sync(tsdata->input);
					ft5x0x_dbg("brn%d=%2d id%d=%1d x=%5d y=%5d \n",
						i,point[i].brn,i,point[i].id,point[i].posx,point[i].posy);
				}			
			}	        //printk("\n");
		} 
		else 
		{
			input_report_key(tsdata->input, BTN_TOUCH, 0);
		}
		input_sync(tsdata->input);
	}
//huanggq
//	for(i=0;i<touch;i++)
//		point[i].brn_pre = point[i].brn;
//end
}




static void ft5x0x_ts_work_func(struct work_struct *work)
{
	struct ft5x0x_i2c_ts_data *tsdata = this_data;
	
	ft5x0x_dbg("%s\n",__FUNCTION__);
	while (!tsdata->exiting) {
		
		ft5x0x_ts_poscheck(tsdata);

		if (attb_read_val()){
			ft5x0x_dbg("%s:irq_pin is 1,irq return\n",__FUNCTION__);
			enable_irq(tsdata->client->irq);
		// after remove, not freq send up_event 
		//	input_report_key(tsdata->input, BTN_TOUCH, 0);
		//	input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, 0);
		//	input_report_key(tsdata->input, ABS_MT_WIDTH_MAJOR,0);
		//	input_sync(tsdata->input);
			break;
		}

		msleep(10);
	}

	return;
}

static irqreturn_t ft5x0x_ts_isr(int irq, void *dev_id)
{
    struct ft5x0x_i2c_ts_data *ts = dev_id;
    ft5x0x_dbg("%s=%d,%d\n",__FUNCTION__,ts->client->irq,ts->use_irq);
	
	if(ts->use_irq){
    	disable_irq_nosync(ts->client->irq);
	}
	queue_work(ft5x0x_wq, &ts->work);
    return IRQ_HANDLED;
}

#ifdef CONFIG_PM_SLEEP
static int ft5x0x_i2c_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	ft5x0x_dbg("%s\n",__FUNCTION__);

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(client->irq);

	return 0;
}

static int ft5x0x_i2c_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	ft5x0x_dbg("%s\n",__FUNCTION__);

	if (device_may_wakeup(&client->dev))
		disable_irq_wake(client->irq);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(ft5x0x_dev_pm_ops,
			 ft5x0x_i2c_ts_suspend, ft5x0x_i2c_ts_resume);


static int __devinit ft5x0x_i2c_ts_probe(struct i2c_client *client,
					 const struct i2c_device_id *id)
{
	//const struct ft5x0x_ts_platform_data *pdata = client->dev.platform_data;
	struct ft5x0x_i2c_ts_data *tsdata;
	struct ft5x0x_platform_data *pdata;
	struct input_dev *input;
	struct device *dev;
	//struct i2c_dev *i2c_dev;
	int error = 0,i;

	//if (!pdata) {
	//	dev_err(&client->dev, "platform data not defined\n");
	//	return -EINVAL;
	//}
	ft5x0x_dbg("%s\n",__FUNCTION__);

	tsdata = kzalloc(sizeof(*tsdata), GFP_KERNEL);
	tsdata->input = input_allocate_device();
	if (!tsdata || !(tsdata->input)) {
		dev_err(&client->dev, "Failed to allocate driver data!\n");
		error = -ENOMEM;
		goto err_free_mem;
	}
	
	ft5x0x_wq = create_singlethread_workqueue("ft5x0x_tp_wq");
	if (!ft5x0x_wq) {
		printk(KERN_ERR"%s: create workqueue failed\n", __FUNCTION__);
		error = -ENOMEM;
		goto err_free_mem;
	}
	INIT_WORK(&tsdata->work, ft5x0x_ts_work_func);

	this_client = client;
	this_data = tsdata;
	tsdata->exiting = false;
	//tsdata->input = input;
	//tsdata->chip = pdata;
	
	tsdata->client = client;
	i2c_set_clientdata(client, tsdata);
	pdata = client->dev.platform_data;


	tsdata->input->phys = "/dev/input/event2";
	tsdata->input->name = "ft5x0x_ts-touchscreen";//client->name;
	tsdata->input->id.bustype = BUS_I2C;
	tsdata->input->dev.parent = &client->dev;
	
	
	///*
	/*set_bit(EV_SYN,    tsdata->input->evbit);
	set_bit(EV_KEY,    tsdata->input->evbit);
	set_bit(EV_ABS,    tsdata->input->evbit);
	set_bit(BTN_TOUCH, tsdata->input->keybit);
	set_bit(BTN_2,     tsdata->input->keybit);//*/
	
	tsdata->input->evbit[0] = BIT_MASK(EV_SYN) |  BIT_MASK(EV_ABS) ;
	//tsdata->input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	/*tsdata->input->absbit[0] = BIT_MASK(ABS_MT_POSITION_X) | BIT_MASK(ABS_MT_POSITION_Y) |
			BIT_MASK(ABS_MT_TOUCH_MAJOR) | BIT_MASK(ABS_MT_WIDTH_MAJOR);  // for android*/
	tsdata->input->keybit[BIT_WORD(BTN_START)] = BIT_MASK(BTN_START);

	//input_set_abs_params(input, ABS_X, 0, X_MAX, 0, 0);
	//input_set_abs_params(input, ABS_Y, 0, Y_MAX, 0, 0);
	input_set_abs_params(tsdata->input, ABS_MT_POSITION_X,  pdata->x_min,  pdata->x_max, 0, 0);
	input_set_abs_params(tsdata->input, ABS_MT_POSITION_Y,  pdata->y_min, pdata->y_max, 0, 0);
	input_set_abs_params(tsdata->input, ABS_MT_WIDTH_MAJOR, 0,   16, 0, 0);
	input_set_abs_params(tsdata->input, ABS_MT_TOUCH_MAJOR, 0,    1, 0, 0);
	input_set_abs_params(tsdata->input, ABS_MT_TRACKING_ID, 0,    5, 0, 0);
	input_set_drvdata(tsdata->input, tsdata);
	//init int and reset ports
	error = gpio_request(client->irq, "TS_INT");	//Request IO
	if (error){
		dev_err(&client->dev, "Failed to request GPIO:%d, ERRNO:%d\n",(int)client->irq, error);
		goto err_free_mem;
	}
	rk29_mux_api_set(pdata->pendown_iomux_name, pdata->pendown_iomux_mode);
	
	gpio_direction_input(client->irq);
	 gpio_set_value(client->irq,GPIO_HIGH);
	gpio_pull_updown(client->irq, 1);

	error = gpio_request(pdata->gpio_reset, "ft5x0x_resetPin");
	if(error){
		dev_err(&client->dev, "failed to request resetPin GPIO%d\n", pdata->gpio_reset);
		goto err_free_mem;
	}
	rk29_mux_api_set(pdata->resetpin_iomux_name, pdata->resetpin_iomux_mode);

	// huanggq
	/*
    gpio_pull_updown(pdata->gpio_reset, 1);
    mdelay(20);
    gpio_direction_output(pdata->gpio_reset, 0);
    gpio_set_value(pdata->gpio_reset,GPIO_HIGH);//GPIO_LOW
    mdelay(100);
    gpio_set_value(pdata->gpio_reset,GPIO_LOW);//GPIO_HIGH
    mdelay(120);
    // gpio_direction_input(pdata->gpio_reset);
    */
    msleep(5);
    gpio_direction_output(pdata->gpio_reset, 0);
    msleep(5);
    gpio_set_value(pdata->gpio_reset,GPIO_LOW);
    msleep(200);
    gpio_set_value(pdata->gpio_reset,GPIO_HIGH);
	// end
	printk("pdata->gpio_reset = %d\n",gpio_get_value(pdata->gpio_reset));
	printk("pdata->gpio_pendown = %d\n",gpio_get_value(client->irq));

	
	client->irq = gpio_to_irq(client->irq);	
  	error = request_irq(client->irq, ft5x0x_ts_isr, IRQF_TRIGGER_FALLING, client->name, (void *)tsdata);
	if (error)
		dev_err(&client->dev, "request_irq failed\n");

	tsdata->use_irq = 1;
	if (error) {
		dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
		tsdata->use_irq = 0;
		goto err_free_mem;
	}

	error = input_register_device(tsdata->input);
	if (error)
		goto err_free_irq;

	i2c_set_clientdata(client, tsdata);
	device_init_wakeup(&client->dev, 1);

	dev_err(&tsdata->client->dev, "insmod successfully!\n");

	return 0;

err_free_irq:
	free_irq(client->irq, tsdata);
err_free_mem:
	input_free_device(tsdata->input);
	kfree(tsdata);
	return error;
}

static int __devexit ft5x0x_i2c_ts_remove(struct i2c_client *client)
{
	int error;
	//struct i2c_dev *i2c_dev;
	struct ft5x0x_i2c_ts_data *tsdata = i2c_get_clientdata(client);

	device_init_wakeup(&client->dev, 0);

	tsdata->exiting = true;
	mb();
	free_irq(client->irq, tsdata);

	input_unregister_device(tsdata->input);
	kfree(tsdata);

	return 0;
}


static const struct i2c_device_id ft5x0x_i2c_ts_id[] = {
	{ "ft5x0x_ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ft5x0x_i2c_ts_id);

static struct i2c_driver ft5x0x_i2c_ts_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "ft5x0x_ts",
		.pm	= &ft5x0x_dev_pm_ops,
	},
	.probe		= ft5x0x_i2c_ts_probe,
	.remove		= __devexit_p(ft5x0x_i2c_ts_remove),
	.id_table	= ft5x0x_i2c_ts_id,
};

static int __init ft5x0x_i2c_ts_init(void)
{
	int ret;

	ft5x0x_dbg("%s\n",__FUNCTION__);

	return i2c_add_driver(&ft5x0x_i2c_ts_driver);
}
module_init(ft5x0x_i2c_ts_init);

static void __exit ft5x0x_i2c_ts_exit(void)
{
	i2c_del_driver(&ft5x0x_i2c_ts_driver);
    if (ft5x0x_wq)
        destroy_workqueue(ft5x0x_wq);
}
module_exit(ft5x0x_i2c_ts_exit);

MODULE_AUTHOR("Jianchun Bian <jcbian@ft5x0x.com.cn>");
MODULE_DESCRIPTION("Ft5x0x I2C Touchscreen Driver");
MODULE_LICENSE("GPL")
