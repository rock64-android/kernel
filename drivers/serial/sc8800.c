
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/spi/spi.h>
#include <linux/freezer.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <mach/iomux.h>
#include <linux/kthread.h>
#include <linux/dma-mapping.h>

#include "sc8800.h"


#define BP_PACKET_SIZE		64
#define BP_PACKET_HEAD_LEN  16
#define BP_PACKET_DATA_LEN    48
#define BP_PACKET_HEADER_TAG	0x7e7f
#define  BP_PACKET_HEADER_TYPE	0xaa55
#define BP_MAX						 (8192-128)
#define BP_CIR_BUF_MAX		 ((8192-128)*10)

struct sc8800_tx{
	char *rx_add;
	char *rx_get;
	int rx_count;
};
struct sc8800_port {
	struct uart_port port;
	struct spi_device *spi;

	int cts;	        /* last CTS received for flow ctrl */
	int tx_empty;		/* last TX empty bit */

	spinlock_t conf_lock;	/* shared data */
	int conf;		/* configuration for the SC88000
				 * (bits 0-7, bits 8-11 are irqs) */

	int rx_enabled;	        /* if we should rx chars */

	int irq;		/* irq assigned to the sc8800 */

	int minor;		/* minor number */
	int loopback;		/* 1 if we are in loopback mode */

	/* for handling irqs: need workqueue since we do spi_sync */
	struct workqueue_struct *workqueue;
	struct work_struct rx_work;
	/* set to 1 to make the workhandler exit as soon as possible */
	int  force_end_work;
	/* need to know we are suspending to avoid deadlock on workqueue */
	int suspending;

	/* hook for suspending SC8800 via dedicated pin */
	void (*sc8800_hw_suspend) (int suspend);

	/* poll time (in ms) for ctrl lines */
	int poll_time;
	/* and its timer */
	struct timer_list	timer;

	/*signal define, always gpio*/
	int slav_rts;
	int slav_rdy;
	int master_rts;
	int master_rdy;
	wait_queue_head_t spi_read_wq;
	wait_queue_head_t spi_write_wq;
	wait_queue_head_t tx_wq;
	char rx_buf[BP_MAX];
	char tx_buf[BP_MAX];
	char buf[BP_MAX];
	char w_spi_buf[BP_MAX];

	char *uart_buf;
	int  get_index;
	int set_index;

	int rx_flag;
	struct wake_lock        wake;
	spinlock_t				work_lock;


	struct task_struct *tx_thread;
	int thread_wakeup_needed;
	int need_wait;
	struct completion tx_done;
	spinlock_t		tx_lock;

	struct sc8800_tx  tx_ctl;

}
;
enum {
	DATA_INT = 0,
	DATA_RXING,
	DATA_TXING,
	DATA_IDLE
};

enum {
	ERR_NONE = 0,
	ERR_HEAD,
	ERR_TAIL,
	ERR_DATA,
	ERR_TYPE,
	ERR_CRC,
	ERR_LEN,
	ERR_STATE,
	ERR_UNKNOWN
};

#define SC8800_MAJOR 205
#define SC8800_MINOR 210
#define MAX_SC8800 1

void * g_dma_buffer = NULL;
dma_addr_t g_dma_addr;


#if 1
#define sc8800_dbg(x...) printk(x)
#else
#define sc8800_dbg(x...)
#endif

static struct sc8800_port *sc8800s[MAX_SC8800]; /* the chips */
static DEFINE_MUTEX(sc8800s_lock);		   /* race on probe */

static DEFINE_MUTEX(rx_count_lock);

static void rx_ctl_int(struct sc8800_port *s)
{
		memset(s->uart_buf,0,BP_CIR_BUF_MAX);
		s->tx_ctl.rx_add = s->uart_buf;
		s->tx_ctl.rx_get = s->uart_buf;
		s->tx_ctl.rx_count = 0;
}

static int rx_ctl_add(struct sc8800_port *s,char *buf,int len)
{	
	int add;
	int get;
	int total_len;
	int f_len,n_len,i,temp;

	for(i = 2; i < len; i += 2)
	{
		temp = buf[i];
		buf[i] = buf[i+1];
		buf[i+1] = temp;
	}
	
	total_len = len+2;
	add = s->tx_ctl.rx_add-s->uart_buf;
	get = s->tx_ctl.rx_get-s->uart_buf;
	if(get > add)
	{
		if((get-add) < total_len)
		{
			printk("rx_ctl_add_1******have no buff to get get = %d,add = %d,total_len = %d\n",get,add,total_len);
			//while(1);
		}
		else
		{
			memcpy(s->tx_ctl.rx_add,buf,total_len);
			s->tx_ctl.rx_add += total_len;	
		}
	}
	else
	{
		if((BP_CIR_BUF_MAX -add+get) < total_len)
		{
			printk("rx_ctl_add_2******have no buff to get get = %d,add = %d,total_len = %d\n",get,add,total_len);
			//while(1);
		}
		f_len =BP_CIR_BUF_MAX - add;
		if(f_len >= total_len)
		{
			memcpy(s->tx_ctl.rx_add, buf, total_len);
			s->tx_ctl.rx_add += total_len;
		}
		else
		{
			n_len = total_len-f_len;
			memcpy(s->tx_ctl.rx_add, buf, f_len);
			s->tx_ctl.rx_add = s->uart_buf;
			memcpy(s->tx_ctl.rx_add,&buf[f_len],n_len);
			s->tx_ctl.rx_add += n_len;
		}
	}

	mutex_lock(&rx_count_lock);
	s->tx_ctl.rx_count++;
	mutex_unlock(&rx_count_lock);

}

static int rx_ctl_get(struct sc8800_port *s,char *tx_buf)
{	
	char len_low,len_h;
	int get;
	int len,i,j;

	get = s->tx_ctl.rx_get-s->uart_buf;
	len_low = s->tx_ctl.rx_get[0];
	if(get == (BP_CIR_BUF_MAX-1))
	{
		s->tx_ctl.rx_get = s->uart_buf;
		len_h = s->tx_ctl.rx_get[0];
		len = (int)(len_low | (len_h << 8));
		s->tx_ctl.rx_get +=1;
		memcpy(tx_buf,s->tx_ctl.rx_get,len);
		s->tx_ctl.rx_get += len;
	}
	else
	{
		len_h = s->tx_ctl.rx_get[1];
		len = (int)(len_low | (len_h << 8));
		if(len != 0)
			s->tx_ctl.rx_get += 2;
		get = s->tx_ctl.rx_get-s->uart_buf;
		i = BP_CIR_BUF_MAX -get;
		if(i >= len)
		{
			memcpy(tx_buf,s->tx_ctl.rx_get,len);
			s->tx_ctl.rx_get += len;
		}	
		else
		{
			memcpy(tx_buf,s->tx_ctl.rx_get,i);
			s->tx_ctl.rx_get = s->uart_buf;
			memcpy(&tx_buf[i],s->tx_ctl.rx_get,len-i);
			s->tx_ctl.rx_get +=(len-i);
		}
	}
	if((s->tx_ctl.rx_get - s->uart_buf) >=  BP_CIR_BUF_MAX)
		s->tx_ctl.rx_get = s->uart_buf;

	mutex_lock(&rx_count_lock);
	s->tx_ctl.rx_count--;
	mutex_unlock(&rx_count_lock);

	return len;
}

static int ap_wakeup(struct sc8800_port *s)
{
	}
static int bp_rts(struct sc8800_port *s)
{
	return gpio_get_value(s->slav_rts);
}

static int bp_rdy(struct sc8800_port *s)
{
	return gpio_get_value(s->slav_rdy);
}

static void ap_rts(struct sc8800_port *s, int value)
{
	gpio_set_value(s->master_rts, value);
}

static void wakeup_thread(struct sc8800_port *s)
{
	/* Tell the main thread that something has happened */
	s->thread_wakeup_needed = 1;
	if (s->tx_thread)
		wake_up_process(s->tx_thread);
}
static int sleep_thread(struct sc8800_port *s)
{
	int	rc = 0;
	/* Wait until a signal arrives or we are woken up */
	for (;;) {
		try_to_freeze();
		set_current_state(TASK_INTERRUPTIBLE);
		if (signal_pending(current)) {
			rc = -EINTR;
			break;
		}
		if (s->thread_wakeup_needed)
			break;
		schedule();
	}
	__set_current_state(TASK_RUNNING);
	return rc;
}
static void ap_rdy(struct sc8800_port *s, int value)
{
	gpio_set_value(s->master_rdy, value);
}
static int spi_write_bp(struct sc8800_port *s, u8 *buf,u32 data_len)
{
	struct spi_message message;
	int status;
	struct spi_transfer tran;
	int i,j;
	int temp;

#if 1	
 	tran.tx_buf = (void *)(buf);
	tran.rx_buf = (void *)(s->buf);
	tran.len = data_len;
	tran.speed_hz = 0;
	//tran.speed_hz = 32*1000*1000;
	tran.bits_per_word = 16;
	spi_message_init(&message);
	spi_message_add_tail(&tran, &message);
	status = spi_sync(s->spi, &message);
	if (status) {
	      printk( "error while calling spi_sync\n");
	      return -EIO;
	}      
#else
{
	char *tx_buf,*rx_buf;
	dma_addr_t tx_dma,rx_dma;
	
	
	tx_buf = dma_alloc_coherent(NULL, data_len, &tx_dma, GFP_KERNEL | GFP_DMA);
	memcpy(tx_buf,buf,data_len);
	tran.tx_dma = tx_dma;
	tran.tx_buf = tx_buf;
	tran.rx_buf = (void *)(s->buf);
	tran.len = data_len;
	tran.speed_hz = 0;
	tran.bits_per_word = 16;
	spi_message_init(&message);
	message.is_dma_mapped = 1;
	spi_message_add_tail(&tran, &message);
	status = spi_sync(s->spi, &message);
	if (status) {
	      printk( "error while calling spi_sync\n");
	      return -EIO;
	}   

	dma_free_coherent(NULL,data_len,tx_buf,tx_dma);
}
#endif
	return 0;
	
}  
 static int sc8800_data_packet(struct sc8800_port *s, u8 *buf, int len)
{       
	struct bp_head *packet = NULL;
	int send_len = 0;
	u8 *send_buf;
	int ret = 0;
	int data_len = 0;

	if(len > BP_PACKET_DATA_LEN)
		data_len = (((len -BP_PACKET_DATA_LEN-1)/BP_PACKET_SIZE)+2)*BP_PACKET_SIZE;
	else
		data_len = BP_PACKET_SIZE;
	memset(s->tx_buf,0,data_len);
	s->tx_buf[0] = (char)(data_len & 0xFF);
	s->tx_buf[1] = (char)((data_len & 0xFF00) >> 8);
	
	//packet =(struct bp_head *)kmalloc(data_len, GFP_KERNEL);
	//memset(packet, 0, data_len);
	packet = (struct bp_head *)&(s->tx_buf[2]);
	packet->tag =0x7e7f;
	packet->type = 0xaa55;
	packet->length = len;
	packet->fram_num = 0;
	packet->reserved = 0;              
	memcpy((u8 *) packet->data, (u8 *) buf, len);
	/*{
		int i;
		printk("add to buffer tx\n");
		for(i=0;i<len;i++)
			printk("0x%02x,",packet->data[i]);
		printk("\n");
	}*/
	//sc8800_dbg("add to buffer tx:%s\n",packet->data);
	//memcpy((u8 *)&(s->tx_buf[2]), (u8 *)packet, data_len);

	rx_ctl_add(s,s->tx_buf,data_len);
	return ret;
 }
	
static int spi_read_bp(struct sc8800_port *s, char *buf, int len)
{
	struct spi_message message;
	int status;
	struct spi_transfer tran;
	int i,temp;
	
	tran.rx_buf = (void *)buf;
	tran.tx_buf = (void *)s->buf;
	tran.len = len;
	tran.speed_hz = 0;
	//tran.speed_hz = 32*1000*1000;
	tran.bits_per_word = 16;
	spi_message_init(&message);
	spi_message_add_tail(&tran, &message);
	status = spi_sync(s->spi, &message);
	if (status) {
		printk("error while calling spi_sync\n");
		return -EIO;
	}
	
	for(i=0;i<len;i+=2)
	{
		temp = buf[i];
		buf[i] = buf[i+1];
		buf[i+1] = temp;
	}
	return 0;
}

static int ap_get_head(struct sc8800_port *s, struct bp_head *packet)
{
	int status;
	u32 data_len = 0;
	int ret = 0;
	int i;
		
	spi_read_bp(s,s->rx_buf,BP_PACKET_SIZE);
	memcpy((char *)(packet),s->rx_buf,BP_PACKET_SIZE);
	//printk("ap_get_head tag= %04x\n",packet->tag);
	//printk("ap_get_head type= %04x\n",packet->type);
	//printk("ap_get_head length= %04x\n",packet->length);
	if ((packet->tag != BP_PACKET_HEADER_TAG) ||
	(packet->type != BP_PACKET_HEADER_TYPE)) {
		printk("sc8800 get head magic_num fail\n");
		ret = -ERR_HEAD;
		goto out;
	}
	out:	
	return ret;
}
//static void sc8800_rx_work(struct work_struct *w);

static void sc8800_dowork(struct sc8800_port *s,int msg)
{	
	int ret=0;
	if (msg == 1)
	{
		ret =schedule_work(&s->rx_work);
	//	sc8800_dbg("hzf___schedule_rx_work ret = %d\n",ret);
	}
}



static void sc8800_tx_work(struct sc8800_port *s)
{
	struct circ_buf *xmit = &s->port.state->xmit;
	int len;

	s->need_wait = 1;

	if (!(uart_circ_empty(xmit))) {
		len = uart_circ_chars_pending(xmit);
		len = (len > BP_MAX) ? BP_MAX : len;
		if(len == 0)  return;
		sc8800_data_packet(s, xmit->buf+xmit->tail, len);
		xmit->tail = (xmit->tail + len) & (UART_XMIT_SIZE - 1);
		s->port.icount.tx += len;
		complete(&s->tx_done);
		mdelay(5);
	}
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&s->port);	
	//wakeup_thread(s);
}	
static void sc8800_rx_work(struct sc8800_port *s)
{
	struct circ_buf *xmit = &s->port.state->xmit;
	u32 len,i,real_len;
	char *buf = NULL;
	unsigned char ch;
	struct bp_head	packet;
	unsigned long flags;
	int msg;

	ap_rdy(s,0);
	if(ap_get_head(s, &packet))
	{
		goto out;
	}
	len = packet.length;
	if(len > BP_PACKET_DATA_LEN)
		real_len =	(((len -BP_PACKET_DATA_LEN-1)/BP_PACKET_SIZE)+2)*BP_PACKET_SIZE;
	else
		real_len = BP_PACKET_SIZE;
	buf = (char *)kmalloc(real_len, GFP_KERNEL);
	if (!buf) {
		printk( "line %d, err while malloc mem\n", __LINE__);
		goto out;
	}
	memset(buf, 0, real_len);
	memcpy(buf,packet.data,len);

	if(len > BP_PACKET_DATA_LEN)
	{
		printk("hzf_spi_read_len more than 48\n");
		spi_read_bp(s, (char *)(buf+BP_PACKET_DATA_LEN), real_len-BP_PACKET_SIZE);	   //read total ????
	}
#if 0
	{
		int i;
		printk("rx data:");
		for(i = 0; i < len; i++) {
			if (i % 16 ==0)
				printk("\n");
			printk(" %#x",buf[i]);
		}
		printk("\n");
	}
#endif
	//sc8800_dbg("rx:%s\n,",buf);
	for (i=0; i<len; i++) {
		ch = buf[i];		
		uart_insert_char(&s->port, 0, 0, ch, TTY_NORMAL);
	}
	tty_flip_buffer_push(s->port.state->port.tty);
	out:
	ap_rdy(s,1);
	if (buf)
	{
		kfree(buf);
	}
}
	
static irqreturn_t sc8800_irq(int irqno, void *dev_id)
{
	struct sc8800_port *s = dev_id;
	int flags;

	sc8800_dbg("%s\n", __func__);
	s->rx_flag = 1;
	//wakeup_thread(s);
	complete(&s->tx_done);
	//wake_up(&s->tx_wq);
	return IRQ_HANDLED;
}

static void sc8800_enable_ms(struct uart_port *port)
{
	struct sc8800_port *s = container_of(port,
					      struct sc8800_port,
					      port);

	if (s->poll_time > 0)
		mod_timer(&s->timer, jiffies);
	sc8800_dbg("%s\n", __func__);
}

static void sc8800_start_tx(struct uart_port *port)
{
	struct sc8800_port *s = container_of(port,
					      struct sc8800_port,
					      port);
	sc8800_tx_work(s);
}

static void sc8800_stop_rx(struct uart_port *port)
{
	struct sc8800_port *s = container_of(port,
					      struct sc8800_port,
					      port);

	sc8800_dbg("%s\n", __func__);

	s->rx_enabled = 0;
	while(s->set_index != s->get_index);
	//sc8800_dowork(s);
}


static void sc8800_stop_tx(struct uart_port *port)
{
	struct sc8800_port *s = container_of(port,
						  struct sc8800_port,
						  port);

	sc8800_dbg("%s\n", __func__);

	s->rx_enabled = 0;
	while(s->set_index != s->get_index);
}

static void sc8800_shutdown(struct uart_port *port)
{
	struct sc8800_port *s = container_of(port,
					      struct sc8800_port,
					      port);

	sc8800_dbg("%s\n", __func__);

	if (s->suspending)
		return;

	s->force_end_work = 1;

	if (s->poll_time > 0)
		del_timer_sync(&s->timer);

	if (s->workqueue) {
		flush_workqueue(s->workqueue);
		destroy_workqueue(s->workqueue);
		s->workqueue = NULL;
	}
	if(s->uart_buf)
	{
		kfree(s->uart_buf);
		s->uart_buf = NULL;
	}
	if (s->irq)
		free_irq(s->irq, s);

	gpio_free(s->master_rdy);
	gpio_free(s->master_rts);
	gpio_free(s->slav_rdy);
	gpio_free(s->slav_rts);

	/* set shutdown mode to save power */
	if (s->sc8800_hw_suspend)
		s->sc8800_hw_suspend(1);
}

unsigned int sc8800_tx_empty(struct uart_port * port)
{
	return 1;
}
static int sc8800_startup(struct uart_port *port)
{
	struct sc8800_port *s = container_of(port,
					      struct sc8800_port,
					      port);
	int ret;
	char b[12];

	sc8800_dbg("%s\n", __func__);
	s->rx_enabled = 1;
	if (s->suspending)
		return 0;
	if (s->sc8800_hw_suspend)
		s->sc8800_hw_suspend(0);
	return ret;

}
static void sc8800_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}
static void sc8800_set_termios(struct uart_port *port, struct ktermios *termios,struct ktermios *old)
{
}
static const char *sc8800_type(struct uart_port *port)
{	
	return (port->type == PORT_RK29) ? "SC8800" : NULL;       
}
static void sc8800_config_port(struct uart_port *port, int flags)
{
         if (flags & UART_CONFIG_TYPE) {
                port->type = PORT_SC8800;

         }
}
static struct uart_ops sc8800_ops = {
	.tx_empty   = sc8800_tx_empty,
	.start_tx	= sc8800_start_tx,
	.stop_tx  = sc8800_stop_tx,
	.stop_rx	= sc8800_stop_rx,
	.startup	= sc8800_startup,
	.shutdown	= sc8800_shutdown,
	.set_mctrl = sc8800_set_mctrl,
	.set_termios = sc8800_set_termios,
	.type = sc8800_type,
	.config_port = sc8800_config_port,
};

static struct uart_driver sc8800_uart_driver = {
	.owner          = THIS_MODULE,
	.driver_name    = "ttySPI",
	.dev_name       = "ttySPI",
	.major          = SC8800_MAJOR,
	.minor          = SC8800_MINOR,
	.nr             = MAX_SC8800,
};

static int uart_driver_registered;
static int tx_thread_function(void *data)
{
	struct sc8800_port *s = (struct sc8800_port *) data;
	int set =0,get = 0;
	int len =0;
	int flags;

	//printk("%s: ==================\n", __func__);
	/* Allow the thread to be frozen */
	//set_freezable();
	/* Arrange for userspace references to be interpreted as kernel
	 * pointers.  That way we can pass a kernel pointer to a routine
	 * that expects a __user pointer and it will work okay. */
	//set_fs(get_ds());

	while (1) {
		set = s->set_index;
		get = s->get_index;
		
rx:		if(s->rx_flag)
		{
			//printk("%s:   rx_flag = 1\n", __func__);
			sc8800_rx_work(s);
			s->rx_flag = 0;
		}
		else if(s->tx_ctl.rx_count)
		{
			//printk("%s:   rx_count >= 1\n", __func__);
			msleep(1);
			while(bp_rts(s) == 0)
			{
				//printk("%s:   bp_rts(s) == 0\n", __func__);
				if(s->rx_flag)
					goto rx;
				msleep(1);
			}
			ap_rts(s,0);
			
			while(bp_rdy(s))
			{
				//printk("%s:   bp_rdy(s)\n", __func__);
				if(s->rx_flag){
					ap_rts(s,1);
					goto rx;
				}
				msleep(1);
			}
			len = rx_ctl_get(s,s->w_spi_buf);
			//printk("tx_thread_function out len = %d\n",len);
			if(len > 0)
			{
				spi_write_bp(s,s->w_spi_buf,len);
			}
			ap_rts(s,1);

			//complete(&s->tx_done);
		}
		else {
			if (s->need_wait) {
				//sleep_thread(s);
				wait_for_completion(&s->tx_done);
				//wait_event(s->tx_wq, s->tx_ctl.rx_count);
				//printk("%s: =========wakeup\n", __func__);
			}
		}
	}
	return 0;
}

static int __devinit sc8800_probe(struct spi_device *spi)
{
	int i, retval,err;
	struct plat_sc8800 *pdata;
	char b[12];
	
	mutex_lock(&sc8800s_lock);
	if (!uart_driver_registered) {
		uart_driver_registered = 1;
		retval = uart_register_driver(&sc8800_uart_driver);
		if (retval) {
			printk(KERN_ERR "Couldn't register sc8800 uart driver\n");
			mutex_unlock(&sc8800s_lock);
			return retval;
		}
	}

	for (i = 0; i < MAX_SC8800; i++)
		if (!sc8800s[i])
			break;
	if (i == MAX_SC8800) {
		printk( "too many SC8800 chips\n");
		mutex_unlock(&sc8800s_lock);
		return -ENOMEM;
	}
	sc8800s[i] = kzalloc(sizeof(struct sc8800_port), GFP_KERNEL);
	if (!sc8800s[i]) {
		printk("kmalloc for sc8800 structure %d failed!\n", i);
		mutex_unlock(&sc8800s_lock);
		return -ENOMEM;
	}

	spi->bits_per_word = 16;
	spi->mode = SPI_MODE_2;
	spi->max_speed_hz = 1000*1000*8;
	err = spi_setup(spi);
	if (err < 0)
		return err;
		
	sc8800s[i]->spi = spi;
	spin_lock_init(&sc8800s[i]->tx_lock);
	spin_lock_init(&sc8800s[i]->work_lock);
	spin_lock_init(&sc8800s[i]->conf_lock);
	dev_set_drvdata(&spi->dev, sc8800s[i]);
	pdata = spi->dev.platform_data;
	sc8800s[i]->irq = gpio_to_irq(pdata->slav_rts_pin);
	sc8800s[i]->slav_rts = pdata->slav_rts_pin;
	sc8800s[i]->slav_rdy = pdata->slav_rdy_pin;
	sc8800s[i]->master_rts = pdata->master_rts_pin;
	sc8800s[i]->master_rdy = pdata->master_rdy_pin;
	//sc8800s[i]->sc8800_hw_suspend = pdata->sc8800_hw_suspend;
	sc8800s[i]->minor = i;
	init_timer(&sc8800s[i]->timer);

	sc8800s[i]->port.irq = sc8800s[i]->irq;
	sc8800s[i]->port.uartclk = 24000000;
	sc8800s[i]->port.fifosize = 16;
	sc8800s[i]->port.ops = &sc8800_ops;
	sc8800s[i]->port.flags = UPF_SKIP_TEST | UPF_BOOT_AUTOCONF;
	sc8800s[i]->port.line = i;
	sc8800s[i]->port.dev = &spi->dev;
	sc8800s[i]->port.type = PORT_SC8800;
	init_waitqueue_head(&sc8800s[i]->spi_write_wq);
	init_waitqueue_head(&sc8800s[i]->spi_read_wq);
	init_waitqueue_head(&sc8800s[i]->tx_wq);
	retval = uart_add_one_port(&sc8800_uart_driver, &sc8800s[i]->port);
	if (retval < 0)
	{
		printk("sc8800_add_one_port fail\n");
	}
	/* set shutdown mode to save power. Will be woken-up on open */
	//if (sc8800s[i]->sc8800_hw_suspend)
	//	sc8800s[i]->sc8800_hw_suspend(1);
	sc8800s[i]->uart_buf = kmalloc(BP_CIR_BUF_MAX, GFP_KERNEL);
	if (!sc8800s[i]->uart_buf) {
		printk("sc8800_get uart buf error\n");
	}

	init_completion(&sc8800s[i]->tx_done);
	sc8800s[i]->tx_thread = kthread_create(tx_thread_function,
						    sc8800s[i], "sc8800 tx thread");
	
	wake_up_process(sc8800s[i]->tx_thread);

	sc8800s[i]->force_end_work = 0;
	sc8800s[i]->rx_flag = 0;
	INIT_WORK(&sc8800s[i]->rx_work, sc8800_rx_work);
	sprintf(b, "sc8800-%d", sc8800s[i]->minor);
	sc8800s[i]->workqueue =  create_singlethread_workqueue("sc8800_workqueue");//create_freezeable_workqueue(b);
	if (!sc8800s[i]->workqueue) {
		printk("cannot create workqueue\n");
		return -EBUSY;
	}
	rx_ctl_int(sc8800s[i]);
	wake_lock_init(&sc8800s[i]->wake, WAKE_LOCK_SUSPEND, "sc8800_wake");

	rk29_mux_api_set(GPIO1C1_UART0RTSN_SDMMC1WRITEPRT_NAME, GPIO1H_GPIO1C1); 			
	rk29_mux_api_set(GPIO1C0_UART0CTSN_SDMMC1DETECTN_NAME, GPIO1H_GPIO1C0); 		

	retval = gpio_request(sc8800s[i]->slav_rts, "slav rts");
	if (retval ){
		printk("line %d: gpio request err\n", __LINE__);
		retval = -EBUSY;
		goto gpio_err1;
	}
	retval = gpio_request(sc8800s[i]->slav_rdy, "slav rdy");
	if (retval) {
		printk("line %d: gpio request err\n", __LINE__);
		retval = -EBUSY;
		goto gpio_err2;
	}
	retval = gpio_request(sc8800s[i]->master_rts, "master rts");
	if (retval) {
		printk("line %d: gpio request err\n", __LINE__);
		retval = -EBUSY;
		goto gpio_err3;
	}
	retval = gpio_request(sc8800s[i]->master_rdy, "master rdy");
	if (retval) {
		printk("line %d: gpio request err\n", __LINE__);
		retval = -EBUSY;
		goto gpio_err4;
	}

	gpio_direction_input(sc8800s[i]->slav_rts);
	gpio_pull_updown(sc8800s[i]->slav_rts, GPIOPullUp);
	gpio_direction_input(sc8800s[i]->slav_rdy);
	gpio_pull_updown(sc8800s[i]->slav_rdy, GPIOPullUp);
	gpio_direction_output(sc8800s[i]->master_rts, GPIO_HIGH);
	gpio_direction_output(sc8800s[i]->master_rdy, GPIO_HIGH);

	if (request_irq(sc8800s[i]->irq, sc8800_irq,
			IRQF_TRIGGER_FALLING, "sc8800", sc8800s[i]) < 0) {
		printk("cannot allocate irq %d\n", sc8800s[i]->irq);
		sc8800s[i]->irq = 0;
		goto irq_err;
	}
	sc8800_dbg("sc8800_probe_out\n");
	mutex_unlock(&sc8800s_lock);
	return 0;
	
irq_err:
	gpio_free(sc8800s[i]->master_rdy);
gpio_err4:
	gpio_free(sc8800s[i]->master_rts);
gpio_err3:
	gpio_free(sc8800s[i]->slav_rdy);
gpio_err2:
	gpio_free(sc8800s[i]->slav_rts);
gpio_err1:
	destroy_workqueue(sc8800s[i]->workqueue);
	sc8800s[i]->workqueue = NULL;	
	mutex_unlock(&sc8800s_lock);
	return retval;


}

static int __devexit sc8800_remove(struct spi_device *spi)
{
	struct sc8800_port *s = dev_get_drvdata(&spi->dev);
	int i;

	mutex_lock(&sc8800s_lock);

	/* find out the index for the chip we are removing */
	for (i = 0; i < MAX_SC8800; i++)
		if (sc8800s[i] == s)
			break;

	printk("%s: removing port %d\n", __func__, i);
	uart_remove_one_port(&sc8800_uart_driver, &sc8800s[i]->port);
	kfree(sc8800s[i]);
	sc8800s[i] = NULL;

	/* check if this is the last chip we have */
	for (i = 0; i < MAX_SC8800; i++)
		if (sc8800s[i]) {
			mutex_unlock(&sc8800s_lock);
			return 0;
		}
	sc8800_dbg("removing sc8800 driver\n");
	uart_unregister_driver(&sc8800_uart_driver);
	
	mutex_unlock(&sc8800s_lock);
	return 0;
}

#ifdef CONFIG_PM

static int sc8800_suspend(struct spi_device *spi, pm_message_t state)
{
	struct sc8800_port *s = dev_get_drvdata(&spi->dev);

	sc8800_dbg( "%s\n", __func__);

	disable_irq(s->irq);

	s->suspending = 1;
	uart_suspend_port(&sc8800_uart_driver, &s->port);

	if (s->sc8800_hw_suspend)
		s->sc8800_hw_suspend(1);

	return 0;
}

static int sc8800_resume(struct spi_device *spi)
{
	struct sc8800_port *s = dev_get_drvdata(&spi->dev);

	sc8800_dbg("%s\n", __func__);

	if (s->sc8800_hw_suspend)
		s->sc8800_hw_suspend(0);
	uart_resume_port(&sc8800_uart_driver, &s->port);
	s->suspending = 0;

	enable_irq(s->irq);

	//if (s->workqueue)
	//	sc8800_dowork(s);

	return 0;
}

#else
#define sc8800_suspend NULL
#define sc8800_resume  NULL
#endif

static struct spi_driver sc8800_driver = {
	.driver = {
		.name		= "sc8800",
		.bus		= &spi_bus_type,
		.owner		= THIS_MODULE,
	},

	.probe		= sc8800_probe,
	.remove		= __devexit_p(sc8800_remove),
	.suspend	= sc8800_suspend,
	.resume		= sc8800_resume,
};

static int __init sc8800_init(void)
{
	return spi_register_driver(&sc8800_driver);
}
module_init(sc8800_init);

static void __exit sc8800_exit(void)
{
	spi_unregister_driver(&sc8800_driver);
}
module_exit(sc8800_exit);

MODULE_DESCRIPTION("SC8800 driver");
MODULE_AUTHOR("liuyixing <lyx@rock-chips.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:SC8800");
