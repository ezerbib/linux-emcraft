/*
 * (C) Copyright 2012
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */


#include <linux/module.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/smp_lock.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/gpio.h>

#include <mach/hardware.h>
#include <mach/irqs.h>

#include <mach/lpc178x.h>
#include <mach/gpio.h>

#define NORMAL_PRIORITY   ( 0x08 )

void lpc178x_gpio_irq_setup(void);
static int gpio_set_irq_type(unsigned int irq, unsigned int type);

/*
 * GPIO register map
 * Should be mapped at (0x20098000 + port * 0x20).
 */
struct lpc178x_gpio_regs {
	u32 fiodir;	/* Fast GPIO Port Direction control register */
	u32 rsv0[3];
	u32 fiomask;	/* Fast Mask register for port */
	u32 fiopin;	/* Fast Port Pin value register using FIOMASK */
	u32 fioset;	/* Fast Port Output Set register using FIOMASK */
	u32 fioclr;	/* Fast Port Output Clear register using FIOMASK */
};

typedef struct
{
  u32 IntStatus;
  u32 IO0IntStatR;
  u32 IO0IntStatF;
  u32 IO0IntClr;
  u32 IO0IntEnR;
  u32 IO0IntEnF;
  u32 RESERVED0[3];
  u32 IO2IntStatR;
  u32 IO2IntStatF;
  u32 IO2IntClr;
  u32 IO2IntEnR;
  u32 IO2IntEnF;
} lpc_gpioint_regs;
/*
 * GPIO registers base
 */
#define LPC178X_GPIO_BASE		(LPC178X_AHB_PERIPH_BASE + 0x00018000)
#define LPC178X_GPIO_PORT_ADDR(port)	(LPC178X_GPIO_BASE + (port) * 0x20)
#define LPC178X_GPIO(port) \
	((volatile struct lpc178x_gpio_regs *)LPC178X_GPIO_PORT_ADDR(port))

#define LPC_APB0_BASE         LPC178X_APB_PERIPH_BASE
#define LPC_GPIOINT_BASE      (LPC_APB0_BASE + 0x28080)
#define LPC_GPIOINT           ((lpc_gpioint_regs   *) LPC_GPIOINT_BASE  )

#define GPIO_BASE(x)		LPC178X_GPIO_PORT_ADDR(x)

/* GPIO registers definition */
//#define GPIO_DIR			0x0
//#define GPIO_DATA_SET		0x18
//#define GPIO_DATA_CLR		0x1C
//#define GPIO_INT_STAT		0x24
//#define GPIO_INT_MASK		0x10
//#define GPIO_INT_CLR		0x1C
//

#define GPIO_PORT_NUM		3

typedef enum IRQn
{
/*...*/
  GPIO_IRQn                     = 38,       /*!< GPIO Interrupt                                   */
} IRQn_Type;
#define GPIO_IRQ_BASE	38

typedef struct
{
	volatile uint32_t P0IntStatR;
	volatile uint32_t P0IntStatF;
	volatile uint32_t P2IntStatR;
	volatile uint32_t P2IntStatF;
} GPIO_INT_REG;

static GPIO_INT_REG IntData = {};

///////////////////////////////////////////////////////////////////////////////////////////////////
#define GPIO_MINOR_A 0
#define GPIO_MINOR_LAST 5

#define LPC178X_GPIO_IOCTYPE 43
/* supported ioctl _IOC_NR's */

#define IO_READBITS  0x1  /* read and return current port bits (obsolete) */
#define IO_SETBITS   0x2  /* set the bits marked by 1 in the argument */
#define IO_CLRBITS   0x3  /* clear the bits marked by 1 in the argument */

/* the alarm is waited for by select() */

#define IO_HIGHALARM 0x4  /* set alarm on high for bits marked by 1 */
#define IO_LOWALARM  0x5  /* set alarm on low for bits marked by 1 */
#define IO_CLRALARM  0x6  /* clear alarm for bits marked by 1 */


/* GPIO direction ioctl's */
#define IO_READDIR    0x8  /* Read direction 0=input 1=output  (obsolete) */
#define IO_SETINPUT   0x9  /* Set direction for bits set, 0=unchanged 1=input,
                              returns mask with current inputs (obsolete) */
#define IO_SETOUTPUT  0xA  /* Set direction for bits set, 0=unchanged 1=output,
                              returns mask with current outputs (obsolete)*/


/* SHUTDOWN ioctl */
#define IO_SHUTDOWN   0xD
#define IO_GET_PWR_BT 0xE

/* Bit toggling in driver settings */
/* bit set in low byte0 is CLK mask (0x00FF),
   bit set in byte1 is DATA mask    (0xFF00)
   msb, data_mask[7:0] , clk_mask[7:0]
 */
#define IO_CFG_WRITE_MODE 0xF
#define IO_CFG_WRITE_MODE_VALUE(msb, data_mask, clk_mask) \
	( (((msb)&1) << 16) | (((data_mask) &0xFF) << 8) | ((clk_mask) & 0xFF) )

/* The following 4 ioctl's take a pointer as argument and handles
 * 32 bit ports (port G) properly.
 * These replaces IO_READBITS,IO_SETINPUT AND IO_SETOUTPUT
 */
#define IO_READ_INBITS   0x10 /* *arg is result of reading the input pins */
#define IO_READ_OUTBITS  0x11 /* *arg is result of reading the output shadow */
#define IO_SETGET_INPUT  0x12 /* bits set in *arg is set to input,
                               * *arg updated with current input pins.
                               */
#define IO_SETGET_OUTPUT 0x13 /* bits set in *arg is set to output,
                               * *arg updated with current output pins.
                               */


static char gpio_name[] = "lpc178x_gpio";
static int gpio_ioctl(struct inode *inode, struct file *file,
	unsigned int cmd, unsigned long arg);
static ssize_t gpio_read(struct file *file, char *buf, size_t len, loff_t *ppos);
static ssize_t gpio_write(struct file *file, const char *buf, size_t count,
	loff_t *off);
static int gpio_open(struct inode *inode, struct file *filp);
static int gpio_release(struct inode *inode, struct file *filp);
static unsigned int gpio_poll(struct file *file, struct poll_table_struct *wait);
/* private data per open() of this driver */

struct gpio_private {
	struct gpio_private *next;
	struct semaphore sem;
	char							*rbufp;		/* read buffer for I/O */
	int								rbufhead;
	int								rbufcnt;
	int								flags;
	/* These fields are generic */
	GPIO_INT_REG 					intalarm;
	wait_queue_head_t alarm_wq;
	int minor;
};

#define	READ_BUFFER_SIZE	0x10000		// 64k
#define	MAX_REQ_PACKET_SIZE	0x10000		// 64k
#define	RETRY_TIMEOUT		(HZ)
#define	MAX_WRITE_RETRY		5

#define LPC178X_GPIO_FLAGS_DEV_OPEN		0x01
#define LPC178X_GPIO_FLAGS_RX_BUSY			0x02
#define LPC178X_GPIO_FLAGS_INTR_BUSY		0x04
#define LPC178X_GPIO_FLAGS_DEV_ERROR		0x08
#define LPC178X_GPIO_FLAGS_PENDING_CLEANUP	0x10
#define LPC178X_GPIO_FLAGS_MASK			0x1f
/* linked list of alarms to check for */

static struct gpio_private *alarmlist=NULL;
static int irq_requested = 0;
static DEFINE_SPINLOCK(alarm_lock);
static DEFINE_SPINLOCK(gpio_lock);

///////////////////////////////////////////////////////////////////////////////////////////////////

irqreturn_t GPIO_IRQHandler(int irq, void *data)
{
	struct gpio_private *priv;
	priv=(struct gpio_private *)data;

	spin_lock(&gpio_lock);
	IntData.P0IntStatR |= LPC_GPIOINT->IO0IntStatR;
	IntData.P0IntStatF |= LPC_GPIOINT->IO0IntStatF;
	IntData.P2IntStatR |= LPC_GPIOINT->IO2IntStatR;
	IntData.P2IntStatF |= LPC_GPIOINT->IO2IntStatF;
	spin_unlock(&gpio_lock);

	//Clear interrupt status only for the relevant pins
	LPC_GPIOINT->IO0IntClr = (IntData.P0IntStatR | IntData.P0IntStatF);
	LPC_GPIOINT->IO2IntClr = (IntData.P2IntStatR | IntData.P2IntStatF);

	/// test EZ ....
//	{
//		char c=*((char*)(0x80000002));
//		pr_err("GPIO_IRQHandler(int irq=%d, void *desc) R:%04X F:%04X CPLD=%02x\n",irq,IntData.P0IntStatR,IntData.P0IntStatF,(int)c);
//	}

	if (priv)
	{
		wake_up_interruptible(&priv->alarm_wq);
	}

	return IRQ_HANDLED;
}


/*
 * Get the current state of a GPIO input pin
 */
static int lpc178x_gpio_get_value(struct gpio_chip *chip, unsigned gpio)
{
	return (LPC178X_GPIO(LPC178X_GPIO_GETPORT(gpio))->fiopin >>
		LPC178X_GPIO_GETPIN(gpio)) & 1;
}

/*
 * Change the direction of a GPIO pin to input
 */
static int lpc178x_gpio_direction_input(struct gpio_chip *chip, unsigned gpio)
{
	LPC178X_GPIO(LPC178X_GPIO_GETPORT(gpio))->fiodir &=
		~(1 << LPC178X_GPIO_GETPIN(gpio));
	gpio_set_irq_type(gpio,IRQ_TYPE_EDGE_BOTH);
	return 0;
}

/*
 * Set the state of a GPIO output pin
 */
static void lpc178x_gpio_set_value(
	struct gpio_chip *chip, unsigned gpio, int value)
{
	if (value) {
		LPC178X_GPIO(LPC178X_GPIO_GETPORT(gpio))->fioset =
			(1 << LPC178X_GPIO_GETPIN(gpio));
	} else {
		LPC178X_GPIO(LPC178X_GPIO_GETPORT(gpio))->fioclr =
			(1 << LPC178X_GPIO_GETPIN(gpio));
	}
}

/*
 * Change the direction of a GPIO pin to output and
 * set the level on this pin.
 */
static int lpc178x_gpio_direction_output(
	struct gpio_chip *chip, unsigned gpio, int level)
{
	LPC178X_GPIO(LPC178X_GPIO_GETPORT(gpio))->fiodir |=
		(1 << LPC178X_GPIO_GETPIN(gpio));

	lpc178x_gpio_set_value(chip, gpio, level);

	return 0;
}


static struct gpio_chip lpc178x_chip = {
	.label			    = "lpc178x",
	.direction_input	= lpc178x_gpio_direction_input,
	.get			    = lpc178x_gpio_get_value,
	.direction_output	= lpc178x_gpio_direction_output,
	.set			    = lpc178x_gpio_set_value,
	.base			    = LPC178X_GPIO_OFF_MCU,
	.ngpio			    = LPC178X_GPIO_LEN_MCU,
	.can_sleep		    = 1,
};

void __init lpc178x_gpio_init(void)
{
	if (gpiochip_add(&lpc178x_chip) < 0)
		pr_err("%s: gpiochip_add failed.\n", __func__);

	lpc178x_gpio_irq_setup();
}

//EZ : only one IRQ
/*

 static int irq_to_gpio(unsigned int irq)
{
	return -EIO;
}

 *
 */
static int gpio_set_irq_type(unsigned int irq, unsigned int type)
{
	unsigned int gpio = irq;//GPIO_IRQn;
	int port;
	int pin;

	//pr_info("EZ: call gpio_set_irq_type(unsigned int irq=%d, unsigned int type=%d)\n",irq,type);
	port=gpio/32;
	pin=gpio%32;
	switch (type) {
		case IRQ_TYPE_EDGE_BOTH:
		{
			if (port==0)
			{
				LPC_GPIOINT->IO0IntEnR|=1<<pin;
				LPC_GPIOINT->IO0IntEnF|=1<<pin;
			}
			if (port==2)
			{
				LPC_GPIOINT->IO2IntEnR|=1<<pin;
				LPC_GPIOINT->IO2IntEnF|=1<<pin;
			}
		}
		break;
		case IRQ_TYPE_EDGE_RISING:
		{
			if (port==0)
			{
				LPC_GPIOINT->IO0IntEnR|=1<<pin;
			}
			if (port==2)
			{
				LPC_GPIOINT->IO2IntEnR|=1<<pin;
			}
		}
		break;
		case IRQ_TYPE_EDGE_FALLING:
		{
			if (port==0)
			{
				LPC_GPIOINT->IO0IntEnF|=1<<pin;
			}
			if (port==2)
			{
				LPC_GPIOINT->IO2IntEnF|=1<<pin;
			}
		}
		break;
	}

	//gpio_ack_irq(irq);
	return 0;
}

#if 0
static int gpio_clear_irq_type(unsigned int irq, unsigned int type)
{
	unsigned int gpio = irq;//GPIO_IRQn;
	int port;
	int pin;

	//pr_info("EZ: call gpio_clear_irq_type(unsigned int irq=%d, unsigned int type=%d)\n",irq,type);
	port=gpio/32;
	pin=gpio%32;
	switch (type) {
		case IRQ_TYPE_EDGE_BOTH:
		{
			if (port==0)
			{
				LPC_GPIOINT->IO0IntEnR &= ~(1<<pin);
				LPC_GPIOINT->IO0IntEnF &= ~(1<<pin);
			}
			if (port==2)
			{
				LPC_GPIOINT->IO2IntEnR &= ~(1<<pin);
				LPC_GPIOINT->IO2IntEnF &= ~(1<<pin);
			}
		}
		break;
		case IRQ_TYPE_EDGE_RISING:
		{
			if (port==0)
			{
				LPC_GPIOINT->IO0IntEnR &= ~(1<<pin);
			}
			if (port==2)
			{
				LPC_GPIOINT->IO2IntEnR &= ~(1<<pin);
			}
		}
		break;
		case IRQ_TYPE_EDGE_FALLING:
		{
			if (port==0)
			{
				LPC_GPIOINT->IO0IntEnF &= ~(1<<pin);
			}
			if (port==2)
			{
				LPC_GPIOINT->IO2IntEnF &= ~(1<<pin);
			}
		}
		break;
	}

	//gpio_ack_irq(irq);
	return 0;
}

static void _set_gpio_irqenable(unsigned int base, unsigned int index,
				int enable)
{
//	unsigned int reg;

//	reg = __raw_readl(base + GPIO_INT_EN);
//	reg = (reg & (~(1 << index))) | (!!enable << index);
//	__raw_writel(reg, base + GPIO_INT_EN);
}

static void gpio_ack_irq(unsigned int irq)
{
//	unsigned int gpio = irq_to_gpio(irq);
//	unsigned int base = GPIO_BASE(gpio / 32);
//
//	__raw_writel(1 << (gpio % 32), base + GPIO_INT_CLR);
	pr_err("EZ: call gpio_ack_irq(unsigned int irq=%d) \n",irq);
}

static void gpio_mask_irq(unsigned int irq)
{
//	unsigned int gpio = irq_to_gpio(irq);
//	unsigned int base = GPIO_BASE(gpio / 32);
//
//	_set_gpio_irqenable(base, gpio % 32, 0);
	pr_err("EZ: call gpio_mask_irq(unsigned int irq=%d) \n",irq);
	//gpio_clear_irq_type(gpio,IRQ_TYPE_EDGE_BOTH);

}

static void gpio_unmask_irq(unsigned int irq)
{
//	unsigned int gpio = irq_to_gpio(irq);
//	unsigned int base = GPIO_BASE(gpio / 32);
//
//	_set_gpio_irqenable(base, gpio % 32, 1);
	pr_err("EZ: call gpio_unmask_irq(unsigned int irq=%d) \n",irq);
	//gpio_set_irq_type(gpio,IRQ_TYPE_EDGE_BOTH);
}
#endif



#if 0
static void lpc178x_gpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	unsigned int gpio_irq_no, irq_stat;
	unsigned int port = (unsigned int)get_irq_data(irq);

	irq_stat = __raw_readl(GPIO_BASE(port) + GPIO_INT_STAT);

	gpio_irq_no = GPIO_IRQ_BASE + port * 32;
	for (; irq_stat != 0; irq_stat >>= 1, gpio_irq_no++) {

		if ((irq_stat & 1) == 0)
			continue;

		BUG_ON(!(irq_desc[gpio_irq_no].handle_irq));
		irq_desc[gpio_irq_no].handle_irq(gpio_irq_no,
				&irq_desc[gpio_irq_no]);
	}
}
#endif

#if 0
static struct irq_chip gpio_irq_chip = {
	.name = "GPIO",
	.ack = gpio_ack_irq,
	.mask = gpio_mask_irq,
	.unmask = gpio_unmask_irq,
	.set_type = gpio_set_irq_type,
};
#endif

int lpc178x_gpio_to_irq(unsigned gpio)
{
	return GPIO_IRQ_BASE;
}

/*
 * Called from the processor-specific init to enable GPIO interrupt support.
 */
/**
 * @description  Install interrupt handler
 *
 * @param  IntNumber    The interrupt number to install
 * @param  HandlerAddr  The interrupt handler address
 * @param  Priority     The interrupt priority
 *
 * @return 0 on success, -1 if IntNumber is out of range
 */
//typedef irqreturn_t (*irq_handler_t)(int, void *);
static int install_irq(IRQn_Type IntNumber,irq_handler_t HandlerAddr,unsigned int Priority)
{
	int ret;

	ret=request_irq(IntNumber,HandlerAddr,0,"LPC Gpio",alarmlist);
	if (ret)
	{
		pr_info("Can't install IRQ %d - return code=%d\n",IntNumber,ret);
	}
	else
	{
		irq_requested = 1;
	}
	return 0;
}

static const struct file_operations gpio_fops = {
	.owner       = THIS_MODULE,
	.poll        = gpio_poll,
	.ioctl       = gpio_ioctl,
	.read        = gpio_read,
	.write       = gpio_write,
	.open        = gpio_open,
	.release     = gpio_release,
};

static struct cdev *c_dev;

void lpc178x_gpio_irq_setup(void)
{
	dev_t first;
	struct class *cl;

#if 0
	set_irq_chip(GPIO_IRQn, &gpio_irq_chip);
	//set_irq_handler(GPIO_IRQn, handle_edge_irq);
	//set_irq_handler(GPIO_IRQn, GPIO_IRQHandler);
	set_irq_flags(GPIO_IRQn, IRQF_VALID);
	//set_irq_chained_handler(GPIO_IRQn, lpc178x_gpio_irq_handler);
	//set_irq_data(GPIO_IRQn, (void *)0);
	//set_irq_flags(GPIO_IRQn, IRQF_VALID);
#endif


	if (alloc_chrdev_region(&first, 0, 1, gpio_name) < 0)  //$cat /proc/devices
	{
		return;
	}

	c_dev = cdev_alloc();
    c_dev->ops=&gpio_fops;
    c_dev->owner=THIS_MODULE;
	cdev_init(c_dev, &gpio_fops);
	if (cdev_add(c_dev, first, 1) == -1)
	{
/*
		device_destroy(cl, first);
		class_destroy(cl);
*/
		unregister_chrdev_region(first, 1);
		return;
	}

	if ((cl = class_create(THIS_MODULE, "chardrv")) == NULL)    //$ls /sys/class
	{
		unregister_chrdev_region(first, 1);
		return;
	}

	if (device_create(cl, NULL, first, NULL, "lpc.gpio") == NULL) //$ls /dev/
	{
		class_destroy(cl);
		unregister_chrdev_region(first, 1);
		return;
	}

#if 0
	//set_irq_chained_handler(GPIO_IRQn, GPIO_IRQHandler);
	//set_irq_data(GPIO_IRQn, (void *)alarmlist);
	//set_irq_flags(GPIO_IRQn, IRQF_VALID);
#endif

	pr_info("lpc178x:  gpio irqs installed\n");

}

#define GPIO_BASE_ITR_REGS 0x40028000
static volatile unsigned long *data_in[] = {
		(unsigned long *)(GPIO_BASE_ITR_REGS+0x80), // STATUS
		(unsigned long *)(GPIO_BASE_ITR_REGS+0x84), // STATR0
		(unsigned long *)(GPIO_BASE_ITR_REGS+0x88), // STATF0
		(unsigned long *)(GPIO_BASE_ITR_REGS+0xA4), // STATR2
		(unsigned long *)(GPIO_BASE_ITR_REGS+0xA8), // STATF2
};

static unsigned int gpio_poll(struct file *file, struct poll_table_struct *wait)
{
	unsigned int mask = 0;
	struct gpio_private *priv = (struct gpio_private *)file->private_data;
	unsigned long data = 0;
	poll_wait(file, &priv->alarm_wq, wait);

#if 1
	if (priv->minor == GPIO_MINOR_A)
	{
		//unsigned long tmp;
		unsigned long flags;

		local_irq_save(flags);
/// TODO: real content or irq
		spin_lock_irq(&gpio_lock);
		data |= IntData.P0IntStatR & priv->intalarm.P0IntStatR;
		data |= IntData.P0IntStatF & priv->intalarm.P0IntStatF;
		data |= IntData.P2IntStatR & priv->intalarm.P2IntStatR;
		data |= IntData.P2IntStatF & priv->intalarm.P2IntStatF;
		spin_unlock_irq(&gpio_lock);
		local_irq_restore(flags);
	}
	else
	{
		return 0;
	}
#endif

	if (data)
	{
		mask = POLLIN|POLLRDNORM;
	}

	printk(KERN_DEBUG "gpio_poll ready: mask 0x%08X\n", mask);
	return mask;

}

static int
gpio_ioctl(struct inode *inode, struct file *file,
	   unsigned int cmd, unsigned long arg)
{
	struct gpio_private *priv = (struct gpio_private *)file->private_data;
	if (_IOC_TYPE(cmd) != LPC178X_GPIO_IOCTYPE)
		return -EINVAL;

	switch (_IOC_NR(cmd))
	{
		case IO_READBITS: /* Use IO_READ_INBITS and IO_READ_OUTBITS instead */
			{
				/* Read the register interrupt port. */
				if (arg<sizeof(data_in)/sizeof(data_in[0]))
					return *data_in[arg];
				else
					return -EINVAL;
			}
			break;
		case IO_SETBITS:
			{
			}
			break;

		case IO_HIGHALARM:
			{
				int portnum=arg/32;
				int pinnum=arg%32;
				if (portnum==0)
				{
					priv->intalarm.P0IntStatR |=  (1<<pinnum);
					//pr_info("lpcpio:install raise alarm for port:%d, pin:%d\n",portnum,pinnum);
				}
				else if (portnum==2)
				{
					priv->intalarm.P2IntStatR |=  (1<<pinnum);
					//pr_info("lpcpio:install raise alarm for port:%d, pin:%d\n",portnum,pinnum);
				}
				else
					return -EINVAL;

			}
			break;

		case IO_LOWALARM:
			{
				int portnum=arg/32;
				int pinnum=arg%32;
				if (portnum==0)
				{
					priv->intalarm.P0IntStatF |=  (1<<pinnum);
					//pr_info("lpcpio:install falling alarm for port:%d, pin:%d\n",portnum,pinnum);
				}
				else if (portnum==2)
				{
					priv->intalarm.P2IntStatF |=  (1<<pinnum);
					//pr_info("lpcpio:install falling alarm for port:%d, pin:%d\n",portnum,pinnum);
				}
				else
					return -EINVAL;
			}
			break;
		default:
				return -EINVAL;
	} /* switch */

	return 0;
}

static ssize_t gpio_read(struct file *file, char *buf, size_t len, loff_t *ppos)
{
	DECLARE_WAITQUEUE(wait, current);
	unsigned long data[4]={};
	unsigned long _data=0;
	ssize_t retval;
	struct gpio_private *devp;

	devp = file->private_data;

	if (len < sizeof(unsigned long))
		return -EINVAL;

	add_wait_queue(&devp->alarm_wq, &wait);

	for ( ; ; ) {
		set_current_state(TASK_INTERRUPTIBLE);

		spin_lock_irq(&gpio_lock);
		data[0] |= IntData.P0IntStatR & devp->intalarm.P0IntStatR;
		data[1] |= IntData.P0IntStatF & devp->intalarm.P0IntStatF;
		data[2] |= IntData.P2IntStatR & devp->intalarm.P2IntStatR;
		data[3] |= IntData.P2IntStatF & devp->intalarm.P2IntStatF;
		_data =data[0]|data[1]|data[2]|data[3];
		IntData.P0IntStatR=0;
		IntData.P0IntStatF=0;
		IntData.P2IntStatR=0;
		IntData.P2IntStatF=0;
		spin_unlock_irq(&gpio_lock);

		if (_data)
			break;
		else if (file->f_flags & O_NONBLOCK) {
			retval = -EAGAIN;
			goto out;
		} else if (signal_pending(current)) {
			retval = -ERESTARTSYS;
			goto out;
		}
		schedule();
	}

	retval = put_user(data[0], (unsigned long __user *)buf);
	retval |= put_user(data[1], ((unsigned long __user *)buf)+1);
	retval |= put_user(data[2], ((unsigned long __user *)buf)+2);
	retval |= put_user(data[3], ((unsigned long __user *)buf)+3);
	if (!retval)
		retval = sizeof(unsigned long)*4;
out:
	__set_current_state(TASK_RUNNING);
	remove_wait_queue(&devp->alarm_wq, &wait);

	return retval;
}

static ssize_t gpio_write(struct file *file, const char *buf, size_t count,
	loff_t *off)
{
	return -EINVAL;
}

static int
gpio_open(struct inode *inode, struct file *filp)
{
	struct gpio_private *priv;
	//int p = iminor(inode);

	//if (p > GPIO_MINOR_LAST)
	//	return -EINVAL;

	priv = kmalloc(sizeof(struct gpio_private), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	lock_kernel();
	memset(priv, 0, sizeof(*priv));

	priv->minor = GPIO_MINOR_A;

	/* initialize the io/alarm struct */

	priv->intalarm.P0IntStatF = 0;
	priv->intalarm.P0IntStatR = 0;
	priv->intalarm.P2IntStatF = 0;
	priv->intalarm.P2IntStatR = 0;
	init_waitqueue_head(&priv->alarm_wq);

	filp->private_data = (void *)priv;

	priv->rbufhead = 0;
	priv->rbufcnt = 0;
	priv->flags = LPC178X_GPIO_FLAGS_DEV_OPEN;

	/* link it into our alarmlist */
	spin_lock_irq(&alarm_lock);
	priv->next = alarmlist;
	alarmlist = priv;
	spin_unlock_irq(&alarm_lock);

#if 1
	if(0!=install_irq(GPIO_IRQn,GPIO_IRQHandler,NORMAL_PRIORITY))
	{
		pr_err("lpc178x:  failed to install gpio irqs\n");
	}
#endif

	unlock_kernel();
	return 0;
}

static int
gpio_release(struct inode *inode, struct file *filp)
{
	struct gpio_private *p;
	struct gpio_private *todel;
	/* local copies while updating them: */
	unsigned long some_alarms;

	/* unlink from alarmlist and free the private structure */

	spin_lock_irq(&alarm_lock);
	p = alarmlist;
	todel = (struct gpio_private *)filp->private_data;

	if (p == todel) {
		alarmlist = todel->next;
	} else {
		while (p->next != todel)
			p = p->next;
		p->next = todel->next;
	}

	/* Check if there are still any alarms set */
	p = alarmlist;
	some_alarms = 0;
	while (p) {
		if (p->minor == GPIO_MINOR_A)
		{
			some_alarms = 1;
		}
		p = p->next;
	}

	if ((some_alarms==0)&&(irq_requested == 1))
	{
		free_irq(GPIO_IRQn, todel);
		irq_requested = 0;
	}

	kfree(todel);

	spin_unlock_irq(&alarm_lock);

	return 0;
}

