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
//#define GPIO_DATA_OUT		0x0
//#define GPIO_DATA_IN		0x4
#define GPIO_DIR			0x0
#define GPIO_DATA_SET		0x18
#define GPIO_DATA_CLR		0x1C
//#define GPIO_INT_EN		0x20
#define GPIO_INT_STAT		0x24
#define GPIO_INT_MASK		0x10
#define GPIO_INT_CLR		0x1C

//#define GPIO_PULL_EN		0x18
//#define GPIO_PULL_TYPE		0x1C
//#define GPIO_INT_TYPE		0x34
//#define GPIO_INT_BOTH_EDGE	0x38
//#define GPIO_INT_LEVEL		0x3C
//#define GPIO_DEBOUNCE_EN	0x40
//#define GPIO_DEBOUNCE_PRESCALE	0x44

#define GPIO_PORT_NUM		3

//typedef unsigned int uint32_t;
#define  __IO
#define  __O
/** \brief  Structure type to access the Nested Vectored Interrupt Controller (NVIC).
 */
typedef struct
{
  __IO uint32_t ISER[8];                 /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register           */
       uint32_t RESERVED0[24];
  __IO uint32_t ICER[8];                 /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register         */
       uint32_t RSERVED1[24];
  __IO uint32_t ISPR[8];                 /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register          */
       uint32_t RESERVED2[24];
  __IO uint32_t ICPR[8];                 /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register        */
       uint32_t RESERVED3[24];
  __IO uint32_t IABR[8];                 /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register           */
       uint32_t RESERVED4[56];
  __IO uint8_t  IP[240];                 /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) */
       uint32_t RESERVED5[644];
  __O  uint32_t STIR;                    /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register     */
}  NVIC_Type;

/* Memory mapping of Cortex-M3 Hardware */
#define SCS_BASE            (0xE000E000UL)                            /*!< System Control Space Base Address  */
#define NVIC_BASE           (SCS_BASE +  0x0100UL)                    /*!< NVIC Base Address                  */
#define NVIC                ((NVIC_Type      *)     NVIC_BASE     )   /*!< NVIC configuration struct          */

typedef enum IRQn
{
/******  Cortex-M3 Processor Exceptions Numbers ***************************************************/
  Reset_IRQn                    = -15,      /*!< 1 Reset Vector, invoked on PowerUp and warm reset*/
  NonMaskableInt_IRQn           = -14,      /*!< 2 Non Maskable Interrupt                         */
  HardFault_IRQn                = -13,      /*!< 3  Hard Fault, all classes of Fault              */
  MemoryManagement_IRQn         = -12,      /*!< 4 Cortex-M3 Memory Management Interrupt          */
  BusFault_IRQn                 = -11,      /*!< 5 Cortex-M3 Bus Fault Interrupt                  */
  UsageFault_IRQn               = -10,      /*!< 6 Cortex-M3 Usage Fault Interrupt                */
  SVCall_IRQn                   = -5,       /*!< 11 Cortex-M3 SV Call Interrupt                   */
  DebugMonitor_IRQn             = -4,       /*!< 12 Cortex-M3 Debug Monitor Interrupt             */
  PendSV_IRQn                   = -2,       /*!< 14 Cortex-M3 Pend SV Interrupt                   */
  SysTick_IRQn                  = -1,       /*!< 15 Cortex-M3 System Tick Interrupt               */

/******  LPC177x_8x Specific Interrupt Numbers *******************************************************/
  WDT_IRQn                      = 0,        /*!< Watchdog Timer Interrupt                         */
  TIMER0_IRQn                   = 1,        /*!< Timer0 Interrupt                                 */
  TIMER1_IRQn                   = 2,        /*!< Timer1 Interrupt                                 */
  TIMER2_IRQn                   = 3,        /*!< Timer2 Interrupt                                 */
  TIMER3_IRQn                   = 4,        /*!< Timer3 Interrupt                                 */
  UART0_IRQn                    = 5,        /*!< UART0 Interrupt                                  */
  UART1_IRQn                    = 6,        /*!< UART1 Interrupt                                  */
  UART2_IRQn                    = 7,        /*!< UART2 Interrupt                                  */
  UART3_IRQn                    = 8,        /*!< UART3 Interrupt                                  */
  PWM1_IRQn                     = 9,        /*!< PWM1 Interrupt                                   */
  I2C0_IRQn                     = 10,       /*!< I2C0 Interrupt                                   */
  I2C1_IRQn                     = 11,       /*!< I2C1 Interrupt                                   */
  I2C2_IRQn                     = 12,       /*!< I2C2 Interrupt                                   */
  Reserved0_IRQn                = 13,       /*!< Reserved                                         */
  SSP0_IRQn                     = 14,       /*!< SSP0 Interrupt                                   */
  SSP1_IRQn                     = 15,       /*!< SSP1 Interrupt                                   */
  PLL0_IRQn                     = 16,       /*!< PLL0 Lock (Main PLL) Interrupt                   */
  RTC_IRQn                      = 17,       /*!< Real Time Clock Interrupt                        */
  EINT0_IRQn                    = 18,       /*!< External Interrupt 0 Interrupt                   */
  EINT1_IRQn                    = 19,       /*!< External Interrupt 1 Interrupt                   */
  EINT2_IRQn                    = 20,       /*!< External Interrupt 2 Interrupt                   */
  EINT3_IRQn                    = 21,       /*!< External Interrupt 3 Interrupt                   */
  ADC_IRQn                      = 22,       /*!< A/D Converter Interrupt                          */
  BOD_IRQn                      = 23,       /*!< Brown-Out Detect Interrupt                       */
  USB_IRQn                      = 24,       /*!< USB Interrupt                                    */
  CAN_IRQn                      = 25,       /*!< CAN Interrupt                                    */
  DMA_IRQn                      = 26,       /*!< General Purpose DMA Interrupt                    */
  I2S_IRQn                      = 27,       /*!< I2S Interrupt                                    */
  ENET_IRQn                     = 28,       /*!< Ethernet Interrupt                               */
  MCI_IRQn                      = 29,       /*!< SD/MMC card I/F Interrupt                        */
  MCPWM_IRQn                    = 30,       /*!< Motor Control PWM Interrupt                      */
  QEI_IRQn                      = 31,       /*!< Quadrature Encoder Interface Interrupt           */
  PLL1_IRQn                     = 32,       /*!< PLL1 Lock (USB PLL) Interrupt                    */
  USBActivity_IRQn              = 33,       /*!< USB Activity interrupt                           */
  CANActivity_IRQn              = 34,       /*!< CAN Activity interrupt                           */
  UART4_IRQn                    = 35,       /*!< UART4 Interrupt                                  */
  SSP2_IRQn                     = 36,       /*!< SSP2 Interrupt                                   */
  LCD_IRQn                      = 37,       /*!< LCD Interrupt                                    */
  GPIO_IRQn                     = 38,       /*!< GPIO Interrupt                                   */
  PWM0_IRQn                     = 39,       /*!< PWM0 Interrupt                                   */
  EEPROM_IRQn                   = 40,       /*!< EEPROM Interrupt                           */
} IRQn_Type;

#define NORMAL_PRIORITY   ( 0x08 )

/** \brief  Enable External Interrupt

    The function enables a device-specific interrupt in the NVIC interrupt controller.

    \param [in]      IRQn  External interrupt number. Value cannot be negative.
 */
static void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  NVIC->ISER[((unsigned int)(IRQn) >> 5)] = (1 << ((unsigned int)(IRQn) & 0x1F)); /* enable interrupt */
}


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
	NVIC_EnableIRQ(IntNumber);

	ret=request_irq(IntNumber,HandlerAddr,0,"LPC Gpio",NULL);
	if (ret)
	{
		pr_info("Can't install IRQ %d - return code=%d\n",IntNumber,ret);
	}
	return 0;
}

typedef struct
{
	volatile uint32_t P0IntStatR;
	volatile uint32_t P0IntStatF;
	volatile uint32_t P2IntStatR;
	volatile uint32_t P2IntStatF;
} GPIO_INT_REG;

static GPIO_INT_REG IntData = {};

irqreturn_t GPIO_IRQHandler(int irq, void *desc)
{
//	DISABLE_IRQ();

	IntData.P0IntStatR |= LPC_GPIOINT->IO0IntStatR;
	IntData.P0IntStatF |= LPC_GPIOINT->IO0IntStatF;
	IntData.P2IntStatR |= LPC_GPIOINT->IO2IntStatR;
	IntData.P2IntStatF |= LPC_GPIOINT->IO2IntStatF;


	//Clear interrupt status only for the relevant pins
	LPC_GPIOINT->IO0IntClr = (IntData.P0IntStatR | IntData.P0IntStatF);
	LPC_GPIOINT->IO2IntClr = (IntData.P2IntStatR | IntData.P2IntStatF);

	//os_event_set(EVENT_GOT_GPIO_INT,pio_task_id);
//	ENABLE_IRQ();

//	ack_irq();
	pr_err("GPIO_IRQHandler(int irq=%d, void *desc) R:%04X F:%04X \n",irq,IntData.P0IntStatR,IntData.P0IntStatF);
	return IRQ_HANDLED;
}

#define GPIO_IRQ_BASE	38

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
	//gpio_set_irq_type(gpio,IRQ_TYPE_EDGE_BOTH);
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
	.label			= "lpc178x",
	.direction_input	= lpc178x_gpio_direction_input,
	.get			= lpc178x_gpio_get_value,
	.direction_output	= lpc178x_gpio_direction_output,
	.set			= lpc178x_gpio_set_value,
	.base			= LPC178X_GPIO_OFF_MCU,
	.ngpio			= LPC178X_GPIO_LEN_MCU,
	.can_sleep		= 1,
};

void __init lpc178x_gpio_init(void)
{
	if (gpiochip_add(&lpc178x_chip) < 0)
		pr_err("%s: gpiochip_add failed.\n", __func__);

	lpc178x_gpio_irq_setup();
}

//EZ : only one IRQ
static int irq_to_gpio(unsigned int irq)
{
	return irq;
}

static void _set_gpio_irqenable(unsigned int base, unsigned int index,
				int enable)
{
	unsigned int reg;

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
	//pr_err("EZ: call gpio_ack_irq(unsigned int irq=%d) \n",irq);
}

static void gpio_mask_irq(unsigned int irq)
{
//	unsigned int gpio = irq_to_gpio(irq);
//	unsigned int base = GPIO_BASE(gpio / 32);
//
//	_set_gpio_irqenable(base, gpio % 32, 0);
	//pr_err("EZ: call gpio_mask_irq(unsigned int irq=%d) \n",irq);
}

static void gpio_unmask_irq(unsigned int irq)
{
//	unsigned int gpio = irq_to_gpio(irq);
//	unsigned int base = GPIO_BASE(gpio / 32);
//
//	_set_gpio_irqenable(base, gpio % 32, 1);
	//pr_err("EZ: call gpio_unmask_irq(unsigned int irq=%d) \n",irq);
}

static int gpio_set_irq_type(unsigned int irq, unsigned int type)
{
	unsigned int gpio = irq_to_gpio(irq);
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

	gpio_ack_irq(irq);
	return 0;
}

static void gpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	//unsigned int gpio_irq_no, irq_stat;
	//unsigned int port = (unsigned int)get_irq_data(irq);

	//GPIO_IRQHandler();
	if (irq_desc[GPIO_IRQn].handle_irq)
	{
		irq_desc[GPIO_IRQn].handle_irq(GPIO_IRQn,	&irq_desc[GPIO_IRQn]);
	}
	/*
	irq_stat = __raw_readl(GPIO_BASE(port) + GPIO_INT_STAT);

	gpio_irq_no = GPIO_IRQ_BASE + port * 32;
	for (; irq_stat != 0; irq_stat >>= 1, gpio_irq_no++) {

		if ((irq_stat & 1) == 0)
			continue;

		BUG_ON(!(irq_desc[gpio_irq_no].handle_irq));
		irq_desc[gpio_irq_no].handle_irq(gpio_irq_no,
				&irq_desc[gpio_irq_no]);
	}
	*/
}

static struct irq_chip gpio_irq_chip = {
	.name = "GPIO",
	.ack = gpio_ack_irq,
	.mask = gpio_mask_irq,
	.unmask = gpio_unmask_irq,
	.set_type = gpio_set_irq_type,
};

int lpc178x_gpio_to_irq(unsigned gpio)
{
	return GPIO_IRQ_BASE;
}
/*
 * Called from the processor-specific init to enable GPIO interrupt support.
 */
void lpc178x_gpio_irq_setup(void)
{
	int i, j;

	for (i = 0; i < GPIO_PORT_NUM; i++)
	{
		// Only GPIO0 and GPIO2 are irq enabled
		if (i>=1) continue;
		/* disable, unmask and clear all interrupts */
//		__raw_writel(0x0, GPIO_BASE(i) + GPIO_INT_EN);
//		__raw_writel(0x0, GPIO_BASE(i) + GPIO_INT_MASK);
//		__raw_writel(~0x0, GPIO_BASE(i) + GPIO_INT_CLR);


#if 0
		for (j = GPIO_IRQ_BASE + i * 32;
				     j < GPIO_IRQ_BASE + (i + 1) * 32; j++) {
					set_irq_chip(j, &gpio_irq_chip);
					set_irq_handler(j, handle_simple_irq);
					set_irq_flags(j, IRQF_VALID);
					//pr_info("set handler");
				}
#endif

		set_irq_chip(GPIO_IRQ_BASE, &gpio_irq_chip);
		set_irq_handler(GPIO_IRQ_BASE, handle_edge_irq);
		//set_irq_handler(GPIO_IRQ_BASE,handle_simple_irq);
		set_irq_flags(GPIO_IRQ_BASE, IRQF_VALID);

		set_irq_chained_handler((int)GPIO_IRQn, gpio_irq_handler);
		set_irq_data((int)GPIO_IRQn, (void *)i);
	}

	//install_irq(IRQn_Type IntNumber,void (*HandlerAddr)(void)__irq,unsigned int Priority)// enable InterruptHandler
#if 0
	if(0!=install_irq(GPIO_IRQn,GPIO_IRQHandler,NORMAL_PRIORITY))
	{
		pr_err("lpc178x:  failed to install gpio irqs\n");
		return;
	}
#endif
	pr_info("lpc178x:  gpio irqs installed\n");
}




