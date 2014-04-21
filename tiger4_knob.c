#include <linux/module.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/signal.h>
#include <linux/timer.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <asm/segment.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/spinlock.h>
#include <asm/irq.h>

#include <asm/mach/time.h>

#include <mach/clkdev.h>
#include <mach/hardware.h>

/******************************************************** 
 * driver used resources
 *******************************************************/ 
#define KNOB1_NAME		"tiger_knob"
#define KNOB1_MAJOR		265
#define KNOB1_A			GPIO(90)
#define KNOB1_B			GPIO(94)
#define KNOB1_UP		50
#define KNOB1_DN		51

#define KNOB2_NAME		"tiger_tuner"
#define KNOB2_MAJOR		266
#define KNOB2_A			GPIO(95)
#define KNOB2_B			GPIO(91)
#define KNOB2_UP		60
#define KNOB2_DN		61

/******************************************************** 
 * MTU1 register definations
 *******************************************************/ 
#define MTU_REG_IMSC    0x00
#define MTU_REG_RIS		0x04
#define MTU_REG_ICR		0x0C
#define MTU_REG_T0LR    0x10
#define MTU_REG_T1LR    0x20
#define MTU_REG_T0CR    0x18
#define MTU_REG_T1CR    0x28
#define MTU_REG_T2CR    0x38
#define MTU_REG_T3CR    0x48
#define IMSC_T0_SET (1 << 0)
#define IMSC_T1_SET (1 << 1)
#define IMSC_T2_SET (1 << 2)
#define IMSC_T3_SET (1 << 3)
#define IMSC_TALL_CLEAR (0)
#define ICR_T0_ACK  (1 << 0)
#define ICR_T1_ACK  (1 << 1)
#define ICR_T2_ACK  (1 << 2)
#define ICR_T3_ACK  (1 << 3)
#define ICR_TALL_ACK    (ICR_T0_ACK | ICR_T1_ACK | ICR_T2_ACK | ICR_T3_ACK)

#define TXCR_WRAPPING   (0 << 0)
#define TXCR_ONESHOT    (1 << 0)
#define TXCR_16BIT  (0 << 1)
#define TXCR_32BIT  (1 << 1)
#define TXCR_DIVBY1 (0 << 2)
#define TXCR_DIVBY16    (1 << 2)
#define TXCR_DIVBY256   (2 << 2)
#define TXCR_FREE   (0 << 6)
#define TXCR_PERIODIC   (1 << 6)
#define TXCR_DISABLE    (0 << 7)
#define TXCR_ENABLE (1 << 7)

#define DEBOUNCE_PERIOD		(CLOCK_TICK_RATE/1000)	/* 1ms */
#define DEBOUNCE_TIMES		8	

/******************************************************** 
 * static variables
 *******************************************************/ 
static void __iomem *mtu1_base;
static atomic_t knob1_value = ATOMIC_INIT(0);
static atomic_t knob2_value = ATOMIC_INIT(0);
DECLARE_WAIT_QUEUE_HEAD(knob1_waitq);
DECLARE_WAIT_QUEUE_HEAD(knob2_waitq);
static int knob1_id = KNOB1_MAJOR, knob2_id = KNOB2_MAJOR;

/******************************************************** 
 * timer irq used for debounce
 *******************************************************/ 
static irqreturn_t knob_timer_irq_handler(int irq, void *id)
{
	static int knob1_debounce = 0;
	static int knob2_debounce = 0;
	static int knob1_a = -1;
	static int knob1_b = -1;
	static int knob2_a = -1;
	static int knob2_b = -1;
	static int knob1_last_value = 0;
	static int knob2_last_value = 0;
	static unsigned long knob1_jiffies = INITIAL_JIFFIES;
	static unsigned long knob2_jiffies = INITIAL_JIFFIES;
	int curr_value;
	int irq_source;
	uint8_t test[0x1800];
	struct task_struct *tsk = current;
	uint32_t sp;

	printk("task id %d, task name %s\n", tsk->pid, tsk->comm);
	asm("mov %0,sp" : "=r" (sp));
	printk("++%08x %08x\n", sp, &sp);

	test[0x1790] = 50;
	test[0x1791] = 50;
	test[0x1794] = 50;
	test[0x1792] = 50;
	test[0x1796] = 50;
	test[0x1793] = 50;
	irq_source = readl(mtu1_base + MTU_REG_RIS);

	if (irq_source & ICR_T0_ACK)
	{
		/* timer ack */
		writel(ICR_T0_ACK, mtu1_base + MTU_REG_ICR);

		knob1_debounce++;

		/* not the first time */
		if (knob1_a >= 0)
		{
			/* invalid key status */
			if ((knob1_a != gpio_get_value(KNOB1_A)) || (knob1_b != gpio_get_value(KNOB1_B)))
				goto knob1_done;

			if (knob1_debounce < DEBOUNCE_TIMES)
				return IRQ_HANDLED;

			/* debounce ok */
			curr_value = (knob1_a != knob1_b)?KNOB1_UP:KNOB1_DN;

			if ((curr_value == knob1_last_value) || (time_after(jiffies, knob1_jiffies+15)))
			{
				/* set value and wake up the process */
				atomic_set(&knob1_value, curr_value);
				wake_up_interruptible(&knob1_waitq);
			}

			knob1_last_value = curr_value;
			knob1_jiffies = jiffies;

			/* initialize static values and disable timer */
knob1_done:
			knob1_debounce = 0;
			knob1_a = -1;
			knob1_b = -1;
			writel(TXCR_DISABLE, mtu1_base + MTU_REG_T0CR);
	asm("mov %0,sp" : "=r" (sp));
	printk("**%08x %08x\n", sp, &sp);
			return IRQ_HANDLED;
		}
		else	/* the first sample */
		{
			knob1_a = gpio_get_value(KNOB1_A);
			knob1_b = gpio_get_value(KNOB1_B);
		}
	}
	else if (irq_source & ICR_T1_ACK)
	{
		/* timer ack */
		writel(ICR_T1_ACK, mtu1_base + MTU_REG_ICR);

		knob2_debounce++;

		/* not the first time */
		if (knob2_a >= 0)
		{
			/* invalid key status */
			if ((knob2_a != gpio_get_value(KNOB2_A)) || (knob2_b != gpio_get_value(KNOB2_B)))
				goto knob2_done;

			if (knob2_debounce < DEBOUNCE_TIMES)
				return IRQ_HANDLED;

			/* debounce ok */
			curr_value = (knob2_a != knob2_b)?KNOB2_UP:KNOB2_DN;

			if ((curr_value == knob2_last_value) || (time_after(jiffies, knob2_jiffies+15)))
			{
				/* set value and wake up the process */
				atomic_set(&knob2_value, curr_value);
				wake_up_interruptible(&knob2_waitq);
			}

			knob2_last_value = curr_value;
			knob2_jiffies = jiffies;

			/* initialize static values and disable timer */
knob2_done:
			knob2_debounce = 0;
			knob2_a = -1;
			knob2_b = -1;
			writel(TXCR_DISABLE, mtu1_base + MTU_REG_T1CR);
			return IRQ_HANDLED;
		}
		else	/* the first sample */
		{
			knob2_a = gpio_get_value(KNOB2_A);
			knob2_b = gpio_get_value(KNOB2_B);
		}
	}

	return IRQ_HANDLED;
}

/******************************************************** 
 * irq process
 *******************************************************/ 
static irqreturn_t knob_interrupt(int irq, void *id)
{
	uint32_t sp;

	asm("mov %0,sp" : "=r" (sp));
	printk("%08x\n", sp);

	if (*(int *)id == KNOB1_MAJOR)
	{
		/* if the last not comsumed */
		if (TXCR_DISABLE != readl(mtu1_base + MTU_REG_T0CR))
			return IRQ_HANDLED;

		/* enable MTU1-timer0 for access */
		writel(DEBOUNCE_PERIOD, mtu1_base + MTU_REG_T0LR);
		writel(TXCR_32BIT|TXCR_PERIODIC|TXCR_ENABLE, mtu1_base + MTU_REG_T0CR);
	}
	else if (*(int *)id == KNOB2_MAJOR)
	{
		/* if the last not comsumed */
		if (TXCR_DISABLE != readl(mtu1_base + MTU_REG_T1CR))
			return IRQ_HANDLED;

		/* enable MTU1-timer0 for access */
		writel(DEBOUNCE_PERIOD, mtu1_base + MTU_REG_T1LR);
		writel(TXCR_32BIT|TXCR_PERIODIC|TXCR_ENABLE, mtu1_base + MTU_REG_T1CR);
	}

	return IRQ_HANDLED;
}

/******************************************************** 
 * the open function
 *******************************************************/ 
static int tigerknob_open(struct inode *inode, struct file *file)
{
	if (MAJOR(inode->i_rdev) == KNOB1_MAJOR)
	{
		/* initialize knob value */
		atomic_set(&knob1_value, 0);

		/* enable gpio irq */
		enable_irq(gpio_to_irq(KNOB1_A));
	}
	else if (MAJOR(inode->i_rdev) == KNOB2_MAJOR)
	{
		/* initialize knob value */
		atomic_set(&knob2_value, 0);

		/* enable gpio irq */
		enable_irq(gpio_to_irq(KNOB2_A));
	}

	file->private_data = (void *)inode;

	return 0;
}

/******************************************************** 
 * the close function
 *******************************************************/ 
static int tigerknob_close(struct inode *inode, struct file *file)
{
	if (MAJOR(inode->i_rdev) == KNOB1_MAJOR)
	{
		disable_irq_nosync(gpio_to_irq(KNOB1_A));
	}
	else if (MAJOR(inode->i_rdev) == KNOB2_MAJOR)
	{
		disable_irq_nosync(gpio_to_irq(KNOB2_A));
	}

	return 0;
}

/******************************************************** 
 * the read function
 *******************************************************/ 
static ssize_t tigerknob_read(struct file *file, char __user *buff, size_t count, loff_t *offp)
{
	struct inode *inode = (struct inode *)file->private_data;
	unsigned long err = -EAGAIN;
	int value = 0;

	unsigned int ttbr = 0;
	struct task_struct *tsk = current;

	printk("task id %d, task name %s\n", tsk->pid, tsk->comm);
	asm("mrc p15,0,%0,c2,c0,0" : "=r" (ttbr));
	printk("TTBR0 is %08x\n", ttbr);

	asm("mov %0,sp" : "=r" (ttbr));
	printk("--%08x\n", ttbr);
	/* if knob_value set */
	if (MAJOR(inode->i_rdev) == KNOB1_MAJOR)
	{
		value = atomic_read(&knob1_value);
		if (value)
			atomic_set(&knob1_value, 0);
	}
	else if (MAJOR(inode->i_rdev) == KNOB2_MAJOR)
	{
		value = atomic_read(&knob2_value);
		if (value)
			atomic_set(&knob2_value, 0);
	}

	if (value)
		err = copy_to_user(buff, (void *)&value, min(sizeof(value), count));

	if (err == 0)
		goto read_return;

	/* if open with flag O_NONBLOCK */
	if (file->f_flags & O_NONBLOCK)
		goto read_return;

	/* wait for switch event */
	if (MAJOR(inode->i_rdev) == KNOB1_MAJOR)
	{
		if (wait_event_interruptible(knob1_waitq, atomic_read(&knob1_value)) == 0)
		{
			/* read data */
			value = atomic_read(&knob1_value);
			atomic_set(&knob1_value, 0);
		}
	}
	else if (MAJOR(inode->i_rdev) == KNOB2_MAJOR)
	{
		if (wait_event_interruptible(knob2_waitq, atomic_read(&knob2_value)) == 0)
		{
			/* read data */
			value = atomic_read(&knob2_value);
			atomic_set(&knob2_value, 0);
		}
	}

	if (value)
		err = copy_to_user(buff, (void *)&value, min(sizeof(value), count));

read_return:
	return err ? -EFAULT : min(sizeof (value), count);
}

/******************************************************** 
 * the poll function
 *******************************************************/ 
static unsigned int tigerknob_poll(struct file *file, struct poll_table_struct *wait)
{
	struct inode *inode = (struct inode *)file->private_data;
	unsigned int mask = 0;

	if (MAJOR(inode->i_rdev) == KNOB1_MAJOR)
	{
		if (!atomic_read(&knob1_value))
			poll_wait(file, &knob1_waitq, wait);

		if (atomic_read(&knob1_value))
			mask |= POLLIN | POLLRDNORM;
	}
	else if (MAJOR(inode->i_rdev) == KNOB2_MAJOR)
	{
		if (!atomic_read(&knob2_value))
			poll_wait(file, &knob2_waitq, wait);

		if (atomic_read(&knob2_value))
			mask |= POLLIN | POLLRDNORM;
	}

	return mask;
}

static struct file_operations knob_ops = 
{
	.owner = THIS_MODULE,
	.open = tigerknob_open,
	.release = tigerknob_close,
	.read = tigerknob_read,
	.poll = tigerknob_poll,
}; 

/******************************************************** 
 * init gpios for knobs
 *******************************************************/ 
static int __init knob_init_gpio(void)
{
	int err = -ENOMEM;

	/* request gpios for knob 1*/
	if (gpio_request(KNOB1_A, "KNOB1_A"))
	{
		printk(KERN_ALERT KNOB1_NAME": unable to request gpio %d\n", KNOB1_A);
		goto err_gpio_return;
	}

	if (gpio_request(KNOB1_B, "KNOB1_B"))
	{
		printk(KERN_ALERT KNOB1_NAME": unable to request gpio %d\n", KNOB1_B);
		goto err_gpio_free_knob1_a;
	}

	/* request gpios for knob 2*/
	if (gpio_request(KNOB2_A, "KNOB2_A"))
	{
		printk(KERN_ALERT KNOB2_NAME": unable to request gpio %d\n", KNOB2_A);
		goto err_gpio_free_knob1_b;
	}

	if (gpio_request(KNOB2_B, "KNOB2_B"))
	{
		printk(KERN_ALERT KNOB2_NAME": unable to request gpio %d\n", KNOB2_B);
		goto err_gpio_free_knob2_a;
	}

	/* setup gpio for knob 1*/
	gpio_direction_input(KNOB1_A);
	gpio_direction_input(KNOB1_B);
	gpio_set_value(KNOB1_A, 1);
	gpio_set_value(KNOB1_B, 1);

	/* setup gpio for knob 2*/
	gpio_direction_input(KNOB2_A);
	gpio_direction_input(KNOB2_B);
	gpio_set_value(KNOB2_A, 1);
	gpio_set_value(KNOB2_B, 1);

	return 0;

err_gpio_free_knob2_a:
	gpio_free(KNOB2_A);
err_gpio_free_knob1_b:
	gpio_free(KNOB1_B);
err_gpio_free_knob1_a:
	gpio_free(KNOB1_A);
err_gpio_return:
	return err;
}

/******************************************************** 
 * free gpios
 *******************************************************/ 
static void knob_free_gpio(void)
{
	gpio_free(KNOB1_A);
	gpio_free(KNOB1_B);
	gpio_free(KNOB2_A);
	gpio_free(KNOB2_B);
}

/******************************************************** 
 * the module init function
 *******************************************************/ 
static int __init knob_init(void)
{
	int err = -EINVAL;
	unsigned int ttbr = 0;
	struct task_struct *tsk = current;
	unsigned char *p = (unsigned char *)0xC0004000;
	unsigned int mtu;

	printk("task id %d, task name %s\n", tsk->pid, tsk->comm);
	asm("mrc p15,0,%0,c2,c0,0" : "=r" (ttbr));
	printk("TTBR0 is %08x\n", ttbr);

	/* init gpio for knobs */
	err = knob_init_gpio();
	if (err)
		goto err_return;

	/* get and map MTU1 register space */
	if (!request_mem_region(CARTESIO_MTU1_BASE, SZ_4K, "MTU1"))
		goto err_free_gpio;

	mtu = 0xc71ba000;
	printk("mtu1_base is %p, physical %08x\n", mtu, CARTESIO_MTU1_BASE);
	/* get the first level descriptor address */
	p += ((mtu >> 20) << 2);
	printk("first level descriptor: %p, %08x\n", p, *(unsigned int *)p);
	/* get the second level descriptor address */
	p = ((*(unsigned int *)p) & 0xFFFFFC00) + 0xC0000000;
	printk("%p \n", p);
	p += (((mtu >> 12) & 0xFF) << 2);
	printk("second level descriptor: %p, %08x\n", p, *(unsigned int *)p);
	/* get the physical address */
	p = ((*(unsigned int*)p) >> 12) << 12;
	p += (mtu & 0xFFF);
	printk("physical address is %p\n", p);

	mtu1_base = ioremap(CARTESIO_MTU1_BASE, SZ_4K);
	if (!mtu1_base)
		goto err_release_mem;

	printk("----------------------------------------------\n");
	p = (unsigned char *)0xC0004000;
	mtu = mtu1_base;
	printk("mtu1_base is %p, physical %08x\n", mtu1_base, CARTESIO_MTU1_BASE);
	/* get the first level descriptor address */
	p += ((mtu >> 20) << 2);
	printk("first level descriptor: %p, %08x\n", p, *(unsigned int *)p);
	/* get the second level descriptor address */
	p = ((*(unsigned int *)p) & 0xFFFFFC00) + 0xC0000000;
	printk("%p \n", p);
	p += (((mtu >> 12) & 0xFF) << 2);
	printk("second level descriptor: %p, %08x\n", p, *(unsigned int *)p);
	/* get the physical address */
	p = ((*(unsigned int*)p) >> 12) << 12;
	p += (mtu & 0xFFF);
	printk("physical address is %p\n", p);
#if 0
	for (err = 0; err < 0x1000; err++)
	{
		if (*(p1 + err) != *(p2 + err))
			printk("%08x: %08x --> %08x\n", err*4, *(p2+err), *(p1+err));
	}
#endif

	/* setup MTU1 */
	writel(TXCR_DISABLE, mtu1_base + MTU_REG_T0CR);
	writel(TXCR_DISABLE, mtu1_base + MTU_REG_T1CR);
	writel(TXCR_DISABLE, mtu1_base + MTU_REG_T2CR);
	writel(TXCR_DISABLE, mtu1_base + MTU_REG_T3CR);
	writel(IMSC_T0_SET|IMSC_T1_SET, mtu1_base + MTU_REG_IMSC);
	writel(ICR_TALL_ACK, mtu1_base + MTU_REG_ICR);

	/* request timer irq */
	err = request_irq(IRQ_MTU1, knob_timer_irq_handler, 
			IRQF_TIMER | IRQF_IRQPOLL, "MTU1", NULL); 
	if (err)
	{
		printk(KERN_ALERT KNOB1_NAME": unable to request irq %d\n", IRQ_MTU1);
		goto err_io_unmap;
	}

	/* setup irq for knob 1*/
	err = request_irq(gpio_to_irq(KNOB1_A), knob_interrupt,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			KNOB1_NAME, &knob1_id);
	if (err)
	{
		printk(KERN_ALERT KNOB1_NAME": unable to request irq %d\n", gpio_to_irq(KNOB1_A));
		goto err_free_mtu_irq;
	}
	disable_irq_nosync(gpio_to_irq(KNOB1_A)); 

	/* setup irq for knob 2*/
	err = request_irq(gpio_to_irq(KNOB2_A), knob_interrupt,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			KNOB2_NAME, &knob2_id);
	if (err)
	{
		printk(KERN_ALERT KNOB2_NAME": unable to request irq %d\n", gpio_to_irq(KNOB2_A));
		goto err_free_knob1_irq;
	}
	disable_irq_nosync(gpio_to_irq(KNOB2_A)); 

	/* register knob1 device */
	err = register_chrdev(KNOB1_MAJOR, KNOB1_NAME, &knob_ops);
	if (err < 0)
	{
		printk(KERN_ALERT KNOB1_NAME": unable to register device\n");
		goto err_free_knob2_irq;
	}

	/* register knob2 device */
	err = register_chrdev(KNOB2_MAJOR, KNOB2_NAME, &knob_ops);
	if (err < 0)
	{
		printk(KERN_ALERT KNOB2_NAME": unable to register device\n");
		goto err_unregister_knob1_device;
	}

	return 0;

err_unregister_knob1_device:
	unregister_chrdev(KNOB1_MAJOR, KNOB1_NAME);
err_free_knob2_irq:
	free_irq(gpio_to_irq(KNOB2_A), &knob2_id);
err_free_knob1_irq:
	free_irq(gpio_to_irq(KNOB1_A), &knob1_id);
err_free_mtu_irq:
	free_irq(IRQ_MTU1, NULL);
err_io_unmap:
	iounmap(mtu1_base);
err_release_mem:
	release_mem_region(CARTESIO_MTU1_BASE, SZ_4K);
err_free_gpio:
	knob_free_gpio();
err_return:
	return err;
}

/******************************************************** 
 * the module exit function
 *******************************************************/ 
static void __exit knob_exit(void)
{
	disable_irq_nosync(gpio_to_irq(KNOB1_A));
	disable_irq_nosync(gpio_to_irq(KNOB2_A));
	disable_irq_nosync(IRQ_MTU1); 
	iounmap(mtu1_base);
	release_mem_region(CARTESIO_MTU1_BASE, SZ_4K);
	free_irq(IRQ_MTU1, NULL);
	free_irq(gpio_to_irq(KNOB1_A), &knob1_id);
	free_irq(gpio_to_irq(KNOB2_A), &knob2_id);

	unregister_chrdev(KNOB1_MAJOR, KNOB1_NAME);
	unregister_chrdev(KNOB2_MAJOR, KNOB2_NAME);
	knob_free_gpio();
}

MODULE_LICENSE("Dual BSD/GPL");
module_init(knob_init);
module_exit(knob_exit);

