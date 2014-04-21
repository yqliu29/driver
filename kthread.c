#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/magic.h>

/******************************************************** 
 * macros
 *******************************************************/ 
#define START_ADDR	0xC0000000
#define END_ADDR	(START_ADDR + 0x8000000)

/********************************************************
 * struct definations
 *******************************************************/

/******************************************************** 
 * static variables
 *******************************************************/ 
static unsigned int start_addr = START_ADDR;
static unsigned int end_addr = END_ADDR;
module_param(start_addr, uint, 0);
module_param(end_addr, uint, 0);

static int kthread_test(void *data)
{
	unsigned int *p;
	struct task_struct *tsk;
	struct thread_info *tinfo;
	int i;

	unsigned char test[8192-0x268];
	int sum = 0;


	while (1)
	{
		for (i = 0;i < sizeof (test); i++)
		{
			test[i] = 0xde;
		}
		
		for (i = 0; i < sizeof (test); i++)
			sum += test[i];

		printk("sum is %d, test[8099] address %p\n", sum, &test[8099]);
		for_each_process(tsk) 
		{
				p = 0x0000dead;
				*p = 1;
			tinfo = task_thread_info(tsk);
			p = (unsigned int *)(tinfo + 1);
			if (*p != STACK_END_MAGIC)
			{
				printk(KERN_ALERT"Task id %s(pid %d) stack overflow happened!\n", tsk->comm, tsk->pid);
				p = 0x0000dead;
				*p = 1;
			}
#if 0
			if (tsk->mm)
			{
				if (tsk->mm->pgd)
				{
					printk("task id %d, task pgd %p, task name %s\n", tsk->pid, tsk->mm->pgd, tsk->comm);
				}
				else
				{
					printk("task id %d, task name %s, no pgd\n", tsk->pid, tsk->comm);
				}
			}
			else
			{
				printk("task id %d, task name %s, no mm\n", tsk->pid, tsk->comm);
			}
#endif
		}

		msleep(1000);
	}

	return 0;
}

/******************************************************** 
 * module init
 *******************************************************/ 
static int __init test_driver_init(void)
{
	unsigned int ttbr = 0;
	struct task_struct *tsk = current;

	request_mem_region(CARTESIO_CAN1_BASE, SZ_4K, "CAN1");
	ioremap(CARTESIO_CAN1_BASE, SZ_4K);

	printk("task id %d, task name %s\n", tsk->pid, tsk->comm);
	asm("mrc p15,0,%0,c2,c0,0" : "=r" (ttbr));
	printk("TTBR0 is %08x\n", ttbr);

	kthread_run(kthread_test, NULL, "kthread_test2");
	//kthread_run(kthread_test, NULL, "kthread_test1");
	//kthread_run(kthread_test, NULL, "kthread_test3");

	return 0;
}

/******************************************************** 
 * module unload
 *******************************************************/ 
static void __exit test_driver_exit(void)
{
}

module_init(test_driver_init);
module_exit(test_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Liu Yanqiang");
MODULE_DESCRIPTION("Test driver");
