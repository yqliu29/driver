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

/******************************************************** 
 * module init
 *******************************************************/ 
static int __init test_driver_init(void)
{
	unsigned long ttbr = 0;
	struct task_struct *tsk = current;

	printk("task id %d, task name %s\n", tsk->pid, tsk->comm);
	asm("mrc p15,0,%0,c2,c0,0" : "=r" (ttbr));
//	asm( "mrc\tp14, 0, %0, c6, c0, 0" : "=r" (clkcfg) );
	printk("TTBR0 is %08x\n", ttbr);
#if 0
	unsigned int *p = (unsigned int *)start_addr;
	unsigned int *e = (unsigned int *)end_addr;

	while (p < e)
	{
		printk("%p: %08x\n", p, *p);
		p += PAGE_SIZE;
	}
#endif

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
