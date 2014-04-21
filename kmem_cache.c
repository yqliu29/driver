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
#define TEST_SIZE		0x200

/********************************************************
 * struct definations
 *******************************************************/

/******************************************************** 
 * static variables
 *******************************************************/ 
static struct kmem_cache *cachep;

/********************************************************
 * interrupt process
 *******************************************************/

/******************************************************** 
 * driver open
 *******************************************************/
static int driver_open(struct inode *inode, struct file *file)
{
	return 0;
}

/********************************************************
 * driver read
 *******************************************************/
static ssize_t driver_read(struct file *file, char __user *buf, size_t count, loff_t *offs)
{
	return count;
}

/********************************************************
 * driver write
 *******************************************************/
static ssize_t driver_write(struct file *file, const char __user *buf, size_t count, loff_t *offs)
{
	return count;
}

/********************************************************
 * driver release
 *******************************************************/
static int driver_release(struct inode *inode, struct file *file)
{
	return 0;
}

/******************************************************** 
 * file operations
 *******************************************************/
static struct file_operations driver_ops = {
	.owner = THIS_MODULE,
	.open = driver_open,
	.read = driver_read,
	.write = driver_write,
	.release = driver_release,
};

/******************************************************** 
 * module init
 *******************************************************/ 
static int __init test_driver_init(void)
{
	void *p;
	unsigned int *pi;
	int i,j,k;

	cachep = kmem_cache_create("kmem_cache_test", TEST_SIZE,
								0, 0, NULL);
	if (cachep == NULL)
	{
		printk("failed to create cache for test!\n");
		return -ENOMEM;
	}

	p = kmem_cache_alloc(cachep, GFP_KERNEL);
	if (p == NULL)
	{
		printk("failed to alloc from cache!\n");
		return -ENOMEM;
	}

	printk("Alloc from cache ok, size is %x, address is %p\n", TEST_SIZE, p);

	pi = (unsigned int *)p;
	j = (TEST_SIZE+3)/4 + 0x1000;
	for (i = 0; i < j; i++)
		printk("%08x: %08x\n", i*4,*(pi+i));	

	p = kmalloc(TEST_SIZE, GFP_KERNEL);

	if (p == NULL)
	{
		printk("failed to alloc with kmalloc!\n");
		return -ENOMEM;
	}

	printk("Alloc with kmalloc ok, size is %x, address is %p\n", TEST_SIZE, p);
	pi = (unsigned int *)p;
	j = (TEST_SIZE+3)/4 + 0x1000;
	for (i = 0; i < j; i++)
		printk("%08x: %08x\n", i*4,*(pi+i));	

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
