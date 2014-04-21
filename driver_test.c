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
#define DEVICE_GPIO		GPIO(90)
#define DEVICE_NAME		"DRIVER_TEST"

#define DEVICE_INFO(fmt, args...)	printk(KERN_INFO DEVICE_NAME": "fmt, ##args)
#define DEVICE_ERROR(fmt, args...)	printk(KERN_ALERT DEVICE_NAME": "fmt, ##args)

/********************************************************
 * struct definations
 *******************************************************/
struct test_driver
{
	struct class		*cls;
	int					major;
	wait_queue_head_t	wq;
	atomic_t			value;
};

/******************************************************** 
 * static variables
 *******************************************************/ 
static struct test_driver driver_data;;

/********************************************************
 * interrupt process
 *******************************************************/
static irqreturn_t driver_interrupt(int irq, void *id)
{
	struct test_driver *drv = &driver_data;

	atomic_set(&drv->value, 1);
	wake_up_interruptible(&drv->wq);

	return IRQ_HANDLED;
}

/******************************************************** 
 * driver open
 *******************************************************/
static int driver_open(struct inode *inode, struct file *file)
{
	struct test_driver *drv = &driver_data;

	atomic_set(&drv->value, 0);
	enable_irq(gpio_to_irq(DEVICE_GPIO));

	return 0;
}

/********************************************************
 * driver read
 *******************************************************/
static ssize_t driver_read(struct file *file, char __user *buf, size_t count, loff_t *offs)
{
	struct test_driver *drv = &driver_data;
	DEFINE_WAIT(wait);

	if (!atomic_read(&drv->value))
	{
		for (;;)
		{
			prepare_to_wait(&drv->wq, &wait, TASK_INTERRUPTIBLE);
			if (atomic_read(&drv->value))
				break;

			if (!signal_pending(current))
			{
				DEVICE_INFO("schedule out\n");
				schedule();
				continue;
			}

			break;
		}

		DEVICE_INFO("finish wait\n");
		finish_wait(&drv->wq, &wait);
	}

	//wait_event_interruptible(drv->wq, atomic_read(&drv->value));

	atomic_set(&drv->value, 0);

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
	int err = -ENOMEM;
	struct test_driver *drv = &driver_data;
	unsigned long addr = __get_free_pages(GFP_KERNEL,10);
	int i, j;
	struct free_area *area;

	area = NODE_DATA(0)->node_zones[0].free_area;
	for (i = 0; i < MAX_ORDER; i++)
	{
		printk("%s: order %d, nr_free %d\n", __func__, i, area->nr_free);
		for (j = 0; j < MIGRATE_TYPES; j++)
		{
		}
		area++;
	}

	printk("get page addr %08x\n", addr);

	/* request gpio */
	if (gpio_request(DEVICE_GPIO, DEVICE_NAME))
	{
		DEVICE_ERROR("failed to request gpio %d!\n", DEVICE_GPIO);
		goto err_return;
	}

	gpio_set_value(DEVICE_GPIO, 1);
	gpio_direction_input(DEVICE_GPIO);

	/* request irq */
	err = request_irq(gpio_to_irq(DEVICE_GPIO), driver_interrupt, IRQF_TRIGGER_RISING,
						DEVICE_NAME, NULL);
	if (err < 0)
	{
		DEVICE_ERROR("failed to request irq %d!\n", gpio_to_irq(DEVICE_GPIO));
		goto err_free_gpio;
	}

	disable_irq_nosync(gpio_to_irq(DEVICE_GPIO));

	init_waitqueue_head(&drv->wq);

	/* register device */
	err = register_chrdev(0, DEVICE_NAME, &driver_ops);
	if (err < 0)
	{
		DEVICE_ERROR("failed to register device!\n");
		goto err_free_irq;
	}

	drv->major = err;

	/* create sysfs entry */
	drv->cls = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(drv->cls))
	{
		DEVICE_ERROR("failed to create class!\n");
		goto err_unregister_device;
	}

	if (device_create(drv->cls, NULL, MKDEV(drv->major, 0), NULL, DEVICE_NAME) == NULL)
	{
		DEVICE_ERROR("failed to create device!\n");
		goto err_destroy_class;
	}

	DEVICE_INFO("device registerd!\n");

	return 0;

err_destroy_class:
	class_destroy(drv->cls);
err_unregister_device:
	unregister_chrdev(drv->major, DEVICE_NAME);
err_free_irq:
	free_irq(gpio_to_irq(DEVICE_GPIO), NULL);
err_free_gpio:
	gpio_free(DEVICE_GPIO);
err_return:
	return err;
}

/******************************************************** 
 * module unload
 *******************************************************/ 
static void __exit test_driver_exit(void)
{
	struct test_driver *drv = &driver_data;

	device_destroy(drv->cls, MKDEV(drv->major, 0));
	class_destroy(drv->cls);
	unregister_chrdev(drv->major, DEVICE_NAME);
	free_irq(gpio_to_irq(DEVICE_GPIO), NULL);
	gpio_free(DEVICE_GPIO);
}

module_init(test_driver_init);
module_exit(test_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Liu Yanqiang");
MODULE_DESCRIPTION("Test driver");
