/*
 * character device wrapper for generic gpio layer
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA02111-1307USA
 *
 * Feedback, Bugs...  blogic@openwrt.org
 *
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/atomic.h>
#include <linux/init.h>
#include <linux/genhd.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>

#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))
#define BUZZER_CTL GPIO_TO_PIN(3, 19)
static int buzzer_state;

static ssize_t buzzer_ctl_read(struct file *filp, char *buf,size_t count,loff_t *f_ops)
{
	return count;
}

static ssize_t buzzer_ctl_write(struct file *filp,const char *buf,size_t count,loff_t *f_ops)
{
	return count;
}

static int buzzer_ctl_ioctl(struct inode * inode, struct file * file, unsigned int cmd, unsigned long arg)
{
	return 0;
}

static int buzzer_ctl_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int buzzer_ctl_close(struct inode * inode, struct file * file)
{
	return 0;
}

static ssize_t buzzer_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%u\n", buzzer_state);
}

static ssize_t buzzer_state_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
        buzzer_state = simple_strtoul(buf, NULL, 10);

        gpio_direction_output(BUZZER_CTL, buzzer_state);

        return size;
}

struct file_operations buzzer_ctl_fops = {
	.read		= buzzer_ctl_read,
	.write		= buzzer_ctl_write,	
	.unlocked_ioctl	= buzzer_ctl_ioctl,
	.open		= buzzer_ctl_open,
	.release	= buzzer_ctl_close,
};

static struct miscdevice buzzer_ctl_dev = {
        .minor         = MISC_DYNAMIC_MINOR,
        .name         = "buzzer_ctl",                   
        .fops         = &buzzer_ctl_fops,
};

static DEVICE_ATTR(state, 0644, buzzer_state_show, buzzer_state_store);

static struct attribute *buzzer_ctl_attributes[] = {
	&dev_attr_state.attr,
        NULL,
};

static struct attribute_group buzzer_ctl_attr_group = {
        .attrs = buzzer_ctl_attributes,
};

static int __init buzzer_ctl_dev_init(void)
{
	int ret;
	
	/*printk("--------buzzer_ctl_dev_init\n");*/
	/*
	ret = gpio_request(BUZZER_CTL, "gpio3_19");
	if(ret < 0)
		printk("--------request gpio %d fail!\n", 115);
	*/
  
	ret = misc_register(&buzzer_ctl_dev);
	if(ret){
		printk(KERN_ERR "misc_register failed\n");
		return ret;
	}

	ret = sysfs_create_group(&buzzer_ctl_dev.this_device->kobj, &buzzer_ctl_attr_group);
        if (ret){
		printk(KERN_ERR "creat attr file failed\n");
		misc_deregister(&buzzer_ctl_dev);
		return ret;
	}

	return 0;
}

static void __exit buzzer_ctl_dev_exit(void)
{
	sysfs_remove_group(&buzzer_ctl_dev.this_device->kobj, &buzzer_ctl_attr_group);
	misc_deregister(&buzzer_ctl_dev);
}

module_init (buzzer_ctl_dev_init);
module_exit (buzzer_ctl_dev_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Timll");
MODULE_DESCRIPTION("Character device for power ctl");
