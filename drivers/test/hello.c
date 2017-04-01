#include <linux/init.h>
#include <linux/module.h>

static int hello_data __initdata =100;

static int __init hello_init(void)
{
	printk(KERN_INFO "Hello World !My first test Init :%d! \n",hello_data);        
        return 0;
}

module_init(hello_init);

static void __exit hello_exit(void)
{
	printk(KERN_INFO "Hello World exit!\n");
}
module_exit(hello_exit);


MODULE_AUTHOR("Join Tong <tongjiaolong@yeah.net>");
MODULE_LICENSE("GPL V2");
//MODLUE_DESCRIPTION("A simple Hello World Module");
//MODLUE_ALIAS("a simple module");

