#include <linux/init.h>
#include <linux/module.h>


/*
#define S_IRUSR 00400       文件所有者可读
#define S_IWUSR 00200       文件所有者可写
#define S_IXUSR 00100       文件所有者可执行
#define S_IRGRP 00040       与文件所有者同组的用户可读
#define S_IWGRP 00020
#define S_IXGRP 00010
#define S_IROTH 00004       与文件所有者不同组的用户可读
#define S_IWOTH 00002
#define S_IXOTH 00001

*/

static char *book_name ="dissecting Linux Device Driver";
module_param(book_name,charp,S_IRUGO);                    //向模块传递字符串指针参数

static int book_num  = 4000;
module_param(book_num,int,S_IRUGO);                       //向模块传递整型参数


static __init book_init()
{
	printk(KERN_INFO "book name :%s\n",book_name);
        printk(KERN_INFO "book num :%d\n",book_num);
        return 0;
}

module_init(book_init);

static __exit book_exit()
{
	printk(KERN_INFO "book module exit\n");
}

module_exit(book_exit);


MODULE_AUTHOR("Join Tong <tongjiaolong@yeah.net>");
MODULE_LICENSE("GPL v2");
