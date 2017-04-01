/*
* a sample char device dirver: globalmem without mutex
* Copyright (C) 2016 Jiaolong Tong (tongjiaolong@yeah.net)
*
* Licensed under GPLv2 or later
* 
*/

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#define GLOBALMEM_SIZE  0x1000
#define MEM_CLEAR   0x1
#define GLOBALMEM_MAJOR 230
#define DEVICE_NUM  10

static  int globalmem_major = GLOBALMEM_MAJOR;         // 设备驱动主设备号

module_param(globalmem_major,int,S_IRUGO);             //模块参数，可传入主设备号

struct globalmem_dev{                                  //设备结构体，封装了cdev和全局memory
       struct cdev cdev;
	   unsigned char mem[GLOBALMEM_SIZE];
};


struct globalmem_dev *globalmem_devp;                  //声明一个全局设备指针



static int globalmem_open(struct inode *inode,struct file *filep){

	filep->private_data = globalmem_devp;          //设备打开，设备指针指向file的私有数据private_data，驱动文件通过访问priavte_data就能访问设备文件
	return 0;
}



static int globalmem_relase(struct inode *inode,struct file *filp)
{
    return 0;
}


static long globalmem_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct globalmem_dev * dev = filp->private_data;
    switch(cmd){
    case MEM_CLEAR :
	memset(dev->mem,0,GLOBALMEM_SIZE);
	printk(KERN_INFO "globalmem is set to zero\n");
	break;
    
    default:
	return -EINVAL;
    }
    return 0;
}

static ssize_t globalmem_read(struct file * filp,char __user * buf,size_t size,loff_t * ppos)
{
    unsigned long p= * ppos;
    unsigned int count =size;
    int ret=0; 
    struct globalmem_dev * dev = filp->private_data;
    if (p>= GLOBALMEM_SIZE)
       return 0;
    if (count > GLOBALMEM_SIZE -p )
          count = GLOBALMEM_SIZE -p;
    if (copy_to_user(buf,dev->mem+p,count)){
       ret= -EINVAL;
    }else{
        *ppos +=count;
        ret=count;
        printk(KERN_INFO "read %u bytes(s) from %lu\n",count,p);
    }
    return ret;
}

static ssize_t globalmem_write(struct file * filp, const char __user *buf,size_t size , loff_t * ppos)
{
    unsigned long p=*ppos;
    unsigned int count = size ;
    int ret=0;
    struct globalmem_dev *dev=filp->private_data;
    if (p>= GLOBALMEM_SIZE)
       return 0;
    if (count > GLOBALMEM_SIZE -p )
          count = GLOBALMEM_SIZE -p;

    if(copy_from_user(dev->mem+p,buf,count)){
       ret= -EINVAL;
    }else{
	*ppos +=count;
        ret=count;
        printk(KERN_INFO "written  %u bytes(s) from %lu\n",count,p);
    } 
    return ret;
}

static loff_t globalmem_llseek(struct file * filp, loff_t offset ,int orig)
{
   loff_t ret=0;
   switch(orig){
   case 0:
        if(offset<0){
            ret =-EINVAL;
            break;
        }
        if((unsigned int)offset >GLOBALMEM_SIZE){
            ret =-EINVAL;
            break;
        }
        filp->f_pos =(unsigned int) offset;
        ret= filp->f_pos;
        break;
   case 1:
        if(filp->f_pos+offset <0){
            ret =-EINVAL;
            break;
        }
        if(filp->f_pos+offset >GLOBALMEM_SIZE){
            ret =-EINVAL;
            break;
        }
        filp->f_pos +=offset;
        ret= filp->f_pos;
        break;
  default:
       ret =-EINVAL;
       break;
  }
  return ret;
}



static const struct file_operations globalmem_ops={
	.owner  = THIS_MODULE,
	.open   = globalmem_open,
	.llseek = globalmem_llseek,
	.read   = globalmem_read,
	.write  = globalmem_write,
	.release = globalmem_relase,
	.unlocked_ioctl  = globalmem_ioctl,
};

static void globalmem_setup_cdev(struct globalmem_dev * dev, int index){

    int err, devno = MKDEV(globalmem_major,index);


    cdev_init(&dev->cdev,&globalmem_ops);

	dev->cdev.owner=THIS_MODULE;

	err=cdev_add(&dev->cdev,devno,1);

    if(err<0)
	 	printk(KERN_NOTICE "ERROR %d adding globalmem %d" ,err,index);

}



static int __init globalmem_init(void){

	   dev_t devno;
       int   ret,i;
	   devno =MKDEV(globalmem_major,0);                       //生成设备号

	   if(globalmem_major)
	   	register_chrdev_region(devno,1,"globalmem");      //手动注册设备号
	   else{
	   	alloc_chrdev_region(&devno,0,1,"globalmem");      //自动注册设备号(系统自动分配设备号)
		globalmem_major = MAJOR(devno);                   //获得主设备号
	   }

           globalmem_devp=kzalloc(sizeof(struct globalmem_dev),GFP_KERNEL);   //为设备分配内存空间
	   if(!globalmem_devp){
		  ret = -ENOMEM;
		  goto fail_alloc; 
	   	}
	   globalmem_setup_cdev(globalmem_devp,0);                  //初始化设备并向系统添加一个cdev，完成设备的注册 

	   return 0;
fail_alloc:
	   unregister_chrdev_region(devno,1);
	   return -1;
	   	
	
}


static void __exit globalmem_exit(void){                          //从系统中删除一个cdev设备，并回收设备号，完成设备卸载
	cdev_del(&(globalmem_devp)->cdev);	
	unregister_chrdev_region(MKDEV(globalmem_major,0),1);      
	kfree(globalmem_devp);
}

module_init(globalmem_init);
module_exit(globalmem_exit);


MODULE_AUTHOR("Jiaolong Tong<tongjiaolong@yeah.net>");

MODULE_LICENSE("GPL v2");
