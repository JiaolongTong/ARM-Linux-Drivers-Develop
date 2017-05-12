/*
* a sample char device dirver: to realize the mmap in kernel.
* Copyright (C) 2017 Jiaolong Tong (tongjiaolong@yeah.net)
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
#include <linux/mm.h>
#define GLOBALMEM_SIZE  8192
#define MEM_CLEAR   0x1
#define GLOBALMEM_MAJOR 230
#define DEVICE_NUM  10

static  int globalmem_major = GLOBALMEM_MAJOR;         

module_param(globalmem_major,int,S_IRUGO);           

struct globalmem_dev{                                  
       struct cdev cdev;
       unsigned char * mem;   //设备内存指针
       struct mutex mutex;
};


struct globalmem_dev *globalmem_devp;                  //声明一个全局设备指针



static int globalmem_open(struct inode *inode,struct file *filep){

        globalmem_devp->mem = kzalloc(GLOBALMEM_SIZE,GFP_KERNEL);          //分配属于设备的内存空间（kzmall时建立虚拟地址个物理地址映射的页表）
        int i;
        struct page *mypage;
        for (i=0;i<GLOBALMEM_SIZE;i++)
            globalmem_devp->mem[i] =i%255;             //向设备内存空间写入初始值
        for(i=1;i<2;i++){                              //保证设备内存不被换出
              mypage=virt_to_page((void *)( globalmem_devp->mem+(i*PAGE_SIZE)));   //找到虚拟地址对应的page结构体    
              SetPageReserved(mypage); //设置页不被换出 
        }
	return 0;
}

static int globalmem_relase(struct inode *inode,struct file *filp)
{
    return 0;
}


static void * globalmem_mmap(struct file *filp,struct vm_area_struct *vma){

        unsigned long phys,len;
        unsigned long pfn;
        phys = virt_to_phys((void *)globalmem_devp->mem);  //将设备内存对应的虚拟地址转换成物理地址
        len =vma->vm_end -vma->vm_start;
        printk(KERN_INFO "## vm_start is %lx\n",vma->vm_start);
        printk(KERN_INFO "## vm_end is %lx\n",vma->vm_end);
        pfn =phys>>PAGE_SHIFT ;                            //phys>>12 根据物理地址找到物理页号  4k/页
        if(remap_pfn_range(vma,vma->vm_start,pfn,len,vma->vm_page_prot)){
                 printk("remap() error\n");
        }                                                  //根据物理页号和进程空间VMA建立进程空间个物理地址的映射
        return 0;
}
static const struct file_operations globalmem_ops={
	.owner   = THIS_MODULE,
	.open    = globalmem_open,
	.mmap    = globalmem_mmap,
	.release = globalmem_relase,
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
	   devno =MKDEV(globalmem_major,0);                      

	   if(globalmem_major)
	   	register_chrdev_region(devno,DEVICE_NUM,"mmmap_driver");      
	   else{
	   	alloc_chrdev_region(&devno,0,DEVICE_NUM,"mmmap_driver");      
		globalmem_major = MAJOR(devno);                   
	   }

           globalmem_devp=kzalloc(sizeof(struct globalmem_dev)*DEVICE_NUM,GFP_KERNEL);   
	   if(!globalmem_devp){
		  ret = -ENOMEM;
		  goto fail_alloc; 
	   	}

           mutex_init(&globalmem_devp->mutex);
           for(i=0;i<DEVICE_NUM;i++)
	   	globalmem_setup_cdev(globalmem_devp+i,i);                             

	   return 0;
fail_alloc:
	   unregister_chrdev_region(devno,DEVICE_NUM);
	   return -1;
	   	
	
}


static void __exit globalmem_exit(void){                                    //独立从系统中删除每个cdev设备，并回收设备号，完成设备卸载  
        int i;
        for(i=0;i<DEVICE_NUM;i++)                                                     
		cdev_del(&(globalmem_devp+i)->cdev);	
	unregister_chrdev_region(MKDEV(globalmem_major,0),DEVICE_NUM);      
	kfree(globalmem_devp);
}

module_init(globalmem_init);
module_exit(globalmem_exit);


MODULE_AUTHOR("Jiaolong Tong<tongjiaolong@yeah.net>");

MODULE_LICENSE("GPL v2");
