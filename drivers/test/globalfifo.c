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
#include <linux/sched.h>
#include <linux/poll.h>
#define GLOBALFIFO_SIZE  20
#define FIFO_CLEAR   0x1
#define GLOBALFIFO_MAJOR 240
#define DEVICE_NUM  10

static  int globalfifo_major = GLOBALFIFO_MAJOR;         // 设备驱动主设备号

module_param(globalfifo_major,int,S_IRUGO);             //模块参数，可传入主设备号

struct globalfifo_dev{                                  //设备结构体，封装了cdev和全局memory
       struct cdev cdev;
       unsigned char mem[GLOBALFIFO_SIZE];
       struct mutex mutex;
       int current_len;
       wait_queue_head_t w_wait;                        //定义写阻塞等待队列头
       wait_queue_head_t r_wait;                        //定义读阻塞等待队列头

       struct fasync_struct *async_queue;               //定义一个异步通知结构
};


struct globalfifo_dev *globalfifo_devp;                  //声明一个全局设备指针



static int globalfifo_open(struct inode *inode,struct file *filep){

        struct globalfifo_dev *dev=container_of(inode->i_cdev,struct globalfifo_dev,cdev);    //从inode结构中获取设备globalmem_devp地址
	filep->private_data = dev;
        //filep->private_data = globalmem_devp          //两种方式都行
	return 0;
}


static long globalfifo_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct globalfifo_dev * dev = filp->private_data;
    switch(cmd){
    case FIFO_CLEAR :
        mutex_lock(&dev->mutex);
	memset(dev->mem,0,GLOBALFIFO_SIZE);
        dev->current_len=0;
        mutex_unlock(&dev->mutex);
	printk(KERN_INFO "globalfifo is set to zero\n");
	break;
    
    default:
	return -EINVAL;
    }
    return 0;
}

static ssize_t globalfifo_read(struct file * filp,char __user * buf,size_t size,loff_t * ppos)
{
    unsigned long p= * ppos;
    unsigned int count =size;
    int ret=0; 
    struct globalfifo_dev * dev = filp->private_data;
    if (p>= GLOBALFIFO_SIZE)
       return 0;
    if (count > GLOBALFIFO_SIZE -p )
          count = GLOBALFIFO_SIZE -p;

    DECLARE_WAITQUEUE(wait,current);               // 定义并初始化一个等待队列

    mutex_lock(&dev->mutex);
    
    add_wait_queue(&dev->r_wait,&wait);            // 将该等待队列加入到读等待队列头

    while(dev->current_len == 0){                  // 当前fifo中数据长度为0，不可读

          if(filp->f_flags & O_NONBLOCK){          // 若设置非阻塞标志，则返回设备忙
                 ret= -EAGAIN;
                 goto out;
          }
 
          __set_current_state(TASK_INTERRUPTIBLE); // 阻塞访问,设定当前状态为可中断睡眠（浅睡眠）
  
          mutex_unlock(&dev->mutex); 
          
          schedule();                              // 执行调度，让出CPU给其他进程

          if(signal_pending(current)){             // 若是信号使之唤醒，则返回错误
               ret= -ERESTARTSYS;
               goto out2;
          }
          mutex_lock(&dev->mutex);
    }

    if(count>dev->current_len)
        dev->current_len = count;

    if (copy_to_user(buf,dev->mem+p,count)){
       ret= -EINVAL;
    }else{
        *ppos +=count;
        dev->current_len -= count;
        ret=count;
        wake_up_interruptible(&dev->w_wait);       //读操作完成,让出一部分内存空间，唤醒由于写满导致的阻塞进程
        printk(KERN_INFO "read %u bytes(s) from %lu\n",count,p);

        if(dev->async_queue){                      //向用户进程发送SIGIO信号,指明设备文件可写

              kill_fasync(&dev->async_queue,SIGIO,POLL_OUT);    
              printk(KERN_DEBUG " %s kill SIGIO\n ",__func__);
        }

    }
out:    
        mutex_unlock(&dev->mutex);
out2:   
        remove_wait_queue(&dev->r_wait,&wait);
        set_current_state(TASK_RUNNING);
    return ret;
}

static ssize_t globalfifo_write(struct file * filp, const char __user *buf,size_t size , loff_t * ppos)
{
    unsigned long p=*ppos;
    unsigned int count = size ;
    int ret=0;
    struct globalfifo_dev *dev=filp->private_data;
    if (p>= GLOBALFIFO_SIZE)
       return 0;
    if (count > GLOBALFIFO_SIZE -p )
          count = GLOBALFIFO_SIZE -p;

    DECLARE_WAITQUEUE(wait,current);

    mutex_lock(&dev->mutex);

    add_wait_queue(&dev->w_wait,&wait);
    while(dev->current_len == GLOBALFIFO_SIZE){

          if(filp->f_flags & O_NONBLOCK){
                 ret= -EAGAIN;
                 goto out;
          }
 
          __set_current_state(TASK_INTERRUPTIBLE);
  
          mutex_unlock(&dev->mutex);
          
          schedule();

          if(signal_pending(current)){
               ret= -ERESTARTSYS;
               goto out2;
          }
          mutex_lock(&dev->mutex);
    }

    if(copy_from_user(dev->mem+p,buf,count)){
       ret= -EINVAL;
    }else{
	*ppos +=count;
        ret=count;
        dev->current_len += count;
        printk(KERN_INFO "written  %u bytes(s) from %lu\n",count,p);
        wake_up_interruptible(&dev->r_wait);                        //写操作完成，内存存在可读数据,唤醒由于读空导致的阻塞进程

        if(dev->async_queue){                                       //向用户进程发送SIGIO信号,指明设备文件可读

              kill_fasync( &dev->async_queue, SIGIO, POLL_IN);    
              printk(KERN_DEBUG " %s kill SIGIO\n ",__func__);
        }
             
    } 
out:    
        mutex_unlock(&dev->mutex);
out2:   
        remove_wait_queue(&dev->w_wait,&wait);
        set_current_state(TASK_RUNNING);
    return ret;
}

static loff_t globalfifo_llseek(struct file * filp, loff_t offset ,int orig)
{
   loff_t ret=0;
   switch(orig){
   case 0:
        if(offset<0){
            ret =-EINVAL;
            break;
        }
        if((unsigned int)offset >GLOBALFIFO_SIZE){
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
        if(filp->f_pos+offset >GLOBALFIFO_SIZE){
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

static unsigned int globalfifo_poll(struct file *filp,poll_table *wait )
{

       unsigned int mask=0;

       struct globalfifo_dev *dev=filp->private_data;     

       mutex_lock(&dev->mutex);

       poll_wait(filp,&dev->r_wait,wait);          //将本进程对应的等待列表加入读等待队列头
       poll_wait(filp,&dev->w_wait,wait);          //将本进程对应的等待列表加入写等待队列头

       if(dev->current_len != 0)                   // FIFO长度大于0,表明可读
             mask |= POLLIN |POLLRDNORM;            // 返回用户可读掩码
       if(dev->current_len != GLOBALFIFO_SIZE)     // FIFO长度小于GLOBALFIFO_SIZE,表明可写
             mask |= POLLOUT |POLLRDNORM;           // 返回用户可写掩码
       mutex_unlock(&dev->mutex);
       return mask;
}

static int globalfifo_fasync(int fd, struct file *filp,int mode){

         struct globalfifo_dev *dev=filp->private_data;

 
         return fasync_helper(fd,filp,mode,&dev->async_queue);    // 处理应用层中对设备文件设置FASYNC命令


}

static int globalfifo_relase(struct inode *inode,struct file *filp)
{
    
        globalfifo_fasync(-1,filp,0);
        return 0;
}

static const struct file_operations globalfifo_ops={
	.owner  = THIS_MODULE,
	.open   = globalfifo_open,
	.llseek = globalfifo_llseek,
	.read   = globalfifo_read,
	.write  = globalfifo_write,
	.release = globalfifo_relase,
	.unlocked_ioctl  = globalfifo_ioctl,
        .poll            = globalfifo_poll,
        .fasync          = globalfifo_fasync,
};


static void globalfifo_setup_cdev(struct globalfifo_dev * dev, int index){

    int err, devno = MKDEV(globalfifo_major,index);


    cdev_init(&dev->cdev,&globalfifo_ops);

	dev->cdev.owner=THIS_MODULE;

	err=cdev_add(&dev->cdev,devno,1);

    if(err<0)
	 	printk(KERN_NOTICE "ERROR %d adding globalfifo %d" ,err,index);

}



static int __init globalfifo_init(void){

	   dev_t devno;
       int   ret,i;
	   devno =MKDEV(globalfifo_major,0);                      

	   if(globalfifo_major)
	   	register_chrdev_region(devno,DEVICE_NUM,"globalfifo");      //手动注册设备号 连续编号的总数为DEVICE_NUM
	   else{
	   	alloc_chrdev_region(&devno,0,DEVICE_NUM,"globalfifo");      //自动注册设备号(系统自动分配设备号)
		globalfifo_major = MAJOR(devno);                   
	   }

           globalfifo_devp=kzalloc(sizeof(struct globalfifo_dev)*DEVICE_NUM,GFP_KERNEL);   // 为设备分配内存空间 单个设备大小*DEVICE_NUM
	   if(!globalfifo_devp){
		  ret = -ENOMEM;
		  goto fail_alloc; 
	   	}

           mutex_init(&globalfifo_devp->mutex);
           init_waitqueue_head(&globalfifo_devp->r_wait);                     // 初始化读阻塞等待队列头
           init_waitqueue_head(&globalfifo_devp->w_wait);                     // 初始化写阻塞等待队列头
           globalfifo_devp->current_len=0;
           for(i=0;i<DEVICE_NUM;i++)
	   	globalfifo_setup_cdev(globalfifo_devp+i,i);                   //为每个设备单独初始化并向系统添加各自的cdev，完成设备的注册                

	   return 0;
fail_alloc:
	   unregister_chrdev_region(devno,DEVICE_NUM);
	   return -1;
	   	
	
}


static void __exit globalfifo_exit(void){                                    //独立从系统中删除每个cdev设备，并回收设备号，完成设备卸载  
        int i;
        for(i=0;i<DEVICE_NUM;i++)                                                     
		cdev_del(&(globalfifo_devp+i)->cdev);	
	unregister_chrdev_region(MKDEV(globalfifo_major,0),DEVICE_NUM);      
	kfree(globalfifo_devp);
}

module_init(globalfifo_init);
module_exit(globalfifo_exit);


MODULE_AUTHOR("Jiaolong Tong<tongjiaolong@yeah.net>");

MODULE_LICENSE("GPL v2");
