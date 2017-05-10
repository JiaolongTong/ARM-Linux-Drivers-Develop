#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/platform_device.h>
//#include <mach/map.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <mach/gpio.h>
#include <plat/mux.h>
#include <linux/gpio.h>
#include "am335_platform_data.h"

static int gpio;

static int my_probe(struct platform_device * pdev){

        printk("Driver is probe in platform bus\n");
        int result;

/*
        result =gpio_direction_output(GPIO_TO_PIN(3,18),1);
        if(result !=0)
             printk("gpio_directiom(3_18 failed!\n)\n");

        gpio_set_value(GPIO_TO_PIN(3,18), 1);
*/
 
	struct am335_platform_data *pdata = pdev->dev.platform_data;
     
        gpio=pdata->gpio;

        result =gpio_direction_output(gpio,1);
        if(result !=0)
             printk("gpio_directiom(3_18 failed!\n)\n");

        gpio_set_value(gpio, 1);

/*
        struct resource *res;
        res = platform_get_resource(pdev,IORESOURCE_MEM,0);
        size = resource_size(res);
        demo_mem=request_mem_region(res->start,size,pdev->name);
        if(demo_base ==NULL){
             printk("failed to get memory region\n");
             return -EINVAL;
         }
         demo_base =ioremap(res->start,size);
         if(demo_base ==NULL){
            printk("failed to ioremap() region \n");
            ret =-EINVAL;
            goto err_req;
         }
         gpfcon = readl(demo_base);
         gpfcon = (gpfcon & (0xff << 8) | (0x55<<8));
         writel(gpfcon,demo_base);

         gpfdata = readl(demo_base+4);
         gpfdata = gpfdata & (0xf<< 4);
         writel(gpfdata,demo_base+4);

err_req:
         release_resource(demo_mem);
         kfree(demo_mem);
*/
         return 0;

}

static int my_remove(struct platform_device * dev){

          printk("Driver found device unpluged!\n");
          gpio_set_value(gpio, 0);
          
          return 0;

}

static struct platform_driver my_driver ={
  .probe = my_probe,
  .remove = my_remove,
  .driver ={
       .owner =THIS_MODULE,
       .name  ="am335-demo",
  },
};

static int __init my_driver_init(void){

      return platform_driver_register(&my_driver);
}

static void __exit my_driver_exit(void){

       platform_driver_unregister(&my_driver);
}

module_init(my_driver_init);
module_exit(my_driver_exit);


MODULE_LICENSE("GPL");


