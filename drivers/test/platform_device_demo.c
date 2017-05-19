#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include "am335_platform_data.h"



struct resource am335_demo_source[] ={
 [0] ={
         .start = 0x56000050,
         .end   = 0x56000054,
         .flags =IORESOURCE_MEM,
 }
};
struct am335_platform_data am335_demo_data[] ={

        {
          .gpio =GPIO_TO_PIN(3,18),
        },
        {
          .gpio =GPIO_TO_PIN(0,22),
        },
};
struct platform_device am335_device_demo ={

       .name = "am335-demo",
       .id   = -1,
       .num_resources=ARRAY_SIZE(am335_demo_source),
       .resource =am335_demo_source,
       .dev ={
           .platform_data =&am335_demo_data,
       },
};


static int __init platform_device_init(void){

        int ret =0;
        ret = platform_device_register(&am335_device_demo);
        if(ret)
            printk("platform_device_register failed!\n");
       
}

static void __exit platform_device_exit(void){

       platform_device_unregister(&am335_device_demo);
}

module_init(platform_device_init);
module_exit(platform_device_exit);

MODULE_LICENSE("GPL");


