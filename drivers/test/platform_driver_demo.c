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
#include <linux/interrupt.h>
#include "am335_platform_data.h"

static int gpio_led;
static int gpio_led_value;
static int gpio_key;
static int timer_debounce;
static int debounce_interval=1;
static struct timer_list *timer;
static irqreturn_t gpio_keys_isr(int irq, void *dev_id)
{

	BUG_ON(irq != gpio_to_irq(gpio_key));

	if (timer_debounce)
		mod_timer(timer,jiffies + msecs_to_jiffies(timer_debounce));
	else{
                if (gpio_led_value == 0)
		    gpio_set_value(gpio_led,1);
                else
                    gpio_set_value(gpio_led,0);
        }
	return IRQ_HANDLED;
}

static int my_probe(struct platform_device * pdev){

        printk("Driver is probe in platform bus\n");
        int error,irq;
        unsigned long irqflags;

	struct am335_platform_data *pdata = pdev->dev.platform_data;
     
        gpio_led=GPIO_TO_PIN(3,18);
        //gpio_key=GPIO_TO_PIN(0,22);
        error =gpio_direction_output(gpio_led,1);
        if(error !=0)
             printk("gpio_directiom(3_18 failed!\n)\n");
        gpio_set_value(gpio_led, 1);
/*
        error =gpio_direction_output(gpio_key,0);
        if(error !=0)
             printk("gpio_directiom(0_22 failed!\n)\n");

        

        error = gpio_set_debounce(gpio_key,debounce_interval *1000);
	if (error < 0)
	    timer_debounce = debounce_interval;

        irq = gpio_to_irq(gpio_key);
	if (irq < 0) 
	    printk("Unable to get irq number for GPIO %d\n",gpio_key);

	irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING ;
        error = request_threaded_irq(irq, NULL, gpio_keys_isr, irqflags, "key_led", NULL);
	if (error < 0) 
	    printk("Unable to claim irq %d\n",irq);
*/
         return 0;

}

static int my_remove(struct platform_device * dev){

          printk("Driver found device unpluged!\n");
          gpio_set_value(gpio_led, 0);
          //free_irq(irq, &ddata->data[i]);
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


