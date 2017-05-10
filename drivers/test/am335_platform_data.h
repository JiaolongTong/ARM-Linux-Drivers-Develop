#ifndef AM335_PLATFORM_DATA
#define AM335_PLATFORM_DATA

#define GPIO_TO_PIN(bank,gpio) (32*(bank) + (gpio))
struct am335_platform_data {
    int  gpio;
};

#endif
