/*
 * Code for AM335X EVM.
 *
 * Copyright (C) 2011 Texas Instruments, Inc. - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/i2c/at24.h>
#include <linux/phy.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/gpio_keys.h>
#include <linux/modules_exints.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/wl12xx.h>
#include <linux/ethtool.h>
#include <linux/mfd/tps65910.h>
#include <linux/mfd/tps65217.h>
#include <linux/pwm_backlight.h>
#include <linux/platform_data/ti_adc.h>
#include <linux/mfd/ti_tscadc.h>
#include <linux/input/ti_tsc.h>

#include <linux/reboot.h>
#include <linux/pwm/pwm.h>
#include <linux/opp.h>

/* LCD controller is similar to DA850 */
#include <video/da8xx-fb.h>

#include <mach/hardware.h>
#include <mach/board-am335xevm.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/hardware/asp.h>

#include <plat/omap_device.h>
#include <plat/irqs.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/lcdc.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/emif.h>
#include <plat/nand.h>

#include "board-flash.h"
#include "cpuidle33xx.h"
#include "mux.h"
#include "devices.h"
#include "hsmmc.h"

/* Add by JBO for support sc16is752 */
#include <linux/sc16is7x2.h>

/* Convert GPIO signal to GPIO pin number */
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

/* AM335X EVM Phy ID and Debug Registers */
#define AM335X_EVM_PHY_ID		0x4dd072
#define AM335X_EVM_PHY_MASK		0xfffffffe
#define AR8051_PHY_DEBUG_ADDR_REG	0x1d
#define AR8051_PHY_DEBUG_DATA_REG	0x1e
#define AR8051_DEBUG_RGMII_CLK_DLY_REG	0x5
#define AR8051_RGMII_TX_CLK_DLY		BIT(8)

/* LCD backlight platform Data */
#define AM335X_BACKLIGHT_MAX_BRIGHTNESS        100
#define AM335X_BACKLIGHT_DEFAULT_BRIGHTNESS    80
#define AM335X_PWM_PERIOD_NANO_SECONDS        (1000 * 50)/* increase to 20KHz */

static struct platform_pwm_backlight_data am335x_backlight_data = {
	.pwm_id         = "ehrpwm.0:0",
	.ch             = -1,
	.lth_brightness	= 21,
	.max_brightness = AM335X_BACKLIGHT_MAX_BRIGHTNESS,
	.dft_brightness = AM335X_BACKLIGHT_DEFAULT_BRIGHTNESS,
	.pwm_period_ns  = AM335X_PWM_PERIOD_NANO_SECONDS,
};

static char lcd_type[11];
static int __init lcd_type_init(char* s) {
	printk("----lcd_type_init %s\n", s);
        strncpy(lcd_type, s, 11);
        return 0;
}
__setup("dispmode=", lcd_type_init);

/* Add by JBO for handling uart2 setting */
static bool uart2_to_uart5hwc = false;
static void __init uart2_setup(char *s) {
	if (s[0] == '1')
		uart2_to_uart5hwc = true;
}
__setup("uart2_to_uart5hwc=", uart2_setup);

/* Add by JBO for handling uart4 setting */
static bool uart4_to_can1 = false;                    //modfiy by tong
static void __init uart4_setup(char *s) {
	if (s[0] == '0')
		uart4_to_can1 = false;
}
__setup("uart4_to_can1=", uart4_setup);

static char* display_mode = "hdmi480p";
module_param(display_mode, charp, S_IRUGO);

/* Modified by Conway. Added  'lcd7ir-k' and 'lcd7ic-k' */
#define NUM_OF_LCDMODE 13
enum  display_num{  lcd4i3  , lcd7i , lcd7ir  ,lcd7ir_k  , lcd7ic,  lcd7ic_k, vga  ,  lvds  ,  hdmi640x480  ,
                             hdmi480p   ,  hdmi1024x768  ,  hdmi720p  ,  hdmi1080i  };
const char *display_num[]={ "lcd4i3" , "lcd7i" , "lcd7ir" ,"lcd7ir-k" , "lcd7ic","lcd7ic-k", "vga" , "lvds" , "hdmi640x480" ,  "hdmi480p"  , "hdmi1024x768" , "hdmi720p" , "hdmi1080i" };

static const struct display_panel disp_panel = {
	WVGA,
	32,
	16,
	COLOR_ACTIVE,
};

static const struct display_panel dvi_panel = {
	WVGA,
	16,
	16,
	COLOR_ACTIVE,
};

static struct lcd_ctrl_config lcd_cfg = {
	&disp_panel,
	.ac_bias		= 255,
	.ac_bias_intrpt		= 0,
	.dma_burst_sz		= 16,
	.bpp			= 16,
	.fdd			= 0x80,
	.tft_alt_mode		= 0,
	.stn_565_mode		= 0,
	.mono_8bit_mode		= 0,
	.invert_line_clock	= 1,
	.invert_frm_clock	= 1,
	.sync_edge		= 0,
	.sync_ctrl		= 1,
	.raster_order		= 0,
};

static struct lcd_ctrl_config dvi_cfg = {
	&dvi_panel,
	.ac_bias    = 255,
	.ac_bias_intrpt    = 0,
	.dma_burst_sz    = 16,
	.bpp      = 16,
	.fdd      = 0x80,
	.tft_alt_mode    = 0,
	.stn_565_mode    = 0,
	.mono_8bit_mode    = 0,
	.invert_line_clock  = 1,
	.invert_frm_clock  = 1,
	.sync_edge    = 0,
	.sync_ctrl    = 1,
	.raster_order    = 0,
};


struct da8xx_lcdc_platform_data	am335x_lcdc_pdata[] = {
	{
		.manu_name		= "InnoLux",
		.controller_data	= &lcd_cfg,
		.type			= "4.3inch_LCD",	
	},{
		.manu_name		= "InnoLux",
		.controller_data	= &lcd_cfg,
		.type			= "7inch_LCD_RES",
	},{
		.manu_name		= "InnoLux",
		.controller_data	= &lcd_cfg,
		.type			= "7inch_LCD_RES",
	},{
		.manu_name		= "InnoLux",
		.controller_data	= &lcd_cfg,
		.type			= "7inch_LCD_RES",
	},{
		.manu_name		= "InnoLux",
		.controller_data	= &lcd_cfg,
		.type			= "7inch_LCD_CAP",		
	},{
		.manu_name		= "InnoLux",
		.controller_data	= &lcd_cfg,
		.type			= "7inch_LCD_CAP",		
	},{
		.manu_name		= "InnoLux",
		.controller_data	= &lcd_cfg,
		.type			= "VGA",		
	},{
		.manu_name		= "InnoLux",
		.controller_data	= &lcd_cfg,
		.type			= "LVDS",	
	},{
		.manu_name    		= "NXP HDMI",
		.controller_data  	= &dvi_cfg,
		.type			= "nxp-640x480@60",
	},{
		.manu_name    		= "NXP HDMI",
		.controller_data  	= &dvi_cfg,
		.type			= "nxp-720x480@60",
	},{
		.manu_name    		= "NXP HDMI",
		.controller_data  	= &dvi_cfg,
		.type			= "1024x768@60",
	},{
		.manu_name    		= "NXP HDMI",
		.controller_data  	= &dvi_cfg,
		.type			= "nxp-1280x720@60",
	},{
		.manu_name    		= "NXP HDMI",
		.controller_data  	= &dvi_cfg,
		.type			= "nxp-1920x1080@24",
	},
};




struct da8xx_lcdc_platform_data *myd_am335x_def_pdata ;



#include "common.h"

#include <linux/lis3lv02d.h>

/* TSc controller */        //触摸屏接口
static struct tsc_data am335x_touchscreen_data  = {
	.wires  = 4,
	.x_plate_resistance = 200,
	.steps_to_configure = 5,
};

static struct adc_data am335x_adc_data = {
	.adc_channels = 4,
};

static struct mfd_tscadc_board tscadc = {
	.tsc_init = &am335x_touchscreen_data,
	.adc_init = &am335x_adc_data,
};

static u8 am335x_iis_serializer_direction0[] = {
	INACTIVE_MODE,	TX_MODE,		RX_MODE,		INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
};

static struct snd_platform_data am335x_snd_data0 = {
	.tx_dma_offset	= 0x46000000,	/* McASP0 */
	.rx_dma_offset	= 0x46000000,
	.op_mode	= DAVINCI_MCASP_IIS_MODE,
	.num_serializer	= ARRAY_SIZE(am335x_iis_serializer_direction0),
	.tdm_slots	= 2,
	.serial_dir	= am335x_iis_serializer_direction0,
	.asp_chan_q	= EVENTQ_2,
	.version	= MCASP_VERSION_3,
	.txnumevt	= 1,
};


static struct omap2_hsmmc_info am335x_mmc[] __initdata = {
	{
		.mmc            = 1,
		.caps           = MMC_CAP_4_BIT_DATA,
//		.gpio_cd        = GPIO_TO_PIN(3, 21),
        .gpio_cd        = GPIO_TO_PIN(3, 19),                  //MMC0_CD
		.gpio_wp        = -EINVAL,
		.ocr_mask       = MMC_VDD_32_33 | MMC_VDD_33_34, /* 3V3 */
	},
	{
		.mmc            = 0,	/* will be set at runtime */
	},
	{
		.mmc            = 0,	/* will be set at runtime */
	},
	{}      /* Terminator */
};


#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/*
	 * Setting SYSBOOT[5] should set xdma_event_intr0 pin to mode 3 thereby
	 * allowing clkout1 to be available on xdma_event_intr0.
	 * However, on some boards (like EVM-SK), SYSBOOT[5] isn't properly
	 * latched.
	 * To be extra cautious, setup the pin-mux manually.
	 * If any modules/usecase requries it in different mode, then subsequent
	 * module init call will change the mux accordingly.
	 */
	AM33XX_MUX(XDMA_EVENT_INTR0, OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT),
	AM33XX_MUX(I2C0_SDA, OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	AM33XX_MUX(I2C0_SCL, OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define	board_mux	NULL
#endif

/* module pin mux structure */
struct pinmux_config {
	const char *string_name; /* signal name format */
	int val; /* Options for the mux register value */
};

struct evm_dev_cfg {
	void (*device_init)(int evm_id, int profile);

/*
* If the device is required on both baseboard & daughter board (ex i2c),
* specify DEV_ON_BASEBOARD
*/
#define DEV_ON_BASEBOARD	0
#define DEV_ON_DGHTR_BRD	1
	u32 device_on;

	u32 profile;	/* Profiles (0-7) in which the module is present */
};

/* AM335X - CPLD Register Offsets */
#define	CPLD_DEVICE_HDR	0x00 /* CPLD Header */
#define	CPLD_DEVICE_ID	0x04 /* CPLD identification */
#define	CPLD_DEVICE_REV	0x0C /* Revision of the CPLD code */
#define	CPLD_CFG_REG	0x10 /* Configuration Register */

//static struct i2c_client *cpld_client;
//static u32 am335x_evm_id;
static struct omap_board_config_kernel am335x_evm_config[] __initdata = {
};

static bool daughter_brd_detected;

#define EEPROM_MAC_ADDRESS_OFFSET	60 /* 4+8+4+12+32 */
#define EEPROM_NO_OF_MAC_ADDR		3
//static char am335x_mac_addr[EEPROM_NO_OF_MAC_ADDR][ETH_ALEN];

#define AM335X_EEPROM_HEADER		0xEE3355AA

static int am33xx_evmid = -EINVAL;

/*
* am335x_evm_set_id - set up board evmid
* @evmid - evm id which needs to be configured
*
* This function is called to configure board evm id.
*/
void am335x_evm_set_id(unsigned int evmid)
{
	am33xx_evmid = evmid;
	return;
}

/*
* am335x_evm_get_id - returns Board Type (EVM/BB/EVM-SK ...)
*
* Note:
*	returns -EINVAL if Board detection hasn't happened yet.
*/
int am335x_evm_get_id(void)
{
	return am33xx_evmid;
}
EXPORT_SYMBOL(am335x_evm_get_id);


/* Module pin mux for LCDC */                      //LCD 必须
static struct pinmux_config lcdc_pin_mux[] = {
	{"lcd_data0.lcd_data0",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data1.lcd_data1",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data2.lcd_data2",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data3.lcd_data3",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data4.lcd_data4",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data5.lcd_data5",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data6.lcd_data6",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data7.lcd_data7",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data8.lcd_data8",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data9.lcd_data9",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data10.lcd_data10",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data11.lcd_data11",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data12.lcd_data12",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data13.lcd_data13",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data14.lcd_data14",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data15.lcd_data15",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_vsync.lcd_vsync",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_hsync.lcd_hsync",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_pclk.lcd_pclk",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_ac_bias_en.lcd_ac_bias_en", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

static struct pinmux_config tsc_pin_mux[] = {    // 触摸屏 必须
	{"ain0.ain0",           OMAP_MUX_MODE0 | AM33XX_INPUT_EN},
	{"ain1.ain1",           OMAP_MUX_MODE0 | AM33XX_INPUT_EN},
	{"ain2.ain2",           OMAP_MUX_MODE0 | AM33XX_INPUT_EN},
	{"ain3.ain3",           OMAP_MUX_MODE0 | AM33XX_INPUT_EN},
	{"vrefp.vrefp",         OMAP_MUX_MODE0 | AM33XX_INPUT_EN},
	{"vrefn.vrefn",         OMAP_MUX_MODE0 | AM33XX_INPUT_EN},
	{NULL, 0},
};

/* Pin mux for nand flash module */               //NAND flash 必须
static struct pinmux_config nand_pin_mux[] = {
	{"gpmc_ad0.gpmc_ad0",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad1.gpmc_ad1",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad2.gpmc_ad2",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad3.gpmc_ad3",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad4.gpmc_ad4",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad5.gpmc_ad5",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad6.gpmc_ad6",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad7.gpmc_ad7",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_wait0.gpmc_wait0", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_wpn.gpmc_wpn",	  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn0.gpmc_csn0",	  OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_advn_ale.gpmc_advn_ale",  OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_oen_ren.gpmc_oen_ren",	 OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_wen.gpmc_wen",     OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_ben0_cle.gpmc_ben0_cle",	 OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{NULL, 0},
};

/* Module pin mux for rgmii1 */
static struct pinmux_config rgmii1_pin_mux[] = {   //联网网口   必须
	{"mii1_txen.rgmii1_tctl", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"mii1_rxdv.rgmii1_rctl", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_txd3.rgmii1_td3", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"mii1_txd2.rgmii1_td2", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"mii1_txd1.rgmii1_td1", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"mii1_txd0.rgmii1_td0", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"mii1_txclk.rgmii1_tclk", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"mii1_rxclk.rgmii1_rclk", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd3.rgmii1_rd3", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd2.rgmii1_rd2", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd1.rgmii1_rd1", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd0.rgmii1_rd0", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mdio_data.mdio_data", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mdio_clk.mdio_clk", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};

/* Module pin mux for rgmii2 */                   //OTDR 网口 必须
static struct pinmux_config rgmii2_pin_mux[] = {
	{"gpmc_a0.rgmii2_tctl", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"gpmc_a1.rgmii2_rctl", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a2.rgmii2_td3", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"gpmc_a3.rgmii2_td2", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"gpmc_a4.rgmii2_td1", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"gpmc_a5.rgmii2_td0", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"gpmc_a6.rgmii2_tclk", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"gpmc_a7.rgmii2_rclk", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a8.rgmii2_rd3", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a9.rgmii2_rd2", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a10.rgmii2_rd1", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a11.rgmii2_rd0", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mdio_data.mdio_data", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mdio_clk.mdio_clk", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};

static struct pinmux_config i2c0_pin_mux[] = {    //第一路 IIC    必须
        {"i2c0_sda.i2c0_sda",    OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
                                        AM33XX_PULL_ENBL | AM33XX_PULL_UP | AM33XX_INPUT_EN},
        {"i2c0_scl.i2c0_scl",   OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
                                        AM33XX_PULL_ENBL | AM33XX_PULL_UP | AM33XX_INPUT_EN},
        {NULL, 0},
};

static struct pinmux_config i2c1_pin_mux[] = {   //第二路 IIC 必须
	{"spi0_d1.i2c1_sda",    OMAP_MUX_MODE2 | AM33XX_SLEWCTRL_SLOW |
             	            AM33XX_PULL_ENBL | AM33XX_INPUT_EN | AM33XX_PULL_UP},
	{"spi0_cs0.i2c1_scl",   OMAP_MUX_MODE2 | AM33XX_SLEWCTRL_SLOW |
	                        AM33XX_PULL_ENBL | AM33XX_INPUT_EN | AM33XX_PULL_UP},
	{NULL, 0},
};

/* Add by JBO */
static struct pinmux_config i2c_gpio_pin_mux[] = {   
	{"spi0_d1.gpio0_4", OMAP_MUX_MODE7 | AM33XX_INPUT_EN},
	{"spi0_cs0.gpio0_5", OMAP_MUX_MODE7 | AM33XX_INPUT_EN},
	{NULL, 0 },
};

/* Module pin mux for mcasp0 */
/*
static struct pinmux_config mcasp0_pin_mux[] = {  //复通道音频接入接口 
        {"mcasp0_aclkx.mcasp0_aclkx", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
        {"mcasp0_ahclkx.mcasp0_ahclkx", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},

        {"mcasp0_fsx.mcasp0_fsx", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
		{"mcasp0_ahclkr.mcasp0_axr2", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
        {"mcasp0_axr1.mcasp0_axr1", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
        {NULL, 0},
};
*/

/* Module pin mux for mmc0 */
static struct pinmux_config mmc0_common_pin_mux[] = {  //SD卡  必须 
	{"mmc0_dat3.mmc0_dat3",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat2.mmc0_dat2",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat1.mmc0_dat1",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat0.mmc0_dat0",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_clk.mmc0_clk",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_cmd.mmc0_cmd",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};


static struct pinmux_config mmc0_cd_only_pin_mux[] = {
//	{"mcasp0_ahclkx.gpio3_21",  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{"mcasp0_fsr.gpio3_19",  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

/* pinmux for gpio based key  Add by Tong Jiaolong*/
static struct pinmux_config gpio_keys_pin_mux[] = {     //外部中断   必须
	{"gpmc_ad8.gpio0_22", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"gpmc_ad9.gpio0_23", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"gpmc_ad15.gpio1_15", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"gpmc_ad14.gpio1_14", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"gpmc_ad13.gpio1_13", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
        {"gpmc_ad12.gpio1_12", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
        {"gpmc_csn3.gpio2_0", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
        {"gpmc_clk.gpio2_1", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{NULL, 0},
};

/* pinmux for led device Add by Tong Jiaolong */
static struct pinmux_config gpio_led_mux[] = {        //GPIO电平控制 必须
	{"mcasp0_aclkr.gpio3_18",  OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
        {"spi0_d0.gpio0_3",  OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
        {"mcasp0_ahclkr.gpio3_17",  OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},             //RS485->D
        //{"mcasp0_fsx.gpio3_15" ,OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
        //{"gpmc_ad11.gpio0_27",  OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

/* pinmux for watch dog timer input, MYIR */     //必须
static struct pinmux_config gpio_wdi_mux[] = {
        {"emu1.gpio3_8",  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
        {NULL, 0},
};

/* pinmux for e2pwp, MYIR */ //for Jtag          //必须
static struct pinmux_config gpio_e2pwp_mux[] = {
        {"emu0.gpio3_7",  OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
        {NULL, 0},
};

/* pinmux for LCD capacitive TP INT
       myd-am335x-y - spi0_cs1.gpio0_6
	       myd-am335x-j - xdma_event_intr1.gpio0_20
*/
static struct pinmux_config gpio_tpint_mux[] = {      //LCD DMA中断 必须
	{"xdma_event_intr1.gpio0_20",  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
    {NULL, 0},
};

/*
* @pin_mux - single module pin-mux structure which defines pin-mux
*			details for all its pins.
*/
static void setup_pin_mux(struct pinmux_config *pin_mux)
{
	int i;

	for (i = 0; pin_mux->string_name != NULL; pin_mux++)
		omap_mux_init_signal(pin_mux->string_name, pin_mux->val);

}

/*
* @evm_id - evm id which needs to be configured
* @dev_cfg - single evm structure which includes
*				all module inits, pin-mux defines
* @profile - if present, else PROFILE_NONE
* @dghtr_brd_flg - Whether Daughter board is present or not
*/
static void _configure_device(int evm_id, struct evm_dev_cfg *dev_cfg,
	int profile)
{
	int i;

	am335x_evm_set_id(evm_id);

	/*
	* Only General Purpose & Industrial Auto Motro Control
	* EVM has profiles. So check if this evm has profile.
	* If not, ignore the profile comparison
	*/

	/*
	* If the device is on baseboard, directly configure it. Else (device on
	* Daughter board), check if the daughter card is detected.
	*/
	if (profile == PROFILE_NONE) {
		for (i = 0; dev_cfg->device_init != NULL; dev_cfg++) {
			if (dev_cfg->device_on == DEV_ON_BASEBOARD)
				dev_cfg->device_init(evm_id, profile);
			else if (daughter_brd_detected == true)
				dev_cfg->device_init(evm_id, profile);
		}
	} else {
		for (i = 0; dev_cfg->device_init != NULL; dev_cfg++) {
			if (dev_cfg->profile & profile) {
				if (dev_cfg->device_on == DEV_ON_BASEBOARD)
					dev_cfg->device_init(evm_id, profile);
				else if (daughter_brd_detected == true)
					dev_cfg->device_init(evm_id, profile);
			}
		}
	}
}


/* pinmux for usb0 drvvbus */
static struct pinmux_config usb0_pin_mux[] = {              //USB0_DRVVBUS  必须
	{"usb0_drvvbus.usb0_drvvbus",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

/* pinmux for usb1 drvvbus */
static struct pinmux_config usb1_pin_mux[] = {             //USB1_DRVVBUS  必须
	{"usb1_drvvbus.usb1_drvvbus",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};


/* Module pin mux for ehrpwm0 */
static struct pinmux_config ehrpwm0_pin_mux[] = {         //PWM 背光  必须
		{"spi0_sclk.ehrpwm0A", OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT},
			{NULL, 0},
};

/* Module pin mux for uart1 */
static struct pinmux_config uart1_pin_mux[] = {           //串口1  必须
        {"uart1_rxd.uart1_rxd", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
        {"uart1_txd.uart1_txd", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL},
		{"uart1_ctsn.uart1_ctsn", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
		{"uart1_rtsn.uart1_rtsn", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT_PULLUP},
        {NULL, 0},
};
/* Module pin mux for uart2 */                           //串口2  必须
static struct pinmux_config uart2_pin_mux[] = {
        {"mii1_crs.uart2_rxd", OMAP_MUX_MODE6 | AM33XX_PIN_INPUT_PULLUP},
        {"mii1_rxerr.uart2_txd", OMAP_MUX_MODE6 | AM33XX_PULL_ENBL},
        {NULL, 0},
};
/* Module pin mux for uart2 to uart5 hw control -- Add by JBO */      //串口2  串口5  选择引脚 
static struct pinmux_config uart2_to_uart5hwc_pin_mux[] = {
	{"mii1_crs.uart5_ctsn", OMAP_MUX_MODE5 | AM33XX_PIN_INPUT_PULLUP},
	{"mii_rxerr.uart5_rtsn", OMAP_MUX_MODE5 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};

/* Module pin mux for uart3 */                            //串口3  必须
static struct pinmux_config uart3_pin_mux[] = {
        {"spi0_cs1.uart3_rxd", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
        {"ecap0_in_pwm0_out.uart3_txd", OMAP_MUX_MODE1 | AM33XX_PULL_ENBL},
        {NULL, 0},
};

/* Module pin mux for uart4 to can1 -- Add by JBO */       //CAN总线
/*
static struct pinmux_config can1_pin_mux[] = {
	{"uart0_ctsn.d_can1_tx", OMAP_MUX_MODE2 | AM33XX_PULL_ENBL},
	{"uart0_rtsn.d_can1_rx", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};
*/
/* Module pin mux for uart4 -- Add by JBO */              //串口4   必须
static struct pinmux_config uart4_pin_mux[] = {
	{"uart0_ctsn.uart4_rxd", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP },
	{"uart0_rtsn.uart4_txd", OMAP_MUX_MODE1 | AM33XX_PULL_ENBL },
	{NULL, 0},
};

/* Module pin mux for uart5 */                            //串口5   必须
static struct pinmux_config uart5_pin_mux[] = {
	{"mii1_col.uart5_rxd", OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLUP},
	{"rmii1_refclk.uart5_txd", OMAP_MUX_MODE3 | AM33XX_PULL_ENBL},
	{NULL, 0},
};

/* Pin mux for sc16is7x2 -- Add by JBO */
static struct pinmux_config sc16is7x2_pin_mux[] = {
	{"gpmc_ad10.gpio0_26", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP},
	{"gpmc_ad11.gpio0_27", OMAP_MUX_MODE7 | AM33XX_INPUT_EN},
	{NULL, 0 },
};
                    
#define AM335XEVM_WLAN_PMENA_GPIO	GPIO_TO_PIN(1, 30)            //WLAN 必须
#define AM335XEVM_WLAN_IRQ_GPIO		GPIO_TO_PIN(3, 17)
#define AM335XEVM_SK_WLAN_IRQ_GPIO      GPIO_TO_PIN(0, 31)

struct wl12xx_platform_data am335xevm_wlan_data = {
	.irq = OMAP_GPIO_IRQ(AM335XEVM_WLAN_IRQ_GPIO),
	.board_ref_clock = WL12XX_REFCLOCK_38_XTAL, /* 38.4Mhz */
	.bt_enable_gpio = GPIO_TO_PIN(3, 21),
	.wlan_enable_gpio = GPIO_TO_PIN(1, 16),
};


static bool backlight_enable;

static void enable_ehrpwm0(int evm_id, int profile)
{
	backlight_enable = true;
	setup_pin_mux(ehrpwm0_pin_mux);
}

/* Setup pwm-backlight */
static struct platform_device am335x_backlight = {
	.name           = "pwm-backlight",
	.id             = -1,
	.dev		= {
		.platform_data = &am335x_backlight_data,
	},
};

static struct pwmss_platform_data  pwm_pdata[3] = {
	{
		.version = PWM_VERSION_1,
/*		.chan_attrib[0].inverse_pol = 1,// for pwm backlight ctrl */
	},{
		.version = PWM_VERSION_1,
	},{
		.version = PWM_VERSION_1,
	},
};

static int __init backlight_init(void)
{
	int status = 0;

	if (backlight_enable) {
		/*
		 * Invert polarity of PWM wave from ECAP to handle
		 * backlight intensity to pwm brightness
		 */

		am33xx_register_ehrpwm(0, &pwm_pdata[0]);
		platform_device_register(&am335x_backlight);
	}
	return status;
}
late_initcall(backlight_init);

static int __init conf_disp_pll(int rate)
{
	struct clk *disp_pll;
	int ret = -EINVAL;

	disp_pll = clk_get(NULL, "dpll_disp_ck");
	if (IS_ERR(disp_pll)) {
		pr_err("Cannot clk_get disp_pll\n");
		goto out;
	}

	ret = clk_set_rate(disp_pll, rate);
	clk_put(disp_pll);
out:
	return ret;
}



static void display_init(int evm_id, int profile)
{
	int i;
	for(i=0; i<NUM_OF_LCDMODE; i++){
		if( strcmp(display_mode ,display_num[i])==0){
			printk(KERN_ERR"--- display_mode: %s\n", display_mode);
			break;
		}
	}
	
	setup_pin_mux(lcdc_pin_mux);
	switch(i){
		case hdmi640x480:
		case hdmi480p:
		case hdmi1024x768:
		case hdmi720p:
		case hdmi1080i:
			if (conf_disp_pll(371000000)) {
				pr_info("Failed to set pixclock to 371000000, not attempting to"
						"register DVI adapter\n");
				return;
			}
			break;
		case lcd4i3:
		case lcd7i:
		case lcd7ir:
		case lcd7ic:
		/* Modified by Conway. Added  'lcd7ir-k' and 'lcd7ic-k' */
		case lcd7ir_k:
		case lcd7ic_k:
		case vga:
		case lvds:
		default:
			if (conf_disp_pll(300000000)) {
				pr_info("Failed configure display PLL, not attempting to"
						"register LCDC\n");
				return;
			}
			break;
	}
	switch(i)
	{
		case lcd4i3:
		case lcd7i:
		case lcd7ir:
		case lcd7ic:
		/* Modified by Conway. Added  'lcd7ir-k' and 'lcd7ic-k' */
		case lcd7ir_k:
		case lcd7ic_k:
		case hdmi640x480:
		case hdmi720p:	
			myd_am335x_def_pdata=&am335x_lcdc_pdata[i];
			break;
		case hdmi480p:
		case hdmi1024x768:
		case hdmi1080i:
			myd_am335x_def_pdata=&am335x_lcdc_pdata[hdmi640x480];
			break;			
		case vga:
		case lvds:
		default:
			myd_am335x_def_pdata=&am335x_lcdc_pdata[lcd4i3];
			break;

	}

	if (am33xx_register_lcdc(myd_am335x_def_pdata))
		pr_info("Failed to register display adapter\n");

	pr_info("Setup display mode:%s complete\n",display_mode);
	return;

}

static void tsc_init(int evm_id, int profile)
{
	int err;

	setup_pin_mux(tsc_pin_mux);
	err = am33xx_register_tsc(&am335x_touchscreen_data);
	if (err)
		pr_err("failed to register touchscreen device\n");
}

static void mfd_tscadc_init(int evm_id, int profile)
{
	int err;

	err = am33xx_register_mfd_tscadc(&tscadc);
	if (err)
		pr_err("failed to register touchscreen device\n");
}

static void rgmii1_init(int evm_id, int profile)
{
	setup_pin_mux(rgmii1_pin_mux);
	return;
}

static void rgmii2_init(int evm_id, int profile)
{
	setup_pin_mux(rgmii2_pin_mux);
	return;
}

static void usb0_init(int evm_id, int profile)
{
	setup_pin_mux(usb0_pin_mux);
	return;
}

static void usb1_init(int evm_id, int profile)
{
	setup_pin_mux(usb1_pin_mux);
	return;
}


/* NAND partition information */                //NAND flash 分区表
static struct mtd_partition am335x_nand_partitions[] = {
/* All the partition sizes are listed in terms of NAND block size */
	{
		.name           = "SPL",
		.offset         = 0,			/* Offset = 0x0 */
		.size           = SZ_128K,
	},{
		.name           = "SPL.backup1",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x20000 */
		.size           = SZ_128K,
	},{
		.name           = "SPL.backup2",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x40000 */
		.size           = SZ_128K,
	},{
		.name           = "SPL.backup3",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x60000 */
		.size           = SZ_128K,
	},{
		.name           = "U-Boot",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x80000 */
		.size           = 15 * SZ_128K,
	},{
		.name           = "U-Boot Env",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x260000 */
		.size           = 1 * SZ_128K,
	},{
		.name           = "Kernel",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x280000 */
		.size           = 40 * SZ_128K,
	},{
		.name           = "File System",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x780000 */
		.size           = MTDPART_SIZ_FULL,
	},
};


static struct gpmc_timings am335x_nand_timings = {
	.sync_clk = 0,

	.cs_on = 0,
	.cs_rd_off = 44,
	.cs_wr_off = 44,

	.adv_on = 6,
	.adv_rd_off = 34,
	.adv_wr_off = 44,
	.we_off = 40,
	.oe_off = 54,

	.access = 64,
	.rd_cycle = 82,
	.wr_cycle = 82,

	.wr_access = 40,
	.wr_data_mux_bus = 0,
};

static void evm_nand_init(int evm_id, int profile)
{
	struct omap_nand_platform_data *pdata;
	struct gpmc_devices_info gpmc_device[2] = {
		{ NULL, 0 },
		{ NULL, 0 },
	};

	setup_pin_mux(nand_pin_mux);
	pdata = omap_nand_init(am335x_nand_partitions,
		ARRAY_SIZE(am335x_nand_partitions), 0, 0,
		&am335x_nand_timings);
	if (!pdata)
		return;
//	pdata->ecc_opt =OMAP_ECC_BCH8_CODE_HW;
	pdata->elm_used = true;
	gpmc_device[0].pdata = pdata;
	gpmc_device[0].flag = GPMC_DEVICE_NAND;

	omap_init_gpmc(gpmc_device, sizeof(gpmc_device));
	omap_init_elm();
}

/* Setup McASP 0   IIS*/
/*
static void mcasp0_init(int evm_id, int profile)
{
	/// Configure McASP 
	setup_pin_mux(mcasp0_pin_mux);
	am335x_register_mcasp(&am335x_snd_data0, 0);

	return;
}
*/


static void uart1_init(int evm_id, int profile)
{
        /* Configure Uart1*/
        setup_pin_mux(uart1_pin_mux);
}

static void uart2_init(int evm_id, int profile)
{
    /* Configure Uart2*/
	if (!uart2_to_uart5hwc) {
		setup_pin_mux(uart2_pin_mux);
	} else {
		setup_pin_mux(uart2_to_uart5hwc_pin_mux);
	}
	
    return;
}

static void uart3_init(int evm_id, int profile)
{
        /* Configure Uart3*/
        setup_pin_mux(uart3_pin_mux);
        return;
}

static void uart4_init(int evm_id, int profile)
{
	if (!uart4_to_can1)
		setup_pin_mux(uart4_pin_mux);
	return;
}

static void uart5_init(int evm_id, int profile)
{
	/* Configure Uart5 */
	setup_pin_mux(uart5_pin_mux);
	return;
}

/*
static void d_can_init(int evm_id, int profile)
{
	// Configure can1 
	if (uart4_to_can1) {
		setup_pin_mux(can1_pin_mux);
		// Instance One 
		am33xx_d_can_init(1);
	}
}
*/
static void mmc0_init(int evm_id, int profile)
{
	setup_pin_mux(mmc0_common_pin_mux);
	setup_pin_mux(mmc0_cd_only_pin_mux);

	omap2_hsmmc_init(am335x_mmc);
	return;
}
/***************************GPIO Keys****************************************
// Configure GPIOs for GPIO Keys 
static struct gpio_keys_button gpio_buttons[] = {
        {
                .code                   = 15,//KEY_MENU,
                .gpio                   = GPIO_TO_PIN(0, 22),
                .active_low             = true,
                .desc                   = "menu",
                .type                   = EV_KEY,
                .wakeup                 = 1,
        },
        {
                .code                   = 16,//KEY_BACK,
                .gpio                   = GPIO_TO_PIN(0, 23),
                .active_low             = true,
                .desc                   = "back",
                .type                   = EV_KEY,
                .wakeup                 = 1,
        },
};

static struct gpio_keys_platform_data am335x_evm_gpio_key_info = {
	.buttons        = gpio_buttons,
	.nbuttons       = ARRAY_SIZE(gpio_buttons),
};

static struct platform_device gpio_keys = {
	.name   = "gpio-keys",
	.id     = -1,
	.dev    = {
		.platform_data  = &am335x_evm_gpio_key_info,
	},
};

static void gpio_keys_init(int evm_id, int profile)
{
	int err;

	setup_pin_mux(gpio_keys_pin_mux);
	err = platform_device_register(&gpio_keys);
	if (err)
		pr_err("failed to register gpio key device\n");
}
*/
/**************************光开端组子单元模块************************************/
/*
      配置光开关子单元的外部中断，当异常产生时快速捕捉
资源	        核心板引脚	米尔底板标识	本底板标识	备注
子模块1中断	GPIO1_13	MMC2_DATA1	EXINT_1	
子模块2中断	GPIO2_0  	MMC2_CMD	EXINT_2	
子模块3中断	GPIO0_23	MMC2_DATA5	EXINT_3	
子模块4中断	GPIO1_15	MMC2_DATA3	EXINT_4	
子模块5中断	GPIO1_12	MMC2_DATA0	EXINT_5	
子模块6中断	GPIO2_1 	MMC2_CLKdd	EXINT_6	
子模块7中断	GPIO0_22	MMC2_DATA4	EXINT_7	
子模块8中断	GPIO1_14	MMC2_DATA2	EXINT_8	
*/
static struct modules_exints exints[] = {
        {
                .code                   = 15,//INT1,
                .gpio                   = GPIO_TO_PIN(1, 13),
                .active_low             = false,                        //常态为高电平，触发态为低电平
                .desc                   = "exint_1",
                .type                   = EV_KEY,                      //按键事件
                .wakeup                 = 1,                           //设置为唤醒引脚
        },
        {
                .code                   = 16,//INT2,
                .gpio                   = GPIO_TO_PIN(2, 0),
                .active_low             = false,
                .desc                   = "exint_2",
                .type                   = EV_KEY,
                .wakeup                 = 1,
        },
        {
                .code                   = 17,//INT3,
                .gpio                   = GPIO_TO_PIN(0, 23),               //have bug
                .active_low             = false,
                .desc                   = "exint_3",
                .type                   = EV_KEY,
                .wakeup                 = 1,
        },
        {
                .code                   = 18,//INT4,
                .gpio                   = GPIO_TO_PIN(1, 15),               //have bug
                .active_low             = false,
                .desc                   = "exint_4",
                .type                   = EV_KEY,
                .wakeup                 = 1,
        },
        {
                .code                   = 19,//INT5,
                .gpio                   = GPIO_TO_PIN(1, 12),
                .active_low             = false,
                .desc                   = "exint_5",
                .type                   = EV_KEY,
                .wakeup                 = 1,
        },
        {
                .code                   = 20,//INT6,
                .gpio                   = GPIO_TO_PIN(2, 1),
                .active_low             = false,
                .desc                   = "exint_6",
                .type                   = EV_KEY,
                .wakeup                 = 1,
        },
        {
                .code                   = 21,//INT7,
                .gpio                   = GPIO_TO_PIN(0, 22),
                .active_low             = false,
                .desc                   = "exint_7",
                .type                   = EV_KEY,
                .wakeup                 = 1,
        },
        {
                .code                   = 22,//INT8,
                .gpio                   = GPIO_TO_PIN(1, 14),
                .active_low             = false,
                .desc                   = "exint_8",
                .type                   = EV_KEY,
                .wakeup                 = 1,
        },
};


static struct modules_exints_platform_data am335x_evm_exits_info = {
	.exints        = exints,
	.nexints       = ARRAY_SIZE(exints),
};

static struct platform_device device_exints = {
	.name   = "gpio-exints",
	.id     = -1,
	.dev    = {
		.platform_data  = &am335x_evm_exits_info,
	},
};

static void modules_exints_init(int evm_id, int profile)
{
	int err;

	setup_pin_mux(gpio_keys_pin_mux);
	err = platform_device_register(&device_exints);
	if (err)
		pr_err("failed to register modules exints device\n");
}



/******************************************************************************/


static struct gpio_led gpio_leds[] = {
        {
                .name                   = "sys_led",
                .default_trigger        = "heartbeat",
                .gpio                   = GPIO_TO_PIN(3, 18),
        },
        {
                .name                   = "user_led0",
                .gpio                   = GPIO_TO_PIN(0, 3),
        },
        {
		.name                   = "GPIO_0",
		.gpio                   = GPIO_TO_PIN(0, 26),
        },
        {
		.name                   = "GPIO_1",
		.gpio                   = GPIO_TO_PIN(0, 27),
        },
        {
		.name                   = "GPRS_RELOAD",
		.gpio                   = GPIO_TO_PIN(1, 0),
        },
        {
		.name                   = "GPRS_RESET",
		.gpio                   = GPIO_TO_PIN(1, 1),
        },
        {
		.name                   = "PWRRGTON",
		.gpio                   = GPIO_TO_PIN(1, 2),
        },
        {
		.name                   = "RS485_TX_RX",
		.gpio                   = GPIO_TO_PIN(3, 17),           //RS485
        },
        {
		.name                   = "SELECT0",
		.gpio                   = GPIO_TO_PIN(2, 5),
        },
        {
		.name                   = "SELECT1",
		.gpio                   = GPIO_TO_PIN(2, 2),
        },
        {
		.name                   = "SELECT2",
		.gpio                   = GPIO_TO_PIN(2, 4),
        },
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};

static void gpio_led_init(int evm_id, int profile)
{
	int err;

	setup_pin_mux(gpio_led_mux);
	err = platform_device_register(&leds_gpio);
	if (err)
		pr_err("failed to register gpio led device\n");
}

/* MYIR gpio init */
static void myir_gpio_init(int evm_id, int profile)
{
	/* GPIO0_6 for capacitive TP INT pin */
	setup_pin_mux(gpio_tpint_mux);
	
#ifdef	CONFIG_MYIR_WDT
	/* gpio3_8, already configured at MLO */
	setup_pin_mux(gpio_wdi_mux);
#endif

	/* export gpio for eeprom wp pin, gpio3_7 */
	setup_pin_mux(gpio_e2pwp_mux);
    gpio_request(GPIO_TO_PIN(3, 7), "e2pwp");
    gpio_direction_output(GPIO_TO_PIN(3, 7), 1);
    gpio_export(GPIO_TO_PIN(3, 7), 0); /* direction may not changed */
}

#ifdef	CONFIG_MYIR_WDT
/* MYIR Watchdog device CAT823, GPIO3_8 is WDI on am335x-j */
static struct myir_wdt_platdata myir_wdt_data = {
    .default_period_ms = 200,
    .gpio_pin = GPIO_TO_PIN(3, 8),
};
static struct platform_device myir_wdt_device = {
    .name = "myir-watchdog",
    .id = -1,
    .dev = {
        .platform_data = &myir_wdt_data,
    },
};

static void myir_wdt_init(int evm_id, int profile)
{
	platform_device_register(&myir_wdt_device);
}
#endif /* CONFIG_MYIR_WDT */


static struct evm_dev_cfg myd_am335x_dev_cfg[] = {
	{evm_nand_init, DEV_ON_BASEBOARD, PROFILE_ALL},
	{mmc0_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
	{rgmii1_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
	{rgmii2_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
	{display_init,     DEV_ON_BASEBOARD, PROFILE_ALL},
	{enable_ehrpwm0,	DEV_ON_BASEBOARD, PROFILE_ALL},
	//{tsc_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
	{mfd_tscadc_init, DEV_ON_BASEBOARD, PROFILE_ALL},
	//{mcasp0_init,   DEV_ON_BASEBOARD, PROFILE_ALL},       //IIS
	{usb0_init,     DEV_ON_BASEBOARD, PROFILE_ALL},
	{usb1_init,     DEV_ON_BASEBOARD, PROFILE_ALL},	
	{uart1_init, 	DEV_ON_BASEBOARD, PROFILE_ALL},
	{uart2_init,    DEV_ON_BASEBOARD, PROFILE_ALL},
	{uart3_init,    DEV_ON_BASEBOARD, PROFILE_ALL},
	{uart4_init,    DEV_ON_BASEBOARD, PROFILE_ALL},
	{uart5_init,    DEV_ON_BASEBOARD, PROFILE_ALL},
	//{d_can_init,    DEV_ON_BASEBOARD, PROFILE_ALL},
	//{gpio_keys_init,  DEV_ON_BASEBOARD, PROFILE_ALL},
	{modules_exints_init,  DEV_ON_BASEBOARD, PROFILE_ALL},
	{gpio_led_init,  DEV_ON_BASEBOARD, PROFILE_ALL},

	{myir_gpio_init, DEV_ON_BASEBOARD, PROFILE_ALL},
#ifdef CONFIG_MYIR_WDT
	{myir_wdt_init, DEV_ON_BASEBOARD, PROFILE_ALL},
#endif
	
	{NULL, 0, 0},
};

static int am33xx_evm_tx_clk_dly_phy_fixup(struct phy_device *phydev)
{
	phy_write(phydev, AR8051_PHY_DEBUG_ADDR_REG,
		  AR8051_DEBUG_RGMII_CLK_DLY_REG);
	phy_write(phydev, AR8051_PHY_DEBUG_DATA_REG, AR8051_RGMII_TX_CLK_DLY);

	return 0;
}

static void setup_myd_am335x(void)
{
	pr_info("The board is MYD-AM335X.\n");

	/* Starter Kit has Micro-SD slot which doesn't have Write Protect pin */
	am335x_mmc[0].gpio_wp = -EINVAL;

	_configure_device(EVM_SK, myd_am335x_dev_cfg, PROFILE_NONE);

	am33xx_cpsw_init(AM33XX_CPSW_MODE_RGMII, "0:04", "0:06");
	/* Atheros Tx Clk delay Phy fixup */
	phy_register_fixup_for_uid(AM335X_EVM_PHY_ID, AM335X_EVM_PHY_MASK,
				   am33xx_evm_tx_clk_dly_phy_fixup);
}

static void am335x_evm_setup(void)
{
	daughter_brd_detected = false;
	setup_myd_am335x();

	/* SmartReflex also requires board information. */
	am33xx_sr_init();

	return;
}

/* TPS65217 voltage regulator support */

/* 1.5V */
static struct regulator_consumer_supply tps65217_dcdc1_consumers[] = {
	{
		.supply = "vdds_ddr",
	},
	{
		.supply = "ddr3",
	},
};

/* 1.1V */
static struct regulator_consumer_supply tps65217_dcdc2_consumers[] = {
	{
		.supply = "vdd_mpu",
	},
};

/* 1.1V */
static struct regulator_consumer_supply tps65217_dcdc3_consumers[] = {
	{
		.supply = "vdd_core",
	},
};

/* 1.8V LDO */
static struct regulator_consumer_supply tps65217_ldo1_consumers[] = {
	{
		.supply = "vdds_rtc",
	},
};

/* 3.3V LDO */
static struct regulator_consumer_supply tps65217_ldo2_consumers[] = {
	{
		.supply = "vdds_any_pn",
	},
};

/* 1.8V LDO */
static struct regulator_consumer_supply tps65217_ldo3_consumers[] = {
	{
		.supply = "vdds_osc",
	},
	{
		.supply = "vdds_pll_ddr",
	},
	{
		.supply = "vdda_usb0_1p8v",
	},
	{
		.supply = "vdds_sram_core_bg",
	},
	{
		.supply = "vdds_sram_mpu_bb",
	},
	{
		.supply = "vdds_pll_mpu",
	},
	{
		.supply = "vdda_adc",
	},
	{
		.supply = "vdds",
	},
	{
		.supply = "vdds_hvx_1p8v",
	},
	{
		.supply = "vdds_pll_core_lcd",
	},
};

/* 3.3V LDO */
static struct regulator_consumer_supply tps65217_ldo4_consumers[] = {
	{
		.supply = "vdds_hvx_ldo4_3p3v",
	},
};

static struct regulator_init_data tps65217_regulator_data[] = {
	/* dcdc1 */
	{
		.constraints = {
			.min_uV = 900000,
			.max_uV = 1800000,
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_dcdc1_consumers),
		.consumer_supplies = tps65217_dcdc1_consumers,
	},

	/* dcdc2 */
	{
		.constraints = {
			.min_uV = 900000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
							   REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_dcdc2_consumers),
		.consumer_supplies = tps65217_dcdc2_consumers,
	},

	/* dcdc3 */
	{
		.constraints = {
			.min_uV = 900000,
			.max_uV = 1500000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
							   REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_dcdc3_consumers),
		.consumer_supplies = tps65217_dcdc3_consumers,
	},

	/* ldo1 */
	{
		.constraints = {
			.min_uV = 1000000,
			.max_uV = 3300000,
			//.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_ldo1_consumers),
		.consumer_supplies = tps65217_ldo1_consumers,
	},

	/* ldo2 */
	{
		.constraints = {
			.min_uV = 900000,
			.max_uV = 3300000,
			//.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
			//				   REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_ldo2_consumers),
		.consumer_supplies = tps65217_ldo2_consumers,
	},

	/* ldo3 */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 3300000,
			//.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
			//				   REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_ldo3_consumers),
		.consumer_supplies = tps65217_ldo3_consumers,
	},

	/* ldo4 */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 3300000,
			//.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
			//				   REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_ldo4_consumers),
		.consumer_supplies = tps65217_ldo4_consumers,
	},
};

static struct tps65217_board myir_tps65217_info = {
	.tps65217_init_data = &tps65217_regulator_data[0],
};

static struct ft5x0x_ts_platform_data ts_plat_data = {
	.irq            = OMAP_GPIO_IRQ(GPIO_TO_PIN(0, 20)),
	.polling_mode   = 0,
	.multi_touch    = 0,
};

static struct at24_platform_data board_eeprom = {
	.byte_len = 4096,
	.page_size = 32,
	.flags = AT24_FLAG_ADDR16,
};

static struct i2c_board_info __initdata am335x_i2c0_boardinfo[] = {
	{
		I2C_BOARD_INFO("tps65217", TPS65217_I2C_ID),
		.platform_data  = &myir_tps65217_info,
	},
	{
		I2C_BOARD_INFO("sgtl5000", 0x0A),
	},
	{
		I2C_BOARD_INFO("ft5x06_ts", 0x38),
		.platform_data = &ts_plat_data,
	},
	{
		I2C_BOARD_INFO("at24", 0x50),
		.platform_data = &board_eeprom,
	},
};

/* sc16is7x2 -- Add by JBO */
static const char *i2c_gpio_names[SC16IS7X2_NR_GPIOS] = {
	"i2c-gpio200",
	"i2c-gpio201",
	"i2c-gpio202",
	"i2c-gpio203",
	"i2c-gpio204",
	"i2c-gpio205",
	"i2c-gpio206",
	"i2c-gpio207",
};
static struct sc16is7x2_platform_data i2c_uart_gpio_data = {
	.uartclk        = /*48000000*/11059200,/* crystal freq of sc16is7x2*/
	.uart_base      = 0,
	.gpio_base      = SC16IS7X2_GPIO_BASE,
	.label          = NULL,
	.names          = i2c_gpio_names,
};

static struct i2c_board_info am335x_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("sc16is7x2", 0x48),
		.platform_data  = &i2c_uart_gpio_data,
		.irq = OMAP_GPIO_IRQ(GPIO_TO_PIN(0, 27)),
	},
};

static void __init am335x_evm_i2c_init(void)
{
	setup_pin_mux(i2c0_pin_mux);
	omap_register_i2c_bus(1, 100, am335x_i2c0_boardinfo,
				ARRAY_SIZE(am335x_i2c0_boardinfo));

	setup_pin_mux(sc16is7x2_pin_mux);
	setup_pin_mux(i2c1_pin_mux);
	omap_register_i2c_bus(2, 100, am335x_i2c1_boardinfo,
							  ARRAY_SIZE(am335x_i2c1_boardinfo));

}

static struct omap_musb_board_data musb_board_data = {
	.interface_type	= MUSB_INTERFACE_ULPI,
	/*
	 * mode[0:3] = USB0PORT's mode
	 * mode[4:7] = USB1PORT's mode
	 * AM335X beta EVM has USB0 in OTG mode and USB1 in host mode.
	 */
	.mode           = (MUSB_HOST << 4) | MUSB_OTG,
	.power		= 500,
	.instances	= 1,
};

static struct resource am335x_rtc_resources[] = {
	{
		.start		= AM33XX_RTC_BASE,
		.end		= AM33XX_RTC_BASE + SZ_4K - 1,
		.flags		= IORESOURCE_MEM,
	},
	{ /* timer irq */
		.start		= AM33XX_IRQ_RTC_TIMER,
		.end		= AM33XX_IRQ_RTC_TIMER,
		.flags		= IORESOURCE_IRQ,
	},
	{ /* alarm irq */
		.start		= AM33XX_IRQ_RTC_ALARM,
		.end		= AM33XX_IRQ_RTC_ALARM,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device am335x_rtc_device = {
	.name           = "omap_rtc",
	.id             = -1,
	.num_resources	= ARRAY_SIZE(am335x_rtc_resources),
	.resource	= am335x_rtc_resources,
};

static int am335x_rtc_init(void)
{
	void __iomem *base;
	struct clk *clk;

	clk = clk_get(NULL, "rtc_fck");
	if (IS_ERR(clk)) {
		pr_err("rtc : Failed to get RTC clock\n");
		return -1;
	}

	if (clk_enable(clk)) {
		pr_err("rtc: Clock Enable Failed\n");
		return -1;
	}

	base = ioremap(AM33XX_RTC_BASE, SZ_4K);

	if (WARN_ON(!base))
		return -ENOMEM;

	/* Unlock the rtc's registers */
	writel(0x83e70b13, base + 0x6c);
	writel(0x95a4f1e0, base + 0x70);

	
	/*
	 * Enable the 32K OSc
	 * TODO: Need a better way to handle this
	 * Since we want the clock to be running before mmc init
	 * we need to do it before the rtc probe happens
	 */
	writel(0x48, base + 0x54);

	iounmap(base);

	return  platform_device_register(&am335x_rtc_device);
}

/* Enable clkout2 */
static struct pinmux_config clkout2_pin_mux[] = {
	{"xdma_event_intr1.clkout2", OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

static void __init clkout2_enable(void)
{
	struct clk *ck_32;

	ck_32 = clk_get(NULL, "clkout2_ck");
	if (IS_ERR(ck_32)) {
		pr_err("Cannot clk_get ck_32\n");
		return;
	}

	clk_enable(ck_32);

	setup_pin_mux(clkout2_pin_mux);
}

void __iomem *am33xx_emif_base;

void __iomem * __init am33xx_get_mem_ctlr(void)
{

	am33xx_emif_base = ioremap(AM33XX_EMIF0_BASE, SZ_32K);

	if (!am33xx_emif_base)
		pr_warning("%s: Unable to map DDR2 controller",	__func__);

	return am33xx_emif_base;
}

void __iomem *am33xx_get_ram_base(void)
{
	return am33xx_emif_base;
}

void __iomem *am33xx_gpio0_base;

void __iomem *am33xx_get_gpio0_base(void)
{
	am33xx_gpio0_base = ioremap(AM33XX_GPIO0_BASE, SZ_4K);

	return am33xx_gpio0_base;
}

static struct resource am33xx_cpuidle_resources[] = {
	{
		.start		= AM33XX_EMIF0_BASE,
		.end		= AM33XX_EMIF0_BASE + SZ_32K - 1,
		.flags		= IORESOURCE_MEM,
	},
};

/* AM33XX devices support DDR2 power down */
static struct am33xx_cpuidle_config am33xx_cpuidle_pdata = {
	.ddr2_pdown	= 1,
};

static struct platform_device am33xx_cpuidle_device = {
	.name			= "cpuidle-am33xx",
	.num_resources		= ARRAY_SIZE(am33xx_cpuidle_resources),
	.resource		= am33xx_cpuidle_resources,
	.dev = {
		.platform_data	= &am33xx_cpuidle_pdata,
	},
};

static void __init am33xx_cpuidle_init(void)
{
	int ret;

	am33xx_cpuidle_pdata.emif_base = am33xx_get_mem_ctlr();

	ret = platform_device_register(&am33xx_cpuidle_device);

	if (ret)
		pr_warning("AM33XX cpuidle registration failed\n");

}

static void __init am335x_evm_init(void)
{
	am33xx_cpuidle_init();
	am33xx_mux_init(board_mux);
	omap_serial_init();
	am335x_rtc_init();
	/* conflicts with cap ts int pin on myd-am335x-j, MYIR
    clkout2_enable(); */
	am335x_evm_i2c_init();
	am335x_evm_setup();
	omap_sdrc_init(NULL, NULL);
	usb_musb_init(&musb_board_data);
	omap_board_config = am335x_evm_config;
	omap_board_config_size = ARRAY_SIZE(am335x_evm_config);
	/* Create an alias for icss clock */
	if (clk_add_alias("pruss", NULL, "pruss_uart_gclk", NULL))
		pr_warn("failed to create an alias: icss_uart_gclk --> pruss\n");
	/* Create an alias for gfx/sgx clock */
	if (clk_add_alias("sgx_ck", NULL, "gfx_fclk", NULL))
		pr_warn("failed to create an alias: gfx_fclk --> sgx_ck\n");
}

static void __init am335x_evm_map_io(void)
{
	omap2_set_globals_am33xx();
	omapam33xx_map_common_io();
}

MACHINE_START(AM335XEVM, "am335xevm")
	/* Maintainer: Texas Instruments */
	.atag_offset	= 0x100,
	.map_io		= am335x_evm_map_io,
	.init_early	= am33xx_init_early,
	.init_irq	= ti81xx_init_irq,
	.handle_irq     = omap3_intc_handle_irq,
	.timer		= &omap3_am33xx_timer,
	.init_machine	= am335x_evm_init,
MACHINE_END

MACHINE_START(AM335XIAEVM, "am335xiaevm")
	/* Maintainer: Texas Instruments */
	.atag_offset	= 0x100,
	.map_io		= am335x_evm_map_io,
	.init_irq	= ti81xx_init_irq,
	.init_early	= am33xx_init_early,
	.timer		= &omap3_am33xx_timer,
	.init_machine	= am335x_evm_init,
MACHINE_END
