/*
 * Code for supporting AM335X EVM.
 *
 * Copyright (C) {2011} Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef _BOARD_AM335X_H
#define _BOARD_AM335X_H

#define BASEBOARD_I2C_ADDR	0x50
#define DAUG_BOARD_I2C_ADDR	0x51
#define LCD_BOARD_I2C_ADDR	0x52

#define GEN_PURP_EVM		0
#define IND_AUT_MTR_EVM		1
#define BEAGLE_BONE_OLD		2
#define BEAGLE_BONE_A3		3
#define EVM_SK			4

/* REVIST : check posibility of PROFILE_(x) syntax usage */
#define PROFILE_NONE	-1	/* Few EVM doesn't have profiles */
#define PROFILE_0		(0x1 << 0)
#define PROFILE_1		(0x1 << 1)
#define PROFILE_2		(0x1 << 2)
#define PROFILE_3		(0x1 << 3)
#define PROFILE_4		(0x1 << 4)
#define PROFILE_5		(0x1 << 5)
#define PROFILE_6		(0x1 << 6)
#define PROFILE_7		(0x1 << 7)
#define PROFILE_ALL		0xFF

#ifndef __ASSEMBLER__
void am335x_evm_set_id(unsigned int evmid);
int am335x_evm_get_id(void);
void am33xx_cpsw_macidfillup(char *eeprommacid0, char *eeprommacid1);
void am33xx_sr_init(void);
void am33xx_d_can_init(unsigned int instance);

/* MYIR watchdog */
struct myir_wdt_platdata {
    int default_period_ms;
    int gpio_pin;
};

/* Structure for ft5x0x */
struct ft5x0x_ts_platform_data {
    u16    irq;            /* irq number of ts used */
    u8     polling_mode;   /* set 1 for polling mode and 0 for interruputing mode */
    u8     multi_touch;    /* set 1 if supporting multi-touch */
};

#endif
#endif
