/*
 * This file contains the address info for various AM33XX modules.
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

#ifndef __ASM_ARCH_AM33XX_H
#define __ASM_ARCH_AM33XX_H

#define L4_SLOW_AM33XX_BASE	0x48000000

#define AM33XX_SCM_BASE		0x44E10000
#define AM33XX_CTRL_BASE	AM33XX_SCM_BASE
#define AM33XX_PRCM_BASE	0x44E00000

#define AM33XX_EMIF0_BASE	0x4C000000

#define AM33XX_GPIO0_BASE	0x44E07000
#define AM33XX_GPIO1_BASE	0x4804C000
#define AM33XX_GPIO2_BASE	0x481AC000
#define AM33XX_GPIO3_BASE	0x481AE000

#define AM33XX_TIMER0_BASE	0x44E05000
#define AM33XX_TIMER1_BASE	0x44E31000
#define AM33XX_TIMER2_BASE	0x48040000
#define AM33XX_TIMER3_BASE	0x48042000
#define AM33XX_TIMER4_BASE	0x48044000
#define AM33XX_TIMER5_BASE	0x48046000
#define AM33XX_TIMER6_BASE	0x48048000
#define AM33XX_TIMER7_BASE	0x4804A000

#define AM33XX_WDT1_BASE	0x44E35000

#define AM33XX_TSC_BASE		0x44E0D000
#define AM33XX_RTC_BASE		0x44E3E000

#define AM33XX_SR0_BASE         0x44E37000
#define AM33XX_SR1_BASE         0x44E39000

#define AM33XX_ASP0_BASE	0x48038000
#define AM33XX_ASP1_BASE	0x4803C000

#define AM33XX_MAILBOX0_BASE	0x480C8000

#define AM33XX_MMC0_BASE	0x48060100
#define AM33XX_MMC1_BASE	0x481D8100
#define AM33XX_MMC2_BASE	0x47810100

#define AM33XX_I2C0_BASE	0x44E0B000
#define AM33XX_I2C1_BASE	0x4802A000
#define AM33XX_I2C2_BASE	0x4819C000

#define AM33XX_SPI0_BASE	0x48030000
#define AM33XX_SPI1_BASE	0x481A0000

#define AM33XX_USBSS_BASE	0x47400000
#define AM33XX_USB0_BASE	0x47401000
#define AM33XX_USB1_BASE	0x47401800

#define AM33XX_ELM_BASE		0x48080000

/* Base address for crypto modules */
#define AM33XX_SHA1MD5_S_BASE	0x53000000
#define AM33XX_SHA1MD5_P_BASE	0x53100000

#define	AM33XX_AES0_S_BASE	0x53400000
#define	AM33XX_AES0_P_BASE	0x53500000
#define	AM33XX_AES1_S_BASE	0x53600000
#define	AM33XX_AES1_P_BASE	0x53700000

#define	AM33XX_RNG_BASE		0x48310000

#define AM33XX_ASP0_BASE	0x48038000
#define AM33XX_ASP1_BASE	0x4803C000

#define AM33XX_CPSW_BASE       0x4A100000
#define AM33XX_CPSW_MDIO_BASE  0x4A101000
#define AM33XX_CPSW_SS_BASE    0x4A101200

#define AM33XX_ICSS_BASE	0x4A300000
#define AM33XX_ICSS_LEN		0x3FFFF

#define AM33XX_EPWMSS0_BASE	0x48300000
#define AM33XX_EPWMSS1_BASE	0x48302000
#define AM33XX_EPWMSS2_BASE	0x48304000

/*
 * ----------------------------------------------------------------------------
 * CPSW
 * ----------------------------------------------------------------------------
 */
#ifndef __ASSEMBLER__
enum am33xx_cpsw_mac_mode {
	AM33XX_CPSW_MODE_MII,
	AM33XX_CPSW_MODE_RMII,
	AM33XX_CPSW_MODE_RGMII,
};
int am33xx_cpsw_init(enum am33xx_cpsw_mac_mode mode, unsigned char *phy_id0,
		     unsigned char *phy_id1);
#endif

#endif /* __ASM_ARCH_AM33XX_H */
