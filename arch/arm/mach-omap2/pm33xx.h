/*
 * AM33XX Power Management Routines
 *
 * Copyright (C) 2012 Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ARCH_ARM_MACH_OMAP2_PM33XX_H
#define __ARCH_ARM_MACH_OMAP2_PM33XX_H

#include <mach/hardware.h>	/* XXX Is this the right one to include? */
#include "control.h"
#include "mux33xx.h"

#ifndef __ASSEMBLER__
extern void __iomem *am33xx_get_ram_base(void);

/*
 * This enum is used to index the array passed to suspend routine with
 * parameters that vary across DDR2 and DDR3 sleep sequence.
 *
 * Since these are used to load into registers by suspend code,
 * entries here must always be in sync with the suspend code
 * in arm/mach-omap2/sleep33xx.S
 */
enum suspend_cfg_params {
	MEMORY_TYPE = 0,
	SUSP_VTP_CTRL_VAL,
	EVM_ID,
	SUSPEND_CFG_PARAMS_END /* Must be the last entry */
};

struct a8_wkup_m3_ipc_data {
	int resume_addr;
	int sleep_mode;
	int ipc_data1;
	int ipc_data2;
} am33xx_lp_ipc;

struct am33xx_padconf_regs {
	u16 offset;
	u32 val;
};

#ifdef CONFIG_SUSPEND
static struct am33xx_padconf_regs am33xx_lp_padconf[] = {
	{.offset = AM33XX_CONTROL_GMII_SEL_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_GPMC_A0_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_GPMC_A1_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_GPMC_A2_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_GPMC_A3_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_GPMC_A4_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_GPMC_A5_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_GPMC_A6_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_GPMC_A7_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_GPMC_A8_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_GPMC_A9_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_GPMC_A10_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_GPMC_A11_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_GPMC_WAIT0_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_GPMC_WPN_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_GPMC_BEN1_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_COL_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_CRS_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_RXERR_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_TXEN_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_RXDV_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_TXD3_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_TXD2_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_TXD1_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_TXD0_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_TXCLK_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_RXCLK_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_RXD3_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_RXD2_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_RXD1_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_RXD0_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_REFCLK_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MDIO_DATA_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MDIO_CLK_OFFSET},
};
#endif /* CONFIG_SUSPEND */
#endif /* ASSEMBLER */

#define M3_TXEV_EOI			(AM33XX_CTRL_BASE + 0x1324)
#define A8_M3_IPC_REGS			(AM33XX_CTRL_BASE + 0x1328)
#define DS_RESUME_BASE			0x40300000
#define DS_IPC_DEFAULT			0xffffffff
#define M3_UMEM				0x44D00000

#define	DS0_ID				0x3
#define DS1_ID				0x5

#define M3_STATE_UNKNOWN		-1
#define M3_STATE_RESET			0
#define M3_STATE_INITED			1
#define M3_STATE_MSG_FOR_LP		2
#define M3_STATE_MSG_FOR_RESET		3

#define VTP_CTRL_READY		(0x1 << 5)
#define VTP_CTRL_ENABLE		(0x1 << 6)
#define VTP_CTRL_LOCK_EN	(0x1 << 4)
#define VTP_CTRL_START_EN	(0x1)

#define DDR_IO_CTRL		(AM33XX_CTRL_BASE + 0x0E04)
#define VTP0_CTRL_REG		(AM33XX_CTRL_BASE + 0x0E0C)
#define DDR_CMD0_IOCTRL		(AM33XX_CTRL_BASE + 0x1404)
#define DDR_CMD1_IOCTRL		(AM33XX_CTRL_BASE + 0x1408)
#define DDR_CMD2_IOCTRL		(AM33XX_CTRL_BASE + 0x140C)
#define DDR_DATA0_IOCTRL	(AM33XX_CTRL_BASE + 0x1440)
#define DDR_DATA1_IOCTRL	(AM33XX_CTRL_BASE + 0x1444)

#define MEM_TYPE_DDR2		2

#define SUSP_VTP_CTRL_DDR2	0x10117
#define SUSP_VTP_CTRL_DDR3	0x0

#endif
