/*
 * TI Touch Screen / ADC MFD driver
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
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

#include <linux/mfd/core.h>

#define TSCADC_REG_RAWIRQSTATUS		0x024
#define TSCADC_REG_IRQSTATUS		0x028
#define TSCADC_REG_IRQENABLE		0x02C
#define TSCADC_REG_IRQCLR		0x030
#define TSCADC_REG_IRQWAKEUP		0x034
#define TSCADC_REG_CTRL			0x040
#define TSCADC_REG_ADCFSM		0x044
#define TSCADC_REG_CLKDIV		0x04C
#define TSCADC_REG_SE			0x054
#define TSCADC_REG_IDLECONFIG		0x058
#define TSCADC_REG_CHARGECONFIG		0x05C
#define TSCADC_REG_CHARGEDELAY		0x060
#define TSCADC_REG_STEPCONFIG(n)	(0x64 + ((n - 1) * 8))
#define TSCADC_REG_STEPDELAY(n)		(0x68 + ((n - 1) * 8))
#define TSCADC_REG_FIFO0CNT		0xE4
#define TSCADC_REG_FIFO0THR		0xE8
#define TSCADC_REG_FIFO1CNT		0xF0
#define TSCADC_REG_FIFO1THR		0xF4
#define TSCADC_REG_FIFO0		0x100
#define TSCADC_REG_FIFO1		0x200

/*	Register Bitfields	*/
/* IRQ wakeup enable */
#define TSCADC_IRQWKUP_ENB		BIT(0)

/* Step Enable */
#define TSCADC_STEPENB_MASK		(0x1FFFF << 0)
#define TSCADC_STEPENB(val)		((val) << 0)
#define TSCADC_STPENB_STEPENB		TSCADC_STEPENB(0x1FFFF)
#define TSCADC_STPENB_STEPENB_TC	TSCADC_STEPENB(0x1FFF)
#define TSCADC_ENB(val)			(1 << (val))

/* IRQ enable */
#define TSCADC_IRQENB_HW_PEN		BIT(0)
#define TSCADC_IRQENB_FIFO0THRES	BIT(2)
#define TSCADC_IRQENB_FIFO0OVRRUN	BIT(3)
#define TSCADC_IRQENB_FIFO0UNDRFLW	BIT(4)
#define TSCADC_IRQENB_FIFO1THRES	BIT(5)
#define TSCADC_IRQENB_FIFO1OVRRUN       BIT(6)
#define TSCADC_IRQENB_FIFO1UNDRFLW      BIT(7)
#define TSCADC_IRQENB_PENUP		BIT(9)

/* Step Configuration */
#define TSCADC_STEPCONFIG_MODE_MASK	(3 << 0)
#define TSCADC_STEPCONFIG_MODE(val)	((val) << 0)
#define TSCADC_STEPCONFIG_MODE_HWSYNC	TSCADC_STEPCONFIG_MODE(2)
#define TSCADC_STEPCONFIG_MODE_SWCNT	TSCADC_STEPCONFIG_MODE(1)
#define TSCADC_STEPCONFIG_AVG_MASK	(7 << 2)
#define TSCADC_STEPCONFIG_AVG(val)	((val) << 2)
#define TSCADC_STEPCONFIG_AVG_16	TSCADC_STEPCONFIG_AVG(4)
#define TSCADC_STEPCONFIG_XPP		BIT(5)
#define TSCADC_STEPCONFIG_XNN		BIT(6)
#define TSCADC_STEPCONFIG_YPP		BIT(7)
#define TSCADC_STEPCONFIG_YNN		BIT(8)
#define TSCADC_STEPCONFIG_XNP		BIT(9)
#define TSCADC_STEPCONFIG_YPN		BIT(10)
#define TSCADC_STEPCONFIG_INM_MASK	(0xF << 15)
#define TSCADC_STEPCONFIG_INM(val)	((val) << 15)
#define TSCADC_STEPCONFIG_INM_ADCREFM	TSCADC_STEPCONFIG_INM(8)
#define TSCADC_STEPCONFIG_INP_MASK	(0xF << 19)
#define TSCADC_STEPCONFIG_INP(val)	((val) << 19)
#define TSCADC_STEPCONFIG_INP_AN2	TSCADC_STEPCONFIG_INP(2)
#define TSCADC_STEPCONFIG_INP_AN3	TSCADC_STEPCONFIG_INP(3)
#define TSCADC_STEPCONFIG_INP_AN4	TSCADC_STEPCONFIG_INP(4)
#define TSCADC_STEPCONFIG_INP_ADCREFM	TSCADC_STEPCONFIG_INP(8)
#define TSCADC_STEPCONFIG_FIFO1		BIT(26)

/* Delay register */
#define TSCADC_STEPDELAY_OPEN_MASK	(0x3FFFF << 0)
#define TSCADC_STEPDELAY_OPEN(val)	((val) << 0)
#define TSCADC_STEPCONFIG_OPENDLY	TSCADC_STEPDELAY_OPEN(0x098)
#define TSCADC_STEPDELAY_SAMPLE_MASK	(0xFF << 24)
#define TSCADC_STEPDELAY_SAMPLE(val)	((val) << 24)
#define TSCADC_STEPCONFIG_SAMPLEDLY	TSCADC_STEPDELAY_SAMPLE(0)

/* Charge Config */
#define TSCADC_STEPCHARGE_RFP_MASK	(7 << 12)
#define TSCADC_STEPCHARGE_RFP(val)	((val) << 12)
#define TSCADC_STEPCHARGE_RFP_XPUL	TSCADC_STEPCHARGE_RFP(1)
#define TSCADC_STEPCHARGE_INM_MASK	(0xF << 15)
#define TSCADC_STEPCHARGE_INM(val)	((val) << 15)
#define TSCADC_STEPCHARGE_INM_AN1	TSCADC_STEPCHARGE_INM(1)
#define TSCADC_STEPCHARGE_INP_MASK	(0xF << 19)
#define TSCADC_STEPCHARGE_INP(val)	((val) << 19)
#define TSCADC_STEPCHARGE_INP_AN1	TSCADC_STEPCHARGE_INP(1)
#define TSCADC_STEPCHARGE_RFM_MASK	(3 << 23)
#define TSCADC_STEPCHARGE_RFM(val)	((val) << 23)
#define TSCADC_STEPCHARGE_RFM_XNUR	TSCADC_STEPCHARGE_RFM(1)

/* Charge delay */
#define TSCADC_CHARGEDLY_OPEN_MASK	(0x3FFFF << 0)
#define TSCADC_CHARGEDLY_OPEN(val)	((val) << 0)
#define TSCADC_CHARGEDLY_OPENDLY	TSCADC_CHARGEDLY_OPEN(1)

/* Control register */
#define TSCADC_CNTRLREG_TSCSSENB	BIT(0)
#define TSCADC_CNTRLREG_STEPID		BIT(1)
#define TSCADC_CNTRLREG_STEPCONFIGWRT	BIT(2)
#define TSCADC_CNTRLREG_POWERDOWN	BIT(4)
#define TSCADC_CNTRLREG_AFE_CTRL_MASK	(3 << 5)
#define TSCADC_CNTRLREG_AFE_CTRL(val)	((val) << 5)
#define TSCADC_CNTRLREG_4WIRE		TSCADC_CNTRLREG_AFE_CTRL(1)
#define TSCADC_CNTRLREG_5WIRE		TSCADC_CNTRLREG_AFE_CTRL(2)
#define TSCADC_CNTRLREG_8WIRE		TSCADC_CNTRLREG_AFE_CTRL(3)
#define TSCADC_CNTRLREG_TSCENB		BIT(7)

/* FIFO READ Register */
#define TSCADC_FIFOREAD_DATA_MASK	(0xfff << 0)
#define TSCADC_FIFOREAD_CHNLID_MASK	(0xf << 16)

/* Sequencer Status */
#define TSCADC_SEQ_STATUS		BIT(5)

#define ADC_CLK				3000000
#define TOTAL_STEPS			16
#define TOTAL_CHANNELS			8
#define FIFO1_THRESHOLD			19

/*
 * ADC runs at 3MHz, and it takes
 * 15 cycles to latch one data output.
 * Hence the idle time for ADC to
 * process one sample data would be
 * around 5 micro seconds.
 */
#define IDLE_TIMEOUT			5 /* microsec */

#define TSCADC_CELLS			2

struct mfd_tscadc_board {
	struct tsc_data *tsc_init;
	struct adc_data *adc_init;
};

struct ti_tscadc_dev {
	struct device *dev;
	void __iomem *tscadc_base;
	int irq;
	struct mfd_cell cells[TSCADC_CELLS];

	/* tsc device */
	struct tscadc *tsc;

	/* adc device */
	struct adc_device *adc;

	/* Context save */
	unsigned int irqstat;
	unsigned int ctrl;
};
