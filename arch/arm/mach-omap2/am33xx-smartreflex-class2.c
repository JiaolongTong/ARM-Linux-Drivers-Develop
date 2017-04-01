/*
 * SmartReflex Voltage Control driver
 *
 * Copyright (C) 2012 Texas Instruments, Inc. - http://www.ti.com/
 * Author: Greg Guyotte <gguyotte@ti.com> (modified for AM33xx)
 *
 * Copyright (C) 2011 Texas Instruments, Inc. - http://www.ti.com/
 * Author: AnilKumar Ch <anilkumar@ti.com>
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
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/cpufreq.h>
#include <linux/opp.h>

#include <plat/common.h>
#include <plat/smartreflex.h>

#include "control.h"
#include "voltage.h"

#define CLK_NAME_LEN		40

static inline void sr_write_reg(struct am33xx_sr *sr, int offset, u32 value,
					u32 srid)
{
	writel(value, sr->sen[srid].base + offset);
}

static inline void sr_modify_reg(struct am33xx_sr *sr, int offset, u32 mask,
				u32 value, u32 srid)
{
	u32 reg_val;

	reg_val = readl(sr->sen[srid].base + offset);
	reg_val &= ~mask;
	reg_val |= (value&mask);

	writel(reg_val, sr->sen[srid].base + offset);
}

static inline u32 sr_read_reg(struct am33xx_sr *sr, int offset, u32 srid)
{
	return readl(sr->sen[srid].base + offset);
}

static void cal_reciprocal(u32 sensor, u32 *sengain, u32 *rnsen) {
         u32 gn, rn, mul;
 
         for (gn = 0; gn < GAIN_MAXLIMIT; gn++) {
                 mul = 1 << (gn + 8);
                 rn = mul / sensor;
                 if (rn < R_MAXLIMIT) {
                         *sengain = gn;
                         *rnsen = rn;
                 }
         }
}
 
static u32 cal_test_nvalue(u32 sennval, u32 senpval) {
         u32 senpgain=0, senngain=0;
         u32 rnsenp=0, rnsenn=0;
 
         /* Calculating the gain and reciprocal of the SenN and SenP values */
         cal_reciprocal(senpval, &senpgain, &rnsenp);
         cal_reciprocal(sennval, &senngain, &rnsenn);
 
         return (senpgain << NVALUERECIPROCAL_SENPGAIN_SHIFT) |
                 (senngain << NVALUERECIPROCAL_SENNGAIN_SHIFT) |
                 (rnsenp << NVALUERECIPROCAL_RNSENP_SHIFT) |
                 (rnsenn << NVALUERECIPROCAL_RNSENN_SHIFT);
}

static unsigned int sr_adjust_efuse_nvalue(unsigned int opp_no,
                                                 unsigned int orig_opp_nvalue,
                                                 unsigned int mv_delta) {
         unsigned int new_opp_nvalue;
         unsigned int senp_gain, senn_gain, rnsenp, rnsenn, pnt_delta, nnt_delta;
         unsigned int new_senn, new_senp, senn, senp;
 
         /* calculate SenN and SenP from the efuse value */
         senp_gain = ((orig_opp_nvalue >> 20) & 0xf);
         senn_gain = ((orig_opp_nvalue >> 16) & 0xf);
         rnsenp = ((orig_opp_nvalue >> 8) & 0xff);
         rnsenn = (orig_opp_nvalue & 0xff);
 
         senp = ((1<<(senp_gain+8))/(rnsenp));
         senn = ((1<<(senn_gain+8))/(rnsenn));
 
         /* calculate the voltage delta */
         pnt_delta = (26 * mv_delta)/10;
         nnt_delta = (3 * mv_delta);
 
         /* now lets add the voltage delta to the sensor values */
         new_senn = senn + nnt_delta;
         new_senp = senp + pnt_delta;
 
         new_opp_nvalue = cal_test_nvalue(new_senn, new_senp);
 
         printk("Compensating OPP%d for %dmV Orig nvalue:0x%x New nvalue:0x%x \n",
                         opp_no, mv_delta, orig_opp_nvalue, new_opp_nvalue);
 
         return new_opp_nvalue;
}

/* irq_sr_reenable - Re-enable SR interrupts (triggered by delayed work queue)
 * @work:	pointer to work_struct embedded in am33xx_sr_sensor struct
 *
 * While servicing the IRQ, this function is added to the delayed work queue.
 * This gives time for the voltage change to settle before we re-enable 
 * the interrupt.
 */
static void irq_sr_reenable(struct work_struct *work)
{
        u32 srid;
	struct am33xx_sr_sensor *sens;
        struct am33xx_sr *sr;

        sens = container_of((void *)work, struct am33xx_sr_sensor, 
                work_reenable);

        srid = sens->sr_id;

        sr = container_of((void *)sens, struct am33xx_sr, sen[srid]);

        dev_dbg(&sr->pdev->dev, "%s: SR %d\n", __func__, srid);

        /* Must clear IRQ status */
        sens->irq_status = 0;

        /* Re-enable the interrupt */
	sr_modify_reg(sr, IRQENABLE_SET, IRQENABLE_MCUBOUNDSINT,
		IRQENABLE_MCUBOUNDSINT, srid);

	/* Restart the module after voltage set */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE,
		SRCONFIG_SRENABLE, srid);
}

/* get_errvolt - get error voltage from SR error register
 * @sr:		contains SR driver data
 * @srid:	contains the srid, indicates which SR moduel lswe are using
 *
 * Read the error from SENSOR error register and then convert
 * to voltage delta, return value is the voltage delta in micro
 * volt.
 */
static int get_errvolt(struct am33xx_sr *sr, s32 srid)
{
        struct am33xx_sr_sensor *sens;
	int senerror_reg;
	s32 uvoltage;
	s8 terror;

        sens = &sr->sen[srid];

	senerror_reg = sr_read_reg(sr, SENERROR_V2, srid);
	senerror_reg = (senerror_reg & 0x0000FF00);
	terror = (s8)(senerror_reg >> 8);

        /* math defined in SR functional spec */
	uvoltage = ((terror) * sr->uvoltage_step_size) >> 7;
	uvoltage = uvoltage * sens->opp_data[sens->curr_opp].e2v_gain;

	return uvoltage;
}

/* set_voltage - Schedule task for setting the voltage
 * @work:	pointer to the work structure
 *
 * Voltage is set based on previous voltage and calculated
 * voltage error.
 *
 * Generic voltage regulator set voltage is used for changing
 * the voltage to new value.  Could potentially use voltdm_scale
 * but at time of testing voltdm was not populated with volt_data.
 *
 * Disabling the module before changing the voltage, this is
 * needed for not generating interrupt during voltage change,
 * enabling after voltage change. This will also take care of
 * resetting the SR registers.
 */
static void set_voltage(struct work_struct *work)
{
	struct am33xx_sr *sr;
	int prev_volt, new_volt, i, ret;
	s32 delta_v;

	sr = container_of((void *)work, struct am33xx_sr, work);

        for (i = 0; i < sr->no_of_sens; i++) {
                if (sr->sen[i].irq_status != 1)
                        continue;

                /* Get the current voltage from PMIC */
                prev_volt = regulator_get_voltage(sr->sen[i].reg);

                if (prev_volt < 0) {
                        dev_err(&sr->pdev->dev, 
                                "%s: SR %d: regulator_get_voltage error %d\n",
                                __func__, i, prev_volt);

                        goto reenable;
                }

        	delta_v = get_errvolt(sr, i);
                new_volt = prev_volt + delta_v;

                /* this is the primary output for debugging SR activity */
                dev_dbg(&sr->pdev->dev, 
                        "%s: SR %d: prev volt=%d, delta_v=%d, req_volt=%d\n",
                         __func__, i, prev_volt, delta_v, new_volt);
         
	        /* Clear the counter, SR module disable */
	        sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE,
			~SRCONFIG_SRENABLE, i);

                if (delta_v != 0) {
	                ret = regulator_set_voltage(sr->sen[i].reg, new_volt, 
                                new_volt + sr->uvoltage_step_size);

                        if (ret < 0)
                                dev_err(&sr->pdev->dev, 
                                "%s: regulator_set_voltage failed! (err %d)\n", 
                                __func__, ret);
                }
reenable:
                /* allow time for voltage to settle before re-enabling SR 
                   module and interrupt */        
                schedule_delayed_work(&sr->sen[i].work_reenable, 
                        msecs_to_jiffies(sr->irq_delay));
        }
}

/* sr_class2_irq - sr irq handling
 * @irq:	Number of the irq serviced
 * @data:	data contains the SR driver structure
 *
 * Smartreflex IRQ handling for class2 IP, once the IRQ handler
 * is here then disable the interrupt and re-enable after some
 * time. This is the work around for handling both interrupts,
 * while one got satisfied with the voltage change but not the
 * other. The same logic helps the case where PMIC cannot set
 * the exact voltage requested by SR IP
 *
 * Schedule work only if both interrupts are serviced
 *
 * Note that same irq handler is used for both the interrupts,
 * needed for decision making for voltage change
 */
static irqreturn_t sr_class2_irq(int irq, void *data)
{
	u32 srid;
        struct am33xx_sr *sr;
        struct am33xx_sr_sensor *sr_sensor = (struct am33xx_sr_sensor *)data;

        srid = sr_sensor->sr_id;

        sr = container_of(data, struct am33xx_sr, sen[srid]);

	sr->sen[srid].irq_status = 1;

	/* Clear MCUBounds Interrupt */
	sr_modify_reg(sr, IRQSTATUS, IRQSTATUS_MCBOUNDSINT,
			IRQSTATUS_MCBOUNDSINT, srid);

	/* Disable the interrupt and re-enable in set_voltage() */
	sr_modify_reg(sr, IRQENABLE_CLR, IRQENABLE_MCUBOUNDSINT,
			IRQENABLE_MCUBOUNDSINT, srid);

        /* Causes set_voltage() to get called at a later time.  Set_voltage()
           will check the irq_status flags to determine which SR needs to
           be serviced.  This was previously done with schedule_work, but
           I observed a crash in set_voltage() when changing OPPs on weak
           silicon, which may have been related to insufficient voltage
           settling time for OPP change.  This additional delay avoids the
           crash. */
        schedule_delayed_work(&sr->work, 
                        msecs_to_jiffies(250));

	return IRQ_HANDLED;
}

static int sr_clk_enable(struct am33xx_sr *sr, u32 srid)
{
	if (clk_enable(sr->sen[srid].fck) != 0) {
		dev_err(&sr->pdev->dev, "%s: Could not enable sr_fck\n",
					__func__);
		return -EINVAL;
	}

	return 0;
}

static int sr_clk_disable(struct am33xx_sr *sr, u32 srid)
{
	clk_disable(sr->sen[srid].fck);

	return 0;
}

static inline int sr_set_nvalues(struct am33xx_sr *sr, u32 srid)
{
        int i;
        struct am33xx_sr_sensor *sens = &sr->sen[srid];

        for (i = 0; i < sens->no_of_opps; i++) {
        	/* Read nTarget value form EFUSE register*/
	        sens->opp_data[i].nvalue = readl(AM33XX_CTRL_REGADDR
			(sens->opp_data[i].efuse_offs)) & 0xFFFFFF;

                /* validate nTarget value */
                if (sens->opp_data[i].nvalue == 0)
                        return -EINVAL;

                /* adjust nTarget based on margin in mv */
                sens->opp_data[i].adj_nvalue = sr_adjust_efuse_nvalue(i, 
                        sens->opp_data[i].nvalue, 
                        sens->opp_data[i].margin);

                dev_dbg(&sr->pdev->dev, 
                        "NValueReciprocal value (from efuse) = %08x\n", 
                        sens->opp_data[i].nvalue);

                dev_dbg(&sr->pdev->dev, 
                        "Adjusted NValueReciprocal value = %08x\n", 
                        sens->opp_data[i].adj_nvalue);
        }
	return 0;
}

/* sr_configure - Configure SR module to work in Error generator mode
 * @sr:		contains SR driver data
 * @srid:	contains the srid, specify whether it is CORE or MPU
 *
 * Configure the corresponding values to SR module registers for
 * operating SR module in Error Generator mode.
 */
static void sr_configure(struct am33xx_sr *sr, u32 srid)
{
        struct am33xx_sr_sensor *sens = &sr->sen[srid];

	/* Configuring the SR module with clock length, enabling the
	 * error generator, enable SR module, enable individual N and P
	 * sensors
	 */
	sr_write_reg(sr, SRCONFIG, (SRCLKLENGTH_125MHZ_SYSCLK |
		SRCONFIG_SENENABLE | SRCONFIG_ERRGEN_EN |
		(sens->senn_en << SRCONFIG_SENNENABLE_V2_SHIFT) |
		(sens->senp_en << SRCONFIG_SENPENABLE_V2_SHIFT)),
		srid);

	/* Configuring the Error Generator */
	sr_modify_reg(sr, ERRCONFIG_V2, (SR_ERRWEIGHT_MASK |
		SR_ERRMAXLIMIT_MASK | SR_ERRMINLIMIT_MASK),
		((sens->opp_data[sens->curr_opp].err_weight << 
                        ERRCONFIG_ERRWEIGHT_SHIFT) |
		(sens->opp_data[sens->curr_opp].err_maxlimit << 
                        ERRCONFIG_ERRMAXLIMIT_SHIFT) |
		(sens->opp_data[sens->curr_opp].err_minlimit <<          
                        ERRCONFIG_ERRMINLIMIT_SHIFT)),
		srid);
}

/* sr_enable - Enable SR module
 * @sr:		contains SR driver data
 * @srid:	contains the srid, specify whether it is CORE or MPU
 *
 * Enable SR module by writing nTarget values to corresponding SR
 * NVALUERECIPROCAL register, enable the interrupt and enable SR
 */
static void sr_enable(struct am33xx_sr *sr, u32 srid)
{
        struct am33xx_sr_sensor *sens;

        sens = &sr->sen[srid];

	/* Check if SR is already enabled. If yes do nothing */
	if (sr_read_reg(sr, SRCONFIG, srid) & SRCONFIG_SRENABLE)
		return;

	if (sens->opp_data[sens->curr_opp].nvalue == 0)
		dev_err(&sr->pdev->dev, 
                        "%s: OPP doesn't support SmartReflex\n", __func__);

	/* Writing the nReciprocal value to the register */
	sr_write_reg(sr, NVALUERECIPROCAL, 
                sens->opp_data[sens->curr_opp].adj_nvalue, srid);

	/* Enable the interrupt */
	sr_modify_reg(sr, IRQENABLE_SET, IRQENABLE_MCUBOUNDSINT,
				IRQENABLE_MCUBOUNDSINT, srid);

	/* SRCONFIG - enable SR */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE,
				SRCONFIG_SRENABLE, srid);
}

/* sr_disable - Disable SR module
 * @sr:		contains SR driver data
 * @srid:	contains the srid, specify whether it is CORE or MPU
 *
 * Disable SR module by disabling the interrupt and Smartreflex module
 */
static void sr_disable(struct am33xx_sr *sr, u32 srid)
{
	/* Disable the interrupt */
	sr_modify_reg(sr, IRQENABLE_CLR, IRQENABLE_MCUBOUNDSINT,
				IRQENABLE_MCUBOUNDSINT, srid);

	/* SRCONFIG - disable SR */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE,
				~SRCONFIG_SRENABLE, srid);
}

/* sr_start_vddautocomp - Start VDD auto compensation
 * @sr:		contains SR driver data
 *
 * This is the starting point for AVS enable from user space.
 * Also used to re-enable SR after OPP change.
 */
static void sr_start_vddautocomp(struct am33xx_sr *sr)
{
	int i;

	if ((sr->sen[SR_CORE].opp_data[0].nvalue == 0) || 
                (sr->sen[SR_MPU].opp_data[0].nvalue == 0)) {
		dev_err(&sr->pdev->dev, "SR module not enabled, nTarget"
					" values are not found\n");
		return;
	}

	if (sr->autocomp_active == 1) {
		dev_warn(&sr->pdev->dev, "SR VDD autocomp already active\n");
		return;
	}

	for (i = 0; i < sr->no_of_sens; i++) {
               	/* Read current regulator value and voltage */
	        sr->sen[i].init_volt_mv = regulator_get_voltage(sr->sen[i].reg);

                dev_dbg(&sr->pdev->dev, "%s: regulator %d, init_volt = %d\n", 
                        __func__, i, sr->sen[i].init_volt_mv);

		if (sr_clk_enable(sr, i))
                        return;
		sr_configure(sr, i);
		sr_enable(sr, i);
	}

	sr->autocomp_active = 1;
}

/* sr_stop_vddautocomp - Stop VDD auto compensation
 * @sr:		contains SR driver data
 *
 * This is the ending point during SR disable from user space.
 * Also used to disable SR after OPP change.
 */
static void sr_stop_vddautocomp(struct am33xx_sr *sr)
{
	int i;

	if (sr->autocomp_active == 0) {
		dev_warn(&sr->pdev->dev, "SR VDD autocomp is not active\n");
		return;
	}

        /* cancel bottom half interrupt handlers that haven't run yet */
	cancel_delayed_work_sync(&sr->work);
	
	for (i = 0; i < sr->no_of_sens; i++) {
                /* cancel any outstanding SR IRQ re-enables on work queue */
                cancel_delayed_work_sync(&sr->sen[i].work_reenable);
		sr_disable(sr, i);
		sr_clk_disable(sr, i);
	}
	
	sr->autocomp_active = 0;
}

/* am33xx_sr_autocomp_show - Store user input value and stop SR
 * @data:		contains SR driver data
 * @val:		pointer to store autocomp_active status
 *
 * This is the Debug Fs enteries to show whether SR is enabled
 * or disabled
 */
static int am33xx_sr_autocomp_show(void *data, u64 *val)
{
	struct am33xx_sr *sr_info = (struct am33xx_sr *) data;

	*val = (u64) sr_info->autocomp_active;

	return 0;
}

static int am33xx_sr_margin_show(void *data, u64 *val)
{
        struct am33xx_sr_opp_data *sr_opp_data = (struct am33xx_sr_opp_data *)data;

	*val = (u64) sr_opp_data->margin;

	return 0;
}

static int am33xx_sr_margin_update(void *data, u64 val)
{
        struct am33xx_sr_opp_data *sr_opp_data = 
                (struct am33xx_sr_opp_data *)data;
        struct am33xx_sr_sensor *sr_sensor;
        struct am33xx_sr *sr_info;

        /* work back to the sr_info pointer */
        sr_sensor = container_of((void *)sr_opp_data, struct am33xx_sr_sensor, 
                opp_data[sr_opp_data->opp_id]); 

        sr_info = container_of((void *)sr_sensor, struct am33xx_sr, 
                sen[sr_sensor->sr_id]);

        /* store the value of margin */
        sr_opp_data->margin = (s32)val;

        dev_warn(&sr_info->pdev->dev, "%s: new margin=%d, srid=%d, opp=%d\n",
                __func__, sr_opp_data->margin, sr_sensor->sr_id, 
                sr_opp_data->opp_id);

        /* updata ntarget values based upon new margin */
        if (sr_set_nvalues(sr_info, sr_sensor->sr_id) == -EINVAL)
                dev_err(&sr_info->pdev->dev,
                        "%s: Zero NValue read from EFUSE\n", __func__);

        /* restart SmartReflex to adapt to new values */
        sr_stop_vddautocomp(sr_info);
        sr_start_vddautocomp(sr_info);

        return 0;
}

/* am33xx_sr_autocomp_store - Store user input and start SR
 * @data:		contains SR driver data
 * @val:		contains the value pased by user
 *
 * This is the Debug Fs enteries to store user input and
 * enable smartreflex.
 */
static int am33xx_sr_autocomp_store(void *data, u64 val)
{
	struct am33xx_sr *sr_info = (struct am33xx_sr *) data;

	/* Sanity check */
	if (val && (val != 1)) {
		dev_warn(&sr_info->pdev->dev, "%s: Invalid argument %llu\n",
		        __func__, val);
		return -EINVAL;
	}

	if (!val) {
                sr_info->disabled_by_user = 1;
		sr_stop_vddautocomp(sr_info);
        }
	else {
                sr_info->disabled_by_user = 0;
		sr_start_vddautocomp(sr_info);
        }

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(sr_fops, am33xx_sr_autocomp_show,
		am33xx_sr_autocomp_store, "%llu\n");

/* sr_curr_volt_show - Show current voltage value
 * @data:		contains SR driver data
 * @val:		pointer to store current voltage value
 *
 * Read the current voltage value and display the same on console
 * This is used in debugfs entries
 */
static int am33xx_sr_curr_volt_show(void *data, u64 *val)
{
	struct am33xx_sr_sensor *sr_sensor = (struct am33xx_sr_sensor *) data;

	*val = (u64) regulator_get_voltage(sr_sensor->reg);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(curr_volt_fops, am33xx_sr_curr_volt_show,
		NULL, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(margin_fops, am33xx_sr_margin_show,
		am33xx_sr_margin_update, "%llu\n");

#ifdef CONFIG_DEBUG_FS
/* sr_debugfs_entries - Create debugfs entries
 * @sr_info:		contains SR driver data
 *
 * Create debugfs entries, which is exposed to user for knowing
 * the current status. Some of the parameters can change during
 * run time
 */
static int sr_debugfs_entries(struct am33xx_sr *sr_info)
{
        struct am33xx_sr_sensor *sens;
	struct dentry *dbg_dir, *sen_dir, *opp_dir;
	int i, j;

	dbg_dir = debugfs_create_dir("smartreflex", NULL);
	if (IS_ERR(dbg_dir)) {
		dev_err(&sr_info->pdev->dev, "%s: Unable to create debugfs"
				" directory\n", __func__);
		return PTR_ERR(dbg_dir);
	}

	(void) debugfs_create_file("autocomp", S_IRUGO | S_IWUGO, dbg_dir,
				(void *)sr_info, &sr_fops);
        (void) debugfs_create_u32("interrupt_delay", S_IRUGO | S_IWUGO,
				dbg_dir, &sr_info->irq_delay);

	for (i = 0; i < sr_info->no_of_sens; i++) {
                sens = &sr_info->sen[i];
		sen_dir = debugfs_create_dir(sens->name, dbg_dir);
		if (IS_ERR(sen_dir)) {
			dev_err(&sr_info->pdev->dev, "%s: Unable to create"
				" debugfs directory\n", __func__);
			return PTR_ERR(sen_dir);
		}

                (void)debugfs_create_u32("initial_voltage", S_IRUGO, sen_dir,
				&sens->init_volt_mv);
	        (void)debugfs_create_file("current_voltage", S_IRUGO, sen_dir,
				(void *)sens, &curr_volt_fops);
		
                for (j = 0; j < sr_info->sen[i].no_of_opps; j++) {
                        char tmp[20];

                        sprintf(&tmp[0], "opp%d", j);
                        opp_dir = debugfs_create_dir(tmp, sen_dir);
                        if (IS_ERR(opp_dir)) {
        			dev_err(&sr_info->pdev->dev, 
                                        "%s: Unable to create debugfs directory\n", 
                                        __func__);
        			return PTR_ERR(opp_dir);
        		}

                        (void)debugfs_create_file("margin", S_IRUGO | S_IWUGO,
	        	       opp_dir, (void *)&sens->opp_data[j], 
                               &margin_fops);
                        (void)debugfs_create_x32("err2voltgain", 
                               S_IRUGO | S_IWUGO,
        		       opp_dir, 
                               &sens->opp_data[j].e2v_gain);
        		(void)debugfs_create_x32("nvalue", S_IRUGO,
        		       opp_dir, 
                               &sens->opp_data[j].nvalue);
                        (void)debugfs_create_x32("adj_nvalue", S_IRUGO,
        		       opp_dir, 
                               &sens->opp_data[j].adj_nvalue);
                }
	}
	return 0;
}
#else
static int sr_debugfs_entries(struct am33xx_sr *sr_info)
{
	return 0;
}
#endif

#ifdef CONFIG_CPU_FREQ

/* Find and return current OPP.  This should change to use system APIs,
   but voltdm is not currently populated, and opp APIs are also not working. */
static int get_current_opp(struct am33xx_sr *sr, u32 srid, u32 freq)  {
        int i;

        for (i = 0; i < sr->sen[srid].no_of_opps; i++) {
                if (sr->sen[srid].opp_data[i].frequency == freq)
                        return i;
        }

        return -EINVAL;
}

static int am33xx_sr_cpufreq_transition(struct notifier_block *nb,
					  unsigned long val, void *data)
{
        struct am33xx_sr *sr;
        struct cpufreq_freqs *cpu;

	sr = container_of(nb, struct am33xx_sr, freq_transition);

        /* We are required to disable SR while OPP change is occurring */
	if (val == CPUFREQ_PRECHANGE) {
                dev_dbg(&sr->pdev->dev, "%s: prechange\n", __func__);
                sr_stop_vddautocomp(sr);
	} else if (val == CPUFREQ_POSTCHANGE) {
                cpu = (struct cpufreq_freqs *)data;
                dev_dbg(&sr->pdev->dev, 
                        "%s: postchange, cpu=%d, old=%d, new=%d\n", 
                        __func__, cpu->cpu, cpu->old, cpu->new);

                /* update current OPP */
                sr->sen[SR_MPU].curr_opp = get_current_opp(sr, SR_MPU, 
                        cpu->new*1000);
                if (sr->sen[SR_MPU].curr_opp == -EINVAL) {
                        dev_err(&sr->pdev->dev, "%s: cannot determine opp\n",
                                __func__);
                        return -EINVAL;
                }

                dev_dbg(&sr->pdev->dev, "%s: postchange, new opp=%d\n", 
                        __func__, sr->sen[SR_MPU].curr_opp);

                /* this handles the case when the user has disabled SR via 
                   debugfs, therefore we do not want to enable SR */
                if (sr->disabled_by_user == 0)
                        sr_start_vddautocomp(sr);
	}

	return 0;
}

static inline int am33xx_sr_cpufreq_register(struct am33xx_sr *sr)
{
        sr->freq_transition.notifier_call = am33xx_sr_cpufreq_transition;

	return cpufreq_register_notifier(&sr->freq_transition,
					 CPUFREQ_TRANSITION_NOTIFIER);
}

static inline void am33xx_sr_cpufreq_deregister(struct am33xx_sr *sr)
{
	cpufreq_unregister_notifier(&sr->freq_transition,
				    CPUFREQ_TRANSITION_NOTIFIER);
}

#endif

static int __init am33xx_sr_probe(struct platform_device *pdev)
{
	struct am33xx_sr *sr_info;
	struct am33xx_sr_platform_data *pdata;
	struct resource *res[MAX_SENSORS];
	int irq;
	int ret;
	int i,j;

	sr_info = kzalloc(sizeof(struct am33xx_sr), GFP_KERNEL);
	if (!sr_info) {
		dev_err(&pdev->dev, "%s: unable to allocate sr_info\n",
					__func__);
		return -ENOMEM;
	}

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "%s: platform data missing\n", __func__);
		ret = -EINVAL;
		goto err_free_sr_info;
	}

	sr_info->pdev = pdev;
	sr_info->sen[SR_CORE].name = "smartreflex0";
	sr_info->sen[SR_MPU].name = "smartreflex1";
	sr_info->ip_type = pdata->ip_type;
        sr_info->irq_delay = pdata->irq_delay;
        sr_info->no_of_sens = pdata->no_of_sens;
        sr_info->no_of_vds = pdata->no_of_vds;
	sr_info->uvoltage_step_size = pdata->vstep_size_uv;
	sr_info->autocomp_active = false;
        sr_info->disabled_by_user = false;
        	
	for (i = 0; i < sr_info->no_of_sens; i++) {
                u32 curr_freq=0;

                sr_info->sen[i].reg_name = pdata->vd_name[i];

                /* this should be determined from voltdm or opp layer, but
                   those approaches are not working */
                sr_info->sen[i].no_of_opps = pdata->sr_sdata[i].no_of_opps;  
                sr_info->sen[i].sr_id = i;

                /* Reading per OPP Values */
                for (j = 0; j < sr_info->sen[i].no_of_opps; j++) {
        		sr_info->sen[i].opp_data[j].efuse_offs = 
                                pdata->sr_sdata[i].sr_opp_data[j].efuse_offs;
                        sr_info->sen[i].opp_data[j].e2v_gain = 
                                pdata->sr_sdata[i].sr_opp_data[j].e2v_gain;
        		sr_info->sen[i].opp_data[j].err_weight = 
                                pdata->sr_sdata[i].sr_opp_data[j].err_weight;
        		sr_info->sen[i].opp_data[j].err_minlimit = 
                                pdata->sr_sdata[i].sr_opp_data[j].err_minlimit;
        		sr_info->sen[i].opp_data[j].err_maxlimit = 
                                pdata->sr_sdata[i].sr_opp_data[j].err_maxlimit;	  
                        sr_info->sen[i].opp_data[j].margin = 
                                pdata->sr_sdata[i].sr_opp_data[j].margin; 
                        sr_info->sen[i].opp_data[j].nominal_volt = 
                                pdata->sr_sdata[i].sr_opp_data[j].nominal_volt; 
                        sr_info->sen[i].opp_data[j].frequency = 
                                pdata->sr_sdata[i].sr_opp_data[j].frequency;  
                        sr_info->sen[i].opp_data[j].opp_id = j;  	
                }

                if (i == SR_MPU) {
                        /* hardcoded CPU NR */
                        curr_freq = cpufreq_get(0); 
                                
                        /* update current OPP */
                        sr_info->sen[i].curr_opp = get_current_opp(sr_info, i, 
                                        curr_freq*1000);
                        if (sr_info->sen[i].curr_opp == -EINVAL) {
                                dev_err(&sr_info->pdev->dev, 
                                        "%s: cannot determine opp\n",__func__);
                                ret = -EINVAL;
                                goto err_free_sr_info;
                        }
                } else {
                        sr_info->sen[i].curr_opp = 
                                pdata->sr_sdata[i].default_opp;
                }   

                dev_dbg(&pdev->dev, 
                        "%s: SR%d, curr_opp=%d, no_of_opps=%d, step_size=%d\n",
                        __func__, i, sr_info->sen[i].curr_opp, 
                        sr_info->sen[i].no_of_opps, 
                        sr_info->uvoltage_step_size);

                ret = sr_set_nvalues(sr_info, i);
                if (ret == -EINVAL) {
                        dev_err(&sr_info->pdev->dev,
                                "%s: Zero NValue read from EFUSE\n", __func__);
                        goto err_free_sr_info;
                }

                INIT_DELAYED_WORK(&sr_info->sen[i].work_reenable, 
                        irq_sr_reenable);

		sr_info->res_name[i] = kzalloc(CLK_NAME_LEN + 1, GFP_KERNEL);

		/* resources */
		res[i] = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					sr_info->sen[i].name);
		if (!res[i]) {
			dev_err(&pdev->dev, "%s: no mem resource\n", __func__);
			ret = -ENOENT;
			goto err_free_mem;
		}

		irq = platform_get_irq_byname(pdev, sr_info->sen[i].name);
		if (irq < 0) {
			dev_err(&pdev->dev, "Can't get interrupt resource\n");
			ret = irq;
			goto err_free_mem;
		}
		sr_info->sen[i].irq = irq;

		res[i] = request_mem_region(res[i]->start,
				resource_size(res[i]), pdev->name);
		if (!res[i]) {
			dev_err(&pdev->dev, "can't request mem region\n");
			ret = -EBUSY;
			goto err_free_mem;
		}

		sr_info->sen[i].base = ioremap(res[i]->start,
				resource_size(res[i]));
		if (!sr_info->sen[i].base) {
			dev_err(&pdev->dev, "%s: ioremap fail\n", __func__);
			ret = -ENOMEM;
			goto err_release_mem;
		}

		strcat(sr_info->res_name[i], sr_info->sen[i].name);
		strcat(sr_info->res_name[i], "_fck");

		sr_info->sen[i].fck = clk_get(NULL, sr_info->res_name[i]);
		if (IS_ERR(sr_info->sen[i].fck)) {
			dev_err(&pdev->dev, "%s: Could not get sr fck\n",
						__func__);
			ret = PTR_ERR(sr_info->sen[i].fck);
			goto err_unmap;
		}

		ret = request_irq(sr_info->sen[i].irq, sr_class2_irq,
			IRQF_DISABLED, sr_info->sen[i].name, 
                        (void *)&sr_info->sen[i]);
		if (ret) {
			dev_err(&pdev->dev, "%s: Could not install SR ISR\n",
						__func__);
			goto err_put_clock;
		}

		sr_info->sen[i].senn_en = pdata->sr_sdata[i].senn_mod;
		sr_info->sen[i].senp_en = pdata->sr_sdata[i].senp_mod;

                sr_info->sen[i].reg = 
                        regulator_get(NULL, sr_info->sen[i].reg_name);
               	if (IS_ERR(sr_info->sen[i].reg)) {
                        ret = -EINVAL;
	                goto err_free_irq;
                }

               	/* Read current regulator value and voltage */
	        sr_info->sen[i].init_volt_mv = 
                        regulator_get_voltage(sr_info->sen[i].reg);

                dev_dbg(&pdev->dev, "%s: regulator %d, init_volt = %d\n", 
                        __func__, i, sr_info->sen[i].init_volt_mv);
	} /* for() */

        /* set_voltage() will be used as the bottom half IRQ handler */
	INIT_DELAYED_WORK(&sr_info->work, set_voltage);

#ifdef CONFIG_CPU_FREQ
	ret = am33xx_sr_cpufreq_register(sr_info);
	if (ret) {
		dev_err(&pdev->dev, "failed to register cpufreq\n");
		goto err_reg_put;
	}
#endif

	/* debugfs entries */
	ret = sr_debugfs_entries(sr_info);
	if (ret)
		dev_warn(&pdev->dev, "%s: Debugfs entries are not created\n",
						__func__);

	platform_set_drvdata(pdev, sr_info);

	dev_info(&pdev->dev, "%s: Driver initialized\n", __func__);

        /* disabled_by_user used to ensure SR doesn't come on via CPUFREQ
           scaling if user has disabled SR via debugfs on enable_on_init */
	if (pdata->enable_on_init)
		sr_start_vddautocomp(sr_info);
        else
                sr_info->disabled_by_user = 1;

	return ret;

#ifdef CONFIG_CPU_FREQ
	am33xx_sr_cpufreq_deregister(sr_info);
#endif

err_reg_put:
        i--; /* back up i by one to walk back through the for loop */
        regulator_put(sr_info->sen[i].reg);
err_free_irq:
	free_irq(sr_info->sen[i].irq, (void *)sr_info);
err_put_clock:
	clk_put(sr_info->sen[i].fck);
err_unmap:
	iounmap(sr_info->sen[i].base);
err_release_mem:
	release_mem_region(res[i]->start, resource_size(res[i]));
err_free_mem:
        kfree(sr_info->res_name[i]);
        /* unwind back through the for loop */
        if (i != 0) {
                goto err_reg_put;
        }
        
err_free_sr_info:
	kfree(sr_info);
	return ret;
}

static int __devexit am33xx_sr_remove(struct platform_device *pdev)
{
	struct am33xx_sr *sr_info;
	struct resource *res[MAX_SENSORS];
	int irq;
	int i;

	sr_info = dev_get_drvdata(&pdev->dev);
	if (!sr_info) {
		dev_err(&pdev->dev, "%s: sr_info missing\n", __func__);
		return -EINVAL;
	}

	if (sr_info->autocomp_active)
		sr_stop_vddautocomp(sr_info);

#ifdef CONFIG_CPU_FREQ
	am33xx_sr_cpufreq_deregister(sr_info);
#endif

	for (i = 0; i < sr_info->no_of_sens; i++) {
                regulator_put(sr_info->sen[i].reg);
                irq = platform_get_irq_byname(pdev, sr_info->sen[i].name);
		free_irq(irq, (void *)sr_info);
		clk_put(sr_info->sen[i].fck);
		iounmap(sr_info->sen[i].base);
		res[i] = platform_get_resource_byname(pdev,
				IORESOURCE_MEM, sr_info->sen[i].name);
		release_mem_region(res[i]->start, resource_size(res[i]));
                kfree(sr_info->res_name[i]);
	}

	kfree(sr_info);

        dev_info(&pdev->dev, "%s: SR has been removed\n", __func__);
	return 0;
}

static struct platform_driver smartreflex_driver = {
	.driver		= {
		.name	= "smartreflex",
		.owner	= THIS_MODULE,
	},
	.remove		= am33xx_sr_remove,
};

static int __init sr_init(void)
{
	int ret;

	ret = platform_driver_probe(&smartreflex_driver, am33xx_sr_probe);
	if (ret) {
		pr_err("%s: platform driver register failed\n", __func__);
		return ret;
	}

	return 0;
}

static void __exit sr_exit(void)
{
	platform_driver_unregister(&smartreflex_driver);
}
late_initcall(sr_init);
module_exit(sr_exit);

MODULE_DESCRIPTION("AM33XX Smartreflex Class2 Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Texas Instruments Inc");
