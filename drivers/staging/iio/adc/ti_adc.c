/*
 * TI ADC MFD driver
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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/sched.h>

#include "../iio.h"
#include "../sysfs.h"
#include "../buffer_generic.h"
#include "../ring_sw.h"

#include <linux/mfd/ti_tscadc.h>
#include <linux/platform_data/ti_adc.h>

struct adc_device {
	struct ti_tscadc_dev	*mfd_tscadc;
	struct iio_dev		*idev;
	struct work_struct	poll_work;
	wait_queue_head_t	wq_data_avail;
	int			channels;
	int			irq;
	bool			is_continuous_mode;
	u16			*buffer;
};

static unsigned int adc_readl(struct adc_device *adc, unsigned int reg)
{
	return readl(adc->mfd_tscadc->tscadc_base + reg);
}

static void adc_writel(struct adc_device *adc, unsigned int reg,
					unsigned int val)
{
	writel(val, adc->mfd_tscadc->tscadc_base + reg);
}

static void adc_step_config(struct adc_device *adc_dev, bool mode)
{
	unsigned int    stepconfig;
	int i, channels = 0, steps;

	/*
	 * There are 16 configurable steps and 8 analog input
	 * lines available which are shared between Touchscreen and ADC.
	 *
	 * Steps backwards i.e. from 16 towards 0 are used by ADC
	 * depending on number of input lines needed.
	 * Channel would represent which analog input
	 * needs to be given to ADC to digitalize data.
	 */

	steps = TOTAL_STEPS - adc_dev->channels;
	channels = TOTAL_CHANNELS - adc_dev->channels;

	if (mode == 0)
		stepconfig = TSCADC_STEPCONFIG_AVG_16 | TSCADC_STEPCONFIG_FIFO1;
	else
		stepconfig = TSCADC_STEPCONFIG_AVG_16 | TSCADC_STEPCONFIG_FIFO1
			| TSCADC_STEPCONFIG_MODE_SWCNT;

	for (i = (steps + 1); i <= TOTAL_STEPS; i++) {
		adc_writel(adc_dev, TSCADC_REG_STEPCONFIG(i),
				stepconfig | TSCADC_STEPCONFIG_INP(channels));
		adc_writel(adc_dev, TSCADC_REG_STEPDELAY(i),
				TSCADC_STEPCONFIG_OPENDLY);
		channels++;
	}
}

static ssize_t tiadc_show_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct adc_device *adc_dev = iio_priv(indio_dev);
	unsigned int tmp;

	tmp = adc_readl(adc_dev, TSCADC_REG_STEPCONFIG(TOTAL_STEPS));
	tmp &= TSCADC_STEPCONFIG_MODE(1);

	if (tmp == 0x00)
		return sprintf(buf, "oneshot\n");
	else if (tmp == 0x01)
		return sprintf(buf, "continuous\n");
	else
		return sprintf(buf, "Operation mode unknown\n");
}

static ssize_t tiadc_set_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct adc_device *adc_dev = iio_priv(indio_dev);
	unsigned int config;

	config = adc_readl(adc_dev, TSCADC_REG_CTRL);
	config &= ~(TSCADC_CNTRLREG_TSCSSENB);
	adc_writel(adc_dev, TSCADC_REG_CTRL, config);

	if (!strncmp(buf, "oneshot", 7))
		adc_dev->is_continuous_mode = false;
	else if (!strncmp(buf, "continuous", 10))
		adc_dev->is_continuous_mode = true;
	else {
		dev_err(dev, "Operational mode unknown\n");
		return -EINVAL;
	}

	adc_step_config(adc_dev, adc_dev->is_continuous_mode);

	config = adc_readl(adc_dev, TSCADC_REG_CTRL);
	adc_writel(adc_dev, TSCADC_REG_CTRL,
			(config | TSCADC_CNTRLREG_TSCSSENB));
	return count;
}

static IIO_DEVICE_ATTR(mode, S_IRUGO | S_IWUSR, tiadc_show_mode,
		tiadc_set_mode, 0);

static struct attribute *tiadc_attributes[] = {
	&iio_dev_attr_mode.dev_attr.attr,
	NULL,
};

static const struct attribute_group tiadc_attribute_group = {
	.attrs = tiadc_attributes,
};

static irqreturn_t tiadc_irq(int irq, void *private)
{
	struct iio_dev *idev = private;
	struct adc_device *adc_dev = iio_priv(idev);
	unsigned int status, config;

	status = adc_readl(adc_dev, TSCADC_REG_IRQSTATUS);
	if (status & TSCADC_IRQENB_FIFO1OVRRUN) {
		config = adc_readl(adc_dev, TSCADC_REG_CTRL);
		config &= ~(TSCADC_CNTRLREG_TSCSSENB);
		adc_writel(adc_dev, TSCADC_REG_CTRL, config);

		adc_writel(adc_dev, TSCADC_REG_IRQSTATUS,
				TSCADC_IRQENB_FIFO1OVRRUN |
				TSCADC_IRQENB_FIFO1UNDRFLW |
				TSCADC_IRQENB_FIFO1THRES);

		adc_writel(adc_dev, TSCADC_REG_CTRL,
			(config | TSCADC_CNTRLREG_TSCSSENB));
		return IRQ_HANDLED;
	} else if (status & TSCADC_IRQENB_FIFO1THRES) {
		adc_writel(adc_dev, TSCADC_REG_IRQCLR,
				TSCADC_IRQENB_FIFO1THRES);

		if (iio_buffer_enabled(idev)) {
			if (!work_pending(&adc_dev->poll_work))
				schedule_work(&adc_dev->poll_work);
		} else {
			wake_up_interruptible(&adc_dev->wq_data_avail);
		}
		return IRQ_HANDLED;
	} else {
		return IRQ_NONE;
	}
}

static void tiadc_poll_handler(struct work_struct *work_s)
{
	struct adc_device *adc_dev =
		container_of(work_s, struct adc_device, poll_work);
	struct iio_dev *idev = iio_priv_to_dev(adc_dev);
	struct iio_buffer *buffer = idev->buffer;
	unsigned int fifo1count, readx1;
	int i;
	u32 *iBuf;

	fifo1count = adc_readl(adc_dev, TSCADC_REG_FIFO1CNT);
	if (fifo1count * sizeof(u32) <
				buffer->access->get_bytes_per_datum(buffer)) {
		dev_err(adc_dev->mfd_tscadc->dev, "%s: Short FIFO event\n",
								__func__);
		goto out;
	}

	iBuf = kmalloc(fifo1count * sizeof(u32), GFP_KERNEL);
	if (iBuf == NULL)
		goto out;

	for (i = 0; i < fifo1count; i++) {
		readx1 = adc_readl(adc_dev, TSCADC_REG_FIFO1);
		readx1 &= TSCADC_FIFOREAD_DATA_MASK;
		iBuf[i] = readx1;
	}

	buffer->access->store_to(buffer, (u8 *) iBuf, iio_get_time_ns());
	kfree(iBuf);

out:
	adc_writel(adc_dev, TSCADC_REG_IRQSTATUS,
				TSCADC_IRQENB_FIFO1THRES);
	adc_writel(adc_dev, TSCADC_REG_IRQENABLE,
				TSCADC_IRQENB_FIFO1THRES);
}

static int tiadc_buffer_preenable(struct iio_dev *idev)
{
	struct iio_buffer *buffer = idev->buffer;

	buffer->access->set_bytes_per_datum(buffer, 16);
	return 0;
}

static int tiadc_buffer_postenable(struct iio_dev *idev)
{
	struct adc_device *adc_dev = iio_priv(idev);
	struct iio_buffer *buffer = idev->buffer;
	unsigned int enb, config;
	int stepnum;
	u8 bit;

	if (!adc_dev->is_continuous_mode) {
		pr_info("Data cannot be read continuously in one shot mode\n");
		return -EINVAL;
	} else {

		config = adc_readl(adc_dev, TSCADC_REG_CTRL);
		adc_writel(adc_dev, TSCADC_REG_CTRL,
					config & ~TSCADC_CNTRLREG_TSCSSENB);
		adc_writel(adc_dev, TSCADC_REG_CTRL,
					config | TSCADC_CNTRLREG_TSCSSENB);


		adc_writel(adc_dev, TSCADC_REG_IRQSTATUS,
				TSCADC_IRQENB_FIFO1THRES |
				TSCADC_IRQENB_FIFO1OVRRUN |
				TSCADC_IRQENB_FIFO1UNDRFLW);
		adc_writel(adc_dev, TSCADC_REG_IRQENABLE,
				TSCADC_IRQENB_FIFO1THRES |
				TSCADC_IRQENB_FIFO1OVRRUN);

		adc_writel(adc_dev, TSCADC_REG_SE, 0x00);
		for_each_set_bit(bit, buffer->scan_mask,
				adc_dev->channels) {
			struct iio_chan_spec const *chan = idev->channels + bit;
			/*
			 * There are a total of 16 steps available
			 * that are shared between ADC and touchscreen.
			 * We start configuring from step 16 to 0 incase of
			 * ADC. Hence the relation between input channel
			 * and step for ADC would be as below.
			 */
			stepnum = chan->channel + 9;
			enb = adc_readl(adc_dev, TSCADC_REG_SE);
			enb |= (1 << stepnum);
			adc_writel(adc_dev, TSCADC_REG_SE, enb);
		}
		return 0;
	}
}

static int tiadc_buffer_postdisable(struct iio_dev *idev)
{
	struct adc_device *adc_dev = iio_priv(idev);

	adc_writel(adc_dev, TSCADC_REG_IRQCLR, (TSCADC_IRQENB_FIFO1THRES |
				TSCADC_IRQENB_FIFO1OVRRUN |
				TSCADC_IRQENB_FIFO1UNDRFLW));
	adc_writel(adc_dev, TSCADC_REG_SE, TSCADC_STPENB_STEPENB_TC);
	return 0;
}

static const struct iio_buffer_setup_ops tiadc_swring_setup_ops = {
	.preenable = &tiadc_buffer_preenable,
	.postenable = &tiadc_buffer_postenable,
	.postdisable = &tiadc_buffer_postdisable,
};

static int tiadc_config_sw_ring(struct iio_dev *idev)
{
	struct adc_device *adc_dev = iio_priv(idev);
	int ret;

	idev->buffer = iio_sw_rb_allocate(idev);
	if (!idev->buffer)
		ret = -ENOMEM;

	idev->buffer->access = &ring_sw_access_funcs;
	idev->buffer->setup_ops = &tiadc_swring_setup_ops;

	INIT_WORK(&adc_dev->poll_work, &tiadc_poll_handler);

	idev->modes |= INDIO_BUFFER_HARDWARE;
	return 0;
}

static int tiadc_channel_init(struct iio_dev *idev, struct adc_device *adc_dev)
{
	struct iio_chan_spec *chan_array;
	int i, channels;

	idev->num_channels = adc_dev->channels;
	chan_array = kcalloc(idev->num_channels, sizeof(struct iio_chan_spec),
					GFP_KERNEL);

	if (chan_array == NULL)
		return -ENOMEM;

	channels = TOTAL_CHANNELS - adc_dev->channels;
	for (i = 0; i < (idev->num_channels); i++) {
		struct iio_chan_spec *chan = chan_array + i;
		chan->type = IIO_VOLTAGE;
		chan->indexed = 1;
		chan->channel = channels;
		chan->scan_index = i;
		chan->scan_type.sign = 'u';
		chan->scan_type.realbits = 12;
		chan->scan_type.storagebits = 32;
		chan->scan_type.shift = 0;
		channels++;
	}

	idev->channels = chan_array;
	return idev->num_channels;
}

static void tiadc_channel_remove(struct iio_dev *idev)
{
	kfree(idev->channels);
}

static int tiadc_read_raw(struct iio_dev *idev,
		struct iio_chan_spec const *chan,
		int *val, int *val2, long mask)
{
	struct adc_device *adc_dev = iio_priv(idev);
	int i, map_val;
	unsigned int fifo1count, readx1, stepid;
	unsigned long timeout = jiffies + usecs_to_jiffies
			(IDLE_TIMEOUT * adc_dev->channels);

	if (adc_dev->is_continuous_mode) {
		pr_info("One shot mode not enabled\n");
		return -EINVAL;
	} else {
		adc_writel(adc_dev, TSCADC_REG_SE, TSCADC_STPENB_STEPENB);

		/* Wait for ADC sequencer to complete sampling */
		while (adc_readl(adc_dev, TSCADC_REG_ADCFSM) &
				TSCADC_SEQ_STATUS) {
			if (time_after(jiffies, timeout))
				return -EAGAIN;
		}

		map_val = chan->channel + TOTAL_CHANNELS;
		fifo1count = adc_readl(adc_dev, TSCADC_REG_FIFO1CNT);
		for (i = 0; i < fifo1count; i++) {
			readx1 = adc_readl(adc_dev, TSCADC_REG_FIFO1);
			stepid = readx1 & TSCADC_FIFOREAD_CHNLID_MASK;
			stepid = stepid >> 0x10;

			if (stepid == map_val) {
				readx1 = readx1 & TSCADC_FIFOREAD_DATA_MASK;
				*val = readx1;
			}
		}
		return IIO_VAL_INT;
	}
}

static const struct iio_info tiadc_info = {
	.read_raw = &tiadc_read_raw,
	.attrs = &tiadc_attribute_group,
};

static int __devinit tiadc_probe(struct platform_device *pdev)
{
	struct iio_dev		*idev;
	struct adc_device	*adc_dev = NULL;
	struct ti_tscadc_dev	*tscadc_dev = pdev->dev.platform_data;
	struct mfd_tscadc_board	*pdata;
	int			err;

	pdata = (struct mfd_tscadc_board *)tscadc_dev->dev->platform_data;
	if (!pdata || !pdata->adc_init)  {
		dev_err(tscadc_dev->dev, "Could not find platform data\n");
		return -EINVAL;
	}

	idev = iio_allocate_device(sizeof(struct adc_device));
	if (idev == NULL) {
		dev_err(&pdev->dev, "failed to allocate iio device.\n");
		err = -ENOMEM;
		goto err_ret;
	}
	adc_dev = iio_priv(idev);

	tscadc_dev->adc = adc_dev;
	adc_dev->mfd_tscadc = tscadc_dev;
	adc_dev->idev = idev;
	adc_dev->channels = pdata->adc_init->adc_channels;
	adc_dev->irq = tscadc_dev->irq;

	idev->dev.parent = &pdev->dev;
	idev->name = dev_name(&pdev->dev);
	idev->modes = INDIO_DIRECT_MODE;
	idev->info = &tiadc_info;

	/* by default driver comes up with oneshot mode */
	adc_step_config(adc_dev, adc_dev->is_continuous_mode);

	/* program FIFO threshold to value minus 1 */
	adc_writel(adc_dev, TSCADC_REG_FIFO1THR, FIFO1_THRESHOLD);

	err = tiadc_channel_init(idev, adc_dev);
	if (err < 0)
		goto err_free_device;

	init_waitqueue_head(&adc_dev->wq_data_avail);

	err = request_irq(adc_dev->irq, tiadc_irq, IRQF_SHARED,
		idev->name, idev);
	if (err)
		goto err_cleanup_channels;

	err = tiadc_config_sw_ring(idev);
	if (err)
		goto err_free_irq;

	err = iio_buffer_register(idev,
			idev->channels, idev->num_channels);
	if (err < 0)
		goto err_free_sw_rb;

	err = iio_device_register(idev);
	if (err)
		goto err_unregister;

	dev_info(&pdev->dev, "attached adc driver\n");
	platform_set_drvdata(pdev, idev);

	return 0;

err_unregister:
	iio_buffer_unregister(idev);
err_free_sw_rb:
	iio_sw_rb_free(idev->buffer);
err_free_irq:
	free_irq(adc_dev->irq, idev);
err_cleanup_channels:
	tiadc_channel_remove(idev);
err_free_device:
	iio_free_device(idev);
err_ret:
	return err;
}

static int __devexit tiadc_remove(struct platform_device *pdev)
{
	struct ti_tscadc_dev	*tscadc_dev = pdev->dev.platform_data;
	struct adc_device	*adc_dev = tscadc_dev->adc;
	struct iio_dev		*idev = adc_dev->idev;

	free_irq(adc_dev->irq, idev);
	iio_device_unregister(idev);
	iio_buffer_unregister(idev);
	iio_sw_rb_free(idev->buffer);
	tiadc_channel_remove(idev);

	tscadc_dev->adc = NULL;
	iio_free_device(idev);

	platform_set_drvdata(pdev, NULL);
	return 0;
}

static int adc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct ti_tscadc_dev   *tscadc_dev = pdev->dev.platform_data;
	struct adc_device	*adc_dev = tscadc_dev->adc;
	unsigned int idle;

	if (!device_may_wakeup(tscadc_dev->dev)) {
		idle = adc_readl(adc_dev, TSCADC_REG_CTRL);
		idle &= ~(TSCADC_CNTRLREG_TSCSSENB);
		adc_writel(adc_dev, TSCADC_REG_CTRL, (idle |
				TSCADC_CNTRLREG_POWERDOWN));
	}
	return 0;
}

static int adc_resume(struct platform_device *pdev)
{
	struct ti_tscadc_dev   *tscadc_dev = pdev->dev.platform_data;
	struct adc_device	*adc_dev = tscadc_dev->adc;
	unsigned int restore;

	restore = adc_readl(adc_dev, TSCADC_REG_CTRL);
	restore &= ~(TSCADC_CNTRLREG_TSCSSENB);
	adc_writel(adc_dev, TSCADC_REG_CTRL, restore);

	adc_writel(adc_dev, TSCADC_REG_FIFO1THR, FIFO1_THRESHOLD);
	adc_step_config(adc_dev, adc_dev->is_continuous_mode);

	/* Make sure ADC is powered up */
	restore &= ~(TSCADC_CNTRLREG_POWERDOWN);
	restore |= TSCADC_CNTRLREG_TSCSSENB;
	adc_writel(adc_dev, TSCADC_REG_CTRL, restore);
	return 0;
}

static struct platform_driver tiadc_driver = {
	.driver = {
		.name   = "tiadc",
		.owner = THIS_MODULE,
	},
	.probe          = tiadc_probe,
	.remove         = __devexit_p(tiadc_remove),
	.suspend = adc_suspend,
	.resume = adc_resume,
};

module_platform_driver(tiadc_driver);

MODULE_DESCRIPTION("TI ADC controller driver");
MODULE_AUTHOR("Rachna Patil <rachna@ti.com>");
MODULE_LICENSE("GPL");
