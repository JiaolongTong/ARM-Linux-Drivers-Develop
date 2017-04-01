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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/mfd/core.h>
#include <linux/pm_runtime.h>
#include <linux/mfd/ti_tscadc.h>
#include <linux/input/ti_tsc.h>
#include <linux/platform_data/ti_adc.h>

static unsigned int tscadc_readl(struct ti_tscadc_dev *tsadc, unsigned int reg)
{
	return readl(tsadc->tscadc_base + reg);
}

static void tscadc_writel(struct ti_tscadc_dev *tsadc, unsigned int reg,
					unsigned int val)
{
	writel(val, tsadc->tscadc_base + reg);
}

static void tscadc_idle_config(struct ti_tscadc_dev *config)
{
	unsigned int idleconfig;

	idleconfig = TSCADC_STEPCONFIG_YNN | TSCADC_STEPCONFIG_INM_ADCREFM |
			TSCADC_STEPCONFIG_INP_ADCREFM | TSCADC_STEPCONFIG_YPN;

	tscadc_writel(config, TSCADC_REG_IDLECONFIG, idleconfig);
}

static int __devinit ti_tscadc_probe(struct platform_device *pdev)
{
	struct ti_tscadc_dev	*tscadc;
	struct resource		*res;
	struct clk		*clk;
	struct mfd_tscadc_board	*pdata = pdev->dev.platform_data;
	struct mfd_cell		*cell;
	int			err, ctrl, children = 0;
	int			clk_value, clock_rate;
	int			tsc_wires = 0, adc_channels = 0, total_channels;

	if (!pdata) {
		dev_err(&pdev->dev, "Could not find platform data\n");
		return -EINVAL;
	}

	if (pdata->adc_init)
		adc_channels = pdata->adc_init->adc_channels;

	if (pdata->tsc_init)
		tsc_wires = pdata->tsc_init->wires;

	total_channels = tsc_wires + adc_channels;

	if (total_channels > 8) {
		dev_err(&pdev->dev, "Number of i/p channels more than 8\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no memory resource defined.\n");
		return -EINVAL;
	}

	/* Allocate memory for device */
	tscadc = kzalloc(sizeof(struct ti_tscadc_dev), GFP_KERNEL);
	if (!tscadc) {
		dev_err(&pdev->dev, "failed to allocate memory.\n");
		return -ENOMEM;
	}

	res = request_mem_region(res->start, resource_size(res),
			pdev->name);
	if (!res) {
		dev_err(&pdev->dev, "failed to reserve registers.\n");
		err = -EBUSY;
		goto err_free_mem;
	}

	tscadc->tscadc_base = ioremap(res->start, resource_size(res));
	if (!tscadc->tscadc_base) {
		dev_err(&pdev->dev, "failed to map registers.\n");
		err = -ENOMEM;
		goto err_release_mem;
	}

	tscadc->irq = platform_get_irq(pdev, 0);
	if (tscadc->irq < 0) {
		dev_err(&pdev->dev, "no irq ID is specified.\n");
		return -ENODEV;
	}

	tscadc->dev = &pdev->dev;

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	/*
	 * The TSC_ADC_Subsystem has 2 clock domains
	 * OCP_CLK and ADC_CLK.
	 * The ADC clock is expected to run at target of 3MHz,
	 * and expected to capture 12-bit data at a rate of 200 KSPS.
	 * The TSC_ADC_SS controller design assumes the OCP clock is
	 * at least 6x faster than the ADC clock.
	 */
	clk = clk_get(&pdev->dev, "adc_tsc_fck");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "failed to get TSC fck\n");
		err = PTR_ERR(clk);
		goto err_fail;
	}
	clock_rate = clk_get_rate(clk);
	clk_put(clk);
	clk_value = clock_rate / ADC_CLK;
	/* TSCADC_CLKDIV needs to be configured to the value minus 1 */
	clk_value = clk_value - 1;
	tscadc_writel(tscadc, TSCADC_REG_CLKDIV, clk_value);

	/* Set the control register bits */
	ctrl = TSCADC_CNTRLREG_STEPCONFIGWRT |
			TSCADC_CNTRLREG_STEPID;
	if (pdata->tsc_init)
		ctrl |= TSCADC_CNTRLREG_4WIRE |
				TSCADC_CNTRLREG_TSCENB;
	tscadc_writel(tscadc, TSCADC_REG_CTRL, ctrl);

	/* Set register bits for Idle Config Mode */
	if (pdata->tsc_init)
		tscadc_idle_config(tscadc);

	/* Enable the TSC module enable bit */
	ctrl = tscadc_readl(tscadc, TSCADC_REG_CTRL);
	ctrl |= TSCADC_CNTRLREG_TSCSSENB;
	tscadc_writel(tscadc, TSCADC_REG_CTRL, ctrl);

	/* TSC Cell */
	if (pdata->tsc_init) {
		cell = &tscadc->cells[children];
		cell->name = "tsc";
		cell->platform_data = tscadc;
		cell->pdata_size = sizeof(*tscadc);
		children++;
	}

	/* ADC Cell */
	if (pdata->adc_init) {
		cell = &tscadc->cells[children];
		cell->name = "tiadc";
		cell->platform_data = tscadc;
		cell->pdata_size = sizeof(*tscadc);
		children++;
	}

	err = mfd_add_devices(&pdev->dev, pdev->id, tscadc->cells,
			children, NULL, 0);
	if (err < 0)
		goto err_fail;

	device_init_wakeup(&pdev->dev, true);
	platform_set_drvdata(pdev, tscadc);
	return 0;

err_fail:
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	iounmap(tscadc->tscadc_base);
err_release_mem:
	release_mem_region(res->start, resource_size(res));
	mfd_remove_devices(tscadc->dev);
err_free_mem:
	platform_set_drvdata(pdev, NULL);
	kfree(tscadc);
	return err;
}

static int __devexit ti_tscadc_remove(struct platform_device *pdev)
{
	struct ti_tscadc_dev	*tscadc = platform_get_drvdata(pdev);
	struct resource		*res;

	tscadc_writel(tscadc, TSCADC_REG_SE, 0x00);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iounmap(tscadc->tscadc_base);
	release_mem_region(res->start, resource_size(res));

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	mfd_remove_devices(tscadc->dev);
	kfree(tscadc);

	platform_set_drvdata(pdev, NULL);
	return 0;
}

static int tscadc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct ti_tscadc_dev	*tscadc_dev = platform_get_drvdata(pdev);

	tscadc_writel(tscadc_dev, TSCADC_REG_SE, 0x00);
	tscadc_dev->irqstat = tscadc_readl(tscadc_dev, TSCADC_REG_IRQENABLE);
	tscadc_dev->ctrl = tscadc_readl(tscadc_dev, TSCADC_REG_CTRL);
	pm_runtime_put_sync(&pdev->dev);
	return 0;
}

static int tscadc_resume(struct platform_device *pdev)
{
	struct ti_tscadc_dev	*tscadc_dev = platform_get_drvdata(pdev);
	struct mfd_tscadc_board	*pdata = pdev->dev.platform_data;
	unsigned int irq_read;

	pm_runtime_get_sync(&pdev->dev);

	/* context restore */
	irq_read = tscadc_readl(tscadc_dev, TSCADC_REG_IRQSTATUS);
	tscadc_writel(tscadc_dev, TSCADC_REG_IRQSTATUS, irq_read);

	tscadc_writel(tscadc_dev, TSCADC_REG_IRQENABLE, tscadc_dev->irqstat);
	if (pdata->tsc_init)
		tscadc_idle_config(tscadc_dev);
	tscadc_writel(tscadc_dev, TSCADC_REG_SE, TSCADC_STPENB_STEPENB_TC);
	tscadc_writel(tscadc_dev, TSCADC_REG_CTRL, tscadc_dev->ctrl);
	return 0;
}

static struct platform_driver ti_tscadc_driver = {
	.driver = {
		.name   = "ti_tscadc",
		.owner	= THIS_MODULE,
	},
	.probe	= ti_tscadc_probe,
	.remove	= __devexit_p(ti_tscadc_remove),
	.suspend = tscadc_suspend,
	.resume = tscadc_resume,
};

module_platform_driver(ti_tscadc_driver);

MODULE_DESCRIPTION("TI touchscreen / ADC MFD controller driver");
MODULE_AUTHOR("Rachna Patil <rachna@ti.com>");
MODULE_LICENSE("GPL");
