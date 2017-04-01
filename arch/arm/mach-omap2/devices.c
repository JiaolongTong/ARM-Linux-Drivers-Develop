/*
 * linux/arch/arm/mach-omap2/devices.c
 *
 * OMAP2 platform device setup/initialization
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/davinci_emac.h>
#include <linux/cpsw.h>
#include <linux/etherdevice.h>
#include <linux/dma-mapping.h>
#include <linux/can/platform/d_can.h>
#include <linux/platform_data/uio_pruss.h>
#include <linux/pwm/pwm.h>
#include <linux/input/ti_tsc.h>
#include <linux/mfd/ti_tscadc.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/board-am335xevm.h>
#include <asm/mach-types.h>
#include <asm/mach/map.h>
#include <asm/pmu.h>

#ifdef	CONFIG_OMAP3_EDMA
#include <mach/edma.h>
#endif

#include <asm/hardware/asp.h>

#include <plat/tc.h>
#include <plat/board.h>
#include <plat/mcbsp.h>
#include <plat/mmc.h>
#include <plat/dma.h>
#include <plat/omap_hwmod.h>
#include <plat/omap_device.h>
#include <plat/omap4-keypad.h>
#include <plat/am33xx.h>
#include <plat/config_pwm.h>
#include <plat/cpu.h>
#include <plat/gpmc.h>
#include <plat/smartreflex.h>
#include <plat/am33xx.h>

/* LCD controller similar DA8xx */
#include <video/da8xx-fb.h>

#include "mux.h"
#include "control.h"
#include "devices.h"
#include "omap_opp_data.h"

#define L3_MODULES_MAX_LEN 12
#define L3_MODULES 3

static unsigned int   am33xx_evmid;

/*
 * am33xx_evmid_fillup - set up board evmid
 * @evmid - evm id which needs to be configured
 *
 * This function is called to configure board evm id.
 * IA Motor Control EVM needs special setting of MAC PHY Id.
 * This function is called when IA Motor Control EVM is detected
 * during boot-up.
 */
void am33xx_evmid_fillup(unsigned int evmid)
{
       am33xx_evmid = evmid;
       return;
}

static int __init omap3_l3_init(void)
{
	int l;
	struct omap_hwmod *oh;
	struct platform_device *pdev;
	char oh_name[L3_MODULES_MAX_LEN];

	/*
	 * To avoid code running on other OMAPs in
	 * multi-omap builds
	 */
	if (!(cpu_is_omap34xx()) || (cpu_is_am33xx()))
		return -ENODEV;

	l = snprintf(oh_name, L3_MODULES_MAX_LEN, "l3_main");

	oh = omap_hwmod_lookup(oh_name);

	if (!oh)
		pr_err("could not look up %s\n", oh_name);

	pdev = omap_device_build("omap_l3_smx", 0, oh, NULL, 0,
							   NULL, 0, 0);

	WARN(IS_ERR(pdev), "could not build omap_device for %s\n", oh_name);

	return IS_ERR(pdev) ? PTR_ERR(pdev) : 0;
}
postcore_initcall(omap3_l3_init);

static int __init omap4_l3_init(void)
{
	int l, i;
	struct omap_hwmod *oh[3];
	struct platform_device *pdev;
	char oh_name[L3_MODULES_MAX_LEN];

	/* If dtb is there, the devices will be created dynamically */
	if (of_have_populated_dt())
		return -ENODEV;

	/*
	 * To avoid code running on other OMAPs in
	 * multi-omap builds
	 */
	if (!(cpu_is_omap44xx()))
		return -ENODEV;

	for (i = 0; i < L3_MODULES; i++) {
		l = snprintf(oh_name, L3_MODULES_MAX_LEN, "l3_main_%d", i+1);

		oh[i] = omap_hwmod_lookup(oh_name);
		if (!(oh[i]))
			pr_err("could not look up %s\n", oh_name);
	}

	pdev = omap_device_build_ss("omap_l3_noc", 0, oh, 3, NULL,
						     0, NULL, 0, 0);

	WARN(IS_ERR(pdev), "could not build omap_device for %s\n", oh_name);

	return IS_ERR(pdev) ? PTR_ERR(pdev) : 0;
}
postcore_initcall(omap4_l3_init);

#if defined(CONFIG_VIDEO_OMAP2) || defined(CONFIG_VIDEO_OMAP2_MODULE)

static struct resource omap2cam_resources[] = {
	{
		.start		= OMAP24XX_CAMERA_BASE,
		.end		= OMAP24XX_CAMERA_BASE + 0xfff,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= INT_24XX_CAM_IRQ,
		.flags		= IORESOURCE_IRQ,
	}
};

static struct platform_device omap2cam_device = {
	.name		= "omap24xxcam",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(omap2cam_resources),
	.resource	= omap2cam_resources,
};
#endif

int __init am33xx_register_lcdc(struct da8xx_lcdc_platform_data *pdata)
{
	int id = 0;
	struct platform_device *pdev;
	struct omap_hwmod *oh;
	char *oh_name = "lcdc";
	char *dev_name = "da8xx_lcdc";

	oh = omap_hwmod_lookup(oh_name);
	if (!oh) {
		pr_err("Could not look up LCD%d hwmod\n", id);
		return -ENODEV;
	}

	pdev = omap_device_build(dev_name, id, oh, pdata,
			sizeof(struct da8xx_lcdc_platform_data), NULL, 0, 0);
	if (IS_ERR(pdev)) {
		WARN(1, "Can't build omap_device for %s:%s.\n",
			dev_name, oh->name);
		return PTR_ERR(pdev);
	}
	return 0;
}

int __init am33xx_register_tsc(struct tsc_data *pdata)
{
	int id = -1;
	struct platform_device *pdev;
	struct omap_hwmod *oh;
	char *oh_name = "adc_tsc";
	char *dev_name = "tsc";

	oh = omap_hwmod_lookup(oh_name);
	if (!oh) {
		pr_err("Could not look up TSC%d hwmod\n", id);
		return -ENODEV;
	}

	pdev = omap_device_build(dev_name, id, oh, pdata,
			sizeof(struct tsc_data), NULL, 0, 0);

	WARN(IS_ERR(pdev), "Can't build omap_device for %s:%s.\n",
			dev_name, oh->name);
	return 0;
}

#if defined(CONFIG_SND_AM335X_SOC_EVM) || \
				defined(CONFIG_SND_AM335X_SOC_EVM_MODULE)
int __init am335x_register_mcasp(struct snd_platform_data *pdata, int ctrl_nr)
{
	int l;
	struct omap_hwmod *oh;
	struct platform_device *pdev;
	char oh_name[12];
	char *dev_name = "davinci-mcasp";

	l = snprintf(oh_name, 12, "mcasp%d", ctrl_nr);

	oh = omap_hwmod_lookup(oh_name);
	if (!oh) {
		pr_err("could not look up %s\n", oh_name);
		return -ENODEV;
	}

	pdev = omap_device_build(dev_name, ctrl_nr, oh, pdata,
			sizeof(struct snd_platform_data), NULL, 0, 0);
	WARN(IS_ERR(pdev), "Can't build omap_device for %s:%s.\n",
			dev_name, oh->name);
	return IS_ERR(pdev) ? PTR_ERR(pdev) : 0;
}

int __init am33xx_register_mfd_tscadc(struct mfd_tscadc_board *pdata)
{
	int id = -1;
	struct platform_device *pdev;
	struct omap_hwmod *oh;
	char *oh_name = "adc_tsc";
	char *dev_name = "ti_tscadc";

	oh = omap_hwmod_lookup(oh_name);
	if (!oh) {
		pr_err("Could not look up TSCADC%d hwmod\n", id);
		return -ENODEV;
	}

	pdev = omap_device_build(dev_name, id, oh, pdata,
							 sizeof(struct mfd_tscadc_board), NULL, 0, 0);

	WARN(IS_ERR(pdev), "Can't build omap_device for %s:%s.\n",
		 dev_name, oh->name);

	return 0;
}

#else
int __init am335x_register_mcasp(struct snd_platform_data *pdata, int ctrl_nr)
{
	return 0;
}
#endif

#if (defined(CONFIG_SND_AM33XX_SOC) || (defined(CONFIG_SND_AM33XX_SOC_MODULE)))
struct platform_device am33xx_pcm_device = {
	.name		= "davinci-pcm-audio",
	.id		= -1,
};

static void am33xx_init_pcm(void)
{
	platform_device_register(&am33xx_pcm_device);
}

#else
static inline void am33xx_init_pcm(void) {}
#endif

static struct resource omap3isp_resources[] = {
	{
		.start		= OMAP3430_ISP_BASE,
		.end		= OMAP3430_ISP_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_CCP2_BASE,
		.end		= OMAP3430_ISP_CCP2_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_CCDC_BASE,
		.end		= OMAP3430_ISP_CCDC_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_HIST_BASE,
		.end		= OMAP3430_ISP_HIST_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_H3A_BASE,
		.end		= OMAP3430_ISP_H3A_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_PREV_BASE,
		.end		= OMAP3430_ISP_PREV_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_RESZ_BASE,
		.end		= OMAP3430_ISP_RESZ_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_SBL_BASE,
		.end		= OMAP3430_ISP_SBL_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_CSI2A_REGS1_BASE,
		.end		= OMAP3430_ISP_CSI2A_REGS1_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_CSIPHY2_BASE,
		.end		= OMAP3430_ISP_CSIPHY2_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3630_ISP_CSI2A_REGS2_BASE,
		.end		= OMAP3630_ISP_CSI2A_REGS2_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3630_ISP_CSI2C_REGS1_BASE,
		.end		= OMAP3630_ISP_CSI2C_REGS1_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3630_ISP_CSIPHY1_BASE,
		.end		= OMAP3630_ISP_CSIPHY1_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3630_ISP_CSI2C_REGS2_BASE,
		.end		= OMAP3630_ISP_CSI2C_REGS2_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= INT_34XX_CAM_IRQ,
		.flags		= IORESOURCE_IRQ,
	}
};

static struct platform_device omap3isp_device = {
	.name		= "omap3isp",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(omap3isp_resources),
	.resource	= omap3isp_resources,
};

int omap3_init_camera(struct isp_platform_data *pdata)
{
	omap3isp_device.dev.platform_data = pdata;
	return platform_device_register(&omap3isp_device);
}

static inline void omap_init_camera(void)
{
#if defined(CONFIG_VIDEO_OMAP2) || defined(CONFIG_VIDEO_OMAP2_MODULE)
	if (cpu_is_omap24xx())
		platform_device_register(&omap2cam_device);
#endif
}

int __init omap4_keyboard_init(struct omap4_keypad_platform_data
			*sdp4430_keypad_data, struct omap_board_data *bdata)
{
	struct platform_device *pdev;
	struct omap_hwmod *oh;
	struct omap4_keypad_platform_data *keypad_data;
	unsigned int id = -1;
	char *oh_name = "kbd";
	char *name = "omap4-keypad";

	oh = omap_hwmod_lookup(oh_name);
	if (!oh) {
		pr_err("Could not look up %s\n", oh_name);
		return -ENODEV;
	}

	keypad_data = sdp4430_keypad_data;

	pdev = omap_device_build(name, id, oh, keypad_data,
			sizeof(struct omap4_keypad_platform_data), NULL, 0, 0);

	if (IS_ERR(pdev)) {
		WARN(1, "Can't build omap_device for %s:%s.\n",
						name, oh->name);
		return PTR_ERR(pdev);
	}
	oh->mux = omap_hwmod_mux_init(bdata->pads, bdata->pads_cnt);

	return 0;
}

#if defined(CONFIG_OMAP_MBOX_FWK) || defined(CONFIG_OMAP_MBOX_FWK_MODULE)
static inline void omap_init_mbox(void)
{
	struct omap_hwmod *oh;
	struct platform_device *pdev;

	oh = omap_hwmod_lookup("mailbox");
	if (!oh) {
		pr_err("%s: unable to find hwmod\n", __func__);
		return;
	}

	pdev = omap_device_build("omap-mailbox", -1, oh, NULL, 0, NULL, 0, 0);
	WARN(IS_ERR(pdev), "%s: could not build device, err %ld\n",
						__func__, PTR_ERR(pdev));
}
#else
static inline void omap_init_mbox(void) { }
#endif /* CONFIG_OMAP_MBOX_FWK */

static inline void omap_init_sti(void) {}

#if defined(CONFIG_SND_SOC) || defined(CONFIG_SND_SOC_MODULE)

static struct platform_device omap_pcm = {
	.name	= "omap-pcm-audio",
	.id	= -1,
};

/*
 * OMAP2420 has 2 McBSP ports
 * OMAP2430 has 5 McBSP ports
 * OMAP3 has 5 McBSP ports
 * OMAP4 has 4 McBSP ports
 */
OMAP_MCBSP_PLATFORM_DEVICE(1);
OMAP_MCBSP_PLATFORM_DEVICE(2);
OMAP_MCBSP_PLATFORM_DEVICE(3);
OMAP_MCBSP_PLATFORM_DEVICE(4);
OMAP_MCBSP_PLATFORM_DEVICE(5);

static void omap_init_audio(void)
{
	if (cpu_is_am33xx())
		return;

	platform_device_register(&omap_mcbsp1);
	platform_device_register(&omap_mcbsp2);
	if (cpu_is_omap243x() || cpu_is_omap34xx() || cpu_is_omap44xx()) {
		platform_device_register(&omap_mcbsp3);
		platform_device_register(&omap_mcbsp4);
	}
	if (cpu_is_omap243x() || cpu_is_omap34xx())
		platform_device_register(&omap_mcbsp5);

	platform_device_register(&omap_pcm);
}

#else
static inline void omap_init_audio(void) {}
#endif

#if defined(CONFIG_SND_OMAP_SOC_MCPDM) || \
		defined(CONFIG_SND_OMAP_SOC_MCPDM_MODULE)

static void omap_init_mcpdm(void)
{
	struct omap_hwmod *oh;
	struct platform_device *pdev;

	oh = omap_hwmod_lookup("mcpdm");
	if (!oh) {
		printk(KERN_ERR "Could not look up mcpdm hw_mod\n");
		return;
	}

	pdev = omap_device_build("omap-mcpdm", -1, oh, NULL, 0, NULL, 0, 0);
	WARN(IS_ERR(pdev), "Can't build omap_device for omap-mcpdm.\n");
}
#else
static inline void omap_init_mcpdm(void) {}
#endif

#if defined(CONFIG_SND_OMAP_SOC_DMIC) || \
		defined(CONFIG_SND_OMAP_SOC_DMIC_MODULE)

static void omap_init_dmic(void)
{
	struct omap_hwmod *oh;
	struct platform_device *pdev;

	oh = omap_hwmod_lookup("dmic");
	if (!oh) {
		printk(KERN_ERR "Could not look up mcpdm hw_mod\n");
		return;
	}

	pdev = omap_device_build("omap-dmic", -1, oh, NULL, 0, NULL, 0, 0);
	WARN(IS_ERR(pdev), "Can't build omap_device for omap-dmic.\n");
}
#else
static inline void omap_init_dmic(void) {}
#endif

#if defined(CONFIG_SPI_OMAP24XX) || defined(CONFIG_SPI_OMAP24XX_MODULE)

#include <plat/mcspi.h>

static int omap_mcspi_init(struct omap_hwmod *oh, void *unused)
{
	struct platform_device *pdev;
	char *name = "omap2_mcspi";
	struct omap2_mcspi_platform_config *pdata;
	static int spi_num;
	struct omap2_mcspi_dev_attr *mcspi_attrib = oh->dev_attr;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		pr_err("Memory allocation for McSPI device failed\n");
		return -ENOMEM;
	}

	pdata->num_cs = mcspi_attrib->num_chipselect;
	switch (oh->class->rev) {
	case OMAP2_MCSPI_REV:
	case OMAP3_MCSPI_REV:
			pdata->regs_offset = 0;
			break;
	case OMAP4_MCSPI_REV:
			pdata->regs_offset = OMAP4_MCSPI_REG_OFFSET;
			break;
	default:
			pr_err("Invalid McSPI Revision value\n");
			return -EINVAL;
	}

	spi_num++;
	pdev = omap_device_build(name, spi_num, oh, pdata,
				sizeof(*pdata),	NULL, 0, 0);
	WARN(IS_ERR(pdev), "Can't build omap_device for %s:%s\n",
				name, oh->name);
	kfree(pdata);
	return 0;
}

static void omap_init_mcspi(void)
{
	omap_hwmod_for_each_by_class("mcspi", omap_mcspi_init, NULL);
}

#else
static inline void omap_init_mcspi(void) {}
#endif

int __init omap_init_elm(void)
{
	int id = -1;
	struct platform_device *pdev;
	struct omap_hwmod *oh;
	char *oh_name = "elm";
	char *name = "omap2_elm";

	oh = omap_hwmod_lookup(oh_name);
	if (!oh) {
		pr_err("Could not look up %s\n", oh_name);
		return -ENODEV;
	}

	pdev = omap_device_build(name, id, oh, NULL, 0, NULL, 0, 0);

	if (IS_ERR(pdev)) {
		WARN(1, "Can't build omap_device for %s:%s.\n",
						name, oh->name);
		return PTR_ERR(pdev);
	}

	return 0;
}

#ifdef CONFIG_SOC_OMAPAM33XX
#define PWM_STR_LEN 10
int __init am33xx_register_ecap(int id, struct pwmss_platform_data *pdata)
{
	struct platform_device *pdev;
	struct omap_hwmod *oh;
	char *oh_name = "ecap";
	char dev_name[PWM_STR_LEN];

	sprintf(dev_name, "ecap.%d", id);

	oh = omap_hwmod_lookup(dev_name);
	if (!oh) {
		pr_err("Could not look up %s hwmod\n", dev_name);
		return -ENODEV;
	}

	pdev = omap_device_build(oh_name, id, oh, pdata,
			sizeof(*pdata), NULL, 0, 0);

	if (IS_ERR(pdev)) {
		WARN(1, "Can't build omap_device for %s:%s.\n",
			dev_name, oh->name);
		return PTR_ERR(pdev);
	}
	return 0;
}

int __init am33xx_register_ehrpwm(int id, struct pwmss_platform_data *pdata)
{
	struct platform_device *pdev;
	struct omap_hwmod *oh;
	char *oh_name = "ehrpwm";
	char dev_name[PWM_STR_LEN];

	sprintf(dev_name, "ehrpwm.%d", id);

	oh = omap_hwmod_lookup(dev_name);
	if (!oh) {
		pr_err("Could not look up %s hwmod\n", dev_name);
		return -ENODEV;
	}

	pdev = omap_device_build(oh_name, id, oh, pdata,
			sizeof(*pdata), NULL, 0, 0);

	if (IS_ERR(pdev)) {
		WARN(1, "Can't build omap_device for %s:%s.\n",
			dev_name, oh->name);
		return PTR_ERR(pdev);
	}
	return 0;
}

#else
static int __init am335x_register_ehrpwm(int id,
		struct pwmss_platform_data *pdata) { }
static int __init am335x_register_ecap(int id,
		struct pwmss_platform_data *pdata) { }
#endif

static struct resource omap2_pmu_resource = {
	.start	= 3,
	.end	= 3,
	.flags	= IORESOURCE_IRQ,
};

static struct resource omap3_pmu_resource = {
	.start	= INT_34XX_BENCH_MPU_EMUL,
	.end	= INT_34XX_BENCH_MPU_EMUL,
	.flags	= IORESOURCE_IRQ,
};

static struct platform_device omap_pmu_device = {
	.name		= "arm-pmu",
	.id		= ARM_PMU_DEVICE_CPU,
	.num_resources	= 1,
};

static void omap_init_pmu(void)
{
	if (cpu_is_omap24xx())
		omap_pmu_device.resource = &omap2_pmu_resource;
	else if (cpu_is_omap34xx() && !cpu_is_am33xx())
		omap_pmu_device.resource = &omap3_pmu_resource;
	else
		return;

	platform_device_register(&omap_pmu_device);
}


#if defined(CONFIG_CRYPTO_DEV_OMAP_SHAM) || defined(CONFIG_CRYPTO_DEV_OMAP_SHAM_MODULE)

#ifdef CONFIG_ARCH_OMAP2
static struct resource omap2_sham_resources[] = {
	{
		.start	= OMAP24XX_SEC_SHA1MD5_BASE,
		.end	= OMAP24XX_SEC_SHA1MD5_BASE + 0x64,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_24XX_SHA1MD5,
		.flags	= IORESOURCE_IRQ,
	}
};
static int omap2_sham_resources_sz = ARRAY_SIZE(omap2_sham_resources);
#else
#define omap2_sham_resources		NULL
#define omap2_sham_resources_sz		0
#endif

#ifdef CONFIG_ARCH_OMAP3
static struct resource omap3_sham_resources[] = {
	{
		.start	= OMAP34XX_SEC_SHA1MD5_BASE,
		.end	= OMAP34XX_SEC_SHA1MD5_BASE + 0x64,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_34XX_SHA1MD52_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= OMAP34XX_DMA_SHA1MD5_RX,
		.flags	= IORESOURCE_DMA,
	}
};
static int omap3_sham_resources_sz = ARRAY_SIZE(omap3_sham_resources);
#else
#define omap3_sham_resources		NULL
#define omap3_sham_resources_sz		0
#endif

static struct platform_device sham_device = {
	.name		= "omap-sham",
	.id		= -1,
};

static void omap_init_sham(void)
{
	if (cpu_is_omap24xx()) {
		sham_device.resource = omap2_sham_resources;
		sham_device.num_resources = omap2_sham_resources_sz;
	} else if (cpu_is_omap34xx() && !cpu_is_am33xx()) {
		sham_device.resource = omap3_sham_resources;
		sham_device.num_resources = omap3_sham_resources_sz;
	} else {
		pr_err("%s: platform not supported\n", __func__);
		return;
	}
	platform_device_register(&sham_device);
}

#elif defined(CONFIG_CRYPTO_DEV_OMAP4_SHAM) || defined(CONFIG_CRYPTO_DEV_OMAP4_SHAM_MODULE)

static struct resource omap4_sham_resources[] = {
	{
		.start	= AM33XX_SHA1MD5_P_BASE,
		.end	= AM33XX_SHA1MD5_P_BASE + 0x120,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= AM33XX_IRQ_SHAEIP57t0_P,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= AM33XX_DMA_SHAEIP57T0_DIN,
		.flags	= IORESOURCE_DMA,
	}
};

static int omap4_sham_resources_sz = ARRAY_SIZE(omap4_sham_resources);


static struct platform_device sham_device = {
	.name		= "omap4-sham",
	.id		= -1,
};

#if 0
static void omap_init_sham(void)
{
	sham_device.resource = omap4_sham_resources;
	sham_device.num_resources = omap4_sham_resources_sz;

	platform_device_register(&sham_device);
}
#endif

int __init omap_init_sham(void)
{
	int id = -1;
	struct platform_device *pdev;
	struct omap_hwmod *oh;
	char *oh_name = "sha0";
	char *name = "omap4-sham";

	oh = omap_hwmod_lookup(oh_name);
	if (!oh) {
		pr_err("Could not look up %s\n", oh_name);
		return -ENODEV;
	}

	pdev = omap_device_build(name, id, oh, NULL, 0, NULL, 0, 0);
	//pdev.resource = omap4_sham_resources;
	//pdev.num_resources = omap4_sham_resources_sz;

	if (IS_ERR(pdev)) {
		WARN(1, "Can't build omap_device for %s:%s.\n",
						name, oh->name);
		return PTR_ERR(pdev);
	}

	return 0;
}





#else
static inline void omap_init_sham(void) { }
#endif

#if defined(CONFIG_CRYPTO_DEV_OMAP_AES) || defined(CONFIG_CRYPTO_DEV_OMAP_AES_MODULE)

#ifdef CONFIG_ARCH_OMAP2
static struct resource omap2_aes_resources[] = {
	{
		.start	= OMAP24XX_SEC_AES_BASE,
		.end	= OMAP24XX_SEC_AES_BASE + 0x4C,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= OMAP24XX_DMA_AES_TX,
		.flags	= IORESOURCE_DMA,
	},
	{
		.start	= OMAP24XX_DMA_AES_RX,
		.flags	= IORESOURCE_DMA,
	}
};
static int omap2_aes_resources_sz = ARRAY_SIZE(omap2_aes_resources);
#else
#define omap2_aes_resources		NULL
#define omap2_aes_resources_sz		0
#endif

#ifdef CONFIG_ARCH_OMAP3
static struct resource omap3_aes_resources[] = {
	{
		.start	= OMAP34XX_SEC_AES_BASE,
		.end	= OMAP34XX_SEC_AES_BASE + 0x4C,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= OMAP34XX_DMA_AES2_TX,
		.flags	= IORESOURCE_DMA,
	},
	{
		.start	= OMAP34XX_DMA_AES2_RX,
		.flags	= IORESOURCE_DMA,
	}
};
static int omap3_aes_resources_sz = ARRAY_SIZE(omap3_aes_resources);
#else
#define omap3_aes_resources		NULL
#define omap3_aes_resources_sz		0
#endif

static struct platform_device aes_device = {
	.name		= "omap-aes",
	.id		= -1,
};

static void omap_init_aes(void)
{
	if (cpu_is_omap24xx()) {
		aes_device.resource = omap2_aes_resources;
		aes_device.num_resources = omap2_aes_resources_sz;
	} else if (cpu_is_omap34xx() && !cpu_is_am33xx()) {
		aes_device.resource = omap3_aes_resources;
		aes_device.num_resources = omap3_aes_resources_sz;
	} else {
		pr_err("%s: platform not supported\n", __func__);
		return;
	}
	platform_device_register(&aes_device);
}

#elif defined(CONFIG_CRYPTO_DEV_OMAP4_AES) || defined(CONFIG_CRYPTO_DEV_OMAP4_AES_MODULE)

static struct resource omap4_aes_resources[] = {
	{
		.start	= AM33XX_AES0_P_BASE,
		.end	= AM33XX_AES0_P_BASE + 0x4C,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= AM33XX_DMA_AESEIP36T0_DOUT,
		.flags	= IORESOURCE_DMA,
	},
	{
		.start	= AM33XX_DMA_AESEIP36T0_DIN,
		.flags	= IORESOURCE_DMA,
	}
};
static int omap4_aes_resources_sz = ARRAY_SIZE(omap4_aes_resources);

static struct platform_device aes_device = {
	.name		= "omap4-aes",
	.id		= -1,
};

#if 0
static void omap_init_aes(void)
{
	aes_device.resource = omap4_aes_resources;
	aes_device.num_resources = omap4_aes_resources_sz;
	platform_device_register(&aes_device);
}
#endif

int __init omap_init_aes(void)
{
	int id = -1;
	struct platform_device *pdev;
	struct omap_hwmod *oh;
	char *oh_name = "aes0";
	char *name = "omap4-aes";

	oh = omap_hwmod_lookup(oh_name);
	if (!oh) {
		pr_err("Could not look up %s\n", oh_name);
		return -ENODEV;
	}

	pdev = omap_device_build(name, id, oh, NULL, 0, NULL, 0, 0);
	//pdev.resource = omap4_sham_resources;
	//pdev.num_resources = omap4_sham_resources_sz;

	if (IS_ERR(pdev)) {
		WARN(1, "Can't build omap_device for %s:%s.\n",
						name, oh->name);
		return PTR_ERR(pdev);
	}

	return 0;
}





#else
static inline void omap_init_aes(void) { }
#endif

/*-------------------------------------------------------------------------*/

#if defined(CONFIG_MMC_OMAP) || defined(CONFIG_MMC_OMAP_MODULE)

static inline void omap242x_mmc_mux(struct omap_mmc_platform_data
							*mmc_controller)
{
	if ((mmc_controller->slots[0].switch_pin > 0) && \
		(mmc_controller->slots[0].switch_pin < OMAP_MAX_GPIO_LINES))
		omap_mux_init_gpio(mmc_controller->slots[0].switch_pin,
					OMAP_PIN_INPUT_PULLUP);
	if ((mmc_controller->slots[0].gpio_wp > 0) && \
		(mmc_controller->slots[0].gpio_wp < OMAP_MAX_GPIO_LINES))
		omap_mux_init_gpio(mmc_controller->slots[0].gpio_wp,
					OMAP_PIN_INPUT_PULLUP);

	omap_mux_init_signal("sdmmc_cmd", 0);
	omap_mux_init_signal("sdmmc_clki", 0);
	omap_mux_init_signal("sdmmc_clko", 0);
	omap_mux_init_signal("sdmmc_dat0", 0);
	omap_mux_init_signal("sdmmc_dat_dir0", 0);
	omap_mux_init_signal("sdmmc_cmd_dir", 0);
	if (mmc_controller->slots[0].caps & MMC_CAP_4_BIT_DATA) {
		omap_mux_init_signal("sdmmc_dat1", 0);
		omap_mux_init_signal("sdmmc_dat2", 0);
		omap_mux_init_signal("sdmmc_dat3", 0);
		omap_mux_init_signal("sdmmc_dat_dir1", 0);
		omap_mux_init_signal("sdmmc_dat_dir2", 0);
		omap_mux_init_signal("sdmmc_dat_dir3", 0);
	}

	/*
	 * Use internal loop-back in MMC/SDIO Module Input Clock
	 * selection
	 */
	if (mmc_controller->slots[0].internal_clock) {
		u32 v = omap_ctrl_readl(OMAP2_CONTROL_DEVCONF0);
		v |= (1 << 24);
		omap_ctrl_writel(v, OMAP2_CONTROL_DEVCONF0);
	}
}

void __init omap242x_init_mmc(struct omap_mmc_platform_data **mmc_data)
{
	char *name = "mmci-omap";

	if (!mmc_data[0]) {
		pr_err("%s fails: Incomplete platform data\n", __func__);
		return;
	}

	omap242x_mmc_mux(mmc_data[0]);
	omap_mmc_add(name, 0, OMAP2_MMC1_BASE, OMAP2420_MMC_SIZE,
					INT_24XX_MMC_IRQ, mmc_data[0]);
}

#endif

/*-------------------------------------------------------------------------*/

#if defined(CONFIG_HDQ_MASTER_OMAP) || defined(CONFIG_HDQ_MASTER_OMAP_MODULE)
#if defined(CONFIG_SOC_OMAP2430) || defined(CONFIG_SOC_OMAP3430)
#define OMAP_HDQ_BASE	0x480B2000
#endif
static struct resource omap_hdq_resources[] = {
	{
		.start		= OMAP_HDQ_BASE,
		.end		= OMAP_HDQ_BASE + 0x1C,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= INT_24XX_HDQ_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
};
static struct platform_device omap_hdq_dev = {
	.name = "omap_hdq",
	.id = 0,
	.dev = {
		.platform_data = NULL,
	},
	.num_resources	= ARRAY_SIZE(omap_hdq_resources),
	.resource	= omap_hdq_resources,
};
static inline void omap_hdq_init(void)
{
	(void) platform_device_register(&omap_hdq_dev);
}
#else
static inline void omap_hdq_init(void) {}
#endif

/*---------------------------------------------------------------------------*/

#if defined(CONFIG_VIDEO_OMAP2_VOUT) || \
	defined(CONFIG_VIDEO_OMAP2_VOUT_MODULE)
#if defined(CONFIG_FB_OMAP2) || defined(CONFIG_FB_OMAP2_MODULE)
static struct resource omap_vout_resource[3 - CONFIG_FB_OMAP2_NUM_FBS] = {
};
#else
static struct resource omap_vout_resource[2] = {
};
#endif

static struct platform_device omap_vout_device = {
	.name		= "omap_vout",
	.num_resources	= ARRAY_SIZE(omap_vout_resource),
	.resource 	= &omap_vout_resource[0],
	.id		= -1,
};
static void omap_init_vout(void)
{
	if (platform_device_register(&omap_vout_device) < 0)
		printk(KERN_ERR "Unable to register OMAP-VOUT device\n");
}
#else
static inline void omap_init_vout(void) {}
#endif

#if defined(CONFIG_SOC_OMAPAM33XX) && defined(CONFIG_OMAP3_EDMA)

#define AM33XX_SCM_BASE_EDMA		0x00000f90

static const s16 am33xx_dma_rsv_chans[][2] = {
	/* (offset, number) */
	{0, 2},
	{14, 2},
	{26, 6},
	{48, 4},
	{56, 8},
	{-1, -1}
};

static const s16 am33xx_dma_rsv_slots[][2] = {
	/* (offset, number) */
	{0, 2},
	{14, 2},
	{26, 6},
	{48, 4},
	{56, 8},
	{64, 127},
	{-1, -1}
};

/* Three Transfer Controllers on AM33XX */
static const s8 am33xx_queue_tc_mapping[][2] = {
	/* {event queue no, TC no} */
	{0, 0},
	{1, 1},
	{2, 2},
	{-1, -1}
};

static const s8 am33xx_queue_priority_mapping[][2] = {
	/* {event queue no, Priority} */
	{0, 0},
	{1, 1},
	{2, 2},
	{-1, -1}
};

static struct event_to_channel_map am33xx_xbar_event_mapping[] = {
	/* {xbar event no, Channel} */
	{1, 12},	/* SDTXEVT1 -> MMCHS2 */
	{2, 13},	/* SDRXEVT1 -> MMCHS2 */
	{3, -1},
	{4, -1},
	{5, -1},
	{6, -1},
	{7, -1},
	{8, -1},
	{9, -1},
	{10, -1},
	{11, -1},
	{12, -1},
	{13, -1},
	{14, -1},
	{15, -1},
	{16, -1},
	{17, -1},
	{18, -1},
	{19, -1},
	{20, -1},
	{21, -1},
	{22, -1},
	{23, -1},
	{24, -1},
	{25, -1},
	{26, -1},
	{27, -1},
	{28, -1},
	{29, -1},
	{30, -1},
	{31, -1},
	{-1, -1}
};

/**
 * map_xbar_event_to_channel - maps a crossbar event to a DMA channel
 * according to the configuration provided
 * @event: the event number for which mapping is required
 * @channel: channel being activated
 * @xbar_event_mapping: array that has the event to channel map
 *
 * Events that are routed by default are not mapped. Only events that
 * are crossbar mapped are routed to available channels according to
 * the configuration provided
 *
 * Returns zero on success, else negative errno.
 */
int map_xbar_event_to_channel(unsigned int event, unsigned int *channel,
			struct event_to_channel_map *xbar_event_mapping)
{
	unsigned int ctrl = 0;
	unsigned int xbar_evt_no = 0;
	unsigned int val = 0;
	unsigned int offset = 0;
	unsigned int mask = 0;

	ctrl = EDMA_CTLR(event);
	xbar_evt_no = event - (edma_cc[ctrl]->num_channels);

	if (event < edma_cc[ctrl]->num_channels) {
		*channel = event;
	} else if (event < edma_cc[ctrl]->num_events) {
		*channel = xbar_event_mapping[xbar_evt_no].channel_no;
		/* confirm the range */
		if (*channel < EDMA_MAX_DMACH)
			clear_bit(*channel, edma_cc[ctrl]->edma_unused);
		mask = (*channel)%4;
		offset = (*channel)/4;
		offset *= 4;
		offset += mask;
		val = (unsigned int)__raw_readl(AM33XX_CTRL_REGADDR(
					AM33XX_SCM_BASE_EDMA + offset));
		val = val & (~(0xFF));
		val = val | (xbar_event_mapping[xbar_evt_no].xbar_event_no);
		__raw_writel(val,
			AM33XX_CTRL_REGADDR(AM33XX_SCM_BASE_EDMA + offset));
		return 0;
	} else {
		return -EINVAL;
	}

	return 0;
}

static struct edma_soc_info am33xx_edma_info[] = {
	{
		.n_channel		= 64,
		.n_region		= 4,
		.n_slot			= 256,
		.n_tc			= 3,
		.n_cc			= 1,
		.rsv_chans		= am33xx_dma_rsv_chans,
		.rsv_slots		= am33xx_dma_rsv_slots,
		.queue_tc_mapping	= am33xx_queue_tc_mapping,
		.queue_priority_mapping	= am33xx_queue_priority_mapping,
		.is_xbar		= 1,
		.n_events		= 95,
		.xbar_event_mapping	= am33xx_xbar_event_mapping,
		.map_xbar_channel	= map_xbar_event_to_channel,
	},
};

static int __init am33xx_register_edma(void)
{
	int i, l;
	struct omap_hwmod *oh[4];
	struct platform_device *pdev;
	struct edma_soc_info *pdata = am33xx_edma_info;
	char oh_name[8];

	if (!cpu_is_am33xx())
		return -ENODEV;

	oh[0] = omap_hwmod_lookup("tpcc");
	if (!oh[0]) {
		pr_err("could not look up %s\n", "tpcc");
		return -ENODEV;
	}

	for (i = 0; i < 3; i++) {
		l = snprintf(oh_name, 8, "tptc%d", i);

		oh[i+1] = omap_hwmod_lookup(oh_name);
		if (!oh[i+1]) {
			pr_err("could not look up %s\n", oh_name);
			return -ENODEV;
		}
	}

	pdev = omap_device_build_ss("edma", 0, oh, 4, pdata, sizeof(*pdata),
								NULL, 0, 0);

	WARN(IS_ERR(pdev), "could not build omap_device for edma\n");

	return IS_ERR(pdev) ? PTR_ERR(pdev) : 0;

}

#else
static inline void am33xx_register_edma(void) {}
#endif

#if defined (CONFIG_SOC_OMAPAM33XX)
struct uio_pruss_pdata am335x_pruss_uio_pdata = {
	.pintc_base	= 0x20000,
};

static struct resource am335x_pruss_resources[] = {
	{
		.start	= AM33XX_ICSS_BASE,
		.end	= AM33XX_ICSS_BASE + AM33XX_ICSS_LEN,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= AM33XX_IRQ_ICSS0_0,
		.end	= AM33XX_IRQ_ICSS0_0,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= AM33XX_IRQ_ICSS0_1,
		.end	= AM33XX_IRQ_ICSS0_1,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= AM33XX_IRQ_ICSS0_2,
		.end	= AM33XX_IRQ_ICSS0_2,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= AM33XX_IRQ_ICSS0_3,
		.end	= AM33XX_IRQ_ICSS0_3,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= AM33XX_IRQ_ICSS0_4,
		.end	= AM33XX_IRQ_ICSS0_4,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= AM33XX_IRQ_ICSS0_5,
		.end	= AM33XX_IRQ_ICSS0_5,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= AM33XX_IRQ_ICSS0_6,
		.end	= AM33XX_IRQ_ICSS0_6,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= AM33XX_IRQ_ICSS0_7,
		.end	= AM33XX_IRQ_ICSS0_7,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device am335x_pruss_uio_dev = {
	.name		= "pruss_uio",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(am335x_pruss_resources),
	.resource	= am335x_pruss_resources,
	.dev	 =	{
		.coherent_dma_mask = 0xffffffff,
	}
};

int __init am335x_register_pruss_uio(struct uio_pruss_pdata *config)
{
	am335x_pruss_uio_dev.dev.platform_data = config;
	return platform_device_register(&am335x_pruss_uio_dev);
}

static struct platform_device am335x_sgx = {
	.name	= "sgx",
	.id	= -1,
};

#endif

#ifdef CONFIG_AM33XX_SMARTREFLEX

/* smartreflex platform data */

/* The values below are based upon silicon characterization data.
 * Each OPP and sensor combination potentially has different values.
 * The values of ERR2VOLT_GAIN and ERR_MIN_LIMIT also change based on
 * the PMIC step size.	Values have been given to cover the AM335 EVM
 * (12.5mV step) and the Beaglebone (25mV step).  If the step
 * size changes, you should update these values, and don't forget to
 * change the step size in the platform data structure, am33xx_sr_pdata.
 */

#define AM33XX_SR0_OPP50_CNTRL_OFFSET		0x07B8
#define AM33XX_SR0_OPP50_EVM_ERR_MIN_LIMIT	0xF0
#define AM33XX_SR0_OPP50_BB_ERR_MIN_LIMIT	0xEA
#define AM33XX_SR0_OPP50_ERR_MAX_LIMIT		0x2
#define AM33XX_SR0_OPP50_ERR_WEIGHT		0x4
#define AM33XX_SR0_OPP50_MARGIN			0

#define AM33XX_SR0_OPP100_CNTRL_OFFSET		0x07BC
#define AM33XX_SR0_OPP100_EVM_ERR_MIN_LIMIT	0xF0
#define AM33XX_SR0_OPP100_BB_ERR_MIN_LIMIT	0xF1
#define AM33XX_SR0_OPP100_ERR_MAX_LIMIT		0x2
#define AM33XX_SR0_OPP100_ERR_WEIGHT		0x4
#define AM33XX_SR0_OPP100_MARGIN		0

#define AM33XX_SR1_OPP50_CNTRL_OFFSET		0x0770
#define AM33XX_SR1_OPP50_EVM_ERR_MIN_LIMIT	0xFA
#define AM33XX_SR1_OPP50_BB_ERR_MIN_LIMIT	0xC0
#define AM33XX_SR1_OPP50_ERR_MAX_LIMIT		0x2
#define AM33XX_SR1_OPP50_ERR_WEIGHT		0x4
#define AM33XX_SR1_OPP50_MARGIN			0

#define AM33XX_SR1_OPP100_CNTRL_OFFSET		0x0774
#define AM33XX_SR1_OPP100_EVM_ERR_MIN_LIMIT	0xFB
#define AM33XX_SR1_OPP100_BB_ERR_MIN_LIMIT	0xDF
#define AM33XX_SR1_OPP100_ERR_MAX_LIMIT		0x2
#define AM33XX_SR1_OPP100_ERR_WEIGHT		0x4
#define AM33XX_SR1_OPP100_MARGIN		0

#define AM33XX_SR1_OPP120_CNTRL_OFFSET		0x0778
#define AM33XX_SR1_OPP120_EVM_ERR_MIN_LIMIT	0xFC
#define AM33XX_SR1_OPP120_BB_ERR_MIN_LIMIT	0xE6
#define AM33XX_SR1_OPP120_ERR_MAX_LIMIT		0x2
#define AM33XX_SR1_OPP120_ERR_WEIGHT		0x7
#define AM33XX_SR1_OPP120_MARGIN		0

#define AM33XX_SR1_OPPTURBO_CNTRL_OFFSET	0x077C
#define AM33XX_SR1_OPPTURBO_EVM_ERR_MIN_LIMIT	0xFD
#define AM33XX_SR1_OPPTURBO_BB_ERR_MIN_LIMIT	0xEA
#define AM33XX_SR1_OPPTURBO_ERR_MAX_LIMIT	0x2
#define AM33XX_SR1_OPPTURBO_ERR_WEIGHT		0x7
#define AM33XX_SR1_OPPTURBO_MARGIN		0

/* bits 31:16 = SenP margin; bit 15:0 = SenN margin */

#define AM33XX_SR1_OPPNITRO_MARGIN		0x018B019A

/* the voltages and frequencies should probably be defined in opp3xxx_data.c.
   Once SR is integrated to the mainline driver, and voltdm is working
   correctly in AM335x, these can be removed.  */
#define AM33XX_VDD_MPU_OPP50_UV			950000
#define AM33XX_VDD_MPU_OPP100_UV		1100000
#define AM33XX_VDD_MPU_OPP120_UV		1200000
#define AM33XX_VDD_MPU_OPPTURBO_UV		1260000
#define AM33XX_VDD_CORE_OPP50_UV		950000
#define AM33XX_VDD_CORE_OPP100_UV		1100000

#define AM33XX_VDD_MPU_OPP50_FREQ		275000000
#define AM33XX_VDD_MPU_OPP100_FREQ		500000000
#define AM33XX_VDD_MPU_OPP120_FREQ		600000000
#define AM33XX_VDD_MPU_OPPTURBO_FREQ		720000000

#define AM33XX_ES2_0_VDD_MPU_OPP50_UV		950000
#define AM33XX_ES2_0_VDD_MPU_OPP100_UV		1100000
#define AM33XX_ES2_0_VDD_MPU_OPP120_UV		1200000
#define AM33XX_ES2_0_VDD_MPU_OPPTURBO_UV	1260000
#define AM33XX_ES2_0_VDD_MPU_OPPNITRO_UV	1320000

#define AM33XX_ES2_0_VDD_MPU_OPP50_FREQ		300000000
#define AM33XX_ES2_0_VDD_MPU_OPP100_FREQ	600000000
#define AM33XX_ES2_0_VDD_MPU_OPP120_FREQ	720000000
#define AM33XX_ES2_0_VDD_MPU_OPPTURBO_FREQ	800000000
#define AM33XX_ES2_0_VDD_MPU_OPPNITRO_FREQ	1000000000

static struct am33xx_sr_opp_data sr1_opp_data_2_0[] = {
	{
		.efuse_offs	= AM33XX_SR1_OPP50_CNTRL_OFFSET,
		.e2v_gain	= 0,
		.err_minlimit	= AM33XX_SR1_OPP50_EVM_ERR_MIN_LIMIT,
		.err_maxlimit	= AM33XX_SR1_OPP50_ERR_MAX_LIMIT,
		.err_weight	= AM33XX_SR1_OPP50_ERR_WEIGHT,
		.margin		= AM33XX_SR1_OPP50_MARGIN,
		.nominal_volt	= AM33XX_ES2_0_VDD_MPU_OPP50_UV,
		.frequency	= AM33XX_ES2_0_VDD_MPU_OPP50_FREQ,
	},
	{
		.efuse_offs	= AM33XX_SR1_OPP100_CNTRL_OFFSET,
		.e2v_gain	= 0,
		.err_minlimit	= AM33XX_SR1_OPP100_EVM_ERR_MIN_LIMIT,
		.err_maxlimit	= AM33XX_SR1_OPP100_ERR_MAX_LIMIT,
		.err_weight	= AM33XX_SR1_OPP100_ERR_WEIGHT,
		.margin		= AM33XX_SR1_OPP100_MARGIN,
		.nominal_volt	= AM33XX_ES2_0_VDD_MPU_OPP100_UV,
		.frequency	= AM33XX_ES2_0_VDD_MPU_OPP100_FREQ,
	},
	{
		.efuse_offs	= AM33XX_SR1_OPP120_CNTRL_OFFSET,
		.e2v_gain	= 0,
		.err_minlimit	= AM33XX_SR1_OPP120_EVM_ERR_MIN_LIMIT,
		.err_maxlimit	= AM33XX_SR1_OPP120_ERR_MAX_LIMIT,
		.err_weight	= AM33XX_SR1_OPP120_ERR_WEIGHT,
		.margin		= AM33XX_SR1_OPP120_MARGIN,
		.nominal_volt	= AM33XX_ES2_0_VDD_MPU_OPP120_UV,
		.frequency	= AM33XX_ES2_0_VDD_MPU_OPP120_FREQ,
	},
	{
		.efuse_offs     = AM33XX_SR1_OPPTURBO_CNTRL_OFFSET,
		.e2v_gain       = 0,
		.err_minlimit   = AM33XX_SR1_OPPTURBO_EVM_ERR_MIN_LIMIT,
		.err_maxlimit   = AM33XX_SR1_OPPTURBO_ERR_MAX_LIMIT,
		.err_weight     = AM33XX_SR1_OPPTURBO_ERR_WEIGHT,
		.margin		= AM33XX_SR1_OPPTURBO_MARGIN,
		.nominal_volt	= AM33XX_ES2_0_VDD_MPU_OPPTURBO_UV,
		.frequency	= AM33XX_ES2_0_VDD_MPU_OPPTURBO_FREQ,
	},
	{
		/* NITRO can use the TURBO data, except for margin */
		.efuse_offs     = AM33XX_SR1_OPPTURBO_CNTRL_OFFSET,
		.e2v_gain       = 0,
		.err_minlimit   = AM33XX_SR1_OPPTURBO_EVM_ERR_MIN_LIMIT,
		.err_maxlimit   = AM33XX_SR1_OPPTURBO_ERR_MAX_LIMIT,
		.err_weight     = AM33XX_SR1_OPPTURBO_ERR_WEIGHT,
		.margin		= AM33XX_SR1_OPPNITRO_MARGIN,
		.nominal_volt	= AM33XX_ES2_0_VDD_MPU_OPPNITRO_UV,
		.frequency	= AM33XX_ES2_0_VDD_MPU_OPPNITRO_FREQ,
	},
};

static struct am33xx_sr_opp_data sr1_opp_data[] = {
	{
		.efuse_offs     = AM33XX_SR1_OPP50_CNTRL_OFFSET,
		.e2v_gain       = 0,
		.err_minlimit   = AM33XX_SR1_OPP50_EVM_ERR_MIN_LIMIT,
		.err_maxlimit   = AM33XX_SR1_OPP50_ERR_MAX_LIMIT,
		.err_weight     = AM33XX_SR1_OPP50_ERR_WEIGHT,
		.margin		= AM33XX_SR1_OPP50_MARGIN,
		.nominal_volt	= AM33XX_VDD_MPU_OPP50_UV,
		.frequency	= AM33XX_VDD_MPU_OPP50_FREQ,
	},
	{
		.efuse_offs     = AM33XX_SR1_OPP100_CNTRL_OFFSET,
		.e2v_gain       = 0,
		.err_minlimit   = AM33XX_SR1_OPP100_EVM_ERR_MIN_LIMIT,
		.err_maxlimit   = AM33XX_SR1_OPP100_ERR_MAX_LIMIT,
		.err_weight     = AM33XX_SR1_OPP100_ERR_WEIGHT,
		.margin		= AM33XX_SR1_OPP100_MARGIN,
		.nominal_volt	= AM33XX_VDD_MPU_OPP100_UV,
		.frequency	= AM33XX_VDD_MPU_OPP100_FREQ,
	},
	{
		.efuse_offs	= AM33XX_SR1_OPP120_CNTRL_OFFSET,
		.e2v_gain       = 0,
		.err_minlimit   = AM33XX_SR1_OPP120_EVM_ERR_MIN_LIMIT,
		.err_maxlimit   = AM33XX_SR1_OPP120_ERR_MAX_LIMIT,
		.err_weight     = AM33XX_SR1_OPP120_ERR_WEIGHT,
		.margin		= AM33XX_SR1_OPP120_MARGIN,
		.nominal_volt	= AM33XX_VDD_MPU_OPP120_UV,
		.frequency	= AM33XX_VDD_MPU_OPP120_FREQ,
	},
	{
		.efuse_offs	= AM33XX_SR1_OPPTURBO_CNTRL_OFFSET,
		.e2v_gain       = 0,
		.err_minlimit   = AM33XX_SR1_OPPTURBO_EVM_ERR_MIN_LIMIT,
		.err_maxlimit   = AM33XX_SR1_OPPTURBO_ERR_MAX_LIMIT,
		.err_weight     = AM33XX_SR1_OPPTURBO_ERR_WEIGHT,
		.margin		= AM33XX_SR1_OPPTURBO_MARGIN,
		.nominal_volt	= AM33XX_VDD_MPU_OPPTURBO_UV,
		.frequency	= AM33XX_VDD_MPU_OPPTURBO_FREQ,
	},
};

static struct am33xx_sr_opp_data sr0_opp_data[] = {
	{
		.efuse_offs     = AM33XX_SR0_OPP50_CNTRL_OFFSET,
		.e2v_gain       = 0,
		.err_minlimit   = AM33XX_SR0_OPP50_EVM_ERR_MIN_LIMIT,
		.err_maxlimit   = AM33XX_SR0_OPP50_ERR_MAX_LIMIT,
		.err_weight     = AM33XX_SR0_OPP50_ERR_WEIGHT,
		.margin		= AM33XX_SR0_OPP50_MARGIN,
		.nominal_volt	= AM33XX_VDD_CORE_OPP50_UV,
	},
	{
		.efuse_offs     = AM33XX_SR0_OPP100_CNTRL_OFFSET,
		.e2v_gain       = 0,
		.err_minlimit   = AM33XX_SR0_OPP100_EVM_ERR_MIN_LIMIT,
		.err_maxlimit   = AM33XX_SR0_OPP100_ERR_MAX_LIMIT,
		.err_weight     = AM33XX_SR0_OPP100_ERR_WEIGHT,
		.margin		= AM33XX_SR0_OPP100_MARGIN,
		.nominal_volt	= AM33XX_VDD_CORE_OPP100_UV,
	},
};

static struct am33xx_sr_sdata sr_sensor_data_2_0[] = {
	{
		.sr_opp_data	= sr0_opp_data,
		/* note that OPP50 is NOT used in Linux kernel for AM335x */
		.no_of_opps	= 0x2,
		.default_opp	= 0x1,
		.senn_mod       = 0x1,
		.senp_mod       = 0x1,
	},
	{
		.sr_opp_data    = sr1_opp_data_2_0,
		/* the opp data below should be determined
		   dynamically during SR probe */
		.no_of_opps	= 0x5,
		.default_opp	= 0x3,
		.senn_mod       = 0x1,
		.senp_mod       = 0x1,
	},
};

static struct am33xx_sr_sdata sr_sensor_data[] = {
	{
		.sr_opp_data	= sr0_opp_data,
		/* note that OPP50 is NOT used in Linux kernel for AM335x */
		.no_of_opps	= 0x2,
		.default_opp	= 0x1,
		.senn_mod       = 0x1,
		.senp_mod       = 0x1,
	},
	{
		.sr_opp_data    = sr1_opp_data,
		/* the opp data below should be determined
		   dynamically during SR probe */
		.no_of_opps	= 0x4,
		.default_opp	= 0x3,
		.senn_mod       = 0x1,
		.senp_mod       = 0x1,
	},
};

static struct am33xx_sr_platform_data am33xx_sr_pdata = {
	.vd_name[0]		= "vdd_core",
	.vd_name[1]		= "vdd_mpu",
	.ip_type		= 2,
	.irq_delay		= 1000,
	.no_of_vds		= 2,
	.no_of_sens		= ARRAY_SIZE(sr_sensor_data),
	.vstep_size_uv		= 12500,
	.enable_on_init		= true,
	.sr_sdata		= sr_sensor_data,
};

static struct resource am33xx_sr_resources[] = {
	{
		.name   =       "smartreflex0",
		.start  =       AM33XX_SR0_BASE,
		.end    =       AM33XX_SR0_BASE + SZ_4K - 1,
		.flags  =       IORESOURCE_MEM,
	},
	{
		.name   =       "smartreflex0",
		.start  =       AM33XX_IRQ_SMARTREFLEX0,
		.end    =       AM33XX_IRQ_SMARTREFLEX0,
		.flags  =       IORESOURCE_IRQ,
	},
	{
		.name   =       "smartreflex1",
		.start  =       AM33XX_SR1_BASE,
		.end    =       AM33XX_SR1_BASE + SZ_4K - 1,
		.flags  =       IORESOURCE_MEM,
	},
	{
		.name   =       "smartreflex1",
		.start  =       AM33XX_IRQ_SMARTREFLEX1,
		.end    =       AM33XX_IRQ_SMARTREFLEX1,
		.flags  =       IORESOURCE_IRQ,
	},
};

/* VCORE for SR regulator init */
static struct platform_device am33xx_sr_device = {
	.name	       = "smartreflex",
	.id	       = -1,
	.num_resources  = ARRAY_SIZE(am33xx_sr_resources),
	.resource       = am33xx_sr_resources,
	.dev = {
	       .platform_data = &am33xx_sr_pdata,
	},
};

/* Refix default opp according to EFUSE_SMA register. Added by MYIR */
#define AM335X_EFUSE_SMA	(AM33XX_CTRL_BASE + 0x7FC)
#define MPU_MAXFREQ_MASK	0x1FFF
    #define MPU_MAX_FREQ_300M   0x1FEF
    #define MPU_MAX_FREQ_600M   0x1FAF
    #define MPU_MAX_FREQ_720M   0x1F2F
    #define MPU_MAX_FREQ_800M   0x1E2F
    #define MPU_MAX_FREQ_1000M  0x1C2F
    #define MPU_MAX_FREQ_300M_ZCE   0x1FDF
    #define MPU_MAX_FREQ_600M_ZCE   0x1F9F
void refix_default_opp_2_0(struct am33xx_sr_sdata * pdata)
{
	u32 reg;
	void __iomem *efuse_sma;
	
	efuse_sma = ioremap(AM335X_EFUSE_SMA, 0x4);
	if (!efuse_sma) {
		printk(KERN_ERR "[MYIR_ERR] ioremap for AM335X_EFUSE_SMA failed!\n");
		return;
	}
	
	reg = readl(efuse_sma);

	iounmap(efuse_sma);

	printk(KERN_ERR "[MYIR_DBG] AM335X_EFUSE_SMA: %#X, mpu freq code: %#X\n",
		reg, 
		reg&MPU_MAXFREQ_MASK);
	
    switch (reg&MPU_MAXFREQ_MASK) {
        case MPU_MAX_FREQ_300M:
        case MPU_MAX_FREQ_300M_ZCE:
            pdata->default_opp = 0;
			break;
        case MPU_MAX_FREQ_600M:
        case MPU_MAX_FREQ_600M_ZCE:
            pdata->default_opp = 1;
			break;
        case MPU_MAX_FREQ_720M:
            pdata->default_opp = 2;
			break;
        case MPU_MAX_FREQ_800M:
            pdata->default_opp = 3;
			break;
        case MPU_MAX_FREQ_1000M:
            pdata->default_opp = 4;
			break;
        default:
            pdata->default_opp = 3;
			break;
    }
	printk(KERN_ERR "[MYIR_DBG] refix default opp to %d (mpu freq: %d MHz)\n",
		pdata->default_opp,
		pdata->sr_opp_data[pdata->default_opp].frequency/1000000);
}

void __init am33xx_sr_init(void)
{
	if (omap_rev() != AM335X_REV_ES1_0) {
		refix_default_opp_2_0(&sr_sensor_data_2_0[1]);
		am33xx_sr_pdata.sr_sdata = sr_sensor_data_2_0;
		printk(KERN_ERR "[MYIR_DBG] use the new sr_sensor_data!!!\n");
	}

	/* For beaglebone, update voltage step size and related parameters
	   appropriately.  All other AM33XX platforms are good with the
	   structure defaults as initialized above. */

/* Modified by MYIR, we use the same PMIC as beaglebone
   if ((am33xx_evmid == BEAGLE_BONE_OLD) ||
			(am33xx_evmid == BEAGLE_BONE_A3)) { */
		printk(KERN_ERR "address of pdata = %08x\n",
			(u32)&am33xx_sr_pdata);

		am33xx_sr_pdata.vstep_size_uv = 25000;
/*	}
MYIR */

	if (platform_device_register(&am33xx_sr_device))
		printk(KERN_ERR "failed to register am33xx_sr device\n");
	else
		printk(KERN_INFO "registered am33xx_sr device\n");
}
#else
inline void am33xx_sr_init(void) {}
#endif

/*-------------------------------------------------------------------------*/

static int __init omap2_init_devices(void)
{
	/*
	 * please keep these calls, and their implementations above,
	 * in alphabetical order so they're easier to sort through.
	 */
	omap_init_audio();
	omap_init_mcpdm();
	omap_init_dmic();
	omap_init_camera();
	omap_init_mbox();
	omap_init_mcspi();
	omap_init_pmu();
	omap_hdq_init();
	omap_init_sti();
	omap_init_sham();
	omap_init_aes();
	omap_init_vout();
	am33xx_register_edma();
	am33xx_init_pcm();
#if defined (CONFIG_SOC_OMAPAM33XX)
	am335x_register_pruss_uio(&am335x_pruss_uio_pdata);
	if (omap3_has_sgx())
		platform_device_register(&am335x_sgx);
#endif
	return 0;
}
arch_initcall(omap2_init_devices);

#define AM33XX_EMAC_MDIO_FREQ		(1000000)
/* Port Vlan IDs for Dual Mac Mode */
#define CPSW_PORT_VLAN_SLAVE_0		2
#define CPSW_PORT_VLAN_SLAVE_1		3

/* TODO : Verify the offsets */
static struct cpsw_slave_data am33xx_cpsw_slaves[] = {
	{
		.slave_reg_ofs  = 0x208,
		.sliver_reg_ofs = 0xd80,
		.phy_id		= "0:00",
		.dual_emac_reserved_vlan = CPSW_PORT_VLAN_SLAVE_0,
	},
	{
		.slave_reg_ofs  = 0x308,
		.sliver_reg_ofs = 0xdc0,
		.phy_id		= "0:01",
		.dual_emac_reserved_vlan = CPSW_PORT_VLAN_SLAVE_1,
	},
};

static struct cpsw_platform_data am33xx_cpsw_pdata = {
	.ss_reg_ofs		= 0x1200,
	.channels		= 8,
	.cpdma_reg_ofs		= 0x800,
	.slaves			= 2,
	.slave_data		= am33xx_cpsw_slaves,
	.ale_reg_ofs		= 0xd00,
	.ale_entries		= 1024,
	.host_port_reg_ofs      = 0x108,
	.hw_stats_reg_ofs       = 0x900,
	.bd_ram_ofs		= 0x2000,
	.bd_ram_size		= SZ_8K,
	.rx_descs               = 64,
	.mac_control            = BIT(5), /* MIIEN */
	.gigabit_en		= 1,
	.host_port_num		= 0,
	.no_bd_ram		= false,
	.version		= CPSW_VERSION_2,
};

static struct mdio_platform_data am33xx_cpsw_mdiopdata = {
	.bus_freq       = AM33XX_EMAC_MDIO_FREQ,
};

static unsigned char  am33xx_macid0[ETH_ALEN];
static unsigned char  am33xx_macid1[ETH_ALEN];

/*
* am33xx_cpsw_macidfillup - setup mac adrresses
* @eeprommacid0 - mac id 0 which needs to be configured
* @eeprommacid1 - mac id 1 which needs to be configured
*
* This function is called to configure mac addresses.
* Mac addresses are read from eeprom and this function is called
* to store those mac adresses in am33xx_macid0 and am33xx_macid1.
* In case, mac address read from eFuse are invalid, mac addresses
* stored in these variable are used.
*/
void am33xx_cpsw_macidfillup(char *eeprommacid0, char *eeprommacid1)
{
	u32 i;

	/* Fillup these mac addresses with the mac adresses from eeprom */
	for (i = 0; i < ETH_ALEN; i++) {
		am33xx_macid0[i] = eeprommacid0[i];
		am33xx_macid1[i] = eeprommacid1[i];
	}

	return;
}

int am33xx_cpsw_init(enum am33xx_cpsw_mac_mode mode, unsigned char *phy_id0,
		     unsigned char *phy_id1)
{
	struct omap_hwmod *oh;
	struct platform_device *pdev;
	u32 mac_lo, mac_hi, gmii_sel;
	u32 i;

	mac_lo = omap_ctrl_readl(TI81XX_CONTROL_MAC_ID0_LO);
	mac_hi = omap_ctrl_readl(TI81XX_CONTROL_MAC_ID0_HI);
	am33xx_cpsw_slaves[0].mac_addr[0] = mac_hi & 0xFF;
	am33xx_cpsw_slaves[0].mac_addr[1] = (mac_hi & 0xFF00) >> 8;
	am33xx_cpsw_slaves[0].mac_addr[2] = (mac_hi & 0xFF0000) >> 16;
	am33xx_cpsw_slaves[0].mac_addr[3] = (mac_hi & 0xFF000000) >> 24;
	am33xx_cpsw_slaves[0].mac_addr[4] = mac_lo & 0xFF;
	am33xx_cpsw_slaves[0].mac_addr[5] = (mac_lo & 0xFF00) >> 8;

	/* Read MACID0 from eeprom if eFuse MACID is invalid */
	if (!is_valid_ether_addr(am33xx_cpsw_slaves[0].mac_addr)) {
		for (i = 0; i < ETH_ALEN; i++)
			am33xx_cpsw_slaves[0].mac_addr[i] = am33xx_macid0[i];
	}

	mac_lo = omap_ctrl_readl(TI81XX_CONTROL_MAC_ID1_LO);
	mac_hi = omap_ctrl_readl(TI81XX_CONTROL_MAC_ID1_HI);
	am33xx_cpsw_slaves[1].mac_addr[0] = mac_hi & 0xFF;
	am33xx_cpsw_slaves[1].mac_addr[1] = (mac_hi & 0xFF00) >> 8;
	am33xx_cpsw_slaves[1].mac_addr[2] = (mac_hi & 0xFF0000) >> 16;
	am33xx_cpsw_slaves[1].mac_addr[3] = (mac_hi & 0xFF000000) >> 24;
	am33xx_cpsw_slaves[1].mac_addr[4] = mac_lo & 0xFF;
	am33xx_cpsw_slaves[1].mac_addr[5] = (mac_lo & 0xFF00) >> 8;

	/* Read MACID1 from eeprom if eFuse MACID is invalid */
	if (!is_valid_ether_addr(am33xx_cpsw_slaves[1].mac_addr)) {
		for (i = 0; i < ETH_ALEN; i++)
			am33xx_cpsw_slaves[1].mac_addr[i] = am33xx_macid1[i];
	}

	switch (mode) {
	case AM33XX_CPSW_MODE_MII:
		gmii_sel = AM33XX_MII_MODE_EN;
		break;
	case AM33XX_CPSW_MODE_RMII:
		gmii_sel = AM33XX_RMII_MODE_EN;
		break;
	case AM33XX_CPSW_MODE_RGMII:
		gmii_sel = AM33XX_RGMII_MODE_EN;
		break;
	default:
		return -EINVAL;
	}

	writel(gmii_sel, AM33XX_CTRL_REGADDR(AM33XX_CONTROL_GMII_SEL_OFFSET));

	if (phy_id0 != NULL)
		am33xx_cpsw_slaves[0].phy_id = phy_id0;

	if (phy_id1 != NULL)
		am33xx_cpsw_slaves[1].phy_id = phy_id1;

	memcpy(am33xx_cpsw_pdata.mac_addr,
			am33xx_cpsw_slaves[0].mac_addr, ETH_ALEN);

	oh = omap_hwmod_lookup("mdio");
	if (!oh) {
		pr_err("could not find cpgmac0 hwmod data\n");
		return -ENODEV;
	}

	pdev = omap_device_build("davinci_mdio", 0, oh, &am33xx_cpsw_mdiopdata,
			sizeof(am33xx_cpsw_mdiopdata), NULL, 0, 0);
	if (IS_ERR(pdev))
		pr_err("could not build omap_device for cpsw\n");

	oh = omap_hwmod_lookup("cpgmac0");
	if (!oh) {
		pr_err("could not find cpgmac0 hwmod data\n");
		return -ENODEV;
	}

	pdev = omap_device_build("cpsw", -1, oh, &am33xx_cpsw_pdata,
			sizeof(am33xx_cpsw_pdata), NULL, 0, 0);
	if (IS_ERR(pdev))
		pr_err("could not build omap_device for cpsw\n");

	return 0;
}

#define AM33XX_DCAN_NUM_MSG_OBJS		64
#define AM33XX_DCAN_RAMINIT_OFFSET		0x644
#define AM33XX_DCAN_RAMINIT_START(n)		(0x1 << n)

static void d_can_hw_raminit(unsigned int instance, unsigned int enable)
{
	u32 val;

	/* Read the value */
	val = readl(AM33XX_CTRL_REGADDR(AM33XX_DCAN_RAMINIT_OFFSET));
	if (enable) {
		/* Set to "1" */
		val &= ~AM33XX_DCAN_RAMINIT_START(instance);
		val |= AM33XX_DCAN_RAMINIT_START(instance);
		writel(val, AM33XX_CTRL_REGADDR(AM33XX_DCAN_RAMINIT_OFFSET));
	} else {
		/* Set to "0" */
		val &= ~AM33XX_DCAN_RAMINIT_START(instance);
		writel(val, AM33XX_CTRL_REGADDR(AM33XX_DCAN_RAMINIT_OFFSET));
	}
}

/* dcan dev_attr */
static struct d_can_platform_data am33xx_dcan_info = {
	.num_of_msg_objs	= AM33XX_DCAN_NUM_MSG_OBJS,
	.ram_init		= d_can_hw_raminit,
	.dma_support		= false,
};

void am33xx_d_can_init(unsigned int instance)
{
	struct omap_hwmod *oh;
	struct platform_device *pdev;
	char oh_name[L3_MODULES_MAX_LEN];

	/* Copy string name to oh_name buffer */
	snprintf(oh_name, L3_MODULES_MAX_LEN, "d_can%d", instance);

	oh = omap_hwmod_lookup(oh_name);
	if (!oh) {
		pr_err("could not find %s hwmod data\n", oh_name);
		return;
	}

	pdev = omap_device_build("d_can", instance, oh, &am33xx_dcan_info,
			sizeof(am33xx_dcan_info), NULL, 0, 0);
	if (IS_ERR(pdev))
		pr_err("could not build omap_device for %s\n", oh_name);
}

#if defined(CONFIG_OMAP_WATCHDOG) || defined(CONFIG_OMAP_WATCHDOG_MODULE)
static int __init omap_init_wdt(void)
{
	int id = -1;
	struct platform_device *pdev;
	struct omap_hwmod *oh;
	char *oh_name = "wd_timer2";
	char *dev_name = "omap_wdt";

	if (!cpu_class_is_omap2())
		return 0;

	oh = omap_hwmod_lookup(oh_name);
	if (!oh) {
		pr_err("Could not look up wd_timer%d hwmod\n", id);
		return -EINVAL;
	}

	pdev = omap_device_build(dev_name, id, oh, NULL, 0, NULL, 0, 0);
	WARN(IS_ERR(pdev), "Can't build omap_device for %s:%s.\n",
				dev_name, oh->name);
	return 0;
}
subsys_initcall(omap_init_wdt);
#endif

int __init omap_init_gpmc(struct gpmc_devices_info *pdata, int pdata_len)
{
	struct omap_hwmod *oh;
	struct platform_device *pdev;
	char *name = "omap-gpmc";
	char *oh_name = "gpmc";

	oh = omap_hwmod_lookup(oh_name);
	if (!oh) {
		pr_err("Could not look up %s\n", oh_name);
		return -ENODEV;
	}

	pdev = omap_device_build(name, -1, oh, pdata,
					pdata_len, NULL, 0, 0);
	if (IS_ERR(pdev)) {
		WARN(1, "Can't build omap_device for %s:%s.\n",
						name, oh->name);
		return PTR_ERR(pdev);
	}

	return 0;
}
