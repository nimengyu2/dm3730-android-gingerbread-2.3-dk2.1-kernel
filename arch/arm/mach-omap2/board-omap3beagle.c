/*
 * linux/arch/arm/mach-omap2/board-omap3beagle.c
 *
 * Copyright (C) 2008 Texas Instruments
 *
 * Modified from mach-omap2/board-3430sdp.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mmc/host.h>

#include <linux/usb/android_composite.h>

#include <linux/regulator/machine.h>
#include <linux/i2c/twl.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/display.h>
#include <plat/gpmc.h>
#include <plat/nand.h>
#include <plat/usb.h>

#include <linux/lierda_debug.h>
#include <linux/i2c/tsc2007.h> // Modify by nmy

#include "mux.h"
#include "hsmmc.h"
#include "timer-gp.h"
#include "board-flash.h"
#include <linux/interrupt.h>
#include <linux/dm9000.h>

#define NAND_BLOCK_SIZE		SZ_128K

#ifdef CONFIG_USB_ANDROID

#define GOOGLE_VENDOR_ID		0x18d1
#define GOOGLE_PRODUCT_ID		0x9018
#define GOOGLE_ADB_PRODUCT_ID		0x9015

static char *usb_functions_adb[] = {
	"adb",
};

static char *usb_functions_all[] = {
	"adb",
};

static struct android_usb_product usb_products[] = {
	{
		.product_id	= GOOGLE_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_adb),
		.functions	= usb_functions_adb,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= GOOGLE_VENDOR_ID,
	.product_id	= GOOGLE_PRODUCT_ID,
	.functions	= usb_functions_all,
	.products	= usb_products,
	.version	= 0x0100,
	.product_name	= "rowboat gadget",
	.manufacturer_name	= "rowboat",
	.serial_number	= "20100720",
	.num_functions	= ARRAY_SIZE(usb_functions_all),
};

static struct platform_device androidusb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data = &android_usb_pdata,
	},
};

static void omap3evm_android_gadget_init(void)
{
	platform_device_register(&androidusb_device);
}
#endif
/*
 * OMAP3 Beagle revision
 * Run time detection of Beagle revision is done by reading GPIO.
 * GPIO ID -
 *	AXBX	= GPIO173, GPIO172, GPIO171: 1 1 1
 *	C1_3	= GPIO173, GPIO172, GPIO171: 1 1 0
 *	C4	= GPIO173, GPIO172, GPIO171: 1 0 1
 *	XM	= GPIO173, GPIO172, GPIO171: 0 0 0
 */
enum {
	OMAP3BEAGLE_BOARD_UNKN = 0,
	OMAP3BEAGLE_BOARD_AXBX,
	OMAP3BEAGLE_BOARD_C1_3,
	OMAP3BEAGLE_BOARD_C4,
	OMAP3BEAGLE_BOARD_XM,
	OMAP3BEAGLE_BOARD_XMC,
};

static u8 omap3_beagle_version;

static u8 omap3_beagle_get_rev(void)
{
	return omap3_beagle_version;
}

static void __init omap3_beagle_init_rev(void)
{
	int ret;
	u16 beagle_rev = 0;

	omap_mux_init_gpio(171, OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_gpio(172, OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_gpio(173, OMAP_PIN_INPUT_PULLUP);

	ret = gpio_request(171, "rev_id_0");
	if (ret < 0)
		goto fail0;

	ret = gpio_request(172, "rev_id_1");
	if (ret < 0)
		goto fail1;

	ret = gpio_request(173, "rev_id_2");
	if (ret < 0)
		goto fail2;

	gpio_direction_input(171);
	gpio_direction_input(172);
	gpio_direction_input(173);

	beagle_rev = gpio_get_value(171) | (gpio_get_value(172) << 1)
			| (gpio_get_value(173) << 2);

	switch (beagle_rev) {
	case 7:
		printk(KERN_INFO "OMAP3 Beagle Rev: Ax/Bx\n");
		omap3_beagle_version = OMAP3BEAGLE_BOARD_AXBX;
		break;
	case 6:
		printk(KERN_INFO "OMAP3 Beagle Rev: C1/C2/C3\n");
		omap3_beagle_version = OMAP3BEAGLE_BOARD_C1_3;
		break;
	case 5:
		printk(KERN_INFO "OMAP3 Beagle Rev: C4\n");
		omap3_beagle_version = OMAP3BEAGLE_BOARD_C4;
		break;
	case 2:
		printk(KERN_INFO "OMAP3 Beagle Rev: xM C\n");
		omap3_beagle_version = OMAP3BEAGLE_BOARD_XMC;
		break;
	case 0:
		printk(KERN_INFO "OMAP3 Beagle Rev: xM\n");
		omap3_beagle_version = OMAP3BEAGLE_BOARD_XM;
		break;
	default:
		printk(KERN_INFO "OMAP3 Beagle Rev: unknown %hd\n", beagle_rev);
		omap3_beagle_version = OMAP3BEAGLE_BOARD_UNKN;
	}

	return;

fail2:
	gpio_free(172);
fail1:
	gpio_free(171);
fail0:
	printk(KERN_ERR "Unable to get revision detection GPIO pins\n");
	omap3_beagle_version = OMAP3BEAGLE_BOARD_UNKN;

	return;
}

static struct mtd_partition omap3beagle_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader",
		.offset		= 0,
		.size		= 4 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 15 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot Env",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x260000 */
		.size		= 1 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "Kernel",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
		.size		= 32 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "File System",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x680000 */
		.size		= MTDPART_SIZ_FULL,
	},
};

/* DSS */

static int beagle_enable_dvi(struct omap_dss_device *dssdev)
{
	if (gpio_is_valid(dssdev->reset_gpio))
		gpio_set_value(dssdev->reset_gpio, 1);

	return 0;
}

static void beagle_disable_dvi(struct omap_dss_device *dssdev)
{
	if (gpio_is_valid(dssdev->reset_gpio))
		gpio_set_value(dssdev->reset_gpio, 0);
}

static struct omap_dss_device beagle_dvi_device = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "dvi",
	.driver_name = "generic_panel",
	.phy.dpi.data_lines = 24,
	.reset_gpio = -EINVAL,
	.platform_enable = beagle_enable_dvi,
	.platform_disable = beagle_disable_dvi,
};

static struct omap_dss_device beagle_tv_device = {
	.name = "tv",
	.driver_name = "venc",
	.type = OMAP_DISPLAY_TYPE_VENC,
	.phy.venc.type = OMAP_DSS_VENC_TYPE_SVIDEO,
};

static struct omap_dss_device *beagle_dss_devices[] = {
	&beagle_dvi_device,
	&beagle_tv_device,
};

static struct omap_dss_board_info beagle_dss_data = {
	.num_devices = ARRAY_SIZE(beagle_dss_devices),
	.devices = beagle_dss_devices,
	.default_device = &beagle_dvi_device,
};

static struct platform_device beagle_dss_device = {
	.name          = "omapdss",
	.id            = -1,
	.dev            = {
		.platform_data = &beagle_dss_data,
	},
};

static struct regulator_consumer_supply beagle_vdac_supply =
	REGULATOR_SUPPLY("vdda_dac", "omapdss");

static struct regulator_consumer_supply beagle_vdvi_supply =
	REGULATOR_SUPPLY("vdds_dsi", "omapdss");

static void __init beagle_display_init(void)
{
	int r;

	r = gpio_request(beagle_dvi_device.reset_gpio, "DVI reset");
	if (r < 0) {
		printk(KERN_ERR "Unable to get DVI reset GPIO\n");
		return;
	}

	gpio_direction_output(beagle_dvi_device.reset_gpio, 0);
}

#include "sdram-micron-mt46h32m32lf-6.h"

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		//.gpio_wp	= 29,
		.gpio_wp	= NULL,
	},
	{}	/* Terminator */
};

static struct regulator_consumer_supply beagle_vmmc1_supply = {
	.supply			= "vmmc",
};

static struct regulator_consumer_supply beagle_vsim_supply = {
	.supply			= "vmmc_aux",
};

static struct regulator_consumer_supply beagle_vaux3_supply = {
	.supply         = "cam_1v8",
};

static struct regulator_consumer_supply beagle_vaux4_supply = {
	.supply         = "cam_2v8",
};

static struct gpio_led gpio_leds[];

static int beagle_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	if (omap3_beagle_get_rev() == OMAP3BEAGLE_BOARD_XM || omap3_beagle_get_rev() == OMAP3BEAGLE_BOARD_XMC) {
		mmc[0].gpio_wp = -EINVAL;
	} else if ((omap3_beagle_get_rev() == OMAP3BEAGLE_BOARD_C1_3) ||
		(omap3_beagle_get_rev() == OMAP3BEAGLE_BOARD_C4)) {
		omap_mux_init_gpio(23, OMAP_PIN_INPUT);
		//mmc[0].gpio_wp = 23;
		mmc[0].gpio_wp = -EINVAL;
	} else {
		omap_mux_init_gpio(29, OMAP_PIN_INPUT);
	}
	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
	mmc[0].gpio_cd = gpio + 0;
	omap2_hsmmc_init(mmc);

	/* link regulators to MMC adapters */
	beagle_vmmc1_supply.dev = mmc[0].dev;
	beagle_vsim_supply.dev = mmc[0].dev;

	/* REVISIT: need ehci-omap hooks for external VBUS
	 * power switch and overcurrent detect
	 */
	if (omap3_beagle_get_rev() != OMAP3BEAGLE_BOARD_XM || omap3_beagle_get_rev() == OMAP3BEAGLE_BOARD_XMC) {
		gpio_request(gpio + 1, "EHCI_nOC");
		gpio_direction_input(gpio + 1);
	}

	/*
	 * TWL4030_GPIO_MAX + 0 == ledA, EHCI nEN_USB_PWR (out, XM active
	 * high / others active low)
	 */
	gpio_request(gpio + TWL4030_GPIO_MAX, "nEN_USB_PWR");
	gpio_direction_output(gpio + TWL4030_GPIO_MAX, 0);
	if (omap3_beagle_get_rev() == OMAP3BEAGLE_BOARD_XM)
		gpio_direction_output(gpio + TWL4030_GPIO_MAX, 1);
	else
		gpio_direction_output(gpio + TWL4030_GPIO_MAX, 0);

	/* DVI reset GPIO is different between beagle revisions */
	if (omap3_beagle_get_rev() == OMAP3BEAGLE_BOARD_XM || omap3_beagle_get_rev() == OMAP3BEAGLE_BOARD_XMC)
		beagle_dvi_device.reset_gpio = 129;
	else
		beagle_dvi_device.reset_gpio = 170;

	if (omap3_beagle_get_rev() == OMAP3BEAGLE_BOARD_XM) {
		/* Power on camera interface */
		gpio_request(gpio + 2, "CAM_EN");
		gpio_direction_output(gpio + 2, 1);

		/* TWL4030_GPIO_MAX + 0 == ledA, EHCI nEN_USB_PWR (out, active low) */
		gpio_request(gpio + TWL4030_GPIO_MAX, "nEN_USB_PWR");
		gpio_direction_output(gpio + TWL4030_GPIO_MAX, 1);
	} else {
		gpio_request(gpio + 1, "EHCI_nOC");
		gpio_direction_input(gpio + 1);

		/* TWL4030_GPIO_MAX + 0 == ledA, EHCI nEN_USB_PWR (out, active low) */
		gpio_request(gpio + TWL4030_GPIO_MAX, "nEN_USB_PWR");
		gpio_direction_output(gpio + TWL4030_GPIO_MAX, 0);
	}
	/* TWL4030_GPIO_MAX + 1 == ledB, PMU_STAT (out, active low LED) */
	gpio_leds[2].gpio = gpio + TWL4030_GPIO_MAX + 1;

	/*
	 * gpio + 1 on Xm controls the TFP410's enable line (active low)
	 * gpio + 2 control varies depending on the board rev as follows:
	 * P7/P8 revisions(prototype): Camera EN
	 * A2+ revisions (production): LDO (supplies DVI, serial, led blocks)
	 */
	if (omap3_beagle_get_rev() == OMAP3BEAGLE_BOARD_XM || omap3_beagle_get_rev() == OMAP3BEAGLE_BOARD_XMC) {
		gpio_request(gpio + 1, "nDVI_PWR_EN");
		gpio_direction_output(gpio + 1, 0);
		gpio_request(gpio + 2, "DVI_LDO_EN");
		gpio_direction_output(gpio + 2, 1);
	}

	return 0;
}

static struct twl4030_gpio_platform_data beagle_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= true,
	.pullups	= BIT(1),
	.pulldowns	= BIT(2) | BIT(6) | BIT(7) | BIT(8) | BIT(13)
				| BIT(15) | BIT(16) | BIT(17),
	.setup		= beagle_twl_gpio_setup,
};

/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
static struct regulator_init_data beagle_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &beagle_vmmc1_supply,
};

/* VSIM for MMC1 pins DAT4..DAT7 (2 mA, plus card == max 50 mA) */
static struct regulator_init_data beagle_vsim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &beagle_vsim_supply,
};

/* VDAC for DSS driving S-Video (8 mA unloaded, max 65 mA) */
static struct regulator_init_data beagle_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &beagle_vdac_supply,
};

/* VPLL2 for digital video outputs */
static struct regulator_init_data beagle_vpll2 = {
	.constraints = {
		.name			= "VDVI",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &beagle_vdvi_supply,
};

/* VAUX3 for CAM_1V8 */
static struct regulator_init_data beagle_vaux3 = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.apply_uV               = true,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &beagle_vaux3_supply,
};

 /* VAUX4 for CAM_2V8 */
static struct regulator_init_data beagle_vaux4 = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.apply_uV               = true,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
			| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &beagle_vaux4_supply,
};

static struct twl4030_usb_data beagle_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static struct twl4030_codec_audio_data beagle_audio_data = {
	.audio_mclk = 26000000,
};

static struct twl4030_codec_data beagle_codec_data = {
	.audio_mclk = 26000000,
	.audio = &beagle_audio_data,
};

static struct twl4030_platform_data beagle_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.usb		= &beagle_usb_data,
	.gpio		= &beagle_gpio_data,
	.codec		= &beagle_codec_data,
	.vmmc1		= &beagle_vmmc1,
	.vsim		= &beagle_vsim,
	.vdac		= &beagle_vdac,
	.vpll2		= &beagle_vpll2,
	.vaux3		= &beagle_vaux3,
	.vaux4		= &beagle_vaux4,
};


//----------------------------------------------------------------------------//
//  nmy add tsc2007 code   start  2010-12-10  14:00
//----------------------------------------------------------------------------//

/*
 * TSC 2007 Support
 */
#define TSC2007_GPIO_IRQ_PIN_TMP	38
#define TSC2007_GPIO_IRQ_PIN	162

static int tsc2007_init_irq(void)
{
	int ret = 0;
        //pr_warning("%s: lierda_tcs2007_init_irq %d\n", __func__, ret);
#if 1
	omap_mux_init_gpio(TSC2007_GPIO_IRQ_PIN_TMP, OMAP_PIN_INPUT_PULLUP);

	ret = gpio_request(TSC2007_GPIO_IRQ_PIN_TMP, "tsc2007-irq-tmp");
	if (ret < 0) {
		printk("%s: failed to TSC2007 IRQ GPIO: %d\n", __func__, ret);
		return ret;
	}
	else
	{
		printk("%s: ok to TSC2007 IRQ GPIO: %d\n", __func__, ret);
	}

	gpio_direction_input(TSC2007_GPIO_IRQ_PIN_TMP);


	omap_mux_init_gpio(TSC2007_GPIO_IRQ_PIN, OMAP_PIN_INPUT_PULLUP);

	ret = gpio_request(TSC2007_GPIO_IRQ_PIN, "tsc2007-irq");
	if (ret < 0) {
		printk("%s: failed to TSC2007 IRQ GPIO: %d\n", __func__, ret);
		return ret;
	}
	else
	{
		printk("%s: ok to TSC2007 IRQ GPIO: %d\n", __func__, ret);
	}

	gpio_direction_input(TSC2007_GPIO_IRQ_PIN);

#endif
#if 0
	omap_mux_init_gpio(TSC2007_GPIO_IRQ_PIN, OMAP_PIN_OUTPUT);
	//omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);
	gpio_request(TSC2007_GPIO_IRQ_PIN, "tsc2007");
	gpio_direction_output(TSC2007_GPIO_IRQ_PIN,1);
	gpio_set_value(TSC2007_GPIO_IRQ_PIN,1);
#endif
	return ret;
}

static void tsc2007_exit_irq(void)
{
	gpio_free(TSC2007_GPIO_IRQ_PIN);
}

static int tsc2007_get_irq_level(void)
{
	//pr_warning("%s: lierda_tsc2007_get_irq_level %d\n", __func__, 0);
	//lsd_ts_dbg(LSD_DBG,"enter tsc2007_get_irq_level\n");
	return gpio_get_value(TSC2007_GPIO_IRQ_PIN) ? 0 : 1;
}

struct tsc2007_platform_data da850evm_tsc2007data = {
	.model = 2007,
	.x_plate_ohms = 180,
	.get_pendown_state = tsc2007_get_irq_level,
	.init_platform_hw = tsc2007_init_irq,
	.exit_platform_hw = tsc2007_exit_irq,
};

//----------------------------------------------------------------------------//
//  nmy add tsc2007 code   end  2010-12-10  14:00
//----------------------------------------------------------------------------//

static struct i2c_board_info __initdata beagle_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &beagle_twldata,
	},
};

static struct i2c_board_info __initdata beagle_i2c_eeprom[] = {
       {
               I2C_BOARD_INFO("eeprom", 0x50),
       },
};

static struct i2c_board_info __initdata beagle_i2c2[] = {
	{
	       I2C_BOARD_INFO("tsc2007", 0x48),
	       .platform_data = &da850evm_tsc2007data,
		.irq = OMAP_GPIO_IRQ(TSC2007_GPIO_IRQ_PIN),
       },
};
static int __init omap3_beagle_i2c_init(void)
{
	omap_register_i2c_bus(1, 2600, beagle_i2c_boardinfo,
			ARRAY_SIZE(beagle_i2c_boardinfo));

	/* Bus 2 is used for Camera/Sensor interface */
	//omap_register_i2c_bus(2, 400, NULL, 0);
	omap_register_i2c_bus(2, 400, beagle_i2c2, ARRAY_SIZE(beagle_i2c2));
	

	/* Bus 3 is attached to the DVI port where devices like the pico DLP
	 * projector don't work reliably with 400kHz */
	omap_register_i2c_bus(3, 100, beagle_i2c_eeprom, ARRAY_SIZE(beagle_i2c_eeprom));

	return 0;
}

static struct gpio_led gpio_leds[] = {
	{
		.name			= "beagleboard::usr0",
		.default_trigger	= "heartbeat",
		.gpio			= 150,
	},
	{
		.name			= "beagleboard::usr1",
		.default_trigger	= "mmc0",
		.gpio			= 149,
	},
	{
		.name			= "beagleboard::pmu_stat",
		.gpio			= -EINVAL,	/* gets replaced */
		.active_low		= true,
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

#if 0
static struct gpio_keys_button gpio_buttons[] = {
	{
		.code			= BTN_EXTRA,
		.gpio			= 7,
		.desc			= "user",
		.wakeup			= 1,
	},
};

static struct gpio_keys_platform_data gpio_key_info = {
	.buttons	= gpio_buttons,
	.nbuttons	= ARRAY_SIZE(gpio_buttons),
};

static struct platform_device keys_gpio = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_key_info,
	},
};
#endif

#if 1
#define GPIO_KEY10   		24
#define GPIO_KEY9    		43
#define GPIO_KEY8    		26
#define GPIO_KEY7    		27
static struct gpio_keys_button am335x_evm_volume_gpio_buttons[] = {
	{
		.code                   = KEY_MENU,
		.gpio                   = GPIO_KEY9,
		.active_low             = true,
		.desc                   = "menu",
		.type                   = EV_KEY,
		.wakeup                 = 1,
	},
	{
		.code                   = KEY_BACK,
		.gpio                   = GPIO_KEY10,
		.active_low             = true,
		.desc                   = "back",
		.type                   = EV_KEY,
		.wakeup                 = 1,
	},
	{
		.code                   = KEY_VOLUMEUP,
		.gpio                   = GPIO_KEY7,
		.active_low             = true,
		.desc                   = "volumeup",
		.type                   = EV_KEY,
		.wakeup                 = 1,
	},
	{
		.code                   = KEY_VOLUMEDOWN,
		.gpio                   = GPIO_KEY8,
		.active_low             = true,
		.desc                   = "volumedown",
		.type                   = EV_KEY,
		.wakeup                 = 1,
	},
};

static struct gpio_keys_platform_data am335x_evm_volume_gpio_key_info = {
	.buttons        = am335x_evm_volume_gpio_buttons,
	.nbuttons       = ARRAY_SIZE(am335x_evm_volume_gpio_buttons),
};

static struct platform_device keys_gpio = {
	.name   = "gpio-keys",
	.id     = -1,
	.dev    = {
		.platform_data  = &am335x_evm_volume_gpio_key_info,
	},
};



static void volume_keys_init(void)
{
	int err;
	omap_mux_init_gpio(24, OMAP_PIN_INPUT_PULLUP);   // KEY10
	omap_mux_init_gpio(43, OMAP_PIN_INPUT_PULLUP);   // KEY9
	omap_mux_init_gpio(26, OMAP_PIN_INPUT_PULLUP);   // KEY8
	omap_mux_init_gpio(27, OMAP_PIN_INPUT_PULLUP);   // KEY7
	lsd_dbg(LSD_DBG,"enter function=%s\n",__FUNCTION__);
}
#endif

static void __init omap3_beagle_init_irq(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(mt46h32m32lf6_sdrc_params,
				  mt46h32m32lf6_sdrc_params);
	omap_init_irq();
	gpmc_init();
#ifdef CONFIG_OMAP_32K_TIMER
	if (omap3_beagle_version == OMAP3BEAGLE_BOARD_AXBX)
		omap2_gp_clockevent_set_gptimer(12);
	else
		omap2_gp_clockevent_set_gptimer(1);
#endif
}

//--------------------------------------------------------------//


#define OMAP_DM9000_BASE        0x2c000000
#define OMAP_DM9000_GPIO_IRQ    25
static struct resource omap3sbc8100_plus_dm9000_resources[] = {
        [0] =   {
                .start  = OMAP_DM9000_BASE,
                .end    = (OMAP_DM9000_BASE + 0x4 - 1),
                .flags  = IORESOURCE_MEM,
        },
        [1] =   {
                .start  = (OMAP_DM9000_BASE + 0x10),
                .end    = (OMAP_DM9000_BASE + 0x10 + 0x4 - 1),
                .flags  = IORESOURCE_MEM,
        },
        [2] =   {
                .start  = OMAP_GPIO_IRQ(OMAP_DM9000_GPIO_IRQ),
                .end    = OMAP_GPIO_IRQ(OMAP_DM9000_GPIO_IRQ),
                .flags  = IORESOURCE_IRQ | IRQF_TRIGGER_LOW,
        },
};

static struct dm9000_plat_data omap_dm9000_platdata = {
        .flags = DM9000_PLATF_16BITONLY | DM9000_PLATF_NO_EEPROM,
	.dev_addr[0] =  0x5C,
	.dev_addr[1] =  0x26,
	.dev_addr[2] =  0x0A,
	.dev_addr[3] =  0x17,
	.dev_addr[4] =  0xD1,
	.dev_addr[5] =  0x88,
};

static struct platform_device omap3sbc8100_plus_dm9000_device = {
        .name           = "dm9000",
        .id             = -1,
        .num_resources  = ARRAY_SIZE(omap3sbc8100_plus_dm9000_resources),
        .resource       = omap3sbc8100_plus_dm9000_resources,
        .dev            = {
                .platform_data = &omap_dm9000_platdata,
        },
};


#define NET_GPMC_CONFIG1	0x00001000
#define NET_GPMC_CONFIG2	0x001e1e00
#define NET_GPMC_CONFIG3	0x00080300
#define NET_GPMC_CONFIG4	0x1c091c09
#define NET_GPMC_CONFIG5	0x04181f1f
#define NET_GPMC_CONFIG6	0x00000FCF
#define NET_GPMC_CONFIG7	0x00000f6c

#define GPMC_CS3 3

static const u32 gpmc_nor2[7] = {
	 NET_GPMC_CONFIG1,
	 NET_GPMC_CONFIG2,
	 NET_GPMC_CONFIG3 ,
	 NET_GPMC_CONFIG4,
	 NET_GPMC_CONFIG5,
	 NET_GPMC_CONFIG6, 
	 NET_GPMC_CONFIG7
};


static void __init omap3sbc8100_plus_init_dm9000(void)
{

	// init gpmc mux
	omap_mux_init_signal("gpmc_ncs3", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("gpmc_ncs6", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("gpmc_ncs7", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("gpmc_a4", OMAP_PIN_OUTPUT);

	omap_mux_init_gpio(OMAP_DM9000_GPIO_IRQ, OMAP_PIN_OUTPUT);
	//omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);
	gpio_request(OMAP_DM9000_GPIO_IRQ, "dm9000");
	gpio_direction_output(OMAP_DM9000_GPIO_IRQ,1);
	gpio_set_value(OMAP_DM9000_GPIO_IRQ,1);

	omap_mux_init_gpio(OMAP_DM9000_GPIO_IRQ, OMAP_PIN_INPUT_PULLUP);
#if 0
        if (gpio_request(OMAP_DM9000_GPIO_IRQ, "dm9000 irq") < 0) {
                printk(KERN_ERR "Failed to request GPIO%d for dm9000 IRQ\n",
                        OMAP_DM9000_GPIO_IRQ);
		lsd_eth_dbg(LSD_ERR,"gpio_request OMAP_DM9000_GPIO_IRQ error\n");
                return;
        }
	else
	{
		lsd_eth_dbg(LSD_OK,"gpio_request OMAP_DM9000_GPIO_IRQ ok\n");
	}
#endif

        gpio_direction_input(OMAP_DM9000_GPIO_IRQ);


	gpmc_cs_write_reg(GPMC_CS3, GPMC_CS_CONFIG1, gpmc_nor2[0]);

	gpmc_cs_write_reg(GPMC_CS3, GPMC_CS_CONFIG2, gpmc_nor2[1]);

	gpmc_cs_write_reg(GPMC_CS3, GPMC_CS_CONFIG3, gpmc_nor2[2]);

	gpmc_cs_write_reg(GPMC_CS3, GPMC_CS_CONFIG4, gpmc_nor2[3]);

	gpmc_cs_write_reg(GPMC_CS3, GPMC_CS_CONFIG5, gpmc_nor2[4]);

	gpmc_cs_write_reg(GPMC_CS3, GPMC_CS_CONFIG6, gpmc_nor2[5]);
	
	//val = gpmc_cs_read_reg(GPMC_CS, GPMC_CS_CONFIG7);
	//val |= (1 << 6);
	gpmc_cs_write_reg(GPMC_CS3, GPMC_CS_CONFIG7, gpmc_nor2[6]);
}


static struct platform_device *omap3_beagle_devices[] __initdata = {
	&leds_gpio,
	&keys_gpio,
	&beagle_dss_device,
	&omap3sbc8100_plus_dm9000_device,
};

static void __init omap3beagle_flash_init(void)
{
	u8 cs = 0;
	u8 nandcs = GPMC_CS_NUM + 1;

	/* find out the chip-select on which NAND exists */
	while (cs < GPMC_CS_NUM) {
		u32 ret = 0;
		ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

		if ((ret & 0xC00) == 0x800) {
			printk(KERN_INFO "Found NAND on CS%d\n", cs);
			if (nandcs > GPMC_CS_NUM)
				nandcs = cs;
		}
		cs++;
	}

	if (nandcs > GPMC_CS_NUM) {
		printk(KERN_INFO "NAND: Unable to find configuration "
				 "in GPMC\n ");
		return;
	}

	if (nandcs < GPMC_CS_NUM) {
		printk(KERN_INFO "Registering NAND on CS%d\n", nandcs);
		board_nand_init(omap3beagle_nand_partitions,
			ARRAY_SIZE(omap3beagle_nand_partitions),
			nandcs, NAND_BUSWIDTH_16);
	}
}

static const struct ehci_hcd_omap_platform_data ehci_pdata __initconst = {

	.port_mode[0] = EHCI_HCD_OMAP_MODE_PHY,
	// nmy add
	//.port_mode[1] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_UNKNOWN,	
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = 147,
	.reset_gpio_port[2]  = -EINVAL
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#endif

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
	.mode			= MUSB_OTG,
	.power			= 100,
};



static void __init omap3_beagle_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	omap3_beagle_init_rev();
	omap3_beagle_i2c_init();
	
	// nmy add
	omap3sbc8100_plus_init_dm9000();
	volume_keys_init();

	platform_add_devices(omap3_beagle_devices,
			ARRAY_SIZE(omap3_beagle_devices));
	omap_serial_init();

	omap_mux_init_gpio(170, OMAP_PIN_INPUT);
	gpio_request(170, "DVI_nPD");
	/* REVISIT leave DVI powered down until it's needed ... */
	gpio_direction_output(170, true);

	usb_musb_init(&musb_board_data);
	usb_ehci_init(&ehci_pdata);
	omap3beagle_flash_init();

	/* Ensure SDRC pins are mux'd for self-refresh */
	omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("sdrc_cke1", OMAP_PIN_OUTPUT);

	beagle_display_init();
#ifdef CONFIG_USB_ANDROID
	omap3evm_android_gadget_init();
#endif
}

MACHINE_START(OMAP3_BEAGLE, "OMAP3 Beagle Board")
	/* Maintainer: Syed Mohammed Khasim - http://beagleboard.org */
	.boot_params	= 0x80000100,
	.map_io		= omap3_map_io,
	.reserve	= omap_reserve,
	.init_irq	= omap3_beagle_init_irq,
	.init_machine	= omap3_beagle_init,
	.timer		= &omap_timer,
MACHINE_END
