/*
 * Hanover Displays HTC using TI DaVinci DM365
 *
 * Copyright (C) 2010 Hanover Displays Ltd
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/i2c/at24.h>
#include <linux/leds.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/spi/eeprom.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc-dai.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/mux.h>
#include <mach/hardware.h>
#include <mach/dm365.h>
#include <mach/psc.h>
#include <mach/common.h>
#include <mach/i2c.h>
#include <mach/serial.h>
#include <mach/mmc.h>
#include <mach/mux.h>
#include <mach/nand.h>
#include <mach/keyscan.h>
#include <mach/gpio.h>
#include <linux/videodev2.h>
#include <media/tvp514x.h>
#include <media/tvp7002.h>
#include <media/davinci/videohd.h>

#include "clock.h"
#include <mach/htc_cpld.h>
#include <mach/htc_fpga.h>

void enable_hd_clk(void)
{
}
EXPORT_SYMBOL(enable_hd_clk);

void enable_lcd(void)
{
}
EXPORT_SYMBOL(enable_lcd);

#define DM365_ASYNC_EMIF_CONTROL_BASE	0x01d10000
#define DM365_ASYNC_EMIF_DATA_CE0_BASE	0x02000000

#define DM365_HTC_PHY_MASK		(0x2)
#define DM365_HTC_MDIO_FREQUENCY	(2200000) /* PHY bus frequency */

static struct at24_platform_data eeprom_info = {
#if 0
	.byte_len       = (256*1024) / 8,
	.page_size      = 64,
	.flags          = AT24_FLAG_ADDR16,
#else
	.byte_len		= 256,
	.page_size      = 8,
	.flags          = 0,
#endif
	//.setup          = davinci_get_mac_addr,
	.setup		= NULL,					// TODO - get MAC address and display HTC serial!
	.context	= (void *)0x7f00,
};

#if 0
static struct snd_platform_data dm365_htc_snd_data;
#else
// Fix to change DMA channel used by audio to a different channel to that used for video decode
static struct snd_platform_data dm365_htc_snd_data = {
	.eventq_no = EVENTQ_3,
};
#endif

static struct i2c_board_info i2c_info[] = {
	{ I2C_BOARD_INFO("24c256", 0x50), .platform_data = &eeprom_info },	// baseboard EEPROM
	{ I2C_BOARD_INFO("tlv320aic23", 0x1b) },		// audio codec
	{ I2C_BOARD_INFO("ths7303", 0x2c) },			// video amp
	{ I2C_BOARD_INFO("pcf8591", 0x48) },			// baseboard ADC
	{ I2C_BOARD_INFO("pcf8563", 0x51) },			// RTC
};

#ifdef CONFIG_HTC_AUDIOBOARD
static struct i2c_board_info i2c_audioboard_info[] = {
	{ I2C_BOARD_INFO("radio-tea5764", 0x10) },		// FM radio
	{ I2C_BOARD_INFO("24c256", 0x53), .platform_data = &eeprom_info },	// audio EEPROM
};
#endif

#ifdef CONFIG_HTC_FPGA_IO
static struct i2c_board_info i2c_ioboard_info[] = {
	{ I2C_BOARD_INFO("pcf8591", 0x49) },			// IO board ADC
	{ I2C_BOARD_INFO("24c256", 0x54), .platform_data = &eeprom_info },	// IO board EEPROM
};
#endif

static struct davinci_i2c_platform_data i2c_pdata = {
	//.bus_freq	= 400			/* kHz, original EVM */,
	.bus_freq	= 100			/* kHz, used in HTC */,
	//.bus_delay = 0      		/* usec, original EVM*/,
	.bus_delay	= 100		/* usec, used in HTC */,
	.sda_pin    = 21            /* GIO21*/,
	.scl_pin    = 20            /* GIO20*/,
};

/* Set the input mux for TVP7002/TVP5146/MTxxxx sensors */
static int dm365htc_setup_video_input(enum vpfe_subdev_id id)
{
	pr_info("dm365htc_setup_video_input (%d)\n", id);
#if 1
	switch((int)id) {
		case VPFE_SUBDEV_TVP5146:
			// GPIO96 is ADV7180 /RESET
			davinci_cfg_reg(DM365_GPIO96);
			gpio_request(96, "adv7180_reset");
			gpio_direction_output(96, 1);
			//gpio_export(96, 0);
			//gpio_export_link(NULL, "adv7180_reset", 96);

			// GPIO96 is ADV7180 /PWRDWN
			davinci_cfg_reg(DM365_GPIO93);
			gpio_request(93, "adv7180_pwrdown");
			gpio_direction_output(93, 1);
			//gpio_export(93, 0);
			//gpio_export_link(NULL, "adv7180_pwrdwn", 96);

			mdelay(10);

			break;
	}
#endif
	return 0;
}

#define ADV7180_STD_ALL        (V4L2_STD_525P_60   | V4L2_STD_625P_50 	|\
				V4L2_STD_NTSC      | V4L2_STD_PAL   	|\
				V4L2_STD_720P_50   | V4L2_STD_720P_60 	|\
				V4L2_STD_1080I_50  | V4L2_STD_1080I_60 	|\
				V4L2_STD_1080P_50  | V4L2_STD_1080P_60)

/* Inputs available at the ADV7180 */
static struct v4l2_input adv7180_inputs[] = {
	{
		.index = 0,
		.name = "Composite",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.std = ADV7180_STD_ALL,
	},
};

static struct vpfe_subdev_info vpfe_sub_devs[] = {
	{
		.module_name = "adv7180",
		//.is_camera = 1,
		//.grp_id = 5,
//		.grp_id = VPFE_SUBDEV_ADV7180,		//
		.grp_id = VPFE_SUBDEV_TVP5146,
		.num_inputs = ARRAY_SIZE(adv7180_inputs),
		.inputs = adv7180_inputs,
#if 1
		.ccdc_if_params = {
			.if_type = VPFE_YCBCR_SYNC_8,
			//.if_type = VPFE_RAW_BAYER,
			//.if_type = VPFE_BT656,
			//.hdpol = VPFE_PINPOL_POSITIVE,
			.vdpol = VPFE_PINPOL_POSITIVE,
			.hdpol = VPFE_PINPOL_NEGATIVE,
			//.vdpol = VPFE_PINPOL_NEGATIVE,
			.ycswap = VPFE_DATA_C,
		},
#endif
		.board_info = {
			I2C_BOARD_INFO("adv7180", 0x20),
		},
	},
};

static struct vpfe_config vpfe_cfg = {
       .setup_input = dm365htc_setup_video_input,
       .num_subdevs = ARRAY_SIZE(vpfe_sub_devs),
       .sub_devs = vpfe_sub_devs,
       .card_name = "DM365 HTC",
       .ccdc = "DM365 ISIF",
       .num_clocks = 1,
       .clocks = {"vpss_master"},
};

#if 0
static int cpld_mmc_get_cd(int module)
{
	int r;
	//pr_warning("cpld_mmc_get_cd(%d)\n", module);
	if(module == 1)
		return 1;		// SD memory is always(?) there
	if(!htc_cpld_is_configured()) {
		pr_warning("MMC: CPLD not configured, cannot get cd\n");
		return -ENXIO;
	}

	/* low == card present */
	r = !htc_cpld_readbit(HTC_GPIO_SD_DET, HTC_CPLD_GPIO_CTRL2);
	//pr_warning("MMC: CPLD returning CD = %d\n", r);
	return r;
}
#else
static int cpld_mmc_get_cd(int module) {
	return 1;
}
#endif

static int cpld_mmc_get_ro(int module)
{
	int r;
	//pr_warning("cpld_mmc_get_ro(%d)\n", module);
	if(module == 1)
		return 0;
	if(!htc_cpld_is_configured()) {
		pr_warning("MMC: CPLD not configured, cannot get ro\n");
		return -ENXIO;
	}

	/* high == card's write protect switch active */
	r = htc_cpld_readbit(HTC_GPIO_SD_WRITE_PROT, HTC_CPLD_GPIO_CTRL2);		// doesn't seem to change!
	//pr_warning("MMC: CPLD returning RO = %d\n", r);
	return r;
}

// mmc0
static struct davinci_mmc_config dm365htc_mmc_config_card = {
//	.get_cd		= cpld_mmc_get_cd,
	.get_ro		= cpld_mmc_get_ro,
	.wires		= 4,
	.max_freq	= 50000000,
	.caps		= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED,
	.version	= MMC_CTLR_VERSION_2,
};

// mmc1
static struct davinci_mmc_config dm365htc_mmc_config_chip = {
	.get_cd		= cpld_mmc_get_cd,
	.get_ro		= cpld_mmc_get_ro,
	.wires		= 4,
	.max_freq	= 50000000,
	.caps		= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED,
	.version	= MMC_CTLR_VERSION_2,
};

static void dm365htc_emac_configure(void)
{
	/*
	 * EMAC pins are multiplexed with GPIO and UART
	 * Further details are available at the DM365 ARM
	 * Subsystem Users Guide(sprufg5.pdf) pages 125 - 127
	 */
	davinci_cfg_reg(DM365_EMAC_TX_EN);
	davinci_cfg_reg(DM365_EMAC_TX_CLK);
	davinci_cfg_reg(DM365_EMAC_COL);
	davinci_cfg_reg(DM365_EMAC_TXD3);
	davinci_cfg_reg(DM365_EMAC_TXD2);
	davinci_cfg_reg(DM365_EMAC_TXD1);
	davinci_cfg_reg(DM365_EMAC_TXD0);
	davinci_cfg_reg(DM365_EMAC_RXD3);
	davinci_cfg_reg(DM365_EMAC_RXD2);
	davinci_cfg_reg(DM365_EMAC_RXD1);
	davinci_cfg_reg(DM365_EMAC_RXD0);
	davinci_cfg_reg(DM365_EMAC_RX_CLK);
	davinci_cfg_reg(DM365_EMAC_RX_DV);
	davinci_cfg_reg(DM365_EMAC_RX_ER);
	davinci_cfg_reg(DM365_EMAC_CRS);
	davinci_cfg_reg(DM365_EMAC_MDIO);
	davinci_cfg_reg(DM365_EMAC_MDCLK);

	/*
	 * EMAC interrupts are multiplexed with GPIO interrupts
	 * Details are available at the DM365 ARM
	 * Subsystem Users Guide(sprufg5.pdf) pages 133 - 134
	 */
	davinci_cfg_reg(DM365_INT_EMAC_RXTHRESH);
	davinci_cfg_reg(DM365_INT_EMAC_RXPULSE);
	davinci_cfg_reg(DM365_INT_EMAC_TXPULSE);
	davinci_cfg_reg(DM365_INT_EMAC_MISCPULSE);
}

static void dm365htc_mmc_configure(void)
{
	/*
	 * MMC/SD pins are multiplexed with GPIO and EMIF
	 * Further details are available at the DM365 ARM
	 * Subsystem Users Guide(sprufg5.pdf) pages 118, 128 - 131
	 */
	davinci_cfg_reg(DM365_SD1_CLK);
	davinci_cfg_reg(DM365_SD1_CMD);
	davinci_cfg_reg(DM365_SD1_DATA3);
	davinci_cfg_reg(DM365_SD1_DATA2);
	davinci_cfg_reg(DM365_SD1_DATA1);
	davinci_cfg_reg(DM365_SD1_DATA0);
}

static void dm365htc_usb_configure(void)
{
	davinci_cfg_reg(DM365_GPIO33);
	gpio_request(33, "usb");
	gpio_direction_output(33, 1);
	setup_usb(500, 8);
}

static void __init htc_init_i2c(void)
{
	davinci_init_i2c(&i2c_pdata);
	i2c_register_board_info(1, i2c_info, ARRAY_SIZE(i2c_info));

#ifdef CONFIG_HTC_AUDIOBOARD
	if(htc_audioboard_present())
		i2c_register_board_info(1, i2c_audioboard_info, ARRAY_SIZE(i2c_audioboard_info));
#endif
#ifdef CONFIG_HTC_FPGA_IO
	if(htc_ioboard_present())
		i2c_register_board_info(1, i2c_ioboard_info, ARRAY_SIZE(i2c_ioboard_info));
#endif
}

#ifdef CONFIG_LEDS_CLASS

struct cpld_led {
	struct led_classdev	cdev;
	int ind;
};

typedef enum {
	lt_cpld,
	lt_fpga_bb,
#ifdef CONFIG_HTC_FPGA_IO
	lt_fpga_io,
#endif
} led_type;

struct cpld_led_def {
	led_type type;
	int reg;
	int bit;
	const char *name;
	const char *trigger;
};

static const struct cpld_led_def cpld_leds[] = {
	{lt_cpld,		HTC_CPLD_GPIO_CTRL1,	HTC_GPIO_LED_SYS,	"htc::sysled", },
	{lt_fpga_bb,	HTC_FPGA_MISC_OUT,	HTC_MISC_OUT_LED_1,	"htc::led1", },
	{lt_fpga_bb,	HTC_FPGA_MISC_OUT,	HTC_MISC_OUT_LED_2,	"htc::led2", },
#ifdef CONFIG_HTC_FPGA_IO
	{lt_fpga_io,	HTC_FPGA_MISC_OUT,	HTC_MISC_OUT_LED_1,	"htc::led3", },
	{lt_fpga_io,	HTC_FPGA_MISC_OUT,	HTC_MISC_OUT_LED_2,	"htc::led4", },
#endif
};

static void cpld_led_set(struct led_classdev *cdev, enum led_brightness b)
{
	struct cpld_led *led = container_of(cdev, struct cpld_led, cdev);
	const struct cpld_led_def *def = &cpld_leds[led->ind];

	switch(def->type) {
		case lt_cpld:
			htc_cpld_writebit(b == LED_OFF, def->bit, def->reg);
			break;
		case lt_fpga_bb:
			htc_fpga_bb_writebit(b == LED_OFF, def->bit, def->reg);
			break;
		case lt_fpga_io:
#ifdef CONFIG_HTC_FPGA_IO
			htc_fpga_io_writebit(b == LED_OFF, def->bit, def->reg);
#endif
			break;
	}
}

static enum led_brightness cpld_led_get(struct led_classdev *cdev)
{
	struct cpld_led *led = container_of(cdev, struct cpld_led, cdev);
	const struct cpld_led_def *def = &cpld_leds[led->ind];

	switch(def->type) {
		case lt_cpld:
			return htc_cpld_readbit(def->bit, def->reg) ? LED_OFF : LED_FULL;
		case lt_fpga_bb:
			return htc_fpga_bb_readbit(def->bit, def->reg) ? LED_OFF : LED_FULL;
		case lt_fpga_io:
#ifdef CONFIG_HTC_FPGA_IO
			return htc_fpga_io_readbit(def->bit, def->reg) ? LED_OFF : LED_FULL;
#else
			break;
#endif
	}
	return LED_OFF;
}

static int __init cpld_leds_init(void)
{
	int	i;

	/* setup LEDs */
	for (i = 0; i < ARRAY_SIZE(cpld_leds); i++) {
		struct cpld_led *led;

		if((cpld_leds[i].type == lt_fpga_io) && !htc_ioboard_present())
			continue;

		led = kzalloc(sizeof(*led), GFP_KERNEL);
		if (!led)
			break;

		led->cdev.name = cpld_leds[i].name;
		led->cdev.brightness_set = cpld_led_set;
		led->cdev.brightness_get = cpld_led_get;
		led->cdev.default_trigger = cpld_leds[i].trigger;
		led->ind = i;

		switch(cpld_leds[i].type) {
			case lt_cpld:
				htc_cpld_writebit(true, cpld_leds[i].bit, cpld_leds[i].reg);
				break;
			case lt_fpga_bb:
				htc_fpga_bb_writebit(true, cpld_leds[i].bit, cpld_leds[i].reg);
				break;
			case lt_fpga_io:
#ifdef CONFIG_HTC_FPGA_IO
				htc_fpga_io_writebit(true, cpld_leds[i].bit, cpld_leds[i].reg);
#endif
				break;
		}

		if (led_classdev_register(NULL, &led->cdev) < 0) {
			kfree(led);
			break;
		}
	}

	return 0;
}
/* run after subsys_initcall() for LEDs */
fs_initcall(cpld_leds_init);
#endif/*CONFIG_LEDS_CLASS*/

static struct davinci_uart_config uart_config __initdata = {
	.enabled_uarts = (1 << 0),
};

static void __init dm365_htc_map_io(void)
{
	/* setup input configuration for VPFE input devices */
	dm365_set_vpfe_config(&vpfe_cfg);
	dm365_init();
}

static struct spi_board_info dm365_htc_spi_info[] __initconst = {
	{
		.modalias		= "owl221a",
		.max_speed_hz	= 10 * 1000 * 1000,	// find out what!
		.bus_num		= 3,
		.chip_select	= 0,
		.mode			= SPI_MODE_0,
	},
	{
		.modalias		= "spidev",
		//.max_speed_hz	= 400000,
		.max_speed_hz	= 12000000,	//
		.bus_num		= 5,
		//.bus_num		= 1,		// TODO: for CPU SPI instead of CPLD SPI
		.chip_select	= 0,
		.mode			= SPI_MODE_0,
	},
};

// Sets up CLKOUT0 to output from CLKOUT0 pin at 12MHz
// We have to do this a bit manually because clock support doesn't seem complete
static int setup_clkout0(void) {
	struct clk *codec_clk;
	void __iomem *pllc1;
	u32 v;

	codec_clk = clk_get(NULL, "clkout0");
	if(codec_clk == NULL || IS_ERR(codec_clk)) {
		pr_err("ERROR: codec clock 'clkout0' not registered!\n");
		return -1;
	}
	// AFAIK all this does is increment the use count
	clk_enable(codec_clk);
	// output it
	davinci_cfg_reg(DM365_CLKOUT0);

	// we can get the PLLC1 register area from the clock's parent's PLL data
	pllc1 = codec_clk->parent->pll_data->base;
	// enable CLKOUT0
	v = __raw_readl(pllc1 + PLLCKEN);
	v |= BIT(1);
	__raw_writel(v, pllc1 + PLLCKEN);
	// set divider to /2 in OSCDIV1
	v = 0x8001;
	__raw_writel(v, pllc1 + 0x124);
	// enable divider in OCSEL
	v = __raw_readl(pllc1 + 0x104);
	v &= ~BIT(4);
	__raw_writel(v, pllc1 + 0x104);

	pr_info("CLKOUT0 setup for 12MHz\n");
	return 0;
}

static struct platform_device htc_odo_device = {
	.name	= "htc_odo",
	.id		= -1,
};

static struct platform_device htc_beeper = {
	.name	= "htc_beep",
	.id		= -1,
};

#ifdef CONFIG_GPIO_HTC_CPLD
static struct platform_device htc_cpld_gpio_device = {
	.name	= "htc_cpld_gpio",
	.id		= -1,
};
#endif

static struct platform_device htc_cpld_wdog_spi_device = {
	.name	= "htc_cpld_wdog_spi",
	.id		= 5,
};

static struct platform_device htc_fpga_gpio_device = {
	.name	= "htc_fpga_gpio",
	.id		= -1,
};

#ifdef CONFIG_HTC_AUDIOBOARD
static struct platform_device htc_fmradio_device = {
	.name	= "radio-tea5764",
	.id		= -1,
};
#endif

static struct platform_device *htc_platform_devices[] __initdata = {
	&htc_odo_device,
	&htc_fpga_gpio_device,
	&htc_cpld_gpio_device,
	&htc_cpld_wdog_spi_device,
	&htc_beeper,
};
#if defined(CONFIG_HTC_AUDIOBOARD) && defined(CONFIG_RADIO_TEA5764)
static struct platform_device *htc_audioboard_devices[] __initdata = {
	&htc_fmradio_device,
};
#endif

#if 0
#define DM365_MMCSD0_BASE	     0x01D11000
static int __init configure_MMCCLK0(void) {
	static void __iomem *emif;

	if (request_mem_region(DM365_MMCSD0_BASE, SZ_4K, "MMC0") == NULL)
		return -1;
	emif = ioremap(DM365_MMCSD0_BASE, SZ_4K);
	if(!emif) {
		release_mem_region(DM365_MMCSD0_BASE, SZ_4K);
		return -1;
	}

	__raw_writel(0, emif + 0x4);

	// we don't need this anymore
	release_mem_region(DM365_MMCSD0_BASE, SZ_4K);
	return 0;
}
#endif

static __init void dm365_htc_init(void)
{
	davinci_cfg_reg(DM365_GPIO50);						// turn off the EMIF_CLK output as soon as possible
														// as it only goes to SD_DET, on the other side of the board!

	davinci_serial_init(&uart_config);
	dm365htc_emac_configure();
	dm365htc_usb_configure();

	/* maybe setup mmc1/etc ... _after_ mmc0 */
	htc_init_cpld();

	htc_init_fpga();
	htc_init_i2c();

	dm365htc_mmc_configure();
	davinci_setup_mmc(0, &dm365htc_mmc_config_card);
	//davinci_setup_mmc(1, &dm365htc_mmc_config_chip);	// for when we only want the SD IC

	setup_clkout0();
	dm365_init_asp(&dm365_htc_snd_data);

	//dm365_init_spi1(0, NULL, 0);
	dm365_init_spi3(BIT(0), dm365_htc_spi_info, ARRAY_SIZE(dm365_htc_spi_info));

// do this later, otherwise we sometimes end up with the two SD card devices swapped
	davinci_setup_mmc(1, &dm365htc_mmc_config_chip);

	platform_add_devices(htc_platform_devices, ARRAY_SIZE(htc_platform_devices));
#if defined(CONFIG_HTC_AUDIOBOARD) && defined(CONFIG_RADIO_TEA5764)
	if(htc_audioboard_present())
		platform_add_devices(htc_audioboard_devices, ARRAY_SIZE(htc_audioboard_devices));
#endif

//	configure_MMCCLK0();

#if 0
	// GPIO96 is ADV7180 /RESET
	davinci_cfg_reg(DM365_GPIO96);
	gpio_request(96, "adv7180_reset");
	gpio_direction_output(96, 1);
	gpio_export(96, 0);
	//gpio_export_link(NULL, "adv7180_reset", 96);

	// GPIO96 is ADV7180 /PWRDWN
	davinci_cfg_reg(DM365_GPIO93);
	gpio_request(93, "adv7180_pwrdown");
	gpio_direction_output(93, 1);
	gpio_export(93, 0);
	//gpio_export_link(NULL, "adv7180_pwrdwn", 96);
#endif
}

static __init void dm365_htc_irq_init(void)
{
	davinci_irq_init();
	htc_init_cpld_irq();
	htc_init_fpga_irq();
}

MACHINE_START(DAVINCI_DM365_HTC, "Hanover Displays DM365 HTC")
	.phys_io	= IO_PHYS,
	.io_pg_offst	= (__IO_ADDRESS(IO_PHYS) >> 18) & 0xfffc,
	.boot_params	= (0x80000100),
	.map_io		= dm365_htc_map_io,
	.init_irq	= dm365_htc_irq_init,
	.timer		= &davinci_timer,
	.init_machine	= dm365_htc_init,
MACHINE_END

