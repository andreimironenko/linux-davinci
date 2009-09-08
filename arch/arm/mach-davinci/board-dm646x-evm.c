/*
 * TI DaVinci DM646X EVM board
 *
 * Derived from: arch/arm/mach-davinci/board-evm.c
 * Copyright (C) 2006 Texas Instruments.
 *
 * (C) 2007-2008, MontaVista Software, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/major.h>
#include <linux/root_dev.h>
#include <linux/dma-mapping.h>
#include <linux/serial.h>
#include <linux/serial_8250.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/i2c/at24.h>
#include <linux/i2c/pcf857x.h>
#include <linux/etherdevice.h>
#include <linux/spi/spi.h>
#include <linux/spi/eeprom.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <mach/dm646x.h>
#include <mach/common.h>
#include <mach/psc.h>
#include <mach/serial.h>
#include <mach/i2c.h>
#include <mach/mmc.h>
#include <mach/emac.h>

#if defined(CONFIG_BLK_DEV_PALMCHIP_BK3710) || \
    defined(CONFIG_BLK_DEV_PALMCHIP_BK3710_MODULE)
#define HAS_ATA 1
#else
#define HAS_ATA 0
#endif

/* CPLD Register 0 bits to control ATA */
#define DM646X_EVM_ATA_RST		BIT(0)
#define DM646X_EVM_ATA_PWD		BIT(1)

#define DM646X_EVM_PHY_MASK		(0x2)
#define DM646X_EVM_MDIO_FREQUENCY	(2200000) /* PHY bus frequency */

#define VIDCLKCTL_OFFSET	(0x38)
#define VSCLKDIS_OFFSET		(0x6c)

#define VCH2CLK_MASK		(BIT_MASK(10) | BIT_MASK(9) | BIT_MASK(8))
#define VCH2CLK_SYSCLK8		(BIT(9))
#define VCH2CLK_AUXCLK		(BIT(9) | BIT(8))
#define VCH3CLK_MASK		(BIT_MASK(14) | BIT_MASK(13) | BIT_MASK(12))
#define VCH3CLK_SYSCLK8		(BIT(13))
#define VCH3CLK_AUXCLK		(BIT(14) | BIT(13))

#define VIDCH2CLK		(BIT(10))
#define VIDCH3CLK		(BIT(11))

static struct davinci_uart_config uart_config __initdata = {
	.enabled_uarts = (1 << 0),
};

/* CPLD Register 0 Client: used for I/O Control */
static int cpld_reg0_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	if (HAS_ATA) {
		u8 data;
		struct i2c_msg msg[2] = {
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = 1,
				.buf = &data,
			},
			{
				.addr = client->addr,
				.flags = 0,
				.len = 1,
				.buf = &data,
			},
		};

		/* Clear ATA_RSTn and ATA_PWD bits to enable ATA operation. */
		i2c_transfer(client->adapter, msg, 1);
		data &= ~(DM646X_EVM_ATA_RST | DM646X_EVM_ATA_PWD);
		i2c_transfer(client->adapter, msg + 1, 1);
	}

	return 0;
}

static const struct i2c_device_id cpld_reg_ids[] = {
	{ "cpld_reg0", 0, },
	{ },
};

static struct i2c_driver dm6467evm_cpld_driver = {
	.driver.name	= "cpld_reg0",
	.id_table	= cpld_reg_ids,
	.probe		= cpld_reg0_probe,
};

/* LEDS */

static struct gpio_led evm_leds[] = {
	{ .name = "DS1", .active_low = 1, },
	{ .name = "DS2", .active_low = 1, },
	{ .name = "DS3", .active_low = 1, },
	{ .name = "DS4", .active_low = 1, },
};

static __initconst struct gpio_led_platform_data evm_led_data = {
	.num_leds = ARRAY_SIZE(evm_leds),
	.leds     = evm_leds,
};

static struct platform_device *evm_led_dev;

static int evm_led_setup(struct i2c_client *client, int gpio,
			unsigned int ngpio, void *c)
{
	struct gpio_led *leds = evm_leds;
	int status;

	while (ngpio--) {
		leds->gpio = gpio++;
		leds++;
	};

	evm_led_dev = platform_device_alloc("leds-gpio", 0);
	platform_device_add_data(evm_led_dev, &evm_led_data,
				sizeof(evm_led_data));

	evm_led_dev->dev.parent = &client->dev;
	status = platform_device_add(evm_led_dev);
	if (status < 0) {
		platform_device_put(evm_led_dev);
		evm_led_dev = NULL;
	}
	return status;
}

static int evm_led_teardown(struct i2c_client *client, int gpio,
				unsigned ngpio, void *c)
{
	if (evm_led_dev) {
		platform_device_unregister(evm_led_dev);
		evm_led_dev = NULL;
	}
	return 0;
}

static int evm_sw_gpio[4] = { -EINVAL, -EINVAL, -EINVAL, -EINVAL };

static int evm_sw_setup(struct i2c_client *client, int gpio,
			unsigned ngpio, void *c)
{
	int status;
	int i;
	char label[10];

	for (i = 0; i < 4; ++i) {
		snprintf(label, 10, "user_sw%d", i);
		status = gpio_request(gpio, label);
		if (status)
			goto out_free;
		evm_sw_gpio[i] = gpio++;

		status = gpio_direction_input(evm_sw_gpio[i]);
		if (status) {
			gpio_free(evm_sw_gpio[i]);
			evm_sw_gpio[i] = -EINVAL;
			goto out_free;
		}

		status = gpio_export(evm_sw_gpio[i], 0);
		if (status) {
			gpio_free(evm_sw_gpio[i]);
			evm_sw_gpio[i] = -EINVAL;
			goto out_free;
		}
	}
	return status;
out_free:
	for (i = 0; i < 4; ++i) {
		if (evm_sw_gpio[i] != -EINVAL) {
			gpio_free(evm_sw_gpio[i]);
			evm_sw_gpio[i] = -EINVAL;
		}
	}
	return status;
}

static int evm_sw_teardown(struct i2c_client *client, int gpio,
			unsigned ngpio, void *c)
{
	int i;

	for (i = 0; i < 4; ++i) {
		if (evm_sw_gpio[i] != -EINVAL) {
			gpio_unexport(evm_sw_gpio[i]);
			gpio_free(evm_sw_gpio[i]);
			evm_sw_gpio[i] = -EINVAL;
		}
	}
	return 0;
}

static int evm_pcf_setup(struct i2c_client *client, int gpio,
			unsigned int ngpio, void *c)
{
	int status;

	if (ngpio < 8)
		return -EINVAL;

	status = evm_sw_setup(client, gpio, 4, c);
	if (status)
		return status;

	return evm_led_setup(client, gpio+4, 4, c);
}

static int evm_pcf_teardown(struct i2c_client *client, int gpio,
			unsigned int ngpio, void *c)
{
	BUG_ON(ngpio < 8);

	evm_sw_teardown(client, gpio, 4, c);
	evm_led_teardown(client, gpio+4, 4, c);

	return 0;
}

static struct pcf857x_platform_data pcf_data = {
	.gpio_base	= DAVINCI_N_GPIO+1,
	.setup		= evm_pcf_setup,
	.teardown	= evm_pcf_teardown,
};

/* Most of this EEPROM is unused, but U-Boot uses some data:
 *  - 0x7f00, 6 bytes Ethernet Address
 *  - ... newer boards may have more
 */

static struct at24_platform_data eeprom_info = {
	.byte_len       = (256*1024) / 8,
	.page_size      = 64,
	.flags          = AT24_FLAG_ADDR16,
	.setup          = davinci_get_mac_addr,
	.context	= (void *)0x7f00,
};

static u8 dm646x_iis_serializer_direction[] = {
       TX_MODE, RX_MODE, INACTIVE_MODE, INACTIVE_MODE,
};

static u8 dm646x_dit_serializer_direction[] = {
       TX_MODE,
};

static struct snd_platform_data dm646x_evm_snd_data[] = {
	{
		.tx_dma_offset  = 0x400,
		.rx_dma_offset  = 0x400,
		.op_mode        = DAVINCI_MCASP_IIS_MODE,
		.num_serializer = ARRAY_SIZE(dm646x_iis_serializer_direction),
		.tdm_slots      = 2,
		.serial_dir     = dm646x_iis_serializer_direction,
		.eventq_no      = EVENTQ_0,
	},
	{
		.tx_dma_offset  = 0x400,
		.rx_dma_offset  = 0,
		.op_mode        = DAVINCI_MCASP_DIT_MODE,
		.num_serializer = ARRAY_SIZE(dm646x_dit_serializer_direction),
		.tdm_slots      = 32,
		.serial_dir     = dm646x_dit_serializer_direction,
		.eventq_no      = EVENTQ_0,
	},
};

static struct i2c_client *cpld_client;

static int cpld_video_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	cpld_client = client;
	return 0;
}

static int __devexit cpld_video_remove(struct i2c_client *client)
{
	cpld_client = NULL;
	return 0;
}

static const struct i2c_device_id cpld_video_id[] = {
	{ "cpld_video", 0 },
	{ }
};

static struct i2c_driver cpld_video_driver = {
	.driver = {
		.name	= "cpld_video",
	},
	.probe		= cpld_video_probe,
	.remove		= cpld_video_remove,
	.id_table	= cpld_video_id,
};

static void evm_init_cpld(void)
{
	i2c_add_driver(&cpld_video_driver);
}

static struct i2c_board_info __initdata i2c_info[] =  {
	{
		I2C_BOARD_INFO("24c256", 0x50),
		.platform_data  = &eeprom_info,
	},
	{
		I2C_BOARD_INFO("pcf8574a", 0x38),
		.platform_data	= &pcf_data,
	},
	{
		I2C_BOARD_INFO("cpld_reg0", 0x3a),
	},
	{
		I2C_BOARD_INFO("cpld_video", 0x3B),
	},
	{
		I2C_BOARD_INFO("tlv320aic33", 0x18),
	}
};

static struct davinci_i2c_platform_data i2c_pdata = {
	.bus_freq       = 100 /* kHz */,
	.bus_delay      = 0 /* usec */,
};

static int set_vpif_clock(int mux_mode, int hd)
{
	int val = 0;
	int err = 0;
	unsigned int value;
	void __iomem *base = IO_ADDRESS(DAVINCI_SYSTEM_MODULE_BASE);

	if (!cpld_client)
		return -ENXIO;

	/* disable the clock */
	value = __raw_readl(base + VSCLKDIS_OFFSET);
	value |= (VIDCH3CLK | VIDCH2CLK);
	__raw_writel(value, base + VSCLKDIS_OFFSET);

	val = i2c_smbus_read_byte(cpld_client);
	if (val < 0)
		return val;

	if (mux_mode == 1)
		val &= ~0x40;
	else
		val |= 0x40;

	err = i2c_smbus_write_byte(cpld_client, val);
	if (err)
		return err;

	value = __raw_readl(base + VIDCLKCTL_OFFSET);
	value &= ~(VCH2CLK_MASK);
	value &= ~(VCH3CLK_MASK);

	if (hd >= 1)
		value |= (VCH2CLK_SYSCLK8 | VCH3CLK_SYSCLK8);
	else
		value |= (VCH2CLK_AUXCLK | VCH3CLK_AUXCLK);

	__raw_writel(value, base + VIDCLKCTL_OFFSET);

	/* enable the clock */
	value = __raw_readl(base + VSCLKDIS_OFFSET);
	value &= ~(VIDCH3CLK | VIDCH2CLK);
	__raw_writel(value, base + VSCLKDIS_OFFSET);

	return 0;
}

static const struct vpif_subdev_info dm646x_vpif_subdev[] = {
	{
		.addr	= 0x2A,
		.name	= "adv7343",
	},
	{
		.addr	= 0x2C,
		.name	= "ths7303",
	},
};

static const char *output[] = {
	"Composite",
	"Component",
	"S-Video",
};

static struct vpif_config dm646x_vpif_config = {
	.set_clock	= set_vpif_clock,
	.subdevinfo	= dm646x_vpif_subdev,
	.subdev_count	= ARRAY_SIZE(dm646x_vpif_subdev),
	.output		= output,
	.output_count	= ARRAY_SIZE(output),
	.card_name	= "DM646x EVM",
};

static void __init evm_init_i2c(void)
{
	davinci_init_i2c(&i2c_pdata);
	i2c_add_driver(&dm6467evm_cpld_driver);
	i2c_register_board_info(1, i2c_info, ARRAY_SIZE(i2c_info));
	evm_init_cpld();
}

static void __init davinci_map_io(void)
{
	dm646x_init();
}

static struct spi_eeprom at25640a = {
	.byte_len	= SZ_64K / 2,
	.name		= "at25640a",
	.page_size	= 64,
	.flags		= EE_ADDR2,
};

static struct spi_board_info dm646x_evm_spi_info[] __initconst = {
	{
		.modalias	= "at25",
		.platform_data	= &at25640a,
		.max_speed_hz	= 10 * 1000 * 1000,	/* at 3v3 */
		.bus_num	= 0,
		.chip_select	= 0,
		.mode		= SPI_MODE_0,
	},
};

static __init void evm_init(void)
{
	struct davinci_soc_info *soc_info = &davinci_soc_info;

	evm_init_i2c();
	davinci_serial_init(&uart_config);
	dm646x_init_mcasp0(&dm646x_evm_snd_data[0]);
	dm646x_init_mcasp1(&dm646x_evm_snd_data[1]);

	if (HAS_ATA)
		dm646x_init_ide();

	soc_info->emac_pdata->phy_mask = DM646X_EVM_PHY_MASK;
	soc_info->emac_pdata->mdio_max_freq = DM646X_EVM_MDIO_FREQUENCY;

	dm646x_init_spi0(dm646x_evm_spi_info, ARRAY_SIZE(dm646x_evm_spi_info));
	dm646x_setup_vpif(&dm646x_vpif_config);
}

static __init void davinci_dm646x_evm_irq_init(void)
{
	davinci_irq_init();
}

MACHINE_START(DAVINCI_DM6467_EVM, "DaVinci DM646x EVM")
	.phys_io      = IO_PHYS,
	.io_pg_offst  = (__IO_ADDRESS(IO_PHYS) >> 18) & 0xfffc,
	.boot_params  = (0x80000100),
	.map_io       = davinci_map_io,
	.init_irq     = davinci_dm646x_evm_irq_init,
	.timer        = &davinci_timer,
	.init_machine = evm_init,
MACHINE_END

