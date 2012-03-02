/*
 * htc-cpld-wdog-spi.c - bitbang SPI bus for HTC CPLD based IOs
 * Needed to support HTC watchdog.
 *
 * Copyright (C) 2010 Hanover Displays Ltd
 *
 */

#if 1
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <mach/htc_cpld.h>

struct wdog_spi {
	struct spi_bitbang	bitbang;
	struct platform_device		*pdev;
};

static inline void setsck(const struct spi_device *spi, int is_on)
{
	htc_cpld_writebit(is_on, HTC_WDOG_SCLK, HTC_CPLD_WDOG_CTRL1);
}

static inline void setmosi(const struct spi_device *spi, int is_on)
{
	htc_cpld_writebit(is_on, HTC_WDOG_MOSI, HTC_CPLD_WDOG_CTRL1);
}

static inline int getmiso(const struct spi_device *spi)
{
	return htc_cpld_readbit(HTC_WDOG_MISO, HTC_CPLD_WDOG_CTRL1);
}

/*
 * NOTE:  this clocks "as fast as we can".  It "should" be a function of the
 * requested device clock.  Software overhead means we usually have trouble
 * reaching even one Mbit/sec (except when we can inline bitops), so for now
 * we'll just assume we never need additional per-bit slowdowns.
 */
//#define spidelay(nsecs)	do {} while (0)
#define spidelay(nsecs)		ndelay(nsecs)

#define	EXPAND_BITBANG_TXRX
#include <linux/spi/spi_bitbang.h>

static u32 wdog_spi_txrx_word_mode0(struct spi_device *spi,
		unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha0(spi, nsecs, 0, word, bits);
}

static u32 wdog_spi_txrx_word_mode1(struct spi_device *spi,
		unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha1(spi, nsecs, 0, word, bits);
}

static u32 wdog_spi_txrx_word_mode2(struct spi_device *spi,
		unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha0(spi, nsecs, 1, word, bits);
}

static u32 wdog_spi_txrx_word_mode3(struct spi_device *spi,
		unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha1(spi, nsecs, 1, word, bits);
}

/*----------------------------------------------------------------------*/

static void wdog_spi_chipselect(struct spi_device *spi, int is_active)
{
	/* set initial clock polarity */
//	if (is_active)
//		setsck(spi, spi->mode & SPI_CPOL);
}

static int wdog_spi_setup(struct spi_device *spi)
{
	int		status = 0;

	if (spi->bits_per_word > 32)
		return -EINVAL;

	if (!status)
		status = spi_bitbang_setup(spi);
	return status;
}

static void wdog_spi_cleanup(struct spi_device *spi)
{
	spi_bitbang_cleanup(spi);
}


static int __init wdog_spi_probe(struct platform_device *pdev)
{
	int				status;
	struct spi_master		*master;
	struct wdog_spi			*ws;

	master = spi_alloc_master(&pdev->dev, sizeof *ws);
	if (!master) {
		status = -ENOMEM;
		goto wdog_free;
	}
	ws = spi_master_get_devdata(master);
	platform_set_drvdata(pdev, ws);

	ws->pdev = pdev;

	master->bus_num = pdev->id;
	master->num_chipselect = 1;
	master->setup = wdog_spi_setup;
	master->cleanup = wdog_spi_cleanup;

	ws->bitbang.master = spi_master_get(master);
	ws->bitbang.chipselect = wdog_spi_chipselect;
	ws->bitbang.txrx_word[SPI_MODE_0] = wdog_spi_txrx_word_mode0;
	ws->bitbang.txrx_word[SPI_MODE_1] = wdog_spi_txrx_word_mode1;
	ws->bitbang.txrx_word[SPI_MODE_2] = wdog_spi_txrx_word_mode2;
	ws->bitbang.txrx_word[SPI_MODE_3] = wdog_spi_txrx_word_mode3;
	ws->bitbang.setup_transfer = spi_bitbang_setup_transfer;
	ws->bitbang.flags = SPI_CS_HIGH;

	htc_cpld_writebit(0, HTC_WDOG_SCLK_TS, HTC_CPLD_WDOG_CTRL1);
	htc_cpld_writebit(0, HTC_WDOG_MOSI_TS, HTC_CPLD_WDOG_CTRL1);

	status = spi_bitbang_start(&ws->bitbang);
	if (status < 0) {
		spi_master_put(ws->bitbang.master);
wdog_free:
		spi_master_put(master);
	} else {
		dev_info(&pdev->dev, "HTC Watchdog SPI driver\n");
	}

	return status;
}

static int __exit wdog_spi_remove(struct platform_device *pdev)
{
	struct wdog_spi	*ws;
	int			status;

	ws = platform_get_drvdata(pdev);

	/* stop() unregisters child devices too */
	status = spi_bitbang_stop(&ws->bitbang);

	spi_master_put(ws->bitbang.master);
	return status;
}

MODULE_ALIAS("platform:htc_cpld_wdog_spi");

static struct platform_driver wdog_spi_driver = {
	.driver.name	= "htc_cpld_wdog_spi",
	.driver.owner	= THIS_MODULE,
	.remove		= __exit_p(wdog_spi_remove),
};

static int __init wdog_spi_init(void)
{
	return platform_driver_probe(&wdog_spi_driver, wdog_spi_probe);
}
device_initcall(wdog_spi_init);

static void __exit wdog_spi_exit(void)
{
	platform_driver_unregister(&wdog_spi_driver);
}
module_exit(wdog_spi_exit);

MODULE_DESCRIPTION("SPI driver for HTC Watchdog");
MODULE_AUTHOR("David Steinberg <dsteinberg@hanoverdisplays.com>");
MODULE_LICENSE("Proprietary");

#else
#define DRIVER_NAME		"htc_cpld_wdog_spi"
#define	SPI_MISO_GPIO	147
#define	SPI_MOSI_GPIO	146
#define	SPI_SCK_GPIO	145
#define	SPI_N_CHIPSEL	0
#include "spi_gpio.c"
MODULE_ALIAS("platform:htc_cpld_wdog_spi");
#endif