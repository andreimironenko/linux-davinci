/*
 * Copyright (C) 2009 Texas Instruments.
 *
 * controller driver with Interrupt.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/platform_device.h>

#include <linux/spi/davinci_spi.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <mach/mux.h>
#include <mach/gpio.h>

static inline void davinci_spi_rx_buf_u8(u32 data,
					 struct davinci_spi *davinci_spi)
{
	u8 *rx = davinci_spi->rx;

	*rx++ = (u8)data;
	davinci_spi->rx = rx;
}

static inline void davinci_spi_rx_buf_u16(u32 data,
					  struct davinci_spi *davinci_spi)
{
	u16 *rx = davinci_spi->rx;

	*rx++ = (u16)data;
	davinci_spi->rx = rx;
}

static inline u32 davinci_spi_tx_buf_u8(struct davinci_spi *davinci_spi)
{
	u32 data;
	const u8 *tx = davinci_spi->tx;

	data = *tx++;
	davinci_spi->tx = tx;
	return data;
}

static inline u32 davinci_spi_tx_buf_u16(struct davinci_spi *davinci_spi)
{
	u32 data;
	const u16 *tx = davinci_spi->tx;

	data = *tx++;
	davinci_spi->tx = tx;
	return data;
}

static inline void set_bits(void __iomem *addr, u32 bits)
{
	u32 v = ioread32(addr);
	v |= bits;
	iowrite32(v, addr);
}

static inline void clear_bits(void __iomem *addr, u32 bits)
{
	u32 v = ioread32(addr);
	v &= ~bits;
	iowrite32(v, addr);
}

static inline void set_fmt_bits(void __iomem *addr, u32 bits, u32 instance)
{
	set_bits(addr + SPIFMT0 + (0x4 * instance), bits);
}

static inline void clear_fmt_bits(void __iomem *addr, u32 bits, u32 instance)
{
	clear_bits(addr + SPIFMT0 + (0x4 * instance), bits);
}

/*
 * Interface to control the chip select signal
 */
static void davinci_spi_chipselect(struct spi_device *spi, int value)
{
	struct davinci_spi *davinci_spi;
	u32 data1_reg_val = 0;
	struct davinci_spi_platform_data *pdata;
	int i;

	davinci_spi = spi_master_get_devdata(spi->master);
	pdata = davinci_spi->pdata;

	/*
	 * Board specific chip select logic decides the polarity and cs
	 * line for the controller
	 */
	if (value == BITBANG_CS_INACTIVE) {
		/* set all chip select high */
		if (pdata->chip_sel != NULL) {
			for (i = 0; i < pdata->num_chipselect; i++) {
				if (pdata->chip_sel[i] != DAVINCI_SPI_INTERN_CS)
					__gpio_set_value(pdata->chip_sel[i], 1);
			}
		}

		set_bits(davinci_spi->base + SPIDEF, CS_DEFAULT);

		data1_reg_val |= CS_DEFAULT << DAVINCI_SPIDAT1_CSNR_SHIFT;
		iowrite32(data1_reg_val, davinci_spi->base + SPIDAT1);

		while (1)
			if (ioread32(davinci_spi->base + SPIBUF)
					& DAVINCI_SPIBUF_RXEMPTY_MASK)
				break;
	}
}

/**
 * davinci_spi_setup_transfer - This functions will determine transfer method
 * @spi: spi device on which data transfer to be done
 * @t: spi transfer in which transfer info is filled
 *
 * This function determines data transfer method (8/16/32 bit transfer).
 * It will also set the SPI Clock Control register according to
 * SPI slave device freq.
 */
static int davinci_spi_setup_transfer(struct spi_device *spi,
		struct spi_transfer *t)
{

	struct davinci_spi *davinci_spi;
	struct davinci_spi_platform_data *pdata;
	u8 bits_per_word = 0;
	u32 hz = 0, prescale;

	davinci_spi = spi_master_get_devdata(spi->master);
	pdata = davinci_spi->pdata;

	if (t) {
		bits_per_word = t->bits_per_word;
		hz = t->speed_hz;
	}

	/* if bits_per_word is not set then set it default */
	if (!bits_per_word)
		bits_per_word = spi->bits_per_word;

	/* Assign function pointer to appropriate transfer method */
	/* 8bit/16bit or 32bit transfer */
	if (bits_per_word <= 8 && bits_per_word >= 2) {
		davinci_spi->get_rx = davinci_spi_rx_buf_u8;
		davinci_spi->get_tx = davinci_spi_tx_buf_u8;
		davinci_spi->slave[spi->chip_select].bytes_per_word = 1;
	} else if (bits_per_word <= 16 && bits_per_word >= 2) {
		davinci_spi->get_rx = davinci_spi_rx_buf_u16;
		davinci_spi->get_tx = davinci_spi_tx_buf_u16;
		davinci_spi->slave[spi->chip_select].bytes_per_word = 2;
	} else
		return -1;

	if (!hz)
		hz = spi->max_speed_hz;

	clear_fmt_bits(davinci_spi->base, DAVINCI_SPIFMT_CHARLEN_MASK,
			pdata->instance);
	set_fmt_bits(davinci_spi->base, bits_per_word & 0x1f, pdata->instance);

	prescale = ((clk_get_rate(davinci_spi->clk) / hz) - 1) & 0xff;

	clear_fmt_bits(davinci_spi->base, 0x0000ff00, pdata->instance);
	set_fmt_bits(davinci_spi->base, prescale << 8, pdata->instance);

	return 0;
}

/**
 * davinci_spi_setup - This functions will set default transfer method
 * @spi: spi device on which data transfer to be done
 *
 * This functions sets the default transfer method.
 */

static int davinci_spi_setup(struct spi_device *spi)
{
	int retval;
	struct davinci_spi *davinci_spi;

	davinci_spi = spi_master_get_devdata(spi->master);

	/* if bits per word length is zero then set it default 8 */
	if (!spi->bits_per_word)
		spi->bits_per_word = 8;

	/*
	 * SPI in DaVinci and DA8xx operate between
	 * 600 KHz and 50 MHz
	 */
	if (spi->max_speed_hz < 600000 || spi->max_speed_hz > 50000000)
		return -EINVAL;

	retval = davinci_spi_setup_transfer(spi, NULL);

	return retval;
}

static int davinci_spi_bufs_prep(struct spi_device *spi,
				 struct davinci_spi *davinci_spi,
				 struct davinci_spi_config *spi_cfg)
{
	u32 sPIPC0;

	/* configuraton parameter for SPI */
	if (spi->mode & SPI_LSB_FIRST)
		set_fmt_bits(davinci_spi->base, DAVINCI_SPIFMT_SHIFTDIR_MASK,
				davinci_spi->instance);
	else
		clear_fmt_bits(davinci_spi->base, DAVINCI_SPIFMT_SHIFTDIR_MASK,
				davinci_spi->instance);

	if (spi->mode & SPI_CPOL)
		set_fmt_bits(davinci_spi->base, DAVINCI_SPIFMT_POLARITY_MASK,
				davinci_spi->instance);
	else
		clear_fmt_bits(davinci_spi->base, DAVINCI_SPIFMT_POLARITY_MASK,
				davinci_spi->instance);

	if (spi->mode & SPI_CPHA)
		set_fmt_bits(davinci_spi->base, DAVINCI_SPIFMT_PHASE_MASK,
				davinci_spi->instance);
	else
		clear_fmt_bits(davinci_spi->base, DAVINCI_SPIFMT_PHASE_MASK,
				davinci_spi->instance);

	if (davinci_spi->version == DAVINCI_SPI_VERSION_2) {
		clear_fmt_bits(davinci_spi->base, DAVINCI_SPIFMT_WDELAY_MASK,
				davinci_spi->instance);
		set_fmt_bits(davinci_spi->base,
			    ((spi_cfg->wdelay << DAVINCI_SPIFMT_WDELAY_SHIFT)
				& DAVINCI_SPIFMT_WDELAY_MASK),
				davinci_spi->instance);

		if (spi_cfg->odd_parity)
			set_fmt_bits(davinci_spi->base,
					DAVINCI_SPIFMT_ODD_PARITY_MASK,
					davinci_spi->instance);
		else
			clear_fmt_bits(davinci_spi->base,
					DAVINCI_SPIFMT_ODD_PARITY_MASK,
					davinci_spi->instance);

		if (spi_cfg->parity_enable)
			set_fmt_bits(davinci_spi->base,
					DAVINCI_SPIFMT_PARITYENA_MASK,
					davinci_spi->instance);
		else
			clear_fmt_bits(davinci_spi->base,
					DAVINCI_SPIFMT_PARITYENA_MASK,
					davinci_spi->instance);

		if (spi_cfg->wait_enable)
			set_fmt_bits(davinci_spi->base,
					DAVINCI_SPIFMT_WAITENA_MASK,
					davinci_spi->instance);
		else
			clear_fmt_bits(davinci_spi->base,
					DAVINCI_SPIFMT_WAITENA_MASK,
					davinci_spi->instance);

		if (spi_cfg->timer_disable)
			set_fmt_bits(davinci_spi->base,
					DAVINCI_SPIFMT_DISTIMER_MASK,
					davinci_spi->instance);
		else
			clear_fmt_bits(davinci_spi->base,
					DAVINCI_SPIFMT_DISTIMER_MASK,
					davinci_spi->instance);
	}

	/* Clock internal */
	if (spi_cfg->clk_internal)
		set_bits(davinci_spi->base + SPIGCR1,
				DAVINCI_SPIGCR1_CLKMOD_MASK);
	else
		clear_bits(davinci_spi->base + SPIGCR1,
				DAVINCI_SPIGCR1_CLKMOD_MASK);

	/* master mode default */
	set_bits(davinci_spi->base + SPIGCR1, DAVINCI_SPIGCR1_MASTER_MASK);

	if (spi_cfg->intr_level)
		iowrite32(DAVINCI_SPI_INTLVL_1, davinci_spi->base + SPILVL);
	else
		iowrite32(DAVINCI_SPI_INTLVL_0, davinci_spi->base + SPILVL);

	switch (spi_cfg->pin_op_modes) {
	case OPMODE_3PIN:
		sPIPC0 = (DAVINCI_SPIPC0_DIFUN_DI <<
				DAVINCI_SPIPC0_DIFUN_SHIFT)
			| (DAVINCI_SPIPC0_DOFUN_DO <<
				DAVINCI_SPIPC0_DOFUN_SHIFT)
			| (DAVINCI_SPIPC0_CLKFUN_CLK <<
				DAVINCI_SPIPC0_CLKFUN_SHIFT);
		break;

	case OPMODE_SPISCS_4PIN:
		sPIPC0 = (DAVINCI_SPIPC0_DIFUN_DI <<
				DAVINCI_SPIPC0_DIFUN_SHIFT)
			| (DAVINCI_SPIPC0_DOFUN_DO <<
				DAVINCI_SPIPC0_DOFUN_SHIFT)
			| (DAVINCI_SPIPC0_CLKFUN_CLK <<
				DAVINCI_SPIPC0_CLKFUN_SHIFT)
			| (1 << spi->chip_select);
		break;

	case OPMODE_SPIENA_4PIN:
		sPIPC0 = (DAVINCI_SPIPC0_DIFUN_DI <<
				DAVINCI_SPIPC0_DIFUN_SHIFT)
			| (DAVINCI_SPIPC0_DOFUN_DO <<
				DAVINCI_SPIPC0_DOFUN_SHIFT)
			| (DAVINCI_SPIPC0_CLKFUN_CLK <<
				DAVINCI_SPIPC0_CLKFUN_SHIFT)
			| (DAVINCI_SPIPC0_SPIENA <<
				DAVINCI_SPIPC0_SPIENA_SHIFT);
		break;

	case OPMODE_5PIN:
		sPIPC0 = (DAVINCI_SPIPC0_DIFUN_DI <<
				DAVINCI_SPIPC0_DIFUN_SHIFT)
			| (DAVINCI_SPIPC0_DOFUN_DO <<
				DAVINCI_SPIPC0_DOFUN_SHIFT)
			| (DAVINCI_SPIPC0_CLKFUN_CLK <<
				DAVINCI_SPIPC0_CLKFUN_SHIFT)
			| (DAVINCI_SPIPC0_SPIENA <<
				DAVINCI_SPIPC0_SPIENA_SHIFT)
			| (1 << spi->chip_select);
		break;

	default:
		return -1;
	}

	iowrite32(sPIPC0, davinci_spi->base + SPIPC0);

	if (spi->mode & SPI_LOOP)
		set_bits(davinci_spi->base + SPIGCR1,
				DAVINCI_SPIGCR1_LOOPBACK_MASK);
	else
		clear_bits(davinci_spi->base + SPIGCR1,
				DAVINCI_SPIGCR1_LOOPBACK_MASK);

	return 0;
}

static int davinci_spi_check_error(struct davinci_spi *davinci_spi,
		int int_status)
{
	int ret = 0;

	if (int_status & DAVINCI_SPIFLG_TIMEOUT_MASK) {
		pr_info("SPI Time-out Error\n");
		ret = DAVINCI_TIMEOUT_ERR;
	}
	if (int_status & DAVINCI_SPIFLG_DESYNC_MASK) {
		pr_info("SPI Desynchronization Error\n");
		ret = DAVINCI_DESYNC_ERR;
	}
	if (int_status & DAVINCI_SPIFLG_BITERR_MASK) {
		pr_info("SPI Bit error\n");
		ret = DAVINCI_BIT_ERR;
	}

	if (davinci_spi->version == DAVINCI_SPI_VERSION_2) {
		if (int_status & DAVINCI_SPIFLG_DLEN_ERR_MASK) {
			pr_info("SPI Data Length Error\n");
			ret = DAVINCI_DLEN_ERR;
		}
		if (int_status & DAVINCI_SPIFLG_PARERR_MASK) {
			pr_info("SPI Parity Error\n");
			ret = DAVINCI_DLEN_ERR;
		}
		if (int_status & DAVINCI_SPIFLG_OVRRUN_MASK) {
			pr_info("SPI Data Overrun error\n");
			ret = DAVINCI_OVERRUN_ERR;
		}
		if (int_status & DAVINCI_SPIFLG_TX_INTR_MASK) {
			pr_info("SPI TX intr bit set\n");
			ret = DAVINCI_TX_INTR_ERR;
		}
		if (int_status & DAVINCI_SPIFLG_BUF_INIT_ACTIVE_MASK) {
			pr_info("SPI Buffer Init Active\n");
			ret = DAVINCI_BUF_INIT_ACTIVE_ERR;
		}
	}

	return ret;
}

/**
 * davinci_spi_bufs - functions which will handle transfer data
 * @spi: spi device on which data transfer to be done
 * @t: spi transfer in which transfer info is filled
 *
 * This function will put data to be transferred into data register
 * of SPI controller and then wait untill the completion will be marked
 * by the IRQ Handler.
 */
static int davinci_spi_bufs_pio(struct spi_device *spi, struct spi_transfer *t)
{
	struct davinci_spi *davinci_spi;
	int int_status, count, ret;
	u8 conv, tmp;
	u32 tx_data, data1_reg_val;
	struct davinci_spi_config *spi_cfg;
	u32 buf_val, flg_val;
	struct davinci_spi_platform_data *pdata;

	davinci_spi = spi_master_get_devdata(spi->master);
	pdata = davinci_spi->pdata;

	davinci_spi->tx = t->tx_buf;
	davinci_spi->rx = t->rx_buf;

	/* convert len to words bbased on bits_per_word */
	conv = davinci_spi->slave[spi->chip_select].bytes_per_word;
	davinci_spi->count = t->len / conv;

	INIT_COMPLETION(davinci_spi->done);

	spi_cfg = spi->controller_data;

	ret = davinci_spi_bufs_prep(spi, davinci_spi, spi_cfg);
	if (ret)
		return ret;

	/* Enable SPI */
	set_bits(davinci_spi->base + SPIGCR1, DAVINCI_SPIGCR1_SPIENA_MASK);

	/* C2TDELAY = 8h and T2CDELAY = 8h */
	iowrite32(0 | (8 << 24) | (8 << 16), davinci_spi->base + SPIDELAY);

	count = davinci_spi->count;
	data1_reg_val = spi_cfg->cs_hold << DAVINCI_SPIDAT1_CSHOLD_SHIFT;

	tmp = ~(0x1 << spi->chip_select);

	/* checking for GPIO */
	if ((pdata->chip_sel != NULL) &&
	    (pdata->chip_sel[spi->chip_select] != DAVINCI_SPI_INTERN_CS))
		__gpio_set_value(pdata->chip_sel[spi->chip_select], 0);
	 else
		clear_bits(davinci_spi->base + SPIDEF, ~tmp);

	data1_reg_val |= tmp << DAVINCI_SPIDAT1_CSNR_SHIFT;

	while (1)
		if (ioread32(davinci_spi->base + SPIBUF)
				& DAVINCI_SPIBUF_RXEMPTY_MASK)
			break;

	/* Determine the command to execute READ or WRITE */
	if (t->tx_buf) {
		clear_bits(davinci_spi->base + SPIINT, DAVINCI_SPIINT_MASKALL);

		while (1) {
			tx_data = davinci_spi->get_tx(davinci_spi);

			data1_reg_val &= ~(0xFFFF);
			data1_reg_val |= (0xFFFF & tx_data);

			buf_val = ioread32(davinci_spi->base + SPIBUF);
			if ((buf_val & DAVINCI_SPIBUF_TXFULL_MASK) == 0) {
				iowrite32(data1_reg_val,
						davinci_spi->base + SPIDAT1);

				count--;
			}
			while (ioread32(davinci_spi->base + SPIBUF)
					& DAVINCI_SPIBUF_RXEMPTY_MASK)
				udelay(1);
			/* getting the returned byte */
			if (t->rx_buf) {
				buf_val = ioread32(davinci_spi->base + SPIBUF);
				davinci_spi->get_rx(buf_val, davinci_spi);
			}
			if (count <= 0)
				break;
		}
	} else {
		if (spi_cfg->poll_mode) {	/* In Polling mode receive */
			while (1) {
				/* keeps the serial clock going */
				if ((ioread32(davinci_spi->base + SPIBUF)
					& DAVINCI_SPIBUF_TXFULL_MASK) == 0)
					iowrite32(data1_reg_val,
						davinci_spi->base + SPIDAT1);

				while ((ioread32(davinci_spi->base + SPIBUF)) &
					(DAVINCI_SPIBUF_RXEMPTY_MASK))
						;

				flg_val = ioread32(davinci_spi->base + SPIFLG);
				buf_val = ioread32(davinci_spi->base + SPIBUF);

				davinci_spi->get_rx(buf_val, davinci_spi);

				count--;
				if (count <= 0)
					break;
			}
		} else {	/* Receive in Interrupt mode */
			int i;

			for (i = 0; i < davinci_spi->count; i++) {
				set_bits(davinci_spi->base + SPIINT,
						DAVINCI_SPIINT_BITERR_INTR
						| DAVINCI_SPIINT_OVRRUN_INTR
						| DAVINCI_SPIINT_RX_INTR);

				iowrite32(data1_reg_val,
						davinci_spi->base + SPIDAT1);

				while (ioread32(davinci_spi->base + SPIINT) &
					(DAVINCI_SPIINT_RX_INTR))
					;
			}
			iowrite32((data1_reg_val & 0x0ffcffff),
					davinci_spi->base + SPIDAT1);
		}
	}

	/*
	 * Check for bit error, desync error,parity error,timeout error and
	 * receive overflow errors
	 */
	int_status = ioread32(davinci_spi->base + SPIFLG);

	ret = davinci_spi_check_error(davinci_spi, int_status);
	if (ret != 0)
		return ret;

	/* SPI Framework maintains the count only in bytes so convert back */
	davinci_spi->count *= conv;

	return t->len;
}

/**
 * davinci_spi_irq - probe function for SPI Master Controller
 * @irq: IRQ number for this SPI Master
 * @context_data: structure for SPI Master controller davinci_spi
 *
 * ISR will determine that interrupt arrives either for READ or WRITE command.
 * According to command it will do the appropriate action. It will check
 * transfer length and if it is not zero then dispatch transfer command again.
 * If transfer length is zero then it will indicate the COMPLETION so that
 * davinci_spi_bufs function can go ahead.
 */
static irqreturn_t davinci_spi_irq(s32 irq, void *context_data)
{
	struct davinci_spi *davinci_spi = context_data;
	u32 int_status, rx_data = 0;
	irqreturn_t ret = IRQ_NONE;

	int_status = ioread32(davinci_spi->base + SPIFLG);
	while ((int_status & DAVINCI_SPIFLG_MASK) != 0) {
		ret = IRQ_HANDLED;

		if (likely(int_status & DAVINCI_SPIFLG_RX_INTR_MASK)) {
			rx_data = ioread32(davinci_spi->base + SPIBUF);
			davinci_spi->get_rx(rx_data, davinci_spi);

			/* Disable Receive Interrupt */
			iowrite32(~DAVINCI_SPIINT_RX_INTR,
					davinci_spi->base + SPIINT);
		} else /* Ignore errors if have good intr */
			(void)davinci_spi_check_error(davinci_spi, int_status);

		int_status = ioread32(davinci_spi->base + SPIFLG);
	}

	return ret;
}

/*
 * davinci_spi_probe - probe function for SPI Master Controller
 * @pdev: platform_device structure which contains plateform specific data
 *
 * According to Linux Device Model this function will be invoked by Linux
 * with plateform_device struct which contains the device specific info.
 * This function will map the SPI controller's memory, register IRQ,
 * Reset SPI controller and setting its registers to default value.
 * It will invoke spi_bitbang_start to create work queue so that client driver
 * can register transfer method to work queue.
 */
static int davinci_spi_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct davinci_spi *davinci_spi;
	struct davinci_spi_platform_data *pdata;
	struct resource *r, *mem;
	int i = 0, ret = 0;

	pdata = pdev->dev.platform_data;
	if (pdata == NULL) {
		ret = -ENODEV;
		goto err;
	}

	master = spi_alloc_master(&pdev->dev, sizeof(struct davinci_spi));
	if (master == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	dev_set_drvdata(&pdev->dev, master);

	davinci_spi = spi_master_get_devdata(master);
	if (davinci_spi == NULL) {
		ret = -ENOENT;
		goto free_master;
	}

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		ret = -ENOENT;
		goto free_master;
	}

	davinci_spi->pbase = r->start;
	davinci_spi->region_size = resource_size(r);
	davinci_spi->pdata = pdata;

	/* initialize gpio used as chip select */
	if (pdata->chip_sel != NULL) {
		for (i = 0; i < pdata->num_chipselect; i++) {
			if (pdata->chip_sel[i] != DAVINCI_SPI_INTERN_CS) {
				gpio_request(pdata->chip_sel[i], "spi");
				gpio_direction_output(pdata->chip_sel[i], 1);
			}
		}
	}

	mem = request_mem_region(r->start, davinci_spi->region_size,
					pdev->name);
	if (mem == NULL) {
		ret = -EBUSY;
		goto free_master;
	}

	davinci_spi->base = (struct davinci_spi_reg __iomem *)
			ioremap(r->start, davinci_spi->region_size);
	if (davinci_spi->base == NULL) {
		ret = -ENOMEM;
		goto release_region;
	}

	davinci_spi->irq = platform_get_irq(pdev, 0);
	if (davinci_spi->irq <= 0) {
		ret = -EINVAL;
		goto unmap_io;
	}

	ret = request_irq(davinci_spi->irq, davinci_spi_irq, IRQF_DISABLED,
			  pdev->name, davinci_spi);
	if (ret != 0) {
		ret = -EAGAIN;
		goto unmap_io;
	}

	davinci_spi->bitbang.master = spi_master_get(master);
	if (davinci_spi->bitbang.master == NULL) {
		ret = -ENODEV;
		goto err1;
	}

	davinci_spi->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(davinci_spi->clk)) {
		ret = -ENODEV;
		goto put_master;
	}
	clk_enable(davinci_spi->clk);


	master->bus_num = pdev->id;
	master->num_chipselect = pdata->num_chipselect;
	master->setup = davinci_spi_setup;

	davinci_spi->bitbang.chipselect = davinci_spi_chipselect;
	davinci_spi->bitbang.setup_transfer = davinci_spi_setup_transfer;

	davinci_spi->bitbang.txrx_bufs = davinci_spi_bufs_pio;
	davinci_spi->version = pdata->version;
	davinci_spi->get_rx = davinci_spi_rx_buf_u8;
	davinci_spi->get_tx = davinci_spi_tx_buf_u8;

	init_completion(&davinci_spi->done);

	/* Reset In/OUT SPI modle */
	iowrite32(0, davinci_spi->base + SPIGCR0);
	udelay(100);
	iowrite32(1, davinci_spi->base + SPIGCR0);

	ret = spi_bitbang_start(&davinci_spi->bitbang);
	if (ret != 0)
		goto free_clk;

	pr_info("Davinci SPI Controller driver at "
		"0x%p (irq = %d) \n",
		davinci_spi->base, davinci_spi->irq);

	return ret;

free_clk:
	clk_disable(davinci_spi->clk);
	clk_put(davinci_spi->clk);
put_master:
	spi_master_put(master);
err1:
	free_irq(davinci_spi->irq, davinci_spi);
unmap_io:
	iounmap(davinci_spi->base);
release_region:
	release_mem_region(davinci_spi->pbase, davinci_spi->region_size);
free_master:
	kfree(master);
err:
	return ret;
}

/**
 * davinci_spi_remove - remove function for SPI Master Controller
 * @pdev: platform_device structure which contains plateform specific data
 *
 * This function will do the reverse action of davinci_spi_probe function
 * It will free the IRQ and SPI controller's memory region.
 * It will also call spi_bitbang_stop to destroy the work queue which was
 * created by spi_bitbang_start.
 */
static int __exit davinci_spi_remove(struct platform_device *pdev)
{
	struct davinci_spi *davinci_spi;
	struct spi_master *master;

	master = dev_get_drvdata(&pdev->dev);
	davinci_spi = spi_master_get_devdata(master);

	spi_bitbang_stop(&davinci_spi->bitbang);

	clk_disable(davinci_spi->clk);
	clk_put(davinci_spi->clk);
	spi_master_put(master);
	free_irq(davinci_spi->irq, davinci_spi);
	iounmap(davinci_spi->base);
	release_mem_region(davinci_spi->pbase, davinci_spi->region_size);

	spi_master_put(master);

	return 0;
}

static struct platform_driver davinci_spi_driver = {
	.driver.name = "spi_davinci",
	.remove = __exit_p(davinci_spi_remove),
};

static int __init davinci_spi_init(void)
{
	return platform_driver_probe(&davinci_spi_driver, davinci_spi_probe);
}

static void __exit davinci_spi_exit(void)
{
	platform_driver_unregister(&davinci_spi_driver);
}

module_init(davinci_spi_init);
module_exit(davinci_spi_exit);

MODULE_DESCRIPTION("TI DaVinci SPI Master Controller Driver");
MODULE_LICENSE("GPL");
