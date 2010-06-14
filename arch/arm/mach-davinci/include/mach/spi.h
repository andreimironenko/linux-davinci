/*
 * Copyright 2009 Texas Instruments.
 *
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __ARCH_ARM_DAVINCI_SPI_H
#define __ARCH_ARM_DAVINCI_SPI_H

#define SPI_INTERN_CS	0xFF

/* resource flags for IORESOURCE_DMA resources */
#define IORESOURCE_DMA_RX_CHAN		0x01
#define IORESOURCE_DMA_TX_CHAN		0x02
#define IORESOURCE_DMA_EVENT_Q		0x04

enum {
	SPI_VERSION_1, /* For DM355/DM365/DM6467*/
	SPI_VERSION_2, /* For DA8xx */
};

struct davinci_spi_platform_data {
	u8	version;
	u16	num_chipselect;
	u8	*chip_sel;
};

struct davinci_spi_config {
	u32     odd_parity:1;
	u32     parity_enable:1;
	u32     intr_level:1;
	u32     io_type:2;
#define SPI_IO_TYPE_INTR    0
#define SPI_IO_TYPE_POLL    1
#define SPI_IO_TYPE_DMA     2
	u32     bytes_per_word:2;
	u32     wdelay:6;
	u32     timer_disable:1;
	u8      c2t_delay;
	u8      t2c_delay;
	u8      t2e_delay;
	u8      c2e_delay;
};

#endif	/* __ARCH_ARM_DAVINCI_SPI_H */
