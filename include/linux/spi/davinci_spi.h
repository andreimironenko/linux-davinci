/*
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <mach/hardware.h>

#define DAVINCI_SPI_INTERN_CS	0xFF

enum {
	DAVINCI_SPI_VERSION_1, /* For DM355/DM365/DM6467*/
	DAVINCI_SPI_VERSION_2, /* For DA8xx(Primus) */
};

struct davinci_spi_platform_data {
	u8	version;
	u16	num_chipselect;
	u32	instance;
	u8	*chip_sel;
};

#define DAVINCI_SPI_MAX_CHIPSELECT	2

#define CS_DEFAULT	0xFF
#define SCS0_SELECT	0x01
#define SCS1_SELECT	0x02
#define SCS2_SELECT	0x04
#define SCS3_SELECT	0x08
#define SCS4_SELECT	0x10
#define SCS5_SELECT	0x20
#define SCS6_SELECT	0x40
#define SCS7_SELECT	0x80

#define DAVINCI_SPIFMT_PHASE_MASK	BIT(16)
#define DAVINCI_SPIFMT_PHASE_SHIFT	BIT(4)

#define DAVINCI_SPIFMT_POLARITY_MASK	BIT(17)
#define DAVINCI_SPIFMT_POLARITY_SHIFT	(BIT(4) | BIT(0))

#define DAVINCI_SPIFMT_DISTIMER_MASK	BIT(18)
#define DAVINCI_SPIFMT_DISTIMER_SHIFT	(BIT(4) | BIT(1))

#define DAVINCI_SPIFMT_SHIFTDIR_MASK	BIT(20)
#define DAVINCI_SPIFMT_SHIFTDIR_SHIFT	(BIT(4) | BIT(2))

#define DAVINCI_SPIFMT_WAITENA_MASK	BIT(21)
#define DAVINCI_SPIFMT_WAITENA_SHIFT	0x00000015u

#define DAVINCI_SPIFMT_PARITYENA_MASK	BIT(22)
#define DAVINCI_SPIFMT_PARITYENA_SHIFT	0x00000016u

#define DAVINCI_SPIFMT_ODD_PARITY_MASK	BIT(23)
#define DAVINCI_SPIFMT_ODD_PARITY_SHIFT	0x00000017u

#define DAVINCI_SPIFMT_WDELAY_MASK	0x3f000000u
#define DAVINCI_SPIFMT_WDELAY_SHIFT	0x00000018u

#define DAVINCI_SPIFMT_CHARLEN_MASK	0x0000001Fu
#define DAVINCI_SPIFMT_CHARLEN_SHIFT	0x00000000u

/* SPIGCR1 */

#define DAVINCI_SPIGCR1_SPIENA_MASK	0x01000000u
#define DAVINCI_SPIGCR1_SPIENA_SHIFT	0x00000018u

#define DAVINCI_SPI_INTLVL_1		0x000001FFu
#define DAVINCI_SPI_INTLVL_0		0x00000000u

/* SPIPC0 */
#define DAVINCI_SPIPC0_DIFUN_MASK	BIT(11)
#define DAVINCI_SPIPC0_DIFUN_SHIFT	(BIT(3) | BIT(1) | BIT(0))

/*----DIFUN Tokens----*/
#define DAVINCI_SPIPC0_DIFUN_DI		BIT(0)

#define DAVINCI_SPIPC0_DOFUN_MASK	BIT(10)
#define DAVINCI_SPIPC0_DOFUN_SHIFT	(BIT(3) | BIT(1))

/*----DOFUN Tokens----*/
#define DAVINCI_SPIPC0_DOFUN_DO		BIT(0)
#define DAVINCI_SPIPC0_CLKFUN_MASK	BIT(9)
#define DAVINCI_SPIPC0_CLKFUN_SHIFT	(BIT(3) | BIT(0))

/*----CLKFUN Tokens----*/
#define DAVINCI_SPIPC0_CLKFUN_CLK	BIT(0)

#define DAVINCI_SPIPC0_EN1FUN_MASK	BIT(1)
#define DAVINCI_SPIPC0_EN1FUN_SHIFT	BIT(0)

/*----EN1FUN Tokens----*/
#define DAVINCI_SPIPC0_EN1FUN_EN1	BIT(0)

#define DAVINCI_SPIPC0_EN0FUN_MASK	BIT(0)
#define DAVINCI_SPIPC0_EN0FUN_SHIFT	0

/*----EN0FUN Tokens----*/
#define DAVINCI_SPIPC0_EN0FUN_EN0	BIT(0)

#define DAVINCI_SPIPC0_SPIENA		BIT(0)
#define DAVINCI_SPIPC0_SPIENA_SHIFT	BIT(3)

#define DAVINCI_SPIINT_MASKALL		0x0101035F

/* SPIDAT1 */

#define DAVINCI_SPIDAT1_CSHOLD_MASK	BIT(28)
#define DAVINCI_SPIDAT1_CSHOLD_SHIFT	(BIT(4) | BIT(3) | BIT(2))

#define DAVINCI_SPIDAT1_CSNR_MASK	(BIT(17 | BIT(16))
#define DAVINCI_SPIDAT1_CSNR_SHIFT	BIT(4)

#define DAVINCI_SPIDAT1_DFSEL_MASK	(BIT(24 | BIT(25))
#define DAVINCI_SPIDAT1_DFSEL_SHIFT	(BIT(4) | BIT(3))

#define DAVINCI_SPIGCR1_CLKMOD_MASK	BIT(1)
#define DAVINCI_SPIGCR1_CLKMOD_SHIFT	BIT(0)

#define DAVINCI_SPIGCR1_MASTER_MASK     BIT(0)
#define DAVINCI_SPIGCR1_MASTER_SHIFT	0

#define DAVINCI_SPIGCR1_LOOPBACK_MASK	BIT(16)
#define DAVINCI_SPIGCR1_LOOPBACK_SHIFT	BIT(4)

/* SPIBUF */
#define DAVINCI_SPIBUF_TXFULL_MASK	BIT(29)
#define DAVINCI_SPIBUF_RXEMPTY_MASK	BIT(31)

#define DAVINCI_SPIFLG_DLEN_ERR_MASK		BIT(0)
#define DAVINCI_SPIFLG_TIMEOUT_MASK		BIT(1)
#define DAVINCI_SPIFLG_PARERR_MASK		BIT(2)
#define DAVINCI_SPIFLG_DESYNC_MASK		BIT(3)
#define DAVINCI_SPIFLG_BITERR_MASK		BIT(4)
#define DAVINCI_SPIFLG_OVRRUN_MASK		BIT(6)
#define DAVINCI_SPIFLG_RX_INTR_MASK		BIT(8)
#define DAVINCI_SPIFLG_TX_INTR_MASK		BIT(9)
#define DAVINCI_SPIFLG_BUF_INIT_ACTIVE_MASK	BIT(24)
#define DAVINCI_SPIFLG_MASK			(DAVINCI_SPIFLG_DLEN_ERR_MASK \
		| DAVINCI_SPIFLG_TIMEOUT_MASK | DAVINCI_SPIFLG_PARERR_MASK \
		| DAVINCI_SPIFLG_DESYNC_MASK | DAVINCI_SPIFLG_BITERR_MASK \
		| DAVINCI_SPIFLG_OVRRUN_MASK | DAVINCI_SPIFLG_RX_INTR_MASK \
		| DAVINCI_SPIFLG_TX_INTR_MASK \
		| DAVINCI_SPIFLG_BUF_INIT_ACTIVE_MASK)

#define DAVINCI_SPIINT_DLEN_ERR_INTR	BIT(0)
#define DAVINCI_SPIINT_TIMEOUT_INTR	BIT(1)
#define DAVINCI_SPIINT_PARERR_INTR	BIT(2)
#define DAVINCI_SPIINT_DESYNC_INTR	BIT(3)
#define DAVINCI_SPIINT_BITERR_INTR	BIT(4)
#define DAVINCI_SPIINT_OVRRUN_INTR	BIT(6)
#define DAVINCI_SPIINT_RX_INTR		BIT(8)
#define DAVINCI_SPIINT_TX_INTR		BIT(9)
#define DAVINCI_SPIINT_DMA_REQ_EN	BIT(16)
#define DAVINCI_SPIINT_ENABLE_HIGHZ	BIT(24)

/*Error return codes */
#define DAVINCI_ERROR_BASE		-30
#define DAVINCI_OVERRUN_ERR		(DAVINCI_ERROR_BASE - 1)
#define DAVINCI_BIT_ERR			(DAVINCI_ERROR_BASE - 2)
#define DAVINCI_DESYNC_ERR		(DAVINCI_ERROR_BASE - 3)
#define DAVINCI_PARITY_ERR		(DAVINCI_ERROR_BASE - 4)
#define DAVINCI_TIMEOUT_ERR		(DAVINCI_ERROR_BASE - 5)
#define DAVINCI_TRANSMIT_FULL_ERR	(DAVINCI_ERROR_BASE - 6)
#define DAVINCI_POWERDOWN		(DAVINCI_ERROR_BASE - 7)
#define DAVINCI_DLEN_ERR		(DAVINCI_ERROR_BASE - 8)
#define DAVINCI_TX_INTR_ERR		(DAVINCI_ERROR_BASE - 9)
#define DAVINCI_BUF_INIT_ACTIVE_ERR	(DAVINCI_ERROR_BASE - 11)

#define DAVINCI_BYTELENGTH 8u

enum spi_pin_op_mode {
	OPMODE_3PIN,
	OPMODE_SPISCS_4PIN,
	OPMODE_SPIENA_4PIN,
	OPMODE_5PIN,
};

struct davinci_spi_config {
	/* SPIFMT */
	u32	wdelay;
	u32	odd_parity;
	u32	parity_enable;
	u32	wait_enable;
	u32	timer_disable;
	/* SPIGCR1 */
	u32	clk_internal;
	/* SPIDAT1 */
	u32	cs_hold;
	/* SPIINTLVL1 */
	u32	intr_level;
	/* Others */
	enum spi_pin_op_mode	pin_op_modes;
	u32	poll_mode;
};

/* SPI Controller registers */
#define SPIGCR0		0x00
#define SPIGCR1		0x04
#define SPIINT		0x08
#define SPILVL		0x0c
#define SPIFLG		0x10
#define SPIPC0		0x14
#define SPIPC1		0x18
#define SPIPC2		0x1c
#define SPIPC3		0x20
#define SPIPC4		0x24
#define SPIPC5		0x28
#define SPIPC6		0x2c
#define SPIPC7		0x30
#define SPIPC8		0x34
#define SPIDAT0		0x38
#define SPIDAT1		0x3c
#define SPIBUF		0x40
#define SPIEMU		0x44
#define SPIDELAY	0x48
#define SPIDEF		0x4c
#define SPIFMT0		0x50
#define SPIFMT1		0x54
#define SPIFMT2		0x58
#define SPIFMT3		0x5c
#define TGINTVEC0	0x60
#define TGINTVEC1	0x64

struct davinci_spi_slave {
	u32	cmd_to_write;
	u32	clk_ctrl_to_write;
	u32	bytes_per_word;
	u8	active_cs;
};

/* SPI Controller driver's private data. */
struct davinci_spi {
	/* bitbang has to be first */
	struct spi_bitbang	bitbang;
	struct clk 		*clk;

	u8			version;
	u32			instance;
	resource_size_t		pbase;
	void __iomem		*base;
	size_t			region_size;
	u32			irq;
	struct completion	done;

	const void		*tx;
	void			*rx;
	int			count;
	struct davinci_spi_platform_data *pdata;

	void			(*get_rx)(u32 rx_data, struct davinci_spi *);
	u32			(*get_tx)(struct davinci_spi *);

	struct davinci_spi_slave slave[DAVINCI_SPI_MAX_CHIPSELECT];
};
