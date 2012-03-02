/*
 * Hanover HTC CPU board CPLD access
 *
 * Author: David Steinberg, Hanover Displays Ltd <dsteinberg@hanoverdisplays.com>
 * Copyright (C)2010 Hanover Displays Ltd
 *
 * Changlog:
 *
 * 18 May 2010		DJS		Initial file
 *
 */

#ifndef __ASM_ARCH_HTC_CPLD_H
#define __ASM_ARCH_HTC_CPLD_H

#define HTC_CPLD_BASE				0x04001000
#define HTC_CPLD_SIZE				0x10

// CPLD registers
#define HTC_CPLD_RESET_CTRL			0x00
#define HTC_CPLD_SPARE_IO_CTRL1		0x01
#define HTC_CPLD_SPARE_IO_CTRL2		0x02
#define HTC_CPLD_GPIO_CTRL1			0x03
#define HTC_CPLD_GPIO_CTRL2			0x04
#define HTC_CPLD_IRQ_STATUS			0x05
#define HTC_CPLD_IRQ_MASK			0x06
#define HTC_CPLD_WDOG_CTRL1			0x08
#define HTC_CPLD_WDOG_CTRL2			0x09
#define HTC_CPLD_CFG_CTRL			0x0a
#define HTC_CPLD_VERSION			0x0f

// HTC_CPLD_RESET_CTRL bits
#define HTC_RESET_GPS				0		// active high
#define HTC_RESET_USB				1		// active low
#define HTC_RESET_FPGA_BB			2		// active low
#define HTC_RESET_FPGA_IO			3		// active low
#define HTC_RESET_WIFI				4		// active high
#define HTC_RESET_EXT				5		// active low
#define HTC_RESET_ETH				6		// active low
#define HTC_RESET_VID_IF			7		// active low

// HTC_CPLD_GPIO_CTRL1 bits
#define HTC_GPIO_PUSHTT_OUT			0
#define HTC_GPIO_LED_SYS			1
#define HTC_GPIO_LED_ETH2			2
#define HTC_GPIO_BUZZER				3
#define HTC_GPIO_CPLD				4		// ???
#define HTC_GPIO_PUSHTT_TS			5
//#define HTC_GPIO_SD_WRITE_PROT		6	// we'll use the one in CTRL2
#define HTC_GPIO_EXT_STAT			7

// HTC_CPLD_GPIO_CTRL2 bits
#define HTC_GPIO_PUSHTT_IN			0
#define HTC_GPIO_IOBOARD_DET		1
#define HTC_GPIO_AUDIOBOARD_DET		2
#define HTC_GPIO_SD_DET				3
#define HTC_GPIO_SD_WRITE_PROT		4		// why does this exist twice?
//#define HTC_GPIO_EXT_STAT			5		// why does this exist twice?

// HTC_CPLD_IRQ_STATUS and _MASK bits
#define HTC_IRQ_FPGA_BB0			0
#define HTC_IRQ_FPGA_BB1			1
#define HTC_IRQ_SPI					2		// Connected to OWL221a WiFi module
#define HTC_IRQ_EXT					3
#define HTC_IRQ_ETH					4		// ???
#define HTC_IRQ_FPGA_IO0			5
#define HTC_IRQ_FPGA_IO1			6

#define HTC_IRQ_FPGA_BB_UART		HTC_IRQ_FPGA_BB0
#define HTC_IRQ_FPGA_BB_IO			HTC_IRQ_FPGA_BB1
#define HTC_IRQ_FPGA_IO_UART		HTC_IRQ_FPGA_IO0
#define HTC_IRQ_FPGA_IO_IO			HTC_IRQ_FPGA_IO1

#ifdef __KERNEL__

// first CPLD virtual IRQ
#define NR_HTC_CPLD_IRQS			8
#define HTC_CPLD_IRQ_BASE			(NR_IRQS - NR_HTC_CPLD_IRQS)

// CPLD virtual IRQ numbers
#define HTC_CPLD_IRQ_FPGA_BB0		(HTC_CPLD_IRQ_BASE + HTC_IRQ_FPGA_BB0)
#define HTC_CPLD_IRQ_FPGA_BB1		(HTC_CPLD_IRQ_BASE + HTC_IRQ_FPGA_BB1)
#define HTC_CPLD_IRQ_SPI			(HTC_CPLD_IRQ_BASE + HTC_IRQ_SPI)
#define HTC_CPLD_IRQ_EXT			(HTC_CPLD_IRQ_BASE + HTC_IRQ_EXT)
#define HTC_CPLD_IRQ_ETH			(HTC_CPLD_IRQ_BASE + HTC_IRQ_ETH)
#define HTC_CPLD_IRQ_FPGA_IO0		(HTC_CPLD_IRQ_BASE + HTC_IRQ_FPGA_IO0)
#define HTC_CPLD_IRQ_FPGA_IO1		(HTC_CPLD_IRQ_BASE + HTC_IRQ_FPGA_IO1)

#define HTC_CPLD_IRQ_FPGA_BB_UART	HTC_CPLD_IRQ_FPGA_BB0
#define HTC_CPLD_IRQ_FPGA_BB_IO		HTC_CPLD_IRQ_FPGA_BB1
#define HTC_CPLD_IRQ_FPGA_IO_UART	HTC_CPLD_IRQ_FPGA_IO0
#define HTC_CPLD_IRQ_FPGA_IO_IO		HTC_CPLD_IRQ_FPGA_IO1

#endif/*__KERNEL__*/

// HTC_CPLD_WDOG_CTRL1 bits
#define HTC_WDOG_SCLK				0
#define HTC_WDOG_MOSI				1
#define HTC_WDOG_SCLK_TS			3	// there was meant to be a single line controlling both SCLK and MOSI, but there was obviously a misunderstanding
#define HTC_WDOG_MOSI_TS			4
#define HTC_WDOG_RESET				5
#define HTC_WDOG_MISO				7

// HTC_CPLD_WDOG_CTRL2 bits aren't used

// HTC_CPLD_CFG_CTRL bits
#define HTC_CFG_SCLK				0
#define HTC_CFG_SDATA				1
#define HTC_CFG_CONFIG_N			2
#define HTC_CFG_STATUS_N_OUT		3
#define HTC_CFG_STATUS_N_DIR		4
#define HTC_CFG_STATUS_N_IN			6
#define HTC_CFG_DONE				7

// values for HTC_CFG_STATUS_N_DIR
#define HTC_CFG_STATUS_N_DIR_INPUT	0
#define HTC_CFG_STATUS_N_DIR_OUTPUT	1

#ifdef __KERNEL__

extern u8 htc_cpld_readb(u32 offset);
extern void htc_cpld_writeb(u8 value, u32 offset);

// htc_cpld_writebit() uses a spinlock to be a bit safer
extern void htc_cpld_writebit(int state, int bit, u32 offset);

static inline int htc_cpld_readbit(int bit, u32 offset) {
	return !!(htc_cpld_readb(offset) & BIT(bit));
}

extern void htc_cpld_lock(void);
extern void htc_cpld_unlock(void);

extern int htc_cpld_is_configured(void);
extern int htc_cpld_version(void);
extern void htc_init_cpld(void);
extern int htc_init_cpld_irq(void);

extern int htc_audioboard_present(void);
extern int htc_ioboard_present(void);

#endif/*__KERNEL__*/

#endif /* __ASM_ARCH_HTC_CPLD_H */
