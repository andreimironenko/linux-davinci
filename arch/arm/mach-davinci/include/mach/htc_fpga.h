/*
 * HTC FPGA access
 *
 * Author: David Steinberg, Hanover Displays Ltd <dsteinberg@hanoverdisplays.com>
 * Copyright (C)2010 Hanover Displays Ltd
 *
 * Changlog:
 *
 * 18 May 2010		DJS		Initial file
 *
 */

#ifndef __ASM_ARCH_HTC_FPGA_H
#define __ASM_ARCH_HTC_FPGA_H

#include <mach/htc_cpld.h>

#define HTC_FPGA_UART_OFFSET	0x00
#define HTC_FPGA_IO_OFFSET		0x80

#define HTC_FPGA_BB_BASE		0x04000400
#define HTC_FPGA_BB_UART_BASE	(HTC_FPGA_BB_BASE + HTC_FPGA_UART_OFFSET)
#define HTC_FPGA_BB_IO_BASE		(HTC_FPGA_BB_BASE + HTC_FPGA_IO_OFFSET)

#define HTC_FPGA_IO_BASE		0x04000C00
#define HTC_FPGA_IO_UART_BASE	(HTC_FPGA_IO_BASE + HTC_FPGA_UART_OFFSET)
#define HTC_FPGA_IO_IO_BASE		(HTC_FPGA_IO_BASE + HTC_FPGA_IO_OFFSET)

#define HTC_FPGA_BB_IO_SIZE		0x2c
#define HTC_FPGA_IO_IO_SIZE		0x2c

// FPGA UARTs
#define NR_HTC_FPGA_BB_UARTS	8
#define NR_HTC_FPGA_IO_UARTS	5
#ifdef CONFIG_HTC_FPGA_IO
#define NR_HTC_FPGA_UARTS		(NR_HTC_FPGA_BB_UARTS + NR_HTC_FPGA_IO_UARTS)
#else
#define NR_HTC_FPGA_UARTS		(NR_HTC_FPGA_BB_UARTS)
#endif

// FPGA registers
#define HTC_FPGA_ODOMETER			0x00		// 32 bit (all others are 8 bit)
#define HTC_FPGA_MISC_OUT			0x04
#define HTC_FPGA_MISC_IN			0x08
#define HTC_FPGA_AUDIO_OUT			0x0C
#define HTC_FPGA_AUDIO_IN			0x10
#define HTC_FPGA_IRQ_STATUS			0x14
#define HTC_FPGA_IRQ_MASK			0x18
#define HTC_FPGA_IRQ_TYPE			0x1C
#define HTC_FPGA_UART_MODE			0x20
#define HTC_FPGA_UART_IRQ_STATUS	0x24
#define HTC_FPGA_UART_IRQ_MASK		0x28

// Bits in the misc out register
#define HTC_MISC_OUT_LED_2			0
#define HTC_MISC_OUT_LED_1			1
#define HTC_MISC_OUT_IGNITION		2		// IO board FPGA only
#define HTC_MISC_OUT_EMERG_OFF		3		// IO board FPGA only
#define HTC_MISC_OUT_RELAY_1		4
#define HTC_MISC_OUT_RELAY_2		5
#define HTC_MISC_OUT_RELAY_3		6
#define HTC_MISC_OUT_RELAY_4		7
#define HTC_MISC_OUT_RELAY_MASK		0xf0

// Bits in the misc in register
#define HTC_MISC_IN_OPTO_1			0
#define HTC_MISC_IN_OPTO_2			1
#define HTC_MISC_IN_OPTO_3			2
#define HTC_MISC_IN_OPTO_4			3
#define HTC_MISC_IN_MODEM_PWR		4		// IO board FPGA only

// Bits in the audio out register
#define HTC_AUDIO_OUT_MUTE_LEFT		0		// Audio board
#define HTC_AUDIO_OUT_MUTE_RIGHT	1		// Audio board
#define HTC_AUDIO_OUT_POWER_LEFT	2		// Audio board
#define HTC_AUDIO_OUT_POWER_RIGHT	3		// Audio board
#define HTC_AUDIO_OUT_MUTE_BASE		4
#define HTC_AUDIO_OUT_POWER_BASE	5
#define HTC_AUDIO_OUT_DSPCLOCK		6		// enable FPGA clock output

// Bits in the audio input registers
#define HTC_AUDIO_IN_DIAG_LEFT		0
#define HTC_AUDIO_IN_DIAG_RIGHT		1
#define HTC_AUDIO_IN_DIAG_BASE		4
#define HTC_AUDIO_IN_DRIVER_PTT		5
#define HTC_AUDIO_IN_DRIVER_PTT_RISING	6
#define HTC_AUDIO_IN_DRIVER_PTT_FALLING	7

#ifdef __KERNEL__

// first FPGA virtual IRQ
#define NR_HTC_FPGA_BB_IO_IRQS		5
#define NR_HTC_FPGA_IO_IO_IRQS		5
#define NR_HTC_FPGA_BB_UART_IRQS	NR_HTC_FPGA_BB_UARTS
#define NR_HTC_FPGA_IO_UART_IRQS	NR_HTC_FPGA_IO_UARTS
#define HTC_FPGA_BB_IO_IRQ_BASE		(HTC_CPLD_IRQ_BASE - NR_HTC_FPGA_BB_IO_IRQS)
#define HTC_FPGA_IO_IO_IRQ_BASE		(HTC_FPGA_BB_IO_IRQ_BASE - NR_HTC_FPGA_IO_IO_IRQS)

#define HTC_FPGA_BB_UART_IRQ_BASE	(HTC_FPGA_IO_IO_IRQ_BASE - NR_HTC_FPGA_BB_UART_IRQS)
#define HTC_FPGA_IO_UART_IRQ_BASE	(HTC_FPGA_BB_UART_IRQ_BASE - NR_HTC_FPGA_IO_UART_IRQS)

#endif/*__KERNEL__*/

// FPGA IRQs
#define HTC_FPGA_IRQ_OPTO_1			(HTC_FPGA_BB_IO_IRQ_BASE + 0)
#define HTC_FPGA_IRQ_OPTO_2			(HTC_FPGA_BB_IO_IRQ_BASE + 1)
#define HTC_FPGA_IRQ_OPTO_3			(HTC_FPGA_BB_IO_IRQ_BASE + 2)
#define HTC_FPGA_IRQ_OPTO_4			(HTC_FPGA_BB_IO_IRQ_BASE + 3)
#define HTC_FPGA_IRQ_OPTO_5			(HTC_FPGA_IO_IO_IRQ_BASE + 0)
#define HTC_FPGA_IRQ_OPTO_6			(HTC_FPGA_IO_IO_IRQ_BASE + 1)
#define HTC_FPGA_IRQ_OPTO_7			(HTC_FPGA_IO_IO_IRQ_BASE + 2)
#define HTC_FPGA_IRQ_OPTO_8			(HTC_FPGA_IO_IO_IRQ_BASE + 3)
#define HTC_FPGA_IRQ_PTT_BB			(HTC_FPGA_BB_IO_IRQ_BASE + 4)
#define HTC_FPGA_IRQ_PTT_IO			(HTC_FPGA_IO_IO_IRQ_BASE + 4)
//#define HTC_FPGA_IRQ_UART_BB		(HTC_FPGA_BB_IO_IRQ_BASE + 5)
//#define HTC_FPGA_IRQ_UART_IO		(HTC_FPGA_IO_IO_IRQ_BASE + 5)

#ifdef __KERNEL__

extern u8 htc_fpga_bb_readb(u32 offset);
extern void htc_fpga_bb_writeb(u8 value, u32 offset);
extern u32 htc_fpga_bb_readl(u32 offset);
extern void htc_fpga_bb_writel(u32 value, u32 offset);
static inline int htc_fpga_bb_readbit(int bit, u32 offset) {
	return !!(htc_fpga_bb_readb(offset) & BIT(bit));
}
extern void htc_fpga_bb_writebit(int state, int bit, u32 offset);

extern void htc_fpga_bb_lock(void);
extern void htc_fpga_bb_unlock(void);

#ifdef CONFIG_HTC_FPGA_IO
extern u8 htc_fpga_io_readb(u32 offset);
extern void htc_fpga_io_writeb(u8 value, u32 offset);
extern u32 htc_fpga_io_readl(u32 offset);
extern void htc_fpga_io_writel(u32 value, u32 offset);
static inline int htc_fpga_io_readbit(int bit, u32 offset) {
	return !!(htc_fpga_io_readb(offset) & BIT(bit));
}
extern void htc_fpga_io_writebit(int state, int bit, u32 offset);

extern void htc_fpga_io_lock(void);
extern void htc_fpga_io_unlock(void);
#endif/*CONFIG_HTC_FPGA_IO*/

extern int htc_fpga_bb_is_configured(void);
extern int htc_fpga_io_is_configured(void);

extern int htc_configure_fpga(const u8 *data, size_t length);
extern int htc_init_fpga(void);
extern int htc_init_fpga_irq(void);

#endif/*__KERNEL__*/

#endif /* __ASM_ARCH_HTC_FPGA_H */


