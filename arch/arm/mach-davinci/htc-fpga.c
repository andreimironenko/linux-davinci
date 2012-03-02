/*
 * Configure HTC FPGA and provide access to registers
 *
 * Author: David Steinberg, Hanover Displays Ltd <dsteinberg@hanoverdisplays.com>
 * Copyright (C)2010 Hanover Displays Ltd
 *
 * Changlog:
 *
 * 18 May 2010		DJS		Initial file
 *
 */

#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/delay.h>
#include <mach/htc_cpld.h>
#include <mach/htc_fpga.h>
#include <linux/serial_8250.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

//#define FPGA_DEBUG_IRQ
//#define FPGA_DEBUG_IO
//#define FPGA_DISABLE_IO_BB
//#define FPGA_DISABLE_IO_IO

//#define DISABLE_IO_IRQS

#ifdef FPGA_DEBUG_IRQ
#define irq_pr_info		pr_info
#else
#define irq_pr_info(x...)	do { } while(0)
#endif
#ifdef FPGA_DEBUG_IO
#define io_pr_info		pr_info
#else
#define io_pr_info(x...)	do { } while(0)
#endif

// default configuration
static const u8 default_config[] __initdata = {
	#include "htc-fpga-bb-config.h"
	#ifdef CONFIG_HTC_FPGA_IO
		#include "htc-fpga-io-config.h"
	#endif
};

static void __iomem *fpga_bb = NULL;
static void __iomem *fpga_io = NULL;

static DEFINE_SPINLOCK(fpga_bb_lock);
#ifdef CONFIG_HTC_FPGA_IO
static DEFINE_SPINLOCK(fpga_io_lock);
#endif

u8 htc_fpga_bb_readb(u32 offset) {
	u8 res = 0;
	if(fpga_bb)
		res = __raw_readb(fpga_bb + offset);
	io_pr_info("FPGA BB: Read byte 0x%02x = 0x%02x\n", offset, res);
	return res;
}
EXPORT_SYMBOL(htc_fpga_bb_readb);

void htc_fpga_bb_writeb(u8 value, u32 offset) {
	io_pr_info("FPGA BB: Write byte 0x%02x = 0x%02x\n", offset, value);
//	if(offset == HTC_FPGA_UART_IRQ_MASK)
//		value = 0;
#ifndef FPGA_DISABLE_IO_BB
	if(fpga_bb)
		__raw_writeb(value, fpga_bb + offset);
#endif
}
EXPORT_SYMBOL(htc_fpga_bb_writeb);

u32 htc_fpga_bb_readl(u32 offset) {
	u32 res = 0;
	if(fpga_bb)
		res = __raw_readl(fpga_bb + offset);
	io_pr_info("FPGA BB: Read long 0x%02x = 0x%08x\n", offset, res);
	return res;
}
EXPORT_SYMBOL(htc_fpga_bb_readl);

void htc_fpga_bb_writel(u32 value, u32 offset) {
	io_pr_info("FPGA BB: Write long 0x%02x = 0x%08x\n", offset, value);
#ifndef FPGA_DISABLE_IO_BB
	if(fpga_bb)
		__raw_writel(value, fpga_bb + offset);
#endif
}
EXPORT_SYMBOL(htc_fpga_bb_writel);

#ifdef CONFIG_HTC_FPGA_IO
u8 htc_fpga_io_readb(u32 offset) {
	u8 res = 0;
	if(fpga_io)
		res = __raw_readb(fpga_io + offset);
	io_pr_info("FPGA IO: Read byte 0x%02x = 0x%02x\n", offset, res);
	return res;
}
EXPORT_SYMBOL(htc_fpga_io_readb);

void htc_fpga_io_writeb(u8 value, u32 offset) {
	io_pr_info("FPGA IO: Write byte 0x%02x = 0x%02x\n", offset, value);
#ifndef FPGA_DISABLE_IO_IO
	if(fpga_io)
		__raw_writeb(value, fpga_io + offset);
#endif
}
EXPORT_SYMBOL(htc_fpga_io_writeb);

u32 htc_fpga_io_readl(u32 offset) {
	u32 res = 0;
	if(fpga_io)
		res = __raw_readl(fpga_io + offset);
	io_pr_info("FPGA IO: Read long 0x%02x = 0x%08x\n", offset, res);
	return res;
}
EXPORT_SYMBOL(htc_fpga_io_readl);

void htc_fpga_io_writel(u32 value, u32 offset) {
	io_pr_info("FPGA IO: Write long 0x%02x = 0x%08x\n", offset, value);
#ifndef FPGA_DISABLE_IO_IO
	if(fpga_io)
		__raw_writel(value, fpga_io + offset);
#endif
}
EXPORT_SYMBOL(htc_fpga_io_writel);
#endif/*CONFIG_HTC_FPGA_IO*/

void htc_fpga_bb_writebit(int state, int bit, u32 offset) {
	u8 value;
	spin_lock(&fpga_bb_lock);
	value = htc_fpga_bb_readb(offset);
	if(state)
		value |= BIT(bit);
	else
		value &= ~BIT(bit);
	htc_fpga_bb_writeb(value, offset);
	spin_unlock(&fpga_bb_lock);
}
EXPORT_SYMBOL(htc_fpga_bb_writebit);

#ifdef CONFIG_HTC_FPGA_IO
void htc_fpga_io_writebit(int state, int bit, u32 offset) {
	u8 value;
	spin_lock(&fpga_io_lock);
	value = htc_fpga_io_readb(offset);
	if(state)
		value |= BIT(bit);
	else
		value &= ~BIT(bit);
	htc_fpga_io_writeb(value, offset);
	spin_unlock(&fpga_io_lock);
}
EXPORT_SYMBOL(htc_fpga_io_writebit);
#endif/*CONFIG_HTC_FPGA_IO*/

void htc_fpga_bb_lock(void) {
	spin_lock(&fpga_bb_lock);
}
EXPORT_SYMBOL(htc_fpga_bb_lock);
void htc_fpga_bb_unlock(void) {
	spin_unlock(&fpga_bb_lock);
}
EXPORT_SYMBOL(htc_fpga_bb_unlock);

#ifdef CONFIG_HTC_FPGA_IO
void htc_fpga_io_lock(void) {
	spin_lock(&fpga_io_lock);
}
EXPORT_SYMBOL(htc_fpga_io_lock);
void htc_fpga_io_unlock(void) {
	spin_unlock(&fpga_io_lock);
}
EXPORT_SYMBOL(htc_fpga_io_unlock);
#endif/*CONFIG_HTC_FPGA_IO*/

// TODO: inside the interrupt routines, rather than testing for -1 and stuff like that, use WARN_ON() like a proper kernel codemonkey

/**************** baseboard IO Interrupt stuff ********************/

static inline int fpga_bb_io_irq_real_to_virt(unsigned int irq) {
	return irq - HTC_FPGA_BB_IO_IRQ_BASE;
}

static inline int fpga_bb_io_irq_virt_to_real(unsigned int irq) {
	return HTC_FPGA_BB_IO_IRQ_BASE + irq;
}

static void fpga_bb_io_ack_irq(unsigned int irq) {
	int ind = fpga_bb_io_irq_real_to_virt(irq);
	irq_pr_info("HTC: ACKing baseboard IO FPGA interrupt %u\n", ind);
	htc_fpga_bb_writeb(BIT(ind), HTC_FPGA_IRQ_STATUS);
}

static u8 fpga_bb_io_irq_mask = 0;

static void fpga_bb_io_mask_irq(unsigned int irq) {
	int ind = fpga_bb_io_irq_real_to_virt(irq);
	irq_pr_info("HTC: Disabling baseboard IO FPGA interrupt %u\n", ind);
	htc_fpga_bb_writebit(0, ind, HTC_FPGA_IRQ_MASK);
	fpga_bb_io_irq_mask &= ~BIT(ind);
}

static void fpga_bb_io_unmask_irq(unsigned int irq) {
	int ind = fpga_bb_io_irq_real_to_virt(irq);
	irq_pr_info("HTC: Enabling baseboard IO FPGA interrupt %u\n", ind);
	htc_fpga_bb_writebit(1, ind, HTC_FPGA_IRQ_MASK);
	fpga_bb_io_irq_mask |= BIT(ind);
}

static int fpga_bb_io_irq_set_type(unsigned int irq, unsigned int flow_type) {
	int ind = fpga_bb_io_irq_real_to_virt(irq);
	int off;
	uint8_t v;
	irq_pr_info("HTC: Set baseboard IO FPGA interrupt %u to type 0x%02x\n", ind, flow_type);
/*	if(irq == HTC_FPGA_IRQ_UART_BB) {
		irq_pr_info("HTC: Cannot set UART interrupt type\n");
		return 0;
	}*/
	if(irq == HTC_FPGA_IRQ_PTT_BB) {
		v = (flow_type & 0x3) << HTC_AUDIO_IN_DRIVER_PTT_RISING;
		htc_fpga_bb_writeb(v, HTC_FPGA_AUDIO_IN);
		return 0;
	}
	off = ind << 1;
	htc_fpga_bb_lock();
	v = htc_fpga_bb_readb(HTC_FPGA_IRQ_TYPE);
	v &= ~(0x3 << off);
	v |= (flow_type & 0x3) << off;
	htc_fpga_bb_writeb(v, HTC_FPGA_IRQ_TYPE);
	htc_fpga_bb_unlock();
	return 0;
}

static struct irq_chip fpga_bb_io_irq_chip = {
	.name		= "FPGA_BB_IO",
	.ack		= fpga_bb_io_ack_irq,
	.mask		= fpga_bb_io_mask_irq,
	.unmask		= fpga_bb_io_unmask_irq,
	.set_type	= fpga_bb_io_irq_set_type,
};

static irqreturn_t htc_fpga_bb_io_irq_handler(int irq, void *dev_id) {
	u8 status;
	int n, res;

	disable_irq_nosync(HTC_CPLD_IRQ_FPGA_BB_IO);
	while(1) {
		status = htc_fpga_bb_readb(HTC_FPGA_IRQ_STATUS);
		if(!status)
			break;
		status &= fpga_bb_io_irq_mask;

		// ack them
		htc_fpga_bb_writeb(status, HTC_FPGA_IRQ_STATUS);

		n = HTC_FPGA_BB_IO_IRQ_BASE;
		while(status) {
			res = ffs(status);
			n += res;
			irq_pr_info("HTC: Trigger baseboard IO FPGA interrupt %u (real IRQ %d)\n", fpga_bb_irq_virt_to_real(n - 1), n - 1);
			generic_handle_irq(n - 1);
			status >>= res;
		}
	}
	enable_irq(HTC_CPLD_IRQ_FPGA_BB_IO);

	return IRQ_HANDLED;
}

#ifdef CONFIG_HTC_FPGA_IO
/**************** IO board IO Interrupt stuff *********************/

static inline int fpga_io_io_irq_real_to_virt(unsigned int irq) {
	return irq - HTC_FPGA_IO_IO_IRQ_BASE;
}

static inline int fpga_io_io_irq_virt_to_real(unsigned int irq) {
	return HTC_FPGA_IO_IO_IRQ_BASE + irq;
}

static void fpga_io_io_ack_irq(unsigned int irq) {
	int ind = fpga_io_io_irq_real_to_virt(irq);
	irq_pr_info("HTC: ACKing IO board IO FPGA interrupt %u\n", ind);
	htc_fpga_io_writeb(BIT(ind), HTC_FPGA_IRQ_STATUS);
}

static void fpga_io_io_mask_irq(unsigned int irq) {
	int ind = fpga_io_io_irq_real_to_virt(irq);
	irq_pr_info("HTC: Disabling IO board IO FPGA interrupt %u\n", ind);
	htc_fpga_io_writebit(0, ind, HTC_FPGA_IRQ_MASK);
}

static void fpga_io_io_unmask_irq(unsigned int irq) {
	int ind = fpga_io_io_irq_real_to_virt(irq);
	irq_pr_info("HTC: Enabling IO board IO FPGA interrupt %u\n", ind);
	htc_fpga_io_writebit(1, ind, HTC_FPGA_IRQ_MASK);
}

static int fpga_io_io_irq_set_type(unsigned int irq, unsigned int flow_type) {
	int ind = fpga_io_io_irq_real_to_virt(irq);
	int off;
	uint8_t v;
	irq_pr_info("HTC: Set IO board IO FPGA interrupt %u to type 0x%02x\n", ind, flow_type);
/*	if(irq == HTC_FPGA_IRQ_UART_IO) {
		irq_pr_info("HTC: Cannot set UART interrupt type\n");
		return 0;
	}*/
#if 0
// TODO: this will possibly become the power sense interrupt edge
	if(irq == HTC_FPGA_IRQ_PTT_BB) {
		v = (flow_type & 0x3) << HTC_AUDIO_IN_DRIVER_PTT_RISING;
		htc_fpga_io_writeb(v, HTC_FPGA_AUDIO_IN);
		return 0;
	}
#endif
	off = ind << 1;
	htc_fpga_io_lock();
	v = htc_fpga_io_readb(HTC_FPGA_IRQ_TYPE);
	v &= ~(0x3 << off);
	v |= (flow_type & 0x3) << off;
	htc_fpga_io_writeb(v, HTC_FPGA_IRQ_TYPE);
	htc_fpga_io_unlock();
	return 0;
}

static struct irq_chip fpga_io_io_irq_chip = {
	.name		= "FPGA_IO_IO",
	.ack		= fpga_io_io_ack_irq,
	.mask		= fpga_io_io_mask_irq,
	.unmask		= fpga_io_io_unmask_irq,
	.set_type	= fpga_io_io_irq_set_type,
};

static irqreturn_t htc_fpga_io_io_irq_handler(int irq, void *dev_id) {
	u8 status = htc_fpga_io_readb(HTC_FPGA_IRQ_STATUS);
	int n = 0;
	//struct irq_desc *desc;

	irq_pr_info("htc_fpga_io_io_irq_handler(%d, %p) - status: 0x%02x\n", irq, dev_id, status);
	/*
		Here we check against the various irq types and the status and change.
		This naive looping implementation will have to do for the moment... later it'd be nice to do a quicker
		version using ffs() and some clever boolean operations that include values calculated when setting
		irq types. So here lies a big TODO :-)
	*/
	for(n=0; n<NR_HTC_FPGA_IO_IO_IRQS; n++) {
		if(status & 1) {
#if 0
			pr_info("HTC: Trigger IO board IO FPGA interrupt %u (real IRQ %d) (irq_desc=%p)\n", n, fpga_io_io_irq_virt_to_real(n), desc);
			desc = irq_to_desc(fpga_io_io_irq_virt_to_real(n));
			if(desc == NULL) {
				pr_warning("Ignoring IO board IO FPGA interrupt %u! No desc!\n", n);
			} else {
				if(desc->handle_irq == NULL)
					pr_warning("Ignoring IO board IO FPGA interrupt %u! No handle_irq()!\n", n);
				else
					generic_handle_irq(fpga_io_io_irq_virt_to_real(n));
			}
#else
			irq_pr_info("HTC: Trigger IO board IO FPGA interrupt %u (real IRQ %d)\n", n, fpga_io_irq_virt_to_real(n));
			generic_handle_irq(fpga_io_io_irq_virt_to_real(n));
#endif
		}
		status >>= 1;
	}

	return IRQ_HANDLED;
}
#endif/*CONFIG_HTC_FPGA_IO*/

/**************** baseboard UART Interrupt stuff ********************/

static inline int fpga_bb_uart_irq_real_to_virt(unsigned int irq) {
	return irq - HTC_FPGA_BB_UART_IRQ_BASE;
}

static inline int fpga_bb_uart_irq_virt_to_real(unsigned int irq) {
	return HTC_FPGA_BB_UART_IRQ_BASE + irq;
}

static u8 fpga_bb_uart_irq_mask = 0;

static void fpga_bb_uart_mask_irq(unsigned int irq) {
	int ind = fpga_bb_uart_irq_real_to_virt(irq);
	irq_pr_info("HTC: Disabling baseboard UART FPGA interrupt %u\n", ind);
	htc_fpga_bb_writebit(0, ind, HTC_FPGA_UART_IRQ_MASK);
	fpga_bb_uart_irq_mask &= ~BIT(ind);
}

static void fpga_bb_uart_unmask_irq(unsigned int irq) {
	int ind = fpga_bb_uart_irq_real_to_virt(irq);
	irq_pr_info("HTC: Enabling baseboard UART FPGA interrupt %u\n", ind);
	htc_fpga_bb_writebit(1, ind, HTC_FPGA_UART_IRQ_MASK);
	fpga_bb_uart_irq_mask |= BIT(ind);
}

static struct irq_chip fpga_bb_uart_irq_chip = {
	.name		= "FPGA_BB_UART",
	.mask		= fpga_bb_uart_mask_irq,
	.unmask		= fpga_bb_uart_unmask_irq,
};

static irqreturn_t htc_fpga_bb_uart_irq_handler(int irq, void *dev_id) {
	u8 status;
	int n, res;

	disable_irq_nosync(HTC_CPLD_IRQ_FPGA_BB_UART);
	while(1) {
		status = htc_fpga_bb_readb(HTC_FPGA_UART_IRQ_STATUS);
		if(!status)
			break;
		status &= fpga_bb_uart_irq_mask;

		// ack them
		htc_fpga_bb_writeb(status, HTC_FPGA_UART_IRQ_STATUS);

		n = HTC_FPGA_BB_UART_IRQ_BASE;
		while(status) {
			res = ffs(status);
			n += res;
			irq_pr_info("HTC: Trigger baseboard UART FPGA interrupt %u (real IRQ %d)\n", fpga_bb_irq_virt_to_real(n - 1), n - 1);
			generic_handle_irq(n - 1);
			status >>= res;
		}
	}
	enable_irq(HTC_CPLD_IRQ_FPGA_BB_UART);

	return IRQ_HANDLED;
}

#ifdef CONFIG_HTC_FPGA_IO
/**************** IO board UART Interrupt stuff ********************/

static inline int fpga_io_uart_irq_real_to_virt(unsigned int irq) {
	return irq - HTC_FPGA_IO_UART_IRQ_BASE;
}

static inline int fpga_io_uart_irq_virt_to_real(unsigned int irq) {
	return HTC_FPGA_IO_UART_IRQ_BASE + irq;
}

static u8 fpga_io_uart_irq_mask = 0;

static void fpga_io_uart_mask_irq(unsigned int irq) {
	int ind = fpga_io_uart_irq_real_to_virt(irq);
	irq_pr_info("HTC: Disabling IO board UART FPGA interrupt %u\n", ind);
	htc_fpga_io_writebit(0, ind, HTC_FPGA_UART_IRQ_MASK);
	fpga_io_uart_irq_mask &= ~BIT(ind);
}

static void fpga_io_uart_unmask_irq(unsigned int irq) {
	int ind = fpga_io_uart_irq_real_to_virt(irq);
	irq_pr_info("HTC: Enabling IO board UART FPGA interrupt %u\n", ind);
	htc_fpga_io_writebit(1, ind, HTC_FPGA_UART_IRQ_MASK);
	fpga_io_uart_irq_mask |= BIT(ind);
}

static struct irq_chip fpga_io_uart_irq_chip = {
	.name		= "FPGA_IO_UART",
	.mask		= fpga_io_uart_mask_irq,
	.unmask		= fpga_io_uart_unmask_irq,
};

static irqreturn_t htc_fpga_io_uart_irq_handler(int irq, void *dev_id) {
	u8 status;
	int n, res;

	disable_irq_nosync(HTC_CPLD_IRQ_FPGA_IO_UART);
	while(1) {
		status = htc_fpga_io_readb(HTC_FPGA_UART_IRQ_STATUS);
		if(!status)
			break;
		status &= fpga_io_uart_irq_mask;

		// ack them
		htc_fpga_io_writeb(status, HTC_FPGA_UART_IRQ_STATUS);

		n = HTC_FPGA_IO_UART_IRQ_BASE;
		while(status) {
			res = ffs(status);
			n += res;
			irq_pr_info("HTC: Trigger IO board UART FPGA interrupt %u (real IRQ %d)\n", fpga_io_irq_virt_to_real(n - 1), n - 1);
			generic_handle_irq(n - 1);
			status >>= res;
		}
	}
	enable_irq(HTC_CPLD_IRQ_FPGA_IO_UART);

	return IRQ_HANDLED;
}
#endif/*CONFIG_HTC_FPGA_IO*/

int htc_init_fpga_irq(void) {
	int i, virq;

	pr_info("HTC: Configuring baseboard IO FPGA IRQ\n");
	for(i=0; i<NR_HTC_FPGA_BB_IO_IRQS; i++) {
		virq = HTC_FPGA_BB_IO_IRQ_BASE + i;
		pr_info("FPGA baseboard virtual IO IRQ %d is mapped to real IRQ %d\n", i, virq);
		if(set_irq_chip(virq, &fpga_bb_io_irq_chip) < 0)
			pr_err("HTC: Failed to set virtual IRQ chip\n");
		set_irq_flags(virq, IRQF_VALID | IRQF_PROBE);
		set_irq_handler(virq, handle_edge_irq);			// except the UART IRQ
	}

#ifdef CONFIG_HTC_FPGA_IO
	// unfortunately we don't know yet whether we actually have an IO board
	pr_info("HTC: Configuring IO board FPGA IRQ\n");
	for(i=0; i<NR_HTC_FPGA_IO_IO_IRQS; i++) {
		virq = HTC_FPGA_IO_IO_IRQ_BASE + i;
		pr_info("FPGA IO board virtual IO IRQ %d is mapped to real IRQ %d\n", i, virq);
		if(set_irq_chip(virq, &fpga_io_io_irq_chip) < 0)
			pr_err("HTC: Failed to set virtual IO IRQ chip\n");
		set_irq_flags(virq, IRQF_VALID | IRQF_PROBE);
		set_irq_handler(virq, handle_edge_irq);
	}
#endif

	pr_info("HTC: Configuring baseboard UART FPGA IRQ\n");
	for(i=0; i<NR_HTC_FPGA_BB_UART_IRQS; i++) {
		virq = HTC_FPGA_BB_UART_IRQ_BASE + i;
		pr_info("FPGA baseboard virtual UART IRQ %d is mapped to real IRQ %d\n", i, virq);
		if(set_irq_chip(virq, &fpga_bb_uart_irq_chip) < 0)
			pr_err("HTC: Failed to set virtual IRQ chip\n");
		set_irq_flags(virq, IRQF_VALID | IRQF_PROBE);
		set_irq_handler(virq, handle_simple_irq);
	}

#ifdef CONFIG_HTC_FPGA_IO
	pr_info("HTC: Configuring IO board UART FPGA IRQ\n");
	for(i=0; i<NR_HTC_FPGA_IO_UART_IRQS; i++) {
		virq = HTC_FPGA_IO_UART_IRQ_BASE + i;
		pr_info("FPGA IO board virtual UART IRQ %d is mapped to real IRQ %d\n", i, virq);
		if(set_irq_chip(virq, &fpga_io_uart_irq_chip) < 0)
			pr_err("HTC: Failed to set virtual IRQ chip\n");
		set_irq_flags(virq, IRQF_VALID | IRQF_PROBE);
		set_irq_handler(virq, handle_simple_irq);
	}
#endif

	return 0;
}

static void htc_init_fpga_irq_handler(void) {
	pr_info("HTC: Configuring FPGA interrupt\n");

#ifndef DISABLE_IO_IRQS
	if(request_irq(HTC_CPLD_IRQ_FPGA_BB_IO, htc_fpga_bb_io_irq_handler, 0, "FPGA_BB_IO", htc_fpga_bb_io_irq_handler) < 0) {
		pr_err("HTC: Unable to register IO IRQ handler for baseboard FPGA\n");
		return;
	}
	set_irq_type(HTC_CPLD_IRQ_FPGA_BB_IO, IRQ_TYPE_LEVEL_HIGH);

#ifdef CONFIG_HTC_FPGA_IO
	if(htc_fpga_io_is_configured()) {
		if(request_irq(HTC_CPLD_IRQ_FPGA_IO_IO, htc_fpga_io_io_irq_handler, 0, "FPGA_IO_IO", htc_fpga_io_io_irq_handler) < 0) {
			pr_err("HTC: Unable to register IO IRQ handler for IO board FPGA\n");
			return;
		}
		set_irq_type(HTC_CPLD_IRQ_FPGA_IO_IO, IRQ_TYPE_LEVEL_HIGH);
	}
#endif
#endif

	if(request_irq(HTC_CPLD_IRQ_FPGA_BB_UART, htc_fpga_bb_uart_irq_handler, 0, "FPGA_BB_UART", htc_fpga_bb_uart_irq_handler) < 0) {
		pr_err("HTC: Unable to register UART IRQ handler for baseboard FPGA\n");
		return;
	}
	set_irq_type(HTC_CPLD_IRQ_FPGA_BB_UART, IRQ_TYPE_LEVEL_HIGH);

#ifdef CONFIG_HTC_FPGA_IO
	if(htc_fpga_io_is_configured()) {
		if(request_irq(HTC_CPLD_IRQ_FPGA_IO_UART, htc_fpga_io_uart_irq_handler, 0, "FPGA_IO_UART", htc_fpga_io_uart_irq_handler) < 0) {
			pr_err("HTC: Unable to register UART IRQ handler for IO board FPGA\n");
			return;
		}
		set_irq_type(HTC_CPLD_IRQ_FPGA_IO_UART, IRQ_TYPE_LEVEL_HIGH);
	}
#endif
}


static struct plat_serial8250_port htc_fpga_serial_platform_data[NR_HTC_FPGA_UARTS + 1];

static struct platform_device htc_fpga_serial_device = {
	.name			= "serial8250",
	.id				= PLAT8250_DEV_PLATFORM1,
	.dev			= {
		.platform_data	= htc_fpga_serial_platform_data,
	},
};

static void configure_fpga_uarts(void) {
	int i;
	struct plat_serial8250_port *port = htc_fpga_serial_platform_data;
	for(i=0; i<NR_HTC_FPGA_BB_UARTS; i++, port++) {
		//port->mapbase	= HTC_FPGA_BB_UART_BASE + (8 * i);
		port->mapbase	= HTC_FPGA_BB_UART_BASE + (16 * i);		// extra room for my custom debug registers
		port->irq		= HTC_FPGA_BB_UART_IRQ_BASE + i;
		port->uartclk	= 18432000;
		port->flags		= UPF_BOOT_AUTOCONF | UPF_FIXED_TYPE | UPF_IOREMAP | UPF_SKIP_TEST | UPF_BUGGY_UART;
		port->iotype	= UPIO_MEM;
		port->regshift	= 0;
		port->type		= PORT_16550; //PORT_16550A;
	}

#ifdef CONFIG_HTC_FPGA_IO
	if(htc_fpga_io_is_configured()) {
#ifdef SUPPORT_OLD_IOBOARD
		for(i=0; i<NR_HTC_FPGA_IO_UARTS + (old_ioboard ? 1 : 0); i++, port++) {
#else
		for(i=0; i<NR_HTC_FPGA_IO_UARTS; i++, port++) {
#endif
			//port->mapbase	= HTC_FPGA_IO_UART_BASE + (8 * i);
			port->mapbase	= HTC_FPGA_IO_UART_BASE + (16 * i);		// extra room for my custom debug registers
			port->irq		= HTC_FPGA_IO_UART_IRQ_BASE + i;
			port->uartclk	= 18432000;
			port->flags		= UPF_BOOT_AUTOCONF | UPF_FIXED_TYPE | UPF_IOREMAP | UPF_SKIP_TEST | UPF_BUGGY_UART /*| UPF_SHARE_IRQ*/;
			port->iotype	= UPIO_MEM;
			port->regshift	= 0;
			port->type		= PORT_16550; //PORT_16550A;
		}
	}
#endif
	// this will now be pointing at the terminating definition
	port->flags		= 0;

	pr_info("HTC: Registering FPGA UARTs\n");
	platform_device_register(&htc_fpga_serial_device);
};

static u8 cfg_ctrl_shadow = 0;

static inline void cfg_writebit(int state, int bit) {
	if(state)
		cfg_ctrl_shadow |= BIT(bit);
	else
		cfg_ctrl_shadow &= ~BIT(bit);
	htc_cpld_writeb(cfg_ctrl_shadow, HTC_CPLD_CFG_CTRL);
	//htc_cpld_writebit(state, bit, HTC_CPLD_CFG_CTRL);
}
static inline int cfg_readbit(int bit) {
	return htc_cpld_readbit(bit, HTC_CPLD_CFG_CTRL);
}

extern void cpld_iodebug_enable(void);
extern void cpld_iodebug_disable(void);

int htc_configure_fpga(const u8 *data, size_t length) {
	int bit, retry;
	u8 byte;

	cpld_iodebug_disable();

	cfg_ctrl_shadow = htc_cpld_readb(HTC_CPLD_CFG_CTRL);

	cfg_writebit(HTC_CFG_STATUS_N_DIR_INPUT, HTC_CFG_STATUS_N_DIR);

	cfg_writebit(0, HTC_CFG_SCLK);

	cfg_writebit(0, HTC_CFG_CONFIG_N);
	udelay(100);
	cfg_writebit(1, HTC_CFG_CONFIG_N);

	retry = 0;
	while(cfg_readbit(HTC_CFG_STATUS_N_IN)) {
		if(++retry > 100) {
			pr_err("HTC: STATUS_N not released at start of FPGA configuration\n");
			return -1;
		}
		udelay(100);
	}

	udelay(100);
	if(cfg_readbit(HTC_CFG_DONE)) {
		pr_err("HTC error: DONE high at start of FPGA configuration\n");
		return -1;
	}

	while(length-- > 0) {
		byte = *data++;
		for(bit=0; bit<8; bit++) {
			cfg_writebit(byte & 1, HTC_CFG_SDATA);
			cfg_writebit(1, HTC_CFG_SCLK);
			cfg_writebit(0, HTC_CFG_SCLK);
			byte >>= 1;
		}
	}
	udelay(100);

	if(!cfg_readbit(HTC_CFG_STATUS_N_IN)) {
		pr_err("HTC error: STATUS_N not high after FPGA configuration\n");
		return -1;
	}

	if(!cfg_readbit(HTC_CFG_DONE)) {
		pr_err("HTC error: DONE not high after FPGA configuration\n");
		return -1;
	}

	htc_cpld_lock();
	byte = htc_cpld_readb(HTC_CPLD_RESET_CTRL);
	byte &= ~(BIT(HTC_RESET_FPGA_BB) | BIT(HTC_RESET_FPGA_IO));
	htc_cpld_writeb(byte, HTC_CPLD_RESET_CTRL);
	htc_cpld_unlock();

	udelay(100);

	htc_cpld_lock();
	byte = htc_cpld_readb(HTC_CPLD_RESET_CTRL);
	byte |= BIT(HTC_RESET_FPGA_BB) | BIT(HTC_RESET_FPGA_IO);
	htc_cpld_writeb(byte, HTC_CPLD_RESET_CTRL);
	htc_cpld_unlock();

	pr_info("HTC: FPGA configuration complete\n");
	cpld_iodebug_enable();
	return 0;
}
EXPORT_SYMBOL(htc_configure_fpga);

__init int htc_init_fpga() {
	size_t cfg_sz;

	cfg_sz = FPGA_BB_CFG_LEN;
	if(htc_ioboard_present()) {
#ifdef CONFIG_HTC_FPGA_IO
		cfg_sz += FPGA_IO_CFG_LEN;
#else
		pr_warning("Warning: Kernel not configured for HTC IO board support\n");
#endif
	}
	pr_info("HTC: Loading default FPGA configuration\n");
	if(htc_configure_fpga(default_config, cfg_sz) != 0) {
		pr_err("HTC: Error configuring FPGA\n");
		return -1;
	}

	if(request_mem_region(HTC_FPGA_BB_IO_BASE, HTC_FPGA_BB_IO_SIZE, "fpga_bb") == NULL)
		goto fail_bb;
	fpga_bb = ioremap_nocache(HTC_FPGA_BB_IO_BASE, HTC_FPGA_BB_IO_SIZE);
	if(!fpga_bb) {
		release_mem_region(HTC_FPGA_BB_IO_BASE, HTC_FPGA_BB_IO_SIZE);
fail_bb:
		pr_err("ERROR: can't map baseboard FPGA\n");
	}

#ifdef CONFIG_HTC_FPGA_IO
	if(htc_ioboard_present()) {
		if(request_mem_region(HTC_FPGA_IO_IO_BASE, HTC_FPGA_IO_IO_SIZE, "fpga_io") == NULL)
			goto fail_io;
		fpga_io = ioremap_nocache(HTC_FPGA_IO_IO_BASE, HTC_FPGA_IO_IO_SIZE);
		if(!fpga_io) {
			release_mem_region(HTC_FPGA_IO_IO_BASE, HTC_FPGA_IO_IO_SIZE);
fail_io:
			pr_err("ERROR: can't map IO board FPGA\n");
		}
	}
#endif

	configure_fpga_uarts();

	htc_init_fpga_irq_handler();
	return 0;
}

int htc_fpga_bb_is_configured(void) {
	return fpga_bb != NULL;
}
EXPORT_SYMBOL(htc_fpga_bb_is_configured);

int htc_fpga_io_is_configured(void) {
	return fpga_io != NULL;
}
EXPORT_SYMBOL(htc_fpga_io_is_configured);

