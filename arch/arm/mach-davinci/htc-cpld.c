/*
 * Configure Hanover HTC CPLD and provide access to registers
 *
 * Author: David Steinberg, Hanover Displays Ltd <dsteinberg@hanoverdisplays.com>
 * Copyright (C)2010 Hanover Displays Ltd
 *
 * Changlog:
 *
 * 18 May 2010		DJS		Initial file
 * 02/08/2011		DJS		New version for new kernel
 *
 */

#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/htc_cpld.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <mach/gpio.h>
#include <linux/clk.h>
#include <mach/mux.h>
#include <linux/timer.h>
#include <mach/dm365.h>
#include "clock.h"

//#define CPLD_DEBUG_IRQ
//#define CPLD_DEBUG_IO

#define DISABLE_IO_IRQS

#ifdef CPLD_DEBUG_IRQ
#define irq_pr_info		pr_alert
#else
#define irq_pr_info(x...)	do { } while(0)
#endif
#ifdef CPLD_DEBUG_IO
static int cpld_debug_io = 1;
void cpld_iodebug_enable(void) { cpld_debug_io = 1; }
void cpld_iodebug_disable(void) { cpld_debug_io = 0; }
#define io_pr_info(x,y...)	do { if(cpld_debug_io) pr_info(x, ##y); } while(0)
#else
void cpld_iodebug_enable(void) { }
void cpld_iodebug_disable(void) { }
#define io_pr_info(x...)	do { } while(0)
#endif

#define DM365_ASYNC_EMIF_CONTROL_BASE 0x01d10000

#define CPLD_CLOCK_NAME			"clkout1"
#define HTC_CPLD_IRQ_GPIO		GPIO(0)

static int have_audioboard = 0, have_ioboard = 0;

static void __iomem *cpld = NULL;

static u8 cpld_ver = 0;

static int cpld_irq_types[8];

static DEFINE_SPINLOCK(cpld_lock);

u8 htc_cpld_readb(u32 offset) {
	u8 res = __raw_readb(cpld + offset);
	io_pr_info("CPLD: Read byte 0x%02x = 0x%02x\n", offset, res);
	return res;
}
EXPORT_SYMBOL(htc_cpld_readb);

void htc_cpld_writeb(u8 value, u32 offset) {
	io_pr_info("CPLD: Write byte 0x%02x = 0x%02x\n", offset, value);
//#ifdef DISABLE_IO_IRQS
#if 0
	if(offset == HTC_CPLD_IRQ_MASK)
		value &= ~(BIT(HTC_IRQ_FPGA_BB_IO) | BIT(HTC_IRQ_FPGA_IO_IO));
//		value &= ~(HTC_IRQ_FPGA_BB0 | HTC_IRQ_FPGA_BB1 | HTC_IRQ_FPGA_IO0 | HTC_IRQ_FPGA_IO1);
#endif
	__raw_writeb(value, cpld + offset);
}
EXPORT_SYMBOL(htc_cpld_writeb);

void htc_cpld_writebit(int state, int bit, u32 offset) {
	u8 value;
#ifdef CPLD_DEBUG_IO
	int orig_cpld_debug_io;
	io_pr_info("CPLD: Write bit 0x%02x : %u = %s\n", offset, bit, state ? "true" : "false");
#endif
	spin_lock(&cpld_lock);
#ifdef CPLD_DEBUG_IO
	orig_cpld_debug_io = cpld_debug_io;
	cpld_debug_io = 0;
#endif
	value = htc_cpld_readb(offset);
//	if(offset == HTC_CPLD_RESET_CTRL)
//		pr_info("htc_cpld_writebit: state=%u , bit=%u. read 0x%02x\n", state, bit, value);
	if(state)
		value |= BIT(bit);
	else
		value &= ~BIT(bit);
//	if(offset == HTC_CPLD_RESET_CTRL)
//		pr_info("htc_cpld_writebit: setting 0x%02x\n", value);
	htc_cpld_writeb(value, offset);
#ifdef CPLD_DEBUG_IO
	cpld_debug_io = orig_cpld_debug_io;
#endif
	spin_unlock(&cpld_lock);
}
EXPORT_SYMBOL(htc_cpld_writebit);

void htc_cpld_lock(void) {
	spin_lock(&cpld_lock);
}
EXPORT_SYMBOL(htc_cpld_lock);
void htc_cpld_unlock(void) {
	spin_unlock(&cpld_lock);
}
EXPORT_SYMBOL(htc_cpld_unlock);

static inline int cpld_irq_real_to_virt(unsigned int irq) {
	return irq - HTC_CPLD_IRQ_BASE;
}

static inline int cpld_irq_virt_to_real(unsigned int irq) {
	return HTC_CPLD_IRQ_BASE + irq;
}

static u8 cpld_irq_mask = 0;

static void cpld_mask_irq(unsigned int irq) {
	int ind = cpld_irq_real_to_virt(irq);
	irq_pr_info("HTC: Disabling CPLD interrupt %u\n", ind);
	htc_cpld_writebit(0, ind, HTC_CPLD_IRQ_MASK);
	cpld_irq_mask &= ~BIT(ind);
}

static void cpld_unmask_irq(unsigned int irq) {
	int ind = cpld_irq_real_to_virt(irq);
	irq_pr_info("HTC: Enabling CPLD interrupt %u\n", ind);
#ifdef DISABLE_IO_IRQS
	if((ind == HTC_IRQ_FPGA_BB_IO) || (ind == HTC_IRQ_FPGA_IO_IO))
		return;
#endif
	htc_cpld_writebit(1, ind, HTC_CPLD_IRQ_MASK);
	cpld_irq_mask |= BIT(ind);
}

static int cpld_irq_set_type(unsigned int irq, unsigned int flow_type) {
	int ind = cpld_irq_real_to_virt(irq);
	irq_pr_info("HTC: Set CPLD interrupt %u to type 0x%02x\n", ind, flow_type);
	cpld_irq_types[ind] = flow_type;

	return 0;
}

static struct irq_chip cpld_irq_chip = {
	.name		= "CPLD",
	.mask		= cpld_mask_irq,
	.unmask		= cpld_unmask_irq,
	.set_type	= cpld_irq_set_type,
};

static irqreturn_t htc_cpld_irq_handler(int irq, void *dev_id) {
	u8 status;
	int n, res;
	
	htc_cpld_writeb(0, HTC_CPLD_IRQ_MASK);
	status = htc_cpld_readb(HTC_CPLD_IRQ_STATUS);
	status &= cpld_irq_mask;

	n = HTC_CPLD_IRQ_BASE;
	while(status) {
		res = ffs(status);
		n += res;
		irq_pr_info("HTC: Trigger CPLD interrupt %u (real IRQ %d)\n", cpld_irq_real_to_virt(n - 1), n - 1);
		generic_handle_irq(n - 1);
		status >>= res;
	}

	htc_cpld_writeb(cpld_irq_mask, HTC_CPLD_IRQ_MASK);
	status = htc_cpld_readb(HTC_CPLD_IRQ_STATUS);
	return IRQ_HANDLED;
}

int htc_init_cpld_irq(void) {
	int i, virq;

	pr_info("HTC: Configuring CPLD IRQ\n");

	// NR_IRQS is defined for DaVinci as being the amount needed for the DA850,
	// which seems to be a lot more than needed for the DM365. We should be able to
	// squeeze ours on the the end!
	for(i=0; i<NR_HTC_CPLD_IRQS; i++) {
		virq = HTC_CPLD_IRQ_BASE + i;
		pr_info("CPLD virtual IRQ %d is mapped to real IRQ %d\n", i, virq);
		if(set_irq_chip(virq, &cpld_irq_chip) < 0)
			pr_err("HTC: Failed to set virtual IRQ chip\n");
		cpld_irq_types[i] = IRQ_TYPE_NONE;
		set_irq_flags(virq, IRQF_VALID | IRQF_PROBE);
		set_irq_handler(virq, handle_level_irq);
	}

	return 0;
}

static void htc_init_cpld_irq_pin(void) {
	pr_info("HTC: Configuring CPLD interrupt GPIO\n");

	if(gpio_request(HTC_CPLD_IRQ_GPIO, "CPLD IRQ") < 0) {
		pr_err("HTC: Unable to get CPLD IRQ pin\n");
		return;
	}
	if(gpio_direction_input(HTC_CPLD_IRQ_GPIO) < 0) {
		pr_err("HTC: Unable to set CPLD IRQ pin to input\n");
		return;
	}

	if(request_irq(gpio_to_irq(HTC_CPLD_IRQ_GPIO), htc_cpld_irq_handler, 0, "CPLD", htc_cpld_irq_handler) < 0) {
		pr_err("HTC: Unable to register IRQ handler for CPLD\n");
		return;
	}
	set_irq_type(gpio_to_irq(HTC_CPLD_IRQ_GPIO), IRQ_TYPE_EDGE_FALLING);
}

// is there a 'more correct' way to do this?
static int __init configure_A2CR(void) {
	static void __iomem *emif;
	u32 a2cr;

	if (request_mem_region(DM365_ASYNC_EMIF_CONTROL_BASE, SECTION_SIZE, "EMIF") == NULL) {
		pr_err("HTC: EMIF control request failed\n");
		return -1;
	}
	emif = ioremap(DM365_ASYNC_EMIF_CONTROL_BASE, SECTION_SIZE);
	if(!emif) {
		pr_err("HTC: EMIF control ioremap failed\n");
		release_mem_region(DM365_ASYNC_EMIF_CONTROL_BASE, SECTION_SIZE);
		return -1;
	}

	a2cr = __raw_readl(emif + 0x14);
	pr_info("A2CR startup: 0x%08x\n", a2cr);
	a2cr = 0;

	a2cr |= 0 << 26;		// write setup cycles
	a2cr |= 29 << 20;		// write strobe cycles
	a2cr |= 0 << 17;		// write hold cycles
	a2cr |= 0 << 13;		// read setup cycles
	a2cr |= 29 << 7;			// read strobe cycles
	a2cr |= 0 << 4;			// read hold cycles
	a2cr |= 1 << 2;			// minimum turn around time
	__raw_writel(a2cr, emif + 0x14);

	// we don't need this anymore
	release_mem_region(DM365_ASYNC_EMIF_CONTROL_BASE, SECTION_SIZE);
	return 0;
}

void __init htc_init_cpld(void)
{
	struct clk *cpld_clk;

	cpld_clk = clk_get(NULL, CPLD_CLOCK_NAME);
	if (cpld_clk == NULL || IS_ERR(cpld_clk)) {
		pr_err("ERROR: CPLD clock '%s' not registered!\n", CPLD_CLOCK_NAME);
		return;
	}
	clk_enable(cpld_clk);
	pr_info("CPLD clock running at %lu Hz\n", clk_get_rate(cpld_clk));
	gpio_request(35, "clkout1");
	gpio_direction_output(35, 1);
	davinci_cfg_reg(DM365_CLKOUT1);

	// TODO: what should these *really* be?
	__raw_writel(0x8000, cpld_clk->parent->pll_data->base + PLLDIV3 + 4);	// osc_div1
	__raw_writel(2, cpld_clk->parent->pll_data->base + PLLCKEN);	// cken

	// setup for CPLD memory access
	davinci_cfg_reg(DM365_AEMIF_CE0);
#if 1
	if(configure_A2CR() != 0) {
		pr_err("Error configuring 8-bit mode in A2CR\n");
		//return;
	}
#endif

	if(request_mem_region(HTC_CPLD_BASE, HTC_CPLD_SIZE,
			"cpld") == NULL)
		goto fail;
	cpld = ioremap_nocache(HTC_CPLD_BASE, HTC_CPLD_SIZE);
	if(!cpld) {
		release_mem_region(HTC_CPLD_BASE, HTC_CPLD_SIZE);
fail:
		pr_err("ERROR: can't map CPLD\n");
		clk_disable(cpld_clk);
		return;
	}

	cpld_ver = htc_cpld_readb(HTC_CPLD_VERSION);
	pr_info("HTC CPLD version V%u.%u\n", (cpld_ver >> 4) & 0xf, cpld_ver & 0xf);
	if((cpld_ver == 0x00) || (cpld_ver == 0xff))
		pr_warning("CPLD version read as 0x%02x. Its likely CPLD access failed!\n", cpld_ver);

	htc_init_cpld_irq_pin();

	htc_cpld_writebit(1, HTC_RESET_GPS, HTC_CPLD_RESET_CTRL);
//	htc_cpld_writebit(0, HTC_RESET_USB, HTC_CPLD_RESET_CTRL);	// hold USB in reset
	htc_cpld_writebit(1, HTC_RESET_USB, HTC_CPLD_RESET_CTRL);	// release USB from reset
	//htc_cpld_writebit(0, HTC_RESET_WIFI, HTC_CPLD_RESET_CTRL);
	//htc_cpld_writebit(1, HTC_RESET_EXT, HTC_CPLD_RESET_CTRL);
//	htc_cpld_writebit(0, HTC_RESET_ETH, HTC_CPLD_RESET_CTRL);	// this breaks things!
	//htc_cpld_writebit(1, HTC_RESET_VID_IF, HTC_CPLD_RESET_CTRL);	// release video interface from reset (this breaks things as well it seems)

#if 1
	// enable CPLD based watchdog SPI
	htc_cpld_writebit(0, HTC_WDOG_SCLK_TS, HTC_CPLD_WDOG_CTRL1);
	//htc_cpld_writebit(0, HTC_WDOG_SCLK_TS, HTC_CPLD_WDOG_CTRL1);		// TODO: this is wrong.. what should it be?
#else
	// disable CPLD SPI for CPU based watchdog SPI
	// now we do this...
	htc_cpld_writebit(1, HTC_WDOG_SCLK_TS, HTC_CPLD_WDOG_CTRL1);
	htc_cpld_writebit(1, HTC_WDOG_MOSI_TS, HTC_CPLD_WDOG_CTRL1);
#endif

	have_audioboard = !htc_cpld_readbit(HTC_GPIO_AUDIOBOARD_DET, HTC_CPLD_GPIO_CTRL2);
	pr_info("HTC: Audio board %sfound\n", have_audioboard ? "" : "not ");
	have_ioboard = !htc_cpld_readbit(HTC_GPIO_IOBOARD_DET, HTC_CPLD_GPIO_CTRL2);
	pr_info("HTC: IO board %sfound\n", have_ioboard ? "" : "not ");
}

int htc_cpld_is_configured(void) {
	return cpld != NULL;
}
EXPORT_SYMBOL(htc_cpld_is_configured);

int htc_cpld_version(void) {
	return cpld_ver;
}
EXPORT_SYMBOL(htc_cpld_version);

int htc_audioboard_present(void) {
	return have_audioboard;
}
EXPORT_SYMBOL(htc_audioboard_present);

int htc_ioboard_present(void) {
	return have_ioboard;
}
EXPORT_SYMBOL(htc_ioboard_present);

