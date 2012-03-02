/*
 * Hanover Displays HTC FPGA I/O
 *
 * Author: David Steinberg <dsteinberg@hanoverdisplays.com>
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <mach/htc_fpga.h>
#include <mach/htc_cpld.h>		// for the htc_xxx_present() functions
#include <linux/slab.h>

#if 0
#define gpio_pr_info		pr_info
#else
#define gpio_pr_info(x...)	do { } while(0);
#endif

// TODO: this should really come in as platform device data
struct fpga_gpio_gc_data {
	char *label;
	int base;
	int reg;
	int dir;
	int ngpio;
	char *names[8];
	int bits[8];
	int flags;
	int irqs[8];
	int naudio_ngpio;
};

typedef enum {
	gt_relay_bb,
	gt_opto_bb,
	gt_audio_out,
	gt_audio_in,
	gt_autorts,
#ifdef CONFIG_HTC_FPGA_IO
	gt_relay_io,
	gt_opto_io,
	gt_modem,
	gt_autorts_io,
#endif

	nr_gpio_types
} gpio_type;

struct fpga_gpio_gc {
	struct gpio_chip gc;
	struct fpga_gpio_gc_data *data;
};

struct fpga_gpio {
	struct device *dev;
	struct fpga_gpio_gc gc[nr_gpio_types];
};

// FPGA GPIO Flags
#define CGF_DIR_HARDCODED	BIT(0)		// if 1, .dir is direction mask, if 0, .dir is direction register
#define CGF_DIR_INVERT		BIT(1)		// applies to CGF_DIR_HARDCODED==0 only. if set, 1 means output, 0 means input
#define CGF_EXPORT			BIT(2)		// if 1, then export all IOs to sysfs
#define CGF_IOBOARD			BIT(3)		// if 1, FPGA is on IO board
#define CGF_INVERT			BIT(4)		// if 1, input and output state is inverted
#define CGF_IRQ				BIT(5)		// if 1, irqs specify IRQ numbers (or -1)
#define CGF_NEEDAUDIOBOARD	BIT(6)		// if 1, if no audio present, ngpio is naudio_ngpio

#define DIR_OUTPUT		0
#define DIR_INPUT		1

static struct fpga_gpio_gc_data gc_data[] = {
	{
		.label		= "htc_relay_bb",
		.base		= 152,
		.reg		= HTC_FPGA_MISC_OUT,
		.dir		= 0,
		.ngpio		= 4,
		.names		= {"relay1", "relay2",
					   "relay3", "relay4"},
		.bits		= {4, 5, 6, 7},
		.flags		= CGF_DIR_HARDCODED | CGF_EXPORT /*| CGF_INVERT*/,
	},
	{
		.label		= "htc_opto_bb",
		.base		= 156,
		.reg		= HTC_FPGA_MISC_IN,
		.dir		= 0xf,
		.ngpio		= 4,
		.names		= {"opto1", "opto2",
					   "opto3", "opto4"},
		.bits		= {0, 1, 2, 3},
		.flags		= CGF_DIR_HARDCODED | CGF_EXPORT | /*CGF_INVERT |*/ CGF_IRQ,
		.irqs		= {HTC_FPGA_IRQ_OPTO_1, HTC_FPGA_IRQ_OPTO_2,
					   HTC_FPGA_IRQ_OPTO_3, HTC_FPGA_IRQ_OPTO_4},
	},
	{
		.label		= "htc_audio_out",
		.base		= 160,
		.reg		= HTC_FPGA_AUDIO_OUT,
		.dir		= 0,
		.ngpio		= 6,
		.names		= {"mute_base", "power_base",
					   "mute_left", "mute_right",
					   "power_left", "power_right"},
		.bits		= {4, 5, 0, 1, 2, 3},
		.flags		= CGF_DIR_HARDCODED | CGF_EXPORT | CGF_NEEDAUDIOBOARD,
		.naudio_ngpio= 2,
	},
	{
		.label		= "htc_audio_in",
		.base		= 166,
		.reg		= HTC_FPGA_AUDIO_IN,
		.dir		= 0xff,
		.ngpio		= 4,
		.names		= {"audio_diag_base", "audio_diag_left",
					   "audio_diag_right", "driver_ptt"},
		.bits		= {HTC_AUDIO_IN_DIAG_BASE, HTC_AUDIO_IN_DIAG_LEFT,
					   HTC_AUDIO_IN_DIAG_RIGHT, HTC_AUDIO_IN_DRIVER_PTT},
		.flags		= CGF_DIR_HARDCODED | CGF_EXPORT | CGF_IRQ | CGF_NEEDAUDIOBOARD,
		.irqs		= {-1, -1, -1, HTC_FPGA_IRQ_PTT_BB},
		.naudio_ngpio= 1,
	},
	{
		.label		= "htc_uart_mode",
		.base		= 170,
		.reg		= HTC_FPGA_UART_MODE,
		.dir		= 0,
		.ngpio		= 3,
		.names		= {"autorts5", "autorts6", "autorts7"},
		.bits		= {5-1, 6-1, 7-1},		// bits start at 0, ports start at 1
		.flags		= CGF_DIR_HARDCODED | CGF_EXPORT,
	},
#ifdef CONFIG_HTC_FPGA_IO
	{
		.label		= "htc_relay_io",
		.base		= 180,
		.reg		= HTC_FPGA_MISC_OUT,
		.dir		= 0,
		.ngpio		= 4,
		.names		= {"relay5", "relay6",
					   "relay7", "relay8"},
		.bits		= {4, 5, 6, 7},
		.flags		= CGF_DIR_HARDCODED | CGF_EXPORT | /*CGF_INVERT |*/ CGF_IOBOARD,
	},
	{
		.label		= "htc_opto_io",
		.base		= 184,
		.reg		= HTC_FPGA_MISC_IN,
		.dir		= 0xf,
		.ngpio		= 4,
		.names		= {"opto5", "opto6",
					   "opto7", "opto8"},
		.bits		= {0, 1, 2, 3},
		.flags		= CGF_DIR_HARDCODED | CGF_EXPORT | /*CGF_INVERT |*/ CGF_IOBOARD | CGF_IRQ,
		.irqs		= {HTC_FPGA_IRQ_OPTO_5, HTC_FPGA_IRQ_OPTO_6,
					   HTC_FPGA_IRQ_OPTO_7, HTC_FPGA_IRQ_OPTO_8},
	},
	{
		.label		= "htc_modem",
		.base		= 188,
		.reg		= HTC_FPGA_MISC_OUT,
		.dir		= 0,
		.ngpio		= 2,
		.names		= {"modem_ign", "modem_emerg"},
		.bits		= {HTC_MISC_OUT_IGNITION, HTC_MISC_OUT_EMERG_OFF},
		.flags		= CGF_DIR_HARDCODED | CGF_EXPORT | CGF_IOBOARD,
	},
	{
		.label		= "htc_uart_mode_io",
		.base		= 190,
		.reg		= HTC_FPGA_UART_MODE,
		.dir		= 0,
		.ngpio		= 4,
#if 0	// old IO board
		.names		= {"autorts10", "autorts12", "autorts13", "autorts14"},
		.bits		= {1-1, 3-1, 4-1, 5-1},		// bits start at 0, ports start at 1	// I think this may be wrong
#elif 0	// new IO board (rev B), wrong order. Thanks Will!
		.names		= {"autorts9", "autorts10", "autorts11", "autorts13"},
		.bits		= {0, 1, 2, 4},
#else	// new IO board (rev B). remove others once verified
		.names		= {"autorts10", "autorts11", "autorts12", "autorts13"},
		.bits		= {1, 2, 3, 4},
#endif
		.flags		= CGF_DIR_HARDCODED | CGF_EXPORT | CGF_IOBOARD,

/*
	NOTE:
		I/O board ports are:
		oldttyS		bad_newttyS		newttyS		oldbit		bad_newbit		newbit		desc				bad_rxd		bad_txd		bad_rts		good_rxd	good_txd	good_rts
		9			12				9			0			3				0			modem port 1		91			93			92			144			143			142
		10			13				10			1			4				1			DE9					90			79			76			100			101			99
		12			9				11			3			0				2			plugin 1			75			74			73			91			93			92
		13			10				12			4			1				3			plugin 2			144			143			142			90?			79?			76
		14			11				13			5			2				4			plugin 3			100			101			99			75?			74?			73
		11			n/a				n/a			2			n/a				n/a			modem port 2		n/a			n/a
*/
	},
#endif
};

static int fpga_bb_gpio_dir_in(struct gpio_chip *gc, unsigned off)
{
	struct fpga_gpio_gc *gpio = container_of(gc, struct fpga_gpio_gc, gc);
	if(gpio->data->flags & CGF_DIR_HARDCODED) {
		if(!(gpio->data->dir & BIT(off))) {
			pr_err("FPGA GPIO: Error: %s.%s cannot be set to input\n", gpio->data->label, gpio->data->names[off]);
			return -EACCES;			// hardcoded to an output
		}
	} else {
		gpio_pr_info("FPGA GPIO: Setting to input %s.%s\n", gpio->data->label, gpio->data->names[off]);
		htc_fpga_bb_writebit(
			!(gpio->data->flags & CGF_DIR_INVERT),
			gpio->data->bits[off],
			gpio->data->dir
		);
	}
	return 0;
}

static int fpga_bb_gpio_dir_out(struct gpio_chip *gc, unsigned off, int val)
{
	struct fpga_gpio_gc *gpio = container_of(gc, struct fpga_gpio_gc, gc);

	if(gpio->data->flags & CGF_DIR_HARDCODED) {
		if(!!(gpio->data->dir & BIT(off))) {
			pr_err("FPGA GPIO: Error: %s.%s cannot be set to output\n", gpio->data->label, gpio->data->names[off]);
			return -EACCES;			// hardcoded to an input
		}
	} else {
		gpio_pr_info("FPGA GPIO: Setting to output %s.%s\n", gpio->data->label, gpio->data->names[off]);
		htc_fpga_bb_writebit(
			!!(gpio->data->flags & CGF_DIR_INVERT),
			gpio->data->bits[off],
			gpio->data->dir
		);
	}
	gpio_pr_info("FPGA GPIO: Setting state of %s.%s to %s\n", gpio->data->label, gpio->data->names[off], val ? "on" : "off");
	if(gpio->data->flags & CGF_INVERT)
		val = !val;
	htc_fpga_bb_writebit(
		val,
		gpio->data->bits[off],
		gpio->data->reg
	);
	return 0;
}

static int fpga_bb_gpio_get(struct gpio_chip *gc, unsigned off)
{
	struct fpga_gpio_gc *gpio = container_of(gc, struct fpga_gpio_gc, gc);
	gpio_pr_info("FPGA GPIO: Reading state of %s.%s\n", gpio->data->label, gpio->data->names[off]);

	if(gpio->data->flags & CGF_INVERT) {
		return !htc_fpga_bb_readbit(
			gpio->data->bits[off],
			gpio->data->reg
		);
	} else {
		return htc_fpga_bb_readbit(
			gpio->data->bits[off],
			gpio->data->reg
		);
	}
}

static void fpga_bb_gpio_set(struct gpio_chip *gc, unsigned off, int val)
{
	struct fpga_gpio_gc *gpio = container_of(gc, struct fpga_gpio_gc, gc);
	gpio_pr_info("FPGA GPIO: Setting state of %s.%s to %s\n", gpio->data->label, gpio->data->names[off], val ? "on" : "off");
	if(gpio->data->flags & CGF_INVERT)
		val = !val;
	htc_fpga_bb_writebit(
		val,
		gpio->data->bits[off],
		gpio->data->reg
	);
}

#ifdef CONFIG_HTC_FPGA_IO
static int fpga_io_gpio_dir_in(struct gpio_chip *gc, unsigned off)
{
	struct fpga_gpio_gc *gpio = container_of(gc, struct fpga_gpio_gc, gc);
	if(gpio->data->flags & CGF_DIR_HARDCODED) {
		if(!(gpio->data->dir & BIT(off))) {
			pr_err("FPGA GPIO: Error: %s.%s cannot be set to input\n", gpio->data->label, gpio->data->names[off]);
			return -EACCES;			// hardcoded to an output
		}
	} else {
		gpio_pr_info("FPGA GPIO: Setting to input %s.%s\n", gpio->data->label, gpio->data->names[off]);
		htc_fpga_io_writebit(
			!(gpio->data->flags & CGF_DIR_INVERT),
			gpio->data->bits[off],
			gpio->data->dir
		);
	}
	return 0;
}

static int fpga_io_gpio_dir_out(struct gpio_chip *gc, unsigned off, int val)
{
	struct fpga_gpio_gc *gpio = container_of(gc, struct fpga_gpio_gc, gc);

	if(gpio->data->flags & CGF_DIR_HARDCODED) {
		if(!!(gpio->data->dir & BIT(off))) {
			pr_err("FPGA GPIO: Error: %s.%s cannot be set to output\n", gpio->data->label, gpio->data->names[off]);
			return -EACCES;			// hardcoded to an input
		}
	} else {
		gpio_pr_info("FPGA GPIO: Setting to output %s.%s\n", gpio->data->label, gpio->data->names[off]);
		htc_fpga_io_writebit(
			!!(gpio->data->flags & CGF_DIR_INVERT),
			gpio->data->bits[off],
			gpio->data->dir
		);
	}

	gpio_pr_info("FPGA GPIO: Setting state of %s.%s to %s\n", gpio->data->label, gpio->data->names[off], val ? "on" : "off");
	if(gpio->data->flags & CGF_INVERT)
		val = !val;
	htc_fpga_io_writebit(
		val,
		gpio->data->bits[off],
		gpio->data->reg
	);
	return 0;
}

static int fpga_io_gpio_get(struct gpio_chip *gc, unsigned off)
{
	struct fpga_gpio_gc *gpio = container_of(gc, struct fpga_gpio_gc, gc);
	gpio_pr_info("FPGA GPIO: Reading state of %s.%s\n", gpio->data->label, gpio->data->names[off]);
	if(gpio->data->flags & CGF_INVERT) {
		return !htc_fpga_io_readbit(
			gpio->data->bits[off],
			gpio->data->reg
		);
	} else {
		return htc_fpga_io_readbit(
			gpio->data->bits[off],
			gpio->data->reg
		);
	}
}

static void fpga_io_gpio_set(struct gpio_chip *gc, unsigned off, int val)
{
	struct fpga_gpio_gc *gpio = container_of(gc, struct fpga_gpio_gc, gc);
	gpio_pr_info("FPGA GPIO: Setting state of %s.%s to %s\n", gpio->data->label, gpio->data->names[off], val ? "on" : "off");
	if(gpio->data->flags & CGF_INVERT)
		val = !val;
	htc_fpga_io_writebit(
		val,
		gpio->data->bits[off],
		gpio->data->reg
	);
}
#endif

static int fpga_gpio_dir_out_dummy(struct gpio_chip *gc, unsigned off, int val)
{
	struct fpga_gpio_gc *gpio = container_of(gc, struct fpga_gpio_gc, gc);
	gpio_pr_info("FPGA GPIO: Dummy out for %s.%s\n", gpio->data->label, gpio->data->names[off]);
	return 0;
}

static int fpga_gpio_to_irq(struct gpio_chip *gc, unsigned off) {
	struct fpga_gpio_gc *gpio = container_of(gc, struct fpga_gpio_gc, gc);
	gpio_pr_info("FPGA GPIO: Getting IRQ for %s.%s\n", gpio->data->label, gpio->data->names[off]);
	return gpio->data->irqs[off];
}

static int fpga_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fpga_gpio *gpio;
	int err = 0, i, j, ngpio;

	gpio = kzalloc(sizeof(struct fpga_gpio), GFP_KERNEL);
	if (gpio == NULL) {
		printk(KERN_ERR "kzalloc failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, gpio);
	gpio->dev = dev;

	for(i=0; i<nr_gpio_types; i++) {
#ifdef CONFIG_HTC_FPGA_IO
		if((gc_data[i].flags & CGF_IOBOARD) && !htc_ioboard_present())
			continue;
#endif

		ngpio = gc_data[i].ngpio;
		if(gc_data[i].flags & CGF_NEEDAUDIOBOARD) {
#ifdef CONFIG_HTC_AUDIOBOARD
			if(!htc_audioboard_present())
#endif
				ngpio = gc_data[i].naudio_ngpio;
		}

		gpio->gc[i].data = &gc_data[i];
		gpio->gc[i].gc.label = gc_data[i].label;
		gpio->gc[i].gc.base = gc_data[i].base;
		gpio->gc[i].gc.ngpio = ngpio;
		gpio->gc[i].gc.owner = THIS_MODULE;
		gpio->gc[i].gc.dev= dev;
		gpio->gc[i].gc.names = gc_data[i].names;

		if(gc_data[i].flags & CGF_IRQ) {
			gpio_pr_info("FPGA GPIO: Chip '%s' is IRQable\n", gc_data[i].label);
			gpio->gc[i].gc.to_irq = fpga_gpio_to_irq;
		}

#ifdef CONFIG_HTC_FPGA_IO
		if(!(gc_data[i].flags & CGF_IOBOARD)) {
#endif
			if(!(gc_data[i].flags & CGF_DIR_HARDCODED) || (gc_data[i].dir != 0))
				gpio->gc[i].gc.direction_input = fpga_bb_gpio_dir_in;
			else
				gpio_pr_info("FPGA GPIO: Not providing 'direction_input' for chip %s\n", gc_data[i].label);
			gpio->gc[i].gc.direction_output = fpga_bb_gpio_dir_out;
			gpio->gc[i].gc.get = fpga_bb_gpio_get;
			gpio->gc[i].gc.set = fpga_bb_gpio_set;
#ifdef CONFIG_HTC_FPGA_IO
		} else {
			if(!(gc_data[i].flags & CGF_DIR_HARDCODED) || (gc_data[i].dir != 0))
				gpio->gc[i].gc.direction_input = fpga_io_gpio_dir_in;
			else
				gpio_pr_info("FPGA GPIO: Not providing 'direction_input' for chip %s\n", gc_data[i].label);
			gpio->gc[i].gc.direction_output = fpga_io_gpio_dir_out;
			gpio->gc[i].gc.get = fpga_io_gpio_get;
			gpio->gc[i].gc.set = fpga_io_gpio_set;
		}
#endif
		gpio->gc[i].gc.can_sleep = 0;
		//gpio->gc[i].gc.exported = !!(gc_data[i].flags & CGF_EXPORT) ;

		err = gpiochip_add(&gpio->gc[i].gc);
		if (err)
			goto err;

		// this is a bit dirty, but it'll mean we have the correct
		// direction from startup.
		if(gc_data[i].flags & CGF_DIR_HARDCODED) {
			gpio->gc[i].gc.direction_output = fpga_gpio_dir_out_dummy;
			for(j=0; j<ngpio; j++) {
				if(!(gc_data[i].dir & BIT(j))) {
					gpio_request(gc_data[i].base + j, gc_data[i].names[j]);
					gpio_direction_output(gc_data[i].base + j, 0);
					gpio_free(gc_data[i].base + j);
				}
			}
#ifdef CONFIG_HTC_FPGA_IO
			if(!(gc_data[i].flags & CGF_IOBOARD)) {
#endif
				gpio->gc[i].gc.direction_output = fpga_bb_gpio_dir_out;
#ifdef CONFIG_HTC_FPGA_IO
			} else {
				gpio->gc[i].gc.direction_output = fpga_io_gpio_dir_out;
			}
#endif
		}

		if(gc_data[i].flags & CGF_EXPORT) {
			for(j=0; j<ngpio; j++) {
				// Must request a GPIO before exporting it!
				gpio_request(gc_data[i].base + j, gc_data[i].names[j]);

				err = gpio_export(gc_data[i].base + j, !(gc_data[i].flags & CGF_DIR_HARDCODED));
				//gpio_pr_info("FPGA GPIO: gpio_export(%s.%u) returned %d\n", gc_data[i].label, j, err);
				err = gpio_export_link(dev, gc_data[i].names[j], gc_data[i].base + j);
				//gpio_pr_info("FPGA GPIO: gpio_export_link(%s.%u) returned %d\n", gc_data[i].label, j, err);
				// TODO: support for 'int gpio_sysfs_set_active_low(unsigned gpio, int value);'?
			}
			err = 0;
		}
	}
	dev_info(dev, "HTC FPGA GPIO driver\n");

err:
	return err;

}

static int fpga_gpio_remove(struct platform_device *pdev)
{
	int err = 0, i;
	struct fpga_gpio *gpio = platform_get_drvdata(pdev);

	for(i=0; i<nr_gpio_types; i++) {
		if((err = gpiochip_remove(&gpio->gc[i].gc)) < 0)
			return err;
	}
	return 0;
}

static struct platform_driver fpga_gpio_driver = {
	.probe	= fpga_gpio_probe,
	.remove	= fpga_gpio_remove,
	.driver	= {
		.name	= "htc_fpga_gpio"
	},
};

static int __init fpga_gpio_init(void)
{
	return platform_driver_register(&fpga_gpio_driver);
}

static void __exit fpga_gpio_exit(void)
{
	platform_driver_unregister(&fpga_gpio_driver);
}

module_init(fpga_gpio_init);
module_exit(fpga_gpio_exit);

MODULE_DESCRIPTION("ASH FPGA GPIO driver");
MODULE_AUTHOR("David Steinberg <dsteinberg@hanoverdisplays.com>");
MODULE_LICENSE("Proprietary");
