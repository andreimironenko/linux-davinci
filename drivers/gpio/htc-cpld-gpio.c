/*
 * Hanover Displays HTC CPLD I/O
 *
 * Author: David Steinberg <dsteinberg@hanoverdisplays.com>
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <mach/htc_cpld.h>
#include <linux/slab.h>

#if 0
#define gpio_pr_info		pr_info
#else
#define gpio_pr_info(x...)	do { } while(0);
#endif

// TODO: IRQ register support (although that shouldn't be how its done really)

// TODO: this should really come in as platform device data
struct cpld_gpio_gc_data {
	char *label;
	int base;
	int reg;
	int dir;
	int ngpio;
	char *names[8];
	int bits[8];
	int flags;
	int irqs[8];
	int expmask;
};

typedef enum {
	gt_reset,
	gt_spare,
	gt_wdog,
	gt_irq,
	gt_gpio2,

	nr_gpio_types
} gpio_type;
// the GPIO_CTRL register, funnily enough, currently has no bits defined that
// we'd actually want to use as a GPIO!
// And we only want the watchdog stuff for the SPI GPIO driver.
// Later, it'd be better to create a SPI driver that uses these pins directly.
// Got that now, but we still need watchdog reset (and SCL)

struct cpld_gpio_gc {
	struct gpio_chip gc;
	struct cpld_gpio_gc_data *data;
};

struct cpld_gpio {
	struct device *dev;
	struct cpld_gpio_gc gc[nr_gpio_types];
};

// CPLD GPIO Flags
#define CGF_DIR_HARDCODED	BIT(0)		// if 1, .dir is direction mask, if 0, .dir is direction register
#define CGF_DIR_INVERT		BIT(1)		// applies to CGF_DIR_HARDCODED==0 only. if set, 1 means output, 0 means input
#define CGF_EXPORT			BIT(2)		// if 1, then export all IOs to sysfs
#define CGF_IRQ				BIT(5)		// if 1, irqs specify IRQ numbers (or -1)

#define DIR_OUTPUT		0
#define DIR_INPUT		1

static struct cpld_gpio_gc_data gc_data[] = {
	{
		.label		= "htc_reset",
		.base		= 128,
		.reg		= HTC_CPLD_RESET_CTRL,
		.dir		= 0,
		.ngpio		= 8,
		.names		= {"reset_gps", "reset_usb",
					   "reset_fpga_bb", "reset_fpga_io",
					   "reset_wifi", "reset_ext",
					   "reset_eth", "reset_vidif"},
		.bits		= {0, 1, 2, 3, 4, 5, 6, 7},
		.flags		= CGF_DIR_HARDCODED | CGF_EXPORT,
		.expmask	= ~(1 << HTC_RESET_WIFI),		// wifi driver needs this for itself
//		.expmask	= 0xff,
	},
	{
		.label		= "htc_spare",
		.base		= 136,
		.reg		= HTC_CPLD_SPARE_IO_CTRL2,
		.dir		= HTC_CPLD_SPARE_IO_CTRL1,
		.ngpio		= 8,
		.names		= {"spare0", "spare1",
					   "spare2", "spare3",
					   "spare4", "spare5",
					   "spare6", "spare7"},
		.bits		= {0, 1, 2, 3, 4, 5, 6, 7},
		.flags		= CGF_DIR_INVERT | CGF_EXPORT,
		.expmask	= 0xff,
	},
#if 0
	{
		.label		= "htc_wdog",
		.base		= 144,
		.reg		= HTC_CPLD_WDOG_CTRL,
		.dir		= BIT(3),	// only miso is an input
		.ngpio		= 4,
		.names		= {"wdog_reset", "wdog_sclk",
					   "wdog_mosi", "wdog_miso"},
		.bits		= {HTC_WDOG_RESET, HTC_WDOG_SCLK,
					   HTC_WDOG_MOSI, HTC_WDOG_MISO},
		.flags		= CGF_DIR_HARDCODED | CGF_EXPORT,
		.expmask	= 0xff,
	},
#else
	{
		.label		= "htc_wdog",
		.base		= 144,
		.reg		= HTC_CPLD_WDOG_CTRL1,
		.dir		= 0,
		.ngpio		= 2,
		.names		= {"wdog_reset", "wdog_sclk"},
		.bits		= {HTC_WDOG_RESET, HTC_WDOG_SCLK},
		.flags		= CGF_DIR_HARDCODED | CGF_EXPORT,
		.expmask	= 0xff,
	},
#endif
// NOTE: audioboard and ioboard detect will move (hopefully)
	{
		.label		= "htc_int",
		.base		= 148,
		.reg		= HTC_CPLD_IRQ_STATUS,
		.dir		= 0,
		.ngpio		= 1,
		.names		= {"wifi_int"},
		.bits		= {HTC_IRQ_SPI},
		.flags		= CGF_DIR_HARDCODED | CGF_IRQ,
		.irqs		= {HTC_CPLD_IRQ_SPI},
	},
	{
		.label		= "htc_gpio2",
		.base		= 150,
		.reg		= HTC_CPLD_GPIO_CTRL2,
		.dir		= 0,
		.ngpio		= 2,
		.names		= {"audioboard_det", "ioboard_det"},
		.bits		= {HTC_GPIO_AUDIOBOARD_DET, HTC_GPIO_IOBOARD_DET},
		.flags		= CGF_DIR_HARDCODED | CGF_EXPORT,
	},
	// FPGA GPIOs start at 152!
};

static int cpld_gpio_dir_in(struct gpio_chip *gc, unsigned off)
{
	struct cpld_gpio_gc *gpio = container_of(gc, struct cpld_gpio_gc, gc);
	if(gpio->data->flags & CGF_DIR_HARDCODED) {
		if(!(gpio->data->dir & BIT(off))) {
			pr_err("CPLD GPIO: Error: %s.%s cannot be set to input\n", gpio->data->label, gpio->data->names[off]);
			return -EACCES;			// hardcoded to an output
		}
	} else {
		gpio_pr_info("CPLD GPIO: Setting to input %s.%s\n", gpio->data->label, gpio->data->names[off]);
		htc_cpld_writebit(
			!(gpio->data->flags & CGF_DIR_INVERT),
			gpio->data->bits[off],
			gpio->data->dir
		);
	}
	return 0;
}

static int cpld_gpio_dir_out(struct gpio_chip *gc, unsigned off, int val)
{
	struct cpld_gpio_gc *gpio = container_of(gc, struct cpld_gpio_gc, gc);

	if(gpio->data->flags & CGF_DIR_HARDCODED) {
		if(!!(gpio->data->dir & BIT(off))) {
			pr_err("CPLD GPIO: Error: %s.%s cannot be set to output\n", gpio->data->label, gpio->data->names[off]);
			return -EACCES;			// hardcoded to an input
		}
	} else {
		gpio_pr_info("CPLD GPIO: Setting to output %s.%s\n", gpio->data->label, gpio->data->names[off]);
		htc_cpld_writebit(
			!!(gpio->data->flags & CGF_DIR_INVERT),
			gpio->data->bits[off],
			gpio->data->dir
		);
	}

	gpio_pr_info("CPLD GPIO: Setting state of %s.%s to %s\n", gpio->data->label, gpio->data->names[off], val ? "on" : "off");
	htc_cpld_writebit(
		val,
		gpio->data->bits[off],
		gpio->data->reg
	);
	return 0;
}

static int cpld_gpio_dir_out_dummy(struct gpio_chip *gc, unsigned off, int val)
{
	return 0;
}

static int cpld_gpio_get(struct gpio_chip *gc, unsigned off)
{
	struct cpld_gpio_gc *gpio = container_of(gc, struct cpld_gpio_gc, gc);
	gpio_pr_info("CPLD GPIO: Reading state of %s.%s\n", gpio->data->label, gpio->data->names[off]);
	return htc_cpld_readbit(
		gpio->data->bits[off],
		gpio->data->reg
	);
}

static void cpld_gpio_set(struct gpio_chip *gc, unsigned off, int val)
{
	struct cpld_gpio_gc *gpio = container_of(gc, struct cpld_gpio_gc, gc);
	gpio_pr_info("CPLD GPIO: Setting state of %s.%s to %s\n", gpio->data->label, gpio->data->names[off], val ? "on" : "off");
	htc_cpld_writebit(
		val,
		gpio->data->bits[off],
		gpio->data->reg
	);
}

static int cpld_gpio_to_irq(struct gpio_chip *gc, unsigned off) {
	struct cpld_gpio_gc *gpio = container_of(gc, struct cpld_gpio_gc, gc);
	gpio_pr_info("CPLD GPIO: Getting IRQ for %s.%s\n", gpio->data->label, gpio->data->names[off]);
	return gpio->data->irqs[off];
}

static int cpld_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cpld_gpio *gpio;
	int err = 0, i, j;

	gpio = kzalloc(sizeof(struct cpld_gpio), GFP_KERNEL);
	if (gpio == NULL) {
		printk(KERN_ERR "kzalloc failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, gpio);
	gpio->dev = dev;

	for(i=0; i<nr_gpio_types; i++) {
		gpio->gc[i].data = &gc_data[i];
		gpio->gc[i].gc.label = gc_data[i].label;
		gpio->gc[i].gc.base = gc_data[i].base;
		gpio->gc[i].gc.ngpio = gc_data[i].ngpio;
		gpio->gc[i].gc.owner = THIS_MODULE;
		gpio->gc[i].gc.dev= dev;
		gpio->gc[i].gc.names = gc_data[i].names;

		if(gc_data[i].flags & CGF_IRQ) {
			gpio_pr_info("CPLD GPIO: Chip '%s' is IRQable\n", gc_data[i].label);
			gpio->gc[i].gc.to_irq = cpld_gpio_to_irq;
		}

		if(!(gc_data[i].flags & CGF_DIR_HARDCODED) || (gc_data[i].dir != 0))
			gpio->gc[i].gc.direction_input = cpld_gpio_dir_in;
		else
			gpio_pr_info("CPLD GPIO: Not providing 'direction_input' for chip %s\n", gc_data[i].label);
		gpio->gc[i].gc.direction_output = cpld_gpio_dir_out;
		gpio->gc[i].gc.get = cpld_gpio_get;
		gpio->gc[i].gc.set = cpld_gpio_set;
		gpio->gc[i].gc.can_sleep = 0;
		//gpio->gc[i].gc.exported = !!(gc_data[i].flags & CGF_EXPORT) ;

		err = gpiochip_add(&gpio->gc[i].gc);
		if (err)
			goto err;

		// this is a bit dirty, but it'll mean we have the correct
		// direction from startup.
		if(gc_data[i].flags & CGF_DIR_HARDCODED) {
			gpio->gc[i].gc.direction_output = cpld_gpio_dir_out_dummy;
			for(j=0; j<gc_data[i].ngpio; j++) {
				if(!(gc_data[i].dir & BIT(j))) {
					gpio_request(gc_data[i].base + j, gc_data[i].names[j]);
					gpio_direction_output(gc_data[i].base + j,
						htc_cpld_readbit(
								gc_data[i].bits[j],
								gc_data[i].reg
							)
					);
					gpio_free(gc_data[i].base + j);
				}
			}
			gpio->gc[i].gc.direction_output = cpld_gpio_dir_out;
		}

		if(gc_data[i].flags & CGF_EXPORT) {
			for(j=0; j<gc_data[i].ngpio; j++) {
				if(!(gc_data[i].expmask & (1 << j)))
					continue;
				// Must request a GPIO before exporting it!
				gpio_request(gc_data[i].base + j, gc_data[i].names[j]);

				err = gpio_export(gc_data[i].base + j, !(gc_data[i].flags & CGF_DIR_HARDCODED));
				//gpio_pr_info("CPLD GPIO: gpio_export(%s.%u) returned %d\n", gc_data[i].label, j, err);
				err = gpio_export_link(dev, gc_data[i].names[j], gc_data[i].base + j);
				//gpio_pr_info("CPLD GPIO: gpio_export_link(%s.%u) returned %d\n", gc_data[i].label, j, err);
				// TODO: support for 'int gpio_sysfs_set_active_low(unsigned gpio, int value);'?
			}
			err = 0;
		}
	}
	dev_info(dev, "HTC CPLD GPIO driver\n");

err:
	return err;

}

static int cpld_gpio_remove(struct platform_device *pdev)
{
	int err = 0, i;
	struct cpld_gpio *gpio = platform_get_drvdata(pdev);

	for(i=0; i<nr_gpio_types; i++) {
		if((err = gpiochip_remove(&gpio->gc[i].gc)) < 0)
			return err;
	}
	return 0;
}

static struct platform_driver cpld_gpio_driver = {
	.probe	= cpld_gpio_probe,
	.remove	= cpld_gpio_remove,
	.driver	= {
		.name	= "htc_cpld_gpio"
	},
};

static int __init cpld_gpio_init(void)
{
	return platform_driver_register(&cpld_gpio_driver);
}

static void __exit cpld_gpio_exit(void)
{
	platform_driver_unregister(&cpld_gpio_driver);
}

module_init(cpld_gpio_init);
module_exit(cpld_gpio_exit);

MODULE_DESCRIPTION("HTC CPLD GPIO driver");
MODULE_AUTHOR("David Steinberg <dsteinberg@hanoverdisplays.com>");
MODULE_LICENSE("Proprietary");
