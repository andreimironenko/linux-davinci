/*
 * htc-fpga-odo.c
 *
 * HTC Odometer support.
 *
 * Copyright 2010 Hanover Displays Ltd
 *
 * Author: David Steinberg <dsteinberg@hanoverdisplays.com>
 *
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <mach/htc_fpga.h>
#include <linux/slab.h>

struct htc_odo {
	struct device *dev;
};

static ssize_t show_odo_bb(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", htc_fpga_bb_readl(HTC_FPGA_ODOMETER));
}
DEVICE_ATTR(odo_bb, S_IRUGO, show_odo_bb, 0);

#ifdef CONFIG_HTC_FPGA_IO
static ssize_t show_odo_io(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", htc_fpga_io_readl(HTC_FPGA_ODOMETER));
}
DEVICE_ATTR(odo_io, S_IRUGO, show_odo_io, 0);

#endif/*CONFIG_HTC_FPGA_IO*/

static int __init odo_probe(struct platform_device *pdev)
{
	struct htc_odo *odo;
	struct device *dev = &pdev->dev;
	int ret = 0;

	odo = kzalloc(sizeof(struct htc_odo), GFP_KERNEL);
	if (odo == NULL) {
		printk(KERN_ERR "kzalloc failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, odo);
	odo->dev = dev;

	// we don't do the normal ioremap() stuff here because the FPGAs are already mapped

	ret = device_create_file(dev, &dev_attr_odo_bb);
	if(ret < 0)
		goto out_free;
#ifdef CONFIG_HTC_FPGA_IO
	if(htc_fpga_io_is_configured()) {
		ret = device_create_file(dev, &dev_attr_odo_io);
		if(ret < 0)
			goto out_free;
	}
#endif

	dev_info(odo->dev, "HTC Odometer driver\n");

	return 0;

 out_free:
	kfree(odo);

	return ret;
}

static int odo_remove(struct platform_device *pdev)
{
	//int ret;
	struct htc_odo *odo = platform_get_drvdata(pdev);

	kfree(odo);

	return 0;
}

static void odo_shutdown(struct platform_device *pdev)
{
}

static struct platform_driver odo_device_driver = {
	.driver		= {
		.name	= "htc_odo",
	},
	.remove		= __devexit_p(odo_remove),
	.shutdown	= odo_shutdown,
};

static int __init odo_init(void)
{
	int retval = 0;
	retval = platform_driver_probe(&odo_device_driver, odo_probe);
	return retval;
}

subsys_initcall(odo_init);
