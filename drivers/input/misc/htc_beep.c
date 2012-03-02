/*
 *  Hanover HTC Beeper driver
 *
 *  Copyright (c) 2010 Hanover Displays
 *
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <mach/htc_cpld.h>

MODULE_AUTHOR("David Steinberg <dsteinberg@hanoverdisplays.com>");
MODULE_DESCRIPTION("Hanover HTC beeper driver");
MODULE_LICENSE("GPL");

static int htcbeep_event(struct input_dev *dev, unsigned int type, unsigned int code, int value)
{
#if 0
	unsigned int count = 0;
	unsigned long flags;
#endif

	if (type != EV_SND)
		return -1;

#if 0
	switch (code) {
		case SND_BELL: if (value) value = 1000;
		case SND_TONE: break;
		default: return -1;
	}

	if (value > 20 && value < 32767)
		count = PIT_TICK_RATE / value;

	spin_lock_irqsave(&i8253_lock, flags);

	if (count) {
		/* set command for counter 2, 2 byte write */
		outb_p(0xB6, 0x43);
		/* select desired HZ */
		outb_p(count & 0xff, 0x42);
		outb((count >> 8) & 0xff, 0x42);
		/* enable counter 2 */
		outb_p(inb_p(0x61) | 3, 0x61);
	} else {
		/* disable counter 2 */
		outb(inb_p(0x61) & 0xFC, 0x61);
	}

	spin_unlock_irqrestore(&i8253_lock, flags);
#endif

	//pr_info("HTC beep: %d\n", value);
	htc_cpld_writebit(value, HTC_GPIO_BUZZER, HTC_CPLD_GPIO_CTRL1);

	return 0;
}

static int __devinit htcbeep_probe(struct platform_device *dev)
{
	struct input_dev *htcbeep_dev;
	int err;

	htcbeep_dev = input_allocate_device();
	if (!htcbeep_dev)
		return -ENOMEM;

	htcbeep_dev->name = "HTC Beeper";
	htcbeep_dev->phys = "htc/generic";
	htcbeep_dev->id.bustype = BUS_HOST;
	htcbeep_dev->id.vendor  = 0x001f;
	htcbeep_dev->id.product = 0x0001;
	htcbeep_dev->id.version = 0x0100;
	htcbeep_dev->dev.parent = &dev->dev;

	htcbeep_dev->evbit[0] = BIT_MASK(EV_SND);
	htcbeep_dev->sndbit[0] = BIT_MASK(SND_BELL) | BIT_MASK(SND_TONE);
	htcbeep_dev->event = htcbeep_event;

	err = input_register_device(htcbeep_dev);
	if (err) {
		//pr_err("htcbeep_probe couldnt register\n");
		input_free_device(htcbeep_dev);
		return err;
	}

	platform_set_drvdata(dev, htcbeep_dev);
	dev_info(&dev->dev, "HTC beeper driver\n");

	return 0;
}

static int __devexit htcbeep_remove(struct platform_device *dev)
{
	struct input_dev *htcbeep_dev = platform_get_drvdata(dev);

	input_unregister_device(htcbeep_dev);
	platform_set_drvdata(dev, NULL);
	/* turn off the speaker */
	htcbeep_event(NULL, EV_SND, SND_BELL, 0);

	return 0;
}

static int htcbeep_suspend(struct device *dev)
{
	htcbeep_event(NULL, EV_SND, SND_BELL, 0);

	return 0;
}

static void htcbeep_shutdown(struct platform_device *dev)
{
	/* turn off the speaker */
	htcbeep_event(NULL, EV_SND, SND_BELL, 0);
}

static struct dev_pm_ops htcbeep_pm_ops = {
	.suspend = htcbeep_suspend,
};

static struct platform_driver htcbeep_platform_driver = {
	.driver		= {
		.name	= "htc_beep",
		.owner	= THIS_MODULE,
		.pm	= &htcbeep_pm_ops,
	},
	.probe		= htcbeep_probe,
	.remove		= __devexit_p(htcbeep_remove),
	.shutdown	= htcbeep_shutdown,
};


static int __init htcbeep_init(void)
{
	//pr_err("htcbeep_init\n");
	return platform_driver_register(&htcbeep_platform_driver);
}

static void __exit htcbeep_exit(void)
{
	platform_driver_unregister(&htcbeep_platform_driver);
}

module_init(htcbeep_init);
module_exit(htcbeep_exit);
