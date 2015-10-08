/*
 * platform_gpio_leds.c: gpio_leds platform data initilization file
 *
 * (C) Copyright 2015 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <asm/intel-mid.h>


static struct platform_device led_dev = {
	.name = "leds-edison-gpio-flash",
	.id = -1,
};

static int __init leds_gpio_init(void)
{
	pr_info("%s: Registering Intel LED GPIO platform device\n", __func__);
	return platform_device_register(&led_dev);
}

rootfs_initcall(leds_gpio_init);
