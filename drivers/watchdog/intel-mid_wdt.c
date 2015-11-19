/*
 *      intel-mid_wdt: generic Intel MID SCU watchdog driver
 *
 *      Platforms supported so far:
 *      - Merrifield only
 *
 *      Copyright (C) 2014 Intel Corporation. All rights reserved.
 *      Contact: David Cohen <david.a.cohen@linux.intel.com>
 *
 *      This program is free software; you can redistribute it and/or
 *      modify it under the terms of version 2 of the GNU General
 *      Public License as published by the Free Software Foundation.
 */

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/nmi.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>
#include <linux/platform_data/intel-mid_wdt.h>

#include <asm/intel_scu_ipc.h>
#include <asm/intel-mid.h>

#define IPC_WATCHDOG 0xf8

#define MID_WDT_PRETIMEOUT		15
#define MID_WDT_TIMEOUT_MIN		(1 + MID_WDT_PRETIMEOUT)
#define MID_WDT_TIMEOUT_MAX		170
/* Change default watchdog timeout from 90 to 80 for issue seen with
 * SCU setting not being properly set and it using a value of of ~70.
 * Needs further debugging of SCU driver code to trace if its in driver
 * or in SCU Firmware. Will adjust in watchdog driver as workaround.
  #define MID_WDT_DEFAULT_TIMEOUT              90
*/
#define MID_WDT_DEFAULT_TIMEOUT                80

/* SCU watchdog messages */
enum {
	SCU_WATCHDOG_START = 0,
	SCU_WATCHDOG_STOP,
	SCU_WATCHDOG_KEEPALIVE,
};

static inline int wdt_command(int sub, u8 *in, int inlen)
{
	return intel_scu_ipc_command(IPC_WATCHDOG, (u32 *)sub, in, (u32 *)inlen, NULL, 0);
}

static int wdt_start(struct watchdog_device *wd)
{
	int ret;
	int timeout = wd->timeout + MID_WDT_PRETIMEOUT;

	struct ipc_wd_start {
		u32 pretimeout;
		u32 timeout;
	} ipc_wd_start = { timeout - MID_WDT_PRETIMEOUT, timeout };

	ret = wdt_command(SCU_WATCHDOG_START, (u8 *)&ipc_wd_start, sizeof(ipc_wd_start));
	if (ret) {
		struct device *dev = watchdog_get_drvdata(wd);
		dev_crit(dev, "error starting watchdog: %d\n", ret);
	}

	return ret;
}

static int wdt_ping(struct watchdog_device *wd)
{
	int ret;

	ret = wdt_command(SCU_WATCHDOG_KEEPALIVE, NULL, 0);
	if (ret) {
		struct device *dev = watchdog_get_drvdata(wd);
		dev_crit(dev, "Error executing keepalive: 0x%x\n", ret);
	}

	return ret;
}

static int wdt_stop(struct watchdog_device *wd)
{
	int ret;

	ret = wdt_command(SCU_WATCHDOG_STOP, NULL, 0);
	if (ret) {
		struct device *dev = watchdog_get_drvdata(wd);
		dev_crit(dev, "Error stopping watchdog: 0x%x\n", ret);
	}

	return ret;
}

static irqreturn_t mid_wdt_irq(int irq, void *dev_id)
{
	panic("Kernel Watchdog");

	/* This code should not be reached */
	return IRQ_HANDLED;
}

static const struct watchdog_info mid_wdt_info = {
	.identity = "Intel MID SCU watchdog",
	.options = WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT | WDIOF_MAGICCLOSE,
};

static const struct watchdog_ops mid_wdt_ops = {
	.owner = THIS_MODULE,
	.start = wdt_start,
	.stop = wdt_stop,
	.ping = wdt_ping,
};

static int mid_wdt_probe(struct platform_device *pdev)
{
	struct watchdog_device *wdt_dev;
	struct intel_mid_wdt_pdata *pdata = pdev->dev.platform_data;
	int ret;

	if (!pdata) {
		dev_err(&pdev->dev, "missing platform data\n");
		return -EINVAL;
	}

	if (pdata->probe) {
		ret = pdata->probe(pdev);
		if (ret)
			return ret;
	}

	wdt_dev = devm_kzalloc(&pdev->dev, sizeof(*wdt_dev), GFP_KERNEL);
	if (!wdt_dev)
		return -ENOMEM;

	wdt_dev->info = &mid_wdt_info;
	wdt_dev->ops = &mid_wdt_ops;
	wdt_dev->min_timeout = MID_WDT_TIMEOUT_MIN;
	wdt_dev->max_timeout = MID_WDT_TIMEOUT_MAX;
	wdt_dev->timeout = MID_WDT_DEFAULT_TIMEOUT;

	watchdog_set_drvdata(wdt_dev, &pdev->dev);
	platform_set_drvdata(pdev, wdt_dev);
	ret = watchdog_register_device(wdt_dev);
	if (ret) {
		dev_err(&pdev->dev, "error registering watchdog device\n");
		return ret;
	}

	dev_info(&pdev->dev, "Intel MID watchdog device probed\n");

	return 0;
}

static int mid_wdt_remove(struct platform_device *pdev)
{
	struct watchdog_device *wd = platform_get_drvdata(pdev);
	watchdog_unregister_device(wd);
	return 0;
}

static struct platform_driver mid_wdt_driver = {
	.probe		= mid_wdt_probe,
	.remove		= mid_wdt_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "intel_mid_wdt",
	},
};

module_platform_driver(mid_wdt_driver);

MODULE_AUTHOR("David Cohen <david.a.cohen@linux.intel.com>");
MODULE_DESCRIPTION("Watchdog Driver for Intel MID platform");
MODULE_LICENSE("GPL");
