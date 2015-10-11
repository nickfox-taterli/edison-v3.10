
/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 * (C) Copyright 2015 Intel Corporation
 * Author: Brian Wood <brian.j.wood@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/printk.h>
#include <linux/list.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/sched.h>

#define LED_GPIO_FLASH_DRIVER_NAME	"leds-edison-gpio-flash"
#define LED_TRIGGER_DEFAULT		"none"

#define GPIO_PIN_NUM 		243	/* temporary hardcoded value for development */
#define GPIO_PIN_MUX_NUM 	261     /* "     "         "       "         "     " */
#define GPIO_LABEL 		"DS2"
#define GPIO_LABEL_MUX 		"DS2_MUX"

#define GPIO_OUT_LOW          (0 << 1)
#define GPIO_OUT_HIGH         (1 << 1)

struct led_gpio_flash_data {
	struct led_classdev cdev;
	int gpio_pin_num;	/* record single GPIO number, hard coded
				 * value for now (will change)
				 */
	int gpio_pin_mux_num;
	int gpio_value;		/* record single GPIO value on/off -> 1/0 */
	struct work_struct	led_flash_work;
};

/* Workqueue handler function */
static void ledwq_work_handler(struct work_struct *ws)
{
	pr_debug("%s: Use the workqueue for flashing the LED\n", __func__);
	int ret = 0;
	struct led_gpio_flash_data *flash_led =
		container_of(ws, struct led_gpio_flash_data, led_flash_work);
	while(1) {
		if (ret) {
			pr_debug("ON\n");
			gpio_set_value_cansleep(flash_led->gpio_pin_num, 0);
			ret = 0;
		} else { /* ret == 0 */
			pr_debug("OFF\n");
			gpio_set_value_cansleep(flash_led->gpio_pin_num, 1);
			ret = 1;
		}
		pr_debug("Delay...\n");
		/* Code below is an alternate timer that would use jiffies,
		 * it's more efficient than msleep but with HZ at 100 it
		 * can be unpredictable with values <10ms; this
		 * could lead to less predictable flash patterns with
		 * faster flashing (higher brightness values).
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout (1000/(flash_led->cdev.brightness+1));
		 */
		msleep(10000/(flash_led->cdev.brightness+1));
		if (flash_led->cdev.brightness == LED_OFF ||
			flash_led->cdev.brightness == LED_FULL)
			break;
	}
}

static void led_gpio_brightness_set(struct led_classdev *led_cdev,
				    enum led_brightness value)
{
	struct led_gpio_flash_data *flash_led =
	    container_of(led_cdev, struct led_gpio_flash_data, cdev);
	int brightness = value;

	if (brightness >= 255) { /* Turn on LED */		
		pr_debug("%s: Turning on LED, brightness=%d\n", __func__, brightness);
		gpio_set_value_cansleep(flash_led->gpio_pin_num, 0);
		/* if 0 we failed to set */
		if (gpio_get_value_cansleep(flash_led->gpio_pin_num)) {
			pr_err("%s: Failed to set gpio %d\n", __func__,
				flash_led->gpio_pin_num);
			goto err;
		}
		flash_led->gpio_value = gpio_get_value_cansleep(flash_led->gpio_pin_num);
		pr_debug("%s: GPIO value=%d\n", __func__, flash_led->gpio_value);
        } else if (brightness <= 0) { /* Turn off LED */
		pr_debug("%s: Turning off LED, brightness=%d\n", __func__, brightness);
		gpio_set_value_cansleep(flash_led->gpio_pin_num, 1);
		/* if nonzero we failed to set */
		if (gpio_get_value_cansleep(flash_led->gpio_pin_num)) {
			pr_err("%s: Failed to set gpio %d\n", __func__,
				flash_led->gpio_pin_num);
			goto err;
		}
		flash_led->gpio_value = gpio_get_value_cansleep(flash_led->gpio_pin_num);
		pr_debug("%s: GPIO value=%d\n", __func__, flash_led->gpio_value);
        } else { /* Use the workqueue for flashing the LED */
		pr_debug("%s: Prepare and queue work\n", __func__);
		schedule_work(&flash_led->led_flash_work);
	}

	flash_led->cdev.brightness = brightness;
	pr_debug("%s: Stored new value for brightness=%d\n", __func__,
		flash_led->cdev.brightness);
err:
	return;
}

static enum led_brightness led_gpio_brightness_get(struct led_classdev
						   *led_cdev)
{
	struct led_gpio_flash_data *flash_led =
	    container_of(led_cdev, struct led_gpio_flash_data, cdev);
	pr_debug("%s: returning brightness=%d\n", __func__, flash_led->cdev.brightness);
	return flash_led->cdev.brightness;
}

int led_gpio_flash_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct led_gpio_flash_data *flash_led = NULL;
	
	flash_led = devm_kzalloc(&pdev->dev, sizeof(struct led_gpio_flash_data),
				 GFP_KERNEL);
	if (flash_led == NULL) {
		dev_err(&pdev->dev, "%s:%d Unable to allocate memory\n",
			__func__, __LINE__);
		return -ENOMEM;
	}

	INIT_WORK(&flash_led->led_flash_work, ledwq_work_handler);
	flash_led->gpio_pin_num = GPIO_PIN_NUM;
	flash_led->gpio_pin_mux_num = GPIO_PIN_MUX_NUM;
	flash_led->cdev.default_trigger = LED_TRIGGER_DEFAULT;
	dev_dbg(&pdev->dev, "%s: GPIO LED number=%d, GPIO LED pin mux number=%d, "
		"default trigger=%s\n", __func__, flash_led->gpio_pin_num,
		flash_led->gpio_pin_mux_num, flash_led->cdev.default_trigger);

	/* Request the output enable pinmux for the DS2 LED GPIO */
	rc = gpio_request(flash_led->gpio_pin_mux_num, GPIO_LABEL_MUX);
	if (rc) {
		dev_err(&pdev->dev, "%s: Failed to request gpio %d,rc = %d\n",
			 __func__, flash_led->gpio_pin_mux_num, rc);
		goto error;
	}
	/* Make sure the direction is out */
	rc = gpio_direction_output(flash_led->gpio_pin_mux_num, 0);
	if (rc)
		dev_err(&pdev->dev, "%s: Setting GPIO %d direction to out failed!, rc=%d\n",
			__func__, flash_led->gpio_pin_mux_num, rc);
	else
		dev_dbg(&pdev->dev, "%s: Setting GPIO %d direction to out succeeded, "
			"rc=%d.\n", __func__, flash_led->gpio_pin_mux_num, rc);

	/* Request the DS2 LED GPIO */
	rc = gpio_request(flash_led->gpio_pin_num, GPIO_LABEL);
	if (rc) {
		dev_err(&pdev->dev, "%s: Failed to request gpio %d,rc = %d\n",
		 __func__, flash_led->gpio_pin_num, rc);
		goto error;
	}
	/* Make sure the direction is out */
	rc = gpio_direction_output(flash_led->gpio_pin_num, 1);
	if (rc)
		dev_err(&pdev->dev, "%s: Setting GPIO %d direction to out failed!, "
			"rc=%d.\n", __func__, flash_led->gpio_pin_num, rc);
	else
		dev_dbg(&pdev->dev, "%s: Setting GPIO %d direction to out succeeded, "
			"rc=%d.\n", __func__, flash_led->gpio_pin_num, rc);

	/* If GPIO value is 0 set to 1 so light is off initially */
	rc = gpio_get_value_cansleep(flash_led->gpio_pin_num);
	dev_dbg(&pdev->dev, "%s: GPIO (%d) initial value=%d, "
		"changing value to %d\n",
		__func__, flash_led->gpio_pin_num, rc, 1);
	if (!rc)
		gpio_set_value_cansleep(flash_led->gpio_pin_num, 1);
	flash_led->gpio_value = gpio_get_value_cansleep(flash_led->gpio_pin_num);
	dev_dbg(&pdev->dev, "%s: GPIO value=%d\n",
		__func__, flash_led->gpio_value);

	flash_led->cdev.name = "DS2_Green_LED";
	dev_dbg(&pdev->dev, "%s: cdev name=%s\n",
		__func__, flash_led->cdev.name);

	platform_set_drvdata(pdev, flash_led);
	flash_led->cdev.max_brightness = LED_FULL;
	flash_led->cdev.brightness_set = led_gpio_brightness_set;
	flash_led->cdev.brightness_get = led_gpio_brightness_get;

	rc = led_classdev_register(&pdev->dev, &flash_led->cdev);
	if (rc) {
		dev_err(&pdev->dev, "%s: Failed to register led dev. rc = %d\n",
			__func__, rc);
		goto error;
	}
	dev_dbg(&pdev->dev, "%s:probe successfully!\n", __func__);
	return 0;

error:
	devm_kfree(&pdev->dev, flash_led);
	return rc;
}

int led_gpio_flash_remove(struct platform_device *pdev)
{
	struct led_gpio_flash_data *flash_led =
	    (struct led_gpio_flash_data *)platform_get_drvdata(pdev);
	gpio_free(flash_led->gpio_pin_num);
	gpio_free(flash_led->gpio_pin_mux_num);
	led_classdev_unregister(&flash_led->cdev);
	devm_kfree(&pdev->dev, flash_led);
	pr_debug("%s: Unregistering Intel LED GPIO driver", __func__);
	return 0;
}

static struct platform_driver led_gpio_flash_driver = {
	.probe = led_gpio_flash_probe,
	.remove = led_gpio_flash_remove,
	.driver = {
		   .name = LED_GPIO_FLASH_DRIVER_NAME,
		   .owner = THIS_MODULE,
		   }
};

static int __init led_gpio_flash_init(void)
{
	pr_debug("%s: Registering Intel LED GPIO driver\n", __func__);
	return platform_driver_register(&led_gpio_flash_driver);
}

static void __exit led_gpio_flash_exit(void)
{
	pr_debug("%s: Unregistering Intel LED GPIO driver\n", __func__);
	return platform_driver_unregister(&led_gpio_flash_driver);
}

late_initcall(led_gpio_flash_init);
module_exit(led_gpio_flash_exit);

MODULE_DESCRIPTION("Intel GPIO LEDs driver");
MODULE_LICENSE("GPL v2");

