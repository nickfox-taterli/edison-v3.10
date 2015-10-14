
/* (C) Copyright 2015 Intel Corporation
 * Author: Brian Wood <brian.j.wood@intel.com>
 *
 * Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
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
#include <linux/device.h>

#define LED_GPIO_FLASH_DRIVER_NAME	"leds-edison-gpio-flash"
#define LED_TRIGGER_DEFAULT		"none"
#define LED_MAX_BRIGHTNESS		1

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
	struct work_struct	led_flash_work;
};

/* Workqueue handler function */
static void ledwq_work_handler(struct work_struct *ws)
{
	int ret = 0;
	struct led_gpio_flash_data *flash_led =
		container_of(ws, struct led_gpio_flash_data, led_flash_work);

	pr_debug("%s: Use the workqueue for flashing the LED\n", __func__);

	while(1) {
		if (ret) {
			pr_debug("ON\n");
			gpio_set_value_cansleep(flash_led->gpio_pin_num, 0);
			ret = 0;
			msleep(flash_led->cdev.blink_delay_on);
		} else { /* ret == 0 */
			pr_debug("OFF\n");
			gpio_set_value_cansleep(flash_led->gpio_pin_num, 1);
			ret = 1;
			msleep(flash_led->cdev.blink_delay_off);
		}
		/* Code below is an alternate timer code flow that would use
		 * jiffies, it's more efficient than msleep but with HZ at
		 * 100 it can be unpredictable with values <10ms; this
		 * could lead to less predictable flash patterns with
		 * faster flashing (lower ms values for delay_on/delay_off).
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout (1000/(flash_led->cdev.brightness+1));
		 */
		if (!flash_led->cdev.blink_delay_on ||
			!flash_led->cdev.blink_delay_off)
			break;
	}
}

static int led_gpio_blink_set(struct led_classdev *led_cdev,
				unsigned long *delay_on,
				unsigned long *delay_off)
{
	struct led_gpio_flash_data *flash_led =
		container_of(led_cdev, struct led_gpio_flash_data, cdev);

	flash_led->cdev.brightness = 0;
	pr_debug("%s: Reset brightness=%d, setup flashing LED "
		"delay_on/delay_off\n",
		__func__, flash_led->cdev.brightness);
	pr_debug("%s: delay_on=%lu, delay_off=%lu\n",
		__func__, *delay_on, *delay_off);

	pr_debug("%s: Prepare and queue work\n", __func__);
	schedule_work(&flash_led->led_flash_work);


	return 0;
}

static ssize_t led_delay_on_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
		struct led_classdev *led_cdev = dev_get_drvdata(dev);

		return sprintf(buf, "%lu\n", led_cdev->blink_delay_on);
}

static ssize_t led_delay_on_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
		struct led_classdev *led_cdev = dev_get_drvdata(dev);
		unsigned long state;
		ssize_t ret = -EINVAL;

		ret = kstrtoul(buf, 10, &state);
		if (ret)
			return ret;

		led_blink_set(led_cdev, &state, &led_cdev->blink_delay_off);
		led_cdev->blink_delay_on = state;

return size;
}

static ssize_t led_delay_off_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
		struct led_classdev *led_cdev = dev_get_drvdata(dev);

		return sprintf(buf, "%lu\n", led_cdev->blink_delay_off);
}

static ssize_t led_delay_off_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
		struct led_classdev *led_cdev = dev_get_drvdata(dev);
		unsigned long state;
		ssize_t ret = -EINVAL;

		ret = kstrtoul(buf, 10, &state);
		if (ret)
			return ret;

		led_blink_set(led_cdev, &led_cdev->blink_delay_on, &state);
		led_cdev->blink_delay_off = state;

		return size;
}

static DEVICE_ATTR(delay_on, 0644, led_delay_on_show, led_delay_on_store);
static DEVICE_ATTR(delay_off, 0644, led_delay_off_show, led_delay_off_store);


static void led_gpio_brightness_set(struct led_classdev *led_cdev,
					enum led_brightness value)
{
	struct led_gpio_flash_data *flash_led =
		container_of(led_cdev, struct led_gpio_flash_data, cdev);
	int brightness = value;

	/* Reset blink*/
	flash_led->cdev.blink_delay_on =
		flash_led->cdev.blink_delay_off = LED_OFF;

	if (brightness >= 1) { /* Turn on LED */
		pr_debug("%s: Turning on LED, brightness=%d\n", __func__, brightness);
		gpio_set_value_cansleep(flash_led->gpio_pin_num, 0);
		/* if 0 we failed to set */
		if (gpio_get_value_cansleep(flash_led->gpio_pin_num)) {
			pr_err("%s: Failed to set gpio %d\n", __func__,
				flash_led->gpio_pin_num);
			goto err;
		}
		pr_debug("%s: GPIO value=%d\n", __func__,
			gpio_get_value_cansleep(flash_led->gpio_pin_num));
		flash_led->cdev.brightness = 1;
		pr_debug("%s: Stored new value for brightness=%d\n", __func__,
			flash_led->cdev.brightness);
	} else { /* brightness <= 0, Turn off LED */
		pr_debug("%s: Turning off LED, brightness=%d\n", __func__, brightness);
		gpio_set_value_cansleep(flash_led->gpio_pin_num, 1);
		/* if nonzero we failed to set */
		if (gpio_get_value_cansleep(flash_led->gpio_pin_num)) {
			pr_err("%s: Failed to set gpio %d\n", __func__,
				flash_led->gpio_pin_num);
			goto err;
		}
		pr_debug("%s: GPIO value=%d\n", __func__,
			gpio_get_value_cansleep(flash_led->gpio_pin_num));
		flash_led->cdev.brightness = 0;
		pr_debug("%s: Stored new value for brightness=%d\n", __func__,
			flash_led->cdev.brightness);
	}

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
	dev_dbg(&pdev->dev, "%s: GPIO value=%d\n",
		__func__, gpio_get_value_cansleep(flash_led->gpio_pin_num));

	flash_led->cdev.name = "DS2_Green_LED";
	dev_dbg(&pdev->dev, "%s: cdev name=%s\n",
		__func__, flash_led->cdev.name);

	platform_set_drvdata(pdev, flash_led);
	flash_led->cdev.max_brightness = LED_MAX_BRIGHTNESS;
	flash_led->cdev.brightness_set = led_gpio_brightness_set;
	flash_led->cdev.brightness_get = led_gpio_brightness_get;
	flash_led->cdev.blink_set = led_gpio_blink_set;

	rc = led_classdev_register(&pdev->dev, &flash_led->cdev);
	if (rc) {
		dev_err(&pdev->dev, "%s: Failed to register led dev. rc = %d\n",
			__func__, rc);
		goto error;
	}

	rc = device_create_file(flash_led->cdev.dev, &dev_attr_delay_on);
	if (rc)
		dev_err(&pdev->dev, "%s: cannot create LED dev_attr delay_on, error=%d\n",
			__func__, rc);
	rc = device_create_file(flash_led->cdev.dev, &dev_attr_delay_off);
	if (rc)
		dev_err(&pdev->dev, "%s: cannot create LED dev_attr delay_off, error=%d\n",
			__func__, rc);

	dev_dbg(&pdev->dev, "%s:probe successfully!\n", __func__);
	return 0;

error:

	device_remove_file(flash_led->cdev.dev, &dev_attr_delay_off);
	device_remove_file(flash_led->cdev.dev, &dev_attr_delay_on);

	devm_kfree(&pdev->dev, flash_led);

	return rc;
}

int led_gpio_flash_remove(struct platform_device *pdev)
{
	struct led_gpio_flash_data *flash_led =
	    (struct led_gpio_flash_data *)platform_get_drvdata(pdev);
	gpio_free(flash_led->gpio_pin_num);
	gpio_free(flash_led->gpio_pin_mux_num);

	device_remove_file(flash_led->cdev.dev, &dev_attr_delay_off);
	device_remove_file(flash_led->cdev.dev, &dev_attr_delay_on);

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

