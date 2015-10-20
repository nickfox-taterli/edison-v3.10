
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

#define GPIO_TRI_STATE_ALL	214	/* hardcoded value for development */
#define GPIO_PIN_NUM		243	/*       "       "         "     " */
#define GPIO_PIN_MUX_NUM	261	/*       "       "         "     " */
#define GPIO_LABEL		"DS2"
#define GPIO_LABEL_MUX		"DS2_MUX"
#define GPIO_LABEL_TRI_STATE	"DS2_TRI_STATE"

#define GPIO_OUT_LOW          (0 << 1)
#define GPIO_OUT_HIGH         (1 << 1)

#undef LED_DBG
#ifdef CONFIG_LEDS_INTEL_GPIO_FLASH_DEBUG
#define LED_DBG(fmt, args...) pr_err(fmt, ##args)
#else
#define LED_DBG(fmt, args...)
#endif

struct led_gpio_flash_data {
	struct led_classdev cdev;
	int gpio_pin_num;	/* record single GPIO number, hard coded
				 * value for now (may change to dynamic)
				 */
	int gpio_pin_state;	/* current GPIO state, 1 locked/0 free */
	int gpio_pin_mux_num;
	int gpio_pin_mux_state;
	int gpio_pin_tri_state_num;
	int gpio_pin_tri_state_state;
	int blinking;           /* blinking state, 1 enabled/0 disabled */
	struct work_struct	led_flash_work;
};

/* GPIO request/free */
static int led_gpio_request(struct led_classdev *led_cdev)
{

	int rc;
	struct led_gpio_flash_data *flash_led =
		container_of(led_cdev, struct led_gpio_flash_data, cdev);

	/* Reserve the GPIO's for LED */

	/* Test if GPIO_TRI_STATE_ALL is available, if not we know MRAA
	 * has exported for SPI use
	 */
	flash_led->gpio_pin_tri_state_state =
		gpio_request(flash_led->gpio_pin_tri_state_num,
			GPIO_LABEL_TRI_STATE);
	if (flash_led->gpio_pin_tri_state_state == -EBUSY) {
		/* GPIO is exported by MRAA */
		pr_info("%s: GPIO_TRI_STATE_ALL (%d) is busy/exported "
			"by someone, we need to set direction to input\n",
			__func__, flash_led->gpio_pin_tri_state_num);
		/* Set direction for pin */
		rc = gpio_direction_input(flash_led->gpio_pin_tri_state_num);
		if (rc)
			pr_err("%s: Setting GPIO %d direction to input failed! "
				",rc=%d\n", __func__,
				flash_led->gpio_pin_tri_state_num, rc);
		else
			pr_info("%s: Setting GPIO %d direction to input "
				"succeeded, rc=%d.\n", __func__,
				flash_led->gpio_pin_tri_state_num, rc);

	} /* else: GPIO is unchanged from defaults, no need to adjust */

	/* Request the output enable pinmux for the DS2 LED GPIO */
	rc = gpio_request(flash_led->gpio_pin_mux_num, GPIO_LABEL_MUX);
	if (rc) {
		pr_err("%s: Failed to request gpio %d,rc = %d\n",
			 __func__, flash_led->gpio_pin_mux_num, rc);
		/* Reset blink values to 0 since we've failed */
		flash_led->cdev.blink_delay_on =
			flash_led->cdev.blink_delay_off =
			flash_led->blinking = LED_OFF;
		goto error;
	}
	/* Make sure the direction is out */
	rc = gpio_direction_output(flash_led->gpio_pin_mux_num, 0);
	if (rc)
		pr_err("%s: Setting GPIO %d direction to out failed!, rc=%d\n",
			__func__, flash_led->gpio_pin_mux_num, rc);
	else
		LED_DBG("%s: Setting GPIO %d direction to out succeeded, "
			"rc=%d.\n", __func__, flash_led->gpio_pin_mux_num, rc);

	/* Request the DS2 LED GPIO */
	rc = gpio_request(flash_led->gpio_pin_num, GPIO_LABEL);
	if (rc) {
		pr_err("%s: Failed to request gpio %d,rc = %d\n",
		 __func__, flash_led->gpio_pin_num, rc);
		goto error2;
	}
	/* Make sure the direction is out */
	rc = gpio_direction_output(flash_led->gpio_pin_num, 1);
	if (rc)
		pr_err("%s: Setting GPIO %d direction to out failed!, "
			"rc=%d.\n", __func__, flash_led->gpio_pin_num, rc);
	else
		LED_DBG("%s: Setting GPIO %d direction to out succeeded, "
			"rc=%d.\n", __func__, flash_led->gpio_pin_num, rc);

	/* If GPIO value is 0 set to 1 so light is off initially */
	rc = gpio_get_value_cansleep(flash_led->gpio_pin_num);
	LED_DBG("%s: GPIO (%d) initial value=%d, "
		"changing value to %d\n",
		__func__, flash_led->gpio_pin_num, rc, 1);
	if (!rc)
		gpio_set_value_cansleep(flash_led->gpio_pin_num, 1);
	LED_DBG("%s: GPIO value=%d\n",
		__func__, gpio_get_value_cansleep(flash_led->gpio_pin_num));

	/* GPIO's reserved for use by driver */
	flash_led->gpio_pin_state = flash_led->gpio_pin_mux_state = 1;

	return 0;

error:
	pr_err("%s: Failed to request pinmux GPIO=%d\n",
		__func__, GPIO_PIN_MUX_NUM);
	return rc;

error2:
	pr_err("%s: Failed to request GPIO=%d\n",
		__func__, GPIO_PIN_NUM);
	gpio_free(flash_led->gpio_pin_mux_num);
	return rc;
}

static int led_gpio_free(struct led_classdev *led_cdev)
{
	int rc;
	struct led_gpio_flash_data *flash_led =
		container_of(led_cdev, struct led_gpio_flash_data, cdev);

	/* Free the GPIO's for use outside driver */
	gpio_free(flash_led->gpio_pin_num);
	gpio_free(flash_led->gpio_pin_mux_num);
	if (!flash_led->gpio_pin_tri_state_state) {
		LED_DBG("%s: Freeing GPIO_TRI_STATE_ALL=%d\n",
			__func__, GPIO_TRI_STATE_ALL);
		gpio_free(flash_led->gpio_pin_tri_state_num);
	} else {
		pr_info("%s: GPIO_TRI_STATE_ALL (%d) is being reset "
			"back to output direction\n",
			__func__, flash_led->gpio_pin_tri_state_num);
		/* Set direction for pin */
		rc = gpio_direction_output(flash_led->gpio_pin_tri_state_num, 0);
		if (rc)
			pr_err("%s: Setting GPIO %d direction to output failed! "
				",rc=%d\n", __func__,
				flash_led->gpio_pin_tri_state_num, rc);
		else
			pr_info("%s: Setting GPIO %d direction to output "
				"succeeded, rc=%d.\n", __func__,
				flash_led->gpio_pin_tri_state_num, rc);

	}
	LED_DBG("%s: Freeing GPIO=%d and GPIO_PIN_MUX=%d\n",
		__func__, GPIO_PIN_NUM, GPIO_PIN_MUX_NUM);

	flash_led->gpio_pin_state = flash_led->gpio_pin_mux_state =
		flash_led->gpio_pin_tri_state_state = 0;

	return 0;
}

/* Workqueue handler */
static void ledwq_work_handler(struct work_struct *ws)
{
	int ret = 0, rc = 0;
	struct led_gpio_flash_data *flash_led =
		container_of(ws, struct led_gpio_flash_data, led_flash_work);

	LED_DBG("%s: Use the workqueue for flashing the LED\n", __func__);

	if (!flash_led->gpio_pin_state && !flash_led->gpio_pin_mux_state) {
		rc = led_gpio_request(&flash_led->cdev);
		if (rc) {
			pr_err("%s: GPIO request failed LED blinking "
				"disabled, rc=%d\n", __func__, rc);
			return;
		}
	}

	flash_led->blinking = 1;

	while (1) {
		if (ret) {
			LED_DBG("ON\n");
			gpio_set_value_cansleep(flash_led->gpio_pin_num, 0);
			ret = 0;
			set_current_state(TASK_INTERRUPTIBLE);
			msleep(flash_led->cdev.blink_delay_on);
		} else { /* ret == 0 */
			LED_DBG("OFF\n");
			gpio_set_value_cansleep(flash_led->gpio_pin_num, 1);
			ret = 1;
			set_current_state(TASK_INTERRUPTIBLE);
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
			!flash_led->cdev.blink_delay_off ||
			!flash_led->blinking) {
			led_gpio_free(&flash_led->cdev);
			return;
		}
	}
}

/* LED blink setup */
static int led_gpio_blink_set(struct led_classdev *led_cdev,
				unsigned long *delay_on,
				unsigned long *delay_off)
{
	struct led_gpio_flash_data *flash_led =
		container_of(led_cdev, struct led_gpio_flash_data, cdev);

	flash_led->cdev.brightness = 0;
	LED_DBG("%s: Reset brightness=%d, setup flashing LED "
		"delay_on/delay_off\n",
		__func__, flash_led->cdev.brightness);
	LED_DBG("%s: delay_on=%lu, delay_off=%lu\n",
		__func__, *delay_on, *delay_off);

	LED_DBG("%s: Cancel any pending queued LED blinking work\n", __func__);
	flash_led->blinking = LED_OFF;
	cancel_work_sync(&flash_led->led_flash_work);
	LED_DBG("%s: Prepare and queue work\n", __func__);
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
	int rc;
	int brightness = value;

	/* Reset blink*/
	flash_led->cdev.blink_delay_on =
		flash_led->cdev.blink_delay_off =
		flash_led->blinking = LED_OFF;

	if (brightness >= 1) { /* Turn on LED */
		if (!flash_led->gpio_pin_state &&
			!flash_led->gpio_pin_mux_state) {
			rc = led_gpio_request(&flash_led->cdev);
			if (rc) {
				pr_err("%s: GPIO request failed LED disabled, "
					"rc=%d\n", __func__, rc);
				goto err;
			}
		}
		LED_DBG("%s: Turning on LED, brightness=%d\n",
			__func__, brightness);
		gpio_set_value_cansleep(flash_led->gpio_pin_num, 0);
		/* if 0 we failed to set */
		if (gpio_get_value_cansleep(flash_led->gpio_pin_num)) {
			pr_err("%s: Failed to set gpio %d\n", __func__,
				flash_led->gpio_pin_num);
			goto err;
		}
		LED_DBG("%s: GPIO value=%d\n", __func__,
			gpio_get_value_cansleep(flash_led->gpio_pin_num));
		flash_led->cdev.brightness = 1;
		LED_DBG("%s: Stored new value for brightness=%d\n", __func__,
			flash_led->cdev.brightness);
	} else { /* brightness <= 0, Turn off LED */
		LED_DBG("%s: Turning off LED, brightness=%d\n",
			__func__, brightness);
		gpio_set_value_cansleep(flash_led->gpio_pin_num, 1);
		/* if nonzero we failed to set */
		if (gpio_get_value_cansleep(flash_led->gpio_pin_num)) {
			pr_err("%s: Failed to set gpio %d\n", __func__,
				flash_led->gpio_pin_num);
			goto err;
		}
		LED_DBG("%s: GPIO value=%d\n", __func__,
			gpio_get_value_cansleep(flash_led->gpio_pin_num));
		if (flash_led->gpio_pin_state && flash_led->gpio_pin_mux_state)
			led_gpio_free(&flash_led->cdev);
		flash_led->cdev.brightness = 0;
		LED_DBG("%s: Stored new value for brightness=%d\n",
			__func__, flash_led->cdev.brightness);
	}

err:
	return;
}

static enum led_brightness led_gpio_brightness_get(struct led_classdev
						   *led_cdev)
{
	struct led_gpio_flash_data *flash_led =
	    container_of(led_cdev, struct led_gpio_flash_data, cdev);
	LED_DBG("%s: returning brightness=%d\n",
		__func__, flash_led->cdev.brightness);
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
	flash_led->gpio_pin_tri_state_num = GPIO_TRI_STATE_ALL;
	flash_led->gpio_pin_state = flash_led->gpio_pin_mux_state =
		flash_led->gpio_pin_tri_state_state = 0;
	flash_led->blinking = LED_OFF;
	flash_led->cdev.default_trigger = LED_TRIGGER_DEFAULT;
	dev_dbg(&pdev->dev, "%s: GPIO LED number=%d, GPIO LED pin mux number=%d, "
		"default trigger=%s\n", __func__, flash_led->gpio_pin_num,
		flash_led->gpio_pin_mux_num, flash_led->cdev.default_trigger);

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

	if (flash_led->gpio_pin_state && flash_led->gpio_pin_mux_state)
		led_gpio_free(&flash_led->cdev);

	device_remove_file(flash_led->cdev.dev, &dev_attr_delay_off);
	device_remove_file(flash_led->cdev.dev, &dev_attr_delay_on);

	led_classdev_unregister(&flash_led->cdev);
	devm_kfree(&pdev->dev, flash_led);
	LED_DBG("%s: Unregistering Intel LED GPIO driver", __func__);
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
	LED_DBG("%s: Registering Intel LED GPIO driver\n", __func__);
	return platform_driver_register(&led_gpio_flash_driver);
}

static void __exit led_gpio_flash_exit(void)
{
	LED_DBG("%s: Unregistering Intel LED GPIO driver\n", __func__);
	return platform_driver_unregister(&led_gpio_flash_driver);
}

late_initcall(led_gpio_flash_init);
module_exit(led_gpio_flash_exit);

MODULE_DESCRIPTION("Intel GPIO LEDs driver");
MODULE_LICENSE("GPL v2");

