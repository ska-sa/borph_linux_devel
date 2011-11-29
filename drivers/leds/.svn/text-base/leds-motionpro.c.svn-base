/*
 * LEDs driver for the Motion-PRO board.
 *
 * Copyright (C) 2007 Semihalf
 * Jan Wrobel <wrr@semihalf.com>
 * Marian Balakowicz <m8@semihalf.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *
 * Decription:
 * This driver enables control over Motion-PRO status and ready LEDs through
 * sysfs. LEDs can be controlled by writing to sysfs files:
 * class/leds/<led-name>/(brightness|delay_off|delay_on).
 * See Documentation/leds-class.txt for more details.
 * <led-name> is the set to the value of 'label' property of the
 * corresponding GPT node.
 *
 * Before user issues first control command via sysfs, LED blinking is
 * controlled by the kernel ('blink-delay' property of the GPT node
 * in the device tree blob).
 *
 */

#define DEBUG

#include <linux/module.h>
#include <linux/leds.h>
#include <linux/of_platform.h>
#include <asm/mpc52xx.h>

/* LED control bits */
#define LED_ON	MPC52xx_GPT_OUTPUT_1

/* LED mode */
#define LED_MODE_KERNEL		1
#define LED_MODE_USER		2

struct motionpro_led {
	spinlock_t led_lock;		/* Protects the LED data */
	struct mpc52xx_gpt __iomem *gpt;/* LED registers */
	struct timer_list blink_timer;	/* Used if blink_delay is nonzero */
	unsigned int blink_delay;	/* [ms], if set to 0 blinking is off */
	unsigned int mode;		/* kernel/user */
	struct led_classdev mpled_cdev;	/* LED class */
};

/*
 * Timer event - blinks LED before user takes control over it
 * with the first access via sysfs.
 */
static void mpled_timer_toggle(unsigned long data)
{
	struct motionpro_led *mpled = (struct motionpro_led *)data;

	spin_lock_bh(&mpled->led_lock);
	if (mpled->mode == LED_MODE_KERNEL) {
		u32 val = in_be32(&mpled->gpt->mode);
		val ^= LED_ON;
		out_be32(&mpled->gpt->mode, val);

		mod_timer(&mpled->blink_timer,
			jiffies + msecs_to_jiffies(mpled->blink_delay));
	}
	spin_unlock_bh(&mpled->led_lock);
}

/*
 * Turn on/off led according to user settings in sysfs.
 * First call to this function disables kernel blinking.
 */
static void mpled_set(struct led_classdev *led_cdev,
		      enum led_brightness brightness)
{
	struct motionpro_led *mpled;
	int old_mode;
	u32 val;

	mpled = container_of(led_cdev, struct motionpro_led, mpled_cdev);

	spin_lock_bh(&mpled->led_lock);
	/* disable kernel controll */
	old_mode = mpled->mode;
	if (old_mode == LED_MODE_KERNEL)
		mpled->mode = LED_MODE_USER;

	val = in_be32(&mpled->gpt->mode);
	if (brightness)
		val |= LED_ON;
	else
		val &= ~LED_ON;
	out_be32(&mpled->gpt->mode, val);
	spin_unlock_bh(&mpled->led_lock);

	/* delete kernel mode blink timer, not needed anymore */
	if ((old_mode == LED_MODE_KERNEL) && mpled->blink_delay)
		del_timer(&mpled->blink_timer);
}

static void mpled_init_led(void __iomem *gpt_mode)
{
	u32 val = in_be32(gpt_mode);
	val |= MPC52xx_GPT_ENABLE_OUTPUT;
	val &= ~LED_ON;
	out_be32(gpt_mode, val);
}

static int __devinit mpled_probe(struct of_device *op,
				 const struct of_device_id *match)
{
	struct motionpro_led *mpled;
	const unsigned int *of_blink_delay;
	const char *label;
	int err;

	dev_dbg(&op->dev, "mpled_probe: node=%s (op=%p, match=%p)\n",
		op->node->name, op, match);

	mpled = kzalloc(sizeof(*mpled), GFP_KERNEL);
	if (!mpled)
		return -ENOMEM;

	mpled->gpt = of_iomap(op->node, 0);
	if (!mpled->gpt) {
		printk(KERN_ERR __FILE__ ": "
			"Error mapping GPT registers for LED %s\n",
			op->node->full_name);
		err = -EIO;
		goto err_free;
	}

	/* initialize GPT for LED use */
	mpled_init_led(&mpled->gpt->mode);

	spin_lock_init(&mpled->led_lock);
	mpled->mode = LED_MODE_KERNEL;

	/* get LED label, used to register led classdev */
	label = of_get_property(op->node, "label", NULL);
	if (label == NULL) {
		printk(KERN_ERR __FILE__ ": "
			"No label property provided for LED %s\n",
			op->node->full_name);
		err = -EINVAL;
		goto err;
	}
	dev_dbg(&op->dev, "mpled_probe: label = '%s'\n", label);

	/* get 'blink-delay' property if present */
	of_blink_delay = of_get_property(op->node, "blink-delay", NULL);
	mpled->blink_delay =  of_blink_delay ? *of_blink_delay : 0;
	dev_dbg(&op->dev, "mpled_probe: blink_delay = %d msec\n",
		mpled->blink_delay);

	/* initialize kernel blink_timer if blink_delay was provided */
	if (mpled->blink_delay) {
		init_timer(&mpled->blink_timer);
		mpled->blink_timer.function = mpled_timer_toggle;
		mpled->blink_timer.data = (unsigned long)mpled;

		mod_timer(&mpled->blink_timer,
			jiffies + msecs_to_jiffies(mpled->blink_delay));
	}

	/* register LED classdev */
	mpled->mpled_cdev.name = label;
	mpled->mpled_cdev.brightness_set = mpled_set;
	mpled->mpled_cdev.default_trigger = "timer";

	err = led_classdev_register(NULL, &mpled->mpled_cdev);
	if (err) {
		printk(KERN_ERR __FILE__ ": "
			"Error registering class device for LED %s\n",
			op->node->full_name);
		goto err;
	}

	dev_set_drvdata(&op->dev, mpled);
	return 0;

err:
	if (mpled->blink_delay)
		del_timer(&mpled->blink_timer);
	iounmap(mpled->gpt);
err_free:
	kfree(mpled);

	return err;
}

static int mpled_remove(struct of_device *op)
{
	struct motionpro_led *mpled = dev_get_drvdata(&op->dev);

	dev_dbg(&op->dev, "mpled_remove: (%p)\n", op);

	if (mpled->blink_delay && (mpled->mode == LED_MODE_KERNEL))
		del_timer(&mpled->blink_timer);

	led_classdev_unregister(&mpled->mpled_cdev);

	iounmap(mpled->gpt);
	kfree(mpled);

	return 0;
}

static const struct of_device_id mpled_match[] = {
	{ .compatible = "promess,motionpro-led", },
	{},
};

static struct of_platform_driver mpled_driver = {
	.match_table	= mpled_match,
	.probe		= mpled_probe,
	.remove		= mpled_remove,
	.driver		= {
		.owner		= THIS_MODULE,
		.name		= "leds-motionpro",
	},
};

static int __init mpled_init(void)
{
	return of_register_platform_driver(&mpled_driver);
}

static void __exit mpled_exit(void)
{
	of_unregister_platform_driver(&mpled_driver);
}

module_init(mpled_init);
module_exit(mpled_exit);

MODULE_LICENSE("GPL")
MODULE_DESCRIPTION("Motion-PRO LED driver");
MODULE_AUTHOR("Jan Wrobel <wrr@semihalf.com>");
MODULE_AUTHOR("Marian Balakowicz <m8@semihalf.com>");
