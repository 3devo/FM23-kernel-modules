// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for custom watchdog circuit consisting of:
 *  - MIC826 (GPIO-controlled voltage supervisor/watchdog chip)
 *  - Flip flop reset by the watchdog output and set by a rising edge on
 *    the ENABLE signal
 *  - AND gate combining the flip-flop output and ENABLE signal
 *
 * Together these form a circuit with a single output that can control
 * other components (like regulator enable lines or IC reset lines).
 * This line is forced off by the watchdog whenever it expires and can
 * be controlled by software whenever the watchdog is happy.
 *
 * This was originally planned to be implemented as just
 * a gpio/reset-controller on top of an existing gpio_wdt device, but
 * I could not find an obvious way to get access to an existing wdt
 * device, so this driver just includes the threedevo_wdt functionality
 * (reduced to what it needs).
 *
 * This driver implements a watchdog driver that starts pinging the
 * watchdog as soon as it is probed, and exposes one GPIO pin that can
 * only be set to output and switched from inactive/low to active/high
 * once (and never back for now). This ensures the flipflop is switched
 * on once during startup. If the watchdog is not fed, the flipflop
 * switches off and a reboot is needed to recover.
 *
 * Author: 2024, Matthijs Kooijman <matthijs@stdin.nl>
 */

#include <linux/err.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>

// These two functions are cherry-picked from commit a481fb5065cd
// (regulator: Add devm helpers for get and enable, 2022-08-12) to
// easily migrate to devm_regulator_get_enable once it is added to our
// kernel version.
static void threedevo_wdt_regulator_action_disable(void *d)
{
       struct regulator *r = (struct regulator *)d;

       regulator_disable(r);
}

static int threedevo_wdt_devm_regulator_get_enable(struct device *dev, const char *id)
{
       struct regulator *r;
       int ret;

       r = devm_regulator_get(dev, id);
       if (IS_ERR(r))
               return PTR_ERR(r);

       ret = regulator_enable(r);
       if (!ret)
               ret = devm_add_action_or_reset(dev, &threedevo_wdt_regulator_action_disable, r);

       if (ret)
               devm_regulator_put(r);

       return ret;
}


struct threedevo_wdt_priv {
	struct watchdog_device	wdd;
	struct gpio_desc	*gpiod_wdi;
	bool			wdi_state;
	ktime_t			reset_done;
	bool			enable_state;

	struct gpio_chip gpio_chip;
	struct gpio_desc	*gpiod_enable;
};

static int threedevo_wdt_ping(struct watchdog_device *wdd)
{
	struct threedevo_wdt_priv *priv = watchdog_get_drvdata(wdd);

	priv->wdi_state = !priv->wdi_state;
	gpiod_set_value_cansleep(priv->gpiod_wdi, priv->wdi_state);
	return 0;
}

static int threedevo_wdt_start(struct watchdog_device *wdd)
{
	// Hardware is always on, so not much to do here
	return threedevo_wdt_ping(wdd);
}

static const struct watchdog_ops threedevo_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= threedevo_wdt_start,
	.ping		= threedevo_wdt_ping,
};

static const struct watchdog_info threedevo_wdt_ident = {
	.options	= WDIOF_KEEPALIVEPING,
	.identity	= "3devo watchdog",
};

static void threedevo_wdt_setup_wdt(struct device *dev, struct threedevo_wdt_priv *priv, unsigned int hw_margin)
{
	struct watchdog_device *wdd;

	wdd = &priv->wdd;

	wdd->info = &threedevo_wdt_ident;
	wdd->ops = &threedevo_wdt_ops;
	// Setting timeout to equal or below tomax_hw_heartbeat_ms and
	// not allowing timeout changes (by not setting
	// WDIOF_SETTIMEOUT) ensures that userspace cannot request an
	// unreasonably large timeout which could cause the kernel to
	// generate intermediate pings. This ensures that all hardware
	// pings are directly caused by userspace pings (except for
	// during system startup).
	// However, we calso cannot have a zero timeout, since then the
	// kernel will not service the watchdog before userspace has
	// taken over.
	wdd->max_hw_heartbeat_ms = hw_margin;
	wdd->timeout = hw_margin / 1000;
	WARN_ON(wdd->timeout == 0);
	wdd->parent = dev;

	watchdog_set_drvdata(wdd, priv);
	watchdog_set_nowayout(wdd, 1);
}


static int threedevo_wdt_gpio_get_value(struct gpio_chip *gc, unsigned off)
{
	struct threedevo_wdt_priv *priv = gpiochip_get_data(gc);
	WARN_ON(off != 0);

	return gpiod_get_value_cansleep(priv->gpiod_enable);
}

static void threedevo_wdt_gpio_set_value(struct gpio_chip *gc, unsigned off, int val)
{
	struct threedevo_wdt_priv *priv = gpiochip_get_data(gc);
	ktime_t now;
	WARN_ON(off != 0);

	dev_dbg(gc->parent, "set_value offset %u val %d", off, val);

	if (val == 0) {
		if (priv->enable_state)
			dev_err(gc->parent, "Enable pin cannot be disabled after enabling, ignoring set operation\n");
		return;
	}

	// If the watchdog might still be outputting a reset pulse from
	// before our first ping, wait for that to complete.
	now = ktime_get();
	if (ktime_before(now, priv->reset_done))
		msleep(ktime_to_ms(ktime_sub(priv->reset_done, now)));

	gpiod_set_value_cansleep(priv->gpiod_enable, val);
	priv->enable_state = true;
}

static int threedevo_wdt_gpio_direction_output(struct gpio_chip *gc, unsigned off, int val)
{
	struct threedevo_wdt_priv *priv = gpiochip_get_data(gc);
	WARN_ON(off != 0);

	dev_dbg(gc->parent, "direction_output offset %u val %d", off, val);

	if (val == 0 && priv->enable_state) {
		dev_err(gc->parent, "Enable pin cannot be disabled after enabling, ignoring direction operation\n");
		return -EINVAL;
	}

	// Just set the value, the pin was already set to output during probe
	threedevo_wdt_gpio_set_value(gc, off, val);

	return 0;
}

static int threedevo_wdt_gpio_get_direction(struct gpio_chip *gc, unsigned off)
{
	WARN_ON(off != 0);
	return GPIO_LINE_DIRECTION_OUT;
}

static void threedevo_wdt_setup_gpio(struct device *dev, struct threedevo_wdt_priv *priv)
{
	struct gpio_chip *gc;

	gc = &priv->gpio_chip;

	gc->get = threedevo_wdt_gpio_get_value;
	gc->set = threedevo_wdt_gpio_set_value;
	gc->direction_output = threedevo_wdt_gpio_direction_output;
	gc->get_direction = threedevo_wdt_gpio_get_direction;
	gc->can_sleep = true;

	gc->base = -1;
	gc->ngpio = 1;
	gc->label = dev_name(dev);
	gc->parent = dev;
	gc->owner = THIS_MODULE;
}

static int threedevo_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct threedevo_wdt_priv *priv;
	unsigned int hw_margin, reset_pulse;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);

	priv->gpiod_wdi = devm_gpiod_get(dev, "wdi", GPIOD_OUT_LOW);
	if (IS_ERR(priv->gpiod_wdi))
		return dev_err_probe(dev, PTR_ERR(priv->gpiod_wdi), "failed to get wdi pin\n");

	// Setting the enable pin to LOW now ensures that there will be
	// a rising edge when it is first enabled later
	priv->gpiod_enable = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(priv->gpiod_enable))
		return dev_err_probe(dev, PTR_ERR(priv->gpiod_enable), "failed to get enable pin\n");

	ret = of_property_read_u32(np, "hw_margin_ms", &hw_margin);
	if (ret)
		return dev_err_probe(dev, ret, "Missing hw_margin_ms\n");
	// See comment in threedevo_wdt_setup_wdt for why this must be min 1000ms
	if (hw_margin < 1000 || hw_margin > 65535)
		return dev_err_probe(dev, -EINVAL, "Invalid hw_margin_ms\n");

	// TODO: use regular version once kernel is upgraded
	ret = threedevo_wdt_devm_regulator_get_enable(dev, "vcc");
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulator\n");

	ret = of_property_read_u32(np, "reset_pulse_ms", &reset_pulse);
	if (ret)
		return dev_err_probe(dev, ret, "Missing reset_pulse_ms\n");

	// There is a pull resistor on WDI to ensure it is always
	// enabled (never floating), so set this bit before registration
	// so watchdog_dev.c will (when configured) start pinging the
	// watchdog until userspace takes over.
	set_bit(WDOG_HW_RUNNING, &priv->wdd.status);

	threedevo_wdt_setup_wdt(dev, priv, hw_margin);

	ret = threedevo_wdt_ping(&priv->wdd);
	if (ret)
		return ret;

	// Worst case, the watchdog has been running for a while and has
	// just started a reset pulse which it has to finish before we
	// can set the flipflop. Since we did the first ping above, this
	// should be the last reset pulse (as long as the watchdog is
	// kept happy).
	priv->reset_done = ktime_add_ms(ktime_get(), reset_pulse);

	ret = devm_watchdog_register_device(dev, &priv->wdd);
	if (ret)
		return ret;

	threedevo_wdt_setup_gpio(dev, priv);
	ret = devm_gpiochip_add_data(dev, &priv->gpio_chip, priv);
	if (ret)
		return ret;

	return 0;
}

static const struct of_device_id threedevo_wdt_dt_ids[] = {
	{ .compatible = "linux,3devo-wdt", },
	{ }
};
MODULE_DEVICE_TABLE(of, threedevo_wdt_dt_ids);

static struct platform_driver threedevo_wdt_driver = {
	.driver	= {
		.name		= "3devo-wdt",
		.of_match_table	= threedevo_wdt_dt_ids,
	},
	.probe	= threedevo_wdt_probe,
};

module_platform_driver(threedevo_wdt_driver);

MODULE_AUTHOR("Matthijs Kooijman <matthijs@stdin.nl>");
MODULE_DESCRIPTION("3devo watchdog");
MODULE_LICENSE("GPL");
